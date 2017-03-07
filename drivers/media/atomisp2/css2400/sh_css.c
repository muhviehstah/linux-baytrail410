/*
 * Support for Intel Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 - 2014 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

/*! \file */
//#include "stdio.h"
#include "ia_css.h"
#include "sh_css_hrt.h"		/* only for file 2 MIPI */
#include "ia_css_binary.h"
#include "sh_css_internal.h"
#include "sh_css_sp.h"		/* sh_css_sp_group */
#if !defined(HAS_NO_INPUT_SYSTEM)
#include "ia_css_isys.h"
#endif
#include "ia_css_frame.h"
#include "sh_css_defs.h"
#include "sh_css_firmware.h"
#include "sh_css_params.h"
#include "sh_css_params_internal.h"
#include "sh_css_param_shading.h"
#include "ia_css_refcount.h"
#include "ia_css_rmgr.h"
#include "ia_css_debug.h"
#include "ia_css_debug_pipe.h"
#include "ia_css_memory_access.h"
#include "ia_css_device_access.h"
#include "sh_css_legacy.h"
#include "ia_css_pipeline.h"
#include "ia_css_stream.h"
#include "ia_css_pipe.h"
#include "ia_css_util.h"
#include "ia_css_pipe_util.h"
#include "ia_css_pipe_binarydesc.h"
#include "ia_css_pipe_stagedesc.h"
//#include "ia_css_stream_manager.h"
/* #include "ia_css_rmgr_gen.h" */

#include "memory_access.h"
#include "tag.h"
#include "assert_support.h"
#include "math_support.h"
#include "sw_event_global.h"			/* Event IDs.*/
#if !defined(HAS_NO_INPUT_FORMATTER)
#include "ia_css_ifmtr.h"
#endif
#if !defined(HAS_NO_INPUT_SYSTEM)
#include "input_system.h"
#endif
#include "mmu_device.h"		/* mmu_set_page_table_base_index(), ... */
#include "gdc_device.h"		/* HRT_GDC_N */
#include "irq.h"			/* virq */
#include "sp.h"				/* cnd_sp_irq_enable() */
#include "isp.h"			/* cnd_isp_irq_enable, ISP_VEC_NELEMS */
#include "gp_device.h"		/* gp_device_reg_store() */
#if !defined(HAS_NO_GPIO)
#define __INLINE_GPIO__
#include "gpio.h"
#endif
#include "timed_ctrl.h"
#include "platform_support.h" /* hrt_sleep() */
#include "ia_css_inputfifo.h"
#define WITH_PC_MONITORING  0

#define SH_CSS_VIDEO_BUFFER_ALIGNMENT 0

#if WITH_PC_MONITORING
#define MULTIPLE_SAMPLES 1
#define NOF_SAMPLES      60
#include "linux/kthread.h"
#include "linux/sched.h"
#include "linux/delay.h"
#include "sh_css_metrics.h"
static int thread_alive;
#endif /* WITH_PC_MONITORING */

#define DVS_REF_TESTING 0
#if DVS_REF_TESTING
#include <stdio.h>
#endif
#include"ia_css_spctrl.h"
#include "ia_css_version_data.h"
#include "sh_css_struct.h"
#include "ia_css_bufq.h"

/* Name of the sp program: should not be built-in */
#define SP_PROG_NAME "sp"
#if defined(IS_ISP_2500_SYSTEM)
#define SP1_PROG_NAME "sp1"
#endif
/* Size of Refcount List */
#define REFCOUNT_SIZE 1000

/* for JPEG, we don't know the length of the image upfront,
 * but since we support sensor upto 16MP, we take this as
 * upper limit.
 */
#define JPEG_BYTES (16 * 1024 * 1024)

#if defined(IS_ISP_2500_SYSTEM)
#define STATS_ENABLED(stage) (stage && stage->binary && stage->binary->info && \
	(stage->binary->info->sp.enable.s3a || stage->binary->info->sp.enable.dvs_stats || \
	stage->binary->info->sp.enable.lace_stats))
#else
#define STATS_ENABLED(stage) (stage && stage->binary && stage->binary->info && \
        (stage->binary->info->sp.enable.s3a || stage->binary->info->sp.enable.dis))
#endif

#define PIPE_ENTRY_EMPTY_TOKEN                (~0)
#define PIPE_ENTRY_RESERVED_TOKEN             (0x1)


#define DEFAULT_PLANES { {0, 0, 0, 0} }

struct sh_css my_css = {
	.mipi_frames = {NULL, NULL, NULL, NULL}
};

int (*sh_css_printf) (const char *fmt, va_list args) = NULL;

/* modes of work: stream_create and stream_destroy will update the save/restore data
   only when in working mode, not suspend/resume
*/
enum ia_sh_css_modes {
	sh_css_mode_none = 0,
	sh_css_mode_working,
	sh_css_mode_suspend,
	sh_css_mode_resume
};

#define SEED_NUM_PIPES	3

/* a stream seed, to save and restore the stream data.
   the stream seed contains all the data required to "grow" the seed again after it was closed.
*/
struct sh_css_stream_seed {
	struct ia_css_stream 		**orig_stream;                /* pointer to restore the original handle */
	struct ia_css_stream 		*stream;                      /* handle, used as ID too.*/
	struct ia_css_stream_config 	stream_config;
	int 				num_pipes;
	struct ia_css_pipe 		*pipes[SEED_NUM_PIPES];
	struct ia_css_pipe_config 	pipe_config[SEED_NUM_PIPES];
};

#define MAX_ACTIVE_STREAMS	5
/* A global struct for save/restore to hold all the data that should sustain power-down:
   MMU base, IRQ type, env for routines, binary loaded FW and the stream seeds.
*/
struct sh_css_save {
	enum ia_sh_css_modes		mode;
	uint32_t 		       mmu_base;				/* the last mmu_base */
	enum ia_css_irq_type           irq_type;
	struct sh_css_stream_seed      stream_seeds[MAX_ACTIVE_STREAMS];
	struct ia_css_fw 	       *loaded_fw;				/* fw struct previously loaded */
	struct ia_css_env 	       *driver_env;				/* driver-supplied env */
};

static bool my_css_save_initialized = false;		// if my_css_save was initialized
static struct sh_css_save my_css_save;

/* pqiao NOTICE: this is for css internal buffer recycling when stopping pipeline,
   this array is temporary and will be replaced by resource manager*/
/* Taking the biggest Size for number of Elements */
#define MAX_HMM_BUFFER_NUM	\
	(SH_CSS_MAX_NUM_QUEUES * (IA_CSS_NUM_ELEMS_SP2HOST_BUFFER_QUEUE + 2))

static struct ia_css_rmgr_vbuf_handle *hmm_buffer_record_h[MAX_HMM_BUFFER_NUM];

#if defined(USE_INPUT_SYSTEM_VERSION_2) || defined(USE_INPUT_SYSTEM_VERSION_2401)
static uint32_t ref_count_mipi_allocation = 0;
#endif

#define GPIO_FLASH_PIN_MASK (1 << HIVE_GPIO_STROBE_TRIGGER_PIN)

static bool fw_explicitly_loaded = false;

/**
 * Local prototypes
 */
static enum ia_css_err
sh_css_pipe_start(struct ia_css_stream *stream);

static enum ia_css_err
ia_css_pipe_load_extension(struct ia_css_pipe *pipe,
			   struct ia_css_fw_info *firmware);

static void
ia_css_pipe_unload_extension(struct ia_css_pipe *pipe,
			     struct ia_css_fw_info *firmware);
static void
ia_css_reset_defaults(struct sh_css* css);

static void
sh_css_init_host_sp_control_vars(void);

static void
sh_css_mmu_set_page_table_base_index(hrt_data base_index);

static bool
need_capture_pp(const struct ia_css_pipe *pipe);

static bool
need_yuv_scaler(const struct ia_css_pipe *pipe);

static enum ia_css_err
sh_css_pipe_load_binaries(struct ia_css_pipe *pipe);

static
enum ia_css_err sh_css_pipe_get_viewfinder_frame_info(
	struct ia_css_pipe *pipe,
	struct ia_css_frame_info *info,
	unsigned int idx);

static enum ia_css_err
sh_css_pipe_get_output_frame_info(struct ia_css_pipe *pipe,
				  struct ia_css_frame_info *info,
				  unsigned int idx);

static enum ia_css_err
capture_start(struct ia_css_pipe *pipe);

static enum ia_css_err
video_start(struct ia_css_pipe *pipe);

static enum ia_css_err
preview_start(struct ia_css_pipe *pipe);

static enum ia_css_err
yuvpp_start(struct ia_css_pipe *pipe);

static bool copy_on_sp(struct ia_css_pipe *pipe);

static enum ia_css_err
init_vf_frameinfo_defaults(struct ia_css_pipe *pipe,
	struct ia_css_frame *vf_frame, unsigned int idx);

static enum ia_css_err
init_in_frameinfo_memory_defaults(struct ia_css_pipe *pipe,
	struct ia_css_frame *frame);

static enum ia_css_err
init_out_frameinfo_defaults(struct ia_css_pipe *pipe,
	struct ia_css_frame *out_frame, unsigned int idx);

static enum ia_css_err
sh_css_pipeline_add_acc_stage(struct ia_css_pipeline *pipeline,
			      const void *acc_fw);

static enum ia_css_err
alloc_continuous_frames(
	struct ia_css_pipe *pipe, bool init_time);

#if defined(USE_INPUT_SYSTEM_VERSION_2) || defined(USE_INPUT_SYSTEM_VERSION_2401)
static enum ia_css_err
allocate_mipi_frames(struct ia_css_pipe *pipe);

static void
free_mipi_frames(struct ia_css_pipe *pipe, bool uninit);
#endif

static void
pipe_global_init(void);

static uint8_t
pipe_generate_pipe_num(const struct ia_css_pipe *pipe);

static void
pipe_release_pipe_num(unsigned int pipe_num);

static enum ia_css_err
create_host_pipeline_structure(struct ia_css_stream *stream);

static enum ia_css_err
create_host_pipeline(struct ia_css_stream *stream);

static enum ia_css_err
create_host_preview_pipeline(struct ia_css_pipe *pipe);

static enum ia_css_err
create_host_video_pipeline(struct ia_css_pipe *pipe);

static enum ia_css_err
create_host_copy_pipeline(struct ia_css_pipe *pipe,
    unsigned max_input_width,
    struct ia_css_frame *out_frame);

static enum ia_css_err
create_host_isyscopy_capture_pipeline(struct ia_css_pipe *pipe);

static enum ia_css_err
create_host_capture_pipeline(struct ia_css_pipe *pipe);

static enum ia_css_err
create_host_yuvpp_pipeline(struct ia_css_pipe *pipe);

static enum ia_css_err
create_host_acc_pipeline(struct ia_css_pipe *pipe);

static unsigned int
sh_css_get_sw_interrupt_value(unsigned int irq);

#if 0
static enum ia_css_err
sh_css_pipeline_stop(struct ia_css_pipe *pipe);
#endif

static struct ia_css_binary *
ia_css_pipe_get_3a_binary (const struct ia_css_pipe *pipe);

#if CONFIG_ON_FRAME_ENQUEUE()
static enum ia_css_err set_config_on_frame_enqueue(struct ia_css_frame_info *info, struct frame_data_wrapper *frame);
#endif

static void
sh_css_pipe_free_shading_table(struct ia_css_pipe *pipe)
{
	assert(pipe != NULL);

	if (pipe->shading_table)
		ia_css_shading_table_free(pipe->shading_table);
	pipe->shading_table = NULL;
}

static enum ia_css_frame_format yuv420_copy_formats[] = {
	IA_CSS_FRAME_FORMAT_NV12,
	IA_CSS_FRAME_FORMAT_NV21,
	IA_CSS_FRAME_FORMAT_YV12,
	IA_CSS_FRAME_FORMAT_YUV420,
	IA_CSS_FRAME_FORMAT_YUV420_16,
	IA_CSS_FRAME_FORMAT_CSI_MIPI_YUV420_8
};

static enum ia_css_frame_format yuv422_copy_formats[] = {
	IA_CSS_FRAME_FORMAT_NV12,
	IA_CSS_FRAME_FORMAT_NV16,
	IA_CSS_FRAME_FORMAT_NV21,
	IA_CSS_FRAME_FORMAT_NV61,
	IA_CSS_FRAME_FORMAT_YV12,
	IA_CSS_FRAME_FORMAT_YV16,
	IA_CSS_FRAME_FORMAT_YUV420,
	IA_CSS_FRAME_FORMAT_YUV420_16,
	IA_CSS_FRAME_FORMAT_YUV422,
	IA_CSS_FRAME_FORMAT_YUV422_16,
	IA_CSS_FRAME_FORMAT_UYVY,
	IA_CSS_FRAME_FORMAT_YUYV,
	IA_CSS_FRAME_FORMAT_RGB565,
	IA_CSS_FRAME_FORMAT_RGBA888
};

#define array_length(array) (sizeof(array)/sizeof(array[0]))


/* Retrieve the CSS version and try to retrieve the FW version too.
 */
enum ia_css_err
ia_css_get_version(char *version, int max_size)
{
	if (max_size <= (int)strlen(CSS_VERSION_STRING) + (int)strlen(sh_css_get_fw_version()) + 5)
		return(IA_CSS_ERR_INVALID_ARGUMENTS);
	assert(version != NULL);
	strcpy(version, CSS_VERSION_STRING);
	strcat(version, "FW:");
	strcat(version, sh_css_get_fw_version());
	strcat(version, "; ");
	return(IA_CSS_SUCCESS);
}


/* Verify whether the selected output format is can be produced
 * by the copy binary given the stream format.
 * */
static enum ia_css_err
verify_copy_out_frame_format(struct ia_css_pipe *pipe)
{
	enum ia_css_frame_format out_fmt = pipe->output_info[0].format;
	unsigned int i, found = 0;

	assert(pipe != NULL);
	assert(pipe->stream != NULL);

	switch (pipe->stream->config.format) {
	case IA_CSS_STREAM_FORMAT_YUV420_8_LEGACY:
	case IA_CSS_STREAM_FORMAT_YUV420_8:
		for (i=0; i<array_length(yuv420_copy_formats) && !found; i++)
			found = (out_fmt == yuv420_copy_formats[i]);
		break;
	case IA_CSS_STREAM_FORMAT_YUV420_10:
		found = (out_fmt == IA_CSS_FRAME_FORMAT_YUV420_16);
		break;
	case IA_CSS_STREAM_FORMAT_YUV422_8:
		for (i=0; i<array_length(yuv422_copy_formats) && !found; i++)
			found = (out_fmt == yuv422_copy_formats[i]);
		break;
	case IA_CSS_STREAM_FORMAT_YUV422_10:
		found = (out_fmt == IA_CSS_FRAME_FORMAT_YUV422_16 ||
			 out_fmt == IA_CSS_FRAME_FORMAT_YUV420_16);
		break;
	case IA_CSS_STREAM_FORMAT_RGB_444:
	case IA_CSS_STREAM_FORMAT_RGB_555:
	case IA_CSS_STREAM_FORMAT_RGB_565:
		found = (out_fmt == IA_CSS_FRAME_FORMAT_RGBA888 ||
			 out_fmt == IA_CSS_FRAME_FORMAT_RGB565);
		break;
	case IA_CSS_STREAM_FORMAT_RGB_666:
	case IA_CSS_STREAM_FORMAT_RGB_888:
		found = (out_fmt == IA_CSS_FRAME_FORMAT_RGBA888);
		break;
	case IA_CSS_STREAM_FORMAT_RAW_6:
	case IA_CSS_STREAM_FORMAT_RAW_7:
	case IA_CSS_STREAM_FORMAT_RAW_8:
	case IA_CSS_STREAM_FORMAT_RAW_10:
	case IA_CSS_STREAM_FORMAT_RAW_12:
	case IA_CSS_STREAM_FORMAT_RAW_14:
	case IA_CSS_STREAM_FORMAT_RAW_16:
		found = (out_fmt == IA_CSS_FRAME_FORMAT_RAW) ||
			(out_fmt == IA_CSS_FRAME_FORMAT_RAW_PACKED);
		break;
	case IA_CSS_STREAM_FORMAT_BINARY_8:
		found = (out_fmt == IA_CSS_FRAME_FORMAT_BINARY_8);
		break;
	default:
		break;
	}
	if (!found)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	return IA_CSS_SUCCESS;
}

/* next function takes care of getting the settings from kernel
 * commited to hmm / isp
 * TODO: see if needs to be made public
 */
static enum ia_css_err
sh_css_commit_isp_config(struct ia_css_stream *stream,
			 struct ia_css_pipeline *pipeline)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct ia_css_pipeline_stage *stage;

	if (pipeline) {
		/* walk through pipeline and commit settings */
		/* TODO: check if this is needed (s3a is handled through this */
		for (stage = pipeline->stages; stage; stage = stage->next) {
			if (stage && stage->binary) {
				err = sh_css_params_write_to_ddr(stream,
								 stage);
				if (err != IA_CSS_SUCCESS)
					return err;
			}
		}
	}
	return err;
}

static void
configure_pipe_inout_port(struct ia_css_pipeline *me, bool continuous)
{
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"configure_pipe_inout_port() enter: pipe_id(%d) continuous(%d)\n",
			me->pipe_id, continuous);
	switch(me->pipe_id) {
		case IA_CSS_PIPE_ID_PREVIEW:
		case IA_CSS_PIPE_ID_VIDEO:
			SH_CSS_PIPE_PORT_CONFIG_SET(me->inout_port_config,
						   (uint8_t)SH_CSS_PORT_INPUT,
						   (uint8_t)(continuous ? SH_CSS_COPYSINK_TYPE : SH_CSS_HOST_TYPE),1);
			SH_CSS_PIPE_PORT_CONFIG_SET(me->inout_port_config,
						   (uint8_t)SH_CSS_PORT_OUTPUT,
						   (uint8_t)SH_CSS_HOST_TYPE,1);
			break;
		case IA_CSS_PIPE_ID_COPY: /*Copy pipe ports configured to "offline" mode*/
			SH_CSS_PIPE_PORT_CONFIG_SET(me->inout_port_config,
						   (uint8_t)SH_CSS_PORT_INPUT,
						   (uint8_t)SH_CSS_HOST_TYPE,1);
			if (continuous) {
				SH_CSS_PIPE_PORT_CONFIG_SET(me->inout_port_config,
						   (uint8_t)SH_CSS_PORT_OUTPUT,
						   (uint8_t)SH_CSS_COPYSINK_TYPE,1);
				SH_CSS_PIPE_PORT_CONFIG_SET(me->inout_port_config,
						   (uint8_t)SH_CSS_PORT_OUTPUT,
						   (uint8_t)SH_CSS_TAGGERSINK_TYPE,1);
			} else {
				SH_CSS_PIPE_PORT_CONFIG_SET(me->inout_port_config,
						   (uint8_t)SH_CSS_PORT_OUTPUT,
						   (uint8_t)SH_CSS_HOST_TYPE,1);
			}
			break;
		case IA_CSS_PIPE_ID_CAPTURE:
			SH_CSS_PIPE_PORT_CONFIG_SET(me->inout_port_config,
						   (uint8_t)SH_CSS_PORT_INPUT,
						   (uint8_t)(continuous ? SH_CSS_TAGGERSINK_TYPE : SH_CSS_HOST_TYPE),1);
			SH_CSS_PIPE_PORT_CONFIG_SET(me->inout_port_config,
						   (uint8_t)SH_CSS_PORT_OUTPUT,
						   (uint8_t)SH_CSS_HOST_TYPE,1);
			break;
		case IA_CSS_PIPE_ID_YUVPP:
			SH_CSS_PIPE_PORT_CONFIG_SET(me->inout_port_config,
						   (uint8_t)SH_CSS_PORT_INPUT,
						   (uint8_t)(SH_CSS_HOST_TYPE),1);
			SH_CSS_PIPE_PORT_CONFIG_SET(me->inout_port_config,
						   (uint8_t)SH_CSS_PORT_OUTPUT,
						   (uint8_t)SH_CSS_HOST_TYPE,1);
			break;
		case IA_CSS_PIPE_ID_ACC:
			SH_CSS_PIPE_PORT_CONFIG_SET(me->inout_port_config,
						   (uint8_t)SH_CSS_PORT_INPUT,
						   (uint8_t)SH_CSS_HOST_TYPE,1);
			SH_CSS_PIPE_PORT_CONFIG_SET(me->inout_port_config,
						   (uint8_t)SH_CSS_PORT_OUTPUT,
						   (uint8_t)SH_CSS_HOST_TYPE,1);
			break;
		default:
			break;
	}
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"configure_pipe_inout_port() leave: inout_port_config(%x)\n",
		me->inout_port_config);
}

unsigned int
ia_css_stream_input_format_bits_per_pixel(struct ia_css_stream *stream)
{
	int bpp = 0;

	if (stream != NULL)
		bpp = ia_css_util_input_format_bpp(stream->config.format,
						stream->config.pixels_per_clock == 2);

	return bpp;
}

#if !defined(HAS_NO_INPUT_SYSTEM) && defined(USE_INPUT_SYSTEM_VERSION_2)
static enum ia_css_err
sh_css_config_input_network(struct ia_css_pipe *pipe,
			    struct ia_css_binary *binary)
{
	unsigned int fmt_type;
	enum ia_css_err err = IA_CSS_SUCCESS;

	assert(pipe != NULL);
	assert(pipe->stream != NULL);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"sh_css_config_input_network() enter:\n");

	if (pipe->pipeline.stages)
		binary = pipe->pipeline.stages->binary;

	err = ia_css_isys_convert_stream_format_to_mipi_format(
				pipe->stream->config.format,
				pipe->stream->csi_rx_config.comp,
				&fmt_type);
	if (err != IA_CSS_SUCCESS)
		return err;
	sh_css_sp_program_input_circuit(fmt_type,
					pipe->stream->config.channel_id,
					pipe->stream->config.mode);

	if (binary && (binary->online || pipe->stream->config.continuous)) {
		err = ia_css_ifmtr_configure(&pipe->stream->config,
			binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	}

	if (pipe->stream->config.mode == IA_CSS_INPUT_MODE_TPG ||
	    pipe->stream->config.mode == IA_CSS_INPUT_MODE_PRBS) {
		unsigned int hblank_cycles = 100,
			     vblank_lines = 6,
			     width,
			     height,
			     vblank_cycles;
		width  = (pipe->stream->config.input_res.width) / (1 + (pipe->stream->config.pixels_per_clock == 2));
		height = pipe->stream->config.input_res.height;
		vblank_cycles = vblank_lines * (width + hblank_cycles);
		sh_css_sp_configure_sync_gen(width, height, hblank_cycles,
					     vblank_cycles);
#if defined(IS_ISP_2400_SYSTEM)
		if (pipe->stream->config.mode == IA_CSS_INPUT_MODE_TPG) {
			/* TODO: move define to proper file in tools */
			#define GP_ISEL_TPG_MODE 0x90058
			ia_css_device_store_uint32(GP_ISEL_TPG_MODE, 0);
		}
#endif
	}
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"sh_css_config_input_network() leave:\n");
	return IA_CSS_SUCCESS;
}
#elif !defined(HAS_NO_INPUT_SYSTEM) && defined(USE_INPUT_SYSTEM_VERSION_2401)
static mipi_predictor_t sh_css_csi2_compression_type_2_mipi_predictor(enum ia_css_csi2_compression_type type)
{
	mipi_predictor_t predictor = MIPI_PREDICTOR_NONE;

	switch (type) {
	case IA_CSS_CSI2_COMPRESSION_TYPE_NONE:
		predictor = MIPI_PREDICTOR_NONE;
		break;
	case IA_CSS_CSI2_COMPRESSION_TYPE_1:
		predictor = MIPI_PREDICTOR_TYPE1;
		break;
	case IA_CSS_CSI2_COMPRESSION_TYPE_2:
		predictor = MIPI_PREDICTOR_TYPE2;
	default:
		break;
	}
	return predictor;
}

static unsigned int sh_css_stream_format_2_bits_per_subpixel(
		enum ia_css_stream_format format)
{
	unsigned int rval;

	switch (format) {
	case IA_CSS_STREAM_FORMAT_RGB_444:
		rval = 4;
		break;
	case IA_CSS_STREAM_FORMAT_RGB_555:
		rval = 5;
		break;
	case IA_CSS_STREAM_FORMAT_RGB_565:
	case IA_CSS_STREAM_FORMAT_RGB_666:
	case IA_CSS_STREAM_FORMAT_RAW_6:
		rval = 6;
		break;
	case IA_CSS_STREAM_FORMAT_RAW_7:
		rval = 7;
		break;
	case IA_CSS_STREAM_FORMAT_YUV420_8_LEGACY:
	case IA_CSS_STREAM_FORMAT_YUV420_8:
	case IA_CSS_STREAM_FORMAT_YUV422_8:
	case IA_CSS_STREAM_FORMAT_RGB_888:
	case IA_CSS_STREAM_FORMAT_RAW_8:
	case IA_CSS_STREAM_FORMAT_BINARY_8:
	case IA_CSS_STREAM_FORMAT_USER_DEF1:
	case IA_CSS_STREAM_FORMAT_USER_DEF2:
	case IA_CSS_STREAM_FORMAT_USER_DEF3:
	case IA_CSS_STREAM_FORMAT_USER_DEF4:
	case IA_CSS_STREAM_FORMAT_USER_DEF5:
	case IA_CSS_STREAM_FORMAT_USER_DEF6:
	case IA_CSS_STREAM_FORMAT_USER_DEF7:
	case IA_CSS_STREAM_FORMAT_USER_DEF8:
		rval = 8;
		break;
	case IA_CSS_STREAM_FORMAT_YUV420_10:
	case IA_CSS_STREAM_FORMAT_YUV422_10:
	case IA_CSS_STREAM_FORMAT_RAW_10:
		rval = 10;
		break;
	case IA_CSS_STREAM_FORMAT_RAW_12:
		rval = 12;
		break;
	case IA_CSS_STREAM_FORMAT_RAW_14:
		rval = 14;
		break;
	case IA_CSS_STREAM_FORMAT_RAW_16:
		rval = 16;
		break;
	default:
		rval = 0;
		break;
	}

	return rval;
}

static unsigned int csi2_protocol_calculate_max_subpixels_per_line(
		enum ia_css_stream_format	format,
		unsigned int			pixels_per_line)
{
	unsigned int rval;

	switch (format) {
	case IA_CSS_STREAM_FORMAT_YUV420_8_LEGACY:
		/*
		 * The frame format layout is shown below.
		 *
		 *		Line	0:	UYY UYY ... UYY
		 *		Line	1:	VYY VYY ... VYY
		 *		Line	2:	UYY UYY ... UYY
		 *		Line	3:	VYY VYY ... VYY
		 *		...
		 *		Line (n-2):	UYY UYY ... UYY
		 *		Line (n-1):	VYY VYY ... VYY
		 *
		 *	In this frame format, the even-line is
		 *	as wide as the odd-line.
		 */
		rval = pixels_per_line + pixels_per_line / 2;
		break;
	case IA_CSS_STREAM_FORMAT_YUV420_8:
	case IA_CSS_STREAM_FORMAT_YUV420_10:
		/*
		 * The frame format layout is shown below.
		 *
		 *		Line	0:	YYYY YYYY ... YYYY
		 *		Line	1:	UYVY UYVY ... UYVY UYVY
		 *		Line	2:	YYYY YYYY ... YYYY
		 *		Line	3:	UYVY UYVY ... UYVY UYVY
		 *		...
		 *		Line (n-2):	YYYY YYYY ... YYYY
		 *		Line (n-1):	UYVY UYVY ... UYVY UYVY
		 *
		 * In this frame format, the odd-line is twice
		 * wider than the even-line.
		 */
		rval = pixels_per_line * 2;
		break;
	case IA_CSS_STREAM_FORMAT_YUV422_8:
	case IA_CSS_STREAM_FORMAT_YUV422_10:
		/*
		 * The frame format layout is shown below.
		 *
		 *		Line	0:	UYVY UYVY ... UYVY
		 *		Line	1:	UYVY UYVY ... UYVY
		 *		Line	2:	UYVY UYVY ... UYVY
		 *		Line	3:	UYVY UYVY ... UYVY
		 *		...
		 *		Line (n-2):	UYVY UYVY ... UYVY
		 *		Line (n-1):	UYVY UYVY ... UYVY
		 *
		 * In this frame format, the even-line is
		 * as wide as the odd-line.
		 */
		rval = pixels_per_line * 2;
		break;
	case IA_CSS_STREAM_FORMAT_RGB_444:
	case IA_CSS_STREAM_FORMAT_RGB_555:
	case IA_CSS_STREAM_FORMAT_RGB_565:
	case IA_CSS_STREAM_FORMAT_RGB_666:
	case IA_CSS_STREAM_FORMAT_RGB_888:
		/*
		 * The frame format layout is shown below.
		 *
		 *		Line	0:	ABGR ABGR ... ABGR
		 *		Line	1:	ABGR ABGR ... ABGR
		 *		Line	2:	ABGR ABGR ... ABGR
		 *		Line	3:	ABGR ABGR ... ABGR
		 *		...
		 *		Line (n-2):	ABGR ABGR ... ABGR
		 *		Line (n-1):	ABGR ABGR ... ABGR
		 *
		 * In this frame format, the even-line is
		 * as wide as the odd-line.
		 */
		rval = pixels_per_line * 4;
		break;
	case IA_CSS_STREAM_FORMAT_RAW_6:
	case IA_CSS_STREAM_FORMAT_RAW_7:
	case IA_CSS_STREAM_FORMAT_RAW_8:
	case IA_CSS_STREAM_FORMAT_RAW_10:
	case IA_CSS_STREAM_FORMAT_RAW_12:
	case IA_CSS_STREAM_FORMAT_RAW_14:
	case IA_CSS_STREAM_FORMAT_RAW_16:
	case IA_CSS_STREAM_FORMAT_BINARY_8:
	case IA_CSS_STREAM_FORMAT_USER_DEF1:
	case IA_CSS_STREAM_FORMAT_USER_DEF2:
	case IA_CSS_STREAM_FORMAT_USER_DEF3:
	case IA_CSS_STREAM_FORMAT_USER_DEF4:
	case IA_CSS_STREAM_FORMAT_USER_DEF5:
	case IA_CSS_STREAM_FORMAT_USER_DEF6:
	case IA_CSS_STREAM_FORMAT_USER_DEF7:
	case IA_CSS_STREAM_FORMAT_USER_DEF8:
		/*
		 * The frame format layout is shown below.
		 *
		 *		Line	0:	Pixel Pixel ... Pixel
		 *		Line	1:	Pixel Pixel ... Pixel
		 *		Line	2:	Pixel Pixel ... Pixel
		 *		Line	3:	Pixel Pixel ... Pixel
		 *		...
		 *		Line (n-2):	Pixel Pixel ... Pixel
		 *		Line (n-1):	Pixel Pixel ... Pixel
		 *
		 * In this frame format, the even-line is
		 * as wide as the odd-line.
		 */
		rval = pixels_per_line;
		break;
	default:
		rval = 0;
		break;
	}

	return rval;
}

static bool sh_css_translate_stream_cfg_to_input_system_input_port_id(
		struct ia_css_stream_config *stream_cfg,
		ia_css_isys_descr_t	*isys_stream_descr)
{
	bool rc;

	rc = true;
	switch (stream_cfg->mode) {
	case IA_CSS_INPUT_MODE_TPG:

		if (stream_cfg->source.tpg.id == IA_CSS_TPG_ID0) {
			isys_stream_descr->input_port_id = INPUT_SYSTEM_PIXELGEN_PORT0_ID;
		} else if (stream_cfg->source.tpg.id == IA_CSS_TPG_ID1) {
			isys_stream_descr->input_port_id = INPUT_SYSTEM_PIXELGEN_PORT1_ID;
		} else if (stream_cfg->source.tpg.id == IA_CSS_TPG_ID2) {
			isys_stream_descr->input_port_id = INPUT_SYSTEM_PIXELGEN_PORT2_ID;
		}

		break;
	case IA_CSS_INPUT_MODE_PRBS:

		if (stream_cfg->source.prbs.id == IA_CSS_PRBS_ID0) {
			isys_stream_descr->input_port_id = INPUT_SYSTEM_PIXELGEN_PORT0_ID;
		} else if (stream_cfg->source.prbs.id == IA_CSS_PRBS_ID1) {
			isys_stream_descr->input_port_id = INPUT_SYSTEM_PIXELGEN_PORT1_ID;
		} else if (stream_cfg->source.prbs.id == IA_CSS_PRBS_ID2) {
			isys_stream_descr->input_port_id = INPUT_SYSTEM_PIXELGEN_PORT2_ID;
		}

		break;
	case IA_CSS_INPUT_MODE_BUFFERED_SENSOR:

		if (stream_cfg->source.port.port == IA_CSS_CSI2_PORT0) {
			isys_stream_descr->input_port_id = INPUT_SYSTEM_CSI_PORT0_ID;
		} else if (stream_cfg->source.port.port == IA_CSS_CSI2_PORT1) {
			isys_stream_descr->input_port_id = INPUT_SYSTEM_CSI_PORT1_ID;
		} else if (stream_cfg->source.port.port == IA_CSS_CSI2_PORT2) {
			isys_stream_descr->input_port_id = INPUT_SYSTEM_CSI_PORT2_ID;
		}

		break;
	default:
		rc = false;
		break;
	}

	return rc;
}

static bool sh_css_translate_stream_cfg_to_input_system_input_port_type(
		struct ia_css_stream_config *stream_cfg,
		ia_css_isys_descr_t	*isys_stream_descr)
{
	bool rc;

	rc = true;
	switch (stream_cfg->mode) {
	case IA_CSS_INPUT_MODE_TPG:

		isys_stream_descr->mode = INPUT_SYSTEM_SOURCE_TYPE_TPG;

		break;
	case IA_CSS_INPUT_MODE_PRBS:

		isys_stream_descr->mode = INPUT_SYSTEM_SOURCE_TYPE_PRBS;

		break;
	case IA_CSS_INPUT_MODE_SENSOR:
	case IA_CSS_INPUT_MODE_BUFFERED_SENSOR:

		isys_stream_descr->mode = INPUT_SYSTEM_SOURCE_TYPE_SENSOR;
		break;

	default:
		rc = false;
		break;
	}

	return rc;
}

static bool sh_css_translate_stream_cfg_to_input_system_input_port_attr(
		struct ia_css_stream_config *stream_cfg,
		ia_css_isys_descr_t	*isys_stream_descr)
{
	bool rc;

	rc = true;
	switch (stream_cfg->mode) {
	case IA_CSS_INPUT_MODE_TPG:
		if (stream_cfg->source.tpg.mode == IA_CSS_TPG_MODE_RAMP) {
			isys_stream_descr->tpg_port_attr.mode = PIXELGEN_TPG_MODE_RAMP;
		} else if (stream_cfg->source.tpg.mode == IA_CSS_TPG_MODE_CHECKERBOARD) {
			isys_stream_descr->tpg_port_attr.mode = PIXELGEN_TPG_MODE_CHBO;
		} else if (stream_cfg->source.tpg.mode == IA_CSS_TPG_MODE_MONO) {
			isys_stream_descr->tpg_port_attr.mode = PIXELGEN_TPG_MODE_MONO;
		} else {
			rc = false;
		}

		/*
		 * zhengjie.lu@intel.com:
		 *
		 * TODO
		 * - Make "color_cfg" as part of "ia_css_tpg_config".
		 */
		isys_stream_descr->tpg_port_attr.color_cfg.R1 = 51;
		isys_stream_descr->tpg_port_attr.color_cfg.G1 = 102;
		isys_stream_descr->tpg_port_attr.color_cfg.B1 = 255;
		isys_stream_descr->tpg_port_attr.color_cfg.R2 = 0;
		isys_stream_descr->tpg_port_attr.color_cfg.G2 = 100;
		isys_stream_descr->tpg_port_attr.color_cfg.B2 = 160;

		isys_stream_descr->tpg_port_attr.mask_cfg.h_mask = stream_cfg->source.tpg.x_mask;
		isys_stream_descr->tpg_port_attr.mask_cfg.v_mask = stream_cfg->source.tpg.y_mask;
		isys_stream_descr->tpg_port_attr.mask_cfg.hv_mask = stream_cfg->source.tpg.xy_mask;

		isys_stream_descr->tpg_port_attr.delta_cfg.h_delta = stream_cfg->source.tpg.x_delta;
		isys_stream_descr->tpg_port_attr.delta_cfg.v_delta = stream_cfg->source.tpg.y_delta;

		/*
		 * zhengjie.lu@intel.com:
		 *
		 * TODO
		 * - Make "sync_gen_cfg" as part of "ia_css_tpg_config".
		 */
		isys_stream_descr->tpg_port_attr.sync_gen_cfg.hblank_cycles = 100;
		isys_stream_descr->tpg_port_attr.sync_gen_cfg.vblank_cycles = 100;
		isys_stream_descr->tpg_port_attr.sync_gen_cfg.pixels_per_clock = stream_cfg->pixels_per_clock;
		isys_stream_descr->tpg_port_attr.sync_gen_cfg.nr_of_frames = ~(0x0);
		isys_stream_descr->tpg_port_attr.sync_gen_cfg.pixels_per_line = stream_cfg->input_res.width;
		isys_stream_descr->tpg_port_attr.sync_gen_cfg.lines_per_frame = stream_cfg->input_res.height;

		break;
	case IA_CSS_INPUT_MODE_PRBS:

		isys_stream_descr->prbs_port_attr.seed0 = stream_cfg->source.prbs.seed;
		isys_stream_descr->prbs_port_attr.seed1 = stream_cfg->source.prbs.seed1;

		/*
		 * zhengjie.lu@intel.com:
		 *
		 * TODO
		 * - Make "sync_gen_cfg" as part of "ia_css_prbs_config".
		 */
		isys_stream_descr->prbs_port_attr.sync_gen_cfg.hblank_cycles = 100;
		isys_stream_descr->prbs_port_attr.sync_gen_cfg.vblank_cycles = 100;
		isys_stream_descr->prbs_port_attr.sync_gen_cfg.pixels_per_clock = stream_cfg->pixels_per_clock;
		isys_stream_descr->prbs_port_attr.sync_gen_cfg.nr_of_frames = ~(0x0);
		isys_stream_descr->prbs_port_attr.sync_gen_cfg.pixels_per_line = stream_cfg->input_res.width;
		isys_stream_descr->prbs_port_attr.sync_gen_cfg.lines_per_frame = stream_cfg->input_res.height;

		break;
	case IA_CSS_INPUT_MODE_BUFFERED_SENSOR:
	{
		enum ia_css_err err;
		unsigned int fmt_type;

		err = ia_css_isys_convert_stream_format_to_mipi_format(
			stream_cfg->format,
			sh_css_csi2_compression_type_2_mipi_predictor(stream_cfg->source.port.compression.type),
				&fmt_type);
		if (err != IA_CSS_SUCCESS)
			rc = false;

		isys_stream_descr->csi_port_attr.active_lanes = stream_cfg->source.port.num_lanes;
		isys_stream_descr->csi_port_attr.fmt_type = fmt_type;
		isys_stream_descr->csi_port_attr.ch_id = stream_cfg->channel_id;
#ifdef USE_INPUT_SYSTEM_VERSION_2401
		isys_stream_descr->online = stream_cfg->online;
#endif

		/* metadata */
		isys_stream_descr->metadata.enable = false;
		if (stream_cfg->metadata_config.resolution.height > 0) {
			err = ia_css_isys_convert_stream_format_to_mipi_format(
				stream_cfg->metadata_config.data_type,
				sh_css_csi2_compression_type_2_mipi_predictor(stream_cfg->source.port.compression.type),
					&fmt_type);
			if (err != IA_CSS_SUCCESS)
				rc = false;
			isys_stream_descr->metadata.fmt_type = fmt_type;
			isys_stream_descr->metadata.bits_per_pixel =
				ia_css_util_input_format_bpp(stream_cfg->metadata_config.data_type, true);
			isys_stream_descr->metadata.pixels_per_line = stream_cfg->metadata_config.resolution.width;
			isys_stream_descr->metadata.lines_per_frame = stream_cfg->metadata_config.resolution.height;
			isys_stream_descr->metadata.enable = true;
		}

		break;
	}
	default:
		rc = false;
		break;
	}

	return rc;
}

static bool sh_css_translate_stream_cfg_to_input_system_input_port_resolution(
		struct ia_css_stream_config *stream_cfg,
		ia_css_isys_descr_t	*isys_stream_descr)
{
	unsigned int bits_per_subpixel;
	/* unsigned int subpixels_per_color_pixel; */
	/* unsigned int color_pixels_per_line; */
	unsigned int max_subpixels_per_line;
	unsigned int lines_per_frame;

	bits_per_subpixel =
		sh_css_stream_format_2_bits_per_subpixel(stream_cfg->format);
	if (bits_per_subpixel == 0)
		return false;

	if (stream_cfg->online){
		max_subpixels_per_line =
			csi2_protocol_calculate_max_subpixels_per_line(stream_cfg->format, stream_cfg->input_res.width);
		}
	else {
		max_subpixels_per_line =
			csi2_protocol_calculate_max_subpixels_per_line(stream_cfg->format, stream_cfg->input_res.width);
		}
	if (max_subpixels_per_line == 0)
		return false;

	lines_per_frame = stream_cfg->input_res.height;
	if (lines_per_frame == 0)
		return false;

	/* HW needs subpixel info for their settings */
	isys_stream_descr->input_port_resolution.bits_per_pixel	= bits_per_subpixel;
	isys_stream_descr->input_port_resolution.pixels_per_line	= max_subpixels_per_line;
	isys_stream_descr->input_port_resolution.lines_per_frame = lines_per_frame;

	return true;
}

static bool sh_css_translate_stream_cfg_to_isys_stream_descr(
		struct ia_css_stream_config *stream_cfg,
		ia_css_isys_descr_t	*isys_stream_descr)
{
	bool rc;

	rc  = sh_css_translate_stream_cfg_to_input_system_input_port_id(stream_cfg, isys_stream_descr);
	rc &= sh_css_translate_stream_cfg_to_input_system_input_port_type(stream_cfg, isys_stream_descr);
	rc &= sh_css_translate_stream_cfg_to_input_system_input_port_attr(stream_cfg, isys_stream_descr);
	rc &= sh_css_translate_stream_cfg_to_input_system_input_port_resolution(stream_cfg, isys_stream_descr);

	return rc;
}

static bool sh_css_translate_binary_info_to_input_system_output_port_attr(
		struct ia_css_binary *binary,
                ia_css_isys_descr_t     *isys_stream_descr)
{
	if (!binary)
		return false;

	isys_stream_descr->output_port_attr.left_padding = binary->left_padding;
	isys_stream_descr->output_port_attr.max_isp_input_width = binary->info->sp.max_input_width;

	return true;
}

static enum ia_css_err
sh_css_config_input_network(struct ia_css_pipe *pipe,
		struct ia_css_binary *binary)
{
	bool					rc;
	ia_css_isys_descr_t			isys_stream_descr;
	unsigned int				sp_thread_id;
	struct sh_css_sp_pipeline_terminal	*sp_pipeline_input_terminal;

	if (pipe->pipeline.stages)
		binary = pipe->pipeline.stages->binary;

	/* initialization */
	memset((void*)(&isys_stream_descr), 0, sizeof(ia_css_isys_descr_t));

	/* translate the stream configuration to the Input System (2401) configuration */
	rc = sh_css_translate_stream_cfg_to_isys_stream_descr(
			&(pipe->stream->config),
			&(isys_stream_descr));

	if (pipe->stream->config.online) {
		rc &= sh_css_translate_binary_info_to_input_system_output_port_attr(
				binary,
				&(isys_stream_descr));
	}

	if (rc != true)
		return IA_CSS_ERR_INTERNAL_ERROR;

	/* get the SP thread id */
	rc = ia_css_pipeline_get_sp_thread_id(ia_css_pipe_get_pipe_num(pipe), &sp_thread_id);
	if (rc != true)
		return IA_CSS_ERR_INTERNAL_ERROR;

	/* get the target input terminal */
	sp_pipeline_input_terminal = &(sh_css_sp_group.pipe_io[sp_thread_id].input);

	/* create the virtual Input System (2401) */
	rc =  ia_css_isys_stream_create(
			&(isys_stream_descr),
			&(sp_pipeline_input_terminal->context.virtual_input_system));
	if (rc != true)
		return IA_CSS_ERR_INTERNAL_ERROR;

	/* calculate the configuration of the virtual Input System (2401) */
	rc = ia_css_isys_stream_calculate_cfg(
			&(sp_pipeline_input_terminal->context.virtual_input_system),
			&(isys_stream_descr),
			&(sp_pipeline_input_terminal->ctrl.virtual_input_system_cfg));
	if (rc != true) {
		ia_css_isys_stream_destroy(&(sp_pipeline_input_terminal->context.virtual_input_system));
		return IA_CSS_ERR_INTERNAL_ERROR;
	}

	return IA_CSS_SUCCESS;
}

static struct ia_css_pipe* stream_get_last_pipe(
		struct ia_css_stream *stream)
{
	struct ia_css_pipe *last_pipe = NULL;
	if (stream != NULL)
		last_pipe = stream->last_pipe;

	return last_pipe;
}

static struct ia_css_pipe* stream_get_copy_pipe(
		struct ia_css_stream *stream)
{
	struct ia_css_pipe *copy_pipe = NULL;
	struct ia_css_pipe *last_pipe = NULL;
	enum ia_css_pipe_id pipe_id;

	last_pipe = stream_get_last_pipe(stream);

	if ((stream != NULL) &&
	    (last_pipe != NULL) &&
	    (stream->config.continuous)) {

		pipe_id = last_pipe->mode;
		switch (pipe_id) {
			case IA_CSS_PIPE_ID_PREVIEW:
				copy_pipe = last_pipe->pipe_settings.preview.copy_pipe;
				break;
			case IA_CSS_PIPE_ID_VIDEO:
				copy_pipe = last_pipe->pipe_settings.video.copy_pipe;
				break;
			default:
				copy_pipe = NULL;
				break;
		}
	}

	return copy_pipe;
}


static enum ia_css_err stream_register_with_csi_rx(
	struct ia_css_stream *stream)
{
	enum ia_css_err retval = IA_CSS_ERR_INTERNAL_ERROR;
	unsigned int	sp_thread_id;

	struct ia_css_pipe *target_pipe = NULL;

	if ((stream != NULL) && (stream->config.mode == IA_CSS_INPUT_MODE_BUFFERED_SENSOR)) {
		/* get the pipe that consumes the stream */
		if (stream->config.continuous) {
			target_pipe = stream_get_copy_pipe(stream);
		} else {
			target_pipe = stream_get_last_pipe(stream);
		}

		if (target_pipe != NULL) {
			bool rc;
			rc = ia_css_pipeline_get_sp_thread_id(
				ia_css_pipe_get_pipe_num(target_pipe),
				&sp_thread_id);
			if (rc) {
				retval = ia_css_isys_csi_rx_register_stream(
						stream->config.source.port.port,
						sp_thread_id);
			}
		}
	}

	return retval;
}

static enum ia_css_err stream_unregister_with_csi_rx(
	struct ia_css_stream *stream)
{
	enum ia_css_err retval = IA_CSS_ERR_INTERNAL_ERROR;
	unsigned int	sp_thread_id;
	struct ia_css_pipe *target_pipe = NULL;

	if ((stream != NULL) && (stream->config.mode == IA_CSS_INPUT_MODE_BUFFERED_SENSOR)) {

		if (stream->config.continuous) {
			target_pipe = stream_get_copy_pipe(stream);
		} else {
			target_pipe = stream_get_last_pipe(stream);
		}

		if (target_pipe != NULL) {
			bool rc;
			rc = ia_css_pipeline_get_sp_thread_id(
				ia_css_pipe_get_pipe_num(target_pipe),
				&sp_thread_id);
			if (rc) {
				retval = ia_css_isys_csi_rx_unregister_stream(
						stream->config.source.port.port,
						sp_thread_id);
			}
		}
	}

	return retval;
}
#endif

#if WITH_PC_MONITORING
static struct task_struct *my_kthread;    /* Handle for the monitoring thread */
static int sh_binary_running;         /* Enable sampling in the thread */

static void print_pc_histo(char *core_name, struct sh_css_pc_histogram *hist)
{
	unsigned i;
	unsigned cnt_run = 0;
	unsigned cnt_stall = 0;

	assert(hist != NULL);

	sh_css_print("%s histogram length = %d\n", core_name, hist->length);
	sh_css_print("%s PC\trun\tstall\n", core_name);

	for (i = 0; i < hist->length; i++) {
		if ((hist->run[i] == 0) && (hist->run[i] == hist->stall[i]))
			continue;
		sh_css_print("%s %d\t%d\t%d\n",
				core_name, i, hist->run[i], hist->stall[i]);
		cnt_run += hist->run[i];
		cnt_stall += hist->stall[i];
	}

	sh_css_print(" Statistics for %s, cnt_run = %d, cnt_stall = %d, "
	       "hist->length = %d\n",
			core_name, cnt_run, cnt_stall, hist->length);
}

static void print_pc_histogram(void)
{
	struct ia_css_binary_metrics *metrics;

	for (metrics = sh_css_metrics.binary_metrics;
	     metrics;
	     metrics = metrics->next) {
		if (metrics->mode == IA_CSS_BINARY_MODE_PREVIEW ||
		    metrics->mode == IA_CSS_BINARY_MODE_VF_PP) {
			sh_css_print("pc_histogram for binary %d is SKIPPED\n",
				metrics->id);
			continue;
		}

		sh_css_print(" pc_histogram for binary %d\n", metrics->id);
		print_pc_histo("  ISP", &metrics->isp_histogram);
		print_pc_histo("  SP",   &metrics->sp_histogram);
		sh_css_print("print_pc_histogram() done for binay->id = %d, "
			     "done.\n", metrics->id);
	}

	sh_css_print("PC_MONITORING:print_pc_histogram() -- DONE\n");
}

static int pc_monitoring(void *data)
{
	int i = 0;

	(void)data;
	while (true) {
		if (sh_binary_running) {
			sh_css_metrics_sample_pcs();
#if MULTIPLE_SAMPLES
			for (i = 0; i < NOF_SAMPLES; i++)
				sh_css_metrics_sample_pcs();
#endif
		}
		usleep_range(10, 50);
	}
	return 0;
}

static void spying_thread_create(void)
{
	my_kthread = kthread_run(pc_monitoring, NULL, "sh_pc_monitor");
	sh_css_metrics_enable_pc_histogram(1);
}

static void input_frame_info(struct ia_css_frame_info frame_info)
{
	sh_css_print("SH_CSS:input_frame_info() -- frame->info.res.width = %d, "
	       "frame->info.res.height = %d, format = %d\n",
			frame_info.res.width, frame_info.res.height, frame_info.format);
}
#endif /* WITH_PC_MONITORING */

static void
start_binary(struct ia_css_pipe *pipe,
	     struct ia_css_binary *binary)
{
	struct ia_css_stream *stream;

	assert(pipe != NULL);
	/* Acceleration uses firmware, the binary thus can be NULL */
	/* assert(binary != NULL); */

	(void)binary;

#if !defined(HAS_NO_INPUT_SYSTEM)
	stream = pipe->stream;
#else
	(void)pipe;
	(void)stream;
#endif

#ifdef THIS_CODE_IS_NO_LONGER_NEEDED_FOR_DUAL_STREAM
#if !defined(HAS_NO_INPUT_SYSTEM)
	if (stream && stream->reconfigure_css_rx)
		ia_css_isys_rx_disable();
#endif
#endif

	if (binary)
		sh_css_metrics_start_binary(&binary->metrics);

#if WITH_PC_MONITORING
	sh_css_print("PC_MONITORING: %s() -- binary id = %d , "
		     "enable_dvs_envelope = %d\n",
		     __func__, binary->info->sp.id,
		     binary->info->sp.enable.dvs_envelope);
	input_frame_info(binary->in_frame_info);

	if (binary && binary->info->sp.mode == IA_CSS_BINARY_MODE_VIDEO)
		sh_binary_running = true;
#endif

	/* sh_css_sp_start_isp(); */

#if !defined(HAS_NO_INPUT_SYSTEM) && !defined(USE_INPUT_SYSTEM_VERSION_2401)
	if (stream->reconfigure_css_rx) {
		ia_css_isys_rx_configure(&pipe->stream->csi_rx_config,
					 pipe->stream->config.mode);
		stream->reconfigure_css_rx = false;
	}
#endif
}

/* start the copy function on the SP */
static enum ia_css_err
start_copy_on_sp(struct ia_css_pipe *pipe,
		 struct ia_css_frame *out_frame)
{

	(void)out_frame;
	assert(pipe != NULL);
	assert(pipe->stream != NULL);

	if ((pipe == NULL) || (pipe->stream == NULL))
		return IA_CSS_ERR_INVALID_ARGUMENTS;

#if !defined(HAS_NO_INPUT_SYSTEM) && !defined(USE_INPUT_SYSTEM_VERSION_2401)
	if (pipe->stream->reconfigure_css_rx)
		ia_css_isys_rx_disable();
#endif

	if (pipe->stream->config.format != IA_CSS_STREAM_FORMAT_BINARY_8)
		return IA_CSS_ERR_INTERNAL_ERROR;
	sh_css_sp_start_binary_copy(ia_css_pipe_get_pipe_num(pipe), out_frame, pipe->stream->config.pixels_per_clock == 2);

	//sh_css_sp_start_isp();

#if !defined(HAS_NO_INPUT_SYSTEM) && !defined(USE_INPUT_SYSTEM_VERSION_2401)
	if (pipe->stream->reconfigure_css_rx) {
		ia_css_isys_rx_configure(&pipe->stream->csi_rx_config, pipe->stream->config.mode);
		pipe->stream->reconfigure_css_rx = false;
	}
#endif

	return IA_CSS_SUCCESS;
}

void sh_css_binary_args_reset(struct sh_css_binary_args *args)
{
	unsigned int i;

	for (i = 0; i < NUM_VIDEO_TNR_FRAMES; i++)
		args->tnr_frames[i] = NULL;
	for (i = 0; i < NUM_VIDEO_DELAY_FRAMES; i++)
		args->delay_frames[i] = NULL;
	args->in_frame      = NULL;
	for (i = 0; i < IA_CSS_BINARY_MAX_OUTPUT_PORTS; i++)
		args->out_frame[i] = NULL;
	args->out_vf_frame  = NULL;
	args->copy_vf       = false;
	args->copy_output   = true;
	args->vf_downscale_log2 = 0;
}


static void
pipe_start(struct ia_css_pipe *pipe)
{
	struct ia_css_pipeline_stage *stage;
	assert(pipe != NULL);
	stage = pipe->pipeline.stages;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"pipe_start() enter:\n");
	if (!stage)
		return;
	pipe->pipeline.current_stage = stage;

	start_binary(pipe, stage->binary);
}

static enum ia_css_err start_pipe(
	struct ia_css_pipe *me,
	enum sh_css_pipe_config_override copy_ovrd,
	enum ia_css_input_mode input_mode)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	assert(me != NULL);

#if defined(HAS_NO_INPUT_SYSTEM)
	(void)input_mode;
#endif

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"start_pipe() enter:\n");

	configure_pipe_inout_port(&me->pipeline, me->stream->config.continuous);

	sh_css_sp_init_pipeline(&me->pipeline,
				me->mode,
				(uint8_t)ia_css_pipe_get_pipe_num(me),
				me->config.default_capture_config.enable_xnr,
				me->stream->config.pixels_per_clock == 2,
				me->stream->config.continuous,
				false,
				me->required_bds_factor,
				copy_ovrd,
				input_mode,
				&me->stream->config.metadata_config,
				&me->stream->info.metadata_info
#if !defined(HAS_NO_INPUT_SYSTEM)
				, (input_mode==IA_CSS_INPUT_MODE_MEMORY)?
					(mipi_port_ID_t)0:
				me->stream->config.source.port.port
#endif
				);

	if (me->config.mode != IA_CSS_PIPE_MODE_COPY){
		/* prepare update of params to ddr */
		err = sh_css_commit_isp_config(me->stream, &me->pipeline);
		if (err == IA_CSS_SUCCESS)
			pipe_start(me);
	}
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"start_pipe() leave: return (%d)\n", err);
	return err;
}

void
sh_css_invalidate_shading_tables(struct ia_css_stream *stream)
{
	int i;
	assert(stream != NULL);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"sh_css_invalidate_shading_tables() enter:\n");

	for (i=0; i<stream->num_pipes; i++) {
		assert(stream->pipes[i] != NULL);
		sh_css_pipe_free_shading_table(stream->pipes[i]);
	}

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"sh_css_invalidate_shading_tables() leave: return_void\n");
}

static void
enable_interrupts(enum ia_css_irq_type irq_type)
{
	bool enable_pulse = irq_type != IA_CSS_IRQ_TYPE_EDGE;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "enable_interrupts() enter:\n");
/* Enable IRQ on the SP which signals that SP goes to idle (aka ready state) */
	cnd_sp_irq_enable(SP0_ID, true);
/* Set the IRQ device 0 to either level or pulse */
	irq_enable_pulse(IRQ0_ID, enable_pulse);

#if defined(IS_ISP_2500_SYSTEM)
	cnd_virq_enable_channel(virq_sp0, true);
	cnd_virq_enable_channel(virq_sp1, true);
#else
	cnd_virq_enable_channel(virq_sp, true);
#endif

	/* Triggered by SP to signal Host that there are new statistics */
	cnd_virq_enable_channel((virq_id_t)(IRQ_SW_CHANNEL1_ID + IRQ_SW_CHANNEL_OFFSET), true);
	/* Triggered by SP to signal Host that there is data in one of the
	 * SP->Host queues.*/
#if !defined(HAS_IRQ_MAP_VERSION_2)
/* IRQ_SW_CHANNEL2_ID does not exist on 240x systems */
	cnd_virq_enable_channel((virq_id_t)(IRQ_SW_CHANNEL2_ID + IRQ_SW_CHANNEL_OFFSET), true);
	virq_clear_all();
#endif

#if !defined(HAS_NO_INPUT_SYSTEM) && !defined(USE_INPUT_SYSTEM_VERSION_2401)
	ia_css_isys_rx_enable_all_interrupts();
#endif

#if defined(HRT_CSIM)
/*
 * Enable IRQ on the SP which signals that SP goes to idle
 * to get statistics for each binary
 */
	cnd_isp_irq_enable(ISP0_ID, true);
	cnd_virq_enable_channel(virq_isp, true);
#endif
}

static bool sh_css_setup_spctrl_config(const struct ia_css_fw_info *fw,
							const char * program,
							ia_css_spctrl_cfg  *spctrl_cfg)
{
	if((fw == NULL)||(spctrl_cfg == NULL))
		return false;
	spctrl_cfg->sp_entry = 0;
	spctrl_cfg->program_name = (char *)(program);

#if !defined(C_RUN) && !defined(HRT_UNSCHED)
	spctrl_cfg->ddr_data_offset =  fw->blob.data_source;
	spctrl_cfg->dmem_data_addr = fw->blob.data_target;
	spctrl_cfg->dmem_bss_addr = fw->blob.bss_target;
	spctrl_cfg->data_size = fw->blob.data_size ;
	spctrl_cfg->bss_size = fw->blob.bss_size;

	spctrl_cfg->spctrl_config_dmem_addr = fw->info.sp.init_dmem_data;
	spctrl_cfg->spctrl_state_dmem_addr = fw->info.sp.sw_state;

	spctrl_cfg->code_size = fw->blob.size;
	spctrl_cfg->code      = fw->blob.code;
	spctrl_cfg->sp_entry  = fw->info.sp.sp_entry; /* entry function ptr on SP */
#endif
	return true;
}
void
ia_css_unload_firmware(bool forced)
{
	int i = 0;
	ia_css_binary_uninit(forced);	//ia_css_binary_uninit has protection within it for multiple acccesses
	if (sh_css_num_binaries)
	{
		/* we have already loaded before so get rid of the old stuff */
		sh_css_unload_firmware();
	}
	fw_explicitly_loaded = false;

	/*When the forced flag is true, free the mipi_frames that were being reused till now*/
	if (forced){
		for (i = 0; i < my_css.num_mipi_frames; i++) {
	    	if (my_css.mipi_frames[i] != NULL) {
	        	ia_css_frame_free(my_css.mipi_frames[i]);
	            my_css.mipi_frames[i] = NULL;
	        }
	    }
	    ref_count_mipi_allocation = 0;
	}
}

static void
ia_css_reset_defaults(struct sh_css* css)
{
	struct sh_css default_css;

	/* Reset everything to zero */
	memset(&default_css, 0, sizeof(default_css));

	/* Initialize the non zero values*/
	default_css.check_system_idle = true;
	default_css.num_cont_raw_frames = NUM_CONTINUOUS_FRAMES;
#if defined(USE_INPUT_SYSTEM_VERSION_2) || defined(USE_INPUT_SYSTEM_VERSION_2401)
	default_css.num_mipi_frames = NUM_MIPI_FRAMES;
#endif
	default_css.contiguous = true;
	default_css.irq_type = IA_CSS_IRQ_TYPE_EDGE;

	/*Set the defaults to the output */
	*css = default_css;
}

enum ia_css_err
ia_css_load_firmware(const struct ia_css_env *env,
	    const struct ia_css_fw  *fw)
{
	enum ia_css_err err;

	if (env == NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	if (fw == NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_load_firmware() enter\n");

	ia_css_memory_access_init(&env->css_mem_env);

	/* make sure we initialize my_css */
	if ((my_css.malloc != env->cpu_mem_env.alloc) ||
	    (my_css.free != env->cpu_mem_env.free) ||
	    (my_css.flush != env->cpu_mem_env.flush)
	    )
	{
		ia_css_reset_defaults(&my_css);

		my_css.malloc = env->cpu_mem_env.alloc;
		my_css.free = env->cpu_mem_env.free;
		my_css.flush = env->cpu_mem_env.flush;
	}

	ia_css_unload_firmware(false); /* in case we are called twice */
	err = sh_css_load_firmware(fw->data, fw->bytes);
	if (err == IA_CSS_SUCCESS) {
		err = ia_css_binary_init_infos();
		if (err == IA_CSS_SUCCESS)
			fw_explicitly_loaded = true;
	}

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_load_firmware() leave \n");
	return err;

}

enum ia_css_err
ia_css_init(const struct ia_css_env *env,
	    const struct ia_css_fw  *fw,
	    uint32_t                 mmu_l1_base,
	    enum ia_css_irq_type     irq_type)
{
	enum ia_css_err err;
	ia_css_spctrl_cfg spctrl_cfg;
#if defined(IS_ISP_2500_SYSTEM)
	ia_css_spctrl_cfg sp1ctrl_cfg;
#endif
	//uint32_t i = 0;
	int i = 0;
	struct ia_css_frame *backup_mipi_frames[NUM_MIPI_FRAMES] = {NULL, NULL, NULL, NULL};

	void *(*malloc_func) (size_t size, bool zero_mem);
	void (*free_func) (void *ptr);
	void (*flush_func) (struct ia_css_acc_fw *fw);
#if !defined(HAS_NO_GPIO)
	hrt_data select, enable;
#endif

	/**
	 * The C99 standard does not specify the exact object representation of structs;
	 * the representation is compiler dependent.
	 *
	 * The structs that are communicated between host and SP/ISP should have the
	 * exact same object representation. The compiler that is used to compile the
	 * firmware is hivecc.
	 *
	 * To check if a different compiler, used to compile a host application, uses
	 * another object representation, macros are defined specifying the size of
	 * the structs as expected by the firmware.
	 *
	 * A host application shall verify that a sizeof( ) of the struct is equal to
	 * the SIZE_OF_XXX macro of the corresponding struct. If they are not
	 * equal, functionality will break.
	 */
#if !defined(IS_ISP_2500_SYSTEM)
	/* Check struct sh_css_ddr_address_map */
	COMPILATION_ERROR_IF( sizeof(struct sh_css_ddr_address_map)		!= SIZE_OF_SH_CSS_DDR_ADDRESS_MAP_STRUCT	);
#endif
	/* Check struct host_sp_queues */
	COMPILATION_ERROR_IF( sizeof(struct host_sp_queues)			!= SIZE_OF_HOST_SP_QUEUES_STRUCT		);
	COMPILATION_ERROR_IF( sizeof(struct ia_css_circbuf_desc_s)		!= SIZE_OF_IA_CSS_CIRCBUF_DESC_S_STRUCT		);
	COMPILATION_ERROR_IF( sizeof(struct ia_css_circbuf_elem_s)		!= SIZE_OF_IA_CSS_CIRCBUF_ELEM_S_STRUCT		);

	/* Check struct host_sp_communication */
	COMPILATION_ERROR_IF( sizeof(struct host_sp_communication)		!= SIZE_OF_HOST_SP_COMMUNICATION_STRUCT		);
	COMPILATION_ERROR_IF( sizeof(struct sh_css_event_irq_mask)		!= SIZE_OF_SH_CSS_EVENT_IRQ_MASK_STRUCT		);

	/* Check struct sh_css_hmm_buffer */
	COMPILATION_ERROR_IF( sizeof(struct sh_css_hmm_buffer)			!= SIZE_OF_SH_CSS_HMM_BUFFER_STRUCT		);
	COMPILATION_ERROR_IF( sizeof(struct ia_css_isp_3a_statistics)		!= SIZE_OF_IA_CSS_ISP_3A_STATISTICS_STRUCT	);
	COMPILATION_ERROR_IF( sizeof(struct ia_css_isp_dvs_statistics)		!= SIZE_OF_IA_CSS_ISP_DVS_STATISTICS_STRUCT	);
	COMPILATION_ERROR_IF( sizeof(struct ia_css_metadata)			!= SIZE_OF_IA_CSS_METADATA_STRUCT		);

	/* Check struct ia_css_init_dmem_cfg */
	COMPILATION_ERROR_IF( sizeof(struct ia_css_sp_init_dmem_cfg)		!= SIZE_OF_IA_CSS_SP_INIT_DMEM_CFG_STRUCT	);

	if (fw == NULL && !fw_explicitly_loaded)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	if (env == NULL)
	    return IA_CSS_ERR_INVALID_ARGUMENTS;

	malloc_func = env->cpu_mem_env.alloc;
	free_func   = env->cpu_mem_env.free;
	flush_func  = env->cpu_mem_env.flush;

	pipe_global_init();
	ia_css_pipeline_init();
	ia_css_queue_map_init();

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "sh_css_init() enter: void\n");

	ia_css_device_access_init(&env->hw_access_env);
	ia_css_memory_access_init(&env->css_mem_env);

#if !defined(HAS_NO_GPIO)
	select = gpio_reg_load(GPIO0_ID, _gpio_block_reg_do_select)
						& (~GPIO_FLASH_PIN_MASK);
	enable = gpio_reg_load(GPIO0_ID, _gpio_block_reg_do_e)
							| GPIO_FLASH_PIN_MASK;
#endif
	sh_css_mmu_set_page_table_base_index(mmu_l1_base);
	my_css_save.mmu_base = mmu_l1_base;

	if (malloc_func == NULL || free_func == NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;

       /*Temporarily backup the mipi_frames pointers, so that they can be reused*/
       if (backup_mipi_frames[0] == NULL) {
               for (i=0;i<NUM_MIPI_FRAMES;i++) {
                       backup_mipi_frames[i] = my_css.mipi_frames[i];
           }
       }

        ia_css_reset_defaults(&my_css);

       /*restore mipi frames' ponters*/
       for (i=0;i<NUM_MIPI_FRAMES;i++) {
               if (backup_mipi_frames[i] != NULL) my_css.mipi_frames[i] = backup_mipi_frames[i];
               else backup_mipi_frames == NULL;
       }

	my_css.malloc = malloc_func;
	my_css.free = free_func;
	my_css.flush = flush_func;
	sh_css_printf = env->print_env.debug_print;
	my_css_save.driver_env = (struct ia_css_env *)env;

	err = ia_css_rmgr_init();
	if (err != IA_CSS_SUCCESS)
		return err;

	ia_css_debug_set_dtrace_level(IA_CSS_DEBUG_WARNING);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "sh_css_init()\n");

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "sh_css_init() init: %d\n",my_css_save_initialized);

	if (!my_css_save_initialized)
	{
		my_css_save_initialized = true;
		my_css_save.mode = sh_css_mode_working;
		memset(my_css_save.stream_seeds, 0, sizeof(struct sh_css_stream_seed) * MAX_ACTIVE_STREAMS);
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "sh_css_init() init: %d mode=%d\n",my_css_save_initialized, my_css_save.mode);
	}
#if defined(USE_INPUT_SYSTEM_VERSION_2) || defined(USE_INPUT_SYSTEM_VERSION_2401)
	ref_count_mipi_allocation = 0;
#endif
	/* In case this has been programmed already, update internal
	   data structure ... DEPRECATED */
	my_css.page_table_base_index = mmu_get_page_table_base_index(MMU0_ID);

	my_css.irq_type = irq_type;
	my_css_save.irq_type = irq_type;
	enable_interrupts(my_css.irq_type);

#if !defined(HAS_NO_GPIO)
	/* configure GPIO to output mode */
	gpio_reg_store(GPIO0_ID, _gpio_block_reg_do_select, select);
	gpio_reg_store(GPIO0_ID, _gpio_block_reg_do_e, enable);
	gpio_reg_store(GPIO0_ID, _gpio_block_reg_do_0, 0);
#endif

	err = ia_css_refcount_init(REFCOUNT_SIZE);
	if (err != IA_CSS_SUCCESS)
		return err;
	err = sh_css_params_init();
	if (err != IA_CSS_SUCCESS)
		goto refcnt_uninit;
	if (fw)
	{
		ia_css_unload_firmware(false); /* in case we already had firmware loaded */
		err = sh_css_load_firmware(fw->data, fw->bytes);
		if (err != IA_CSS_SUCCESS)
			goto params_uninit;
		err = ia_css_binary_init_infos();
		if (err != IA_CSS_SUCCESS)
			goto params_uninit;
		fw_explicitly_loaded = false;
		my_css_save.loaded_fw = (struct ia_css_fw *)fw;
	}
	if(!sh_css_setup_spctrl_config(&sh_css_sp_fw,SP_PROG_NAME,&spctrl_cfg))
		return IA_CSS_ERR_INTERNAL_ERROR;

	err = ia_css_spctrl_load_fw(SP0_ID, &spctrl_cfg);
	if (err != IA_CSS_SUCCESS) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "sh_css_init() leave: return_err=%d\n",err);
		goto unload_fw;
	}
#if defined(IS_ISP_2500_SYSTEM)
	if(!sh_css_setup_spctrl_config(&sh_css_sp1_fw,SP1_PROG_NAME,&sp1ctrl_cfg))
		return IA_CSS_ERR_INTERNAL_ERROR;
	err = ia_css_sp1ctrl_load_fw(SP1_ID, &sp1ctrl_cfg);
	if (err != IA_CSS_SUCCESS) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "sh_css_init() leave: return_err=%d\n",err);
		goto unload_fw;
	}
#endif

#if defined(HRT_CSIM)
	/**
	 * In compiled simulator context include debug support by default.
	 * In all other cases (e.g. Android phone), the user (e.g. driver)
	 * must explicitly enable debug support by calling this function.
	 */
	if (!ia_css_debug_mode_init()) {
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "sh_css_init() leave: return_err=%d\n",IA_CSS_ERR_INTERNAL_ERROR);
			err = IA_CSS_ERR_INTERNAL_ERROR;
	        goto free_sp_bin_addr;
	}
#endif

#if WITH_PC_MONITORING
	if (!thread_alive) {
		thread_alive++;
		sh_css_print("PC_MONITORING: %s() -- create thread DISABLED\n",
			     __func__);
		spying_thread_create();
	}
	sh_css_printf = printk;
#endif
	if (!sh_css_hrt_system_is_idle()) {
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "sh_css_init() leave: return_err=%d\n",IA_CSS_ERR_SYSTEM_NOT_IDLE);
		err = IA_CSS_ERR_SYSTEM_NOT_IDLE;
        goto free_sp_bin_addr;
	}
	/* can be called here, queuing works, but:
	   - when sp is started later, it will wipe queued items
	   so for now we leave it for later and make sure
	   updates are not called to frequently.
	sh_css_init_buffer_queues();
	*/

#if defined(HAS_INPUT_SYSTEM_VERSION_2) && defined(HAS_INPUT_SYSTEM_VERSION_2401)
#if	defined(USE_INPUT_SYSTEM_VERSION_2)
	gp_device_reg_store(GP_DEVICE0_ID, _REG_GP_SWITCH_ISYS2401_ADDR, 0);
#elif defined (USE_INPUT_SYSTEM_VERSION_2401)
	gp_device_reg_store(GP_DEVICE0_ID, _REG_GP_SWITCH_ISYS2401_ADDR, 1);
#endif
#endif

#if !defined(HAS_NO_INPUT_SYSTEM)
	if(ia_css_isys_init() != INPUT_SYSTEM_ERR_NO_ERROR)
		err = IA_CSS_ERR_INVALID_ARGUMENTS;
#endif

    if(err== IA_CSS_SUCCESS)
       goto init_done;

free_sp_bin_addr:
         mmgr_free(my_css.sp_bin_addr);
         my_css.sp_bin_addr = mmgr_NULL;
unload_fw:
         if(fw)
           ia_css_unload_firmware(true);
params_uninit:
         sh_css_params_uninit();
refcnt_uninit:
         ia_css_refcount_uninit();
init_done:
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "sh_css_init() leave: return_err=%d\n",err);
	return err;
}

enum ia_css_err
ia_css_suspend(void)
{
	int i;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_suspend() enter\n");
	my_css_save.mode = sh_css_mode_suspend;
	for(i=0;i<MAX_ACTIVE_STREAMS;i++)
		if (my_css_save.stream_seeds[i].stream != NULL)
		{
			ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "==*> unloading seed %d (%p)\n", i, my_css_save.stream_seeds[i].stream);
			ia_css_stream_unload(my_css_save.stream_seeds[i].stream);
		}
	my_css_save.mode = sh_css_mode_working;
	ia_css_stop_sp();
	ia_css_uninit();
	for(i=0;i<MAX_ACTIVE_STREAMS;i++)
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "==*> after 1: seed %d (%p)\n", i, my_css_save.stream_seeds[i].stream);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_suspend() leave\n");
	return(IA_CSS_SUCCESS);
}

enum ia_css_err
ia_css_resume(void)
{
	int i;
	enum ia_css_err err;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_resume() enter: void\n");

	err = ia_css_init(my_css_save.driver_env, my_css_save.loaded_fw, my_css_save.mmu_base, my_css_save.irq_type);
	if (err != IA_CSS_SUCCESS)
		return(err);
	err = ia_css_start_sp();
	if (err != IA_CSS_SUCCESS)
		return(err);
	my_css_save.mode = sh_css_mode_resume;
	for(i=0;i<MAX_ACTIVE_STREAMS;i++)
	{
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "==*> seed stream %p\n", my_css_save.stream_seeds[i].stream);
		if (my_css_save.stream_seeds[i].stream != NULL)
		{
			ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "==*> loading seed %d\n", i);
			err = ia_css_stream_load(my_css_save.stream_seeds[i].stream);
			if (err != IA_CSS_SUCCESS)
			{
				int j;
				if (i)
					for(j=0;j<i;j++)
						ia_css_stream_unload(my_css_save.stream_seeds[j].stream);
				return(err);
			}
			*my_css_save.stream_seeds[i].orig_stream = my_css_save.stream_seeds[i].stream;
		}
	}
	my_css_save.mode = sh_css_mode_working;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_resume() leave: return_void\n");
	return(IA_CSS_SUCCESS);
}

void *
sh_css_malloc(size_t size)
{
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "sh_css_malloc() enter: size=%d\n",size);
	if (size > 0 && my_css.malloc)
		return my_css.malloc(size, false);
	return NULL;
}

void *
sh_css_calloc(size_t N, size_t size)
{
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "sh_css_calloc() enter: N=%d, size=%d\n",N,size);
	if (size > 0 && my_css.malloc) {
		return my_css.malloc(N*size, true);
	}
	return NULL;
}

void
sh_css_free(void *ptr)
{
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "sh_css_free() enter:\n");
	if (ptr && my_css.free)
		my_css.free(ptr);
}

/* For Acceleration API: Flush FW (shared buffer pointer) arguments */
void
sh_css_flush(struct ia_css_acc_fw *fw)
{
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "sh_css_flush() enter:\n");
	if ((fw != NULL) && (my_css.flush != NULL))
		my_css.flush(fw);
}

/* Mapping sp threads. Currently, this is done when a stream is created and
 * pipelines are ready to be converted to sp pipelines. Be careful if you are
 * doing it from stream_create since we could run out of sp threads due to
 * allocation on inactive pipelines. */
static enum ia_css_err
map_sp_threads(struct ia_css_stream *stream, bool map)
{
	struct ia_css_pipe *main_pipe = NULL;
	struct ia_css_pipe *copy_pipe = NULL, *capture_pipe = NULL;
	enum ia_css_err err = IA_CSS_SUCCESS;
	enum ia_css_pipe_id pipe_id;

	assert(stream != NULL);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"map_sp_threads() enter: map = %d\n", map);

	main_pipe 	= stream->last_pipe;
	pipe_id 	= main_pipe->mode;

	ia_css_pipeline_map(main_pipe->pipe_num, map);

	switch (pipe_id) {
	case IA_CSS_PIPE_ID_PREVIEW:
		copy_pipe    = main_pipe->pipe_settings.preview.copy_pipe;
		capture_pipe = main_pipe->pipe_settings.preview.capture_pipe;
		break;

	case IA_CSS_PIPE_ID_VIDEO:
		copy_pipe    = main_pipe->pipe_settings.video.copy_pipe;
		capture_pipe = main_pipe->pipe_settings.video.capture_pipe;
		break;

	case IA_CSS_PIPE_ID_CAPTURE:
	case IA_CSS_PIPE_ID_ACC:
	default:
		break;
	}

	if(capture_pipe) {
		ia_css_pipeline_map(capture_pipe->pipe_num, map);
	}

	/* Firmware expects copy pipe to be the last pipe mapped. (if needed) */
	if(copy_pipe) {
		ia_css_pipeline_map(copy_pipe->pipe_num, map);
	}

	return err;
}

/* creates a host pipeline skeleton for all pipes in a stream. Called during
 * stream_create. */
static enum ia_css_err
create_host_pipeline_structure(struct ia_css_stream *stream)
{
	struct ia_css_pipe *copy_pipe = NULL, *capture_pipe = NULL;
	enum ia_css_pipe_id pipe_id;
	struct ia_css_pipe *main_pipe = NULL;
	enum ia_css_err err = IA_CSS_SUCCESS;

	assert(stream != NULL);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"create_host_pipeline_structure() enter:\n");

	main_pipe 	= stream->last_pipe;
	assert(main_pipe != NULL);

	pipe_id 	= main_pipe->mode;

#if !defined(USE_INPUT_SYSTEM_VERSION_2401)
	/* Standalone Capture pipe cannot work with continuous capture. */
	if((pipe_id == IA_CSS_PIPE_ID_CAPTURE) && (stream->num_pipes == 1)) {
		if(!stream->config.online &&
			!main_pipe->pipe_settings.capture.copy_binary.info) {
			goto ERR;
		}
	}
#endif

	switch (pipe_id) {
	case IA_CSS_PIPE_ID_PREVIEW:
		copy_pipe    = main_pipe->pipe_settings.preview.copy_pipe;
		capture_pipe = main_pipe->pipe_settings.preview.capture_pipe;
		err = ia_css_pipeline_create(&main_pipe->pipeline, main_pipe->mode, main_pipe->pipe_num);
		break;

	case IA_CSS_PIPE_ID_VIDEO:
		copy_pipe    = main_pipe->pipe_settings.video.copy_pipe;
		capture_pipe = main_pipe->pipe_settings.video.capture_pipe;
		err = ia_css_pipeline_create(&main_pipe->pipeline, main_pipe->mode, main_pipe->pipe_num);
		break;

	case IA_CSS_PIPE_ID_CAPTURE:
		capture_pipe = main_pipe;
		break;

	case IA_CSS_PIPE_ID_YUVPP:
		err = ia_css_pipeline_create(&main_pipe->pipeline, main_pipe->mode,
						main_pipe->pipe_num);
		break;

	case IA_CSS_PIPE_ID_ACC:
		err = ia_css_pipeline_create(&main_pipe->pipeline, main_pipe->mode, main_pipe->pipe_num);
		break;

	default:
		err = IA_CSS_ERR_INVALID_ARGUMENTS;
	}

	if ((IA_CSS_SUCCESS == err) && copy_pipe) {
		err = ia_css_pipeline_create(&copy_pipe->pipeline, copy_pipe->mode, copy_pipe->pipe_num);
	}

	if ((IA_CSS_SUCCESS == err) && capture_pipe) {
		err = ia_css_pipeline_create(&capture_pipe->pipeline, capture_pipe->mode, capture_pipe->pipe_num);
	}

#if !defined(USE_INPUT_SYSTEM_VERSION_2401)
ERR:
#endif

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"create_host_pipeline_structure() leave: return (%d)\n", err);
	return err;
}

/* creates a host pipeline for all pipes in a stream. Called during
 * stream_start. */
static enum ia_css_err
create_host_pipeline(struct ia_css_stream *stream)
{
	struct ia_css_pipe *copy_pipe = NULL, *capture_pipe = NULL;
	enum ia_css_pipe_id pipe_id;
	struct ia_css_pipe *main_pipe = NULL;
	enum ia_css_err err = IA_CSS_SUCCESS;
    unsigned max_input_width = 0;

	assert(stream != NULL);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"create_host_pipeline() enter:\n");

	main_pipe 	= stream->last_pipe;
	pipe_id 	= main_pipe->mode;

#if !defined(USE_INPUT_SYSTEM_VERSION_2401)
	/* Standalone Capture pipe cannot work with continuous capture. */
	if((pipe_id == IA_CSS_PIPE_ID_CAPTURE) && (stream->num_pipes == 1)) {
		if (!stream->config.online &&
		    !main_pipe->pipe_settings.capture.copy_binary.info) {
			err = IA_CSS_ERR_INVALID_ARGUMENTS;
			goto ERR;
		}
	}
#endif
	/* No continuous frame allocation for capture pipe. It uses the
	 * "main" pipe's frames. */
	if((pipe_id == IA_CSS_PIPE_ID_PREVIEW) ||
	   (pipe_id == IA_CSS_PIPE_ID_VIDEO)) {
		if(stream->config.continuous || pipe_id == IA_CSS_PIPE_ID_PREVIEW) {
			err = alloc_continuous_frames(main_pipe, true);
			if (err != IA_CSS_SUCCESS)
				goto ERR;
		}

#ifdef HRT_CSIM
		if(main_pipe->continuous_frames[0])
			ia_css_frame_zero(main_pipe->continuous_frames[0]);
#endif
	}

#if defined(USE_INPUT_SYSTEM_VERSION_2) || defined(USE_INPUT_SYSTEM_VERSION_2401)
	if((pipe_id != IA_CSS_PIPE_ID_ACC) &&
	   (main_pipe->config.mode != IA_CSS_PIPE_MODE_COPY)) {
		err = allocate_mipi_frames(main_pipe);
		if (err != IA_CSS_SUCCESS)
			goto ERR;
	}
#endif

	switch (pipe_id) {
	case IA_CSS_PIPE_ID_PREVIEW:
		copy_pipe    = main_pipe->pipe_settings.preview.copy_pipe;
		capture_pipe = main_pipe->pipe_settings.preview.capture_pipe;
		max_input_width =
			main_pipe->pipe_settings.preview.preview_binary.info->sp.max_input_width;

		err = create_host_preview_pipeline(main_pipe);
		if (err != IA_CSS_SUCCESS)
			goto ERR;

		break;

	case IA_CSS_PIPE_ID_VIDEO:
		copy_pipe    = main_pipe->pipe_settings.video.copy_pipe;
		capture_pipe = main_pipe->pipe_settings.video.capture_pipe;
		max_input_width =
			main_pipe->pipe_settings.video.video_binary.info->sp.max_input_width;

		err = create_host_video_pipeline(main_pipe);
		if (err != IA_CSS_SUCCESS)
			goto ERR;

		break;

	case IA_CSS_PIPE_ID_CAPTURE:
		capture_pipe = main_pipe;

		break;

	case IA_CSS_PIPE_ID_YUVPP:
		err = create_host_yuvpp_pipeline(main_pipe);
		if (err != IA_CSS_SUCCESS)
			goto ERR;

		break;

	case IA_CSS_PIPE_ID_ACC:
		err = create_host_acc_pipeline(main_pipe);
		if (err != IA_CSS_SUCCESS)
			goto ERR;

		break;
	default:
		err = IA_CSS_ERR_INVALID_ARGUMENTS;
	}
	if (err != IA_CSS_SUCCESS)
		goto ERR;

	if(copy_pipe) {
		err = create_host_copy_pipeline(copy_pipe, max_input_width,
					  main_pipe->continuous_frames[0]);
		if (err != IA_CSS_SUCCESS)
			goto ERR;
	}

	if(capture_pipe) {
		err = create_host_capture_pipeline(capture_pipe);
		if (err != IA_CSS_SUCCESS)
			goto ERR;
	}

ERR:
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"create_host_pipeline() leave: return (%d)\n", err);
	return err;
}

static enum ia_css_err
init_pipe_defaults(enum ia_css_pipe_mode mode,
	       struct ia_css_pipe *pipe,
	       bool copy_pipe)
{
	static struct ia_css_pipe default_pipe = DEFAULT_PIPE;
	static struct ia_css_preview_settings prev  = DEFAULT_PREVIEW_SETTINGS;
	static struct ia_css_capture_settings capt  = DEFAULT_CAPTURE_SETTINGS;
	static struct ia_css_video_settings   video = DEFAULT_VIDEO_SETTINGS;
	static struct ia_css_yuvpp_settings   yuvpp = DEFAULT_YUVPP_SETTINGS;

	assert(pipe != NULL);

	/* Initialize pipe to pre-defined defaults */
	*pipe = default_pipe;

	/* TODO: JB should not be needed, but temporary backward reference */
	switch (mode) {
	case IA_CSS_PIPE_MODE_PREVIEW:
		pipe->mode = IA_CSS_PIPE_ID_PREVIEW;
		pipe->pipe_settings.preview = prev;
		break;
	case IA_CSS_PIPE_MODE_CAPTURE:
		if (copy_pipe) {
			pipe->mode = IA_CSS_PIPE_ID_COPY;
		} else {
			pipe->mode = IA_CSS_PIPE_ID_CAPTURE;
		}
		pipe->pipe_settings.capture = capt;
		break;
	case IA_CSS_PIPE_MODE_VIDEO:
		pipe->mode = IA_CSS_PIPE_ID_VIDEO;
		pipe->pipe_settings.video = video;
		break;
	case IA_CSS_PIPE_MODE_ACC:
		pipe->mode = IA_CSS_PIPE_ID_ACC;
		break;
	case IA_CSS_PIPE_MODE_COPY:
		pipe->mode = IA_CSS_PIPE_ID_CAPTURE;
		break;
	case IA_CSS_PIPE_MODE_YUVPP:
		pipe->mode = IA_CSS_PIPE_ID_YUVPP;
		pipe->pipe_settings.yuvpp = yuvpp;
		break;
	default:
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	}

	return IA_CSS_SUCCESS;
}

static void
pipe_global_init(void)
{
	uint8_t i;

	my_css.pipe_counter = 0;
	for (i = 0; i < IA_CSS_PIPELINE_NUM_MAX; i++) {
		my_css.all_pipes[i] = NULL;
	}
}

static uint8_t
pipe_generate_pipe_num(const struct ia_css_pipe *pipe)
{
	uint8_t i;
	uint8_t pipe_num = ~0; /* UINT8_MAX; but Linux does not have this macro */

	assert(pipe != NULL);
	/* Assign a new pipe_num .... search for empty place */
	for (i = 0; i < IA_CSS_PIPELINE_NUM_MAX; i++) {
		if (my_css.all_pipes[i] == NULL) {
			/*position is reserved */
			my_css.all_pipes[i] = (struct ia_css_pipe *)pipe;
			pipe_num = i;
			break;
		}
	}
	assert(pipe_num != (uint8_t)~0); /* UINT8_MAX; but Linux does not have this macro */
	my_css.pipe_counter++;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"pipe_generate_pipe_num (%d)\n", pipe_num);
	return pipe_num;
}

static void
pipe_release_pipe_num(unsigned int pipe_num)
{
	my_css.all_pipes[pipe_num] = NULL;
	my_css.pipe_counter--;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"pipe_release_pipe_num (%d)\n", pipe_num);
}

static enum ia_css_err
create_pipe(enum ia_css_pipe_mode mode,
	    struct ia_css_pipe **pipe,
	    bool copy_pipe)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct ia_css_pipe *me = sh_css_malloc(sizeof(*me));

	assert(pipe != NULL);

	if (!me)
		return IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;

	err = init_pipe_defaults(mode, me, copy_pipe);
	if (err != IA_CSS_SUCCESS) {
		sh_css_free(me);
		return IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
	}

	me->pipe_num = pipe_generate_pipe_num(me);
	*pipe = me;
	return IA_CSS_SUCCESS;
}

static struct ia_css_pipe *
find_pipe_by_num(uint8_t pipe_num)
{
	unsigned int i;
	for (i = 0; i < IA_CSS_PIPELINE_NUM_MAX; i++){
		if (my_css.all_pipes[i] &&
				ia_css_pipe_get_pipe_num(my_css.all_pipes[i]) == pipe_num) {
			return my_css.all_pipes[i];
		}
	}
	return NULL;
}

static void sh_css_pipe_free_acc_binaries (
    struct ia_css_pipe *pipe)
{
	struct ia_css_pipeline *pipeline;
	struct ia_css_pipeline_stage *stage;

	assert(pipe != NULL);
	pipeline = &pipe->pipeline;

	/* loop through the stages and unload them */
	for (stage = pipeline->stages; stage; stage = stage->next) {
		struct ia_css_fw_info *firmware = (struct ia_css_fw_info *)
						stage->firmware;
		if (firmware)
			ia_css_pipe_unload_extension(pipe, firmware);
	}
}

enum ia_css_err
ia_css_pipe_destroy(struct ia_css_pipe *pipe)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_pipe_destroy() enter\n");

	if (pipe == NULL) return IA_CSS_ERR_INVALID_ARGUMENTS;

	if (pipe->stream != NULL) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_pipe_destroy(): "
							"ia_css_stream_destroy not called!\n");
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	}

	switch (pipe->config.mode) {
	case IA_CSS_PIPE_MODE_PREVIEW:
		/* need to take into account that this function is also called
		   on the internal copy pipe */
		if (pipe->mode == IA_CSS_PIPE_ID_PREVIEW) {
			ia_css_frame_free_multiple(NUM_CONTINUOUS_FRAMES,
					pipe->continuous_frames);
			ia_css_metadata_free_multiple(NUM_CONTINUOUS_FRAMES,
					pipe->cont_md_buffers);
			if (pipe->pipe_settings.preview.copy_pipe) {
				err = ia_css_pipe_destroy(pipe->pipe_settings.preview.copy_pipe);
				ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_pipe_destroy(): "
					"destroyed internal copy pipe err=%d\n", err);
			}
		}
		break;
	case IA_CSS_PIPE_MODE_VIDEO:
		if (pipe->mode == IA_CSS_PIPE_ID_VIDEO) {
			ia_css_frame_free_multiple(NUM_CONTINUOUS_FRAMES,
				pipe->continuous_frames);
			ia_css_metadata_free_multiple(NUM_CONTINUOUS_FRAMES,
					pipe->cont_md_buffers);
			if (pipe->pipe_settings.video.copy_pipe) {
				err = ia_css_pipe_destroy(pipe->pipe_settings.video.copy_pipe);
				ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_pipe_destroy(): "
					"destroyed internal copy pipe err=%d\n", err);
			}
		}
		ia_css_frame_free_multiple(NUM_VIDEO_TNR_FRAMES, pipe->pipe_settings.video.tnr_frames);
		ia_css_frame_free_multiple((pipe->dvs_frame_delay + 1), pipe->pipe_settings.video.delay_frames);
		break;
	case IA_CSS_PIPE_MODE_CAPTURE:
#if 0
		/* Do not destroy, these are shared with preview */
		ia_css_frame_free_multiple(NUM_CONTINUOUS_FRAMES,
				pipe->pipe_settings.capture.continuous_frames);
#endif
		break;
	case IA_CSS_PIPE_MODE_ACC:
		sh_css_pipe_free_acc_binaries(pipe);
		break;
	case IA_CSS_PIPE_MODE_COPY:
		break;
	case IA_CSS_PIPE_MODE_YUVPP:
		break;
	}

	my_css.active_pipes[ia_css_pipe_get_pipe_num(pipe)] = NULL;
	sh_css_pipe_free_shading_table(pipe);

	ia_css_pipeline_destroy(&pipe->pipeline);
	pipe_release_pipe_num(ia_css_pipe_get_pipe_num(pipe));

	/* Temporarily, not every sh_css_pipe has an acc_extension. */
	if (pipe->config.acc_extension) {
		ia_css_pipe_unload_extension(pipe, pipe->config.acc_extension);
	}
	sh_css_free(pipe);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_pipe_destroy() exit, err=%d\n", err);
	return err;
}

void
ia_css_uninit(void)
{
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_uninit() enter: void\n");
#if WITH_PC_MONITORING
	sh_css_print("PC_MONITORING: %s() -- started\n", __func__);
	print_pc_histogram();
#endif
	/* TODO: JB: implement decent check and handling of freeing mipi frames */
	//assert(ref_count_mipi_allocation == 0); //mipi frames are not freed
	/* cleanup generic data */
	sh_css_params_uninit();
	ia_css_refcount_uninit();

	ia_css_rmgr_uninit();

#if !defined(HAS_NO_INPUT_FORMATTER)
	/* needed for reprogramming the inputformatter after power cycle of css */
	ifmtr_set_if_blocking_mode_reset = true;
#endif

	if (fw_explicitly_loaded == false) {
		ia_css_unload_firmware(false);
	}
	ia_css_spctrl_unload_fw(SP0_ID);
#if defined(IS_ISP_2500_SYSTEM)
	ia_css_spctrl_unload_fw(SP1_ID);
#endif

	sh_css_sp_set_sp_running(false);

#if defined(USE_INPUT_SYSTEM_VERSION_2) || defined(USE_INPUT_SYSTEM_VERSION_2401)
	/* check and free any remaining mipi frames */
	free_mipi_frames(NULL, true);
#endif

	sh_css_sp_reset_global_vars();

#if !defined(HAS_NO_INPUT_SYSTEM)
	ia_css_isys_uninit();
#endif

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_uninit() leave: return_void\n");
}

static unsigned int translate_sw_interrupt(unsigned value)
{
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "translate_sw_interrupt() enter:\n");
	/* previous versions of sp would put info in the upper word
	   better safe than sorry so mask that away
	*/
	value = value & 0xffff;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "translate_sw_interrupt() leave: return %d\n", value);
	return value;
}

static unsigned int translate_sw_interrupt1(void)
{
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "translate_sw_interrupt1() enter:\n");
	return translate_sw_interrupt(sh_css_get_sw_interrupt_value(1));
}

#if 0
static unsigned int translate_sw_interrupt2(void)
{
	/* By smart coding the flag/bits in value (on the SP side),
	 * no translation is required. The returned value can be
	 * binary ORed with existing interrupt info
	 * (it is compatible with enum ia_css_irq_info)
	 */
/* MW: No smart coding required, we should just keep interrupt info
   and local context info separated */
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "translate_sw_interrupt2() enter:\n");
	return translate_sw_interrupt(sh_css_get_sw_interrupt_value(2));
}
#endif

/* Deprecated, this is an HRT backend function (memory_access.h) */
static void
sh_css_mmu_set_page_table_base_index(hrt_data base_index)
{
	int i;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "sh_css_mmu_set_page_table_base_index() enter: base_index=0x%08x\n",base_index);
	my_css.page_table_base_index = base_index;
	for (i = 0; i < (int)N_MMU_ID; i++) {
		mmu_ID_t mmu_id = (mmu_ID_t)i;
		mmu_set_page_table_base_index(mmu_id, base_index);
		mmu_invalidate_cache(mmu_id);
	}
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "sh_css_mmu_set_page_table_base_index() leave: return_void\n");
}

void
ia_css_mmu_invalidate_cache(void)
{
	const struct ia_css_fw_info *fw = &sh_css_sp_fw;
	unsigned int HIVE_ADDR_ia_css_dmaproxy_sp_invalidate_tlb;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_mmu_invalidate_cache() enter\n");

	/* if the SP is not running we should not access its dmem */
	if (sh_css_sp_is_running())
	{
		HIVE_ADDR_ia_css_dmaproxy_sp_invalidate_tlb = fw->info.sp.invalidate_tlb;

		(void)HIVE_ADDR_ia_css_dmaproxy_sp_invalidate_tlb; /* Suppres warnings in CRUN */

		sp_dmem_store_uint32(SP0_ID,
			(unsigned int)sp_address_of(ia_css_dmaproxy_sp_invalidate_tlb),
			true);
	}
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_mmu_invalidate_cache() leave\n");
}

#if defined(HAS_IRQ_MAP_VERSION_1) || defined(HAS_IRQ_MAP_VERSION_1_DEMO)
enum ia_css_err ia_css_irq_translate(
	unsigned int *irq_infos)
{
	virq_id_t	irq;
	enum hrt_isp_css_irq_status status = hrt_isp_css_irq_status_more_irqs;
	unsigned int infos = 0;

/* irq_infos can be NULL, but that would make the function useless */
/* assert(irq_infos != NULL); */
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_irq_translate() enter: irq_infos=%p\n",irq_infos);

	while (status == hrt_isp_css_irq_status_more_irqs) {
		status = virq_get_channel_id(&irq);
		if (status == hrt_isp_css_irq_status_error)
			return IA_CSS_ERR_INTERNAL_ERROR;

#if WITH_PC_MONITORING
		sh_css_print("PC_MONITORING: %s() irq = %d, "
			     "sh_binary_running set to 0\n", __func__, irq);
		sh_binary_running = 0 ;
#endif

		switch (irq) {
		case virq_sp:
			infos |= IA_CSS_IRQ_INFO_EVENTS_READY;
			break;
		case virq_isp:
#ifdef HRT_CSIM
			/* Enable IRQ which signals that ISP goes to idle
			 * to get statistics for each binary */
			infos |= IA_CSS_IRQ_INFO_ISP_BINARY_STATISTICS_READY;
#endif
			break;
		case virq_isys_csi:
			/* css rx interrupt, read error bits from css rx */
			infos |= IA_CSS_IRQ_INFO_CSS_RECEIVER_ERROR;
			break;
		case virq_isys_fifo_full:
			infos |=
			    IA_CSS_IRQ_INFO_CSS_RECEIVER_FIFO_OVERFLOW;
			break;
		case virq_isys_sof:
			infos |= IA_CSS_IRQ_INFO_CSS_RECEIVER_SOF;
			break;
		case virq_isys_eof:
			infos |= IA_CSS_IRQ_INFO_CSS_RECEIVER_EOF;
			break;
/* Temporarily removed, until we have a seperate flag for FRAME_READY irq */
#if 0
/* hmm, an interrupt mask, why would we have that ? */
		case virq_isys_sol:
			infos |= IA_CSS_IRQ_INFO_CSS_RECEIVER_SOL;
			break;
#endif
		case virq_isys_eol:
			infos |= IA_CSS_IRQ_INFO_CSS_RECEIVER_EOL;
			break;
/*
 * MW: The 2300 demo system does not have a receiver, and it
 * does not have the following three IRQ channels defined
 */
#if defined(HAS_IRQ_MAP_VERSION_1)
		case virq_ifmt_sideband_changed:
			infos |=
			    IA_CSS_IRQ_INFO_CSS_RECEIVER_SIDEBAND_CHANGED;
			break;
		case virq_gen_short_0:
			infos |= IA_CSS_IRQ_INFO_CSS_RECEIVER_GEN_SHORT_0;
			break;
		case virq_gen_short_1:
			infos |= IA_CSS_IRQ_INFO_CSS_RECEIVER_GEN_SHORT_1;
			break;
#endif
		case virq_ifmt0_id:
			infos |= IA_CSS_IRQ_INFO_IF_PRIM_ERROR;
			break;
		case virq_ifmt1_id:
			infos |= IA_CSS_IRQ_INFO_IF_PRIM_B_ERROR;
			break;
		case virq_ifmt2_id:
			infos |= IA_CSS_IRQ_INFO_IF_SEC_ERROR;
			break;
		case virq_ifmt3_id:
			infos |= IA_CSS_IRQ_INFO_STREAM_TO_MEM_ERROR;
			break;
		case virq_sw_pin_0:
			infos |= IA_CSS_IRQ_INFO_SW_0;
			break;
		case virq_sw_pin_1:
			infos |= translate_sw_interrupt1();
			/* pqiao TODO: also assumption here */
			break;
		default:
			break;
		}
	}

	if (irq_infos)
		*irq_infos = infos;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_irq_translate() "
		"leave: irq_infos=%p\n", infos);

	return IA_CSS_SUCCESS;
}

enum ia_css_err
ia_css_irq_enable(enum ia_css_irq_info info,
		  bool enable)
{
	virq_id_t	irq = N_virq_id;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_irq_enable() enter: info=%d, enable=%d\n",info,enable);

	switch (info) {
	case IA_CSS_IRQ_INFO_CSS_RECEIVER_ERROR:
		irq = virq_isys_csi;
		break;
	case IA_CSS_IRQ_INFO_CSS_RECEIVER_FIFO_OVERFLOW:
		irq = virq_isys_fifo_full;
		break;
	case IA_CSS_IRQ_INFO_CSS_RECEIVER_SOF:
		irq = virq_isys_sof;
		break;
	case IA_CSS_IRQ_INFO_CSS_RECEIVER_EOF:
		irq = virq_isys_eof;
		break;
/* Temporarily removed, until we have a seperate flag for FRAME_READY irq */
#if 0
/* hmm, an interrupt mask, why would we have that ? */
	case IA_CSS_IRQ_INFO_CSS_RECEIVER_SOL:
		irq = virq_isys_sol;
		break;
#endif
	case IA_CSS_IRQ_INFO_CSS_RECEIVER_EOL:
		irq = virq_isys_eol;
		break;
#if defined(HAS_IRQ_MAP_VERSION_1)
	case IA_CSS_IRQ_INFO_CSS_RECEIVER_SIDEBAND_CHANGED:
		irq = virq_ifmt_sideband_changed;
		break;
	case IA_CSS_IRQ_INFO_CSS_RECEIVER_GEN_SHORT_0:
		irq = virq_gen_short_0;
		break;
	case IA_CSS_IRQ_INFO_CSS_RECEIVER_GEN_SHORT_1:
		irq = virq_gen_short_1;
		break;
#endif
	case IA_CSS_IRQ_INFO_IF_PRIM_ERROR:
		irq = virq_ifmt0_id;
		break;
	case IA_CSS_IRQ_INFO_IF_PRIM_B_ERROR:
		irq = virq_ifmt1_id;
		break;
	case IA_CSS_IRQ_INFO_IF_SEC_ERROR:
		irq = virq_ifmt2_id;
		break;
	case IA_CSS_IRQ_INFO_STREAM_TO_MEM_ERROR:
		irq = virq_ifmt3_id;
		break;
	case IA_CSS_IRQ_INFO_SW_0:
		irq = virq_sw_pin_0;
		break;
	case IA_CSS_IRQ_INFO_SW_1:
		irq = virq_sw_pin_1;
		break;
	case IA_CSS_IRQ_INFO_SW_2:
		irq = virq_sw_pin_2;
		break;
	default:
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_irq_enable() leave: return_err=%d\n",IA_CSS_ERR_INVALID_ARGUMENTS);
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	}

	cnd_virq_enable_channel(irq, enable);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_irq_enable() leave: return_err=%d\n",IA_CSS_SUCCESS);
	return IA_CSS_SUCCESS;
}

#elif defined(HAS_IRQ_MAP_VERSION_2)

enum ia_css_err ia_css_irq_translate(
	unsigned int *irq_infos)
{
	virq_id_t	irq;
	enum hrt_isp_css_irq_status status = hrt_isp_css_irq_status_more_irqs;
	unsigned int infos = 0;

/* irq_infos can be NULL, but that would make the function useless */
/* assert(irq_infos != NULL); */
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_irq_translate() enter: irq_infos=%p\n",irq_infos);

	while (status == hrt_isp_css_irq_status_more_irqs) {
		status = virq_get_channel_id(&irq);
		if (status == hrt_isp_css_irq_status_error)
			return IA_CSS_ERR_INTERNAL_ERROR;

#if WITH_PC_MONITORING
		sh_css_print("PC_MONITORING: %s() irq = %d, "
			     "sh_binary_running set to 0\n", __func__, irq);
		sh_binary_running = 0 ;
#endif

		switch (irq) {
#if defined(IS_ISP_2500_SYSTEM)
		case virq_sp0:
#else
		case virq_sp:
#endif
			infos |= IA_CSS_IRQ_INFO_EVENTS_READY;
			break;
		case virq_isp:
#ifdef HRT_CSIM
			/* Enable IRQ which signals that ISP goes to idle
			 * to get statistics for each binary */
			infos |= IA_CSS_IRQ_INFO_ISP_BINARY_STATISTICS_READY;
#endif
			break;
#if !defined(HAS_NO_INPUT_SYSTEM)
		case virq_isys_sof:
			infos |= IA_CSS_IRQ_INFO_CSS_RECEIVER_SOF;
			break;
		case virq_isys_eof:
			infos |= IA_CSS_IRQ_INFO_CSS_RECEIVER_EOF;
			break;
		case virq_isys_csi:
			infos |= IA_CSS_IRQ_INFO_INPUT_SYSTEM_ERROR;
			break;
#endif
#if !defined(HAS_NO_INPUT_FORMATTER)
		case virq_ifmt0_id:
			infos |= IA_CSS_IRQ_INFO_IF_ERROR;
			break;
#endif
		case virq_dma:
			infos |= IA_CSS_IRQ_INFO_DMA_ERROR;
			break;
		case virq_sw_pin_0:
			infos |= IA_CSS_IRQ_INFO_SW_0;
			break;
		case virq_sw_pin_1:
			infos |= translate_sw_interrupt1();
			/* pqiao TODO: also assumption here */
			break;
		default:
			break;
		}
	}

	if (irq_infos)
		*irq_infos = infos;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_irq_translate() "
		"leave: irq_infos=%p\n", infos);

	return IA_CSS_SUCCESS;
}

enum ia_css_err ia_css_irq_enable(
	enum ia_css_irq_info info,
	bool enable)
{
	virq_id_t	irq = N_virq_id;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_irq_enable() enter: info=%d, enable=%d\n",info,enable);

	switch (info) {
#if !defined(HAS_NO_INPUT_FORMATTER)
	case IA_CSS_IRQ_INFO_CSS_RECEIVER_SOF:
		irq = virq_isys_sof;
		break;
	case IA_CSS_IRQ_INFO_CSS_RECEIVER_EOF:
		irq = virq_isys_eof;
		break;
	case IA_CSS_IRQ_INFO_INPUT_SYSTEM_ERROR:
		irq = virq_isys_csi;
		break;
#endif
#if !defined(HAS_NO_INPUT_FORMATTER)
	case IA_CSS_IRQ_INFO_IF_ERROR:
		irq = virq_ifmt0_id;
		break;
#endif
	case IA_CSS_IRQ_INFO_DMA_ERROR:
		irq = virq_dma;
		break;
	case IA_CSS_IRQ_INFO_SW_0:
		irq = virq_sw_pin_0;
		break;
	case IA_CSS_IRQ_INFO_SW_1:
		irq = virq_sw_pin_1;
		break;
	default:
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_irq_enable() leave: return_err=%d\n",IA_CSS_ERR_INVALID_ARGUMENTS);
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	}

	cnd_virq_enable_channel(irq, enable);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_irq_enable() leave: return_err=%d\n",IA_CSS_SUCCESS);
	return IA_CSS_SUCCESS;
}

#else
#error "sh_css.c: IRQ MAP must be one of \
	{IRQ_MAP_VERSION_1, IRQ_MAP_VERSION_1_DEMO, IRQ_MAP_VERSION_2}"
#endif

static unsigned int
sh_css_get_sw_interrupt_value(unsigned int irq)
{
	unsigned int irq_value;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "sh_css_get_sw_interrupt_value() enter: irq=%d\n",irq);
	irq_value = sh_css_sp_get_sw_interrupt_value(irq);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "sh_css_get_sw_interrupt_value() leave: irq_value=%d\n",irq_value);
	return irq_value;
}

#if 0
/* Disabled because it is currently unused. */
static void
sh_css_pipe_get_extra_pixels_count(const struct sh_css_pipe *pipe,
				   struct ia_css_resolution *extra)
{
	int rows = SH_CSS_MAX_LEFT_CROPPING,
	    cols = SH_CSS_MAX_LEFT_CROPPING;

	assert(pipe != NULL);
	assert(extra != NULL);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "sh_css_pipe_get_extra_pixels_count() enter: void\n");

	if (ia_css_ifmtr_lines_needed_for_bayer_order(&pipe->stream->config))
		rows += 2;

	if (ia_css_ifmtr_columns_needed_for_bayer_order(&pipe->stream->config))
		cols  += 2;

	extra->width  = cols;
	extra->height = rows;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"sh_css_pipe_get_extra_pixels_count() leave: extra_rows=%d, extra_cols=%d\n",
		rows, cols);
}
#endif

/* configure and load the copy binary, the next binary is used to
   determine whether the copy binary needs to do left padding. */
static enum ia_css_err load_copy_binary(
	struct ia_css_pipe *pipe,
	struct ia_css_binary *copy_binary,
	struct ia_css_binary *next_binary)
{
	struct ia_css_frame_info copy_out_info, copy_in_info, copy_vf_info;
	unsigned int left_padding;
	enum ia_css_err err;
	struct ia_css_binary_descr copy_descr;

	/* next_binary can be NULL */
	assert(pipe != NULL);
	assert(copy_binary != NULL);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"load_copy_binary() enter:\n");

	if (next_binary != NULL) {
		copy_out_info = next_binary->in_frame_info;
		left_padding = next_binary->left_padding;
	} else {
		copy_out_info = pipe->output_info[0];
		copy_vf_info = pipe->vf_output_info[0];
		ia_css_frame_info_set_format(&copy_vf_info, IA_CSS_FRAME_FORMAT_YUV_LINE);
		left_padding = 0;
	}

	ia_css_pipe_get_copy_binarydesc(pipe, &copy_descr,
		&copy_in_info, &copy_out_info, (next_binary != NULL) ? NULL : NULL/*TODO: &copy_vf_info*/);
	err = ia_css_binary_find(&copy_descr, copy_binary);
	if (err != IA_CSS_SUCCESS)
		return err;
	copy_binary->left_padding = left_padding;
	return IA_CSS_SUCCESS;
}

static enum ia_css_err
alloc_continuous_frames(
	struct ia_css_pipe *pipe, bool init_time)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct ia_css_frame_info ref_info;
	enum ia_css_pipe_id pipe_id;
	bool continuous;
	unsigned int i, idx;
	unsigned int num_frames;
	unsigned int left_cropping = 0, top_cropping;
	uint8_t raw_binning = 0;
	struct ia_css_pipe *capture_pipe = NULL;

	assert(pipe != NULL);
	assert(pipe->stream != NULL);

	pipe_id = pipe->mode;
	continuous = pipe->stream->config.continuous;

	if (continuous) {
		if (init_time) {
			num_frames = pipe->stream->config.init_num_cont_raw_buf;
			pipe->stream->continuous_pipe = pipe;
		} else
			num_frames = pipe->stream->config.target_num_cont_raw_buf;
	} else {
	    num_frames = NUM_ONLINE_INIT_CONTINUOUS_FRAMES;
	}

	if (pipe_id == IA_CSS_PIPE_ID_PREVIEW) {
		left_cropping = pipe->pipe_settings.preview.preview_binary.info->sp.left_cropping;
		top_cropping = pipe->pipe_settings.preview.preview_binary.info->sp.top_cropping;
		ref_info = pipe->pipe_settings.preview.preview_binary.in_frame_info;
		raw_binning = pipe->pipe_settings.preview.preview_binary.info->sp.enable.raw_binning;
	} else if (pipe_id == IA_CSS_PIPE_ID_VIDEO) {
		left_cropping = pipe->pipe_settings.video.video_binary.info->sp.left_cropping;
		top_cropping = pipe->pipe_settings.video.video_binary.info->sp.top_cropping;
		ref_info = pipe->pipe_settings.video.video_binary.in_frame_info;
		raw_binning = pipe->pipe_settings.video.video_binary.info->sp.enable.raw_binning;
	}
	else {
		/* should not happen */
		assert(false);
		return IA_CSS_ERR_INTERNAL_ERROR;
	}

#if defined(USE_INPUT_SYSTEM_VERSION_2401)
	/* [Must Revert]
	 * The following changes are interim updates in order to
	 * have an immediate solution.  The formal solutions will
	 * be forthcoming on follow-up updates. These changes
	 * are intended to be reverted.
	 */

	/* Ensure padded width is aligned for 2401 */
	ref_info.padded_width = CEIL_MUL(ref_info.res.width, 2 * ISP_VEC_NELEMS);

	/* avoid memguard error */
	ref_info.res.height += 4;
#endif

#if !defined(HAS_NO_PACKED_RAW_PIXELS)
	if (pipe->stream->config.pack_raw_pixels) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
			"alloc_continuous_frames() IA_CSS_FRAME_FORMAT_RAW_PACKED\n");
		ref_info.format = IA_CSS_FRAME_FORMAT_RAW_PACKED;
	} else
#endif
	{
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
			"alloc_continuous_frames() IA_CSS_FRAME_FORMAT_RAW\n");
		ref_info.format = IA_CSS_FRAME_FORMAT_RAW;
	}

	/* Write format back to binary */
	if (pipe_id == IA_CSS_PIPE_ID_PREVIEW) {
		pipe->pipe_settings.preview.preview_binary.in_frame_info.format = ref_info.format;
		capture_pipe = pipe->pipe_settings.preview.capture_pipe;
	} else if (pipe_id == IA_CSS_PIPE_ID_VIDEO) {
		pipe->pipe_settings.video.video_binary.in_frame_info.format = ref_info.format;
		capture_pipe = pipe->pipe_settings.video.capture_pipe;
	} else {
		/* should not happen */
		assert(false);
		return IA_CSS_ERR_INTERNAL_ERROR;
	}

	if (init_time)
	    idx = 0;
	else
		idx = pipe->stream->config.init_num_cont_raw_buf;

	for (i = idx; i < NUM_CONTINUOUS_FRAMES; i++) {
		/* free previous frame */
		if (pipe->continuous_frames[i]) {
			ia_css_frame_free(pipe->continuous_frames[i]);
			pipe->continuous_frames[i] = NULL;
		}
		/* free previous metadata buffer */
		ia_css_metadata_free(pipe->cont_md_buffers[i]);
		pipe->cont_md_buffers[i] = NULL;

		/* check if new frame needed */
		if (i < num_frames) {
			/* allocate new frame */
			err = ia_css_frame_allocate_from_info(
				&pipe->continuous_frames[i],
				&ref_info);
			if (err != IA_CSS_SUCCESS)
				return err;
			/* allocate metadata buffer */
			pipe->cont_md_buffers[i] = ia_css_metadata_allocate(
					&pipe->stream->info.metadata_info);
		}
	}
	return IA_CSS_SUCCESS;
}

enum ia_css_err
ia_css_alloc_continuous_frame_remain(struct ia_css_stream *stream)
{
	if (stream == NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	return alloc_continuous_frames(stream->continuous_pipe, false);
}

#if defined(USE_INPUT_SYSTEM_VERSION_2) || defined(USE_INPUT_SYSTEM_VERSION_2401)
/* start of MIPI functions */
static enum ia_css_err
allocate_mipi_frames(struct ia_css_pipe *pipe)
{
	enum ia_css_err err = IA_CSS_ERR_INTERNAL_ERROR;
	unsigned int i, j;
	struct ia_css_frame_info mipi_intermediate_info;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"allocate_mipi_frames(%p) enter:\n", pipe);

	assert(pipe != NULL);
	assert(pipe->stream != NULL);

#ifdef USE_INPUT_SYSTEM_VERSION_2401
	if (pipe->stream->config.online) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
			"allocate_mipi_frames(%p) exit: no buffers needed for 2401 pipe mode\n",
			pipe);
		return IA_CSS_SUCCESS;
	}

#endif
	if (pipe->stream->config.mode != IA_CSS_INPUT_MODE_BUFFERED_SENSOR) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
			"allocate_mipi_frames(%p) exit: no buffers needed for pipe mode\n",
			pipe);
		return IA_CSS_SUCCESS;
	}

	assert(my_css.size_mem_words != 0);
	if (my_css.size_mem_words == 0) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
			"allocate_mipi_frames(%p) exit: mipi frame size not specified\n",
			pipe);
		return err;
	}

	ref_count_mipi_allocation++;
	if (ref_count_mipi_allocation > 1) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
			"allocate_mipi_frames(%p) exit: already allocated\n",
			pipe);
		return IA_CSS_SUCCESS;
	}
	assert(ref_count_mipi_allocation == 1);

// This code needs to modified to allocate the MIPI frames in the correct normal way with a allocate from info
// by justin
	mipi_intermediate_info = pipe->pipe_settings.video.video_binary.internal_frame_info;
	mipi_intermediate_info.res.width = 0;
	mipi_intermediate_info.res.height = 0;
	// To indicate it is not (yet) valid format.
	mipi_intermediate_info.format = IA_CSS_FRAME_FORMAT_NUM;
	mipi_intermediate_info.padded_width = 0;
	mipi_intermediate_info.raw_bit_depth = 0;
	//mipi_intermediate_info.data_bytes = my_css.size_mem_words * HIVE_ISP_DDR_WORD_BYTES;
	//mipi_intermediate_info.contiguous = my_css.contiguous;
	// To indicate it is not valid frame.
	//mipi_intermediate_info.dynamic_data_index = SH_CSS_INVALID_FRAME_ID;

	for (i = 0; i < my_css.num_mipi_frames; i++) {
		/* If there is a previous frame available, reuse it */
		if (my_css.mipi_frames[i]) {
			err = IA_CSS_SUCCESS;
			continue;

		}
		/* check if new frame needed */
		if (i < my_css.num_mipi_frames) {
			/* allocate new frame */
			err = ia_css_frame_allocate_with_buffer_size(
				&my_css.mipi_frames[i],
				my_css.size_mem_words * HIVE_ISP_DDR_WORD_BYTES,
				my_css.contiguous);
			if (err != IA_CSS_SUCCESS) {
				for (j = 0; j < i; j++) {
					if (my_css.mipi_frames[j]) {
						ia_css_frame_free(my_css.mipi_frames[j]);
						my_css.mipi_frames[j] = NULL;
					}
				}
				return err;
			}
#ifdef HRT_CSIM
			ia_css_frame_zero(my_css.mipi_frames[i]);
#endif
		}
	}
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"allocate_mipi_frames(%p) exit:\n", pipe);

	return err;
}

static void
free_mipi_frames(struct ia_css_pipe *pipe, bool uninit)
{
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"free_mipi_frames(%p, %d) enter:\n", pipe, uninit);
	if (!uninit) {
		assert(pipe != NULL);
		assert(pipe->stream != NULL);

		if (pipe->stream->config.mode != IA_CSS_INPUT_MODE_BUFFERED_SENSOR) {
			ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
				"free_mipi_frames() exit: wrong mode\n");
			return;
		}

		assert(ref_count_mipi_allocation > 0);
		ref_count_mipi_allocation--;

		if (ref_count_mipi_allocation > 0) {
			ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
				"free_mipi_frames(%p, %d) exit: "
				"not last pipe (ref_count=%d):\n",
				pipe, uninit, ref_count_mipi_allocation);
			return;
		}
	}

	/* TODO: change into return error value instead of assert
	 * task is pending, disable for now
	 */
	//assert(ref_count_mipi_allocation == 0);
	ref_count_mipi_allocation = 0;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"free_mipi_frames(%p, %d) exit (deallocated):\n", pipe, uninit);
}


static void
send_mipi_frames(struct ia_css_pipe *pipe)
{
	unsigned int i;

	assert(pipe != NULL);
	assert(pipe->stream != NULL);

	/* multi stream video needs mipi buffers */
	if (pipe->stream->config.mode != IA_CSS_INPUT_MODE_BUFFERED_SENSOR)
		return;

	/* Hand-over the SP-internal mipi buffers */
	for (i = 0; i < my_css.num_mipi_frames; i++) {
		sh_css_update_host2sp_mipi_frame(i,
			my_css.mipi_frames[i]);
	}
	sh_css_update_host2sp_num_mipi_frames
		(my_css.num_mipi_frames);

	/**********************************
	 *
	 * Hack for Baytrail.
	 *
	 * AUTHOR: zhengjie.lu@intel.com
	 * TIME: 2013-01-19, 14:38.
	 * LOCATION: Santa Clara, U.S.A.
	 * COMMENT:
	 * Send an event to inform the SP
	 * that all MIPI frames are passed.
	 *
	 **********************************/
	{
		if (!sh_css_sp_is_running()) {
			/* SP is not running. The queues are not valid */
			return;
		}
		ia_css_bufq_enqueue_event(SP_SW_EVENT_ID_6,	/* the event ID  */
			0,				/* not used */
			0,				/* not used */
			0				/* not used */);

	}
	/** End of hack of Baytrail **/
}
/* end of MIPI functions */
#endif

static enum ia_css_err
load_preview_binaries(struct ia_css_pipe *pipe)
{
	struct ia_css_frame_info prev_in_info,
				 prev_bds_out_info,
				 prev_out_info,
				 prev_vf_info;
	struct ia_css_binary_descr preview_descr;
	bool online;
	enum ia_css_err err = IA_CSS_SUCCESS;
	bool continuous, need_vf_pp = false;
	unsigned int left_cropping;
	bool need_isp_copy_binary = false;
#ifdef USE_INPUT_SYSTEM_VERSION_2401
	bool sensor = false;
#endif
	/* preview only have 1 output pin now */
	struct ia_css_frame_info *pipe_out_info = &pipe->output_info[0];

	assert(pipe != NULL);
	assert(pipe->stream != NULL);

	online = pipe->stream->config.online;
	continuous = pipe->stream->config.continuous;
#ifdef USE_INPUT_SYSTEM_VERSION_2401
	sensor = pipe->stream->config.mode == IA_CSS_INPUT_MODE_SENSOR;
#endif

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"load_preview_binaries() enter:\n");

	if (pipe->pipe_settings.preview.preview_binary.info)
		return IA_CSS_SUCCESS;

	err = ia_css_util_check_input(&pipe->stream->config, false, false);
	if (err != IA_CSS_SUCCESS)
		return err;
	err = ia_css_frame_check_info(pipe_out_info);
	if (err != IA_CSS_SUCCESS)
		return err;

	/* Note: the current selection of vf_pp binary and
	 * parameterization of the preview binary contains a few pieces
	 * of hardcoded knowledge. This needs to be cleaned up such that
	 * the binary selection becomes more generic.
	 * The vf_pp binary is needed if one or more of the following features
	 * are required:
	 * 1. YUV downscaling.
	 * 2. Digital zoom.
	 * 3. An output format that is not supported by the preview binary.
	 *    In practice this means something other than yuv_line or nv12.
	 * The decision if the vf_pp binary is needed for YUV downscaling is
	 * made after the preview binary selection, since some preview binaries
	 * can perform the requested YUV downscaling.
	 * */
	need_vf_pp = pipe->config.enable_dz;
	need_vf_pp |= pipe_out_info->format != IA_CSS_FRAME_FORMAT_YUV_LINE &&
		      pipe_out_info->format != IA_CSS_FRAME_FORMAT_NV12;

	/* Preview step 1 */
	if (pipe->vf_yuv_ds_input_info.res.width)
		prev_vf_info = pipe->vf_yuv_ds_input_info;
	else
		prev_vf_info = *pipe_out_info;
	/* If vf_pp is needed, then preview must output yuv_line.
	 * The exception is when vf_pp is manually disabled, that is only
	 * used in combination with a pipeline extension that requires
	 * yuv_line as input.
	 * */
	if (need_vf_pp)
		ia_css_frame_info_set_format(&prev_vf_info,
					     IA_CSS_FRAME_FORMAT_YUV_LINE);

	err = ia_css_pipe_get_preview_binarydesc(pipe, &preview_descr,
		&prev_in_info, &prev_bds_out_info, &prev_out_info, &prev_vf_info);
	if (err != IA_CSS_SUCCESS)
		return err;
	err = ia_css_binary_find(&preview_descr,
				 &pipe->pipe_settings.preview.preview_binary);
	if (err != IA_CSS_SUCCESS)
		return err;

	/* The vf_pp binary is needed when (further) YUV downscaling is required */
	need_vf_pp |= pipe->pipe_settings.preview.preview_binary.out_frame_info[0].res.width != pipe_out_info->res.width;
	need_vf_pp |= pipe->pipe_settings.preview.preview_binary.out_frame_info[0].res.height != pipe_out_info->res.height;

	/* When vf_pp is needed, then the output format of the selected
	 * preview binary must be yuv_line. If this is not the case,
	 * then the preview binary selection is done again.
	 */
	if (need_vf_pp &&
	    (pipe->pipe_settings.preview.preview_binary.out_frame_info[0].format != IA_CSS_FRAME_FORMAT_YUV_LINE)) {

		/* Preview step 2 */
		if (pipe->vf_yuv_ds_input_info.res.width)
			prev_vf_info = pipe->vf_yuv_ds_input_info;
		else
			prev_vf_info = *pipe_out_info;

		ia_css_frame_info_set_format(&prev_vf_info,
					     IA_CSS_FRAME_FORMAT_YUV_LINE);

		err = ia_css_pipe_get_preview_binarydesc(pipe, &preview_descr,
			&prev_in_info, &prev_bds_out_info, &prev_out_info, &prev_vf_info);
		if (err != IA_CSS_SUCCESS)
			return err;
		err = ia_css_binary_find(&preview_descr,
					 &pipe->pipe_settings.preview.preview_binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	}

	if (need_vf_pp) {
		struct ia_css_binary_descr vf_pp_descr;

		/* Viewfinder post-processing */
		ia_css_pipe_get_vfpp_binarydesc(pipe, &vf_pp_descr,
			&pipe->pipe_settings.preview.preview_binary.out_frame_info[0],
			pipe_out_info);
		err = ia_css_binary_find(&vf_pp_descr,
				 &pipe->pipe_settings.preview.vf_pp_binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	}

#ifdef USE_INPUT_SYSTEM_VERSION_2401
	/* When the input system is 2401, only the Direct Sensor Mode
	 * Offline Preview uses the ISP copy binary.
	 */
	need_isp_copy_binary = !online && sensor;
#else
	need_isp_copy_binary = !online && !continuous;
#endif

	/* Copy */
	if (need_isp_copy_binary) {
		err = load_copy_binary(pipe,
				       &pipe->pipe_settings.preview.copy_binary,
				       &pipe->pipe_settings.preview.preview_binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	}

	left_cropping = pipe->pipe_settings.preview.preview_binary.info->sp.left_cropping;

	if (pipe->shading_table) {
		ia_css_shading_table_free(pipe->shading_table);
		pipe->shading_table = NULL;
	}

	return IA_CSS_SUCCESS;
}

static void
ia_css_binary_unload(struct ia_css_binary *binary)
{
	ia_css_binary_destroy_isp_parameters(binary);
}

static void
unload_preview_binaries(struct ia_css_pipe *pipe)
{
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"unload_preview_binaries() enter:\n");

	ia_css_binary_unload(&pipe->pipe_settings.preview.copy_binary);
	ia_css_binary_unload(&pipe->pipe_settings.preview.preview_binary);
	ia_css_binary_unload(&pipe->pipe_settings.preview.vf_pp_binary);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"unload_preview_binaries() leave:\n");
}

static const struct ia_css_fw_info *last_output_firmware(
	const struct ia_css_fw_info *fw)
{
	const struct ia_css_fw_info *last_fw = NULL;
/* fw can be NULL */
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"last_output_firmware() enter:\n");

	for (; fw; fw = fw->next) {
		const struct ia_css_fw_info *info = fw;
		if (info->info.isp.sp.enable.output)
			last_fw = fw;
	}
	return last_fw;
}

static enum ia_css_err add_firmwares(
	struct ia_css_pipeline *me,
	struct ia_css_binary *binary,
	const struct ia_css_fw_info *fw,
	const struct ia_css_fw_info *last_fw,
	unsigned int binary_mode,
	struct ia_css_frame *in_frame,
	struct ia_css_frame *out_frame,
	struct ia_css_frame *vf_frame,
	struct ia_css_pipeline_stage **my_stage,
	struct ia_css_pipeline_stage **vf_stage)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct ia_css_pipeline_stage *extra_stage = NULL;
	struct ia_css_pipeline_stage_desc stage_desc;

/* all args can be NULL ??? */
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"add_firmwares() enter:\n");

	for (; fw; fw = fw->next) {
		struct ia_css_frame *out[IA_CSS_BINARY_MAX_OUTPUT_PORTS] = {NULL};
		struct ia_css_frame *in = NULL;
		struct ia_css_frame *vf = NULL;
		if ((fw == last_fw) && (fw->info.isp.sp.enable.out_frame  != 0)) {
			out[0] = out_frame;
		}
		if (fw->info.isp.sp.enable.in_frame != 0) {
			in = in_frame;
		}
		if (fw->info.isp.sp.enable.out_frame != 0) {
			vf = vf_frame;
		}
		ia_css_pipe_get_firmwares_stage_desc(&stage_desc, binary,
			out, in, vf, fw, binary_mode);
		err = ia_css_pipeline_create_and_add_stage(me,
				&stage_desc,
				&extra_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
		if (fw->info.isp.sp.enable.output != 0)
			in_frame = extra_stage->args.out_frame[0];
		if (my_stage && !*my_stage && extra_stage)
			*my_stage = extra_stage;
		if (vf_stage && !*vf_stage && extra_stage &&
		    fw->info.isp.sp.enable.vf_veceven)
			*vf_stage = extra_stage;
	}
	return err;
}

static enum ia_css_err add_vf_pp_stage(
	struct ia_css_pipe *pipe,
	struct ia_css_frame *out_frame,
	struct ia_css_binary *vf_pp_binary,
	struct ia_css_pipeline_stage *post_stage,
	struct ia_css_pipeline_stage **vf_pp_stage)
{

	struct ia_css_pipeline *me = NULL;
	const struct ia_css_fw_info *last_fw = NULL;
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct ia_css_frame *in_frame = NULL;
	struct ia_css_frame *out_frames[IA_CSS_BINARY_MAX_OUTPUT_PORTS];
	struct ia_css_pipeline_stage_desc stage_desc;

/* out_frame can be NULL ??? */

	assert(pipe != NULL);
	assert(vf_pp_binary != NULL);
	assert(post_stage != NULL);
	assert(vf_pp_stage != NULL);

	ia_css_pipe_util_create_output_frames(out_frames);
	me = &pipe->pipeline;
	in_frame = post_stage->args.out_vf_frame;


	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
	"add_vf_pp_stage() enter:\n");

	*vf_pp_stage = NULL;

	if (in_frame == NULL)
		in_frame = post_stage->args.out_frame[0];

	last_fw = last_output_firmware(pipe->vf_stage);
	if (!pipe->extra_config.disable_vf_pp) {
		if (last_fw) {
			ia_css_pipe_util_set_output_frames(out_frames, 0, NULL);
			ia_css_pipe_get_generic_stage_desc(&stage_desc, vf_pp_binary,
				out_frames, in_frame, NULL, NULL);
		} else{
			ia_css_pipe_util_set_output_frames(out_frames, 0, out_frame);
			ia_css_pipe_get_generic_stage_desc(&stage_desc, vf_pp_binary,
				out_frames, in_frame, NULL, NULL);
		}
		err = ia_css_pipeline_create_and_add_stage(me, &stage_desc, vf_pp_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
		in_frame = (*vf_pp_stage)->args.out_frame[0];
	}
	err = add_firmwares(me, vf_pp_binary, pipe->vf_stage, last_fw,
			    IA_CSS_BINARY_MODE_VF_PP,
			    in_frame, out_frame, NULL,
			    vf_pp_stage, NULL);
	return err;
}

static enum ia_css_err add_yuv_scaler_stage(
	struct ia_css_pipe *pipe,
	struct ia_css_pipeline *me,
	struct ia_css_frame *in_frame,
	struct ia_css_frame *out_frame,
	struct ia_css_frame *internal_out_frame,
	struct ia_css_binary *yuv_scaler_binary,
	struct ia_css_pipeline_stage *prev_stage,
	struct ia_css_pipeline_stage **pre_vf_pp_stage,
	bool is_output_stage)
{
	const struct ia_css_fw_info *last_fw;
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct ia_css_frame *vf_frame = NULL;
	struct ia_css_frame *out_frames[IA_CSS_BINARY_MAX_OUTPUT_PORTS];
	struct ia_css_pipeline_stage_desc stage_desc;

	/* out_frame can be NULL ??? */
	assert(in_frame != NULL);
	assert(pipe != NULL);
	assert(me != NULL);
	assert(yuv_scaler_binary != NULL);
	assert(prev_stage != NULL);
	assert(pre_vf_pp_stage != NULL);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"add_yuv_scaler_stage() enter:\n");

	*pre_vf_pp_stage = NULL;
	ia_css_pipe_util_create_output_frames(out_frames);

	last_fw = last_output_firmware(pipe->output_stage);
	if (need_yuv_scaler(pipe)) {
		if (is_output_stage) {
			err = ia_css_frame_allocate_from_info(&vf_frame,
					    &yuv_scaler_binary->vf_frame_info);
			if (err != IA_CSS_SUCCESS)
				return err;
		}
		if(last_fw)	{
			ia_css_pipe_util_set_output_frames(out_frames, 0, NULL);
			ia_css_pipe_get_generic_stage_desc(&stage_desc,
				yuv_scaler_binary, out_frames, in_frame, NULL, vf_frame);
		} else {
			ia_css_pipe_util_set_output_frames(out_frames, 0, out_frame);
			ia_css_pipe_util_set_output_frames(out_frames, 1, internal_out_frame);
			ia_css_pipe_get_generic_stage_desc(&stage_desc,
				yuv_scaler_binary, out_frames, in_frame, NULL, vf_frame);
		}
		err = ia_css_pipeline_create_and_add_stage(me,
			&stage_desc,
			pre_vf_pp_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
		in_frame = (*pre_vf_pp_stage)->args.out_frame[0];
	}
	err = add_firmwares(me, yuv_scaler_binary, pipe->output_stage, last_fw,
			    IA_CSS_BINARY_MODE_CAPTURE_PP,
			    in_frame, out_frame, vf_frame,
			    NULL, pre_vf_pp_stage);
	/* If a firmware produce vf_pp output, we set that as vf_pp input */
	if (*pre_vf_pp_stage) {
		(*pre_vf_pp_stage)->args.vf_downscale_log2 =
		  yuv_scaler_binary->vf_downscale_log2;
	} else {
		*pre_vf_pp_stage = prev_stage;
	}
	return err;
}

static enum ia_css_err add_capture_pp_stage(
	struct ia_css_pipe *pipe,
	struct ia_css_pipeline *me,
	struct ia_css_frame *out_frame,
	struct ia_css_binary *capture_pp_binary,
	struct ia_css_pipeline_stage *capture_stage,
	struct ia_css_pipeline_stage **pre_vf_pp_stage)
{
	const struct ia_css_fw_info *last_fw = NULL;
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct ia_css_frame *in_frame = NULL;
	struct ia_css_frame *vf_frame = NULL;
	struct ia_css_frame *out_frames[IA_CSS_BINARY_MAX_OUTPUT_PORTS];
	struct ia_css_pipeline_stage_desc stage_desc;
	//unsigned int i;

	/* out_frame can be NULL ??? */
	assert(pipe != NULL);
	assert(me != NULL);
	assert(capture_pp_binary != NULL);
	assert(capture_stage != NULL);
	assert(pre_vf_pp_stage != NULL);
	in_frame = capture_stage->args.out_frame[0];
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"add_capture_pp_stage() enter:\n");

	*pre_vf_pp_stage = NULL;
	ia_css_pipe_util_create_output_frames(out_frames);

	last_fw = last_output_firmware(pipe->output_stage);
	if (need_capture_pp(pipe)) {
		err = ia_css_frame_allocate_from_info(&vf_frame,
					    &capture_pp_binary->vf_frame_info);
		if (err != IA_CSS_SUCCESS)
			return err;
		if(last_fw)	{
			ia_css_pipe_util_set_output_frames(out_frames, 0, NULL);
			ia_css_pipe_get_generic_stage_desc(&stage_desc,
				capture_pp_binary, out_frames, NULL, NULL, vf_frame);
		} else {
			ia_css_pipe_util_set_output_frames(out_frames, 0, out_frame);
			ia_css_pipe_get_generic_stage_desc(&stage_desc,
				capture_pp_binary, out_frames, NULL, NULL, vf_frame);
		}
		err = ia_css_pipeline_create_and_add_stage(me,
			&stage_desc,
			pre_vf_pp_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
		in_frame = (*pre_vf_pp_stage)->args.out_frame[0];
	}
	err = add_firmwares(me, capture_pp_binary, pipe->output_stage, last_fw,
			    IA_CSS_BINARY_MODE_CAPTURE_PP,
			    in_frame, out_frame, vf_frame,
			    NULL, pre_vf_pp_stage);
	/* If a firmware produce vf_pp output, we set that as vf_pp input */
	if (*pre_vf_pp_stage) {
		(*pre_vf_pp_stage)->args.vf_downscale_log2 =
		  capture_pp_binary->vf_downscale_log2;
	} else {
		*pre_vf_pp_stage = capture_stage;
	}
	return err;
}

/* keep this function for future use */
#if 0
static void
ia_css_connect_buf_queues(unsigned int host2sp_addr,
	unsigned int desc_addr,
	unsigned int elems_addr,
	ia_css_queue_t *qhandle)
{
	ia_css_queue_remote_t remoteq;
	/* Setup queue location as SP and proc id as SP0_ID*/
	remoteq.location = IA_CSS_QUEUE_LOC_SP;
	remoteq.proc_id = SP0_ID;

	remoteq.cb_desc_addr = host2sp_addr + desc_addr;
	remoteq.cb_elems_addr = host2sp_addr + elems_addr;

	/* Initialize the queue instance and obtain handle */
	ia_css_queue_remote_init(qhandle, &remoteq);
}
#endif

static void sh_css_setup_queues(void)
{
	enum ia_css_err error;
	const struct ia_css_fw_info *fw;
	unsigned int HIVE_ADDR_host_sp_queues_initialized;
	int i;

	for (i = 0; i < MAX_HMM_BUFFER_NUM; i++)
		hmm_buffer_record_h[i] = NULL;

	sh_css_event_init_irq_mask();

	fw = &sh_css_sp_fw;
	HIVE_ADDR_host_sp_queues_initialized =
		fw->info.sp.host_sp_queues_initialized;

	error = ia_css_bufq_init();
	if(error != IA_CSS_SUCCESS)
		return;

		/* set "host_sp_queues_initialized" to "true" */
	sp_dmem_store_uint32(SP0_ID,
		(unsigned int)sp_address_of(host_sp_queues_initialized),
		(uint32_t)(1));
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "sh_css_setup_queues() leave:\n");
	return;
}



static enum ia_css_err
init_vf_frameinfo_defaults(struct ia_css_pipe *pipe,
	struct ia_css_frame *vf_frame, unsigned int idx)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	unsigned int thread_id;
	enum sh_css_queue_id queue_id;

	assert(vf_frame != NULL);

	sh_css_pipe_get_viewfinder_frame_info(pipe, &vf_frame->info, idx);
	vf_frame->contiguous = false;
	vf_frame->flash_state = IA_CSS_FRAME_FLASH_STATE_NONE;
	ia_css_pipeline_get_sp_thread_id(ia_css_pipe_get_pipe_num(pipe), &thread_id);
	ia_css_query_internal_queue_id(IA_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME + idx, thread_id, &queue_id);
	vf_frame->dynamic_data_index = queue_id;
	vf_frame->buf_type = IA_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME + idx;

	err = ia_css_frame_init_planes(vf_frame);
	return err;
}

static enum ia_css_err
init_in_frameinfo_memory_defaults(struct ia_css_pipe *pipe,
	struct ia_css_frame *frame)
{
	struct ia_css_frame *in_frame;
	enum ia_css_err err = IA_CSS_SUCCESS;
	unsigned int thread_id;
	enum sh_css_queue_id queue_id;

	assert(frame != NULL);

	in_frame = frame;

	in_frame->info.format = IA_CSS_FRAME_FORMAT_RAW;
	in_frame->info.res.width = pipe->stream->config.input_res.width;
	in_frame->info.res.height = pipe->stream->config.input_res.height;
	in_frame->info.raw_bit_depth =
		ia_css_pipe_util_pipe_input_format_bpp(pipe);
	ia_css_frame_info_set_width(&in_frame->info, pipe->stream->config.input_res.width, 0);

	in_frame->contiguous = false;
	in_frame->flash_state = IA_CSS_FRAME_FLASH_STATE_NONE;
	ia_css_pipeline_get_sp_thread_id(ia_css_pipe_get_pipe_num(pipe), &thread_id);
	ia_css_query_internal_queue_id(IA_CSS_BUFFER_TYPE_INPUT_FRAME, thread_id, &queue_id);
	in_frame->dynamic_data_index = queue_id;
	in_frame->buf_type = IA_CSS_BUFFER_TYPE_INPUT_FRAME;

	err = ia_css_frame_init_planes(in_frame);

	return err;
}

static enum ia_css_err
init_out_frameinfo_defaults(struct ia_css_pipe *pipe,
	struct ia_css_frame *out_frame, unsigned int idx)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	unsigned int thread_id;
	enum sh_css_queue_id queue_id;

	assert(out_frame != NULL);

	sh_css_pipe_get_output_frame_info(pipe, &out_frame->info, idx);
	out_frame->contiguous = false;
	out_frame->flash_state = IA_CSS_FRAME_FLASH_STATE_NONE;
	ia_css_pipeline_get_sp_thread_id(ia_css_pipe_get_pipe_num(pipe), &thread_id);
	ia_css_query_internal_queue_id(IA_CSS_BUFFER_TYPE_OUTPUT_FRAME + idx, thread_id, &queue_id);
	out_frame->dynamic_data_index = queue_id;
	out_frame->buf_type = IA_CSS_BUFFER_TYPE_OUTPUT_FRAME + idx;
	err = ia_css_frame_init_planes(out_frame);

	return err;
}

/* Create stages for video pipe */
static enum ia_css_err create_host_video_pipeline(struct ia_css_pipe *pipe)
{
	struct ia_css_pipeline_stage_desc stage_desc;
	struct ia_css_binary *copy_binary, *video_binary, *vf_pp_binary;
	struct ia_css_pipeline_stage *copy_stage  = NULL;
	struct ia_css_pipeline_stage *video_stage = NULL;
	struct ia_css_pipeline_stage *vf_pp_stage = NULL;
	struct ia_css_pipeline_stage *in_stage    = NULL;
	struct ia_css_pipeline *me;
	struct ia_css_frame *in_frame = NULL;
	struct ia_css_frame *out_frame;
	struct ia_css_frame *out_frames[IA_CSS_BINARY_MAX_OUTPUT_PORTS];
	struct ia_css_frame *vf_frame;
	enum ia_css_err err = IA_CSS_SUCCESS;
	bool resolution_differs;
	bool need_vf_pp = false;
	unsigned num_output_pins;
	bool need_in_frameinfo_memory = false;

	assert(pipe != NULL);
	assert(pipe->stream != NULL);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"create_host_video_pipeline() enter:\n");

	ia_css_pipe_util_create_output_frames(out_frames);
	in_frame = &pipe->in_frame_struct;
	out_frame = &pipe->out_frame_struct;
	vf_frame = &pipe->vf_frame_struct;

	pipe->out_frame_struct.data = 0;
	pipe->vf_frame_struct.data = 0;

	/* pipeline already created as part of create_host_pipeline_structure */
	me = &pipe->pipeline;
	ia_css_pipeline_clean(me);

	me->dvs_frame_delay = pipe->dvs_frame_delay;

#ifdef USE_INPUT_SYSTEM_VERSION_2401
	/* When the input system is 2401, always enable 'in_frameinfo_memory'
	 * except for the following: online or continuous
	 */
	need_in_frameinfo_memory = !(pipe->stream->config.online || pipe->stream->config.continuous);
#else
	/* Construct in_frame info (only in case we have dynamic input */
	need_in_frameinfo_memory = pipe->stream->config.mode == IA_CSS_INPUT_MODE_MEMORY;
#endif

	/* Construct in_frame info (only in case we have dynamic input */
	if (need_in_frameinfo_memory) {
		err = init_in_frameinfo_memory_defaults(pipe, in_frame);
		if (err != IA_CSS_SUCCESS)
			goto ERR;
	} else {
		in_frame = NULL;
	}

	out_frame->data = 0;
	err = init_out_frameinfo_defaults(pipe, out_frame, 0);
	if (err != IA_CSS_SUCCESS)
		goto ERR;

	if (!pipe->enable_viewfinder) {
		/* These situations don't support viewfinder output */
		vf_frame = NULL;
	} else {
		vf_frame->data = 0;
		err = init_vf_frameinfo_defaults(pipe, vf_frame, 0);
		if (err != IA_CSS_SUCCESS)
			goto ERR;
	}

	copy_binary  = &pipe->pipe_settings.video.copy_binary;
	video_binary = &pipe->pipe_settings.video.video_binary;
	vf_pp_binary = &pipe->pipe_settings.video.vf_pp_binary;
	num_output_pins = video_binary->info->num_output_pins;

	if (pipe->pipe_settings.video.copy_binary.info) {
		ia_css_pipe_util_set_output_frames(out_frames, 0, NULL);
		ia_css_pipe_get_generic_stage_desc(&stage_desc, copy_binary,
			out_frames, NULL, NULL, NULL);
		err = ia_css_pipeline_create_and_add_stage(me,
			&stage_desc,
			&copy_stage);
		if (err != IA_CSS_SUCCESS)
			goto ERR;
		in_frame = me->stages->args.out_frame[0];
		in_stage = copy_stage;
	} else if (pipe->stream->config.continuous) {
#ifdef USE_INPUT_SYSTEM_VERSION_2401
		/* When continous is enabled, configure in_frame with the
		 * last pipe, which is the copy pipe.
		 */
		in_frame = pipe->stream->last_pipe->continuous_frames[0];
#else
		in_frame = pipe->continuous_frames[0];
#endif
	}

	resolution_differs =
		(pipe->vf_output_info[0].res.width != pipe->output_info[0].res.width) ||
		(pipe->vf_output_info[0].res.height != pipe->output_info[0].res.height);

	need_vf_pp = pipe->enable_viewfinder && ((num_output_pins == 1) ||
		((num_output_pins == 2) && resolution_differs));

	/* when the video binary supports a second output pin,
	   it can directly produce the vf_frame.  */
	ia_css_pipe_util_set_output_frames(out_frames, 0, out_frame);
	if(need_vf_pp) {
		ia_css_pipe_get_generic_stage_desc(&stage_desc, video_binary,
			out_frames, in_frame, NULL, NULL);
	} else {
		ia_css_pipe_get_generic_stage_desc(&stage_desc, video_binary,
			out_frames, in_frame, NULL, vf_frame);
	}
	err = ia_css_pipeline_create_and_add_stage(me,
			&stage_desc,
			&video_stage);
	if (err != IA_CSS_SUCCESS)
		goto ERR;

	/* If we use copy iso video, the input must be yuv iso raw */
	if(video_stage) {
		video_stage->args.copy_vf =
			video_binary->info->sp.mode == IA_CSS_BINARY_MODE_COPY;
		video_stage->args.copy_output = video_stage->args.copy_vf;
	}

	/* when the video binary supports only 1 output pin, vf_pp is needed to
	produce the vf_frame.*/
	if (need_vf_pp && video_stage) {
		err = add_vf_pp_stage(pipe, vf_frame, vf_pp_binary,
				      video_stage, &vf_pp_stage);
		if (err != IA_CSS_SUCCESS)
			goto ERR;
	}

	pipe->pipeline.acquire_isp_each_stage = false;
	ia_css_pipeline_finalize_stages(&pipe->pipeline);

	if (video_stage) {
		int i;
		for (i = 0; i < NUM_VIDEO_TNR_FRAMES; i++) {
			video_stage->args.tnr_frames[i] =
				pipe->pipe_settings.video.tnr_frames[i];
		}
		for (i = 0; i < NUM_VIDEO_DELAY_FRAMES; i++) {
			video_stage->args.delay_frames[i] =
				pipe->pipe_settings.video.delay_frames[i];
		}
	}

	/* update the arguments with the latest info */
	if (video_stage)
		video_stage->args.out_frame[0] = out_frame;

	if (vf_pp_stage)
		vf_pp_stage->args.out_frame[0] = vf_frame;

	if (pipe->stream->config.continuous) {
		if (video_stage)
			video_stage->args.in_frame =  pipe->continuous_frames[0];
	}

	configure_pipe_inout_port(&pipe->pipeline,
		pipe->stream->config.continuous);

ERR:
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"create_host_video_pipeline() leave: return (%x)\n", err);
	return err;
}

static enum ia_css_err
create_host_acc_pipeline(struct ia_css_pipe *pipe)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	unsigned int i;

	assert(pipe != NULL);
	assert(pipe->stream != NULL);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"create_host_acc_pipeline() enter\n");

	pipe->pipeline.num_execs = pipe->config.acc_num_execs;

	for (i=0; i<pipe->config.num_acc_stages; i++) {
		struct ia_css_fw_info *fw = pipe->config.acc_stages[i];
		err = sh_css_pipeline_add_acc_stage(&pipe->pipeline, fw);
		if (err != IA_CSS_SUCCESS)
			goto ERR;
	}

	ia_css_pipeline_finalize_stages(&pipe->pipeline);
	configure_pipe_inout_port(&pipe->pipeline, false);

ERR:
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"create_host_acc_pipeline() leave: return(%d)\n", err);
	return err;
}

/* Create stages for preview */
static enum ia_css_err
create_host_preview_pipeline(struct ia_css_pipe *pipe)
{
	struct ia_css_pipeline_stage *post_stage;
	struct ia_css_pipeline_stage_desc stage_desc;
	struct ia_css_pipeline_stage *preview_stage, *out_stage = NULL;
	struct ia_css_pipeline *me = NULL;
	struct ia_css_binary *copy_binary, *preview_binary, *vf_pp_binary = NULL;
	struct ia_css_frame *in_frame = NULL, *cc_frame = NULL;
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct ia_css_frame *out_frame;
	struct ia_css_frame *out_frames[IA_CSS_BINARY_MAX_OUTPUT_PORTS];
	bool need_in_frameinfo_memory = false;
#ifdef USE_INPUT_SYSTEM_VERSION_2401
	bool sensor = false;
	bool buffered_sensor = false;
	bool online = false;
	bool continuous = false;
#endif

	assert(pipe != NULL);
	assert(pipe->stream != NULL);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"create_host_preview_pipeline(): enter\n");

	ia_css_pipe_util_create_output_frames(out_frames);
	/* pipeline already created as part of create_host_pipeline_structure */
	me = &pipe->pipeline;
	ia_css_pipeline_clean(me);

#ifdef USE_INPUT_SYSTEM_VERSION_2401
	/* When the input system is 2401, always enable 'in_frameinfo_memory'
	 * except for the following:
	 * - Direct Sensor Mode Online Preview
	 * - Buffered Sensor Mode Online Preview
	 * - Direct Sensor Mode Continuous Preview
	 * - Buffered Sensor Mode Continous Preview
	 */
	sensor = (pipe->stream->config.mode == IA_CSS_INPUT_MODE_SENSOR);
	buffered_sensor = (pipe->stream->config.mode == IA_CSS_INPUT_MODE_BUFFERED_SENSOR);
	online = pipe->stream->config.online;
	continuous = pipe->stream->config.continuous;
	need_in_frameinfo_memory =
		!((sensor && (online || continuous)) || (buffered_sensor && (online || continuous)));
#else
	/* Construct in_frame info (only in case we have dynamic input */
	need_in_frameinfo_memory = pipe->stream->config.mode == IA_CSS_INPUT_MODE_MEMORY;
#endif
	if (need_in_frameinfo_memory) {
		err = init_in_frameinfo_memory_defaults(pipe, &me->in_frame);
		if (err != IA_CSS_SUCCESS)
			goto ERR;

		in_frame = &me->in_frame;
	} else {
		in_frame = NULL;
	}

	err = init_out_frameinfo_defaults(pipe, &me->out_frame[0], 0);
	if (err != IA_CSS_SUCCESS)
		goto ERR;
	out_frame = &me->out_frame[0];

	copy_binary    = &pipe->pipe_settings.preview.copy_binary;
	preview_binary = &pipe->pipe_settings.preview.preview_binary;
	if (pipe->pipe_settings.preview.vf_pp_binary.info)
		vf_pp_binary = &pipe->pipe_settings.preview.vf_pp_binary;

	if (pipe->pipe_settings.preview.copy_binary.info) {
		ia_css_pipe_util_set_output_frames(out_frames, 0, NULL);
		ia_css_pipe_get_generic_stage_desc(&stage_desc, copy_binary,
			out_frames, NULL, NULL, NULL);
		err = ia_css_pipeline_create_and_add_stage(me,
				&stage_desc,
				&post_stage);
		if (err != IA_CSS_SUCCESS)
			goto ERR;
		in_frame = me->stages->args.out_frame[0];
	} else {
#ifdef USE_INPUT_SYSTEM_VERSION_2401
		/* When continuous is enabled, configure in_frame with the
		 * last pipe, which is the copy pipe.
		 */
		if (continuous)
			in_frame = pipe->stream->last_pipe->continuous_frames[0];
#else
		in_frame = pipe->continuous_frames[0];
#endif
	}

	if (vf_pp_binary) {
		ia_css_pipe_util_set_output_frames(out_frames, 0, NULL);
		ia_css_pipe_get_generic_stage_desc(&stage_desc, preview_binary,
			out_frames, in_frame, cc_frame, NULL);
	} else {
		ia_css_pipe_util_set_output_frames(out_frames, 0, out_frame);
		ia_css_pipe_get_generic_stage_desc(&stage_desc, preview_binary,
			out_frames, in_frame, cc_frame, NULL);
	}
	err = ia_css_pipeline_create_and_add_stage(me,
		&stage_desc,
		&post_stage);
	if (err != IA_CSS_SUCCESS)
		goto ERR;
	/* If we use copy iso preview, the input must be yuv iso raw */
	post_stage->args.copy_vf =
		preview_binary->info->sp.mode == IA_CSS_BINARY_MODE_COPY;
	post_stage->args.copy_output = !post_stage->args.copy_vf;
	if (post_stage->args.copy_vf && !post_stage->args.out_vf_frame) {
		/* in case of copy, use the vf frame as output frame */
		post_stage->args.out_vf_frame =
			post_stage->args.out_frame[0];
	}
	if (vf_pp_binary) {
		err = add_vf_pp_stage(pipe, out_frame, vf_pp_binary,
				post_stage, &out_stage);
		if (err != IA_CSS_SUCCESS)
			goto ERR;
	}

	pipe->pipeline.acquire_isp_each_stage = false;
	ia_css_pipeline_finalize_stages(&pipe->pipeline);

	if (vf_pp_binary) {
		err = ia_css_pipeline_get_output_stage(me, IA_CSS_BINARY_MODE_VF_PP,
						       &out_stage);
		if (err != IA_CSS_SUCCESS)
			goto ERR;
	}

	err = ia_css_pipeline_get_stage(me, preview_binary->info->sp.mode,
					&preview_stage);
	if (err != IA_CSS_SUCCESS)
		goto ERR;

	if (!out_stage)
		out_stage = preview_stage;

	out_stage->args.out_frame[0] = out_frame;
	if (pipe->stream->config.continuous) {
		preview_stage->args.in_frame = pipe->continuous_frames[0];
	}

	configure_pipe_inout_port(&pipe->pipeline,
		pipe->stream->config.continuous);

ERR:
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"create_host_preview_pipeline() leave: return(%d)\n", err);
	return err;
}

static void send_raw_frames(struct ia_css_pipe *pipe)
{
	if (pipe->stream->config.continuous) {
		struct ia_css_frame *in_frame = NULL;
		unsigned int i;

		in_frame = pipe->continuous_frames[0];

		sh_css_update_host2sp_cont_num_raw_frames
			(pipe->stream->config.init_num_cont_raw_buf, true);
		sh_css_update_host2sp_cont_num_raw_frames
			(pipe->stream->config.target_num_cont_raw_buf, false);

		/* Hand-over all the SP-internal buffers */
		for (i = 0; i < pipe->stream->config.init_num_cont_raw_buf; i++) {
			sh_css_update_host2sp_offline_frame(i,
				pipe->continuous_frames[i], pipe->cont_md_buffers[i]);
		}
	}

	return;
}

static enum ia_css_err
preview_start(struct ia_css_pipe *pipe)
{
	struct ia_css_pipeline *me ;
	struct ia_css_binary *copy_binary, *preview_binary, *vf_pp_binary = NULL;
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct ia_css_pipe *copy_pipe, *capture_pipe;
	enum sh_css_pipe_config_override copy_ovrd;
	enum ia_css_input_mode preview_pipe_input_mode;
#ifdef USE_INPUT_SYSTEM_VERSION_2401
	struct ia_css_pipe *target_pipe = NULL;
#endif

	assert(pipe != NULL);
	assert(pipe->stream != NULL);

	me = &pipe->pipeline;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
	"preview_start() enter: void\n");

	preview_pipe_input_mode = pipe->stream->config.mode;

	copy_pipe    = pipe->pipe_settings.preview.copy_pipe;
	capture_pipe = pipe->pipe_settings.preview.capture_pipe;

	copy_binary    = &pipe->pipe_settings.preview.copy_binary;
	preview_binary = &pipe->pipe_settings.preview.preview_binary;
	if (pipe->pipe_settings.preview.vf_pp_binary.info)
		vf_pp_binary = &pipe->pipe_settings.preview.vf_pp_binary;

	sh_css_metrics_start_frame();

#if defined(USE_INPUT_SYSTEM_VERSION_2) || defined(USE_INPUT_SYSTEM_VERSION_2401)
	/* multi stream video needs mipi buffers */
	send_mipi_frames(pipe);
#endif
	send_raw_frames(pipe);

	{
		unsigned int thread_id;

		ia_css_pipeline_get_sp_thread_id(ia_css_pipe_get_pipe_num(pipe), &thread_id);
		copy_ovrd = 1 << thread_id;

		if (pipe->stream->cont_capt) {
			ia_css_pipeline_get_sp_thread_id(ia_css_pipe_get_pipe_num(capture_pipe), &thread_id);
			copy_ovrd |= 1 << thread_id;
		}
	}

#if !defined(HAS_NO_INPUT_SYSTEM)
#if defined(USE_INPUT_SYSTEM_VERSION_2)
	err = sh_css_config_input_network(pipe, copy_binary);
	if (err != IA_CSS_SUCCESS)
		goto ERR;
#elif defined(USE_INPUT_SYSTEM_VERSION_2401)
	if (pipe->stream->config.continuous) {
		/* make the copy pipe create/own the ISYS stream */
		target_pipe = copy_pipe;
	} else {
		/* make the preview pipe create/own the ISYS stream */
		target_pipe = pipe;
	}

	err = sh_css_config_input_network(target_pipe, NULL);
	if (err != IA_CSS_SUCCESS)
		goto ERR;
#endif
#endif /* !HAS_NO_INPUT_SYSTEM */

	/* Construct and load the copy pipe */
	if (pipe->stream->config.continuous) {
		sh_css_sp_init_pipeline(&copy_pipe->pipeline,
			IA_CSS_PIPE_ID_COPY,
			(uint8_t)ia_css_pipe_get_pipe_num(copy_pipe),
			false,
			pipe->stream->config.pixels_per_clock == 2, false,
			false, pipe->required_bds_factor,
			copy_ovrd,
			pipe->stream->config.mode,
			&pipe->stream->config.metadata_config,
			&pipe->stream->info.metadata_info
#if !defined(HAS_NO_INPUT_SYSTEM)
			, pipe->stream->config.source.port.port
#endif
			);

		/* make the preview pipe start with mem mode input, copy handles
		   the actual mode */
		preview_pipe_input_mode = IA_CSS_INPUT_MODE_MEMORY;
	}

	/* Construct and load the capture pipe */
	if (pipe->stream->cont_capt) {
		sh_css_sp_init_pipeline(&capture_pipe->pipeline,
			IA_CSS_PIPE_ID_CAPTURE,
			(uint8_t)ia_css_pipe_get_pipe_num(capture_pipe),
			pipe->config.default_capture_config.enable_xnr,
			capture_pipe->stream->config.pixels_per_clock == 2,
			true, /* continuous */
			false, /* offline */
			capture_pipe->required_bds_factor,
			0,
			IA_CSS_INPUT_MODE_MEMORY,
			&pipe->stream->config.metadata_config,
			&pipe->stream->info.metadata_info
#if !defined(HAS_NO_INPUT_SYSTEM)
			, (mipi_port_ID_t)0
#endif
			);
	}

	err = start_pipe(pipe, copy_ovrd, preview_pipe_input_mode);

#if !defined(HAS_NO_INPUT_SYSTEM)
ERR:
#endif
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"preview_start() leave: return (%d)\n", err);
	return err;
}

enum ia_css_err
ia_css_pipe_enqueue_buffer(struct ia_css_pipe *pipe,
			   const struct ia_css_buffer *buffer)
{
	enum ia_css_err return_err = IA_CSS_SUCCESS;
	unsigned int thread_id, i;
	enum sh_css_queue_id queue_id;
	struct ia_css_pipeline *pipeline;
	struct ia_css_pipeline_stage *stage;
	struct ia_css_rmgr_vbuf_handle p_vbuf;
	struct ia_css_rmgr_vbuf_handle *h_vbuf;
	struct sh_css_hmm_buffer ddr_buffer;
	enum ia_css_buffer_type buf_type;
	enum ia_css_pipe_id pipe_id;

	if (pipe == NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	if (buffer == NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;

	buf_type = buffer->type;
	pipe_id = pipe->mode;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_pipe_enqueue_buffer() enter: pipe_id=%d, buf_type=%d, buffer=%p\n",
		pipe_id, buf_type, buffer);

	assert(pipe_id < IA_CSS_PIPE_ID_NUM);
	assert(buf_type < IA_CSS_NUM_DYNAMIC_BUFFER_TYPE);
	if ((buf_type == IA_CSS_BUFFER_TYPE_INVALID) ||
	    (buf_type >= IA_CSS_NUM_DYNAMIC_BUFFER_TYPE) ||
	    (pipe_id >= IA_CSS_PIPE_ID_NUM))
		return IA_CSS_ERR_INTERNAL_ERROR;

	//ia_css_pipeline_get_sp_thread_id(pipe_id, &thread_id);
	ia_css_pipeline_get_sp_thread_id(ia_css_pipe_get_pipe_num(pipe), &thread_id);

	ia_css_query_internal_queue_id(buf_type, thread_id, &queue_id);
	if ((queue_id <= SH_CSS_INVALID_QUEUE_ID) || (queue_id >= SH_CSS_MAX_NUM_QUEUES))
			return IA_CSS_ERR_INVALID_ARGUMENTS;

	if (!sh_css_sp_is_running()) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
			"ia_css_pipe_enqueue_buffer() leaving:"
			"queues unavailable\n");
		/* SP is not running. The queues are not valid */
		return IA_CSS_ERR_RESOURCE_NOT_AVAILABLE;
	}


	pipeline = &pipe->pipeline;

	assert(pipeline != NULL ||
	       pipe_id == IA_CSS_PIPE_ID_COPY ||
	       pipe_id == IA_CSS_PIPE_ID_ACC);

	assert(sizeof(NULL) <= sizeof(ddr_buffer.kernel_ptr));
	ddr_buffer.kernel_ptr = HOST_ADDRESS(NULL);
	ddr_buffer.cookie_ptr = buffer->driver_cookie;

	if (buf_type == IA_CSS_BUFFER_TYPE_3A_STATISTICS) {
		if (buffer->data.stats_3a == NULL)
			return IA_CSS_ERR_INVALID_ARGUMENTS;
		ddr_buffer.kernel_ptr = HOST_ADDRESS(buffer->data.stats_3a);
		ddr_buffer.payload.s3a = *buffer->data.stats_3a;
	} else if (buf_type == IA_CSS_BUFFER_TYPE_DIS_STATISTICS) {
		if (buffer->data.stats_dvs == NULL)
			return IA_CSS_ERR_INVALID_ARGUMENTS;
		ddr_buffer.kernel_ptr = HOST_ADDRESS(buffer->data.stats_dvs);
		ddr_buffer.payload.dis = *buffer->data.stats_dvs;
#if defined(IS_ISP_2500_SYSTEM)
	} else if (buf_type == IA_CSS_BUFFER_TYPE_LACE_STATISTICS) {
		if (buffer->data.stats_acc_lace == 0)
			return IA_CSS_ERR_INVALID_ARGUMENTS;
		ddr_buffer.kernel_ptr = HOST_ADDRESS(buffer->data.stats_acc_lace);
		ddr_buffer.payload.lace_stat = buffer->data.stats_acc_lace;
#endif
	} else if (buf_type == IA_CSS_BUFFER_TYPE_METADATA) {
		ddr_buffer.kernel_ptr = HOST_ADDRESS(buffer->data.metadata);
		ddr_buffer.payload.metadata = *buffer->data.metadata;
	} else if ((buf_type == IA_CSS_BUFFER_TYPE_INPUT_FRAME)
		|| (buf_type == IA_CSS_BUFFER_TYPE_OUTPUT_FRAME)
		|| (buf_type == IA_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME)
		|| (buf_type == IA_CSS_BUFFER_TYPE_SEC_OUTPUT_FRAME)
		|| (buf_type == IA_CSS_BUFFER_TYPE_SEC_VF_OUTPUT_FRAME)) {
		if (buffer->data.frame == NULL)
			return IA_CSS_ERR_INVALID_ARGUMENTS;
		ddr_buffer.kernel_ptr = HOST_ADDRESS(buffer->data.frame);
		ddr_buffer.payload.frame.frame_data = buffer->data.frame->data;
		ddr_buffer.payload.frame.flashed = 0;
#if CONFIG_ON_FRAME_ENQUEUE()
		return_err = set_config_on_frame_enqueue(
				&buffer->data.frame->info,
				&ddr_buffer.payload.frame);
		if (IA_CSS_SUCCESS != return_err)
			return return_err;
#endif
	}



	/* start of test for using rmgr for acq/rel memory */
	p_vbuf.vptr = 0;
	p_vbuf.count = 0;
	p_vbuf.size = sizeof(struct sh_css_hmm_buffer);
	h_vbuf = &p_vbuf;
	// TODO: change next to correct pool for optimization
	ia_css_rmgr_acq_vbuf(hmm_buffer_pool, &h_vbuf);

	assert(h_vbuf != NULL);
	assert(h_vbuf->vptr != 0x0);

	if ((h_vbuf == NULL) || (h_vbuf->vptr == 0x0))
		return IA_CSS_ERR_INTERNAL_ERROR;

	mmgr_store(h_vbuf->vptr,
				(void *)(&ddr_buffer),
				sizeof(struct sh_css_hmm_buffer));
	if ((buf_type == IA_CSS_BUFFER_TYPE_3A_STATISTICS)
		|| (buf_type == IA_CSS_BUFFER_TYPE_DIS_STATISTICS)
		|| (buf_type == IA_CSS_BUFFER_TYPE_LACE_STATISTICS)) {
		assert(pipeline != NULL);
		for (stage = pipeline->stages; stage; stage = stage->next) {
			/* The SP will read the params
				after it got empty 3a and dis */
			if (STATS_ENABLED(stage)) {
				/* there is a stage that needs it */
				return_err = ia_css_bufq_enqueue_buffer(thread_id,
								queue_id,
								(uint32_t)h_vbuf->vptr);
			}
		}
	} else if ((buf_type == IA_CSS_BUFFER_TYPE_INPUT_FRAME)
		|| (buf_type == IA_CSS_BUFFER_TYPE_OUTPUT_FRAME)
		|| (buf_type == IA_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME)
		|| (buf_type == IA_CSS_BUFFER_TYPE_SEC_OUTPUT_FRAME)
		|| (buf_type == IA_CSS_BUFFER_TYPE_SEC_VF_OUTPUT_FRAME)
		|| (buf_type == IA_CSS_BUFFER_TYPE_METADATA)) {
			return_err = ia_css_bufq_enqueue_buffer(thread_id,
							queue_id,
							(uint32_t)h_vbuf->vptr);
	}

	if (return_err == IA_CSS_SUCCESS) {
		for (i = 0; i < MAX_HMM_BUFFER_NUM; i++) {
			if (hmm_buffer_record_h[i] == NULL) {
				hmm_buffer_record_h[i] = h_vbuf;
				ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
					     "ia_css_pipe_enqueue_buffer()"
					     " send vbuf=%x\n",h_vbuf);
				break;
			}
		}
	} else {
		ia_css_rmgr_rel_vbuf(hmm_buffer_pool, &h_vbuf);
	}

		/*
		 * Tell the SP which queues are not empty,
		 * by sending the software event.
		 */
	if (return_err == IA_CSS_SUCCESS)
		if (!sh_css_sp_is_running()) {
			/* SP is not running. The queues are not valid */
			ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
				"ia_css_pipe_enqueue_buffer() leaving:"
				"queues unavailable\n");
			return IA_CSS_ERR_RESOURCE_NOT_AVAILABLE;
		}
		return_err = ia_css_bufq_enqueue_event(
				SP_SW_EVENT_ID_1,
				(uint8_t)thread_id,
				queue_id,
				0);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_pipe_enqueue_buffer() leave: return_err=%d\n",return_err);

	return return_err;
}

/*
 * TODO: Free up the hmm memory space.
	 */
enum ia_css_err
ia_css_pipe_dequeue_buffer(struct ia_css_pipe *pipe,
			   struct ia_css_buffer *buffer)
{
	enum ia_css_err return_err;
	enum sh_css_queue_id queue_id;
	hrt_vaddress ddr_buffer_addr;
	struct sh_css_hmm_buffer ddr_buffer;
	unsigned int i, found_record;
	enum ia_css_buffer_type buf_type;
	enum ia_css_pipe_id pipe_id;
	unsigned int thread_id;

	if (buffer == NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	if (pipe == NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;

	pipe_id = pipe->mode;

	buf_type = buffer->type;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_pipe_dequeue_buffer() enter: pipe_id=%d, buf_type=%d\n",
		(int)pipe_id, buf_type);

	ddr_buffer.kernel_ptr = 0;

	ia_css_pipeline_get_sp_thread_id(ia_css_pipe_get_pipe_num(pipe), &thread_id);
	ia_css_query_internal_queue_id(buf_type, thread_id, &queue_id);
	if ((queue_id <= SH_CSS_INVALID_QUEUE_ID) || (queue_id >= SH_CSS_MAX_NUM_QUEUES))
			return IA_CSS_ERR_INVALID_ARGUMENTS;
	if (!sh_css_sp_is_running()) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
			"ia_css_pipe_dequeue_buffer() leaving:"
			"queues unavailable\n");
		/* SP is not running. The queues are not valid */
		return IA_CSS_ERR_RESOURCE_NOT_AVAILABLE;
	}

	return_err = ia_css_bufq_dequeue_buffer(queue_id,
		(uint32_t *)&ddr_buffer_addr);

	if (return_err == IA_CSS_SUCCESS) {
		struct ia_css_frame *frame;
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
			     "ia_css_pipe_dequeue_buffer() receive vbuf=%x\n",
			     (int)ddr_buffer_addr);

		found_record = 0;
		for (i = 0; i < MAX_HMM_BUFFER_NUM; i++) {
			if (hmm_buffer_record_h[i] != NULL && hmm_buffer_record_h[i]->vptr == ddr_buffer_addr) {
				ia_css_rmgr_rel_vbuf(hmm_buffer_pool, &hmm_buffer_record_h[i]);
				hmm_buffer_record_h[i] = NULL;
				found_record = 1;
				break;
			}
		}

		assert(found_record == 1);

		mmgr_load(ddr_buffer_addr,
				&ddr_buffer,
				sizeof(struct sh_css_hmm_buffer));

		assert(ddr_buffer.kernel_ptr != 0);

		/* if the pointer is 0 we only want to return an error;
		   we do not want to touch buffer
		*/
		if (ddr_buffer.kernel_ptr == 0)
			return_err = IA_CSS_ERR_INTERNAL_ERROR;
		else {
			/* buffer->exp_id : all instances to be removed later once the driver change
			 * is completed. See patch #5758 for reference */
			buffer->exp_id = 0;
			buffer->driver_cookie = ddr_buffer.cookie_ptr;

			switch (buf_type) {
			case IA_CSS_BUFFER_TYPE_INPUT_FRAME:
			case IA_CSS_BUFFER_TYPE_OUTPUT_FRAME:
			case IA_CSS_BUFFER_TYPE_SEC_OUTPUT_FRAME:
				if ((pipe) && (pipe->stop_requested == true))
				{

#if defined(USE_INPUT_SYSTEM_VERSION_2) || defined(USE_INPUT_SYSTEM_VERSION_2401)
					free_mipi_frames(pipe, false);
#endif
					pipe->stop_requested = false;
				}
			case IA_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME:
			case IA_CSS_BUFFER_TYPE_SEC_VF_OUTPUT_FRAME:
				frame = (struct ia_css_frame*)HOST_ADDRESS(ddr_buffer.kernel_ptr);
				buffer->data.frame = frame;
				buffer->exp_id = ddr_buffer.payload.frame.exp_id;
				frame->exp_id = ddr_buffer.payload.frame.exp_id;
				/* JWU */
				frame->frame_nr= ddr_buffer.payload.frame.frame_nr;
				frame->isp_config_id = ddr_buffer.payload.frame.isp_parameters_id;
				if (ddr_buffer.payload.frame.flashed == 1)
					frame->flash_state =
						IA_CSS_FRAME_FLASH_STATE_PARTIAL;
				if (ddr_buffer.payload.frame.flashed == 2)
					frame->flash_state =
						IA_CSS_FRAME_FLASH_STATE_FULL;
				frame->valid = pipe->num_invalid_frames == 0;
				if (!frame->valid)
					pipe->num_invalid_frames--;
				if (frame->info.format == IA_CSS_FRAME_FORMAT_BINARY_8) {
#ifdef USE_INPUT_SYSTEM_VERSION_2401
					frame->planes.binary.size = frame->data_bytes;
#else
					frame->planes.binary.size =
					    sh_css_sp_get_binary_copy_size();
#endif
				}
				ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,"ia_css_pipe_dequeue_buffer() pipe number=%d, frame number=%p\n", pipe->pipe_num, frame->frame_nr);
				break;
			case IA_CSS_BUFFER_TYPE_3A_STATISTICS:
				buffer->data.stats_3a =
					(struct ia_css_isp_3a_statistics*)HOST_ADDRESS(ddr_buffer.kernel_ptr);
				buffer->exp_id = ddr_buffer.payload.s3a.exp_id;
				buffer->data.stats_3a->exp_id = ddr_buffer.payload.s3a.exp_id;
				break;
			case IA_CSS_BUFFER_TYPE_DIS_STATISTICS:
				buffer->data.stats_dvs =
					(struct ia_css_isp_dvs_statistics*)HOST_ADDRESS(ddr_buffer.kernel_ptr);
				buffer->exp_id = ddr_buffer.payload.dis.exp_id;
				buffer->data.stats_dvs->exp_id = ddr_buffer.payload.dis.exp_id;
				break;
			case IA_CSS_BUFFER_TYPE_LACE_STATISTICS:
#if defined(IS_ISP_2500_SYSTEM)
				buffer->data.stats_acc_lace =
					(ia_css_ptr)HOST_ADDRESS(ddr_buffer.kernel_ptr);
#endif
				break;
			case IA_CSS_BUFFER_TYPE_METADATA:
				buffer->data.metadata =
					(struct ia_css_metadata*)HOST_ADDRESS(ddr_buffer.kernel_ptr);
				buffer->exp_id = ddr_buffer.payload.metadata.exp_id;
				buffer->data.metadata->exp_id = ddr_buffer.payload.metadata.exp_id;
				break;
			default:
				return_err = IA_CSS_ERR_INTERNAL_ERROR;
				break;
			}
		}
	}


	/*
	 * Tell the SP which queues are not full,
	 * by sending the software event.
	 */
	if (return_err == IA_CSS_SUCCESS){
		if (!sh_css_sp_is_running()) {
			ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
				"ia_css_pipe_dequeue_buffer() leaving:"
				"queues unavailable\n");
			/* SP is not running. The queues are not valid */
			return IA_CSS_ERR_RESOURCE_NOT_AVAILABLE;
		}
		ia_css_bufq_enqueue_event(SP_SW_EVENT_ID_2,
					0,
					queue_id,
					0);
	}
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_pipe_dequeue_buffer() leave: buffer=%p\n", buffer);

	return return_err;
}

/*
 * Cannot Move this to event module as it is of ia_css_event_type which is declared in ia_css.h
 * TODO: modify and move it if possible.
 * */
static enum ia_css_event_type convert_event_sp_to_host_domain[] = {
	IA_CSS_EVENT_TYPE_OUTPUT_FRAME_DONE,	/**< Output frame ready. */
	IA_CSS_EVENT_TYPE_SECOND_OUTPUT_FRAME_DONE,	/**< Second output frame ready. */
	IA_CSS_EVENT_TYPE_VF_OUTPUT_FRAME_DONE,	/**< Viewfinder Output frame ready. */
	IA_CSS_EVENT_TYPE_SECOND_VF_OUTPUT_FRAME_DONE,	/**< Second viewfinder Output frame ready. */
	IA_CSS_EVENT_TYPE_3A_STATISTICS_DONE,	/**< Indication that 3A statistics are available. */
	IA_CSS_EVENT_TYPE_DIS_STATISTICS_DONE,	/**< Indication that DIS statistics are available. */
	IA_CSS_EVENT_TYPE_PIPELINE_DONE,	/**< Pipeline Done event, sent after last pipeline stage. */
	IA_CSS_EVENT_TYPE_FRAME_TAGGED,		/**< Frame tagged. */
	IA_CSS_EVENT_TYPE_INPUT_FRAME_DONE,	/**< Input frame ready. */
	IA_CSS_EVENT_TYPE_METADATA_DONE,	/**< Metadata ready. */
	IA_CSS_EVENT_TYPE_LACE_STATISTICS_DONE,	/**< Indication that LACE statistics are available. */
	IA_CSS_EVENT_TYPE_PORT_EOF,		/**< End Of Frame event, sent when in buffered sensor mode.  MUST BE LAST */
	0,					/** error if sp passes  SH_CSS_SP_EVENT_NR_OF_TYPES as a valid event. */
};

enum ia_css_err
ia_css_dequeue_event(struct ia_css_event *event)
{
	enum ia_css_pipe_id pipe_id;
	uint8_t payload[4] = {0,0,0,0};
	enum ia_css_err ret_err;

	if (event == NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;

        /*TODO:
	 * a) use generic decoding function , same as the one used by sp.
	 * b) group decode and dequeue into eventQueue module
	 */
	if (!sh_css_sp_is_running()) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
			"ia_css_dequeue_event() leaving:"
			"queues unavailable\n");
		/* SP is not running. The queues are not valid */
		return IA_CSS_ERR_RESOURCE_NOT_AVAILABLE;
	}

	/* dequeue the IRQ event */
	/* check whether the IRQ event is available or not */

	ret_err = ia_css_bufq_dequeue_event(payload);
	if (ret_err != IA_CSS_SUCCESS) {
		/* ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
			"ia_css_bufq_dequeue_event() returned error\n"); */
		return ret_err;
	} else {
		/*
		 * Tell the SP which queues are not full,
		 * by sending the software event.
		 */
		/* No need to check here as queue is definetly initialized
		 * by the time we reach here.*/
		ia_css_bufq_enqueue_event(SP_SW_EVENT_ID_3, 0, 0, 0);
	}

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_dequeue_event() enter: queue not empty\n");
	/*  queue contains an event, it is decoded into 4 bytes of payload,
	 *  convert sp event type in payload to host event type,
	 *  TODO: can this enum conversion be eliminated */
	event->type = convert_event_sp_to_host_domain[payload[0]];
	pipe_id = (enum ia_css_pipe_id)payload[2];

	if (event->type != IA_CSS_EVENT_TYPE_PORT_EOF) {
		/* pipe related events */
		event->pipe = find_pipe_by_num(payload[1]);
		if (!event->pipe)
		{
			/* an event is generated of a port that is not longer available*/
			return IA_CSS_ERR_RESOURCE_NOT_AVAILABLE;
		}
		event->port = 0;
		event->exp_id = 0;
		if (event->type == IA_CSS_EVENT_TYPE_FRAME_TAGGED)
		{
			/* find the capture pipe that goes with this */
			int i, n;
			n = event->pipe->stream->num_pipes;
			for (i = 0; i < n; i++)
			{
				if (event->pipe->stream->pipes[i]->config.mode == IA_CSS_PIPE_MODE_CAPTURE)
				{
					event->pipe = event->pipe->stream->pipes[i];
					break;
				}
			}
			event->exp_id = payload[3];
		}
	}
	else {
		/* handle IA_CSS_EVENT_TYPE_PORT_EOF */
		/* event is not directly correlated to a pipe */
		event->pipe = NULL;
		event->port = payload[1];
		event->exp_id = payload[3];
	}
#if 0
  /* temporary debug code to dump the intermediate vf buffer to raw file*/
	if (event->type == IA_CSS_EVENT_TYPE_VF_OUTPUT_FRAME_DONE)
	{
	  static int count = 0;
	  FILE* fp;
    char *xmem_y_addr  = (char*)(event->pipe->pipeline.stages->args.out_vf_frame->data + event->pipe->pipeline.stages->args.out_vf_frame->planes.yuyv.offset);
    unsigned stride = event->pipe->pipeline.stages->args.out_vf_frame->planes.yuyv.stride;
    unsigned height = event->pipe->pipeline.stages->args.out_vf_frame->planes.yuyv.height;
    //printf ("init_frame_pointers format %d\n", stage->frames.in.info.format);
    //if (stage->frames.in.info.format == IA_CSS_FRAME_FORMAT_YUV_LINE) {
    unsigned char pixel;
    unsigned char buf[5000];
    unsigned int i;
    mmgr_load(xmem_y_addr, &pixel, sizeof(char));
    printf ("vf_pp: line 0 pixel %d\n", pixel);
    mmgr_load(xmem_y_addr+stride, &pixel, sizeof(char));
    printf ("vf_pp: line 1 pixel %d\n", pixel);
    mmgr_load(xmem_y_addr+2*stride, &pixel, sizeof(char));
    printf ("vf_pp: line 2 pixel %d\n", pixel);
    mmgr_load(xmem_y_addr+3*stride, &pixel, sizeof(char));
    printf ("vf_pp: line 3 pixel %d\n", pixel);

    fp = fopen("dump.raw", "w+");

    for (i=0; i< height/2; i++)
    {
      mmgr_load(xmem_y_addr, buf, stride);
      fwrite(buf, 1, stride, fp);
      xmem_y_addr += stride;
      mmgr_load(xmem_y_addr, buf, stride);
      fwrite(buf, 1, stride, fp);
      xmem_y_addr += stride;
      xmem_y_addr += stride;

    }
    fclose(fp);

    count++;
	}
#endif
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_dequeue_event() leave: pipe_id=%d, event_id=%d\n",
				pipe_id, event->type);

	return IA_CSS_SUCCESS;
}

static enum ia_css_err
acc_start(struct ia_css_pipe *pipe)
{
	assert(pipe != NULL);
	assert(pipe->stream != NULL);

	return start_pipe(pipe, SH_CSS_PIPE_CONFIG_OVRD_NO_OVRD,
			pipe->stream->config.mode);
}

static enum ia_css_err
sh_css_pipe_start(struct ia_css_stream *stream)
{
	enum ia_css_err err = IA_CSS_SUCCESS;

	struct ia_css_pipe *pipe;
	enum ia_css_pipe_id pipe_id;
	unsigned int thread_id;

	assert(stream != NULL);
	pipe = stream->last_pipe;
	assert(pipe != NULL);

	pipe_id = pipe->mode;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"sh_css_pipe_start() enter: pipe_id=%d\n", pipe_id);
#ifdef __KERNEL__
	printk("sh_css_pipe_start() enter: pipe_id=%d\n", pipe_id);
#endif

	if(stream->started == true) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
			"Cannot start stream that is already started\n");
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
			"sh_css_pipe_start() leave: return_err=%d\n", err);
		return err;
	}

	pipe->stop_requested = false;

	switch (pipe_id) {
	case IA_CSS_PIPE_ID_PREVIEW:
		err = preview_start(pipe);
		break;
	case IA_CSS_PIPE_ID_VIDEO:
		err = video_start(pipe);
		break;
	case IA_CSS_PIPE_ID_CAPTURE:
		err = capture_start(pipe);
		break;
	case IA_CSS_PIPE_ID_YUVPP:
		err = yuvpp_start(pipe);
		break;
	case IA_CSS_PIPE_ID_ACC:
		err = acc_start(pipe);
		break;
	default:
		err = IA_CSS_ERR_INVALID_ARGUMENTS;
	}
	if (err != IA_CSS_SUCCESS) {
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"sh_css_pipe_start() leave: return_err=%d\n", err);
		return err;
	}

	/* Force ISP parameter calculation after a mode change
	 * Acceleration API examples pass NULL for stream but they
	 * don't use ISP parameters anyway. So this should be okay.
	 * The SP binary (jpeg) copy does not use any parameters.
	 */
	if (!copy_on_sp(pipe)) {
		sh_css_invalidate_params(stream);
		err = sh_css_param_update_isp_params(stream, true, NULL);
		if (err != IA_CSS_SUCCESS)
			return err;
	}

	ia_css_debug_pipe_graph_dump_epilogue();

	ia_css_pipeline_get_sp_thread_id(ia_css_pipe_get_pipe_num(pipe), &thread_id);

	if (!sh_css_sp_is_running()) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
			"sh_css_pipe_start() leaving:"
			"queue unavailable\n");
		/* SP is not running. The queues are not valid */
		return IA_CSS_ERR_RESOURCE_NOT_AVAILABLE;
	}
	ia_css_bufq_enqueue_event(SP_SW_EVENT_ID_4, (uint8_t)thread_id, 0, 0);

	/* in case of continuous capture mode, we also start capture thread and copy thread*/
	if (pipe->stream->config.continuous) {
		struct ia_css_pipe *copy_pipe = NULL;

		if (pipe_id == IA_CSS_PIPE_ID_PREVIEW)
			copy_pipe = pipe->pipe_settings.preview.copy_pipe;
		else if (pipe_id == IA_CSS_PIPE_ID_VIDEO)
			copy_pipe = pipe->pipe_settings.video.copy_pipe;

		assert(copy_pipe != NULL);
		if (copy_pipe == NULL)
			return IA_CSS_ERR_INTERNAL_ERROR;
		ia_css_pipeline_get_sp_thread_id(ia_css_pipe_get_pipe_num(copy_pipe), &thread_id);
		 /* by the time we reach here q is initialized and handle is available.*/
		ia_css_bufq_enqueue_event(SP_SW_EVENT_ID_4, (uint8_t)thread_id, 0,  0);
	}
	if (pipe->stream->cont_capt) {
		struct ia_css_pipe *capture_pipe = NULL;
		if (pipe_id == IA_CSS_PIPE_ID_PREVIEW)
			capture_pipe = pipe->pipe_settings.preview.capture_pipe;
		else if (pipe_id == IA_CSS_PIPE_ID_VIDEO)
			capture_pipe = pipe->pipe_settings.video.capture_pipe;

		assert(capture_pipe != NULL);
		if (capture_pipe == NULL)
			return IA_CSS_ERR_INTERNAL_ERROR;
		ia_css_pipeline_get_sp_thread_id(ia_css_pipe_get_pipe_num(capture_pipe), &thread_id);
		 /* by the time we reach here q is initialized and handle is available.*/
		ia_css_bufq_enqueue_event(SP_SW_EVENT_ID_4, (uint8_t)thread_id, 0,  0);
	}

	stream->started = true;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"sh_css_pipe_start() leave: return_err=%d\n", err);
	return err;
}

#if 0
static enum ia_css_err sh_css_pipeline_stop(
	struct ia_css_pipe *pipe)
{
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "sh_css_pipeline_stop() enter\n");
	(void)pipe;
	/* TO BE IMPLEMENTED */
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "sh_css_pipeline_stop() exit\n");
	return IA_CSS_SUCCESS;
}
#endif

/* this is a legacy from CSS 1.5, we still need this here because we need this info
   to configure the sensor ouput in CameraDriver, but it is not needed in real device.
   add this macro to make this explicit. */
#if defined(HRT_CSIM) || defined(HRT_RTL)
static enum ia_css_err
sh_css_pipe_get_input_resolution(struct ia_css_pipe *pipe,
				 unsigned int *width,
				 unsigned int *height)
{
	enum ia_css_err err = IA_CSS_SUCCESS;

	assert(pipe != NULL);
	assert(pipe->stream != NULL);
	assert(width != NULL);
	assert(height != NULL);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "sh_css_pipe_get_input_resolution() enter: void\n");
	if (pipe->mode == IA_CSS_PIPE_ID_CAPTURE &&
	    copy_on_sp(pipe) &&
	    pipe->stream->config.format == IA_CSS_STREAM_FORMAT_BINARY_8) {
		*width  = JPEG_BYTES;
		*height = 1;
		return IA_CSS_SUCCESS;
	}

	{
		const struct ia_css_binary *binary = NULL;
		if (pipe->mode == IA_CSS_PIPE_ID_PREVIEW) {
			ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
				"sh_css_pipe_get_input_resolution: preview\n");
			if (pipe->pipe_settings.preview.copy_binary.info)
				binary = &pipe->pipe_settings.preview.copy_binary;
			else
				binary = &pipe->pipe_settings.preview.preview_binary;

		} else if (pipe->mode == IA_CSS_PIPE_ID_VIDEO) {
			ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
				"sh_css_pipe_get_input_resolution: video\n");
			if (pipe->pipe_settings.video.copy_binary.info)
				binary = &pipe->pipe_settings.video.copy_binary;
			else
				binary = &pipe->pipe_settings.video.video_binary;
		} else if (pipe->mode == IA_CSS_PIPE_ID_CAPTURE) {
			ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
				"sh_css_pipe_get_input_resolution: capture\n");
			if (pipe->pipe_settings.capture.copy_binary.info)
				binary = &pipe->pipe_settings.capture.copy_binary;
			else if (pipe->pipe_settings.capture.primary_binary.info)
				binary = &pipe->pipe_settings.capture.primary_binary;
			else
				binary = &pipe->pipe_settings.capture.pre_isp_binary;
		} else if (pipe->mode == IA_CSS_PIPE_ID_YUVPP) {
			binary = &pipe->pipe_settings.yuvpp.copy_binary;
		}
		if (binary) {
			*width  = binary->in_frame_info.res.width +
#if !defined(HAS_NO_INPUT_SYSTEM) && !defined(HAS_NO_INPUT_FORMATTER)
				ia_css_ifmtr_columns_needed_for_bayer_order(&pipe->stream->config);
#else
				0;
#endif

			*height = binary->in_frame_info.res.height +
#if !defined(HAS_NO_INPUT_SYSTEM) && !defined(HAS_NO_INPUT_FORMATTER)
				ia_css_ifmtr_lines_needed_for_bayer_order(&pipe->stream->config);
#else
				0;
#endif
		} else
			err = IA_CSS_ERR_INTERNAL_ERROR;
	}
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"sh_css_pipe_get_input_resolution() leave: width=%d, height=%d err=%d\n",
		*width, *height, err);
	return err;
}
#endif

void
sh_css_enable_cont_capt(bool enable, bool stop_copy_preview)
{
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"sh_css_enable_cont_capt() enter: enable=%d\n", enable);
	//my_css.cont_capt = enable;
	my_css.stop_copy_preview = stop_copy_preview;
}

bool
sh_css_continuous_is_enabled(uint8_t pipe_num)
{
	struct ia_css_pipe *pipe;
	bool continuous;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "sh_css_continuous_is_enabled() enter: pipe_num=%d\n", pipe_num);

	pipe = find_pipe_by_num(pipe_num);
	continuous = pipe && pipe->stream->config.continuous;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"sh_css_continuous_is_enabled() leave: enable=%d\n",
		continuous);
	return continuous;
}

enum ia_css_err
ia_css_stream_get_max_buffer_depth(struct ia_css_stream *stream, int *buffer_depth)
{
	if (buffer_depth == NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_stream_get_max_buffer_depth() enter: void\n");
	(void)stream;
	*buffer_depth = NUM_CONTINUOUS_FRAMES;
	return IA_CSS_SUCCESS;
}

enum ia_css_err
ia_css_stream_set_buffer_depth(struct ia_css_stream *stream, int buffer_depth)
{
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_stream_set_buffer_depth() enter: num_frames=%d\n",buffer_depth);
	(void)stream;
	if (buffer_depth > NUM_CONTINUOUS_FRAMES || buffer_depth < 1)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	/* ok, value allowed */
	stream->config.target_num_cont_raw_buf = buffer_depth;
	// TODO: check what to regarding initialization
	return IA_CSS_SUCCESS;
}

enum ia_css_err
ia_css_stream_get_buffer_depth(struct ia_css_stream *stream, int *buffer_depth)
{
	if (buffer_depth == NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_stream_get_buffer_depth() enter: void\n");
	(void)stream;
	*buffer_depth = stream->config.target_num_cont_raw_buf;
	return IA_CSS_SUCCESS;
}

#if !defined(HAS_NO_INPUT_SYSTEM) && defined(USE_INPUT_SYSTEM_VERSION_2)
unsigned int
sh_css_get_mipi_sizes_for_check(const unsigned int port, const unsigned int idx)
{
	OP___assert(port < N_CSI_PORTS);
	OP___assert(idx  < IA_CSS_MIPI_SIZE_CHECK_MAX_NOF_ENTRIES_PER_PORT);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "sh_css_get_mipi_sizes_for_check(port %d, idx %d): %d\n",
		port, idx, my_css.mipi_sizes_for_check[port][idx]);
	return my_css.mipi_sizes_for_check[port][idx];
}
#endif

static enum ia_css_err sh_css_pipe_configure_output(
	struct ia_css_pipe *pipe,
	unsigned int width,
	unsigned int height,
	unsigned int padded_width,
	enum ia_css_frame_format format,
	unsigned int idx)
{
	enum ia_css_err err = IA_CSS_SUCCESS;

	assert(pipe != NULL);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "sh_css_pipe_configure_output() enter:\n");

	err = ia_css_util_check_res(width, height);
	if (err != IA_CSS_SUCCESS)
		return err;
	if (pipe->output_info[idx].res.width != width ||
	    pipe->output_info[idx].res.height != height ||
	    pipe->output_info[idx].format != format)
	{
		ia_css_frame_info_init(
				&pipe->output_info[idx],
				width,
				height,
				format,
				padded_width);
	}
	return IA_CSS_SUCCESS;
}

static enum ia_css_err
sh_css_pipe_get_grid_info(struct ia_css_pipe *pipe,
			  struct ia_css_grid_info *info)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct ia_css_binary *s3a_binary = NULL;

	assert(pipe != NULL);
	assert(info != NULL);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "sh_css_pipe_get_grid_info() enter:\n");

	s3a_binary = ia_css_pipe_get_3a_binary(pipe);
	if (s3a_binary)
		ia_css_binary_grid_info(s3a_binary, info);
	else
		memset(info, 0, sizeof(*info));

	return err;
}

/*
 * @GC: TEMPORARY CODE TO TEST DVS AGAINST THE REFERENCE
 * PLEASE DO NOT REMOVE IT!
 */
#if DVS_REF_TESTING
static enum ia_css_err alloc_frame_from_file(
	struct ia_css_pipe *pipe,
	int width,
	int height)
{
	FILE *fp;
	int len = 0, err;
	int bytes_per_pixel;
	const char *file = "../File_input/dvs_input2.yuv";
	char *y_buf, *u_buf, *v_buf;
	char *uv_buf;
	int offset = 0;
	int h, w;

	hrt_vaddress out_base_addr;
	hrt_vaddress out_y_addr;
	hrt_vaddress out_uv_addr;

	assert(pipe != NULL);

	out_base_addr = pipe->pipe_settings.video.delay_frames[0]->data;
	out_y_addr  = out_base_addr
		+ pipe->pipe_settings.video.delay_frames[0]->planes.yuv.y.offset;
	out_uv_addr = out_base_addr
		+ pipe->pipe_settings.video.delay_frames[0]->planes.yuv.u.offset;


	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "alloc_frame_from_file() enter:\n");


	bytes_per_pixel = sizeof(char);

	if (!file) {printf("Error: Input file for dvs is not specified\n"); return 1;}
	fp = fopen(file, "rb");
	if (!fp) {printf("Error: Input file for dvs is not found\n"); return 1;}

	err = fseek(fp, 0, SEEK_END);
	if (err) {
		fclose(fp);
		printf("Error: Fseek error\n");
		return 1;
	}
	len = ftell(fp);

	err = fseek(fp, 0, SEEK_SET);
	if (err) {
		fclose(fp);
		printf("Error: Fseek error2\n");
		return 1;
	}

	len = 2 * len / 3;
	if (len != width * height * bytes_per_pixel) {
		fclose(fp);
		printf("Error: File size mismatches with the internal resolution\n");
		return 1;
	}

	y_buf = (char *) malloc(len);
	u_buf = (char *) malloc(len/4);
	v_buf = (char *) malloc(len/4);
	uv_buf= (char *) malloc(len/2);

	fread(y_buf, 1, len, fp);
	fread(u_buf, 1, len/4, fp);
	fread(v_buf, 1, len/4, fp);

	for (h=0; h<height/2; h++) {
		for (w=0; w<width/2; w++) {
			*(uv_buf + offset + w) = *(u_buf++);
			*(uv_buf + offset + w + width/2) = *(v_buf++);
			//printf("width: %d\n", width);
			//printf("offset_u: %d\n", offset+w);
			//printf("offset_v: %d\n", offset+w+width/2);
		}
		offset += width;
	}

	mmgr_store(out_y_addr, y_buf, len);
	mmgr_store(out_uv_addr, uv_buf, len/2);

	out_base_addr = pipe->pipe_settings.video.delay_frames[1]->data;
	out_y_addr  = out_base_addr + pipe->pipe_settings.video.delay_frames[1]->planes.yuv.y.offset;
	out_uv_addr = out_base_addr + pipe->pipe_settings.video.delay_frames[1]->planes.yuv.u.offset;
	mmgr_store(out_y_addr, y_buf, len);
	mmgr_store(out_uv_addr, uv_buf, len/2);

	fclose(fp);

	return IA_CSS_SUCCESS;
}

/* MW: Why do we not pass the pointer to the struct ? */
static enum ia_css_err fill_ref_frame_for_dvs(
	struct ia_css_pipe *pipe,
	struct ia_css_frame_info ref_info)
{
	enum ia_css_err err = IA_CSS_SUCCESS;

assert(pipe != NULL);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "fill_ref_frame_for_dvs() enter:\n");
	/* Allocate tmp_frame which is used to store YUV420 input.
	 * Read YUV420 input from the file to tmp_frame.
	 * Convert from YUV420 to NV12 format */
	err = alloc_frame_from_file(pipe, ref_info.res.width, ref_info.res.height);

	return err;
}
#endif

#define SH_CSS_TNR_BIT_DEPTH 8
#if defined(IS_ISP_2500_SYSTEM)
#define SH_CSS_REF_BIT_DEPTH 12   /* Skylake: reference frame is in 12 bit YUV domain */
#else
#define SH_CSS_REF_BIT_DEPTH 8
#endif

static enum ia_css_err load_video_binaries(struct ia_css_pipe *pipe)
{
	struct ia_css_frame_info video_in_info, ref_info, tnr_info,
				 *video_vf_info, video_bds_out_info, *pipe_out_info, *pipe_vf_out_info;
	bool online;
	enum ia_css_err err = IA_CSS_SUCCESS;
#if !defined(IS_ISP_2500_SYSTEM)
	bool continuous = pipe->stream->config.continuous;
#endif
	int i;
	unsigned num_output_pins;
	bool resolution_differs = false;

	assert(pipe != NULL);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "load_video_binaries() enter:\n");
	/* we only test the video_binary because offline video doesn't need a
	 * vf_pp binary and online does not (always use) the copy_binary.
	 * All are always reset at the same time anyway.
	 */
	if (pipe->pipe_settings.video.video_binary.info)
		return IA_CSS_SUCCESS;

	pipe_out_info = &pipe->output_info[0];
	pipe_vf_out_info = &pipe->vf_output_info[0];
	online = pipe->stream->config.online;
	err = ia_css_util_check_input(&pipe->stream->config, !online, false);
	if (err != IA_CSS_SUCCESS)
		return err;
	/* cannot have online video and input_mode memory */
	if (online && pipe->stream->config.mode == IA_CSS_INPUT_MODE_MEMORY)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	if (pipe->enable_viewfinder) {
		err = ia_css_util_check_vf_out_info(pipe_out_info,
					pipe_vf_out_info);
		if (err != IA_CSS_SUCCESS)
			return err;
	} else {
		err = ia_css_frame_check_info(pipe_out_info);
		if (err != IA_CSS_SUCCESS)
			return err;
	}

	/* Video */
	if (pipe->enable_viewfinder){
		video_vf_info = pipe_vf_out_info;
		resolution_differs = (video_vf_info->res.width != pipe_out_info->res.width) ||
				(video_vf_info->res.height != pipe_out_info->res.height);
	}
	else {
		video_vf_info = NULL;
	}

	{
		struct ia_css_binary_descr video_descr;

		err = ia_css_pipe_get_video_binarydesc(pipe,
			&video_descr, &video_in_info, &video_bds_out_info, video_vf_info,
			pipe->stream->config.left_padding);
		if (err != IA_CSS_SUCCESS)
			return err;

		err = ia_css_binary_find(&video_descr,
					 &pipe->pipe_settings.video.video_binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	}

	num_output_pins = pipe->pipe_settings.video.video_binary.info->num_output_pins;
	/* This is where we set the flag for invalid first frame */
	if (video_vf_info || (pipe->dvs_frame_delay == IA_CSS_FRAME_DELAY_2) )
		pipe->num_invalid_frames = 2;
	else
		pipe->num_invalid_frames = 1;

#if !defined(IS_ISP_2500_SYSTEM)    // skycam doesn't need the copy stage
	/* Copy */
	if (!online && !continuous) {
		/* TODO: what exactly needs doing, prepend the copy binary to
		 *	 video base this only on !online?
		 */
		err = load_copy_binary(pipe,
				       &pipe->pipe_settings.video.copy_binary,
				       &pipe->pipe_settings.video.video_binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	}
#endif

	/* Viewfinder post-processing */
	if (pipe->enable_viewfinder &&  // only when viewfinder is enabled.
	   ((num_output_pins == 1)        // when the binary has a single output pin, we need vf_pp
      || ((num_output_pins == 2) && resolution_differs)) ) { // when the binary has dual output pin, we only need vf_pp in case the resolution is different.
		struct ia_css_binary_descr vf_pp_descr;

		ia_css_pipe_get_vfpp_binarydesc(pipe, &vf_pp_descr,
			&pipe->pipe_settings.video.video_binary.vf_frame_info,
			pipe_vf_out_info);
		err = ia_css_binary_find(&vf_pp_descr,
				&pipe->pipe_settings.video.vf_pp_binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	}

	/* yuv copy does not use reference frames */
	if (ia_css_util_is_input_format_yuv(pipe->stream->config.format)) {
		return err;
	}

	ref_info = pipe->pipe_settings.video.video_binary.internal_frame_info;
#if defined(IS_ISP_2500_SYSTEM)
	ref_info.format = IA_CSS_FRAME_FORMAT_YUV420_16;   /* Skylake: reference frame is in 12 bit YUV domain */
#else
	ref_info.format = IA_CSS_FRAME_FORMAT_YUV420;
#endif
	ref_info.raw_bit_depth = SH_CSS_REF_BIT_DEPTH;

	/*Allocate the exact number of required reference buffers */
	for (i = 0; i <= (int)pipe->dvs_frame_delay ; i++){
		if (pipe->pipe_settings.video.delay_frames[i]) {
			ia_css_frame_free(pipe->pipe_settings.video.delay_frames[i]);
			pipe->pipe_settings.video.delay_frames[i] = NULL;
		}
		err = ia_css_frame_allocate_from_info(
				&pipe->pipe_settings.video.delay_frames[i],
				&ref_info);
#ifdef HRT_CSIM
		ia_css_frame_zero(pipe->pipe_settings.video.delay_frames[i]);
#endif
		if (err != IA_CSS_SUCCESS)
			return err;
	}


#if DVS_REF_TESTING
	/* @GC: TEMPORARY CODE TO TEST DVS AGAINST THE REFERENCE
	 * To test dvs-6axis:
	 * 1. Enable this function call
	 * 2. Set "reqs.ref_out_requests" to "0" in lineloop.hive.c
	 */
	err = fill_ref_frame_for_dvs(pipe, ref_info);
	if (err != IA_CSS_SUCCESS)
		return err;
#endif


  if (pipe->pipe_settings.video.video_binary.info->sp.enable.block_output){
    tnr_info = pipe->pipe_settings.video.video_binary.out_frame_info[0];
  }
  else {
    tnr_info = pipe->pipe_settings.video.video_binary.internal_frame_info;
  }
	tnr_info.format = IA_CSS_FRAME_FORMAT_YUV_LINE;
	tnr_info.raw_bit_depth = SH_CSS_TNR_BIT_DEPTH;

	for (i = 0; i < NUM_VIDEO_TNR_FRAMES; i++) {
		if (pipe->pipe_settings.video.tnr_frames[i]) {
			ia_css_frame_free(pipe->pipe_settings.video.tnr_frames[i]);
			pipe->pipe_settings.video.tnr_frames[i] = NULL;
		}
		err = ia_css_frame_allocate_from_info(
				&pipe->pipe_settings.video.tnr_frames[i],
				&tnr_info);
#ifdef HRT_CSIM
		ia_css_frame_zero(pipe->pipe_settings.video.tnr_frames[i]);
#endif
		if (err != IA_CSS_SUCCESS)
			return err;
	}

	return IA_CSS_SUCCESS;
}

static void
unload_video_binaries(struct ia_css_pipe *pipe)
{
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "unload_video_binaries() enter:\n");

	ia_css_binary_unload(&pipe->pipe_settings.video.copy_binary);
	ia_css_binary_unload(&pipe->pipe_settings.video.video_binary);
	ia_css_binary_unload(&pipe->pipe_settings.video.vf_pp_binary);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "unload_video_binaries() leave:\n");
}

static enum ia_css_err video_start(struct ia_css_pipe *pipe)
{
	struct ia_css_binary *copy_binary;
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct ia_css_pipe *copy_pipe, *capture_pipe;
	enum sh_css_pipe_config_override copy_ovrd;
	enum ia_css_input_mode video_pipe_input_mode;

#ifdef USE_INPUT_SYSTEM_VERSION_2401
	struct ia_css_pipe *target_pipe = NULL;
#endif
	assert(pipe != NULL);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "video_start() enter:\n");

	video_pipe_input_mode = pipe->stream->config.mode;

	copy_pipe    = pipe->pipe_settings.video.copy_pipe;
	capture_pipe = pipe->pipe_settings.video.capture_pipe;

	copy_binary  = &pipe->pipe_settings.video.copy_binary;

	sh_css_metrics_start_frame();

	/* multi stream video needs mipi buffers */

#if defined(USE_INPUT_SYSTEM_VERSION_2) || defined(USE_INPUT_SYSTEM_VERSION_2401)
	send_mipi_frames(pipe);
#endif

	send_raw_frames(pipe);
	{
		unsigned int thread_id;

		ia_css_pipeline_get_sp_thread_id(ia_css_pipe_get_pipe_num(pipe), &thread_id);
		copy_ovrd = 1 << thread_id;

		if (pipe->stream->cont_capt) {
			ia_css_pipeline_get_sp_thread_id(ia_css_pipe_get_pipe_num(capture_pipe), &thread_id);
			copy_ovrd |= 1 << thread_id;
		}
	}

#if !defined(HAS_NO_INPUT_SYSTEM)
#if defined(USE_INPUT_SYSTEM_VERSION_2)
	err = sh_css_config_input_network(pipe, copy_binary);
#elif defined(USE_INPUT_SYSTEM_VERSION_2401)
	if (pipe->stream->config.continuous) {
		/* make the copy pipe create/own the ISYS stream */
		target_pipe = copy_pipe;
	} else {
		/* make the video pipe create/own the ISYS stream */
		target_pipe = pipe;
	}

	err = sh_css_config_input_network(target_pipe, NULL);
#endif
	if (err != IA_CSS_SUCCESS)
		goto EXIT;
#endif

	/* Construct and load the copy pipe */
	if (pipe->stream->config.continuous) {
		sh_css_sp_init_pipeline(&copy_pipe->pipeline,
			IA_CSS_PIPE_ID_COPY,
			(uint8_t)ia_css_pipe_get_pipe_num(copy_pipe),
			false,
			pipe->stream->config.pixels_per_clock == 2, false,
			false, pipe->required_bds_factor,
			copy_ovrd,
			pipe->stream->config.mode,
			&pipe->stream->config.metadata_config,
			&pipe->stream->info.metadata_info
#if !defined(HAS_NO_INPUT_SYSTEM)
			, pipe->stream->config.source.port.port
#endif
			);

		/* make the video pipe start with mem mode input, copy handles
		   the actual mode */
		video_pipe_input_mode = IA_CSS_INPUT_MODE_MEMORY;
	}

	/* Construct and load the capture pipe */
	if (pipe->stream->cont_capt) {
		sh_css_sp_init_pipeline(&capture_pipe->pipeline,
			IA_CSS_PIPE_ID_CAPTURE,
			(uint8_t)ia_css_pipe_get_pipe_num(capture_pipe),
			pipe->config.default_capture_config.enable_xnr,
			capture_pipe->stream->config.pixels_per_clock == 2,
			true, /* continuous */
			false, /* offline */
			capture_pipe->required_bds_factor,
			0,
			IA_CSS_INPUT_MODE_MEMORY,
			&pipe->stream->config.metadata_config,
			&pipe->stream->info.metadata_info
#if !defined(HAS_NO_INPUT_SYSTEM)
			, (mipi_port_ID_t)0
#endif
			);
	}

	err = start_pipe(pipe, copy_ovrd, video_pipe_input_mode);

#if !defined(HAS_NO_INPUT_SYSTEM)
EXIT:
#endif
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"video_start() leave: return (%d)\n", err);
	return err;
}

static
enum ia_css_err sh_css_pipe_get_viewfinder_frame_info(
	struct ia_css_pipe *pipe,
	struct ia_css_frame_info *info,
	unsigned int idx)
{
	assert(pipe != NULL);
	assert(info != NULL);

/* We could print the pointer as input arg, and the values as output */
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "sh_css_pipe_get_viewfinder_frame_info() enter: void\n");

	if ( pipe->mode == IA_CSS_PIPE_ID_CAPTURE &&
	    (pipe->config.default_capture_config.mode == IA_CSS_CAPTURE_MODE_RAW ||
	     pipe->config.default_capture_config.mode == IA_CSS_CAPTURE_MODE_BAYER))
		return IA_CSS_ERR_MODE_HAS_NO_VIEWFINDER;
	/* offline video does not generate viewfinder output */
	if ( pipe->mode == IA_CSS_PIPE_ID_VIDEO &&
	    !pipe->stream->config.online && !pipe->stream->config.continuous) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
			"sh_css_pipe_get_viewfinder_frame_info() leave: return_err=%d\n",
			IA_CSS_ERR_MODE_HAS_NO_VIEWFINDER);
		return IA_CSS_ERR_MODE_HAS_NO_VIEWFINDER;
	} else {
		*info = pipe->vf_output_info[idx];
	}

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"sh_css_pipe_get_viewfinder_frame_info() leave: \
		info.res.width=%d, info.res.height=%d, \
		info.padded_width=%d, info.format=%d, \
		info.raw_bit_depth=%d, info.raw_bayer_order=%d\n",
		info->res.width,info->res.height,
		info->padded_width,info->format,
		info->raw_bit_depth,info->raw_bayer_order);

	return IA_CSS_SUCCESS;
}

static enum ia_css_err
sh_css_pipe_configure_viewfinder(struct ia_css_pipe *pipe, unsigned int width,
				 unsigned int height, unsigned int min_width,
				 enum ia_css_frame_format format,
				 unsigned int idx)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	assert(pipe != NULL);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"sh_css_pipe_configure_viewfinder() enter: \
		width=%d, height=%d format=%d\n",
		width, height, format);

	err = ia_css_util_check_res(width, height);
	if (err != IA_CSS_SUCCESS) {
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "sh_css_pipe_configure_viewfinder() leave: return_err=%d\n",err);
		return err;
	}
	if (pipe->vf_output_info[idx].res.width != width ||
	    pipe->vf_output_info[idx].res.height != height ||
	    pipe->vf_output_info[idx].format != format) {
		ia_css_frame_info_init(&pipe->vf_output_info[idx], width, height,
				       format, min_width);
	}
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "sh_css_pipe_configure_viewfinder() leave: return_err=%d\n",IA_CSS_SUCCESS);
	return IA_CSS_SUCCESS;
}

static enum ia_css_err load_copy_binaries(struct ia_css_pipe *pipe)
{
	enum ia_css_err err = IA_CSS_SUCCESS;

	assert(pipe != NULL);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "load_copy_binaries() enter:\n");

	if (pipe->pipe_settings.capture.copy_binary.info)
		return IA_CSS_SUCCESS;

	err = ia_css_frame_check_info(&pipe->output_info[0]);
	if (err != IA_CSS_SUCCESS)
		return err;

	err = verify_copy_out_frame_format(pipe);
	if (err != IA_CSS_SUCCESS)
		return err;

	err = load_copy_binary(pipe,
				&pipe->pipe_settings.capture.copy_binary,
				NULL);
	if (err != IA_CSS_SUCCESS)
		return err;

	return err;
}

static bool need_capture_pp(
	const struct ia_css_pipe *pipe)
{
	const struct ia_css_frame_info *out_info = &pipe->output_info[0];
	assert(pipe != NULL);
	assert(pipe->mode == IA_CSS_PIPE_ID_CAPTURE);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "need_capture_pp() enter:\n");
	/* determine whether we need to use the capture_pp binary.
	 * This is needed for:
	 *   1. XNR or
	 *   2. Digital Zoom or
	 *   3. YUV downscaling
	 */
	if (pipe->out_yuv_ds_input_info.res.width &&
	    ((pipe->out_yuv_ds_input_info.res.width != out_info->res.width) ||
	     (pipe->out_yuv_ds_input_info.res.height != out_info->res.height)))
		return true;
	if (pipe->config.default_capture_config.enable_xnr)
		return true;

	if ((pipe->stream->isp_params_configs->dz_config.dx < HRT_GDC_N) ||
	    (pipe->stream->isp_params_configs->dz_config.dy < HRT_GDC_N) ||
	    pipe->config.enable_dz)
		return true;

	return false;
}

static enum ia_css_err load_primary_binaries(
	struct ia_css_pipe *pipe)
{
	bool online = false;
	bool memory = false;
	bool continuous = false;
	bool need_pp = false;
	bool need_isp_copy_binary = false;
#ifdef USE_INPUT_SYSTEM_VERSION_2401
	bool sensor = false;
#endif
	struct ia_css_frame_info prim_in_info,
				 prim_out_info, vf_info,
				 *vf_pp_in_info, *pipe_out_info,
				 *pipe_vf_out_info;
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct ia_css_capture_settings *mycs;

	assert(pipe != NULL);
	assert(pipe->stream != NULL);

	online = pipe->stream->config.online;
	memory = pipe->stream->config.mode == IA_CSS_INPUT_MODE_MEMORY;
	continuous = pipe->stream->config.continuous;
#ifdef USE_INPUT_SYSTEM_VERSION_2401
	sensor = (pipe->stream->config.mode == IA_CSS_INPUT_MODE_SENSOR);
#endif

	mycs = &pipe->pipe_settings.capture;
	pipe_out_info = &pipe->output_info[0];
	pipe_vf_out_info = &pipe->vf_output_info[0];
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "load_primary_binaries() enter:\n");

	if (mycs->primary_binary.info)
		return IA_CSS_SUCCESS;

	err = ia_css_util_check_vf_out_info(pipe_out_info,
			pipe_vf_out_info);
	if (err != IA_CSS_SUCCESS)
		return err;
	need_pp = need_capture_pp(pipe);

	/* we use the vf output info to get the primary/capture_pp binary
	   configured for vf_veceven. It will select the closest downscaling
	   factor. */
	vf_info = *pipe_vf_out_info;
	ia_css_frame_info_set_format(&vf_info, IA_CSS_FRAME_FORMAT_YUV_LINE);

	/* we build up the pipeline starting at the end */
	/* Capture post-processing */
	if (need_pp) {
		struct ia_css_binary_descr capture_pp_descr;
		ia_css_pipe_get_capturepp_binarydesc(pipe,
			&capture_pp_descr, &prim_out_info, &vf_info);
		err = ia_css_binary_find(&capture_pp_descr,
					&mycs->capture_pp_binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	} else {
		prim_out_info = *pipe_out_info;
	}

	/* Primary */
	{
		struct ia_css_binary_descr prim_descr;

		ia_css_pipe_get_primary_binarydesc(pipe,
			&prim_descr, &prim_in_info, &prim_out_info, &vf_info);
		err = ia_css_binary_find(&prim_descr, &mycs->primary_binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	}

	/* Viewfinder post-processing */
	if (need_pp) {
		vf_pp_in_info =
		    &mycs->capture_pp_binary.vf_frame_info;
	} else {
		vf_pp_in_info =
		    &mycs->primary_binary.vf_frame_info;
	}

	{
		struct ia_css_binary_descr vf_pp_descr;

		ia_css_pipe_get_vfpp_binarydesc(pipe,
			&vf_pp_descr, vf_pp_in_info, pipe_vf_out_info);
		err = ia_css_binary_find(&vf_pp_descr, &mycs->vf_pp_binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	}

#ifdef USE_INPUT_SYSTEM_VERSION_2401
	/* When the input system is 2401, only the Direct Sensor Mode
	 * Offline Capture uses the ISP copy binary.
	 */
	need_isp_copy_binary = !online && sensor;
#else
	need_isp_copy_binary = !online && !continuous && !memory;
#endif
	/* ISP Copy */
	if (need_isp_copy_binary) {
		err = load_copy_binary(pipe,
				       &mycs->copy_binary,
				       &mycs->primary_binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	}

	return IA_CSS_SUCCESS;
}

static enum ia_css_err load_advanced_binaries(
	struct ia_css_pipe *pipe)
{
	struct ia_css_frame_info pre_in_info, gdc_in_info,
				 post_in_info, post_out_info,
				 vf_info, *vf_pp_in_info, *pipe_out_info,
				 *pipe_vf_out_info;
	bool need_pp;
	enum ia_css_err err = IA_CSS_SUCCESS;

	assert(pipe != NULL);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "load_advanced_binaries() enter:\n");

	if (pipe->pipe_settings.capture.pre_isp_binary.info)
		return IA_CSS_SUCCESS;
	pipe_out_info = &pipe->output_info[0];
	pipe_vf_out_info = &pipe->vf_output_info[0];

	vf_info = *pipe_vf_out_info;
	err = ia_css_util_check_vf_out_info(pipe_out_info, &vf_info);
	if (err != IA_CSS_SUCCESS)
		return err;
	need_pp = need_capture_pp(pipe);

	ia_css_frame_info_set_format(&vf_info,
				     IA_CSS_FRAME_FORMAT_YUV_LINE);

	/* we build up the pipeline starting at the end */
	/* Capture post-processing */
	if (need_pp) {
		struct ia_css_binary_descr capture_pp_descr;

		ia_css_pipe_get_capturepp_binarydesc(pipe,
			&capture_pp_descr, &post_out_info, &vf_info);
		err = ia_css_binary_find(&capture_pp_descr,
				&pipe->pipe_settings.capture.capture_pp_binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	} else {
		post_out_info = *pipe_out_info;
	}

	/* Post-gdc */
	{
		struct ia_css_binary_descr post_gdc_descr;

		ia_css_pipe_get_post_gdc_binarydesc(pipe,
			&post_gdc_descr, &post_in_info, &post_out_info, &vf_info);
		err = ia_css_binary_find(&post_gdc_descr,
					 &pipe->pipe_settings.capture.post_isp_binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	}

	/* Gdc */
	{
		struct ia_css_binary_descr gdc_descr;

		ia_css_pipe_get_gdc_binarydesc(pipe, &gdc_descr, &gdc_in_info,
			       &pipe->pipe_settings.capture.post_isp_binary.in_frame_info);
		err = ia_css_binary_find(&gdc_descr,
					 &pipe->pipe_settings.capture.anr_gdc_binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	}
	pipe->pipe_settings.capture.anr_gdc_binary.left_padding =
		pipe->pipe_settings.capture.post_isp_binary.left_padding;

	/* Pre-gdc */
	{
		struct ia_css_binary_descr pre_gdc_descr;

		ia_css_pipe_get_pre_gdc_binarydesc(pipe, &pre_gdc_descr, &pre_in_info,
				   &pipe->pipe_settings.capture.anr_gdc_binary.in_frame_info);
		err = ia_css_binary_find(&pre_gdc_descr,
					 &pipe->pipe_settings.capture.pre_isp_binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	}
	pipe->pipe_settings.capture.pre_isp_binary.left_padding =
		pipe->pipe_settings.capture.anr_gdc_binary.left_padding;

	/* Viewfinder post-processing */
	if (need_pp) {
		vf_pp_in_info =
		    &pipe->pipe_settings.capture.capture_pp_binary.vf_frame_info;
	} else {
		vf_pp_in_info =
		    &pipe->pipe_settings.capture.post_isp_binary.vf_frame_info;
	}

	{
		struct ia_css_binary_descr vf_pp_descr;

		ia_css_pipe_get_vfpp_binarydesc(pipe,
			&vf_pp_descr, vf_pp_in_info, pipe_vf_out_info);
		err = ia_css_binary_find(&vf_pp_descr,
					 &pipe->pipe_settings.capture.vf_pp_binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	}

	/* Copy */
	err = load_copy_binary(pipe,
			       &pipe->pipe_settings.capture.copy_binary,
			       &pipe->pipe_settings.capture.pre_isp_binary);
	if (err != IA_CSS_SUCCESS)
		return err;

	return IA_CSS_SUCCESS;
}

static enum ia_css_err load_bayer_isp_binaries(
	struct ia_css_pipe *pipe)
{
	struct ia_css_frame_info pre_isp_in_info, *pipe_out_info;
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct ia_css_binary_descr pre_de_descr;

	assert(pipe != NULL);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "load_pre_isp_binaries() enter:\n");
	pipe_out_info = &pipe->output_info[0];

	if (pipe->pipe_settings.capture.pre_isp_binary.info)
		return IA_CSS_SUCCESS;

	err = ia_css_frame_check_info(pipe_out_info);
	if (err != IA_CSS_SUCCESS)
		return err;

	ia_css_pipe_get_pre_de_binarydesc(pipe, &pre_de_descr,
				&pre_isp_in_info,
				pipe_out_info);

	err = ia_css_binary_find(&pre_de_descr,
				 &pipe->pipe_settings.capture.pre_isp_binary);

	return err;
}

static enum ia_css_err load_low_light_binaries(
	struct ia_css_pipe *pipe)
{
	struct ia_css_frame_info pre_in_info, anr_in_info,
				 post_in_info, post_out_info,
				 vf_info, *pipe_vf_out_info, *pipe_out_info,
				 *vf_pp_in_info;
	bool need_pp;
	enum ia_css_err err = IA_CSS_SUCCESS;

	assert(pipe != NULL);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "load_low_light_binaries() enter:\n");

	if (pipe->pipe_settings.capture.pre_isp_binary.info)
		return IA_CSS_SUCCESS;
	pipe_vf_out_info = &pipe->vf_output_info[0];
	pipe_out_info = &pipe->output_info[0];

	vf_info = *pipe_vf_out_info;
	err = ia_css_util_check_vf_out_info(pipe_out_info,
				&vf_info);
	if (err != IA_CSS_SUCCESS)
		return err;
	need_pp = need_capture_pp(pipe);

	ia_css_frame_info_set_format(&vf_info,
				     IA_CSS_FRAME_FORMAT_YUV_LINE);

	/* we build up the pipeline starting at the end */
	/* Capture post-processing */
	if (need_pp) {
		struct ia_css_binary_descr capture_pp_descr;

		ia_css_pipe_get_capturepp_binarydesc(pipe,
			&capture_pp_descr, &post_out_info, &vf_info);
		err = ia_css_binary_find(&capture_pp_descr,
				&pipe->pipe_settings.capture.capture_pp_binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	} else {
		post_out_info = *pipe_out_info;
	}

	/* Post-anr */
	{
		struct ia_css_binary_descr post_anr_descr;

		ia_css_pipe_get_post_anr_binarydesc(pipe,
			&post_anr_descr, &post_in_info, &post_out_info, &vf_info);
		err = ia_css_binary_find(&post_anr_descr,
					 &pipe->pipe_settings.capture.post_isp_binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	}

	/* Anr */
	{
		struct ia_css_binary_descr anr_descr;

		ia_css_pipe_get_anr_binarydesc(pipe, &anr_descr, &anr_in_info,
				&pipe->pipe_settings.capture.post_isp_binary.in_frame_info);
		err = ia_css_binary_find(&anr_descr,
					 &pipe->pipe_settings.capture.anr_gdc_binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	}
	pipe->pipe_settings.capture.anr_gdc_binary.left_padding =
		pipe->pipe_settings.capture.post_isp_binary.left_padding;

	/* Pre-anr */
	{
		struct ia_css_binary_descr pre_anr_descr;

		ia_css_pipe_get_pre_anr_binarydesc(pipe, &pre_anr_descr, &pre_in_info,
				   &pipe->pipe_settings.capture.anr_gdc_binary.in_frame_info);
		err = ia_css_binary_find(&pre_anr_descr,
				&pipe->pipe_settings.capture.pre_isp_binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	}
	pipe->pipe_settings.capture.pre_isp_binary.left_padding =
		pipe->pipe_settings.capture.anr_gdc_binary.left_padding;

	/* Viewfinder post-processing */
	if (need_pp) {
		vf_pp_in_info =
		    &pipe->pipe_settings.capture.capture_pp_binary.vf_frame_info;
	} else {
		vf_pp_in_info =
		    &pipe->pipe_settings.capture.post_isp_binary.vf_frame_info;
	}

	{
		struct ia_css_binary_descr vf_pp_descr;

		ia_css_pipe_get_vfpp_binarydesc(pipe,
			&vf_pp_descr, vf_pp_in_info, pipe_vf_out_info);
		err = ia_css_binary_find(&vf_pp_descr,
					 &pipe->pipe_settings.capture.vf_pp_binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	}

	/* Copy */
	err = load_copy_binary(pipe,
			       &pipe->pipe_settings.capture.copy_binary,
			       &pipe->pipe_settings.capture.pre_isp_binary);
	if (err != IA_CSS_SUCCESS)
		return err;

	return IA_CSS_SUCCESS;
}

static bool copy_on_sp(struct ia_css_pipe *pipe)
{
	bool rval;

	assert(pipe != NULL);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "copy_on_sp() enter:\n");

	rval = true;

	rval &=	(pipe->mode == IA_CSS_PIPE_ID_CAPTURE);

	rval &= (pipe->config.default_capture_config.mode == IA_CSS_CAPTURE_MODE_RAW);

	rval &= ((pipe->stream->config.format == IA_CSS_STREAM_FORMAT_BINARY_8) ||
		(pipe->config.mode == IA_CSS_PIPE_MODE_COPY));

	return rval;
}

static enum ia_css_err load_capture_binaries(
	struct ia_css_pipe *pipe)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	bool must_be_raw;

	assert(pipe != NULL);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "load_capture_binaries() enter:\n");

	if (pipe->pipe_settings.capture.primary_binary.info)
		return IA_CSS_SUCCESS;

	/* in primary, advanced,low light or bayer,
						the input format must be raw */
	must_be_raw =
		pipe->config.default_capture_config.mode == IA_CSS_CAPTURE_MODE_ADVANCED ||
		pipe->config.default_capture_config.mode == IA_CSS_CAPTURE_MODE_BAYER ||
		pipe->config.default_capture_config.mode == IA_CSS_CAPTURE_MODE_LOW_LIGHT;
	err = ia_css_util_check_input(&pipe->stream->config, must_be_raw, false);
	if (err != IA_CSS_SUCCESS)
		return err;
	if (copy_on_sp(pipe) &&
	    pipe->stream->config.format == IA_CSS_STREAM_FORMAT_BINARY_8) {
		ia_css_frame_info_init(
			&pipe->output_info[0],
			JPEG_BYTES,
			1,
			IA_CSS_FRAME_FORMAT_BINARY_8,
			0);
		return IA_CSS_SUCCESS;
	}

	switch (pipe->config.default_capture_config.mode) {
	case IA_CSS_CAPTURE_MODE_RAW:
		err = load_copy_binaries(pipe);
		break;
	case IA_CSS_CAPTURE_MODE_BAYER:
		err = load_bayer_isp_binaries(pipe);
		break;
	case IA_CSS_CAPTURE_MODE_PRIMARY:
		err = load_primary_binaries(pipe);
		break;
	case IA_CSS_CAPTURE_MODE_ADVANCED:
		err = load_advanced_binaries(pipe);
		break;
	case IA_CSS_CAPTURE_MODE_LOW_LIGHT:
		err = load_low_light_binaries(pipe);
		break;
	}
	if (err != IA_CSS_SUCCESS)
		return err;

	return err;
}

static void
unload_capture_binaries(struct ia_css_pipe *pipe)
{
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "unload_capture_binaries() enter:\n");

	ia_css_binary_unload(&pipe->pipe_settings.capture.copy_binary);
	ia_css_binary_unload(&pipe->pipe_settings.capture.primary_binary);
	ia_css_binary_unload(&pipe->pipe_settings.capture.pre_isp_binary);
	ia_css_binary_unload(&pipe->pipe_settings.capture.anr_gdc_binary);
	ia_css_binary_unload(&pipe->pipe_settings.capture.post_isp_binary);
	ia_css_binary_unload(&pipe->pipe_settings.capture.capture_pp_binary);
	ia_css_binary_unload(&pipe->pipe_settings.capture.vf_pp_binary);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "unload_capture_binaries() leave:\n");
}

static bool
need_yuv_scaler(const struct ia_css_pipe *pipe)
{
	assert(pipe != NULL);
	assert(pipe->mode == IA_CSS_PIPE_ID_YUVPP);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "need_yuv_scaler() enter:\n");

	if (((pipe->output_info[0].res.width != 0) && (pipe->output_info[0].res.width < pipe->stream->info.effective_info.width)) ||
		((pipe->output_info[1].res.width != 0) && (pipe->output_info[1].res.width < pipe->stream->info.effective_info.width)) ||
		pipe->config.enable_dz)
		return true;

	return false;
}

static enum ia_css_err ia_css_pipe_get_cas_scaler_desc(struct ia_css_pipe *pipe,
	struct ia_css_cas_binary_descr *descr)
{
	struct ia_css_frame_info in_info = IA_CSS_BINARY_DEFAULT_FRAME_INFO;
	struct ia_css_frame_info *out_info[IA_CSS_PIPE_MAX_OUTPUT_STAGE];
	struct ia_css_frame_info *vf_out_info[IA_CSS_PIPE_MAX_OUTPUT_STAGE];
	struct ia_css_frame_info tmp_in_info = IA_CSS_BINARY_DEFAULT_FRAME_INFO;
	unsigned int i, j;
	unsigned int hor_ds_factor[IA_CSS_PIPE_MAX_OUTPUT_STAGE],
				ver_ds_factor[IA_CSS_PIPE_MAX_OUTPUT_STAGE],
				ds_factor = 0;
	unsigned int num_stages = 0;
	enum ia_css_err err = IA_CSS_SUCCESS;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "ia_css_pipe_get_cas_scaler_desc() enter:\n");

	for (i = 0; i < IA_CSS_PIPE_MAX_OUTPUT_STAGE; i++) {
		out_info[i] = NULL;
		vf_out_info[i] = NULL;
		hor_ds_factor[i] = 0;
		ver_ds_factor[i] = 0;
	}

	in_info.res = pipe->stream->info.effective_info;
	in_info.padded_width = in_info.res.width;
	descr->num_output_stage = 0;
	for (i = 0; i < IA_CSS_PIPE_MAX_OUTPUT_STAGE; i++) {
		if (pipe->output_info[i].res.width != 0) {
			out_info[i] = &pipe->output_info[i];
			if (pipe->vf_output_info[i].res.width != 0)
				vf_out_info[i] = &pipe->vf_output_info[i];
			descr->num_output_stage += 1;
		}

		if (out_info[i] != NULL) {
			hor_ds_factor[i] = CEIL_DIV(in_info.res.width, out_info[i]->res.width);
			ver_ds_factor[i] = CEIL_DIV(in_info.res.height, out_info[i]->res.height);
			/* use the same horizontal and vertical downscaling factor for simplicity */
			assert(hor_ds_factor[i] == ver_ds_factor[i]);
		}
		if (hor_ds_factor[i] > ds_factor)
			ds_factor = hor_ds_factor[i];
	}
#if 0
	hor_ds_factor = MAX(hor_ds_factor_a, hor_ds_factor_b);
	ver_ds_factor = MAX(ver_ds_factor_a, ver_ds_factor_b);
	while (hor_ds_factor > 1) {
		hor_num_stages++;
		hor_ds_factor /= 2;
	}
	while (ver_ds_factor > 1) {
		ver_num_stages++;
		ver_ds_factor /= 2;
	}
	assert(hor_num_stages == ver_num_stages);
	descr->num_stage = MAX(hor_num_stages, ver_num_stages);
#else
	i = 1;
	while (i < ds_factor) {
		num_stages++;
		i *= 2;
	}
	if (pipe->config.enable_dz && (num_stages == 0))
		num_stages = 1;
	descr->num_stage = num_stages;
#endif

	/* if two outputs requires the same number of downscaling stages, we
	 * extent it by one because we only have fixed number of output pins. */
	if ((out_info[0] != NULL) && (out_info[1] != NULL) &&
		(out_info[0]->res.width == out_info[1]->res.width) &&
		(descr->num_output_stage > 1)) {
		descr->num_stage += 1;
	}
	descr->in_info = sh_css_malloc(descr->num_stage * sizeof(struct ia_css_frame_info));
	if (descr->in_info == NULL) {
		err = IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
		goto ERR;
	}
	descr->internal_out_info = sh_css_malloc(descr->num_stage * sizeof(struct ia_css_frame_info));
	if (descr->internal_out_info == NULL) {
		err = IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
		goto ERR;
	}
	descr->out_info = sh_css_malloc(descr->num_stage * sizeof(struct ia_css_frame_info));
	if (descr->out_info == NULL) {
		err = IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
		goto ERR;
	}
	descr->vf_info = sh_css_malloc(descr->num_stage * sizeof(struct ia_css_frame_info));
	if (descr->vf_info == NULL) {
		err = IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
		goto ERR;
	}
	descr->is_output_stage = sh_css_malloc(descr->num_stage * sizeof(bool));
	if (descr->is_output_stage == NULL) {
		err = IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
		goto ERR;
	}

#if 0
	/* swap the frame info if sec_output is bigger than output */
	if ((out_info->res.width < sec_out_info->res.width) &&
		(out_info->res.height < sec_out_info->res.height)) {
		tmp_out_info[0] = sec_out_info;
		tmp_out_info[1] = out_info;
		tmp_vf_out_info[0] = secvf__out_info;
		tmp_vf_out_info[1] = vf_out_info;
	} else {
		tmp_out_info[0] = out_info;
		tmp_out_info[1] = sec_out_info;
		tmp_vf_out_info[0] = vf_out_info;
		tmp_vf_out_info[1] = sec_vf_out_info;
	}
#else
	for (i = 0; i < IA_CSS_PIPE_MAX_OUTPUT_STAGE; i++) {
		if (out_info[i]) {
			if (i > 0) {
				assert((out_info[i-1]->res.width >= out_info[i]->res.width) &&
						(out_info[i-1]->res.height >= out_info[i]->res.height));
			}
		}
	}
#endif

	tmp_in_info.res = pipe->stream->info.effective_info;
	tmp_in_info.format = IA_CSS_FRAME_FORMAT_YUV420;
	for (i = 0, j = 0; i < descr->num_stage; i++) {
		assert(j < 2);
		assert(out_info[j] != NULL);

		descr->in_info[i] = tmp_in_info;
		if ((tmp_in_info.res.width / 2) <= out_info[j]->res.width) {
			descr->is_output_stage[i] = true;
			if ((descr->num_output_stage > 1) && (i != (descr->num_stage - 1))) {
				descr->internal_out_info[i].res.width = out_info[j]->res.width;
				descr->internal_out_info[i].res.height = out_info[j]->res.height;
				descr->internal_out_info[i].padded_width = out_info[j]->padded_width;
				descr->internal_out_info[i].format = IA_CSS_FRAME_FORMAT_YUV420;
			} else {
				assert(i == (descr->num_stage - 1));
				descr->internal_out_info[i].res.width = 0;
				descr->internal_out_info[i].res.height = 0;
			}
			descr->out_info[i].res.width = out_info[j]->res.width;
			descr->out_info[i].res.height = out_info[j]->res.height;
			descr->out_info[i].padded_width = out_info[j]->padded_width;
			descr->out_info[i].format = out_info[j]->format;
			descr->vf_info[i].res.width = vf_out_info[j]->res.width;
			descr->vf_info[i].res.height = vf_out_info[j]->res.height;
			descr->vf_info[i].padded_width = vf_out_info[j]->padded_width;
			ia_css_frame_info_set_format(&descr->vf_info[i], IA_CSS_FRAME_FORMAT_YUV_LINE);
			//descr->vf_info[i].format = tmp_vf_out_info[j]->format;
			j++;
		} else {
			descr->is_output_stage[i] = false;
			descr->internal_out_info[i].res.width = tmp_in_info.res.width / 2;
			descr->internal_out_info[i].res.height = tmp_in_info.res.height / 2;
			descr->internal_out_info[i].format = IA_CSS_FRAME_FORMAT_YUV420;
			ia_css_frame_info_init(&descr->internal_out_info[i], tmp_in_info.res.width / 2,
					tmp_in_info.res.height / 2, IA_CSS_FRAME_FORMAT_YUV420, 0);
			descr->out_info[i].res.width = 0;
			descr->out_info[i].res.height = 0;
			descr->vf_info[i].res.width = 0;
			descr->vf_info[i].res.height = 0;
		}
		tmp_in_info = descr->internal_out_info[i];
	}
ERR:
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "ia_css_pipe_get_cas_scaler_desc() leave, err=%d\n",
			err);
	return err;
}

static enum ia_css_err
load_yuvpp_binaries(struct ia_css_pipe *pipe)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	bool need_scaler = false;
	struct ia_css_frame_info *vf_pp_in_info[IA_CSS_PIPE_MAX_OUTPUT_STAGE];
	struct ia_css_yuvpp_settings *mycs;
	struct ia_css_binary *next_binary;
	struct ia_css_cas_binary_descr cas_scaler_descr = IA_CSS_DEFAULT_CAS_BINARY_DESCR;
	unsigned int i, j;

	assert(pipe != NULL);
	assert(pipe->stream != NULL);

	if (pipe->pipe_settings.yuvpp.copy_binary.info)
		goto ERR;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "load_yuvpp_binaries() enter:\n");

	err = ia_css_util_check_input(&pipe->stream->config, false, true);
	if (err != IA_CSS_SUCCESS)
		goto ERR;

	mycs = &pipe->pipe_settings.yuvpp;

	err = ia_css_util_check_vf_out_info(&pipe->output_info[0],
			&pipe->vf_output_info[0]);
	if (err != IA_CSS_SUCCESS)
		goto ERR;

	for (i = 0; i < IA_CSS_PIPE_MAX_OUTPUT_STAGE; i++) {
		vf_pp_in_info[i] = NULL;
	}

	need_scaler = need_yuv_scaler(pipe);

	/* we use the vf output info to get the primary/capture_pp binary
	   configured for vf_veceven. It will select the closest downscaling
	   factor. */
	//vf_info = pipe->vf_output_info;
	//ia_css_frame_info_set_format(&vf_info, IA_CSS_FRAME_FORMAT_YUV_LINE);

	/* we build up the pipeline starting at the end */
	/* Capture post-processing */
	if (need_scaler) {
		struct ia_css_binary_descr yuv_scaler_descr;

		ia_css_pipe_get_cas_scaler_desc(pipe,
			&cas_scaler_descr);
		mycs->num_output = cas_scaler_descr.num_output_stage;
		mycs->num_yuv_scaler = cas_scaler_descr.num_stage;
		mycs->yuv_scaler_binary = sh_css_calloc(cas_scaler_descr.num_stage,
			sizeof(struct ia_css_binary));
		if (mycs->yuv_scaler_binary == NULL) {
			err = IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
			goto ERR;
		}
		mycs->is_output_stage = sh_css_calloc(cas_scaler_descr.num_stage,
			sizeof(bool));
		if (mycs->is_output_stage == NULL) {
			err = IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
			goto ERR;
		}
		for (i = 0; i < cas_scaler_descr.num_stage; i++) {
			mycs->is_output_stage[i] = cas_scaler_descr.is_output_stage[i];
			ia_css_pipe_get_yuvscaler_binarydesc(pipe,
				&yuv_scaler_descr, &cas_scaler_descr.in_info[i],
				&cas_scaler_descr.out_info[i],
				&cas_scaler_descr.internal_out_info[i],
				&cas_scaler_descr.vf_info[i]);
			err = ia_css_binary_find(&yuv_scaler_descr,
						&mycs->yuv_scaler_binary[i]);
			if (err != IA_CSS_SUCCESS)
				goto ERR;
		}
	} else {
		mycs->num_output = 1;
	}

	if (need_scaler) {
		next_binary = &mycs->yuv_scaler_binary[0];
	} else {
		next_binary = NULL;
	}
	/* ISP Copy */
	err = load_copy_binary(pipe,
			       &mycs->copy_binary,
			       next_binary);
	if (err != IA_CSS_SUCCESS)
		goto ERR;

	/* Viewfinder post-processing */
	if (need_scaler) {
		for (i = 0, j = 0; i < cas_scaler_descr.num_stage; i++) {
			if (cas_scaler_descr.is_output_stage[i]) {
				assert(j < 2);
				vf_pp_in_info[j] =
					&mycs->yuv_scaler_binary[i].vf_frame_info;
				j++;
			}
		}
		mycs->num_vf_pp = j;
	} else {
		vf_pp_in_info[0] =
		    &mycs->copy_binary.vf_frame_info;
		for (i = 1; i < IA_CSS_PIPE_MAX_OUTPUT_STAGE; i++) {
			vf_pp_in_info[i] = NULL;
		}
		mycs->num_vf_pp = 1;
	}
	mycs->vf_pp_binary = sh_css_calloc(mycs->num_vf_pp, sizeof(struct ia_css_binary));
	if (mycs->vf_pp_binary == NULL) {
		err = IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
		goto ERR;
	}

	{
		struct ia_css_binary_descr vf_pp_descr;

		for (i = 0; i < mycs->num_vf_pp; i++) {
			ia_css_pipe_get_vfpp_binarydesc(pipe,
				&vf_pp_descr, vf_pp_in_info[i], &pipe->vf_output_info[i]);
			err = ia_css_binary_find(&vf_pp_descr, &mycs->vf_pp_binary[i]);
			if (err != IA_CSS_SUCCESS)
				goto ERR;
		}
	}

	if (err != IA_CSS_SUCCESS)
		goto ERR;

ERR:
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "load_yuvpp_binaries() leave, err=%d\n",
			err);
	return err;
}

static void
unload_yuvpp_binaries(struct ia_css_pipe *pipe)
{
	unsigned int i;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "unload_yuvpp_binaries() enter:\n");

	ia_css_binary_unload(&pipe->pipe_settings.yuvpp.copy_binary);
	for (i = 0; i < pipe->pipe_settings.yuvpp.num_yuv_scaler; i++) {
		ia_css_binary_unload(&pipe->pipe_settings.yuvpp.yuv_scaler_binary[i]);
	}
	for (i = 0; i < pipe->pipe_settings.yuvpp.num_vf_pp; i++) {
		ia_css_binary_unload(&pipe->pipe_settings.yuvpp.vf_pp_binary[i]);
	}

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "unload_yuvpp_binaries() leave:\n");
}

static enum ia_css_err yuvpp_start(struct ia_css_pipe *pipe)
{
	struct ia_css_binary *copy_binary;
	enum ia_css_err err = IA_CSS_SUCCESS;
	enum sh_css_pipe_config_override copy_ovrd;
	enum ia_css_input_mode yuvpp_pipe_input_mode;

#ifdef USE_INPUT_SYSTEM_VERSION_2401
	struct ia_css_pipe *target_pipe = NULL;
#endif
	assert(pipe != NULL);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "yuvpp_start() enter:\n");

	yuvpp_pipe_input_mode = pipe->stream->config.mode;

	copy_binary  = &pipe->pipe_settings.yuvpp.copy_binary;

	sh_css_metrics_start_frame();

	/* multi stream video needs mipi buffers */

#if !defined(HAS_NO_INPUT_SYSTEM) && ( defined(USE_INPUT_SYSTEM_VERSION_2) || defined(USE_INPUT_SYSTEM_VERSION_2401) )
	send_mipi_frames(pipe);
#endif

	{
		unsigned int thread_id;

		ia_css_pipeline_get_sp_thread_id(ia_css_pipe_get_pipe_num(pipe), &thread_id);
		copy_ovrd = 1 << thread_id;
	}

#if !defined(HAS_NO_INPUT_SYSTEM)
#if defined(USE_INPUT_SYSTEM_VERSION_2)
	err = sh_css_config_input_network(pipe, copy_binary);
#elif defined(USE_INPUT_SYSTEM_VERSION_2401)
	/* make the video pipe create/own the ISYS stream */
	target_pipe = pipe;

	err = sh_css_config_input_network(target_pipe, NULL);
#endif
	if (err != IA_CSS_SUCCESS)
		goto EXIT;
#endif

	err = start_pipe(pipe, copy_ovrd, yuvpp_pipe_input_mode);

#if !defined(HAS_NO_INPUT_SYSTEM)
EXIT:
#endif
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"video_start() leave: return (%d)\n", err);
	return err;
}

static enum ia_css_err
sh_css_pipe_load_binaries(struct ia_css_pipe *pipe)
{
	enum ia_css_err err = IA_CSS_SUCCESS;

	assert(pipe != NULL);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "sh_css_pipe_load_binaries() enter:\n");

	/* PIPE_MODE_COPY has no binaries, but has output frames to outside*/
	if (pipe->config.mode == IA_CSS_PIPE_MODE_COPY)
		return err;

	switch (pipe->mode) {
	case IA_CSS_PIPE_ID_PREVIEW:
		err = load_preview_binaries(pipe);
		break;
	case IA_CSS_PIPE_ID_VIDEO:
		err = load_video_binaries(pipe);
		break;
	case IA_CSS_PIPE_ID_CAPTURE:
		err = load_capture_binaries(pipe);
		break;
	case IA_CSS_PIPE_ID_YUVPP:
		err = load_yuvpp_binaries(pipe);
		break;
	default:
		err = IA_CSS_ERR_INTERNAL_ERROR;
		break;
	}
	return err;
}

static void
sh_css_pipe_unload_binaries(struct ia_css_pipe *pipe)
{
	assert(pipe != NULL);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "sh_css_pipe_unload_binaries() enter:\n");

	/* PIPE_MODE_COPY has no binaries, but has output frames to outside*/
	if (pipe->config.mode == IA_CSS_PIPE_MODE_COPY)
		return;

	switch (pipe->mode) {
	case IA_CSS_PIPE_ID_PREVIEW:
		unload_preview_binaries(pipe);
		break;
	case IA_CSS_PIPE_ID_VIDEO:
		unload_video_binaries(pipe);
		break;
	case IA_CSS_PIPE_ID_CAPTURE:
		unload_capture_binaries(pipe);
		break;
	case IA_CSS_PIPE_ID_YUVPP:
		unload_yuvpp_binaries(pipe);
		break;
	default:
		break;
	}
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "sh_css_pipe_unload_binaries() leave:\n");
}

static enum ia_css_err
create_host_yuvpp_pipeline(struct ia_css_pipe *pipe)
{
	struct ia_css_pipeline *me;
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct ia_css_pipeline_stage *vf_pp_stage = NULL,
				     *post_stage = NULL;
	struct ia_css_binary *copy_binary,
			     *vf_pp_binary,
			     *yuv_scaler_binary;
	bool need_scaler = false;
	unsigned int num_stage, num_vf_pp_stage, num_output_stage;
	unsigned int i, j;

	struct ia_css_frame *in_frame = NULL;
	struct ia_css_frame *out_frame[IA_CSS_PIPE_MAX_OUTPUT_STAGE];
	struct ia_css_frame *bin_out_frame[IA_CSS_BINARY_MAX_OUTPUT_PORTS];
	struct ia_css_frame *vf_frame[IA_CSS_PIPE_MAX_OUTPUT_STAGE];
	struct ia_css_pipeline_stage_desc stage_desc;
	bool need_in_frameinfo_memory = false;
#ifdef USE_INPUT_SYSTEM_VERSION_2401
	bool sensor = false;
	bool buffered_sensor = false;
	bool online = false;
	bool continuous = false;
#endif

	assert(pipe != NULL);
	assert(pipe->stream != NULL);

	me = &pipe->pipeline;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		 "create_host_yuvpp_pipeline() enter:\n");
	ia_css_pipeline_clean(me);
	for (i = 0; i < IA_CSS_PIPE_MAX_OUTPUT_STAGE; i++) {
		out_frame[i] = NULL;
		vf_frame[i] = NULL;
	}
	ia_css_pipe_util_create_output_frames(bin_out_frame);
	num_stage  = pipe->pipe_settings.yuvpp.num_yuv_scaler;
	num_vf_pp_stage   = pipe->pipe_settings.yuvpp.num_vf_pp;
	num_output_stage   = pipe->pipe_settings.yuvpp.num_output;

#ifdef USE_INPUT_SYSTEM_VERSION_2401
	/* When the input system is 2401, always enable 'in_frameinfo_memory'
	 * except for the following:
	 * - Direct Sensor Mode Online Capture
	 * - Direct Sensor Mode Continuous Capture
	 * - Buffered Sensor Mode Continous Capture
	 */
	sensor = pipe->stream->config.mode == IA_CSS_INPUT_MODE_SENSOR;
	buffered_sensor = pipe->stream->config.mode == IA_CSS_INPUT_MODE_BUFFERED_SENSOR;
	online = pipe->stream->config.online;
	continuous = pipe->stream->config.continuous;
	need_in_frameinfo_memory =
		!((sensor && (online || continuous)) || (buffered_sensor && continuous));
#else
	/* Construct in_frame info (only in case we have dynamic input */
	need_in_frameinfo_memory = pipe->stream->config.mode == IA_CSS_INPUT_MODE_MEMORY;
#endif
	if (need_in_frameinfo_memory) {
		err = init_in_frameinfo_memory_defaults(pipe, &me->in_frame);
		if (err != IA_CSS_SUCCESS)
			return err;

		in_frame = &me->in_frame;
	} else {
		in_frame = NULL;
	}

	for (i = 0; i < num_output_stage; i++) {
		assert(i < IA_CSS_PIPE_MAX_OUTPUT_STAGE);
		if (pipe->output_info[i].res.width != 0) {
			err = init_out_frameinfo_defaults(pipe, &me->out_frame[i], i);
			if (err != IA_CSS_SUCCESS)
				return err;
			out_frame[i] = &me->out_frame[i];
		}

		/* Construct vf_frame info (only in case we have VF) */
		if (pipe->vf_output_info[i].res.width != 0) {
			err = init_vf_frameinfo_defaults(pipe, &me->vf_frame[i], i);
			if (err != IA_CSS_SUCCESS)
				return err;
			vf_frame[i] = &me->vf_frame[i];
		}
	}

	copy_binary       = &pipe->pipe_settings.yuvpp.copy_binary;
	vf_pp_binary      = pipe->pipe_settings.yuvpp.vf_pp_binary;
	yuv_scaler_binary = pipe->pipe_settings.yuvpp.yuv_scaler_binary;
	need_scaler = need_yuv_scaler(pipe);

	if (pipe->pipe_settings.yuvpp.copy_binary.info) {
		if(need_scaler) {
			ia_css_pipe_util_set_output_frames(bin_out_frame, 0, NULL);
			ia_css_pipe_get_generic_stage_desc(&stage_desc, copy_binary,
				bin_out_frame, NULL, NULL, NULL);
		} else {
			ia_css_pipe_util_set_output_frames(bin_out_frame, 0, out_frame[0]);
			ia_css_pipe_get_generic_stage_desc(&stage_desc, copy_binary,
				bin_out_frame, NULL, NULL, NULL);
		}

		err = ia_css_pipeline_create_and_add_stage(me,
			&stage_desc,
			&post_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
		if (post_stage) {
			/* if we use yuv scaler binary, vf output should be from there */
			post_stage->args.copy_vf = !need_scaler;
			/* for yuvpp pipe, it should always be enabled */
			post_stage->args.copy_output = true;
		}
	}

	if (need_scaler && post_stage) {
		struct ia_css_frame *tmp_out_frame = NULL;
		struct ia_css_frame *tmp_vf_frame = NULL;
		struct ia_css_frame *tmp_in_frame = post_stage->args.out_frame[0];

		for (i = 0, j = 0; i < num_stage; i++) {
			assert(j < num_output_stage);
			if (pipe->pipe_settings.yuvpp.is_output_stage[i] == true) {
				tmp_out_frame = out_frame[j];
				tmp_vf_frame = vf_frame[j];
			} else {
				tmp_out_frame = NULL;
				tmp_vf_frame = NULL;
			}

			err = add_yuv_scaler_stage(pipe, me, tmp_in_frame, tmp_out_frame,
						   NULL,
						   &yuv_scaler_binary[i],
						   post_stage, &post_stage,
						   pipe->pipe_settings.yuvpp.is_output_stage[i]);
			if (err != IA_CSS_SUCCESS)
				return err;
			/* we use output port 1 as internal output port */
			tmp_in_frame = post_stage->args.out_frame[1];
			if (pipe->pipe_settings.yuvpp.is_output_stage[i] == true) {
				err = add_vf_pp_stage(pipe, tmp_vf_frame, &vf_pp_binary[j],
					      post_stage, &vf_pp_stage);
				if (err != IA_CSS_SUCCESS)
					return err;
				j++;
			}
		}
	} else if (post_stage != NULL) {
		err = add_vf_pp_stage(pipe, vf_frame[0], &vf_pp_binary[0],
				      post_stage, &vf_pp_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
	}

	ia_css_pipeline_finalize_stages(&pipe->pipeline);
#if 0
	if (!need_scaler) {
		err = ia_css_pipeline_get_stage(me, copy_binary->info->sp.mode,
						&out_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
		copy_stage = out_stage;
	} else {
		if (copy_binary->info) {
			err = ia_css_pipeline_get_stage(me,
							copy_binary->info->sp.mode,
							&copy_stage);
			if (err != IA_CSS_SUCCESS)
				return err;
		}
		if (yuv_scaler_binary->info) {
			err = ia_css_pipeline_get_stage(me,
					yuv_scaler_binary->info->sp.mode,
					&out_stage);
			if (err != IA_CSS_SUCCESS)
				return err;
		}
	}
	if (vf_pp_stage)
		vf_pp_stage->args.out_frame = vf_frame;
#if 0
	/* rvanimme: why is this? */
	/* TODO: investigate if this can be removed */
	if (!pipe->output_stage)
		out_stage->args.out_frame = out_frame;
#endif
	if (copy_stage && in_frame)
		copy_stage->args.out_frame = in_frame;
#endif
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"create_host_regular_capture_pipeline() leave:\n");

	return IA_CSS_SUCCESS;
}

static enum ia_css_err
create_host_copy_pipeline(struct ia_css_pipe *pipe,
    unsigned max_input_width,
    struct ia_css_frame *out_frame)
{
	struct ia_css_pipeline *me;
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct ia_css_pipeline_stage_desc stage_desc;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"create_host_copy_pipeline() enter:\n");

	/* pipeline already created as part of create_host_pipeline_structure */
	me = &pipe->pipeline;
	ia_css_pipeline_clean(me);

	/* Construct out_frame info */
	out_frame->contiguous = false;
	out_frame->flash_state = IA_CSS_FRAME_FLASH_STATE_NONE;

	if (copy_on_sp(pipe) &&
	    pipe->stream->config.format == IA_CSS_STREAM_FORMAT_BINARY_8) {
		ia_css_frame_info_init(
			&out_frame->info,
			JPEG_BYTES,
			1,
			IA_CSS_FRAME_FORMAT_BINARY_8,
			0);
	} else if (out_frame->info.format == IA_CSS_FRAME_FORMAT_RAW) {
		out_frame->info.raw_bit_depth =
			ia_css_pipe_util_pipe_input_format_bpp(pipe);
	}

	me->num_stages = 1;
	me->pipe_id = IA_CSS_PIPE_ID_COPY;
	pipe->mode  = IA_CSS_PIPE_ID_COPY;

	ia_css_pipe_get_sp_func_stage_desc(&stage_desc, out_frame,
		IA_CSS_PIPELINE_RAW_COPY, max_input_width);
	err = ia_css_pipeline_create_and_add_stage(me,
		&stage_desc,
		NULL);

	ia_css_pipeline_finalize_stages(&pipe->pipeline);

	configure_pipe_inout_port(&pipe->pipeline, true);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"create_host_copy_pipeline() leave:\n");

	return err;
}

static enum ia_css_err
create_host_isyscopy_capture_pipeline(struct ia_css_pipe *pipe)
{
	struct ia_css_pipeline *me = &pipe->pipeline;
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct ia_css_pipeline_stage_desc stage_desc;
	struct ia_css_frame *out_frame = &me->out_frame[0];
	struct ia_css_pipeline_stage *out_stage = NULL;
	unsigned int thread_id;
	enum sh_css_queue_id queue_id;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"create_host_isyscopy_capture_pipeline() enter:\n");
	ia_css_pipeline_clean(me);

	/* Construct out_frame info */
	err = sh_css_pipe_get_output_frame_info(pipe, &out_frame->info, 0);
	if (err != IA_CSS_SUCCESS)
		return err;
	out_frame->contiguous = false;
	out_frame->flash_state = IA_CSS_FRAME_FLASH_STATE_NONE;
	ia_css_pipeline_get_sp_thread_id(ia_css_pipe_get_pipe_num(pipe), &thread_id);
	ia_css_query_internal_queue_id(IA_CSS_BUFFER_TYPE_OUTPUT_FRAME, thread_id, &queue_id);
	out_frame->dynamic_data_index = queue_id;
	out_frame->buf_type = IA_CSS_BUFFER_TYPE_OUTPUT_FRAME;

	me->num_stages = 1;
	me->pipe_id = IA_CSS_PIPE_ID_CAPTURE;
	pipe->mode  = IA_CSS_PIPE_ID_CAPTURE;
	ia_css_pipe_get_sp_func_stage_desc(&stage_desc, out_frame,
		IA_CSS_PIPELINE_ISYS_COPY, 0);
	err = ia_css_pipeline_create_and_add_stage(me,
		&stage_desc, &out_stage);
	if(err != IA_CSS_SUCCESS)
		return err;

	ia_css_pipeline_finalize_stages(me);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"create_host_isyscopy_capture_pipeline() leave:\n");

	return err;
}

static enum ia_css_err
create_host_regular_capture_pipeline(struct ia_css_pipe *pipe)
{
	struct ia_css_pipeline *me;
	enum ia_css_err err = IA_CSS_SUCCESS;
	enum ia_css_capture_mode mode;
	struct ia_css_pipeline_stage *out_stage = NULL,
				     *vf_pp_stage = NULL,
				     *copy_stage = NULL,
				     *in_stage = NULL,
				     *post_stage = NULL;
	struct ia_css_frame *cc_frame = NULL;
	struct ia_css_binary *copy_binary,
			     *primary_binary,
			     *vf_pp_binary,
			     *pre_isp_binary,
			     *anr_gdc_binary,
			     *post_isp_binary,
			     *capture_pp_binary;
	bool need_pp = false;
	bool raw;

	struct ia_css_frame *in_frame;
	struct ia_css_frame *out_frame;
	struct ia_css_frame *out_frames[IA_CSS_BINARY_MAX_OUTPUT_PORTS];
	struct ia_css_frame *vf_frame;
	struct ia_css_pipeline_stage_desc stage_desc;
	bool need_in_frameinfo_memory = false;
#ifdef USE_INPUT_SYSTEM_VERSION_2401
	bool sensor = false;
	bool buffered_sensor = false;
	bool online = false;
	bool continuous = false;
#endif

	assert(pipe != NULL);
	assert(pipe->stream != NULL);

	me = &pipe->pipeline;
	mode = pipe->config.default_capture_config.mode;
	raw = (mode == IA_CSS_CAPTURE_MODE_RAW);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		 "create_host_regular_capture_pipeline() enter:\n");
	ia_css_pipeline_clean(me);
	ia_css_pipe_util_create_output_frames(out_frames);

#ifdef USE_INPUT_SYSTEM_VERSION_2401
	/* When the input system is 2401, always enable 'in_frameinfo_memory'
	 * except for the following:
	 * - Direct Sensor Mode Online Capture
	 * - Direct Sensor Mode Online Capture
	 * - Direct Sensor Mode Continuous Capture
	 * - Buffered Sensor Mode Continous Capture
	 */
	sensor = (pipe->stream->config.mode == IA_CSS_INPUT_MODE_SENSOR);
	buffered_sensor = (pipe->stream->config.mode == IA_CSS_INPUT_MODE_BUFFERED_SENSOR);
	online = pipe->stream->config.online;
	continuous = pipe->stream->config.continuous;
	need_in_frameinfo_memory =
		!((sensor && (online || continuous)) || (buffered_sensor && (online || continuous)));
#else
	/* Construct in_frame info (only in case we have dynamic input */
	need_in_frameinfo_memory = pipe->stream->config.mode == IA_CSS_INPUT_MODE_MEMORY;
#endif
	if (need_in_frameinfo_memory) {
		err = init_in_frameinfo_memory_defaults(pipe, &me->in_frame);
		if (err != IA_CSS_SUCCESS)
			return err;

		in_frame = &me->in_frame;
	} else {
		in_frame = NULL;
	}

	err = init_out_frameinfo_defaults(pipe, &me->out_frame[0], 0);
	if (err != IA_CSS_SUCCESS)
		return err;
	out_frame = &me->out_frame[0];

	/* Construct vf_frame info (only in case we have VF) */
	if (mode == IA_CSS_CAPTURE_MODE_RAW ||
			mode == IA_CSS_CAPTURE_MODE_BAYER) {
		/* These modes don't support viewfinder output */
		vf_frame = NULL;
	} else {
		init_vf_frameinfo_defaults(pipe, &me->vf_frame[0], 0);
		vf_frame = &me->vf_frame[0];
	}

	copy_stage = NULL;
	in_stage = NULL;

	copy_binary       = &pipe->pipe_settings.capture.copy_binary;
	primary_binary    = &pipe->pipe_settings.capture.primary_binary;
	vf_pp_binary      = &pipe->pipe_settings.capture.vf_pp_binary;
	pre_isp_binary    = &pipe->pipe_settings.capture.pre_isp_binary;
	anr_gdc_binary    = &pipe->pipe_settings.capture.anr_gdc_binary;
	post_isp_binary   = &pipe->pipe_settings.capture.post_isp_binary;
	capture_pp_binary = &pipe->pipe_settings.capture.capture_pp_binary;
	need_pp = (need_capture_pp(pipe) || pipe->output_stage) &&
		  mode != IA_CSS_CAPTURE_MODE_RAW &&
		  mode != IA_CSS_CAPTURE_MODE_BAYER;

	if (pipe->pipe_settings.capture.copy_binary.info) {
		if(raw) {
			ia_css_pipe_util_set_output_frames(out_frames, 0, out_frame);
			ia_css_pipe_get_generic_stage_desc(&stage_desc, copy_binary,
				out_frames, NULL, NULL, NULL);
		} else {
			ia_css_pipe_util_set_output_frames(out_frames, 0, in_frame);
			ia_css_pipe_get_generic_stage_desc(&stage_desc, copy_binary,
				out_frames, NULL, NULL, NULL);
		}

		err = ia_css_pipeline_create_and_add_stage(me,
			&stage_desc,
			&post_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
		in_stage = post_stage;
	} else if (pipe->stream->config.continuous) {
		in_frame = pipe->stream->last_pipe->continuous_frames[0];
	}

	if (mode == IA_CSS_CAPTURE_MODE_PRIMARY) {

		if(need_pp) {
			ia_css_pipe_util_set_output_frames(out_frames, 0, NULL);
			ia_css_pipe_get_generic_stage_desc(&stage_desc, primary_binary,
				out_frames, in_frame, cc_frame, NULL);
		} else {
			ia_css_pipe_util_set_output_frames(out_frames, 0, out_frame);
			ia_css_pipe_get_generic_stage_desc(&stage_desc, primary_binary,
				out_frames, in_frame, cc_frame, NULL);
		}

		err = ia_css_pipeline_create_and_add_stage(me,
			&stage_desc,
			&post_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
		/* If we use copy iso primary,
		   the input must be yuv iso raw */
		post_stage->args.copy_vf =
			primary_binary->info->sp.mode ==
			IA_CSS_BINARY_MODE_COPY;
		post_stage->args.copy_output = post_stage->args.copy_vf;
	} else if (mode == IA_CSS_CAPTURE_MODE_ADVANCED ||
	           mode == IA_CSS_CAPTURE_MODE_LOW_LIGHT) {
		ia_css_pipe_util_set_output_frames(out_frames, 0, NULL);
		ia_css_pipe_get_generic_stage_desc(&stage_desc, pre_isp_binary,
			out_frames, in_frame, cc_frame, NULL);
		err = ia_css_pipeline_create_and_add_stage(me,
				&stage_desc, NULL);
		if (err != IA_CSS_SUCCESS)
			return err;
		ia_css_pipe_util_set_output_frames(out_frames, 0, NULL);
		ia_css_pipe_get_generic_stage_desc(&stage_desc, anr_gdc_binary,
			out_frames, NULL, NULL, NULL);
		err = ia_css_pipeline_create_and_add_stage(me,
				&stage_desc, NULL);
		if (err != IA_CSS_SUCCESS)
			return err;

		if(need_pp) {
			ia_css_pipe_util_set_output_frames(out_frames, 0, NULL);
			ia_css_pipe_get_generic_stage_desc(&stage_desc, post_isp_binary,
				out_frames, NULL, NULL, NULL);
		} else {
			ia_css_pipe_util_set_output_frames(out_frames, 0, out_frame);
			ia_css_pipe_get_generic_stage_desc(&stage_desc, post_isp_binary,
				out_frames, NULL, NULL, NULL);
		}

		err = ia_css_pipeline_create_and_add_stage(me,
				&stage_desc, &post_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
	} else if (mode == IA_CSS_CAPTURE_MODE_BAYER) {
		ia_css_pipe_util_set_output_frames(out_frames, 0, out_frame);
		ia_css_pipe_get_generic_stage_desc(&stage_desc, pre_isp_binary,
			out_frames, in_frame, cc_frame, NULL);
		err = ia_css_pipeline_create_and_add_stage(me,
			&stage_desc,
			NULL);
		if (err != IA_CSS_SUCCESS)
			return err;
	}
	if (!in_stage)
		in_stage = post_stage;

	if (need_pp) {
		err = add_capture_pp_stage(pipe, me, out_frame,
					   capture_pp_binary,
					   post_stage, &post_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
	}
	if (mode != IA_CSS_CAPTURE_MODE_RAW &&
		mode != IA_CSS_CAPTURE_MODE_BAYER) {
		err = add_vf_pp_stage(pipe, vf_frame, vf_pp_binary,
				      post_stage, &vf_pp_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
	}

	ia_css_pipeline_finalize_stages(&pipe->pipeline);

	/**
	 * Maybe we can return earlier but this was the original position
	 * in the original version of capture_start()
	 */
	if (pipe->config.default_capture_config.mode == IA_CSS_CAPTURE_MODE_RAW ||
	    pipe->config.default_capture_config.mode == IA_CSS_CAPTURE_MODE_BAYER) {
		if (copy_on_sp(pipe))
			return IA_CSS_SUCCESS;
	}

	if (mode == IA_CSS_CAPTURE_MODE_RAW) {
		err = ia_css_pipeline_get_stage(me, copy_binary->info->sp.mode,
						&out_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
		copy_stage = out_stage;
	} else if (mode == IA_CSS_CAPTURE_MODE_BAYER) {
		err = ia_css_pipeline_get_stage(me,
				pre_isp_binary->info->sp.mode,
				&out_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
	} else {
		if (copy_binary->info) {
			err = ia_css_pipeline_get_stage(me,
							copy_binary->info->sp.mode,
							&copy_stage);
			if (err != IA_CSS_SUCCESS)
				return err;
		}
		if (capture_pp_binary->info) {
			err = ia_css_pipeline_get_stage(me,
					capture_pp_binary->info->sp.mode,
					&out_stage);
			if (err != IA_CSS_SUCCESS)
				return err;
		} else if (mode ==
			   IA_CSS_CAPTURE_MODE_PRIMARY) {
			err = ia_css_pipeline_get_stage(me,
					primary_binary->info->sp.mode, &out_stage);
			if (err != IA_CSS_SUCCESS)
				return err;
		} else if (mode ==
			   IA_CSS_CAPTURE_MODE_LOW_LIGHT) {
			err = ia_css_pipeline_get_stage(me,
					post_isp_binary->info->sp.mode,
					&out_stage);
			if (err != IA_CSS_SUCCESS)
				return err;
		} else {
			err = ia_css_pipeline_get_stage(me,
					post_isp_binary->info->sp.mode,
					&out_stage);
			if (err != IA_CSS_SUCCESS)
				return err;
		}
	}

	/* rvanimme: why is this? */
	/* TODO: investigate if this can be removed */
	if (!pipe->output_stage)
		out_stage->args.out_frame[0] = out_frame;

	if (copy_stage && in_frame)
		copy_stage->args.out_frame[0] = in_frame;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"create_host_regular_capture_pipeline() leave:\n");

	return IA_CSS_SUCCESS;
}

static enum ia_css_err
create_host_capture_pipeline(struct ia_css_pipe *pipe)
{
	enum ia_css_err err = IA_CSS_SUCCESS;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"create_host_capture_pipeline() enter:\n");

	if (pipe->config.mode == IA_CSS_PIPE_MODE_COPY)
		err = create_host_isyscopy_capture_pipeline(pipe);
	else
		err = create_host_regular_capture_pipeline(pipe);
	if (err != IA_CSS_SUCCESS)
		return err;

	/* pipeline already created as part of create_host_pipeline_structure */
	configure_pipe_inout_port(&pipe->pipeline,
		pipe->stream->config.continuous);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"create_host_capture_pipeline() leave:\n");

	return err;
}

static enum ia_css_err capture_start(
	struct ia_css_pipe *pipe)
{
	struct ia_css_pipeline *me;

	enum ia_css_err err = IA_CSS_SUCCESS;
	enum sh_css_pipe_config_override copy_ovrd;

	assert(pipe != NULL);

	me = &pipe->pipeline;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "capture_start() enter:\n");

#if !defined(HAS_NO_INPUT_SYSTEM)
	if (pipe->stream->config.mode != IA_CSS_INPUT_MODE_MEMORY) {
		err = sh_css_config_input_network(pipe,
			&pipe->pipe_settings.capture.copy_binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	}
#endif

	if ((pipe->config.default_capture_config.mode == IA_CSS_CAPTURE_MODE_RAW   ||
	     pipe->config.default_capture_config.mode == IA_CSS_CAPTURE_MODE_BAYER   ) &&
		(pipe->config.mode != IA_CSS_PIPE_MODE_COPY)) {
		if (copy_on_sp(pipe)) {
			return start_copy_on_sp(pipe, &me->out_frame[0]);
		}
	}

#if defined(USE_INPUT_SYSTEM_VERSION_2) || defined(USE_INPUT_SYSTEM_VERSION_2401)
	/* multi stream video needs mipi buffers */
	if (pipe->config.mode != IA_CSS_PIPE_MODE_COPY) {
		send_mipi_frames(pipe);
	}
#endif

	{
		unsigned int thread_id;

		ia_css_pipeline_get_sp_thread_id(ia_css_pipe_get_pipe_num(pipe), &thread_id);
		copy_ovrd = 1 << thread_id;

	}
	err = start_pipe(pipe, copy_ovrd, pipe->stream->config.mode);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"capture_start() leave: return (%d)\n", err);
	return err;

}

static enum ia_css_err
sh_css_pipe_get_output_frame_info(struct ia_css_pipe *pipe,
				  struct ia_css_frame_info *info,
				  unsigned int idx)
{
	enum ia_css_err err = IA_CSS_SUCCESS;

	assert(pipe != NULL);
	assert(info != NULL);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
						"sh_css_pipe_get_output_frame_info() enter:\n");

	*info = pipe->output_info[idx];
	if (copy_on_sp(pipe) &&
	    pipe->stream->config.format == IA_CSS_STREAM_FORMAT_BINARY_8) {
		ia_css_frame_info_init(
			info,
			JPEG_BYTES,
			1,
			IA_CSS_FRAME_FORMAT_BINARY_8,
			0);
	} else if (info->format == IA_CSS_FRAME_FORMAT_RAW ||
		   info->format == IA_CSS_FRAME_FORMAT_RAW_PACKED) {
		info->raw_bit_depth =
			ia_css_pipe_util_pipe_input_format_bpp(pipe);
	}

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
						"sh_css_pipe_get_output_frame_info() leave:\n");
	return err;
}

#if !defined(HAS_NO_INPUT_SYSTEM)
void
ia_css_stream_send_input_frame(const struct ia_css_stream *stream,
			       const unsigned short *data,
			       unsigned int width,
			       unsigned int height)
{
	assert(stream != NULL);

	ia_css_inputfifo_send_input_frame(
			data, width, height,
			stream->config.channel_id,
			stream->config.format,
			stream->config.pixels_per_clock == 2);
}

void
ia_css_stream_start_input_frame(const struct ia_css_stream *stream)
{
	assert(stream != NULL);

	ia_css_inputfifo_start_frame(
			stream->config.channel_id,
			stream->config.format,
			stream->config.pixels_per_clock == 2);
}

void
ia_css_stream_send_input_line(const struct ia_css_stream *stream,
			      const unsigned short *data,
			      unsigned int width,
			      const unsigned short *data2,
			      unsigned int width2)
{
	assert(stream != NULL);

	ia_css_inputfifo_send_line(stream->config.channel_id,
					       data, width, data2, width2);
}

void
ia_css_stream_send_input_embedded_line(const struct ia_css_stream *stream,
		enum ia_css_stream_format format,
		const unsigned short *data,
		unsigned int width)
{
	assert(stream != NULL);
	if (data == NULL || width == 0)
		return;
	ia_css_inputfifo_send_embedded_line(stream->config.channel_id,
			format, data, width);
}

void
ia_css_stream_end_input_frame(const struct ia_css_stream *stream)
{
	assert(stream != NULL);

	ia_css_inputfifo_end_frame(stream->config.channel_id);
}
#endif

static void
append_firmware(struct ia_css_fw_info **l, struct ia_css_fw_info *firmware)
{
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "append_firmware() enter:\n");
	while (*l)
		l = &(*l)->next;
	*l = firmware;
	firmware->next = NULL;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "append_firmware() leave:\n");
}

static void
remove_firmware(struct ia_css_fw_info **l, struct ia_css_fw_info *firmware)
{
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "remove_firmware() enter:\n");
	while (*l && *l != firmware)
		l = &(*l)->next;
	if (!*l)
		return;
	*l = firmware->next;
	firmware->next = NULL;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "remove_firmware() leave:\n");
}

#if !defined(C_RUN) && !defined(HRT_UNSCHED)
static enum ia_css_err
upload_isp_code(struct ia_css_fw_info *firmware)
{
	hrt_vaddress binary = firmware->info.isp.xmem_addr;
	if (!binary) {
		unsigned size = firmware->blob.size;
		const unsigned char *blob;
		const unsigned char *binary_name;
		binary_name =
			(const unsigned char *)(IA_CSS_EXT_ISP_PROG_NAME(
						firmware));
		blob = binary_name +
			strlen((const char *)binary_name) +
			1;
		binary = sh_css_load_blob(blob, size);
		firmware->info.isp.xmem_addr = binary;
	}

	if (!binary)
		return IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
	return IA_CSS_SUCCESS;
}
#endif

static enum ia_css_err
acc_load_extension(struct ia_css_fw_info *firmware)
{
#if !defined(C_RUN) && !defined(HRT_UNSCHED)
	enum ia_css_err err = upload_isp_code(firmware);
	if (err != IA_CSS_SUCCESS)
		return err;
#endif

	firmware->loaded = true;
	return IA_CSS_SUCCESS;
}

static void
acc_unload_extension(struct ia_css_fw_info *firmware)
{
	if (firmware->info.isp.xmem_addr) {
		mmgr_free(firmware->info.isp.xmem_addr);
		firmware->info.isp.xmem_addr = mmgr_NULL;
	}
	firmware->isp_code = NULL;
	firmware->loaded = false;
}
/* Load firmware for extension */
static enum ia_css_err
ia_css_pipe_load_extension(struct ia_css_pipe *pipe,
			   struct ia_css_fw_info *firmware)
{
	enum ia_css_err err = IA_CSS_SUCCESS;

	assert(firmware != NULL);
	assert(pipe != NULL);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "sh_css_pipe_load_extension() enter:\n");
	if (firmware->info.isp.type == IA_CSS_ACC_OUTPUT)
		append_firmware(&pipe->output_stage, firmware);
	else if (firmware->info.isp.type == IA_CSS_ACC_VIEWFINDER) {
		append_firmware(&pipe->vf_stage, firmware);
	}
	err = acc_load_extension(firmware);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "sh_css_pipe_load_extension() leave:\n");
	return err;
}

/* Unload firmware for extension */
static void
ia_css_pipe_unload_extension(struct ia_css_pipe *pipe,
			     struct ia_css_fw_info *firmware)
{
	assert(firmware != NULL);
	assert(pipe != NULL);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "sh_css_pipe_unload_extension() enter:\n");
	if (firmware->info.isp.type == IA_CSS_ACC_OUTPUT)
		remove_firmware(&pipe->output_stage, firmware);
	else if (firmware->info.isp.type == IA_CSS_ACC_VIEWFINDER)
		remove_firmware(&pipe->vf_stage, firmware);
	acc_unload_extension(firmware);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "sh_css_pipe_unload_extension() leave:\n");
}

bool
ia_css_pipeline_uses_params(struct ia_css_pipeline *me)
{
	struct ia_css_pipeline_stage *stage;

	assert(me != NULL);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_pipeline_uses_params() enter: me=%p\n", me);

	for (stage = me->stages; stage; stage = stage->next)
		if (stage->binary_info && stage->binary_info->enable.params) {
			ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
				"ia_css_pipeline_uses_params() leave: "
				"return_bool=true\n");
			return true;
		}
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_pipeline_uses_params() leave: return_bool=false\n");
	return false;
}

static enum ia_css_err
sh_css_pipeline_add_acc_stage(struct ia_css_pipeline *pipeline,
			      const void *acc_fw)
{
	struct ia_css_fw_info *fw = (struct ia_css_fw_info *)acc_fw;
	enum ia_css_err	err = acc_load_extension(fw);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"sh_css_pipeline_add_acc_stage() enter: pipeline=%p,"
		" acc_fw=%p\n", pipeline, acc_fw);

	if (err == IA_CSS_SUCCESS) {
		struct ia_css_pipeline_stage_desc stage_desc;
		ia_css_pipe_get_acc_stage_desc(&stage_desc, NULL, fw);
		err = ia_css_pipeline_create_and_add_stage(pipeline,
			&stage_desc,
			NULL);
	}

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"sh_css_pipeline_add_acc_stage() leave: return_err=%d\n",err);
	return err;
}

/**
 * @brief Tag a specific frame in continuous capture.
 * Refer to "sh_css_internal.h" for details.
 */
enum ia_css_err ia_css_stream_capture_frame(struct ia_css_stream *stream,
				unsigned int exp_id)
{
	struct sh_css_tag_descr tag_descr;
	unsigned int encoded_tag_descr;
	enum ia_css_err return_err;

	(void)stream;
	assert(stream != NULL);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_stream_capture_frame() enter: exp_id=%d\n",
		exp_id);

	if (exp_id == 0) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
			"ia_css_stream_capture_frame() "
			"leave: return_err=%d\n",
			IA_CSS_ERR_INVALID_ARGUMENTS);
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	}

	/* Create the tag descriptor from the parameters */
	sh_css_create_tag_descr(0, 0, 0, exp_id, &tag_descr);


	/* Encode the tag descriptor into a 32-bit value */
	encoded_tag_descr = sh_css_encode_tag_descr(&tag_descr);

	if (!sh_css_sp_is_running()) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
			"ia_css_stream_capture_frame() leaving:"
			"queue unavailable\n");
		/* SP is not running. The queues are not valid */
		return IA_CSS_ERR_RESOURCE_NOT_AVAILABLE;
	}


	/* Enqueue the encoded tag to the host2sp queue.
	 * Note: The pipe and stage IDs for tag_cmd queue are hard-coded to 0
	 * on both host and the SP side.
	 * It is mainly because it is enough to have only one tag_cmd queue */
	return_err= ia_css_bufq_enqueue_tag_cmd((uint32_t)encoded_tag_descr);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_stream_capture_frame() leave: return_err=%d\n",
		return_err);

	return return_err;
}

/**
 * @brief Configure the continuous capture.
 * Refer to "sh_css_internal.h" for details.
 */
enum ia_css_err ia_css_stream_capture(
	struct ia_css_stream *stream,
	int num_captures,
	unsigned int skip,
	int offset)
{
	struct sh_css_tag_descr tag_descr;
	unsigned int encoded_tag_descr;
	enum ia_css_err return_err;

	if (stream == NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_stream_capture() enter: num_captures=%d,"
		" skip=%d, offset=%d\n", num_captures, skip,offset);

	/* Check if the tag descriptor is valid */
	if (num_captures < SH_CSS_MINIMUM_TAG_ID) {
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_stream_capture() leave: return_err=%d\n",
		IA_CSS_ERR_INVALID_ARGUMENTS);
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	}

	/* Create the tag descriptor from the parameters */
	sh_css_create_tag_descr(num_captures, skip, offset, 0, &tag_descr);


	/* Encode the tag descriptor into a 32-bit value */
	encoded_tag_descr = sh_css_encode_tag_descr(&tag_descr);

	if (!sh_css_sp_is_running()) {
		/* SP is not running. The queues are not valid */
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
			"ia_css_stream_capture() leaving:"
			"queues unavailable\n");
		return IA_CSS_ERR_RESOURCE_NOT_AVAILABLE;
	}

	/* Enqueue the encoded tag to the host2sp queue.
	 * Note: The pipe and stage IDs for tag_cmd queue are hard-coded to 0
	 * on both host and the SP side.
	 * It is mainly because it is enough to have only one tag_cmd queue */
	return_err = ia_css_bufq_enqueue_tag_cmd((uint32_t)encoded_tag_descr);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_stream_capture() leave: return_err=%d\n",
		return_err);

	return return_err;
}

void ia_css_stream_request_flash(struct ia_css_stream *stream)
{
	(void)stream;

	assert(stream != NULL);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_stream_request_flash() enter: void\n");

	sh_css_write_host2sp_command(host2sp_cmd_start_flash);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_stream_request_flash() leave: return_void\n");
}

static void
sh_css_init_host_sp_control_vars(void)
{
	const struct ia_css_fw_info *fw;
	unsigned int HIVE_ADDR_ia_css_ispctrl_sp_isp_started;

	unsigned int HIVE_ADDR_host_sp_queues_initialized;
	unsigned int HIVE_ADDR_sp_sleep_mode;
	unsigned int HIVE_ADDR_ia_css_dmaproxy_sp_invalidate_tlb;
	unsigned int HIVE_ADDR_sp_stop_copy_preview;
	unsigned int HIVE_ADDR_host_sp_com;
	unsigned int o = offsetof(struct host_sp_communication, host2sp_command)
				/ sizeof(int);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"sh_css_init_host_sp_control_vars() enter: void\n");

	fw = &sh_css_sp_fw;
	HIVE_ADDR_ia_css_ispctrl_sp_isp_started = fw->info.sp.isp_started;

	HIVE_ADDR_host_sp_queues_initialized =
		fw->info.sp.host_sp_queues_initialized;
	HIVE_ADDR_sp_sleep_mode = fw->info.sp.sleep_mode;
	HIVE_ADDR_ia_css_dmaproxy_sp_invalidate_tlb = fw->info.sp.invalidate_tlb;
	HIVE_ADDR_sp_stop_copy_preview = fw->info.sp.stop_copy_preview;
	HIVE_ADDR_host_sp_com = fw->info.sp.host_sp_com;

	(void)HIVE_ADDR_ia_css_ispctrl_sp_isp_started; /* Suppres warnings in CRUN */

	(void)HIVE_ADDR_sp_sleep_mode;
	(void)HIVE_ADDR_ia_css_dmaproxy_sp_invalidate_tlb;
	(void)HIVE_ADDR_sp_stop_copy_preview;
	(void)HIVE_ADDR_host_sp_com;

	sp_dmem_store_uint32(SP0_ID,
		(unsigned int)sp_address_of(ia_css_ispctrl_sp_isp_started),
		(uint32_t)(0));

	sp_dmem_store_uint32(SP0_ID,
		(unsigned int)sp_address_of(host_sp_queues_initialized),
		(uint32_t)(0));
	sp_dmem_store_uint32(SP0_ID,
		(unsigned int)sp_address_of(sp_sleep_mode),
		(uint32_t)(0));
	sp_dmem_store_uint32(SP0_ID,
		(unsigned int)sp_address_of(ia_css_dmaproxy_sp_invalidate_tlb),
		(uint32_t)(false));
	sp_dmem_store_uint32(SP0_ID,
		(unsigned int)sp_address_of(sp_stop_copy_preview),
		my_css.stop_copy_preview?(uint32_t)(1):(uint32_t)(0));
	store_sp_array_uint(host_sp_com, o, host2sp_cmd_ready);

#if defined(USE_INPUT_SYSTEM_VERSION_2) || defined(USE_INPUT_SYSTEM_VERSION_2401)
	sh_css_update_host2sp_num_mipi_frames
			(my_css.num_mipi_frames);
#endif

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"sh_css_init_host_sp_control_vars() leave: return_void\n");
}

/**
 * create the internal structures and fill in the configuration data
 */
void ia_css_pipe_config_defaults(struct ia_css_pipe_config *pipe_config)
{
	struct ia_css_pipe_config def_config = {
		IA_CSS_PIPE_MODE_PREVIEW, /* mode */
		1,      /* isp_pipe_version */
		{0, 0}, /* bayer_ds_out_res */
		{0, 0}, /* capt_pp_in_res */
		{0, 0}, /* vf_pp_in_res */
		{0, 0}, /* dvs_crop_out_res */
		{{0, 0}, 0, 0, 0, 0}, /* output_info */
		{{0, 0}, 0, 0, 0, 0}, /* second_output_info */
		{{0, 0}, 0, 0, 0, 0}, /* vf_output_info */
		{{0, 0}, 0, 0, 0, 0}, /* second_vf_output_info */
		NULL,   /* acc_extension */
		NULL,   /* acc_stages */
		0,      /* num_acc_stages */
		{
			IA_CSS_CAPTURE_MODE_RAW, /* mode */
			false, /* enable_xnr */
			false  /* enable_raw_output */
		},      /* default_capture_config */
		{0, 0}, /* dvs_envelope */
		IA_CSS_FRAME_DELAY_1, /* dvs_frame_delay */
		-1,     /* acc_num_execs */
		true,   /* enable_dz */
	};
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_pipe_config_defaults()\n");
	*pipe_config = def_config;
}

void
ia_css_pipe_extra_config_defaults(struct ia_css_pipe_extra_config *extra_config)
{
	assert(extra_config != NULL);

	extra_config->enable_raw_binning = false;
	extra_config->enable_yuv_ds = false;
	extra_config->enable_high_speed = false;
	extra_config->enable_dvs_6axis = false;
	extra_config->enable_reduced_pipe = false;
	extra_config->disable_vf_pp = false;
	extra_config->enable_fractional_ds = false;
}

void ia_css_stream_config_defaults(struct ia_css_stream_config *stream_config)
{
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_stream_config_defaults()\n");
	assert(stream_config != NULL);
	memset(stream_config, 0, sizeof(*stream_config));
	stream_config->online = true;
	stream_config->left_padding = -1;
	/* temporary default value for backwards compatibility.
	 * This field used to be hardcoded within CSS but this has now
	 * been moved to the stream_config struct. */
	stream_config->source.port.rxcount = 0x04040404;
}

static enum ia_css_err
ia_css_acc_pipe_create(struct ia_css_pipe *pipe)
{
	enum ia_css_err err = IA_CSS_SUCCESS;

	assert(pipe != NULL);

	/* There is not meaning for num_execs = 0 semantically. Run atleast once. */
	if (pipe->config.acc_num_execs == 0)
		pipe->config.acc_num_execs = 1;

	return err;
}

enum ia_css_err
ia_css_pipe_create(const struct ia_css_pipe_config *config,
		   struct ia_css_pipe **pipe)
{
	if (config == NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	if (pipe == NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	return ia_css_pipe_create_extra(config, NULL, pipe);
}

enum ia_css_err
ia_css_pipe_create_extra(const struct ia_css_pipe_config *config,
			 const struct ia_css_pipe_extra_config *extra_config,
			 struct ia_css_pipe **pipe)
{
	enum ia_css_err err = IA_CSS_ERR_INTERNAL_ERROR;
	struct ia_css_pipe *internal_pipe = NULL;
	unsigned int i;
	struct ia_css_frame_info *tmp_output_info[IA_CSS_PIPE_MAX_OUTPUT_STAGE],
				 *tmp_vf_output_info[IA_CSS_PIPE_MAX_OUTPUT_STAGE];
	bool *tmp_enable_viewfinder[IA_CSS_PIPE_MAX_OUTPUT_STAGE];

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_pipe_create()\n");
	ia_css_debug_dump_pipe_config(config);
	ia_css_debug_dump_pipe_extra_config(extra_config);

	if (pipe == NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;

	err = create_pipe(config->mode, &internal_pipe, false);
	if (internal_pipe == NULL)
		return IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;

	/* now we have a pipe structure to fill */
	internal_pipe->config = *config;
	if (extra_config)
		internal_pipe->extra_config = *extra_config;
	else
		ia_css_pipe_extra_config_defaults(&internal_pipe->extra_config);

	if (config->mode == IA_CSS_PIPE_MODE_ACC) {
		/* Temporary hack to migrate acceleration to CSS 2.0.
		 * In the future the code for all pipe types should be
		 * unified. */
		*pipe = internal_pipe;
		return ia_css_acc_pipe_create(internal_pipe);
	}

	/*Use config value when dvs_frame_delay setting equal to 2, otherwise always 1 by default */
	internal_pipe->dvs_frame_delay =
		(internal_pipe->config.dvs_frame_delay == IA_CSS_FRAME_DELAY_2) ?
		IA_CSS_FRAME_DELAY_2 : IA_CSS_FRAME_DELAY_1;

	/* we still keep enable_raw_binning for backward compatibility, for any new
	   fractional bayer downscaling, we should use bayer_ds_out_res. if both are
	   specified, bayer_ds_out_res will take precedence.if none is specified, we
	   set bayer_ds_out_res equal to IF output resolution(IF may do cropping on
	   sensor output) or use default decimation factor 1. */
	if (internal_pipe->extra_config.enable_raw_binning &&
		 internal_pipe->config.bayer_ds_out_res.width) {
		/* fill some code here, if no code is needed, please remove it during integration */
	}

	/* YUV downscaling */
	if ((internal_pipe->config.vf_pp_in_res.width ||
		 internal_pipe->config.capt_pp_in_res.width) &&
	     internal_pipe->config.mode == IA_CSS_PIPE_MODE_CAPTURE) {
		enum ia_css_frame_format format;
		if (internal_pipe->config.vf_pp_in_res.width) {
			format = IA_CSS_FRAME_FORMAT_YUV_LINE;
			ia_css_frame_info_init(
				&internal_pipe->vf_yuv_ds_input_info,
				internal_pipe->config.vf_pp_in_res.width,
				internal_pipe->config.vf_pp_in_res.height,
				format, 0);
		}
		if (internal_pipe->config.capt_pp_in_res.width) {
			format = IA_CSS_FRAME_FORMAT_YUV420;
			ia_css_frame_info_init(
				&internal_pipe->out_yuv_ds_input_info,
				internal_pipe->config.capt_pp_in_res.width,
				internal_pipe->config.capt_pp_in_res.height,
				format, 0);
		}
	}
	if (internal_pipe->config.vf_pp_in_res.width &&
	    internal_pipe->config.mode == IA_CSS_PIPE_MODE_PREVIEW) {
		ia_css_frame_info_init(
				&internal_pipe->vf_yuv_ds_input_info,
				internal_pipe->config.vf_pp_in_res.width,
				internal_pipe->config.vf_pp_in_res.height,
				IA_CSS_FRAME_FORMAT_YUV_LINE, 0);
	}
	/* handle bayer downscaling output info */
	if (internal_pipe->config.bayer_ds_out_res.width) {
			ia_css_frame_info_init(
				&internal_pipe->bds_output_info,
				internal_pipe->config.bayer_ds_out_res.width,
				internal_pipe->config.bayer_ds_out_res.height,
				IA_CSS_FRAME_FORMAT_RAW, 0);
	}

	tmp_output_info[0]	= &internal_pipe->config.output_info;
	tmp_output_info[1] 	= &internal_pipe->config.second_output_info;
	tmp_vf_output_info[0]	= &internal_pipe->config.vf_output_info;
	tmp_vf_output_info[1] 	= &internal_pipe->config.second_vf_output_info;
	tmp_enable_viewfinder[0]= &internal_pipe->enable_viewfinder;
	tmp_enable_viewfinder[1]= &internal_pipe->enable_second_viewfinder;
	/* handle output info, asume always needed */
	for (i = 0; i < IA_CSS_PIPE_MAX_OUTPUT_STAGE; i++) {
		if (tmp_output_info[i]->res.width) {
#if defined(USE_INPUT_SYSTEM_VERSION_2401)
			if ( tmp_output_info[i]->format == IA_CSS_FRAME_FORMAT_BINARY_8) {
				tmp_output_info[i]->res.height =
					ceil_div(tmp_output_info[i]->res.width, HIVE_ISP_DDR_WORD_BYTES);
				tmp_output_info[i]->res.width = HIVE_ISP_DDR_WORD_BYTES;
			}
#endif
			err = sh_css_pipe_configure_output(
					internal_pipe,
					tmp_output_info[i]->res.width,
					tmp_output_info[i]->res.height,
					tmp_output_info[i]->padded_width,
					tmp_output_info[i]->format,
					i);
			if (err != IA_CSS_SUCCESS) {
				ia_css_debug_dtrace(IA_CSS_DEBUG_ERROR, "ia_css_pipe_create: "
								"invalid output info\n");
				sh_css_free(internal_pipe);
				internal_pipe = NULL;
				return err;
			}
		}

		/* handle vf output info, when configured */
		*tmp_enable_viewfinder[i] = (tmp_vf_output_info[i]->res.width != 0);
		if (tmp_vf_output_info[i]->res.width) {
			err = sh_css_pipe_configure_viewfinder(
					internal_pipe,
					tmp_vf_output_info[i]->res.width,
					tmp_vf_output_info[i]->res.height,
					tmp_vf_output_info[i]->padded_width,
					tmp_vf_output_info[i]->format,
					i);
			if (err != IA_CSS_SUCCESS) {
				ia_css_debug_dtrace(IA_CSS_DEBUG_ERROR, "ia_css_pipe_create: "
								"invalid vf output info\n");
				sh_css_free(internal_pipe);
				internal_pipe = NULL;
				return err;
			}
		}
	}
	if (internal_pipe->config.acc_extension) {
		ia_css_pipe_load_extension(internal_pipe,
			internal_pipe->config.acc_extension);
	}
	/* set all info to zeroes first */
	memset(&internal_pipe->info, 0, sizeof(internal_pipe->info));

	/* all went well, return the pipe */
	*pipe = internal_pipe;
	return IA_CSS_SUCCESS;
}


enum ia_css_err
ia_css_pipe_get_info(const struct ia_css_pipe *pipe,
		     struct ia_css_pipe_info *pipe_info)
{
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_pipe_get_info()\n");
	assert(pipe_info != NULL);
	if (pipe_info == NULL) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_ERROR,
			"ia_css_pipe_get_info: pipe_info cannot be NULL\n");
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	}
	if (pipe == NULL || pipe->stream == NULL) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_ERROR,
			"ia_css_pipe_get_info: ia_css_stream_create needs to"
			" be called before ia_css_[stream/pipe]_get_info\n");
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	}
	/* we succeeded return the info */
	*pipe_info = pipe->info;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_pipe_get_info() leave\n");
	return IA_CSS_SUCCESS;
}

#if defined(USE_INPUT_SYSTEM_VERSION_2)
/* Configuration of INPUT_SYSTEM_VERSION_2401 is done on SP */
static enum ia_css_err
ia_css_stream_configure_rx(struct ia_css_stream *stream)
{
	struct ia_css_input_port *config;
	assert(stream != NULL);

	config = &stream->config.source.port;
// AM: this code is not reliable, especially for 2400
	if (config->num_lanes == 1)
		stream->csi_rx_config.mode = MONO_1L_1L_0L;
	else if (config->num_lanes == 2)
		stream->csi_rx_config.mode = STEREO_2L_1L_2L;
	else if (config->num_lanes == 3)
		stream->csi_rx_config.mode = MONO_3L_1L_0L;
	else if (config->num_lanes == 4)
		stream->csi_rx_config.mode = MONO_4L_1L_0L;
	else if (config->num_lanes != 0)
		return IA_CSS_ERR_INVALID_ARGUMENTS;

	if (config->port == IA_CSS_CSI2_PORT1)
		stream->csi_rx_config.port = MIPI_PORT1_ID;
	else if (config->port == IA_CSS_CSI2_PORT2)
		stream->csi_rx_config.port = MIPI_PORT2_ID;
	else if (config->port == IA_CSS_CSI2_PORT0)
		stream->csi_rx_config.port = MIPI_PORT0_ID;
	else
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	stream->csi_rx_config.timeout    = config->timeout;
	stream->csi_rx_config.initcount  = 0;
	stream->csi_rx_config.synccount  = 0x28282828;
	stream->csi_rx_config.rxcount    = config->rxcount;
	if (config->compression.type == IA_CSS_CSI2_COMPRESSION_TYPE_NONE)
		stream->csi_rx_config.comp = MIPI_PREDICTOR_NONE;
	else {
		/* not implemented yet, requires extension of the rx_cfg_t
		 * struct */
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	}
	stream->csi_rx_config.is_two_ppc = (stream->config.pixels_per_clock == 2);
	stream->reconfigure_css_rx = true;
	return IA_CSS_SUCCESS;
}
#endif

static struct ia_css_pipe *
find_pipe(struct ia_css_pipe *pipes[],
		unsigned int num_pipes,
		enum ia_css_pipe_mode mode,
		bool copy_pipe)
{
	unsigned i;
	assert(pipes != NULL);
	for (i = 0; i < num_pipes; i++) {
		assert(pipes[i] != NULL);
		if (pipes[i]->config.mode != mode)
			continue;
		if (copy_pipe && pipes[i]->mode != IA_CSS_PIPE_ID_COPY)
			continue;
		return pipes[i];
	}
	return NULL;
}

static enum ia_css_err
ia_css_acc_stream_create(struct ia_css_stream *stream)
{
	int i;
	enum ia_css_err err = IA_CSS_SUCCESS;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"ia_css_acc_stream_create() enter:\n");

	assert(stream != NULL);

	for (i = 0;  i< stream->num_pipes; i++) {
		struct ia_css_pipe *pipe = stream->pipes[i];
		assert(pipe != NULL);
		pipe->stream = stream;
	}

	/* Map SP threads before doing anything. */
	err = map_sp_threads(stream, true);
	if (err != IA_CSS_SUCCESS) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
			"ia_css_acc_stream_create(): map_sp_threads: err=%d\n", err);
		return err;
	}

	for (i = 0;  i< stream->num_pipes; i++) {
		struct ia_css_pipe *pipe = stream->pipes[i];
		assert(pipe != NULL);
		ia_css_pipe_map_queue(pipe, true);
	}

	err = create_host_pipeline_structure(stream);
	if (err != IA_CSS_SUCCESS) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
			"ia_css_acc_stream_create(): create_host_pipeline_structure: err=%d\n", err);
		return err;
	}

	stream->started = false;


	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
		"ia_css_acc_stream_create() leave:\n");

	return IA_CSS_SUCCESS;
}

static enum ia_css_err
metadata_info_init(const struct ia_css_metadata_config *mdc,
		   struct ia_css_metadata_info *md)
{
	/* Either both width and height should be set or neither */
	if ((mdc->resolution.height > 0) ^ (mdc->resolution.width > 0))
		return IA_CSS_ERR_INVALID_ARGUMENTS;

	md->resolution = mdc->resolution;
        /* We round up the stride to a multiple of the width
         * of the port going to DDR, this is a HW requirements (DMA). */
	md->stride = CEIL_MUL(mdc->resolution.width, HIVE_ISP_DDR_WORD_BYTES);
	md->size = mdc->resolution.height * md->stride;
	return IA_CSS_SUCCESS;
}

enum ia_css_err
ia_css_stream_create(const struct ia_css_stream_config *stream_config,
					 int num_pipes,
					 struct ia_css_pipe *pipes[],
					 struct ia_css_stream **stream)
{
	struct ia_css_pipe *curr_pipe;
	struct ia_css_stream *curr_stream = NULL;
	bool spcopyonly;
	bool sensor_binning_changed;
	int i;
	enum ia_css_err err = IA_CSS_ERR_INTERNAL_ERROR;
	struct ia_css_metadata_info md_info;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_stream_create() enter, num_pipes=%d\n", num_pipes);
	ia_css_debug_dump_stream_config(stream_config, num_pipes);

	/* some checks */
	if (num_pipes == 0 ||
		stream == NULL ||
		pipes == NULL) {
		err = IA_CSS_ERR_INVALID_ARGUMENTS;
		goto ERR;
	}

	/* We don't support metadata for JPEG stream, since they both use str2mem */
	if (stream_config->format == IA_CSS_STREAM_FORMAT_BINARY_8 &&
	    stream_config->metadata_config.resolution.height > 0)
		return IA_CSS_ERR_INVALID_ARGUMENTS;

#if !defined(HAS_NO_INPUT_SYSTEM)
	ia_css_debug_pipe_graph_dump_stream_config(stream_config);

	/* check if mipi size specified */
	if (stream_config->mode == IA_CSS_INPUT_MODE_BUFFERED_SENSOR)
#ifdef USE_INPUT_SYSTEM_VERSION_2401
	if (stream_config->online)
#endif
	{
		if (my_css.size_mem_words == 0) {
			ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
				"ia_css_stream_create() exit, need to set mipi frame size\n");
			assert(my_css.size_mem_words != 0);
			err = IA_CSS_ERR_INTERNAL_ERROR;
			goto ERR;
		}
	}
#endif

	/* Currently we only supported metadata up to a certain size. */
	err = metadata_info_init(&stream_config->metadata_config, &md_info);
	if (err != IA_CSS_SUCCESS)
		goto ERR;

	/* allocate the stream instance */
	curr_stream = sh_css_malloc(sizeof(struct ia_css_stream));
	if (curr_stream == NULL) {
		err = IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
		goto ERR;
	}
	/* default all to 0 */
	memset(curr_stream, 0, sizeof(struct ia_css_stream));
	curr_stream->info.metadata_info = md_info;

	/* allocate pipes */
	curr_stream->num_pipes = num_pipes;
	curr_stream->pipes = sh_css_malloc(num_pipes * sizeof(struct ia_css_pipe *));
	if (curr_stream->pipes == NULL) {
		sh_css_free(curr_stream);
		err = IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
		goto ERR;
	}
	/* store pipes */
	spcopyonly = (num_pipes == 1) && (pipes[0]->config.mode == IA_CSS_PIPE_MODE_COPY);
	for (i = 0; i < num_pipes; i++)
		curr_stream->pipes [i] = pipes[i];
	curr_stream->last_pipe = curr_stream->pipes[0];
	/* take over stream config */
	curr_stream->config = *stream_config;

#ifdef USE_INPUT_SYSTEM_VERSION_2401
	if (curr_stream->config.online) {
		curr_stream->config.source.port.num_lanes = stream_config->source.port.num_lanes;
		curr_stream->config.mode =  IA_CSS_INPUT_MODE_BUFFERED_SENSOR;
	}
#endif
	/* TO BE REMOVED when all drivers move to CSS API 2.1 */
	if (curr_stream->config.pixels_per_clock == 0)
		curr_stream->config.pixels_per_clock =
			curr_stream->config.two_pixels_per_clock ? 2 : 1;

	/* in case driver doesn't configure init number of raw buffers, configure it here */
	if (curr_stream->config.target_num_cont_raw_buf == 0)
		curr_stream->config.target_num_cont_raw_buf = NUM_CONTINUOUS_FRAMES;
	if (curr_stream->config.init_num_cont_raw_buf == 0)
		curr_stream->config.init_num_cont_raw_buf = curr_stream->config.target_num_cont_raw_buf;

	/* copy mode specific stuff */
	switch (curr_stream->config.mode) {
		case IA_CSS_INPUT_MODE_SENSOR:
		case IA_CSS_INPUT_MODE_BUFFERED_SENSOR:
#if defined(USE_INPUT_SYSTEM_VERSION_2)
		ia_css_stream_configure_rx(curr_stream);
#endif
		break;
	case IA_CSS_INPUT_MODE_TPG:
#if !defined(HAS_NO_INPUT_SYSTEM) && defined(USE_INPUT_SYSTEM_VERSION_2)
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
			"ia_css_stream_create tpg_configuration: "
			"x_mask=%d, y_mask=%d, x_delta=%d, "
			"y_delta=%d, xy_mask=%d\n",
			curr_stream->config.source.tpg.x_mask,
			curr_stream->config.source.tpg.y_mask,
			curr_stream->config.source.tpg.x_delta,
			curr_stream->config.source.tpg.y_delta,
			curr_stream->config.source.tpg.xy_mask);

		sh_css_sp_configure_tpg(
			curr_stream->config.source.tpg.x_mask,
			curr_stream->config.source.tpg.y_mask,
			curr_stream->config.source.tpg.x_delta,
			curr_stream->config.source.tpg.y_delta,
			curr_stream->config.source.tpg.xy_mask);
#endif
		break;
	case IA_CSS_INPUT_MODE_PRBS:
#if !defined(HAS_NO_INPUT_SYSTEM) && defined(USE_INPUT_SYSTEM_VERSION_2)
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_stream_create: mode prbs\n");
		sh_css_sp_configure_prbs(curr_stream->config.source.prbs.seed);
#endif
		break;
	case IA_CSS_INPUT_MODE_MEMORY:
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_stream_create: mode memory\n");
		curr_stream->reconfigure_css_rx = false;
		break;
	default:
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_stream_create: mode sensor/default\n");
	}
	err = ia_css_stream_isp_parameters_init(curr_stream);
	if (err != IA_CSS_SUCCESS)
		goto ERR;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_stream_create: isp_params_configs: %p\n",curr_stream->isp_params_configs);

	if (num_pipes == 1 && pipes[0]->config.mode == IA_CSS_PIPE_MODE_ACC) {
		*stream = curr_stream;
		err = ia_css_acc_stream_create(curr_stream);
		goto ERR;
	}
	/* sensor binning */
	if (!spcopyonly){
		sensor_binning_changed =
			sh_css_params_set_binning_factor(curr_stream, curr_stream->config.sensor_binning_factor);
	} else {
		sensor_binning_changed = false;
	}

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_stream_create: sensor_binning=%d, changed=%d\n",
		curr_stream->config.sensor_binning_factor, sensor_binning_changed);
	/* loop over pipes */
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_stream_create: num_pipes=%d\n",
		num_pipes);
	curr_stream->cont_capt = false;
	/* Temporary hack: we give the preview pipe a reference to the capture
	 * pipe in continuous capture mode. */
	if (curr_stream->config.continuous) {
		/* Search for the preview pipe and create the copy pipe */
		struct ia_css_pipe *preview_pipe;
		struct ia_css_pipe *video_pipe;
		struct ia_css_pipe *capture_pipe = NULL;
		struct ia_css_pipe *copy_pipe = NULL;

		if (num_pipes >= 2)
			curr_stream->cont_capt = true;

		/* Create copy pipe here, since it may not be exposed to the driver */
		preview_pipe = find_pipe(pipes, num_pipes,
						IA_CSS_PIPE_MODE_PREVIEW, false);
		video_pipe = find_pipe(pipes, num_pipes,
						IA_CSS_PIPE_MODE_VIDEO, false);
		if (curr_stream->cont_capt == true) {
			capture_pipe = find_pipe(pipes, num_pipes,
						IA_CSS_PIPE_MODE_CAPTURE, false);
			if (capture_pipe == NULL) {
				err = IA_CSS_ERR_INTERNAL_ERROR;
				goto ERR;
			}
		}
		/* We do not support preview and video pipe at the same time */
		if (preview_pipe && video_pipe) {
			err = IA_CSS_ERR_INVALID_ARGUMENTS;
			goto ERR;
		}

		if (preview_pipe && !preview_pipe->pipe_settings.preview.copy_pipe) {
			err = create_pipe(IA_CSS_PIPE_MODE_CAPTURE, &copy_pipe, true);
			if (err != IA_CSS_SUCCESS)
				goto ERR;
			ia_css_pipe_config_defaults(&copy_pipe->config);
			preview_pipe->pipe_settings.preview.copy_pipe = copy_pipe;
			copy_pipe->stream = curr_stream;
		}
		if (preview_pipe && (curr_stream->cont_capt == true)) {
			preview_pipe->pipe_settings.preview.capture_pipe = capture_pipe;
		}
		if (video_pipe && !video_pipe->pipe_settings.video.copy_pipe) {
			err = create_pipe(IA_CSS_PIPE_MODE_CAPTURE, &copy_pipe, true);
			if (err != IA_CSS_SUCCESS)
				goto ERR;
			ia_css_pipe_config_defaults(&copy_pipe->config);
			video_pipe->pipe_settings.video.copy_pipe = copy_pipe;
			copy_pipe->stream = curr_stream;
		}
		if (video_pipe && (curr_stream->cont_capt == true)) {
			video_pipe->pipe_settings.video.capture_pipe = capture_pipe;
		}
	}
	for (i = 0; i < num_pipes; i++) {
		curr_pipe = pipes[i];
		/* set current stream */
		curr_pipe->stream = curr_stream;
		/* take over effective info */
#if defined(USE_INPUT_SYSTEM_VERSION_2401)
		if ((curr_stream->config.format >= IA_CSS_STREAM_FORMAT_USER_DEF1) &&
		    (curr_stream->config.format <= IA_CSS_STREAM_FORMAT_USER_DEF8)) {
			curr_stream->config.effective_res.height =
				ceil_div(curr_stream->config.effective_res.width, HIVE_ISP_DDR_WORD_BYTES);
			curr_stream->config.effective_res.width = HIVE_ISP_DDR_WORD_BYTES;
		}
#endif
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_stream_create: effective_res=%dx%d\n",
			curr_stream->config.effective_res.width,
			curr_stream->config.effective_res.height);

		err = ia_css_util_check_res(
			curr_stream->config.effective_res.width,
			curr_stream->config.effective_res.height);
		if (err != IA_CSS_SUCCESS)
			goto ERR;

		if (curr_stream->info.effective_info.width != curr_stream->config.effective_res.width ||
			curr_stream->info.effective_info.height != curr_stream->config.effective_res.height) {
			curr_stream->info.effective_info.width = curr_stream->config.effective_res.width;
			curr_stream->info.effective_info.height = curr_stream->config.effective_res.height;
		}
		/* sensor binning per pipe */
		if (sensor_binning_changed)
			sh_css_pipe_free_shading_table(curr_pipe);
	}

	/* now pipes have been configured, info should be available */
	for (i = 0; i < num_pipes; i++) {
		struct ia_css_pipe_info *pipe_info = NULL;
		curr_pipe = pipes[i];

		err = sh_css_pipe_load_binaries(curr_pipe);
		if (err != IA_CSS_SUCCESS)
			goto ERR;

		/* handle each pipe */
		pipe_info = &curr_pipe->info;
		err = sh_css_pipe_get_output_frame_info(curr_pipe,
				&pipe_info->output_info, 0);
		if (err != IA_CSS_SUCCESS)
			goto ERR;
		err = sh_css_pipe_get_output_frame_info(curr_pipe,
				&pipe_info->second_output_info, 1);
		if (err != IA_CSS_SUCCESS)
			goto ERR;
		if (!spcopyonly){
			err = sh_css_pipe_get_grid_info(curr_pipe,
						&pipe_info->grid_info);
			if (err != IA_CSS_SUCCESS)
				goto ERR;
			sh_css_pipe_get_viewfinder_frame_info(curr_pipe,
					&pipe_info->vf_output_info, 0);
			if (err != IA_CSS_SUCCESS)
				goto ERR;
			sh_css_pipe_get_viewfinder_frame_info(curr_pipe,
					&pipe_info->second_vf_output_info, 1);
			if (err != IA_CSS_SUCCESS)
				goto ERR;
		}

		my_css.active_pipes[ia_css_pipe_get_pipe_num(curr_pipe)] = curr_pipe;
	}
	/* this is a legacy from CSS 1.5, we still need this here because we need this info
	   to configure the sensor ouput in CameraDriver, but it is not needed in real device.
	   add this macro to make this explicit. */
#if defined(HRT_CSIM) || defined(HRT_RTL)
	/* stream has been configured, info should be available */
	{
		/* use first pipe in list, should be identical anyway */
		// TODO: should come from stream something
		struct ia_css_stream_info *stream_info = &curr_stream->info;
		// TODO: JB implement stream info
		err = sh_css_pipe_get_input_resolution(pipes[0],
				&stream_info->raw_info.width,
				&stream_info->raw_info.height);
		if (err != IA_CSS_SUCCESS)
			goto ERR;
	}
#endif

	curr_stream->started = false;

	/* Map SP threads before doing anything. */
	err = map_sp_threads(curr_stream, true);
	if (err != IA_CSS_SUCCESS) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
			"ia_css_stream_create(): map_sp_threads: return_err=%d\n", err);
	}

	for (i = 0; i < num_pipes; i++) {
		curr_pipe = pipes[i];
		ia_css_pipe_map_queue(curr_pipe, true);
	}

	/* Create host side pipeline objects without stages */
	err = create_host_pipeline_structure(curr_stream);
	if (err != IA_CSS_SUCCESS) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
			"ia_css_stream_create(): create_host_pipeline_structure: err=%d\n", err);
		goto ERR;
	}

	/* assign curr_stream */
	*stream = curr_stream;

ERR:
	if (err == IA_CSS_SUCCESS)
	{
		/* working mode: enter into the seed list */
		if (my_css_save.mode == sh_css_mode_working)
		for(i=0;i<MAX_ACTIVE_STREAMS;i++)
			if (my_css_save.stream_seeds[i].stream == NULL)
			{
				int j;
				ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "ia_css_stream_create(): entered stream into loc=%d\n", i);
				my_css_save.stream_seeds[i].orig_stream = stream;
				my_css_save.stream_seeds[i].stream = curr_stream;
				my_css_save.stream_seeds[i].num_pipes = num_pipes;
				my_css_save.stream_seeds[i].stream_config = *stream_config;
				for(j=0;j<num_pipes;j++)
				{
					my_css_save.stream_seeds[i].pipe_config[j] = pipes[j]->config;
					my_css_save.stream_seeds[i].pipes[j] = pipes[j];
				}
				break;
			}
	}
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE, "ia_css_stream_create() leave, err=%d mode=%d\n",
			err, my_css_save.mode);
	return err;
}

enum ia_css_err
ia_css_stream_destroy(struct ia_css_stream *stream)
{
	int i;
	enum ia_css_err err = IA_CSS_SUCCESS;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_stream_destroy: enter\n");
	assert(stream != NULL);

	ia_css_stream_isp_parameters_uninit(stream);

#if defined(USE_INPUT_SYSTEM_VERSION_2401)
	stream_unregister_with_csi_rx(stream);
#endif

	for (i = 0; i < stream->num_pipes; i++) {
		struct ia_css_pipe *curr_pipe = stream->pipes[i];
		assert(curr_pipe != NULL);
		ia_css_pipe_map_queue(curr_pipe, false);
	}

	err = map_sp_threads(stream, false);
	if (err != IA_CSS_SUCCESS) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,
			"ia_css_stream_destroy(): map_sp_threads: return_err=%d\n", err);
		return err;
	}

	/* remove references from pipes to stream */
	for (i = 0; i < stream->num_pipes; i++) {
		struct ia_css_pipe *entry = stream->pipes[i];
		assert(entry != NULL);
		if (entry != NULL) {
			/* clear reference to stream */
			entry->stream = NULL;
			/* check internal copy pipe */
			if (entry->mode == IA_CSS_PIPE_ID_PREVIEW &&
			    entry->pipe_settings.preview.copy_pipe) {
				ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
					"ia_css_stream_destroy: "
					"clearing stream on internal preview copy pipe\n");
				entry->pipe_settings.preview.copy_pipe->stream = NULL;
			}
			if (entry->mode == IA_CSS_PIPE_ID_VIDEO &&
				entry->pipe_settings.video.copy_pipe) {
				ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
					"ia_css_stream_destroy: "
					"clearing stream on internal video copy pipe\n");
				entry->pipe_settings.video.copy_pipe->stream = NULL;
			}
			sh_css_pipe_unload_binaries(entry);
		}
	}
	/* free associated memory of stream struct */
	sh_css_free(stream->pipes);
	stream->pipes = NULL;
	stream->num_pipes = 0;
	/* working mode: take out of the seed list */
	if (my_css_save.mode == sh_css_mode_working)
		for(i=0;i<MAX_ACTIVE_STREAMS;i++)
			if (my_css_save.stream_seeds[i].stream == stream)
			{
				ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE_PRIVATE,"ia_css_stream_destroy: took out stream %d", i);
				my_css_save.stream_seeds[i].stream = NULL;
				break;
			}
	sh_css_free(stream);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_stream_destroy: leave\n");

	return err;
}

enum ia_css_err
ia_css_stream_get_info(const struct ia_css_stream *stream,
		       struct ia_css_stream_info *stream_info)
{
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_stream_get_info: enter/exit\n");
	assert(stream != NULL);
	assert(stream_info != NULL);

	*stream_info = stream->info;
	return IA_CSS_SUCCESS;
}

/*
 * Rebuild a stream, including allocating structs, setting configuration and
 * building the required pipes.
 * The data is taken from the css_save struct updated upon stream creation.
 * The stream handle is used to identify the correct entry in the css_save struct
 */
enum ia_css_err
ia_css_stream_load(struct ia_css_stream *stream)
{
	int i;
	enum ia_css_err err;
	assert(stream != NULL);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,	"ia_css_stream_load() enter, \n");
	for(i=0;i<MAX_ACTIVE_STREAMS;i++)
		if (my_css_save.stream_seeds[i].stream == stream)
		{
			int j;
			for(j=0;j<my_css_save.stream_seeds[i].num_pipes;j++)
				if ((err = ia_css_pipe_create(&(my_css_save.stream_seeds[i].pipe_config[j]), &my_css_save.stream_seeds[i].pipes[j])) != IA_CSS_SUCCESS)
				{
					if (j)
					{
						int k;
						for(k=0;k<j;k++)
							ia_css_pipe_destroy(my_css_save.stream_seeds[i].pipes[k]);
					}
					return(err);
				}
			err = ia_css_stream_create(&(my_css_save.stream_seeds[i].stream_config), my_css_save.stream_seeds[i].num_pipes,
						    my_css_save.stream_seeds[i].pipes, &(my_css_save.stream_seeds[i].stream));
		    if (err != IA_CSS_SUCCESS)
			{
				ia_css_stream_destroy(stream);
				for(j=0;j<my_css_save.stream_seeds[i].num_pipes;j++)
					ia_css_pipe_destroy(my_css_save.stream_seeds[i].pipes[j]);
				return(err);
			}
			break;
		}
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,	"ia_css_stream_load() exit, \n");
	return(IA_CSS_SUCCESS);
}

enum ia_css_err
ia_css_stream_start(struct ia_css_stream *stream)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_stream_start()\n");
	assert(stream != NULL);
	assert(stream->last_pipe != NULL);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_stream_start: starting %d\n",
		stream->last_pipe->mode);

	/* Create host side pipeline. */
	err = create_host_pipeline(stream);
	if (err != IA_CSS_SUCCESS)
		return err;

#if !defined(HAS_NO_INPUT_SYSTEM)
#if defined(USE_INPUT_SYSTEM_VERSION_2401)
	if((stream->config.mode == IA_CSS_INPUT_MODE_SENSOR) ||
	   (stream->config.mode == IA_CSS_INPUT_MODE_BUFFERED_SENSOR))
		stream_register_with_csi_rx(stream);
#endif
#endif

#if !defined(HAS_NO_INPUT_SYSTEM) && defined(USE_INPUT_SYSTEM_VERSION_2)
	// Initialize mipi size checks
	if (stream->config.mode == IA_CSS_INPUT_MODE_BUFFERED_SENSOR)
	{
		unsigned int idx;
		unsigned int port = (unsigned int) (stream->config.source.port.port) ;

		for (idx = 0; idx < IA_CSS_MIPI_SIZE_CHECK_MAX_NOF_ENTRIES_PER_PORT; idx++) {
			sh_css_sp_group.config.mipi_sizes_for_check[port][idx] =  sh_css_get_mipi_sizes_for_check(port, idx);
		}
	}
#endif

	return sh_css_pipe_start(stream);
}

enum ia_css_err
ia_css_stream_stop(struct ia_css_stream *stream)
{
	enum ia_css_err err = IA_CSS_SUCCESS;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_stream_stop() enter/exit\n");
	assert(stream != NULL);
	assert(stream->last_pipe != NULL);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_stream_stop: stopping %d\n",
		stream->last_pipe->mode);

	// Check if this is the last pipe, if not return else stop everything
	//if (pipe_num_counter > 1) return IA_CSS_SUCCESS;

#if !defined(HAS_NO_INPUT_SYSTEM) && defined(USE_INPUT_SYSTEM_VERSION_2)
	// De-initialize mipi size checks
	if (stream->config.mode == IA_CSS_INPUT_MODE_BUFFERED_SENSOR)
	{
		unsigned int idx;
		unsigned int port = (unsigned int) (stream->config.source.port.port) ;

		for (idx = 0; idx < IA_CSS_MIPI_SIZE_CHECK_MAX_NOF_ENTRIES_PER_PORT; idx++) {
			sh_css_sp_group.config.mipi_sizes_for_check[port][idx] = 0;
		}
	}
#endif
	err = ia_css_pipeline_request_stop(&stream->last_pipe->pipeline);
	if (err != IA_CSS_SUCCESS)
		return err;

	/* Ideally, unmapping should happen after pipeline_stop, but current
	 * semantics do not allow that. */
	/* err = map_sp_threads(stream, false); */

	return err;
}

bool
ia_css_stream_has_stopped(struct ia_css_stream *stream)
{
	bool stopped;
	assert(stream != NULL);

	stopped = ia_css_pipeline_has_stopped(&stream->last_pipe->pipeline);

	return stopped;
}

/*
 * Destroy the stream and all the pipes related to it.
 * The stream handle is used to identify the correct entry in the css_save struct
 */
enum ia_css_err
ia_css_stream_unload(struct ia_css_stream *stream)
{
	int i;
	assert(stream != NULL);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,	"ia_css_stream_unload() enter, \n");
	/* some checks */
	assert (stream != NULL);
	for(i=0;i<MAX_ACTIVE_STREAMS;i++)
		if (my_css_save.stream_seeds[i].stream == stream)
		{
			int j;
			ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,	"ia_css_stream_unload(): unloading %d (%p)\n", i, my_css_save.stream_seeds[i].stream);
			ia_css_stream_destroy(stream);
			for(j=0;j<my_css_save.stream_seeds[i].num_pipes;j++)
				ia_css_pipe_destroy(my_css_save.stream_seeds[i].pipes[j]);
			ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,	"ia_css_stream_unload(): after unloading %d (%p)\n", i, my_css_save.stream_seeds[i].stream);
			break;
		}
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,	"ia_css_stream_unload() exit, \n");
	return(IA_CSS_SUCCESS);
}

enum ia_css_err
ia_css_temp_pipe_to_pipe_id(const struct ia_css_pipe *pipe, enum ia_css_pipe_id *pipe_id)
{
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_temp_pipe_to_pipe_id() enter/exit\n");
	if (pipe != NULL)
		*pipe_id = pipe->mode;
	else
		*pipe_id = IA_CSS_PIPE_ID_COPY;

	return IA_CSS_SUCCESS;
}

enum ia_css_stream_format
ia_css_stream_get_format(const struct ia_css_stream *stream)
{
	return stream->config.format;
}

bool
ia_css_stream_get_two_pixels_per_clock(const struct ia_css_stream *stream)
{
	return (stream->config.pixels_per_clock == 2);
}

struct ia_css_binary *
ia_css_stream_get_dvs_binary(const struct ia_css_stream *stream)
{
	int i;
	struct ia_css_pipe *video_pipe = NULL;

	/* First we find the video pipe */
	for (i=0; i<stream->num_pipes; i++) {
		struct ia_css_pipe *pipe = stream->pipes[i];
		if (pipe->config.mode == IA_CSS_PIPE_MODE_VIDEO) {
			video_pipe = pipe;
			break;
		}
	}
	if (video_pipe)
		return &video_pipe->pipe_settings.video.video_binary;
	return NULL;
}

struct ia_css_binary *
ia_css_stream_get_3a_binary(const struct ia_css_stream *stream)
{
	struct ia_css_pipe *pipe;

	assert(stream != NULL);

	pipe = stream->pipes[0];

	if (stream->num_pipes == 2) {
		assert(stream->pipes[1] != NULL);
		if (stream->pipes[1]->config.mode == IA_CSS_PIPE_MODE_VIDEO ||
		    stream->pipes[1]->config.mode == IA_CSS_PIPE_MODE_PREVIEW)
			pipe = stream->pipes[1];
	}

	return ia_css_pipe_get_3a_binary(pipe);
}

enum ia_css_err
ia_css_stream_set_output_padded_width(struct ia_css_stream *stream, unsigned int output_padded_width)
{
	enum ia_css_err err = IA_CSS_SUCCESS;

	struct ia_css_pipe *pipe;

	assert(stream != NULL);

	pipe = stream->last_pipe;

	assert(pipe != NULL);

	// set the config also just in case (redundant info? why do we save config in pipe?)
	pipe->config.output_info.padded_width = output_padded_width;
	pipe->output_info[0].padded_width = output_padded_width;

	return err;
}


static struct ia_css_binary *
ia_css_pipe_get_3a_binary (const struct ia_css_pipe *pipe)
{
	struct ia_css_binary *s3a_binary = NULL;

	assert(pipe != NULL);

	switch (pipe->config.mode) {
	case IA_CSS_PIPE_MODE_PREVIEW:
		s3a_binary = (struct ia_css_binary*)&pipe->pipe_settings.preview.preview_binary;
		break;
	case IA_CSS_PIPE_MODE_VIDEO:
		s3a_binary = (struct ia_css_binary*)&pipe->pipe_settings.video.video_binary;
		break;
	case IA_CSS_PIPE_MODE_CAPTURE:
		if (pipe->config.default_capture_config.mode == IA_CSS_CAPTURE_MODE_PRIMARY)
			s3a_binary = (struct ia_css_binary*)&pipe->pipe_settings.capture.primary_binary;
		else if (pipe->config.default_capture_config.mode == IA_CSS_CAPTURE_MODE_ADVANCED ||
			 pipe->config.default_capture_config.mode == IA_CSS_CAPTURE_MODE_LOW_LIGHT ||
			 pipe->config.default_capture_config.mode == IA_CSS_CAPTURE_MODE_BAYER) {
			s3a_binary
				= (struct ia_css_binary*)&pipe->pipe_settings.capture.pre_isp_binary;
		}
		break;
	default:
		break;
	}

	if (s3a_binary && s3a_binary->info->sp.enable.s3a)
		return s3a_binary;

	return NULL;
}

#if defined(IS_ISP_2500_SYSTEM)
void ia_css_pipe_get_bds_resolution(const struct ia_css_pipe *pipe, struct ia_css_resolution *res)
{
	assert(pipe != NULL);

	*res = pipe->bds_output_info.res;
}

void ia_css_pipe_get_dvs_envelope(const struct ia_css_pipe *pipe, struct ia_css_resolution *res)
{
	assert(pipe != NULL);

	*res = pipe->config.dvs_envelope;
}
#endif

struct ia_css_pipeline *
ia_css_pipe_get_pipeline(const struct ia_css_pipe *pipe)
{
	assert(pipe != NULL);

	return (struct ia_css_pipeline*)&pipe->pipeline;
}

unsigned int
ia_css_pipe_get_pipe_num(const struct ia_css_pipe *pipe)
{
	assert(pipe != NULL);

	/* KW was not sure this function was not returning a value
	   that was out of range; so added an assert, and, for the
	   case when asserts are not enabled, clip to the largest
	   value; pipe_num is unsigned so the value cannot be too small
	*/
	assert(pipe->pipe_num < IA_CSS_PIPELINE_NUM_MAX);

	if (pipe->pipe_num >= IA_CSS_PIPELINE_NUM_MAX)
		return (IA_CSS_PIPELINE_NUM_MAX - 1);

	return pipe->pipe_num;
}


unsigned int
ia_css_pipe_get_isp_pipe_version(const struct ia_css_pipe *pipe)
{
	assert(pipe != NULL);

	return pipe->config.isp_pipe_version;
}

#define SP_START_TIMEOUT_US 30000000

#if defined(IS_ISP_2500_SYSTEM)
void
ia_css_start_sp1(void)
{
	unsigned long timeout;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_start_sp1() enter\n");
	sh_css_sp1_start();
	/* waiting for the SP is completely started */
	timeout = SP_START_TIMEOUT_US;
	while((ia_css_sp1ctrl_get_state(SP1_ID) != IA_CSS_SP_SW_INITIALIZED) && timeout) {
		timeout--;
		hrt_sleep();
	}
	if (timeout == 0) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_ERROR, "ia_css_start_sp1() timeout\n");
		return; /* IA_CSS_ERR_INTERNAL_ERROR; */
	}
	sh_css_write_host2sp1_command(host2sp_cmd_ready);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_start_sp1() exit\n");
}
#endif
enum ia_css_err
ia_css_start_sp(void)
{
	unsigned long timeout;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_start_sp() enter\n");
	sh_css_sp_start_isp();

	/* waiting for the SP is completely started */
	timeout = SP_START_TIMEOUT_US;
	while((ia_css_spctrl_get_state(SP0_ID) != IA_CSS_SP_SW_INITIALIZED) && timeout) {
		timeout--;
		hrt_sleep();
	}
	if (timeout == 0) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_ERROR, "ia_css_start_sp() timeout\n");
		return IA_CSS_ERR_INTERNAL_ERROR;
	}

	/* Workaround, in order to run two streams in parallel. See TASK 4271*/
	/* TODO: Fix this. */

	sh_css_init_host_sp_control_vars();

	/* buffers should be initialized only when sp is started */
	/* AM: At the moment it will be done only when there is no stream active. */

	sh_css_setup_queues();

#if defined(IS_ISP_2500_SYSTEM)
	/* Start the SP1 Core */
	ia_css_start_sp1();
#endif

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_start_sp() exit\n");
	return IA_CSS_SUCCESS;
}

/**
 *	Time to wait SP for termincate. Only condition when this can happen
 *	is a fatal hw failure, but we must be able to detect this and emit
 *	a proper error trace.
 */
#define SP_SHUTDOWN_TIMEOUT_US 200000

#if defined(IS_ISP_2500_SYSTEM)
enum ia_css_err
ia_css_stop_sp1(void)
{
	unsigned long timeout;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "sh_css_stop_sp1() enter\n");
	/* For now, stop whole SP1 */
	sh_css_write_host2sp1_command(host2sp_cmd_terminate);
	sh_css_sp1_set_sp1_running(false);
#ifdef __KERNEL__
	printk("STOP_FUNC SP1: reach point 1\n");
#endif
	timeout = SP_SHUTDOWN_TIMEOUT_US;
	while ((ia_css_sp1ctrl_get_state(SP1_ID)!= IA_CSS_SP_SW_TERMINATED) && timeout) {
		timeout--;
		hrt_sleep();
	}
	if (timeout == 0) {
		ia_css_debug_dump_debug_info("sh_css_stop_sp1 point1");
		//ia_css_debug_dump_sp_sw_debug_info();
#ifdef __KERNEL__
		printk(KERN_ERR "%s poll timeout point 1!!!\n", __func__);
#endif
	}
#ifdef __KERNEL__
	printk("STOP_FUNC SP1: reach point 2\n");
#endif

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "sh_css_stop_sp1() exit\n");

	return IA_CSS_SUCCESS;
}
#endif //#if defined(IS_ISP_2500_SYSTEM)

enum ia_css_err
ia_css_stop_sp(void)
{
	unsigned int i;
	unsigned long timeout;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "sh_css_stop_sp() enter\n");
	/* For now, stop whole SP */
	sh_css_write_host2sp_command(host2sp_cmd_terminate);
	sh_css_sp_set_sp_running(false);
#ifdef __KERNEL__
	printk("STOP_FUNC: reach point 1\n");
#endif
	timeout = SP_SHUTDOWN_TIMEOUT_US;
	while ((ia_css_spctrl_get_state(SP0_ID)!= IA_CSS_SP_SW_TERMINATED) && timeout) {
		timeout--;
		hrt_sleep();
	}
	if (timeout == 0) {
		ia_css_debug_dump_debug_info("sh_css_stop_sp point1");
		ia_css_debug_dump_sp_sw_debug_info();
#ifdef __KERNEL__
		printk(KERN_ERR "%s poll timeout point 1!!!\n", __func__);
#endif
	}
#ifdef __KERNEL__
	printk("STOP_FUNC: reach point 2\n");
#endif
	while (!isp_ctrl_getbit(ISP0_ID, ISP_SC_REG, ISP_IDLE_BIT) && timeout) {
		timeout--;
		hrt_sleep();
	}
	if (timeout == 0) {
		ia_css_debug_dump_debug_info("sh_css_stop_sp point2");
		ia_css_debug_dump_sp_sw_debug_info();
#ifdef __KERNEL__
		printk(KERN_ERR "%s poll timeout point 2!!!\n", __func__);
#endif
	}
#ifdef __KERNEL__
	printk("STOP_FUNC: reach point 3\n");
#endif

	for (i = 0; i < MAX_HMM_BUFFER_NUM; i++) {
		if (hmm_buffer_record_h[i] != NULL) {
			ia_css_rmgr_rel_vbuf(hmm_buffer_pool, &hmm_buffer_record_h[i]);
		}
	}

	/* clear pending param sets from refcount */
	sh_css_param_clear_param_sets();

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "sh_css_stop_sp() exit\n");

#if defined(IS_ISP_2500_SYSTEM)
	/* Stop SP1 Core */
	ia_css_stop_sp1();
#endif
	return IA_CSS_SUCCESS;
}

enum ia_css_err
ia_css_update_continuous_frames(struct ia_css_stream *stream)
{
	struct ia_css_pipe *pipe;
	unsigned int i;

	ia_css_debug_dtrace(
	    IA_CSS_DEBUG_TRACE,
	    "sh_css_update_continuous_frames() enter:\n");

	if (stream == NULL) {
		ia_css_debug_dtrace(
			IA_CSS_DEBUG_TRACE,
			"sh_css_update_continuous_frames() leave: invalid stream, return_void\n");
			return IA_CSS_ERR_INVALID_ARGUMENTS;
	}

	pipe = stream->continuous_pipe;

	for (i = stream->config.init_num_cont_raw_buf;
				i < stream->config.target_num_cont_raw_buf; i++) {
		sh_css_update_host2sp_offline_frame(i,
				pipe->continuous_frames[i], pipe->cont_md_buffers[i]);
	}
	sh_css_update_host2sp_cont_num_raw_frames
			(stream->config.target_num_cont_raw_buf, true);
	ia_css_debug_dtrace(
	    IA_CSS_DEBUG_TRACE,
	    "sh_css_update_continuous_frames() leave: return_void\n");

	return IA_CSS_SUCCESS;
}

void ia_css_pipe_map_queue(struct ia_css_pipe *pipe, bool map)
{
	unsigned int thread_id;
	enum ia_css_pipe_id pipe_id;
	unsigned int pipe_num;

	assert(pipe != NULL);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_pipeline_create() enter:\n");

	pipe_id = pipe->mode;
	pipe_num = pipe->pipe_num;

	ia_css_pipeline_get_sp_thread_id(pipe_num, &thread_id);

	/* map required buffer queues to resources */
	/* TODO: to be improved */
	if (pipe->mode == IA_CSS_PIPE_ID_PREVIEW) {
#if defined(HAS_NO_INPUT_SYSTEM) || defined(USE_INPUT_SYSTEM_VERSION_2401)
		ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_INPUT_FRAME, map);
#endif
		ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_OUTPUT_FRAME, map);
		ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_PARAMETER_SET, map);
#if defined SH_CSS_ENABLE_METADATA
		ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_METADATA, map);
#endif
		if (pipe->pipe_settings.preview.preview_binary.info &&
			pipe->pipe_settings.preview.preview_binary.info->sp.enable.s3a)
			ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_3A_STATISTICS, map);
	} else if (pipe->mode == IA_CSS_PIPE_ID_CAPTURE) {
#if defined(HAS_NO_INPUT_SYSTEM) || defined(USE_INPUT_SYSTEM_VERSION_2401)
		ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_INPUT_FRAME, map);
#endif
		ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_OUTPUT_FRAME, map);
		ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME, map);
		ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_PARAMETER_SET, map);
#if defined SH_CSS_ENABLE_METADATA
		ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_METADATA, map);
#endif
		if ((pipe->pipe_settings.capture.primary_binary.info &&
			pipe->pipe_settings.capture.primary_binary.info->sp.enable.s3a) ||
			(pipe->pipe_settings.capture.pre_isp_binary.info &&
			pipe->pipe_settings.capture.pre_isp_binary.info->sp.enable.s3a))
			ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_3A_STATISTICS, map);
	} else if (pipe->mode == IA_CSS_PIPE_ID_VIDEO) {
#if defined(HAS_NO_INPUT_SYSTEM) || defined(USE_INPUT_SYSTEM_VERSION_2401)
		ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_INPUT_FRAME, map);
#endif
		ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_OUTPUT_FRAME, map);
		if (pipe->enable_viewfinder)
			ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME, map);
		ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_PARAMETER_SET, map);
#if defined SH_CSS_ENABLE_METADATA
		ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_METADATA, map);
#endif
		if (pipe->pipe_settings.video.video_binary.info &&
			pipe->pipe_settings.video.video_binary.info->sp.enable.s3a)
			ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_3A_STATISTICS, map);
		if (pipe->pipe_settings.video.video_binary.info &&
			pipe->pipe_settings.video.video_binary.info->sp.enable.dis)
			ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_DIS_STATISTICS, map);
#if defined(IS_ISP_2500_SYSTEM)
		if (pipe->pipe_settings.video.video_binary.info &&
			pipe->pipe_settings.video.video_binary.info->sp.enable.lace_stats)
			ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_LACE_STATISTICS, map);
#endif
	} else if (pipe->mode == IA_CSS_PIPE_ID_COPY) {
#if defined(HAS_NO_INPUT_SYSTEM) || defined(USE_INPUT_SYSTEM_VERSION_2401)
		ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_INPUT_FRAME, map);
#endif
		if (!pipe->stream->config.continuous)
			ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_OUTPUT_FRAME, map);
#if defined SH_CSS_ENABLE_METADATA
		ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_METADATA, map);
#endif
	} else if (pipe->mode == IA_CSS_PIPE_ID_ACC) {
#if defined(HAS_NO_INPUT_SYSTEM) || defined(USE_INPUT_SYSTEM_VERSION_2401)
		ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_INPUT_FRAME, map);
#endif
		ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_OUTPUT_FRAME, map);
		ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_PARAMETER_SET, map);
#if defined SH_CSS_ENABLE_METADATA
		ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_METADATA, map);
#endif
	} else if (pipe->mode == IA_CSS_PIPE_ID_YUVPP) {
		ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_OUTPUT_FRAME, map);
		ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_SEC_OUTPUT_FRAME, map);
		if (pipe->enable_viewfinder)
			ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME, map);
		if (pipe->enable_second_viewfinder)
			ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_SEC_VF_OUTPUT_FRAME, map);
		ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_PARAMETER_SET, map);
#if defined SH_CSS_ENABLE_METADATA
		ia_css_queue_map(thread_id, IA_CSS_BUFFER_TYPE_METADATA, map);
#endif
	}
}

#if CONFIG_ON_FRAME_ENQUEUE()
static enum ia_css_err set_config_on_frame_enqueue(struct ia_css_frame_info *info, struct frame_data_wrapper *frame)
{
	frame->config_on_frame_enqueue.padded_width = 0;

	// currently we support configuration on frame enqueue only on YUV formats
	// on other formats the padded_width is zeroed for no configuration override
	switch (info->format) {
	case IA_CSS_FRAME_FORMAT_YUV420:
	case IA_CSS_FRAME_FORMAT_NV12:
		if (info->padded_width > info->res.width)
		{
			frame->config_on_frame_enqueue.padded_width = info->padded_width;
		}
		else if ((info->padded_width < info->res.width) && (info->padded_width > 0))
		{
			return IA_CSS_ERR_INVALID_ARGUMENTS;
		}
		// nothing to do if width == padded width or padded width is zeroed (the same)
		break;
	default:
		break;
	}

	return IA_CSS_SUCCESS;
}
#endif
