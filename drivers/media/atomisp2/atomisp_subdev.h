/*
 * Support for Medifield PNW Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 Intel Corporation. All Rights Reserved.
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
#ifndef __ATOMISP_SUBDEV_H__
#define __ATOMISP_SUBDEV_H__

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-core.h>

#include "atomisp_common.h"
#include "atomisp_v4l2.h"
#include "ia_css.h"

enum atomisp_subdev_input_entity {
	ATOMISP_SUBDEV_INPUT_NONE,
	ATOMISP_SUBDEV_INPUT_MEMORY,
	ATOMISP_SUBDEV_INPUT_CSI2,
	/*
	 * The following enum for CSI2 port must go together in one row.
	 * Otherwise it breaks the code logic.
	 */
	ATOMISP_SUBDEV_INPUT_CSI2_PORT1,
	ATOMISP_SUBDEV_INPUT_CSI2_PORT2,
	ATOMISP_SUBDEV_INPUT_CSI2_PORT3,
};

#define ATOMISP_SUBDEV_PAD_SINK			0
/* capture output for still and video frames */
#define ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE	1
/* viewfinder output for downscaled capture output */
#define ATOMISP_SUBDEV_PAD_SOURCE_VF		2
/* preview output for display */
#define ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW	3
#define ATOMISP_SUBDEV_PADS_NUM			4

enum atomisp_pipe_type {
	ATOMISP_PIPE_CAPTURE,
	ATOMISP_PIPE_VIEWFINDER,
	ATOMISP_PIPE_PREVIEW,
	ATOMISP_PIPE_FILEINPUT
};

struct atomisp_in_fmt_conv {
	enum media_bus_format code;
	enum ia_css_stream_format in_sh_fmt;
	enum ia_css_bayer_order bayer_order;
};

struct atomisp_3a_dis_stat_buf {
	struct ia_css_isp_3a_statistics s3a_data;
	struct ia_css_isp_dvs_statistics dvs_stat;
	struct list_head list;
};

struct atomisp_s3a_buf {
	struct ia_css_isp_3a_statistics *s3a_stat;
	struct list_head list;
};

struct atomisp_dvs_buf {
	struct ia_css_isp_dvs_statistics *dvs_stat;
	struct list_head list;
};

struct atomisp_sub_device;

struct atomisp_video_pipe {
	struct video_device vdev;
	enum v4l2_buf_type type;
	struct media_pad pad;
	struct vb2_queue vb2q;
	struct vb2_queue vb2outq; /* Just for file injection */
	struct list_head activeq;
	struct list_head activeq_out;
	unsigned int buffers_in_css;

	/* irq_lock is used to protect video buffer state change operations and
	 * also to make activeq, activeq_out, capq and outq list
	 * operations atomic. */
	spinlock_t irq_lock;
	unsigned int users;
	enum atomisp_pipe_type pipe_type;

	struct atomisp_device *isp;
	struct v4l2_pix_format pix;
	uint32_t sh_fmt;
	struct atomisp_sub_device *isp_subdev;
	int field_sequence;
	int previous_frame_exp_id;
};

struct atomisp_pad_format {
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_rect crop;
	struct v4l2_rect compose;
};

/* Internal states for flash process */
enum atomisp_flash_state {
        ATOMISP_FLASH_IDLE,
        ATOMISP_FLASH_REQUESTED,
        ATOMISP_FLASH_ONGOING,
        ATOMISP_FLASH_DONE
};

enum atomisp_css2_stream_state {
        CSS2_STREAM_UNINIT,
        CSS2_STREAM_CREATED,
        CSS2_STREAM_STARTED,
        CSS2_STREAM_STOPPED,
};

struct atomisp_css_params {
	int online_process;
	int yuv_ds_en;
	unsigned int color_effect;
	bool gdc_cac_en;
	bool macc_en;
	bool bad_pixel_en;
	bool video_dis_en;
	bool sc_en;
	bool fpn_en;
	bool xnr_en;
	bool low_light;
	bool continuous_vf;
	int false_color;
	unsigned int histogram_elenum;
	struct ia_css_isp_config config;

	/* current configurations */
	struct ia_css_dp_config   dp_config;
	struct ia_css_wb_config   wb_config;
	struct ia_css_cc_config   cc_config;
	struct ia_css_nr_config   nr_config;
	struct ia_css_ee_config   ee_config;
	struct ia_css_ob_config   ob_config;
	struct ia_css_de_config   de_config;
	struct ia_css_ce_config   ce_config;
	struct ia_css_gc_config   gc_config;
	struct ia_css_tnr_config  tnr_config;
	struct ia_css_3a_config   s3a_config;
	struct ia_css_macc_config   macc_config;
	struct ia_css_anr_config  anr_config;
	struct ia_css_dz_config   dz_config;  /**< Digital Zoom */
	struct ia_css_capture_config   capture_config;
	struct ia_css_gamma_table gamma_table;
	struct ia_css_ctc_table   ctc_table;
	struct ia_css_macc_table  macc_table;
	struct ia_css_xnr_table   xnr_table;
	struct ia_css_dvs_coefficients dvs_coefs;
	struct ia_css_vector  motion_vector;
	/*
	 * overlay removed from css 1.5
	 * struct ia_css_overlay	*vf_overlay;
	 */

	/* Current grid info */
	struct ia_css_grid_info curr_grid_info;

	/* Intermediate buffers used to communicate data between
	   CSS and user space. These are needed to perform the
	   copy_to_user. */
	struct ia_css_3a_statistics *s3a_user_stat;
	int s3a_output_bytes;
	bool s3a_buf_data_valid;

	struct ia_css_dvs_coefficients *dvs_coeff;
	struct ia_css_dvs_statistics *dvs_stat;
	bool dvs_proj_data_valid;
	int  dvs_hor_coef_bytes;
	int  dvs_ver_coef_bytes;
	int  dvs_ver_proj_bytes;
	int  dvs_hor_proj_bytes;

	/* Flash */
	int num_flash_frames;
	enum atomisp_flash_state flash_state;
	enum atomisp_frame_status last_frame_status;
	/* continuous capture */
	struct atomisp_cont_capture_conf offline_parm;
	/* Flag to check if driver needs to update params to css */
	bool css_update_params_needed;
};

struct atomisp_css2_basis {
	struct ia_css_stream *stream;
	struct ia_css_stream_config stream_config;
	struct ia_css_pipe *pipes[IA_CSS_PIPE_ID_NUM];
	struct ia_css_pipe *multi_pipes[IA_CSS_PIPE_ID_NUM];
	struct ia_css_pipe_config pipe_configs[IA_CSS_PIPE_ID_NUM];
	struct ia_css_pipe_extra_config pipe_extra_configs[IA_CSS_PIPE_ID_NUM];
	bool update_pipe[IA_CSS_PIPE_ID_NUM];
	unsigned int curr_pipe;
	enum atomisp_css2_stream_state stream_state;
};

struct atomisp_sub_device {
	struct v4l2_subdev subdev;
	struct media_pad pads[ATOMISP_SUBDEV_PADS_NUM];
	struct atomisp_pad_format fmt[ATOMISP_SUBDEV_PADS_NUM];
	uint16_t capture_pad; /* main capture pad; defines much of isp config */

	struct atomisp_css_params params;

	enum atomisp_subdev_input_entity input;
	unsigned int output;
	struct atomisp_video_pipe video_in;
	struct atomisp_video_pipe video_out_capture; /* capture output */
	struct atomisp_video_pipe video_out_vf;      /* viewfinder output */
	struct atomisp_video_pipe video_out_preview; /* preview output */
	/* struct isp_subdev_params params; */
	spinlock_t lock;
	struct atomisp_device *isp;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *fmt_auto;
	struct v4l2_ctrl *run_mode;
	struct v4l2_ctrl *enable_vfpp;

	struct atomisp_css2_basis css2_basis;

	struct list_head s3a_stats;
	struct list_head dvs_stats;
        unsigned int s3a_bufs_in_css[IA_CSS_PIPE_ID_NUM];
        unsigned int dis_bufs_in_css;


	struct ia_css_frame *vf_frame; /* TODO: needed? */
	struct ia_css_frame *raw_output_frame;
	enum atomisp_frame_status frame_status[VIDEO_MAX_FRAME];

	int input_curr;

	unsigned int streaming; /* Hold both mutex and lock to change this */
	bool stream_prepared; /* whether css stream is created */

	bool video_pipe_vf_enable; // flag is set when video pipe generates vf frame

	/*
	 * subdev index;
	 * recored in resource(atomisp_input_subdev) to show which sub device
	 * is using this resource
	 */
	int index;

	/*
	 * this is to notify all the buffers are dequeued from CSS.
	 *
	 * css2.0 bug: all the buffers needs to be dequeued after stream off
	 */
	struct completion buf_done;

	 /* delayed memory allocation for css */
        struct completion init_done;
        struct workqueue_struct *delayed_init_workq;
        unsigned int delayed_init;
        struct work_struct delayed_init_work;

        atomic_t sof_count;
        atomic_t sequence;      /* Sequence value that is assigned to buffer. */
        atomic_t sequence_temp;
};

extern const struct atomisp_in_fmt_conv atomisp_in_fmt_conv[];

const struct atomisp_in_fmt_conv *atomisp_find_in_fmt_conv(
	enum media_bus_format code);

/* Get pointer to appropriate format */
struct v4l2_mbus_framefmt
*atomisp_subdev_get_ffmt(struct atomisp_sub_device *isp_sd, struct v4l2_subdev_fh *fh,
			 uint32_t which, uint32_t pad);
struct v4l2_rect *atomisp_subdev_get_rect(struct atomisp_sub_device *isp_sd,
					  struct v4l2_subdev_fh *fh,
					  uint32_t which, uint32_t pad,
					  uint32_t target);
int atomisp_subdev_set_selection(struct atomisp_sub_device *sd,
				 struct v4l2_subdev_fh *fh, uint32_t which,
				 uint32_t pad, uint32_t target, uint32_t flags,
				 struct v4l2_rect *r);
/* Actually set the format */
int atomisp_subdev_set_ffmt(struct atomisp_sub_device *sd, struct v4l2_subdev_fh *fh,
			    uint32_t which, uint32_t pad,
			    struct v4l2_mbus_framefmt *ffmt);

int atomisp_update_run_mode(struct atomisp_sub_device *isp_subdev);

void atomisp_subdev_unregister_entities(struct atomisp_sub_device *isp_subdev);
int atomisp_subdev_register_entities(struct atomisp_sub_device *isp_subdev,
	struct v4l2_device *vdev);
int atomisp_subdev_init(struct atomisp_device *isp);
void atomisp_subdev_cleanup(struct atomisp_sub_device *isp_subdev);

#endif /* __ATOMISP_SUBDEV_H__ */
