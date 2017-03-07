/*
 * Support for Clovertrail PNW Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2012 Intel Corporation. All Rights Reserved.
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

#include "ia_css.h"
#include "atomisp_compat.h"
#include "atomisp_fops.h"
#include "ia_css_types.h"

#include <linux/delay.h>
#include "hrt/hive_isp_css_mm_hrt.h"
#include <linux/pci.h>

/*
 *   to serialize MMIO access , this is due to ISP2400 silicon issue Sighting
 *    #4684168, if concurrency access happened, system may hard hang.
 *    Sighting #4600742 also points to system hard hang upon access to IUNIT MMIO.
 *
 */
static DEFINE_SPINLOCK(mmio_lock);
extern raw_spinlock_t pci_config_lock;

#ifdef ISOF_SILICON_BUG_WORKAROUND
#define MMIO_LOCK spin_lock_irqsave(&mmio_lock, flags);
#define PCI_CONFIG_LOCK raw_spin_lock(&pci_config_lock);
#define MMIO_UNLOCK spin_unlock_irqrestore(&mmio_lock, flags);
#define PCI_CONFIG_UNLOCK raw_spin_unlock(&pci_config_lock);
#else
#define MMIO_LOCK
#define PCI_CONFIG_LOCK
#define MMIO_UNLOCK
#define PCI_CONFIG_UNLOCK
#endif

enum frame_info_type {
	VF_FRAME,
	OUTPUT_FRAME,
	RAW_FRAME,
};

static void atomisp_ISP_parameters_clean_up(struct atomisp_sub_device *isp_subdev,
				     struct ia_css_isp_config *config)
{
	if (config->wb_config) {
		memset(isp_subdev->params.config.wb_config, 0 ,
		       sizeof(struct ia_css_wb_config));
		config->wb_config = NULL;
	}
	if (config->cc_config) {
		memset(isp_subdev->params.config.cc_config, 0 ,
		       sizeof(struct ia_css_cc_config));
		config->cc_config = NULL;
	}
	if (config->tnr_config) {
		memset(isp_subdev->params.config.tnr_config, 0 ,
		       sizeof(struct ia_css_tnr_config));
		config->tnr_config = NULL;
	}
	if (config->ob_config) {
		memset(isp_subdev->params.config.ob_config, 0 ,
		       sizeof(struct ia_css_ob_config));
		config->ob_config = NULL;
	}
	if (config->nr_config) {
		memset(isp_subdev->params.config.nr_config, 0 ,
		       sizeof(struct ia_css_nr_config));
		config->nr_config = NULL;
	}
	if (config->ee_config) {
		memset(isp_subdev->params.config.ee_config, 0 ,
		       sizeof(struct ia_css_ee_config));
		config->ee_config = NULL;
	}
	if (config->de_config) {
		memset(isp_subdev->params.config.de_config, 0 ,
		       sizeof(struct ia_css_de_config));
		config->de_config = NULL;
	}
	if (config->gc_config) {
		memset(isp_subdev->params.config.gc_config, 0 ,
		       sizeof(struct ia_css_gc_config));
		config->gc_config = NULL;
	}
	if (config->ecd_config) {
		memset(isp_subdev->params.config.ecd_config, 0 ,
		       sizeof(struct ia_css_ecd_config));
		config->ecd_config = NULL;
	}
	if (config->ynr_config) {
		memset(isp_subdev->params.config.ynr_config, 0 ,
		       sizeof(struct ia_css_ynr_config));
		config->ynr_config = NULL;
	}
	if (config->fc_config) {
		memset(isp_subdev->params.config.fc_config, 0 ,
		       sizeof(struct ia_css_fc_config));
		config->fc_config = NULL;
	}
	if (config->cnr_config) {
		memset(isp_subdev->params.config.cnr_config, 0 ,
		       sizeof(struct ia_css_cnr_config));
		config->cnr_config = NULL;
	}
	if (config->macc_config) {
		memset(isp_subdev->params.config.macc_config, 0 ,
		       sizeof(struct ia_css_macc_config));
		config->macc_config = NULL;
	}
	if (config->ctc_config) {
		memset(isp_subdev->params.config.ctc_config, 0 ,
		       sizeof(struct ia_css_ctc_config));
		config->ctc_config = NULL;
	}
	if (config->aa_config) {
		memset(isp_subdev->params.config.aa_config, 0 ,
		       sizeof(struct ia_css_aa_config));
		config->aa_config = NULL;
	}
	if (config->ce_config) {
		memset(isp_subdev->params.config.ce_config, 0 ,
		       sizeof(struct ia_css_ce_config));
		config->ce_config = NULL;
	}
	if (config->dvs_6axis_config) {
		config->dvs_6axis_config = NULL;
	}
	if (config->yuv2rgb_cc_config) {
		memset(isp_subdev->params.config.yuv2rgb_cc_config, 0 ,
		       sizeof(struct ia_css_cc_config));
		config->yuv2rgb_cc_config = NULL;
	}
	if (config->rgb2yuv_cc_config) {
		memset(isp_subdev->params.config.rgb2yuv_cc_config, 0 ,
		       sizeof(struct ia_css_cc_config));
		config->rgb2yuv_cc_config = NULL;
	}
	if (config->anr_config) {
		memset(isp_subdev->params.config.anr_config, 0 ,
		       sizeof(struct ia_css_anr_config));
		config->anr_config = NULL;
	}
	if (config->s3a_config) {
		memset(isp_subdev->params.config.s3a_config, 0 ,
		       sizeof(struct ia_css_3a_config));
		config->s3a_config = NULL;
	}
	if (config->macc_table) {
		memset(isp_subdev->params.config.macc_table, 0 ,
		       sizeof(struct ia_css_macc_table));
		config->macc_table = NULL;
	}
	if (config->gamma_table) {
		memset(isp_subdev->params.config.gamma_table, 0 ,
		       sizeof(struct ia_css_gamma_table));
		config->gamma_table = NULL;
	}
	if (config->ctc_table) {
		memset(isp_subdev->params.config.ctc_table, 0 ,
		       sizeof(struct ia_css_ctc_table));
		config->ctc_table = NULL;
	}
	if (config->xnr_table) {
		memset(isp_subdev->params.config.xnr_table, 0 ,
		       sizeof(struct ia_css_xnr_table));
		config->xnr_table = NULL;
	}
	if (config->r_gamma_table) {
		memset(isp_subdev->params.config.r_gamma_table, 0 ,
		       sizeof(struct ia_css_rgb_gamma_table));
		config->r_gamma_table = NULL;
	}
	if (config->g_gamma_table) {
		memset(isp_subdev->params.config.g_gamma_table, 0 ,
		       sizeof(struct ia_css_rgb_gamma_table));
		config->g_gamma_table = NULL;
	}
	if (config->b_gamma_table) {
		memset(isp_subdev->params.config.b_gamma_table, 0 ,
		       sizeof(struct ia_css_rgb_gamma_table));
		config->b_gamma_table = NULL;
	}
	if (config->dz_config) {
		memset(isp_subdev->params.config.dz_config, 0 ,
		       sizeof(struct ia_css_dz_config));
		config->dz_config = NULL;
	}
	if (config->motion_vector) {
		memset(isp_subdev->params.config.motion_vector, 0 ,
		       sizeof(struct ia_css_vector));
		config->motion_vector = NULL;
	}
	if (config->shading_table) {
		ia_css_shading_table_free(config->shading_table);
		config->shading_table = NULL;
	}
	if (config->morph_table) {
		ia_css_morph_table_free(config->morph_table);
		config->morph_table = NULL;
	}
	if (config->dvs_coefs) {
		config->dvs_coefs = NULL;
	}
}

static inline
enum ia_css_pipe_mode __pipe_id_to_pipe_mode(enum ia_css_pipe_id pipe_id)
{
	switch (pipe_id) {
	case IA_CSS_PIPE_ID_PREVIEW:
		return IA_CSS_PIPE_MODE_PREVIEW;
	case IA_CSS_PIPE_ID_CAPTURE:
		return IA_CSS_PIPE_MODE_CAPTURE;
	case IA_CSS_PIPE_ID_VIDEO:
		return IA_CSS_PIPE_MODE_VIDEO;
	case IA_CSS_PIPE_ID_ACC:
		return IA_CSS_PIPE_MODE_ACC;
	default:
		return IA_CSS_PIPE_MODE_NUM;
	}

}
static void __apply_additional_pipe_config(struct atomisp_sub_device *isp_subdev)
{
	int i = 0;
	for (i = 0; i< IA_CSS_PIPE_ID_NUM; i++) {
		isp_subdev->css2_basis.pipe_configs[i].isp_pipe_version = 2;

		switch(i) {
		  case IA_CSS_PIPE_ID_CAPTURE:
		       break;
		  case IA_CSS_PIPE_ID_VIDEO:
			  //In order to use isp2_dz_min set enable_reduce_pipe
		      isp_subdev->css2_basis.pipe_extra_configs[i].enable_reduced_pipe = true;

		      //In order to load isp_dz binary set enable_dz
		      isp_subdev->css2_basis.pipe_configs[i].enable_dz = false;

		      if(isp_subdev->params.video_dis_en){
		           isp_subdev->css2_basis.pipe_extra_configs[i].enable_dvs_6axis = true;
		           isp_subdev->css2_basis.pipe_configs[i].dvs_frame_delay = 2;
		      }
		       break;
		  case IA_CSS_PIPE_ID_PREVIEW:
		  case IA_CSS_PIPE_ID_COPY:
		  case IA_CSS_PIPE_ID_ACC:
		       break;
		  default:
		      break;
		  }
	}

	}

static void __configure_output(struct atomisp_sub_device *isp_subdev,
			       unsigned int width,
			       unsigned int height,
			       enum ia_css_frame_format format,
			       enum ia_css_pipe_id pipe_id)
{

	isp_subdev->css2_basis.curr_pipe = pipe_id;
	isp_subdev->css2_basis.pipe_configs[pipe_id].mode = __pipe_id_to_pipe_mode(pipe_id);
	isp_subdev->css2_basis.update_pipe[pipe_id] = true;

	isp_subdev->css2_basis.pipe_configs[pipe_id].output_info.res.width =
	    width;
	isp_subdev->css2_basis.pipe_configs[pipe_id].output_info.res.height =
	    height;
	isp_subdev->css2_basis.pipe_configs[pipe_id].output_info.format =
	    format;
	if (width * height > isp_subdev->css2_basis.stream_config.effective_res.width *
	    isp_subdev->css2_basis.stream_config.effective_res.height) {
		isp_subdev->css2_basis.stream_config.effective_res.width = width;
		isp_subdev->css2_basis.stream_config.effective_res.height = height;
	}

	v4l2_dbg(3, dbg_level, &atomisp_dev,
		 "configuring pipe[%d] output info w=%d.h=%d.f=%d.\n",
		 pipe_id, width, height, format);
}
static void __configure_pp_input(struct atomisp_sub_device *isp_subdev,
				 unsigned int width,
				 unsigned int height,
				 enum ia_css_pipe_id pipe_id)
{
	if (width == 0 && height == 0)
		return;

	isp_subdev->css2_basis.curr_pipe = pipe_id;
	isp_subdev->css2_basis.pipe_configs[pipe_id].mode = __pipe_id_to_pipe_mode(pipe_id);
	isp_subdev->css2_basis.update_pipe[pipe_id] = true;

	if (width <=
	    isp_subdev->css2_basis.pipe_configs[pipe_id].output_info.res.width ||
	    height <=
	    isp_subdev->css2_basis.pipe_configs[pipe_id].output_info.res.height
	   )
		return;
	isp_subdev->css2_basis.pipe_extra_configs[pipe_id].enable_yuv_ds = true;
	isp_subdev->css2_basis.pipe_configs[pipe_id].bayer_ds_out_res.width = width;
	isp_subdev->css2_basis.pipe_configs[pipe_id].bayer_ds_out_res.height = height;
	v4l2_dbg(3, dbg_level, &atomisp_dev,
		 "configuring pipe[%d]capture pp input w=%d.h=%d.\n",
		 pipe_id, width, height);
}
static void __configure_vf_output(struct atomisp_sub_device *isp_subdev,
				  unsigned int width,
				  unsigned int height,
				  enum ia_css_frame_format format,
				  enum ia_css_pipe_id pipe_id)
{

	//Setting VF width to zero disable VF interrupts for the particular pipe
	if(!isp_subdev->video_pipe_vf_enable)
		width = 0;

	isp_subdev->css2_basis.curr_pipe = pipe_id;
	isp_subdev->css2_basis.pipe_configs[pipe_id].mode = __pipe_id_to_pipe_mode(pipe_id);
	isp_subdev->css2_basis.update_pipe[pipe_id] = true;

	isp_subdev->css2_basis.pipe_configs[pipe_id].vf_output_info.res.width = width;
	isp_subdev->css2_basis.pipe_configs[pipe_id].vf_output_info.res.height = height;
	isp_subdev->css2_basis.pipe_configs[pipe_id].vf_output_info.format = format;
	v4l2_dbg(3, dbg_level, &atomisp_dev,
		 "configuring pipe[%d] vf output info w=%d.h=%d.f=%d.\n",
		 pipe_id, width, height, format);
}

enum ia_css_err __destroy_pipes(struct atomisp_sub_device *isp_subdev,
                                       bool force)
{
	int i;
	enum ia_css_err ret = IA_CSS_SUCCESS;
	struct atomisp_device *isp = isp_subdev->isp;

	if (isp_subdev->css2_basis.stream) {
		dev_dbg(isp->dev, "destroy css stream first.\n");
		return IA_CSS_ERR_INTERNAL_ERROR;
	}

	for (i = 0; i < IA_CSS_PIPE_ID_NUM; i++) {
		if (isp_subdev->css2_basis.pipes[i]
		    		&& (force || isp_subdev->css2_basis.update_pipe[i])) {
			ret |= ia_css_pipe_destroy(isp_subdev->css2_basis.pipes[i]);
			if (ret) {
				v4l2_err(&atomisp_dev,
					 "destroy pipe[%d]failed.\
					 cannot recover\n", i);
			}
			isp_subdev->css2_basis.pipes[i] = NULL;
			isp_subdev->css2_basis.update_pipe[i] = false;
		}
	}
	return ret;
}

static enum ia_css_err __create_pipe(struct atomisp_sub_device *isp_subdev)
{

	int i, j;
	enum ia_css_err ret;
	struct ia_css_pipe_extra_config extra_config;

	v4l2_dbg(5, dbg_level, &atomisp_dev, ">%s\n", __func__);
	__apply_additional_pipe_config(isp_subdev);
	ia_css_pipe_extra_config_defaults(&extra_config);
	for (i = 0; i < IA_CSS_PIPE_ID_NUM; i++) {
		if (isp_subdev->css2_basis.pipe_configs[i].output_info.res.width) {
			if (!memcmp(&extra_config,
				    &isp_subdev->css2_basis.pipe_extra_configs[i],
				    sizeof(extra_config)))
				ret = ia_css_pipe_create(
					&isp_subdev->css2_basis.pipe_configs[i],
					&isp_subdev->css2_basis.pipes[i]);
			else
				ret = ia_css_pipe_create_extra(
					&isp_subdev->css2_basis.pipe_configs[i],
					&isp_subdev->css2_basis.pipe_extra_configs[i],
					&isp_subdev->css2_basis.pipes[i]);
			if (ret) {
				v4l2_err(&atomisp_dev, "create pipe[%d] error.\n", i);
				goto pipe_err;
	}
	v4l2_dbg(3, dbg_level, &atomisp_dev,
				 "dump pipe[%d] info w=%d, h=%d,f=%d vf_w=%d vf_h=%d vf_f=%d.\n",
				 i,
				 isp_subdev->css2_basis.pipe_configs[i].output_info.res.width,
				 isp_subdev->css2_basis.pipe_configs[i].output_info.res.height,
				 isp_subdev->css2_basis.pipe_configs[i].output_info.format,
				 isp_subdev->css2_basis.pipe_configs[i].vf_output_info.res.width,
				 isp_subdev->css2_basis.pipe_configs[i].vf_output_info.res.height,
				 isp_subdev->css2_basis.pipe_configs[i].vf_output_info.format);
		}
	}

	return IA_CSS_SUCCESS;
pipe_err:
	for (j = i; j >= 0; j--)
		if (isp_subdev->css2_basis.pipes[j]) {
			ia_css_pipe_destroy(isp_subdev->css2_basis.pipes[j]);
			isp_subdev->css2_basis.pipes[j] = NULL;
}

	return ret;
}
static void dump_stream_pipe_config(struct atomisp_sub_device *isp_subdev)
{
	struct ia_css_pipe_config *p_config;
	struct ia_css_pipe_extra_config *pe_config;
	struct ia_css_stream_config *s_config;
	int i = 0;

	for (i = 0; i < IA_CSS_PIPE_ID_NUM; i++) {
		if (isp_subdev->css2_basis.pipes[i]) {
			p_config = &isp_subdev->css2_basis.pipe_configs[i];
			pe_config =
			    &isp_subdev->css2_basis.pipe_extra_configs[i];
			v4l2_dbg(3, dbg_level, &isp_subdev->subdev,
				 "dumping pipe[%d] config:\n"
				 "pipe_config.pipe_id:%d.\n"
				 "pipe_config.output_info w=%d, h=%d.\n"
				 "pipe_config.vf_pp_in_res w=%d, h=%d.\n"
				 "pipe_config.capt_pp_in_res w=%d, h=%d.\n"
				 "pipe_config.vf_output_info w=%d, h=%d.\n"
				 "pipe_config.bayer_ds_out_res w=%d, h=%d.\n"
				 "pipe_config.envelope w=%d, h=%d.\n"
				 "pipe_config.default_capture_config.capture_mode=%d.\n"
                 		 "pipe_config.default_capture_config..enable_xnr=%d.\n"
				 "pipe_config.enable_dz:%d.\n"
				 "pipe_config.isp_pipe_version:%d.\n"
				 "dumping pipe[%d] extra config:\n"
				 "pipe_extra_config.enable_raw_binning:%d.\n"
				 "pipe_extra_config.enable_yuv_ds:%d.\n"
				 "pipe_extra_config.enable_high_speed:%d.\n"
				 "pipe_extra_config.enable_dvs_6axis:%d.\n"
				 "pipe_extra_config.enable_reduced_pipe:%d.\n"
				 "pipe_extra_config.disable_vf_pp:%d.\n",
				 i,
				 p_config->mode,
				 p_config->output_info.res.width,
				 p_config->output_info.res.height,
				 p_config->vf_pp_in_res.width,
                 		 p_config->vf_pp_in_res.height,
                 		 p_config->capt_pp_in_res.width,
                 		 p_config->capt_pp_in_res.height,
				 p_config->vf_output_info.res.width,
				 p_config->vf_output_info.res.height,
				 p_config->bayer_ds_out_res.width,
				 p_config->bayer_ds_out_res.height,
				 p_config->dvs_envelope.width,
				 p_config->dvs_envelope.height,
				 p_config->default_capture_config.mode,
				 p_config->default_capture_config.enable_xnr,
                 		 p_config->enable_dz,
                 		 p_config->isp_pipe_version,
				 i,
				 pe_config->enable_raw_binning,
				 pe_config->enable_yuv_ds,
				 pe_config->enable_high_speed,
				 pe_config->enable_dvs_6axis,
				 pe_config->enable_reduced_pipe,
				 pe_config->disable_vf_pp);
		}
	}

	s_config = &isp_subdev->css2_basis.stream_config;
	v4l2_dbg(3, dbg_level, &isp_subdev->subdev,
		 "dumping stream config:\n"
		 "stream_config.mode=%d.\n"
		 "stream_config.input_res w=%d, h=%d.\n"
		 "stream_config.effective_res w=%d, h=%d.\n"
		 "stream_config.format=%d.\n"
		 "stream_config.bayer_order=%d.\n"
		 "stream_config.2ppc=%d.\n"
		 "stream_config.online=%d.\n"
         	 "stream_config.continuous=%d.\n"
		 "stream_config.channel_id=%d.\n"
		 "stream_config.init_num_cont_raw_buf=%d\n"
		 "stream_config.left_padding=%d\n",
		 s_config->mode,
		 s_config->input_res.width, s_config->input_res.height,
		 s_config->effective_res.width, s_config->effective_res.height,
		 s_config->format,
		 s_config->bayer_order,
		 s_config->two_pixels_per_clock,
		 s_config->online, s_config->continuous,
		 s_config->channel_id,
		 s_config->init_num_cont_raw_buf,
		 s_config->left_padding);
}

void atomisp_css_update_isp_params(struct atomisp_sub_device *isp_subdev)
{
	ia_css_stream_set_isp_config(
			isp_subdev->css2_basis.stream,
			&isp_subdev->params.config);
	atomisp_ISP_parameters_clean_up(isp_subdev, &isp_subdev->params.config);
}

enum ia_css_err __destroy_stream(struct atomisp_sub_device *isp_subdev,
                                        bool force)
{
	int i;
	bool pipe_updated = false;

	if (!isp_subdev->css2_basis.stream)
		return IA_CSS_SUCCESS;

	if (!force) {
		for (i = 0; i < IA_CSS_PIPE_ID_NUM; i++)
			if (isp_subdev->css2_basis.update_pipe[i]) {
				pipe_updated = true;
 				break;
			}
	}

	if (!(force || pipe_updated))
		return IA_CSS_SUCCESS;

	if (isp_subdev->css2_basis.stream_state == CSS2_STREAM_STARTED
	    && ia_css_stream_stop(isp_subdev->css2_basis.stream) != IA_CSS_SUCCESS) {
		dev_err(isp_subdev->isp->dev, "stop stream failed.\n");
		return -EINVAL;
	}
	isp_subdev->css2_basis.stream_state = CSS2_STREAM_STOPPED;

	if (ia_css_stream_destroy(isp_subdev->css2_basis.stream) != IA_CSS_SUCCESS) {
		dev_err(isp_subdev->isp->dev, "destroy stream failed.\n");
		return -EINVAL;
	}

	isp_subdev->css2_basis.stream_state = CSS2_STREAM_UNINIT;
	isp_subdev->css2_basis.stream = NULL;
    	isp_subdev->stream_prepared = false;

	return IA_CSS_SUCCESS;
}

static enum ia_css_err __create_stream(struct atomisp_sub_device *isp_subdev)
{
	int pipe_index = 0, i;
	struct ia_css_pipe *multi_pipes[IA_CSS_PIPE_ID_NUM];
	const struct ia_css_stream_config *s_config =
	    		&isp_subdev->css2_basis.stream_config;
	enum ia_css_err ret;

	v4l2_dbg(3, dbg_level, &atomisp_dev,
		 ">%s.\n", __func__);
	for (i = 0; i < IA_CSS_PIPE_ID_NUM; i++) {
		if (isp_subdev->css2_basis.pipes[i])
			multi_pipes[pipe_index++] = isp_subdev->css2_basis.pipes[i];
	}

	ia_css_input_set_mode(isp_subdev,
			get_input_mode(isp_subdev->isp->inputs[isp_subdev->input_curr].type, isp_subdev->isp));

	v4l2_dbg(3, dbg_level, &atomisp_dev,
		 "<%s.\n", __func__);
	dump_stream_pipe_config(isp_subdev);
	ret = ia_css_stream_create(s_config, pipe_index, multi_pipes,
				    &isp_subdev->css2_basis.stream);
	isp_subdev->stream_prepared = true;
	return ret;
}

void atomisp_destroy_pipes_stream_force(struct atomisp_sub_device *isp_subdev)
{
	__destroy_stream(isp_subdev, true);
	__destroy_pipes(isp_subdev, true);
}

void
ia_css_input_set_mode(struct atomisp_sub_device *isp_subdev,
		      enum ia_css_input_mode mode)
{
	v4l2_dbg(6, dbg_level, &atomisp_dev, "ENTER %s with mode set as=%d \n",__func__,mode);
	isp_subdev->css2_basis.stream_config.mode = mode;
	if (mode == IA_CSS_INPUT_MODE_BUFFERED_SENSOR) {
			ia_css_mipi_frame_specify(ATOMISP_MIPI_BUFFER_SIZE, false);
	}
	v4l2_dbg(6, dbg_level, &atomisp_dev, "EXIT %s \n",__func__);
}

enum ia_css_input_mode get_input_mode(unsigned int input_type, struct atomisp_device *isp)
{
	if (input_type != TEST_PATTERN && input_type != FILE_INPUT){
		if (fastboot){
 	               if(isp->firmware_switched == false){
	                       return multistream_enabled ? IA_CSS_INPUT_MODE_BUFFERED_SENSOR : IA_CSS_INPUT_MODE_SENSOR;
                       }
        	}
		return IA_CSS_INPUT_MODE_BUFFERED_SENSOR;
	}
	else if(input_type == FILE_INPUT)
		return IA_CSS_INPUT_MODE_FIFO;
	else if(input_type == TEST_PATTERN)
		return IA_CSS_INPUT_MODE_TPG;
	return -1;
}

enum ia_css_err ia_css_update_stream(struct atomisp_sub_device *isp_subdev)
{
	enum ia_css_err ret;

	if (__destroy_stream(isp_subdev, true) != IA_CSS_SUCCESS)
		dev_warn(isp_subdev->isp->dev, "destroy stream failed.\n");

	if (__destroy_pipes(isp_subdev, true) != IA_CSS_SUCCESS)
		dev_warn(isp_subdev->isp->dev, "destroy pipe failed.\n");

	ret = __create_pipe(isp_subdev);
	if (ret != IA_CSS_SUCCESS) {
		dev_err(isp_subdev->isp->dev, "create pipe failed.\n");
		return ret;
	}

	ret = __create_stream(isp_subdev);
	if (ret != IA_CSS_SUCCESS) {
		dev_warn(isp_subdev->isp->dev, "create stream failed.\n");
		__destroy_pipes(isp_subdev, true);
		return ret;
	}

	return ret;
}

static enum ia_css_err __get_frame_info(struct atomisp_sub_device *isp_subdev,
				struct ia_css_frame_info *info,
				enum frame_info_type type)
{
	enum ia_css_err ret;
	struct ia_css_pipe_info p_info;
	unsigned int pipe_id = isp_subdev->css2_basis.curr_pipe;
	struct atomisp_device *isp = isp_subdev->isp;

	v4l2_dbg(3, dbg_level, &atomisp_dev,
		 ">%s.\n", __func__);

	if (__destroy_stream(isp_subdev, true) != IA_CSS_SUCCESS)
		dev_warn(isp->dev, "destroy stream failed.\n");

	if (__destroy_pipes(isp_subdev, true) != IA_CSS_SUCCESS)
		dev_warn(isp->dev, "destroy pipe failed.\n");

	if((ret = __create_pipe(isp_subdev)) != IA_CSS_SUCCESS)
		goto pipe_err;

	if((ret = __create_stream(isp_subdev)) != IA_CSS_SUCCESS)
		goto stream_err;

	ret = ia_css_pipe_get_info(
			isp_subdev->css2_basis.pipes[pipe_id], &p_info);
	if (ret == IA_CSS_SUCCESS) {
		switch (type) {
		case VF_FRAME:
			*info = p_info.vf_output_info;
			v4l2_dbg(3, dbg_level, &atomisp_dev,
				 "getting vf frame info.\n");
			break;
		case OUTPUT_FRAME:
			*info = p_info.output_info;
			v4l2_dbg(3, dbg_level, &atomisp_dev,
				 "getting main frame info.\n");
			break;
		case RAW_FRAME:
			*info = p_info.raw_output_info;
			v4l2_dbg(3, dbg_level, &atomisp_dev,
				 "getting raw frame info.\n");
			break;
		default:
			info = NULL;
			v4l2_err(&atomisp_dev,
				  "wrong type for getting frame info");
		}
		v4l2_dbg(3, dbg_level, &atomisp_dev,
			 "get frame info: w=%d, h=%d.\n",
			 	info->res.width, info->res.height);
		v4l2_dbg(3, dbg_level, &atomisp_dev,
			 "<%s.\n", __func__);
		return IA_CSS_SUCCESS;
	}

stream_err:
	__destroy_pipes(isp_subdev, true);
pipe_err:

	return ret;
}

enum ia_css_err ia_css_preview_configure_output(
	struct atomisp_sub_device *isp_subdev,
	unsigned int width,
	unsigned int height,
	enum ia_css_frame_format format)
{
	__configure_output(isp_subdev, width, height, format, IA_CSS_PIPE_ID_PREVIEW);
	return IA_CSS_SUCCESS;
}

enum ia_css_err ia_css_preview_get_output_frame_info(
	struct atomisp_sub_device *isp_subdev,
	struct ia_css_frame_info *info)
{
	isp_subdev->css2_basis.curr_pipe = IA_CSS_PIPE_ID_PREVIEW;
	return __get_frame_info(isp_subdev, info, OUTPUT_FRAME);
}

enum ia_css_err ia_css_capture_configure_output(
	struct atomisp_sub_device *isp_subdev,
	unsigned int width,
	unsigned int height,
	enum ia_css_frame_format format)
{
	__configure_output(isp_subdev, width, height, format,
			   IA_CSS_PIPE_ID_CAPTURE);
	return IA_CSS_SUCCESS;
}

enum ia_css_err ia_css_capture_get_output_frame_info(
	struct atomisp_sub_device *isp_subdev,
	struct ia_css_frame_info *info)
{
	isp_subdev->css2_basis.curr_pipe = IA_CSS_PIPE_ID_CAPTURE;
	return __get_frame_info(isp_subdev, info, OUTPUT_FRAME);
}

enum ia_css_err ia_css_capture_configure_viewfinder(
	struct atomisp_sub_device *isp_subdev,
	unsigned int width,
	unsigned int height,
	enum ia_css_frame_format format)
{
	__configure_vf_output(isp_subdev, width, height, format, IA_CSS_PIPE_ID_CAPTURE);
	return IA_CSS_SUCCESS;
}

enum ia_css_err ia_css_capture_get_viewfinder_frame_info(
	struct atomisp_sub_device *isp_subdev,
	struct ia_css_frame_info *info)
{
	isp_subdev->css2_basis.curr_pipe = IA_CSS_PIPE_ID_CAPTURE;
	__get_frame_info(isp_subdev, info, VF_FRAME);
	return IA_CSS_SUCCESS;
}

enum ia_css_err ia_css_video_configure_output(
	struct atomisp_sub_device *isp_subdev,
	unsigned int width,
	unsigned int height,
	enum ia_css_frame_format format)
{
	__configure_output(isp_subdev, width, height, format,
			   IA_CSS_PIPE_ID_VIDEO);

	return IA_CSS_SUCCESS;
}

enum ia_css_err ia_css_video_get_output_frame_info(
	struct atomisp_sub_device *isp_subdev,
	struct ia_css_frame_info *info)
{
	isp_subdev->css2_basis.curr_pipe = IA_CSS_PIPE_ID_VIDEO;
	return __get_frame_info(isp_subdev, info, OUTPUT_FRAME);
}

enum ia_css_err ia_css_video_configure_viewfinder(
	struct atomisp_sub_device *isp_subdev,
	unsigned int width,
	unsigned int height,
	enum ia_css_frame_format format)
{
	__configure_vf_output(isp_subdev, width, height, format, IA_CSS_PIPE_ID_VIDEO);
	return IA_CSS_SUCCESS;
}

enum ia_css_err ia_css_video_get_viewfinder_frame_info(
	struct atomisp_sub_device *isp_subdev,
	struct ia_css_frame_info *info)
{
	isp_subdev->css2_basis.curr_pipe = IA_CSS_PIPE_ID_VIDEO;
	return __get_frame_info(isp_subdev, info, VF_FRAME);
}

enum ia_css_err ia_css_preview_configure_pp_input(
	struct atomisp_sub_device *isp_subdev,
	unsigned int width,
	unsigned int height)
{
		__configure_pp_input(isp_subdev, width, height, IA_CSS_PIPE_ID_PREVIEW);

	return IA_CSS_SUCCESS;
}

enum ia_css_err ia_css_capture_configure_pp_input(
	struct atomisp_sub_device *isp_subdev,
	unsigned int width,
	unsigned int height)
{
	__configure_pp_input(isp_subdev, width, height, IA_CSS_PIPE_ID_CAPTURE);
	return IA_CSS_SUCCESS;
}
void
ia_css_capture_set_mode(struct atomisp_sub_device *isp_subdev,
			enum ia_css_capture_mode mode)
{
	isp_subdev->css2_basis.curr_pipe = IA_CSS_PIPE_ID_CAPTURE;
	if (isp_subdev->css2_basis.pipe_configs[IA_CSS_PIPE_ID_CAPTURE].default_capture_config.mode != mode) {
		isp_subdev->css2_basis.pipe_configs[IA_CSS_PIPE_ID_CAPTURE].default_capture_config.mode = mode;
		isp_subdev->css2_basis.update_pipe[IA_CSS_PIPE_ID_CAPTURE] = true;
	}
}

void
ia_css_capture_enable_online(struct atomisp_sub_device *isp_subdev,
			     bool enable)
{
	isp_subdev->css2_basis.curr_pipe = IA_CSS_PIPE_ID_CAPTURE;
	if (isp_subdev->css2_basis.stream_config.online != enable) {
		isp_subdev->css2_basis.stream_config.online = enable;
		isp_subdev->css2_basis.update_pipe[IA_CSS_PIPE_ID_CAPTURE] =
		    true;
	}
}

void
ia_css_input_set_two_pixels_per_clock(struct atomisp_sub_device *isp_subdev,
					   bool enable)
{
	int i;

	if (isp_subdev->css2_basis.stream_config.two_pixels_per_clock !=
	    enable) {
		isp_subdev->css2_basis.stream_config.two_pixels_per_clock =
		    enable;
		for (i = 0; i < IA_CSS_PIPE_ID_NUM; i++)
			isp_subdev->css2_basis.update_pipe[i] = true;
	}
}
void
ia_css_enable_dz(struct atomisp_sub_device *isp_subdev, bool enable)
{
 int i;
 for (i = 0; i < IA_CSS_PIPE_ID_NUM; i++)
 {
	 isp_subdev->css2_basis.pipe_configs[i].enable_dz = enable;
 }
}
void
ia_css_input_set_resolution(struct atomisp_sub_device *isp_subdev,struct v4l2_mbus_framefmt *ffmt){
       int pixel_pad_for_raw = 0;

       if(isp_subdev->css2_basis.stream_config.format == IA_CSS_STREAM_FORMAT_RAW_10)
               pixel_pad_for_raw = 12;
       else
               pixel_pad_for_raw = 0;
       isp_subdev->css2_basis.stream_config.input_res.width = ffmt->width + pixel_pad_for_raw;
       isp_subdev->css2_basis.stream_config.input_res.height = ffmt->height + pixel_pad_for_raw;

}
void
ia_css_input_set_format(struct atomisp_sub_device *isp_subdev,enum ia_css_stream_format format){
       isp_subdev->css2_basis.stream_config.format = format;
}
void
ia_css_input_set_binning_factor(struct atomisp_sub_device *isp_subdev, unsigned int binning_factor){

       isp_subdev->css2_basis.stream_config.sensor_binning_factor =  binning_factor;
}
void
ia_css_input_set_bayer_order(struct atomisp_sub_device *isp_subdev, enum ia_css_bayer_order bayer_order){

       isp_subdev->css2_basis.stream_config.bayer_order = bayer_order;
}
void
ia_css_video_set_dis_envelope(struct atomisp_sub_device *isp_subdev,
                                       unsigned int dvs_w, unsigned int dvs_h)
{
       isp_subdev->css2_basis.pipe_configs[IA_CSS_PIPE_ID_VIDEO].dvs_envelope.width = dvs_w;
       isp_subdev->css2_basis.pipe_configs[IA_CSS_PIPE_ID_VIDEO].dvs_envelope.height  = dvs_h;
}

void
ia_css_enable_raw_binning(struct atomisp_sub_device *isp_subdev,
			     bool enable)
{
	int i;

	for(i=0;i<IA_CSS_PIPE_ID_NUM;i++) {
		if(isp_subdev->css2_basis.pipe_extra_configs[i].enable_raw_binning != enable) {
			isp_subdev->css2_basis.pipe_extra_configs[i].enable_raw_binning = enable;
			isp_subdev->css2_basis.update_pipe[i] = true;
		}
	}
}

void ia_css_enable_continuous(struct atomisp_sub_device *isp_subdev,
				  bool enable)
{
	int i;

	if (isp_subdev->css2_basis.stream_config.continuous != enable) {
		isp_subdev->css2_basis.stream_config.continuous = enable;
		for (i = 0; i < IA_CSS_PIPE_ID_NUM; i++)
			isp_subdev->css2_basis.update_pipe[i] = true;
	}
}

void ia_css_preview_enable_online(struct atomisp_sub_device *isp_subdev,
				  bool enable)
{
	int i;

	if (isp_subdev->css2_basis.stream_config.online != enable) {
		isp_subdev->css2_basis.stream_config.online = enable;
		for (i = 0; i < IA_CSS_PIPE_ID_NUM; i++)
			isp_subdev->css2_basis.update_pipe[i] = true;
	}
}

enum ia_css_err ia_css_capture_get_output_raw_frame_info(
	struct atomisp_sub_device *isp_subdev,
	struct ia_css_frame_info *info)
{
	isp_subdev->css2_basis.curr_pipe = IA_CSS_PIPE_ID_CAPTURE;
	return __get_frame_info(isp_subdev, info, RAW_FRAME);
}

int atomisp_css_init(struct atomisp_device *isp)
{
        int i, retval, ret = 0;

	v4l2_dbg(6, dbg_level, &atomisp_dev, ">%s.\n", __func__);
        for(i = 0; i <  MAX_LOAD_FIRMWARE_TIMEOUTS; i++)
        {
                retval = wait_event_interruptible_timeout(atomisp_wait_queue, isp->firmware_load_complete == true, 250);
                if(retval>=0 && isp->firmware_load_complete)
                        break;
        }

        if(isp->firmware == NULL)
        {
                ret = -EINVAL;
                goto css_init_failed;
        }

        if (ia_css_init(&css_env,
                        &isp->css_fw,
                        (uint32_t)isp->mmu_base_addr,
                        IA_CSS_IRQ_TYPE_PULSE)) {
                ret = -EINVAL;
                goto css_init_failed;
        }
	v4l2_dbg(6, dbg_level, &atomisp_dev, "<%s.\n", __func__);

css_init_failed:
        return ret;
}

void atomisp_css_uninit(struct atomisp_device *isp)
{
	struct atomisp_sub_device *isp_subdev;
	unsigned int i;

	v4l2_dbg(6, dbg_level, &atomisp_dev, ">%s.\n", __func__);
	for (i = 0; i < isp->num_of_streams; i++) {
		isp_subdev = &isp->isp_subdev[i];
		atomisp_ISP_parameters_clean_up(isp_subdev, &isp_subdev->params.config);
		isp_subdev->params.css_update_params_needed = false;
	}

	ia_css_uninit();
	v4l2_dbg(6, dbg_level, &atomisp_dev, "<%s.\n", __func__);
}

enum ia_css_err ia_css_start(struct atomisp_sub_device *isp_subdev, bool in_reset)
{
	enum ia_css_err ret;
	struct atomisp_device *isp = isp_subdev->isp;

	if (in_reset) {
		if (__destroy_stream(isp_subdev, true) != IA_CSS_SUCCESS)
			dev_warn(isp->dev, "destroy stream failed.\n");

		if (__destroy_pipes(isp_subdev, true) != IA_CSS_SUCCESS)
			dev_warn(isp->dev, "destroy pipe failed.\n");

		if ((ret = __create_pipe(isp_subdev)) != IA_CSS_SUCCESS) {
			v4l2_err(&atomisp_dev, "create pipe error.\n");
			goto pipe_err;
		}
		if ((ret = __create_stream(isp_subdev)) != IA_CSS_SUCCESS) {
			v4l2_err(&atomisp_dev, "create stream error.\n");
			goto stream_err;
		}
	}

	/*
	 * For dual steam case, it is possible that:
	 * 1: for this stream, it is at the stage that:
	 * - after set_fmt is called
	 * - before stream on is called
	 * 2: for the other stream, the stream off is called which css reset
	 * has been done.
	 *
	 * Thus the stream created in set_fmt get destroyed and need to be
	 * recreated in the next stream on.
	 */
	if (isp_subdev->stream_prepared == false) {
		if (__create_pipe(isp_subdev)) {
			dev_err(isp->dev, "create pipe error.\n");
			return -EINVAL;
		}
		if (__create_stream(isp_subdev)) {
			dev_err(isp->dev, "create stream error.\n");
			ret = -EINVAL;
			goto stream_err;
		}
	}

	/*
	 * SP can only be started one time
	 * if atomisp_subdev_streaming_count() tell there aleady has some subdev
	 * at streamming, then SP should already be started previously, so
	 * need to skip start sp procedure
	 */
	if (atomisp_subdev_streaming_count(isp_subdev->isp)) {
		dev_dbg(isp_subdev->isp->dev, "skip start sp.\n");
	} else {
		ret = ia_css_start_sp();
		if (ret != IA_CSS_SUCCESS) {
			dev_err(isp_subdev->isp->dev, "start sp error.\n");
			goto start_err;
		}
 	}

	if ((ret = ia_css_stream_start(isp_subdev->css2_basis.stream)) !=
	    				IA_CSS_SUCCESS) {
		v4l2_err(&atomisp_dev, "stream start error.\n");
		goto start_err;
	}

	isp_subdev->css2_basis.stream_state = CSS2_STREAM_STARTED;
	return IA_CSS_SUCCESS;

start_err:
	__destroy_stream(isp_subdev, true);
stream_err:
	__destroy_pipes(isp_subdev, true);

	/* css 2.0 API limitation: ia_css_stop_sp() could be only called after
	 * destroy all pipes
	 * */
	if (atomisp_subdev_streaming_count(isp_subdev->isp)){
			v4l2_dbg(3, dbg_level, &atomisp_dev, "can not stop sp.\n");
	} else if(ia_css_isp_has_started())
		if(ia_css_stop_sp() != IA_CSS_SUCCESS)
			v4l2_warn(&atomisp_dev, "stop sp failed\n");
pipe_err:
	return ret;
}


enum ia_css_err ia_css_stop(struct atomisp_sub_device *isp_subdev, bool in_reset)
{
	int i = 0;
	enum ia_css_err ret = IA_CSS_SUCCESS;


	/* if is called in atomisp_reset(), force destroy stream */
	if ((ret = __destroy_stream(isp_subdev, true)) != IA_CSS_SUCCESS) {
		v4l2_err(&atomisp_dev, "destroy stream failed.\n");
		goto err;
	}
	/* if is called in atomisp_reset(), force destroy all pipes */

	if ((ret = __destroy_pipes(isp_subdev, true)) != IA_CSS_SUCCESS) {
		v4l2_err(&atomisp_dev, "destroy pipes failed.\n");
		goto err;
	}

	/*
	 * SP can not be stopped if other streams are still running
	 * if atomisp_subdev_streaming_count() tell there aleady has some subdev
	 * at streamming, then SP can not be stopped, so
	 * need to skip start sp procedure
	 */
	if (atomisp_subdev_streaming_count(isp_subdev->isp)) {
		v4l2_info(&atomisp_dev, "skip stop sp.\n");
	} else if (ia_css_isp_has_started()) {
		if (ia_css_stop_sp() != IA_CSS_SUCCESS) {
			v4l2_err(&atomisp_dev, "stop sp failed.\n");
			goto err;
		}
	}

	isp_subdev->css2_basis.stream_state = CSS2_STREAM_STOPPED;

	/* FIXME: Current code would cause streamon, then streamoff failed
	 * If configs are not cleared, it would create wrong pipe/stream in
	 * set format. No better solution has found yet.*/
	if (!in_reset) {
		for (i = 0; i < IA_CSS_PIPE_ID_NUM; i++) {
			ia_css_pipe_config_defaults(&isp_subdev->css2_basis.pipe_configs[i]);
			ia_css_pipe_extra_config_defaults(
						&isp_subdev->css2_basis.pipe_extra_configs[i]);
		}
		ia_css_stream_config_defaults(&isp_subdev->css2_basis.stream_config);
	}

	return IA_CSS_SUCCESS;

err:
	v4l2_err(&atomisp_dev, "stop css fatal error. cannot recover\n");
	return ret;
}

void
ia_css_disable_vf_pp(struct atomisp_sub_device *isp_subdev, bool disable)
{
	int i;
	for (i = 0; i < IA_CSS_PIPE_ID_NUM; i++)
		isp_subdev->css2_basis.pipe_extra_configs[i].disable_vf_pp
    							= !!disable;
}
void atomisp_css2_hw_store_8(hrt_address addr, uint8_t data)
{
        unsigned long flags;

	MMIO_LOCK
	PCI_CONFIG_LOCK
        _hrt_master_port_store_8(addr, data);
	PCI_CONFIG_UNLOCK
	MMIO_UNLOCK
}

void atomisp_css2_hw_store_16(hrt_address addr, uint16_t data)
{
        unsigned long flags;

	MMIO_LOCK
	PCI_CONFIG_LOCK
        _hrt_master_port_store_16(addr, data);
	PCI_CONFIG_UNLOCK
	MMIO_UNLOCK
}

void atomisp_css2_hw_store_32(hrt_address addr, uint32_t data)
{
        unsigned long flags;

	MMIO_LOCK
	PCI_CONFIG_LOCK
        _hrt_master_port_store_32(addr, data);
	PCI_CONFIG_UNLOCK
	MMIO_UNLOCK
}

uint8_t atomisp_css2_hw_load_8(hrt_address addr)
{
        unsigned long flags;
        uint8_t ret;

	MMIO_LOCK
	PCI_CONFIG_LOCK
        ret = _hrt_master_port_load_8(addr);
	PCI_CONFIG_UNLOCK
	MMIO_UNLOCK
        return ret;
}

uint16_t atomisp_css2_hw_load_16(hrt_address addr)
{
        unsigned long flags;
        uint16_t ret;

	MMIO_LOCK
	PCI_CONFIG_LOCK
        ret = _hrt_master_port_load_16(addr);
	PCI_CONFIG_UNLOCK
	MMIO_UNLOCK
        return ret;
}
uint32_t atomisp_css2_hw_load_32(hrt_address addr)
{
        unsigned long flags;
        uint32_t ret;

	MMIO_LOCK
	PCI_CONFIG_LOCK
        ret = _hrt_master_port_load_32(addr);
	PCI_CONFIG_UNLOCK
	MMIO_UNLOCK
        return ret;
}

void atomisp_css2_hw_store(hrt_address addr,
                                  const void *from, uint32_t n)
{
        unsigned long flags;
        unsigned i;
        unsigned int _to = (unsigned int)addr;
        const char *_from = (const char *)from;

	MMIO_LOCK
	PCI_CONFIG_LOCK
        for (i = 0; i < n; i++, _to++, _from++)
                _hrt_master_port_store_8(_to , *_from);
	PCI_CONFIG_UNLOCK
	MMIO_UNLOCK
}

void atomisp_css2_hw_load(hrt_address addr, void *to, uint32_t n)
{
        unsigned long flags;
        unsigned i;
        char *_to = (char *)to;
        unsigned int _from = (unsigned int)addr;

	MMIO_LOCK
	PCI_CONFIG_LOCK
        for (i = 0; i < n; i++, _to++, _from++)
                *_to = _hrt_master_port_load_8(_from);
	PCI_CONFIG_UNLOCK
	MMIO_UNLOCK
}
