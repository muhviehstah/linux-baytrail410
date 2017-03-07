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
#include "sh_css_sp.h"
#include "atomisp_internal.h"

#define ia_css_sp_has_booted() ia_css_sp_has_initialized()

struct atomisp_sub_device;

//static inline enum sh_css_err sh_css_allocate_continuous_frames(bool enable)
//{
	//return sh_css_err_unsupported_configuration;
//}

void atomisp_css_uninit(struct atomisp_device *isp);

enum ia_css_err ia_css_preview_configure_output(
	struct atomisp_sub_device *isp_subdev,
	unsigned int width,
	unsigned int height,
	enum ia_css_frame_format format);

enum ia_css_err ia_css_preview_get_output_frame_info(
	struct atomisp_sub_device *isp_subdev,
	struct ia_css_frame_info *info);

enum ia_css_err ia_css_capture_configure_output(
	struct atomisp_sub_device *isp_subdev,
	unsigned int width,
	unsigned int height,
	enum ia_css_frame_format format);

enum ia_css_err ia_css_capture_get_output_frame_info(
	struct atomisp_sub_device *isp_subdev,
	struct ia_css_frame_info *info);

enum ia_css_err ia_css_capture_configure_viewfinder(
	struct atomisp_sub_device *isp_subdev,
	unsigned int width,
	unsigned int height,
	enum ia_css_frame_format format);

enum ia_css_err ia_css_capture_get_viewfinder_frame_info(
	struct atomisp_sub_device *isp_subdev,
	struct ia_css_frame_info *info);

enum ia_css_err ia_css_video_configure_output(
	struct atomisp_sub_device *isp_subdev,
	unsigned int width,
	unsigned int height,
	enum ia_css_frame_format format);

enum ia_css_err ia_css_video_get_output_frame_info(
	struct atomisp_sub_device *isp_subdev,
	struct ia_css_frame_info *info);

enum ia_css_err ia_css_video_configure_viewfinder(
	struct atomisp_sub_device *isp_subdev,
	unsigned int width,
	unsigned int height,
	enum ia_css_frame_format format);

enum ia_css_err ia_css_video_get_viewfinder_frame_info(
	struct atomisp_sub_device *isp_subdev,
	struct ia_css_frame_info *info);

enum ia_css_err ia_css_preview_configure_pp_input(
	struct atomisp_sub_device *isp_subdev,
	unsigned int width,
	unsigned int height);

enum ia_css_err ia_css_capture_configure_pp_input(
	struct atomisp_sub_device *isp_subdev,
	unsigned int width,
	unsigned int height);

void
ia_css_enable_dz(struct atomisp_sub_device *isp_subdev,
                       bool enable);
void
ia_css_capture_set_mode(struct atomisp_sub_device *isp_subdev,
			enum ia_css_capture_mode mode);

void
ia_css_input_set_resolution(struct atomisp_sub_device *isp_subdev,
                       struct v4l2_mbus_framefmt *ffmt);
void
ia_css_input_set_format(struct atomisp_sub_device *isp_subdev,
                       enum ia_css_stream_format format);
void
ia_css_input_set_binning_factor(struct atomisp_sub_device *isp_subdev,
                       unsigned int binning_factor);
void
ia_css_input_set_bayer_order(struct atomisp_sub_device *isp_subdev,
                       enum ia_css_bayer_order bayer_order);

void
ia_css_capture_enable_online(struct atomisp_sub_device *isp_subdev,
			     bool enable);

void
ia_css_enable_raw_binning(struct atomisp_sub_device *isp_subdev,
			     bool enable);
void
ia_css_video_set_dis_envelope(struct atomisp_sub_device *isp_subdev,
                 unsigned int dvs_w, unsigned int dvs_h);

void
ia_css_input_set_two_pixels_per_clock(struct atomisp_sub_device *isp_subdev,
					   bool enable);

void ia_css_preview_enable_online(struct atomisp_sub_device *isp_subdev,
				  bool enable);
void ia_css_enable_continuous(struct atomisp_sub_device *isp_subdev,
				  bool enable);
enum ia_css_err ia_css_capture_get_output_raw_frame_info(
	struct atomisp_sub_device *isp_subdev,
	struct ia_css_frame_info *info);

//void atomisp_sh_css_mmu_set_page_table_base_index(unsigned int base_index);
int atomisp_css_init(struct atomisp_device *isp);
enum ia_css_err ia_css_stop(struct atomisp_sub_device *isp_subdev, bool need_reset);
enum ia_css_err ia_css_start(struct atomisp_sub_device *isp_subdev, bool need_reset);
void ia_css_disable_vf_pp(struct atomisp_sub_device *isp_subdev, bool disable);

void
ia_css_input_set_mode(struct atomisp_sub_device *isp_subdev,
		      enum ia_css_input_mode mode);

enum ia_css_err __destroy_stream(struct atomisp_sub_device *isp_subdev, bool force);

enum ia_css_err __destroy_pipes(struct atomisp_sub_device *isp_subdev, bool force);

enum ia_css_input_mode get_input_mode(unsigned int input_type, struct atomisp_device *isp);

enum ia_css_err ia_css_update_stream(struct atomisp_sub_device *isp_subdev);

void atomisp_destroy_pipes_stream_force(struct atomisp_sub_device *isp_subdev);
void atomisp_css_update_isp_params(struct atomisp_sub_device *isp_subdev);

void atomisp_css2_hw_store_8(hrt_address addr, uint8_t data);
void atomisp_css2_hw_store_16(hrt_address addr, uint16_t data);
void atomisp_css2_hw_store_32(hrt_address addr, uint32_t data);
uint8_t atomisp_css2_hw_load_8(hrt_address addr);
uint16_t atomisp_css2_hw_load_16(hrt_address addr);
uint32_t atomisp_css2_hw_load_32(hrt_address addr);
void atomisp_css2_hw_store(hrt_address addr, const void *from, uint32_t n);
void atomisp_css2_hw_load(hrt_address addr, void *to, uint32_t n);
