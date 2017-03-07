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

#include "ia_css_frame.h"
#include "ia_css.h"
#include "ia_css_pipeline.h"
#define IA_CSS_INCLUDE_CONFIGURATIONS
#include "ia_css_isp_configs.h"
#include "ia_css_iterator.host.h"

void
ia_css_iterator_config(
	struct sh_css_isp_iterator_isp_config *to,
	const struct ia_css_iterator_configuration *from)
{
	ia_css_frame_info_to_frame_sp_info(&to->input_info,    from->input_info);
	ia_css_frame_info_to_frame_sp_info(&to->internal_info, from->internal_info);
	ia_css_frame_info_to_frame_sp_info(&to->output_info,   from->output_info);
	ia_css_resolution_to_sp_resolution(&to->dvs_envelope,  from->dvs_envelope);
}

enum ia_css_err
ia_css_iterator_configure(
	const struct ia_css_binary *binary,
	const struct ia_css_frame_info *in_info)
{
	struct ia_css_iterator_configuration config = {
		&binary->in_frame_info,
		&binary->internal_frame_info,
		&binary->out_frame_info[0],
		&binary->dvs_envelope };
	/* Use in_info iso binary->in_frame_info.
	 * They can differ in padded width in case of scaling, e.g. for capture_pp.
	 * Find out why.
	*/
	if (in_info)
		config.input_info = in_info;
	if (binary->out_frame_info[0].res.width == 0)
		config.output_info = &binary->out_frame_info[1];
	ia_css_configure_iterator (binary, &config);
	return IA_CSS_SUCCESS;
}
