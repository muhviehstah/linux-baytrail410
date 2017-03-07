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
#include "math_support.h"
#define IA_CSS_INCLUDE_CONFIGURATIONS
#include "ia_css_isp_configs.h"

#include "sh_css_param_dvs.h"

#include "ia_css_dvs.host.h"


void
ia_css_dvs_config(
	struct sh_css_isp_dvs_isp_config *to,
	const struct ia_css_dvs_configuration  *from)
{
	to->num_horizontal_blocks =
	    DVS_NUM_BLOCKS_X(from->info->res.width);
	to->num_vertical_blocks =
	    DVS_NUM_BLOCKS_Y(from->info->res.height);
}

void
ia_css_dvs_configure(
	const struct ia_css_binary     *binary,
	const struct ia_css_frame_info *info)
{
	const struct ia_css_dvs_configuration config =
		{ info };
	ia_css_configure_dvs(binary, &config);
}
