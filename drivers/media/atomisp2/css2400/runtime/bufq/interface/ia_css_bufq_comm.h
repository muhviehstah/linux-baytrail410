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

#ifndef _IA_CSS_BUFQ_COMM_H
#define _IA_CSS_BUFQ_COMM_H

#include "system_global.h"

/** Enumeration of buffer types. Buffers can be queued and de-queued
 *  to hand them over between IA and ISP.
 */
/*
 * The first frames (with comment Dynamic) can be dynamic or static
 * The other frames (ref_in and below) can only be static
 * Static means that the data addres will not change during the life time
 * of the associated pipe. Dynamic means that the data address can
 * change with every (frame) iteration of the associated pipe
 *
 * s3a and dis are now also dynamic but (stil) handled seperately
 */
enum ia_css_buffer_type {
	IA_CSS_BUFFER_TYPE_INVALID = -1,
	IA_CSS_BUFFER_TYPE_3A_STATISTICS = 0,
	IA_CSS_BUFFER_TYPE_DIS_STATISTICS,
	IA_CSS_BUFFER_TYPE_LACE_STATISTICS,
	IA_CSS_BUFFER_TYPE_INPUT_FRAME,
	IA_CSS_BUFFER_TYPE_OUTPUT_FRAME,
	IA_CSS_BUFFER_TYPE_SEC_OUTPUT_FRAME,
	IA_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME,
	IA_CSS_BUFFER_TYPE_SEC_VF_OUTPUT_FRAME,
	IA_CSS_BUFFER_TYPE_RAW_OUTPUT_FRAME,
	IA_CSS_BUFFER_TYPE_CUSTOM_INPUT,
	IA_CSS_BUFFER_TYPE_CUSTOM_OUTPUT,
	IA_CSS_BUFFER_TYPE_METADATA,
	IA_CSS_BUFFER_TYPE_PARAMETER_SET,
	IA_CSS_NUM_DYNAMIC_BUFFER_TYPE,
	IA_CSS_BUFFER_TYPE_VIDEO_DELAY_0,
	IA_CSS_BUFFER_TYPE_VIDEO_DELAY_1,
	IA_CSS_BUFFER_TYPE_VIDEO_DELAY_2,
	IA_CSS_BUFFER_TYPE_EXTRA,
	IA_CSS_NUM_BUFFER_TYPE
};

#if 0
enum sh_css_queue_id {
	SH_CSS_INVALID_BUFFER_QUEUE_ID     = -1,
	SH_CSS_INPUT_BUFFER_QUEUE_ID       = 0,
	SH_CSS_OUTPUT_BUFFER_QUEUE_ID      = 1,
	SH_CSS_VF_OUTPUT_BUFFER_QUEUE_ID   = 2,
	SH_CSS_S3A_BUFFER_QUEUE_ID         = 3,
	SH_CSS_DIS_BUFFER_QUEUE_ID         = 4,
	SH_CSS_PARAM_BUFFER_QUEUE_ID       = 5,
	SH_CSS_TAG_CMD_QUEUE_ID            = 6,
#if !defined (SH_CSS_ENABLE_METADATA)
	SH_CSS_NUM_BUFFER_QUEUE_ID         = SH_CSS_TAG_CMD_QUEUE_ID,
#else
	SH_CSS_METADATA_BUFFER_QUEUE_ID    = 7,
	SH_CSS_NUM_BUFFER_QUEUE_ID         = SH_CSS_METADATA_BUFFER_QUEUE_ID,
#endif
};
#endif

enum sh_css_queue_id {
	SH_CSS_INVALID_QUEUE_ID     = -1,
	SH_CSS_QUEUE_A_ID = 0,
	SH_CSS_QUEUE_B_ID,
	SH_CSS_QUEUE_C_ID,
	SH_CSS_QUEUE_D_ID,
	SH_CSS_QUEUE_E_ID,
	SH_CSS_QUEUE_F_ID,
#if defined(HAS_NO_INPUT_SYSTEM) || defined(USE_INPUT_SYSTEM_VERSION_2401)
	/* input frame queue for skycam */
	SH_CSS_QUEUE_G_ID,
	SH_CSS_QUEUE_H_ID, /* for metadata */
#endif
	SH_CSS_MAX_NUM_QUEUES
};

#define SH_CSS_MAX_DYNAMIC_BUFFERS_PER_THREAD SH_CSS_MAX_NUM_QUEUES
/* for now we staticaly assign queue 0 to parameter */
#define IA_CSS_PARAMETER_SET_QUEUE_ID SH_CSS_QUEUE_A_ID
//#define SH_CSS_MAX_NUM_QUEUES SH_CSS_MAX_DYNAMIC_BUFFERS_PER_THREAD



#endif
