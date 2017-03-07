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
#ifndef ATOMISP_PLATFORM_H_
#define ATOMISP_PLATFORM_H_

#include <linux/i2c.h>
#include <linux/sfi.h>
#include <media/v4l2-subdev.h>
#include "atomisp.h"

enum atomisp_bayer_order {
	atomisp_bayer_order_grbg,
	atomisp_bayer_order_rggb,
	atomisp_bayer_order_bggr,
	atomisp_bayer_order_gbrg
};

enum atomisp_input_format {
	ATOMISP_INPUT_FORMAT_YUV420_8_LEGACY,/* 8 bits per subpixel (legacy) */
	ATOMISP_INPUT_FORMAT_YUV420_8, /* 8 bits per subpixel */
	ATOMISP_INPUT_FORMAT_YUV420_10,/* 10 bits per subpixel */
	ATOMISP_INPUT_FORMAT_YUV422_8, /* UYVY..UVYV, 8 bits per subpixel */
	ATOMISP_INPUT_FORMAT_YUV422_10,/* UYVY..UVYV, 10 bits per subpixel */
	ATOMISP_INPUT_FORMAT_RGB_444,  /* BGR..BGR, 4 bits per subpixel */
	ATOMISP_INPUT_FORMAT_RGB_555,  /* BGR..BGR, 5 bits per subpixel */
	ATOMISP_INPUT_FORMAT_RGB_565,  /* BGR..BGR, 5 bits B and $, 6 bits G */
	ATOMISP_INPUT_FORMAT_RGB_666,  /* BGR..BGR, 6 bits per subpixel */
	ATOMISP_INPUT_FORMAT_RGB_888,  /* BGR..BGR, 8 bits per subpixel */
	ATOMISP_INPUT_FORMAT_RAW_6,    /* RAW data, 6 bits per pixel */
	ATOMISP_INPUT_FORMAT_RAW_7,    /* RAW data, 7 bits per pixel */
	ATOMISP_INPUT_FORMAT_RAW_8,    /* RAW data, 8 bits per pixel */
	ATOMISP_INPUT_FORMAT_RAW_10,   /* RAW data, 10 bits per pixel */
	ATOMISP_INPUT_FORMAT_RAW_12,   /* RAW data, 12 bits per pixel */
	ATOMISP_INPUT_FORMAT_RAW_14,   /* RAW data, 14 bits per pixel */
	ATOMISP_INPUT_FORMAT_RAW_16,   /* RAW data, 16 bits per pixel */
	ATOMISP_INPUT_FORMAT_BINARY_8, /* Binary byte stream. */
};

enum intel_v4l2_subdev_type {
	RAW_CAMERA = 1,
	SOC_CAMERA = 2,
	CAMERA_MOTOR = 3,
	LED_FLASH = 4,
	XENON_FLASH = 5,
	FILE_INPUT = 6,
	TEST_PATTERN = 7,
};

struct intel_v4l2_subdev_id {
	char name[17];
	enum intel_v4l2_subdev_type type;
	enum atomisp_camera_port    port;
	int num_lanes;
};

struct intel_v4l2_subdev_i2c_board_info {
	struct i2c_board_info board_info;
	int i2c_adapter_id;
};

struct intel_v4l2_subdev_table {
	struct intel_v4l2_subdev_i2c_board_info v4l2_subdev;
	enum intel_v4l2_subdev_type type;
	enum atomisp_camera_port port;
	int num_lanes;
};

struct atomisp_platform_data {
	struct intel_v4l2_subdev_table *subdevs;
};

struct camera_sensor_platform_data {
	int (*gpio_ctrl)(struct v4l2_subdev *subdev, int flag);
	int (*power_ctrl)(struct v4l2_subdev *subdev, int flag);
	int (*csi_cfg)(struct v4l2_subdev *subdev, int flag);
	bool (*low_fps)(void);
	int (*platform_init)(struct i2c_client *);
	int (*platform_deinit)(void);
};

struct camera_af_platform_data {
	int (*power_ctrl)(struct v4l2_subdev *subdev, int flag);
};

const struct camera_af_platform_data *camera_get_af_platform_data(void);

struct camera_mipi_info {
	enum atomisp_camera_port        port;
	unsigned int                    num_lanes;
	struct atomisp_sensor_mode_data data;
};

extern const struct atomisp_platform_data *atomisp_get_platform_data(void);

/* NOTE: Most of below constants could come from platform data.
 *  * To be fixed when appropriate ACPI support comes.
 *   */
#define VLV2_PMC_CLK_BASE_ADDRESS       0xfed03060
#define PLT_CLK_CTL_OFFSET(x)           (0x04 * (x))

#define CLK_CONFG_BIT_POS               0
#define CLK_CONFG_BIT_LEN               2
#define CLK_CONFG_D3_GATED              0
#define CLK_CONFG_FORCE_ON              1
#define CLK_CONFG_FORCE_OFF             2

#define CLK_FREQ_TYPE_BIT_POS           2
#define CLK_FREQ_TYPE_BIT_LEN           1
#define CLK_FREQ_TYPE_XTAL              0       /* 25 MHz */
#define CLK_FREQ_TYPE_PLL               1       /* 19.2 MHz */

#define MAX_CLK_COUNT                   6

extern struct atomisp_platform_data *__intel_get_v4l2_subdev_table(void);

int byt_plat_set_clock_freq(int clk_num, int freq_type);
int byt_plat_get_clock_freq(int clk_num);
int byt_plat_configure_clock(int clk_num, u32 conf);
int byt_plat_get_clock_status(int clk_num);
int byt_plat_clk_init(void);

#endif /* ATOMISP_PLATFORM_H_ */
