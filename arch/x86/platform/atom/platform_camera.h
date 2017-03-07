/*
 * platform_camera.h: CAMERA platform library header file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_CAMERA_H_
#define _PLATFORM_CAMERA_H_

#include <linux/atomisp_platform.h>

/*extern const struct intel_v4l2_subdev_id v4l2_ids[] __attribute__((weak));*/

/* MFLD iCDK camera sensor GPIOs */


#define ANVL_GPIO_CAM2_PWDN 		"cam_mt_power"  //Power down signal for J3 ov5640 sensor
#define ANVL_GPIO_CAM2_RST			"cam_mt_reset"  //Power reset signal for 1.2MP sensor
#define ANVL_GPIO_CAM1_PWDN			"cam_ov1_power" //Power down signal for J2 ov5640 sensor
#define ANVL_GPIO_CAM1_RST 			"cam_ov1_reset" //Power reset signal for J2 ov5640 sensor
#define ANVL_GPIO_CAM3_PWDN 		"cam_ov2_power"
#define ANVL_GPIO_CAM3_RST 			"cam_ov2_reset" //Power reset signal for J3 ov5640 sensor

#define ANVL_GPIO_CAM2_RST_NUM 		25		//Power reset signal for 1.2MP sensor
#define ANVL_GPIO_CAM2_PWDN_NUM 	22		//Power down signal for J3 ov5640 sensor
#define ANVL_GPIO_CAM1_PWDN_NUM		21		//Power down signal for J2 ov5640 sensor
#define ANVL_GPIO_CAM1_RST_NUM 		24		//Power reset signal for J2 ov5640 sensor
#define ANVL_GPIO_CAM3_PWDN_NUM 	21
#define ANVL_GPIO_CAM3_RST_NUM 		26		//Power reset signal for J3 ov5640 sensor

#define ANVL_I2C_BUS_NUM 		3
#define IOBASEADDR				0xFED0C000
#define GPIONC_21_OFFSET		0x1060		//Power down signal for J2 ov5640 sensor
#define GPIONC_22_OFFSET		0x10A0		//Power down signal for J3 ov5640 sensor
#define GPIONC_24_OFFSET		0x1020		//Power reset signal for J2 ov5640 sensor
#define GPIONC_25_OFFSET		0x1050		//Power reset signal for 1.2MP sensor
#define GPIONC_26_OFFSET		0x1090		//Power reset signal for J3 ov5640 sensor

extern int camera_sensor_gpio(int gpio, char *name, int dir, int value);
extern int camera_sensor_csi(struct v4l2_subdev *sd, int port,
			int lanes, int flag);

extern struct atomisp_platform_data *intel_get_v4l2_subdev_table(void)  __attribute__((weak));;

#endif
