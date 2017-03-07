/*
 * platform_ov5640.h: ov5640 platform data header file
 *
 * (C) Copyright 2012 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_OV5460_2_H_
#define _PLATFORM_OV5460_2_H_
#define I2C_OV5640_2_NAME "ov5640-2"

void *ov5640_2_platform_data(void);
void ov5640_2_powerdown_camera_1(void);
struct intel_v4l2_subdev_table *ov5640_2_camera_data(void);

#endif
