/*
 * platform_mt9m114.h: mt9m114 platform data header file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_MT9M114_H_
#define _PLATFORM_MT9M114_H_
#define I2C_MT9M114_NAME "mt9m114"


void *mt9m114_platform_data(void);
struct intel_v4l2_subdev_table *mt9m114_camera_data(void);

#endif
