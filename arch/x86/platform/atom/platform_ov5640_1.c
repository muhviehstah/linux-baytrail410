/*
 * platform_ov5640_1.c: ov5640_1 platform data initilization file
 *
 * (C) Copyright 2012 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <asm/io.h>
#include <media/v4l2-subdev.h>
#include "platform_camera.h"
#include "platform_ov5640_1.h"


static int camera_reset;
static int camera_power_down;

/*
 * GRACELAND DV1 primary camera sensor - OV5640_1 platform data
 */

static int ov5640_1_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{

	if (flag) {
	  camera_sensor_gpio(ANVL_GPIO_CAM1_RST_NUM, ANVL_GPIO_CAM1_RST,GPIOF_DIR_OUT, 0);
	  msleep(20);
	  camera_sensor_gpio(ANVL_GPIO_CAM1_RST_NUM, ANVL_GPIO_CAM1_RST,GPIOF_DIR_OUT, 1);
	} else
	{
	  camera_sensor_gpio(ANVL_GPIO_CAM1_RST_NUM, ANVL_GPIO_CAM1_RST,GPIOF_DIR_OUT, 0);
	}

	return 0;
}

static int ov5640_1_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	if (flag) {
	  if (camera_power_down < 0) {
               ret = camera_sensor_gpio(-1, ANVL_GPIO_CAM1_PWDN,
                                       GPIOF_DIR_OUT, 1);
		pr_debug("ov5640_1 - gpio power down to up  ov5640_1 - primary\n");
		camera_power_down = ret;
	  }
	  if (camera_reset < 0) {
               ret = camera_sensor_gpio(-1, ANVL_GPIO_CAM1_RST,
                                        GPIOF_DIR_OUT, 1);
		pr_debug("ov5640_1 - gpio power reset  ov5640_1-primary\n");
            	camera_reset = ret;
	  }
            /* delay 20ms to wait sensor power up stable.*/
	  msleep(20);
	}

	return 0;
}

struct intel_v4l2_subdev_table *ov5640_1_camera_data(void)
{
	const struct atomisp_platform_data *pdata;
	struct intel_v4l2_subdev_table *subdevs;

	pdata = __intel_get_v4l2_subdev_table();
	if (pdata == NULL) {
			pr_err("no platform data available\n");
			return NULL;
		}
	for (subdevs = pdata->subdevs; subdevs->type; ++subdevs) {
		struct i2c_board_info *board_info =	&subdevs->v4l2_subdev.board_info;
		//printk("camera_intel_platform_data check %s\n",board_info->type);
		if(strcmp(board_info->type, I2C_OV5640_1_NAME) == 0)
			return subdevs;
	}
	return NULL;

}
static int ov5640_1_csi_configure(struct v4l2_subdev *sd, int flag)
{
	int ret;
	struct intel_v4l2_subdev_table *subdev;

	subdev = ov5640_1_camera_data();
	if(subdev == NULL){
		pr_err("Error in getting OV5640_1 camera data\n");
		return -EINVAL;
	}
	ret =  camera_sensor_csi(sd, subdev->port, subdev->num_lanes, flag);
	if (ret != 0) {
		pr_err("Error in configure camera csi  - OV5640_1-primary\n");
		return ret;
	}
	return ret;
}

static struct camera_sensor_platform_data ov5640_1_sensor_platform_data = {
	.gpio_ctrl      = ov5640_1_gpio_ctrl,
	.power_ctrl     = ov5640_1_power_ctrl,
	.csi_cfg        = ov5640_1_csi_configure,
};

void *ov5640_1_platform_data(void)
{
	camera_reset = -1;
	camera_power_down = -1;
	return &ov5640_1_sensor_platform_data;
}
