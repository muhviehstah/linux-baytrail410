/*
 * platform_mt9m114.c: mt9m114 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/atomisp_platform.h>

#include <media/v4l2-subdev.h>
#include <linux/regulator/consumer.h>
#include "platform_camera.h"
#include "platform_mt9m114.h"


#define VPROG1_VAL 2800000
static int camera_reset;
static int camera_power_down;
static int camera_vprog1_on;

#ifdef CONFIG_BOARD_CTP
static struct regulator *vprog1_reg;
#endif

/*
 * MFLD PR2 secondary camera sensor - MT9M114 platform data
 */
static int mt9m114_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;
	TIMING_ENTER
	ret = camera_sensor_gpio(-1, ANVL_GPIO_CAM2_RST,
				 GPIOF_DIR_OUT, 1);
	if (ret < 0)
		return ret;

	if (flag) {
		//Reset the camera sensor
		camera_sensor_gpio(-1, ANVL_GPIO_CAM2_RST,
					 GPIOF_DIR_OUT, 0);
		msleep(60);
		camera_sensor_gpio(-1, ANVL_GPIO_CAM2_RST,
					 GPIOF_DIR_OUT, 1);
	} else
		camera_sensor_gpio(-1, ANVL_GPIO_CAM2_RST,
					 GPIOF_DIR_OUT, 1);
	TIMING_END
	return 0;
}

static int mt9e013_reset_value;
static int mt9m114_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	//Do nothing for now
#ifdef CONFIG_BOARD_CTP
	int reg_err;
	return 0;
#endif
#ifndef CONFIG_BOARD_CTP
	int ret;
	return 0;
	/* Note here, there maybe a workaround to avoid I2C SDA issue */
	if (camera_power_down < 0) {
		ret = camera_sensor_gpio(-1, ANVL_GPIO_CAM2_PWDN,
					GPIOF_DIR_OUT, 1);
#ifndef CONFIG_BOARD_REDRIDGE
		if (ret < 0)
			return ret;
#endif
		camera_power_down = ret;
	}

	if (camera_reset < 0) {
		ret = camera_sensor_gpio(-1, ANVL_GPIO_CAM2_RST,
					 GPIOF_DIR_OUT, 1);
		if (ret < 0)
			return ret;
		camera_reset = ret;
	}
#endif
	if (flag) {
#ifndef CONFIG_BOARD_CTP
#if 0
		if (!mt9e013_reset_value) {
			if (mt9e013_reset)
				mt9e013_reset(sd);
			mt9e013_reset_value = 1;
		}
#endif
#ifdef CONFIG_BOARD_REDRIDGE
		gpio_direction_output(camera_reset, 0);
#endif
		gpio_set_value(camera_reset, 0);
#endif
		if (!camera_vprog1_on) {
			camera_vprog1_on = 1;
#ifdef CONFIG_BOARD_CTP
			reg_err = regulator_enable(vprog1_reg);
			if (reg_err) {
				printk(KERN_ALERT "Failed to enable regulator vprog1\n");
				return reg_err;
			}
#endif
		}
#ifndef CONFIG_BOARD_CTP
#ifdef CONFIG_BOARD_REDRIDGE
		if (camera_power_down >= 0)
			gpio_set_value(camera_power_down, 1);
#else
		gpio_set_value(camera_power_down, 1);
#endif
#endif
	} else {
		if (camera_vprog1_on) {
			camera_vprog1_on = 0;
#ifdef CONFIG_BOARD_CTP
			reg_err = regulator_disable(vprog1_reg);
			if (reg_err) {
				printk(KERN_ALERT "Failed to disable regulator vprog1\n");
				return reg_err;
			}
#endif
		}
#ifndef CONFIG_BOARD_CTP
#ifdef CONFIG_BOARD_REDRIDGE
		if (camera_power_down >= 0)
			gpio_set_value(camera_power_down, 0);
#else
		gpio_set_value(camera_power_down, 0);
#endif

		mt9e013_reset_value = 0;
#endif
	}

	return 0;
}

struct intel_v4l2_subdev_table *mt9m114_camera_data(void)
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
		if(strcmp(board_info->type, I2C_MT9M114_NAME) == 0)
			return subdevs;
	}
	return NULL;

}
static int mt9m114_csi_configure(struct v4l2_subdev *sd, int flag)
{
	int ret;
	/* soc sensor, there is no raw bayer order (set to -1) */
	struct intel_v4l2_subdev_table *subdev;

		subdev = mt9m114_camera_data();
		if(subdev == NULL){
			pr_err("Error in getting OV5640_1 camera data\n");
			return -EINVAL;
		}
	TIMING_ENTER
	ret = camera_sensor_csi(sd, subdev->port, subdev->num_lanes, flag);
	TIMING_END;
	return ret;
}

#ifdef CONFIG_BOARD_CTP
static int mt9m114_platform_init(struct i2c_client *client)
{
	int ret;

	vprog1_reg = regulator_get(&client->dev, "vprog1");
	if (IS_ERR(vprog1_reg)) {
		dev_err(&client->dev, "regulator_get failed\n");
		return PTR_ERR(vprog1_reg);
	}
	ret = regulator_set_voltage(vprog1_reg, VPROG1_VAL, VPROG1_VAL);
	if (ret) {
		dev_err(&client->dev, "regulator voltage set failed\n");
		regulator_put(vprog1_reg);
	}
	return ret;
}

static int mt9m114_platform_deinit(void)
{
	regulator_put(vprog1_reg);
	return 0;
}
#endif

static struct camera_sensor_platform_data mt9m114_sensor_platform_data = {
	.gpio_ctrl	= mt9m114_gpio_ctrl,
	.power_ctrl	= mt9m114_power_ctrl,
	.csi_cfg	= mt9m114_csi_configure,
#ifdef CONFIG_BOARD_CTP
	.platform_init = mt9m114_platform_init,
	.platform_deinit = mt9m114_platform_deinit,
#endif
};


void *mt9m114_platform_data(void)
{
	camera_reset = -1;
	camera_power_down = -1;

	return &mt9m114_sensor_platform_data;
}



