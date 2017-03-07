/*
 * platform_camera.c: Camera platform library file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/atomisp_platform.h>
#include <media/v4l2-subdev.h>
#include "platform_camera.h"
#include "platform_mt9m114.h"
#include "platform_ov5640_1.h"
#include "platform_ov5640_2.h"


static const struct intel_v4l2_subdev_id v4l2_ids[] = {
	 {I2C_MT9M114_NAME, SOC_CAMERA, ATOMISP_CAMERA_PORT_SECONDARY, 1},
	 {I2C_OV5640_2_NAME, SOC_CAMERA, ATOMISP_CAMERA_PORT_THIRD, 2},
	 {I2C_OV5640_1_NAME, SOC_CAMERA, ATOMISP_CAMERA_PORT_PRIMARY, 2},
	{},
 };


#define GPIO_NAME_LEN 16
struct gpio_table_entry {
	int	pin_no;
	char	pin_name[GPIO_NAME_LEN];
};

static const struct gpio_table_entry gpio_table[] = {
	{ANVL_GPIO_CAM2_PWDN_NUM, ANVL_GPIO_CAM2_PWDN},
	{ANVL_GPIO_CAM2_RST_NUM, ANVL_GPIO_CAM2_RST},
	{ANVL_GPIO_CAM1_PWDN_NUM, ANVL_GPIO_CAM1_PWDN},
	{ANVL_GPIO_CAM1_RST_NUM, ANVL_GPIO_CAM1_RST},
	{ANVL_GPIO_CAM3_PWDN_NUM, ANVL_GPIO_CAM3_PWDN},
	{ANVL_GPIO_CAM3_RST_NUM, ANVL_GPIO_CAM3_RST},
	{}
};

static int __gpio_direction_output(int pin, int value)
{
	volatile void __iomem *gpio_base = NULL;
	int GPIO_OFFSET = 0;
	int temp = 0;
	switch(pin)
	{
	  case 21:
	    GPIO_OFFSET = GPIONC_21_OFFSET;
	    break;
	  case 22:
	    GPIO_OFFSET = GPIONC_22_OFFSET;
	    break;
	  case 24:
	    GPIO_OFFSET = GPIONC_24_OFFSET;
	    break;
	  case 25:
	    GPIO_OFFSET = GPIONC_25_OFFSET;
	    break;
	  case 26:
	    GPIO_OFFSET = GPIONC_26_OFFSET;
	    break;
	  default:
	    return -EINVAL;
	}

	gpio_base = ioremap_nocache(IOBASEADDR+GPIO_OFFSET,0xF);
	if(gpio_base == NULL)
	{
	  return -EINVAL;
	}

	//configure the pin to output pin
	temp=readl(gpio_base+8);
	temp = 0x2;

	writel(temp,gpio_base+8);


	temp=readl(gpio_base);

	if(value)
	  temp=temp|0x00000080; //set it high
	else
	  temp=temp|0x00000100;	//set it low

	writel(temp,gpio_base);


	iounmap(gpio_base);

	return 0;
}

static int get_gpio_by_name(const char *name)
{
	const struct gpio_table_entry *pentry = gpio_table;
	int i;
	int arraysize = sizeof(gpio_table)/sizeof(gpio_table[0]);

	if (!pentry)
		return -1;
	for (i = 0; i < arraysize; i++, pentry++) {
		if (!strncmp(name, pentry->pin_name, GPIO_NAME_LEN))
			return pentry->pin_no;
	}
	return -1;
}

/*
 * One-time gpio initialization.
 * @name: gpio name: coded in SFI table
 * @gpio: gpio pin number (bypass @name)
 * @dir: GPIOF_DIR_IN or GPIOF_DIR_OUT
 * @value: if dir = GPIOF_DIR_OUT, this is the init value for output pin
 * if dir = GPIOF_DIR_IN, this argument is ignored
 * return: a positive pin number if succeeds, otherwise a negative value
 */
int camera_sensor_gpio(int gpio, char *name, int dir, int value)
{
	int ret, pin;
	TIMING_ENTER
	if (gpio == -1) {
		pin = get_gpio_by_name(name);
		if (pin == -1) {
			pr_err("%s: failed to get gpio(name: %s)\n",
						__func__, name);
			return -EINVAL;
		}
	} else {
		pin = gpio;
	}

	if (dir == GPIOF_DIR_OUT)
		ret = __gpio_direction_output(pin, value);
	else
		ret = gpio_direction_input(pin);

	if (ret) {
		pr_err("%s: failed to set gpio(pin %d) direction\n",
							__func__, pin);
		gpio_free(pin);
	}
	TIMING_END
	return ret ? ret : pin;
}

/*
 * Configure MIPI CSI physical parameters.
 * @port: ATOMISP_CAMERA_PORT_PRIMARY or ATOMISP_CAMERA_PORT_SECONDARY
 * @lanes: for ATOMISP_CAMERA_PORT_PRIMARY, there could be 2 or 4 lanes
 * for ATOMISP_CAMERA_PORT_SECONDARY, there is only one lane.
 * @format: MIPI CSI pixel format, see include/linux/atomisp_platform.h
 * @bayer_order: MIPI CSI bayer order, see include/linux/atomisp_platform.h
 */
int camera_sensor_csi(struct v4l2_subdev *sd, int port,
			int lanes, int flag)
{
	struct i2c_client *client;
	struct camera_mipi_info *csi = NULL;
	TIMING_ENTER

	client = v4l2_get_subdevdata(sd);
	if (flag) {
		csi = kzalloc(sizeof(*csi), GFP_KERNEL);
		if (!csi) {
			dev_err(&client->dev, "out of memory\n");
			return -ENOMEM;
		}
		csi->port = port;
		csi->num_lanes = lanes;
		v4l2_set_subdev_hostdata(sd, (void *)csi);
	} else {
		csi = v4l2_get_subdev_hostdata(sd);
		kfree(csi);
	}
	TIMING_END
	return 0;
}



static struct atomisp_platform_data *v4l2_subdev_table_head;

static void intel_ignore_i2c_device_register(int bus,
					     struct i2c_board_info *idev)
{
	const struct intel_v4l2_subdev_id *vdev = v4l2_ids;
	struct intel_v4l2_subdev_i2c_board_info *info;
	static struct intel_v4l2_subdev_table *subdev_table;
	enum intel_v4l2_subdev_type type = 0;
	enum atomisp_camera_port port;
	int num_lanes;
	static int i;

	while (vdev->name[0]) {
		if (!strncmp(vdev->name, idev->type, 16)) {
			/* compare name */
			type = vdev->type;
			port = vdev->port;
			num_lanes = vdev->num_lanes;
			break;
		}
		vdev++;
	}

	if (!type) /* not found */
		return;

	info = kzalloc(sizeof(struct intel_v4l2_subdev_i2c_board_info),
		       GFP_KERNEL);
	if (!info) {
		pr_err("fail to alloc mem for ignored i2c dev %s\n",
		       idev->type);
		return;
	}

	info->i2c_adapter_id = bus;
	/* set platform data */
	memcpy(&info->board_info, idev, sizeof(*idev));

	if (v4l2_subdev_table_head == NULL) {
		subdev_table = kzalloc(sizeof(struct intel_v4l2_subdev_table)
			* ARRAY_SIZE(v4l2_ids), GFP_KERNEL);

		if (!subdev_table) {
			pr_err("fail to alloc mem for v4l2_subdev_table %s\n",
			       idev->type);
			kfree(info);
			return;
		}

		v4l2_subdev_table_head = kzalloc(
			sizeof(struct atomisp_platform_data), GFP_KERNEL);
		if (!v4l2_subdev_table_head) {
			pr_err("fail to alloc mem for v4l2_subdev_table %s\n",
			       idev->type);
			kfree(info);
			kfree(subdev_table);
			return;
		}
		v4l2_subdev_table_head->subdevs = subdev_table;
	}

	memcpy(&subdev_table[i].v4l2_subdev, info, sizeof(*info));
	subdev_table[i].type = type;
	subdev_table[i].port = port;
	subdev_table[i].num_lanes = num_lanes;

	i++;
	return;
}


struct atomisp_platform_data *__intel_get_v4l2_subdev_table(void)
{
	if (v4l2_subdev_table_head)
		return v4l2_subdev_table_head;
	else {
		pr_err("no camera device in the table\n");
		return NULL;
	}
}
EXPORT_SYMBOL_GPL(__intel_get_v4l2_subdev_table);


static struct i2c_board_info i2c_board_mt9m114[] = {
	{
		I2C_BOARD_INFO(I2C_MT9M114_NAME,0x48),
	},
};

static struct i2c_board_info i2c_board_ov5640_1[] = {
	{
		I2C_BOARD_INFO(I2C_OV5640_1_NAME,0x32),
	},
};

static struct i2c_board_info i2c_board_ov5640_2[] = {
	{
		I2C_BOARD_INFO(I2C_OV5640_2_NAME,0x34),
	},
};



static int __init platform_init(void)
{

	i2c_board_mt9m114[0].platform_data = mt9m114_platform_data();
	intel_ignore_i2c_device_register(ANVL_I2C_BUS_NUM, i2c_board_mt9m114);

	i2c_board_ov5640_2[0].platform_data = ov5640_2_platform_data();
	intel_ignore_i2c_device_register(ANVL_I2C_BUS_NUM, i2c_board_ov5640_2);

	i2c_board_ov5640_1[0].platform_data = ov5640_1_platform_data();
	intel_ignore_i2c_device_register(ANVL_I2C_BUS_NUM, i2c_board_ov5640_1);

	return 0;
}
arch_initcall(platform_init);

MODULE_LICENSE("GPL");


