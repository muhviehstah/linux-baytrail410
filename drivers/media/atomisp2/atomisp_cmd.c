/*
 * Support for Medifield PNW Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 Intel Corporation. All Rights Reserved.
 *
 * Copyright (c) 2010 Silicon Hive www.siliconhive.com.
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
#ifdef ENABLE_DEV_DEBUG
#define DEBUG
#endif

#include <linux/firmware.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/pm_runtime.h>
#include <linux/timer.h>
#include <linux/delay.h>

#include <media/v4l2-event.h>

#include "atomisp_cmd.h"
#include "atomisp_common.h"
#include "atomisp_fops.h"
#include "atomisp_internal.h"
#include "atomisp_ioctl.h"
#include "atomisp-regs.h"
#include "atomisp_tables.h"
#include "atomisp_acc.h"
#include "atomisp_compat.h"

#include "hrt/hive_isp_css_mm_hrt.h"

#include "ia_css_debug.h"
#include "sh_css_hrt.h"
#include "sh_css_defs.h"
#include "system_global.h"
#include "sh_css_internal.h"
#include "sh_css_sp.h"
#include "gp_device.h"
#include "device_access.h"
#include "irq.h"

#include "ia_css_types.h"

#include "hrt/bits.h"


/* We should never need to run the flash for more than 2 frames.
 * At 15fps this means 133ms. We set the timeout a bit longer.
 * Each flash driver is supposed to set its own timeout, but
 * just in case someone else changed the timeout, we set it
 * here to make sure we don't damage the flash hardware. */
#define FLASH_TIMEOUT 800 /* ms */

/*
 * atomisp_kernel_malloc: chooses whether kmalloc() or vmalloc() is preferable.
 *
 * It is also a wrap functions to pass into css framework.
 */
void *atomisp_kernel_malloc(size_t bytes)
{
	/* vmalloc() is preferable if allocating more than 1 page */
	if (bytes > PAGE_SIZE)
		return vmalloc(bytes);

	return kmalloc(bytes, GFP_KERNEL);
}

/*
 * Free buffer allocated with atomisp_kernel_malloc()/atomisp_kernel_zalloc
 * helper
 */
void atomisp_kernel_free(void *ptr)
{
	/* Verify if buffer was allocated by vmalloc() or kmalloc() */
	if (is_vmalloc_addr(ptr))
		vfree(ptr);
	else
		kfree(ptr);
}

/*
 * get sensor:dis71430/ov2720 related info from v4l2_subdev->priv data field.
 * subdev->priv is set in mrst.c
 */
struct camera_mipi_info *atomisp_to_sensor_mipi_info(struct v4l2_subdev *sd)
{
	return (struct camera_mipi_info *)v4l2_get_subdev_hostdata(sd);
}

/*
 * get struct atomisp_video_pipe from v4l2 video_device
 */
struct atomisp_video_pipe *atomisp_to_video_pipe(struct video_device *dev)
{
	return (struct atomisp_video_pipe *)
	    container_of(dev, struct atomisp_video_pipe, vdev);
}

/*
 * get struct atomisp_sub_device from atomisp_video_pipe
 */
struct atomisp_sub_device *atomisp_to_sub_device(struct atomisp_video_pipe
						 *atomisp_pipe)
{
	return atomisp_pipe->isp_subdev;
}

/* This is just a draft rules, should be tuned when sensor is ready*/
static struct atomisp_freq_scaling_rule dfs_rules[] = {
	{
		.width = ISP_FREQ_RULE_ANY,
		.height = ISP_FREQ_RULE_ANY,
		.fps = ISP_FREQ_RULE_ANY,
		.isp_freq = ISP_FREQ_400MHZ,
		.run_mode = ATOMISP_RUN_MODE_VIDEO,
	},
	{
		.width = 1920,
		.height = 1080,
		.fps = 60,
		.isp_freq = ISP_FREQ_400MHZ,
		.run_mode = ATOMISP_RUN_MODE_VIDEO,
	},
	{
		.width = 4192,
		.height = 3104,
		.fps = ISP_FREQ_RULE_ANY,
		.isp_freq = ISP_FREQ_400MHZ,
		.run_mode = ATOMISP_RUN_MODE_STILL_CAPTURE,
	},
	{
		.width = 4096,
		.height = 3072,
		.fps = ISP_FREQ_RULE_ANY,
		.isp_freq = ISP_FREQ_400MHZ,
		.run_mode = ATOMISP_RUN_MODE_STILL_CAPTURE,
	},
	{
		.width = 3648,
		.height = 2736,
		.fps = ISP_FREQ_RULE_ANY,
		.isp_freq = ISP_FREQ_400MHZ,
		.run_mode = ATOMISP_RUN_MODE_STILL_CAPTURE,
	},
	{
		.width = ISP_FREQ_RULE_ANY,
		.height = ISP_FREQ_RULE_ANY,
		.fps = ISP_FREQ_RULE_ANY,
		.isp_freq = ISP_FREQ_400MHZ,
		.run_mode = ATOMISP_RUN_MODE_STILL_CAPTURE,
	},
	{
		.width = 1280,
		.height = 720,
		.fps = 60,
		.isp_freq = ISP_FREQ_320MHZ,
		.run_mode = ATOMISP_RUN_MODE_VIDEO,
	},
};

const struct atomisp_format_bridge atomisp_output_fmts[] = {
        {
                .pixelformat = V4L2_PIX_FMT_YUV420,
                .depth = 12,
                .mbus_code = 0x8001,
                .sh_fmt = IA_CSS_FRAME_FORMAT_YUV420,
                .description = "YUV420, planner"
        }, {
                .pixelformat = V4L2_PIX_FMT_YVU420,
                .depth = 12,
                .mbus_code = 0x8002,
                .sh_fmt = IA_CSS_FRAME_FORMAT_YV12,
                .description = "YVU420, planner"
        }, {
                .pixelformat = V4L2_PIX_FMT_YUV422P,
                .depth = 16,
                .mbus_code = 0x8003,
                .sh_fmt = IA_CSS_FRAME_FORMAT_YUV422,
                .description = "YUV422, planar"
        }, {
                .pixelformat = V4L2_PIX_FMT_YUV444,
                .depth = 24,
                .mbus_code = 0x8004,
                .sh_fmt = IA_CSS_FRAME_FORMAT_YUV444,
                .description = "YUV444"
        }, {
                .pixelformat = V4L2_PIX_FMT_NV12,
                .depth = 12,
                .mbus_code = 0x8005,
                .sh_fmt = IA_CSS_FRAME_FORMAT_NV12,
                .description = "NV12, interleaved"
        }, {
                .pixelformat = V4L2_PIX_FMT_NV21,
                .depth = 12,
                .mbus_code = 0x8006,
                .sh_fmt = IA_CSS_FRAME_FORMAT_NV21,
                .description = "NV21, interleaved"
        }, {
                .pixelformat = V4L2_PIX_FMT_NV16,
                .depth = 16,
                .mbus_code = 0x8007,
                .sh_fmt = IA_CSS_FRAME_FORMAT_NV16,
                .description = "NV16, interleaved"
        }, {
                .pixelformat = V4L2_PIX_FMT_YUYV,
                .depth = 16,
                .mbus_code = 0x8008,
                .sh_fmt = IA_CSS_FRAME_FORMAT_YUYV,
                .description = "YUYV, interleaved"
        }, {
                .pixelformat = V4L2_PIX_FMT_UYVY,
                .depth = 16,
                .mbus_code = MEDIA_BUS_FMT_UYVY8_1X16,
                .sh_fmt = IA_CSS_FRAME_FORMAT_UYVY,
                .description = "UYVY, interleaved"
        }, { /* This one is for parallel sensors! DO NOT USE! */
                .pixelformat = V4L2_PIX_FMT_UYVY,
                .depth = 16,
                .mbus_code = MEDIA_BUS_FMT_UYVY8_2X8,
                .sh_fmt = IA_CSS_FRAME_FORMAT_UYVY,
                .description = "UYVY, interleaved"
        }, {
                .pixelformat = V4L2_PIX_FMT_SBGGR16,
                .depth = 16,
                .mbus_code = 0x8009,
                .sh_fmt = IA_CSS_FRAME_FORMAT_RAW,
                .description = "Bayer 16"
        }, {
                .pixelformat = V4L2_PIX_FMT_SBGGR8,
                .depth = 8,
                .mbus_code = MEDIA_BUS_FMT_SBGGR8_1X8,
                .sh_fmt = IA_CSS_FRAME_FORMAT_RAW,
                .description = "Bayer 8"
        }, {
                .pixelformat = V4L2_PIX_FMT_SGBRG8,
                .depth = 8,
                .mbus_code = MEDIA_BUS_FMT_SGBRG8_1X8,
                .sh_fmt = IA_CSS_FRAME_FORMAT_RAW,
                .description = "Bayer 8"
        }, {
                .pixelformat = V4L2_PIX_FMT_SGRBG8,
                .depth = 8,
                .mbus_code = MEDIA_BUS_FMT_SGRBG8_1X8,
                .sh_fmt = IA_CSS_FRAME_FORMAT_RAW,
                .description = "Bayer 8"
        }, {
                .pixelformat = V4L2_PIX_FMT_SRGGB8,
                .depth = 8,
                .mbus_code = MEDIA_BUS_FMT_SRGGB8_1X8,
        }, {
                .pixelformat = V4L2_PIX_FMT_SBGGR10,
                .depth = 16,
                .mbus_code = MEDIA_BUS_FMT_SBGGR10_1X10,
                .sh_fmt = IA_CSS_FRAME_FORMAT_RAW,
                .description = "Bayer 10"
        }, {
                .pixelformat = V4L2_PIX_FMT_SGBRG10,
                .depth = 16,
                .mbus_code = MEDIA_BUS_FMT_SGBRG10_1X10,
                .sh_fmt = IA_CSS_FRAME_FORMAT_RAW,
                .description = "Bayer 10"
        }, {
                .pixelformat = V4L2_PIX_FMT_SGRBG10,
                .depth = 16,
                .mbus_code = MEDIA_BUS_FMT_SGRBG10_1X10,
                .sh_fmt = IA_CSS_FRAME_FORMAT_RAW,
                .description = "Bayer 10"
        }, {
                .pixelformat = V4L2_PIX_FMT_SRGGB10,
                .depth = 16,
                .mbus_code = MEDIA_BUS_FMT_SRGGB10_1X10,
                .sh_fmt = IA_CSS_FRAME_FORMAT_RAW,
                .description = "Bayer 10"
        }, {
                .pixelformat = V4L2_PIX_FMT_SBGGR12,
                .depth = 16,
                .mbus_code = MEDIA_BUS_FMT_SBGGR12_1X12,
                .sh_fmt = IA_CSS_FRAME_FORMAT_RAW,
                .description = "Bayer 12"
        }, {
                .pixelformat = V4L2_PIX_FMT_SGBRG12,
                .depth = 16,
                .mbus_code = MEDIA_BUS_FMT_SBGGR12_1X12,
                .sh_fmt = IA_CSS_FRAME_FORMAT_RAW,
                .description = "Bayer 12"
        }, {
                .pixelformat = V4L2_PIX_FMT_SGRBG12,
                .depth = 16,
                .mbus_code = MEDIA_BUS_FMT_SBGGR12_1X12,
                .sh_fmt = IA_CSS_FRAME_FORMAT_RAW,
                .description = "Bayer 12"
        }, {
                .pixelformat = V4L2_PIX_FMT_SRGGB12,
                .depth = 16,
                .mbus_code = MEDIA_BUS_FMT_SBGGR12_1X12,
                .sh_fmt = IA_CSS_FRAME_FORMAT_RAW,
                .description = "Bayer 12"
        }, {
                .pixelformat = V4L2_PIX_FMT_RGB32,
                .depth = 32,
                .mbus_code = 0x800a,
                .sh_fmt = IA_CSS_FRAME_FORMAT_RGBA888,
                .description = "32 RGB 8-8-8-8"
        }, {
                .pixelformat = V4L2_PIX_FMT_RGB24,
                .depth = 32,
                .mbus_code = 0x800b, /* TODO verify this MBUS code */
                .sh_fmt = IA_CSS_FRAME_FORMAT_RGBA888,
                .description = "24 RGB 8-8-8"
        }, {
                .pixelformat = V4L2_PIX_FMT_RGB565,
                .depth = 16,
                .mbus_code = MEDIA_BUS_FMT_RGB565_2X8_LE,
                .sh_fmt = IA_CSS_FRAME_FORMAT_RGB565,
                .description = "16 RGB 5-6-5"
        },
};

const struct atomisp_format_bridge *atomisp_get_format_bridge(
        unsigned int pixelformat)
{
        unsigned int i;

        for (i = 0; i < ARRAY_SIZE(atomisp_output_fmts); i++) {
                if (atomisp_output_fmts[i].pixelformat == pixelformat)
                        return &atomisp_output_fmts[i];
        }

        return NULL;
}

const struct atomisp_format_bridge *atomisp_get_format_bridge_from_mbus(
        enum media_bus_format mbus_code)
{
        unsigned int i;
        trace_printk("mbus code requested 0X%X\n", mbus_code);
        for (i = 0; i < ARRAY_SIZE(atomisp_output_fmts); i++) {
                if (mbus_code == atomisp_output_fmts[i].mbus_code)
                        return &atomisp_output_fmts[i];
        }

        return NULL;
}

int atomisp_enum_fmt_cap(struct file *file, void *fh,
        struct v4l2_fmtdesc *f)
{
        if (f->index >= ARRAY_SIZE(atomisp_output_fmts))
                return -EINVAL;

        f->pixelformat = atomisp_output_fmts[f->index].pixelformat;
        memset(f->description, 0, sizeof(f->description));
        strncpy(f->description, atomisp_output_fmts[f->index].description,
                strlen(atomisp_output_fmts[f->index].description));

        return 0;
}

#define ISP_FREQ_RULE_MAX (ARRAY_SIZE(dfs_rules))

static unsigned short atomisp_get_sensor_fps(struct atomisp_sub_device *isp_subdev)
{
	struct v4l2_subdev_frame_interval frame_interval;
	unsigned short fps;
	struct atomisp_device *isp = isp_subdev->isp;

	if (v4l2_subdev_call(isp->inputs[isp_subdev->input_curr].camera,
		video, g_frame_interval, &frame_interval)) {
		fps = 0;
	} else {
		if (frame_interval.interval.numerator)
			fps = frame_interval.interval.denominator /
			    frame_interval.interval.numerator;
		else
			fps = 0;
	}
	return fps;
}
/*
 * DFS progress is shown as follows:
 * 1. Target frequency is calculated according to FPS/Resolution/ISP running
 * mode.
 * 2. Ratio is calucated in formula: 2 * (HPLL / target frequency) - 1
 * 3. Set ratio to ISPFREQ40, 1 to FREQVALID and ISPFREQGUAR40
 *    to 200MHz in ISPSSPM1.
 * 4. Wait for FREQVALID to be cleared by P-Unit.
 * 5. Wait for field ISPFREQSTAT40 in ISPSSPM1 turn to ratio set in 3.
 */
static int write_target_freq_to_hw(int new_freq)
{
	int ratio, timeout;
	u32 isp_sspm1 = 0;
	isp_sspm1 = intel_mid_msgbus_read32(PUNIT_PORT, ISPSSPM1);
	if (isp_sspm1 & ISP_FREQ_VALID_MASK) {
		v4l2_dbg(6, dbg_level, &atomisp_dev,
			  "clearing ISPSSPM1 valid bit.\n");
		intel_mid_msgbus_write32(PUNIT_PORT, ISPSSPM1,
				    isp_sspm1 & ~(1 << ISP_FREQ_VALID_OFFSET));
	}

	ratio = 2 * (HPLL_FREQ / new_freq) - 1;
	isp_sspm1 = intel_mid_msgbus_read32(PUNIT_PORT, ISPSSPM1);
	isp_sspm1 &= ~(0x1F << ISP_REQ_FREQ_OFFSET);
	intel_mid_msgbus_write32(PUNIT_PORT, ISPSSPM1,
				   isp_sspm1
				   | ratio << ISP_REQ_FREQ_OFFSET
				   | 1 << ISP_FREQ_VALID_OFFSET
				   | 0xF << ISP_REQ_GUAR_FREQ_OFFSET);

	isp_sspm1 = intel_mid_msgbus_read32(PUNIT_PORT, ISPSSPM1);
	timeout = 10;
	while ((isp_sspm1 & ISP_FREQ_VALID_MASK) && timeout) {
		isp_sspm1 = intel_mid_msgbus_read32(PUNIT_PORT, ISPSSPM1);
		v4l2_dbg(6, dbg_level, &atomisp_dev,
			"waiting for ISPSSPM1 valid bit to be 0.\n");
		udelay(100);
		timeout--;
	}
	if (timeout == 0) {
		v4l2_err(&atomisp_dev, "DFS failed due to HW error.\n");
		return -EINVAL;
	}

	isp_sspm1 = intel_mid_msgbus_read32(PUNIT_PORT, ISPSSPM1);
	timeout = 10;
	while (((isp_sspm1 >> ISP_FREQ_STAT_OFFSET) != ratio) && timeout) {
		isp_sspm1 = intel_mid_msgbus_read32(PUNIT_PORT, ISPSSPM1);
		v4l2_dbg(6, dbg_level, &atomisp_dev,
				"waiting for ISPSSPM1 status bit to be 0x%x.\n",
				 new_freq);
		udelay(100);
		timeout--;
	}
	if (timeout == 0) {
		v4l2_warn(&atomisp_dev, "DFS target freq is rejected by HW.\n");
		return -EINVAL;
	}

	return 0;
}
int atomisp_freq_scaling(struct atomisp_device *isp, enum atomisp_dfs_mode mode)
{
	unsigned int new_freq;
	struct atomisp_freq_scaling_rule curr_rules;
	int i, ret;
	unsigned short fps = 0;
	struct atomisp_sub_device *isp_subdev = NULL;

	if (isp->sw_contex.power_state != ATOM_ISP_POWER_UP) {
		v4l2_err(&atomisp_dev, "DFS cannot proceed due to no power.\n");
		return -EINVAL;
	}

	/* ISP will run at full speed in multi stream mode */
	if (atomisp_subdev_streaming_count(isp) > 1) {
		new_freq = ISP_FREQ_400MHZ;
		goto done;
	}

	if (mode == ATOMISP_DFS_MODE_LOW) {
		new_freq = ISP_FREQ_200MHZ;
		goto done;
	}

	if (mode == ATOMISP_DFS_MODE_MAX) {
		new_freq = ISP_FREQ_400MHZ;
		goto done;
	}

	/* check which stream is enabled */
	for (i = 0; i < isp->num_of_streams; i++)
		if (isp->isp_subdev[i].streaming ==
		    ATOMISP_DEVICE_STREAMING_ENABLED) {
			isp_subdev = &isp->isp_subdev[i];
			break;
		}

	if (!isp_subdev) {
		dev_err(isp->dev,
			"DFS auto mode can not be done due to no streaming.\n");
		return -EINVAL;
	}

	fps = atomisp_get_sensor_fps(isp_subdev);
	if (fps == 0)
		return -EINVAL;

	curr_rules.width = isp_subdev->fmt[isp_subdev->capture_pad].fmt.width;
	curr_rules.height = isp_subdev->fmt[isp_subdev->capture_pad].fmt.height;
	curr_rules.fps = fps;
	curr_rules.run_mode = isp_subdev->run_mode->val;
	/*
	 * For continuous vf mode, we need to make the capture setting applied
	 * since preview mode, because there is no chance to do this when
	 * starting image capture.
	 */

	if (isp_subdev->params.continuous_vf)
		curr_rules.run_mode = ATOMISP_RUN_MODE_STILL_CAPTURE;

	/* search for the target frequency by looping freq rules*/
	for (i = 0; i < ISP_FREQ_RULE_MAX; i++) {
		if (curr_rules.width != dfs_rules[i].width
			&& dfs_rules[i].width != ISP_FREQ_RULE_ANY)
			continue;
		if (curr_rules.height != dfs_rules[i].height
			&& dfs_rules[i].height != ISP_FREQ_RULE_ANY)
			continue;
		if (curr_rules.fps != dfs_rules[i].fps
			&& dfs_rules[i].fps != ISP_FREQ_RULE_ANY)
			continue;
		if (curr_rules.run_mode != dfs_rules[i].run_mode
			&& dfs_rules[i].run_mode != ISP_FREQ_RULE_ANY)
			continue;
		break;
	}
	if (i == ISP_FREQ_RULE_MAX)
		new_freq = ISP_FREQ_320MHZ;
	else
		new_freq = dfs_rules[i].isp_freq;

done:
	/* workround to get isp works at 400Mhz for byt due to perf issue */
	if (IS_MRFLD)
		new_freq = ISP_FREQ_400MHZ;

	v4l2_dbg(3, dbg_level, &atomisp_dev,
		 "DFS target frequency=%d.\n", new_freq);
	if (new_freq == isp->sw_contex.running_freq) {
		v4l2_dbg(3, dbg_level, &atomisp_dev,
			 "ignoring DFS target freq.\n");
		return 0;
	}
	ret = write_target_freq_to_hw(new_freq);
	if (!ret)
		isp->sw_contex.running_freq = new_freq;
	return ret;
}

static void switch_to_full_firmware(struct atomisp_device *isp)
{
	if (fastboot)
	{
                if(isp->firmware_switched == false)
        	{
	                /*
		         * Function calls should implement their own locks
                	*/
                	release_firmware(isp->firmware);
                	isp->firmware = NULL;
                	isp->firmware_load_complete = false;
                	/*Wait for the big firmware to load*/

                	/* Load isp firmware from user space */
                	/*Switch the pointers*/
               		isp->firmware = isp->aux_firmware;
                	isp->firmware_switched = true;
                	if(isp->firmware != NULL)
                	{
	                	//Fixing the issue where insmod failed because firmware data is accessed before we even check whether firmware load completed or not
                        	isp->css_fw.data = (void *)isp->firmware->data;
                        	isp->css_fw.bytes = isp->firmware->size;
                	}
                	isp->firmware_load_complete = true;
                	wake_up_interruptible(&atomisp_wait_queue);
        	}

        }
}

/*
 * reset and restore ISP
 */
int atomisp_reset(struct atomisp_device *isp)
{
	/* Reset ISP by power-cycling it */
	int ret = 0;
	v4l2_dbg(2, dbg_level, &atomisp_dev, "%s\n",__func__);

	if (isp->switch_fw_on_streamoff == true) ia_css_unload_firmware(true);

	ia_css_uninit();
	ret = pm_runtime_put_sync(isp->dev);

	if (ret < 0) {
		v4l2_err(&atomisp_dev, "can not disable ISP power\n");
	} else {
		ret = pm_runtime_get_sync(isp->dev);
		if (ret < 0)
			v4l2_err(&atomisp_dev, "can not enable ISP power\n");
	}

        /*
        *Switch firmware if flag set during ioctl
        */
        if (isp->switch_fw_on_streamoff == true) {
		switch_to_full_firmware(isp);
                v4l2_dbg(2, dbg_level, &atomisp_dev, "Firmware switch Complete. Also switched to BUFFERED_SENSOR Mode for multistreaming\n");
                isp->switch_fw_on_streamoff = false;
	}
	ret = atomisp_css_init(isp);

	return ret;
}

#ifdef ISP_IRQ_HELPER
/*
 * interrupt enable/disable functions
 */
static void enable_isp_irq(enum hrt_isp_css_irq irq, bool enable)
{
	if (enable) {
		irq_enable_channel(IRQ0_ID, irq);
		/*sh_css_hrt_irq_enable(irq, true, false);*/
		switch (irq) { /*We only have sp interrupt right now*/
		case hrt_isp_css_irq_sp:
			/*sh_css_hrt_irq_enable_sp(true);*/
			cnd_sp_irq_enable(SP0_ID, true);
			break;
		default:
			break;
		}

	} else {
		/*sh_css_hrt_irq_disable(irq);*/
		irq_disable_channel(IRQ0_ID, irq);
		switch (irq) {
		case hrt_isp_css_irq_sp:
			/*sh_css_hrt_irq_enable_sp(false);*/
			cnd_sp_irq_enable(SP0_ID, false);
			break;
		default:
			break;
		}
	}
}

/*
 * interrupt clean function
 */
static void clear_isp_irq(enum hrt_isp_css_irq irq)
{
	irq_clear_all(IRQ0_ID);
}
#endif

void atomisp_msi_irq_init(struct atomisp_device *isp, struct pci_dev *dev)
{
	u32 msg32;
	u16 msg16;

	pci_read_config_dword(dev, PCI_MSI_CAPID, &msg32);
	msg32 |= 1 << MSI_ENABLE_BIT;
	pci_write_config_dword(dev, PCI_MSI_CAPID, msg32);

	msg32 = (1 << INTR_IER) | (1 << INTR_IIR);
	pci_write_config_dword(dev, PCI_INTERRUPT_CTRL, msg32);

	pci_read_config_word(dev, PCI_COMMAND, &msg16);
	msg16 |= (PCI_COMMAND_MEMORY |
		  PCI_COMMAND_MASTER |
		  PCI_COMMAND_INTX_DISABLE);
	pci_write_config_word(dev, PCI_COMMAND, msg16);
}

void atomisp_msi_irq_uninit(struct atomisp_device *isp, struct pci_dev *dev)
{
	u32 msg32;
	u16 msg16;

	pci_read_config_dword(dev, PCI_MSI_CAPID, &msg32);
	msg32 &=  ~(1 << MSI_ENABLE_BIT);
	pci_write_config_dword(dev, PCI_MSI_CAPID, msg32);

	msg32 = 0x0;
	pci_write_config_dword(dev, PCI_INTERRUPT_CTRL, msg32);

	pci_read_config_word(dev, PCI_COMMAND, &msg16);
	msg16 &= ~(PCI_COMMAND_MASTER |
		   PCI_COMMAND_INTX_DISABLE);
	pci_write_config_word(dev, PCI_COMMAND, msg16);
}

static void atomisp_sof_event(struct atomisp_sub_device *isp_subdev)
{
	struct v4l2_event event;

	event.type = V4L2_EVENT_FRAME_SYNC;
	event.u.frame_sync.frame_sequence = atomic_read(&isp_subdev->sof_count);

	if(isp_subdev->subdev.devnode != NULL)
		v4l2_event_queue(isp_subdev->subdev.devnode, &event);
}

static void atomisp_3a_stats_ready_event(struct atomisp_sub_device *isp_subdev)
{
	struct v4l2_event event = {0};

	event.type = V4L2_EVENT_ATOMISP_3A_STATS_READY;
	event.u.frame_sync.frame_sequence = atomic_read(&isp_subdev->sequence);

	v4l2_event_queue(isp_subdev->subdev.devnode, &event);
}

static void print_csi_rx_errors(struct atomisp_device *isp)
{
	u32 infos = 0;

	ia_css_rx_get_irq_info(&infos);

	dev_err(isp->dev, "CSI Receiver errors:\n");
	if (infos & IA_CSS_RX_IRQ_INFO_BUFFER_OVERRUN)
		dev_err(isp->dev, "  buffer overrun");
	if (infos & IA_CSS_RX_IRQ_INFO_ERR_SOT)
		dev_err(isp->dev, "  start-of-transmission error");
	if (infos & IA_CSS_RX_IRQ_INFO_ERR_SOT_SYNC)
		dev_err(isp->dev, "  start-of-transmission sync error");
	if (infos & IA_CSS_RX_IRQ_INFO_ERR_CONTROL)
		dev_err(isp->dev, "  control error");
	if (infos & IA_CSS_RX_IRQ_INFO_ERR_ECC_DOUBLE)
		dev_err(isp->dev, "  2 or more ECC errors");
	if (infos & IA_CSS_RX_IRQ_INFO_ERR_CRC)
		dev_err(isp->dev, "  CRC mismatch");
	if (infos & IA_CSS_RX_IRQ_INFO_ERR_UNKNOWN_ID)
		dev_err(isp->dev, "  unknown error");
	if (infos & IA_CSS_RX_IRQ_INFO_ERR_FRAME_SYNC)
		dev_err(isp->dev, "  frame sync error");
	if (infos & IA_CSS_RX_IRQ_INFO_ERR_FRAME_DATA)
		dev_err(isp->dev, "  frame data error");
	if (infos & IA_CSS_RX_IRQ_INFO_ERR_DATA_TIMEOUT)
		dev_err(isp->dev, "  data timeout");
	if (infos & IA_CSS_RX_IRQ_INFO_ERR_UNKNOWN_ESC)
		dev_err(isp->dev, "  unknown escape command entry");
	if (infos & IA_CSS_RX_IRQ_INFO_ERR_LINE_SYNC)
		dev_err(isp->dev, "  line sync error");
}

/* interrupt handling function*/
irqreturn_t atomisp_isr(int irq, void *dev)
{
	u32 msg_ret;
	struct atomisp_device *isp = (struct atomisp_device *)dev;
	unsigned int irq_infos = 0;
	unsigned long flags;
	int err;
	int i, streaming;

	err = ia_css_irq_translate(&irq_infos);
	v4l2_dbg(5, dbg_level, &atomisp_dev, "ENTER atomisp_isr, irq:0x%x\n", irq_infos);

	if (err != IA_CSS_SUCCESS) {
		v4l2_warn(&atomisp_dev, "%s:failed to translate irq (err = %d,"
			  " infos = %d)\n", __func__, err, irq_infos);
		v4l2_err(&atomisp_dev, "ERROR ia_css_irq_translate failed\n");
		return IRQ_NONE;
	}

	/* Clear irq reg at PENWELL B0 */
	pci_read_config_dword(isp->pdev, PCI_INTERRUPT_CTRL, &msg_ret);
	msg_ret |= 1 << INTR_IIR;
	pci_write_config_dword(isp->pdev, PCI_INTERRUPT_CTRL, msg_ret);

	spin_lock_irqsave(&isp->lock, flags);

	for (i = 0, streaming = 0; i < isp->num_of_streams; i++)
		streaming += isp->isp_subdev[i].streaming ==
		    ATOMISP_DEVICE_STREAMING_ENABLED;
	if (!streaming)
 		goto out_nowake;

	for(i = 0; i < isp->num_of_streams; i++) {
		struct atomisp_sub_device *isp_subdev = &isp->isp_subdev[i];

		if (irq_infos & IA_CSS_IRQ_INFO_CSS_RECEIVER_SOF) {
			atomic_inc(&isp_subdev->sof_count);
			atomisp_sof_event(isp_subdev);

		/* If sequence_temp and sequence are the same
		 * there where no frames lost so we can increase sequence_temp.
		 * If not then processing of frame is still in progress and
		 * driver needs to keep old sequence_temp value.
		 * NOTE: There is assumption here that ISP will not start
		 * processing next frame from sensor before old one is
		 * completely done. */
			if (atomic_read(&isp_subdev->sequence) == atomic_read(
						&isp_subdev->sequence_temp))
				atomic_set(&isp_subdev->sequence_temp,
						atomic_read(&isp_subdev->sof_count));

			/* signal streamon after delayed init is done */
			if (isp_subdev->delayed_init == ATOMISP_DELAYED_INIT_WORK_DONE) {
				isp_subdev->delayed_init = ATOMISP_DELAYED_INIT_DONE;
				complete(&isp_subdev->init_done);
			}
		}

		if (irq_infos & IA_CSS_IRQ_INFO_EVENTS_READY)
			atomic_set(&isp_subdev->sequence,
					atomic_read(&isp_subdev->sequence_temp));
	}

	if ((irq_infos & IA_CSS_IRQ_INFO_INPUT_SYSTEM_ERROR) ||
		(irq_infos & IA_CSS_IRQ_INFO_IF_ERROR)) {
		/* handle mipi receiver error */
		u32 rx_infos;

		print_csi_rx_errors(isp);
		ia_css_rx_get_irq_info(&rx_infos);
		ia_css_rx_clear_irq_info(rx_infos);
		/* TODO: handle SH_CSS_RX_IRQ_INFO_BUFFER_OVERRUN */
	}

	spin_unlock_irqrestore(&isp->lock, flags);

	return IRQ_WAKE_THREAD;

out_nowake:
	spin_unlock_irqrestore(&isp->lock, flags);

	return IRQ_HANDLED;
}

/*
 * Background:
 * The IUNITPHY register CSI_CONTROL bit definition was changed since PNW C0.
 * For PNW A0 and B0, CSI4_TERM_EN_COUNT is bit 23:20 (4 bits).
 * Starting from PWN C0, including all CLV and CLV+ steppings,
 * CSI4_TERM_EN_COUNT is bit 30:24 (7 bits).
 *
 * ------------------------------------------
 * Silicon	Stepping	PCI revision
 * Penwell	A0		0x00
 * Penwell	B0		0x04
 * Penwell	C0		0x06
 * Penwell	D0		0x06
 * Penwell	D1		0x06
 * Penwell	D2		0x06
 * Cloverview	A0		0x06
 * Cloverview	B0		0x05
 * Cloverview	C0		0x04
 * Cloverview+	A0		0x08
 * Cloverview+	B0		0x0C
 *
 */

#define TERM_EN_COUNT_1LANE_OFFSET		16	/* bit 22:16 */
#define TERM_EN_COUNT_1LANE_MASK		0x7f0000
#define TERM_EN_COUNT_4LANE_OFFSET		24	/* bit 30:24 */
#define TERM_EN_COUNT_4LANE_MASK		0x7f000000
#define TERM_EN_COUNT_4LANE_PWN_B0_OFFSET	20	/* bit 23:20 */
#define TERM_EN_COUNT_4LANE_PWN_B0_MASK		0xf00000

void atomisp_set_term_en_count(struct atomisp_device *isp)
{
	uint32_t val;
	int pwn_b0 = 0;

	/* For MRFLD, there is no Tescape-clock cycles control. */
	if (IS_MRFLD)
		return;

	/*if (isp->pdev->device == 0x0148 && isp->pdev->revision < 0x6 &&
		IS_MRFLD)
		pwn_b0 = 1;*/

	val = intel_mid_msgbus_read32(MFLD_IUNITPHY_PORT, MFLD_CSI_CONTROL);

	/* set TERM_EN_COUNT_1LANE to 0xf */
	val &= ~TERM_EN_COUNT_1LANE_MASK;
	val |= 0xf << TERM_EN_COUNT_1LANE_OFFSET;

	/* set TERM_EN_COUNT_4LANE to 0xf */
	val &= pwn_b0 ? ~TERM_EN_COUNT_4LANE_PWN_B0_MASK :
				~TERM_EN_COUNT_4LANE_MASK;
	val |= 0xf << (pwn_b0 ? TERM_EN_COUNT_4LANE_PWN_B0_OFFSET :
				TERM_EN_COUNT_4LANE_OFFSET);

	intel_mid_msgbus_write32(MFLD_IUNITPHY_PORT, MFLD_CSI_CONTROL, val);
}

void atomisp_clear_css_buffer_counters(struct atomisp_sub_device *isp_subdev)
{
	memset(isp_subdev->s3a_bufs_in_css, 0, sizeof(isp_subdev->s3a_bufs_in_css));
	isp_subdev->dis_bufs_in_css = 0;
	isp_subdev->video_out_capture.buffers_in_css = 0;
	isp_subdev->video_out_vf.buffers_in_css = 0;
	isp_subdev->video_out_preview.buffers_in_css = 0;
}

void atomisp_clear_frame_counters(struct atomisp_sub_device *isp_subdev)
{
        isp_subdev->video_out_capture.field_sequence = 0;
        isp_subdev->video_out_vf.field_sequence = 0;
        isp_subdev->video_out_preview.field_sequence = 0;
        isp_subdev->video_out_capture.previous_frame_exp_id = 0;
        isp_subdev->video_out_vf.previous_frame_exp_id = 0;
        isp_subdev->video_out_preview.previous_frame_exp_id = 0;
}

/* return total number of buffers in css */
static int __buffers_in_css(struct atomisp_sub_device *isp_subdev)
{
	int i;
	int sum = 0;

	for (i = 0; i < IA_CSS_PIPE_ID_NUM; i++)
		sum += isp_subdev->s3a_bufs_in_css[i];

	sum += isp_subdev->dis_bufs_in_css;
	sum += isp_subdev->video_out_capture.buffers_in_css;
	sum += isp_subdev->video_out_vf.buffers_in_css;
	sum += isp_subdev->video_out_preview.buffers_in_css;

	return sum;
}

bool atomisp_buffers_queued(struct atomisp_sub_device *isp_subdev)
{
	return isp_subdev->video_out_capture.buffers_in_css ||
		isp_subdev->video_out_vf.buffers_in_css ||
		isp_subdev->video_out_preview.buffers_in_css ?
		    true : false;
}

/* 0x100000 is the start of dmem inside SP */
#define SP_DMEM_BASE	0x100000
#if 0
void dump_sp_dmem(unsigned int addr, unsigned int size)
{
	unsigned int data = 0;
	unsigned int size32 = (size + (sizeof(u32) - 1)) / sizeof(u32);
	v4l2_dbg(3, dbg_level, &atomisp_dev,
		 "atomisp_io_base:0x%x\n",(unsigned int)atomisp_io_base);
	v4l2_dbg(3, dbg_level, &atomisp_dev,
		 "%s, addr:0x%x, size: %d, size32: %d\n",
		 __func__, addr, size, size32);
	if (size32 * 4 + addr > 0x4000) {
		v4l2_err(&atomisp_dev,
			 "illegal size (%d) or addr (0x%x)\n",
			 size32, addr);
		return;
	}
	addr += SP_DMEM_BASE;
	do {
		data = _hrt_master_port_uload_32(addr);

		/* printk/dtrace */
#if 1
		v4l2_dbg(3, dbg_level, &atomisp_dev,
			 "%s, \t [0x%x]:0x%x\n",
			 __func__, addr, data);
#else
		atomisp_dtrace(2,"%s, \t [0x%x]:0x%x\n",
				__func__, addr, data);
#endif
		addr += sizeof(unsigned int);
		size32 -= 1;
	} while(size32 > 0);
}
#endif

static bool is_NTSC_format(struct ia_css_frame *frame)
{
	return frame->info.res.height==240?true:false;
}

static enum v4l2_field get_field_type(struct ia_css_frame *frame)
{
	if(frame->frame_nr == 1)
		return is_NTSC_format(frame)?V4L2_FIELD_BOTTOM:V4L2_FIELD_TOP;
	else if(frame->frame_nr == 2)
		return is_NTSC_format(frame)?V4L2_FIELD_TOP:V4L2_FIELD_BOTTOM;
	return V4L2_FIELD_NONE;
}

static struct atomisp_vb2 *atomisp_css_frame_to_vb2(
	struct atomisp_video_pipe *pipe, struct ia_css_frame *frame)
{
	struct atomisp_vb2 *buf;
	int i;

	for (i = 0; pipe->vb2q.bufs[i]; i++) {
		buf = container_of(pipe->vb2q.bufs[i], struct atomisp_vb2, vb);
		if (buf->cssframe == frame){
			if (pipe->pix.field ==V4L2_FIELD_ALTERNATE){
				buf->vb.v4l2_buf.field = get_field_type(frame);
				if ((buf->vb.v4l2_buf.field == V4L2_FIELD_TOP && !is_NTSC_format(frame)) || (buf->vb.v4l2_buf.field == V4L2_FIELD_BOTTOM && is_NTSC_format(frame))){
                                                buf->vb.v4l2_buf.sequence = ++pipe->field_sequence;
				}
				else if ((buf->vb.v4l2_buf.field == V4L2_FIELD_BOTTOM && !is_NTSC_format(frame)) || (buf->vb.v4l2_buf.field == V4L2_FIELD_TOP && is_NTSC_format(frame))){
					int exp_id_diff = frame->exp_id - pipe->previous_frame_exp_id - 1;
					if( exp_id_diff == 0 || exp_id_diff == -EXP_ID_WRAPAROUND)
						buf->vb.v4l2_buf.sequence = pipe->field_sequence;
					else
						buf->vb.v4l2_buf.sequence = pipe->field_sequence + exp_id_diff>0?exp_id_diff:(exp_id_diff + EXP_ID_WRAPAROUND);
				}
				pipe->previous_frame_exp_id = frame->exp_id;
				v4l2_dbg(4, dbg_level, &atomisp_dev, "Buffer sequence = %d, Buffer exp_id = %d, Buffer field = %d\n", buf->vb.v4l2_buf.sequence, frame->exp_id, buf->vb.v4l2_buf.field);
			}
			return buf;
		}
	}

	v4l2_dbg(6, dbg_level, &atomisp_dev, "DMA ERROR return NULL from atomisp_css_frame_to_vb2\n");
	return NULL;
}

static void get_buf_timestamp(struct timeval *tv)
{
	/* This function is duplicated from v4l2_get_timestamp */
	struct timespec ts;
	ktime_get_ts(&ts);
	tv->tv_sec = ts.tv_sec;
	tv->tv_usec = ts.tv_nsec / NSEC_PER_USEC;
}

/* Returns queued buffers back to video-core */
void atomisp_flush_bufs_and_wakeup(struct atomisp_sub_device *isp_subdev)
{
}


/* find atomisp_video_pipe with css pipe id, buffer type and atomisp run_mode */
static struct atomisp_video_pipe *__atomisp_get_pipe(struct atomisp_sub_device *isp_subdev,
		enum ia_css_pipe_id css_pipe_id,
		enum ia_css_buffer_type buf_type)
{
	/* video is same in online as in continuouscapture mode */
	if (!isp_subdev->enable_vfpp->val) {
		return &isp_subdev->video_out_capture;
	} else if (isp_subdev->run_mode->val == ATOMISP_RUN_MODE_VIDEO) {
		if (buf_type == IA_CSS_BUFFER_TYPE_OUTPUT_FRAME)
			return &isp_subdev->video_out_capture;
		else if(buf_type == IA_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME)
	        return &isp_subdev->video_out_vf;
		return &isp_subdev->video_out_preview;
	} else if (buf_type == IA_CSS_BUFFER_TYPE_OUTPUT_FRAME) {
		if (css_pipe_id == IA_CSS_PIPE_ID_PREVIEW)
			return &isp_subdev->video_out_preview;
		return &isp_subdev->video_out_capture;
	/* statistic buffers are needed only in css capture & preview pipes */
	} else if (buf_type == IA_CSS_BUFFER_TYPE_3A_STATISTICS ||
		   buf_type == IA_CSS_BUFFER_TYPE_DIS_STATISTICS) {
		if (css_pipe_id == IA_CSS_PIPE_ID_PREVIEW)
			return &isp_subdev->video_out_preview;
		return &isp_subdev->video_out_capture;
	}
	return &isp_subdev->video_out_vf;
}

static void atomisp_buf_done(struct atomisp_sub_device *isp_subdev, int error,
			enum ia_css_buffer_type buf_type,
			enum ia_css_pipe_id css_pipe_id, bool q_buffers)
{
	struct atomisp_vb2 *vb2 = NULL;
	struct atomisp_video_pipe *pipe = NULL;
	struct ia_css_buffer buffer;
	bool requeue = false;
	int err, pipe_index;
	struct ia_css_frame *frame = NULL;
	struct atomisp_device *isp = isp_subdev->isp;
	v4l2_dbg(6, dbg_level, &atomisp_dev, "DMA ENTER atomisp_buf_done\n");

	if (buf_type != IA_CSS_BUFFER_TYPE_3A_STATISTICS &&
	    buf_type != IA_CSS_BUFFER_TYPE_DIS_STATISTICS &&
	    buf_type != IA_CSS_BUFFER_TYPE_OUTPUT_FRAME &&
	    buf_type != IA_CSS_BUFFER_TYPE_RAW_OUTPUT_FRAME &&
	    buf_type != IA_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME) {
		v4l2_err(&atomisp_dev,
			"%s, unsupported buffer type: %d\n",
			__func__, buf_type);
		return;
	}

	memset(&buffer, 0, sizeof(struct ia_css_buffer));
	buffer.type = buf_type;
	pipe_index = (unsigned int)css_pipe_id;
	err = ia_css_pipe_dequeue_buffer(isp_subdev->css2_basis.pipes[pipe_index],
					&buffer);
	if (err){
		v4l2_err(&atomisp_dev,
			"sh_css_dequeue_buffer failed: 0x%x\n",
			err);
		return;
	}

	/* need to know the atomisp pipe for frame buffers */
	pipe = __atomisp_get_pipe(isp_subdev, css_pipe_id,
				  buffer.type);
	if (pipe == NULL) {
		dev_err(isp->dev, "error getting atomisp pipe\n");
		return;
	}

	switch (buf_type) {
	case IA_CSS_BUFFER_TYPE_3A_STATISTICS:
		/* ignore error in case of 3a statistics for now */
		if (isp->sw_contex.invalid_s3a) {
			requeue = true;
			isp->sw_contex.invalid_s3a = 0;
			break;
		}
		/* update the 3A data to ISP context */
		if (isp_subdev->params.s3a_user_stat &&
			isp_subdev->params.s3a_output_bytes && !error) {
			/* To avoid racing with atomisp_3a_stat() */
			ia_css_get_3a_statistics(isp_subdev->params.s3a_user_stat,
						 buffer.data.stats_3a);
			isp_subdev->params.s3a_buf_data_valid = true;
		}
		isp_subdev->s3a_bufs_in_css[css_pipe_id]--;
		atomisp_3a_stats_ready_event(isp_subdev);
		break;
	case IA_CSS_BUFFER_TYPE_DIS_STATISTICS:
		/* ignore error in case of dis statistics for now */
		if (isp->sw_contex.invalid_dis) {
			requeue = true;
			isp->sw_contex.invalid_dis = 0;
			break;
		}
		if (isp_subdev->params.dvs_stat && !error) {
			/* To avoid racing with atomisp_get_dis_stat()*/
			ia_css_get_dvs_statistics(isp_subdev->params.dvs_stat,
						  buffer.data.stats_dvs);

			isp_subdev->params.dvs_proj_data_valid = true;
		}
		isp_subdev->dis_bufs_in_css--;
		break;
	case IA_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME:
		if (!buffer.data.frame->valid) {
			error = true;
			/*isp->sw_contex.invalid_vf_frame = 0;*/
			v4l2_dbg(3, dbg_level, &atomisp_dev,
				 "%s css has marked this vf frame as invalid\n",
				 __func__);
		}

		pipe->buffers_in_css--;
		frame = buffer.data.frame;
		if (isp_subdev->params.flash_state == ATOMISP_FLASH_ONGOING) {
			if (frame->flash_state
			    == IA_CSS_FRAME_FLASH_STATE_PARTIAL)
				v4l2_dbg(3, dbg_level, &atomisp_dev,
					 "%s thumb partially flashed\n", __func__);
			else if (frame->flash_state
				 == IA_CSS_FRAME_FLASH_STATE_FULL)
				v4l2_dbg(3, dbg_level, &atomisp_dev,
					 "%s thumb completely flashed\n", __func__);
			else
				v4l2_dbg(3, dbg_level, &atomisp_dev,
					 "%s thumb no flash in this frame\n", __func__);
		}
		v4l2_dbg(4, dbg_level, &atomisp_dev, "IA_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME, about to call atomisp_css_frame_to_vb2\n");
		vb2 = atomisp_css_frame_to_vb2(pipe, buffer.data.frame);
		if (!vb2)
			v4l2_dbg(4, dbg_level, &atomisp_dev, "ERROR dequeued frame unknown!");
		break;
	case IA_CSS_BUFFER_TYPE_OUTPUT_FRAME:
		if (!buffer.data.frame->valid) {
			error = true;
			/*isp->sw_contex.invalid_frame = 0;*/
			v4l2_dbg(3, dbg_level, &atomisp_dev,
				 "%s css has marked this frame as invalid\n",
				 __func__);
		}
		pipe->buffers_in_css--;
		v4l2_dbg(6, dbg_level, &atomisp_dev, "DMA CSS_BUFFER_TYPE_OUTPUT, buffers_in_css--=%d, about to call atomisp_css_frame_to_vb2\n", pipe->buffers_in_css);
		vb2 = atomisp_css_frame_to_vb2(pipe, buffer.data.frame);
		frame = buffer.data.frame;
		if (!vb2) {
			v4l2_dbg(4, dbg_level, &atomisp_dev, "ERROR dequeued output frame NULL!\n");
			break;
		}
		v4l2_dbg(6, dbg_level, &atomisp_dev, "setting isp->frame_status to OK\n");
		if (isp_subdev->params.flash_state == ATOMISP_FLASH_ONGOING) {
			if (frame->flash_state
			    == IA_CSS_FRAME_FLASH_STATE_PARTIAL) {
				isp_subdev->frame_status[vb2->vb.v4l2_buf.index] =
				    ATOMISP_FRAME_STATUS_FLASH_PARTIAL;
				v4l2_dbg(3, dbg_level, &atomisp_dev,
					 "%s partially flashed\n",
					 __func__);
			} else if (frame->flash_state
				   == IA_CSS_FRAME_FLASH_STATE_FULL) {
				isp_subdev->frame_status[vb2->vb.v4l2_buf.index] =
				    ATOMISP_FRAME_STATUS_FLASH_EXPOSED;
				isp_subdev->params.num_flash_frames--;
				v4l2_dbg(3, dbg_level, &atomisp_dev,
					 "%s completely flashed\n",
					 __func__);
			} else {
				isp_subdev->frame_status[vb2->vb.v4l2_buf.index] =
				    ATOMISP_FRAME_STATUS_OK;
				v4l2_dbg(3, dbg_level, &atomisp_dev,
					 "%s no flash in this frame\n",
					 __func__);
			}

			/* Check if flashing sequence is done */
			if (isp_subdev->frame_status[vb2->vb.v4l2_buf.index] == ATOMISP_FRAME_STATUS_FLASH_EXPOSED)
				isp_subdev->params.flash_state = ATOMISP_FLASH_DONE;
		} else {
			isp_subdev->frame_status[vb2->vb.v4l2_buf.index] = ATOMISP_FRAME_STATUS_OK;
		}

		if (!vb2)
			v4l2_err(&atomisp_dev, "dequeued frame unknown!\n");
		else
			isp_subdev->params.last_frame_status = isp_subdev->frame_status[vb2->vb.v4l2_buf.index];
		break;
	default:
		break;
	}

	if (vb2) {
		get_buf_timestamp(&vb2->vb.v4l2_buf.timestamp);
		v4l2_dbg(6, dbg_level, &atomisp_dev, "DMA calling vb2_buffer_done, buf index=%d, fd=%d, VB2_BUF_STATE=%d (should be active=3), vaddr=%X, length=%u\n", vb2->vb.v4l2_buf.index,vb2->vb.v4l2_buf.m.fd, vb2->vb.state, vb2->cssframe->data, vb2->cssframe->data_bytes);
		vb2_buffer_done(&vb2->vb, VB2_BUF_STATE_DONE);
		v4l2_dbg(6, dbg_level, &atomisp_dev, "DMA Finished, now VB2_BUF_STATE=%d (should be DONE=4)\n", vb2->vb.state);
	}	else {
		v4l2_dbg(6, dbg_level, &atomisp_dev, "ERROR atomisp_buf_done vb2 is NULL\n");
	}

	/*
	 * Requeue should only be done for 3a and dis buffers.
	 * Queue/dequeue order will change if driver recycles image buffers.
	 */
	if (requeue) {
		err = ia_css_pipe_enqueue_buffer(isp_subdev->css2_basis.pipes[pipe_index], &buffer);
		if (err)
			v4l2_err(&atomisp_dev,"%s, q to css fails: %d\n",
					__func__, err);
		return;
	}
	if (!error && q_buffers){
		v4l2_dbg(6, dbg_level, &atomisp_dev, "DMA End of atomisp_buf_done, About to call atomisp_qbuffers_to_css\n");
		atomisp_qbuffers_to_css(isp_subdev, false);
	}
}

void atomisp_delayed_init_work(struct work_struct *work)
{
	struct atomisp_sub_device *isp_subdev = container_of(work,
						  struct atomisp_sub_device,
						  delayed_init_work);
	isp_subdev->delayed_init = ATOMISP_DELAYED_INIT_DONE;
}

static void __atomisp_css_recover(struct atomisp_device *isp)
{
	enum ia_css_pipe_id css_pipe_id;
	struct atomisp_sub_device *isp_subdev;
	int i, ret;

		if (!isp->sw_contex.file_input)
			ia_css_irq_enable(
				IA_CSS_IRQ_INFO_CSS_RECEIVER_SOF, false);

		for (i = 0; i < isp->num_of_streams; i++) {
			isp_subdev = &isp->isp_subdev[i];
			if (isp_subdev->delayed_init == ATOMISP_DELAYED_INIT_QUEUED)
				cancel_work_sync(&isp_subdev->delayed_init_work);

			complete(&isp_subdev->init_done);
			isp_subdev->delayed_init = ATOMISP_DELAYED_INIT_NOT_QUEUED;

			css_pipe_id = atomisp_get_css_pipe_id(isp_subdev);

			if (isp_subdev->streaming ==
			    ATOMISP_DEVICE_STREAMING_ENABLED) {
				isp_subdev->streaming =
				    ATOMISP_DEVICE_STREAMING_STOPPING;

				if (ia_css_stop(isp_subdev, true))
					v4l2_warn(&atomisp_dev,
						  "stop css failed, reset may"
						  "be failed.\n");

				/* stream off sensor */
				ret = v4l2_subdev_call(
					isp->inputs[isp_subdev->input_curr].
					camera, video, s_stream, 0);
				if (ret)
					dev_warn(isp->dev,
						 "can't stop streaming"
						 "on sensor!\n");

				atomisp_clear_css_buffer_counters(isp_subdev);
				isp_subdev->streaming =
				    ATOMISP_DEVICE_STREAMING_STARTING;
			}
		}

		atomisp_acc_unload_extensions(isp);

		/* clear irq */
		/*enable_isp_irq(hrt_isp_css_irq_sp, false);*/
		/*clear_isp_irq(hrt_isp_css_irq_sp);*/

		/* reset ISP and restore its state */
		isp->isp_timeout = true;
		atomisp_reset(isp);
		isp->isp_timeout = false;

		if (atomisp_acc_load_extensions(isp) < 0)
			dev_err(isp->dev, "acc extension failed to reload\n");


		for( i = 0; i < isp->num_of_streams; i++) {
			isp_subdev = &isp->isp_subdev[i];
			if (isp_subdev->streaming !=
			    ATOMISP_DEVICE_STREAMING_STARTING)
					continue;

				ia_css_input_set_mode(isp_subdev,
				      get_input_mode(isp->inputs[isp_subdev->input_curr].type, isp));

				if (ia_css_start(isp_subdev, true) !=
				    IA_CSS_SUCCESS)
					v4l2_warn(&atomisp_dev,
					  "re-start css failed, reset may be"
					  "failed.\n");
				else
					isp_subdev->streaming =
					    ATOMISP_DEVICE_STREAMING_ENABLED;
		}

		if (!isp->sw_contex.file_input) {
			atomisp_control_irq_sof(isp);

			atomisp_set_term_en_count(isp);

			if (IS_MRFLD &&
			atomisp_freq_scaling(isp, ATOMISP_DFS_MODE_AUTO) < 0)
				dev_dbg(isp->dev, "dfs failed!\n");
		} else {
			if (IS_MRFLD &&
			atomisp_freq_scaling(isp, ATOMISP_DFS_MODE_MAX) < 0)
				dev_dbg(isp->dev, "dfs failed!\n");
		}

		for (i = 0; i < isp->num_of_streams; i++) {
			isp_subdev = &isp->isp_subdev[i];
			if (isp_subdev->streaming !=
			    ATOMISP_DEVICE_STREAMING_ENABLED)
					continue;

				ret = v4l2_subdev_call(
					isp->inputs[isp_subdev->input_curr].
					camera, video, s_stream, 1);
				if (ret)
					dev_warn(isp->dev,
						 "can't start streaming on"
						 "sensor!\n");
				/*
				 * FIXME!
				 * only one stream on continous mode now
				 */
				if (isp_subdev->params.continuous_vf &&
				 isp_subdev->run_mode->val !=
				    ATOMISP_RUN_MODE_VIDEO &&
				    isp_subdev->delayed_init ==
				    ATOMISP_DELAYED_INIT_NOT_QUEUED) {
					INIT_COMPLETION(isp_subdev->init_done);
					isp_subdev->delayed_init =
					    ATOMISP_DELAYED_INIT_QUEUED;
					queue_work(isp_subdev->delayed_init_workq,
						   &isp_subdev->delayed_init_work);
				}
				/*
				 * dequeueing buffers is not needed. CSS will
				 * recycle buffers that it has.
				 */
				atomisp_flush_bufs_and_wakeup(isp_subdev);
			}
			dev_err(isp->dev, "timeout recovery handling done\n");
}

void atomisp_wdt_work(struct work_struct *work)
{
	struct atomisp_device *isp = container_of(work, struct atomisp_device,
						  wdt_work);
	char debug_context[64];
	int i;
	struct atomisp_sub_device *isp_subdev = NULL;

	v4l2_dbg(6, dbg_level, &atomisp_dev, "DMA WARNING: timer not reset, entering atomisp_wdt_work\n");

	dev_err(isp->dev, "timeout %d of %d\n",
		atomic_read(&isp->wdt_count) + 1,
		ATOMISP_ISP_MAX_TIMEOUT_COUNT);

	mutex_lock(&isp->mutex);
	if (!atomisp_subdev_streaming_count(isp)) {
		mutex_unlock(&isp->mutex);
		return;
	}

	switch (atomic_inc_return(&isp->wdt_count)) {
	case ATOMISP_ISP_MAX_TIMEOUT_COUNT:
		v4l2_dbg(4, dbg_level, &atomisp_dev, "atomisp_wdt_work: ATOMISP_ISP_MAX_TIMEOUT_COUNT\n");
		for (i = 0; i < isp->num_of_streams; i++) {
			isp_subdev = &isp->isp_subdev[i];
			if (isp_subdev->streaming ==
			    ATOMISP_DEVICE_STREAMING_ENABLED) {
				atomisp_clear_css_buffer_counters(isp_subdev);
				atomisp_flush_bufs_and_wakeup(isp_subdev);
				complete(&isp_subdev->init_done);
			}
		}

		atomic_set(&isp->wdt_count, 0);
		isp->isp_fatal_error = true;

		mutex_unlock(&isp->mutex);
		return;
	default:
		ia_css_debug_dump_sp_sw_debug_info();
		ia_css_debug_dump_debug_info(debug_context);
		for (i = 0; i < isp->num_of_streams; i++) {
			isp_subdev = &isp->isp_subdev[i];
			if (isp_subdev->streaming ==
		    		ATOMISP_DEVICE_STREAMING_ENABLED) {
				dev_err(isp->dev, "%s, pipe[%d] buffers in css: %d\n", __func__,
					isp_subdev->video_out_capture.pipe_type,
					isp_subdev->video_out_capture.buffers_in_css);
				dev_err(isp->dev, "%s, pipe[%d] buffers in css: %d\n", __func__,
					isp_subdev->video_out_vf.pipe_type,
					isp_subdev->video_out_vf.buffers_in_css);
				dev_err(isp->dev, "%s, pipe[%d] buffers in css: %d\n", __func__,
					isp_subdev->video_out_preview.pipe_type,
					isp_subdev->video_out_preview.buffers_in_css);
				dev_err(isp->dev, "%s, s3a buffers in css preview pipe: %d\n",
					__func__,
					isp_subdev->s3a_bufs_in_css[IA_CSS_PIPE_ID_PREVIEW]);
				dev_err(isp->dev, "%s, s3a buffers in css capture pipe: %d\n",
					__func__,
					isp_subdev->s3a_bufs_in_css[IA_CSS_PIPE_ID_CAPTURE]);
				dev_err(isp->dev, "%s, s3a buffers in css video pipe: %d\n",
					__func__, isp_subdev->s3a_bufs_in_css[IA_CSS_PIPE_ID_VIDEO]);
				dev_err(isp->dev, "%s, dis buffers in css: %d\n",
					__func__, isp_subdev->dis_bufs_in_css);
			}
		}
		ia_css_debug_dump_sp_state();
		ia_css_debug_dump_isp_state();
	}

	__atomisp_css_recover(isp);

	mutex_unlock(&isp->mutex);
}

void atomisp_css_flush(struct atomisp_device *isp)
{
        if (!atomisp_subdev_streaming_count(isp))
                return;

        /* Disable wdt */
        del_timer_sync(&isp->wdt);
        cancel_work_sync(&isp->wdt_work);

        /* Start recover */
        __atomisp_css_recover(isp);

        /* Restore wdt */
        if (isp->sw_contex.file_input)
                isp->wdt_duration = ATOMISP_ISP_FILE_TIMEOUT_DURATION;
        else
                isp->wdt_duration = ATOMISP_ISP_TIMEOUT_DURATION;

        mod_timer(&isp->wdt, jiffies + isp->wdt_duration);

        dev_dbg(isp->dev, "atomisp css flush done\n");
}

void atomisp_wdt(unsigned long isp_addr)
{
	struct atomisp_device *isp = (struct atomisp_device *)isp_addr;

	queue_work(isp->wdt_work_queue, &isp->wdt_work);
}

void atomisp_setup_flash(struct atomisp_sub_device *isp_subdev)
{
	struct atomisp_device *isp = isp_subdev->isp;

	if (isp_subdev->params.flash_state != ATOMISP_FLASH_REQUESTED &&
	    isp_subdev->params.flash_state != ATOMISP_FLASH_DONE)
		return;

	if (isp_subdev->params.num_flash_frames) {
		struct v4l2_control ctrl;

		/* make sure the timeout is set before setting flash mode */
		ctrl.id = V4L2_CID_FLASH_TIMEOUT;
		ctrl.value = FLASH_TIMEOUT;

		if (v4l2_subdev_call(isp->flash, core, s_ctrl, &ctrl)) {
			v4l2_err(&atomisp_dev, "flash timeout configure failed\n");
			return;
		}
		ia_css_stream_request_flash(isp_subdev->css2_basis.stream);
		isp_subdev->params.flash_state = ATOMISP_FLASH_ONGOING;
	} else {
		/* Flashing all frames is done */
		isp_subdev->params.flash_state = ATOMISP_FLASH_IDLE;
	}
}

//DIFFERENT - EXTRA
static struct atomisp_sub_device *__get_atomisp_subdev(
					struct ia_css_pipe *css_pipe,
					struct atomisp_device *isp) {
	int i, j;
	struct atomisp_sub_device *isp_subdev;

	for (i = 0; i < isp->num_of_streams; i++) {
		isp_subdev = &isp->isp_subdev[i];
		if (isp_subdev->streaming !=
		    ATOMISP_DEVICE_STREAMING_DISABLED) {
			for (j = 0; j < IA_CSS_PIPE_ID_NUM; j++)
				if (isp_subdev->css2_basis.pipes[j] &&
				    isp_subdev->css2_basis.pipes[j] == css_pipe)
					return isp_subdev;
		}
	}

	return NULL;
}

irqreturn_t atomisp_isr_thread(int irq, void *isp_ptr)
{
	struct atomisp_device *isp = isp_ptr;
	struct atomisp_css_event current_event;
	unsigned long flags;
	bool frame_done_found[isp->num_of_streams];
	bool css_pipe_done[isp->num_of_streams];
	bool reset_wdt_timer = false;
	struct atomisp_sub_device *isp_subdev;
	int i, streaming;
	DEFINE_KFIFO(events, struct atomisp_css_event, ATOMISP_CSS_EVENTS_MAX);

	v4l2_dbg(6, dbg_level, &atomisp_dev, "ENTER DMA atomisp_isr_thread\n");

	v4l2_dbg(5, dbg_level, &atomisp_dev, ">%s\n", __func__);
	memset(frame_done_found, 0, sizeof(frame_done_found));
	memset(css_pipe_done, 0, sizeof(css_pipe_done));
	mutex_lock(&isp->mutex);

	spin_lock_irqsave(&isp->lock, flags);
	for (i = 0, streaming = 0; i < isp->num_of_streams; i++)
		streaming += isp->isp_subdev[i].streaming ==
		    ATOMISP_DEVICE_STREAMING_ENABLED;
	spin_unlock_irqrestore(&isp->lock, flags);
	if (!streaming)
		goto out;

	while (ia_css_dequeue_event(&current_event.event)
				 == IA_CSS_SUCCESS) {
		enum  ia_css_pipe_id temp_id;
		ia_css_temp_pipe_to_pipe_id(current_event.event.pipe, &temp_id);
		isp_subdev = __get_atomisp_subdev(current_event.event.pipe, isp);

		if (!isp_subdev) {
			/* EOF Event does not have the css_pipe returned */
			if (current_event.event.type !=
			    IA_CSS_EVENT_TYPE_PORT_EOF) {
				v4l2_dbg(6, dbg_level, &atomisp_dev,
				         "%s:no subdev. event:%d\n", __func__,
		                         current_event.event.type);
				goto out;
			}
		}

		switch (current_event.event.type) {
		case IA_CSS_EVENT_TYPE_OUTPUT_FRAME_DONE:
			v4l2_dbg(6, dbg_level, &atomisp_dev, "IA_CSS_EVENT_TYPE_OUTPUT_FRAME_DONE, frame_done_found=true, about to call atomisp_buf_done\n");
			frame_done_found[isp_subdev->index] = true;
			atomisp_buf_done(isp_subdev, 0,
					 IA_CSS_BUFFER_TYPE_OUTPUT_FRAME,
					 temp_id,
					 true);
			reset_wdt_timer = true;
			break;
		case IA_CSS_EVENT_TYPE_3A_STATISTICS_DONE:
			atomisp_buf_done(isp_subdev, 0,
					 IA_CSS_BUFFER_TYPE_3A_STATISTICS,
					 temp_id,
					 css_pipe_done[isp_subdev->index]);
			break;
		case IA_CSS_EVENT_TYPE_VF_OUTPUT_FRAME_DONE:
			atomisp_buf_done(isp_subdev, 0,
					 IA_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME,
					 temp_id,
					 true);
			reset_wdt_timer = true;
			break;
		case IA_CSS_EVENT_TYPE_DIS_STATISTICS_DONE:
			atomisp_buf_done(isp_subdev, 0,
					 IA_CSS_BUFFER_TYPE_DIS_STATISTICS,
					 temp_id,
					 css_pipe_done[isp_subdev->index]);
			break;
		case IA_CSS_EVENT_TYPE_PIPELINE_DONE:
			css_pipe_done[isp_subdev->index] = true;
		case IA_CSS_EVENT_TYPE_PORT_EOF:
		case IA_CSS_EVENT_TYPE_FRAME_TAGGED:
			break;
		default:
			dev_err(isp->dev, "unhandled css stored event: 0x%x\n",
					current_event.event.type);
			break;
		}
	}

	/*
	 * css2.0 bug: no buffer flush mechanism for css buffer queue, but
	 * need to ensure there is no buffer in css after stream off
	 *
	 * before calling ia_css_stream_stop(), we will wait all the buffers are
	 * dequeued from css, then call ia_css_stream_destroy()
	 */
	for (i = 0; i < isp->num_of_streams; i++) {
		isp_subdev = &isp->isp_subdev[i];
		if (isp_subdev->streaming == ATOMISP_DEVICE_STREAMING_STOPPING
		    && !__buffers_in_css(isp_subdev))
			complete(&isp_subdev->buf_done);
	}

	for (i = 0; i < isp->num_of_streams; i++) {
		isp_subdev = &isp->isp_subdev[i];
		if (isp_subdev->streaming == ATOMISP_DEVICE_STREAMING_ENABLED) {
			if (frame_done_found[isp_subdev->index] &&
			    isp_subdev->params.css_update_params_needed) {
				atomisp_css_update_isp_params(isp_subdev);
				isp_subdev->params.css_update_params_needed = false;
				frame_done_found[isp_subdev->index] = false;
			}

			atomisp_setup_flash(isp_subdev);
			/*
		 	* If there are no buffers queued
		 	* then delete wdt timer.
		 	*/
			if (!atomisp_buffers_queued(isp_subdev)) {
				del_timer(&isp->wdt);
			} else {
				/* SOF irq should not reset wdt timer. */
				if (reset_wdt_timer) {
					mod_timer(&isp->wdt, jiffies +
					  isp->wdt_duration);
					atomic_set(&isp->wdt_count, 0);
				}
			}
		}
	}

out:
	mutex_unlock(&isp->mutex);

	/* s_stream will be called later outside of this thread */
	/* Calling s_stream now causes RGB FIFO to fail */
	if(!isp->sw_contex.file_input){
		for (i = 0; i < isp->num_of_streams; i++) {
			isp_subdev = &isp->isp_subdev[i];
			if (isp_subdev->streaming == ATOMISP_DEVICE_STREAMING_ENABLED
				&& css_pipe_done[isp_subdev->index]
				&& isp->sw_contex.file_input)
				v4l2_subdev_call(isp->inputs[isp_subdev->input_curr].
						 camera, video, s_stream, 1);
 		}
	}

	v4l2_dbg(5, dbg_level, &atomisp_dev, "<%s\n", __func__);

	return IRQ_HANDLED;
}

/*
 * utils for buffer allocation/free
 */

int atomisp_get_frame_pgnr(const struct ia_css_frame *frame, u32 *p_pgnr)
{
	if (!frame) {
		v4l2_err(&atomisp_dev,
			    "%s: NULL frame pointer ERROR.\n",
			    __func__);
		return -EINVAL;
	}

	*p_pgnr = DIV_ROUND_UP(frame->data_bytes, PAGE_SIZE);
	return 0;
}

/*
 * Get internal fmt according to V4L2 fmt
 */
static enum ia_css_frame_format v4l2_fmt_to_sh_fmt(u32 fmt)
{
	switch (fmt) {
	case V4L2_PIX_FMT_YUV420:
		return IA_CSS_FRAME_FORMAT_YUV420;
	case V4L2_PIX_FMT_YVU420:
		return IA_CSS_FRAME_FORMAT_YV12;
	case V4L2_PIX_FMT_YUV422P:
		return IA_CSS_FRAME_FORMAT_YUV422;
	case V4L2_PIX_FMT_YUV444:
		return IA_CSS_FRAME_FORMAT_YUV444;
	case V4L2_PIX_FMT_NV12:
		return IA_CSS_FRAME_FORMAT_NV12;
	case V4L2_PIX_FMT_NV21:
		return IA_CSS_FRAME_FORMAT_NV21;
	case V4L2_PIX_FMT_NV16:
		return IA_CSS_FRAME_FORMAT_NV16;
	case V4L2_PIX_FMT_NV61:
		return IA_CSS_FRAME_FORMAT_NV61;
	case V4L2_PIX_FMT_UYVY:
		return IA_CSS_FRAME_FORMAT_UYVY;
	case V4L2_PIX_FMT_YUYV:
		return IA_CSS_FRAME_FORMAT_YUYV;
	case V4L2_PIX_FMT_RGB24:
		return IA_CSS_FRAME_FORMAT_RGBA888;
	case V4L2_PIX_FMT_RGB32:
		return IA_CSS_FRAME_FORMAT_RGBA888;
	case V4L2_PIX_FMT_RGB565:
		return IA_CSS_FRAME_FORMAT_RGB565;
	case V4L2_PIX_FMT_SBGGR16:
	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SRGGB10:
	case V4L2_PIX_FMT_SBGGR12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SRGGB12:
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SRGGB8:
		return IA_CSS_FRAME_FORMAT_RAW;
	default:
		return -EINVAL;
	}
}

/*
 * raw format match between SH format and V4L2 format
 */
static int raw_output_format_match_input(u32 input, u32 output)
{
	if ((input == IA_CSS_STREAM_FORMAT_RAW_12) &&
	    ((output == V4L2_PIX_FMT_SRGGB12) ||
	     (output == V4L2_PIX_FMT_SGRBG12) ||
	     (output == V4L2_PIX_FMT_SBGGR12) ||
	     (output == V4L2_PIX_FMT_SGBRG12)))
		return 0;

	if ((input == IA_CSS_STREAM_FORMAT_RAW_10) &&
	    ((output == V4L2_PIX_FMT_SRGGB10) ||
	     (output == V4L2_PIX_FMT_SGRBG10) ||
	     (output == V4L2_PIX_FMT_SBGGR10) ||
	     (output == V4L2_PIX_FMT_SGBRG10)))
		return 0;

	if ((input == IA_CSS_STREAM_FORMAT_RAW_8) &&
	    ((output == V4L2_PIX_FMT_SRGGB8) ||
	     (output == V4L2_PIX_FMT_SGRBG8) ||
	     (output == V4L2_PIX_FMT_SBGGR8) ||
	     (output == V4L2_PIX_FMT_SGBRG8)))
		return 0;

	if ((input == IA_CSS_STREAM_FORMAT_RAW_16) &&
	    (output == V4L2_PIX_FMT_SBGGR16))
		return 0;

	return -EINVAL;
}

static u32 get_pixel_depth(u32 pixelformat)
{
	switch (pixelformat) {
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_YVU420:
		return 12;
	case V4L2_PIX_FMT_YUV422P:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_NV61:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_SBGGR16:
	case V4L2_PIX_FMT_SBGGR12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SRGGB12:
	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SRGGB10:
		return 16;
	case V4L2_PIX_FMT_RGB24:
	case V4L2_PIX_FMT_YUV444:
		return 24;
	case V4L2_PIX_FMT_RGB32:
		return 32;
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SRGGB8:
		return 8;
	default:
		return 8 * 2;	/* raw type now */
	}
}

static int is_pixelformat_raw(u32 pixelformat)
{
	switch (pixelformat) {
	case V4L2_PIX_FMT_SBGGR16:
	case V4L2_PIX_FMT_SBGGR12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SRGGB12:
	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SRGGB10:
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SRGGB8:
		return 1;
	default:
		return 0;
	}
}

int atomisp_is_mbuscode_raw(uint32_t code)
{
	const struct atomisp_format_bridge *b =
		atomisp_get_format_bridge_from_mbus(code);

	BUG_ON(!b);

	return is_pixelformat_raw(b->pixelformat);
}

static int get_sh_input_format(u32 pixelformat)
{
	switch (pixelformat) {
	case V4L2_PIX_FMT_YUV420:
		return IA_CSS_STREAM_FORMAT_YUV420_8;

	case V4L2_PIX_FMT_YUV422P:
		return IA_CSS_STREAM_FORMAT_YUV422_8;

	case V4L2_PIX_FMT_RGB565:
		return IA_CSS_STREAM_FORMAT_RGB_565;
	case V4L2_PIX_FMT_RGB24:
		return IA_CSS_STREAM_FORMAT_RGB_888;
	case V4L2_PIX_FMT_BGR666:
		return IA_CSS_STREAM_FORMAT_RGB_666;

	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SRGGB8:
		return IA_CSS_STREAM_FORMAT_RAW_8;

	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SRGGB10:
		return IA_CSS_STREAM_FORMAT_RAW_10;

	case V4L2_PIX_FMT_SBGGR12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SRGGB12:
		return IA_CSS_STREAM_FORMAT_RAW_12;

	case V4L2_PIX_FMT_SBGGR16:
		return IA_CSS_STREAM_FORMAT_RAW_16;

	default:
		return -EINVAL;
	}
}

/*
 * ISP features control function
 */

/*
 * Set ISP capture mode based on current settings
 */
static void atomisp_update_capture_mode(struct atomisp_sub_device *isp_subdev)
{
	struct ia_css_isp_config isp_config;
	enum ia_css_capture_mode capture_mode;

	if (!isp_subdev->css2_basis.stream) {
		v4l2_err(&atomisp_dev,
			 "%s called after streamoff, skipping.\n",
			 __func__);
		return;
	}
	memset(&isp_config, 0, sizeof(struct ia_css_isp_config));
	isp_config.capture_config = &isp_subdev->params.capture_config;
	ia_css_stream_get_isp_config(isp_subdev->css2_basis.stream, &isp_config);

	if (isp_subdev->params.low_light)
		capture_mode = IA_CSS_CAPTURE_MODE_LOW_LIGHT;
	else if (isp_subdev->params.gdc_cac_en)
		capture_mode = IA_CSS_CAPTURE_MODE_ADVANCED;
	else
		capture_mode = IA_CSS_CAPTURE_MODE_PRIMARY;
	if (capture_mode != isp_subdev->params.capture_config.mode) {
		isp_subdev->params.capture_config.mode = capture_mode;
		isp_subdev->params.config.capture_config = &isp_subdev->params.capture_config;
	}
}

/*
 * Function to enable/disable lens geometry distortion correction (GDC) and
 * chromatic aberration correction (CAC)
 */
int atomisp_gdc_cac(struct atomisp_sub_device *isp_subdev, int flag, __s32 * value)
{
	struct atomisp_device *isp = isp_subdev->isp;
	if (flag == 0) {
		*value = isp_subdev->params.gdc_cac_en;
		return 0;
	}

	isp_subdev->params.gdc_cac_en = !!*value;
	if (isp_subdev->params.gdc_cac_en) {
		isp_subdev->params.config.morph_table = isp->inputs[isp_subdev->input_curr].morph_table;
		isp_subdev->params.css_update_params_needed = true;
		atomisp_update_capture_mode(isp_subdev);
	}
	return 0;
}

/*
 * Function to enable/disable low light mode including ANR
 */
int atomisp_low_light(struct atomisp_sub_device *isp_subdev, int flag, __s32 * value)
{
	if (flag == 0) {
		*value = isp_subdev->params.low_light;
		return 0;
	}

	isp_subdev->params.low_light = (*value != 0);
	atomisp_update_capture_mode(isp_subdev);
	return 0;
}

/*
 * Function to enable/disable extra noise reduction (XNR) in low light
 * condition
 */
int atomisp_xnr(struct atomisp_sub_device *isp_subdev, int flag, int *xnr_enable)
{
	struct ia_css_isp_config isp_config;
	if (!xnr_enable)
		return 0;

	if (flag == 0) {
		*xnr_enable = isp_subdev->params.xnr_en;
		return 0;
	}

	if (!isp_subdev->css2_basis.stream) {
		v4l2_err(&atomisp_dev,
			 "%s called after streamoff, skipping.\n",
			 __func__);
		return -EINVAL;
	}
	memset(&isp_config, 0, sizeof(struct ia_css_isp_config));
	isp_config.capture_config = &isp_subdev->params.capture_config;
	ia_css_stream_get_isp_config(isp_subdev->css2_basis.stream, &isp_config);

	if (isp_subdev->params.capture_config.enable_xnr != !!*xnr_enable)
		isp_subdev->params.capture_config.enable_xnr = !!*xnr_enable;

	return 0;
}

/*
 * Function to load the larger FW
 */
int atomisp_fw(struct atomisp_device *isp, int flag, int *data){
		int ret = 0;

		if (fastboot){
				v4l2_dbg(3, dbg_level, &atomisp_dev, "Starting firmware switch\n");
				if (isp->firmware_switched == false){
						ret = firmware_load_helper(&(isp->aux_firmware), isp);
						if (atomisp_subdev_streaming_count(isp) == 0) {
								atomisp_destroy_pipes_stream_force(isp->isp_subdev);
								ia_css_unload_firmware(true);
								atomisp_css_uninit(isp);
								switch_to_full_firmware(isp);
								ret = atomisp_css_init(isp);
								v4l2_dbg(2, dbg_level, &atomisp_dev, "Firmware switch Complete. Also switched to BUFFERED_SENSOR Mode for multistreaming\n");
								return 0;
						}
						else {
								v4l2_dbg(2, dbg_level, &atomisp_dev, "Cannot switch firmware now. Device is currently streaming. Switch will happen in the next streamoff\n");
								isp->switch_fw_on_streamoff = true;
								/*
								 * do switch in atomisp_reset() of streamoff
								 */
								return -EBUSY;
						}
				}
				else{
						v4l2_dbg(2, dbg_level, &atomisp_dev, "Firmware already switched\n");
						return 0;
				}
		}
		else{
				v4l2_dbg(2, dbg_level, &atomisp_dev, "fastboot disabled\n");
				return -EINVAL;
		}
}


/*
 * Function to configure bayer noise reduction
 */
int atomisp_nr(struct atomisp_sub_device *isp_subdev, int flag,
	       struct atomisp_nr_config *arg)
{
	if (arg == NULL)
		return -EINVAL;

	if (sizeof(*arg) != sizeof(isp_subdev->params.config.nr_config)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

	if (flag == 0) {
		struct ia_css_nr_config nr_config;
		struct ia_css_isp_config isp_config;

		if (!isp_subdev->css2_basis.stream) {
			v4l2_err(&atomisp_dev,
				 "%s called after streamoff, skipping.\n",
				 __func__);
			return -EINVAL;
		}
		memset(&nr_config, 0, sizeof(struct ia_css_nr_config));
		memset(&isp_config, 0, sizeof(struct ia_css_isp_config));

		/* Get nr config from current setup */
		isp_config.nr_config = &nr_config;
		ia_css_stream_get_isp_config(isp_subdev->css2_basis.stream, &isp_config);
		memcpy(arg, &nr_config, sizeof(*arg));
	} else {
		/* Set nr config to isp parameters */
		if (!isp_subdev->params.config.nr_config)
			isp_subdev->params.config.nr_config = &isp_subdev->params.nr_config;
		memcpy(isp_subdev->params.config.nr_config, arg,
			sizeof(struct ia_css_nr_config));
		isp_subdev->params.css_update_params_needed = true;
	}
	return 0;
}

/*
 * Function to configure temporal noise reduction (TNR)
 */
int atomisp_tnr(struct atomisp_sub_device *isp_subdev, int flag,
		struct atomisp_tnr_config *config)
{
	if (config == NULL)
		return -EINVAL;

	if (sizeof(*config) != sizeof(isp_subdev->params.config.tnr_config)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}


	/* Get tnr config from current setup */
	if (flag == 0) {
		struct ia_css_tnr_config tnr_config;
		struct ia_css_isp_config isp_config;
		if (!isp_subdev->css2_basis.stream) {
			v4l2_err(&atomisp_dev,
				 "%s called after streamoff, skipping.\n",
				 __func__);
			return -EINVAL;
		}
		memset(&tnr_config, 0, sizeof(struct ia_css_tnr_config));
		memset(&isp_config, 0, sizeof(struct ia_css_isp_config));
		isp_config.tnr_config = &tnr_config;
		ia_css_stream_get_isp_config(isp_subdev->css2_basis.stream, &isp_config);

		/* Get tnr config from current setup */
		memcpy(config, &isp_config.tnr_config, sizeof(*config));
	} else {
		if (!isp_subdev->params.config.tnr_config)
			isp_subdev->params.config.tnr_config = &isp_subdev->params.tnr_config;
		/* Set tnr config to isp parameters */
		memcpy(isp_subdev->params.config.tnr_config, config,
			sizeof(struct ia_css_tnr_config));
		isp_subdev->params.css_update_params_needed = true;
	}

	return 0;
}

/*
 * Function to get histogram data for image frame
 */
int atomisp_histogram(struct atomisp_sub_device *isp_subdev, int flag, void *config)
{
#if defined(CONFIG_CSS_ONE)
	struct atomisp_histogram *arg = (struct atomisp_histogram *)config;
	struct ia_css_histogram *histogram;
	int ret = 0;
	unsigned int *buffer;

	if (arg == NULL)
		return -EINVAL;

	if (sizeof(*arg) != sizeof(*histogram)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

	if (flag == 0) {
		buffer = kzalloc(2048 * sizeof(unsigned int), GFP_KERNEL);
		if (buffer == NULL) {
			v4l2_err(&atomisp_dev,
					"buffer allocate error\n");
			return -ENOMEM;
		}

		ret = ia_css_histogram_allocate(2048, &histogram);
		if (ret != IA_CSS_SUCCESS) {
			v4l2_err(&atomisp_dev,
					"ia_css_histogram_allocate failed\n");
			goto buffer_free;
		}

		if (isp->vf_frame == NULL) {
			v4l2_err(&atomisp_dev,
					"No frame for histogram\n");
			ret = -EINVAL;
			goto histogram_free;
		}

		ret = ia_css_histogram_start(isp_subdev->vf_frame, histogram);
		if (ret != IA_CSS_SUCCESS) {
			v4l2_err(&atomisp_dev,
					"ia_css_get_y_histogram failed\n");
			goto histogram_free;
		}
		ia_css_wait_for_completion();

		ret = hmm_load(histogram->data, buffer,
			histogram->num_elements * sizeof(unsigned int));
		if (ret) {
			v4l2_err(&atomisp_dev, "hmm_load failed\n");
			goto histogram_free;
		}

		ret = copy_to_user(arg->data, buffer,
			histogram->num_elements * sizeof(unsigned int));
		if (ret) {
			v4l2_err(&atomisp_dev,
					"copy to user failed\n");
			ret = -EFAULT;
			goto histogram_free;
		}

		ret = 0;
		arg->num_elements = histogram->num_elements;

histogram_free:
		ia_css_histogram_free(histogram);
buffer_free:
		kfree(buffer);

		return ret;
	}

	isp_subdev->params.config.histogram_elenum = arg->num_elements;
#endif
	return 0;
}

/*
 * Function to configure black level compensation
 */
int atomisp_black_level(struct atomisp_sub_device *isp_subdev, int flag,
			struct atomisp_ob_config *config)
{
	if (config == NULL)
		return -EINVAL;

	if (sizeof(*config) != sizeof(isp_subdev->params.config.ob_config)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

	if (flag == 0) {
		struct ia_css_ob_config ob_config;
		struct ia_css_isp_config isp_config;
		if (!isp_subdev->css2_basis.stream) {
			v4l2_err(&atomisp_dev,
				 "%s called after streamoff, skipping.\n",
				 __func__);
			return -EINVAL;
		}
		memset(&ob_config, 0, sizeof(struct ia_css_ob_config));
		memset(&isp_config, 0, sizeof(struct ia_css_isp_config));
		isp_config.ob_config = &ob_config;
		/* Get ob config from current setup */
		ia_css_stream_get_isp_config(isp_subdev->css2_basis.stream, &isp_config);
		memcpy(config, &ob_config, sizeof(*config));
	} else {
		if (!isp_subdev->params.config.ob_config)
			isp_subdev->params.config.ob_config = &isp_subdev->params.ob_config;
		/* Set ob config to isp parameters */
		memcpy(isp_subdev->params.config.ob_config, config,
			sizeof(struct ia_css_ob_config));
		isp_subdev->params.css_update_params_needed = true;
	}

	return 0;
}

/*
 * Function to configure edge enhancement
 */
int atomisp_ee(struct atomisp_sub_device *isp_subdev, int flag,
	       struct atomisp_ee_config *config)
{
	if (config == NULL)
		return -EINVAL;

	if (sizeof(*config) != sizeof(isp_subdev->params.config.ee_config)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

	if (flag == 0) {
		struct ia_css_ee_config ee_config;
		struct ia_css_isp_config isp_config;
		if (!isp_subdev->css2_basis.stream) {
			v4l2_err(&atomisp_dev,
				 "%s called after streamoff, skipping.\n",
				 __func__);
			return -EINVAL;
		}
		memset(&ee_config, 0, sizeof(struct ia_css_ee_config));
		memset(&isp_config, 0, sizeof(struct ia_css_isp_config));
		/* Get ee config from current setup */
		isp_config.ee_config = &ee_config;
		ia_css_stream_get_isp_config(isp_subdev->css2_basis.stream, &isp_config);
		memcpy(config, &ee_config, sizeof(*config));
	} else {
		if (!isp_subdev->params.config.ee_config)
			isp_subdev->params.config.ee_config = &isp_subdev->params.ee_config;
		/* Set ee config to isp parameters */
		memcpy(isp_subdev->params.config.ee_config, config,
		       sizeof(*isp_subdev->params.config.ee_config));
		isp_subdev->params.css_update_params_needed = true;
	}

	return 0;
}

/*
 * Function to update Gamma table for gamma, brightness and contrast config
 */
int atomisp_gamma(struct atomisp_sub_device *isp_subdev, int flag,
		  struct atomisp_gamma_table *config)
{
	if (config == NULL)
		return -EINVAL;

	if (sizeof(*config) != sizeof(isp_subdev->params.config.gamma_table)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

	if (flag == 0) {
		struct ia_css_gamma_table tab;
		struct ia_css_isp_config isp_config;
		if (!isp_subdev->css2_basis.stream) {
			v4l2_err(&atomisp_dev,
				 "%s called after streamoff, skipping.\n",
				 __func__);
			return -EINVAL;
		}
		memset(&tab, 0, sizeof(struct ia_css_gamma_table));
		memset(&isp_config, 0, sizeof(struct ia_css_isp_config));
		/* Get gamma table from current setup */
		isp_config.gamma_table = &tab;
		ia_css_stream_get_isp_config(isp_subdev->css2_basis.stream, &isp_config);
		memcpy(config, &tab, sizeof(*config));
	} else {
		if (!isp_subdev->params.config.gamma_table)
			isp_subdev->params.config.gamma_table = &isp_subdev->params.gamma_table;
		/* Set gamma table to isp parameters */
		memcpy(isp_subdev->params.config.gamma_table, config,
		       sizeof(*isp_subdev->params.config.gamma_table));
	}

	return 0;
}

/*
 * Function to update Ctc table for Chroma Enhancement
 */
int atomisp_ctc(struct atomisp_sub_device *isp_subdev, int flag,
		struct atomisp_ctc_table *config)
{
	if (config == NULL)
		return -EINVAL;

	if (sizeof(*config) != sizeof(isp_subdev->params.config.ctc_table)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

	if (flag == 0) {
		struct ia_css_ctc_table tab;
		struct ia_css_isp_config isp_config;
		if (!isp_subdev->css2_basis.stream) {
			v4l2_err(&atomisp_dev,
				 "%s called after streamoff, skipping.\n",
				 __func__);
			return -EINVAL;
		}
		memset(&tab, 0, sizeof(struct ia_css_ctc_table));
		memset(&isp_config, 0, sizeof(struct ia_css_isp_config));
		/* Get ctc table from current setup */
		isp_config.ctc_table = &tab;
		ia_css_stream_get_isp_config(isp_subdev->css2_basis.stream, &isp_config);
		memcpy(config, &tab, sizeof(*config));
	} else {
		if (!isp_subdev->params.config.ctc_table)
			isp_subdev->params.config.ctc_table = &isp_subdev->params.ctc_table;
		/* Set ctc table to isp parameters */
		memcpy(isp_subdev->params.config.ctc_table, config,
			sizeof(*isp_subdev->params.config.ctc_table));
	}

	return 0;
}

/*
 * Function to update gamma correction parameters
 */
int atomisp_gamma_correction(struct atomisp_sub_device *isp_subdev, int flag,
	struct atomisp_gc_config *config)
{

	if (sizeof(*config) != sizeof(isp_subdev->params.config.gc_config)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

	if (flag == 0) {
		struct ia_css_gc_config gc_config;
		struct ia_css_isp_config isp_config;
		if (!isp_subdev->css2_basis.stream) {
			v4l2_err(&atomisp_dev,
				 "%s called after streamoff, skipping.\n",
				 __func__);
			return -EINVAL;
		}
		memset(&gc_config, 0, sizeof(struct ia_css_gc_config));
		memset(&isp_config, 0, sizeof(struct ia_css_isp_config));
		isp_config.gc_config = &gc_config;
		ia_css_stream_get_isp_config(isp_subdev->css2_basis.stream, &isp_config);
		/* Get gamma correction params from current setup */
		memcpy(config, &gc_config, sizeof(*config));
	} else {
		if (!isp_subdev->params.config.gc_config)
			isp_subdev->params.config.gc_config = &isp_subdev->params.gc_config;
		/* Set gamma correction params to isp parameters */
		memcpy(isp_subdev->params.config.gc_config, config, sizeof(*config));
		isp_subdev->params.css_update_params_needed = true;
	}

	return 0;
}

void atomisp_free_internal_buffers(struct atomisp_sub_device *isp_subdev)
{
	struct atomisp_device *isp = isp_subdev->isp;
	struct ia_css_morph_table *tab;

	tab = isp->inputs[isp_subdev->input_curr].morph_table;
	if (tab) {
		ia_css_morph_table_free(tab);
		isp->inputs[isp_subdev->input_curr].morph_table = NULL;
	}

	if (isp_subdev->raw_output_frame) {
		ia_css_frame_free(isp_subdev->raw_output_frame);
		isp_subdev->raw_output_frame = NULL;
	}
}

void atomisp_free_3a_dvs_buffers(struct atomisp_sub_device *isp_subdev)
{
	struct atomisp_s3a_buf *s3a_buf, *_s3a_buf;
	struct atomisp_dvs_buf *dvs_buf, *_dvs_buf;

	/* 3A statistics use vmalloc, DIS use kmalloc */
	if (isp_subdev->params.curr_grid_info.dvs_grid.enable) {
		ia_css_dvs_coefficients_free(isp_subdev->params.dvs_coeff);
		ia_css_dvs_statistics_free(isp_subdev->params.dvs_stat);
		isp_subdev->params.dvs_coeff = NULL;
		isp_subdev->params.dvs_stat = NULL;
		isp_subdev->params.dvs_hor_proj_bytes = 0;
		isp_subdev->params.dvs_ver_proj_bytes = 0;
		isp_subdev->params.dvs_hor_coef_bytes = 0;
		isp_subdev->params.dvs_ver_coef_bytes = 0;
		isp_subdev->params.dvs_proj_data_valid = false;
		list_for_each_entry_safe(dvs_buf, _dvs_buf,
					 &isp_subdev->dvs_stats, list) {
			ia_css_isp_dvs_statistics_free(dvs_buf->dvs_stat);
			list_del(&dvs_buf->list);
			kfree(dvs_buf);
		}
	}
	if (isp_subdev->params.curr_grid_info.s3a_grid.enable) {
		ia_css_3a_statistics_free(isp_subdev->params.s3a_user_stat);
		isp_subdev->params.s3a_user_stat = NULL;
		isp_subdev->params.s3a_buf_data_valid = false;
		isp_subdev->params.s3a_output_bytes = 0;
		list_for_each_entry_safe(s3a_buf, _s3a_buf,
					 &isp_subdev->s3a_stats, list) {
			ia_css_isp_3a_statistics_free(s3a_buf->s3a_stat);
			list_del(&s3a_buf->list);
			kfree(s3a_buf);
		}
	}
}

static void atomisp_update_grid_info(struct atomisp_sub_device *isp_subdev)
{
	int err;
	struct ia_css_pipe_info p_info;
	struct ia_css_grid_info old_info;

	memset(&p_info, 0, sizeof(struct ia_css_pipe_info));
	memset(&old_info, 0, sizeof(struct ia_css_grid_info));
	ia_css_pipe_get_info(isp_subdev->css2_basis.pipes[isp_subdev->css2_basis.curr_pipe],
			     &p_info);
	memcpy(&old_info,&isp_subdev->params.curr_grid_info,sizeof(struct ia_css_grid_info));
	memcpy(&isp_subdev->params.curr_grid_info, &p_info.grid_info,
	       		sizeof(struct ia_css_grid_info));

	/* If the grid info has not changed and the buffers for 3A and
	 * DIS statistics buffers are allocated or buffer size would be zero
	 * then no need to do anything. */
	if ((!memcmp(&old_info, &isp_subdev->params.curr_grid_info,
		     sizeof(old_info)) &&
	    isp_subdev->params.s3a_user_stat && isp_subdev->params.dvs_stat) ||
		isp_subdev->params.curr_grid_info.s3a_grid.width == 0 ||
		isp_subdev->params.curr_grid_info.s3a_grid.height == 0) {
		return;
	}

	/* We must free all buffers because they no longer match
	   the grid size. */
	atomisp_free_3a_dvs_buffers(isp_subdev);

	v4l2_dbg(3, dbg_level, &atomisp_dev, "grid info changed. re-allocate status buffer.\n");
	err = atomisp_alloc_css_stat_bufs(isp_subdev);
	if (err) {
		v4l2_err(&atomisp_dev,
			 "stat_buf allocate error\n");
		goto err_3a;
	}

	isp_subdev->params.s3a_user_stat =
	    ia_css_3a_statistics_allocate(&isp_subdev->params.curr_grid_info.s3a_grid);
	if (!isp_subdev->params.s3a_user_stat)
		goto err_3a;

	/* 3A statistics. These can be big, so we use vmalloc. */
	isp_subdev->params.s3a_output_bytes =
	    isp_subdev->params.curr_grid_info.s3a_grid.width *
	    isp_subdev->params.curr_grid_info.s3a_grid.height *
	    sizeof(*isp_subdev->params.s3a_user_stat->data);

	isp_subdev->params.s3a_buf_data_valid = false;
	if (isp_subdev->params.curr_grid_info.dvs_grid.enable) {
		/* DIS coefficients. */
		isp_subdev->params.dvs_coeff =
		    ia_css_dvs_coefficients_allocate(
				&isp_subdev->params.curr_grid_info.dvs_grid);
		if (!isp_subdev->params.dvs_coeff)
			goto err_dvs;

		isp_subdev->params.dvs_hor_coef_bytes =
		    isp_subdev->params.curr_grid_info.dvs_grid.num_hor_coefs*
		    IA_CSS_DVS_NUM_COEF_TYPES *
		    sizeof(*isp_subdev->params.dvs_coeff->hor_coefs);

		isp_subdev->params.dvs_ver_coef_bytes =
		    isp_subdev->params.curr_grid_info.dvs_grid.num_ver_coefs *
		    IA_CSS_DVS_NUM_COEF_TYPES *
		    sizeof(*isp_subdev->params.dvs_coeff->ver_coefs);

		/* DIS projections. */
		isp_subdev->params.dvs_proj_data_valid = false;
		isp_subdev->params.dvs_stat = ia_css_dvs_statistics_allocate(&isp_subdev->params.curr_grid_info.dvs_grid);
		if (!isp_subdev->params.dvs_stat)
			goto err_dvs;
		isp_subdev->params.dvs_hor_proj_bytes =
		    isp_subdev->params.curr_grid_info.dvs_grid.aligned_height *
		    IA_CSS_DVS_NUM_COEF_TYPES *
		    sizeof(*isp_subdev->params.dvs_stat->hor_proj);

		isp_subdev->params.dvs_ver_proj_bytes =
		    isp_subdev->params.curr_grid_info.dvs_grid.aligned_width *
		    IA_CSS_DVS_NUM_COEF_TYPES *
		    sizeof(*isp_subdev->params.dvs_stat->ver_proj);
	}
	return;

	/* Failure for 3A buffers does not influence DIS buffers */
err_3a:
	if (isp_subdev->params.s3a_output_bytes != 0) {
		/* For SOC sensor happens s3a_output_bytes == 0,
		 *  using if condition to exclude false error log */
		v4l2_err(&atomisp_dev, "Failed allocate memory for 3A statistics\n");
	}
	atomisp_free_3a_dvs_buffers(isp_subdev);
	return;

err_dvs:
	v4l2_err(&atomisp_dev, "Failed allocate memory for DIS statistics\n");
	atomisp_free_3a_dvs_buffers(isp_subdev);
}

static void atomisp_curr_user_grid_info(struct atomisp_sub_device *isp_subdev,
				    struct atomisp_grid_info *info)
{
#ifndef ATOMISP_CSS2
	info->isp_in_width          =
	    isp_subdev->params.curr_grid_info.isp_in_width;
	info->isp_in_height         =
	    isp_subdev->params.curr_grid_info.isp_in_height;
	info->s3a_width             =
	    isp_subdev->params.curr_grid_info.s3a_grid.width;
	info->s3a_height            =
		isp_subdev->params.curr_grid_info.s3a_grid.height;
	info->s3a_bqs_per_grid_cell =
		isp_subdev->params.curr_grid_info.s3a_grid.bqs_per_grid_cell;

	info->dis_width          =
	    isp_subdev->params.curr_grid_info.dvs_grid.width;
	info->dis_aligned_width  =
		isp_subdev->params.curr_grid_info.dvs_grid.aligned_width;
	info->dis_height         =
	    isp_subdev->params.curr_grid_info.dvs_grid.height;
	info->dis_aligned_height =
		isp_subdev->params.curr_grid_info.dvs_grid.aligned_height;
	info->dis_bqs_per_grid_cell =
		isp_subdev->params.curr_grid_info.dvs_grid.bqs_per_grid_cell;
	info->dis_hor_coef_num      =
		isp_subdev->params.curr_grid_info.dvs_grid.num_hor_coefs;
	info->dis_ver_coef_num      =
		isp_subdev->params.curr_grid_info.dvs_grid.num_ver_coefs;
#else
	memcpy(info, &isp_subdev->params.curr_grid_info.s3a_grid,
	       sizeof(struct ia_css_3a_grid_info));

#endif
}

static int atomisp_compare_grid(struct atomisp_sub_device *isp_subdev,
				struct atomisp_grid_info *atomgrid)
{
	struct atomisp_grid_info tmp = {0};

	atomisp_curr_user_grid_info(isp_subdev, &tmp);
	return memcmp(atomgrid, &tmp, sizeof(tmp));
}

/*
 * Function to update Gdc table for gdc
 */
int atomisp_gdc_cac_table(struct atomisp_sub_device *isp_subdev, int flag,
			  struct atomisp_morph_table *config)
{
	struct atomisp_device *isp = isp_subdev->isp;
	int ret;
	int i;

	if (flag == 0) {
		struct ia_css_morph_table tab;
		struct ia_css_isp_config isp_config;
		memset(&tab, 0, sizeof(struct ia_css_morph_table));
		memset(&isp_config, 0, sizeof(struct ia_css_isp_config));
		if (!isp_subdev->css2_basis.stream) {
			v4l2_err(&atomisp_dev,
				 "%s called after streamoff, skipping.\n",
				 __func__);
			return -EINVAL;
		}
		/* Get gdc table from current setup */
		isp_subdev->params.config.morph_table = &tab;
		ia_css_stream_get_isp_config(isp_subdev->css2_basis.stream, &isp_subdev->params.config);

		config->width = tab.width;
		config->height = tab.height;

		for (i = 0; i < IA_CSS_MORPH_TABLE_NUM_PLANES; i++) {
			ret = copy_to_user(config->coordinates_x[i],
				tab.coordinates_x[i], tab.height *
				tab.width * sizeof(*tab.coordinates_x[i]));
			if (ret) {
				v4l2_err(&atomisp_dev,
					"Failed to copy to User for x\n");
				return -EFAULT;
			}
			ret = copy_to_user(config->coordinates_y[i],
				tab.coordinates_y[i], tab.height *
				tab.width * sizeof(*tab.coordinates_y[i]));
			if (ret) {
				v4l2_err(&atomisp_dev,
					"Failed to copy to User for y\n");
				return -EFAULT;
			}
		}
	} else {
		struct ia_css_morph_table *table =
			isp->inputs[isp_subdev->input_curr].morph_table;

		/* free first if we have one */
		if (table) {
			ia_css_morph_table_free(table);
			isp->inputs[isp_subdev->input_curr].morph_table = NULL;
		}

		/* allocate new one */
		table = ia_css_morph_table_allocate(config->width,
						  config->height);

		if (!table) {
			v4l2_err(&atomisp_dev, "out of memory\n");
			return -EINVAL;
		}

		for (i = 0; i < IA_CSS_MORPH_TABLE_NUM_PLANES; i++) {
			ret = copy_from_user(table->coordinates_x[i],
				config->coordinates_x[i],
				config->height * config->width *
				sizeof(*config->coordinates_x[i]));
			if (ret) {
				v4l2_err(&atomisp_dev,
				"Failed to copy from User for x, ret %d\n",
				ret);
				ia_css_morph_table_free(table);
				return -EFAULT;
			}
			ret = copy_from_user(table->coordinates_y[i],
				config->coordinates_y[i],
				config->height * config->width *
				sizeof(*config->coordinates_y[i]));
			if (ret) {
				v4l2_err(&atomisp_dev,
				"Failed to copy from User for y, ret is %d\n",
				ret);
				ia_css_morph_table_free(table);
				return -EFAULT;
			}
		}
		isp->inputs[isp_subdev->input_curr].morph_table = table;
		if (isp_subdev->params.gdc_cac_en)
			isp_subdev->params.config.morph_table = table;
	}

	return 0;
}

int atomisp_macc_table(struct atomisp_sub_device *isp_subdev, int flag,
		       struct atomisp_macc_config *config)
{
	struct ia_css_macc_table *macc_table;
	struct ia_css_macc_table tmp_macc_table;
	struct ia_css_isp_config isp_config;

	if (config == NULL)
		return -EINVAL;

	if (sizeof(config->table) != sizeof(*macc_table)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}
	memset(&tmp_macc_table, 0, sizeof(struct ia_css_macc_table));
	memset(&isp_config, 0, sizeof(struct ia_css_isp_config));
	switch (config->color_effect) {
	case V4L2_COLORFX_NONE:
		macc_table = NULL;
		return 0;
	case V4L2_COLORFX_SKY_BLUE:
		macc_table = &blue_macc_table;
		break;
	case V4L2_COLORFX_GRASS_GREEN:
		macc_table = &green_macc_table;
		break;
	case V4L2_COLORFX_SKIN_WHITEN_LOW:
		macc_table = &skin_low_macc_table;
		break;
	case V4L2_COLORFX_SKIN_WHITEN:
		macc_table = &skin_medium_macc_table;
		break;
	case V4L2_COLORFX_SKIN_WHITEN_HIGH:
		macc_table = &skin_high_macc_table;
		break;
	default:
		return -EINVAL;
	}

	if (flag == 0) {
		/* Get macc table from current setup */
		memcpy(&config->table, macc_table,
		       sizeof(struct ia_css_macc_table));
	} else {
		memcpy(macc_table, &config->table,
		       sizeof(struct ia_css_macc_table));
		if (config->color_effect == isp_subdev->params.color_effect)
			isp_subdev->params.config.macc_table = macc_table;
	}

	return 0;
}

int atomisp_set_dis_vector(struct atomisp_sub_device *isp_subdev,
			   struct atomisp_dis_vector *vector)
{
	if (!isp_subdev->params.config.motion_vector)
		isp_subdev->params.config.motion_vector =
			&isp_subdev->params.motion_vector;

	memset(isp_subdev->params.config.motion_vector,
	       		0, sizeof(struct ia_css_vector));
	isp_subdev->params.motion_vector.x = vector->x;
	isp_subdev->params.motion_vector.y = vector->y;

	isp_subdev->params.dvs_proj_data_valid = false;
	isp_subdev->params.css_update_params_needed = true;
	return 0;
}

/*
 * Function to set/get image stablization statistics
 */
int atomisp_get_dis_stat(struct atomisp_sub_device *isp_subdev,
			 struct atomisp_dis_statistics *stats)
{
	unsigned long flags;
	int error;
	struct atomisp_device *isp = isp_subdev->isp;

	if (stats->vertical_projections   == NULL ||
	    stats->horizontal_projections == NULL ||
	    isp_subdev->params.dvs_stat->hor_proj == NULL ||
	    isp_subdev->params.dvs_stat->ver_proj == NULL)
		return -EINVAL;

	/* isp needs to be streaming to get DIS statistics */
	spin_lock_irqsave(&isp->lock, flags);
	if (isp_subdev->streaming != ATOMISP_DEVICE_STREAMING_ENABLED) {
		spin_unlock_irqrestore(&isp->lock, flags);
		return -EINVAL;
	}
	spin_unlock_irqrestore(&isp->lock, flags);

	if (!isp_subdev->params.video_dis_en)
		return -EINVAL;

	if (atomisp_compare_grid(isp_subdev, &stats->grid_info) != 0)
		/* If the grid info in the argument differs from the current
		   grid info, we tell the caller to reset the grid size and
		   try again. */
		return -EAGAIN;

	if (!isp_subdev->params.dvs_proj_data_valid)
		return -EBUSY;

	error = copy_to_user(stats->vertical_projections,
			     isp_subdev->params.dvs_stat->ver_proj,
			     isp_subdev->params.dvs_ver_proj_bytes);

	error |= copy_to_user(stats->horizontal_projections,
			     isp_subdev->params.dvs_stat->hor_proj,
			     isp_subdev->params.dvs_hor_proj_bytes);

	if (error)
		return -EFAULT;

	return 0;
}

int atomisp_set_dis_coefs(struct atomisp_sub_device *isp_subdev,
			  struct atomisp_dis_coefficients *coefs)
{
	int error;

	if (coefs->horizontal_coefficients == NULL ||
	    coefs->vertical_coefficients   == NULL ||
	    isp_subdev->params.dvs_coeff->hor_coefs == NULL ||
	    isp_subdev->params.dvs_coeff->ver_coefs == NULL)
		return -EINVAL;

	error = copy_from_user(isp_subdev->params.dvs_coeff->hor_coefs,
			       coefs->horizontal_coefficients,
			       isp_subdev->params.dvs_hor_coef_bytes);
	if (error)
		return -EFAULT;
	error = copy_from_user(isp_subdev->params.dvs_coeff->ver_coefs,
			       coefs->vertical_coefficients,
			       isp_subdev->params.dvs_ver_coef_bytes);
	if (error)
		return -EFAULT;
	isp_subdev->params.config.dvs_coefs = isp_subdev->params.dvs_coeff;
	isp_subdev->params.dvs_proj_data_valid = false;

	return 0;
}

/*
 * Function to set/get 3A stat from isp
 */
int atomisp_3a_stat(struct atomisp_sub_device *isp_subdev, int flag,
		    struct atomisp_3a_statistics *config)
{
	unsigned long ret;

	if (flag != 0)
		return -EINVAL;

	/* sanity check to avoid writing into unallocated memory. */
	if (isp_subdev->params.s3a_output_bytes == 0)
		return -EINVAL;

	if (atomisp_compare_grid(isp_subdev, &config->grid_info) != 0) {
		/* If the grid info in the argument differs from the current
		   grid info, we tell the caller to reset the grid size and
		   try again. */
		return -EAGAIN;
	}

	/* This is done in the atomisp_s3a_buf_done() */
	if(!isp_subdev->params.s3a_buf_data_valid) {
		v4l2_err(&atomisp_dev, "3a statistics is not valid.\n");
		return -EAGAIN;
	}

	ret = copy_to_user(config->data,
			   isp_subdev->params.s3a_user_stat->data,
			   isp_subdev->params.s3a_output_bytes);
	if (ret) {
		v4l2_err(&atomisp_dev,
			    "copy to user failed: copied %lu bytes\n", ret);
		return -EFAULT;
	}
	return 0;
}

static int __atomisp_set_general_isp_parameters(struct atomisp_sub_device *isp_subdev,
					struct atomisp_parameters *arg)
{
	if (arg->wb_config) {
		if (!isp_subdev->params.config.wb_config)
			isp_subdev->params.config.wb_config = &isp_subdev->params.wb_config;
		memset(isp_subdev->params.config.wb_config, 0 , sizeof(struct ia_css_wb_config));
		if (copy_from_user(isp_subdev->params.config.wb_config, arg->wb_config,
				   sizeof(struct ia_css_wb_config))) {
			isp_subdev->params.config.wb_config = NULL;
			return -EFAULT;
		}
	}

	if (arg->ob_config) {
		if (!isp_subdev->params.config.ob_config)
			isp_subdev->params.config.ob_config = &isp_subdev->params.ob_config;
		memset(isp_subdev->params.config.ob_config, 0 , sizeof(struct ia_css_ob_config));
		if (copy_from_user(isp_subdev->params.config.ob_config, arg->ob_config,
				   sizeof(struct ia_css_ob_config))) {
			isp_subdev->params.config.ob_config = NULL;
			return -EFAULT;
		}
	}

	if (arg->dp_config) {
		if (!isp_subdev->params.config.dp_config)
			isp_subdev->params.config.dp_config = &isp_subdev->params.dp_config;
		memset(isp_subdev->params.config.dp_config, 0 , sizeof(struct ia_css_dp_config));
		if (copy_from_user(isp_subdev->params.config.dp_config, arg->dp_config,
				   sizeof(struct ia_css_dp_config))) {
			isp_subdev->params.config.dp_config = NULL;
			return -EFAULT;
		}
	}

	if (arg->de_config) {
		if (!isp_subdev->params.config.de_config)
			isp_subdev->params.config.de_config = &isp_subdev->params.de_config;
		memset(isp_subdev->params.config.de_config, 0 , sizeof(struct ia_css_de_config));
		if (copy_from_user(isp_subdev->params.config.de_config, arg->de_config,
				   sizeof(struct ia_css_de_config))) {
			isp_subdev->params.config.de_config = NULL;
			return -EFAULT;
		}
	}

	if (arg->ce_config) {
		if (!isp_subdev->params.config.ce_config)
			isp_subdev->params.config.ce_config = &isp_subdev->params.ce_config;
		memset(isp_subdev->params.config.ce_config, 0 , sizeof(struct ia_css_ce_config));
		if (copy_from_user(isp_subdev->params.config.ce_config, arg->ce_config,
				   sizeof(struct ia_css_ce_config))) {
			isp_subdev->params.config.ce_config = NULL;
			return -EFAULT;
		}
	}

	if (arg->nr_config) {
		if (!isp_subdev->params.config.nr_config)
			isp_subdev->params.config.nr_config = &isp_subdev->params.nr_config;
		memset(isp_subdev->params.config.nr_config, 0 , sizeof(struct ia_css_nr_config));
		if (copy_from_user(isp_subdev->params.config.nr_config, arg->nr_config,
				   sizeof(struct ia_css_nr_config))) {
			isp_subdev->params.config.nr_config = NULL;
			return -EFAULT;
		}
	}

	if (arg->ee_config) {
		if (!isp_subdev->params.config.ee_config)
			isp_subdev->params.config.ee_config = &isp_subdev->params.ee_config;
		memset(isp_subdev->params.config.ee_config, 0 , sizeof(struct ia_css_ee_config));
		if (copy_from_user(isp_subdev->params.config.ee_config, arg->ee_config,
				   sizeof(struct ia_css_ee_config))) {
			isp_subdev->params.config.ee_config = NULL;
			return -EFAULT;
		}
	}

	if (arg->tnr_config) {
		if (!isp_subdev->params.config.tnr_config)
			isp_subdev->params.config.tnr_config = &isp_subdev->params.tnr_config;
		memset(isp_subdev->params.config.tnr_config, 0 , sizeof(struct ia_css_tnr_config));
		if (copy_from_user(isp_subdev->params.config.tnr_config, arg->tnr_config,
				   sizeof(struct ia_css_tnr_config))) {
			isp_subdev->params.config.tnr_config = NULL;
			return -EFAULT;
		}
	}

	if (arg->cc_config) {
		if (!isp_subdev->params.config.cc_config)
				isp_subdev->params.config.cc_config = &isp_subdev->params.cc_config;
		memset(isp_subdev->params.config.cc_config, 0 , sizeof(struct ia_css_cc_config));
		if (copy_from_user(isp_subdev->params.config.cc_config, arg->cc_config, sizeof(struct ia_css_cc_config))) {
			isp_subdev->params.config.cc_config = NULL;
			return -EFAULT;
		}
	}

	if (arg->macc_config) {
		if (!isp_subdev->params.config.macc_table)
				isp_subdev->params.config.macc_table = &isp_subdev->params.macc_table;
		memset(isp_subdev->params.config.macc_table, 0 , sizeof(struct ia_css_macc_table));
		if (copy_from_user(isp_subdev->params.config.macc_table,
				   &arg->macc_config->table,
				   sizeof(struct ia_css_macc_table))) {
			isp_subdev->params.config.macc_table = NULL;
			return -EFAULT;
		}
		isp_subdev->params.color_effect = arg->macc_config->color_effect;
	}

	if (arg->gc_config) {
		if (!isp_subdev->params.config.gc_config)
			isp_subdev->params.config.gc_config = &isp_subdev->params.gc_config;
		memset(isp_subdev->params.config.gc_config, 0 , sizeof(struct ia_css_gc_config));
		if (copy_from_user(isp_subdev->params.config.gc_config, arg->gc_config,
				   sizeof(*arg->gc_config))) {
			isp_subdev->params.config.gc_config = NULL;
			return -EFAULT;
		}
	}

	if (arg->a3a_config) {
		if (!isp_subdev->params.config.s3a_config)
			isp_subdev->params.config.s3a_config = &isp_subdev->params.s3a_config;
		memset(isp_subdev->params.config.s3a_config, 0 , sizeof(struct ia_css_3a_config));
		if (copy_from_user(isp_subdev->params.config.s3a_config, arg->a3a_config,
				   sizeof(*arg->a3a_config))) {
			isp_subdev->params.config.s3a_config = NULL;
			return -EFAULT;
		}
	}

	return 0;
}

static int __atomisp_set_lsc_table(struct atomisp_sub_device *isp_subdev,
			struct atomisp_shading_table *user_st)
{
	unsigned int i;
	unsigned int len_table;
	struct ia_css_shading_table *shading_table;
	struct ia_css_shading_table *old_shading_table;
	struct atomisp_device *isp = isp_subdev->isp;

	if (!user_st)
		return 0;

	old_shading_table = isp->inputs[isp_subdev->input_curr].shading_table;

	/* user config is to disable the shading table. */
	if (!user_st->enable) {
		shading_table = NULL;
		goto set_lsc;
	}

	/* Setting a new table. Validate first - all tables must be set */
	for (i = 0; i < ATOMISP_NUM_SC_COLORS; i++) {
		if (!user_st->data[i])
			return -EINVAL;
	}

	/* Shading table size per color */
	if (user_st->width > SH_CSS_MAX_SCTBL_WIDTH_PER_COLOR ||
		user_st->height > SH_CSS_MAX_SCTBL_HEIGHT_PER_COLOR)
		return -EINVAL;

	shading_table = ia_css_shading_table_alloc(user_st->width,
			user_st->height);
	if (!shading_table)
			return -ENOMEM;

	len_table = user_st->width * user_st->height * ATOMISP_SC_TYPE_SIZE;
	for (i = 0; i < ATOMISP_NUM_SC_COLORS; i++) {
		if (copy_from_user(shading_table->data[i],
			user_st->data[i], len_table)) {
			ia_css_shading_table_free(shading_table);
			return -EFAULT;
		}

	}
	shading_table->sensor_width = user_st->sensor_width;
	shading_table->sensor_height = user_st->sensor_height;
	shading_table->fraction_bits = user_st->fraction_bits;

set_lsc:
	/* set LSC to CSS */
	isp->inputs[isp_subdev->input_curr].shading_table = shading_table;
	isp_subdev->params.config.shading_table = shading_table;
	isp_subdev->params.sc_en = shading_table != NULL;

	if (old_shading_table)
		ia_css_shading_table_free(old_shading_table);

	return 0;
}

static int __atomisp_set_morph_table(struct atomisp_sub_device *isp_subdev,
				struct atomisp_morph_table *user_morph_table)
{
	int ret = -EFAULT;
	unsigned int i;
	struct atomisp_device *isp = isp_subdev->isp;
	struct ia_css_morph_table *morph_table;
	struct ia_css_morph_table *old_morph_table;

	if (!user_morph_table)
		return 0;

	old_morph_table = isp->inputs[isp_subdev->input_curr].morph_table;

	morph_table = ia_css_morph_table_allocate(user_morph_table->width,
				user_morph_table->height);
	if (!morph_table)
		return -ENOMEM;

	for (i = 0; i < IA_CSS_MORPH_TABLE_NUM_PLANES; i++) {
		if (copy_from_user(morph_table->coordinates_x[i],
			user_morph_table->coordinates_x[i],
			user_morph_table->height * user_morph_table->width *
			sizeof(*user_morph_table->coordinates_x[i])))
			goto error;

		if (copy_from_user(morph_table->coordinates_y[i],
			user_morph_table->coordinates_y[i],
			user_morph_table->height * user_morph_table->width *
			sizeof(*user_morph_table->coordinates_y[i])))
			goto error;
	}

	isp->inputs[isp_subdev->input_curr].morph_table = morph_table;
	if (isp_subdev->params.gdc_cac_en)
		isp_subdev->params.config.morph_table = morph_table;

	if (old_morph_table)
		ia_css_morph_table_free(old_morph_table);

	return 0;

error:
	if (morph_table)
		ia_css_morph_table_free(morph_table);
	return ret;
}

/*
* Function to configure ISP parameters
*/
int atomisp_set_parameters(struct atomisp_sub_device *isp_subdev,
			struct atomisp_parameters *arg)
{
	int ret;

	ret = __atomisp_set_general_isp_parameters(isp_subdev, arg);
	if (ret)
		return ret;

	ret = __atomisp_set_lsc_table(isp_subdev, arg->shading_table);
	if (ret)
		return ret;

	ret = __atomisp_set_morph_table(isp_subdev, arg->morph_table);
	if (ret)
		return ret;

	/* indicate to CSS that we have parametes to be updated */
	isp_subdev->params.css_update_params_needed = true;

	if (isp_subdev->css2_basis.stream
		&& (isp_subdev->css2_basis.stream_state != CSS2_STREAM_STARTED
		|| isp_subdev->run_mode->val
			== ATOMISP_RUN_MODE_STILL_CAPTURE)) {
		atomisp_css_update_isp_params(isp_subdev);
		isp_subdev->params.css_update_params_needed = false;
	}

	return 0;
}

/*
 * Function to set/get isp parameters to isp
 */
int atomisp_param(struct atomisp_sub_device *isp_subdev, int flag,
		  struct atomisp_parm *config)
{
	/* Read parameter for 3A bianry info */
	if (flag == 0) {
		if (&config->info == NULL) {
			v4l2_err(&atomisp_dev,
				    "ERROR: NULL pointer in grid_info\n");
			return -EINVAL;
		}
		atomisp_curr_user_grid_info(isp_subdev, &config->info);
		return 0;
	}

	if (sizeof(config->wb_config) != sizeof(isp_subdev->params.config.wb_config))
		goto INVALID_PARM;
	if (sizeof(config->cc_config) != sizeof(isp_subdev->params.config.cc_config))
		goto INVALID_PARM;
	if (sizeof(config->ob_config) != sizeof(isp_subdev->params.config.ob_config))
		goto INVALID_PARM;
	if (sizeof(config->de_config) != sizeof(isp_subdev->params.config.de_config))
		goto INVALID_PARM;
	if (sizeof(config->ce_config) != sizeof(isp_subdev->params.config.ce_config))
		goto INVALID_PARM;
	if (sizeof(config->dp_config) != sizeof(isp_subdev->params.config.dp_config))
		goto INVALID_PARM;
	if (sizeof(config->nr_config) != sizeof(isp_subdev->params.config.nr_config))
		goto INVALID_PARM;
	if (sizeof(config->ee_config) != sizeof(isp_subdev->params.config.ee_config))
		goto INVALID_PARM;
	if (sizeof(config->tnr_config) != sizeof(isp_subdev->params.config.tnr_config))
		goto INVALID_PARM;

	memcpy(&isp_subdev->params.wb_config, &config->wb_config,
	       sizeof(struct ia_css_wb_config));
	memcpy(&isp_subdev->params.cc_config, &config->cc_config,
	       sizeof(struct ia_css_cc_config));
	memcpy(&isp_subdev->params.ob_config, &config->ob_config,
	       sizeof(struct ia_css_ob_config));
	memcpy(&isp_subdev->params.dp_config, &config->dp_config,
	       sizeof(struct ia_css_dp_config));
	memcpy(&isp_subdev->params.de_config, &config->de_config,
	       sizeof(struct ia_css_de_config));
	memcpy(&isp_subdev->params.ce_config, &config->ce_config,
	       sizeof(struct ia_css_ce_config));
	memcpy(&isp_subdev->params.nr_config, &config->nr_config,
	       sizeof(struct ia_css_nr_config));
	memcpy(&isp_subdev->params.ee_config, &config->ee_config,
	       sizeof(struct ia_css_ee_config));
	memcpy(&isp_subdev->params.tnr_config, &config->tnr_config,
	       sizeof(struct ia_css_tnr_config));

	isp_subdev->params.config.wb_config = &isp_subdev->params.wb_config;
	isp_subdev->params.config.cc_config = &isp_subdev->params.cc_config;
	isp_subdev->params.config.ob_config = &isp_subdev->params.ob_config;
	isp_subdev->params.config.dp_config = &isp_subdev->params.dp_config;
	isp_subdev->params.config.de_config = &isp_subdev->params.de_config;
	isp_subdev->params.config.ce_config = &isp_subdev->params.ce_config;
	isp_subdev->params.config.nr_config = &isp_subdev->params.nr_config;
	isp_subdev->params.config.ee_config = &isp_subdev->params.ee_config;
	isp_subdev->params.config.tnr_config = &isp_subdev->params.tnr_config;

	if (isp_subdev->params.color_effect == V4L2_COLORFX_NEGATIVE) {
		config->cc_config.matrix[3] = -config->cc_config.matrix[3];
		config->cc_config.matrix[4] = -config->cc_config.matrix[4];
		config->cc_config.matrix[5] = -config->cc_config.matrix[5];
		config->cc_config.matrix[6] = -config->cc_config.matrix[6];
		config->cc_config.matrix[7] = -config->cc_config.matrix[7];
		config->cc_config.matrix[8] = -config->cc_config.matrix[8];
	}

	if (isp_subdev->params.color_effect != V4L2_COLORFX_SEPIA &&
	    isp_subdev->params.color_effect != V4L2_COLORFX_BW) {
		memcpy(&isp_subdev->params.cc_config, &config->cc_config,
		       sizeof(struct ia_css_cc_config));
		isp_subdev->params.config.cc_config = &isp_subdev->params.cc_config;
	}

	isp_subdev->params.css_update_params_needed = true;

	return 0;

INVALID_PARM:
	v4l2_err(&atomisp_dev,
		"%s: incompatible param.\n", __func__);
	return -EINVAL;
}

/*
 * Function to configure color effect of the image
 */
int atomisp_color_effect(struct atomisp_sub_device *isp_subdev, int flag, __s32 *effect)
{
	struct ia_css_cc_config *cc_config = NULL;
	struct ia_css_macc_table *macc_table = NULL;
	struct ia_css_ctc_table *ctc_table = NULL;
	int ret = 0;
	struct atomisp_device *isp = isp_subdev->isp;
	struct v4l2_control control;

	if (flag == 0) {
		*effect = isp_subdev->params.color_effect;
		return 0;
	}


	control.id = V4L2_CID_COLORFX;
	control.value = *effect;
	ret = v4l2_subdev_call(isp->inputs[isp_subdev->input_curr].camera,
				core, s_ctrl, &control);
	/*
	 * if set color effect to sensor successfully, return
	 * 0 directly.
	 */
	if (!ret) {
		isp_subdev->params.color_effect = (u32)*effect;
		return 0;
	}

	if (*effect == isp_subdev->params.color_effect)
		return 0;

	/*
	 * set macc enable to false by default:
	 * when change from macc to sepia/mono,
	 * isp_subdev->params.macc_en should be set to false.
	 */
	isp_subdev->params.macc_en = false;

	switch (*effect) {
	case V4L2_COLORFX_NONE:
		macc_table = NULL;
		isp_subdev->params.macc_en = true;
		break;
	case V4L2_COLORFX_SEPIA:
		cc_config = &sepia_cc_config;
		break;
	case V4L2_COLORFX_NEGATIVE:
		cc_config = &nega_cc_config;
		break;
	case V4L2_COLORFX_BW:
		cc_config = &mono_cc_config;
		break;
	case V4L2_COLORFX_SKY_BLUE:
		macc_table = &blue_macc_table;
		isp_subdev->params.macc_en = true;
		break;
	case V4L2_COLORFX_GRASS_GREEN:
		macc_table = &green_macc_table;
		isp_subdev->params.macc_en = true;
		break;
	case V4L2_COLORFX_SKIN_WHITEN_LOW:
		macc_table = &skin_low_macc_table;
		isp_subdev->params.macc_en = true;
		break;
	case V4L2_COLORFX_SKIN_WHITEN:
		macc_table = &skin_medium_macc_table;
		isp_subdev->params.macc_en = true;
		break;
	case V4L2_COLORFX_SKIN_WHITEN_HIGH:
		macc_table = &skin_high_macc_table;
		isp_subdev->params.macc_en = true;
		break;
	case V4L2_COLORFX_VIVID:
		ctc_table = &vivid_ctc_table;
		break;
	default:
		return -EINVAL;
	}
	atomisp_update_capture_mode(isp_subdev);

	if (cc_config)
		isp_subdev->params.config.cc_config = cc_config;
	if (macc_table)
		isp_subdev->params.config.macc_table = macc_table;
	if (ctc_table)
		isp_subdev->params.config.ctc_table = ctc_table;
	isp_subdev->params.color_effect = (u32)*effect;
	isp_subdev->params.css_update_params_needed = true;
	return 0;
}

/*
 * Function to configure bad pixel correction
 */
int atomisp_bad_pixel(struct atomisp_sub_device *isp_subdev, int flag, __s32 *value)
{

	if (flag == 0) {
		*value = isp_subdev->params.bad_pixel_en;
		return 0;
	}
	isp_subdev->params.bad_pixel_en = !!*value;

	return 0;
}

/*
 * Function to configure bad pixel correction params
 */
int atomisp_bad_pixel_param(struct atomisp_sub_device *isp_subdev, int flag,
			    struct atomisp_dp_config *config)
{
	if (config == NULL)
		return -EINVAL;

	if (sizeof(*config) != sizeof(isp_subdev->params.config.dp_config)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

	if (flag == 0) {
		struct ia_css_dp_config dp_config;
		struct ia_css_isp_config isp_config;
		if (!isp_subdev->css2_basis.stream) {
			v4l2_err(&atomisp_dev,
				 "%s called after streamoff, skipping.\n",
				 __func__);
			return -EINVAL;
		}
		memset(&dp_config, 0, sizeof(struct ia_css_dp_config));
		memset(&isp_config, 0, sizeof(struct ia_css_isp_config));
		isp_config.dp_config = &dp_config;
		ia_css_stream_get_isp_config(isp_subdev->css2_basis.stream, &isp_config);
		/* Get bad pixel from current setup */
		memcpy(config, isp_subdev->params.config.dp_config, sizeof(*config));
	} else {
		if (!isp_subdev->params.config.dp_config)
			isp_subdev->params.config.dp_config = &isp_subdev->params.dp_config;
		/* Set bad pixel to isp parameters */
		memcpy(isp_subdev->params.config.dp_config, config, sizeof(*config));
		isp_subdev->params.css_update_params_needed = true;
	}

	return 0;
}

/*
 * Function to enable/disable video image stablization
 */
int atomisp_video_stable(struct atomisp_sub_device *isp_subdev, int flag, __s32 * value)
{
	if (flag == 0)
		*value = isp_subdev->params.video_dis_en;
	else
		isp_subdev->params.video_dis_en = !!*value;

	return 0;
}

/*
 * Function to configure fixed pattern noise
 */
int atomisp_fixed_pattern(struct atomisp_sub_device *isp_subdev, int flag, __s32 * value)
{

	if (flag == 0) {
		*value = isp_subdev->params.fpn_en;
		return 0;
	}

	if (*value == 0) {
		isp_subdev->params.fpn_en = 0;
		return 0;
	}

	/* Add function to get black from from sensor with shutter off */
	return 0;
}

static unsigned int
atomisp_bytesperline_to_padded_width(unsigned int bytesperline,
				     enum ia_css_frame_format format)
{
	switch (format) {
	case IA_CSS_FRAME_FORMAT_UYVY:
	case IA_CSS_FRAME_FORMAT_YUYV:
	case IA_CSS_FRAME_FORMAT_RAW:
	case IA_CSS_FRAME_FORMAT_RGB565:
		return bytesperline/2;
	case IA_CSS_FRAME_FORMAT_RGBA888:
		return bytesperline/4;
	/* The following cases could be removed, but we leave them
	   in to document the formats that are included. */
	case IA_CSS_FRAME_FORMAT_NV11:
	case IA_CSS_FRAME_FORMAT_NV12:
	case IA_CSS_FRAME_FORMAT_NV16:
	case IA_CSS_FRAME_FORMAT_NV21:
	case IA_CSS_FRAME_FORMAT_NV61:
	case IA_CSS_FRAME_FORMAT_YV12:
	case IA_CSS_FRAME_FORMAT_YV16:
	case IA_CSS_FRAME_FORMAT_YUV420:
	case IA_CSS_FRAME_FORMAT_YUV420_16:
	case IA_CSS_FRAME_FORMAT_YUV422:
	case IA_CSS_FRAME_FORMAT_YUV422_16:
	case IA_CSS_FRAME_FORMAT_YUV444:
	case IA_CSS_FRAME_FORMAT_YUV_LINE:
	case IA_CSS_FRAME_FORMAT_QPLANE6:
	case IA_CSS_FRAME_FORMAT_BINARY_8:
	default:
		return bytesperline;
	}
}

static int
atomisp_v4l2_framebuffer_to_ia_css_frame(const struct v4l2_framebuffer *arg,
					 struct ia_css_frame **result)
{
	struct ia_css_frame *res;
	unsigned int padded_width;
	enum ia_css_frame_format sh_format;
	char *tmp_buf = NULL;
	int ret = 0;

	sh_format = v4l2_fmt_to_sh_fmt(arg->fmt.pixelformat);
	padded_width = atomisp_bytesperline_to_padded_width(
					arg->fmt.bytesperline, sh_format);

	/* Note: the padded width on an ia_css_frame is in elements, not in
	   bytes. The RAW frame we use here should always be a 16bit RAW
	   frame. This is why we bytesperline/2 is equal to the padded with */
	if (ia_css_frame_allocate(&res, arg->fmt.width, arg->fmt.height,
				  sh_format, padded_width, 0)) {
		ret = -ENOMEM;
		goto err;
	}

	tmp_buf = vmalloc(arg->fmt.sizeimage);
	if (!tmp_buf) {
		ret = -ENOMEM;
		goto err;
	}
	if (copy_from_user(tmp_buf, (void __user __force *)arg->base,
			   arg->fmt.sizeimage)) {
		ret = -EFAULT;
		goto err;
	}

	if (hmm_store(res->data, tmp_buf, arg->fmt.sizeimage)) {
		ret = -EINVAL;
		goto err;
	}

err:
	if (ret && res)
		ia_css_frame_free(res);
	if (tmp_buf)
		vfree(tmp_buf);
	if (ret == 0)
		*result = res;
	return ret;
}

/*
 * Function to configure fixed pattern noise table
 */
int atomisp_fixed_pattern_table(struct atomisp_sub_device *isp_subdev,
				struct v4l2_framebuffer *arg)
{
	struct ia_css_frame *raw_black_frame = NULL;
	int ret;

	if (arg == NULL)
		return -EINVAL;

	ret = atomisp_v4l2_framebuffer_to_ia_css_frame(arg, &raw_black_frame);
	if (ret)
		return ret;

	if (sh_css_set_black_frame(isp_subdev->css2_basis.stream, raw_black_frame) != IA_CSS_SUCCESS)
		ret = -ENOMEM;

	ia_css_frame_free(raw_black_frame);
	return ret;
}

/*
 * Function to configure false color correction
 */
int atomisp_false_color(struct atomisp_sub_device *isp_subdev, int flag, __s32 *value)
{
	/* Get nr config from current setup */
	if (flag == 0) {
		*value = isp_subdev->params.false_color;
		return 0;
	}

	/* Set nr config to isp parameters */
	if (*value) {
		isp_subdev->params.config.de_config = NULL;
	} else {
		if (!isp_subdev->params.config.de_config)
			isp_subdev->params.config.de_config = &isp_subdev->params.de_config;
		isp_subdev->params.config.de_config->pixelnoise = 0;
	}
	isp_subdev->params.css_update_params_needed = true;
	isp_subdev->params.false_color = *value;
	return 0;
}

/*
 * Function to configure bad pixel correction params
 */
int atomisp_false_color_param(struct atomisp_sub_device *isp_subdev, int flag,
			      struct atomisp_de_config *config)
{
	if (config == NULL)
		return -EINVAL;

	if (sizeof(*config) != sizeof(isp_subdev->params.config.de_config)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

	if (flag == 0) {
		struct ia_css_de_config de_config;
		struct ia_css_isp_config isp_config;
		if (!isp_subdev->css2_basis.stream) {
			v4l2_err(&atomisp_dev,
				 "%s called after streamoff, skipping.\n",
				 __func__);
			return -EINVAL;
		}
		memset(&de_config, 0, sizeof(struct ia_css_de_config));
		memset(&isp_config, 0, sizeof(struct ia_css_isp_config));
		isp_config.de_config = &de_config;
		ia_css_stream_get_isp_config(isp_subdev->css2_basis.stream, &isp_config);
		/* Get false color from current setup */
		memcpy(config, &de_config, sizeof(*config));
	} else {
		if (!isp_subdev->params.config.de_config)
			isp_subdev->params.config.de_config = &isp_subdev->params.de_config;
		/* Set false color to isp parameters */
		memcpy(isp_subdev->params.config.de_config, config, sizeof(*config));
		isp_subdev->params.css_update_params_needed = true;
	}

	return 0;
}

/*
 * Function to configure white balance params
 */
int atomisp_white_balance_param(struct atomisp_sub_device *isp_subdev, int flag,
	struct atomisp_wb_config *config)
{
	if (config == NULL)
		return -EINVAL;

	if (sizeof(*config) != sizeof(isp_subdev->params.config.wb_config)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

	if (flag == 0) {
		struct ia_css_wb_config wb_config;
		struct ia_css_isp_config isp_config;
		if (!isp_subdev->css2_basis.stream) {
			v4l2_err(&atomisp_dev,
				 "%s called after streamoff, skipping.\n",
				 __func__);
			return -EINVAL;
		}
		memset(&wb_config, 0, sizeof(struct ia_css_wb_config));
		memset(&isp_config, 0, sizeof(struct ia_css_isp_config));
		isp_config.wb_config = &wb_config;
		ia_css_stream_get_isp_config(isp_subdev->css2_basis.stream, &isp_config);
		/* Get white balance from current setup */
		memcpy(config, &wb_config, sizeof(*config));
	} else {
		if (!isp_subdev->params.config.wb_config)
			isp_subdev->params.config.wb_config = &isp_subdev->params.wb_config;
		/* Set white balance to isp parameters */
		memcpy(isp_subdev->params.config.wb_config, config, sizeof(*config));
		isp_subdev->params.css_update_params_needed = true;
	}

	return 0;
}

int atomisp_3a_config_param(struct atomisp_sub_device *isp_subdev, int flag,
			    struct atomisp_3a_config *config)
{
	v4l2_dbg(5, dbg_level, &atomisp_dev, ">%s %d\n", __func__, flag);
	if (config == NULL)
		return -EINVAL;

	if (sizeof(*config) != sizeof(isp_subdev->params.config.s3a_config)) {
		v4l2_err(&atomisp_dev,
			"%s: incompatible param.\n", __func__);
		return -EINVAL;
	}

	if (flag == 0) {
		struct ia_css_3a_config s3a_config;
		struct ia_css_isp_config isp_config;
		if (!isp_subdev->css2_basis.stream) {
			v4l2_err(&atomisp_dev,
				 "%s called after streamoff, skipping.\n",
				 __func__);
			return -EINVAL;
		}
		memset(&s3a_config, 0, sizeof(struct ia_css_3a_config));
		memset(&isp_config, 0, sizeof(struct ia_css_isp_config));
		isp_config.s3a_config = &s3a_config;
		ia_css_stream_get_isp_config(isp_subdev->css2_basis.stream, &isp_config);
		/* Get white balance from current setup */
		memcpy(config, &s3a_config, sizeof(*config));
	} else {
		if (!isp_subdev->params.config.s3a_config)
			isp_subdev->params.config.s3a_config = &isp_subdev->params.s3a_config;
		/* Set white balance to isp parameters */
		memcpy(isp_subdev->params.config.s3a_config, config, sizeof(*config));
		isp_subdev->params.css_update_params_needed = true;
		/* isp_subdev->params.s3a_buf_data_valid = false; */
	}

	v4l2_dbg(5, dbg_level, &atomisp_dev, "<%s %d\n", __func__, flag);
	return 0;
}

/*
 * Function to enable/disable lens shading correction
 */
int atomisp_shading_correction(struct atomisp_sub_device *isp_subdev, int flag,
				       __s32 *value)
{
	struct atomisp_device *isp = isp_subdev->isp;
	if (flag == 0) {
		*value = isp_subdev->params.sc_en;
		return 0;
	}

	if (*value == 0)
		isp_subdev->params.config.shading_table = NULL;
	else
		isp_subdev->params.config.shading_table =
		    isp->inputs[isp_subdev->input_curr].shading_table;

	isp_subdev->params.sc_en = *value;

	return 0;
}

/*
 * Function to setup digital zoom
 */
int atomisp_digital_zoom(struct atomisp_sub_device *isp_subdev, int flag, __s32 *value)
{
	u32 zoom;
	unsigned int max_zoom =
		IS_MRFLD ? MRFLD_MAX_ZOOM_FACTOR : MFLD_MAX_ZOOM_FACTOR;

	if (flag == 0) {
		struct ia_css_dz_config dz_config;  /**< Digital Zoom */
		struct ia_css_isp_config isp_config;
		if (!isp_subdev->css2_basis.stream) {
			v4l2_err(&atomisp_dev,
				 "%s called after streamoff, skipping.\n",
				 __func__);
			return -EINVAL;
		}
		memset(&dz_config, 0, sizeof(struct ia_css_dz_config));
		memset(&isp_config, 0, sizeof(struct ia_css_isp_config));
		isp_config.dz_config = &dz_config;
		ia_css_stream_get_isp_config(isp_subdev->css2_basis.stream, &isp_config);
		*value = max_zoom - dz_config.dx;
	} else {
		if (*value < 0)
			return -EINVAL;

		zoom = max_zoom - min_t(u32, max_zoom, (*value));

		if (!isp_subdev->params.config.dz_config)
			isp_subdev->params.config.dz_config = &isp_subdev->params.dz_config;

		if (zoom == isp_subdev->params.config.dz_config->dx &&
			 zoom == isp_subdev->params.config.dz_config->dy) {
			v4l2_dbg(3, dbg_level, &atomisp_dev, "same zoom scale. skipped.\n");
			return 0;
		}

		memset(isp_subdev->params.config.dz_config, 0, sizeof(struct ia_css_dz_config));
		isp_subdev->params.dz_config.dx = zoom;
		isp_subdev->params.dz_config.dy = zoom;
		v4l2_dbg(3, dbg_level, &atomisp_dev, "%s, zoom: %d\n",
			 __func__, zoom);

		isp_subdev->params.css_update_params_needed = true;
	}

	return 0;
}

/*
 * Function to get sensor specific info for current resolution,
 * which will be used for auto exposure conversion.
 */
int atomisp_get_sensor_mode_data(struct atomisp_sub_device *isp_subdev,
				 struct atomisp_sensor_mode_data *config)
{
	struct camera_mipi_info *mipi_info;
	struct atomisp_device *isp = isp_subdev->isp;

	mipi_info = atomisp_to_sensor_mipi_info(
		isp->inputs[isp_subdev->input_curr].camera);
	if (mipi_info == NULL)
		return -EINVAL;

	memcpy(config, &mipi_info->data, sizeof(*config));
	return 0;
}

int atomisp_get_fmt(struct video_device *vdev, struct v4l2_format *f)
{
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
	    f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		v4l2_err(&atomisp_dev,
			    "unsupported v4l2 buf type\n");
		return -EINVAL;
	}

	memset(f, 0, sizeof(struct v4l2_format));
	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	/* VIDIOC_S_FMT already called,*/
	/* return fmt set by app */
	if (pipe->pix.width != 0) {
		memcpy(&f->fmt.pix, &pipe->pix,
			sizeof(struct v4l2_pix_format));
	} else {
		v4l2_dbg(4, dbg_level, &atomisp_dev, "Pipe width was 0, setting default 640x480\n");
		f->fmt.pix.width = 640;
		f->fmt.pix.height = 480;
		f->fmt.pix.pixelformat = atomisp_output_fmts[0].pixelformat;
		f->fmt.pix.bytesperline =
			get_pixel_depth(f->fmt.pix.pixelformat) *
						f->fmt.pix.width;
		f->fmt.pix.sizeimage = f->fmt.pix.height *
						f->fmt.pix.bytesperline;
		f->fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
	}

	return 0;
}


/* This function looks up the closest available resolution. */
int atomisp_try_fmt(struct video_device *vdev, struct v4l2_format *f,
						bool *res_overflow)
{
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct v4l2_mbus_framefmt snr_mbus_fmt;
	const struct atomisp_format_bridge *fmt;
	int ret;
	struct atomisp_sub_device *isp_subdev = atomisp_to_sub_device(atomisp_to_video_pipe(vdev));

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		v4l2_err(&atomisp_dev, "Wrong v4l2 buf type\n");
		return -EINVAL;
	}

	if (isp->inputs[isp_subdev->input_curr].camera == NULL)
		return -EINVAL;

	fmt = atomisp_get_format_bridge(f->fmt.pix.pixelformat);
	if (fmt == NULL) {
		v4l2_err(&atomisp_dev, "unsupported pixelformat!\n");
		fmt = atomisp_output_fmts;
	}

	if (isp->inputs[isp_subdev->input_curr].type == TEST_PATTERN)
		return 0;
	snr_mbus_fmt.code = fmt->mbus_code;
	snr_mbus_fmt.width = f->fmt.pix.width;
	snr_mbus_fmt.height = f->fmt.pix.height;

	dev_dbg(isp->dev, "try_mbus_fmt: asking for %ux%u\n",
		snr_mbus_fmt.width, snr_mbus_fmt.height);

	ret = v4l2_subdev_call(isp->inputs[isp_subdev->input_curr].camera,
			video, try_mbus_fmt, &snr_mbus_fmt);
	if (ret)
		return ret;

	dev_dbg(isp->dev, "try_mbus_fmt: got %ux%u mbus code = %d\n",
		snr_mbus_fmt.width, snr_mbus_fmt.height, snr_mbus_fmt.code);
	fmt = atomisp_get_format_bridge_from_mbus(snr_mbus_fmt.code);

	if (fmt == NULL){
		v4l2_err(&atomisp_dev, "unknown sensor format.\n");
		return -EINVAL;
	}

	f->fmt.pix.pixelformat = fmt->pixelformat;

	if (snr_mbus_fmt.width < f->fmt.pix.width
	    && snr_mbus_fmt.height < f->fmt.pix.height) {
		f->fmt.pix.width = snr_mbus_fmt.width;
		f->fmt.pix.height = snr_mbus_fmt.width;
		/* Set the flag when resolution requested is
		 * beyond the max value supported by sensor
		 */
		if (res_overflow != NULL)
			*res_overflow = true;
	}

	/* app vs isp */
	f->fmt.pix.width = rounddown(
		clamp_t(u32, f->fmt.pix.width, ATOM_ISP_MIN_WIDTH,
			ATOM_ISP_MAX_WIDTH), ATOM_ISP_STEP_WIDTH);
	f->fmt.pix.height = rounddown(
		clamp_t(u32, f->fmt.pix.height, ATOM_ISP_MIN_HEIGHT,
			ATOM_ISP_MAX_HEIGHT), ATOM_ISP_STEP_HEIGHT);

	return 0;
}

static int
atomisp_try_fmt_file(struct atomisp_device *isp, struct v4l2_format *f)
{
	u32 width = f->fmt.pix.width;
	u32 height = f->fmt.pix.height;
	u32 pixelformat = f->fmt.pix.pixelformat;
	enum v4l2_field field = f->fmt.pix.field;
	u32 depth;

	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		v4l2_err(&atomisp_dev, "Wrong v4l2 buf type\n");
		return -EINVAL;
	}

	if (!atomisp_get_format_bridge(pixelformat)) {
		v4l2_err(&atomisp_dev, "Wrong output pixelformat\n");
		return -EINVAL;
	}

	depth = get_pixel_depth(pixelformat);

	if (!field || field == V4L2_FIELD_ANY)
		field = V4L2_FIELD_NONE;
	else if (field != V4L2_FIELD_NONE) {
		v4l2_err(&atomisp_dev, "Wrong output field\n");
		return -EINVAL;
	}

	f->fmt.pix.field = field;
	f->fmt.pix.width = clamp_t(u32,
				   rounddown(width, (u32)ATOM_ISP_STEP_WIDTH),
				   ATOM_ISP_MIN_WIDTH, ATOM_ISP_MAX_WIDTH);
	f->fmt.pix.height = clamp_t(u32, rounddown(height,
						   (u32)ATOM_ISP_STEP_HEIGHT),
				    ATOM_ISP_MIN_HEIGHT, ATOM_ISP_MAX_HEIGHT);
	f->fmt.pix.bytesperline = (width * depth) >> 3;

	return 0;
}

static mipi_port_ID_t __get_mipi_port(enum atomisp_camera_port port)
{
	switch (port) {
	case ATOMISP_CAMERA_PORT_PRIMARY:
		return MIPI_PORT0_ID;
	case ATOMISP_CAMERA_PORT_SECONDARY:
		return MIPI_PORT1_ID;
	case ATOMISP_CAMERA_PORT_THIRD:
		if (MIPI_PORT1_ID + 1 != N_MIPI_PORT_ID)
			return MIPI_PORT1_ID + 1;
		/* go through down for else case */
	default:
		v4l2_err(&atomisp_dev, "unsupported port: %d\n", port);
		return MIPI_PORT0_ID;
	}
}

static void __enable_continuous_vf(struct atomisp_sub_device *isp_subdev, bool enable)
{
	struct atomisp_device *isp = isp_subdev->isp;
	ia_css_capture_set_mode(isp_subdev, IA_CSS_CAPTURE_MODE_PRIMARY);
	ia_css_capture_enable_online(isp_subdev, !enable);
	ia_css_preview_enable_online(isp_subdev, !enable);
	ia_css_enable_continuous(isp_subdev, enable);

	sh_css_enable_cont_capt(enable, enable);
	if (!enable) {
		ia_css_enable_raw_binning(isp_subdev, false);
		ia_css_input_set_two_pixels_per_clock(isp_subdev, false);
	}

	ia_css_input_set_mode(isp_subdev, get_input_mode(isp->inputs[isp_subdev->input_curr].type, isp));
}


void atomisp_configure_viewfinder(struct atomisp_sub_device *isp_subdev, int width,
		int height, unsigned int source_pad)
{
	if (isp_subdev->fmt_auto->val
			|| !isp_subdev->enable_vfpp->val) {
		struct v4l2_rect vf_size;
		struct v4l2_mbus_framefmt vf_ffmt;

		memset(&vf_size, 0, sizeof(vf_size));
		if (width < 640 || height < 480) {
			vf_size.width = width;
			vf_size.height = height;
		} else {
			vf_size.width = 640;
			vf_size.height = 480;
		}

		memset(&vf_ffmt, 0, sizeof(vf_ffmt));
		/* FIXME: proper format name for this one. See
		   atomisp_output_fmts[] in atomisp_v4l2.c */
		vf_ffmt.code = 0x8001;

		atomisp_subdev_set_selection(isp_subdev, NULL,
				V4L2_SUBDEV_FORMAT_ACTIVE,
				ATOMISP_SUBDEV_PAD_SOURCE_VF,
				V4L2_SEL_TGT_COMPOSE, 0, &vf_size);
		atomisp_subdev_set_ffmt(isp_subdev, NULL,
				V4L2_SUBDEV_FORMAT_ACTIVE,
				ATOMISP_SUBDEV_PAD_SOURCE_VF, &vf_ffmt);

		//CSS2.0 supports NV12 as VF format
		isp_subdev->video_out_vf.sh_fmt = IA_CSS_FRAME_FORMAT_NV12;

		if (isp_subdev->run_mode->val == ATOMISP_RUN_MODE_VIDEO
				|| !isp_subdev->enable_vfpp->val)
			ia_css_video_configure_viewfinder(isp_subdev,
					vf_size.width, vf_size.height,
					isp_subdev->video_out_vf.sh_fmt);
		else if (source_pad != ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW)
			ia_css_capture_configure_viewfinder(isp_subdev,
					vf_size.width, vf_size.height,
					isp_subdev->video_out_vf.sh_fmt);
	}
}

void atomisp_set_continuous_vf(struct atomisp_sub_device *isp_subdev, enum ia_css_stream_format sh_input_format,
		unsigned int width, unsigned int height, int *raw_override_p)
{

	/* get input format */
	sh_input_format = isp_subdev->css2_basis.stream_config.format;
	v4l2_dbg(5, dbg_level, &atomisp_dev, "ENTER %s continuous_vf enable?=%d\n",__func__,isp_subdev->params.continuous_vf);
	if (isp_subdev->params.continuous_vf) {
		if (isp_subdev->run_mode->val != ATOMISP_RUN_MODE_VIDEO) {
			__enable_continuous_vf(isp_subdev, true);
			/* enable only if resolution is equal or above 5M */
			if (width >= 2576 || height >= 1936) {
				ia_css_enable_raw_binning(isp_subdev, true);
				ia_css_input_set_two_pixels_per_clock(isp_subdev, false);
			}
		} else {
			__enable_continuous_vf(isp_subdev, false);
		}
	}
	else if (sh_input_format == IA_CSS_STREAM_FORMAT_RGB_565
			|| sh_input_format == IA_CSS_STREAM_FORMAT_RGB_888) {
		/* make sure it sets the right configurations for RGB passthrough FIFO */
		__enable_continuous_vf(isp_subdev, false);
		*raw_override_p = 1;
	}
	v4l2_dbg(5, dbg_level, &atomisp_dev, "EXIT %s\n",__func__);
}


int atomisp_set_frame_function_pointers(struct atomisp_sub_device *isp_subdev,
		configure_output_t *configure_output, get_frame_info_t *get_frame_info,
		configure_pp_input_t *configure_pp_input, unsigned int source_pad,
		const struct atomisp_format_bridge *format, int raw_override,
		struct ia_css_frame_info *raw_output_info)
{
	if (isp_subdev->run_mode->val == ATOMISP_RUN_MODE_VIDEO
			|| !isp_subdev->enable_vfpp->val) {
		*configure_output = ia_css_video_configure_output;
		*get_frame_info = ia_css_video_get_output_frame_info;
	} else if (source_pad == ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW) {
		*configure_output = ia_css_preview_configure_output;
		*get_frame_info = ia_css_preview_get_output_frame_info;
		*configure_pp_input = ia_css_preview_configure_pp_input;
	} else {
		if (format->sh_fmt == IA_CSS_FRAME_FORMAT_RAW || raw_override) {
			ia_css_capture_set_mode(isp_subdev, IA_CSS_CAPTURE_MODE_RAW);
		}  else {
			if (fastboot){
				ia_css_capture_set_mode(isp_subdev, IA_CSS_CAPTURE_MODE_RAW);
			}
			else{
				ia_css_capture_set_mode(isp_subdev, IA_CSS_CAPTURE_MODE_PRIMARY);
			}
		}
		if (!isp_subdev->params.continuous_vf) {
			ia_css_capture_enable_online(isp_subdev,
					isp_subdev->params.online_process);
		}

		*configure_output = ia_css_capture_configure_output;
		*get_frame_info = ia_css_capture_get_output_frame_info;
		*configure_pp_input = ia_css_capture_configure_pp_input;

		if (!isp_subdev->params.online_process && !isp_subdev->params.continuous_vf) {
			if (ia_css_capture_get_output_raw_frame_info(isp_subdev,
						raw_output_info)) {
				return -EINVAL;
			}
		}
		if (!isp_subdev->params.continuous_vf &&
				isp_subdev->run_mode->val
				!= ATOMISP_RUN_MODE_STILL_CAPTURE) {
			v4l2_err(&atomisp_dev,
					"Need to set the running mode first\n");
			isp_subdev->run_mode->val =
				ATOMISP_RUN_MODE_STILL_CAPTURE;
		}
	}
	return 0;
}

enum ia_css_err configure_pp_input_nop(struct atomisp_sub_device *asd,
                           unsigned int width, unsigned int height)
{
        return 0;
}

static int atomisp_set_fmt_to_isp(struct video_device *vdev,
				   struct ia_css_frame_info *output_info,
				   struct ia_css_frame_info *raw_output_info,
				   int width, int height,
				  unsigned int pixelformat,
				  unsigned int source_pad)
{
	struct atomisp_device *isp = video_get_drvdata(vdev);
	const struct atomisp_format_bridge *format;
	struct v4l2_rect *isp_sink_crop;
	/*
	 * Initialized to take care of the warning
	 */
	configure_output_t configure_output = 0;
	get_frame_info_t get_frame_info = 0;
	configure_pp_input_t configure_pp_input = configure_pp_input_nop;

	struct atomisp_sub_device *isp_subdev = atomisp_to_sub_device(atomisp_to_video_pipe(vdev));

	int ret;
	int raw_override = 0;
	enum ia_css_stream_format sh_input_format = IA_CSS_STREAM_FORMAT_YUV422_8;

	v4l2_dbg(4, dbg_level, &atomisp_dev, "ENTER atomisp_set_fmt_to_isp\n");

	isp_sink_crop = atomisp_subdev_get_rect(
		isp_subdev, NULL, V4L2_SUBDEV_FORMAT_ACTIVE,
		ATOMISP_SUBDEV_PAD_SINK, V4L2_SEL_TGT_CROP);

	format = atomisp_get_format_bridge(pixelformat);
	if (format == NULL)
		return -EINVAL;

	/*
	 * Configure viewfinder also if enable_vfpp is disabled: the
	 * CSS still requires viewfinder configuration.
	 */
	atomisp_configure_viewfinder(isp_subdev, width, height, source_pad );

	atomisp_set_continuous_vf(isp_subdev, sh_input_format, width, height, &raw_override);

	ia_css_disable_vf_pp(isp_subdev, !isp_subdev->enable_vfpp->val);

	/* video same in continuouscapture and online modes */
	ret = atomisp_set_frame_function_pointers(isp_subdev, &configure_output, &get_frame_info,
                        &configure_pp_input, source_pad, format, raw_override, raw_output_info);

	if( ret < 0) {
		return ret;
	}

	ret = configure_output(isp_subdev, width, height, format->sh_fmt);
	if (ret) {
		dev_err(isp->dev, "configure_output %ux%u, format %8.8x\n",
			width, height, format->sh_fmt);
		return -EINVAL;
	}

        if (isp_subdev->params.continuous_vf && configure_pp_input == ia_css_preview_configure_pp_input) {
                /* for isp 2.2, configure pp input is available for continuous
 *                  * mode */
                ret = configure_pp_input(isp_subdev, isp_sink_crop->width, isp_sink_crop->height);
                if (ret) {
                        dev_err(isp->dev, "configure_pp_input %ux%u\n",
                                isp_sink_crop->width,
                                isp_sink_crop->height);
                        return -EINVAL;
                }
        } else {
                ret = configure_pp_input(isp_subdev, isp_sink_crop->width, isp_sink_crop->height);
                if (ret) {
                        dev_err(isp->dev, "configure_pp_input %ux%u\n",isp_sink_crop->width, isp_sink_crop->height);
                        return -EINVAL;
                }
        }

	ret = get_frame_info(isp_subdev, output_info);
	if (ret) {
		dev_err(isp->dev, "get_frame_info %ux%u\n", width, height);
		return -EINVAL;
	}

	atomisp_update_grid_info(isp_subdev);

	/* Free the raw_dump buffer first */
	ia_css_frame_free(isp_subdev->raw_output_frame);
	isp_subdev->raw_output_frame = NULL;

	if (!isp_subdev->params.continuous_vf && !isp_subdev->params.online_process &&
	    !isp->sw_contex.file_input &&
	    ia_css_frame_allocate_from_info(&isp_subdev->raw_output_frame,
					       raw_output_info))
		return -ENOMEM;

	v4l2_dbg(4, dbg_level, &atomisp_dev, "EXIT atomisp_set_fmt_to_isp\n");
	return 0;
}

static void atomisp_get_dis_envelop(struct atomisp_sub_device *isp_subdev,
			    unsigned int width, unsigned int height,
			    unsigned int *dvs_env_w,
			    unsigned int *dvs_env_h)
{
	struct atomisp_device *isp = isp_subdev->isp;
	/* if subdev type is SOC camera,we do not need to set DVS */
	if (isp->inputs[isp_subdev->input_curr].type == SOC_CAMERA)
		isp_subdev->params.video_dis_en = 0;

	if (isp_subdev->params.video_dis_en &&
	    isp_subdev->run_mode->val == ATOMISP_RUN_MODE_VIDEO) {
		/* envelope is 20% of the output resolution */
		/*
		 * dvs envelope cannot be round up.
		 * it would cause ISP timeout and color switch issue
		 */
		*dvs_env_w = rounddown(width / 5, ATOM_ISP_STEP_WIDTH);
		*dvs_env_h = rounddown(height / 5, ATOM_ISP_STEP_HEIGHT);
	}

	isp_subdev->params.dvs_proj_data_valid = false;
	isp_subdev->params.css_update_params_needed = true;
}

static int atomisp_set_fmt_to_subdev(struct atomisp_sub_device *isp_subdev,
			  struct v4l2_format *f, unsigned int pixelformat,
			  unsigned int padding_w, unsigned int padding_h,
			  unsigned int dvs_env_w, unsigned int dvs_env_h)
{
	const struct atomisp_format_bridge *format;
	struct v4l2_mbus_framefmt ffmt;
	int ret;
	struct atomisp_device *isp = isp_subdev->isp;
	v4l2_dbg(4, dbg_level, &atomisp_dev, "ENTER %s \n",__func__);

	format = atomisp_get_format_bridge(pixelformat);
	if (format == NULL) {
		trace_printk("atomisp_get_format_bridge failed");
		return -EINVAL;
	}

	if (!isp->sw_contex.file_input) {
	v4l2_fill_mbus_format(&ffmt, &f->fmt.pix, format->mbus_code);
	ffmt.height += padding_h + dvs_env_h;
	ffmt.width += padding_w + dvs_env_w;

	dev_dbg(isp->dev, "s_mbus_fmt: ask %ux%u (padding %ux%u, dvs %ux%u) mbus code 0x%0x\n",
		ffmt.width, ffmt.height, padding_w, padding_h,
		dvs_env_w, dvs_env_h,format->mbus_code);

	ret = v4l2_subdev_call(isp->inputs[isp_subdev->input_curr].camera, video,
			       s_mbus_fmt, &ffmt);
	if (ret) {
		trace_printk("v4l2_subdev_call s_mbus_fmt failed");
		return ret;
	}

	dev_dbg(isp->dev, "sensor width: %d, height: %d\n",
			ffmt.width, ffmt.height);
	}
	else {	/* file input case */
		ffmt.width = f->fmt.pix.width;
		ffmt.height = f->fmt.pix.height;
		ffmt.code = format->mbus_code;
	}

	if (ffmt.width < ATOM_ISP_STEP_WIDTH ||
	    ffmt.height < ATOM_ISP_STEP_HEIGHT)
			return -EINVAL;

	return atomisp_subdev_set_ffmt(isp_subdev, NULL,
				       V4L2_SUBDEV_FORMAT_ACTIVE,
				       ATOMISP_SUBDEV_PAD_SINK, &ffmt);
	v4l2_dbg(4, dbg_level, &atomisp_dev, "EXIT %s \n",__func__);
}

uint16_t atomisp_set_source_pad(enum atomisp_pipe_type pipe_type, uint16_t *source_pad)
{
	switch (pipe_type) {
	case ATOMISP_PIPE_CAPTURE:
		*source_pad = ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE;
		break;
	case ATOMISP_PIPE_VIEWFINDER:
		*source_pad = ATOMISP_SUBDEV_PAD_SOURCE_VF;
		break;
	case ATOMISP_PIPE_PREVIEW:
		*source_pad = ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}


void atomisp_set_viewfinder_frame_info(struct v4l2_format *f, struct atomisp_sub_device *isp_subdev,
		uint16_t source_pad, const struct atomisp_format_bridge *format_bridge,
		struct ia_css_frame_info *output_info)
{
	isp_subdev->video_pipe_vf_enable = true;

	if (isp_subdev->fmt_auto->val) {
		struct v4l2_rect *capture_comp =
			atomisp_subdev_get_rect(
					isp_subdev, NULL,
					V4L2_SUBDEV_FORMAT_ACTIVE,
					ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE,
					V4L2_SEL_TGT_COMPOSE);
		struct v4l2_rect r;

		memset(&r, 0, sizeof(r));

		r.width = f->fmt.pix.width;
		r.height = f->fmt.pix.height;

		if (capture_comp->width < r.width
				|| capture_comp->height < r.height) {
			r.width = capture_comp->width;
			r.height = capture_comp->height;
		}

		atomisp_subdev_set_selection(
				isp_subdev, NULL,
				V4L2_SUBDEV_FORMAT_ACTIVE, source_pad,
				V4L2_SEL_TGT_COMPOSE, 0, &r);

		f->fmt.pix.width = r.width;
		f->fmt.pix.height = r.height;
	}

	if (isp_subdev->run_mode->val == ATOMISP_RUN_MODE_VIDEO) {
		ia_css_video_configure_viewfinder(isp_subdev,
				f->fmt.pix.width, f->fmt.pix.height,
				format_bridge->sh_fmt);
		ia_css_video_get_viewfinder_frame_info(isp_subdev, output_info);
	} else {
		ia_css_capture_configure_viewfinder(isp_subdev,
				f->fmt.pix.width, f->fmt.pix.height,
				format_bridge->sh_fmt);
		ia_css_capture_get_viewfinder_frame_info(isp_subdev, output_info);
	}
}

static enum ia_css_stream_format convert_pixel_format_to_stream_format(unsigned int pixelformat)
{
	switch(pixelformat)
	{
		case V4L2_PIX_FMT_YUV422P:
			return IA_CSS_STREAM_FORMAT_YUV422_8;
		case V4L2_PIX_FMT_RGB565:
			return IA_CSS_STREAM_FORMAT_RGB_565;
		case V4L2_PIX_FMT_RGB24:
			return IA_CSS_STREAM_FORMAT_RGB_888;
		case V4L2_PIX_FMT_UYVY:
			return IA_CSS_STREAM_FORMAT_YUV422_8;
		case V4L2_PIX_FMT_SGRBG10:
		    return IA_CSS_STREAM_FORMAT_RAW_10;
		default:
			return -EINVAL;
	}
}

static enum ia_css_frame_format convert_pixel_format_to_frame_format(unsigned int pixelformat)
{
	switch(pixelformat)
	{
		case V4L2_PIX_FMT_YUV422P:
			return IA_CSS_FRAME_FORMAT_YUV422;
		case V4L2_PIX_FMT_RGB565:
			return IA_CSS_FRAME_FORMAT_RGB565;
		case V4L2_PIX_FMT_RGB24:
			return IA_CSS_FRAME_FORMAT_RGBA888;
		case V4L2_PIX_FMT_UYVY:
			return IA_CSS_FRAME_FORMAT_UYVY;
		case V4L2_PIX_FMT_NV12:
			return IA_CSS_FRAME_FORMAT_NV12;
		case V4L2_PIX_FMT_SGRBG10:
		    return IA_CSS_FRAME_FORMAT_RAW;
		default:
			return -EINVAL;
	}
}

unsigned int atomisp_try_mipi_frame_buffer_size(struct atomisp_sub_device *isp_subdev, unsigned int width, unsigned int height, unsigned int pixelformat)
{
        unsigned int input_buffer_size = width * height * DIV_ROUND_UP(get_pixel_depth(pixelformat),8);

        if(!(input_buffer_size <= ATOMISP_MIPI_BUFFER_SIZE * HIVE_ISP_DDR_WORD_BYTES))
        {
                return -EINVAL;
        }

        return 0;
}

int atomisp_configure_isp_subdev_input(struct v4l2_format *format,
		struct atomisp_sub_device *isp_subdev)
{
	struct atomisp_device *isp = isp_subdev->isp;
	enum ia_css_stream_format css_fmt;
	struct v4l2_mbus_framefmt snr_mbus_fmt;
	int ret;
	unsigned int width = format->fmt.pix.width;
	unsigned int height = format->fmt.pix.height;
	const struct atomisp_format_bridge *format_bridge;

        v4l2_dbg(6 , dbg_level, &atomisp_dev, " >%s \n", __func__);
	css_fmt = convert_pixel_format_to_stream_format(format->fmt.pix.pixelformat);
	format_bridge = atomisp_get_format_bridge(format->fmt.pix.pixelformat);

	if(css_fmt == -EINVAL)
	{
		dev_err(isp->dev, "pixelformat error!!\n");
		return css_fmt;
	}

	if (format_bridge == NULL) {
		dev_err(isp->dev, "atomisp_get_format_bridge failed");
		return -EINVAL;
	}

	isp_subdev->css2_basis.stream_config.input_res.width = width;
        isp_subdev->css2_basis.stream_config.input_res.height = height;

        isp_subdev->css2_basis.stream_config.effective_res.width = width;
        isp_subdev->css2_basis.stream_config.effective_res.height = height;

        isp_subdev->css2_basis.stream_config.format =
                (enum ia_css_stream_format)css_fmt;

        if(atomisp_try_mipi_frame_buffer_size(isp_subdev, width, height, format->fmt.pix.pixelformat))
        {
                v4l2_err(&atomisp_dev, "ERROR: Input frame buffer size is larger than the allocated intermediate MIPI buffers:[%x0x]\n",ATOMISP_MIPI_BUFFER_SIZE);
                return -EINVAL;
        }

	//configure sensor output format&resolution
	v4l2_fill_mbus_format(&snr_mbus_fmt, &format->fmt.pix,
			format_bridge->mbus_code);

	ret = v4l2_subdev_call(
			isp->inputs[isp_subdev->input_curr].camera,
			video, s_mbus_fmt, &snr_mbus_fmt);
	if (ret) {
		v4l2_err(&atomisp_dev, "ERROR v4l2_subdev_call, s_mbus_fmt\n");
	}

        v4l2_dbg(6 , dbg_level, &atomisp_dev, " <%s \n", __func__);
	return 0;
}

static void atomisp_configure_isp_port(struct atomisp_sub_device *isp_subdev)
{
	struct atomisp_device *isp = isp_subdev->isp;
	struct camera_mipi_info *mipi_info;
	mipi_info = atomisp_to_sensor_mipi_info(
			isp->inputs[isp_subdev->input_curr].camera);
	isp_subdev->css2_basis.stream_config.source.port.port = __get_mipi_port(mipi_info->port);
	isp_subdev->css2_basis.stream_config.source.port.num_lanes = mipi_info->num_lanes;
	isp_subdev->css2_basis.stream_config.source.port.timeout = 0xffff4;
}

static void atomisp_configure_isp_subdev_misc(struct atomisp_sub_device *isp_subdev, uint16_t source_pad)
{
	struct atomisp_device *isp = isp_subdev->isp;

        v4l2_dbg(6 , dbg_level, &atomisp_dev, " >%s \n", __func__);
	ia_css_input_set_mode(isp_subdev, get_input_mode(isp->inputs[isp_subdev->input_curr].type, isp));
	ia_css_capture_set_mode(isp_subdev, IA_CSS_CAPTURE_MODE_RAW);

	/* Only main stream pipe will be here */
	isp_subdev->capture_pad = source_pad;
        v4l2_dbg(6 , dbg_level, &atomisp_dev, " <%s \n", __func__);
}

static int atomisp_configure_isp_subdev_output(struct v4l2_format *format,
		struct atomisp_sub_device *isp_subdev, struct ia_css_frame_info *output_info)
{
	struct atomisp_device *isp = isp_subdev->isp;
	unsigned int width = format->fmt.pix.width;
	unsigned int height = format->fmt.pix.height;
	enum ia_css_frame_format frame_format;

        v4l2_dbg(6 , dbg_level, &atomisp_dev, " >%s \n", __func__);
	frame_format = convert_pixel_format_to_frame_format(format->fmt.pix.pixelformat);

	if(frame_format == -EINVAL)
		dev_err(isp->dev, "pixelformat error!!\n");

	if (ia_css_capture_configure_output(isp_subdev, width, height, frame_format))
	{
		v4l2_err(&atomisp_dev, "ERROR: css_capture_configure_output failed\n");
		return -EINVAL;
	}

	//This should create a port if it does not exist
	if (ia_css_capture_get_output_frame_info(isp_subdev, output_info)) {
		v4l2_err(&atomisp_dev, "ERROR: ia_css_capture_get_output_frame_info failed\n");
	}
	trace_printk("Returned from get_css_frame_info, output_info.padded_width=%d\n", output_info->padded_width);

	if(output_info->padded_width!= width)
	{
		v4l2_dbg(4,dbg_level,&atomisp_dev,"padded width is not equal to set width. Resetting the requested format\n");
		format->fmt.pix.width = output_info->padded_width;
	}

        v4l2_dbg(6 , dbg_level, &atomisp_dev, " <%s \n", __func__);
	return 0;
}

/*
 * Check whether main resolution configured smaller
 * than snapshot resolution. If so, force main resolution
 * to be the same as snapshot resolution
 */
void atomisp_check_resolution(uint16_t source_pad,struct video_device *vdev, struct v4l2_format *f)
{

	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	struct atomisp_sub_device *isp_subdev = atomisp_to_sub_device(pipe);
	if (source_pad == ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE) {
		struct v4l2_rect *r;

		r = atomisp_subdev_get_rect(
				isp_subdev, NULL,
				V4L2_SUBDEV_FORMAT_ACTIVE,
				ATOMISP_SUBDEV_PAD_SOURCE_VF, V4L2_SEL_TGT_COMPOSE);

		if (r->width && r->height
				&& (r->width > f->fmt.pix.width
					|| r->height > f->fmt.pix.height))
			dev_warn(isp->dev,
					"Main Resolution config smaller then Vf Resolution. Force to be equal with Vf Resolution.");
	}
}
/*
 * Get sensor resolution and format
 */
void atomisp_get_sensor_attributes(struct video_device *vdev, struct v4l2_format *f,
		bool *res_overflow, struct v4l2_format *snr_fmt)
{
	struct atomisp_device *isp = video_get_drvdata(vdev);
	/* get sensor resolution and format */
	if (!isp->sw_contex.file_input) {
		atomisp_try_fmt(vdev, snr_fmt, res_overflow);
		f->fmt.pix.width = snr_fmt->fmt.pix.width;
		f->fmt.pix.height = snr_fmt->fmt.pix.height;
	}
	else {
		snr_fmt->fmt.pix.width = f->fmt.pix.width;
		snr_fmt->fmt.pix.height = f->fmt.pix.height;
	}
}

/*
 * Set mbus codes for bypass mode configuration, but do
 * nothing else.
 */
void atomisp_set_bypass_mode_mbus_codes(struct atomisp_sub_device *isp_subdev, struct v4l2_format *snr_fmt,
		struct v4l2_format *f, struct v4l2_mbus_framefmt *isp_sink_fmt,
		uint16_t source_pad)
{
	struct v4l2_mbus_framefmt isp_source_fmt;
	struct atomisp_format_bridge *snr_fmt_bridge = atomisp_get_format_bridge(snr_fmt->fmt.pix.pixelformat);
	struct atomisp_format_bridge *fmt_bridge = atomisp_get_format_bridge(f->fmt.pix.pixelformat);

	if(snr_fmt_bridge != NULL)
		atomisp_subdev_get_ffmt(isp_subdev, NULL,
				V4L2_SUBDEV_FORMAT_ACTIVE,
				ATOMISP_SUBDEV_PAD_SINK)->code = snr_fmt_bridge->mbus_code;

	isp_sink_fmt = atomisp_subdev_get_ffmt(isp_subdev, NULL,
			V4L2_SUBDEV_FORMAT_ACTIVE,
			ATOMISP_SUBDEV_PAD_SINK);

	memset(&isp_source_fmt, 0, sizeof(isp_source_fmt));
	if(fmt_bridge != NULL)
		isp_source_fmt.code = fmt_bridge->mbus_code;
	atomisp_subdev_set_ffmt(isp_subdev, NULL,
			V4L2_SUBDEV_FORMAT_ACTIVE,
			source_pad, &isp_source_fmt);
}

/*
 * construct resolution supported by isp
 */
void atomisp_construct_supported_resolution(bool res_overflow, struct atomisp_sub_device *isp_subdev,
		struct v4l2_format *f, 	unsigned int padding_w, unsigned int padding_h)
{
	if (res_overflow && !isp_subdev->params.continuous_vf) {
		f->fmt.pix.width = rounddown(
				clamp_t(u32, f->fmt.pix.width - padding_w,
					ATOM_ISP_MIN_WIDTH,
					ATOM_ISP_MAX_WIDTH), ATOM_ISP_STEP_WIDTH);
		f->fmt.pix.height = rounddown(
				clamp_t(u32, f->fmt.pix.height - padding_h,
					ATOM_ISP_MIN_HEIGHT,
					ATOM_ISP_MAX_HEIGHT), ATOM_ISP_STEP_HEIGHT);

	}
}

/*
 * Try to enable YUV downscaling if ISP input is 10 % (either
 * width or height) bigger than the desired result.
 */
void atomisp_yuv_downscaling(struct v4l2_rect *isp_sink_crop, struct v4l2_format *f,
		struct atomisp_device *isp, struct atomisp_sub_device *isp_subdev,
		uint16_t source_pad, unsigned int padding_w, unsigned int padding_h)
{
	v4l2_dbg(4, dbg_level, &atomisp_dev, "ENTER %s run_mode=%d , width=%d, height=%d\n",__func__,isp_subdev->run_mode->val,f->fmt.pix.width,f->fmt.pix.height);
	if (isp_sink_crop->width * 9 / 10 < f->fmt.pix.width
			|| isp_sink_crop->height * 9 / 10 < f->fmt.pix.height
			|| isp->sw_contex.file_input
			|| (isp->sw_contex.bypass
				&& isp_subdev->run_mode->val != ATOMISP_RUN_MODE_VIDEO
				&& isp_subdev->enable_vfpp->val)
			|| (!isp->sw_contex.bypass
				&& isp_subdev->run_mode->val == ATOMISP_RUN_MODE_VIDEO)) {
		isp_sink_crop->width = f->fmt.pix.width;
		isp_sink_crop->height = f->fmt.pix.height;
		atomisp_subdev_set_selection(isp_subdev, NULL,
				V4L2_SUBDEV_FORMAT_ACTIVE,
				ATOMISP_SUBDEV_PAD_SINK,
				V4L2_SEL_TGT_CROP,
				V4L2_SEL_FLAG_KEEP_CONFIG,
				isp_sink_crop);
		v4l2_dbg(5, dbg_level, &atomisp_dev, "%s: isp_sink_crop width=%d, height=%d\n",__func__,isp_sink_crop->width,isp_sink_crop->width);
		atomisp_subdev_set_selection(isp_subdev, NULL,
				V4L2_SUBDEV_FORMAT_ACTIVE,
				source_pad, V4L2_SEL_TGT_COMPOSE,
				0, isp_sink_crop);
	} else {
		struct v4l2_rect main_compose;

		memset(&main_compose, 0, sizeof(main_compose));

		main_compose.width = isp_sink_crop->width - padding_w;
		main_compose.height =
			DIV_ROUND_UP(main_compose.width * f->fmt.pix.height,
					f->fmt.pix.width);
		if (main_compose.height > isp_sink_crop->height - padding_h) {
			main_compose.height = isp_sink_crop->height - padding_h;
			main_compose.width =
				DIV_ROUND_UP(main_compose.height *
						f->fmt.pix.width,
						f->fmt.pix.height);
		}
	v4l2_dbg(5, dbg_level, &atomisp_dev, "%s: main_compose width=%d, height=%d\n",__func__,main_compose.width,main_compose.height);
		atomisp_subdev_set_selection(isp_subdev, NULL,
				V4L2_SUBDEV_FORMAT_ACTIVE,
				ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE,
				V4L2_SEL_TGT_COMPOSE, 0,
				&main_compose);
	}
	v4l2_dbg(4, dbg_level, &atomisp_dev, "EXIT %s \n",__func__);
}

void atomisp_set_video_pipe(struct video_device *vdev, struct v4l2_format *format,
			struct ia_css_frame_info output_info, struct atomisp_sub_device *isp_subdev,
			const struct atomisp_format_bridge *format_bridge)
{
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);

	pipe->pix.width = format->fmt.pix.width;
	pipe->pix.height = format->fmt.pix.height;
	pipe->pix.pixelformat = format->fmt.pix.pixelformat;
	pipe->pix.bytesperline =
		DIV_ROUND_UP(format_bridge->depth * output_info.padded_width,
			     8);
	pipe->pix.sizeimage =
	    PAGE_ALIGN(format->fmt.pix.height * pipe->pix.bytesperline);
	if (format->fmt.pix.field == V4L2_FIELD_ANY)
		format->fmt.pix.field = V4L2_FIELD_NONE;
	pipe->pix.field = format->fmt.pix.field;

	format->fmt.pix = pipe->pix;
	format->fmt.pix.priv = PAGE_ALIGN(pipe->pix.width *
				     pipe->pix.height * 2);

	/*
	 * If in video 480P case, no GFX throttle
	 */
	if (pipe->pipe_type == ATOMISP_PIPE_CAPTURE) {
		if (isp_subdev->run_mode->val == ATOMISP_RUN_MODE_VIDEO &&
		    format->fmt.pix.width == 720 && format->fmt.pix.height == 480) {
			isp->need_gfx_throttle = false;
		}
		else {
			isp->need_gfx_throttle = true;
		}
	}
}
int atomisp_set_fmt(struct video_device *vdev, struct v4l2_format *format)
{
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	struct atomisp_sub_device *isp_subdev = atomisp_to_sub_device(pipe);
	const struct atomisp_format_bridge *format_bridge;
	struct ia_css_frame_info output_info, raw_output_info;
	struct v4l2_format snr_fmt = *format;
	unsigned int dvs_env_w = 0,
		     dvs_env_h = 0;
	unsigned int padding_w = pad_w,
		     padding_h = pad_h;
	bool res_overflow = false;
	struct v4l2_mbus_framefmt isp_sink_fmt;
	struct v4l2_rect isp_sink_crop;
	uint16_t source_pad;
	int ret;

	trace_printk("ENTER DMA atomisp_set_fmt\n");

	if (format->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
	    format->type != V4L2_BUF_TYPE_PRIVATE) {
		v4l2_err(&atomisp_dev, "Wrong v4l2 buf type\n");
		trace_printk("Wrong v4l2 buf type\n");
		return -EINVAL;
	}

	format_bridge = atomisp_get_format_bridge(format->fmt.pix.pixelformat);
	if (format_bridge == NULL) {
		trace_printk("Format bridge is NULL\n");
		return -EINVAL;
	}

	pipe->sh_fmt = format_bridge->sh_fmt;
	pipe->pix.pixelformat = format->fmt.pix.pixelformat;

	ret = atomisp_set_source_pad(pipe->pipe_type, &source_pad);
	if( ret < 0 ) {
		dev_err(isp->dev, "can't get source pad");
		return ret;
	}

	atomisp_configure_isp_port(isp_subdev);

	dev_dbg(isp->dev, "setting resolution %ux%u on pad %u\n",
		format->fmt.pix.width, format->fmt.pix.height, source_pad);

	if (source_pad == ATOMISP_SUBDEV_PAD_SOURCE_VF
	    || (source_pad == ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW
		&& isp_subdev->run_mode->val == ATOMISP_RUN_MODE_VIDEO)) {
                atomisp_set_viewfinder_frame_info(format, isp_subdev, source_pad, format_bridge, &output_info);
		goto done;
	}

	if(source_pad == ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE && !(isp_subdev->run_mode->val == ATOMISP_RUN_MODE_VIDEO) && !(source_pad == ATOMISP_SUBDEV_PAD_SOURCE_VF) && !isp->sw_contex.file_input)
	{

		if (!isp_subdev->css2_basis.stream_config.input_res.width) {
		if(-EINVAL == atomisp_configure_isp_subdev_input(format, isp_subdev)) {
			return -EINVAL;
		}
		}
		atomisp_configure_isp_subdev_misc(isp_subdev, source_pad);

		if(-EINVAL == atomisp_configure_isp_subdev_output(format, isp_subdev, &output_info)) {
			return -EINVAL;
		}

		goto done;
	}

	/* Check main resolution against configured resolution*/
	atomisp_check_resolution(source_pad, vdev, format);

	/* V4L2_BUF_TYPE_PRIVATE will set offline processing */
	if (format->type == V4L2_BUF_TYPE_PRIVATE)
		isp_subdev->params.online_process = 0;
	else
		isp_subdev->params.online_process = 1;

	/* Pipeline configuration done through subdevs. Bail out now. */
	if (!isp_subdev->fmt_auto->val)
		goto done;

	/* get sensor resolution and format */
	atomisp_get_sensor_attributes(vdev, format, &res_overflow, &snr_fmt);


	/* Set mbus codes for bypass mode configuration */
	atomisp_set_bypass_mode_mbus_codes(isp_subdev, &snr_fmt,
			format, &isp_sink_fmt, source_pad);

	if (isp->sw_contex.bypass)
		padding_w = 0, padding_h = 0;

	/* construct resolution supported by isp */
	atomisp_construct_supported_resolution(res_overflow, isp_subdev, format,
			padding_w, padding_h);

	atomisp_get_dis_envelop(isp_subdev, format->fmt.pix.width, format->fmt.pix.height,
				&dvs_env_w, &dvs_env_h);

	/*
	 * set format info to sensor
	 * In case of continuous_vf, resolution is set only if it is higher than
	 * existing value. This because preview pipe will be configured after
	 * capture pipe and usually has lower resolution than capture pipe.
	 */

	if (!isp_subdev->params.continuous_vf ||
	    isp_subdev->run_mode->val == ATOMISP_RUN_MODE_VIDEO ||
	    (isp_sink_fmt.width < (format->fmt.pix.width + padding_w + dvs_env_w) &&
	     isp_sink_fmt.height < (format->fmt.pix.height + padding_h +
				    dvs_env_h))) {
			/*ret = atomisp_set_fmt_to_subdev(isp_subdev, format, format->fmt.pix.pixelformat,
					     padding_w, padding_h,
					     dvs_env_w, dvs_env_h);*/
		if (ret) {
			trace_printk("atomisp_set_fmt_to_subdev failed\n");
			return -EINVAL;
		}
	}

	isp_sink_crop = *atomisp_subdev_get_rect(isp_subdev, NULL,
						 V4L2_SUBDEV_FORMAT_ACTIVE,
						 ATOMISP_SUBDEV_PAD_SINK,
						 V4L2_SEL_TGT_CROP);


	/* Try to enable YUV downscaling if ISP input is 10 % (either
	 * width or height) bigger than the desired result. */
	atomisp_yuv_downscaling(&isp_sink_crop, format, isp, isp_subdev, source_pad,
			padding_w, padding_h);

	/* set format to isp */
	ret = atomisp_set_fmt_to_isp(vdev, &output_info, &raw_output_info,
				     format->fmt.pix.width, format->fmt.pix.height,
				     format->fmt.pix.pixelformat, source_pad);
	if (ret) {
		trace_printk("atomisp_set_fmt_to_isp failed\n");
		return -EINVAL;
	}
done:
	atomisp_update_run_mode(isp_subdev);
	atomisp_set_video_pipe(vdev, format, output_info, isp_subdev, format_bridge);
	trace_printk("EXIT DMA atomisp_set_fmt\n");
	return 0;
}

int atomisp_set_fmt_file(struct video_device *vdev, struct v4l2_format *f)
{
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	struct atomisp_sub_device *isp_subdev = atomisp_to_sub_device(pipe);
	enum ia_css_stream_format sh_input_format;
	int ret = 0;

	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		v4l2_err(&atomisp_dev,
				"Wrong v4l2 buf type for output\n");
		return -EINVAL;
	}

	ret = atomisp_try_fmt_file(isp, f);
	if (ret)
		return ret;

		pipe->pix = f->fmt.pix;
	sh_input_format = get_sh_input_format(
						pipe->pix.pixelformat);
	if (sh_input_format == -EINVAL) {
		v4l2_err(&atomisp_dev,
				"Wrong v4l2 format for output\n");
		return -EINVAL;
	}

	isp_subdev->css2_basis.stream_config.format = sh_input_format;
	isp_subdev->css2_basis.stream_config.mode = IA_CSS_INPUT_MODE_FIFO;
	isp_subdev->css2_basis.stream_config.bayer_order = 0; /* TODO set real value */
	isp_subdev->css2_basis.stream_config.source.port.port = __get_mipi_port(ATOMISP_CAMERA_PORT_PRIMARY);
	isp_subdev->css2_basis.stream_config.source.port.num_lanes = 2;
	isp_subdev->css2_basis.stream_config.source.port.timeout = 0xffff4;
	return 0;
}

void atomisp_free_all_shading_tables(struct atomisp_device *isp)
{
	int i;

	for (i = 0; i < isp->input_cnt; i++) {
		if (isp->inputs[i].shading_table == NULL)
			continue;
		ia_css_shading_table_free(isp->inputs[i].shading_table);
		isp->inputs[i].shading_table = NULL;
	}
}

int atomisp_set_shading_table(struct atomisp_sub_device *isp_subdev,
		struct atomisp_shading_table *user_shading_table)
{
	struct ia_css_shading_table *shading_table;
	struct ia_css_shading_table *free_table;
	unsigned int len_table;
	int i;
	int ret = 0;
	struct atomisp_device *isp = isp_subdev->isp;

	if (!user_shading_table)
		return -EINVAL;

	if (user_shading_table->flags & ATOMISP_SC_FLAG_QUERY) {
		user_shading_table->enable = isp_subdev->params.sc_en;
		return 0;
	}

	if (!user_shading_table->enable) {
		isp_subdev->params.config.shading_table = NULL;
		isp_subdev->params.sc_en = 0;
		return 0;
	}

	/* If enabling, all tables must be set */
	for (i = 0; i < ATOMISP_NUM_SC_COLORS; i++) {
		if (!user_shading_table->data[i])
			return -EINVAL;
	}

	/* Shading table size per color */
	if (user_shading_table->width > SH_CSS_MAX_SCTBL_WIDTH_PER_COLOR ||
	    user_shading_table->height > SH_CSS_MAX_SCTBL_HEIGHT_PER_COLOR)
		return -EINVAL;

	shading_table = ia_css_shading_table_alloc(user_shading_table->width,
						   user_shading_table->height);
	if (!shading_table)
		return -ENOMEM;

	len_table = user_shading_table->width * user_shading_table->height *
		    ATOMISP_SC_TYPE_SIZE;
	for (i = 0; i < ATOMISP_NUM_SC_COLORS; i++) {
		ret = copy_from_user(shading_table->data[i],
				     user_shading_table->data[i], len_table);
		if (ret) {
			free_table = shading_table;
			ret = -EFAULT;
			goto out;
		}
	}
	shading_table->sensor_width = user_shading_table->sensor_width;
	shading_table->sensor_height = user_shading_table->sensor_height;
	shading_table->fraction_bits = user_shading_table->fraction_bits;

	free_table = isp->inputs[isp_subdev->input_curr].shading_table;
	isp->inputs[isp_subdev->input_curr].shading_table = shading_table;
	isp_subdev->params.config.shading_table = shading_table;
	isp_subdev->params.sc_en = 1;

out:
	if (free_table != NULL)
		ia_css_shading_table_free(free_table);

	return ret;
}

/*Turn off ISP dphy */
int atomisp_ospm_dphy_down(struct atomisp_device *isp)
{
	u32 pwr_cnt = 0;
	int timeout = 100;
	bool idle;
	u32 reg;
	v4l2_dbg(3, dbg_level, &atomisp_dev, "%s\n", __func__);

	/* if ISP timeout, we can force powerdown */
	if (isp->isp_timeout)
		goto done;

	if (!atomisp_dev_users(isp))
		goto done;

	idle = sh_css_hrt_system_is_idle();
	v4l2_dbg(3, dbg_level, &atomisp_dev,
		 "%s system_is_idle:%d\n", __func__, idle);
	while (!idle && timeout--) {
		udelay(20);
		idle = sh_css_hrt_system_is_idle();
	}

	if (timeout < 0) {
		v4l2_err(&atomisp_dev,
			 "Timeout to stop ISP HW\n");
		/* force power down here */
	}

done:
	if (IS_MRFLD) {
		/*
		 * MRFLD IUNIT DPHY is located in an always-power-on island
		 * MRFLD HW design need all CSI ports are disabled before
		 * powering down the IUNIT.
		 */
		pci_read_config_dword(isp->pdev, MRFLD_PCI_CSI_CONTROL, &reg);
		reg |= MRFLD_ALL_CSI_PORTS_OFF_MASK;
		pci_write_config_dword(isp->pdev, MRFLD_PCI_CSI_CONTROL, reg);
	} else {
		/* power down DPHY */
		pwr_cnt = intel_mid_msgbus_read32(MFLD_IUNITPHY_PORT,
							MFLD_CSI_CONTROL);
		pwr_cnt |= 0x300;
		intel_mid_msgbus_write32(MFLD_IUNITPHY_PORT,
						MFLD_CSI_CONTROL, pwr_cnt);
	}

	isp->sw_contex.power_state = ATOM_ISP_POWER_DOWN;
	return 0;
}

/*Turn on ISP dphy */
int atomisp_ospm_dphy_up(struct atomisp_device *isp)
{
	u32 pwr_cnt = 0;
	v4l2_dbg(3, dbg_level, &atomisp_dev, "%s\n", __func__);

	/* MRFLD IUNIT DPHY is located in an always-power-on island */
	if (!IS_MRFLD) {
		/* power on DPHY */
		pwr_cnt = intel_mid_msgbus_read32(MFLD_IUNITPHY_PORT,
							MFLD_CSI_CONTROL);
		pwr_cnt &= ~0x300;
		intel_mid_msgbus_write32(MFLD_IUNITPHY_PORT,
						MFLD_CSI_CONTROL, pwr_cnt);
	}

	isp->sw_contex.power_state = ATOM_ISP_POWER_UP;

	return 0;
}


int atomisp_exif_makernote(struct atomisp_sub_device *isp_subdev,
			   struct atomisp_makernote_info *config)
{
	struct atomisp_device *isp = isp_subdev->isp;
	struct v4l2_control ctrl;

	ctrl.id = V4L2_CID_FOCAL_ABSOLUTE;
	if (v4l2_subdev_call(isp->inputs[isp_subdev->input_curr].camera,
				 core, g_ctrl, &ctrl))
		v4l2_warn(&atomisp_dev, "failed to g_ctrl for focal length\n");
	else
		config->focal_length = ctrl.value;

	ctrl.id = V4L2_CID_FNUMBER_ABSOLUTE;
	if (v4l2_subdev_call(isp->inputs[isp_subdev->input_curr].camera,
				core, g_ctrl, &ctrl))
		v4l2_warn(&atomisp_dev, "failed to g_ctrl for f-number\n");
	else
		config->f_number_curr = ctrl.value;

	ctrl.id = V4L2_CID_FNUMBER_RANGE;
	if (v4l2_subdev_call(isp->inputs[isp_subdev->input_curr].camera,
				core, g_ctrl, &ctrl))
		v4l2_warn(&atomisp_dev,
				"failed to g_ctrl for f number range\n");
	else
		config->f_number_range = ctrl.value;

	return 0;
}

int atomisp_offline_capture_configure(struct atomisp_sub_device *isp_subdev,
			      struct atomisp_cont_capture_conf *cvf_config)
{
	isp_subdev->params.offline_parm = *cvf_config;
	if (isp_subdev->params.offline_parm.num_captures) {
		if (isp_subdev->streaming == ATOMISP_DEVICE_STREAMING_DISABLED) {
			int num_raw_frames =
				min_t(int, ATOMISP_CONT_RAW_FRAMES,
				      isp_subdev->params.offline_parm.num_captures
				      + 3);

			ia_css_stream_set_buffer_depth(isp_subdev->css2_basis.stream,
							num_raw_frames);
		}

		isp_subdev->params.continuous_vf = true;
	} else {
		isp_subdev->params.continuous_vf = false;
		__enable_continuous_vf(isp_subdev, false);
	}

	return 0;
}

int atomisp_flash_enable(struct atomisp_sub_device *isp_subdev, int num_frames)
{
	if (num_frames < 0) {
		v4l2_dbg(3, dbg_level, &atomisp_dev,
			 "%s ERROR: num_frames: %d\n", __func__, num_frames);
		return -EINVAL;
	}
	/* a requested flash is still in progress. */
	if (num_frames && isp_subdev->params.flash_state != ATOMISP_FLASH_IDLE) {
		v4l2_dbg(3, dbg_level, &atomisp_dev, "%s flash busy: %d frames left: %d\n",__func__,
				isp_subdev->params.flash_state,
				isp_subdev->params.num_flash_frames);
		return -EBUSY;
	}

	isp_subdev->params.num_flash_frames = num_frames;
	isp_subdev->params.flash_state = ATOMISP_FLASH_REQUESTED;
	return 0;
}

/* Added msgbus functions */
static DEFINE_SPINLOCK(msgbus_lock);

static struct pci_dev *pci_root;

int intel_mid_msgbus_init(void)
{
        pci_root = pci_get_bus_and_slot(0, PCI_DEVFN(0, 0));
        if (!pci_root) {
                v4l2_err(&atomisp_dev, "%s: Error: msgbus PCI handle NULL", __func__);
                return -ENODEV;
        }
        return 0;
}

u32 intel_mid_msgbus_read32_raw(u32 cmd)
{
        unsigned long irq_flags;
        u32 data;

        spin_lock_irqsave(&msgbus_lock, irq_flags);
        pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_CTRL_REG, cmd);
        pci_read_config_dword(pci_root, PCI_ROOT_MSGBUS_DATA_REG, &data);
        spin_unlock_irqrestore(&msgbus_lock, irq_flags);

        return data;
}

void intel_mid_msgbus_write32_raw(u32 cmd, u32 data)
{
        unsigned long irq_flags;

        spin_lock_irqsave(&msgbus_lock, irq_flags);
        pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_DATA_REG, data);
        pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_CTRL_REG, cmd);
        spin_unlock_irqrestore(&msgbus_lock, irq_flags);
}

u32 intel_mid_msgbus_read32(u8 port, u32 addr)
{
        unsigned long irq_flags;
        u32 data;
        u32 cmd;
        u32 cmdext;

        cmd = (PCI_ROOT_MSGBUS_READ << 24) | (port << 16) |
                ((addr & 0xff) << 8) | PCI_ROOT_MSGBUS_DWORD_ENABLE;
        cmdext = addr & 0xffffff00;

        spin_lock_irqsave(&msgbus_lock, irq_flags);

        if (cmdext) {
                /* This resets to 0 automatically, no need to write 0 */
                pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_CTRL_EXT_REG,
                        cmdext);
        }

        pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_CTRL_REG, cmd);
        pci_read_config_dword(pci_root, PCI_ROOT_MSGBUS_DATA_REG, &data);
        spin_unlock_irqrestore(&msgbus_lock, irq_flags);

        return data;
}

void intel_mid_msgbus_write32(u8 port, u32 addr, u32 data)
{
        unsigned long irq_flags;
        u32 cmd;
        u32 cmdext;

        cmd = (PCI_ROOT_MSGBUS_WRITE << 24) | (port << 16) |
                ((addr & 0xFF) << 8) | PCI_ROOT_MSGBUS_DWORD_ENABLE;
        cmdext = addr & 0xffffff00;

        spin_lock_irqsave(&msgbus_lock, irq_flags);
        pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_DATA_REG, data);

        if (cmdext) {
                /* This resets to 0 automatically, no need to write 0 */
                pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_CTRL_EXT_REG,
                        cmdext);
        }

        pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_CTRL_REG, cmd);
        spin_unlock_irqrestore(&msgbus_lock, irq_flags);
}
