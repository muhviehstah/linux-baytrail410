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
#include <linux/delay.h>

//#include <asm/intel-mid.h>

#include <media/v4l2-ioctl.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>
#include <uapi/linux/v4l2-subdev.h>

#include "atomisp_acc.h"
#include "atomisp_compat.h"
#include "atomisp_cmd.h"
#include "atomisp_common.h"
#include "atomisp_fops.h"
#include "atomisp_internal.h"
#include "atomisp_ioctl.h"
#include "atomisp-regs.h"
#include "atomisp_v4l2.h"

#include "sh_css_hrt.h"
#include "ia_css_debug.h"
#include "gp_device.h"
#include "device_access.h"
#include "irq.h"

#include "hrt/hive_isp_css_mm_hrt.h"

#define IOCTL_ENTER v4l2_dbg(5, dbg_level, &atomisp_dev, ">%s.\n", __func__);
#define IOCTL_EXIT v4l2_dbg(5, dbg_level, &atomisp_dev, "<%s.\n", __func__);

/* for v4l2_capability */
static const char *DRIVER = "atomisp";	/* max size 15 */
static const char *CARD = "ATOM ISP";	/* max size 31 */
static const char *BUS_INFO = "PCI-3";	/* max size 31 */
static const u32 VERSION = DRIVER_VERSION;

/*
 * FIXME: ISP should not know beforehand all CIDs supported by sensor.
 * Instead, it needs to propagate to sensor unkonwn CIDs.
 */
static struct v4l2_queryctrl ci_v4l2_controls[] = {
	{
		.id = V4L2_CID_AUTO_WHITE_BALANCE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "Automatic White Balance",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_RED_BALANCE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Red Balance",
		.minimum = 0x00,
		.maximum = 0xff,
		.step = 1,
		.default_value = 0x00,
	},
	{
		.id = V4L2_CID_BLUE_BALANCE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Blue Balance",
		.minimum = 0x00,
		.maximum = 0xff,
		.step = 1,
		.default_value = 0x00,
	},
	{
		.id = V4L2_CID_GAMMA,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Gamma",
		.minimum = 0x00,
		.maximum = 0xff,
		.step = 1,
		.default_value = 0x00,
	},
	{
		.id = V4L2_CID_HFLIP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Image h-flip",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_VFLIP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Image v-flip",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_POWER_LINE_FREQUENCY,
		.type = V4L2_CTRL_TYPE_MENU,
		.name = "Light frequency filter",
		.minimum = 1,
		.maximum = 2,
		.step = 1,
		.default_value = 1,
	},
	{
		.id = V4L2_CID_COLORFX,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Image Color Effect",
		.minimum = 0,
		.maximum = 9,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_ATOMISP_BAD_PIXEL_DETECTION,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Bad Pixel Correction",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_ATOMISP_POSTPROCESS_GDC_CAC,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "GDC/CAC",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_ATOMISP_VIDEO_STABLIZATION,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Video Stablization",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_ATOMISP_FIXED_PATTERN_NR,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Fixed Pattern Noise Reduction",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_ATOMISP_FALSE_COLOR_CORRECTION,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "False Color Correction",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_REQUEST_FLASH,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Request flash frames",
		.minimum = 0,
		.maximum = 10,
		.step = 1,
		.default_value = 1,
	},
	{
		.id = V4L2_CID_ATOMISP_LOW_LIGHT,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "Low light mode",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 1,
	},
	{
		.id = V4L2_CID_BIN_FACTOR_HORZ,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Horizontal binning factor",
		.minimum = 0,
		.maximum = 10,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_BIN_FACTOR_VERT,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Vertical binning factor",
		.minimum = 0,
		.maximum = 10,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_2A_STATUS,
		.type = V4L2_CTRL_TYPE_BITMASK,
		.name = "AE and AWB status",
		.minimum = 0,
		.maximum = V4L2_2A_STATUS_AE_READY | V4L2_2A_STATUS_AWB_READY,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_EXPOSURE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "exposure",
		.minimum = -4,
		.maximum = 4,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_SCENE_MODE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "scene mode",
		.minimum = 0,
		.maximum = 13,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_ISO_SENSITIVITY,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "iso",
		.minimum = -4,
		.maximum = 4,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "white balance",
		.minimum = 0,
		.maximum = 9,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_EXPOSURE_METERING,
		.type = V4L2_CTRL_TYPE_MENU,
		.name = "metering",
		.minimum = 0,
		.maximum = 2,
		.step = 1,
		.default_value = 1,
	},
	{
		.id = V4L2_CID_3A_LOCK,
		.type = V4L2_CTRL_TYPE_BITMASK,
		.name = "3a lock",
		.minimum = 0,
		.maximum = V4L2_LOCK_EXPOSURE | V4L2_LOCK_WHITE_BALANCE
			 | V4L2_LOCK_FOCUS,
		.step = 1,
		.default_value = 0,
	},
};
static const u32 ctrls_num = ARRAY_SIZE(ci_v4l2_controls);

static unsigned int atomisp_get_pipe_index(struct atomisp_sub_device *isp_subdev,
				    enum atomisp_pipe_type pipe_type)
{
	switch (pipe_type) {
	case ATOMISP_PIPE_CAPTURE:
		if (isp_subdev->run_mode->val == ATOMISP_RUN_MODE_VIDEO
					|| !isp_subdev->enable_vfpp->val)
			return (unsigned int) IA_CSS_PIPE_ID_VIDEO;
		else
			return (unsigned int) IA_CSS_PIPE_ID_CAPTURE;
	case ATOMISP_PIPE_VIEWFINDER:
		if (isp_subdev->run_mode->val == ATOMISP_RUN_MODE_VIDEO)
			return (unsigned int) IA_CSS_PIPE_ID_VIDEO;
		else if (!atomisp_is_mbuscode_raw(
				 isp_subdev->
				 fmt[isp_subdev->capture_pad].fmt.code))
			return (unsigned int) IA_CSS_PIPE_ID_CAPTURE;
	case ATOMISP_PIPE_PREVIEW:
		if (isp_subdev->run_mode->val == ATOMISP_RUN_MODE_VIDEO)
			return (unsigned int) IA_CSS_PIPE_ID_VIDEO;
		else
			return (unsigned int) IA_CSS_PIPE_ID_PREVIEW;
	case ATOMISP_PIPE_FILEINPUT:
	default:
		break;
	}

	return 0;
}

static int __get_css_frame_info(struct atomisp_sub_device *isp_subdev,
				enum atomisp_pipe_type pipe_type,
				struct ia_css_frame_info *frame_info)
{
	enum ia_css_err ret;
	struct ia_css_pipe_info info;
	int pipe_index = atomisp_get_pipe_index(isp_subdev, pipe_type);

	if (pipe_index >= IA_CSS_PIPE_ID_NUM)
		return -EINVAL;

	ret = ia_css_pipe_get_info(isp_subdev->css2_basis.pipes[pipe_index],
					   &info);
	if (ret != IA_CSS_SUCCESS)
		return -EINVAL;

	switch (pipe_type) {
	case ATOMISP_PIPE_CAPTURE:
		*frame_info = info.output_info;
		break;
	case ATOMISP_PIPE_VIEWFINDER:
		*frame_info = info.vf_output_info;
		break;
	case ATOMISP_PIPE_PREVIEW:
		if (isp_subdev->run_mode->val == ATOMISP_RUN_MODE_VIDEO)
			*frame_info = info.vf_output_info;
		else
			*frame_info = info.output_info;
		break;
	default:
		frame_info = NULL;
		break;
	}
	return frame_info ? 0 : -EINVAL;
}

/*
 * v4l2 ioctls
 * return ISP capabilities
 *
 * FIXME: capabilities should be different for video0/video2/video3
 */
static int atomisp_querycap(struct file *file, void *fh,
			    struct v4l2_capability *cap)
{
	int ret = 0;
	IOCTL_ENTER
	memset(cap, 0, sizeof(struct v4l2_capability));

	WARN_ON(sizeof(DRIVER) > sizeof(cap->driver) ||
		sizeof(CARD) > sizeof(cap->card) ||
		sizeof(BUS_INFO) > sizeof(cap->bus_info));

	strncpy(cap->driver, DRIVER, sizeof(cap->driver) - 1);
	strncpy(cap->card, CARD, sizeof(cap->card) - 1);
	strncpy(cap->bus_info, BUS_INFO, sizeof(cap->card) - 1);

	cap->version = VERSION;

	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE |
	    V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_OUTPUT;
	IOCTL_EXIT

	return ret;
}

/*
 * return sensor chip identification
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0)
static int atomisp_g_chip_ident(struct file *file, void *fh,
	struct v4l2_dbg_chip_ident *chip)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_sub_device *isp_subdev = atomisp_to_sub_device(atomisp_to_video_pipe(vdev));
	int ret = 0;

	IOCTL_ENTER
	ret = v4l2_subdev_call(isp->inputs[isp_subdev->input_curr].camera,
			       core, g_chip_ident, chip);

	if (ret)
		v4l2_err(&atomisp_dev,
			    "failed to g_chip_ident for sensor\n");
	IOCTL_EXIT
	return ret;
}
#endif

/*
 * enum input are used to check primary/secondary camera
 */
static int atomisp_enum_input(struct file *file, void *fh,
	struct v4l2_input *input)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);

	int index = input->index;

	IOCTL_ENTER
	if (index >= isp->input_cnt)
		return -EINVAL;

	if (!isp->inputs[index].camera)
		return -EINVAL;

	memset(input, 0, sizeof(struct v4l2_input));
	strncpy(input->name, isp->inputs[index].camera->name,
		sizeof(input->name) - 1);

	/*
	 * HACK: append actuator's name to sensor's
	 * As currently userspace can't talk directly to subdev nodes, this
	 * ioctl is the only way to enum inputs + possible external actuators
	 * for 3A tuning purpose.
	 */
	if (isp->inputs[index].motor &&
	    strlen(isp->inputs[index].motor->name) > 0) {
		const int cur_len = strnlen(input->name, 32);
		const int max_size = sizeof(input->name) - cur_len - 1;

		if (max_size > 0) {
			input->name[cur_len] = '+';
			strncpy(&input->name[cur_len + 1],
				isp->inputs[index].motor->name, max_size - 1);
		}
	}

	input->type = V4L2_INPUT_TYPE_CAMERA;
	input->index = index;
	input->reserved[0] = isp->inputs[index].type;
	input->reserved[1] = isp->inputs[index].port;
	IOCTL_EXIT

	return 0;
}

static unsigned int atomisp_streaming_count(struct atomisp_sub_device *isp_subdev)
{
	//TODO DMA
	return isp_subdev->video_out_preview.vb2q.streaming
		+ isp_subdev->video_out_capture.vb2q.streaming
		+ isp_subdev->video_in.vb2q.streaming;
}

/*
 * get input are used to get current primary/secondary camera
 */
static int atomisp_g_input(struct file *file, void *fh, unsigned int *input)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_sub_device *isp_subdev = atomisp_to_sub_device(atomisp_to_video_pipe(vdev));

	IOCTL_ENTER
	mutex_lock(&isp->mutex);
	*input = isp_subdev->input_curr;
	mutex_unlock(&isp->mutex);
	IOCTL_EXIT

	return 0;
}


/*
 * set input are used to set current primary/secondary camera
 */
static int atomisp_s_input(struct file *file, void *fh, unsigned int port)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_sub_device *isp_subdev = atomisp_to_sub_device(atomisp_to_video_pipe(vdev));
	struct v4l2_subdev *camera = NULL;
	int ret;
	int i, input = 0;
	int mipi_port = (int)port;

	IOCTL_ENTER
	mutex_lock(&isp->mutex);
	if (mipi_port >= ATOM_ISP_MAX_INPUTS) {
		v4l2_dbg(3, dbg_level, &atomisp_dev,
			 "input_cnt: %d\n",isp->input_cnt);
		ret = -EINVAL;
		goto error;
	}

	if(mipi_port < 0) input = 0; /* always use the first default one */
	else
	{
	  /* convernt to input index from a physical mipi port */
	  for(i = 0; i < ATOM_ISP_MAX_INPUTS; i++) {
	    if(isp->inputs[i].port == mipi_port) {
	      input = i;
	      break;
	    }
	  }
	  if( i == ATOM_ISP_MAX_INPUTS) {
		v4l2_err(&atomisp_dev,
			 "%s, no camera on mipi port=%d\n",__func__, mipi_port);
		ret = -EINVAL;
		goto error;
	  }
	}
	trace_printk("Input selected: index=%d mipi-port=%d num_lanes=%d\n",
	       input, isp->inputs[input].port, isp->inputs[input].num_lanes);

	/*
	 * Checked whether the request camera:
	 * 1: already in use
	 * 2: if in use, whether it is by other streams
	 */
	if (isp->inputs[input].used_by > -1 &&
	    isp->inputs[input].used_by != isp_subdev->index) {
		v4l2_err(&atomisp_dev,
			 "%s, camera is already used by:%d\n", __func__,
			 isp->inputs[input].used_by);
		ret = -EBUSY;
		goto error;
	}

	camera = isp->inputs[input].camera;
	if (!camera) {
		v4l2_err(&atomisp_dev,
			 "%s, no camera\n",__func__);
		ret = -EINVAL;
		goto error;
	}

	if (atomisp_streaming_count(isp_subdev)) {
		v4l2_err(&atomisp_dev,
			 "ISP is still streaming, stop first\n");
		ret = -EINVAL;
		goto error;
	}

	/* power off the current sensor, as it is not used this time */
	if (isp->inputs[isp_subdev->input_curr].used_by == isp_subdev->index &&
		isp_subdev->input_curr != input) {
		ret = v4l2_subdev_call(isp->inputs[isp_subdev->input_curr].camera,
				       core, s_power, 0);
		if (ret)
			v4l2_warn(&atomisp_dev,
				    "Failed to power-off previous sensor\n");
	}

	/* powe on the new sensor */
	if (!isp->sw_contex.file_input) {
		ret = v4l2_subdev_call(isp->inputs[input].camera,
				       core, s_power, 1);
		if (ret) {
			v4l2_err(&atomisp_dev,
				    "Failed to power-on new sensor\n");
			ret = -EINVAL;
			goto error;
		}

	if (!isp->sw_contex.file_input && isp->inputs[input].motor)
			ret = v4l2_subdev_call(isp->inputs[input].motor, core,
					       init, 1);
	}

	isp_subdev->input_curr = input;
	/* marked this camera is used by stream */
	isp->inputs[input].used_by = isp_subdev->index;
	mutex_unlock(&isp->mutex);
	IOCTL_EXIT

	return 0;

error:
	mutex_unlock(&isp->mutex);

	return ret;
}

static int atomisp_g_fmt_cap(struct file *file, void *fh,
	struct v4l2_format *f)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);

	int ret;

	IOCTL_ENTER
	mutex_lock(&isp->mutex);
	ret = atomisp_get_fmt(vdev, f);
	mutex_unlock(&isp->mutex);
	IOCTL_EXIT
	return ret;
}

static int atomisp_g_fmt_file(struct file *file, void *fh,
		struct v4l2_format *f)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);

	IOCTL_ENTER
	mutex_lock(&isp->mutex);
	f->fmt.pix = pipe->pix;
	mutex_unlock(&isp->mutex);
	IOCTL_EXIT
	return 0;
}

/* This function looks up the closest available resolution. */
static int atomisp_try_fmt_cap(struct file *file, void *fh,
	struct v4l2_format *f)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	int ret;

	IOCTL_ENTER
	mutex_lock(&isp->mutex);
	ret = atomisp_try_fmt(vdev, f, NULL);
	mutex_unlock(&isp->mutex);
	IOCTL_EXIT
	return ret;
}

static int atomisp_s_fmt_cap(struct file *file, void *fh,
	struct v4l2_format *input_format)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	int ret;
	IOCTL_ENTER
	mutex_lock(&isp->mutex);
	ret = atomisp_set_fmt(vdev, input_format);
	mutex_unlock(&isp->mutex);
	IOCTL_EXIT
	return ret;
}

static int atomisp_s_fmt_file(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	int ret;

	IOCTL_ENTER
	mutex_lock(&isp->mutex);
	ret = atomisp_set_fmt_file(vdev, f);
	mutex_unlock(&isp->mutex);
	IOCTL_EXIT
	return ret;
}

/*
 * is_resolution_supported - Check whether resolution is supported
 * @width: check resolution width
 * @height: check resolution height
 *
 * Return 1 on supported or 0 otherwise.
*/
static int is_resolution_supported(u32 width, u32 height)
{
	if ((width > ATOM_ISP_MIN_WIDTH) && (width <= ATOM_ISP_MAX_WIDTH) &&
	    (height > ATOM_ISP_MIN_HEIGHT) && (height <= ATOM_ISP_MAX_HEIGHT)) {
		if (!(width % ATOM_ISP_STEP_WIDTH) &&
		    !(height % ATOM_ISP_STEP_HEIGHT))
			return 1;
	}

	return 0;
}

/*
 * This ioctl allows applications to enumerate all frame intervals that the
 * device supports for the given pixel format and frame size.
 *
 * framerate =  1 / frameintervals
 */
static int atomisp_enum_frameintervals(struct file *file, void *fh,
	struct v4l2_frmivalenum *arg)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_sub_device *isp_subdev = atomisp_to_sub_device(atomisp_to_video_pipe(vdev));

	int ret;

	IOCTL_ENTER
	if (arg->index != 0)
		return -EINVAL;

	if (!atomisp_get_format_bridge(arg->pixel_format))
		return -EINVAL;

	if (!is_resolution_supported(arg->width, arg->height))
		return -EINVAL;

	mutex_lock(&isp->mutex);
	ret = v4l2_subdev_call(isp->inputs[isp_subdev->input_curr].camera,
		video, enum_frameintervals, arg);

	if (ret) {
		/* set the FPS to default 15*/
		arg->type = V4L2_FRMIVAL_TYPE_DISCRETE;
		arg->discrete.numerator = 1;
		arg->discrete.denominator = 15;
	}
	mutex_unlock(&isp->mutex);
	IOCTL_EXIT

	return 0;
}

void atomisp_free_css_frames(struct vb2_queue *q)
{
	struct atomisp_vb2 *avb2;
	int i;
	for (i = 0; i < q->num_buffers; i++) {
		avb2 = container_of(q->bufs[i], struct atomisp_vb2, vb);
		ia_css_frame_free(avb2->cssframe);
		avb2->cssframe = NULL;
	}
}

int atomisp_alloc_css_stat_bufs(struct atomisp_sub_device *isp_subdev)
{
	struct atomisp_s3a_buf *s3a_buf = NULL;
	struct atomisp_dvs_buf *dvs_buf = NULL;
	/* 2 css pipes consuming 3a buffers */
	int count = ATOMISP_CSS_Q_DEPTH * 2;

	trace_printk("ENTER atomisp_alloc_css_stat_bufs\n");

	if (!list_empty(&isp_subdev->s3a_stats) && !list_empty(&isp_subdev->dvs_stats))
		return 0;

	while (count--) {
		if (isp_subdev->params.curr_grid_info.s3a_grid.enable) {
			v4l2_dbg(5, dbg_level, &atomisp_dev,
				 "allocating %d 3a buffers\n", count);
			s3a_buf = kzalloc(sizeof(struct atomisp_s3a_buf), GFP_KERNEL);
			if (!s3a_buf) {
				v4l2_err(&atomisp_dev, "s3a stat buf alloc failed\n");
				goto s3a_error;
			}

			s3a_buf->s3a_stat =
			    ia_css_isp_3a_statistics_allocate(&isp_subdev->params.curr_grid_info.s3a_grid);
			if (!s3a_buf->s3a_stat) {
				v4l2_err(&atomisp_dev,
					 "3a stat buf allocation failed\n");
				kfree(s3a_buf);
				goto s3a_error;
			}
			list_add_tail(&s3a_buf->list, &isp_subdev->s3a_stats);
		}

		if (isp_subdev->params.curr_grid_info.dvs_grid.enable) {
			v4l2_dbg(2, dbg_level, &atomisp_dev,
				 "allocating %d dvs buffers\n", count);
			dvs_buf = kzalloc(sizeof(struct atomisp_dvs_buf), GFP_KERNEL);
			if (!dvs_buf) {
				v4l2_err(&atomisp_dev, "dis stat buf alloc failed\n");
				goto dvs_error;
			}

			dvs_buf->dvs_stat =
			    ia_css_isp_dvs_statistics_allocate(&isp_subdev->params.curr_grid_info.dvs_grid);
			if (!dvs_buf->dvs_stat) {
				v4l2_err(&atomisp_dev,
					 "dvs stat buf allocation failed\n");
				kfree(dvs_buf);
				goto dvs_error;
			}
			list_add_tail(&dvs_buf->list, &isp_subdev->dvs_stats);
		}

	}

	trace_printk("EXIT atomisp_alloc_css_stat_bufs\n");
	return 0;
dvs_error:
	v4l2_err(&atomisp_dev,
		    "failed to allocate statistics buffers\n");

	while (!list_empty(&isp_subdev->dvs_stats)) {
		dvs_buf = list_entry(isp_subdev->dvs_stats.next,
				     struct atomisp_dvs_buf, list);
		ia_css_isp_dvs_statistics_free(dvs_buf->dvs_stat);
		list_del(&dvs_buf->list);
		kfree(dvs_buf);
	}
s3a_error:
	while (!list_empty(&isp_subdev->s3a_stats)) {
		s3a_buf = list_entry(isp_subdev->s3a_stats.next,
					   struct atomisp_s3a_buf, list);
		ia_css_isp_3a_statistics_free(s3a_buf->s3a_stat);
		list_del(&s3a_buf->list);
		kfree(s3a_buf);
	}
	trace_printk("ERROR in atomisp_alloc_css_stat_bufs\n");
	return -ENOMEM;
}

/*
 * Initiate Memory Mapping or User Pointer I/O
 */
int __atomisp_reqbufs(struct file *file, void *fh,
	struct v4l2_requestbuffers *req)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	struct atomisp_sub_device *isp_subdev = atomisp_to_sub_device(pipe);
	struct ia_css_frame_info frame_info;
	struct ia_css_frame *frame;
	struct atomisp_vb2 *avb2;
	int ret = 0, i = 0;

	v4l2_dbg(6, dbg_level, &atomisp_dev, "Enter __atomisp_reqbufs, count=%d, req->mem=%d, req->type=%d\n",
		req->count, req->memory, req->type);
	ia_css_update_stream(isp_subdev);

	ret =  vb2_ioctl_reqbufs(file, fh, req);
	v4l2_dbg(6, dbg_level, &atomisp_dev, "Return value from vb2_ioctl_reqbufs=%d\n", ret);
	if (req->count == 0) {
		return 0; //TODO DMA is this check needed?
	}
	if (ret)
		return ret;
	//Currently allowing only video mode to allocate stats buff.
	//TODO: Enable stat bufs allocation if capture/preview mode supports dis & s3a
	if(isp_subdev->run_mode->val == ATOMISP_RUN_MODE_VIDEO)
	atomisp_alloc_css_stat_bufs(isp_subdev);

	/*
	 * for user pointer type, buffers are not really allcated here,
	 * buffers are setup in QBUF operation through v4l2_buffer structure
	 */
	if (req->memory == V4L2_MEMORY_USERPTR || req->memory == V4L2_MEMORY_DMABUF)
		return 0;

	ret = __get_css_frame_info(isp_subdev, pipe->pipe_type, &frame_info);
	if (ret)
		return -EINVAL;

	/*
	 * Allocate the real frame here for selected node using our
	 * memory management function
	 */
	for (i = 0; i < req->count; i++) {
		if (ia_css_frame_allocate_from_info(&frame, &frame_info)) {
			v4l2_err(&atomisp_dev, "ERROR allocating css frame, buf=%d\n", i);
			goto error;
		}
		avb2 = container_of(pipe->vb2q.bufs[i], struct atomisp_vb2, vb);
		avb2->cssframe = frame;
	}
	v4l2_dbg(4, dbg_level, &atomisp_dev, "Exiting successfully from __atomisp_reqbufs\n");
	return ret;

error:
	while (i--) {
		avb2 = container_of(pipe->vb2q.bufs[i], struct atomisp_vb2, vb);
		ia_css_frame_free(avb2->cssframe);
		avb2->cssframe = NULL;
	}

	if (isp_subdev->vf_frame)
		ia_css_frame_free(isp_subdev->vf_frame);

	return -ENOMEM;
}

int atomisp_reqbufs(struct file *file, void *fh,
	struct v4l2_requestbuffers *req)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	int ret;

	IOCTL_ENTER
	mutex_lock(&isp->mutex);
	ret = __atomisp_reqbufs(file, fh, req);
	mutex_unlock(&isp->mutex);
	IOCTL_EXIT

	return ret;
}

static int atomisp_reqbufs_file(struct file *file, void *fh,
		struct v4l2_requestbuffers *req)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	int ret;

	IOCTL_ENTER
	if (req->count == 0) {
		atomisp_free_css_frames(&pipe->vb2outq);
	}
	ret = vb2_reqbufs(&pipe->vb2outq, req);
	IOCTL_EXIT
	return ret;
}


/* application query the status of a buffer */
static int atomisp_querybuf(struct file *file, void *fh,
	struct v4l2_buffer *buf)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	int ret;

	IOCTL_ENTER
	ret = vb2_querybuf(&pipe->vb2q, buf);
	IOCTL_EXIT
	return ret;
}

static int atomisp_querybuf_file(struct file *file, void *fh,
				struct v4l2_buffer *buf)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	int ret;

	IOCTL_ENTER
	ret = vb2_querybuf(&pipe->vb2outq, buf);
	IOCTL_EXIT
	return ret;
}

/*
 * Applications call the VIDIOC_QBUF ioctl to enqueue an empty (capturing) or
 * filled (output) buffer in the drivers incoming queue.
 */
static int atomisp_qbuf(struct file *file, void *fh, struct v4l2_buffer *buf)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	struct atomisp_sub_device *isp_subdev = atomisp_to_sub_device(pipe);
	unsigned long userptr = 0;
	struct vb2_queue *q = vdev->queue;
	struct vb2_buffer *vb2 = q->bufs[buf->index];
	struct atomisp_vb2 *avb2 = container_of(vb2, struct atomisp_vb2, vb);
	struct ia_css_frame_info frame_info;
	struct ia_css_frame *handle = NULL;
	u32 length;
	u32 pgnr;
	int ret = 0;

	IOCTL_ENTER
	mutex_lock(&isp->mutex);
	if (isp->isp_fatal_error) {
		ret = -EIO;
		goto error;
	}

	if (isp_subdev->streaming == ATOMISP_DEVICE_STREAMING_STOPPING) {
		v4l2_err(&atomisp_dev, "ISP ERROR\n");
		ret = -EIO;
		goto error;
	}

	if (!buf || buf->index >= VIDEO_MAX_FRAME ||
		!pipe->vb2q.bufs[buf->index]) {
		v4l2_err(&atomisp_dev,
			    "Invalid index for qbuf.\n");
		ret = -EINVAL;
		goto error;
	}

	v4l2_dbg(6, dbg_level, &atomisp_dev, "atomisp_qbuf fd=%d, VB2_BUF_STATE=%d\n", buf->m.fd, vdev->queue->bufs[buf->index]->state);
	ret = vb2_ioctl_qbuf(file, fh, buf); //This will call atomisp_vb2_buf_queue
	if (ret) {
		v4l2_dbg(6, dbg_level, &atomisp_dev, "ERROR with vb2_ioctl_qbuf\n");
	   	goto error;
	}

	if (buf->memory == V4L2_MEMORY_DMABUF) {
		struct vb2_dc_buf *dc_buf;
		struct hrt_userbuffer_attr attributes;
		length = vb2_plane_size(vb2, 0);
		pgnr = (length + (PAGE_SIZE - 1)) >> PAGE_SHIFT;
		attributes.pgnr = pgnr;
		attributes.type = HRT_KERNEL_PTR;
		dc_buf = (struct vb2_dc_buf*) vb2->planes[0].mem_priv;
		attributes.priv = (void*) dc_buf->dma_sgt;

		if (__get_css_frame_info(isp_subdev, pipe->pipe_type,
             &frame_info)) {
      			ret = -EIO;
      			goto error;
    		}

		ret = ia_css_frame_map(&handle, &frame_info, NULL,
               0, &attributes);

    		if (ret != IA_CSS_SUCCESS) {
      			dev_err(isp->dev, "Failed ia_css_frame_dma \n");
      			ret = -ENOMEM;
      			goto error;
    		}

		if (avb2->cssframe) {
			v4l2_dbg(4, dbg_level, &atomisp_dev, "WARNING cssframe already exists or was not initialized to NULL\n");
			ia_css_frame_free(avb2->cssframe);
			avb2->cssframe = NULL;
		}
		avb2->cssframe = handle;

		//Note in vb2 do not need to set V4L2_BUF_FLAG
		v4l2_dbg(4, dbg_level, &atomisp_dev, "DMA after vb2_ioctl_qbuf V4L2_BUF_FLAG=0X%X, buf fd=%d, index=%d, vaddr=%X\n", buf->flags, buf->m.fd,avb2->vb.v4l2_buf.index, avb2->cssframe->data);

		goto cssqbuf;
	}
	/*
	 * For userptr type frame, we convert user space address to physic
	 * address and reprograme out page table properly
	 */
	else if (buf->memory == V4L2_MEMORY_USERPTR) {
		struct hrt_userbuffer_attr attributes;
		userptr = buf->m.userptr;
		length = vb2_plane_size(vb2, 0);

		if (__get_css_frame_info(isp_subdev, pipe->pipe_type,
				   &frame_info)) {
			ret = -EIO;
			goto error;
		}

		pgnr = (length + (PAGE_SIZE - 1)) >> PAGE_SHIFT;
		attributes.pgnr = pgnr;
#ifdef CONFIG_ION
		attributes.type = buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_ION
					? HRT_USR_ION : HRT_USR_PTR;
#else
		attributes.type = HRT_USR_PTR;
#endif
		ret = ia_css_frame_map(&handle, &frame_info, (void *)userptr,
				       0, &attributes);
		if (ret != IA_CSS_SUCCESS) {
			dev_err(isp->dev, "Failed to map user buffer\n");
			ret = -ENOMEM;
			goto error;
		}

		if (avb2->cssframe) {
			ia_css_frame_free(avb2->cssframe);
			avb2->cssframe = NULL;
		}

		avb2->cssframe = handle;
	} else if (buf->memory == V4L2_MEMORY_MMAP) {
	}

cssqbuf:

	/* TODO: do this better, not best way to queue to css */
	if (isp_subdev->streaming == ATOMISP_DEVICE_STREAMING_ENABLED){
		atomisp_qbuffers_to_css(isp_subdev, false);

		if (!timer_pending(&isp->wdt) &&
		    atomisp_buffers_queued(isp_subdev))
			mod_timer(&isp->wdt, jiffies + isp->wdt_duration);
	}
	mutex_unlock(&isp->mutex);
	IOCTL_EXIT
	return ret;

error:
	mutex_unlock(&isp->mutex);
	return ret;
}

static int atomisp_qbuf_file(struct file *file, void *fh,
					struct v4l2_buffer *buf)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);

	IOCTL_ENTER
	if (buf->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
    		v4l2_err(&atomisp_dev, "Unsupported buffer type\n");
    		return -EINVAL;
  	}

	IOCTL_EXIT
	return vb2_qbuf(&pipe->vb2outq, buf);
}

/*
 * Applications call the VIDIOC_DQBUF ioctl to dequeue a filled (capturing) or
 * displayed (output buffer)from the driver's outgoing queue
 */
static int atomisp_dqbuf(struct file *file, void *fh, struct v4l2_buffer *buf)
{
	int ret = 0;

	IOCTL_ENTER
	ret = vb2_ioctl_dqbuf(file, fh, buf);
	IOCTL_EXIT
	return ret;
}

static int atomisp_get_dc_buf(struct vb2_dc_buf *dc_buf,struct atomisp_device *isp, int pgnr)
{
	unsigned int ret = 0;
	struct sg_table *sgt;

	if(!dc_buf){
		dev_err(isp->dev,"%s: vb2 buffer is NULL\n",__func__);
		return -EINVAL;
	}

	sgt = kmalloc(sizeof(*sgt),GFP_KERNEL);
	if(!sgt){
		dev_err(isp->dev,"%s: failed to alloc sg table\n",__func__);
		return -ENOMEM;
	}

	sgt->orig_nents = pgnr;

	ret = sg_alloc_table(sgt,sgt->orig_nents,GFP_KERNEL);
	if(ret!=0){
		dev_err(isp->dev,"%s: failed to initialize sg_table\n",__func__);
		return -ENOMEM;
	}

	dc_buf->sgt_base = sgt;

	return 0;
}

static int atomisp_expbuf(struct file *file, void *priv, struct v4l2_exportbuffer *eb)
{
		struct video_device *vdev = video_devdata(file);
		struct atomisp_device *isp = video_get_drvdata(vdev);
		struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
		struct atomisp_sub_device *isp_subdev = atomisp_to_sub_device(pipe);
		struct vb2_queue *q = vdev->queue;
		struct vb2_buffer *vb2 = q->bufs[eb->index];
		struct vb2_plane *vb_plane;
		struct vb2_dc_buf *dc_buf;
		struct hrt_userbuffer_attr attributes;
		struct atomisp_vb2 *avb2 = container_of(vb2, struct atomisp_vb2, vb);
		struct ia_css_frame_info frame_info;
		struct ia_css_frame *handle = NULL;
		u32 length;
		u32 pgnr;
		int ret = 0;

		IOCTL_ENTER
		length = vb2_plane_size(vb2,0);
		pgnr = (length+(PAGE_SIZE -1)) >> PAGE_SHIFT;
		attributes.pgnr = pgnr;
		attributes.type = HRT_USR_PTR;
		vb_plane = &vb2->planes[eb->plane];

		if(!vb_plane->mem_priv){
				dev_err(isp->dev, "%s: private member error for a plane\n",__func__);
				ret = -EINVAL;
				return ret;
			}
		dc_buf = (struct vb2_dc_buf*)vb_plane->mem_priv;

		ret = atomisp_get_dc_buf(dc_buf,isp,pgnr);
		if(ret != IA_CSS_SUCCESS)
				dev_err(isp->dev,"Failed to create dc buffer\n");

		attributes.priv = (void*)dc_buf->sgt_base;


		ret = vb2_ioctl_expbuf(file,priv,eb);
		if(ret != IA_CSS_SUCCESS)
				dev_err(isp->dev,"Failed to export dma_buf\n");


		if (__get_css_frame_info(isp_subdev, pipe->pipe_type,&frame_info)) {
		      			ret = -EIO;
		      			return ret;
		    		}

				ret = ia_css_frame_map(&handle, &frame_info, NULL,0, &attributes);

		    		if (ret != IA_CSS_SUCCESS) {
		      			dev_err(isp->dev, "Failed ia_css_frame_dma \n");
		      			ret = -ENOMEM;
		      			return ret;
		    		}

				if (avb2->cssframe) {
					v4l2_dbg(4, dbg_level, &atomisp_dev, "WARNING cssframe already exists or was not initialized to NULL\n");
					ia_css_frame_free(avb2->cssframe);
					avb2->cssframe = NULL;
				}
				avb2->cssframe = handle;

		IOCTL_EXIT
		return ret;
}




enum ia_css_pipe_id atomisp_get_css_pipe_id(struct atomisp_sub_device *isp_subdev)
{
	if (isp_subdev->params.continuous_vf &&
	    isp_subdev->run_mode->val != ATOMISP_RUN_MODE_VIDEO)
		return IA_CSS_PIPE_ID_PREVIEW;

	if (!isp_subdev->enable_vfpp->val)
		return IA_CSS_PIPE_ID_CAPTURE;

	switch (isp_subdev->run_mode->val) {
	case ATOMISP_RUN_MODE_PREVIEW:
		return IA_CSS_PIPE_ID_PREVIEW;
	case ATOMISP_RUN_MODE_VIDEO:
		return IA_CSS_PIPE_ID_VIDEO;
	case ATOMISP_RUN_MODE_STILL_CAPTURE:
		/* fall through */
	default:
		return IA_CSS_PIPE_ID_CAPTURE;
	}
}

int atomisp_get_css_buf_type(struct atomisp_sub_device *isp_subdev,
			 struct atomisp_video_pipe *pipe)
{
	if (pipe->pipe_type == ATOMISP_PIPE_CAPTURE ||
	    (pipe->pipe_type == ATOMISP_PIPE_PREVIEW &&
	     isp_subdev->run_mode->val != ATOMISP_RUN_MODE_VIDEO))
		return IA_CSS_BUFFER_TYPE_OUTPUT_FRAME;
	else
		return IA_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME;
}

static unsigned int atomisp_sensor_start_stream(struct atomisp_sub_device *isp_subdev)
{
	if (!isp_subdev->enable_vfpp->val)
		return 1;

	if (isp_subdev->run_mode->val == ATOMISP_RUN_MODE_VIDEO ||
	    (isp_subdev->run_mode->val == ATOMISP_RUN_MODE_STILL_CAPTURE &&
	     !atomisp_is_mbuscode_raw(
		     isp_subdev->fmt[
			     isp_subdev->capture_pad].fmt.code) &&
	     !isp_subdev->params.continuous_vf))
		return 2;
	else
		return 1;
}


/*
 * This ioctl start the capture during streaming I/O.
 */
static int atomisp_streamon(struct file *file, void *fh,
	enum v4l2_buf_type type)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_sub_device *isp_subdev = atomisp_to_sub_device(pipe);
	enum ia_css_pipe_id css_pipe_id;
	unsigned int sensor_start_stream;
	int ret = 0;
#ifdef PUNIT_CAMERA_BUSY
	u32 msg_ret;
#endif

	IOCTL_ENTER
	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		dev_dbg(isp->dev, "unsupported v4l2 buf type\n");
		return -EINVAL;
	}

	mutex_lock(&isp->mutex);

	if (isp_subdev->streaming == ATOMISP_DEVICE_STREAMING_STOPPING) {
		ret = -EBUSY;
		goto out;
	}

	if (vb2_is_streaming(&pipe->vb2q)) {
		v4l2_dbg(6, dbg_level, &atomisp_dev, "DMA WARNING: pipe->vb2q.streaming. Need to verify\n");
		goto out;
	}

	/*
	 * The number of streaming video nodes is based on which
	 * binary is going to be run.
	 */
	sensor_start_stream = atomisp_sensor_start_stream(isp_subdev);

	trace_printk("DMA About to call vb2_ioctl_streamon\n");
	ret = vb2_ioctl_streamon(file, fh, type);
	trace_printk("Return from streamon: %d\n", ret);
	if (ret)
		goto out;

	if (atomisp_streaming_count(isp_subdev) > sensor_start_stream) {
		v4l2_dbg(6, dbg_level, &atomisp_dev, "DMA atomisp_streaming_count(isp) > sensor_start_stream\n");
		/* trigger still capture */
		if (isp_subdev->params.continuous_vf &&
		    pipe->pipe_type == ATOMISP_PIPE_CAPTURE &&
		    isp_subdev->run_mode->val != ATOMISP_RUN_MODE_VIDEO) {
			if (isp_subdev->delayed_init != ATOMISP_DELAYED_INIT_DONE){
				flush_work(&isp_subdev->delayed_init_work);
				mutex_unlock(&isp->mutex);
				if (wait_for_completion_interruptible(
						&isp_subdev->init_done) != 0)
					return -ERESTARTSYS;
				mutex_lock(&isp->mutex);
			}
			ret = ia_css_stream_capture(
					isp_subdev->css2_basis.stream,
					isp_subdev->params.offline_parm.num_captures,
					isp_subdev->params.offline_parm.skip_frames,
					isp_subdev->params.offline_parm.offset);
			if (ret)
				return -EINVAL;
		}
		atomisp_qbuffers_to_css(isp_subdev, false);
		goto out;
	}

	if (isp_subdev->streaming == ATOMISP_DEVICE_STREAMING_ENABLED) {
		atomisp_qbuffers_to_css(isp_subdev, false);
		goto start_sensor;
	}

#ifdef PUNIT_CAMERA_BUSY
	if (!IS_MRFLD && isp->need_gfx_throttle) {
		/*
		 * As per h/w architect and ECO 697611 we need to throttle the
		 * GFX performance (freq) while camera is up to prevent peak
		 * current issues. this is done by setting the camera busy bit.
		 */
		msg_ret = intel_mid_msgbus_read32(PUNIT_PORT, MFLD_OR1);
		msg_ret |= 0x100;
		intel_mid_msgbus_write32(PUNIT_PORT, MFLD_OR1, msg_ret);
	}
#endif

	css_pipe_id = atomisp_get_css_pipe_id(isp_subdev);

	ret = atomisp_acc_load_extensions(isp);
	if (ret < 0) {
		dev_err(isp->dev, "acc extension failed to load\n");
		goto out;
	}

	v4l2_dbg(4, dbg_level, &atomisp_dev, "About to call ia_css_start in atomisp_streamon\n");
	ret = ia_css_start(isp_subdev, false);
	if (ret != IA_CSS_SUCCESS) {
		dev_err(isp->dev, "ia_css_start fails: %d\n", ret);
		ret = -EINVAL;
		v4l2_dbg(2, dbg_level, &atomisp_dev, "<%s [%d] LINE:%d.\n",
			 __func__, pipe->pipe_type, __LINE__);
		goto out;
	}

	if (isp_subdev->params.continuous_vf &&
	    isp_subdev->run_mode->val != ATOMISP_RUN_MODE_VIDEO) {
		INIT_COMPLETION(isp_subdev->init_done);
		isp_subdev->delayed_init = ATOMISP_DELAYED_INIT_QUEUED;
		queue_work(isp_subdev->delayed_init_workq,
			   &isp_subdev->delayed_init_work);
	}

	/* Make sure that update_isp_params is called at least once.*/
	isp_subdev->params.css_update_params_needed = true;
	isp_subdev->streaming = ATOMISP_DEVICE_STREAMING_ENABLED;
	atomic_set(&isp_subdev->sof_count, -1);
	atomic_set(&isp_subdev->sequence, -1);
	atomic_set(&isp_subdev->sequence_temp, -1);
	atomic_set(&isp->wdt_count, 0);
	if (isp->sw_contex.file_input)
		isp->wdt_duration = ATOMISP_ISP_FILE_TIMEOUT_DURATION;
	else
		isp->wdt_duration = ATOMISP_ISP_TIMEOUT_DURATION;
	isp->fr_status = ATOMISP_FRAME_STATUS_OK;
	isp->sw_contex.invalid_frame = false;
	isp_subdev->params.dvs_proj_data_valid = false;

	v4l2_dbg(6, dbg_level, &atomisp_dev, "DMA wdt_duration=%u, About to call atomisp_qbuffers_to_css\n", isp->wdt_duration);
	atomisp_qbuffers_to_css(isp_subdev, false);

	/* Only start sensor when the last streaming instance started */
	/* File injection fails this check  */
#if 0
	if (atomisp_streaming_count(isp) < sensor_start_stream) {

		v4l2_dbg(2, dbg_level, &atomisp_dev, "<%s [%d] LINE:%d.\n",
			 __func__, pipe->pipe_type, __LINE__);
		goto out;
	}
#endif

start_sensor:
	if (isp->flash) {
		ret += v4l2_subdev_call(isp->flash, core, s_power, 1);
		isp_subdev->params.num_flash_frames = 0;
		isp_subdev->params.flash_state = ATOMISP_FLASH_IDLE;
		atomisp_setup_flash(isp_subdev);
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

	/* stream on the sensor */
		ret = v4l2_subdev_call(isp->inputs[isp_subdev->input_curr].camera,
				       video, s_stream, 1);
		if (ret) {
			atomisp_reset(isp);
			ret = -EINVAL;
		}

out:
	mutex_unlock(&isp->mutex);
	IOCTL_EXIT
	return ret;
}

int __atomisp_streamoff(struct file *file, void *fh, enum v4l2_buf_type type)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	struct atomisp_sub_device *isp_subdev = atomisp_to_sub_device(pipe);
	struct atomisp_vb2 *vb2 = NULL;
	int ret = 0;
	unsigned long flags;
	bool first_streamoff = false;
#ifdef PUNIT_CAMERA_BUSY
	u32 msg_ret;
#endif

	v4l2_dbg(2, dbg_level, &atomisp_dev, ">%s [%d]\n",
				__func__, pipe->pipe_type);
	BUG_ON(!mutex_is_locked(&isp->mutex));
	BUG_ON(!mutex_is_locked(&isp->streamoff_mutex));

	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		dev_dbg(isp->dev, "unsupported v4l2 buf type\n");
		return -EINVAL;
	}

	/*
	 * do only streamoff for capture & vf pipes in
	 * case of continuous capture
	 */
	if (isp_subdev->run_mode->val != ATOMISP_RUN_MODE_VIDEO &&
	    isp_subdev->params.continuous_vf &&
	    pipe->pipe_type != ATOMISP_PIPE_PREVIEW) {

		v4l2_dbg(6, dbg_level, &atomisp_dev, "VB2 streamoff for continuous_vf not supported\n");
		return -EINVAL;
	}
	if (isp_subdev->run_mode->val == ATOMISP_RUN_MODE_VIDEO){
	   if(pipe->pipe_type == ATOMISP_PIPE_VIEWFINDER){
		  if(pipe->buffers_in_css !=0){
			  v4l2_dbg(4, dbg_level, &atomisp_dev, "%s freeing up buffers in CSS\n",__func__);
			  atomisp_css_flush(isp);

		  }
		  atomisp_free_css_frames(&pipe->vb2q);
		  if (pipe->vb2q.streaming) return vb2_ioctl_streamoff(file, fh,  type);

	    }
	}


	atomisp_clear_frame_counters(isp_subdev);
	spin_lock_irqsave(&isp->lock, flags);
	if (isp_subdev->streaming == ATOMISP_DEVICE_STREAMING_ENABLED
	    || isp_subdev->streaming == ATOMISP_DEVICE_STREAMING_STARTING) {
		isp_subdev->streaming = ATOMISP_DEVICE_STREAMING_STOPPING;
		first_streamoff = true;
	}
	spin_unlock_irqrestore(&isp->lock, flags);

	if (first_streamoff) {
		mutex_unlock(&isp->mutex);
		/* not disable watch dog if other streams still running */
		if (!atomisp_subdev_streaming_count(isp_subdev->isp)) {
			del_timer_sync(&isp->wdt);
			cancel_work_sync(&isp->wdt_work);
		}

		/*
		 * must stop sending pixels into GP_FIFO before stop
		 * the pipeline.
		 */
		if (isp->sw_contex.file_input)
			v4l2_subdev_call(isp->inputs[isp_subdev->input_curr].camera,
					video, s_stream, 0);

		mutex_lock(&isp->mutex);
		atomisp_acc_unload_extensions(isp);
	}

	spin_lock_irqsave(&isp->lock, flags);
	if (atomisp_streaming_count(isp_subdev) <= 1)
		isp_subdev->streaming = ATOMISP_DEVICE_STREAMING_DISABLED;
	spin_unlock_irqrestore(&isp->lock, flags);

	if (!first_streamoff) {
		if(pipe->vb2q.streaming)
		{
			ret = vb2_ioctl_streamoff(file, fh,  type);
			if (ret)
				return ret;
		}
		goto stopsensor;
	}

	if (!isp->sw_contex.file_input)
		atomisp_control_irq_sof(isp);

	if (isp_subdev->delayed_init == ATOMISP_DELAYED_INIT_QUEUED) {
		cancel_work_sync(&isp_subdev->delayed_init_work);
		isp_subdev->delayed_init = ATOMISP_DELAYED_INIT_NOT_QUEUED;
	}

	/*Set the width, height etc. in atomisp_video_pipe to 0*/
	memset(&pipe->pix, 0, sizeof(struct v4l2_pix_format));

	ret = ia_css_stop(isp_subdev, false);

	if (ret) {
		v4l2_err(&atomisp_dev, "stop css failed, ret=%d.\n", ret);
		return ret;
	}

	//TODO DMA wake up interruptible pipes
	if(pipe->vb2q.streaming)
    		ret = vb2_ioctl_streamoff(file, fh,  type);

    	atomisp_clear_css_buffer_counters(isp_subdev);

	if (ret)
		return ret;

	spin_lock_irqsave(&pipe->irq_lock, flags);

	while (!list_empty(&pipe->activeq)) {
		vb2 = list_first_entry(&pipe->activeq, struct atomisp_vb2, list);
		if (!vb2)
			break;
		list_del(&vb2->list);
	}
	spin_unlock_irqrestore(&pipe->irq_lock, flags);

stopsensor:
	if (!isp->sw_contex.file_input)
		ret = v4l2_subdev_call(isp->inputs[isp_subdev->input_curr].camera,
				       video, s_stream, 0);

	if (isp->flash) {
		ret += v4l2_subdev_call(isp->flash, core, s_power, 0);
		isp_subdev->params.num_flash_frames = 0;
		isp_subdev->params.flash_state = ATOMISP_FLASH_IDLE;
	}

	atomisp_free_css_frames(&pipe->vb2q);
	/* if other streams are running, we should not power off isp */
	if (atomisp_subdev_streaming_count(isp))
		return 0;

#ifdef PUNIT_CAMERA_BUSY
	if (!IS_MRFLD && isp->need_gfx_throttle) {
		/* Free camera_busy bit */
		msg_ret = intel_mid_msgbus_read32(PUNIT_PORT, MFLD_OR1);
		msg_ret &= ~0x100;
		intel_mid_msgbus_write32(PUNIT_PORT, MFLD_OR1, msg_ret);
	}
#endif

	if (IS_MRFLD && atomisp_freq_scaling(isp, ATOMISP_DFS_MODE_LOW))
		v4l2_warn(&atomisp_dev, "DFS failed.\n");
	/*
	 * ISP work around, need to reset isp
	 * Is it correct time to reset ISP when first node does streamoff?
	 */
	if (isp->sw_contex.power_state == ATOM_ISP_POWER_UP) {
		unsigned int i;
		if (isp->isp_timeout)
			dev_err(isp->dev, "%s: Resetting with WA activated",
				__func__);
		/*
		 * It is possible that the other asd stream is in the stage
		 * that v4l2_setfmt is just get called on it, which will
		 * create css stream on that stream. But at this point, there
		 * is no way to destroy the css stream created on that stream.
		 *
		 * So force stream destroy here.
		 */
		for (i = 0; i < isp->num_of_streams; i++) {
			if (isp->isp_subdev[i].stream_prepared)
				atomisp_destroy_pipes_stream_force(&isp->
						isp_subdev[i]);
		}
		atomisp_reset(isp);
		isp->isp_timeout = false;
	}

	v4l2_dbg(2, dbg_level, &atomisp_dev, "<%s [%d]\n",
		 __func__, pipe->pipe_type);
	return ret;
}

static int atomisp_streamoff(struct file *file, void *fh,
			     enum v4l2_buf_type type)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	int rval;

	IOCTL_ENTER
	mutex_lock(&isp->streamoff_mutex);
	mutex_lock(&isp->mutex);
	rval = __atomisp_streamoff(file, fh, type);
	mutex_unlock(&isp->mutex);
	mutex_unlock(&isp->streamoff_mutex);
	IOCTL_EXIT

	return rval;
}

/*
 * To get the current value of a control.
 * applications initialize the id field of a struct v4l2_control and
 * call this ioctl with a pointer to this structure
 */
static int atomisp_g_ctrl(struct file *file, void *fh,
	struct v4l2_control *control)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_sub_device *isp_subdev = atomisp_to_sub_device(atomisp_to_video_pipe(vdev));
	int i, ret = -EINVAL;

	for (i = 0; i < ctrls_num; i++) {
		if (ci_v4l2_controls[i].id == control->id) {
			ret = 0;
			break;
		}
	}

	if (ret)
		return ret;

	mutex_lock(&isp->mutex);

	switch (control->id) {
	case V4L2_CID_IRIS_ABSOLUTE:
	case V4L2_CID_EXPOSURE_ABSOLUTE:
	case V4L2_CID_FNUMBER_ABSOLUTE:
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
	case V4L2_CID_2A_STATUS:
	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
	case V4L2_CID_EXPOSURE:
	case V4L2_CID_SCENE_MODE:
	case V4L2_CID_ISO_SENSITIVITY:
	case V4L2_CID_EXPOSURE_METERING:
	case V4L2_CID_CONTRAST:
	case V4L2_CID_SATURATION:
	case V4L2_CID_SHARPNESS:
	case V4L2_CID_3A_LOCK:
		mutex_unlock(&isp->mutex);
		return v4l2_subdev_call(isp->inputs[isp_subdev->input_curr].camera,
				       core, g_ctrl, control);
	case V4L2_CID_COLORFX:
		ret = atomisp_color_effect(isp_subdev, 0, &control->value);
		break;
	case V4L2_CID_ATOMISP_BAD_PIXEL_DETECTION:
		ret = atomisp_bad_pixel(isp_subdev, 0, &control->value);
		break;
	case V4L2_CID_ATOMISP_POSTPROCESS_GDC_CAC:
		ret = atomisp_gdc_cac(isp_subdev, 0, &control->value);
		break;
	case V4L2_CID_ATOMISP_VIDEO_STABLIZATION:
		ret = atomisp_video_stable(isp_subdev, 0, &control->value);
		break;
	case V4L2_CID_ATOMISP_FIXED_PATTERN_NR:
		ret = atomisp_fixed_pattern(isp_subdev, 0, &control->value);
		break;
	case V4L2_CID_ATOMISP_FALSE_COLOR_CORRECTION:
		ret = atomisp_false_color(isp_subdev, 0, &control->value);
		break;
	case V4L2_CID_ATOMISP_LOW_LIGHT:
		ret = atomisp_low_light(isp_subdev, 0, &control->value);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&isp->mutex);
	return ret;
}

/*
 * To change the value of a control.
 * applications initialize the id and value fields of a struct v4l2_control
 * and call this ioctl.
 */
static int atomisp_s_ctrl(struct file *file, void *fh,
			  struct v4l2_control *control)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_sub_device *isp_subdev = atomisp_to_sub_device(atomisp_to_video_pipe(vdev));
	int i, ret = -EINVAL;

	for (i = 0; i < ctrls_num; i++) {
		if (ci_v4l2_controls[i].id == control->id) {
			ret = 0;
			break;
		}
	}

	if (ret)
		return ret;

	mutex_lock(&isp->mutex);
	switch (control->id) {
	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
	case V4L2_CID_EXPOSURE:
	case V4L2_CID_SCENE_MODE:
	case V4L2_CID_ISO_SENSITIVITY:
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
	case V4L2_CID_POWER_LINE_FREQUENCY:
	case V4L2_CID_EXPOSURE_METERING:
	case V4L2_CID_CONTRAST:
	case V4L2_CID_SATURATION:
	case V4L2_CID_SHARPNESS:
	case V4L2_CID_3A_LOCK:
		mutex_unlock(&isp->mutex);
		return v4l2_subdev_call(isp->inputs[isp_subdev->input_curr].camera,
				       core, s_ctrl, control);
	case V4L2_CID_COLORFX:
		ret = atomisp_color_effect(isp_subdev, 1, &control->value);
		break;
	case V4L2_CID_ATOMISP_BAD_PIXEL_DETECTION:
		ret = atomisp_bad_pixel(isp_subdev, 1, &control->value);
		break;
	case V4L2_CID_ATOMISP_POSTPROCESS_GDC_CAC:
		ret = atomisp_gdc_cac(isp_subdev, 1, &control->value);
		break;
	case V4L2_CID_ATOMISP_VIDEO_STABLIZATION:
		ret = atomisp_video_stable(isp_subdev, 1, &control->value);
		break;
	case V4L2_CID_ATOMISP_FIXED_PATTERN_NR:
		ret = atomisp_fixed_pattern(isp_subdev, 1, &control->value);
		break;
	case V4L2_CID_ATOMISP_FALSE_COLOR_CORRECTION:
		ret = atomisp_false_color(isp_subdev, 1, &control->value);
		break;
	case V4L2_CID_REQUEST_FLASH:
		ret = atomisp_flash_enable(isp_subdev, control->value);
		break;
	case V4L2_CID_ATOMISP_LOW_LIGHT:
		ret = atomisp_low_light(isp_subdev, 1, &control->value);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&isp->mutex);
	return ret;
}
/*
 * To query the attributes of a control.
 * applications set the id field of a struct v4l2_queryctrl and call the
 * this ioctl with a pointer to this structure. The driver fills
 * the rest of the structure.
 */
static int atomisp_queryctl(struct file *file, void *fh,
			    struct v4l2_queryctrl *qc)
{
	int i, ret = -EINVAL;

	IOCTL_ENTER
	if (qc->id & V4L2_CTRL_FLAG_NEXT_CTRL)
		return ret;

	for (i = 0; i < ctrls_num; i++) {
		if (ci_v4l2_controls[i].id == qc->id) {
			memcpy(qc, &ci_v4l2_controls[i],
			       sizeof(struct v4l2_queryctrl));
			qc->reserved[0] = 0;
			ret = 0;
			break;
		}
	}
	if (ret != 0)
		qc->flags = V4L2_CTRL_FLAG_DISABLED;

	IOCTL_EXIT
	return ret;
}

static int atomisp_camera_g_ext_ctrls(struct file *file, void *fh,
	struct v4l2_ext_controls *c)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_sub_device *isp_subdev = atomisp_to_sub_device(atomisp_to_video_pipe(vdev));
	struct v4l2_control ctrl;
	int i;
	int ret = 0;

	for (i = 0; i < c->count; i++) {
		ctrl.id = c->controls[i].id;
		ctrl.value = c->controls[i].value;
		switch (ctrl.id) {
		case V4L2_CID_EXPOSURE_ABSOLUTE:
		case V4L2_CID_IRIS_ABSOLUTE:
		case V4L2_CID_FNUMBER_ABSOLUTE:
		case V4L2_CID_BIN_FACTOR_HORZ:
		case V4L2_CID_BIN_FACTOR_VERT:
		case V4L2_CID_3A_LOCK:
			/*
			 * Exposure related control will be handled by sensor
			 * driver
			 */
			ret = v4l2_subdev_call(isp->inputs
					       [isp_subdev->input_curr].camera,
					       core, g_ctrl, &ctrl);
			break;
		case V4L2_CID_FOCUS_ABSOLUTE:
		case V4L2_CID_FOCUS_RELATIVE:
		case V4L2_CID_FOCUS_STATUS:
		case V4L2_CID_FOCUS_AUTO:
			if (isp->inputs[isp_subdev->input_curr].motor)
				ret = v4l2_subdev_call(
					isp->inputs[isp_subdev->input_curr].motor,
					core, g_ctrl, &ctrl);
			else
				ret = v4l2_subdev_call(
					isp->inputs[isp_subdev->input_curr].camera,
					core, g_ctrl, &ctrl);
			break;
		case V4L2_CID_FLASH_STATUS:
		case V4L2_CID_FLASH_INTENSITY:
		case V4L2_CID_FLASH_TORCH_INTENSITY:
		case V4L2_CID_FLASH_INDICATOR_INTENSITY:
		case V4L2_CID_FLASH_TIMEOUT:
		case V4L2_CID_FLASH_STROBE:
		case V4L2_CID_FLASH_MODE:
			if (isp->flash)
				ret = v4l2_subdev_call(
					isp->flash, core, g_ctrl, &ctrl);
			break;
		case V4L2_CID_ZOOM_ABSOLUTE:
			mutex_lock(&isp->mutex);
			ret = atomisp_digital_zoom(isp_subdev, 0, &ctrl.value);
			mutex_unlock(&isp->mutex);
			break;
		case V4L2_CID_G_SKIP_FRAMES:
			ret = v4l2_subdev_call(
				isp->inputs[isp_subdev->input_curr].camera,
				sensor, g_skip_frames, (u32 *)&ctrl.value);
			break;
		default:
			ret = -EINVAL;
		}

		if (ret) {
			c->error_idx = i;
			break;
		}
		c->controls[i].value = ctrl.value;
	}
	return ret;
}

/* This ioctl allows the application to get multiple controls by class */
static int atomisp_g_ext_ctrls(struct file *file, void *fh,
	struct v4l2_ext_controls *c)
{
	struct v4l2_control ctrl;
	int i, ret = 0;

	IOCTL_ENTER
	/* input_lock is not need for the Camera releated IOCTLs
	 * The input_lock downgrade the FPS of 3A*/
	ret = atomisp_camera_g_ext_ctrls(file, fh, c);
	if (ret != -EINVAL)
		return ret;

	for (i = 0; i < c->count; i++) {
		ctrl.id = c->controls[i].id;
		ctrl.value = c->controls[i].value;
		ret = atomisp_g_ctrl(file, fh, &ctrl);
		c->controls[i].value = ctrl.value;
		if (ret) {
			c->error_idx = i;
			break;
		}
	}
	IOCTL_EXIT
	return ret;
}

static int atomisp_camera_s_ext_ctrls(struct file *file, void *fh,
	struct v4l2_ext_controls *c)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_sub_device *isp_subdev = atomisp_to_sub_device(atomisp_to_video_pipe(vdev));
	struct v4l2_control ctrl;
	int i;
	int ret = 0;

	for (i = 0; i < c->count; i++) {
		struct v4l2_ctrl *ctr;

		ctrl.id = c->controls[i].id;
		ctrl.value = c->controls[i].value;
		switch (ctrl.id) {
		case V4L2_CID_EXPOSURE_ABSOLUTE:
		case V4L2_CID_IRIS_ABSOLUTE:
		case V4L2_CID_FNUMBER_ABSOLUTE:
		case V4L2_CID_VCM_TIMEING:
		case V4L2_CID_VCM_SLEW:
		case V4L2_CID_TEST_PATTERN:
		case V4L2_CID_3A_LOCK:
			ret = v4l2_subdev_call(
				isp->inputs[isp_subdev->input_curr].camera,
				core, s_ctrl, &ctrl);
			break;
		case V4L2_CID_FOCUS_ABSOLUTE:
		case V4L2_CID_FOCUS_RELATIVE:
		case V4L2_CID_FOCUS_STATUS:
		case V4L2_CID_FOCUS_AUTO:
			if (isp->inputs[isp_subdev->input_curr].motor)
				ret = v4l2_subdev_call(
					isp->inputs[isp_subdev->input_curr].motor,
					core, s_ctrl, &ctrl);
			else
				ret = v4l2_subdev_call(
					isp->inputs[isp_subdev->input_curr].camera,
					core, s_ctrl, &ctrl);
			break;
		case V4L2_CID_FLASH_STATUS:
		case V4L2_CID_FLASH_INTENSITY:
		case V4L2_CID_FLASH_TORCH_INTENSITY:
		case V4L2_CID_FLASH_INDICATOR_INTENSITY:
		case V4L2_CID_FLASH_TIMEOUT:
		case V4L2_CID_FLASH_STROBE:
		case V4L2_CID_FLASH_MODE:
			mutex_lock(&isp->mutex);
			if (isp->flash) {
				ret = v4l2_subdev_call(isp->flash,
					core, s_ctrl, &ctrl);
				/* When flash mode is changed we need to reset
				 * flash state */
				if (ctrl.id == V4L2_CID_FLASH_MODE) {
					isp_subdev->params.flash_state = ATOMISP_FLASH_IDLE;
					isp_subdev->params.num_flash_frames = 0;
				}
			}
			mutex_unlock(&isp->mutex);
			break;
		case V4L2_CID_ZOOM_ABSOLUTE:
			mutex_lock(&isp->mutex);
			ret = atomisp_digital_zoom(isp_subdev, 1, &ctrl.value);
			mutex_unlock(&isp->mutex);
			break;
		default:
			ctr = v4l2_ctrl_find(&isp_subdev->ctrl_handler,
					     ctrl.id);
			if (ctr)
				ret = v4l2_ctrl_s_ctrl(ctr, ctrl.value);
			else
				ret = -EINVAL;
		}

		if (ret) {
			c->error_idx = i;
			break;
		}
		c->controls[i].value = ctrl.value;
	}
	return ret;
}

/* This ioctl allows the application to set multiple controls by class */
static int atomisp_s_ext_ctrls(struct file *file, void *fh,
	struct v4l2_ext_controls *c)
{
	struct v4l2_control ctrl;
	int i, ret = 0;

	IOCTL_ENTER
	/* input_lock is not need for the Camera releated IOCTLs
	 * The input_lock downgrade the FPS of 3A*/
	ret = atomisp_camera_s_ext_ctrls(file, fh, c);
	if (ret != -EINVAL)
		return ret;

	for (i = 0; i < c->count; i++) {
		ctrl.id = c->controls[i].id;
		ctrl.value = c->controls[i].value;
		ret = atomisp_s_ctrl(file, fh, &ctrl);
		c->controls[i].value = ctrl.value;
		if (ret) {
			c->error_idx = i;
			break;
		}
	}
	IOCTL_EXIT
	return ret;
}

/*
 * vidioc_g/s_param are used to switch isp running mode
 */
static int atomisp_g_parm(struct file *file, void *fh,
	struct v4l2_streamparm *parm)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_sub_device *isp_subdev = atomisp_to_sub_device(atomisp_to_video_pipe(vdev));

	IOCTL_ENTER
	if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		v4l2_err(&atomisp_dev,
			    "unsupport v4l2 buf type\n");
		return -EINVAL;
	}

	mutex_lock(&isp->mutex);
	parm->parm.capture.capturemode = isp_subdev->run_mode->val;
	mutex_unlock(&isp->mutex);
	IOCTL_EXIT

	return 0;
}

static int atomisp_s_parm(struct file *file, void *fh,
	struct v4l2_streamparm *parm)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_sub_device *isp_subdev = atomisp_to_sub_device(atomisp_to_video_pipe(vdev));
	struct v4l2_subdev_frame_interval fi;
	int mode;
	int rval;

	IOCTL_ENTER

	if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		v4l2_err(&atomisp_dev,
			    "unsupport v4l2 buf type\n");
		return -EINVAL;
	}

	mutex_lock(&isp->mutex);

	switch (parm->parm.capture.capturemode) {
	case CI_MODE_NONE:
		trace_printk("CI_MODE_NONE\n");
		memset(&fi, 0, sizeof(fi));
		fi.interval = parm->parm.capture.timeperframe;

		rval = v4l2_subdev_call(isp->inputs[isp_subdev->input_curr].camera,
					video, s_frame_interval, &fi);
		if (!rval)
			parm->parm.capture.timeperframe = fi.interval;
		goto out;
	case CI_MODE_VIDEO:
		trace_printk("CI_MODE_VIDEO\n");
		mode = ATOMISP_RUN_MODE_VIDEO;
		break;
	case CI_MODE_STILL_CAPTURE:
		trace_printk("CI_MODE_STILL_CAPTURE\n");
		mode = ATOMISP_RUN_MODE_STILL_CAPTURE;
		break;
	case CI_MODE_CONTINUOUS:
		trace_printk("CI_MODE_CONTINOUS\n");
		mode = ATOMISP_RUN_MODE_CONTINUOUS_CAPTURE;
		break;
	case CI_MODE_PREVIEW:
		trace_printk("CI_MODE_PREVIEW\n");
		mode = ATOMISP_RUN_MODE_PREVIEW;
		break;
	default:
		rval = -EINVAL;
		goto out;
	}

	rval = v4l2_ctrl_s_ctrl(isp_subdev->run_mode, mode);

out:
	mutex_unlock(&isp->mutex);

	IOCTL_EXIT
	return rval == -ENOIOCTLCMD ? 0 : rval;
}

static int atomisp_s_parm_file(struct file *file, void *fh,
				struct v4l2_streamparm *parm)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_sub_device *isp_subdev =
	    atomisp_to_sub_device(atomisp_to_video_pipe(vdev));

	IOCTL_ENTER
	if (parm->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		v4l2_err(&atomisp_dev,
			    "unsupport v4l2 buf type for output\n");
		return -EINVAL;
	}

	/*
	 * only isp_subdev[0] support file injection function.
	 *
	 * This is to simplify the driver design, since there would be no
	 * context passed to atomisp_file.c to state which subdev is using the
	 * file injection.
	 *
	 * Just hardcode in atomisp_file.c that isp_subdev[0] is always used.
	 *
	 * also, we should not have the obscure UCs that for example, one
	 * stream is in sensor mode, while the other is in file injection.
	 */
	if (isp_subdev->index != 0)
		return 0;

	mutex_lock(&isp->mutex);
	isp->sw_contex.file_input = 1;
	mutex_unlock(&isp->mutex);

	IOCTL_EXIT
	return 0;
}

/* set default atomisp ioctl value */
static long atomisp_vidioc_default(struct file *file, void *fh,
	bool valid_prio, int cmd, void *arg)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	struct atomisp_sub_device *isp_subdev = atomisp_to_sub_device(pipe);
	int err;
	struct v4l2_subdev_format *fmt;

	v4l2_dbg(5, dbg_level, &atomisp_dev, ">%s. 0x%x\n", __func__, cmd);
	mutex_lock(&isp->mutex);
	switch (cmd) {
	case VIDIOC_SUBDEV_G_FMT:
		fmt = (struct v4l2_subdev_format *)arg;

		if (fmt->which != V4L2_SUBDEV_FORMAT_TRY &&
                    fmt->which != V4L2_SUBDEV_FORMAT_ACTIVE)
                        return -EINVAL;

                if (fmt->pad >= isp_subdev->subdev.entity.num_pads)
                        return -EINVAL;

		fmt->format = *atomisp_subdev_get_ffmt(isp_subdev, fh, fmt->which, fmt->pad);
		err = 0;
		break;
	case VIDIOC_SUBDEV_S_FMT:
		fmt = (struct v4l2_subdev_format *)arg;
		if (fmt->which != V4L2_SUBDEV_FORMAT_TRY &&
                    fmt->which != V4L2_SUBDEV_FORMAT_ACTIVE)
                        return -EINVAL;

                if (fmt->pad >= isp_subdev->subdev.entity.num_pads)
                        return -EINVAL;

		err = atomisp_subdev_set_ffmt(isp_subdev, fh, fmt->which, fmt->pad,
                                       &fmt->format);
		break;
	case ATOMISP_FW_LOAD:
		v4l2_dbg(1, dbg_level, &atomisp_dev, "Entering ATOMISP_FW_LOAD ioctl\n");
		err = atomisp_fw(isp, 0, arg);
		v4l2_dbg(1, dbg_level, &atomisp_dev, "Exiting ATOMISP_FW_LOAD ioctl call\n");
		break;
	case ATOMISP_IOC_G_XNR:
		err = atomisp_xnr(isp_subdev, 0, arg);
		break;

	case ATOMISP_IOC_S_XNR:
		err = atomisp_xnr(isp_subdev, 1, arg);
		break;

	case ATOMISP_IOC_G_NR:
		err = atomisp_nr(isp_subdev, 0, arg);
		break;

	case ATOMISP_IOC_S_NR:
		err = atomisp_nr(isp_subdev, 1, arg);
		break;

	case ATOMISP_IOC_G_TNR:
		err = atomisp_tnr(isp_subdev, 0, arg);
		break;

	case ATOMISP_IOC_S_TNR:
		err = atomisp_tnr(isp_subdev, 1, arg);
		break;

	case ATOMISP_IOC_G_HISTOGRAM:
		err = atomisp_histogram(isp_subdev, 0, arg);
		break;

	case ATOMISP_IOC_S_HISTOGRAM:
		err = atomisp_histogram(isp_subdev, 1, arg);
		break;

	case ATOMISP_IOC_G_BLACK_LEVEL_COMP:
		err = atomisp_black_level(isp_subdev, 0, arg);
		break;

	case ATOMISP_IOC_S_BLACK_LEVEL_COMP:
		err = atomisp_black_level(isp_subdev, 1, arg);
		break;

	case ATOMISP_IOC_G_EE:
		err = atomisp_ee(isp_subdev, 0, arg);
		break;

	case ATOMISP_IOC_S_EE:
		err = atomisp_ee(isp_subdev, 1, arg);
		break;

	case ATOMISP_IOC_G_DIS_STAT:
		err = atomisp_get_dis_stat(isp_subdev, arg);
		break;

	case ATOMISP_IOC_S_DIS_COEFS:
		err = atomisp_set_dis_coefs(isp_subdev, arg);
		break;

	case ATOMISP_IOC_S_DIS_VECTOR:
		err = atomisp_set_dis_vector(isp_subdev, arg);
		break;

	case ATOMISP_IOC_G_ISP_PARM:
		err = atomisp_param(isp_subdev, 0, arg);
		break;

	case ATOMISP_IOC_S_ISP_PARM:
		err = atomisp_param(isp_subdev, 1, arg);
		break;

	case ATOMISP_IOC_G_3A_STAT:
		err = atomisp_3a_stat(isp_subdev, 0, arg);
		break;

	case ATOMISP_IOC_G_ISP_GAMMA:
		err = atomisp_gamma(isp_subdev, 0, arg);
		break;

	case ATOMISP_IOC_S_ISP_GAMMA:
		err = atomisp_gamma(isp_subdev, 1, arg);
		break;

	case ATOMISP_IOC_G_ISP_GDC_TAB:
		err = atomisp_gdc_cac_table(isp_subdev, 0, arg);
		break;

	case ATOMISP_IOC_S_ISP_GDC_TAB:
		err = atomisp_gdc_cac_table(isp_subdev, 1, arg);
		break;

	case ATOMISP_IOC_G_ISP_MACC:
		err = atomisp_macc_table(isp_subdev, 0, arg);
		break;

	case ATOMISP_IOC_S_ISP_MACC:
		err = atomisp_macc_table(isp_subdev, 1, arg);
		break;

	case ATOMISP_IOC_G_ISP_BAD_PIXEL_DETECTION:
		err = atomisp_bad_pixel_param(isp_subdev, 0, arg);
		break;

	case ATOMISP_IOC_S_ISP_BAD_PIXEL_DETECTION:
		err = atomisp_bad_pixel_param(isp_subdev, 1, arg);
		break;

	case ATOMISP_IOC_G_ISP_FALSE_COLOR_CORRECTION:
		err = atomisp_false_color_param(isp_subdev, 0, arg);
		break;

	case ATOMISP_IOC_S_ISP_FALSE_COLOR_CORRECTION:
		err = atomisp_false_color_param(isp_subdev, 1, arg);
		break;

	case ATOMISP_IOC_G_ISP_CTC:
		err = atomisp_ctc(isp_subdev, 0, arg);
		break;

	case ATOMISP_IOC_S_ISP_CTC:
		err = atomisp_ctc(isp_subdev, 1, arg);
		break;

	case ATOMISP_IOC_G_ISP_WHITE_BALANCE:
		err = atomisp_white_balance_param(isp_subdev, 0, arg);
		break;

	case ATOMISP_IOC_S_ISP_WHITE_BALANCE:
		err = atomisp_white_balance_param(isp_subdev, 1, arg);
		break;

	case ATOMISP_IOC_G_3A_CONFIG:
		err = atomisp_3a_config_param(isp_subdev, 0, arg);
		break;

	case ATOMISP_IOC_S_3A_CONFIG:
		err = atomisp_3a_config_param(isp_subdev, 1, arg);
		break;

	case ATOMISP_IOC_S_ISP_FPN_TABLE:
		err = atomisp_fixed_pattern_table(isp_subdev, arg);
		break;

	case ATOMISP_IOC_ISP_MAKERNOTE:
		err = atomisp_exif_makernote(isp_subdev, arg);
		break;

	case ATOMISP_IOC_G_SENSOR_MODE_DATA:
		err = atomisp_get_sensor_mode_data(isp_subdev, arg);
		break;

	case ATOMISP_IOC_G_MOTOR_PRIV_INT_DATA:
		mutex_unlock(&isp->mutex);
		if (isp->inputs[isp_subdev->input_curr].motor)
			return v4l2_subdev_call(
					isp->inputs[isp_subdev->input_curr].motor,
					core, ioctl, cmd, arg);
		else
			return v4l2_subdev_call(
					isp->inputs[isp_subdev->input_curr].camera,
					core, ioctl, cmd, arg);

	case ATOMISP_IOC_S_EXPOSURE:
	case ATOMISP_IOC_G_SENSOR_CALIBRATION_GROUP:
	case ATOMISP_IOC_G_SENSOR_PRIV_INT_DATA:
		mutex_unlock(&isp->mutex);
		return v4l2_subdev_call(isp->inputs[isp_subdev->input_curr].camera,
					core, ioctl, cmd, arg);

	case ATOMISP_IOC_ACC_LOAD:
		err = atomisp_acc_load(isp, arg);
		break;

	case ATOMISP_IOC_ACC_LOAD_TO_PIPE:
		err = atomisp_acc_load_to_pipe(isp, arg);
		break;

	case ATOMISP_IOC_ACC_UNLOAD:
		err = atomisp_acc_unload(isp, arg);
		break;

	case ATOMISP_IOC_ACC_START:
		err = atomisp_acc_start(isp, arg);
		break;

	case ATOMISP_IOC_ACC_WAIT:
		err = atomisp_acc_wait(isp, arg);
		break;

	case ATOMISP_IOC_ACC_MAP:
		err = atomisp_acc_map(isp, arg);
		break;

	case ATOMISP_IOC_ACC_UNMAP:
		err = atomisp_acc_unmap(isp, arg);
		break;

	case ATOMISP_IOC_ACC_S_MAPPED_ARG:
		err = atomisp_acc_s_mapped_arg(isp, arg);
		break;

	case ATOMISP_IOC_CAMERA_BRIDGE:
		err = -EINVAL;
		break;

	case ATOMISP_IOC_S_ISP_SHD_TAB:
		err = atomisp_set_shading_table(isp_subdev, arg);
		break;

	case ATOMISP_IOC_G_ISP_GAMMA_CORRECTION:
		err = atomisp_gamma_correction(isp_subdev, 0, arg);
		break;

	case ATOMISP_IOC_S_ISP_GAMMA_CORRECTION:
		err = atomisp_gamma_correction(isp_subdev, 1, arg);
		break;

	case ATOMISP_IOC_S_PARAMETERS:
		err = atomisp_set_parameters(isp_subdev, arg);
		break;

	case ATOMISP_IOC_S_CONT_CAPTURE_CONFIG:
		err = atomisp_offline_capture_configure(isp_subdev, arg);
		break;

	default:
		mutex_unlock(&isp->mutex);
		return -EINVAL;
	}
	mutex_unlock(&isp->mutex);
	v4l2_dbg(5, dbg_level, &atomisp_dev, "<%s.\n", __func__);
	return err;
}

const struct v4l2_ioctl_ops atomisp_ioctl_ops = {
	.vidioc_querycap = atomisp_querycap,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0)
	.vidioc_g_chip_ident = atomisp_g_chip_ident,
#endif
	.vidioc_enum_input = atomisp_enum_input,
	.vidioc_g_input = atomisp_g_input,
	.vidioc_s_input = atomisp_s_input,
	.vidioc_queryctrl = atomisp_queryctl,
	.vidioc_s_ctrl = atomisp_s_ctrl,
	.vidioc_g_ctrl = atomisp_g_ctrl,
	.vidioc_s_ext_ctrls = atomisp_s_ext_ctrls,
	.vidioc_g_ext_ctrls = atomisp_g_ext_ctrls,
	.vidioc_enum_fmt_vid_cap = atomisp_enum_fmt_cap,
	.vidioc_try_fmt_vid_cap = atomisp_try_fmt_cap,
	.vidioc_g_fmt_vid_cap = atomisp_g_fmt_cap,
	.vidioc_s_fmt_vid_cap = atomisp_s_fmt_cap,
	//.vidioc_s_fmt_type_private = atomisp_s_fmt_cap,
	.vidioc_reqbufs = atomisp_reqbufs,
	.vidioc_querybuf = atomisp_querybuf,
	.vidioc_qbuf = atomisp_qbuf,
	.vidioc_dqbuf = atomisp_dqbuf,
	.vidioc_expbuf = atomisp_expbuf,
	.vidioc_streamon = atomisp_streamon,
	.vidioc_streamoff = atomisp_streamoff,
	.vidioc_default = atomisp_vidioc_default,
	.vidioc_enum_frameintervals = atomisp_enum_frameintervals,
	.vidioc_s_parm = atomisp_s_parm,
	.vidioc_g_parm = atomisp_g_parm,
};

const struct v4l2_ioctl_ops atomisp_file_ioctl_ops = {
	.vidioc_querycap = atomisp_querycap,
	.vidioc_g_fmt_vid_out = atomisp_g_fmt_file,
	.vidioc_s_fmt_vid_out = atomisp_s_fmt_file,
	.vidioc_s_parm = atomisp_s_parm_file,
	.vidioc_reqbufs = atomisp_reqbufs_file,
	.vidioc_querybuf = atomisp_querybuf_file,
	.vidioc_qbuf = atomisp_qbuf_file,
	/* .vidioc_streamon = atomisp_streamon_out, */
};
