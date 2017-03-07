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

#include <media/v4l2-event.h>
#include <media/v4l2-mediabus.h>

#include <linux/delay.h>

#include <ia_css.h>

#include "atomisp_cmd.h"
#include "atomisp_common.h"
#include "atomisp_file.h"
#include "atomisp_internal.h"
#include "atomisp_ioctl.h"

static void file_work(struct work_struct *work)
{
	struct atomisp_file_device *file_dev = container_of(work,
					struct atomisp_file_device, work_thread);
	struct atomisp_device *isp = file_dev->isp;
	struct atomisp_sub_device *isp_subdev = &isp->isp_subdev[0];
	struct atomisp_video_pipe *out_pipe = &isp_subdev->video_in;
	unsigned short *buf = NULL;
	struct v4l2_mbus_framefmt isp_sink_fmt;

	if (isp_subdev->streaming != ATOMISP_DEVICE_STREAMING_ENABLED)
		return;

	dev_dbg(isp->dev, ">%s: ready to start streaming\n", __func__);
	isp_sink_fmt = *atomisp_subdev_get_ffmt(isp_subdev, NULL,
						V4L2_SUBDEV_FORMAT_ACTIVE,
						ATOMISP_SUBDEV_PAD_SINK);
	while (1) {
		wait_for_completion(&file_dev->file_inject_start);
		INIT_COMPLETION(file_dev->file_inject_start);

		if (out_pipe->vb2outq.bufs[file_dev->framecount] == NULL)
			file_dev->framecount = 0;
		buf = vb2_plane_vaddr(out_pipe->vb2outq.bufs[file_dev->framecount], 0);
		file_dev->framecount++;

		if (isp_subdev->streaming != ATOMISP_DEVICE_STREAMING_ENABLED || !file_dev->fStreaming)
			return;

		while (!ia_css_isp_has_started())
			usleep_range(1000, 1500);

		ia_css_stream_send_input_frame(isp_subdev->css2_basis.stream,
				       buf, isp_sink_fmt.width, isp_sink_fmt.height);
	}

	dev_dbg(isp->dev, "<%s: streaming done\n", __func__);
}

static int file_input_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct atomisp_file_device *file_dev = v4l2_get_subdevdata(sd);
	struct atomisp_device *isp = file_dev->isp;

	dev_dbg(isp->dev, "%s: enable %d\n", __func__, enable);
	if (enable) {
		if (isp->isp_subdev[0].streaming != ATOMISP_DEVICE_STREAMING_ENABLED)
			return 0;

		if (!file_dev->fWork_queued) {
			queue_work(file_dev->work_queue, &file_dev->work_thread);
			file_dev->fWork_queued = true;
		}

		file_dev->fStreaming = true;
		complete(&file_dev->file_inject_start);
	} else {
		if (!file_dev->fWork_queued)
			return 0;

		file_dev->fStreaming = false;
		complete(&file_dev->file_inject_start);
		cancel_work_sync(&file_dev->work_thread);
		file_dev->fWork_queued = false;
	}
	return 0;
}

static int file_input_g_parm(struct v4l2_subdev *sd,
		struct v4l2_streamparm *param)
{
	/*to fake*/
	return 0;
}

static int file_input_s_parm(struct v4l2_subdev *sd,
		struct v4l2_streamparm *param)
{
	/*to fake*/
	return 0;
}

static int file_input_enum_framesizes(struct v4l2_subdev *sd,
				  struct v4l2_frmsizeenum *fsize)
{
	/*to fake*/
	return 0;
}

static int file_input_enum_frameintervals(struct v4l2_subdev *sd,
				      struct v4l2_frmivalenum *fival)
{
	/*to fake*/
	return 0;
}

static int file_input_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned int index,
				enum media_bus_format *code)
{
	/*to fake*/
	return 0;
}

static int file_input_g_mbus_fmt(struct v4l2_subdev *sd,
			     struct v4l2_mbus_framefmt *fmt)
{
	struct atomisp_file_device *file_dev = v4l2_get_subdevdata(sd);
	struct atomisp_device *isp = file_dev->isp;
	struct v4l2_mbus_framefmt *isp_sink_fmt;

	isp_sink_fmt = atomisp_subdev_get_ffmt(&isp->isp_subdev[0], NULL,
					       V4L2_SUBDEV_FORMAT_ACTIVE,
					       ATOMISP_SUBDEV_PAD_SINK);

	fmt->width = isp_sink_fmt->width;
	fmt->height = isp_sink_fmt->height;
	trace_printk("WARNING: in file_input_g_mbus_fmt, overwriting MBUS FMT to FIXED!!!. %dx%d\n", fmt->width, fmt->height);
	fmt->code = isp_sink_fmt->code; //TODO set right code

	return 0;
}

static int file_input_s_mbus_fmt(struct v4l2_subdev *sd,
			     struct v4l2_mbus_framefmt *fmt)
{
	struct atomisp_file_device *file_dev = v4l2_get_subdevdata(sd);
        struct atomisp_device *isp = file_dev->isp;

	file_input_g_mbus_fmt(sd, fmt);
	isp->isp_subdev[0].css2_basis.stream_config.mode = IA_CSS_INPUT_MODE_FIFO;
	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0)
static int file_input_g_chip_ident(struct v4l2_subdev *sd,
			       struct v4l2_dbg_chip_ident *chip)
{
	if (!chip)
		return -EINVAL;

	return 0;
}
#endif

static int file_input_log_status(struct v4l2_subdev *sd)
{
	/*to fake*/
	return 0;
}

static int file_input_queryctrl(struct v4l2_subdev *sd,
		struct v4l2_queryctrl *qc)
{
	/*to fake*/
	return -EINVAL;
}

static int file_input_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	/*to fake*/
	return -EINVAL;
}

static int file_input_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	/*to fake*/
	return 0;
}

static int file_input_s_power(struct v4l2_subdev *sd, int on)
{
	/* to fake */
	return 0;
}

static int file_input_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_fh *fh,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	/*to fake*/
	return 0;
}

static int file_input_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	/*to fake*/
	return 0;
}

static int file_input_enum_frame_ival(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_frame_interval_enum *fie)
{
	/*to fake*/
	return 0;
}

static const struct v4l2_subdev_video_ops file_input_video_ops = {
	.s_stream = file_input_s_stream,
	.g_parm = file_input_g_parm,
	.s_parm = file_input_s_parm,
	.enum_framesizes = file_input_enum_framesizes,
	.enum_frameintervals = file_input_enum_frameintervals,
	.enum_mbus_fmt = file_input_enum_mbus_fmt,
	.try_mbus_fmt = file_input_g_mbus_fmt,
	.g_mbus_fmt = file_input_g_mbus_fmt,
	.s_mbus_fmt = file_input_s_mbus_fmt,
};

static const struct v4l2_subdev_core_ops file_input_core_ops = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0)
	.g_chip_ident = file_input_g_chip_ident,
#endif
	.log_status = file_input_log_status,
	.queryctrl = file_input_queryctrl,
	.g_ctrl = file_input_g_ctrl,
	.s_ctrl = file_input_s_ctrl,
	.s_power = file_input_s_power,
};

static const struct v4l2_subdev_pad_ops file_input_pad_ops = {
	.enum_mbus_code = file_input_enum_mbus_code,
	.enum_frame_size = file_input_enum_frame_size,
	.enum_frame_interval = file_input_enum_frame_ival,
};

static const struct v4l2_subdev_ops file_input_ops = {
	.core = &file_input_core_ops,
	.video = &file_input_video_ops,
	.pad = &file_input_pad_ops,
};

void
atomisp_file_input_unregister_entities(struct atomisp_file_device *file_dev)
{
	media_entity_cleanup(&file_dev->sd.entity);
	v4l2_device_unregister_subdev(&file_dev->sd);
}

int atomisp_file_input_register_entities(struct atomisp_file_device *file_dev,
			struct v4l2_device *vdev)
{
	/* Register the subdev and video nodes. */
	return  v4l2_device_register_subdev(vdev, &file_dev->sd);
}

void atomisp_file_input_cleanup(struct atomisp_device *isp)
{
	struct atomisp_file_device *file_dev = &isp->file_dev;

	if (file_dev->fWork_queued) {
		file_dev->fStreaming = false;
		complete(&file_dev->file_inject_start);
		cancel_work_sync(&file_dev->work_thread);
		file_dev->fWork_queued = false;
	}

	if (file_dev->work_queue) {
		destroy_workqueue(file_dev->work_queue);
		file_dev->work_queue = NULL;
	}

	return;
}

int atomisp_file_input_init(struct atomisp_device *isp)
{
	struct atomisp_file_device *file_dev = &isp->file_dev;
	struct v4l2_subdev *sd = &file_dev->sd;
	struct media_pad *pads = file_dev->pads;
	struct media_entity *me = &sd->entity;
	struct camera_mipi_info *file_input_info = NULL;
	int ret;

	file_dev->isp = isp;
	file_dev->work_queue = create_singlethread_workqueue(
				"ATOMISP file injection work queue");
	if (!file_dev->work_queue) {
		dev_warn(isp->dev,
			"Failed to create file inject work queue\n");
		return -ENOMEM;
	}
	INIT_WORK(&file_dev->work_thread, file_work);
	init_completion(&file_dev->file_inject_start);
	file_dev->fWork_queued = false;
	file_dev->fStreaming = false;
	file_dev->framecount = 0;
	v4l2_subdev_init(sd, &file_input_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	strcpy(sd->name, "file_input_subdev");
	v4l2_set_subdevdata(sd, file_dev);

	file_input_info = kzalloc(sizeof(*file_input_info), GFP_KERNEL);
	if (!file_input_info) {
		v4l2_err(&atomisp_dev,
			    "Failed to allocate memory for file input\n");
		return -ENOMEM;
	}

	file_input_info->port = ATOMISP_CAMERA_PORT_PRIMARY;
	/* the default input format is IA_CSS_STREAM_FORMAT_YUV422_8
	and no bayer order */
	v4l2_set_subdev_hostdata(sd, (void *)file_input_info);

	pads[0].flags = MEDIA_PAD_FL_SINK;
	me->type = MEDIA_ENT_T_V4L2_SUBDEV;

	ret = media_entity_init(me, 1, pads, 0);
	if (ret < 0)
		goto fail;
	return 0;
fail:
	kfree(file_input_info);
	atomisp_file_input_cleanup(isp);
	return ret;
}
