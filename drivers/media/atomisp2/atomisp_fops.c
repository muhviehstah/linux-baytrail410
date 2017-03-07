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

#include <linux/module.h>
#include <linux/pm_runtime.h>

#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#include "atomisp_cmd.h"
#include "atomisp_common.h"
#include "atomisp_compat.h"
#include "atomisp_fops.h"
#include "atomisp_internal.h"
#include "atomisp_ioctl.h"
#include "atomisp_subdev.h"
#include "atomisp-regs.h"

#include "hrt/hive_isp_css_mm_hrt.h"
#include "hrt/hive_isp_css_custom_host_hrt.h"

#include "ia_css_debug.h"
#include "host/mmu_local.h"
#include "device_access/device_access.h"
#include "memory_access/memory_access.h"

#include "atomisp_acc.h"

#define ISP_LEFT_PAD			128	/* equal to 2*NWAY */
#define CSS_DTRACE_VERBOSITY_LEVEL	5	/* Controls trace verbosity */

/*
 * input image data, and current frame resolution for test
 */
#define	ISP_PARAM_MMAP_OFFSET	0xfffff000

#define MAGIC_CHECK(is, should)	\
	if (unlikely((is) != (should))) { \
		printk(KERN_ERR "magic mismatch: %x (expected %x)\n", \
			is, should); \
		BUG(); \
	}



int atomisp_q_video_buffers_to_css(struct atomisp_sub_device *isp_subdev,
			     struct atomisp_video_pipe *pipe,
			     enum ia_css_buffer_type css_buf_type,
			     enum ia_css_pipe_id css_pipe_id, bool streamoff)
{
	struct atomisp_vb2 *vb2;
	unsigned long irqflags;
	int err, pipe_index;
	struct atomisp_device *isp = isp_subdev->isp;

	v4l2_dbg(4, dbg_level, &atomisp_dev, "ENTER q_video_buffers_to_css, pipe=%p, pipe_type=%d, activeq list empty=%s, buffers_in_css=%d\n", pipe, pipe->pipe_type, (list_empty(&pipe->activeq))?"true":"false", pipe->buffers_in_css);

	while (pipe->buffers_in_css < ATOMISP_CSS_Q_DEPTH) {
		struct ia_css_buffer css_buf;
		memset(&css_buf, 0, sizeof(struct ia_css_buffer));
		spin_lock_irqsave(&pipe->irq_lock, irqflags);
        if (list_empty(&pipe->activeq) || (!pipe->users)) {
	        if(list_empty(&pipe->activeq))
		        v4l2_dbg(4, dbg_level, &atomisp_dev, "atomisp_q_video_buffers_to_css: DMA activeq empty, exiting function\n");
		    else
			    v4l2_dbg(4, dbg_level, &atomisp_dev, "atomisp_q_video_buffers_to_css: pipe not in use, exiting function\n");
		spin_unlock_irqrestore(&pipe->irq_lock, irqflags);
		return -EINVAL;
		}
		vb2 = list_entry(pipe->activeq.next,
			struct atomisp_vb2, list);
		list_del_init(&vb2->list);
		spin_unlock_irqrestore(&pipe->irq_lock, irqflags);
		css_buf.type = css_buf_type;
		css_buf.data.frame = vb2->cssframe;
		v4l2_dbg(4, dbg_level, &atomisp_dev, "q_video_buffers_to_css: DMA delete from activeq vb2 vaddr=%X, fd=%d, index=%d\n", vb2->cssframe->data, vb2->vb.v4l2_buf.m.fd, vb2->vb.v4l2_buf.index);
		pipe_index = (unsigned int)(css_pipe_id);
		err = ia_css_pipe_enqueue_buffer(isp_subdev->css2_basis.pipes[pipe_index],
						 &css_buf);
		if (err) {
		v4l2_dbg(4, dbg_level, &atomisp_dev, "ERROR with ia_css_pipe_enqueue_buffer");
			spin_lock_irqsave(&pipe->irq_lock, irqflags);
			list_add_tail(&vb2->list, &pipe->activeq);
			spin_unlock_irqrestore(&pipe->irq_lock, irqflags);
			dev_err(isp->dev, "%s, css q fails: %d\n",
					__func__, err);
			return -EINVAL;
		}

		/*
		 * CSS2.0 Issue: after stream off, need to queue 1 buffer to CSS
		 * to recover SP.
		 * So if it is in stream off state, only need to queue 1 buffer
		 */

		if (streamoff)
			return 0;

		pipe->buffers_in_css++;
		v4l2_dbg(4, dbg_level, &atomisp_dev, "Buffers in css incremented to %d\n", pipe->buffers_in_css);
		vb2 = NULL;
	}
	return 0;
}

int atomisp_q_s3a_buffers_to_css(struct atomisp_sub_device *isp_subdev,
			   enum ia_css_pipe_id css_pipe_id, bool streamoff)
{
	struct atomisp_s3a_buf *s3a_buf;
	int pipe_index = 0;
	struct atomisp_device *isp = isp_subdev->isp;

	if (list_empty(&isp_subdev->s3a_stats)) {
		WARN(1, "%s: No s3a buffers available!\n", __func__);
		return -EINVAL;
	}

	while (isp_subdev->s3a_bufs_in_css[css_pipe_id] < ATOMISP_CSS_Q_DEPTH) {
		struct ia_css_buffer buffer;
		memset(&buffer, 0, sizeof(struct ia_css_buffer));
		s3a_buf = list_entry(isp_subdev->s3a_stats.next,
				struct atomisp_s3a_buf, list);
		list_move_tail(&s3a_buf->list, &isp_subdev->s3a_stats);

		buffer.type = IA_CSS_BUFFER_TYPE_3A_STATISTICS;
		buffer.data.stats_3a = s3a_buf->s3a_stat;
		pipe_index = (unsigned int)css_pipe_id;
		if (ia_css_pipe_enqueue_buffer(
					isp_subdev->css2_basis.pipes[pipe_index],
					&buffer)) {
			dev_err(isp->dev, "failed to q s3a stat buffer\n");
			return -EINVAL;
		}

		/*
		 * CSS2.0 Issue: after stream off, need to queue 1 buffer to CSS
		 * to recover SP.
		 * So if it is in stream off state, only need to queue 1 buffer
		 */

		if (streamoff)
			return 0;

		isp_subdev->s3a_bufs_in_css[css_pipe_id]++;
	}
	return 0;
}

int atomisp_q_dis_buffers_to_css(struct atomisp_sub_device *isp_subdev,
			   enum ia_css_pipe_id css_pipe_id, bool streamoff)
{
	struct atomisp_device *isp = isp_subdev->isp;
	struct atomisp_dvs_buf *dvs_buf;
	int pipe_index = 0;

	if (list_empty(&isp_subdev->dvs_stats)) {
		WARN(1, "%s: No dis buffers available!\n", __func__);
		return -EINVAL;
	}

	while (isp_subdev->dis_bufs_in_css < ATOMISP_CSS_Q_DEPTH) {
		struct ia_css_buffer buffer = {0};
		dvs_buf = list_entry(isp_subdev->dvs_stats.next,
				struct atomisp_dvs_buf, list);
		list_move_tail(&dvs_buf->list, &isp_subdev->dvs_stats);

		buffer.type = IA_CSS_BUFFER_TYPE_DIS_STATISTICS;
		buffer.data.stats_dvs = dvs_buf->dvs_stat;
		pipe_index = (unsigned int )(css_pipe_id);
		if (ia_css_pipe_enqueue_buffer(isp_subdev->css2_basis.pipes[pipe_index],
					&buffer)) {
			dev_err(isp->dev, "failed to q dvs stat buffer\n");
			return -EINVAL;
		}
		/*
		 * CSS2.0 Issue: after stream off, need to queue 1 buffer to CSS
		 * to recover SP.
		 * So if it is in stream off state, only need to queue 1 buffer
		 */
		if (streamoff)
			return 0;

		isp_subdev->dis_bufs_in_css++;
		dvs_buf = NULL;
	}
	return 0;
}

/* queue all available buffers to css */
int atomisp_qbuffers_to_css(struct atomisp_sub_device *isp_subdev, bool streamoff)
{
	enum ia_css_buffer_type buf_type;
	enum ia_css_pipe_id css_capture_pipe_id = IA_CSS_PIPE_ID_NUM;
	enum ia_css_pipe_id css_preview_pipe_id = IA_CSS_PIPE_ID_NUM;
	struct atomisp_video_pipe *capture_pipe = NULL;
	struct atomisp_video_pipe *vf_pipe = NULL;
	struct atomisp_video_pipe *preview_pipe = NULL;

	if (!isp_subdev->enable_vfpp->val) {
		preview_pipe = &isp_subdev->video_out_capture;
		css_preview_pipe_id = IA_CSS_PIPE_ID_VIDEO;
	} else if (isp_subdev->run_mode->val == ATOMISP_RUN_MODE_VIDEO) {
		if(isp_subdev->video_pipe_vf_enable)
		     vf_pipe =  &isp_subdev->video_out_vf;
		capture_pipe = &isp_subdev->video_out_capture;
		preview_pipe = &isp_subdev->video_out_preview;
		css_capture_pipe_id = IA_CSS_PIPE_ID_VIDEO;
		css_preview_pipe_id = IA_CSS_PIPE_ID_VIDEO;
	} else if (isp_subdev->params.continuous_vf) {
		capture_pipe = &isp_subdev->video_out_capture;
		vf_pipe = &isp_subdev->video_out_vf;
		preview_pipe = &isp_subdev->video_out_preview;
		css_capture_pipe_id = IA_CSS_PIPE_ID_CAPTURE;
		css_preview_pipe_id = IA_CSS_PIPE_ID_PREVIEW;
	} else if (isp_subdev->run_mode->val == ATOMISP_RUN_MODE_PREVIEW) {
		preview_pipe = &isp_subdev->video_out_preview;
		css_preview_pipe_id = IA_CSS_PIPE_ID_PREVIEW;
	} else {
		/* ATOMISP_RUN_MODE_STILL_CAPTURE */
		capture_pipe = &isp_subdev->video_out_capture;
		if (!atomisp_is_mbuscode_raw(isp_subdev->
			    fmt[isp_subdev->capture_pad].fmt.code))
			vf_pipe = &isp_subdev->video_out_vf;
		else
			vf_pipe = NULL; // Making vf_pipe = NULL since video_mode initialize it & for next capture_mode run it cause kernel panic
		css_capture_pipe_id = IA_CSS_PIPE_ID_CAPTURE;
	}

	if (capture_pipe) {
		buf_type = atomisp_get_css_buf_type(isp_subdev, capture_pipe);
		atomisp_q_video_buffers_to_css(isp_subdev, capture_pipe, buf_type,
					 css_capture_pipe_id, streamoff);
	}

	if (vf_pipe) {
		buf_type = atomisp_get_css_buf_type(isp_subdev, vf_pipe);
		atomisp_q_video_buffers_to_css(isp_subdev, vf_pipe, buf_type,
					 css_capture_pipe_id, streamoff);
	}

	if (preview_pipe) {
		buf_type = atomisp_get_css_buf_type(isp_subdev, preview_pipe);
		atomisp_q_video_buffers_to_css(isp_subdev, preview_pipe, buf_type,
					 css_preview_pipe_id, streamoff);
	}

	//Currently allowing only video mode to q stats buff.
	//TODO: Enable stat bufs queuing if capture/preview mode supports dis & s3a
	if(isp_subdev->run_mode->val == ATOMISP_RUN_MODE_VIDEO){
	if (isp_subdev->params.curr_grid_info.s3a_grid.enable) {
		if (css_capture_pipe_id < IA_CSS_PIPE_ID_NUM)
			atomisp_q_s3a_buffers_to_css(isp_subdev, css_capture_pipe_id, streamoff);
		if (css_preview_pipe_id < IA_CSS_PIPE_ID_NUM && css_preview_pipe_id < IA_CSS_PIPE_ID_NUM)
			atomisp_q_s3a_buffers_to_css(isp_subdev, css_preview_pipe_id, streamoff);
	}

	if (isp_subdev->params.curr_grid_info.dvs_grid.enable)
		atomisp_q_dis_buffers_to_css(isp_subdev, css_capture_pipe_id, streamoff);
	}
	return 0;
}

static int atomisp_vb2_queue_setup(struct vb2_queue *vq, const struct v4l2_format *fmt,
	unsigned int *nbuffers, unsigned int *nplanes,
	unsigned int sizes[], void *alloc_ctxs[]) {
	struct video_device *vdev = vb2_get_drv_priv(vq);
  	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	unsigned long size;

	if (fmt)
		size = fmt->fmt.pix.sizeimage;
	else {
		size = pipe->pix.sizeimage;
	}
	v4l2_dbg(6, dbg_level, &atomisp_dev, "atomisp_vb2_queue_setup: size=%lu\n", size);
	trace_printk("atomisp_vb2_queue_setup: size=%lu\n", size);

  if (size == 0)
    return -EINVAL;

  if (0 == *nbuffers) {
		v4l2_dbg(4, dbg_level, &atomisp_dev, "WARNING in atomisp_vb2_queue_setup, nbuffers == 0. Setting to 3\n");
    *nbuffers = 3;
	}

  *nplanes = 1;
  sizes[0] = size;

	isp->raw_dmamask = DMA_BIT_MASK(36); /* 36 is ISP IUNIT MMU limit */
	vdev->dev.dma_mask = &isp->raw_dmamask;
	vdev->dev.coherent_dma_mask = isp->raw_dmamask;

	alloc_ctxs[0] = vb2_dma_contig_init_ctx(&vdev->dev);
	if (!alloc_ctxs[0]) {
		v4l2_err(&atomisp_dev, "Error in atomisp_vb2_queue_setup allocating alloc_ctx\n");
		return -EINVAL;
	}

	return 0;
}

/* This gets called from vb2_qbuf if already streaming, and from vb2_streamon*/
static void atomisp_vb2_buf_queue(struct vb2_buffer *vb) {
	struct vb2_queue *q = vb->vb2_queue;
	struct video_device *vdev = vb2_get_drv_priv(q);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	struct atomisp_vb2 *buf = container_of(vb, struct atomisp_vb2, vb);

	v4l2_dbg(6, dbg_level, &atomisp_dev, "DMA atomisp_vb2_buf_queue, adding buf %d to activeq\n", vb->v4l2_buf.index);

	list_add_tail(&buf->list, &pipe->activeq);

}
static int atomisp_vb2_buf_prepare(struct vb2_buffer *vb) {
  struct video_device *vdev = vb2_get_drv_priv(vb->vb2_queue);
  struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	unsigned long size;

	size = pipe->pix.sizeimage;

	vb2_set_plane_payload(vb, 0, size);
	//TODO handle more planes

	return 0;
}

static void atomisp_vb2_unlock(struct vb2_queue *vq) {
	struct video_device *vdev = vb2_get_drv_priv(vq);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	mutex_unlock(&isp->vb2mutex);
}

static void atomisp_vb2_lock(struct vb2_queue *vq) {
	struct video_device *vdev = vb2_get_drv_priv(vq);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	mutex_lock(&isp->vb2mutex);
}

static int atomisp_vb2_start_streaming(struct vb2_queue *vq, unsigned int count) {
/*
	called once to enter 'streaming' state; the driver may
  receive buffers with @buf_queue callback before
  @start_streaming is called; the driver gets the number
  of already queued buffers in count parameter; driver
  can return an error if hardware fails or not enough
  buffers has been queued, in such case all buffers that
  have been already given by the @buf_queue callback are
  invalidated.
*/
	v4l2_dbg(6, dbg_level, &atomisp_dev, "ENTER atomisp_vb2_start_streaming, queued buffer count=%d\n", count);
	return 0;
}

static struct vb2_ops vb2_qops = {
  .queue_setup    = atomisp_vb2_queue_setup, //required
	.buf_prepare    = atomisp_vb2_buf_prepare,
  .buf_queue      = atomisp_vb2_buf_queue, //required
  .start_streaming = atomisp_vb2_start_streaming, //Called after streaming enabled in streamon
  //.stop_streaming = atomisp_vb2_stop_streaming,
  .wait_prepare   = atomisp_vb2_unlock,
  .wait_finish    = atomisp_vb2_lock,
};

static int atomisp_vb2_setup(struct video_device *vdev) {
	int rc;
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	struct vb2_queue *q, *outq;
	q = &pipe->vb2q;
	memset(q, 0, sizeof(struct vb2_queue));
  	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  	q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
  	q->drv_priv = vdev;
	q->buf_struct_size = sizeof(struct atomisp_vb2);
 	q->ops = &vb2_qops;
	q->mem_ops = &vb2_dma_contig_memops;

  /* We currently use the monotonic clock for both input and output buffers
   * (for details, see atomisp_cmd.c:atomisp_buf_done).
   */
#if LINUX_VERSION_CODE > KERNEL_VERSION(3,8,0)
	q->timestamp_type = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
#endif

  rc = vb2_queue_init(q);
  if (rc < 0)
    return rc;
	vdev->queue = q;
	INIT_LIST_HEAD(&pipe->activeq);

	/* Just for file injection */
	outq = &pipe->vb2outq;
	memset(outq, 0, sizeof(struct vb2_queue));
	outq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	outq->io_modes = VB2_MMAP;
	outq->drv_priv = vdev;
#if LINUX_VERSION_CODE > KERNEL_VERSION(3,8,0)
        outq->timestamp_type = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
#endif
	outq->buf_struct_size = sizeof(struct atomisp_vb2);
	outq->ops = &vb2_qops;
	outq->mem_ops = &vb2_dma_contig_memops;
	rc = vb2_queue_init(outq);
	if (rc < 0)
		return rc;
	INIT_LIST_HEAD(&pipe->activeq_out);

	return 0;
}

int atomisp_dev_init_struct(struct atomisp_device *isp)
{
	int i = 0;
	if (isp == NULL)
		return -EINVAL;

	isp->sw_contex.file_input = 0;
	isp->need_gfx_throttle = true;
	isp->isp_fatal_error = false;

	for(i=0;i<ATOM_ISP_MAX_INPUTS;i++)
		isp->inputs[i].used_by = -1;

	/*
	 * For Merrifield, frequency is scalable.
	 * After boot-up, the default frequency is 200MHz.
	 * For Medfield/Clovertrail, all running at 320MHz
	 */
	if (IS_MRFLD)
		isp->sw_contex.running_freq = ISP_FREQ_200MHZ;
	else
		isp->sw_contex.running_freq = ISP_FREQ_320MHZ;

	return 0;
}

int atomisp_subdev_init_struct(struct atomisp_sub_device *isp_subdev)
{
	int i = 0;

	v4l2_ctrl_s_ctrl(isp_subdev->run_mode,
			 ATOMISP_RUN_MODE_STILL_CAPTURE);
	isp_subdev->params.color_effect = V4L2_COLORFX_NONE;
	isp_subdev->params.bad_pixel_en = 1;
	isp_subdev->params.gdc_cac_en = 0;
	isp_subdev->params.video_dis_en = 0;
	isp_subdev->params.sc_en = 0;
	isp_subdev->params.fpn_en = 0;
	isp_subdev->params.xnr_en = 0;
	isp_subdev->params.false_color = 0;
	isp_subdev->params.online_process = 1;
	isp_subdev->params.yuv_ds_en = 0;
	isp_subdev->params.offline_parm.num_captures = 1;
	isp_subdev->params.offline_parm.skip_frames = 0;
	isp_subdev->params.offline_parm.offset = 0;
	isp_subdev->params.continuous_vf = false;
	isp_subdev->delayed_init = ATOMISP_DELAYED_INIT_NOT_QUEUED;

	isp_subdev->css2_basis.stream = NULL;
	for (i = 0; i < IA_CSS_PIPE_MODE_NUM; i++) {
		isp_subdev->css2_basis.pipes[i] = NULL;
		ia_css_pipe_config_defaults(&isp_subdev->css2_basis.pipe_configs[i]);
		ia_css_pipe_extra_config_defaults(
				&isp_subdev->css2_basis.pipe_extra_configs[i]);
	}
	ia_css_stream_config_defaults(&isp_subdev->css2_basis.stream_config);
	isp_subdev->css2_basis.curr_pipe = 0;
	isp_subdev->video_pipe_vf_enable = false;
	isp_subdev->stream_prepared = false;
	/* Add for channel */
	if (isp_subdev->isp->inputs[0].camera)
		isp_subdev->input_curr = 0;

	init_completion(&isp_subdev->buf_done);

	return 0;
}

static void *my_kernel_malloc(size_t bytes, bool zero_mem)
{
	void *ptr = atomisp_kernel_malloc(bytes);

	if (ptr && zero_mem)
		memset(ptr, 0, bytes);

	return ptr;
}

/*
 * Return the number of concurrent running streams.
 */
int atomisp_subdev_streaming_count(struct atomisp_device *isp)
{
	int i, sum;

	for (i = 0, sum = 0; i < isp->num_of_streams; i++)
		sum += isp->isp_subdev[i].streaming ==
		    ATOMISP_DEVICE_STREAMING_ENABLED;

	return sum;
}

/* SOF IRQ enable only in single stream mode */
void atomisp_control_irq_sof(struct atomisp_device *isp)
{
	if (atomisp_subdev_streaming_count(isp) == 1)
		ia_css_irq_enable(IA_CSS_IRQ_INFO_CSS_RECEIVER_SOF, true);
	else
		ia_css_irq_enable(IA_CSS_IRQ_INFO_CSS_RECEIVER_SOF, false);
}

/*
 * file operation functions
 */
unsigned int atomisp_subdev_users(struct atomisp_sub_device *isp_subdev)
{

	int no_of_users = isp_subdev->video_out_preview.users +
	       isp_subdev->video_out_vf.users +
	       isp_subdev->video_out_capture.users +
	       isp_subdev->video_in.users;
	return no_of_users;
}

unsigned int atomisp_dev_users(struct atomisp_device *isp)
{
	unsigned int i, sum = 0;

	for (i = 0; i < isp->num_of_streams; i++)
		sum += atomisp_subdev_users(&isp->isp_subdev[i]);

	return sum;
}

static int atomisp_css2_dbg_print(const char *fmt, va_list args)
{
	if (dbg_level > 5)
		vprintk(fmt, args);
	return 0;
}

static int atomisp_css2_err_print(const char *fmt, va_list args)
{
	vprintk(fmt, args);
	return 0;
}

struct ia_css_env css_env = {
                .cpu_mem_env.alloc = my_kernel_malloc,
                .cpu_mem_env.free = atomisp_kernel_free,
                .css_mem_env.alloc = atomisp_css2_mm_alloc,
                .css_mem_env.free = atomisp_css2_mm_free,
                .css_mem_env.load = atomisp_css2_mm_load,
                .css_mem_env.store = atomisp_css2_mm_store,
                .css_mem_env.set = atomisp_css2_mm_set,
                .css_mem_env.mmap = atomisp_css2_mm_mmap,

                .hw_access_env.store_8 = atomisp_css2_hw_store_8,
                .hw_access_env.store_16 = atomisp_css2_hw_store_16,
                .hw_access_env.store_32 = atomisp_css2_hw_store_32,

                .hw_access_env.load_8 = atomisp_css2_hw_load_8,
                .hw_access_env.load_16 = atomisp_css2_hw_load_16,
                .hw_access_env.load_32 = atomisp_css2_hw_load_32,

                .hw_access_env.load = atomisp_css2_hw_load,
                .hw_access_env.store = atomisp_css2_hw_store,

                .print_env.debug_print = atomisp_css2_dbg_print,
                .print_env.error_print = atomisp_css2_err_print,
};

static int atomisp_open(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	struct atomisp_sub_device *isp_subdev = atomisp_to_sub_device(pipe);
	int ret = -EINVAL;

	v4l2_dbg(3, dbg_level, &atomisp_dev, ">%s [%d]\n",
				__func__, pipe->pipe_type);

	//following conditions gets executed if only one sub_dev is registered
	if(isp->detected_snr_cnt==1)
			if(isp_subdev!=&isp->isp_subdev[0])
				return 0;
if (fastboot){

	if(atomisp_subdev_streaming_count(isp)>=1 && isp->firmware_switched == false){
        	return ret;
        }
}

	mutex_lock(&isp->mutex);

	if (!isp->input_cnt) {
		v4l2_err(&atomisp_dev, "no camera attached\n");
		ret = -EINVAL;
		goto error;
	}

	if (pipe->users)
		goto done;

	atomisp_vb2_setup(vdev);

	if (atomisp_subdev_users(isp_subdev)) {
		goto done;
	}

	atomisp_subdev_init_struct(isp_subdev);

	if(atomisp_dev_users(isp))
		goto done;

	hrt_isp_css_mm_init();
	isp->mmu_base_addr = hrt_isp_get_mmu_base_address();
	if (isp->mmu_base_addr < 0) {
		hrt_isp_css_mm_clear();
		goto error;
	}

	/* runtime power management, turn on ISP */
	ret = pm_runtime_get_sync(vdev->v4l2_dev->dev);
	if (ret < 0) {
		v4l2_err(&atomisp_dev,
				"Failed to power on device\n");
		goto error;
	}

	ret = atomisp_css_init(isp);
	if(ret < 0)
		goto css_init_failed;
	/*uncomment the following to lines to enable offline preview*/
	/*sh_css_enable_continuous(true);*/
	/*sh_css_preview_enable_online(false);*/
	atomisp_dev_init_struct(isp);

done:
	pipe->users++;
	mutex_unlock(&isp->mutex);
	dev_warn(isp->dev, "atomisp_open: successfully opened video%d\n", pipe->pipe_type);
	return 0;

css_init_failed:
	dev_err(isp->dev, "css init failed --- bad firmware?\n");
error:
	pm_runtime_put(vdev->v4l2_dev->dev);
	mutex_unlock(&isp->mutex);
	return ret;
}

static int atomisp_release(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	struct atomisp_sub_device *isp_subdev = atomisp_to_sub_device(pipe);
	struct v4l2_mbus_framefmt isp_sink_fmt;
	struct v4l2_requestbuffers req;
	int ret = 0;
	if (isp == NULL)
		return -EBADF;
	v4l2_dbg(5, dbg_level, &atomisp_dev, ">%s [%d]\n", __func__, pipe->pipe_type);

	req.count = 0;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = vdev->queue->memory;
	mutex_lock(&isp->streamoff_mutex);
	mutex_lock(&isp->mutex);
	pipe->users--;

	if (pipe->users)
		goto done;

	/*When release is called before streaming begins, the allocated frames need to be freed*/
	atomisp_free_css_frames(&pipe->vb2q);

	if (pipe->vb2q.streaming && __atomisp_streamoff(file, NULL, V4L2_BUF_TYPE_VIDEO_CAPTURE))
	{
		dev_err(isp->dev,
			"atomisp_streamoff failed on release, driver bug");
		goto done;
	}

	vb2_fop_release(file);
	vb2_queue_release(&pipe->vb2outq);

	/*
	 * A little trick here:
	 * file injection input resolution is recorded in the sink pad,
	 * therefore can not be cleared when releaseing one device node.
	 * The sink pad setting can only be cleared when all device nodes
	 * get released.
	 */
	if (!isp->sw_contex.file_input) {
		memset(&isp_sink_fmt, 0, sizeof(isp_sink_fmt));
		atomisp_subdev_set_ffmt(isp_subdev, NULL,
				V4L2_SUBDEV_FORMAT_ACTIVE,
				ATOMISP_SUBDEV_PAD_SINK, &isp_sink_fmt);
	}

	if (atomisp_subdev_users(isp_subdev))
	{
		goto done;
	}

	/* clear the sink pad for file input */
	if (isp->sw_contex.file_input) {
		memset(&isp_sink_fmt, 0, sizeof(isp_sink_fmt));
		atomisp_subdev_set_ffmt(isp_subdev, NULL,
				V4L2_SUBDEV_FORMAT_ACTIVE,
				ATOMISP_SUBDEV_PAD_SINK, &isp_sink_fmt);
	}

	atomisp_free_3a_dvs_buffers(isp_subdev);
	atomisp_free_internal_buffers(isp_subdev);

	/*uninit the camera subdev*/
	ret = v4l2_subdev_call(isp->inputs[isp_subdev->input_curr].camera,
			       core, init, 0);
	if (ret == -1 || ret == -EINVAL)
		v4l2_err(&atomisp_dev, "sensor firmware failed\n");

	ret = v4l2_subdev_call(isp->inputs[isp_subdev->input_curr].camera,
				       core, s_power, 0);
	if (ret)
		v4l2_warn(&atomisp_dev, "Failed to power-off sensor\n");

	if(atomisp_dev_users(isp))
	{
		goto done;
	}


	del_timer_sync(&isp->wdt);
	atomisp_acc_release(isp);
	atomisp_free_all_shading_tables(isp);

	atomisp_destroy_pipes_stream_force(isp_subdev);
	atomisp_css_uninit(isp);


	if (pm_runtime_put_sync(vdev->v4l2_dev->dev) < 0)
		v4l2_dbg(3, dbg_level, &atomisp_dev, "Failed to power off device\n");
done:
	mutex_unlock(&isp->mutex);
	mutex_unlock(&isp->streamoff_mutex);
	return 0;
}

/*
 * Memory help functions for image frame and private parameters
 */
static int do_isp_mm_remap(struct vm_area_struct *vma,
		ia_css_ptr isp_virt, u32 host_virt, u32 pgnr)
{
	u32 pfn;

	while (pgnr) {
		pfn = hmm_virt_to_phys(isp_virt) >> PAGE_SHIFT;
		if (remap_pfn_range(vma, host_virt, pfn,
				    PAGE_SIZE, PAGE_SHARED)) {
			v4l2_err(&atomisp_dev,
				    "remap_pfn_range err.\n");
			return -EAGAIN;
		}

		isp_virt += PAGE_SIZE;
		host_virt += PAGE_SIZE;
		pgnr--;
	}

	return 0;
}

static int frame_mmap(const struct ia_css_frame *frame,
	struct vm_area_struct *vma)
{
	ia_css_ptr isp_virt;
	u32 host_virt;
	u32 pgnr;

	if (!frame) {
		v4l2_err(&atomisp_dev,
			    "%s: NULL frame pointer.\n", __func__);
		return -EINVAL;
	}

	host_virt = vma->vm_start;
	isp_virt = frame->data;
	atomisp_get_frame_pgnr(frame, &pgnr);

	if (do_isp_mm_remap(vma, isp_virt, host_virt, pgnr))
		return -EAGAIN;

	return 0;
}

static int atomisp_vb2_mmap_mapper(struct vb2_queue *q,
	struct vm_area_struct *vma)
{
	u32 offset = vma->vm_pgoff << PAGE_SHIFT;
	int ret = -EINVAL, i;
	struct vb2_buffer *vb2;
	struct atomisp_vb2 *avb2;
	v4l2_dbg(5, dbg_level, &atomisp_dev, ">%s.\n", __func__);

	for (i = 0; i < VIDEO_MAX_FRAME; i++) {
		vb2 = q->bufs[i];
		if (vb2 && vb2->v4l2_buf.memory == V4L2_MEMORY_MMAP
		     && vb2->v4l2_planes[0].m.mem_offset == offset)
		{
			avb2 = container_of(vb2, struct atomisp_vb2, vb);
			ret = frame_mmap(avb2->cssframe, vma);
			vma->vm_flags |= VM_DONTEXPAND | VM_DONTDUMP;
			break;
		}
	}
	v4l2_dbg(5, dbg_level, &atomisp_dev, "<%s.\n", __func__);
	return ret;
}

/* The input frame contains left and right padding that need to be removed.
 * There is always ISP_LEFT_PAD padding on the left side.
 * There is also padding on the right (padded_width - width).
 */
static int
remove_pad_from_frame(struct ia_css_frame *in_frame, __u32 width, __u32 height)
{
	unsigned int i;
	unsigned short *buffer;
	int ret = 0;
	ia_css_ptr load = in_frame->data;
	ia_css_ptr store = load;

	buffer = kmalloc(width*sizeof(load), GFP_KERNEL);
	if (!buffer) {
		v4l2_err(&atomisp_dev, "out of memory.\n");
		return -ENOMEM;
	}

	load += ISP_LEFT_PAD;
	for (i = 0; i < height; i++) {
		ret = hrt_isp_css_mm_load(load, buffer, width*sizeof(load));
		if (ret < 0)
			goto remove_pad_error;

		ret = hrt_isp_css_mm_store(store, buffer, width*sizeof(store));
		if (ret < 0)
			goto remove_pad_error;

		load  += in_frame->info.padded_width;
		store += width;
	}

remove_pad_error:
	kfree(buffer);
	return ret;
}

static int atomisp_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	struct atomisp_sub_device *isp_subdev = atomisp_to_sub_device(pipe);
	struct ia_css_frame *raw_virt_addr;
	u32 start = vma->vm_start;
	u32 end = vma->vm_end;
	u32 size = end - start;
	u32 origin_size, new_size;
	int ret;

	if (!(vma->vm_flags & (VM_WRITE | VM_READ)))
		return -EACCES;

	if (!(vma->vm_flags & VM_SHARED))
		return -EINVAL;

	mutex_lock(&isp->mutex);
	new_size = pipe->pix.width * pipe->pix.height * 2;

	/* mmap for ISP offline raw data */
	if ((pipe->pipe_type == ATOMISP_PIPE_CAPTURE) &&
	    (vma->vm_pgoff == (ISP_PARAM_MMAP_OFFSET >> PAGE_SHIFT))) {
		v4l2_dbg(6, dbg_level, &atomisp_dev, "vma->vm_pgoff == (ISP_PARAM_MMAP_OFFSET >> PAGE_SHIFT)\n");
		if (isp_subdev->params.online_process != 0) {
			ret = -EINVAL;
			goto error;
		}
		raw_virt_addr = isp_subdev->raw_output_frame;
		if (raw_virt_addr == NULL) {
			v4l2_err(&atomisp_dev,
				 "Failed to request RAW frame\n");
			ret = -EINVAL;
			goto error;
		}

		ret = remove_pad_from_frame(raw_virt_addr,
				      pipe->pix.width,
				      pipe->pix.height);
		if (ret < 0) {
			v4l2_err(&atomisp_dev, "remove pad failed.\n");
			goto error;
		}
		origin_size = raw_virt_addr->data_bytes;
		raw_virt_addr->data_bytes = new_size;

		if (size != PAGE_ALIGN(new_size)) {
			v4l2_err(&atomisp_dev,
				 "incorrect size for mmap ISP"
				 " Raw Frame\n");
			ret = -EINVAL;
			goto error;
		}

		if (frame_mmap(raw_virt_addr, vma)) {
			v4l2_err(&atomisp_dev,
				 "frame_mmap failed.\n");
			raw_virt_addr->data_bytes = origin_size;
			ret = -EAGAIN;
			goto error;
		}
		raw_virt_addr->data_bytes = origin_size;
		vma->vm_flags |= (VM_DONTEXPAND | VM_DONTDUMP);
		mutex_unlock(&isp->mutex);
		return 0;
	}

	/*
	 * mmap for normal frames
	 */
	if (size != pipe->pix.sizeimage) {
		v4l2_err(&atomisp_dev,
			    "incorrect size for mmap ISP frames\n");
		ret = -EINVAL;
		goto error;
	}
	mutex_unlock(&isp->mutex);

  return atomisp_vb2_mmap_mapper(&pipe->vb2q, vma);

error:
	mutex_unlock(&isp->mutex);

	return ret;
}

int atomisp_file_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	return vb2_mmap(&pipe->vb2outq, vma);
}

static unsigned int
atomisp_poll(struct file *file, struct poll_table_struct *pt)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	v4l2_dbg(4, dbg_level, &atomisp_dev, "inside atomisp_poll before mutex lock\n");
	mutex_lock(&isp->mutex);
	if (pipe->vb2q.streaming != 1) {
		mutex_unlock(&isp->mutex);
		v4l2_dbg(4, dbg_level, &atomisp_dev,  KERN_ERR "pipe is not streaming. unlocking mutex\n");
		return POLLERR;
	}
	mutex_unlock(&isp->mutex);
	v4l2_dbg(4, dbg_level, &atomisp_dev, "device is streaming so unlock mutex\n");
	return vb2_fop_poll(file, pt);
}

const struct v4l2_file_operations atomisp_fops = {
	.owner = THIS_MODULE,
	.open = atomisp_open,
	.release = atomisp_release,
	.mmap = atomisp_mmap,
	.unlocked_ioctl = video_ioctl2,
	.poll = atomisp_poll,
};

const struct v4l2_file_operations atomisp_file_fops = {
	.owner = THIS_MODULE,
	.open = atomisp_open,
	.release = atomisp_release,
	.mmap = atomisp_file_mmap,
	.ioctl = video_ioctl2,
	.poll = atomisp_poll,
};

