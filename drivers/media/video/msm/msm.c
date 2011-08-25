/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
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

#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/ioctl.h>
#include <linux/spinlock.h>
#include <linux/proc_fs.h>
#include "msm.h"


#define MSM_V4L2_PROC_NAME		 "msm_v4l2"

#define MSM_MAX_CAMERA_SENSORS 5
#define MSM_CDEV_PER_SENSOR 1

#ifdef CONFIG_MSM_CAMERA_DEBUG
#define D(fmt, args...) printk(KERN_DEBUG "msm: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif

static unsigned msm_camera_v4l2_nr = -1;
static unsigned number_of_sensors;
module_param(msm_camera_v4l2_nr, uint, 0644);
MODULE_PARM_DESC(msm_camera_v4l2_nr, "videoX start number, -1 is autodetect");

static void msm_queue_init(struct msm_device_queue *queue, const char *name)
{
	D("%s\n", __func__);
	spin_lock_init(&queue->lock);
	queue->len = 0;
	queue->max = 0;
	queue->name = name;
	INIT_LIST_HEAD(&queue->list);
	init_waitqueue_head(&queue->wait);
}

static void msm_enqueue(struct msm_device_queue *queue,
			struct list_head *entry)
{
	unsigned long flags;
	spin_lock_irqsave(&queue->lock, flags);
	queue->len++;
	if (queue->len > queue->max) {
		queue->max = queue->len;
		pr_info("%s: queue %s new max is %d\n", __func__,
			queue->name, queue->max);
	}
	list_add_tail(entry, &queue->list);
	wake_up(&queue->wait);
	D("%s: woke up %s\n", __func__, queue->name);
	spin_unlock_irqrestore(&queue->lock, flags);
}

static int msm_ctrl_cmd_done(struct msm_cam_v4l2_device *ctrl_pmsm,
						void __user *arg)
{
	struct msm_ctrl_cmd command;
	struct msm_queue_cmd *qcmd;
	if (copy_from_user(&command, arg, sizeof(command)))
		return -EINVAL;

	qcmd = kzalloc(sizeof(struct msm_queue_cmd), GFP_KERNEL);
	atomic_set(&qcmd->on_heap, 0);
	msm_enqueue(&ctrl_pmsm->ctrl_q, &qcmd->list_control);
	return 0;
}

/*
 *
 * implementation of v4l2_ioctl_ops
 *
 */
static int msm_camera_v4l2_querycap(struct file *f, void *pctx,
				struct v4l2_capability *pcaps)
{
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	/* some other day, some other time */
	/*cap->version = LINUX_VERSION_CODE; */
	strlcpy(pcaps->driver, pcam->pdev->name, sizeof(pcaps->driver));
	pcaps->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	return 0;
}

static int msm_camera_v4l2_queryctrl(struct file *f, void *pctx,
				struct v4l2_queryctrl *pqctrl)
{
	int rc = 0;

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	return rc;
}

static int msm_camera_v4l2_g_ctrl(struct file *f, void *pctx,
					struct v4l2_control *c)
{
	int rc = 0;

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	return rc;
}

static int msm_camera_v4l2_s_ctrl(struct file *f, void *pctx,
					struct v4l2_control *c)
{
	int rc = 0;

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	return rc;
}

static int msm_camera_v4l2_reqbufs(struct file *f, void *pctx,
				struct v4l2_requestbuffers *pb)
{
	int rc = 0;
	int i = 0;
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	rc = videobuf_reqbufs(&pcam->vid_bufq, pb);
	if (rc < 0)
		return rc;

	/* Now initialize the local msm_frame_buffer structure */
	for (i = 0; i < pb->count; i++) {
		struct msm_frame_buffer *buf = container_of(
					pcam->vid_bufq.bufs[i],
					struct msm_frame_buffer,
					vidbuf);
		buf->inuse = 0;
		INIT_LIST_HEAD(&buf->vidbuf.queue);
	}
	return 0;
}

static int msm_camera_v4l2_querybuf(struct file *f, void *pctx,
					struct v4l2_buffer *pb)
{
	/* get the video device */
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	return videobuf_querybuf(&pcam->vid_bufq, pb);
}

static int msm_camera_v4l2_qbuf(struct file *f, void *pctx,
					struct v4l2_buffer *pb)
{
	int rc = 0;
	/* get the camera device */
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	rc = videobuf_qbuf(&pcam->vid_bufq, pb);
	D("%s, videobuf_qbuf returns %d\n", __func__, rc);

	return rc;
}

static int msm_camera_v4l2_dqbuf(struct file *f, void *pctx,
					struct v4l2_buffer *pb)
{
	int rc = 0;
	/* get the camera device */
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	rc = videobuf_dqbuf(&pcam->vid_bufq, pb, f->f_flags & O_NONBLOCK);
	D("%s, videobuf_dqbuf returns %d\n", __func__, rc);

	return rc;
}

static int msm_camera_v4l2_streamon(struct file *f, void *pctx,
					enum v4l2_buf_type i)
{
	int rc = 0;
	struct videobuf_buffer *buf;
	int cnt = 0;
	/* get the camera device */
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	/* turn HW (VFE/sensor) streaming */
	rc = msm_isp_streamon(pcam);
	D("%s rc = %d\n", __func__, rc);
	if (rc < 0)
		D("%s: hw failed to start streaming\n", __func__);

	list_for_each_entry(buf, &pcam->vid_bufq.stream, stream) {
		D("%s index %d, state %d\n", __func__, cnt, buf->state);
		cnt++;
	}

	D("%s Calling videobuf_streamon", __func__);
	/* if HW streaming on is successful, start buffer streaming */
	rc = videobuf_streamon(&pcam->vid_bufq);
	D("%s, videobuf_streamon returns %d\n", __func__, rc);

	return rc;
}

static int msm_camera_v4l2_streamoff(struct file *f, void *pctx,
					enum v4l2_buf_type i)
{
	int rc = 0;
	/* get the camera device */
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	/* first turn of HW (VFE/sensor) streaming so that buffers are
	  not in use when we free the buffers */
	rc = msm_isp_streamoff(pcam);
	if (rc < 0)
		D("%s: hw failed to stop streaming\n", __func__);

	/* stop buffer streaming */
	rc = videobuf_streamoff(&pcam->vid_bufq);
	D("%s, videobuf_streamoff returns %d\n", __func__, rc);

	return rc;
}

static int msm_camera_v4l2_enum_fmt_cap(struct file *f, void *pctx,
					struct v4l2_fmtdesc *pfmtdesc)
{
	/* get the video device */
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);
	const struct msm_isp_color_fmt *isp_fmt;

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	if (pfmtdesc->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (pfmtdesc->index >= pcam->num_fmts)
		return -EINVAL;

	isp_fmt = &pcam->usr_fmts[pfmtdesc->index];

	if (isp_fmt->name)
		strlcpy(pfmtdesc->description, isp_fmt->name,
						sizeof(pfmtdesc->description));

	pfmtdesc->pixelformat = isp_fmt->fourcc;

	D("%s: [%d] 0x%x, %s\n", __func__, pfmtdesc->index,
		isp_fmt->fourcc, isp_fmt->name);
	return 0;
}

static int msm_camera_v4l2_g_fmt_cap(struct file *f,
		void *pctx, struct v4l2_format *pfmt)
{
	int rc = 0;
	/* get the video device */
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	if (pfmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	rc = msm_isp_get_fmt(pcam, pfmt);

	D("%s: current_fmt->fourcc: 0x%08x, rc = %d\n", __func__,
				pfmt->fmt.pix.pixelformat, rc);
	return rc;
}

/* This function will readjust the format parameters based in HW
  capabilities. Called by s_fmt_cap
*/
static int msm_camera_v4l2_try_fmt_cap(struct file *f, void *pctx,
					struct v4l2_format *pfmt)
{
	int rc = 0;
	/* get the video device */
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);
	struct v4l2_pix_format *pix = &pfmt->fmt.pix;

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	rc = msm_isp_try_fmt(pcam, pfmt);
	if (rc)
		D("Format %x not found, rc = %d\n",
				pix->pixelformat, rc);

	return rc;
}

/* This function will reconfig the v4l2 driver and HW device, it should be
   called after the streaming is stopped.
*/
static int msm_camera_v4l2_s_fmt_cap(struct file *f, void *pctx,
					struct v4l2_format *pfmt)
{
	int rc;
	/* get the video device */
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);

	D("%s\n", __func__);
	D("%s priv = 0x%p\n", __func__, (void *)pfmt->fmt.pix.priv);
	WARN_ON(pctx != f->private_data);

	mutex_lock(&pcam->vid_bufq.vb_lock);

	rc = msm_isp_set_fmt(pcam, pfmt);
	D("%s rc = %d\n", __func__, rc);
	if (rc < 0)
		D("msm_isp_set_fmt Error\n");

	/*unlock:*/
	mutex_unlock(&pcam->vid_bufq.vb_lock);

	return rc;
}

static int msm_camera_v4l2_g_jpegcomp(struct file *f, void *pctx,
				struct v4l2_jpegcompression *pcomp)
{
	int rc = 0;

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	return rc;
}

static int msm_camera_v4l2_s_jpegcomp(struct file *f, void *pctx,
				struct v4l2_jpegcompression *pcomp)
{
	int rc = 0;

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	return rc;
}


static int msm_camera_v4l2_g_crop(struct file *f, void *pctx,
					struct v4l2_crop *a)
{
	int rc = 0;

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	return rc;
}

static int msm_camera_v4l2_s_crop(struct file *f, void *pctx,
					struct v4l2_crop *a)
{
	int rc = 0;

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	return rc;
}

/* Stream type-dependent parameter ioctls */
static int msm_camera_v4l2_g_parm(struct file *f, void *pctx,
				struct v4l2_streamparm *a)
{
	int rc = 0;
	/* get the video device */
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	rc = msm_isp_g_parm(pcam, a);
	if (rc)
		D("msm_isp_s_parm Error\n");

	return rc;
}

static int msm_camera_v4l2_s_parm(struct file *f, void *pctx,
				struct v4l2_streamparm *a)
{
	int rc = 0;
	/* get the video device */
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	rc = msm_isp_s_parm(pcam, a);
	if (rc)
		D("msm_isp_s_parm Error\n");

	return rc;
}

static int msm_camera_v4l2_subscribe_event(struct v4l2_fh *fh,
			struct v4l2_event_subscription *sub)
{
	int rc = 0;

	D("%s\n", __func__);
	D("fh = 0x%x\n", (u32)fh);

	/* handle special case where user wants to subscribe to all
	the events */
	D("sub->type = 0x%x\n", sub->type);

	if (sub->type == V4L2_EVENT_ALL) {
		/*sub->type = MSM_ISP_EVENT_START;*/
		sub->type = V4L2_EVENT_PRIVATE_START + MSM_CAM_RESP_CTRL;

		D("sub->type start = 0x%x\n", sub->type);
		do {
			rc = v4l2_event_subscribe(fh, sub);
			if (rc < 0) {
				D("%s: failed for evtType = 0x%x, rc = %d\n",
						__func__, sub->type, rc);
			/* unsubscribe all events here and return */
			sub->type = V4L2_EVENT_ALL;
			v4l2_event_unsubscribe(fh, sub);
			return rc;
			} else
				D("%s: subscribed evtType = 0x%x, rc = %d\n",
						__func__, sub->type, rc);
			sub->type++;
			D("sub->type while = 0x%x\n", sub->type);
		} while (sub->type !=
			V4L2_EVENT_PRIVATE_START + MSM_CAM_RESP_MAX);
	} else {
		D("sub->type not V4L2_EVENT_ALL = 0x%x\n", sub->type);
		rc = v4l2_event_subscribe(fh, sub);
		if (rc < 0)
			D("%s: failed for evtType = 0x%x, rc = %d\n",
						__func__, sub->type, rc);
	}

	D("%s: rc = %d\n", __func__, rc);
	return rc;
}

static int msm_camera_v4l2_unsubscribe_event(struct v4l2_fh *fh,
			struct v4l2_event_subscription *sub)
{
	int rc = 0;

	D("%s\n", __func__);
	D("fh = 0x%x\n", (u32)fh);

	rc = v4l2_event_unsubscribe(fh, sub);

	D("%s: rc = %d\n", __func__, rc);
	return rc;
}

/* v4l2_ioctl_ops */
static const struct v4l2_ioctl_ops g_msm_ioctl_ops = {
	.vidioc_querycap = msm_camera_v4l2_querycap,

	.vidioc_s_crop = msm_camera_v4l2_s_crop,
	.vidioc_g_crop = msm_camera_v4l2_g_crop,

	.vidioc_queryctrl = msm_camera_v4l2_queryctrl,
	.vidioc_g_ctrl = msm_camera_v4l2_g_ctrl,
	.vidioc_s_ctrl = msm_camera_v4l2_s_ctrl,

	.vidioc_reqbufs = msm_camera_v4l2_reqbufs,
	.vidioc_querybuf = msm_camera_v4l2_querybuf,
	.vidioc_qbuf = msm_camera_v4l2_qbuf,
	.vidioc_dqbuf = msm_camera_v4l2_dqbuf,

	.vidioc_streamon = msm_camera_v4l2_streamon,
	.vidioc_streamoff = msm_camera_v4l2_streamoff,

	/* format ioctls */
	.vidioc_enum_fmt_vid_cap = msm_camera_v4l2_enum_fmt_cap,
	.vidioc_try_fmt_vid_cap = msm_camera_v4l2_try_fmt_cap,
	.vidioc_g_fmt_vid_cap = msm_camera_v4l2_g_fmt_cap,
	.vidioc_s_fmt_vid_cap = msm_camera_v4l2_s_fmt_cap,

	.vidioc_g_jpegcomp = msm_camera_v4l2_g_jpegcomp,
	.vidioc_s_jpegcomp = msm_camera_v4l2_s_jpegcomp,

	/* Stream type-dependent parameter ioctls */
	.vidioc_g_parm =  msm_camera_v4l2_g_parm,
	.vidioc_s_parm =  msm_camera_v4l2_s_parm,

	/* event subscribe/unsubscribe */
	.vidioc_subscribe_event = msm_camera_v4l2_subscribe_event,
	.vidioc_unsubscribe_event = msm_camera_v4l2_unsubscribe_event,
};

/* v4l2_file_operations */
static int msm_open(struct file *f)
{
	int rc = -EINVAL;
	struct msm_isp_ops *p_isp = 0;
	/* get the video device */
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);

	D("%s\n", __func__);

	if (!pcam) {
		D("%s NULL pointer passed in!\n", __func__);
		return rc;
	}
	D("%s for %s\n", __func__, pcam->pdev->name);

	p_isp = &pcam->isp;

	/* Should be set to sensor ops if any but right now its OK!! */
	if (!p_isp || !p_isp->isp_init) {
		D("%s: p_isp or isp_init is NULL\n", __func__);
		return -ENODEV;
	}

	mutex_lock(&pcam->vid_lock);

	pcam->use_count++;

	/* Now we really have to activate the camera */
	if (pcam->use_count == 1) {
		D("%s: call isp_init\n", __func__);
		p_isp->isp_init(pcam);
		D("pcam open = 0x%x\n", (u32)pcam);
	} else {
		/* if already opened just return*/
		mutex_unlock(&pcam->vid_lock);
		return 0;
	}

	D("%s: camera open\n", __func__);
	D("%s: vfe subdev address is 0x%p\n", __func__, &(pcam->isp.vdev));
	/* for any video buffer queue */
	INIT_LIST_HEAD(&pcam->framequeue);
	spin_lock_init(&pcam->vb_irqlock);

	/* Initialize the video queue */
	rc = p_isp->isp_init_vidbuf(&pcam->vid_bufq, pcam);
	if (rc < 0) {
		mutex_unlock(&pcam->vid_lock);
		return rc;
	}

	f->private_data = &pcam->eventHandle;

	D("f->private_data = 0x%x, pcam = 0x%x\n",
		(u32)f->private_data, (u32)pcam);

	/* setup native event_q and ctrl_q for internal communication*/
	msm_queue_init(&pcam->ctrl_q, "control");

	mutex_unlock(&pcam->vid_lock);

	/* more code and error handling to be done */
	return 0;
}

static int msm_mmap(struct file *f, struct vm_area_struct *vma)
{
	int rc = 0;
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);

	D("mmap called, vma=0x%08lx\n", (unsigned long)vma);

	rc = videobuf_mmap_mapper(&pcam->vid_bufq, vma);

	D("vma start=0x%08lx, size=%ld, ret=%d\n",
		(unsigned long)vma->vm_start,
		(unsigned long)vma->vm_end - (unsigned long)vma->vm_start,
		rc);

	return rc;
}

static int msm_close(struct file *f)
{
	int rc = 0;
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);

	D("%s\n", __func__);
	if (!pcam) {
		D("%s NULL pointer of camera device!\n", __func__);
		return -EINVAL;
	}

	videobuf_stop(&pcam->vid_bufq);

	mutex_lock(&pcam->vid_lock);
	pcam->use_count--;
	mutex_unlock(&pcam->vid_lock);

	f->private_data = NULL;

	/* release dma capable memory */
	dma_release_declared_memory(&pcam->pdev->dev);

	/* free the queue: function name is ambiguous it frees all
	types of buffers (mmap or userptr - it doesn't matter) */
	rc = videobuf_mmap_free(&pcam->vid_bufq);
	if (rc  < 0)
		D("%s: unable to free buffers\n", __func__);
	return rc;
}

static unsigned int msm_poll(struct file *f, struct poll_table_struct *wait)
{
	int rc = 0;
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);

	D("%s\n", __func__);
	if (!pcam) {
		D("%s NULL pointer of camera device!\n", __func__);
		return -EINVAL;
	}

	if (!pcam->vid_bufq.streaming) {
		D("%s vid_bufq.streaming is off\n", __func__);
		return -EINVAL;
	}

	rc |= videobuf_poll_stream(f, &pcam->vid_bufq, wait);
	D("%s returns, rc  = 0x%x\n", __func__, rc);

	return rc;
}

static unsigned int msm_poll_config(struct file *fp,
					struct poll_table_struct *wait)
{
	int rc = 0;

	struct msm_cam_v4l2_device *pcam  =
		(struct msm_cam_v4l2_device *)fp->private_data;

	D("%s\n", __func__);
	if (!pcam) {
		D("%s NULL pointer of camera device!\n", __func__);
		return -EINVAL;
	}

	poll_wait(fp, &pcam->eventHandle.events->wait, wait);
	if (v4l2_event_pending(&pcam->eventHandle))
		rc |= POLLPRI;

	return rc;
}
static long msm_ioctl_config(struct file *fp, unsigned int cmd,
	unsigned long arg)
{
	int rc = -EINVAL;
	struct v4l2_event ev;
	struct msm_cam_v4l2_device *pcam = fp->private_data;
	struct msm_isp_stats_event_ctrl *isp_event;

	D("%s: cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {

	case VIDIOC_DQEVENT:
		D("%s: VIDIOC_DQEVENT\n", __func__);
		if (copy_from_user(&ev, (void __user *)arg,
				sizeof(struct v4l2_event)))
			break;

		rc = v4l2_event_dequeue(&pcam->eventHandle, &ev,
			fp->f_flags & O_NONBLOCK);
		if (rc < 0) {
			printk(KERN_ERR "no pending events?");
			break;
		} else if (copy_to_user((void __user *)arg, &ev,
					sizeof(struct v4l2_event))) {
			rc = -EINVAL;
			break;
			}

		isp_event = (struct msm_isp_stats_event_ctrl *)ev.u.data;

		break;

	case MSM_CAM_IOCTL_CTRL_CMD_DONE:
		D("%s: MSM_CAM_IOCTL_CTRL_CMD_DONE\n", __func__);
		rc = msm_ctrl_cmd_done(pcam, (void __user *)arg);
		break;

	default:
		/* For the rest of config command, call HW config function*/
	  rc =  pcam->isp.isp_config(pcam, cmd, arg);
		break;
	}
	return rc;
}

static int msm_open_config(struct inode *inode, struct file *fp)
{
	int rc;
	struct msm_cam_v4l2_device *pcam =
	container_of(inode->i_cdev, struct msm_cam_v4l2_device, cdev);

	D("%s: open %s\n", __func__, fp->f_path.dentry->d_name.name);

	rc = nonseekable_open(inode, fp);
	if (rc < 0) {
		pr_err("%s: nonseekable_open error %d\n", __func__, rc);
		return rc;
	}

	fp->private_data = pcam;

	D("%s: rc %d\n", __func__, rc);
	D("%s initializing frame and stats list heads", __func__);
	INIT_HLIST_HEAD(&pcam->sync->pmem_frames);
	INIT_HLIST_HEAD(&pcam->sync->pmem_stats);
	D("%s pmem_stats address is 0x%p\n", __func__, &pcam->sync->pmem_stats);
	pcam->sync->unblock_poll_frame = 0;
	return rc;
}

#ifdef CONFIG_PROC_FS
static int msm_camera_v4l2_read_proc(char *pbuf, char **start, off_t offset,
			   int count, int *eof, void *data)
{
	int len = 0;

	*eof = 1;
	return len;
}
#endif

static struct v4l2_file_operations g_msm_fops = {
	.owner   = THIS_MODULE,
	.open	= msm_open,
	.poll	= msm_poll,
	.mmap	= msm_mmap,
	.release = msm_close,
	.ioctl   = video_ioctl2,
};

/* Init a config node for ISP control,
   which will create a config device (/dev/config0/ and plug in
   ISP's operation "v4l2_ioctl_ops*"
*/
static const struct file_operations msm_fops_config = {
	.owner = THIS_MODULE,
	.open  = msm_open_config,
	.poll  = msm_poll_config,
	.unlocked_ioctl = msm_ioctl_config,
};

static int msm_setup_cdev(struct msm_cam_v4l2_device *pcam, int node)
{
	int rc = -ENODEV;
	struct device *device;
	int dev_num = MSM_CDEV_PER_SENSOR * node;
	dev_t devno;

	static struct class *msm_class;
	static dev_t msm_devno;

	D("%s\n", __func__);

	if (!msm_class) {
		/* There is one device nodes per sensor */
		rc = alloc_chrdev_region(&msm_devno, 0,
		MSM_CDEV_PER_SENSOR * MSM_MAX_CAMERA_SENSORS,
		"msm_camera");
		if (rc < 0) {
			pr_err("%s: failed to allocate chrdev: %d\n", __func__,
			rc);
			return rc;
		}

		msm_class = class_create(THIS_MODULE, "msm_camera");
		if (IS_ERR(msm_class)) {
			rc = PTR_ERR(msm_class);
			pr_err("%s: create device class failed: %d\n",
			__func__, rc);
			return rc;
		}
	}

	devno = MKDEV(MAJOR(msm_devno), dev_num);
	device = device_create(msm_class, NULL, devno, NULL, "%s%d", "config",
									node);

	if (IS_ERR(device)) {
		rc = PTR_ERR(device);
		pr_err("%s: error creating device: %d\n", __func__, rc);
		return rc;
	}

	cdev_init(&pcam->cdev, &msm_fops_config);
	pcam->cdev.owner = THIS_MODULE;

	rc = cdev_add(&pcam->cdev, devno, 1);
	if (rc < 0) {
		pr_err("%s: error adding cdev: %d\n", __func__, rc);
		device_destroy(msm_class, devno);
		return rc;
	}

	return rc;
}

/* Init a msm device for ISP control,
   which will create a video device (/dev/video0/ and plug in
   ISP's operation "v4l2_ioctl_ops*"
*/
static int msm_cam_dev_init(struct msm_cam_v4l2_device *pcam)
{
	int rc = -ENOMEM;
	struct video_device *pvdev = NULL;
	D("%s\n", __func__);

	/* first register the v4l2 device */
	pcam->v4l2_dev.dev = &pcam->pdev->dev;
	rc = v4l2_device_register(pcam->v4l2_dev.dev, &pcam->v4l2_dev);
	if (rc < 0)
		return -EINVAL;

	/* now setup video device */
	pvdev = video_device_alloc();
	if (pvdev == NULL) {
		D("%s: video_device_alloc failed\n", __func__);
		return rc;
	}

	/* init video device's driver interface */
	D("sensor name = %s, sizeof(pvdev->name)=%d\n",
		pcam->pdev->name, sizeof(pvdev->name));

	/* device info - strlcpy is safer than strncpy but
	   only if architecture supports*/
	strlcpy(pvdev->name, pcam->pdev->name, sizeof(pvdev->name));

	pvdev->release   = video_device_release;
	pvdev->fops	  = &g_msm_fops;
	pvdev->ioctl_ops = &g_msm_ioctl_ops;
	pvdev->minor	 = -1;
	pvdev->vfl_type  = 1;

	/* register v4l2 video device to kernel as /dev/videoXX */
	D("video_register_device\n");
	rc = video_register_device(pvdev,
				   VFL_TYPE_GRABBER,
				   msm_camera_v4l2_nr);
	if (rc) {
		D("%s: video_register_device failed\n", __func__);
		goto reg_fail;
	}
	D("%s: video device registered as /dev/video%d\n",
		__func__, pvdev->num);

	/* connect pcam and video dev to each other */
	pcam->pvdev	= pvdev;
	video_set_drvdata(pcam->pvdev, pcam);

	/* plug in v4l2_ioctl_ops specific to a isp HW*/
	D("msm_isp_register\n");
	rc = msm_isp_register(pcam);
	if (rc) {
		D("%s: msm_isp_register failed\n", __func__);
		goto cleanup;
	} else
		D("%s: msm_isp_register done, ioctl_ops is 0X%x\n",
			__func__, (unsigned int) pcam->pvdev->ioctl_ops);

	/* If isp HW registeration is successful, then create event queue to
	   receievent event froms HW
	 */
	rc = v4l2_fh_init(&pcam->eventHandle, pcam->pvdev);
	D("v4l2_fh_init: eventHandle = 0x%x, rc = 0x%x\n",
	  (u32)&pcam->eventHandle,  rc);
	if (rc < 0)
		return rc;

	D("calling v4l2_event_init\n");
	rc = v4l2_event_init(&pcam->eventHandle);
	if (rc < 0)
		return rc;

	/* queue of max size 30 */
	D("calling v4l2_event_alloc\n");
	rc = v4l2_event_alloc(&pcam->eventHandle, 30);
	if (rc < 0)
		return rc;

	D("calling v4l2_fh_add\n");
	v4l2_fh_add(&pcam->eventHandle);
	D("v4l2_fh_add: eventHandle = 0x%x\n", (u32)&pcam->eventHandle);
	pcam->eventQueueSize = 1;

	/* yyan: no global - each sensor will create a new vidoe node! */
	/* g_pmsm_camera_v4l2_dev = pmsm_camera_v4l2_dev; */
	/* g_pmsm_camera_v4l2_dev->pvdev = pvdev; */


#ifdef CONFIG_PROC_FS
	create_proc_read_entry(MSM_V4L2_PROC_NAME,
				   0, NULL, msm_camera_v4l2_read_proc, NULL);
#endif


	return rc ;

cleanup:
	video_set_drvdata(pcam->pvdev, NULL);
	pcam->pvdev = NULL;
	video_unregister_device(pvdev);

reg_fail:
	video_device_release(pvdev);
	v4l2_device_unregister(&pcam->v4l2_dev);
	pcam->v4l2_dev.dev = NULL;
	return rc;
}

static int msm_sync_destroy(struct msm_sync *sync)
{
	if (sync)
		wake_lock_destroy(&sync->wake_lock);
	return 0;
}
static int msm_sync_init(struct msm_sync *sync,
	struct platform_device *pdev, struct msm_sensor_ctrl *sctrl)
{
	int rc = 0;

	sync->sdata = pdev->dev.platform_data;

	wake_lock_init(&sync->wake_lock, WAKE_LOCK_IDLE, "msm_camera");

	sync->pdev = pdev;
	sync->sctrl = *sctrl;
	sync->opencnt = 0;
	mutex_init(&sync->lock);
	D("%s: initialized %s\n", __func__, sync->sdata->sensor_name);
	return rc;
}

/* register a msm sensor into the msm device, which will probe the
   sensor HW. if the HW exist then create a video device (/dev/videoX/)
   to represent this sensor */
int msm_sensor_register(struct platform_device *pdev,
		int (*sensor_probe)(const struct msm_camera_sensor_info *,
			struct v4l2_subdev **, struct msm_sensor_ctrl *))
{

	int rc = -EINVAL;
	struct msm_camera_sensor_info *sdata = pdev->dev.platform_data;
	struct msm_cam_v4l2_device *pcam;
	struct v4l2_subdev *sdev;
	struct msm_sensor_ctrl sctrl;

	D("%s for %s\n", __func__, pdev->name);

	/* come sensor probe logic */
	rc = msm_camio_probe_on(pdev);
	if (rc < 0)
		return rc;

	rc = sensor_probe(sdata, &sdev, &sctrl);

	msm_camio_probe_off(pdev);
	if (rc < 0) {
		D("%s: failed to detect %s\n",
			__func__,
		sdata->sensor_name);
		return rc;
	}

	/* if the probe is successfull, allocat the camera driver object
	   for this sensor */
	pcam = kzalloc(sizeof(*pcam), GFP_KERNEL);
	if (!pcam) {
		D("%s: could not allocate memory for msm_cam_v4l2_device\n",
			__func__);
		return -ENOMEM;
	}

	pcam->sync = kzalloc(sizeof(struct msm_sync), GFP_ATOMIC);
	if (!pcam->sync) {
		D("%s: could not allocate memory for msm_sync object\n",
			__func__);
		return -ENOMEM;
	}

	/* setup a manager object*/
	rc = msm_sync_init(pcam->sync, pdev, &sctrl);
	if (rc < 0)
		goto failure;
	D("%s: pcam =0x%p\n", __func__, pcam);
	D("%s: pcam->sync =0x%p\n", __func__, pcam->sync);

	(pcam->sync)->pcam_sync = pcam;
	/* bind the driver device to the sensor device */
	pcam->pdev = pdev;
	/* get the sensor subdevice */
	pcam->isp.sdev = sdev;

	/* init the user count and lock*/
	pcam->use_count = 0;
	mutex_init(&pcam->vid_lock);

	/* Initialize the formats supported */
	rc  = msm_isp_init_user_formats(pcam);
	if (rc < 0)
		goto failure;

	/* now initialize the camera device object */
	rc  = msm_cam_dev_init(pcam);
	if (rc < 0)
		goto failure;

	/* now initialize the native config device node */
	rc = msm_setup_cdev(pcam, number_of_sensors);
	if (rc < 0)
		goto failure;

	number_of_sensors++;
	D("%s done, rc = %d\n", __func__, rc);
	D("%s number of sensors connected is %d\n", __func__,
					number_of_sensors);
	if (number_of_sensors == 1) {
		rc = add_axi_qos();
		if (rc < 0)
			goto failure;
	}
	return rc;

failure:
	/* mutex_destroy not needed at this moment as the associated
	implemenation of mutex_init is not consuming resources */
	msm_sync_destroy(pcam->sync);
	pcam->isp.sdev = NULL;
	pcam->pdev = NULL;
	kfree(pcam);
	return rc;
}
EXPORT_SYMBOL(msm_sensor_register);
