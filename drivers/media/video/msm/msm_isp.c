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
#include <linux/videodev2.h>
#include <linux/proc_fs.h>
#include <linux/vmalloc.h>

#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-device.h>

#include <linux/android_pmem.h>

#include "msm.h"
#include "msm_vfe31.h"

#ifdef CONFIG_MSM_CAMERA_DEBUG
#define D(fmt, args...) printk(KERN_DEBUG "msm_isp: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif
#define ERR_USER_COPY(to) pr_err("%s(%d): copy %s user\n", \
				__func__, __LINE__, ((to) ? "to" : "from"))
#define ERR_COPY_FROM_USER() ERR_USER_COPY(0)
#define ERR_COPY_TO_USER() ERR_USER_COPY(1)


#define PAD_TO_WORD(a)	  (((a) + 3) & ~3)

#define __CONTAINS(r, v, l, field) ({			   \
	typeof(r) __r = r;				  \
	typeof(v) __v = v;				  \
	typeof(v) __e = __v + l;				\
	int res = __v >= __r->field &&			  \
		__e <= __r->field + __r->len;		   \
	res;							\
})

#define CONTAINS(r1, r2, field) ({			  \
	typeof(r2) __r2 = r2;				   \
	__CONTAINS(r1, __r2->field, __r2->len, field);	  \
})

#define IN_RANGE(r, v, field) ({				\
	typeof(r) __r = r;				  \
	typeof(v) __vv = v;				 \
	int res = ((__vv >= __r->field) &&		  \
		(__vv < (__r->field + __r->len)));	  \
	res;							\
})

#define OVERLAPS(r1, r2, field) ({			  \
	typeof(r1) __r1 = r1;				   \
	typeof(r2) __r2 = r2;				   \
	typeof(__r2->field) __v = __r2->field;		  \
	typeof(__v) __e = __v + __r2->len - 1;		  \
	int res = (IN_RANGE(__r1, __v, field) ||		\
		IN_RANGE(__r1, __e, field));				 \
	res;							\
})

#define MAX_VIDEO_MEM 16
/* VFE required buffer number for streaming */
#define VFE_OUT1_BUF 3

static DEFINE_MUTEX(hlist_mut);

static struct msm_isp_color_fmt msm_isp_formats[] = {
	{
	.name	   = "NV21YUV",
	.depth	  = 12,
	.bitsperpxl = 8,
	.fourcc	 = V4L2_PIX_FMT_NV21,
	.pxlcode	= V4L2_MBUS_FMT_YUYV8_2X8_BE, /* YUV sensor */
	.colorspace = V4L2_COLORSPACE_JPEG,
	},
	{
	.name	   = "NV21BAYER",
	.depth	  = 8,
	.bitsperpxl = 8,
	.fourcc	 = V4L2_PIX_FMT_NV21,
	.pxlcode	= V4L2_MBUS_FMT_SBGGR8_1X8, /* Bayer sensor */
	.colorspace = V4L2_COLORSPACE_JPEG,
	},
};

/* msm queue management APIs*/

#define msm_dequeue(queue, member) ({	   \
	unsigned long flags;		  \
	struct msm_device_queue *__q = (queue);	 \
	struct msm_queue_cmd *qcmd = 0;	   \
	spin_lock_irqsave(&__q->lock, flags);	 \
	if (!list_empty(&__q->list)) {		\
		__q->len--;		 \
		qcmd = list_first_entry(&__q->list,   \
		struct msm_queue_cmd, member);  \
		list_del_init(&qcmd->member);	 \
	}			 \
	spin_unlock_irqrestore(&__q->lock, flags);  \
	qcmd;			 \
})

#define msm_queue_drain(queue, member) do {	 \
	unsigned long flags;		  \
	struct msm_device_queue *__q = (queue);	 \
	struct msm_queue_cmd *qcmd;	   \
	spin_lock_irqsave(&__q->lock, flags);	 \
	D("%s: draining queue %s\n", __func__, __q->name); \
	while (!list_empty(&__q->list)) {	 \
		qcmd = list_first_entry(&__q->list,   \
			struct msm_queue_cmd, member);	\
			list_del_init(&qcmd->member);	 \
			free_qcmd(qcmd);		\
	 };			  \
	spin_unlock_irqrestore(&__q->lock, flags);	\
} while (0)

static inline void free_qcmd(struct msm_queue_cmd *qcmd)
{
	D("%s\n", __func__);
	if (!qcmd || !atomic_read(&qcmd->on_heap))
		return;
	if (!atomic_sub_return(1, &qcmd->on_heap))
		kfree(qcmd);
}

/* send control command to config and wait for results*/
static int msm_isp_control(struct msm_cam_v4l2_device *pcam,
				struct msm_ctrl_cmd *out)
{
	int rc = 0;
	struct msm_queue_cmd *rcmd;
	struct msm_device_queue *queue =  &pcam->ctrl_q;

	struct v4l2_event v4l2_evt;
	struct msm_isp_stats_event_ctrl *isp_event;
	D("%s\n", __func__);

	v4l2_evt.type = V4L2_EVENT_PRIVATE_START + MSM_CAM_RESP_V4L2;

	/* setup event object to transfer the command; */
	isp_event = (struct msm_isp_stats_event_ctrl *)v4l2_evt.u.data;
	isp_event->resptype = MSM_CAM_RESP_V4L2;
	isp_event->isp_data.ctrl = *out;

	/* now send command to config thread in usersspace,
	 * and wait for results */
	v4l2_event_queue(pcam->pvdev, &v4l2_evt);

	D("%s v4l2_event_queue: type = 0x%x\n", __func__, v4l2_evt.type);

	/* wait for config return status */
	D("Waiting for config status\n");
	rc = wait_event_interruptible_timeout(queue->wait,
		!list_empty_careful(&queue->list),
		out->timeout_ms);
	D("Waiting over for config status\n");
	if (list_empty_careful(&queue->list)) {
		if (!rc)
			rc = -ETIMEDOUT;
		if (rc < 0) {
			pr_err("%s: wait_event error %d\n", __func__, rc);
			return rc;
		}
	}

	rcmd = msm_dequeue(queue, list_control);
	BUG_ON(!rcmd);
	D("%s Finished servicing ioctl\n", __func__);

	free_qcmd(rcmd);
	D("%s: rc %d\n", __func__, rc);
	return rc;
}

/*
 *  Videobuf operations
 */
static void free_buffer(struct videobuf_queue *vq,
			struct msm_frame_buffer *buf)
{
	struct videobuf_buffer *vb = &buf->vidbuf;

	BUG_ON(in_interrupt());

	D("%s (vb=0x%p) 0x%08lx %d\n", __func__,
			vb, vb->baddr, vb->bsize);

	/* This waits until this buffer is out of danger, i.e.,
	 * until it is no longer in STATE_QUEUED or STATE_ACTIVE */
	videobuf_waiton(vb, 0, 0);
	videobuf_pmem_contig_free(vq, vb);
	vb->state = VIDEOBUF_NEEDS_INIT;
}

/* Setup # of buffers and size of each buffer for the videobuf_queue.
   This is called when videobuf_reqbufs() is called, so this function
   should tell how many buffer should be used and how big the size is.

   The caller will allocate the real buffers, either in user space or
   in kernel */
static int msm_vidbuf_setup(struct videobuf_queue *vq, unsigned int *count,
							unsigned int *size)
{
	/* get the video device */
	struct msm_cam_v4l2_device *pcam = vq->priv_data;

	D("%s\n", __func__);
	if (!pcam || !count || !size) {
		D("%s error : invalid input\n", __func__);
		return -EINVAL;
	}

	/* at least we should provide 3 buffers to VFE and 1 in buffer queue*/
	if (*count <= VFE_OUT1_BUF)
		*count = VFE_OUT1_BUF + 1;

	/* we support only NV21 format for any input mediabus format */
	D("%s width = %d\n", __func__, pcam->vid_fmt.fmt.pix.width);
	D("%s height = %d\n", __func__, pcam->vid_fmt.fmt.pix.height);
	*size = pcam->vid_fmt.fmt.pix.width *
		pcam->vid_fmt.fmt.pix.height * 3/2;
	/* if the total mem size is bigger than available memory,
	 * reduce the count*/
	while ((*size) * (*count) > MAX_VIDEO_MEM * 1024 * 1024)
		(*count)--;

	/* if there are not buffers for VFE then fail*/
	if (*count < VFE_OUT1_BUF + 1) {
		D("%s error : out of memory input\n", __func__);
		return -ENOMEM;
	}

	D("count=%d, size=%d\n", *count, *size);

	return 0;
}

/* Prepare the buffer before it is put into the videobuf_queue for streaming.
   This is called when videobuf_qbuf() is called, so this function should
   setup the video buffer to receieve the VFE output. */
static int msm_vidbuf_prepare(struct videobuf_queue *vq,
	struct videobuf_buffer *vb, enum v4l2_field field)
{
	int rc = 0;
	struct msm_cam_v4l2_device *pcam = vq->priv_data;
	struct msm_frame_buffer *buf =
		container_of(vb, struct msm_frame_buffer, vidbuf);
	D("%s\n", __func__);
	if (!pcam) {
		D("%s error : pcam is NULL\n", __func__);
		return -EINVAL;
	}

	D("%s (vb=0x%p) 0x%08lx %d\n", __func__,
		vb, vb->baddr, vb->bsize);

	/* by this time vid_fmt should be already set */
	/* return error if it is not */
	if ((pcam->vid_fmt.fmt.pix.width == 0) ||
		(pcam->vid_fmt.fmt.pix.height == 0)) {
		D("%s error : pcam vid_fmt is not set\n", __func__);
		return -EINVAL;
	}

	buf->inuse = 1;

	D("buf->pxlcode=%d, pcam->sensor_pxlcode=%d, vb->width=%d,"
		"pcam->vid_fmt.fmt.pix.width = %d, vb->height = %d,"
		"pcam->vid_fmt.fmt.pix.height=%d, vb->field=%d, field=%d\n",
		buf->pxlcode, pcam->sensor_pxlcode, vb->width,
		pcam->vid_fmt.fmt.pix.width, vb->height,
		pcam->vid_fmt.fmt.pix.height, vb->field, field);

	if (buf->pxlcode != pcam->sensor_pxlcode ||
		vb->width   != pcam->vid_fmt.fmt.pix.width ||
		vb->height	!= pcam->vid_fmt.fmt.pix.height ||
		vb->field   != field) {
		buf->pxlcode  = pcam->sensor_pxlcode;
		vb->width = pcam->vid_fmt.fmt.pix.width;
		vb->height  = pcam->vid_fmt.fmt.pix.height;
		vb->field = field;
		vb->state = VIDEOBUF_NEEDS_INIT;
		D("VIDEOBUF_NEEDS_INIT\n");
	}

	/* For us, output will always be in NV21 format at least for now */
	vb->size = pcam->vid_fmt.fmt.pix.width * vb->height * 3/2;

	D("vb->size=%lu, vb->bsize=%u, vb->baddr=0x%x\n",
		vb->size, vb->bsize, (uint32_t)vb->baddr);

	if (0 != vb->baddr && vb->bsize < vb->size) {
		D("Something wrong vb->size=%lu, vb->bsize=%u,\
					vb->baddr=0x%x\n",
					vb->size, vb->bsize,
					(uint32_t)vb->baddr);
		rc = -EINVAL;
		goto out;
	}

	if (vb->state == VIDEOBUF_NEEDS_INIT) {
		rc = videobuf_iolock(vq, vb, NULL);
		if (rc)
			goto fail;
		D("%s: setting buffer state to prepared\n", __func__);
		vb->state = VIDEOBUF_PREPARED;
	}

	buf->inuse = 0;

	/* finally if everything is oK, set the VIDEOBUF_PREPARED state*/
	if (0 == rc)
		vb->state = VIDEOBUF_PREPARED;
	return rc;

fail:
	free_buffer(vq, buf);

out:
	buf->inuse = 0;
	return rc;
}

/* Called under spin_lock_irqsave(q->irqlock, flags) in videobuf-core.c*/
static void msm_vidbuf_queue(struct videobuf_queue *vq,
				struct videobuf_buffer *vb)
{
	struct msm_cam_v4l2_device *pcam = vq->priv_data;
	unsigned long phyaddr = 0;
	int rc;

	D("%s\n", __func__);
	if (!pcam) {
		D("%s error : pcam is NULL\n", __func__);
		return;
	}
	D("%s (vb=0x%p) 0x%08lx %d\n", __func__,
		vb, vb->baddr, vb->bsize);

	/* not needed at the moment */
	/* list_add_tail(&vb->queue, &pcam->framequeue); */

	vb->state = VIDEOBUF_QUEUED;
	if (vq->streaming) {
		struct msm_frame frame;
		/* we are returning a buffer to the queue */
		struct videobuf_contig_pmem *mem = vb->priv;
		/* get the physcial address of the buffer */
		phyaddr = (unsigned long) videobuf_to_pmem_contig(vb);

		D("%s buffer type is %d\n", __func__, mem->buffer_type);
		frame.path = mem->buffer_type;
		frame.buffer = 0;
		frame.y_off = mem->y_off;
		frame.cbcr_off = mem->cbcr_off;
		/* now release frame to vfe */
		{
			struct msm_vfe_cfg_cmd cfgcmd;
			cfgcmd.cmd_type = CMD_FRAME_BUF_RELEASE;
			cfgcmd.value	= (void *)&frame;
			rc = pcam->sync->vfefn.vfe_config(&cfgcmd, &phyaddr);
		}
	}
}

/* This will be called when streamingoff is called. */
static void msm_vidbuf_release(struct videobuf_queue *vq,
				struct videobuf_buffer *vb)
{
	struct msm_cam_v4l2_device *pcam = vq->priv_data;
	struct msm_frame_buffer *buf = container_of(vb, struct msm_frame_buffer,
									vidbuf);

	D("%s\n", __func__);
	if (!pcam || !vb || !vq) {
		D("%s error : input is NULL\n", __func__);
		return ;
	}
#ifdef DEBUG
	D("%s (vb=0x%p) 0x%08lx %d\n", __func__,
		vb, vb->baddr, vb->bsize);

	switch (vb->state) {
	case VIDEOBUF_ACTIVE:
		D("%s (active)\n", __func__);
		break;
	case VIDEOBUF_QUEUED:
		D("%s (queued)\n", __func__);
		break;
	case VIDEOBUF_PREPARED:
		D("%s (prepared)\n", __func__);
		break;
	default:
		D("%s (unknown) state = %d\n", __func__, vb->state);
		break;
	}
#endif

	/* free the buffer */
	free_buffer(vq, buf);
}

static unsigned long msm_pmem_stats_vtop_lookup(
				struct msm_sync *sync,
				unsigned long buffer,
				int fd)
{
	struct msm_pmem_region *region;
	struct hlist_node *node, *n;

	hlist_for_each_entry_safe(region, node, n, &sync->pmem_stats, list) {
		if (((unsigned long)(region->info.vaddr) == buffer) &&
						(region->info.fd == fd) &&
						region->info.active == 0) {
			region->info.active = 1;
			return region->paddr;
		}
	}

	return 0;
}

static unsigned long msm_pmem_stats_ptov_lookup(struct msm_sync *sync,
						unsigned long addr, int *fd)
{
	struct msm_pmem_region *region;
	struct hlist_node *node, *n;

	hlist_for_each_entry_safe(region, node, n, &sync->pmem_stats, list) {
		if (addr == region->paddr && region->info.active) {
			/* offset since we could pass vaddr inside a
			 * registered pmem buffer */
			*fd = region->info.fd;
			region->info.active = 0;
			return (unsigned long)(region->info.vaddr);
		}
	}

	return 0;
}

/* This will enqueue ISP events or signal buffer completion */
static int msm_isp_enqueue(struct msm_cam_v4l2_device *pcam,
				struct msm_vfe_resp *data,
				enum msm_queue qtype)
{
	struct v4l2_event v4l2_evt;

	struct videobuf_queue *q;
	struct videobuf_buffer *buf = NULL;
	uint32_t buf_phyaddr = 0;
	struct msm_stats_buf stats;

	struct msm_isp_stats_event_ctrl *isp_event;
	/* struct msm_stats_buf stats; */
	int i;
	unsigned long flags = 0;
	isp_event = (struct msm_isp_stats_event_ctrl *)v4l2_evt.u.data;
	if (!data) {
		D("%s !!!!data = 0x%p\n", __func__, data);
		return -EINVAL;
	}

	D("%s data->type = %d\n", __func__, data->type);

	switch (qtype) {
	case MSM_CAM_Q_VFE_EVT:
	case MSM_CAM_Q_VFE_MSG:
		/* adsp event and message */
		v4l2_evt.type = V4L2_EVENT_PRIVATE_START +
					MSM_CAM_RESP_STAT_EVT_MSG;

		isp_event->resptype = MSM_CAM_RESP_STAT_EVT_MSG;

		/* 0 - msg from aDSP, 1 - event from mARM */
		isp_event->isp_data.isp_msg.type   = data->evt_msg.type;
		isp_event->isp_data.isp_msg.msg_id = data->evt_msg.msg_id;
		isp_event->isp_data.isp_msg.len	= data->evt_msg.len;

		D("%s: qtype %d length %d msd_id %d\n", __func__,
					qtype,
					isp_event->isp_data.isp_msg.len,
					isp_event->isp_data.isp_msg.msg_id);

		if ((data->type >= VFE_MSG_STATS_AEC) &&
			(data->type <=  VFE_MSG_STATS_WE)) {

			D("%s data->phy.sbuf_phy = 0x%x\n", __func__,
						data->phy.sbuf_phy);
			stats.buffer = msm_pmem_stats_ptov_lookup(pcam->sync,
							data->phy.sbuf_phy,
							&(stats.fd));
			if (!stats.buffer) {
				pr_err("%s: msm_pmem_stats_ptov_lookup error\n",
								__func__);
				isp_event->isp_data.isp_msg.len = 0;
			} else {
				memcpy(&isp_event->isp_data.isp_msg.data[0],
						&stats,
						sizeof(struct msm_stats_buf));
				isp_event->isp_data.isp_msg.len =
						sizeof(struct msm_stats_buf);
			}

		} else if ((data->evt_msg.len > 0) &&
			(data->evt_msg.len <= 48) && /* only 48 bytes */
			(data->type == VFE_MSG_GENERAL)) {
			memcpy(&isp_event->isp_data.isp_msg.data[0],
						data->evt_msg.data,
						data->evt_msg.len);
		} else if (data->type == VFE_MSG_OUTPUT_P) {
			q = &(pcam->vid_bufq);

			D("pcam=0x%x\n", (u32)pcam);
			D("q=0x%x\n", (u32)q);

			/* find the videobuf which is done */
			for (i = 0; i < VIDEO_MAX_FRAME; i++) {
				if (NULL == q->bufs[i])
					continue;
				buf = q->bufs[i];
				buf_phyaddr = videobuf_to_pmem_contig(buf);
				D("buf_phyaddr=0x%x\n", (u32)buf_phyaddr);
				D("data->phy.y_phy=0x%x\n",
							(u32)data->phy.y_phy);
				D("buf = 0x%x\n", (u32)buf);
				if (buf_phyaddr == data->phy.y_phy)
					break;
			}

			/* signal that buffer is done */
			/* get the buf lock first */
			spin_lock_irqsave(q->irqlock, flags);
			buf->state = VIDEOBUF_DONE;
			D("queuedequeue video_buffer 0x%x,"
					"phyaddr = 0x%x\n",
					(u32)buf, (u32)data->phy.y_phy);

			do_gettimeofday(&buf->ts);
			buf->field_count++;
			wake_up(&buf->done);
			spin_unlock_irqrestore(q->irqlock, flags);
		}
		break;
	default:
		break;
	}

	/* now queue the event */
	v4l2_event_queue(pcam->pvdev, &v4l2_evt);
	return 0;
}

/*
 * This function executes in interrupt context.
 */

static void *msm_vfe_sync_alloc(int size,
	  void *syncdata __attribute__((unused)),
	  gfp_t gfp)
{
	struct msm_queue_cmd *qcmd =
		kmalloc(sizeof(struct msm_queue_cmd) + size, gfp);

	if (qcmd) {
		atomic_set(&qcmd->on_heap, 1);
		return qcmd + 1;
	}
	return NULL;
}

static void msm_vfe_sync_free(void *ptr)
{
	if (ptr) {
		struct msm_queue_cmd *qcmd =
			(struct msm_queue_cmd *)ptr;
		qcmd--;
		if (atomic_read(&qcmd->on_heap))
			kfree(qcmd);
	}
}

/*
 * This function executes in interrupt context.
 */

static void msm_vfe_sync(struct msm_vfe_resp *vdata,
		enum msm_queue qtype, void *syncdata,
		gfp_t gfp)
{
	struct msm_queue_cmd *qcmd = NULL;
	struct msm_sync *sync1 = (struct msm_sync *)syncdata;

	if (!sync1) {
		pr_err("%s: no context in dsp callback.\n", __func__);
		return;
	}

	qcmd = ((struct msm_queue_cmd *)vdata) - 1;
	qcmd->type = qtype;
	qcmd->command = vdata;

	if (qtype != MSM_CAM_Q_VFE_MSG)
		goto for_config;

	D("%s: vdata->type %d\n", __func__, vdata->type);
	switch (vdata->type) {
	case VFE_MSG_STATS_AWB:
		D("%s: qtype %d, AWB stats, enqueue event_q.\n",
					__func__, vdata->type);
		break;

	case VFE_MSG_STATS_AEC:
		D("%s: qtype %d, AEC stats, enqueue event_q.\n",
					__func__, vdata->type);
		break;

	case VFE_MSG_STATS_IHIST:
		D("%s: qtype %d, ihist stats, enqueue event_q.\n",
					__func__, vdata->type);
		break;

	case VFE_MSG_STATS_RS:
		D("%s: qtype %d, rs stats, enqueue event_q.\n",
					__func__, vdata->type);
		break;

	case VFE_MSG_STATS_CS:
		D("%s: qtype %d, cs stats, enqueue event_q.\n",
					__func__, vdata->type);
	break;

	case VFE_MSG_GENERAL:
		D("%s: qtype %d, general msg, enqueue event_q.\n",
					__func__, vdata->type);
		break;
	default:
		D("%s: qtype %d not handled\n", __func__, vdata->type);
		/* fall through, send to config. */
	}

for_config:
	D("%s: msm_enqueue event_q\n", __func__);
	msm_isp_enqueue(sync1->pcam_sync, vdata, qtype);

	msm_vfe_sync_free(vdata);
}

static struct msm_vfe_callback msm_vfe_s = {
	.vfe_resp = msm_vfe_sync,
	.vfe_alloc = msm_vfe_sync_alloc,
	.vfe_free = msm_vfe_sync_free,
};

static int __msm_open(struct msm_sync *sync, const char *const apps_id)
{
	int rc = 0;
	D("%s\n", __func__);
	mutex_lock(&sync->lock);
	sync->apps_id = apps_id;

	if (!sync->opencnt) {
		wake_lock(&sync->wake_lock);

		msm_camvfe_fn_init(&sync->vfefn, sync);
		if (sync->vfefn.vfe_init) {
			sync->get_pic_abort = 0;
			rc = msm_camio_sensor_clk_on(sync->pdev);
			if (rc < 0) {
				D("%s: setting sensor clocks failed: %d\n",
								__func__, rc);
				goto msm_open_done;
			}
			rc = sync->vfefn.vfe_init(&msm_vfe_s, sync->pdev);
			if (rc < 0) {
				pr_err("%s: vfe_init failed at %d\n",
							__func__, rc);
				goto msm_open_done;
			}
			rc = sync->sctrl.s_init(sync->sdata);
			if (rc < 0) {
				pr_err("%s: sensor init failed: %d\n",
							__func__, rc);
				goto msm_open_done;
			}
		} else {
			pr_err("%s: no sensor init func\n", __func__);
			rc = -ENODEV;
			goto msm_open_done;
		}
	}
	sync->opencnt++;

msm_open_done:
	mutex_unlock(&sync->lock);
	return rc;
}

/* This function is called by open() function, so we need to init HW*/
static int msm_isp_init(struct msm_cam_v4l2_device *pcam)
{
	/* init vfe and senor, register sync callbacks for init*/
	int rc = 0;
	D("%s\n", __func__);
	/* use old code for vfe31 for now*/
	rc = __msm_open(pcam->sync, MSM_APPS_ID_V4L2);
	return rc;
}

static int msm_isp_release(struct msm_cam_v4l2_device *pcam)
{
	D("%s\n", __func__);
	return 0;
}

static int check_pmem_info(struct msm_pmem_info *info, int len)
{
	if (info->offset < len &&
		info->offset + info->len <= len &&
		info->y_off < len &&
		info->cbcr_off < len)
		return 0;

	pr_err("%s: check failed: off %d len %d y %d cbcr %d (total len %d)\n",
						__func__,
						info->offset,
						info->len,
						info->y_off,
						info->cbcr_off,
						len);
	return -EINVAL;
}

static int check_overlap(struct hlist_head *ptype,
				unsigned long paddr,
				unsigned long len)
{
	struct msm_pmem_region *region;
	struct msm_pmem_region t = { .paddr = paddr, .len = len };
	struct hlist_node *node;

	hlist_for_each_entry(region, node, ptype, list) {
		if (CONTAINS(region, &t, paddr) ||
			CONTAINS(&t, region, paddr) ||
			OVERLAPS(region, &t, paddr)) {
			CDBG(" region (PHYS %p len %ld)"
				" clashes with registered region"
				" (paddr %p len %ld)\n",
				(void *)t.paddr, t.len,
				(void *)region->paddr, region->len);
			return -EINVAL;
		}
	}

	return 0;
}

static int msm_pmem_table_add(struct hlist_head *ptype,
	struct msm_pmem_info *info)
{
	struct file *file;
	unsigned long paddr;
	unsigned long kvstart;
	unsigned long len;
	int rc;
	struct msm_pmem_region *region;

	rc = get_pmem_file(info->fd, &paddr, &kvstart, &len, &file);
	if (rc < 0) {
		pr_err("%s: get_pmem_file fd %d error %d\n",
						__func__,
						info->fd, rc);
		return rc;
	}

	if (!info->len)
		info->len = len;

	rc = check_pmem_info(info, len);
	if (rc < 0)
		return rc;

	paddr += info->offset;
	len = info->len;

	if (check_overlap(ptype, paddr, len) < 0)
		return -EINVAL;

	CDBG("%s: type %d, active flag %d, paddr 0x%lx, vaddr 0x%lx\n",
		__func__, info->type, info->active, paddr,
		(unsigned long)info->vaddr);

	region = kmalloc(sizeof(struct msm_pmem_region), GFP_KERNEL);
	if (!region)
		return -ENOMEM;

	INIT_HLIST_NODE(&region->list);

	region->paddr = paddr;
	region->len = len;
	region->file = file;
	memcpy(&region->info, info, sizeof(region->info));
	D("%s Adding region to list with type %d\n", __func__,
						region->info.type);
	D("%s pmem_stats address is 0x%p\n", __func__, ptype);
	hlist_add_head(&(region->list), ptype);

	return 0;
}

static int __msm_register_pmem(struct msm_sync *sync,
			struct msm_pmem_info *pinfo)
{
	int rc = 0;

	switch (pinfo->type) {
	case MSM_PMEM_VIDEO:
	case MSM_PMEM_PREVIEW:
	case MSM_PMEM_THUMBNAIL:
	case MSM_PMEM_MAINIMG:
	case MSM_PMEM_RAW_MAINIMG:
	case MSM_PMEM_VIDEO_VPE:
		rc = msm_pmem_table_add(&sync->pmem_frames, pinfo);
		break;

	case MSM_PMEM_AEC_AWB:
	case MSM_PMEM_AF:
	case MSM_PMEM_AEC:
	case MSM_PMEM_AWB:
	case MSM_PMEM_RS:
	case MSM_PMEM_CS:
	case MSM_PMEM_IHIST:
	case MSM_PMEM_SKIN:
		rc = msm_pmem_table_add(&sync->pmem_stats, pinfo);
		break;

	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int msm_register_pmem(struct msm_sync *sync, void __user *arg)
{
	struct msm_pmem_info info;

	if (copy_from_user(&info, arg, sizeof(info))) {
		ERR_COPY_FROM_USER();
			return -EFAULT;
	}

	return __msm_register_pmem(sync, &info);
}

static int __msm_pmem_table_del(struct msm_sync *sync,
			struct msm_pmem_info *pinfo)
{
	int rc = 0;
	struct msm_pmem_region *region;
	struct hlist_node *node, *n;

	switch (pinfo->type) {
	case MSM_PMEM_VIDEO:
	case MSM_PMEM_PREVIEW:
	case MSM_PMEM_THUMBNAIL:
	case MSM_PMEM_MAINIMG:
	case MSM_PMEM_RAW_MAINIMG:
	case MSM_PMEM_VIDEO_VPE:
		hlist_for_each_entry_safe(region, node, n,
				&sync->pmem_frames, list) {

			if (pinfo->type == region->info.type &&
				pinfo->vaddr == region->info.vaddr &&
				pinfo->fd == region->info.fd) {
				hlist_del(node);
				put_pmem_file(region->file);
				kfree(region);
			}
		}
		break;

	case MSM_PMEM_AEC_AWB:
	case MSM_PMEM_AF:
		hlist_for_each_entry_safe(region, node, n,
				&sync->pmem_stats, list) {

			if (pinfo->type == region->info.type &&
				pinfo->vaddr == region->info.vaddr &&
				pinfo->fd == region->info.fd) {
				hlist_del(node);
				put_pmem_file(region->file);
				kfree(region);
			}
		}
		break;

	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int msm_pmem_table_del(struct msm_sync *sync, void __user *arg)
{
	struct msm_pmem_info info;

	if (copy_from_user(&info, arg, sizeof(info))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	return __msm_pmem_table_del(sync, &info);
}

static long msm_ioctl_common(struct msm_cam_v4l2_device *pmsm,
						unsigned int cmd,
						void __user *argp)
{
	D("%s\n", __func__);
	switch (cmd) {
	case MSM_CAM_IOCTL_REGISTER_PMEM:
		return msm_register_pmem(pmsm->sync, argp);
	case MSM_CAM_IOCTL_UNREGISTER_PMEM:
		return msm_pmem_table_del(pmsm->sync, argp);
	default:
		return -EINVAL;
	}
}

static int msm_get_sensor_info(struct msm_sync *sync, void __user *arg)
{
	int rc = 0;
	struct msm_camsensor_info info;
	struct msm_camera_sensor_info *sdata;

	if (copy_from_user(&info,
			arg,
			sizeof(struct msm_camsensor_info))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	sdata = sync->pdev->dev.platform_data;
	D("%s: sensor_name %s\n", __func__, sdata->sensor_name);

	memcpy(&info.name[0], sdata->sensor_name, MAX_SENSOR_NAME);
	info.flash_enabled = sdata->flash_data->flash_type !=
					MSM_CAMERA_FLASH_NONE;

	/* copy back to user space */
	if (copy_to_user((void *)arg,
				&info,
				sizeof(struct msm_camsensor_info))) {
		ERR_COPY_TO_USER();
		rc = -EFAULT;
	}

	return rc;
}

/* return of 0 means failure */
static uint8_t msm_pmem_region_lookup(struct hlist_head *ptype,
	int pmem_type, struct msm_pmem_region *reg, uint8_t maxcount)
{
	struct msm_pmem_region *region;
	struct msm_pmem_region *regptr;
	struct hlist_node *node, *n;

	uint8_t rc = 0;
	D("%s\n", __func__);
	regptr = reg;
	mutex_lock(&hlist_mut);
	hlist_for_each_entry_safe(region, node, n, ptype, list) {
		if (region->info.type == pmem_type && region->info.active) {
			*regptr = *region;
			rc += 1;
			if (rc >= maxcount)
				break;
			regptr++;
		}
	}
	D("%s finished, rc=%d\n", __func__, rc);
	mutex_unlock(&hlist_mut);
	return rc;
}

static uint8_t msm_pmem_region_lookup_2(struct hlist_head *ptype,
					int pmem_type,
					struct msm_pmem_region *reg,
					uint8_t maxcount)
{
	struct msm_pmem_region *region;
	struct msm_pmem_region *regptr;
	struct hlist_node *node, *n;
	uint8_t rc = 0;
	regptr = reg;
	mutex_lock(&hlist_mut);
	hlist_for_each_entry_safe(region, node, n, ptype, list) {
		D(KERN_ERR "Mio: info.type=%d, pmem_type = %d,"
						"info.active = %d\n",
		region->info.type, pmem_type, region->info.active);

		if (region->info.type == pmem_type && region->info.active) {
			D("info.type=%d, pmem_type = %d,"
							"info.active = %d,\n",
				region->info.type, pmem_type,
				region->info.active);
			*regptr = *region;
			region->info.type = MSM_PMEM_VIDEO;
			rc += 1;
			if (rc >= maxcount)
				break;
			regptr++;
		}
	}
	mutex_unlock(&hlist_mut);
	return rc;
}

static uint8_t msm_pmem_region_lookup_3(struct msm_cam_v4l2_device *pcam,
						struct msm_pmem_region *reg,
						uint8_t start_index,
						uint8_t stop_index)
{
	struct videobuf_contig_pmem *mem;
	int i;
	uint8_t rc = 0;

	mutex_lock(&hlist_mut);

	for (i = start_index; i < stop_index ; i++) {
		if ((pcam->vid_bufq).bufs[i] != NULL) {
			mem = ((pcam->vid_bufq).bufs[i])->priv;
			reg->paddr = mem->phyaddr;
			D("%s paddr for buf number %d is 0x%p\n", __func__, i,
							(void *)reg->paddr);
			reg->len = sizeof(struct msm_pmem_info);
			reg->file = NULL;
			reg->info.len = mem->size;

			reg->info.vaddr =
				(void *)(((pcam->vid_bufq).bufs[i])->baddr);

			if (i >= 0 && i <= 3)
				reg->info.type = MSM_PMEM_PREVIEW;
			else if (i >= 4 && i <= 7)
				reg->info.type = MSM_PMEM_VIDEO;

			reg->info.offset = 0;
			reg->info.y_off = mem->y_off;
			reg->info.cbcr_off = PAD_TO_WORD(mem->cbcr_off);
			D("%s y_off = %d, cbcr_off = %d\n", __func__,
				reg->info.y_off, reg->info.cbcr_off);
			rc += 1;
			reg++;
		}
	}

	mutex_unlock(&hlist_mut);
	D("%s returning rc= %d\n", __func__, rc);
	return rc;
}


static int msm_config_vfe(struct msm_sync *sync, void __user *arg)
{
	struct msm_vfe_cfg_cmd cfgcmd;
	struct msm_pmem_region region[8];
	struct axidata axi_data;

	if (!sync->vfefn.vfe_config) {
		pr_err("%s: no vfe_config!\n", __func__);
		return -EIO;
	}

	if (copy_from_user(&cfgcmd, arg, sizeof(cfgcmd))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	memset(&axi_data, 0, sizeof(axi_data));
	CDBG("%s: cmd_type %d\n", __func__, cfgcmd.cmd_type);
	switch (cfgcmd.cmd_type) {
	case CMD_STATS_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
					MSM_PMEM_AEC_AWB, &region[0],
					NUM_STAT_OUTPUT_BUFFERS);
		axi_data.bufnum2 =
			msm_pmem_region_lookup(&sync->pmem_stats,
					MSM_PMEM_AF, &region[axi_data.bufnum1],
					NUM_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1 || !axi_data.bufnum2) {
			pr_err("%s: pmem region lookup error\n", __func__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);
	case CMD_STATS_AF_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
					MSM_PMEM_AF, &region[0],
					NUM_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);
	case CMD_STATS_AEC_AWB_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
			MSM_PMEM_AEC_AWB, &region[0],
			NUM_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);
	case CMD_STATS_AEC_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
			MSM_PMEM_AEC, &region[0],
			NUM_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);
	case CMD_STATS_AWB_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
			MSM_PMEM_AWB, &region[0],
			NUM_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);


	case CMD_STATS_IHIST_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
			MSM_PMEM_IHIST, &region[0],
			NUM_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);

	case CMD_STATS_RS_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
			MSM_PMEM_RS, &region[0],
			NUM_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);

	case CMD_STATS_CS_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
			MSM_PMEM_CS, &region[0],
			NUM_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);

	case CMD_GENERAL:
	case CMD_STATS_DISABLE:
		return sync->vfefn.vfe_config(&cfgcmd, NULL);
	default:
		pr_err("%s: unknown command type %d\n",
			__func__, cfgcmd.cmd_type);
	}

	return -EINVAL;
}

static int msm_vpe_frame_cfg(struct msm_sync *sync,
				void *cfgcmdin)
{
	int rc = -EIO;
	struct axidata axi_data;
	void *data = &axi_data;
	struct msm_pmem_region region[8];
	int pmem_type;

	struct msm_vpe_cfg_cmd *cfgcmd;
	cfgcmd = (struct msm_vpe_cfg_cmd *)cfgcmdin;

	memset(&axi_data, 0, sizeof(axi_data));
	CDBG("In vpe_frame_cfg cfgcmd->cmd_type = %d\n",
		cfgcmd->cmd_type);
	switch (cfgcmd->cmd_type) {
	case CMD_AXI_CFG_VPE:
		pmem_type = MSM_PMEM_VIDEO_VPE;
		axi_data.bufnum1 =
			msm_pmem_region_lookup_2(&sync->pmem_frames, pmem_type,
								&region[0], 8);
		CDBG("axi_data.bufnum1 = %d\n", axi_data.bufnum1);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		pmem_type = MSM_PMEM_VIDEO;
		break;
	default:
		pr_err("%s: unknown command type %d\n",
			__func__, cfgcmd->cmd_type);
		break;
	}
	axi_data.region = &region[0];
	CDBG("out vpe_frame_cfg cfgcmd->cmd_type = %d\n",
		cfgcmd->cmd_type);
	/* send the AXI configuration command to driver */
	if (sync->vpefn.vpe_config)
		rc = sync->vpefn.vpe_config(cfgcmd, data);
	return rc;
}

static int msm_stats_axi_cfg(struct msm_sync *sync,
		struct msm_vfe_cfg_cmd *cfgcmd)
{
	int rc = -EIO;
	struct axidata axi_data;
	void *data = &axi_data;

	struct msm_pmem_region region[3];
	int pmem_type = MSM_PMEM_MAX;

	memset(&axi_data, 0, sizeof(axi_data));

	switch (cfgcmd->cmd_type) {
	case CMD_STATS_AXI_CFG:
		pmem_type = MSM_PMEM_AEC_AWB;
		break;
	case CMD_STATS_AF_AXI_CFG:
		pmem_type = MSM_PMEM_AF;
		break;
	case CMD_GENERAL:
		data = NULL;
		break;
	default:
		pr_err("%s: unknown command type %d\n",
			__func__, cfgcmd->cmd_type);
		return -EINVAL;
	}

	if (cfgcmd->cmd_type != CMD_GENERAL) {
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats, pmem_type,
				&region[0], NUM_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
	axi_data.region = &region[0];
	}

	/* send the AEC/AWB STATS configuration command to driver */
	if (sync->vfefn.vfe_config)
		rc = sync->vfefn.vfe_config(cfgcmd, &axi_data);

	return rc;
}

static int msm_frame_axi_cfg(struct msm_sync *sync,
		struct msm_vfe_cfg_cmd *cfgcmd)
{
	int rc = -EIO;
	struct axidata axi_data;
	void *data = &axi_data;
	struct msm_pmem_region region[8];
	int pmem_type;
	int i = 0;

	memset(&axi_data, 0, sizeof(axi_data));

	switch (cfgcmd->cmd_type) {

	case CMD_AXI_CFG_PREVIEW:
		axi_data.bufnum2 =
			msm_pmem_region_lookup_3(sync->pcam_sync,
				&region[0], 0, 4);
		if (!axi_data.bufnum2) {
			pr_err("%s %d: pmem region 3 lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		D("%s __func__ axi_data.bufnum2 = %d\n", __func__,
						axi_data.bufnum2);
		break;

	case CMD_AXI_CFG_VIDEO:
		pmem_type = MSM_PMEM_PREVIEW;
		axi_data.bufnum1 =
			msm_pmem_region_lookup_3(sync->pcam_sync,
				&region[0], 0, 3);
		D("%s bufnum1 = %d\n", __func__, axi_data.bufnum1);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}

		pmem_type = MSM_PMEM_VIDEO;
		axi_data.bufnum2 =
			msm_pmem_region_lookup_3(sync->pcam_sync,
				&region[axi_data.bufnum1],
				4, 7);
		D("%s bufnum2 = %d\n", __func__, axi_data.bufnum2);
		if (!axi_data.bufnum2) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		break;


	case CMD_AXI_CFG_SNAP:
		pmem_type = MSM_PMEM_THUMBNAIL;
		axi_data.bufnum1 =
			msm_pmem_region_lookup_3(sync->pcam_sync,
				&region[0], 0, 4);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}

		pmem_type = MSM_PMEM_MAINIMG;
		axi_data.bufnum2 =
		msm_pmem_region_lookup_3(sync->pcam_sync,
				&region[axi_data.bufnum1],
				axi_data.bufnum1, 8);
		if (!axi_data.bufnum2) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		break;

	case CMD_RAW_PICT_AXI_CFG:
		pmem_type = MSM_PMEM_RAW_MAINIMG;
		axi_data.bufnum2 =
			msm_pmem_region_lookup_3(sync->pcam_sync,
				&region[0], 0, 4);
		if (!axi_data.bufnum2) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		break;

	case CMD_GENERAL:
		data = NULL;
		break;

	default:
		pr_err("%s: unknown command type %d\n",
			__func__, cfgcmd->cmd_type);
		return -EINVAL;
	}

	axi_data.region = &region[0];
	D("%s bufnum1 = %d, bufnum2 = %d\n", __func__,
	  axi_data.bufnum1, axi_data.bufnum2);
	for (i = 0; i < 8; i++) {
		D("%s region %d paddr = 0x%p\n", __func__, i,
					(void *)region[i].paddr);
		D("%s region y_off = %d cbcr_off = %d\n", __func__,
			region[i].info.y_off, region[i].info.cbcr_off);
	}
	/* send the AXI configuration command to driver */
	if (sync->vfefn.vfe_config)
		rc = sync->vfefn.vfe_config(cfgcmd, data);

	return rc;
}

static int msm_axi_config(struct msm_sync *sync, void __user *arg)
{
	struct msm_vfe_cfg_cmd cfgcmd;

	if (copy_from_user(&cfgcmd, arg, sizeof(cfgcmd))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	switch (cfgcmd.cmd_type) {
	case CMD_AXI_CFG_VIDEO:
	case CMD_AXI_CFG_PREVIEW:
	case CMD_AXI_CFG_SNAP:
	case CMD_RAW_PICT_AXI_CFG:
		return msm_frame_axi_cfg(sync, &cfgcmd);
	case CMD_AXI_CFG_VPE:
		return 0;
		return msm_vpe_frame_cfg(sync, (void *)&cfgcmd);

	case CMD_STATS_AXI_CFG:
	case CMD_STATS_AF_AXI_CFG:
		return msm_stats_axi_cfg(sync, &cfgcmd);

	default:
		pr_err("%s: unknown command type %d\n",
			__func__,
			cfgcmd.cmd_type);
		return -EINVAL;
	}

	return 0;
}

static int msm_set_crop(struct msm_sync *sync, void __user *arg)
{
	struct crop_info crop;

	if (copy_from_user(&crop,
				arg,
				sizeof(struct crop_info))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	if (!sync->croplen) {
		sync->cropinfo = kmalloc(crop.len, GFP_KERNEL);
		if (!sync->cropinfo)
			return -ENOMEM;
	} else if (sync->croplen < crop.len)
		return -EINVAL;

	if (copy_from_user(sync->cropinfo,
				crop.info,
				crop.len)) {
		ERR_COPY_FROM_USER();
		kfree(sync->cropinfo);
		return -EFAULT;
	}

	sync->croplen = crop.len;

	return 0;
}

static int msm_put_stats_buffer(struct msm_sync *sync, void __user *arg)
{
	int rc = -EIO;

	struct msm_stats_buf buf;
	unsigned long pphy;
	struct msm_vfe_cfg_cmd cfgcmd;

	if (copy_from_user(&buf, arg,
				sizeof(struct msm_stats_buf))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	CDBG("%s\n", __func__);
	pphy = msm_pmem_stats_vtop_lookup(sync, buf.buffer, buf.fd);

	if (pphy != 0) {
		if (buf.type == STAT_AEAW)
			cfgcmd.cmd_type = CMD_STATS_BUF_RELEASE;
		else if (buf.type == STAT_AF)
			cfgcmd.cmd_type = CMD_STATS_AF_BUF_RELEASE;
		else if (buf.type == STAT_AEC)
			cfgcmd.cmd_type = CMD_STATS_AEC_BUF_RELEASE;
		else if (buf.type == STAT_AWB)
			cfgcmd.cmd_type = CMD_STATS_AWB_BUF_RELEASE;
		else if (buf.type == STAT_IHIST)
			cfgcmd.cmd_type = CMD_STATS_IHIST_BUF_RELEASE;
		else if (buf.type == STAT_RS)
			cfgcmd.cmd_type = CMD_STATS_RS_BUF_RELEASE;
		else if (buf.type == STAT_CS)
			cfgcmd.cmd_type = CMD_STATS_CS_BUF_RELEASE;

		else {
			pr_err("%s: invalid buf type %d\n",
				__func__,
				buf.type);
			rc = -EINVAL;
			goto put_done;
		}

		cfgcmd.value = (void *)&buf;

		if (sync->vfefn.vfe_config) {
			rc = sync->vfefn.vfe_config(&cfgcmd, &pphy);
			if (rc < 0)
				pr_err("%s: vfe_config error %d\n",
					__func__, rc);
		} else
			pr_err("%s: vfe_config is NULL\n", __func__);
	} else {
		pr_err("%s: NULL physical address\n", __func__);
		rc = -EINVAL;
	}

put_done:
	return rc;
}

/* config function simliar to origanl msm_ioctl_config*/
static int msm_isp_config(struct msm_cam_v4l2_device *pmsm, unsigned int cmd,
							unsigned long arg)
{
	int rc = -EINVAL;
	void __user *argp = (void __user *)arg;

	D("%s: cmd %d\n", __func__, _IOC_NR(cmd));
	switch (cmd) {
	case MSM_CAM_IOCTL_GET_SENSOR_INFO:
		rc = msm_get_sensor_info(pmsm->sync, argp);
		D("%s rc = %d\n", __func__, rc);
		break;

	case MSM_CAM_IOCTL_SENSOR_IO_CFG:
		rc = pmsm->sync->sctrl.s_config(argp);
		break;

	case MSM_CAM_IOCTL_PICT_PP_DONE:
		/* Release the preview of snapshot frame
		 * that was grabbed.
		 */
		/*rc = msm_pp_release(pmsm->sync, arg);*/
		break;

	case MSM_CAM_IOCTL_CONFIG_VFE:
		/* Coming from config thread for update */
		rc = msm_config_vfe(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_CONFIG_VPE:
		/* Coming from config thread for update */
		/*rc = msm_config_vpe(pmsm->sync, argp);*/
		rc = 0;
		break;

	case MSM_CAM_IOCTL_AXI_CONFIG:
	case MSM_CAM_IOCTL_AXI_VPE_CONFIG:
		D("Received MSM_CAM_IOCTL_AXI_CONFIG\n");
		rc = msm_axi_config(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_SET_CROP:
		rc = msm_set_crop(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_RELEASE_STATS_BUFFER:
		rc = msm_put_stats_buffer(pmsm->sync, argp);
		break;

	default:
		rc = msm_ioctl_common(pmsm, cmd, argp);
		break;
	}

	D("%s: cmd %d DONE\n", __func__, _IOC_NR(cmd));

	return rc;
}

int msm_isp_g_parm(struct msm_cam_v4l2_device *pcam, struct v4l2_streamparm *a)
{
	int rc = 0;

	D("%s: mode = %d\n", __func__, a->parm.capture.capturemode);
	D("%s: get mode returned %d\n", __func__,
		a->parm.capture.capturemode);
	return rc;
}

int msm_isp_s_parm(struct msm_cam_v4l2_device *pcam, struct v4l2_streamparm *a)
{
	int rc = 0;

	D("%s: mode = %d\n", __func__, a->parm.capture.capturemode);
	D("%s: mode set to %d\n", __func__, a->parm.capture.capturemode);
				pcam->mode = a->parm.capture.capturemode;
	return rc;
}

int msm_isp_try_fmt(struct msm_cam_v4l2_device *pcam, struct v4l2_format *pfmt)
{
	int rc = 0;
	int i = 0;
	struct v4l2_pix_format *pix = &pfmt->fmt.pix;
	struct v4l2_mbus_framefmt sensor_fmt;

	D("%s: 0x%x\n", __func__, pix->pixelformat);

	if (pfmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		D("%s: pfmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE!\n",
							__func__);
		return -EINVAL;
	}

	/* check if the format is supported by this host-sensor combo */
	for (i = 0; i < pcam->num_fmts; i++) {
		D("%s: usr_fmts.fourcc: 0x%x\n", __func__,
			pcam->usr_fmts[i].fourcc);
		if (pcam->usr_fmts[i].fourcc == pix->pixelformat)
			break;
	}

	if (i == pcam->num_fmts) {
		D("%s: Format %x not found\n", __func__, pix->pixelformat);
		return -EINVAL;
	}

	sensor_fmt.width  = pix->width;
	sensor_fmt.height = pix->height;
	sensor_fmt.field  = pix->field;
	sensor_fmt.colorspace = pix->colorspace;
	sensor_fmt.code   = pcam->usr_fmts[i].pxlcode;

	pix->width	= sensor_fmt.width;
	pix->height   = sensor_fmt.height;
	pix->field	= sensor_fmt.field;
	pix->colorspace   = sensor_fmt.colorspace;

	return rc;
}

int msm_isp_get_fmt(struct msm_cam_v4l2_device *pcam, struct v4l2_format *pfmt)
{
	struct v4l2_pix_format *pix = &pfmt->fmt.pix;

	pix->width	  = pcam->vid_fmt.fmt.pix.width;
	pix->height	 = pcam->vid_fmt.fmt.pix.height;
	pix->field	  = pcam->vid_fmt.fmt.pix.field;
	pix->pixelformat	= pcam->vid_fmt.fmt.pix.pixelformat;
	pix->bytesperline   = pcam->vid_fmt.fmt.pix.bytesperline;
	pix->colorspace	 = pcam->vid_fmt.fmt.pix.colorspace;
	if (pix->bytesperline < 0)
		return pix->bytesperline;

	pix->sizeimage	  = pix->height * pix->bytesperline;

	return 0;
}

int msm_isp_set_fmt(struct msm_cam_v4l2_device *pcam, struct v4l2_format *pfmt)
{
	int rc = 0;
	int i = 0;
	struct v4l2_pix_format *pix = &pfmt->fmt.pix;
	struct msm_ctrl_cmd ctrlcmd;

	D("%s: %d, %d, 0x%x\n", __func__,
		pfmt->fmt.pix.width, pfmt->fmt.pix.height,
		pfmt->fmt.pix.pixelformat);

	if (pfmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		D("%s, Attention! Wrong buf-type %d\n", __func__, pfmt->type);

	ctrlcmd.type = MSM_V4L2_VID_CAP_TYPE;
	ctrlcmd.length = 0;
	ctrlcmd.value = NULL;
	ctrlcmd.timeout_ms = 10000;

	/* send command to config thread in usersspace, and get return value */
	rc = msm_isp_control(pcam, &ctrlcmd);

	if (rc >= 0) {

		for (i = 0; i < pcam->num_fmts; i++)
			if (pcam->usr_fmts[i].fourcc == pix->pixelformat)
				break;

		pcam->vid_fmt.fmt.pix.width    = pix->width;
		pcam->vid_fmt.fmt.pix.height   = pix->height;
		pcam->vid_fmt.fmt.pix.field    = pix->field;
		pcam->vid_fmt.fmt.pix.pixelformat = pcam->usr_fmts[i].fourcc;
		pcam->vid_fmt.fmt.pix.bytesperline = (pix->width *
				pcam->usr_fmts[i].bitsperpxl) / 8;
		pcam->vid_bufq.field   = pix->field;
		pcam->sensor_pxlcode  = pcam->usr_fmts[i].pxlcode;
	}

	return rc;
}

/*
 *  Videobuf operations
 */

static struct videobuf_queue_ops msm_vidbuf_ops = {
	.buf_setup  = msm_vidbuf_setup,
	.buf_prepare  = msm_vidbuf_prepare,
	.buf_queue  = msm_vidbuf_queue,
	.buf_release  = msm_vidbuf_release,
};

static int msm_isp_init_vidbuf(struct videobuf_queue *q,
			struct msm_cam_v4l2_device *pcam)
{
	int rc = 0;
	struct resource *res;

	D("%s\n", __func__);
	/* first check if we have resources */
	res = platform_get_resource(pcam->pdev, IORESOURCE_DMA, 0);
	if (res) {
		D("res->start = 0x%x\n", (u32)res->start);
		D("res->size = 0x%x\n", (u32)resource_size(res));
		D("res->end = 0x%x\n", (u32)res->end);
		rc = dma_declare_coherent_memory(&pcam->pdev->dev, res->start,
			res->start,
			resource_size(res),
			DMA_MEMORY_MAP |
			DMA_MEMORY_EXCLUSIVE);
		if (!rc) {
			D("%s: Unable to declare coherent memory.\n", __func__);
			rc = -ENXIO;
			return rc;
		}

		pcam->memsize = resource_size(res);
		D("%s: found DMA capable resource\n", __func__);
	} else {
		D("%s: no DMA capable resource\n", __func__);
		return -ENOMEM;
	}
	videobuf_queue_pmem_contig_init(q, &msm_vidbuf_ops, &pcam->pdev->dev,
		&pcam->vb_irqlock,
		V4L2_BUF_TYPE_VIDEO_CAPTURE,
		V4L2_FIELD_NONE,
		sizeof(struct msm_frame_buffer), pcam);
	return 0;
}

int msm_isp_streamon(struct msm_cam_v4l2_device *pcam)
{
	int rc = 0;
	struct msm_ctrl_cmd ctrlcmd;
	D("%s\n", __func__);
	ctrlcmd.type	   = MSM_V4L2_STREAM_ON;
	ctrlcmd.timeout_ms = 10000;
	ctrlcmd.length	 = 0;
	ctrlcmd.value	  = NULL;

	/* send command to config thread in usersspace, and get return value */
	rc = msm_isp_control(pcam, &ctrlcmd);

	return rc;
}

int msm_isp_streamoff(struct msm_cam_v4l2_device *pcam)
{
	int rc = 0;
	struct msm_ctrl_cmd ctrlcmd;

	D("%s, pcam = 0x%x\n", __func__, (u32)pcam);
	ctrlcmd.type	   = MSM_V4L2_STREAM_OFF;
	ctrlcmd.timeout_ms = 10000;
	ctrlcmd.length	 = 0;
	ctrlcmd.value	  = NULL;

	/* send command to config thread in usersspace, and get return value */
	rc = msm_isp_control(pcam, &ctrlcmd);

	return rc;
}

/* Init a msm device for ISP control,
   which will create a video device (/dev/video0/ and plug in
   ISP's operation "v4l2_ioctl_ops*"
*/
int msm_isp_init_user_formats(struct msm_cam_v4l2_device *pcam)
{
	struct v4l2_subdev *sd = pcam->isp.sdev;
	enum v4l2_mbus_pixelcode pxlcode;
	int numfmt = 0;
	int rc = 0;
	int i, j;

	D("%s\n", __func__);
	while (!v4l2_subdev_call(sd, video, enum_mbus_fmt, numfmt, &pxlcode))
		numfmt++;

	D("%s, numfmt = %d\n", __func__, numfmt);
	if (!numfmt)
		return -ENXIO;

	pcam->usr_fmts =
		vmalloc(numfmt * sizeof(struct msm_isp_color_fmt));
	if (!pcam->usr_fmts)
		return -ENOMEM;

	pcam->num_fmts = numfmt;

	D("Found %d supported formats.\n", pcam->num_fmts);

	/* from sensor to ISP.. fill the data structure */
	for (i = 0; i < numfmt; i++) {
		rc = v4l2_subdev_call(sd, video, enum_mbus_fmt, i, &pxlcode);
		D("rc is  %d\n", rc);
		if (rc < 0) {
			vfree(pcam->usr_fmts);
			return rc;
		}

		for (j = 0; j < ARRAY_SIZE(msm_isp_formats); j++) {
			/* find the corresponding format */
			if (pxlcode == msm_isp_formats[j].pxlcode) {
				pcam->usr_fmts[i] = msm_isp_formats[j];
				D("pcam->usr_fmts=0x%x\n", (u32)pcam->usr_fmts);
				D("format pxlcode 0x%x (0x%x) found\n",
					  pcam->usr_fmts[i].pxlcode,
					  pcam->usr_fmts[i].fourcc);
				break;
			}
		}
		if (j == ARRAY_SIZE(msm_isp_formats)) {
			D("format pxlcode 0x%x not found\n", pxlcode);
			vfree(pcam->usr_fmts);
			return -EINVAL;
		}
	}

	/* set the default pxlcode, in any case, it will be set through
	 * setfmt */
	pcam->sensor_pxlcode = pcam->usr_fmts[0].pxlcode;

	return 0;
}

/* Create a isp control device for a sensor,
   which will then register into the main device msm
*/
int msm_isp_register(struct msm_cam_v4l2_device *pcam)
{
	int rc = -EINVAL;

	D("%s pcam = %x\n", __func__, (unsigned int)pcam);

	if (!pcam)
		return rc;


	pcam->isp.isp_init = msm_isp_init;
	pcam->isp.isp_init_vidbuf = msm_isp_init_vidbuf;
	pcam->isp.isp_config = msm_isp_config;
	pcam->isp.isp_release  = msm_isp_release;
	pcam->isp.isp_enqueue = msm_isp_enqueue;

	D("%s\n", __func__);

	return 0;
}
EXPORT_SYMBOL(msm_isp_register);
