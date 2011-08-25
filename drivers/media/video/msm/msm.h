/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _MSM_H
#define _MSM_H

#ifdef __KERNEL__

/* Header files */
#include <linux/i2c.h>
#include <linux/videodev2.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-event.h>
#include <media/v4l2-mediabus.h>
#include <media/videobuf-dma-contig.h>
#include <media/videobuf-pmem.h>
#include <mach/camera.h>

#define MSM_V4L2_DIMENSION_SIZE 84
enum isp_vfe_cmd_id {
	/*
	*Important! Command_ID are arranged in order.
	*Don't change!*/
	ISP_VFE_CMD_ID_STREAM_ON,
	ISP_VFE_CMD_ID_STREAM_OFF,
	ISP_VFE_CMD_ID_FRAME_BUF_RELEASE
};

struct msm_cam_v4l2_device;

/* buffer for one video frame */
struct msm_frame_buffer {
	/* common v4l buffer stuff -- must be first */
	struct videobuf_buffer    vidbuf;
	enum v4l2_mbus_pixelcode  pxlcode;
	int                       inuse;
	int                       active;
};

struct msm_isp_color_fmt {
	char *name;
	int depth;
	int bitsperpxl;
	u32 fourcc;
	enum v4l2_mbus_pixelcode pxlcode;
	enum v4l2_colorspace colorspace;
};

/* abstract camera device represents a VFE and connected sensor */
struct msm_isp_ops {
	int (*isp_init)(struct msm_cam_v4l2_device *pcam);
	int (*isp_config)(struct msm_cam_v4l2_device *pcam, unsigned int cmd,
		unsigned long arg);
	int (*isp_enqueue)(struct msm_cam_v4l2_device *pcam,
		struct msm_vfe_resp *data,
		enum msm_queue qtype);

	int (*isp_init_vidbuf)(struct videobuf_queue *,
		struct msm_cam_v4l2_device *);
	int (*isp_release)(struct msm_cam_v4l2_device *pcam);

	/* vfe subdevice */
	struct v4l2_subdev vdev;
	/* sensor subdevice */
	struct v4l2_subdev *sdev;
};

struct msm_isp_buf_info {
	int type;
	unsigned long buffer;
	int fd;
};

/* abstract camera device for each sensor successfully probed*/
struct msm_cam_v4l2_device {
	/* standard device interfaces */
	/* parent of video device to trace back */
	struct device dev;
	/* sensor's platform device*/
	struct platform_device *pdev;
	/* V4l2 device */
	struct v4l2_device v4l2_dev;
	struct v4l2_fh  eventHandle;
	int eventQueueSize;

	/* will be registered as /dev/video*/
	struct video_device *pvdev;
	int use_count;
	/* will be used to init/release HW */
	struct msm_isp_ops isp;

	/* parent device */
	struct device *parent_dev;

	struct mutex vid_lock;

	/* Buffer queue used in video-buf */
	struct videobuf_queue vid_bufq;
	/* Current vid buf format */
	struct v4l2_format vid_fmt;
	/* senssor pixel code*/
	enum v4l2_mbus_pixelcode sensor_pxlcode;

	/* Queue of filled frames */
	struct list_head framequeue;
	/* lock for video buffers and queue */
	spinlock_t vb_irqlock;

	struct msm_frame_buffer *activebuf;

	/* v4l2 format support */
	struct msm_isp_color_fmt *usr_fmts;
	int num_fmts;
	/* preview or snapshot */
	u32 mode;
	u32 memsize;

	/* native config device */
	struct cdev cdev;

	/* most-frequently accessed manager object*/
	struct msm_sync *sync;

	/* The message queue is used by the control thread to send commands
	 * to the config thread, and also by the HW to send messages to the
	 * config thread.  Thus it is the only queue that is accessed from
	 * both interrupt and process context.
	 */
	/* struct msm_device_queue event_q; */

	/* This queue used by the config thread to send responses back to the
	 * control thread.  It is accessed only from a process context.
	 */
	struct msm_device_queue ctrl_q;
};

/* ISP related functions */
int msm_isp_g_parm(struct msm_cam_v4l2_device *pcam, struct v4l2_streamparm *a);
int msm_isp_s_parm(struct msm_cam_v4l2_device *pcam, struct v4l2_streamparm *a);
int msm_isp_try_fmt(struct msm_cam_v4l2_device *pcam, struct v4l2_format *pfmt);
int msm_isp_get_fmt(struct msm_cam_v4l2_device *pcam, struct v4l2_format *pfmt);
int msm_isp_set_fmt(struct msm_cam_v4l2_device *pcam, struct v4l2_format *pfmt);
int msm_isp_init_user_formats(struct msm_cam_v4l2_device *pcam);
int msm_isp_streamon(struct msm_cam_v4l2_device *pcam);
int msm_isp_streamoff(struct msm_cam_v4l2_device *pcam);

void msm_isp_vfe_dev_init(struct v4l2_subdev *vd);
int msm_isp_register(struct msm_cam_v4l2_device *pcam);
int msm_sensor_register(struct platform_device *pdev,
	int (*sensor_probe)(const struct msm_camera_sensor_info *,
	struct v4l2_subdev **, struct msm_sensor_ctrl *));


#endif /* __KERNEL__ */

#endif /* _MSM_H */
