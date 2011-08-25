/*
 *  include/linux/msm_v4l2_video.h
 *
 *  Copyright (C) 2008 Palm Inc,
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Authors: Kevin McCray (kevin.mccray@palm.com)
 *
 */

#ifndef MSMV4L2DEF_H
#define MSMV4L2DEF_H

#define PMEM_FREE		0
#define PMEM_ALLOCATED 		1

#define VIDIOC_S_MSM_ROTATION		_IOW ('V', 1,  int)
#define VIDIOC_G_MSM_ROTATION		_IOR ('V', 2,  int)
#define VIDIOC_S_MSM_FB_INFO		_IOW ('V', 3,  int)
#define VIDIOC_G_MSM_FB_INFO		_IOW ('V', 4,  int)
#define VIDIOC_G_LAST_FRAME		_IOWR ('V', 5,  struct v4l2_last_frame) 

#define MSMV4L2_LAYER0 0
#define MSMV4L2_LAYER1 1

struct pmem_device {
	struct file *filp;
	struct pmem_region region;
	int allocated;
};

// msmv4l2 device data
struct msmv4l2_device {
	struct device dev;
	int fb_fd;
	struct file *fb_fp;

	int ref_count;
	int id;

	int screen_width;
	int screen_height;
	int streaming;

	struct v4l2_pix_format pix;
	struct v4l2_window win;
	struct v4l2_rect crop_rect;

	struct mdp_blit_int_req req;

	struct mutex update_lock;
};

// Context data. Every open instance of the device has its own.
struct msmv4l2_fh {
	struct msmv4l2_device *vout;
	enum v4l2_buf_type type;
};

struct v4l2_last_frame {
	int screen_width;
	int screen_height;
	int size;
	void * buffer;
};

#endif //MSMV4L2DEF_H

