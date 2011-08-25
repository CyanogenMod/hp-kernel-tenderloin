/*
 *  linux/drivers/media/video/msm/msm_v4l2_video.c - MSM V4L2 Video Out
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
 * Author: Kevin McCray (kevin.mccray@palm.com)
 * Author: Jacky Cheung (jacky.cheung@palm.com)
 *
 */

#include <linux/init.h>
#include <linux/fb.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/time.h>
#include <linux/videodev2.h>
#include <linux/platform_device.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/msm_mdp.h>
#include <linux/android_pmem.h>

#include <media/v4l2-ioctl.h>
#include <media/v4l2-dev.h>
#include <media/videobuf-dma-sg.h>

#include <mach/board.h>
#include <mach/msm_fb.h>

#include <linux/msm_v4l2_video.h>

#define MSM_VIDEO -1

#define MSMV4L2_DEBUG_MSGS 0
#if MSMV4L2_DEBUG_MSGS
#define DLOG(fmt,args...) \
	do { printk(KERN_INFO "[%s:%s:%d] "fmt, __FILE__, __func__, __LINE__, \
		    ##args); } \
	while (0)
#else
#define DLOG(x...) do {} while (0)
#endif

#define  MSMV4L2_DEBUG_PROFILING   0
#if MSMV4L2_DEBUG_PROFILING
static suseconds_t msmv4l2_stTime, msmv4l2_cbst;
#define MSMV4L2_PROFILE_START \
	{ \
		struct timeval msmv4l2_time; \
		do_gettimeofday(&msmv4l2_time);\
		msmv4l2_stTime = msmv4l2_time.tv_usec; \
	}

#define MSMV4L2_PROFILE_END(TEXT) \
	{ \
		unsigned long x; \
		struct timeval msmv4l2_time; \
		do_gettimeofday(&msmv4l2_time);\
		x = msmv4l2_time.tv_usec - msmv4l2_stTime; \
		printk(KERN_INFO "%s  %lu ms\n", TEXT, x/1000);\
		msmv4l2_stTime = msmv4l2_time.tv_usec; \
	}
#else
#define MSMV4L2_PROFILE_START
#define MSMV4L2_PROFILE_END(TEXT)
#endif

static struct msmv4l2_device 	*saved_vout0;
static struct msmv4l2_device 	*saved_vout1;

struct mutex msmfb_lock;

static int msmv4l2_startstreaming(struct msmv4l2_device *vout)
{
	mutex_lock(&msmfb_lock);
	msm_fb_v4l2_enable(true, 
		(vout->id == 0)? MSMV4L2_LAYER0 : MSMV4L2_LAYER1);
	mutex_unlock(&msmfb_lock);

	vout->streaming = 1;

	return 0;
}

static int msmv4l2_stopstreaming(struct msmv4l2_device *vout)
{
	if (!vout->streaming)
		return 0;

	mutex_lock(&msmfb_lock);
	//Enable msm_fb to switch the video layer off
	msm_fb_v4l2_enable(false, 
		(vout->id == 0)? MSMV4L2_LAYER0 : MSMV4L2_LAYER1);
	mutex_unlock(&msmfb_lock);

	vout->streaming = 0;

	return 0;
}

static int msmv4l2_mapformat(uint32_t pixelformat)
{
	int mdp_format;

	switch(pixelformat) {
		case V4L2_PIX_FMT_RGB565:
			mdp_format = MDP_RGB_565;
			break;
		case V4L2_PIX_FMT_RGB32:
			mdp_format = MDP_ARGB_8888;
			break;
		case V4L2_PIX_FMT_RGB24:
			mdp_format = MDP_RGB_888;
			break;
		case V4L2_PIX_FMT_NV12:
			mdp_format = MDP_Y_CRCB_H2V2;
			break;
		case V4L2_PIX_FMT_NV21:
			mdp_format = MDP_Y_CBCR_H2V2;
			break;
		case V4L2_PIX_FMT_NV12_TILE:
			mdp_format = MDP_Y_CRCB_H2V2_TILE;
			break;
		case V4L2_PIX_FMT_NV21_TILE:
			mdp_format = MDP_Y_CBCR_H2V2_TILE;
			break;
		default:	
			mdp_format = MDP_Y_CBCR_H2V2;	
			break;
	}	

	return mdp_format;
}

static int
msmv4l2_get_buffer_mem_offset(struct v4l2_buffer *buffer, unsigned long* offset)
{
	int rc = 0;

	struct vm_area_struct *vma;

	down_read(&current->mm->mmap_sem);

	if (!(vma = find_vma(current->mm, buffer->m.userptr))
		|| (vma->vm_start > buffer->m.userptr)
		|| (vma->vm_end < buffer->m.userptr)) {
		rc = -EINVAL;
		goto exit;
	}

	*offset = buffer->m.userptr - vma->vm_start;

	DLOG("userptr=0x%X vm_start=0x%X vm_end=0x%X -> offset=0x%X\n",
		(unsigned int)buffer->m.userptr, 
		(unsigned int)vma->vm_start, 
		(unsigned int)vma->vm_end, 
		(unsigned int)*offset);

exit:
 	up_read(&current->mm->mmap_sem);

	return rc;
}

static int 
msmv4l2_fb_update(struct msmv4l2_device *vout, struct v4l2_buffer *buffer)
{
	int ret;
	int put_needed;	
	unsigned long offset=0;
	struct file *f;

	MSMV4L2_PROFILE_START;

	memset (&vout->req, 0, sizeof(struct mdp_blit_int_req));

	//Src dimensions
	vout->req.src.width = vout->pix.width;
	vout->req.src.height = vout->pix.height;

	//Crop of the Src 
	vout->req.src_rect.x = vout->crop_rect.left;
	vout->req.src_rect.y = vout->crop_rect.top;
	vout->req.src_rect.w = vout->crop_rect.width;
	vout->req.src_rect.h = vout->crop_rect.height;

	vout->req.src.format = msmv4l2_mapformat(vout->pix.pixelformat);
	msmv4l2_get_buffer_mem_offset(buffer, &offset);
	vout->req.src.offset = offset;

	f = fget_light(buffer->reserved, &put_needed);
	vout->req.src.filp = f;
	fput_light(f, put_needed);

	//Dest size (Screen size)
	vout->req.dst.width = vout->screen_width;
	vout->req.dst.height = vout->screen_height; 

	vout->req.dst_rect.x = vout->win.w.left;
	vout->req.dst_rect.y = vout->win.w.top;
	vout->req.dst_rect.w = vout->win.w.width;
	vout->req.dst_rect.h = vout->win.w.height;

	vout->req.alpha = MDP_ALPHA_NOP;
	vout->req.transp_mask = MDP_TRANSP_NOP;
	vout->req.flags = MDP_ROT_NOP;

	DLOG("SRC: width=%d, height=%d, filp=0x%x offset=0x%x\n", 
		vout->req.src.width, vout->req.src.height, 
		(unsigned int)vout->req.src.filp, vout->req.src.offset);
	DLOG("DST: width=%d, height=%d, filp=0x%x\n", 
		vout->req.dst.width, vout->req.dst.height, 
		(unsigned int)vout->req.dst.filp);


	/*
	 * Push frame to msmfb. 
	 */
	mutex_lock(&msmfb_lock);

	ret = msm_fb_v4l2_update(&vout->req, 
		(vout->id == 0)? MSMV4L2_LAYER0 : MSMV4L2_LAYER1);

	mutex_unlock(&msmfb_lock);

	MSMV4L2_PROFILE_END("Time taken for pass 1: ");

	return ret;
}

static int 
msmv4l2_vidioc_qbuf(struct file *file, struct msmv4l2_fh* fh, void *arg)
{
	struct msmv4l2_device *vout = fh->vout;
	struct v4l2_buffer *buffer = (struct v4l2_buffer *) arg;
	int ret;

	DLOG("VIDIOC_QBUF: buffer=0x%X, fd=%d\n", 
		(unsigned int)buffer->m.userptr, buffer->reserved);

	if(!vout->streaming) {
		printk(KERN_ERR "msmv4l2: Video Stream not enabled\n");
		return -EINVAL;
	}

	DLOG("VIDIOC_QBUF: buffer=0x%X, fd=%d\n", 
		(unsigned int)buffer->m.userptr, buffer->reserved);

	ret =  msmv4l2_fb_update( vout, buffer );

	return 0;
}


static long 
msmv4l2_do_ioctl (struct file *file,
		       unsigned int cmd, void *arg)
{
	struct msmv4l2_fh *fh = (struct msmv4l2_fh *)file->private_data;
	struct msmv4l2_device *vout = fh->vout;
	int ret;

	DLOG("msmv4l2_do_ioctl: cmd=0x%x\n", cmd);

	switch (cmd){
	case VIDIOC_REQBUFS:
		DLOG("VIDIOC_REQBUFS\n");
		break;

	case VIDIOC_QBUF:
		mutex_lock(&vout->update_lock);
		ret = msmv4l2_vidioc_qbuf(file, fh, arg);
		mutex_unlock(&vout->update_lock);

		return ret;
	
	case VIDIOC_DQBUF:
		DLOG("VIDIOC_DQBUF\n");
		break;

	case VIDIOC_S_FMT: {
		struct v4l2_format *f = (struct v4l2_format *) arg;
		DLOG("VIDIOC_S_FMT\n");

#ifdef CONFIG_FB_MSM_MDP30
		//Only needed for 7x27 to make sure the userspace to 
		//blank the screen before reformatting
		if(vout->streaming)
			return -EBUSY;
#endif

		switch(f->type){
			case V4L2_BUF_TYPE_VIDEO_OVERLAY: {

				mutex_lock(&vout->update_lock);
				memcpy(&vout->win, &f->fmt.win, sizeof(struct v4l2_window));
				mutex_unlock(&vout->update_lock);

				DLOG("Overlay Window: left=%d, top=%d, width=%d, height=%d\n",
					vout->win.w.left, vout->win.w.top, vout->win.w.width,
					vout->win.w.height);

				break;
			}
			case V4L2_BUF_TYPE_VIDEO_OUTPUT: {				

				mutex_lock(&vout->update_lock);
				memcpy(&vout->pix, &f->fmt.pix, sizeof(struct v4l2_pix_format));
				mutex_unlock(&vout->update_lock);

				DLOG("Pixel Data: format=%d, width=%d, height=%d\n",
					vout->pix.pixelformat, vout->pix.width, vout->pix.height);
		
				break;
			}
			default:
				return -EINVAL;
		}
		break;
	}
	case VIDIOC_G_FMT: {
		struct v4l2_format *f = (struct v4l2_format *) arg;
		DLOG("VIDIOC_G_FMT\n");

		switch (f->type){
			case V4L2_BUF_TYPE_VIDEO_OUTPUT: {
				struct v4l2_pix_format *pix = &f->fmt.pix;
				memset (pix, 0, sizeof (*pix));
				*pix = vout->pix;
				break;
			}

			case V4L2_BUF_TYPE_VIDEO_OVERLAY: {
				struct v4l2_window *win = &f->fmt.win;
				memset (win, 0, sizeof (*win));
				win->w = vout->win.w;
				break;
			}
			default:
				return -EINVAL;
		}
		break;
	}

	case VIDIOC_S_CROP: {
		struct v4l2_crop *crop = (struct v4l2_crop *) arg;
		DLOG("VIDIOC_S_CROP\n");

		switch(crop->type) {

			case V4L2_BUF_TYPE_VIDEO_OUTPUT:

				mutex_lock(&vout->update_lock);
				memcpy(&vout->crop_rect, &crop->c, sizeof(struct v4l2_rect));
				mutex_unlock(&vout->update_lock);

				DLOG("New Crop Rect: left=%d, top=%d, width=%d, height=%d\n",
					vout->crop_rect.left, vout->crop_rect.top, 
					vout->crop_rect.width, vout->crop_rect.height);
				break;

			default:

				return -EINVAL;
		}
		break;
	}
	case VIDIOC_G_CROP: {
		struct v4l2_crop *crop = (struct v4l2_crop *) arg;
		DLOG("VIDIOC_G_CROP\n");
		
		switch (crop->type) {

			case V4L2_BUF_TYPE_VIDEO_OUTPUT:
				memcpy(&crop->c, &vout->crop_rect, sizeof(struct v4l2_rect));
				break;

			default:
				return -EINVAL;
		}
		break;
	}
	case VIDIOC_STREAMON: {
		DLOG("VIDIOC_STREAMON\n");

		if(vout->streaming) {
			printk(KERN_ERR "msmv4l2: VIDIOC_STREAMON: already streaming.\n");
			return -EBUSY;
		}
		
		mutex_lock(&vout->update_lock);
		msmv4l2_startstreaming(vout);
		mutex_unlock(&vout->update_lock);

		break;
	}

	case VIDIOC_STREAMOFF: {
		DLOG("VIDIOC_STREAMOFF\n");

		if(!vout->streaming) {
			printk(KERN_ERR "msmv4l2: VIDIOC_STREAMOFF: Invalid IOCTL request.\n");
			return -EINVAL;
		}

		mutex_lock(&vout->update_lock);
		msmv4l2_stopstreaming( vout );
		mutex_unlock(&vout->update_lock);

		break;
	}

	case VIDIOC_S_MSM_ROTATION: { 

		printk(KERN_ERR "msmv4l2: VIDIOC_S_MSM_ROTATION not supported\n"); 
		break;
	}

	case VIDIOC_G_MSM_ROTATION: {

		printk(KERN_ERR "msmv4l2: VIDIOC_G_MSM_ROTATION not supported\n"); 
		break;
	}

	case VIDIOC_S_MSM_FB_INFO: {
		DLOG("VIDIOC_S_MSM_FB_INFO deprecated\n");
	}
	break;
	case VIDIOC_G_MSM_FB_INFO: {
		int *fd = arg;
		DLOG("VIDIOC_G_MSM_FB_INFO deprecated\n");
		*fd = -1;
	}
	break;

	case VIDIOC_G_LAST_FRAME: {
		printk(KERN_ERR "msmv4l2: VIDIOC_G_LAST_FRAME: Not implemented\n");
		return -EFAULT;
	}
	break;

	default:
		DLOG("Unrecognized IOCTL\n");
		return -ENOIOCTLCMD;

	}/* switch */

	return 0;
}

static long 
msmv4l2_ioctl (struct file *file, unsigned int cmd,
		    unsigned long arg)
{
	return video_usercopy (file, cmd, arg, msmv4l2_do_ioctl);
}

int
msmv4l2_release(struct file *file)
{
	struct msmv4l2_fh *fh = file->private_data;
	struct msmv4l2_device *vout = fh->vout;

	if(vout->streaming)
 		msmv4l2_stopstreaming( vout );

	vout->ref_count--;

	kfree(fh);

	return 0;
}

int
msmv4l2_open(struct file *file)
{
	struct msmv4l2_device 	*vout = 0;
	struct v4l2_pix_format	*pix = 0;
	struct msmv4l2_fh *fh;

	struct inode *inode = file->f_path.dentry->d_inode;
	int video_idx = iminor(inode);

	DLOG("msmv4l2_open+++ %d\n", video_idx);

	if(video_idx == 0) {
		vout = saved_vout0;
		vout->id = 0;
	}
	else if(video_idx == 1) {
		vout = saved_vout1;
		vout->id = 1;
	}

	if (!vout) {
		printk (KERN_ERR "msmv4l2_open: no context\n");
		return -EBUSY;
	}

	if (vout->ref_count) {
		printk (KERN_ERR "msmv4l2_open: multiple open currently is not supported!\n");
		return -EBUSY;
	}

	// Increment reference count
	vout->ref_count++;

	/* allocate per-filehandle data */
	fh = kmalloc (sizeof(struct msmv4l2_fh), GFP_KERNEL);
	if (NULL == fh) {
		printk (KERN_ERR "msmv4l2_open: kmalloc failed\n");
		return -ENOMEM;
	}

	fh->vout = vout;
	fh->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;

	if(!file)
		printk("Empty file!!\n");
	file->private_data = fh;

	vout->streaming 	= 0;
	vout->crop_rect.left 	= vout->crop_rect.top = 0;
	vout->crop_rect.width 	= vout->screen_width;
	vout->crop_rect.height 	= vout->screen_height;

	pix 				= &vout->pix;
	pix->width 			= vout->screen_width;
	pix->height 		= vout->screen_height;
	pix->pixelformat 	= V4L2_PIX_FMT_RGB32;
	pix->field 			= V4L2_FIELD_NONE;
	pix->bytesperline 	= pix->width * 4;
	pix->sizeimage 		= pix->bytesperline * pix->height;
	pix->priv 			= 0;
	pix->colorspace 	= V4L2_COLORSPACE_SRGB;

	vout->win.w.left 	= 0;
	vout->win.w.top 	= 0;
	vout->win.w.width 	= vout->screen_width;
	vout->win.w.height 	= vout->screen_height;

	mutex_init(&vout->update_lock);

	return 0;
}

static int 
msmv4l2_probe(struct platform_device *pdev)
{	
	//struct msmv4l2_device	*vout; 	
	//struct msm_v4l2_pd		*pi = pdev->dev.platform_data;

	//TODO: Specify certain restrictions in board file 
	DLOG("msmv4l2_probe+++\n");


	return 0;
}

static int msmv4l2_remove(struct platform_device *pdev)
{
	DLOG("msmv4l2_remove+++\n");

	return 0;
}

void msmv4l2_videodev_release(struct video_device *vfd)
{
	DLOG("msmv4l2_videodev_release+++\n");

	return;
}

static const struct v4l2_file_operations msmv4l2_fops = {
	.owner		= THIS_MODULE,
	.open		= msmv4l2_open,
	.release	= msmv4l2_release,
	.ioctl		= msmv4l2_ioctl,
};

static struct video_device msmv4l2_vid_device0 = {
	.name		= "msmv4l2",
	//.vfl_type		= VFL_TYPE_GRABBER, 
	.fops       = &msmv4l2_fops,
	.minor		= -1,
	.release	= msmv4l2_videodev_release,
};

static struct video_device msmv4l2_vid_device1 = {
	.name		= "msmv4l2",
	//.vfl_type		= VFL_TYPE_GRABBER, 
	.fops       = &msmv4l2_fops,
	.minor		= -1,
	.release	= msmv4l2_videodev_release,
};

static struct platform_driver msmv4l2_platform_driver = {
	.probe   = msmv4l2_probe,
	.remove  = msmv4l2_remove,
	.driver  = {
			 .name = "msmv4l2_pd",
		   },
};

static int __init
msmv4l2_init (void)
{
	int ret = 0;

	DLOG("msmv4l2_init+++\n");

	saved_vout0 = kmalloc (sizeof (struct msmv4l2_device), GFP_KERNEL);
	saved_vout1 = kmalloc (sizeof (struct msmv4l2_device), GFP_KERNEL);

	if (!saved_vout0 || !saved_vout1){
		printk (KERN_ERR "msmv4l2: kmalloc failed\n");
		goto end;
	}

	memset (saved_vout0, 0, sizeof (struct msmv4l2_device));
	memset (saved_vout1, 0, sizeof (struct msmv4l2_device));

	ret = platform_driver_register(&msmv4l2_platform_driver);
	if(ret < 0) {
		printk (KERN_ERR "msmv4l2: platform_driver_register failed\n");
		goto end;
	}

	//Register the device with videodev. 
	//Videodev will make IOCTL calls on application requests
	ret = video_register_device (&msmv4l2_vid_device0, VFL_TYPE_GRABBER, MSM_VIDEO);
	if (ret < 0) {
		printk (KERN_ERR "msmv4l2: could not register Video for Linux device 1\n");
		goto end_unregister;
	}	

	ret = video_register_device (&msmv4l2_vid_device1, VFL_TYPE_GRABBER, MSM_VIDEO);
	if (ret < 0) {
		printk (KERN_ERR "msmv4l2: could not register Video for Linux device 2\n");
		goto end_unregister;
	}	

	mutex_init(&msmfb_lock);
	
	return 0;

end_unregister:
	platform_driver_unregister(&msmv4l2_platform_driver);

end:
	kfree(saved_vout0);
	kfree(saved_vout1);
	saved_vout0 = NULL;
	saved_vout1 = NULL;
	return ret;
}

static void
msmv4l2_exit (void)
{

	DLOG("mmsv4l2_exit+++\n");

	video_unregister_device(&msmv4l2_vid_device0);
	video_unregister_device(&msmv4l2_vid_device1);

	platform_driver_unregister(&msmv4l2_platform_driver);

	kfree(saved_vout0);
	kfree(saved_vout1);
	saved_vout0 = NULL;
	saved_vout1 = NULL;
	return;
}

module_init (msmv4l2_init);
module_exit (msmv4l2_exit);

MODULE_AUTHOR ("Palm");
MODULE_DESCRIPTION ("MSM V4L2 Driver");
MODULE_LICENSE ("GPL");

