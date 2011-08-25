/* arch/arm/mach-msm/qdsp6/auxpcm_lb_out.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/uaccess.h>

#include <linux/msm_audio.h>

#include <mach/msm_qdsp6_audio.h>
#include <mach/debug_mm.h>

struct auxpcm {
	struct mutex lock;
	struct audio_client *ac;
	uint32_t sample_rate;
	uint32_t channel_count;
	int opened;;
};

static long auxpcmout_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	struct auxpcm *auxpcmout = file->private_data;
	int rc = 0;

	mutex_lock(&auxpcmout->lock);
	switch (cmd) {
	case AUDIO_START: {
		uint32_t acdb_id;
		if (arg == 0) {
			acdb_id = 0;
		} else if (copy_from_user(&acdb_id, (void *) arg,
					sizeof(acdb_id))) {
			pr_info("[%s:%s] copy acdb_id from user failed\n",
					__MM_FILE__, __func__);
			rc = -EFAULT;
			break;
		}
		if (auxpcmout->ac) {
			rc = -EBUSY;
		} else {
			auxpcmout->ac =
				q6audio_open_auxpcm(auxpcmout->sample_rate,
						auxpcmout->channel_count,
						AUDIO_FLAG_WRITE, acdb_id);
			if (!auxpcmout->ac)
				rc = -ENOMEM;
		}
		break;
	}
	case AUDIO_STOP:
		break;
	case AUDIO_FLUSH:
		break;
	case AUDIO_SET_CONFIG: {
		struct msm_audio_config config;
		if (auxpcmout->ac) {
			rc = -EBUSY;
			break;
		}
		if (copy_from_user(&config, (void *) arg, sizeof(config))) {
			rc = -EFAULT;
			break;
		}
		if (config.channel_count != 1) {
			rc = -EINVAL;
			break;
		}
		if (config.sample_rate != 8000) {
			rc = -EINVAL;
			break;
		}
		auxpcmout->sample_rate = config.sample_rate;
		auxpcmout->channel_count = config.channel_count;
		break;
	}
	case AUDIO_GET_CONFIG: {
		struct msm_audio_config config;
		config.buffer_size = 0;
		config.buffer_count = 0;
		config.sample_rate = auxpcmout->sample_rate;
		config.channel_count = auxpcmout->channel_count;
		config.unused[0] = 0;
		config.unused[1] = 0;
		config.unused[2] = 0;
		if (copy_to_user((void *) arg, &config, sizeof(config)))
			rc = -EFAULT;
		break;
	}
	default:
		rc = -EINVAL;
	}
	mutex_unlock(&auxpcmout->lock);
	return rc;
}

static struct auxpcm the_auxpcmout;

static int auxpcmout_open(struct inode *inode, struct file *file)
{
	struct auxpcm *auxpcmout = &the_auxpcmout;

	pr_info("[%s:%s] open\n", __MM_FILE__, __func__);

	mutex_lock(&auxpcmout->lock);

	if (auxpcmout->opened) {
		pr_err("aux pcm loopback rx already open!\n");
		mutex_unlock(&auxpcmout->lock);
		return -EBUSY;
	}
	auxpcmout->channel_count = 1;
	auxpcmout->sample_rate = 8000;
	auxpcmout->opened = 1;
	file->private_data = auxpcmout;
	mutex_unlock(&auxpcmout->lock);
	return 0;
}

static int auxpcmout_release(struct inode *inode, struct file *file)
{
	struct auxpcm *auxpcmout = file->private_data;
	mutex_lock(&auxpcmout->lock);
	if (auxpcmout->ac)
		q6audio_auxpcm_close(auxpcmout->ac);
	auxpcmout->ac = NULL;
	auxpcmout->opened = 0;
	mutex_unlock(&auxpcmout->lock);
	pr_info("[%s:%s] release\n", __MM_FILE__, __func__);
	return 0;
}

static const struct file_operations auxpcmout_fops = {
	.owner		= THIS_MODULE,
	.open		= auxpcmout_open,
	.release	= auxpcmout_release,
	.unlocked_ioctl	= auxpcmout_ioctl,
};

struct miscdevice auxpcmout_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_aux_pcm_lb_out",
	.fops	= &auxpcmout_fops,
};

static int __init auxpcmout_init(void)
{
	mutex_init(&the_auxpcmout.lock);
	return misc_register(&auxpcmout_misc);
}

device_initcall(auxpcmout_init);
