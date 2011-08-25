/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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

struct dtmf {
	struct mutex lock;
	struct audio_client *ac;
	struct msm_dtmf_config cfg;
};

static long dtmf_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct dtmf *dtmf = file->private_data;
	int rc = 0;

	mutex_lock(&dtmf->lock);
	switch (cmd) {

	case AUDIO_START: {
		if (dtmf->ac) {
			rc = -EBUSY;
		} else {
			dtmf->ac = q6audio_open_dtmf(48000, 2, 0);
			if (!dtmf->ac)
				rc = -ENOMEM;
		}
		break;
	}
	case AUDIO_PLAY_DTMF: {
		rc = copy_from_user((void *)&dtmf->cfg, (void *)arg,
					sizeof(struct msm_dtmf_config));

		rc = q6audio_play_dtmf(dtmf->ac, dtmf->cfg.dtmf_hi,
					dtmf->cfg.dtmf_low, dtmf->cfg.duration,
					dtmf->cfg.rx_gain);
		if (rc) {
			pr_err("[%s:%s] DTMF_START failed\n", __MM_FILE__,
					__func__);
			break;
		}
		break;
	}
	default:
		rc = -EINVAL;
	}
	mutex_unlock(&dtmf->lock);

	return rc;
}

static int dtmf_open(struct inode *inode, struct file *file)
{
	int rc = 0;

	struct dtmf *dtmf;
	dtmf = kzalloc(sizeof(struct dtmf), GFP_KERNEL);

	if (!dtmf)
		return -ENOMEM;

	mutex_init(&dtmf->lock);

	file->private_data = dtmf;
	return rc;
}

static int dtmf_release(struct inode *inode, struct file *file)
{
	struct dtmf *dtmf = file->private_data;
	if (dtmf->ac)
		q6audio_close(dtmf->ac);
	kfree(dtmf);
	return 0;
}

static const struct file_operations dtmf_fops = {
	.owner		= THIS_MODULE,
	.open		= dtmf_open,
	.release	= dtmf_release,
	.unlocked_ioctl	= dtmf_ioctl,
};

struct miscdevice dtmf_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_dtmf",
	.fops	= &dtmf_fops,
};

static int __init dtmf_init(void)
{
	return misc_register(&dtmf_misc);
}

device_initcall(dtmf_init);
