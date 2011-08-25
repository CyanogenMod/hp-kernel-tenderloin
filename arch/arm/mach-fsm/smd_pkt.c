/* Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
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
/*
 * SMD Packet Driver -- Provides a binary SMD non-muxed packet port
 *                       interface.
 */

#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/completion.h>
#include <asm/ioctls.h>

#include <mach/msm_smd.h>
#include <mach/peripheral-loader.h>

#include "modem_notifier.h"
#include "smd_private.h"

#define NUM_SMD_PKT_PORTS 5
#define DEVICE_NAME "smdpkt"
#define MAX_BUF_SIZE 2048

struct smd_pkt_dev {
	struct cdev cdev;
	char name[9];
	struct device *devicep;
	void *pil;
	struct platform_driver driver;

	struct smd_channel *ch;
	struct mutex ch_lock;
	struct mutex rx_lock;
	struct mutex tx_lock;
	struct mutex is_open_lock;
	wait_queue_head_t ch_wait_queue;
	wait_queue_head_t ch_opened_wait_queue;

	int i;

	unsigned char tx_buf[MAX_BUF_SIZE];
	unsigned char rx_buf[MAX_BUF_SIZE];
	int is_open;

	struct notifier_block nb;
	int has_reset;
	struct mutex has_reset_lock;

} *smd_pkt_devp[NUM_SMD_PKT_PORTS];

struct class *smd_pkt_classp;
static dev_t smd_pkt_number;
static void check_and_wakeup(struct smd_pkt_dev *smd_pkt_devp);

#define DEBUG
#undef DEBUG

#ifdef DEBUG
#define D_DUMP_BUFFER(prestr, cnt, buf) \
do { \
	int i; \
	printk(KERN_ERR "%s", prestr); \
	for (i = 0; i < cnt; i++) \
		printk(KERN_ERR "%.2x", buf[i]); \
	printk(KERN_ERR "\n"); \
} while (0)
#else
#define D_DUMP_BUFFER(prestr, cnt, buf) do {} while (0)
#endif

#ifdef DEBUG
#define D(x...) printk(x)
#else
#define D(x...) do {} while (0)
#endif

static void clean_and_signal(struct smd_pkt_dev *smd_pkt_devp)
{
	mutex_lock(&smd_pkt_devp->has_reset_lock);
	smd_pkt_devp->has_reset = 1;
	mutex_unlock(&smd_pkt_devp->has_reset_lock);

	mutex_lock(&smd_pkt_devp->is_open_lock);
	smd_pkt_devp->is_open = 0;
	mutex_unlock(&smd_pkt_devp->is_open_lock);

	wake_up_interruptible(&smd_pkt_devp->ch_wait_queue);
	wake_up_interruptible(&smd_pkt_devp->ch_opened_wait_queue);
}

static int modem_notifier(struct notifier_block *this,
			  unsigned long code,
			  void *_cmd)
{
	struct smd_pkt_dev *smd_pkt_devp =
		container_of(this,
			     struct smd_pkt_dev,
			     nb);

	if (!smd_pkt_devp)
		return NOTIFY_DONE;

	switch (code) {
	case MODEM_NOTIFIER_START_RESET:
		printk(KERN_ERR "Notify: start reset ch:%i\n",
		       smd_pkt_devp->i);
		clean_and_signal(smd_pkt_devp);
		break;
	case MODEM_NOTIFIER_END_RESET:
		printk(KERN_ERR "Notify: end reset\n");
		break;
	default:
		printk(KERN_ERR "Notify: general\n");
		break;
	}
	return NOTIFY_DONE;
}

int smd_pkt_ioctl(struct inode *inode,
		    struct file *file,
		    unsigned int cmd,
		    unsigned long arg)
{
	int ret;
	struct smd_pkt_dev *smd_pkt_devp;

	smd_pkt_devp = file->private_data;

	switch (cmd) {
	case TIOCMGET:
		ret = smd_tiocmget(smd_pkt_devp->ch);
		break;
	case TIOCMSET:
		ret = smd_tiocmset(smd_pkt_devp->ch, arg, ~arg);
		break;
	default:
		ret = -1;
	}

	return ret;
}

ssize_t smd_pkt_read(struct file *file,
		       char __user *buf,
		       size_t count,
		       loff_t *ppos)
{
	int r;
	int bytes_read;
	struct smd_pkt_dev *smd_pkt_devp;
	struct smd_channel *chl;

	D(KERN_ERR "%s: read %i bytes\n",
	  __func__, count);

	smd_pkt_devp = file->private_data;

	if (!smd_pkt_devp->ch)
		return -EINVAL;

	chl = smd_pkt_devp->ch;
wait_for_packet:
	r = wait_event_interruptible(smd_pkt_devp->ch_wait_queue,
				     (smd_cur_packet_size(chl) > 0 &&
				      smd_read_avail(chl) >=
				      smd_cur_packet_size(chl)) |
				     smd_pkt_devp->has_reset);

	if (smd_pkt_devp->has_reset)
		return -ENETRESET;

	if (r < 0) {
		/* qualify error message */
		if (r != -ERESTARTSYS) {
			/* we get this anytime a signal comes in */
			printk(KERN_ERR "ERROR:%s:%i:%s: "
			       "wait_event_interruptible ret %i\n",
			       __FILE__,
			       __LINE__,
			       __func__,
			       r
				);
		}
		return r;
	}

	/* Here we have a whole packet waiting for us */

	mutex_lock(&smd_pkt_devp->rx_lock);
	bytes_read = smd_cur_packet_size(smd_pkt_devp->ch);

	D(KERN_ERR "%s: after wait_event_interruptible bytes_read = %i\n",
	  __func__, bytes_read);

	if (bytes_read == 0 ||
	    bytes_read < smd_read_avail(smd_pkt_devp->ch)) {
		D(KERN_ERR "%s: Nothing to read\n", __func__);
		mutex_unlock(&smd_pkt_devp->rx_lock);
		goto wait_for_packet;
	}

	if (bytes_read > count) {
		printk(KERN_ERR "packet size %i > buffer size %i, "
		       "dropping packet!", bytes_read, count);
		smd_read(smd_pkt_devp->ch, 0, bytes_read);
		mutex_unlock(&smd_pkt_devp->rx_lock);
		return -EINVAL;
	}

	/* smd_read and copy_to_user need to be merged to only do 1 copy */
	if (smd_read(smd_pkt_devp->ch, smd_pkt_devp->rx_buf, bytes_read)
	    != bytes_read) {
		mutex_unlock(&smd_pkt_devp->rx_lock);
		if (smd_pkt_devp->has_reset)
			return -ENETRESET;

		printk(KERN_ERR "user read: not enough data?!\n");
		return -EINVAL;
	}
	D_DUMP_BUFFER("read: ", bytes_read, smd_pkt_devp->rx_buf);
	r = copy_to_user(buf, smd_pkt_devp->rx_buf, bytes_read);
	mutex_unlock(&smd_pkt_devp->rx_lock);
	if (r > 0) {
		printk(KERN_ERR "ERROR:%s:%i:%s: "
		       "copy_to_user could not copy %i bytes.\n",
		       __FILE__,
		       __LINE__,
		       __func__,
		       r);
		return r;
	}

	D(KERN_ERR "%s: just read %i bytes\n",
	  __func__, bytes_read);

	/* check and wakeup read threads waiting on this device */
	check_and_wakeup(smd_pkt_devp);

	return bytes_read;
}

ssize_t smd_pkt_write(struct file *file,
		       const char __user *buf,
		       size_t count,
		       loff_t *ppos)
{
	int r;
	struct smd_pkt_dev *smd_pkt_devp;

	if (count > MAX_BUF_SIZE)
		return -EINVAL;

	D(KERN_ERR "%s: writting %i bytes\n",
	  __func__, count);

	smd_pkt_devp = file->private_data;

	if (!smd_pkt_devp->ch)
		return -EINVAL;

	r = wait_event_interruptible(smd_pkt_devp->ch_opened_wait_queue,
				     smd_pkt_devp->is_open |
				     smd_pkt_devp->has_reset);

	if (smd_pkt_devp->has_reset)
		return -ENETRESET;

	if (r < 0) {
		/* qualify error message */
		if (r != -ERESTARTSYS) {
			/* we get this anytime a signal comes in */
			printk(KERN_ERR "ERROR:%s:%i:%s: "
			       "wait_event_interruptible ret %i\n",
			       __FILE__,
			       __LINE__,
			       __func__,
			       r
				);
		}
		return r;
	}

	D_DUMP_BUFFER("write: ", count, buf);

	mutex_lock(&smd_pkt_devp->tx_lock);
	r = copy_from_user(smd_pkt_devp->tx_buf, buf, count);
	if (r > 0) {
		printk(KERN_ERR "ERROR:%s:%i:%s: "
		       "copy_from_user could not copy %i bytes.\n",
		       __FILE__,
		       __LINE__,
		       __func__,
		       r);
		mutex_unlock(&smd_pkt_devp->tx_lock);
		return r;
	}

	D(KERN_ERR "%s: after copy_from_user. count = %i\n",
	  __func__, count);

	r = smd_write(smd_pkt_devp->ch, smd_pkt_devp->tx_buf, count);
	if (r != count) {
		mutex_unlock(&smd_pkt_devp->tx_lock);
		if (smd_pkt_devp->has_reset)
			return -ENETRESET;

		printk(KERN_ERR "ERROR:%s:%i:%s: "
		       "smd_write(ch,buf,count = %i) ret %i.\n",
		       __FILE__,
		       __LINE__,
		       __func__,
		       count,
		       r);
		return r;
	}
	mutex_unlock(&smd_pkt_devp->tx_lock);

	D(KERN_ERR "%s: just wrote %i bytes\n",
	       __func__, count);

	return count;
}

static void check_and_wakeup(struct smd_pkt_dev *smd_pkt_devp)
{
	int sz;

	if (!smd_pkt_devp || !smd_pkt_devp->ch)
		return;

	sz = smd_cur_packet_size(smd_pkt_devp->ch);
	if (sz == 0) {
		D(KERN_ERR "%s: packet size is 0\n", __func__);
		return;
	}
	if (sz > smd_read_avail(smd_pkt_devp->ch)) {
		D(KERN_ERR "%s: packet size is %i - "
		  "the whole packet isn't here\n",
		  __func__, sz);
		return;
	}

	/* here we have a packet of size sz ready */
	wake_up_interruptible(&smd_pkt_devp->ch_wait_queue);
	D(KERN_ERR "%s: after wake_up\n", __func__);
}

static void ch_notify(void *priv, unsigned event)
{
	struct smd_pkt_dev *smd_pkt_devp = priv;

	if (smd_pkt_devp->ch == 0)
		return;

	switch (event) {
	case SMD_EVENT_DATA: {
		D(KERN_ERR "%s: data\n", __func__);
		check_and_wakeup(smd_pkt_devp);
		D(KERN_ERR "%s: data after check_and_wakeup\n", __func__);
		break;
	}
	case SMD_EVENT_OPEN:
		D(KERN_ERR "%s: smd opened\n",
		  __func__);
		smd_pkt_devp->is_open = 1;
		wake_up_interruptible(&smd_pkt_devp->ch_opened_wait_queue);
		break;
	case SMD_EVENT_CLOSE:
		smd_pkt_devp->is_open = 0;
		printk(KERN_ERR "%s: smd closed\n",
		       __func__);
		break;
	}
}

static char *smd_pkt_dev_name[] = {
	"smdcntl0",
	"smdcntl1",
	"smdcntl2",
	"smd22",
	"smd_pkt_loopback",
};

static char *smd_ch_name[] = {
	"DATA5_CNTL",
	"DATA6_CNTL",
	"DATA7_CNTL",
	"DATA22",
	"LOOPBACK",
};

static DECLARE_COMPLETION(modem_loaded);

static int smd_pkt_dummy_probe(struct platform_device *pdev)
{
	complete_all(&modem_loaded);
	return 0;
}

int smd_pkt_open(struct inode *inode, struct file *file)
{
	int r = 0;
	struct smd_pkt_dev *smd_pkt_devp;

	smd_pkt_devp = container_of(inode->i_cdev, struct smd_pkt_dev, cdev);

	if (!smd_pkt_devp)
		return -EINVAL;

	file->private_data = smd_pkt_devp;

	mutex_lock(&smd_pkt_devp->ch_lock);
	if (smd_pkt_devp->ch == 0) {

		smd_pkt_devp->pil = pil_get("modem");
		if (IS_ERR(smd_pkt_devp->pil)) {
			r = PTR_ERR(smd_pkt_devp->pil);
			goto out;
		}
		/*
		 * Wait for a packet channel to be allocated so we know
		 * the modem is ready enough.
		 */
		r = wait_for_completion_interruptible_timeout(&modem_loaded,
				msecs_to_jiffies(60000));
		if (r == 0) {
			pr_err("Timed out waiting for SMD channel\n");
			r = -ETIMEDOUT;
			pil_put(smd_pkt_devp->pil);
			goto out;
		} else if (r < 0) {
			pr_err("Error waiting for SMD channel: %d\n", r);
			pil_put(smd_pkt_devp->pil);
			goto out;
		}

		if (!strcmp(smd_ch_name[smd_pkt_devp->i], "LOOPBACK")) {
			/* set smsm state to SMSM_SMD_LOOPBACK state
			** and wait allowing enough time for Modem side
			** to open the loopback port (Currently, this is
			** this is effecient than polling).*/
			smsm_change_state(SMSM_APPS_STATE,
					  0, SMSM_SMD_LOOPBACK);
			msleep(100);
		}

		r = smd_open(smd_ch_name[smd_pkt_devp->i],
			     &smd_pkt_devp->ch,
			     smd_pkt_devp,
			     ch_notify);
		if (r < 0) {
			printk(KERN_ERR "%s failed for %s with rc %d\n",
					__func__, smd_ch_name[smd_pkt_devp->i],
					r);
			pil_put(smd_pkt_devp->pil);
		}
	}
out:
	mutex_unlock(&smd_pkt_devp->ch_lock);

	return r;
}

int smd_pkt_release(struct inode *inode, struct file *file)
{
	int r = 0;
	struct smd_pkt_dev *smd_pkt_devp = file->private_data;

	if (!smd_pkt_devp)
		return -EINVAL;

	clean_and_signal(smd_pkt_devp);

	mutex_lock(&smd_pkt_devp->ch_lock);
	if (smd_pkt_devp->ch != 0) {
		r = smd_close(smd_pkt_devp->ch);
		smd_pkt_devp->ch = 0;
		pil_put(smd_pkt_devp->pil);
	}
	mutex_unlock(&smd_pkt_devp->ch_lock);

	mutex_lock(&smd_pkt_devp->has_reset_lock);
	smd_pkt_devp->has_reset = 0;
	mutex_unlock(&smd_pkt_devp->has_reset_lock);

	return r;
}

static const struct file_operations smd_pkt_fops = {
	.owner = THIS_MODULE,
	.open = smd_pkt_open,
	.release = smd_pkt_release,
	.read = smd_pkt_read,
	.write = smd_pkt_write,
	.ioctl = smd_pkt_ioctl,
};

static int __init smd_pkt_init(void)
{
	int i;
	int r;

	r = alloc_chrdev_region(&smd_pkt_number,
			       0,
			       NUM_SMD_PKT_PORTS,
			       DEVICE_NAME);
	if (IS_ERR_VALUE(r)) {
		printk(KERN_ERR "ERROR:%s:%i:%s: "
		       "alloc_chrdev_region() ret %i.\n",
		       __FILE__,
		       __LINE__,
		       __func__,
		       r);
		goto error0;
	}

	smd_pkt_classp = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(smd_pkt_classp)) {
		printk(KERN_ERR "ERROR:%s:%i:%s: "
		       "class_create() ENOMEM\n",
		       __FILE__,
		       __LINE__,
		       __func__);
		r = -ENOMEM;
		goto error1;
	}

	for (i = 0; i < NUM_SMD_PKT_PORTS; ++i) {
		smd_pkt_devp[i] = kzalloc(sizeof(struct smd_pkt_dev),
					 GFP_KERNEL);
		if (IS_ERR(smd_pkt_devp[i])) {
			printk(KERN_ERR "ERROR:%s:%i:%s kmalloc() ENOMEM\n",
			       __FILE__,
			       __LINE__,
			       __func__);
			r = -ENOMEM;
			goto error2;
		}

		smd_pkt_devp[i]->i = i;

		init_waitqueue_head(&smd_pkt_devp[i]->ch_wait_queue);
		smd_pkt_devp[i]->is_open = 0;
		init_waitqueue_head(&smd_pkt_devp[i]->ch_opened_wait_queue);

		mutex_init(&smd_pkt_devp[i]->ch_lock);
		mutex_init(&smd_pkt_devp[i]->rx_lock);
		mutex_init(&smd_pkt_devp[i]->tx_lock);
		mutex_init(&smd_pkt_devp[i]->is_open_lock);

		cdev_init(&smd_pkt_devp[i]->cdev, &smd_pkt_fops);
		smd_pkt_devp[i]->cdev.owner = THIS_MODULE;

		r = cdev_add(&smd_pkt_devp[i]->cdev,
			     (smd_pkt_number + i),
			     1);

		if (IS_ERR_VALUE(r)) {
			printk(KERN_ERR "%s:%i:%s: cdev_add() ret %i\n",
			       __FILE__,
			       __LINE__,
			       __func__,
			       r);
			kfree(smd_pkt_devp[i]);
			goto error2;
		}

		smd_pkt_devp[i]->devicep =
			device_create(smd_pkt_classp,
				      NULL,
				      (smd_pkt_number + i),
				      NULL,
				      smd_pkt_dev_name[i]);

		if (IS_ERR(smd_pkt_devp[i]->devicep)) {
			printk(KERN_ERR "%s:%i:%s: "
			       "device_create() ENOMEM\n",
			       __FILE__,
			       __LINE__,
			       __func__);
			r = -ENOMEM;
			cdev_del(&smd_pkt_devp[i]->cdev);
			kfree(smd_pkt_devp[i]);
			goto error2;
		}

		smd_pkt_devp[i]->nb.notifier_call = modem_notifier;
		modem_register_notifier(&smd_pkt_devp[i]->nb);
		mutex_init(&smd_pkt_devp[i]->has_reset_lock);

		smd_pkt_devp[i]->driver.probe = smd_pkt_dummy_probe;
		smd_pkt_devp[i]->driver.driver.name = smd_ch_name[i];
		smd_pkt_devp[i]->driver.driver.owner = THIS_MODULE;
		r = platform_driver_register(&smd_pkt_devp[i]->driver);
		if (r)
			goto error2;
	}

	printk(KERN_INFO "SMD Packet Port Driver Initialized.\n");
	return 0;

 error2:
	if (i > 0) {
		while (--i >= 0) {
			platform_driver_unregister(&smd_pkt_devp[i]->driver);
			cdev_del(&smd_pkt_devp[i]->cdev);
			kfree(smd_pkt_devp[i]);
			device_destroy(smd_pkt_classp,
				       MKDEV(MAJOR(smd_pkt_number), i));
		}
	}

	class_destroy(smd_pkt_classp);
 error1:
	unregister_chrdev_region(MAJOR(smd_pkt_number), NUM_SMD_PKT_PORTS);
 error0:
	return r;
}

static void __exit smd_pkt_cleanup(void)
{
	int i;

	for (i = 0; i < NUM_SMD_PKT_PORTS; ++i) {
		platform_driver_unregister(&smd_pkt_devp[i]->driver);
		modem_unregister_notifier(&smd_pkt_devp[i]->nb);
		cdev_del(&smd_pkt_devp[i]->cdev);
		kfree(smd_pkt_devp[i]);
		device_destroy(smd_pkt_classp,
			       MKDEV(MAJOR(smd_pkt_number), i));
	}

	class_destroy(smd_pkt_classp);

	unregister_chrdev_region(MAJOR(smd_pkt_number), NUM_SMD_PKT_PORTS);
}

module_init(smd_pkt_init);
module_exit(smd_pkt_cleanup);

MODULE_DESCRIPTION("MSM Shared Memory Packet Port");
MODULE_LICENSE("GPL v2");
