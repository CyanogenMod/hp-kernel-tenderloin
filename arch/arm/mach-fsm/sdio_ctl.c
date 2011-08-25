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
 */

/*
 * SDIO Control Driver -- Provides a binary SDIO muxed control port
 *                       interface.
 */

#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <asm/ioctls.h>
#include <linux/platform_device.h>
#include <mach/msm_smd.h>
#include <mach/sdio_al.h>
#include "modem_notifier.h"

#define MAX_WRITE_RETRY 5
#define MAGIC_NO_V1 0x33FC
#define NUM_SDIO_CTL_PORTS 8
#define DEVICE_NAME "sdioctl"
#define MAX_BUF_SIZE 2048

static int msm_sdio_ctl_debug_mask;
module_param_named(debug_mask, msm_sdio_ctl_debug_mask,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);

enum cmd_type {
	DATA = 0,
	OPEN,
	CLOSE,
};

struct sdio_cmux_ch {
	int lc_id;

	spinlock_t lc_lock;
	int is_remote_open;
	int is_local_open;

	spinlock_t rx_lock;
	uint32_t read_avail;
	struct list_head rx_list;

	void *priv;
} logical_ch[NUM_SDIO_CTL_PORTS];

struct sdio_ctl_dev {
	int id;
	char name[9];
	struct cdev cdev;
	struct device *devicep;
	struct mutex dev_lock;
	int ref_count;

	struct sdio_cmux_ch *ch;

	wait_queue_head_t open_wait_queue;
	wait_queue_head_t read_wait_queue;

} *sdio_ctl_devp[NUM_SDIO_CTL_PORTS];

struct sdio_cmux_hdr {
	uint16_t magic_no;
	uint8_t reserved;
	uint8_t cmd;
	uint8_t pad_bytes;
	uint8_t lc_id;
	uint16_t pkt_len;
};

struct sdio_ctl_pkt {
	struct sdio_cmux_hdr *hdr;
	void *data;
};

struct sdio_ctl_list_elem {
	struct list_head list;
	struct sdio_ctl_pkt ctl_pkt;
};

static void sdio_demux_fn(struct work_struct *work);
static DECLARE_WORK(sdio_demux_work, sdio_demux_fn);
static struct workqueue_struct *sdio_demux_wq;

static spinlock_t tx_lock;
static uint32_t bytes_to_write;
static LIST_HEAD(tx_list);
static wait_queue_head_t write_wait_queue;

static void sdio_mux_fn(struct work_struct *work);
static DECLARE_WORK(sdio_mux_work, sdio_mux_fn);
static struct workqueue_struct *sdio_mux_wq;

static struct sdio_channel *sdio_ctl_chl;
struct class *sdio_ctl_classp;
static dev_t sdio_ctl_number;

#if defined(CONFIG_MSM_SDIO_CTL_DEBUG)
#define D_DUMP_BUFFER(prestr, cnt, buf) \
do { \
	if (msm_sdio_ctl_debug_mask) { \
		int i; \
		printk(KERN_ERR "%s", prestr); \
		for (i = 0; i < cnt; i++) \
			printk(KERN_ERR "%.2x", buf[i]); \
		printk(KERN_ERR "\n"); \
	} \
} while (0)

#define D(x...) \
do { \
	if (msm_sdio_ctl_debug_mask) \
		printk(x); \
} while (0)

#else
#define D_DUMP_BUFFER(prestr, cnt, buf) do {} while (0)
#define D(x...) do {} while (0)
#endif

static int sdio_cmux_ch_alloc(int id)
{
	if (id < 0 || id >= NUM_SDIO_CTL_PORTS) {
		D(KERN_ERR "%s: Invalid lc_id - %d\n", __func__, id);
		return -EINVAL;
	}

	logical_ch[id].lc_id = id;
	spin_lock_init(&logical_ch[id].lc_lock);
	logical_ch[id].is_remote_open = 0;
	logical_ch[id].is_local_open = 0;

	logical_ch[id].read_avail = 0;
	INIT_LIST_HEAD(&logical_ch[id].rx_list);
	spin_lock_init(&logical_ch[id].rx_lock);

	logical_ch[id].priv = NULL;
	return 0;
}

static int sdio_cmux_ch_clear_and_signal(int id)
{
	unsigned long flags;
	struct sdio_ctl_list_elem *list_elem;

	if (id < 0 || id >= NUM_SDIO_CTL_PORTS) {
		D(KERN_ERR "%s: Invalid lc_id - %d\n", __func__, id);
		return -EINVAL;
	}

	spin_lock_irqsave(&logical_ch[id].lc_lock, flags);
	logical_ch[id].is_remote_open = 0;

	spin_lock(&logical_ch[id].rx_lock);
	while (!list_empty(&logical_ch[id].rx_list)) {
		list_elem = list_first_entry(&logical_ch[id].rx_list,
					     struct sdio_ctl_list_elem,
					     list);
		list_del(&list_elem->list);
		kfree(list_elem->ctl_pkt.hdr);
		kfree(list_elem);
	}
	logical_ch[id].read_avail = 0;
	spin_unlock(&logical_ch[id].rx_lock);
	logical_ch[id].priv = NULL;

	spin_unlock_irqrestore(&logical_ch[id].lc_lock, flags);
	wake_up(&sdio_ctl_devp[id]->read_wait_queue);
	wake_up(&write_wait_queue);
	return 0;
}

static int sdio_cmux_write_cmd(const int id, enum cmd_type type)
{
	int write_size = 0;
	void *write_data = NULL;
	unsigned long flags;
	struct sdio_ctl_list_elem *list_elem;

	if (id < 0 || id >= NUM_SDIO_CTL_PORTS) {
		D(KERN_ERR "%s: Invalid lc_id - %d\n", __func__, id);
		return -EINVAL;
	}

	if (type < OPEN || type > CLOSE) {
		D(KERN_ERR "%s: Invalid cmd - %d\n", __func__, type);
		return -EINVAL;
	}

	write_size = sizeof(struct sdio_cmux_hdr);
	list_elem = kmalloc(sizeof(struct sdio_ctl_list_elem), GFP_KERNEL);
	if (!list_elem) {
		D(KERN_ERR "%s: list_elem alloc failed\n", __func__);
		return -ENOMEM;
	}

	write_data = kmalloc(write_size, GFP_KERNEL);
	if (!write_data) {
		D(KERN_ERR "%s: write_data alloc failed\n", __func__);
		kfree(list_elem);
		return -ENOMEM;
	}

	list_elem->ctl_pkt.hdr = (struct sdio_cmux_hdr *)write_data;
	list_elem->ctl_pkt.data = NULL;

	list_elem->ctl_pkt.hdr->lc_id = (uint8_t)id;
	list_elem->ctl_pkt.hdr->pkt_len = (uint16_t)0;
	list_elem->ctl_pkt.hdr->cmd = (uint8_t)type;
	list_elem->ctl_pkt.hdr->reserved = (uint8_t)0;
	list_elem->ctl_pkt.hdr->pad_bytes = (uint8_t)0;
	list_elem->ctl_pkt.hdr->magic_no = (uint16_t)MAGIC_NO_V1;

	spin_lock_irqsave(&logical_ch[id].lc_lock, flags);
	spin_lock(&tx_lock);
	list_add_tail(&list_elem->list, &tx_list);
	bytes_to_write += write_size;
	spin_unlock(&tx_lock);
	spin_unlock_irqrestore(&logical_ch[id].lc_lock, flags);
	queue_work(sdio_mux_wq, &sdio_mux_work);

	return 0;
}

static int sdio_cmux_open(const int id, struct sdio_cmux_ch **ch,
			  void *priv)
{
	unsigned long flags;

	if (id < 0 || id >= NUM_SDIO_CTL_PORTS) {
		printk(KERN_ERR "%s: Invalid id - %d\n", __func__, id);
		return -EINVAL;
	}

	spin_lock_irqsave(&logical_ch[id].lc_lock, flags);
	if (!logical_ch[id].is_remote_open) {
		D(KERN_ERR "%s: Remote ch%d not opened\n", __func__, id);
		spin_unlock_irqrestore(&logical_ch[id].lc_lock, flags);
		return -EINVAL;
	}
	if (!logical_ch[id].is_local_open) {
		logical_ch[id].is_local_open = 1;
		if (!priv && !logical_ch[id].priv)
			logical_ch[id].priv = priv;

		*ch = &logical_ch[id];
	}
	spin_unlock_irqrestore(&logical_ch[id].lc_lock, flags);
	sdio_cmux_write_cmd(id, OPEN);
	return 0;
}

static int sdio_cmux_close(struct sdio_cmux_ch *ch)
{
	unsigned long flags;
	struct sdio_ctl_list_elem *list_elem;

	if (!ch) {
		printk(KERN_ERR "%s: Invalid channel close\n", __func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&ch->rx_lock, flags);
	ch->is_local_open = 0;

	spin_lock(&ch->rx_lock);
	while (!list_empty(&ch->rx_list)) {
		list_elem = list_first_entry(&ch->rx_list,
					     struct sdio_ctl_list_elem,
					     list);
		list_del(&list_elem->list);
		kfree(list_elem->ctl_pkt.hdr);
		kfree(list_elem);
	}
	ch->read_avail = 0;
	spin_unlock(&ch->rx_lock);
	ch->priv = NULL;

	spin_unlock_irqrestore(&ch->rx_lock, flags);
	sdio_cmux_write_cmd(ch->lc_id, CLOSE);
	return 0;
}

static int sdio_cmux_read_avail(struct sdio_cmux_ch *ch)
{
	if (!ch)
		return -EINVAL;

	return ch->read_avail;
}

static int sdio_cmux_write_avail(struct sdio_cmux_ch *ch)
{
	int write_avail;
	write_avail = sdio_write_avail(sdio_ctl_chl);
	return write_avail - bytes_to_write;
}

static int sdio_cmux_read(struct sdio_cmux_ch *ch, void *data, int len)
{
	unsigned long flags;
	int r;
	uint32_t bytes_to_read = 0;
	struct sdio_ctl_list_elem *list_elem = NULL;

	if (!ch) {
		D(KERN_ERR "%s: Invalid channel\n", __func__);
		return -EINVAL;
	}

	if (len <= 0) {
		D(KERN_ERR "%s: Invalid len %d bytes to read\n",
				 __func__, len);
		return -EINVAL;
	}

	spin_lock_irqsave(&ch->rx_lock, flags);
	if (list_empty(&ch->rx_list)) {
		D(KERN_ERR "%s: Nothing in ch%d's rx_list\n",
				 __func__, ch->lc_id);
		spin_unlock_irqrestore(&ch->rx_lock, flags);
		return -EAGAIN;
	}

	list_elem = list_first_entry(&ch->rx_list,
				     struct sdio_ctl_list_elem,	list);
	bytes_to_read = (uint32_t)list_elem->ctl_pkt.hdr->pkt_len;
	D(KERN_ERR "%s: Copying %d bytes to ch%d\n", __func__,
		    bytes_to_read, ch->lc_id);
	if (bytes_to_read > len) {
		D(KERN_ERR "%s: ch%d Data size %d > "
			   "buffer len %d\n", __func__,
			    ch->lc_id, bytes_to_read, len);
		spin_unlock_irqrestore(&ch->rx_lock, flags);
		return -EINVAL;
	}

	r = copy_to_user(data, list_elem->ctl_pkt.data,
			 bytes_to_read);
	if (r > 0) {
		D(KERN_ERR "%s: copy_to_user failed for ch%d\n",
				 __func__, ch->lc_id);
		spin_unlock_irqrestore(&ch->rx_lock, flags);
		return -EINVAL;
	}
	ch->read_avail -= bytes_to_read;
	list_del(&list_elem->list);
	kfree(list_elem->ctl_pkt.hdr);
	kfree(list_elem);
	spin_unlock_irqrestore(&ch->rx_lock, flags);

	return bytes_to_read;
}

static int sdio_cmux_write(struct sdio_cmux_ch *ch, void *data, int len)
{
	int r = 0;
	struct sdio_ctl_list_elem *list_elem;
	uint32_t write_size;
	void *write_data = NULL;
	unsigned long flags;

	if (!ch) {
		printk(KERN_ERR "%s: Invalid channel\n", __func__);
		return -EINVAL;
	}

	if (len <= 0) {
		printk(KERN_ERR "%s: Invalid len %d bytes to write\n",
				 __func__, len);
		return -EINVAL;
	}

	write_size = sizeof(struct sdio_cmux_hdr) + len;
	list_elem = kmalloc(sizeof(struct sdio_ctl_list_elem), GFP_KERNEL);
	if (!list_elem) {
		D(KERN_ERR "%s: list_elem alloc failed\n", __func__);
		return -ENOMEM;
	}

	write_data = kmalloc(write_size, GFP_KERNEL);
	if (!write_data) {
		D(KERN_ERR "%s: write_data alloc failed\n", __func__);
		kfree(list_elem);
		return -ENOMEM;
	}

	list_elem->ctl_pkt.hdr = (struct sdio_cmux_hdr *)write_data;
	list_elem->ctl_pkt.data = (void *)((char *)write_data +
					sizeof(struct sdio_cmux_hdr));

	r = copy_from_user(list_elem->ctl_pkt.data, data, len);
	if (r > 0) {
		D(KERN_ERR "%s: copy_from_user failed\n", __func__);
		kfree(write_data);
		kfree(list_elem);
		return -EINVAL;
	}
	list_elem->ctl_pkt.hdr->lc_id = (uint8_t)ch->lc_id;
	list_elem->ctl_pkt.hdr->pkt_len = (uint16_t)len;
	list_elem->ctl_pkt.hdr->cmd = (uint8_t)DATA;
	list_elem->ctl_pkt.hdr->reserved = (uint8_t)0;
	list_elem->ctl_pkt.hdr->pad_bytes = (uint8_t)0;
	list_elem->ctl_pkt.hdr->magic_no = (uint16_t)MAGIC_NO_V1;

	spin_lock_irqsave(&ch->lc_lock, flags);
	if (!ch->is_remote_open) {
		printk(KERN_ERR "%s: Remote ch%d is not open\n", __func__,
				 ch->lc_id);
		spin_unlock_irqrestore(&ch->lc_lock, flags);
		kfree(write_data);
		kfree(list_elem);
		return -ENODEV;
	}
	spin_lock(&tx_lock);
	list_add_tail(&list_elem->list, &tx_list);
	bytes_to_write += write_size;
	spin_unlock(&tx_lock);
	spin_unlock_irqrestore(&ch->lc_lock, flags);
	queue_work(sdio_mux_wq, &sdio_mux_work);

	return len;
}

static int is_remote_open(int id)
{
	if (id < 0 || id >= NUM_SDIO_CTL_PORTS)
		return -EINVAL;

	return logical_ch[id].is_remote_open;
}

ssize_t sdio_ctl_read(struct file *file,
		      char __user *buf,
		      size_t count,
		      loff_t *ppos)
{
	int r = 0, id, bytes_read;
	struct sdio_ctl_dev *sdio_ctl_devp;
	struct sdio_cmux_ch *ch;

	sdio_ctl_devp = file->private_data;

	if (!sdio_ctl_devp->ch) {
		D(KERN_ERR "%s: ch%d not opened\n",
				 __func__, sdio_ctl_devp->id);
		return -ENODEV;
	}
	D(KERN_ERR "%s: read from ch%d\n", __func__, sdio_ctl_devp->id);

	ch = sdio_ctl_devp->ch;
	id = sdio_ctl_devp->id;
	while (sdio_cmux_read_avail(ch) <= 0) {
		r = wait_event_interruptible(sdio_ctl_devp->read_wait_queue,
					     sdio_cmux_read_avail(ch) > 0 ||
					     !is_remote_open(id));

		if (!is_remote_open(id))
			return -ENODEV;

		if (r < 0) {
			/* qualify error message */
			if (r != -ERESTARTSYS) {
				/* we get this anytime a signal comes in */
				D(KERN_ERR "ERROR:%s:%i:%s: "
				       "wait_event_interruptible ret %i\n",
				       __FILE__,
				       __LINE__,
				       __func__,
				       r
					);
			}
			return r;
		}
	}
	/* Here we have a whole packet waiting for us */
	bytes_read = sdio_cmux_read(ch, buf, count);

	D(KERN_ERR "%s: Returning %d bytes to ch%d\n", __func__,
			bytes_read, sdio_ctl_devp->id);
	return bytes_read;
}


ssize_t sdio_ctl_write(struct file *file,
		       const char __user *buf,
		       size_t count,
		       loff_t *ppos)
{
	int r = 0, id;
	struct sdio_cmux_ch *ch;
	struct sdio_ctl_dev *sdio_ctl_devp;

	if (count <= 0)
		return -EINVAL;

	sdio_ctl_devp = file->private_data;
	if (!sdio_ctl_devp)
		return -EINVAL;

	D(KERN_INFO "%s: writing %i bytes on ch%d\n",
	  __func__, count, sdio_ctl_devp->id);
	ch = sdio_ctl_devp->ch;
	id = sdio_ctl_devp->id;
	while (sdio_cmux_write_avail(ch) < count) {
		r = wait_event_interruptible(write_wait_queue,
					     sdio_cmux_write_avail(ch) > count
					     || !is_remote_open(id));

		if (!is_remote_open(id))
			return -ENODEV;

		if (r < 0) {
			/* qualify error message */
			if (r != -ERESTARTSYS) {
				/* we get this anytime a signal comes in */
				D(KERN_ERR "ERROR:%s:%i:%s: "
					   "wait_event_interruptible ret %i\n",
					   __FILE__,
					   __LINE__,
					   __func__,
					   r
					);
			}
			return r;
		}
	}

	r = sdio_cmux_write(ch, (void *)buf, count);
	return r;
}


int sdio_ctl_open(struct inode *inode, struct file *file)
{
	int r = 0;
	struct sdio_ctl_dev *sdio_ctl_devp;

	sdio_ctl_devp = container_of(inode->i_cdev, struct sdio_ctl_dev, cdev);

	if (!sdio_ctl_devp)
		return -EINVAL;

	file->private_data = sdio_ctl_devp;
	D(KERN_INFO "%s called on sdioctl%d device\n",
		     __func__, sdio_ctl_devp->id);

	r = wait_event_timeout(
			sdio_ctl_devp->open_wait_queue,
			is_remote_open(sdio_ctl_devp->id),
			(1 * HZ));

	if (r < 0) {
		D(KERN_ERR "ERROR %s: wait_event_timeout() failed for"
			   " ch%d with rc %d\n", __func__,
			   sdio_ctl_devp->id, r);
		return r;
	}

	if (r == 0) {
		D(KERN_ERR "ERROR %s: Wait Timed Out for ch%d\n",
			    __func__, sdio_ctl_devp->id);
		return -ETIMEDOUT;
	}

	r = sdio_cmux_open(sdio_ctl_devp->id, &sdio_ctl_devp->ch,
			   NULL);
	if (r < 0) {
		D(KERN_ERR "ERROR %s: sdio_cmux_open failed with rc %d\n",
			    __func__, r);
	}
	mutex_lock(&sdio_ctl_devp->dev_lock);
	sdio_ctl_devp->ref_count++;
	mutex_unlock(&sdio_ctl_devp->dev_lock);

	return r;
}

int sdio_ctl_release(struct inode *inode, struct file *file)
{
	struct sdio_ctl_dev *sdio_ctl_devp;

	sdio_ctl_devp = file->private_data;
	if (!sdio_ctl_devp)
		return -EINVAL;

	D(KERN_INFO "%s called on sdioctl%d device\n",
		    __func__, sdio_ctl_devp->id);

	mutex_lock(&sdio_ctl_devp->dev_lock);
	if (sdio_ctl_devp->ref_count > 0) {
		sdio_ctl_devp->ref_count--;
		if (!sdio_ctl_devp->ref_count) {
			sdio_cmux_close(sdio_ctl_devp->ch);
			sdio_ctl_devp->ch = NULL;
		}
	}
	mutex_unlock(&sdio_ctl_devp->dev_lock);

	return 0;
}

static int process_ctl_pkt(void *pkt, int size, int copy)
{
	struct sdio_cmux_hdr *mux_hdr;
	struct sdio_ctl_list_elem *list_elem;
	uint32_t id;
	void *temp_pkt;
	char *dump_buf = (char *)pkt;
	unsigned long flags;

	D_DUMP_BUFFER("process_ctl_pkt:", size, dump_buf);
	mux_hdr = (struct sdio_cmux_hdr *)pkt;
	switch (mux_hdr->cmd) {
	case OPEN:
		id = (uint32_t)mux_hdr->lc_id;
		D(KERN_ERR "%s: Received OPEN command for ch%d\n",
				__func__, id);
		spin_lock_irqsave(&logical_ch[id].lc_lock, flags);
		logical_ch[id].is_remote_open = 1;
		spin_unlock_irqrestore(&logical_ch[id].lc_lock, flags);
		wake_up(&sdio_ctl_devp[id]->open_wait_queue);
		if (!copy)
			kfree(pkt);
		break;

	case CLOSE:
		id = (uint32_t)mux_hdr->lc_id;
		D(KERN_ERR "%s: Received CLOSE command for ch%d\n",
				__func__, id);
		sdio_cmux_ch_clear_and_signal(id);
		if (!copy)
			kfree(pkt);
		break;

	case DATA:
		id = (uint32_t)mux_hdr->lc_id;
		D(KERN_ERR "%s: Received DATA for ch%d\n", __func__, id);
		/*Channel is not locally open & if single packet received
		  then drop it*/
		spin_lock_irqsave(&logical_ch[id].lc_lock, flags);
		if (!logical_ch[id].is_remote_open) {
			spin_unlock_irqrestore(&logical_ch[id].lc_lock, flags);
			if (!copy)
				kfree(pkt);
			return -ENODEV;
		}

		/*Create a list elem to put the pkt into channel's rx_list*/
		list_elem = kmalloc(sizeof(struct sdio_ctl_list_elem),
				    GFP_KERNEL);
		if (!list_elem) {
			D(KERN_ERR "%s: list_elem kmalloc failed\n",
				    __func__);
			spin_unlock_irqrestore(&logical_ch[id].lc_lock,
						flags);
			if (!copy)
				kfree(pkt);
			return -ENOMEM;
		}

		/*If multiple packets received in single sdio_read
		  copy the contents of a packet & maintain it as an
		  element in the channel's rx_list*/
		if (copy) {
			temp_pkt = kmalloc(size, GFP_KERNEL);
			if (!temp_pkt) {
				D(KERN_ERR "%s: temp_pkt kmalloc failed\n",
					   __func__);
				spin_unlock_irqrestore(&logical_ch[id].lc_lock,
							flags);
				kfree(list_elem);
				return -ENOMEM;
			}
			memcpy(temp_pkt, pkt, size);
		} else
			temp_pkt = pkt;
		list_elem->ctl_pkt.hdr = (struct sdio_cmux_hdr *)temp_pkt;
		list_elem->ctl_pkt.data = (void *)((char *)temp_pkt +
					   sizeof(struct sdio_cmux_hdr));

		spin_lock(&logical_ch[id].rx_lock);
		list_add_tail(&list_elem->list,
			      &logical_ch[id].rx_list);
		logical_ch[id].read_avail += (int)mux_hdr->pkt_len;
		spin_unlock(&logical_ch[id].rx_lock);
		spin_unlock_irqrestore(&logical_ch[id].lc_lock, flags);

		wake_up(&sdio_ctl_devp[id]->read_wait_queue);
		break;
	}
	return 0;
}

static void parse_ctl_data(void *data, int size)
{
	int data_parsed = 0, multi_pkts = 0, pkt_size;
	char *temp_ptr;

	D(KERN_INFO "Entered %s\n", __func__);
	temp_ptr = (char *)data;
	if (size > (sizeof(struct sdio_cmux_hdr) +
		    (int)((struct sdio_cmux_hdr *)temp_ptr)->pkt_len)) {
		multi_pkts = 1;
		D(KERN_INFO "Multi packets in a single notification\n");
	}

	while (data_parsed < size) {
		pkt_size = sizeof(struct sdio_cmux_hdr) +
			   (int)((struct sdio_cmux_hdr *)temp_ptr)->pkt_len;
		D(KERN_INFO "Parsed %d bytes, Current Pkt Size %d bytes,"
			    " Total size %d bytes\n", data_parsed,
			     pkt_size, size);
		process_ctl_pkt((void *)temp_ptr, pkt_size, multi_pkts);
		data_parsed += pkt_size;
		temp_ptr += pkt_size;
	}

	if (multi_pkts)
		kfree(data);
}

static void sdio_demux_fn(struct work_struct *work)
{
	int r = 0, read_avail = 0;
	void *ctl_data;

	while (1) {
		read_avail = sdio_read_avail(sdio_ctl_chl);
		if (read_avail < 0) {
			D(KERN_ERR "%s: sdio_read_avail failed with rc %d\n",
			  __func__, read_avail);
			return;
		}

		if (read_avail == 0) {
			D(KERN_INFO "%s: Nothing to read\n", __func__);
			return;
		}

		D(KERN_INFO "%s: kmalloc %d bytes\n", __func__, read_avail);
		ctl_data = kmalloc(read_avail, GFP_KERNEL);
		if (!ctl_data) {
			D(KERN_ERR "%s: kmalloc Failed\n", __func__);
			return;
		}

		D(KERN_INFO "%s: sdio_read %d bytes\n", __func__, read_avail);
		r = sdio_read(sdio_ctl_chl, ctl_data, read_avail);
		if (r < 0) {
			D(KERN_ERR "%s: sdio_read failed with rc %d\n",
			  __func__, r);
			kfree(ctl_data);
			return;
		}

		parse_ctl_data(ctl_data, read_avail);
	}
	return;
}

static void sdio_mux_fn(struct work_struct *work)
{
	int r = 0;
	void *write_data;
	uint32_t write_size, write_avail, write_retry = 0;
	unsigned long flags;
	struct sdio_ctl_list_elem *list_elem = NULL;

	spin_lock_irqsave(&tx_lock, flags);
	while (!list_empty(&tx_list)) {
		list_elem = list_first_entry(&tx_list,
					     struct sdio_ctl_list_elem,
					     list);
		list_del(&list_elem->list);
		spin_unlock_irqrestore(&tx_lock, flags);

		write_data = (void *)list_elem->ctl_pkt.hdr;
		write_size = sizeof(struct sdio_cmux_hdr) +
				(uint32_t)list_elem->ctl_pkt.hdr->pkt_len;

		while ((write_avail = sdio_write_avail(sdio_ctl_chl))
					< write_size) {
			D(KERN_ERR "%s: sdio_write_avail %d bytes, "
					"write size %d bytes. Waiting...\n",
					__func__, write_avail, write_size);
			msleep(250);
		}
		while (((r = sdio_write(sdio_ctl_chl, write_data, write_size))
			< 0) && (write_retry++ < MAX_WRITE_RETRY)) {
			D(KERN_ERR "%s: sdio_write failed with rc %d."
					"Retrying...", __func__, r);
			msleep(250);
		}
		D(KERN_INFO "%s: sdio_write_completed %dbytes\n",
			     __func__, write_size);
		kfree(list_elem->ctl_pkt.hdr);
		kfree(list_elem);
		spin_lock_irqsave(&tx_lock, flags);
		bytes_to_write -= write_size;
		wake_up(&write_wait_queue);
	}
	spin_unlock_irqrestore(&tx_lock, flags);
	return;
}

static void sdio_ctl_chl_notify(void *priv, unsigned event)
{
	if (event == SDIO_EVENT_DATA_READ_AVAIL) {
		D(KERN_INFO "%s: Received SDIO_EVENT_DATA_READ_AVAIL\n",
			    __func__);
		queue_work(sdio_demux_wq, &sdio_demux_work);
	}
}

static const struct file_operations sdio_ctl_fops = {
	.owner = THIS_MODULE,
	.open = sdio_ctl_open,
	.release = sdio_ctl_release,
	.read = sdio_ctl_read,
	.write = sdio_ctl_write,
};

static int sdio_ctl_probe(struct platform_device *pdev)
{
	int i;
	int r;

	printk(KERN_INFO "%s Begins\n", __func__);
	r = alloc_chrdev_region(&sdio_ctl_number,
			       0,
			       NUM_SDIO_CTL_PORTS,
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

	sdio_ctl_classp = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(sdio_ctl_classp)) {
		printk(KERN_ERR "ERROR:%s:%i:%s: "
		       "class_create() ENOMEM\n",
		       __FILE__,
		       __LINE__,
		       __func__);
		r = -ENOMEM;
		goto error1;
	}

	for (i = 0; i < NUM_SDIO_CTL_PORTS; ++i) {
		sdio_ctl_devp[i] = kzalloc(sizeof(struct sdio_ctl_dev),
					 GFP_KERNEL);
		if (IS_ERR(sdio_ctl_devp[i])) {
			printk(KERN_ERR "ERROR:%s:%i:%s kmalloc() ENOMEM\n",
			       __FILE__,
			       __LINE__,
			       __func__);
			r = -ENOMEM;
			goto error2;
		}

		sdio_ctl_devp[i]->id = i;
		sdio_ctl_devp[i]->ch = NULL;
		sdio_ctl_devp[i]->ref_count = 0;

		mutex_init(&sdio_ctl_devp[i]->dev_lock);
		init_waitqueue_head(&sdio_ctl_devp[i]->read_wait_queue);
		init_waitqueue_head(&sdio_ctl_devp[i]->open_wait_queue);
		sdio_cmux_ch_alloc(i);

		cdev_init(&sdio_ctl_devp[i]->cdev, &sdio_ctl_fops);
		sdio_ctl_devp[i]->cdev.owner = THIS_MODULE;

		r = cdev_add(&sdio_ctl_devp[i]->cdev,
			     (sdio_ctl_number + i),
			     1);

		if (IS_ERR_VALUE(r)) {
			printk(KERN_ERR "%s:%i:%s: cdev_add() ret %i\n",
			       __FILE__,
			       __LINE__,
			       __func__,
			       r);
			kfree(sdio_ctl_devp[i]);
			goto error2;
		}

		sdio_ctl_devp[i]->devicep =
			device_create(sdio_ctl_classp,
				      NULL,
				      (sdio_ctl_number + i),
				      NULL,
				      DEVICE_NAME "%d",
				      i);

		if (IS_ERR(sdio_ctl_devp[i]->devicep)) {
			printk(KERN_ERR "%s:%i:%s: "
			       "device_create() ENOMEM\n",
			       __FILE__,
			       __LINE__,
			       __func__);
			r = -ENOMEM;
			cdev_del(&sdio_ctl_devp[i]->cdev);
			kfree(sdio_ctl_devp[i]);
			goto error2;
		}
	}

	bytes_to_write = 0;
	INIT_LIST_HEAD(&tx_list);
	spin_lock_init(&tx_lock);
	init_waitqueue_head(&write_wait_queue);

	sdio_mux_wq = create_singlethread_workqueue("sdio_mux");
	if (IS_ERR(sdio_mux_wq)) {
		printk(KERN_ERR "%s:%i:%s: create_singlethread_workqueue()"
				" ENOMEM\n", __FILE__, __LINE__, __func__);
		r = -ENOMEM;
		goto error2;
	}

	sdio_demux_wq = create_singlethread_workqueue("sdio_demux");
	if (IS_ERR(sdio_demux_wq)) {
		printk(KERN_ERR "%s:%i:%s: create_singlethread_workqueue()"
				" ENOMEM\n", __FILE__, __LINE__, __func__);
		destroy_workqueue(sdio_mux_wq);
		r = -ENOMEM;
		goto error2;
	}

	r = sdio_open("SDIO_QMI", &sdio_ctl_chl, NULL, sdio_ctl_chl_notify);
	if (r < 0) {
		D(KERN_ERR "%s:%i:%s: sdio_open() failed\n",
			   __FILE__, __LINE__, __func__);
		destroy_workqueue(sdio_mux_wq);
		destroy_workqueue(sdio_demux_wq);
		goto error2;
	}

	D(KERN_INFO "SDIO Control Port Driver Initialized.\n");
	return 0;

 error2:
	if (i > 0) {
		while (--i >= 0) {
			cdev_del(&sdio_ctl_devp[i]->cdev);
			kfree(sdio_ctl_devp[i]);
			device_destroy(sdio_ctl_classp,
				       MKDEV(MAJOR(sdio_ctl_number), i));
		}
	}

	class_destroy(sdio_ctl_classp);
 error1:
	unregister_chrdev_region(MAJOR(sdio_ctl_number), NUM_SDIO_CTL_PORTS);
 error0:
	return r;
}

static int sdio_ctl_remove(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < NUM_SDIO_CTL_PORTS; ++i) {
		cdev_del(&sdio_ctl_devp[i]->cdev);
		sdio_cmux_ch_clear_and_signal(i);
		kfree(sdio_ctl_devp[i]);
		device_destroy(sdio_ctl_classp,
			       MKDEV(MAJOR(sdio_ctl_number), i));
	}

	sdio_close(sdio_ctl_chl);
	destroy_workqueue(sdio_mux_wq);
	destroy_workqueue(sdio_demux_wq);

	class_destroy(sdio_ctl_classp);

	unregister_chrdev_region(MAJOR(sdio_ctl_number), NUM_SDIO_CTL_PORTS);
	return 0;
}

static struct platform_driver sdio_ctl_driver = {
	.probe		= sdio_ctl_probe,
	.remove		= sdio_ctl_remove,
	.driver		= {
			.name	= "SDIO_QMI",
			.owner	= THIS_MODULE,
	},
};

static int __init sdio_ctl_init(void)
{
	msm_sdio_ctl_debug_mask = 1;
	return platform_driver_register(&sdio_ctl_driver);
}

module_init(sdio_ctl_init);
MODULE_DESCRIPTION("MSM SDIO Control Port");
MODULE_LICENSE("GPL v2");
