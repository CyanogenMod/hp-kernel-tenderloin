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
 * RPCROUTER SDIO XPRT module.
 */

#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/wakelock.h>
#include <asm/uaccess.h>

#include <mach/sdio_al.h>
#include "smd_rpcrouter.h"

enum buf_state_type {
	FREE = 0,
	FULL,
	DATA_READY,
};

enum {
	MSM_SDIO_XPRT_DEBUG = 1U << 0,
	MSM_SDIO_XPRT_INFO = 1U << 1,
};

static int msm_sdio_xprt_debug_mask;
module_param_named(debug_mask, msm_sdio_xprt_debug_mask,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);

#if defined(CONFIG_MSM_RPC_SDIO_DEBUG)
#define SDIO_XPRT_DBG(x...) do {                \
	if (msm_sdio_xprt_debug_mask & MSM_SDIO_XPRT_DEBUG)     \
		printk(KERN_DEBUG x);           \
	} while (0)

#define SDIO_XPRT_INFO(x...) do {               \
	if (msm_sdio_xprt_debug_mask & MSM_SDIO_XPRT_INFO)      \
		printk(KERN_INFO x);            \
	} while (0)
#else
#define SDIO_XPRT_DBG(x...) do { } while (0)
#define SDIO_XPRT_INFO(x...) do { } while (0)
#endif

#define MAX_SDIO_WRITE_RETRY 5
#define SDIO_IN_BUF_SIZE 16384
#define NO_OF_SDIO_IN_BUF 1
struct sdio_in_buf {
	enum buf_state_type state;
	uint32_t size;
	uint32_t read_avail;
	uint32_t read_start_index;
	uint32_t read_end_index;
	void *in_buf;
};

struct sdio_xprt {
	struct sdio_channel *handle;
	spinlock_t lock;
	struct sdio_in_buf buffer[NO_OF_SDIO_IN_BUF];
};

struct rpcrouter_sdio_xprt {
	struct rpcrouter_xprt xprt;
	struct sdio_xprt *channel;
};

static struct rpcrouter_sdio_xprt sdio_remote_xprt;

static void sdio_xprt_read_data(struct work_struct *work);
static DECLARE_WORK(work_read_data, sdio_xprt_read_data);
static struct workqueue_struct *sdio_xprt_read_workqueue;
static wait_queue_head_t free_buf_wait;

static uint32_t active_in_buf;
static uint32_t work_queued;

static LIST_HEAD(write_list);
static DEFINE_SPINLOCK(write_list_lock);
struct sdio_write_data_struct {
	struct list_head list;
	uint32_t write_len;
	void *write_data;
};
static struct sdio_write_data_struct *sdio_write_pkt;
static void sdio_xprt_write_data(struct work_struct *work);
static DECLARE_WORK(work_write_data, sdio_xprt_write_data);
static wait_queue_head_t write_avail_wait_q;

static void free_sdio_xprt(struct sdio_xprt *chnl)
{
	int i;
	if (chnl) {
		for (i = 0; i < NO_OF_SDIO_IN_BUF; i++)
			kfree(chnl->buffer[i].in_buf);
		kfree(chnl);
	}
}

static int rpcrouter_sdio_remote_read_avail(void)
{
	int i, size = 0;
	unsigned long flags;
	SDIO_XPRT_DBG("sdio_xprt Called %s\n", __func__);

	spin_lock_irqsave(&sdio_remote_xprt.channel->lock, flags);
	for (i = 0; i < NO_OF_SDIO_IN_BUF; i++)
		size += sdio_remote_xprt.channel->buffer[i].read_avail;

	spin_unlock_irqrestore(&sdio_remote_xprt.channel->lock, flags);

	return size;
}

static int sdio_buf_read(void *data, uint32_t len)
{
	uint32_t xfer = len;
	uint32_t avail;
	unsigned char *src;
	struct sdio_in_buf *active_buf;

	SDIO_XPRT_DBG("sdio_xprt Called %s\n", __func__);

	active_buf = &sdio_remote_xprt.channel->buffer[active_in_buf];
	while ((xfer > 0) && (active_buf->read_avail > 0)) {
		src = ((unsigned char *)active_buf->in_buf +
					active_buf->read_end_index);
		avail = SDIO_IN_BUF_SIZE - active_buf->read_end_index;

		if (avail > active_buf->read_avail)
			avail = active_buf->read_avail;

		if (avail > xfer)
			avail = xfer;

		memcpy(data, src, avail);
		active_buf->size -= avail;
		active_buf->read_avail -= avail;
		active_buf->read_end_index += avail;
		active_buf->read_end_index &= (SDIO_IN_BUF_SIZE - 1);
		data = (void *)((unsigned char *)data + avail);
		xfer -= avail;
		if (active_buf->read_avail <= 0 && (!work_queued)) {
			active_buf->read_start_index = 0;
			active_buf->read_end_index = 0;
			active_buf->size = 0;
			active_buf->state = FREE;
		} else
			active_buf->state = DATA_READY;
	}
	return len - xfer;
}

static int rpcrouter_sdio_remote_read(void *data, uint32_t len)
{
	uint32_t i, xfer = 0, size;
	unsigned long flags;

	SDIO_XPRT_DBG("sdio_xprt Called %s\n", __func__);
	if (len < 0 || !data)
		return -EINVAL;
	else if (len == 0)
		return 0;

	spin_lock_irqsave(&sdio_remote_xprt.channel->lock, flags);
	for (i = 0; i < NO_OF_SDIO_IN_BUF; i++)
		xfer += sdio_remote_xprt.channel->buffer[i].read_avail;

	if (xfer < len) {
		spin_unlock_irqrestore(&sdio_remote_xprt.channel->lock, flags);
		return -EINVAL;
	}
	xfer = len;

	size = sdio_buf_read(data, xfer);
	spin_unlock_irqrestore(&sdio_remote_xprt.channel->lock, flags);

	return size;
}

static int rpcrouter_sdio_remote_write_avail(void)
{
	SDIO_XPRT_DBG("sdio_xprt Called %s\n", __func__);
	return sdio_write_avail(sdio_remote_xprt.channel->handle);
}

static int rpcrouter_sdio_remote_write(void *data, uint32_t len,
					enum write_data_type type)
{
	unsigned long flags;
	static void *buf;

	switch (type) {
	case HEADER:
		SDIO_XPRT_DBG("sdio_xprt WRITE HEADER %s\n", __func__);
		sdio_write_pkt = kmalloc(sizeof(struct sdio_write_data_struct),
					 GFP_KERNEL);
		sdio_write_pkt->write_len = len +
					    ((struct rr_header *)data)->size;
		sdio_write_pkt->write_data = kmalloc(sdio_write_pkt->write_len,
						     GFP_KERNEL);
		buf = sdio_write_pkt->write_data;
		memcpy(buf, data, len);
		buf = (void *)((unsigned char *)buf + len);
		return len;
	case PACKMARK:
		SDIO_XPRT_DBG("sdio_xprt WRITE PACKMARK %s\n",	__func__);
		memcpy(buf, data, len);
		buf = (void *)((unsigned char *)buf + len);
		return len;
	case PAYLOAD:
		SDIO_XPRT_DBG("sdio_xprt WRITE PAYLOAD %s \n",	__func__);
		memcpy(buf, data, len);

		SDIO_XPRT_DBG("sdio_xprt flush %d bytes\n",
				sdio_write_pkt->write_len);
		spin_lock_irqsave(&write_list_lock, flags);
		list_add_tail(&sdio_write_pkt->list, &write_list);
		spin_unlock_irqrestore(&write_list_lock, flags);
		queue_work(sdio_xprt_read_workqueue, &work_write_data);
		return len;
	default:
		return -EINVAL;
	}
}

static void sdio_xprt_write_data(struct work_struct *work)
{
	int rc = 0, sdio_write_retry = 0;
	unsigned long flags;
	struct sdio_write_data_struct *sdio_write_data;

	spin_lock_irqsave(&write_list_lock, flags);
	while (!list_empty(&write_list)) {
		sdio_write_data = list_first_entry(&write_list,
					struct sdio_write_data_struct,
					list);
		list_del(&sdio_write_data->list);
		spin_unlock_irqrestore(&write_list_lock, flags);

		wait_event(write_avail_wait_q,
			   (sdio_write_avail(
			   sdio_remote_xprt.channel->handle) >=
			   sdio_write_data->write_len));
		while (((rc = sdio_write(sdio_remote_xprt.channel->handle,
					sdio_write_data->write_data,
					sdio_write_data->write_len)) < 0) &&
			(sdio_write_retry++ < MAX_SDIO_WRITE_RETRY)) {
			printk(KERN_ERR "sdio_write failed with RC %d\n",
					rc);
			msleep(250);
		}
		if (!rc)
			SDIO_XPRT_DBG("sdio_write %d bytes completed\n",
					sdio_write_data->write_len);

		kfree(sdio_write_data->write_data);
		kfree(sdio_write_data);
		spin_lock_irqsave(&write_list_lock, flags);
	}
	spin_unlock_irqrestore(&write_list_lock, flags);
}

static int rpcrouter_sdio_remote_close(void)
{
	SDIO_XPRT_DBG("sdio_xprt Called %s\n", __func__);
	sdio_close(sdio_remote_xprt.channel->handle);
	free_sdio_xprt(sdio_remote_xprt.channel);
	return 0;
}

static void sdio_xprt_read_data(struct work_struct *work)
{
	int i, size = 0, avail, read_avail;
	unsigned long flags;
	unsigned char *data;
	struct sdio_in_buf *input_buffer;
	SDIO_XPRT_DBG("sdio_xprt Called %s\n", __func__);

find_buf:
	spin_lock_irqsave(&sdio_remote_xprt.channel->lock, flags);
	for (i = 0; i < NO_OF_SDIO_IN_BUF; i++) {
		if (sdio_remote_xprt.channel->buffer[i].state != FULL)
			break;
	}

	if (i >= NO_OF_SDIO_IN_BUF) {
		spin_unlock_irqrestore(&sdio_remote_xprt.channel->lock, flags);
		msm_rpcrouter_xprt_notify(&sdio_remote_xprt.xprt,
					   RPCROUTER_XPRT_EVENT_DATA);
		SDIO_XPRT_DBG("sdio_xprt enter wait:\n");
		wait_event_timeout(free_buf_wait,
			   (sdio_remote_xprt.channel->buffer[0].state != FULL),
				(1 * HZ));
		SDIO_XPRT_DBG("sdio_xprt exit wait:\n");
		goto find_buf;
	}

	work_queued = 1;
	input_buffer = &sdio_remote_xprt.channel->buffer[i];
	avail = SDIO_IN_BUF_SIZE - input_buffer->size;

	while ((read_avail = sdio_read_avail(sdio_remote_xprt.channel->handle))
		&& avail) {
		data = (unsigned char *)input_buffer->in_buf +
					input_buffer->read_start_index;

		/* Trim to end of buffer */
		avail = min(read_avail, avail);
		if (avail >
		    (SDIO_IN_BUF_SIZE - input_buffer->read_start_index)) {
			avail = (SDIO_IN_BUF_SIZE -
				 input_buffer->read_start_index);
		}
		spin_unlock_irqrestore(&sdio_remote_xprt.channel->lock, flags);

		size = sdio_read(sdio_remote_xprt.channel->handle,
				 data, avail);
		spin_lock_irqsave(&sdio_remote_xprt.channel->lock, flags);
		if (size < 0) {
			work_queued = 0;
			spin_unlock_irqrestore(&sdio_remote_xprt.channel->lock,
						 flags);
			printk(KERN_ERR "sdio_read failed,"
					" read %d bytes, expected %d \n",
					size, avail);
			return;
		}

		if (size == 0)
			size = avail;

		input_buffer->size += size;
		input_buffer->read_avail += size;
		input_buffer->read_start_index += size;
		input_buffer->read_start_index &= (SDIO_IN_BUF_SIZE - 1);
		input_buffer->state = DATA_READY;
		avail = SDIO_IN_BUF_SIZE - input_buffer->size;
	}
	if (input_buffer->size >= SDIO_IN_BUF_SIZE)
		input_buffer->state = FULL;

	work_queued = 0;
	spin_unlock_irqrestore(&sdio_remote_xprt.channel->lock, flags);

	msm_rpcrouter_xprt_notify(&sdio_remote_xprt.xprt,
				  RPCROUTER_XPRT_EVENT_DATA);
	SDIO_XPRT_DBG("%s Notify Router Event Data\n", __func__);
	for (i = 0; i < NO_OF_SDIO_IN_BUF; i++) {
		SDIO_XPRT_DBG("sdio_xprt Buffer %d: RA %4d bytes,"
			"SI: %4d, EI %4d\n", i,
			sdio_remote_xprt.channel->buffer[i].read_avail,
			sdio_remote_xprt.channel->buffer[0].read_start_index,
			sdio_remote_xprt.channel->buffer[0].read_end_index);
	}
}

static void rpcrouter_sdio_remote_notify(void *_dev, unsigned event)
{
	if (event == SDIO_EVENT_DATA_READ_AVAIL) {
		SDIO_XPRT_DBG("%s Received Notify"
			      "SDIO_EVENT_DATA_READ_AVAIL\n", __func__);
		queue_work(sdio_xprt_read_workqueue, &work_read_data);
	}
	if (event == SDIO_EVENT_DATA_WRITE_AVAIL) {
		SDIO_XPRT_DBG("%s Received Notify"
			      "SDIO_EVENT_DATA_WRITE_AVAIL\n", __func__);
		wake_up(&write_avail_wait_q);
	}
}

static int allocate_sdio_xprt(struct sdio_xprt **sdio_xprt_chnl)
{
	void *buf;
	struct sdio_xprt *chnl;
	int i;
	int rc = -ENOMEM;

	buf = kmalloc(SDIO_IN_BUF_SIZE, GFP_KERNEL);
	if (!buf) {
		printk(KERN_ERR "sdio_in_buf allocation failed\n");
		return -ENOMEM;
	}

	chnl = kmalloc(sizeof(struct sdio_xprt), GFP_KERNEL);
	if (!chnl) {
		printk(KERN_ERR "sdio_xprt channel allocation failed\n");
		goto alloc_failure;
	}

	spin_lock_init(&chnl->lock);

	for (i = 0; i < NO_OF_SDIO_IN_BUF; i++) {
		chnl->buffer[i].state = FREE;
		chnl->buffer[i].size = 0;
		chnl->buffer[i].read_avail = 0;
		chnl->buffer[i].read_start_index = 0;
		chnl->buffer[i].read_end_index = 0;
	}
	chnl->buffer[0].in_buf = buf;

	*sdio_xprt_chnl = chnl;
	return 0;

alloc_failure:
	kfree(buf);

	return rc;
}

static int rpcrouter_sdio_remote_probe(struct platform_device *pdev)
{
	int rc;

	SDIO_XPRT_INFO("%s Called\n", __func__);
	rc = allocate_sdio_xprt(&sdio_remote_xprt.channel);
	if (rc)
		return rc;

	sdio_xprt_read_workqueue = create_singlethread_workqueue("sdio_xprt");
	if (!sdio_xprt_read_workqueue) {
		free_sdio_xprt(sdio_remote_xprt.channel);
		return -ENOMEM;
	}

	sdio_remote_xprt.xprt.name = "rpcrotuer_sdio_xprt";
	sdio_remote_xprt.xprt.read_avail = rpcrouter_sdio_remote_read_avail;
	sdio_remote_xprt.xprt.read = rpcrouter_sdio_remote_read;
	sdio_remote_xprt.xprt.write_avail = rpcrouter_sdio_remote_write_avail;
	sdio_remote_xprt.xprt.write = rpcrouter_sdio_remote_write;
	sdio_remote_xprt.xprt.close = rpcrouter_sdio_remote_close;
	sdio_remote_xprt.xprt.priv = NULL;

	init_waitqueue_head(&free_buf_wait);
	init_waitqueue_head(&write_avail_wait_q);

	INIT_LIST_HEAD(&write_list);

	/* Open up SDIO channel */
	rc = sdio_open("SDIO_RPC", &sdio_remote_xprt.channel->handle, NULL,
		      rpcrouter_sdio_remote_notify);

	if (rc < 0) {
		free_sdio_xprt(sdio_remote_xprt.channel);
		destroy_workqueue(sdio_xprt_read_workqueue);
		return rc;
	}

	msm_rpcrouter_xprt_notify(&sdio_remote_xprt.xprt,
				  RPCROUTER_XPRT_EVENT_OPEN);

	SDIO_XPRT_INFO("%s Completed\n", __func__);

	return 0;
}

/*Remove this platform driver after mainline of SDIO_AL update*/
static struct platform_driver rpcrouter_sdio_remote_driver = {
	.probe		= rpcrouter_sdio_remote_probe,
	.driver		= {
			.name	= "SDIO_AL",
			.owner	= THIS_MODULE,
	},
};

static struct platform_driver rpcrouter_sdio_driver = {
	.probe		= rpcrouter_sdio_remote_probe,
	.driver		= {
			.name	= "SDIO_RPC",
			.owner	= THIS_MODULE,
	},
};

static int __init rpcrouter_sdio_init(void)
{
	int rc;
	msm_sdio_xprt_debug_mask = 0x2;
	rc = platform_driver_register(&rpcrouter_sdio_remote_driver);
	if (rc < 0)
		return rc;
	return platform_driver_register(&rpcrouter_sdio_driver);
}

module_init(rpcrouter_sdio_init);
MODULE_DESCRIPTION("RPC Router SDIO XPRT");
MODULE_LICENSE("GPL v2");
