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
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/reboot.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/stringify.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/uaccess.h>

#include <asm-generic/poll.h>

#include "ramdump.h"

#define RAMDUMP_WAIT_MSECS	120000

struct ramdump_device {
	char name[256];

	unsigned long ramdump_size;
	unsigned long ramdump_begin_addr;
	unsigned int data_ready;
	unsigned int consumer_present;
	int ramdump_status;

	struct completion ramdump_complete;
	struct miscdevice device;

	wait_queue_head_t dump_wait_q;
};

static int ramdump_open(struct inode *inode, struct file *filep)
{
	struct ramdump_device *rd_dev = container_of(filep->private_data,
				struct ramdump_device, device);
	rd_dev->consumer_present = 1;
	return 0;
}

static int ramdump_release(struct inode *inode, struct file *filep)
{
	struct ramdump_device *rd_dev = container_of(filep->private_data,
				struct ramdump_device, device);
	rd_dev->consumer_present = 0;
	rd_dev->data_ready = 0;
	complete(&rd_dev->ramdump_complete);
	return 0;
}

static int ramdump_mmap(struct file *filep, struct vm_area_struct *vma)
{
	int ret = 0;
	struct ramdump_device *rd_dev = container_of(filep->private_data,
				struct ramdump_device, device);

	if (rd_dev->data_ready == 0) {
		pr_err("Ramdump(%s): Mmap when there's no dump available!",
			rd_dev->name);
		return -EINVAL;
	}

	if (vma->vm_end - vma->vm_start != rd_dev->ramdump_size) {
		pr_err("Ramdump(%s): Invalid size passed in from userspace in mmap.",
			rd_dev->name);
		ret = -EINVAL;
		goto err;
	}

	vma->vm_flags |= (VM_IO | VM_RESERVED);
	vma->vm_page_prot = PAGE_READONLY;

	ret = io_remap_pfn_range(vma, vma->vm_start,
		rd_dev->ramdump_begin_addr >> PAGE_SHIFT,
		vma->vm_end - vma->vm_start, vma->vm_page_prot);

	if (ret) {
		pr_err("Ramdump(%s): io_remap_pfn_range failed (ret = %d)",
			rd_dev->name, ret);
		goto err;
	}

	return 0;

err:
	rd_dev->data_ready = 0;
	complete(&rd_dev->ramdump_complete);
	return ret;
}

static long ramdump_unlocked_ioctl(struct file *filep, unsigned int cmd,
					unsigned long arg)
{
	int ret = 0;
	struct ramdump_device *rd_dev = container_of(filep->private_data,
				struct ramdump_device, device);

	if (_IOC_TYPE(cmd) != RAMDUMP_IOCTL_CODE) {
		pr_err("%s: invalid ioctl code\n", __func__);
		return -EINVAL;
	}

	if (rd_dev->data_ready == 0) {
		pr_err("Ramdump(%s): Ioctl when there's no dump available!",
			rd_dev->name);
		return -EINVAL;
	}

	switch (cmd) {

	case RAMDUMP_SIZE:
		put_user(rd_dev->ramdump_size, (unsigned long __user *) arg);
	break;

	case RAMDUMP_COMPLETE:
		rd_dev->data_ready = 0;
		get_user(rd_dev->ramdump_status, (unsigned long __user *) arg);
		complete(&rd_dev->ramdump_complete);
	break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static unsigned int ramdump_poll(struct file *filep,
					struct poll_table_struct *wait)
{
	struct ramdump_device *rd_dev = container_of(filep->private_data,
				struct ramdump_device, device);
	unsigned int mask = 0;

	if (rd_dev->data_ready)
		mask |= (POLLIN | POLLRDNORM);

	poll_wait(filep, &rd_dev->dump_wait_q, wait);
	return mask;
}

const struct file_operations ramdump_file_ops = {
	.open = ramdump_open,
	.release = ramdump_release,
	.unlocked_ioctl = ramdump_unlocked_ioctl,
	.mmap = ramdump_mmap,
	.poll = ramdump_poll
};

void *create_ramdump_device(const char *dev_name)
{
	int ret;
	struct ramdump_device *rd_dev;

	if (!dev_name) {
		pr_err("%s: Invalid device name.\n", __func__);
		return NULL;
	}

	rd_dev = kzalloc(sizeof(struct ramdump_device), GFP_KERNEL);

	if (!rd_dev) {
		pr_err("%s: Couldn't alloc space for ramdump device!",
			__func__);
		return NULL;
	}

	strncpy(rd_dev->name, "ramdump_", 256);
	strncat(rd_dev->name, dev_name, 256);

	init_completion(&rd_dev->ramdump_complete);

	rd_dev->device.minor = MISC_DYNAMIC_MINOR;
	rd_dev->device.name = rd_dev->name;
	rd_dev->device.fops = &ramdump_file_ops;

	init_waitqueue_head(&rd_dev->dump_wait_q);

	ret = misc_register(&rd_dev->device);

	if (ret) {
		pr_err("%s: misc_register failed for %s (%d)", __func__,
				dev_name, ret);
		kfree(rd_dev);
		return NULL;
	}

	return (void *)rd_dev;
}

int do_ramdump(void *handle, unsigned long addr, unsigned long size)
{
	int ret;
	struct ramdump_device *rd_dev = (struct ramdump_device *)handle;

	if (!rd_dev->consumer_present) {
		pr_err("Ramdump(%s): No consumers. Aborting..\n", rd_dev->name);
		return -EPIPE;
	}

	rd_dev->ramdump_begin_addr = addr;
	rd_dev->ramdump_size = size;

	rd_dev->data_ready = 1;
	rd_dev->ramdump_status = -1;

	/* Tell userspace that the data is ready */
	wake_up(&rd_dev->dump_wait_q);

	/* Wait (with a timeout) to let the ramdump complete */
	ret = wait_for_completion_timeout(&rd_dev->ramdump_complete,
			msecs_to_jiffies(RAMDUMP_WAIT_MSECS));

	INIT_COMPLETION(rd_dev->ramdump_complete);

	if (!ret) {
		pr_err("Ramdump(%s): Timed out waiting for userspace.\n",
			rd_dev->name);
		ret = -EPIPE;
	} else
		ret = (rd_dev->ramdump_status == 0) ? 0 : -EPIPE;

	rd_dev->data_ready = 0;

	return ret;
}
