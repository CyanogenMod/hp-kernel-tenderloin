/*
 *  linux/drivers/mmc/core/host.c
 *
 *  Copyright (C) 2003 Russell King, All Rights Reserved.
 *  Copyright (C) 2007-2008 Pierre Ossman
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  MMC host class device management
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/idr.h>
#include <linux/pagemap.h>
#include <linux/leds.h>
#include <linux/slab.h>

#include <linux/mmc/host.h>
#include <linux/suspend.h>

#include "core.h"
#include "host.h"

#define cls_dev_to_mmc_host(d)	container_of(d, struct mmc_host, class_dev)

static void mmc_host_classdev_release(struct device *dev)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	kfree(host);
}

static struct class mmc_host_class = {
	.name		= "mmc_host",
	.dev_release	= mmc_host_classdev_release,
};

int mmc_register_host_class(void)
{
	return class_register(&mmc_host_class);
}

void mmc_unregister_host_class(void)
{
	class_unregister(&mmc_host_class);
}

static DEFINE_IDR(mmc_host_idr);
static DEFINE_SPINLOCK(mmc_host_lock);

/**
 *	mmc_alloc_host - initialise the per-host structure.
 *	@extra: sizeof private data structure
 *	@dev: pointer to host device model structure
 *
 *	Initialise the per-host structure.
 */
struct mmc_host *mmc_alloc_host(int extra, struct device *dev)
{
	int err;
	struct mmc_host *host;

	if (!idr_pre_get(&mmc_host_idr, GFP_KERNEL))
		return NULL;

	host = kzalloc(sizeof(struct mmc_host) + extra, GFP_KERNEL);
	if (!host)
		return NULL;

	spin_lock(&mmc_host_lock);
	err = idr_get_new(&mmc_host_idr, host, &host->index);
	spin_unlock(&mmc_host_lock);
	if (err)
		goto free;

	dev_set_name(&host->class_dev, "mmc%d", host->index);

	host->parent = dev;
	host->class_dev.parent = dev;
	host->class_dev.class = &mmc_host_class;
	device_initialize(&host->class_dev);

	spin_lock_init(&host->lock);
	init_waitqueue_head(&host->wq);
	INIT_DELAYED_WORK(&host->detect, mmc_rescan);
	INIT_DELAYED_WORK_DEFERRABLE(&host->disable, mmc_host_deeper_disable);
#ifdef CONFIG_PM
	host->pm_notify.notifier_call = mmc_pm_notify;
#endif

	/*
	 * By default, hosts do not support SGIO or large requests.
	 * They have to set these according to their abilities.
	 */
	host->max_hw_segs = 1;
	host->max_phys_segs = 1;
	host->max_seg_size = PAGE_CACHE_SIZE;

	host->max_req_size = PAGE_CACHE_SIZE;
	host->max_blk_size = 512;
	host->max_blk_count = PAGE_CACHE_SIZE / 512;

	return host;

free:
	kfree(host);
	return NULL;
}

EXPORT_SYMBOL(mmc_alloc_host);
#ifdef CONFIG_MMC_PERF_PROFILING
static ssize_t
show_perf(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = dev_get_drvdata(dev);
	int64_t rtime_mmcq, wtime_mmcq, rtime_drv, wtime_drv;
	unsigned long rbytes_mmcq, wbytes_mmcq, rbytes_drv, wbytes_drv;

	spin_lock(&host->lock);

	rbytes_mmcq = host->perf.rbytes_mmcq;
	wbytes_mmcq = host->perf.wbytes_mmcq;
	rbytes_drv = host->perf.rbytes_drv;
	wbytes_drv = host->perf.wbytes_drv;

	rtime_mmcq = ktime_to_us(host->perf.rtime_mmcq);
	wtime_mmcq = ktime_to_us(host->perf.wtime_mmcq);
	rtime_drv = ktime_to_us(host->perf.rtime_drv);
	wtime_drv = ktime_to_us(host->perf.wtime_drv);

	spin_unlock(&host->lock);

	return snprintf(buf, PAGE_SIZE, "Write performance at MMCQ Level:"
					"%lu bytes in %lld microseconds\n"
					"Read performance at MMCQ Level:"
					"%lu bytes in %lld microseconds\n"
					"Write performance at driver Level:"
					"%lu bytes in %lld microseconds\n"
					"Read performance at driver Level:"
					"%lu bytes in %lld microseconds\n",
					wbytes_mmcq, wtime_mmcq, rbytes_mmcq,
					rtime_mmcq, wbytes_drv, wtime_drv,
					rbytes_drv, rtime_drv);
}

static ssize_t
set_perf(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int64_t value;
	struct mmc_host *host = dev_get_drvdata(dev);
	sscanf(buf, "%lld", &value);
	if (!value) {
		spin_lock(&host->lock);
		memset(&host->perf, 0, sizeof(host->perf));
		spin_unlock(&host->lock);
	}

	return count;
}

static DEVICE_ATTR(perf, S_IRUGO | S_IWUSR,
		show_perf, set_perf);
#endif

static struct attribute *dev_attrs[] = {
#ifdef CONFIG_MMC_PERF_PROFILING
	&dev_attr_perf.attr,
#endif
	NULL,
};
static struct attribute_group dev_attr_grp = {
	.attrs = dev_attrs,
};

/**
 *	mmc_add_host - initialise host hardware
 *	@host: mmc host
 *
 *	Register the host with the driver model. The host must be
 *	prepared to start servicing requests before this function
 *	completes.
 */
int mmc_add_host(struct mmc_host *host)
{
	int err;

	WARN_ON((host->caps & MMC_CAP_SDIO_IRQ) &&
		!host->ops->enable_sdio_irq);

	led_trigger_register_simple(dev_name(&host->class_dev), &host->led);

	err = device_add(&host->class_dev);
	if (err)
		return err;

#ifdef CONFIG_DEBUG_FS
	mmc_add_host_debugfs(host);
#endif
	err = sysfs_create_group(&host->parent->kobj, &dev_attr_grp);
	if (err)
		pr_err("%s: failed to create sysfs group with err %d\n",
							 __func__, err);

	mmc_start_host(host);
	if (!(host->pm_flags & MMC_PM_IGNORE_PM_NOTIFY))
		register_pm_notifier(&host->pm_notify);

	return 0;
}

EXPORT_SYMBOL(mmc_add_host);

/**
 *	mmc_remove_host - remove host hardware
 *	@host: mmc host
 *
 *	Unregister and remove all cards associated with this host,
 *	and power down the MMC bus. No new requests will be issued
 *	after this function has returned.
 */
void mmc_remove_host(struct mmc_host *host)
{
	if (!(host->pm_flags & MMC_PM_IGNORE_PM_NOTIFY))
		unregister_pm_notifier(&host->pm_notify);

	mmc_stop_host(host);

#ifdef CONFIG_DEBUG_FS
	mmc_remove_host_debugfs(host);
#endif
	sysfs_remove_group(&host->parent->kobj, &dev_attr_grp);


	device_del(&host->class_dev);

	led_trigger_unregister_simple(host->led);

}

EXPORT_SYMBOL(mmc_remove_host);

/**
 *	mmc_free_host - free the host structure
 *	@host: mmc host
 *
 *	Free the host once all references to it have been dropped.
 */
void mmc_free_host(struct mmc_host *host)
{
	spin_lock(&mmc_host_lock);
	idr_remove(&mmc_host_idr, host->index);
	spin_unlock(&mmc_host_lock);

	put_device(&host->class_dev);
}

EXPORT_SYMBOL(mmc_free_host);

