/* Copyright (c) 2002,2007-2011, Code Aurora Forum. All rights reserved.
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
#ifndef _KGSL_DEVICE_H
#define _KGSL_DEVICE_H

#include <linux/types.h>
#include <linux/irqreturn.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/msm_kgsl.h>
#include <linux/idr.h>
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>

#include <asm/atomic.h>

#include "kgsl_mmu.h"
#include "kgsl_pwrctrl.h"
#include "kgsl_log.h"

#define KGSL_TIMEOUT_NONE       0
#define KGSL_TIMEOUT_DEFAULT    0xFFFFFFFF

#define FIRST_TIMEOUT (HZ / 2)

#define KGSL_CHIPID_YAMATODX_REV21  0x20100
#define KGSL_CHIPID_YAMATODX_REV211 0x20101
#define KGSL_CHIPID_LEIA_REV470_TEMP 0x10001
#define KGSL_CHIPID_LEIA_REV470 0x2010000

/* KGSL device state is initialized to INIT when platform_probe		*
 * sucessfully initialized the device.  Once a device has been opened	*
 * (started) it becomes active.  NAP implies that only low latency	*
 * resources (for now clocks on some platforms) are off.  SLEEP implies	*
 * that the KGSL module believes a device is idle (has been inactive	*
 * past its timer) and all system resources are released.  SUSPEND is	*
 * requested by the kernel and will be enforced upon all open devices.	*/

#define KGSL_STATE_NONE		0x00000000
#define KGSL_STATE_INIT		0x00000001
#define KGSL_STATE_ACTIVE	0x00000002
#define KGSL_STATE_NAP		0x00000004
#define KGSL_STATE_SLEEP	0x00000008
#define KGSL_STATE_SUSPEND	0x00000010
#define KGSL_STATE_HUNG		0x00000020
#define KGSL_STATE_DUMP_AND_RECOVER	0x00000040

#define KGSL_GRAPHICS_MEMORY_LOW_WATERMARK  0x1000000

#define KGSL_IS_PAGE_ALIGNED(addr) (!((addr) & (~PAGE_MASK)))

struct kgsl_device;
struct platform_device;
struct kgsl_device_private;
struct kgsl_context;
struct kgsl_power_stats;

struct kgsl_functable {
	void (*device_regread) (struct kgsl_device *device,
					unsigned int offsetwords,
					unsigned int *value);
	void (*device_regwrite) (struct kgsl_device *device,
					unsigned int offsetwords,
					unsigned int value);
	void (*device_regread_isr) (struct kgsl_device *device,
				    unsigned int offsetwords,
				    unsigned int *value);
	void (*device_regwrite_isr) (struct kgsl_device *device,
				     unsigned int offsetwords,
				     unsigned int value);
	int (*device_setstate) (struct kgsl_device *device, uint32_t flags);
	int (*device_idle) (struct kgsl_device *device, unsigned int timeout);
	unsigned int (*device_isidle) (struct kgsl_device *device);
	int (*device_suspend_context) (struct kgsl_device *device);
	int (*device_resume_context) (struct kgsl_device *device);
	int (*device_start) (struct kgsl_device *device, unsigned int init_ram);
	int (*device_stop) (struct kgsl_device *device);
	int (*device_getproperty) (struct kgsl_device *device,
					enum kgsl_property_type type,
					void *value,
					unsigned int sizebytes);
	int (*device_waittimestamp) (struct kgsl_device *device,
					unsigned int timestamp,
					unsigned int msecs);
	unsigned int (*device_cmdstream_readtimestamp) (
					struct kgsl_device *device,
					enum kgsl_timestamp_type type);
	int (*device_issueibcmds) (struct kgsl_device_private *dev_priv,
				struct kgsl_context *context,
				struct kgsl_ibdesc *ibdesc,
				unsigned int sizedwords,
				uint32_t *timestamp,
				unsigned int flags);
	int (*device_drawctxt_create) (struct kgsl_device_private *dev_priv,
					uint32_t flags,
					struct kgsl_context *context);
	int (*device_drawctxt_destroy) (struct kgsl_device *device,
					struct kgsl_context *context);
	long (*device_ioctl) (struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
	int (*device_setup_pt)(struct kgsl_device *device,
			       struct kgsl_pagetable *pagetable);

	int (*device_cleanup_pt)(struct kgsl_device *device,
				 struct kgsl_pagetable *pagetable);
	void (*device_power_stats)(struct kgsl_device *device,
		struct kgsl_power_stats *stats);
};

struct kgsl_memregion {
	unsigned char  *mmio_virt_base;
	unsigned int   mmio_phys_base;
	uint32_t      gpu_base;
	unsigned int   sizebytes;
};

struct kgsl_device {
	struct device *dev;
	const char *name;
	unsigned int ver_major;
	unsigned int ver_minor;
	uint32_t       flags;
	enum kgsl_deviceid    id;
	unsigned int      chip_id;
	struct kgsl_memregion regspace;
	struct kgsl_memdesc memstore;
	const char *iomemname;

	struct kgsl_mmu 	  mmu;
	struct completion hwaccess_gate;
	struct kgsl_functable ftbl;
	struct work_struct idle_check_ws;
	struct timer_list idle_timer;
	struct kgsl_pwrctrl pwrctrl;
	int open_count;

	struct atomic_notifier_head ts_notifier_list;
	struct mutex mutex;
	uint32_t		state;
	uint32_t		requested_state;

	struct list_head memqueue;
	unsigned int active_cnt;
	struct completion suspend_gate;

	wait_queue_head_t wait_queue;
	struct workqueue_struct *work_queue;
	struct platform_device *pdev;
	struct completion recovery_gate;
	struct dentry *d_debugfs;
	struct idr context_idr;
	struct early_suspend display_off;

	/* Logging levels */
	int cmd_log;
	int ctxt_log;
	int drv_log;
	int mem_log;
	int pwr_log;
	struct wake_lock idle_wakelock;
};

struct kgsl_context {
	uint32_t id;

	/* Pointer to the owning device instance */
	struct kgsl_device_private *dev_priv;

	/* Pointer to the device specific context information */
	void *devctxt;
};

struct kgsl_process_private {
	unsigned int refcnt;
	pid_t pid;
	spinlock_t mem_lock;
	struct list_head mem_list;
	struct kgsl_pagetable *pagetable;
	struct list_head list;
	struct kobject *kobj;

	struct {
		unsigned int vmalloc;
		unsigned int vmalloc_max;
		unsigned int exmem;
		unsigned int exmem_max;
		unsigned int flushes;
	} stats;
};

struct kgsl_device_private {
	struct kgsl_device *device;
	struct kgsl_process_private *process_priv;
};

struct kgsl_devconfig {
	struct kgsl_memregion regspace;

	unsigned int     mmu_config;
	uint32_t        mpu_base;
	int              mpu_range;
	uint32_t        va_base;
	unsigned int     va_range;

	struct kgsl_memregion gmemspace;
};

struct kgsl_power_stats {
	s64 total_time;
	s64 busy_time;
};

struct kgsl_device *kgsl_get_device(int dev_idx);

static inline struct kgsl_mmu *
kgsl_get_mmu(struct kgsl_device *device)
{
	return (struct kgsl_mmu *) (device ? &device->mmu : NULL);
}

static inline int kgsl_create_device_workqueue(struct kgsl_device *device)
{
	device->work_queue = create_workqueue(device->name);
	if (!device->work_queue) {
		KGSL_DRV_ERR(device, "create_workqueue(%s) failed\n",
			device->name);
		return -EINVAL;
	}
	return 0;
}

static inline struct kgsl_context *
kgsl_find_context(struct kgsl_device_private *dev_priv, uint32_t id)
{
	struct kgsl_context *ctxt =
		idr_find(&dev_priv->device->context_idr, id);

	/* Make sure that the context belongs to the current instance so
	   that other processes can't guess context IDs and mess things up */

	return  (ctxt && ctxt->dev_priv == dev_priv) ? ctxt : NULL;
}

#endif  /* _KGSL_DEVICE_H */
