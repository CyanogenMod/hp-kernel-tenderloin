/* Copyright (c) 2008-2011, Code Aurora Forum. All rights reserved.
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
#ifndef _GSL_DRIVER_H
#define _GSL_DRIVER_H

#include <linux/types.h>
#include <linux/msm_kgsl.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/regulator/consumer.h>

#include <asm/atomic.h>

#include "kgsl_device.h"
#include "kgsl_sharedmem.h"

#define DRIVER_NAME "kgsl"
#define CLASS_NAME "msm_kgsl"
#define CHIP_REV_251 0x020501

/* Flags to control whether to flush or invalidate a cached memory range */
#define KGSL_CACHE_INV		0x00000000
#define KGSL_CACHE_CLEAN	0x00000001
#define KGSL_CACHE_FLUSH	0x00000002

#define KGSL_CACHE_USER_ADDR	0x00000010
#define KGSL_CACHE_VMALLOC_ADDR	0x00000020

/*cache coherency ops */
#define DRM_KGSL_GEM_CACHE_OP_TO_DEV	0x0001
#define DRM_KGSL_GEM_CACHE_OP_FROM_DEV	0x0002

#define KGSL_NUM_2D_DEVICES 2
#define IDX_2D(X) ((X)-KGSL_DEVICE_2D0)

/* The size of each entry in a page table */
#define KGSL_PAGETABLE_ENTRY_SIZE  4

/* Pagetable Virtual Address base */
#define KGSL_PAGETABLE_BASE	0x66000000

/* Extra accounting entries needed in the pagetable */
#define KGSL_PT_EXTRA_ENTRIES      16

#define KGSL_PAGETABLE_ENTRIES(_sz) (((_sz) >> KGSL_PAGESIZE_SHIFT) + \
				     KGSL_PT_EXTRA_ENTRIES)

#ifdef CONFIG_KGSL_PER_PROCESS_PAGE_TABLE
#define KGSL_PAGETABLE_COUNT (CONFIG_MSM_KGSL_PAGE_TABLE_COUNT)
#else
#define KGSL_PAGETABLE_COUNT 1
#endif

/* Casting using container_of() for structures that kgsl owns. */
#define KGSL_CONTAINER_OF(ptr, type, member) \
		container_of(ptr, type, member)
#define KGSL_YAMATO_DEVICE(device) \
		KGSL_CONTAINER_OF(device, struct kgsl_yamato_device, dev)
#define KGSL_G12_DEVICE(device) \
		KGSL_CONTAINER_OF(device, struct kgsl_g12_device, dev)

struct kgsl_driver {
	struct cdev cdev;
	dev_t dev_num;
	struct class *class;
	struct kgsl_device *devp[KGSL_DEVICE_MAX];
	int num_devs;
	struct platform_device *pdev;

	uint32_t flags_debug;

	/* Global lilst of open processes */
	struct list_head process_list;
	/* Global list of pagetables */
	struct list_head pagetable_list;
	/* Mutex for accessing the pagetable list */
	struct mutex pt_mutex;
	/* Mutex for accessing the process list */
	struct mutex process_mutex;

	struct kgsl_pagetable *global_pt;

	/* Size (in bytes) for each pagetable */
	unsigned int ptsize;

	struct kgsl_ptpool ptpool;

};

extern struct kgsl_driver kgsl_driver;

struct kgsl_mem_entry {
	struct kgsl_memdesc memdesc;
	struct file *file_ptr;
	struct list_head list;
	uint32_t free_timestamp;
	/* back pointer to private structure under whose context this
	* allocation is made */
	struct kgsl_process_private *priv;
};

enum kgsl_status {
	KGSL_SUCCESS = 0,
	KGSL_FAILURE = 1
};

#define KGSL_TRUE 1
#define KGSL_FALSE 0

#ifdef CONFIG_MSM_KGSL_MMU_PAGE_FAULT
#define MMU_CONFIG 2
#else
#define MMU_CONFIG 1
#endif

void kgsl_destroy_mem_entry(struct kgsl_mem_entry *entry);
uint8_t *kgsl_gpuaddr_to_vaddr(const struct kgsl_memdesc *memdesc,
	unsigned int gpuaddr, unsigned int *size);
struct kgsl_mem_entry *kgsl_sharedmem_find_region(
	struct kgsl_process_private *private, unsigned int gpuaddr,
	size_t size);
uint8_t *kgsl_sharedmem_convertaddr(struct kgsl_device *device,
	unsigned int pt_base, unsigned int gpuaddr, unsigned int *size);
int kgsl_idle(struct kgsl_device *device, unsigned int timeout);
int kgsl_setstate(struct kgsl_device *device, uint32_t flags);

static inline void kgsl_regread(struct kgsl_device *device,
				unsigned int offsetwords,
				unsigned int *value)
{
	device->ftbl.device_regread(device, offsetwords, value);
}

static inline void kgsl_regwrite(struct kgsl_device *device,
				 unsigned int offsetwords,
				 unsigned int value)
{
	device->ftbl.device_regwrite(device, offsetwords, value);
}

static inline void kgsl_regread_isr(struct kgsl_device *device,
				    unsigned int offsetwords,
				    unsigned int *value)
{
	device->ftbl.device_regread_isr(device, offsetwords, value);
}

static inline void kgsl_regwrite_isr(struct kgsl_device *device,
				      unsigned int offsetwords,
				      unsigned int value)
{
	device->ftbl.device_regwrite_isr(device, offsetwords, value);
}

int kgsl_check_timestamp(struct kgsl_device *device, unsigned int timestamp);

int kgsl_setup_pt(struct kgsl_pagetable *);

int kgsl_cleanup_pt(struct kgsl_pagetable *);

int kgsl_register_ts_notifier(struct kgsl_device *device,
			      struct notifier_block *nb);

int kgsl_unregister_ts_notifier(struct kgsl_device *device,
				struct notifier_block *nb);

#ifdef CONFIG_MSM_KGSL_DRM
extern int kgsl_drm_init(struct platform_device *dev);
extern void kgsl_drm_exit(void);
extern void kgsl_gpu_mem_flush(int op);
#else
static inline int kgsl_drm_init(struct platform_device *dev)
{
	return 0;
}

static inline void kgsl_drm_exit(void)
{
}
#endif

static inline int kgsl_gpuaddr_in_memdesc(const struct kgsl_memdesc *memdesc,
				unsigned int gpuaddr)
{
	if (gpuaddr >= memdesc->gpuaddr && (gpuaddr + sizeof(unsigned int)) <=
		(memdesc->gpuaddr + memdesc->size)) {
		return 1;
	}
	return 0;
}

static inline struct kgsl_device *kgsl_device_from_dev(struct device *dev)
{
	int i;

	for (i = 0; i < KGSL_DEVICE_MAX; i++) {
		if (kgsl_driver.devp[i] && kgsl_driver.devp[i]->dev == dev)
			return kgsl_driver.devp[i];
	}

	return NULL;
}


#endif /* _GSL_DRIVER_H */
