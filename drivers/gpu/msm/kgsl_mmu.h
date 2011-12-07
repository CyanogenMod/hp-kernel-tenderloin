/* Copyright (c) 2002,2007-2010, Code Aurora Forum. All rights reserved.
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
#ifndef __GSL_MMU_H
#define __GSL_MMU_H
#include <linux/types.h>
#include <linux/msm_kgsl.h>
#include "kgsl_sharedmem.h"

/* Identifier for the global page table */
/* Per process page tables will probably pass in the thread group
   as an identifier */

#define KGSL_MMU_GLOBAL_PT 0

#define GSL_PT_SUPER_PTE 8
#define GSL_PT_PAGE_WV		0x00000001
#define GSL_PT_PAGE_RV		0x00000002
#define GSL_PT_PAGE_DIRTY	0x00000004
/* MMU Flags */
#define KGSL_MMUFLAGS_TLBFLUSH         0x10000000
#define KGSL_MMUFLAGS_PTUPDATE         0x20000000

/* Macros to manage TLB flushing */
#define GSL_TLBFLUSH_FILTER_ENTRY_NUMBITS     (sizeof(unsigned char) * 8)
#define GSL_TLBFLUSH_FILTER_GET(superpte)			     \
	      (*((unsigned char *)				    \
	      (((unsigned int)pagetable->tlbflushfilter.base)    \
	      + (superpte / GSL_TLBFLUSH_FILTER_ENTRY_NUMBITS))))
#define GSL_TLBFLUSH_FILTER_SETDIRTY(superpte)				\
	      (GSL_TLBFLUSH_FILTER_GET((superpte)) |= 1 <<	    \
	      (superpte % GSL_TLBFLUSH_FILTER_ENTRY_NUMBITS))
#define GSL_TLBFLUSH_FILTER_ISDIRTY(superpte)			 \
	      (GSL_TLBFLUSH_FILTER_GET((superpte)) &		  \
	      (1 << (superpte % GSL_TLBFLUSH_FILTER_ENTRY_NUMBITS)))
#define GSL_TLBFLUSH_FILTER_RESET() memset(pagetable->tlbflushfilter.base,\
				      0, pagetable->tlbflushfilter.size)


#ifdef CONFIG_MSM_KGSL_MMU
extern unsigned int kgsl_cache_enable;
#endif

struct kgsl_device;

struct kgsl_ptstats {
	int64_t  maps;
	int64_t  unmaps;
	int64_t  superpteallocs;
	int64_t  superptefrees;
	int64_t  ptswitches;
	int64_t  tlbflushes[KGSL_DEVICE_MAX];
};

struct kgsl_tlbflushfilter {
	unsigned int *base;
	unsigned int size;
};

struct kgsl_pagetable {
	spinlock_t lock;
	unsigned int   refcnt;
	struct kgsl_memdesc  base;
	uint32_t      va_base;
	unsigned int   va_range;
	unsigned int   last_superpte;
	unsigned int   max_entries;
	struct gen_pool *pool;
	struct list_head list;
	unsigned int name;
	/* Maintain filter to manage tlb flushing */
	struct kgsl_tlbflushfilter tlbflushfilter;
	unsigned int tlb_flags;
	struct kobject *kobj;

	struct {
		unsigned int entries;
		unsigned int mapped;
		unsigned int max_mapped;
		unsigned int max_entries;
	} stats;
};

struct kgsl_mmu_reg {

	uint32_t config;
	uint32_t mpu_base;
	uint32_t mpu_end;
	uint32_t va_range;
	uint32_t pt_page;
	uint32_t page_fault;
	uint32_t tran_error;
	uint32_t invalidate;
	uint32_t interrupt_mask;
	uint32_t interrupt_status;
	uint32_t interrupt_clear;
	uint32_t axi_error;
};

struct kgsl_mmu {
	unsigned int     refcnt;
	uint32_t      flags;
	struct kgsl_device     *device;
	unsigned int     config;
	uint32_t        mpu_base;
	int              mpu_range;
	struct kgsl_memdesc    dummyspace;
	struct kgsl_mmu_reg    reg;
	/* current page table object being used by device mmu */
	struct kgsl_pagetable  *defaultpagetable;
	struct kgsl_pagetable  *hwpagetable;
};

struct kgsl_ptpool_chunk {
	size_t size;
	unsigned int count;
	int dynamic;

	void *data;
	unsigned int phys;

	unsigned long *bitmap;
	struct list_head list;
};

struct kgsl_ptpool {
	size_t ptsize;
	struct mutex lock;
	struct list_head list;
	int entries;
	int static_entries;
	int chunks;
};

int kgsl_mmu_init(struct kgsl_device *device);

int kgsl_mmu_start(struct kgsl_device *device);

int kgsl_mmu_stop(struct kgsl_device *device);

int kgsl_mmu_close(struct kgsl_device *device);

struct kgsl_pagetable *kgsl_mmu_getpagetable(unsigned long name);

void kgsl_mmu_putpagetable(struct kgsl_pagetable *pagetable);

int kgsl_mmu_setstate(struct kgsl_device *device,
			struct kgsl_pagetable *pagetable);

static inline unsigned int kgsl_pt_get_flags(struct kgsl_pagetable *pt,
					     enum kgsl_deviceid id)
{
	unsigned int result = 0;
	spin_lock(&pt->lock);
	if (pt->tlb_flags && (1<<id)) {
		result = KGSL_MMUFLAGS_TLBFLUSH;
		pt->tlb_flags &= ~(1<<id);
	}
	spin_unlock(&pt->lock);
	return result;
}

#ifdef CONFIG_MSM_KGSL_MMU
int kgsl_mmu_map(struct kgsl_pagetable *pagetable,
		 unsigned int address,
		 int range,
		 unsigned int protflags,
		 unsigned int *gpuaddr,
		 unsigned int flags);

int kgsl_mmu_unmap(struct kgsl_pagetable *pagetable,
					unsigned int gpuaddr, int range);

unsigned int kgsl_virtaddr_to_physaddr(unsigned int virtaddr);

void kgsl_ptpool_destroy(struct kgsl_ptpool *pool);

int kgsl_ptpool_init(struct kgsl_ptpool *pool, int ptsize, int entries);

static inline int
kgsl_mmu_isenabled(struct kgsl_mmu *mmu)
{
	return ((mmu)->flags & KGSL_FLAGS_STARTED) ? 1 : 0;
}

#else
static inline int kgsl_mmu_map(struct kgsl_pagetable *pagetable,
		 unsigned int address,
		 int range,
		 unsigned int protflags,
		 unsigned int *gpuaddr,
		 unsigned int flags)
{
	*gpuaddr = address;
	return 0;
}

static inline int kgsl_mmu_unmap(struct kgsl_pagetable *pagetable,
					unsigned int gpuaddr, int range)
{ return 0; }

static inline int kgsl_mmu_isenabled(struct kgsl_mmu *mmu) { return 0; }

static inline int kgsl_ptpool_init(struct kgsl_ptpool *pool, int ptsize,
				    int entries)
{
	return 0;
}

static inline void kgsl_ptpool_free(struct kgsl_ptpool *pool) { }

#endif

int kgsl_mmu_map_global(struct kgsl_pagetable *pagetable,
			struct kgsl_memdesc *memdesc, unsigned int protflags,
			unsigned int flags);

int kgsl_mmu_querystats(struct kgsl_pagetable *pagetable,
			struct kgsl_ptstats *stats);

void kgsl_mh_intrcallback(struct kgsl_device *device);

#endif /* __GSL_MMU_H */
