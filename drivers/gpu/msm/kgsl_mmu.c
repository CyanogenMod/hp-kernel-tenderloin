/* Copyright (c) 2002,2007-2011, Code Aurora Forum. All rights reserved.
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
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/genalloc.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/bitmap.h>
#ifdef CONFIG_MSM_KGSL_MMU
#include <asm/pgalloc.h>
#include <asm/pgtable.h>
#endif
#include "kgsl_mmu.h"
#include "kgsl_drawctxt.h"
#include "kgsl.h"
#include "kgsl_log.h"
#include "yamato_reg.h"
#include "g12_reg.h"
#include "kgsl_device.h"
#include "kgsl_g12.h"
#include "kgsl_yamato.h"

struct kgsl_pte_debug {
	unsigned int read:1;
	unsigned int write:1;
	unsigned int dirty:1;
	unsigned int reserved:9;
	unsigned int phyaddr:20;
};

#define GSL_PT_PAGE_BITS_MASK	0x00000007
#define GSL_PT_PAGE_ADDR_MASK	(~(KGSL_PAGESIZE - 1))

#define GSL_MMU_INT_MASK \
	(MH_INTERRUPT_MASK__AXI_READ_ERROR | \
	 MH_INTERRUPT_MASK__AXI_WRITE_ERROR)

static const struct kgsl_mmu_reg mmu_reg[KGSL_DEVICE_MAX] = {
	{
		.config = REG_MH_MMU_CONFIG,
		.mpu_base = REG_MH_MMU_MPU_BASE,
		.mpu_end = REG_MH_MMU_MPU_END,
		.va_range = REG_MH_MMU_VA_RANGE,
		.pt_page = REG_MH_MMU_PT_BASE,
		.page_fault = REG_MH_MMU_PAGE_FAULT,
		.tran_error = REG_MH_MMU_TRAN_ERROR,
		.invalidate = REG_MH_MMU_INVALIDATE,
		.interrupt_mask = REG_MH_INTERRUPT_MASK,
		.interrupt_status = REG_MH_INTERRUPT_STATUS,
		.interrupt_clear = REG_MH_INTERRUPT_CLEAR
	},
	{
		.config = ADDR_MH_MMU_CONFIG,
		.mpu_base = ADDR_MH_MMU_MPU_BASE,
		.mpu_end = ADDR_MH_MMU_MPU_END,
		.va_range = ADDR_MH_MMU_VA_RANGE,
		.pt_page = ADDR_MH_MMU_PT_BASE,
		.page_fault = ADDR_MH_MMU_PAGE_FAULT,
		.tran_error = ADDR_MH_MMU_TRAN_ERROR,
		.invalidate = ADDR_MH_MMU_INVALIDATE,
		.interrupt_mask = ADDR_MH_INTERRUPT_MASK,
		.interrupt_status = ADDR_MH_INTERRUPT_STATUS,
		.interrupt_clear = ADDR_MH_INTERRUPT_CLEAR
	},
	{
		.config = ADDR_MH_MMU_CONFIG,
		.mpu_base = ADDR_MH_MMU_MPU_BASE,
		.mpu_end = ADDR_MH_MMU_MPU_END,
		.va_range = ADDR_MH_MMU_VA_RANGE,
		.pt_page = ADDR_MH_MMU_PT_BASE,
		.page_fault = ADDR_MH_MMU_PAGE_FAULT,
		.tran_error = ADDR_MH_MMU_TRAN_ERROR,
		.invalidate = ADDR_MH_MMU_INVALIDATE,
		.interrupt_mask = ADDR_MH_INTERRUPT_MASK,
		.interrupt_status = ADDR_MH_INTERRUPT_STATUS,
		.interrupt_clear = ADDR_MH_INTERRUPT_CLEAR
	}
};

static ssize_t
sysfs_show_ptpool_entries(struct kobject *kobj,
			  struct kobj_attribute *attr,
			  char *buf)
{
	return sprintf(buf, "%d\n", kgsl_driver.ptpool.entries);
}

static ssize_t
sysfs_show_ptpool_min(struct kobject *kobj,
			 struct kobj_attribute *attr,
			 char *buf)
{
	return sprintf(buf, "%d\n", kgsl_driver.ptpool.static_entries);
}

static ssize_t
sysfs_show_ptpool_chunks(struct kobject *kobj,
			 struct kobj_attribute *attr,
			 char *buf)
{
	return sprintf(buf, "%d\n", kgsl_driver.ptpool.chunks);
}

static ssize_t
sysfs_show_ptpool_ptsize(struct kobject *kobj,
			 struct kobj_attribute *attr,
			 char *buf)
{
	return sprintf(buf, "%d\n", kgsl_driver.ptpool.ptsize);
}

static struct kobj_attribute attr_ptpool_entries = {
	.attr = { .name = "ptpool_entries", .mode = 0444 },
	.show = sysfs_show_ptpool_entries,
	.store = NULL,
};

static struct kobj_attribute attr_ptpool_min = {
	.attr = { .name = "ptpool_min", .mode = 0444 },
	.show = sysfs_show_ptpool_min,
	.store = NULL,
};

static struct kobj_attribute attr_ptpool_chunks = {
	.attr = { .name = "ptpool_chunks", .mode = 0444 },
	.show = sysfs_show_ptpool_chunks,
	.store = NULL,
};

static struct kobj_attribute attr_ptpool_ptsize = {
	.attr = { .name = "ptpool_ptsize", .mode = 0444 },
	.show = sysfs_show_ptpool_ptsize,
	.store = NULL,
};

static struct attribute *ptpool_attrs[] = {
	&attr_ptpool_entries.attr,
	&attr_ptpool_min.attr,
	&attr_ptpool_chunks.attr,
	&attr_ptpool_ptsize.attr,
	NULL,
};

static struct attribute_group ptpool_attr_group = {
	.attrs = ptpool_attrs,
};

static int
_kgsl_ptpool_add_entries(struct kgsl_ptpool *pool, int count, int dynamic)
{
	struct kgsl_ptpool_chunk *chunk;
	size_t size = ALIGN(count * pool->ptsize, PAGE_SIZE);

	BUG_ON(count == 0);

	if (get_order(size) >= MAX_ORDER) {
		KGSL_DRV_ERR("ptpool allocation is too big: %d\n", size);
		return -EINVAL;
	}

	chunk = kzalloc(sizeof(*chunk), GFP_KERNEL);
	if (chunk == NULL) {
		KGSL_DRV_ERR("kzalloc(%d) failed\n", sizeof(*chunk));
		return -ENOMEM;
	}

	chunk->size = size;
	chunk->count = count;
	chunk->dynamic = dynamic;

	chunk->data = dma_alloc_coherent(NULL, size,
					 &chunk->phys, GFP_KERNEL);

	if (chunk->data == NULL) {
		KGSL_DRV_ERR("dma_alloc_coherent(%d) failed\n", size);
		goto err;
	}

	chunk->bitmap = kzalloc(BITS_TO_LONGS(count) * 4, GFP_KERNEL);

	if (chunk->bitmap == NULL) {
		KGSL_DRV_ERR("kzalloc(%d) failed\n",
			BITS_TO_LONGS(count) * 4);
		goto err_dma;
	}

	list_add_tail(&chunk->list, &pool->list);

	pool->chunks++;
	pool->entries += count;

	if (!dynamic)
		pool->static_entries += count;

	return 0;

err_dma:
	dma_free_coherent(NULL, chunk->size, chunk->data, chunk->phys);
err:
	kfree(chunk);
	return -ENOMEM;
}

static void *
_kgsl_ptpool_get_entry(struct kgsl_ptpool *pool, unsigned int *physaddr)
{
	struct kgsl_ptpool_chunk *chunk;

	list_for_each_entry(chunk, &pool->list, list) {
		int bit = find_first_zero_bit(chunk->bitmap, chunk->count);

		if (bit >= chunk->count)
			continue;

		set_bit(bit, chunk->bitmap);
		*physaddr = chunk->phys + (bit * pool->ptsize);

		return chunk->data + (bit * pool->ptsize);
	}

	return NULL;
}

/**
 * kgsl_ptpool_add
 * @pool:  A pointer to a ptpool structure
 * @entries: Number of entries to add
 *
 * Add static entries to the pagetable pool.
 */

int
kgsl_ptpool_add(struct kgsl_ptpool *pool, int count)
{
	int ret = 0;
	BUG_ON(count == 0);

	mutex_lock(&pool->lock);

	/* Only 4MB can be allocated in one chunk, so larger allocations
	   need to be split into multiple sections */

	while (count) {
		int entries = ((count * pool->ptsize) > SZ_4M) ?
			SZ_4M / pool->ptsize : count;

		/* Add the entries as static, i.e. they don't ever stand
		   a chance of being removed */

		ret =  _kgsl_ptpool_add_entries(pool, entries, 0);
		if (ret)
			break;

		count -= entries;
	}

	mutex_unlock(&pool->lock);
	return ret;
}

/**
 * kgsl_ptpool_alloc
 * @pool:  A pointer to a ptpool structure
 * @addr: A pointer to store the physical address of the chunk
 *
 * Allocate a pagetable from the pool.  Returns the virtual address
 * of the pagetable, the physical address is returned in physaddr
 */

void *kgsl_ptpool_alloc(struct kgsl_ptpool *pool, unsigned int *physaddr)
{
	void *addr = NULL;
	int ret;

	mutex_lock(&pool->lock);
	addr = _kgsl_ptpool_get_entry(pool, physaddr);
	if (addr)
		goto done;

	/* Add a chunk for 1 more pagetable and mark it as dynamic */
	ret = _kgsl_ptpool_add_entries(pool, 1, 1);

	if (ret)
		goto done;

	addr = _kgsl_ptpool_get_entry(pool, physaddr);
done:
	mutex_unlock(&pool->lock);
	return addr;
}

static inline void _kgsl_ptpool_rm_chunk(struct kgsl_ptpool_chunk *chunk)
{
	list_del(&chunk->list);

	if (chunk->data)
		dma_free_coherent(NULL, chunk->size, chunk->data,
			chunk->phys);
	kfree(chunk->bitmap);
	kfree(chunk);
}

/**
 * kgsl_ptpool_free
 * @pool:  A pointer to a ptpool structure
 * @addr: A pointer to the virtual address to free
 *
 * Free a pagetable allocated from the pool
 */

void kgsl_ptpool_free(struct kgsl_ptpool *pool, void *addr)
{
	struct kgsl_ptpool_chunk *chunk, *tmp;

	if (pool == NULL || addr == NULL)
		return;

	mutex_lock(&pool->lock);
	list_for_each_entry_safe(chunk, tmp, &pool->list, list)  {
		if (addr >=  chunk->data &&
		    addr < chunk->data + chunk->size) {
			int bit = ((unsigned long) (addr - chunk->data)) /
				pool->ptsize;

			clear_bit(bit, chunk->bitmap);
			memset(addr, 0, pool->ptsize);

			if (chunk->dynamic &&
				bitmap_empty(chunk->bitmap, chunk->count))
				_kgsl_ptpool_rm_chunk(chunk);

			break;
		}
	}

	mutex_unlock(&pool->lock);
}

void kgsl_ptpool_destroy(struct kgsl_ptpool *pool)
{
	struct kgsl_ptpool_chunk *chunk, *tmp;

	if (pool == NULL)
		return;

	mutex_lock(&pool->lock);
	list_for_each_entry_safe(chunk, tmp, &pool->list, list)
		_kgsl_ptpool_rm_chunk(chunk);
	mutex_unlock(&pool->lock);

	memset(pool, 0, sizeof(*pool));
}

/**
 * kgsl_ptpool_init
 * @pool:  A pointer to a ptpool structure to initialize
 * @ptsize: The size of each pagetable entry
 * @entries:  The number of inital entries to add to the pool
 *
 * Initalize a pool and allocate an initial chunk of entries.
 */

int kgsl_ptpool_init(struct kgsl_ptpool *pool, int ptsize, int entries)
{
	int ret = 0;
	BUG_ON(ptsize == 0);

	pool->ptsize = ptsize;
	mutex_init(&pool->lock);
	INIT_LIST_HEAD(&pool->list);

	if (entries) {
		ret = kgsl_ptpool_add(pool, entries);
		if (ret)
			return ret;
	}

	/* PALM: sysfs not supported yet
	return sysfs_create_group(kgsl_driver.ptkobj, &ptpool_attr_group);
	*/
	return 0;
}

/* pt_mutex needs to be held in this function */

static struct kgsl_pagetable *
kgsl_get_pagetable(unsigned long name)
{
	struct kgsl_pagetable *pt;

	list_for_each_entry(pt,	&kgsl_driver.pagetable_list, list) {
		if (pt->name == name)
			return pt;
	}
};

static inline uint32_t
kgsl_pt_entry_get(struct kgsl_pagetable *pt, uint32_t va)
{
	return (va - pt->va_base) >> KGSL_PAGESIZE_SHIFT;
}

static inline void
kgsl_pt_map_set(struct kgsl_pagetable *pt, uint32_t pte, uint32_t val)
{
	uint32_t *baseptr = (uint32_t *)pt->base.hostptr;
	writel(val, &baseptr[pte]);
}

static inline uint32_t
kgsl_pt_map_getaddr(struct kgsl_pagetable *pt, uint32_t pte)
{
	uint32_t *baseptr = (uint32_t *)pt->base.hostptr;
	return readl(&baseptr[pte]) & GSL_PT_PAGE_ADDR_MASK;
}

void kgsl_mh_intrcallback(struct kgsl_device *device)
{
	unsigned int status = 0;
	unsigned int reg;

	KGSL_MEM_VDBG("enter (device=%p)\n", device);

	kgsl_regread_isr(device, mmu_reg[device->id].interrupt_status, &status);

	if (status & MH_INTERRUPT_MASK__AXI_READ_ERROR) {
		KGSL_MEM_FATAL("axi read error interrupt\n");
	} else if (status & MH_INTERRUPT_MASK__AXI_WRITE_ERROR) {
		KGSL_MEM_FATAL("axi write error interrupt\n");
	} else if (status & MH_INTERRUPT_MASK__MMU_PAGE_FAULT) {
		kgsl_regread_isr(device, mmu_reg[device->id].page_fault, &reg);
		KGSL_MEM_FATAL("mmu page fault interrupt: %08x\n", reg);
	} else {
		KGSL_MEM_DBG("bad bits in REG_MH_INTERRUPT_STATUS %08x\n",
			     status);
	}

	kgsl_regwrite_isr(device, mmu_reg[device->id].interrupt_clear, status);

	/*TODO: figure out how to handle errror interupts.
	* specifically, page faults should probably nuke the client that
	* caused them, but we don't have enough info to figure that out yet.
	*/

	KGSL_MEM_VDBG("return\n");
}

int
kgsl_get_ptname_from_ptbase(unsigned int pt_base)
{
	struct kgsl_pagetable *pt;
	int ptid = -1;

	mutex_lock(&kgsl_driver.pt_mutex);

	list_for_each_entry(pt, &kgsl_driver.pagetable_list, list) {
		if (pt_base == pt->base.gpuaddr) {
			ptid = (int) pt->name;
			break;
		}
	}
	mutex_unlock(&kgsl_driver.pt_mutex);

	return ptid;
}

static struct kgsl_pagetable *kgsl_mmu_createpagetableobject(
				struct kgsl_mmu *mmu,
				unsigned int name)
{
	int status = 0;
	struct kgsl_pagetable *pagetable = NULL;

	KGSL_MEM_VDBG("enter (mmu=%p)\n", mmu);

	pagetable = kzalloc(sizeof(struct kgsl_pagetable), GFP_KERNEL);
	if (pagetable == NULL) {
		KGSL_MEM_ERR("Unable to allocate pagetable object.\n");
		return NULL;
	}

	pagetable->magic_number = KGSL_PAGETABLE_INIT_NUMBER;
	pagetable->refcnt = 1;

	spin_lock_init(&pagetable->lock);
	pagetable->tlb_flags = 0;
	pagetable->name = name;
	pagetable->va_base = KGSL_PAGETABLE_BASE;
	pagetable->va_range = CONFIG_MSM_KGSL_PAGE_TABLE_SIZE;
	pagetable->last_superpte = 0;
	pagetable->max_entries = KGSL_PAGETABLE_ENTRIES(pagetable->va_range);

	pagetable->tlbflushfilter.size = (pagetable->va_range /
				(PAGE_SIZE * GSL_PT_SUPER_PTE * 8)) + 1;
	pagetable->tlbflushfilter.base = (unsigned int *)
			kzalloc(pagetable->tlbflushfilter.size, GFP_KERNEL);
	if (!pagetable->tlbflushfilter.base) {
		KGSL_MEM_ERR("Failed to create tlbflushfilter\n");
		goto err_alloc;
	}
	GSL_TLBFLUSH_FILTER_RESET();

	pagetable->pool = gen_pool_create(KGSL_PAGESIZE_SHIFT, -1);
	if (pagetable->pool == NULL) {
		KGSL_MEM_ERR("Unable to allocate virtualaddr pool.\n");
		goto err_flushfilter;
	}

	if (gen_pool_add(pagetable->pool, pagetable->va_base,
				pagetable->va_range, -1)) {
		KGSL_MEM_ERR("gen_pool_create failed for pagetable %p\n",
				pagetable);
		goto err_pool;
	}

	pagetable->base.hostptr = kgsl_ptpool_alloc(&kgsl_driver.ptpool,
		&pagetable->base.physaddr);

	if (pagetable->base.hostptr == NULL)
		goto err_pool;

	pagetable->base.gpuaddr = pagetable->base.physaddr;

	status = kgsl_setup_pt(pagetable);
	if (status)
		goto err_free_sharedmem;

	list_add(&pagetable->list, &kgsl_driver.pagetable_list);

	KGSL_MEM_VDBG("return %p\n", pagetable);
	return pagetable;

err_free_sharedmem:
	kgsl_ptpool_free(&kgsl_driver.ptpool, &pagetable->base.hostptr);
err_pool:
	gen_pool_destroy(pagetable->pool);
err_flushfilter:
	kfree(pagetable->tlbflushfilter.base);
err_alloc:
	kfree(pagetable);

	return NULL;
}

static void kgsl_mmu_destroypagetable(struct kgsl_pagetable *pagetable)
{
	KGSL_MEM_VDBG("enter (pagetable=%p)\n", pagetable);

	list_del(&pagetable->list);

	kgsl_cleanup_pt(pagetable);

	kgsl_ptpool_free(&kgsl_driver.ptpool, pagetable->base.hostptr);

	if (pagetable->pool) {
		gen_pool_destroy(pagetable->pool);
		pagetable->pool = NULL;
	}

	if (pagetable->tlbflushfilter.base) {
		pagetable->tlbflushfilter.size = 0;
		kfree(pagetable->tlbflushfilter.base);
		pagetable->tlbflushfilter.base = NULL;
	}

	pagetable->magic_number = KGSL_PAGETABLE_POISON_NUMBER;
	kfree(pagetable);
}

struct kgsl_pagetable *kgsl_mmu_getpagetable(struct kgsl_mmu *mmu,
					     unsigned long name)
{
	struct kgsl_pagetable *pt;

	if (mmu == NULL)
		return NULL;

	mutex_lock(&kgsl_driver.pt_mutex);

	list_for_each_entry(pt,	&kgsl_driver.pagetable_list, list) {
		if (pt->name == name) {
			spin_lock(&pt->lock);
			pt->refcnt++;
			spin_unlock(&pt->lock);
			mutex_unlock(&kgsl_driver.pt_mutex);
			return pt;
		}
	}

	pt = kgsl_mmu_createpagetableobject(mmu, name);
	mutex_unlock(&kgsl_driver.pt_mutex);

	return pt;
}

void kgsl_mmu_putpagetable(struct kgsl_pagetable *pagetable)
{
	bool dead;
	if (pagetable == NULL)
		return;

	mutex_lock(&kgsl_driver.pt_mutex);

	spin_lock(&pagetable->lock);
	dead = (--pagetable->refcnt) == 0;
	spin_unlock(&pagetable->lock);

	if (dead)
		kgsl_mmu_destroypagetable(pagetable);

	mutex_unlock(&kgsl_driver.pt_mutex);
}

int kgsl_mmu_setstate(struct kgsl_device *device,
				struct kgsl_pagetable *pagetable)
{
	int status = 0;
	struct kgsl_mmu *mmu = &device->mmu;

	KGSL_MEM_VDBG("enter (device=%p, pagetable=%p)\n", device, pagetable);

	if (mmu->flags & KGSL_FLAGS_STARTED) {
		/* page table not current, then setup mmu to use new
		 *  specified page table
		 */
//		printk(KERN_DEBUG"from %p to %p magic %x\n", mmu->hwpagetable, pagetable,
//			pagetable->magic_number);

		if (mmu->hwpagetable != pagetable) {
//			KGSL_MEM_ERR("In kgsl_mmu_setstate() - pagetable = %x pagetable->magic_number = %x\n"
//					, pagetable, pagetable->magic_number);
			mmu->hwpagetable = pagetable;
			spin_lock(&mmu->hwpagetable->lock);
			mmu->hwpagetable->tlb_flags &= ~(1<<device->id);
			spin_unlock(&mmu->hwpagetable->lock);

			/* call device specific set page table */
			status = kgsl_setstate(mmu->device,
				KGSL_MMUFLAGS_TLBFLUSH |
				KGSL_MMUFLAGS_PTUPDATE);

		}
	}
//	else {
//		printk(KERN_DEBUG"kgsl flags not started (device=%p, pagetable=%p)\n", device, pagetable);
//	}

	KGSL_MEM_VDBG("return %d\n", status);

	return status;
}

int kgsl_mmu_init(struct kgsl_device *device)
{
	/*
	 * intialize device mmu
	 *
	 * call this with the global lock held
	 */
	int status;
	struct kgsl_mmu *mmu = &device->mmu;

	KGSL_MEM_VDBG("enter (device=%p)\n", device);

	mmu->device = device;

#ifndef CONFIG_MSM_KGSL_MMU
	mmu->config = 0x00000000;
#endif

	/* MMU not enabled */
	if ((mmu->config & 0x1) == 0) {
		KGSL_MEM_VDBG("return %d\n", 0);
		return 0;
	}

	/* make sure aligned to pagesize */
	BUG_ON(mmu->mpu_base & (KGSL_PAGESIZE - 1));
	BUG_ON((mmu->mpu_base + mmu->mpu_range) & (KGSL_PAGESIZE - 1));

	/* sub-client MMU lookups require address translation */
	if ((mmu->config & ~0x1) > 0) {
		/*make sure virtual address range is a multiple of 64Kb */
		BUG_ON(CONFIG_MSM_KGSL_PAGE_TABLE_SIZE & ((1 << 16) - 1));

		/* allocate memory used for completing r/w operations that
		 * cannot be mapped by the MMU
		 */
		status = kgsl_sharedmem_alloc_coherent(&mmu->dummyspace, 64);
		if (status != 0) {
			KGSL_MEM_ERR
			    ("Unable to allocate dummy space memory.\n");
			goto error;
		}

		kgsl_sharedmem_set(&mmu->dummyspace, 0, 0,
				   mmu->dummyspace.size);

	}

	KGSL_MEM_VDBG("return %d\n", 0);

	return 0;

error:
	return status;
}

int kgsl_mmu_start(struct kgsl_device *device)
{
	/*
	 * intialize device mmu
	 *
	 * call this with the global lock held
	 */
	int status;
	struct kgsl_mmu *mmu = &device->mmu;

	KGSL_MEM_VDBG("enter (device=%p)\n", device);

	if (mmu->flags & KGSL_FLAGS_STARTED) {
		KGSL_MEM_INFO("MMU already started.\n");
		return 0;
	}

	/* MMU not enabled */
	if ((mmu->config & 0x1) == 0) {
		KGSL_MEM_VDBG("return %d\n", 0);
		return 0;
	}

	mmu->flags |= KGSL_FLAGS_STARTED;

	/* setup MMU and sub-client behavior */
	kgsl_regwrite(device, mmu_reg[device->id].config, mmu->config);

	/* enable axi interrupts */
	KGSL_MEM_DBG("enabling mmu interrupts mask=0x%08lx\n",
		     GSL_MMU_INT_MASK);
	kgsl_regwrite(device, mmu_reg[device->id].interrupt_mask,
				GSL_MMU_INT_MASK);

	/* idle device */
	kgsl_idle(device,  KGSL_TIMEOUT_DEFAULT);

	/* define physical memory range accessible by the core */
	kgsl_regwrite(device, mmu_reg[device->id].mpu_base, mmu->mpu_base);
	kgsl_regwrite(device, mmu_reg[device->id].mpu_end,
			mmu->mpu_base + mmu->mpu_range);

	/* enable axi interrupts */
	KGSL_MEM_DBG("enabling mmu interrupts mask=0x%08lx\n",
		     GSL_MMU_INT_MASK | MH_INTERRUPT_MASK__MMU_PAGE_FAULT);
	kgsl_regwrite(device, mmu_reg[device->id].interrupt_mask,
			GSL_MMU_INT_MASK | MH_INTERRUPT_MASK__MMU_PAGE_FAULT);

	/* sub-client MMU lookups require address translation */
	if ((mmu->config & ~0x1) > 0) {

		kgsl_sharedmem_set(&mmu->dummyspace, 0, 0,
				   mmu->dummyspace.size);

		/* TRAN_ERROR needs a 32 byte (32 byte aligned) chunk of memory
		 * to complete transactions in case of an MMU fault. Note that
		 * we'll leave the bottom 32 bytes of the dummyspace for other
		 * purposes (e.g. use it when dummy read cycles are needed
		 * for other blocks */
		kgsl_regwrite(device, mmu_reg[device->id].tran_error,
						mmu->dummyspace.physaddr + 32);

		BUG_ON(mmu->defaultpagetable == NULL);
		mmu->hwpagetable = mmu->defaultpagetable;

		kgsl_regwrite(device, mmu_reg[device->id].pt_page,
			      mmu->hwpagetable->base.gpuaddr);
		kgsl_regwrite(device, mmu_reg[device->id].va_range,
			      (mmu->hwpagetable->va_base |
			      (mmu->hwpagetable->va_range >> 16)));
		status = kgsl_setstate(device, KGSL_MMUFLAGS_TLBFLUSH);
		if (status) {
			KGSL_MEM_ERR("Failed to setstate TLBFLUSH\n");
			goto error;
		}
	}

	KGSL_MEM_VDBG("return %d\n", 0);

	return 0;
error:
	/* disable MMU */
	kgsl_regwrite(device, mmu_reg[device->id].interrupt_mask, 0);
	kgsl_regwrite(device, mmu_reg[device->id].config, 0x00000000);
	return status;
}



#ifdef CONFIG_MSM_KGSL_MMU

unsigned int kgsl_virtaddr_to_physaddr(unsigned int virtaddr)
{
	unsigned int physaddr = 0;
	pgd_t *pgd_ptr = NULL;
	pmd_t *pmd_ptr = NULL;
	pte_t *pte_ptr = NULL, pte;

	pgd_ptr = pgd_offset(current->mm, virtaddr);
	if (pgd_none(*pgd) || pgd_bad(*pgd)) {
		KGSL_MEM_ERR
		    ("Invalid pgd entry found while trying to convert virtual "
		     "address to physical\n");
		return 0;
	}

	pmd_ptr = pmd_offset(pgd_ptr, virtaddr);
	if (pmd_none(*pmd_ptr) || pmd_bad(*pmd_ptr)) {
		KGSL_MEM_ERR
		    ("Invalid pmd entry found while trying to convert virtual "
		     "address to physical\n");
		return 0;
	}

	pte_ptr = pte_offset_map(pmd_ptr, virtaddr);
	if (!pte_ptr) {
		KGSL_MEM_ERR
		    ("Unable to map pte entry while trying to convert virtual "
		     "address to physical\n");
		return 0;
	}
	pte = *pte_ptr;
	physaddr = pte_pfn(pte);
	pte_unmap(pte_ptr);
	physaddr <<= PAGE_SHIFT;
	return physaddr;
}

int
kgsl_mmu_map(struct kgsl_pagetable *pagetable,
				unsigned int address,
				int range,
				unsigned int protflags,
				unsigned int *gpuaddr,
				unsigned int flags)
{
	int numpages;
	unsigned int pte, ptefirst, ptelast, physaddr;
	int flushtlb, alloc_size;
	unsigned int align = flags & KGSL_MEMFLAGS_ALIGN_MASK;

	KGSL_MEM_VDBG("enter (pt=%p, physaddr=%08x, range=%08d, gpuaddr=%p)\n",
		      pagetable, address, range, gpuaddr);

	BUG_ON(protflags & ~(GSL_PT_PAGE_RV | GSL_PT_PAGE_WV));
	BUG_ON(protflags == 0);
	BUG_ON(range <= 0);

	/* Only support 4K and 8K alignment for now */
	if (align != KGSL_MEMFLAGS_ALIGN8K && align != KGSL_MEMFLAGS_ALIGN4K) {
		KGSL_MEM_ERR("Cannot map memory according to "
			     "requested flags: %08x\n", flags);
		return -EINVAL;
	}

	/* Make sure address being mapped is at 4K boundary */
	if (!IS_ALIGNED(address, KGSL_PAGESIZE) || range & ~KGSL_PAGEMASK) {
		KGSL_MEM_ERR("Cannot map address not aligned "
			     "at page boundary: address: %08x, range: %08x\n",
			     address, range);
		return -EINVAL;
	}
	alloc_size = range;
	if (align == KGSL_MEMFLAGS_ALIGN8K)
		alloc_size += KGSL_PAGESIZE;

	*gpuaddr = gen_pool_alloc(pagetable->pool, alloc_size);
	if (*gpuaddr == 0) {
		KGSL_MEM_ERR("gen_pool_alloc failed: %d\n", alloc_size);
		return -ENOMEM;
	}

	if (align == KGSL_MEMFLAGS_ALIGN8K) {
		if (*gpuaddr & ((1 << 13) - 1)) {
			/* Not 8k aligned, align it */
			gen_pool_free(pagetable->pool, *gpuaddr, KGSL_PAGESIZE);
			*gpuaddr = *gpuaddr + KGSL_PAGESIZE;
		} else
			gen_pool_free(pagetable->pool, *gpuaddr + range,
				      KGSL_PAGESIZE);
	}

	numpages = (range >> KGSL_PAGESIZE_SHIFT);

	ptefirst = kgsl_pt_entry_get(pagetable, *gpuaddr);
	ptelast = ptefirst + numpages;

	pte = ptefirst;
	flushtlb = 0;

	/* tlb needs to be flushed when the first and last pte are not at
	* superpte boundaries */
	if ((ptefirst & (GSL_PT_SUPER_PTE - 1)) != 0 ||
		((ptelast + 1) & (GSL_PT_SUPER_PTE-1)) != 0)
		flushtlb = 1;

	spin_lock(&pagetable->lock);
	for (pte = ptefirst; pte < ptelast; pte++) {
#ifdef VERBOSE_DEBUG
		/* check if PTE exists */
		uint32_t val = kgsl_pt_map_getaddr(pagetable, pte);
		BUG_ON(val != 0 && val != GSL_PT_PAGE_DIRTY);
#endif
		if ((pte & (GSL_PT_SUPER_PTE-1)) == 0)
			if (GSL_TLBFLUSH_FILTER_ISDIRTY(pte / GSL_PT_SUPER_PTE))
				flushtlb = 1;
		/* mark pte as in use */
		if (flags & KGSL_MEMFLAGS_CONPHYS)
			physaddr = address;
		else if (flags & KGSL_MEMFLAGS_VMALLOC_MEM) {
			physaddr = vmalloc_to_pfn((void *)address);
			physaddr <<= PAGE_SHIFT;
		} else if (flags & KGSL_MEMFLAGS_HOSTADDR)
			physaddr = kgsl_virtaddr_to_physaddr(address);
		else
			physaddr = 0;

		if (physaddr) {
			kgsl_pt_map_set(pagetable, pte, physaddr | protflags);
		} else {
			KGSL_MEM_ERR
			("Unable to find physaddr for address: %x\n",
			     address);
			spin_unlock(&pagetable->lock);
			kgsl_mmu_unmap(pagetable, *gpuaddr, range);
			return -EFAULT;
		}
		address += KGSL_PAGESIZE;
	}

	KGSL_MEM_INFO("pt %p p %08x g %08x pte f %d l %d n %d f %d\n",
		      pagetable, address, *gpuaddr, ptefirst, ptelast,
		      numpages, flushtlb);

	mb();
	dsb();
	outer_sync();

	/* Invalidate tlb only if current page table used by GPU is the
	 * pagetable that we used to allocate */
	if (flushtlb) {
		/*set all devices as needing flushing*/
		pagetable->tlb_flags = UINT_MAX;
		GSL_TLBFLUSH_FILTER_RESET();
	}
	spin_unlock(&pagetable->lock);


	KGSL_MEM_VDBG("return %d\n", 0);

	return 0;
}

int
kgsl_mmu_unmap(struct kgsl_pagetable *pagetable, unsigned int gpuaddr,
		int range)
{
	unsigned int numpages;
	unsigned int pte, ptefirst, ptelast, superpte;

	KGSL_MEM_VDBG("enter (pt=%p, gpuaddr=0x%08x, range=%d)\n",
			pagetable, gpuaddr, range);

	BUG_ON(range <= 0);

	numpages = (range >> KGSL_PAGESIZE_SHIFT);
	if (range & (KGSL_PAGESIZE - 1))
		numpages++;

	ptefirst = kgsl_pt_entry_get(pagetable, gpuaddr);
	ptelast = ptefirst + numpages;

	KGSL_MEM_INFO("pt %p gpu %08x pte first %d last %d numpages %d\n",
		      pagetable, gpuaddr, ptefirst, ptelast, numpages);

	spin_lock(&pagetable->lock);
	superpte = ptefirst - (ptefirst & (GSL_PT_SUPER_PTE-1));
	GSL_TLBFLUSH_FILTER_SETDIRTY(superpte / GSL_PT_SUPER_PTE);
	for (pte = ptefirst; pte < ptelast; pte++) {
#ifdef VERBOSE_DEBUG
		/* check if PTE exists */
		BUG_ON(!kgsl_pt_map_getaddr(pagetable, pte));
#endif
		kgsl_pt_map_set(pagetable, pte, GSL_PT_PAGE_DIRTY);
		superpte = pte - (pte & (GSL_PT_SUPER_PTE - 1));
		if (pte == superpte)
			GSL_TLBFLUSH_FILTER_SETDIRTY(superpte /
				GSL_PT_SUPER_PTE);
	}

	mb();
	dsb();
	outer_sync();

	spin_unlock(&pagetable->lock);

	gen_pool_free(pagetable->pool, gpuaddr, range);

	KGSL_MEM_VDBG("return %d\n", 0);

	return 0;
}
#endif /*CONFIG_MSM_KGSL_MMU*/

int kgsl_mmu_map_global(struct kgsl_pagetable *pagetable,
			struct kgsl_memdesc *memdesc, unsigned int protflags,
			unsigned int flags)
{
	int result = -EINVAL;
	unsigned int gpuaddr = 0;

	if (memdesc == NULL)
		goto error;

	result = kgsl_mmu_map(pagetable, memdesc->physaddr, memdesc->size,
				protflags, &gpuaddr, flags);
	if (result)
		goto error;

	/*global mappings must have the same gpu address in all pagetables*/
	if (memdesc->gpuaddr == 0)
		memdesc->gpuaddr = gpuaddr;

	else if (memdesc->gpuaddr != gpuaddr) {
		KGSL_MEM_ERR("pt %p addr mismatch phys 0x%08x gpu 0x%0x 0x%08x",
				pagetable, memdesc->physaddr,
				memdesc->gpuaddr, gpuaddr);
		goto error_unmap;
	}
	return result;
error_unmap:
	kgsl_mmu_unmap(pagetable, gpuaddr, memdesc->size);
error:
	return result;
}

int kgsl_mmu_stop(struct kgsl_device *device)
{
	/*
	 *  stop device mmu
	 *
	 *  call this with the global lock held
	 */
	struct kgsl_mmu *mmu = &device->mmu;

	KGSL_MEM_VDBG("enter (device=%p)\n", device);

	if (mmu->flags & KGSL_FLAGS_STARTED) {
		/* disable mh interrupts */
		KGSL_MEM_DBG("disabling mmu interrupts\n");
		/* disable MMU */
		kgsl_regwrite(device, mmu_reg[device->id].interrupt_mask, 0);
		kgsl_regwrite(device, mmu_reg[device->id].config, 0x00000000);

		mmu->flags &= ~KGSL_FLAGS_STARTED;
	}

	KGSL_MEM_VDBG("return %d\n", 0);

	return 0;


}

int kgsl_mmu_close(struct kgsl_device *device)
{
	/*
	 *  close device mmu
	 *
	 *  call this with the global lock held
	 */
	struct kgsl_mmu *mmu = &device->mmu;

	KGSL_MEM_VDBG("enter (device=%p)\n", device);

	if (mmu->dummyspace.gpuaddr)
		kgsl_sharedmem_free(&mmu->dummyspace);

	KGSL_MEM_VDBG("return %d\n", 0);

	return 0;
}
