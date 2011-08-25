/* arch/arm/mach-msm/memory.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/bootmem.h>
#include <linux/module.h>
#include <asm/pgtable.h>
#include <asm/io.h>
#include <asm/mach/map.h>
#include <asm/cacheflush.h>
#include <linux/hardirq.h>
#if defined(CONFIG_MSM_NPA_REMOTE)
#include "npa_remote.h"
#include <linux/completion.h>
#include <linux/err.h>
#endif

int arch_io_remap_pfn_range(struct vm_area_struct *vma, unsigned long addr,
			    unsigned long pfn, unsigned long size, pgprot_t prot)
{
	unsigned long pfn_addr = pfn << PAGE_SHIFT;
#if defined(CONFIG_ARCH_MSM8X60)
	if (pfn_addr < 0x40000000) {
#else
	if ((pfn_addr >= 0x88000000) && (pfn_addr < 0xD0000000)) {
#endif
		prot = pgprot_device(prot);
		printk("remapping device %lx\n", prot);
	}
	return remap_pfn_range(vma, addr, pfn, size, prot);
}

void *strongly_ordered_page;

/*
 * The trick of making the zero page strongly ordered no longer
 * works. We no longer want to make a second alias to the zero
 * page that is strongly ordered. Manually changing the bits
 * in the page table for the zero page would have side effects
 * elsewhere that aren't necessary. The result is that we need
 * to get a page from else where. Given when the first call
 * to write_to_strongly_ordered_memory occurs, using bootmem
 * to get a page makes the most sense.
 */
void map_page_strongly_ordered(void)
{
#if defined(CONFIG_ARCH_MSM7X27)
	long unsigned int phys;

	if (strongly_ordered_page)
		return;

	strongly_ordered_page = alloc_bootmem(PAGE_SIZE);
	phys = __pa(strongly_ordered_page);
	ioremap_page((long unsigned int) strongly_ordered_page,
		phys,
		get_mem_type(MT_DEVICE_STRONGLY_ORDERED));
	printk(KERN_ALERT "Initialized strongly ordered page successfully\n");
#endif
}
EXPORT_SYMBOL(map_page_strongly_ordered);

void write_to_strongly_ordered_memory(void)
{
#if defined(CONFIG_ARCH_MSM7X27)
	if (!strongly_ordered_page) {
		if (!in_interrupt())
			map_page_strongly_ordered();
		else {
			printk(KERN_ALERT "Cannot map strongly ordered page in "
				"Interrupt Context\n");
			/* capture it here before the allocation fails later */
			BUG();
		}
	}
	*(int *)strongly_ordered_page = 0;
#endif
}
EXPORT_SYMBOL(write_to_strongly_ordered_memory);

void flush_axi_bus_buffer(void)
{
#if defined(CONFIG_ARCH_MSM7X27)
	__asm__ __volatile__ ("mcr p15, 0, %0, c7, c10, 5" \
				    : : "r" (0) : "memory");
	write_to_strongly_ordered_memory();
#endif
}

#define CACHE_LINE_SIZE 32

/* These cache related routines make the assumption that the associated
 * physical memory is contiguous. They will operate on all (L1
 * and L2 if present) caches.
 */
void clean_and_invalidate_caches(unsigned long vstart,
	unsigned long length, unsigned long pstart)
{
	unsigned long vaddr;

	for (vaddr = vstart; vaddr < vstart + length; vaddr += CACHE_LINE_SIZE)
		asm ("mcr p15, 0, %0, c7, c14, 1" : : "r" (vaddr));
#ifdef CONFIG_OUTER_CACHE
	outer_flush_range(pstart, pstart + length);
#endif
	asm ("mcr p15, 0, %0, c7, c10, 4" : : "r" (0));
	asm ("mcr p15, 0, %0, c7, c5, 0" : : "r" (0));

	flush_axi_bus_buffer();
}

void clean_caches(unsigned long vstart,
	unsigned long length, unsigned long pstart)
{
	unsigned long vaddr;

	for (vaddr = vstart; vaddr < vstart + length; vaddr += CACHE_LINE_SIZE)
		asm ("mcr p15, 0, %0, c7, c10, 1" : : "r" (vaddr));
#ifdef CONFIG_OUTER_CACHE
	outer_clean_range(pstart, pstart + length);
#endif
	asm ("mcr p15, 0, %0, c7, c10, 4" : : "r" (0));
	asm ("mcr p15, 0, %0, c7, c5, 0" : : "r" (0));

	flush_axi_bus_buffer();
}

void invalidate_caches(unsigned long vstart,
	unsigned long length, unsigned long pstart)
{
	unsigned long vaddr;

	for (vaddr = vstart; vaddr < vstart + length; vaddr += CACHE_LINE_SIZE)
		asm ("mcr p15, 0, %0, c7, c6, 1" : : "r" (vaddr));
#ifdef CONFIG_OUTER_CACHE
	outer_inv_range(pstart, pstart + length);
#endif
	asm ("mcr p15, 0, %0, c7, c10, 4" : : "r" (0));
	asm ("mcr p15, 0, %0, c7, c5, 0" : : "r" (0));

	flush_axi_bus_buffer();
}

void *alloc_bootmem_aligned(unsigned long size, unsigned long alignment)
{
	void *unused_addr = NULL;
	unsigned long addr, tmp_size, unused_size;

	/* Allocate maximum size needed, see where it ends up.
	 * Then free it -- in this path there are no other allocators
	 * so we can depend on getting the same address back
	 * when we allocate a smaller piece that is aligned
	 * at the end (if necessary) and the piece we really want,
	 * then free the unused first piece.
	 */

	tmp_size = size + alignment - PAGE_SIZE;
	addr = (unsigned long)alloc_bootmem(tmp_size);
	free_bootmem(__pa(addr), tmp_size);

	unused_size = alignment - (addr % alignment);
	if (unused_size)
		unused_addr = alloc_bootmem(unused_size);

	addr = (unsigned long)alloc_bootmem(size);
	if (unused_size)
		free_bootmem(__pa(unused_addr), unused_size);

	return (void *)addr;
}

int platform_physical_remove_pages(unsigned long start_pfn,
	unsigned long nr_pages)
{
	return 1;
}

int platform_physical_active_pages(unsigned long start_pfn,
	unsigned long nr_pages)
{
	return 1;
}

int platform_physical_low_power_pages(unsigned long start_pfn,
	unsigned long nr_pages)
{
	return 1;
}
