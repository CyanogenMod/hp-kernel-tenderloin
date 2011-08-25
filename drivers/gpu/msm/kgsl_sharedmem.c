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
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/genalloc.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <asm/cacheflush.h>

#include "kgsl_sharedmem.h"
#include "kgsl_device.h"
#include "kgsl.h"
#include "kgsl_log.h"
#include "kgsl_cffdump.h"

#ifdef CONFIG_OUTER_CACHE
static void _outer_cache_range_op(unsigned long addr, int size,
				  unsigned int flags)
{
	unsigned long end;

	for (end = addr; end < (addr + size); end += KGSL_PAGESIZE) {
		unsigned long physaddr = 0;

		if (flags & KGSL_MEMFLAGS_VMALLOC_MEM)
			physaddr = page_to_phys(vmalloc_to_page((void *) end));
		else if (flags & KGSL_MEMFLAGS_HOSTADDR)
			physaddr = kgsl_virtaddr_to_physaddr(end);
		else if (flags & KGSL_MEMFLAGS_CONPHYS)
			physaddr = __pa(end);

		if (physaddr == 0) {
			KGSL_MEM_ERR("Unable to find physaddr for "
				     "address: %x\n", (unsigned int)end);
			return;
		}

		if (flags & KGSL_MEMFLAGS_CACHE_FLUSH)
			outer_flush_range(physaddr, physaddr + KGSL_PAGESIZE);
		else if (flags & KGSL_MEMFLAGS_CACHE_CLEAN)
			outer_clean_range(physaddr, physaddr + KGSL_PAGESIZE);
		else if (flags & KGSL_MEMFLAGS_CACHE_INV)
			outer_inv_range(physaddr, physaddr + KGSL_PAGESIZE);
	}
	mb();
}
#else
static void _outer_cache_range_op(unsigned long addr, int size,
				  unsigned int flags)
{
}
#endif

void kgsl_cache_range_op(unsigned long addr, int size,
			 unsigned int flags)
{
	BUG_ON(addr & (KGSL_PAGESIZE - 1));
	BUG_ON(size & (KGSL_PAGESIZE - 1));

	if (flags & KGSL_MEMFLAGS_CACHE_FLUSH)
		dmac_flush_range((const void *)addr,
				 (const void *)(addr + size));
	else if (flags & KGSL_MEMFLAGS_CACHE_CLEAN)
		dmac_clean_range((const void *)addr,
				 (const void *)(addr + size));
	else if (flags & KGSL_MEMFLAGS_CACHE_INV)
		dmac_inv_range((const void *)addr,
			       (const void *)(addr + size));

	_outer_cache_range_op(addr, size, flags);

}

/* PALM; called from cacheflush syscall */
void kgsl_palm_cache_inv_range(unsigned long addr, int size)
{
    kgsl_cache_range_op(addr, size, KGSL_MEMFLAGS_CACHE_INV);
}
/* /PALM */

int
kgsl_sharedmem_vmalloc(struct kgsl_memdesc *memdesc,
		       struct kgsl_pagetable *pagetable, size_t size)
{
	int result;

	size = ALIGN(size, KGSL_PAGESIZE * 2);

	memdesc->hostptr = vmalloc(size);
	if (memdesc->hostptr == NULL) {
		KGSL_MEM_ERR("vmalloc failed: %x\n", size);
		return -ENOMEM;
	}

	memdesc->size = size;
	memdesc->pagetable = pagetable;
	memdesc->priv = KGSL_MEMFLAGS_VMALLOC_MEM | KGSL_MEMFLAGS_CACHE_CLEAN;

	kgsl_cache_range_op((unsigned int) memdesc->hostptr,
			    size, KGSL_MEMFLAGS_CACHE_INV |
			    KGSL_MEMFLAGS_VMALLOC_MEM);

	result = kgsl_mmu_map(pagetable, (unsigned long) memdesc->hostptr,
			      memdesc->size,
			      GSL_PT_PAGE_RV | GSL_PT_PAGE_WV,
			      &memdesc->gpuaddr,
			      KGSL_MEMFLAGS_ALIGN8K |
			      KGSL_MEMFLAGS_VMALLOC_MEM);

	if (result) {
		vfree(memdesc->hostptr);
		memset(memdesc, 0, sizeof(*memdesc));
	}

	return result;
}

void
kgsl_sharedmem_free(struct kgsl_memdesc *memdesc)
{
	KGSL_MEM_VDBG("enter (memdesc=%p, physaddr=%08x, size=%d)\n",
			memdesc, memdesc->physaddr, memdesc->size);

	BUG_ON(memdesc == NULL);

	if (memdesc->size > 0) {
		if (memdesc->priv & KGSL_MEMFLAGS_VMALLOC_MEM) {
			if (memdesc->gpuaddr)
				kgsl_mmu_unmap(memdesc->pagetable,
					       memdesc->gpuaddr,
					       memdesc->size);

			if (memdesc->hostptr)
				vfree(memdesc->hostptr);
		} else if (memdesc->priv & KGSL_MEMFLAGS_CONPHYS)
			dma_free_coherent(NULL, memdesc->size,
					  memdesc->hostptr,
					  memdesc->physaddr);
		else
			BUG();
	}

	memset(memdesc, 0, sizeof(struct kgsl_memdesc));
	KGSL_MEM_VDBG("return\n");
}

int
kgsl_sharedmem_readl(const struct kgsl_memdesc *memdesc,
			uint32_t *dst,
			unsigned int offsetbytes)
{
	if (memdesc == NULL || memdesc->hostptr == NULL || dst == NULL) {
		KGSL_MEM_ERR("bad ptr memdesc %p hostptr %p dst %p\n",
				memdesc,
				(memdesc ? memdesc->hostptr : NULL),
				dst);
		return -EINVAL;
	}
	if (offsetbytes + sizeof(unsigned int) > memdesc->size) {
		KGSL_MEM_ERR("bad range: offset %d memdesc %d\n",
				offsetbytes, memdesc->size);
		return -ERANGE;
	}
	*dst = readl(memdesc->hostptr + offsetbytes);
	return 0;
}

int
kgsl_sharedmem_read(const struct kgsl_memdesc *memdesc, void *dst,
			unsigned int offsetbytes, unsigned int sizebytes)
{
	BUG_ON(sizebytes == sizeof(unsigned int));
	if (memdesc == NULL || memdesc->hostptr == NULL || dst == NULL) {
		KGSL_MEM_ERR("bad ptr memdesc %p hostptr %p dst %p\n",
				memdesc,
				(memdesc ? memdesc->hostptr : NULL),
				dst);
		return -EINVAL;
	}
	if (offsetbytes + sizebytes > memdesc->size) {
		KGSL_MEM_ERR("bad range: offset %d size %d memdesc %d\n",
				offsetbytes, sizebytes, memdesc->size);
		return -ERANGE;
	}
	memcpy(dst, memdesc->hostptr + offsetbytes, sizebytes);
	return 0;
}

int
kgsl_sharedmem_writel(const struct kgsl_memdesc *memdesc,
			unsigned int offsetbytes,
			uint32_t src)
{
	if (memdesc == NULL || memdesc->hostptr == NULL) {
		KGSL_MEM_ERR("bad ptr memdesc %p hostptr %p\n", memdesc,
				(memdesc ? memdesc->hostptr : NULL));
		return -EINVAL;
	}
	if (offsetbytes + sizeof(unsigned int) > memdesc->size) {
		KGSL_MEM_ERR("bad range: offset %d memdesc %d\n",
				offsetbytes, memdesc->size);
		return -ERANGE;
	}
	kgsl_cffdump_setmem(memdesc->gpuaddr + offsetbytes,
		src, sizeof(uint));
	writel(src, memdesc->hostptr + offsetbytes);
	return 0;
}


int
kgsl_sharedmem_write(const struct kgsl_memdesc *memdesc,
			unsigned int offsetbytes,
			void *src, unsigned int sizebytes)
{
	BUG_ON(sizebytes == sizeof(unsigned int));
	if (memdesc == NULL || memdesc->hostptr == NULL) {
		KGSL_MEM_ERR("bad ptr memdesc %p hostptr %p\n", memdesc,
				(memdesc ? memdesc->hostptr : NULL));
		return -EINVAL;
	}
	if (offsetbytes + sizebytes > memdesc->size) {
		KGSL_MEM_ERR("bad range: offset %d size %d memdesc %d\n",
				offsetbytes, sizebytes, memdesc->size);
		return -ERANGE;
	}

	memcpy(memdesc->hostptr + offsetbytes, src, sizebytes);
	kgsl_cffdump_syncmem(NULL, memdesc, memdesc->gpuaddr + offsetbytes,
		sizebytes, false);
	return 0;
}

int
kgsl_sharedmem_set(const struct kgsl_memdesc *memdesc, unsigned int offsetbytes,
			unsigned int value, unsigned int sizebytes)
{
	if (memdesc == NULL || memdesc->hostptr == NULL) {
		KGSL_MEM_ERR("bad ptr memdesc %p hostptr %p\n", memdesc,
				(memdesc ? memdesc->hostptr : NULL));
		return -EINVAL;
	}
	if (offsetbytes + sizebytes > memdesc->size) {
		KGSL_MEM_ERR("bad range: offset %d size %d memdesc %d\n",
				offsetbytes, sizebytes, memdesc->size);
		return -ERANGE;
	}
	kgsl_cffdump_setmem(memdesc->gpuaddr + offsetbytes,
			    value, sizebytes);
	memset(memdesc->hostptr + offsetbytes, value, sizebytes);
	return 0;
}

