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
#ifndef __GSL_SHAREDMEM_H
#define __GSL_SHAREDMEM_H

#include <linux/dma-mapping.h>

#define KGSL_PAGESIZE           0x1000
#define KGSL_PAGESIZE_SHIFT     12
#define KGSL_PAGEMASK           (~(KGSL_PAGESIZE - 1))

struct kgsl_pagetable;

/* Memflags for caching operations */
#define KGSL_MEMFLAGS_CACHE_INV		0x00000001
#define KGSL_MEMFLAGS_CACHE_FLUSH	0x00000002
#define KGSL_MEMFLAGS_CACHE_CLEAN	0x00000004
#define KGSL_MEMFLAGS_CACHE_MASK	0x0000000F

/* Flags to differentiate memory types */
#define KGSL_MEMFLAGS_CONPHYS 	0x00001000
#define KGSL_MEMFLAGS_VMALLOC_MEM	0x00002000
#define KGSL_MEMFLAGS_HOSTADDR		0x00004000

#define KGSL_MEMFLAGS_ALIGNANY	0x00000000
#define KGSL_MEMFLAGS_ALIGN32	0x00000000
#define KGSL_MEMFLAGS_ALIGN64	0x00060000
#define KGSL_MEMFLAGS_ALIGN128	0x00070000
#define KGSL_MEMFLAGS_ALIGN256	0x00080000
#define KGSL_MEMFLAGS_ALIGN512	0x00090000
#define KGSL_MEMFLAGS_ALIGN1K	0x000A0000
#define KGSL_MEMFLAGS_ALIGN2K	0x000B0000
#define KGSL_MEMFLAGS_ALIGN4K	0x000C0000
#define KGSL_MEMFLAGS_ALIGN8K	0x000D0000
#define KGSL_MEMFLAGS_ALIGN16K	0x000E0000
#define KGSL_MEMFLAGS_ALIGN32K	0x000F0000
#define KGSL_MEMFLAGS_ALIGN64K	0x00100000
#define KGSL_MEMFLAGS_ALIGNPAGE	KGSL_MEMFLAGS_ALIGN4K


#define KGSL_MEMFLAGS_ALIGN_MASK 	0x00FF0000
#define KGSL_MEMFLAGS_ALIGN_SHIFT	16


/* shared memory allocation */
struct kgsl_memdesc {
	struct kgsl_pagetable *pagetable;
	void  *hostptr;
	unsigned int gpuaddr;
	unsigned int physaddr;
	unsigned int size;
	unsigned int priv;
};

int kgsl_sharedmem_vmalloc(struct kgsl_memdesc *memdesc,
			   struct kgsl_pagetable *pagetable, size_t size);

static inline int
kgsl_sharedmem_alloc_coherent(struct kgsl_memdesc *memdesc, size_t size)
{
	size = ALIGN(size, KGSL_PAGESIZE);

	memdesc->hostptr = dma_alloc_coherent(NULL, size, &memdesc->physaddr,
					      GFP_KERNEL);
	if (!memdesc->hostptr)
		return -ENOMEM;
	memdesc->size = size;
	memdesc->priv = KGSL_MEMFLAGS_CONPHYS;
	return 0;
}

void kgsl_sharedmem_free(struct kgsl_memdesc *memdesc);

int kgsl_sharedmem_readl(const struct kgsl_memdesc *memdesc,
			uint32_t *dst,
			unsigned int offsetbytes);

int kgsl_sharedmem_read(const struct kgsl_memdesc *memdesc, void *dst,
			unsigned int offsetbytes, unsigned int sizebytes);

int kgsl_sharedmem_writel(const struct kgsl_memdesc *memdesc,
			unsigned int offsetbytes,
			uint32_t src);

int kgsl_sharedmem_write(const struct kgsl_memdesc *memdesc,
			unsigned int offsetbytes,
			void *src, unsigned int sizebytes);

int kgsl_sharedmem_set(const struct kgsl_memdesc *memdesc,
			unsigned int offsetbytes, unsigned int value,
			unsigned int sizebytes);

void kgsl_cache_range_op(unsigned long addr, int size,
			 unsigned int flags);

#endif /* __GSL_SHAREDMEM_H */
