/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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

#ifndef VCM_ALLOC_H
#define VCM_ALLOC_H

#include <linux/list.h>
#include <linux/vcm.h>
#include <linux/vcm_types.h>

#define MAX_NUM_PRIO_POOLS 8

/* Data structure to inform VCM about the memory it manages */
struct physmem_region {
	size_t addr;
	size_t size;
	int chunk_size;
};

/* Mapping between memtypes and physmem_regions based on chunk size */
struct vcm_memtype_map {
	int pool_id[MAX_NUM_PRIO_POOLS];
	int num_pools;
};

int vcm_alloc_pool_idx_to_size(int pool_idx);
int vcm_alloc_idx_to_size(int idx);
int vcm_alloc_get_mem_size(void);
int vcm_alloc_blocks_avail(enum memtype_t memtype, int idx);
int vcm_alloc_get_num_chunks(enum memtype_t memtype);
int vcm_alloc_all_blocks_avail(enum memtarget_t memtype);
int vcm_alloc_count_allocated(enum memtype_t memtype);
void vcm_alloc_print_list(enum memtype_t memtype, int just_allocated);
int vcm_alloc_idx_to_size(int idx);
int vcm_alloc_destroy(void);
int vcm_alloc_init(struct physmem_region *mem, int n_regions,
		   struct vcm_memtype_map *mt_map, int n_mt);
int vcm_alloc_free_blocks(enum memtype_t memtype,
			  struct phys_chunk *alloc_head);
int vcm_alloc_num_blocks(int num, enum memtype_t memtype,
			 int idx, /* chunk size */
			 struct phys_chunk *alloc_head);
int vcm_alloc_max_munch(int len, enum memtype_t memtype,
			struct phys_chunk *alloc_head);

/* bring-up init, destroy */
int vcm_sys_init(struct physmem_region *mem, int n_regions,
		 struct vcm_memtype_map *mt_map, int n_mt,
		 void *cont_pa, unsigned int cont_sz);

int vcm_sys_destroy(void);

#endif /* VCM_ALLOC_H */
