/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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

#include "vidc_type.h"
#include "vcd_ddl_utils.h"

#if DEBUG
#define DBG(x...) printk(KERN_DEBUG x)
#else
#define DBG(x...)
#endif

#define ERR(x...) printk(KERN_ERR x)

static unsigned int ddl_dec_t1, ddl_enc_t1;
static unsigned int ddl_dec_ttotal, ddl_enc_ttotal;
static unsigned int ddl_dec_count, ddl_enc_count;

#ifdef NO_IN_KERNEL_PMEM

void ddl_pmem_alloc(struct ddl_buf_addr *buff_addr, size_t sz, u32 align)
{
	u32 guard_bytes, align_mask;
	u32 physical_addr, align_offset;
	dma_addr_t phy_addr;

	if (align == DDL_LINEAR_BUFFER_ALIGN_BYTES) {

		guard_bytes = 31;
		align_mask = 0xFFFFFFE0U;

	} else {

		guard_bytes = DDL_TILE_BUF_ALIGN_GUARD_BYTES;
		align_mask = DDL_TILE_BUF_ALIGN_MASK;
	}

	buff_addr->virtual_base_addr =
		kmalloc((sz + guard_bytes), GFP_KERNEL);

	if (!buff_addr->virtual_base_addr) {
		ERR("\n ERROR %s:%u kamlloc fails to allocate"
			" sz + guard_bytes = %u\n", __func__, __LINE__,
			(sz + guard_bytes));
		return;
	}

	phy_addr = dma_map_single(NULL, buff_addr->virtual_base_addr,
				  sz + guard_bytes, DMA_TO_DEVICE);

	buff_addr->buffer_size = sz;
	physical_addr = (u32) phy_addr;
	buff_addr->align_physical_addr =
	    (u32 *) ((physical_addr + guard_bytes) & align_mask);
	align_offset =
	    (u32) (buff_addr->align_physical_addr) - physical_addr;
	buff_addr->align_virtual_addr =
	    (u32 *) ((u32) (buff_addr->virtual_base_addr)
		     + align_offset);
}

void ddl_pmem_free(struct ddl_buf_addr buff_addr)
{
	kfree(buff_addr.virtual_base_addr);
	buff_addr.buffer_size = 0;
	buff_addr.virtual_base_addr = NULL;
}

#else

void ddl_pmem_alloc(struct ddl_buf_addr *buff_addr, size_t sz, u32 align)
{
	u32 guard_bytes, align_mask;
	s32 physical_addr;
	u32 align_offset;

	DBG("\n%s() IN : phy_addr(%p) ker_addr(%p) size(%u)",
		__func__, buff_addr->physical_base_addr,
		buff_addr->virtual_base_addr, (u32)sz);

	if (align == DDL_LINEAR_BUFFER_ALIGN_BYTES) {

		guard_bytes = 31;
		align_mask = 0xFFFFFFE0U;

	} else {

		guard_bytes = DDL_TILE_BUF_ALIGN_GUARD_BYTES;
		align_mask = DDL_TILE_BUF_ALIGN_MASK;
	}

	physical_addr = pmem_kalloc((sz + guard_bytes),
				      PMEM_MEMTYPE_EBI1 | PMEM_ALIGNMENT_4K);
	buff_addr->physical_base_addr = (u32 *)physical_addr;

	if (IS_ERR((void *)physical_addr)) {
		pr_err("%s(): could not allocte in kernel pmem buffers\n",
		       __func__);
		return;
	}

	buff_addr->virtual_base_addr =
	    (u32 *) ioremap((unsigned long)physical_addr,
			    sz + guard_bytes);
	memset(buff_addr->virtual_base_addr, 0 , sz + guard_bytes);
	if (!buff_addr->virtual_base_addr) {

		pr_err("%s: could not ioremap in kernel pmem buffers\n",
		       __func__);
		pmem_kfree(physical_addr);
		return;
	}

	buff_addr->buffer_size = sz;

	buff_addr->align_physical_addr =
	    (u32 *) ((physical_addr + guard_bytes) & align_mask);

	align_offset =
	    (u32) (buff_addr->align_physical_addr) - physical_addr;

	buff_addr->align_virtual_addr =
	    (u32 *) ((u32) (buff_addr->virtual_base_addr)
		     + align_offset);

	DBG("\n%s() OUT : phy_addr(%p) ker_addr(%p) size(%u)", __func__,
		buff_addr->physical_base_addr, buff_addr->virtual_base_addr,
		buff_addr->buffer_size);

	return;
}

void ddl_pmem_free(struct ddl_buf_addr buff_addr)
{
	DBG("\n%s(): phy_addr %p ker_addr %p", __func__,
		buff_addr.physical_base_addr, buff_addr.virtual_base_addr);

	if (buff_addr.virtual_base_addr)
		iounmap((void *)buff_addr.virtual_base_addr);

	if ((buff_addr.physical_base_addr) &&
		pmem_kfree((s32) buff_addr.physical_base_addr)) {
		ERR("\n %s(): Error in Freeing ddl_pmem_free "
		"Physical Address %p", __func__,
		buff_addr.physical_base_addr);
	}

	buff_addr.buffer_size = 0;
	buff_addr.physical_base_addr = NULL;
	buff_addr.virtual_base_addr = NULL;
}
#endif

void ddl_get_core_start_time(u8 codec)
{
	u32 *ddl_t1 = NULL;
	if (!codec)
		ddl_t1 = &ddl_dec_t1;
	else if (codec == 1)
		ddl_t1 = &ddl_enc_t1;

	if (!*ddl_t1) {
		struct timeval ddl_tv;
		do_gettimeofday(&ddl_tv);
		*ddl_t1 = (ddl_tv.tv_sec * 1000) + (ddl_tv.tv_usec / 1000);
	}
}

void ddl_calc_core_time(u8 codec)
{
	u32 *ddl_t1 = NULL, *ddl_ttotal = NULL,
		*ddl_count = NULL;
	if (!codec) {
		DBG("\n720p Core Decode ");
		ddl_t1 = &ddl_dec_t1;
		ddl_ttotal = &ddl_dec_ttotal;
		ddl_count = &ddl_dec_count;
	} else if (codec == 1) {
		DBG("\n720p Core Encode ");
		ddl_t1 = &ddl_enc_t1;
		ddl_ttotal = &ddl_enc_ttotal;
		ddl_count = &ddl_enc_count;
	}

	if (*ddl_t1) {
		int ddl_t2;
		struct timeval ddl_tv;
		do_gettimeofday(&ddl_tv);
		ddl_t2 = (ddl_tv.tv_sec * 1000) + (ddl_tv.tv_usec / 1000);
		*ddl_ttotal += (ddl_t2 - *ddl_t1);
		*ddl_count = *ddl_count + 1;
		DBG("time %u, average time %u, count %u",
			ddl_t2 - *ddl_t1, (*ddl_ttotal)/(*ddl_count),
			*ddl_count);
		*ddl_t1 = 0;
	}
}

void ddl_reset_time_variables(u8 codec)
{
	if (!codec) {
		DBG("\n Reset Decoder time variables");
		ddl_dec_t1 = 0;
		ddl_dec_ttotal = 0;
		ddl_dec_count = 0;
	} else if (codec == 1) {
		DBG("\n Reset Encoder time variables ");
		ddl_enc_t1 = 0;
		ddl_enc_ttotal = 0;
		ddl_enc_count = 0;
	}
}
