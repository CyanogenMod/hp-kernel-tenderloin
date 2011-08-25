/*
 * Basic implementation of a SW emulation of the domain manager feature in
 * ARM architecture.  Assumes single processor ARMv7 chipset.
 *
 * Requires hooks to be alerted to any runtime changes of dacr or MMU context.
 *
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/sched.h>
#include <asm/domain.h>
#include <asm/pgtable.h>
#include <asm/tlbflush.h>

#define DOMAIN_MANAGER_BITS (0xAAAAAAAA)

#define DFSR_DOMAIN(dfsr) ((dfsr >> 4) & (16-1))

#define FSR_PERMISSION_FAULT(fsr) ((fsr & 0x40D) == 0x00D)
#define FSR_PERMISSION_SECT(fsr) ((fsr & 0x40F) == 0x00D)

/* ARMv7 MMU HW Macros.  Not conveniently defined elsewhere */
#define MMU_TTB_ADDRESS(x)   ((u32 *)(((u32)(x)) & ~((1 << 14) - 1)))
#define MMU_PMD_INDEX(addr) (((u32)addr) >> SECTION_SHIFT)
#define MMU_TABLE_ADDRESS(x) ((u32 *)((x) & ~((1 << 10) - 1)))
#define MMU_TABLE_INDEX(x) ((((u32)x) >> 12) & (256 - 1))

/* Convenience Macros */
#define PMD_IS_VALID(x) (PMD_IS_TABLE(x) || PMD_IS_SECTION(x))
#define PMD_IS_TABLE(x) ((x & PMD_TYPE_MASK) == PMD_TYPE_TABLE)
#define PMD_IS_SECTION(x) ((x & PMD_TYPE_MASK) == PMD_TYPE_SECT)
#define PMD_IS_SUPERSECTION(x) \
	(PMD_IS_SECTION(x) && ((x & PMD_SECT_SUPER) == PMD_SECT_SUPER))

#define PMD_GET_DOMAIN(x)					\
	(PMD_IS_TABLE(x) ||					\
	(PMD_IS_SECTION(x) && !PMD_IS_SUPERSECTION(x)) ?	\
		 0 : (x >> 5) & (16-1))

#define PTE_IS_LARGE(x) ((x & PTE_TYPE_MASK) == PTE_TYPE_LARGE)


/* Only DOMAIN_MMU_ENTRIES will be granted access simultaneously */
#define DOMAIN_MMU_ENTRIES (8)

#define LRU_INC(lru) ((lru + 1) >= DOMAIN_MMU_ENTRIES ? 0 : lru + 1)


static DEFINE_SPINLOCK(edm_lock);

static u32 edm_manager_bits;

struct domain_entry_save {
	u32 *mmu_entry;
	u32 *addr;
	u32 value;
	u16 sect;
	u16 size;
};

static struct domain_entry_save edm_save[DOMAIN_MMU_ENTRIES];

static u32 edm_lru;


/*
 *  Return virtual address of pmd (level 1) entry for addr
 *
 *  This routine walks the ARMv7 page tables in HW.
 */
static inline u32 *__get_pmd_v7(u32 *addr)
{
	u32 *ttb;

	__asm__ __volatile__(
		"mrc	p15, 0, %0, c2, c0, 0	@ ttbr0\n\t"
		: "=r" (ttb)
		:
	);

	return __va(MMU_TTB_ADDRESS(ttb) + MMU_PMD_INDEX(addr));
}

/*
 *  Return virtual address of pte (level 2) entry for addr
 *
 *  This routine walks the ARMv7 page tables in HW.
 */
static inline u32 *__get_pte_v7(u32 *addr)
{
	u32 *pmd = __get_pmd_v7(addr);
	u32 *table_pa = pmd && PMD_IS_TABLE(*pmd) ?
		MMU_TABLE_ADDRESS(*pmd) : 0;
	u32 *entry = table_pa ? __va(table_pa[MMU_TABLE_INDEX(addr)]) : 0;

	return entry;
}

/*
 *  Invalidate the TLB for a given address for the current context
 *
 *  After manipulating access permissions, TLB invalidation changes are
 *  observed
 */
static inline void __tlb_invalidate(u32 *addr)
{
	__asm__ __volatile__(
		"mrc	p15, 0, %%r2, c13, c0, 1	@ contextidr\n\t"
		"and %%r2, %%r2, #0xff			@ asid\n\t"
		"mov %%r3, %0, lsr #12			@ mva[31:12]\n\t"
		"orr %%r2, %%r2, %%r3, lsl #12		@ tlb mva and asid\n\t"
		"mcr	p15, 0, %%r2, c8, c7, 1		@ utlbimva\n\t"
		"isb"
		:
		: "r" (addr)
		: "r2", "r3"
	);
}

/*
 *  Set HW MMU entry and do required synchronization operations.
 */
static inline void __set_entry(u32 *entry, u32 *addr, u32 value, int size)
{
	int i;

	if (!entry)
		return;

	entry = (u32 *)((u32) entry & ~(size * sizeof(u32) - 1));

	for (i = 0; i < size; i++)
		entry[i] = value;

	__asm__ __volatile__(
		"mcr	p15, 0, %0, c7, c10, 1		@ flush entry\n\t"
		"dsb\n\t"
		"isb\n\t"
		:
		: "r" (entry)
	);
	__tlb_invalidate(addr);
}

/*
 *  Return the number of duplicate entries associated with entry value.
 *  Supersections and Large page table entries are replicated 16x.
 */
static inline int __entry_size(int sect, int value)
{
	u32 size;

	if (sect)
		size = PMD_IS_SUPERSECTION(value) ? 16 : 1;
	else
		size = PTE_IS_LARGE(value) ? 16 : 1;

	return size;
}

/*
 *  Change entry permissions to emulate domain manager access
 */
static inline int __manager_perm(int sect, int value)
{
	u32 edm_value;

	if (sect) {
		edm_value = (value & ~(PMD_SECT_APX | PMD_SECT_XN)) |
		(PMD_SECT_AP_READ | PMD_SECT_AP_WRITE);
	} else {
		edm_value = (value & ~(PTE_EXT_APX | PTE_EXT_XN)) |
			(PTE_EXT_AP1 | PTE_EXT_AP0);
	}
	return edm_value;
}

/*
 *  Restore original HW MMU entry.  Cancels domain manager access
 */
static inline void __restore_entry(int index)
{
	struct domain_entry_save *entry = &edm_save[index];
	u32 edm_value;

	if (!entry->mmu_entry)
		return;

	edm_value = __manager_perm(entry->sect, entry->value);

	if (*entry->mmu_entry == edm_value)
		__set_entry(entry->mmu_entry, entry->addr,
			entry->value, entry->size);

	entry->mmu_entry = 0;
}

/*
 *  Modify HW MMU entry to grant domain manager access for a given MMU entry.
 *  This adds full read, write, and exec access permissions.
 */
static inline void __set_manager(int sect, u32 *addr)
{
	u32 *entry = sect ? __get_pmd_v7(addr) : __get_pte_v7(addr);
	u32 value;
	u32 edm_value;
	u16 size;

	if (!entry)
		return;

	value = *entry;

	size = __entry_size(sect, value);
	edm_value = __manager_perm(sect, value);

	__set_entry(entry, addr, edm_value, size);

	__restore_entry(edm_lru);

	edm_save[edm_lru].mmu_entry = entry;
	edm_save[edm_lru].addr = addr;
	edm_save[edm_lru].value = value;
	edm_save[edm_lru].sect = sect;
	edm_save[edm_lru].size = size;

	edm_lru = LRU_INC(edm_lru);
}

/*
 *  Restore original HW MMU entries.
 *
 *  entry - MVA for HW MMU entry
 */
static inline void __restore(void)
{
	if (unlikely(edm_manager_bits)) {
		u32 i;

		for (i = 0; i < DOMAIN_MMU_ENTRIES; i++)
			__restore_entry(i);
	}
}

/*
 * Common abort handler code
 *
 * If domain manager was actually set, permission fault would not happen.
 * Open access permissions to emulate.  Save original settings to restore
 * later. Return 1 to pretend fault did not happen.
 */
static int __emulate_domain_manager_abort(u32 fsr, u32 far, int dabort)
{
	if (unlikely(FSR_PERMISSION_FAULT(fsr) && edm_manager_bits)) {
		int domain = dabort ? DFSR_DOMAIN(fsr) : PMD_GET_DOMAIN(far);
		if (edm_manager_bits & domain_val(domain, DOMAIN_MANAGER)) {
			unsigned long flags;

			spin_lock_irqsave(&edm_lock, flags);

			__set_manager(FSR_PERMISSION_SECT(fsr), (u32 *) far);

			spin_unlock_irqrestore(&edm_lock, flags);
			return 1;
		}
	}
	return 0;
}

/*
 * Change domain setting.
 *
 * Lock and restore original contents.  Extract and save manager bits.  Set
 * DACR, excluding manager bits.
 */
void emulate_domain_manager_set(u32 domain)
{
	unsigned long flags;

	spin_lock_irqsave(&edm_lock, flags);

	if (edm_manager_bits != (domain & DOMAIN_MANAGER_BITS)) {
		__restore();
		edm_manager_bits = domain & DOMAIN_MANAGER_BITS;
	}

	__asm__ __volatile__(
		"mcr	p15, 0, %0, c3, c0, 0	@ set domain\n\t"
		"isb"
		:
		: "r" (domain & ~DOMAIN_MANAGER_BITS)
	);

	spin_unlock_irqrestore(&edm_lock, flags);
}

/*
 * Switch thread context.  Restore original contents.
 */
void emulate_domain_manager_switch_mm(unsigned long pgd_phys,
	struct mm_struct *mm,
	void (*switch_mm)(unsigned long pgd_phys, struct mm_struct *))
{
	unsigned long flags;

	spin_lock_irqsave(&edm_lock, flags);

	__restore();

	/* Call underlying kernel handler */
	switch_mm(pgd_phys, mm);

	spin_unlock_irqrestore(&edm_lock, flags);
}

/*
 * Kernel data_abort hook
 */
int emulate_domain_manager_data_abort(u32 dfsr, u32 dfar)
{
	return __emulate_domain_manager_abort(dfsr, dfar, 1);
}

/*
 * Kernel prefetch_abort hook
 */
int emulate_domain_manager_prefetch_abort(u32 ifsr, u32 ifar)
{
	return __emulate_domain_manager_abort(ifsr, ifar, 0);
}


