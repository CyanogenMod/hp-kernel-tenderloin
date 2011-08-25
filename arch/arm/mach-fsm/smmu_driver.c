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
 */

#define pr_fmt(fmt) "%s %i " fmt, __func__, __LINE__

#include <linux/kernel.h>
#include <linux/mmu_context.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/vcm_mm.h>
#include <linux/spinlock.h>

#include <asm/pgtable-hwdef.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>
#include <asm/sizes.h>

#include <mach/smmu_driver.h>
#include <mach/smmu_hw-8xxx.h>

#define MRC(reg, processor, op1, crn, crm, op2)				\
  __asm__ __volatile__ (						\
"   mrc   "   #processor "," #op1 ", %0,"  #crn "," #crm "," #op2 " \n" \
: "=r" (reg))

#define RCP15_PRRR(reg)		MRC(reg, p15, 0, c10, c2, 0)
#define RCP15_NMRR(reg) 	MRC(reg, p15, 0, c10, c2, 1)

DEFINE_SPINLOCK(smmulock);

static int smmu_context_reset(unsigned long base, int ctx)
{
	SET_BPRCOSH(base, ctx, 0); mb();
	SET_BPRCISH(base, ctx, 0); mb();
	SET_BPRCNSH(base, ctx, 0); mb();
	SET_BPSHCFG(base, ctx, 0); mb();
	SET_BPMTCFG(base, ctx, 0); mb();	/* memory type = 0 */
	SET_ACTLR(base, ctx, 0);
	SET_SCTLR(base, ctx, 0);
	SET_FSRRESTORE(base, ctx, 0);
	SET_TTBR0(base, ctx, 0);
	SET_TTBR1(base, ctx, 0);
	SET_TTBCR(base, ctx, 0);
	SET_BFBCR(base, ctx, 0);
	SET_PAR(base, ctx, 0);
	SET_FAR(base, ctx, 0);

	SET_CTX_TLBIALL(base, ctx, 0);
	SET_TLBFLPTER(base, ctx, 0);
	SET_TLBSLPTER(base, ctx, 0);
	SET_TLBLKCR(base, ctx, 0);
	SET_PRRR(base, ctx, 0);
	SET_NMRR(base, ctx, 0);
	SET_CONTEXTIDR(base, ctx, 0);
	mb();
	return 0;
}


static void print_ctx_regs(unsigned long base, int ctx)
{
	unsigned int fsr = GET_FSR(base, ctx);
	printk(KERN_ERR "FAR    = %08x    PAR    = %08x\n",
	       GET_FAR(base, ctx), GET_PAR(base, ctx));
	printk(KERN_ERR "FSR    = %08x [%s%s%s%s%s%s%s%s%s%s]\n", fsr,
			(fsr & 0x02) ? "TF " : "",
			(fsr & 0x04) ? "AFF " : "",
			(fsr & 0x08) ? "APF " : "",
			(fsr & 0x10) ? "TLBMF " : "",
			(fsr & 0x20) ? "HTWDEEF " : "",
			(fsr & 0x40) ? "HTWSEEF " : "",
			(fsr & 0x80) ? "MHF " : "",
			(fsr & 0x10000) ? "SL " : "",
			(fsr & 0x40000000) ? "SS " : "",
			(fsr & 0x80000000) ? "MULTI " : "");


	printk(KERN_ERR "FSYNR0 = %08x    FSYNR1 = %08x\n",
			GET_FSYNR0(base, ctx), GET_FSYNR1(base, ctx));
	printk(KERN_ERR "TTBR0  = %08x    TTBR1  = %08x\n",
			GET_TTBR0(base, ctx), GET_TTBR1(base, ctx));
	printk(KERN_ERR "SCTLR  = %08x    ACTLR  = %08x\n",
			GET_SCTLR(base, ctx), GET_ACTLR(base, ctx));
	printk(KERN_ERR "PRRR   = %08x    NMRR   = %08x\n",
			GET_PRRR(base, ctx), GET_NMRR(base, ctx));
}

static void print_fault_data(struct smmu_driver *drv, struct smmu_dev *dev)
{
	int ncb = 0, i = 0;

	if (!drv)
		panic("Null driver while printing fault data.");

	printk(KERN_ERR "===== WOAH! =====\n");
	printk(KERN_ERR "Unexpected SMMU page fault!\n");
	printk(KERN_ERR "base = %08x, irq = %d\n", (unsigned int)drv->base,
			drv->irq);
	if (dev) {
		printk(KERN_ERR "Fault occurred in context %d\n", dev->context);
		printk(KERN_ERR "Interesting registers:\n");
		print_ctx_regs(drv->base, dev->context);
	} else {
		printk(KERN_ERR "Fault occurred within inactive context.\n");
		printk(KERN_ERR "You might have wedged your multimedia core\n");
		ncb = GET_NCB(drv->base)+1;
		for (i = 0; i < ncb; i++) {
			printk(KERN_ERR "Context %d registers:\n", i);
			print_ctx_regs(drv->base, i);
			printk(KERN_ERR "\n");
		}
	}
}


static irqreturn_t smmu_secure_irpt_handler(int irq, void *dev_id)
{
	struct smmu_driver *drv = (struct smmu_driver *) dev_id;
	struct list_head *tmp = NULL;
	unsigned int fsr = 0;

	spin_lock(&smmulock);

	if (!drv) {
		pr_err("Invalid device ID in context interrupt handler\n");
		spin_unlock(&smmulock);
		return 0;
	}

	/* Find the context for this interrupt */
	list_for_each(tmp, &drv->list_active) {
		struct smmu_dev *dev;
		dev = list_entry(tmp, struct smmu_dev, dev_elm);

		mb();

		/* Preserve FSR value in case we need it to print debug info */
		fsr = GET_FSR(dev->base, dev->context);
		if (fsr != 0x00) {
			if (dev->smmu_vcm_handler != NULL) {
				int ret;
				/* Need better clearing mechanism based on
				 * fault type
				 */
				SET_FSR(dev->base, dev->context, 0x4000000F);
				mb();
				ret = dev->smmu_vcm_handler((size_t) dev,
					    dev->smmu_vcm_handler_data,	NULL);

				/* Terminate access if handler says so */
				if (ret)
					SET_RESUME(dev->base, dev->context, 1);
				else
					SET_RESUME(dev->base, dev->context, 0);
			} else { /* Terminate by default */
				print_fault_data(drv, dev);

				/* Clear the interrupt */
				SET_FSR(dev->base, dev->context, 0x4000000F);
				SET_RESUME(dev->base, dev->context, 1);
				BUG();
			}
			spin_unlock(&smmulock);
			return 0;
		}
	}

	pr_err("Unhandled SMMU context interrupt - no matching contexts.\n");
	print_fault_data(drv, NULL);
	BUG();
	spin_unlock(&smmulock);
	return 0;
}


struct smmu_driver *smmu_get_driver(struct smmu_dev *dev)
{
	if (!dev) {
		pr_err("Null device\n");
		return NULL;
	}
	return dev->drv;
}


int smmu_drvdata_init(struct smmu_driver *drv, unsigned long base, int irq)
{
	int ret, i, ncb;

	if (!drv) {
		pr_err("Null driver\n");
		goto fail;
	}
	if (irq <  0) {
		pr_err("Invalid IRQ: %d\n", irq);
		goto fail;
	}

	INIT_LIST_HEAD(&drv->list_active);

	drv->base = base;
	drv->irq = irq;

	SET_RPUE(base, 0);	mb();
	SET_CLIENTPD(base, 1); mb();
	SET_RPUEIE(base, 0); mb();
	SET_ESRRESTORE(base, 0);
	SET_TBE(base, 0); mb();
	SET_CR(base, 0);
	SET_SPDMBE(base, 0); mb();

	SET_TESTBUSCR(base, 0);
	SET_TLBRSW(base, 0);
	SET_GLOBAL_TLBIALL(base, 0);
	SET_RPU_ACR(base, 0);
	SET_TLBLKCRWE(base, 1);
	ncb = GET_NCB(base)+1;

	for (i = 0; i < ncb; i++)
		smmu_context_reset(base, i);

	ret = request_irq(irq, smmu_secure_irpt_handler,
			  0, "smmu_secure_irpt_handler", drv);
	if (ret) {
		pr_err("Request irq %d failed with result %d\n", irq, ret);
		goto fail;
	}
	return 0;
fail:
	return 1;
}


struct smmu_dev *smmu_ctx_init(int context)
{
	struct smmu_dev *dev = NULL;

	dev = kzalloc(sizeof(struct smmu_dev), GFP_KERNEL);

	if (!dev) {
		pr_err("dev kzalloc failed\n");
		goto fail;
	}

	INIT_LIST_HEAD(&dev->dev_elm);

	dev->drv = NULL;
	dev->base = 0;
	dev->context = context;
	dev->fl_table = (unsigned long *)__get_free_pages(GFP_KERNEL, 2);
	dev->smmu_vcm_handler = NULL;
	dev->smmu_vcm_handler_data = NULL;

	if (!dev->fl_table) {
		pr_err("null page table; out of memory?\n");
		goto fail2;
	}

	memset(dev->fl_table, 0, SZ_16K);

	return dev;

fail2:
	memset(dev, 0, sizeof(struct smmu_dev));
	kfree(dev);
fail:

	return NULL;
}


int smmu_ctx_bind(struct smmu_dev *ctx, struct smmu_driver *drv)
{
	if (!drv) {
		pr_err("null driver\n");
		goto fail;
	}

	if (!ctx) {
		pr_err("null context\n");
		goto fail;
	}

	/* Already bound to something else */
	if (ctx->drv != NULL && ctx->drv != drv) {
		pr_err("context already bound to another driver, drv=%p, \
			 ctx->drv=%p\n", drv, ctx->drv);
		goto fail;
	}

	if (ctx->context < 0 || ctx->context > (GET_NCB(drv->base)+1)) {
		pr_err("bad context number: %d\n", ctx->context);
		goto fail;
	}

	ctx->drv = drv;
	ctx->base = drv->base;

	return 0;
fail:
	return -1;
}


static int __smmu_is_active(struct smmu_dev *dev)
{
	if (!dev) {
		pr_err("null device\n");
		goto fail;
	}
	if (!dev->drv)
		return 0;	/* Can't be active if no SMMU device */

	return !list_empty(&dev->dev_elm);
fail:
	return -1;
}


int smmu_is_active(struct smmu_dev *dev)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&smmulock, flags);
	ret = __smmu_is_active(dev);
	spin_unlock_irqrestore(&smmulock, flags);
	return ret;
}


int smmu_ctx_deinit(struct smmu_dev *dev)
{
	int i;
	if (!dev) {
		pr_err("null device\n");
		goto fail_bad_input;
	}

	/* Is this on the active list? */
	if (dev->drv && __smmu_is_active(dev)) {
		pr_err("device still active\n");
		goto fail_busy;
	}

	/* Do mappings still exist? */
	if (dev->fl_table) {
		for (i = 0; i < NUM_FL_PTE; i++)
			if (dev->fl_table[i]) {
				pr_err("mappings still exist. pgtable at %p,\
					 i=%d\n", (void *) dev->fl_table, i);
				goto fail_busy;
			}

		free_pages((unsigned long)dev->fl_table, 2);
	}

	memset(dev, 0, sizeof(*dev));
	kfree(dev);

	return 0;
fail_bad_input:
	return -ENODEV;
fail_busy:
	return -EBUSY;
}


int smmu_hook_irpt(struct smmu_dev *dev, vcm_handler handler, void *data)
{
	unsigned long flags;
	if (!dev) {
		pr_err("null device\n");
		return -1;
	}

	spin_lock_irqsave(&smmulock, flags);
	dev->smmu_vcm_handler = handler;
	dev->smmu_vcm_handler_data = data;
	spin_unlock_irqrestore(&smmulock, flags);

	return 0;
}


/* Returns 0 on success. Will basically be 'activate'
 * Locking is to be handled by the caller (VCM)
 */
int smmu_activate(struct smmu_dev *dev)
{
	unsigned long flags;
	struct list_head *tmp = NULL;
	int ret;
	unsigned int cpu_prrr = 0;
	unsigned int cpu_nmrr = 0;

	spin_lock_irqsave(&smmulock, flags);

	if (!dev) {
		pr_err("null device\n");
		goto fail;
	}
	if (!dev->drv) {
		pr_err("null driver\n");
		goto fail;
	}

	/* Already active; do nothing */
	if (__smmu_is_active(dev)) {
		spin_unlock_irqrestore(&smmulock, flags);
		return 0;
	}

	/* See if someone else was previously active */
	list_for_each(tmp, &dev->drv->list_active) {
		struct smmu_dev *a_dev;
		a_dev = list_entry(tmp, struct smmu_dev, dev_elm);

		if (a_dev->context == dev->context) {
			/* Don't clean up since we will reinit anyway */
			list_del_init(&a_dev->dev_elm);
			break;
		}
	}

	ret = smmu_context_reset(dev->base, dev->context);
	if (ret != 0) {
		pr_err("failed to reset ctx %d on smmu at %p with error %d\n",
			 dev->context, (void *) dev->base, ret);
		goto fail;
	}

	/* Set up HTW mode */
	/* HTW on miss */
	SET_TLBMCFG(dev->base, dev->context, 0x3);

	/* HTW for V2P access */
	SET_V2PCFG(dev->base, dev->context, 0x3);


	SET_TTBCR(dev->base, dev->context, 0);
	SET_TTBR0_PA(dev->base, dev->context, (__pa(dev->fl_table) >> 14));

	mb();

	/* Invalidate the TLB for this context */
	SET_CTX_TLBIALL(dev->base, dev->context, 0);

	/* Set interrupt number to "secure" interrupt */
	SET_IRPTNDX(dev->base, dev->context, 0);

	/* Context interrupts enabled */
	SET_CFEIE(dev->base, dev->context, 1);

	SET_CFCFG(dev->base, dev->context, 1);	/* STALL on context fault */

	/* Redirect ALL cacheable requests to L2 slave port
	 * SH bit in PTE determines coherency. TEX class determines cache
	 * policy.
	 */
	SET_RCISH(dev->base, dev->context, 1);
	SET_RCOSH(dev->base, dev->context, 1);
	SET_RCNSH(dev->base, dev->context, 1);

	/* Turn on TEX Remap */
	SET_TRE(dev->base, dev->context, 1);

	/* Instead of manually configuring PRRR / NMRR, use the CPU's version.
	 * Register format matches between CPU and SMMU. If not, you have the
	 * old version of the ARM spec.
	 */
	RCP15_PRRR(cpu_prrr);
	RCP15_NMRR(cpu_nmrr);

	SET_PRRR(dev->base, dev->context, cpu_prrr);
	SET_NMRR(dev->base, dev->context, cpu_nmrr);

	/* Turn on BFB fetch */
	SET_BFBDFE(dev->base, dev->context, 1);

	/* Set PAGE TABLES as inner-shareable.. or not, for now */
#ifdef CONFIG_SMMU_PGTABLES_L2
	SET_TTBR0_SH(dev->base, dev->context, 1);
	SET_TTBR1_SH(dev->base, dev->context, 1);

	SET_TTBR0_NOS(dev->base, dev->context, 1);
	SET_TTBR1_NOS(dev->base, dev->context, 1);

	SET_TTBR0_IRGNH(dev->base, dev->context, 0); /* WB, WA */
	SET_TTBR0_IRGNL(dev->base, dev->context, 1);

	SET_TTBR1_IRGNH(dev->base, dev->context, 0); /* WB, WA */
	SET_TTBR1_IRGNL(dev->base, dev->context, 1);

	SET_TTBR0_ORGN(dev->base, dev->context, 0x01); /* WB, WA */
	SET_TTBR1_ORGN(dev->base, dev->context, 0x01); /* WB, WA */
#endif

	mb();

#ifndef CONFIG_SMMU_PGTABLES_L2
	v7_flush_kern_cache_all();
#endif

	SET_M(dev->base, dev->context, 1); 	/* Enable the MMU */

	mb();

	/* Record us as active */
	list_add(&(dev->dev_elm), &dev->drv->list_active);
	spin_unlock_irqrestore(&smmulock, flags);
	return 0;
fail:
	spin_unlock_irqrestore(&smmulock, flags);
	return -1;
}


int smmu_deactivate(struct smmu_dev *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&smmulock, flags);

	if (!dev) {
		pr_err("null device\n");
		goto fail;
	}

	/* Not active, cannot deactivate */
	if (!__smmu_is_active(dev)) {
		pr_err("device not active\n");
		spin_unlock_irqrestore(&smmulock, flags);
		return -1;
	}

	/* Turn off MMU first */
	SET_ACTLR(dev->base, dev->context, 0);
	mb();
	SET_CTX_TLBIALL(dev->base, dev->context, 0);

	if (smmu_context_reset(dev->base, dev->context) != 0) {
		pr_err("context reset failed, ctx=%d, addr=%p\n",
			 dev->context, (void *) dev->base);
		goto fail;
	}

	mb();

	/* Remove from active list */
	list_del_init(&dev->dev_elm);

	spin_unlock_irqrestore(&smmulock, flags);
	return 0;
fail:
	spin_unlock_irqrestore(&smmulock, flags);
	return -1;
}


int smmu_update_start(struct smmu_dev *dev)
{
	if (!dev) {
		pr_err("null device\n");
		goto fail;
	}
	return 0;
fail:
	return -1;
}


int smmu_update_done(struct smmu_dev *dev)
{
	if (!dev) {
		pr_err("null device\n");
		goto fail;
	}

#ifndef CONFIG_SMMU_PGTABLES_L2
	v7_flush_kern_cache_all();
#endif
	mb();
	if (__smmu_is_active(dev))
		SET_CTX_TLBIALL(dev->base, dev->context, 0);

	return 0;
fail:
	return -1;
}


int __smmu_map(struct smmu_dev *dev, unsigned long pa, unsigned long va,
	     unsigned long len, unsigned int attr)
{
	unsigned long *fl_table = NULL;
	unsigned long *fl_pte = NULL;
	unsigned long fl_offset = 0;
	unsigned long *sl_table = NULL;
	unsigned long *sl_pte = NULL;
	unsigned long sl_offset = 0;
	int ret;

	if (!dev) {
		pr_err("null device\n");
		goto fail;
	}
	/* Alignment */
	if ((pa & (len-1)) || ((va & (len-1)))) {
		pr_err("misaligned address. pa=%p, va=%p, len=%lu\n",
			 (void *) pa, (void *) va, len);
		goto fail;
	}
	if (len != SZ_16M && len != SZ_1M &&
	    len != SZ_64K && len != SZ_4K) {
		pr_err("bad size: %lu\n", len);
		goto fail;
	}

	fl_table = dev->fl_table;

	if (!fl_table) {
		pr_err("null page table\n");
		goto fail;
	}

	fl_offset = FL_OFFSET(va);	/* Upper 12 bits */
	fl_pte = fl_table + fl_offset;	/* int pointers, 4 bytes */

	if (len == SZ_16M) {
		int i = 0;
		for (i = 0; i < 16; i++)
			*(fl_pte+i) = (pa & 0xFF000000) | PMD_SECT_SUPER |
				PMD_SECT_AP_READ | PMD_SECT_AP_WRITE |
				PMD_TYPE_SECT | PMD_SECT_S;
	}

	if (len == SZ_1M)
		*fl_pte = (pa & 0xFFF00000) | PMD_SECT_AP_READ |
			PMD_SECT_AP_WRITE | PMD_TYPE_SECT |
			PMD_SECT_S;

	/* Need a 2nd level table */
	if ((len == SZ_4K || len == SZ_64K) && (*fl_pte) == 0) {
		unsigned long *sl;
		sl = (unsigned long *) __get_free_pages(GFP_KERNEL, 0);

		if (!sl) {
			pr_err("null second level table\n");
			goto fail;
		}

		memset(sl, 0, SZ_4K);
		*fl_pte = ((((int)__pa(sl)) & 0xFFFFFC00) | PMD_TYPE_TABLE);
	}

	sl_table = (unsigned long *) __va(((*fl_pte) & 0xFFFFFC00));
	sl_offset = SL_OFFSET(va);
	sl_pte = sl_table + sl_offset;


	if (len == SZ_4K)
		*sl_pte = (pa & 0xFFFFF000) | PTE_EXT_AP0 | PTE_EXT_AP1 |
			PTE_EXT_SHARED | PTE_TYPE_SMALL;

	if (len == SZ_64K) {
		int i;

		for (i = 0; i < 16; i++)
			*(sl_pte+i) = (pa & 0xFFFF0000) | PTE_EXT_AP0 |
				PTE_EXT_AP1 | PTE_EXT_SHARED | PTE_TYPE_LARGE;
	}

	ret = set_arm7_pte_attr((unsigned long) dev->fl_table, va, len, attr);

	if (ret) {
		pr_err("could not set attr %d: error %d, pgtable at %p, \
			  va=%p, len=%lu\n", attr, ret, (void *) dev->fl_table,
			 (void *) va, len);
		goto fail;
	}

#ifndef CONFIG_SMMU_PGTABLES_L2
	v7_flush_kern_cache_all();
#endif
	mb();

	return 0;
fail:
	return -1;
}


int smmu_map(struct smmu_dev *dev, unsigned long pa, unsigned long va,
	     unsigned long len, unsigned int attr)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&smmulock, flags);
	ret = __smmu_map(dev, pa, va, len, attr);
	spin_unlock_irqrestore(&smmulock, flags);
	return ret;
}


int smmu_unmap(struct smmu_dev *dev, unsigned long va, unsigned long len)
{
	unsigned long flags;
	unsigned long *fl_table = NULL;
	unsigned long *fl_pte = NULL;
	unsigned long fl_offset = 0;
	unsigned long *sl_table = NULL;
	unsigned long *sl_pte = NULL;
	unsigned long sl_offset = 0;
	int i;

	spin_lock_irqsave(&smmulock, flags);

	if (!dev) {
		pr_err("null device\n");
		goto fail;
	}

	/* Alignment */
	if (va & (len-1)) {
		pr_err("misaligned va: %p\n", (void *) va);
		goto fail;
	}

	if (len != SZ_16M && len != SZ_1M &&
	    len != SZ_64K && len != SZ_4K) {
		pr_err("bad length: %lu\n", len);
		goto fail;
	}

	fl_table = dev->fl_table;

	if (!fl_table) {
		pr_err("null page table\n");
		goto fail;
	}

	fl_offset = FL_OFFSET(va);	/* Upper 12 bits */
	fl_pte = fl_table + fl_offset;	/* int pointers, 4 bytes */

	if (*fl_pte == 0) {	/* Nothing there! */
		pr_err("first level PTE is 0\n");
		goto fail;
	}

	/* Unmap supersection */
	if (len == SZ_16M)
		for (i = 0; i < 16; i++)
			*(fl_pte+i) = 0;

	if (len == SZ_1M)
		*fl_pte = 0;

	sl_table = (unsigned long *) __va(((*fl_pte) & 0xFFFFFC00));
	sl_offset = SL_OFFSET(va);
	sl_pte = sl_table + sl_offset;

	if (len == SZ_64K) {
		for (i = 0; i < 16; i++)
			*(sl_pte+i) = 0;
	}

	if (len == SZ_4K)
		*sl_pte = 0;

	if (len == SZ_4K || len == SZ_64K) {
		int used = 0;

		for (i = 0; i < 256; i++)
			if (sl_table[i])
				used = 1;
		if (!used) {
			free_page((unsigned long)sl_table);
			*fl_pte = 0;
		}
	}

#ifndef CONFIG_SMMU_PGTABLES_L2
	v7_flush_kern_cache_all();
#endif
	mb();

	spin_unlock_irqrestore(&smmulock, flags);
	return 0;
fail:
	spin_unlock_irqrestore(&smmulock, flags);
	return 1;
}


/* Triggers interrupt and returns -1 on fault/error */
unsigned long smmu_translate(struct smmu_dev *dev, unsigned long va)
{
	unsigned long flags;
	unsigned long par = 0;

	spin_lock_irqsave(&smmulock, flags);

	if (!dev) {
		pr_err("null device\n");
		goto fail;
	}

	if (!dev->drv) {
		pr_err("null driver\n");
		goto fail;
	}

	/* Might collide with a different instance if we don't do this */
	if (!__smmu_is_active(dev)) {
		pr_err("device not active\n");
		goto fail;
	}

	/* Invalidate context TLB */
	SET_CTX_TLBIALL(dev->base, dev->context, 0);

	mb();
	SET_V2PPR_VA(dev->base, dev->context, va >> 12);
	mb();

	if (GET_FAULT(dev->base, dev->context)) {
		/* Trigger fault interrupt */
		SET_FSRRESTORE(dev->base, dev->context, 0x02);
		spin_unlock_irqrestore(&smmulock, flags);
		return 0;
	}

	mb();
	par = GET_PAR(dev->base, dev->context);
	mb();

	/* We are dealing with a supersection */
	if (GET_NOFAULT_SS(dev->base, dev->context)) {
		/* Upper 8 bits from PAR, lower 24 from VA */
		spin_unlock_irqrestore(&smmulock, flags);
		return (par & 0xFF000000) | (va & 0x00FFFFFF);
	}

	/* Upper 20 bits from PAR, lower 12 from VA */
	spin_unlock_irqrestore(&smmulock, flags);
	return (par & 0xFFFFF000) | (va & 0x00000FFF);
fail:
	spin_unlock_irqrestore(&smmulock, flags);
	return -1;
}
