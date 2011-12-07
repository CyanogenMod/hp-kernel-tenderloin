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
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/notifier.h>
#include <linux/pm_runtime.h>

#include <mach/msm_bus.h>
#include <linux/vmalloc.h>

#include "kgsl.h"
#include "kgsl_yamato.h"
#include "kgsl_log.h"
#include "kgsl_pm4types.h"
#include "kgsl_cmdstream.h"
#include "kgsl_postmortem.h"
#include "kgsl_cffdump.h"
#include "kgsl_drawctxt.h"

#include "yamato_reg.h"

#define DRIVER_VERSION_MAJOR   3
#define DRIVER_VERSION_MINOR   1

#define GSL_RBBM_INT_MASK \
	 (RBBM_INT_CNTL__RDERR_INT_MASK |  \
	  RBBM_INT_CNTL__DISPLAY_UPDATE_INT_MASK)

#define GSL_SQ_INT_MASK \
	(SQ_INT_CNTL__PS_WATCHDOG_MASK | \
	 SQ_INT_CNTL__VS_WATCHDOG_MASK)

/* Yamato MH arbiter config*/
#define KGSL_CFG_YAMATO_MHARB \
	(0x10 \
		| (0 << MH_ARBITER_CONFIG__SAME_PAGE_GRANULARITY__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__L1_ARB_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__L1_ARB_HOLD_ENABLE__SHIFT) \
		| (0 << MH_ARBITER_CONFIG__L2_ARB_CONTROL__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__PAGE_SIZE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__TC_REORDER_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__TC_ARB_HOLD_ENABLE__SHIFT) \
		| (0 << MH_ARBITER_CONFIG__IN_FLIGHT_LIMIT_ENABLE__SHIFT) \
		| (0x8 << MH_ARBITER_CONFIG__IN_FLIGHT_LIMIT__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__CP_CLNT_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__VGT_CLNT_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__TC_CLNT_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__RB_CLNT_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__PA_CLNT_ENABLE__SHIFT))

#define YAMATO_MMU_CONFIG						\
	(0x01								\
	 | (MMU_CONFIG << MH_MMU_CONFIG__RB_W_CLNT_BEHAVIOR__SHIFT)	\
	 | (MMU_CONFIG << MH_MMU_CONFIG__CP_W_CLNT_BEHAVIOR__SHIFT)	\
	 | (MMU_CONFIG << MH_MMU_CONFIG__CP_R0_CLNT_BEHAVIOR__SHIFT)	\
	 | (MMU_CONFIG << MH_MMU_CONFIG__CP_R1_CLNT_BEHAVIOR__SHIFT)	\
	 | (MMU_CONFIG << MH_MMU_CONFIG__CP_R2_CLNT_BEHAVIOR__SHIFT)	\
	 | (MMU_CONFIG << MH_MMU_CONFIG__CP_R3_CLNT_BEHAVIOR__SHIFT)	\
	 | (MMU_CONFIG << MH_MMU_CONFIG__CP_R4_CLNT_BEHAVIOR__SHIFT)	\
	 | (MMU_CONFIG << MH_MMU_CONFIG__VGT_R0_CLNT_BEHAVIOR__SHIFT)	\
	 | (MMU_CONFIG << MH_MMU_CONFIG__VGT_R1_CLNT_BEHAVIOR__SHIFT)	\
	 | (MMU_CONFIG << MH_MMU_CONFIG__TC_R_CLNT_BEHAVIOR__SHIFT)	\
	 | (MMU_CONFIG << MH_MMU_CONFIG__PA_W_CLNT_BEHAVIOR__SHIFT))

static struct kgsl_yamato_device yamato_device = {
	.dev = {
		.name = DEVICE_3D0_NAME,
		.id = KGSL_DEVICE_YAMATO,
		.ver_major = DRIVER_VERSION_MAJOR,
		.ver_minor = DRIVER_VERSION_MINOR,
		.mmu = {
			.config = YAMATO_MMU_CONFIG,
			/* turn off memory protection unit by setting
			   acceptable physical address range to include
			   all pages. */
			.mpu_base = 0x00000000,
			.mpu_range =  0xFFFFF000,
			.reg = {
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
				.interrupt_clear = REG_MH_INTERRUPT_CLEAR,
				.axi_error = REG_MH_AXI_ERROR,
			},
		},
		.pwrctrl = {
			.pwr_rail = PWR_RAIL_GRP_CLK,
			.regulator_name = "fs_gfx3d",
			.irq_name = KGSL_3D0_IRQ,
			.src_clk_name = "grp_src_clk",
		},
		.mutex = __MUTEX_INITIALIZER(yamato_device.dev.mutex),
		.state = KGSL_STATE_INIT,
		.active_cnt = 0,
		.iomemname = KGSL_3D0_REG_MEMORY,
		.display_off = {
#ifdef CONFIG_HAS_EARLYSUSPEND
			.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING,
			.suspend = kgsl_early_suspend_driver,
			.resume = kgsl_late_resume_driver,
#endif
		},
	},
	.gmemspace = {
		.gpu_base = 0,
		.sizebytes = SZ_256K,
	},
	.pfp_fw = NULL,
	.pm4_fw = NULL,
};

/* max msecs to wait for gpu to finish its operation(s) */
#define MAX_WAITGPU_SECS (HZ + HZ/2)


static int kgsl_yamato_start(struct kgsl_device *device,
						unsigned int init_ram);
static int kgsl_yamato_stop(struct kgsl_device *device);

static int kgsl_yamato_gmeminit(struct kgsl_yamato_device *yamato_device)
{
	struct kgsl_device *device = &yamato_device->dev;
	union reg_rb_edram_info rb_edram_info;
	unsigned int gmem_size;
	unsigned int edram_value = 0;

	/* make sure edram range is aligned to size */
	BUG_ON(yamato_device->gmemspace.gpu_base &
				(yamato_device->gmemspace.sizebytes - 1));

	/* get edram_size value equivalent */
	gmem_size = (yamato_device->gmemspace.sizebytes >> 14);
	while (gmem_size >>= 1)
		edram_value++;

	rb_edram_info.val = 0;

	rb_edram_info.f.edram_size = edram_value;
	if (device->chip_id != KGSL_CHIPID_LEIA_REV470)
		rb_edram_info.f.edram_mapping_mode = 0; /* EDRAM_MAP_UPPER */

	/* must be aligned to size */
	rb_edram_info.f.edram_range = (yamato_device->gmemspace.gpu_base >> 14);

	kgsl_yamato_regwrite(device, REG_RB_EDRAM_INFO, rb_edram_info.val);

	return 0;
}

static int kgsl_yamato_gmemclose(struct kgsl_device *device)
{
	kgsl_yamato_regwrite(device, REG_RB_EDRAM_INFO, 0x00000000);

	return 0;
}

static void kgsl_yamato_rbbm_intrcallback(struct kgsl_device *device)
{
	unsigned int status = 0;
	unsigned int rderr = 0;

	kgsl_yamato_regread_isr(device, REG_RBBM_INT_STATUS, &status);

	if (status & RBBM_INT_CNTL__RDERR_INT_MASK) {
		union rbbm_read_error_u rerr;
		kgsl_yamato_regread_isr(device, REG_RBBM_READ_ERROR, &rderr);
		rerr.val = rderr;
		if (rerr.f.read_address == REG_CP_INT_STATUS &&
			rerr.f.read_error &&
			rerr.f.read_requester)
			KGSL_DRV_WARN(device,
				"rbbm read error interrupt: %08x\n", rderr);
		else
			KGSL_DRV_CRIT(device,
				"rbbm read error interrupt: %08x\n", rderr);
	} else if (status & RBBM_INT_CNTL__DISPLAY_UPDATE_INT_MASK) {
		KGSL_DRV_INFO(device, "rbbm display update interrupt\n");
	} else if (status & RBBM_INT_CNTL__GUI_IDLE_INT_MASK) {
		KGSL_DRV_INFO(device, "rbbm gui idle interrupt\n");
	} else {
		KGSL_CMD_WARN(device,
			"bad bits in REG_CP_INT_STATUS %08x\n", status);
	}

	status &= GSL_RBBM_INT_MASK;
	kgsl_yamato_regwrite_isr(device, REG_RBBM_INT_ACK, status);
}

static void kgsl_yamato_sq_intrcallback(struct kgsl_device *device)
{
	unsigned int status = 0;

	kgsl_yamato_regread_isr(device, REG_SQ_INT_STATUS, &status);

	if (status & SQ_INT_CNTL__PS_WATCHDOG_MASK)
		KGSL_DRV_INFO(device, "sq ps watchdog interrupt\n");
	else if (status & SQ_INT_CNTL__VS_WATCHDOG_MASK)
		KGSL_DRV_INFO(device, "sq vs watchdog interrupt\n");
	else
		KGSL_DRV_WARN(device,
			"bad bits in REG_SQ_INT_STATUS %08x\n", status);


	status &= GSL_SQ_INT_MASK;
	kgsl_yamato_regwrite_isr(device, REG_SQ_INT_ACK, status);
}

irqreturn_t kgsl_yamato_isr(int irq, void *data)
{
	irqreturn_t result = IRQ_NONE;
	struct kgsl_device *device;
	unsigned int status;

	device = (struct kgsl_device *) data;

	BUG_ON(device == NULL);
	BUG_ON(device->regspace.sizebytes == 0);
	BUG_ON(device->regspace.mmio_virt_base == 0);

	kgsl_yamato_regread_isr(device, REG_MASTER_INT_SIGNAL, &status);

	if (status & MASTER_INT_SIGNAL__MH_INT_STAT) {
		kgsl_mh_intrcallback(device);
		result = IRQ_HANDLED;
	}

	if (status & MASTER_INT_SIGNAL__CP_INT_STAT) {
		kgsl_cp_intrcallback(device);
		result = IRQ_HANDLED;
	}

	if (status & MASTER_INT_SIGNAL__RBBM_INT_STAT) {
		kgsl_yamato_rbbm_intrcallback(device);
		result = IRQ_HANDLED;
	}

	if (status & MASTER_INT_SIGNAL__SQ_INT_STAT) {
		kgsl_yamato_sq_intrcallback(device);
		result = IRQ_HANDLED;
	}

	if (device->requested_state == KGSL_STATE_NONE) {
		if (device->pwrctrl.nap_allowed == true) {
			device->requested_state = KGSL_STATE_NAP;
			queue_work(device->work_queue, &device->idle_check_ws);
		} else if (device->pwrctrl.idle_pass == true) {
			queue_work(device->work_queue, &device->idle_check_ws);
		}
	}

	/* Reset the time-out in our idle timer */
	mod_timer(&device->idle_timer,
		jiffies + device->pwrctrl.interval_timeout);
	return result;
}

static int kgsl_yamato_cleanup_pt(struct kgsl_device *device,
			struct kgsl_pagetable *pagetable)
{
	struct kgsl_yamato_device *yamato_device = KGSL_YAMATO_DEVICE(device);
	struct kgsl_ringbuffer *rb = &yamato_device->ringbuffer;

	kgsl_mmu_unmap(pagetable, rb->buffer_desc.gpuaddr,
			rb->buffer_desc.size);

	kgsl_mmu_unmap(pagetable, rb->memptrs_desc.gpuaddr,
			rb->memptrs_desc.size);

	kgsl_mmu_unmap(pagetable, device->memstore.gpuaddr,
			device->memstore.size);

	kgsl_mmu_unmap(pagetable, device->mmu.dummyspace.gpuaddr,
			device->mmu.dummyspace.size);

	return 0;
}

static int kgsl_yamato_setup_pt(struct kgsl_device *device,
			struct kgsl_pagetable *pagetable)
{
	int result = 0;
	unsigned int flags = KGSL_MEMFLAGS_CONPHYS | KGSL_MEMFLAGS_ALIGN4K;
	struct kgsl_yamato_device *yamato_device = KGSL_YAMATO_DEVICE(device);
	struct kgsl_ringbuffer *rb = &yamato_device->ringbuffer;

	BUG_ON(rb->buffer_desc.physaddr == 0);
	BUG_ON(rb->memptrs_desc.physaddr == 0);
	BUG_ON(device->memstore.physaddr == 0);
#ifdef CONFIG_MSM_KGSL_MMU
	BUG_ON(device->mmu.dummyspace.physaddr == 0);
#endif
	result = kgsl_mmu_map_global(pagetable, &rb->buffer_desc,
				     GSL_PT_PAGE_RV, flags);
	if (result)
		goto error;

	result = kgsl_mmu_map_global(pagetable, &rb->memptrs_desc,
				     GSL_PT_PAGE_RV | GSL_PT_PAGE_WV, flags);
	if (result)
		goto unmap_buffer_desc;

	result = kgsl_mmu_map_global(pagetable, &device->memstore,
				     GSL_PT_PAGE_RV | GSL_PT_PAGE_WV, flags);
	if (result)
		goto unmap_memptrs_desc;

	result = kgsl_mmu_map_global(pagetable, &device->mmu.dummyspace,
				     GSL_PT_PAGE_RV | GSL_PT_PAGE_WV, flags);
	if (result)
		goto unmap_memstore_desc;

	return result;

unmap_memstore_desc:
	kgsl_mmu_unmap(pagetable, device->memstore.gpuaddr,
			device->memstore.size);

unmap_memptrs_desc:
	kgsl_mmu_unmap(pagetable, rb->memptrs_desc.gpuaddr,
			rb->memptrs_desc.size);
unmap_buffer_desc:
	kgsl_mmu_unmap(pagetable, rb->buffer_desc.gpuaddr,
			rb->buffer_desc.size);
error:
	return result;
}

static int kgsl_yamato_setstate(struct kgsl_device *device, uint32_t flags)
{
	struct kgsl_yamato_device *yamato_device = KGSL_YAMATO_DEVICE(device);
	unsigned int link[32];
	unsigned int *cmds = &link[0];
	int sizedwords = 0;
	unsigned int mh_mmu_invalidate = 0x00000003; /*invalidate all and tc */

#ifndef CONFIG_MSM_KGSL_MMU
	return 0;
#endif
	/* if possible, set via command stream,
	* otherwise set via direct register writes
	*/
	if (yamato_device->drawctxt_active) {
		if (flags & KGSL_MMUFLAGS_PTUPDATE) {
			/* wait for graphics pipe to be idle */
			*cmds++ = pm4_type3_packet(PM4_WAIT_FOR_IDLE, 1);
			*cmds++ = 0x00000000;

			/* set page table base */
			*cmds++ = pm4_type0_packet(REG_MH_MMU_PT_BASE, 1);
			*cmds++ = device->mmu.hwpagetable->base.gpuaddr;
			sizedwords += 4;
		}

		if (flags & KGSL_MMUFLAGS_TLBFLUSH) {
			if (!(flags & KGSL_MMUFLAGS_PTUPDATE)) {
				*cmds++ = pm4_type3_packet(PM4_WAIT_FOR_IDLE,
								1);
				*cmds++ = 0x00000000;
				sizedwords += 2;
			}
			*cmds++ = pm4_type0_packet(REG_MH_MMU_INVALIDATE, 1);
			*cmds++ = mh_mmu_invalidate;
			sizedwords += 2;
		}

		if (flags & KGSL_MMUFLAGS_PTUPDATE &&
			device->chip_id != KGSL_CHIPID_LEIA_REV470) {
			/* HW workaround: to resolve MMU page fault interrupts
			* caused by the VGT.It prevents the CP PFP from filling
			* the VGT DMA request fifo too early,thereby ensuring
			* that the VGT will not fetch vertex/bin data until
			* after the page table base register has been updated.
			*
			* Two null DRAW_INDX_BIN packets are inserted right
			* after the page table base update, followed by a
			* wait for idle. The null packets will fill up the
			* VGT DMA request fifo and prevent any further
			* vertex/bin updates from occurring until the wait
			* has finished. */
			*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
			*cmds++ = (0x4 << 16) |
				(REG_PA_SU_SC_MODE_CNTL - 0x2000);
			*cmds++ = 0;	  /* disable faceness generation */
			*cmds++ = pm4_type3_packet(PM4_SET_BIN_BASE_OFFSET, 1);
			*cmds++ = device->mmu.dummyspace.gpuaddr;
			*cmds++ = pm4_type3_packet(PM4_DRAW_INDX_BIN, 6);
			*cmds++ = 0;	  /* viz query info */
			*cmds++ = 0x0003C004; /* draw indicator */
			*cmds++ = 0;	  /* bin base */
			*cmds++ = 3;	  /* bin size */
			*cmds++ = device->mmu.dummyspace.gpuaddr; /* dma base */
			*cmds++ = 6;	  /* dma size */
			*cmds++ = pm4_type3_packet(PM4_DRAW_INDX_BIN, 6);
			*cmds++ = 0;	  /* viz query info */
			*cmds++ = 0x0003C004; /* draw indicator */
			*cmds++ = 0;	  /* bin base */
			*cmds++ = 3;	  /* bin size */
			/* dma base */
			*cmds++ = device->mmu.dummyspace.gpuaddr;
			*cmds++ = 6;	  /* dma size */
			*cmds++ = pm4_type3_packet(PM4_WAIT_FOR_IDLE, 1);
			*cmds++ = 0x00000000;
			sizedwords += 21;
		}

		if (flags & (KGSL_MMUFLAGS_PTUPDATE | KGSL_MMUFLAGS_TLBFLUSH)) {
			*cmds++ = pm4_type3_packet(PM4_INVALIDATE_STATE, 1);
			*cmds++ = 0x7fff; /* invalidate all base pointers */
			sizedwords += 2;
		}

		kgsl_ringbuffer_issuecmds(device, KGSL_CMD_FLAGS_PMODE,
					&link[0], sizedwords);
	} else {
		if (flags & KGSL_MMUFLAGS_PTUPDATE) {
			kgsl_yamato_idle(device, KGSL_TIMEOUT_DEFAULT);
			kgsl_yamato_regwrite(device, REG_MH_MMU_PT_BASE,
				     device->mmu.hwpagetable->base.gpuaddr);
		}

		if (flags & KGSL_MMUFLAGS_TLBFLUSH) {
			kgsl_yamato_regwrite(device, REG_MH_MMU_INVALIDATE,
					   mh_mmu_invalidate);
		}
	}

	return 0;
}

static unsigned int
kgsl_yamato_getchipid(struct kgsl_device *device)
{
	unsigned int chipid;
	unsigned int coreid, majorid, minorid, patchid, revid;

	/* YDX */
	kgsl_yamato_regread(device, REG_RBBM_PERIPHID1, &coreid);
	coreid &= 0xF;

	kgsl_yamato_regread(device, REG_RBBM_PERIPHID2, &majorid);
	majorid = (majorid >> 4) & 0xF;

	kgsl_yamato_regread(device, REG_RBBM_PATCH_RELEASE, &revid);
	/* this is a 16bit field, but extremely unlikely it would ever get
	* this high
	*/
	minorid = ((revid >> 0)  & 0xFF);


	patchid = ((revid >> 16) & 0xFF);

	chipid  = ((coreid << 24) | (majorid << 16) |
			(minorid << 8) | (patchid << 0));

	/* Hardware revision 211 (8650) returns the wrong chip ID */
	if (chipid == KGSL_CHIPID_YAMATODX_REV21)
		chipid = KGSL_CHIPID_YAMATODX_REV211;

	/* Workaround Hardware revision issue of Z470 */
	if (chipid == KGSL_CHIPID_LEIA_REV470_TEMP)
		chipid = KGSL_CHIPID_LEIA_REV470;


	return chipid;
}

static void __devinit kgsl_yamato_getfunctable(struct kgsl_functable *ftbl);

static int __devinit
kgsl_3d_probe(struct platform_device *pdev)
{
	struct kgsl_device *device;
	int status = -EINVAL;

	device = (struct kgsl_device *)pdev->id_entry->driver_data;
	device->pdev = pdev;

	init_completion(&device->recovery_gate);

	kgsl_yamato_getfunctable(&device->ftbl);

	status = kgsl_ringbuffer_init(device);
	if (status != 0)
		goto error;

	status = kgsl_device_probe(device, kgsl_yamato_isr);
	if (status)
		goto error_close_rb;

	kgsl_postmortem_init(device);
	kgsl_yamato_debugfs_init(device);

	device->flags &= ~KGSL_FLAGS_SOFT_RESET;
	return 0;

error_close_rb:
	kgsl_ringbuffer_close(&yamato_device.ringbuffer);
error:
	device->pdev = NULL;
	return status;
}

static int __devexit kgsl_3d_remove(struct platform_device *pdev)
{
	struct kgsl_device *device = NULL;
	struct kgsl_yamato_device *device_3d = NULL;

	device = (struct kgsl_device *)pdev->id_entry->driver_data;
	device_3d = KGSL_YAMATO_DEVICE(device);

	kgsl_device_remove(device);

	kgsl_ringbuffer_close(&device_3d->ringbuffer);

	return 0;
}

static int kgsl_yamato_start(struct kgsl_device *device, unsigned int init_ram)
{
	int status = -EINVAL;
	struct kgsl_yamato_device *yamato_device = KGSL_YAMATO_DEVICE(device);
	int init_reftimestamp = 0x7fffffff;

	device->state = KGSL_STATE_INIT;
	device->requested_state = KGSL_STATE_NONE;
	/* Order pwrrail/clk sequence based upon platform. */
	if (device->pwrctrl.pwrrail_first)
		kgsl_pwrctrl_pwrrail(device, KGSL_PWRFLAGS_POWER_ON);
	kgsl_pwrctrl_clk(device, KGSL_PWRFLAGS_CLK_ON);
	kgsl_pwrctrl_axi(device, KGSL_PWRFLAGS_AXI_ON);
	if (!device->pwrctrl.pwrrail_first)
		kgsl_pwrctrl_pwrrail(device, KGSL_PWRFLAGS_POWER_ON);

	if (kgsl_mmu_start(device))
		goto error_clk_off;

	/*We need to make sure all blocks are powered up and clocked before
	*issuing a soft reset.  The overrides will then be turned off (set to 0)
	*/
	kgsl_yamato_regwrite(device, REG_RBBM_PM_OVERRIDE1, 0xfffffffe);
	device->chip_id = kgsl_yamato_getchipid(device);

	if (device->chip_id == CHIP_REV_251)
		kgsl_yamato_regwrite(device, REG_RBBM_PM_OVERRIDE2, 0x000000ff);
	else
		kgsl_yamato_regwrite(device, REG_RBBM_PM_OVERRIDE2, 0xffffffff);

	/* Only reset CP block if all blocks have previously been reset */
	if (!(device->flags & KGSL_FLAGS_SOFT_RESET) ||
		(device->chip_id != KGSL_CHIPID_LEIA_REV470)) {
		kgsl_yamato_regwrite(device, REG_RBBM_SOFT_RESET, 0xFFFFFFFF);
		device->flags |= KGSL_FLAGS_SOFT_RESET;
	} else
		kgsl_yamato_regwrite(device, REG_RBBM_SOFT_RESET, 0x00000001);

	/* The core is in an indeterminate state until the reset completes
	 * after 30ms.
	 */
	msleep(30);

	kgsl_yamato_regwrite(device, REG_RBBM_SOFT_RESET, 0x00000000);

	kgsl_yamato_regwrite(device, REG_RBBM_CNTL, 0x00004442);

	kgsl_yamato_regwrite(device, REG_MH_ARBITER_CONFIG,
				KGSL_CFG_YAMATO_MHARB);

	if (device->chip_id != KGSL_CHIPID_LEIA_REV470) {
		kgsl_yamato_regwrite(device,
			 REG_MH_CLNT_INTF_CTRL_CONFIG1, 0x00030f27);
		kgsl_yamato_regwrite(device,
			 REG_MH_CLNT_INTF_CTRL_CONFIG2, 0x00472747);
	}

	/* Remove 1k boundary check in z470 to avoid GPU hang.
	   Notice that, this solution won't work if both EBI and SMI are used */
	if (device->chip_id == KGSL_CHIPID_LEIA_REV470) {
		kgsl_yamato_regwrite(device, REG_MH_CLNT_INTF_CTRL_CONFIG1,
				 0x00032f07);
	}

	kgsl_yamato_regwrite(device, REG_SQ_VS_PROGRAM, 0x00000000);
	kgsl_yamato_regwrite(device, REG_SQ_PS_PROGRAM, 0x00000000);

	kgsl_yamato_regwrite(device, REG_RBBM_PM_OVERRIDE1, 0);
	if (device->chip_id != KGSL_CHIPID_LEIA_REV470)
		kgsl_yamato_regwrite(device, REG_RBBM_PM_OVERRIDE2, 0);
	else
		kgsl_yamato_regwrite(device, REG_RBBM_PM_OVERRIDE2, 0x80);

	kgsl_sharedmem_writel(&device->memstore,
			      KGSL_DEVICE_MEMSTORE_OFFSET(ref_wait_ts),
			      init_reftimestamp);

	kgsl_yamato_regwrite(device, REG_RBBM_DEBUG, 0x00080000);

	kgsl_yamato_regwrite(device, REG_RBBM_INT_CNTL, GSL_RBBM_INT_MASK);

	/* make sure SQ interrupts are disabled */
	kgsl_yamato_regwrite(device, REG_SQ_INT_CNTL, 0);

	if (device->chip_id == KGSL_CHIPID_LEIA_REV470)
		yamato_device->gmemspace.sizebytes = SZ_512K;
	else
		yamato_device->gmemspace.sizebytes = SZ_256K;
	kgsl_yamato_gmeminit(yamato_device);

	kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_IRQ_ON);

	status = kgsl_ringbuffer_start(&yamato_device->ringbuffer, init_ram);
	if (status != 0)
		goto error_irq_off;

	mod_timer(&device->idle_timer, jiffies + FIRST_TIMEOUT);
#ifdef CONFIG_KGSL_PER_PROCESS_PAGE_TABLE
	pr_info("kgsl: initialized dev=%d mmu=%s "
		"per_process_pagetable=on\n",
		device->id, kgsl_mmu_isenabled(&device->mmu) ? "on" : "off");
#else
	pr_info("kgsl: initialized dev=%d mmu=%s "
		"per_process_pagetable=off\n",
		device->id, kgsl_mmu_isenabled(&device->mmu) ? "on" : "off");
#endif
	return status;

error_irq_off:
	kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_IRQ_OFF);
error_clk_off:
	kgsl_pwrctrl_axi(device, KGSL_PWRFLAGS_AXI_OFF);
	kgsl_pwrctrl_clk(device, KGSL_PWRFLAGS_CLK_OFF);

	kgsl_mmu_stop(device);
	return status;
}

static int kgsl_yamato_stop(struct kgsl_device *device)
{
	struct kgsl_yamato_device *yamato_device = KGSL_YAMATO_DEVICE(device);
	del_timer(&device->idle_timer);
	kgsl_yamato_regwrite(device, REG_RBBM_INT_CNTL, 0);

	kgsl_yamato_regwrite(device, REG_SQ_INT_CNTL, 0);

	yamato_device->drawctxt_active = NULL;

	kgsl_ringbuffer_stop(&yamato_device->ringbuffer);

	kgsl_yamato_gmemclose(device);

	kgsl_mmu_stop(device);

	kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_IRQ_OFF);
	if (!device->pwrctrl.pwrrail_first)
		kgsl_pwrctrl_pwrrail(device, KGSL_PWRFLAGS_POWER_OFF);
	kgsl_pwrctrl_axi(device, KGSL_PWRFLAGS_AXI_OFF);
	kgsl_pwrctrl_clk(device, KGSL_PWRFLAGS_CLK_OFF);
	if (device->pwrctrl.pwrrail_first)
		kgsl_pwrctrl_pwrrail(device, KGSL_PWRFLAGS_POWER_OFF);

	return 0;
}

static int
kgsl_yamato_recover_hang(struct kgsl_device *device)
{
	int ret;
	unsigned int *rb_buffer;
	struct kgsl_yamato_device *yamato_device =
			(struct kgsl_yamato_device *)device;
	struct kgsl_ringbuffer *rb = &yamato_device->ringbuffer;
	unsigned int timestamp;
	unsigned int num_rb_contents;
	unsigned int bad_context;
	unsigned int reftimestamp;
	unsigned int enable_ts;
	unsigned int soptimestamp;
	unsigned int eoptimestamp;
	struct kgsl_yamato_context *drawctxt;

	KGSL_DRV_ERR(device, "Starting recovery from 3D GPU hang....\n");
	rb_buffer = vmalloc(rb->buffer_desc.size);
	if (!rb_buffer) {
		KGSL_MEM_ERR(device,
			"Failed to allocate memory for recovery: %x\n",
			rb->buffer_desc.size);
		return -ENOMEM;
	}
	/* Extract valid contents from rb which can stil be executed after
	 * hang */
	ret = kgsl_ringbuffer_extract(rb, rb_buffer, &num_rb_contents);
	if (ret)
		goto done;
	timestamp = rb->timestamp;
	KGSL_DRV_ERR(device, "Last issued timestamp: %x\n", timestamp);
	kgsl_sharedmem_readl(&device->memstore, &bad_context,
				KGSL_DEVICE_MEMSTORE_OFFSET(current_context));
	kgsl_sharedmem_readl(&device->memstore, &reftimestamp,
				KGSL_DEVICE_MEMSTORE_OFFSET(ref_wait_ts));
	kgsl_sharedmem_readl(&device->memstore, &enable_ts,
				KGSL_DEVICE_MEMSTORE_OFFSET(ts_cmp_enable));
	kgsl_sharedmem_readl(&device->memstore, &soptimestamp,
				KGSL_DEVICE_MEMSTORE_OFFSET(soptimestamp));
	kgsl_sharedmem_readl(&device->memstore, &eoptimestamp,
				KGSL_DEVICE_MEMSTORE_OFFSET(eoptimestamp));
	rmb();
	KGSL_CTXT_ERR(device,
		"Context that caused a GPU hang: %x\n", bad_context);
	/* restart device */
	ret = kgsl_yamato_stop(device);
	if (ret)
		goto done;
	ret = kgsl_yamato_start(device, true);
	if (ret)
		goto done;
	KGSL_DRV_ERR(device, "Device has been restarted after hang\n");
	/* Restore timestamp states */
	kgsl_sharedmem_writel(&device->memstore,
			KGSL_DEVICE_MEMSTORE_OFFSET(soptimestamp),
			soptimestamp);
	kgsl_sharedmem_writel(&device->memstore,
			KGSL_DEVICE_MEMSTORE_OFFSET(eoptimestamp),
			eoptimestamp);
	kgsl_sharedmem_writel(&device->memstore,
			KGSL_DEVICE_MEMSTORE_OFFSET(soptimestamp),
			soptimestamp);
	if (num_rb_contents) {
		kgsl_sharedmem_writel(&device->memstore,
			KGSL_DEVICE_MEMSTORE_OFFSET(ref_wait_ts),
			reftimestamp);
		kgsl_sharedmem_writel(&device->memstore,
			KGSL_DEVICE_MEMSTORE_OFFSET(ts_cmp_enable),
			enable_ts);
	}
	wmb();
	/* Mark the invalid context so no more commands are accepted from
	 * that context */

	drawctxt = (struct kgsl_yamato_context *) bad_context;

	KGSL_CTXT_ERR(device,
		"Context that caused a GPU hang: %x\n", bad_context);

	drawctxt->flags |= CTXT_FLAGS_GPU_HANG;

	/* Restore valid commands in ringbuffer */
	kgsl_ringbuffer_restore(rb, rb_buffer, num_rb_contents);
	rb->timestamp = timestamp;
done:
	vfree(rb_buffer);
	return ret;
}

static int
kgsl_yamato_dump_and_recover(struct kgsl_device *device)
{
	static int recovery;
	int result = -ETIMEDOUT;

	if (device->state == KGSL_STATE_HUNG)
		goto done;
	if (device->state == KGSL_STATE_DUMP_AND_RECOVER && !recovery) {
		mutex_unlock(&device->mutex);
		wait_for_completion(&device->recovery_gate);
		mutex_lock(&device->mutex);
		if (!(device->state & KGSL_STATE_HUNG))
			/* recovery success */
			result = 0;
	} else {
		INIT_COMPLETION(device->recovery_gate);
		/* Detected a hang - trigger an automatic dump */
		kgsl_postmortem_dump(device, 0);
		if (!recovery) {
			recovery = 1;
			result = kgsl_yamato_recover_hang(device);
			if (result)
				device->state = KGSL_STATE_HUNG;
			recovery = 0;
			complete_all(&device->recovery_gate);
		} else
			KGSL_DRV_ERR(device,
				"Cannot recover from another hang while "
				"recovering from a hang\n");
	}
done:
	return result;
}

struct kgsl_device *kgsl_get_yamato_generic_device(void)
{
	return &yamato_device.dev;
}

static int kgsl_yamato_getproperty(struct kgsl_device *device,
				enum kgsl_property_type type,
				void *value,
				unsigned int sizebytes)
{
	int status = -EINVAL;
	struct kgsl_yamato_device *yamato_device = KGSL_YAMATO_DEVICE(device);

	switch (type) {
	case KGSL_PROP_DEVICE_INFO:
		{
			struct kgsl_devinfo devinfo;

			if (sizebytes != sizeof(devinfo)) {
				status = -EINVAL;
				break;
			}

			memset(&devinfo, 0, sizeof(devinfo));
			devinfo.device_id = device->id+1;
			devinfo.chip_id = device->chip_id;
			devinfo.mmu_enabled = kgsl_mmu_isenabled(&device->mmu);
			devinfo.gmem_hostbaseaddr = (unsigned int)
					yamato_device->gmemspace.mmio_virt_base;
			devinfo.gmem_gpubaseaddr = yamato_device->gmemspace.
					gpu_base;
			devinfo.gmem_sizebytes = yamato_device->gmemspace.
					sizebytes;

			if (copy_to_user(value, &devinfo, sizeof(devinfo)) !=
					0) {
				status = -EFAULT;
				break;
			}
			status = 0;
		}
		break;
	case KGSL_PROP_DEVICE_SHADOW:
		{
			struct kgsl_shadowprop shadowprop;

			if (sizebytes != sizeof(shadowprop)) {
				status = -EINVAL;
				break;
			}
			memset(&shadowprop, 0, sizeof(shadowprop));
			if (device->memstore.hostptr) {
				/*NOTE: with mmu enabled, gpuaddr doesn't mean
				 * anything to mmap().
				 */
				shadowprop.gpuaddr = device->memstore.physaddr;
				shadowprop.size = device->memstore.size;
				/* GSL needs this to be set, even if it
				   appears to be meaningless */
				shadowprop.flags = KGSL_FLAGS_INITIALIZED;
			}
			if (copy_to_user(value, &shadowprop,
				sizeof(shadowprop))) {
				status = -EFAULT;
				break;
			}
			status = 0;
		}
		break;
	case KGSL_PROP_MMU_ENABLE:
		{
#ifdef CONFIG_MSM_KGSL_MMU
			int mmuProp = 1;
#else
			int mmuProp = 0;
#endif
			if (sizebytes != sizeof(int)) {
				status = -EINVAL;
				break;
			}
			if (copy_to_user(value, &mmuProp, sizeof(mmuProp))) {
				status = -EFAULT;
				break;
			}
			status = 0;
		}
		break;
	case KGSL_PROP_INTERRUPT_WAITS:
		{
			int int_waits = 1;
			if (sizebytes != sizeof(int)) {
				status = -EINVAL;
				break;
			}
			if (copy_to_user(value, &int_waits, sizeof(int))) {
				status = -EFAULT;
				break;
			}
			status = 0;
		}
		break;
	default:
		status = -EINVAL;
	}

	return status;
}

/* Caller must hold the device mutex. */
int kgsl_yamato_idle(struct kgsl_device *device, unsigned int timeout)
{
	struct kgsl_yamato_device *yamato_device = KGSL_YAMATO_DEVICE(device);
	struct kgsl_ringbuffer *rb = &yamato_device->ringbuffer;
	unsigned int rbbm_status;
	unsigned long wait_time = jiffies + MAX_WAITGPU_SECS;

	kgsl_cffdump_regpoll(device->id, REG_RBBM_STATUS << 2,
		0x00000000, 0x80000000);
	/* first, wait until the CP has consumed all the commands in
	 * the ring buffer
	 */
retry:
	if (rb->flags & KGSL_FLAGS_STARTED) {
		do {
			GSL_RB_GET_READPTR(rb, &rb->rptr);
			if (time_after(jiffies, wait_time)) {
				KGSL_DRV_ERR(device, "rptr: %x, wptr: %x\n",
					rb->rptr, rb->wptr);
				goto err;
			}
		} while (rb->rptr != rb->wptr);
	}

	/* now, wait for the GPU to finish its operations */
	wait_time = jiffies + MAX_WAITGPU_SECS;
	while (time_before(jiffies, wait_time)) {
		kgsl_yamato_regread(device, REG_RBBM_STATUS, &rbbm_status);
		if (rbbm_status == 0x110)
			return 0;
	}

err:
	KGSL_DRV_ERR(device, "spun too long waiting for RB to idle\n");
	if (!kgsl_yamato_dump_and_recover(device)) {
		wait_time = jiffies + MAX_WAITGPU_SECS;
		goto retry;
	}
	return -ETIMEDOUT;
}

static unsigned int kgsl_yamato_isidle(struct kgsl_device *device)
{
	int status = false;
	struct kgsl_yamato_device *yamato_device = KGSL_YAMATO_DEVICE(device);
	struct kgsl_ringbuffer *rb = &yamato_device->ringbuffer;
	unsigned int rbbm_status;

	if (rb->flags & KGSL_FLAGS_STARTED) {
		/* Is the ring buffer is empty? */
		GSL_RB_GET_READPTR(rb, &rb->rptr);
		if (!device->active_cnt && (rb->rptr == rb->wptr)) {
			/* Is the core idle? */
			kgsl_yamato_regread(device, REG_RBBM_STATUS,
					    &rbbm_status);
			if (rbbm_status == 0x110)
				status = true;
		}
	} else {
		KGSL_DRV_ERR(device, "ringbuffer not started\n");
		BUG();
	}
	return status;
}


/******************************************************************/
/* Caller must hold the driver mutex. */
static int kgsl_yamato_resume_context(struct kgsl_device *device)
{
	int status = 0;
	struct kgsl_yamato_device *yamato_device = KGSL_YAMATO_DEVICE(device);

	if (device->pwrctrl.suspended_ctxt != NULL) {
		kgsl_drawctxt_switch(yamato_device,
				     device->pwrctrl.suspended_ctxt, 0);
		status = kgsl_yamato_idle(device, 0);

	}

	return status;
}

/******************************************************************/
/* Caller must hold the device mutex. */
static int kgsl_yamato_suspend_context(struct kgsl_device *device)
{
	int status = 0;
	struct kgsl_yamato_device *yamato_device = KGSL_YAMATO_DEVICE(device);

	/* save ctxt ptr and switch to NULL ctxt */
	device->pwrctrl.suspended_ctxt = yamato_device->drawctxt_active;
	if (device->pwrctrl.suspended_ctxt != NULL) {
		kgsl_drawctxt_switch(yamato_device, NULL, 0);
		status = kgsl_yamato_idle(device, KGSL_TIMEOUT_DEFAULT);
	}

	return status;
}
static void _yamato_regread(struct kgsl_device *device,
			    unsigned int offsetwords,
			    unsigned int *value)
{
	unsigned int *reg;
	BUG_ON(offsetwords*sizeof(uint32_t) >= device->regspace.sizebytes);
	reg = (unsigned int *)(device->regspace.mmio_virt_base
				+ (offsetwords << 2));
	*value = readl(reg);
}

void kgsl_yamato_regread(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int *value)
{
	kgsl_pre_hwaccess(device);
	_yamato_regread(device, offsetwords, value);
}

void kgsl_yamato_regread_isr(struct kgsl_device *device,
			     unsigned int offsetwords,
			     unsigned int *value)
{
	_yamato_regread(device, offsetwords, value);
}

static void _yamato_regwrite(struct kgsl_device *device,
			     unsigned int offsetwords,
			     unsigned int value)
{
	unsigned int *reg;

	BUG_ON(offsetwords*sizeof(uint32_t) >= device->regspace.sizebytes);

	kgsl_cffdump_regwrite(device->id, offsetwords << 2, value);
	reg = (unsigned int *)(device->regspace.mmio_virt_base
				+ (offsetwords << 2));

	writel(value, reg);

}

void kgsl_yamato_regwrite(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int value)
{
	kgsl_pre_hwaccess(device);
	_yamato_regwrite(device, offsetwords, value);
}

void kgsl_yamato_regwrite_isr(struct kgsl_device *device,
			      unsigned int offsetwords,
			      unsigned int value)
{
	_yamato_regwrite(device, offsetwords, value);
}

static int kgsl_check_interrupt_timestamp(struct kgsl_device *device,
					unsigned int timestamp)
{
	int status;
	unsigned int ref_ts, enableflag;

	status = kgsl_check_timestamp(device, timestamp);
	if (!status) {
		mutex_lock(&device->mutex);
		kgsl_sharedmem_readl(&device->memstore, &enableflag,
			KGSL_DEVICE_MEMSTORE_OFFSET(ts_cmp_enable));
		rmb();

		if (enableflag) {
			kgsl_sharedmem_readl(&device->memstore, &ref_ts,
				KGSL_DEVICE_MEMSTORE_OFFSET(ref_wait_ts));
			rmb();
			if (timestamp_cmp(ref_ts, timestamp)) {
				kgsl_sharedmem_writel(&device->memstore,
				KGSL_DEVICE_MEMSTORE_OFFSET(ref_wait_ts),
				timestamp);
				wmb();
			}
		} else {
			unsigned int cmds[2];
			kgsl_sharedmem_writel(&device->memstore,
				KGSL_DEVICE_MEMSTORE_OFFSET(ref_wait_ts),
				timestamp);
			enableflag = 1;
			kgsl_sharedmem_writel(&device->memstore,
				KGSL_DEVICE_MEMSTORE_OFFSET(ts_cmp_enable),
				enableflag);
			wmb();
			/* submit a dummy packet so that even if all
			* commands upto timestamp get executed we will still
			* get an interrupt */
			cmds[0] = pm4_type3_packet(PM4_NOP, 1);
			cmds[1] = 0;
			kgsl_ringbuffer_issuecmds(device, 0, &cmds[0], 2);
		}
		mutex_unlock(&device->mutex);
	}

	return status;
}

/*
 wait_event_interruptible_timeout checks for the exit condition before
 placing a process in wait q. For conditional interrupts we expect the
 process to already be in its wait q when its exit condition checking
 function is called.
*/
#define kgsl_wait_event_interruptible_timeout(wq, condition, timeout, io)\
({									\
	long __ret = timeout;						\
	if (io)						\
		__wait_io_event_interruptible_timeout(wq, condition, __ret);\
	else						\
		__wait_event_interruptible_timeout(wq, condition, __ret);\
	__ret;								\
})

/* MUST be called with the device mutex held */
static int kgsl_yamato_waittimestamp(struct kgsl_device *device,
				unsigned int timestamp,
				unsigned int msecs)
{
	long status = 0;
	uint io = 1;
	static uint io_cnt;
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	struct kgsl_yamato_device *yamato_device = KGSL_YAMATO_DEVICE(device);

	if (timestamp != yamato_device->ringbuffer.timestamp &&
		timestamp_cmp(timestamp,
		yamato_device->ringbuffer.timestamp)) {
		KGSL_DRV_ERR(device, "Cannot wait for invalid ts: %x, "
			"rb->timestamp: %x\n",
			timestamp, yamato_device->ringbuffer.timestamp);
		status = -EINVAL;
		goto done;
	}
	if (!kgsl_check_timestamp(device, timestamp)) {
		io_cnt = (io_cnt + 1) % 100;
		if (io_cnt < pwr->pwrlevels[pwr->active_pwrlevel].io_fraction)
			io = 0;
		mutex_unlock(&device->mutex);
		/* We need to make sure that the process is placed in wait-q
		 * before its condition is called */
		status = kgsl_wait_event_interruptible_timeout(
				device->wait_queue,
				kgsl_check_interrupt_timestamp(device,
					timestamp),
				msecs_to_jiffies(msecs), io);
		mutex_lock(&device->mutex);

		if (status > 0)
			status = 0;
		else if (status == 0) {
			if (!kgsl_check_timestamp(device, timestamp)) {
				status = -ETIMEDOUT;
				KGSL_DRV_ERR(device,
					"Device hang detected while waiting "
					"for timestamp: %x, last "
					"submitted(rb->timestamp): %x, wptr: "
					"%x\n", timestamp,
					yamato_device->ringbuffer.timestamp,
					yamato_device->ringbuffer.wptr);
				if (!kgsl_yamato_dump_and_recover(device)) {
					/* wait for idle after recovery as the
					 * timestamp that this process wanted
					 * to wait on may be invalid */
					if (!kgsl_yamato_idle(device,
						KGSL_TIMEOUT_DEFAULT))
						status = 0;
				}
			}
		}
	}

done:
	return (int)status;
}

static long kgsl_yamato_ioctl(struct kgsl_device_private *dev_priv,
			      unsigned int cmd, void *data)
{
	int result = 0;
	struct kgsl_drawctxt_set_bin_base_offset *binbase;
	struct kgsl_context *context;

	switch (cmd) {
	case IOCTL_KGSL_DRAWCTXT_SET_BIN_BASE_OFFSET:
		binbase = data;

		context = kgsl_find_context(dev_priv, binbase->drawctxt_id);
		if (context) {
			result = kgsl_drawctxt_set_bin_base_offset(
					dev_priv->device,
					context,
					binbase->offset);
		} else {
			result = -EINVAL;
			KGSL_DRV_ERR(dev_priv->device,
				"invalid drawctxt drawctxt_id %d "
				"device_id=%d\n",
				binbase->drawctxt_id, dev_priv->device->id);
		}
		break;

	default:
		KGSL_DRV_INFO(dev_priv->device,
			"invalid ioctl code %08x\n", cmd);
		result = -EINVAL;
		break;
	}
	return result;

}

static inline s64 kgsl_yamato_ticks_to_us(u32 ticks, u32 gpu_freq)
{
	gpu_freq /= 1000000;
	return ticks / gpu_freq;
}

static void kgsl_yamato_power_stats(struct kgsl_device *device,
				struct kgsl_power_stats *stats)
{
	unsigned int reg;
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	/* In order to calculate idle you have to have run the algorithm *
	 * at least once to get a start time. */
	if (pwr->time != 0) {
		s64 tmp;
		/* Stop the performance moniter and read the current *
		 * busy cycles. */
		kgsl_yamato_regwrite(device,
					REG_CP_PERFMON_CNTL,
					REG_PERF_MODE_CNT |
					REG_PERF_STATE_FREEZE);
		kgsl_yamato_regread(device, REG_RBBM_PERFCOUNTER1_LO, &reg);
		tmp = ktime_to_us(ktime_get());
		stats->total_time = tmp - pwr->time;
		pwr->time = tmp;
		stats->busy_time  = kgsl_yamato_ticks_to_us(reg,
				device->pwrctrl.
				pwrlevels[device->pwrctrl.active_pwrlevel].
				gpu_freq);
		kgsl_yamato_regwrite(device,
					REG_CP_PERFMON_CNTL,
					REG_PERF_MODE_CNT |
					REG_PERF_STATE_RESET);
	} else {
		stats->total_time = 0;
		stats->busy_time = 0;
		pwr->time = ktime_to_us(ktime_get());
	}

	/* re-enable the performance moniters */
	kgsl_yamato_regread(device, REG_RBBM_PM_OVERRIDE2, &reg);
	kgsl_yamato_regwrite(device, REG_RBBM_PM_OVERRIDE2, (reg | 0x40));
	kgsl_yamato_regwrite(device, REG_RBBM_PERFCOUNTER1_SELECT, 0x1);
	kgsl_yamato_regwrite(device,
				REG_CP_PERFMON_CNTL,
				REG_PERF_MODE_CNT | REG_PERF_STATE_ENABLE);
}

static void __devinit kgsl_yamato_getfunctable(struct kgsl_functable *ftbl)
{
	if (ftbl == NULL)
		return;
	ftbl->device_regread = kgsl_yamato_regread;
	ftbl->device_regwrite = kgsl_yamato_regwrite;
	ftbl->device_regread_isr = kgsl_yamato_regread_isr;
	ftbl->device_regwrite_isr = kgsl_yamato_regwrite_isr;
	ftbl->device_setstate = kgsl_yamato_setstate;
	ftbl->device_idle = kgsl_yamato_idle;
	ftbl->device_isidle = kgsl_yamato_isidle;
	ftbl->device_suspend_context = kgsl_yamato_suspend_context;
	ftbl->device_resume_context = kgsl_yamato_resume_context;
	ftbl->device_start = kgsl_yamato_start;
	ftbl->device_stop = kgsl_yamato_stop;
	ftbl->device_getproperty = kgsl_yamato_getproperty;
	ftbl->device_waittimestamp = kgsl_yamato_waittimestamp;
	ftbl->device_cmdstream_readtimestamp = kgsl_cmdstream_readtimestamp;
	ftbl->device_issueibcmds = kgsl_ringbuffer_issueibcmds;
	ftbl->device_drawctxt_create = kgsl_drawctxt_create;
	ftbl->device_drawctxt_destroy = kgsl_drawctxt_destroy;
	ftbl->device_ioctl = kgsl_yamato_ioctl;
	ftbl->device_setup_pt = kgsl_yamato_setup_pt;
	ftbl->device_cleanup_pt = kgsl_yamato_cleanup_pt;
	ftbl->device_power_stats = kgsl_yamato_power_stats;
}

static struct platform_device_id kgsl_3d_id_table[] = {
	{ DEVICE_3D0_NAME, (kernel_ulong_t)&yamato_device.dev, },
	{ },
};
MODULE_DEVICE_TABLE(platform, kgsl_3d_id_table);

static struct platform_driver kgsl_3d_platform_driver = {
	.probe = kgsl_3d_probe,
	.remove = __devexit_p(kgsl_3d_remove),
	.suspend = kgsl_suspend_driver,
	.resume = kgsl_resume_driver,
	.id_table = kgsl_3d_id_table,
	.driver = {
		.owner = THIS_MODULE,
		.name = DEVICE_3D_NAME,
		.pm = &kgsl_pm_ops,
	}
};

static int __init kgsl_3d_init(void)
{
	return platform_driver_register(&kgsl_3d_platform_driver);
}

static void __exit kgsl_3d_exit(void)
{
	platform_driver_unregister(&kgsl_3d_platform_driver);
}

module_init(kgsl_3d_init);
module_exit(kgsl_3d_exit);

MODULE_DESCRIPTION("3D Graphics driver");
MODULE_VERSION("1.2");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:kgsl_3d");
