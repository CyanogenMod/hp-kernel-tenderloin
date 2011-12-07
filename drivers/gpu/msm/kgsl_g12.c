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
#include <linux/workqueue.h>
#include <linux/notifier.h>
#include <linux/pm_runtime.h>

#include <mach/msm_bus.h>

#include "kgsl.h"
#include "kgsl_g12.h"
#include "kgsl_log.h"
#include "kgsl_g12_drawctxt.h"
#include "kgsl_g12_cmdstream.h"
#include "kgsl_g12_cmdwindow.h"
#include "kgsl_sharedmem.h"
#include "kgsl_g12_vgv3types.h"
#include "kgsl_cffdump.h"

#include "g12_reg.h"

#define DRIVER_VERSION_MAJOR   3
#define DRIVER_VERSION_MINOR   1

#define GSL_VGC_INT_MASK \
	 (REG_VGC_IRQSTATUS__MH_MASK | \
	  REG_VGC_IRQSTATUS__G2D_MASK | \
	  REG_VGC_IRQSTATUS__FIFO_MASK)

/* G12 MH arbiter config*/
#define KGSL_G12_CFG_G12_MHARB \
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

#define KGSL_G12_TIMESTAMP_EPSILON 20000
#define KGSL_G12_IDLE_COUNT_MAX 1000000

#define KGSL_G12_CMDWINDOW_TARGET_MASK		0x000000FF
#define KGSL_G12_CMDWINDOW_ADDR_MASK		0x00FFFF00
#define KGSL_G12_CMDWINDOW_TARGET_SHIFT		0
#define KGSL_G12_CMDWINDOW_ADDR_SHIFT		8

static int kgsl_g12_start(struct kgsl_device *device, unsigned int init_ram);
static int kgsl_g12_stop(struct kgsl_device *device);
static int kgsl_g12_wait(struct kgsl_device *device,
				unsigned int timestamp,
				unsigned int msecs);
static int kgsl_g12_waittimestamp(struct kgsl_device *device,
				unsigned int timestamp,
				unsigned int msecs);

#define KGSL_2D_MMU_CONFIG					     \
	(0x01							     \
	| (MMU_CONFIG << MH_MMU_CONFIG__RB_W_CLNT_BEHAVIOR__SHIFT)   \
	| (MMU_CONFIG << MH_MMU_CONFIG__CP_W_CLNT_BEHAVIOR__SHIFT)   \
	| (MMU_CONFIG << MH_MMU_CONFIG__CP_R0_CLNT_BEHAVIOR__SHIFT)  \
	| (MMU_CONFIG << MH_MMU_CONFIG__CP_R1_CLNT_BEHAVIOR__SHIFT)  \
	| (MMU_CONFIG << MH_MMU_CONFIG__CP_R2_CLNT_BEHAVIOR__SHIFT)  \
	| (MMU_CONFIG << MH_MMU_CONFIG__CP_R3_CLNT_BEHAVIOR__SHIFT)  \
	| (MMU_CONFIG << MH_MMU_CONFIG__CP_R4_CLNT_BEHAVIOR__SHIFT)  \
	| (MMU_CONFIG << MH_MMU_CONFIG__VGT_R0_CLNT_BEHAVIOR__SHIFT) \
	| (MMU_CONFIG << MH_MMU_CONFIG__VGT_R1_CLNT_BEHAVIOR__SHIFT) \
	| (MMU_CONFIG << MH_MMU_CONFIG__TC_R_CLNT_BEHAVIOR__SHIFT)   \
	| (MMU_CONFIG << MH_MMU_CONFIG__PA_W_CLNT_BEHAVIOR__SHIFT))

static struct kgsl_g12_device device_2d0 = {
	.dev = {
		.name = DEVICE_2D0_NAME,
		.id = KGSL_DEVICE_2D0,
		.ver_major = DRIVER_VERSION_MAJOR,
		.ver_minor = DRIVER_VERSION_MINOR,
		.mmu = {
			.config = KGSL_2D_MMU_CONFIG,
			/* turn off memory protection unit by setting
			   acceptable physical address range to include
			   all pages. */
			.mpu_base = 0x00000000,
			.mpu_range =  0xFFFFF000,
			.reg = {
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
				.interrupt_clear = ADDR_MH_INTERRUPT_CLEAR,
				.axi_error = ADDR_MH_AXI_ERROR,
			},
		},
		.pwrctrl = {
			.pwr_rail = PWR_RAIL_GRP_2D_CLK,
			.regulator_name = "fs_gfx2d0",
			.irq_name = KGSL_2D0_IRQ,
		},
		.mutex = __MUTEX_INITIALIZER(device_2d0.dev.mutex),
		.state = KGSL_STATE_INIT,
		.active_cnt = 0,
		.iomemname = KGSL_2D0_REG_MEMORY,
		.display_off = {
#ifdef CONFIG_HAS_EARLYSUSPEND
			.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING,
			.suspend = kgsl_early_suspend_driver,
			.resume = kgsl_late_resume_driver,
#endif
		},
	},
};

static struct kgsl_g12_device device_2d1 = {
	.dev = {
		.name = DEVICE_2D1_NAME,
		.id = KGSL_DEVICE_2D1,
		.ver_major = DRIVER_VERSION_MAJOR,
		.ver_minor = DRIVER_VERSION_MINOR,
		.mmu = {
			.config = KGSL_2D_MMU_CONFIG,
			/* turn off memory protection unit by setting
			   acceptable physical address range to include
			   all pages. */
			.mpu_base = 0x00000000,
			.mpu_range =  0xFFFFF000,
			.reg = {
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
				.interrupt_clear = ADDR_MH_INTERRUPT_CLEAR,
				.axi_error = ADDR_MH_AXI_ERROR,
			},
		},
		.pwrctrl = {
			.pwr_rail = PWR_RAIL_GRP_2D_CLK,
			.regulator_name = "fs_gfx2d1",
			.irq_name = KGSL_2D1_IRQ,
		},
		.mutex = __MUTEX_INITIALIZER(device_2d1.dev.mutex),
		.state = KGSL_STATE_INIT,
		.active_cnt = 0,
		.iomemname = KGSL_2D1_REG_MEMORY,
		.display_off = {
#ifdef CONFIG_HAS_EARLYSUSPEND
			.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING,
			.suspend = kgsl_early_suspend_driver,
			.resume = kgsl_late_resume_driver,
#endif
		},
	},
};

irqreturn_t kgsl_g12_isr(int irq, void *data)
{
	irqreturn_t result = IRQ_NONE;
	unsigned int status;
	struct kgsl_device *device = (struct kgsl_device *) data;
	struct kgsl_g12_device *g12_device = KGSL_G12_DEVICE(device);

	kgsl_g12_regread_isr(device, ADDR_VGC_IRQSTATUS >> 2, &status);

	if (status & GSL_VGC_INT_MASK) {
		kgsl_g12_regwrite_isr(device,
			ADDR_VGC_IRQSTATUS >> 2, status & GSL_VGC_INT_MASK);

		result = IRQ_HANDLED;

		if (status & REG_VGC_IRQSTATUS__FIFO_MASK)
			KGSL_DRV_ERR(device, "g12 fifo interrupt\n");
		if (status & REG_VGC_IRQSTATUS__MH_MASK)
			kgsl_mh_intrcallback(device);
		if (status & REG_VGC_IRQSTATUS__G2D_MASK) {
			int count;

			kgsl_g12_regread_isr(device,
					 ADDR_VGC_IRQ_ACTIVE_CNT >> 2,
					 &count);

			count >>= 8;
			count &= 255;
			g12_device->timestamp += count;

			wake_up_interruptible(&device->wait_queue);

			atomic_notifier_call_chain(
				&(device->ts_notifier_list),
				device->id, NULL);
		}
	}

	if ((device->pwrctrl.nap_allowed == true) &&
		(device->requested_state == KGSL_STATE_NONE)) {
		device->requested_state = KGSL_STATE_NAP;
		queue_work(device->work_queue, &device->idle_check_ws);
	}

	mod_timer(&device->idle_timer,
			jiffies + device->pwrctrl.interval_timeout);

	return result;
}

static int kgsl_g12_cleanup_pt(struct kgsl_device *device,
			       struct kgsl_pagetable *pagetable)
{
	struct kgsl_g12_device *g12_device = KGSL_G12_DEVICE(device);

	kgsl_mmu_unmap(pagetable, device->mmu.dummyspace.gpuaddr,
			device->mmu.dummyspace.size);

	kgsl_mmu_unmap(pagetable, device->memstore.gpuaddr,
			device->memstore.size);

	kgsl_mmu_unmap(pagetable, g12_device->ringbuffer.cmdbufdesc.gpuaddr,
			g12_device->ringbuffer.cmdbufdesc.size);
	return 0;
}

static int kgsl_g12_setup_pt(struct kgsl_device *device,
			     struct kgsl_pagetable *pagetable)
{
	int result = 0;
	unsigned int flags = KGSL_MEMFLAGS_CONPHYS | KGSL_MEMFLAGS_ALIGN4K;
	struct kgsl_g12_device *g12_device = KGSL_G12_DEVICE(device);

	result = kgsl_mmu_map_global(pagetable, &device->mmu.dummyspace,
				     GSL_PT_PAGE_RV | GSL_PT_PAGE_WV, flags);
	if (result)
		goto error;

	result = kgsl_mmu_map_global(pagetable, &device->memstore,
				     GSL_PT_PAGE_RV | GSL_PT_PAGE_WV, flags);
	if (result)
		goto error_unmap_dummy;

	result = kgsl_mmu_map_global(pagetable,
				     &g12_device->ringbuffer.cmdbufdesc,
				     GSL_PT_PAGE_RV, flags);
	if (result)
		goto error_unmap_memstore;
	return result;

error_unmap_dummy:
	kgsl_mmu_unmap(pagetable, device->mmu.dummyspace.gpuaddr,
			device->mmu.dummyspace.size);
error_unmap_memstore:
	kgsl_mmu_unmap(pagetable, device->memstore.gpuaddr,
			device->memstore.size);
error:
	return result;
}

int kgsl_g12_setstate(struct kgsl_device *device, uint32_t flags)
{
#ifdef CONFIG_MSM_KGSL_MMU
	unsigned int mh_mmu_invalidate = 0x00000003; /*invalidate all and tc */

	if (flags & KGSL_MMUFLAGS_PTUPDATE) {
		kgsl_g12_idle(device, KGSL_TIMEOUT_DEFAULT);
		kgsl_g12_regwrite(device, ADDR_MH_MMU_PT_BASE,
				     device->mmu.hwpagetable->base.gpuaddr);
		kgsl_g12_regwrite(device, ADDR_MH_MMU_VA_RANGE,
				     (device->mmu.hwpagetable->
				      va_base | (device->mmu.hwpagetable->
						 va_range >> 16)));
		kgsl_g12_regwrite(device, ADDR_MH_MMU_INVALIDATE,
				     mh_mmu_invalidate);
	}

	if (flags & KGSL_MMUFLAGS_TLBFLUSH)
		kgsl_g12_regwrite(device, ADDR_MH_MMU_INVALIDATE,
			     mh_mmu_invalidate);
#endif
	return 0;
}

static void kgsl_g12_getfunctable(struct kgsl_functable *ftbl);

static int __devinit kgsl_2d_probe(struct platform_device *pdev)
{
	int status = -EINVAL;
	struct kgsl_device *device = NULL;
	struct kgsl_g12_device *g12_device;

	device = (struct kgsl_device *)pdev->id_entry->driver_data;
	device->pdev = pdev;

	kgsl_g12_getfunctable(&device->ftbl);

	g12_device = KGSL_G12_DEVICE(device);
	spin_lock_init(&g12_device->cmdwin_lock);

	status = kgsl_g12_cmdstream_init(device);
	if (status != 0)
		goto error;

	status = kgsl_device_probe(device, kgsl_g12_isr);
	if (status)
		goto error_close_cmdstream;

	return status;

error_close_cmdstream:
	kgsl_g12_cmdstream_close(device);
error:
	device->pdev = NULL;
	return status;
}

static int __devexit kgsl_2d_remove(struct platform_device *pdev)
{
	struct kgsl_device *device = NULL;

	device = (struct kgsl_device *)pdev->id_entry->driver_data;

	kgsl_device_remove(device);

	kgsl_g12_cmdstream_close(device);

	return 0;
}

static int kgsl_g12_start(struct kgsl_device *device, unsigned int init_ram)
{
	int status = 0;

	device->state = KGSL_STATE_INIT;
	device->requested_state = KGSL_STATE_NONE;
	KGSL_PWR_WARN(device, "state -> INIT, device %d\n", device->id);

	/* Order pwrrail/clk sequence based upon platform. */
	if (device->pwrctrl.pwrrail_first)
		kgsl_pwrctrl_pwrrail(device, KGSL_PWRFLAGS_POWER_ON);
	kgsl_pwrctrl_clk(device, KGSL_PWRFLAGS_CLK_ON);
	kgsl_pwrctrl_axi(device, KGSL_PWRFLAGS_AXI_ON);
	if (!device->pwrctrl.pwrrail_first)
		kgsl_pwrctrl_pwrrail(device, KGSL_PWRFLAGS_POWER_ON);

	/* Set up MH arbiter.  MH offsets are considered to be dword
	 * based, therefore no down shift. */
	kgsl_g12_regwrite(device, ADDR_MH_ARBITER_CONFIG,
			  KGSL_G12_CFG_G12_MHARB);

	kgsl_g12_regwrite(device, ADDR_MH_CLNT_INTF_CTRL_CONFIG1, 0x00030F27);
	kgsl_g12_regwrite(device, ADDR_MH_CLNT_INTF_CTRL_CONFIG2, 0x004B274F);

	kgsl_g12_regwrite(device, (ADDR_VGC_IRQENABLE >> 2), 0x3);

	status = kgsl_mmu_start(device);
	if (status)
		goto error_clk_off;

	status = kgsl_g12_cmdstream_start(device);
	if (status)
		goto error_mmu_stop;

	mod_timer(&device->idle_timer, jiffies + FIRST_TIMEOUT);
	kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_IRQ_ON);
	return 0;
error_clk_off:
	kgsl_g12_regwrite(device, (ADDR_VGC_IRQENABLE >> 2), 0);
	kgsl_pwrctrl_axi(device, KGSL_PWRFLAGS_AXI_OFF);
	kgsl_pwrctrl_clk(device, KGSL_PWRFLAGS_CLK_OFF);
error_mmu_stop:
	kgsl_mmu_stop(device);
	return status;
}

static int kgsl_g12_stop(struct kgsl_device *device)
{
	kgsl_g12_idle(device, KGSL_TIMEOUT_DEFAULT);

	del_timer(&device->idle_timer);

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

static int kgsl_g12_getproperty(struct kgsl_device *device,
				enum kgsl_property_type type,
				void *value,
				unsigned int sizebytes)
{
	int status = -EINVAL;

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

		if (copy_to_user(value, &devinfo, sizeof(devinfo)) !=
				0) {
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

	default:
		KGSL_DRV_ERR(device, "invalid property: %d\n", type);
		status = -EINVAL;
	}
	return status;
}

int kgsl_g12_idle(struct kgsl_device *device, unsigned int timeout)
{
	int status = 0;
	struct kgsl_g12_device *g12_device = KGSL_G12_DEVICE(device);

	if (g12_device->current_timestamp > g12_device->timestamp)
		status = kgsl_g12_wait(device,
					g12_device->current_timestamp, timeout);

	if (status)
		KGSL_DRV_ERR(device, "kgsl_g12_waittimestamp() timed out\n");

	return status;
}

static unsigned int kgsl_g12_isidle(struct kgsl_device *device)
{
	int status = false;
	struct kgsl_g12_device *g12_device = KGSL_G12_DEVICE(device);

	int timestamp = g12_device->timestamp;

	if (timestamp == g12_device->current_timestamp)
		status = true;

	return status;
}

static int kgsl_g12_resume_context(struct kgsl_device *device)
{
	/* Context is in the pre-amble, automatically restored. */

	return 0;
}

static int kgsl_g12_suspend_context(struct kgsl_device *device)
{
	struct kgsl_g12_device *g12_device = KGSL_G12_DEVICE(device);

	g12_device->ringbuffer.prevctx = KGSL_G12_INVALID_CONTEXT;

	return 0;
}

/* Not all Z180 registers are directly accessible.
 * The _g12_(read|write)_simple functions below handle the ones that are.
 */
static void _g12_regread_simple(struct kgsl_device *device,
				unsigned int offsetwords,
				unsigned int *value)
{
	unsigned int *reg;

	BUG_ON(offsetwords * sizeof(uint32_t) >= device->regspace.sizebytes);

	reg = (unsigned int *)(device->regspace.mmio_virt_base
			+ (offsetwords << 2));

	*value = readl(reg);
}

static void _g12_regwrite_simple(struct kgsl_device *device,
				 unsigned int offsetwords,
				 unsigned int value)
{
	unsigned int *reg;

	BUG_ON(offsetwords*sizeof(uint32_t) >= device->regspace.sizebytes);

	reg = (unsigned int *)(device->regspace.mmio_virt_base
			+ (offsetwords << 2));
	kgsl_cffdump_regwrite(device->id, offsetwords << 2, value);
	writel(value, reg);
}


/* The MH registers must be accessed through via a 2 step write, (read|write)
 * process. These registers may be accessed from interrupt context during
 * the handling of MH or MMU error interrupts. Therefore a spin lock is used
 * to ensure that the 2 step sequence is not interrupted.
 */
static void _g12_regread_mmu(struct kgsl_device *device,
			     unsigned int offsetwords,
			     unsigned int *value)
{
	struct kgsl_g12_device *g12_device = KGSL_G12_DEVICE(device);
	unsigned long flags;

	spin_lock_irqsave(&g12_device->cmdwin_lock, flags);
	_g12_regwrite_simple(device, (ADDR_VGC_MH_READ_ADDR >> 2), offsetwords);
	_g12_regread_simple(device, (ADDR_VGC_MH_DATA_ADDR >> 2), value);
	spin_unlock_irqrestore(&g12_device->cmdwin_lock, flags);
}


static void _g12_regwrite_mmu(struct kgsl_device *device,
			      unsigned int offsetwords,
			      unsigned int value)
{
	struct kgsl_g12_device *g12_device = KGSL_G12_DEVICE(device);
	unsigned int cmdwinaddr;
	unsigned long flags;

	cmdwinaddr = ((KGSL_CMDWINDOW_MMU << KGSL_G12_CMDWINDOW_TARGET_SHIFT) &
			KGSL_G12_CMDWINDOW_TARGET_MASK);
	cmdwinaddr |= ((offsetwords << KGSL_G12_CMDWINDOW_ADDR_SHIFT) &
			KGSL_G12_CMDWINDOW_ADDR_MASK);

	spin_lock_irqsave(&g12_device->cmdwin_lock, flags);
	_g12_regwrite_simple(device, ADDR_VGC_MMUCOMMANDSTREAM >> 2,
			     cmdwinaddr);
	_g12_regwrite_simple(device, ADDR_VGC_MMUCOMMANDSTREAM >> 2, value);
	spin_unlock_irqrestore(&g12_device->cmdwin_lock, flags);
}

/* the rest of the code doesn't want to think about if it is writing mmu
 * registers or normal registers so handle it here
 */
static void _g12_regread(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int *value)
{
	if ((offsetwords >= ADDR_MH_ARBITER_CONFIG &&
	     offsetwords <= ADDR_MH_AXI_HALT_CONTROL) ||
	    (offsetwords >= ADDR_MH_MMU_CONFIG &&
	     offsetwords <= ADDR_MH_MMU_MPU_END)) {
		_g12_regread_mmu(device, offsetwords, value);
	} else {
		_g12_regread_simple(device, offsetwords, value);
	}
}

static void _g12_regwrite(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int value)
{
	if ((offsetwords >= ADDR_MH_ARBITER_CONFIG &&
	     offsetwords <= ADDR_MH_CLNT_INTF_CTRL_CONFIG2) ||
	    (offsetwords >= ADDR_MH_MMU_CONFIG &&
	     offsetwords <= ADDR_MH_MMU_MPU_END)) {
		_g12_regwrite_mmu(device, offsetwords, value);

	} else {
		_g12_regwrite_simple(device, offsetwords, value);
	}
}


void kgsl_g12_regread(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int *value)
{
	kgsl_pre_hwaccess(device);
	_g12_regread(device, offsetwords, value);
}

void kgsl_g12_regread_isr(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int *value)
{
	_g12_regread(device, offsetwords, value);
}

void kgsl_g12_regwrite(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int value)
{
	kgsl_pre_hwaccess(device);
	_g12_regwrite(device, offsetwords, value);
}

void kgsl_g12_regwrite_isr(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int value)
{
	_g12_regwrite(device, offsetwords, value);
}

int kgsl_g12_cmdwindow_write(struct kgsl_device *device,
		enum kgsl_cmdwindow_type target, unsigned int addr,
		unsigned int data)
{
	struct kgsl_g12_device *g12_device = KGSL_G12_DEVICE(device);
	unsigned int cmdwinaddr;
	unsigned int cmdstream;
	unsigned long flags;

	if (target < KGSL_CMDWINDOW_MIN ||
		target > KGSL_CMDWINDOW_MAX) {
		KGSL_DRV_ERR(device, "invalid target %d\n", target);
		return -EINVAL;
	}

	if (target == KGSL_CMDWINDOW_MMU)
		cmdstream = ADDR_VGC_MMUCOMMANDSTREAM;
	else
		cmdstream = ADDR_VGC_COMMANDSTREAM;

	cmdwinaddr = ((target << KGSL_G12_CMDWINDOW_TARGET_SHIFT) &
			KGSL_G12_CMDWINDOW_TARGET_MASK);
	cmdwinaddr |= ((addr << KGSL_G12_CMDWINDOW_ADDR_SHIFT) &
			KGSL_G12_CMDWINDOW_ADDR_MASK);

	kgsl_pre_hwaccess(device);

	spin_lock_irqsave(&g12_device->cmdwin_lock, flags);
	_g12_regwrite_simple(device, cmdstream >> 2, cmdwinaddr);
	_g12_regwrite_simple(device, cmdstream >> 2, data);
	spin_unlock_irqrestore(&g12_device->cmdwin_lock, flags);

	return 0;
}

static int kgsl_g12_waittimestamp(struct kgsl_device *device,
				unsigned int timestamp,
				unsigned int msecs)
{
	int status = -EINVAL;
	mutex_unlock(&device->mutex);
	status = kgsl_g12_wait(device, timestamp, msecs);
	mutex_lock(&device->mutex);

	return status;
}

static int kgsl_g12_wait(struct kgsl_device *device,
				unsigned int timestamp,
				unsigned int msecs)
{
	int status = -EINVAL;
	long timeout = 0;

	timeout = wait_io_event_interruptible_timeout(
			device->wait_queue,
			kgsl_check_timestamp(device, timestamp),
			msecs_to_jiffies(msecs));

	if (timeout > 0)
		status = 0;
	else if (timeout == 0) {
		status = -ETIMEDOUT;
		device->state = KGSL_STATE_HUNG;
		KGSL_PWR_WARN(device, "state -> HUNG, device %d\n", device->id);
	}
	else
		status = timeout;

	return status;
}

static long kgsl_g12_ioctl_cmdwindow_write(struct kgsl_device_private *dev_priv,
					   void *data)
{
	struct kgsl_cmdwindow_write *param = data;

	return kgsl_g12_cmdwindow_write(dev_priv->device,
					param->target,
					param->addr,
					param->data);
}

static long kgsl_g12_ioctl(struct kgsl_device_private *dev_priv,
			   unsigned int cmd, void *data)
{
	int result = 0;

	switch (cmd) {
	case IOCTL_KGSL_CMDWINDOW_WRITE:
		result = kgsl_g12_ioctl_cmdwindow_write(dev_priv, data);
		break;
	default:
		KGSL_DRV_INFO(dev_priv->device,
			"invalid ioctl code %08x\n", cmd);
		result = -EINVAL;
		break;
	}
	return result;

}

static void kgsl_g12_power_stats(struct kgsl_device *device,
				struct kgsl_power_stats *stats)
{
	stats->total_time = 0;
	stats->busy_time = 0;
}

static void kgsl_g12_getfunctable(struct kgsl_functable *ftbl)
{

	if (ftbl == NULL)
		return;
	ftbl->device_regread = kgsl_g12_regread;
	ftbl->device_regwrite = kgsl_g12_regwrite;
	ftbl->device_regread_isr = kgsl_g12_regread_isr;
	ftbl->device_regwrite_isr = kgsl_g12_regwrite_isr;
	ftbl->device_setstate = kgsl_g12_setstate;
	ftbl->device_idle = kgsl_g12_idle;
	ftbl->device_isidle = kgsl_g12_isidle;
	ftbl->device_suspend_context = kgsl_g12_suspend_context;
	ftbl->device_resume_context = kgsl_g12_resume_context;
	ftbl->device_start = kgsl_g12_start;
	ftbl->device_stop = kgsl_g12_stop;
	ftbl->device_getproperty = kgsl_g12_getproperty;
	ftbl->device_waittimestamp = kgsl_g12_waittimestamp;
	ftbl->device_cmdstream_readtimestamp = kgsl_g12_cmdstream_readtimestamp;
	ftbl->device_issueibcmds = kgsl_g12_cmdstream_issueibcmds;
	ftbl->device_drawctxt_create = kgsl_g12_drawctxt_create;
	ftbl->device_drawctxt_destroy = kgsl_g12_drawctxt_destroy;
	ftbl->device_ioctl = kgsl_g12_ioctl;
	ftbl->device_setup_pt = kgsl_g12_setup_pt;
	ftbl->device_cleanup_pt = kgsl_g12_cleanup_pt;
	ftbl->device_power_stats = kgsl_g12_power_stats;
}

static struct platform_device_id kgsl_2d_id_table[] = {
	{ DEVICE_2D0_NAME, (kernel_ulong_t)&device_2d0.dev, },
	{ DEVICE_2D1_NAME, (kernel_ulong_t)&device_2d1.dev, },
	{ },
};
MODULE_DEVICE_TABLE(platform, kgsl_2d_id_table);

static struct platform_driver kgsl_2d_platform_driver = {
	.probe = kgsl_2d_probe,
	.remove = __devexit_p(kgsl_2d_remove),
	.suspend = kgsl_suspend_driver,
	.resume = kgsl_resume_driver,
	.id_table = kgsl_2d_id_table,
	.driver = {
		.owner = THIS_MODULE,
		.name = DEVICE_2D_NAME,
		.pm = &kgsl_pm_ops,
	}
};

static int __init kgsl_2d_init(void)
{
	return platform_driver_register(&kgsl_2d_platform_driver);
}

static void __exit kgsl_2d_exit(void)
{
	platform_driver_unregister(&kgsl_2d_platform_driver);
}

module_init(kgsl_2d_init);
module_exit(kgsl_2d_exit);

MODULE_DESCRIPTION("2D Graphics driver");
MODULE_VERSION("1.2");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:kgsl_2d");
