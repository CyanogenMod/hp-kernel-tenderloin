/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/spinlock.h>
#include <mach/msm_iomap.h>

#include "mpm.h"

/******************************************************************************
 * Debug Definitions
 *****************************************************************************/

enum {
	MSM_MPM_DEBUG_NON_DETECTABLE_IRQ = BIT(0),
	MSM_MPM_DEBUG_PENDING_IRQ = BIT(1),
	MSM_MPM_DEBUG_WRITE = BIT(2),
};

static int msm_mpm_debug_mask;
module_param_named(
	debug_mask, msm_mpm_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP
);

/******************************************************************************
 * Request and Status Definitions
 *****************************************************************************/

#define MSM_MPM_REQUEST_REG_START  0x9d8
#define MSM_MPM_STATUS_REG_START  0xdf8
#define MSM_MPM_REQUEST_BASE  (MSM_RPM_BASE + MSM_MPM_REQUEST_REG_START)
#define MSM_MPM_STATUS_BASE  (MSM_RPM_BASE + MSM_MPM_STATUS_REG_START)

enum {
	MSM_MPM_REQUEST_REG_ENABLE,
	MSM_MPM_REQUEST_REG_DETECT_CTL,
	MSM_MPM_REQUEST_REG_POLARITY,
	MSM_MPM_REQUEST_REG_CLEAR,
};

enum {
	MSM_MPM_STATUS_REG_PENDING,
};

/******************************************************************************
 * IPC Definitions
 *****************************************************************************/

#define MSM_MPM_APPS_IPC  (MSM_GCC_BASE + 0x008)
#define MSM_MPM_APPS_IPC_MPM  BIT(1)
#define MSM_MPM_IPC_IRQ  RPM_SCSS_CPU0_GP_MEDIUM_IRQ

/******************************************************************************
 * IRQ Mapping Definitions
 *****************************************************************************/

#define MSM_MPM_NR_APPS_IRQS  (NR_MSM_IRQS + NR_GPIO_IRQS)
#define MSM_MPM_NR_MPM_IRQS  64

#define MSM_MPM_REG_WIDTH  DIV_ROUND_UP(MSM_MPM_NR_MPM_IRQS, 32)
#define MSM_MPM_IRQ_INDEX(irq)  (irq / 32)
#define MSM_MPM_IRQ_MASK(irq)  BIT(irq % 32)

static uint8_t msm_mpm_irqs_a2m[MSM_MPM_NR_APPS_IRQS];
static uint16_t msm_mpm_irqs_m2a[MSM_MPM_NR_MPM_IRQS] = {
	[1] = MSM_GPIO_TO_INT(61),
	[4] = MSM_GPIO_TO_INT(87),
	[5] = MSM_GPIO_TO_INT(88),
	[6] = MSM_GPIO_TO_INT(89),
	[7] = MSM_GPIO_TO_INT(90),
	[8] = MSM_GPIO_TO_INT(91),
	[9] = MSM_GPIO_TO_INT(34),
	[10] = MSM_GPIO_TO_INT(38),
	[11] = MSM_GPIO_TO_INT(42),
	[12] = MSM_GPIO_TO_INT(46),
	[13] = MSM_GPIO_TO_INT(50),
	[14] = MSM_GPIO_TO_INT(54),
	[15] = MSM_GPIO_TO_INT(58),
	[16] = MSM_GPIO_TO_INT(63),
	[17] = MSM_GPIO_TO_INT(160),
	[18] = MSM_GPIO_TO_INT(162),
	[19] = MSM_GPIO_TO_INT(144),
	[20] = MSM_GPIO_TO_INT(146),
	[25] = USB1_HS_IRQ,
	[26] = TV_ENC_IRQ,
	[27] = HDMI_IRQ,
	[29] = MSM_GPIO_TO_INT(123),
	[30] = MSM_GPIO_TO_INT(172),
	[31] = MSM_GPIO_TO_INT(99),
	[32] = MSM_GPIO_TO_INT(96),
	[33] = MSM_GPIO_TO_INT(67),
	[34] = MSM_GPIO_TO_INT(71),
	[35] = MSM_GPIO_TO_INT(105),
	[36] = MSM_GPIO_TO_INT(117),
	[37] = MSM_GPIO_TO_INT(29),
	[38] = MSM_GPIO_TO_INT(30),
	[39] = MSM_GPIO_TO_INT(31),
	[40] = MSM_GPIO_TO_INT(37),
	[41] = MSM_GPIO_TO_INT(40),
	[42] = MSM_GPIO_TO_INT(41),
	[43] = MSM_GPIO_TO_INT(45),
	[44] = MSM_GPIO_TO_INT(51),
	[45] = MSM_GPIO_TO_INT(52),
	[46] = MSM_GPIO_TO_INT(57),
	[47] = MSM_GPIO_TO_INT(73),
	[48] = MSM_GPIO_TO_INT(93),
	[49] = MSM_GPIO_TO_INT(94),
	[50] = MSM_GPIO_TO_INT(103),
	[51] = MSM_GPIO_TO_INT(104),
	[52] = MSM_GPIO_TO_INT(106),
	[53] = MSM_GPIO_TO_INT(115),
	[54] = MSM_GPIO_TO_INT(124),
	[55] = MSM_GPIO_TO_INT(125),
	[56] = MSM_GPIO_TO_INT(126),
	[57] = MSM_GPIO_TO_INT(127),
	[58] = MSM_GPIO_TO_INT(128),
	[59] = MSM_GPIO_TO_INT(129),
};

static uint16_t msm_mpm_bypassed_apps_irqs[] = {
	TLMM_SCSS_SUMMARY_IRQ,
	RPM_SCSS_CPU0_GP_HIGH_IRQ,
	RPM_SCSS_CPU0_GP_MEDIUM_IRQ,
	RPM_SCSS_CPU0_GP_LOW_IRQ,
	RPM_SCSS_CPU0_WAKE_UP_IRQ,
	RPM_SCSS_CPU1_GP_HIGH_IRQ,
	RPM_SCSS_CPU1_GP_MEDIUM_IRQ,
	RPM_SCSS_CPU1_GP_LOW_IRQ,
	RPM_SCSS_CPU1_WAKE_UP_IRQ,
	MARM_SCSS_GP_IRQ_0,
	MARM_SCSS_GP_IRQ_1,
	MARM_SCSS_GP_IRQ_2,
	MARM_SCSS_GP_IRQ_3,
	MARM_SCSS_GP_IRQ_4,
	MARM_SCSS_GP_IRQ_5,
	MARM_SCSS_GP_IRQ_6,
	MARM_SCSS_GP_IRQ_7,
	MARM_SCSS_GP_IRQ_8,
	MARM_SCSS_GP_IRQ_9,
	LPASS_SCSS_GP_LOW_IRQ,
	LPASS_SCSS_GP_MEDIUM_IRQ,
	LPASS_SCSS_GP_HIGH_IRQ,
	SDC4_IRQ_0,
	SPS_MTI_31,
};

static DEFINE_SPINLOCK(msm_mpm_lock);

/*
 * Note: the following two bitmaps only mark irqs that are _not_
 * mappable to MPM.
 */
static DECLARE_BITMAP(msm_mpm_enabled_apps_irqs, MSM_MPM_NR_APPS_IRQS);
static DECLARE_BITMAP(msm_mpm_wake_apps_irqs, MSM_MPM_NR_APPS_IRQS);

static DECLARE_BITMAP(msm_mpm_gic_irqs_mask, MSM_MPM_NR_APPS_IRQS);
static DECLARE_BITMAP(msm_mpm_gpio_irqs_mask, MSM_MPM_NR_APPS_IRQS);


static uint32_t msm_mpm_enabled_irq[MSM_MPM_REG_WIDTH];
static uint32_t msm_mpm_wake_irq[MSM_MPM_REG_WIDTH];
static uint32_t msm_mpm_detect_ctl[MSM_MPM_REG_WIDTH];
static uint32_t msm_mpm_polarity[MSM_MPM_REG_WIDTH];


/******************************************************************************
 * Low Level Functions for Accessing MPM
 *****************************************************************************/

static inline uint32_t msm_mpm_read(
	unsigned int reg, unsigned int subreg_index)
{
	unsigned int offset = reg * MSM_MPM_REG_WIDTH + subreg_index;
	return readl(MSM_MPM_STATUS_BASE + offset * 4);
}

static inline void msm_mpm_write(
	unsigned int reg, unsigned int subreg_index, uint32_t value)
{
	unsigned int offset = reg * MSM_MPM_REG_WIDTH + subreg_index;
	writel(value, MSM_MPM_REQUEST_BASE + offset * 4);

	if (MSM_MPM_DEBUG_WRITE & msm_mpm_debug_mask)
		pr_info("%s: reg %u.%u: 0x%08x\n",
			__func__, reg, subreg_index, value);
}

static inline void msm_mpm_write_barrier(void)
{
	/*
	 * By the time the read from memory returns, all previous
	 * writes are guaranteed visible.
	 */
	msm_mpm_read(MSM_MPM_STATUS_REG_PENDING, 0);
}

static inline void msm_mpm_send_interrupt(void)
{
	writel(MSM_MPM_APPS_IPC_MPM, MSM_MPM_APPS_IPC);
}

static irqreturn_t msm_mpm_irq(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

/******************************************************************************
 * MPM Access Functions
 *****************************************************************************/

static void msm_mpm_set(bool wakeset)
{
	uint32_t *irqs;
	unsigned int reg;
	int i;

	irqs = wakeset ? msm_mpm_wake_irq : msm_mpm_enabled_irq;
	for (i = 0; i < MSM_MPM_REG_WIDTH; i++) {
		reg = MSM_MPM_REQUEST_REG_ENABLE;
		msm_mpm_write(reg, i, irqs[i]);

		reg = MSM_MPM_REQUEST_REG_DETECT_CTL;
		msm_mpm_write(reg, i, msm_mpm_detect_ctl[i]);

		reg = MSM_MPM_REQUEST_REG_POLARITY;
		msm_mpm_write(reg, i, msm_mpm_polarity[i]);

		reg = MSM_MPM_REQUEST_REG_CLEAR;
		msm_mpm_write(reg, i, 0xffffffff);
	}

	msm_mpm_write_barrier();
	msm_mpm_send_interrupt();
}

/******************************************************************************
 * Interrupt Mapping Functions
 *****************************************************************************/

static inline bool msm_mpm_is_valid_apps_irq(unsigned int irq)
{
	return irq < ARRAY_SIZE(msm_mpm_irqs_a2m);
}

static inline uint8_t msm_mpm_get_irq_a2m(unsigned int irq)
{
	return msm_mpm_irqs_a2m[irq];
}

static inline void msm_mpm_set_irq_a2m(unsigned int apps_irq,
	unsigned int mpm_irq)
{
	msm_mpm_irqs_a2m[apps_irq] = (uint8_t) mpm_irq;
}

static inline bool msm_mpm_is_valid_mpm_irq(unsigned int irq)
{
	return irq < ARRAY_SIZE(msm_mpm_irqs_m2a);
}

static inline uint16_t msm_mpm_get_irq_m2a(unsigned int irq)
{
	return msm_mpm_irqs_m2a[irq];
}

static bool msm_mpm_bypass_apps_irq(unsigned int irq)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(msm_mpm_bypassed_apps_irqs); i++)
		if (irq == msm_mpm_bypassed_apps_irqs[i])
			return true;

	return false;
}

static int msm_mpm_enable_irq_exclusive(
	unsigned int irq, bool enable, bool wakeset)
{
	uint32_t mpm_irq;

	if (!msm_mpm_is_valid_apps_irq(irq))
		return -EINVAL;

	if (msm_mpm_bypass_apps_irq(irq))
		return 0;

	mpm_irq = msm_mpm_get_irq_a2m(irq);
	if (mpm_irq) {
		uint32_t *mpm_irq_masks = wakeset ?
				msm_mpm_wake_irq : msm_mpm_enabled_irq;
		uint32_t index = MSM_MPM_IRQ_INDEX(mpm_irq);
		uint32_t mask = MSM_MPM_IRQ_MASK(mpm_irq);

		if (enable)
			mpm_irq_masks[index] |= mask;
		else
			mpm_irq_masks[index] &= ~mask;
	} else {
		unsigned long *apps_irq_bitmap = wakeset ?
			msm_mpm_wake_apps_irqs : msm_mpm_enabled_apps_irqs;

		if (enable)
			__set_bit(irq, apps_irq_bitmap);
		else
			__clear_bit(irq, apps_irq_bitmap);
	}

	return 0;
}

static int msm_mpm_set_irq_type_exclusive(
	unsigned int irq, unsigned int flow_type)
{
	uint32_t mpm_irq;

	if (!msm_mpm_is_valid_apps_irq(irq))
		return -EINVAL;

	if (msm_mpm_bypass_apps_irq(irq))
		return 0;

	mpm_irq = msm_mpm_get_irq_a2m(irq);
	if (mpm_irq) {
		uint32_t index = MSM_MPM_IRQ_INDEX(mpm_irq);
		uint32_t mask = MSM_MPM_IRQ_MASK(mpm_irq);

		if (flow_type & IRQ_TYPE_EDGE_BOTH)
			msm_mpm_detect_ctl[index] |= mask;
		else
			msm_mpm_detect_ctl[index] &= ~mask;

		if (flow_type & (IRQ_TYPE_EDGE_RISING | IRQ_TYPE_LEVEL_HIGH))
			msm_mpm_polarity[index] |= mask;
		else
			msm_mpm_polarity[index] &= ~mask;
	}

	return 0;
}

/******************************************************************************
 * Public functions
 *****************************************************************************/

int msm_mpm_is_app_irq_wakeup_capable(unsigned int irq) 
{
	return msm_mpm_get_irq_a2m(irq);
}

int msm_mpm_enable_irq(unsigned int irq, unsigned int enable)
{
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&msm_mpm_lock, flags);
	rc = msm_mpm_enable_irq_exclusive(irq, (bool)enable, false);
	spin_unlock_irqrestore(&msm_mpm_lock, flags);

	return rc;
}

int msm_mpm_set_irq_wake(unsigned int irq, unsigned int on)
{
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&msm_mpm_lock, flags);
	rc = msm_mpm_enable_irq_exclusive(irq, (bool)on, true);
	spin_unlock_irqrestore(&msm_mpm_lock, flags);

	return rc;
}

int msm_mpm_set_irq_type(unsigned int irq, unsigned int flow_type)
{
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&msm_mpm_lock, flags);
	rc = msm_mpm_set_irq_type_exclusive(irq, flow_type);
	spin_unlock_irqrestore(&msm_mpm_lock, flags);

	return rc;
}

int msm_mpm_enable_pin(enum msm_mpm_pin pin, unsigned int enable)
{
	uint32_t index = MSM_MPM_IRQ_INDEX(pin);
	uint32_t mask = MSM_MPM_IRQ_MASK(pin);
	unsigned long flags;

	spin_lock_irqsave(&msm_mpm_lock, flags);

	if (enable)
		msm_mpm_enabled_irq[index] |= mask;
	else
		msm_mpm_enabled_irq[index] &= ~mask;

	spin_unlock_irqrestore(&msm_mpm_lock, flags);
	return 0;
}

int msm_mpm_set_pin_wake(enum msm_mpm_pin pin, unsigned int on)
{
	uint32_t index = MSM_MPM_IRQ_INDEX(pin);
	uint32_t mask = MSM_MPM_IRQ_MASK(pin);
	unsigned long flags;

	spin_lock_irqsave(&msm_mpm_lock, flags);

	if (on)
		msm_mpm_wake_irq[index] |= mask;
	else
		msm_mpm_wake_irq[index] &= ~mask;

	spin_unlock_irqrestore(&msm_mpm_lock, flags);
	return 0;
}

int msm_mpm_set_pin_type(enum msm_mpm_pin pin, unsigned int flow_type)
{
	uint32_t index = MSM_MPM_IRQ_INDEX(pin);
	uint32_t mask = MSM_MPM_IRQ_MASK(pin);
	unsigned long flags;

	spin_lock_irqsave(&msm_mpm_lock, flags);

	if (flow_type & IRQ_TYPE_EDGE_BOTH)
		msm_mpm_detect_ctl[index] |= mask;
	else
		msm_mpm_detect_ctl[index] &= ~mask;

	if (flow_type & (IRQ_TYPE_EDGE_RISING | IRQ_TYPE_LEVEL_HIGH))
		msm_mpm_polarity[index] |= mask;
	else
		msm_mpm_polarity[index] &= ~mask;

	spin_unlock_irqrestore(&msm_mpm_lock, flags);
	return 0;
}

bool msm_mpm_irqs_detectable(bool from_idle)
{
	unsigned long *apps_irq_bitmap = from_idle ?
			msm_mpm_enabled_apps_irqs : msm_mpm_wake_apps_irqs;

	if (MSM_MPM_DEBUG_NON_DETECTABLE_IRQ & msm_mpm_debug_mask) {
		static char buf[DIV_ROUND_UP(MSM_MPM_NR_APPS_IRQS, 32)*9+1];

		bitmap_scnprintf(buf, sizeof(buf), apps_irq_bitmap,
				MSM_MPM_NR_APPS_IRQS);
		buf[sizeof(buf) - 1] = '\0';

		pr_info("%s: cannot monitor %s", __func__, buf);
	}

	return (bool)__bitmap_empty(apps_irq_bitmap, MSM_MPM_NR_APPS_IRQS);
}

bool msm_mpm_gic_irq_enabled(bool from_idle)
{
	unsigned long *apps_irq_bitmap = from_idle ?
			msm_mpm_enabled_apps_irqs : msm_mpm_wake_apps_irqs;

	return __bitmap_intersects(msm_mpm_gic_irqs_mask, apps_irq_bitmap,
			MSM_MPM_NR_APPS_IRQS);
}

bool msm_mpm_gpio_irq_enabled(bool from_idle)
{
	unsigned long *apps_irq_bitmap = from_idle ?
			msm_mpm_enabled_apps_irqs : msm_mpm_wake_apps_irqs;

	return __bitmap_intersects(msm_mpm_gpio_irqs_mask, apps_irq_bitmap,
			MSM_MPM_NR_APPS_IRQS);
}

void msm_mpm_enter_sleep(bool from_idle)
{
	msm_mpm_set(!from_idle);
}

void msm_mpm_exit_sleep(bool from_idle)
{
	unsigned long pending;
	int i;
	int k;

	for (i = 0; i < MSM_MPM_REG_WIDTH; i++) {
		pending = msm_mpm_read(MSM_MPM_STATUS_REG_PENDING, i);

		if (MSM_MPM_DEBUG_PENDING_IRQ & msm_mpm_debug_mask)
			pr_info("%s: pending.%d: 0x%08lx", __func__,
				i, pending);

		k = find_first_bit(&pending, 32);
		while (k < 32) {
			unsigned int mpm_irq = 32 * i + k;
			unsigned int apps_irq = msm_mpm_get_irq_m2a(mpm_irq);
			struct irq_desc *desc = apps_irq ?
						irq_to_desc(apps_irq) : NULL;

			/*
			 * This function is called when only CPU 0 is
			 * running and when both preemption and irqs
			 * are disabled.  There is no need to lock desc.
			 */
			if (desc && (desc->status & IRQ_TYPE_EDGE_BOTH)) {
				desc->status |= IRQ_PENDING;
				if (from_idle)
					check_irq_resend(desc, apps_irq);
			}

			k = find_next_bit(&pending, 32, k + 1);
		}
	}

	msm_mpm_set(!from_idle);
}

static int __init msm_mpm_early_init(void)
{
	uint8_t mpm_irq;
	uint16_t apps_irq;

	for (mpm_irq = 0; msm_mpm_is_valid_mpm_irq(mpm_irq); mpm_irq++) {
		apps_irq = msm_mpm_get_irq_m2a(mpm_irq);
		if (apps_irq && msm_mpm_is_valid_apps_irq(apps_irq))
			msm_mpm_set_irq_a2m(apps_irq, mpm_irq);
	}

	return 0;
}

static int __init msm_mpm_init(void)
{
	unsigned int irq = MSM_MPM_IPC_IRQ;
	int rc;

	bitmap_set(msm_mpm_gic_irqs_mask, 0, NR_MSM_IRQS - 1);
	bitmap_set(msm_mpm_gpio_irqs_mask, NR_MSM_IRQS,
			MSM_MPM_NR_APPS_IRQS - 1);

	rc = request_irq(irq, msm_mpm_irq,
			IRQF_TRIGGER_RISING, "mpm_drv", msm_mpm_irq);

	if (rc) {
		pr_err("%s: failed to request irq %u: %d\n",
			__func__, irq, rc);
		goto init_bail;
	}

	rc = set_irq_wake(irq, 1);
	if (rc) {
		pr_err("%s: failed to set wakeup irq %u: %d\n",
			__func__, irq, rc);
		goto init_free_bail;
	}

	return 0;

init_free_bail:
	free_irq(irq, msm_mpm_irq);

init_bail:
	return rc;
}

core_initcall(msm_mpm_early_init);
device_initcall(msm_mpm_init);
