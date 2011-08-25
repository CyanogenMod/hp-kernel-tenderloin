/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

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
#include <linux/init.h>
#include <linux/sysdev.h>
#include <linux/suspend.h>
#include <asm/hardware/gic.h>
#include <linux/mfd/pmic8058.h>
#include <mach/gpio.h>

/******************************************************************************
 * Debug Definitions
 *****************************************************************************/

enum {
	MSM_SHOW_IRQ_DEBUG_RESUME = BIT(0),
};

static int msm_show_resume_irq_mask = MSM_SHOW_IRQ_DEBUG_RESUME;
module_param_named(
	debug_mask, msm_show_resume_irq_mask, int, S_IRUGO | S_IWUSR | S_IWGRP
);

static int msm_show_resume_irq_resume(struct sys_device *dev)
{
	if (msm_show_resume_irq_mask & MSM_SHOW_IRQ_DEBUG_RESUME) {
		gic_show_resume_irq(0);
		msm_gpio_show_resume_irq();
		pm8058_show_resume_irq();
	}
	return 0;
}

static struct sysdev_class msm_show_resume_irq_sysclass = {
	.name = "msm_show_resume_irq",
	.resume = msm_show_resume_irq_resume,
};

static struct sys_device msm_show_resume_irq_device = {
	.cls = &msm_show_resume_irq_sysclass,
};

static int __init msm_show_resume_irq_init(void)
{
	int ret = -ENODEV;

	ret = sysdev_class_register(&msm_show_resume_irq_sysclass);
	if (ret == 0)
		ret = sysdev_register(&msm_show_resume_irq_device);
	return ret;
}

static void __exit msm_show_resume_irq_exit(void)
{
	sysdev_register(&msm_show_resume_irq_device);
	sysdev_class_register(&msm_show_resume_irq_sysclass);
}

module_init(msm_show_resume_irq_init);
module_exit(msm_show_resume_irq_exit);
