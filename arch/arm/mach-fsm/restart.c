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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/reboot.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/mfd/pmic8058.h>

#include <mach/msm_iomap.h>

#define TCSR_BASE 0x16B00000
#define TCSR_WDT_CFG 0x30

#define WDT0_RST       (MSM_TMR_BASE + 0x38)
#define WDT0_EN        (MSM_TMR_BASE + 0x40)
#define WDT0_BARK_TIME (MSM_TMR_BASE + 0x4C)

#define PSHOLD_CTL_SU (MSM_TLMM_BASE + 0x820)

static void *tcsr_base;

static void msm_power_off(void)
{
	printk(KERN_NOTICE "Powering off the SoC\n");
	pm8058_reset_pwr_off(0);
	writel(0, PSHOLD_CTL_SU);
	mdelay(10000);
	printk(KERN_ERR "Powering off has failed\n");
	return;
}

static void msm_restart(char str, const char *cmd)
{
	printk(KERN_NOTICE "Going down for restart now\n");
	writel(1, WDT0_RST);
	writel(0, WDT0_EN);
	writel(0x31F3, WDT0_BARK_TIME);
	writel(3, WDT0_EN);
	dmb();
	if (tcsr_base != NULL)
		writel(3, tcsr_base + TCSR_WDT_CFG);

	mdelay(10000);
	printk(KERN_ERR "Restarting has failed\n");
	return;
}

static int __init msm_restart_init(void)
{
	pm_power_off = msm_power_off;
	arm_pm_restart = msm_restart;

	tcsr_base = ioremap_nocache(TCSR_BASE, SZ_4K);
	if (tcsr_base == NULL)
		return -ENOMEM;
	return 0;
}

late_initcall(msm_restart_init);
