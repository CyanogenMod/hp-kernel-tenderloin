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

#include <linux/platform_device.h>
#include <asm/pmu.h>
#include <mach/irqs.h>

static struct resource pmu_resource = {
	.start = INT_ARMQC_PERFMON,
	.end = INT_ARMQC_PERFMON,
	.flags	= IORESOURCE_IRQ,
};

static struct platform_device pmu_device = {
	.name		= "arm-pmu",
	.id		= ARM_PMU_DEVICE_CPU,
	.resource	= &pmu_resource,
	.num_resources	= 1,
};

static int __init scorpion_pmu_init(void)
{
	platform_device_register(&pmu_device);
	printk(KERN_INFO "Scorpion registered PMU device\n");
	return 0;
}

arch_initcall(scorpion_pmu_init);
