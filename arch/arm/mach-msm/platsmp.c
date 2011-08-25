/*
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *  Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>

#include <asm/hardware/gic.h>
#include <asm/cacheflush.h>
#include <asm/mach-types.h>

#include <mach/smp.h>
#include <mach/hardware.h>
#include <mach/msm_iomap.h>

#include "pm.h"
#include "scm-boot.h"

#define SECONDARY_CPU_WAIT_MS 10

int pen_release = -1;

int get_core_count(void)
{
#ifdef CONFIG_NR_CPUS
	return CONFIG_NR_CPUS;
#else
	return 1;
#endif
}

/* Initialize the present map (cpu_set(i, cpu_present_map)). */
void smp_prepare_cpus(unsigned int max_cpus)
{
	int i;
	unsigned int cpu = smp_processor_id();

	smp_store_cpu_info(cpu);

	for (i = 0; i < max_cpus; i++)
		cpu_set(i, cpu_present_map);
}

void smp_init_cpus(void)
{
	unsigned int i, ncores = get_core_count();

	for (i = 0; i < ncores; i++)
		cpu_set(i, cpu_possible_map);
}

/* Executed by primary CPU, brings other CPUs out of reset. Called at boot
   as well as when a CPU is coming out of shutdown induced by echo 0 >
   /sys/devices/.../cpuX.
*/
int boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	static int cold_boot_done;
	int cnt = 0;
	int ret;

	pr_debug("Starting secondary CPU %d\n", cpu);

	/* Set preset_lpj to avoid subsequent lpj recalculations */
	preset_lpj = loops_per_jiffy;

	if (cold_boot_done == false) {
		ret = scm_set_boot_addr((void *)
					virt_to_phys(msm_secondary_startup),
					SCM_FLAG_COLDBOOT_CPU1);
		if (ret == 0) {
			void *sc1_base_ptr;
			sc1_base_ptr = ioremap_nocache(0x00902000, SZ_4K*2);
			if (sc1_base_ptr) {
				writel(0x0, sc1_base_ptr+0x15A0);
				writel(0x0, sc1_base_ptr+0xD80);
				writel(0x3, sc1_base_ptr+0xE64);
				iounmap(sc1_base_ptr);
			}
		} else
			printk(KERN_DEBUG "Failed to set secondary core boot "
					  "address\n");
		cold_boot_done = true;
	}

	pen_release = cpu;
	dmac_flush_range((void *)&pen_release,
			 (void *)(&pen_release + sizeof(pen_release)));
	__asm__("sev");
	dsb();

	/* Use smp_cross_call() to send a soft interrupt to wake up
	 * the other core.
	 */
	smp_cross_call(cpumask_of(cpu));

	while (pen_release != 0xFFFFFFFF) {
		dmac_inv_range((void *)&pen_release,
			       (void *)(&pen_release+sizeof(pen_release)));
		msleep_interruptible(1);
		if (cnt++ >= SECONDARY_CPU_WAIT_MS)
			break;
	}

	return 0;
}

/* Initialization routine for secondary CPUs after they are brought out of
 * reset.
*/
void platform_secondary_init(unsigned int cpu)
{
	pr_debug("CPU%u: Booted secondary processor\n", cpu);

#ifdef CONFIG_HOTPLUG_CPU
	WARN_ON(msm_pm_platform_secondary_init(cpu));
#endif

	trace_hardirqs_off();

	/* Edge trigger PPIs except AVS_SVICINT and AVS_SVICINTSWDONE */
	writel(0xFFFFD7FF, MSM_QGIC_DIST_BASE + GIC_DIST_CONFIG + 4);

	/* RUMI does not adhere to GIC spec by enabling STIs by default.
	 * Enable/clear is supposed to be RO for STIs, but is RW on RUMI.
	 */
	if (!machine_is_msm8x60_sim())
		writel(0x0000FFFF, MSM_QGIC_DIST_BASE + GIC_DIST_ENABLE_SET);

	/*
	 * setup GIC (GIC number NOT CPU number and the base address of the
	 * GIC CPU interface
	 */
	gic_cpu_init(0, MSM_QGIC_CPU_BASE);
}
