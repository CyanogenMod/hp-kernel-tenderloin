/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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
#include <linux/completion.h>
#include <linux/cpuidle.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/pm.h>
#include <linux/pm_qos_params.h>
#include <linux/proc_fs.h>
#include <linux/smp.h>
#include <linux/suspend.h>
#include <linux/tick.h>
#include <linux/uaccess.h>
#include <linux/wakelock.h>
#include <mach/msm_iomap.h>
#include <mach/system.h>
#include <asm/cacheflush.h>
#include <asm/hardware/gic.h>
#include <asm/pgtable.h>
#include <asm/pgalloc.h>
#include <mach/gpio.h>
#ifdef CONFIG_VFP
#include <asm/vfp.h>
#endif

#include "acpuclock.h"
#include "clock.h"
#include "avs.h"
#include "cpuidle.h"
#include "idle.h"
#include "pm.h"
#include "rpm_resources.h"
#include "scm-boot.h"
#include "spm.h"
#include "timer.h"
#include "scm-boot.h"

/******************************************************************************
 * Debug Definitions
 *****************************************************************************/

enum {
	MSM_PM_DEBUG_SUSPEND = BIT(0),
	MSM_PM_DEBUG_POWER_COLLAPSE = BIT(1),
	MSM_PM_DEBUG_SUSPEND_LIMITS = BIT(2),
	MSM_PM_DEBUG_CLOCK = BIT(3),
	MSM_PM_DEBUG_RESET_VECTOR = BIT(4),
	MSM_PM_DEBUG_IDLE = BIT(6),
	MSM_PM_DEBUG_IDLE_LIMITS = BIT(7),
	MSM_PM_DEBUG_HOTPLUG = BIT(8),
	MSM_PM_DEBUG_SUSPEND_PINS = BIT(9),
};

static int msm_pm_debug_mask = 0;
module_param_named(
	debug_mask, msm_pm_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP
);

#ifdef CONFIG_DEBUG_TZ_COUNTERS
#define IMEM_DEBUG_LOC 0x2a05f028
void *vik_regsave, *imem_debug_loc, *tz_counters;
#endif

/******************************************************************************
 * Sleep Modes and Parameters
 *****************************************************************************/

static struct msm_pm_platform_data *msm_pm_modes;

void __init msm_pm_set_platform_data(
	struct msm_pm_platform_data *data, int count)
{
	BUG_ON(MSM_PM_SLEEP_MODE_NR * num_possible_cpus() > count);
	msm_pm_modes = data;
}

#define MSM_PM_MODE_ATTR_SUSPEND_ENABLED "suspend_enabled"
#define MSM_PM_MODE_ATTR_IDLE_ENABLED "idle_enabled"
#define MSM_PM_MODE_ATTR_NR (2)

struct msm_pm_kobj_attribute {
	unsigned int cpu;
	struct kobj_attribute ka;
};

#define GET_CPU_OF_ATTR(attr) \
	(container_of(attr, struct msm_pm_kobj_attribute, ka)->cpu)

struct msm_pm_sysfs_sleep_mode {
	struct kobject *kobj;
	struct attribute_group attr_group;
	struct attribute *attrs[MSM_PM_MODE_ATTR_NR + 1];
	struct msm_pm_kobj_attribute kas[MSM_PM_MODE_ATTR_NR];
};

static char *msm_pm_sleep_mode_labels[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] = "power_collapse",
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT] = "wfi",
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE] =
		"standalone_power_collapse",
};

/*
 * Write out the attribute.
 */
static ssize_t msm_pm_mode_attr_show(
	struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = -EINVAL;
	int i;

	for (i = 0; i < MSM_PM_SLEEP_MODE_NR; i++) {
		struct kernel_param kp;
		unsigned int cpu;
		struct msm_pm_platform_data *mode;

		if (msm_pm_sleep_mode_labels[i] == NULL)
			continue;

		if (strcmp(kobj->name, msm_pm_sleep_mode_labels[i]))
			continue;

		cpu = GET_CPU_OF_ATTR(attr);
		mode = &msm_pm_modes[MSM_PM_MODE(cpu, i)];

		if (!strcmp(attr->attr.name,
			MSM_PM_MODE_ATTR_SUSPEND_ENABLED)) {
			u32 arg = mode->suspend_enabled;
			kp.arg = &arg;
			ret = param_get_ulong(buf, &kp);
		} else if (!strcmp(attr->attr.name,
			MSM_PM_MODE_ATTR_IDLE_ENABLED)) {
			u32 arg = mode->idle_enabled;
			kp.arg = &arg;
			ret = param_get_ulong(buf, &kp);
		}

		break;
	}

	if (ret > 0) {
		strcat(buf, "\n");
		ret++;
	}

	return ret;
}

/*
 * Read in the new attribute value.
 */
static ssize_t msm_pm_mode_attr_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret = -EINVAL;
	int i;

	for (i = 0; i < MSM_PM_SLEEP_MODE_NR; i++) {
		struct kernel_param kp;
		unsigned int cpu;
		struct msm_pm_platform_data *mode;

		if (msm_pm_sleep_mode_labels[i] == NULL)
			continue;

		if (strcmp(kobj->name, msm_pm_sleep_mode_labels[i]))
			continue;

		cpu = GET_CPU_OF_ATTR(attr);
		mode = &msm_pm_modes[MSM_PM_MODE(cpu, i)];

		if (!strcmp(attr->attr.name,
			MSM_PM_MODE_ATTR_SUSPEND_ENABLED)) {
			kp.arg = &mode->suspend_enabled;
			ret = param_set_byte(buf, &kp);
		} else if (!strcmp(attr->attr.name,
			MSM_PM_MODE_ATTR_IDLE_ENABLED)) {
			kp.arg = &mode->idle_enabled;
			ret = param_set_byte(buf, &kp);
		}

		break;
	}

	return ret ? ret : count;
}

/*
 * Add sysfs entries for one cpu.
 */
static int __init msm_pm_mode_sysfs_add_cpu(
	unsigned int cpu, struct kobject *modes_kobj)
{
	char cpu_name[8];
	struct kobject *cpu_kobj;
	struct msm_pm_sysfs_sleep_mode *mode;
	int i, k;
	int ret;

	snprintf(cpu_name, sizeof(cpu_name), "cpu%u", cpu);
	cpu_kobj = kobject_create_and_add(cpu_name, modes_kobj);
	if (!cpu_kobj) {
		pr_err("%s: cannot create %s kobject\n", __func__, cpu_name);
		ret = -ENOMEM;
		goto mode_sysfs_add_cpu_exit;
	}

	for (i = 0; i < MSM_PM_SLEEP_MODE_NR; i++) {
		if (!msm_pm_modes[MSM_PM_MODE(cpu, i)].supported)
			continue;

		mode = kzalloc(sizeof(*mode), GFP_KERNEL);
		if (!mode) {
			pr_err("%s: cannot allocate memory for attributes\n",
				__func__);
			ret = -ENOMEM;
			goto mode_sysfs_add_cpu_exit;
		}

		mode->kobj = kobject_create_and_add(
				msm_pm_sleep_mode_labels[i], cpu_kobj);
		if (!mode->kobj) {
			pr_err("%s: cannot create kobject\n", __func__);
			ret = -ENOMEM;
			goto mode_sysfs_add_cpu_exit;
		}

		mode->kas[0].ka.attr.name = MSM_PM_MODE_ATTR_SUSPEND_ENABLED;
		mode->kas[1].ka.attr.name = MSM_PM_MODE_ATTR_IDLE_ENABLED;

		for (k = 0; k < MSM_PM_MODE_ATTR_NR; k++) {
			mode->kas[k].cpu = cpu;
			mode->kas[k].ka.attr.mode = 0644;
			mode->kas[k].ka.show = msm_pm_mode_attr_show;
			mode->kas[k].ka.store = msm_pm_mode_attr_store;

			mode->attrs[k] = &mode->kas[k].ka.attr;
		}
		mode->attrs[MSM_PM_MODE_ATTR_NR] = NULL;

		mode->attr_group.attrs = mode->attrs;
		ret = sysfs_create_group(mode->kobj, &mode->attr_group);
		if (ret) {
			pr_err("%s: cannot create kobject attribute group\n",
				__func__);
			goto mode_sysfs_add_cpu_exit;
		}
	}

	ret = 0;

mode_sysfs_add_cpu_exit:
	return ret;
}

/*
 * Add sysfs entries for the sleep modes.
 */
static int __init msm_pm_mode_sysfs_add(void)
{
	struct kobject *module_kobj;
	struct kobject *modes_kobj;
	unsigned int cpu;
	int ret;

	module_kobj = kset_find_obj(module_kset, KBUILD_MODNAME);
	if (!module_kobj) {
		pr_err("%s: cannot find kobject for module %s\n",
			__func__, KBUILD_MODNAME);
		ret = -ENOENT;
		goto mode_sysfs_add_exit;
	}

	modes_kobj = kobject_create_and_add("modes", module_kobj);
	if (!modes_kobj) {
		pr_err("%s: cannot create modes kobject\n", __func__);
		ret = -ENOMEM;
		goto mode_sysfs_add_exit;
	}

	for_each_possible_cpu(cpu) {
		ret = msm_pm_mode_sysfs_add_cpu(cpu, modes_kobj);
		if (ret)
			goto mode_sysfs_add_exit;
	}

	ret = 0;

mode_sysfs_add_exit:
	return ret;
}

/******************************************************************************
 * CONFIG_MSM_IDLE_STATS
 *****************************************************************************/

#ifdef CONFIG_MSM_IDLE_STATS
enum msm_pm_time_stats_id {
	MSM_PM_STAT_REQUESTED_IDLE,
	MSM_PM_STAT_IDLE_WFI,
	MSM_PM_STAT_IDLE_STANDALONE_POWER_COLLAPSE,
	MSM_PM_STAT_IDLE_POWER_COLLAPSE,
	MSM_PM_STAT_SUSPEND,
	MSM_PM_STAT_COUNT
};

struct msm_pm_time_stats {
	const char *name;
	int64_t first_bucket_time;
	int bucket[CONFIG_MSM_IDLE_STATS_BUCKET_COUNT];
	int64_t min_time[CONFIG_MSM_IDLE_STATS_BUCKET_COUNT];
	int64_t max_time[CONFIG_MSM_IDLE_STATS_BUCKET_COUNT];
	int count;
	int64_t total_time;
};

struct msm_pm_cpu_time_stats {
	struct msm_pm_time_stats stats[MSM_PM_STAT_COUNT];
};

static DEFINE_SPINLOCK(msm_pm_stats_lock);
static DEFINE_PER_CPU_SHARED_ALIGNED(
	struct msm_pm_cpu_time_stats, msm_pm_stats);

/*
 * Add the given time data to the statistics collection.
 */
static void msm_pm_add_stat(enum msm_pm_time_stats_id id, int64_t t)
{
	unsigned long flags;
	struct msm_pm_time_stats *stats;
	int64_t bt;
	int i;

	spin_lock_irqsave(&msm_pm_stats_lock, flags);
	stats = __get_cpu_var(msm_pm_stats).stats;

	stats[id].total_time += t;
	stats[id].count++;

	bt = t;
	do_div(bt, stats[id].first_bucket_time);

	if (bt < 1ULL << (CONFIG_MSM_IDLE_STATS_BUCKET_SHIFT *
				(CONFIG_MSM_IDLE_STATS_BUCKET_COUNT - 1)))
		i = DIV_ROUND_UP(fls((uint32_t)bt),
					CONFIG_MSM_IDLE_STATS_BUCKET_SHIFT);
	else
		i = CONFIG_MSM_IDLE_STATS_BUCKET_COUNT - 1;

	stats[id].bucket[i]++;

	if (t < stats[id].min_time[i] || !stats[id].max_time[i])
		stats[id].min_time[i] = t;
	if (t > stats[id].max_time[i])
		stats[id].max_time[i] = t;

	spin_unlock_irqrestore(&msm_pm_stats_lock, flags);
}

/*
 * Helper function of snprintf where buf is auto-incremented, size is auto-
 * decremented, and there is no return value.
 *
 * NOTE: buf and size must be l-values (e.g. variables)
 */
#define SNPRINTF(buf, size, format, ...) \
	do { \
		if (size > 0) { \
			int ret; \
			ret = snprintf(buf, size, format, ## __VA_ARGS__); \
			if (ret > size) { \
				buf += size; \
				size = 0; \
			} else { \
				buf += ret; \
				size -= ret; \
			} \
		} \
	} while (0)

/*
 * Write out the power management statistics.
 */
static int msm_pm_read_proc
	(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	unsigned int cpu = off / MSM_PM_STAT_COUNT;
	int id = off % MSM_PM_STAT_COUNT;
	char *p = page;

	if (count < 1024) {
		*start = (char *) 0;
		*eof = 0;
		return 0;
	}

	if (cpu < num_possible_cpus()) {
		unsigned long flags;
		struct msm_pm_time_stats *stats;
		int i;
		int64_t bucket_time;
		int64_t s;
		uint32_t ns;

		spin_lock_irqsave(&msm_pm_stats_lock, flags);
		stats = per_cpu(msm_pm_stats, cpu).stats;

		s = stats[id].total_time;
		ns = do_div(s, NSEC_PER_SEC);
		SNPRINTF(p, count,
			"[cpu %u] %s:\n"
			"  count: %7d\n"
			"  total_time: %lld.%09u\n",
			cpu, stats[id].name,
			stats[id].count,
			s, ns);

		bucket_time = stats[id].first_bucket_time;
		for (i = 0; i < CONFIG_MSM_IDLE_STATS_BUCKET_COUNT - 1; i++) {
			s = bucket_time;
			ns = do_div(s, NSEC_PER_SEC);
			SNPRINTF(p, count,
				"   <%6lld.%09u: %7d (%lld-%lld)\n",
				s, ns, stats[id].bucket[i],
				stats[id].min_time[i],
				stats[id].max_time[i]);

			bucket_time <<= CONFIG_MSM_IDLE_STATS_BUCKET_SHIFT;
		}

		SNPRINTF(p, count, "  >=%6lld.%09u: %7d (%lld-%lld)\n",
			s, ns, stats[id].bucket[i],
			stats[id].min_time[i],
			stats[id].max_time[i]);

		*start = (char *) 1;
		*eof = (off + 1 >= MSM_PM_STAT_COUNT * num_possible_cpus());

		spin_unlock_irqrestore(&msm_pm_stats_lock, flags);
	}

	return p - page;
}
#undef SNPRINTF

#define MSM_PM_STATS_RESET "reset"

/*
 * Reset the power management statistics values.
 */
static int msm_pm_write_proc(struct file *file, const char __user *buffer,
	unsigned long count, void *data)
{
	char buf[sizeof(MSM_PM_STATS_RESET)];
	int ret;
	unsigned long flags;
	unsigned int cpu;

	if (count < strlen(MSM_PM_STATS_RESET)) {
		ret = -EINVAL;
		goto write_proc_failed;
	}

	if (copy_from_user(buf, buffer, strlen(MSM_PM_STATS_RESET))) {
		ret = -EFAULT;
		goto write_proc_failed;
	}

	if (memcmp(buf, MSM_PM_STATS_RESET, strlen(MSM_PM_STATS_RESET))) {
		ret = -EINVAL;
		goto write_proc_failed;
	}

	spin_lock_irqsave(&msm_pm_stats_lock, flags);
	for_each_possible_cpu(cpu) {
		struct msm_pm_time_stats *stats;
		int i;

		stats = per_cpu(msm_pm_stats, cpu).stats;
		for (i = 0; i < MSM_PM_STAT_COUNT; i++) {
			memset(stats[i].bucket,
				0, sizeof(stats[i].bucket));
			memset(stats[i].min_time,
				0, sizeof(stats[i].min_time));
			memset(stats[i].max_time,
				0, sizeof(stats[i].max_time));
			stats[i].count = 0;
			stats[i].total_time = 0;
		}
	}

	spin_unlock_irqrestore(&msm_pm_stats_lock, flags);
	return count;

write_proc_failed:
	return ret;
}
#undef MSM_PM_STATS_RESET
#endif /* CONFIG_MSM_IDLE_STATS */


/******************************************************************************
 * Configure Hardware before/after Low Power Mode
 *****************************************************************************/

/*
 * Configure hardware registers in preparation for Apps power down.
 */
static void msm_pm_config_hw_before_power_down(void)
{
	return;
}

/*
 * Clear hardware registers after Apps powers up.
 */
static void msm_pm_config_hw_after_power_up(void)
{
	return;
}

/*
 * Configure hardware registers in preparation for SWFI.
 */
static void msm_pm_config_hw_before_swfi(void)
{
	return;
}


/******************************************************************************
 * Suspend Max Sleep Time
 *****************************************************************************/

#ifdef CONFIG_MSM_SLEEP_TIME_OVERRIDE
static int msm_pm_sleep_time_override;
module_param_named(sleep_time_override,
	msm_pm_sleep_time_override, int, S_IRUGO | S_IWUSR | S_IWGRP);
#endif

#define SCLK_HZ (32768)
#define MSM_PM_SLEEP_TICK_LIMIT (0x6DDD000)

static uint32_t msm_pm_max_sleep_time;

/*
 * Convert time from nanoseconds to slow clock ticks, then cap it to the
 * specified limit
 */
static int64_t msm_pm_convert_and_cap_time(int64_t time_ns, int64_t limit)
{
	do_div(time_ns, NSEC_PER_SEC / SCLK_HZ);
	return (time_ns > limit) ? limit : time_ns;
}

/*
 * Set the sleep time for suspend.  0 means infinite sleep time.
 */
void msm_pm_set_max_sleep_time(int64_t max_sleep_time_ns)
{
	if (max_sleep_time_ns == 0) {
		msm_pm_max_sleep_time = 0;
	} else {
		msm_pm_max_sleep_time = (uint32_t)msm_pm_convert_and_cap_time(
			max_sleep_time_ns, MSM_PM_SLEEP_TICK_LIMIT);

		if (msm_pm_max_sleep_time == 0)
			msm_pm_max_sleep_time = 1;
	}

	if (msm_pm_debug_mask & MSM_PM_DEBUG_SUSPEND)
		pr_info("%s: Requested %lld ns Giving %u sclk ticks\n",
			__func__, max_sleep_time_ns, msm_pm_max_sleep_time);
}
EXPORT_SYMBOL(msm_pm_set_max_sleep_time);


/******************************************************************************
 *
 *****************************************************************************/

struct msm_pm_device {
	unsigned int cpu;
#ifdef CONFIG_HOTPLUG_CPU
	unsigned long saved_acpu_rate;
	struct completion cpu_killed;
	unsigned int warm_boot;
#endif
};

static DEFINE_PER_CPU_SHARED_ALIGNED(struct msm_pm_device, msm_pm_devices);
static struct msm_rpmrs_limits *msm_pm_idle_rs_limits;

static void msm_pm_swfi(void)
{
	msm_pm_config_hw_before_swfi();
	msm_arch_idle();
}

static void msm_pm_spm_power_collapse(
	struct msm_pm_device *dev, bool from_idle, bool notify_rpm)
{
	void *entry;
	int collapsed = 0;
	int ret;

	if (MSM_PM_DEBUG_POWER_COLLAPSE & msm_pm_debug_mask)
		pr_info("CPU%u: %s: notify_rpm %d\n",
			dev->cpu, __func__, (int) notify_rpm);

	ret = msm_spm_set_low_power_mode(
			MSM_SPM_MODE_POWER_COLLAPSE, notify_rpm);
	WARN_ON(ret);

	entry = (!dev->cpu || from_idle) ?
		msm_pm_collapse_exit : msm_secondary_startup;
	msm_pm_write_boot_vector(dev->cpu, virt_to_phys(entry));

#ifdef CONFIG_DEBUG_TZ_COUNTERS
	/* Write 0x20 to ddr to tell tz,rpm that we're about to PC */
	if (!dev->cpu) {
		*((unsigned long *)vik_regsave) = 0x20;
		writel(0x20, imem_debug_loc);


		if (!from_idle) {
			printk(KERN_WARNING "TZ says cpu0 Pc entry count: %d\n",
				readl(tz_counters));
			printk(KERN_WARNING "TZ says cpu1 Pc entry count: %d\n",
				readl(tz_counters+0xC));
			printk(KERN_WARNING "TZ says cpu0 Pc exit count: %d\n",
				readl(tz_counters+0x4));
			printk(KERN_WARNING "TZ says cpu1 Pc exit count: %d\n",
				readl(tz_counters+0x8));
		}
	}
#endif

	if (MSM_PM_DEBUG_RESET_VECTOR & msm_pm_debug_mask)
		pr_info("CPU%u: %s: program vector to %p\n",
			dev->cpu, __func__, entry);

#ifdef CONFIG_VFP
	vfp_flush_context();
#endif

	collapsed = msm_pm_collapse();

	if (collapsed) {
#ifdef CONFIG_VFP
		vfp_reinit();
#endif
		cpu_init();
		gic_cpu_init(0, MSM_QGIC_CPU_BASE);
		local_fiq_enable();
	}

#ifdef CONFIG_DEBUG_TZ_COUNTERS
	/* Add 0x10 to vik_regsave, to inform the world we've exited PC
	and are now in virt space. */
	if (!dev->cpu) {
		*((unsigned long *)vik_regsave) += 0x10;
		writel(0x30, imem_debug_loc);
	}
#endif

	if (MSM_PM_DEBUG_POWER_COLLAPSE & msm_pm_debug_mask)
		pr_info("CPU%u: %s: msm_pm_collapse returned, collapsed %d\n",
			dev->cpu, __func__, collapsed);

	ret = msm_spm_set_low_power_mode(MSM_SPM_MODE_CLOCK_GATING, false);
	WARN_ON(ret);
}

static void msm_pm_power_collapse_standalone(bool from_idle)
{
	struct msm_pm_device *dev = &__get_cpu_var(msm_pm_devices);
	unsigned int avsdscr_setting;

	avsdscr_setting = avs_get_avsdscr();
	avs_disable();
	msm_pm_spm_power_collapse(dev, from_idle, false);
	avs_reset_delays(avsdscr_setting);
}

#ifdef CONFIG_DEBUG_TZ_COUNTERS
static unsigned long debug_val = 0;
#endif

static void msm_pm_power_collapse(bool from_idle)
{
	struct msm_pm_device *dev = &__get_cpu_var(msm_pm_devices);
	unsigned long saved_acpuclk_rate;
	unsigned int avsdscr_setting;

	if (MSM_PM_DEBUG_POWER_COLLAPSE & msm_pm_debug_mask)
		pr_info("CPU%u: %s: idle %d\n",
			dev->cpu, __func__, (int)from_idle);

	msm_pm_config_hw_before_power_down();
	if (MSM_PM_DEBUG_POWER_COLLAPSE & msm_pm_debug_mask)
		pr_info("CPU%u: %s: pre power down\n", dev->cpu, __func__);

	avsdscr_setting = avs_get_avsdscr();
	avs_disable();

	if (cpu_online(dev->cpu))
		saved_acpuclk_rate = acpuclk_power_collapse();
	else
		saved_acpuclk_rate = 0;

	if (MSM_PM_DEBUG_CLOCK & msm_pm_debug_mask)
		pr_info("CPU%u: %s: change clock rate (old rate = %lu)\n",
			dev->cpu, __func__, saved_acpuclk_rate);

	msm_pm_spm_power_collapse(dev, from_idle, true);

#ifdef CONFIG_DEBUG_TZ_COUNTERS
	/* Add 0x10 to vik_regsave to tell the world that we're out of PC
	and about to do set_rate */
	if (!dev->cpu) {
		*((unsigned long *)vik_regsave) += 0x10;
		debug_val = readl(imem_debug_loc);
		writel(debug_val+0x10, imem_debug_loc);
	}
#endif

	if (MSM_PM_DEBUG_CLOCK & msm_pm_debug_mask)
		pr_info("CPU%u: %s: restore clock rate to %lu\n",
			dev->cpu, __func__, saved_acpuclk_rate);
	if (acpuclk_set_rate(dev->cpu, saved_acpuclk_rate, SETRATE_PC) < 0)
		pr_err("CPU%u: %s: failed to restore clock rate(%lu)\n",
			dev->cpu, __func__, saved_acpuclk_rate);

#ifdef CONFIG_DEBUG_TZ_COUNTERS
	/* Add 0x10 to vik_regsave to tell the world that we're out of PC
	and we're done with set_rate */
	if (!dev->cpu) {
		*((unsigned long *)vik_regsave) += 0x10;
		debug_val = readl(imem_debug_loc);
		writel(debug_val+0x10, imem_debug_loc);
	}
#endif

	avs_reset_delays(avsdscr_setting);
	msm_pm_config_hw_after_power_up();
	if (MSM_PM_DEBUG_POWER_COLLAPSE & msm_pm_debug_mask)
		pr_info("CPU%u: %s: post power up\n", dev->cpu, __func__);

	if (MSM_PM_DEBUG_POWER_COLLAPSE & msm_pm_debug_mask)
		pr_info("CPU%u: %s: return\n", dev->cpu, __func__);
}

static irqreturn_t msm_pm_rpm_wakeup_interrupt(int irq, void *dev_id)
{
	if (dev_id != &msm_pm_rpm_wakeup_interrupt)
		return IRQ_NONE;

	return IRQ_HANDLED;
}


/******************************************************************************
 * External Idle/Suspend Functions
 *****************************************************************************/

void arch_idle(void)
{
	return;
}

int msm_pm_idle_prepare(struct cpuidle_device *dev)
{
	uint32_t latency_us;
	uint32_t sleep_us;
	int i;

	latency_us = (uint32_t) pm_qos_request(PM_QOS_CPU_DMA_LATENCY);
	sleep_us = (uint32_t) ktime_to_ns(tick_nohz_get_sleep_length());
	sleep_us = DIV_ROUND_UP(sleep_us, 1000);

	for (i = 0; i < dev->state_count; i++) {
		struct cpuidle_state *state = &dev->states[i];
		enum msm_pm_sleep_mode mode;
		bool allow;
		struct msm_rpmrs_limits *rs_limits = NULL;

		mode = (enum msm_pm_sleep_mode) state->driver_data;
		allow = msm_pm_modes[MSM_PM_MODE(dev->cpu, mode)].idle_enabled;

		switch (mode) {
		case MSM_PM_SLEEP_MODE_POWER_COLLAPSE:
			if (!allow)
				break;

			if (num_online_cpus() > 1) {
				allow = false;
				break;
			}
#ifdef CONFIG_HAS_WAKELOCK
			if (has_wake_lock(WAKE_LOCK_IDLE)) {
				allow = false;
				break;
			}
#endif
			/* fall through */

		case MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE:
			if (!allow)
				break;

			if (!dev->cpu &&
				msm_rpm_local_request_is_outstanding()) {
				allow = false;
				break;
			}
			/* fall through */

		case MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT:
			if (!allow)
				break;

			rs_limits = msm_rpmrs_lowest_limits(true,
						mode, latency_us, sleep_us);

			if (MSM_PM_DEBUG_IDLE & msm_pm_debug_mask)
				pr_info("CPU%u: %s: %s, latency %uus, "
					"sleep %uus, limit %p\n",
					dev->cpu, __func__, state->desc,
					latency_us, sleep_us, rs_limits);

			if ((MSM_PM_DEBUG_IDLE_LIMITS & msm_pm_debug_mask) &&
					rs_limits)
				pr_info("CPU%u: %s: limit %p: "
					"pxo %d, l2_cache %d, "
					"vdd_mem %d, vdd_dig %d\n",
					dev->cpu, __func__, rs_limits,
					rs_limits->pxo,
					rs_limits->l2_cache,
					rs_limits->vdd_mem,
					rs_limits->vdd_dig);

			if (!rs_limits)
				allow = false;
			break;

		default:
			allow = false;
			break;
		}

		if (MSM_PM_DEBUG_IDLE & msm_pm_debug_mask)
			pr_info("CPU%u: %s: allow %s: %d\n",
				dev->cpu, __func__, state->desc, (int)allow);

		if (allow) {
			state->flags &= ~CPUIDLE_FLAG_IGNORE;
			state->target_residency = 0;
			state->exit_latency = 0;
			state->power_usage = rs_limits->power[dev->cpu];

			if (MSM_PM_SLEEP_MODE_POWER_COLLAPSE == mode)
				msm_pm_idle_rs_limits = rs_limits;
		} else {
			state->flags |= CPUIDLE_FLAG_IGNORE;
		}
	}

	return 0;
}

int msm_pm_idle_enter(enum msm_pm_sleep_mode sleep_mode)
{
	int64_t time;
#ifdef CONFIG_MSM_IDLE_STATS
	int exit_stat;
#endif

	if (MSM_PM_DEBUG_IDLE & msm_pm_debug_mask)
		pr_info("CPU%u: %s: mode %d\n",
			smp_processor_id(), __func__, sleep_mode);

	time = ktime_to_ns(ktime_get());

	switch (sleep_mode) {
	case MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT:
		msm_pm_swfi();
#ifdef CONFIG_MSM_IDLE_STATS
		exit_stat = MSM_PM_STAT_IDLE_WFI;
#endif
		break;

	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE:
		msm_pm_power_collapse_standalone(true);
#ifdef CONFIG_MSM_IDLE_STATS
		exit_stat = MSM_PM_STAT_IDLE_STANDALONE_POWER_COLLAPSE;
#endif
		break;

	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE: {
		int64_t timer_expiration = msm_timer_enter_idle();
		bool timer_halted = false;
		uint32_t sleep_delay;
		int ret;

		sleep_delay = (uint32_t) msm_pm_convert_and_cap_time(
			timer_expiration, MSM_PM_SLEEP_TICK_LIMIT);
		if (sleep_delay == 0) /* 0 would mean infinite time */
			sleep_delay = 1;

		ret = msm_rpmrs_enter_sleep(
				true, sleep_delay, msm_pm_idle_rs_limits);
		if (!ret) {
			msm_pm_power_collapse(true);
			timer_halted = true;

			msm_rpmrs_exit_sleep(true, msm_pm_idle_rs_limits);
		}

		msm_timer_exit_idle((int) timer_halted);
#ifdef CONFIG_MSM_IDLE_STATS
		exit_stat = MSM_PM_STAT_IDLE_POWER_COLLAPSE;
#endif
		break;
	}

	default:
		__WARN();
		goto cpuidle_enter_bail;
	}

	time = ktime_to_ns(ktime_get()) - time;
#ifdef CONFIG_MSM_IDLE_STATS
	msm_pm_add_stat(exit_stat, time);
#endif

	do_div(time, 1000);
	return (int) time;

cpuidle_enter_bail:
	return 0;
}

extern void soc_dump_active_pxo_clocks(void);
extern void msm_dump_gpio_table  (int flags);

static int msm_pm_enter(suspend_state_t state)
{
	bool allow[MSM_PM_SLEEP_MODE_NR];
	int i;

#ifdef CONFIG_MSM_IDLE_STATS
	int64_t period = 0;
	int64_t time = msm_timer_get_sclk_time(&period);
#endif

	if (MSM_PM_DEBUG_SUSPEND & msm_pm_debug_mask) {
		pr_info("%s\n", __func__);
#if 0
		if (MSM_PM_DEBUG_SUSPEND_LIMITS & msm_pm_debug_mask) {
			soc_dump_active_pxo_clocks ();
		}
#endif
		if (MSM_PM_DEBUG_SUSPEND_PINS & msm_pm_debug_mask) {
			msm_dump_gpio_table(0);
		}
	}

	if (smp_processor_id()) {
		__WARN();
		goto enter_exit;
	}


	for (i = 0; i < MSM_PM_SLEEP_MODE_NR; i++) {
		struct msm_pm_platform_data *mode;

		mode = &msm_pm_modes[MSM_PM_MODE(0, i)];
		allow[i] = mode->supported && mode->suspend_enabled;
	}

	if (allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE]) {
		struct msm_rpmrs_limits *rs_limits;
		int ret;

		if (MSM_PM_DEBUG_SUSPEND & msm_pm_debug_mask)
			pr_info("%s: power collapse\n", __func__);

		clock_debug_print_enabled();

#ifdef CONFIG_MSM_SLEEP_TIME_OVERRIDE
		if (msm_pm_sleep_time_override > 0) {
			int64_t ns = NSEC_PER_SEC *
				(int64_t) msm_pm_sleep_time_override;
			msm_pm_set_max_sleep_time(ns);
			msm_pm_sleep_time_override = 0;
		}
#endif /* CONFIG_MSM_SLEEP_TIME_OVERRIDE */

		if (MSM_PM_DEBUG_SUSPEND_LIMITS & msm_pm_debug_mask)
			msm_rpmrs_show_resources();

		rs_limits = msm_rpmrs_lowest_limits(false,
				MSM_PM_SLEEP_MODE_POWER_COLLAPSE, -1, -1);

		if ((MSM_PM_DEBUG_SUSPEND_LIMITS & msm_pm_debug_mask) &&
				rs_limits)
			pr_info("%s: limit %p: pxo %d, l2_cache %d, "
				"vdd_mem %d, vdd_dig %d\n",
				__func__, rs_limits,
				rs_limits->pxo, rs_limits->l2_cache,
				rs_limits->vdd_mem, rs_limits->vdd_dig);

		if (rs_limits) {
			ret = msm_rpmrs_enter_sleep(
				false, msm_pm_max_sleep_time, rs_limits);
			if (!ret) {
				msm_pm_power_collapse(false);
				msm_rpmrs_exit_sleep(false, rs_limits);
			}
		} else {
			pr_err("%s: cannot find the lowest power limit\n",
				__func__);
		}

#ifdef CONFIG_MSM_IDLE_STATS
		if (time != 0) {
			int64_t end_time = msm_timer_get_sclk_time(NULL);
			if (end_time != 0) {
				time = end_time - time;
				if (time < 0)
					time += period;
			} else
				time = 0;
		}

		msm_pm_add_stat(MSM_PM_STAT_SUSPEND, time);
#endif /* CONFIG_MSM_IDLE_STATS */
	} else if (allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE]) {
		if (MSM_PM_DEBUG_SUSPEND & msm_pm_debug_mask)
			pr_info("%s: standalone power collapse\n", __func__);
		msm_pm_power_collapse_standalone(false);
	} else if (allow[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT]) {
		if (MSM_PM_DEBUG_SUSPEND & msm_pm_debug_mask)
			pr_info("%s: swfi\n", __func__);
		msm_pm_swfi();
	}

	msm_gpio_save_wakeup_gpio();

enter_exit:
	if (MSM_PM_DEBUG_SUSPEND & msm_pm_debug_mask)
		pr_info("%s: return\n", __func__);

	return 0;
}

static struct platform_suspend_ops msm_pm_ops = {
	.enter = msm_pm_enter,
	.valid = suspend_valid_only_mem,
};

#ifdef CONFIG_HOTPLUG_CPU
int platform_cpu_disable(unsigned int cpu)
{
	return cpu == 0 ? -EPERM : 0;
}

int platform_cpu_kill(unsigned int cpu)
{
	struct completion *killed = &per_cpu(msm_pm_devices, cpu).cpu_killed;
	return wait_for_completion_timeout(killed, HZ * 5);
}

void platform_cpu_die(unsigned int cpu)
{
	bool allow[MSM_PM_SLEEP_MODE_NR];
	int i;

	if (unlikely(cpu != smp_processor_id())) {
		pr_crit("%s: running on %u, should be %u\n",
			__func__, smp_processor_id(), cpu);
		BUG();
	}

	for (i = 0; i < MSM_PM_SLEEP_MODE_NR; i++) {
		struct msm_pm_platform_data *mode;

		mode = &msm_pm_modes[MSM_PM_MODE(cpu, i)];
		allow[i] = mode->supported && mode->suspend_enabled;
	}

	if (MSM_PM_DEBUG_HOTPLUG & msm_pm_debug_mask)
		pr_notice("CPU%u: %s: shutting down cpu\n", cpu, __func__);
	complete(&__get_cpu_var(msm_pm_devices).cpu_killed);

	flush_cache_all();

	for (;;) {
		if (allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE]) {
			struct msm_pm_device *dev =
				&__get_cpu_var(msm_pm_devices);
			dev->saved_acpu_rate = acpuclk_get_rate(cpu);
			msm_pm_power_collapse(false);
		}
		else if (allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE])
			msm_pm_power_collapse_standalone(false);
		else if (allow[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT])
			msm_pm_swfi();

		if (pen_release == cpu) {
			/* OK, proper wakeup, we're done */
			break;
		}
	}

	pen_release = -1;
	pr_notice("CPU%u: %s: normal wakeup\n", cpu, __func__);
}

int msm_pm_platform_secondary_init(unsigned int cpu)
{
	int ret;
	struct msm_pm_device *dev = &__get_cpu_var(msm_pm_devices);

	if (!dev->warm_boot) {
		dev->warm_boot = 1;
		return 0;
	}
#ifdef CONFIG_VFP
	vfp_reinit();
#endif

	if (dev->saved_acpu_rate) {
		ret = acpuclk_set_rate(dev->cpu,
				dev->saved_acpu_rate,
				SETRATE_PC);
		if (ret)
			pr_err("CPU%u: %s: failed clock rate restore(%lu)\n",
			dev->cpu, __func__, dev->saved_acpu_rate);
		dev->saved_acpu_rate = 0;
	}
	ret = msm_spm_set_low_power_mode(MSM_SPM_MODE_CLOCK_GATING, false);

	return ret;
}
#endif  /* CONFIG_HOTPLUG_CPU */

/******************************************************************************
 * Initialization routine
 *****************************************************************************/

static int __init msm_pm_init(void)
{
	pgd_t *pc_pgd;
	pmd_t *pmd;
	unsigned int irq;
	unsigned int cpu;
#ifdef CONFIG_MSM_IDLE_STATS
	struct proc_dir_entry *d_entry;
#endif
	int ret;

	/* Page table for cores to come back up safely. */
	pc_pgd = pgd_alloc(&init_mm);
	if (!pc_pgd)
		return -ENOMEM;

	pmd = pmd_offset(pc_pgd +
			 pgd_index(virt_to_phys(msm_pm_collapse_exit)),
			 virt_to_phys(msm_pm_collapse_exit));
	*pmd = __pmd((virt_to_phys(msm_pm_collapse_exit) & PGDIR_MASK) |
		     PMD_TYPE_SECT | PMD_SECT_AP_WRITE);
	flush_pmd_entry(pmd);
	msm_pm_pc_pgd = virt_to_phys(pc_pgd);

	irq = RPM_SCSS_CPU0_WAKE_UP_IRQ;
	ret = request_irq(irq, msm_pm_rpm_wakeup_interrupt, IRQF_TRIGGER_RISING,
			"pm_drv", msm_pm_rpm_wakeup_interrupt);
	if (ret) {
		pr_err("%s: failed to request irq %u: %d\n",
			__func__, irq, ret);
		return ret;
	}

	ret = set_irq_wake(irq, 1);
	if (ret) {
		pr_err("%s: failed to set wakeup irq %u: %d\n",
			__func__, irq, ret);
		return ret;
	}

	for_each_possible_cpu(cpu) {
		struct msm_pm_device *dev = &per_cpu(msm_pm_devices, cpu);

		dev->cpu = cpu;
#ifdef CONFIG_HOTPLUG_CPU
		init_completion(&dev->cpu_killed);
#endif
	}

	ret = scm_set_boot_addr((void *)virt_to_phys(msm_pm_boot_entry),
			SCM_FLAG_WARMBOOT_CPU0 | SCM_FLAG_WARMBOOT_CPU1);
	if (ret) {
		pr_err("%s: failed to set up scm boot addr: %d\n",
			__func__, ret);
		return ret;
	}

#ifdef CONFIG_MSM_IDLE_STATS
	for_each_possible_cpu(cpu) {
		struct msm_pm_time_stats *stats =
			per_cpu(msm_pm_stats, cpu).stats;

		stats[MSM_PM_STAT_REQUESTED_IDLE].name = "idle-request";
		stats[MSM_PM_STAT_REQUESTED_IDLE].first_bucket_time =
			CONFIG_MSM_IDLE_STATS_FIRST_BUCKET;

		stats[MSM_PM_STAT_IDLE_WFI].name = "idle-wfi";
		stats[MSM_PM_STAT_IDLE_WFI].first_bucket_time =
			CONFIG_MSM_IDLE_STATS_FIRST_BUCKET;

		stats[MSM_PM_STAT_IDLE_STANDALONE_POWER_COLLAPSE].name =
			"idle-standalone-power-collapse";
		stats[MSM_PM_STAT_IDLE_STANDALONE_POWER_COLLAPSE].
			first_bucket_time = CONFIG_MSM_IDLE_STATS_FIRST_BUCKET;

		stats[MSM_PM_STAT_IDLE_POWER_COLLAPSE].name =
			"idle-power-collapse";
		stats[MSM_PM_STAT_IDLE_POWER_COLLAPSE].first_bucket_time =
			CONFIG_MSM_IDLE_STATS_FIRST_BUCKET;

		stats[MSM_PM_STAT_SUSPEND].name = "suspend";
		stats[MSM_PM_STAT_SUSPEND].first_bucket_time =
			CONFIG_MSM_SUSPEND_STATS_FIRST_BUCKET;
	}

	d_entry = create_proc_entry("msm_pm_stats",
			S_IRUGO | S_IWUSR | S_IWGRP, NULL);
	if (d_entry) {
		d_entry->read_proc = msm_pm_read_proc;
		d_entry->write_proc = msm_pm_write_proc;
		d_entry->data = NULL;
	}
#endif  /* CONFIG_MSM_IDLE_STATS */

	msm_pm_mode_sysfs_add();
	msm_spm_allow_x_cpu_set_vdd(false);

	suspend_set_ops(&msm_pm_ops);
	msm_cpuidle_init();

#ifdef CONFIG_DEBUG_TZ_COUNTERS
	vik_regsave = (void *)__get_free_page(GFP_KERNEL);

	if (vik_regsave) {
		imem_debug_loc = ioremap_nocache(IMEM_DEBUG_LOC, 8);

		if (!imem_debug_loc)
			panic("Power collapse experiment fail! ioremap failed.");
		writel(0xDEADBEEF, imem_debug_loc);

		pr_info("PC DEBUG COUNTER LOCATION: 0x%lX\n",
		__pa(vik_regsave));

	} else {
		panic("PC EXPERIMENT FAIL!! gfp failed\n");
	}
	tz_counters = ioremap_nocache(0x2a05f810, 16);
#endif

	return 0;
}

late_initcall(msm_pm_init);
