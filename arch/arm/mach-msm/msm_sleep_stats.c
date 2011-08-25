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
/*
 * Qualcomm MSM Sleep Stats Interface for Userspace
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/cpu.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/cpufreq.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/spinlock.h>

#include "cpuidle.h"

struct sleep_data {
	int cpu;
	atomic_t idle_microsec;
	atomic_t busy_microsec;
	atomic_t timer_val_ms;
	atomic_t timer_expired;
	atomic_t policy_changed;
	struct hrtimer timer;
	struct attribute_group *attr_group;
	struct kobject *kobj;
	struct notifier_block nb;
	struct wait_to_notify *wait;
	struct work_struct work;
};

struct rq_data {
	unsigned int rq_avg;
	unsigned int rq_poll_ms;
	unsigned int def_timer_ms;
	unsigned int def_interval;
	int64_t last_time;
	int64_t total_time;
	int64_t def_start_time;
	struct delayed_work rq_work;
	struct attribute_group *attr_group;
	struct kobject *kobj;
	struct delayed_work def_timer_work;
};

DEFINE_PER_CPU(struct sleep_data, core_sleep_info);
static struct rq_data rq_info;
static DEFINE_SPINLOCK(rq_lock);

static struct workqueue_struct *msm_stats_wq;

static void idle_enter(int cpu, unsigned int microsec)
{
	struct sleep_data *sleep_info = &per_cpu(core_sleep_info, cpu);

	if (!sleep_info || (sleep_info && (sleep_info->cpu < 0)))
		return;

	/* cumulative atomic counter, reset after reading */
	atomic_add(microsec, &sleep_info->busy_microsec);
	hrtimer_cancel(&sleep_info->timer);
}

static void idle_exit(int cpu, unsigned int microsec)
{
	struct sleep_data *sleep_info = &per_cpu(core_sleep_info, cpu);

	if (!sleep_info || (sleep_info && (sleep_info->cpu < 0)))
		return;

	/* cumulative atomic counter, reset after reading */
	atomic_add(microsec, &sleep_info->idle_microsec);
	if (atomic_read(&sleep_info->timer_val_ms) != INT_MAX &&
		atomic_read(&sleep_info->timer_val_ms))
		hrtimer_start(&sleep_info->timer,
			ktime_set(0,
			atomic_read(&sleep_info->timer_val_ms) * NSEC_PER_MSEC),
			HRTIMER_MODE_REL);
}

static void notify_uspace_work_fn(struct work_struct *work)
{
	struct sleep_data *sleep_info = container_of(work, struct sleep_data,
			work);

	/* Notify polling threads on change of value */
	sysfs_notify(sleep_info->kobj, NULL, "timer_expired");
}

static void rq_work_fn(struct work_struct *work)
{
	int64_t time_diff = 0;
	int64_t rq_avg = 0;
	unsigned long flags = 0;

	spin_lock_irqsave(&rq_lock, flags);

	if (!rq_info.last_time)
		rq_info.last_time = ktime_to_ns(ktime_get());
	if (!rq_info.rq_avg)
		rq_info.total_time = 0;

	rq_avg = nr_running() * 10;
	time_diff = ktime_to_ns(ktime_get()) - rq_info.last_time;
	do_div(time_diff, (1000 * 1000));

	if (time_diff && rq_info.total_time) {
		rq_avg = (rq_avg * time_diff) +
			(rq_info.rq_avg * rq_info.total_time);
		do_div(rq_avg, rq_info.total_time + time_diff);
	}

	rq_info.rq_avg =  (unsigned int)rq_avg;

	/* Set the next poll */
	if (rq_info.rq_poll_ms)
		queue_delayed_work(msm_stats_wq, &rq_info.rq_work,
			msecs_to_jiffies(rq_info.rq_poll_ms));

	rq_info.total_time += time_diff;
	rq_info.last_time = ktime_to_ns(ktime_get());

	spin_unlock_irqrestore(&rq_lock, flags);
}

static void def_work_fn(struct work_struct *work)
{
	int64_t diff;

	diff = ktime_to_ns(ktime_get()) - rq_info.def_start_time;
	do_div(diff, 1000 * 1000);
	rq_info.def_interval = (unsigned int) diff;

	/* Notify polling threads on change of value */
	sysfs_notify(rq_info.kobj, NULL, "def_timer_ms");
}

static enum hrtimer_restart timer_func(struct hrtimer *handle)
{
	struct sleep_data *sleep_info = container_of(handle, struct sleep_data,
			timer);

	if (atomic_read(&sleep_info->timer_expired))
		pr_info("msm_sleep_stats: Missed timer interrupt on cpu %d\n",
				sleep_info->cpu);

	atomic_set(&sleep_info->timer_val_ms, 0);
	atomic_set(&sleep_info->timer_expired, 1);

	queue_work_on(sleep_info->cpu, msm_stats_wq, &sleep_info->work);

	return HRTIMER_NORESTART;
}

static ssize_t show_idle_ms(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int val = 0;
	int cpu = 0;
	struct sleep_data *sleep_info = NULL;

	sscanf(kobj->parent->name, "cpu%d", &cpu);
	sleep_info = &per_cpu(core_sleep_info, cpu);
	val = atomic_read(&sleep_info->idle_microsec);
	atomic_sub(val, &sleep_info->idle_microsec);

	return sprintf(buf, "%d\n", val/1000);
}

static ssize_t show_busy_ms(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int val = 0;
	int cpu = 0;
	struct sleep_data *sleep_info = NULL;

	sscanf(kobj->parent->name, "cpu%d", &cpu);
	sleep_info = &per_cpu(core_sleep_info, cpu);
	val = atomic_read(&sleep_info->busy_microsec);
	atomic_sub(val, &sleep_info->busy_microsec);

	return sprintf(buf, "%d\n", val/1000);
}

static ssize_t show_timer_val_ms(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int val = 0;
	int cpu = 0;
	struct sleep_data *sleep_info = NULL;

	sscanf(kobj->parent->name, "cpu%d", &cpu);
	sleep_info = &per_cpu(core_sleep_info, cpu);
	val = atomic_read(&sleep_info->timer_val_ms);
	atomic_sub(val, &sleep_info->timer_val_ms);

	return sprintf(buf, "%d\n", val);
}

static ssize_t store_timer_val_ms(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val = 0;
	int cpu = 0;
	struct sleep_data *sleep_info = NULL;

	sscanf(kobj->parent->name, "cpu%d", &cpu);
	sleep_info = &per_cpu(core_sleep_info, cpu);
	hrtimer_cancel(&sleep_info->timer);
	sscanf(buf, "%du", &val);
	atomic_set(&sleep_info->timer_val_ms, val);
	if (atomic_read(&sleep_info->timer_val_ms) != INT_MAX &&
		atomic_read(&sleep_info->timer_val_ms))
		hrtimer_start(&sleep_info->timer,
			ktime_set(0,
			atomic_read(&sleep_info->timer_val_ms) * NSEC_PER_MSEC),
			HRTIMER_MODE_REL);

	return count;
}

static ssize_t show_timer_expired(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int val = 0;
	int cpu = 0;
	struct sleep_data *sleep_info = NULL;

	sscanf(kobj->parent->name, "cpu%d", &cpu);
	sleep_info = &per_cpu(core_sleep_info, cpu);
	val = atomic_read(&sleep_info->timer_expired);
	atomic_set(&sleep_info->timer_expired, 0);

	return sprintf(buf, "%d\n", val);
}

static ssize_t show_policy_changed(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int val = 0;
	int cpu = 0;
	struct sleep_data *sleep_info = NULL;

	sscanf(kobj->parent->name, "cpu%d", &cpu);
	sleep_info = &per_cpu(core_sleep_info, cpu);
	val = atomic_read(&sleep_info->policy_changed);
	atomic_set(&sleep_info->policy_changed, 0);

	return sprintf(buf, "%d\n", val);
}

static ssize_t show_run_queue_avg(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	unsigned int val = 0;
	unsigned long flags = 0;

	spin_lock_irqsave(&rq_lock, flags);
	/* rq avg currently available only on one core */
	val = rq_info.rq_avg;
	rq_info.rq_avg = 0;
	spin_unlock_irqrestore(&rq_lock, flags);

	return sprintf(buf, "%d.%d\n", val/10, val%10);
}

static ssize_t show_run_queue_poll_ms(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned long flags = 0;

	spin_lock_irqsave(&rq_lock, flags);
	ret = sprintf(buf, "%u\n", rq_info.rq_poll_ms);
	spin_unlock_irqrestore(&rq_lock, flags);

	return ret;
}

static ssize_t store_run_queue_poll_ms(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val = 0;
	unsigned long flags = 0;
	static DEFINE_MUTEX(lock_poll_ms);

	mutex_lock(&lock_poll_ms);

	spin_lock_irqsave(&rq_lock, flags);
	sscanf(buf, "%u", &val);
	rq_info.rq_poll_ms = val;
	spin_unlock_irqrestore(&rq_lock, flags);

	if (val <= 0)
		cancel_delayed_work(&rq_info.rq_work);
	else
		queue_delayed_work(msm_stats_wq, &rq_info.rq_work, 
			msecs_to_jiffies(val));

	mutex_unlock(&lock_poll_ms);

	return count;
}

static ssize_t show_def_timer_ms(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", rq_info.def_interval);
}

static ssize_t store_def_timer_ms(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int val = 0;

	sscanf(buf, "%u", &val);
	rq_info.def_timer_ms = val;

	if (val <= 0)
		cancel_delayed_work(&rq_info.def_timer_work);
	else {
		rq_info.def_start_time = ktime_to_ns(ktime_get());
		queue_delayed_work(msm_stats_wq, &rq_info.def_timer_work,
				msecs_to_jiffies(val));
	}

	return count;
}

static int policy_change_notifier(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct sleep_data *sleep_info = container_of(nb, struct sleep_data, nb);
	struct cpufreq_policy *policy = (struct cpufreq_policy *)data;

	if (event == CPUFREQ_ADJUST && sleep_info->cpu == policy->cpu) {
		atomic_set(&sleep_info->policy_changed, 1);
		sysfs_notify(sleep_info->kobj, NULL, "policy_changed");
	}

	return 0;
}

#define MSM_SLEEP_RO_ATTRIB(att) ({ \
		struct attribute *attrib = NULL; \
		struct kobj_attribute *ptr = NULL; \
		ptr = kzalloc(sizeof(struct kobj_attribute), GFP_KERNEL); \
		if (ptr) { \
			ptr->attr.name = #att; \
			ptr->attr.mode = S_IRUGO; \
			ptr->show = show_##att; \
			ptr->store = NULL; \
			attrib = &ptr->attr; \
		} \
		attrib; })

#define MSM_SLEEP_RW_ATTRIB(att) ({ \
		struct attribute *attrib = NULL; \
		struct kobj_attribute *ptr = NULL; \
		ptr = kzalloc(sizeof(struct kobj_attribute), GFP_KERNEL); \
		if (ptr) { \
			ptr->attr.name = #att; \
			ptr->attr.mode = S_IWUSR|S_IRUSR; \
			ptr->show = show_##att; \
			ptr->store = store_##att; \
			attrib = &ptr->attr; \
		} \
		attrib; })


static int add_sysfs_objects(struct sleep_data *sleep_info)
{
	int err = 0;
	int i = 0;
	const int attr_count = 6;

	struct attribute **attribs =
		kzalloc(sizeof(struct attribute *) * attr_count, GFP_KERNEL);

	if (!attribs)
		goto rel;

	atomic_set(&sleep_info->idle_microsec, 0);
	atomic_set(&sleep_info->busy_microsec, 0);
	atomic_set(&sleep_info->timer_expired, 0);
	atomic_set(&sleep_info->policy_changed, 0);
	atomic_set(&sleep_info->timer_val_ms, INT_MAX);

	attribs[0] = MSM_SLEEP_RO_ATTRIB(idle_ms);
	attribs[1] = MSM_SLEEP_RO_ATTRIB(busy_ms);
	attribs[2] = MSM_SLEEP_RW_ATTRIB(timer_val_ms);
	attribs[3] = MSM_SLEEP_RO_ATTRIB(timer_expired);
	attribs[4] = MSM_SLEEP_RO_ATTRIB(policy_changed);
	attribs[5] = NULL;

	for (i = 0; i < attr_count - 1 ; i++) {
		if (!attribs[i])
			goto rel;
	}

	sleep_info->attr_group = kzalloc(sizeof(struct attribute_group),
						GFP_KERNEL);
	if (!sleep_info->attr_group)
		goto rel;
	sleep_info->attr_group->attrs = attribs;

	/* Create /sys/devices/system/cpu/cpuX/sleep-stats/... */
	sleep_info->kobj = kobject_create_and_add("sleep-stats",
			&get_cpu_sysdev(sleep_info->cpu)->kobj);
	if (!sleep_info->kobj)
		goto rel;

	err = sysfs_create_group(sleep_info->kobj, sleep_info->attr_group);
	if (err)
		kobject_put(sleep_info->kobj);
	else
		kobject_uevent(sleep_info->kobj, KOBJ_ADD);

	if (!err)
		return err;

rel:
	for (i = 0; i < attr_count - 1 ; i++)
		kfree(attribs[i]);
	kfree(attribs);
	kfree(sleep_info->attr_group);
	kfree(sleep_info->kobj);

	return -ENOMEM;
}

static void remove_sysfs_objects(struct sleep_data *sleep_info)
{
	if (sleep_info->kobj)
		sysfs_remove_group(sleep_info->kobj, sleep_info->attr_group);

	kfree(sleep_info->attr_group);
	kfree(sleep_info->kobj);
}

static int init_rq_attribs(void)
{
	int i;
	int err = 0;
	const int attr_count = 4;

	struct attribute **attribs =
		kzalloc(sizeof(struct attribute *) * attr_count, GFP_KERNEL);

	if (!attribs)
		goto rel;

	rq_info.rq_avg = 0;
	rq_info.rq_poll_ms = 0;

	attribs[0] = MSM_SLEEP_RW_ATTRIB(def_timer_ms);
	attribs[1] = MSM_SLEEP_RO_ATTRIB(run_queue_avg);
	attribs[2] = MSM_SLEEP_RW_ATTRIB(run_queue_poll_ms);
	attribs[3] = NULL;

	for (i = 0; i < attr_count - 1 ; i++) {
		if (!attribs[i])
			goto rel;
	}

	rq_info.attr_group = kzalloc(sizeof(struct attribute_group),
						GFP_KERNEL);
	if (!rq_info.attr_group)
		goto rel;
	rq_info.attr_group->attrs = attribs;

	/* Create /sys/devices/system/cpu/cpu0/rq-stats/... */
	rq_info.kobj = kobject_create_and_add("rq-stats",
			&get_cpu_sysdev(0)->kobj);
	if (!rq_info.kobj)
		goto rel;

	err = sysfs_create_group(rq_info.kobj, rq_info.attr_group);
	if (err)
		kobject_put(rq_info.kobj);
	else
		kobject_uevent(rq_info.kobj, KOBJ_ADD);

	if (!err)
		return err;

rel:
	for (i = 0; i < attr_count - 1 ; i++)
		kfree(attribs[i]);
	kfree(attribs);
	kfree(rq_info.attr_group);
	kfree(rq_info.kobj);

	return -ENOMEM;
}

static int __init msm_sleep_info_init(void)
{
	int err = 0;
	int cpu;
	struct sleep_data *sleep_info = NULL;

	msm_stats_wq = create_workqueue("msm_stats_wq");
	if (!msm_stats_wq) {
		printk(KERN_ERR "Creation of msm_stats_wq failed!!\n");
		return -EINVAL;
	}
	

	/* Register callback from idle for all cpus */
	msm_idle_register_cb(idle_enter, idle_exit);
	INIT_DELAYED_WORK_DEFERRABLE(&rq_info.rq_work, rq_work_fn);
	INIT_DELAYED_WORK_DEFERRABLE(&rq_info.def_timer_work, def_work_fn);
	init_rq_attribs();

	for_each_possible_cpu(cpu) {
		printk(KERN_INFO "msm_sleep_stats: Initializing sleep stats "
				"for CPU[%d]\n", cpu);
		sleep_info = &per_cpu(core_sleep_info, cpu);
		sleep_info->cpu = cpu;
		INIT_WORK(&sleep_info->work, notify_uspace_work_fn);

		/* Initialize high resolution timer */
		hrtimer_init(&sleep_info->timer, CLOCK_MONOTONIC,
				HRTIMER_MODE_REL);
		sleep_info->timer.function = timer_func;

		/* Register for cpufreq policy changes */
		sleep_info->nb.notifier_call = policy_change_notifier;
		err = cpufreq_register_notifier(&sleep_info->nb,
					CPUFREQ_POLICY_NOTIFIER);
		if (err)
			goto cleanup;

		/* Create sysfs object */
		err = add_sysfs_objects(sleep_info);
		if (err)
			goto cleanup;

		continue;
cleanup:
		printk(KERN_INFO "msm_sleep_stats: Failed to initialize sleep "
				"stats for CPU[%d]\n", cpu);
		sleep_info->cpu = -1;
		cpufreq_unregister_notifier(&sleep_info->nb,
				CPUFREQ_POLICY_NOTIFIER);
		remove_sysfs_objects(sleep_info);
	}

	return 0;
}
late_initcall(msm_sleep_info_init);

