/*
 *  drivers/cpufreq/cpufreq_ondemand_tickle.c
 *
 *  A version of cpufreq_ondemand supporing hinting, or tickling, into
 *  high performance levels based on platform defined events. This governor
 *  also supports setting a temporary frequency floor for maintaining
 *  minimum required performance levels while still conserving power, such
 *  as may be required in codecs, com stacks, etc. Ondemand_tickle should
 *  behave identically to ondemand when neither a tickel nor a floor is active.
 *
 *  Copyright (C)  2001 Russell King
 *            (C)  2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *                      Jun Nakajima <jun.nakajima@intel.com>
 *            (C)  2009 Palm Inc, Corey Tabaka <corey.tabaka@palm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* Palm modifications to the ondemand driver fall into 2 categories:
 * 1) Floor and tickle capabilities (raise the frequency for controlled
 *	periods of time)
 * 2) Logging of cpufreq data (controlled by "sampling_enabled"; the name
 *	"sample" for this is a little unfortunate).
 *	Log data is retrieved from /proc/ondemandtcl_samples, and stored
 *	with calls to "record_sample". The intent is to store any events that
 *	might be interesting for post-analysis. The "load" field is overloaded
 *	for this. For load-based frequency changes, it contains the calculated
 *	load (0-100). For frequency changes caused for other reasons (tickles,
 *	floor, limit changes, input subsystem), it will contain a value between
 *	-1 and -99. Values that do not record an actual frequency change but
 *	record a relevant event will have a load value < -100; this includes
 *	CPU1 on/offline transitions. For further detail, look at the code.
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpufreq_tickle.h>
#include <linux/cpu.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/ktime.h>
#include <linux/sched.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/vmalloc.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/list.h>

/*
 * dbs is used in this file as a shortform for demandbased switching
 * It helps to keep variable names smaller, simpler
 */

#define DEF_FREQUENCY_DOWN_DIFFERENTIAL		(10)
#define DEF_FREQUENCY_UP_THRESHOLD		(80)
#define DEF_SAMPLING_DOWN_FACTOR		(1)
#define MAX_SAMPLING_DOWN_FACTOR		(100000)
#define MICRO_FREQUENCY_DOWN_DIFFERENTIAL	(3)
#define MICRO_FREQUENCY_UP_THRESHOLD		(95)
#define MICRO_FREQUENCY_MIN_SAMPLE_RATE		(10000)
#define MIN_FREQUENCY_UP_THRESHOLD		(11)
#define MAX_FREQUENCY_UP_THRESHOLD		(100)
#define MIN_FREQUENCY_DOWN_DIFFERENTIAL		(1)

/* empirically determined reasonable sampling value; setting it
 * here as a default is intended as a temporary fix. Set it to 0 if you
 * want this setting to be ignored.
 */
#define DEF_SAMPLING_RATE			(200000)

/*
 * The max and default time in mS to keep the processor at max freq from a tickle.
 */
#define MAX_TICKLE_WINDOW				(10000)
#define DEF_TICKLE_WINDOW				(3000)


#if defined(CONFIG_CPU_FREQ_MIN_TICKS)
#define CPU_FREQ_MIN_TICKS CONFIG_CPU_FREQ_MIN_TICKS
#else
#define CPU_FREQ_MIN_TICKS 10
#endif

#if defined(CPU_FREQ_SAMPLING_LATENCY_MULTIPLIER)
#define CPU_FREQ_SAMPLING_LATENCY_MULTIPLIER CONFIG_CPU_FREQ_SAMPLING_LATENCY_MULTIPLIER
#else
#define CPU_FREQ_SAMPLING_LATENCY_MULTIPLIER 1000
#endif

/*
 * The polling frequency of this governor depends on the capability of
 * the processor. Default polling frequency is 1000 times the transition
 * latency of the processor. The governor will work on any processor with
 * transition latency <= 10mS, using appropriate sampling
 * rate.
 * For CPUs with transition latency > 10mS (mostly drivers with CPUFREQ_ETERNAL)
 * this governor will not work.
 * All times here are in uS.
 */
#define MIN_SAMPLING_RATE_RATIO			(2)

static unsigned int min_sampling_rate;

#define LATENCY_MULTIPLIER			(1000)
#define MIN_LATENCY_MULTIPLIER			(100)
#define TRANSITION_LATENCY_LIMIT		(10 * 1000 * 1000)

static void do_dbs_timer(struct work_struct *work);

static void do_tickle_timer(unsigned long arg);

/* Sampling types */
enum {DBS_NORMAL_SAMPLE, DBS_SUB_SAMPLE};

/* stashed values from dbs_check_cpu, in us: for log sampling */
struct raw_load_values {
	unsigned int wall;
	unsigned int idle;
	unsigned int iowait;
};

struct cpu_dbs_info_s {
	cputime64_t prev_cpu_idle;
	cputime64_t prev_cpu_iowait;
	cputime64_t prev_cpu_wall;
	cputime64_t prev_cpu_nice;
	struct cpufreq_policy *cur_policy;
	struct delayed_work work;
	struct cpufreq_frequency_table *freq_table;
	unsigned int freq_lo;
	unsigned int freq_lo_jiffies;
	unsigned int freq_hi_jiffies;
	unsigned int rate_mult;

	int tickle_active;
	int floor_active;
	unsigned int freq_floor;
	unsigned int rel_save;

	/* values stashed by dbs_check_cpu and retrieved by adjust_for_load */
	int cur_load;
	unsigned int max_load_freq;
	struct raw_load_values rlv;

	int cpu;
	unsigned int enable:1,
				sample_type:1;

	/*
	 * percpu mutex that serializes governor limit change with
	 * do_dbs_timer invocation. We do not want do_dbs_timer to run
	 * when user is changing the governor or limits.
	 */
	struct mutex timer_mutex;
};
static DEFINE_PER_CPU(struct cpu_dbs_info_s, od_cpu_dbs_info);

static unsigned int dbs_enable;	/* number of CPUs using this policy */

static void adjust_for_load(struct cpu_dbs_info_s *this_dbs_info);

/*
 * dbs_mutex protects data in dbs_tuners_ins from concurrent changes on
 * different CPUs. It protects dbs_enable in governor start/stop.
 */
static DEFINE_MUTEX(dbs_mutex);

static struct workqueue_struct	*kondemand_wq;

static struct dbs_tuners {
	unsigned int sampling_rate;
	unsigned int up_threshold;
	unsigned int down_differential;
	unsigned int ignore_nice;
	unsigned int sampling_down_factor;
	unsigned int powersave_bias;
	unsigned int io_is_busy;
	unsigned int max_tickle_window;
} dbs_tuners_ins = {
	.up_threshold = DEF_FREQUENCY_UP_THRESHOLD,
	.sampling_down_factor = DEF_SAMPLING_DOWN_FACTOR,
	.down_differential = DEF_FREQUENCY_DOWN_DIFFERENTIAL,
	.ignore_nice = 0,
	.powersave_bias = 0,
	.max_tickle_window = DEF_TICKLE_WINDOW,
};

/*
 * tickle/floor state (shared across CPUs)
 *   active:floor_active: is a tickle or floor currently held
 *   active_tickle: is there an active _timed_ tickle.
 *   active_count, floor_count: number of current tickle/floor
 *	"clients" (timer = 1, each hold = 1)
 */
static struct {
	spinlock_t lock;	/* protects this structure,  tickle_file_data
				 * structures, and tickle_clients list
				 */

	int active;
	int active_tickle;
	int active_count;
	unsigned long tickle_jiffies;


	int floor_active;
	int floor_count;
	unsigned int cur_freq_floor;

	struct work_struct tickle_work;
	struct work_struct floor_work;

	struct timer_list tickle_timer;
} tickle_state = {
	.lock = SPIN_LOCK_UNLOCKED,
	.active = 0,
	.active_tickle = 0,
	.active_count = 0,
	.floor_active = 0,
	.floor_count = 0,
	.cur_freq_floor = 0,
};

/*
 * Stats for profiling response characteristics.
 */
#define NUM_SAMPLES 2000
struct sample_data {
	unsigned long long timestamp;
	cputime64_t user;
	cputime64_t system;
	unsigned int wall;
	unsigned int idle;
	unsigned int iowait;
	unsigned int cur_freq;
	unsigned int target_freq;
	int load;
};

struct sample_stats {
	struct sample_data *samples;
	unsigned int current_sample;
};

static DEFINE_PER_CPU(struct sample_stats, sample_stats);

static int sampling_enabled = 0;
module_param(sampling_enabled, bool, S_IRUGO | S_IWUSR);

static int tickling_enabled = 1;
module_param(tickling_enabled, bool, S_IRUGO | S_IWUSR);

static int clear_samples = 0;
module_param(clear_samples, bool, S_IRUGO | S_IWUSR);

static int print_tickles = 0;
module_param(print_tickles, bool, S_IRUGO | S_IWUSR);

/********************* procfs ********************/
struct stats_state {
	int cpu;
	int sample_index;
	int num_samples;
};

static struct seq_operations stats_op;

static void *stats_start(struct seq_file *, loff_t *);
static void *stats_next(struct seq_file *, void *v, loff_t *);
static void stats_stop(struct seq_file *, void *);
static int stats_show(struct seq_file *, void *);

static int stats_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &stats_op);
}

static struct file_operations proc_stats_operations = {
	.open		= stats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static struct seq_operations stats_op = {
	.start		= stats_start,
	.next		= stats_next,
	.stop		= stats_stop,
	.show		= stats_show,
};

static int get_num_sample_records(void) {
	int i, count = 0;

	for (i = 0; i < NR_CPUS; i++) {
		struct sample_stats *stats = &per_cpu(sample_stats, i);

		if (stats->samples)
			count += NUM_SAMPLES;
	}

	return count;
}

static void *stats_start(struct seq_file *m, loff_t *pos)
{
	int i;
	struct stats_state *state = kmalloc(sizeof(struct stats_state),  GFP_KERNEL);

	pr_debug("%s: %u\n", __func__, (unsigned int) *pos);
	if (!state)
		return ERR_PTR(-ENOMEM);

	state->cpu = 0;
	state->sample_index = *pos;
	state->num_samples = get_num_sample_records();

	if (*pos >= state->num_samples)
		return NULL;

	// Find the first CPU running this governor with sampling enabled
	for (i = 0; i < NR_CPUS; i++) {
		struct sample_stats *stats = &per_cpu(sample_stats, i);

		if (stats->samples) {
			state->cpu = i;
			break;
		}
	}

	while (state->sample_index >= NUM_SAMPLES) {
		struct sample_stats *stats = &per_cpu(sample_stats, state->cpu);

		state->cpu++;
		if (stats->samples)
			state->sample_index -= NUM_SAMPLES;
	}

	return (void *) state;
}

static void *stats_next(struct seq_file *m, void *v, loff_t *pos)
{
	int i;
	struct stats_state *state = (struct stats_state *) v;

	pr_debug("%s: CPU:%u, Pos:%u\n" , __func__, state->cpu,
		(unsigned int) *pos);

	++*pos;
	if (*pos >= state->num_samples)
		return NULL;

	state->cpu = 0;
	state->sample_index = *pos;

	// Find the first CPU running this governor with sampling enabled
	for (i = 0; i < NR_CPUS; i++) {
		struct sample_stats *stats = &per_cpu(sample_stats, i);

		if (stats->samples) {
			state->cpu = i;
			break;
		}
	}

	while (state->sample_index >= NUM_SAMPLES) {
		struct sample_stats *stats = &per_cpu(sample_stats, state->cpu);

		state->cpu++;
		if (stats->samples)
			state->sample_index -= NUM_SAMPLES;
	}

	return (void *) state;
}

static void stats_stop(struct seq_file *m, void *v)
{
	kfree(v);
}

static int stats_show(struct seq_file *m, void *v)
{
	struct stats_state *state = v;
	struct sample_stats *stats = &per_cpu(sample_stats, state->cpu);
	int rc = 0;
	unsigned long long t;
	unsigned long  ms;


	if (stats->samples) {
		/* conversion borrowed from printk_time */
		t = stats->samples[state->sample_index].timestamp;
		ms = do_div(t, NSEC_PER_SEC);
		ms = ms/NSEC_PER_MSEC;

		rc = seq_printf(m, "%u %4u %5lu.%03lu %6llu"
			" %6llu %8u %8u %8u %4d %7u %7u\n",
				state->cpu,
				state->sample_index,
				(unsigned long)t,
				ms,
				stats->samples[state->sample_index].user,
				stats->samples[state->sample_index].system,
				stats->samples[state->sample_index].wall,
				stats->samples[state->sample_index].idle,
				stats->samples[state->sample_index].iowait,
				stats->samples[state->sample_index].load,
				stats->samples[state->sample_index].cur_freq,
				stats->samples[state->sample_index].target_freq
		);
		if (rc)
			goto done;
	}
done:
	return rc;
}
/******************** end procfs ********************/

/********************** ioctl ***********************/

static struct class *tickle_class = NULL;
static struct cdev tickle_cdev;
static dev_t tickle_dev;

static LIST_HEAD(tickle_clients);

struct tickle_file_data {
	struct list_head list;

	int tickle_hold_flag;
	int floor_hold_flag;
	unsigned int floor_freq;
};

/* kernel_floor_data can be used safely for a single
 * active kernel floor (hold/unhold).
 */
struct tickle_file_data kernel_floor_data;

static int get_max_client_floor(void);

static int tickle_open(struct inode *inode, struct file *filp);
static int tickle_release(struct inode *inode, struct file *filp);
static int tickle_ioctl(struct inode *inode, struct file *filp,
	unsigned int cmd, unsigned long arg);

static struct file_operations tickle_fops = {
	.owner			= THIS_MODULE,
	.open			= tickle_open,
	.release		= tickle_release,
	.ioctl			= tickle_ioctl,
};

static int setup_tickle_device(void)
{
	int res = 0;
	struct device *dev = NULL;

	res = alloc_chrdev_region(&tickle_dev, 0, 1, "ondemandtcl");
	if (res < 0) {
		printk(KERN_ERR "%s: can't alloc major number (%d)\n", __FILE__, res);
		goto error;
	}

	tickle_class = class_create(THIS_MODULE, "ondemandtcl");
	if (IS_ERR(tickle_class)) {
		res = PTR_ERR(tickle_class);
		printk(KERN_ERR "%s: can't create class (%d)\n", __FILE__, res);
		goto error_unregister_region;
	}

	cdev_init(&tickle_cdev, &tickle_fops);
	tickle_cdev.owner = THIS_MODULE;
	res = cdev_add(&tickle_cdev, tickle_dev, 1);
	if (res < 0) {
		printk(KERN_ERR "%s: failed to add device (%d)\n", __FILE__, res);
		goto error_remove_class;
	}

	/* always use ondemandtcl0, since userspace is already depending on that name */
	dev = device_create(tickle_class, NULL, tickle_dev, NULL, "ondemandtcl%d", 0);
	if (IS_ERR(dev)) {
		res = PTR_ERR(dev);
		printk(KERN_ERR "%s: failed to create device (%d)\n", __FILE__, res);

		goto error_remove_cdev;
	}
	/* add the default data for a single kernel floor client */
	kernel_floor_data.floor_freq = 0;
	list_add(&kernel_floor_data.list, &tickle_clients);

	return res;

error_remove_cdev:
	cdev_del(&tickle_cdev);

error_remove_class:
	class_destroy(tickle_class);
	tickle_class = NULL;

error_unregister_region:
	unregister_chrdev_region(tickle_dev, 1);

error:
	return res;
}

static void remove_tickle_device(void)
{
	if (tickle_class) {
		device_destroy(tickle_class, tickle_dev);
		class_destroy(tickle_class);
		cdev_del(&tickle_cdev);
		unregister_chrdev_region(tickle_dev, 1);

		/* for posterity */
		list_del(&kernel_floor_data.list);
	}
}

static int tickle_open(struct inode *inode, struct file *filp)
{
	unsigned long flags;
	struct tickle_file_data *data = kzalloc(sizeof(struct tickle_file_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	spin_lock_irqsave(&tickle_state.lock, flags);
	list_add(&data->list, &tickle_clients);
	spin_unlock_irqrestore(&tickle_state.lock, flags);

	filp->private_data = data;

	return 0;
}

static int tickle_release(struct inode *inode, struct file *filp)
{
	unsigned long flags;
	struct tickle_file_data *data = filp->private_data;

	if (data) {
		spin_lock_irqsave(&tickle_state.lock, flags);
		list_del(&data->list);
		spin_unlock_irqrestore(&tickle_state.lock, flags);

		CPUFREQ_UNHOLD_CHECK(&data->tickle_hold_flag);
		cpufreq_ondemand_floor_unhold_check(data,
			&data->floor_hold_flag);

		kfree(data);
	}

	return 0;
}

/* maximum of one floor hold and one tickle hold per fd */
static int tickle_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned long flags;
	struct tickle_file_data *data = filp->private_data;

	if (_IOC_TYPE(cmd) != TICKLE_IOC_MAGIC)
		return -ENOTTY;

	if (_IOC_NR(cmd) > TICKLE_IOC_MAXNR)
		return -ENOTTY;

	switch (cmd) {
		case TICKLE_IOCT_TICKLE:
			CPUFREQ_TICKLE_MILLIS((unsigned int) arg);
			break;

		case TICKLE_IOC_TICKLE_HOLD:
			if (data->tickle_hold_flag)
				return -EBUSY;
			CPUFREQ_HOLD_CHECK(&data->tickle_hold_flag);
			break;

		case TICKLE_IOC_TICKLE_UNHOLD:
			if (data->tickle_hold_flag == 0)
				return -EINVAL;
			CPUFREQ_UNHOLD_CHECK(&data->tickle_hold_flag);
			break;

		case TICKLE_IOCT_FLOOR_HOLD:
			if (data->floor_hold_flag)
				return -EBUSY;
			spin_lock_irqsave(&tickle_state.lock, flags);
			data->floor_freq = (unsigned int) arg;
			spin_unlock_irqrestore(&tickle_state.lock, flags);

			cpufreq_ondemand_floor_hold_check(data,
				&data->floor_hold_flag);
			break;

		case TICKLE_IOC_FLOOR_UNHOLD:
			if (data->floor_hold_flag == 0)
				return -EINVAL;

			cpufreq_ondemand_floor_unhold_check(data,
				&data->floor_hold_flag);
			break;

		case TICKLE_IOC_TICKLE_HOLD_SYNC:
			CPUFREQ_HOLD_SYNC();
			break;

		default:
			return -ENOTTY;
	}

	return 0;
}

/* This needs to be called with spinlock tickle_state.lock held */
static int get_max_client_floor(void)
{
	int floor_freq = 0;
	struct tickle_file_data *file_data;

	list_for_each_entry(file_data, &tickle_clients, list) {
		if (file_data->floor_freq > floor_freq)
			floor_freq = file_data->floor_freq;
	}

	return floor_freq;
}

/******************** end ioctl *********************/

static inline cputime64_t get_cpu_idle_time_jiffy(unsigned int cpu,
							cputime64_t *wall)
{
	cputime64_t idle_time;
	cputime64_t cur_wall_time;
	cputime64_t busy_time;

	cur_wall_time = jiffies64_to_cputime64(get_jiffies_64());
	busy_time = cputime64_add(kstat_cpu(cpu).cpustat.user,
			kstat_cpu(cpu).cpustat.system);

	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.irq);
	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.softirq);
	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.steal);
	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.nice);

	idle_time = cputime64_sub(cur_wall_time, busy_time);
	if (wall)
		*wall = (cputime64_t)jiffies_to_usecs(cur_wall_time);

	return (cputime64_t)jiffies_to_usecs(idle_time);
}

static inline cputime64_t get_cpu_idle_time(unsigned int cpu, cputime64_t *wall)
{
	u64 idle_time = get_cpu_idle_time_us(cpu, wall);

	if (idle_time == -1ULL)
		return get_cpu_idle_time_jiffy(cpu, wall);

	return idle_time;
}

static inline cputime64_t get_cpu_iowait_time(unsigned int cpu, cputime64_t *wall)
{
	u64 iowait_time = get_cpu_iowait_time_us(cpu, wall);

	if (iowait_time == -1ULL)
		return 0;

	return iowait_time;
}

/*
 * Find right freq to be set now with powersave_bias on.
 * Returns the freq_hi to be used right now and will set freq_hi_jiffies,
 * freq_lo, and freq_lo_jiffies in percpu area for averaging freqs.
 */
static unsigned int powersave_bias_target(struct cpufreq_policy *policy,
					  unsigned int freq_next,
					  unsigned int relation)
{
	unsigned int freq_req, freq_reduc, freq_avg;
	unsigned int freq_hi, freq_lo;
	unsigned int index = 0;
	unsigned int jiffies_total, jiffies_hi, jiffies_lo;
	struct cpu_dbs_info_s *dbs_info = &per_cpu(od_cpu_dbs_info,
						   policy->cpu);

	if (!dbs_info->freq_table) {
		dbs_info->freq_lo = 0;
		dbs_info->freq_lo_jiffies = 0;
		return freq_next;
	}

	cpufreq_frequency_table_target(policy, dbs_info->freq_table, freq_next,
			relation, &index);
	freq_req = dbs_info->freq_table[index].frequency;
	freq_reduc = freq_req * dbs_tuners_ins.powersave_bias / 1000;
	freq_avg = freq_req - freq_reduc;

	/* Find freq bounds for freq_avg in freq_table */
	index = 0;
	cpufreq_frequency_table_target(policy, dbs_info->freq_table, freq_avg,
			CPUFREQ_RELATION_H, &index);
	freq_lo = dbs_info->freq_table[index].frequency;
	index = 0;
	cpufreq_frequency_table_target(policy, dbs_info->freq_table, freq_avg,
			CPUFREQ_RELATION_L, &index);
	freq_hi = dbs_info->freq_table[index].frequency;

	/* Find out how long we have to be in hi and lo freqs */
	if (freq_hi == freq_lo) {
		dbs_info->freq_lo = 0;
		dbs_info->freq_lo_jiffies = 0;
		return freq_lo;
	}
	jiffies_total = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);
	jiffies_hi = (freq_avg - freq_lo) * jiffies_total;
	jiffies_hi += ((freq_hi - freq_lo) / 2);
	jiffies_hi /= (freq_hi - freq_lo);
	jiffies_lo = jiffies_total - jiffies_hi;
	dbs_info->freq_lo = freq_lo;
	dbs_info->freq_lo_jiffies = jiffies_lo;
	dbs_info->freq_hi_jiffies = jiffies_hi;
	return freq_hi;
}

static void ondemand_powersave_bias_init_cpu(int cpu)
{
	struct cpu_dbs_info_s *dbs_info = &per_cpu(od_cpu_dbs_info, cpu);
	dbs_info->freq_table = cpufreq_frequency_get_table(cpu);
	dbs_info->freq_lo = 0;
}

static void ondemand_powersave_bias_init(void)
{
	int i;
	for_each_online_cpu(i) {
		ondemand_powersave_bias_init_cpu(i);
	}
}

/************************** sysfs interface ************************/

static ssize_t show_sampling_rate_max(struct kobject *kobj,
				      struct attribute *attr, char *buf)
{
	printk_once(KERN_INFO "CPUFREQ: ondemand sampling_rate_max "
	       "sysfs file is deprecated - used by: %s\n", current->comm);
	return sprintf(buf, "%u\n", -1U);
}

static ssize_t show_sampling_rate_min(struct kobject *kobj,
				      struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", min_sampling_rate);
}

define_one_global_ro(sampling_rate_max);
define_one_global_ro(sampling_rate_min);

/* cpufreq_ondemand Governor Tunables */
#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)              \
{									\
	return sprintf(buf, "%u\n", dbs_tuners_ins.object);		\
}
show_one(sampling_rate, sampling_rate);
show_one(io_is_busy, io_is_busy);
show_one(up_threshold, up_threshold);
show_one(down_differential, down_differential);
show_one(sampling_down_factor, sampling_down_factor);
show_one(ignore_nice_load, ignore_nice);
show_one(powersave_bias, powersave_bias);
show_one(max_tickle_window, max_tickle_window);

/*** delete after deprecation time ***/

#define DEPRECATION_MSG(file_name)					\
	printk_once(KERN_INFO "CPUFREQ: Per core ondemand sysfs "	\
		    "interface is deprecated - " #file_name "\n");

#define show_one_old(file_name)						\
static ssize_t show_##file_name##_old					\
(struct cpufreq_policy *unused, char *buf)				\
{									\
	printk_once(KERN_INFO "CPUFREQ: Per core ondemand sysfs "	\
		    "interface is deprecated - " #file_name "\n");	\
	return show_##file_name(NULL, NULL, buf);			\
}
show_one_old(sampling_rate);
show_one_old(up_threshold);
show_one_old(ignore_nice_load);
show_one_old(powersave_bias);
show_one_old(sampling_rate_min);
show_one_old(sampling_rate_max);
show_one_old(down_differential);
show_one_old(max_tickle_window);

cpufreq_freq_attr_ro_old(sampling_rate_min);
cpufreq_freq_attr_ro_old(sampling_rate_max);

/*** delete after deprecation time ***/

static ssize_t store_sampling_rate(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&dbs_mutex);
	dbs_tuners_ins.sampling_rate = max(input, min_sampling_rate);
	mutex_unlock(&dbs_mutex);

	return count;
}

static ssize_t store_io_is_busy(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&dbs_mutex);
	dbs_tuners_ins.io_is_busy = !!input;
	mutex_unlock(&dbs_mutex);

	return count;
}

static ssize_t store_up_threshold(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_FREQUENCY_UP_THRESHOLD ||
			input < MIN_FREQUENCY_UP_THRESHOLD) {
		return -EINVAL;
	}

	mutex_lock(&dbs_mutex);
	dbs_tuners_ins.up_threshold = input;
	mutex_unlock(&dbs_mutex);

	return count;
}

static ssize_t store_down_differential(struct kobject *a, struct attribute *b,
		const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	mutex_lock(&dbs_mutex);
	if (ret != 1 || input >= dbs_tuners_ins.up_threshold ||
			input < MIN_FREQUENCY_DOWN_DIFFERENTIAL) {
		mutex_unlock(&dbs_mutex);
		return -EINVAL;
	}

	dbs_tuners_ins.down_differential = input;
	mutex_unlock(&dbs_mutex);

	return count;
}

static ssize_t store_sampling_down_factor(struct kobject *a,
			struct attribute *b, const char *buf, size_t count)
{
	unsigned int input, j;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_SAMPLING_DOWN_FACTOR || input < 1)
		return -EINVAL;
	mutex_lock(&dbs_mutex);
	dbs_tuners_ins.sampling_down_factor = input;

	/* Reset down sampling multiplier in case it was active */
	for_each_online_cpu(j) {
		struct cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->rate_mult = 1;
	}
	mutex_unlock(&dbs_mutex);

	return count;
}

static ssize_t store_ignore_nice_load(struct kobject *a, struct attribute *b,
				      const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	mutex_lock(&dbs_mutex);
	if (input == dbs_tuners_ins.ignore_nice) { /* nothing to do */
		mutex_unlock(&dbs_mutex);
		return count;
	}
	dbs_tuners_ins.ignore_nice = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
						&dbs_info->prev_cpu_wall);
		if (dbs_tuners_ins.ignore_nice)
			dbs_info->prev_cpu_nice = kstat_cpu(j).cpustat.nice;

	}
	mutex_unlock(&dbs_mutex);

	return count;
}

static ssize_t store_powersave_bias(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	if (input > 1000)
		input = 1000;

	mutex_lock(&dbs_mutex);
	dbs_tuners_ins.powersave_bias = input;
	ondemand_powersave_bias_init();
	mutex_unlock(&dbs_mutex);

	return count;
}

static ssize_t store_max_tickle_window(struct kobject *a, struct attribute *b,
		const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	if (input > MAX_TICKLE_WINDOW)
		input = MAX_TICKLE_WINDOW;

	mutex_lock(&dbs_mutex);
	dbs_tuners_ins.max_tickle_window = input;
	mutex_unlock(&dbs_mutex);

	return count;
}


define_one_global_rw(sampling_rate);
define_one_global_rw(io_is_busy);
define_one_global_rw(up_threshold);
define_one_global_rw(down_differential);
define_one_global_rw(sampling_down_factor);
define_one_global_rw(ignore_nice_load);
define_one_global_rw(powersave_bias);
define_one_global_rw(max_tickle_window);

static struct attribute *dbs_attributes[] = {
	&sampling_rate_max.attr,
	&sampling_rate_min.attr,
	&sampling_rate.attr,
	&up_threshold.attr,
	&down_differential.attr,
	&sampling_down_factor.attr,
	&ignore_nice_load.attr,
	&powersave_bias.attr,
	&io_is_busy.attr,
	&max_tickle_window.attr,
	NULL
};

static struct attribute_group dbs_attr_group = {
	.attrs = dbs_attributes,
	.name = "ondemandtcl",
};

/*** delete after deprecation time ***/

#define write_one_old(file_name)					\
static ssize_t store_##file_name##_old					\
(struct cpufreq_policy *unused, const char *buf, size_t count)		\
{									\
       printk_once(KERN_INFO "CPUFREQ: Per core ondemand sysfs "	\
		   "interface is deprecated - " #file_name "\n");	\
       return store_##file_name(NULL, NULL, buf, count);		\
}
write_one_old(sampling_rate);
write_one_old(up_threshold);
write_one_old(ignore_nice_load);
write_one_old(powersave_bias);
write_one_old(down_differential);
write_one_old(max_tickle_window);

cpufreq_freq_attr_rw_old(sampling_rate);
cpufreq_freq_attr_rw_old(up_threshold);
cpufreq_freq_attr_rw_old(ignore_nice_load);
cpufreq_freq_attr_rw_old(powersave_bias);
cpufreq_freq_attr_rw_old(down_differential);
cpufreq_freq_attr_rw_old(max_tickle_window);

static struct attribute *dbs_attributes_old[] = {
       &sampling_rate_max_old.attr,
       &sampling_rate_min_old.attr,
       &sampling_rate_old.attr,
       &up_threshold_old.attr,
       &ignore_nice_load_old.attr,
       &powersave_bias_old.attr,
       &max_tickle_window_old.attr,
       &down_differential_old.attr,
       NULL
};

static struct attribute_group dbs_attr_group_old = {
       .attrs = dbs_attributes_old,
       .name = "ondemandtcl",
};

/*** delete after deprecation time ***/

/************************** sysfs end ************************/

static void __attribute__((unused))
dbs_freq_increase(struct cpufreq_policy *p, unsigned int freq)
{
	if (dbs_tuners_ins.powersave_bias)
		freq = powersave_bias_target(p, freq, CPUFREQ_RELATION_H);
	else if (p->cur == p->max)
		return;

	__cpufreq_driver_target(p, freq, dbs_tuners_ins.powersave_bias ?
			CPUFREQ_RELATION_L : CPUFREQ_RELATION_H);
}

static void record_sample(unsigned int cur_freq, unsigned int target_freq,
		int load, int cpu, struct raw_load_values *rlvp)
{
	struct sample_stats *stats;
	int i;

	/* abuse a module parameter to trigger clearing the sample buffer. Note
	 * that after setting the parameter the buffer won't get cleared until
	 * a sample is made!
	 */
	if (clear_samples) {
		for_each_present_cpu(i) {
			stats = &per_cpu(sample_stats, i);
			if (stats->samples)
				memset(stats->samples, 0, sizeof(struct sample_data) * NUM_SAMPLES);
			stats->current_sample = 0;
		}
		clear_samples = 0;
	}

	if (!sampling_enabled)
		return;

	stats = &per_cpu(sample_stats, cpu);

	if (!stats->samples)
		return;

	i = stats->current_sample;
	/* for timestamp, use same timer as printk does. CPU0 and
	 * CPU1 appear to match to fractions of a millisecond
	 */
	stats->samples[i].timestamp = cpu_clock(0);
	stats->samples[i].user = kstat_cpu(cpu).cpustat.user;
	stats->samples[i].system = kstat_cpu(cpu).cpustat.system;
	stats->samples[i].wall = rlvp ? rlvp->wall : 0;
	stats->samples[i].idle = rlvp ? rlvp->idle : 0;
	stats->samples[i].iowait = rlvp ? rlvp->iowait : 0;
	stats->samples[i].cur_freq = cur_freq;
	stats->samples[i].target_freq = target_freq;
	stats->samples[i].load = load;
	stats->current_sample = (i + 1) % NUM_SAMPLES;
}

static void do_tickle_state_change(struct work_struct *work)
{
	int cpu;
	unsigned long flags;
	int active, active_tickle, active_count;
	unsigned long tickle_jiffies;

	spin_lock_irqsave(&tickle_state.lock, flags);

	active = tickle_state.active;
	active_count = tickle_state.active_count;
	active_tickle = tickle_state.active_tickle;
	tickle_jiffies = tickle_state.tickle_jiffies;

	spin_unlock_irqrestore(&tickle_state.lock, flags);

	if (print_tickles)
		printk("%s: active=%u, active_count=%u, tickle_jiffies=%lu\n",
				__func__, active, active_count, tickle_jiffies);

	if (active_count) {
		if (!active) {
			for_each_online_cpu(cpu) {
				struct cpu_dbs_info_s *dbs_info;
				struct cpufreq_policy *policy;

				mutex_lock(&dbs_mutex);
				dbs_info = &per_cpu(od_cpu_dbs_info, cpu);

				if (!dbs_info->enable) {
					mutex_unlock(&dbs_mutex);
					continue;
				}

				mutex_lock(&dbs_info->timer_mutex);
				policy = dbs_info->cur_policy;
				/* if we don't have a policy then we probably
				 * got tickled before setup completed
				 */

				if (!policy) {
					mutex_unlock(&dbs_info->timer_mutex);
					mutex_unlock(&dbs_mutex);
					continue;
				}

				/* ramp up to the policy max */
				if (policy->cur < policy->max) {
					record_sample(policy->cur, policy->max,
						 -21, policy->cpu, NULL);
					__cpufreq_driver_target(policy, policy->max, CPUFREQ_RELATION_H);
				}

				dbs_info->tickle_active = 1;

				mutex_unlock(&dbs_info->timer_mutex);
				mutex_unlock(&dbs_mutex);
			}
		}
		if (active_tickle)
			mod_timer(&tickle_state.tickle_timer, tickle_jiffies);

		active = 1;
	} else if (active) {

		for_each_online_cpu(cpu) {
			struct cpu_dbs_info_s *dbs_info;
			struct cpufreq_policy *policy;

			mutex_lock(&dbs_mutex);
			dbs_info = &per_cpu(od_cpu_dbs_info, cpu);

			if (!dbs_info->enable) {
				mutex_unlock(&dbs_mutex);
				continue;
			}

			mutex_lock(&dbs_info->timer_mutex);
			policy = dbs_info->cur_policy;
			dbs_info->tickle_active = 0;
			adjust_for_load(dbs_info);

			mutex_unlock(&dbs_info->timer_mutex);
			mutex_unlock(&dbs_mutex);
		}

		active = 0;
	}

	/* update tickle_state */
	spin_lock_irqsave(&tickle_state.lock, flags);
	tickle_state.active = active;
	spin_unlock_irqrestore(&tickle_state.lock, flags);
}

void cpufreq_ondemand_tickle_millis(unsigned int millis)
{
	int queue = 0;
	unsigned long flags, expire;

	if (!tickling_enabled)
		return;

	if (millis > dbs_tuners_ins.max_tickle_window)
		millis = dbs_tuners_ins.max_tickle_window;

	expire = jiffies + msecs_to_jiffies(millis);

	/* record it on CPU 0 */
	record_sample(0, millis, -225, 0, NULL);
	spin_lock_irqsave(&tickle_state.lock, flags);

	if (time_after(expire, tickle_state.tickle_jiffies)) {
		tickle_state.tickle_jiffies = expire;

		if (!tickle_state.active_tickle)
			tickle_state.active_count += 1;

		tickle_state.active_tickle = 1;
		queue = 1;
	}

	spin_unlock_irqrestore(&tickle_state.lock, flags);

	if (queue)
		queue_work(kondemand_wq, &tickle_state.tickle_work);
}
EXPORT_SYMBOL(cpufreq_ondemand_tickle_millis);

void cpufreq_ondemand_tickle(void)
{
	cpufreq_ondemand_tickle_millis(dbs_tuners_ins.max_tickle_window);
}
EXPORT_SYMBOL(cpufreq_ondemand_tickle);

void cpufreq_ondemand_hold(void)
{
	unsigned long flags;

	/* record it on CPU 0 */
	record_sample(0, 0, -221, 0, NULL);

	spin_lock_irqsave(&tickle_state.lock, flags);
	tickle_state.active_count += 1;
	spin_unlock_irqrestore(&tickle_state.lock, flags);

	queue_work(kondemand_wq, &tickle_state.tickle_work);
}
EXPORT_SYMBOL(cpufreq_ondemand_hold);

void cpufreq_ondemand_unhold(void)
{
	int queue = 0;
	unsigned long flags;

	/* record it on CPU 0 */
	record_sample(0, 0, -220, 0, NULL);

	spin_lock_irqsave(&tickle_state.lock, flags);
	if (tickle_state.active_count) {
		tickle_state.active_count -= 1;
		queue = 1;
	} else {
		printk(KERN_WARNING "%s: attempt to decrement when active_count == 0!\n",
				__func__);
	}

	spin_unlock_irqrestore(&tickle_state.lock, flags);

	if (queue)
		queue_work(kondemand_wq, &tickle_state.tickle_work);
}
EXPORT_SYMBOL(cpufreq_ondemand_unhold);

void cpufreq_ondemand_hold_check(int *flag)
{
	if (!*flag) {
		cpufreq_ondemand_hold();
		*flag = 1;
	}
}
EXPORT_SYMBOL(cpufreq_ondemand_hold_check);

/* this tickle option waits until the CPU is up to the max freq before it returns */
void cpufreq_ondemand_hold_sync(void)
{
	cpufreq_ondemand_hold();
	flush_work(&tickle_state.tickle_work);
}
EXPORT_SYMBOL(cpufreq_ondemand_hold_sync);

void cpufreq_ondemand_unhold_check(int *flag)
{
	if (*flag) {
		cpufreq_ondemand_unhold();
		*flag = 0;
	}
}
EXPORT_SYMBOL(cpufreq_ondemand_unhold_check);

static void do_floor_state_change(struct work_struct *work)
{
	int cpu, floor_active, floor_count, cur_freq_floor, pending_freq_floor;
	unsigned long flags;

	spin_lock_irqsave(&tickle_state.lock, flags);

	floor_active = tickle_state.floor_active;
	floor_count = tickle_state.floor_count;
	cur_freq_floor = tickle_state.cur_freq_floor;

	pending_freq_floor = get_max_client_floor();

	spin_unlock_irqrestore(&tickle_state.lock, flags);


	if (print_tickles)
		printk("%s: floor_active=%u, floor_count=%u, "
				"cur_freq_floor=%u, pending_freq_floor=%u\n",
				__func__, floor_active, floor_count,
				cur_freq_floor, pending_freq_floor);

	if (floor_count) {
		if (!floor_active || pending_freq_floor != cur_freq_floor) {

			for_each_online_cpu(cpu) {
				unsigned int f = pending_freq_floor;
				struct cpu_dbs_info_s *dbs_info;
				struct cpufreq_policy *policy;

				mutex_lock(&dbs_mutex);
				dbs_info = &per_cpu(od_cpu_dbs_info, cpu);

				if (!dbs_info->enable) {
					mutex_unlock(&dbs_mutex);
					continue;
				}

				mutex_lock(&dbs_info->timer_mutex);

				policy = dbs_info->cur_policy;

				if (!policy) {
					mutex_unlock(&dbs_info->timer_mutex);
					mutex_unlock(&dbs_mutex);
					continue;
				}

				if (f < policy->min)
					f = policy->min;

				if (f > policy->max)
					f = policy->max;

				dbs_info->freq_floor = f;
				dbs_info->floor_active = 1;

				if (policy->cur < f) {
					record_sample(policy->cur, f, -31,
						policy->cpu, NULL);
					__cpufreq_driver_target(policy, f,
						CPUFREQ_RELATION_L);
				}

				mutex_unlock(&dbs_info->timer_mutex);
				mutex_unlock(&dbs_mutex);
			}

		}

		floor_active = 1;
		cur_freq_floor = pending_freq_floor;
	} else if (floor_active) {

		for_each_online_cpu(cpu) {
			struct cpu_dbs_info_s *dbs_info;
			struct cpufreq_policy *policy;

			mutex_lock(&dbs_mutex);
			dbs_info = &per_cpu(od_cpu_dbs_info, cpu);

			if (!dbs_info->enable) {
				mutex_unlock(&dbs_mutex);
				continue;
			}

			mutex_lock(&dbs_info->timer_mutex);
			policy = dbs_info->cur_policy;
			dbs_info->floor_active = 0;
			adjust_for_load(dbs_info);

			mutex_unlock(&dbs_info->timer_mutex);
			mutex_unlock(&dbs_mutex);
		}


		floor_active = 0;
		cur_freq_floor = 0;
	}

	spin_lock_irqsave(&tickle_state.lock, flags);
	tickle_state.floor_active = floor_active;
	tickle_state.cur_freq_floor = cur_freq_floor;
	spin_unlock_irqrestore(&tickle_state.lock, flags);
}


void cpufreq_ondemand_floor_hold(struct tickle_file_data *tfdp)
{
	unsigned long flags;
	unsigned int freq;

	int cpu, index = 0;

	struct cpu_dbs_info_s *dbs_info;

	/* record it on CPU 0 */
	record_sample(0, tfdp->floor_freq, -231, 0, NULL);

	/* it is reasonable for a client to set a floor that is not an actual
	 * CPU frequency, but that will cause a bit of flailing on each sample.
	 * Round it up (once) as necessary here.
	 */
	cpu  = smp_processor_id();
	dbs_info = &per_cpu(od_cpu_dbs_info, cpu);
	cpufreq_frequency_table_target(dbs_info->cur_policy,
		dbs_info->freq_table, tfdp->floor_freq, CPUFREQ_RELATION_L,
		&index);
	freq = dbs_info->freq_table[index].frequency;

	spin_lock_irqsave(&tickle_state.lock, flags);
	tickle_state.floor_count += 1;
	if (tfdp->floor_freq != freq) {
		pr_debug("%s: changing floor from %u to %u\n",
			__func__, tfdp->floor_freq, freq);
		tfdp->floor_freq  = freq ;
	}
	spin_unlock_irqrestore(&tickle_state.lock, flags);

	queue_work(kondemand_wq, &tickle_state.floor_work);
}

void cpufreq_ondemand_floor_unhold(struct tickle_file_data *tfdp)
{
	int queue = 0;
	unsigned long flags;

	/* record it on CPU 0 */
	record_sample(0, tfdp->floor_freq, -230, 0, NULL);

	spin_lock_irqsave(&tickle_state.lock, flags);
	tfdp->floor_freq = 0;

	if (tickle_state.floor_count) {
		tickle_state.floor_count -= 1;
		queue = 1;
	} else {
		printk(KERN_WARNING "%s: attempt to decrement when floor_count == 0!\n",
				__func__);
	}

	spin_unlock_irqrestore(&tickle_state.lock, flags);

	if (queue)
		queue_work(kondemand_wq, &tickle_state.floor_work);
}

void cpufreq_ondemand_floor_hold_check(struct tickle_file_data *tfdp,
	int *flag)
{
	if (!*flag) {
		cpufreq_ondemand_floor_hold(tfdp);
		*flag = 1;
	}
}

void cpufreq_ondemand_floor_unhold_check(struct tickle_file_data *tfdp,
	int *flag)
{
	if (*flag) {
		cpufreq_ondemand_floor_unhold(tfdp);
		*flag = 0;
	}
}

/* allow a single kernel held floor using kernel_floor_data.
 * Having more than one kernel floor will require a mechanism
 * for creating and unregistering clients.
 */
int cpufreq_ondemand_floor_hold_kernel(unsigned int freq)
{
	unsigned long flags;
	if (freq == 0)
		return -EINVAL;
	spin_lock_irqsave(&tickle_state.lock, flags);
	if (kernel_floor_data.floor_freq) {	/* in use */
		spin_unlock_irqrestore(&tickle_state.lock, flags);
		return -EBUSY;
	}
	kernel_floor_data.floor_freq = freq;
	spin_unlock_irqrestore(&tickle_state.lock, flags);

	cpufreq_ondemand_floor_hold(&kernel_floor_data);
	return 0;

}
EXPORT_SYMBOL(cpufreq_ondemand_floor_hold_kernel);

int cpufreq_ondemand_floor_unhold_kernel(void)
{
	unsigned long flags;
	spin_lock_irqsave(&tickle_state.lock, flags);
	if (kernel_floor_data.floor_freq == 0) {
		spin_unlock_irqrestore(&tickle_state.lock, flags);
		return -EINVAL;
	}
	spin_unlock_irqrestore(&tickle_state.lock, flags);

	cpufreq_ondemand_floor_unhold(&kernel_floor_data);

	return 0;

}
EXPORT_SYMBOL(cpufreq_ondemand_floor_unhold_kernel);



/**
* @brief
*
* Should be called with the timer_mutex already held.
* dbs_check_cpu will be called every sample even when a tickle
* or floor is held (so the load data is always current), but if
* a tickle is held it will not change the frequency on that sample.
*
* @param  this_dbs_info
*/
static void dbs_check_cpu(struct cpu_dbs_info_s *this_dbs_info)
{
	unsigned int max_load_freq;

	struct cpufreq_policy *policy;
	unsigned int j;

	this_dbs_info->cur_load = 0;

	this_dbs_info->freq_lo = 0;
	policy = this_dbs_info->cur_policy;

	/*
	 * Every sampling_rate, we check, if current idle time is less
	 * than 20% (default), then we try to increase frequency
	 * Every sampling_rate, we look for a the lowest
	 * frequency which can sustain the load while keeping idle time over
	 * 30%. If such a frequency exist, we try to decrease to this frequency.
	 *
	 * Any frequency increase takes it to the maximum frequency.
	 * Frequency reduction happens at minimum steps of
	 * 5% (default) of current frequency
	 */

	/* Get Absolute Load - in terms of freq */
	max_load_freq = 0;

	for_each_cpu(j, policy->cpus) {
		struct cpu_dbs_info_s *j_dbs_info;
		cputime64_t cur_wall_time, cur_idle_time, cur_iowait_time;
		unsigned int idle_time, wall_time, iowait_time;
		unsigned int load, load_freq;
		int freq_avg;

		j_dbs_info = &per_cpu(od_cpu_dbs_info, j);

		cur_idle_time = get_cpu_idle_time(j, &cur_wall_time);
		cur_iowait_time = get_cpu_iowait_time(j, &cur_wall_time);

		wall_time = (unsigned int) cputime64_sub(cur_wall_time,
				j_dbs_info->prev_cpu_wall);
		j_dbs_info->prev_cpu_wall = cur_wall_time;

		idle_time = (unsigned int) cputime64_sub(cur_idle_time,
				j_dbs_info->prev_cpu_idle);
		j_dbs_info->prev_cpu_idle = cur_idle_time;

		iowait_time = (unsigned int) cputime64_sub(cur_iowait_time,
				j_dbs_info->prev_cpu_iowait);
		j_dbs_info->prev_cpu_iowait = cur_iowait_time;

		if (dbs_tuners_ins.ignore_nice) {
			cputime64_t cur_nice;
			unsigned long cur_nice_jiffies;

			cur_nice = cputime64_sub(kstat_cpu(j).cpustat.nice,
					 j_dbs_info->prev_cpu_nice);
			/*
			 * Assumption: nice time between sampling periods will
			 * be less than 2^32 jiffies for 32 bit sys
			 */
			cur_nice_jiffies = (unsigned long)
					cputime64_to_jiffies64(cur_nice);

			j_dbs_info->prev_cpu_nice = kstat_cpu(j).cpustat.nice;
			idle_time += jiffies_to_usecs(cur_nice_jiffies);

		}

		/*
		 * For the purpose of ondemand, waiting for disk IO is an
		 * indication that you're performance critical, and not that
		 * the system is actually idle. So subtract the iowait time
		 * from the cpu idle time.
		 */

		if (dbs_tuners_ins.io_is_busy && idle_time >= iowait_time)
			idle_time -= iowait_time;

		if (unlikely(!wall_time || wall_time < idle_time))
			continue;

		load = 100 * (wall_time - idle_time) / wall_time;
		this_dbs_info->cur_load = load;

		/* stash values for later sample storage */
		if (sampling_enabled) {
			j_dbs_info->rlv.wall = wall_time;
			j_dbs_info->rlv.idle = idle_time;
			j_dbs_info->rlv.iowait = iowait_time;
		}

		freq_avg = __cpufreq_driver_getavg(policy, j);
		if (freq_avg <= 0)
			freq_avg = policy->cur;

		load_freq = load * freq_avg;
		if (load_freq > max_load_freq)
			max_load_freq = load_freq;
	}
	this_dbs_info->max_load_freq = max_load_freq;
}

/**
* @brief
*
* Should be called with the timer_mutex already held.
*
* @param  this_dbs_info
*/
static void adjust_for_load(struct cpu_dbs_info_s *this_dbs_info)
{
	unsigned int		max_load_freq;
	unsigned int		load;
	struct cpufreq_policy*	policy;

	if (this_dbs_info->tickle_active) {
		return;
	}

	max_load_freq	= this_dbs_info->max_load_freq;
	load		= this_dbs_info->cur_load;
	policy		= this_dbs_info->cur_policy;
	/* Check for frequency increase */
	if (max_load_freq > dbs_tuners_ins.up_threshold * policy->cur) {
		/* If switching to max speed, apply sampling_down_factor */
		if (policy->cur < policy->max)
			this_dbs_info->rate_mult =
				dbs_tuners_ins.sampling_down_factor;
		/* if we are already at full speed then break out early */
		if (!dbs_tuners_ins.powersave_bias) {
			if (policy->cur == policy->max)
				return;


			record_sample(policy->cur, policy->max, load,
				policy->cpu, &this_dbs_info->rlv);
			__cpufreq_driver_target(policy, policy->max,
				CPUFREQ_RELATION_H);
		} else {
			int freq = powersave_bias_target(policy, policy->max,
					CPUFREQ_RELATION_H);

			if (this_dbs_info->floor_active) {
				if (freq <= this_dbs_info->freq_floor) {
					freq = this_dbs_info->freq_floor;
				}
			}
			if (policy->cur != freq)
				record_sample(policy->cur, freq, load,
					policy->cpu, &this_dbs_info->rlv);
			__cpufreq_driver_target(policy, freq,
				CPUFREQ_RELATION_L);
		}
		return;
	}

	/* Check for frequency decrease */
	/* if we cannot reduce the frequency anymore, break out early */
	if (policy->cur == policy->min)
		return;

	/*
	 * The optimal frequency is the frequency that is the lowest that
	 * can support the current CPU usage without triggering the up
	 * policy. To be safe, we focus 10 points under the threshold.
	 */
	if (max_load_freq <
	    (dbs_tuners_ins.up_threshold - dbs_tuners_ins.down_differential) *
	     policy->cur) {
		unsigned int freq_next;
		freq_next = max_load_freq /
				(dbs_tuners_ins.up_threshold -
				 dbs_tuners_ins.down_differential);

		/* No longer fully busy, reset rate_mult */
		this_dbs_info->rate_mult = 1;

		if (freq_next < policy->min)
			freq_next = policy->min;

		if (!dbs_tuners_ins.powersave_bias) {
			if (this_dbs_info->floor_active) {
				if (freq_next <= this_dbs_info->freq_floor) {
					freq_next = this_dbs_info->freq_floor;
				}
			}
			if (policy->cur != freq_next)
				record_sample(policy->cur, freq_next, load,
					policy->cpu, &this_dbs_info->rlv);
			__cpufreq_driver_target(policy, freq_next,
					CPUFREQ_RELATION_L);
		} else {
			int freq = powersave_bias_target(policy, freq_next,
					CPUFREQ_RELATION_L);
			if (this_dbs_info->floor_active) {
				if (freq <= this_dbs_info->freq_floor) {
					freq = this_dbs_info->freq_floor;
				}
			}
			if (policy->cur != freq)
				record_sample(policy->cur, freq, load,
					policy->cpu, &this_dbs_info->rlv);
			__cpufreq_driver_target(policy, freq,
				CPUFREQ_RELATION_L);
		}
	}
}

static void do_dbs_timer(struct work_struct *work)
{
	struct cpu_dbs_info_s *dbs_info =
		container_of(work, struct cpu_dbs_info_s, work.work);
	unsigned int cpu = dbs_info->cpu;
	int sample_type = dbs_info->sample_type;

	/* We want all CPUs to do sampling nearly on same jiffy */
	int delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate
		* dbs_info->rate_mult);

	if (num_online_cpus() > 1)
		delay -= jiffies % delay;

	mutex_lock(&dbs_info->timer_mutex);

	/* Common NORMAL_SAMPLE setup */
	dbs_info->sample_type = DBS_NORMAL_SAMPLE;
	if (!dbs_tuners_ins.powersave_bias ||
	    sample_type == DBS_NORMAL_SAMPLE) {
		dbs_check_cpu(dbs_info);
		adjust_for_load(dbs_info);
		if (dbs_info->freq_lo) {
			/* Setup timer for SUB_SAMPLE */
			dbs_info->sample_type = DBS_SUB_SAMPLE;
			delay = dbs_info->freq_hi_jiffies;
		}
	} else {
		record_sample(dbs_info->cur_policy->cur, dbs_info->freq_lo,
			-1, cpu, NULL);
		__cpufreq_driver_target(dbs_info->cur_policy,
			dbs_info->freq_lo, CPUFREQ_RELATION_H);
	}
	queue_delayed_work_on(cpu, kondemand_wq, &dbs_info->work, delay);
	mutex_unlock(&dbs_info->timer_mutex);
}

static void do_tickle_timer(unsigned long arg)
{
	unsigned long flags;

	spin_lock_irqsave(&tickle_state.lock, flags);

	if (tickle_state.active_count) {
		if (tickle_state.active_tickle) {
			tickle_state.active_count -= 1;
			tickle_state.active_tickle = 0;
		}
	} else {
		printk(KERN_WARNING "%s: attempt to decrement when active_count == 0!\n",
				__func__);
	}

	spin_unlock_irqrestore(&tickle_state.lock, flags);

	queue_work(kondemand_wq, &tickle_state.tickle_work);
}

static inline void dbs_timer_init(struct cpu_dbs_info_s *dbs_info)
{
	/* We want all CPUs to do sampling nearly on same jiffy */
	int delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);

	if (num_online_cpus() > 1)
		delay -= jiffies % delay;

	dbs_info->enable = 1;
	dbs_info->sample_type = DBS_NORMAL_SAMPLE;
	INIT_DELAYED_WORK_DEFERRABLE(&dbs_info->work, do_dbs_timer);
	queue_delayed_work_on(dbs_info->cpu, kondemand_wq, &dbs_info->work,
		delay);
	dbs_info->tickle_active = 0;
	dbs_info->floor_active = 0;
}

static inline void dbs_timer_exit(struct cpu_dbs_info_s *dbs_info)
{
	dbs_info->enable = 0;
	cancel_delayed_work_sync(&dbs_info->work);
}

/*
 * Not all CPUs want IO time to be accounted as busy; this dependson how
 * efficient idling at a higher frequency/voltage is.
 * Pavel Machek says this is not so for various generations of AMD and old
 * Intel systems.
 * Mike Chan (androidlcom) calis this is also not true for ARM.
 * Because of this, whitelist specific known (series) of CPUs by default, and
 * leave all others up to the user.
 */
static int should_io_be_busy(void)
{
#if defined(CONFIG_X86)
	/*
	 * For Intel, Core 2 (model 15) andl later have an efficient idle.
	 */
	if (boot_cpu_data.x86_vendor == X86_VENDOR_INTEL &&
	    boot_cpu_data.x86 == 6 &&
	    boot_cpu_data.x86_model >= 15)
		return 1;
#endif
	return 0;
}

static void dbs_refresh_callback(struct work_struct *unused)
{
	struct cpufreq_policy *policy;
	struct cpu_dbs_info_s *this_dbs_info;

	mutex_lock(&dbs_mutex);
	this_dbs_info = &per_cpu(od_cpu_dbs_info, 0);

	if (!this_dbs_info->enable) {
		mutex_unlock(&dbs_mutex);
		return;
	}

	mutex_lock(&this_dbs_info->timer_mutex);
	policy = this_dbs_info->cur_policy;

	record_sample(0, 0, -251, policy->cpu, NULL);
	if (policy->cur < policy->max) {
		record_sample(policy->cur, policy->max, -51, policy->cpu, NULL);
		policy->cur = policy->max;

		__cpufreq_driver_target(policy, policy->max,
					CPUFREQ_RELATION_L);
		this_dbs_info->prev_cpu_idle = get_cpu_idle_time(0,
				&this_dbs_info->prev_cpu_wall);
	}
	mutex_unlock(&this_dbs_info->timer_mutex);
	mutex_unlock(&dbs_mutex);
}

static DECLARE_WORK(dbs_refresh_work, dbs_refresh_callback);

static void dbs_input_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{
	schedule_work_on(0, &dbs_refresh_work);
}

static int dbs_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpufreq";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void dbs_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id dbs_ids[] = {
	{ .driver_info = 1 },
	{ },
};

static struct input_handler dbs_input_handler = {
	.event		= dbs_input_event,
	.connect	= dbs_input_connect,
	.disconnect	= dbs_input_disconnect,
	.name		= "cpufreq_ond_tcl",
	.id_table	= dbs_ids,
};

/* support for logging CPU hotplug in the sample file. Getting the
 * frequency right is a little messy, hence the separate notifiers
 * with different priorities. Only used if logging is enabled.
 */
static int cpufreq_hotplug_online_callback(struct notifier_block *nfb,
					   unsigned long action,
					   void *hcpu)
{	unsigned int cpu = (unsigned long)hcpu;
	switch (action) {
	case CPU_ONLINE:
		record_sample(0, cpufreq_quick_get(cpu), -501, cpu, NULL);
		break;
	}
	return NOTIFY_OK;
}

static int cpufreq_hotplug_offline_callback(struct notifier_block *nfb,
					   unsigned long action,
					   void *hcpu)
{	unsigned int cpu = (unsigned long)hcpu;
	switch (action) {
	case CPU_DOWN_PREPARE:
		record_sample(cpufreq_quick_get(cpu), 0, -500, cpu, NULL);
		break;
	}
	return NOTIFY_OK;
}


static struct notifier_block cpufreq_online_notifier = {
	.notifier_call = cpufreq_hotplug_online_callback,
	.priority = 0,
};

static struct notifier_block cpufreq_offline_notifier = {
	.notifier_call = cpufreq_hotplug_offline_callback,
	.priority = 1,
};

static int cpufreq_governor_dbs(struct cpufreq_policy *policy,
				   unsigned int event)
{
	unsigned int cpu = policy->cpu;
	struct cpu_dbs_info_s *this_dbs_info;
	unsigned int j;
	int rc;
	struct proc_dir_entry *entry;

	this_dbs_info = &per_cpu(od_cpu_dbs_info, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!policy->cur))
			return -EINVAL;

		mutex_lock(&dbs_mutex);

		rc = sysfs_create_group(&policy->kobj, &dbs_attr_group_old);
		if (rc) {
			mutex_unlock(&dbs_mutex);
			return rc;
		}

		dbs_enable++;
		for_each_cpu(j, policy->cpus) {
			struct cpu_dbs_info_s *j_dbs_info;
			j_dbs_info = &per_cpu(od_cpu_dbs_info, j);
			j_dbs_info->cur_policy = policy;

			j_dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
						&j_dbs_info->prev_cpu_wall);
			if (dbs_tuners_ins.ignore_nice) {
				j_dbs_info->prev_cpu_nice =
						kstat_cpu(j).cpustat.nice;
			}
		}
		this_dbs_info->cpu = cpu;
		this_dbs_info->rate_mult = 1;
		ondemand_powersave_bias_init_cpu(cpu);
		/* sampling must be enabled prior to becoming the active governor */
		if (sampling_enabled) {
			for_each_cpu(j, policy->cpus) {
				struct sample_stats *stats = &per_cpu(sample_stats, j);

				// if allocation fails, sampling is disabled on this cpu
				if (!stats->samples) {
					stats->samples = vmalloc(sizeof(struct sample_data) * NUM_SAMPLES);
					if (!stats->samples)
						continue;

					/* if it's a  new allocation,  touch
					 * each page to force allocation of
					 * physical pages. This also initializes
					 * the structure to 0.
					 */
					memset(stats->samples, 0, sizeof(struct sample_data) *
						NUM_SAMPLES);
					stats->current_sample = 0;
				}
			}
		}

		/*
		 * Start the timerschedule work and create procfs entry
		 * for sampling statistics when this governor is used for
		 * the first time.
		 */
		if (dbs_enable == 1) {
			unsigned int latency;

			entry = create_proc_entry("ondemandtcl_samples", 0444, NULL);
			if (entry)
				entry->proc_fops = &proc_stats_operations;
			if (sampling_enabled) {
				int rc1 = 0;
				rc = register_hotcpu_notifier(&cpufreq_online_notifier);
				if (rc == 0) {
					rc1 = register_hotcpu_notifier(&cpufreq_offline_notifier);
					if (rc1)
						unregister_hotcpu_notifier(&cpufreq_online_notifier);
				}

				if (rc || rc1)
					pr_err("ondemandtcl not logging CPU hotplug\n");
			}

			rc = sysfs_create_group(cpufreq_global_kobject,
						&dbs_attr_group);
			if (rc) {
				mutex_unlock(&dbs_mutex);
				return rc;
			}

			/* policy latency is in nS. Convert it to uS first */
			latency = policy->cpuinfo.transition_latency / 1000;
			if (latency == 0)
				latency = 1;
			/* Bring kernel and HW constraints together */
			min_sampling_rate = max(min_sampling_rate,
					MIN_LATENCY_MULTIPLIER * latency);
			dbs_tuners_ins.sampling_rate =
				max(min_sampling_rate,
				    latency * LATENCY_MULTIPLIER);

			/* Until user-space startup scripts are set up,
			 * override it here
			 */
			if (DEF_SAMPLING_RATE > 0)
				dbs_tuners_ins.sampling_rate = DEF_SAMPLING_RATE;

			dbs_tuners_ins.io_is_busy = should_io_be_busy();
		}
		if (!cpu)
			rc = input_register_handler(&dbs_input_handler);
		mutex_unlock(&dbs_mutex);

		mutex_init(&this_dbs_info->timer_mutex);
		dbs_timer_init(this_dbs_info);
		break;

	case CPUFREQ_GOV_STOP:
		dbs_timer_exit(this_dbs_info);

		mutex_lock(&dbs_mutex);
		sysfs_remove_group(&policy->kobj, &dbs_attr_group_old);
		mutex_destroy(&this_dbs_info->timer_mutex);
		dbs_enable--;
		if (!cpu)
			input_unregister_handler(&dbs_input_handler);
		if (!dbs_enable) {
			/* don't free sampling data for _any_ CPU until
			 * there's no one using this governor, then free it
			 * all. Implicitly assumes that all CPU's are using
			 * this governor, but will be harmless if not.
			 */
			for_each_possible_cpu(j) {
				struct sample_stats *stats;
				stats = &per_cpu(sample_stats, j);

				if (stats->samples) {
					vfree(stats->samples);
					stats->samples = NULL;
				}
			}
			/* if registration failed earlier, unregistration will
			 * benignly return an ENOENT error code)
			 */
			unregister_hotcpu_notifier(&cpufreq_online_notifier);
			unregister_hotcpu_notifier(&cpufreq_offline_notifier);
			remove_proc_entry("ondemandtcl_samples", NULL);
			sysfs_remove_group(cpufreq_global_kobject,
					   &dbs_attr_group);
		}
		mutex_unlock(&dbs_mutex);
		break;

	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&this_dbs_info->timer_mutex);

		record_sample(policy->min, policy->max, -240,
			policy->cpu, NULL);
		if (policy->max < this_dbs_info->cur_policy->cur) {
			record_sample(policy->cur, policy->max, -40,
				policy->cpu, NULL);
			__cpufreq_driver_target(this_dbs_info->cur_policy,
			                        policy->max,
			                        CPUFREQ_RELATION_H);
		} else if (policy->min > this_dbs_info->cur_policy->cur) {
			record_sample(policy->cur, policy->min, -41,
				policy->cpu, NULL);
			__cpufreq_driver_target(this_dbs_info->cur_policy,
			                        policy->min,
			                        CPUFREQ_RELATION_L);
		}
		mutex_unlock(&this_dbs_info->timer_mutex);
		break;
	}
	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND_TICKLE
static
#endif
struct cpufreq_governor cpufreq_gov_ondemand_tickle = {
	.name			= "ondemandtcl",
	.governor		= cpufreq_governor_dbs,
	.max_transition_latency = TRANSITION_LATENCY_LIMIT,
	.owner			= THIS_MODULE,
};

static int __init cpufreq_gov_dbs_init(void)
{
	int err;
	cputime64_t wall;
	u64 idle_time;
	int cpu = get_cpu();

	idle_time = get_cpu_idle_time_us(cpu, &wall);
	put_cpu();
	if (idle_time != -1ULL) {
		/* Idle micro accounting is supported. Use finer thresholds */
		dbs_tuners_ins.up_threshold = MICRO_FREQUENCY_UP_THRESHOLD;
		dbs_tuners_ins.down_differential =
					MICRO_FREQUENCY_DOWN_DIFFERENTIAL;
		/*
		 * In no_hz/micro accounting case we set the minimum frequency
		 * not depending on HZ, but fixed (very low). The deferred
		 * timer might skip some samples if idle/sleeping as needed.
		*/
		min_sampling_rate = MICRO_FREQUENCY_MIN_SAMPLE_RATE;
	} else {
		/* For correct statistics, we need 10 ticks for each measure */
		min_sampling_rate =
			MIN_SAMPLING_RATE_RATIO * jiffies_to_usecs(10);
	}

	kondemand_wq = create_workqueue("kondemandtcl");
	if (!kondemand_wq) {
		printk(KERN_ERR "Creation of kondemandtcl failed\n");
		return -EFAULT;
	}

	init_timer(&tickle_state.tickle_timer);

	tickle_state.tickle_timer.function = do_tickle_timer;

	INIT_WORK(&tickle_state.tickle_work, do_tickle_state_change);
	INIT_WORK(&tickle_state.floor_work, do_floor_state_change);

	/* init this to current jiffies or short circuiting
	 * doesn't work until jiffies wraps
	 */
	tickle_state.tickle_jiffies =  jiffies;

	err = setup_tickle_device();
	if (err < 0) {
		return err;
	}
	err = cpufreq_register_governor(&cpufreq_gov_ondemand_tickle);
	if (err)
		destroy_workqueue(kondemand_wq);

	return err;
}

static void __exit cpufreq_gov_dbs_exit(void)
{
	remove_tickle_device();
	cpufreq_unregister_governor(&cpufreq_gov_ondemand_tickle);
	/* cancel any pending timers before destroying the work queue */
	del_timer(&tickle_state.tickle_timer);
	destroy_workqueue(kondemand_wq);
}


MODULE_AUTHOR("Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>");
MODULE_AUTHOR("Alexey Starikovskiy <alexey.y.starikovskiy@intel.com>");
MODULE_AUTHOR("Corey Tabaka <corey.tabaka@palm.com>");
MODULE_DESCRIPTION("'cpufreq_ondemand_tickle' - A dynamic cpufreq governor for "
                   "Low Latency Frequency Transition capable processors");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND_TICKLE
fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);
