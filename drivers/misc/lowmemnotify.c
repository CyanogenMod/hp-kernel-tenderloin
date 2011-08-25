/** 
* @file lowmemnotify.c
* 
* Copyright (c) 2008 Palm, Inc. or its subsidiaries.
* All rights reserved.
* 
* @brief A low-memory notification mechanism.
*/
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>

#include <linux/mm.h>
#include <linux/oom.h>
#include <linux/swap.h>
#include <linux/vmstat.h>
#include <linux/hugetlb.h>

#include <linux/sched.h>

#include <linux/spinlock.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/shmem_fs.h>

#include <asm/atomic.h>

#include <linux/lowmemnotify.h>

#define LOWMEMNOTIFY_NAME "lowmemnotify"
//#define MEMNOTIFY_WAKEUP_GUARD  (30*HZ) // 30s ping

#define MB(pages) ((K(pages))/1000)
#define K(pages) ((pages) << (PAGE_SHIFT - 10))

struct memnotify_file_info {
	unsigned long	 last_memnotify;
	int		 last_threshold;

	struct file		*file;
};

static DECLARE_WAIT_QUEUE_HEAD(memnotify_wait);
static atomic_t nr_watcher_task = ATOMIC_INIT(0);

static uint32_t memnotify_debug_level = 2;

enum {
	THRESHOLD_NORMAL,
	THRESHOLD_MEDIUM,
	THRESHOLD_LOW,
	THRESHOLD_CRITICAL,
	THRESHOLD_REBOOT,
};

static const char *_threshold_string[6] = {
	"normal",
	"medium",
	"low",
	"critical",
	"reboot",
};

static const char * threshold_string(int threshold)
{
	if (threshold > ARRAY_SIZE(_threshold_string))
		return "";

	return _threshold_string[threshold];
}

static unsigned long memnotify_messages[6] = {
	MEMNOTIFY_NORMAL,      /* The happy state */
	MEMNOTIFY_MEDIUM,      /* Not so happy, we are getting to a dangerous state */
	MEMNOTIFY_LOW,         /* Userspace drops uneeded caches. */
	MEMNOTIFY_CRITICAL,    /* Userspace should attempt to stop new allocs */
	MEMNOTIFY_REBOOT,      /* Super critical point...we should reboot */
};
static int memnotify_messages_size = 5;

static atomic_t memnotify_last_threshold = ATOMIC_INIT(THRESHOLD_NORMAL);

static atomic_long_t memnotify_last_report =
						ATOMIC_LONG_INIT(INITIAL_JIFFIES);

/** 
* @brief Thresholds are % used of (TOTAL_MEM + TOTAL_SWAP)
*/
static size_t memnotify_enter_thresholds[6] = {
	0,
	90,
	100,
	114,
	120,
};
static int memnotify_enter_thresholds_size = 5;

static size_t memnotify_leave_thresholds[6] = {
	0,
	84,
	94,
	108,
	112,   /* You can't leave the reboot threshold. */
};
static int memnotify_leave_thresholds_size = 5;

#define lowmem_print(level, x...) do { if(memnotify_debug_level >= (level)) printk(x); } while(0)

module_param_array_named(thresholds_enter,
	memnotify_enter_thresholds, uint,
	&memnotify_enter_thresholds_size, S_IRUGO | S_IWUSR);

module_param_array_named(thresholds_leave,
	memnotify_leave_thresholds, uint,
	&memnotify_leave_thresholds_size, S_IRUGO | S_IWUSR);

module_param_named(debug_level,
	memnotify_debug_level, uint, S_IRUGO | S_IWUSR);

/** 
* @brief Wakes up low-memory watchers if state changed.
* 
* @param  free 
* @param  threshold 
* 
* @retval
*/
static int memory_pressure_notify(unsigned long used, int used_ratio, int threshold)
{
	int changed;
	int last_threshold;
	unsigned long now = jiffies;
	
	last_threshold = atomic_read(&memnotify_last_threshold);

	changed = (threshold != last_threshold);
	if (!changed) {
		goto out;
#if 0   /* logic to repeat message periodically.  I don't think we need this.
		 * Apps will get transition from Low -> Normal or Critical -> Low ->
		 * Normal.  Also, apps can read the threshold themselves.
	 	 */
		unsigned long timeout;
		unsigned long last_memnotify;

		last_memnotify = atomic_long_read(&memnotify_last_report);

		/* Rate limit the messages */
		timeout = last_memnotify + MEMNOTIFY_WAKEUP_GUARD;

		if (time_before(now, timeout) || threshold == THRESHOLD_NORMAL)
			goto out;
#endif
	}

	lowmem_print(2, "%s (%d%%, U %ldMB, F %ldMB, %s)\n",
		__FUNCTION__, used_ratio, MB(used), MB(memnotify_get_free()),
		threshold_string(threshold));

	atomic_long_set(&memnotify_last_report, now);
	atomic_set(&memnotify_last_threshold, threshold);

	wake_up_interruptible_all(&memnotify_wait);
	return 0;
out:
	return 1;
}

/** 
* @brief Free memory measurement.
* 
* @retval
*/
unsigned long memnotify_get_free(void)
{
	unsigned long free;
	unsigned long other_free;
	
	/* File cache pages less the shmem/tmpfs ones, even if we can swap
	 * shmem out (which really depends on the amount of swap) our total
	 * 'used' amount of memory will still remain the same. */
	free = global_page_state(NR_FILE_PAGES) -
			global_page_state(NR_SHMEM);

	/* Add slab reclaimable */
	free += global_page_state(NR_SLAB_RECLAIMABLE);

	/* Now add actual free pages (less the reserved ones) */
	other_free = nr_free_pages();
	if (other_free > totalreserve_pages)
		free += other_free - totalreserve_pages;

	return free;
}
EXPORT_SYMBOL(memnotify_get_free);

/** 
* @brief Used memory measurement (Swap+Mem) (in pages).
* 
* @retval
*/
unsigned long memnotify_get_used(void)
{
	unsigned long used_mem;
	unsigned long used_swap;

	unsigned long free_mem;
	
	free_mem = memnotify_get_free();
	used_swap = total_swap_pages - nr_swap_pages;
	used_mem = totalram_pages - free_mem;

	return used_mem + used_swap;
}

/** 
* @brief Find current threshold and broadcast if necessary.
* 
* @retval
*/
int memnotify_threshold(void)
{
	unsigned long used;
	int used_ratio;
	int threshold;
	int last_threshold;
	int i;

	used = memnotify_get_used();
	used_ratio = used * 100 / totalram_pages;

	threshold = THRESHOLD_NORMAL;
	last_threshold = atomic_read(&memnotify_last_threshold);

	/* Obtain threshold level */
	for(i = memnotify_messages_size - 1; i >= 0; i--) {
		if(used_ratio > memnotify_enter_thresholds[i]) {
			threshold = i;
			break;
		}
	}

	/* Need to leave a threshold by a certain margin. */
	if (threshold < last_threshold) {
		
		int leave_ratio = memnotify_leave_thresholds[last_threshold];

		if (used_ratio > leave_ratio) {
			threshold = last_threshold;
		}
	}

	/* Rate limited notification of threshold changes. */
	memory_pressure_notify(used, used_ratio, threshold);

	return threshold;
}

static int lowmemnotify_open(struct inode *inode, struct file *file)
{
	struct memnotify_file_info *info;
	int err = 0;

	info = kmalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		err = -ENOMEM;
		goto out;
	}

	info->last_memnotify = INITIAL_JIFFIES;
	info->last_threshold = MEMNOTIFY_INVALID; 
	info->file = file;
	file->private_data = info;
	atomic_inc(&nr_watcher_task);
out:
	return err;
}

static int lowmemnotify_release(struct inode *inode, struct file *file)
{
	struct memnotify_file_info *info = file->private_data;

	kfree(info);
	atomic_dec(&nr_watcher_task);
	return 0;
}

static unsigned int lowmemnotify_poll(struct file *file, poll_table *wait)
{
	unsigned int  retval = 0;

	struct memnotify_file_info *info = file->private_data;
	int last_threshold;
	int changed = 0;

	poll_wait(file, &memnotify_wait, wait);

	last_threshold = atomic_read(&memnotify_last_threshold);
	changed = (info->last_threshold != last_threshold);

	/* Notify only in the case that the threshold has changed */
	if (changed) {
		retval = POLLIN;
	}

	return retval;
}

static ssize_t lowmemnotify_read(struct file *file,
		char __user *buf, size_t count, loff_t *ppos) 
{
	struct memnotify_file_info *info = file->private_data;
	unsigned long now = jiffies;
	unsigned long data;
	ssize_t ret = 0;

	if (count < sizeof(unsigned long))
		return -EINVAL;

	info->last_threshold = atomic_read(&memnotify_last_threshold);
	info->last_memnotify = now;
	data = memnotify_messages[info->last_threshold];

	if (0 == ret) {
		ret = put_user(data, (unsigned long __user*)buf);
		if (0 == ret)
			ret = sizeof(unsigned long);
	}
	return ret;
}

/*********************************************************************
 * Print Current Free
 */
static ssize_t
meminfo_show(struct class *class, struct class_attribute *attr, char *buf)
{
	unsigned long used;
	unsigned long total_mem;
	int used_ratio;

	int threshold;
	int last_threshold;

	int len = 0;
	int i;

	used = memnotify_get_used();
	total_mem = totalram_pages;

	threshold = memnotify_threshold();
	last_threshold = atomic_read(&memnotify_last_threshold);

	used_ratio = used * 100 / total_mem;

	len += scnprintf(buf+len, PAGE_SIZE - len,
		"Used (Mem+Swap): %ldMB\n", MB(used));

	len += scnprintf(buf+len, PAGE_SIZE - len,
		"Used (Mem): %ldMB\n", MB(totalram_pages-memnotify_get_free()));

	len += scnprintf(buf+len, PAGE_SIZE - len,
		"Used (Swap): %ldMB\n", MB(total_swap_pages - nr_swap_pages));

	len += scnprintf(buf+len, PAGE_SIZE - len,
		"Used Ratio: %d%%\n", used_ratio);
	len += scnprintf(buf+len, PAGE_SIZE - len,
		"Free: %ldMB\n", MB((long)(total_mem-used)));

	len += scnprintf(buf+len, PAGE_SIZE - len,
		"Current Threshold: %s\n", threshold_string(threshold));
	len += scnprintf(buf+len, PAGE_SIZE - len,
		"Last Threshold: %s\n", threshold_string(last_threshold));

	len += scnprintf(buf+len, PAGE_SIZE - len, "Enter Thresholds:\n");
	for (i = 0; i < memnotify_enter_thresholds_size; i++) {

		unsigned long limit =
				memnotify_enter_thresholds[i]*total_mem/100;

		long left = limit - used;

		len += scnprintf(buf+len, PAGE_SIZE - len, "%s: %d, %ldMB, Rem: %ldMB:\n",
			threshold_string(i),
			memnotify_enter_thresholds[i],
			MB(limit),
			MB(left));
	}
	len += scnprintf(buf+len, PAGE_SIZE - len, "Leave Thresholds:\n");
	for (i = 0; i < memnotify_leave_thresholds_size; i++) {

		unsigned long limit =
				memnotify_leave_thresholds[i]*total_mem/100;

		long left = limit - used;

		len += scnprintf(buf+len, PAGE_SIZE - len, "%s: %d, %ldMB, Rem: %ldMB:\n",
			threshold_string(i),
			memnotify_leave_thresholds[i],
			MB(limit),
			MB(left));
	}

	return len;
}

static CLASS_ATTR(meminfo, S_IRUGO, meminfo_show, NULL);

struct file_operations memnotify_fops = {
	.open = lowmemnotify_open,
	.release = lowmemnotify_release,
	.read = lowmemnotify_read,
	.poll = lowmemnotify_poll,
};

struct device *memnotify_device = NULL;
struct class *memnotify_class = NULL;
int memnotify_major = -1;

static int __init lowmemnotify_init(void)
{
	int err;

	lowmem_print(4, "lowmemorynotify loaded\n");

	memnotify_major = register_chrdev(0, MEMNOTIFY_DEVICE, &memnotify_fops);
	if (memnotify_major < 0) {
		printk("Unable to get major number for memnotify dev\n");
		err = -EBUSY;
		goto error_create_chr_dev;
	}

	memnotify_class = class_create(THIS_MODULE, MEMNOTIFY_DEVICE);
	if (IS_ERR(memnotify_class)) {
		err = PTR_ERR(memnotify_class);
		goto error_class_create;
	}

	memnotify_device = device_create(
			memnotify_class, NULL, MKDEV(memnotify_major, 0),
			NULL, MEMNOTIFY_DEVICE);
	if (IS_ERR(memnotify_device)) {
		err = PTR_ERR(memnotify_device);
		goto error_create_class_dev;
	}

	err = class_create_file(memnotify_class, &class_attr_meminfo);
	if (err) {
		printk(KERN_ERR "%s: couldn't create meminfo.\n", __FUNCTION__);
		goto error_create_class_file;
	}

	return 0;

error_create_class_file:
	device_del(memnotify_device);
error_create_class_dev:
	class_destroy(memnotify_class);
error_class_create:
	unregister_chrdev(memnotify_major, MEMNOTIFY_DEVICE);
error_create_chr_dev:
	return err;
}

static void __exit lowmemnotify_exit(void)
{
	if (memnotify_device)
		device_del(memnotify_device);
	if (memnotify_class)
		class_destroy(memnotify_class);
	if (memnotify_major >= 0)
		unregister_chrdev(memnotify_major, MEMNOTIFY_DEVICE);
}

module_init(lowmemnotify_init);
module_exit(lowmemnotify_exit);

MODULE_LICENSE("GPL");
