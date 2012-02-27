/*
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 * Copyright (C) 2012 James Sullins
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/miscdevice.h>
#include <linux/string.h>
#include <linux/spinlock.h>
#include <linux/klog.h>
#include <asm/memory.h>
#include <asm/io.h>

#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/io.h>

#define MIN(a,b) ((a)<(b) ? (a):(b))

#define KLOG_MAGIC 0x6b6c6f67 // 'klog'
#define KLOG_VERSION 1

extern int log_buf_get_len(void);

static const char *last_klog_names[] = {
	"last_klog",
	"last_klog2",
	"last_klog3",
	NULL
};

struct klog_header {
	uint32_t magic;
	uint32_t ver;
	uint32_t len;

	uint32_t buf_count;
	uint32_t current_buf;

	uint32_t buf_table[0]; // offsets from start of this header to klog buffers
};

#define KLOG_BUFFER_MAGIC 0x6b627566 // 'kbuf'

struct klog_buffer_header {
	uint32_t magic;
	uint32_t len;
	uint32_t head;
	uint32_t tail;

	uint8_t data[0];
};

static unsigned long klog_phys;
static unsigned long klog_len;

static int init_done = 0;

static char *klog_buffer;

static struct klog_header *klog;
static struct klog_buffer_header *klog_buf;

static void klog_copy_logbuf(void);
int log_buf_copy(char *dest, int idx, int len);

static DEFINE_SPINLOCK(klog_lock);

static inline struct klog_buffer_header *get_kbuf(int num)
{
	return (struct klog_buffer_header *)((uint8_t *)klog + klog->buf_table[num]);
}

static inline struct klog_buffer_header *get_last_kbuf(int num)
{
	int lastnum;

	if (num < 1 || num >= klog->buf_count) return NULL;

	lastnum = klog->current_buf - num;
	if (lastnum <0) lastnum += klog->buf_count;

	return (struct klog_buffer_header *)
		((uint8_t *)klog + klog->buf_table[lastnum]);
}

static inline unsigned get_last_klog_size(int num)
{
	struct klog_buffer_header *lkbuf = get_last_kbuf(num);

	if (!lkbuf) return 0;

	if (lkbuf->magic != KLOG_BUFFER_MAGIC) {
		return 0;
	}
	else if (lkbuf->tail == lkbuf->head) {
		return 0;
	}
	else if (lkbuf->head > lkbuf->tail) {
		return lkbuf->head - lkbuf->tail;
	}
	else {
		return lkbuf->len - 1;
	}
}

static inline uint32_t
inc_pointer(struct klog_buffer_header *klog, uint32_t pointer, uint32_t inc)
{
	pointer += inc;
	if (pointer >= klog->len)
		pointer -= klog->len;

	return pointer;
}

static void _klog_write(const char *s, unsigned int count)
{
	unsigned int towrite;

	if (klog_buf == NULL)
		return;

	/* trim the write if it happens to be huge */
	if (count > klog_buf->len - 1)
		count = klog_buf->len - 1;

	while (count > 0) {
		/* write up to the end of the buffer */
		towrite = MIN(count, klog_buf->len - klog_buf->head);

		/* does this need to increment the tail? */
		{
			uint32_t vtail = klog_buf->tail;
			if (klog_buf->tail <= klog_buf->head)
				vtail += klog_buf->len;

			if (klog_buf->head + towrite >= vtail)
				klog_buf->tail = inc_pointer(klog_buf, klog_buf->head, towrite + 1);
		}

		/* copy */
		memcpy(klog_buf->data + klog_buf->head, s, towrite);
		klog_buf->head = inc_pointer(klog_buf, klog_buf->head, towrite);
		count -= towrite;
		s += towrite;
	}
}

static void klog_copy_logbuf()
{
	unsigned int count;
	unsigned int towrite;

	if (klog_buf == NULL)
		return;

	count = log_buf_get_len();

	while (count > 0) {
		/* write up to the end of the buffer */
		towrite = MIN(count, klog_buf->len - klog_buf->head);

		/* does this need to increment the tail? */
		{
			uint32_t vtail = klog_buf->tail;
			if (klog_buf->tail <= klog_buf->head)
				vtail += klog_buf->len;

			if (klog_buf->head + towrite >= vtail)
				klog_buf->tail = inc_pointer(klog_buf, klog_buf->head, towrite + 1);
		}

		/* copy */
		log_buf_copy(klog_buf->data + klog_buf->head, 0, towrite);
		klog_buf->head = inc_pointer(klog_buf, klog_buf->head, towrite);
		count -= towrite;
	}
}


void klog_printf(const char *fmt, ...)
{
	static char klog_print_buf[1024];

	unsigned int flags;
	unsigned int len;
	va_list args;

	spin_lock_irqsave(&klog_lock, flags);

	va_start(args, fmt);
	len = vscnprintf(klog_print_buf, sizeof(klog_print_buf), fmt, args);
	va_end(args);

	_klog_write(klog_print_buf, len);

	spin_unlock_irqrestore(&klog_lock, flags);
}

void klog_write(const char *s, unsigned int count)
{
	unsigned int flags;

	spin_lock_irqsave(&klog_lock, flags);

	_klog_write(s, count);

	spin_unlock_irqrestore(&klog_lock, flags);
}

static ssize_t last_klog_read(struct file *file, char __user *buf,
				    size_t len, loff_t *offset)
{
	loff_t pos = *offset;
	ssize_t count, lcount, ocount;
	unsigned lastnum = 0;
	unsigned lastsize;
	const char* req_name = NULL;
	struct klog_buffer_header *lkbuf;
	char *lbuf;
	unsigned i;

	if (file && file->f_path.dentry) {
		req_name = file->f_path.dentry->d_name.name;
	}

	if (!req_name) return -EFAULT;

	for (i = 0; (i < klog->buf_count - 1 && last_klog_names[i]); i++) {
		if (!strcmp(req_name, last_klog_names[i])) {
			lastnum = i + 1;
			break;
		}
	}
	if (lastnum == 0) return -EFAULT;

	lastsize = get_last_klog_size(lastnum);
	if (pos >= lastsize) return 0;

	lkbuf = get_last_kbuf(lastnum);
	lbuf = (char*)lkbuf->data;

	if (lkbuf->head > lkbuf->tail) {
		count = min(len, (size_t)(lastsize - pos));
		if (copy_to_user(buf, lbuf + lkbuf->tail + pos, count))
			return -EFAULT;

		*offset += count;
		return count;
	}

	count = min(len, (size_t)(lastsize - pos));
	ocount = count;

	while (count) {
		if (pos >= 0 && pos < lkbuf->len - lkbuf->tail) {
			lcount = min((uint32_t)count, (uint32_t)(lkbuf->len - lkbuf->tail)
							- (uint32_t)pos);
			if (copy_to_user(buf, lbuf + lkbuf->tail + pos, lcount))
				return -EFAULT;
		} else {
			lcount = min((uint32_t)count,
							lastsize - (lkbuf->len - lkbuf->tail));
			if (copy_to_user(buf, lbuf + pos - (lkbuf->len - lkbuf->tail),
								lcount))
				return -EFAULT;
		}
		buf += lcount;
		pos += lcount;
		count -= lcount;
	}

	*offset += ocount;
	return ocount;
}

static const struct file_operations last_klog_file_ops = {
	.owner = THIS_MODULE,
	.read = last_klog_read,
};

void setup_proc_last_klog(void)
{
	struct proc_dir_entry *entry;
	unsigned i;

	for (i = 0; (i < klog->buf_count - 1 && last_klog_names[i]); i++) {
		entry = create_proc_entry(last_klog_names[i]
									, S_IFREG | S_IRUGO, NULL);
		if (!entry) {
			printk(KERN_ERR "klog_init: failed to create proc entry '%s'\n",
					last_klog_names[i]);
		} else {
			entry->proc_fops = &last_klog_file_ops;
			entry->size = get_last_klog_size(i+1);
			printk(KERN_INFO "klog_init: %s head=%u tail=%u size=%u\n",
					last_klog_names[i], get_last_kbuf(i+1)->head,
					get_last_kbuf(i+1)->tail, get_last_klog_size(i+1));
		}
	}
}

static int __init klog_init(void)
{
	void *base;
	unsigned long flags;

	printk(KERN_INFO "klog_init: phys buffer at 0x%lx\n", klog_phys);

	if (klog_phys == 0 || klog_len == 0)
	    return 0;

	if (!request_mem_region(klog_phys, klog_len, "klog"))
	    return 0;

	base = ioremap(klog_phys, klog_len);
	if (base == 0)
	    return 0;

	/* set up the klog structure */
	klog_buffer = (char *)base;
	klog = (struct klog_header *)klog_buffer;
	printk(KERN_INFO "klog_init: virt address at 0x%p\n", klog);

	/* check to see if it's valid */
	if (klog->magic != KLOG_MAGIC || klog->ver != KLOG_VERSION) {
	    printk(KERN_ERR "klog_init: valid klog not found\n");
	    return 0;
	}

	printk(KERN_INFO "klog_init: found valid klog, len %u\n", klog->len);

	klog_buf = get_kbuf(klog->current_buf);

	spin_lock_irqsave(&klog_lock, flags);

	klog_copy_logbuf();

	init_done = 1;

	spin_unlock_irqrestore(&klog_lock, flags);

	printk(KERN_INFO "klog_init: using buffer %u at 0x%p, length %d\n",
			klog->current_buf, klog_buf, klog_buf->len);

	setup_proc_last_klog();

	return 0;
}

subsys_initcall(klog_init);

static int __init klog_setup(char *this_opt)
{
	klog_phys = simple_strtoul(this_opt, NULL, 0);

	return 1;
}

__setup("klog=", klog_setup);

static int __init klog_len_setup(char *this_opt)
{
	klog_len = simple_strtoul(this_opt, NULL, 0);

	return 1;
}

__setup("klog_len=", klog_len_setup);


