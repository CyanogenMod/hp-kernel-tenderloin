/*
 *
 * Copyright (C) 2008-2009 Palm, Inc.
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

#define MIN(a,b) ((a)<(b) ? (a):(b))

#define KLOG_MAGIC 0x6b6c6f67 // 'klog'
#define KLOG_VERSION 1

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

static char *klog_buffer;

static struct klog_header *klog;
static struct klog_buffer_header *klog_buf;

static DEFINE_SPINLOCK(klog_lock);

static inline struct klog_buffer_header *get_kbuf(int num)
{
	return (struct klog_buffer_header *)((uint8_t *)klog + klog->buf_table[num]);
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

static int __init klog_init(void)
{
	void *base;

	printk("klog_init: entry\n");
	printk("klog_init: phys buffer is at 0x%lx\n", klog_phys);

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
	printk("klog_init: virt address is %p\n", klog);

	printk("klog_init: magic 0x%x version 0x%x\n", klog->magic, klog->ver);

	/* check to see if it's valid */
	if (klog->magic != KLOG_MAGIC || klog->ver != KLOG_VERSION) {
	    printk("klog_init: didn't find existing klog\n");
	    return 0;
	}

	printk("found klog, len %u, using buffer number %d\n", klog->len, klog->current_buf);

	klog_buf = get_kbuf(klog->current_buf);

	klog_printf("welcome to klog, buffer at %p, length %d\n", klog_buf, klog_buf->len);

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


