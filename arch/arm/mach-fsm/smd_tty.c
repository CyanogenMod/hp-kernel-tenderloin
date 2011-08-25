/* arch/arm/mach-msm/smd_tty.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/wakelock.h>

#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>

#include <mach/msm_smd.h>
#include "smd_private.h"

#define MAX_SMD_TTYS 37

static DEFINE_MUTEX(smd_tty_lock);

struct smd_tty_info {
	smd_channel_t *ch;
	struct tty_struct *tty;
	struct wake_lock wake_lock;
	int open_count;
	struct tasklet_struct tty_tsklt;
};

static struct smd_tty_info smd_tty[MAX_SMD_TTYS];

static void smd_tty_read(unsigned long param)
{
	unsigned char *ptr;
	int avail;
	struct smd_tty_info *info = (struct smd_tty_info *)param;
	struct tty_struct *tty = info->tty;

	if (!tty)
		return;

	for (;;) {
		if (test_bit(TTY_THROTTLED, &tty->flags)) break;
		avail = smd_read_avail(info->ch);
		if (avail == 0)
			break;

		avail = tty_prepare_flip_string(tty, &ptr, avail);

		if (smd_read(info->ch, ptr, avail) != avail) {
			/* shouldn't be possible since we're in interrupt
			** context here and nobody else could 'steal' our
			** characters.
			*/
			printk(KERN_ERR "OOPS - smd_tty_buffer mismatch?!");
		}

		wake_lock_timeout(&info->wake_lock, HZ / 2);
		tty_flip_buffer_push(tty);
	}

	/* XXX only when writable and necessary */
	tty_wakeup(tty);
}

static void smd_tty_notify(void *priv, unsigned event)
{
	struct smd_tty_info *info = priv;

	if (event != SMD_EVENT_DATA)
		return;

	tasklet_hi_schedule(&info->tty_tsklt);
}

static int smd_tty_open(struct tty_struct *tty, struct file *f)
{
	int res = 0;
	int n = tty->index;
	struct smd_tty_info *info;
	const char *name;

	if (n == 0)
		name = "DS";
	else if (n == 7)
		name = "DATA1";
	else if (n == 21)
		name = "DATA21";
	else if (n == 27)
		name = "GPSNMEA";
	else if (n == 36)
		name = "LOOPBACK";
	else
		return -ENODEV;

	info = smd_tty + n;

	mutex_lock(&smd_tty_lock);
	tty->driver_data = info;

	if (info->open_count++ == 0) {
		info->tty = tty;
		tasklet_init(&info->tty_tsklt, smd_tty_read,
			     (unsigned long)info);
		wake_lock_init(&info->wake_lock, WAKE_LOCK_SUSPEND, name);
		if (!info->ch) {
			if (n == 36) {
				/* set smsm state to SMSM_SMD_LOOPBACK state
				** and wait allowing enough time for Modem side
				** to open the loopback port (Currently, this is
				** this is effecient than polling).
				*/
				smsm_change_state(SMSM_APPS_STATE,
						  0, SMSM_SMD_LOOPBACK);
				msleep(100);
			}

			res = smd_open(name, &info->ch, info,
				       smd_tty_notify);
		}
	}
	mutex_unlock(&smd_tty_lock);

	return res;
}

static void smd_tty_close(struct tty_struct *tty, struct file *f)
{
	struct smd_tty_info *info = tty->driver_data;

	if (info == 0)
		return;

	mutex_lock(&smd_tty_lock);
	if (--info->open_count == 0) {
		info->tty = 0;
		tty->driver_data = 0;
		tasklet_kill(&info->tty_tsklt);
		wake_lock_destroy(&info->wake_lock);
		if (info->ch) {
			smd_close(info->ch);
			info->ch = 0;
		}
	}
	mutex_unlock(&smd_tty_lock);
}

static int smd_tty_write(struct tty_struct *tty, const unsigned char *buf, int len)
{
	struct smd_tty_info *info = tty->driver_data;
	int avail;

	/* if we're writing to a packet channel we will
	** never be able to write more data than there
	** is currently space for
	*/
	avail = smd_write_avail(info->ch);
	if (len > avail)
		len = avail;

	return smd_write(info->ch, buf, len);
}

static int smd_tty_write_room(struct tty_struct *tty)
{
	struct smd_tty_info *info = tty->driver_data;
	return smd_write_avail(info->ch);
}

static int smd_tty_chars_in_buffer(struct tty_struct *tty)
{
	struct smd_tty_info *info = tty->driver_data;
	return smd_read_avail(info->ch);
}

static void smd_tty_unthrottle(struct tty_struct *tty)
{
	struct smd_tty_info *info = tty->driver_data;
	tasklet_hi_schedule(&info->tty_tsklt);
	return;
}

static int smd_tty_tiocmget(struct tty_struct *tty, struct file *file)
{
	struct smd_tty_info *info = tty->driver_data;

	return smd_tiocmget(info->ch);
}

static int smd_tty_tiocmset(struct tty_struct *tty, struct file *file,
				unsigned int set, unsigned int clear)
{
	struct smd_tty_info *info = tty->driver_data;

	return smd_tiocmset(info->ch, set, clear);
}

static struct tty_operations smd_tty_ops = {
	.open = smd_tty_open,
	.close = smd_tty_close,
	.write = smd_tty_write,
	.write_room = smd_tty_write_room,
	.chars_in_buffer = smd_tty_chars_in_buffer,
	.unthrottle = smd_tty_unthrottle,
	.tiocmget = smd_tty_tiocmget,
	.tiocmset = smd_tty_tiocmset,
};

static struct tty_driver *smd_tty_driver;

static int __init smd_tty_init(void)
{
	int ret;

	smd_tty_driver = alloc_tty_driver(MAX_SMD_TTYS);
	if (smd_tty_driver == 0)
		return -ENOMEM;

	smd_tty_driver->owner = THIS_MODULE;
	smd_tty_driver->driver_name = "smd_tty_driver";
	smd_tty_driver->name = "smd";
	smd_tty_driver->major = 0;
	smd_tty_driver->minor_start = 0;
	smd_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	smd_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	smd_tty_driver->init_termios = tty_std_termios;
	smd_tty_driver->init_termios.c_iflag = 0;
	smd_tty_driver->init_termios.c_oflag = 0;
	smd_tty_driver->init_termios.c_cflag = B38400 | CS8 | CREAD;
	smd_tty_driver->init_termios.c_lflag = 0;
	smd_tty_driver->flags = TTY_DRIVER_RESET_TERMIOS |
		TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	tty_set_operations(smd_tty_driver, &smd_tty_ops);

	ret = tty_register_driver(smd_tty_driver);
	if (ret) return ret;

	/* this should be dynamic */
	tty_register_device(smd_tty_driver, 0, 0);
	tty_register_device(smd_tty_driver, 7, 0);
	tty_register_device(smd_tty_driver, 21, 0);
	tty_register_device(smd_tty_driver, 27, 0);
	tty_register_device(smd_tty_driver, 36, 0);

	return 0;
}

module_init(smd_tty_init);
