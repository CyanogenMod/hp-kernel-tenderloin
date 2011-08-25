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
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/reboot.h>
#include <linux/workqueue.h>

#include <mach/irqs.h>

static void modem_fatal_fn(struct work_struct *work)
{
	printk(KERN_CRIT "Watchdog bite received from modem! Rebooting.\n");

	lock_kernel();
	kernel_restart(NULL);
	unlock_kernel();
}

static DECLARE_WORK(modem_fatal_work, modem_fatal_fn);

static irqreturn_t modem_wdog_bite_irq(int irq, void *dev_id)
{
	int ret;

	ret = schedule_work(&modem_fatal_work);
	disable_irq_nosync(MARM_WDOG_EXPIRED);

	return IRQ_HANDLED;
}

static void __exit subsystem_fatal_exit(void)
{
	free_irq(MARM_WDOG_EXPIRED, NULL);
}

static int __init subsystem_fatal_init(void)
{
	int ret;

	ret = request_irq(MARM_WDOG_EXPIRED, modem_wdog_bite_irq,
			  IRQF_TRIGGER_RISING, "modem_wdog", NULL);

	return ret;
}

module_init(subsystem_fatal_init);
module_exit(subsystem_fatal_exit);
