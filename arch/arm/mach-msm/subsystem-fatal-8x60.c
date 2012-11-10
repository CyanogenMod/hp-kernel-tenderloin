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
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/jiffies.h>

#include <mach/irqs.h>

#include "smd_private.h"
#include "modem_notifier.h"
#include <mach/scm.h>

#define MODEM_HWIO_MSS_RESET_ADDR       0x00902C48
#define SCM_Q6_NMI_CMD			0x1

static void q6_fatal_fn(struct work_struct *);
static void modem_fatal_fn(struct work_struct *);
static void modem_unlock_timeout(struct work_struct *work);
static int modem_notif_handler(struct notifier_block *this,
				unsigned long code,
				void *_cmd);

static DECLARE_WORK(modem_fatal_work, modem_fatal_fn);
static DECLARE_WORK(q6_fatal_work, q6_fatal_fn);
static DECLARE_DELAYED_WORK(modem_unlock_timeout_work,
				modem_unlock_timeout);

static struct notifier_block modem_notif_nb = {
	.notifier_call = modem_notif_handler,
};

static void do_soc_restart(void)
{
	printk(KERN_CRIT "subsys-restart: Rebooting SoC..\n");
	lock_kernel();
	kernel_restart(NULL);
	unlock_kernel();
}

static void send_q6_nmi(void)
{
	/* Send NMI to QDSP6 via an SCM call. */
	uint32_t cmd = 0x1;
	scm_call(SCM_SVC_UTIL, SCM_Q6_NMI_CMD,
		&cmd, sizeof(cmd), NULL, 0);

	/* Q6 requires atleast 5ms to dump caches etc.
	 * Check panic timeout value and usleep if necessary.
	*/
	if (panic_timeout < 1)
		usleep(5000);
}

static void modem_unlock_timeout(struct work_struct *work)
{
	send_q6_nmi();
	panic("subsys-restart: Timeout waiting for modem to unwedge.\n");
}

static void modem_fatal_fn(struct work_struct *work)
{
	uint32_t modem_state;
	uint32_t panic_smsm_states = SMSM_RESET | SMSM_SYSTEM_DOWNLOAD;
	uint32_t reset_smsm_states = SMSM_SYSTEM_REBOOT_USR |
					SMSM_SYSTEM_PWRDWN_USR;

	printk(KERN_CRIT "Watchdog bite received from modem!\n");

	modem_state = smsm_get_state(SMSM_MODEM_STATE);

	if (modem_state == 0 || modem_state & panic_smsm_states) {

		send_q6_nmi();
		panic("subsys-restart: Modem SMSM state = 0x%x!", modem_state);

	} else if (modem_state & reset_smsm_states) {

		printk(KERN_CRIT
			"subsys-restart: User invoked system reset/powerdown. \
			Modem SMSM state = 0x%x", modem_state);
		do_soc_restart();

	} else {

		int ret;
		void *hwio_modem_reset_addr =
				ioremap_nocache(MODEM_HWIO_MSS_RESET_ADDR, 8);

		printk(KERN_CRIT "subsys-restart: Modem AHB locked up.\n");
		printk(KERN_CRIT "subsys-restart: Trying to free up modem!\n");

		writel(0x3, hwio_modem_reset_addr);

		/* If we are still alive after 6 seconds (allowing for
		 * the 5-second-delayed-panic-reboot), modem is either
		 * still wedged or SMSM didn't come through. Force panic
		 * in that case.
		*/
		ret = schedule_delayed_work(&modem_unlock_timeout_work,
					msecs_to_jiffies(6000));

		iounmap(hwio_modem_reset_addr);
	}
}

static void q6_fatal_fn(struct work_struct *work)
{
	send_q6_nmi();

	/* Send modem an SMSM_RESET message to allow flushing of caches etc.
	 * Unregister for SMSM notifications to prevent second panic.
	 */
	modem_unregister_notifier(&modem_notif_nb);
	smsm_reset_modem(SMSM_RESET);

	panic("Watchdog bite received from Q6! Rebooting.\n");
}

static irqreturn_t subsys_wdog_bite_irq(int irq, void *dev_id)
{
	int ret;

	switch (irq) {

	case MARM_WDOG_EXPIRED:
		ret = schedule_work(&modem_fatal_work);
		disable_irq_nosync(MARM_WDOG_EXPIRED);
	break;

	case LPASS_Q6SS_WDOG_EXPIRED:
		ret = schedule_work(&q6_fatal_work);
		disable_irq_nosync(LPASS_Q6SS_WDOG_EXPIRED);
	break;

	default:
		printk(KERN_CRIT "subsys-fatal: %s: Unknown IRQ!\n", __func__);

	}

	return IRQ_HANDLED;
}

static void __exit subsystem_fatal_exit(void)
{
	free_irq(MARM_WDOG_EXPIRED, NULL);
	free_irq(LPASS_Q6SS_WDOG_EXPIRED, NULL);
}

static int __init subsystem_fatal_init(void)
{
	int ret;

	/* Need to listen for SMSM_RESET always */
	modem_register_notifier(&modem_notif_nb);

	ret = request_irq(MARM_WDOG_EXPIRED, subsys_wdog_bite_irq,
			IRQF_TRIGGER_RISING, "modem_wdog", NULL);

	ret = request_irq(LPASS_Q6SS_WDOG_EXPIRED, subsys_wdog_bite_irq,
			IRQF_TRIGGER_RISING, "q6_wdog", NULL);

	return ret;
}

static int modem_notif_handler(struct notifier_block *this,
				unsigned long code,
				void *_cmd)
{
	if (code == MODEM_NOTIFIER_START_RESET) {
		send_q6_nmi();
		panic("subsys-restart: Modem error fatal'ed.");
	}
	return NOTIFY_DONE;
}

module_init(subsystem_fatal_init);
module_exit(subsystem_fatal_exit);
