/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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
#include <linux/jiffies.h>
#include <linux/stringify.h>
#include <linux/delay.h>
#include <linux/module.h>

#include <mach/irqs.h>
#include <mach/scm.h>
#include <mach/peripheral-loader.h>
#include <mach/subsystem_restart.h>
#include <mach/subsystem_notif.h>

#include "smd_private.h"
#include "modem_notifier.h"

#define MODEM_HWIO_MSS_RESET_ADDR       0x00902C48
#define SCM_Q6_NMI_CMD                  0x1
#define MODULE_NAME			"subsystem_fatal_8x60"

#define SUBSYS_FATAL_DEBUG

#if defined(SUBSYS_FATAL_DEBUG)
static void subsys_notif_reg_test_notifier(const char *subsys_name);
static void debug_crash_modem_fn(struct work_struct *);
static int reset_modem;

static DECLARE_DELAYED_WORK(debug_crash_modem_work,
				debug_crash_modem_fn);

module_param(reset_modem, int, 0644);
#endif

static void do_soc_restart(void);

/* Subsystem restart: QDSP6 data, functions */
static void q6_fatal_fn(struct work_struct *);
static DECLARE_WORK(q6_fatal_work, q6_fatal_fn);

static void q6_fatal_fn(struct work_struct *work)
{
	pr_err("%s: Watchdog bite received from Q6!\n", MODULE_NAME);
	subsystem_restart("lpass");
}

static void send_q6_nmi(void)
{
	/* Send NMI to QDSP6 via an SCM call. */
	uint32_t cmd = 0x1;
	scm_call(SCM_SVC_UTIL, SCM_Q6_NMI_CMD,
	&cmd, sizeof(cmd), NULL, 0);

	/* Q6 requires atleast 5ms to dump caches etc.*/
	usleep(5000);
}

int subsys_q6_shutdown(void)
{
	send_q6_nmi();
	pil_force_shutdown("q6");
	disable_irq_nosync(LPASS_Q6SS_WDOG_EXPIRED);

	return 0;
}

int subsys_q6_powerup(void)
{
	int ret = pil_force_boot("q6");
	enable_irq(LPASS_Q6SS_WDOG_EXPIRED);
	return ret;
}

void subsys_q6_crash_shutdown(struct subsys_data *subsys)
{
	send_q6_nmi();
}

/* Subsystem restart: Modem data, functions */
static void modem_fatal_fn(struct work_struct *);
static void modem_unlock_timeout(struct work_struct *work);
static int modem_notif_handler(struct notifier_block *this,
				unsigned long code,
				void *_cmd);
static DECLARE_WORK(modem_fatal_work, modem_fatal_fn);
static DECLARE_DELAYED_WORK(modem_unlock_timeout_work,
				modem_unlock_timeout);

static struct notifier_block modem_notif_nb = {
	.notifier_call = modem_notif_handler,
};

static void modem_unlock_timeout(struct work_struct *work)
{
	pr_crit("%s: Timeout waiting for modem to unlock.\n", MODULE_NAME);
	subsystem_restart("modem");
}

static void modem_fatal_fn(struct work_struct *work)
{
	uint32_t modem_state;
	uint32_t panic_smsm_states = SMSM_RESET | SMSM_SYSTEM_DOWNLOAD;
	uint32_t reset_smsm_states = SMSM_SYSTEM_REBOOT_USR |
					SMSM_SYSTEM_PWRDWN_USR;

	pr_err("%s: Watchdog bite received from modem!\n", MODULE_NAME);

	modem_state = smsm_get_state(SMSM_MODEM_STATE);
	pr_err("%s: Modem SMSM state = 0x%x!", MODULE_NAME, modem_state);

	if (modem_state == 0 || modem_state & panic_smsm_states) {

		subsystem_restart("modem");

	} else if (modem_state & reset_smsm_states) {

		pr_err("%s: User-invoked system reset/powerdown.",
			MODULE_NAME);
		do_soc_restart();

	} else {

		int ret;
		void *hwio_modem_reset_addr =
				ioremap_nocache(MODEM_HWIO_MSS_RESET_ADDR, 8);

		pr_err("%s: Modem AHB locked up.\n", MODULE_NAME);
		pr_err("%s: Trying to free up modem!\n", MODULE_NAME);

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

static int modem_notif_handler(struct notifier_block *this,
				unsigned long code,
				void *_cmd)
{
	if (code == MODEM_NOTIFIER_START_RESET) {

		pr_err("%s: Modem error fatal'ed.", MODULE_NAME);
		subsystem_restart("modem");
	}
	return NOTIFY_DONE;
}

static int subsys_modem_shutdown(void)
{

	int smsm_notif_unregistered = 0;

	/* If the modem didn't already crash, setting SMSM_RESET
	 * here will help flush caches etc. Unregister for SMSM
	 * notifications to prevent unnecessary secondary calls to
	 * subsystem_restart.
	 */
	if (!(smsm_get_state(SMSM_MODEM_STATE) & SMSM_RESET)) {
		modem_unregister_notifier(&modem_notif_nb);
		smsm_notif_unregistered = 1;
		smsm_reset_modem(SMSM_RESET);
	}

	/* Wait for 5ms to allow the modem to clean up caches etc. */
	usleep(5000);
	pil_force_shutdown("modem");
	disable_irq_nosync(MARM_WDOG_EXPIRED);

	/* Re-register for SMSM notifications if necessary */
	if (smsm_notif_unregistered)
		modem_register_notifier(&modem_notif_nb);


	return 0;
}

static int subsys_modem_powerup(void)
{
	int ret;

	ret = pil_force_boot("modem");
	enable_irq(MARM_WDOG_EXPIRED);

	return ret;
}

static void subsys_modem_crash_shutdown(struct subsys_data *subsys)
{
	/* If modem hasn't already crashed, send SMSM_RESET. */
	if (!(smsm_get_state(SMSM_MODEM_STATE) & SMSM_RESET)) {
		modem_unregister_notifier(&modem_notif_nb);
		smsm_reset_modem(SMSM_RESET);
	}

	/* Wait for 5ms to allow the modem to clean up caches etc. */
	usleep(5000);
}

/* Non-subsystem-specific functions */
static void do_soc_restart(void)
{
	pr_err("%s: Rebooting SoC..\n", MODULE_NAME);
	lock_kernel();
	kernel_restart(NULL);
	unlock_kernel();
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
		pr_err("%s: %s: Unknown IRQ!\n", MODULE_NAME, __func__);
	}

	return IRQ_HANDLED;
}

static struct subsys_data subsys_8x60_q6 = {
	.name = "lpass",
	.shutdown = subsys_q6_shutdown,
	.powerup = subsys_q6_powerup,
	.crash_shutdown = subsys_q6_crash_shutdown
};

static struct subsys_data subsys_8x60_modem = {
	.name = "modem",
	.shutdown = subsys_modem_shutdown,
	.powerup = subsys_modem_powerup,
	.crash_shutdown = subsys_modem_crash_shutdown
};

static int __init subsystem_restart_8x60_init(void)
{
	ssr_register_subsystem(&subsys_8x60_modem);
	ssr_register_subsystem(&subsys_8x60_q6);

#ifdef SUBSYS_FATAL_DEBUG
	subsys_notif_reg_test_notifier("modem");
	subsys_notif_reg_test_notifier("lpass");
#endif

	return 0;
}

static int __init subsystem_fatal_init(void)
{
	int ret;

	/* Need to listen for SMSM_RESET always */
	modem_register_notifier(&modem_notif_nb);

#if defined(SUBSYS_FATAL_DEBUG)
	schedule_delayed_work(&debug_crash_modem_work, msecs_to_jiffies(5000));
#endif

	ret = request_irq(MARM_WDOG_EXPIRED, subsys_wdog_bite_irq,
			IRQF_TRIGGER_RISING, "modem_wdog", NULL);

	if (ret < 0) {
		pr_err("%s: Unable to request MARM_WDOG_EXPIRED irq.",
			__func__);
		goto out;
	}

	ret = request_irq(LPASS_Q6SS_WDOG_EXPIRED, subsys_wdog_bite_irq,
			IRQF_TRIGGER_RISING, "q6_wdog", NULL);

	if (ret < 0) {
		pr_err("%s: Unable to request LPASS_Q6SS_WDOG_EXPIRED irq.",
			__func__);
		goto out;
	}

	ret = subsystem_restart_8x60_init();

out:
	return ret;
}

static void __exit subsystem_fatal_exit(void)
{
	free_irq(MARM_WDOG_EXPIRED, NULL);
	free_irq(LPASS_Q6SS_WDOG_EXPIRED, NULL);
}

#ifdef SUBSYS_FATAL_DEBUG
static const char *notif_to_string(enum subsys_notif_type notif_type)
{
	switch (notif_type) {

	case	SUBSYS_BEFORE_SHUTDOWN:
		return __stringify(SUBSYS_BEFORE_SHUTDOWN);

	case	SUBSYS_AFTER_SHUTDOWN:
		return __stringify(SUBSYS_AFTER_SHUTDOWN);

	case	SUBSYS_BEFORE_POWERUP:
		return __stringify(SUBSYS_BEFORE_POWERUP);

	case	SUBSYS_AFTER_POWERUP:
		return __stringify(SUBSYS_AFTER_POWERUP);

	default:
		return "unknown";
	}
}

static int subsys_notifier_test_call(struct notifier_block *this,
				  unsigned long code,
				  void *data)
{
	switch (code) {

	default:
		pr_warn("%s: Notification %s from subsystem %p\n",
			__func__, notif_to_string(code), data);
	break;

	}

	return NOTIFY_DONE;
}

static struct notifier_block nb = {
	.notifier_call = subsys_notifier_test_call,
};

static void subsys_notif_reg_test_notifier(const char *subsys_name)
{
	void *handle = subsys_notif_register_notifier(subsys_name, &nb);
	pr_warn("%s: Registered test notifier, handle=%p",
			__func__, handle);
}

static void debug_crash_modem_fn(struct work_struct *work)
{
	if (reset_modem == 1)
		smsm_reset_modem(SMSM_RESET);
	else if (reset_modem == 2)
		subsystem_restart("lpass");

	reset_modem = 0;
	schedule_delayed_work(&debug_crash_modem_work, msecs_to_jiffies(1000));
}
#endif

module_init(subsystem_fatal_init);
module_exit(subsystem_fatal_exit);
