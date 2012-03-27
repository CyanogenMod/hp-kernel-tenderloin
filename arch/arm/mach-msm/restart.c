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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/reboot.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/pmic8901.h>

#include <mach/msm_iomap.h>
#include <mach/scm-io.h>
#include <mach/restart.h>

#define CONFIG_WALL_MSG

#define TCSR_WDT_CFG 0x30

#define WDT0_RST       (MSM_TMR0_BASE + 0x38)
#define WDT0_EN        (MSM_TMR0_BASE + 0x40)
#define WDT0_BARK_TIME (MSM_TMR0_BASE + 0x4C)
#define WDT0_BITE_TIME (MSM_TMR0_BASE + 0x5C)

#define PSHOLD_CTL_SU (MSM_TLMM_BASE + 0x820)

#define IMEM_BASE           0x2A05F000

#define RESTART_REASON_ADDR 0x65C
#define DLOAD_MODE_ADDR     0x0

static void *imem;
static int restart_mode;
#ifdef CONFIG_WALL_MSG
static void *restart_reason_addr;
#endif

/******************************************************************************
 * Restart Definitions
 *****************************************************************************/

#define RESTART_REASON_SHUTDOWN   0x6f656d00
#define RESTART_REASON_RECOVER    0x6f656d11
#define RESTART_REASON_PANIC      0x6f656d22
#define RESTART_REASON_DFU        0x6f656d33
#define RESTART_REASON_REBOOT     0x6f656d44
#define RESTART_REASON_LATE_BOOT  0x6f656d55
#define RESTART_REASON_UPDATE     0x6f656d66
#define RESTART_REASON_UNKNOWN    0x77665501

#ifdef CONFIG_MSM_DLOAD_MODE
static int in_panic;
static int reset_detection;
static void *dload_mode_addr;

static int panic_prep_restart(struct notifier_block *this,
			      unsigned long event, void *ptr)
{
	in_panic = 1;
	return NOTIFY_DONE;
}

static struct notifier_block panic_blk = {
	.notifier_call	= panic_prep_restart,
};

static void set_dload_mode(int on)
{
	if (dload_mode_addr) {
		writel(on ? 0xE47B337D : 0, dload_mode_addr);
		writel(on ? 0xCE14091A : 0,
		       dload_mode_addr + sizeof(unsigned int));
		dmb();
	}
}

static int get_dload_mode(void)
{
	int on = 0;
	if (dload_mode_addr) {
		if (readl(dload_mode_addr) == 0xE47B337D)
			on = 1;
	}
	return on;
}

static int reset_detect_set(const char *val, struct kernel_param *kp)
{
	int ret;
	int old_val = reset_detection;

	ret = param_set_int(val, kp);

	if (ret)
		return ret;

	switch (reset_detection) {

	case 0:
		/*
		*  Deactivate reset detection. Unset the download mode flag only
		*  if someone hasn't already set restart_mode to something other
		*  than RESTART_NORMAL.
		*/
		if (restart_mode == RESTART_NORMAL)
			set_dload_mode(0);
	break;

	case 1:
		set_dload_mode(1);
	break;

	default:
		reset_detection = old_val;
		return -EINVAL;
	break;

	}

	return 0;
}

module_param_call(reset_detection, reset_detect_set, param_get_int,
			&reset_detection, 0644);
#else
#define set_dload_mode(x) do {} while (0)
#endif

void arch_reset(char mode, const char *cmd);

void msm_set_restart_mode(int mode)
{
	restart_mode = mode;
}
EXPORT_SYMBOL(msm_set_restart_mode);

static void msm_power_off(void)
{
#ifndef CONFIG_RESTART_USES_PALM_BOOTLOADER
	printk(KERN_NOTICE "Powering off the SoC\n");
	pm8058_reset_pwr_off(0);
	pm8901_reset_pwr_off(0);
	writel(0, PSHOLD_CTL_SU);
	mdelay(10000);
	printk(KERN_ERR "Powering off has failed\n");
#else
	/* use HPalm bootloader to shutdown/poweroff */
	printk(KERN_INFO "%s: Initiating poweroff via bootloader\n", __func__);
	arch_reset(0, "shutdown");
#endif
	return;
}

/*
 * write some u32 message at specified offset
 */
void msm_imem_put_dword(u32 offset, u32 msg)
{
	/* IMEM is only 4k */
	if (offset > (PAGE_SIZE - sizeof(msg)))
		return;

	writel(msg, imem + offset);
	dmb();
}
EXPORT_SYMBOL(msm_imem_put_dword);

/* write msg */
static void msm_put_msg(u32 msg)
{
	if (restart_reason_addr) {
		writel(msg, restart_reason_addr);
		dmb();
	}
}

/* read msg */
static u32 msm_get_msg(void)
{
	u32 val = 0;

	if (restart_reason_addr) {
		val = readl(restart_reason_addr);
	}

	return val;
}

void arch_reset(char mode, const char *cmd)
{
	uint32_t reason = RESTART_REASON_UNKNOWN;

#ifdef CONFIG_MSM_DLOAD_MODE
#ifndef CONFIG_WALL_MSG
	if (in_panic || restart_mode == RESTART_DLOAD)
		set_dload_mode(1);

	/*
	*  If we're not currently panic-ing, and if reset detection is active,
	*  unset the download mode flag. However, do this only if the current
	*  restart mode is RESTART_NORMAL.
	*/
	if (reset_detection && !in_panic && restart_mode == RESTART_NORMAL)
		set_dload_mode(0);
#else
	if (RESTART_DLOAD == restart_mode) {
		set_dload_mode(1);
	}
	else {
		set_dload_mode(0);
	}
#endif
#endif
	printk(KERN_NOTICE "Going down for restart now\n");

	pm8058_reset_pwr_off(1);

	if (cmd != NULL && *cmd != NULL) {
		if (RESTART_DLOAD != restart_mode) {
			if (!strncmp(cmd, "bootloader", 10)) {
				reason = 0x77665500;
			} else if (!strncmp(cmd, "recovery", 8)) {
				reason = 0x77665502;
			} else if (!strncmp(cmd, "oem-", 4)) {
				unsigned long code;
				strict_strtoul(cmd + 4, 16, &code);
				code = code & 0xff;
				reason = 0x6f656d00 | code;
			// Palm specific
			} else if (!strcmp(cmd, "shutdown")) {
				reason = RESTART_REASON_SHUTDOWN;
			} else if (!strcmp(cmd, "recover")) {
				reason = RESTART_REASON_RECOVER;
			} else if (!strcmp(cmd, "panic")) {
				reason = RESTART_REASON_PANIC;
			} else if (!strcmp(cmd, "dfu")) {
				reason = RESTART_REASON_DFU;
			} else if (!strcmp(cmd, "reboot")) {
				reason = RESTART_REASON_REBOOT;
			} else if (!strcmp(cmd, "update")) {
				reason = RESTART_REASON_UPDATE;
			} else {
				reason = RESTART_REASON_UNKNOWN;
			}
			msm_put_msg(reason);
		}
	} else {
#ifdef CONFIG_RESTART_USES_PALM_BOOTLOADER
		/* use HPalm bootloader to reboot */
		if (RESTART_DLOAD != restart_mode) {
			printk(KERN_INFO "%s: Initiating reboot via bootloader\n", __func__);
			msm_put_msg(RESTART_REASON_REBOOT);
		}
#endif
	}

	writel(0, WDT0_EN);
	writel(0, PSHOLD_CTL_SU); /* Actually reset the chip */
	mdelay(5000);

	printk(KERN_NOTICE "PS_HOLD didn't work, falling back to watchdog\n");

	writel(0x31F3, WDT0_BARK_TIME);
	writel(0x31F3, WDT0_BITE_TIME);
	writel(3, WDT0_EN);
	dmb();
	secure_writel(3, MSM_TCSR_BASE + TCSR_WDT_CFG);

	mdelay(10000);
	printk(KERN_ERR "Restarting has failed\n");
}

#ifdef CONFIG_WALL_MSG
#include <linux/slab.h>
#include <linux/input.h>
static int wall_msg = 0;

static int panic_wall_msg(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	if (wall_msg)
		return NOTIFY_DONE;
	wall_msg = 1;
	if (RESTART_DLOAD != restart_mode) {  
		msm_put_msg(RESTART_REASON_PANIC) ;
	}
	return NOTIFY_DONE;
}

static struct notifier_block panic_wall_blk = {
	.notifier_call	= panic_wall_msg,
};

static int reboot_wall_msg(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	if (wall_msg)
		return NOTIFY_DONE;
	wall_msg = 1;
	msm_put_msg(RESTART_REASON_REBOOT);
	return NOTIFY_DONE;
}

static struct notifier_block reboot_wall_blk = {
	.notifier_call	= reboot_wall_msg,
};

static bool reboot_state = false;
static int power_state = 0;
static int home_state = 0;
static u32 stored_reason = 0;
static int stored_dload_mode = 0;

static void reboot_state_callback(struct work_struct *unused)
{
	if (power_state && home_state) {
		printk(KERN_WARNING "restart: possible user-initiated reset, clear the wall\n");
		stored_reason = msm_get_msg();
		stored_dload_mode = get_dload_mode();
		msm_put_msg(RESTART_REASON_REBOOT);
		set_dload_mode(0);
		reboot_state = true;
	}
	else if (true == reboot_state) {
		printk(KERN_WARNING "restart: false alarm, restore original wall values\n");
		set_dload_mode(stored_dload_mode);
		msm_put_msg(stored_reason);
		reboot_state = false;
	}
}

static DECLARE_WORK(reboot_state_work, reboot_state_callback);
static void reboot_input_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{

	bool trigger_change = false;

	if (EV_KEY == type) {
		if (KEY_END == code) {
			power_state = value;
			trigger_change = true;
		}
		if (KEY_CENTER == code) {
			home_state = value;
			trigger_change = true;
		}

		if (trigger_change)
			schedule_work_on(0, &reboot_state_work);

	}
}

static int reboot_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "reboot";

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

static void reboot_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id reboot_ids[] = {
	{ .driver_info = 1 },
	{ },
};

static struct input_handler reboot_input_handler = {
	.event		= reboot_input_event,
	.connect	= reboot_input_connect,
	.disconnect	= reboot_input_disconnect,
	.name		= "reboot_catcher",
	.id_table	= reboot_ids,
};

#endif


static int __init msm_restart_init(void)
{
	imem = ioremap_nocache(IMEM_BASE, SZ_4K);

#ifdef CONFIG_MSM_DLOAD_MODE
	atomic_notifier_chain_register(&panic_notifier_list, &panic_blk);
	dload_mode_addr = imem + DLOAD_MODE_ADDR;
#endif

#ifdef CONFIG_WALL_MSG
	atomic_notifier_chain_register(&panic_notifier_list, &panic_wall_blk);
	register_reboot_notifier(&reboot_wall_blk);

	input_register_handler(&reboot_input_handler);
	restart_reason_addr = imem + RESTART_REASON_ADDR;
#endif
	pm_power_off = msm_power_off;

	return 0;
}

late_initcall(msm_restart_init);
