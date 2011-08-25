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
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <asm/mach-types.h>
#include <asm/uaccess.h>
#include <mach/mdm.h>
#include "mdm_ioctls.h"

#define CHARM_MODEM_TIMEOUT	2000

static void (*power_on_charm)(void);
static void (*power_down_charm)(void);

static void charm_dummy_reset(void){
	return;
}

void (*charm_intentional_reset)(void) = charm_dummy_reset;


static void __soc_restart(void)
{
	lock_kernel();
	kernel_restart(NULL);
	unlock_kernel();
}

static void charm_wait_for_mdm(void)
{
	msleep(CHARM_MODEM_TIMEOUT);
	if (gpio_get_value(MDM2AP_STATUS) != 0) {
		pr_err("%s: MDM2AP_STATUS never went low.\n",
			__func__);
		gpio_request(AP2MDM_PMIC_RESET_N, "AP2MDM_PMIC_RESET_N");
		gpio_direction_output(AP2MDM_PMIC_RESET_N, 1);
	}

}

static int charm_panic_prep(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	pr_err("%s: setting AP2MDM_ERRFATAL high\n", __func__);
	gpio_set_value(AP2MDM_ERRFATAL, 1);
	return NOTIFY_DONE;
}

void __charm_intentional_reset(void)
{
	pr_err("%s: setting AP2MDM_STATUS low\n", __func__);
	gpio_set_value(AP2MDM_STATUS, 0);
	charm_wait_for_mdm();
}

static struct notifier_block charm_panic_blk = {
	.notifier_call  = charm_panic_prep,
};


static long charm_modem_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{

	int ret = 0;

	if (_IOC_TYPE(cmd) != CHARM_CODE) {
		pr_err("%s: invalid ioctl code\n", __func__);
		return -EINVAL;
	}

	gpio_request(MDM2AP_STATUS, "MDM2AP_STATUS");
	gpio_direction_input(MDM2AP_STATUS);

	switch (cmd) {
	case WAKE_CHARM:
		/* turn the charm on */
		power_on_charm();
		break;

	case RESET_CHARM:
		/* put the charm back in reset */
		power_down_charm();
		break;

	case CHECK_FOR_BOOT:
		if (gpio_get_value(MDM2AP_STATUS) == 0)
			put_user(1, (unsigned long __user *) arg);
		else
			put_user(0, (unsigned long __user *) arg);
		break;

	case WAIT_FOR_BOOT:
		/* wait for status to be high */
		while (gpio_get_value(MDM2AP_STATUS) == 0)
			;
		break;

	default:
		ret = -EINVAL;
	}

	gpio_free(MDM2AP_STATUS);

	return ret;
}

static int charm_modem_open(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations charm_modem_fops = {
	.owner		= THIS_MODULE,
	.open		= charm_modem_open,
	.unlocked_ioctl	= charm_modem_ioctl,
};


struct miscdevice charm_modem_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "mdm",
	.fops	= &charm_modem_fops
};



static void mdm_status_fn(struct work_struct *work)
{
	int val;

	val = gpio_get_value(MDM2AP_STATUS);
	pr_err("%s: Status went low! = %d\n", __func__, val);
	__soc_restart();
}

static DECLARE_WORK(mdm_status_work, mdm_status_fn);

static void mdm_fatal_fn(struct work_struct *work)
{
	pr_err("%s: Got an error fatal!\n", __func__);
	__soc_restart();
}

static DECLARE_WORK(mdm_fatal_work, mdm_fatal_fn);

static int irqc;

static irqreturn_t errfatal(int irq, void *dev_id)
{
	pr_debug("charm got errfatal! Scheduling work to panic now...\n");
	irqc++;
	schedule_work(&mdm_fatal_work);
	disable_irq_nosync(MSM_GPIO_TO_INT(MDM2AP_ERRFATAL));
	return IRQ_HANDLED;
}
int first_time = 1;

static irqreturn_t status_change(int irq, void *dev_id)
{

	if (first_time) {
		first_time = 0;
		goto done;
	}
	pr_debug("Charm status went low! Scheduling work to panic now...\n");
	schedule_work(&mdm_status_work);
	disable_irq_nosync(MSM_GPIO_TO_INT(MDM2AP_STATUS));
done:
	return IRQ_HANDLED;
}

static int __init charm_modem_probe(struct platform_device *pdev)
{
	int ret, irq;
	struct charm_platform_data *d = pdev->dev.platform_data;

	gpio_request(AP2MDM_STATUS, "AP2MDM_STATUS");
	gpio_request(AP2MDM_ERRFATAL, "AP2MDM_ERRFATAL");

	gpio_direction_output(AP2MDM_STATUS, 1);
	gpio_direction_output(AP2MDM_ERRFATAL, 0);

	power_on_charm = d->charm_modem_on;
	power_down_charm = d->charm_modem_off;
	charm_intentional_reset = __charm_intentional_reset;

	gpio_request(MDM2AP_ERRFATAL, "MDM2AP_ERRFATAL");
	gpio_direction_input(MDM2AP_ERRFATAL);

	atomic_notifier_chain_register(&panic_notifier_list, &charm_panic_blk);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		pr_err("%s: could not get MDM2AP_ERRFATAL IRQ resource. \
			error=%d No IRQ will be generated on errfatal.",
			__func__, irq);
		goto errfatal_err;
	}

	ret = request_irq(irq, errfatal,
		IRQF_TRIGGER_HIGH , "charm errfatal", NULL);

	if (ret < 0) {
		pr_err("%s: MDM2AP_ERRFATAL IRQ#%d request failed with error=%d\
			. No IRQ will be generated on errfatal.",
			__func__, irq, ret);
	}

errfatal_err:

	irq = platform_get_irq(pdev, 1);
	if (irq < 0) {
		pr_err("%s: could not get MDM2AP_STATUS IRQ resource. \
			error=%d No IRQ will be generated on status change.",
			__func__, irq);
		goto status_err;
	}

	ret = request_threaded_irq(irq, NULL, status_change,
		IRQF_TRIGGER_FALLING, "charm status", NULL);

	if (ret < 0) {
		pr_err("%s: MDM2AP_STATUS IRQ#%d request failed with error=%d\
			. No IRQ will be generated on status change.",
			__func__, irq, ret);
	}

status_err:
	pr_info("%s: Registering charm modem\n", __func__);

	return misc_register(&charm_modem_misc);
}


static int __devexit charm_modem_remove(struct platform_device *pdev)
{

	return misc_deregister(&charm_modem_misc);
}

static struct platform_driver charm_modem_driver = {
	.remove         = charm_modem_remove,
	.driver         = {
		.name = "charm_modem",
		.owner = THIS_MODULE
	},
};

static int __init charm_modem_init(void)
{
	return platform_driver_probe(&charm_modem_driver, charm_modem_probe);
}

static void __exit charm_modem_exit(void)
{
	platform_driver_unregister(&charm_modem_driver);
}

module_init(charm_modem_init);
module_exit(charm_modem_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("msm8660 charm modem driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("charm_modem")


