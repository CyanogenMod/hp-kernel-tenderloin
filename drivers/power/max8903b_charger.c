/*
 * Copyright (c) 2010, Hewlett-Packard Co. All rights reserved.
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
#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#include <linux/pm_runtime.h>
#include <mach/msm_iomap.h>

#include <linux/kernel.h>
#include <linux/power_supply.h>
#include <linux/sysfs.h>
#include <linux/max8903b_charger.h>


static struct max8903b_platform_data	*pdevice_resource;
static enum max8903b_current  current_limit;

static int max8903b_current_setup(enum max8903b_current value)
{
	int rc = 0;

	enum max8903b_current old_current_limit = current_limit;

	current_limit = value;

	switch (value)	{
		case CHARGE_DISABLE:
			/* disable charging */
			gpio_set_value(pdevice_resource->CEN_N_in, pdevice_resource->CEN_N_in_polarity ? 0 : 1);  /* charger disable */
			printk(KERN_INFO "%s: ### CHARGE_DISABLE\n", __func__);
			break;
		case CURRENT_ZERO:  // this is for no charger connection.
			gpio_set_value(pdevice_resource->DCM_in, pdevice_resource->DCM_in_polarity ? 1 : 0); /* usb mode */
			gpio_set_value(pdevice_resource->USUS_in, pdevice_resource->USUS_in_polarity ? 0 : 1); /* usb suspend */
			pdevice_resource->suspend_gpio_config();
			printk(KERN_INFO "%s: CURRENT_ZERO\n", __func__);
			break;
		case CURRENT_100MA:
			gpio_set_value(pdevice_resource->DCM_in, pdevice_resource->DCM_in_polarity? 1 : 0); /* usb mode */
			gpio_set_value(pdevice_resource->IUSB_in, pdevice_resource->IUSB_in_polarity ? 1 : 0);/* usb 100mA */
			gpio_set_value(pdevice_resource->USUS_in, pdevice_resource->USUS_in_polarity ? 1 : 0);
			gpio_set_value(pdevice_resource->CEN_N_in, pdevice_resource->CEN_N_in_polarity ? 1 : 0);  /* charger enable */
			printk(KERN_INFO "%s: CURRENT_100MA\n", __func__);
			break;
		case CURRENT_500MA:
			gpio_set_value(pdevice_resource->DCM_in, pdevice_resource->DCM_in_polarity ? 1 : 0); /* usb mode */
			gpio_set_value(pdevice_resource->IUSB_in, pdevice_resource->IUSB_in_polarity ? 0 : 1); /* usb 500mA */
			gpio_set_value(pdevice_resource->USUS_in, pdevice_resource->USUS_in_polarity ? 1 : 0);
			gpio_set_value(pdevice_resource->CEN_N_in, pdevice_resource->CEN_N_in_polarity ? 1 : 0);  /* charger enable */
			printk(KERN_INFO "%s: CURRENT_500MA\n", __func__);
			break;
		case CURRENT_750MA:
		case CURRENT_900MA:
		case CURRENT_1000MA:
		case CURRENT_1400MA:
		case CURRENT_1500MA:
		case CURRENT_2000MA:
			gpio_set_value(pdevice_resource->DCM_in, pdevice_resource->DCM_in_polarity ? 0 : 1); /* DC mode */
			if (pdevice_resource->set_DC_CHG_Mode_current) {
				if (pdevice_resource->set_DC_CHG_Mode_current(value) != 0) {
						current_limit = old_current_limit;
				}
			} else {
				printk(KERN_NOTICE "%s: set_DC_CHG_Mode_current is NULL\n", __func__);
			}
			gpio_set_value(pdevice_resource->CEN_N_in, pdevice_resource->CEN_N_in_polarity ? 1 : 0);  /* charger enable */
			printk(KERN_INFO "%s: CURRENT_750(4), 900(5), 1000(6), 1400(7), 2000MA(9): %d\n", __func__, value);
			break;
		default:
			current_limit = old_current_limit;
			printk(KERN_INFO "%s: Not supported current setting\n", __func__);
			rc = -EINVAL;
			break;
	}

	return rc;
}


static void max8903b_hw_init()
{
	gpio_set_value(pdevice_resource->DCM_in, pdevice_resource->DCM_in_polarity ? 1 : 0); /* usb mode */
	gpio_set_value(pdevice_resource->IUSB_in, pdevice_resource->IUSB_in_polarity ? 0 : 1); /* usb 500mA */
	gpio_set_value(pdevice_resource->USUS_in, pdevice_resource->USUS_in_polarity ? 1 : 0);
	gpio_set_value(pdevice_resource->CEN_N_in, pdevice_resource->CEN_N_in_polarity ? 1 : 0);  /* charger enable */
}

/* /sys/power/charger/status */
/*
 * - DC-input OK
 * - Charge in progress
 * - Charge Done
 * - Charge stop w/Fault
 */

/* /sys/power/charger/charging */
/*
 * Show the current current limit setting.
 */
static ssize_t show_currentlimit(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	char *str;

	//printk(KERN_INFO "show_currentlimit! \n");

	switch (current_limit) {
		case CHARGE_DISABLE:
			str = "none"; break;
		case CURRENT_ZERO:
			str = "current0ma";	break;
		case CURRENT_100MA:
			str = "current100ma"; break;
		case CURRENT_500MA:
			str = "current500ma"; break;
		case CURRENT_750MA:
			str = "current750ma"; break;
		case CURRENT_900MA:
			str = "current900ma"; break;
		case CURRENT_1000MA:
			str = "current1000ma"; break;
		case CURRENT_1400MA:
			str = "current1400ma"; break;
		case CURRENT_1500MA:
			str = "current1500ma"; break;
		case CURRENT_2000MA:
			str = "current2000ma"; break;
		default:
			return 0;
	}

	return  snprintf(buf, PAGE_SIZE, "%s\n", str);
}


/**
 * Parse the input for a charging command and optional current limit.
 */
static ssize_t store_currentlimit(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	enum max8903b_current value;
	//printk(KERN_INFO "store_currentlimit! \n");

	if (strncmp(buf, "off", 3) == 0 || strncmp(buf, "none", 4) == 0) {
		value = CHARGE_DISABLE;
	}
	else if (strncmp(buf, "current0ma", 10) == 0) {
		value = CURRENT_ZERO;
	}
	else if (strncmp(buf, "current100ma", 12) == 0) {
		value =  CURRENT_100MA;
	}
	else if (strncmp(buf, "current500ma", 12) == 0) {
		value = CURRENT_500MA;
	}
	else if (strncmp(buf, "current750ma", 12) == 0) {
		value = CURRENT_750MA;
	}
	else if (strncmp(buf, "current900ma", 12) == 0) {
		value = CURRENT_900MA;
	}
	else if (strncmp(buf, "current1000ma", 13) == 0) {
		value = CURRENT_1000MA;
	}
	else if (strncmp(buf, "current1400ma", 13) == 0) {
		value = CURRENT_1400MA;
	}
	else if (strncmp(buf, "current1500ma", 13) == 0) {
		value = CURRENT_1500MA;
	}
	else if (strncmp(buf, "current2000ma", 13) == 0) {
		value = CURRENT_2000MA;
	}
	else {
		printk(KERN_INFO "Invalid charging command: %s\n", buf);
		return 0;
	}

	max8903b_current_setup(value);

	return count;
}

void max8903b_set_charge_ma (unsigned ma)
{
	enum max8903b_current value;

	switch (ma){
		case 0:
			value = CURRENT_ZERO; break;
		case 100:
			value =  CURRENT_100MA; break;
		case 500:
			value = CURRENT_500MA; break;
		case 750:
			value = CURRENT_750MA; break;
		case 900:
			value = CURRENT_900MA; break;
		case 1000:
			value = CURRENT_1000MA; break;
		case 1400:
			value = CURRENT_1400MA; break;
		case 1500:
			value = CURRENT_1500MA; break;
		case 2000:
			value = CURRENT_2000MA; break;
		default:
			printk(KERN_INFO "%s: Invalid value: %d\n", __func__, ma);
			return;
	}

	if (current_limit != value) max8903b_current_setup(value);

	return;
}
EXPORT_SYMBOL (max8903b_set_charge_ma);

void max8903b_disable_charge()
{
	max8903b_current_setup(CHARGE_DISABLE);
}
EXPORT_SYMBOL (max8903b_disable_charge);


/* /sys/power/charger/chargerstatus */
/*
 * Show the current current limit setting.
 */
static ssize_t show_chargerstatus(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	char *str;
	int dc_ok, fault, status;

	if ( (dc_ok = gpio_get_value(pdevice_resource->DOK_N_out)) != 0)  /* invalid DC-In */
		str = "InvalidDC";   /* invalid DC-in */
	else 	if ( (fault = gpio_get_value(pdevice_resource->FLT_N_out)) == 0)
		str = "fault";  /* charger stopped with fault */
	else 	if ( (status = gpio_get_value(pdevice_resource->CHG_N_out)) == 0)
		str = "progress";  /* charge in progress */
	else
		str = "completed";  /* charge completed */

	dc_ok = gpio_get_value(pdevice_resource->DOK_N_out);
	fault = gpio_get_value(pdevice_resource->FLT_N_out);
	status = gpio_get_value(pdevice_resource->CHG_N_out);

//	printk(KERN_INFO "chargerstatus %s! | DC_OK=%d, STAT=%d, FAULT=%d\n",
//									str, dc_ok, status, fault);

	return  snprintf(buf, PAGE_SIZE, "%s\n", str);
}

static DEVICE_ATTR(currentlimit, S_IRUGO | S_IWUSR, show_currentlimit, store_currentlimit);
static DEVICE_ATTR(chargerstatus, S_IRUGO, show_chargerstatus, NULL);


static struct attribute *max8903b_power_sysfs_entries[] = {
	&dev_attr_currentlimit.attr,
	&dev_attr_chargerstatus.attr,
	NULL
};

static struct attribute_group max8903b_power_attr_group = {
	.name = NULL,
	.attrs = max8903b_power_sysfs_entries
};


static int __devinit max8903b_charger_probe(struct platform_device *pdev)
{
	struct max8903b_platform_data  *pdata;
	int ret;

	pdata = pdev->dev.platform_data;

	pdevice_resource = pdata;

	pdata->request_release_gpios(1);

	ret = sysfs_create_group(&pdev->dev.kobj, &max8903b_power_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "failed to create sysfs 'charger' entries\n");
	} else {
		ret = sysfs_create_link(power_kobj, &pdev->dev.kobj, "charger");
		if (ret)
			dev_err(&pdev->dev, "failed to create sysfs 'charger' link\n");
	}

	//max8903b_hw_init();

	return 0;
}

static int __devexit max8903b_charger_remove(struct platform_device *pdev)
{
	struct max8903b_platform_data  *pdata;
	pdata = pdev->dev.platform_data;
	pdata->request_release_gpios(0);
	return 0;
}

#ifdef CONFIG_PM
static int max8903b_resume(struct platform_device *pdev)
{
	int rc = 0;
	struct max8903b_platform_data  *pdata;

	pdata = pdev->dev.platform_data;

	printk("%s: resume\n", __func__);
	rc = pdata->request_release_gpios(1);

	return rc;
}

static int max8903b_suspend(struct platform_device *pdev, pm_message_t state)
{
	int rc = 0;
	struct max8903b_platform_data  *pdata;

	pdata = pdev->dev.platform_data;

	printk("%s: suspend\n", __func__);
	rc = pdata->request_release_gpios(0);

	return rc;
}
#endif

static struct platform_driver max8903b_charger_driver = {
	.probe = max8903b_charger_probe,
	.remove = __devexit_p(max8903b_charger_remove),
	.driver = {
		.name = "max8903b_chg",
		.owner = THIS_MODULE,
	},
#ifdef CONFIG_PM
	.suspend = max8903b_suspend,
	.resume = max8903b_resume,
#endif
};

static int __init max8903b_charger_init(void)
{
	int rc;

	rc = platform_driver_register(&max8903b_charger_driver);
	if (rc ==0)
		printk(KERN_INFO "max8903b driver registeration! rc = %d\n", rc);
	else
		printk(KERN_DEBUG "NO max8903b driver registeration!");

	return rc;
}

static void __exit max8903b_charger_exit(void)
{
	platform_driver_unregister(&max8903b_charger_driver);
}

module_init(max8903b_charger_init);
module_exit(max8903b_charger_exit);

MODULE_DESCRIPTION("MAX8903B battery charger driver");
MODULE_AUTHOR("Kyoung Kim <kyoung-il.kim@hp.com><kyoung.kim@palm.com>");
MODULE_LICENSE("GPL v2");
