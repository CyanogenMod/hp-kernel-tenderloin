/* drivers/misc/bluetooth-power-pe.c
 *
 * Bluetooth power state driver - Used by the user space to notify
 * the kernel, about current power on status of the Bluetooth.
 * Copyright (C) 2009 Palm, Inc.
 * Author: Rajmohan Mani <rajmohan.mani@palm.com>
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
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/bluetooth-power-pe.h>

#define MODULE_NAME "bt_power"

typedef struct {
	struct bluetooth_power_state_platform_data * pdata;
	struct mutex lock;
	int (* bt_power) (unsigned int on);
	unsigned int power_state;
} bt_power_state;

static ssize_t bluetooth_power_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc = 0;
	bt_power_state * bt_powerstate = (bt_power_state *) (dev_get_drvdata (dev) );

	if (!bt_powerstate) {
		return -EINVAL;
	}

	if (!buf) {
		return 0;
	}

	mutex_lock (&bt_powerstate->lock);
	rc = snprintf(buf, PAGE_SIZE, "%s\n", bt_powerstate->power_state ? "1" : "0");
	mutex_unlock (&bt_powerstate->lock);

	return rc;
}

static ssize_t bluetooth_power_state_store(struct device *dev, struct device_attribute *attr, 
				      const char *buf, size_t count)
{
	bt_power_state * bt_powerstate = (bt_power_state *) (dev_get_drvdata (dev) );
	unsigned int last_power_state = 0;
	if (!bt_powerstate) {
		return -EINVAL;
	}

	if (!buf || !count) {
		return 0;
	}

	mutex_lock (&bt_powerstate->lock);
	last_power_state = bt_powerstate->power_state;
	if ('0' == buf[0]) {
		bt_powerstate->power_state = 0;
	} else {
		bt_powerstate->power_state = 1;
	}

	// Call the bt_power function, only when the power state changes
	if (bt_powerstate->power_state != last_power_state) {
		bt_powerstate->pdata->bt_power (bt_powerstate->power_state);
	}
	mutex_unlock (&bt_powerstate->lock);

	return count;
}

static DEVICE_ATTR(bluetooth_power, S_IRUGO | S_IWUSR, bluetooth_power_state_show, bluetooth_power_state_store);

static int __devinit bluetooth_power_state_probe(struct platform_device *pdev)
{
	int ret = 0;

	bt_power_state * bt_powerstate;

	if (pdev->dev.platform_data == NULL ) {
        printk(KERN_ERR "%s: no platform data found.\n", __func__ );
        return -ENODEV;
    }

	bt_powerstate = kzalloc(sizeof(bt_power_state), GFP_KERNEL);
	if (!bt_powerstate) {
		printk ( KERN_ERR "bt power state state memory allocation failed \n");
		return -ENOMEM;
	}

	if ((ret = device_create_file(&(pdev->dev), &dev_attr_bluetooth_power))) {
		printk(KERN_ERR "Failed to create sysfs device for bluetooth power state\n");
		kfree (bt_powerstate);
		return -EINVAL;
	}

	bt_powerstate->pdata = (struct bluetooth_power_state_platform_data *) (pdev->dev.platform_data);
	bt_powerstate->bt_power = bt_powerstate->pdata->bt_power;

	bt_powerstate->power_state = 1;

	if(bt_powerstate->bt_power)
		bt_powerstate->bt_power(bt_powerstate->power_state);

	// Init mutex 
	mutex_init(&bt_powerstate->lock);

	dev_set_drvdata(&pdev->dev, bt_powerstate);

	return ret;
}

static struct platform_driver bluetooth_power_state_driver = {
	.probe = bluetooth_power_state_probe,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init bluetooth_power_state_init(void)
{
	return platform_driver_register(&bluetooth_power_state_driver);
}

module_init(bluetooth_power_state_init);

MODULE_DESCRIPTION("Bluetooth power state");
MODULE_AUTHOR("Rajmohan Mani <rajmohan.mani@palm.com>");
MODULE_LICENSE("GPL");

