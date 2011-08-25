/* Copyright (c) 2010, Hewlett-Packard Development Company, L.P. All rights reserved.
 *
 * brcm4751.c - Linux kernel modules for broadcom GPS module 4751
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/brcm4751.h>

static struct brcm4751_cfg brcm4751_cfg;

static ssize_t regpu_gpio_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int value;

	value = gpio_get_value_cansleep(brcm4751_cfg.regpu_sys_gpio);

	return sprintf(buf, "%u\n", value);
}

static ssize_t regpu_gpio_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = -EINVAL;
	char *after;
	unsigned long value = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (isspace(*after))
		count++;

	if (count == size) {
		ret = count;
		gpio_set_value_cansleep(brcm4751_cfg.regpu_sys_gpio, value);
	}

	return ret;
}

static ssize_t reset_gpio_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int value;

	value = gpio_get_value_cansleep(brcm4751_cfg.reset_sys_gpio);

	return sprintf(buf, "%u\n", value);
}

static ssize_t reset_gpio_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = -EINVAL;
	char *after;
	unsigned long value = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (isspace(*after))
		count++;

	if (count == size) {
		ret = count;
		gpio_set_value_cansleep(brcm4751_cfg.reset_sys_gpio, value);
	}

	return ret;
}

static struct device_attribute brcm4751_attrs[] = {
	__ATTR(regpu_gpio, 0644, regpu_gpio_show, regpu_gpio_store),
	__ATTR(reset_gpio, 0644, reset_gpio_show, reset_gpio_store),
	__ATTR_NULL,
};

static int brcm4751_probe(struct platform_device *pdev)
{
	int ret = 0;
	int i;
	struct brcm4751_platform_data *pdata;

	pdata = (struct brcm4751_platform_data *)pdev->dev.platform_data;
	if (pdata == NULL) {
		printk(KERN_ERR "%s: null platform data\n", __func__);
		return -EINVAL;
	}

	ret = pdata->config(true, &brcm4751_cfg);

	if (ret < 0){
		printk(KERN_ERR "%s: failed to configure brcm4751\n", __func__);
		return ret;
	}

	for (i = 0; attr_name(brcm4751_attrs[i]); i++) {
		ret = device_create_file(&pdev->dev, &brcm4751_attrs[i]);
		if (ret < 0) {
			printk(KERN_ERR "%s: failed to create sysfs file %s\n",
				__func__, attr_name(brcm4751_attrs[i]));
			break;
		}
	}
	if (ret < 0)
		while (--i >= 0)
			device_remove_file(&pdev->dev, &brcm4751_attrs[i]);

	return ret;
}

static int brcm4751_remove(struct platform_device *pdev)
{
	int ret = 0;
	int i;
	struct brcm4751_platform_data *pdata;

	for (i = 0; attr_name(brcm4751_attrs[i]); i++) {
		device_remove_file(&pdev->dev, &brcm4751_attrs[i]);
	}

	pdata = (struct brcm4751_platform_data *)pdev->dev.platform_data;
	if (pdata == NULL) {
		printk(KERN_ERR "%s: null platform data\n", __func__);
		return -EINVAL;
	}

	ret = pdata->config(false, NULL);
	if (ret < 0){
		printk(KERN_ERR "%s: failed to configure brcm4751\n", __func__);
		return ret;
	}

	return ret;
}

static struct platform_driver brcm4751_driver = {
	.probe  = brcm4751_probe,
	.remove = brcm4751_remove,
	.driver = {
		.name = "brcm4751",
		.owner = THIS_MODULE,
	},
};

static int __init brcm4751_init(void)
{
	return platform_driver_register(&brcm4751_driver);
}

static void __exit brcm4751_exit(void)
{
	platform_driver_unregister(&brcm4751_driver);
}

module_init(brcm4751_init);
module_exit(brcm4751_exit);

MODULE_AUTHOR("Sam Lin <zhangbin.lin@hp.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Broadcom GPS module 4751 GPIO configurations");
