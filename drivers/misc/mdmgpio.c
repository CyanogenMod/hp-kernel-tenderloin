/* Copyright (c) 2011, Hewlett-Packard Development Company,  All rights reserved.
 *
 * mdmgpio.c - Linux kernel modules for modem gppio control
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
#include <linux/mdmgpio.h>

static ssize_t uim_cd_gpio_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int value = 0;
	struct mdmgpio_platform_data *pdata;

	pdata = (struct mdmgpio_platform_data *)dev->platform_data;

	if (!pdata)
		return -EINVAL;

	if (pdata->get_gpio_value)
		value = pdata->get_gpio_value(pdata->uim_cd_gpio);
	else
		return -EINVAL;

	return sprintf(buf, "%u\n", value);
}

static ssize_t uim_cd_gpio_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = -EINVAL;
	char *after;
	struct mdmgpio_platform_data *pdata;
	unsigned long value = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	pdata = (struct mdmgpio_platform_data *)dev->platform_data;


	if (!pdata)
		return -EINVAL;
	if (isspace(*after))
		count++;
	if (count == size) {
		ret = count;
		if (pdata->set_gpio_value)
			pdata->set_gpio_value(pdata->uim_cd_gpio, !!value);
		else
			return -EINVAL;
	}
	return ret;
}

static ssize_t mdm_disable_gpio_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int value = 0;
	struct mdmgpio_platform_data *pdata;

	pdata = (struct mdmgpio_platform_data *)dev->platform_data;

	if (!pdata)
		return -EINVAL;
	if (pdata->get_gpio_value)
		value = pdata->get_gpio_value(pdata->mdm_disable_gpio);
	else
		return -EINVAL;

	return sprintf(buf,"%u\n", value);
}

static ssize_t mdm_disable_gpio_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = -EINVAL;
	char *after;
	struct mdmgpio_platform_data *pdata;
	unsigned long value = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	pdata = (struct mdmgpio_platform_data *)dev->platform_data;

	if (!pdata)
		return -EINVAL;
	if (isspace(*after))
		count++;
	if (count == size) {
		ret = count;
		if (pdata->set_gpio_value)
			pdata->set_gpio_value(pdata->mdm_disable_gpio, !!value);
		else
			return -EINVAL;
	}
	return ret;
}

static ssize_t mdm_poweron_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int value = 0;
	struct mdmgpio_platform_data *pdata;

	pdata = (struct mdmgpio_platform_data *)dev->platform_data;

	if (!pdata)
		return -EINVAL;
	if (pdata->mdm_poweron_status)
		value = pdata->mdm_poweron_status();

	return sprintf(buf,"%u\n", value);
}

static ssize_t mdm_poweron_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = -EINVAL;
	char *after;
	struct mdmgpio_platform_data *pdata;
	unsigned long value = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	pdata = (struct mdmgpio_platform_data *)dev->platform_data;

	if (!pdata)
		return -EINVAL;
	if (isspace(*after))
		count++;
	if (count == size) {
		ret = count;
		if (pdata->mdm_poweron)
			pdata->mdm_poweron(!!value);

	}
	return ret;
}

static struct device_attribute mdmgpio_attrs[] = {
	__ATTR(uim_cd_gpio,	0644, uim_cd_gpio_show, uim_cd_gpio_store),
	__ATTR(mdm_disable_gpio, 0644, mdm_disable_gpio_show, mdm_disable_gpio_store),
	__ATTR(mdm_poweron, 0644, mdm_poweron_show, mdm_poweron_store),
	__ATTR_NULL,
};

static int mdmgpio_probe(struct platform_device *pdev)
{
	int ret = 0;
	int i;
	struct mdmgpio_platform_data *pdata;

	printk("%s\n", __func__);
	pdata = (struct mdmgpio_platform_data *)pdev->dev.platform_data;
	if (pdata == NULL) {
		printk(KERN_ERR"%s: null platform data\n", __func__);
		return -EINVAL;
	}

	for( i = 0; attr_name(mdmgpio_attrs[i]); i++) {
		ret = device_create_file(&pdev->dev, &mdmgpio_attrs[i]);
		if (ret < 0) {
			printk(KERN_ERR "%s:failed to create sysfs file %s\n",
			__func__, attr_name(mdmgpio_attrs[i]));
			break;
		}
	}
	if (ret < 0)
		while(--i >= 0)
			device_remove_file(&pdev->dev, &mdmgpio_attrs[i]);

	return ret;
}

static int mdmgpio_remove(struct platform_device *pdev)
{
	int ret = 0;
	int i;
	struct mdmgpio_platform_data *pdata;

	printk("%s\n", __func__);

	pdata = (struct mdmgpio_platform_data *)pdev->dev.platform_data;

	if (!pdata)
		return -EINVAL;
	for(i = 0; attr_name(mdmgpio_attrs[i]); i++) {
		device_remove_file(&pdev->dev, &mdmgpio_attrs[i]);
	}
	return ret;
}

static struct platform_driver mdmgpio_driver = {
	.probe = mdmgpio_probe,
	.remove = mdmgpio_remove,
	.driver = {
		.name = "mdmgpio",
		.owner = THIS_MODULE,
	},
};

int __init mdmgpio_init(void)
{
	printk("%s\n", __func__);
	platform_driver_register(&mdmgpio_driver);
	return 0;
}

static void __exit mdmgpio_exit(void)
{
	printk("%s\n", __func__);
	platform_driver_unregister(&mdmgpio_driver);
}

module_init(mdmgpio_init);
module_exit(mdmgpio_exit);

MODULE_AUTHOR("Hao Song<hao.song@hp.com>");
MODULE_LICENSE("GPL");

