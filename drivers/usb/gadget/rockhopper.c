/*
 * rockhopper.c -- Rockhopper gadget driver
 *
 * Copyright (C) 2009 Palm, Inc.
 * Copyright (C) 2011 HP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* #define VERBOSE_DEBUG */

#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#ifdef CONFIG_USB_GADGET_EVENT
#include <linux/usb/gadget_event.h>
#endif

#define CONFIG_USB_ETH_RNDIS /* REVISIT - always use rndis */
//#define CONFIG_MASSSTORAGE_PE
#define USE_ROCKHOPPER_CLASS_PARENT
#define USE_NDUID_AS_SERIAL_NUMBER
#define USE_SYSFS_CONFIG_CHANGE


#include "u_ether.h"
#include "u_serial.h"
#include "gadget_chips.h"
#include "rockhopper.h"

#define DRIVER_DESC		"Rockhopper Gadget"
#define DRIVER_VERSION		"Cinco de Mayo 2009"

#ifdef USE_NDUID_AS_SERIAL_NUMBER
extern char *nduid_string_get(void);
#endif
#ifdef USE_SYSFS_CONFIG_CHANGE
/* avoid putting code and data into init sections */
#undef __init
#undef __initdata
#undef __exit
#undef __exit_p

#define __init
#define __initdata
#define __exit
#define __exit_p(x) x
#endif

/*-------------------------------------------------------------------------*/
/*
 * Kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */

#include "composite_pe.c"
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"

#include "f_rndis.c"
#include "rndis.c"
#include "u_ether.c"

#undef DBG
#undef VDBG
#undef ERROR
#undef WARNING
#undef INFO

#ifdef CONFIG_MASSSTORAGE_PE
#include "f_mass_storage_pe.c"
#else
#include "f_mass_storage.c"
#endif

#include "f_acm.c"
#include "f_racm.c"
#include "f_serial.c"
#include "u_serial.c"

#include "f_novacom.c"

/*-------------------------------------------------------------------------*/

#undef DBG
#undef VDBG
#undef ERROR
#undef WARNING
#undef INFO

#define DBG(d, fmt, args...) \
	dev_dbg(&(d)->gadget->dev , fmt , ## args)
#define VDBG(d, fmt, args...) \
	dev_vdbg(&(d)->gadget->dev , fmt , ## args)
#define ERROR(d, fmt, args...) \
	dev_err(&(d)->gadget->dev , fmt , ## args)
#define WARNING(d, fmt, args...) \
	dev_warn(&(d)->gadget->dev , fmt , ## args)
#define INFO(d, fmt, args...) \
	dev_info(&(d)->gadget->dev , fmt , ## args)

/*-------------------------------------------------------------------------*/

#define RH_CONFIG_UMS			1
#define RH_CONFIG_UMS_NOVACOM		2
#define RH_CONFIG_PASSTHRU		3
#define RH_CONFIG_USBNET_PASSTHRU	4
#define RH_CONFIG_USBNET_UMS_NOVACOM	5
#define RH_CONFIG_PASSTHRU_NOVACOM	6

#define RH_CONFIG_SENTINEL		7

#define RH_FUNC_RNDIS		(1<<0)	/* this needs to be the first */
#define RH_FUNC_ACM		(1<<1)
#define RH_FUNC_SERIAL		(1<<2)
#define RH_FUNC_MASS_STORAGE	(1<<3)
#define RH_FUNC_NOVACOM		(1<<4)

static unsigned short rh_func_bitmap[] = {
	[RH_CONFIG_UMS] = RH_FUNC_MASS_STORAGE,
	[RH_CONFIG_UMS_NOVACOM] = RH_FUNC_MASS_STORAGE | RH_FUNC_NOVACOM,
	[RH_CONFIG_PASSTHRU] = RH_FUNC_ACM | RH_FUNC_SERIAL,
	[RH_CONFIG_USBNET_PASSTHRU] =
				RH_FUNC_RNDIS | RH_FUNC_ACM | RH_FUNC_SERIAL,
	[RH_CONFIG_USBNET_UMS_NOVACOM] =
			RH_FUNC_RNDIS | RH_FUNC_MASS_STORAGE | RH_FUNC_NOVACOM,
	[RH_CONFIG_PASSTHRU_NOVACOM] =
		RH_FUNC_ACM | RH_FUNC_SERIAL | RH_FUNC_NOVACOM,
	[RH_CONFIG_SENTINEL] = 0,
};

#define RH_OPT_NO_SERIAL_NUMBER	(1<<0)

static unsigned char rh_option[] = {
	[RH_CONFIG_PASSTHRU] = RH_OPT_NO_SERIAL_NUMBER,
	[RH_CONFIG_USBNET_PASSTHRU] = RH_OPT_NO_SERIAL_NUMBER,
	[RH_CONFIG_PASSTHRU_NOVACOM] = RH_OPT_NO_SERIAL_NUMBER,
	[RH_CONFIG_SENTINEL] = 0,
};

static unsigned short rh_product_id[] = {
	[RH_CONFIG_UMS] = CONFIG_USB_ROCKHOPPER_PID_BASE +
			(0x4 << CONFIG_USB_ROCKHOPPER_PID_SHIFT),
	[RH_CONFIG_UMS_NOVACOM] = CONFIG_USB_ROCKHOPPER_PID_BASE +
			(0x2 << CONFIG_USB_ROCKHOPPER_PID_SHIFT),
	[RH_CONFIG_PASSTHRU] = CONFIG_USB_ROCKHOPPER_PID_BASE +
			(0x3 << CONFIG_USB_ROCKHOPPER_PID_SHIFT),
	[RH_CONFIG_USBNET_PASSTHRU] = CONFIG_USB_ROCKHOPPER_PID_BASE +
			(0x5 << CONFIG_USB_ROCKHOPPER_PID_SHIFT),
	[RH_CONFIG_USBNET_UMS_NOVACOM] = CONFIG_USB_ROCKHOPPER_PID_BASE +
			(0x6 << CONFIG_USB_ROCKHOPPER_PID_SHIFT),
	[RH_CONFIG_PASSTHRU_NOVACOM] = CONFIG_USB_ROCKHOPPER_PID_BASE +
			(0x7 << CONFIG_USB_ROCKHOPPER_PID_SHIFT),
	[RH_CONFIG_SENTINEL] = 0,
};

static char *rh_config_description[] = {
	[RH_CONFIG_UMS] = "ums",
	[RH_CONFIG_UMS_NOVACOM] = "ums+novcom",
	[RH_CONFIG_PASSTHRU] = "passthru",
	[RH_CONFIG_USBNET_PASSTHRU] = "usbnet+passthru",
	[RH_CONFIG_USBNET_UMS_NOVACOM] = "usbnet+ums+novacom",
	[RH_CONFIG_PASSTHRU_NOVACOM] = "passthru+novacom",
	[RH_CONFIG_SENTINEL] = NULL,
};

static unsigned int config_num = RH_CONFIG_UMS; /* 1 */
module_param(config_num, uint, S_IRUGO);
MODULE_PARM_DESC(config_num, "config number, default=1 (UMS)");

static int use_acm = false;
module_param(use_acm, bool, 0);
MODULE_PARM_DESC(use_acm, "Use CDC ACM, default=false");

/* REVISIT */
#define RH_NLUNS	1	/* # of luns */
#define RH_NPORTS	2	/* # of serial ports */

/*-------------------------------------------------------------------------*/

static struct usb_device_descriptor device_desc = {
	.bLength =		sizeof device_desc,
	.bDescriptorType =	USB_DT_DEVICE,

	.bcdUSB =		__constant_cpu_to_le16 (0x0200),

	.bDeviceClass =		USB_CLASS_PER_INTERFACE,
	.bDeviceSubClass =	USB_CLASS_PER_INTERFACE,
	.bDeviceProtocol =	0,
	/* .bMaxPacketSize0 = f(hardware) */

	/* Vendor and product id defaults change according to what configs
	 * we support.  (As does bNumConfigurations.)  These values can
	 * also be overridden by module parameters.
	 */
	.idVendor =	__constant_cpu_to_le16 (CONFIG_USB_ROCKHOPPER_VID),
	/* .idProduct =	DYNAMIC */
	/* .bcdDevice = f(hardware) */
	/* .iManufacturer = DYNAMIC */
	/* .iProduct = DYNAMIC */
	/* NO SERIAL NUMBER */
	.bNumConfigurations =	1,
};

static struct usb_otg_descriptor otg_descriptor = {
	.bLength =		sizeof otg_descriptor,
	.bDescriptorType =	USB_DT_OTG,

	/* REVISIT SRP-only hardware is possible, although
	 * it would not be called "OTG" ...
	 */
	.bmAttributes =		USB_OTG_SRP | USB_OTG_HNP,
};

static const struct usb_descriptor_header *otg_desc[] = {
	(struct usb_descriptor_header *) &otg_descriptor,
	NULL,
};


/* string IDs are assigned dynamically */

#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIALNUMBER_IDX		2
#define STRING_DESCRIPTION_IDX		3

static char serial_number[64];
static char config_description[64];

static int serial_number_idx;

static struct usb_string strings_dev[] = {
	[STRING_MANUFACTURER_IDX].s = CONFIG_USB_ROCKHOPPER_MANUFACTURER_STRING,
	[STRING_PRODUCT_IDX].s = CONFIG_USB_ROCKHOPPER_PRODUCT_STRING,
	[STRING_SERIALNUMBER_IDX].s = serial_number,
	[STRING_DESCRIPTION_IDX].s = config_description,
	{  } /* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

/*-------------------------------------------------------------------------*/
#ifndef CONFIG_MASSSTORAGE_PE
static struct fsg_module_parameters mod_data = {
	.stall = 1,
	.luns = RH_NLUNS,
};
FSG_MODULE_PARAMETERS(/* no prefix */, mod_data);
static struct fsg_common *fsg_common;
#endif

static u8 hostaddr[ETH_ALEN];

static int __init rockhopper_do_config(struct usb_configuration *c)
{
	int ret;

	if (rh_func_bitmap[config_num] & RH_FUNC_RNDIS) {
		/* set up network link layer */
		ret = gether_setup(c->cdev->gadget, hostaddr);
		if (ret < 0) {
			ERROR(c->cdev, "gether_setup failed (ret=%d)\n", ret);
			return ret;
		}

		ret = rndis_bind_config(c, hostaddr);
		if (ret) {
			ERROR(c->cdev, "rndis_bind_config failed (ret=%d)\n", ret);
			return ret;
		}
		INFO(c->cdev, "rndis function is added\n");
	}
	if (rh_func_bitmap[config_num] & RH_FUNC_ACM) {
		/* initialize tty driver */
		ret = gserial_setup(c->cdev->gadget, RH_NPORTS);
		if (ret < 0) {
			ERROR(c->cdev, "gserial_setup failed (ret=%d)\n", ret);
			return ret;
		}

		if (use_acm)
			ret = acm_bind_config(c, 0);
		else
			ret = racm_bind_config(c, 0);
		if (ret) {
			ERROR(c->cdev, "(r)acm_bind_config failed (ret=%d)\n", ret);
			return ret;
		}
		INFO(c->cdev, "acm function is added\n");
	}
	if (rh_func_bitmap[config_num] & RH_FUNC_SERIAL) {
		ret = gser_bind_config(c, 1);
		if (ret) {
			ERROR(c->cdev, "gser_bind_config failed (ret=%d)\n", ret);
			return ret;
		}
		INFO(c->cdev, "serial function is added\n");
	}
	if (rh_func_bitmap[config_num] & RH_FUNC_MASS_STORAGE) {
#ifdef CONFIG_MASSSTORAGE_PE
		ret = mass_storage_function_add(c, RH_NLUNS);
#else
		ret = fsg_add(c->cdev, c, fsg_common);
#endif
		if (ret) {
			ERROR(c->cdev, "mass_storage_function failed (ret=%d)\n", ret);
			return ret;
		}
		INFO(c->cdev, "mass storage function is added\n");
	}
	if (rh_func_bitmap[config_num] & RH_FUNC_NOVACOM) {
		ret = novacom_bind_config(c);
		if (ret) {
			ERROR(c->cdev, "novacom_bind_config failed (ret=%d)\n", ret);
			return ret;
		}
		INFO(c->cdev, "novacom function is added\n");
	}

	if (rh_option[config_num] & RH_OPT_NO_SERIAL_NUMBER) {
		device_desc.iSerialNumber = 0;
	} else {
		device_desc.iSerialNumber = serial_number_idx;
	}

	return ret;
}

static struct usb_configuration rockhopper_config_driver = {
	.label			= "ROCKHOPPER",
	.bind			= rockhopper_do_config,
	/* .bConfigurationValue	= DYNAMIC (config_num) */
	/* .iConfiguration = DYNAMIC */
	.bmAttributes		= USB_CONFIG_ATT_SELFPOWER,
};

/*-------------------------------------------------------------------------*/

static int __init rockhopper_bind(struct usb_composite_dev *cdev)
{
	int			gcnum;
	struct usb_gadget	*gadget = cdev->gadget;
	int			status;

#ifndef CONFIG_MASSSTORAGE_PE
	/* set up mass storage function */
	fsg_common = fsg_common_from_params(0, cdev, &mod_data);
	if (IS_ERR(fsg_common)) {
		status = PTR_ERR(fsg_common);
		goto fail1;
	}
#endif

	/* set up device descriptor */
	device_desc.idVendor = cpu_to_le16(CONFIG_USB_ROCKHOPPER_VID);
	device_desc.idProduct = cpu_to_le16(rh_product_id[config_num]);
	device_desc.bNumConfigurations = 1;

	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0300 | gcnum);
	else {
		/* We assume that can_support_ecm() tells the truth;
		 * but if the controller isn't recognized at all then
		 * that assumption is a bit more likely to be wrong.
		 */
		dev_warn(&gadget->dev,
				"controller '%s' not recognized; trying %s\n",
				gadget->name,
				rockhopper_config_driver.label);
		device_desc.bcdDevice =
			__constant_cpu_to_le16(0x0300 | 0x0099);
	}

	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */

	/* device descriptor strings: manufacturer, product */
	status = usb_string_id(cdev);
	if (status < 0)
		goto fail;
	strings_dev[STRING_MANUFACTURER_IDX].id = status;
	device_desc.iManufacturer = status;

	/* product id */
	status = usb_string_id(cdev);
	if (status < 0)
		goto fail;
	strings_dev[STRING_PRODUCT_IDX].id = status;
	device_desc.iProduct = status;

	/* serial number */
#if defined(USE_NDUID_AS_SERIAL_NUMBER) && defined(CONFIG_PALM_NDUID)
	if (nduid_string_get()) {
		strncpy(serial_number, nduid_string_get(),
			sizeof(serial_number)-1);
		dev_info(&gadget->dev, "serial_number=%s\n", serial_number);
	} else {
		dev_err(&gadget->dev, "can't get nduid\n");
		snprintf(serial_number, sizeof(serial_number),
			 "0123456789ABCDEF");
	}
#else
	snprintf(serial_number, sizeof(serial_number), "0123456789ABCDEF");
#endif
	status = usb_string_id(cdev);
	if (status < 0)
		goto fail;
	strings_dev[STRING_SERIALNUMBER_IDX].id = status;
	serial_number_idx = status; /* save for later use */
	device_desc.iSerialNumber = status;

	/* config description */
	snprintf(config_description, sizeof(config_description),
		 "%s", rh_config_description[config_num]);
	status = usb_string_id(cdev);
	if (status < 0)
		goto fail;
	strings_dev[STRING_DESCRIPTION_IDX].id = status;
	rockhopper_config_driver.iConfiguration = status;

	rockhopper_config_driver.bConfigurationValue = config_num;

	if (gadget_is_otg(cdev->gadget)) {
		rockhopper_config_driver.descriptors = otg_desc;
		rockhopper_config_driver.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	}

	/* register our configuration */
	status = usb_add_config(cdev, &rockhopper_config_driver);
	if (status < 0) {
		dev_err(&gadget->dev,
			"usb_add_config failed (status=%d)\n", status);
		goto fail;
	}

	dev_info(&gadget->dev, "%s, version: " DRIVER_VERSION "\n",
			DRIVER_DESC);
#ifndef CONFIG_MASSSTORAGE_PE
	fsg_common_put(fsg_common);
#endif
	return 0;

fail:
#ifndef CONFIG_MASSSTORAGE_PE
	fsg_common_put(fsg_common);
fail1:
#endif
	gether_cleanup();
	gserial_cleanup();
	return status;
}

static int __exit rockhopper_unbind(struct usb_composite_dev *cdev)
{
	gether_cleanup();
	gserial_cleanup();
	return 0;
}

static struct usb_composite_driver rockhopper_driver = {
	.name		= "g_rockhopper",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.bind		= rockhopper_bind,
	.unbind		= __exit_p(rockhopper_unbind),
};

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Toshi Kikuchi");
MODULE_LICENSE("GPL");

/*-------------------------------------------------------------------------*/

static struct class *usb_gadget_class;

#ifdef USE_SYSFS_CONFIG_CHANGE

static ssize_t
config_num_show(struct class *class,
				struct class_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%d\n", config_num);
}

static ssize_t
config_num_store(struct class *class,
					struct class_attribute *attr,
					const char *buf,
					size_t count)
{
	char *endp;
	int n = simple_strtoul(buf, &endp, 0);
	int status;

	if (n != config_num) {
		printk(KERN_INFO "rockhopper: switching to config_num=%d\n", n);
		usb_composite_unregister(&rockhopper_driver);
		printk(KERN_INFO "rockhopper: unregister driver done\n");
		config_num = n;
		printk(KERN_INFO "rockhopper: start re-registering driver\n");
		status = usb_composite_register(&rockhopper_driver);
		if (status) {
			printk(KERN_ERR
			       "rockhopper: usb_composite_register failed %d\n",
			       status);
		}
	}
	return count;
}

static CLASS_ATTR(config_num, S_IRUGO | S_IWUSR, config_num_show, config_num_store);

#endif /* USE_SYSFS_CONFIG_CHANGE */

struct device *
rockhopper_create_device(const char *name)
{
	struct class *cls = usb_gadget_class;
	struct device *dev;
	dev_t dev0 = MKDEV(0, 0);

	if (!cls) {
		printk(KERN_ERR "rockhopper: class is not itialized\n");
		return NULL;
	}
	dev = device_create(cls, NULL, dev0, NULL, name);
	if (IS_ERR(dev)) {
		printk(KERN_ERR "rockhopper: device_create error\n");
		return NULL;
	}

	return dev;
}
EXPORT_SYMBOL(rockhopper_create_device);

void
rockhopper_destroy_device(struct device *dev)
{
	if (dev) {
		device_unregister(dev);
	}
}
EXPORT_SYMBOL(rockhopper_destroy_device);

/*-------------------------------------------------------------------------*/

static int __init init(void)
{
	int status;

	usb_gadget_class = class_create(THIS_MODULE, "usb_gadget");
	if (IS_ERR(usb_gadget_class)) {
		status = PTR_ERR(usb_gadget_class);
		printk(KERN_ERR
		       "rockhopper: unable to create usb_gadget class %d\n",
		       status);
		goto err1;
	}

#ifdef USE_SYSFS_CONFIG_CHANGE
	status = class_create_file(usb_gadget_class, &class_attr_config_num);
	if (status) {
		printk(KERN_ERR
		       "class_create_file failed (status=%d)\n", status);
		goto err2;
	}
#endif
	status = usb_composite_register(&rockhopper_driver);
	if (status) {
		goto err3;
	}

	return 0;

err3:
#ifdef USE_SYSFS_CONFIG_CHANGE
	class_remove_file(usb_gadget_class, &class_attr_config_num);
#endif
err2:
	class_destroy(usb_gadget_class);
err1:
	return status;
}
module_init(init);

static void __exit cleanup(void)
{
	usb_composite_unregister(&rockhopper_driver);

#ifdef USE_SYSFS_CONFIG_CHANGE
	class_remove_file(usb_gadget_class, &class_attr_config_num);
#endif
	class_destroy(usb_gadget_class);
}
module_exit(cleanup);
