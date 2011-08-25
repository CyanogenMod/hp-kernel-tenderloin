/*
 * gadget_event.c
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/usb/gadget_event.h>

#define GADGET_EVENT_NAME	"usb_gadget"

static struct gadget_event_state {
	/* mass storage */
	int	storage_events_enabled;
	int	host_connected;
	int	bus_suspended;
	int	media_loaded;
	int	media_requested;
	/* power */
	enum gadget_event_source_type	source;
	int	current_mA;
} the_state = {
	/* mass storage */
	0, -1, -1, -1, -1,
	/* power */
	G_EV_SOURCE_UNKNOWN, -1,
};

static struct platform_device gadget_event_device = {
	.name = GADGET_EVENT_NAME,
	.id = -1,
};

static struct platform_driver gadget_event_driver = {
	.driver = {
		.name  = GADGET_EVENT_NAME,
		.bus   = &platform_bus_type,
		.owner = THIS_MODULE,
	},
};

/*
 * storage events
 */
void
gadget_event_enable_storage_events(int enable)
{
	if (enable == the_state.storage_events_enabled) {
		printk(KERN_INFO "gadget_event: %s storage events (no change)\n", enable ? "enable" : "disable");
		return;
	}
	if (enable) {
		printk(KERN_INFO "gadget_event: enable storage events\n");
		the_state.storage_events_enabled = 1;
	} else if (!enable) {
		printk(KERN_INFO "gadget_event: disable storage events\n");
		if (the_state.host_connected)
			gadget_event_host_connected(0);
		if (the_state.bus_suspended)
			gadget_event_bus_suspended(0);
		if (the_state.media_loaded)
			gadget_event_media_loaded(0);
		if (the_state.media_requested)
			gadget_event_media_requested(0);
		the_state.storage_events_enabled = 0;
	}
}
EXPORT_SYMBOL(gadget_event_enable_storage_events);

/*
 * host_connected
 */
static ssize_t
show_host_connected(struct device *dev, struct device_attribute *attr,
		    char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", the_state.host_connected);
}

static DEVICE_ATTR(host_connected, S_IRUGO, show_host_connected, NULL);

void
gadget_event_host_connected(int host_connected)
{
	char *var = host_connected ?
		"G_HOST_CONNECTED=1" : "G_HOST_CONNECTED=0";
	char *envp[] = {
		"G_SUBSYSTEM=storage",
		"G_ACTION=HOST_STATE_CHANGED",
		var,
		NULL
	};

	if (!the_state.storage_events_enabled) {
		printk(KERN_INFO "gadget_event: storage events is disabled\n");
		return;
	}
	if (the_state.host_connected == host_connected) {
		printk(KERN_INFO "gadget_event: host_connected=%d (no change)\n", host_connected);
		return;
	}
	the_state.host_connected = host_connected;

	printk(KERN_INFO "gadget_event: host_connected=%d\n", host_connected);
	kobject_uevent_env(&gadget_event_device.dev.kobj, KOBJ_CHANGE, envp);
}
EXPORT_SYMBOL(gadget_event_host_connected);

static struct delayed_work host_connected_work;
static struct delayed_work host_disconnected_work;

static void host_connected_func(struct work_struct *w)
{
	gadget_event_host_connected(1);
}

static void host_disconnected_func(struct work_struct *w)
{
	gadget_event_host_connected(0);
}

void
gadget_event_host_connected_async(int host_connected, unsigned long delay)
{
	static int scheduled_host_connected = 0;
	static int scheduled_host_disconnected = 0;

	if (host_connected) {
		if (scheduled_host_connected)
			return;
		scheduled_host_disconnected = 0;
		scheduled_host_connected = 1;
		printk(KERN_INFO "gadget_event: schedule host_connected\n");
		cancel_delayed_work(&host_disconnected_work);
		schedule_delayed_work(&host_connected_work, delay);
	} else {
		if (scheduled_host_disconnected)
			return;
		scheduled_host_connected = 0;
		scheduled_host_disconnected = 1;
		printk(KERN_INFO "gadget_event: schedule host_disconnected\n");
		cancel_delayed_work(&host_connected_work);
		schedule_delayed_work(&host_disconnected_work, delay);
	}
}
EXPORT_SYMBOL(gadget_event_host_connected_async);

/*
 * bus_suspended
 */
static ssize_t
show_bus_suspended(struct device *dev, struct device_attribute *attr,
		    char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", the_state.bus_suspended);
}

static DEVICE_ATTR(bus_suspended, S_IRUGO, show_bus_suspended, NULL);

void
gadget_event_bus_suspended(int bus_suspended)
{
	char *var = bus_suspended ?
		"G_BUS_SUSPENDED=1" : "G_BUS_SUSPENDED=0";
	char *envp[] = {
		"G_SUBSYSTEM=storage",
		"G_ACTION=BUS_STATE_CHANGED",
		var,
		NULL
	};

	if (!the_state.storage_events_enabled) {
		printk(KERN_INFO "gadget_event: storage events is disabled\n");
		return;
	}
	if (the_state.bus_suspended == bus_suspended) {
		printk(KERN_INFO "gadget_event: bus_suspended=%d (no change)\n", bus_suspended);
		return;
	}
	the_state.bus_suspended = bus_suspended;

	printk(KERN_INFO "gadget_event: bus_suspended=%d\n", bus_suspended);
	kobject_uevent_env(&gadget_event_device.dev.kobj, KOBJ_CHANGE, envp);
}
EXPORT_SYMBOL(gadget_event_bus_suspended);

/*
 * media_loaded
 */
static ssize_t
show_media_loaded(struct device *dev, struct device_attribute *attr,
		      char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", the_state.media_loaded);
}

static DEVICE_ATTR(media_loaded, S_IRUGO, show_media_loaded, NULL);

void
gadget_event_media_loaded(int media_loaded)
{
	char *var = media_loaded ?
		"G_MEDIA_LOADED=1" : "G_MEDIA_LOADED=0";
	char *envp[] = {
		"G_SUBSYSTEM=storage",
		"G_ACTION=MEDIA_STATE_CHANGED",
		var,
		NULL
	};

	if (!the_state.storage_events_enabled) {
		printk(KERN_INFO "gadget_event: storage events is disabled\n");
		return;
	}
	if (the_state.media_loaded == media_loaded) {
		printk(KERN_INFO "gadget_event: media_loaded=%d (no change)\n", media_loaded);
		return;
	}
	the_state.media_loaded = media_loaded;

	printk(KERN_INFO "gadget_event: media_loaded=%d\n", media_loaded);
	kobject_uevent_env(&gadget_event_device.dev.kobj, KOBJ_CHANGE, envp);
}
EXPORT_SYMBOL(gadget_event_media_loaded);

/*
 * media_requested
 */
static ssize_t
show_media_requested(struct device *dev, struct device_attribute *attr,
		      char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", the_state.media_requested);
}

static ssize_t
store_media_requested(struct device *dev, struct device_attribute *attr,
		      const char *buf, size_t count)
{
	int i;

	if (sscanf(buf, "%d", &i) != 1)
		return -EINVAL;

	the_state.media_requested = !!i;

	return count;
}

static DEVICE_ATTR(media_requested, S_IRUGO|S_IWUSR, show_media_requested, store_media_requested);

void
gadget_event_media_requested(int media_requested)
{
	char *var = media_requested ?
		"G_MEDIA_REQUESTED=1" : "G_MEDIA_REQUESTED=0";
	char *envp[] = {
		"G_SUBSYSTEM=storage",
		"G_ACTION=MEDIA_REQUEST_STATE_CHANGED",
		var,
		NULL
	};

	if (!the_state.storage_events_enabled) {
		printk(KERN_INFO "gadget_event: storage events is disabled\n");
		return;
	}
	if (the_state.media_requested == media_requested) {
		printk(KERN_INFO "gadget_event: media_requested=%d (no change)\n", media_requested);
		return;
	}
	the_state.media_requested = media_requested;

	printk(KERN_INFO "gadget_event: media_requested=%d\n", media_requested);
	kobject_uevent_env(&gadget_event_device.dev.kobj, KOBJ_CHANGE, envp);
}
EXPORT_SYMBOL(gadget_event_media_requested);

/*
 * current_mA & source
 */
static char*
source_to_string(enum gadget_event_source_type source)
{
	switch (source) {
	case G_EV_SOURCE_NONE:
		return ("none");
	case G_EV_SOURCE_BUS:
		return ("bus");
	case G_EV_SOURCE_CHARGER:
		return ("charger");
#ifdef CONFIG_USB_MULTIPLE_CHARGER_DETECT
	case G_EV_SOURCE_DETECTED:
		return ("detected");
#endif
	default:
		return ("unknown");
	}
}

static ssize_t
show_current_mA(struct device *dev, struct device_attribute *attr,
	     char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", the_state.current_mA);
}

static DEVICE_ATTR(current_mA, S_IRUGO, show_current_mA, NULL);

static ssize_t
show_source(struct device *dev, struct device_attribute *attr,
	    char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n",
			source_to_string(the_state.source));
}

static DEVICE_ATTR(source, S_IRUGO, show_source, NULL);

void
gadget_event_power_state_changed(enum gadget_event_source_type source,
				 int current_mA)
{
	char var_source[32];
	char var_current[32];
	char *envp[] = {
		"G_SUBSYSTEM=power",
		"G_ACTION=POWER_STATE_CHANGED",
		var_source,
		var_current,
		NULL
	};

#ifndef CONFIG_USB_MULTIPLE_CHARGER_DETECT
	if (current_mA == 0) /* if mA is 0, source needs to be none */
		source = G_EV_SOURCE_NONE;
#endif

	sprintf(var_source,
		"G_POWER_SOURCE=%s", source_to_string(source));
	sprintf(var_current, "G_CURRENT_MA=%d", current_mA);

	if (the_state.source == source && the_state.current_mA == current_mA)
	{
		printk(KERN_INFO "gadget_event: source=%s mA=%d (no change)\n", source_to_string(source), current_mA);
		return;
	}
	the_state.source = source;
	the_state.current_mA = current_mA;

	printk(KERN_INFO "gadget_event: source=%s mA=%d\n",
	       source_to_string(source), current_mA);
	kobject_uevent_env(&gadget_event_device.dev.kobj, KOBJ_CHANGE, envp);
}
EXPORT_SYMBOL(gadget_event_power_state_changed);

static int __init init(void)
{
	int ret = 0;

	if ((ret = platform_device_register(&gadget_event_device)))
		goto done;

	if ((ret = device_create_file(&gadget_event_device.dev,
				      &dev_attr_current_mA)))
		goto fail1;
	if ((ret = device_create_file(&gadget_event_device.dev,
				      &dev_attr_source)))
		goto fail2;
	if ((ret = device_create_file(&gadget_event_device.dev,
				      &dev_attr_media_loaded)))
		goto fail3;
	if ((ret = device_create_file(&gadget_event_device.dev,
				      &dev_attr_media_requested)))
		goto fail4;
	if ((ret = device_create_file(&gadget_event_device.dev,
				      &dev_attr_host_connected)))
		goto fail5;
	if ((ret = device_create_file(&gadget_event_device.dev,
				      &dev_attr_bus_suspended)))
		goto fail6;

	if ((ret = platform_driver_register(&gadget_event_driver)))
		goto fail7;

	INIT_DELAYED_WORK(&host_connected_work, host_connected_func);
	INIT_DELAYED_WORK(&host_disconnected_work, host_disconnected_func);

	goto done;

fail7:
	device_remove_file(&gadget_event_device.dev, &dev_attr_bus_suspended);
fail6:
	device_remove_file(&gadget_event_device.dev, &dev_attr_host_connected);
fail5:
	device_remove_file(&gadget_event_device.dev, &dev_attr_media_requested);
fail4:
	device_remove_file(&gadget_event_device.dev, &dev_attr_media_loaded);
fail3:
	device_remove_file(&gadget_event_device.dev, &dev_attr_current_mA);
fail2:
	device_remove_file(&gadget_event_device.dev, &dev_attr_source);
fail1:
	platform_device_unregister(&gadget_event_device);
done:
	return (ret);
}
module_init(init);

static void __exit cleanup(void)
{
	platform_driver_unregister(&gadget_event_driver);
	device_remove_file(&gadget_event_device.dev, &dev_attr_bus_suspended);
	device_remove_file(&gadget_event_device.dev, &dev_attr_host_connected);
	device_remove_file(&gadget_event_device.dev, &dev_attr_media_requested);
	device_remove_file(&gadget_event_device.dev, &dev_attr_media_loaded);
	device_remove_file(&gadget_event_device.dev, &dev_attr_current_mA);
	device_remove_file(&gadget_event_device.dev, &dev_attr_source);
	platform_device_unregister(&gadget_event_device);
}
module_exit(cleanup);

MODULE_AUTHOR("Palm, Inc.");
MODULE_DESCRIPTION("USB gadget event driver");
MODULE_LICENSE("GPL");
