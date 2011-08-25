/*
 * drivers/user-pins.c
 *
 * Copyright (C) 2008 Palm, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/user-pins.h>
#ifdef CONFIG_PINMUX
#include <linux/pinmux.h>
#endif

#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#undef  MODDEBUG
//#define MODDEBUG  1

#ifdef  MODDEBUG
#define PDBG(args...)   printk(args)
#else
#define PDBG(args...)   
#endif

#define DRIVER_NAME    "user-pins"

enum event {
	USER_PIN_EVENT_IRQ,
	USER_PIN_EVENT_IRQ_READ,
};

struct user_pins_log_event {
	ktime_t timestamp;
	enum event event;
	int gpio;
};

#if defined(CONFIG_DEBUG_FS)
#define NR_LOG_ENTRIES 512

static struct user_pins_log_event user_pins_log[NR_LOG_ENTRIES];
static int user_pins_log_idx;

static DEFINE_SPINLOCK(debug_lock);

static char debug_buffer[PAGE_SIZE];

static void user_pins_log_event(int gpio, enum event event)
{
	unsigned long flags;
	spin_lock_irqsave(&debug_lock, flags);
	user_pins_log[user_pins_log_idx].timestamp = ktime_get();
	user_pins_log[user_pins_log_idx].event = event;
	user_pins_log[user_pins_log_idx].gpio = gpio;
	user_pins_log_idx += 1;
	if (user_pins_log_idx == NR_LOG_ENTRIES) {
		user_pins_log_idx = 0;
	}
	spin_unlock_irqrestore(&debug_lock, flags);
}

static int debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t debug_show_log(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	char *buf = debug_buffer;
	int i, n = 0;
	const char *event;
	unsigned long flags;

	spin_lock_irqsave(&debug_lock, flags);

	for (i = 0; i < user_pins_log_idx; i++) {
		switch (user_pins_log[i].event) {
			case USER_PIN_EVENT_IRQ:
				event = "IRQ";
				break;
			case USER_PIN_EVENT_IRQ_READ:
				event = "IRQ_READ";
				break;
			default:
				event = "<null>";
				break;
		}
		n += scnprintf(buf + n, PAGE_SIZE - n,
				"%010llu: %-8d %s\n",
				ktime_to_ns(user_pins_log[i].timestamp),
				user_pins_log[i].gpio, event);
	}

	user_pins_log_idx = 0;

	spin_unlock_irqrestore(&debug_lock, flags);

	return simple_read_from_buffer(ubuf, count, ppos, buf, n);
}

static const struct file_operations debug_log_fops = {
	.open = debug_open,
	.read = debug_show_log,
};

static void user_pins_debug_init(void)
{
	struct dentry *dent;
	dent = debugfs_create_dir(DRIVER_NAME, 0);
	if (IS_ERR(dent)) {
		return;
	}
	debugfs_create_file("log", 0444, dent, NULL, &debug_log_fops);
	user_pins_log_idx = 0;
}
#else
static void user_pins_log_event(int gpio, enum event event) { }
static void user_pins_debug_init(void) { }
#endif

static struct kobject *user_hw_kobj;
static struct kobject *pins_kobj;

DEFINE_SPINLOCK(pins_lock);

static int __init user_hw_init(void)
{
	user_hw_kobj = kobject_create_and_add("user_hw", NULL);
	if (user_hw_kobj == NULL) {
		return -ENOMEM;
	}
	return 0;
}

arch_initcall(user_hw_init);

struct pin_attribute {
	struct attribute attr;
	ssize_t (*show) (struct pin_attribute *attr, char *buf);
	ssize_t (*store)(struct pin_attribute *attr, const char *buf, size_t count);
};

#define to_pin_attr(_attr) container_of(_attr, struct pin_attribute, attr)

static ssize_t
pin_attr_show(struct kobject *kobj, struct attribute *attr, char *buf)
{
	struct pin_attribute * pin_attr = to_pin_attr(attr);
	if (pin_attr->show) {
		return pin_attr->show(pin_attr, buf);
	} else {
		return -EIO; 
	}
	return 0;
}

static ssize_t
pin_attr_store(struct kobject *kobj, struct attribute *attr, const char *buf,
		size_t count)
{
	struct pin_attribute * pin_attr = to_pin_attr(attr);
	if (pin_attr->store) {
		return pin_attr->store (pin_attr, buf, count);
	} else {
		return -EIO;
	}
}

static struct sysfs_ops pin_sysfs_ops = {
	.show	= pin_attr_show,
	.store	= pin_attr_store,
};

static struct kobj_type ktype_pin = {
	.release	= NULL,
	.sysfs_ops	= &pin_sysfs_ops,
};

struct gpio_pin {
	int     gpio;
	int     options;
	int     direction;
	int     act_level;
	int     def_level;
	int		active_power_collapse;
	irqreturn_t (*irq_handler)(int irq, void *data);
	int (*pinmux)(int gpio, int mode);
	atomic_t irq_count;
	int irq_config;	
	int irq_masked;
	int irq_requested;
	const char * name;
	int	irq_handle_mode;
	atomic_t irqs_during_suspend;
	struct sysfs_dirent *sd;
	struct  pin_attribute attr_gpio;
	struct  pin_attribute attr_level;
	struct  pin_attribute attr_active;
	struct  pin_attribute attr_direction;
	struct  pin_attribute attr_irq;
	struct  pin_attribute attr_irqconfig;
	struct  pin_attribute attr_irqrequest;
	struct  pin_attribute attr_irqmask;
	struct	pin_attribute attr_irq_handle_mode;
	struct  pin_attribute attr_active_power_collapse;
	struct  attribute *attr_ptr_arr[11];
};

struct gpio_pin_set_item {
	struct attribute_group  attr_grp;
	struct gpio_pin         pin;
};

struct gpio_pin_set {
	const  char *set_name;
	struct kobject   kobj;
	int          num_pins;
	struct gpio_pin_set_item pins[];
};

struct gpio_pin_dev_ctxt {
	int    num_sets;
	struct gpio_pin_set *sets[];	
};

/*
 *  Show irq handle mode
 *
 *  If AUTO, irq will be handled by irq_handler 
 */
static int 
pin_show_irq_mode ( struct pin_attribute *attr, char  *buf)
{
	struct gpio_pin *pin = container_of(attr, struct gpio_pin, attr_irq_handle_mode );
	return sprintf(buf, "%d\n", pin->irq_handle_mode );
}

/*
 *  Set irq handle mode for specified pin
 *
 */
static ssize_t 
pin_store_irq_mode( struct pin_attribute *attr, const char * buf, size_t count)
{
	int irq_handle_mode;
	unsigned long flags;
	struct gpio_pin *pin = container_of(attr, struct gpio_pin, attr_irq_handle_mode );
	sscanf(buf, "%d", &irq_handle_mode);

	spin_lock_irqsave(&pins_lock, flags);

	// reset irq count to detect user suspend and kernel suspend
	pin->irq_handle_mode = irq_handle_mode;
	if(pin->irq_handle_mode == IRQ_HANDLE_OFF)
		atomic_set(&pin->irqs_during_suspend, 0);

	spin_unlock_irqrestore(&pins_lock, flags);

	printk(KERN_INFO"USERPIN: setting irq handle mode of pin gpio %d to %d\n",
		pin->gpio, irq_handle_mode);

	return count;
}

static irqreturn_t user_pins_irq(int irq, void *data)
{
	unsigned long flags;
	struct gpio_pin *pin = (struct gpio_pin *)data;

	user_pins_log_event(pin->gpio, USER_PIN_EVENT_IRQ);

	atomic_inc(&pin->irq_count);

	spin_lock_irqsave(&pins_lock, flags);

	if (pin->irq_handle_mode & IRQ_HANDLE_OFF)
		atomic_inc(&pin->irqs_during_suspend);

	spin_unlock_irqrestore(&pins_lock, flags);

	if (pin->sd != NULL) {
		sysfs_notify_dirent(pin->sd);
	}

	if (pin->irq_handler != NULL) {
		pin->irq_handler(irq, NULL);
	}

	return IRQ_HANDLED;
}

static int user_pins_irq_request(struct gpio_pin *pin)
{
	int rc = 0;

	printk("user-pins: configuring irq for gpio %d\n", pin->gpio);

	rc = request_irq(gpio_to_irq(pin->gpio), user_pins_irq,
			pin->irq_config, "userpins", pin);
	if (rc) {
		printk("user-pins: failed to request irq\n");
	}

	return rc;
}

/*
 *  Show gpio direction 
 */
static int pin_show_direction(struct pin_attribute *attr, char *buf)
{
	struct gpio_pin *pin = container_of(attr, struct gpio_pin, attr_direction);
	return sprintf(buf, "%d\n", pin->direction);
}

/*
 *  Show  active level for specified pin
 */
static int pin_show_active(struct pin_attribute *attr, char *buf)
{
	struct gpio_pin *pin = container_of(attr, struct gpio_pin, attr_active);
	return sprintf(buf, "%d\n", pin->act_level);
}

static int pin_show_active_power_collapse(struct pin_attribute *attr, char *buf)
{
	struct gpio_pin *pin = container_of(attr, struct gpio_pin, attr_active_power_collapse);
	return sprintf(buf, "%d\n", pin->active_power_collapse);
}

/*
 *  Show gpio number
 */
static int pin_show_gpio(struct pin_attribute *attr, char *buf)
{
	struct gpio_pin *pin = container_of(attr, struct gpio_pin, attr_gpio);
	return sprintf(buf, "%d\n", pin->gpio);
}

/*
 *  Show current for specified pin
 */
static int pin_show_level(struct pin_attribute *attr, char *buf)
{
	int    val;
	struct gpio_pin *pin = container_of(attr, struct gpio_pin, attr_level);

	val = gpio_get_value(pin->gpio);
	PDBG("get: gpio[%d] = %d\n", pin->gpio, val);
	if (val) {
		return sprintf(buf, "1\n" );
	} else {
		return sprintf(buf, "0\n" );
	}
}

/*
 *  Set level for specified pin
 */
static ssize_t
pin_store_level(struct pin_attribute *attr, const char *buf, size_t count)
{
	int i = 0, len,  val = -1;
	struct gpio_pin *pin = container_of(attr, struct gpio_pin, attr_level);

	if (pin->options & PIN_READ_ONLY) {
		return count;  // just ignore writes 
	}

	/* skip leading white spaces */
	while (i < count && isspace(buf[i])) {
		i++;
	}

	len = count - i;
	if (len >= 1 && strncmp(buf+i, "1", 1) == 0) {
		val = 1;
		goto set;
	}

	if (len >= 1 && strncmp(buf+i, "0", 1) == 0) {
		val = 0;
		goto set;
	}

	if (len >= 4 && strncmp(buf+i, "high", 4) == 0) {
		val = 1;
		goto set;
	}

	if (len >= 3 && strncmp(buf+i, "low", 3) == 0) {
		val = 0;
		goto set;
	}

	return count;

set:
	PDBG("set: gpio[%d] = %d\n", pin->gpio, val);
	gpio_set_value(pin->gpio, val);
	return count;
}


static ssize_t
pin_store_active_power_collapse(struct pin_attribute *attr, const char *buf, size_t count)
{
	int i = 0, len,  val = -1;
	struct gpio_pin *pin = container_of(attr, struct gpio_pin, attr_active_power_collapse);

	if (pin->options & PIN_READ_ONLY) {
		return count;  // just ignore writes 
	}

	/* skip leading white spaces */
	while (i < count && isspace(buf[i])) {
		i++;
	}

	len = count - i;
	if (len >= 1 && strncmp(buf+i, "1", 1) == 0) {
		val = 1;
	} else if (len >= 1 && strncmp(buf+i, "0", 1) == 0) {
		val = 0;
	} else { /* invalid input */
		goto end;
	}

	PDBG("set: active_power_collapse[%d] = %d\n", pin->gpio, val);

#ifdef CONFIG_PINMUX
	// This is an old legacy mechanism for muxing pins, it's not that clean
	// and creates dependency on pinmux driver that is not really
	// used on some systems.
	pinmux_set_power_collapse(pin->name, val);
#endif

	pin->active_power_collapse = val;

end:
	return count;

}

static int pin_show_irq(struct pin_attribute *attr, char *buf)
{
	struct gpio_pin *pin = container_of(attr, struct gpio_pin, attr_irq);
	int count = atomic_xchg(&pin->irq_count, 0);
	user_pins_log_event(pin->gpio, USER_PIN_EVENT_IRQ_READ);
	return sprintf(buf, "%d\n", count);
}

static int pin_show_irqconfig(struct pin_attribute *attr, char *buf)
{
	struct gpio_pin *pin = container_of(attr, struct gpio_pin, attr_irqconfig);
	return sprintf(buf, "%d\n", pin->irq_config);
}

static ssize_t
pin_store_irqconfig(struct pin_attribute *attr, const char *buf, size_t count)
{
	struct gpio_pin *pin = container_of(attr, struct gpio_pin, attr_irqconfig);
	unsigned long flags;
	int config;

	config = simple_strtoul(buf, NULL, 10) & IRQF_TRIGGER_MASK;

	spin_lock_irqsave(&pins_lock, flags);
	pin->irq_config = config;
	spin_unlock_irqrestore(&pins_lock, flags);

	return count;
}

static int pin_show_irqrequest(struct pin_attribute *attr, char *buf)
{
	struct gpio_pin *pin = container_of(attr, struct gpio_pin, attr_irqrequest);
	return sprintf(buf, "%d\n", !!pin->irq_requested);
}

static ssize_t
pin_store_irqrequest(struct pin_attribute *attr, const char *buf, size_t count)
{
	struct gpio_pin *pin = container_of(attr, struct gpio_pin, attr_irqrequest);
	unsigned long flags;
	int request;
	int rc = 0;

	if ((count > 0) && (buf[0] == '1')) {
		request = 1;
	} else {
		request = 0;
	}

	if (request != pin->irq_requested) {
		if (request) {
			rc = user_pins_irq_request(pin);
			if (rc) {
				goto fail;
			}
		} else {
			free_irq(gpio_to_irq(pin->gpio), pin);
		}

		spin_lock_irqsave(&pins_lock, flags);
		pin->irq_requested = request;
		pin->irq_masked = 0;
		spin_unlock_irqrestore(&pins_lock, flags);
	}
	return count;
fail:
	return rc;
}

static int pin_show_irqmask(struct pin_attribute *attr, char *buf)
{
	struct gpio_pin *pin = container_of(attr, struct gpio_pin, attr_irqmask);
	return sprintf(buf, "%d\n", !!pin->irq_masked);
}

static ssize_t
pin_store_irqmask(struct pin_attribute *attr, const char *buf, size_t count)
{
	struct gpio_pin *pin = container_of(attr, struct gpio_pin, attr_irqmask);
	unsigned long flags;
	int mask;

	if ((count > 0) && (buf[0] == '1')) {
		mask = 1;
	} else {
		mask = 0;
	}

	spin_lock_irqsave(&pins_lock, flags);
	if (mask != pin->irq_masked) {
		if (mask) {
			disable_irq(gpio_to_irq(pin->gpio));
		} else {
			enable_irq(gpio_to_irq(pin->gpio));
		}
		pin->irq_masked = mask;
	}
	spin_unlock_irqrestore(&pins_lock, flags);

	return count;
}

static void
pin_set_item_init(struct gpio_pin_set_item *psi, struct user_pin *up)
{
	psi->pin.name      = up->name;
	psi->pin.gpio      = up->gpio;
	psi->pin.options   = up->options;
	psi->pin.direction = up->direction;
	psi->pin.act_level = up->act_level;
	psi->pin.def_level = up->def_level;
	psi->pin.irq_handler = up->irq_handler;
	psi->pin.pinmux = up->pinmux;
	psi->pin.irq_config = up->irq_config;
	psi->pin.irq_handle_mode = up->irq_handle_mode;
	atomic_set(&psi->pin.irqs_during_suspend, 0);
	atomic_set(&psi->pin.irq_count, 0);
	psi->pin.irq_requested = 0;
	psi->pin.irq_masked = 0;

	// gpio attr
	psi->pin.attr_gpio.attr.name  = "gpio";
	psi->pin.attr_gpio.attr.mode  =  0444;
	psi->pin.attr_gpio.attr.owner =  THIS_MODULE;
	psi->pin.attr_gpio.show       =  pin_show_gpio;

	// level attr
	psi->pin.attr_level.attr.name  = "level";
	psi->pin.attr_level.attr.mode  =  0644;
	psi->pin.attr_level.attr.owner =  THIS_MODULE;
	psi->pin.attr_level.show       =  pin_show_level;
	psi->pin.attr_level.store      =  pin_store_level;

	// active attr
	psi->pin.attr_active.attr.name = "active";
	psi->pin.attr_active.attr.mode =  0444;
	psi->pin.attr_active.attr.owner=  THIS_MODULE;
	psi->pin.attr_active.show = pin_show_active;

	// direction
	psi->pin.attr_direction.attr.name  = "direction";
	psi->pin.attr_direction.attr.mode  =  0444;
	psi->pin.attr_direction.attr.owner =  THIS_MODULE;
	psi->pin.attr_direction.show       = pin_show_direction;

	psi->pin.attr_active_power_collapse.attr.name = "active_power_collapse";
	psi->pin.attr_active_power_collapse.attr.mode  =  0644;
	psi->pin.attr_active_power_collapse.attr.owner =  THIS_MODULE;
	psi->pin.attr_active_power_collapse.show       =  pin_show_active_power_collapse;
	psi->pin.attr_active_power_collapse.store      =  pin_store_active_power_collapse;

	if (psi->pin.options & PIN_IRQ) {
		// irq
		psi->pin.attr_irq.attr.name  = "irq";
		psi->pin.attr_irq.attr.mode = 0444;
		psi->pin.attr_irq.attr.owner = THIS_MODULE;
		psi->pin.attr_irq.show = pin_show_irq;

		// irqconfig
		psi->pin.attr_irqconfig.attr.name = "irqconfig";
		psi->pin.attr_irqconfig.attr.mode = 0666;
		psi->pin.attr_irqconfig.attr.owner = THIS_MODULE;
		psi->pin.attr_irqconfig.show = pin_show_irqconfig;
		psi->pin.attr_irqconfig.store = pin_store_irqconfig;

		// irqrequest
		psi->pin.attr_irqrequest.attr.name = "irqrequest";
		psi->pin.attr_irqrequest.attr.mode = 0666;
		psi->pin.attr_irqrequest.attr.owner = THIS_MODULE;
		psi->pin.attr_irqrequest.show = pin_show_irqrequest;
		psi->pin.attr_irqrequest.store = pin_store_irqrequest;

		// irqmask
		psi->pin.attr_irqmask.attr.name = "irqmask";
		psi->pin.attr_irqmask.attr.mode = 0666;
		psi->pin.attr_irqmask.attr.owner = THIS_MODULE;
		psi->pin.attr_irqmask.show = pin_show_irqmask;
		psi->pin.attr_irqmask.store = pin_store_irqmask;

		// irq handle mode 
		psi->pin.attr_irq_handle_mode.attr.name  = "irq_handle_mode";
		psi->pin.attr_irq_handle_mode.attr.mode  =  0644;
		psi->pin.attr_irq_handle_mode.attr.owner =  THIS_MODULE;
		psi->pin.attr_irq_handle_mode.show       =  pin_show_irq_mode;
		psi->pin.attr_irq_handle_mode.store      =  pin_store_irq_mode;
	}

	// setup attr pointer array 
	{
		int i = 0;
		psi->pin.attr_ptr_arr[i++] = &psi->pin.attr_gpio.attr;
		psi->pin.attr_ptr_arr[i++] = &psi->pin.attr_level.attr;
		psi->pin.attr_ptr_arr[i++] = &psi->pin.attr_active.attr;
		psi->pin.attr_ptr_arr[i++] = &psi->pin.attr_direction.attr;
		psi->pin.attr_ptr_arr[i++] = &psi->pin.attr_active_power_collapse.attr;
		if (psi->pin.options & PIN_IRQ) {
			psi->pin.attr_ptr_arr[i++] = &psi->pin.attr_irq.attr;
			psi->pin.attr_ptr_arr[i++] = &psi->pin.attr_irqconfig.attr;
			psi->pin.attr_ptr_arr[i++] = &psi->pin.attr_irqrequest.attr;
			psi->pin.attr_ptr_arr[i++] = &psi->pin.attr_irqmask.attr;
			psi->pin.attr_ptr_arr[i++] = &psi->pin.attr_irq_handle_mode.attr;
		}
		psi->pin.attr_ptr_arr[i++] = NULL;

		/* if this is triggered, then we need a larger attr_ptr_arr array */
		BUG_ON(i > ARRAY_SIZE(psi->pin.attr_ptr_arr));
	}

	// setup  attribute group
	psi->attr_grp.name  = psi->pin.name;
	psi->attr_grp.attrs = psi->pin.attr_ptr_arr;

	return;
}

/*
 *
 */
static struct gpio_pin_set * pin_set_alloc(struct user_pin_set *ups)
{
	int i;
	struct gpio_pin_set *gps = NULL;

	gps = kzalloc(sizeof(struct gpio_pin_set) +
			ups->num_pins * sizeof(struct gpio_pin_set_item), GFP_KERNEL);
	if (gps == NULL) {
		return NULL;
	}

	gps->num_pins  = ups->num_pins;
	gps->set_name  = ups->set_name;

	for (i = 0; i < gps->num_pins; i++) {
		pin_set_item_init(&gps->pins[i], &ups->pins[i]);
	}
 
	return gps;
}

/*
 *   Registers specified pin set
 */
static int pin_set_register(struct gpio_pin_set *s)
{   
	int rc, i;
	struct sysfs_dirent *grp_sd;

	if (s == NULL) {
		return -EINVAL;
	}

	rc = kobject_init_and_add(&s->kobj, &ktype_pin, pins_kobj, s->set_name);
	if (rc) {
		printk (KERN_ERR "Failed to register kobject (%s)\n", s->set_name);
		return -ENODEV;
	}	

	/* for all pins */
	for (i = 0; i < s->num_pins; i++) {
		rc = gpio_request(s->pins[i].pin.gpio, "gpio");
		if (rc) {
			printk(KERN_ERR "Failed to request gpio (%d)\n",
					s->pins[i].pin.gpio);
			continue;
		}

		if (s->pins[i].pin.direction != -1) { // direction is set
			if (s->pins[i].pin.direction == 0) { // an output
				/* A setting of def_level == -1 means that we
				 * keep the current level of the GPIO.
				 * Otherwise we set def_level.
				 */
				int level = (-1 == s->pins[i].pin.def_level) ?
						gpio_get_value(s->pins[i].pin.gpio) :
						s->pins[i].pin.def_level;

				gpio_direction_output(s->pins[i].pin.gpio, level);
			} else { // an input
				gpio_direction_input(s->pins[i].pin.gpio);
			}
		}

		// create attribute group
		rc = sysfs_create_group(&s->kobj, &s->pins[i].attr_grp);
		if (rc) {
			printk(KERN_ERR "Failed to create sysfs attr group (%s)\n",
					s->pins[i].pin.name );
		}

		grp_sd = sysfs_get_dirent(s->kobj.sd, NULL, s->pins[i].attr_grp.name);
		if (grp_sd == NULL) {
			printk(KERN_ERR "user-pins: failed to get sd for %s\n",
					s->pins[i].attr_grp.name);
		} else {
			s->pins[i].pin.sd = sysfs_get_dirent(grp_sd, NULL, "irq");
			if ((s->pins[i].pin.sd == NULL) &&
					(s->pins[i].pin.options & PIN_IRQ)) {
				printk(KERN_ERR "user-pins: failed to get sd for %s/irq\n",
						s->pins[i].attr_grp.name);
			}
		}
	}

	return 0;
}

static void pin_set_unregister(struct gpio_pin_set *s)
{
	int i;

	if (s == NULL) {
		return;
	}

	/* for all pins */
	for (i = 0; s->num_pins; i++) {
		if ((s->pins[i].pin.options & PIN_IRQ) &&
				(s->pins[i].pin.irq_requested != 0)) {
			free_irq(gpio_to_irq(s->pins[i].pin.gpio), &s->pins[i].pin);
		}
	
		sysfs_remove_group(&s->kobj, &s->pins[i].attr_grp);
		gpio_free(s->pins[i].pin.gpio);
	}
	kobject_del(&s->kobj);
	kfree(s);
}

static int user_pins_probe(struct platform_device *pdev)
{
	int i, rc;
	struct user_pins_platform_data *pdata;
	struct gpio_pin_dev_ctxt *dev_ctxt;
	
	pdata = pdev->dev.platform_data;
	if( pdata == NULL ) {
		return -ENODEV; 
	}

	dev_ctxt = kzalloc(sizeof (struct gpio_pin_dev_ctxt) +
			pdata->num_sets * sizeof(struct gpio_pin_set *), GFP_KERNEL );
	if (dev_ctxt == NULL) {
		return -ENOMEM; 
	}
	dev_ctxt->num_sets = pdata->num_sets;

	for (i = 0; i < dev_ctxt->num_sets; i++) {
		dev_ctxt->sets[i] = pin_set_alloc (pdata->sets + i);
		if (dev_ctxt->sets[i] == NULL) {
			printk(KERN_ERR "Failed to init pin set '%s'\n",
					pdata->sets[i].set_name );
			continue;
		}
		rc = pin_set_register(dev_ctxt->sets[i]);
		if (rc) {
			printk(KERN_ERR "Failed to register pin set '%s'\n",
					pdata->sets[i].set_name );
		}
	}
	
	dev_set_drvdata(&pdev->dev, dev_ctxt);

	return 0;
}

/*
 *
 */
static int user_pins_remove(struct platform_device *pdev)
{
	int i;
	struct gpio_pin_dev_ctxt *dev_ctxt;

	dev_ctxt = dev_get_drvdata(&pdev->dev);
	if (dev_ctxt == NULL) {
		return 0;
	}

	for (i = 0; i < dev_ctxt->num_sets; i++) {
		pin_set_unregister(dev_ctxt->sets[i]);
	}
	
	dev_set_drvdata(&pdev->dev, NULL);
	kfree(dev_ctxt);

	return 0;
}

#ifdef CONFIG_PM

static int user_pins_suspend(struct platform_device *pdev, pm_message_t state)
{
	int i, j;
	struct gpio_pin_dev_ctxt *dev_ctxt;
	struct gpio_pin_set      *pset;
	struct gpio_pin		 *pin;
	unsigned long flags;

	dev_ctxt = platform_get_drvdata(pdev);
	if (dev_ctxt == NULL) {
		return 0;
	}

	spin_lock_irqsave ( &pins_lock, flags );

	/*
 	 * The first loop is to check for any pending interrupts from
 	 * the time starting IRQ_HANDLE_OFF is set (from user space).
 	 * If so, fail the suspend, and let the system to go back up
 	 * to userspace.
 	 */

	for( i = 0; i < dev_ctxt->num_sets; i++ ) {
		pset = dev_ctxt->sets[i];
		for( j = 0; j < pset->num_pins; j++ ) {
			pin = &(pset->pins[j].pin);

			if ((pin->options & PIN_IRQ) &&
				(pin->irq_handle_mode & IRQ_HANDLE_OFF) &&
				(atomic_read(&pin->irqs_during_suspend) != 0)) {

				atomic_set(&pin->irqs_during_suspend, 0);
				spin_unlock_irqrestore ( &pins_lock, flags );

				printk(KERN_INFO"%s: not suspending due to pending irqs for gpio %d\n",
					__func__, pin->gpio);

				return -EBUSY;
			}
		}
	}

	for (i = 0; i < dev_ctxt->num_sets; i++) {
		pset = dev_ctxt->sets[i];
		for (j = 0; j < pset->num_pins; j++) {
			pin = &(pset->pins[j].pin);

			if (pin->options & PIN_WAKEUP_SOURCE) {
				int irq = gpio_to_irq(pin->gpio);

				if ((pin->options & PIN_IRQ) &&
						pin->irq_requested &&
						!pin->irq_masked) {
					disable_irq(gpio_to_irq(pin->gpio));
				}
				enable_irq_wake(irq);
			}

			// If machine installed pinmux hook for the pin, call it
			// to mux it into suspended mode. Exception is made for
			// pins that are specifically configured to stay active
			// even during suspend periods.
			if (pin->pinmux && !pin->active_power_collapse)
				pin->pinmux(pin->gpio, PIN_MODE_SUSPENDED);
		}
	}

	spin_unlock_irqrestore ( &pins_lock, flags );


	return 0;
}

static int user_pins_resume(struct platform_device *pdev)
{
	int i, j;
	unsigned long flags;
	struct gpio_pin_dev_ctxt *dev_ctxt;
	struct gpio_pin_set      *pset;
	struct gpio_pin		 *pin;

	dev_ctxt = platform_get_drvdata(pdev);
	if (dev_ctxt == NULL) {
		return 0;
	}

	spin_lock_irqsave(&pins_lock, flags);

	for (i = 0; i < dev_ctxt->num_sets; i++) {
		pset = dev_ctxt->sets[i];
		for (j = 0; j < pset->num_pins; j++) {
			pin = &(pset->pins[j].pin);

			// If machine installed pinmux hook for the pin, call it
			// to mux it into active mode. If the pin is configured
			// to be active during suspend periods we assume we don't
			// need to remux it into active state again.
			if (pin->pinmux && !pin->active_power_collapse)
				pin->pinmux(pin->gpio, PIN_MODE_ACTIVE);

			if (pin->options & PIN_WAKEUP_SOURCE) {
				int irq = gpio_to_irq(pin->gpio);
				disable_irq_wake(irq);
				if ((pin->options & PIN_IRQ) &&
						pin->irq_requested &&
						!pin->irq_masked) {
					enable_irq(gpio_to_irq(pin->gpio));
				}
			}
			atomic_set(&pin->irqs_during_suspend, 0);
		}
	}

	spin_unlock_irqrestore(&pins_lock, flags);

	return 0;
}

#else
#define user_pins_suspend  NULL
#define user_pins_resume   NULL
#endif

static struct platform_driver user_pins_driver = {
	.driver		= {
		.name	= DRIVER_NAME,
	},
	.probe		= user_pins_probe,
	.remove		= __devexit_p(user_pins_remove),
	.suspend	= user_pins_suspend,
	.resume		= user_pins_resume,
};

static int __init user_pins_init(void)
{
	int rc;

	if (user_hw_kobj == NULL) {
		return -ENOMEM;
	}
	
	pins_kobj = kobject_create_and_add("pins", user_hw_kobj);
	if (pins_kobj == NULL) {
		return -ENOMEM;
	}
	
	/* register pins platform device */
	rc = platform_driver_register(&user_pins_driver);
	if (rc) {
		kobject_del(pins_kobj);
	}

	user_pins_debug_init();
	
	return rc; 
}

static void __exit   
user_pins_exit(void)
{
	kobject_del(pins_kobj);
}

module_init(user_pins_init);
module_exit(user_pins_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
