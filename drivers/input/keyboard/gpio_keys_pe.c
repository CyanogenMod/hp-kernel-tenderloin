/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2008 Palm Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/version.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/ctype.h>
#include <linux/gpio_keys_pe.h>

#include <asm/gpio.h>

#ifdef CONFIG_INPUT_EVDEV_TRACK_QUEUE
extern int evdev_get_queue_state(struct input_dev *dev);
#endif

#ifdef  CONFIG_GPIO_KEYS_REBOOT_TRIGGER
RAW_NOTIFIER_HEAD(reboot_key_notifier_list);
EXPORT_SYMBOL(reboot_key_notifier_list);
#endif

#ifdef  CONFIG_GPIO_KEYS_CONSOLE_TRIGGER
RAW_NOTIFIER_HEAD(console_key_notifier_list);
EXPORT_SYMBOL(console_key_notifier_list);
#endif

struct btn_ctxt;

struct gpio_keys_dev {
	struct input_dev *idev;
	struct platform_device  *pdev;
	struct gpio_keys_platform_data *pdata;
	int              nbtns; 
	struct btn_ctxt *pbtns;
#ifdef CONFIG_GPIO_KEYS_TRIGGER
	spinlock_t keytrigger_lock;
#endif
#ifdef  CONFIG_GPIO_KEYS_REBOOT_TRIGGER
	int        reboot_state;
#endif
#ifdef  CONFIG_GPIO_KEYS_CONSOLE_TRIGGER
	int        console_state;
#endif
	uint32_t   wake_source;
	int			suspended;
};

struct btn_ctxt {
	struct gpio_keys_button *btn;  // points to platform data
	struct gpio_keys_dev   *kdev;  // points back to device context
	struct timer_list      timer;  // debounce timer
	int    pending;
	int    state;
	int    prev_state;
	spinlock_t lock;
};


static int gpiokeys_gpio_get_value(struct gpio_keys_button *button)
{
	if(button->gpio_controller_cansleep)
	{
		return gpio_get_value_cansleep(button->gpio);
	}
	else
	{
		return gpio_get_value(button->gpio);
	}
}
		

/******************************************************************************
*
* gpio_key_get_option()
*
* Parse integer from an option string.
* Adapted from lib/cmdline.c
*
* Inputs
*   str         option string
*   pint        integer value parsed from @str
*
* Returns
*   0           no int in string
*   1           int found, possibly more to follow
*  -1           error
*
******************************************************************************/
static int 
gpio_key_get_option(char **str, int *pint)
{
	char *cur = *str;

	if (! cur)
		return 0;

	/* Linux's simple_strtol() does not skip over white space, unlike
	   the libc implementation. */
	for (;; cur++) {
		if (! *cur)
			return 0;

		if (! isspace(*cur))
			break;
	}
	*str = cur;

	*pint = simple_strtol(cur, str, 0);
	if (cur == *str) {
		/* At character that's not a digit, but trailing spaces OK */
		return isspace(*cur) ? 0 : -1;
	}
	return 1;
}

/******************************************************************************
*
* gpio_wake_keys_show()
*
* SysFS interface for displaying /sys/class/input/inputXX/gpio_wake_keys
* 
* Inputs
*   none
*
* Returns
*   none
*
******************************************************************************/
static ssize_t
gpio_wake_keys_show( struct device *dev,
                     struct device_attribute *dev_attr, 
                     char  *buf)
{
	ssize_t count = 0 ;
	int i;
	struct gpio_keys_platform_data *pdata = dev->parent->platform_data;

	if (!buf)
		return 0;

	for (i = 0; i < pdata->nbuttons; i++) {
		if (!pdata->buttons[i].wakeup) 
			continue;
		count += snprintf(buf + count, PAGE_SIZE - count, "%d ", 
		                  pdata->buttons[i].code);
	}

	count += snprintf(buf + count, PAGE_SIZE - count, "\n");
	return count;
}

/******************************************************************************
*
* gpio_wake_keys_store()
*
* SysFS interface for configuring /sys/class/input/inputXX/gpio_wake_keys
* Accept a space delimited list of integers.  Invalid key scan codes are
* discarded.  If there is a parse error, do not reconfigure the wake keys.
*
* Inputs
*   none
*
* Returns
*   none
*
******************************************************************************/
static ssize_t
gpio_wake_keys_store(struct device *dev, 
                     struct device_attribute *dev_attr, 
                     const char *buf, size_t count)
{

	char *s;
	int i, j;
	int key_code;
	int res;

	struct gpio_keys_platform_data *pdata = dev->parent->platform_data;

	for (i = 0; i < pdata->nbuttons; i++)
		pdata->buttons[i].wakeup = 0;
	
	for (i = 0, s = (char *)buf; i < pdata->nbuttons;) {
		res = gpio_key_get_option(&s, &key_code);

		if (res == 0)
			break;

		if (res == -1)
			return -EINVAL; /* Parse error, do nothing */

		if ((key_code > 0)&&(key_code <= KEY_MAX)) {
			for (j = 0; j < pdata->nbuttons; j++)
				if (key_code == pdata->buttons[j].code)	
					pdata->buttons[j].wakeup = 1;
			i++; 
		}
	}

    return count;
}

static DEVICE_ATTR(gpio_wake_keys, S_IRUGO | S_IWUGO, gpio_wake_keys_show, gpio_wake_keys_store);


#ifdef CONFIG_GPIO_KEYS_TRIGGER
static void
gpio_keys_check_keytrigger_event(struct gpio_keys_dev *kdev)
{
	unsigned long flags;
	int i, reboot, console;

	reboot = console = 1;

	spin_lock_irqsave(&kdev->keytrigger_lock, flags);
	for (i = 0; i < kdev->nbtns; i++) {
		struct btn_ctxt *btn = &kdev->pbtns[i];
		struct gpio_keys_button *button = btn->btn;
		
		if( button == NULL )
			continue;
			
#ifdef  CONFIG_GPIO_KEYS_REBOOT_TRIGGER
		if( button->options & OPT_REBOOT_TRIGGER ) {
			if( button->options & OPT_REBOOT_TRIGGER_EDGE ) {
				reboot &=(btn->state != btn->prev_state);
			} else {
				reboot &= btn->state;
			}
		}
#endif
#ifdef  CONFIG_GPIO_KEYS_CONSOLE_TRIGGER
		if( button->options & OPT_CONSOLE_TRIGGER )
			console &= btn->state;
#endif
	}
#ifdef  CONFIG_GPIO_KEYS_REBOOT_TRIGGER
	if( reboot || kdev->reboot_state != reboot ) {
		//printk("%s: reboot event; reboot=%d\n", __FUNCTION__, reboot);

		kdev->reboot_state = reboot;
		raw_notifier_call_chain(&reboot_key_notifier_list, 
		                         reboot, NULL);
	}
#endif
#ifdef  CONFIG_GPIO_KEYS_CONSOLE_TRIGGER
	if( kdev->console_state != console ) {
		kdev->console_state = console;
		raw_notifier_call_chain(&console_key_notifier_list, console, NULL);
	}
#endif
	spin_unlock_irqrestore(&kdev->keytrigger_lock, flags);
	return;
}
#endif

static void
gpio_keys_send_event (struct input_dev *input,
		      struct gpio_keys_button *button,
		      unsigned int type, unsigned int code, int value)
{
       if(button->type == EV_KEY) {  /* button event */
	       printk(KERN_INFO "%s: %s button %s\n",
		       "gpio-keys", button->desc,
		       value ? "pressed" : "released");
       }
       input_event(input, type, code, value);
}

static void
gpio_keys_debounce_timer(unsigned long data)
{
	unsigned long flags;
	struct btn_ctxt *btn = (struct btn_ctxt *) data;
	struct gpio_keys_button *button = btn->btn;

	int state = (gpiokeys_gpio_get_value(button) ? 1 : 0) ^ button->active_low;
	
	spin_lock_irqsave(&btn->lock, flags);
	if (state != btn->state) {
		btn->state = state;
		mod_timer(&btn->timer, jiffies + msecs_to_jiffies(button->debounce));
		spin_unlock_irqrestore(&btn->lock, flags);
	} else {
		unsigned int type = button->type ?: EV_KEY;

		btn->pending = 0;
		spin_unlock_irqrestore(&btn->lock, flags);
		

		/* 
 		 * NOT sending out event
 		 *
		 * State before interrupt matches the SETTLED state after interrupt.
		 * Interpretation option to ignore such condition as noise 
		 */
		if (btn->prev_state == btn->state &&
			button->noise_mode & MODE_NOISE_INTERPRETATION) {
			printk(KERN_INFO"%s: State for gpio: %d did not change. Noise and ignore.\n",
				__func__,button->gpio);
			return;
		}

		/* 
 		 * Sending out event(s) if:
 		 *
 		 * 1)there is a change in state before interrupt and the settled state, 
 		 *   send out a event.
 		 *
 		 * 2)there is NO change in state before interrupt and the settled state. 
 		 *   Interpretation option to handle such condition as missing interrupt, 
 		 *   and send out a pair of events.
 		 */
 		 
		if (btn->prev_state == btn->state &&
			button->noise_mode & MODE_MISSING_INT_INTERPRETATION) {

			gpio_keys_send_event(btn->kdev->idev, button, type, button->code, !(btn->state));
		}

		gpio_keys_send_event(btn->kdev->idev, button, type, button->code, !!(btn->state));
		input_sync (btn->kdev->idev);

#ifdef CONFIG_GPIO_KEYS_TRIGGER
		gpio_keys_check_keytrigger_event ( btn->kdev );
#endif
	}
}

static irqreturn_t 
gpio_keys_isr(int irq, void *dev_id)
{
	int i, gpio;
	unsigned long flags;
	struct platform_device *pdev = dev_id;
	struct gpio_keys_dev *kdev = platform_get_drvdata(pdev);
	unsigned int type;

	for (i = 0; i < kdev->nbtns; i++) {
		struct btn_ctxt *btn = &kdev->pbtns[i];
		struct gpio_keys_button *button = btn->btn;
		
		if( button == NULL )
			continue;
			
		gpio = button->gpio;
		if (irq == gpio_to_irq(gpio)) {
			spin_lock_irqsave(&btn->lock, flags);

			if(button->is_wake_source && 
				button->is_wake_source(button->gpio) && kdev->wake_source)
			{
				/* If we have woken up from a known wake source and this is the first
				   time we have run the ISR, a few things could have occured.
					1. woke up while the button is still pressed
					2. woke up after the button has been released
				*/
				type = button->type ?: EV_KEY;
				btn->state = (gpiokeys_gpio_get_value(button) ? 1 : 0) ^ button->active_low;

				/* Either case we should send the pressed event */
				printk("gpio_keys: forcing GPIO %d to state %d\n",
					button->gpio, !(btn->prev_state));
				gpio_keys_send_event(kdev->idev, button, type, button->code, !(btn->prev_state));

				/* If the current button state is the same state as it was when we suspended
				   then we know we missed the key pressed event and we need to schedule
				   the debounce routine to register the key release event.
				   
				   If the current button state is not the same then we know the key is
				   pressed.  We will skip the debouce routine since the debounce is implied
				   from the delay it took to wake us up in the first place.  We should not
				   schedule the timer since we have already sent the key pressed event
				*/
				if (btn->prev_state == btn->state) {
					if (btn->pending) {
						mod_timer(&btn->timer, jiffies + msecs_to_jiffies(button->debounce));
					} else {
						btn->pending = 1;

						btn->timer.data = (unsigned long) btn;
						btn->timer.function = &gpio_keys_debounce_timer;
						btn->timer.expires = jiffies + msecs_to_jiffies(button->debounce);

						add_timer(&btn->timer);
					}
	
				}

				/* Only allow the wake source special case the first time through the ISR */
				kdev->wake_source = 0;
			}
			else {
				if (btn->pending) {
					mod_timer(&btn->timer, jiffies + msecs_to_jiffies(button->debounce));
				} else {
					btn->pending = 1;
					btn->prev_state = btn->state;
					btn->state = (gpiokeys_gpio_get_value(button) ? 1 : 0) ^ button->active_low;
	
					btn->timer.data = (unsigned long) btn;
					btn->timer.function = &gpio_keys_debounce_timer;
					btn->timer.expires = jiffies + msecs_to_jiffies(button->debounce);
	
					add_timer(&btn->timer);
				}
			}

			spin_unlock_irqrestore(&btn->lock, flags);
		}
	}

	return IRQ_HANDLED;
}


static void
gpio_keys_unregister_button ( struct platform_device  *pdev, int idx)
{
	struct gpio_keys_dev *kdev = platform_get_drvdata(pdev);
	struct gpio_keys_button *button = kdev->pbtns[idx].btn;

	if( button || button->gpio == -1 ) 
		return;

	// free irq
	free_irq ( gpio_to_irq(button->gpio), pdev );

	// free gpio
	gpio_free( button->gpio );

	kdev->pbtns[idx].btn = NULL;
}


static void
gpio_keys_register_button ( struct platform_device *pdev, int idx)
{
	int rc, irq;
	const char *descr;
	struct gpio_keys_dev *kdev = platform_get_drvdata(pdev);
	struct input_dev *input = kdev->idev;
	struct gpio_keys_button *button;
	
	button = kdev->pdata->buttons + idx;
	descr  =  button->desc ? button->desc : "gpio_keys";

	/* request gpio */
	rc = gpio_request ( button->gpio, descr );
	if( rc ) {
		printk(KERN_ERR "%s: failed (%d) to request gpio %d\n",
		       input->name, rc, button->gpio );
		return;
	}

	/* request irq */
	irq = gpio_to_irq( button->gpio );
	rc  = request_any_context_irq( irq, gpio_keys_isr,
	                   IRQF_SAMPLE_RANDOM | IRQ_TYPE_EDGE_BOTH,
	                   descr, pdev );
	if( rc < 0 ) {
		printk(KERN_ERR "%s: unable to claim irq %d; error %d\n",
		       input->name, irq, rc);
		goto err_request_irq;
	}

	if(gpio_cansleep(button->gpio)) {
		button->gpio_controller_cansleep = 1;
	}

	kdev->pbtns[idx].btn  = button;
	kdev->pbtns[idx].kdev = kdev;

	init_timer(&kdev->pbtns[idx].timer);
	spin_lock_init(&kdev->pbtns[idx].lock);

#ifdef CONFIG_KEYBOARD_GPIO_DEBOUNCE_PE
	/* Enable GPIO keys debounce */
	gpio_set_debounce(button->gpio, 1);
	/* Set GPIO keys debounce time */
	gpio_set_debounce_time(button->gpio, button->debounce);
#endif

	input_set_capability(input, button->type, button->code);

	// configure gpio key state during the intialization
	kdev->pbtns[idx].state = 
	            (gpiokeys_gpio_get_value(button) ? 1 : 0) ^ button->active_low;

	gpio_keys_send_event(input, button, button->type, button->code, !!(kdev->pbtns[idx].state));
	input_sync(input);


	return;

err_request_irq:
	gpio_free ( button->gpio );
	button->gpio = -1; // mark it as failed
	return;
}


static void
gpio_keys_free_dev(struct platform_device *pdev)
{
	struct gpio_keys_dev *kdev = platform_get_drvdata(pdev);

	if(!kdev) 
		return;

	platform_set_drvdata(pdev, NULL);

	if( kdev->idev) {
		input_unregister_device(kdev->idev);
		input_free_device(kdev->idev);
		kdev->idev = NULL;
	}

	if( kdev->pbtns ) {
		kfree(kdev->pbtns);
		kdev->pbtns = NULL;
		kdev->nbtns = 0;
	}
	
	kfree(kdev);
	
	return;
}

static struct gpio_keys_dev *
gpio_keys_alloc_dev (struct platform_device *pdev)
{
	struct gpio_keys_dev *kdev = NULL;
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;

	if( pdata == NULL ||  pdata->nbuttons == 0 || pdata->buttons == NULL) 
		return NULL;

	kdev = kzalloc(sizeof(struct gpio_keys_dev), GFP_KERNEL);
	if(!kdev)
		return NULL;

	kdev->pbtns = kzalloc(sizeof(struct btn_ctxt) * pdata->nbuttons, 
	                      GFP_KERNEL);
	if( kdev->pbtns == NULL )
		goto err;
	kdev->nbtns = pdata->nbuttons;

	platform_set_drvdata(pdev, kdev);
	kdev->pdata = pdata;

	return kdev;

err:
	kfree(kdev);
	return NULL;
}
#ifdef CONFIG_PM 

// setup sysfs area

static ssize_t gpio_keys_pm_state_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input_dev = to_input_dev(dev);
	struct gpio_keys_dev *kdev = input_get_drvdata(input_dev);

	if(kdev)
		return snprintf(buf, PAGE_SIZE, "%d\n", kdev->suspended);
	else
		return snprintf(buf, PAGE_SIZE, "%d\n", -2);
}

static int gpio_keys_resume ( struct platform_device *dev );
static int gpio_keys_suspend( struct platform_device *dev, pm_message_t state );

static ssize_t gpio_keys_pm_state_store(struct device *dev, struct device_attribute *attr,
             const char *buf, size_t count)
{
	struct input_dev *input_dev = to_input_dev(dev);
	struct gpio_keys_dev *kdev = input_get_drvdata(input_dev);

	int s = simple_strtol(buf, NULL, 10);
	if(s)
		gpio_keys_suspend(kdev->pdev, (pm_message_t){0});
	else
		gpio_keys_resume(kdev->pdev);

	return count;
}

static DEVICE_ATTR(pm_suspend, S_IRUGO | S_IWUSR, gpio_keys_pm_state_show, gpio_keys_pm_state_store);
#define DEVICE_ATTR_NAME(_name) dev_attr_##_name

#endif

static int __devinit 
gpio_keys_probe(struct platform_device *pdev)
{
	struct gpio_keys_dev *kdev = NULL;
	struct input_dev *input = NULL;
	int i, rc = -ENODEV;

	kdev = gpio_keys_alloc_dev (pdev);
	if( kdev == NULL)
		return -ENODEV;
	
	input = input_allocate_device();
	if (!input)
		goto err_input_alloc_device;

	set_bit(EV_KEY, input->evbit);
	set_bit(EV_SW,  input->evbit);

	input->name = pdev->name;
	input->phys = "gpio-keys/input0";
	input->dev.parent = &pdev->dev;

	input->id.bustype = BUS_HOST;
	input->id.vendor  = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	rc = input_register_device(input);
	if (rc) {
		printk(KERN_ERR "%s: unable to register input device\n", 
		                 pdev->name );
		input_free_device(input);
		goto err_input_alloc_device;
	}
	input_set_drvdata(input, kdev);

	// Attach input device
	kdev->idev = input;
	kdev->pdev = pdev;

	// This device can wakeup
	device_init_wakeup(&input->dev, 1);

	/* Register gpio buttons */
	for (i = 0; i < kdev->pdata->nbuttons; i++) {
		gpio_keys_register_button ( pdev, i );
	}

	kdev->suspended = 0;

	/* Create SYSFS attr for gpio keys */
	rc = device_create_file(&input->dev, &dev_attr_gpio_wake_keys);
	if (rc){
		printk(KERN_ERR "%s: failed to create device attr for gpio wake keys\n", 
		                 pdev->name );
		goto err_device_create_file0;
	}
#ifdef CONFIG_PM 
    if (device_create_file(&input->dev, &DEVICE_ATTR_NAME(pm_suspend)))
	    if (rc){
		printk(KERN_ERR "%s: failed to create device attr for pm\n", 
		                 pdev->name );
		goto err_device_create_file0;
	}
#endif	    


	return 0;

err_device_create_file0:
	for (i = i - 1; i >= 0; i--)
		gpio_keys_unregister_button (pdev, i);

err_input_alloc_device:
	gpio_keys_free_dev(pdev);

	return rc;
}

static int __devexit 
gpio_keys_remove(struct platform_device *pdev)
{
	int i;
	struct gpio_keys_dev *kdev = platform_get_drvdata(pdev);

	device_init_wakeup(&kdev->idev->dev, 0);
	device_remove_file(&kdev->idev->dev, &dev_attr_gpio_wake_keys);

	for (i = 0; i < kdev->pdata->nbuttons; i++) {
		gpio_keys_unregister_button ( pdev, i );
	}

	gpio_keys_free_dev(pdev);

	return 0;
}

/******************************************************************************
 *
 * gpio_keys_suspend()
 * 
 ******************************************************************************/
#ifdef CONFIG_PM

static int gpio_keys_resume (struct platform_device *pdev);
static int gpio_keys_suspend(struct platform_device *pdev, pm_message_t );

static int gpio_keys_suspend(struct platform_device *pdev, pm_message_t pm_state)
{
	int i;
	struct gpio_keys_dev *kdev = platform_get_drvdata(pdev);
	struct gpio_keys_platform_data *pdata = kdev->pdata;

	kdev->suspended = 1;

	if (device_may_wakeup(&kdev->idev->dev)) {
		for (i = 0; i < pdata->nbuttons; i++) {
			struct gpio_keys_button *button = &pdata->buttons[i];
			int irq = gpio_to_irq(button->gpio);
			disable_irq(irq);
			if (button->wakeup) {
				enable_irq_wake(irq);
			}
		}
#ifdef CONFIG_INPUT_EVDEV_TRACK_QUEUE
		{ 
			int rc = evdev_get_queue_state(kdev->idev);
			if( rc ) {
				gpio_keys_resume ( pdev ); // unwind 
				return -EBUSY;
			}
		}
#endif			
	}

	for (i = 0; i < kdev->nbtns; i++) {
		struct btn_ctxt *btn = &kdev->pbtns[i];
		struct gpio_keys_button *button = btn->btn;

		btn->prev_state = (gpiokeys_gpio_get_value(button) ? 1 : 0) ^ button->active_low;
	}

	/* allow special wake_source case in ISR for resuming */
	kdev->wake_source = 1;

	return 0;
}

/******************************************************************************
 *
 * gpio_keys_resume()
 * 
 ******************************************************************************/
static int gpio_keys_resume ( struct platform_device *pdev )
{
	int i;
	struct gpio_keys_dev *kdev = platform_get_drvdata(pdev);
	struct gpio_keys_platform_data *pdata = kdev->pdata;

	if (device_may_wakeup(&kdev->idev->dev)) {
		for (i = 0; i < pdata->nbuttons; i++) {
			struct gpio_keys_button *button = &pdata->buttons[i];
			int irq = gpio_to_irq(button->gpio);
			if (button->wakeup) {
				disable_irq_wake(irq);
			}
			enable_irq(irq);
		}
	}

	kdev->suspended = 0;

	return 0;
}
#else
#define gpio_keys_suspend	NULL
#define gpio_keys_resume	NULL
#endif  /* CONFIG_PM */

struct platform_driver gpio_keys_device_driver = {
	.probe		= gpio_keys_probe,
	.remove		= __devexit_p(gpio_keys_remove),
	.suspend	= gpio_keys_suspend,
	.resume		= gpio_keys_resume,
	.driver		= {
		.name	= "gpio-keys",
	},
};

static int __init gpio_keys_init(void)
{
	return platform_driver_register(&gpio_keys_device_driver);
}

static void __exit gpio_keys_exit(void)
{
	platform_driver_unregister(&gpio_keys_device_driver);
}

module_init(gpio_keys_init);
module_exit(gpio_keys_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Keyboard driver for GPIO Keys");
