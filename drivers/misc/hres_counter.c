 /* 
 * linux/drivers/misc/hres_counter.c
 * 
 * Copyright (C) 2008 Palm, Inc. 
 * 
 * This program is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public License. 
 */ 
 
#include <linux/module.h> 
#include <linux/init.h> 
#include <linux/types.h> 
#include <linux/kernel.h> 
#include <linux/errno.h> 
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/hres_counter.h>
#include <linux/sysrq.h>
#include <linux/platform_device.h>

#include <asm/io.h>

#define NUM_CHANNELS    32
#define NUM_EVENTS     512

struct count_ch {
	u32 count;
	u32 total;
	u32 tstamp;
	u32 active;
};

struct hres_event {
	u32   tstamp;
	char* type;
	u32   arg1;
	u32   arg2;
};

struct dev_ctxt {
	void *timer;
	int suspended;
	struct platform_device *pdev;
	struct count_ch ch[NUM_CHANNELS];

	int (*init_hres_timer)(void **);
	int (*release_hres_timer)(void *);
	int (*suspend_hres_timer)(void *);
	int (*resume_hres_timer)(void *);

	u32 (*read_hres_timer)(void *);
	u32 (*convert_hres_timer)(u32);
};

static struct dev_ctxt *gdev = NULL;

static int    evlog_on  = 1;
static int    evlog_cnt = 0;
static int    evlog_num = 0;
static struct hres_event evlog[NUM_EVENTS];


/*
 *
 */
static inline u32
hres_read_tick ( void ) {
	if( unlikely(gdev == NULL))
		return 0;

	return gdev->read_hres_timer( gdev->timer );
}

/*
 *   Current value of high res counter
 */
u32 
hres_get_counter ( void )
{
	return hres_read_tick();
}
EXPORT_SYMBOL(hres_get_counter);

/*
 *   Return the delta in useconds from start to end
 */
u32
hres_get_delta_usec ( u32 start, u32 end )
{
	return gdev->convert_hres_timer(end - start);
}

/*
 *   Reset channel
 */
void 
hres_ch_reset ( uint ch ) 
{
	if( unlikely(gdev == NULL))
		return;

	if( unlikely(ch >= NUM_CHANNELS))
		return;
	
	memset ( &gdev->ch[ch], 0, sizeof(struct count_ch));
}
EXPORT_SYMBOL(hres_ch_reset);

/*
 *   Increment Event count
 */
void 
hres_event_cnt ( uint ch )
{
	unsigned long flags;
	
	if( unlikely(gdev == NULL))
		return;
		
	if( unlikely(ch >= NUM_CHANNELS))
		return;
		
	local_irq_save(flags);
	gdev->ch[ch].count++;
	local_irq_restore(flags);
}
EXPORT_SYMBOL(hres_event_cnt);

/*
 *   Mark Event Start
 */
void 
hres_event_start ( uint ch )
{
	unsigned long flags;
	
	if( unlikely(gdev == NULL))
		return;
		
	if( unlikely(ch >= NUM_CHANNELS))
		return;
		
	local_irq_save(flags);
	gdev->ch[ch].tstamp = hres_read_tick();
	gdev->ch[ch].active = 1;
	local_irq_restore(flags);
}
EXPORT_SYMBOL(hres_event_start);

/*
 *   Mark Event End and count
 */
u32 
hres_event_end ( uint ch )
{
	u32 ts = 0;
	unsigned long flags;

	if( unlikely(gdev == NULL))
		return 0;

	if( unlikely(ch >= NUM_CHANNELS)) 
		return 0;

	if(!gdev->ch[ch].active)
		return 0;
		
	local_irq_save(flags);
	ts = gdev->read_hres_timer( gdev->timer ) - gdev->ch[ch].tstamp;
	gdev->ch[ch].total += ts;
	gdev->ch[ch].count++;
	gdev->ch[ch].active = 0;
	local_irq_restore(flags);
	return ts;
	
}
EXPORT_SYMBOL(hres_event_end);


/*
 *   Mark Event End and count
 */
void
hres_event ( char *type, u32 arg1, u32 arg2 )
{
	unsigned long flags;
	struct hres_event *evt;

	if( unlikely(gdev == NULL))
		return;

	if( !evlog_on )
		return;

	local_irq_save(flags);
	evt = evlog + evlog_cnt;
	evlog_cnt++;
	if( evlog_cnt > evlog_num )
		evlog_num = evlog_cnt;
	if( evlog_cnt == NUM_EVENTS )
		evlog_cnt = 0;
	evt->tstamp = hres_read_tick();
	evt->type   = type;
	evt->arg1   = arg1;
	evt->arg2   = arg2;
	local_irq_restore(flags);
	return;
	
}
EXPORT_SYMBOL(hres_event);


/*
 *
 */
void
hres_evlog_reset(void)
{
	unsigned long flags;
	
	local_irq_save(flags);
	evlog_cnt = 0;
	evlog_num = 0;
	local_irq_restore(flags);
}
EXPORT_SYMBOL(hres_evlog_reset);


/*
 *  Enable logging
 */
int 
hres_evlog_enable(void)
{
	int rc = evlog_on;
	evlog_on = 1;
	return rc;
}
EXPORT_SYMBOL(hres_evlog_enable);

/*
 *  Disable logging
 */
int 
hres_evlog_disable(void)
{
	int rc = evlog_on;
	evlog_on = 0;
	return rc;
}
EXPORT_SYMBOL(hres_evlog_disable);

/*
 *  Print log
 */
void
hres_evlog_print ( void )
{
	int i;
	u32 last_tstamp = 0;
	struct hres_event *evt = evlog;

	printk ("evlog: beg\n");
	for (i = 0; i < evlog_num; i++, evt++ ) {
		printk ( "%010d (d%10d): %12d (0x%08x)  %12d (0x%08x)  %s\n",
		   gdev->convert_hres_timer(evt->tstamp),
		   gdev->convert_hres_timer(evt->tstamp - last_tstamp),
		   evt->arg1, evt->arg1,
		   evt->arg2, evt->arg2, 
		   evt->type );
		last_tstamp = evt->tstamp;
	}
	printk ("evlog: end\n");
}
EXPORT_SYMBOL(hres_evlog_print);


/*
 *   Sysfs
 */
static ssize_t 
counter_show ( struct device *dev, 
               struct device_attribute *attr, 
               char *buf )
{
	return snprintf( buf, PAGE_SIZE, "%d\n", hres_read_tick());
}

static DEVICE_ATTR( counter, S_IRUGO | S_IWUSR, counter_show, NULL );



static ssize_t 
evlog_show ( struct device *dev, 
             struct device_attribute *attr, 
             char *buf )
{
	hres_evlog_print ();
	return 0;
}

static ssize_t 
evlog_store( struct device *dev,
             struct device_attribute *attr,
             const char *buf, size_t count )
{
	hres_evlog_reset();
	return count;
}


static DEVICE_ATTR( evlog, S_IRUGO | S_IWUSR, evlog_show, evlog_store );



static ssize_t 
channels_show ( struct device *dev, 
                struct device_attribute *attr, 
                char *buf )
{
	int ch; 
	ssize_t len = 0;

	for ( ch = 0; ch < NUM_CHANNELS; ch++ ) {
		len += snprintf( buf + len, PAGE_SIZE - len, 
		"%02d:  %010u   %4d\n", ch,
		gdev->convert_hres_timer(gdev->ch[ch].total),
		gdev->ch[ch].count
		);
	}
	return len;
}

static ssize_t 
channels_store( struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count )
{
	int ch;

	ch = simple_strtol ( buf, NULL, 10 );
	if( ch == -1 ) {
		for ( ch = 0; ch < NUM_CHANNELS; ch++ )
			hres_ch_reset ( ch );
	}
	else {
		hres_ch_reset ((uint) ch );
	}
	return count;
}

static DEVICE_ATTR( channels, S_IRUGO | S_IWUSR, channels_show, channels_store);


static void 
hres_sysrq_show_evlog(int key, struct tty_struct *tty)
{
	hres_evlog_print();
}


/* 
 *
 */
static int
hres_panic(struct notifier_block *this, unsigned long event, void *ptr)
{
	hres_evlog_print();
	
	return NOTIFY_DONE;
}

static struct notifier_block panic_block = {
	.notifier_call = hres_panic,
};

/*
 *  Sys req related 
 */
static struct sysrq_key_op sysrq_show_log_op = {
	.handler	= hres_sysrq_show_evlog,
	.help_msg	= "show-evLog",
	.action_msg	= "Show HiRes EvLog",
};

static void 
hres_sysrq_reset_evlog(int key, struct tty_struct *tty)
{
	hres_evlog_reset();
}


static struct sysrq_key_op sysrq_reset_log_op = {
	.handler	= hres_sysrq_reset_evlog,
	.help_msg	= "reset-evlog(c)",
	.action_msg	= "Reset Hires EvLog",
};

/*
 *
 */
static int __devexit
hres_counter_remove ( struct platform_device *pdev )
{
	struct dev_ctxt *ctxt = gdev;

	gdev = NULL;

	device_remove_file ( &pdev->dev, &dev_attr_counter  );
	device_remove_file ( &pdev->dev, &dev_attr_channels );
	device_remove_file ( &pdev->dev, &dev_attr_evlog    );
		
	gdev->release_hres_timer  ( ctxt->timer );

	platform_set_drvdata( pdev, NULL );

	kfree ( ctxt );

	return 0;	 
}



/*
 *
 */
static int __devinit 
hres_counter_probe ( struct platform_device  *pdev )
{
	int rc;
	struct dev_ctxt *ctxt;
	struct hres_counter_platform_data *pdata = pdev->dev.platform_data;

	ctxt = kzalloc ( sizeof(struct dev_ctxt), GFP_KERNEL );
	if( ctxt == NULL ) 
		goto ret_nodev;

	ctxt->pdev = pdev;
	ctxt->init_hres_timer = pdata->init_hres_timer;
	ctxt->release_hres_timer = pdata->release_hres_timer;
	ctxt->read_hres_timer = pdata->read_hres_timer;
	ctxt->convert_hres_timer = pdata->convert_hres_timer;

	ctxt->suspend_hres_timer = pdata->suspend_hres_timer;
	ctxt->resume_hres_timer = pdata->resume_hres_timer;
			
	rc = ctxt->init_hres_timer(&ctxt->timer);
	if( rc )
		goto free_ctxt;
	
	platform_set_drvdata ( pdev, ctxt );

	rc = device_create_file ( &pdev->dev, &dev_attr_counter );
	if( rc ) 
		goto free_timer;

	rc = device_create_file ( &pdev->dev, &dev_attr_channels );
	if( rc ) 
		goto free_sysfs_counter;

	rc = device_create_file ( &pdev->dev, &dev_attr_evlog );
	if( rc ) 
		goto free_sysfs_channels;

	gdev = ctxt;

	printk( KERN_INFO "Initialize %s device\n", 
	        pdev->name);
#if 0
#ifndef CONFIG_HRES_COUNTER_TIMER32K
	printk( KERN_INFO "Initialize %s device (%dMHz)\n", 
	        pdev->name, clk_rate );
#else
	printk( KERN_INFO "Initialize %s device (32KHz)\n", pdev->name);
#endif
#endif

	// register sys request key
	register_sysrq_key( 'l', &sysrq_show_log_op );
	register_sysrq_key( 'c', &sysrq_reset_log_op );

	// register panic callback
	atomic_notifier_chain_register(&panic_notifier_list, &panic_block);

	return 0;
	
free_sysfs_channels:
	device_remove_file ( &pdev->dev, &dev_attr_channels );
free_sysfs_counter:
	device_remove_file ( &pdev->dev, &dev_attr_counter );
free_timer:
	ctxt->release_hres_timer ( ctxt->timer );
free_ctxt:
	kfree ( ctxt );
ret_nodev:
	return -ENODEV;
}


#ifdef CONFIG_HRES_COUNTER_TIMER32K
#undef CONFIG_PM
#endif

#ifdef CONFIG_PM
static int 
hres_counter_suspend( struct platform_device *dev, pm_message_t state )
{
	struct dev_ctxt *ctxt = gdev;

	if( ctxt->suspended )
		return 0;

	ctxt->resume_hres_timer(gdev->timer);

	ctxt->suspended = 1;

	return 0;
}

static int 
hres_counter_resume ( struct platform_device *dev )
{
	struct dev_ctxt *ctxt = gdev;

	if(!ctxt->suspended )
		return 0;

	ctxt->suspend_hres_timer(gdev->timer);

	ctxt->suspended = 0;

	return 0;
}
#else
#define hres_counter_suspend	NULL
#define hres_counter_resume	NULL
#endif	/* CONFIG_PM */


/*
 *
 */

static struct platform_driver hres_counter_driver = {
	.driver   = {
		.name = "hres_counter",
	},
	.probe	  = hres_counter_probe,
	.remove   = __devexit_p(hres_counter_remove),
	.suspend  = hres_counter_suspend,
	.resume   = hres_counter_resume,
};

static int __init  
hres_counter_init(void) 
{ 
	platform_driver_register ( &hres_counter_driver );
	return 0;
} 

 
static void __exit 
hres_counter_exit(void) 
{ 
	platform_driver_unregister ( &hres_counter_driver );
	return;
} 
 
module_init(hres_counter_init); 
module_exit(hres_counter_exit); 
 
MODULE_DESCRIPTION("OMAP High resolution counter driver" );
MODULE_LICENSE("GPL");  


