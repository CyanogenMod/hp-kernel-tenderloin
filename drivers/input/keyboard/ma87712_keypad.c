/*
 * linux/drivers/input/keyboard/ma87712.c
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#include <linux/input/ma87712.h>

#include <asm/irq.h>
#include <mach/gpio.h>
#include <linux/gpio.h>


#undef MA87712_DEBUG
#define MA87712_DEBUG	1

#ifdef MA87712_DEBUG
#define debug(f,a...)	printk("%s:" f,  __FUNCTION__ , ## a);
#else
#define debug(f,a...)
#endif

#define MA87712_ENABLE_TOUCH	0

struct ma87712_state {
	struct i2c_client *i2c_dev;
	struct input_dev *keypad_dev;
	struct ma87712_platform_data *pdata;
	struct work_struct scan;
	struct workqueue_struct *keypad_wq;
	int suspended;

	/* [CM3212]  */
	struct  mutex		update_lock;
};

static struct input_dev *ma87712_inp = NULL;

/******************************************************************************
*
* ma87712_i2c_read_u8()
*
* Inputs
*
* Returns
*
*
******************************************************************************/
static int ma87712_i2c_read_u8(struct i2c_client *client, u8 index, u8 * out)
{
	int ret;
	struct i2c_msg msgs[2];

	// Write the register index.
	msgs[0].addr	= client->addr;
	msgs[0].len	= 1;
	msgs[0].flags	= 0;
	msgs[0].buf	= &index;

	// Read the register value.
	msgs[1].addr	= client->addr;
	msgs[1].len	= 1;
	msgs[1].flags	= I2C_M_RD;
	msgs[1].buf	= out;

	// delay 100us
	udelay(100);

	// Make it so...
	ret = i2c_transfer(client->adapter, msgs, 2);

	return (ret);
}

/******************************************************************************
*
* ma877129_i2c_write_u8()
*
* Inputs
*
* Returns
*
*
******************************************************************************/
#if 0
static int ma87712_i2c_write_u8(struct i2c_client *client, u8 index, u8 value)
{
	u8 buf[2] = { index, value };
	int ret;
	struct i2c_msg msg[1];

	msg[0].addr	= client->addr;
	msg[0].flags	= 0;
	msg[0].len	= 2;
	msg[0].buf	= buf;

	// delay 100us
	udelay(100);

	ret = i2c_transfer(client->adapter, msg, 1);
	return ret;
}
#endif
/******************************************************************************
*
* ma877129_i2c_read()
*
* Inputs
*
* Returns
*
*
******************************************************************************/
static int ma87712_i2c_read(struct i2c_client *client, u8 index, u8 *out, u8 num_bytes)
{
	int ret;
	struct i2c_msg msgs[2];

	// Write the register index.
	msgs[0].addr	= client->addr;
	msgs[0].len	= 1;
	msgs[0].flags	= 0;
	msgs[0].buf	= &index;

	// Read the register value.
	msgs[1].addr	= client->addr;
	msgs[1].len	= num_bytes;
	msgs[1].flags	= I2C_M_RD;
	msgs[1].buf	= out;

	// delay 100us
	udelay(100);

	// Make it so...
	ret = i2c_transfer(client->adapter, msgs, 2);

	/* i2cTransfer returns num messages.translate it pls.. */
	if (ret >= 0)
	   ret = 0;
	return ret;
}

/******************************************************************************
*
* ma877129_dummy_cmd()
*
* Inputs
*
* Returns
*
*
******************************************************************************/
//static int ma87712_dummy_cmd(struct i2c_client *client)
//{
//	u8 ma87712_id;
//	int ret;
//
//	ret = ma87712_i2c_read_u8(client, REG_DEVICE_ID, &ma87712_id);
 //	if (ma87712_id != SLAVE_ID)
//		return 1;
//	else
//		return 0;
//}

/******************************************************************************
*
* ma877129_check_awake()
*
* Inputs
*
* Returns
*
*
******************************************************************************/
//static int ma87712_check_awake(struct i2c_client *client)
//{
//	u8 val;
//
//	ma87712_dummy_cmd(client);
//	udelay(200);
//
//	return ma87712_i2c_read_u8(client, REG_CTRL0, &val);
//}

/******************************************************************************
*
* ma87712_kb_interrupt()
*
* The handler for the irq when a key is pressed or released. Disable the
* irq lines and schedules workqueue to scan the key and report the event
* to the input subsystem
*
* Inputs
*   irq         The IRQ number being serviced. Not used.
*   dev_id      Device ID provided when IRQ was registered
*
* Returns
*   IRQ_HANDLED
*
******************************************************************************/
static irqreturn_t ma87712_interrupt(int irq, void *dev_id)
{
	struct ma87712_state *state = dev_id;

	debug("ma87712 interrupt fired, name = %s, irq = %d\n", state->i2c_dev->name, irq);
	//don't use this function, which will cause exception. schedule_work(&state->scan);
	queue_work(state->keypad_wq, &state->scan);
	return IRQ_HANDLED;
}

/******************************************************************************
*
* ma87712_scan()
*
* Scan for interpreting the keyboard or touchpad interrupt.
*
*
* Inputs
*   None.
*
* Returns
*   None.
*
******************************************************************************/
static void ma87712_scan(struct work_struct *work)
{
	u8 val, num;
	u16 code;
	int i;
	unsigned int key;

	struct ma87712_state *state =
	    container_of(work, struct ma87712_state, scan);

	//ma87712_dummy_cmd(state->i2c_dev);
	//ma87712_check_awake(state->i2c_dev);

	// in normal or idel mode
	ma87712_i2c_read_u8(state->i2c_dev, REG_STATUS, &val);
	debug("++++ ma87712 scan REG_STATUS(0x40) = 0x%x\n", val);

	// keyboard interrupt
	num = val;
	debug("======>keyboard interrupt, # of keys pressed = %d\n", num );

	// read key code
	for (i = 0; i < num; i++)
	{
		// Read key scan code
		ma87712_i2c_read(state->i2c_dev, REG_KEY_TABLE_L, (u8 *) &code, 2);
		debug("======>loop = %d, REG_KEY_TABLE_L = %d\n", i, code );
		key = (code & 0x7FFF);

		if (code & KEY_RELEASED){
			debug("======>key released! key = %x, 0\n", key);
			if (key == 0x86)
			{
				key = 0x8b;
			}
			//input_report_key(state->keypad_dev, key, 0);  // raw data
			if (key < state->pdata->keycodemax)
				input_report_key(state->keypad_dev, state->pdata->keycode[key], 0);
		}else {
			debug("======>key pressed! key = %x, 1\n", key);
			if (key == 0x86)
			{
				key = 0x8b;
			}
			//input_report_key(state->keypad_dev, key, 1);	// raw data
			if (key < state->pdata->keycodemax)
				input_report_key(state->keypad_dev, state->pdata->keycode[key], 1);
		}
		input_sync(state->keypad_dev);
	}

}

/******************************************************************************
*
* ma87712_config()
*
* MA87712 Initial Setting
*
* Returns
*   0 on success, or non-zero otherwise.
*
******************************************************************************/
static int ma87712_config(struct i2c_client *client, struct ma87712_platform_data *pdata)
{
	u8 val;
	int i, rc;

	if (pdata->set_power == NULL)
		return -EINVAL;

	if (pdata->set_power(1) == -1)
		return -EINVAL;

	// Power up
	rc = gpio_request(pdata->gpio_pd_n, "ma87712 pd_n");
	if (rc != 0){
		debug("++++++ gpio%d request err(%d)!\n",pdata->gpio_pd_n, rc);
		goto err0;
	}

	rc = gpio_direction_output(pdata->gpio_pd_n, 1);
	if (rc != 0) {
		debug("++++++ gpio%d direction-input err(%d)!\n",pdata->gpio_pd_n, rc);
		goto err1;
	}

	mdelay(5);

	rc = gpio_request(pdata->gpio_rst_n, "ma87712 rst_n");
	if (rc != 0){
		debug("++++++ gpio%d request err(%d)!\n",pdata->gpio_rst_n, rc);
		goto err1;
	}

	rc = gpio_direction_output(pdata->gpio_rst_n, 1);
	if (rc != 0) {
		debug("++++++ gpio%d direction-input err(%d)!\n",pdata->gpio_rst_n, rc);
		goto err2;
	}

	// delay 50ms - 100ms
	mdelay(100);

	/* Check Device ID */
	ma87712_i2c_read_u8(client, REG_DEVICE_ID, &val);
	if (val != SLAVE_ID){
		printk(KERN_ERR "+++++++ SLAVE_ID is not match = %x\n", val);
		//goto err;
	}

	/* MCU FW version */
	ma87712_i2c_read_u8(client, REG_SW_VERSION, &val);
	printk(KERN_ERR "#####++++ MCU FW Version = %x\n", val);

#if 0 // Henry
	// SW Reset
	ma87712_i2c_write_u8(client, REG_CTRL0, DEVICE_RESET);
#endif

	/* Keyboard initialization */
#if 0 // Henry
	// Set host wake up-- to check if the device is alive -- make is as a wrap
	rc = ma87712_i2c_read_u8(client, REG_CTRL0, &val);
	if (!rc) {
		ma87712_i2c_write_u8(client, REG_CTRL0, (val & 0xFB));
	}

	// Idle time setting  --15
	rc = ma87712_i2c_read_u8(client, REG_CTRL1, &val);
	if (!rc) {
		val &= 0xF0;
		ma87712_i2c_write_u8(client, REG_CTRL1, (val|15));
	}

	// interrupt type select
	rc = ma87712_i2c_read_u8(client, REG_CTRL1, &val);
	if (!rc) {
		val |= 0x10;
		ma87712_i2c_write_u8(client, REG_CTRL1, val );
	}

	// Repeat setting-- key/mouse interrupt no repeat, repeat interval 200ms
	rc = ma87712_i2c_read_u8(client, REG_CTRL1, &val);
	if (!rc) {
		val &= 0x7F;
		ma87712_i2c_write_u8(client, REG_CTRL1, val);
	}

	// Don't report Z
	rc = ma87712_i2c_read_u8(client, REG_CTRL1, &val);
	if (!rc) {
		val &= 0xBF;
		ma87712_i2c_write_u8(client, REG_CTRL1, val);
	}
#endif // if 0

	// read out the 40h
	rc = ma87712_i2c_read_u8(client, REG_STATUS ,&val);
	debug("++++++ REG_STATUS = %x\n", val);

	// Clean the fifo
	for (i = 0; i < FIFO_LENGTH; i++)
	{
		ma87712_i2c_read_u8(client, REG_KEY_TABLE_L, &val);
		ma87712_i2c_read_u8(client, REG_KEY_TABLE_H, &val);
	}

	return (0);
err2:
	gpio_free(pdata->gpio_rst_n);
err1:
	gpio_free(pdata->gpio_pd_n);
err0:
	return -EINVAL;

}

struct input_dev *ma87712_input_dev(void)
{
	return ma87712_inp;
}

/******************************************************************************
*
* ma87712_kb_probe()
*
* Called by driver model to initialize device
*
* Returns
*   0 on success, or non-zero otherwise.
*
******************************************************************************/
static int ma87712_probe(struct i2c_client  *client, const struct i2c_device_id *id)
{
	u16 val;
	int rc;
	int i;

	struct ma87712_state *state = NULL;
	struct ma87712_platform_data *pdata = NULL;

	debug("%s +\n", __func__);

	/* get platform data */
	pdata = client->dev.platform_data;
	if (pdata == NULL || pdata->keycode == NULL || pdata->keycodemax== 0 || pdata->keycodesize == 0) {
		printk(KERN_ERR "%s: missing platform data\n",
			MA87712_DRIVER);
		return (-ENODEV);
	}

	/* Create the keypad state */
	state = kzalloc(sizeof(struct ma87712_state), GFP_KERNEL);
	if (!state)
		return (-ENOMEM);

	/* attach i2c_dev */
	state->i2c_dev = client;

	/* attach platform data */
	state->pdata = pdata;

	/* init workq */
	INIT_WORK(&state->scan, ma87712_scan);
	state->keypad_wq = create_workqueue("keypad_irq_wq");

	/* setup input device */
	state->keypad_dev = input_allocate_device();
	if (state->keypad_dev == NULL)
		goto err0;

	/* setup driver name */
	state->keypad_dev->name = "ma87712_keypad"; //MA87712_NAME;

	/* Enable key event and auto repeat feature of Linux input subsystem */
	set_bit(EV_KEY, state->keypad_dev->evbit);
	set_bit(EV_REP, state->keypad_dev->evbit);

	// setup keymap
	state->keypad_dev->keycode		= pdata->keycode;
	state->keypad_dev->keycodemax	= pdata->keycodemax;		// FIXME later
	state->keypad_dev->keycodesize	= pdata->keycodesize;

	// Use input device default autorepeat settings
	state->keypad_dev->rep[REP_DELAY] = 0;
	state->keypad_dev->rep[REP_PERIOD] = 0;

	for (i = 0; i < state->keypad_dev->keycodemax; i++)
		set_bit(pdata->keycode[i] & KEY_MAX, state->keypad_dev->keybit);
	//	set_bit(i & KEY_MAX, state->keypad_dev->keybit);

	/* indicate that we generate *any* key and movement event */
	//bitmap_fill(state->keypad_dev->keybit, KEY_MAX);


	/* [CM3212]  RESEVERED for LS Event for future(?) */
#if 0 /* [INPUT DEVICE] set_bit() does not work when 'module' build but is OK for 'static' build  */
	/* When LS event is necessary, define new event node other than 'keypad_dev'. */
	set_bit(EV_MSC, state->keypad_dev->evbit);  /* MISC Event */
	set_bit(MSC_RAW, state->keypad_dev->mscbit);   /* MSC_RAW */
#endif


	/* register input device */
	rc = input_register_device(state->keypad_dev);
	if (rc != 0) {
		printk(KERN_ERR "Unable to register ma87712-keypad input device\n");
		goto err1;
	}

	ma87712_inp = state->keypad_dev;

	/* [CM3212]: */
	mutex_init(&state->update_lock);

	// FIXME: check peripheral device 0x6E, such as battery

	// MA87712 configuration initialization
	if ((rc = ma87712_config(client, pdata)))
		goto err4;

#if 0
	/* [ CM3212] initialize */
	ma87712_i2c_write_u8(client, REG_LS_CTRL, CM3212_INIT_SETUP);
	/* [CM3212] end of init */
#endif

	if (pdata->gpio_int_n)
	{
		debug(" ## config gpio%d as input\n", pdata->gpio_int_n);
		gpio_tlmm_config( GPIO_CFG(pdata->gpio_int_n, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), GPIO_CFG_ENABLE ); /* config as INPUT */
	}

	debug(" client->irq = %d\n", client->irq);
	/* Enable interrupt */
	rc = request_irq(client->irq,  ma87712_interrupt,
		IRQF_TRIGGER_FALLING, "ma87712_int", (void *)state);

	if (rc < 0) {
		debug("++++++ request irq  err (%d)!\n", rc);
		goto err5;
	}

	/**********************************************************/
	/******************** Hacking for QSD BUG **********************/
	ma87712_i2c_read(state->i2c_dev, REG_KEY_TABLE_L, (u8 *) &val, 2);
	debug("============> hacking way, reg_key_table_l = %x \n", val);

#if 0
	///*
	ma87712_i2c_read(state->i2c_dev, REG_MOUSE_X, (u8 *) &val, 2);
	debug("============> hacking way, reg_mouse_x = %x \n", val);
	//*/
	/**********************************************************/
	/******************** Hacking for QSD BUG **********************/
#endif

	/* attach driver data */
	i2c_set_clientdata(client, state);

#if 0
	/* Create SYSFS entries for MA87712 */
	if ((rc = device_create_file(&state->i2c_dev->dev, &dev_attr_bl_brightness)))
		goto err6;

	if ((rc = device_create_file(&state->i2c_dev->dev, &dev_attr_auto_on)))
		goto err7;

	if ((rc = device_create_file(&state->i2c_dev->dev, &dev_attr_caplock_on)))
		goto err8;

	if ((rc = device_create_file(&state->i2c_dev->dev, &dev_attr_mute_on)))
		goto err9;

	if ((rc = device_create_file(&state->i2c_dev->dev, &dev_attr_leds_state)))
		goto err10;

	/* [CM3212] */
	if ((rc = device_create_file(&state->i2c_dev->dev, &dev_attr_lsensor_value)))
		goto err11;
#endif

	return 0;


#if 0
err12: /* [CM3212] free GPIO84 */
	gpio_free(105);
#endif
#if 0
err11:  /* [CM3212] */
	device_remove_file(&state->i2c_dev->dev, &dev_attr_lsensor_value);


err10:
	device_remove_file(&state->i2c_dev->dev, &dev_attr_mute_on);
err9:
	device_remove_file(&state->i2c_dev->dev, &dev_attr_caplock_on);
err8:
	device_remove_file(&state->i2c_dev->dev, &dev_attr_auto_on);
err7:
	device_remove_file(&state->i2c_dev->dev, &dev_attr_bl_brightness);
#endif

//err6:
	free_irq(state->i2c_dev->irq, state);
err5:
	//	gpio_free(state->pdata->gpio);

err4:
#if 0
	input_unregister_device(state->touchpad_dev);
#endif

//err3:
	input_unregister_device(state->keypad_dev);

#if 0
err2:
	input_free_device(state->touchpad_dev);
#endif

err1:
	input_free_device(state->keypad_dev);
err0:
	kfree(state);

	debug("Failed to intialize MosArt %s MCU river\n", MA87712_DRIVER);
	return -ENODEV;
}

/******************************************************************************
*
* ma87712_kb_remove()
*
* Called by driver model to remove device
*
*
******************************************************************************/
static int ma87712_remove(struct i2c_client *client)
{
	struct ma87712_state *state = i2c_get_clientdata(client);

	/* cancel any pending work */
	cancel_work_sync(&state->scan);

	/* unregister input device */
	input_free_device(state->keypad_dev);
//	input_free_device(state->touchpad_dev);

	input_unregister_device(state->keypad_dev);
//	input_unregister_device(state->touchpad_dev);

	/* add hardware deinitialization here (lowest power mode please) */

	/* unregister interrupt handler and free gpio */
	free_irq(state->i2c_dev->irq, state);
	//gpio_free(state->pdata->gpio);
	/* [CM3212] */

	// Power down
	gpio_direction_output(state->pdata->gpio_pd_n, 0);
	gpio_direction_output(state->pdata->gpio_rst_n, 0);

	gpio_free(state->pdata->gpio_rst_n);
	gpio_free(state->pdata->gpio_pd_n);

	state->pdata->set_power(0);

#if 0
	/* remove sysfs entries */
	device_remove_file(&state->i2c_dev->dev, &dev_attr_bl_brightness);
	device_remove_file(&state->i2c_dev->dev, &dev_attr_auto_on);
	device_remove_file(&state->i2c_dev->dev, &dev_attr_caplock_on);
	device_remove_file(&state->i2c_dev->dev, &dev_attr_mute_on);
	device_remove_file(&state->i2c_dev->dev, &dev_attr_leds_state);
	/* [CM3212] */
	device_remove_file(&state->i2c_dev->dev, &dev_attr_lsensor_value);
#endif

	/* unregister interrupt handler and free gpio */
	//free_irq(state->i2c_dev->irq, state);
	//gpio_free(irq_to_gpio(state->i2c_dev->irq));

	/* detach state */
	i2c_set_clientdata(client, NULL);

	/* free state */
	kfree(state);

	return 0;
}

#ifdef CONFIG_PM
/******************************************************************************
*
* ma87712_kb_suspend()
*
******************************************************************************/
static int ma87712_suspend(struct i2c_client *dev, pm_message_t event)
{
	struct ma87712_state *state = i2c_get_clientdata(dev);

	/* Check if already suspended */
	if (state->suspended)
		return 0;

	/* cancel any pending work */
	cancel_work_sync(&state->scan);

	/* Put the hardware to low power state */

	/* generate release event for all pressed keys (implement me) */

	state->suspended = 1;


	return 0;
}

/******************************************************************************
*
* ma87712_kb_resume()
*
******************************************************************************/

static int ma87712_resume(struct i2c_client *dev)
{
	struct ma87712_state *state = i2c_get_clientdata(dev);

	if (!state->suspended)
		return 0;

	/* reinitialize hardware */

	/*? enable interrupts */

	state->suspended = 0;

	return 0;
}

#else	/* CONFIG_PM */
#  define ma87712_suspend  NULL
#  define ma87712_resume   NULL
#endif	/* CONFIG_PM */

static const struct i2c_device_id ma87712_id_table[] = { //You need to have id_table //jyoo
	{ "ma87712", 0},
	//{0,0}
	{ }
};

static struct i2c_driver ma87712_driver = {
	.probe		= ma87712_probe,
	.remove		= ma87712_remove,
	.suspend	= ma87712_suspend,
	.resume		= ma87712_resume,
	.id_table   = ma87712_id_table,
	.driver		= {
		.name	= MA87712_DRIVER,
	},
};

/******************************************************************************
*
* ma87712_init()
*
* This is the function called when the module is loaded.
* Only call register driver here.
*
* Returns
*   0 on success, or non-zero otherwise.
*
******************************************************************************/
static int __init ma87712_init(void)
{
	debug("Loading MosArt MA87712 driver!\n");
	return i2c_add_driver(&ma87712_driver);
}

/******************************************************************************
*
* ma87712_exit()
*
* This is the function called when the module is unloaded.
* Only call unregister driver here.
*
* Returns
*   0 on success, or non-zero otherwise.
*
******************************************************************************/
static void __exit ma87712_exit(void)
{
	i2c_del_driver(&ma87712_driver);
}

module_init(ma87712_init);
module_exit(ma87712_exit);

MODULE_DESCRIPTION("MosArt MA87712 Keypad Driver");
MODULE_LICENSE("GPL");
