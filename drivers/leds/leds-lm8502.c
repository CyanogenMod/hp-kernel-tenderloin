/*
 *  linux/drivers/misc/lm8502 - Driver for the LM8502 which is a combo <LED + Vibrator + Camera flash> device
 *
 *  Copyright (C) 2008 Palm Inc,
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Authors: Kevin McCray (kevin.mccray@palm.com)
 *          Brian Xiong (brian.xiong@palm.com)
 *          Rajat Gupta (rajat.gupta@palm.com)
 *          Amon Xie (amon.xie@palm.com)
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/leds.h>
#include <linux/i2c.h>
#include <linux/ctype.h>
#include <linux/mutex.h>
#include <linux/i2c_lm8502_led.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include "../staging/android/timed_output.h"
#include "leds-lm8502.h"


//TODO: There are no register settings for auto-increment. Assuming feature missing in chip for now.
//#define AUTO_INCR_WRITE
//TODO: The dev board did not have the strobe pin connected. Disabling for now.
//#define STROBE_ENABLE

static int lm8502_ioctl(struct inode *inode, struct file *file,
             unsigned int cmd, unsigned long arg);

DECLARE_WAIT_QUEUE_HEAD(lm8502_wait_queue);
DECLARE_WAIT_QUEUE_HEAD(lm8502_wait_stop_engine);

struct lm8502_device_state {
    struct i2c_client *i2c_dev;
    struct lm8502_platform_data pdata;
    struct work_struct notify_work;
    struct mutex lock;
    struct file_operations fops;
    struct miscdevice mdev;
    u16 instruct[INSTR_LEN];
    int suspended;
    int interrupt_state;
    int stop_engine_state;
    u8 stopengine;
    struct class *vib_class;
    struct device *vib_class_dev;
    u8 vib_enable;
    u8 vib_start;
    u8 vib_duty_cycle;
    u8 vib_direction;
    struct class *flash_class;
    struct device *flash_class_dev;
    u8 is_flash_mode;  // true => flash_mode false=> torch_mode
    u8 flash_enable;
    u8 flash_start;
    u16 flash_duration;
    u16 flash_current;
    u16 torch_current;
    /* debugfs variables */
    struct dentry        *debug_dir;
    u8 addr;
    u8 value;

    struct hrtimer vib_timer;
    struct timed_output_dev timed_dev;
    spinlock_t spinlock;
    struct work_struct work;
    int state;
};


struct engine_cmd {
    struct lm8502_device_state *context;
    struct work_struct workitem;
    int engine;
    int cmd;
};

struct brightness_entry {
    u8 code;
    u16 current_value;
};

static struct brightness_entry flash_brightness_map[] = {
    {0x0, 38},
    {0x1, 75},
    {0x2, 113},
    {0x3, 150},
    {0x4, 188},
    {0x5, 225},
    {0x6, 263},
    {0x7, 300},
    {0x8, 338},
    {0x9, 375},
    {0xA, 413},
    {0xB, 450},
    {0xC, 488},
    {0xD, 525},
    {0xE, 563},
    {0xF, 600}
};

static struct brightness_entry torch_brightness_map[] = {
    {0x0, 18},
    {0x1, 37},
    {0x2, 56},
    {0x3, 75},
    {0x4, 93},
    {0x5, 112},
    {0x6, 131},
    {0x7, 150},
};

static struct lm8502_device_state *lm8502_state;
static struct workqueue_struct *lm8502_vib_wq;



static u8 lm8502_get_closest_flash_current(u16 desired_current)
{
    u16 i;
    for(i = 0; i < sizeof(flash_brightness_map) / sizeof(flash_brightness_map[0]); i++) {
        if (flash_brightness_map[i].current_value >= desired_current) {
            return i;
        }
    }
    return (sizeof(flash_brightness_map)/sizeof(flash_brightness_map[0]))-1;
}

static u8 lm8502_get_closest_torch_current(u16 desired_current)
{
    u16 i;
    for(i = 0; i < sizeof(torch_brightness_map) / sizeof(torch_brightness_map[0]); i++) {
        if(torch_brightness_map[i].current_value >= desired_current) {
            return i;
        }
    }
    return (sizeof(torch_brightness_map)/sizeof(torch_brightness_map[0]))-1;
}


int lm8502_i2c_read_reg(struct i2c_client* client, u8 addr, u8* out)
{
    int ret;
    struct i2c_msg msg[2];

    msg[0].addr = client->addr;
    msg[0].len = 1;
    msg[0].flags = 0;
    msg[0].buf = &addr;

    msg[1].addr = client->addr;
    msg[1].len = 1;
    msg[1].flags = I2C_M_RD;
    msg[1].buf = out;

    ret = i2c_transfer(client->adapter, msg, 2);
    if (ret != 2)
        printk(KERN_ERR "Unable to read LM8502 registers\n");

    return ret;
}

int lm8502_i2c_write_reg(struct i2c_client* client, u8 addr, u8 val)
{
    u8 buf[2] = {addr, val};
    int ret;
    struct i2c_msg msg[1];

    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].len = 2;
    msg[0].buf = buf;

    ret = i2c_transfer(client->adapter, msg, 1);
    if (ret != 1)
        printk(KERN_ERR "Unable to write to LM8502 registers\n");

	return ret;
}

int lm8502_set_current(uint8_t is_flash_mode, uint32_t mA)
{
	u8 index;
	int rc = 0;

	lm8502_state->pdata.select_flash(lm8502_state->i2c_dev);
	lm8502_state->is_flash_mode = is_flash_mode;
	// if vibrator enable, disable it to avoid vibrator operation
	lm8502_state->vib_enable = 0;
	// avoid vib_enable be set
	lm8502_state->flash_enable = 1;

	if (is_flash_mode) {
		index = lm8502_get_closest_flash_current(mA);
		rc = lm8502_i2c_write_reg(lm8502_state->i2c_dev, FLASH_BRIGHTNESS, 
			flash_brightness_map[index].code << 3);
		lm8502_state->flash_current = flash_brightness_map[index].current_value;
	}
	else {
		index = lm8502_get_closest_torch_current(mA);
		rc = lm8502_i2c_write_reg(lm8502_state->i2c_dev, TORCH_BRIGHTNESS,
			torch_brightness_map[index].code << 3);
		lm8502_state->torch_current = torch_brightness_map[index].current_value;
	}

	lm8502_state->flash_enable = 0;

	return rc;
}

#ifdef AUTO_INCR_WRITE
static int lm8502_i2c_write_reg_auto_incr(struct i2c_client* client, u8 * address_and_data, u8 length)
{
    int ret;
    struct i2c_msg msg[1];

    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].len = length;
    msg[0].buf = address_and_data;

    ret = i2c_transfer(client->adapter, msg, 1);
    return ret;
}
#endif

static ssize_t lm8502_flash_or_torch_start_show(struct device *dev, struct device_attribute *attr, char *buf)
{

    struct lm8502_device_state *state;

    if (!buf)
        return -EINVAL;

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%d\n", state->flash_start);
}

static ssize_t lm8502_flash_or_torch_start_store(struct device *dev, struct device_attribute *attr,
                       const char *buf, size_t count)
{
    struct lm8502_device_state *state;
    u32 start;
    u8 reg;

    if (!buf || !count)
        return -EINVAL;

    if (sscanf(buf, "%u", &start) != 1)
        return -EINVAL;

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);
    if (!state->flash_enable) {
        printk(KERN_ERR "Flash must be enabled before use\n");
        return -EINVAL;
    }

    lm8502_i2c_read_reg(state->i2c_dev, FLASH_BRIGHTNESS, &reg);
    reg = reg & ~0x03;
    if (start) {
        if (state->is_flash_mode)
            reg = reg | FLASH_MODE;
        else
            reg = reg | TORCH_MODE;
    }
    lm8502_i2c_write_reg(state->i2c_dev, FLASH_BRIGHTNESS, reg);

    state->flash_start = start;

    return count;
}

static ssize_t lm8502_flash_duration_show(struct device *dev, struct device_attribute *attr, char *buf)
{

    struct lm8502_device_state *state;

    if (!buf)
        return -EINVAL;

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%d\n", state->flash_duration);
}

static ssize_t lm8502_flash_duration_store(struct device *dev, struct device_attribute *attr,
                   const char *buf, size_t count)
{
    struct lm8502_device_state *state;
    u32 duration;
    u8 reg;

    if (!buf || !count)
        return -EINVAL;

    if (sscanf(buf, "%u", &duration) != 1)
        return -EINVAL;

    if (duration >= 1024) {
        printk(KERN_ERR "Duration is too long\n");
        return -EINVAL;
    }

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);
    if (!state->flash_enable) {
        printk(KERN_ERR "Flash must be enabled before use\n");
        return -EINVAL;
    }

    state->flash_duration = duration;
    duration = duration >> 5;

    lm8502_i2c_read_reg(state->i2c_dev, FLASH_DURATION, &reg);
    reg = (reg & ~0x1F) | duration;
    lm8502_i2c_write_reg(state->i2c_dev, FLASH_DURATION, reg);

    return count;
}

static ssize_t lm8502_flash_current_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct lm8502_device_state *state;

    if (!buf)
        return -EINVAL;

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%u\n", state->flash_current);
}

static ssize_t lm8502_flash_current_store(struct device *dev, struct device_attribute *attr,
                       const char *buf, size_t count)
{
    struct lm8502_device_state *state;
    u32 desired_current;
    u8 index;
    u8 reg;

    if (!buf || !count)
        return -EINVAL;

    if (sscanf(buf, "%u", &desired_current) != 1)
        return -EINVAL;

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);
    if (!state->flash_enable) {
        printk(KERN_ERR "Flash must be enabled before use\n");
        return -EINVAL;
    }

    index = lm8502_get_closest_flash_current(desired_current);
    lm8502_i2c_read_reg(state->i2c_dev, FLASH_BRIGHTNESS, &reg);
    reg = reg & 0x07; //bit[7:3]
    reg = reg | STROBE_TIMEOUT | (flash_brightness_map[index].code << 3);
    lm8502_i2c_write_reg(state->i2c_dev, FLASH_BRIGHTNESS, reg);

    state->flash_current = flash_brightness_map[index].current_value;

    return count;
}

static ssize_t lm8502_torch_current_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct lm8502_device_state *state;

    if (!buf)
        return -EINVAL;

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%u\n", state->torch_current);
}

static ssize_t lm8502_torch_current_store(struct device *dev, struct device_attribute *attr,
                      const char *buf, size_t count)
{
    struct lm8502_device_state *state;
    u32 desired_current;
    u8 index;
    u8 reg;

    if (!buf || !count)
        return -EINVAL;

    if (sscanf(buf, "%u", &desired_current) != 1)
        return -EINVAL;

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);
    if (!state->flash_enable) {
        printk(KERN_ERR "Flash must be enabled before use\n");
        return -EINVAL;
    }

    index = lm8502_get_closest_torch_current(desired_current);
    lm8502_i2c_read_reg(state->i2c_dev, TORCH_BRIGHTNESS, &reg);
    reg = reg & ~0x38; //bit[5:3]
    reg = reg | (torch_brightness_map[index].code << 3);
    lm8502_i2c_write_reg(state->i2c_dev, TORCH_BRIGHTNESS, reg);

    state->torch_current = torch_brightness_map[index].current_value;

    return count;
}

static ssize_t lm8502_flash_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{

    struct lm8502_device_state *state;

    if (!buf)
        return -EINVAL;

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%d\n", state->flash_enable);
}

static ssize_t lm8502_flash_enable_store(struct device *dev, struct device_attribute *attr,
                      const char *buf, size_t count)
{
    struct lm8502_device_state *state;
    u32 enable;

    if (!buf || !count)
        return -EINVAL;

    if (sscanf(buf, "%u", &enable) != 1)
        return -EINVAL;

    if (enable > 2)
        return -EINVAL;

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);
    state->flash_enable = (u8)enable;

    if (!enable)
        return count;

    if (state->vib_enable) {
        printk(KERN_ERR "Flash and vibrator cannot be used together\n");
        return -EINVAL;
    }

    state->pdata.select_flash(state->i2c_dev);
    state->is_flash_mode = (enable == 1); // 1 == flash, 2 == torch

    return count;
}

static DEVICE_ATTR(flash_enable, S_IRUGO | S_IWUSR, lm8502_flash_enable_show, lm8502_flash_enable_store);
static DEVICE_ATTR(torch_current, S_IRUGO | S_IWUSR, lm8502_torch_current_show, lm8502_torch_current_store);
static DEVICE_ATTR(flash_current, S_IRUGO | S_IWUSR, lm8502_flash_current_show, lm8502_flash_current_store);
static DEVICE_ATTR(flash_duration, S_IRUGO | S_IWUSR, lm8502_flash_duration_show, lm8502_flash_duration_store);
static DEVICE_ATTR(flash_or_torch_start, S_IRUGO | S_IWUSR, lm8502_flash_or_torch_start_show, lm8502_flash_or_torch_start_store);

static ssize_t lm8502_vib_start_show(struct device *dev, struct device_attribute *attr, char *buf)
{

    struct lm8502_device_state *state;

    if (!buf)
        return -EINVAL;

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%u\n", state->vib_start);
}

static ssize_t lm8502_vib_start_store(struct device *dev, struct device_attribute *attr,
                      const char *buf, size_t count)
{
    struct lm8502_device_state *state;
    u32 start;

    if (!buf || !count)
        return -EINVAL;

    if (sscanf(buf, "%u", &start) != 1)
        return -EINVAL;

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);
    if (!state->vib_enable) {
        printk(KERN_ERR "Vibrator must be enabled before use\n");
        return -EINVAL;
    }

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);
    state->vib_start = (u8)start;

    if (start) {
        lm8502_i2c_write_reg(state->i2c_dev, HAPTIC_FEEDBACK_CTRL,
            0x02 + !!(state->pdata.vib_invert_direction ^ state->vib_direction));
    } else {
        lm8502_i2c_write_reg(state->i2c_dev, HAPTIC_FEEDBACK_CTRL, 0);
    }

    return count;
}

static ssize_t lm8502_vib_direction_show(struct device *dev, struct device_attribute *attr, char *buf)
{

    struct lm8502_device_state *state;

    if (!buf)
        return -EINVAL;

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%d\n", state->vib_direction);
}

static ssize_t lm8502_vib_direction_store(struct device *dev, struct device_attribute *attr,
                   const char *buf, size_t count)
{
    struct lm8502_device_state *state;
    u32 dir;

    if (!buf || !count)
        return -EINVAL;

    if (sscanf(buf, "%u", &dir) != 1)
        return -EINVAL;

    if (dir != 0 && dir != 1)
        return -EINVAL;

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);
    if (!state->vib_enable) {
        printk(KERN_ERR "Vibrator must be enabled before use\n");
        return -EINVAL;
    }

    if (state->vib_direction != (u8)dir) {
        state->vib_direction = (u8)dir;
        lm8502_i2c_write_reg(state->i2c_dev, HAPTIC_FEEDBACK_CTRL,
            0x02 + !!(state->pdata.vib_invert_direction ^ state->vib_direction ));
    }

    return count;
}


static ssize_t lm8502_vib_duty_cycle_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct lm8502_device_state *state;

    if (!buf)
        return -EINVAL;

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%u\n", state->vib_duty_cycle);
}

static ssize_t lm8502_vib_duty_cycle_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
    struct lm8502_device_state *state;
    u32 duty_cycle;

    if (!buf || !count)
        return -EINVAL;

    if (sscanf(buf, "%u", &duty_cycle) != 1)
        return -EINVAL;

    if (duty_cycle > 100)
        return -EINVAL;

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);
    if (!state->vib_enable) {
        printk(KERN_ERR "Vibrator must be enabled before use\n");
        return -EINVAL;
    }

    lm8502_i2c_write_reg(state->i2c_dev, HAPTIC_PWM_DUTY_CYCLE, (duty_cycle * 255) / 100);
    state->vib_duty_cycle = (u8)duty_cycle;

    return count;
}

static ssize_t lm8502_vib_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{

    struct lm8502_device_state *state;

    if (!buf)
        return -EINVAL;

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%d\n", state->vib_enable);
}

static ssize_t lm8502_vib_enable_store(struct device *dev, struct device_attribute *attr,
                       const char *buf, size_t count)
{
    struct lm8502_device_state *state;
    u32 enable;

    /* Do we have valid inputs? */
    if (!buf || !count)
        return -EINVAL;

    if (sscanf(buf, "%u", &enable) != 1)
        return -EINVAL;

    if (enable > 1)
        return -EINVAL;

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);
    state->vib_enable = (u8)enable;

    if (!enable) {
        return count;
    }

    /* Logic to enable follows */
    if (state->flash_enable) {
        printk(KERN_ERR "Vibrator and flash cannot be enabled together\n");
        return -EINVAL;
    }

    state->pdata.select_vibrator(state->i2c_dev);
    lm8502_i2c_write_reg(state->i2c_dev, HAPTIC_FEEDBACK_CTRL, 0);

    return count;
}

static DEVICE_ATTR(vib_enable, S_IRUGO | S_IWUSR, lm8502_vib_enable_show, lm8502_vib_enable_store);
static DEVICE_ATTR(vib_direction, S_IRUGO | S_IWUSR, lm8502_vib_direction_show, lm8502_vib_direction_store);
static DEVICE_ATTR(vib_duty_cycle, S_IRUGO | S_IWUSR, lm8502_vib_duty_cycle_show, lm8502_vib_duty_cycle_store);
static DEVICE_ATTR(vib_start, S_IRUGO | S_IWUSR, lm8502_vib_start_show, lm8502_vib_start_store);

static int lm8502_convert_percent_to_pwm(int percent, int *pwm)
{
    int ret = 0;

    if(percent < 0 || percent > 100)
    {
        ret = -EINVAL;
        goto done;
    }

    *pwm = (255 * percent) / 100;

done:
    return ret;
}

static int lm8502_config_engine_control_mode(struct lm8502_device_state *p_state, int eng, u8 engine_cntrl, u8 mode)
{
    u8 reg;
    int ret = 0;

    lm8502_i2c_read_reg(p_state->i2c_dev, engine_cntrl, &reg);

    // ENGINE_CNTRL1 and ENGINE_CNTRL2 have the same bit mappings
    // bits[5:4] = Engine 1
    // bits[3:2] = Engine 2

    switch(eng)
    {
        case 1:
            // clear existing bits for ENG1 control
            reg = reg & ~(3 << ENGINE_CNTRL_ENG1_SHIFT);
            lm8502_i2c_write_reg(p_state->i2c_dev, engine_cntrl, reg | (mode << ENGINE_CNTRL_ENG1_SHIFT));
            break;
        case 2:
            // clear existing bits for ENG2 control
            reg = reg & ~(3 << ENGINE_CNTRL_ENG2_SHIFT);
            lm8502_i2c_write_reg(p_state->i2c_dev, engine_cntrl, reg | (mode << ENGINE_CNTRL_ENG2_SHIFT));
            break;
        default:
            break;
    }

    return ret;
}

static void lm8502_setup_page_sizes(struct lm8502_device_state *p_state, int eng, int *startpage, int *endpage)
{
    switch(eng)
    {
        case 1:
            *startpage = p_state->pdata.memcfg->eng1_startpage;
            *endpage = p_state->pdata.memcfg->eng1_endpage;
            break;
        case 2:
            *startpage = p_state->pdata.memcfg->eng2_startpage;
            *endpage = p_state->pdata.memcfg->eng2_endpage;
            break;
        default:
            break;
    }
}

static void lm8502_setup_engine_params(int eng, int *engine_pc, int *engine_prog_start_addr)
{
    switch(eng)
    {
        case 1:
            *engine_pc = ENGINE1_PC;
            *engine_prog_start_addr = ENG1_PROG_START_ADDR;
            break;
        case 2:
            *engine_pc = ENGINE2_PC;
            *engine_prog_start_addr = ENG2_PROG_START_ADDR;
            break;
        default:
            break;
    }
}

static int lm8502_start_engine(struct lm8502_device_state *p_state, int eng)
{
#ifdef AUTO_INCR_WRITE
    u8 reg, address_and_data [LM8502_INSTR_LEN_PER_PAGE * 2 + 1];
    int k = 0;
#else
    u8 upper, lower;
#endif
    int i, j, ret=0;
    int page_start=0, page_end=0, engine_pc=0, engine_prog_start_addr=0;
    // configure start/end page address and engine pc/start address
    lm8502_setup_page_sizes(p_state, eng, &page_start, &page_end);
    lm8502_setup_engine_params(eng, &engine_pc, &engine_prog_start_addr);

    //read and write to the pc needs to be in hold mode
    lm8502_config_engine_control_mode(p_state, eng, ENGINE_CNTRL1, ENGINE_CNTRL1_HOLD);

    //disable current engine
    lm8502_config_engine_control_mode(p_state, eng, ENGINE_CNTRL2, ENGINE_CNTRL2_DISABLE);

    // configure engine_pc, program start address and page number
    lm8502_i2c_write_reg(p_state->i2c_dev, engine_pc, page_start * 16);
    lm8502_i2c_write_reg(p_state->i2c_dev, engine_prog_start_addr, page_start * 16);
    lm8502_i2c_write_reg(p_state->i2c_dev, PROG_MEM_PAGE_SELECT, page_start);

    // configure load program mode
    lm8502_config_engine_control_mode(p_state, eng, ENGINE_CNTRL2, ENGINE_CNTRL2_LOAD);

#ifdef AUTO_INCR_WRITE
    // Enable the serial bus address auto increment
    lm8502_i2c_read_reg(p_state->i2c_dev, CONFIG, &reg);
    lm8502_i2c_write_reg(p_state->i2c_dev, CONFIG, reg | CONFIG_AUTO_INCR_ON);
#endif

    for (i = page_start; i <= page_end; i++) {

        /* Configure the memory page */
        lm8502_i2c_write_reg(p_state->i2c_dev, PROG_MEM_PAGE_SELECT, i);

    #ifdef AUTO_INCR_WRITE
        k = 0;
        // Assemble the data to be written
        address_and_data [k++] = PROG_MEM_START;

        for (j = 0; j < 16; j++)
        {
            // Arm has oppose upper 8 bits and lower 8 bits, needs to be reverted
            address_and_data [k++] = p_state->instruct[i*16 + j]>>8;
            address_and_data [k++] = p_state->instruct[i*16 + j];
        }

        // Write the entire page
        lm8502_i2c_write_reg_auto_incr (p_state->i2c_dev, address_and_data, k);

    #else
        for (j = 0; j < 16; j++)
        {
            // Arm has oppose upper 8 bits and lower 8 bits, needs to be reverted
            lower = p_state->instruct[i*16 + j];
            upper = p_state->instruct[i*16 + j]>>8;

            //printk(KERN_INFO "page=%d, offset=%d, instruction= %x, data= %x\n", i, j, upper, lower);

            lm8502_i2c_write_reg(p_state->i2c_dev, (PROG_MEM_START+(2*j)%32), upper);
            lm8502_i2c_write_reg(p_state->i2c_dev, (PROG_MEM_START+(2*j+1)%32), lower);

        }
    #endif

    }

#ifdef AUTO_INCR_WRITE
    // Disable the serial bus address auto increment
    lm8502_i2c_write_reg(p_state->i2c_dev, CONFIG, reg);
#endif


    // Run the program
    lm8502_config_engine_control_mode(p_state, eng, ENGINE_CNTRL1, ENGINE_CNTRL1_FREERUN);
    lm8502_config_engine_control_mode(p_state, eng, ENGINE_CNTRL2, ENGINE_CNTRL2_RUN);

    // allow the engine time to start before processing the next request
    msleep(1);

    return ret;
}

static int lm8502_eng_to_bitmask(int eng)
{
    if(eng == 1)
        return 1;
    else if(eng == 2)
        return 2;
    else
        return 0;
}

static int lm8502_stop_engine(struct lm8502_device_state *p_state, int eng)
{
    int ret=0;

    // hold execution and disable the engine
    // NOTE: The user space must make sure all LEDs are put back into the proper state
    // if an engine was terminated before it finished.
    lm8502_config_engine_control_mode(p_state, eng, ENGINE_CNTRL1, ENGINE_CNTRL1_HOLD);
    lm8502_config_engine_control_mode(p_state, eng, ENGINE_CNTRL2, ENGINE_CNTRL2_DISABLE);

    // allow the engine time to stop before processing the next request
    msleep(1);

    p_state->stop_engine_state |= lm8502_eng_to_bitmask(eng);

    if(p_state->stop_engine_state > 0)
    {
        wake_up_interruptible(&lm8502_wait_stop_engine);
    }

    return ret;
}

static void lm8502_process_command(struct work_struct *work)
{
    struct engine_cmd *command = container_of(work, struct engine_cmd, workitem);

    mutex_lock(&command->context->lock);

    switch(command->cmd)
    {
        case LM8502_START_ENGINE:
            lm8502_start_engine(command->context, command->engine);
            break;
        case LM8502_STOP_ENGINE:
            lm8502_stop_engine(command->context, command->engine);
            break;
        default:
            break;
    }

    mutex_unlock(&command->context->lock);

    kfree(command);
}

static void lm8502_allocate_workitem(struct lm8502_device_state *context, int engine, int cmd)
{
    struct engine_cmd *newcmd = kzalloc(sizeof(struct engine_cmd), GFP_KERNEL);

    newcmd->context = context;
    newcmd->engine = engine;
    newcmd->cmd = cmd;

    INIT_WORK(&newcmd->workitem, lm8502_process_command);
    schedule_work(&newcmd->workitem);
}

static void lm8502_mod_brightness(struct work_struct *work)
{
    struct lm8502_led_config *led_config =
        container_of(work, struct lm8502_led_config, brightness_work);
    struct lm8502_device_state *state = i2c_get_clientdata(led_config->client);

    int pwm_value;
    int i = 0;

    if(lm8502_convert_percent_to_pwm(led_config->brightness, &pwm_value) != 0)
    {
        printk(KERN_ERR "lm8502_mod_brightness: invalid brightness value (%d)\n", led_config->brightness);
        return;
    }

    mutex_lock(&state->lock);

    switch(led_config->hw_group)
    {
        case LED_HW_GRP_NONE:
            // Loop through each led within group_id
            for(i = 0; i < led_config->nleds; i++)
            {
                lm8502_i2c_write_reg(led_config->client, led_config->led_list[i].current_addr, pwm_value);
                printk(KERN_DEBUG"lm8502_mod_brightness: set LED%d brightness (%d) percent\n", 
                    (led_config->led_list[i].current_addr - 0x25), led_config->brightness);
            }
            break;
        /* HW Groups can change brightness for all LEDs with a single command */
        case LED_HW_GRP_1:
            lm8502_i2c_write_reg(led_config->client, GROUP_FADER1, pwm_value);
            break;
        case LED_HW_GRP_2:
            lm8502_i2c_write_reg(led_config->client, GROUP_FADER2, pwm_value);
            break;
        case LED_HW_GRP_3:
            lm8502_i2c_write_reg(led_config->client, GROUP_FADER3, pwm_value);
            break;
        default:
            break;
    }
    mutex_unlock(&state->lock);
    return;
}

/* Program current control register, ramp registers, blink
 * registers, and then turn on/off the LED regulators.
 */
static void lm8502_set_brightness(struct led_classdev *led_cdev,
                    enum led_brightness value)
{
    struct lm8502_led_config *led_config;
    struct lm8502_platform_data *pdata;
    struct lm8502_led_config *leds = NULL;

    struct i2c_client *client;

    /* Get the instance of LED configuration block. */
    led_config = container_of(led_cdev, struct lm8502_led_config, cdev);

    pdata = led_cdev->dev->platform_data;
    leds = pdata->leds;

    /* Sanity check. We can't be here without these set up correctly. */
    if ((!led_config) || (!led_config->client)) {
        printk(KERN_ERR "%s: Invalid LED. Can't set led brightness.\n",
               LM8502_I2C_DRIVER);
        return;
    }

    /* Get the handle to the I2C client. */
    client = led_config->client;

    /* Change the brightness */
    led_config->brightness = value;
    schedule_work(&led_config->brightness_work);

    return;
}

static void lm8502_vib_enable(struct timed_output_dev *dev, int value)
{
	struct lm8502_device_state *vib = container_of(dev,
            struct lm8502_device_state, timed_dev);
	unsigned long flags;

	hrtimer_cancel(&vib->vib_timer);
	spin_lock_irqsave(&vib->spinlock, flags);
#ifdef DEBUG
	printk("%s(parent:%s): vibrates %d msec\n",
			current->comm, current->parent->comm, value);
#endif
	if (value == 0)
		vib->state = 0;
	else {
		vib->state = 1;
		hrtimer_start(&vib->vib_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
	spin_unlock_irqrestore(&vib->spinlock, flags);
	queue_work_on(0, lm8502_vib_wq, &vib->work);
}

static int lm8502_vib_set(struct lm8502_device_state *vib, int on)
{
    int rc;
	if (on) {
        rc =lm8502_i2c_write_reg(vib->i2c_dev, HAPTIC_FEEDBACK_CTRL,
            0x02 + !!(vib->pdata.vib_invert_direction ^ vib->vib_direction));
	} else {
        rc = lm8502_i2c_write_reg(vib->i2c_dev, HAPTIC_FEEDBACK_CTRL, 0);
	}
    return rc;
}

static void lm8502_vib_update(struct work_struct *work)
{
	struct lm8502_device_state *vib = container_of(work,
            struct lm8502_device_state, work);

	lm8502_vib_set(vib, vib->state);
}


static int lm8502_vib_get_time(struct timed_output_dev *dev)
{
	struct lm8502_device_state *vib = container_of(dev,
            struct lm8502_device_state, timed_dev);

	if (hrtimer_active(&vib->vib_timer)) {
		ktime_t r = hrtimer_get_remaining(&vib->vib_timer);
		return r.tv.sec * 1000 + r.tv.nsec / 1000000;
	} else
		return 0;
}

static enum hrtimer_restart lm8502_vib_timer_func(struct hrtimer *timer)
{
	struct lm8502_device_state *vib = container_of(timer,
            struct lm8502_device_state, vib_timer);
#ifdef DEBUG
	printk("%s\n", __func__);
#endif
	vib->state = 0;
	queue_work_on(0, lm8502_vib_wq, &vib->work);
	return HRTIMER_NORESTART;
}


static void lm8502_notify(struct work_struct *work)
{
    u8 reg;
    int interrupt = 0;
    struct lm8502_device_state *p_state =
        container_of(work, struct lm8502_device_state, notify_work);

    mutex_lock(&p_state->lock);

    // clear interrrupt
    lm8502_i2c_read_reg(p_state->i2c_dev, STATUS, &reg);

    // figure out which engine fired the interrupt
    if(reg & 0x01)
        p_state->interrupt_state = 2;
    else if(reg & 0x02)
        p_state->interrupt_state = 1;
    else
        p_state->interrupt_state = 0;

    interrupt = p_state->interrupt_state;

    if(p_state->interrupt_state > 0)
    {
        wake_up_interruptible(&lm8502_wait_queue);
    }

    if(p_state->stopengine > 0)
    {
        // If the user space requested the engine be stopped after the next interrupt
        // then call lm8502_stop_engine directly and clear the appropriate bits in p_state->stopengine
        if( (p_state->stopengine & LM8502_STOP_ENGINE1) && (interrupt == 1))
        {
            p_state->stopengine = p_state->stopengine &~ LM8502_STOP_ENGINE1;
            lm8502_allocate_workitem(p_state, 1, LM8502_STOP_ENGINE);
        }
        if( (p_state->stopengine & LM8502_STOP_ENGINE2) && (interrupt == 2))
        {
            p_state->stopengine = p_state->stopengine &~ LM8502_STOP_ENGINE2;
            lm8502_allocate_workitem(p_state, 2, LM8502_STOP_ENGINE);
        }
    }

    mutex_unlock(&p_state->lock);
}

static irqreturn_t lm8502_isr(int irq, void *dev_id)
{
    struct lm8502_device_state *state = dev_id;
    schedule_work(&state->notify_work);
    return IRQ_HANDLED;
}

static void select_flash(struct i2c_client *client)
{
    //LED10 as switch between flash and vibrator
    lm8502_i2c_write_reg(client, D10_CURRENT_CTRL, 0xFF);
}

static void select_vibrator(struct i2c_client *client)
{
    //LED10 as switch between flash and vibrator
    lm8502_i2c_write_reg(client, D10_CURRENT_CTRL, 0);
}


static int lm8502_ioctl(struct inode *ip, struct file *fp,
             unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    struct lm8502_device_state *p_state = container_of(fp->f_op, struct lm8502_device_state, fops);
    struct lm8502_led_config *leds = NULL;
    int i,j;
    u8 reg;

    if(!p_state)
    {
        printk(KERN_ERR "lm8502_ioctl: invalid context\n");
        return -EINVAL;
    }

    switch(cmd)
    {
        case LM8502_READ_PWM:
        {
            struct lm8502_read_pwm pwm;
            u8 addr;
            if(copy_from_user(&pwm, (struct lm8502_read_pwm *)arg, sizeof(struct lm8502_read_pwm)))
            {
                printk(KERN_ERR "ERROR: lm8502_ioctl: LM8502_READ_PWM parameter is invalid\n");
                ret = -EINVAL;
                goto done;
            }

            addr = pwm.led + D1_CURRENT_CTRL - 1;
            if((addr < D1_CURRENT_CTRL) || (addr > D10_CURRENT_CTRL))
            {
                printk(KERN_ERR "ERROR: lm8502_ioctl: LM8502_READ_PWM addr is out of range\n");
                ret = -EINVAL;
                goto done;
            }

            lm8502_i2c_read_reg(p_state->i2c_dev, addr, &pwm.value);

            if(copy_to_user((void *)arg, &pwm, sizeof(struct lm8502_read_pwm)))
            {
                printk(KERN_ERR "ERROR: lm8502_ioctl: LM8502_READ_PWM failed to copy result\n");
                ret = -EINVAL;
            }

            break;

        }

        case LM8502_DOWNLOAD_MICROCODE:
            mutex_lock(&p_state->lock);
            if(copy_from_user(&p_state->instruct, (u16 *)arg, sizeof(u16)*INSTR_LEN))
            {
                printk(KERN_ERR "ERROR: lm8502_ioctl: LM8502_DOWNLOAD_MICROCODE parameter is invalid\n");
                ret = -EINVAL;
            }
            mutex_unlock(&p_state->lock);
            break;

        case LM8502_START_ENGINE:
        case LM8502_STOP_ENGINE:
            if ((u8)arg == 1 || (u8)arg == 2)
                lm8502_allocate_workitem(p_state, (int)arg, cmd);
            else
                ret = -EINVAL;
            break;
        case LM8502_CONFIG_MAX_CURRENT:
            if ((u8)arg > 4)
                ret = -EINVAL;
            /* Set current full-scale setting for LED output*/
            for (i = 0,leds = p_state->pdata.leds; i < p_state->pdata.nleds; i++)
            for (j = 0; j < leds[i].nleds; j++)
            {
                lm8502_i2c_read_reg(p_state->i2c_dev, leds[i].led_list[j].control_addr, &reg);
                reg = reg & (~0x18); //clear bit[4:3]
                reg = reg | ((u8)arg << 3); //set bit[4:3]
                lm8502_i2c_write_reg(p_state->i2c_dev, leds[i].led_list[j].control_addr, reg);
            }
            break;
        case LM8502_STOP_ENGINE_AFTER_INTERRUPT:
            // lm8502 will stop the engine execution if stopengine > 0
            mutex_lock(&p_state->lock);
            switch((u8)arg)
            {
                case 1:
                    p_state->stopengine |= LM8502_STOP_ENGINE1;
                    break;
                case 2:
                    p_state->stopengine |= LM8502_STOP_ENGINE2;
                    break;
                default:
                        ret = -EINVAL;
                    break;
            }
            mutex_unlock(&p_state->lock);
            break;

        case LM8502_WAIT_FOR_INTERRUPT:
            wait_event_interruptible(lm8502_wait_queue, p_state->interrupt_state > 0);

            mutex_lock(&p_state->lock);
            if(copy_to_user((void *)arg, &p_state->interrupt_state, sizeof(int)))
            {
                printk(KERN_ERR "ERROR: lm8502_ioctl: LM8502_WAIT_FOR_INTERRUPT parameter is invalid\n");
                ret = -EINVAL;
            }

            // clear internal interrupt state
            p_state->interrupt_state = 0;
            mutex_unlock(&p_state->lock);
            break;

        case LM8502_WAIT_FOR_ENGINE_STOPPED:
            wait_event_interruptible(lm8502_wait_stop_engine, p_state->stop_engine_state > 0);

            mutex_lock(&p_state->lock);
            if(copy_to_user((void *)arg, &p_state->stop_engine_state, sizeof(int)))
            {
                printk(KERN_ERR "ERROR: lm8502_ioctl: LM8502_WAIT_FOR_ENGINE_STOPPED parameter is invalid\n");
                ret = -EINVAL;
            }

            // clear internal interrupt state
            p_state->stop_engine_state = 0;
            mutex_unlock(&p_state->lock);
            break;

        case LM8502_CONFIGURE_MEMORY:
            mutex_lock(&p_state->lock);
            if(copy_from_user(p_state->pdata.memcfg, (void *)arg, sizeof(struct lm8502_memory_config)))
            {
                printk(KERN_ERR "ERROR: lm8502_ioctl: LM8502_CONFIGURE_MEMORY parameter is invalid\n");
                ret = -EINVAL;
            }
            mutex_unlock(&p_state->lock);
            break;

        default:
            break;
    }

done:
    return ret;

}

static int lm8502_open(struct inode *ip, struct file *fp)
{
    struct lm8502_device_state * p_state;
    int ret = 0;

    p_state = container_of(fp->f_op, struct lm8502_device_state, fops);

    if(!p_state)
    {
        printk("ERROR: lm8502_open: p_state is bad\n");
        return -EINVAL;
    }

    return ret;
}

static const struct file_operations lm8502_fops = {
    .open = lm8502_open,
    .ioctl = lm8502_ioctl,
};

int debug_reg_open(struct inode *inode, struct file *file)
{
    file->private_data = inode->i_private;
    return 0;
}

ssize_t debug_reg_read(struct file *file, char __user *userbuf,
	size_t count, loff_t *ppos)
{
	int rc, ret;
	u8 addr, value;
	u8 pdata[255];
	char *buf;
	char *tmpbuf = NULL;
	int line, rest;
	int i, j;
	struct lm8502_device_state *p_state;

	p_state = file->private_data;
	if (p_state == NULL) {
		printk(KERN_ERR "debug_reg_read,%x", (int)p_state);
		return -EINVAL;
	}

	/* Alloc memory */
	buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		goto done;
	}
	memset(buf, 0, PAGE_SIZE);

	addr = 0x00;
	/* Read 0xF1 registers */
	value = 0xF1;
	pdata[0] = 1;

	for (i = 0; i < value; i++) {
		rc = lm8502_i2c_read_reg(p_state->i2c_dev, addr+i, pdata+1+i);
		if (rc != 2) {
			pdata[0] = 0;
			break;
		}
	}

	if (pdata[0] == 0) {
		ret =  -EINVAL;
		goto done;
	} else {
		ret = 0;
		line = value / 0x10;
		rest = value % 0x10;

		if (line > 0) {
			for (i = 0; i < line; i++) {
				ret += snprintf(buf + ret, \
					PAGE_SIZE - ret, "reg%02x--%02x:   ", \
					addr + i*0x10, addr + (i+1)*0x10 - 1);

				for (j = 0; j < 0x10; j++) {
					ret += snprintf(buf + ret, \
						PAGE_SIZE - ret, \
						"%02x ", pdata[i*0x10+j+1]);
				}

				ret += snprintf(buf + ret, PAGE_SIZE - ret, \
					"\n");
			}
		}

		if (rest == 1) {
			ret += snprintf(buf + ret, PAGE_SIZE - ret, \
				"reg%02x:       %02x", line*0x10 + addr, \
				pdata[line*0x10+1]);
		} else if (rest > 1) {
				ret += snprintf(buf + ret, PAGE_SIZE - ret, \
					"reg%02x--%02x:  ", addr + line*0x10, \
					addr + count-1);
				for (i = 0; i < rest; i++) {
					ret += snprintf(buf + ret, \
						PAGE_SIZE - ret, \
						"%02x ", pdata[i+1]);
				}
		}

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");
	}

	ret =  simple_read_from_buffer(userbuf, count, ppos, buf, ret);
done:
	kfree(buf);
	kfree(tmpbuf);
	return ret;
}

ssize_t debug_reg_write(struct file *file, const char __user *ubuf,
	size_t count, loff_t *ppos)
{
	int rc, ret;
	u8 addr, value;
	struct lm8502_device_state *p_state;

	if (!ubuf) {
		printk(KERN_ERR "debug_reg_write buf=%s", ubuf);
		ret = -EINVAL;
		goto done;
	}

	p_state = file->private_data;
	if (p_state == NULL) {
		printk(KERN_ERR "debug_reg_write p_state=%x", (int)p_state);
		return -EINVAL;
	}

	addr = p_state->addr;
	value = p_state->value;

	if ((addr < 0) || (addr > 0xF0)) {
		printk(KERN_ERR "ERROR:debug_reg_write addr is out of range\n");
		ret = -EINVAL;
		goto done;
	}

	rc = lm8502_i2c_write_reg(p_state->i2c_dev, addr, value);
	if (rc != 1)
		ret = -EINVAL;
	else
		ret = 1;
done:
	return ret;
}

static const struct file_operations reg_fops = {
	.owner = THIS_MODULE,
	.open = debug_reg_open,
	.read = debug_reg_read,
	.write = debug_reg_write,
};

static int lm8502_i2c_probe(struct i2c_client *client, const struct i2c_device_id *dev_id)
{
    struct lm8502_device_state *state = NULL;
    struct lm8502_platform_data *pdata = client->dev.platform_data;
    struct lm8502_led_config *leds = NULL;
    int i, j, ret;
    u8 index;
    u8 reg;

    printk(KERN_DEBUG"LM8502 probe called\n");

    /* Check the platform data. */
    if (pdata == NULL) {
        printk(KERN_ERR "%s: missing platform data\n",
               LM8502_I2C_DRIVER);
        return -ENODEV;
    }

    /* Create the device state */
    state = kzalloc(sizeof(struct lm8502_device_state), GFP_KERNEL);
    if (!state) {
        return -ENOMEM;
    }

    /* Attach i2c_dev */
    state->i2c_dev = client;

	/* Attach driver data. */
	i2c_set_clientdata(client, state);
	lm8502_state = state;

    /* Attach platform data */
    memcpy(&state->pdata, pdata, sizeof(struct lm8502_platform_data));

    /*init the  vibrator or Flash select functions*/
    state->pdata.select_flash = select_flash;
    state->pdata.select_vibrator = select_vibrator;

    /* Init workq */
    INIT_WORK(&state->notify_work, lm8502_notify);

    /* Init mutex */
    mutex_init(&state->lock);

    /* Default interrupt state */
    state->interrupt_state = 0;
    state->stop_engine_state = 0;
    state->stopengine = 0;

    /* Enable interrupt */
    ret = gpio_request(pdata->interrupt_gpio, "LM8502 interrupt");
    if (ret != 0) {
        printk(KERN_ERR "LM8502: Failed to get GPIO for interrupt line.\n");
        goto err0;
    }
    ret = gpio_direction_input(pdata->interrupt_gpio);
    if (ret != 0) {
        printk(KERN_ERR "LM8502: Failed to set interrupt line for input.\n");
        goto err1;
    }
    ret = request_irq(gpio_to_irq(pdata->interrupt_gpio), lm8502_isr,
              IRQF_TRIGGER_FALLING, pdata->dev_name, (void *)state);
    if (ret < 0)
        goto err1;


    /* Enable the chip by setting the correct GPIO pin */
    ret = gpio_request(pdata->enable_gpio, "LM8502 enable");
    if (ret != 0) {
        printk(KERN_ERR "LM8502: Failed to get GPIO for enable line. Error code = %d\n", ret);
        goto err2;
    }
    ret = gpio_direction_output(pdata->enable_gpio, 1);
    if (ret != 0) {
        printk(KERN_ERR "LM8502: Failed to set enable line for output. Error code = %d\n", ret);
        goto err3;
    }
    gpio_set_value(pdata->enable_gpio, 1);

    // Reset lm8502 when power on
    lm8502_i2c_write_reg(client, RESET, 0xFF );
    mdelay(50);

    /* Enable the chip in software by flipping the CHIP_EN bit*/
    lm8502_i2c_read_reg(client, ENGINE_CNTRL1, &reg);
    lm8502_i2c_write_reg(client, ENGINE_CNTRL1, (reg|0x40) );

    /* Configure power savings mode and enable boost,select pwm input*/
    lm8502_i2c_read_reg(client, MISC, &reg);
    lm8502_i2c_write_reg(client, MISC, reg | (pdata->power_mode) | (1<<3) | (1<<1));

    /* Create misc device */
    memcpy(&state->fops, &lm8502_fops, sizeof(struct file_operations));
    state->mdev.name = "lm8502";
    state->mdev.minor = MISC_DYNAMIC_MINOR;
    state->mdev.fops = &state->fops;

    spin_lock_init(&state->spinlock);
    INIT_WORK(&state->work, lm8502_vib_update);
    lm8502_vib_wq = create_singlethread_workqueue("lm8502_vib_wq");
    if (!lm8502_vib_wq) {
        printk("%s: create_singlethread_workqueue failed\n", __func__);
    }

    hrtimer_init(&state->vib_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    state->vib_timer.function = lm8502_vib_timer_func;

    state->timed_dev.name = "vibrator";
    state->timed_dev.get_time = lm8502_vib_get_time;
    state->timed_dev.enable = lm8502_vib_enable;

    if(timed_output_dev_register(&state->timed_dev) < 0)
        printk(KERN_ERR "LM8502: unable to register timed_output dev\n");


    ret = misc_register(&state->mdev);
    if(ret) {
        printk(KERN_ERR "LM8502: Failed to create /dev/lm8502\n");
        goto err4;
    }


    /* Register to led class, process platform data and assign default values*/
    for (i = 0,leds = pdata->leds; i < pdata->nleds; i++)
    {
        /* check to see if this group can be put into a single hw group */
        if(leds[i].hw_group != LED_HW_GRP_NONE)
        {
            /* assign hardware groups to each led in the group */
            for(j = 0; j < leds[i].nleds; j++)
            {
                lm8502_i2c_read_reg(client, leds[i].led_list[j].control_addr, &reg);
                reg = reg & (~0xC0); //clear bit[7:6]
                reg = reg | (leds[i].hw_group << 6); //set bit[7:6]
                lm8502_i2c_write_reg(client, leds[i].led_list[j].control_addr, reg);
            }

        }

        /* Set current full-scale setting for LED output*/
        for(j = 0; j < leds[i].nleds; j++)
        {
            lm8502_i2c_read_reg(client, leds[i].led_list[j].control_addr, &reg);
            reg = reg & (~0x18); //clear bit[4:3]
            reg = reg | (leds[i].default_max_current << 3); //set bit[4:3]
            lm8502_i2c_write_reg(client, leds[i].led_list[j].control_addr, reg);
        }

        INIT_WORK(&leds[i].brightness_work, lm8502_mod_brightness);

        if(leds[i].default_state == LED_ON)
        {
            printk(KERN_DEBUG"LM8502: Turning %s on to default %d duty cycle\n",
                leds[i].cdev.name, leds[i].default_brightness);

            leds[i].cdev.brightness = leds[i].default_brightness;
            leds[i].brightness = leds[i].default_brightness;
            schedule_work(&leds[i].brightness_work);
        }

        leds[i].cdev.brightness_set = lm8502_set_brightness;

        /* Save the i2c client information for each led. */
        leds[i].client = client;

        /* Register this LED instance to the LED class. */
        ret = led_classdev_register(&client->dev, &leds[i].cdev);
        if (ret < 0)
            goto err5;
    }

    /* Create sysfs entries for the vibrator portion of the part */
    state->vib_class = class_create(THIS_MODULE, "vibrator");
    if (!state->vib_class){
        ret = -ENOMEM;
        printk(KERN_ERR "Unable to create vibrator class node\n");
        goto err6;
    }

    state->vib_class_dev = device_create(state->vib_class, &client->dev, 0, state, "vib0");
    if (!state->vib_class_dev) {
        ret = -ENOMEM;
        printk(KERN_ERR "Unable to create vibrator class device node\n");
        goto err7;
    }

    if ((ret = device_create_file(state->vib_class_dev, &dev_attr_vib_enable)) < 0) {
        printk(KERN_ERR "Unable to create vibrator enable node\n");
        goto err8;
    }

    if ((ret = device_create_file(state->vib_class_dev, &dev_attr_vib_duty_cycle)) < 0) {
        printk(KERN_ERR "Unable to create vibrator duty cycle node\n");
        goto err9;
    }

    if ((ret = device_create_file(state->vib_class_dev, &dev_attr_vib_direction)) < 0) {
        printk(KERN_ERR "Unable to create vibrator direction node\n");
        goto err10;
    }

    if ((ret = device_create_file(state->vib_class_dev, &dev_attr_vib_start)) < 0) {
        printk(KERN_ERR "Unable to create vibrator start node\n");
        goto err11;
    }

    /* Program the default vibrator settings as specified in the board file */
    state->vib_direction = pdata->vib_default_direction;
    state->vib_duty_cycle = pdata->vib_default_duty_cycle;
    lm8502_i2c_write_reg(state->i2c_dev, HAPTIC_PWM_DUTY_CYCLE, (state->vib_duty_cycle * 255) / 100);

    /* Create sysfs entries for the flash portion of the part */
    state->flash_class = class_create(THIS_MODULE, "flash");
    if (!state->flash_class){
        ret = -ENOMEM;
        printk(KERN_ERR "Unable to create flash class node\n");
        goto err12;
    }

    state->flash_class_dev = device_create(state->flash_class, &client->dev, 0, state, "flash0");
    if (!state->flash_class_dev) {
        ret = -ENOMEM;
        printk(KERN_ERR "Unable to create flash class device node\n");
        goto err13;
    }

    if ((ret = device_create_file(state->flash_class_dev, &dev_attr_flash_enable)) < 0) {
        printk(KERN_ERR "Unable to create flash current node\n");
        goto err14;
    }

    if ((ret = device_create_file(state->flash_class_dev, &dev_attr_flash_current)) < 0) {
        printk(KERN_ERR "Unable to create flash current node\n");
        goto err15;
    }

    if ((ret = device_create_file(state->flash_class_dev, &dev_attr_flash_duration)) < 0) {
        printk(KERN_ERR "Unable to create flash duration node\n");
        goto err16;
    }

    if ((ret = device_create_file(state->flash_class_dev, &dev_attr_torch_current)) < 0) {
        printk(KERN_ERR "Unable to create flash duration node\n");
        goto err17;
    }

    if ((ret = device_create_file(state->flash_class_dev, &dev_attr_flash_or_torch_start)) < 0) {
        printk(KERN_ERR "Unable to create flash fire node\n");
        goto err18;
    }

    /* Program the default flash settings as specified in the board file */
    index = lm8502_get_closest_flash_current(pdata->flash_default_current);
    state->flash_current = flash_brightness_map[index].current_value;
    lm8502_i2c_write_reg(state->i2c_dev, FLASH_BRIGHTNESS, STROBE_TIMEOUT | (flash_brightness_map[index].code << 3));

    state->flash_duration = pdata->flash_default_duration;
    lm8502_i2c_write_reg(state->i2c_dev, FLASH_DURATION, state->flash_duration >> 5);

    index = lm8502_get_closest_torch_current(pdata->torch_default_current);
    state->torch_current = torch_brightness_map[index].current_value;
    lm8502_i2c_write_reg(state->i2c_dev, TORCH_BRIGHTNESS, torch_brightness_map[index].code << 3);

	/* Create debugfs entries. */
	state->debug_dir = debugfs_create_dir("lm8502", NULL);
	if ((int)(state->debug_dir) == -ENODEV) {
		/* debugfs is not enabled. */
		printk(KERN_WARNING "debugfs not enabled in kernel\n");

	} else if (state->debug_dir == NULL) {
		printk(KERN_WARNING "error creating debugfs dir\n");

	} else {
		debugfs_create_x8("addr", S_IRUSR|S_IWUSR, \
			state->debug_dir, &(state->addr));
		debugfs_create_x8("value", S_IRUSR|S_IWUSR, \
			state->debug_dir, &(state->value));
		debugfs_create_file("lm8502_reg", 0666, \
			state->debug_dir, state, &reg_fops);
	}

    return 0;

err18:
    device_remove_file(state->flash_class_dev, &dev_attr_torch_current);
err17:
    device_remove_file(state->flash_class_dev, &dev_attr_flash_duration);
err16:
    device_remove_file(state->flash_class_dev, &dev_attr_flash_current);
err15:
    device_remove_file(state->flash_class_dev, &dev_attr_flash_enable);
err14:
    device_unregister(state->flash_class_dev);
err13:
    class_destroy(state->flash_class);
err12:
    device_remove_file(state->vib_class_dev, &dev_attr_vib_start);
err11:
    device_remove_file(state->vib_class_dev, &dev_attr_vib_direction);
err10:
    device_remove_file(state->vib_class_dev, &dev_attr_vib_duty_cycle);
err9:
    device_remove_file(state->vib_class_dev, &dev_attr_vib_enable);
err8:
    device_unregister(state->vib_class_dev);
err7:
    class_destroy(state->vib_class);
err6:
err5:
    /* Unregister previously created attribute files and return error. */
    if (i > 1)
    {
        for (i = i - 1; i >= 0; i--)
        {
            led_classdev_unregister(&leds[i].cdev);
        }
    }
err4:
    misc_deregister(&state->mdev);
err3:
    gpio_free(pdata->enable_gpio);
err2:
    free_irq(gpio_to_irq(pdata->interrupt_gpio), state);
err1:
    gpio_free(pdata->interrupt_gpio);
err0:
    kfree(state);

    return (ret);
}

static int lm8502_i2c_remove(struct i2c_client *client)
{
    struct lm8502_device_state *state = i2c_get_clientdata(client);
    struct lm8502_platform_data *pdata = client->dev.platform_data;
    struct lm8502_led_config *leds = NULL;
    int i;

    cancel_work_sync(&state->notify_work);

    cancel_work_sync(&state->work);
    hrtimer_cancel(&state->vib_timer);
    timed_output_dev_unregister(&state->timed_dev);

    device_remove_file(state->flash_class_dev, &dev_attr_flash_or_torch_start);
    device_remove_file(state->flash_class_dev, &dev_attr_torch_current);
    device_remove_file(state->flash_class_dev, &dev_attr_flash_duration);
    device_remove_file(state->flash_class_dev, &dev_attr_flash_current);
    device_remove_file(state->flash_class_dev, &dev_attr_flash_enable);
    device_unregister(state->flash_class_dev);
    class_destroy(state->flash_class);

    device_remove_file(state->vib_class_dev, &dev_attr_vib_start);
    device_remove_file(state->vib_class_dev, &dev_attr_vib_direction);
    device_remove_file(state->vib_class_dev, &dev_attr_vib_duty_cycle);
    device_remove_file(state->vib_class_dev, &dev_attr_vib_enable);
    device_unregister(state->vib_class_dev);
    class_destroy(state->vib_class);

    debugfs_remove_recursive(state->debug_dir);

#ifdef CONFIG_LEDS_LM8502_DBG
    lm8502_dbg_unregister_i2c_client(state->i2c_dev);
#endif // CONFIG_LEDS_LM8502_DBG

    if (pdata) {
        leds = pdata->leds;
        for (i = 0; i < pdata->nleds; i++)
            led_classdev_unregister(&leds[i].cdev);
    }

    misc_deregister(&state->mdev);

    gpio_free(pdata->enable_gpio);
    free_irq(gpio_to_irq(pdata->interrupt_gpio), state);
    gpio_free(pdata->interrupt_gpio);

    i2c_set_clientdata(client, NULL);

    kfree(state);

    return 0;
}

#ifdef CONFIG_PM
static u8 enable_reg = 0;
static int lm8502_i2c_suspend(struct i2c_client *client, pm_message_t pm_state)
{
    struct lm8502_device_state *state = i2c_get_clientdata(client);
    struct lm8502_platform_data *pdata = client->dev.platform_data;
    struct lm8502_led_config *leds = NULL;
    u8 reg;
    int i;

    if (state->suspended) {
        return 0;
    }

    /* cancel any pending work */
    cancel_work_sync(&state->notify_work);

    hrtimer_cancel(&state->vib_timer);
    cancel_work_sync(&state->work);

    if (pdata) {
        leds = pdata->leds;
        for (i = 0; i < pdata->nleds; i++)
            led_classdev_suspend(&leds[i].cdev);
    }

    if (state->vib_start)
        lm8502_i2c_write_reg(state->i2c_dev, HAPTIC_FEEDBACK_CTRL, 0);

    if (state->flash_start && !state->is_flash_mode) {
        lm8502_i2c_read_reg(state->i2c_dev, FLASH_BRIGHTNESS, &reg);
        lm8502_i2c_write_reg(state->i2c_dev, FLASH_BRIGHTNESS, reg & ~0x03);
    }

    lm8502_i2c_read_reg(state->i2c_dev, ENGINE_CNTRL1, &enable_reg);
    if(((enable_reg & (3 << 4)) == ENGINE_CNTRL1_HOLD) && ((enable_reg & (3 << 2)) == ENGINE_CNTRL1_HOLD) ){
        // Disable chip on suspend
        lm8502_i2c_read_reg(state->i2c_dev, ENGINE_CNTRL1, &enable_reg);
        lm8502_i2c_write_reg(state->i2c_dev, ENGINE_CNTRL1, 0);
    }

    state->suspended = 1;

    return 0;
}

static int lm8502_i2c_resume(struct i2c_client *client)
{
    struct lm8502_device_state *state = i2c_get_clientdata(client);
    struct lm8502_platform_data *pdata = client->dev.platform_data;
    struct lm8502_led_config *leds = NULL;
    u8 reg;
    int i;

    if (!state->suspended) {
        return 0;
    }

    if((((enable_reg & (3 << 4)) == ENGINE_CNTRL1_HOLD) && ((enable_reg & (3 << 2)) == ENGINE_CNTRL1_HOLD) )){
        //Enable chip and set to voltage mode on resume
        lm8502_i2c_write_reg(state->i2c_dev, ENGINE_CNTRL1, (enable_reg));
        lm8502_i2c_write_reg(state->i2c_dev, TORCH_BRIGHTNESS, 0x4);
    }

    if (pdata) {
        leds = pdata->leds;
        for (i = 0; i < pdata->nleds; i++)
            led_classdev_resume(&leds[i].cdev);
    }

    if (state->vib_start)
        lm8502_i2c_write_reg(state->i2c_dev, HAPTIC_FEEDBACK_CTRL, 0x02 + state->vib_direction);

    if (state->flash_start && !state->is_flash_mode) {
        lm8502_i2c_read_reg(state->i2c_dev, FLASH_BRIGHTNESS, &reg);
        lm8502_i2c_write_reg(state->i2c_dev, FLASH_BRIGHTNESS, reg | TORCH_MODE);
    }

    state->suspended = 0;

    return 0;
}
#else
#define lm8502_i2c_suspend NULL
#define lm8502_i2c_resume  NULL
#endif

static const struct i2c_device_id lm8502_ids[] = {
    {LM8502_I2C_DRIVER, },
    {},
};

MODULE_DEVICE_TABLE(i2c, lm8502_ids);

static struct i2c_driver lm8502_i2c_driver = {
    .driver = {
        .name = LM8502_I2C_DRIVER,
        .owner = THIS_MODULE,
    },
    .id_table   = lm8502_ids,
    .probe      = lm8502_i2c_probe,
    .remove     = lm8502_i2c_remove,
    .suspend    = lm8502_i2c_suspend,
    .resume     = lm8502_i2c_resume,
};

static int __init lm8502_module_init(void)
{
    int ret;

    printk(KERN_DEBUG "LM8502 module init called\n");
    ret = i2c_add_driver(&lm8502_i2c_driver);
    return (ret);
}

static void __exit lm8502_module_exit(void)
{
    i2c_del_driver(&lm8502_i2c_driver);
}

module_init(lm8502_module_init);
module_exit(lm8502_module_exit);

MODULE_DESCRIPTION("National LM8502 driver");
MODULE_LICENSE("GPL");
