/*
 *  include/linux/i2c_lm8502_led.h
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
 *          Amon Xie(amon.xie@palm.com)
 *
 */

#ifndef _LM8502_H
#define _LM8502_H

#include <linux/i2c.h>
#include <linux/leds.h>

#define LED_OFF     0
#define LED_ON      1
#define MISC_POWER_SAVE_ON  (1 << 5)
#define MISC_POWER_SAVE_OFF (0 << 5)

#define LM8502_I2C_DEVICE       "LM8502"
#define LM8502_I2C_DRIVER       "LM8502"
#define LM8502_I2C_ADDR     0x33

/*ioctl codes */
#define LM8502_DOWNLOAD_MICROCODE       1
#define LM8502_START_ENGINE         9
#define LM8502_STOP_ENGINE          3
#define LM8502_WAIT_FOR_INTERRUPT       4
#define LM8502_CONFIGURE_MEMORY         5
#define LM8502_STOP_ENGINE_AFTER_INTERRUPT  6
#define LM8502_READ_PWM             7
#define LM8502_WAIT_FOR_ENGINE_STOPPED      8
#define LM8502_CONFIG_MAX_CURRENT   10

/* lm8502 LED platform data structure */
enum {
    LED_GRP_1 = 0,
    LED_GRP_2,
    LED_GRP_3,
    LED_GRP_4,
    LED_GRP_5,
    LED_GRP_6,
    LED_GRP_7,
    LED_GRP_8,
    LED_GRP_9,
    LED_GRP_10,
};

enum {
    LED_HW_GRP_NONE = 0,
    LED_HW_GRP_1,
    LED_HW_GRP_2,
    LED_HW_GRP_3,
};

enum {
    LED_NONE = 0,
    LED_RED,
    LED_GREEN,
    LED_BLUE,
    LED_WHITE,
};

struct lm8502_read_pwm {
    u8 led;
    u8 value;
};

struct led_cfg {
    int type;
    u8 current_addr;
    u8 control_addr;
};


struct lm8502_memory_config {
    int eng1_startpage;
    int eng1_endpage;
    int eng2_startpage;
    int eng2_endpage;
};

struct lm8502_led_config {
    struct led_classdev cdev;
    struct i2c_client *client;
    struct led_cfg *led_list;
    struct work_struct brightness_work;
    int nleds;
    int brightness;
    int group_id;
    int hw_group;
    u8 default_max_current;
    int default_brightness;
    int default_state;
};

struct lm8502_platform_data {
    int enable_gpio;
    int interrupt_gpio;
    int strobe_gpio;
    u8 vib_default_duty_cycle;
    u8 vib_default_direction;
    u8 vib_invert_direction;
    u16 flash_default_duration;
    u16 flash_default_current;
    u16 torch_default_current;
    void (*select_flash) (struct i2c_client* client);
    void (*select_vibrator) (struct i2c_client* client);
    struct lm8502_led_config *leds;
    struct lm8502_memory_config *memcfg;
    int nleds;
    u8 power_mode;
    char *dev_name;
};

int lm8502_set_current(uint8_t is_flash_mode, uint32_t mA);
#endif // _LM8502_H
