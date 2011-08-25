/*
 *  leds-lm8502.h
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

#ifndef _LEDS_LM8502_H
#define _LEDS_LM8502_H


/* LED program engine */
#define INSTR_LEN       96
#define LM8502_INSTR_LEN_PER_PAGE 16
#define LM8502_STOP_ENGINE1 0x01
#define LM8502_STOP_ENGINE2 0x02

/* LM8502 i2c Registers */
#define ENGINE_CNTRL1       0x00
#define ENGINE_CNTRL2       0x01
#define GROUP_FADING1       0x02
#define GROUP_FADING2       0x03
#define GROUP_FADING3       0x04

#define D1_CONTROL      0x06
#define D2_CONTROL      0x07
#define D3_CONTROL      0x08
#define D4_CONTROL      0x09
#define D5_CONTROL      0x0A
#define D6_CONTROL      0x0B
#define D7_CONTROL      0x0C
#define D8_CONTROL      0x0D
#define D9_CONTROL      0x0E
#define D10_CONTROL     0x0F

#define HAPTIC_CONTROL      0x10

#define ALS_CONTROL         0x11
#define ZLINE0          0x12
#define ZLINE1          0x13
#define ZLINE2          0x14
#define ZLINE3          0x15
#define TARGET_LIGHT_Z0     0x16
#define TARGET_LIGHT_Z1     0x17
#define TARGET_LIGHT_Z2     0x18
#define TARGET_LIGHT_Z3     0x19
#define TARGET_LIGHT_Z4     0x1A
#define ALS_START_VALUE     0x1B
#define DBC_CONTROL     0x1D

#define HAPTIC_FEEDBACK_CTRL    0x21
#define HAPTIC_PWM_DUTY_CYCLE   0x22

#define D1_CURRENT_CTRL     0x26
#define D2_CURRENT_CTRL     0x27
#define D3_CURRENT_CTRL     0x28
#define D4_CURRENT_CTRL     0x29
#define D5_CURRENT_CTRL     0x2A
#define D6_CURRENT_CTRL     0x2B
#define D7_CURRENT_CTRL     0x2C
#define D8_CURRENT_CTRL     0x2D
#define D9_CURRENT_CTRL     0x2E
#define D10_CURRENT_CTRL    0x2F

#define ADAPT_FLASH_CTRL    0x35
#define MISC            0x36

#define ENGINE1_PC      0x37
#define ENGINE2_PC      0x38

#define STATUS          0x3A
#define INT         0x3B
#define I2C_VARIABLE        0x3C
#define RESET           0x3D

#define LED_TEST_CONTROL    0x41
#define LED_TEST_ADC        0x42

#define GROUP_FADER1        0x48
#define GROUP_FADER2        0x49
#define GROUP_FADER3        0x4A

#define ENG1_PROG_START_ADDR    0x4C
#define ENG2_PROG_START_ADDR    0x4D
#define PROG_MEM_PAGE_SELECT    0x4F

#define PROG_MEM_START      0x50
#define PROG_MEM_END        0x6F

#define TORCH_BRIGHTNESS    0xA0
#define FLASH_BRIGHTNESS    0xB0
#define FLASH_DURATION      0xC0
#define FLAG_REGISTER       0xD0
#define CONFIG_REG1     0xE0
#define CONFIG_REG2     0xF0

#define ENGINE_CNTRL_ENG1_SHIFT 4
#define ENGINE_CNTRL_ENG2_SHIFT 2

#define ENGINE_CNTRL1_HOLD  0
#define ENGINE_CNTRL1_STEP  1
#define ENGINE_CNTRL1_FREERUN   2
#define ENGINE_CNTRL1_EXECONCE  3

#define ENGINE_CNTRL2_DISABLE   0
#define ENGINE_CNTRL2_LOAD  1
#define ENGINE_CNTRL2_RUN   2
#define ENGINE_CNTRL2_HALT  3

#define MISC_POWER_SAVE_ON  (1 << 5)
#define MISC_POWER_SAVE_OFF (0 << 5)

#define STROBE_TIMEOUT      (1 << 7)

#define FLASH_MODE      (3 << 0)
#define TORCH_MODE      (2 << 0)

#endif // _LEDS_LM8502_H
