/* linux/arch/arm/mach-msm/devices.h
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ARCH_ARM_MACH_MSM_DEVICES_H
#define __ARCH_ARM_MACH_MSM_DEVICES_H

#include "clock.h"

extern struct platform_device msm_device_uart1;
extern struct platform_device msm_device_uart2;
extern struct platform_device msm_device_uart3;

extern struct platform_device msm_device_uart_dm1;
extern struct platform_device msm_device_uart_dm2;
extern struct platform_device msm_device_uart_dm12;

extern struct platform_device msm_device_sdc1;
extern struct platform_device msm_device_sdc2;
extern struct platform_device msm_device_sdc3;
extern struct platform_device msm_device_sdc4;

extern struct platform_device msm_device_hsusb_otg;
extern struct platform_device msm_device_hsusb_peripheral;
extern struct platform_device msm_device_gadget_peripheral;
extern struct platform_device msm_device_hsusb_host;
extern struct platform_device msm_device_hsusb_host2;

extern struct platform_device msm_device_otg;

extern struct platform_device msm_device_i2c;

extern struct platform_device msm_device_i2c_2;

extern struct platform_device qup_device_i2c;

extern struct platform_device msm_gsbi3_qup_i2c_device;
extern struct platform_device msm_gsbi4_qup_i2c_device;
extern struct platform_device msm_gsbi7_qup_i2c_device;
extern struct platform_device msm_gsbi8_qup_i2c_device;
extern struct platform_device msm_gsbi9_qup_i2c_device;

extern struct platform_device msm_device_smd;
extern struct platform_device msm_device_dmov;

extern struct platform_device msm_device_nand;

extern struct platform_device msm_device_tssc;

extern struct platform_device msm_rotator_device;

extern struct platform_device msm_device_tsif;

extern struct platform_device msm_device_ssbi1;
extern struct platform_device msm_device_ssbi2;
extern struct platform_device msm_device_ssbi3;
extern struct platform_device msm_device_ssbi6;
extern struct platform_device msm_device_ssbi7;

extern struct platform_device msm_gsbi1_qup_spi_device;

extern struct platform_device msm_device_vidc_720p;

extern struct clk msm_clocks_7x01a[];
extern unsigned msm_num_clocks_7x01a;

extern struct clk msm_clocks_7x25[];
extern unsigned msm_num_clocks_7x25;

extern struct clk msm_clocks_7x27[];
extern unsigned msm_num_clocks_7x27;

extern struct clk msm_clocks_7x30[];
extern unsigned msm_num_clocks_7x30;

extern struct clk msm_clocks_8x50[];
extern unsigned msm_num_clocks_8x50;

extern struct clk msm_clocks_8x60[];
extern unsigned msm_num_clocks_8x60;

void __init msm_fb_register_device(char *name, void *data);
void __init msm_camera_register_device(void *, uint32_t, void *);
extern struct platform_device msm_device_touchscreen;

#ifdef CONFIG_GPIOLIB
#define MSM7200A_GPIO_PLATFORM_DATA(ix, begin, end, irq)		\
	[ix] = {							\
		.gpio_base		= begin,			\
		.ngpio			= end - begin + 1,		\
		.irq_base		= MSM_GPIO_TO_INT(begin),	\
		.irq_summary		= irq,				\
		.latch_level_irqs	= false,			\
		.regs = {						\
			.in		= GPIO_IN_ ## ix,		\
			.out		= GPIO_OUT_ ## ix,		\
			.oe		= GPIO_OE_ ## ix,		\
			.int_status	= GPIO_INT_STATUS_ ## ix,	\
			.int_clear	= GPIO_INT_CLEAR_ ## ix,	\
			.int_en		= GPIO_INT_EN_ ## ix,		\
			.int_edge	= GPIO_INT_EDGE_ ## ix,		\
			.int_pos	= GPIO_INT_POS_ ## ix,		\
		},							\
	}

#define MSM7200A_GPIO_DEVICE(ix, pdata)			\
	{						\
		.name		= "msm7200a-gpio",	\
		.id		= ix,			\
		.num_resources	= 0,			\
		.dev = {				\
			.platform_data = &pdata[ix],	\
		},					\
	}

extern struct platform_device msm_gpio_devices[];
#endif

#endif
