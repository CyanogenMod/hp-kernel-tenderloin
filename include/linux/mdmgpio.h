/* Copyright (c) 2010, Hewlett-Packard Development Company, L.P. All rights reserved.
 *
 * mdmgpio.h - Linux kernel modules for modem gpio control
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __LINUX_MDMGPIO_H
#define __LINUX_MDMGPIO_H


struct mdmgpio_platform_data {
	int uim_cd_gpio;
	int mdm_disable_gpio;
	int (*get_gpio_value)(int gpionum);
	int (*set_gpio_value)(int gpionum, int value);
	int (*mdm_poweron)(int on);
	int (*mdm_poweron_status)(void);
};
#endif