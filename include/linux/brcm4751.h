/* Copyright (c) 2010, Hewlett-Packard Development Company, L.P. All rights reserved.
 *
 * brcm4751.h - Linux kernel modules for broadcom GPS module 4751
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

#ifndef __LINUX_BRCM4751_H
#define __LINUX_BRCM4751_H

struct brcm4751_cfg {
	int regpu_sys_gpio;
	int reset_sys_gpio;
};

struct brcm4751_platform_data {
	int (*config)(bool, struct brcm4751_cfg *);
};

#endif
