/*
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
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

#ifndef __ARCH_ARM_MACH_MSM_GPIO_HW_H
#define __ARCH_ARM_MACH_MSM_GPIO_HW_H

#include <mach/msm_iomap.h>

/* see 80-VA736-2 Rev C pp 695-751
**
** These are actually the *shadow* gpio registers, since the
** real ones (which allow full access) are only available to the
** ARM9 side of the world.
**
** Since the _BASE need to be page-aligned when we're mapping them
** to virtual addresses, adjust for the additional offset in these
** macros.
*/

#if defined(CONFIG_ARCH_MSM7X30)
#define GPIO1_REG(off) (MSM_GPIO1_BASE + (off))
#define GPIO2_REG(off) (MSM_GPIO2_BASE + 0x400 + (off))
#else
#define GPIO1_REG(off) (MSM_GPIO1_BASE + 0x800 + (off))
#define GPIO2_REG(off) (MSM_GPIO2_BASE + 0xC00 + (off))
#endif

#if defined(CONFIG_ARCH_QSD8X50)
#include "gpio_hw-8xxx.h"
#elif defined(CONFIG_ARCH_MSM7X30)
#include "gpio_hw-7x30.h"
#else
#include "gpio_hw-7xxx.h"
#endif

#endif
