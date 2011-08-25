/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */
#include <linux/bitops.h>
#include <linux/io.h>
#include <mach/msm_iomap.h>
#include "gpiomux.h"

#define GPIO_CFG(n)    (MSM_TLMM_BASE + 0x1000 + (0x10 * n))
#define GPIO_IN_OUT(n) (MSM_TLMM_BASE + 0x1004 + (0x10 * n))

void __msm_gpiomux_write(unsigned gpio, struct gpiomux_setting val)
{
	uint32_t bits;

	bits = (val.drv << 6) | (val.func << 2) | val.pull;
	if (val.func == GPIOMUX_FUNC_GPIO) {
		bits |= val.dir > GPIOMUX_IN ? BIT(9) : 0;
		writel(val.dir == GPIOMUX_OUT_HIGH ? BIT(1) : 0,
			GPIO_IN_OUT(gpio));
	}
	writel(bits, GPIO_CFG(gpio));
}

void __msm_gpiomux_read(unsigned gpio, struct gpiomux_setting *set)
{
	uint32_t bits;

	if (!set)
		return;

	bits = readl(GPIO_CFG(gpio));
	set->dir  = (bits >> 9) & 0x1;
	set->func = (bits >> 2) & 0xF;
	set->drv  = (bits >> 6) & 0x7;
	set->pull = (bits) & 0x3;
	
	if (set->func == GPIOMUX_FUNC_GPIO && set->dir > GPIOMUX_IN ) {
		if (readl(GPIO_IN_OUT(gpio)) & BIT(1))
			set->dir = GPIOMUX_OUT_HIGH;
		else 
			set->dir = GPIOMUX_OUT_LOW;
	}
}



