/* arch/arm/mach-msm/gpio_chip.h
 *
 * Copyright (C) 2007 Google, Inc.
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

#ifndef _LINUX_GPIO_CHIP_H
#define _LINUX_GPIO_CHIP_H

#include <linux/list.h>

#ifndef CONFIG_GPIOLIB
#define gpio_chip goog_gpio_chip
#endif

struct goog_gpio_chip {
	struct list_head list;
	struct gpio_state *state;

	struct device	*dev;

	unsigned int start;
	unsigned int end;

	int (*configure)(struct goog_gpio_chip *chip,
			unsigned int gpio,
			unsigned long flags);
	int (*get_irq_num)(struct goog_gpio_chip *chip,
			unsigned int gpio,
			unsigned int *irqp,
			unsigned long *irqnumflagsp);
	int (*read)(struct goog_gpio_chip *chip, unsigned int gpio);
	int (*write)(struct goog_gpio_chip *chip,
		unsigned int gpio,
		unsigned on);
	int (*read_detect_status)(struct goog_gpio_chip *chip,
				unsigned int gpio);
	int (*clear_detect_status)(struct goog_gpio_chip *chip,
				unsigned int gpio);
};

struct msm_gpio_regs {
	void __iomem *out;
	void __iomem *in;
	void __iomem *int_status;
	void __iomem *int_clear;
	void __iomem *int_en;
	void __iomem *int_edge;
	void __iomem *int_pos;
	void __iomem *oe;
};

#define MSM_GPIO_BROKEN_INT_CLEAR 1

struct msm_gpio_chip {
	struct goog_gpio_chip        chip;
	struct msm_gpio_regs    regs;
#if MSM_GPIO_BROKEN_INT_CLEAR
	unsigned                int_status_copy;
#endif
	unsigned int            both_edge_detect;
	unsigned int            int_enable[2]; /* 0: awake, 1: sleep */
};

int register_gpio_chip(struct goog_gpio_chip *gpio_chip);

#endif
