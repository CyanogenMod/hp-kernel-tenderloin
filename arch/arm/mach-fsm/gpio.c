/* linux/arch/arm/mach-msm/gpio.c
 *
 * Copyright (C) 2007 Google, Inc.
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

#include <asm/io.h>
#include <asm/gpio.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include "gpio_chip.h"
#include "gpio_hw.h"
#include "proc_comm.h"

#include "smd_private.h"

#ifndef CONFIG_GPIOLIB

enum {
	GPIO_DEBUG_SLEEP = 1U << 0,
};
static int msm_gpio_debug_mask = 0;
module_param_named(debug_mask, msm_gpio_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

/* private gpio_configure flags */
#define MSM_GPIOF_ENABLE_INTERRUPT      0x10000000
#define MSM_GPIOF_DISABLE_INTERRUPT     0x20000000
#define MSM_GPIOF_ENABLE_WAKE           0x40000000
#define MSM_GPIOF_DISABLE_WAKE          0x80000000

static int msm_gpio_configure(struct goog_gpio_chip *chip,
			unsigned int gpio,
			unsigned long flags);
static int msm_gpio_get_irq_num(struct goog_gpio_chip *chip,
				unsigned int gpio,
				unsigned int *irqp,
				unsigned long *irqnumflagsp);
static int msm_gpio_read(struct goog_gpio_chip *chip, unsigned n);
static int msm_gpio_write(struct goog_gpio_chip *chip,
			unsigned n,
			unsigned on);
static int msm_gpio_read_detect_status(struct goog_gpio_chip *chip,
				unsigned int gpio);
static int msm_gpio_clear_detect_status(struct goog_gpio_chip *chip,
					unsigned int gpio);

struct msm_gpio_chip msm_gpio_chips[] = {
	{
		.regs = {
			.out =         GPIO_OUT_0,
			.in =          GPIO_IN_0,
			.int_status =  GPIO_INT_STATUS_0,
			.int_clear =   GPIO_INT_CLEAR_0,
			.int_en =      GPIO_INT_EN_0,
			.int_edge =    GPIO_INT_EDGE_0,
			.int_pos =     GPIO_INT_POS_0,
			.oe =          GPIO_OE_0,
		},
		.chip = {
			.start = 0,
			.end = 15,
			.configure = msm_gpio_configure,
			.get_irq_num = msm_gpio_get_irq_num,
			.read = msm_gpio_read,
			.write = msm_gpio_write,
			.read_detect_status = msm_gpio_read_detect_status,
			.clear_detect_status = msm_gpio_clear_detect_status
		}
	},
	{
		.regs = {
			.out =         GPIO_OUT_1,
			.in =          GPIO_IN_1,
			.int_status =  GPIO_INT_STATUS_1,
			.int_clear =   GPIO_INT_CLEAR_1,
			.int_en =      GPIO_INT_EN_1,
			.int_edge =    GPIO_INT_EDGE_1,
			.int_pos =     GPIO_INT_POS_1,
			.oe =          GPIO_OE_1,
		},
		.chip = {
			.start = 16,
#if defined(CONFIG_ARCH_MSM7X30)
			.end = 43,
#else
			.end = 42,
#endif
			.configure = msm_gpio_configure,
			.get_irq_num = msm_gpio_get_irq_num,
			.read = msm_gpio_read,
			.write = msm_gpio_write,
			.read_detect_status = msm_gpio_read_detect_status,
			.clear_detect_status = msm_gpio_clear_detect_status
		}
	},
	{
		.regs = {
			.out =         GPIO_OUT_2,
			.in =          GPIO_IN_2,
			.int_status =  GPIO_INT_STATUS_2,
			.int_clear =   GPIO_INT_CLEAR_2,
			.int_en =      GPIO_INT_EN_2,
			.int_edge =    GPIO_INT_EDGE_2,
			.int_pos =     GPIO_INT_POS_2,
			.oe =          GPIO_OE_2,
		},
		.chip = {
#if defined(CONFIG_ARCH_MSM7X30)
			.start = 44,
#else
			.start = 43,
#endif
			.end = 67,
			.configure = msm_gpio_configure,
			.get_irq_num = msm_gpio_get_irq_num,
			.read = msm_gpio_read,
			.write = msm_gpio_write,
			.read_detect_status = msm_gpio_read_detect_status,
			.clear_detect_status = msm_gpio_clear_detect_status
		}
	},
	{
		.regs = {
			.out =         GPIO_OUT_3,
			.in =          GPIO_IN_3,
			.int_status =  GPIO_INT_STATUS_3,
			.int_clear =   GPIO_INT_CLEAR_3,
			.int_en =      GPIO_INT_EN_3,
			.int_edge =    GPIO_INT_EDGE_3,
			.int_pos =     GPIO_INT_POS_3,
			.oe =          GPIO_OE_3,
		},
		.chip = {
			.start = 68,
			.end = 94,
			.configure = msm_gpio_configure,
			.get_irq_num = msm_gpio_get_irq_num,
			.read = msm_gpio_read,
			.write = msm_gpio_write,
			.read_detect_status = msm_gpio_read_detect_status,
			.clear_detect_status = msm_gpio_clear_detect_status
		}
	},
	{
		.regs = {
			.out =         GPIO_OUT_4,
			.in =          GPIO_IN_4,
			.int_status =  GPIO_INT_STATUS_4,
			.int_clear =   GPIO_INT_CLEAR_4,
			.int_en =      GPIO_INT_EN_4,
			.int_edge =    GPIO_INT_EDGE_4,
			.int_pos =     GPIO_INT_POS_4,
			.oe =          GPIO_OE_4,
		},
		.chip = {
			.start = 95,
#if defined(CONFIG_ARCH_QSD8X50)
			.end = 103,
#else
			.end = 106,
#endif
			.configure = msm_gpio_configure,
			.get_irq_num = msm_gpio_get_irq_num,
			.read = msm_gpio_read,
			.write = msm_gpio_write,
			.read_detect_status = msm_gpio_read_detect_status,
			.clear_detect_status = msm_gpio_clear_detect_status
		}
	},
	{
		.regs = {
			.out =         GPIO_OUT_5,
			.in =          GPIO_IN_5,
			.int_status =  GPIO_INT_STATUS_5,
			.int_clear =   GPIO_INT_CLEAR_5,
			.int_en =      GPIO_INT_EN_5,
			.int_edge =    GPIO_INT_EDGE_5,
			.int_pos =     GPIO_INT_POS_5,
			.oe =          GPIO_OE_5,
		},
		.chip = {
#if defined(CONFIG_ARCH_QSD8X50)
			.start = 104,
			.end = 121,
#elif defined(CONFIG_ARCH_MSM7X30)
			.start = 107,
			.end = 133,
#else
			.start = 107,
			.end = 132,
#endif
			.configure = msm_gpio_configure,
			.get_irq_num = msm_gpio_get_irq_num,
			.read = msm_gpio_read,
			.write = msm_gpio_write,
			.read_detect_status = msm_gpio_read_detect_status,
			.clear_detect_status = msm_gpio_clear_detect_status
		}
	},
#if defined(CONFIG_ARCH_MSM_SCORPION)
	{
		.regs = {
			.out =         GPIO_OUT_6,
			.in =          GPIO_IN_6,
			.int_status =  GPIO_INT_STATUS_6,
			.int_clear =   GPIO_INT_CLEAR_6,
			.int_en =      GPIO_INT_EN_6,
			.int_edge =    GPIO_INT_EDGE_6,
			.int_pos =     GPIO_INT_POS_6,
			.oe =          GPIO_OE_6,
		},
		.chip = {
#if defined(CONFIG_ARCH_MSM7X30)
			.start = 134,
			.end = 150,
#else
			.start = 122,
			.end = 152,
#endif
			.configure = msm_gpio_configure,
			.get_irq_num = msm_gpio_get_irq_num,
			.read = msm_gpio_read,
			.write = msm_gpio_write,
			.read_detect_status = msm_gpio_read_detect_status,
			.clear_detect_status = msm_gpio_clear_detect_status
		}
	},
	{
		.regs = {
			.out =         GPIO_OUT_7,
			.in =          GPIO_IN_7,
			.int_status =  GPIO_INT_STATUS_7,
			.int_clear =   GPIO_INT_CLEAR_7,
			.int_en =      GPIO_INT_EN_7,
			.int_edge =    GPIO_INT_EDGE_7,
			.int_pos =     GPIO_INT_POS_7,
			.oe =          GPIO_OE_7,
		},
		.chip = {
#if defined(CONFIG_ARCH_MSM7X30)
			.start = 151,
			.end = 181,
#else
			.start = 153,
			.end = 164,
#endif
			.configure = msm_gpio_configure,
			.get_irq_num = msm_gpio_get_irq_num,
			.read = msm_gpio_read,
			.write = msm_gpio_write,
			.read_detect_status = msm_gpio_read_detect_status,
			.clear_detect_status = msm_gpio_clear_detect_status
		}
	},
#endif
};

static void msm_gpio_update_both_edge_detect(struct msm_gpio_chip *msm_chip)
{
	int loop_limit = 100;
	unsigned pol, val, val2, intstat;
	do {
		val = readl(msm_chip->regs.in);
		pol = readl(msm_chip->regs.int_pos);
		pol = (pol & ~msm_chip->both_edge_detect) | (~val & msm_chip->both_edge_detect);
		writel(pol, msm_chip->regs.int_pos);
		intstat = readl(msm_chip->regs.int_status);
		val2 = readl(msm_chip->regs.in);
		if (((val ^ val2) & msm_chip->both_edge_detect & ~intstat) == 0)
			return;
	} while (loop_limit-- > 0);
	printk(KERN_ERR "msm_gpio_update_both_edge_detect, failed to reach stable state %x != %x\n", val, val2);
}

static int msm_gpio_write(struct goog_gpio_chip *chip, unsigned n, unsigned on)
{
	struct msm_gpio_chip *msm_chip = container_of(chip, struct msm_gpio_chip, chip);
	unsigned b = 1U << (n - chip->start);
	unsigned v;

	v = readl(msm_chip->regs.out);
	if (on) {
		writel(v | b, msm_chip->regs.out);
	} else {
		writel(v & (~b), msm_chip->regs.out);
	}
	return 0;
}

static int msm_gpio_read(struct goog_gpio_chip *chip, unsigned n)
{
	struct msm_gpio_chip *msm_chip = container_of(chip, struct msm_gpio_chip, chip);
	unsigned b = 1U << (n - chip->start);

	return (readl(msm_chip->regs.in) & b) ? 1 : 0;
}

static int msm_gpio_read_detect_status(struct goog_gpio_chip *chip,
				unsigned int gpio)
{
	struct msm_gpio_chip *msm_chip = container_of(chip, struct msm_gpio_chip, chip);
	unsigned b = 1U << (gpio - chip->start);
	unsigned v;

	v = readl(msm_chip->regs.int_status);
#if MSM_GPIO_BROKEN_INT_CLEAR
	v |= msm_chip->int_status_copy;
#endif
	return (v & b) ? 1 : 0;
}

static int msm_gpio_clear_detect_status(struct goog_gpio_chip *chip,
					unsigned int gpio)
{
	struct msm_gpio_chip *msm_chip = container_of(chip, struct msm_gpio_chip, chip);
	unsigned b = 1U << (gpio - chip->start);

#if MSM_GPIO_BROKEN_INT_CLEAR
	/* Save interrupts that already triggered before we loose them. */
	/* Any interrupt that triggers between the read of int_status */
	/* and the write to int_clear will still be lost though. */
	msm_chip->int_status_copy |= readl(msm_chip->regs.int_status);
	msm_chip->int_status_copy &= ~b;
#endif
	writel(b, msm_chip->regs.int_clear);
	msm_gpio_update_both_edge_detect(msm_chip);
	return 0;
}

int msm_gpio_configure(struct goog_gpio_chip *chip,
		unsigned int gpio,
		unsigned long flags)
{
	struct msm_gpio_chip *msm_chip = container_of(chip, struct msm_gpio_chip, chip);
	unsigned b = 1U << (gpio - chip->start);
	unsigned v;

	if (flags & (GPIOF_OUTPUT_LOW | GPIOF_OUTPUT_HIGH))
		msm_gpio_write(chip, gpio, flags & GPIOF_OUTPUT_HIGH);

	if (flags & (GPIOF_INPUT | GPIOF_DRIVE_OUTPUT)) {
		v = readl(msm_chip->regs.oe);
		if (flags & GPIOF_DRIVE_OUTPUT) {
			writel(v | b, msm_chip->regs.oe);
		} else {
			writel(v & (~b), msm_chip->regs.oe);
		}
	}

	if (flags & (IRQF_TRIGGER_MASK | GPIOF_IRQF_TRIGGER_NONE)) {
		v = readl(msm_chip->regs.int_edge);
		if (flags & (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING)) {
			writel(v | b, msm_chip->regs.int_edge);
			irq_desc[MSM_GPIO_TO_INT(gpio)].handle_irq = handle_edge_irq;
		} else {
			writel(v & (~b), msm_chip->regs.int_edge);
			irq_desc[MSM_GPIO_TO_INT(gpio)].handle_irq = handle_level_irq;
		}
		if ((flags & (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING)) == (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING)) {
			msm_chip->both_edge_detect |= b;
			msm_gpio_update_both_edge_detect(msm_chip);
		} else {
			msm_chip->both_edge_detect &= ~b;
			v = readl(msm_chip->regs.int_pos);
			if (flags & (IRQF_TRIGGER_RISING | IRQF_TRIGGER_HIGH)) {
				writel(v | b, msm_chip->regs.int_pos);
			} else {
				writel(v & (~b), msm_chip->regs.int_pos);
			}
		}
	}

	/* used by msm_gpio_irq_mask and msm_gpio_irq_unmask */
	if (flags & (MSM_GPIOF_ENABLE_INTERRUPT | MSM_GPIOF_DISABLE_INTERRUPT)) {
		v = readl(msm_chip->regs.int_edge);
		/* level triggered interrupts are also latched */
		if (!(v & b))
			msm_gpio_clear_detect_status(chip, gpio);
		if (flags & MSM_GPIOF_ENABLE_INTERRUPT) {
			msm_chip->int_enable[0] |= b;
		} else {
			msm_chip->int_enable[0] &= ~b;
		}
		writel(msm_chip->int_enable[0], msm_chip->regs.int_en);
	}

	if (flags & (MSM_GPIOF_ENABLE_WAKE | MSM_GPIOF_DISABLE_WAKE)) {
		if (flags & MSM_GPIOF_ENABLE_WAKE)
			msm_chip->int_enable[1] |= b;
		else
			msm_chip->int_enable[1] &= ~b;
	}

	return 0;
}

static int msm_gpio_get_irq_num(struct goog_gpio_chip *chip,
				unsigned int gpio,
				unsigned int *irqp,
				unsigned long *irqnumflagsp)
{
	*irqp = MSM_GPIO_TO_INT(gpio);
	if (irqnumflagsp)
		*irqnumflagsp = 0;
	return 0;
}


static void msm_gpio_irq_ack(unsigned int irq)
{
	gpio_clear_detect_status(irq - NR_MSM_IRQS);
}

static void msm_gpio_irq_mask(unsigned int irq)
{
	gpio_configure(irq - NR_MSM_IRQS, MSM_GPIOF_DISABLE_INTERRUPT);
}

static void msm_gpio_irq_unmask(unsigned int irq)
{
	gpio_configure(irq - NR_MSM_IRQS, MSM_GPIOF_ENABLE_INTERRUPT);
}

static void msm_gpio_irq_disable(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);

	msm_gpio_irq_mask(irq);
	desc->status |= IRQ_MASKED;
}

static void msm_gpio_irq_enable(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);

	msm_gpio_irq_unmask(irq);
	desc->status &= ~IRQ_MASKED;
}

static int msm_gpio_irq_set_wake(unsigned int irq, unsigned int on)
{
	return gpio_configure(irq - NR_MSM_IRQS, on ? MSM_GPIOF_ENABLE_WAKE : MSM_GPIOF_DISABLE_WAKE);
}


static int msm_gpio_irq_set_type(unsigned int irq, unsigned int flow_type)
{
	return gpio_configure(irq - NR_MSM_IRQS, flow_type);
}

static void msm_gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	int i, j, m;
	unsigned v;

	for (i = 0; i < ARRAY_SIZE(msm_gpio_chips); i++) {
		struct msm_gpio_chip *msm_chip = &msm_gpio_chips[i];
		v = readl(msm_chip->regs.int_status);
		v &= msm_chip->int_enable[0];
		while (v) {
			m = v & -v;
			j = fls(m) - 1;
			/* printk("msm_gpio_irq_handler %08x %08x bit %d gpio %d irq %d\n", v, m, j, msm_chip->chip.start + j, NR_MSM_IRQS + msm_chip->chip.start + j); */
			v &= ~m;
			generic_handle_irq(NR_MSM_IRQS + msm_chip->chip.start + j);
		}
	}
	desc->chip->ack(irq);
}

static struct irq_chip msm_gpio_irq_chip = {
	.name      = "msmgpio",
	.ack       = msm_gpio_irq_ack,
	.mask      = msm_gpio_irq_mask,
	.unmask    = msm_gpio_irq_unmask,
	.disable   = msm_gpio_irq_disable,
	.enable    = msm_gpio_irq_enable,
	.set_wake  = msm_gpio_irq_set_wake,
	.set_type  = msm_gpio_irq_set_type,
};

#define NUM_GPIO_SMEM_BANKS 6
#define GPIO_SMEM_NUM_GROUPS 2
#define GPIO_SMEM_MAX_PC_INTERRUPTS 8
struct tramp_gpio_smem
{
	uint16_t num_fired[GPIO_SMEM_NUM_GROUPS];
	uint16_t fired[GPIO_SMEM_NUM_GROUPS][GPIO_SMEM_MAX_PC_INTERRUPTS];
	uint32_t enabled[NUM_GPIO_SMEM_BANKS];
	uint32_t detection[NUM_GPIO_SMEM_BANKS];
	uint32_t polarity[NUM_GPIO_SMEM_BANKS];
};

static void msm_gpio_sleep_int(unsigned long arg)
{
	int i, j;
	struct tramp_gpio_smem *smem_gpio;

	BUILD_BUG_ON(NR_GPIO_IRQS > NUM_GPIO_SMEM_BANKS * 32);

	smem_gpio = smem_alloc(SMEM_GPIO_INT, sizeof(*smem_gpio)); 
	if (smem_gpio == NULL)
		return;

	local_irq_disable();
	for(i = 0; i < GPIO_SMEM_NUM_GROUPS; i++) {
		int count = smem_gpio->num_fired[i];
		for(j = 0; j < count; j++) {
			/* TODO: Check mask */
			generic_handle_irq(MSM_GPIO_TO_INT(smem_gpio->fired[i][j]));
		}
	}
	local_irq_enable();
}

static DECLARE_TASKLET(msm_gpio_sleep_int_tasklet, msm_gpio_sleep_int, 0);

void msm_gpio_enter_sleep(int from_idle)
{
	int i;
	struct tramp_gpio_smem *smem_gpio;

	smem_gpio = smem_alloc(SMEM_GPIO_INT, sizeof(*smem_gpio)); 

	if (smem_gpio) {
		for (i = 0; i < ARRAY_SIZE(smem_gpio->enabled); i++) {
			smem_gpio->enabled[i] = 0;
			smem_gpio->detection[i] = 0;
			smem_gpio->polarity[i] = 0;
		}
	}

	for (i = 0; i < ARRAY_SIZE(msm_gpio_chips); i++) {
		writel(msm_gpio_chips[i].int_enable[!from_idle], msm_gpio_chips[i].regs.int_en);
		if (smem_gpio) {
			uint32_t tmp;
			int start, index, shiftl, shiftr;
			start = msm_gpio_chips[i].chip.start;
			index = start / 32;
			shiftl = start % 32;
			shiftr = 32 - shiftl;
			tmp = msm_gpio_chips[i].int_enable[!from_idle];
			smem_gpio->enabled[index] |= tmp << shiftl;
			smem_gpio->enabled[index+1] |= tmp >> shiftr;
			smem_gpio->detection[index] |= readl(msm_gpio_chips[i].regs.int_edge) << shiftl;
			smem_gpio->detection[index+1] |= readl(msm_gpio_chips[i].regs.int_edge) >> shiftr;
			smem_gpio->polarity[index] |= readl(msm_gpio_chips[i].regs.int_pos) << shiftl;
			smem_gpio->polarity[index+1] |= readl(msm_gpio_chips[i].regs.int_pos) >> shiftr;
		}
	}

	if (smem_gpio) {
		if (msm_gpio_debug_mask & GPIO_DEBUG_SLEEP)
			for (i = 0; i < ARRAY_SIZE(smem_gpio->enabled); i++) {
				printk("msm_gpio_enter_sleep gpio %d-%d: enable"
				       " %08x, edge %08x, polarity %08x\n",
				       i * 32, i * 32 + 31,
				       smem_gpio->enabled[i],
				       smem_gpio->detection[i],
				       smem_gpio->polarity[i]);
			}
		for(i = 0; i < GPIO_SMEM_NUM_GROUPS; i++)
			smem_gpio->num_fired[i] = 0;
	}
}

void msm_gpio_exit_sleep(void)
{
	int i;
	struct tramp_gpio_smem *smem_gpio;

	smem_gpio = smem_alloc(SMEM_GPIO_INT, sizeof(*smem_gpio)); 

	for (i = 0; i < ARRAY_SIZE(msm_gpio_chips); i++) {
		writel(msm_gpio_chips[i].int_enable[0], msm_gpio_chips[i].regs.int_en);
	}

	if (smem_gpio && (smem_gpio->num_fired[0] || smem_gpio->num_fired[1])) {
		if (msm_gpio_debug_mask & GPIO_DEBUG_SLEEP)
			printk(KERN_INFO "gpio: fired %x %x\n",
			      smem_gpio->num_fired[0], smem_gpio->num_fired[1]);
		tasklet_schedule(&msm_gpio_sleep_int_tasklet);
	}
}

static int __init msm_init_gpio(void)
{
	int i;

	for (i = NR_MSM_IRQS; i < NR_MSM_IRQS + NR_GPIO_IRQS; i++) {
		set_irq_chip(i, &msm_gpio_irq_chip);
		set_irq_handler(i, handle_edge_irq);
		set_irq_flags(i, IRQF_VALID);
	}

	for (i = 0; i < ARRAY_SIZE(msm_gpio_chips); i++) {
		writel(0, msm_gpio_chips[i].regs.int_en);
		register_gpio_chip(&msm_gpio_chips[i].chip);
	}

	set_irq_chained_handler(INT_GPIO_GROUP1, msm_gpio_irq_handler);
	set_irq_chained_handler(INT_GPIO_GROUP2, msm_gpio_irq_handler);
	set_irq_wake(INT_GPIO_GROUP1, 1);
	set_irq_wake(INT_GPIO_GROUP2, 2);
	return 0;
}

postcore_initcall(msm_init_gpio);

#if defined(CONFIG_DEBUG_FS)

static int msm_gpio_debug_result = 1;

static int gpio_enable_set(void *data, u64 val)
{
	msm_gpio_debug_result = gpio_tlmm_config(val, 0);
	return 0;
}
static int gpio_disable_set(void *data, u64 val)
{
	msm_gpio_debug_result = gpio_tlmm_config(val, 1);
	return 0;
}

static int gpio_debug_get(void *data, u64 *val)
{
	unsigned int result = msm_gpio_debug_result;
	msm_gpio_debug_result = 1;
	if (result)
		*val = 1;
	else
		*val = 0;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(gpio_enable_fops, gpio_debug_get,
						gpio_enable_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(gpio_disable_fops, gpio_debug_get,
						gpio_disable_set, "%llu\n");

static int __init gpio_debug_init(void)
{
	struct dentry *dent;
	dent = debugfs_create_dir("gpio", 0);
	if (IS_ERR(dent))
		return 0;

	debugfs_create_file("enable", 0644, dent, 0, &gpio_enable_fops);
	debugfs_create_file("disable", 0644, dent, 0, &gpio_disable_fops);
	return 0;
}

device_initcall(gpio_debug_init);
#endif
#endif
