/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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
 *
 */
#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <mach/msm_iomap.h>
#include "gpiomux.h"
#include "mpm.h"

/* Bits of interest in the GPIO_IN_OUT register.
 */
enum {
	GPIO_IN_BIT  = 0,
	GPIO_OUT_BIT = 1
};

/* Bits of interest in the GPIO_INTR_STATUS register.
 */
enum {
	INTR_STATUS_BIT = 0,
};

/* Bits of interest in the GPIO_CFG register.
 */
enum {
	GPIO_OE_BIT = 9,
};

/* Bits of interest in the GPIO_INTR_CFG register.
 */
enum {
	INTR_ENABLE_BIT        = 0,
	INTR_POL_CTL_BIT       = 1,
	INTR_DECT_CTL_BIT      = 2,
	INTR_RAW_STATUS_EN_BIT = 3,
};

/* Codes of interest in GPIO_INTR_CFG_SU.
 */
enum {
	TARGET_PROC_SCORPION = 4,
	TARGET_PROC_NONE     = 7,
};

/*
 * There is no 'DC_POLARITY_LO' because the GIC is incapable
 * of asserting on falling edge or level-low conditions.  Even though
 * the registers allow for low-polarity inputs, the case can never arise.
 */
enum {
	DC_POLARITY_HI	= BIT(11),
	DC_IRQ_ENABLE	= BIT(3),
};

enum msm_tlmm_register {
	SDC4_HDRV_PULL_CTL = 0x20a0,
	SDC3_HDRV_PULL_CTL = 0x20a4,
};

struct tlmm_field_cfg {
	enum msm_tlmm_register reg;
	u8                     off;
};

static const struct tlmm_field_cfg tlmm_hdrv_cfgs[] = {
	{SDC4_HDRV_PULL_CTL, 6}, /* TLMM_HDRV_SDC4_CLK  */
	{SDC4_HDRV_PULL_CTL, 3}, /* TLMM_HDRV_SDC4_CMD  */
	{SDC4_HDRV_PULL_CTL, 0}, /* TLMM_HDRV_SDC4_DATA */
	{SDC3_HDRV_PULL_CTL, 6}, /* TLMM_HDRV_SDC3_CLK  */
	{SDC3_HDRV_PULL_CTL, 3}, /* TLMM_HDRV_SDC3_CMD  */
	{SDC3_HDRV_PULL_CTL, 0}, /* TLMM_HDRV_SDC3_DATA */
};

static const struct tlmm_field_cfg tlmm_pull_cfgs[] = {
	{SDC4_HDRV_PULL_CTL, 11}, /* TLMM_PULL_SDC4_CMD  */
	{SDC4_HDRV_PULL_CTL, 9},  /* TLMM_PULL_SDC4_DATA */
	{SDC3_HDRV_PULL_CTL, 11}, /* TLMM_PULL_SDC3_CMD  */
	{SDC3_HDRV_PULL_CTL, 9},  /* TLMM_PULL_SDC3_DATA */
};

/*
 * When a GPIO triggers, two separate decisions are made, controlled
 * by two separate flags.
 *
 * - First, INTR_RAW_STATUS_EN controls whether or not the GPIO_INTR_STATUS
 * register for that GPIO will be updated to reflect the triggering of that
 * gpio.  If this bit is 0, this register will not be updated.
 * - Second, INTR_ENABLE controls whether an interrupt is triggered.
 *
 * If INTR_ENABLE is set and INTR_RAW_STATUS_EN is NOT set, an interrupt
 * can be triggered but the status register will not reflect it.
 */
#define INTR_RAW_STATUS_EN BIT(INTR_RAW_STATUS_EN_BIT)
#define INTR_ENABLE        BIT(INTR_ENABLE_BIT)
#define INTR_DECT_CTL_EDGE BIT(INTR_DECT_CTL_BIT)
#define INTR_POL_CTL_HI    BIT(INTR_POL_CTL_BIT)

#define GPIO_INTR_CFG_SU(gpio)    (MSM_TLMM_BASE + 0x0400 + (0x04 * (gpio)))
#define DIR_CONN_INTR_CFG_SU(irq) (MSM_TLMM_BASE + 0x0700 + (0x04 * (irq)))
#define GPIO_CONFIG(gpio)         (MSM_TLMM_BASE + 0x1000 + (0x10 * (gpio)))
#define GPIO_IN_OUT(gpio)         (MSM_TLMM_BASE + 0x1004 + (0x10 * (gpio)))
#define GPIO_INTR_CFG(gpio)       (MSM_TLMM_BASE + 0x1008 + (0x10 * (gpio)))
#define GPIO_INTR_STATUS(gpio)    (MSM_TLMM_BASE + 0x100c + (0x10 * (gpio)))

/**
 * struct msm_gpio_dev: the MSM8660 SoC GPIO device structure
 *
 * @enabled_irqs: a bitmap used to optimize the summary-irq handler.  By
 * keeping track of which gpios are unmasked as irq sources, we avoid
 * having to do readl calls on hundreds of iomapped registers each time
 * the summary interrupt fires in order to locate the active interrupts.
 *
 * @wake_irqs: a bitmap for tracking which interrupt lines are enabled
 * as wakeup sources.  When the device is suspended, interrupts which are
 * not wakeup sources are disabled.
 *
 * @dual_edge_irqs: a bitmap used to track which irqs are configured
 * as dual-edge, as this is not supported by the hardware and requires
 * some special handling in the driver.
 */
struct msm_gpio_dev {
	struct gpio_chip gpio_chip;
	int last_wake_gpio;
	DECLARE_BITMAP(enabled_irqs, NR_MSM_GPIOS);
	DECLARE_BITMAP(wake_irqs, NR_MSM_GPIOS);
	DECLARE_BITMAP(dual_edge_irqs, NR_MSM_GPIOS);
	DECLARE_BITMAP(saved_wake_source, NR_MSM_GPIOS);
};

static DEFINE_SPINLOCK(tlmm_lock);

static inline struct msm_gpio_dev *to_msm_gpio_dev(struct gpio_chip *chip)
{
	return container_of(chip, struct msm_gpio_dev, gpio_chip);
}

static inline void set_gpio_bits(unsigned n, void __iomem *reg)
{
	writel(readl(reg) | n, reg);
}

static inline void clr_gpio_bits(unsigned n, void __iomem *reg)
{
	writel(readl(reg) & ~n, reg);
}

static int msm_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	return readl(GPIO_IN_OUT(offset)) & BIT(GPIO_IN_BIT);
}

static void msm_gpio_set(struct gpio_chip *chip, unsigned offset, int val)
{
	writel(val ? BIT(GPIO_OUT_BIT) : 0, GPIO_IN_OUT(offset));
}

static int msm_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&tlmm_lock, irq_flags);
	clr_gpio_bits(BIT(GPIO_OE_BIT), GPIO_CONFIG(offset));
	spin_unlock_irqrestore(&tlmm_lock, irq_flags);
	return 0;
}

static int msm_gpio_direction_output(struct gpio_chip *chip,
				unsigned offset,
				int val)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&tlmm_lock, irq_flags);
	msm_gpio_set(chip, offset, val);
	set_gpio_bits(BIT(GPIO_OE_BIT), GPIO_CONFIG(offset));
	spin_unlock_irqrestore(&tlmm_lock, irq_flags);
	return 0;
}

static int msm_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	return MSM_GPIO_TO_INT(offset - chip->base);
}

static inline int msm_irq_to_gpio(struct gpio_chip *chip, unsigned irq)
{
	return irq - MSM_GPIO_TO_INT(chip->base);
}

static int msm_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	return msm_gpiomux_get(chip->base + offset);
}

static void msm_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	msm_gpiomux_put(chip->base + offset);
}

static struct msm_gpio_dev msm_gpio = {
	.gpio_chip = {
		.base             = 0,
		.ngpio            = NR_MSM_GPIOS,
		.direction_input  = msm_gpio_direction_input,
		.direction_output = msm_gpio_direction_output,
		.get              = msm_gpio_get,
		.set              = msm_gpio_set,
		.to_irq           = msm_gpio_to_irq,
		.request          = msm_gpio_request,
		.free             = msm_gpio_free,
	},
	.last_wake_gpio = -1,
};

/* For dual-edge interrupts in software, since the hardware has no
 * such support:
 *
 * At appropriate moments, this function may be called to flip the polarity
 * settings of both-edge irq lines to try and catch the next edge.
 *
 * The attempt is considered successful if:
 * - the status bit goes high, indicating that an edge was caught, or
 * - the input value of the gpio doesn't change during the attempt.
 * If the value changes twice during the process, that would cause the first
 * test to fail but would force the second, as two opposite
 * transitions would cause a detection no matter the polarity setting.
 *
 * The do-loop tries to sledge-hammer closed the timing hole between
 * the initial value-read and the polarity-write - if the line value changes
 * during that window, an interrupt is lost, the new polarity setting is
 * incorrect, and the first success test will fail, causing a retry.
 *
 * Algorithm comes from Google's msmgpio driver, see mach-msm/gpio.c.
 */
static void msm_gpio_update_dual_edge_pos(unsigned gpio)
{
	int loop_limit = 100;
	unsigned val, val2, intstat;

	do {
		val = readl(GPIO_IN_OUT(gpio)) & BIT(GPIO_IN_BIT);
		if (val)
			clr_gpio_bits(INTR_POL_CTL_HI, GPIO_INTR_CFG(gpio));
		else
			set_gpio_bits(INTR_POL_CTL_HI, GPIO_INTR_CFG(gpio));
		val2 = readl(GPIO_IN_OUT(gpio)) & BIT(GPIO_IN_BIT);
		intstat = readl(GPIO_INTR_STATUS(gpio)) & BIT(INTR_STATUS_BIT);
		if (intstat || val == val2) {
			/* switch the configuration in the mpm as well*/
			if (val)
				msm_mpm_set_irq_type(msm_gpio_to_irq(
						&msm_gpio.gpio_chip, gpio),
						IRQF_TRIGGER_FALLING);
			else
				msm_mpm_set_irq_type(msm_gpio_to_irq(
						&msm_gpio.gpio_chip, gpio),
						IRQF_TRIGGER_RISING);
			return;
		}
	} while (loop_limit-- > 0);
	pr_err("%s: dual-edge irq failed to stabilize, "
	       "interrupts dropped. %#08x != %#08x\n",
	       __func__, val, val2);
}

static void msm_gpio_irq_ack(unsigned int irq)
{
	int gpio = msm_irq_to_gpio(&msm_gpio.gpio_chip, irq);

	writel(BIT(INTR_STATUS_BIT), GPIO_INTR_STATUS(gpio));
	if (test_bit(gpio, msm_gpio.dual_edge_irqs))
		msm_gpio_update_dual_edge_pos(gpio);
}

static void msm_gpio_irq_mask(unsigned int irq)
{
	int gpio = msm_irq_to_gpio(&msm_gpio.gpio_chip, irq);
	unsigned long irq_flags;

	spin_lock_irqsave(&tlmm_lock, irq_flags);
	writel(TARGET_PROC_NONE, GPIO_INTR_CFG_SU(gpio));
	clr_gpio_bits(INTR_RAW_STATUS_EN | INTR_ENABLE, GPIO_INTR_CFG(gpio));
	__clear_bit(gpio, msm_gpio.enabled_irqs);
	spin_unlock_irqrestore(&tlmm_lock, irq_flags);

	msm_mpm_enable_irq(irq, 0);
}

static void msm_gpio_irq_unmask(unsigned int irq)
{
	int gpio = msm_irq_to_gpio(&msm_gpio.gpio_chip, irq);
	unsigned long irq_flags;

	spin_lock_irqsave(&tlmm_lock, irq_flags);
	__set_bit(gpio, msm_gpio.enabled_irqs);
	set_gpio_bits(INTR_RAW_STATUS_EN | INTR_ENABLE, GPIO_INTR_CFG(gpio));
	writel(TARGET_PROC_SCORPION, GPIO_INTR_CFG_SU(gpio));
	spin_unlock_irqrestore(&tlmm_lock, irq_flags);

	msm_mpm_enable_irq(irq, 1);
}

static int msm_gpio_irq_set_type(unsigned int irq, unsigned int flow_type)
{
	int gpio = msm_irq_to_gpio(&msm_gpio.gpio_chip, irq);
	unsigned long irq_flags;
	uint32_t bits;

	spin_lock_irqsave(&tlmm_lock, irq_flags);

	bits = readl(GPIO_INTR_CFG(gpio));

	if (flow_type & IRQ_TYPE_EDGE_BOTH) {
		bits |= INTR_DECT_CTL_EDGE;
		irq_desc[irq].handle_irq = handle_edge_irq;
		if ((flow_type & IRQ_TYPE_EDGE_BOTH) == IRQ_TYPE_EDGE_BOTH)
			__set_bit(gpio, msm_gpio.dual_edge_irqs);
		else
			__clear_bit(gpio, msm_gpio.dual_edge_irqs);
	} else {
		bits &= ~INTR_DECT_CTL_EDGE;
		irq_desc[irq].handle_irq = handle_level_irq;
		__clear_bit(gpio, msm_gpio.dual_edge_irqs);
	}

	if (flow_type & (IRQ_TYPE_EDGE_RISING | IRQ_TYPE_LEVEL_HIGH))
		bits |= INTR_POL_CTL_HI;
	else
		bits &= ~INTR_POL_CTL_HI;

	writel(bits, GPIO_INTR_CFG(gpio));

	if ((flow_type & IRQ_TYPE_EDGE_BOTH) == IRQ_TYPE_EDGE_BOTH)
		msm_gpio_update_dual_edge_pos(gpio);

	spin_unlock_irqrestore(&tlmm_lock, irq_flags);
	msm_mpm_set_irq_type(irq, flow_type);

	return 0;
}

/*
 * When the summary IRQ is raised, any number of GPIO lines may be high.
 * It is the job of the summary handler to find all those GPIO lines
 * which have been set as summary IRQ lines and which are triggered,
 * and to call their interrupt handlers.
 */
static void msm_summary_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	unsigned long i;

	for (i = find_first_bit(msm_gpio.enabled_irqs, NR_MSM_GPIOS);
	     i < NR_MSM_GPIOS;
	     i = find_next_bit(msm_gpio.enabled_irqs, NR_MSM_GPIOS, i + 1)) {
		if (readl(GPIO_INTR_STATUS(i)) & BIT(INTR_STATUS_BIT))
			generic_handle_irq(msm_gpio_to_irq(&msm_gpio.gpio_chip,
							   i));
	}
	desc->chip->ack(irq);
}

static int msm_gpio_irq_set_wake(unsigned int irq, unsigned int on)
{
	int gpio = msm_irq_to_gpio(&msm_gpio.gpio_chip, irq);

	if (on) {
		if (bitmap_empty(msm_gpio.wake_irqs, NR_MSM_GPIOS))
			set_irq_wake(TLMM_SCSS_SUMMARY_IRQ, 1);
		set_bit(gpio, msm_gpio.wake_irqs);
	} else {
		clear_bit(gpio, msm_gpio.wake_irqs);
		if (bitmap_empty(msm_gpio.wake_irqs, NR_MSM_GPIOS))
			set_irq_wake(TLMM_SCSS_SUMMARY_IRQ, 0);
	}

	msm_mpm_set_irq_wake(irq, on);
	return 0;
}

static struct irq_chip msm_gpio_irq_chip = {
	.name		= "msmgpio",
	.mask		= msm_gpio_irq_mask,
	.unmask		= msm_gpio_irq_unmask,
	.ack		= msm_gpio_irq_ack,
	.set_type	= msm_gpio_irq_set_type,
	.set_wake	= msm_gpio_irq_set_wake,
};

static int __devinit msm_gpio_probe(struct platform_device *dev)
{
	int i, irq, ret;

	spin_lock_init(&tlmm_lock);
	bitmap_zero(msm_gpio.enabled_irqs, NR_MSM_GPIOS);
	bitmap_zero(msm_gpio.wake_irqs, NR_MSM_GPIOS);
	bitmap_zero(msm_gpio.dual_edge_irqs, NR_MSM_GPIOS);
	bitmap_zero(msm_gpio.saved_wake_source, NR_MSM_GPIOS);
	msm_gpio.gpio_chip.label = dev->name;
	ret = gpiochip_add(&msm_gpio.gpio_chip);
	if (ret < 0)
		return ret;

	for (i = 0; i < msm_gpio.gpio_chip.ngpio; ++i) {
		irq = msm_gpio_to_irq(&msm_gpio.gpio_chip, i);
		set_irq_chip(irq, &msm_gpio_irq_chip);
		set_irq_handler(irq, handle_level_irq);
		set_irq_flags(irq, IRQF_VALID);
	}

	set_irq_chained_handler(TLMM_SCSS_SUMMARY_IRQ,
				msm_summary_irq_handler);
	return 0;
}

static int __devexit msm_gpio_remove(struct platform_device *dev)
{
	int ret = gpiochip_remove(&msm_gpio.gpio_chip);

	if (ret < 0)
		return ret;

	set_irq_handler(TLMM_SCSS_SUMMARY_IRQ, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int msm_gpio_suspend_noirq(struct device *dev)
{
	unsigned long irq_flags;
	unsigned long i;

	spin_lock_irqsave(&tlmm_lock, irq_flags);
	for_each_set_bit(i, msm_gpio.enabled_irqs, NR_MSM_GPIOS) {
		if (!test_bit(i, msm_gpio.wake_irqs))
			writel(TARGET_PROC_NONE, GPIO_INTR_CFG_SU(i));
	}
	spin_unlock_irqrestore(&tlmm_lock, irq_flags);

	bitmap_zero(msm_gpio.saved_wake_source, NR_MSM_GPIOS);

	return 0;
}

void msm_gpio_save_wakeup_gpio(void)
{
	unsigned long irq_flags;
	int i, intstat;

	spin_lock_irqsave(&tlmm_lock, irq_flags);
	for_each_set_bit(i, msm_gpio.wake_irqs, NR_MSM_GPIOS) {
		intstat = readl(GPIO_INTR_STATUS(i)) & BIT(INTR_STATUS_BIT);

		if (intstat)
			set_bit(i, msm_gpio.saved_wake_source);
	}

	spin_unlock_irqrestore(&tlmm_lock, irq_flags);
}

void msm_gpio_show_resume_irq(void)
{
	unsigned long irq_flags;
	int i, irq, intstat;

	spin_lock_irqsave(&tlmm_lock, irq_flags);
	for_each_set_bit(i, msm_gpio.wake_irqs, NR_MSM_GPIOS) {
		intstat = readl(GPIO_INTR_STATUS(i)) & BIT(INTR_STATUS_BIT);
		if (intstat) {
			irq = msm_gpio_to_irq(&msm_gpio.gpio_chip, i);
			pr_warning("%s: irq %d/gpio %d triggered wake\n",
				__func__, irq, i);
		}
	}
	spin_unlock_irqrestore(&tlmm_lock, irq_flags);
}

int msm_gpio_is_wake_source(int gpio)
{
	return test_bit(gpio, msm_gpio.saved_wake_source);
}

static int msm_gpio_resume_noirq(struct device *dev)
{
	unsigned long irq_flags;
	unsigned long i;

	spin_lock_irqsave(&tlmm_lock, irq_flags);
	for_each_set_bit(i, msm_gpio.enabled_irqs, NR_MSM_GPIOS)
		writel(TARGET_PROC_SCORPION, GPIO_INTR_CFG_SU(i));
	spin_unlock_irqrestore(&tlmm_lock, irq_flags);
	return 0;
}
#else
#define msm_gpio_suspend_noirq NULL
#define msm_gpio_resume_noirq NULL
#endif

static const struct dev_pm_ops msm_gpio_dev_pm_ops = {
	.suspend_noirq  = msm_gpio_suspend_noirq,
	.resume_noirq   = msm_gpio_resume_noirq,
	.freeze_noirq   = msm_gpio_suspend_noirq,
	.thaw_noirq     = msm_gpio_resume_noirq,
	.poweroff_noirq = msm_gpio_suspend_noirq,
	.restore_noirq  = msm_gpio_resume_noirq,
};

static struct platform_driver msm_gpio_driver = {
	.probe = msm_gpio_probe,
	.remove = __devexit_p(msm_gpio_remove),
	.driver = {
		.name = "msmgpio",
		.owner = THIS_MODULE,
		.pm = &msm_gpio_dev_pm_ops,
	},
};

static struct platform_device msm_device_gpio = {
	.name = "msmgpio",
	.id   = 0,
};

static int __init msm_gpio_init(void)
{
	int rc;

	rc = platform_driver_register(&msm_gpio_driver);
	if (rc == 0)
		rc = platform_device_register(&msm_device_gpio);

	return rc;
}

static void __exit msm_gpio_exit(void)
{
	platform_device_unregister(&msm_device_gpio);
	platform_driver_unregister(&msm_gpio_driver);
}

postcore_initcall(msm_gpio_init);
module_exit(msm_gpio_exit);

static void msm_tlmm_set_field(const struct tlmm_field_cfg *configs,
			       unsigned id, unsigned width, unsigned val)
{
	unsigned long irqflags;
	u32 mask = (1 << width) - 1;
	u32 __iomem *reg = MSM_TLMM_BASE + configs[id].reg;
	u32 reg_val;

	spin_lock_irqsave(&tlmm_lock, irqflags);
	reg_val = readl(reg);
	reg_val &= ~(mask << configs[id].off);
	reg_val |= (val & mask) << configs[id].off;
	writel(reg_val, reg);
	spin_unlock_irqrestore(&tlmm_lock, irqflags);
}

void msm_tlmm_set_hdrive(enum msm_tlmm_hdrive_tgt tgt, int drv_str)
{
	msm_tlmm_set_field(tlmm_hdrv_cfgs, tgt, 3, drv_str);
}
EXPORT_SYMBOL(msm_tlmm_set_hdrive);

void msm_tlmm_set_pull(enum msm_tlmm_pull_tgt tgt, int pull)
{
	msm_tlmm_set_field(tlmm_pull_cfgs, tgt, 2, pull);
}
EXPORT_SYMBOL(msm_tlmm_set_pull);

int gpio_tlmm_config(unsigned config, unsigned disable)
{
	uint32_t flags;
	unsigned gpio = GPIO_PIN(config);

	if (gpio > NR_MSM_GPIOS)
		return -EINVAL;

	flags = ((GPIO_DIR(config) << 9) & (0x1 << 9)) |
		((GPIO_DRVSTR(config) << 6) & (0x7 << 6)) |
		((GPIO_FUNC(config) << 2) & (0xf << 2)) |
		((GPIO_PULL(config) & 0x3));
	writel(flags, GPIO_CONFIG(gpio));

	return 0;
}
EXPORT_SYMBOL(gpio_tlmm_config);

int msm_gpio_install_direct_irq(unsigned gpio, unsigned irq,
					unsigned int input_polarity)
{
	unsigned long irq_flags;
	uint32_t bits;

	if (gpio >= NR_MSM_GPIOS || irq >= NR_TLMM_SCSS_DIR_CONN_IRQ)
		return -EINVAL;

	spin_lock_irqsave(&tlmm_lock, irq_flags);

	writel(readl(GPIO_CONFIG(gpio)) | BIT(GPIO_OE_BIT),
		GPIO_CONFIG(gpio));
	writel(readl(GPIO_INTR_CFG(gpio)) &
		~(INTR_RAW_STATUS_EN | INTR_ENABLE),
		GPIO_INTR_CFG(gpio));
	writel(DC_IRQ_ENABLE | TARGET_PROC_NONE,
		GPIO_INTR_CFG_SU(gpio));

	bits = TARGET_PROC_SCORPION | (gpio << 3);
	if (input_polarity)
		bits |= DC_POLARITY_HI;
	writel(bits, DIR_CONN_INTR_CFG_SU(irq));

	spin_unlock_irqrestore(&tlmm_lock, irq_flags);

	return 0;
}
EXPORT_SYMBOL(msm_gpio_install_direct_irq);

MODULE_AUTHOR("Gregory Bean <gbean@codeaurora.org>");
MODULE_DESCRIPTION("Driver for Qualcomm MSM TLMMv2 SoC GPIOs");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:msmgpio");
