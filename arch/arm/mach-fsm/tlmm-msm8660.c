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
#include <linux/io.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <mach/tlmm.h>
#include <mach/msm_iomap.h>
#include <mach/gpio.h>
#include "tlmm-msm8660.h"

enum msm_tlmm_register {
	SDC4_HDRV_PULL_CTL       = 0x20a0,
	SDC3_HDRV_PULL_CTL       = 0x20a4,
};

struct tlmm_field_cfg {
	enum msm_tlmm_register reg;
	u8                     off;
};

static struct tlmm_field_cfg tlmm_hdrv_cfgs[] = {
	{SDC4_HDRV_PULL_CTL, 6},       /* TLMM_HDRV_SDC4_CLK          */
	{SDC4_HDRV_PULL_CTL, 3},       /* TLMM_HDRV_SDC4_CMD          */
	{SDC4_HDRV_PULL_CTL, 0},       /* TLMM_HDRV_SDC4_DATA         */
	{SDC3_HDRV_PULL_CTL, 6},       /* TLMM_HDRV_SDC3_CLK          */
	{SDC3_HDRV_PULL_CTL, 3},       /* TLMM_HDRV_SDC3_CMD          */
	{SDC3_HDRV_PULL_CTL, 0},       /* TLMM_HDRV_SDC3_DATA         */
};

static struct tlmm_field_cfg tlmm_pull_cfgs[] = {
	{SDC4_HDRV_PULL_CTL, 11},       /* TLMM_PULL_SDC4_CMD       */
	{SDC4_HDRV_PULL_CTL, 9},        /* TLMM_PULL_SDC4_DATA      */
	{SDC3_HDRV_PULL_CTL, 11},       /* TLMM_PULL_SDC3_CMD       */
	{SDC3_HDRV_PULL_CTL, 9},        /* TLMM_PULL_SDC3_DATA      */
};

static DEFINE_SPINLOCK(tlmm_lock);

static void msm_tlmm_set_field(struct tlmm_field_cfg *configs,
			unsigned               id,
			unsigned               width,
			unsigned               val)
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

int msm_gpio_install_direct_irq(unsigned gpio, unsigned irq)
{
	unsigned long irq_flags;

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
	writel(DC_POLARITY_HI |	TARGET_PROC_SCORPION | (gpio << 3),
		DIR_CONN_INTR_CFG_SU(irq));

	spin_unlock_irqrestore(&tlmm_lock, irq_flags);

	return 0;
}
EXPORT_SYMBOL(msm_gpio_install_direct_irq);
