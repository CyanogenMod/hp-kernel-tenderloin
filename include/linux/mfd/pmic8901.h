/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef __PMIC8901_H__
#define __PMIC8901_H__
/*
 * Qualcomm PMIC8901 driver header file
 *
 */

#include <linux/irq.h>
#include <linux/mfd/core.h>

/* PM8901 interrupt numbers */

#define PM8901_MPPS		4

#define PM8901_IRQ_BLOCK_BIT(block, bit) ((block) * 8 + (bit))

/* MPPs [0,N) */
#define PM8901_MPP_IRQ(base, mpp)	((base) + \
					PM8901_IRQ_BLOCK_BIT(6, (mpp)))

#define PM8901_TEMP_ALARM_IRQ(base)	((base) + PM8901_IRQ_BLOCK_BIT(6, 4))
#define PM8901_TEMP_HI_ALARM_IRQ(base)	((base) + PM8901_IRQ_BLOCK_BIT(6, 5))

struct pm8901_chip;

struct pm8901_platform_data {
	/* This table is only needed for misc interrupts. */
	int		irq_base;

	int		num_subdevs;
	struct mfd_cell *sub_devices;
	int		irq_trigger_flags;
};

struct pm8901_gpio_platform_data {
	int	gpio_base;
	int	irq_base;
};

/* chip revision */
#define PM_8901_REV_1p0			0xF1
#define PM_8901_REV_1p1			0xF2
#define PM_8901_REV_2p0			0xF3

int pm8901_read(struct pm8901_chip *pm_chip, u16 addr, u8 *values,
		unsigned int len);
int pm8901_write(struct pm8901_chip *pm_chip, u16 addr, u8 *values,
		 unsigned int len);

int pm8901_rev(struct pm8901_chip *pm_chip);

int pm8901_irq_get_rt_status(struct pm8901_chip *pm_chip, int irq);

int pm8901_reset_pwr_off(int reset);

#endif /* __PMIC8901_H__ */
