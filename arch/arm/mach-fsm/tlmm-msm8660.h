/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef _ARCH_ARM_MACH_MSM_TLMM_MSM8660_H
#define _ARCH_ARM_MACH_MSM_TLMM_MSM8660_H

/* Bits of interest in the GPIO_CFG register.
 */
enum {
	GPIO_OE_BIT = 9,
};

/* Bits of interest in the GPIO_IN_OUT register.
 */
enum {
	GPIO_IN_BIT  = 0,
	GPIO_OUT_BIT = 1
};

/* Bits of interest in the GPIO_INTR_CFG register.
 */
enum {
	INTR_ENABLE_BIT        = 0,
	INTR_POL_CTL_BIT       = 1,
	INTR_DECT_CTL_BIT      = 2,
	INTR_RAW_STATUS_EN_BIT = 3,
};

/* Bits of interest in the GPIO_INTR_STATUS register.
 */
enum {
	INTR_STATUS_BIT = 0,
};

/* "super-user" irq-processor targets.
 */
enum {
	TARGET_PROC_SCORPION = 4,
	TARGET_PROC_NONE     = 7,
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
#define INTR_DECT_CTL_EDGE BIT(INTR_DECT_CTL_BIT)
#define INTR_POL_CTL_HI    BIT(INTR_POL_CTL_BIT)
#define INTR_ENABLE        BIT(INTR_ENABLE_BIT)

#define DC_IRQ_DISABLE (0 << 3)
#define DC_IRQ_ENABLE  (1 << 3)

/*
 * There is no 'DC_POLARITY_LO' because the GIC is incapable
 * of asserting on falling edge or level-low conditions.  Even though
 * the registers allow for low-polarity inputs, the case can never arise.
 */
#define DC_POLARITY_HI (1 << 11)

#define GPIO_INTR_CFG_SU(gpio)    (MSM_TLMM_BASE + 0x0400 + (0x04 * (gpio)))
#define DIR_CONN_INTR_CFG_SU(irq) (MSM_TLMM_BASE + 0x0700 + (0x04 * (irq)))
#define GPIO_CONFIG(gpio)         (MSM_TLMM_BASE + 0x1000 + (0x10 * (gpio)))
#define GPIO_IN_OUT(gpio)         (MSM_TLMM_BASE + 0x1004 + (0x10 * (gpio)))
#define GPIO_INTR_CFG(gpio)       (MSM_TLMM_BASE + 0x1008 + (0x10 * (gpio)))
#define GPIO_INTR_STATUS(gpio)    (MSM_TLMM_BASE + 0x100c + (0x10 * (gpio)))
#define GPIO_OE_CLR(gpio)  (MSM_TLMM_BASE + 0x3100 + (0x04 * ((gpio) / 32)))
#define GPIO_OE_SET(gpio)  (MSM_TLMM_BASE + 0x3120 + (0x04 * ((gpio) / 32)))

#endif
