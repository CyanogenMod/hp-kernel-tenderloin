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

#ifndef __ASM_ARCH_MSM_GPIO_V2_H
#define __ASM_ARCH_MSM_GPIO_V2_H

#include <mach/gpio-tlmm-v1.h>
#if defined(CONFIG_ARCH_MSM8X60)
#include <mach/gpio-v2-8x60.h>
#endif

/* MSM now supports generic GPIOLIB.  See Documentation/gpio.txt. */

#include <asm-generic/gpio.h>
#include <mach/irqs.h>

#define gpio_get_value __gpio_get_value
#define gpio_set_value __gpio_set_value
#define gpio_cansleep  __gpio_cansleep
#define gpio_to_irq    __gpio_to_irq

/*
 * A GPIO can be set as a direct-connect IRQ.  This can be used to bypass
 * the normal summary-interrupt mechanism for those GPIO lines deemed to be
 * higher priority or otherwise worthy of special treatment, but resources
 * are limited: only a few DC interrupt lines are available.
 * Care must be taken when usurping a GPIO in this manner, as the summary
 * interrupt controller has no idea that the GPIO has been taken away from it.
 * Clients can still register to receive the summary interrupt assigned
 * to that GPIO, which will uninstall it as a direct connect IRQ with
 * no warning.
 *
 * The irq passed to this function is the DC IRQ number, not the
 * irq number seen by the scorpion when the interrupt triggers.  For example,
 * if 0 is specified, then when DC IRQ 0 triggers, the scorpion will see
 * interrupt TLMM_SCSS_DIR_CONN_IRQ_0.
 */
int msm_gpio_install_direct_irq(unsigned gpio, unsigned irq);

#endif /* __ASM_ARCH_MSM_GPIO_V2_H */
