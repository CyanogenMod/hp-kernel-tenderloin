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
#ifndef __ARCH_ARM_MACH_MSM_GPIOMUX_H
#define __ARCH_ARM_MACH_MSM_GPIOMUX_H

#include <linux/bitops.h>

#if defined(CONFIG_MSM8660_TLMM)
#include "gpiomux-8x60.h"
#else
#include "gpiomux-7x00.h"
#endif

/**
 * struct msm_gpiomux_config: gpiomux settings for one gpio line.
 *
 * A complete gpiomux config is the bitwise-or of a drive-strength,
 * function, and pull.  For functions other than GPIO, the OE
 * is hard-wired according to the function.  For GPIO mode,
 * OE is controlled by gpiolib.
 *
 * Available settings differ by target; see the gpiomux header
 * specific to your target arch for available configurations.
 *
 * @active: The configuration to be installed when the line is
 * active, or its reference count is > 0.
 * @suspended: The configuration to be installed when the line
 * is suspended, or its reference count is 0.
 * @ref: The reference count of the line.  For internal use of
 * the gpiomux framework only.
 */
struct msm_gpiomux_config {
	gpiomux_config_t active;
	gpiomux_config_t suspended;
	unsigned         ref;
};

/**
 * @GPIOMUX_VALID:	If set, the config field contains 'good data'.
 *                      The absence of this bit will prevent the gpiomux
 *			system from applying the configuration under all
 *			circumstances.
 */
enum {
	GPIOMUX_VALID	 = BIT(sizeof(gpiomux_config_t) * BITS_PER_BYTE - 1),
	GPIOMUX_CTL_MASK = GPIOMUX_VALID,
};

/* Each architecture must provide its own instance of this table.
 * To avoid having gpiomux manage any given gpio, one or both of
 * the entries can be or'ed with GPIOMUX_UNUSED - the presence
 * of that flag will prevent the configuration from being applied
 * during state transitions.
 */
extern struct msm_gpiomux_config msm_gpiomux_configs[GPIOMUX_NGPIOS];

/* Increment a gpio's reference count, possibly activating the line. */
int __must_check msm_gpiomux_get(unsigned gpio);

/* Decrement a gpio's reference count, possibly suspending the line. */
int msm_gpiomux_put(unsigned gpio);

/* Install a new configuration to the gpio line.  To avoid overwriting
 * a configuration, leave the VALID bit out.
 */
int msm_gpiomux_write(unsigned gpio,
		      gpiomux_config_t active,
		      gpiomux_config_t suspended);

/* Architecture-internal function for use by the framework only.
 * This function can assume the following:
 * - the gpio value has passed a bounds-check
 * - the gpiomux spinlock has been obtained
 *
 * This function is not for public consumption.  External users
 * should use msm_gpiomux_write.
 */
void __msm_gpiomux_write(unsigned gpio, gpiomux_config_t val);

#endif
