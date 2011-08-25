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
#ifndef __ARCH_ARM_MACH_MSM_GPIOMUX_8X60_H
#define __ARCH_ARM_MACH_MSM_GPIOMUX_8X60_H

#define GPIOMUX_NGPIOS 173

typedef u16 gpiomux_config_t;

enum {
	GPIOMUX_DRV_2MA  = 0UL << 6,
	GPIOMUX_DRV_4MA  = 1UL << 6,
	GPIOMUX_DRV_6MA  = 2UL << 6,
	GPIOMUX_DRV_8MA  = 3UL << 6,
	GPIOMUX_DRV_10MA = 4UL << 6,
	GPIOMUX_DRV_12MA = 5UL << 6,
	GPIOMUX_DRV_14MA = 6UL << 6,
	GPIOMUX_DRV_16MA = 7UL << 6,
};

enum {
	GPIOMUX_FUNC_GPIO = 0UL  << 2,
	GPIOMUX_FUNC_1    = 1UL  << 2,
	GPIOMUX_FUNC_2    = 2UL  << 2,
	GPIOMUX_FUNC_3    = 3UL  << 2,
	GPIOMUX_FUNC_4    = 4UL  << 2,
	GPIOMUX_FUNC_5    = 5UL  << 2,
	GPIOMUX_FUNC_6    = 6UL  << 2,
	GPIOMUX_FUNC_7    = 7UL  << 2,
	GPIOMUX_FUNC_8    = 8UL  << 2,
	GPIOMUX_FUNC_9    = 9UL  << 2,
	GPIOMUX_FUNC_A    = 10UL << 2,
	GPIOMUX_FUNC_B    = 11UL << 2,
	GPIOMUX_FUNC_C    = 12UL << 2,
	GPIOMUX_FUNC_D    = 13UL << 2,
	GPIOMUX_FUNC_E    = 14UL << 2,
	GPIOMUX_FUNC_F    = 15UL << 2,
};

enum {
	GPIOMUX_PULL_NONE   = 0UL,
	GPIOMUX_PULL_DOWN   = 1UL,
	GPIOMUX_PULL_KEEPER = 2UL,
	GPIOMUX_PULL_UP     = 3UL,
};

#endif
