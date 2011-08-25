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
#ifndef __ARCH_ARM_MACH_MSM_GPIOMUX_7X00_H
#define __ARCH_ARM_MACH_MSM_GPIOMUX_7X00_H

#if defined(CONFIG_ARCH_MSM7X30)
#define GPIOMUX_NGPIOS 182
#elif defined(CONFIG_ARCH_QSD8X50)
#define GPIOMUX_NGPIOS 165
#else
#define GPIOMUX_NGPIOS 133
#endif

typedef u32 gpiomux_config_t;

enum {
	GPIOMUX_DRV_2MA  = 0UL << 17,
	GPIOMUX_DRV_4MA  = 1UL << 17,
	GPIOMUX_DRV_6MA  = 2UL << 17,
	GPIOMUX_DRV_8MA  = 3UL << 17,
	GPIOMUX_DRV_10MA = 4UL << 17,
	GPIOMUX_DRV_12MA = 5UL << 17,
	GPIOMUX_DRV_14MA = 6UL << 17,
	GPIOMUX_DRV_16MA = 7UL << 17,
};

enum {
	GPIOMUX_FUNC_GPIO = 0UL,
	GPIOMUX_FUNC_1    = 1UL,
	GPIOMUX_FUNC_2    = 2UL,
	GPIOMUX_FUNC_3    = 3UL,
	GPIOMUX_FUNC_4    = 4UL,
	GPIOMUX_FUNC_5    = 5UL,
	GPIOMUX_FUNC_6    = 6UL,
	GPIOMUX_FUNC_7    = 7UL,
	GPIOMUX_FUNC_8    = 8UL,
	GPIOMUX_FUNC_9    = 9UL,
	GPIOMUX_FUNC_A    = 10UL,
	GPIOMUX_FUNC_B    = 11UL,
	GPIOMUX_FUNC_C    = 12UL,
	GPIOMUX_FUNC_D    = 13UL,
	GPIOMUX_FUNC_E    = 14UL,
	GPIOMUX_FUNC_F    = 15UL,
};

enum {
	GPIOMUX_PULL_NONE   = 0UL << 15,
	GPIOMUX_PULL_DOWN   = 1UL << 15,
	GPIOMUX_PULL_KEEPER = 2UL << 15,
	GPIOMUX_PULL_UP     = 3UL << 15,
};

#endif
