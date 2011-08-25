/*
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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
 */

#ifndef AVS_H
#define AVS_H

#define VOLTAGE_MIN  1000 /* mV */
#define VOLTAGE_MAX  1250
#define VOLTAGE_STEP 25

int __init avs_init(int (*set_vdd)(int), u32 freq_cnt, u32 freq_idx);
void __exit avs_exit(void);

int avs_adjust_freq(u32 freq_index, int begin);

/* Routines exported from avs_hw.S */
#ifdef CONFIG_MSM_CPU_AVS
u32 avs_test_delays(void);
#else
static inline u32 avs_test_delays(void)
{ return 0; }
#endif

#ifdef CONFIG_MSM_AVS_HW
u32 avs_reset_delays(u32 avsdscr);
u32 avs_get_avscsr(void);
u32 avs_get_avsdscr(void);
u32 avs_get_tscsr(void);
void avs_set_tscsr(u32 to_tscsr);
void avs_disable(void);
#else
static inline u32 avs_reset_delays(u32 avsdscr)
{ return 0; }
static inline u32 avs_get_avscsr(void)
{ return 0; }
static inline u32 avs_get_avsdscr(void)
{ return 0; }
static inline u32 avs_get_tscsr(void)
{ return 0; }
static inline void avs_set_tscsr(u32 to_tscsr) {}
static inline void avs_disable(void) {}
#endif

/*#define AVSDEBUG(x...) pr_info("AVS: " x);*/
#define AVSDEBUG(...)

#define AVS_DISABLE(cpu) do {			\
		if (get_cpu() == (cpu))		\
			avs_disable();		\
		put_cpu();			\
	} while (0);

#define AVS_ENABLE(cpu, x) do {			\
		if (get_cpu() == (cpu))		\
			avs_reset_delays((x));	\
		put_cpu();			\
	} while (0);

#endif /* AVS_H */
