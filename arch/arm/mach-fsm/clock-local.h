/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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

#ifndef __ARCH_ARM_MACH_MSM_CLOCK_LOCAL_H
#define __ARCH_ARM_MACH_MSM_CLOCK_LOCAL_H

#include <linux/spinlock.h>
#include "clock.h"

/*
 * Bit manipulation macros
 */
#define B(x)	BIT(x)
#define BM(msb, lsb)	(((((uint32_t)-1) << (31-msb)) >> (31-msb+lsb)) << lsb)
#define BVAL(msb, lsb, val)	(((val) << lsb) & BM(msb, lsb))

/*
 * Clock types
 */
#define MND		1 /* Integer predivider and fractional MN:D divider. */
#define BASIC		2 /* Integer divider. */
#define NORATE		3 /* Just on/off. */
#define RESET		4 /* Reset only. */

/*
 * IDs for invalid sources, source selects, and XOs
 */
#define SRC_NONE	-1
#define SRC_SEL_NONE	-1
#define XO_NONE		-1

/*
 * Halt/Status Checking Mode Macros
 */
#define NOCHECK 0	/* No bit to check, do nothing */
#define HALT	1	/* Bit polarity: 1 = halted */
#define ENABLE	2	/* Bit polarity: 1 = running */
#define DELAY	3	/* No bit to check, just delay */

/*
 * Generic frequency-definition structs and macros
 */
struct clk_freq_tbl {
	const uint32_t	freq_hz;
	const int	src;
	const uint32_t	md_val;
	const uint32_t	ns_val;
	const uint32_t	cc_val;
	uint32_t	mnd_en_mask;
	const unsigned	sys_vdd;
	void 		*const extra_freq_data;
};

/* Some clocks have two banks to avoid glitches when switching frequencies.
 * The unused bank is programmed while running on the other bank, and
 * switched to afterwards. The following two structs describe the banks. */
struct bank_mask_info {
	void *const md_reg;
	const uint32_t	ns_mask;
	const uint32_t	rst_mask;
	const uint32_t	mnd_en_mask;
	const uint32_t	mode_mask;
};

struct banked_mnd_masks {
	const uint32_t			bank_sel_mask;
	const struct bank_mask_info	bank0_mask;
	const struct bank_mask_info	bank1_mask;
};

#define F_RAW(f, s, m_v, n_v, c_v, m_m, v, e) { \
	.freq_hz = f, \
	.src = s, \
	.md_val = m_v, \
	.ns_val = n_v, \
	.cc_val = c_v, \
	.mnd_en_mask = m_m, \
	.sys_vdd = v, \
	.extra_freq_data = e, \
	}
#define FREQ_END	(UINT_MAX-1)
#define F_END	F_RAW(FREQ_END, SRC_NONE, 0, 0, 0, 0, 0, NULL)
#define PLL_RATE(r, l, m, n, v, d) { l, m, n, v, (d>>1) }

/*
 * Generic clock-definition struct and macros
 */
struct clk_local {
	int		count;
	const uint32_t	type;
	void		*const ns_reg;
	void		*const cc_reg;
	void		*const md_reg;
	void		*const reset_reg;
	void		*const halt_reg;
	const uint32_t	reset_mask;
	const uint16_t	halt_check;
	const uint16_t	halt_bit;
	const uint32_t	br_en_mask;
	const uint32_t	root_en_mask;
	const uint32_t	ns_mask;
	const uint32_t	cc_mask;
	const uint32_t	test_vector;
	struct banked_mnd_masks *const banked_mnd_masks;
	const int	parent;
	const uint32_t	*const children;
	void		(*set_rate)(struct clk_local *, struct clk_freq_tbl *);
	struct clk_freq_tbl *const freq_tbl;
	struct clk_freq_tbl *current_freq;
};

#define C(x)		L_##x##_CLK
#define L_NONE_CLK	-1
#define CLK(id, t, ns_r, cc_r, md_r, r_r, r_m, h_r, h_c, h_b, br, root, \
		n_m, c_m, s_fn, tbl, bmnd, par, chld_lst, tv) \
	[C(id)] = { \
	.type = t, \
	.ns_reg = ns_r, \
	.cc_reg = cc_r, \
	.md_reg = md_r, \
	.reset_reg = r_r, \
	.halt_reg = h_r, \
	.halt_check = h_c, \
	.halt_bit = h_b, \
	.reset_mask = r_m, \
	.br_en_mask = br, \
	.root_en_mask = root, \
	.ns_mask = n_m, \
	.cc_mask = c_m, \
	.test_vector = tv, \
	.banked_mnd_masks = bmnd, \
	.parent = C(par), \
	.children = chld_lst, \
	.set_rate = s_fn, \
	.freq_tbl = tbl, \
	.current_freq = &local_dummy_freq, \
	}

/*
 * Convenience macros
 */
#define set_1rate(clk) \
	local_clk_set_rate(C(clk), soc_clk_local_tbl[C(clk)].freq_tbl->freq_hz)

/*
 * SYS_VDD voltage levels
 */
enum sys_vdd_level {
	LOW,
	NOMINAL,
	HIGH,
	NUM_SYS_VDD_LEVELS
};

/*
 * Clock source descriptions
 */
struct clk_source {
	int		(*const enable_func)(unsigned src, unsigned enable);
	const signed 	par;
};

/*
 * Variables from SoC-specific clock drivers
 */
extern struct clk_local		soc_clk_local_tbl[];
extern struct clk_source	soc_clk_sources[];

/*
 * Variables from clock-local driver
 */
extern spinlock_t		local_clock_reg_lock;
extern struct clk_freq_tbl	local_dummy_freq;

/*
 * Local-clock APIs
 */
int local_src_enable(int src);
int local_src_disable(int src);
void local_clk_enable_reg(unsigned id);
void local_clk_disable_reg(unsigned id);
int local_vote_sys_vdd(enum sys_vdd_level level);
int local_unvote_sys_vdd(enum sys_vdd_level level);

/*
 * clk_ops APIs
 */
int local_clk_enable(unsigned id);
void local_clk_disable(unsigned id);
void local_clk_auto_off(unsigned id);
int local_clk_set_rate(unsigned id, unsigned rate);
int local_clk_set_min_rate(unsigned id, unsigned rate);
int local_clk_set_max_rate(unsigned id, unsigned rate);
unsigned local_clk_get_rate(unsigned id);
int local_clk_list_rate(unsigned id, unsigned n);
unsigned local_clk_is_enabled(unsigned id);
long local_clk_round_rate(unsigned id, unsigned rate);

/*
 * Required SoC-specific functions, implemented for every supported SoC
 */
int soc_update_sys_vdd(enum sys_vdd_level level);
int soc_set_pwr_rail(unsigned id, int enable);
int soc_clk_measure_rate(unsigned id);
int soc_clk_set_flags(unsigned id, unsigned flags);
int soc_clk_reset(unsigned id, enum clk_reset_action action);

/*
 * Generic set-rate implementations
 */
void set_rate_basic(struct clk_local *clk, struct clk_freq_tbl *nf);
void set_rate_mnd(struct clk_local *clk, struct clk_freq_tbl *nf);
void set_rate_nop(struct clk_local *clk, struct clk_freq_tbl *nf);

#endif /* __ARCH_ARM_MACH_MSM_CLOCK_LOCAL_H */

