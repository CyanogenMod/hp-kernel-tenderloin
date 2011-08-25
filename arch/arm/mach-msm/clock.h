/* arch/arm/mach-msm/clock.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007-2011, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ARCH_ARM_MACH_MSM_CLOCK_H
#define __ARCH_ARM_MACH_MSM_CLOCK_H

#include <linux/init.h>
#include <linux/types.h>
#include <linux/list.h>
#include <mach/clk.h>

#define CLKFLAG_INVERT			0x00000001
#define CLKFLAG_NOINVERT		0x00000002
#define CLKFLAG_NONEST			0x00000004
#define CLKFLAG_NORESET			0x00000008
#define CLKFLAG_VOTER			0x00000010

#define CLK_FIRST_AVAILABLE_FLAG	0x00000100
#define CLKFLAG_AUTO_OFF		0x00000200
#define CLKFLAG_MIN			0x00000400
#define CLKFLAG_MAX			0x00000800

struct clk_ops {
	int (*enable)(unsigned id);
	void (*disable)(unsigned id);
	void (*auto_off)(unsigned id);
	int (*reset)(unsigned id, enum clk_reset_action action);
	int (*set_rate)(unsigned id, unsigned rate);
	int (*set_min_rate)(unsigned id, unsigned rate);
	int (*set_max_rate)(unsigned id, unsigned rate);
	int (*set_flags)(unsigned id, unsigned flags);
	unsigned (*get_rate)(unsigned id);
	int (*list_rate)(unsigned id, unsigned n);
	int (*measure_rate)(unsigned id);
	unsigned (*is_enabled)(unsigned id);
	long (*round_rate)(unsigned id, unsigned rate);
	int (*set_parent)(unsigned id, struct clk *parent);
};

struct clk {
	uint32_t id;
	uint32_t remote_id;
	uint32_t flags;
	struct clk_ops *ops;
	const char *dbg_name;
	struct list_head list;
	struct hlist_head voters;
	const char *aggregator;
};

#define OFF CLKFLAG_AUTO_OFF
#define CLK_MIN CLKFLAG_MIN
#define CLK_MAX CLKFLAG_MAX
#define CLK_MINMAX (CLK_MIN | CLK_MAX)

enum clkvote_client {
	CLKVOTE_ACPUCLK = 0,
	CLKVOTE_PMQOS,
	CLKVOTE_MAX,
};

#ifdef CONFIG_ARCH_MSM7X30
void __init msm_clk_soc_set_ops(struct clk *clk);
#else
static inline void __init msm_clk_soc_set_ops(struct clk *clk) { }
#endif

#if defined(CONFIG_ARCH_MSM7X30) || defined(CONFIG_ARCH_MSM8X60)
void __init msm_clk_soc_init(void);
#else
static inline void __init msm_clk_soc_init(void) { }
#endif

#ifdef CONFIG_DEBUG_FS
int __init clock_debug_init(struct list_head *head);
int __init clock_debug_add(struct clk *clock);
void clock_debug_print_enabled(void);
#else
static inline int __init clock_debug_init(struct list_head *head) { return 0; }
static inline int __init clock_debug_add(struct clk *clock) { return 0; }
static inline void clock_debug_print_enabled(void) { return; }
#endif

extern struct clk_ops clk_ops_remote;

static inline int msm_clock_require_tcxo(unsigned long *reason, int nbits)
{
	return 0;
}

static inline int msm_clock_get_name(uint32_t id, char *name, uint32_t size)
{
	return 0;
}

int ebi1_clk_set_min_rate(enum clkvote_client client, unsigned long rate);
unsigned long clk_get_max_axi_khz(void);

#endif

