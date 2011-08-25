/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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

#ifndef __ARCH_ARM_MACH_MSM_CLOCK_VOTER_H
#define __ARCH_ARM_MACH_MSM_CLOCK_VOTER_H

enum {
	V_EBI_ACPU_CLK,
	V_EBI_DSI_CLK,
	V_EBI_DTV_CLK,
	V_EBI_KGSL_CLK,
	V_EBI_LCDC_CLK,
	V_EBI_MDDI_CLK,
	V_EBI_MDP_CLK,
	V_EBI_PM_CLK,
	V_EBI_TV_CLK,
	V_EBI_USB_CLK,
	V_EBI_VCD_CLK,
	V_EBI_VFE_CLK,
	V_EBI_MSMBUS_CLK,
	V_EBI_ADM0_CLK,
	V_EBI_ADM1_CLK,
	V_DFAB_DSPS_CLK,
	V_DFAB_USB_HS_CLK,
	V_DFAB_SDC1_CLK,
	V_DFAB_SDC2_CLK,
	V_DFAB_SDC3_CLK,
	V_DFAB_SDC4_CLK,
	V_DFAB_SDC5_CLK,

	V_EBI_PM_QOS_CLK,

	V_NR_CLKS
};

struct clk_ops;
extern struct clk_ops clk_ops_voter;

#define CLK_VOTER(clk_name, clk_id, agg_name, clk_dev, clk_flags) {	\
	.con_id = clk_name, \
	.dev_id = clk_dev, \
	.clk = &(struct clk){ \
		.id = V_##clk_id, \
		.flags = clk_flags | CLKFLAG_VOTER, \
		.aggregator = agg_name, \
		.dbg_name = #clk_id, \
		.ops = &clk_ops_voter, \
	}, \
	}

#endif
