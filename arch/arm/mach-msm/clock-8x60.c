/* Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <mach/msm_iomap.h>
#include <mach/clk.h>
#include <mach/msm_xo.h>
#include <mach/scm-io.h>

#include "clock-local.h"
#include "clock-8x60.h"
#include "rpm.h"
#include "rpm-regulator.h"

#ifdef CONFIG_MSM_SECURE_IO
#undef readl
#undef writel
#define readl secure_readl
#define writel secure_writel
#endif

#define REG(off)	(MSM_CLK_CTL_BASE + (off))
#define REG_MM(off)	(MSM_MMSS_CLK_CTL_BASE + (off))
#define REG_LPA(off)	(MSM_LPASS_CLK_CTL_BASE + (off))

/* Peripheral clock registers. */
#define CE2_HCLK_CTL_REG			REG(0x2740)
#define CLK_HALT_CFPB_STATEA_REG		REG(0x2FCC)
#define CLK_HALT_CFPB_STATEB_REG		REG(0x2FD0)
#define CLK_HALT_CFPB_STATEC_REG		REG(0x2FD4)
#define CLK_HALT_DFAB_STATE_REG			REG(0x2FC8)
#define CLK_HALT_MSS_SMPSS_MISC_STATE_REG	REG(0x2FDC)
#define CLK_HALT_SFPB_MISC_STATE_REG		REG(0x2FD8)
#define CLK_TEST_REG				REG(0x2FA0)
#define GSBIn_HCLK_CTL_REG(n)			REG(0x29C0+(0x20*((n)-1)))
#define GSBIn_QUP_APPS_MD_REG(n)		REG(0x29C8+(0x20*((n)-1)))
#define GSBIn_QUP_APPS_NS_REG(n)		REG(0x29CC+(0x20*((n)-1)))
#define GSBIn_RESET_REG(n)			REG(0x29DC+(0x20*((n)-1)))
#define GSBIn_UART_APPS_MD_REG(n)		REG(0x29D0+(0x20*((n)-1)))
#define GSBIn_UART_APPS_NS_REG(n)		REG(0x29D4+(0x20*((n)-1)))
#define PDM_CLK_NS_REG				REG(0x2CC0)
#define BB_PLL_ENA_SC0_REG			REG(0x34C0)
#define BB_PLL0_STATUS_REG			REG(0x30D8)
#define BB_PLL6_STATUS_REG			REG(0x3118)
#define BB_PLL8_L_VAL_REG			REG(0x3144)
#define BB_PLL8_M_VAL_REG			REG(0x3148)
#define BB_PLL8_MODE_REG			REG(0x3140)
#define BB_PLL8_N_VAL_REG			REG(0x314C)
#define BB_PLL8_STATUS_REG			REG(0x3158)
#define PLLTEST_PAD_CFG_REG			REG(0x2FA4)
#define PMEM_ACLK_CTL_REG			REG(0x25A0)
#define PPSS_HCLK_CTL_REG			REG(0x2580)
#define PRNG_CLK_NS_REG				REG(0x2E80)
#define RINGOSC_NS_REG				REG(0x2DC0)
#define RINGOSC_STATUS_REG			REG(0x2DCC)
#define RINGOSC_TCXO_CTL_REG			REG(0x2DC4)
#define SC0_U_CLK_BRANCH_ENA_VOTE_REG		REG(0x3080)
#define SC1_U_CLK_BRANCH_ENA_VOTE_REG		REG(0x30A0)
#define SC0_U_CLK_SLEEP_ENA_VOTE_REG		REG(0x3084)
#define SC1_U_CLK_SLEEP_ENA_VOTE_REG		REG(0x30A4)
#define SDCn_APPS_CLK_MD_REG(n)			REG(0x2828+(0x20*((n)-1)))
#define SDCn_APPS_CLK_NS_REG(n)			REG(0x282C+(0x20*((n)-1)))
#define SDCn_HCLK_CTL_REG(n)			REG(0x2820+(0x20*((n)-1)))
#define SDCn_RESET_REG(n)			REG(0x2830+(0x20*((n)-1)))
#define TSIF_HCLK_CTL_REG			REG(0x2700)
#define TSIF_REF_CLK_MD_REG			REG(0x270C)
#define TSIF_REF_CLK_NS_REG			REG(0x2710)
#define TSSC_CLK_CTL_REG			REG(0x2CA0)
#define USB_FSn_HCLK_CTL_REG(n)			REG(0x2960+(0x20*((n)-1)))
#define USB_FSn_RESET_REG(n)			REG(0x2974+(0x20*((n)-1)))
#define USB_FSn_SYSTEM_CLK_CTL_REG(n)		REG(0x296C+(0x20*((n)-1)))
#define USB_FSn_XCVR_FS_CLK_MD_REG(n)		REG(0x2964+(0x20*((n)-1)))
#define USB_FSn_XCVR_FS_CLK_NS_REG(n)		REG(0x2968+(0x20*((n)-1)))
#define USB_HS1_HCLK_CTL_REG			REG(0x2900)
#define USB_HS1_RESET_REG			REG(0x2910)
#define USB_HS1_XCVR_FS_CLK_MD_REG		REG(0x2908)
#define USB_HS1_XCVR_FS_CLK_NS_REG		REG(0x290C)
#define USB_PHY0_RESET_REG			REG(0x2E20)

/* Multimedia clock registers. */
#define AHB_EN_REG				REG_MM(0x0008)
#define AHB_EN2_REG				REG_MM(0x0038)
#define AHB_NS_REG				REG_MM(0x0004)
#define AXI_NS_REG				REG_MM(0x0014)
#define CAMCLK_CC_REG				REG_MM(0x0140)
#define CAMCLK_MD_REG				REG_MM(0x0144)
#define CAMCLK_NS_REG				REG_MM(0x0148)
#define CSI_CC_REG				REG_MM(0x0040)
#define CSI_NS_REG				REG_MM(0x0048)
#define DBG_BUS_VEC_A_REG			REG_MM(0x01C8)
#define DBG_BUS_VEC_B_REG			REG_MM(0x01CC)
#define DBG_BUS_VEC_C_REG			REG_MM(0x01D0)
#define DBG_BUS_VEC_D_REG			REG_MM(0x01D4)
#define DBG_BUS_VEC_E_REG			REG_MM(0x01D8)
#define DBG_BUS_VEC_F_REG			REG_MM(0x01DC)
#define DBG_BUS_VEC_H_REG			REG_MM(0x01E4)
#define DBG_CFG_REG_HS_REG			REG_MM(0x01B4)
#define DBG_CFG_REG_LS_REG			REG_MM(0x01B8)
#define GFX2D0_CC_REG				REG_MM(0x0060)
#define GFX2D0_MD0_REG				REG_MM(0x0064)
#define GFX2D0_MD1_REG				REG_MM(0x0068)
#define GFX2D0_NS_REG				REG_MM(0x0070)
#define GFX2D1_CC_REG				REG_MM(0x0074)
#define GFX2D1_MD0_REG				REG_MM(0x0078)
#define GFX2D1_MD1_REG				REG_MM(0x006C)
#define GFX2D1_NS_REG				REG_MM(0x007C)
#define GFX3D_CC_REG				REG_MM(0x0080)
#define GFX3D_MD0_REG				REG_MM(0x0084)
#define GFX3D_MD1_REG				REG_MM(0x0088)
#define GFX3D_NS_REG				REG_MM(0x008C)
#define IJPEG_CC_REG				REG_MM(0x0098)
#define IJPEG_MD_REG				REG_MM(0x009C)
#define IJPEG_NS_REG				REG_MM(0x00A0)
#define JPEGD_CC_REG				REG_MM(0x00A4)
#define JPEGD_NS_REG				REG_MM(0x00AC)
#define MAXI_EN_REG				REG_MM(0x0018)
#define MAXI_EN3_REG				REG_MM(0x002C)
#define MDP_CC_REG				REG_MM(0x00C0)
#define MDP_MD0_REG				REG_MM(0x00C4)
#define MDP_MD1_REG				REG_MM(0x00C8)
#define MDP_NS_REG				REG_MM(0x00D0)
#define MISC_CC_REG				REG_MM(0x0058)
#define MISC_CC2_REG				REG_MM(0x005C)
#define PIXEL_CC_REG				REG_MM(0x00D4)
#define PIXEL_CC2_REG				REG_MM(0x0120)
#define PIXEL_MD_REG				REG_MM(0x00D8)
#define PIXEL_NS_REG				REG_MM(0x00DC)
#define MM_PLL0_MODE_REG			REG_MM(0x0300)
#define MM_PLL0_STATUS_REG			REG_MM(0x0318)
#define MM_PLL1_MODE_REG			REG_MM(0x031C)
#define MM_PLL1_STATUS_REG			REG_MM(0x0334)
#define MM_PLL2_CONFIG_REG			REG_MM(0x0348)
#define MM_PLL2_L_VAL_REG			REG_MM(0x033C)
#define MM_PLL2_M_VAL_REG			REG_MM(0x0340)
#define MM_PLL2_MODE_REG			REG_MM(0x0338)
#define MM_PLL2_N_VAL_REG			REG_MM(0x0344)
#define MM_PLL2_STATUS_REG			REG_MM(0x0350)
#define ROT_CC_REG				REG_MM(0x00E0)
#define ROT_NS_REG				REG_MM(0x00E8)
#define SAXI_EN_REG				REG_MM(0x0030)
#define SW_RESET_AHB_REG			REG_MM(0x020C)
#define SW_RESET_ALL_REG			REG_MM(0x0204)
#define SW_RESET_AXI_REG			REG_MM(0x0208)
#define SW_RESET_CORE_REG			REG_MM(0x0210)
#define TV_CC_REG				REG_MM(0x00EC)
#define TV_CC2_REG				REG_MM(0x0124)
#define TV_MD_REG				REG_MM(0x00F0)
#define TV_NS_REG				REG_MM(0x00F4)
#define VCODEC_CC_REG				REG_MM(0x00F8)
#define VCODEC_MD0_REG				REG_MM(0x00FC)
#define VCODEC_MD1_REG				REG_MM(0x0128)
#define VCODEC_NS_REG				REG_MM(0x0100)
#define VFE_CC_REG				REG_MM(0x0104)
#define VFE_MD_REG				REG_MM(0x0108)
#define VFE_NS_REG				REG_MM(0x010C)
#define VPE_CC_REG				REG_MM(0x0110)
#define VPE_NS_REG				REG_MM(0x0118)

/* Low-power Audio clock registers. */
#define LCC_CLK_LS_DEBUG_CFG_REG		REG_LPA(0x00A8)
#define LCC_CODEC_I2S_MIC_MD_REG		REG_LPA(0x0064)
#define LCC_CODEC_I2S_MIC_NS_REG		REG_LPA(0x0060)
#define LCC_CODEC_I2S_MIC_STATUS_REG		REG_LPA(0x0068)
#define LCC_CODEC_I2S_SPKR_MD_REG		REG_LPA(0x0070)
#define LCC_CODEC_I2S_SPKR_NS_REG		REG_LPA(0x006C)
#define LCC_CODEC_I2S_SPKR_STATUS_REG		REG_LPA(0x0074)
#define LCC_MI2S_MD_REG				REG_LPA(0x004C)
#define LCC_MI2S_NS_REG				REG_LPA(0x0048)
#define LCC_MI2S_STATUS_REG			REG_LPA(0x0050)
#define LCC_PCM_MD_REG				REG_LPA(0x0058)
#define LCC_PCM_NS_REG				REG_LPA(0x0054)
#define LCC_PCM_STATUS_REG			REG_LPA(0x005C)
#define LCC_PLL0_CONFIG_REG			REG_LPA(0x0014)
#define LCC_PLL0_L_VAL_REG			REG_LPA(0x0004)
#define LCC_PLL0_M_VAL_REG			REG_LPA(0x0008)
#define LCC_PLL0_MODE_REG			REG_LPA(0x0000)
#define LCC_PLL0_N_VAL_REG			REG_LPA(0x000C)
#define LCC_PLL0_STATUS_REG			REG_LPA(0x0018)
#define LCC_PRI_PLL_CLK_CTL_REG			REG_LPA(0x00C4)
#define LCC_SPARE_I2S_MIC_MD_REG		REG_LPA(0x007C)
#define LCC_SPARE_I2S_MIC_NS_REG		REG_LPA(0x0078)
#define LCC_SPARE_I2S_MIC_STATUS_REG		REG_LPA(0x0080)
#define LCC_SPARE_I2S_SPKR_MD_REG		REG_LPA(0x0088)
#define LCC_SPARE_I2S_SPKR_NS_REG		REG_LPA(0x0084)
#define LCC_SPARE_I2S_SPKR_STATUS_REG		REG_LPA(0x008C)

/* MUX source input identifiers. */
#define SRC_SEL_BB_PXO		0
#define SRC_SEL_BB_MXO		1
#define SRC_SEL_BB_CXO		SRC_SEL_BB_PXO
#define SRC_SEL_BB_PLL0		2
#define SRC_SEL_BB_PLL8		3
#define SRC_SEL_BB_PLL6		4
#define SRC_SEL_BB_GND		6
#define SRC_SEL_MM_PXO		0
#define SRC_SEL_MM_PLL0		1
#define SRC_SEL_MM_PLL1		1
#define SRC_SEL_MM_PLL2		3
#define SRC_SEL_MM_GPERF	2
#define SRC_SEL_MM_GPLL0	3
#define SRC_SEL_MM_MXO		4
#define SRC_SEL_MM_GND		6
#define SRC_SEL_XO_CXO		0
#define SRC_SEL_XO_PXO		1
#define SRC_SEL_XO_MXO		2
#define SRC_SEL_XO_GND		3
#define SRC_SEL_LPA_PXO		0
#define SRC_SEL_LPA_CXO		1
#define SRC_SEL_LPA_PLL0	2
#define SRC_SEL_LPA_GND		6

/* Source name mapping. */
#define SRC_BB_PXO		PXO
#define SRC_BB_MXO		SRC_NONE
#define SRC_BB_CXO		CXO
#define SRC_BB_PLL0		PLL_0
#define SRC_BB_PLL8		PLL_8
#define SRC_BB_PLL6		PLL_6
#define SRC_BB_GND		SRC_NONE
#define SRC_MM_PXO		PXO
#define SRC_MM_PLL0		PLL_1
#define SRC_MM_PLL1		PLL_2
#define SRC_MM_PLL2		PLL_3
#define SRC_MM_GPERF		PLL_8
#define SRC_MM_GPLL0		PLL_0
#define SRC_MM_MXO		SRC_NONE
#define SRC_MM_GND		SRC_NONE
#define SRC_XO_CXO		CXO
#define SRC_XO_PXO		PXO
#define SRC_XO_MXO		SRC_NONE
#define SRC_XO_GND		SRC_NONE
#define SRC_LPA_PXO		PXO
#define SRC_LPA_CXO		CXO
#define SRC_LPA_PLL0		PLL_4
#define SRC_LPA_GND		SRC_NONE

/* Test Vector Macros */
#define TEST_TYPE_PER_LS	1
#define TEST_TYPE_PER_HS	2
#define TEST_TYPE_MM_LS		3
#define TEST_TYPE_MM_HS		4
#define TEST_TYPE_LPA		5
#define TEST_TYPE_SHIFT		24
#define TEST_CLK_SEL_MASK	BM(23, 0)
#define TEST_VECTOR(s, t)	(((t) << TEST_TYPE_SHIFT) | BVAL(23, 0, (s)))
#define TEST_PER_LS(s)		TEST_VECTOR((s), TEST_TYPE_PER_LS)
#define TEST_PER_HS(s)		TEST_VECTOR((s), TEST_TYPE_PER_HS)
#define TEST_MM_LS(s)		TEST_VECTOR((s), TEST_TYPE_MM_LS)
#define TEST_MM_HS(s)		TEST_VECTOR((s), TEST_TYPE_MM_HS)
#define TEST_LPA(s)		TEST_VECTOR((s), TEST_TYPE_LPA)

/*
 * SoC-specific Set-Rate Functions
 */

static void set_rate_cam(struct clk_local *clk, struct clk_freq_tbl *nf)
{
	uint32_t cc_reg_val;

	/* Assert MND reset. */
	cc_reg_val = readl(clk->cc_reg);
	cc_reg_val |= BIT(8);
	writel(cc_reg_val, clk->cc_reg);

	/* Program M and D values. */
	writel(nf->md_val, clk->md_reg);

	/* Program MN counter Enable and Mode. */
	cc_reg_val &= ~(clk->cc_mask);
	cc_reg_val |= nf->cc_val;
	writel(cc_reg_val, clk->cc_reg);

	/* Deassert MND reset. */
	cc_reg_val &= ~BIT(8);
	writel(cc_reg_val, clk->cc_reg);
}

/* Unlike other clocks, the TV rate is adjusted through PLL
 * re-programming. It is also routed through an MND divider. */
static void set_rate_tv(struct clk_local *clk, struct clk_freq_tbl *nf)
{
	struct pll_rate *rate = nf->extra_freq_data;
	uint32_t pll_mode, pll_config, misc_cc2;

	/* Disable PLL output. */
	pll_mode = readl(MM_PLL2_MODE_REG);
	pll_mode &= ~BIT(0);
	writel(pll_mode, MM_PLL2_MODE_REG);

	/* Assert active-low PLL reset. */
	pll_mode &= ~BIT(2);
	writel(pll_mode, MM_PLL2_MODE_REG);

	/* Program L, M and N values. */
	writel(rate->l_val, MM_PLL2_L_VAL_REG);
	writel(rate->m_val, MM_PLL2_M_VAL_REG);
	writel(rate->n_val, MM_PLL2_N_VAL_REG);

	/* Configure MN counter, post-divide, VCO, and i-bits. */
	pll_config = readl(MM_PLL2_CONFIG_REG);
	pll_config &= ~(BM(22, 20) | BM(18, 0));
	pll_config |= rate->n_val ? BIT(22) : 0;
	pll_config |= BVAL(21, 20, rate->post_div);
	pll_config |= BVAL(17, 16, rate->vco);
	pll_config |= rate->i_bits;
	writel(pll_config, MM_PLL2_CONFIG_REG);

	/* Configure MND. */
	set_rate_mnd(clk, nf);

	/* Configure hdmi_ref_clk to be equal to the TV clock rate. */
	misc_cc2 = readl(MISC_CC2_REG);
	misc_cc2 &= ~(BIT(28)|BM(21, 18));
	misc_cc2 |= (BIT(28)|BVAL(21, 18, (nf->ns_val >> 14) & 0x3));
	writel(misc_cc2, MISC_CC2_REG);

	/* De-assert active-low PLL reset. */
	pll_mode |= BIT(2);
	writel(pll_mode, MM_PLL2_MODE_REG);

	/* Enable PLL output. */
	pll_mode |= BIT(0);
	writel(pll_mode, MM_PLL2_MODE_REG);
}

static void set_rate_mnd_banked(struct clk_local *clk, struct clk_freq_tbl *nf)
{
	struct bank_masks *banks = clk->bank_masks;
	const struct bank_mask_info *new_bank_masks;
	const struct bank_mask_info *old_bank_masks;
	uint32_t ns_reg_val, cc_reg_val;
	uint32_t bank_sel;

	/* Determine active bank and program the other one. If the clock is
	 * off, program the active bank since bank switching won't work if
	 * both banks aren't running. */
	cc_reg_val = readl(clk->cc_reg);
	bank_sel = !!(cc_reg_val & banks->bank_sel_mask);
	 /* If clock isn't running, don't switch banks. */
	bank_sel ^= (clk->count == 0 || clk->current_freq->freq_hz == 0);
	if (bank_sel == 0) {
		new_bank_masks = &banks->bank1_mask;
		old_bank_masks = &banks->bank0_mask;
	} else {
		new_bank_masks = &banks->bank0_mask;
		old_bank_masks = &banks->bank1_mask;
	}

	ns_reg_val = readl(clk->ns_reg);

	/* Assert bank MND reset. */
	ns_reg_val |= new_bank_masks->rst_mask;
	writel(ns_reg_val, clk->ns_reg);

	/* Program NS only if the clock is enabled, since the NS will be set
	 * as part of the enable procedure and should remain with a low-power
	 * MUX input selected until then. */
	if (clk->count) {
		ns_reg_val &= ~(new_bank_masks->ns_mask);
		ns_reg_val |= (nf->ns_val & new_bank_masks->ns_mask);
		writel(ns_reg_val, clk->ns_reg);
	}

	writel(nf->md_val, new_bank_masks->md_reg);

	/* Enable counter only if clock is enabled. */
	if (clk->count)
		cc_reg_val |= new_bank_masks->mnd_en_mask;
	else
		cc_reg_val &= ~(new_bank_masks->mnd_en_mask);

	cc_reg_val &= ~(new_bank_masks->mode_mask);
	cc_reg_val |= (nf->cc_val & new_bank_masks->mode_mask);
	writel(cc_reg_val, clk->cc_reg);

	/* Deassert bank MND reset. */
	ns_reg_val &= ~(new_bank_masks->rst_mask);
	writel(ns_reg_val, clk->ns_reg);

	/* Switch to the new bank if clock is running.  If it isn't, then
	 * no switch is necessary since we programmed the active bank. */
	if (clk->count && clk->current_freq->freq_hz) {
		cc_reg_val ^= banks->bank_sel_mask;
		writel(cc_reg_val, clk->cc_reg);
		/* Wait at least 6 cycles of slowest bank's clock
		 * for the glitch-free MUX to fully switch sources. */
		udelay(1);

		/* Disable old bank's MN counter. */
		cc_reg_val &= ~(old_bank_masks->mnd_en_mask);
		writel(cc_reg_val, clk->cc_reg);

		/* Program old bank to a low-power source and divider. */
		ns_reg_val &= ~(old_bank_masks->ns_mask);
		ns_reg_val |= (clk->freq_tbl->ns_val & old_bank_masks->ns_mask);
		writel(ns_reg_val, clk->ns_reg);
	}

	/* If this freq requires the MN counter to be enabled,
	 * update the enable mask to match the current bank. */
	if (nf->mnd_en_mask)
		nf->mnd_en_mask = new_bank_masks->mnd_en_mask;
	/* Update the NS mask to match the current bank. */
	clk->ns_mask = new_bank_masks->ns_mask;
}

static void set_rate_div_banked(struct clk_local *clk, struct clk_freq_tbl *nf)
{
	struct bank_masks *banks = clk->bank_masks;
	const struct bank_mask_info *new_bank_masks;
	const struct bank_mask_info *old_bank_masks;
	uint32_t ns_reg_val, bank_sel;

	/* Determine active bank and program the other one. If the clock is
	 * off, program the active bank since bank switching won't work if
	 * both banks aren't running. */
	ns_reg_val = readl(clk->ns_reg);
	bank_sel = !!(ns_reg_val & banks->bank_sel_mask);
	 /* If clock isn't running, don't switch banks. */
	bank_sel ^= (clk->count == 0 || clk->current_freq->freq_hz == 0);
	if (bank_sel == 0) {
		new_bank_masks = &banks->bank1_mask;
		old_bank_masks = &banks->bank0_mask;
	} else {
		new_bank_masks = &banks->bank0_mask;
		old_bank_masks = &banks->bank1_mask;
	}

	/* Program NS only if the clock is enabled, since the NS will be set
	 * as part of the enable procedure and should remain with a low-power
	 * MUX input selected until then. */
	if (clk->count) {
		ns_reg_val &= ~(new_bank_masks->ns_mask);
		ns_reg_val |= (nf->ns_val & new_bank_masks->ns_mask);
		writel(ns_reg_val, clk->ns_reg);
	}

	/* Switch to the new bank if clock is running.  If it isn't, then
	 * no switch is necessary since we programmed the active bank. */
	if (clk->count && clk->current_freq->freq_hz) {
		ns_reg_val ^= banks->bank_sel_mask;
		writel(ns_reg_val, clk->ns_reg);
		/* Wait at least 6 cycles of slowest bank's clock
		 * for the glitch-free MUX to fully switch sources. */
		udelay(1);

		/* Program old bank to a low-power source and divider. */
		ns_reg_val &= ~(old_bank_masks->ns_mask);
		ns_reg_val |= (clk->freq_tbl->ns_val & old_bank_masks->ns_mask);
		writel(ns_reg_val, clk->ns_reg);
	}

	/* Update the NS mask to match the current bank. */
	clk->ns_mask = new_bank_masks->ns_mask;
}

/*
 * Generic clock declaration macros
 */
#define CLK_NORATE(id, reg, br, r_r, r_m, h_r, h_c, h_b, tv) \
	[L_##id##_CLK] = { \
		.type = NORATE, \
		.cc_reg = reg, \
		.reset_reg = r_r, \
		.reset_mask = r_m, \
		.halt_reg = h_r, \
		.halt_check = h_c, \
		.halt_bit = h_b, \
		.test_vector = tv, \
		.br_en_mask = br, \
		.parent = L_NONE_CLK, \
		.current_freq = &local_dummy_freq, \
	}

#define CLK_SLAVE(id, reg, br, r_r, r_m, h_r, h_c, h_b, par, tv) \
	[L_##id##_CLK] = { \
		.type = NORATE, \
		.cc_reg = reg, \
		.reset_reg = r_r, \
		.reset_mask = r_m, \
		.halt_reg = h_r, \
		.halt_check = h_c, \
		.halt_bit = h_b, \
		.br_en_mask = br, \
		.parent = L_##par##_CLK, \
		.test_vector = tv, \
		.current_freq = &local_dummy_freq, \
	}

#define CLK_RESET(id, ns, r_m) \
	[L_##id##_CLK] = { \
		.type = RESET, \
		.reset_reg = ns, \
		.reset_mask = r_m, \
		.parent = L_NONE_CLK, \
		.current_freq = &local_dummy_freq, \
	}

/*
 * Clock frequency definitions and macros
 */
#define MN_MODE_DUAL_EDGE 0x2

/* MD Registers */
#define MD4(m_lsb, m, n_lsb, n) \
		(BVAL((m_lsb+3), m_lsb, m) | BVAL((n_lsb+3), n_lsb, ~(n)))
#define MD8(m_lsb, m, n_lsb, n) \
		(BVAL((m_lsb+7), m_lsb, m) | BVAL((n_lsb+7), n_lsb, ~(n)))
#define MD16(m, n) (BVAL(31, 16, m) | BVAL(15, 0, ~(n)))

/* NS Registers */
#define NS(n_msb, n_lsb, n, m, mde_lsb, d_msb, d_lsb, d, s_msb, s_lsb, s) \
		(BVAL(n_msb, n_lsb, ~(n-m)) \
		| (BVAL((mde_lsb+1), mde_lsb, MN_MODE_DUAL_EDGE) * !!(n)) \
		| BVAL(d_msb, d_lsb, (d-1)) | BVAL(s_msb, s_lsb, SRC_SEL_##s))

#define NS_MM(n_msb, n_lsb, n, m, d_msb, d_lsb, d, s_msb, s_lsb, s) \
		(BVAL(n_msb, n_lsb, ~(n-m)) | BVAL(d_msb, d_lsb, (d-1)) \
		| BVAL(s_msb, s_lsb, SRC_SEL_##s))

#define NS_DIVSRC(d_msb , d_lsb, d, s_msb, s_lsb, s) \
		(BVAL(d_msb, d_lsb, (d-1)) | BVAL(s_msb, s_lsb, SRC_SEL_##s))

#define NS_DIV(d_msb , d_lsb, d) \
		BVAL(d_msb, d_lsb, (d-1))

#define NS_SRC_SEL(s_msb, s_lsb, s) \
		BVAL(s_msb, s_lsb, SRC_SEL_##s)

#define NS_MND_BANKED4(n0_lsb, n1_lsb, n, m, s0_lsb, s1_lsb, s) \
		 (BVAL((n0_lsb+3), n0_lsb, ~(n-m)) \
		| BVAL((n1_lsb+3), n1_lsb, ~(n-m)) \
		| BVAL((s0_lsb+2), s0_lsb, SRC_SEL_##s) \
		| BVAL((s1_lsb+2), s1_lsb, SRC_SEL_##s))

#define NS_MND_BANKED8(n0_lsb, n1_lsb, n, m, s0_lsb, s1_lsb, s) \
		 (BVAL((n0_lsb+7), n0_lsb, ~(n-m)) \
		| BVAL((n1_lsb+7), n1_lsb, ~(n-m)) \
		| BVAL((s0_lsb+2), s0_lsb, SRC_SEL_##s) \
		| BVAL((s1_lsb+2), s1_lsb, SRC_SEL_##s))

#define NS_DIVSRC_BANKED(d0_msb, d0_lsb, d1_msb, d1_lsb, d, \
	s0_msb, s0_lsb, s1_msb, s1_lsb, s) \
		 (BVAL(d0_msb, d0_lsb, (d-1)) | BVAL(d1_msb, d1_lsb, (d-1)) \
		| BVAL(s0_msb, s0_lsb, SRC_SEL_##s) \
		| BVAL(s1_msb, s1_lsb, SRC_SEL_##s))

/* CC Registers */
#define CC(mde_lsb, n) (BVAL((mde_lsb+1), mde_lsb, MN_MODE_DUAL_EDGE) * !!(n))
#define CC_BANKED(mde0_lsb, mde1_lsb, n) \
		((BVAL((mde0_lsb+1), mde0_lsb, MN_MODE_DUAL_EDGE) \
		| BVAL((mde1_lsb+1), mde1_lsb, MN_MODE_DUAL_EDGE)) \
		* !!(n))

/*
 * Clock Descriptions
 */

/* ADM */
#define CLK_ADM(id, br, h_r, h_c, h_b, tv) \
	[L_##id##_CLK] = { \
		.type = BASIC, \
		.ns_reg = SC0_U_CLK_BRANCH_ENA_VOTE_REG, \
		.cc_reg = SC0_U_CLK_BRANCH_ENA_VOTE_REG, \
		.halt_reg = h_r, \
		.halt_check = h_c, \
		.halt_bit = h_b, \
		.br_en_mask = br, \
		.set_rate = set_rate_nop, \
		.freq_tbl = clk_tbl_adm, \
		.parent = L_NONE_CLK, \
		.test_vector = tv, \
		.current_freq = &local_dummy_freq, \
	}
#define F_ADM(f, s, v) \
	{ \
		.freq_hz = f, \
		.src = SRC_##s, \
		.sys_vdd = v, \
	}
static struct clk_freq_tbl clk_tbl_adm[] = {
	F_ADM(1, BB_PXO, NONE),
	F_END,
};

/* GSBI_UART */
#define CLK_GSBI_UART(id, n, h_r, h_c, h_b, tv) \
	[L_##id##_CLK] = { \
		.type = MND, \
		.ns_reg = GSBIn_UART_APPS_NS_REG(n), \
		.cc_reg = GSBIn_UART_APPS_NS_REG(n), \
		.md_reg = GSBIn_UART_APPS_MD_REG(n), \
		.reset_reg = GSBIn_RESET_REG(n), \
		.reset_mask = BIT(0), \
		.halt_reg = h_r, \
		.halt_check = h_c, \
		.halt_bit = h_b, \
		.br_en_mask = BIT(9), \
		.root_en_mask = BIT(11), \
		.ns_mask = (BM(31, 16) | BM(6, 0)), \
		.set_rate = set_rate_mnd, \
		.freq_tbl = clk_tbl_gsbi_uart, \
		.parent = L_NONE_CLK, \
		.test_vector = tv, \
		.current_freq = &local_dummy_freq, \
	}
#define F_GSBI_UART(f, s, d, m, n, v) \
	{ \
		.freq_hz = f, \
		.src = SRC_##s, \
		.md_val = MD16(m, n), \
		.ns_val = NS(31, 16, n, m, 5, 4, 3, d, 2, 0, s), \
		.mnd_en_mask = BIT(8) * !!(n), \
		.sys_vdd = v, \
	}
static struct clk_freq_tbl clk_tbl_gsbi_uart[] = {
	F_GSBI_UART(       0, BB_GND,  1,  0,   0, NONE),
	F_GSBI_UART( 3686400, BB_PLL8, 1,  6, 625, LOW),
	F_GSBI_UART( 7372800, BB_PLL8, 1, 12, 625, LOW),
	F_GSBI_UART(14745600, BB_PLL8, 1, 24, 625, LOW),
	F_GSBI_UART(16000000, BB_PLL8, 4,  1,   6, LOW),
	F_GSBI_UART(24000000, BB_PLL8, 4,  1,   4, LOW),
	F_GSBI_UART(32000000, BB_PLL8, 4,  1,   3, LOW),
	F_GSBI_UART(40000000, BB_PLL8, 1,  5,  48, NOMINAL),
	F_GSBI_UART(46400000, BB_PLL8, 1, 29, 240, NOMINAL),
	F_GSBI_UART(48000000, BB_PLL8, 4,  1,   2, NOMINAL),
	F_GSBI_UART(51200000, BB_PLL8, 1,  2,  15, NOMINAL),
	F_GSBI_UART(48000000, BB_PLL8, 4,  1,   2, NOMINAL),
	F_GSBI_UART(51200000, BB_PLL8, 1,  2,  15, NOMINAL),
	F_GSBI_UART(56000000, BB_PLL8, 1,  7,  48, NOMINAL),
	F_GSBI_UART(58982400, BB_PLL8, 1, 96, 625, NOMINAL),
	F_GSBI_UART(64000000, BB_PLL8, 2,  1,   3, NOMINAL),
	F_END,
};

/* GSBI_QUP */
#define CLK_GSBI_QUP(id, n, h_r, h_c, h_b, tv) \
	[L_##id##_CLK] = { \
		.type = MND, \
		.ns_reg = GSBIn_QUP_APPS_NS_REG(n), \
		.cc_reg = GSBIn_QUP_APPS_NS_REG(n), \
		.md_reg = GSBIn_QUP_APPS_MD_REG(n), \
		.reset_reg = GSBIn_RESET_REG(n), \
		.reset_mask = BIT(0), \
		.halt_reg = h_r, \
		.halt_check = h_c, \
		.halt_bit = h_b, \
		.br_en_mask = BIT(9), \
		.root_en_mask = BIT(11), \
		.ns_mask = (BM(23, 16) | BM(6, 0)), \
		.set_rate = set_rate_mnd, \
		.freq_tbl = clk_tbl_gsbi_qup, \
		.parent = L_NONE_CLK, \
		.test_vector = tv, \
		.current_freq = &local_dummy_freq, \
	}
#define F_GSBI_QUP(f, s, d, m, n, v) \
	{ \
		.freq_hz = f, \
		.src = SRC_##s, \
		.md_val = MD8(16, m, 0, n), \
		.ns_val = NS(23, 16, n, m, 5, 4, 3, d, 2, 0, s), \
		.mnd_en_mask = BIT(8) * !!(n), \
		.sys_vdd = v, \
	}
static struct clk_freq_tbl clk_tbl_gsbi_qup[] = {
	F_GSBI_QUP(       0, BB_GND,  1, 0,  0, NONE),
	F_GSBI_QUP( 1100000, BB_PXO,  1, 2, 49, LOW),
	F_GSBI_QUP( 5400000, BB_PXO,  1, 1,  5, LOW),
	F_GSBI_QUP(10800000, BB_PXO,  1, 2,  5, LOW),
	F_GSBI_QUP(15060000, BB_PLL8, 1, 2, 51, LOW),
	F_GSBI_QUP(24000000, BB_PLL8, 4, 1,  4, LOW),
	F_GSBI_QUP(25600000, BB_PLL8, 1, 1, 15, NOMINAL),
	F_GSBI_QUP(27000000, BB_PXO,  1, 0,  0, NOMINAL),
	F_GSBI_QUP(48000000, BB_PLL8, 4, 1,  2, NOMINAL),
	F_GSBI_QUP(51200000, BB_PLL8, 1, 2, 15, NOMINAL),
	F_END,
};

/* PDM */
#define CLK_PDM(id) \
	[L_##id##_CLK] = { \
		.type = BASIC, \
		.ns_reg = PDM_CLK_NS_REG, \
		.cc_reg = PDM_CLK_NS_REG, \
		.reset_reg = PDM_CLK_NS_REG, \
		.reset_mask = BIT(12), \
		.halt_reg = CLK_HALT_CFPB_STATEC_REG, \
		.halt_check = HALT, \
		.halt_bit = 3, \
		.br_en_mask = BIT(9), \
		.root_en_mask = BIT(11), \
		.ns_mask = BM(1, 0), \
		.set_rate = set_rate_nop, \
		.freq_tbl = clk_tbl_pdm, \
		.parent = L_NONE_CLK, \
		.current_freq = &local_dummy_freq, \
	}
#define F_PDM(f, s, d, v) \
	{ \
		.freq_hz = f, \
		.src = SRC_##s, \
		.ns_val = NS_SRC_SEL(1, 0, s), \
		.sys_vdd = v, \
	}
static struct clk_freq_tbl clk_tbl_pdm[] = {
	F_PDM(       0, XO_GND, 1, NONE),
	F_PDM(27000000, XO_PXO, 1, LOW),
	F_END,
};

/* PRNG */
#define CLK_PRNG(id) \
	[L_##id##_CLK] = { \
		.type = BASIC, \
		.ns_reg = PRNG_CLK_NS_REG, \
		.cc_reg = SC0_U_CLK_BRANCH_ENA_VOTE_REG, \
		.reset_reg = PRNG_CLK_NS_REG, \
		.reset_mask = BIT(12), \
		.halt_reg = CLK_HALT_SFPB_MISC_STATE_REG, \
		.halt_check = HALT_VOTED, \
		.halt_bit = 10, \
		.br_en_mask = BIT(10), \
		.ns_mask = (BM(6, 3) | BM(2, 0)), \
		.set_rate = set_rate_nop, \
		.freq_tbl = clk_tbl_prng, \
		.parent = L_NONE_CLK, \
		.test_vector = TEST_PER_LS(0x7D), \
		.current_freq = &local_dummy_freq, \
	}
#define F_PRNG(f, s, d, v) \
	{ \
		.freq_hz = f, \
		.src = SRC_##s, \
		.ns_val = NS_DIVSRC(6, 3, d, 2, 0, s), \
		.sys_vdd = v, \
	}
static struct clk_freq_tbl clk_tbl_prng[] = {
	F_PRNG(64000000, BB_PLL8,  6, NOMINAL),
	F_END,
};

/* SDC */
#define CLK_SDC(id, n, h_r, h_c, h_b, tv) \
	[L_##id##_CLK] = { \
		.type = MND, \
		.ns_reg = SDCn_APPS_CLK_NS_REG(n), \
		.cc_reg = SDCn_APPS_CLK_NS_REG(n), \
		.md_reg = SDCn_APPS_CLK_MD_REG(n), \
		.reset_reg = SDCn_RESET_REG(n), \
		.reset_mask = BIT(0), \
		.halt_reg = h_r, \
		.halt_check = h_c, \
		.halt_bit = h_b, \
		.br_en_mask = BIT(9), \
		.root_en_mask = BIT(11), \
		.ns_mask = (BM(23, 16) | BM(6, 0)), \
		.set_rate = set_rate_mnd, \
		.freq_tbl = clk_tbl_sdc, \
		.parent = L_NONE_CLK, \
		.test_vector = tv, \
		.current_freq = &local_dummy_freq, \
	}
#define F_SDC(f, s, d, m, n, v) \
	{ \
		.freq_hz = f, \
		.src = SRC_##s, \
		.md_val = MD8(16, m, 0, n), \
		.ns_val = NS(23, 16, n, m, 5, 4, 3, d, 2, 0, s), \
		.mnd_en_mask = BIT(8) * !!(n), \
		.sys_vdd = v, \
	}
static struct clk_freq_tbl clk_tbl_sdc[] = {
	F_SDC(       0,  BB_GND,  1, 0,   0, NONE),
	F_SDC(  144000,  BB_PXO,  3, 2, 125, LOW),
	F_SDC(  400000, BB_PLL8,  4, 1, 240, LOW),
	F_SDC(16000000, BB_PLL8,  4, 1,   6, LOW),
	F_SDC(17070000, BB_PLL8,  1, 2,  45, LOW),
	F_SDC(20210000, BB_PLL8,  1, 1,  19, LOW),
	F_SDC(24000000, BB_PLL8,  4, 1,   4, LOW),
	F_SDC(48000000, BB_PLL8,  4, 1,   2, NOMINAL),
	F_END,
};

/* TSIF_REF */
#define CLK_TSIF_REF(id) \
	[L_##id##_CLK] = { \
		.type = MND, \
		.ns_reg = TSIF_REF_CLK_NS_REG, \
		.cc_reg = TSIF_REF_CLK_NS_REG, \
		.md_reg = TSIF_REF_CLK_MD_REG, \
		.halt_reg = CLK_HALT_CFPB_STATEC_REG, \
		.halt_check = HALT, \
		.halt_bit = 5, \
		.br_en_mask = BIT(9), \
		.root_en_mask = BIT(11), \
		.ns_mask = (BM(31, 16) | BM(6, 0)), \
		.set_rate = set_rate_mnd, \
		.freq_tbl = clk_tbl_tsif_ref, \
		.parent = L_NONE_CLK, \
		.test_vector = TEST_PER_LS(0x91), \
		.current_freq = &local_dummy_freq, \
	}
#define F_TSIF_REF(f, s, d, m, n, v) \
	{ \
		.freq_hz = f, \
		.src = SRC_##s, \
		.md_val = MD16(m, n), \
		.ns_val = NS(31, 16, n, m, 5, 4, 3, d, 2, 0, s), \
		.mnd_en_mask = BIT(8) * !!(n), \
		.sys_vdd = v, \
	}
static struct clk_freq_tbl clk_tbl_tsif_ref[] = {
	F_TSIF_REF(     0, BB_GND, 1, 0,   0, NONE),
	F_TSIF_REF(105000, BB_PXO, 1, 1, 256, LOW),
	F_END,
};


/* TSSC */
#define CLK_TSSC(id) \
	[L_##id##_CLK] = { \
		.type = BASIC, \
		.ns_reg = TSSC_CLK_CTL_REG, \
		.cc_reg = TSSC_CLK_CTL_REG, \
		.halt_reg = CLK_HALT_CFPB_STATEC_REG, \
		.halt_check = HALT, \
		.halt_bit = 4, \
		.br_en_mask = BIT(4), \
		.ns_mask = BM(1, 0), \
		.set_rate = set_rate_nop, \
		.freq_tbl = clk_tbl_tssc, \
		.parent = L_NONE_CLK, \
		.test_vector = TEST_PER_LS(0x94), \
		.current_freq = &local_dummy_freq, \
	}
#define F_TSSC(f, s, v) \
	{ \
		.freq_hz = f, \
		.src = SRC_##s, \
		.ns_val = NS_SRC_SEL(1, 0, s), \
		.sys_vdd = v, \
	}
static struct clk_freq_tbl clk_tbl_tssc[] = {
	F_TSSC(       0, XO_GND, NONE),
	F_TSSC(27000000, XO_PXO, LOW),
	F_END,
};

/* USB_HS and USB_FS */
#define CLK_USB_HS(id) \
	[L_##id##_CLK] = { \
		.type = MND, \
		.ns_reg = USB_HS1_XCVR_FS_CLK_NS_REG, \
		.cc_reg = USB_HS1_XCVR_FS_CLK_NS_REG, \
		.md_reg = USB_HS1_XCVR_FS_CLK_MD_REG, \
		.reset_reg = USB_HS1_RESET_REG, \
		.reset_mask = BIT(0), \
		.halt_reg = CLK_HALT_DFAB_STATE_REG, \
		.halt_check = HALT, \
		.halt_bit = 0, \
		.br_en_mask = BIT(9), \
		.root_en_mask = BIT(11), \
		.ns_mask = (BM(23, 16) | BM(6, 0)), \
		.set_rate = set_rate_mnd, \
		.freq_tbl = clk_tbl_usb, \
		.parent = L_NONE_CLK, \
		.test_vector = TEST_PER_LS(0x85), \
		.current_freq = &local_dummy_freq, \
	}
#define CLK_USB_FS(id, n, chld_lst) \
	[L_##id##_CLK] = { \
		.type = MND, \
		.ns_reg = USB_FSn_XCVR_FS_CLK_NS_REG(n), \
		.cc_reg = USB_FSn_XCVR_FS_CLK_NS_REG(n), \
		.md_reg = USB_FSn_XCVR_FS_CLK_MD_REG(n), \
		.root_en_mask = BIT(11), \
		.ns_mask = (BM(23, 16) | BM(6, 0)), \
		.set_rate = set_rate_mnd, \
		.freq_tbl = clk_tbl_usb, \
		.parent = L_NONE_CLK, \
		.children = chld_lst, \
		.current_freq = &local_dummy_freq, \
	}
#define F_USB(f, s, d, m, n, v) \
	{ \
		.freq_hz = f, \
		.src = SRC_##s, \
		.md_val = MD8(16, m, 0, n), \
		.ns_val = NS(23, 16, n, m, 5, 4, 3, d, 2, 0, s), \
		.mnd_en_mask = BIT(8) * !!(n), \
		.sys_vdd = v, \
	}
static struct clk_freq_tbl clk_tbl_usb[] = {
	F_USB(       0, BB_GND,  1, 0,  0, NONE),
	F_USB(60000000, BB_PLL8, 1, 5, 32, NOMINAL),
	F_END,
};

/* CAM */
#define CLK_CAM(id) \
	[L_##id##_CLK] = { \
		.type = MND, \
		.ns_reg = CAMCLK_NS_REG, \
		.cc_reg = CAMCLK_CC_REG, \
		.md_reg = CAMCLK_MD_REG, \
		.halt_reg = NULL, \
		.halt_check = DELAY, \
		.br_en_mask = BIT(0), \
		.root_en_mask = BIT(2), \
		.ns_mask = (BM(31, 24) | BM(15, 14) | BM(2, 0)), \
		.cc_mask = BM(7, 6), \
		.set_rate = set_rate_cam, \
		.freq_tbl = clk_tbl_cam, \
		.parent = L_NONE_CLK, \
		.test_vector = TEST_MM_LS(0x1D), \
		.current_freq = &local_dummy_freq, \
	}
#define F_CAM(f, s, d, m, n, v) \
	{ \
		.freq_hz = f, \
		.src = SRC_##s, \
		.md_val = MD8(8, m, 0, n), \
		.ns_val = NS_MM(31, 24, n, m, 15, 14, d, 2, 0, s), \
		.cc_val = CC(6, n), \
		.mnd_en_mask = BIT(5) * !!(n), \
		.sys_vdd = v \
	}
static struct clk_freq_tbl clk_tbl_cam[] = {
	F_CAM(        0, MM_GND,   1, 0,  0, NONE),
	F_CAM(  6000000, MM_GPERF, 4, 1, 16, LOW),
	F_CAM(  8000000, MM_GPERF, 4, 1, 12, LOW),
	F_CAM( 12000000, MM_GPERF, 4, 1,  8, LOW),
	F_CAM( 16000000, MM_GPERF, 4, 1,  6, LOW),
	F_CAM( 19200000, MM_GPERF, 4, 1,  5, LOW),
	F_CAM( 24000000, MM_GPERF, 4, 1,  4, LOW),
	F_CAM( 32000000, MM_GPERF, 4, 1,  3, LOW),
	F_CAM( 48000000, MM_GPERF, 4, 1,  2, LOW),
	F_CAM( 64000000, MM_GPERF, 3, 1,  2, LOW),
	F_CAM( 96000000, MM_GPERF, 4, 0,  0, NOMINAL),
	F_CAM(128000000, MM_GPERF, 3, 0,  0, NOMINAL),
	F_END,
};

/* CSI */
#define CLK_CSI(id) \
	[L_##id##_CLK] = { \
		.type = BASIC, \
		.ns_reg = CSI_NS_REG, \
		.cc_reg = CSI_CC_REG, \
		.root_en_mask = BIT(2), \
		.ns_mask = (BM(15, 12) | BM(2, 0)), \
		.set_rate = set_rate_nop, \
		.freq_tbl = clk_tbl_csi, \
		.parent = L_NONE_CLK, \
		.children = chld_csi_src, \
		.current_freq = &local_dummy_freq, \
	}
#define F_CSI(f, s, d, v) \
	{ \
		.freq_hz = f, \
		.src = SRC_##s, \
		.ns_val = NS_DIVSRC(15, 12, d, 2, 0, s),  \
		.sys_vdd = v \
	}
static struct clk_freq_tbl clk_tbl_csi[] = {
	F_CSI(        0, MM_GND,   1, NONE),
	F_CSI(192000000, MM_GPERF, 2, LOW),
	F_CSI(384000000, MM_GPERF, 1, NOMINAL),
	F_END,
};

/* DSI_BYTE */
#define CLK_DSI_BYTE(id) \
	[L_##id##_CLK] = { \
		.type = BASIC, \
		.ns_reg = MISC_CC2_REG, \
		.cc_reg = MISC_CC_REG, \
		.reset_reg = SW_RESET_CORE_REG, \
		.reset_mask = BIT(7), \
		.halt_reg = DBG_BUS_VEC_B_REG, \
		.halt_check = DELAY, \
		.root_en_mask = BIT(2), \
		.ns_mask = BM(27, 24), \
		.set_rate = set_rate_nop, \
		.freq_tbl = clk_tbl_dsi_byte, \
		.parent = L_NONE_CLK, \
		.test_vector = TEST_MM_LS(0x00), \
		.current_freq = &local_dummy_freq, \
	}
#define F_DSI(d) \
	{ \
		.freq_hz = d, \
		.src = SRC_NONE, \
		.ns_val = BVAL(27, 24, (d-1)), \
	}
/* The DSI_BYTE clock is sourced from the DSI PHY PLL, which may change rate
 * without this clock driver knowing.  So, overload the clk_set_rate() to set
 * the divider (1 to 16) of the clock with respect to the PLL rate. */
static struct clk_freq_tbl clk_tbl_dsi_byte[] = {
	F_DSI(1),  F_DSI(2),  F_DSI(3),  F_DSI(4),
	F_DSI(5),  F_DSI(6),  F_DSI(7),  F_DSI(8),
	F_DSI(9),  F_DSI(10), F_DSI(11), F_DSI(12),
	F_DSI(13), F_DSI(14), F_DSI(15), F_DSI(16),
	F_END,
};

/* GFX2D0 and GFX2D1 */
static struct bank_masks bmnd_info_gfx2d0 = {
	.bank_sel_mask =			BIT(11),
	.bank0_mask = {
			.md_reg =		GFX2D0_MD0_REG,
			.ns_mask =		BM(23, 20) | BM(5, 3),
			.rst_mask =		BIT(25),
			.mnd_en_mask =		BIT(8),
			.mode_mask =		BM(10, 9),
	},
	.bank1_mask = {
			.md_reg =		GFX2D0_MD1_REG,
			.ns_mask =		BM(19, 16) | BM(2, 0),
			.rst_mask =		BIT(24),
			.mnd_en_mask =		BIT(5),
			.mode_mask =		BM(7, 6),
	},
};
#define CLK_GFX2D0(id) \
	[L_##id##_CLK] = { \
		.type = MND, \
		.ns_reg = GFX2D0_NS_REG, \
		.cc_reg = GFX2D0_CC_REG, \
		.reset_reg = SW_RESET_CORE_REG, \
		.reset_mask = BIT(14), \
		.halt_reg = DBG_BUS_VEC_A_REG, \
		.halt_check = HALT, \
		.halt_bit = 9, \
		.br_en_mask = BIT(0), \
		.root_en_mask = BIT(2), \
		.set_rate = set_rate_mnd_banked, \
		.freq_tbl = clk_tbl_gfx2d, \
		.bank_masks = &bmnd_info_gfx2d0, \
		.parent = L_NONE_CLK, \
		.test_vector = TEST_MM_HS(0x07), \
		.current_freq = &local_dummy_freq, \
	}
static struct bank_masks bmnd_info_gfx2d1 = {
	.bank_sel_mask =		BIT(11),
	.bank0_mask = {
			.md_reg =		GFX2D1_MD0_REG,
			.ns_mask =		BM(23, 20) | BM(5, 3),
			.rst_mask =		BIT(25),
			.mnd_en_mask =		BIT(8),
			.mode_mask =		BM(10, 9),
	},
	.bank1_mask = {
			.md_reg =		GFX2D1_MD1_REG,
			.ns_mask =		BM(19, 16) | BM(2, 0),
			.rst_mask =		BIT(24),
			.mnd_en_mask =		BIT(5),
			.mode_mask =		BM(7, 6),
	},
};
#define CLK_GFX2D1(id) \
	[L_##id##_CLK] = { \
		.type = MND, \
		.ns_reg = GFX2D1_NS_REG, \
		.cc_reg = GFX2D1_CC_REG, \
		.reset_reg = SW_RESET_CORE_REG, \
		.reset_mask = BIT(13), \
		.halt_reg = DBG_BUS_VEC_A_REG, \
		.halt_check = HALT, \
		.halt_bit = 14, \
		.br_en_mask = BIT(0), \
		.root_en_mask = BIT(2), \
		.set_rate = set_rate_mnd_banked, \
		.freq_tbl = clk_tbl_gfx2d, \
		.bank_masks = &bmnd_info_gfx2d1, \
		.parent = L_NONE_CLK, \
		.test_vector = TEST_MM_HS(0x08), \
		.current_freq = &local_dummy_freq, \
	}
#define F_GFX2D(f, s, m, n, v) \
	{ \
		.freq_hz = f, \
		.src = SRC_##s, \
		.md_val = MD4(4, m, 0, n), \
		.ns_val = NS_MND_BANKED4(20, 16, n, m, 3, 0, s), \
		.cc_val = CC_BANKED(9, 6, n), \
		.mnd_en_mask = (BIT(8) | BIT(5)) * !!(n), \
		.sys_vdd = v, \
	}
static struct clk_freq_tbl clk_tbl_gfx2d[] = {
	F_GFX2D(        0, MM_GND,   0,  0, NONE),
	F_GFX2D( 27000000, MM_PXO,   0,  0, LOW),
	F_GFX2D( 48000000, MM_GPERF, 1,  8, LOW),
	F_GFX2D( 54857000, MM_GPERF, 1,  7, LOW),
	F_GFX2D( 64000000, MM_GPERF, 1,  6, LOW),
	F_GFX2D( 76800000, MM_GPERF, 1,  5, LOW),
	F_GFX2D( 96000000, MM_GPERF, 1,  4, LOW),
	F_GFX2D(128000000, MM_GPERF, 1,  3, NOMINAL),
	F_GFX2D(145455000, MM_PLL1,  2, 11, NOMINAL),
	F_GFX2D(160000000, MM_PLL1,  1,  5, NOMINAL),
	F_GFX2D(177778000, MM_PLL1,  2,  9, NOMINAL),
	F_GFX2D(200000000, MM_PLL1,  1,  4, NOMINAL),
	F_GFX2D(228571000, MM_PLL1,  2,  7, HIGH),
	F_END,
};

/* GFX3D */
static struct bank_masks bmnd_info_gfx3d = {
	.bank_sel_mask =		BIT(11),
	.bank0_mask = {
			.md_reg =		GFX3D_MD0_REG,
			.ns_mask =		BM(21, 18) | BM(5, 3),
			.rst_mask =		BIT(23),
			.mnd_en_mask =		BIT(8),
			.mode_mask =		BM(10, 9),
	},
	.bank1_mask = {
			.md_reg =		GFX3D_MD1_REG,
			.ns_mask =		BM(17, 14) | BM(2, 0),
			.rst_mask =		BIT(22),
			.mnd_en_mask =		BIT(5),
			.mode_mask =		BM(7, 6),
	},
};
#define CLK_GFX3D(id, par) \
	[L_##id##_CLK] = { \
		.type = MND, \
		.ns_reg = GFX3D_NS_REG, \
		.cc_reg = GFX3D_CC_REG, \
		.reset_reg = SW_RESET_CORE_REG, \
		.reset_mask = BIT(12), \
		.halt_reg = DBG_BUS_VEC_A_REG, \
		.halt_check = HALT, \
		.halt_bit = 4, \
		.br_en_mask = BIT(0), \
		.root_en_mask = BIT(2), \
		.set_rate = set_rate_mnd_banked, \
		.freq_tbl = clk_tbl_gfx3d, \
		.bank_masks = &bmnd_info_gfx3d, \
		.parent = L_##par##_CLK, \
		.test_vector = TEST_MM_HS(0x09), \
		.current_freq = &local_dummy_freq, \
	}
#define F_GFX3D(f, s, m, n, v) \
	{ \
		.freq_hz = f, \
		.src = SRC_##s, \
		.md_val = MD4(4, m, 0, n), \
		.ns_val = NS_MND_BANKED4(18, 14, n, m, 3, 0, s), \
		.cc_val = CC_BANKED(9, 6, n), \
		.mnd_en_mask = (BIT(8) | BIT(5)) * !!(n), \
		.sys_vdd = v, \
	}
static struct clk_freq_tbl clk_tbl_gfx3d[] = {
	F_GFX3D(        0, MM_GND,   0,  0, NONE),
	F_GFX3D( 27000000, MM_PXO,   0,  0, LOW),
	F_GFX3D( 48000000, MM_GPERF, 1,  8, LOW),
	F_GFX3D( 54857000, MM_GPERF, 1,  7, LOW),
	F_GFX3D( 64000000, MM_GPERF, 1,  6, LOW),
	F_GFX3D( 76800000, MM_GPERF, 1,  5, LOW),
	F_GFX3D( 96000000, MM_GPERF, 1,  4, LOW),
	F_GFX3D(128000000, MM_GPERF, 1,  3, NOMINAL),
	F_GFX3D(145455000, MM_PLL1,  2, 11, NOMINAL),
	F_GFX3D(160000000, MM_PLL1,  1,  5, NOMINAL),
	F_GFX3D(177778000, MM_PLL1,  2,  9, NOMINAL),
	F_GFX3D(200000000, MM_PLL1,  1,  4, NOMINAL),
	F_GFX3D(228571000, MM_PLL1,  2,  7, NOMINAL),
	F_GFX3D(266667000, MM_PLL1,  1,  3, HIGH),
	F_GFX3D(320000000, MM_PLL1,  2,  5, HIGH),
	F_END,
};

/* IJPEG */
#define CC_MASK_IJPEG (BM(7, 6))
#define CLK_IJPEG(id, par) \
	[L_##id##_CLK] = { \
		.type = MND, \
		.ns_reg = IJPEG_NS_REG, \
		.cc_reg = IJPEG_CC_REG, \
		.md_reg = IJPEG_MD_REG, \
		.reset_reg = SW_RESET_CORE_REG, \
		.reset_mask = BIT(9), \
		.halt_reg = DBG_BUS_VEC_A_REG, \
		.halt_check = HALT, \
		.halt_bit = 24, \
		.br_en_mask = BIT(0), \
		.root_en_mask = BIT(2), \
		.ns_mask = (BM(23, 16) | BM(15, 12) | BM(2, 0)), \
		.cc_mask = BM(7, 6), \
		.set_rate = set_rate_mnd, \
		.freq_tbl = clk_tbl_ijpeg, \
		.parent = L_##par##_CLK, \
		.test_vector = TEST_MM_HS(0x05), \
		.current_freq = &local_dummy_freq, \
	}
#define F_IJPEG(f, s, d, m, n, v) \
	{ \
		.freq_hz = f, \
		.src = SRC_##s, \
		.md_val = MD8(8, m, 0, n), \
		.ns_val = NS_MM(23, 16, n, m, 15, 12, d, 2, 0, s), \
		.cc_val = CC(6, n), \
		.mnd_en_mask = BIT(5) * !!n, \
		.sys_vdd = v, \
	}
static struct clk_freq_tbl clk_tbl_ijpeg[] = {
	F_IJPEG(        0, MM_GND,   1, 0,  0, NONE),
	F_IJPEG( 27000000, MM_PXO,   1, 0,  0, LOW),
	F_IJPEG( 36570000, MM_GPERF, 1, 2, 21, LOW),
	F_IJPEG( 54860000, MM_GPERF, 7, 0,  0, LOW),
	F_IJPEG( 96000000, MM_GPERF, 4, 0,  0, LOW),
	F_IJPEG(109710000, MM_GPERF, 1, 2,  7, LOW),
	F_IJPEG(128000000, MM_GPERF, 3, 0,  0, NOMINAL),
	F_IJPEG(153600000, MM_GPERF, 1, 2,  5, NOMINAL),
	F_IJPEG(200000000, MM_PLL1,  4, 0,  0, NOMINAL),
	F_IJPEG(228571000, MM_PLL1,  1, 2,  7, NOMINAL),
	F_END,
};

/* JPEGD */
#define CLK_JPEGD(id, par) \
	[L_##id##_CLK] = { \
		.type = BASIC, \
		.ns_reg = JPEGD_NS_REG, \
		.cc_reg = JPEGD_CC_REG, \
		.reset_reg = SW_RESET_CORE_REG, \
		.reset_mask = BIT(19), \
		.halt_reg = DBG_BUS_VEC_A_REG, \
		.halt_check = HALT, \
		.halt_bit = 19, \
		.br_en_mask = BIT(0), \
		.root_en_mask = BIT(2), \
		.ns_mask =  (BM(15, 12) | BM(2, 0)), \
		.set_rate = set_rate_nop, \
		.freq_tbl = clk_tbl_jpegd, \
		.parent = L_##par##_CLK, \
		.test_vector = TEST_MM_HS(0x0A), \
		.current_freq = &local_dummy_freq, \
	}
#define F_JPEGD(f, s, d, v) \
	{ \
		.freq_hz = f, \
		.src = SRC_##s, \
		.ns_val = NS_DIVSRC(15, 12, d, 2, 0, s), \
		.sys_vdd = v, \
	}
static struct clk_freq_tbl clk_tbl_jpegd[] = {
	F_JPEGD(        0, MM_GND,   1, NONE),
	F_JPEGD( 64000000, MM_GPERF, 6, LOW),
	F_JPEGD( 76800000, MM_GPERF, 5, LOW),
	F_JPEGD( 96000000, MM_GPERF, 4, LOW),
	F_JPEGD(160000000, MM_PLL1,  5, NOMINAL),
	F_JPEGD(200000000, MM_PLL1,  4, NOMINAL),
	F_END,
};

/* MDP */
static struct bank_masks bmnd_info_mdp = {
	.bank_sel_mask =		BIT(11),
	.bank0_mask = {
			.md_reg =		MDP_MD0_REG,
			.ns_mask =		BM(29, 22) | BM(5, 3),
			.rst_mask =		BIT(31),
			.mnd_en_mask =		BIT(8),
			.mode_mask =		BM(10, 9),
	},
	.bank1_mask = {
			.md_reg =		MDP_MD1_REG,
			.ns_mask =		BM(21, 14) | BM(2, 0),
			.rst_mask =		BIT(30),
			.mnd_en_mask =		BIT(5),
			.mode_mask =		BM(7, 6),
	},
};
#define CLK_MDP(id) \
	[L_##id##_CLK] = { \
		.type = MND, \
		.ns_reg = MDP_NS_REG, \
		.cc_reg = MDP_CC_REG, \
		.reset_reg = SW_RESET_CORE_REG, \
		.reset_mask = BIT(21), \
		.halt_reg = DBG_BUS_VEC_C_REG, \
		.halt_check = HALT, \
		.halt_bit = 10, \
		.br_en_mask = BIT(0), \
		.root_en_mask = BIT(2), \
		.set_rate = set_rate_mnd_banked, \
		.freq_tbl = clk_tbl_mdp, \
		.bank_masks = &bmnd_info_mdp, \
		.parent = L_NONE_CLK, \
		.test_vector = TEST_MM_HS(0x1A), \
		.current_freq = &local_dummy_freq, \
	}
#define F_MDP(f, s, m, n, v) \
	{ \
		.freq_hz = f, \
		.src = SRC_##s, \
		.md_val = MD8(8, m, 0, n), \
		.ns_val = NS_MND_BANKED8(22, 14, n, m, 3, 0, s), \
		.cc_val = CC_BANKED(9, 6, n), \
		.mnd_en_mask = (BIT(8) | BIT(5)) * !!(n), \
		.sys_vdd = v, \
	}
static struct clk_freq_tbl clk_tbl_mdp[] = {
	F_MDP(        0, MM_GND,   0,  0, NONE),
	F_MDP(  9600000, MM_GPERF, 1, 40, LOW),
	F_MDP( 13710000, MM_GPERF, 1, 28, LOW),
	F_MDP( 27000000, MM_PXO,   0,  0, LOW),
	F_MDP( 29540000, MM_GPERF, 1, 13, LOW),
	F_MDP( 34910000, MM_GPERF, 1, 11, LOW),
	F_MDP( 38400000, MM_GPERF, 1, 10, LOW),
	F_MDP( 59080000, MM_GPERF, 2, 13, LOW),
	F_MDP( 76800000, MM_GPERF, 1,  5, LOW),
	F_MDP( 85330000, MM_GPERF, 2,  9, LOW),
	F_MDP( 96000000, MM_GPERF, 1,  4, NOMINAL),
	F_MDP(128000000, MM_GPERF, 1,  3, NOMINAL),
	F_MDP(160000000, MM_PLL1,  1,  5, NOMINAL),
	F_MDP(177780000, MM_PLL1,  2,  9, NOMINAL),
	F_MDP(200000000, MM_PLL1,  1,  4, NOMINAL),
	F_END,
};

/* MDP VSYNC */
#define CLK_MDP_VSYNC(id) \
	[L_##id##_CLK] = { \
		.type = BASIC, \
		.ns_reg = MISC_CC2_REG, \
		.cc_reg = MISC_CC_REG, \
		.reset_reg = SW_RESET_CORE_REG, \
		.reset_mask = BIT(3), \
		.halt_reg = DBG_BUS_VEC_B_REG, \
		.halt_check = HALT, \
		.halt_bit = 22, \
		.br_en_mask = BIT(6), \
		.ns_mask = BIT(13), \
		.set_rate = set_rate_nop, \
		.freq_tbl = clk_tbl_mdp_vsync, \
		.parent = L_NONE_CLK, \
		.test_vector = TEST_MM_LS(0x20), \
		.current_freq = &local_dummy_freq, \
	}
#define F_MDP_VSYNC(f, s, v) \
	{ \
		.freq_hz = f, \
		.src = SRC_##s, \
		.ns_val = NS_SRC_SEL(13, 13, s), \
		.sys_vdd = v, \
	}
static struct clk_freq_tbl clk_tbl_mdp_vsync[] = {
	F_MDP_VSYNC(27000000, BB_PXO, LOW),
	F_END,
};

/* PIXEL_MDP */
#define CLK_PIXEL_MDP(id) \
	[L_##id##_CLK] = { \
		.type = MND, \
		.ns_reg = PIXEL_NS_REG, \
		.cc_reg = PIXEL_CC_REG, \
		.md_reg = PIXEL_MD_REG, \
		.reset_reg = SW_RESET_CORE_REG, \
		.reset_mask = BIT(5), \
		.halt_reg = DBG_BUS_VEC_C_REG, \
		.halt_check = HALT, \
		.halt_bit = 23, \
		.br_en_mask = BIT(0), \
		.root_en_mask = BIT(2), \
		.ns_mask = (BM(31, 16) | BM(15, 14) | BM(2, 0)), \
		.cc_mask = BM(7, 6), \
		.set_rate = set_rate_mnd, \
		.freq_tbl = clk_tbl_pixel_mdp, \
		.parent = L_NONE_CLK, \
		.children = chld_pixel_mdp, \
		.test_vector = TEST_MM_LS(0x04), \
		.current_freq = &local_dummy_freq, \
	}
#define F_PIXEL_MDP(f, s, d, m, n, v) \
	{ \
		.freq_hz = f, \
		.src = SRC_##s, \
		.md_val = MD16(m, n), \
		.ns_val = NS_MM(31, 16, n, m, 15, 14, d, 2, 0, s), \
		.cc_val = CC(6, n), \
		.mnd_en_mask = BIT(5) * !!(n), \
		.sys_vdd = v \
	}
static struct clk_freq_tbl clk_tbl_pixel_mdp[] = {
	F_PIXEL_MDP(        0, MM_GND,   1,   0,    0, NONE),
	F_PIXEL_MDP( 25600000, MM_GPERF, 3,   1,    5, LOW),
	F_PIXEL_MDP( 42667000, MM_GPERF, 1,   1,    9, LOW),
	F_PIXEL_MDP( 43192000, MM_GPERF, 1,  64,  569, LOW),
	F_PIXEL_MDP( 48000000, MM_GPERF, 4,   1,    2, LOW),
	F_PIXEL_MDP( 53990000, MM_GPERF, 2, 169,  601, LOW),
	F_PIXEL_MDP( 64000000, MM_GPERF, 2,   1,    3, LOW),
	F_PIXEL_MDP( 69300000, MM_GPERF, 1, 231, 1280, LOW),
	F_PIXEL_MDP( 76800000, MM_GPERF, 1,   1,    5, LOW),
	F_PIXEL_MDP( 85333000, MM_GPERF, 1,   2,    9, LOW),
	F_PIXEL_MDP( 96000000, MM_GPERF, 4,   0,    0, LOW),
	F_PIXEL_MDP(100030000, MM_GPERF, 2, 211,  405, LOW),
	F_PIXEL_MDP(106500000, MM_GPERF, 1,  71,  256, NOMINAL),
	F_PIXEL_MDP(109714000, MM_GPERF, 1,   2,    7, NOMINAL),
	F_END,
};

/* ROT */
static struct bank_masks bdiv_info_rot = {
	.bank_sel_mask = BIT(30),
	.bank0_mask = {
		.ns_mask =	BM(25, 22) | BM(18, 16),
	},
	.bank1_mask = {
		.ns_mask =	BM(29, 26) | BM(21, 19),
	},
};
#define CLK_ROT(id) \
	[L_##id##_CLK] = { \
		.type = BASIC, \
		.ns_reg = ROT_NS_REG, \
		.cc_reg = ROT_CC_REG, \
		.reset_reg = SW_RESET_CORE_REG, \
		.reset_mask = BIT(2), \
		.halt_reg = DBG_BUS_VEC_C_REG, \
		.halt_check = HALT, \
		.halt_bit = 15, \
		.br_en_mask = BIT(0), \
		.root_en_mask = BIT(2), \
		.set_rate = set_rate_div_banked, \
		.freq_tbl = clk_tbl_rot, \
		.bank_masks = &bdiv_info_rot, \
		.parent = L_NONE_CLK, \
		.test_vector = TEST_MM_HS(0x1B), \
		.current_freq = &local_dummy_freq, \
	}
#define F_ROT(f, s, d, v) \
	{ \
		.freq_hz = f, \
		.src = SRC_##s, \
		.ns_val = NS_DIVSRC_BANKED(29, 26, 25, 22, d, \
				21, 19, 18, 16, s), \
		.sys_vdd = v, \
	}
static struct clk_freq_tbl clk_tbl_rot[] = {
	F_ROT(        0, MM_GND,    1, NONE),
	F_ROT( 27000000, MM_PXO,    1, LOW),
	F_ROT( 29540000, MM_GPERF, 13, LOW),
	F_ROT( 32000000, MM_GPERF, 12, LOW),
	F_ROT( 38400000, MM_GPERF, 10, LOW),
	F_ROT( 48000000, MM_GPERF,  8, LOW),
	F_ROT( 54860000, MM_GPERF,  7, LOW),
	F_ROT( 64000000, MM_GPERF,  6, LOW),
	F_ROT( 76800000, MM_GPERF,  5, LOW),
	F_ROT( 96000000, MM_GPERF,  4, NOMINAL),
	F_ROT(100000000, MM_PLL1,   8, NOMINAL),
	F_ROT(114290000, MM_PLL1,   7, NOMINAL),
	F_ROT(133330000, MM_PLL1,   6, NOMINAL),
	F_ROT(160000000, MM_PLL1,   5, NOMINAL),
	F_END,
};

/* TV */
#define CLK_TV(id) \
	[L_##id##_CLK] = { \
		.type = MND, \
		.ns_reg = TV_NS_REG, \
		.cc_reg = TV_CC_REG, \
		.md_reg = TV_MD_REG, \
		.root_en_mask = BIT(2), \
		.ns_mask = (BM(23, 16) | BM(15, 14) | BM(2, 0)), \
		.cc_mask = BM(7, 6), \
		.set_rate = set_rate_tv, \
		.freq_tbl = clk_tbl_tv, \
		.parent = L_NONE_CLK, \
		.children = chld_tv_src, \
		.current_freq = &local_dummy_freq, \
	}
#define F_TV(f, s, p_r, d, m, n, v) \
	{ \
		.freq_hz = f, \
		.src = SRC_##s, \
		.md_val = MD8(8, m, 0, n), \
		.ns_val = NS_MM(23, 16, n, m, 15, 14, d, 2, 0, s), \
		.cc_val = CC(6, n), \
		.mnd_en_mask = BIT(5) * !!(n), \
		.sys_vdd = v, \
		.extra_freq_data = p_r, \
	}
/* Switching TV freqs requires PLL reconfiguration. */
static struct pll_rate mm_pll2_rate[] = {
	[0] = PLL_RATE( 7, 6301, 13500, 0, 4, 0x4248B), /*  50400500 Hz */
	[1] = PLL_RATE( 8,    0,     0, 0, 4, 0x4248B), /*  54000000 Hz */
	[2] = PLL_RATE(16,    2,   125, 0, 4, 0x5248F), /* 108108000 Hz */
	[3] = PLL_RATE(22,    0,     0, 2, 4, 0x6248B), /* 148500000 Hz */
	[4] = PLL_RATE(44,    0,     0, 2, 4, 0x6248F), /* 297000000 Hz */
};
static struct clk_freq_tbl clk_tbl_tv[] = {
	F_TV(        0, MM_GND,  &mm_pll2_rate[0], 1, 0, 0, NONE),
	F_TV( 25200000, MM_PLL2, &mm_pll2_rate[0], 2, 0, 0, LOW),
	F_TV( 27000000, MM_PLL2, &mm_pll2_rate[1], 2, 0, 0, LOW),
	F_TV( 27030000, MM_PLL2, &mm_pll2_rate[2], 4, 0, 0, LOW),
	F_TV( 74250000, MM_PLL2, &mm_pll2_rate[3], 2, 0, 0, NOMINAL),
	F_TV(148500000, MM_PLL2, &mm_pll2_rate[4], 2, 0, 0, NOMINAL),
	F_END,
};

/* VCODEC */
#define CLK_VCODEC(id, par) \
	[L_##id##_CLK] = { \
		.type = MND, \
		.ns_reg = VCODEC_NS_REG, \
		.cc_reg = VCODEC_CC_REG, \
		.md_reg = VCODEC_MD0_REG, \
		.reset_reg = SW_RESET_CORE_REG, \
		.reset_mask = BIT(6), \
		.halt_reg = DBG_BUS_VEC_C_REG, \
		.halt_check = HALT, \
		.halt_bit = 29, \
		.br_en_mask = BIT(0), \
		.root_en_mask = BIT(2), \
		.ns_mask = (BM(18, 11) | BM(2, 0)), \
		.cc_mask = BM(7, 6), \
		.set_rate = set_rate_mnd, \
		.freq_tbl = clk_tbl_vcodec, \
		.parent = L_##par##_CLK, \
		.test_vector = TEST_MM_HS(0x0B), \
		.current_freq = &local_dummy_freq, \
	}
#define F_VCODEC(f, s, m, n, v) \
	{ \
		.freq_hz = f, \
		.src = SRC_##s, \
		.md_val = MD8(8, m, 0, n), \
		.ns_val = NS_MM(18, 11, n, m, 0, 0, 1, 2, 0, s), \
		.cc_val = CC(6, n), \
		.mnd_en_mask = BIT(5) * !!(n), \
		.sys_vdd = v, \
	}
static struct clk_freq_tbl clk_tbl_vcodec[] = {
	F_VCODEC(        0, MM_GND,   0,  0, NONE),
	F_VCODEC( 27000000, MM_PXO,   0,  0, LOW),
	F_VCODEC( 32000000, MM_GPERF, 1, 12, LOW),
	F_VCODEC( 48000000, MM_GPERF, 1,  8, LOW),
	F_VCODEC( 54860000, MM_GPERF, 1,  7, LOW),
	F_VCODEC( 96000000, MM_GPERF, 1,  4, LOW),
	F_VCODEC(133330000, MM_PLL1,  1,  6, NOMINAL),
	F_VCODEC(200000000, MM_PLL1,  1,  4, NOMINAL),
	F_VCODEC(228570000, MM_PLL1,  2,  7, HIGH),
	F_END,
};

/* VPE */
#define CLK_VPE(id) \
	[L_##id##_CLK] = { \
		.type = BASIC, \
		.ns_reg = VPE_NS_REG, \
		.cc_reg = VPE_CC_REG, \
		.reset_reg = SW_RESET_CORE_REG, \
		.reset_mask = BIT(17), \
		.halt_reg = DBG_BUS_VEC_A_REG, \
		.halt_check = HALT, \
		.halt_bit = 28, \
		.br_en_mask = BIT(0), \
		.root_en_mask = BIT(2), \
		.ns_mask = (BM(15, 12) | BM(2, 0)), \
		.set_rate = set_rate_nop, \
		.freq_tbl = clk_tbl_vpe, \
		.parent = L_NONE_CLK, \
		.test_vector = TEST_MM_HS(0x1C), \
		.current_freq = &local_dummy_freq, \
	}
#define F_VPE(f, s, d, v) \
	{ \
		.freq_hz = f, \
		.src = SRC_##s, \
		.ns_val = NS_DIVSRC(15, 12, d, 2, 0, s), \
		.sys_vdd = v, \
	}
static struct clk_freq_tbl clk_tbl_vpe[] = {
	F_VPE(        0, MM_GND,    1, NONE),
	F_VPE( 27000000, MM_PXO,    1, LOW),
	F_VPE( 34909000, MM_GPERF, 11, LOW),
	F_VPE( 38400000, MM_GPERF, 10, LOW),
	F_VPE( 64000000, MM_GPERF,  6, LOW),
	F_VPE( 76800000, MM_GPERF,  5, LOW),
	F_VPE( 96000000, MM_GPERF,  4, NOMINAL),
	F_VPE(100000000, MM_PLL1,   8, NOMINAL),
	F_VPE(160000000, MM_PLL1,   5, NOMINAL),
	F_VPE(200000000, MM_PLL1,   4, HIGH),
	F_END,
};

/* VFE */
#define CLK_VFE(id, par) \
	[L_##id##_CLK] = { \
		.type = MND, \
		.ns_reg = VFE_NS_REG, \
		.cc_reg = VFE_CC_REG, \
		.md_reg = VFE_MD_REG, \
		.reset_reg = SW_RESET_CORE_REG, \
		.reset_mask = BIT(15), \
		.halt_reg = DBG_BUS_VEC_B_REG, \
		.halt_check = HALT, \
		.halt_bit = 6, \
		.br_en_mask = BIT(0), \
		.root_en_mask = BIT(2), \
		.ns_mask = (BM(23, 16) | BM(11, 10) | BM(2, 0)), \
		.cc_mask = BM(7, 6), \
		.set_rate = set_rate_mnd, \
		.freq_tbl = clk_tbl_vfe, \
		.parent = L_##par##_CLK, \
		.children = chld_vfe, \
		.test_vector = TEST_MM_HS(0x06), \
		.current_freq = &local_dummy_freq, \
	}
#define F_VFE(f, s, d, m, n, v) \
	{ \
		.freq_hz = f, \
		.src = SRC_##s, \
		.md_val = MD8(8, m, 0, n), \
		.ns_val = NS_MM(23, 16, n, m, 11, 10, d, 2, 0, s), \
		.cc_val = CC(6, n), \
		.mnd_en_mask = BIT(5) * !!(n), \
		.sys_vdd = v, \
	}
static struct clk_freq_tbl clk_tbl_vfe[] = {
	F_VFE(        0, MM_GND,    1, 0,  0, NONE),
	F_VFE( 13960000, MM_GPERF,  1, 2, 55, LOW),
	F_VFE( 27000000, MM_PXO,    1, 0,  0, LOW),
	F_VFE( 36570000, MM_GPERF,  1, 2, 21, LOW),
	F_VFE( 38400000, MM_GPERF,  2, 1,  5, LOW),
	F_VFE( 45180000, MM_GPERF,  1, 2, 17, LOW),
	F_VFE( 48000000, MM_GPERF,  2, 1,  4, LOW),
	F_VFE( 54860000, MM_GPERF,  1, 1,  7, LOW),
	F_VFE( 64000000, MM_GPERF,  2, 1,  3, LOW),
	F_VFE( 76800000, MM_GPERF,  1, 1,  5, LOW),
	F_VFE( 96000000, MM_GPERF,  2, 1,  2, LOW),
	F_VFE(109710000, MM_GPERF,  1, 2,  7, LOW),
	F_VFE(128000000, MM_GPERF,  1, 1,  3, NOMINAL),
	F_VFE(153600000, MM_GPERF,  1, 2,  5, NOMINAL),
	F_VFE(200000000, MM_PLL1,   2, 1,  2, NOMINAL),
	F_VFE(228570000, MM_PLL1,   1, 2,  7, NOMINAL),
	F_VFE(266667000, MM_PLL1,   1, 1,  3, HIGH),
	F_END,
};

/* Audio Interface OSR */
#define CLK_AIF_OSR(id, ns, md, h_r, tv) \
	[L_##id##_CLK] = { \
		.type = MND, \
		.ns_reg = ns, \
		.cc_reg = ns, \
		.md_reg = md, \
		.reset_reg = ns, \
		.reset_mask = BIT(19), \
		.halt_reg = h_r, \
		.halt_check = ENABLE, \
		.halt_bit = 1, \
		.br_en_mask = BIT(17), \
		.root_en_mask = BIT(9), \
		.ns_mask = (BM(31, 24) | BM(6, 0)), \
		.set_rate = set_rate_mnd, \
		.freq_tbl = clk_tbl_aif_osr, \
		.parent = L_NONE_CLK, \
		.test_vector = tv, \
		.current_freq = &local_dummy_freq, \
	}
#define F_AIF_OSR(f, s, d, m, n, v) \
	{ \
		.freq_hz = f, \
		.src = SRC_##s, \
		.md_val = MD8(8, m, 0, n), \
		.ns_val = NS(31, 24, n, m, 5, 4, 3, d, 2, 0, s), \
		.mnd_en_mask = BIT(8) * !!(n), \
		.sys_vdd = v, \
	}
static struct clk_freq_tbl clk_tbl_aif_osr[] = {
	F_AIF_OSR(       0, LPA_GND,  1, 0,   0, NONE),
	F_AIF_OSR(  768000, LPA_PLL0, 4, 1, 176, LOW),
	F_AIF_OSR( 1024000, LPA_PLL0, 4, 1, 132, LOW),
	F_AIF_OSR( 1536000, LPA_PLL0, 4, 1,  88, LOW),
	F_AIF_OSR( 2048000, LPA_PLL0, 4, 1,  66, LOW),
	F_AIF_OSR( 3072000, LPA_PLL0, 4, 1,  44, LOW),
	F_AIF_OSR( 4096000, LPA_PLL0, 4, 1,  33, LOW),
	F_AIF_OSR( 6144000, LPA_PLL0, 4, 1,  22, LOW),
	F_AIF_OSR( 8192000, LPA_PLL0, 2, 1,  33, LOW),
	F_AIF_OSR(12288000, LPA_PLL0, 4, 1,  11, LOW),
	F_AIF_OSR(24576000, LPA_PLL0, 2, 1,  11, LOW),
	F_END,
};

/* Audio Interface Bit */
#define CLK_AIF_BIT(id, ns, h_r, tv) \
	[L_##id##_CLK] = { \
		.type = BASIC, \
		.ns_reg = ns, \
		.cc_reg = ns, \
		.reset_reg = ns, \
		.reset_mask = BIT(19), \
		.halt_reg = h_r, \
		.halt_check = DELAY, \
		.br_en_mask = BIT(15), \
		.ns_mask = BM(14, 10), \
		.set_rate = set_rate_nop, \
		.freq_tbl = clk_tbl_aif_bit, \
		.parent = L_NONE_CLK, \
		.test_vector = tv, \
		.current_freq = &local_dummy_freq, \
	}
#define F_AIF_BIT(d, s) \
	{ \
		.freq_hz = d, \
		.src = SRC_NONE, 0, \
		.ns_val = (BVAL(14, 14, s) | BVAL(13, 10, (d-1))) \
	}
static struct clk_freq_tbl clk_tbl_aif_bit[] = {
	F_AIF_BIT(0, 1),  /* Use external clock. */
	F_AIF_BIT(1, 0),  F_AIF_BIT(2, 0),  F_AIF_BIT(3, 0),  F_AIF_BIT(4, 0),
	F_AIF_BIT(5, 0),  F_AIF_BIT(6, 0),  F_AIF_BIT(7, 0),  F_AIF_BIT(8, 0),
	F_AIF_BIT(9, 0),  F_AIF_BIT(10, 0), F_AIF_BIT(11, 0), F_AIF_BIT(12, 0),
	F_AIF_BIT(13, 0), F_AIF_BIT(14, 0), F_AIF_BIT(15, 0), F_AIF_BIT(16, 0),
	F_END,
};

/* PCM */
#define CLK_PCM(id) \
	[L_##id##_CLK] = { \
		.type = MND, \
		.ns_reg = LCC_PCM_NS_REG, \
		.cc_reg = LCC_PCM_NS_REG, \
		.md_reg = LCC_PCM_MD_REG, \
		.reset_reg = LCC_PCM_NS_REG, \
		.reset_mask = BIT(13), \
		.halt_reg = LCC_PCM_STATUS_REG, \
		.halt_check = ENABLE, \
		.halt_bit = 0, \
		.br_en_mask = BIT(11), \
		.root_en_mask = BIT(9), \
		.ns_mask = (BM(31, 16) | BM(6, 0)), \
		.set_rate = set_rate_mnd, \
		.freq_tbl = clk_tbl_pcm, \
		.parent = L_NONE_CLK, \
		.test_vector = TEST_LPA(0x14), \
		.current_freq = &local_dummy_freq, \
	}
#define F_PCM(f, s, d, m, n, v) \
	{ \
		.freq_hz = f, \
		.src = SRC_##s, \
		.md_val = MD16(m, n), \
		.ns_val = NS(31, 16, n, m, 5, 4, 3, d, 2, 0, s), \
		.mnd_en_mask = BIT(8) * !!(n), \
		.sys_vdd = v, \
	}
static struct clk_freq_tbl clk_tbl_pcm[] = {
	F_PCM(       0, LPA_GND,  1, 0,   0, NONE),
	F_PCM(  512000, LPA_PLL0, 4, 1, 264, LOW),
	F_PCM(  768000, LPA_PLL0, 4, 1, 176, LOW),
	F_PCM( 1024000, LPA_PLL0, 4, 1, 132, LOW),
	F_PCM( 1536000, LPA_PLL0, 4, 1,  88, LOW),
	F_PCM( 2048000, LPA_PLL0, 4, 1,  66, LOW),
	F_PCM( 3072000, LPA_PLL0, 4, 1,  44, LOW),
	F_PCM( 4096000, LPA_PLL0, 4, 1,  33, LOW),
	F_PCM( 6144000, LPA_PLL0, 4, 1,  22, LOW),
	F_PCM( 8192000, LPA_PLL0, 2, 1,  33, LOW),
	F_PCM(12288000, LPA_PLL0, 4, 1,  11, LOW),
	F_PCM(24580000, LPA_PLL0, 2, 1,  11, LOW),
	F_END,
};

/*
 * Clock children lists
 */
static const uint32_t chld_usb_fs1_src[] =	{C(USB_FS1_XCVR),
						 C(USB_FS1_SYS), C(NONE)};
static const uint32_t chld_usb_fs2_src[] =	{C(USB_FS2_XCVR),
						 C(USB_FS2_SYS), C(NONE)};
static const uint32_t chld_csi_src[] =		{C(CSI0), C(CSI1), C(NONE)};
static const uint32_t chld_pixel_mdp[] =	{C(PIXEL_LCDC), C(NONE)};
static const uint32_t chld_tv_src[] =		{C(TV_ENC), C(TV_DAC),
						 C(MDP_TV), C(HDMI_TV),
						 C(NONE)};
static const uint32_t chld_vfe[] =		{C(CSI0_VFE), C(CSI1_VFE),
						 C(NONE)};

/*
 * Clock table
 */
struct clk_local soc_clk_local_tbl[] = {
	/*
	 * Peripheral Clocks
	 */
	CLK_GSBI_UART(GSBI1_UART,   1, CLK_HALT_CFPB_STATEA_REG, HALT, 10,
			TEST_PER_LS(0x3E)),
	CLK_GSBI_UART(GSBI2_UART,   2, CLK_HALT_CFPB_STATEA_REG, HALT,  6,
			TEST_PER_LS(0x42)),
	CLK_GSBI_UART(GSBI3_UART,   3, CLK_HALT_CFPB_STATEA_REG, HALT,  2,
			TEST_PER_LS(0x46)),
	CLK_GSBI_UART(GSBI4_UART,   4, CLK_HALT_CFPB_STATEB_REG, HALT, 26,
			TEST_PER_LS(0x4A)),
	CLK_GSBI_UART(GSBI5_UART,   5, CLK_HALT_CFPB_STATEB_REG, HALT, 22,
			TEST_PER_LS(0x4E)),
	CLK_GSBI_UART(GSBI6_UART,   6, CLK_HALT_CFPB_STATEB_REG, HALT, 18,
			TEST_PER_LS(0x52)),
	CLK_GSBI_UART(GSBI7_UART,   7, CLK_HALT_CFPB_STATEB_REG, HALT, 14,
			TEST_PER_LS(0x56)),
	CLK_GSBI_UART(GSBI8_UART,   8, CLK_HALT_CFPB_STATEB_REG, HALT, 10,
			TEST_PER_LS(0x5A)),
	CLK_GSBI_UART(GSBI9_UART,   9, CLK_HALT_CFPB_STATEB_REG, HALT,  6,
			TEST_PER_LS(0x5E)),
	CLK_GSBI_UART(GSBI10_UART, 10, CLK_HALT_CFPB_STATEB_REG, HALT,  2,
			TEST_PER_LS(0x62)),
	CLK_GSBI_UART(GSBI11_UART, 11, CLK_HALT_CFPB_STATEC_REG, HALT, 17,
			TEST_PER_LS(0x66)),
	CLK_GSBI_UART(GSBI12_UART, 12, CLK_HALT_CFPB_STATEC_REG, HALT, 13,
			TEST_PER_LS(0x6A)),

	CLK_GSBI_QUP(GSBI1_QUP,   1, CLK_HALT_CFPB_STATEA_REG, HALT,  9,
			TEST_PER_LS(0x3F)),
	CLK_GSBI_QUP(GSBI2_QUP,   2, CLK_HALT_CFPB_STATEA_REG, HALT,  4,
			TEST_PER_LS(0x44)),
	CLK_GSBI_QUP(GSBI3_QUP,   3, CLK_HALT_CFPB_STATEA_REG, HALT,  0,
			TEST_PER_LS(0x48)),
	CLK_GSBI_QUP(GSBI4_QUP,   4, CLK_HALT_CFPB_STATEB_REG, HALT, 24,
			TEST_PER_LS(0x4C)),
	CLK_GSBI_QUP(GSBI5_QUP,   5, CLK_HALT_CFPB_STATEB_REG, HALT, 20,
			TEST_PER_LS(0x50)),
	CLK_GSBI_QUP(GSBI6_QUP,   6, CLK_HALT_CFPB_STATEB_REG, HALT, 16,
			TEST_PER_LS(0x54)),
	CLK_GSBI_QUP(GSBI7_QUP,   7, CLK_HALT_CFPB_STATEB_REG, HALT, 12,
			TEST_PER_LS(0x58)),
	CLK_GSBI_QUP(GSBI8_QUP,   8, CLK_HALT_CFPB_STATEB_REG, HALT,  8,
			TEST_PER_LS(0x5C)),
	CLK_GSBI_QUP(GSBI9_QUP,   9, CLK_HALT_CFPB_STATEB_REG, HALT,  4,
			TEST_PER_LS(0x60)),
	CLK_GSBI_QUP(GSBI10_QUP, 10, CLK_HALT_CFPB_STATEB_REG, HALT,  0,
			TEST_PER_LS(0x64)),
	CLK_GSBI_QUP(GSBI11_QUP, 11, CLK_HALT_CFPB_STATEC_REG, HALT, 15,
			TEST_PER_LS(0x68)),
	CLK_GSBI_QUP(GSBI12_QUP, 12, CLK_HALT_CFPB_STATEC_REG, HALT, 11,
			TEST_PER_LS(0x6C)),

	CLK_PDM(PDM),

	CLK_NORATE(PMEM, PMEM_ACLK_CTL_REG, BIT(4), NULL, 0,
		CLK_HALT_DFAB_STATE_REG, HALT, 20, TEST_PER_LS(0x26)),

	CLK_PRNG(PRNG),

	CLK_SDC(SDC1, 1, CLK_HALT_DFAB_STATE_REG, HALT, 6, TEST_PER_LS(0x13)),
	CLK_SDC(SDC2, 2, CLK_HALT_DFAB_STATE_REG, HALT, 5, TEST_PER_LS(0x15)),
	CLK_SDC(SDC3, 3, CLK_HALT_DFAB_STATE_REG, HALT, 4, TEST_PER_LS(0x17)),
	CLK_SDC(SDC4, 4, CLK_HALT_DFAB_STATE_REG, HALT, 3, TEST_PER_LS(0x19)),
	CLK_SDC(SDC5, 5, CLK_HALT_DFAB_STATE_REG, HALT, 2, TEST_PER_LS(0x1B)),

	CLK_TSIF_REF(TSIF_REF),

	CLK_TSSC(TSSC),

	CLK_USB_HS(USB_HS1_XCVR),
	CLK_RESET(USB_PHY0, USB_PHY0_RESET_REG, BIT(0)),

	CLK_USB_FS(USB_FS1_SRC, 1, chld_usb_fs1_src),
	CLK_SLAVE(USB_FS1_XCVR, USB_FSn_XCVR_FS_CLK_NS_REG(1), BIT(9),
			USB_FSn_RESET_REG(1), BIT(1), CLK_HALT_CFPB_STATEA_REG,
			HALT, 15, USB_FS1_SRC, TEST_PER_LS(0x8B)),
	CLK_SLAVE(USB_FS1_SYS, USB_FSn_SYSTEM_CLK_CTL_REG(1), BIT(4),
			USB_FSn_RESET_REG(1), BIT(0), CLK_HALT_CFPB_STATEA_REG,
			HALT, 16, USB_FS1_SRC, TEST_PER_LS(0x8A)),

	CLK_USB_FS(USB_FS2_SRC, 2, chld_usb_fs2_src),
	CLK_SLAVE(USB_FS2_XCVR,  USB_FSn_XCVR_FS_CLK_NS_REG(2), BIT(9),
			USB_FSn_RESET_REG(2), BIT(1), CLK_HALT_CFPB_STATEA_REG,
			HALT, 12, USB_FS2_SRC, TEST_PER_LS(0x8E)),
	CLK_SLAVE(USB_FS2_SYS,   USB_FSn_SYSTEM_CLK_CTL_REG(2), BIT(4),
			USB_FSn_RESET_REG(2), BIT(0), CLK_HALT_CFPB_STATEA_REG,
			HALT, 13, USB_FS2_SRC, TEST_PER_LS(0x8D)),

	/* Fast Peripheral Bus Clocks */
	CLK_NORATE(CE2_P,  CE2_HCLK_CTL_REG,  BIT(4), NULL, 0,
		CLK_HALT_CFPB_STATEC_REG, HALT, 0, TEST_PER_LS(0x93)),

	CLK_NORATE(GSBI1_P,  GSBIn_HCLK_CTL_REG(1),  BIT(4), NULL, 0,
		CLK_HALT_CFPB_STATEA_REG, HALT, 11, TEST_PER_LS(0x3D)),
	CLK_NORATE(GSBI2_P,  GSBIn_HCLK_CTL_REG(2),  BIT(4), NULL, 0,
		CLK_HALT_CFPB_STATEA_REG, HALT,  7, TEST_PER_LS(0x41)),
	CLK_NORATE(GSBI3_P,  GSBIn_HCLK_CTL_REG(3),  BIT(4), NULL, 0,
		CLK_HALT_CFPB_STATEA_REG, HALT, 3,  TEST_PER_LS(0x45)),
	CLK_NORATE(GSBI4_P,  GSBIn_HCLK_CTL_REG(4),  BIT(4), NULL, 0,
		CLK_HALT_CFPB_STATEB_REG, HALT, 27, TEST_PER_LS(0x49)),
	CLK_NORATE(GSBI5_P,  GSBIn_HCLK_CTL_REG(5),  BIT(4), NULL, 0,
		CLK_HALT_CFPB_STATEB_REG, HALT, 23, TEST_PER_LS(0x4D)),
	CLK_NORATE(GSBI6_P,  GSBIn_HCLK_CTL_REG(6),  BIT(4), NULL, 0,
		CLK_HALT_CFPB_STATEB_REG, HALT, 19, TEST_PER_LS(0x51)),
	CLK_NORATE(GSBI7_P,  GSBIn_HCLK_CTL_REG(7),  BIT(4), NULL, 0,
		CLK_HALT_CFPB_STATEB_REG, HALT, 15, TEST_PER_LS(0x55)),
	CLK_NORATE(GSBI8_P,  GSBIn_HCLK_CTL_REG(8),  BIT(4), NULL, 0,
		CLK_HALT_CFPB_STATEB_REG, HALT, 11, TEST_PER_LS(0x59)),
	CLK_NORATE(GSBI9_P,  GSBIn_HCLK_CTL_REG(9),  BIT(4), NULL, 0,
		CLK_HALT_CFPB_STATEB_REG, HALT, 7,  TEST_PER_LS(0x5D)),
	CLK_NORATE(GSBI10_P, GSBIn_HCLK_CTL_REG(10), BIT(4), NULL, 0,
		CLK_HALT_CFPB_STATEB_REG, HALT, 3,  TEST_PER_LS(0x61)),
	CLK_NORATE(GSBI11_P, GSBIn_HCLK_CTL_REG(11), BIT(4), NULL, 0,
		CLK_HALT_CFPB_STATEC_REG, HALT, 18, TEST_PER_LS(0x65)),
	CLK_NORATE(GSBI12_P, GSBIn_HCLK_CTL_REG(12), BIT(4), NULL, 0,
		CLK_HALT_CFPB_STATEC_REG, HALT, 14, TEST_PER_LS(0x69)),

	CLK_NORATE(PPSS_P, PPSS_HCLK_CTL_REG, BIT(4), NULL, 0,
		CLK_HALT_DFAB_STATE_REG, HALT, 19, TEST_PER_LS(0x2B)),

	CLK_NORATE(TSIF_P, TSIF_HCLK_CTL_REG, BIT(4), NULL, 0,
		CLK_HALT_CFPB_STATEC_REG, HALT, 7, TEST_PER_LS(0x8F)),

	CLK_NORATE(USB_FS1_P, USB_FSn_HCLK_CTL_REG(1), BIT(4), NULL, 0,
		CLK_HALT_CFPB_STATEA_REG, HALT, 17, TEST_PER_LS(0x89)),
	CLK_NORATE(USB_FS2_P, USB_FSn_HCLK_CTL_REG(2), BIT(4), NULL, 0,
		CLK_HALT_CFPB_STATEA_REG, HALT, 14, TEST_PER_LS(0x8C)),

	CLK_NORATE(USB_HS1_P, USB_HS1_HCLK_CTL_REG, BIT(4), NULL, 0,
		CLK_HALT_DFAB_STATE_REG, HALT, 1, TEST_PER_LS(0x84)),

	CLK_NORATE(SDC1_P, SDCn_HCLK_CTL_REG(1), BIT(4), NULL, 0,
		CLK_HALT_DFAB_STATE_REG, HALT, 11, TEST_PER_LS(0x12)),
	CLK_NORATE(SDC2_P, SDCn_HCLK_CTL_REG(2), BIT(4), NULL, 0,
		CLK_HALT_DFAB_STATE_REG, HALT, 10, TEST_PER_LS(0x14)),
	CLK_NORATE(SDC3_P, SDCn_HCLK_CTL_REG(3), BIT(4), NULL, 0,
		CLK_HALT_DFAB_STATE_REG, HALT,  9, TEST_PER_LS(0x16)),
	CLK_NORATE(SDC4_P, SDCn_HCLK_CTL_REG(4), BIT(4), NULL, 0,
		CLK_HALT_DFAB_STATE_REG, HALT,  8, TEST_PER_LS(0x18)),
	CLK_NORATE(SDC5_P, SDCn_HCLK_CTL_REG(5), BIT(4), NULL, 0,
		CLK_HALT_DFAB_STATE_REG, HALT,  7, TEST_PER_LS(0x1A)),

	/* HW-Voteable Clocks */
	CLK_ADM(ADM0, BIT(2), CLK_HALT_MSS_SMPSS_MISC_STATE_REG, HALT_VOTED, 14,
		TEST_PER_HS(0x2A)),
	CLK_ADM(ADM1, BIT(4), CLK_HALT_MSS_SMPSS_MISC_STATE_REG, HALT_VOTED, 12,
		TEST_PER_HS(0x2B)),
	CLK_NORATE(ADM0_P, SC0_U_CLK_BRANCH_ENA_VOTE_REG, BIT(3), NULL, 0,
		CLK_HALT_MSS_SMPSS_MISC_STATE_REG, HALT_VOTED, 13,
		TEST_PER_LS(0x80)),
	CLK_NORATE(ADM1_P, SC0_U_CLK_BRANCH_ENA_VOTE_REG, BIT(5), NULL, 0,
		CLK_HALT_MSS_SMPSS_MISC_STATE_REG, HALT_VOTED, 11,
		TEST_PER_LS(0x81)),
	CLK_NORATE(MODEM_AHB1_P, SC0_U_CLK_BRANCH_ENA_VOTE_REG, BIT(0), NULL, 0,
		CLK_HALT_MSS_SMPSS_MISC_STATE_REG, HALT_VOTED, 8,
		TEST_PER_LS(0x08)),
	CLK_NORATE(MODEM_AHB2_P, SC0_U_CLK_BRANCH_ENA_VOTE_REG, BIT(1), NULL, 0,
		CLK_HALT_MSS_SMPSS_MISC_STATE_REG, HALT_VOTED, 7,
		TEST_PER_LS(0x09)),
	CLK_NORATE(PMIC_ARB0_P, SC0_U_CLK_BRANCH_ENA_VOTE_REG, BIT(8), NULL, 0,
		CLK_HALT_SFPB_MISC_STATE_REG, HALT_VOTED, 22,
		TEST_PER_LS(0x7B)),
	CLK_NORATE(PMIC_ARB1_P, SC0_U_CLK_BRANCH_ENA_VOTE_REG, BIT(9), NULL, 0,
		CLK_HALT_SFPB_MISC_STATE_REG, HALT_VOTED, 21,
		TEST_PER_LS(0x7C)),
	CLK_NORATE(PMIC_SSBI2, SC0_U_CLK_BRANCH_ENA_VOTE_REG, BIT(7), NULL, 0,
		CLK_HALT_SFPB_MISC_STATE_REG, HALT_VOTED, 23,
		TEST_PER_LS(0x7A)),
	CLK_NORATE(RPM_MSG_RAM_P, SC0_U_CLK_BRANCH_ENA_VOTE_REG, BIT(6), NULL,
		0, CLK_HALT_SFPB_MISC_STATE_REG, HALT_VOTED, 12,
		TEST_PER_LS(0x7F)),

	/*
	 * Multimedia Clocks
	 */

	CLK_RESET(AMP, SW_RESET_CORE_REG, BIT(20)),

	CLK_CAM(CAM),

	CLK_CSI(CSI_SRC),
	CLK_SLAVE(CSI0, CSI_CC_REG, BIT(0), SW_RESET_CORE_REG, BIT(8),
		DBG_BUS_VEC_B_REG, HALT, 13, CSI_SRC, TEST_MM_HS(0x00)),
	CLK_SLAVE(CSI1, CSI_CC_REG, BIT(7), SW_RESET_CORE_REG, BIT(18),
		DBG_BUS_VEC_B_REG, HALT, 14, CSI_SRC, TEST_MM_HS(0x01)),

	CLK_DSI_BYTE(DSI_BYTE),
	CLK_NORATE(DSI_ESC, MISC_CC_REG, BIT(0), NULL, 0,
		DBG_BUS_VEC_B_REG, HALT, 24, TEST_MM_LS(0x23)),

	CLK_GFX2D0(GFX2D0),
	CLK_GFX2D1(GFX2D1),
	CLK_GFX3D(GFX3D, GMEM_AXI),

	CLK_IJPEG(IJPEG, IJPEG_AXI),

	CLK_JPEGD(JPEGD, JPEGD_AXI),

	CLK_MDP(MDP),
	CLK_MDP_VSYNC(MDP_VSYNC),

	CLK_PIXEL_MDP(PIXEL_MDP),
	CLK_SLAVE(PIXEL_LCDC, PIXEL_CC_REG, BIT(8), NULL, 0,
		DBG_BUS_VEC_C_REG, HALT, 21, PIXEL_MDP, TEST_MM_LS(0x01)),

	CLK_ROT(ROT),

	CLK_TV(TV_SRC),
	CLK_SLAVE(TV_ENC,  TV_CC_REG,  BIT(8), SW_RESET_CORE_REG, BIT(0),
		DBG_BUS_VEC_D_REG, HALT, 8,  TV_SRC, TEST_MM_LS(0x22)),
	CLK_SLAVE(TV_DAC,  TV_CC_REG,  BIT(10), NULL, 0,
		DBG_BUS_VEC_D_REG, HALT, 9,  TV_SRC, TEST_MM_LS(0x21)),
	CLK_SLAVE(MDP_TV,  TV_CC_REG,  BIT(0),  SW_RESET_CORE_REG, BIT(4),
		DBG_BUS_VEC_D_REG, HALT, 11, TV_SRC, TEST_MM_HS(0x1F)),
	CLK_SLAVE(HDMI_TV, TV_CC_REG,  BIT(12), SW_RESET_CORE_REG, BIT(1),
		DBG_BUS_VEC_D_REG, HALT, 10, TV_SRC, TEST_MM_HS(0x1E)),

	CLK_NORATE(HDMI_APP, MISC_CC2_REG, BIT(11), SW_RESET_CORE_REG, BIT(11),
		DBG_BUS_VEC_B_REG, HALT, 25, TEST_MM_LS(0x1F)),

	CLK_VCODEC(VCODEC, VCODEC_AXI),

	CLK_VPE(VPE),

	CLK_VFE(VFE, VFE_AXI),
	CLK_SLAVE(CSI0_VFE, VFE_CC_REG, BIT(12), SW_RESET_CORE_REG, BIT(24),
		DBG_BUS_VEC_B_REG, HALT, 7, VFE, TEST_MM_HS(0x03)),
	CLK_SLAVE(CSI1_VFE, VFE_CC_REG, BIT(10), SW_RESET_CORE_REG, BIT(23),
		DBG_BUS_VEC_B_REG, HALT, 8, VFE, TEST_MM_HS(0x04)),

	/* AXI Interfaces */
	CLK_NORATE(GMEM_AXI,  MAXI_EN_REG, BIT(24), NULL, 0,
		DBG_BUS_VEC_E_REG, HALT, 6, TEST_MM_HS(0x11)),
	CLK_NORATE(IJPEG_AXI, MAXI_EN_REG, BIT(21), SW_RESET_AXI_REG, BIT(14),
		DBG_BUS_VEC_E_REG, HALT, 4, TEST_MM_HS(0x12)),
	CLK_NORATE(IMEM_AXI,  MAXI_EN_REG, BIT(22), SW_RESET_CORE_REG, BIT(10),
		DBG_BUS_VEC_E_REG, HALT, 7, TEST_MM_HS(0x13)),
	CLK_NORATE(JPEGD_AXI, MAXI_EN_REG, BIT(25), NULL, 0,
		DBG_BUS_VEC_E_REG, HALT, 5, TEST_MM_HS(0x14)),
	CLK_NORATE(VCODEC_AXI,   MAXI_EN_REG, BIT(19), SW_RESET_AXI_REG,
		BIT(4)|BIT(5), DBG_BUS_VEC_E_REG, HALT, 3, TEST_MM_HS(0x17)),
	CLK_NORATE(VFE_AXI,   MAXI_EN_REG, BIT(18), SW_RESET_AXI_REG, BIT(9),
		DBG_BUS_VEC_E_REG, HALT, 0, TEST_MM_HS(0x18)),
	CLK_RESET(MDP_AXI,    SW_RESET_AXI_REG, BIT(13)),
	CLK_RESET(ROT_AXI,    SW_RESET_AXI_REG, BIT(6)),
	CLK_RESET(VPE_AXI,    SW_RESET_AXI_REG, BIT(15)),

	/* AHB Interfaces */
	CLK_NORATE(AMP_P,    AHB_EN_REG, BIT(24), NULL, 0,
		DBG_BUS_VEC_F_REG, HALT, 18, TEST_MM_LS(0x06)),
	CLK_NORATE(CSI0_P,   AHB_EN_REG, BIT(7),  SW_RESET_AHB_REG, BIT(17),
		DBG_BUS_VEC_F_REG, HALT, 16, TEST_MM_LS(0x07)),
	CLK_NORATE(CSI1_P,   AHB_EN_REG, BIT(20), SW_RESET_AHB_REG, BIT(16),
		DBG_BUS_VEC_F_REG, HALT, 17, TEST_MM_LS(0x08)),
	CLK_NORATE(DSI_M_P,  AHB_EN_REG, BIT(9),  SW_RESET_AHB_REG, BIT(6),
		DBG_BUS_VEC_F_REG, HALT, 19, TEST_MM_LS(0x09)),
	CLK_NORATE(DSI_S_P,  AHB_EN_REG, BIT(18), SW_RESET_AHB_REG, BIT(5),
		DBG_BUS_VEC_F_REG, HALT, 20, TEST_MM_LS(0x0A)),
	CLK_NORATE(GFX2D0_P, AHB_EN_REG, BIT(19), SW_RESET_AHB_REG, BIT(12),
		DBG_BUS_VEC_F_REG, HALT,  2, TEST_MM_LS(0x0C)),
	CLK_NORATE(GFX2D1_P, AHB_EN_REG, BIT(2),  SW_RESET_AHB_REG, BIT(11),
		DBG_BUS_VEC_F_REG, HALT,  3, TEST_MM_LS(0x0D)),
	CLK_NORATE(GFX3D_P,  AHB_EN_REG, BIT(3),  SW_RESET_AHB_REG, BIT(10),
		DBG_BUS_VEC_F_REG, HALT,  4, TEST_MM_LS(0x0E)),
	CLK_NORATE(HDMI_M_P, AHB_EN_REG, BIT(14), SW_RESET_AHB_REG, BIT(9),
		DBG_BUS_VEC_F_REG, HALT,  5, TEST_MM_LS(0x0F)),
	CLK_NORATE(HDMI_S_P, AHB_EN_REG, BIT(4),  SW_RESET_AHB_REG, BIT(9),
		DBG_BUS_VEC_F_REG, HALT,  6, TEST_MM_LS(0x10)),
	CLK_NORATE(IJPEG_P,  AHB_EN_REG, BIT(5),  SW_RESET_AHB_REG, BIT(7),
		DBG_BUS_VEC_F_REG, HALT,  9, TEST_MM_LS(0x11)),
	CLK_NORATE(IMEM_P,   AHB_EN_REG, BIT(6),  SW_RESET_AHB_REG, BIT(8),
		DBG_BUS_VEC_F_REG, HALT,  10, TEST_MM_LS(0x12)),
	CLK_NORATE(JPEGD_P,  AHB_EN_REG, BIT(21), SW_RESET_AHB_REG, BIT(4),
		DBG_BUS_VEC_F_REG, HALT,  7, TEST_MM_LS(0x13)),
	CLK_NORATE(MDP_P,    AHB_EN_REG, BIT(10), SW_RESET_AHB_REG, BIT(3),
		DBG_BUS_VEC_F_REG, HALT, 11, TEST_MM_LS(0x14)),
	CLK_NORATE(ROT_P,    AHB_EN_REG, BIT(12), SW_RESET_AHB_REG, BIT(2),
		DBG_BUS_VEC_F_REG, HALT, 13, TEST_MM_LS(0x16)),
	CLK_NORATE(SMMU_P,   AHB_EN_REG, BIT(15), NULL, 0,
		DBG_BUS_VEC_F_REG, HALT, 22, TEST_MM_LS(0x18)),
	CLK_NORATE(TV_ENC_P, AHB_EN_REG, BIT(25), SW_RESET_AHB_REG, BIT(15),
		DBG_BUS_VEC_F_REG, HALT, 23, TEST_MM_LS(0x19)),
	CLK_NORATE(VCODEC_P, AHB_EN_REG, BIT(11), SW_RESET_AHB_REG, BIT(1),
		DBG_BUS_VEC_F_REG, HALT, 12, TEST_MM_LS(0x1A)),
	CLK_NORATE(VFE_P,    AHB_EN_REG, BIT(13), SW_RESET_AHB_REG, BIT(0),
		DBG_BUS_VEC_F_REG, HALT, 14, TEST_MM_LS(0x1B)),
	CLK_NORATE(VPE_P,    AHB_EN_REG, BIT(16), SW_RESET_AHB_REG, BIT(14),
		DBG_BUS_VEC_F_REG, HALT, 15, TEST_MM_LS(0x1C)),

	/*
	 * Low Power Audio Clocks
	 */

	CLK_AIF_OSR(MI2S_OSR, LCC_MI2S_NS_REG, LCC_MI2S_MD_REG,
		LCC_MI2S_STATUS_REG, TEST_LPA(0x0A)),
	CLK_AIF_BIT(MI2S_BIT, LCC_MI2S_NS_REG, LCC_MI2S_STATUS_REG,
		TEST_LPA(0x0B)),

	CLK_AIF_OSR(CODEC_I2S_MIC_OSR, LCC_CODEC_I2S_MIC_NS_REG,
		LCC_CODEC_I2S_MIC_MD_REG, LCC_CODEC_I2S_MIC_STATUS_REG,
		TEST_LPA(0x0C)),
	CLK_AIF_BIT(CODEC_I2S_MIC_BIT, LCC_CODEC_I2S_MIC_NS_REG,
		LCC_CODEC_I2S_MIC_STATUS_REG, TEST_LPA(0x0D)),

	CLK_AIF_OSR(SPARE_I2S_MIC_OSR, LCC_SPARE_I2S_MIC_NS_REG,
		LCC_SPARE_I2S_MIC_MD_REG, LCC_SPARE_I2S_MIC_STATUS_REG,
		TEST_LPA(0x10)),
	CLK_AIF_BIT(SPARE_I2S_MIC_BIT, LCC_SPARE_I2S_MIC_NS_REG,
		LCC_SPARE_I2S_MIC_STATUS_REG, TEST_LPA(0x11)),

	CLK_AIF_OSR(CODEC_I2S_SPKR_OSR, LCC_CODEC_I2S_SPKR_NS_REG,
		LCC_CODEC_I2S_SPKR_MD_REG, LCC_CODEC_I2S_SPKR_STATUS_REG,
		TEST_LPA(0x0E)),
	CLK_AIF_BIT(CODEC_I2S_SPKR_BIT, LCC_CODEC_I2S_SPKR_NS_REG,
		LCC_CODEC_I2S_SPKR_STATUS_REG, TEST_LPA(0x0F)),

	CLK_AIF_OSR(SPARE_I2S_SPKR_OSR, LCC_SPARE_I2S_SPKR_NS_REG,
		LCC_SPARE_I2S_SPKR_MD_REG, LCC_SPARE_I2S_SPKR_STATUS_REG,
		TEST_LPA(0x12)),
	CLK_AIF_BIT(SPARE_I2S_SPKR_BIT, LCC_SPARE_I2S_SPKR_NS_REG,
		LCC_SPARE_I2S_SPKR_STATUS_REG, TEST_LPA(0x13)),

	CLK_PCM(PCM),
};

static struct msm_xo_voter *xo_pxo, *xo_cxo;
/*
 * SoC-specific functions required by clock-local driver
 */

/* Enable/disable for voteable XOs. */
static int xo_enable(unsigned src, unsigned enable)
{
	int xo_mode = enable ? MSM_XO_MODE_ON : MSM_XO_MODE_OFF;
	struct msm_xo_voter *xo = NULL;

	if (src == PXO)
		xo = xo_pxo;
	else if (src == CXO)
		xo = xo_cxo;

	if (!xo)
		return -ENODEV;

	return msm_xo_mode_vote(xo, xo_mode);
}

/* Enable/disable for hardware-voteable PLLs. */
static int voteable_pll_enable(unsigned src, unsigned enable)
{
	/* PLL enable/disable voting registers and masks. */
	static const struct {
		uint32_t *const enable_reg;
		const uint32_t enable_mask;
		uint32_t *const status_reg;
	} pll_reg[] = {
		[PLL_0] = { BB_PLL_ENA_SC0_REG, BIT(0), BB_PLL0_STATUS_REG },
		[PLL_6] = { BB_PLL_ENA_SC0_REG, BIT(6), BB_PLL6_STATUS_REG },
		[PLL_8] = { BB_PLL_ENA_SC0_REG, BIT(8), BB_PLL8_STATUS_REG },
	};
	int reg_val;

	reg_val = readl(pll_reg[src].enable_reg);
	if (enable) {
		reg_val |= pll_reg[src].enable_mask;
		writel(reg_val, pll_reg[src].enable_reg);

		/* Wait until PLL is enabled. */
		while (!(readl(pll_reg[src].status_reg) & BIT(16)))
			cpu_relax();
	} else {
		reg_val &= ~(pll_reg[src].enable_mask);
		writel(reg_val, pll_reg[src].enable_reg);
	}

	return 0;
}

/* Enable/disable for non-shared NT PLLs. */
static int nt_pll_enable(unsigned src, unsigned enable)
{
	static const struct {
		uint32_t *const mode_reg;
		uint32_t *const status_reg;
	} pll_reg[] = {
		[PLL_1] = { MM_PLL0_MODE_REG, MM_PLL0_STATUS_REG },
		[PLL_2] = { MM_PLL1_MODE_REG, MM_PLL1_STATUS_REG },
		[PLL_3] = { MM_PLL2_MODE_REG, MM_PLL2_STATUS_REG },
	};
	uint32_t pll_mode;

	pll_mode = readl(pll_reg[src].mode_reg);
	if (enable) {
		/* Disable PLL bypass mode. */
		pll_mode |= BIT(1);
		writel(pll_mode, pll_reg[src].mode_reg);

		/* H/W requires a 5us delay between disabling the bypass and
		 * de-asserting the reset. Delay 10us just to be safe. */
		udelay(10);

		/* De-assert active-low PLL reset. */
		pll_mode |= BIT(2);
		writel(pll_mode, pll_reg[src].mode_reg);

		/* Enable PLL output. */
		pll_mode |= BIT(0);
		writel(pll_mode, pll_reg[src].mode_reg);

		/* Wait until PLL is enabled. */
		while (!readl(pll_reg[src].status_reg))
			cpu_relax();
	} else {
		/* Disable the PLL output, disable test mode, enable
		 * the bypass mode, and assert the reset. */
		pll_mode &= ~BM(3, 0);
		writel(pll_mode, pll_reg[src].mode_reg);
	}

	return 0;
}

/* Enable/disable for RPM-controlled voteable PLL4. */
static int pll4_enable(unsigned src, unsigned enable)
{
	struct msm_rpm_iv_pair iv = { MSM_RPM_ID_PLL_4, enable };

	return msm_rpm_set_noirq(MSM_RPM_CTX_SET_0, &iv, 1);
}

#define CLK_SRC(_src, _func, _par) \
	[(_src)] = { .enable_func = (_func), .par = (_par), }
struct clk_source soc_clk_sources[NUM_SRC] = {
	CLK_SRC(CXO,   xo_enable, SRC_NONE),
	CLK_SRC(PXO,   xo_enable, SRC_NONE),
	CLK_SRC(PLL_0, voteable_pll_enable, PXO),
	CLK_SRC(PLL_1, nt_pll_enable, PXO),
	CLK_SRC(PLL_2, nt_pll_enable, PXO),
	CLK_SRC(PLL_3, nt_pll_enable, PXO),
	CLK_SRC(PLL_4, pll4_enable, PXO),
	CLK_SRC(PLL_6, voteable_pll_enable, CXO),
	CLK_SRC(PLL_7, nt_pll_enable, PXO),
	CLK_SRC(PLL_8, voteable_pll_enable, PXO),
};
#if 0
/* */
extern void msm_printk_clock_by_id (int id);

static void soc_dump_active_clocks_by_src (int src)
{
	int i;
	struct clk_local *l_clk = soc_clk_local_tbl; 

//	printk("%s: %d\n", __func__, src );

	if (src < 0 || src >= NUM_SRC) 
		return;

	if (soc_clk_local_tbl != soc_clk_local_tbl_pxo )
		return;

	for ( i = 0; i < ARRAY_SIZE(soc_clk_local_tbl_pxo); i++, l_clk++ ) {

		if (!l_clk->current_freq)
			continue;

//		printk("l_clk[%d]: %3d: %3d\n", i, 
//			l_clk->parent, l_clk->current_freq->src );

		if (!l_clk->count)
			continue;

		if ( l_clk->current_freq->src == src || l_clk->parent == src) {
			msm_printk_clock_by_id (i);
		}
	}
}


void soc_dump_active_pxo_clocks(void)
{
	int i, cnt;
	struct clk_source *c_src;

	printk("%s:\n", __func__);
	
	for ( i = 0; i < NUM_SRC; i++ ) {
		cnt = local_get_src_vote_cnt(i);
		if( cnt == 0 )
			continue; /* no votes */ 

//		printk("src_votes[%d] = %d\n", i, cnt );

		/* we have votes */
		c_src = &soc_clk_sources[i];
		if( c_src->par != PXO )
			continue; /*  */

		soc_dump_active_clocks_by_src (i);
	}

	soc_dump_active_clocks_by_src (PXO);

	printk("%s: done\n", __func__);
}

#endif

/* Update the sys_vdd voltage given a level. */
int soc_update_sys_vdd(enum sys_vdd_level level)
{
	static const int vdd_uv[] = {
		[NONE]    =  500000,
		[LOW]     = 1000000,
		[NOMINAL] = 1100000,
		[HIGH]    = 1200000,
	};

	return rpm_vreg_set_voltage(RPM_VREG_ID_PM8058_S1,
				    RPM_VREG_VOTER3, vdd_uv[level], 1);
}

/* Enable/disable a power rail associated with a clock. */
int soc_set_pwr_rail(unsigned id, int enable)
{
	/* TODO */
	return 0;
}

/* Sample clock for 'ticks' reference clock ticks. */
static uint32_t run_measurement(unsigned ticks)
{
	/* Stop counters and set the XO4 counter start value. */
	writel(0x0, RINGOSC_TCXO_CTL_REG);
	writel(ticks, RINGOSC_TCXO_CTL_REG);

	/* Wait for timer to become ready. */
	while ((readl(RINGOSC_STATUS_REG) & BIT(25)) != 0)
		cpu_relax();

	/* Run measurement and wait for completion. */
	writel(BIT(20)|ticks, RINGOSC_TCXO_CTL_REG);
	while ((readl(RINGOSC_STATUS_REG) & BIT(25)) == 0)
		cpu_relax();

	/* Stop counters. */
	writel(0x0, RINGOSC_TCXO_CTL_REG);

	/* Return measured ticks. */
	return readl(RINGOSC_STATUS_REG) & BM(24, 0);
}

/* Perform a hardware rate measurement for a given clock.
   FOR DEBUG USE ONLY: Measurements take ~15 ms! */
int soc_clk_measure_rate(unsigned id)
{
	struct clk_local *clk = &soc_clk_local_tbl[id];
	unsigned long flags;
	uint32_t clk_sel, pdm_reg_backup, ringosc_reg_backup;
	uint64_t raw_count_short, raw_count_full;
	int ret;

	spin_lock_irqsave(&local_clock_reg_lock, flags);

	/* Program the test vector. */
	clk_sel = clk->test_vector & TEST_CLK_SEL_MASK;
	switch (clk->test_vector >> TEST_TYPE_SHIFT) {
	case TEST_TYPE_PER_LS:
		writel(0x4030D00|BVAL(7, 0, clk_sel), CLK_TEST_REG);
		break;
	case TEST_TYPE_PER_HS:
		writel(0x4020000|BVAL(16, 10, clk_sel), CLK_TEST_REG);
		break;
	case TEST_TYPE_MM_LS:
		writel(0x4030D97, CLK_TEST_REG);
		writel(BVAL(6, 1, clk_sel)|BIT(0), DBG_CFG_REG_LS_REG);
		break;
	case TEST_TYPE_MM_HS:
		writel(0x402B800, CLK_TEST_REG);
		writel(BVAL(6, 1, clk_sel)|BIT(0), DBG_CFG_REG_HS_REG);
		break;
	case TEST_TYPE_LPA:
		writel(0x4030D98, CLK_TEST_REG);
		writel(BVAL(6, 1, clk_sel)|BIT(0), LCC_CLK_LS_DEBUG_CFG_REG);
		break;
	default:
		ret = -EPERM;
		goto err;
	}

	/* Enable CXO/4 and RINGOSC branch and root. */
	pdm_reg_backup = readl(PDM_CLK_NS_REG);
	ringosc_reg_backup = readl(RINGOSC_NS_REG);
	writel(0x2898, PDM_CLK_NS_REG);
	writel(0xA00, RINGOSC_NS_REG);

	/*
	 * The ring oscillator counter will not reset if the measured clock
	 * is not running.  To detect this, run a short measurement before
	 * the full measurement.  If the raw results of the two are the same
	 * then the clock must be off.
	 */

	/* Run a short measurement. (~1 ms) */
	raw_count_short = run_measurement(0x1000);
	/* Run a full measurement. (~14 ms) */
	raw_count_full = run_measurement(0x10000);

	writel(ringosc_reg_backup, RINGOSC_NS_REG);
	writel(pdm_reg_backup, PDM_CLK_NS_REG);

	/* Return 0 if the clock is off. */
	if (raw_count_full == raw_count_short)
		ret = 0;
	else {
		/* Compute rate in Hz. */
		raw_count_full = ((raw_count_full * 10) + 15) * 4800000;
		do_div(raw_count_full, ((0x10000 * 10) + 35));
		ret = (int)raw_count_full;
	}

	/* Route dbg_hs_clk to PLLTEST.  300mV single-ended amplitude. */
	writel(0x3CF8, PLLTEST_PAD_CFG_REG);
err:
	spin_unlock_irqrestore(&local_clock_reg_lock, flags);

	return ret;
}

/* Implementation for clk_set_flags(). */
int soc_clk_set_flags(unsigned id, unsigned flags)
{
	return -EPERM;
}

/* Implementation for clk_reset(). */
int soc_clk_reset(unsigned id, enum clk_reset_action action)
{
	struct clk_local *clk = &soc_clk_local_tbl[id];
	uint32_t reg_val, ret = 0;
	unsigned long flags;

	if (clk->reset_reg == NULL)
		return -EPERM;

	spin_lock_irqsave(&local_clock_reg_lock, flags);

	reg_val = readl(clk->reset_reg);
	switch (action) {
	case CLK_RESET_ASSERT:
		reg_val |= clk->reset_mask;
		break;
	case CLK_RESET_DEASSERT:
		reg_val &= ~(clk->reset_mask);
		break;
	default:
		ret = -EINVAL;
	}
	writel(reg_val, clk->reset_reg);

	spin_unlock_irqrestore(&local_clock_reg_lock, flags);

	return ret;
}

/*
 * Miscellaneous clock register initializations
 */

/* Read, modify, then write-back a register. */
static void rmwreg(uint32_t val, void *reg, uint32_t mask)
{
	uint32_t regval = readl(reg);
	regval &= ~mask;
	regval |= val;
	writel(regval, reg);
}

static void reg_init(void)
{
	/* Setup MM_PLL2 (PLL3), but turn it off. Rate set by set_rate_tv(). */
	rmwreg(0, MM_PLL2_MODE_REG, BIT(0)); /* Disable output */
	/* Set ref, bypass, assert reset, disable output, disable test mode */
	writel(0, MM_PLL2_MODE_REG); /* PXO */
	writel(0x00800000, MM_PLL2_CONFIG_REG); /* Enable main out. */

	/* TODO:
	 * The ADM clock votes below should removed once all users of the ADMs
	 * begin voting for the clocks appropriately.
	 */
	/* The clock driver doesn't use SC1's voting register to control
	 * HW-voteable clocks.  Clear its bits so that disabling bits in the
	 * SC0 register will cause the corresponding clocks to be disabled. */
	rmwreg(BIT(12)|BIT(11), SC0_U_CLK_BRANCH_ENA_VOTE_REG, BM(12, 11));
	writel(BIT(12)|BIT(11)|BM(5, 2), SC1_U_CLK_BRANCH_ENA_VOTE_REG);
	/* Let sc_aclk and sc_clk halt when both Scorpions are collapsed. */
	writel(BIT(12)|BIT(11), SC0_U_CLK_SLEEP_ENA_VOTE_REG);
	writel(BIT(12)|BIT(11), SC1_U_CLK_SLEEP_ENA_VOTE_REG);

	/* Deassert MM SW_RESET_ALL signal. */
	writel(0, SW_RESET_ALL_REG);

	/* Initialize MM AHB registers: Enable the FPB clock and disable HW
	 * gating for all clocks. Also set VFE_AHB's FORCE_CORE_ON bit to
	 * prevent its memory from being collapsed when the clock is halted.
	 * The sleep and wake-up delays are set to safe values. */
	rmwreg(0x00000003, AHB_EN_REG,  0x0F7FFFFF);
	rmwreg(0x000007F9, AHB_EN2_REG, 0x7FFFBFFF);

	/* Deassert all locally-owned MM AHB resets. */
	rmwreg(0, SW_RESET_AHB_REG, 0xFFF7DFFF);

	/* Initialize MM AXI registers: Enable HW gating for all clocks that
	 * support it. Also set FORCE_CORE_ON bits, and any sleep and wake-up
	 * delays to safe values. */
	rmwreg(0x000307F9, MAXI_EN_REG,  0x0FFFFFFF);
	/* MAXI_EN2_REG is owned by the RPM.  Don't touch it. */
	writel(0x3FE7FCFF, MAXI_EN3_REG);
	writel(0x000001D8, SAXI_EN_REG);

	/* Initialize MM CC registers: Set MM FORCE_CORE_ON bits so that core
	 * memories retain state even when not clocked. Also, set sleep and
	 * wake-up delays to safe values. */
	writel(0x00000000, CSI_CC_REG);
	rmwreg(0x00000000, MISC_CC_REG,  0xFEFFF3FF);
	rmwreg(0x000007FD, MISC_CC2_REG, 0xFFFF7FFF);
	writel(0x80FF0000, GFX2D0_CC_REG);
	writel(0x80FF0000, GFX2D1_CC_REG);
	writel(0x80FF0000, GFX3D_CC_REG);
	writel(0x80FF0000, IJPEG_CC_REG);
	writel(0x80FF0000, JPEGD_CC_REG);
	/* MDP and PIXEL clocks may be running at boot, don't turn them off. */
	rmwreg(0x80FF0000, MDP_CC_REG,   BM(31, 29) | BM(23, 16));
	rmwreg(0x80FF0000, PIXEL_CC_REG, BM(31, 29) | BM(23, 16));
	writel(0x000004FF, PIXEL_CC2_REG);
	writel(0x80FF0000, ROT_CC_REG);
	writel(0x80FF0000, TV_CC_REG);
	writel(0x000004FF, TV_CC2_REG);
	writel(0xC0FF0000, VCODEC_CC_REG);
	writel(0x80FF0000, VFE_CC_REG);
	writel(0x80FF0000, VPE_CC_REG);

	/* De-assert MM AXI resets to all hardware blocks. */
	writel(0, SW_RESET_AXI_REG);

	/* Deassert all MM core resets. */
	writel(0, SW_RESET_CORE_REG);

	/* Reset 3D core once more, with its clock enabled. This can
	 * eventually be done as part of the GDFS footswitch driver. */
	local_clk_set_rate(C(GFX3D), 27000000);
	local_clk_enable(C(GFX3D));
	writel(BIT(12), SW_RESET_CORE_REG);
	udelay(5);
	writel(0, SW_RESET_CORE_REG);
	local_clk_disable(C(GFX3D));

	/* Enable TSSC and PDM PXO sources. */
	writel(BIT(11), TSSC_CLK_CTL_REG);
	writel(BIT(15), PDM_CLK_NS_REG);
	/* Set the dsi_byte_clk src to the DSI PHY PLL,
	 * dsi_esc_clk to PXO/2, and the hdmi_app_clk src to PXO */
	rmwreg(0x400001, MISC_CC2_REG, 0x424003);
}

/* Local clock driver initialization. */
void __init msm_clk_soc_init(void)
{
	xo_pxo = msm_xo_get(MSM_XO_PXO, "clock-8x60");
	if (IS_ERR(xo_pxo)) {
		pr_err("%s: msm_xo_get(PXO) failed.\n", __func__);
		BUG();
	}
	xo_cxo = msm_xo_get(MSM_XO_TCXO_D1, "clock-8x60");
	if (IS_ERR(xo_cxo)) {
		pr_err("%s: msm_xo_get(CXO) failed.\n", __func__);
		BUG();
	}

	local_vote_sys_vdd(HIGH);
	/* Initialize clock registers. */
	reg_init();

	/* Initialize rates for clocks that only support one. */
	local_clk_set_rate(C(ADM0), 1);
	local_clk_set_rate(C(ADM1), 1);
	local_clk_set_rate(C(PDM), 27000000);
	local_clk_set_rate(C(PRNG), 64000000);
	local_clk_set_rate(C(MDP_VSYNC), 27000000);
	local_clk_set_rate(C(TSIF_REF), 105000);
	local_clk_set_rate(C(TSSC), 27000000);
	local_clk_set_rate(C(USB_HS1_XCVR), 60000000);
	local_clk_set_rate(C(USB_FS1_SRC), 60000000);
	local_clk_set_rate(C(USB_FS2_SRC), 60000000);
}

static int msm_clk_soc_late_init(void)
{
	return local_unvote_sys_vdd(HIGH);
}
late_initcall(msm_clk_soc_late_init);

/*
 * Clock operation handler registration
 */
struct clk_ops soc_clk_ops_8x60 = {
	.enable = local_clk_enable,
	.disable = local_clk_disable,
	.auto_off = local_clk_auto_off,
	.set_rate = local_clk_set_rate,
	.set_min_rate = local_clk_set_min_rate,
	.set_max_rate = local_clk_set_max_rate,
	.get_rate = local_clk_get_rate,
	.list_rate = local_clk_list_rate,
	.is_enabled = local_clk_is_enabled,
	.round_rate = local_clk_round_rate,
	.reset = soc_clk_reset,
	.set_flags = soc_clk_set_flags,
	.measure_rate = soc_clk_measure_rate,
};
