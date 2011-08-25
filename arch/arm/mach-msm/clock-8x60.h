/* Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
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

#ifndef __ARCH_ARM_MACH_MSM_CLOCK_8X60_H
#define __ARCH_ARM_MACH_MSM_CLOCK_8X60_H

#include "clock-local.h"

enum {
	/* Peripheral Clocks */
	L_GSBI1_UART_CLK,
	L_GSBI2_UART_CLK,
	L_GSBI3_UART_CLK,
	L_GSBI4_UART_CLK,
	L_GSBI5_UART_CLK,
	L_GSBI6_UART_CLK,
	L_GSBI7_UART_CLK,
	L_GSBI8_UART_CLK,
	L_GSBI9_UART_CLK,
	L_GSBI10_UART_CLK,
	L_GSBI11_UART_CLK,
	L_GSBI12_UART_CLK,
	L_GSBI1_QUP_CLK,
	L_GSBI2_QUP_CLK,
	L_GSBI3_QUP_CLK,
	L_GSBI4_QUP_CLK,
	L_GSBI5_QUP_CLK,
	L_GSBI6_QUP_CLK,
	L_GSBI7_QUP_CLK,
	L_GSBI8_QUP_CLK,
	L_GSBI9_QUP_CLK,
	L_GSBI10_QUP_CLK,
	L_GSBI11_QUP_CLK,
	L_GSBI12_QUP_CLK,
	L_PDM_CLK,
	L_PMEM_CLK,
	L_PRNG_CLK,
	L_SDC1_CLK,
	L_SDC2_CLK,
	L_SDC3_CLK,
	L_SDC4_CLK,
	L_SDC5_CLK,
	L_TSIF_REF_CLK,
	L_TSSC_CLK,
	L_USB_HS1_XCVR_CLK,
	L_USB_PHY0_CLK,
	L_USB_FS1_SRC_CLK,
	L_USB_FS1_XCVR_CLK,
	L_USB_FS1_SYS_CLK,
	L_USB_FS2_SRC_CLK,
	L_USB_FS2_XCVR_CLK,
	L_USB_FS2_SYS_CLK,

	/* HW-Voteable Clocks */
	L_ADM0_CLK,
	L_ADM0_P_CLK,
	L_ADM1_CLK,
	L_ADM1_P_CLK,
	L_MODEM_AHB1_P_CLK,
	L_MODEM_AHB2_P_CLK,
	L_PMIC_ARB0_P_CLK,
	L_PMIC_ARB1_P_CLK,
	L_PMIC_SSBI2_CLK,
	L_RPM_MSG_RAM_P_CLK,

	/* Fast Peripheral Bus Clocks */
	L_CE2_P_CLK,
	L_GSBI1_P_CLK,
	L_GSBI2_P_CLK,
	L_GSBI3_P_CLK,
	L_GSBI4_P_CLK,
	L_GSBI5_P_CLK,
	L_GSBI6_P_CLK,
	L_GSBI7_P_CLK,
	L_GSBI8_P_CLK,
	L_GSBI9_P_CLK,
	L_GSBI10_P_CLK,
	L_GSBI11_P_CLK,
	L_GSBI12_P_CLK,
	L_PPSS_P_CLK,
	L_TSIF_P_CLK,
	L_USB_FS1_P_CLK,
	L_USB_FS2_P_CLK,
	L_USB_HS1_P_CLK,
	L_SDC1_P_CLK,
	L_SDC2_P_CLK,
	L_SDC3_P_CLK,
	L_SDC4_P_CLK,
	L_SDC5_P_CLK,

	/* Multimedia Clocks */
	L_AMP_CLK,
	L_CAM_CLK,
	L_CSI_SRC_CLK,
	L_CSI0_CLK,
	L_CSI1_CLK,
	L_DSI_BYTE_CLK,
	L_DSI_ESC_CLK,
	L_GFX2D0_CLK,
	L_GFX2D1_CLK,
	L_GFX3D_CLK,
	L_IJPEG_CLK,
	L_JPEGD_CLK,
	L_MDP_CLK,
	L_MDP_VSYNC_CLK,
	L_PIXEL_SRC_CLK,
	L_PIXEL_MDP_CLK,
	L_PIXEL_LCDC_CLK,
	L_ROT_CLK,
	L_TV_SRC_CLK,
	L_TV_ENC_CLK,
	L_TV_DAC_CLK,
	L_VCODEC_CLK,
	L_MDP_TV_CLK,
	L_HDMI_TV_CLK,
	L_HDMI_APP_CLK,
	L_VPE_CLK,
	L_VFE_CLK,
	L_CSI0_VFE_CLK,
	L_CSI1_VFE_CLK,
	L_GMEM_AXI_CLK,
	L_IJPEG_AXI_CLK,
	L_IMEM_AXI_CLK,
	L_JPEGD_AXI_CLK,
	L_VCODEC_AXI_CLK,
	L_VFE_AXI_CLK,
	L_MDP_AXI_CLK,
	L_ROT_AXI_CLK,
	L_VPE_AXI_CLK,

	/* Multimedia Fast Peripheral Bus Clocks */
	L_AMP_P_CLK,
	L_CSI0_P_CLK,
	L_CSI1_P_CLK,
	L_DSI_M_P_CLK,
	L_DSI_S_P_CLK,
	L_GFX2D0_P_CLK,
	L_GFX2D1_P_CLK,
	L_GFX3D_P_CLK,
	L_HDMI_M_P_CLK,
	L_HDMI_S_P_CLK,
	L_IJPEG_P_CLK,
	L_IMEM_P_CLK,
	L_JPEGD_P_CLK,
	L_MDP_P_CLK,
	L_ROT_P_CLK,
	L_SMMU_P_CLK,
	L_TV_ENC_P_CLK,
	L_VCODEC_P_CLK,
	L_VFE_P_CLK,
	L_VPE_P_CLK,

	/* LPA Clocks */
	L_MI2S_SRC_CLK,
	L_MI2S_OSR_CLK,
	L_MI2S_BIT_CLK,
	L_CODEC_I2S_MIC_OSR_CLK,
	L_CODEC_I2S_MIC_BIT_CLK,
	L_SPARE_I2S_MIC_OSR_CLK,
	L_SPARE_I2S_MIC_BIT_CLK,
	L_CODEC_I2S_SPKR_OSR_CLK,
	L_CODEC_I2S_SPKR_BIT_CLK,
	L_SPARE_I2S_SPKR_OSR_CLK,
	L_SPARE_I2S_SPKR_BIT_CLK,
	L_PCM_CLK,

	L_NR_CLKS
};

enum clk_sources {
	PLL_0 = 0,
	PLL_1,
	PLL_2,
	PLL_3,
	PLL_4,
	PLL_6,
	PLL_7,
	PLL_8,
	PXO,
	CXO,
	NUM_SRC
};

extern struct clk_local soc_clk_local_tbl_mxo[];

struct pll_rate {
	const uint32_t	l_val;
	const uint32_t	m_val;
	const uint32_t	n_val;
	const uint32_t	vco;
	const uint32_t	post_div;
	const uint32_t	i_bits;
};
#define PLL_RATE(l, m, n, v, d, i) { l, m, n, v, (d>>1), i }

extern struct clk_ops soc_clk_ops_8x60;
#define CLK_8X60(clk_name, clk_id, clk_dev, clk_flags) {	\
	.con_id = clk_name, \
	.dev_id = clk_dev, \
	.clk = &(struct clk){ \
		.id = L_##clk_id, \
		.ops = &soc_clk_ops_8x60, \
		.flags = clk_flags, \
		.dbg_name = #clk_id, \
	}, \
	}

#endif

