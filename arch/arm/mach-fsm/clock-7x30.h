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

#ifndef __ARCH_ARM_MACH_MSM_CLOCK_7X30_H
#define __ARCH_ARM_MACH_MSM_CLOCK_7X30_H

enum {
	L_ADM_CLK,
	L_I2C_CLK,
	L_I2C_2_CLK,
	L_QUP_I2C_CLK,
	L_UART1DM_CLK,
	L_UART1DM_P_CLK,
	L_UART2DM_CLK,
	L_UART2DM_P_CLK,
	L_EMDH_CLK,
	L_EMDH_P_CLK,
	L_PMDH_CLK,
	L_PMDH_P_CLK,
	L_GRP_2D_CLK,
	L_GRP_2D_P_CLK,
	L_GRP_3D_SRC_CLK,
	L_GRP_3D_CLK,
	L_GRP_3D_P_CLK,
	L_IMEM_CLK,
	L_SDC1_CLK,
	L_SDC1_P_CLK,
	L_SDC2_CLK,
	L_SDC2_P_CLK,
	L_SDC3_CLK,
	L_SDC3_P_CLK,
	L_SDC4_CLK,
	L_SDC4_P_CLK,
	L_MDP_CLK,
	L_MDP_P_CLK,
	L_MDP_LCDC_PCLK_CLK,
	L_MDP_LCDC_PAD_PCLK_CLK,
	L_MDP_VSYNC_CLK,
	L_MI2S_CODEC_RX_M_CLK,
	L_MI2S_CODEC_RX_S_CLK,
	L_MI2S_CODEC_TX_M_CLK,
	L_MI2S_CODEC_TX_S_CLK,
	L_MI2S_M_CLK,
	L_MI2S_S_CLK,
	L_LPA_CODEC_CLK,
	L_LPA_CORE_CLK,
	L_LPA_P_CLK,
	L_MIDI_CLK,
	L_MDC_CLK,
	L_ROTATOR_IMEM_CLK,
	L_ROTATOR_P_CLK,
	L_SDAC_M_CLK,
	L_SDAC_CLK,
	L_UART1_CLK,
	L_UART2_CLK,
	L_UART3_CLK,
	L_TV_CLK,
	L_TV_DAC_CLK,
	L_TV_ENC_CLK,
	L_HDMI_CLK,
	L_TSIF_REF_CLK,
	L_TSIF_P_CLK,
	L_USB_HS_SRC_CLK,
	L_USB_HS_CLK,
	L_USB_HS_CORE_CLK,
	L_USB_HS_P_CLK,
	L_USB_HS2_CLK,
	L_USB_HS2_CORE_CLK,
	L_USB_HS2_P_CLK,
	L_USB_HS3_CLK,
	L_USB_HS3_CORE_CLK,
	L_USB_HS3_P_CLK,
	L_VFE_CLK,
	L_VFE_P_CLK,
	L_VFE_MDC_CLK,
	L_VFE_CAMIF_CLK,
	L_CAMIF_PAD_P_CLK,
	L_CAM_M_CLK,
	L_JPEG_CLK,
	L_JPEG_P_CLK,
	L_VPE_CLK,
	L_MFC_CLK,
	L_MFC_DIV2_CLK,
	L_MFC_P_CLK,
	L_SPI_CLK,
	L_SPI_P_CLK,
	L_CSI0_CLK,
	L_CSI0_VFE_CLK,
	L_CSI0_P_CLK,
	L_CSI1_CLK,
	L_CSI1_VFE_CLK,
	L_CSI1_P_CLK,
	L_GLBL_ROOT_CLK,

	L_AXI_LI_VG_CLK,
	L_AXI_LI_GRP_CLK,
	L_AXI_LI_JPEG_CLK,
	L_AXI_GRP_2D_CLK,
	L_AXI_MFC_CLK,
	L_AXI_VPE_CLK,
	L_AXI_LI_VFE_CLK,
	L_AXI_LI_APPS_CLK,
	L_AXI_MDP_CLK,
	L_AXI_IMEM_CLK,
	L_AXI_LI_ADSP_A_CLK,
	L_AXI_ROTATOR_CLK,

	L_NR_CLKS
};

enum clk_sources {
	PLL_0 = 0,
	PLL_1,
	PLL_2,
	PLL_3,
	PLL_4,
	PLL_5,
	PLL_6,
	AXI,
	LPXO,
	TCXO,
	NUM_SRC
};

extern int internal_pwr_rail_ctl_auto(unsigned rail_id, bool enable);

extern struct clk_ops soc_clk_ops_7x30;
#define CLK_7X30(clk_name, clk_id, clk_dev, clk_flags) {	\
	.name = clk_name, \
	.id = L_##clk_id, \
	.remote_id = P_##clk_id, \
	.flags = clk_flags, \
	.dev = clk_dev, \
	.dbg_name = #clk_id, \
	}

#define CLK_7X30S(clk_name, l_id, r_id, clk_dev, clk_flags) {	\
	.name = clk_name, \
	.id = L_##l_id, \
	.remote_id = P_##r_id, \
	.flags = clk_flags, \
	.dev = clk_dev, \
	.dbg_name = #l_id, \
	}

#endif

