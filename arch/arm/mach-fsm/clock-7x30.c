/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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
#include <mach/internal_power_rail.h>

#include "clock.h"
#include "clock-local.h"
#include "clock-7x30.h"
#include "proc_comm.h"

#define REG_BASE(off) (MSM_CLK_CTL_BASE + (off))
#define REG(off) (MSM_CLK_CTL_SH2_BASE + (off))

/* Shadow-region 2 (SH2) registers. */
#define	QUP_I2C_NS_REG		REG(0x04F0)
#define CAM_NS_REG		REG(0x0374)
#define CAM_VFE_NS_REG		REG(0x0044)
#define CLK_HALT_STATEA_REG	REG(0x0108)
#define CLK_HALT_STATEB_REG	REG(0x010C)
#define CLK_HALT_STATEC_REG	REG(0x02D4)
#define CSI_NS_REG		REG(0x0174)
#define EMDH_NS_REG		REG(0x0050)
#define GLBL_CLK_ENA_2_SC_REG	REG(0x03C0)
#define GLBL_CLK_ENA_SC_REG	REG(0x03BC)
#define GLBL_CLK_STATE_2_REG	REG(0x037C)
#define GLBL_CLK_STATE_REG	REG(0x0004)
#define GRP_2D_NS_REG		REG(0x0034)
#define GRP_NS_REG		REG(0x0084)
#define HDMI_NS_REG		REG(0x0484)
#define I2C_2_NS_REG		REG(0x02D8)
#define I2C_NS_REG		REG(0x0068)
#define JPEG_NS_REG		REG(0x0164)
#define LPA_CORE_CLK_MA0_REG	REG(0x04F4)
#define LPA_CORE_CLK_MA2_REG	REG(0x04FC)
#define LPA_NS_REG		REG(0x02E8)
#define MDC_NS_REG		REG(0x007C)
#define MDP_LCDC_NS_REG		REG(0x0390)
#define MDP_NS_REG		REG(0x014C)
#define MDP_VSYNC_REG		REG(0x0460)
#define MFC_NS_REG		REG(0x0154)
#define MI2S_CODEC_RX_DIV_REG	REG(0x02EC)
#define MI2S_CODEC_TX_DIV_REG	REG(0x02F0)
#define MI2S_DIV_REG		REG(0x02E4)
#define MI2S_NS_REG		REG(0x02E0)
#define MI2S_RX_NS_REG		REG(0x0070)
#define MI2S_TX_NS_REG		REG(0x0078)
#define MIDI_NS_REG		REG(0x02D0)
#define PLL_ENA_REG		REG(0x0264)
#define PMDH_NS_REG		REG(0x008C)
#define SDAC_NS_REG		REG(0x009C)
#define SDCn_NS_REG(n)		REG(0x00A4+(0x8*((n)-1)))
#define SPI_NS_REG		REG(0x02C8)
#define TSIF_NS_REG		REG(0x00C4)
#define TV_NS_REG		REG(0x00CC)
#define UART1DM_NS_REG		REG(0x00D4)
#define UART2DM_NS_REG		REG(0x00DC)
#define UART2_NS_REG		REG(0x0464)
#define UART_NS_REG		REG(0x00E0)
#define USBH2_NS_REG		REG(0x046C)
#define USBH3_NS_REG		REG(0x0470)
#define USBH_MD_REG		REG(0x02BC)
#define USBH_NS_REG		REG(0x02C0)
#define VPE_NS_REG		REG(0x015C)

/* Registers in the base (non-shadow) region. */
#define CLK_TEST_BASE_REG	REG_BASE(0x011C)
#define CLK_TEST_2_BASE_REG	REG_BASE(0x0384)
#define MISC_CLK_CTL_BASE_REG	REG_BASE(0x0110)
#define PRPH_WEB_NS_BASE_REG	REG_BASE(0x0080)
#define PLL0_STATUS_BASE_REG	REG_BASE(0x0318)
#define PLL1_STATUS_BASE_REG	REG_BASE(0x0334)
#define PLL2_STATUS_BASE_REG	REG_BASE(0x0350)
#define PLL3_STATUS_BASE_REG	REG_BASE(0x036C)
#define PLL4_STATUS_BASE_REG	REG_BASE(0x0254)
#define PLL5_STATUS_BASE_REG	REG_BASE(0x0258)
#define PLL6_STATUS_BASE_REG	REG_BASE(0x04EC)
#define RINGOSC_CNT_BASE_REG	REG_BASE(0x00FC)
#define SH2_OWN_APPS1_BASE_REG	REG_BASE(0x040C)
#define SH2_OWN_APPS2_BASE_REG	REG_BASE(0x0414)
#define SH2_OWN_APPS3_BASE_REG	REG_BASE(0x0444)
#define SH2_OWN_GLBL_BASE_REG	REG_BASE(0x0404)
#define SH2_OWN_ROW1_BASE_REG	REG_BASE(0x041C)
#define SH2_OWN_ROW2_BASE_REG	REG_BASE(0x0424)
#define TCXO_CNT_BASE_REG	REG_BASE(0x00F8)
#define TCXO_CNT_DONE_BASE_REG	REG_BASE(0x00F8)


/* MUX source input identifiers. */
#define SRC_PLL0	4 /* Modem PLL */
#define SRC_PLL1	1 /* Global PLL */
#define SRC_PLL3	3 /* Multimedia/Peripheral PLL or Backup PLL1 */
#define SRC_PLL4	2 /* Display PLL */
#define SRC_LPXO	6 /* Low-power XO */
#define SRC_TCXO	0 /* Used for sources that always source from TCXO */
#define SRC_AXI		0 /* Used for rates that sync to AXI */

/* Source name to PLL mappings. */
#define SRC_SEL_PLL0	PLL_0
#define SRC_SEL_PLL1	PLL_1
#define SRC_SEL_PLL3	PLL_3
#define SRC_SEL_PLL4	PLL_4
#define SRC_SEL_LPXO	LPXO
#define SRC_SEL_TCXO	TCXO
#define SRC_SEL_AXI	AXI

/* Clock declaration macros. */
#define MN_MODE_DUAL_EDGE	0x2
#define MD8(m, n)		(BVAL(15, 8, m) | BVAL(7, 0, ~(n)))
#define N8(msb, lsb, m, n)	(BVAL(msb, lsb, ~(n-m)) | BVAL(6, 5, \
					(MN_MODE_DUAL_EDGE * !!(n))))
#define MD16(m, n)		(BVAL(31, 16, m) | BVAL(15, 0, ~(n)))
#define N16(m, n)		(BVAL(31, 16, ~(n-m)) | BVAL(6, 5, \
					(MN_MODE_DUAL_EDGE * !!(n))))
#define SPDIV(s, d)		(BVAL(4, 3, d-1) | BVAL(2, 0, s))
#define SDIV(s, d)		(BVAL(6, 3, d-1) | BVAL(2, 0, s))
#define F_MASK_BASIC		(BM(6, 3)|BM(2, 0))
#define F_MASK_MND16		(BM(31, 16)|BM(6, 5)|BM(4, 3)|BM(2, 0))
#define F_MASK_MND8(m, l)	(BM(m, l)|BM(6, 5)|BM(4, 3)|BM(2, 0))

/*
 * Clock frequency definitions and macros
 */
#define F_BASIC(f, s, div, v) \
	F_RAW(f, SRC_##s, 0, SDIV(SRC_SEL_##s, div), 0, 0, v, NULL)
#define F_MND16(f, s, div, m, n, v) \
	F_RAW(f, SRC_##s, MD16(m, n), N16(m, n)|SPDIV(SRC_SEL_##s, div), \
		0, (B(8) * !!(n)), v, NULL)
#define F_MND8(f, nmsb, nlsb, s, div, m, n, v) \
	F_RAW(f, SRC_##s, MD8(m, n), \
		N8(nmsb, nlsb, m, n)|SPDIV(SRC_SEL_##s, div), 0, \
		(B(8) * !!(n)), v, NULL)

static struct clk_freq_tbl clk_tbl_csi[] = {
	F_MND8(153600000, 24, 17, PLL1, 2, 2, 5, NOMINAL),
	F_MND8(192000000, 24, 17, PLL1, 4, 0, 0, NOMINAL),
	F_MND8(384000000, 24, 17, PLL1, 2, 0, 0, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_tcxo[] = {
	F_RAW(19200000, SRC_TCXO, 0, 0, 0, 0, NOMINAL, NULL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_axi[] = {
	F_RAW(1, SRC_AXI, 0, 0, 0, 0, NOMINAL, NULL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_uartdm[] = {
	F_MND16( 3686400, PLL3, 3,   3, 200, NOMINAL),
	F_MND16( 7372800, PLL3, 3,   3, 100, NOMINAL),
	F_MND16(14745600, PLL3, 3,   3,  50, NOMINAL),
	F_MND16(46400000, PLL3, 3, 145, 768, NOMINAL),
	F_MND16(51200000, PLL3, 3,   5,  24, NOMINAL),
	F_MND16(58982400, PLL3, 3,   6,  25, NOMINAL),
	F_MND16(64000000, PLL1, 4,   1,   3, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_mdh[] = {
	F_BASIC( 73728000, PLL3, 10, NOMINAL),
	F_BASIC( 92160000, PLL3,  8, NOMINAL),
	F_BASIC(122880000, PLL3,  6, NOMINAL),
	F_BASIC(184320000, PLL3,  4, NOMINAL),
	F_BASIC(245760000, PLL3,  3, NOMINAL),
	F_BASIC(368640000, PLL3,  2, NOMINAL),
	F_BASIC(384000000, PLL1,  2, NOMINAL),
	F_BASIC(445500000, PLL4,  2, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_grp[] = {
	F_BASIC( 24576000, LPXO,  1, NOMINAL),
	F_BASIC( 46080000, PLL3, 16, NOMINAL),
	F_BASIC( 49152000, PLL3, 15, NOMINAL),
	F_BASIC( 52662875, PLL3, 14, NOMINAL),
	F_BASIC( 56713846, PLL3, 13, NOMINAL),
	F_BASIC( 61440000, PLL3, 12, NOMINAL),
	F_BASIC( 67025454, PLL3, 11, NOMINAL),
	F_BASIC( 73728000, PLL3, 10, NOMINAL),
	F_BASIC( 81920000, PLL3,  9, NOMINAL),
	F_BASIC( 92160000, PLL3,  8, NOMINAL),
	F_BASIC(105325714, PLL3,  7, NOMINAL),
	F_BASIC(122880000, PLL3,  6, NOMINAL),
	F_BASIC(147456000, PLL3,  5, NOMINAL),
	F_BASIC(184320000, PLL3,  4, NOMINAL),
	F_BASIC(192000000, PLL1,  4, NOMINAL),
	F_BASIC(245760000, PLL3,  3, HIGH),
	/* Sync to AXI. Hence this "rate" is not fixed. */
	F_RAW(1, SRC_AXI, 0, B(14), 0, 0, NOMINAL, NULL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_sdc1_3[] = {
	F_MND8(  144000, 19, 12, LPXO, 1,   1,  171, NOMINAL),
	F_MND8(  400000, 19, 12, LPXO, 1,   2,  123, NOMINAL),
	F_MND8(16027000, 19, 12, PLL3, 3,  14,  215, NOMINAL),
	F_MND8(17000000, 19, 12, PLL3, 4,  19,  206, NOMINAL),
	F_MND8(20480000, 19, 12, PLL3, 4,  23,  212, NOMINAL),
	F_MND8(24576000, 19, 12, LPXO, 1,   0,    0, NOMINAL),
	F_MND8(49152000, 19, 12, PLL3, 3,   1,    5, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_sdc2_4[] = {
	F_MND8(  144000, 20, 13, LPXO, 1,   1,  171, NOMINAL),
	F_MND8(  400000, 20, 13, LPXO, 1,   2,  123, NOMINAL),
	F_MND8(16027000, 20, 13, PLL3, 3,  14,  215, NOMINAL),
	F_MND8(17000000, 20, 13, PLL3, 4,  19,  206, NOMINAL),
	F_MND8(20480000, 20, 13, PLL3, 4,  23,  212, NOMINAL),
	F_MND8(24576000, 20, 13, LPXO, 1,   0,    0, NOMINAL),
	F_MND8(49152000, 20, 13, PLL3, 3,   1,    5, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_mdp_core[] = {
	F_BASIC( 46080000, PLL3, 16, NOMINAL),
	F_BASIC( 49152000, PLL3, 15, NOMINAL),
	F_BASIC( 52663000, PLL3, 14, NOMINAL),
	F_BASIC( 92160000, PLL3,  8, NOMINAL),
	F_BASIC(122880000, PLL3,  6, NOMINAL),
	F_BASIC(147456000, PLL3,  5, NOMINAL),
	F_BASIC(153600000, PLL1,  5, NOMINAL),
	F_BASIC(192000000, PLL1,  4, HIGH),
	F_END,
};

static struct clk_freq_tbl clk_tbl_mdp_lcdc[] = {
	F_MND16(24576000, LPXO, 1,   0,   0, NOMINAL),
	F_MND16(30720000, PLL3, 4,   1,   6, NOMINAL),
	F_MND16(40960000, PLL3, 2,   1,   9, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_mdp_vsync[] = {
	F_RAW(24576000, SRC_LPXO, 0, 0, 0, 0, NOMINAL, NULL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_mi2s_codec[] = {
	F_MND16( 2048000, LPXO, 4,   1,   3, NOMINAL),
	F_MND16(12288000, LPXO, 2,   0,   0, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_mi2s[] = {
	F_MND16(12288000, LPXO, 2,   0,   0, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_midi[] = {
	F_MND8(98304000, 19, 12, PLL3, 3,  2,  5, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_sdac[] = {
	F_MND16( 256000, LPXO, 4,   1,    24, NOMINAL),
	F_MND16( 352800, LPXO, 1, 147, 10240, NOMINAL),
	F_MND16( 384000, LPXO, 4,   1,    16, NOMINAL),
	F_MND16( 512000, LPXO, 4,   1,    12, NOMINAL),
	F_MND16( 705600, LPXO, 1, 147,  5120, NOMINAL),
	F_MND16( 768000, LPXO, 4,   1,     8, NOMINAL),
	F_MND16(1024000, LPXO, 4,   1,     6, NOMINAL),
	F_MND16(1411200, LPXO, 1, 147,  2560, NOMINAL),
	F_MND16(1536000, LPXO, 4,   1,     4, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_tv[] = {
	F_MND8(27000000, 23, 16, PLL4, 2,  2,  33, NOMINAL),
	F_MND8(74250000, 23, 16, PLL4, 2,  1,   6, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_usb[] = {
	F_MND8(60000000, 23, 16, PLL1, 2,  5,  32, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_vfe_jpeg[] = {
	F_MND16( 36864000, PLL3, 4,   1,   5, NOMINAL),
	F_MND16( 46080000, PLL3, 4,   1,   4, NOMINAL),
	F_MND16( 61440000, PLL3, 4,   1,   3, NOMINAL),
	F_MND16( 73728000, PLL3, 2,   1,   5, NOMINAL),
	F_MND16( 81920000, PLL3, 3,   1,   3, NOMINAL),
	F_MND16( 92160000, PLL3, 4,   1,   2, NOMINAL),
	F_MND16( 98304000, PLL3, 3,   2,   5, NOMINAL),
	F_MND16(105326000, PLL3, 2,   2,   7, NOMINAL),
	F_MND16(122880000, PLL3, 2,   1,   3, NOMINAL),
	F_MND16(147456000, PLL3, 2,   2,   5, NOMINAL),
	F_MND16(153600000, PLL1, 2,   2,   5, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_cam[] = {
	F_MND16( 6000000, PLL1, 4,   1,  32, NOMINAL),
	F_MND16( 8000000, PLL1, 4,   1,  24, NOMINAL),
	F_MND16(12000000, PLL1, 4,   1,  16, NOMINAL),
	F_MND16(16000000, PLL1, 4,   1,  12, NOMINAL),
	F_MND16(19200000, PLL1, 4,   1,  10, NOMINAL),
	F_MND16(24000000, PLL1, 4,   1,   8, NOMINAL),
	F_MND16(32000000, PLL1, 4,   1,   6, NOMINAL),
	F_MND16(48000000, PLL1, 4,   1,   4, NOMINAL),
	F_MND16(64000000, PLL1, 4,   1,   3, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_vpe[] = {
	F_MND8( 24576000, 22, 15, LPXO, 1,   0,   0, NOMINAL),
	F_MND8( 30720000, 22, 15, PLL3, 4,   1,   6, NOMINAL),
	F_MND8( 61440000, 22, 15, PLL3, 4,   1,   3, NOMINAL),
	F_MND8( 81920000, 22, 15, PLL3, 3,   1,   3, NOMINAL),
	F_MND8(122880000, 22, 15, PLL3, 3,   1,   2, NOMINAL),
	F_MND8(147456000, 22, 15, PLL3, 1,   1,   5, NOMINAL),
	F_MND8(153600000, 22, 15, PLL1, 1,   1,   5, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_mfc[] = {
	F_MND8( 24576000, 24, 17, LPXO, 1,   0,   0, NOMINAL),
	F_MND8( 30720000, 24, 17, PLL3, 4,   1,   6, NOMINAL),
	F_MND8( 61440000, 24, 17, PLL3, 4,   1,   3, NOMINAL),
	F_MND8( 81920000, 24, 17, PLL3, 3,   1,   3, NOMINAL),
	F_MND8(122880000, 24, 17, PLL3, 3,   1,   2, NOMINAL),
	F_MND8(147456000, 24, 17, PLL3, 1,   1,   5, NOMINAL),
	F_MND8(153600000, 24, 17, PLL1, 1,   1,   5, NOMINAL),
	F_MND8(170667000, 24, 17, PLL1, 1,   2,   9, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_spi[] = {
	F_MND8( 9963243, 19, 12, PLL3, 4,   7,   129, NOMINAL),
	F_MND8(26331429, 19, 12, PLL3, 4,  34,   241, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_lpa_codec[] = {
	F_RAW(1, SRC_NONE, 0, 0, 0, 0, LOW, NULL), /* src = MI2S_CODEC_RX */
	F_RAW(2, SRC_NONE, 0, 1, 0, 0, LOW, NULL), /* src = ECODEC_CIF */
	F_RAW(3, SRC_NONE, 0, 2, 0, 0, LOW, NULL), /* src = MI2S */
	F_RAW(4, SRC_NONE, 0, 3, 0, 0, LOW, NULL), /* src = SDAC */
	F_END,
};

/*
 * Clock children lists
 */
static const uint32_t chld_grp_3d_src[] = 	{C(IMEM), C(GRP_3D), C(NONE)};
static const uint32_t chld_mdp_lcdc_p[] = 	{C(MDP_LCDC_PAD_PCLK), C(NONE)};
static const uint32_t chld_mfc[] = 		{C(MFC_DIV2), C(NONE)};
static const uint32_t chld_mi2s_codec_rx[] =	{C(MI2S_CODEC_RX_S), C(NONE)};
static const uint32_t chld_mi2s_codec_tx[] =	{C(MI2S_CODEC_TX_S), C(NONE)};
static const uint32_t chld_mi2s[] = 		{C(MI2S_S), C(NONE)};
static const uint32_t chld_sdac[] = 		{C(SDAC_M), C(NONE)};
static const uint32_t chld_tv[] = 		{C(TV_DAC), C(TV_ENC), C(HDMI),
						 C(TSIF_REF), C(NONE)};
static const uint32_t chld_usb_src[] = 	{C(USB_HS), C(USB_HS_CORE),
					 C(USB_HS2), C(USB_HS2_CORE),
					 C(USB_HS3), C(USB_HS3_CORE),
					 C(NONE)};
static uint32_t const chld_vfe[] =	{C(VFE_MDC), C(VFE_CAMIF), C(CSI0_VFE),
					 C(NONE)};

/*
 * Clock declaration macros
 */
#define CLK_BASIC(id, ns, br, root, tbl, par, h_r, h_c, h_b, tv) \
		CLK(id, BASIC, ns, ns, NULL, NULL, 0, h_r, h_c, \
			h_b, br, root, F_MASK_BASIC, 0, set_rate_basic, tbl, \
			NULL, par, NULL, tv)
#define CLK_MND8_P(id, ns, m, l, br, root, tbl, par, chld_lst, \
						h_r, h_c, h_b, tv) \
		CLK(id, MND, ns, ns, (ns-4), NULL, 0, h_r, h_c, \
			h_b, br, root, F_MASK_MND8(m, l), 0, set_rate_mnd, \
			tbl, NULL, par, chld_lst, tv)
#define CLK_MND8(id, ns, m, l, br, root, tbl, chld_lst, h_r, h_c, h_b, tv) \
		CLK_MND8_P(id, ns, m, l, br, root, tbl, NONE, chld_lst, \
							h_r, h_c, h_b, tv)
#define CLK_MND16(id, ns, br, root, tbl, par, chld_lst, h_r, h_c, h_b, tv) \
		CLK(id, MND, ns, ns, (ns-4), NULL, 0, h_r, h_c, \
			h_b, br, root, F_MASK_MND16, 0, set_rate_mnd, tbl, \
			NULL, par, chld_lst, tv)
#define CLK_1RATE(id, ns, br, root, tbl, h_r, h_c, h_b, tv) \
		CLK(id, BASIC, NULL, ns, NULL, NULL, 0, h_r, h_c, \
			h_b, br, root, 0, 0, set_rate_nop, tbl, \
			NULL, NONE, NULL, tv)
#define CLK_SLAVE(id, ns, br, par, h_r, h_c, h_b, tv) \
		CLK(id, NORATE, NULL, ns, NULL, NULL, 0, h_r, h_c, \
			h_b, br, 0, 0, 0, NULL, NULL, NULL, par, NULL, tv)
#define CLK_NORATE(id, ns, br, root, h_r, h_c, h_b, tv) \
		CLK(id, NORATE, NULL, ns, NULL, NULL, 0, h_r, h_c, \
			h_b, br, root, 0, 0, NULL, NULL, NULL, NONE, NULL, tv)
#define CLK_GLBL(id, glbl, br, h_r, h_c, h_b, tv) \
		CLK(id, NORATE, NULL, glbl, NULL, NULL, 0, h_r, h_c, \
			h_b, br, 0, 0, 0, NULL, NULL, NULL, NONE, NULL, tv)
#define CLK_BRIDGE(id, glbl, br, par, h_r, h_c, h_b, tv) \
		CLK(id, NORATE, NULL, glbl, NULL, NULL, 0, h_r, h_c, \
			h_b, br, 0, 0, 0, NULL, NULL, NULL, par, NULL, tv)

/*
 * Clock table
 */
struct clk_local soc_clk_local_tbl[] = {
	CLK_NORATE(MDC,	MDC_NS_REG, B(9), B(11),
			CLK_HALT_STATEA_REG, HALT, 10, 0x4D56),
	CLK_NORATE(LPA_CORE, LPA_NS_REG, B(5), 0,
			CLK_HALT_STATEC_REG, HALT, 5, 0x0E),

	CLK_1RATE(I2C, I2C_NS_REG, B(9), B(11), clk_tbl_tcxo,
			CLK_HALT_STATEA_REG, HALT, 15, 0x4D4D),
	CLK_1RATE(I2C_2, I2C_2_NS_REG, B(0), B(2), clk_tbl_tcxo,
			CLK_HALT_STATEC_REG, HALT, 2, 0x0B),
	CLK_1RATE(QUP_I2C, QUP_I2C_NS_REG, B(0), B(2), clk_tbl_tcxo,
			CLK_HALT_STATEB_REG, HALT, 31, 0x1C),
	CLK_1RATE(UART1, UART_NS_REG, B(5), B(4), clk_tbl_tcxo,
			CLK_HALT_STATEB_REG, HALT, 7, 0x4D6F),
	CLK_1RATE(UART2, UART2_NS_REG, B(5), B(4), clk_tbl_tcxo,
			CLK_HALT_STATEB_REG, HALT, 5, 0x4D71),

	CLK_BASIC(EMDH,	EMDH_NS_REG, 0, B(11), clk_tbl_mdh, AXI_LI_ADSP_A,
			NULL, DELAY, 0, 0x4F00),
	CLK_BASIC(PMDH,	PMDH_NS_REG, 0, B(11), clk_tbl_mdh, AXI_LI_ADSP_A,
			NULL, DELAY, 0, 0x5500),
	CLK_BASIC(MDP,	MDP_NS_REG, B(9), B(11), clk_tbl_mdp_core, AXI_MDP,
			CLK_HALT_STATEB_REG, HALT, 16, 0x5400),

	CLK_MND8_P(VPE, VPE_NS_REG, 22, 15, B(9), B(11), clk_tbl_vpe,
			AXI_VPE, NULL, CLK_HALT_STATEC_REG, HALT, 10, 0x6C00),
	CLK_MND8_P(MFC, MFC_NS_REG, 24, 17, B(9), B(11), clk_tbl_mfc,
			AXI_MFC, chld_mfc, CLK_HALT_STATEC_REG,
			HALT, 12, 0x38),
	CLK_SLAVE(MFC_DIV2, MFC_NS_REG, B(15), MFC, CLK_HALT_STATEC_REG,
			HALT, 11, 0x1F),

	CLK_MND8(SDC1,	SDCn_NS_REG(1), 19, 12, B(9), B(11), clk_tbl_sdc1_3,
			NULL, CLK_HALT_STATEA_REG, HALT, 1, 0x4D62),
	CLK_MND8(SDC2,	SDCn_NS_REG(2), 20, 13, B(9), B(11), clk_tbl_sdc2_4,
			NULL, CLK_HALT_STATEA_REG, HALT, 0, 0x4D64),
	CLK_MND8(SDC3,	SDCn_NS_REG(3), 19, 12, B(9), B(11), clk_tbl_sdc1_3,
			NULL, CLK_HALT_STATEB_REG, HALT, 24, 0x4D7A),
	CLK_MND8(SDC4,	SDCn_NS_REG(4), 20, 13, B(9), B(11), clk_tbl_sdc2_4,
			NULL, CLK_HALT_STATEB_REG, HALT, 25, 0x4D7C),
	CLK_MND8(SPI,	SPI_NS_REG, 19, 12, B(9), B(11), clk_tbl_spi, NULL,
			CLK_HALT_STATEC_REG, HALT, 0, 0x09),
	CLK_MND8(MIDI,	MIDI_NS_REG, 19, 12, B(9), B(11), clk_tbl_midi,	NULL,
			CLK_HALT_STATEC_REG, HALT, 1, 0x0A),
	CLK_MND8_P(USB_HS_SRC, USBH_NS_REG, 23, 16, 0, B(11), clk_tbl_usb,
			AXI_LI_ADSP_A,	chld_usb_src, NULL, NOCHECK, 0, 0),
	CLK_SLAVE(USB_HS,	USBH_NS_REG,	B(9),	USB_HS_SRC,
			CLK_HALT_STATEB_REG, HALT, 26, 0x4D7F),
	CLK_SLAVE(USB_HS_CORE,	USBH_NS_REG,	B(13),	USB_HS_SRC,
			CLK_HALT_STATEA_REG, HALT, 27, 0x14),
	CLK_SLAVE(USB_HS2,	USBH2_NS_REG,	B(9),	USB_HS_SRC,
			CLK_HALT_STATEB_REG, HALT, 3, 0x4D73),
	CLK_SLAVE(USB_HS2_CORE,	USBH2_NS_REG,	B(4),	USB_HS_SRC,
			CLK_HALT_STATEA_REG, HALT, 28, 0x15),
	CLK_SLAVE(USB_HS3,	USBH3_NS_REG,	B(9),	USB_HS_SRC,
			CLK_HALT_STATEB_REG, HALT, 2, 0x4D74),
	CLK_SLAVE(USB_HS3_CORE,	USBH3_NS_REG,	B(4),	USB_HS_SRC,
			CLK_HALT_STATEA_REG, HALT, 29, 0x16),
	CLK_MND8(TV,	TV_NS_REG, 23, 16, 0, B(11), clk_tbl_tv, chld_tv,
			NULL, NOCHECK, 0, 0),
	CLK_SLAVE(HDMI,	HDMI_NS_REG, B(9),  TV,
			CLK_HALT_STATEC_REG, HALT, 7, 0x13),
	CLK_SLAVE(TV_DAC, TV_NS_REG, B(12), TV,
			CLK_HALT_STATEB_REG, HALT, 27, 0x4D6C),
	CLK_SLAVE(TV_ENC, TV_NS_REG, B(9),  TV,
			CLK_HALT_STATEB_REG, HALT, 10, 0x4D6B),
	/* Hacking root & branch into one param. */
	CLK_SLAVE(TSIF_REF, TSIF_NS_REG, B(9)|B(11), TV,
			CLK_HALT_STATEB_REG, HALT, 11, 0x4D6A),

	CLK_MND16(UART1DM, UART1DM_NS_REG, B(9), B(11), clk_tbl_uartdm, NONE,
			NULL, CLK_HALT_STATEB_REG, HALT, 6, 0x4D70),
	CLK_MND16(UART2DM, UART2DM_NS_REG, B(9), B(11), clk_tbl_uartdm, NONE,
			NULL, CLK_HALT_STATEB_REG, HALT, 23, 0x4D7D),
	CLK_MND16(JPEG, JPEG_NS_REG, B(9), B(11), clk_tbl_vfe_jpeg, AXI_LI_JPEG,
			NULL, CLK_HALT_STATEB_REG, HALT, 1, 0x6000),
	CLK_MND16(CAM_M, CAM_NS_REG, 0, B(9), clk_tbl_cam, NONE, NULL,
			NULL, DELAY, 0, 0x4D44),
	CLK_MND16(VFE, CAM_VFE_NS_REG, B(9), B(13), clk_tbl_vfe_jpeg,
			AXI_LI_VFE, chld_vfe, CLK_HALT_STATEB_REG,
			HALT, 0, 0x4D76),
	CLK_SLAVE(VFE_MDC, CAM_VFE_NS_REG, B(11), VFE, CLK_HALT_STATEA_REG,
			HALT, 9, 0x4D57),
	CLK_SLAVE(VFE_CAMIF, CAM_VFE_NS_REG, B(15), VFE, CLK_HALT_STATEC_REG,
			HALT, 13, 0x7000),
	CLK_SLAVE(CSI0_VFE, CSI_NS_REG, B(15), VFE, CLK_HALT_STATEA_REG,
			HALT, 4, 0),

	CLK_MND16(SDAC, SDAC_NS_REG, B(9), B(11), clk_tbl_sdac,
			NONE, chld_sdac, CLK_HALT_STATEA_REG, HALT, 2, 0x4D60),
	CLK_SLAVE(SDAC_M, SDAC_NS_REG, B(12), SDAC, CLK_HALT_STATEB_REG,
			HALT, 17, 0x4D66),

	CLK_MND16(MDP_LCDC_PCLK, MDP_LCDC_NS_REG, B(9), B(11),
			clk_tbl_mdp_lcdc, NONE, chld_mdp_lcdc_p,
			CLK_HALT_STATEB_REG, HALT, 28, 0x4200),
	CLK_SLAVE(MDP_LCDC_PAD_PCLK, MDP_LCDC_NS_REG, B(12), MDP_LCDC_PCLK,
			CLK_HALT_STATEB_REG, HALT, 29, 0x4100),
	CLK_1RATE(MDP_VSYNC, MDP_VSYNC_REG, B(0), 0, clk_tbl_mdp_vsync,
			CLK_HALT_STATEB_REG, HALT, 30, 0x4D53),

	CLK_MND16(MI2S_CODEC_RX_M, MI2S_RX_NS_REG, B(12), B(11),
				clk_tbl_mi2s_codec, NONE, chld_mi2s_codec_rx,
				CLK_HALT_STATEA_REG, HALT, 12, 0x4D4E),
	CLK_SLAVE(MI2S_CODEC_RX_S, MI2S_RX_NS_REG, B(9), MI2S_CODEC_RX_M,
			CLK_HALT_STATEA_REG, HALT, 13, 0x4D4F),

	CLK_MND16(MI2S_CODEC_TX_M, MI2S_TX_NS_REG, B(12), B(11),
				clk_tbl_mi2s_codec, NONE, chld_mi2s_codec_tx,
				CLK_HALT_STATEC_REG, HALT, 8, 0x4D50),
	CLK_SLAVE(MI2S_CODEC_TX_S, MI2S_TX_NS_REG, B(9), MI2S_CODEC_TX_M,
			CLK_HALT_STATEA_REG, HALT, 11, 0x17),

	CLK_MND16(MI2S_M, MI2S_NS_REG, B(12), B(11),
			clk_tbl_mi2s, NONE, chld_mi2s, CLK_HALT_STATEC_REG,
			HALT, 4, 0x0D),
	CLK_SLAVE(MI2S_S, MI2S_NS_REG, B(9), MI2S_M, CLK_HALT_STATEC_REG,
			HALT, 3, 0),

	CLK(GRP_2D, BASIC, GRP_2D_NS_REG, GRP_2D_NS_REG, NULL, NULL, 0,
			CLK_HALT_STATEA_REG, HALT, 31, B(7), B(11),
			F_MASK_BASIC | (7 << 12), 0, set_rate_basic,
			clk_tbl_grp, NULL, AXI_GRP_2D, NULL, 0x5C00),
	CLK(GRP_3D_SRC, BASIC, GRP_NS_REG, GRP_NS_REG, NULL, NULL, 0,
			NULL, NOCHECK, 0, 0, B(11), F_MASK_BASIC | (7 << 12),
			0, set_rate_basic, clk_tbl_grp, NULL, AXI_LI_GRP,
			chld_grp_3d_src, 0),
	CLK_SLAVE(GRP_3D, GRP_NS_REG, B(7), GRP_3D_SRC, CLK_HALT_STATEB_REG,
			HALT, 18, 0x5E00),
	CLK_SLAVE(IMEM, GRP_NS_REG, B(9), GRP_3D_SRC, CLK_HALT_STATEB_REG,
			HALT, 19, 0x5F00),
	CLK(LPA_CODEC, BASIC, LPA_NS_REG, LPA_NS_REG, NULL, NULL, 0,
			CLK_HALT_STATEC_REG, HALT, 6, B(9), 0, BM(1, 0), 0,
			set_rate_basic,	clk_tbl_lpa_codec, NULL, NONE, NULL,
			0x0F),

	CLK_MND8(CSI0, CSI_NS_REG, 24, 17, B(9), B(11), clk_tbl_csi, NULL,
			CLK_HALT_STATEB_REG, HALT, 9, 0x5F00),

	/* For global clocks to be on we must have GLBL_ROOT_ENA set */
	CLK_1RATE(GLBL_ROOT, GLBL_CLK_ENA_SC_REG, 0, B(29), clk_tbl_axi,
			NULL, NOCHECK, 0, 0),

	/* Peripheral bus clocks. */
	CLK_GLBL(ADM,		GLBL_CLK_ENA_SC_REG,	B(5),
				GLBL_CLK_STATE_REG,	DELAY, 0, 0x4000),
	CLK_GLBL(CAMIF_PAD_P,	GLBL_CLK_ENA_SC_REG,	B(9),
				GLBL_CLK_STATE_REG,	HALT, 9, 0x1A),
	CLK_GLBL(CSI0_P,	GLBL_CLK_ENA_SC_REG,	B(30),
				GLBL_CLK_STATE_REG,	HALT, 30, 0),
	CLK_GLBL(EMDH_P,	GLBL_CLK_ENA_2_SC_REG,	B(3),
				GLBL_CLK_STATE_2_REG,	DELAY, 0, 0x03),
	CLK_GLBL(GRP_2D_P,	GLBL_CLK_ENA_SC_REG,	B(24),
				GLBL_CLK_STATE_REG,	HALT, 24, 0x4D4C),
	CLK_GLBL(GRP_3D_P,	GLBL_CLK_ENA_2_SC_REG,	B(17),
				GLBL_CLK_STATE_2_REG,	HALT, 17, 0x4D67),
	CLK_GLBL(JPEG_P,	GLBL_CLK_ENA_2_SC_REG,	B(24),
				GLBL_CLK_STATE_2_REG,	HALT, 24, 0x4D5E),
	CLK_GLBL(LPA_P,		GLBL_CLK_ENA_2_SC_REG,	B(7),
				GLBL_CLK_STATE_2_REG,	HALT, 7, 0x07),
	CLK_GLBL(MDP_P,		GLBL_CLK_ENA_2_SC_REG,	B(6),
				GLBL_CLK_STATE_2_REG,	DELAY, 0, 0x06),
	CLK_GLBL(MFC_P,		GLBL_CLK_ENA_2_SC_REG,	B(26),
				GLBL_CLK_STATE_2_REG,	HALT, 26, 0x4D75),
	CLK_GLBL(PMDH_P,	GLBL_CLK_ENA_2_SC_REG,	B(4),
				GLBL_CLK_STATE_2_REG,	DELAY, 0, 0x04),
	CLK_GLBL(ROTATOR_IMEM,	GLBL_CLK_ENA_2_SC_REG,	B(23),
				GLBL_CLK_STATE_2_REG,	HALT, 23, 0x6600),
	CLK_GLBL(ROTATOR_P,	GLBL_CLK_ENA_2_SC_REG,	B(25),
				GLBL_CLK_STATE_2_REG,	HALT, 25, 0x4D6D),
	CLK_GLBL(SDC1_P,	GLBL_CLK_ENA_SC_REG,	B(7),
				GLBL_CLK_STATE_REG,	HALT, 7, 0x4D61),
	CLK_GLBL(SDC2_P,	GLBL_CLK_ENA_SC_REG,	B(8),
				GLBL_CLK_STATE_REG,	HALT, 8, 0x4F63),
	CLK_GLBL(SDC3_P,	GLBL_CLK_ENA_SC_REG,	B(27),
				GLBL_CLK_STATE_REG,	HALT, 27, 0x4D79),
	CLK_GLBL(SDC4_P,	GLBL_CLK_ENA_SC_REG,	B(28),
				GLBL_CLK_STATE_REG,	HALT, 28, 0x4D7B),
	CLK_GLBL(SPI_P,		GLBL_CLK_ENA_2_SC_REG,	B(10),
				GLBL_CLK_STATE_2_REG,	HALT, 10, 0x18),
	CLK_GLBL(TSIF_P,	GLBL_CLK_ENA_SC_REG,	B(18),
				GLBL_CLK_STATE_REG,	HALT, 18, 0x4D65),
	CLK_GLBL(UART1DM_P,	GLBL_CLK_ENA_SC_REG,	B(17),
				GLBL_CLK_STATE_REG,	HALT, 17, 0x4D5C),
	CLK_GLBL(UART2DM_P,	GLBL_CLK_ENA_SC_REG,	B(26),
				GLBL_CLK_STATE_REG,	HALT, 26, 0x4D7E),
	CLK_GLBL(USB_HS2_P,	GLBL_CLK_ENA_2_SC_REG,	B(8),
				GLBL_CLK_STATE_2_REG,	HALT, 8, 0x08),
	CLK_GLBL(USB_HS3_P,	GLBL_CLK_ENA_2_SC_REG,	B(9),
				GLBL_CLK_STATE_2_REG,	HALT, 9, 0x10),
	CLK_GLBL(USB_HS_P,	GLBL_CLK_ENA_SC_REG,	B(25),
				GLBL_CLK_STATE_REG,	HALT, 25, 0x4D58),
	CLK_GLBL(VFE_P,		GLBL_CLK_ENA_2_SC_REG,	B(27),
				GLBL_CLK_STATE_2_REG,	HALT, 27, 0x4D55),

	/* AXI bridge clocks. */
	CLK_BRIDGE(AXI_LI_APPS,	GLBL_CLK_ENA_SC_REG,	B(2),	GLBL_ROOT,
			GLBL_CLK_STATE_REG, HALT, 2, 0x4900),
	CLK_BRIDGE(AXI_LI_ADSP_A, GLBL_CLK_ENA_2_SC_REG, B(14), AXI_LI_APPS,
			GLBL_CLK_STATE_2_REG, DELAY, 0, 0x6400),
	CLK_BRIDGE(AXI_LI_JPEG,	GLBL_CLK_ENA_2_SC_REG,	B(19),	AXI_LI_APPS,
			GLBL_CLK_STATE_2_REG, HALT, 19, 0x4E00),
	CLK_BRIDGE(AXI_LI_VFE,	GLBL_CLK_ENA_SC_REG,	B(23),	AXI_LI_APPS,
			GLBL_CLK_STATE_REG, DELAY, 0, 0x5B00),
	CLK_BRIDGE(AXI_MDP,	GLBL_CLK_ENA_2_SC_REG,	B(29),	AXI_LI_APPS,
			GLBL_CLK_STATE_2_REG, HALT, 29, 0x6B00),

	CLK_BRIDGE(AXI_IMEM,	GLBL_CLK_ENA_2_SC_REG,	B(18),	GLBL_ROOT,
			GLBL_CLK_STATE_2_REG, HALT, 18, 0x4B00),

	CLK_BRIDGE(AXI_LI_VG,	GLBL_CLK_ENA_SC_REG,	B(3),	GLBL_ROOT,
			GLBL_CLK_STATE_REG, DELAY, 0, 0x4700),
	CLK_BRIDGE(AXI_GRP_2D,	GLBL_CLK_ENA_SC_REG,	B(21),	AXI_LI_VG,
			GLBL_CLK_STATE_REG, DELAY, 0, 0x5900),
	CLK_BRIDGE(AXI_LI_GRP,	GLBL_CLK_ENA_SC_REG,	B(22),	AXI_LI_VG,
			GLBL_CLK_STATE_REG, DELAY, 0, 0x5A00),
	CLK_BRIDGE(AXI_MFC,	GLBL_CLK_ENA_2_SC_REG,	B(20),	AXI_LI_VG,
			GLBL_CLK_STATE_2_REG, HALT, 20, 0x6A00),
	CLK_BRIDGE(AXI_ROTATOR,	GLBL_CLK_ENA_2_SC_REG,	B(22),	AXI_LI_VG,
			GLBL_CLK_STATE_2_REG, HALT, 22, 0x4300),
	CLK_BRIDGE(AXI_VPE,	GLBL_CLK_ENA_2_SC_REG,	B(21),	AXI_LI_VG,
			GLBL_CLK_STATE_2_REG, HALT, 21, 0x6700),
};

/*
 * SoC-specific functions required by clock-local driver
 */

/* Update the sys_vdd voltage given a level. */
int soc_update_sys_vdd(enum sys_vdd_level level)
{
	int rc, target_mv;
	static const int mv[NUM_SYS_VDD_LEVELS] = {
		[LOW]     = 1000,
		[NOMINAL] = 1100,
		[HIGH]    = 1200,
	};

	target_mv = mv[level];
	rc = msm_proc_comm(PCOM_CLKCTL_RPC_MIN_MSMC1, &target_mv, NULL);
	if (rc)
		goto out;
	if (target_mv) {
		rc = -EINVAL;
		goto out;
	}
out:
	return rc;
}

/* Enable/disable a power rail associated with a clock. */
int soc_set_pwr_rail(unsigned id, int enable)
{
	int pwr_id = 0;
	switch (id) {
	case C(AXI_ROTATOR):
		pwr_id = PWR_RAIL_ROTATOR_CLK;
		break;
	case C(GRP_2D):
		pwr_id = PWR_RAIL_GRP_2D_CLK;
		break;
	case C(GRP_3D):
		pwr_id = PWR_RAIL_GRP_CLK;
		break;
	case C(MDP):
		pwr_id = PWR_RAIL_MDP_CLK;
		break;
	case C(MFC):
		pwr_id = PWR_RAIL_MFC_CLK;
		break;
	case C(VFE):
		pwr_id = PWR_RAIL_VFE_CLK;
		break;
	case C(VPE):
		pwr_id = PWR_RAIL_VPE_CLK;
		break;
	default:
		return 0;
	}

	return internal_pwr_rail_ctl_auto(pwr_id, enable);
}

/* Sample clock for 'tcxo4_ticks' reference clock ticks. */
static uint32_t run_measurement(unsigned tcxo4_ticks)
{
	/* TCXO4_CNT_EN and RINGOSC_CNT_EN register values. */
	uint32_t reg_val_enable = readl(MISC_CLK_CTL_BASE_REG) | 0x3;
	uint32_t reg_val_disable = reg_val_enable & ~0x3;

	/* Stop counters and set the TCXO4 counter start value. */
	writel(reg_val_disable, MISC_CLK_CTL_BASE_REG);
	writel(tcxo4_ticks, TCXO_CNT_BASE_REG);

	/* Run measurement and wait for completion. */
	writel(reg_val_enable, MISC_CLK_CTL_BASE_REG);
	while (readl(TCXO_CNT_DONE_BASE_REG) == 0)
		cpu_relax();

	/* Stop counters. */
	writel(reg_val_disable, MISC_CLK_CTL_BASE_REG);

	return readl(RINGOSC_CNT_BASE_REG);
}

/* Perform a hardware rate measurement for a given clock.
   FOR DEBUG USE ONLY: Measurements take ~15 ms! */
signed soc_clk_measure_rate(unsigned id)
{
	struct clk_local *t = &soc_clk_local_tbl[id];
	unsigned long flags;
	uint32_t regval, prph_web_reg_old;
	uint64_t raw_count_short, raw_count_full;
	signed ret;

	if (t->test_vector == 0)
		return -EPERM;

	spin_lock_irqsave(&local_clock_reg_lock, flags);

	/* Program test vector. */
	if (t->test_vector <= 0xFF) {
		/* Select CLK_TEST_2 */
		writel(0x4D40, CLK_TEST_BASE_REG);
		writel(t->test_vector, CLK_TEST_2_BASE_REG);
	} else
		writel(t->test_vector, CLK_TEST_BASE_REG);

	/* Enable TCXO4 clock branch and root. */
	prph_web_reg_old = readl(PRPH_WEB_NS_BASE_REG);
	regval = prph_web_reg_old | B(9) | B(11);
	local_src_enable(TCXO);
	writel(regval, PRPH_WEB_NS_BASE_REG);

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

	/* Disable TCXO4 clock branch and root. */
	writel(prph_web_reg_old, PRPH_WEB_NS_BASE_REG);
	local_src_disable(TCXO);

	/* Return 0 if the clock is off. */
	if (raw_count_full == raw_count_short)
		ret = 0;
	else {
		/* Compute rate in Hz. */
		raw_count_full = ((raw_count_full * 10) + 15) * 4800000;
		do_div(raw_count_full, ((0x10000 * 10) + 35));
		ret = (signed)raw_count_full;
	}

	spin_unlock_irqrestore(&local_clock_reg_lock, flags);

	return ret;
}

/* Implementation for clk_set_flags(). */
int soc_clk_set_flags(unsigned id, unsigned clk_flags)
{
	uint32_t regval, ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&local_clock_reg_lock, flags);
	switch (id) {
	case C(VFE):
		regval = readl(CAM_VFE_NS_REG);
		/* Flag values chosen for backward compatibility
		 * with proc_comm remote clock control. */
		if (clk_flags == 0x00000100) {
			/* Select external source. */
			regval |= B(14);
		} else if (clk_flags == 0x00000200) {
			/* Select internal source. */
			regval &= ~B(14);
		} else
			ret = -EINVAL;

		writel(regval, CAM_VFE_NS_REG);
		break;
	default:
		ret = -EPERM;
	}
	spin_unlock_irqrestore(&local_clock_reg_lock, flags);

	return ret;
}

/* Implementation for clk_reset(). */
int soc_clk_reset(unsigned id, enum clk_reset_action action)
{
	/* Local clock resets are not support on 7x30. */
	return -EPERM;
}

/* Enable function for TCXO and LPXO. */
static int soc_xo_enable(unsigned src, unsigned enable)
{
	unsigned pcom_xo_id;

	if (src == TCXO)
		pcom_xo_id = 0;
	else if (src == LPXO)
		pcom_xo_id = 1;
	else
		return 0;

	return msm_proc_comm(PCOM_CLKCTL_RPC_SRC_REQUEST, &pcom_xo_id, &enable);
}

/* Enable function for PLLs. */
static int soc_pll_enable(unsigned src, unsigned enable)
{
	static const struct pll_ena {
		uint32_t *const reg;
		const uint32_t mask;
	} soc_pll_ena[NUM_SRC] = {
		[PLL_0] = {PLL_ENA_REG, B(0)},
		[PLL_1] = {PLL_ENA_REG, B(1)},
		[PLL_2] = {PLL_ENA_REG, B(2)},
		[PLL_3] = {PLL_ENA_REG, B(3)},
		[PLL_4] = {PLL_ENA_REG, B(4)},
		[PLL_5] = {PLL_ENA_REG, B(5)},
		[PLL_6] = {PLL_ENA_REG, B(6)},
	};
	uint32_t *const soc_pll_status_reg[NUM_SRC] = {

		[PLL_0] = PLL0_STATUS_BASE_REG,
		[PLL_1] = PLL1_STATUS_BASE_REG,
		[PLL_2] = PLL2_STATUS_BASE_REG,
		[PLL_3] = PLL3_STATUS_BASE_REG,
		[PLL_4] = PLL4_STATUS_BASE_REG,
		[PLL_5] = PLL5_STATUS_BASE_REG,
		[PLL_6] = PLL6_STATUS_BASE_REG,
	};
	uint32_t reg_val;

	reg_val = readl(soc_pll_ena[src].reg);
	reg_val |= soc_pll_ena[src].mask;
	writel(reg_val, soc_pll_ena[src].reg);

	/* Wait until PLL is enabled */
	while ((readl(soc_pll_status_reg[src]) & B(16)) == 0)
		cpu_relax();

	return 0;
}

struct clk_source soc_clk_sources[NUM_SRC] = {
	[TCXO] =	{	.enable_func = soc_xo_enable,
				.par = SRC_NONE,
			},
	[LPXO] =	{	.enable_func = soc_xo_enable,
				.par = SRC_NONE,
			},
	[AXI] =		{	.enable_func = NULL,
				.par = LPXO,
			},
	[PLL_0] =	{	.enable_func = soc_pll_enable,
				.par = TCXO,
			},
	[PLL_1] =	{	.enable_func = soc_pll_enable,
				.par = TCXO,
			},
	[PLL_2] =	{	.enable_func = soc_pll_enable,
				.par = TCXO,
			},
	[PLL_3] =	{	.enable_func = soc_pll_enable,
				.par = LPXO,
			},
	[PLL_4] =	{	.enable_func = soc_pll_enable,
				.par = LPXO,
			},
	[PLL_5] =	{	.enable_func = soc_pll_enable,
				.par = TCXO,
			},
	[PLL_6] =	{	.enable_func = soc_pll_enable,
				.par = TCXO,
			},
};

/*
 * Clock ownership detection code
 */

enum {
	SH2_OWN_GLBL,
	SH2_OWN_APPS1,
	SH2_OWN_APPS2,
	SH2_OWN_ROW1,
	SH2_OWN_ROW2,
	SH2_OWN_APPS3,
	NUM_OWNERSHIP
};
static __initdata uint32_t ownership_regs[NUM_OWNERSHIP];

static void __init cache_ownership(void)
{
	ownership_regs[SH2_OWN_GLBL]  = readl(SH2_OWN_GLBL_BASE_REG);
	ownership_regs[SH2_OWN_APPS1] = readl(SH2_OWN_APPS1_BASE_REG);
	ownership_regs[SH2_OWN_APPS2] = readl(SH2_OWN_APPS2_BASE_REG);
	ownership_regs[SH2_OWN_ROW1]  = readl(SH2_OWN_ROW1_BASE_REG);
	ownership_regs[SH2_OWN_ROW2]  = readl(SH2_OWN_ROW2_BASE_REG);
	ownership_regs[SH2_OWN_APPS3] = readl(SH2_OWN_APPS3_BASE_REG);
}

static void __init print_ownership(void)
{
	pr_info("Clock ownership\n");
	pr_info("  GLBL  : %08x\n", ownership_regs[SH2_OWN_GLBL]);
	pr_info("  APPS  : %08x %08x %08x\n", ownership_regs[SH2_OWN_APPS1],
		ownership_regs[SH2_OWN_APPS2], ownership_regs[SH2_OWN_APPS3]);
	pr_info("  ROW   : %08x %08x\n", ownership_regs[SH2_OWN_ROW1],
		ownership_regs[SH2_OWN_ROW2]);
}

/*
 * This is a many-to-one mapping since we don't know how the remote clock code
 * has decided to handle the dependencies between clocks for a particular
 * hardware block. We determine the ownership for all the clocks on a block by
 * checking the ownership bit of one register (usually the ns register).
 */
#define O(x) &ownership_regs[x]
static const struct clk_local_ownership {
	const uint32_t *reg;
	const uint32_t bit;
} ownership_map[] __initconst = {
	[C(GRP_2D)]			= { O(SH2_OWN_APPS1), B(6) },
	[C(GRP_2D_P)]			= { O(SH2_OWN_APPS1), B(6) },
	[C(HDMI)]			= { O(SH2_OWN_APPS1), B(31) },
	[C(JPEG)]			= { O(SH2_OWN_APPS1), B(0) },
	[C(JPEG_P)]			= { O(SH2_OWN_APPS1), B(0) },
	[C(LPA_CODEC)]			= { O(SH2_OWN_APPS1), B(23) },
	[C(LPA_CORE)]			= { O(SH2_OWN_APPS1), B(23) },
	[C(LPA_P)]			= { O(SH2_OWN_APPS1), B(23) },
	[C(MI2S_M)]			= { O(SH2_OWN_APPS1), B(28) },
	[C(MI2S_S)]			= { O(SH2_OWN_APPS1), B(28) },
	[C(MI2S_CODEC_RX_M)]		= { O(SH2_OWN_APPS1), B(12) },
	[C(MI2S_CODEC_RX_S)]		= { O(SH2_OWN_APPS1), B(12) },
	[C(MI2S_CODEC_TX_M)]		= { O(SH2_OWN_APPS1), B(14) },
	[C(MI2S_CODEC_TX_S)]		= { O(SH2_OWN_APPS1), B(14) },
	[C(MIDI)]			= { O(SH2_OWN_APPS1), B(22) },
	[C(SDAC)]			= { O(SH2_OWN_APPS1), B(26) },
	[C(SDAC_M)]			= { O(SH2_OWN_APPS1), B(26) },
	[C(VFE)]			= { O(SH2_OWN_APPS1), B(8) },
	[C(VFE_CAMIF)]			= { O(SH2_OWN_APPS1), B(8) },
	[C(VFE_MDC)]			= { O(SH2_OWN_APPS1), B(8) },
	[C(VFE_P)]			= { O(SH2_OWN_APPS1), B(8) },

	[C(GRP_3D)]			= { O(SH2_OWN_APPS2), B(0) },
	[C(GRP_3D_P)]			= { O(SH2_OWN_APPS2), B(0) },
	[C(GRP_3D_SRC)]			= { O(SH2_OWN_APPS2), B(0) },
	[C(IMEM)]			= { O(SH2_OWN_APPS2), B(0) },
	[C(MDP_LCDC_PAD_PCLK)]		= { O(SH2_OWN_APPS2), B(4) },
	[C(MDP_LCDC_PCLK)]		= { O(SH2_OWN_APPS2), B(4) },
	[C(MDP_P)]			= { O(SH2_OWN_APPS2), B(4) },
	[C(MDP_VSYNC)]			= { O(SH2_OWN_APPS2), B(28) },
	[C(TSIF_REF)]			= { O(SH2_OWN_APPS2), B(5) },
	[C(TSIF_P)]			= { O(SH2_OWN_APPS2), B(5) },
	[C(TV)]				= { O(SH2_OWN_APPS2), B(2) },
	[C(TV_DAC)]			= { O(SH2_OWN_APPS2), B(2) },
	[C(TV_ENC)]			= { O(SH2_OWN_APPS2), B(2) },

	[C(EMDH)]			= { O(SH2_OWN_ROW1), B(7) },
	[C(EMDH_P)]			= { O(SH2_OWN_ROW1), B(7) },
	[C(I2C)]			= { O(SH2_OWN_ROW1), B(11) },
	[C(I2C_2)]			= { O(SH2_OWN_ROW1), B(12) },
	[C(MDC)]			= { O(SH2_OWN_ROW1), B(17) },
	[C(PMDH)]			= { O(SH2_OWN_ROW1), B(19) },
	[C(PMDH_P)]			= { O(SH2_OWN_ROW1), B(19) },
	[C(SDC1)]			= { O(SH2_OWN_ROW1), B(23) },
	[C(SDC1_P)]			= { O(SH2_OWN_ROW1), B(23) },
	[C(SDC2)]			= { O(SH2_OWN_ROW1), B(25) },
	[C(SDC2_P)]			= { O(SH2_OWN_ROW1), B(25) },
	[C(SDC3)]			= { O(SH2_OWN_ROW1), B(27) },
	[C(SDC3_P)]			= { O(SH2_OWN_ROW1), B(27) },
	[C(SDC4)]			= { O(SH2_OWN_ROW1), B(29) },
	[C(SDC4_P)]			= { O(SH2_OWN_ROW1), B(29) },
	[C(UART2)]			= { O(SH2_OWN_ROW1), B(0) },
	[C(USB_HS2)]			= { O(SH2_OWN_ROW1), B(2) },
	[C(USB_HS2_CORE)]		= { O(SH2_OWN_ROW1), B(2) },
	[C(USB_HS2_P)]			= { O(SH2_OWN_ROW1), B(2) },
	[C(USB_HS3)]			= { O(SH2_OWN_ROW1), B(4) },
	[C(USB_HS3_CORE)]		= { O(SH2_OWN_ROW1), B(4) },
	[C(USB_HS3_P)]			= { O(SH2_OWN_ROW1), B(4) },

	[C(QUP_I2C)]			= { O(SH2_OWN_ROW2), B(3) },
	[C(SPI)]			= { O(SH2_OWN_ROW2), B(1) },
	[C(SPI_P)]			= { O(SH2_OWN_ROW2), B(1) },
	[C(UART1)]			= { O(SH2_OWN_ROW2), B(9) },
	[C(UART1DM)]			= { O(SH2_OWN_ROW2), B(6) },
	[C(UART1DM_P)]			= { O(SH2_OWN_ROW2), B(6) },
	[C(UART2DM)]			= { O(SH2_OWN_ROW2), B(8) },
	[C(UART2DM_P)]			= { O(SH2_OWN_ROW2), B(8) },
	[C(USB_HS)]			= { O(SH2_OWN_ROW2), B(11) },
	[C(USB_HS_CORE)]		= { O(SH2_OWN_ROW2), B(11) },
	[C(USB_HS_SRC)]			= { O(SH2_OWN_ROW2), B(11) },
	[C(USB_HS_P)]			= { O(SH2_OWN_ROW2), B(11) },

	[C(CAM_M)]			= { O(SH2_OWN_APPS3), B(6) },
	[C(CAMIF_PAD_P)]		= { O(SH2_OWN_APPS3), B(6) },
	[C(CSI0)]			= { O(SH2_OWN_APPS3), B(11) },
	[C(CSI0_VFE)]			= { O(SH2_OWN_APPS3), B(11) },
	[C(CSI0_P)]			= { O(SH2_OWN_APPS3), B(11) },
	[C(MDP)]			= { O(SH2_OWN_APPS3), B(0) },
	[C(MFC)]			= { O(SH2_OWN_APPS3), B(2) },
	[C(MFC_DIV2)]			= { O(SH2_OWN_APPS3), B(2) },
	[C(MFC_P)]			= { O(SH2_OWN_APPS3), B(2) },
	[C(VPE)]			= { O(SH2_OWN_APPS3), B(4) },

	[C(ADM)]			= { O(SH2_OWN_GLBL), B(8) },
	[C(AXI_ROTATOR)]		= { O(SH2_OWN_GLBL), B(13) },
	[C(ROTATOR_IMEM)]		= { O(SH2_OWN_GLBL), B(13) },
	[C(ROTATOR_P)]			= { O(SH2_OWN_GLBL), B(13) },
};

static struct clk_ops * __init clk_is_local(uint32_t id)
{
	uint32_t local, bit = ownership_map[id].bit;
	const uint32_t *reg = ownership_map[id].reg;

	BUG_ON(id >= ARRAY_SIZE(ownership_map) || !reg);

	local = *reg & bit;
	return local ? &soc_clk_ops_7x30 : NULL;
}

/* SoC-specific clk_ops initialization. */
void __init msm_clk_soc_set_ops(struct clk *clk)
{
	if (!clk->ops) {
		struct clk_ops *ops = clk_is_local(clk->id);
		if (ops)
			clk->ops = ops;
		else {
			clk->ops = &clk_ops_remote;
			clk->id = clk->remote_id;
		}
	}
}

/*
 * Miscellaneous clock register initializations
 */
static const struct reg_init {
	const void *reg;
	uint32_t mask;
	uint32_t val;
} ri_list[] __initconst = {
	/* Enable UMDX_P clock. Known to causes issues, so never turn off. */
	{GLBL_CLK_ENA_2_SC_REG, B(2), B(2)},

	{EMDH_NS_REG, BM(18, 17) , BVAL(18, 17, 0x3)}, /* RX div = div-4. */
	{PMDH_NS_REG, BM(18, 17), BVAL(18, 17, 0x3)}, /* RX div = div-4. */
	/* MI2S_CODEC_RX_S src = MI2S_CODEC_RX_M. */
	{MI2S_RX_NS_REG, B(14), 0x0},
	/* MI2S_CODEC_TX_S src = MI2S_CODEC_TX_M. */
	{MI2S_TX_NS_REG, B(14), 0x0},
	{MI2S_NS_REG, B(14), 0x0}, /* MI2S_S src = MI2S_M. */
	/* Allow DSP to decide the LPA CORE src. */
	{LPA_CORE_CLK_MA0_REG, B(0), B(0)},
	{LPA_CORE_CLK_MA2_REG, B(0), B(0)},
	{MI2S_CODEC_RX_DIV_REG, 0xF, 0xD}, /* MI2S_CODEC_RX_S div = div-8. */
	{MI2S_CODEC_TX_DIV_REG, 0xF, 0xD}, /* MI2S_CODEC_TX_S div = div-8. */
	{MI2S_DIV_REG, 0xF, 0x7}, /* MI2S_S div = div-8. */
	{MDC_NS_REG, 0x3, 0x3}, /* MDC src = external MDH src. */
	{SDAC_NS_REG, BM(15, 14), 0x0}, /* SDAC div = div-1. */
	/* Disable sources TCXO/5 & TCXO/6. UART1 src = TCXO*/
	{UART_NS_REG, BM(26, 25) | BM(2, 0), 0x0},
	{MDP_VSYNC_REG, 0xC, 0x4}, /* MDP VSYNC src = LPXO. */
	/* HDMI div = div-1, non-inverted. tv_enc_src = tv_clk_src */
	{HDMI_NS_REG, 0x7, 0x0},
	{TV_NS_REG, BM(15, 14), 0x0}, /* tv_clk_src_div2 = div-1 */

	/* USBH core clocks src = USB_HS_SRC. */
	{USBH_NS_REG, B(15), B(15)},
	{USBH2_NS_REG, B(6), B(6)},
	{USBH3_NS_REG, B(6), B(6)},
};

/* Local clock driver initialization. */
void __init msm_clk_soc_init(void)
{
	int i;
	uint32_t val;

	cache_ownership();
	print_ownership();

	/* When we have no local clock control, the rest of the code in this
	 * function is a NOP since writes to shadow regions that we don't own
	 * are ignored. */

	/* Disable all the child clocks of USB_HS_SRC. This needs to be done
	 * before the register init loop since it changes the source of the
	 * USB HS core clocks. */
	for (i = 0; chld_usb_src[i] != C(NONE); i++)
		if (clk_is_local(chld_usb_src[i]))
			local_clk_disable_reg(chld_usb_src[i]);

	if (clk_is_local(C(USB_HS_SRC)))
		local_clk_set_rate(C(USB_HS_SRC), clk_tbl_usb[0].freq_hz);

	for (i = 0; i < ARRAY_SIZE(ri_list); i++) {
		val = readl(ri_list[i].reg);
		val &= ~ri_list[i].mask;
		val |= ri_list[i].val;
		writel(val, ri_list[i].reg);
	}

	/* This is just to update the driver data structures. The actual
	 * register set up is taken care of in the register init loop
	 * or is the default value out of reset. */
	set_1rate(I2C);
	set_1rate(I2C_2);
	set_1rate(QUP_I2C);
	set_1rate(UART1);
	set_1rate(UART2);
	set_1rate(LPA_CODEC);
	set_1rate(GLBL_ROOT);
}

/*
 * Clock operation handler registration
 */
struct clk_ops soc_clk_ops_7x30 = {
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

