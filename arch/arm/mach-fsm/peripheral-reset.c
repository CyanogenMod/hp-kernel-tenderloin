/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/elf.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

#include <mach/msm_iomap.h>

#include "peripheral-reset.h"

#define MSM_MMS_REGS_BASE		0x10200000
#define MSM_LPASS_QDSP6SS_BASE		0x28800000

#define MARM_RESET			(MSM_CLK_CTL_BASE + 0x2BD4)
#define MARM_BOOT_CONTROL		(msm_mms_regs_base + 0x0010)
#define MAHB0_SFAB_PORT_RESET		(MSM_CLK_CTL_BASE + 0x2304)
#define MARM_CLK_BRANCH_ENA_VOTE	(MSM_CLK_CTL_BASE + 0x3000)
#define MARM_CLK_SRC0_NS		(MSM_CLK_CTL_BASE + 0x2BC0)
#define MARM_CLK_SRC1_NS		(MSM_CLK_CTL_BASE + 0x2BC4)
#define MARM_CLK_SRC_CTL		(MSM_CLK_CTL_BASE + 0x2BC8)
#define MARM_CLK_CTL			(MSM_CLK_CTL_BASE + 0x2BCC)
#define SFAB_MSS_S_HCLK_CTL		(MSM_CLK_CTL_BASE + 0x2C00)
#define MSS_MODEM_CXO_CLK_CTL		(MSM_CLK_CTL_BASE + 0x2C44)
#define MSS_SLP_CLK_CTL			(MSM_CLK_CTL_BASE + 0x2C60)
#define MSS_MARM_SYS_REF_CLK_CTL	(MSM_CLK_CTL_BASE + 0x2C64)
#define MAHB0_CLK_CTL			(MSM_CLK_CTL_BASE + 0x2300)
#define MAHB1_CLK_CTL			(MSM_CLK_CTL_BASE + 0x2BE4)
#define MAHB2_CLK_CTL			(MSM_CLK_CTL_BASE + 0x2C20)
#define MAHB1_NS			(MSM_CLK_CTL_BASE + 0x2BE0)
#define MARM_CLK_FS			(MSM_CLK_CTL_BASE + 0x2BD0)
#define MAHB2_CLK_FS			(MSM_CLK_CTL_BASE + 0x2C24)
#define PLL_ENA_MARM			(MSM_CLK_CTL_BASE + 0x3500)

#define LCC_Q6_FUNC			(MSM_LPASS_CLK_CTL_BASE + 0x001C)
#define QDSP6SS_RST_EVB			(msm_lpass_qdsp6ss_base + 0x0000)
#define QDSP6SS_STRAP_TCM		(msm_lpass_qdsp6ss_base + 0x001C)
#define QDSP6SS_STRAP_AHB		(msm_lpass_qdsp6ss_base + 0x0020)

#define PPSS_RESET			(MSM_CLK_CTL_BASE + 0x2594)

static int modem_start, q6_start, dsps_start;
static void __iomem *msm_mms_regs_base;
static void __iomem *msm_lpass_qdsp6ss_base;

int init_image(int id, const u8 *metadata, size_t size)
{
	struct elf32_hdr *ehdr = (struct elf32_hdr *)metadata;
	switch (id) {
	case PIL_MODEM:
		modem_start = ehdr->e_entry;
		break;
	case PIL_Q6:
		q6_start = ehdr->e_entry;
		break;
	case PIL_DSPS:
		dsps_start = ehdr->e_entry;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

int verify_blob(u32 phy_addr, size_t size)
{
	return 0;
}

static int reset_modem(void)
{
	u32 reg;

	/* Put modem into reset */
	writel(0x1, MARM_RESET);

	/* Put modem AHB0,1,2 clocks into reset */
	writel(BIT(0) | BIT(1), MAHB0_SFAB_PORT_RESET);
	writel(BIT(7), MAHB1_CLK_CTL);
	writel(BIT(7), MAHB2_CLK_CTL);

	/* Vote for pll8 on behalf of the modem */
	reg = readl(PLL_ENA_MARM);
	reg |= BIT(8);
	writel(reg, PLL_ENA_MARM);

	/* Set MAHB1 divider to Div-5 to run MAHB1,2 and sfab at 79.8 Mhz*/
	writel(0x4, MAHB1_NS);

	/* Vote for modem AHB1 and 2 clocks to be on on behalf of the modem */
	reg = readl(MARM_CLK_BRANCH_ENA_VOTE);
	reg |= BIT(0) | BIT(1);
	writel(reg, MARM_CLK_BRANCH_ENA_VOTE);

	/* Source marm_clk off of PLL8 */
	reg = readl(MARM_CLK_SRC_CTL);
	if ((reg & 0x1) == 0) {
		writel(0x3, MARM_CLK_SRC1_NS);
		reg |= 0x1;
	} else {
		writel(0x3, MARM_CLK_SRC0_NS);
		reg &= ~0x1;
	}
	writel(reg | 0x2, MARM_CLK_SRC_CTL);

	/*
	 * Force core on and periph on signals to remain active during halt
	 * for marm_clk and mahb2_clk
	 */
	writel(0x6F, MARM_CLK_FS);
	writel(0x6F, MAHB2_CLK_FS);

	/*
	 * Enable all of the marm_clk branches, cxo sourced marm branches,
	 * and sleep clock branches
	 */
	writel(0x10, MARM_CLK_CTL);
	writel(0x10, MAHB0_CLK_CTL);
	writel(0x10, SFAB_MSS_S_HCLK_CTL);
	writel(0x10, MSS_MODEM_CXO_CLK_CTL);
	writel(0x10, MSS_SLP_CLK_CTL);
	writel(0x10, MSS_MARM_SYS_REF_CLK_CTL);

	/* Take MAHB0,1,2 clocks out of reset */
	writel(0x0, MAHB2_CLK_CTL);
	writel(0x0, MAHB1_CLK_CTL);
	writel(0x0, MAHB0_SFAB_PORT_RESET);

	/* Wait for above clocks to be turned on */
	mb();
	/* Setup exception vector table base address */
	writel(modem_start | 0x1, MARM_BOOT_CONTROL);

	/* Wait for vector table to be setup */
	mb();

	/* Bring modem out of reset */
	writel(0x0, MARM_RESET);
	return 0;
}

static int shutdown_modem(void)
{
	u32 reg;
	/* Put modem into reset */
	writel(0x1, MARM_RESET);

	/* Put modem AHB0,1,2 clocks into reset */
	writel(BIT(0) | BIT(1), MAHB0_SFAB_PORT_RESET);
	writel(BIT(7), MAHB1_CLK_CTL);
	writel(BIT(7), MAHB2_CLK_CTL);

	/*
	 * Disable all of the marm_clk branches, cxo sourced marm branches,
	 * and sleep clock branches
	 */
	writel(0x0, MARM_CLK_CTL);
	writel(0x0, MAHB0_CLK_CTL);
	writel(0x0, SFAB_MSS_S_HCLK_CTL);
	writel(0x0, MSS_MODEM_CXO_CLK_CTL);
	writel(0x0, MSS_SLP_CLK_CTL);
	writel(0x0, MSS_MARM_SYS_REF_CLK_CTL);

	/* Disable marm_clk */
	reg = readl(MARM_CLK_SRC_CTL);
	reg &= ~0x2;
	writel(reg, MARM_CLK_SRC_CTL);

	/* Clear modem's votes for ahb clocks */
	writel(0x0, MARM_CLK_BRANCH_ENA_VOTE);

	/* Clear modem's votes for PLLs */
	writel(0x0, PLL_ENA_MARM);

	return 0;
}

#define LV_EN 			BIT(27)
#define STOP_CORE		BIT(26)
#define CLAMP_IO 		BIT(25)
#define Q6SS_PRIV_ARES		BIT(24)
#define Q6SS_SS_ARES		BIT(23)
#define Q6SS_ISDB_ARES		BIT(22)
#define Q6SS_ETM_ARES		BIT(21)
#define Q6_JTAG_CRC_EN		BIT(20)
#define Q6_JTAG_INV_EN		BIT(19)
#define Q6_JTAG_CXC_EN		BIT(18)
#define Q6_PXO_CRC_EN		BIT(17)
#define Q6_PXO_INV_EN		BIT(16)
#define Q6_PXO_CXC_EN		BIT(15)
#define Q6_PXO_SLEEP_EN		BIT(14)
#define Q6_SLP_CRC_EN		BIT(13)
#define Q6_SLP_INV_EN		BIT(12)
#define Q6_SLP_CXC_EN		BIT(11)
#define CORE_ARES		BIT(10)
#define CORE_L1_MEM_CORE_EN	BIT(9)
#define CORE_TCM_MEM_CORE_EN	BIT(8)
#define CORE_TCM_MEM_PERPH_EN	BIT(7)
#define CORE_GFM4_CLK_EN	BIT(2)
#define CORE_GFM4_RES		BIT(1)
#define RAMP_PLL_SRC_SEL	BIT(0)

#define Q6_STRAP_AHB_UPPER	(0x290 << 12)
#define Q6_STRAP_AHB_LOWER	0x280
#define Q6_STRAP_TCM_BASE	(0x28C << 15)
#define Q6_STRAP_TCM_CONFIG	0x28B

static int reset_q6(void)
{
	int ret;
	struct regulator *s3;
	u32 reg;

	/* Enable Q6 VDD */
	s3 = regulator_get(NULL, "8901_s3");
	if (IS_ERR(s3))
		return PTR_ERR(s3);
	ret = regulator_enable(s3);
	if (ret)
		return ret;
	/* Wait for VDD to settle */
	usleep_range(1000, 2000);

	/* Put Q6 into reset */
	reg = readl(LCC_Q6_FUNC);
	reg |= Q6SS_SS_ARES | Q6SS_ISDB_ARES | Q6SS_ETM_ARES | STOP_CORE |
		CORE_ARES;
	reg &= ~CORE_GFM4_CLK_EN;
	writel(reg, LCC_Q6_FUNC);

	/* Wait 8 AHB cycles for Q6 to be fully reset (AHB = 1.5Mhz) */
	usleep_range(20, 30);

	/* Turn on Q6 memory */
	reg |= CORE_GFM4_CLK_EN | CORE_L1_MEM_CORE_EN | CORE_TCM_MEM_CORE_EN |
		CORE_TCM_MEM_PERPH_EN;
	writel(reg, LCC_Q6_FUNC);

	/* Turn on Q6 core clocks and take core out of reset */
	reg &= ~(CLAMP_IO | Q6SS_SS_ARES | Q6SS_ISDB_ARES | Q6SS_ETM_ARES |
			CORE_ARES);
	writel(reg, LCC_Q6_FUNC);

	/* Wait for clocks to be enabled */
	mb();
	/* Program boot address */
	writel((q6_start >> 12) & 0xFFFFF, QDSP6SS_RST_EVB);

	writel(Q6_STRAP_TCM_CONFIG | Q6_STRAP_TCM_BASE, QDSP6SS_STRAP_TCM);
	writel(Q6_STRAP_AHB_UPPER | Q6_STRAP_AHB_LOWER, QDSP6SS_STRAP_AHB);

	/* Wait for addresses to be programmed before starting Q6 */
	mb();

	/* Start Q6 instruction execution */
	reg &= ~STOP_CORE;
	writel(reg, LCC_Q6_FUNC);

	return 0;
}

static int shutdown_q6(void)
{
	struct regulator *s3;
	u32 reg;
	int ret;

	reg = readl(LCC_Q6_FUNC);
	/* Halt clocks and turn off memory */
	reg &= ~(CORE_L1_MEM_CORE_EN | CORE_TCM_MEM_CORE_EN |
		CORE_TCM_MEM_PERPH_EN);
	reg |= CLAMP_IO | CORE_GFM4_CLK_EN;
	writel(reg, LCC_Q6_FUNC);

	/* Disable Q6 VDD */
	s3 = regulator_get(NULL, "8901_s3");
	if (IS_ERR(s3))
		return PTR_ERR(s3);
	ret = regulator_disable(s3);
	if (ret)
		return ret;
	/* Wait for VDD to settle */
	usleep_range(1000, 2000);

	return 0;
}

static int reset_dsps(void)
{
	/* Bring DSPS out of reset */
	writel(0x0, PPSS_RESET);
	return 0;
}

static int shutdown_dsps(void)
{
	return 0;
}

int auth_and_reset(int id)
{
	int ret;
	switch (id) {
	case PIL_MODEM:
		ret = reset_modem();
		break;
	case PIL_Q6:
		ret = reset_q6();
		break;
	case PIL_DSPS:
		ret = reset_dsps();
		break;
	default:
		ret = -ENODEV;
	}
	return ret;
}

int peripheral_shutdown(int id)
{
	int ret;
	switch (id) {
	case PIL_MODEM:
		ret = shutdown_modem();
		break;
	case PIL_Q6:
		ret = shutdown_q6();
		break;
	case PIL_DSPS:
		ret = shutdown_dsps();
		break;
	default:
		ret = -ENODEV;
	}
	return ret;
}

static int msm_peripheral_reset_init(void)
{
	msm_mms_regs_base = ioremap(MSM_MMS_REGS_BASE, SZ_256);
	if (!msm_mms_regs_base)
		goto err;

	msm_lpass_qdsp6ss_base = ioremap(MSM_LPASS_QDSP6SS_BASE, SZ_256);
	if (!msm_lpass_qdsp6ss_base)
		goto err_lpass;

	return 0;

err_lpass:
	iounmap(msm_mms_regs_base);
err:
	return -ENOMEM;
}

static void msm_peripheral_reset_exit(void)
{
	iounmap(msm_mms_regs_base);
	iounmap(msm_lpass_qdsp6ss_base);
}

arch_initcall(msm_peripheral_reset_init);
module_exit(msm_peripheral_reset_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Validate and bring peripherals out of reset");
