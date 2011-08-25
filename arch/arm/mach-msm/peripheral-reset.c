/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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
#include <linux/module.h>
#include <linux/slab.h>

#include <mach/scm.h>
#include <mach/msm_iomap.h>

#include "peripheral-loader.h"
#include "clock-8x60.h"

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

#define PAS_MODEM	0
#define PAS_Q6		1
#define PAS_DSPS	2

#define PAS_INIT_IMAGE_CMD	1
#define PAS_MEM_CMD		2
#define PAS_AUTH_AND_RESET_CMD	5
#define PAS_SHUTDOWN_CMD	6

struct pas_init_image_req {
	u32	proc;
	u32	image_addr;
};

struct pas_init_image_resp {
	u32	image_valid;
};

struct pas_auth_image_req {
	u32	proc;
};

struct pas_auth_image_resp {
	u32	reset_initiated;
};

struct pas_shutdown_req {
	u32	proc;
};

struct pas_shutdown_resp {
	u32	success;
};

static int modem_start, q6_start, dsps_start;
static void __iomem *msm_mms_regs_base;
static void __iomem *msm_lpass_qdsp6ss_base;

static int init_image_trusted(int id, const u8 *metadata, size_t size)
{
	int ret;
	struct pas_init_image_req request;
	struct pas_init_image_resp resp = {0};
	void *mdata_buf;

	/* Make memory physically contiguous */
	mdata_buf = kmemdup(metadata, size, GFP_KERNEL);
	if (!mdata_buf)
		return -ENOMEM;

	request.proc = id;
	request.image_addr = virt_to_phys(mdata_buf);

	ret = scm_call(SCM_SVC_PIL, PAS_INIT_IMAGE_CMD, &request,
			sizeof(request), &resp, sizeof(resp));
	kfree(mdata_buf);

	if (ret)
		return ret;
	return resp.image_valid;
}

static int init_image_modem_trusted(const u8 *metadata, size_t size)
{
	return init_image_trusted(PAS_MODEM, metadata, size);
}

static int init_image_modem_untrusted(const u8 *metadata, size_t size)
{
	struct elf32_hdr *ehdr = (struct elf32_hdr *)metadata;
	modem_start = ehdr->e_entry;
	return 0;
}

static int init_image_q6_trusted(const u8 *metadata, size_t size)
{
	return init_image_trusted(PAS_Q6, metadata, size);
}

static int init_image_q6_untrusted(const u8 *metadata, size_t size)
{
	struct elf32_hdr *ehdr = (struct elf32_hdr *)metadata;
	q6_start = ehdr->e_entry;
	return 0;
}

static int init_image_dsps_trusted(const u8 *metadata, size_t size)
{
	return init_image_trusted(PAS_DSPS, metadata, size);
}

static int init_image_dsps_untrusted(const u8 *metadata, size_t size)
{
	struct elf32_hdr *ehdr = (struct elf32_hdr *)metadata;
	dsps_start = ehdr->e_entry;
	return 0;
}

static int verify_blob(u32 phy_addr, size_t size)
{
	return 0;
}

static int auth_and_reset_trusted(int id)
{
	int ret;
	struct pas_auth_image_req request;
	struct pas_auth_image_resp resp = {0};

	request.proc = id;
	ret = scm_call(SCM_SVC_PIL, PAS_AUTH_AND_RESET_CMD, &request,
			sizeof(request), &resp, sizeof(resp));
	if (ret)
		return ret;

	return resp.reset_initiated;
}

static int reset_modem_untrusted(void)
{
	u32 reg;

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

static int reset_modem_trusted(void)
{
	return auth_and_reset_trusted(PAS_MODEM);
}

static int shutdown_trusted(int id)
{
	int ret;
	struct pas_shutdown_req request;
	struct pas_shutdown_resp resp = {0};

	request.proc = id;
	ret = scm_call(SCM_SVC_PIL, PAS_SHUTDOWN_CMD, &request, sizeof(request),
			&resp, sizeof(resp));
	if (ret)
		return ret;

	return resp.success;
}

static int shutdown_modem_untrusted(void)
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

static int shutdown_modem_trusted(void)
{
	return shutdown_trusted(PAS_MODEM);
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

static int reset_q6_untrusted(void)
{
	int ret;
	u32 reg;

	ret = local_src_enable(PLL_4);
	if (ret)
		goto err;

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

err:
	return ret;
}

static int reset_q6_trusted(void)
{
	int ret;

	ret = local_src_enable(PLL_4);
	if (ret)
		return ret;

	return auth_and_reset_trusted(PAS_Q6);
}

static int shutdown_q6_untrusted(void)
{
	u32 reg;

	reg = readl(LCC_Q6_FUNC);
	/* Halt clocks and turn off memory */
	reg &= ~(CORE_L1_MEM_CORE_EN | CORE_TCM_MEM_CORE_EN |
		CORE_TCM_MEM_PERPH_EN);
	reg |= CLAMP_IO | CORE_GFM4_CLK_EN;
	writel(reg, LCC_Q6_FUNC);
	return 0;
}

static int shutdown_q6_trusted(void)
{
	return shutdown_trusted(PAS_Q6);
}

static int reset_dsps_untrusted(void)
{
	/* Bring DSPS out of reset */
	writel(0x0, PPSS_RESET);
	return 0;
}

static int reset_dsps_trusted(void)
{
	return auth_and_reset_trusted(PAS_DSPS);
}

static int shutdown_dsps_trusted(void)
{
	return shutdown_trusted(PAS_DSPS);
}

static int shutdown_dsps_untrusted(void)
{
	writel(0x3, PPSS_RESET);
	return 0;
}

struct pil_reset_ops pil_modem_ops = {
	.init_image = init_image_modem_untrusted,
	.verify_blob = verify_blob,
	.auth_and_reset = reset_modem_untrusted,
	.shutdown = shutdown_modem_untrusted,
};

struct pil_reset_ops pil_q6_ops = {
	.init_image = init_image_q6_untrusted,
	.verify_blob = verify_blob,
	.auth_and_reset = reset_q6_untrusted,
	.shutdown = shutdown_q6_untrusted,
};

struct pil_reset_ops pil_dsps_ops = {
	.init_image = init_image_dsps_untrusted,
	.verify_blob = verify_blob,
	.auth_and_reset = reset_dsps_untrusted,
	.shutdown = shutdown_dsps_untrusted,
};

static struct pil_device peripherals[] = {
	{
		.name = "modem",
		.depends_on = "q6",
		.pdev = {
			.name = "pil_modem",
			.id = -1,
		},
		.ops = &pil_modem_ops,
	},
	{
		.name = "q6",
		.pdev = {
			.name = "pil_q6",
			.id = -1,
		},
		.ops = &pil_q6_ops,
	},
	{
		.name = "dsps",
		.pdev = {
			.name = "pil_dsps",
			.id = -1,
		},
		.ops = &pil_dsps_ops,
	},
};


#ifdef CONFIG_MSM_SECURE_PIL
#define SECURE_PIL 1
#else
#define SECURE_PIL 0
#endif

static int __init msm_peripheral_reset_init(void)
{
	unsigned i;

	msm_mms_regs_base = ioremap(MSM_MMS_REGS_BASE, SZ_256);
	if (!msm_mms_regs_base)
		goto err;

	msm_lpass_qdsp6ss_base = ioremap(MSM_LPASS_QDSP6SS_BASE, SZ_256);
	if (!msm_lpass_qdsp6ss_base)
		goto err_lpass;

	if (SECURE_PIL) {
		pil_modem_ops.init_image = init_image_modem_trusted;
		pil_modem_ops.auth_and_reset = reset_modem_trusted;
		pil_modem_ops.shutdown = shutdown_modem_trusted;

		pil_q6_ops.init_image = init_image_q6_trusted;
		pil_q6_ops.auth_and_reset = reset_q6_trusted;
		pil_q6_ops.shutdown = shutdown_q6_trusted;

		pil_dsps_ops.init_image = init_image_dsps_trusted;
		pil_dsps_ops.auth_and_reset = reset_dsps_trusted;
		pil_dsps_ops.shutdown = shutdown_dsps_trusted;
	}

	for (i = 0; i < ARRAY_SIZE(peripherals); i++)
		msm_pil_add_device(&peripherals[i]);

	return 0;

err_lpass:
	iounmap(msm_mms_regs_base);
err:
	return -ENOMEM;
}

static void __exit msm_peripheral_reset_exit(void)
{
	iounmap(msm_mms_regs_base);
	iounmap(msm_lpass_qdsp6ss_base);
}

arch_initcall(msm_peripheral_reset_init);
module_exit(msm_peripheral_reset_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Validate and bring peripherals out of reset");
