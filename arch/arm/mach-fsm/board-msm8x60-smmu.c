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
#include <linux/platform_device.h>
#include <linux/bootmem.h>

#include <linux/vcm.h>
#include <linux/vcm_alloc.h>
#include <mach/msm_iomap-8x60.h>
#include <mach/irqs-8x60.h>
#include <mach/smmu_device.h>

#define MSM_SMI_BASE           0x38000000
#define MSM_SMI_SIZE           0x04000000

#define VCM_EBI_SIZE	SZ_32M
#define VCM_EBI_ALIGN	SZ_1M

static struct resource msm_smmu_jpegd_resources[] = {
	{
		.start = MSM_SMMU_JPEGD_PHYS,
		.end   = MSM_SMMU_JPEGD_PHYS + MSM_SMMU_JPEGD_SIZE - 1,
		.name  = "physbase",
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "nonsecure_irq",
		.start = SMMU_JPEGD_CB_SC_NON_SECURE_IRQ,
		.end   = SMMU_JPEGD_CB_SC_NON_SECURE_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "secure_irq",
		.start = SMMU_JPEGD_CB_SC_SECURE_IRQ,
		.end   = SMMU_JPEGD_CB_SC_SECURE_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource msm_smmu_vpe_resources[] = {
	{
		.start = MSM_SMMU_VPE_PHYS,
		.end   = MSM_SMMU_VPE_PHYS + MSM_SMMU_VPE_SIZE - 1,
		.name  = "physbase",
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "nonsecure_irq",
		.start = SMMU_VPE_CB_SC_NON_SECURE_IRQ,
		.end   = SMMU_VPE_CB_SC_NON_SECURE_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "secure_irq",
		.start = SMMU_VPE_CB_SC_SECURE_IRQ,
		.end   = SMMU_VPE_CB_SC_SECURE_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource msm_smmu_mdp0_resources[] = {
	{
		.start = MSM_SMMU_MDP0_PHYS,
		.end   = MSM_SMMU_MDP0_PHYS + MSM_SMMU_MDP0_SIZE - 1,
		.name  = "physbase",
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "nonsecure_irq",
		.start = SMMU_MDP0_CB_SC_NON_SECURE_IRQ,
		.end   = SMMU_MDP0_CB_SC_NON_SECURE_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "secure_irq",
		.start = SMMU_MDP0_CB_SC_SECURE_IRQ,
		.end   = SMMU_MDP0_CB_SC_SECURE_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource msm_smmu_mdp1_resources[] = {
	{
		.start = MSM_SMMU_MDP1_PHYS,
		.end   = MSM_SMMU_MDP1_PHYS + MSM_SMMU_MDP1_SIZE - 1,
		.name  = "physbase",
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "nonsecure_irq",
		.start = SMMU_MDP1_CB_SC_NON_SECURE_IRQ,
		.end   = SMMU_MDP1_CB_SC_NON_SECURE_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "secure_irq",
		.start = SMMU_MDP1_CB_SC_SECURE_IRQ,
		.end   = SMMU_MDP1_CB_SC_SECURE_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource msm_smmu_rot_resources[] = {
	{
		.start = MSM_SMMU_ROT_PHYS,
		.end   = MSM_SMMU_ROT_PHYS + MSM_SMMU_ROT_SIZE - 1,
		.name  = "physbase",
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "nonsecure_irq",
		.start = SMMU_ROT_CB_SC_NON_SECURE_IRQ,
		.end   = SMMU_ROT_CB_SC_NON_SECURE_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "secure_irq",
		.start = SMMU_ROT_CB_SC_SECURE_IRQ,
		.end   = SMMU_ROT_CB_SC_SECURE_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource msm_smmu_ijpeg_resources[] = {
	{
		.start = MSM_SMMU_IJPEG_PHYS,
		.end   = MSM_SMMU_IJPEG_PHYS + MSM_SMMU_IJPEG_SIZE - 1,
		.name  = "physbase",
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "nonsecure_irq",
		.start = SMMU_IJPEG_CB_SC_NON_SECURE_IRQ,
		.end   = SMMU_IJPEG_CB_SC_NON_SECURE_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "secure_irq",
		.start = SMMU_IJPEG_CB_SC_SECURE_IRQ,
		.end   = SMMU_IJPEG_CB_SC_SECURE_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource msm_smmu_vfe_resources[] = {
	{
		.start = MSM_SMMU_VFE_PHYS,
		.end   = MSM_SMMU_VFE_PHYS + MSM_SMMU_VFE_SIZE - 1,
		.name  = "physbase",
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "nonsecure_irq",
		.start = SMMU_VFE_CB_SC_NON_SECURE_IRQ,
		.end   = SMMU_VFE_CB_SC_NON_SECURE_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "secure_irq",
		.start = SMMU_VFE_CB_SC_SECURE_IRQ,
		.end   = SMMU_VFE_CB_SC_SECURE_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource msm_smmu_vcodec_a_resources[] = {
	{
		.start = MSM_SMMU_VCODEC_A_PHYS,
		.end   = MSM_SMMU_VCODEC_A_PHYS + MSM_SMMU_VCODEC_A_SIZE - 1,
		.name  = "physbase",
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "nonsecure_irq",
		.start = SMMU_VCODEC_A_CB_SC_NON_SECURE_IRQ,
		.end   = SMMU_VCODEC_A_CB_SC_NON_SECURE_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "secure_irq",
		.start = SMMU_VCODEC_A_CB_SC_SECURE_IRQ,
		.end   = SMMU_VCODEC_A_CB_SC_SECURE_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource msm_smmu_vcodec_b_resources[] = {
	{
		.start = MSM_SMMU_VCODEC_B_PHYS,
		.end   = MSM_SMMU_VCODEC_B_PHYS + MSM_SMMU_VCODEC_B_SIZE - 1,
		.name  = "physbase",
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "nonsecure_irq",
		.start = SMMU_VCODEC_B_CB_SC_NON_SECURE_IRQ,
		.end   = SMMU_VCODEC_B_CB_SC_NON_SECURE_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "secure_irq",
		.start = SMMU_VCODEC_B_CB_SC_SECURE_IRQ,
		.end   = SMMU_VCODEC_B_CB_SC_SECURE_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource msm_smmu_gfx3d_resources[] = {
	{
		.start = MSM_SMMU_GFX3D_PHYS,
		.end   = MSM_SMMU_GFX3D_PHYS + MSM_SMMU_GFX3D_SIZE - 1,
		.name  = "physbase",
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "nonsecure_irq",
		.start = SMMU_GFX3D_CB_SC_NON_SECURE_IRQ,
		.end   = SMMU_GFX3D_CB_SC_NON_SECURE_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "secure_irq",
		.start = SMMU_GFX3D_CB_SC_SECURE_IRQ,
		.end   = SMMU_GFX3D_CB_SC_SECURE_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource msm_smmu_gfx2d0_resources[] = {
	{
		.start = MSM_SMMU_GFX2D0_PHYS,
		.end   = MSM_SMMU_GFX2D0_PHYS + MSM_SMMU_GFX2D0_SIZE - 1,
		.name  = "physbase",
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "nonsecure_irq",
		.start = SMMU_GFX2D0_CB_SC_NON_SECURE_IRQ,
		.end   = SMMU_GFX2D0_CB_SC_NON_SECURE_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "secure_irq",
		.start = SMMU_GFX2D0_CB_SC_SECURE_IRQ,
		.end   = SMMU_GFX2D0_CB_SC_SECURE_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

/* TODO GFX2D1 once it gets into irqs-8x60.h */

static void ctx_release(struct device *dev)
{
	pr_info("%s\n", __func__);
	return;
}

static void smmu_release(struct device *dev)
{
	pr_info("%s\n", __func__);
	return;
}

static void smmu_release_top(struct device *dev)
{
	pr_info("%s\n", __func__);
	return;
}

static struct platform_device msm_device_all_smmus = {
	.name = "smmu",
	.id = -1,
	.dev = {
		.release = smmu_release_top,
	},
};

static struct smmu_device jpegd_smmu = {
	.name = "jpegd",
	.clk = "smmu_jpegd_clk"
};

static struct smmu_device vpe_smmu = {
	.name = "vpe"
};

static struct smmu_device mdp0_smmu = {
	.name = "mdp0"
};

static struct smmu_device mdp1_smmu = {
	.name = "mdp1"
};

static struct smmu_device rot_smmu = {
	.name = "rot"
};

static struct smmu_device ijpeg_smmu = {
	.name = "ijpeg"
};

static struct smmu_device vfe_smmu = {
	.name = "vfe",
	.clk = "smmu_vfe_clk"
};

static struct smmu_device vcodec_a_smmu = {
	.name = "vcodec_a"
};

static struct smmu_device vcodec_b_smmu = {
	.name = "vcodec_b"
};

static struct smmu_device gfx3d_smmu = {
	.name = "gfx3d",
	.clk = "gfx3d_clk",
	.clk_rate = 27000000
};

static struct smmu_device gfx2d0_smmu = {
	.name = "gfx2d0",
	.clk = "gfx2d_clk",
	.clk_rate = 27000000
};

static struct platform_device msm_device_smmu_jpegd = {
	.name = "smmu",
	.id = 0,
	.dev = {
		.release = smmu_release,
		.parent = &msm_device_all_smmus.dev,
	},
	.num_resources = ARRAY_SIZE(msm_smmu_jpegd_resources),
	.resource = msm_smmu_jpegd_resources,
};

static struct platform_device msm_device_smmu_vpe = {
	.name = "smmu",
	.id = 1,
	.dev = {
		.release = smmu_release,
		.parent = &msm_device_all_smmus.dev,
	},
	.num_resources = ARRAY_SIZE(msm_smmu_vpe_resources),
	.resource = msm_smmu_vpe_resources,
};

static struct platform_device msm_device_smmu_mdp0 = {
	.name = "smmu",
	.id = 2,
	.dev = {
		.release = smmu_release,
		.parent = &msm_device_all_smmus.dev,
	},
	.num_resources = ARRAY_SIZE(msm_smmu_mdp0_resources),
	.resource = msm_smmu_mdp0_resources,
};

static struct platform_device msm_device_smmu_mdp1 = {
	.name = "smmu",
	.id = 3,
	.dev = {
		.release = smmu_release,
		.parent = &msm_device_all_smmus.dev,
	},
	.num_resources = ARRAY_SIZE(msm_smmu_mdp1_resources),
	.resource = msm_smmu_mdp1_resources,
};

static struct platform_device msm_device_smmu_rot = {
	.name = "smmu",
	.id = 4,
	.dev = {
		.release = smmu_release,
		.parent = &msm_device_all_smmus.dev,
	},
	.num_resources = ARRAY_SIZE(msm_smmu_rot_resources),
	.resource = msm_smmu_rot_resources,
};

static struct platform_device msm_device_smmu_ijpeg = {
	.name = "smmu",
	.id = 5,
	.dev = {
		.release = smmu_release,
		.parent = &msm_device_all_smmus.dev,
	},
	.num_resources = ARRAY_SIZE(msm_smmu_ijpeg_resources),
	.resource = msm_smmu_ijpeg_resources,
};

static struct platform_device msm_device_smmu_vfe = {
	.name = "smmu",
	.id = 6,
	.dev = {
		.release = smmu_release,
		.parent = &msm_device_all_smmus.dev,
	},
	.num_resources = ARRAY_SIZE(msm_smmu_vfe_resources),
	.resource = msm_smmu_vfe_resources,
};

static struct platform_device msm_device_smmu_vcodec_a = {
	.name = "smmu",
	.id = 7,
	.dev = {
		.release = smmu_release,
		.parent = &msm_device_all_smmus.dev,
	},
	.num_resources = ARRAY_SIZE(msm_smmu_vcodec_a_resources),
	.resource = msm_smmu_vcodec_a_resources,
};

static struct platform_device msm_device_smmu_vcodec_b = {
	.name = "smmu",
	.id = 8,
	.dev = {
		.release = smmu_release,
		.parent = &msm_device_all_smmus.dev,
	},
	.num_resources = ARRAY_SIZE(msm_smmu_vcodec_b_resources),
	.resource = msm_smmu_vcodec_b_resources,
};


static struct platform_device msm_device_smmu_gfx3d = {
	.name = "smmu",
	.id = 9,
	.dev = {
		.release = smmu_release,
		.parent = &msm_device_all_smmus.dev,
	},
	.num_resources = ARRAY_SIZE(msm_smmu_gfx3d_resources),
	.resource = msm_smmu_gfx3d_resources,
};



static struct platform_device msm_device_smmu_gfx2d0 = {
	.name = "smmu",
	.id = 10,
	.dev = {
		.release = smmu_release,
		.parent = &msm_device_all_smmus.dev,
	},
	.num_resources = ARRAY_SIZE(msm_smmu_gfx2d0_resources),
	.resource = msm_smmu_gfx2d0_resources,
};


static struct smmu_ctx jpegd_src_ctx = {
	.name = "jpegd_src",
	.num = 0,
	.mids = {0, -1}
};

static struct smmu_ctx jpegd_dst_ctx = {
	.name = "jpegd_dst",
	.num = 1,
	.mids = {1, -1}
};

static struct smmu_ctx vpe_src_ctx = {
	.name = "vpe_src",
	.num = 0,
	.mids = {0, -1}
};

static struct smmu_ctx vpe_dst_ctx = {
	.name = "vpe_dst",
	.num = 1,
	.mids = {1, -1}
};

static struct smmu_ctx mdp_vg1_ctx = {
	.name = "mdp_vg1",
	.num = 0,
	.mids = {0, 2, -1}
};

static struct smmu_ctx mdp_rgb1_ctx = {
	.name = "mdp_rgb1",
	.num = 1,
	.mids = {1, 3, 4, 5, 6, 7, 8, 9, 10, -1}
};

static struct smmu_ctx mdp_vg2_ctx = {
	.name = "mdp_vg2",
	.num = 0,
	.mids = {0, 2, -1}
};

static struct smmu_ctx mdp_rgb2_ctx = {
	.name = "mdp_rgb2",
	.num = 1,
	.mids = {1, 3, 4, 5, 6, 7, 8, 9, 10, -1}
};

static struct smmu_ctx rot_src_ctx = {
	.name = "rot_src",
	.num = 0,
	.mids = {0, -1}
};

static struct smmu_ctx rot_dst_ctx = {
	.name = "rot_dst",
	.num = 1,
	.mids = {1, -1}
};

static struct smmu_ctx ijpeg_src_ctx = {
	.name = "ijpeg_src",
	.num = 0,
	.mids = {0, -1}
};

static struct smmu_ctx ijpeg_dst_ctx = {
	.name = "ijpeg_dst",
	.num = 1,
	.mids = {1, -1}
};

static struct smmu_ctx vfe_imgwr_ctx = {
	.name = "vfe_imgwr",
	.num = 0,
	.mids = {2, 3, 4, 5, 6, 7, 8, -1}
};

static struct smmu_ctx vfe_misc_ctx = {
	.name = "vfe_misc",
	.num = 1,
	.mids = {0, 1, 9, -1}
};

static struct smmu_ctx vcodec_a_stream_ctx = {
	.name = "vcodec_a_stream",
	.num = 0,
	.mids = {2, 5, -1}
};

static struct smmu_ctx vcodec_a_mm1_ctx = {
	.name = "vcodec_a_mm1",
	.num = 1,
	.mids = {0, 1, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, -1}
};

static struct smmu_ctx vcodec_b_mm2_ctx = {
	.name = "vcodec_b_mm2",
	.num = 0,
	.mids = {0, 1, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, -1}
};

static struct smmu_ctx gfx3d_rbpa_ctx = {
	.name = "gfx3d_rbpa",
	.num = 0,
	.mids = {-1}
};

static struct smmu_ctx gfx3d_cpvgttc_ctx = {
	.name = "gfx3d_cpvgttc",
	.num = 1,
	.mids = {0, 1, 2, 3, 4, 5, 6, 7, -1}
};

static struct smmu_ctx gfx3d_smmu_ctx = {
	.name = "gfx3d_smmu",
	.num = 2,
	.mids = {8, 9, 10, 11, 12, -1}
};

static struct smmu_ctx gfx2d0_pixv1_ctx = {
	.name = "gfx2d0_pixv1_smmu",
	.num = 0,
	.mids = {0, 3, 4, -1}
};

static struct smmu_ctx gfx2d0_texv3_ctx = {
	.name = "gfx2d0_texv3_smmu",
	.num = 1,
	.mids = {1, 6, 7, -1}
};

static struct platform_device msm_device_jpegd_src_ctx = {
	.name = "ctx",
	.id = 0,
	.dev = {
		.release = ctx_release,
		.parent = &msm_device_smmu_jpegd.dev,
	},
};

static struct platform_device msm_device_jpegd_dst_ctx = {
	.name = "ctx",
	.id = 1,
	.dev = {
		.release = ctx_release,
		.parent = &msm_device_smmu_jpegd.dev,
	},
};

static struct platform_device msm_device_vpe_src_ctx = {
	.name = "ctx",
	.id = 2,
	.dev = {
		.release = ctx_release,
		.parent = &msm_device_smmu_vpe.dev,
	},
};

static struct platform_device msm_device_vpe_dst_ctx = {
	.name = "ctx",
	.id = 3,
	.dev = {
		.release = ctx_release,
		.parent = &msm_device_smmu_vpe.dev,
	},
};

static struct platform_device msm_device_mdp_vg1_ctx = {
	.name = "ctx",
	.id = 4,
	.dev = {
		.release = ctx_release,
		.parent = &msm_device_smmu_mdp0.dev,
	},
};

static struct platform_device msm_device_mdp_rgb1_ctx = {
	.name = "ctx",
	.id = 5,
	.dev = {
		.release = ctx_release,
		.parent = &msm_device_smmu_mdp0.dev,
	},
};

static struct platform_device msm_device_mdp_vg2_ctx = {
	.name = "ctx",
	.id = 6,
	.dev = {
		.release = ctx_release,
		.parent = &msm_device_smmu_mdp1.dev,
	},
};

static struct platform_device msm_device_mdp_rgb2_ctx = {
	.name = "ctx",
	.id = 7,
	.dev = {
		.release = ctx_release,
		.parent = &msm_device_smmu_mdp1.dev,
	},
};

static struct platform_device msm_device_rot_src_ctx = {
	.name = "ctx",
	.id = 8,
	.dev = {
		.release = ctx_release,
		.parent = &msm_device_smmu_rot.dev,
	},
};

static struct platform_device msm_device_rot_dst_ctx = {
	.name = "ctx",
	.id = 9,
	.dev = {
		.release = ctx_release,
		.parent = &msm_device_smmu_rot.dev,
	},
};

static struct platform_device msm_device_ijpeg_src_ctx = {
	.name = "ctx",
	.id = 10,
	.dev = {
		.release = ctx_release,
		.parent = &msm_device_smmu_ijpeg.dev,
	},
};

static struct platform_device msm_device_ijpeg_dst_ctx = {
	.name = "ctx",
	.id = 11,
	.dev = {
		.release = ctx_release,
		.parent = &msm_device_smmu_ijpeg.dev,
	},
};

static struct platform_device msm_device_vfe_imgwr_ctx = {
	.name = "ctx",
	.id = 12,
	.dev = {
		.release = ctx_release,
		.parent = &msm_device_smmu_vfe.dev,
	},
};

static struct platform_device msm_device_vfe_misc_ctx = {
	.name = "ctx",
	.id = 13,
	.dev = {
		.release = ctx_release,
		.parent = &msm_device_smmu_vfe.dev,
	},
};

static struct platform_device msm_device_vcodec_a_stream_ctx = {
	.name = "ctx",
	.id = 14,
	.dev = {
		.release = ctx_release,
		.parent = &msm_device_smmu_vcodec_a.dev,
	},
};

static struct platform_device msm_device_vcodec_a_mm1_ctx = {
	.name = "ctx",
	.id = 15,
	.dev = {
		.release = ctx_release,
		.parent = &msm_device_smmu_vcodec_a.dev,
	},
};

static struct platform_device msm_device_vcodec_b_mm2_ctx = {
	.name = "ctx",
	.id = 16,
	.dev = {
		.release = ctx_release,
		.parent = &msm_device_smmu_vcodec_b.dev,
	},
};

static struct platform_device msm_device_gfx3d_rbpa_ctx = {
	.name = "ctx",
	.id = 17,
	.dev = {
		.release = ctx_release,
		.parent = &msm_device_smmu_gfx3d.dev,
	},
};

static struct platform_device msm_device_gfx3d_cpvgttc_ctx = {
	.name = "ctx",
	.id = 18,
	.dev = {
		.release = ctx_release,
		.parent = &msm_device_smmu_gfx3d.dev,
	},
};

static struct platform_device msm_device_gfx3d_smmu_ctx = {
	.name = "ctx",
	.id = 19,
	.dev = {
		.release = ctx_release,
		.parent = &msm_device_smmu_gfx3d.dev,
	},
};

static struct platform_device msm_device_gfx2d0_pixv1_ctx = {
	.name = "ctx",
	.id = 20,
	.dev = {
		.release = ctx_release,
		.parent = &msm_device_smmu_gfx2d0.dev,
	},
};

static struct platform_device msm_device_gfx2d0_texv3_ctx = {
	.name = "ctx",
	.id = 21,
	.dev = {
		.release = ctx_release,
		.parent = &msm_device_smmu_gfx2d0.dev,
	},
};

static struct platform_device *smmu_devices[] = {
	&msm_device_smmu_jpegd,
	&msm_device_smmu_vpe,
	&msm_device_smmu_mdp0,
	&msm_device_smmu_mdp1,
	&msm_device_smmu_rot,
	&msm_device_smmu_ijpeg,
	&msm_device_smmu_vfe,
	&msm_device_smmu_vcodec_a,
	&msm_device_smmu_vcodec_b,
	&msm_device_smmu_gfx3d,
	&msm_device_smmu_gfx2d0,
};

static struct smmu_device *smmu_device_data[] = {
	&jpegd_smmu,
	&vpe_smmu,
	&mdp0_smmu,
	&mdp1_smmu,
	&rot_smmu,
	&ijpeg_smmu,
	&vfe_smmu,
	&vcodec_a_smmu,
	&vcodec_b_smmu,
	&gfx3d_smmu,
	&gfx2d0_smmu,
};

static struct platform_device *smmu_ctxs[] = {
	&msm_device_jpegd_src_ctx,
	&msm_device_jpegd_dst_ctx,
	&msm_device_vpe_src_ctx,
	&msm_device_vpe_dst_ctx,
	&msm_device_mdp_vg1_ctx,
	&msm_device_mdp_rgb1_ctx,
	&msm_device_mdp_vg2_ctx,
	&msm_device_mdp_rgb2_ctx,
	&msm_device_rot_src_ctx,
	&msm_device_rot_dst_ctx,
	&msm_device_ijpeg_src_ctx,
	&msm_device_ijpeg_dst_ctx,
	&msm_device_vfe_imgwr_ctx,
	&msm_device_vfe_misc_ctx,
	&msm_device_vcodec_a_stream_ctx,
	&msm_device_vcodec_a_mm1_ctx,
	&msm_device_vcodec_b_mm2_ctx,
	&msm_device_gfx3d_rbpa_ctx,
	&msm_device_gfx3d_cpvgttc_ctx,
	&msm_device_gfx3d_smmu_ctx,
	&msm_device_gfx2d0_pixv1_ctx,
	&msm_device_gfx2d0_texv3_ctx,
};

static struct smmu_ctx *smmu_ctxs_data[] = {
	&jpegd_src_ctx,
	&jpegd_dst_ctx,
	&vpe_src_ctx,
	&vpe_dst_ctx,
	&mdp_vg1_ctx,
	&mdp_rgb1_ctx,
	&mdp_vg2_ctx,
	&mdp_rgb2_ctx,
	&rot_src_ctx,
	&rot_dst_ctx,
	&ijpeg_src_ctx,
	&ijpeg_dst_ctx,
	&vfe_imgwr_ctx,
	&vfe_misc_ctx,
	&vcodec_a_stream_ctx,
	&vcodec_a_mm1_ctx,
	&vcodec_b_mm2_ctx,
	&gfx3d_rbpa_ctx,
	&gfx3d_cpvgttc_ctx,
	&gfx3d_smmu_ctx,
	&gfx2d0_pixv1_ctx,
	&gfx2d0_texv3_ctx,
};

#define SMI  0
#define EBI  1

#define HLF 2
#define QTR 4
#define EIGTH 8
struct physmem_region memory[] = {
	{
		.addr = MSM_SMI_BASE,	/* Base address */
		.size = SZ_32M,		/* Memory size */

		/* 1/2 to 16MB, 1/4th to 1MB, 1/8th to 64KB and 4KB */
		.chunk_fraction = {HLF, QTR, EIGTH, EIGTH}
	},
	{
		.addr = 0,		/* To be dynamically allocated */
		.size = VCM_EBI_SIZE,
		.chunk_fraction = {HLF, QTR, EIGTH, EIGTH}
	},
};

struct vcm_memtype_map mt_map[] = {	/* Sources of 1MB, 64K, and 4K chunks */
	{
		.pool_id = {SMI, SMI, SMI, SMI}	/* MEMTYPE_0 */
	},
	{
		.pool_id = {SMI, SMI, SMI, EBI}	/* MEMTYPE_1 */
	},
	{
		.pool_id = {SMI, EBI, EBI, EBI}	/* MEMTYPE_2 */
	}
};

static int msm8x60_vcm_init(void)
{
	int ret;
	void *vcm_ebi_base;

	vcm_ebi_base = __alloc_bootmem(VCM_EBI_SIZE, VCM_EBI_ALIGN, 0);

	if (!vcm_ebi_base) {
		pr_err("Could not allocate VCM-managed physical memory\n");
		goto fail;
	}
	memory[1].addr = __pa(vcm_ebi_base);

	ret = vcm_sys_init(memory, ARRAY_SIZE(memory),
			   mt_map, ARRAY_SIZE(mt_map),
			   (void *)MSM_SMI_BASE + MSM_SMI_SIZE - SZ_8M, SZ_8M);

	if (ret != 0) {
		pr_err("vcm_sys_init() ret %i\n", ret);
		goto fail;
	}

	return 0;
fail:
	return -1;
};

/* Useful for testing, and if VCM is ever unloaded */
static void msm8x60_vcm_exit(void)
{
	int ret;

	ret = vcm_sys_destroy();
	if (ret != 0) {
		pr_err("vcm_sys_destroy() ret %i\n", ret);
		goto fail;
	}
fail:
	return;
}

static int msm8x60_smmu_init(void)
{
	int ret, i;

	ret = platform_device_register(&msm_device_all_smmus);
	if (ret != 0) {
		pr_err("Failed to register parent device!\n");
		goto failure;
	}

	for (i = 0; i < ARRAY_SIZE(smmu_devices); i++) {
		ret = platform_device_add_data(smmu_devices[i],
					       smmu_device_data[i],
					       sizeof(struct smmu_device));
		if (ret != 0) {
			pr_err("platform_device_add_data smmu failed, "
			       "i = %d\n", i);
			goto failure_unwind;
		}

		ret = platform_device_register(smmu_devices[i]);

		if (ret != 0) {
			pr_err("platform_device_register smmu failed, "
			       "i = %d\n", i);
			goto failure_unwind;
		}
	}

	for (i = 0; i < ARRAY_SIZE(smmu_ctxs); i++) {
		ret = platform_device_add_data(smmu_ctxs[i],
					       smmu_ctxs_data[i],
					       sizeof(struct smmu_ctx));
		if (ret != 0) {
			pr_err("platform_device_add_data smmu failed, "
			       "i = %d\n", i);
			goto failure_unwind2;
		}

		ret = platform_device_register(smmu_ctxs[i]);
		if (ret != 0) {
			pr_err("platform_device_register ctx failed, "
			       "i = %d\n", i);
			goto failure_unwind2;
		}
	}

	if (msm8x60_vcm_init() != 0) {
		pr_err("Could not initialize VCM\n");
		goto failure;
	}

	return 0;

failure_unwind2:
	while (--i >= 0)
		platform_device_unregister(smmu_ctxs[i]);
failure_unwind:
	while (--i >= 0)
		platform_device_unregister(smmu_devices[i]);

	platform_device_unregister(&msm_device_all_smmus);
failure:
	return ret;
}

static void msm8x60_smmu_exit(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(smmu_ctxs); i++)
		platform_device_unregister(smmu_ctxs[i]);

	for (i = 0; i < ARRAY_SIZE(smmu_devices); ++i)
		platform_device_unregister(smmu_devices[i]);

	platform_device_unregister(&msm_device_all_smmus);
	msm8x60_vcm_exit();
	pr_info("%s\n", __func__);
}

subsys_initcall(msm8x60_smmu_init);
module_exit(msm8x60_smmu_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Zach Pfeffer <zpfeffer@codeaurora.org>");
