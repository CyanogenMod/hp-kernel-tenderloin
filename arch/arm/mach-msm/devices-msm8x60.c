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
 *
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <mach/irqs.h>
#include <mach/dma.h>
#include <asm/mach/mmc.h>
#include <asm/clkdev.h>
#include <linux/msm_kgsl.h>
#include <linux/msm_rotator.h>
#include <mach/msm_hsusb.h>
#include "footswitch.h"
#include "clock.h"
#include "clock-8x60.h"
#include "clock-rpm.h"
#include "clock-voter.h"
#include "devices.h"
#include "devices-msm8x60.h"
#include "devices-msm8x60-iommu.h"
#include <linux/dma-mapping.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <asm/hardware/gic.h>
#include <asm/mach-types.h>
#include <asm/clkdev.h>
#include <mach/usbdiag.h>
#include <mach/msm_bus.h>
#include <mach/msm_bus_board.h>
#include <mach/socinfo.h>
#ifdef CONFIG_MSM_DSPS
#include <mach/msm_dsps.h>
#endif
#include <linux/gpio.h>
#include <mach/mdm.h>
#include "rpm_stats.h"

/* Address of GSBI blocks */
#define MSM_GSBI1_PHYS	0x16000000
#define MSM_GSBI2_PHYS	0x16100000
#define MSM_GSBI3_PHYS	0x16200000
#define MSM_GSBI4_PHYS	0x16300000
#define MSM_GSBI5_PHYS	0x16400000
#define MSM_GSBI6_PHYS	0x16500000
#define MSM_GSBI7_PHYS	0x16600000
#define MSM_GSBI8_PHYS	0x19800000
#define MSM_GSBI9_PHYS	0x19900000
#define MSM_GSBI10_PHYS	0x19A00000
#define MSM_GSBI11_PHYS	0x19B00000
#define MSM_GSBI12_PHYS	0x19C00000

/* GSBI QUPe devices */
#define MSM_GSBI1_QUP_PHYS	0x16080000
#define MSM_GSBI2_QUP_PHYS	0x16180000
#define MSM_GSBI3_QUP_PHYS	0x16280000
#define MSM_GSBI4_QUP_PHYS	0x16380000
#define MSM_GSBI5_QUP_PHYS	0x16480000
#define MSM_GSBI6_QUP_PHYS	0x16580000
#define MSM_GSBI7_QUP_PHYS	0x16680000
#define MSM_GSBI8_QUP_PHYS	0x19880000
#define MSM_GSBI9_QUP_PHYS	0x19980000
#define MSM_GSBI10_QUP_PHYS	0x19A80000
#define MSM_GSBI11_QUP_PHYS	0x19B80000
#define MSM_GSBI12_QUP_PHYS	0x19C80000

/* GSBI UART devices */
#define MSM_UART1DM_PHYS    (MSM_GSBI6_PHYS + 0x40000)
#define INT_UART1DM_IRQ     GSBI6_UARTDM_IRQ
#define INT_UART2DM_IRQ     GSBI12_UARTDM_IRQ
#define MSM_UART2DM_PHYS    0x19C40000
#define MSM_UART3DM_PHYS    (MSM_GSBI3_PHYS + 0x40000)
#define INT_UART3DM_IRQ     GSBI3_UARTDM_IRQ
#define TCSR_BASE_PHYS      0x16b00000
#define MSM_GSBI10_UART_DM_PHYS	(MSM_GSBI10_PHYS + 0x40000)
#define MSM_GSBI11_UARTDM_PHYS  (MSM_GSBI11_PHYS + 0x40000)

/* PRNG device */
#define MSM_PRNG_PHYS		0x16C00000
#define MSM_UART9DM_PHYS    (MSM_GSBI9_PHYS + 0x40000)
#define INT_UART9DM_IRQ     GSBI9_UARTDM_IRQ

static void charm_ap2mdm_kpdpwr_on(void)
{
	gpio_request(132, "AP2MDM_KPDPWR_N");
	if (machine_is_msm8x60_charm_surf())
		gpio_direction_output(132, 0);
	else
		gpio_direction_output(132, 1);
}

static void charm_ap2mdm_kpdpwr_off(void)
{
	if (machine_is_msm8x60_charm_surf())
		gpio_direction_output(132, 1);
	else
		gpio_direction_output(132, 0);

}

static struct resource charm_resources[] = {
	/* MDM2AP_ERRFATAL */
	{
		.start	= MSM_GPIO_TO_INT(133),
		.end	= MSM_GPIO_TO_INT(133),
		.flags = IORESOURCE_IRQ,
	},
	/* MDM2AP_STATUS */
	{
		.start	= MSM_GPIO_TO_INT(134),
		.end	= MSM_GPIO_TO_INT(134),
		.flags = IORESOURCE_IRQ,
	}
};

static struct charm_platform_data mdm_platform_data = {
	.charm_modem_on		= charm_ap2mdm_kpdpwr_on,
	.charm_modem_off	= charm_ap2mdm_kpdpwr_off,
};

struct platform_device msm_charm_modem = {
	.name		= "charm_modem",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(charm_resources),
	.resource	= charm_resources,
	.dev		= {
		.platform_data = &mdm_platform_data,
	},
};

#ifdef CONFIG_MSM_DSPS
#define GSBI12_DEV (&msm_dsps_device.dev)
#else
#define GSBI12_DEV (&msm_gsbi12_qup_i2c_device.dev)
#endif

#ifdef CONFIG_WEBCAM_MT9M113
#define WEBCAM_DEV  "msm_camera_mt9m113.0"
#else
#ifdef CONFIG_WEBCAM_MT9M114
#define WEBCAM_DEV  "msm_camera_mt9m114.0"
#else
#define WEBCAM_DEV NULL
#endif
#endif

void __iomem *gic_cpu_base_addr;

void __init msm8x60_init_irq(void)
{
	unsigned int i;

	gic_dist_init(0, MSM_QGIC_DIST_BASE, GIC_PPI_START);
	gic_cpu_base_addr = (void *)MSM_QGIC_CPU_BASE;
	gic_cpu_init(0, MSM_QGIC_CPU_BASE);

	/* Edge trigger PPIs except AVS_SVICINT and AVS_SVICINTSWDONE */
	writel(0xFFFFD7FF, MSM_QGIC_DIST_BASE + GIC_DIST_CONFIG + 4);

	/* QGIC does not adhere to GIC spec by enabling STIs by default.
	 * Enable/clear is supposed to be RO for STIs, but is RW on QGIC.
	 */
	if (!machine_is_msm8x60_sim())
		writel(0x0000FFFF, MSM_QGIC_DIST_BASE + GIC_DIST_ENABLE_SET);

	/* FIXME: Not installing AVS_SVICINT and AVS_SVICINTSWDONE yet
	 * as they are configured as level, which does not play nice with
	 * handle_percpu_irq.
	 */
	for (i = GIC_PPI_START; i < GIC_SPI_START; i++) {
		if (i != AVS_SVICINT && i != AVS_SVICINTSWDONE)
			set_irq_handler(i, handle_percpu_irq);
	}
}

static struct resource msm_uart1_dm_resources[] = {
	{
		.start = MSM_UART1DM_PHYS,
		.end   = MSM_UART1DM_PHYS + PAGE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_UART1DM_IRQ,
		.end   = INT_UART1DM_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		/* GSBI6 is UARTDM1 */
		.start = MSM_GSBI6_PHYS,
		.end   = MSM_GSBI6_PHYS + 4 - 1,
		.name  = "gsbi_resource",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = TCSR_BASE_PHYS,
		.end   = TCSR_BASE_PHYS + 0x80 - 1,
		.name  = "tcsr_resource",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = DMOV_HSUART1_TX_CHAN,
		.end   = DMOV_HSUART1_RX_CHAN,
		.name  = "uartdm_channels",
		.flags = IORESOURCE_DMA,
	},
	{
		.start = DMOV_HSUART1_TX_CRCI,
		.end   = DMOV_HSUART1_RX_CRCI,
		.name  = "uartdm_crci",
		.flags = IORESOURCE_DMA,
	},
};

static u64 msm_uart_dm1_dma_mask = DMA_BIT_MASK(32);

struct platform_device msm_device_uart_dm1 = {
	.name = "msm_uartdm",
	.id = 0,
	.num_resources = ARRAY_SIZE(msm_uart1_dm_resources),
	.resource = msm_uart1_dm_resources,
	.dev            = {
		.dma_mask = &msm_uart_dm1_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

static struct resource msm_uart_dm2_resources[] = {
	{
		.start = MSM_GSBI10_UART_DM_PHYS,
		.end   = MSM_GSBI10_UART_DM_PHYS + PAGE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = GSBI10_UARTDM_IRQ,
		.end   = GSBI10_UARTDM_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = MSM_GSBI10_PHYS,
		.end   = MSM_GSBI10_PHYS + 4 - 1,
		.name  = "gsbi_resource",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = TCSR_BASE_PHYS,
		.end   = TCSR_BASE_PHYS + 0x80 - 1,
		.name  = "tcsr_resource",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = DMOV_HSUART2_TX_CHAN,
		.end   = DMOV_HSUART2_RX_CHAN,
		.name  = "uartdm_channels",
		.flags = IORESOURCE_DMA,
	},
	{
		.start = DMOV_HSUART2_TX_CRCI,
		.end   = DMOV_HSUART2_RX_CRCI,
		.name  = "uartdm_crci",
		.flags = IORESOURCE_DMA,
	},
};

static u64 msm_uart_dm2_dma_mask = DMA_BIT_MASK(32);

struct platform_device msm_device_uart_dm2 = {
	.name = "msm_uartdm",
	.id = 1,
	.num_resources = ARRAY_SIZE(msm_uart_dm2_resources),
	.resource = msm_uart_dm2_resources,
	.dev = {
		.dma_mask = &msm_uart_dm2_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

static struct resource msm_uart3_dm_resources[] = {
	{
		.start = MSM_UART3DM_PHYS,
		.end   = MSM_UART3DM_PHYS + PAGE_SIZE - 1,
		.name  = "uartdm_resource",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_UART3DM_IRQ,
		.end   = INT_UART3DM_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = MSM_GSBI3_PHYS,
		.end   = MSM_GSBI3_PHYS + PAGE_SIZE - 1,
		.name  = "gsbi_resource",
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device msm_device_uart_dm3 = {
	.name = "msm_serial_hsl",
	.id = 2,
	.num_resources = ARRAY_SIZE(msm_uart3_dm_resources),
	.resource = msm_uart3_dm_resources,
};

static struct resource msm_uart4_dm_resources[] = {
	{
		.start = MSM_GSBI11_UARTDM_PHYS,
		.end   = MSM_GSBI11_UARTDM_PHYS + PAGE_SIZE - 1,
		.name  = "uartdm_resource",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = GSBI11_UARTDM_IRQ,
		.end   = GSBI11_UARTDM_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		/* GSBI11 is UARTDM4 */
		.start = MSM_GSBI11_PHYS,
		.end   = MSM_GSBI11_PHYS + PAGE_SIZE - 1,
		.name  = "gsbi_resource",
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device msm_device_uart_dm4 = {
	.name = "msm_serial_hsl",
	.id = 3,
	.num_resources = ARRAY_SIZE(msm_uart4_dm_resources),
	.resource = msm_uart4_dm_resources,
};

static struct resource msm_uart12_dm_resources[] = {
	{
		.start = MSM_UART2DM_PHYS,
		.end   = MSM_UART2DM_PHYS + PAGE_SIZE - 1,
		.name  = "uartdm_resource",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_UART2DM_IRQ,
		.end   = INT_UART2DM_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		/* GSBI 12 is UARTDM2 */
		.start = MSM_GSBI12_PHYS,
		.end   = MSM_GSBI12_PHYS + PAGE_SIZE - 1,
		.name  = "gsbi_resource",
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device msm_device_uart_dm12 = {
	.name = "msm_serial_hsl",
	.id = 0,
	.num_resources = ARRAY_SIZE(msm_uart12_dm_resources),
	.resource = msm_uart12_dm_resources,
};

#ifdef CONFIG_MSM_GSBI9_UART
static struct resource msm_uart_gsbi9_resources[] = {
       {
		.start	= MSM_UART9DM_PHYS,
		.end	= MSM_UART9DM_PHYS + PAGE_SIZE - 1,
		.name	= "uartdm_resource",
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_UART9DM_IRQ,
		.end	= INT_UART9DM_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		/* GSBI 9 is UART_GSBI9 */
		.start	= MSM_GSBI9_PHYS,
		.end	= MSM_GSBI9_PHYS + PAGE_SIZE - 1,
		.name	= "gsbi_resource",
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device msm_device_uart_gsbi9 = {
	.name	= "msm_serial_hsl",
	.id	= 1,
	.num_resources	= ARRAY_SIZE(msm_uart_gsbi9_resources),
	.resource	= msm_uart_gsbi9_resources,
};
#endif

static struct resource gsbi3_qup_i2c_resources[] = {
	{
		.name	= "qup_phys_addr",
		.start	= MSM_GSBI3_QUP_PHYS,
		.end	= MSM_GSBI3_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI3_PHYS,
		.end	= MSM_GSBI3_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "qup_err_intr",
		.start	= GSBI3_QUP_IRQ,
		.end	= GSBI3_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource gsbi4_qup_i2c_resources[] = {
	{
		.name	= "qup_phys_addr",
		.start	= MSM_GSBI4_QUP_PHYS,
		.end	= MSM_GSBI4_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI4_PHYS,
		.end	= MSM_GSBI4_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "qup_err_intr",
		.start	= GSBI4_QUP_IRQ,
		.end	= GSBI4_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

//add gsbi5 for i2c
static struct resource gsbi5_qup_i2c_resources[] = {
	{
		.name	= "qup_phys_addr",
		.start	= MSM_GSBI5_QUP_PHYS,
		.end	= MSM_GSBI5_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI5_PHYS,
		.end	= MSM_GSBI5_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "qup_err_intr",
		.start	= GSBI5_QUP_IRQ,
		.end	= GSBI5_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource gsbi7_qup_i2c_resources[] = {
	{
		.name	= "qup_phys_addr",
		.start	= MSM_GSBI7_QUP_PHYS,
		.end	= MSM_GSBI7_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI7_PHYS,
		.end	= MSM_GSBI7_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "qup_err_intr",
		.start	= GSBI7_QUP_IRQ,
		.end	= GSBI7_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource gsbi8_qup_i2c_resources[] = {
	{
		.name	= "qup_phys_addr",
		.start	= MSM_GSBI8_QUP_PHYS,
		.end	= MSM_GSBI8_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI8_PHYS,
		.end	= MSM_GSBI8_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "qup_err_intr",
		.start	= GSBI8_QUP_IRQ,
		.end	= GSBI8_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource gsbi9_qup_i2c_resources[] = {
	{
		.name	= "qup_phys_addr",
		.start	= MSM_GSBI9_QUP_PHYS,
		.end	= MSM_GSBI9_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI9_PHYS,
		.end	= MSM_GSBI9_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "qup_err_intr",
		.start	= GSBI9_QUP_IRQ,
		.end	= GSBI9_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

// config gsbi10 as i2c function
static struct resource gsbi10_qup_i2c_resources[] = {
    {
        .name   = "qup_phys_addr",
        .start  = MSM_GSBI10_QUP_PHYS,
        .end    = MSM_GSBI10_QUP_PHYS + SZ_4K - 1,
        .flags  = IORESOURCE_MEM,
    },
    {
        .name   = "gsbi_qup_i2c_addr",
        .start  = MSM_GSBI10_PHYS,
        .end    = MSM_GSBI10_PHYS + 4 - 1,
        .flags  = IORESOURCE_MEM,
    },
    {
        .name   = "qup_err_intr",
        .start  = GSBI10_QUP_IRQ,
        .end    = GSBI10_QUP_IRQ,
        .flags  = IORESOURCE_IRQ,
    },
};

static struct resource gsbi12_qup_i2c_resources[] = {
	{
		.name	= "qup_phys_addr",
		.start	= MSM_GSBI12_QUP_PHYS,
		.end	= MSM_GSBI12_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI12_PHYS,
		.end	= MSM_GSBI12_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "qup_err_intr",
		.start	= GSBI12_QUP_IRQ,
		.end	= GSBI12_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource kgsl_resources[] = {
	{
		.name = "kgsl_reg_memory",
		.start = 0x04300000, /* GFX3D address */
		.end = 0x0431ffff,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "kgsl_yamato_irq",
		.start = GFX3D_IRQ,
		.end = GFX3D_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "kgsl_2d0_reg_memory",
		.start = 0x04100000, /* Z180 base address */
		.end = 0x04100FFF,
		.flags = IORESOURCE_MEM,
	},
	{
		.name  = "kgsl_2d0_irq",
		.start = GFX2D0_IRQ,
		.end = GFX2D0_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "kgsl_2d1_reg_memory",
		.start = 0x04200000, /* Z180 device 1 base address */
		.end =   0x04200FFF,
		.flags = IORESOURCE_MEM,
	},
	{
		.name  = "kgsl_2d1_irq",
		.start = GFX2D1_IRQ,
		.end = GFX2D1_IRQ,
		.flags = IORESOURCE_IRQ,
	},

};

#ifdef CONFIG_MSM_BUS_SCALING
static struct msm_bus_vectors grp3d_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_3D,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors grp3d_max_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_3D,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 2008000000U,
		.ib = 2008000000U,
	},
};

static struct msm_bus_paths grp3d_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(grp3d_init_vectors),
		grp3d_init_vectors,
	},
	{
		ARRAY_SIZE(grp3d_max_vectors),
		grp3d_max_vectors,
	},
};

static struct msm_bus_scale_pdata grp3d_bus_scale_pdata = {
	grp3d_bus_scale_usecases,
	ARRAY_SIZE(grp3d_bus_scale_usecases),
	.name = "grp3d",
};

static struct msm_bus_vectors grp2d0_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_2D_CORE0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors grp2d0_max_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_2D_CORE0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 2096000000U,
	},
};

static struct msm_bus_paths grp2d0_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(grp2d0_init_vectors),
		grp2d0_init_vectors,
	},
	{
		ARRAY_SIZE(grp2d0_max_vectors),
		grp2d0_max_vectors,
	},
};

struct msm_bus_scale_pdata grp2d0_bus_scale_pdata = {
	grp2d0_bus_scale_usecases,
	ARRAY_SIZE(grp2d0_bus_scale_usecases),
	.name = "grp2d0",
};

static struct msm_bus_vectors grp2d1_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_2D_CORE1,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors grp2d1_max_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_2D_CORE1,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 2096000000U,
	},
};

static struct msm_bus_paths grp2d1_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(grp2d1_init_vectors),
		grp2d1_init_vectors,
	},
	{
		ARRAY_SIZE(grp2d1_max_vectors),
		grp2d1_max_vectors,
	},
};

struct msm_bus_scale_pdata grp2d1_bus_scale_pdata = {
	grp2d1_bus_scale_usecases,
	ARRAY_SIZE(grp2d1_bus_scale_usecases),
	.name = "grp2d1",
};
#endif

#ifdef CONFIG_HW_RANDOM_MSM
static struct resource rng_resources = {
	.flags = IORESOURCE_MEM,
	.start = MSM_PRNG_PHYS,
	.end   = MSM_PRNG_PHYS + SZ_512 - 1,
};

struct platform_device msm_device_rng = {
	.name          = "msm_rng",
	.id            = 0,
	.num_resources = 1,
	.resource      = &rng_resources,
};
#endif

struct kgsl_platform_data kgsl_pdata = {
#ifdef CONFIG_MSM_NPA_SYSTEM_BUS
	/* NPA Flow IDs */
	.high_axi_3d = MSM_AXI_FLOW_3D_GPU_HIGH,
	.high_axi_2d = MSM_AXI_FLOW_2D_GPU_HIGH,
#else
	/* AXI rates in KHz */
	.high_axi_3d = 200000,
	.high_axi_2d = 160000,
#endif
	.max_grp2d_freq = 228571000,
	.min_grp2d_freq = 228571000,
	.set_grp2d_async = NULL, /* HW workaround, run Z180 SYNC @ 192 MHZ */
	.max_grp3d_freq = 266667000,
	.min_grp3d_freq = 266667000,
	.set_grp3d_async = NULL,
	.imem_clk_name = "imem_axi_clk",
	.imem_pclk_name = "imem_pclk",
	.grp3d_clk_name = "gfx3d_clk",
	.grp3d_pclk_name = "gfx3d_pclk",
#ifdef CONFIG_MSM_KGSL_2D
	.grp2d0_clk_name = "gfx2d0_clk", /* note: 2d clocks disabled on v1 */
	.grp2d0_pclk_name = "gfx2d0_pclk",
	.grp2d1_clk_name = "gfx2d1_clk",
	.grp2d1_pclk_name = "gfx2d1_pclk",
#else
	.grp2d0_clk_name = NULL,
	.grp2d1_clk_name = NULL,
#endif
	.idle_timeout_3d = HZ/5,
	.idle_timeout_2d = HZ/10,
#ifdef CONFIG_MSM_BUS_SCALING
	.grp3d_bus_scale_table = &grp3d_bus_scale_pdata,
	.grp2d0_bus_scale_table = &grp2d0_bus_scale_pdata,
	.grp2d1_bus_scale_table = &grp2d1_bus_scale_pdata,
	.nap_allowed = true,
#endif
#ifdef CONFIG_KGSL_PER_PROCESS_PAGE_TABLE
	.pt_va_size = SZ_256M - SZ_64K,
#else
	.pt_va_size = SZ_256M - SZ_64K,
#endif
};

/*
 * this a software workaround for not having two distinct board
 * files for 8660v1 and 8660v2. 8660v1 has a faulty 2d clock, and
 * this workaround detects the cpu version to tell if the kernel is on a
 * 8660v1, and should disable the 2d core. it is called from the board file
 */
void __init msm8x60_check_2d_hardware(void)
{
	if ((SOCINFO_VERSION_MAJOR(socinfo_get_version()) == 1) &&
	    (SOCINFO_VERSION_MINOR(socinfo_get_version()) == 0)) {
		printk(KERN_WARNING "kgsl: 2D cores disabled on 8660v1\n");
		kgsl_pdata.grp2d0_clk_name = NULL;
		kgsl_pdata.grp2d1_clk_name = NULL;
	}
}

struct platform_device msm_device_kgsl = {
	.name = "kgsl",
	.id = -1,
	.num_resources = ARRAY_SIZE(kgsl_resources),
	.resource = kgsl_resources,
	.dev = {
		.platform_data = &kgsl_pdata,
	},
};

/* Use GSBI3 QUP for /dev/i2c-0 */
struct platform_device msm_gsbi3_qup_i2c_device = {
	.name		= "qup_i2c",
	.id		= MSM_GSBI3_QUP_I2C_BUS_ID,
	.num_resources	= ARRAY_SIZE(gsbi3_qup_i2c_resources),
	.resource	= gsbi3_qup_i2c_resources,
};

/* Use GSBI4 QUP for /dev/i2c-1 */
struct platform_device msm_gsbi4_qup_i2c_device = {
	.name		= "qup_i2c",
	.id		= MSM_GSBI4_QUP_I2C_BUS_ID,
	.num_resources	= ARRAY_SIZE(gsbi4_qup_i2c_resources),
	.resource	= gsbi4_qup_i2c_resources,
};

/* Use GSBI8 QUP for /dev/i2c-3 */
struct platform_device msm_gsbi8_qup_i2c_device = {
	.name		= "qup_i2c",
	.id		= MSM_GSBI8_QUP_I2C_BUS_ID,
	.num_resources	= ARRAY_SIZE(gsbi8_qup_i2c_resources),
	.resource	= gsbi8_qup_i2c_resources,
};

/* Use GSBI9 QUP for /dev/i2c-2 */
struct platform_device msm_gsbi9_qup_i2c_device = {
	.name		= "qup_i2c",
	.id		= MSM_GSBI9_QUP_I2C_BUS_ID,
	.num_resources	= ARRAY_SIZE(gsbi9_qup_i2c_resources),
	.resource	= gsbi9_qup_i2c_resources,
};

/* Use GSBI7 QUP for /dev/i2c-4 (Marimba) */
struct platform_device msm_gsbi7_qup_i2c_device = {
	.name		= "qup_i2c",
	.id		= MSM_GSBI7_QUP_I2C_BUS_ID,
	.num_resources	= ARRAY_SIZE(gsbi7_qup_i2c_resources),
	.resource	= gsbi7_qup_i2c_resources,
};

/* Use GSBI10 QUP for /dev/i2c-5  */
struct platform_device msm_gsbi10_qup_i2c_device = {
    .name       = "qup_i2c",
    .id     = MSM_GSBI10_QUP_I2C_BUS_ID,
    .num_resources  = ARRAY_SIZE(gsbi10_qup_i2c_resources),
    .resource   = gsbi10_qup_i2c_resources,
};

/* Use GSBI5 QUP for /dev/i2c-9  */
struct platform_device msm_gsbi5_qup_i2c_device = {
    .name       = "qup_i2c",
    .id     = MSM_GSBI5_QUP_I2C_BUS_ID,
    .num_resources  = ARRAY_SIZE(gsbi5_qup_i2c_resources),
    .resource   = gsbi5_qup_i2c_resources,
};

#ifdef CONFIG_I2C_SSBI
/* 8058 PMIC SSBI on /dev/i2c-6 */
#define MSM_SSBI1_PMIC1C_PHYS	0x00500000
static struct resource msm_ssbi1_resources[] = {
	{
		.name   = "ssbi_base",
		.start	= MSM_SSBI1_PMIC1C_PHYS,
		.end	= MSM_SSBI1_PMIC1C_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device msm_device_ssbi1 = {
	.name		= "i2c_ssbi",
	.id		= MSM_SSBI1_I2C_BUS_ID,
	.num_resources	= ARRAY_SIZE(msm_ssbi1_resources),
	.resource	= msm_ssbi1_resources,
};

/* 8901 PMIC SSBI on /dev/i2c-7 */
#define MSM_SSBI2_PMIC2B_PHYS	0x00C00000
static struct resource msm_ssbi2_resources[] = {
	{
		.name   = "ssbi_base",
		.start	= MSM_SSBI2_PMIC2B_PHYS,
		.end	= MSM_SSBI2_PMIC2B_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device msm_device_ssbi2 = {
	.name		= "i2c_ssbi",
	.id		= MSM_SSBI2_I2C_BUS_ID,
	.num_resources	= ARRAY_SIZE(msm_ssbi2_resources),
	.resource	= msm_ssbi2_resources,
};

/* CODEC SSBI on /dev/i2c-8 */
#define MSM_SSBI3_PHYS  0x18700000
static struct resource msm_ssbi3_resources[] = {
	{
		.name   = "ssbi_base",
		.start  = MSM_SSBI3_PHYS,
		.end    = MSM_SSBI3_PHYS + SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
};

struct platform_device msm_device_ssbi3 = {
	.name		= "i2c_ssbi",
	.id		= MSM_SSBI3_I2C_BUS_ID,
	.num_resources	= ARRAY_SIZE(msm_ssbi3_resources),
	.resource	= msm_ssbi3_resources,
};
#endif /* CONFIG_I2C_SSBI */

static struct resource gsbi1_qup_spi_resources[] = {
	{
		.name	= "spi_base",
		.start	= MSM_GSBI1_QUP_PHYS,
		.end	= MSM_GSBI1_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_base",
		.start	= MSM_GSBI1_PHYS,
		.end	= MSM_GSBI1_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "spi_irq_in",
		.start	= GSBI1_QUP_IRQ,
		.end	= GSBI1_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spidm_channels",
		.start  = 5,
		.end    = 6,
		.flags  = IORESOURCE_DMA,
	},
	{
		.name   = "spidm_crci",
		.start  = 8,
		.end    = 7,
		.flags  = IORESOURCE_DMA,
	},
};

/* Use GSBI1 QUP for SPI-0 */
struct platform_device msm_gsbi1_qup_spi_device = {
	.name		= "spi_qsd",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(gsbi1_qup_spi_resources),
	.resource	= gsbi1_qup_spi_resources,
};

static struct resource gsbi5_qup_spi_resources[] = {
	{
		.name	= "spi_base",
		.start	= MSM_GSBI5_QUP_PHYS,
		.end	= MSM_GSBI5_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_base",
		.start	= MSM_GSBI5_PHYS,
		.end	= MSM_GSBI5_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "spi_irq_in",
		.start	= GSBI5_QUP_IRQ,
		.end	= GSBI5_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "spidm_channels",
		.start	= 5,
		.end	= 6,
		.flags	= IORESOURCE_DMA,
	},
	{
		.name	= "spidm_crci",
		.start	= 8,
		.end	= 7,
		.flags	= IORESOURCE_DMA,
	},
};

/* Use GSBI5 QUP for SPI-1 */
struct platform_device msm_gsbi5_qup_spi_device = {
	.name	= "spi_qsd",
	.id	= MSM_GSBI5_QUP_SPI_BUS_ID,
	.num_resources	= ARRAY_SIZE(gsbi5_qup_spi_resources),
	.resource	= gsbi5_qup_spi_resources,
};

static struct resource gsbi10_qup_spi_resources[] = {
	{
		.name	= "spi_base",
		.start	= MSM_GSBI10_QUP_PHYS,
		.end	= MSM_GSBI10_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_base",
		.start	= MSM_GSBI10_PHYS,
		.end	= MSM_GSBI10_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "spi_irq_in",
		.start	= GSBI10_QUP_IRQ,
		.end	= GSBI10_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

/* Use GSBI10 QUP for SPI-1 */
struct platform_device msm_gsbi10_qup_spi_device = {
	.name		= "spi_qsd",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(gsbi10_qup_spi_resources),
	.resource	= gsbi10_qup_spi_resources,
};
#define MSM_SDC1_BASE         0x12400000
#define MSM_SDC2_BASE         0x12140000
#define MSM_SDC3_BASE         0x12180000
#define MSM_SDC4_BASE         0x121C0000
#define MSM_SDC5_BASE         0x12200000

static struct resource resources_sdc1[] = {
	{
		.start	= MSM_SDC1_BASE,
		.end	= MSM_SDC1_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= SDC1_IRQ_0,
		.end	= SDC1_IRQ_0,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= DMOV_SDC1_CHAN,
		.end	= DMOV_SDC1_CHAN,
		.flags	= IORESOURCE_DMA,
	},
};

static struct resource resources_sdc2[] = {
	{
		.start	= MSM_SDC2_BASE,
		.end	= MSM_SDC2_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= SDC2_IRQ_0,
		.end	= SDC2_IRQ_0,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= DMOV_SDC2_CHAN,
		.end	= DMOV_SDC2_CHAN,
		.flags	= IORESOURCE_DMA,
	},
};

static struct resource resources_sdc3[] = {
	{
		.start	= MSM_SDC3_BASE,
		.end	= MSM_SDC3_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= SDC3_IRQ_0,
		.end	= SDC3_IRQ_0,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= DMOV_SDC3_CHAN,
		.end	= DMOV_SDC3_CHAN,
		.flags	= IORESOURCE_DMA,
	},
};

static struct resource resources_sdc4[] = {
	{
		.start	= MSM_SDC4_BASE,
		.end	= MSM_SDC4_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= SDC4_IRQ_0,
		.end	= SDC4_IRQ_0,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= DMOV_SDC4_CHAN,
		.end	= DMOV_SDC4_CHAN,
		.flags	= IORESOURCE_DMA,
	},
};

static struct resource resources_sdc5[] = {
	{
		.start	= MSM_SDC5_BASE,
		.end	= MSM_SDC5_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= SDC5_IRQ_0,
		.end	= SDC5_IRQ_0,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= DMOV_SDC5_CHAN,
		.end	= DMOV_SDC5_CHAN,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device msm_device_sdc1 = {
	.name		= "msm_sdcc",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(resources_sdc1),
	.resource	= resources_sdc1,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct platform_device msm_device_sdc2 = {
	.name		= "msm_sdcc",
	.id		= 2,
	.num_resources	= ARRAY_SIZE(resources_sdc2),
	.resource	= resources_sdc2,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct platform_device msm_device_sdc3 = {
	.name		= "msm_sdcc",
	.id		= 3,
	.num_resources	= ARRAY_SIZE(resources_sdc3),
	.resource	= resources_sdc3,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct platform_device msm_device_sdc4 = {
	.name		= "msm_sdcc",
	.id		= 4,
	.num_resources	= ARRAY_SIZE(resources_sdc4),
	.resource	= resources_sdc4,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct platform_device msm_device_sdc5 = {
	.name		= "msm_sdcc",
	.id		= 5,
	.num_resources	= ARRAY_SIZE(resources_sdc5),
	.resource	= resources_sdc5,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

static struct platform_device *msm_sdcc_devices[] __initdata = {
	&msm_device_sdc1,
	&msm_device_sdc2,
	&msm_device_sdc3,
	&msm_device_sdc4,
	&msm_device_sdc5,
};

int __init msm_add_sdcc(unsigned int controller, struct mmc_platform_data *plat)
{
	struct platform_device	*pdev;

	if (controller < 1 || controller > 5)
		return -EINVAL;

	pdev = msm_sdcc_devices[controller-1];
	pdev->dev.platform_data = plat;
	return platform_device_register(pdev);
}

#define MIPI_DSI_HW_BASE	0x04700000
#define ROTATOR_HW_BASE		0x04E00000
#define TVENC_HW_BASE		0x04F00000
#define MDP_HW_BASE		0x05100000

static struct resource msm_mipi_dsi_resources[] = {
	{
		.name   = "mipi_dsi",
		.start  = MIPI_DSI_HW_BASE,
		.end    = MIPI_DSI_HW_BASE + 0x000F0000 - 1,
		.flags  = IORESOURCE_MEM,
	}
};

static struct platform_device msm_mipi_dsi_device = {
	.name   = "mipi_dsi",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_mipi_dsi_resources),
	.resource       = msm_mipi_dsi_resources,
};

static struct resource msm_mdp_resources[] = {
	{
		.name   = "mdp",
		.start  = MDP_HW_BASE,
		.end    = MDP_HW_BASE + 0x000F0000 - 1,
		.flags  = IORESOURCE_MEM,
	}
};

static struct platform_device msm_mdp_device = {
	.name   = "mdp",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_mdp_resources),
	.resource       = msm_mdp_resources,
};
#ifdef CONFIG_MSM_ROTATOR
static struct resource resources_msm_rotator[] = {
	{
		.start	= 0x04E00000,
		.end	= 0x04F00000 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= ROT_IRQ,
		.end	= ROT_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct msm_rot_clocks rotator_clocks[] = {
	{
		.clk_name = "rot_clk",
		.clk_type = ROTATOR_AXI_CLK,
		.clk_rate = 160 * 1000 * 1000,
	},
	{
		.clk_name = "rotator_pclk",
		.clk_type = ROTATOR_PCLK,
		.clk_rate = 0,
	},
};

static struct msm_rotator_platform_data rotator_pdata = {
	.number_of_clocks = ARRAY_SIZE(rotator_clocks),
	.hardware_version_number = 0x01010307,
	.rotator_clks = rotator_clocks,
	.regulator_name = "fs_rot",
};

struct platform_device msm_rotator_device = {
	.name		= "msm_rotator",
	.id		= 0,
	.num_resources  = ARRAY_SIZE(resources_msm_rotator),
	.resource       = resources_msm_rotator,
	.dev		= {
		.platform_data = &rotator_pdata,
	},
};
#endif


/* Sensors DSPS platform data */
#ifdef CONFIG_MSM_DSPS

#define PPSS_REG_PHYS_BASE	0x12080000

#define MHZ (1000*1000)

static struct dsps_clk_info dsps_clks[] = {
	{
		.name = "ppss_pclk",
		.rate =	0, /* no rate just on/off */
	},
	{
		.name = "pmem_clk",
		.rate =	0, /* no rate just on/off */
	},
	{
		.name = "gsbi_qup_clk",
		.rate =	24 * MHZ, /* See clk_tbl_gsbi_qup[] */
	},
	{
		.name = "dfab_dsps_clk",
		.rate =	64 * MHZ, /* Same rate as USB. */
	}
};

static struct dsps_regulator_info dsps_regs[] = {
	{
		.name = "8058_l5",
		.volt = 2850000, /* in uV */
	},
	{
		.name = "8058_s3",
		.volt = 1800000, /* in uV */
	}
};

/*
 * Note: GPIOs field is	intialized in run-time at the function
 * msm8x60_init_dsps().
 */

struct msm_dsps_platform_data msm_dsps_pdata = {
	.clks = dsps_clks,
	.clks_num = ARRAY_SIZE(dsps_clks),
	.gpios = NULL,
	.gpios_num = 0,
	.regs = dsps_regs,
	.regs_num = ARRAY_SIZE(dsps_regs),
	.signature = DSPS_SIGNATURE,
};

static struct resource msm_dsps_resources[] = {
	{
		.start = PPSS_REG_PHYS_BASE,
		.end   = PPSS_REG_PHYS_BASE + SZ_8K - 1,
		.name  = "ppss_reg",
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device msm_dsps_device = {
	.name          = "msm_dsps",
	.id            = 0,
	.num_resources = ARRAY_SIZE(msm_dsps_resources),
	.resource      = msm_dsps_resources,
	.dev.platform_data = &msm_dsps_pdata,
};

#endif /* CONFIG_MSM_DSPS */

#ifdef CONFIG_FB_MSM_TVOUT
static struct resource msm_tvenc_resources[] = {
	{
		.name   = "tvenc",
		.start  = TVENC_HW_BASE,
		.end    = TVENC_HW_BASE + PAGE_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	}
};

static struct resource tvout_device_resources[] = {
	{
		.name  = "tvout_device_irq",
		.start = TV_ENC_IRQ,
		.end   = TV_ENC_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};
#endif
static void __init msm_register_device(struct platform_device *pdev, void *data)
{
	int ret;

	pdev->dev.platform_data = data;

	ret = platform_device_register(pdev);
	if (ret)
		dev_err(&pdev->dev,
			  "%s: platform_device_register() failed = %d\n",
			  __func__, ret);
}

static struct platform_device msm_lcdc_device = {
	.name   = "lcdc",
	.id     = 0,
};

#ifdef CONFIG_FB_MSM_TVOUT
static struct platform_device msm_tvenc_device = {
	.name   = "tvenc",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_tvenc_resources),
	.resource       = msm_tvenc_resources,
};

static struct platform_device msm_tvout_device = {
	.name = "tvout_device",
	.id = 0,
	.num_resources = ARRAY_SIZE(tvout_device_resources),
	.resource = tvout_device_resources,
};
#endif

#ifdef CONFIG_MSM_BUS_SCALING
static struct platform_device msm_dtv_device = {
	.name   = "dtv",
	.id     = 0,
};
#endif

void __init msm_fb_register_device(char *name, void *data)
{
	if (!strncmp(name, "mdp", 3))
		msm_register_device(&msm_mdp_device, data);
	else if (!strncmp(name, "lcdc", 4))
		msm_register_device(&msm_lcdc_device, data);
	else if (!strncmp(name, "mipi_dsi", 8))
		msm_register_device(&msm_mipi_dsi_device, data);
#ifdef CONFIG_FB_MSM_TVOUT
	else if (!strncmp(name, "tvenc", 5))
		msm_register_device(&msm_tvenc_device, data);
	else if (!strncmp(name, "tvout_device", 12))
		msm_register_device(&msm_tvout_device, data);
#endif
#ifdef CONFIG_MSM_BUS_SCALING
	else if (!strncmp(name, "dtv", 3))
		msm_register_device(&msm_dtv_device, data);
#endif
	else
		printk(KERN_ERR "%s: unknown device! %s\n", __func__, name);
}

static struct resource resources_otg[] = {
	{
		.start	= 0x12500000,
		.end	= 0x12500000 + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= USB1_HS_IRQ,
		.end	= USB1_HS_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_otg = {
	.name		= "msm_otg",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_otg),
	.resource	= resources_otg,
};

static u64 dma_mask = 0xffffffffULL;
struct platform_device msm_device_gadget_peripheral = {
	.name		= "msm_hsusb",
	.id		= -1,
	.dev		= {
		.dma_mask 		= &dma_mask,
		.coherent_dma_mask	= 0xffffffffULL,
	},
};
#ifdef CONFIG_USB_EHCI_MSM
static struct resource resources_hsusb_host[] = {
	{
		.start	= 0x12500000,
		.end	= 0x12500000 + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= USB1_HS_IRQ,
		.end	= USB1_HS_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_hsusb_host = {
	.name		= "msm_hsusb_host",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(resources_hsusb_host),
	.resource	= resources_hsusb_host,
	.dev		= {
		.dma_mask 		= &dma_mask,
		.coherent_dma_mask	= 0xffffffffULL,
	},
};

static struct platform_device *msm_host_devices[] = {
	&msm_device_hsusb_host,
};

int msm_add_host(unsigned int host, struct msm_usb_host_platform_data *plat)
{
	struct platform_device	*pdev;

	pdev = msm_host_devices[host];
	if (!pdev)
		return -ENODEV;
	pdev->dev.platform_data = plat;
	return platform_device_register(pdev);
}
#endif

#ifdef CONFIG_USB_ANDROID_DIAG
#define PID_MAGIC_ID		0x71432909
#define SERIAL_NUM_MAGIC_ID	0x61945374
#define SERIAL_NUMBER_LENGTH	127
#define DLOAD_USB_BASE_ADD	0x2A05F0C8

struct magic_num_struct {
	uint32_t pid;
	uint32_t serial_num;
};

struct dload_struct {
	uint32_t	reserved1;
	uint32_t	reserved2;
	uint32_t	reserved3;
	uint16_t	reserved4;
	uint16_t	pid;
	char		serial_number[SERIAL_NUMBER_LENGTH];
	uint16_t	reserved5;
	struct magic_num_struct
			magic_struct;
};

static int usb_diag_update_pid_and_serial_num(uint32_t pid, const char *snum)
{
	struct dload_struct __iomem *dload = 0;

	dload = ioremap(DLOAD_USB_BASE_ADD, sizeof(*dload));
	if (!dload) {
		pr_err("%s: cannot remap I/O memory region: %08x\n",
					__func__, DLOAD_USB_BASE_ADD);
		return -ENXIO;
	}

	pr_debug("%s: dload:%p pid:%x serial_num:%s\n",
				__func__, dload, pid, snum);
	/* update pid */
	dload->magic_struct.pid = PID_MAGIC_ID;
	dload->pid = pid;

	/* update serial number */
	dload->magic_struct.serial_num = 0;
	if (!snum)
		return 0;

	dload->magic_struct.serial_num = SERIAL_NUM_MAGIC_ID;
	strncpy(dload->serial_number, snum, SERIAL_NUMBER_LENGTH);
	dload->serial_number[SERIAL_NUMBER_LENGTH - 1] = '\0';

	iounmap(dload);

	return 0;
}

struct usb_diag_platform_data usb_diag_pdata = {
	.ch_name = DIAG_LEGACY,
	.update_pid_and_serial_num = usb_diag_update_pid_and_serial_num,
};

struct platform_device usb_diag_device = {
	.name	= "usb_diag",
	.id	= 0,
	.dev	= {
		.platform_data = &usb_diag_pdata,
	},
};

struct usb_diag_platform_data usb_diag_mdm_pdata = {
	.ch_name = DIAG_MDM,
};

struct platform_device usb_diag_mdm_device = {
	.name	= "usb_diag",
	.id	= 1,
	.dev	= {
		.platform_data = &usb_diag_mdm_pdata,
	},
};
#endif

struct platform_device msm_device_smd = {
	.name           = "msm_smd",
	.id             = -1,
};

struct resource msm_dmov_resource_adm0[] = {
	{
		.start = INT_ADM0_AARM,
		.end = (resource_size_t)MSM_DMOV_ADM0_BASE,
		.flags = IORESOURCE_IRQ,
	},
};

struct resource msm_dmov_resource_adm1[] = {
	{
		.start = INT_ADM1_AARM,
		.end = (resource_size_t)MSM_DMOV_ADM1_BASE,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_dmov_adm0 = {
	.name	= "msm_dmov",
	.id	= 0,
	.resource = msm_dmov_resource_adm0,
	.num_resources = ARRAY_SIZE(msm_dmov_resource_adm0),
};

struct platform_device msm_device_dmov_adm1 = {
	.name	= "msm_dmov",
	.id	= 1,
	.resource = msm_dmov_resource_adm1,
	.num_resources = ARRAY_SIZE(msm_dmov_resource_adm1),
};

/* MSM Video core device */

#define MSM_VIDC_BASE_PHYS 0x04400000
#define MSM_VIDC_BASE_SIZE 0x00100000

static struct resource msm_device_vidc_resources[] = {
	{
		.start	= MSM_VIDC_BASE_PHYS,
		.end	= MSM_VIDC_BASE_PHYS + MSM_VIDC_BASE_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= VCODEC_IRQ,
		.end	= VCODEC_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_vidc = {
	.name = "msm_vidc",
	.id = 0,
	.num_resources = ARRAY_SIZE(msm_device_vidc_resources),
	.resource = msm_device_vidc_resources,
};

#if defined(CONFIG_MSM_RPM_STATS_LOG)
static struct msm_rpmstats_platform_data msm_rpm_stat_pdata = {
	.phys_addr_base = 0x00107E04,
	.phys_size = SZ_8K,
};

struct platform_device msm_rpm_stat_device = {
	.name = "msm_rpm_stat",
	.id = -1,
	.dev = {
		.platform_data = &msm_rpm_stat_pdata,
	},
};
#endif

#ifdef CONFIG_MSM_BUS_SCALING
struct platform_device msm_bus_sys_fabric = {
	.name  = "msm_bus_fabric",
	.id    =  MSM_BUS_FAB_SYSTEM,
};
struct platform_device msm_bus_apps_fabric = {
	.name  = "msm_bus_fabric",
	.id    = MSM_BUS_FAB_APPSS,
};
struct platform_device msm_bus_mm_fabric = {
	.name  = "msm_bus_fabric",
	.id    = MSM_BUS_FAB_MMSS,
};
struct platform_device msm_bus_sys_fpb = {
	.name  = "msm_bus_fabric",
	.id    = MSM_BUS_FAB_SYSTEM_FPB,
};
struct platform_device msm_bus_cpss_fpb = {
	.name  = "msm_bus_fabric",
	.id    = MSM_BUS_FAB_CPSS_FPB,
};
#endif

#define FS(_id, _name) (&(struct platform_device){ \
	.name	= "footswitch-msm8x60", \
	.id	= (_id), \
	.dev	= { \
		.platform_data = &(struct regulator_init_data){ \
			.constraints = { \
				.valid_modes_mask = REGULATOR_MODE_NORMAL, \
				.valid_ops_mask   = REGULATOR_CHANGE_STATUS, \
			}, \
			.num_consumer_supplies = 1, \
			.consumer_supplies = \
				&(struct regulator_consumer_supply) \
				REGULATOR_SUPPLY((_name), NULL), \
		} \
	}, \
})
struct platform_device *msm_footswitch_devices[] = {
	FS(FS_IJPEG,	"fs_ijpeg"),
	FS(FS_MDP,	"fs_mdp"),
	FS(FS_ROT,	"fs_rot"),
	FS(FS_VED,	"fs_ved"),
	FS(FS_VFE,	"fs_vfe"),
	FS(FS_VPE,	"fs_vpe"),
	FS(FS_GFX3D,	"fs_gfx3d"),
	FS(FS_GFX2D0,	"fs_gfx2d0"),
	FS(FS_GFX2D1,	"fs_gfx2d1"),
};
unsigned msm_num_footswitch_devices = ARRAY_SIZE(msm_footswitch_devices);

struct clk_lookup msm_clocks_8x60[] = {
	CLK_RPM("afab_clk",		AFAB_CLK,		NULL, CLK_MIN),
	CLK_RPM("afab_a_clk",		AFAB_A_CLK,		NULL, CLK_MIN),
	CLK_RPM("cfpb_clk",		CFPB_CLK,		NULL, CLK_MIN),
	CLK_RPM("cfpb_a_clk",		CFPB_A_CLK,		NULL, CLK_MIN),
	CLK_RPM("dfab_clk",		DFAB_CLK,		NULL, CLK_MIN),
	CLK_RPM("dfab_a_clk",		DFAB_A_CLK,		NULL, CLK_MIN),
	CLK_RPM("ebi1_clk",		EBI1_CLK,		NULL, CLK_MIN),
	CLK_RPM("ebi1_a_clk",		EBI1_A_CLK,		NULL, CLK_MIN),
	CLK_RPM("mmfab_clk",		MMFAB_CLK,		NULL, CLK_MIN),
	CLK_RPM("mmfab_a_clk",		MMFAB_A_CLK,		NULL, CLK_MIN),
	CLK_RPM("mmfpb_clk",		MMFPB_CLK,		NULL, CLK_MIN),
	CLK_RPM("mmfpb_a_clk",		MMFPB_A_CLK,		NULL, CLK_MIN),
	CLK_RPM("sfab_clk",		SFAB_CLK,		NULL, CLK_MIN),
	CLK_RPM("sfab_a_clk",		SFAB_A_CLK,		NULL, CLK_MIN),
	CLK_RPM("sfpb_clk",		SFPB_CLK,		NULL, CLK_MIN),
	CLK_RPM("sfpb_a_clk",		SFPB_A_CLK,		NULL, CLK_MIN),
	CLK_RPM("smi_clk",		SMI_CLK,		NULL, CLK_MIN),
	CLK_RPM("smi_a_clk",		SMI_A_CLK,		NULL, CLK_MIN),

	CLK_8X60("gsbi_uart_clk",	GSBI1_UART_CLK,		NULL, OFF),
	CLK_8X60("gsbi_uart_clk",	GSBI2_UART_CLK,		NULL, OFF),
	CLK_8X60("gsbi_uart_clk",	GSBI3_UART_CLK, "msm_serial_hsl.2",
			OFF),
	CLK_8X60("gsbi_uart_clk",	GSBI4_UART_CLK,		NULL, OFF),
	CLK_8X60("gsbi_uart_clk",	GSBI5_UART_CLK,		NULL, OFF),
	CLK_8X60("uartdm_clk",	GSBI6_UART_CLK, "msm_uartdm.0", OFF), // Palm added
	CLK_8X60("uartdm_clk",	GSBI6_UART_CLK, "msm_serial_hs.0", OFF),
	CLK_8X60("gsbi_uart_clk",	GSBI7_UART_CLK,		NULL, OFF),
	CLK_8X60("gsbi_uart_clk",	GSBI8_UART_CLK,		NULL, OFF),
	CLK_8X60("gsbi_uart_clk",	GSBI9_UART_CLK, "msm_serial_hsl.1", OFF),
	CLK_8X60("uartdm_clk",		GSBI10_UART_CLK, "msm_uartdm.1", OFF), // Palm added
	CLK_8X60("gsbi_uart_clk",	GSBI11_UART_CLK, "msm_serial_hsl.3", OFF),
	CLK_8X60("gsbi_uart_clk",	GSBI12_UART_CLK, "msm_serial_hsl.0",
			OFF),
	CLK_8X60("gsbi_qup_clk",	GSBI1_QUP_CLK, "spi_qsd.0", OFF),
	CLK_8X60("gsbi_qup_clk",	GSBI2_QUP_CLK,		NULL, OFF),
	CLK_8X60("gsbi_qup_clk",	GSBI3_QUP_CLK, "qup_i2c.0", OFF),
	CLK_8X60("gsbi_qup_clk",	GSBI4_QUP_CLK, "qup_i2c.1", OFF),
	CLK_8X60("gsbi_qup_clk",	GSBI5_QUP_CLK, "spi_qsd.2", OFF),
	CLK_8X60("gsbi_qup_clk",	GSBI5_QUP_CLK, "qup_i2c.9", OFF),
	CLK_8X60("gsbi_qup_clk",	GSBI6_QUP_CLK,		NULL, OFF),
	CLK_8X60("gsbi_qup_clk",	GSBI7_QUP_CLK, "qup_i2c.4", OFF),
	CLK_8X60("gsbi_qup_clk",	GSBI8_QUP_CLK, "qup_i2c.3", OFF),
	CLK_8X60("gsbi_qup_clk",	GSBI9_QUP_CLK, "qup_i2c.2", OFF),
	CLK_8X60("gsbi_qup_clk",	GSBI10_QUP_CLK,	"qup_i2c.5", OFF), // Palm added
	CLK_8X60("gsbi_qup_clk",	GSBI10_QUP_CLK,	"spi_qsd.1", OFF),
	CLK_8X60("gsbi_qup_clk",	GSBI11_QUP_CLK,		NULL, OFF),
	CLK_8X60("gsbi_qup_clk",	GSBI12_QUP_CLK,	"msm_dsps.0", OFF),
	CLK_8X60("pdm_clk",		PDM_CLK,		NULL, OFF),
	CLK_8X60("pmem_clk",		PMEM_CLK,		NULL, OFF),
	CLK_8X60("prng_clk",		PRNG_CLK,		NULL, OFF),
	CLK_8X60("sdc_clk",		SDC1_CLK, "msm_sdcc.1", OFF),
	CLK_8X60("sdc_clk",		SDC2_CLK, "msm_sdcc.2", OFF),
	CLK_8X60("sdc_clk",		SDC3_CLK, "msm_sdcc.3", OFF),
	CLK_8X60("sdc_clk",		SDC4_CLK, "msm_sdcc.4", OFF),
	CLK_8X60("sdc_clk",		SDC5_CLK, "msm_sdcc.5", OFF),
	CLK_8X60("tsif_ref_clk",	TSIF_REF_CLK,		NULL, OFF),
	CLK_8X60("tssc_clk",		TSSC_CLK,		NULL, OFF),
	CLK_8X60("usb_hs_clk",		USB_HS1_XCVR_CLK,	NULL, OFF),
	CLK_8X60("usb_phy_clk",		USB_PHY0_CLK,		NULL, OFF),
	CLK_8X60("usb_fs_clk",		USB_FS1_XCVR_CLK,	NULL, OFF),
	CLK_8X60("usb_fs_sys_clk",	USB_FS1_SYS_CLK,	NULL, OFF),
	CLK_8X60("usb_fs_src_clk",	USB_FS1_SRC_CLK,	NULL, OFF),
	CLK_8X60("usb_fs_clk",		USB_FS2_XCVR_CLK,	NULL, OFF),
	CLK_8X60("usb_fs_sys_clk",	USB_FS2_SYS_CLK,	NULL, OFF),
	CLK_8X60("usb_fs_src_clk",	USB_FS2_SRC_CLK,	NULL, OFF),
	CLK_8X60("ce_clk",		CE2_P_CLK,		NULL, OFF),
	CLK_8X60("gsbi_pclk",		GSBI1_P_CLK, "spi_qsd.0", OFF),
	CLK_8X60("gsbi_pclk",		GSBI2_P_CLK,		NULL, OFF),
	CLK_8X60("gsbi_pclk",		GSBI3_P_CLK, "msm_serial_hsl.2", 0),
	CLK_8X60("gsbi_pclk",		GSBI3_P_CLK, "qup_i2c.0", OFF),
	CLK_8X60("gsbi_pclk",		GSBI4_P_CLK, "qup_i2c.1", OFF),
	CLK_8X60("gsbi_pclk",		GSBI5_P_CLK, "spi_qsd.2", OFF),
	CLK_8X60("gsbi_pclk",		GSBI5_P_CLK, "qup_i2c.9", OFF),
	CLK_8X60("uartdm_pclk",		GSBI6_P_CLK, "msm_uartdm.0", OFF),
	CLK_8X60("gsbi_pclk",		GSBI7_P_CLK, "qup_i2c.4", OFF),
	CLK_8X60("gsbi_pclk",		GSBI8_P_CLK, "qup_i2c.3", OFF),
	CLK_8X60("gsbi_pclk",		GSBI9_P_CLK, "msm_serial_hsl.1", OFF),
	CLK_8X60("gsbi_pclk",		GSBI9_P_CLK, "qup_i2c.2", OFF),
	CLK_8X60("gsbi_pclk",		GSBI10_P_CLK, "spi_qsd.1", OFF),
	CLK_8X60("gsbi_pclk",		GSBI10_P_CLK, "qup_i2c.5", OFF),
	CLK_8X60("uartdm_pclk",		GSBI10_P_CLK, "msm_uartdm.1", OFF),
	CLK_8X60("gsbi_pclk",		GSBI11_P_CLK, "msm_serial_hsl.3", OFF),
	CLK_8X60("gsbi_pclk",		GSBI12_P_CLK, "msm_dsps.0", 0),
	CLK_8X60("gsbi_pclk",		GSBI12_P_CLK, "msm_serial_hsl.0", 0),
	CLK_8X60("ppss_pclk",		PPSS_P_CLK,		NULL, OFF),
	CLK_8X60("tsif_pclk",		TSIF_P_CLK,		NULL, OFF),
	CLK_8X60("usb_fs_pclk",		USB_FS1_P_CLK,		NULL, OFF),
	CLK_8X60("usb_fs_pclk",		USB_FS2_P_CLK,		NULL, OFF),
	CLK_8X60("usb_hs_pclk",		USB_HS1_P_CLK,		NULL, OFF),
	CLK_8X60("sdc_pclk",		SDC1_P_CLK, "msm_sdcc.1", OFF),
	CLK_8X60("sdc_pclk",		SDC2_P_CLK, "msm_sdcc.2", OFF),
	CLK_8X60("sdc_pclk",		SDC3_P_CLK, "msm_sdcc.3", OFF),
	CLK_8X60("sdc_pclk",		SDC4_P_CLK, "msm_sdcc.4", OFF),
	CLK_8X60("sdc_pclk",		SDC5_P_CLK, "msm_sdcc.5", OFF),
	CLK_8X60("adm_clk",		ADM0_CLK, "msm_dmov.0", OFF),
	CLK_8X60("adm_pclk",		ADM0_P_CLK, "msm_dmov.0", OFF),
	CLK_8X60("adm_clk",		ADM1_CLK, "msm_dmov.1", OFF),
	CLK_8X60("adm_pclk",		ADM1_P_CLK, "msm_dmov.1", OFF),
	CLK_8X60("modem_ahb1_pclk",	MODEM_AHB1_P_CLK,	NULL, OFF),
	CLK_8X60("modem_ahb2_pclk",	MODEM_AHB2_P_CLK,	NULL, OFF),
	CLK_8X60("pmic_arb_pclk",	PMIC_ARB0_P_CLK,	NULL, OFF),
	CLK_8X60("pmic_arb_pclk",	PMIC_ARB1_P_CLK,	NULL, OFF),
	CLK_8X60("pmic_ssbi2",		PMIC_SSBI2_CLK,		NULL, OFF),
	CLK_8X60("rpm_msg_ram_pclk",	RPM_MSG_RAM_P_CLK,	NULL, OFF),
	CLK_8X60("amp_clk",		AMP_CLK,		NULL, OFF),
	CLK_8X60("cam_clk",		CAM_CLK,		NULL, OFF),
	CLK_8X60("csi_clk",		CSI0_CLK,		NULL, OFF),
	CLK_8X60("csi_clk",		CSI1_CLK,	  WEBCAM_DEV, OFF),
	CLK_8X60("csi_src_clk",		CSI_SRC_CLK,		NULL, OFF),
	CLK_8X60("dsi_byte_div_clk",	DSI_BYTE_CLK,		NULL, OFF),
	CLK_8X60("dsi_esc_clk",		DSI_ESC_CLK,		NULL, OFF),
	CLK_8X60("gfx2d0_clk",		GFX2D0_CLK,		NULL, OFF),
	CLK_8X60("gfx2d1_clk",		GFX2D1_CLK,		NULL, OFF),
	CLK_8X60("gfx3d_clk",		GFX3D_CLK,		NULL, OFF),
	CLK_8X60("ijpeg_clk",		IJPEG_CLK,		NULL, OFF),
	CLK_8X60("jpegd_clk",		JPEGD_CLK,		NULL, OFF),
	CLK_8X60("mdp_clk",		MDP_CLK,		NULL, OFF),
	CLK_8X60("mdp_vsync_clk",	MDP_VSYNC_CLK,		NULL, OFF),
	CLK_8X60("pixel_lcdc_clk",	PIXEL_LCDC_CLK,		NULL, OFF),
	CLK_8X60("pixel_mdp_clk",	PIXEL_MDP_CLK,		NULL, OFF),
	CLK_8X60("rot_clk",		ROT_CLK,		NULL, OFF),
	CLK_8X60("tv_enc_clk",		TV_ENC_CLK,		NULL, OFF),
	CLK_8X60("tv_dac_clk",		TV_DAC_CLK,		NULL, OFF),
	CLK_8X60("vcodec_clk",		VCODEC_CLK,		NULL, OFF),
	CLK_8X60("mdp_tv_clk",		MDP_TV_CLK,		NULL, OFF),
	CLK_8X60("hdmi_clk",		HDMI_TV_CLK,		NULL, OFF),
	CLK_8X60("tv_src_clk",		TV_SRC_CLK,		NULL, OFF),
	CLK_8X60("hdmi_app_clk",	HDMI_APP_CLK,		NULL, OFF),
	CLK_8X60("vpe_clk",		VPE_CLK,		NULL, OFF),
	CLK_8X60("csi_vfe_clk",		CSI0_VFE_CLK,		NULL, OFF),
	CLK_8X60("csi_vfe_clk",		CSI1_VFE_CLK,	  WEBCAM_DEV, OFF),
	CLK_8X60("vfe_clk",		VFE_CLK,		NULL, OFF),
	CLK_8X60("smmu_jpegd_clk",	JPEGD_AXI_CLK,		NULL, OFF),
	CLK_8X60("smmu_vfe_clk",	VFE_AXI_CLK,		NULL, OFF),
	CLK_8X60("vfe_axi_clk",		VFE_AXI_CLK,		NULL, OFF),
	CLK_8X60("ijpeg_axi_clk",	IJPEG_AXI_CLK,		NULL, OFF),
	CLK_8X60("imem_axi_clk",	IMEM_AXI_CLK,		NULL, OFF),
	CLK_8X60("mdp_axi_clk",		MDP_AXI_CLK,		NULL, OFF),
	CLK_8X60("rot_axi_clk",		ROT_AXI_CLK,		NULL, OFF),
	CLK_8X60("vcodec_axi_clk",	VCODEC_AXI_CLK,		NULL, OFF),
	CLK_8X60("vpe_axi_clk",		VPE_AXI_CLK,		NULL, OFF),
	CLK_8X60("amp_pclk",		AMP_P_CLK,		NULL, OFF),
	CLK_8X60("csi_pclk",		CSI0_P_CLK,		NULL, OFF),
	CLK_8X60("csi_pclk",		CSI1_P_CLK,	  WEBCAM_DEV, OFF),
	CLK_8X60("dsi_m_pclk",		DSI_M_P_CLK,		NULL, OFF),
	CLK_8X60("dsi_s_pclk",		DSI_S_P_CLK,		NULL, OFF),
	CLK_8X60("gfx2d0_pclk",		GFX2D0_P_CLK,		NULL, OFF),
	CLK_8X60("gfx2d1_pclk",		GFX2D1_P_CLK,		NULL, OFF),
	CLK_8X60("gfx3d_pclk",		GFX3D_P_CLK,		NULL, OFF),
	CLK_8X60("hdmi_m_pclk",		HDMI_M_P_CLK,		NULL, OFF),
	CLK_8X60("hdmi_s_pclk",		HDMI_S_P_CLK,		NULL, OFF),
	CLK_8X60("ijpeg_pclk",		IJPEG_P_CLK,		NULL, OFF),
	CLK_8X60("jpegd_pclk",		JPEGD_P_CLK,		NULL, OFF),
	CLK_8X60("imem_pclk",		IMEM_P_CLK,		NULL, OFF),
	CLK_8X60("mdp_pclk",		MDP_P_CLK,		NULL, OFF),
	CLK_8X60("smmu_pclk",		SMMU_P_CLK,		NULL, OFF),
	CLK_8X60("rotator_pclk",	ROT_P_CLK,		NULL, OFF),
	CLK_8X60("tv_enc_pclk",		TV_ENC_P_CLK,		NULL, OFF),
	CLK_8X60("vcodec_pclk",		VCODEC_P_CLK,		NULL, OFF),
	CLK_8X60("vfe_pclk",		VFE_P_CLK,		NULL, OFF),
	CLK_8X60("vpe_pclk",		VPE_P_CLK,		NULL, OFF),
	CLK_8X60("mi2s_osr_clk",	MI2S_OSR_CLK,		NULL, OFF),
	CLK_8X60("mi2s_bit_clk",	MI2S_BIT_CLK,		NULL, OFF),
	CLK_8X60("i2s_mic_osr_clk",	CODEC_I2S_MIC_OSR_CLK,	NULL, OFF),
	CLK_8X60("i2s_mic_bit_clk",	CODEC_I2S_MIC_BIT_CLK,	NULL, OFF),
	CLK_8X60("i2s_mic_osr_clk",	SPARE_I2S_MIC_OSR_CLK,	NULL, OFF),
	CLK_8X60("i2s_mic_bit_clk",	SPARE_I2S_MIC_BIT_CLK,	NULL, OFF),
	CLK_8X60("i2s_spkr_osr_clk",	CODEC_I2S_SPKR_OSR_CLK,	NULL, OFF),
	CLK_8X60("i2s_spkr_bit_clk",	CODEC_I2S_SPKR_BIT_CLK,	NULL, OFF),
	CLK_8X60("i2s_spkr_osr_clk",	SPARE_I2S_SPKR_OSR_CLK,	NULL, OFF),
	CLK_8X60("i2s_spkr_bit_clk",	SPARE_I2S_SPKR_BIT_CLK,	NULL, OFF),
	CLK_8X60("pcm_clk",		PCM_CLK,		NULL, OFF),
	CLK_8X60("iommu_clk",           JPEGD_AXI_CLK, "msm_iommu.0", 0),
	CLK_8X60("iommu_clk",           VFE_AXI_CLK, "msm_iommu.6", 0),
	CLK_8X60("iommu_clk",           VCODEC_AXI_CLK, "msm_iommu.7", 0),
	CLK_8X60("iommu_clk",           VCODEC_AXI_CLK, "msm_iommu.8", 0),
	CLK_8X60("iommu_clk",           GFX3D_CLK, "msm_iommu.9", 0),
	CLK_8X60("iommu_clk",           GFX2D0_CLK, "msm_iommu.10", 0),
	CLK_8X60("iommu_clk",           GFX2D1_CLK, "msm_iommu.11", 0),

	CLK_VOTER("dfab_dsps_clk",     DFAB_DSPS_CLK,
					"dfab_clk",    NULL, 0),
	CLK_VOTER("dfab_usb_hs_clk",   DFAB_USB_HS_CLK,
					"dfab_clk",    NULL, 0),
	CLK_VOTER("dfab_sdc_clk",      DFAB_SDC1_CLK,
					"dfab_clk",    "msm_sdcc.1", 0),
	CLK_VOTER("dfab_sdc_clk",      DFAB_SDC2_CLK,
					"dfab_clk",    "msm_sdcc.2", 0),
	CLK_VOTER("dfab_sdc_clk",      DFAB_SDC3_CLK,
					"dfab_clk",    "msm_sdcc.3", 0),
	CLK_VOTER("dfab_sdc_clk",      DFAB_SDC4_CLK,
					"dfab_clk",    "msm_sdcc.4", 0),
	CLK_VOTER("dfab_sdc_clk",      DFAB_SDC5_CLK,
					"dfab_clk",    "msm_sdcc.5", 0),
	CLK_VOTER("ebi1_msmbus_clk",   EBI_MSMBUS_CLK,
					"ebi1_clk",    NULL, 0),
	CLK_VOTER("ebi1_adm_clk",     EBI_ADM0_CLK,
					"ebi1_clk",    "msm_dmov.0", 0),
	CLK_VOTER("ebi1_adm_clk",     EBI_ADM1_CLK,
					"ebi1_clk",    "msm_dmov.1", 0),
};

unsigned msm_num_clocks_8x60 = ARRAY_SIZE(msm_clocks_8x60);
