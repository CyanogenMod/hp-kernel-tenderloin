/*
 * Copyright (C) 2008 Google, Inc.
 * Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/msm_rotator.h>
#include <linux/dma-mapping.h>
#include <mach/irqs.h>
#include <mach/msm_iomap.h>
#include <mach/dma.h>
#include <mach/board.h>

#include "devices.h"
#include "clock-7x30.h"
#include "gpio_hw.h"

#include <asm/mach/flash.h>

#include <asm/mach/mmc.h>
#include <mach/msm_hsusb.h>
#ifdef CONFIG_PMIC8058
#include <linux/mfd/pmic8058.h>
#endif

#include "msm7200a-gpio.h"

static struct resource resources_uart1[] = {
	{
		.start	= INT_UART1,
		.end	= INT_UART1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART1_PHYS,
		.end	= MSM_UART1_PHYS + MSM_UART1_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource resources_uart2[] = {
	{
		.start	= INT_UART2,
		.end	= INT_UART2,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART2_PHYS,
		.end	= MSM_UART2_PHYS + MSM_UART2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource resources_uart3[] = {
	{
		.start	= INT_UART3,
		.end	= INT_UART3,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART3_PHYS,
		.end	= MSM_UART3_PHYS + MSM_UART3_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device msm_device_uart1 = {
	.name	= "msm_serial",
	.id	= 0,
	.num_resources	= ARRAY_SIZE(resources_uart1),
	.resource	= resources_uart1,
};

struct platform_device msm_device_uart2 = {
	.name	= "msm_serial",
	.id	= 1,
	.num_resources	= ARRAY_SIZE(resources_uart2),
	.resource	= resources_uart2,
};

struct platform_device msm_device_uart3 = {
	.name	= "msm_serial",
	.id	= 2,
	.num_resources	= ARRAY_SIZE(resources_uart3),
	.resource	= resources_uart3,
};

#define MSM_UART1DM_PHYS      0xA3300000
#define MSM_UART2DM_PHYS      0xA3200000
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
		.start = INT_UART1DM_RX,
		.end   = INT_UART1DM_RX,
		.flags = IORESOURCE_IRQ,
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
	.name = "msm_serial_hs",
	.id = 0,
	.num_resources = ARRAY_SIZE(msm_uart1_dm_resources),
	.resource = msm_uart1_dm_resources,
	.dev		= {
		.dma_mask = &msm_uart_dm1_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

static struct resource msm_uart2_dm_resources[] = {
	{
		.start = MSM_UART2DM_PHYS,
		.end   = MSM_UART2DM_PHYS + PAGE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_UART2DM_IRQ,
		.end   = INT_UART2DM_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = INT_UART2DM_RX,
		.end   = INT_UART2DM_RX,
		.flags = IORESOURCE_IRQ,
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
	.name = "msm_serial_hs",
	.id = 1,
	.num_resources = ARRAY_SIZE(msm_uart2_dm_resources),
	.resource = msm_uart2_dm_resources,
	.dev		= {
		.dma_mask = &msm_uart_dm2_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

#define MSM_I2C_SIZE          SZ_4K
#define MSM_I2C_PHYS          0xACD00000
#define MSM_I2C_2_PHYS        0xACF00000
static struct resource resources_i2c_2[] = {
	{
		.start	= MSM_I2C_2_PHYS,
		.end	= MSM_I2C_2_PHYS + MSM_I2C_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_PWB_I2C_2,
		.end	= INT_PWB_I2C_2,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_i2c_2 = {
	.name		= "msm_i2c",
	.id		= 2,
	.num_resources	= ARRAY_SIZE(resources_i2c_2),
	.resource	= resources_i2c_2,
};

static struct resource resources_i2c[] = {
	{
		.start	= MSM_I2C_PHYS,
		.end	= MSM_I2C_PHYS + MSM_I2C_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_PWB_I2C,
		.end	= INT_PWB_I2C,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_i2c = {
	.name		= "msm_i2c",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(resources_i2c),
	.resource	= resources_i2c,
};

#define MSM_QUP_PHYS           0xA8301000
#define MSM_GSBI_QUP_I2C_PHYS  0xA8300000
#define MSM_QUP_SIZE           SZ_4K
static struct resource resources_qup[] = {
	{
		.name   = "qup_phys_addr",
		.start	= MSM_QUP_PHYS,
		.end	= MSM_QUP_PHYS + MSM_QUP_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI_QUP_I2C_PHYS,
		.end	= MSM_GSBI_QUP_I2C_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "qup_in_intr",
		.start	= INT_PWB_QUP_IN,
		.end	= INT_PWB_QUP_IN,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "qup_out_intr",
		.start	= INT_PWB_QUP_OUT,
		.end	= INT_PWB_QUP_OUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "qup_err_intr",
		.start	= INT_PWB_QUP_ERR,
		.end	= INT_PWB_QUP_ERR,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device qup_device_i2c = {
	.name		= "qup_i2c",
	.id		= 4,
	.num_resources	= ARRAY_SIZE(resources_qup),
	.resource	= resources_qup,
};

#ifdef CONFIG_I2C_SSBI
#define MSM_SSBI6_PHYS	0xAD900000
static struct resource msm_ssbi6_resources[] = {
	{
		.name   = "ssbi_base",
		.start	= MSM_SSBI6_PHYS,
		.end	= MSM_SSBI6_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device msm_device_ssbi6 = {
	.name		= "i2c_ssbi",
	.id		= 6,
	.num_resources	= ARRAY_SIZE(msm_ssbi6_resources),
	.resource	= msm_ssbi6_resources,
};

#define MSM_SSBI7_PHYS  0xAC800000
static struct resource msm_ssbi7_resources[] = {
	{
		.name   = "ssbi_base",
		.start  = MSM_SSBI7_PHYS,
		.end    = MSM_SSBI7_PHYS + SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
};

struct platform_device msm_device_ssbi7 = {
	.name		= "i2c_ssbi",
	.id		= 7,
	.num_resources	= ARRAY_SIZE(msm_ssbi7_resources),
	.resource	= msm_ssbi7_resources,
};
#endif /* CONFIG_I2C_SSBI */

#define MSM_HSUSB_PHYS        0xA3600000
static struct resource resources_hsusb_otg[] = {
	{
		.start	= MSM_HSUSB_PHYS,
		.end	= MSM_HSUSB_PHYS + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_USB_HS,
		.end	= INT_USB_HS,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 dma_mask = 0xffffffffULL;
struct platform_device msm_device_hsusb_otg = {
	.name		= "msm_hsusb_otg",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_hsusb_otg),
	.resource	= resources_hsusb_otg,
	.dev		= {
		.dma_mask 		= &dma_mask,
		.coherent_dma_mask	= 0xffffffffULL,
	},
};

static struct resource resources_hsusb_peripheral[] = {
	{
		.start	= MSM_HSUSB_PHYS,
		.end	= MSM_HSUSB_PHYS + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_USB_HS,
		.end	= INT_USB_HS,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource resources_gadget_peripheral[] = {
	{
		.start	= MSM_HSUSB_PHYS,
		.end	= MSM_HSUSB_PHYS + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_USB_HS,
		.end	= INT_USB_HS,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_hsusb_peripheral = {
	.name		= "msm_hsusb_peripheral",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_hsusb_peripheral),
	.resource	= resources_hsusb_peripheral,
	.dev		= {
		.dma_mask 		= &dma_mask,
		.coherent_dma_mask	= 0xffffffffULL,
	},
};

struct platform_device msm_device_gadget_peripheral = {
	.name		= "msm_hsusb",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_gadget_peripheral),
	.resource	= resources_gadget_peripheral,
	.dev		= {
		.dma_mask 		= &dma_mask,
		.coherent_dma_mask	= 0xffffffffULL,
	},
};

static struct resource resources_hsusb_host[] = {
	{
		.start	= MSM_HSUSB_PHYS,
		.end	= MSM_HSUSB_PHYS + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_USB_HS,
		.end	= INT_USB_HS,
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

#define MSM_NAND_PHYS		0xA0200000
#define MSM_NANDC01_PHYS	0xA0240000
#define MSM_NANDC10_PHYS	0xA0280000
#define MSM_NANDC11_PHYS	0xA02C0000
#define EBI2_REG_BASE		0xA0000000
static struct resource resources_nand[] = {
	[0] = {
		.name   = "msm_nand_dmac",
		.start	= DMOV_NAND_CHAN,
		.end	= DMOV_NAND_CHAN,
		.flags	= IORESOURCE_DMA,
	},
	[1] = {
		.name   = "msm_nand_phys",
		.start  = MSM_NAND_PHYS,
		.end    = MSM_NAND_PHYS + 0x7FF,
		.flags  = IORESOURCE_MEM,
	},
	[2] = {
		.name   = "msm_nandc01_phys",
		.start  = MSM_NANDC01_PHYS,
		.end    = MSM_NANDC01_PHYS + 0x7FF,
		.flags  = IORESOURCE_MEM,
	},
	[3] = {
		.name   = "msm_nandc10_phys",
		.start  = MSM_NANDC10_PHYS,
		.end    = MSM_NANDC10_PHYS + 0x7FF,
		.flags  = IORESOURCE_MEM,
	},
	[4] = {
		.name   = "msm_nandc11_phys",
		.start  = MSM_NANDC11_PHYS,
		.end    = MSM_NANDC11_PHYS + 0x7FF,
		.flags  = IORESOURCE_MEM,
	},
	[5] = {
		.name   = "ebi2_reg_base",
		.start  = EBI2_REG_BASE,
		.end    = EBI2_REG_BASE + 0x60,
		.flags  = IORESOURCE_MEM,
	},
};

static struct resource resources_otg[] = {
	{
		.start	= MSM_HSUSB_PHYS,
		.end	= MSM_HSUSB_PHYS + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_USB_HS,
		.end	= INT_USB_HS,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "vbus_on",
		.start	= PM8058_CHGVAL_IRQ(PMIC8058_IRQ_BASE),
		.end	= PM8058_CHGVAL_IRQ(PMIC8058_IRQ_BASE),
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_otg = {
	.name		= "msm_otg",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_otg),
	.resource	= resources_otg,
	.dev		= {
		.coherent_dma_mask	= 0xffffffffULL,
	},
};

struct flash_platform_data msm_nand_data = {
	.parts		= NULL,
	.nr_parts	= 0,
	.interleave     = 0,
};

struct platform_device msm_device_nand = {
	.name		= "msm_nand",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_nand),
	.resource	= resources_nand,
	.dev		= {
		.platform_data	= &msm_nand_data,
	},
};

struct platform_device msm_device_smd = {
	.name	= "msm_smd",
	.id	= -1,
};

struct platform_device msm_device_dmov = {
	.name	= "msm_dmov",
	.id	= -1,
};

#define MSM_SDC1_BASE         0xA0400000
#define MSM_SDC2_BASE         0xA0500000
#define MSM_SDC3_BASE         0xA3000000
#define MSM_SDC4_BASE         0xA3100000
static struct resource resources_sdc1[] = {
	{
		.start	= MSM_SDC1_BASE,
		.end	= MSM_SDC1_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SDC1_0,
		.end	= INT_SDC1_1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= 8,
		.end	= 8,
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
		.start	= INT_SDC2_0,
		.end	= INT_SDC2_1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= 8,
		.end	= 8,
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
		.start	= INT_SDC3_0,
		.end	= INT_SDC3_1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= 8,
		.end	= 8,
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
		.start	= INT_SDC4_0,
		.end	= INT_SDC4_1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= 8,
		.end	= 8,
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

static struct platform_device *msm_sdcc_devices[] __initdata = {
	&msm_device_sdc1,
	&msm_device_sdc2,
	&msm_device_sdc3,
	&msm_device_sdc4,
};

int __init msm_add_sdcc(unsigned int controller, struct mmc_platform_data *plat)
{
	struct platform_device	*pdev;

	if (controller < 1 || controller > 4)
		return -EINVAL;

	pdev = msm_sdcc_devices[controller-1];
	pdev->dev.platform_data = plat;
	return platform_device_register(pdev);
}

static struct resource msm_vidc_720p_resources[] = {
	{
		.start	= 0xA3B00000,
		.end	= 0xA3B00000 + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_MFC720,
		.end	= INT_MFC720,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_vidc_720p = {
	.name = "msm_vidc",
	.id = 0,
	.num_resources = ARRAY_SIZE(msm_vidc_720p_resources),
	.resource = msm_vidc_720p_resources,
};

#if defined(CONFIG_FB_MSM_MDP40)
#define MDP_BASE          0xA3F00000
#define PMDH_BASE         0xAD600000
#define EMDH_BASE         0xAD700000
#define TVENC_BASE        0xAD400000
#else
#define MDP_BASE          0xAA200000
#define PMDH_BASE         0xAA600000
#define EMDH_BASE         0xAA700000
#define TVENC_BASE        0xAA400000
#endif

static struct resource msm_mdp_resources[] = {
	{
		.name   = "mdp",
		.start  = MDP_BASE,
		.end    = MDP_BASE + 0x000F0000 - 1,
		.flags  = IORESOURCE_MEM,
	}
};

static struct resource msm_mddi_resources[] = {
	{
		.name   = "pmdh",
		.start  = PMDH_BASE,
		.end    = PMDH_BASE + PAGE_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	}
};

static struct resource msm_mddi_ext_resources[] = {
	{
		.name   = "emdh",
		.start  = EMDH_BASE,
		.end    = EMDH_BASE + PAGE_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	}
};

static struct resource msm_ebi2_lcd_resources[] = {
	{
		.name   = "base",
		.start  = 0xa0d00000,
		.end    = 0xa0d00000 + PAGE_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "lcd01",
		.start  = 0x98000000,
		.end    = 0x98000000 + 0x80000 - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "lcd02",
		.start  = 0x9c000000,
		.end    = 0x9c000000 + 0x80000 - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct resource msm_tvenc_resources[] = {
	{
		.name   = "tvenc",
		.start  = TVENC_BASE,
		.end    = TVENC_BASE + PAGE_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	}
};

static struct platform_device msm_mdp_device = {
	.name   = "mdp",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_mdp_resources),
	.resource       = msm_mdp_resources,
};

static struct platform_device msm_mddi_device = {
	.name   = "mddi",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_mddi_resources),
	.resource       = msm_mddi_resources,
};

static struct platform_device msm_mddi_ext_device = {
	.name   = "mddi_ext",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_mddi_ext_resources),
	.resource       = msm_mddi_ext_resources,
};

static struct platform_device msm_ebi2_lcd_device = {
	.name   = "ebi2_lcd",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_ebi2_lcd_resources),
	.resource       = msm_ebi2_lcd_resources,
};

static struct platform_device msm_lcdc_device = {
	.name   = "lcdc",
	.id     = 0,
};

static struct platform_device msm_dtv_device = {
	.name   = "dtv",
	.id     = 0,
};

static struct platform_device msm_tvenc_device = {
	.name   = "tvenc",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_tvenc_resources),
	.resource       = msm_tvenc_resources,
};

/* TSIF begin */
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)

#define MSM_TSIF_PHYS        (0xa3400000)
#define MSM_TSIF_SIZE        (0x200)

static struct resource tsif_resources[] = {
	[0] = {
		.flags = IORESOURCE_IRQ,
		.start = INT_TSIF,
		.end   = INT_TSIF,
	},
	[1] = {
		.flags = IORESOURCE_MEM,
		.start = MSM_TSIF_PHYS,
		.end   = MSM_TSIF_PHYS + MSM_TSIF_SIZE - 1,
	},
	[2] = {
		.flags = IORESOURCE_DMA,
		.start = DMOV_TSIF_CHAN,
		.end   = DMOV_TSIF_CRCI,
	},
};

static void tsif_release(struct device *dev)
{
	dev_info(dev, "release\n");
}

struct platform_device msm_device_tsif = {
	.name          = "msm_tsif",
	.id            = 0,
	.num_resources = ARRAY_SIZE(tsif_resources),
	.resource      = tsif_resources,
	.dev = {
		.release       = tsif_release,
	},
};
#endif /* defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE) */
/* TSIF end   */



#ifdef CONFIG_MSM_ROTATOR
static struct resource resources_msm_rotator[] = {
	{
		.start	= 0xA3E00000,
		.end	= 0xA3F00000 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_ROTATOR,
		.end	= INT_ROTATOR,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct msm_rot_clocks rotator_clocks[] = {
	{
		.clk_name = "rotator_clk",
		.clk_type = ROTATOR_AXI_CLK,
		.clk_rate = 0,
	},
	{
		.clk_name = "rotator_pclk",
		.clk_type = ROTATOR_PCLK,
		.clk_rate = 0,
	},
	{
		.clk_name = "rotator_imem_clk",
		.clk_type = ROTATOR_IMEM_CLK,
		.clk_rate = 0,
	},
};

static struct msm_rotator_platform_data rotator_pdata = {
	.number_of_clocks = ARRAY_SIZE(rotator_clocks),
	.hardware_version_number = 0x1000303,
	.rotator_clks = rotator_clocks,
};

struct platform_device msm_rotator_device = {
	.name		= "msm_rotator",
	.id		= 0,
	.num_resources  = ARRAY_SIZE(resources_msm_rotator),
	.resource       = resources_msm_rotator,
	.dev = {
		.platform_data = &rotator_pdata,
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

void __init msm_fb_register_device(char *name, void *data)
{
	if (!strncmp(name, "mdp", 3))
		msm_register_device(&msm_mdp_device, data);
	else if (!strncmp(name, "pmdh", 4))
		msm_register_device(&msm_mddi_device, data);
	else if (!strncmp(name, "emdh", 4))
		msm_register_device(&msm_mddi_ext_device, data);
	else if (!strncmp(name, "ebi2", 4))
		msm_register_device(&msm_ebi2_lcd_device, data);
	else if (!strncmp(name, "tvenc", 5))
		msm_register_device(&msm_tvenc_device, data);
	else if (!strncmp(name, "lcdc", 4))
		msm_register_device(&msm_lcdc_device, data);
	else if (!strncmp(name, "dtv", 3))
		msm_register_device(&msm_dtv_device, data);
	else
		printk(KERN_ERR "%s: unknown device! %s\n", __func__, name);
}

static struct platform_device msm_camera_device = {
	.name	= "msm_camera",
	.id	= 0,
};

void __init msm_camera_register_device(void *res, uint32_t num,
	void *data)
{
	msm_camera_device.num_resources = num;
	msm_camera_device.resource = res;

	msm_register_device(&msm_camera_device, data);
}

struct clk msm_clocks_7x30[] = {
	CLK_PCOM("adsp_clk",	ADSP_CLK,	NULL, 0),
	CLK_PCOM("codec_ssbi_clk",	CODEC_SSBI_CLK,	NULL, 0),
	CLK_PCOM("ebi1_clk",	EBI1_CLK,	NULL, CLK_MIN),
	CLK_PCOM("ecodec_clk",	ECODEC_CLK,	NULL, 0),
	CLK_PCOM("gp_clk",	GP_CLK,		NULL, 0),
	CLK_PCOM("uart_clk",	UART3_CLK,	&msm_device_uart3.dev, OFF),
	CLK_PCOM("usb_phy_clk",	USB_PHY_CLK,	NULL, 0),
	CLK_PCOM("vdc_clk",	VDC_CLK,	NULL, OFF | CLK_MIN),
	CLK_PCOM("pbus_clk",	PBUS_CLK,	NULL, CLK_MIN),

	CLK_7X30("adm_clk",	ADM_CLK,	NULL, 0),
	CLK_7X30("cam_m_clk",	CAM_M_CLK,	NULL, 0),
	CLK_7X30("camif_pad_pclk",	CAMIF_PAD_P_CLK,	NULL, OFF),
	CLK_7X30("emdh_clk",	EMDH_CLK,	NULL, OFF | CLK_MINMAX),
	CLK_7X30("emdh_pclk",	EMDH_P_CLK,	NULL, OFF),
	CLK_7X30("grp_2d_clk",	GRP_2D_CLK,	NULL, 0),
	CLK_7X30("grp_2d_pclk",	GRP_2D_P_CLK,	NULL, 0),
	CLK_7X30("grp_clk",	GRP_3D_CLK,	NULL, 0),
	CLK_7X30("grp_pclk",	GRP_3D_P_CLK,	NULL, 0),
	CLK_7X30S("grp_src_clk", GRP_3D_SRC_CLK, GRP_3D_CLK,	NULL, 0),
	CLK_7X30("hdmi_clk",	HDMI_CLK,	NULL, 0),
	CLK_7X30("i2c_clk",	I2C_CLK,	&msm_device_i2c.dev, 0),
	CLK_7X30("i2c_clk",	I2C_2_CLK,	&msm_device_i2c_2.dev, 0),
	CLK_7X30("imem_clk",	IMEM_CLK,	NULL, OFF),
	CLK_7X30("jpeg_clk",	JPEG_CLK,	NULL, OFF),
	CLK_7X30("jpeg_pclk",	JPEG_P_CLK,	NULL, OFF),
	CLK_7X30("lpa_codec_clk",	LPA_CODEC_CLK,		NULL, 0),
	CLK_7X30("lpa_core_clk",	LPA_CORE_CLK,		NULL, 0),
	CLK_7X30("lpa_pclk",		LPA_P_CLK,		NULL, 0),
	CLK_7X30("mdc_clk",	MDC_CLK,	NULL, 0),
	CLK_7X30("mddi_clk",	PMDH_CLK,	NULL, OFF | CLK_MINMAX),
	CLK_7X30("mddi_pclk",	PMDH_P_CLK,	NULL, 0),
	CLK_7X30("mdp_clk",	MDP_CLK,	NULL, OFF),
	CLK_7X30("mdp_pclk",	MDP_P_CLK,	NULL, 0),
	CLK_7X30("mdp_lcdc_pclk_clk", MDP_LCDC_PCLK_CLK, NULL, 0),
	CLK_7X30("mdp_lcdc_pad_pclk_clk", MDP_LCDC_PAD_PCLK_CLK, NULL, 0),
	CLK_7X30("mdp_vsync_clk",	MDP_VSYNC_CLK,  NULL, OFF),
	CLK_7X30("mfc_clk",		MFC_CLK,		NULL, 0),
	CLK_7X30("mfc_div2_clk",	MFC_DIV2_CLK,		NULL, 0),
	CLK_7X30("mfc_pclk",		MFC_P_CLK,		NULL, 0),
	CLK_7X30("mi2s_codec_rx_m_clk",	MI2S_CODEC_RX_M_CLK,  NULL, 0),
	CLK_7X30("mi2s_codec_rx_s_clk",	MI2S_CODEC_RX_S_CLK,  NULL, 0),
	CLK_7X30("mi2s_codec_tx_m_clk",	MI2S_CODEC_TX_M_CLK,  NULL, 0),
	CLK_7X30("mi2s_codec_tx_s_clk",	MI2S_CODEC_TX_S_CLK,  NULL, 0),
	CLK_7X30("mi2s_m_clk",		MI2S_M_CLK,  		NULL, 0),
	CLK_7X30("mi2s_s_clk",		MI2S_S_CLK,  		NULL, 0),
	CLK_7X30("qup_clk",	QUP_I2C_CLK,	&qup_device_i2c.dev, 0),
	CLK_7X30("rotator_clk",	AXI_ROTATOR_CLK,		NULL, 0),
	CLK_7X30("rotator_imem_clk",	ROTATOR_IMEM_CLK,	NULL, OFF),
	CLK_7X30("rotator_pclk",	ROTATOR_P_CLK,		NULL, OFF),
	CLK_7X30("sdac_clk",	SDAC_CLK,	NULL, OFF),
	CLK_7X30("sdc_clk",	SDC1_CLK,	&msm_device_sdc1.dev, OFF),
	CLK_7X30("sdc_pclk",	SDC1_P_CLK,	&msm_device_sdc1.dev, OFF),
	CLK_7X30("sdc_clk",	SDC2_CLK,	&msm_device_sdc2.dev, OFF),
	CLK_7X30("sdc_pclk",	SDC2_P_CLK,	&msm_device_sdc2.dev, OFF),
	CLK_7X30("sdc_clk",	SDC3_CLK,	&msm_device_sdc3.dev, OFF),
	CLK_7X30("sdc_pclk",	SDC3_P_CLK,	&msm_device_sdc3.dev, OFF),
	CLK_7X30("sdc_clk",	SDC4_CLK,	&msm_device_sdc4.dev, OFF),
	CLK_7X30("sdc_pclk",	SDC4_P_CLK,	&msm_device_sdc4.dev, OFF),
	CLK_7X30("spi_clk",	SPI_CLK,	NULL, 0),
	CLK_7X30("spi_pclk",	SPI_P_CLK,	NULL, 0),
	CLK_7X30("tsif_ref_clk", TSIF_REF_CLK,	NULL, 0),
	CLK_7X30("tsif_pclk",	TSIF_P_CLK,	NULL, 0),
	CLK_7X30("tv_dac_clk",	TV_DAC_CLK,	NULL, 0),
	CLK_7X30("tv_enc_clk",	TV_ENC_CLK,	NULL, 0),
	CLK_7X30S("tv_src_clk",	TV_CLK, 	TV_ENC_CLK,	NULL, 0),
	CLK_7X30("uart_clk",	UART1_CLK,	&msm_device_uart1.dev, OFF),
	CLK_7X30("uart_clk",	UART2_CLK,	&msm_device_uart2.dev, 0),
	CLK_7X30("uartdm_clk",	UART1DM_CLK,	&msm_device_uart_dm1.dev, OFF),
	CLK_7X30("uartdm_clk",	UART2DM_CLK,	&msm_device_uart_dm2.dev, 0),
	CLK_7X30("usb_hs_clk",		USB_HS_CLK,		NULL, OFF),
	CLK_7X30("usb_hs_pclk",		USB_HS_P_CLK,		NULL, OFF),
	CLK_7X30("usb_hs_core_clk",	USB_HS_CORE_CLK,	NULL, OFF),
	CLK_7X30("usb_hs2_clk",		USB_HS2_CLK,		NULL, OFF),
	CLK_7X30("usb_hs2_pclk",	USB_HS2_P_CLK,		NULL, OFF),
	CLK_7X30("usb_hs2_core_clk",	USB_HS2_CORE_CLK,	NULL, OFF),
	CLK_7X30("usb_hs3_clk",		USB_HS3_CLK,		NULL, OFF),
	CLK_7X30("usb_hs3_pclk",	USB_HS3_P_CLK,		NULL, OFF),
	CLK_7X30("usb_hs3_core_clk",	USB_HS3_CORE_CLK,	NULL, OFF),
	CLK_7X30("vfe_camif_clk",	VFE_CAMIF_CLK, 	NULL, 0),
	CLK_7X30("vfe_clk",	VFE_CLK,	NULL, 0),
	CLK_7X30("vfe_mdc_clk",	VFE_MDC_CLK,	NULL, 0),
	CLK_7X30("vfe_pclk",	VFE_P_CLK,	NULL, OFF),
	CLK_7X30("vpe_clk",	VPE_CLK,	NULL, 0),

	/* 7x30 v2 hardware only. */
	CLK_7X30("csi_clk",	CSI0_CLK,	NULL, 0),
	CLK_7X30("csi_pclk",	CSI0_P_CLK,	NULL, 0),
	CLK_7X30("csi_vfe_clk",	CSI0_VFE_CLK,	NULL, 0),
};

unsigned msm_num_clocks_7x30 = ARRAY_SIZE(msm_clocks_7x30);

#ifdef CONFIG_GPIOLIB
static struct msm7200a_gpio_platform_data gpio_platform_data[] = {
	MSM7200A_GPIO_PLATFORM_DATA(0,   0,  15, INT_GPIO_GROUP1),
	MSM7200A_GPIO_PLATFORM_DATA(1,  16,  43, INT_GPIO_GROUP2),
	MSM7200A_GPIO_PLATFORM_DATA(2,  44,  67, INT_GPIO_GROUP1),
	MSM7200A_GPIO_PLATFORM_DATA(3,  68,  94, INT_GPIO_GROUP1),
	MSM7200A_GPIO_PLATFORM_DATA(4,  95, 106, INT_GPIO_GROUP1),
	MSM7200A_GPIO_PLATFORM_DATA(5, 107, 133, INT_GPIO_GROUP1),
	MSM7200A_GPIO_PLATFORM_DATA(6, 134, 150, INT_GPIO_GROUP1),
	MSM7200A_GPIO_PLATFORM_DATA(7, 151, 181, INT_GPIO_GROUP1),
};

struct platform_device msm_gpio_devices[] = {
	MSM7200A_GPIO_DEVICE(0, gpio_platform_data),
	MSM7200A_GPIO_DEVICE(1, gpio_platform_data),
	MSM7200A_GPIO_DEVICE(2, gpio_platform_data),
	MSM7200A_GPIO_DEVICE(3, gpio_platform_data),
	MSM7200A_GPIO_DEVICE(4, gpio_platform_data),
	MSM7200A_GPIO_DEVICE(5, gpio_platform_data),
	MSM7200A_GPIO_DEVICE(6, gpio_platform_data),
	MSM7200A_GPIO_DEVICE(7, gpio_platform_data),
};
#endif

