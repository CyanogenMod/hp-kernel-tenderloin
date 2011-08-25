/* Copyright (c) 2008-2011, Code Aurora Forum. All rights reserved.
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
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/android_pmem.h>
#include <linux/bootmem.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/mfd/tps65023.h>
#include <linux/bma150.h>
#include <linux/power_supply.h>
#include <linux/clk.h>
#include <linux/gpio_keys.h>

#include <linux/input/qci_kbd.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/io.h>
#include <asm/setup.h>

#include <asm/mach/mmc.h>
#include <mach/vreg.h>
#include <mach/mpp.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/sirc.h>
#include <mach/dma.h>
#include <mach/rpc_hsusb.h>
#include <mach/msm_hsusb.h>
#include <mach/msm_hsusb_hw.h>
#include <mach/msm_serial_hs.h>
#include <mach/msm_touchpad.h>
#include <mach/msm_i2ckbd.h>
#include <mach/pmic.h>
#include <mach/camera.h>
#include <mach/memory.h>
#include <mach/msm_spi.h>
#include <mach/msm_tsif.h>
#include <mach/msm_battery.h>
#include <mach/clk.h>
#include <mach/tpm_st_i2c.h>
#include <mach/rpc_server_handset.h>
#include <mach/socinfo.h>

#include "devices.h"
#include "timer.h"
#include "msm-keypad-devices.h"
#include "pm.h"
#include <linux/msm_kgsl.h>
#include <linux/smsc911x.h>
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android_composite.h>
#endif
#include "smd_private.h"

#define MSM_PMEM_MDP_SIZE	0x408000

#define SMEM_SPINLOCK_I2C	"D:I2C02000021"

#define MSM_PMEM_ADSP_SIZE	0x2A05000

#define MSM_FB_SIZE_ST15	0x810000
#define MSM_AUDIO_SIZE		0x80000

#define MSM_SMI_BASE		0xE0000000

#define MSM_SHARED_RAM_PHYS	(MSM_SMI_BASE + 0x00100000)

#define MSM_PMEM_SMI_BASE	(MSM_SMI_BASE + 0x02B00000)
#define MSM_PMEM_SMI_SIZE	0x01500000

#define MSM_FB_BASE		MSM_PMEM_SMI_BASE
#define MSM_PMEM_SMIPOOL_BASE	(MSM_FB_BASE + MSM_FB_SIZE_ST15)
#define MSM_PMEM_SMIPOOL_SIZE	(MSM_PMEM_SMI_SIZE - MSM_FB_SIZE_ST15)

#define PMEM_KERNEL_EBI1_SIZE	(10 * 1024 * 1024)

#define PMIC_VREG_GP6_LEVEL	2900

#define MSM_GPIO_SD_DET        100

static struct resource smsc911x_resources[] = {
	[0] = {
		.name	= "smsc911x-memory",
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.flags  = IORESOURCE_IRQ,
	},
};

static struct smsc911x_platform_config smsc911x_config = {
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type	= SMSC911X_IRQ_TYPE_OPEN_DRAIN,
	.flags		= SMSC911X_USE_32BIT,
	.phy_interface	= PHY_INTERFACE_MODE_MII,
};

static struct platform_device smsc911x_device = {
	.name           = "smsc911x",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(smsc911x_resources),
	.resource       = smsc911x_resources,
	.dev		= {
		.platform_data		= &smsc911x_config,
	},
};

static struct resource smc91x_resources[] = {
	[0] = {
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.flags  = IORESOURCE_IRQ,
	},
};

static struct clk *hs_clk;
static struct clk *phy_clk;
#ifdef CONFIG_USB_FUNCTION
static struct usb_mass_storage_platform_data usb_mass_storage_pdata = {
	.nluns          = 0x02,
	.buf_size       = 16384,
	.vendor         = "GOOGLE",
	.product        = "Mass storage",
	.release        = 0xffff,
};

static struct platform_device mass_storage_device = {
	.name           = "usb_mass_storage",
	.id             = -1,
	.dev            = {
		.platform_data          = &usb_mass_storage_pdata,
	},
};
#endif

#ifdef CONFIG_USB_ANDROID
static char *usb_functions_default[] = {
	"diag",
	"modem",
	"nmea",
	"rmnet",
	"usb_mass_storage",
};

static char *usb_functions_default_adb[] = {
	"diag",
	"adb",
	"modem",
	"nmea",
	"rmnet",
	"usb_mass_storage",
};

static char *usb_functions_rndis[] = {
	"rndis",
	"usb_mass_storage",
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
	"usb_mass_storage",
};

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_DIAG
	"diag",
#endif
	"adb",
#ifdef CONFIG_USB_F_SERIAL
	"modem",
	"nmea",
#endif
#ifdef CONFIG_USB_ANDROID_RMNET
	"rmnet",
#endif
	"usb_mass_storage",
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
};

static struct android_usb_product usb_products[] = {
	{
		.product_id	= 0x9026,
		.num_functions	= ARRAY_SIZE(usb_functions_default),
		.functions	= usb_functions_default,
	},
	{
		.product_id	= 0x9025,
		.num_functions	= ARRAY_SIZE(usb_functions_default_adb),
		.functions	= usb_functions_default_adb,
	},
	{
		.product_id	= 0xf00e,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
	{
		.product_id	= 0x9024,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
	},
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "Qualcomm Incorporated",
	.product        = "Mass storage",
	.release	= 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

static struct usb_ether_platform_data rndis_pdata = {
	/* ethaddr is filled by board_serialno_setup */
	.vendorID	= 0x05C6,
	.vendorDescr	= "Qualcomm Incorporated",
};

static struct platform_device rndis_device = {
	.name	= "rndis",
	.id	= -1,
	.dev	= {
		.platform_data = &rndis_pdata,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x05C6,
	.product_id	= 0x9026,
	.version	= 0x0100,
	.product_name		= "Qualcomm HSUSB Device",
	.manufacturer_name	= "Qualcomm Incorporated",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
	.serial_number = "1234567890ABCDEF",
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};

static int __init board_serialno_setup(char *serialno)
{
	int i;
	char *src = serialno;

	/* create a fake MAC address from our serial number.
	 * first byte is 0x02 to signify locally administered.
	 */
	rndis_pdata.ethaddr[0] = 0x02;
	for (i = 0; *src; i++) {
		/* XOR the USB serial across the remaining bytes */
		rndis_pdata.ethaddr[i % (ETH_ALEN - 1) + 1] ^= *src++;
	}

	android_usb_pdata.serial_number = serialno;
	return 1;
}
__setup("androidboot.serialno=", board_serialno_setup);
#endif

static struct platform_device smc91x_device = {
	.name           = "smc91x",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(smc91x_resources),
	.resource       = smc91x_resources,
};

#ifdef CONFIG_USB_FUNCTION
static struct usb_function_map usb_functions_map[] = {
	{"diag", 0},
	{"adb", 1},
	{"modem", 2},
	{"nmea", 3},
	{"mass_storage", 4},
	{"ethernet", 5},
};

/* dynamic composition */
static struct usb_composition usb_func_composition[] = {
	{
		.product_id         = 0x9012,
		.functions	    = 0x5, /* 0101 */
	},

	{
		.product_id         = 0x9013,
		.functions	    = 0x15, /* 10101 */
	},

	{
		.product_id         = 0x9014,
		.functions	    = 0x30, /* 110000 */
	},

	{
		.product_id         = 0x9015,
		.functions	    = 0x12, /* 10010 */
	},

	{
		.product_id         = 0x9016,
		.functions	    = 0xD, /* 01101 */
	},

	{
		.product_id         = 0x9017,
		.functions	    = 0x1D, /* 11101 */
	},

	{
		.product_id         = 0xF000,
		.functions	    = 0x10, /* 10000 */
	},

	{
		.product_id         = 0xF009,
		.functions	    = 0x20, /* 100000 */
	},

	{
		.product_id         = 0x9018,
		.functions	    = 0x1F, /* 011111 */
	},

	{
		.product_id         = 0x901A,
		.functions	    = 0x0F, /* 01111 */
	},
};
#endif

static struct msm_handset_platform_data hs_platform_data = {
	.hs_name = "8k_handset",
	.pwr_key_delay_ms = 500, /* 0 will disable end key */
};

static struct platform_device hs_device = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = &hs_platform_data,
	},
};

#define HUB_3V3_GPIO		38		/* Power to HUB and switch */
#define SWITCH_EN_GPIO		98		/* !CS of analog switch */
#define SWITCH_CONTROL_GPIO	104		/* 0: Host, 1: Peripheral */
#define HUB_RESET_GPIO		108		/* 0: HUB is RESET */

static struct msm_gpio hsusb_gpio_config_data[] = {
	{ GPIO_CFG(HUB_3V3_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
						GPIO_CFG_2MA), "hub_power" },
	{ GPIO_CFG(SWITCH_EN_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
						GPIO_CFG_2MA), "swch_enable" },
	{ GPIO_CFG(SWITCH_CONTROL_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
						GPIO_CFG_2MA), "swch_ctrl" },
	{ GPIO_CFG(HUB_RESET_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
						GPIO_CFG_2MA), "hub_reset" },
};

static int msm_otg_gpio_init(void)
{
	int rc;

	rc = msm_gpios_request_enable(hsusb_gpio_config_data,
		ARRAY_SIZE(hsusb_gpio_config_data));

	if (rc)
		printk(KERN_ERR "Error gpio req for hsusb\n");
	return rc;
}

static void msm_otg_setup_gpio(enum usb_switch_control mode)
{
	switch (mode) {
	case USB_SWITCH_HOST:
		gpio_set_value(HUB_3V3_GPIO, 1);
		/* Configure analog switch as USB host. */
		gpio_set_value(SWITCH_EN_GPIO, 0);
		gpio_set_value(SWITCH_CONTROL_GPIO, 0);
		/* Bring HUB out of RESET */
		gpio_set_value(HUB_RESET_GPIO, 1);
		break;

	case USB_SWITCH_PERIPHERAL:
		/* Power-up switch */
		gpio_set_value(HUB_3V3_GPIO, 1);
		/* Configure analog switch as USB peripheral. */
		gpio_set_value(SWITCH_EN_GPIO, 0);
		gpio_set_value(SWITCH_CONTROL_GPIO, 1);
		break;

	case USB_SWITCH_DISABLE:
	default:
		/* Disable Switch */
		gpio_set_value(SWITCH_EN_GPIO, 1);
		gpio_set_value(HUB_RESET_GPIO, 0);
		/* Power-down HUB and analog switch */
		gpio_set_value(HUB_3V3_GPIO, 0);
	}
}

#ifdef CONFIG_USB_FS_HOST
static struct msm_gpio fsusb_config[] = {
	{ GPIO_CFG(139, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "fs_dat" },
	{ GPIO_CFG(140, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "fs_se0" },
	{ GPIO_CFG(141, 3, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "fs_oe_n" },
};

static int fsusb_gpio_init(void)
{
	return msm_gpios_request(fsusb_config, ARRAY_SIZE(fsusb_config));
}

static void msm_fsusb_setup_gpio(unsigned int enable)
{
	if (enable)
		msm_gpios_enable(fsusb_config, ARRAY_SIZE(fsusb_config));
	else
		msm_gpios_disable(fsusb_config, ARRAY_SIZE(fsusb_config));

}
#endif

#define MSM_USB_BASE              ((unsigned)addr)
static unsigned ulpi_read(void __iomem *addr, unsigned reg)
{
	unsigned timeout = 100000;

	/* initiate read operation */
	writel(ULPI_RUN | ULPI_READ | ULPI_ADDR(reg),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while ((readl(USB_ULPI_VIEWPORT) & ULPI_RUN) && (--timeout))
		cpu_relax();

	if (timeout == 0) {
		printk(KERN_ERR "ulpi_read: timeout %08x\n",
			readl(USB_ULPI_VIEWPORT));
		return 0xffffffff;
	}
	return ULPI_DATA_READ(readl(USB_ULPI_VIEWPORT));
}

static int ulpi_write(void __iomem *addr, unsigned val, unsigned reg)
{
	unsigned timeout = 10000;

	/* initiate write operation */
	writel(ULPI_RUN | ULPI_WRITE |
	       ULPI_ADDR(reg) | ULPI_DATA(val),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while ((readl(USB_ULPI_VIEWPORT) & ULPI_RUN) && (--timeout))
		cpu_relax();

	if (timeout == 0) {
		printk(KERN_ERR "ulpi_write: timeout\n");
		return -1;
	}

	return 0;
}

static void msm_hsusb_apps_reset_link(int reset)
{
	if (reset)
		clk_reset(hs_clk, CLK_RESET_ASSERT);
	else
		clk_reset(hs_clk, CLK_RESET_DEASSERT);
}

static void msm_hsusb_apps_reset_phy(void)
{
	clk_reset(phy_clk, CLK_RESET_ASSERT);
	msleep(1);
	clk_reset(phy_clk, CLK_RESET_DEASSERT);
}

#define ULPI_VERIFY_MAX_LOOP_COUNT  3
static int msm_hsusb_phy_verify_access(void __iomem *addr)
{
	int temp;

	for (temp = 0; temp < ULPI_VERIFY_MAX_LOOP_COUNT; temp++) {
		if (ulpi_read(addr, ULPI_DEBUG) != (unsigned)-1)
			break;
		msm_hsusb_apps_reset_phy();
	}

	if (temp == ULPI_VERIFY_MAX_LOOP_COUNT) {
		pr_err("%s: ulpi read failed for %d times\n",
				__func__, ULPI_VERIFY_MAX_LOOP_COUNT);
		return -1;
	}

	return 0;
}

static unsigned msm_hsusb_ulpi_read_with_reset(void __iomem *addr, unsigned reg)
{
	int temp;
	unsigned res;

	for (temp = 0; temp < ULPI_VERIFY_MAX_LOOP_COUNT; temp++) {
		res = ulpi_read(addr, reg);
		if (res != -1)
			return res;
		msm_hsusb_apps_reset_phy();
	}

	pr_err("%s: ulpi read failed for %d times\n",
			__func__, ULPI_VERIFY_MAX_LOOP_COUNT);

	return -1;
}

static int msm_hsusb_ulpi_write_with_reset(void __iomem *addr,
		unsigned val, unsigned reg)
{
	int temp;
	int res;

	for (temp = 0; temp < ULPI_VERIFY_MAX_LOOP_COUNT; temp++) {
		res = ulpi_write(addr, val, reg);
		if (!res)
			return 0;
		msm_hsusb_apps_reset_phy();
	}

	pr_err("%s: ulpi write failed for %d times\n",
			__func__, ULPI_VERIFY_MAX_LOOP_COUNT);
	return -1;
}

static int msm_hsusb_phy_caliberate(void __iomem *addr)
{
	int ret;
	unsigned res;

	ret = msm_hsusb_phy_verify_access(addr);
	if (ret)
		return -ETIMEDOUT;

	res = msm_hsusb_ulpi_read_with_reset(addr, ULPI_FUNC_CTRL_CLR);
	if (res == -1)
		return -ETIMEDOUT;

	res = msm_hsusb_ulpi_write_with_reset(addr,
			res | ULPI_SUSPENDM,
			ULPI_FUNC_CTRL_CLR);
	if (res)
		return -ETIMEDOUT;

	msm_hsusb_apps_reset_phy();

	return msm_hsusb_phy_verify_access(addr);
}

#define USB_LINK_RESET_TIMEOUT      (msecs_to_jiffies(10))
static int msm_hsusb_native_phy_reset(void __iomem *addr)
{
	u32 temp;
	unsigned long timeout;

	msm_hsusb_apps_reset_link(1);
	msm_hsusb_apps_reset_phy();
	msm_hsusb_apps_reset_link(0);

	/* select ULPI phy */
	temp = (readl(USB_PORTSC) & ~PORTSC_PTS);
	writel(temp | PORTSC_PTS_ULPI, USB_PORTSC);

	if (msm_hsusb_phy_caliberate(addr))
		return -1;

	/* soft reset phy */
	writel(USBCMD_RESET, USB_USBCMD);
	timeout = jiffies + USB_LINK_RESET_TIMEOUT;
	while (readl(USB_USBCMD) & USBCMD_RESET) {
		if (time_after(jiffies, timeout)) {
			pr_err("usb link reset timeout\n");
			break;
		}
		msleep(1);
	}

	return 0;
}

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
#ifdef CONFIG_USB_FUNCTION
	.version	= 0x0100,
	.phy_info	= (USB_PHY_INTEGRATED | USB_PHY_MODEL_180NM),
	.vendor_id          = 0x5c6,
	.product_name       = "Qualcomm HSUSB Device",
	.serial_number      = "1234567890ABCDEF",
	.manufacturer_name  = "Qualcomm Incorporated",
	.compositions	= usb_func_composition,
	.num_compositions = ARRAY_SIZE(usb_func_composition),
	.function_map   = usb_functions_map,
	.num_functions	= ARRAY_SIZE(usb_functions_map),
	.config_gpio    = NULL,

	.phy_reset = msm_hsusb_native_phy_reset,
#endif
};

static struct vreg *vreg_usb;
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{

	switch (PHY_TYPE(phy_info)) {
	case USB_PHY_INTEGRATED:
		if (on)
			msm_hsusb_vbus_powerup();
		else
			msm_hsusb_vbus_shutdown();
		break;
	case USB_PHY_SERIAL_PMIC:
		if (on)
			vreg_enable(vreg_usb);
		else
			vreg_disable(vreg_usb);
		break;
	default:
		pr_err("%s: undefined phy type ( %X ) \n", __func__,
						phy_info);
	}

}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
	.phy_info	= (USB_PHY_INTEGRATED | USB_PHY_MODEL_180NM),
	.vbus_power = msm_hsusb_vbus_power,
};

#ifdef CONFIG_USB_FS_HOST
static struct msm_usb_host_platform_data msm_usb_host2_pdata = {
	.phy_info	= USB_PHY_SERIAL_PMIC,
	.config_gpio = msm_fsusb_setup_gpio,
	.vbus_power = msm_hsusb_vbus_power,
};
#endif

static struct android_pmem_platform_data android_pmem_kernel_ebi1_pdata = {
	.name = PMEM_KERNEL_EBI1_DATA_NAME,
	/* if no allocator_type, defaults to PMEM_ALLOCATORTYPE_BITMAP,
	 * the only valid choice at this time. The board structure is
	 * set to all zeros by the C runtime initialization and that is now
	 * the enum value of PMEM_ALLOCATORTYPE_BITMAP, now forced to 0 in
	 * include/linux/android_pmem.h.
	 */
	.cached = 0,
};

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
static struct android_pmem_platform_data android_pmem_kernel_smi_pdata = {
	.name = PMEM_KERNEL_SMI_DATA_NAME,
	/* if no allocator_type, defaults to PMEM_ALLOCATORTYPE_BITMAP,
	 * the only valid choice at this time. The board structure is
	 * set to all zeros by the C runtime initialization and that is now
	 * the enum value of PMEM_ALLOCATORTYPE_BITMAP, now forced to 0 in
	 * include/linux/android_pmem.h.
	 */
	.cached = 0,
};
#else
static struct android_pmem_platform_data android_pmem_smipool_pdata = {
	.name = "pmem_smipool",
	.start = MSM_PMEM_SMIPOOL_BASE,
	.size = MSM_PMEM_SMIPOOL_SIZE,
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};
#endif

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};


static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct platform_device android_pmem_kernel_ebi1_device = {
	.name = "android_pmem",
	.id = 3,
	.dev = { .platform_data = &android_pmem_kernel_ebi1_pdata },
};

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
static struct platform_device android_pmem_kernel_smi_device = {
	.name = "android_pmem",
	.id = 4,
	.dev = { .platform_data = &android_pmem_kernel_smi_pdata },
};
#else
static struct platform_device android_pmem_smipool_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_smipool_pdata },
};
#endif

static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static int msm_fb_detect_panel(const char *name)
{
	int ret;

	if (!strcmp(name, "lcdc_st15") ||
	    !strcmp(name, "hdmi_sii9022"))
		ret = 0;
	else
		ret = -ENODEV;

	return ret;
}

/* Only allow a small subset of machines to set the offset via
   FB PAN_DISPLAY */

static int msm_fb_allow_set_offset(void)
{
	return 1;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
	.allow_set_offset = msm_fb_allow_set_offset,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
	.dev    = {
		.platform_data = &msm_fb_pdata,
	}
};

static struct msm_gpio bma_spi_gpio_config_data[] = {
	{ GPIO_CFG(22, 0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "bma_irq" },
};

static int msm_bma_gpio_setup(struct device *dev)
{
	int rc;

	rc = msm_gpios_request_enable(bma_spi_gpio_config_data,
		ARRAY_SIZE(bma_spi_gpio_config_data));

	return rc;
}

static void msm_bma_gpio_teardown(struct device *dev)
{
	msm_gpios_disable_free(bma_spi_gpio_config_data,
		ARRAY_SIZE(bma_spi_gpio_config_data));
}

static struct bma150_platform_data bma_pdata = {
	.setup    = msm_bma_gpio_setup,
	.teardown = msm_bma_gpio_teardown,
};

static struct resource qsd_spi_resources[] = {
	{
		.name   = "spi_irq_in",
		.start	= INT_SPI_INPUT,
		.end	= INT_SPI_INPUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_out",
		.start	= INT_SPI_OUTPUT,
		.end	= INT_SPI_OUTPUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_err",
		.start	= INT_SPI_ERROR,
		.end	= INT_SPI_ERROR,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_base",
		.start	= 0xA1200000,
		.end	= 0xA1200000 + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "spidm_channels",
		.flags  = IORESOURCE_DMA,
	},
	{
		.name   = "spidm_crci",
		.flags  = IORESOURCE_DMA,
	},
};

static struct platform_device qsd_device_spi = {
	.name	        = "spi_qsd",
	.id	        = 0,
	.num_resources	= ARRAY_SIZE(qsd_spi_resources),
	.resource	= qsd_spi_resources,
};

static struct spi_board_info msm_spi_board_info[] __initdata = {
	{
		.modalias	= "bma150",
		.mode		= SPI_MODE_3,
		.irq		= MSM_GPIO_TO_INT(22),
		.bus_num	= 0,
		.chip_select	= 0,
		.max_speed_hz	= 10000000,
		.platform_data	= &bma_pdata,
	},
};

#define CT_CSR_PHYS		0xA8700000
#define TCSR_SPI_MUX		(ct_csr_base + 0x54)
static int msm_qsd_spi_dma_config(void)
{
	void __iomem *ct_csr_base = 0;
	u32 spi_mux;
	int ret = 0;

	ct_csr_base = ioremap(CT_CSR_PHYS, PAGE_SIZE);
	if (!ct_csr_base) {
		pr_err("%s: Could not remap %x\n", __func__, CT_CSR_PHYS);
		return -1;
	}

	spi_mux = readl(TCSR_SPI_MUX);
	switch (spi_mux) {
	case (1):
		qsd_spi_resources[4].start  = DMOV_HSUART1_RX_CHAN;
		qsd_spi_resources[4].end    = DMOV_HSUART1_TX_CHAN;
		qsd_spi_resources[5].start  = DMOV_HSUART1_RX_CRCI;
		qsd_spi_resources[5].end    = DMOV_HSUART1_TX_CRCI;
		break;
	case (2):
		qsd_spi_resources[4].start  = DMOV_HSUART2_RX_CHAN;
		qsd_spi_resources[4].end    = DMOV_HSUART2_TX_CHAN;
		qsd_spi_resources[5].start  = DMOV_HSUART2_RX_CRCI;
		qsd_spi_resources[5].end    = DMOV_HSUART2_TX_CRCI;
		break;
	case (3):
		qsd_spi_resources[4].start  = DMOV_CE_OUT_CHAN;
		qsd_spi_resources[4].end    = DMOV_CE_IN_CHAN;
		qsd_spi_resources[5].start  = DMOV_CE_OUT_CRCI;
		qsd_spi_resources[5].end    = DMOV_CE_IN_CRCI;
		break;
	default:
		ret = -1;
	}

	iounmap(ct_csr_base);
	return ret;
}

static struct msm_gpio qsd_spi_gpio_config_data[] = {
	{ GPIO_CFG(17, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,
		       GPIO_CFG_2MA), "spi_clk" },
	{ GPIO_CFG(18, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,
			GPIO_CFG_2MA), "spi_mosi" },
	{ GPIO_CFG(19, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,
			GPIO_CFG_2MA), "spi_miso" },
	{ GPIO_CFG(20, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,
			GPIO_CFG_2MA), "spi_cs0" },
	{ GPIO_CFG(21, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
			GPIO_CFG_16MA), "spi_pwr" },
};

static int msm_qsd_spi_gpio_config(void)
{
	int rc;

	rc = msm_gpios_request_enable(qsd_spi_gpio_config_data,
		ARRAY_SIZE(qsd_spi_gpio_config_data));
	if (rc)
		return rc;

	/* Set direction for SPI_PWR */
	gpio_direction_output(21, 1);

	return 0;
}

static void msm_qsd_spi_gpio_release(void)
{
	msm_gpios_disable_free(qsd_spi_gpio_config_data,
		ARRAY_SIZE(qsd_spi_gpio_config_data));
}

static struct msm_spi_platform_data qsd_spi_pdata = {
	.max_clock_speed = 19200000,
	.gpio_config  = msm_qsd_spi_gpio_config,
	.gpio_release = msm_qsd_spi_gpio_release,
	.dma_config = msm_qsd_spi_dma_config,
};

static void __init msm_qsd_spi_init(void)
{
	qsd_device_spi.dev.platform_data = &qsd_spi_pdata;
}

static int mddi_toshiba_pmic_bl(int level)
{
	return -EPERM;
}

static struct msm_panel_common_pdata mddi_toshiba_pdata = {
	.pmic_backlight = mddi_toshiba_pmic_bl,
};

static struct platform_device mddi_toshiba_device = {
	.name   = "mddi_toshiba",
	.id     = 0,
	.dev    = {
		.platform_data = &mddi_toshiba_pdata,
	}
};

static void msm_fb_vreg_config(const char *name, int on)
{
	struct vreg *vreg;
	int ret = 0;

	vreg = vreg_get(NULL, name);
	if (IS_ERR(vreg)) {
		printk(KERN_ERR "%s: vreg_get(%s) failed (%ld)\n",
		__func__, name, PTR_ERR(vreg));
		return;
	}

	ret = (on) ? vreg_enable(vreg) : vreg_disable(vreg);
	if (ret)
		printk(KERN_ERR "%s: %s(%s) failed!\n",
			__func__, (on) ? "vreg_enable" : "vreg_disable", name);
}

#define MDDI_RST_OUT_GPIO 100

static int mddi_power_save_on;
static int msm_fb_mddi_power_save(int on)
{
	int ret;
	int flag_on = !!on;

	if (mddi_power_save_on == flag_on)
		return 0;

	mddi_power_save_on = flag_on;

	ret = pmic_lp_mode_control(flag_on ? OFF_CMD : ON_CMD,
		PM_VREG_LP_MSME2_ID);
	if (ret)
		printk(KERN_ERR "%s: pmic_lp_mode_control failed!\n", __func__);

	msm_fb_vreg_config("gp5", flag_on);
	msm_fb_vreg_config("boost", flag_on);
	return ret;
}

static int msm_fb_mddi_sel_clk(u32 *clk_rate)
{
	*clk_rate *= 2;
	return 0;
}

static struct mddi_platform_data mddi_pdata = {
	.mddi_power_save = msm_fb_mddi_power_save,
	.mddi_sel_clk = msm_fb_mddi_sel_clk,
};

static struct vreg *vreg_msme2;
static int __init st15_hdmi_vreg_init(void)
{
	int rc;

	vreg_msme2 = vreg_get(NULL, "msme2");

	if (IS_ERR(vreg_msme2)) {
		pr_err("%s: msme2 vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_msme2));
		return -EINVAL;
	}

	rc = vreg_set_level(vreg_msme2, 1200);
	if (rc) {
		pr_err("%s: vreg msme2 set level failed (%d)\n",
		       __func__, rc);
		return rc;
	}
	return 0;
}

static int st15_hdmi_power(int on)
{
	int rc;

	if (on)
		rc = vreg_enable(vreg_msme2);
	else {
		rc = vreg_disable(vreg_msme2);
		msleep(30);
	}

	if (rc) {
		pr_err("%s: msme2 vreg %s failed (%d)\n",
		       __func__, on ? "enable" : "disable", rc);
		return rc;
	}

	return 0;
}

static unsigned int msm_fb_lcdc_get_clk(void)
{
	/* Return 160MHz(in Hz) as the AXI clock for st1x device */
	return 192000000;
}

static int msm_fb_lcdc_gpio_config(int on)
{

	if (on) {
		gpio_set_value(17, 1);
		gpio_set_value(19, 1);
		gpio_set_value(20, 1);
		gpio_set_value(22, 0);
		gpio_set_value(32, 1);
		gpio_set_value(155, 1);
		st15_hdmi_power(1);
		gpio_set_value(22, 1);
		udelay(100);
		/* reset pulse */
		gpio_set_value(22, 0);
		udelay(50);
		gpio_set_value(22, 1);
	} else {
		gpio_set_value(17, 0);
		gpio_set_value(19, 0);
		gpio_set_value(20, 0);
		gpio_set_value(22, 0);
		gpio_set_value(32, 0);
		gpio_set_value(155, 0);
		st15_hdmi_power(0);
	}
	return 0;
}

static struct msm_gpio msm_fb_st15_gpio_config_data[] = {
	{ GPIO_CFG(17, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_en0" },
	{ GPIO_CFG(19, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "dat_pwr_sv" },
	{ GPIO_CFG(20, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lvds_pwr_dn" },
	{ GPIO_CFG(22, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_en1" },
	{ GPIO_CFG(32, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_en2" },
	{ GPIO_CFG(103, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "hdmi_irq" },
	{ GPIO_CFG(155, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "hdmi_3v3" },
};

static struct lcdc_platform_data lcdc_pdata = {
	.lcdc_gpio_config = msm_fb_lcdc_gpio_config,
	.lcdc_get_clk = msm_fb_lcdc_get_clk,
};

static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = 98,
};

#define LID_SENSOR_GPIO		41

static struct gpio_keys_button gpio_keys_buttons[] = {
	{
		.code           = SW_LID,
		.gpio           = LID_SENSOR_GPIO,
		.desc           = "Lid",
		.active_low     = 1,
		.type		= EV_SW,
		.wakeup		= 1
	},
};

static struct gpio_keys_platform_data gpio_keys_data = {
	.buttons        = gpio_keys_buttons,
	.nbuttons       = ARRAY_SIZE(gpio_keys_buttons),
	.rep		= 0,
};

static struct platform_device msm_gpio_keys = {
	.name           = "gpio-keys",
	.id             = -1,
	.dev            = {
		.platform_data  = &gpio_keys_data,
	},
};

static void __init lid_sensor_gpio_init(void)
{
	if (gpio_tlmm_config(GPIO_CFG(LID_SENSOR_GPIO, 0, GPIO_CFG_INPUT,
		GPIO_CFG_PULL_UP, GPIO_CFG_6MA), GPIO_CFG_ENABLE)) {
		pr_err("%s: gpio_tlmm_config for gpio=%d failed", __func__,
						LID_SENSOR_GPIO);
	}
}

static void __init msm_fb_add_devices(void)
{
	int rc;
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("pmdh", &mddi_pdata);
	msm_fb_register_device("emdh", &mddi_pdata);
	msm_fb_register_device("tvenc", 0);

	rc = st15_hdmi_vreg_init();
	if (rc)
		return;
	rc = msm_gpios_request_enable(
		msm_fb_st15_gpio_config_data,
		ARRAY_SIZE(msm_fb_st15_gpio_config_data));
	if (rc) {
		printk(KERN_ERR "%s: unable to init lcdc gpios\n",
		       __func__);
		return;
	}
	msm_fb_register_device("lcdc", &lcdc_pdata);
}

static struct resource msm_audio_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	},
	{
		.name   = "aux_pcm_dout",
		.start  = 68,
		.end    = 68,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_din",
		.start  = 69,
		.end    = 69,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_syncout",
		.start  = 70,
		.end    = 70,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_clkin_a",
		.start  = 71,
		.end    = 71,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "sdac_din",
		.start  = 144,
		.end    = 144,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "sdac_dout",
		.start  = 145,
		.end    = 145,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "sdac_wsout",
		.start  = 143,
		.end    = 143,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "cc_i2s_clk",
		.start  = 142,
		.end    = 142,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "audio_master_clkout",
		.start  = 146,
		.end    = 146,
		.flags  = IORESOURCE_IO,
	},
	{
		.name	= "audio_base_addr",
		.start	= 0xa0700000,
		.end	= 0xa0700000 + 4,
		.flags	= IORESOURCE_MEM,
	},

};

static unsigned st15_audio_gpio_on[] = {
	/* enable headset amplifier */
	GPIO_CFG(48, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/* enable speaker amplifier */
	GPIO_CFG(39, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static unsigned audio_gpio_on[] = {
	GPIO_CFG(68, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* PCM_DOUT */
	GPIO_CFG(69, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* PCM_DIN */
	GPIO_CFG(70, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* PCM_SYNC */
	GPIO_CFG(71, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* PCM_CLK */
	GPIO_CFG(142, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* CC_I2S_CLK */
	GPIO_CFG(143, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* SADC_WSOUT */
	GPIO_CFG(145, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* SDAC_DOUT */
	GPIO_CFG(146, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* MA_CLK_OUT */
};

void q6audio_enable_spkr_phone(int enable)
{
	gpio_set_value(48, (enable != 0));
	pmic_secure_mpp_control_digital_output(
			PM_MPP_21,
			PM_MPP__DLOGIC__LVL_VDD,
			PM_MPP__DLOGIC_OUT__CTRL_HIGH);
}

static void __init audio_gpio_init(void)
{
	int pin, rc;

	for (pin = 0; pin < ARRAY_SIZE(audio_gpio_on); pin++) {
		rc = gpio_tlmm_config(audio_gpio_on[pin],
			GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR
				"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, audio_gpio_on[pin], rc);
			return;
		}
	}

	/* enable headset amplifier */
	gpio_tlmm_config(st15_audio_gpio_on[0], GPIO_CFG_ENABLE);
	/* enable speaker amplifier */
	gpio_tlmm_config(st15_audio_gpio_on[1], GPIO_CFG_ENABLE);
}

static struct platform_device msm_audio_device = {
	.name   = "msm_audio",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_audio_resources),
	.resource       = msm_audio_resources,
};

static struct resource bluesleep_resources[] = {
	{
		.name	= "gpio_host_wake",
		.start	= 21,
		.end	= 21,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "gpio_ext_wake",
		.start	= 19,
		.end	= 19,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "host_wake",
		.start	= MSM_GPIO_TO_INT(21),
		.end	= MSM_GPIO_TO_INT(21),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_bluesleep_device = {
	.name = "bluesleep",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(bluesleep_resources),
	.resource	= bluesleep_resources,
};

#ifdef CONFIG_BT
static struct platform_device msm_bt_power_device = {
	.name = "bt_power",
};

enum {
	BT_SYSRST,
	BT_WAKE,
	BT_HOST_WAKE,
	BT_VDD_IO,
	BT_RFR,
	BT_CTS,
	BT_RX,
	BT_TX,
	BT_VDD_FREG
};

static struct msm_gpio st1_5_bt_config_power_on[] = {
	{ GPIO_CFG(18, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"BT SYSRST" },
	{ GPIO_CFG(29, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"BT WAKE" },
	{ GPIO_CFG(40, 0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"HOST WAKE" },
	{ GPIO_CFG(22, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"BT VDD_IO" },
	{ GPIO_CFG(43, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"UART1DM_RFR" },
	{ GPIO_CFG(44, 2, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"UART1DM_CTS" },
	{ GPIO_CFG(45, 2, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"UART1DM_RX" },
	{ GPIO_CFG(46, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"UART1DM_TX" },
};

static int bluetooth_power_st_1_5(int on)
{
	gpio_set_value(18, on); /* SYSRST */
	printk(KERN_DEBUG "Bluetooth power switch: %d\n", on);
	return 0;
}

static void __init bt_power_init_st_1_5(void)
{
	int rc;

	gpio_set_value(18, 0); /* SYSRST */

	rc = msm_gpios_enable(st1_5_bt_config_power_on,
		ARRAY_SIZE(st1_5_bt_config_power_on));

	if (rc < 0) {
		printk(KERN_ERR
			"%s: bt power on gpio config failed: %d\n",
			__func__, rc);
		return;
	}

	msm_bt_power_device.dev.platform_data = &bluetooth_power_st_1_5;

	printk(KERN_DEBUG "Bluetooth power switch ST-1.5: initialized\n");

	return;
}
#else
#define bt_power_init_st_1_5(x) do {} while (0)
#endif

static struct resource kgsl_resources[] = {
	{
		.name  = "kgsl_reg_memory",
		.start = 0xA0000000,
		.end = 0xA001ffff,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "kgsl_yamato_irq",
		.start = INT_GRAPHICS,
		.end = INT_GRAPHICS,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "kgsl_2d0_reg_memory",
		.start = 0xA1300000,
		.end =  0xA13fffff,
		.flags = IORESOURCE_MEM,
	},
	{
	       .name = "kgsl_2d0_irq",
	       .start = INT_GRP2D,
	       .end = INT_GRP2D,
	       .flags = IORESOURCE_IRQ,
	},
};
static struct kgsl_platform_data kgsl_pdata = {
	.high_axi_3d = 128000, /* Max for 8K */
	.max_grp2d_freq = 0,
	.min_grp2d_freq = 0,
	.set_grp2d_async = NULL,
	.max_grp3d_freq = 235*1000*1000,
	.min_grp3d_freq = 192*1000*1000,
	/*note: on 8650a async mode is the default */
	.set_grp3d_async = NULL,
	.imem_clk_name = "imem_clk",
	.grp3d_clk_name = "grp_clk",
	.grp3d_pclk_name = "grp_pclk",
#ifdef CONFIG_MSM_KGSL_2D
	.grp2d0_clk_name = "grp_2d_clk",
	.grp2d0_pclk_name = "grp_2d_pclk",
#else
	.grp2d0_clk_name = NULL,
#endif
	.idle_timeout_3d = HZ/5,
	.idle_timeout_2d = HZ/10,
#ifdef CONFIG_KGSL_PER_PROCESS_PAGE_TABLE
	.pt_va_size = SZ_32M,
#else
	.pt_va_size = SZ_128M,
#endif
};

static struct platform_device msm_device_kgsl = {
	.name = "kgsl",
	.id = -1,
	.num_resources = ARRAY_SIZE(kgsl_resources),
	.resource = kgsl_resources,
	.dev = {
		.platform_data = &kgsl_pdata,
	},
};

static struct platform_device msm_device_pmic_leds = {
	.name	= "pmic-leds",
	.id	= -1,
};

#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)

#define TSIF_A_SYNC      GPIO_CFG(106, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_A_DATA      GPIO_CFG(107, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_A_EN        GPIO_CFG(108, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_A_CLK       GPIO_CFG(109, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)

static const struct msm_gpio tsif_gpios[] = {
	{ .gpio_cfg = TSIF_A_CLK,  .label =  "tsif_clk", },
	{ .gpio_cfg = TSIF_A_EN,   .label =  "tsif_en", },
	{ .gpio_cfg = TSIF_A_DATA, .label =  "tsif_data", },
	{ .gpio_cfg = TSIF_A_SYNC, .label =  "tsif_sync", },
};

static struct msm_tsif_platform_data tsif_platform_data = {
	.num_gpios = ARRAY_SIZE(tsif_gpios),
	.gpios = tsif_gpios,
};

#endif /* CONFIG_TSIF || CONFIG_TSIF_MODULE */

#ifdef CONFIG_QSD_SVS
#define TPS65023_MAX_DCDC1	1600
#else
#define TPS65023_MAX_DCDC1	CONFIG_QSD_PMIC_DEFAULT_DCDC1
#endif

static int qsd8x50_tps65023_set_dcdc1(int mVolts)
{
	int rc = 0;
#ifdef CONFIG_QSD_SVS
	rc = tps65023_set_dcdc1_level(mVolts);
	/* By default the TPS65023 will be initialized to 1.225V.
	 * So we can safely switch to any frequency within this
	 * voltage even if the device is not probed/ready.
	 */
	if (rc == -ENODEV && mVolts <= CONFIG_QSD_PMIC_DEFAULT_DCDC1)
		rc = 0;
#else
	/* Disallow frequencies not supported in the default PMIC
	 * output voltage.
	 */
	if (mVolts > CONFIG_QSD_PMIC_DEFAULT_DCDC1)
		rc = -EFAULT;
#endif
	return rc;
}

static struct msm_acpu_clock_platform_data qsd8x50_clock_data = {
	.acpu_switch_time_us = 20,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.max_vdd = TPS65023_MAX_DCDC1,
	.acpu_set_vdd = qsd8x50_tps65023_set_dcdc1,
};

static struct msm_gpio qup_i2c_gpios_io[] = {
	{ GPIO_CFG(154, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_scl" },
	{ GPIO_CFG(156, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_sda" },
};
static struct msm_gpio qup_i2c_gpios_hw[] = {
	{ GPIO_CFG(154, 4, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_scl" },
	{ GPIO_CFG(156, 3, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_sda" },
};

static void
qup_i2c_gpio_config(int adap_id, int config_type)
{
	int rc = 0;
	struct msm_gpio *qup_i2c_table;
	/* Each adapter gets 2 lines from the table */
	if (adap_id != 4)
		return;
	if (config_type)
		qup_i2c_table = qup_i2c_gpios_hw;
	else
		qup_i2c_table = qup_i2c_gpios_io;
	rc = msm_gpios_enable(qup_i2c_table, 2);
	if (rc < 0)
		printk(KERN_ERR "QUP GPIO enable failed: %d\n", rc);
}

static struct msm_i2c_platform_data qup_i2c_pdata = {
	.clk_freq = 100000,
	.clk = "qup_clk",
	.pclk = "qup_pclk",
	.msm_i2c_config_gpio = qup_i2c_gpio_config,
};

static void __init qup_device_i2c_init(void)
{
	if (msm_gpios_request(qup_i2c_gpios_hw, ARRAY_SIZE(qup_i2c_gpios_hw)))
		pr_err("failed to request I2C gpios\n");

	qup_device_i2c.dev.platform_data = &qup_i2c_pdata;
}

#define TPM_ACCEPT_CMD_GPIO 85
#define TPM_DATA_AVAIL_GPIO 84
static struct msm_gpio tpm_gpio_config[] = {
	{ GPIO_CFG(TPM_ACCEPT_CMD_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"tpm_accept_cmd" },
	{ GPIO_CFG(TPM_DATA_AVAIL_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"tpm_data_avail" },
};

static void tpm_st_i2c_gpio_release(void)
{
	msm_gpios_disable_free(tpm_gpio_config, ARRAY_SIZE(tpm_gpio_config));
}

static int tpm_st_i2c_gpio_setup(void)
{
	return msm_gpios_request_enable(tpm_gpio_config,
					ARRAY_SIZE(tpm_gpio_config));
}

static struct tpm_st_i2c_platform_data tpm_st_i2c_data = {
	.accept_cmd_gpio = TPM_ACCEPT_CMD_GPIO,
	.data_avail_gpio = TPM_DATA_AVAIL_GPIO,
	.accept_cmd_irq = MSM_GPIO_TO_INT(TPM_ACCEPT_CMD_GPIO),
	.data_avail_irq = MSM_GPIO_TO_INT(TPM_DATA_AVAIL_GPIO),
	.gpio_setup = tpm_st_i2c_gpio_setup,
	.gpio_release = tpm_st_i2c_gpio_release,
};

static int hdmi_sii9022_cable_detect(int insert)
{
	/* turn off on-board & external VGA
	 * when HDMI is plugged in
	 */
	if (insert) {
		gpio_set_value(32, 0);
		gpio_set_value(19, 0);
	} else {
		gpio_set_value(32, 1);
		gpio_set_value(19, 1);
	}
	return 0;
}

static struct msm_hdmi_platform_data hdmi_sii9022_i2c_data = {
	.irq = MSM_GPIO_TO_INT(103),
	.cable_detect = hdmi_sii9022_cable_detect,
};

static struct qci_kbd_platform_data qci_i2ckbd_pdata = {
#ifdef CONFIG_KEYBOARD_QCIKBD_REPEAT
	.repeat = true,
#endif
};

static struct i2c_board_info msm_i2c_st1_5_info[] __initdata = {
	{
		I2C_BOARD_INFO("qci-i2ckbd", 0x18),
		.irq = 37,
		.platform_data = &qci_i2ckbd_pdata,
	},
	{
		I2C_BOARD_INFO("qci-i2cpad", 0x19),
		.irq = 35,
	},
	{
		I2C_BOARD_INFO("qci-i2cec", 0x1A),
		.irq = 42,
	},
	{
		I2C_BOARD_INFO("hmc5843", 0x1E),
	},
	{
		I2C_BOARD_INFO("bma150", 0x38),
	},
	{
		I2C_BOARD_INFO("isl29011", 0x44),
	},
	{
		I2C_BOARD_INFO("tps65023", 0x48),
	},
};

static struct i2c_board_info msm_qup_st1_5_info[] __initdata = {
	{
		I2C_BOARD_INFO("sii9022", 0x72 >> 1),
		.platform_data = &hdmi_sii9022_i2c_data,
	},
	{
		I2C_BOARD_INFO("tpm_st_i2c", 0x13),
		.platform_data = &tpm_st_i2c_data,
	},
};

static u32 msm_calculate_batt_capacity(u32 current_voltage);

static struct msm_psy_batt_pdata msm_psy_batt_data = {
	.voltage_min_design 	= 3200,
	.voltage_max_design	= 4200,
	.avail_chg_sources   	= AC_CHG | USB_CHG ,
	.batt_technology        = POWER_SUPPLY_TECHNOLOGY_LION,
	.calculate_capacity	= &msm_calculate_batt_capacity,
};

static u32 msm_calculate_batt_capacity(u32 current_voltage)
{
	u32 low_voltage   = msm_psy_batt_data.voltage_min_design;
	u32 high_voltage  = msm_psy_batt_data.voltage_max_design;

	return (current_voltage - low_voltage) * 100
		/ (high_voltage - low_voltage);
}

static struct platform_device msm_batt_device = {
	.name 		    = "msm-battery",
	.id		    = -1,
	.dev.platform_data  = &msm_psy_batt_data,
};

static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}

static struct vreg *vreg_3p3;
static int msm_hsusb_ldo_init(int init)
{
	if (init) {
		vreg_3p3 = vreg_get(NULL, "usb");
		if (IS_ERR(vreg_3p3))
			return PTR_ERR(vreg_3p3);
		vreg_set_level(vreg_3p3, 3300);
	} else
		vreg_put(vreg_3p3);

	return 0;
}

static int msm_hsusb_ldo_enable(int enable)
{
	static int ldo_status;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (ldo_status == enable)
		return 0;

	ldo_status = enable;

	if (enable)
		return vreg_enable(vreg_3p3);

	return vreg_disable(vreg_3p3);
}

static struct msm_otg_platform_data msm_otg_pdata = {
	.rpc_connect	= hsusb_rpc_connect,
	.phy_reset	= msm_hsusb_native_phy_reset,
	.setup_gpio 	= msm_otg_setup_gpio,
	.otg_mode	= OTG_USER_CONTROL,
	.usb_mode	= USB_PERIPHERAL_MODE,
	.vbus_power 	= msm_hsusb_vbus_power,
	.chg_vbus_draw  = hsusb_chg_vbus_draw,
	.chg_connected  = hsusb_chg_connected,
	.chg_init	= hsusb_chg_init,
	.ldo_enable	= msm_hsusb_ldo_enable,
	.ldo_init	= msm_hsusb_ldo_init,
	.pclk_src_name	= "ebi1_usb_clk",
};

static struct msm_hsusb_gadget_platform_data msm_gadget_pdata;

static struct platform_device *devices[] __initdata = {
	&msm_fb_device,
	&mddi_toshiba_device,
	&smc91x_device,
	&msm_device_smd,
	&msm_device_dmov,
	&android_pmem_kernel_ebi1_device,
#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
	&android_pmem_kernel_smi_device,
#else
	&android_pmem_smipool_device,
#endif
	&android_pmem_device,
	&android_pmem_adsp_device,
	&msm_device_nand,
	&msm_device_i2c,
	&qup_device_i2c,
	&qsd_device_spi,
	&msm_device_hsusb_peripheral,
	&msm_device_gadget_peripheral,
#ifdef CONFIG_USB_FUNCTION
	&mass_storage_device,
#endif
#ifdef CONFIG_USB_ANDROID
	&usb_mass_storage_device,
	&rndis_device,
#ifdef CONFIG_USB_ANDROID_DIAG
	&usb_diag_device,
#endif
	&android_usb_device,
#endif
	&msm_device_otg,
	&msm_device_tssc,
	&msm_audio_device,
	&msm_device_uart_dm1,
	&msm_bluesleep_device,
#ifdef CONFIG_BT
	&msm_bt_power_device,
#endif
#if !defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_device_uart3,
#endif
	&msm_device_pmic_leds,
	&msm_device_kgsl,
	&hs_device,
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
	&msm_device_tsif,
#endif
#ifdef CONFIG_MT9T013
	&msm_camera_sensor_mt9t013,
#endif
#ifdef CONFIG_MT9D112
	&msm_camera_sensor_mt9d112,
#endif
#ifdef CONFIG_S5K3E2FX
	&msm_camera_sensor_s5k3e2fx,
#endif
#ifdef CONFIG_MT9P012
	&msm_camera_sensor_mt9p012,
#endif
#ifdef CONFIG_MT9P012_KM
	&msm_camera_sensor_mt9p012_km,
#endif
	&msm_batt_device,
	&msm_gpio_keys,
};

static void __init qsd8x50_init_irq(void)
{
	msm_init_irq();
	msm_init_sirc();
}

static void __init qsd8x50_init_host(void)
{
	vreg_usb = vreg_get(NULL, "boost");

	if (IS_ERR(vreg_usb)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_usb));
		return;
	}

	platform_device_register(&msm_device_hsusb_otg);
	if (msm_otg_gpio_init())
		return;
	msm_add_host(0, &msm_usb_host_pdata);
#ifdef CONFIG_USB_FS_HOST
	if (fsusb_gpio_init())
		return;
	msm_add_host(1, &msm_usb_host2_pdata);
#endif
}

#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT))

struct sdcc_gpio {
	struct msm_gpio *cfg_data;
	uint32_t size;
	struct msm_gpio *sleep_cfg_data;
};

static struct msm_gpio sdc1_cfg_data[] = {
	{GPIO_CFG(51, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_3"},
	{GPIO_CFG(52, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_2"},
	{GPIO_CFG(53, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_1"},
	{GPIO_CFG(54, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_0"},
	{GPIO_CFG(55, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_cmd"},
	{GPIO_CFG(56, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc1_clk"},
};

static struct msm_gpio sdc1_sleep_cfg_data[] = {
	{GPIO_CFG(51, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"sdc1_dat_3"},
	{GPIO_CFG(52, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"sdc1_dat_2"},
	{GPIO_CFG(53, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"sdc1_dat_1"},
	{GPIO_CFG(54, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"sdc1_dat_0"},
	{GPIO_CFG(55, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"sdc1_cmd"},
	{GPIO_CFG(56, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"sdc1_clk"},
};

static struct msm_gpio sdc2_cfg_data[] = {
	{GPIO_CFG(62, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc2_clk"},
	{GPIO_CFG(63, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_cmd"},
	{GPIO_CFG(64, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_3"},
	{GPIO_CFG(65, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_2"},
	{GPIO_CFG(66, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_1"},
	{GPIO_CFG(67, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_0"},
};

static struct msm_gpio sdc3_cfg_data[] = {
	{GPIO_CFG(88, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc3_clk"},
	{GPIO_CFG(89, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_cmd"},
	{GPIO_CFG(90, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_3"},
	{GPIO_CFG(91, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_2"},
	{GPIO_CFG(92, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_1"},
	{GPIO_CFG(93, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_0"},

#ifdef CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT
	{GPIO_CFG(158, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_4"},
	{GPIO_CFG(159, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_5"},
	{GPIO_CFG(160, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_6"},
	{GPIO_CFG(161, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_7"},
#endif
};

static struct msm_gpio sdc4_cfg_data[] = {
	{GPIO_CFG(142, 3, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc4_clk"},
	{GPIO_CFG(143, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_cmd"},
	{GPIO_CFG(144, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_dat_0"},
	{GPIO_CFG(145, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_dat_1"},
	{GPIO_CFG(146, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_dat_2"},
	{GPIO_CFG(147, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_dat_3"},
};

static struct sdcc_gpio sdcc_cfg_data[] = {
	{
		.cfg_data = sdc1_cfg_data,
		.size = ARRAY_SIZE(sdc1_cfg_data),
		.sleep_cfg_data = sdc1_sleep_cfg_data,
	},
	{
		.cfg_data = sdc2_cfg_data,
		.size = ARRAY_SIZE(sdc2_cfg_data),
	},
	{
		.cfg_data = sdc3_cfg_data,
		.size = ARRAY_SIZE(sdc3_cfg_data),
	},
	{
		.cfg_data = sdc4_cfg_data,
		.size = ARRAY_SIZE(sdc4_cfg_data),
	},
};

static unsigned long vreg_sts, gpio_sts;
static struct vreg *vreg_mmc;
static struct vreg *vreg_movi;

static void msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_gpio *curr;

	curr = &sdcc_cfg_data[dev_id - 1];
	if (!(test_bit(dev_id, &gpio_sts)^enable))
		return;

	if (enable) {
		set_bit(dev_id, &gpio_sts);
		rc = msm_gpios_request_enable(curr->cfg_data, curr->size);
		if (rc)
			printk(KERN_ERR "%s: Failed to turn on GPIOs for slot %d\n",
				__func__,  dev_id);
	} else {
		clear_bit(dev_id, &gpio_sts);
		if (curr->sleep_cfg_data) {
			msm_gpios_enable(curr->sleep_cfg_data, curr->size);
			msm_gpios_free(curr->sleep_cfg_data, curr->size);
			return;
		}
		msm_gpios_disable_free(curr->cfg_data, curr->size);
	}
}

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);
	msm_sdcc_setup_gpio(pdev->id, !!vdd);

	if (vdd == 0) {
		if (!vreg_sts)
			return 0;

		clear_bit(pdev->id, &vreg_sts);

		if (!vreg_sts) {
			rc = vreg_disable(vreg_mmc);
			if (rc)
				printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
		}
		return 0;
	}

	if (!vreg_sts) {
		rc = vreg_set_level(vreg_mmc, PMIC_VREG_GP6_LEVEL);
		if (!rc)
			rc = vreg_enable(vreg_mmc);
		if (rc)
			printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
	}
	set_bit(pdev->id, &vreg_sts);
	return 0;
}


static int msm_sdcc_get_wpswitch(struct device *dv)
{
	return -1;
}

#if defined(CONFIG_MMC_MSM_CARD_HW_DETECTION)
static unsigned int st1_5_sdcc_slot_status(struct device *dev)
{
	return (unsigned int)
		!(gpio_get_value(MSM_GPIO_SD_DET));
}
#endif

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static struct mmc_platform_data qsd8x50_sdc1_data = {
	.ocr_mask	= MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.wpswitch	= msm_sdcc_get_wpswitch,
#ifdef CONFIG_MMC_MSM_SDC1_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
#if defined(CONFIG_MMC_MSM_CARD_HW_DETECTION)
	.status      = st1_5_sdcc_slot_status,
	.status_irq  = MSM_GPIO_TO_INT(MSM_GPIO_SD_DET),
	.irq_flags   = IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 25000000,
	.msmsdcc_fmax	= 40000000,
	.nonremovable	= 0,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static struct mmc_platform_data qsd8x50_sdc2_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.wpswitch	= msm_sdcc_get_wpswitch,
#ifdef CONFIG_MMC_MSM_SDC2_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 25000000,
	.msmsdcc_fmax	= 40000000,
	.nonremovable   = 0,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static struct mmc_platform_data qsd8x50_sdc3_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
#ifdef CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
#else
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#endif
#ifdef CONFIG_MMC_MSM_SDC3_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 25000000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable   = 1,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static struct mmc_platform_data qsd8x50_sdc4_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.wpswitch	= msm_sdcc_get_wpswitch,
#ifdef CONFIG_MMC_MSM_SDC4_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 25000000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable   = 0,
};
#endif

static void __init qsd8x50_init_mmc(void)
{
	int rc;
	vreg_mmc = vreg_get(NULL, "wlan");
	if (IS_ERR(vreg_mmc)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_mmc));
		return;
	}

	rc = vreg_disable(vreg_mmc);
	if (rc)
		printk(KERN_ERR "%s: vreg_disable(vreg_mmc) returned %d\n",
		       __func__, rc);
	/* TODO: Conflicts with BT. */
	vreg_movi = vreg_get(NULL, "wlan");
	if (IS_ERR(vreg_movi)) {
		printk(KERN_ERR "%s: vreg_get(mmc) failed (%ld)\n",
		       __func__, PTR_ERR(vreg_movi));
		return;
	}
	rc = vreg_disable(vreg_movi);
	if (rc)
		printk(KERN_ERR "%s: vreg_disable(vreg_movi) returned %d\n",
		       __func__, rc);
	mdelay(100);

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	msm_add_sdcc(1, &qsd8x50_sdc1_data);
#if defined(CONFIG_MMC_MSM_CARD_HW_DETECTION)
	if (!gpio_request(MSM_GPIO_SD_DET, "sd-det")) {
		if (gpio_tlmm_config(GPIO_CFG(MSM_GPIO_SD_DET, 0, GPIO_CFG_INPUT,
					 GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
			pr_err("Failed to configure GPIO %d\n",
					 MSM_GPIO_SD_DET);
	} else
		pr_err("Failed to request GPIO%d\n", MSM_GPIO_SD_DET);
#endif
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	msm_add_sdcc(2, &qsd8x50_sdc2_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	msm_add_sdcc(3, &qsd8x50_sdc3_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	msm_add_sdcc(4, &qsd8x50_sdc4_data);
#endif
}

#endif
static int __init qsd8x50_cfg_smsc911x(void)
{
	int rc = 0;
	u8 enet_clk_en_gpio = 33;
	u8 irq_gpio = 105;
	smsc911x_resources[0].start = 0x9C000000;
	smsc911x_resources[0].end = 0x9C0002ff;
	smsc911x_resources[1].start = MSM_GPIO_TO_INT(irq_gpio);
	smsc911x_resources[1].end = MSM_GPIO_TO_INT(irq_gpio);

	rc = gpio_request(enet_clk_en_gpio, "smsc911x_enet_clk_en");
	if (rc) {
		pr_err("Failed to request GPIO pin %d (rc=%d)\n",
			enet_clk_en_gpio, rc);
		goto err;
	}

	rc = gpio_request(irq_gpio, "smsc911x_irq");
	if (rc) {
		pr_err("Failed to request GPIO pin %d (rc=%d)\n",
			irq_gpio, rc);
		goto err;
	}

	rc = gpio_tlmm_config(GPIO_CFG(irq_gpio, 0, GPIO_CFG_INPUT,
					GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
					GPIO_CFG_ENABLE);
	if (rc) {
		printk(KERN_ERR "smsc911x: Could not configure IRQ gpio\n");
		goto err;
	}

	gpio_direction_output(irq_gpio, 1);
	gpio_direction_input(irq_gpio);

	rc = gpio_tlmm_config(GPIO_CFG(enet_clk_en_gpio, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
					GPIO_CFG_ENABLE);
	if (rc) {
		printk(KERN_ERR
			"smsc911x: Could not configure ENET_CLK_EN gpio\n");
		goto err;
	}

	gpio_direction_output(enet_clk_en_gpio, 1);

	platform_device_register(&smsc911x_device);
err:
	gpio_free(enet_clk_en_gpio);
	gpio_free(irq_gpio);
	return -ENODEV;
}

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 8594,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].residency = 23740,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 4594,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].residency = 23740,

	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].suspend_enabled
		= 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].idle_enabled = 0,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 443,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].residency = 1098,

	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].latency = 2,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].residency = 0,
};

static void
msm_i2c_gpio_config(int iface, int config_type)
{
	int gpio_scl;
	int gpio_sda;
	if (iface) {
		gpio_scl = 60;
		gpio_sda = 61;
	} else {
		gpio_scl = 95;
		gpio_sda = 96;
	}
	if (config_type) {
		gpio_tlmm_config(GPIO_CFG(gpio_scl, 1, GPIO_CFG_INPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_sda, 1, GPIO_CFG_INPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	} else {
		gpio_tlmm_config(GPIO_CFG(gpio_scl, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_sda, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	}
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
	.clk_freq = 100000,
	.rsl_id = SMEM_SPINLOCK_I2C,
	.pri_clk = 95,
	.pri_dat = 96,
	.aux_clk = 60,
	.aux_dat = 61,
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_init(void)
{
	if (gpio_request(95, "i2c_pri_clk"))
		pr_err("failed to request gpio i2c_pri_clk\n");
	if (gpio_request(96, "i2c_pri_dat"))
		pr_err("failed to request gpio i2c_pri_dat\n");
	if (gpio_request(60, "i2c_sec_clk"))
		pr_err("failed to request gpio i2c_sec_clk\n");
	if (gpio_request(61, "i2c_sec_dat"))
		pr_err("failed to request gpio i2c_sec_dat\n");

	msm_i2c_pdata.rmutex = 1;
	msm_i2c_pdata.pm_lat =
		msm_pm_data[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN]
		.latency;
	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static unsigned pmem_kernel_ebi1_size = PMEM_KERNEL_EBI1_SIZE;
static int __init pmem_kernel_ebi1_size_setup(char *p)
{
	pmem_kernel_ebi1_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_kernel_ebi1_size", pmem_kernel_ebi1_size_setup);

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
static unsigned pmem_kernel_smi_size = MSM_PMEM_SMIPOOL_SIZE;
static int __init pmem_kernel_smi_size_setup(char *p)
{
	pmem_kernel_smi_size = memparse(p, NULL);

	/* Make sure that we don't allow more SMI memory then is
	   available - the kernel mapping code has no way of knowing
	   if it has gone over the edge */

	if (pmem_kernel_smi_size > MSM_PMEM_SMIPOOL_SIZE)
		pmem_kernel_smi_size = MSM_PMEM_SMIPOOL_SIZE;

	return 0;
}
early_param("pmem_kernel_smi_size", pmem_kernel_smi_size_setup);
#endif

static unsigned pmem_mdp_size = MSM_PMEM_MDP_SIZE;
static int __init pmem_mdp_size_setup(char *p)
{
	pmem_mdp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_mdp_size", pmem_mdp_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_adsp_size", pmem_adsp_size_setup);


static unsigned audio_size = MSM_AUDIO_SIZE;
static int __init audio_size_setup(char *p)
{
	audio_size = memparse(p, NULL);
	return 0;
}
early_param("audio_size", audio_size_setup);

static void __init qsd8x50_init(void)
{
	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n",
		       __func__);
	msm_clock_init(msm_clocks_8x50, msm_num_clocks_8x50);
	hs_clk = clk_get(NULL, "usb_hs_clk");
	if (IS_ERR(hs_clk))
		printk(KERN_ERR "%s: hs_clk get failed!\n", __func__);
	phy_clk = clk_get(NULL, "usb_phy_clk");
	if (IS_ERR(phy_clk))
		printk(KERN_ERR "%s: phy_clk get failed!\n", __func__);
	qsd8x50_cfg_smsc911x();
	msm_acpu_clock_init(&qsd8x50_clock_data);

	msm_hsusb_pdata.swfi_latency =
		msm_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_hsusb_peripheral.dev.platform_data = &msm_hsusb_pdata;
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;

#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
	msm_device_tsif.dev.platform_data = &tsif_platform_data;
#endif
	platform_add_devices(devices, ARRAY_SIZE(devices));
	msm_fb_add_devices();

	qsd8x50_init_host();

#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT))
	qsd8x50_init_mmc();
#endif
	bt_power_init_st_1_5();
	audio_gpio_init();
	msm_device_i2c_init();

	lid_sensor_gpio_init();
	msm_qsd_spi_init();
	qup_device_i2c_init();
	i2c_register_board_info(0, msm_i2c_st1_5_info,
			ARRAY_SIZE(msm_i2c_st1_5_info));
	i2c_register_board_info(4, msm_qup_st1_5_info,
			ARRAY_SIZE(msm_qup_st1_5_info));
	spi_register_board_info(msm_spi_board_info,
				ARRAY_SIZE(msm_spi_board_info));
	msm_pm_set_platform_data(msm_pm_data, ARRAY_SIZE(msm_pm_data));

#ifdef CONFIG_SURF_FFA_GPIO_KEYPAD
	platform_device_register(&keypad_device_surf);
#endif
}

static void __init qsd8x50_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = pmem_kernel_ebi1_size;
	if (size) {
		addr = alloc_bootmem_aligned(size, 0x100000);
		android_pmem_kernel_ebi1_pdata.start = __pa(addr);
		android_pmem_kernel_ebi1_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for kernel"
			" ebi1 pmem arena\n", size, addr, __pa(addr));
	}

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
	size = pmem_kernel_smi_size;
	if (size > MSM_PMEM_SMIPOOL_SIZE) {
		printk(KERN_ERR "pmem kernel smi arena size %lu is too big\n",
			size);

		size = MSM_PMEM_SMIPOOL_SIZE;
	}

	android_pmem_kernel_smi_pdata.start = MSM_PMEM_SMIPOOL_BASE;
	android_pmem_kernel_smi_pdata.size = size;

	pr_info("allocating %lu bytes at %lx (%lx physical)"
		"for pmem kernel smi arena\n", size,
		(long unsigned int) MSM_PMEM_SMIPOOL_BASE,
		__pa(MSM_PMEM_SMIPOOL_BASE));
#endif

	size = pmem_mdp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_pdata.start = __pa(addr);
		android_pmem_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for mdp "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = pmem_adsp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_adsp_pdata.start = __pa(addr);
		android_pmem_adsp_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for adsp "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = MSM_FB_SIZE_ST15;
	addr = (void *)MSM_FB_BASE;
	msm_fb_resources[0].start = (unsigned long)addr;
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("using %lu bytes of SMI at %lx physical for fb\n",
		size, (unsigned long)addr);

	size = audio_size ? : MSM_AUDIO_SIZE;
	addr = alloc_bootmem(size);
	msm_audio_resources[0].start = __pa(addr);
	msm_audio_resources[0].end = msm_audio_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for audio\n",
		size, addr, __pa(addr));
}

static void __init qsd8x50_map_io(void)
{
	msm_shared_ram_phys = MSM_SHARED_RAM_PHYS;
	msm_map_qsd8x50_io();
	qsd8x50_allocate_memory_regions();
}

MACHINE_START(QSD8X50A_ST1_5, "QCT QSD8X50A ST1.5")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = qsd8x50_map_io,
	.init_irq = qsd8x50_init_irq,
	.init_machine = qsd8x50_init,
	.timer = &msm_timer,
MACHINE_END
