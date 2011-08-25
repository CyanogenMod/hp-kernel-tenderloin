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
 *
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/mfd/pmic8058.h>
#include <linux/input/pmic8058-keypad.h>
#include <linux/pmic8058-pwrkey.h>
#include <linux/pmic8058-vibrator.h>
#include <linux/leds.h>
#include <linux/pmic8058-othc.h>
#include <linux/mfd/pmic8901.h>
#include <linux/regulator/pmic8058-regulator.h>
#include <linux/regulator/pmic8901-regulator.h>
#include <linux/bootmem.h>
#include <linux/pwm.h>
#include <linux/pmic8058-pwm.h>
#include <linux/leds-pmic8058.h>
#include <linux/rtc/rtc-pm8058.h>
#include <linux/mfd/marimba.h>

#include <linux/i2c.h>
#include <linux/i2c/sx150x.h>
#include <linux/smsc911x.h>
#include <linux/spi/spi.h>
#include <linux/input/tdisc_shinetsu.h>
#include <linux/input/cy8c_ts.h>

#ifdef CONFIG_ANDROID_PMEM
#include <linux/android_pmem.h>
#endif

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>
#include <asm/setup.h>

#include <mach/mpp.h>
#include <mach/board.h>
#include <mach/irqs.h>
#include <mach/msm_spi.h>
#include <mach/msm_serial_hs.h>
#include <mach/msm_iomap.h>
#include <asm/mach/mmc.h>
#include <mach/tlmm.h>
#include <mach/msm_battery.h>
#include <mach/msm_hsusb.h>
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android_composite.h>
#endif
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>

#include "devices.h"
#include "devices-msm8x60.h"
#include "cpuidle.h"
#include "pm.h"
#include "rpm.h"
#include "spm.h"
#include "timer.h"

#define MSM_SHARED_RAM_PHYS 0x40000000

/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_BASE			NR_MSM_GPIOS
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)		(pm_gpio + PM8058_GPIO_BASE)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)		(sys_gpio - PM8058_GPIO_BASE)
#define PM8058_IRQ_BASE				(NR_MSM_IRQS + NR_GPIO_IRQS)

#define PM8901_GPIO_BASE			(PM8058_GPIO_BASE + \
						PM8058_GPIOS + PM8058_MPPS)
#define PM8901_GPIO_PM_TO_SYS(pm_gpio)		(pm_gpio + PM8901_GPIO_BASE)
#define PM8901_GPIO_SYS_TO_PM(sys_gpio)		(sys_gpio - PM901_GPIO_BASE)
#define PM8901_IRQ_BASE				(PM8058_IRQ_BASE + \
						NR_PMIC8058_IRQS)

#define GPIO_EXPANDER_GPIO_BASE \
	(PM8901_GPIO_BASE + PM8901_MPPS)
#define GPIO_EXPANDER_IRQ_BASE (PM8901_IRQ_BASE + NR_PMIC8901_IRQS)

/*
 * The UI_INTx_N lines are pmic gpio lines which connect i2c
 * gpio expanders to the pm8058.
 */
#define UI_INT1_N 25
#define UI_INT2_N 34
#define UI_INT3_N 14

void __iomem *gic_cpu_base_addr;

static struct msm_spm_platform_data msm_spm_data[] __initdata = {
	[0] = {
		.reg_base_addr = MSM_SAW0_BASE,

		.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x0F,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x68,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0xFFFFFFFF,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0xFFFFFFFF,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x17,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x07,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

		.awake_vlevel = 0x9C,
		.retention_vlevel = 0x81,
		.collapse_vlevel = 0x20,
		.retention_mid_vlevel = 0x94,
		.collapse_mid_vlevel = 0x8C,

		.vctl_timeout_us = 50,
	},

	[1] = {
		.reg_base_addr = MSM_SAW1_BASE,

		.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x0F,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x68,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0xFFFFFFFF,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0xFFFFFFFF,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x17,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x07,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

		.awake_vlevel = 0x9C,
		.retention_vlevel = 0x81,
		.collapse_vlevel = 0x20,
		.retention_mid_vlevel = 0x94,
		.collapse_mid_vlevel = 0x8C,

		.vctl_timeout_us = 50,
	},
};

static struct msm_acpu_clock_platform_data msm8x60_acpu_clock_data = {
	/* SoC has no frequency step size constraints. */
	.max_speed_delta_khz = UINT_MAX,
};

/*
 * The smc91x configuration varies depending on platform.
 * The resources data structure is filled in at runtime.
 */
static struct resource smc91x_resources[] = {
	[0] = {
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device smc91x_device = {
	.name          = "smc91x",
	.id            = 0,
	.num_resources = ARRAY_SIZE(smc91x_resources),
	.resource      = smc91x_resources,
};

static struct resource smsc911x_resources[] = {
	[0] = {
		.flags = IORESOURCE_MEM,
		.start = 0x1b800000,
		.end   = 0x1b8000ff
	},
	[1] = {
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	},
};

static struct smsc911x_platform_config smsc911x_config = {
	.irq_polarity = SMSC911X_IRQ_POLARITY_ACTIVE_HIGH,
	.irq_type     = SMSC911X_IRQ_TYPE_PUSH_PULL,
	.flags        = SMSC911X_USE_16BIT
};

static struct platform_device smsc911x_device = {
	.name          = "smsc911x",
	.id            = 0,
	.num_resources = ARRAY_SIZE(smsc911x_resources),
	.resource      = smsc911x_resources,
	.dev           = {
		.platform_data = &smsc911x_config
	}
};

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR * 2] = {
	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_POWER_COLLAPSE)] = {
		.supported = 1,
		.suspend_enabled = 1,
		.idle_enabled = 1,
		.latency = 1000,
		.residency = 9000,
	},

	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE)] = {
		.supported = 1,
		.suspend_enabled = 1,
		.idle_enabled = 1,
		.latency = 500,
		.residency = 6000,
	},

	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT)] = {
		.supported = 1,
		.suspend_enabled = 1,
		.idle_enabled = 1,
		.latency = 2,
		.residency = 0,
	},

	[MSM_PM_MODE(1, MSM_PM_SLEEP_MODE_POWER_COLLAPSE)] = {
		.supported = 1,
		.suspend_enabled = 1,
		.idle_enabled = 0,
		.latency = 600,
		.residency = 7200,
	},

	[MSM_PM_MODE(1, MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE)] = {
		.supported = 1,
		.suspend_enabled = 1,
		.idle_enabled = 1,
		.latency = 500,
		.residency = 6000,
	},

	[MSM_PM_MODE(1, MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT)] = {
		.supported = 1,
		.suspend_enabled = 1,
		.idle_enabled = 1,
		.latency = 2,
		.residency = 0,
	},
};

static struct msm_cpuidle_state msm_cstates[] __initdata = {
	{0, 0, "C0", "WFI", MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT},
	{1, 0, "C0", "WFI", MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT},
};

#ifdef CONFIG_USB_EHCI_MSM
struct regulator *votg_5v_switch;
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
	static int vbus_is_on;

	/* If VBUS is already on (or off), do nothing. */
	if (on == vbus_is_on)
		return;

	if (on) {
		votg_5v_switch = regulator_get(NULL, "8901_usb_otg");
		if (IS_ERR(votg_5v_switch)) {
			pr_err("%s: unable to get votg_5v_switch\n", __func__);
			return;
		}
		if (regulator_enable(votg_5v_switch)) {
			pr_err("%s: Unable to enable the regulator:"
					" votg_5v_switch\n", __func__);
			return;
		}
	} else {
		if (regulator_disable(votg_5v_switch))
			pr_err("%s: Unable to enable the regulator:"
					" votg_5v_switch\n", __func__);
		regulator_put(votg_5v_switch);
	}

	vbus_is_on = on;
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
	.phy_info   = (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
};
#endif

#if defined(CONFIG_USB_GADGET_MSM_72K) || defined(CONFIG_USB_EHCI_MSM)
static struct msm_otg_platform_data msm_otg_pdata = {
	/* if usb link is in sps there is no need for
	 * usb pclk as dayatona fabric clock will be
	 * used instead
	 */
	.usb_in_sps = 1,
	.vbus_power = msm_hsusb_vbus_power,
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
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
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

#ifdef CONFIG_MSM_VPE
static struct resource msm_vpe_resources[] = {
	{
		.start	= 0x05300000,
		.end	= 0x05300000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VPE,
		.end	= INT_VPE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_vpe_device = {
	.name = "msm_vpe",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_vpe_resources),
	.resource = msm_vpe_resources,
};
#endif

#ifdef CONFIG_MSM_CAMERA

static uint32_t camera_off_gpio_table[] = {
	GPIO_CFG(47, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(48, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(32, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(105, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(106, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static uint32_t camera_on_gpio_table[] = {
	GPIO_CFG(47, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(48, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(32, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(105, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(106, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}
#define GPIO_CAM_EN (GPIO_EXPANDER_GPIO_BASE + 13)
static void config_camera_on_gpios(void)
{
	int rc;
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));

	rc = gpio_request(GPIO_CAM_EN, "CAM_EN");
	if (rc) {
		printk(KERN_ERR "%s: CAMSENSOR gpio %d request"
			"failed\n", __func__, GPIO_CAM_EN);
		return;
	}
	gpio_direction_output(GPIO_CAM_EN, 0);
	mdelay(20);
	gpio_set_value(GPIO_CAM_EN, 1);
}

static void config_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));

	gpio_set_value(GPIO_CAM_EN, 0);
	mdelay(20);
	gpio_free(GPIO_CAM_EN);
}


struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.csiphy = 0x04800000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = CSI_0_IRQ,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 228570000,
};

struct resource msm_camera_resources[] = {
	{
		.start	= 0x04500000,
		.end	= 0x04500000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= VFE_IRQ,
		.end	= VFE_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};
static struct msm_camera_sensor_flash_src msm_flash_src = {
	.flash_sr_type				= MSM_CAMERA_FLASH_SRC_PWM,
	._fsrc.pwm_src.freq			= 1000,
	._fsrc.pwm_src.max_load		= 300,
	._fsrc.pwm_src.low_load		= 30,
	._fsrc.pwm_src.high_load	= 100,
	._fsrc.pwm_src.channel		= 7,
};
#ifdef CONFIG_IMX074
static struct msm_camera_sensor_flash_data flash_imx074 = {
	.flash_type		= MSM_CAMERA_FLASH_LED,
	.flash_src		= &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_imx074_data = {
	.sensor_name	= "imx074",
	.sensor_reset	= 106,
	.sensor_pwd		= 85,
	.vcm_pwd		= 1,
	.vcm_enable		= 0,
	.pdata			= &msm_camera_device_data,
	.resource		= msm_camera_resources,
	.num_resources	= ARRAY_SIZE(msm_camera_resources),
	.flash_data		= &flash_imx074,
	.csi_if			= 1
};
struct platform_device msm_camera_sensor_imx074 = {
	.name	= "msm_camera_imx074",
	.dev	= {
		.platform_data = &msm_camera_sensor_imx074_data,
	},
};
#endif
static struct i2c_board_info msm_camera_boardinfo[] __initdata = {
	#ifdef CONFIG_IMX074
	{
		I2C_BOARD_INFO("imx074", 0x1A),
	},
	#endif
};
#endif

#ifdef CONFIG_MSM_GEMINI
static struct resource msm_gemini_resources[] = {
	{
		.start  = 0x04600000,
		.end    = 0x04600000 + SZ_1M - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_JPEG,
		.end    = INT_JPEG,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_gemini_device = {
	.name           = "msm_gemini",
	.resource       = msm_gemini_resources,
	.num_resources  = ARRAY_SIZE(msm_gemini_resources),
};
#endif

#ifdef CONFIG_I2C_QUP
static void gsbi_qup_i2c_gpio_config(int adap_id, int config_type)
{
}

static struct msm_i2c_platform_data msm_gsbi3_qup_i2c_pdata = {
	.clk_freq = 100000,
	.src_clk_rate = 24000000,
	.clk = "gsbi_qup_clk",
	.pclk = "gsbi_pclk",
	.msm_i2c_config_gpio = gsbi_qup_i2c_gpio_config,
};

static struct msm_i2c_platform_data msm_gsbi4_qup_i2c_pdata = {
	.clk_freq = 100000,
	.src_clk_rate = 24000000,
	.clk = "gsbi_qup_clk",
	.pclk = "gsbi_pclk",
	.msm_i2c_config_gpio = gsbi_qup_i2c_gpio_config,
};

static struct msm_i2c_platform_data msm_gsbi7_qup_i2c_pdata = {
	.clk_freq = 100000,
	.src_clk_rate = 24000000,
	.clk = "gsbi_qup_clk",
	.pclk = "gsbi_pclk",
	.msm_i2c_config_gpio = gsbi_qup_i2c_gpio_config,
};

static struct msm_i2c_platform_data msm_gsbi8_qup_i2c_pdata = {
	.clk_freq = 100000,
	.src_clk_rate = 24000000,
	.clk = "gsbi_qup_clk",
	.pclk = "gsbi_pclk",
	.msm_i2c_config_gpio = gsbi_qup_i2c_gpio_config,
};

static struct msm_i2c_platform_data msm_gsbi9_qup_i2c_pdata = {
	.clk_freq = 100000,
	.src_clk_rate = 24000000,
	.clk = "gsbi_qup_clk",
	.pclk = "gsbi_pclk",
	.msm_i2c_config_gpio = gsbi_qup_i2c_gpio_config,
};
#endif

#if defined(CONFIG_SPI_QUP) || defined(CONFIG_SPI_QUP_MODULE)
static struct msm_spi_platform_data msm_gsbi1_qup_spi_pdata = {
	.max_clock_speed = 24000000,
	.clk_name = "gsbi_qup_clk",
	.pclk_name = "gsbi_pclk",
};
#endif

#ifdef CONFIG_I2C_SSBI
/* PMIC SSBI */
static struct msm_ssbi_platform_data msm_ssbi1_pdata = {
	.controller_type = MSM_SBI_CTRL_PMIC_ARBITER,
};

/* PMIC SSBI */
static struct msm_ssbi_platform_data msm_ssbi2_pdata = {
	.controller_type = MSM_SBI_CTRL_PMIC_ARBITER,
};

/* CODEC/TSSC SSBI */
static struct msm_ssbi_platform_data msm_ssbi3_pdata = {
	.controller_type = MSM_SBI_CTRL_SSBI,
};
#endif

#ifdef CONFIG_BATTERY_MSM
/* Use basic value for fake MSM battery */
static struct msm_psy_batt_pdata msm_psy_batt_data = {
	.avail_chg_sources = AC_CHG,
};

static struct platform_device msm_batt_device = {
	.name              = "msm-battery",
	.id                = -1,
	.dev.platform_data = &msm_psy_batt_data,
};
#endif

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
/* prim = 1024 x 600 x 4(bpp) x 2(pages)
 * hdmi = 1920 x 1080 x 2(bpp) x 1(page)
 * Note: must be multiple of 4096 */
#define MSM_FB_SIZE 0x8A5000;
#else /* CONFIG_FB_MSM_HDMI_MSM_PANEL */
#define MSM_FB_SIZE 0x500000;
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */
#define MSM_PMEM_SF_SIZE 0x1700000;
#define MSM_GPU_PHYS_SIZE       SZ_2M

#define MSM_PMEM_KERNEL_EBI1_SIZE  0x600000
#define MSM_PMEM_ADSP_SIZE         0x2000000

#define MSM_SMI_BASE          0x38000000
/* Kernel SMI PMEM Region for video core, used for Firmware */
/* and encoder,decoder scratch buffers */
/* Kernel SMI PMEM Region Should always precede the user space */
/* SMI PMEM Region, as the video core will use offset address */
/* from the Firmware base */
#define PMEM_KERNEL_SMI_BASE  (MSM_SMI_BASE)
#define PMEM_KERNEL_SMI_SIZE  0x1000000
/* User space SMI PMEM Region for video core*/
/* used for encoder, decoder input & output buffers  */
#define MSM_PMEM_SMIPOOL_BASE (PMEM_KERNEL_SMI_BASE + PMEM_KERNEL_SMI_SIZE)
#define MSM_PMEM_SMIPOOL_SIZE 0x2800000

static unsigned fb_size = MSM_FB_SIZE;
static void __init fb_size_setup(char **p)
{
	fb_size = memparse(*p, p);
}
__early_param("fb_size=", fb_size_setup);

static unsigned gpu_phys_size = MSM_GPU_PHYS_SIZE;
static void __init gpu_phys_size_setup(char **p)
{
	gpu_phys_size = memparse(*p, p);
}
__early_param("gpu_phys_size=", gpu_phys_size_setup);

#ifdef CONFIG_KERNEL_PMEM_EBI_REGION
static unsigned pmem_kernel_ebi1_size = MSM_PMEM_KERNEL_EBI1_SIZE;
static void __init pmem_kernel_ebi1_size_setup(char **p)
{
	pmem_kernel_ebi1_size = memparse(*p, p);
}
__early_param("pmem_kernel_ebi1_size=", pmem_kernel_ebi1_size_setup);
#endif

#ifdef CONFIG_ANDROID_PMEM
static unsigned pmem_sf_size = MSM_PMEM_SF_SIZE;
static void __init pmem_sf_size_setup(char **p)
{
	pmem_sf_size = memparse(*p, p);
}
__early_param("pmem_sf_size=", pmem_sf_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;

static void __init pmem_adsp_size_setup(char **p)
{
	pmem_adsp_size = memparse(*p, p);
}
__early_param("pmem_adsp_size=", pmem_adsp_size_setup);
#endif

static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
static int msm_fb_detect_panel(const char *name)
{
	if (!strcmp(name, "hdmi_msm"))
		return 0;
	pr_warning("%s: not supported '%s'", __func__, name);
	return -ENODEV;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
};
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources     = ARRAY_SIZE(msm_fb_resources),
	.resource          = msm_fb_resources,
#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
	.dev.platform_data = &msm_fb_pdata,
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */
};

#ifdef CONFIG_KERNEL_PMEM_EBI_REGION
static struct android_pmem_platform_data android_pmem_kernel_ebi1_pdata = {
	.name = PMEM_KERNEL_EBI1_DATA_NAME,
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct platform_device android_pmem_kernel_ebi1_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_kernel_ebi1_pdata },
};
#endif

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
static struct android_pmem_platform_data android_pmem_kernel_smi_pdata = {
	.name = PMEM_KERNEL_SMI_DATA_NAME,
	/* defaults to bitmap don't edit */
	.cached = 0,
};

static struct platform_device android_pmem_kernel_smi_device = {
	.name = "android_pmem",
	.id = 6,
	.dev = { .platform_data = &android_pmem_kernel_smi_pdata },
};
#endif

#ifdef CONFIG_ANDROID_PMEM
static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = {.platform_data = &android_pmem_pdata},
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};
static struct android_pmem_platform_data android_pmem_smipool_pdata = {
	.name = "pmem_smipool",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};
static struct platform_device android_pmem_smipool_device = {
	.name = "android_pmem",
	.id = 7,
	.dev = { .platform_data = &android_pmem_smipool_pdata },
};

#endif

#define GPIO_BACKLIGHT_PWM0 0
#define GPIO_BACKLIGHT_PWM1 1

static int pmic_backlight_gpio[2]
	= { GPIO_BACKLIGHT_PWM0, GPIO_BACKLIGHT_PWM1 };
static struct msm_panel_common_pdata lcdc_samsung_panel_data = {
	.gpio_num = pmic_backlight_gpio, /* two LPG CHANNELS for backlight */
};

static struct platform_device lcdc_samsung_panel_device = {
	.name = "lcdc_samsung_wsvga",
	.id = 0,
	.dev = {
		.platform_data = &lcdc_samsung_panel_data,
	}
};

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
static struct resource hdmi_msm_resources[] = {
	{
		.name  = "hdmi_msm_qfprom_addr",
		.start = 0x00700000,
		.end   = 0x007060FF,
		.flags = IORESOURCE_MEM,
	},
	{
		.name  = "hdmi_msm_hdmi_addr",
		.start = 0x04A00000,
		.end   = 0x04A00FFF,
		.flags = IORESOURCE_MEM,
	},
	{
		.name  = "hdmi_msm_irq",
		.start = HDMI_IRQ,
		.end   = HDMI_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device hdmi_msm_device = {
	.name = "hdmi_msm",
	.id = 0,
	.num_resources = ARRAY_SIZE(hdmi_msm_resources),
	.resource = hdmi_msm_resources,
};
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */

static struct platform_device mipi_dsi_video_toshiba_wvga_panel_device = {
	.name = "dsi_video_toshiba_wvga",
	.id = 0,
};

static void __init msm8x60_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = MSM_FB_SIZE;
	addr = alloc_bootmem(size);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));

	size = gpu_phys_size;
	if (size) {
		addr = alloc_bootmem(size);
		msm_device_kgsl.resource[1].start = __pa(addr);
		msm_device_kgsl.resource[1].end =
			msm_device_kgsl.resource[1].start + size - 1;
		pr_info("allocating %lu bytes at %p (%lx physical) for "
		"KGSL\n", size, addr, __pa(addr));
	}

#ifdef CONFIG_KERNEL_PMEM_EBI_REGION
	size = pmem_kernel_ebi1_size;
	if (size) {
		addr = alloc_bootmem_aligned(size, 0x100000);
		android_pmem_kernel_ebi1_pdata.start = __pa(addr);
		android_pmem_kernel_ebi1_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for kernel"
			" ebi1 pmem arena\n", size, addr, __pa(addr));
	}
#endif

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
	size = PMEM_KERNEL_SMI_SIZE;
	if (size) {
		android_pmem_kernel_smi_pdata.start = PMEM_KERNEL_SMI_BASE;
		android_pmem_kernel_smi_pdata.size = size;
		pr_info("allocating %lu bytes at %lx physical for kernel"
			" smi pmem arena\n", size,
			(unsigned long) PMEM_KERNEL_SMI_BASE);
	}
#endif

#ifdef CONFIG_ANDROID_PMEM
	size = pmem_adsp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_adsp_pdata.start = __pa(addr);
		android_pmem_adsp_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for adsp "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = MSM_PMEM_SMIPOOL_SIZE;
	if (size) {
		android_pmem_smipool_pdata.start = MSM_PMEM_SMIPOOL_BASE;
		android_pmem_smipool_pdata.size = size;
		pr_info("allocating %lu bytes at %lx physical for user"
			" smi  pmem arena\n", size,
			(unsigned long) MSM_PMEM_SMIPOOL_BASE);
	}

	size = pmem_sf_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_pdata.start = __pa(addr);
		android_pmem_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for sf "
			"pmem arena\n", size, addr, __pa(addr));
	}
#endif
}

static struct regulator *vreg_tmg200;

#define TS_PEN_IRQ_GPIO 61
static int tmg200_power(int vreg_on)
{
	int rc = -EINVAL;

	if (!vreg_tmg200) {
		printk(KERN_ERR "%s: regulator 8058_s3 not found (%d)\n",
			__func__, rc);
		return rc;
	}

	rc = vreg_on ? regulator_enable(vreg_tmg200) :
		  regulator_disable(vreg_tmg200);
	if (rc < 0)
		printk(KERN_ERR "%s: vreg 8058_s3 %s failed (%d)\n",
				__func__, vreg_on ? "enable" : "disable", rc);
	return rc;
}

static int tmg200_dev_setup(bool enable)
{
	int rc;

	if (enable) {
		vreg_tmg200 = regulator_get(NULL, "8058_s3");
		if (IS_ERR(vreg_tmg200)) {
			pr_err("%s: regulator get of 8058_s3 failed (%ld)\n",
				__func__, PTR_ERR(vreg_tmg200));
			rc = PTR_ERR(vreg_tmg200);
			return rc;
		}

		rc = regulator_set_voltage(vreg_tmg200, 1800000, 1800000);
		if (rc) {
			pr_err("%s: regulator_set_voltage() = %d\n",
				__func__, rc);
			goto reg_put;
		}

		/* configure touchscreen interrupt gpio */
		rc = gpio_tlmm_config(GPIO_CFG(TS_PEN_IRQ_GPIO, 0, GPIO_CFG_INPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 0);
		if (rc) {
			pr_err("%s: unable to configure gpio %d\n",
				__func__, TS_PEN_IRQ_GPIO);
			goto reg_put;
		}

		rc = gpio_request(TS_PEN_IRQ_GPIO, "cy8ctmg200_irq_gpio");
		if (rc) {
			pr_err("%s: unable to request gpio %d\n",
				__func__, TS_PEN_IRQ_GPIO);
			goto reg_put;
		}
	} else {
		/* put voltage sources */
		regulator_put(vreg_tmg200);
		/* free gpio */
		gpio_free(TS_PEN_IRQ_GPIO);
	}
	return 0;
reg_put:
	regulator_put(vreg_tmg200);
	return rc;
}

static struct cy8c_ts_platform_data cy8ctmg200_pdata = {
	.ts_name = "msm_tmg200_ts",
	.dis_min_x = 0,
	.dis_max_x = 1023,
	.dis_min_y = 0,
	.dis_max_y = 599,
	.use_polling = 0,
	.min_tid = 1,
	.max_tid = 255,
	.min_touch = 0,
	.max_touch = 255,
	.min_width = 0,
	.max_width = 255,
	.power_on = tmg200_power,
	.dev_setup = tmg200_dev_setup,
	.nfingers = 2,
};

static struct i2c_board_info cy8ctmg200_board_info[] = {
	{
		I2C_BOARD_INFO("cy8ctmg200", 0x2),
		.platform_data = &cy8ctmg200_pdata,
		.irq = MSM_GPIO_TO_INT(TS_PEN_IRQ_GPIO),
	}
};

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
       .inject_rx_on_wakeup = 1,
       .rx_to_inject = 0xFD,
       .clk_name = "gsbi_uart_clk",
};
#endif

#if defined(CONFIG_GPIO_SX150X) || defined(CONFIG_GPIO_SX150X_MODULE)

#define GPIO_LEFT_LED_1		(GPIO_EXPANDER_GPIO_BASE + (16 * 3))
#define GPIO_LEFT_LED_2		(GPIO_EXPANDER_GPIO_BASE + (16 * 3) + 1)
#define GPIO_LEFT_LED_3		(GPIO_EXPANDER_GPIO_BASE + (16 * 3) + 2)
#define GPIO_LEFT_LED_WLAN	(GPIO_EXPANDER_GPIO_BASE + (16 * 3) + 3)
#define GPIO_LEFT_LED_5		(GPIO_EXPANDER_GPIO_BASE + (16 * 3) + 7)
#define GPIO_RIGHT_LED_1	(GPIO_EXPANDER_GPIO_BASE + (16 * 3) + 8)
#define GPIO_RIGHT_LED_2	(GPIO_EXPANDER_GPIO_BASE + (16 * 3) + 8 + 1)
#define GPIO_RIGHT_LED_3	(GPIO_EXPANDER_GPIO_BASE + (16 * 3) + 8 + 2)
#define GPIO_RIGHT_LED_BT	(GPIO_EXPANDER_GPIO_BASE + (16 * 3) + 8 + 3)
#define GPIO_RIGHT_LED_5	(GPIO_EXPANDER_GPIO_BASE + (16 * 3) + 8 + 7)

static struct gpio_led gpio_exp_leds_config[] = {
	{
		.name = "left_led1:green",
		.gpio = GPIO_LEFT_LED_1,
		.active_low = 1,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	{
		.name = "left_led2:red",
		.gpio = GPIO_LEFT_LED_2,
		.active_low = 1,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	{
		.name = "left_led3:green",
		.gpio = GPIO_LEFT_LED_3,
		.active_low = 1,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	{
		.name = "wlan_led:orange",
		.gpio = GPIO_LEFT_LED_WLAN,
		.active_low = 1,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	{
		.name = "left_led5:green",
		.gpio = GPIO_LEFT_LED_5,
		.active_low = 1,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	{
		.name = "right_led1:green",
		.gpio = GPIO_RIGHT_LED_1,
		.active_low = 1,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	{
		.name = "right_led2:red",
		.gpio = GPIO_RIGHT_LED_2,
		.active_low = 1,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	{
		.name = "right_led3:green",
		.gpio = GPIO_RIGHT_LED_3,
		.active_low = 1,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	{
		.name = "bt_led:blue",
		.gpio = GPIO_RIGHT_LED_BT,
		.active_low = 1,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	{
		.name = "right_led5:green",
		.gpio = GPIO_RIGHT_LED_5,
		.active_low = 1,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
};

static struct gpio_led_platform_data gpio_leds_pdata = {
	.num_leds = ARRAY_SIZE(gpio_exp_leds_config),
	.leds = gpio_exp_leds_config,
};

static struct platform_device gpio_leds = {
	.name          = "leds-gpio",
	.id            = -1,
	.dev           = {
		.platform_data = &gpio_leds_pdata,
	},
};
#endif

static struct platform_device *early_devices[] __initdata = {
	&msm_device_gpio,
};

static struct platform_device *rumi_sim_devices[] __initdata = {
	&smc91x_device,
	&msm_device_uart_dm12,
#ifdef CONFIG_I2C_QUP
	&msm_gsbi3_qup_i2c_device,
	&msm_gsbi4_qup_i2c_device,
	&msm_gsbi7_qup_i2c_device,
	&msm_gsbi8_qup_i2c_device,
	&msm_gsbi9_qup_i2c_device,
#endif
#if defined(CONFIG_SPI_QUP) || defined(CONFIG_SPI_QUP_MODULE)
	&msm_gsbi1_qup_spi_device,
#endif
#ifdef CONFIG_I2C_SSBI
	&msm_device_ssbi1,
	&msm_device_ssbi2,
	&msm_device_ssbi3,
#endif
#ifdef CONFIG_KERNEL_PMEM_EBI_REGION
	&android_pmem_kernel_ebi1_device,
#endif
#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
	&android_pmem_kernel_smi_device,
#endif
#ifdef CONFIG_ANDROID_PMEM
	&android_pmem_device,
	&android_pmem_adsp_device,
	&android_pmem_smipool_device,
#endif
#ifdef CONFIG_MSM_ROTATOR
	&msm_rotator_device,
#endif
	&msm_fb_device,
	&msm_device_kgsl,
	&lcdc_samsung_panel_device,
#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
	&hdmi_msm_device,
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */
#ifdef CONFIG_MSM_CAMERA
#ifdef CONFIG_IMX074
	&msm_camera_sensor_imx074,
#endif
#endif
#ifdef CONFIG_MSM_GEMINI
	&msm_gemini_device,
#endif
#ifdef CONFIG_MSM_VPE
	&msm_vpe_device,
#endif
	&msm_device_vidc
};

static struct platform_device *surf_devices[] __initdata = {
	&msm_device_smd,
	&smsc911x_device,
	&msm_device_uart_dm12,
#ifdef CONFIG_I2C_QUP
	&msm_gsbi3_qup_i2c_device,
	&msm_gsbi4_qup_i2c_device,
	&msm_gsbi7_qup_i2c_device,
	&msm_gsbi8_qup_i2c_device,
	&msm_gsbi9_qup_i2c_device,
#endif
#if defined(CONFIG_SPI_QUP) || defined(CONFIG_SPI_QUP_MODULE)
	&msm_gsbi1_qup_spi_device,
#endif
#ifdef CONFIG_SERIAL_MSM_HS
	&msm_device_uart_dm1,
#endif
#ifdef CONFIG_I2C_SSBI
	&msm_device_ssbi1,
	&msm_device_ssbi2,
	&msm_device_ssbi3,
#endif
#if defined(CONFIG_USB_GADGET_MSM_72K) || defined(CONFIG_USB_EHCI_HCD)
	&msm_device_otg,
#endif
#ifdef CONFIG_USB_GADGET_MSM_72K
	&msm_device_gadget_peripheral,
#endif
#ifdef CONFIG_USB_ANDROID
	&usb_mass_storage_device,
	&rndis_device,
	&android_usb_device,
#endif
#ifdef CONFIG_BATTERY_MSM
	&msm_batt_device,
#endif
#if defined(CONFIG_GPIO_SX150X) || defined(CONFIG_GPIO_SX150X_MODULE)
	&gpio_leds,
#endif
#ifdef CONFIG_KERNEL_PMEM_EBI_REGION
	&android_pmem_kernel_ebi1_device,
#endif
#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
	&android_pmem_kernel_smi_device,
#endif
#ifdef CONFIG_ANDROID_PMEM
	&android_pmem_device,
	&android_pmem_adsp_device,
	&android_pmem_smipool_device,
#endif
#ifdef CONFIG_MSM_ROTATOR
	&msm_rotator_device,
#endif
	&msm_fb_device,
	&msm_device_kgsl,
	&lcdc_samsung_panel_device,
#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
	&hdmi_msm_device,
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */
	&mipi_dsi_video_toshiba_wvga_panel_device,
#ifdef CONFIG_MSM_CAMERA
#ifdef CONFIG_IMX074
	&msm_camera_sensor_imx074,
#endif
#endif
#ifdef CONFIG_MSM_GEMINI
	&msm_gemini_device,
#endif
#ifdef CONFIG_MSM_VPE
	&msm_vpe_device,
#endif
	&msm_device_vidc
};

#if defined(CONFIG_GPIO_SX150X) || defined(CONFIG_GPIO_SX150X_MODULE)
static struct sx150x_platform_data sx150x_data[] __initdata = {
	/* "CORE" expander */
	[0] = {
		.gpio_base         = GPIO_EXPANDER_GPIO_BASE,
		.oscio_is_gpo      = false,
		.io_pullup_ena     = 0xCFFB,
		.io_pulldn_ena     = 0,
		.io_open_drain_ena = 0xCFFF,
		.io_polarity       = 0,
		.irq_summary       = -1, /* see fixup_i2c_configs() */
		.irq_base          = GPIO_EXPANDER_IRQ_BASE,
	},
	/* "DOCKING" expander */
	[1] = {
		.gpio_base         = GPIO_EXPANDER_GPIO_BASE + 16,
		.oscio_is_gpo      = false,
		.io_pullup_ena     = 0,
		.io_pulldn_ena     = 0,
		.io_open_drain_ena = 0,
		.io_polarity       = 0,
		.irq_summary       = PM8058_GPIO_IRQ(PM8058_IRQ_BASE,
						     UI_INT2_N),
		.irq_base          = GPIO_EXPANDER_IRQ_BASE + 16,
	},
	/* "SURF" expander */
	[2] = {
		.gpio_base         = GPIO_EXPANDER_GPIO_BASE + (16 * 2),
		.oscio_is_gpo      = false,
		.io_pullup_ena     = 0,
		.io_pulldn_ena     = 0,
		.io_open_drain_ena = 0,
		.io_polarity       = 0,
		.irq_summary       = PM8058_GPIO_IRQ(PM8058_IRQ_BASE,
						     UI_INT1_N),
		.irq_base          = GPIO_EXPANDER_IRQ_BASE + (16 * 2),
	},
	/* left keyboard FHA/FFA I/O */
	[3] = {
		.gpio_base         = GPIO_EXPANDER_GPIO_BASE + (16 * 3),
		.oscio_is_gpo      = false,
		.io_pullup_ena     = 0,
		.io_pulldn_ena     = 0,
		.io_open_drain_ena = 0,
		.io_polarity       = 0,
		.irq_summary       = PM8058_GPIO_IRQ(PM8058_IRQ_BASE,
						     UI_INT3_N),
		.irq_base          = GPIO_EXPANDER_IRQ_BASE + (16 * 3),
	},
	/* right keyboard FHA/FFA I/O */
	[4] = {
		.gpio_base         = GPIO_EXPANDER_GPIO_BASE + (16 * 3) + 8,
		.oscio_is_gpo      = true,
		.io_pullup_ena     = 0,
		.io_pulldn_ena     = 0,
		.io_open_drain_ena = 0,
		.io_polarity       = 0,
		.irq_summary       = PM8058_GPIO_IRQ(PM8058_IRQ_BASE,
						     UI_INT3_N),
		.irq_base          = GPIO_EXPANDER_IRQ_BASE + (16 * 3) + 8,
	},
};

#ifdef CONFIG_I2C
static struct i2c_board_info core_expanders_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("sx1509q", 0x3e),
		.platform_data = &sx150x_data[0]
	},
	{
		I2C_BOARD_INFO("sx1509q", 0x3f),
		.platform_data = &sx150x_data[1]
	},
};

static struct i2c_board_info surf_expanders_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("sx1509q", 0x70),
		.platform_data = &sx150x_data[2]
	}
};

static struct i2c_board_info fha_expanders_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("sx1508q", 0x21),
		.platform_data = &sx150x_data[3]
	},
	{
		I2C_BOARD_INFO("sx1508q", 0x22),
		.platform_data = &sx150x_data[4]
	}
};
#endif
#endif

#ifdef CONFIG_PMIC8058
#define PMIC_GPIO_SDC3_DET 22

static int pm8058_gpios_init(void)
{
	int i;
	int rc;
	struct pm8058_gpio_cfg {
		int                gpio;
		struct pm8058_gpio cfg;
	};

	struct pm8058_gpio_cfg gpio_cfgs[] = {
		{ /* FFA ethernet */
			6,
			{
				.direction      = PM_GPIO_DIR_IN,
				.pull           = PM_GPIO_PULL_DN,
				.vin_sel        = 2,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			},
		},
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
		{
			PMIC_GPIO_SDC3_DET - 1,
			{
				.direction      = PM_GPIO_DIR_IN,
				.pull           = PM_GPIO_PULL_UP_30,
				.vin_sel        = 2,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			},
		},
#endif
		{ /* core&surf gpio expander */
			UI_INT1_N,
			{
				.direction      = PM_GPIO_DIR_IN,
				.pull           = PM_GPIO_PULL_NO,
				.vin_sel        = PM_GPIO_VIN_S3,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			},
		},
		{ /* docking gpio expander */
			UI_INT2_N,
			{
				.direction      = PM_GPIO_DIR_IN,
				.pull           = PM_GPIO_PULL_NO,
				.vin_sel        = PM_GPIO_VIN_S3,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			},
		},
		{ /* FHA/keypad gpio expanders */
			UI_INT3_N,
			{
				.direction      = PM_GPIO_DIR_IN,
				.pull           = PM_GPIO_PULL_NO,
				.vin_sel        = PM_GPIO_VIN_S3,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			},
		},
		{ /* TouchDisc Interrupt */
			5,
			{
				.direction      = PM_GPIO_DIR_IN,
				.pull           = PM_GPIO_PULL_UP_1P5,
				.vin_sel        = 2,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			}
		},
		{ /* Timpani Reset */
			20,
			{
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 1,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_DN,
				.out_strength	= PM_GPIO_STRENGTH_HIGH,
				.function	= PM_GPIO_FUNC_NORMAL,
				.vin_sel	= 2,
				.inv_int_pol	= 0,
			}
		}
	};

	for (i = 0; i < ARRAY_SIZE(gpio_cfgs); ++i) {
		rc = pm8058_gpio_config(gpio_cfgs[i].gpio,
				&gpio_cfgs[i].cfg);
		if (rc < 0) {
			pr_err("%s pmic gpio config failed\n",
				__func__);
			return rc;
		}
	}

	return 0;
}

static const unsigned int ffa_keymap[] = {
	KEY(0, 0, KEY_FN_F1),	 /* LS - PUSH1 */
	KEY(0, 1, KEY_UP),	 /* NAV - UP */
	KEY(0, 2, KEY_LEFT),	 /* NAV - LEFT */
	KEY(0, 3, KEY_VOLUMEUP), /* Shuttle SW_UP */

	KEY(1, 0, KEY_FN_F2), 	 /* LS - PUSH2 */
	KEY(1, 1, KEY_RIGHT),    /* NAV - RIGHT */
	KEY(1, 2, KEY_DOWN),     /* NAV - DOWN */
	KEY(1, 3, KEY_VOLUMEDOWN),

	KEY(2, 3, KEY_ENTER),     /* SW_PUSH key */

	KEY(4, 0, KEY_CAMERA_FOCUS), /* RS - PUSH1 */
	KEY(4, 1, KEY_UP),	  /* USER_UP */
	KEY(4, 2, KEY_LEFT),	  /* USER_LEFT */
	KEY(4, 3, KEY_HOME),	  /* Right switch: MIC Bd */
	KEY(4, 4, KEY_FN_F3),	  /* Reserved MIC */

	KEY(5, 0, KEY_CAMERA_SNAPSHOT), /* RS - PUSH2 */
	KEY(5, 1, KEY_RIGHT),	  /* USER_RIGHT */
	KEY(5, 2, KEY_DOWN),	  /* USER_DOWN */
	KEY(5, 3, KEY_BACK),	  /* Left switch: MIC */
	KEY(5, 4, KEY_MENU),	  /* Center switch: MIC */
};

static struct resource resources_keypad[] = {
	{
		.start	= PM8058_KEYPAD_IRQ(PM8058_IRQ_BASE),
		.end	= PM8058_KEYPAD_IRQ(PM8058_IRQ_BASE),
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= PM8058_KEYSTUCK_IRQ(PM8058_IRQ_BASE),
		.end	= PM8058_KEYSTUCK_IRQ(PM8058_IRQ_BASE),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct matrix_keymap_data ffa_keymap_data = {
	.keymap_size	= ARRAY_SIZE(ffa_keymap),
	.keymap		= ffa_keymap,
};

static struct pmic8058_keypad_data ffa_keypad_data = {
	.input_name		= "ffa-keypad",
	.input_phys_device	= "ffa-keypad/input0",
	.num_rows		= 6,
	.num_cols		= 5,
	.rows_gpio_start	= 8,
	.cols_gpio_start	= 0,
	.debounce_ms		= {8, 10},
	.scan_delay_ms		= 32,
	.row_hold_ns            = 91500,
	.wakeup			= 1,
	.keymap_data		= &ffa_keymap_data,
};

static struct resource resources_pwrkey[] = {
	{
		.start	= PM8058_PWRKEY_REL_IRQ(PM8058_IRQ_BASE),
		.end	= PM8058_PWRKEY_REL_IRQ(PM8058_IRQ_BASE),
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= PM8058_PWRKEY_PRESS_IRQ(PM8058_IRQ_BASE),
		.end	= PM8058_PWRKEY_PRESS_IRQ(PM8058_IRQ_BASE),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct pmic8058_pwrkey_pdata pwrkey_pdata = {
	.pull_up		= 1,
	.kpd_trigger_delay_us   = 970,
	.wakeup			= 1,
	.pwrkey_time_ms		= 500,
};

static struct pmic8058_vibrator_pdata pmic_vib_pdata = {
	.initial_vibrate_ms  = 500,
	.level_mV = 3000,
	.max_timeout_ms = 15000,
};


#define PM8058_OTHC_CNTR_BASE0	0xA0
#define PM8058_OTHC_CNTR_BASE1	0x134
#define PM8058_OTHC_CNTR_BASE2	0x137

static struct othc_hsed_config hsed_config_1 = {
	.othc_headset = OTHC_HEADSET_NO,
	.othc_lowcurr_thresh_uA = 100,
	.othc_highcurr_thresh_uA = 700,
	.othc_hyst_prediv_us = 7800,
	.othc_period_clkdiv_us = 62500,
	.othc_hyst_clk_us = 121000,
	.othc_period_clk_us = 312500,
	.othc_wakeup = 1,
};

/* MIC_BIAS0 is configured as normal MIC BIAS */
static struct pmic8058_othc_config_pdata othc_config_pdata_0 = {
	.micbias_select = OTHC_MICBIAS_0,
	.micbias_capability = OTHC_MICBIAS,
	.micbias_enable = OTHC_SIGNAL_OFF,
};

/* MIC_BIAS1 is configured as HSED_BIAS for OTHC */
static struct pmic8058_othc_config_pdata othc_config_pdata_1 = {
	.micbias_select = OTHC_MICBIAS_1,
	.micbias_capability = OTHC_MICBIAS_HSED,
	.micbias_enable = OTHC_SIGNAL_PWM_TCXO,
	.hsed_config = &hsed_config_1,
};

/* MIC_BIAS2 is configured as normal MIC BIAS */
static struct pmic8058_othc_config_pdata othc_config_pdata_2 = {
	.micbias_select = OTHC_MICBIAS_2,
	.micbias_capability = OTHC_MICBIAS,
	.micbias_enable = OTHC_SIGNAL_OFF,
};

static struct resource resources_othc_0[] = {
	{
		.name = "othc_base",
		.start = PM8058_OTHC_CNTR_BASE0,
		.end   = PM8058_OTHC_CNTR_BASE0,
		.flags = IORESOURCE_IO,
	},
};

static struct resource resources_othc_1[] = {
	{
		.start = PM8058_SW_1_IRQ(PM8058_IRQ_BASE),
		.end   = PM8058_SW_1_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = PM8058_IR_1_IRQ(PM8058_IRQ_BASE),
		.end   = PM8058_IR_1_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "othc_base",
		.start = PM8058_OTHC_CNTR_BASE1,
		.end   = PM8058_OTHC_CNTR_BASE1,
		.flags = IORESOURCE_IO,
	},
};

static struct resource resources_othc_2[] = {
	{
		.name = "othc_base",
		.start = PM8058_OTHC_CNTR_BASE2,
		.end   = PM8058_OTHC_CNTR_BASE2,
		.flags = IORESOURCE_IO,
	},
};

static int pm8058_pwm_config(struct pwm_device *pwm, int ch, int on)
{
	struct pm8058_gpio pwm_gpio_config = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM_GPIO_VIN_VPH,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_2,
	};

	int rc = -EINVAL;
	int id;

	switch (ch) {
	case 0:
	case 1:
	case 2:
		if (on) {
			id = 24 + ch;
			rc = pm8058_gpio_config(id - 1, &pwm_gpio_config);
			if (rc)
				pr_err("%s: pm8058_gpio_config(%d): rc=%d\n",
					__func__, id, rc);
		}

	default:
		break;
	}
	return rc;

}

static struct pm8058_pwm_pdata pm8058_pwm_data = {
	.config		= pm8058_pwm_config,
};

#define PM8058_GPIO_INT           88

static struct pm8058_gpio_platform_data pm8058_gpio_data = {
	.gpio_base	= PM8058_GPIO_PM_TO_SYS(0),
	.irq_base	= PM8058_GPIO_IRQ(PM8058_IRQ_BASE, 0),
	.init		= pm8058_gpios_init,
};

static struct pm8058_gpio_platform_data pm8058_mpp_data = {
	.gpio_base	= PM8058_GPIO_PM_TO_SYS(PM8058_GPIOS),
	.irq_base	= PM8058_MPP_IRQ(PM8058_IRQ_BASE, 0),
};

static struct regulator_consumer_supply pm8058_vreg_supply[PM8058_VREG_MAX] = {
	[PM8058_VREG_ID_L0]  = REGULATOR_SUPPLY("8058_l0",  NULL),
	[PM8058_VREG_ID_L1]  = REGULATOR_SUPPLY("8058_l1",  NULL),
	[PM8058_VREG_ID_L2]  = REGULATOR_SUPPLY("8058_l2",  NULL),
	[PM8058_VREG_ID_L3]  = REGULATOR_SUPPLY("8058_l3",  NULL),
	[PM8058_VREG_ID_L4]  = REGULATOR_SUPPLY("8058_l4",  NULL),
	[PM8058_VREG_ID_L5]  = REGULATOR_SUPPLY("8058_l5",  NULL),
	[PM8058_VREG_ID_L6]  = REGULATOR_SUPPLY("8058_l6",  NULL),
	[PM8058_VREG_ID_L7]  = REGULATOR_SUPPLY("8058_l7",  NULL),
	[PM8058_VREG_ID_L8]  = REGULATOR_SUPPLY("8058_l8",  NULL),
	[PM8058_VREG_ID_L9]  = REGULATOR_SUPPLY("8058_l9",  NULL),
	[PM8058_VREG_ID_L10] = REGULATOR_SUPPLY("8058_l10", NULL),
	[PM8058_VREG_ID_L11] = REGULATOR_SUPPLY("8058_l11", NULL),
	[PM8058_VREG_ID_L12] = REGULATOR_SUPPLY("8058_l12", NULL),
	[PM8058_VREG_ID_L13] = REGULATOR_SUPPLY("8058_l13", NULL),
	[PM8058_VREG_ID_L14] = REGULATOR_SUPPLY("8058_l14", NULL),
	[PM8058_VREG_ID_L15] = REGULATOR_SUPPLY("8058_l15", NULL),
	[PM8058_VREG_ID_L16] = REGULATOR_SUPPLY("8058_l16", NULL),
	[PM8058_VREG_ID_L17] = REGULATOR_SUPPLY("8058_l17", NULL),
	[PM8058_VREG_ID_L18] = REGULATOR_SUPPLY("8058_l18", NULL),
	[PM8058_VREG_ID_L19] = REGULATOR_SUPPLY("8058_l19", NULL),
	[PM8058_VREG_ID_L20] = REGULATOR_SUPPLY("8058_l20", NULL),
	[PM8058_VREG_ID_L21] = REGULATOR_SUPPLY("8058_l21", NULL),
	[PM8058_VREG_ID_L22] = REGULATOR_SUPPLY("8058_l22", NULL),
	[PM8058_VREG_ID_L23] = REGULATOR_SUPPLY("8058_l23", NULL),
	[PM8058_VREG_ID_L24] = REGULATOR_SUPPLY("8058_l24", NULL),
	[PM8058_VREG_ID_L25] = REGULATOR_SUPPLY("8058_l25", NULL),

	[PM8058_VREG_ID_S0] = REGULATOR_SUPPLY("8058_s0", NULL),
	[PM8058_VREG_ID_S1] = REGULATOR_SUPPLY("8058_s1", NULL),
	[PM8058_VREG_ID_S2] = REGULATOR_SUPPLY("8058_s2", NULL),
	[PM8058_VREG_ID_S3] = REGULATOR_SUPPLY("8058_s3", NULL),
	[PM8058_VREG_ID_S4] = REGULATOR_SUPPLY("8058_s4", NULL),

	[PM8058_VREG_ID_LVS0] = REGULATOR_SUPPLY("8058_lvs0", NULL),
	[PM8058_VREG_ID_LVS1] = REGULATOR_SUPPLY("8058_lvs1", NULL),

	[PM8058_VREG_ID_NCP] = REGULATOR_SUPPLY("8058_ncp", NULL),
};

#define PM8058_VREG_INIT(_id, _min_uV, _max_uV, _modes, _ops, _apply_uV) \
	[_id] = { \
		.constraints = { \
			.valid_modes_mask = _modes, \
			.valid_ops_mask = _ops, \
			.min_uV = _min_uV, \
			.max_uV = _max_uV, \
			.apply_uV = _apply_uV, \
		}, \
		.num_consumer_supplies = 1, \
		.consumer_supplies = &pm8058_vreg_supply[_id], \
	}

#define PM8058_VREG_INIT_LDO(_id, _min_uV, _max_uV) \
	PM8058_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_NORMAL | \
			REGULATOR_MODE_IDLE | REGULATOR_MODE_STANDBY, \
			REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS | \
			REGULATOR_CHANGE_MODE, 1)

#define PM8058_VREG_INIT_SMPS(_id, _min_uV, _max_uV) \
	PM8058_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_NORMAL | \
			REGULATOR_MODE_IDLE | REGULATOR_MODE_STANDBY, \
			REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS | \
			REGULATOR_CHANGE_MODE, 1)

#define PM8058_VREG_INIT_LVS(_id) \
	PM8058_VREG_INIT(_id, 0, 0, REGULATOR_MODE_NORMAL, \
			REGULATOR_CHANGE_STATUS, 0)

#define PM8058_VREG_INIT_NCP(_id, _min_uV, _max_uV) \
	PM8058_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_NORMAL, \
			REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS, 1)

static struct regulator_init_data pm8058_vreg_init[PM8058_VREG_MAX] = {
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L0,  1200000, 1200000),
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L1,  1200000, 1200000),
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L2,  1800000, 1800000),
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L3,  1800000, 1800000),
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L4,  2850000, 2850000),
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L5,  2850000, 2850000),
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L6,  3050000, 3050000),
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L7,  1800000, 1800000),
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L8,  2900000, 2900000),
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L9,  1800000, 1800000),
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L10, 2600000, 2600000),
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L11, 1500000, 1500000),
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L12, 2900000, 2900000),
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L13, 2050000, 2050000),
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L14, 2850000, 2850000),
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L15, 2850000, 2850000),
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L16, 1800000, 1800000),
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L17, 2600000, 2600000),
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L18, 2200000, 2200000),
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L19, 2500000, 2500000),
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L20, 1800000, 1800000),
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L21, 1100000, 1100000),
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L22, 1200000, 1200000),
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L23, 1200000, 1200000),
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L24, 1200000, 1200000),
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L25, 1200000, 1200000),

	PM8058_VREG_INIT_SMPS(PM8058_VREG_ID_S0, 1100000, 1100000),
	PM8058_VREG_INIT_SMPS(PM8058_VREG_ID_S1, 1100000, 1100000),
	PM8058_VREG_INIT_SMPS(PM8058_VREG_ID_S2, 1350000, 1350000),
	PM8058_VREG_INIT_SMPS(PM8058_VREG_ID_S3, 1800000, 1800000),
	PM8058_VREG_INIT_SMPS(PM8058_VREG_ID_S4, 2200000, 2200000),

	PM8058_VREG_INIT_LVS(PM8058_VREG_ID_LVS0),
	PM8058_VREG_INIT_LVS(PM8058_VREG_ID_LVS0),

	PM8058_VREG_INIT_NCP(PM8058_VREG_ID_NCP, -1800000, -1800000),
};

#define PM8058_VREG(_id) { \
	.name = "pm8058-regulator", \
	.id = _id, \
	.platform_data = &pm8058_vreg_init[_id], \
	.data_size = sizeof(pm8058_vreg_init[_id]), \
}

static struct resource resources_rtc[] = {
       {
		.start  = PM8058_RTC_IRQ(PM8058_IRQ_BASE),
		.end    = PM8058_RTC_IRQ(PM8058_IRQ_BASE),
		.flags  = IORESOURCE_IRQ,
       },
       {
		.start  = PM8058_RTC_ALARM_IRQ(PM8058_IRQ_BASE),
		.end    = PM8058_RTC_ALARM_IRQ(PM8058_IRQ_BASE),
		.flags  = IORESOURCE_IRQ,
       },
};

static struct pm8058_rtc_pdata pm8058_rtc_data = {
	.rtc_write_enable = false,
};

static struct pmic8058_led pmic8058_flash_leds[] = {
	[0] = {
		.name		= "camera:flash0",
		.max_brightness = 15,
		.id		= PMIC8058_ID_FLASH_LED_0,
	},
	[1] = {
		.name		= "camera:flash1",
		.max_brightness = 15,
		.id		= PMIC8058_ID_FLASH_LED_1,
	},
};

static struct pmic8058_leds_platform_data pm8058_flash_leds_data = {
	.num_leds = ARRAY_SIZE(pmic8058_flash_leds),
	.leds	= pmic8058_flash_leds,
};

static struct mfd_cell pm8058_subdevs[] = {
	{
		.name = "pm8058-keypad",
		.id		= -1,
		.num_resources	= ARRAY_SIZE(resources_keypad),
		.resources	= resources_keypad,
		.platform_data	= &ffa_keypad_data,
		.data_size	= sizeof(ffa_keypad_data),
	},
	{	.name = "pm8058-gpio",
		.id		= -1,
		.platform_data	= &pm8058_gpio_data,
		.data_size	= sizeof(pm8058_gpio_data),
	},
	{	.name = "pm8058-mpp",
		.id		= -1,
		.platform_data	= &pm8058_mpp_data,
		.data_size	= sizeof(pm8058_mpp_data),
	},
	{	.name = "pm8058-pwrkey",
		.id	= -1,
		.resources = resources_pwrkey,
		.num_resources = ARRAY_SIZE(resources_pwrkey),
		.platform_data = &pwrkey_pdata,
		.data_size = sizeof(pwrkey_pdata),
	},
	{
		.name = "pm8058-vib",
		.id = -1,
		.platform_data = &pmic_vib_pdata,
		.data_size     = sizeof(pmic_vib_pdata),
	},
	{
		.name = "pm8058-pwm",
		.id = -1,
		.platform_data = &pm8058_pwm_data,
		.data_size = sizeof(pm8058_pwm_data),
	},
	{
		.name = "pm8058-othc",
		.id = 0,
		.platform_data = &othc_config_pdata_0,
		.data_size = sizeof(othc_config_pdata_0),
		.num_resources = ARRAY_SIZE(resources_othc_0),
		.resources = resources_othc_0,
	},
	{
		/* OTHC1 module has headset/switch dection */
		.name = "pm8058-othc",
		.id = 1,
		.num_resources = ARRAY_SIZE(resources_othc_1),
		.resources = resources_othc_1,
		.platform_data = &othc_config_pdata_1,
		.data_size = sizeof(othc_config_pdata_1),
	},
	{
		.name = "pm8058-othc",
		.id = 2,
		.platform_data = &othc_config_pdata_2,
		.data_size = sizeof(othc_config_pdata_2),
		.num_resources = ARRAY_SIZE(resources_othc_2),
		.resources = resources_othc_2,
	},
	{
		.name = "pm8058-rtc",
		.id = -1,
		.platform_data = &pm8058_rtc_data,
		.data_size = sizeof(pm8058_rtc_data),
		.num_resources  = ARRAY_SIZE(resources_rtc),
		.resources      = resources_rtc,
	},
	{	.name = "pm8058-led",
		.id		= -1,
		.platform_data = &pm8058_flash_leds_data,
		.data_size = sizeof(pm8058_flash_leds_data),
	},
	PM8058_VREG(PM8058_VREG_ID_L0),
	PM8058_VREG(PM8058_VREG_ID_L1),
	PM8058_VREG(PM8058_VREG_ID_L2),
	PM8058_VREG(PM8058_VREG_ID_L3),
	PM8058_VREG(PM8058_VREG_ID_L4),
	PM8058_VREG(PM8058_VREG_ID_L5),
	PM8058_VREG(PM8058_VREG_ID_L6),
	PM8058_VREG(PM8058_VREG_ID_L7),
	PM8058_VREG(PM8058_VREG_ID_L8),
	PM8058_VREG(PM8058_VREG_ID_L9),
	PM8058_VREG(PM8058_VREG_ID_L10),
	PM8058_VREG(PM8058_VREG_ID_L11),
	PM8058_VREG(PM8058_VREG_ID_L12),
	PM8058_VREG(PM8058_VREG_ID_L13),
	PM8058_VREG(PM8058_VREG_ID_L14),
	PM8058_VREG(PM8058_VREG_ID_L15),
	PM8058_VREG(PM8058_VREG_ID_L16),
	PM8058_VREG(PM8058_VREG_ID_L17),
	PM8058_VREG(PM8058_VREG_ID_L18),
	PM8058_VREG(PM8058_VREG_ID_L19),
	PM8058_VREG(PM8058_VREG_ID_L20),
	PM8058_VREG(PM8058_VREG_ID_L21),
	PM8058_VREG(PM8058_VREG_ID_L22),
	PM8058_VREG(PM8058_VREG_ID_L23),
	PM8058_VREG(PM8058_VREG_ID_L24),
	PM8058_VREG(PM8058_VREG_ID_L25),
	PM8058_VREG(PM8058_VREG_ID_S0),
	PM8058_VREG(PM8058_VREG_ID_S1),
	PM8058_VREG(PM8058_VREG_ID_S2),
	PM8058_VREG(PM8058_VREG_ID_S3),
	PM8058_VREG(PM8058_VREG_ID_S4),
	PM8058_VREG(PM8058_VREG_ID_NCP),
	PM8058_VREG(PM8058_VREG_ID_LVS0),
	PM8058_VREG(PM8058_VREG_ID_LVS1),
};

static struct pm8058_platform_data pm8058_platform_data = {
	.irq_base = PM8058_IRQ_BASE,

	.num_subdevs = ARRAY_SIZE(pm8058_subdevs),
	.sub_devices = pm8058_subdevs,
};

static struct i2c_board_info pm8058_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("pm8058-core", 0),
		.irq = MSM_GPIO_TO_INT(PM8058_GPIO_INT),
		.platform_data = &pm8058_platform_data,
	},
};
#endif /* CONFIG_PMIC8058 */

#if defined(CONFIG_TOUCHDISC_VTD518_SHINETSU) || \
		defined(CONFIG_TOUCHDISC_VTD518_SHINETSU_MODULE)
#define TDISC_I2C_SLAVE_ADDR	0x67
#define PMIC_GPIO_TDISC		PM8058_GPIO_PM_TO_SYS(5)
#define TDISC_INT		PM8058_GPIO_IRQ(PM8058_IRQ_BASE, 5)
#define TDISC_OE		(GPIO_EXPANDER_GPIO_BASE + (16 * 3) + 4)

static const char *vregs_tdisc_name[] = {
	"8058_l5",
	"8058_s3",
};

static const int vregs_tdisc_val[] = {
	2850000,/* uV */
	1800000,
};
static struct regulator *vregs_tdisc[ARRAY_SIZE(vregs_tdisc_name)];

static int tdisc_shinetsu_setup(void)
{
	int rc, i;

	rc = gpio_request(PMIC_GPIO_TDISC, "tdisc_interrupt");
	if (rc) {
		pr_err("%s: gpio_request failed for PMIC_GPIO_TDISC\n",
								__func__);
		return rc;
	}

	rc = gpio_request(TDISC_OE, "tdisc_oe");
	if (rc) {
		pr_err("%s: gpio_request failed for TDISC_OE\n",
							__func__);
		goto fail_gpio_oe;
	}

	rc = gpio_direction_output(TDISC_OE, 1);
	if (rc) {
		pr_err("%s: gpio_direction_output failed for TDISC_OE\n",
								__func__);
		gpio_free(TDISC_OE);
		goto fail_gpio_oe;
	}

	for (i = 0; i < ARRAY_SIZE(vregs_tdisc_name); i++) {
		vregs_tdisc[i] = regulator_get(NULL, vregs_tdisc_name[i]);
		if (IS_ERR(vregs_tdisc[i])) {
			printk(KERN_ERR "%s: regulator get %s failed (%ld)\n",
				__func__, vregs_tdisc_name[i],
				PTR_ERR(vregs_tdisc[i]));
			rc = PTR_ERR(vregs_tdisc[i]);
			goto vreg_get_fail;
		}

		rc = regulator_set_voltage(vregs_tdisc[i],
				vregs_tdisc_val[i], vregs_tdisc_val[i]);
		if (rc) {
			printk(KERN_ERR "%s: regulator_set_voltage() = %d\n",
				__func__, rc);
			goto vreg_set_voltage_fail;
		}
	}

	return rc;
vreg_set_voltage_fail:
	i++;
vreg_get_fail:
	while (i)
		regulator_put(vregs_tdisc[--i]);
fail_gpio_oe:
	gpio_free(PMIC_GPIO_TDISC);
	return rc;
}

static void tdisc_shinetsu_release(void)
{
	for (i = 0; i < ARRAY_SIZE(vregs_tdisc_name); i++)
		regulator_put(vregs_tdisc_name[i]);

	gpio_free(PMIC_GPIO_TDISC);
	gpio_free(TDISC_OE);
}

static int tdisc_shinetsu_enable(void)
{
	int i, rc = -EINVAL;

	for (i = 0; i < ARRAY_SIZE(vregs_tdisc_name); i++) {
		rc = regulator_enable(vregs_tdisc[i]);
		if (rc < 0) {
			printk(KERN_ERR "%s: vreg %s enable failed (%d)\n",
				__func__, vregs_tdisc_name[i], rc);
			goto vreg_fail;
		}
	}

	/* Enable the OE (output enable) gpio */
	gpio_set_value(TDISC_OE, 1);

	return 0;
vreg_fail:
	while (i)
		regulator_disable(vregs_tdisc[--i]);
	return rc;
}

static void tdisc_shinetsu_disable(void)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(vregs_tdisc_name); i++) {
		rc = regulator_disable(vregs_tdisc[i]);
		if (rc < 0)
			printk(KERN_ERR "%s: vreg %s disable failed (%d)\n",
				__func__, vregs_tdisc_name[i], rc);
	}

	/* Disable the OE (output enable) gpio */
	gpio_set_value(TDISC_OE, 0);
}

static struct tdisc_abs_values tdisc_abs = {
	.x_max = 32,
	.y_max = 32,
	.x_min = -32,
	.y_min = -32,
	.pressure_max = 32,
	.pressure_min = 0,
};

static struct tdisc_platform_data tdisc_data = {
	.tdisc_setup = tdisc_shinetsu_setup,
	.tdisc_release = tdisc_shinetsu_release,
	.tdisc_enable = tdisc_shinetsu_enable,
	.tdisc_disable = tdisc_shinetsu_disable,
	.tdisc_wakeup  = 1,
	.tdisc_gpio = PMIC_GPIO_TDISC,
	.tdisc_abs  = &tdisc_abs,
};

static struct i2c_board_info msm_i2c_gsbi3_tdisc_info[] = {
	{
		I2C_BOARD_INFO("vtd518", TDISC_I2C_SLAVE_ADDR),
		.irq =  TDISC_INT,
		.platform_data = &tdisc_data,
	},
};
#endif

static struct regulator *vreg_timpani_1;
static struct regulator *vreg_timpani_2;

static unsigned int msm_timpani_setup_power(void)
{
	int rc;

	vreg_timpani_1 = regulator_get(NULL, "8058_l0");
	if (IS_ERR(vreg_timpani_1)) {
		pr_err("%s: Unable to get 8058_l0\n", __func__);
		return -ENODEV;
	}

	vreg_timpani_2 = regulator_get(NULL, "8058_s3");
	if (IS_ERR(vreg_timpani_2)) {
		pr_err("%s: Unable to get 8058_s3\n", __func__);
		return -ENODEV;
	}

	rc = regulator_enable(vreg_timpani_1);

	if (rc) {
		pr_err("%s: Enable regulator 8058_l0 failed\n", __func__);
		return rc;
	}

	rc = regulator_enable(vreg_timpani_2);

	if (rc) {
		pr_err("%s: Enable regulator 8058_s3 failed\n", __func__);
		goto fail_s3;
	}
	return rc;

fail_s3:
	regulator_put(vreg_timpani_1);
	return rc;
}

static void msm_timpani_shutdown_power(void)
{
	int rc;

	rc = regulator_disable(vreg_timpani_1);
	if (rc)
		pr_err("%s: Disable regulator 8058_l0 failed\n", __func__);

	regulator_put(vreg_timpani_1);

	rc = regulator_disable(vreg_timpani_2);
	if (rc)
		pr_err("%s: Disable regulator 8058_s3 failed\n", __func__);

	regulator_put(vreg_timpani_2);
}

/* Power analog function of codec */
static struct regulator *vreg_timpani_cdc_apwr;
static int msm_timpani_codec_power(int vreg_on)
{
	int rc = 0;

	if (!vreg_timpani_cdc_apwr) {

		vreg_timpani_cdc_apwr = regulator_get(NULL, "8058_s4");

		if (IS_ERR(vreg_timpani_cdc_apwr)) {
			pr_err("%s: vreg_get failed (%ld)\n",
			__func__, PTR_ERR(vreg_timpani_cdc_apwr));
			rc = PTR_ERR(vreg_timpani_cdc_apwr);
			goto vreg_fail;
		}
	}

	if (vreg_on) {
		rc = regulator_enable(vreg_timpani_cdc_apwr);
		if (rc)
			pr_err("%s: vreg_enable failed %d \n",
			__func__, rc);
		goto vreg_fail;
	} else {
		rc = regulator_disable(vreg_timpani_cdc_apwr);
		if (rc)
			pr_err("%s: vreg_disable failed %d \n",
			__func__, rc);
		goto vreg_fail;
	}

vreg_fail:
	return rc;
}

static struct marimba_codec_platform_data timpani_codec_pdata = {
	.marimba_codec_power =  msm_timpani_codec_power,
};

#define TIMPANI_SLAVE_ID_CDC_ADDR		0X77
#define TIMPANI_SLAVE_ID_QMEMBIST_ADDR		0X66

static struct marimba_platform_data timpani_pdata = {
	.slave_id[MARIMBA_SLAVE_ID_CDC]	= TIMPANI_SLAVE_ID_CDC_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_QMEMBIST] = TIMPANI_SLAVE_ID_QMEMBIST_ADDR,
	.marimba_setup = msm_timpani_setup_power,
	.marimba_shutdown = msm_timpani_shutdown_power,
	.codec = &timpani_codec_pdata,
};

#define TIMPANI_I2C_SLAVE_ADDR	0xD

static struct i2c_board_info msm_i2c_gsbi7_timpani_info[] = {
	{
		I2C_BOARD_INFO("timpani", TIMPANI_I2C_SLAVE_ADDR),
		.platform_data = &timpani_pdata,
	},
};

#ifdef CONFIG_PMIC8901

#define PM8901_GPIO_INT           91

static int pm8901_mpp0_init(void *data)
{
	int val = machine_is_msm8x60_surf() ?
		PM_MPP_DOUT_CTL_LOW : PM_MPP_DOUT_CTL_HIGH;
	int rc = pm8901_mpp_config(0, PM_MPP_TYPE_D_OUTPUT,
			PM8901_MPP_DIG_LEVEL_VPH,
			val);
	if (rc)
		pr_err("%s: pm8901_mpp_config failed with %d\n", __func__, rc);

	return rc;
}

static struct pm8901_gpio_platform_data pm8901_mpp_data = {
	.gpio_base	= PM8901_GPIO_PM_TO_SYS(0),
	.irq_base	= PM8901_MPP_IRQ(PM8901_IRQ_BASE, 0),
};

static struct regulator_consumer_supply pm8901_vreg_supply[PM8901_VREG_MAX] = {
	[PM8901_VREG_ID_L0]  = REGULATOR_SUPPLY("8901_l0",  NULL),
	[PM8901_VREG_ID_L1]  = REGULATOR_SUPPLY("8901_l1",  NULL),
	[PM8901_VREG_ID_L2]  = REGULATOR_SUPPLY("8901_l2",  NULL),
	[PM8901_VREG_ID_L3]  = REGULATOR_SUPPLY("8901_l3",  NULL),
	[PM8901_VREG_ID_L4]  = REGULATOR_SUPPLY("8901_l4",  NULL),
	[PM8901_VREG_ID_L5]  = REGULATOR_SUPPLY("8901_l5",  NULL),
	[PM8901_VREG_ID_L6]  = REGULATOR_SUPPLY("8901_l6",  NULL),

	[PM8901_VREG_ID_S0] = REGULATOR_SUPPLY("8901_s0", NULL),
	[PM8901_VREG_ID_S1] = REGULATOR_SUPPLY("8901_s1", NULL),
	[PM8901_VREG_ID_S3] = REGULATOR_SUPPLY("8901_s3", NULL),
	[PM8901_VREG_ID_S4] = REGULATOR_SUPPLY("8901_s4", NULL),

	[PM8901_VREG_ID_LVS0]     = REGULATOR_SUPPLY("8901_lvs0",     NULL),
	[PM8901_VREG_ID_LVS1]     = REGULATOR_SUPPLY("8901_lvs1",     NULL),
	[PM8901_VREG_ID_LVS2]     = REGULATOR_SUPPLY("8901_lvs2",     NULL),
	[PM8901_VREG_ID_LVS3]     = REGULATOR_SUPPLY("8901_lvs3",     NULL),
	[PM8901_VREG_ID_MVS0]     = REGULATOR_SUPPLY("8901_mvs0",     NULL),
	[PM8901_VREG_ID_USB_OTG]  = REGULATOR_SUPPLY("8901_usb_otg",  NULL),
	[PM8901_VREG_ID_HDMI_MVS] = REGULATOR_SUPPLY("8901_hdmi_mvs", NULL),
};

#define PM8901_VREG_INIT(_id, _min_uV, _max_uV, \
		_modes, _ops, _apply_uV, _init) \
	[_id] = { \
		.constraints = { \
			.valid_modes_mask = _modes, \
			.valid_ops_mask = _ops, \
			.min_uV = _min_uV, \
			.max_uV = _max_uV, \
			.apply_uV = _apply_uV, \
		}, \
		.num_consumer_supplies = 1, \
		.consumer_supplies = &pm8901_vreg_supply[_id], \
		.regulator_init = _init, \
	}

#define PM8901_VREG_INIT_LDO(_id, _min_uV, _max_uV) \
	PM8901_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_NORMAL | \
			REGULATOR_MODE_IDLE | REGULATOR_MODE_STANDBY, \
			REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS | \
			REGULATOR_CHANGE_MODE, 1, NULL)

#define PM8901_VREG_INIT_SMPS(_id, _min_uV, _max_uV) \
	PM8901_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_NORMAL | \
			REGULATOR_MODE_IDLE | REGULATOR_MODE_STANDBY, \
			REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS | \
			REGULATOR_CHANGE_MODE, 1, NULL)

#define PM8901_VREG_INIT_VS(_id, _init) \
	PM8901_VREG_INIT(_id, 0, 0, REGULATOR_MODE_NORMAL, \
			REGULATOR_CHANGE_STATUS, 0, _init)

static struct regulator_init_data pm8901_vreg_init[PM8901_VREG_MAX] = {
	PM8901_VREG_INIT_LDO(PM8901_VREG_ID_L0, 1200000, 1200000),
	PM8901_VREG_INIT_LDO(PM8901_VREG_ID_L1, 3300000, 3300000),
	PM8901_VREG_INIT_LDO(PM8901_VREG_ID_L2, 3300000, 3300000),
	PM8901_VREG_INIT_LDO(PM8901_VREG_ID_L3, 3300000, 3300000),
	PM8901_VREG_INIT_LDO(PM8901_VREG_ID_L4, 2600000, 2600000),
	PM8901_VREG_INIT_LDO(PM8901_VREG_ID_L5, 2850000, 2850000),
	PM8901_VREG_INIT_LDO(PM8901_VREG_ID_L6, 2200000, 2200000),

	PM8901_VREG_INIT_SMPS(PM8901_VREG_ID_S0, 1100000, 1100000),
	PM8901_VREG_INIT_SMPS(PM8901_VREG_ID_S1, 1100000, 1100000),
	PM8901_VREG_INIT_SMPS(PM8901_VREG_ID_S3, 1100000, 1100000),
	PM8901_VREG_INIT_SMPS(PM8901_VREG_ID_S4, 1100000, 1100000),

	PM8901_VREG_INIT_VS(PM8901_VREG_ID_LVS0,     NULL),
	PM8901_VREG_INIT_VS(PM8901_VREG_ID_LVS1,     NULL),
	PM8901_VREG_INIT_VS(PM8901_VREG_ID_LVS2,     NULL),
	PM8901_VREG_INIT_VS(PM8901_VREG_ID_LVS3,     NULL),
	PM8901_VREG_INIT_VS(PM8901_VREG_ID_MVS0,     NULL),
	PM8901_VREG_INIT_VS(PM8901_VREG_ID_USB_OTG,  pm8901_mpp0_init),
	PM8901_VREG_INIT_VS(PM8901_VREG_ID_HDMI_MVS, NULL),
};

#define PM8901_VREG(_id) { \
	.name = "pm8901-regulator", \
	.id = _id, \
	.platform_data = &pm8901_vreg_init[_id], \
	.data_size = sizeof(pm8901_vreg_init[_id]), \
}

static struct mfd_cell pm8901_subdevs[] = {
	{	.name = "pm8901-mpp",
		.id		= -1,
		.platform_data	= &pm8901_mpp_data,
		.data_size	= sizeof(pm8901_mpp_data),
	},
	PM8901_VREG(PM8901_VREG_ID_L0),
	PM8901_VREG(PM8901_VREG_ID_L1),
	PM8901_VREG(PM8901_VREG_ID_L2),
	PM8901_VREG(PM8901_VREG_ID_L3),
	PM8901_VREG(PM8901_VREG_ID_L4),
	PM8901_VREG(PM8901_VREG_ID_L5),
	PM8901_VREG(PM8901_VREG_ID_L6),
	PM8901_VREG(PM8901_VREG_ID_S0),
	PM8901_VREG(PM8901_VREG_ID_S1),
	PM8901_VREG(PM8901_VREG_ID_S3),
	PM8901_VREG(PM8901_VREG_ID_S4),
	PM8901_VREG(PM8901_VREG_ID_LVS0),
	PM8901_VREG(PM8901_VREG_ID_LVS1),
	PM8901_VREG(PM8901_VREG_ID_LVS2),
	PM8901_VREG(PM8901_VREG_ID_LVS3),
	PM8901_VREG(PM8901_VREG_ID_MVS0),
	PM8901_VREG(PM8901_VREG_ID_USB_OTG),
	PM8901_VREG(PM8901_VREG_ID_HDMI_MVS),
};

static struct pm8901_platform_data pm8901_platform_data = {
	.irq_base = PM8901_IRQ_BASE,
	.num_subdevs = ARRAY_SIZE(pm8901_subdevs),
	.sub_devices = pm8901_subdevs,
};

static struct i2c_board_info pm8901_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("pm8901-core", 0),
		.irq = MSM_GPIO_TO_INT(PM8901_GPIO_INT),
		.platform_data = &pm8901_platform_data,
	},
};

#endif /* CONFIG_PMIC8901 */

unsigned long clk_get_max_axi_khz(void)
{
	return 0;
}

#ifdef CONFIG_I2C
#define I2C_SURF 1
#define I2C_FFA  (1 << 1)
#define I2C_RUMI (1 << 2)
#define I2C_SIM  (1 << 3)

struct i2c_registry {
	u8                     machs;
	int                    bus;
	struct i2c_board_info *info;
	int                    len;
};

static struct i2c_registry msm8x60_i2c_devices[] __initdata = {
#ifdef CONFIG_PMIC8058
	{
		I2C_SURF | I2C_FFA,
		MSM_SSBI1_I2C_BUS_ID,
		pm8058_boardinfo,
		ARRAY_SIZE(pm8058_boardinfo),
	},
#endif
#ifdef CONFIG_PMIC8901
	{
		I2C_SURF | I2C_FFA,
		MSM_SSBI2_I2C_BUS_ID,
		pm8901_boardinfo,
		ARRAY_SIZE(pm8901_boardinfo),
	},
#endif
#if defined(CONFIG_GPIO_SX150X) || defined(CONFIG_GPIO_SX150X_MODULE)
	{
		I2C_SURF | I2C_FFA,
		MSM_GSBI8_QUP_I2C_BUS_ID,
		core_expanders_i2c_info,
		ARRAY_SIZE(core_expanders_i2c_info),
	},
	{
		I2C_SURF,
		MSM_GSBI8_QUP_I2C_BUS_ID,
		surf_expanders_i2c_info,
		ARRAY_SIZE(surf_expanders_i2c_info),
	},
	{
		I2C_SURF | I2C_FFA,
		MSM_GSBI3_QUP_I2C_BUS_ID,
		fha_expanders_i2c_info,
		ARRAY_SIZE(fha_expanders_i2c_info),
	},
#endif
#if defined(CONFIG_TOUCHDISC_VTD518_SHINETSU) || \
		defined(CONFIG_TOUCHDISC_VTD518_SHINETSU_MODULE)
	{
		I2C_SURF | I2C_FFA,
		MSM_GSBI3_QUP_I2C_BUS_ID,
		msm_i2c_gsbi3_tdisc_info,
		ARRAY_SIZE(msm_i2c_gsbi3_tdisc_info),
	},
#endif
	{
		I2C_SURF | I2C_FFA,
		MSM_GSBI3_QUP_I2C_BUS_ID,
		cy8ctmg200_board_info,
		ARRAY_SIZE(cy8ctmg200_board_info),
	},
#ifdef CONFIG_MSM_CAMERA
    {
		I2C_SURF | I2C_FFA,
		MSM_GSBI4_QUP_I2C_BUS_ID,
		msm_camera_boardinfo,
		ARRAY_SIZE(msm_camera_boardinfo),
	},
#endif
	{
		I2C_SURF | I2C_FFA,
		MSM_GSBI7_QUP_I2C_BUS_ID,
		msm_i2c_gsbi7_timpani_info,
		ARRAY_SIZE(msm_i2c_gsbi7_timpani_info),
	},
};
#endif /* CONFIG_I2C */

static void fixup_i2c_configs(void)
{
#ifdef CONFIG_I2C
#if defined(CONFIG_GPIO_SX150X) || defined(CONFIG_GPIO_SX150X_MODULE)
	if (machine_is_msm8x60_surf())
		sx150x_data[0].irq_summary = PM8058_GPIO_IRQ(PM8058_IRQ_BASE,
							     UI_INT2_N);
	else if (machine_is_msm8x60_ffa())
		sx150x_data[0].irq_summary = PM8058_GPIO_IRQ(PM8058_IRQ_BASE,
							     UI_INT1_N);
#endif
#endif
}

static void register_i2c_devices(void)
{
#ifdef CONFIG_I2C
	u8 mach_mask = 0;
	int i;

	/* Build the matching 'supported_machs' bitmask */
	if (machine_is_msm8x60_surf())
		mach_mask = I2C_SURF;
	else if (machine_is_msm8x60_ffa())
		mach_mask = I2C_FFA;
	else if (machine_is_msm8x60_rumi3())
		mach_mask = I2C_RUMI;
	else if (machine_is_msm8x60_sim())
		mach_mask = I2C_SIM;
	else
		pr_err("unmatched machine ID in register_i2c_devices\n");

	/* Run the array and install devices as appropriate */
	for (i = 0; i < ARRAY_SIZE(msm8x60_i2c_devices); ++i) {
		if (msm8x60_i2c_devices[i].machs & mach_mask)
			i2c_register_board_info(msm8x60_i2c_devices[i].bus,
						msm8x60_i2c_devices[i].info,
						msm8x60_i2c_devices[i].len);
	}
#endif
}

static void __init msm8x60_init_uart12dm(void)
{
	void *fpga_mem = ioremap_nocache(0x1D000000, SZ_4K);
	/* Advanced mode */
	writew(0xFFFF, fpga_mem + 0x15C);
	/* FPGA_UART_SEL */
	writew(0, fpga_mem + 0x172);
	/* FPGA_GPIO_CONFIG_117 */
	writew(1, fpga_mem + 0xEA);
	/* FPGA_GPIO_CONFIG_118 */
	writew(1, fpga_mem + 0xEC);
	dmb();
	iounmap(fpga_mem);
}

static void __init msm8x60_init_buses(void)
{
#ifdef CONFIG_I2C_QUP
	msm_gsbi3_qup_i2c_device.dev.platform_data = &msm_gsbi3_qup_i2c_pdata;
	msm_gsbi4_qup_i2c_device.dev.platform_data = &msm_gsbi4_qup_i2c_pdata;
	msm_gsbi7_qup_i2c_device.dev.platform_data = &msm_gsbi7_qup_i2c_pdata;
	msm_gsbi8_qup_i2c_device.dev.platform_data = &msm_gsbi8_qup_i2c_pdata;
	msm_gsbi9_qup_i2c_device.dev.platform_data = &msm_gsbi9_qup_i2c_pdata;
#endif
#if defined(CONFIG_SPI_QUP) || defined(CONFIG_SPI_QUP_MODULE)
	msm_gsbi1_qup_spi_device.dev.platform_data = &msm_gsbi1_qup_spi_pdata;
#endif
#ifdef CONFIG_I2C_SSBI
	msm_device_ssbi1.dev.platform_data = &msm_ssbi1_pdata;
	msm_device_ssbi2.dev.platform_data = &msm_ssbi2_pdata;
	msm_device_ssbi3.dev.platform_data = &msm_ssbi3_pdata;
#endif
#if defined(CONFIG_USB_GADGET_MSM_72K) || defined(CONFIG_USB_EHCI_HCD)
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
#endif
#ifdef CONFIG_SERIAL_MSM_HS
	msm_uart_dm1_pdata.wakeup_irq = gpio_to_irq(54); /* GSBI6(2) */
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#endif
}

static void __init msm8x60_map_io(void)
{
	msm_shared_ram_phys = MSM_SHARED_RAM_PHYS;
	msm_map_msm8x60_io();
	msm8x60_allocate_memory_regions();
	msm_clock_init(msm_clocks_8x60, msm_num_clocks_8x60);
}

static void __init msm8x60_init_irq(void)
{
	unsigned int i;

	gic_dist_init(0, MSM_QGIC_DIST_BASE, GIC_PPI_START);
	gic_cpu_base_addr = (void *)MSM_QGIC_CPU_BASE;
	gic_cpu_init(0, MSM_QGIC_CPU_BASE);

	/* Edge trigger PPIs except AVS_SVICINT and AVS_SVICINTSWDONE */
	writel(0xFFFFD7FF, MSM_QGIC_DIST_BASE + GIC_DIST_CONFIG + 4);

	/* RUMI does not adhere to GIC spec by enabling STIs by default.
	 * Enable/clear is supposed to be RO for STIs, but is RW on RUMI.
	 */
	if (machine_is_msm8x60_surf() ||
	    machine_is_msm8x60_ffa()  ||
	    machine_is_msm8x60_rumi3())
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

/*
 * Most segments of the EBI2 bus are disabled by default.
 */
static void __init msm8x60_init_ebi2(void)
{
	uint32_t ebi2_cfg;
	void *ebi2_cfg_ptr;

	ebi2_cfg_ptr = ioremap_nocache(0x1a100000, sizeof(uint32_t));
	if (ebi2_cfg_ptr != 0) {
		ebi2_cfg = readl(ebi2_cfg_ptr);

		if (machine_is_msm8x60_surf() || machine_is_msm8x60_ffa())
			ebi2_cfg |= (1 << 4) | (1 << 5); /* CS2, CS3 */
		else if (machine_is_msm8x60_sim())
			ebi2_cfg |= (1 << 4); /* CS2 */
		else if (machine_is_msm8x60_rumi3())
			ebi2_cfg |= (1 << 5); /* CS3 */

		writel(ebi2_cfg, ebi2_cfg_ptr);
		iounmap(ebi2_cfg_ptr);
	}

	if (machine_is_msm8x60_surf() || machine_is_msm8x60_ffa()) {
		ebi2_cfg_ptr = ioremap_nocache(0x1a110000, SZ_4K);
		if (ebi2_cfg_ptr != 0) {
			/* EBI2_XMEM_CFG:PWRSAVE_MODE off */
			writel(0UL, ebi2_cfg_ptr);

			/* CS2: Delay 9 cycles (140ns@64MHz) between SMSC
			 * LAN9221 Ethernet controller reads and writes.
			 * The lowest 4 bits are the read delay, the next
			 * 4 are the write delay. */
			writel(0x031F1C99, ebi2_cfg_ptr + 0x10);

			/* EBI2 CS3 muxed address/data,
			 * two cyc addr enable */
			writel(0xA3030020, ebi2_cfg_ptr + 0x34);
			iounmap(ebi2_cfg_ptr);
		}
	}
}

static void __init msm8x60_configure_smc91x(void)
{
	if (machine_is_msm8x60_sim()) {

		smc91x_resources[0].start = 0x1b800300;
		smc91x_resources[0].end   = 0x1b8003ff;

		smc91x_resources[1].start = (NR_MSM_IRQS + 40);
		smc91x_resources[1].end   = (NR_MSM_IRQS + 40);

	} else if (machine_is_msm8x60_rumi3()) {

		smc91x_resources[0].start = 0x1d000300;
		smc91x_resources[0].end   = 0x1d0003ff;

		smc91x_resources[1].start = TLMM_SCSS_DIR_CONN_IRQ_0;
		smc91x_resources[1].end   = TLMM_SCSS_DIR_CONN_IRQ_0;
	}
}

struct msm8x60_tlmm_cfg_struct {
	unsigned gpio;
	u32      flags;
};

static uint32_t msm8x60_tlmm_cfgs[] = {
	/* PS_HOLD */
	GPIO_CFG(92, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_12MA),
	/*
	 * EBI2
	 */
	/* address lines */
	GPIO_CFG(123, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(124, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(125, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(126, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(127, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(128, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(129, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(130, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	/* A_D lines */
	GPIO_CFG(135, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(136, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(137, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(138, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(139, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(140, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(141, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(142, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(143, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(144, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(145, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(146, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(147, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(148, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(149, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(150, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	/* OE */
	GPIO_CFG(151, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	/* WE */
	GPIO_CFG(157, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	/* CS2 */
	GPIO_CFG(40, 2, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	/* CS3 */
	GPIO_CFG(133, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	/* ADV */
	GPIO_CFG(153, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	/*
	 * SD/MMC Slot-1 (CLK, CMD, D0-D7)
	 */
	GPIO_CFG(167, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
	GPIO_CFG(168, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
	GPIO_CFG(159, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
	GPIO_CFG(160, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
	GPIO_CFG(161, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
	GPIO_CFG(162, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
#ifdef CONFIG_MMC_MSM_SDC1_8_BIT_SUPPORT
	GPIO_CFG(163, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
	GPIO_CFG(164, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
	GPIO_CFG(165, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
	GPIO_CFG(166, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
#endif
#endif

#ifdef CONFIG_MMC_MSM_SDC5_SUPPORT
	/*
	 * SD/MMC Slot-5 (CLK, CMD, D0-D3)
	 */
	GPIO_CFG(97,  2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
	GPIO_CFG(95,  2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
	GPIO_CFG(100, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
	GPIO_CFG(99,  2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
	GPIO_CFG(98,  2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
	GPIO_CFG(96,  2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
#endif

#ifdef CONFIG_I2C_QUP
	/* GSBI3 QUP I2C */
	GPIO_CFG(43, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(44, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	/* GSBI7 QUP I2C (Marimba) */
	GPIO_CFG(59, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(60, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	/* GSBI8 QUP I2C */
	GPIO_CFG(64, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(65, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
#ifdef CONFIG_MSM_CAMERA
	/* GSBI4 QUP I2C */
	GPIO_CFG(47, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(48, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
#endif
#endif

	/*
	 * GSBI12
	 */
	/* UARTDM_RFR */
	GPIO_CFG(115, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	/* UARTDM_CTS */
	GPIO_CFG(116, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	/* UARTDM_RX */
	GPIO_CFG(117, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	/* UARTDM_TX */
	GPIO_CFG(118, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),

#ifdef CONFIG_PMIC8058
	/* PMIC8058 */
	GPIO_CFG(PM8058_GPIO_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
#endif

#if defined(CONFIG_SPI_QUP) || defined(CONFIG_SPI_QUP_MODULE)
	/* GSBI1 QUP SPI */
	GPIO_CFG(33, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(34, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(35, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(36, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
#endif
#ifdef CONFIG_SERIAL_MSM_HS
	/* UARTDM_TX */
	GPIO_CFG(53, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	/* UARTDM_RX */
	GPIO_CFG(54, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	/* UARTDM_CTS */
	GPIO_CFG(55, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	/* UARTDM_RFR */
	GPIO_CFG(56, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
#endif
#ifdef CONFIG_PMIC8901
	/* PMIC8901 */
	GPIO_CFG(PM8901_GPIO_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
#endif

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
	/* gpio_cec_var */
	GPIO_CFG(169, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
	/* gpio_ddc_clk */
	GPIO_CFG(170, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
	/* gpio_ddc_data */
	GPIO_CFG(171, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
	/* gpio_hpd */
	GPIO_CFG(172, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */
};

static uint32_t msm8x60_hdrive_cfgs[][2] = {
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	{TLMM_HDRV_SDC3_CLK, GPIO_CFG_8MA},
	{TLMM_HDRV_SDC3_CMD, GPIO_CFG_8MA},
	{TLMM_HDRV_SDC3_DATA, GPIO_CFG_8MA},
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	{TLMM_HDRV_SDC4_CLK, GPIO_CFG_8MA},
	{TLMM_HDRV_SDC4_CMD, GPIO_CFG_8MA},
	{TLMM_HDRV_SDC4_DATA, GPIO_CFG_8MA},
#endif
};

static uint32_t msm8x60_pull_cfgs[][2] = {
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	{TLMM_PULL_SDC3_CMD, GPIO_CFG_PULL_UP},
	{TLMM_PULL_SDC3_DATA, GPIO_CFG_PULL_UP},
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	{TLMM_PULL_SDC4_CMD, GPIO_CFG_PULL_UP},
	{TLMM_PULL_SDC4_DATA, GPIO_CFG_PULL_UP},
#endif
};

static void __init msm8x60_init_tlmm(void)
{
	unsigned n;

	if (machine_is_msm8x60_rumi3())
		msm_gpio_install_direct_irq(0, 0);
	else if (machine_is_msm8x60_surf() || machine_is_msm8x60_ffa()) {
		for (n = 0; n < ARRAY_SIZE(msm8x60_tlmm_cfgs); ++n)
			gpio_tlmm_config(msm8x60_tlmm_cfgs[n], 0);
		for (n = 0; n < ARRAY_SIZE(msm8x60_hdrive_cfgs); ++n)
			msm_tlmm_set_hdrive(msm8x60_hdrive_cfgs[n][0],
					msm8x60_hdrive_cfgs[n][1]);
		for (n = 0; n < ARRAY_SIZE(msm8x60_pull_cfgs); ++n)
			msm_tlmm_set_pull(msm8x60_pull_cfgs[n][0],
					msm8x60_pull_cfgs[n][1]);
	}
}

#define GPIO_SDC3_WP_SWITCH (GPIO_EXPANDER_GPIO_BASE + (16 * 1) + 6)
#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC5_SUPPORT))

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);

	/* Handle VREGs here once VREG support is available */

	return rc;
}

static int msm_sdc3_get_wpswitch(struct device *dev)
{
	struct platform_device *pdev;
	int status;
	pdev = container_of(dev, struct platform_device, dev);

	status = gpio_request(GPIO_SDC3_WP_SWITCH, "SD_WP_Switch");
	if (status) {
		pr_err("%s:Failed to request GPIO %d\n",
					__func__, GPIO_SDC3_WP_SWITCH);
	} else {
		status = gpio_get_value(GPIO_SDC3_WP_SWITCH);
		pr_info("%s: WP Status for Slot %d = %d\n", __func__,
							pdev->id, status);
		gpio_free(GPIO_SDC3_WP_SWITCH);
	}
	return (unsigned int) status;
}
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
static unsigned int msm8x60_sdcc_slot_status(struct device *dev)
{
	int status;

	status = !(gpio_get_value(
			PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC3_DET - 1)));
	return (unsigned int) status;
}
#endif
#endif
#endif

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static struct mmc_platform_data msm8x60_sdc1_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
#ifdef CONFIG_MMC_MSM_SDC1_8_BIT_SUPPORT
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
#else
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#endif
	.msmsdcc_fmin	= 400000,
	.msmsdcc_fmid	= 24000000,
	.msmsdcc_fmax	= 48000000,
	.nonremovable	= 1,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static struct mmc_platform_data msm8x60_sdc2_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 400000,
	.msmsdcc_fmid	= 24000000,
	.msmsdcc_fmax	= 48000000,
	.nonremovable	= 0,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static struct mmc_platform_data msm8x60_sdc3_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.wpswitch  	= msm_sdc3_get_wpswitch,
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	.status      = msm8x60_sdcc_slot_status,
	.status_irq  = PM8058_GPIO_IRQ(PM8058_IRQ_BASE,
				       PMIC_GPIO_SDC3_DET - 1),
	.irq_flags   = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
#endif
	.msmsdcc_fmin	= 400000,
	.msmsdcc_fmid	= 24000000,
	.msmsdcc_fmax	= 48000000,
	.nonremovable	= 0,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static struct mmc_platform_data msm8x60_sdc4_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 400000,
	.msmsdcc_fmid	= 24000000,
	.msmsdcc_fmax	= 48000000,
	.nonremovable	= 1,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC5_SUPPORT
static struct mmc_platform_data msm8x60_sdc5_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 400000,
	.msmsdcc_fmid	= 24000000,
	.msmsdcc_fmax	= 48000000,
	.nonremovable	= 0,
};
#endif

static void __init msm8x60_init_mmc(void)
{
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	msm_add_sdcc(1, &msm8x60_sdc1_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	msm_add_sdcc(2, &msm8x60_sdc2_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	msm_add_sdcc(3, &msm8x60_sdc3_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	msm_add_sdcc(4, &msm8x60_sdc4_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC5_SUPPORT
	msm_add_sdcc(5, &msm8x60_sdc5_data);
#endif
}

#if !defined(CONFIG_GPIO_SX150X) && !defined(CONFIG_GPIO_SX150X_MODULE)
static inline void display_common_power(int on) {}
#else
/* the LVDS reset line is on the first expander pin 2 */
#define GPIO_LVDS_STDN_OUT_N (GPIO_EXPANDER_GPIO_BASE + 2)
/* the backlight control line is on the first expander pin 12 */
#define GPIO_BACKLIGHT_EN (GPIO_EXPANDER_GPIO_BASE + 12)

static void display_common_power(int on)
{
	static int rc = -EINVAL; /* remember if the gpio_requests succeeded */

	if (machine_is_msm8x60_surf() || machine_is_msm8x60_ffa()) {
		if (on) {
			/* LVDS */
			rc = gpio_request(GPIO_LVDS_STDN_OUT_N,
				"LVDS_STDN_OUT_N");
			if (rc) {
				printk(KERN_ERR "%s: LVDS gpio %d request"
					"failed\n", __func__,
					 GPIO_LVDS_STDN_OUT_N);
				return;
			}

			/* BACKLIGHT */
			rc = gpio_request(GPIO_BACKLIGHT_EN, "BACKLIGHT_EN");
			if (rc) {
				printk(KERN_ERR "%s: BACKLIGHT gpio %d request"
					"failed\n", __func__,
					 GPIO_BACKLIGHT_EN);
				gpio_free(GPIO_LVDS_STDN_OUT_N);
				return;
			}

			gpio_direction_output(GPIO_LVDS_STDN_OUT_N, 0);
			gpio_direction_output(GPIO_BACKLIGHT_EN, 0);
			mdelay(20);
			gpio_set_value_cansleep(GPIO_LVDS_STDN_OUT_N, 1);
			gpio_set_value_cansleep(GPIO_BACKLIGHT_EN, 1);

		} else {
			if (!rc) {
				/* BACKLIGHT */
				gpio_set_value_cansleep(GPIO_BACKLIGHT_EN, 0);
				/* LVDS */
				gpio_set_value_cansleep(GPIO_LVDS_STDN_OUT_N,
				0);
				mdelay(20);
				gpio_free(GPIO_BACKLIGHT_EN);
				gpio_free(GPIO_LVDS_STDN_OUT_N);
			}
		}

	}
}
#endif

static uint32_t lcd_panel_gpios[] = {
	GPIO_CFG(0,  1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* lcdc_pclk */
	GPIO_CFG(1,  1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* lcdc_hsync*/
	GPIO_CFG(2,  1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* lcdc_vsync*/
	GPIO_CFG(3,  1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* lcdc_den */
	GPIO_CFG(4,  1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* lcdc_red7 */
	GPIO_CFG(5,  1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* lcdc_red6 */
	GPIO_CFG(6,  1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* lcdc_red5 */
	GPIO_CFG(7,  1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* lcdc_red4 */
	GPIO_CFG(8,  1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* lcdc_red3 */
	GPIO_CFG(9,  1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* lcdc_red2 */
	GPIO_CFG(10, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* lcdc_red1 */
	GPIO_CFG(11, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* lcdc_red0 */
	GPIO_CFG(12, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* lcdc_grn7 */
	GPIO_CFG(13, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* lcdc_grn6 */
	GPIO_CFG(14, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* lcdc_grn5 */
	GPIO_CFG(15, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* lcdc_grn4 */
	GPIO_CFG(16, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* lcdc_grn3 */
	GPIO_CFG(17, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* lcdc_grn2 */
	GPIO_CFG(18, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* lcdc_grn1 */
	GPIO_CFG(19, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* lcdc_grn0 */
	GPIO_CFG(20, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* lcdc_blu7 */
	GPIO_CFG(21, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* lcdc_blu6 */
	GPIO_CFG(22, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* lcdc_blu5 */
	GPIO_CFG(23, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* lcdc_blu4 */
	GPIO_CFG(24, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* lcdc_blu3 */
	GPIO_CFG(25, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* lcdc_blu2 */
	GPIO_CFG(26, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* lcdc_blu1 */
	GPIO_CFG(27, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* lcdc_blu0 */
};


static void lcdc_samsung_panel_power(int on)
{
	int n;

	display_common_power(on);

	/*TODO if on = 0 free the gpio's */
	for (n = 0; n < ARRAY_SIZE(lcd_panel_gpios); ++n)
		gpio_tlmm_config(lcd_panel_gpios[n], 0);
}

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
static struct regulator *reg_8058_l16;
static struct regulator *reg_8901_l3;
static struct regulator *reg_8901_hdmi_mvs;
static int __init hdmi_msm_init(void)
{
	#define _GET_REGULATOR(var, name) do {				\
		var = regulator_get(NULL, name);			\
		if (IS_ERR(var)) {					\
			pr_err("'%s' regulator not found, rc=%ld\n",	\
				name, IS_ERR(var));			\
			var = NULL;					\
			return -ENODEV;					\
		}							\
	} while (0)

	_GET_REGULATOR(reg_8058_l16, "8058_l16");
	_GET_REGULATOR(reg_8901_l3, "8901_l3");
	_GET_REGULATOR(reg_8901_hdmi_mvs, "8901_hdmi_mvs");

	#undef _GET_REGULATOR

	return 0;
}

static int hdmi_msm_regulators(int on)
{
	int rc;

	if (!reg_8058_l16 || !reg_8901_l3 || !reg_8901_hdmi_mvs) {
		if (hdmi_msm_init()) {
			pr_err("%s: failed, HDMI regulators not initialized\n",
				__func__);
			return -ENODEV;
		}
	}

	if (on) {
		rc = regulator_enable(reg_8058_l16);
		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
				"8058_l16", rc);
			return rc;
		}
		rc = regulator_enable(reg_8901_l3);
		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
				"8901_l3", rc);
			return rc;
		}
		rc = regulator_enable(reg_8901_hdmi_mvs);
		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
				"8901_hdmi_mvs", rc);
			return rc;
		}
		pr_info("%s(on): success\n", __func__);
	} else {
		rc = regulator_disable(reg_8058_l16);
		if (rc)
			pr_warning("'%s' regulator disable failed, rc=%d\n",
				"8058_l16", rc);

		rc = regulator_disable(reg_8901_l3);
		if (rc)
			pr_warning("'%s' regulator disable failed, rc=%d\n",
				"8901_l3", rc);

		rc = regulator_disable(reg_8901_hdmi_mvs);
		if (rc)
			pr_warning("'%s' regulator disable failed, rc=%d\n",
				"8901_hdmi_mvs", rc);
		pr_info("%s(off): done\n", __func__);
	}

	return 0;
}
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */

static int lcdc_panel_power(int on)
{
	int flag_on = !!on;
	static int lcdc_power_save_on;

	if (lcdc_power_save_on == flag_on)
		return 0;

	lcdc_power_save_on = flag_on;

	lcdc_samsung_panel_power(on);

	return 0;
}

static struct lcdc_platform_data lcdc_pdata = {
	.lcdc_power_save   = lcdc_panel_power,
#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
	/* TODO: consider moving this into the hdmi platform data */
	.lcdc_gpio_config  = hdmi_msm_regulators,
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */
};

static struct msm_panel_common_pdata mdp_pdata = {
	.mdp_core_clk_rate = 200000000,
};

static void __init msm_fb_add_devices(void)
{
	if (machine_is_msm8x60_rumi3())
		msm_fb_register_device("mdp", NULL);
	else
		msm_fb_register_device("mdp", &mdp_pdata);

	msm_fb_register_device("lcdc", &lcdc_pdata);
	msm_fb_register_device("mipi_dsi", 0);
}

static void __init msm8x60_cfg_smsc911x(void)
{
	smsc911x_resources[1].start =
		PM8058_GPIO_IRQ(PM8058_IRQ_BASE, 6);
	smsc911x_resources[1].end =
		PM8058_GPIO_IRQ(PM8058_IRQ_BASE, 6);
}

#ifdef CONFIG_MSM_RPM
static struct msm_rpm_platform_data msm_rpm_data = {
	.reg_base_addrs = {
		[MSM_RPM_PAGE_STATUS] = MSM_RPM_BASE,
		[MSM_RPM_PAGE_CTRL] = MSM_RPM_BASE + 0x400,
		[MSM_RPM_PAGE_REQ] = MSM_RPM_BASE + 0x600,
		[MSM_RPM_PAGE_ACK] = MSM_RPM_BASE + 0xa00,
	},

	.irq_ack = RPM_SCSS_CPU0_GP_HIGH_IRQ,
	.irq_err = RPM_SCSS_CPU0_GP_LOW_IRQ,
	.irq_vmpm = RPM_SCSS_CPU0_GP_MEDIUM_IRQ,
};
#endif

static void __init msm8x60_init(void)
{
	/*
	 * Initialize RPM first as other drivers and devices may need
	 * it for their initialization.
	 */
#ifdef CONFIG_MSM_RPM
	BUG_ON(msm_rpm_init(&msm_rpm_data));
#endif
	/* initialize SPM before acpuclock as the latter calls into SPM
	 * driver to set ACPU voltages.
	 */
	msm_spm_init(msm_spm_data, ARRAY_SIZE(msm_spm_data));
	/* CPU frequency control is not supported on simulated targets. */
	if (!machine_is_msm8x60_rumi3() && !machine_is_msm8x60_sim())
		msm_acpu_clock_init(&msm8x60_acpu_clock_data);

	platform_add_devices(early_devices, ARRAY_SIZE(early_devices));
	msm8x60_init_ebi2();
	msm8x60_init_tlmm();
	msm8x60_init_uart12dm();
	msm8x60_init_mmc();
	msm8x60_init_buses();
	if (machine_is_msm8x60_surf() || machine_is_msm8x60_ffa()) {
		msm8x60_cfg_smsc911x();
		platform_add_devices(surf_devices,
				     ARRAY_SIZE(surf_devices));
#ifdef CONFIG_USB_EHCI_MSM
		msm_add_host(0, &msm_usb_host_pdata);
#endif
	} else {
		msm8x60_configure_smc91x();
		platform_add_devices(rumi_sim_devices,
				     ARRAY_SIZE(rumi_sim_devices));
	}
	if (!machine_is_msm8x60_sim())
		msm_fb_add_devices();
	fixup_i2c_configs();
	register_i2c_devices();

	msm_pm_set_platform_data(msm_pm_data, ARRAY_SIZE(msm_pm_data));
	msm_cpuidle_set_states(msm_cstates, ARRAY_SIZE(msm_cstates),
				msm_pm_data);

}

MACHINE_START(MSM8X60_RUMI3, "QCT MSM8X60 RUMI3")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.map_io = msm8x60_map_io,
	.init_irq = msm8x60_init_irq,
	.init_machine = msm8x60_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(MSM8X60_SIM, "QCT MSM8X60 SIMULATOR")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.map_io = msm8x60_map_io,
	.init_irq = msm8x60_init_irq,
	.init_machine = msm8x60_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(MSM8X60_SURF, "QCT MSM8X60 SURF")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io = MSM_DEBUG_UART_PHYS
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.map_io = msm8x60_map_io,
	.init_irq = msm8x60_init_irq,
	.init_machine = msm8x60_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(MSM8X60_FFA, "QCT MSM8X60 FFA")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io = MSM_DEBUG_UART_PHYS
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.map_io = msm8x60_map_io,
	.init_irq = msm8x60_init_irq,
	.init_machine = msm8x60_init,
	.timer = &msm_timer,
MACHINE_END
