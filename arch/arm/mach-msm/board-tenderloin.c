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
#include <linux/pmic8058-pwrkey.h>
#include <linux/pmic8058-vibrator.h>
#include <linux/leds.h>
#include <linux/mfd/pmic8901.h>
#include <linux/regulator/pmic8058-regulator.h>
#include <linux/regulator/pmic8901-regulator.h>
#include <linux/bootmem.h>
#include <linux/pwm.h>
#include <linux/pmic8058-pwm.h>
#include <linux/leds-pmic8058.h>
#include <linux/clk.h>
#include <linux/msm-charger.h>
#include <linux/i2c_lm8502_led.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/cy8ctma395.h>
#include <linux/i2c/lsm303dlh.h>
#include <linux/mpu.h>
#include <linux/isl29023.h>

#ifdef CONFIG_ANDROID_PMEM
#include <linux/android_pmem.h>
#endif

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>

#include <mach/dma.h>
#include <mach/camera.h>
#include <mach/mpp.h>
#include <mach/board.h>
#include <mach/irqs.h>
#include <mach/msm_spi.h>
#include <mach/msm_serial_hs.h>
#include <mach/msm_serial_hs_lite.h>
#include <mach/msm_iomap.h>
#include <asm/mach/mmc.h>
#include <mach/msm_battery.h>
#include <mach/msm_hsusb.h>
#ifdef CONFIG_MSM_DSPS
#include <mach/msm_dsps.h>
#endif
#include <mach/mdm.h>
#include <mach/msm_xo.h>
#include <mach/msm_bus_board.h>
#include <mach/socinfo.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <mach/gpio.h>
#ifdef CONFIG_MAX8903B_CHARGER
#include <linux/max8903b_charger.h>
#endif
#ifdef CONFIG_HRES_COUNTER
#include <linux/hres_counter.h>
#endif
#ifdef CONFIG_A6
#include <linux/a6_sbw_interface.h>
#include <linux/a6.h>
#include "a6_sbw_impl_tenderloin.h"
#endif

#ifdef CONFIG_HSUART
#include <linux/hsuart.h>
#endif

#ifdef CONFIG_USER_PINS
#include <linux/user-pins.h>
#endif

#ifdef CONFIG_KEYBOARD_GPIO_PE
#include <linux/gpio_keys_pe.h>
#endif

#ifdef CONFIG_NDUID
#include <linux/nduid.h>
#endif

#include <linux/i2c/atmel_maxtouch.h>

#ifdef CONFIG_MFD_WM8994
#include <linux/mfd/wm8994/pdata.h>
#include <linux/mfd/wm8994/core.h>
#endif

#ifdef CONFIG_USB_G_ANDROID
#include <linux/usb/android_composite.h>
#endif

#include "devices.h"
#include "devices-msm8x60.h"
#include "devices-tenderloin.h"
#include "cpuidle.h"
#include "pm.h"
#include "rpm.h"
#include "spm.h"
#include "rpm_log.h"
#include "timer.h"
#include "saw-regulator.h"
#include "rpm-regulator.h"
#include "gpiomux.h"
#include "gpiomux-tenderloin.h"
#include "clock-8x60.h"
#include "mpm.h"
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/mdmgpio.h>
#include <mach/htc_usb.h>


#ifdef CONFIG_MAX8903B_CHARGER
static unsigned max8903b_ps_connected = 0;
static unsigned max8903b_vbus_draw_ma = 0;
static unsigned max8903b_vbus_draw_ma_max = 0;
void max8903b_set_vbus_draw (unsigned ma);
#endif

#define MSM_SHARED_RAM_PHYS 0x40000000

/* helper function to manipulate group of gpios (msm_gpiomux)*/
static int configure_gpiomux_gpios(int on, int gpios[], int cnt)
{
	int ret = 0;
	int i;

	for (i = 0; i < cnt; i++) {
		//printk(KERN_ERR "%s:pin(%d):%s\n", __func__, gpios[i], on?"on":"off");
		if (on) {
			ret = msm_gpiomux_get(gpios[i]);
			if (unlikely(ret))
				break;
		} else {
			ret = msm_gpiomux_put(gpios[i]);
			if (unlikely(ret))
				return ret;
		}
	}
	if (ret)
		for (; i >= 0; i--)
			msm_gpiomux_put(gpios[i]);
	return ret;
}
/* helper function to manipulate group of gpios (gpio)*/
static int configure_gpios(int on, int gpios[], int cnt)
{
	int ret = 0;
	int i;

	for (i = 0; i < cnt; i++) {
		//printk(KERN_ERR "%s:pin(%d):%s\n", __func__, gpios[i], on?"request":"free");
		if (on) {
			ret = gpio_request(gpios[i], NULL);
			if (unlikely(ret))
				break;
		} else {
			gpio_free(gpios[i]);
		}
	}
	if (ret)
		for (; i >= 0; i--)
			gpio_free(gpios[i]);
	return ret;
}

/*
 * The UI_INTx_N lines are pmic gpio lines which connect i2c
 * gpio expanders to the pm8058.
 */
#define UI_INT1_N 25
#define UI_INT2_N 34
#define UI_INT3_N 14

enum topaz_board_types {
	TOPAZ_PROTO = 0,
	TOPAZ_PROTO2,
	TOPAZ_EVT1,
	TOPAZ_EVT2,
	TOPAZ_EVT3,
	TOPAZ_DVT,
	TOPAZ_PVT,
	TOPAZ_3G_PROTO,
	TOPAZ_3G_PROTO2,
	TOPAZ_3G_EVT1,
	TOPAZ_3G_EVT2,
	TOPAZ_3G_EVT3,
	TOPAZ_3G_EVT4,
	TOPAZ_3G_DVT,
	TOPAZ_3G_PVT
};

static u32 board_type = TOPAZ_EVT1; /* In case we get a lazy bootloader */

static struct {
	enum topaz_board_types type;
	const char *str;
} boardtype_tbl[] = {
	/* WiFi */
	{TOPAZ_PROTO,    "topaz-Wifi-proto"},
	{TOPAZ_PROTO2,   "topaz-Wifi-proto2"},
	{TOPAZ_EVT1,     "topaz-Wifi-evt1"},
	{TOPAZ_EVT2,     "topaz-Wifi-evt2"},
	{TOPAZ_EVT3,     "topaz-Wifi-evt3"},
	{TOPAZ_DVT,      "topaz-Wifi-dvt"},
	{TOPAZ_PVT,      "topaz-Wifi-pvt"},

	/* 3G */
	{TOPAZ_3G_PROTO,    "topaz-3G-proto"},
	{TOPAZ_3G_PROTO2,   "topaz-3G-proto2"},
	{TOPAZ_3G_EVT1,     "topaz-3G-evt1"},
	{TOPAZ_3G_EVT2,     "topaz-3G-evt2"},
	{TOPAZ_3G_EVT3,     "topaz-3G-evt3"},
	{TOPAZ_3G_EVT4,     "topaz-3G-evt4"},
	{TOPAZ_3G_DVT,      "topaz-3G-dvt"},
	{TOPAZ_3G_PVT,      "topaz-3G-pvt"},

	/* TODO: Non-standard board strings, to be removed once all copies of
	 * bootie in the wild are updated to use the above format */

	/* WiFi */
	{TOPAZ_PROTO,    "topaz-1stbuild-Wifi"},
	{TOPAZ_PROTO2,   "topaz-2ndbuild-Wifi"},
	{TOPAZ_EVT1,     "topaz-3rdbuild-Wifi"},
	{TOPAZ_EVT2,     "topaz-4thbuild-Wifi"},
	{TOPAZ_EVT3,     "topaz-5thbuild-Wifi"},
	{TOPAZ_DVT,      "topaz-6thbuild-Wifi"},
	{TOPAZ_PVT,      "topaz-7thbuild-Wifi"},
	{TOPAZ_PVT,      "topaz-pvt-Wifi"},

	/* 3G */
	{TOPAZ_3G_PROTO,    "topaz-1stbuild-3G"},
	{TOPAZ_3G_PROTO2,   "topaz-2ndbuild-3G"},
	{TOPAZ_3G_EVT1,     "topaz-3rdbuild-3G"},
	{TOPAZ_3G_EVT2,     "topaz-4thbuild-3G"},
	{TOPAZ_3G_EVT3,     "topaz-5thbuild-3G"},
	{TOPAZ_3G_DVT,      "topaz-6thbuild-3G"},
	{TOPAZ_3G_PVT,      "topaz-7thbuild-3G"},
	{TOPAZ_3G_PVT,      "topaz-pvt-3G"}
};

static int __init boardtype_setup(char *boardtype_str)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(boardtype_tbl); i++)
		if (!strcmp(boardtype_str, boardtype_tbl[i].str))
			board_type = boardtype_tbl[i].type;

	return 0;
}
__setup("boardtype=", boardtype_setup);

// Pointer to wifi/3G pin arrays
int *pin_table = NULL;

static u8 boardtype_is_3g(void) {
	if (board_type >= TOPAZ_3G_PROTO) {
		return 1;
	}
	else {
		return 0;
	}
}

static struct msm_spm_platform_data msm_spm_data_v1[] __initdata = {
	[0] = {
		.reg_base_addr = MSM_SAW0_BASE,

#ifdef CONFIG_MSM_AVS_HW
		.reg_init_values[MSM_SPM_REG_SAW_AVS_CTL] = 0x586020FF,
#endif
		.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x0F,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x68,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0xFFFFFFFF,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0xFFFFFFFF,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x07,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

		.awake_vlevel = 0x94,
		.retention_vlevel = 0x81,
		.collapse_vlevel = 0x20,
		.retention_mid_vlevel = 0x94,
		.collapse_mid_vlevel = 0x8C,

		.vctl_timeout_us = 50,
	},

	[1] = {
		.reg_base_addr = MSM_SAW1_BASE,

#ifdef CONFIG_MSM_AVS_HW
		.reg_init_values[MSM_SPM_REG_SAW_AVS_CTL] = 0x586020FF,
#endif
		.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x0F,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x68,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0xFFFFFFFF,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0xFFFFFFFF,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x13,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x07,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

		.awake_vlevel = 0x94,
		.retention_vlevel = 0x81,
		.collapse_vlevel = 0x20,
		.retention_mid_vlevel = 0x94,
		.collapse_mid_vlevel = 0x8C,

		.vctl_timeout_us = 50,
	},
};

static struct msm_spm_platform_data msm_spm_data[] __initdata = {
	[0] = {
		.reg_base_addr = MSM_SAW0_BASE,

#ifdef CONFIG_MSM_AVS_HW
		.reg_init_values[MSM_SPM_REG_SAW_AVS_CTL] = 0x586020FF,
#endif
		.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x1C,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x68,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0x0C0CFFFF,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0x78780FFF,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x07,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

		.awake_vlevel = 0xA0,
		.retention_vlevel = 0x89,
		.collapse_vlevel = 0x20,
		.retention_mid_vlevel = 0x89,
		.collapse_mid_vlevel = 0x89,

		.vctl_timeout_us = 50,
	},

	[1] = {
		.reg_base_addr = MSM_SAW1_BASE,

#ifdef CONFIG_MSM_AVS_HW
		.reg_init_values[MSM_SPM_REG_SAW_AVS_CTL] = 0x586020FF,
#endif
		.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x1C,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x68,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0x0C0CFFFF,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0x78780FFF,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x13,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x07,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

		.awake_vlevel = 0xA0,
		.retention_vlevel = 0x89,
		.collapse_vlevel = 0x20,
		.retention_mid_vlevel = 0x89,
		.collapse_mid_vlevel = 0x89,

		.vctl_timeout_us = 50,
	},
};

static struct msm_acpu_clock_platform_data msm8x60_acpu_clock_data = {
};

static struct regulator_consumer_supply saw_s0_supply =
	REGULATOR_SUPPLY("8901_s0", NULL);
static struct regulator_consumer_supply saw_s1_supply =
	REGULATOR_SUPPLY("8901_s1", NULL);

static struct regulator_init_data saw_s0_init_data = {
		.constraints = {
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.min_uV = 840000,
			.max_uV = 1350000,
		},
		.num_consumer_supplies = 1,
		.consumer_supplies = &saw_s0_supply,
};

static struct regulator_init_data saw_s1_init_data = {
		.constraints = {
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.min_uV = 840000,
			.max_uV = 1350000,
		},
		.num_consumer_supplies = 1,
		.consumer_supplies = &saw_s1_supply,
};

static struct platform_device msm_device_saw_s0 = {
	.name          = "saw-regulator",
	.id            = SAW_VREG_ID_S0,
	.dev           = {
		.platform_data = &saw_s0_init_data,
	},
};

static struct platform_device msm_device_saw_s1 = {
	.name          = "saw-regulator",
	.id            = SAW_VREG_ID_S1,
	.dev           = {
		.platform_data = &saw_s1_init_data,
	},
};

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR * 2] = {
	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_POWER_COLLAPSE)] = {
		.supported = 1,
		.suspend_enabled = 0,
		.idle_enabled = 0,
		.latency = 4000,
		.residency = 13000,
	},

	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE)] = {
		.supported = 1,
		.suspend_enabled = 0,
		.idle_enabled = 0,
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
		.suspend_enabled = 0,
		.idle_enabled = 0,
		.latency = 600,
		.residency = 7200,
	},

	[MSM_PM_MODE(1, MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE)] = {
		.supported = 1,
		.suspend_enabled = 0,
		.idle_enabled = 0,
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
	{0, 0, "C0", "WFI",
		MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT},

	{0, 1, "C1", "STANDALONE_POWER_COLLAPSE",
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE},

	{0, 2, "C2", "POWER_COLLAPSE",
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE},

	{1, 0, "C0", "WFI",
		MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT},

	{1, 1, "C1", "STANDALONE_POWER_COLLAPSE",
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE},
};
#ifdef CONFIG_USB_PEHCI_HCD

/* Important fields to initialize for Non-PCI based driver*/

/* The base physical memory address assigned for the ISP176x */
#define ISP176x_MEM_BASE 0x1D000000 //base address

/* The memory range assigned for the ISP176x 24k*/
#define ISP176x_MEM_RANGE 0x6000

/* The IRQ number assigned to the ISP176x */
#define ISP176x_IRQ_NUM 	MSM_GPIO_TO_INT(ISP1763_INT_GPIO)


static struct resource isp1763_resources[] = {
	[0] = {
		.flags	= IORESOURCE_MEM,
		.start	= ISP176x_MEM_BASE,
		.end	= (ISP176x_MEM_BASE + ISP176x_MEM_RANGE - 1),		/* 24KB */
	},
	[1] = {
		.flags	= IORESOURCE_IRQ,
		.start	= ISP176x_IRQ_NUM,
		.end	= ISP176x_IRQ_NUM,
	},
};
static void __init msm8x60_cfg_isp1763(void)
{
	isp1763_resources[1].start = gpio_to_irq(ISP1763_INT_GPIO);
	isp1763_resources[1].end = gpio_to_irq(ISP1763_INT_GPIO);
}

static int isp1763_setup_gpio(int enable)
{
	int status = 0;
	static int gpio_requested = 0;
	pr_info("%s\n", __func__);

	if (!gpio_requested)
	{
		status = gpio_request(ISP1763_INT_GPIO, "isp1763_usb");
		if (status) {
			pr_err("%s:Failed to request GPIO %d\n",
						__func__, ISP1763_INT_GPIO);
			return status;
		}
		status = gpio_request(ISP1763_RST_GPIO, "isp1763_usb");
		if (status) {
			pr_err("%s:Failed to request GPIO %d\n",
						__func__, ISP1763_RST_GPIO);
			gpio_free(ISP1763_INT_GPIO);
			return status;
		}
		gpio_requested = 1;
	}

	if (enable) {
		status = gpio_direction_input(ISP1763_INT_GPIO);
		if (status) {
			pr_err("%s:Failed to configure GPIO %d\n",
					__func__, ISP1763_INT_GPIO);
		}
		status = gpio_direction_output(ISP1763_RST_GPIO, 1);
		if (status) {
			pr_err("%s:Failed to configure GPIO %d\n",
					__func__, ISP1763_RST_GPIO);
		}
		pr_debug("\nISP GPIO configuration done\n");
		return status;
	}
	else
	{
		status = gpio_direction_output(ISP1763_RST_GPIO, 0);
		if (status) {
			pr_err("%s:Failed to configure GPIO %d\n",
					__func__, ISP1763_RST_GPIO);
		}
	}
	return status;
}

static struct isp1763_platform_data isp1763_pdata = {
	.reset_gpio	= ISP1763_RST_GPIO,
	.setup_gpio	= isp1763_setup_gpio
};

static struct platform_device isp1763_device = {
	.name          = "isp1763_usb",
	.num_resources = ARRAY_SIZE(isp1763_resources),
	.resource      = isp1763_resources,
	.dev           = {
		.platform_data = &isp1763_pdata
	}
};


/*
  This function only is charge of modem gpio initialize in board level, and should not
  take responsiblity for enabling/disabling external modem, which will be implemented
  in exmdm (external modem) driver.
*/
static int isp1763_modem_gpio_init(int on)
{
	int rc;
	static int gpio_requested=0;

	int gpio_pwr_3g_en = pin_table[TENDERLOIN_GPIO_3G_3V3_EN];

	if (!gpio_requested)
	{
		rc = gpio_request(gpio_pwr_3g_en, "VDD_3V3_EN");
		if (rc < 0) {
			pr_err("%s: VDD_3V3_EN gpio %d request failed\n", __func__, gpio_pwr_3g_en);
		}
		else
		{
			pr_debug("%s: VDD_3V3_EN gpio %d statu: %d\n", __func__, gpio_pwr_3g_en, gpio_get_value(gpio_pwr_3g_en));
		}

		rc = gpio_request(GPIO_3G_DISABLE_N, "3G_DISABLE_N");
		if (rc < 0) {
			pr_err("%s: GPIO_3G_DISABLE_N gpio %d request failed\n", __func__, GPIO_3G_DISABLE_N);
		}
		else
		{
			pr_debug( "%s: GPIO_3G_DISABLE_N gpio %d status: %d\n", __func__, GPIO_3G_DISABLE_N, gpio_get_value(GPIO_3G_DISABLE_N));
		}

		rc = gpio_request(GPIO_3G_WAKE_N, "3G_WAKE");
		if (rc < 0) {
			pr_err("%s: GPIO_3G_WAKE_N gpio %d request failed\n", __func__, GPIO_3G_WAKE_N);
		}

		rc = gpio_request(ISP1763_DACK_GPIO, "ISP1763A_DACK");
		if (rc < 0) {
			pr_err("%s: ISP1763A_DACK gpio %d request failed\n", __func__, ISP1763_DACK_GPIO);
		}

		rc = gpio_request(ISP1763_DREQ_GPIO, "ISP1763A_DREQ");
		if (rc < 0) {
			pr_err("%s: ISP1763A_DREQ gpio %d request failed\n", __func__, ISP1763_DREQ_GPIO);
		}
		gpio_requested = 1;
	}

	gpio_set_value(GPIO_3G_DISABLE_N, 0);
	gpio_set_value(gpio_pwr_3g_en, 0);

	if (on)
	{
		mdelay(300);
		gpio_set_value(gpio_pwr_3g_en, 1);
		gpio_set_value(GPIO_3G_DISABLE_N, 1);
	}

	return 0;
}

static struct platform_device *isp1763_pdev=NULL;
static int isp1763_modem_poweron_status(void)
{
	return isp1763_pdev? 1: 0;
}

static int isp1763_modem_poweron(int on)
{
	int ret=0;
	static struct mutex poweron_lock;
	static int inited = 0;

	pr_info("%s: isp1763_pdev:%p, on:%d\n", __func__, isp1763_pdev, on);

	if (!inited)
	{
		inited = 1;
		mutex_init(&poweron_lock);
	}

	mutex_lock(&poweron_lock);
	if (boardtype_is_3g())
	{
		if (on && !isp1763_pdev)
		{
			isp1763_modem_gpio_init(1);
			pr_info("%s: platform_device_register\n", __func__);
			isp1763_pdev = platform_device_alloc(isp1763_device.name, 0);
			if (!isp1763_pdev)
			{
				mutex_unlock(&poweron_lock);
				pr_err("%s: Platform device allocation failed\n", __func__);
				return -ENOMEM;
			}
			if ((ret=platform_device_add_resources(isp1763_pdev, isp1763_resources, ARRAY_SIZE(isp1763_resources)))!=0)
				goto out_free_resources;
			if ((ret=platform_device_add_data(isp1763_pdev, &isp1763_pdata,
						sizeof(isp1763_pdata)))!=0)
				goto out_free_resources;
			if ((ret = platform_device_add(isp1763_pdev))!=0)
				goto out_free_resources;
		}
		else if (!on && isp1763_pdev)
		{
			msleep(500);
			pr_info("%s: platform_device_unregister\n", __func__);
			platform_device_unregister(isp1763_pdev);
			isp1763_pdev = NULL;
			isp1763_modem_gpio_init(0);
			msleep(500);
		}
	}
	mutex_unlock(&poweron_lock);
	pr_info("%s: isp1763_pdev:%p, on:%d -- return\n", __func__, isp1763_pdev, on);
	return 0;

out_free_resources:
	pr_err("%s: out_free_resources\n", __func__);
	platform_device_put(isp1763_pdev);
	isp1763_pdev = NULL;
	isp1763_modem_gpio_init(0);
	pr_err("%s: return\n", __func__);
	mutex_unlock(&poweron_lock);
	return ret;
}

#endif

static int set_gpio_value(int gpionum, int value)
{
	gpio_set_value_cansleep(gpionum, value);
	return 0;
}

static int get_gpio_value(int gpionum)
{
	int value;
	value = gpio_get_value_cansleep(gpionum);
	return value;
}

static struct mdmgpio_platform_data mdmgpio_pdata = {
	.uim_cd_gpio = GPIO_3G_UIM_CD_N,
	.mdm_disable_gpio = GPIO_3G_DISABLE_N,
	.set_gpio_value = set_gpio_value,
	.get_gpio_value = get_gpio_value,
#ifdef CONFIG_USB_PEHCI_HCD
	.mdm_poweron    = isp1763_modem_poweron,
	.mdm_poweron_status= isp1763_modem_poweron_status,
#endif
};

struct platform_device mdmgpio_device = {
	.name = "mdmgpio",
	.id = -1,
	.dev = {
		.platform_data = &mdmgpio_pdata,
	},
};

#ifdef CONFIG_NDUID
static struct nduid_config nduid_cfg[] = {
	{
		.dev_name = "nduid",
	},
};

static struct platform_device nduid_device = {
	.name = "nduid",
	.id   = -1,
	.dev  = {
		.platform_data = &nduid_cfg,
	},
};
#endif

#if defined(CONFIG_USB_GADGET_MSM_72K) || defined(CONFIG_USB_EHCI_MSM)
static struct regulator *ldo6_3p3;
static struct regulator *ldo7_1p8;
static struct regulator *vdd_cx;
#define PMICID_INT		PM8058_GPIO_IRQ(PM8058_IRQ_BASE, 36)
notify_vbus_state notify_vbus_state_func_ptr;

#ifdef CONFIG_USB_EHCI_MSM
#define USB_PMIC_ID_DET_DELAY	msecs_to_jiffies(100)
struct delayed_work pmic_id_det;
static void pmic_id_detect(struct work_struct *w)
{
	int val = gpio_get_value_cansleep(PM8058_GPIO_PM_TO_SYS(36));
	pr_info("%s(): gpio_read_value = %d\n", __func__, val);

	if (notify_vbus_state_func_ptr)
		(*notify_vbus_state_func_ptr) (val);
}

static irqreturn_t pmic_id_on_irq(int irq, void *data)
{
	/*
	 * Spurious interrupts are observed on pmic gpio line
	 * even though there is no state change on USB ID. Schedule the
	 * work to to allow debounce on gpio
	 */
	schedule_delayed_work(&pmic_id_det, USB_PMIC_ID_DET_DELAY);

	return IRQ_HANDLED;
}

#ifdef CONFIG_USB_G_ANDROID

static struct usb_ether_platform_data rndis_pdata = {
	/* ethaddr is filled by board_serialno_setup */
        .vendorID       = 0x05C6,
        .vendorDescr    = "Qualcomm Incorporated",
};

static struct platform_device rndis_device = {
	.name	= "rndis",
	.id	= -1,
	.dev	= {
		.platform_data = &rndis_pdata,
	},
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "HP",
	.product	= "Android Tablet",
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x0BB4,
	.product_id	= 0x0C02,
	.version	= 0x0100,
	.product_name		= "Android Tablet",
	.manufacturer_name	= "HP",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
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


static int msm_hsusb_pmic_id_notif_init(void (*callback)(int online), int init)
{
	unsigned ret = -ENODEV;

	if (!callback)
		return -EINVAL;

	if (SOCINFO_VERSION_MAJOR(socinfo_get_version()) != 2) {
		pr_debug("%s: USB_ID pin is not routed to PMIC"
					"on V1 surf/ffa\n", __func__);
		return -ENOTSUPP;
	}

	if (machine_is_msm8x60_ffa() || machine_is_tenderloin()) {
		pr_debug("%s: USB_ID is not routed to PMIC"
			"on V2 ffa\n", __func__);
		return -ENOTSUPP;
	}

	if (init) {
		notify_vbus_state_func_ptr = callback;
		ret = pm8901_mpp_config_digital_out(1,
			PM8901_MPP_DIG_LEVEL_L5, 1);
		if (ret) {
			pr_err("%s: MPP2 configuration failed\n", __func__);
			return -ENODEV;
		}
		INIT_DELAYED_WORK(&pmic_id_det, pmic_id_detect);
		ret = request_threaded_irq(PMICID_INT, NULL, pmic_id_on_irq,
			(IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING),
						"msm_otg_id", NULL);
		if (ret) {
			pm8901_mpp_config_digital_out(1,
					PM8901_MPP_DIG_LEVEL_L5, 0);
			pr_err("%s:pmic_usb_id interrupt registration failed",
					__func__);
			return ret;
		}
	} else {
		free_irq(PMICID_INT, 0);
		cancel_delayed_work_sync(&pmic_id_det);
		notify_vbus_state_func_ptr = NULL;
		ret = pm8901_mpp_config_digital_out(1,
			PM8901_MPP_DIG_LEVEL_L5, 0);
		if (ret) {
			pr_err("%s:MPP2 configuration failed\n", __func__);
			return -ENODEV;
		}
	}
	return 0;
}
#endif

#define USB_PHY_SUSPEND_MIN_VDD_DIG_VOL		750000
#define USB_PHY_OPERATIONAL_MIN_VDD_DIG_VOL	1000000
#define USB_PHY_MAX_VDD_DIG_VOL			1320000
static int msm_hsusb_init_vddcx(int init)
{
	int ret = 0;

	if (init) {
		vdd_cx = regulator_get(NULL, "8058_s1");
		if (IS_ERR(vdd_cx)) {
			return PTR_ERR(vdd_cx);
		}

		ret = regulator_set_voltage(vdd_cx,
				USB_PHY_OPERATIONAL_MIN_VDD_DIG_VOL,
				USB_PHY_MAX_VDD_DIG_VOL);
		if (ret) {
			pr_err("%s: unable to set the voltage for regulator"
				"vdd_cx\n", __func__);
			regulator_put(vdd_cx);
			return ret;
		}

		ret = regulator_enable(vdd_cx);
		if (ret) {
			pr_err("%s: unable to enable regulator"
				"vdd_cx\n", __func__);
			regulator_put(vdd_cx);
		}
	} else {
		ret = regulator_disable(vdd_cx);
		if (ret) {
			pr_err("%s: Unable to disable the regulator:"
				"vdd_cx\n", __func__);
			return ret;
		}

		regulator_put(vdd_cx);
	}

	return ret;
}

static int msm_hsusb_config_vddcx(int high)
{
	int max_vol = USB_PHY_MAX_VDD_DIG_VOL;
	int min_vol;
	int ret;

	if (high)
		min_vol = USB_PHY_OPERATIONAL_MIN_VDD_DIG_VOL;
	else
		min_vol = USB_PHY_SUSPEND_MIN_VDD_DIG_VOL;

	ret = regulator_set_voltage(vdd_cx, min_vol, max_vol);
	if (ret) {
		pr_err("%s: unable to set the voltage for regulator"
			"vdd_cx\n", __func__);
		return ret;
	}

	pr_debug("%s: min_vol:%d max_vol:%d\n", __func__, min_vol, max_vol);

	return ret;
}

#define USB_PHY_3P3_VOL_MIN	3300000 /* uV */ //Palm changed to 3.3v
#define USB_PHY_3P3_VOL_MAX	3300000 /* uV */
#define USB_PHY_3P3_HPM_LOAD	50000	/* uA */
#define USB_PHY_3P3_LPM_LOAD	4000	/* uA */

#define USB_PHY_1P8_VOL_MIN	1800000 /* uV */
#define USB_PHY_1P8_VOL_MAX	1800000 /* uV */
#define USB_PHY_1P8_HPM_LOAD	50000	/* uA */
#define USB_PHY_1P8_LPM_LOAD	4000	/* uA */
static int msm_hsusb_ldo_init(int init)
{
	int rc = 0;

	if (init) {
		ldo6_3p3 = regulator_get(NULL, "8058_l6");
		if (IS_ERR(ldo6_3p3))
			return PTR_ERR(ldo6_3p3);

		ldo7_1p8 = regulator_get(NULL, "8058_l7");
		if (IS_ERR(ldo7_1p8)) {
			rc = PTR_ERR(ldo7_1p8);
			goto put_3p3;
		}

		rc = regulator_set_voltage(ldo6_3p3, USB_PHY_3P3_VOL_MIN,
				USB_PHY_3P3_VOL_MAX);
		if (rc) {
			pr_err("%s: Unable to set voltage level for"
				"ldo6_3p3 regulator\n", __func__);
			goto put_1p8;
		}
		rc = regulator_enable(ldo6_3p3);
		if (rc) {
			pr_err("%s: Unable to enable the regulator:"
				"ldo6_3p3\n", __func__);
			goto put_1p8;
		}
		rc = regulator_set_voltage(ldo7_1p8, USB_PHY_1P8_VOL_MIN,
				USB_PHY_1P8_VOL_MAX);
		if (rc) {
			pr_err("%s: Unable to set voltage level for"
				"ldo7_1p8 regulator\n", __func__);
			goto disable_3p3;
		}
		rc = regulator_enable(ldo7_1p8);
		if (rc) {
			pr_err("%s: Unable to enable the regulator:"
				"ldo7_1p8\n", __func__);
			goto disable_3p3;
		}

		return 0;
	}

	regulator_disable(ldo7_1p8);
disable_3p3:
	regulator_disable(ldo6_3p3);
put_1p8:
	regulator_put(ldo7_1p8);
put_3p3:
	regulator_put(ldo6_3p3);
	return rc;
}

static int msm_hsusb_ldo_enable(int on)
{
	int ret = 0;

	if (!ldo7_1p8 || IS_ERR(ldo7_1p8)) {
		pr_err("%s: ldo7_1p8 is not initialized\n", __func__);
		return -ENODEV;
	}

	if (!ldo6_3p3 || IS_ERR(ldo6_3p3)) {
		pr_err("%s: ldo6_3p3 is not initialized\n", __func__);
		return -ENODEV;
	}

	if (on) {
		ret = regulator_set_optimum_mode(ldo7_1p8,
				USB_PHY_1P8_HPM_LOAD);
		if (ret < 0) {
			pr_err("%s: Unable to set HPM of the regulator:"
				"ldo7_1p8\n", __func__);
			return ret;
		}
		ret = regulator_set_optimum_mode(ldo6_3p3,
				USB_PHY_3P3_HPM_LOAD);
		if (ret < 0) {
			pr_err("%s: Unable to set HPM of the regulator:"
				"ldo6_3p3\n", __func__);
			regulator_set_optimum_mode(ldo7_1p8,
				USB_PHY_1P8_LPM_LOAD);
			return ret;
		}
	} else {
		ret = regulator_set_optimum_mode(ldo7_1p8,
				USB_PHY_1P8_LPM_LOAD);
		if (ret < 0)
			pr_err("%s: Unable to set LPM of the regulator:"
				"ldo7_1p8\n", __func__);
		ret = regulator_set_optimum_mode(ldo6_3p3,
				USB_PHY_3P3_LPM_LOAD);
		if (ret < 0)
			pr_err("%s: Unable to set LPM of the regulator:"
				"ldo6_3p3\n", __func__);
	}

	pr_debug("reg (%s)\n", on ? "HPM" : "LPM");
	return ret < 0 ? ret : 0;
 }
#endif
#ifdef CONFIG_USB_EHCI_MSM
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
	static struct regulator *votg_5v_switch;
	static struct regulator *ext_5v_reg;
	static int vbus_is_on;

	/* If VBUS is already on (or off), do nothing. */
	if (on == vbus_is_on)
		return;

	if (!votg_5v_switch) {
		votg_5v_switch = regulator_get(NULL, "8901_usb_otg");
		if (IS_ERR(votg_5v_switch)) {
			pr_err("%s: unable to get votg_5v_switch\n", __func__);
			return;
		}
	}
	if (!ext_5v_reg) {
		ext_5v_reg = regulator_get(NULL, "8901_mpp0");
		if (IS_ERR(ext_5v_reg)) {
			pr_err("%s: unable to get ext_5v_reg\n", __func__);
			return;
		}
	}
	if (on) {
		if (regulator_enable(ext_5v_reg)) {
			pr_err("%s: Unable to enable the regulator:"
					" ext_5v_reg\n", __func__);
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
		if (regulator_disable(ext_5v_reg))
			pr_err("%s: Unable to enable the regulator:"
				" ext_5v_reg\n", __func__);
	}

	vbus_is_on = on;
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
	.phy_info	= (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
	.power_budget	= 390,
};
#endif

#ifdef CONFIG_BATTERY_MSM8X60
static int msm_hsusb_pmic_vbus_notif_init(void (*callback)(int online),
								int init)
{
	int ret = -ENOTSUPP;

	/* ID and VBUS lines are connected to pmic on 8660.V2.SURF,
	 * hence, irrespective of either peripheral only mode or
	 * OTG (host and peripheral) modes, can depend on pmic for
	 * vbus notifications
	 */
	if ((SOCINFO_VERSION_MAJOR(socinfo_get_version()) == 2)
			&& (machine_is_msm8x60_surf())) {
		if (init)
			ret = msm_charger_register_vbus_sn(callback);
		else {
			msm_charger_unregister_vbus_sn(callback);
			ret = 0;
		}
	} else {
#if !defined(CONFIG_USB_EHCI_MSM)
	if (init)
		ret = msm_charger_register_vbus_sn(callback);
	else {
		msm_charger_unregister_vbus_sn(callback);
		ret = 0;
	}
#endif
	}
	return ret;
}
#endif

#if defined(CONFIG_USB_GADGET_MSM_72K) || defined(CONFIG_USB_EHCI_MSM)
static struct msm_otg_platform_data msm_otg_pdata = {
	/* if usb link is in sps there is no need for
	 * usb pclk as dayatona fabric clock will be
	 * used instead
	 */
	.pclk_src_name		 = "dfab_usb_hs_clk",
	.pemp_level		 = PRE_EMPHASIS_WITH_20_PERCENT,
	.cdr_autoreset		 = CDR_AUTO_RESET_DISABLE,
	.se1_gating		 = SE1_GATING_DISABLE,
	.hsdrvslope		 = 0x05,
#ifdef CONFIG_USB_EHCI_MSM
	.pmic_id_notif_init = msm_hsusb_pmic_id_notif_init,
#endif
#ifdef CONFIG_USB_EHCI_MSM
	.vbus_power = msm_hsusb_vbus_power,
#endif
#ifdef CONFIG_BATTERY_MSM8X60
	.pmic_vbus_notif_init	= msm_hsusb_pmic_vbus_notif_init,
#endif
	.ldo_init		 = msm_hsusb_ldo_init,
	.ldo_enable		 = msm_hsusb_ldo_enable,
	.config_vddcx            = msm_hsusb_config_vddcx,
	.init_vddcx              = msm_hsusb_init_vddcx,
#if defined(CONFIG_BATTERY_MSM8X60)
	.chg_vbus_draw = msm_charger_vbus_draw,
#elif defined (CONFIG_MAX8903B_CHARGER)
	.chg_vbus_draw = max8903b_set_vbus_draw,
#endif
#ifdef CONFIG_A6
	.chg_connected = a6_charger_event,
#endif
};
#endif

#ifdef CONFIG_USB_GADGET_MSM_72K
static struct msm_hsusb_gadget_platform_data msm_gadget_pdata = {
	.is_phy_status_timer_on = 1,
};
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

#ifdef CONFIG_MAX8903B_CHARGER
// available current settings for MAX8903B per HW revision
static int sel_tbl_protos[] = {
	CURRENT_900MA,
	CURRENT_1000MA,
	CURRENT_1500MA,
	CURRENT_2000MA,
};

static int sel_tbl_evt1[] = {
	CURRENT_750MA,
	CURRENT_900MA,
	CURRENT_1500MA,
	CURRENT_1400MA,
};

static int sel_tbl_evt2[] = {
	CURRENT_750MA,
	CURRENT_900MA,
	CURRENT_2000MA,
	CURRENT_1400MA,
};

static int max8903b_control_index(enum max8903b_current value)
{
	int i, index;
	int *table;
	int v = value;

	switch (board_type)
	{
		case TOPAZ_PROTO:
		case TOPAZ_PROTO2:
		case TOPAZ_3G_PROTO:
		case TOPAZ_3G_PROTO2:
			table = sel_tbl_protos;
			break;

		case TOPAZ_EVT1:
			table = sel_tbl_evt1;
			if (v == CURRENT_2000MA)
				v = CURRENT_1500MA;
			break;

		default:
			table = sel_tbl_evt2;
			break;
	}

	// find index from table
	for (i = 0 ; i < 4 ; i++)
	{
		if (v == table[i]) {
			index = i;
			break;
		}

		if (i == 3) {
			index = -EINVAL;  // invalid input
		}
	}

	return index;
}

int max8903b_set_DC_CHG_Mode_current(enum max8903b_current value)
{
	int index;

	index = max8903b_control_index(value);
	switch(index)	{
		case 0: // select output port 0
			gpio_set_value(MAX8903B_GPIO_CHG_D_ISET_1,0);
			gpio_set_value(MAX8903B_GPIO_CHG_D_ISET_2,0);
			pr_debug("max8903 MUX : LOW / LOW\n");
			break;
		case 1: // select output port 1
			gpio_set_value(MAX8903B_GPIO_CHG_D_ISET_1,0);
			gpio_set_value(MAX8903B_GPIO_CHG_D_ISET_2,1);
			pr_debug("max8903 MUX : LOW / HIGH\n");
			break;
		case 2: // select output port 2
			gpio_set_value(MAX8903B_GPIO_CHG_D_ISET_1,1);
			gpio_set_value(MAX8903B_GPIO_CHG_D_ISET_2,0);
			pr_debug("max8903 MUX : HIGH / LOW\n");
			break;
		case 3: // select output port 3
			gpio_set_value(MAX8903B_GPIO_CHG_D_ISET_1,1);
			gpio_set_value(MAX8903B_GPIO_CHG_D_ISET_2,1);
			pr_debug("max8903 MUX : HIGH / HIGH\n");
			break;
		default:
			printk(KERN_INFO "%s: Invalid current setting, not supported in HW rev.\n", __func__);
			return -1;
			break;
	}
	return 0;
};

int max8903b_request_release_gpios(int request);
void max8903b_suspend_gpio_config(void);

static struct	max8903b_platform_data	max8903b_charger_pdata = {
	.DCM_in 	= MAX8903B_GPIO_DC_CHG_MODE,
	.DCM_in_polarity	= 1,
	.IUSB_in 	= MAX8903B_GPIO_USB_CHG_MODE,
	.IUSB_in_polarity	= 0,
	.USUS_in 	= MAX8903B_GPIO_USB_CHG_SUS,
	.USUS_in_polarity	= 0,
	.CEN_N_in 	= MAX8903B_GPIO_CHG_EN,
	.CEN_N_in_polarity	= 1,
	.DOK_N_out 	= MAX8903B_GPIO_DC_OK,
	.CHG_N_out 	= MAX8903B_GPIO_STATUS_N,
	.FLT_N_out 	= MAX8903B_GPIO_FAULT_N,
	.set_DC_CHG_Mode_current = max8903b_set_DC_CHG_Mode_current,
	.request_release_gpios = max8903b_request_release_gpios,
	.suspend_gpio_config  = max8903b_suspend_gpio_config,
};

static struct 	platform_device 	max8903b_charger_device = {
	.name               = "max8903b_chg",
	.id                 = 0,
	.dev.platform_data  = &max8903b_charger_pdata,
};

int max8903b_request_release_gpios(int request)
{
	static int gpio_array_init = 0;
	static int is_gpio_active = 0;
	static int max8903b_gpios[9];
	int rc = 0;

	if (gpio_array_init == 0) {
		max8903b_gpios[0] = 	max8903b_charger_pdata.DCM_in;
		max8903b_gpios[1] = 	max8903b_charger_pdata.IUSB_in;
		max8903b_gpios[2] = 	max8903b_charger_pdata.USUS_in;
		max8903b_gpios[3] =	max8903b_charger_pdata.CEN_N_in;
		max8903b_gpios[4] =	max8903b_charger_pdata.DOK_N_out;
		max8903b_gpios[5] =	max8903b_charger_pdata.CHG_N_out;
		max8903b_gpios[6] =	max8903b_charger_pdata.FLT_N_out;
		max8903b_gpios[7] =	MAX8903B_GPIO_CHG_D_ISET_1;
		max8903b_gpios[8] =	MAX8903B_GPIO_CHG_D_ISET_2;

		gpio_array_init = 1;
	}

	if (request) {   // reqest gpios
		if (is_gpio_active == 0) {
			rc = configure_gpios(1, max8903b_gpios, ARRAY_SIZE(max8903b_gpios));
			if (rc == 0)
				is_gpio_active = request;  // requested
		} else {
			printk("%s: GPIOs were already active(requested) !!!\n", __func__);
		}
	} else {    // free gpios
		if (is_gpio_active != 0) {
			rc = configure_gpios(0, max8903b_gpios, ARRAY_SIZE(max8903b_gpios));
			if (rc == 0)
				is_gpio_active = request;  // freed
		} else {
			printk("%s: GPIOs were already released(freed) !!!\n", __func__);
		}
	}

	return rc;
}

/* Set necessary GPIOs for suspend mode state. */
void max8903b_suspend_gpio_config(void)
{
	gpio_set_value(MAX8903B_GPIO_USB_CHG_MODE, 1);
	gpio_set_value(MAX8903B_GPIO_CHG_EN, 0);

	gpio_set_value(MAX8903B_GPIO_CHG_D_ISET_1, 0);
	gpio_set_value(MAX8903B_GPIO_CHG_D_ISET_2, 0);
}

void max8903b_set_connected_ps (unsigned connected)
{
	max8903b_ps_connected = connected;

	if (!connected) {
		max8903b_disable_charge();
		max8903b_vbus_draw_ma_max = 0;
	} else if (connected & MAX8903B_CONNECTED_PS_DOCK) {
		max8903b_set_charge_ma(MAX8903B_DOCK_DRAW_MA);
	}
}
EXPORT_SYMBOL (max8903b_set_connected_ps);

/* Callback for msm72k_otg to notify charge supplied by phy.
 * Current draw defaults to 500mA. WebOS seems to use the sysfs to
 * set current_limit on usb plugin.
 */
void max8903b_set_vbus_draw (unsigned ma)
{
	max8903b_vbus_draw_ma = ma;

	if (ma > max8903b_vbus_draw_ma_max) {
		max8903b_vbus_draw_ma_max = ma;
	}

	if (!(max8903b_ps_connected & MAX8903B_CONNECTED_PS_DOCK)) {
		if (max8903b_ps_connected & MAX8903B_CONNECTED_PS_AC) {
			max8903b_set_charge_ma(max8903b_vbus_draw_ma_max);
		} else {
			max8903b_set_charge_ma(ma);
		}
	}
}
#endif  /* CONFIG_MAX8903B_CHARGER */

#ifdef CONFIG_A6
/* The following #defines facilitate selective inclusion of a specific a6 wakeup strategy:
   [constraint]: A6_PMIC_EXTERNAL_WAKE and A6_INTERNAL_WAKE are mutually exclusive
   A6_PMIC_EXTERNAL_WAKE: configures a6 driver to use PMIC-based LPG PWM for wakeup
   A6_INTERNAL_WAKE: configures for a6-based internal wake
   if neither defined: configures A6 driver to keep a6 constantly awake using SBW_WAKEUP pin
*/
#define A6_INTERNAL_WAKE
/* a6 wakeup selection */
/*
A6_TCK_GPIO 	157
A6_TDIO_GPIO 	158
A6_WAKEUP_GPIO 	155
A6_MSM_INT_GPIO 	156
*/
static struct a6_sbw_interface 	sbw_ops_impl_0;
static struct a6_sbw_interface 	sbw_ops_impl_1;
#ifdef A6_INTERNAL_WAKE
static struct a6_wake_ops 	a6_wake_ops_impl_0;
static struct a6_wake_ops 	a6_wake_ops_impl_1;
#endif

static int a6_0_sbw_gpio_config[] = {
	/* first A6 config */
	TENDERLOIN_A6_0_TCK,
	TENDERLOIN_A6_0_WAKEUP,
	TENDERLOIN_A6_0_TDIO,
};

static int a6_0_sbw_gpio_config_3g[] = {
	/* first A6 config */
	TENDERLOIN_A6_0_TCK_3G,
	TENDERLOIN_A6_0_WAKEUP_3G,
	TENDERLOIN_A6_0_TDIO_3G,
};

static int a6_1_sbw_gpio_config[] = {
	/* second A6 config */
	TENDERLOIN_A6_1_TCK,
	TENDERLOIN_A6_1_WAKEUP,
	TENDERLOIN_A6_1_TDIO,
};

static int a6_1_sbw_gpio_config_3g[] = {
	/* second A6 config */
	TENDERLOIN_A6_1_TCK_3G,
	TENDERLOIN_A6_1_WAKEUP_3G,
	TENDERLOIN_A6_1_TDIO_3G,
};

static int a6_sbw_init_imp(struct a6_platform_data* plat_data)
{
	int rc = 0;

	rc = configure_gpiomux_gpios(1, (int *)plat_data->sbw_init_gpio_config,
			plat_data->sbw_init_gpio_config_size);
	if (rc < 0) {
		printk(KERN_ERR "%s: failed to configure A6 SBW gpios.\n", __func__);
	}

	return rc;
}


static int a6_sbw_deinit_imp(struct a6_platform_data* plat_data)
{
	int rc = 0;

	rc = configure_gpiomux_gpios(0, (int *)plat_data->sbw_deinit_gpio_config,
				plat_data->sbw_deinit_gpio_config_size);
	if (rc < 0) {
		printk(KERN_ERR "%s: failed to de-configure A6 SBW gpios.\n", __func__);
	}

	return rc;
}


static struct a6_platform_data tenderloin_a6_0_platform_data = {
	.dev_name			= A6_DEVICE_0,
	.pwr_gpio			= TENDERLOIN_A6_0_MSM_IRQ,
	.sbw_tck_gpio			= TENDERLOIN_A6_0_TCK,
	.sbw_tdio_gpio			= TENDERLOIN_A6_0_TDIO,
	.sbw_wkup_gpio			= TENDERLOIN_A6_0_WAKEUP,
	.sbw_ops			= &sbw_ops_impl_0,

	.sbw_init_gpio_config		= a6_0_sbw_gpio_config,
	.sbw_init_gpio_config_size 	= ARRAY_SIZE(a6_0_sbw_gpio_config),
	.sbw_deinit_gpio_config		= a6_0_sbw_gpio_config,
	.sbw_deinit_gpio_config_size 	= ARRAY_SIZE(a6_0_sbw_gpio_config),

	.sbw_init		= a6_sbw_init_imp,
	.sbw_deinit		= a6_sbw_deinit_imp,

	.power_supply_connected = 1,
};

static struct a6_platform_data tenderloin_a6_1_platform_data = {
	.dev_name			= A6_DEVICE_1,
	.pwr_gpio       		= TENDERLOIN_A6_1_MSM_IRQ,
	.sbw_tck_gpio			= TENDERLOIN_A6_1_TCK,
	.sbw_tdio_gpio			= TENDERLOIN_A6_1_TDIO,
	.sbw_wkup_gpio			= TENDERLOIN_A6_1_WAKEUP,
	.sbw_ops			= &sbw_ops_impl_1,

	.sbw_init_gpio_config		= a6_1_sbw_gpio_config,
	.sbw_init_gpio_config_size 	= ARRAY_SIZE(a6_1_sbw_gpio_config),
	.sbw_deinit_gpio_config		= a6_1_sbw_gpio_config,
	.sbw_deinit_gpio_config_size 	= ARRAY_SIZE(a6_1_sbw_gpio_config),

	.sbw_init		= a6_sbw_init_imp,
	.sbw_deinit		= a6_sbw_deinit_imp,

	.power_supply_connected = 0,
};


static int a6_0_config_data[] = {
	TENDERLOIN_A6_0_MSM_IRQ,
};

static int a6_0_config_data_3g[] = {
	TENDERLOIN_A6_0_MSM_IRQ_3G,
};

static int a6_1_config_data[] = {
	TENDERLOIN_A6_1_MSM_IRQ,
};

static int a6_1_config_data_3g[] = {
	TENDERLOIN_A6_1_MSM_IRQ_3G,
};

static struct i2c_board_info a6_0_i2c_board_info = {
	I2C_BOARD_INFO( A6_DEVICE_0, (0x62>>1)),
	.platform_data = NULL,
};

static struct i2c_board_info a6_1_i2c_board_info = {
	I2C_BOARD_INFO( A6_DEVICE_1, (0x64>>1)),
	.platform_data = NULL,
};

static struct isl29023_platform_data isl29023_pdata = {
    .rext = 10,
    .polled = 1,
    .poll_interval = 500,
};

static struct lsm303dlh_acc_platform_data lsm303dlh_acc_pdata = {
	.poll_interval = 200,
	.min_interval = 10,
	.g_range = LSM303DLH_ACC_G_2G,
	.axis_map_x = 1,
	.axis_map_y = 0,
	.axis_map_z = 2,
	.negate_x = 1,
	.negate_y = 0,
	.negate_z = 0,
	.gpio_int1 = -1,
	.gpio_int2 = -1,
};

static struct lsm303dlh_mag_platform_data lsm303dlh_mag_pdata = {
	.poll_interval = 200,
	.min_interval = 10,
	.h_range = LSM303DLH_MAG_H_4_0G,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
};

static struct mpu3050_platform_data mpu_pdata = {
	.int_config  = 0x10,
	.orientation = {  -1,  0,  0,
			           0,  1,  0,
			           0,  0, -1 },
    /* accel */
	.accel = {
		 .get_slave_descr = get_accel_slave_descr,
		 .adapt_num   = 0,
		 .bus         = EXT_SLAVE_BUS_SECONDARY,
		 .address     = 0x18,
		 .orientation = {  -1,  0,  0,
		         		    0,  1,  0,
				            0,  0, -1 },
	 },
};

static struct i2c_board_info __initdata lsm303dlh_acc_i2c_board_info[] = {
    {
        I2C_BOARD_INFO ( "lsm303dlh_acc_sysfs", 0x18),
        .platform_data = &lsm303dlh_acc_pdata,
    },
};

static struct i2c_board_info __initdata lsm303dlh_mag_i2c_board_info[] = {
    {
        I2C_BOARD_INFO ( "lsm303dlh_mag_sysfs", 0x1e),
        .platform_data = &lsm303dlh_mag_pdata,
    },
};

static struct i2c_board_info __initdata mpu3050_i2c_board_info[] = {
    {
        I2C_BOARD_INFO ( MPU_NAME, 0x68),
        .irq = MSM_GPIO_TO_INT(TENDERLOIN_GYRO_INT),
        .platform_data = &mpu_pdata,
    },
};

static struct i2c_board_info __initdata isl29023_i2c_board_info[] = {
    {
        I2C_BOARD_INFO ( "isl29023", 0x44),
        .irq = MSM_GPIO_TO_INT(TENDERLOIN_LS_INT),
        .platform_data = &isl29023_pdata,
    },
};


#ifdef A6_INTERNAL_WAKE
struct a6_internal_wake_interface_data {
	int	wake_enable: 1;
	int	wake_period: 9;
	int	wake_gpio;
};

static struct a6_internal_wake_interface_data a6_0_wi_data =
{
	.wake_enable = 1,
	.wake_period = 16,
};

static struct a6_internal_wake_interface_data a6_1_wi_data =
{
	.wake_enable = 1,
	.wake_period = 16,
};


static int32_t a6_force_wake(void* data)
{
	struct a6_internal_wake_interface_data *pdata = (struct a6_internal_wake_interface_data *)data;
	unsigned long flags = 0;

	a6_disable_interrupts(flags);
	gpio_set_value(pdata->wake_gpio, 1);
	udelay(1);
	gpio_set_value(pdata->wake_gpio, 0);
	udelay(1);
	gpio_set_value(pdata->wake_gpio, 1);
	a6_enable_interrupts(flags);

	msleep(30);

	return 0;
}

static int a6_force_sleep(void* data)
{
	struct a6_internal_wake_interface_data *pdata = (struct a6_internal_wake_interface_data *)data;

	gpio_set_value(pdata->wake_gpio, 0);

	return 0;
}

static int a6_internal_wake_enable_state(void* data)
{
	return (((struct a6_internal_wake_interface_data *)data)->wake_enable);
}

static int a6_internal_wake_period(void* data)
{
	return (((struct a6_internal_wake_interface_data *)data)->wake_period);
}
#endif // #ifdef A6_INTERNAL_WAKE


static void __init tenderloin_init_a6(void)
{
	// Fixup A6 platform data
	tenderloin_a6_0_platform_data.pwr_gpio_wakeup_cap =
		msm_mpm_is_app_irq_wakeup_capable(gpio_to_irq(tenderloin_a6_0_platform_data.pwr_gpio));

	tenderloin_a6_1_platform_data.pwr_gpio_wakeup_cap =
		msm_mpm_is_app_irq_wakeup_capable(gpio_to_irq(tenderloin_a6_1_platform_data.pwr_gpio));

	printk(KERN_ERR "Registering a6_0 device.\n");
	a6_0_i2c_board_info.platform_data = &tenderloin_a6_0_platform_data;
	i2c_register_board_info(MSM_GSBI8_QUP_I2C_BUS_ID, &a6_0_i2c_board_info, 1);
	a6_sbw_init_imp(&tenderloin_a6_0_platform_data);

	printk(KERN_ERR "Registering a6_1 device.\n");
	a6_1_i2c_board_info.platform_data = &tenderloin_a6_1_platform_data;
	i2c_register_board_info(MSM_GSBI8_QUP_I2C_BUS_ID, &a6_1_i2c_board_info, 1);
	a6_sbw_init_imp(&tenderloin_a6_1_platform_data);

	if(boardtype_is_3g()) {
		configure_gpiomux_gpios(1, a6_0_config_data_3g, ARRAY_SIZE(a6_0_config_data_3g));
		configure_gpiomux_gpios(1, a6_1_config_data_3g, ARRAY_SIZE(a6_1_config_data_3g));
	}
	else {
		configure_gpiomux_gpios(1, a6_0_config_data, ARRAY_SIZE(a6_0_config_data));
		configure_gpiomux_gpios(1, a6_1_config_data, ARRAY_SIZE(a6_1_config_data));
	}

#ifdef A6_INTERNAL_WAKE
	((struct a6_platform_data*)a6_0_i2c_board_info.platform_data)->wake_ops = &a6_wake_ops_impl_0;
	((struct a6_platform_data*)a6_1_i2c_board_info.platform_data)->wake_ops = &a6_wake_ops_impl_1;

	// for non-PMIC external wakes, use sbw_wkup_gpio for force wakes...
	a6_0_wi_data.wake_gpio =
		((struct a6_platform_data *)a6_0_i2c_board_info.platform_data)->sbw_wkup_gpio;
	a6_wake_ops_impl_0.data = &a6_0_wi_data;
	a6_wake_ops_impl_0.enable_periodic_wake = NULL;
	a6_wake_ops_impl_0.disable_periodic_wake = NULL;
	a6_wake_ops_impl_0.internal_wake_enable_state = &a6_internal_wake_enable_state;
	a6_wake_ops_impl_0.internal_wake_period = &a6_internal_wake_period;
	a6_wake_ops_impl_0.force_wake = &a6_force_wake;
	a6_wake_ops_impl_0.force_sleep = &a6_force_sleep;

	a6_1_wi_data.wake_gpio =
		((struct a6_platform_data *)a6_1_i2c_board_info.platform_data)->sbw_wkup_gpio;
	a6_wake_ops_impl_1.data = &a6_1_wi_data;
	a6_wake_ops_impl_1.enable_periodic_wake = NULL;
	a6_wake_ops_impl_1.disable_periodic_wake = NULL;
	a6_wake_ops_impl_1.internal_wake_enable_state = &a6_internal_wake_enable_state;
	a6_wake_ops_impl_1.internal_wake_period = &a6_internal_wake_period;
	a6_wake_ops_impl_1.force_wake = &a6_force_wake;
	a6_wake_ops_impl_1.force_sleep = &a6_force_sleep;
#else
	((struct a6_platform_data*)a6_0_i2c_board_info.platform_data)->wake_ops = NULL;
	((struct a6_platform_data*)a6_1_i2c_board_info.platform_data)->wake_ops = NULL;
#endif

	sbw_ops_impl_0.a6_per_device_interface.SetSBWTCK = &a6_0_set_sbwtck;
	sbw_ops_impl_0.a6_per_device_interface.ClrSBWTCK = &a6_0_clr_sbwtck;
	sbw_ops_impl_0.a6_per_device_interface.SetSBWTDIO = &a6_0_set_sbwtdio;
	sbw_ops_impl_0.a6_per_device_interface.ClrSBWTDIO = &a6_0_clr_sbwtdio;
	sbw_ops_impl_0.a6_per_device_interface.SetInSBWTDIO = &a6_0_set_in_sbwtdio;
	sbw_ops_impl_0.a6_per_device_interface.SetOutSBWTDIO = &a6_0_set_out_sbwtdio;
	sbw_ops_impl_0.a6_per_device_interface.GetSBWTDIO = &a6_0_get_sbwtdio;
	sbw_ops_impl_0.a6_per_device_interface.SetSBWAKEUP = &a6_0_set_sbwakeup;
	sbw_ops_impl_0.a6_per_device_interface.ClrSBWAKEUP = &a6_0_clr_sbwakeup;
	sbw_ops_impl_0.a6_per_target_interface.delay = a6_delay_impl;

	sbw_ops_impl_1.a6_per_device_interface.SetSBWTCK = &a6_1_set_sbwtck;
	sbw_ops_impl_1.a6_per_device_interface.ClrSBWTCK = &a6_1_clr_sbwtck;
	sbw_ops_impl_1.a6_per_device_interface.SetSBWTDIO = &a6_1_set_sbwtdio;
	sbw_ops_impl_1.a6_per_device_interface.ClrSBWTDIO = &a6_1_clr_sbwtdio;
	sbw_ops_impl_1.a6_per_device_interface.SetInSBWTDIO = &a6_1_set_in_sbwtdio;
	sbw_ops_impl_1.a6_per_device_interface.SetOutSBWTDIO = &a6_1_set_out_sbwtdio;
	sbw_ops_impl_1.a6_per_device_interface.GetSBWTDIO = &a6_1_get_sbwtdio;
	sbw_ops_impl_1.a6_per_device_interface.SetSBWAKEUP = &a6_1_set_sbwakeup;
	sbw_ops_impl_1.a6_per_device_interface.ClrSBWAKEUP = &a6_1_clr_sbwakeup;
	sbw_ops_impl_1.a6_per_target_interface.delay = a6_delay_impl;
}
#endif // CONFIG_A6


#ifdef CONFIG_MSM_CAMERA
#ifdef CONFIG_WEBCAM_MT9M113

static int camera_mt9m113_gpios[] = {
	TENDERLOIN_CAM_I2C_DATA,
	TENDERLOIN_CAM_I2C_CLK,
	TENDERLOIN_CAMIF_MCLK,
	TENDERLOIN_WEBCAM_RST,
	TENDERLOIN_WEBCAM_PWDN,
};

struct regulator *votg_lvs0 = NULL;
struct regulator *votg_vreg_l11 = NULL;
bool gpios_web_cam_mt9m113_on = false;

static int config_camera_on_gpios_web_cam_mt9m113(void)
{
	int rc = 0;

	CDBG("+++ %s\n", __func__);
	if ( !gpios_web_cam_mt9m113_on ) {

		configure_gpiomux_gpios(1, camera_mt9m113_gpios,
			ARRAY_SIZE(camera_mt9m113_gpios));

		votg_lvs0 = regulator_get(NULL, "8058_lvs0");
		if (IS_ERR_OR_NULL(votg_lvs0)) {
			CDBG("%s: unable to get votg_lvs0\n", __func__);
			goto err;
		}

		if (regulator_enable(votg_lvs0)) {
			CDBG("%s:Unable to enable the regulator votg_lvs0\n", __func__);
			goto err1;
		}
		else {
			CDBG("%s:enable the regulator votg_lvs0 succeed\n", __func__);
		}

		votg_vreg_l11 = regulator_get(NULL, "8058_l11");
		if (IS_ERR_OR_NULL(votg_vreg_l11)) {
			CDBG("%s: unable to get votg_vreg_l11\n", __func__);
			goto err1;
		}

		if(regulator_set_voltage(votg_vreg_l11, 2850000, 2850000)) {
			CDBG("%s: Unable to set regulator voltage:"
			" votg_l11\n", __func__);
			goto err2;
		}

		if (regulator_enable(votg_vreg_l11)) {
			CDBG("%s:Unable to enable the regulator votg_vreg_l11\n", __func__);
			goto err2;
		}
		else {
			CDBG("%s:enable the regulator votg_vreg_l11 succeed\n", __func__);
		}

		gpios_web_cam_mt9m113_on = true;
	}

	CDBG("--- %s\n", __func__);
	return 0;

err2:
	regulator_disable(votg_vreg_l11);
	regulator_put(votg_vreg_l11);
	votg_vreg_l11 = NULL;

err1:
	regulator_disable(votg_lvs0);
	regulator_put(votg_lvs0);
	votg_lvs0 = NULL;

err:
	configure_gpiomux_gpios(0, camera_mt9m113_gpios,
			ARRAY_SIZE(camera_mt9m113_gpios));

	//If error code is not specified return -1
	if (!rc) {
		rc = -1;
	}

	CDBG("--- %s\n", __func__);
	return rc;
}

static void config_camera_off_gpios_web_cam_mt9m113(void)
{
	CDBG("+++ %s\n", __func__);

	if (gpios_web_cam_mt9m113_on) {

		configure_gpiomux_gpios(0, camera_mt9m113_gpios,
				ARRAY_SIZE(camera_mt9m113_gpios));

		if (IS_ERR_OR_NULL(votg_lvs0)) {
			CDBG("%s: unable to get votg_lvs0\n", __func__);
		} else {

			if (regulator_disable(votg_lvs0)) {
				CDBG("%s:Unable to disable the regulator: votg_lvs0\n", __func__);
			}
			else {
				CDBG("%s:disable the regulator: votg_lvs0 succeed\n", __func__);
			}

			regulator_put(votg_lvs0);
			votg_lvs0 = NULL;
		}

		if (IS_ERR_OR_NULL(votg_vreg_l11)) {
			CDBG("%s: unable to get votg_vreg_l11\n", __func__);
		} else {

			if (regulator_disable(votg_vreg_l11)) {
				CDBG("%s:Unable to disable the regulator: votg_vreg_l11\n", __func__);
			}
			else {
				CDBG("%s:disable the regulator: votg_vreg_l11 succeed\n", __func__);
			}

			regulator_put(votg_vreg_l11);
			votg_vreg_l11 = NULL;
		}

		gpios_web_cam_mt9m113_on = false;
	}

	CDBG("--- %s\n", __func__);
}

struct msm_camera_device_platform_data msm_camera_device_data_web_cam_mt9m113 = {
	.camera_gpio_on  = config_camera_on_gpios_web_cam_mt9m113,
	.camera_gpio_off = config_camera_off_gpios_web_cam_mt9m113,
	.ioext.csiphy = 0x04900000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = CSI_1_IRQ,
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

static struct msm_camera_sensor_flash_data msm_flash_none = {
       .flash_type = MSM_CAMERA_FLASH_NONE,
       .flash_src  = NULL
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9m113_data = {
	.sensor_name	= "mt9m113",
	.sensor_reset	= 106,
	.sensor_pwd		= 107,
	.vcm_pwd		= 1,
	.vcm_enable		= 0,
	.pdata			= &msm_camera_device_data_web_cam_mt9m113,
	.resource		= msm_camera_resources,
	.num_resources	= ARRAY_SIZE(msm_camera_resources),
	.flash_data		= &msm_flash_none,
	.csi_if			= 1
};

struct platform_device msm_camera_sensor_webcam_mt9m113 = {
	.name	= "msm_camera_mt9m113",
	.dev	= {
		.platform_data = &msm_camera_sensor_mt9m113_data,
	},
};

static struct i2c_board_info msm_camera_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("mt9m113", 0x78),
	},
};
#endif
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

static void board_i2c_recover(void)
{
	int i = 0;
	int rc = 0;
	struct gpiomux_setting sda_old;
	struct gpiomux_setting scl_old;
	struct gpiomux_setting new = {
		.func = GPIOMUX_FUNC_GPIO,
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_IN,
	};

	printk(KERN_NOTICE "%s\n", __func__);

	rc = gpio_request(GPIO_SENSORS_SDA, "i2c_data");
	if (rc < 0) {
		printk(KERN_ERR "%s: error %d requesting gpio %d (i2c_data)\n", __func__, rc, GPIO_SENSORS_SDA);
		goto gpio_request_data_failed;
	}

	rc = msm_gpiomux_write(GPIO_SENSORS_SDA, GPIOMUX_ACTIVE, &new, &sda_old);
	if (rc < 0)  {
		printk(KERN_ERR "%s: error %d muxing gpio %d (i2c_data)\n", __func__, rc, GPIO_SENSORS_SDA);
		goto msm_gpiomux_write_data_failed;
	}

	rc = msm_gpiomux_get(GPIO_SENSORS_SDA);
	if (rc < 0)  {
		printk(KERN_ERR "%s: error %d 'getting' gpio %d (i2c_data)\n", __func__, rc, GPIO_SENSORS_SDA);
		goto msm_gpiomux_get_data_failed;
	}

	rc = gpio_request(GPIO_SENSORS_SCL, "i2c_clock");
	if (rc < 0) {
		printk(KERN_ERR "%s: error %d requesting gpio %d (i2c_clock)\n", __func__, rc, GPIO_SENSORS_SCL);
		goto gpio_request_clock_failed;
	}

	// Configure gpio as an output
	new.dir = GPIOMUX_OUT_HIGH;

	rc = msm_gpiomux_write(GPIO_SENSORS_SCL, GPIOMUX_ACTIVE, &new, &scl_old);
	if (rc < 0)  {
		printk(KERN_ERR "%s: error %d muxing gpio %d (i2c_clock)\n", __func__, rc, GPIO_SENSORS_SCL);
		goto msm_gpiomux_write_clock_failed;
	}

	rc = msm_gpiomux_get(GPIO_SENSORS_SCL);
	if (rc < 0)  {
		printk(KERN_ERR "%s: error %d 'getting' gpio %d (i2c_clock)\n", __func__, rc, GPIO_SENSORS_SCL);
		goto msm_gpiomux_get_clock_failed;
	}

	// We can give a stop sequence only when the data line is not being set to
	// low by any slave.
	// Give as many clock cycles as necessary in order to be able to abort any
	// pending transaction. This shouldn't take more than 9 cycles, but we'll
	// try a little bit longer, just in case.
	printk(KERN_DEBUG "%s: get control of the data line\n", __func__);
	for(i=0; i < I2C_RECOVER_CLOCK_CYCLES; i++)
	{
		if (gpio_get_value(GPIO_SENSORS_SDA) == 1)
			break;

		gpio_set_value(GPIO_SENSORS_SCL, 0);
		udelay(3);
		gpio_set_value(GPIO_SENSORS_SCL, 1);
		udelay(3);
	}
	if (i == I2C_RECOVER_CLOCK_CYCLES)
	{
		printk(KERN_ERR "%s: failed getting the data line after %d clock cycles\n", __func__, i);
		goto exit;
	}

	printk(KERN_DEBUG "%s Got conrol of the data line after %d cycles\n", __func__, i);

	// Reconfigure the data line as an output
	rc = msm_gpiomux_write(GPIO_SENSORS_SDA, GPIOMUX_ACTIVE, &new, NULL);
	if (rc < 0)  {
		printk(KERN_ERR "error %d 'muxing' gpio %d (i2c_data)\n", rc, GPIO_SENSORS_SDA);
		goto exit;
	}

	printk(KERN_DEBUG "%s: generate a START/STOP sequence\n", __func__);
	gpio_set_value(GPIO_SENSORS_SDA, 0);
	udelay(10);
	gpio_set_value(GPIO_SENSORS_SDA, 1);

exit:
	msm_gpiomux_put(GPIO_SENSORS_SCL);

msm_gpiomux_get_clock_failed:
	msm_gpiomux_write(GPIO_SENSORS_SCL, GPIOMUX_ACTIVE, &scl_old, NULL);

msm_gpiomux_write_clock_failed:
	gpio_free(GPIO_SENSORS_SCL);

gpio_request_clock_failed:
	msm_gpiomux_put(GPIO_SENSORS_SDA);

msm_gpiomux_get_data_failed:
	msm_gpiomux_write(GPIO_SENSORS_SDA, GPIOMUX_ACTIVE, &sda_old, NULL);

msm_gpiomux_write_data_failed:
	gpio_free(GPIO_SENSORS_SDA);

gpio_request_data_failed:
	return;
}


static struct msm_i2c_platform_data msm_gsbi3_qup_i2c_pdata = {
	.clk_freq = 100000,
	.src_clk_rate = 24000000,
	.clk = "gsbi_qup_clk",
	.pclk = "gsbi_pclk",
	.msm_i2c_config_gpio = gsbi_qup_i2c_gpio_config,
	.msm_i2c_recover = board_i2c_recover,
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

static int board_gsbi10_init(void);

static struct msm_i2c_platform_data msm_gsbi10_qup_i2c_pdata = {
	.clk_freq = 400000,
	.src_clk_rate = 24000000,
	.clk = "gsbi_qup_clk",
	.pclk = "gsbi_pclk",
	.use_gsbi_shared_mode = 1,
	.msm_i2c_config_gpio = gsbi_qup_i2c_gpio_config,
	.msm_i2c_config_gsbi = board_gsbi10_init,
};
#endif

#if defined(CONFIG_SPI_QUP) || defined(CONFIG_SPI_QUP_MODULE)
static struct msm_spi_platform_data msm_gsbi1_qup_spi_pdata = {
	.max_clock_speed = 24000000,
	.clk_name = "gsbi_qup_clk",
	.pclk_name = "gsbi_pclk",
};
#endif

#define GSBI1_PHYS			0x16000000
#define GSBI8_PHYS			0x19800000
#define GSBI_CTRL			0x0
#define PROTOCOL_CODE(code)		(((code) & 0x7) << 4)
#define UART_WITH_FLOW_CONTROL		0x4
#define I2C_ON_2_PORTS_UART		0x6

static DEFINE_MUTEX(gsbi_init_lock);

static int board_gsbi_init(int gsbi, int *inited, u32 prot)
{
	int rc;
	u32 gsbi_phys;
	void *gsbi_virt;

	pr_debug("%s: gsbi=%d inited=%d\n", __func__, gsbi, *inited);

	mutex_lock(&gsbi_init_lock);

	if (*inited) {
		rc = 0;
		goto exit;
	}

	pr_debug("%s: gsbi=%d prot=%x", __func__, gsbi, prot);

	if ((gsbi >= 1) && (gsbi <= 7))
		gsbi_phys = GSBI1_PHYS + ((gsbi - 1) * 0x100000);

	else if ((gsbi >= 8) && (gsbi <= 12))
		gsbi_phys = GSBI8_PHYS + ((gsbi - 8) * 0x100000);

	else {
		rc = -EINVAL;
		goto exit;
	}

	gsbi_virt = ioremap(gsbi_phys, 4);
	if (!gsbi_virt) {
		pr_err("error remapping address 0x%08x\n", gsbi_phys);
		rc = -ENXIO;
		goto exit;
	}

	pr_debug("%s: %08x=%08x\n", __func__, gsbi_phys + GSBI_CTRL,
			PROTOCOL_CODE(prot));
	writel(PROTOCOL_CODE(prot), gsbi_virt + GSBI_CTRL);
	iounmap(gsbi_virt);
	rc = 0;
exit:
	mutex_unlock(&gsbi_init_lock);

	return (rc);
}

static int board_gsbi6_init(void)
{
	static int inited = 0;

	return (board_gsbi_init(6, &inited, UART_WITH_FLOW_CONTROL));
}

static int board_gsbi10_init(void)
{
	static int inited = 0;

	return (board_gsbi_init(10, &inited, I2C_ON_2_PORTS_UART));
}

#if defined (CONFIG_TOUCHSCREEN_CY8CTMA395) \
	|| defined (CONFIG_TOUCHSCREEN_CY8CTMA395_MODULE)
static struct user_pin ctp_pins[] = {
	{
		.name = "wake",
  		.gpio = GPIO_CTP_WAKE,
		.act_level = 0,
		.direction = 0,
		.def_level = 1,
		.sysfs_mask = 0660,
	},
};

#define CTP_UART_SPEED_SLOW		3000000
#define CTP_UART_SPEED_FAST		4000000

static int ctp_uart_pin_mux(int on)
{
	int rc;
	static int is_on = 0;

	pr_debug("%s: on=%d\n", __func__, on);

	if (on && !is_on)
		rc = msm_gpiomux_get(GPIO_CTP_RX);

	else if (!on && is_on)
		rc = msm_gpiomux_put(GPIO_CTP_RX);

	else
		rc = 0;

	is_on = on;

	return (rc);
}

static struct hsuart_platform_data ctp_uart_data = {
	.dev_name = "ctp_uart",
	.uart_mode = HSUART_MODE_FLOW_CTRL_NONE | HSUART_MODE_PARITY_NONE,
	.uart_speed = CTP_UART_SPEED_SLOW,
	.options = HSUART_OPTION_RX_DM | HSUART_OPTION_SCHED_RT,

	.tx_buf_size = 4096,
	.tx_buf_num = 1,
	.rx_buf_size = 16384,
	.rx_buf_num = 2,
	.max_packet_size = 10240,
	.min_packet_size = 1,
	.rx_latency = CTP_UART_SPEED_FAST/20000,	/* bytes per 500 us */
	.p_board_pin_mux_cb = ctp_uart_pin_mux,
	.p_board_config_gsbi_cb = board_gsbi10_init,
};

static struct platform_device ctp_uart_device = {
	.name = "hsuart",
	.id =  1,
	.dev  = {
		.platform_data = &ctp_uart_data,
	}
};
#endif /* CONFIG_TOUCHSCREEN_CY8CTMA395[_MODULE] */

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

#ifdef CONFIG_FB_MSM_LCDC_DSUB
 /* VGA = 1440 x 900 x 4(bpp) x 2(pages)
    prim = 1024 x 600 x 4(bpp) x 2(pages)
    This is the difference. */
#define MSM_FB_DSUB_PMEM_ADDER (0x9E3400-0x4B0000)
#else
#define MSM_FB_DSUB_PMEM_ADDER (0)
#endif

#ifdef CONFIG_FB_MSM_TRIPLE_BUFFER
#define MSM_FB_PRIM_BUF_SIZE (1024 * 768 * 4 * 3) /* 4 bpp x 3 pages */
#else
#define MSM_FB_PRIM_BUF_SIZE (1024 * 768 * 4 * 2) /* 4 bpp x 2 pages */
#endif

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
#define MSM_FB_EXT_BUF_SIZE  (1920 * 1080 * 2 * 1) /* 2 bpp x 1 page */
#elif defined(CONFIG_FB_MSM_TVOUT)
#define MSM_FB_EXT_BUF_SIZE  (720 * 576 * 2 * 2) /* 2 bpp x 2 pages */
#else
#define MSM_FB_EXT_BUF_SIZE     0
#endif

#ifdef CONFIG_FB_MSM_OVERLAY_WRITEBACK
/* width x height x 3 bpp x 2 frame buffer */
#define MSM_FB_WRITEBACK_SIZE (1024 * 768 * 3 * 2)
#define MSM_FB_WRITEBACK_OFFSET  \
                (MSM_FB_PRIM_BUF_SIZE + MSM_FB_EXT_BUF_SIZE)
#else
#define MSM_FB_WRITEBACK_SIZE   0
#define MSM_FB_WRITEBACK_OFFSET 0
#endif


/* Note: must be multiple of 4096 */
#define MSM_FB_SIZE roundup(MSM_FB_PRIM_BUF_SIZE + MSM_FB_EXT_BUF_SIZE + \
                                 MSM_FB_WRITEBACK_SIZE + \
                                 MSM_FB_DSUB_PMEM_ADDER, 4096)

#define MSM_PMEM_SF_SIZE 0x4000000 /* 64 Mbytes */

static int writeback_offset(void)
{
         return MSM_FB_WRITEBACK_OFFSET;
}

#define MSM_OVERLAY_BLT_SIZE roundup(0x500000, 4096)

#define MSM_PMEM_KERNEL_EBI1_SIZE  0x600000
#define MSM_PMEM_ADSP_SIZE         0x2000000
#define MSM_PMEM_AUDIO_SIZE        0x239000

#define MSM_SMI_BASE          0x38000000
/* Kernel SMI PMEM Region for video core, used for Firmware */
/* and encoder,decoder scratch buffers */
/* Kernel SMI PMEM Region Should always precede the user space */
/* SMI PMEM Region, as the video core will use offset address */
/* from the Firmware base */
#define PMEM_KERNEL_SMI_BASE  (MSM_SMI_BASE)
#define PMEM_KERNEL_SMI_SIZE  0x300000
/* User space SMI PMEM Region for video core*/
/* used for encoder, decoder input & output buffers  */
#define MSM_PMEM_SMIPOOL_BASE (PMEM_KERNEL_SMI_BASE + PMEM_KERNEL_SMI_SIZE)
#define MSM_PMEM_SMIPOOL_SIZE 0x3D00000

/** allow the framebuffer's address to be passed from the bootloader on the command line */
static unsigned long fb_phys = 0;
static int __init fb_args(char *str)
{
	fb_phys = memparse(str, NULL);
	return 0;
}
early_param("fb", fb_args);

static unsigned fb_size = MSM_FB_SIZE;
static int __init fb_size_setup(char *p)
{
	fb_size = memparse(p, NULL);
	return 0;
}
early_param("fb_size", fb_size_setup);

#ifdef CONFIG_KERNEL_PMEM_EBI_REGION
static unsigned pmem_kernel_ebi1_size = MSM_PMEM_KERNEL_EBI1_SIZE;
static int __init pmem_kernel_ebi1_size_setup(char *p)
{
	pmem_kernel_ebi1_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_kernel_ebi1_size", pmem_kernel_ebi1_size_setup);
#endif

#ifdef CONFIG_ANDROID_PMEM
static unsigned pmem_sf_size = MSM_PMEM_SF_SIZE;
static int __init pmem_sf_size_setup(char *p)
{
	pmem_sf_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_sf_size", pmem_sf_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;

static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_adsp_size", pmem_adsp_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;

static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);
#endif

static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	},
	{
		.flags  = IORESOURCE_DMA,
	}
};

static int msm_fb_detect_panel(const char *name)
{
	if (!strcmp(name, "lcdc_lg_xga"))
		return 0;
	pr_warning("%s: not supported '%s'", __func__, name);
	return -ENODEV;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources     = ARRAY_SIZE(msm_fb_resources),
	.resource          = msm_fb_resources,
	.dev.platform_data = &msm_fb_pdata,
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
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 1,
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

static struct android_pmem_platform_data android_pmem_audio_pdata = {
	.name = "pmem_audio",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct platform_device android_pmem_audio_device = {
	.name = "android_pmem",
	.id = 4,
	.dev = { .platform_data = &android_pmem_audio_pdata },
};

static struct android_pmem_platform_data android_pmem_smipool_pdata = {
	.name = "pmem_smipool",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
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
static struct msm_panel_common_pdata lcdc_lg_panel_data = {
	.gpio_num = pmic_backlight_gpio, /* two LPG CHANNELS for backlight */
};

static struct platform_device lcdc_lg_panel_device = {
	.name = "lcdc_lg_xga",
	.id = 0,
	.dev = {
		.platform_data = &lcdc_lg_panel_data,
	}
};


static void __init msm8x60_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = MSM_FB_SIZE;
	if(fb_phys) {
		addr = (void *)fb_phys;
		msm_fb_resources[0].start = (unsigned long)addr;
		msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
		pr_info("passing from bootie %lu bytes at %lx physical for fb\n",
			size, fb_phys);
	}
	else {
		addr = alloc_bootmem(size);
		msm_fb_resources[0].start = __pa(addr);
		msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
		pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
			size, addr, __pa(addr));
	}
#if 0
	size = MSM_FB1_SIZE;
	addr = alloc_bootmem(size);
	msm_fb_resources[1].start = __pa(addr);
	msm_fb_resources[1].end = msm_fb_resources[1].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb1\n",
		size, addr, __pa(addr));
#endif
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

	size = MSM_PMEM_AUDIO_SIZE;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_audio_pdata.start = __pa(addr);
		android_pmem_audio_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for audio "
			"pmem arena\n", size, addr, __pa(addr));
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

struct touch_vreg_data {
	struct regulator *votg_l10;
	struct regulator *votg_vdd5v;
};
static struct touch_vreg_data mxt1386_vreg_data = {NULL, NULL};

static u8 get_touch_gpio_pin(void)
{
	return pin_table[MXT1386_TS_PEN_IRQ];
}

static void touch_poweron(struct touch_vreg_data *vreg_data, bool on)
{
	// power on
	struct regulator *votg_l10 = vreg_data->votg_l10;
	struct regulator *votg_vdd5v = vreg_data->votg_vdd5v;
	static bool isPowerOn = false;

	if (on == isPowerOn)
	{
		return;
	}

	if(on)
	{
		/* VDD_LVDS_3.3V ENABLE*/
		if (regulator_enable(votg_l10))
		{
			pr_err("%s: Unable to enable the regulator:"
			" votg_l10\n", __func__);
			regulator_disable(votg_vdd5v);
			return;
		}

		/* VDD_BACKLIGHT_5.0V ENABLE*/
		if (regulator_enable(votg_vdd5v))
		{
			pr_err("%s:Touch Unable to enable the regulator:"
			              " votg_vdd5v\n", __func__);
			return;
		}
	}
	else
	{
		regulator_disable(votg_l10);
		regulator_disable(votg_vdd5v);
	}

	isPowerOn = on;

	return;
}

static void touchscreen_gpio_release(void)
{
#if !defined (CONFIG_TOUCHSCREEN_CY8CTMA395) \
	&& !defined (CONFIG_TOUCHSCREEN_CY8CTMA395_MODULE)
	int irq_pin = get_touch_gpio_pin();

	gpio_free(irq_pin);
	gpio_free(MXT1386_TS_PWR_RST_GPIO);
#endif /* CONFIG_TOUCHSCREEN_CY8CTMA395[_MODULE] */
}

static int init_touch_hw(void)
{
	int rc;
	int irq_pin = get_touch_gpio_pin();
	unsigned irq_cfg =
		GPIO_CFG(irq_pin,  0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
	struct regulator *votg_l10, *votg_vdd5v;

	votg_l10 = regulator_get(NULL, "8058_l10");
	if (IS_ERR(votg_l10))
	{
		pr_err("%s: failed to get regulator \"8058_l10\"\n", __func__);
		return PTR_ERR(votg_l10);
	}

	votg_vdd5v = regulator_get(NULL, "vdd50_boost");
	if (IS_ERR(votg_vdd5v))
	{
		pr_err("%s: failed to get regulator \"vdd50_boost\"\n", __func__);
		regulator_put(votg_l10);
		return PTR_ERR(votg_vdd5v);
	}

	rc = regulator_set_voltage(votg_l10, 3050000, 3050000);
	if (rc)
	{
		pr_err("%s: Unable to set regulator voltage:"
						" votg_l10\n", __func__);
		goto err_vreg;
	}

	mxt1386_vreg_data.votg_l10 = votg_l10;
	mxt1386_vreg_data.votg_vdd5v = votg_vdd5v;

#if !defined (CONFIG_TOUCHSCREEN_CY8CTMA395) \
	&& !defined (CONFIG_TOUCHSCREEN_CY8CTMA395_MODULE)
	rc = gpio_request(MXT1386_TS_PWR_RST_GPIO, "touch_reset");
	if (rc)
	{
		pr_err("gpio_request failed on pin %d (rc=%d)\n",
				MXT1386_TS_PWR_RST_GPIO, rc);
		goto err_vreg;
	}
#endif /* CONFIG_TOUCHSCREEN_CY8CTMA395[_MODULE] */

	/* pull reset high */
	gpio_set_value(MXT1386_TS_PWR_RST_GPIO, 1);
	mdelay(100);

	// power on
	touch_poweron(&mxt1386_vreg_data, true);

	// power reset
	gpio_direction_output(MXT1386_TS_PWR_RST_GPIO, 0);
	mdelay(100);
	gpio_direction_output(MXT1386_TS_PWR_RST_GPIO, 1);
	mdelay(100);

#if !defined (CONFIG_TOUCHSCREEN_CY8CTMA395) \
	&& !defined (CONFIG_TOUCHSCREEN_CY8CTMA395_MODULE)
	rc = gpio_request(irq_pin, "msm_touchpad_irq");
	if (rc) {
		pr_err("gpio_request failed on pin %d (rc=%d)\n",
				irq_pin, rc);
		gpio_free(MXT1386_TS_PWR_RST_GPIO);
		goto err_vreg;
	}
#endif /* CONFIG_TOUCHSCREEN_CY8CTMA395[_MODULE] */

	rc = gpio_tlmm_config(irq_cfg, 0);
	if (rc) {
		pr_err("gpio_tlmm_config failed on pin %d (rc=%d)\n",
				irq_pin, rc);
		goto err_gpioconfig;
	}

	rc = gpio_direction_input(irq_pin);
	if (rc) {
		pr_err("gpio_direction_input failed on pin %d (rc=%d)\n",
				irq_pin, rc);
		goto err_gpioconfig;
	}

	return 0;

err_gpioconfig:
	touchscreen_gpio_release();
err_vreg:
	regulator_put(votg_l10);
	regulator_put(votg_vdd5v);
	return rc;
}

static void exit_touch_hw(void)
{
	touchscreen_gpio_release();
	touch_poweron(&mxt1386_vreg_data, false);

	regulator_put(mxt1386_vreg_data.votg_vdd5v);
	regulator_put(mxt1386_vreg_data.votg_l10);
}

/*
 * Reads the CHANGELINE state; interrupt is valid if the changeline
 * is low.
 */
static u8 read_chg(void)
{
	return gpio_get_value(pin_table[MXT1386_TS_PEN_IRQ]);
}

static u8 valid_interrupt(void)
{
	return !read_chg();
}

static struct mxt_platform_data msm_touchscreen_data = {
	/* Maximum number of simultaneous touches to report. */
	.numtouch         = 10,
	.init_platform_hw = &init_touch_hw,
	.exit_platform_hw = &exit_touch_hw,
	.max_x            = 1024,
	.max_y            = 768,
	.valid_interrupt  = &valid_interrupt,
	.read_chg         = &read_chg,
	.irq_gpio         = &get_touch_gpio_pin,
};

static struct i2c_board_info xMT1386_board_info[] = {
    {
        I2C_BOARD_INFO("maXTouch", 0x4C),
        .platform_data = &msm_touchscreen_data,
        .irq = MSM_GPIO_TO_INT(MXT1386_TS_PEN_IRQ_GPIO),
    }
};

#ifdef CONFIG_LEDS_LM8502

static struct lm8502_memory_config lm8502_memcfg = {
    .eng1_startpage = 0,
    .eng1_endpage = 0,
    .eng2_startpage = 1,
    .eng2_endpage = 1,
};

static struct led_cfg lm8502_ledlist_0[] = {
    [0] = { // LED1
        .type = LED_WHITE,
        .current_addr = 0x26,
        .control_addr = 0x06,
    },
};

static struct led_cfg lm8502_ledlist_1[] = {
    [0] = { // LED2
        .type = LED_WHITE,
        .current_addr = 0x27,
        .control_addr = 0x07,
    },
};

static struct lm8502_led_config lm8502_leds[] = {
    [0] = {
        //led class dev
        .cdev.name	= "core_navi_left",
        .cdev.max_brightness = 100, // 100 percent brightness
        .cdev.flags = LED_CORE_SUSPENDRESUME,
        //led list
        .led_list = lm8502_ledlist_0,
        .nleds = ARRAY_SIZE(lm8502_ledlist_0),
        //hw group
        .hw_group = LED_HW_GRP_NONE,
        //defualt value
        .default_max_current = 0x02, // Current full-scale setting, max 3mA
        .default_state = LED_OFF,
        .default_brightness = 100, // 100 percent brightness
    },
    [1] = {
        //led class dev
        .cdev.name	= "core_navi_right",
        .cdev.max_brightness = 100, // 100 percent brightness
        .cdev.flags = LED_CORE_SUSPENDRESUME,
        //led list
        .led_list = lm8502_ledlist_1,
        .nleds = ARRAY_SIZE(lm8502_ledlist_1),
        //hw group
        .hw_group = LED_HW_GRP_NONE,
        //defualt value
        .default_max_current = 0x02, // Current full-scale setting, max 12.5mA
        .default_state = LED_OFF,
        .default_brightness = 100, // 100 percent brightness
	},
};

static struct lm8502_platform_data lm8502_platform_data = {
    .enable_gpio = LM8502_LIGHTING_EN_GPIO,
    .interrupt_gpio = LM8502_LIGHTING_INT_IRQ_GPIO,
     //vib
    .vib_default_duty_cycle = 100, //percent
    .vib_default_direction = 0,
    .vib_invert_direction = 1,
    //flash or torch
    .flash_default_duration = 3,    // 3*32ms
    .flash_default_current = 500,   // 500mA
    .torch_default_current = 100,   // 100mA
    //leds
    .leds = lm8502_leds,
    .nleds = ARRAY_SIZE(lm8502_leds),
    //leds firmware memory configuration
    .memcfg = &lm8502_memcfg,
    //power save mode
    .power_mode = MISC_POWER_SAVE_ON,
    //others
    .dev_name = "lm8502",
};

static struct i2c_board_info lm8502_board_info[] = {
    {
        I2C_BOARD_INFO(LM8502_I2C_DEVICE, LM8502_I2C_ADDR),
        .platform_data = &lm8502_platform_data,
        .irq = MSM_GPIO_TO_INT(LM8502_LIGHTING_INT_IRQ_GPIO),
    }
};
#endif

#ifdef CONFIG_SERIAL_MSM_HS
static int configure_uart_gpios(int on)
{
	int ret = 0;
	int uart_gpios[] = {53, 54, 55, 56};

	ret = configure_gpiomux_gpios(on, uart_gpios, ARRAY_SIZE(uart_gpios));

	return ret;
}
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.inject_rx_on_wakeup = 1,
	.rx_to_inject = 0xFD,
	.gpio_config = configure_uart_gpios,
};
#endif

#if defined(CONFIG_MSM_RPM_LOG) || defined(CONFIG_MSM_RPM_LOG_MODULE)

static struct msm_rpm_log_platform_data msm_rpm_log_pdata = {
	.phys_addr_base = 0x00106000,
	.reg_offsets = {
		[MSM_RPM_LOG_PAGE_INDICES] = 0x00000C80,
		[MSM_RPM_LOG_PAGE_BUFFER]  = 0x00000CA0,
	},
	.phys_size = SZ_8K,
	.log_len = 4096,		  /* log's buffer length in bytes */
	.log_len_mask = (4096 >> 2) - 1,  /* length mask in units of u32 */
};

static struct platform_device msm_rpm_log_device = {
	.name	= "msm_rpm_log",
	.id	= -1,
	.dev	= {
		.platform_data = &msm_rpm_log_pdata,
	},
};
#endif

#ifdef CONFIG_BATTERY_MSM8X60
static struct msm_charger_platform_data msm_charger_data = {
	.safety_time = 180,
	.update_time = 1,
	.max_voltage = 4200,
	.min_voltage = 3200,
	.resume_voltage = 4100,
};

static struct platform_device msm_charger_device = {
	.name = "msm-charger",
	.id = -1,
	.dev = {
		.platform_data = &msm_charger_data,
	}
};
#endif

static struct regulator_consumer_supply rpm_vreg_supply[RPM_VREG_ID_MAX] = {
	[RPM_VREG_ID_PM8058_L0]  = REGULATOR_SUPPLY("8058_l0", NULL),
	[RPM_VREG_ID_PM8058_L1]  = REGULATOR_SUPPLY("8058_l1", NULL),
	[RPM_VREG_ID_PM8058_L2]  = REGULATOR_SUPPLY("8058_l2", NULL),
	[RPM_VREG_ID_PM8058_L3]  = REGULATOR_SUPPLY("8058_l3", NULL),
	[RPM_VREG_ID_PM8058_L4]  = REGULATOR_SUPPLY("8058_l4", NULL),
	[RPM_VREG_ID_PM8058_L5]  = REGULATOR_SUPPLY("8058_l5", NULL),
	[RPM_VREG_ID_PM8058_L6]  = REGULATOR_SUPPLY("8058_l6", NULL),
	[RPM_VREG_ID_PM8058_L7]  = REGULATOR_SUPPLY("8058_l7", NULL),
	[RPM_VREG_ID_PM8058_L8]  = REGULATOR_SUPPLY("8058_l8", NULL),
	[RPM_VREG_ID_PM8058_L9]  = REGULATOR_SUPPLY("8058_l9", NULL),
	[RPM_VREG_ID_PM8058_L10] = REGULATOR_SUPPLY("8058_l10", NULL),
	[RPM_VREG_ID_PM8058_L11] = REGULATOR_SUPPLY("8058_l11", NULL),
	[RPM_VREG_ID_PM8058_L12] = REGULATOR_SUPPLY("8058_l12", NULL),
	[RPM_VREG_ID_PM8058_L13] = REGULATOR_SUPPLY("8058_l13", NULL),
	[RPM_VREG_ID_PM8058_L14] = REGULATOR_SUPPLY("8058_l14", NULL),
	[RPM_VREG_ID_PM8058_L15] = REGULATOR_SUPPLY("8058_l15", NULL),
	[RPM_VREG_ID_PM8058_L16] = REGULATOR_SUPPLY("8058_l16", NULL),
	[RPM_VREG_ID_PM8058_L17] = REGULATOR_SUPPLY("8058_l17", NULL),
	[RPM_VREG_ID_PM8058_L18] = REGULATOR_SUPPLY("8058_l18", NULL),
	[RPM_VREG_ID_PM8058_L19] = REGULATOR_SUPPLY("8058_l19", NULL),
	[RPM_VREG_ID_PM8058_L20] = REGULATOR_SUPPLY("8058_l20", NULL),
	[RPM_VREG_ID_PM8058_L21] = REGULATOR_SUPPLY("8058_l21", NULL),
	[RPM_VREG_ID_PM8058_L22] = REGULATOR_SUPPLY("8058_l22", NULL),
	[RPM_VREG_ID_PM8058_L23] = REGULATOR_SUPPLY("8058_l23", NULL),
	[RPM_VREG_ID_PM8058_L24] = REGULATOR_SUPPLY("8058_l24", NULL),
	[RPM_VREG_ID_PM8058_L25] = REGULATOR_SUPPLY("8058_l25", NULL),

	[RPM_VREG_ID_PM8058_S0] = REGULATOR_SUPPLY("8058_s0", NULL),
	[RPM_VREG_ID_PM8058_S1] = REGULATOR_SUPPLY("8058_s1", NULL),
	[RPM_VREG_ID_PM8058_S2] = REGULATOR_SUPPLY("8058_s2", NULL),
	[RPM_VREG_ID_PM8058_S3] = REGULATOR_SUPPLY("8058_s3", NULL),
	[RPM_VREG_ID_PM8058_S4] = REGULATOR_SUPPLY("8058_s4", NULL),

	[RPM_VREG_ID_PM8058_LVS0] = REGULATOR_SUPPLY("8058_lvs0", NULL),
	[RPM_VREG_ID_PM8058_LVS1] = REGULATOR_SUPPLY("8058_lvs1", NULL),

	[RPM_VREG_ID_PM8058_NCP] = REGULATOR_SUPPLY("8058_ncp", NULL),

	[RPM_VREG_ID_PM8901_L0]  = REGULATOR_SUPPLY("8901_l0",  NULL),
	[RPM_VREG_ID_PM8901_L1]  = REGULATOR_SUPPLY("8901_l1",  NULL),
	[RPM_VREG_ID_PM8901_L2]  = REGULATOR_SUPPLY("8901_l2",  NULL),
	[RPM_VREG_ID_PM8901_L3]  = REGULATOR_SUPPLY("8901_l3",  NULL),
	[RPM_VREG_ID_PM8901_L4]  = REGULATOR_SUPPLY("8901_l4",  NULL),
	[RPM_VREG_ID_PM8901_L5]  = REGULATOR_SUPPLY("8901_l5",  NULL),
	[RPM_VREG_ID_PM8901_L6]  = REGULATOR_SUPPLY("8901_l6",  NULL),

	[RPM_VREG_ID_PM8901_S2] = REGULATOR_SUPPLY("8901_s2", NULL),
	[RPM_VREG_ID_PM8901_S3] = REGULATOR_SUPPLY("8901_s3", NULL),
	[RPM_VREG_ID_PM8901_S4] = REGULATOR_SUPPLY("8901_s4", NULL),

	[RPM_VREG_ID_PM8901_LVS0] = REGULATOR_SUPPLY("8901_lvs0", NULL),
	[RPM_VREG_ID_PM8901_LVS1] = REGULATOR_SUPPLY("8901_lvs1", NULL),
	[RPM_VREG_ID_PM8901_LVS2] = REGULATOR_SUPPLY("8901_lvs2", NULL),
	[RPM_VREG_ID_PM8901_LVS3] = REGULATOR_SUPPLY("8901_lvs3", NULL),
	[RPM_VREG_ID_PM8901_MVS0] = REGULATOR_SUPPLY("8901_mvs0", NULL),
};

#define RPM_VREG_INIT(_id, _min_uV, _max_uV, _modes, _ops, _apply_uV, \
		      _default_uV, _peak_uA, _avg_uA, _pull_down, _pin_ctrl, \
		      _freq, _pin_fn, _rpm_mode, _state, _sleep_selectable, \
		      _always_on) \
	[RPM_VREG_ID_##_id] = { \
		.init_data = { \
			.constraints = { \
				.valid_modes_mask = _modes, \
				.valid_ops_mask = _ops, \
				.min_uV = _min_uV, \
				.max_uV = _max_uV, \
				.input_uV = _min_uV, \
				.apply_uV = _apply_uV, \
				.always_on = _always_on, \
			}, \
			.num_consumer_supplies = 1, \
			.consumer_supplies = \
				&rpm_vreg_supply[RPM_VREG_ID_##_id], \
		}, \
		.default_uV = _default_uV, \
		.peak_uA = _peak_uA, \
		.avg_uA = _avg_uA, \
		.pull_down_enable = _pull_down, \
		.pin_ctrl = _pin_ctrl, \
		.freq = _freq, \
		.pin_fn = _pin_fn, \
		.mode = _rpm_mode, \
		.state = _state, \
		.sleep_selectable = _sleep_selectable, \
	}

/*
 * The default LPM/HPM state of an RPM controlled regulator can be controlled
 * via the peak_uA value specified in the table below.  If the value is less
 * than the high power min threshold for the regulator, then the regulator will
 * be set to LPM.  Otherwise, it will be set to HPM.
 *
 * This value can be further overridden by specifying an initial mode via
 * .init_data.constraints.initial_mode.
 */

#define RPM_VREG_INIT_LDO(_id, _always_on, _pd, _sleep_selectable, _min_uV, \
			  _max_uV, _init_peak_uA, _pin_ctrl) \
	RPM_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_FAST | \
		      REGULATOR_MODE_NORMAL | REGULATOR_MODE_IDLE | \
		      REGULATOR_MODE_STANDBY, REGULATOR_CHANGE_VOLTAGE | \
		      REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE | \
		      REGULATOR_CHANGE_DRMS, 0, _min_uV, _init_peak_uA, \
		      _init_peak_uA, _pd, _pin_ctrl, RPM_VREG_FREQ_NONE, \
		      RPM_VREG_PIN_FN_ENABLE, RPM_VREG_MODE_NONE, \
		      RPM_VREG_STATE_OFF, _sleep_selectable, _always_on)

#define RPM_VREG_INIT_LDO_PF(_id, _always_on, _pd, _sleep_selectable, _min_uV, \
			  _max_uV, _init_peak_uA, _pin_ctrl, _pin_fn) \
	RPM_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_FAST | \
		      REGULATOR_MODE_NORMAL | REGULATOR_MODE_IDLE | \
		      REGULATOR_MODE_STANDBY, REGULATOR_CHANGE_VOLTAGE | \
		      REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE | \
		      REGULATOR_CHANGE_DRMS, 0, _min_uV, _init_peak_uA, \
		      _init_peak_uA, _pd, _pin_ctrl, RPM_VREG_FREQ_NONE, \
		      _pin_fn, RPM_VREG_MODE_NONE, RPM_VREG_STATE_OFF, \
		      _sleep_selectable, _always_on)

#define RPM_VREG_INIT_SMPS(_id, _always_on, _pd, _sleep_selectable, _min_uV, \
			   _max_uV, _init_peak_uA, _pin_ctrl, _freq) \
	RPM_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_FAST | \
		      REGULATOR_MODE_NORMAL | REGULATOR_MODE_IDLE | \
		      REGULATOR_MODE_STANDBY, REGULATOR_CHANGE_VOLTAGE | \
		      REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE | \
		      REGULATOR_CHANGE_DRMS, 0, _min_uV, _init_peak_uA, \
		      _init_peak_uA, _pd, _pin_ctrl, _freq, \
		      RPM_VREG_PIN_FN_ENABLE, RPM_VREG_MODE_NONE, \
		      RPM_VREG_STATE_OFF, _sleep_selectable, _always_on)

#define RPM_VREG_INIT_VS(_id, _always_on, _pd, _sleep_selectable, _pin_ctrl) \
	RPM_VREG_INIT(_id, 0, 0, REGULATOR_MODE_NORMAL | REGULATOR_MODE_IDLE, \
		      REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE, 0, 0, \
		      1000, 1000, _pd, _pin_ctrl, RPM_VREG_FREQ_NONE, \
		      RPM_VREG_PIN_FN_ENABLE, RPM_VREG_MODE_NONE, \
		      RPM_VREG_STATE_OFF, _sleep_selectable, _always_on)

#define RPM_VREG_INIT_NCP(_id, _always_on, _pd, _sleep_selectable, _min_uV, \
			  _max_uV, _pin_ctrl) \
	RPM_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_NORMAL, \
		      REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS, 0, \
		      _min_uV, 1000, 1000, _pd, _pin_ctrl, RPM_VREG_FREQ_NONE, \
		      RPM_VREG_PIN_FN_ENABLE, RPM_VREG_MODE_NONE, \
		      RPM_VREG_STATE_OFF, _sleep_selectable, _always_on)

#define LDO50HMIN	RPM_VREG_LDO_50_HPM_MIN_LOAD
#define LDO150HMIN	RPM_VREG_LDO_150_HPM_MIN_LOAD
#define LDO300HMIN	RPM_VREG_LDO_300_HPM_MIN_LOAD
#define SMPS_HMIN	RPM_VREG_SMPS_HPM_MIN_LOAD
#define FTS_HMIN	RPM_VREG_FTSMPS_HPM_MIN_LOAD

static struct rpm_vreg_pdata rpm_vreg_init_pdata[RPM_VREG_ID_MAX] = {
	RPM_VREG_INIT_LDO_PF(PM8058_L0,  0, 1, 0, 1200000, 1200000, LDO150HMIN,
		RPM_VREG_PIN_CTRL_NONE, RPM_VREG_PIN_FN_SLEEP_B),
	RPM_VREG_INIT_LDO(PM8058_L1,  0, 1, 0, 1200000, 1200000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L2,  0, 1, 0, 1800000, 2600000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L3,  0, 1, 0, 1800000, 1800000, LDO150HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L4,  0, 1, 0, 2850000, 2850000,  LDO50HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L5,  0, 1, 0, 2850000, 2850000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L6,  0, 1, 0, 3000000, 3600000,  LDO50HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L7,  0, 1, 0, 1800000, 1800000,  LDO50HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L8,  0, 1, 0, 2900000, 3050000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L9,  0, 1, 0, 1800000, 1800000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L10, 0, 1, 0, 3050000, 3050000, LDO300HMIN, 0), // Palm changed
//	RPM_VREG_INIT_LDO(PM8058_L10, 0, 1, 0, 2600000, 2600000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L11, 0, 1, 0, 2850000, 2850000, LDO150HMIN, 0), // Palm changed
//	RPM_VREG_INIT_LDO(PM8058_L11, 0, 1, 0, 1500000, 1500000, LDO150HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L12, 0, 1, 0, 1800000, 1800000, LDO150HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L13, 0, 1, 0, 2050000, 2050000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L14, 0, 0, 0, 2850000, 2850000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L15, 0, 1, 0, 2850000, 2850000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L16, 1, 1, 1, 1800000, 1800000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L17, 0, 1, 0, 2600000, 2600000, LDO150HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L18, 0, 1, 1, 2200000, 2200000, LDO150HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L19, 0, 1, 0, 1800000, 1800000, LDO150HMIN, 0), // Palm changed
//	RPM_VREG_INIT_LDO(PM8058_L19, 0, 1, 0, 2500000, 2500000, LDO150HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L20, 0, 1, 0, 1800000, 1800000, LDO150HMIN, 0),
	RPM_VREG_INIT_LDO_PF(PM8058_L21, 1, 1, 0, 1200000, 1200000, LDO150HMIN,
		RPM_VREG_PIN_CTRL_NONE, RPM_VREG_PIN_FN_SLEEP_B),
	RPM_VREG_INIT_LDO(PM8058_L22, 0, 1, 0, 1200000, 1200000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L23, 0, 1, 0, 1200000, 1200000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L24, 0, 1, 0, 1200000, 1200000, LDO150HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L25, 0, 1, 0, 1200000, 1200000, LDO150HMIN, 0),

	RPM_VREG_INIT_SMPS(PM8058_S0, 0, 1, 1,  500000, 1350000,  SMPS_HMIN, 0,
		RPM_VREG_FREQ_1p60),
	RPM_VREG_INIT_SMPS(PM8058_S1, 0, 1, 1,  500000, 1350000,  SMPS_HMIN, 0,
		RPM_VREG_FREQ_1p60),
	RPM_VREG_INIT_SMPS(PM8058_S2, 0, 1, 0, 1200000, 1400000,  SMPS_HMIN,
		RPM_VREG_PIN_CTRL_A0, RPM_VREG_FREQ_1p60),
	RPM_VREG_INIT_SMPS(PM8058_S3, 1, 1, 0, 1800000, 1800000,  SMPS_HMIN, 0,
		RPM_VREG_FREQ_1p60),
	RPM_VREG_INIT_SMPS(PM8058_S4, 1, 1, 0, 2200000, 2200000,  SMPS_HMIN, 0,
		RPM_VREG_FREQ_1p60),

	RPM_VREG_INIT_VS(PM8058_LVS0, 0, 1, 0,				 0),
	RPM_VREG_INIT_VS(PM8058_LVS1, 0, 1, 0,				 0),

	RPM_VREG_INIT_NCP(PM8058_NCP, 0, 1, 0, 1800000, 1800000,	 0),

	RPM_VREG_INIT_LDO(PM8901_L0,  0, 1, 0, 1200000, 1200000, LDO300HMIN,
		RPM_VREG_PIN_CTRL_A0),
	RPM_VREG_INIT_LDO(PM8901_L1,  0, 1, 0, 3300000, 3300000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8901_L2,  0, 1, 0, 2850000, 3300000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8901_L3,  0, 1, 0, 3300000, 3300000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8901_L4,  0, 1, 0, 2600000, 2600000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8901_L5,  0, 1, 0, 2850000, 2850000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8901_L6,  0, 1, 0, 2200000, 2200000, LDO300HMIN, 0),

	RPM_VREG_INIT_SMPS(PM8901_S2, 0, 1, 0, 1300000, 1300000,   FTS_HMIN, 0,
		RPM_VREG_FREQ_1p60),
	RPM_VREG_INIT_SMPS(PM8901_S3, 0, 1, 0, 1100000, 1100000,   FTS_HMIN, 0,
		RPM_VREG_FREQ_1p60),
	RPM_VREG_INIT_SMPS(PM8901_S4, 0, 1, 0, 1225000, 1225000,   FTS_HMIN,
		RPM_VREG_PIN_CTRL_A0, RPM_VREG_FREQ_1p60),

	RPM_VREG_INIT_VS(PM8901_LVS0, 1, 1, 0,				 0),
	RPM_VREG_INIT_VS(PM8901_LVS1, 0, 1, 0,				 0),
	RPM_VREG_INIT_VS(PM8901_LVS2, 0, 1, 0,				 0),
	RPM_VREG_INIT_VS(PM8901_LVS3, 0, 1, 0,				 0),
	RPM_VREG_INIT_VS(PM8901_MVS0, 0, 1, 0,				 0),
};

#define RPM_VREG(_id) \
	[_id] = { \
		.name = "rpm-regulator", \
		.id = _id, \
		.dev = { \
			.platform_data = &rpm_vreg_init_pdata[_id], \
		}, \
	}

static struct platform_device rpm_vreg_device[RPM_VREG_ID_MAX] = {
	RPM_VREG(RPM_VREG_ID_PM8058_L0),
	RPM_VREG(RPM_VREG_ID_PM8058_L1),
	RPM_VREG(RPM_VREG_ID_PM8058_L2),
	RPM_VREG(RPM_VREG_ID_PM8058_L3),
	RPM_VREG(RPM_VREG_ID_PM8058_L4),
	RPM_VREG(RPM_VREG_ID_PM8058_L5),
	RPM_VREG(RPM_VREG_ID_PM8058_L6),
	RPM_VREG(RPM_VREG_ID_PM8058_L7),
	RPM_VREG(RPM_VREG_ID_PM8058_L8),
	RPM_VREG(RPM_VREG_ID_PM8058_L9),
	RPM_VREG(RPM_VREG_ID_PM8058_L10),
	RPM_VREG(RPM_VREG_ID_PM8058_L11),
	RPM_VREG(RPM_VREG_ID_PM8058_L12),
	RPM_VREG(RPM_VREG_ID_PM8058_L13),
	RPM_VREG(RPM_VREG_ID_PM8058_L14),
	RPM_VREG(RPM_VREG_ID_PM8058_L15),
	RPM_VREG(RPM_VREG_ID_PM8058_L16),
	RPM_VREG(RPM_VREG_ID_PM8058_L17),
	RPM_VREG(RPM_VREG_ID_PM8058_L18),
	RPM_VREG(RPM_VREG_ID_PM8058_L19),
	RPM_VREG(RPM_VREG_ID_PM8058_L20),
	RPM_VREG(RPM_VREG_ID_PM8058_L21),
	RPM_VREG(RPM_VREG_ID_PM8058_L22),
	RPM_VREG(RPM_VREG_ID_PM8058_L23),
	RPM_VREG(RPM_VREG_ID_PM8058_L24),
	RPM_VREG(RPM_VREG_ID_PM8058_L25),
	RPM_VREG(RPM_VREG_ID_PM8058_S0),
	RPM_VREG(RPM_VREG_ID_PM8058_S1),
	RPM_VREG(RPM_VREG_ID_PM8058_S2),
	RPM_VREG(RPM_VREG_ID_PM8058_S3),
	RPM_VREG(RPM_VREG_ID_PM8058_S4),
	RPM_VREG(RPM_VREG_ID_PM8058_LVS0),
	RPM_VREG(RPM_VREG_ID_PM8058_LVS1),
	RPM_VREG(RPM_VREG_ID_PM8058_NCP),
	RPM_VREG(RPM_VREG_ID_PM8901_L0),
	RPM_VREG(RPM_VREG_ID_PM8901_L1),
	RPM_VREG(RPM_VREG_ID_PM8901_L2),
	RPM_VREG(RPM_VREG_ID_PM8901_L3),
	RPM_VREG(RPM_VREG_ID_PM8901_L4),
	RPM_VREG(RPM_VREG_ID_PM8901_L5),
	RPM_VREG(RPM_VREG_ID_PM8901_L6),
	RPM_VREG(RPM_VREG_ID_PM8901_S2),
	RPM_VREG(RPM_VREG_ID_PM8901_S3),
	RPM_VREG(RPM_VREG_ID_PM8901_S4),
	RPM_VREG(RPM_VREG_ID_PM8901_LVS0),
	RPM_VREG(RPM_VREG_ID_PM8901_LVS1),
	RPM_VREG(RPM_VREG_ID_PM8901_LVS2),
	RPM_VREG(RPM_VREG_ID_PM8901_LVS3),
	RPM_VREG(RPM_VREG_ID_PM8901_MVS0),
};

static struct platform_device *early_regulators[] __initdata = {
	&msm_device_saw_s0,
	&msm_device_saw_s1,
#ifdef CONFIG_PMIC8058
	&rpm_vreg_device[RPM_VREG_ID_PM8058_S0],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_S1],
#endif
};
#ifdef CONFIG_HSUART

static int btuart_pin_mux(int on)
{
	int ret = 0;
	int gpios[] = {UART1DM_CTS_GPIO, UART1DM_RX_GPIO, UART1DM_TX_GPIO};

	printk(KERN_INFO "btuart_pin_mux: %s\n", on?"on":"off");

	ret = configure_gpiomux_gpios(on, gpios, ARRAY_SIZE(gpios));

	return ret;
}

static int btuart_deassert_rts(int deassert)
{
	int rc = 0;
	static int active = 0;
	printk(KERN_INFO "btuart_deassert_rts: %d(%s)\n", deassert, deassert?"put":"get");
	if (deassert) {
		if (active) {
			rc = msm_gpiomux_put(UART1DM_RTS_GPIO);
			if (!rc)
				active = 0;
		}
	} else {
		if (!active) {
			rc = msm_gpiomux_get(UART1DM_RTS_GPIO);
			if (!rc)
				active = 1;
		}
	}

	return rc;
}


/*
 * BT High speed UART interface
 */
static struct hsuart_platform_data btuart_data = {
	.dev_name   = "ttyHS0",
	.uart_mode  = HSUART_MODE_FLOW_CTRL_NONE | HSUART_MODE_PARITY_NONE,
	.uart_speed = HSUART_SPEED_115K,
	.options    = HSUART_OPTION_DEFERRED_LOAD | HSUART_OPTION_TX_PIO | HSUART_OPTION_RX_DM ,

	.tx_buf_size = 512,
	.tx_buf_num  = 64,
	.rx_buf_size = 512,
	.rx_buf_num  = 64,
	.max_packet_size = 450, // ~450
	.min_packet_size = 6,   // min packet size
	.rx_latency      = 10, // in bytes at current speed
	.p_board_pin_mux_cb = btuart_pin_mux,
	.p_board_config_gsbi_cb = board_gsbi6_init,
	.p_board_rts_pin_deassert_cb = btuart_deassert_rts,
//	.rts_pin         = 145,   // uart rts line pin
};

static u64 btuart_dmamask = ~(u32)0;
static struct platform_device btuart_device = {
	.name = "hsuart_tty",
	.id   =  0, // configure UART2 as hi speed uart
	.dev  = {
		.dma_mask           = &btuart_dmamask,
		.coherent_dma_mask  = 0xffffffff,
		.platform_data      = &btuart_data,
	}
};
#endif // CONFIG_HSUART

#ifdef CONFIG_KEYBOARD_GPIO_PE
static struct gpio_keys_button board_gpio_keys_buttons[] = {
	{
		.code        = KEY_VOLUMEUP,
		.gpio        = VOL_UP_GPIO,
		.active_low  = 1,
		.desc        = "volume up",
		.debounce    = 20,
		.type	     = EV_KEY,
		.wakeup      = 0,
#ifdef  CONFIG_GPIO_KEYS_CONSOLE_TRIGGER
		.options     = OPT_CONSOLE_TRIGGER,
#endif
	},
	{
		.code        = KEY_VOLUMEDOWN,
		.gpio        = VOL_DN_GPIO,
		.active_low  = 1,
		.desc        = "volume down",
		.debounce    = 20,
		.type	     = EV_KEY,
		.wakeup      = 0,
	},
	{
		.code        = KEY_CENTER,
		.gpio        = CORE_NAVI_GPIO,
		.active_low  = 1,
		.desc        = "core navi",
		.debounce    = 20,
		.type        = EV_KEY,
		.wakeup      = 1,
		.is_wake_source = msm_gpio_is_wake_source,
#ifdef  CONFIG_GPIO_KEYS_CONSOLE_TRIGGER
		.options     = OPT_CONSOLE_TRIGGER,
#endif
	},
#ifdef CONFIG_USB_PEHCI_HCD
	{
		.code        = KEY_UIM_INSERT,
		.gpio        = (GPIO_3G_UIM_CD_N),
		.active_low  = 1,
		.desc        = "mdmuim",
		.debounce    = 500,
		.type        = EV_KEY,
		.wakeup      = 1,
                .is_wake_source = msm_gpio_is_wake_source,
	}
#endif
};

static struct gpio_keys_platform_data board_gpio_keys = {
	.buttons  = board_gpio_keys_buttons,
	.nbuttons = ARRAY_SIZE(board_gpio_keys_buttons),
};

static struct platform_device board_gpio_keys_device = {
	.name = "gpio-keys",
	.id   = -1,
	.dev  = {
		.platform_data  = &board_gpio_keys,
	},
};
#endif

#ifdef CONFIG_KEYBOARD_GPIO_PE
static void __init board_gpio_keys_init(void)
{
	(void) platform_device_register(&board_gpio_keys_device);
}
#endif


#ifdef CONFIG_USER_PINS

static struct user_pin bt_pins[] = {
	{
		.name       =  "reset",
		.gpio       =  BT_RST_N,
		.act_level  =  0, // active low
		.direction  =  0, // output
		.def_level  =  0, // low
		.pin_mode   =  (void *)-1,// undefined
		.sysfs_mask =  0777,
		.options    =  0,
		.irq_handler = NULL,
		.irq_config =  0,
	},
	{
		.name       =  "host_wake",
		.gpio       =  BT_HOST_WAKE,
		.act_level  =  1,  // active high
		.direction  =  1,  // input
		.def_level  = -1,  // undefined
		.pin_mode   =  (void *)-1, // undefined
		.sysfs_mask =  0777,
		.options    =  PIN_IRQ | PIN_WAKEUP_SOURCE,
		.irq_handler = NULL,
		.irq_config = IRQF_TRIGGER_RISING,
		.irq_handle_mode = IRQ_HANDLE_AUTO
	},
};

static struct user_pin_set  board_user_pins_sets[] = {
	{
		.set_name = "bt",
		.num_pins = ARRAY_SIZE(bt_pins),
		.pins     = bt_pins,
	},
#if defined (CONFIG_TOUCHSCREEN_CY8CTMA395) \
	|| defined (CONFIG_TOUCHSCREEN_CY8CTMA395_MODULE)
	{
		.set_name = "ctp",
		.num_pins = ARRAY_SIZE(ctp_pins),
		.pins = ctp_pins,
	},
#endif /* CONFIG_TOUCHSCREEN_CY8CTMA395[_MODULE] */
};

static struct user_pins_platform_data board_user_pins_pdata = {
	.num_sets = ARRAY_SIZE(board_user_pins_sets),
	.sets     = board_user_pins_sets,
};

static struct platform_device board_user_pins_device = {
	.name = "user-pins",
	.id   = -1,
	.dev  = {
		.platform_data  = &board_user_pins_pdata,
	}
};
#endif

static struct platform_device tenderloin_rfkill = {
	.name = "tenderloin_rfkill",
	.id = -1,
};


#ifdef CONFIG_HRES_COUNTER
static int msm_hres_timer_init(void** timer)
{
	return 0;
}

static int msm_hres_timer_release(void* timer)
{
	return 0;
}

#ifdef CONFIG_PM
static int msm_hres_timer_suspend(void *timer)
{
	return 0;
}

static int msm_hres_timer_resume(void *timer)
{
	return 0;
}
#else
#define msm_hres_timer_suspend    NULL
#define msm_hres_timer_resume     NULL
#endif

extern u32 msm_dgt_read_count(void);
static u32 msm_hres_timer_read(void* timer)
{
	return msm_dgt_read_count();
}

extern u32 msm_dgt_convert_usec(u32);
static u32 msm_hres_timer_convert(u32 count)
{
	// Count is in 24.576Mhz terms
	// convert it to uSec
	// return (count * 400) / 2457;
	return msm_dgt_convert_usec(count);
}

static struct hres_counter_platform_data msm_hres_counter_platform_data = {
	.init_hres_timer = msm_hres_timer_init,
	.release_hres_timer = msm_hres_timer_release,
	.suspend_hres_timer = msm_hres_timer_suspend,
	.resume_hres_timer = msm_hres_timer_resume,
	.read_hres_timer = msm_hres_timer_read,
	.convert_hres_timer = msm_hres_timer_convert,
};

static struct platform_device hres_counter_device = {
	.name = "hres_counter",
	.id   = -1,
	.dev  = {
		.platform_data  = &msm_hres_counter_platform_data,
	}
};
#endif

#if defined (CONFIG_TOUCHSCREEN_CY8CTMA395) \
	|| defined (CONFIG_TOUCHSCREEN_CY8CTMA395_MODULE)
static int board_cy8ctma395_gpio_request(unsigned gpio, const char *name,
						int request, int *requested,
						struct gpiomux_setting *new,
						struct gpiomux_setting *old,
						int *replaced)
{
	int rc;

	if (request && !*requested) {
		rc = gpio_request(gpio, name);
		if (rc < 0) {
			pr_err("error %d requesting gpio %u (%s)\n", rc, gpio, name);
			goto gpio_request_failed;
		}

		rc = msm_gpiomux_write(gpio, GPIOMUX_ACTIVE, new, old);
		if (rc < 0)  {
			pr_err("error %d muxing gpio %u (%s)\n", rc, gpio, name);
			goto msm_gpiomux_write_failed;
		}

		*replaced = !rc;

		rc = msm_gpiomux_get(gpio);
		if (rc < 0)  {
			pr_err("error %d 'getting' gpio %u (%s)\n", rc, gpio, name);
			goto msm_gpiomux_get_failed;
		}

		*requested = 1;
	}

	else if (!request && *requested) {
		msm_gpiomux_put(gpio);
		msm_gpiomux_write(gpio, GPIOMUX_ACTIVE, *replaced ? old : NULL, NULL);
		gpio_free(gpio);
		*requested = 0;
	}

	return (0);

msm_gpiomux_get_failed:
	msm_gpiomux_write(gpio, GPIOMUX_ACTIVE, *replaced ? old : NULL, NULL);
msm_gpiomux_write_failed:
	gpio_free(gpio);
gpio_request_failed:

	return (rc);
}

static int board_cy8ctma395_swdck_request(int request)
{
	static int replaced = 0;
	static int requested = 0;
	static struct gpiomux_setting scl;

	struct gpiomux_setting swdck = {
		.func = GPIOMUX_FUNC_GPIO,
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_OUT_HIGH,
	};

	return (board_cy8ctma395_gpio_request(GPIO_CTP_SCL, "CY8CTMA395_SWDCK",
						request, &requested, &swdck,
						&scl, &replaced));
}

static int board_cy8ctma395_swdio_request(int request)
{
	static int replaced = 0;
	static int requested = 0;
	static struct gpiomux_setting sda;

	struct gpiomux_setting swdio = {
		.func = GPIOMUX_FUNC_GPIO,
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_OUT_HIGH,
	};

	return (board_cy8ctma395_gpio_request(GPIO_CTP_SDA, "CY8CTMA395_SWDIO",
						request, &requested, &swdio,
						&sda, &replaced));
}

static void board_cy8ctma395_vdd_enable(int enable)
{
	int rc;
	static struct regulator *tp_5v0 = NULL;
	static struct regulator *tp_l10 = NULL;
	static int isPowerOn = 0;

	if (!tp_l10) {
		tp_l10 = regulator_get(NULL, "8058_l10");
		if (IS_ERR(tp_l10)) {
			pr_err("%s: failed to get regulator \"8058_l10\"\n", __func__);
			return;
		}

		rc = regulator_set_voltage(tp_l10, 3050000, 3050000);
		if (rc) {
			pr_err("%s: Unable to set regulator voltage:"
			       " tp_l10\n", __func__);
			regulator_put(tp_l10);
			tp_l10 = NULL;
			return;
		}
	}

	if (!tp_5v0) {
		tp_5v0 = regulator_get(NULL, "vdd50_boost");
		if (IS_ERR(tp_5v0)) {
			pr_err("failed to get regulator 'vdd50_boost' with %ld\n",
				PTR_ERR(tp_5v0));
			regulator_put(tp_l10);
			tp_l10 = NULL;
			tp_5v0 = NULL;
			return;
		}
	}

	if (enable == isPowerOn) {
		return;
	}

	if (enable) {
		rc = regulator_enable(tp_l10);
		if (rc < 0) {
			pr_err("failed to enable regulator '8058_l10' with %d\n", rc);
			return;
		}

		rc = regulator_enable(tp_5v0);
		if (rc < 0) {
			pr_err("failed to enable regulator 'vdd50_boost' with %d\n", rc);
			return;
		}

		// Make sure the voltage is stabilized
		msleep(2);
	}

	else {
		rc = regulator_disable(tp_5v0);
		if (rc < 0) {
			pr_err("failed to disable regulator 'vdd50_boost' with %d\n", rc);
			return;
		}

		rc = regulator_disable(tp_l10);
		if (rc < 0) {
			pr_err("failed to enable regulator '8058_l10' with %d\n", rc);
			return;
		}
	}

	isPowerOn = enable;
}

static struct cy8ctma395_platform_data board_cy8ctma395_data = {
	.swdck_request = board_cy8ctma395_swdck_request,
	.swdio_request = board_cy8ctma395_swdio_request,
	.vdd_enable = board_cy8ctma395_vdd_enable,
	.xres = GPIO_CY8CTMA395_XRES,
	.xres_us = 1000,
	.swdck = GPIO_CTP_SCL,
	.swdio = GPIO_CTP_SDA,
	.swd_wait_retries = 0,
	.port_acquire_retries = 4,
	.status_reg_timeout_ms = 1000,
	.nr_blocks = 256,
};

static struct platform_device board_cy8ctma395_device = {
	.name = CY8CTMA395_DEVICE,
	.id = -1,
	.dev = {
		.platform_data = &board_cy8ctma395_data,
         },
};

static struct platform_device msm_wlan_pm_device = {
	.name = "wlan_ar6000_pm",
	.id = -1,
};

static int tenderloin_tp_init(int on)
{
	int ret;
	int tp_gpios[] = {GPIO_CTP_SCL, GPIO_CTP_SDA};
	ret = configure_gpiomux_gpios(on, tp_gpios, ARRAY_SIZE(tp_gpios));
	if (ret < 0) {
		printk(KERN_ERR "%s: Error %d while configuring touch panel gpios.\n", __func__, ret);
	}
	return ret;
}

#endif /* CONFIG_TOUCHSCREEN_CY8CTMA395[_MODULE] */

static struct platform_device *early_devices[] __initdata = {
#ifdef CONFIG_MSM_BUS_SCALING
	&msm_bus_apps_fabric,
	&msm_bus_sys_fabric,
	&msm_bus_mm_fabric,
	&msm_bus_sys_fpb,
	&msm_bus_cpss_fpb,
#endif
};


static struct platform_device *tenderloin_devices_early[] __initdata = {
	&msm_device_smd,
	&msm_device_uart_dm12,
	&msm_device_dmov_adm0,
	&msm_device_dmov_adm1,
};

static struct platform_device *tenderloin_devices[] __initdata = {
#ifdef CONFIG_I2C_QUP
	&msm_gsbi3_qup_i2c_device,
	&msm_gsbi4_qup_i2c_device,
	&msm_gsbi7_qup_i2c_device,
	&msm_gsbi8_qup_i2c_device,
	&msm_gsbi9_qup_i2c_device,
	&msm_gsbi10_qup_i2c_device,
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

#ifdef CONFIG_MSM_DSPS
	&msm_dsps_device,
#endif

#ifdef CONFIG_MAX8903B_CHARGER
	&max8903b_charger_device,
#endif
#if defined(CONFIG_USB_GADGET_MSM_72K) || defined(CONFIG_USB_EHCI_HCD)
	&msm_device_otg,
#endif
#ifdef CONFIG_USB_GADGET_MSM_72K
	&msm_device_gadget_peripheral,
#endif
#ifdef CONFIG_BATTERY_MSM
	&msm_batt_device,
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
	&android_pmem_audio_device,
	&android_pmem_smipool_device,
#endif
#ifdef CONFIG_MSM_ROTATOR
	&msm_rotator_device,
#endif
	&msm_fb_device,
	&msm_kgsl_3d0,
#ifdef CONFIG_MSM_KGSL_2D
	&msm_kgsl_2d0,
	&msm_kgsl_2d1,
#endif
#ifdef CONFIG_FB_MSM_LCDC_LG_XGA_PANEL
	&lcdc_lg_panel_device,
#endif

#ifdef CONFIG_MSM_CAMERA
#ifdef CONFIG_WEBCAM_MT9M113
	&msm_camera_sensor_webcam_mt9m113,
#endif
#endif //CONFIG_MSM_CAMERA

#ifdef CONFIG_MSM_GEMINI
	&msm_gemini_device,
#endif
#ifdef CONFIG_MSM_VPE
	&msm_vpe_device,
#endif
#if defined(CONFIG_MSM_RPM_LOG) || defined(CONFIG_MSM_RPM_LOG_MODULE)
	&msm_rpm_log_device,
#endif
#ifdef CONFIG_HSUART
	&btuart_device,
#endif
#ifdef CONFIG_HRES_COUNTER
	&hres_counter_device,
#endif
#ifdef CONFIG_USER_PINS
	&board_user_pins_device,
#endif
	&tenderloin_rfkill,
	&msm_device_vidc,
#ifdef CONFIG_PMIC8058
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L0],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L1],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L2],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L3],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L4],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L5],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L6],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L7],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L8],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L9],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L10],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L11],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L12],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L13],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L14],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L15],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L16],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L17],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L18],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L19],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L20],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L21],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L22],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L23],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L24],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L25],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_S2],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_S3],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_S4],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_LVS0],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_LVS1],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_NCP],
#endif
#ifdef CONFIG_PMIC8901
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L0],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L1],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L2],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L3],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L4],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L5],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L6],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_S2],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_S3],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_S4],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_LVS0],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_LVS1],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_LVS2],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_LVS3],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_MVS0],
#endif

	&tenderloin_fixed_reg_device[0],
#ifdef CONFIG_NDUID
	&nduid_device,
#endif
#if defined (CONFIG_TOUCHSCREEN_CY8CTMA395) \
	|| defined (CONFIG_TOUCHSCREEN_CY8CTMA395_MODULE)
 	&board_cy8ctma395_device,
 	&msm_device_uart_dm2,
 	&ctp_uart_device,
#endif /* CONFIG_TOUCHSCREEN_CY8CTMA395[_MODULE] */

//#if defined (CONFIG_ATH6K_LEGACY)	\
//	|| defined (CONFIG_ATH6K_LEGACY_MODULE)
	&msm_wlan_pm_device,
//#endif

};

#ifdef CONFIG_PMIC8058

static int pm8058_gpios_init(void)
{
	int i;
	int rc;
	struct pm8058_gpio_cfg {
		int                gpio;
		struct pm8058_gpio cfg;
	};

	struct pm8058_gpio_cfg gpio_cfgs[] = {
		{
            /*LCD BL PWM, PMIC GPIO24 in schematic*/
			23,
			{
				.direction      = PM_GPIO_DIR_OUT,
				.pull           = PM_GPIO_PULL_NO,
				.vin_sel        = PM_GPIO_VIN_VPH,
				.function       = PM_GPIO_FUNC_2,
				.inv_int_pol    = 0,
			},
		},
		{
            /*LCD BL ENABLE, PMIC GPIO25 in schematic*/
			24,
			{
				.direction      = PM_GPIO_DIR_OUT,
				.pull           = PM_GPIO_PULL_NO,
				.vin_sel        = PM_GPIO_VIN_VPH,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
				.output_value	= 1,
			},
		},
		{
            /*UIM CLK, PMIC GPIO29 in schematic*/
			28,
			{
				.direction      = PM_GPIO_DIR_IN,
				.pull           = PM_GPIO_PULL_NO,
				.vin_sel        = PM_GPIO_VIN_S3,
				.function       = PM_GPIO_FUNC_1,
				.inv_int_pol    = 0,
			},
		},
		{
            /*UIM CLK, PMIC GPIO30 in schematic*/
			29,
			{
				.direction      = PM_GPIO_DIR_OUT,
				.pull           = PM_GPIO_PULL_NO,
				.vin_sel        = PM_GPIO_VIN_S3,
				.function       = PM_GPIO_FUNC_1,
				.inv_int_pol    = 0,
			},
		},
		{
            /*UIM Reset, PMIC GPIO32 in schematic*/
			31,
			{
				.direction      = PM_GPIO_DIR_OUT,
				.pull           = PM_GPIO_PULL_NO,
				.vin_sel        = PM_GPIO_VIN_S3,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			},
		},
		{
            /*Proximity reset, PMIC GPIO36 in schematic*/
			35,
			{
				.direction      = PM_GPIO_DIR_OUT,
				.pull           = PM_GPIO_PULL_NO,
				.vin_sel        = PM_GPIO_VIN_S3,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			},
		},
		{
            /*USB ID, PMIC GPIO37 in schematic*/
			36,
			{
				.direction      = PM_GPIO_DIR_IN,
				.pull           = PM_GPIO_PULL_NO,
				.vin_sel        = PM_GPIO_VIN_S3,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			},
		},
		{
            /*SSBI PMIC CLK, PMIC GPIO39 in schematic*/
			38,
			{
				.direction      = PM_GPIO_DIR_OUT,
				.pull           = PM_GPIO_PULL_NO,
				.vin_sel        = PM_GPIO_VIN_S3,
				.function       = PM_GPIO_FUNC_1,
				.inv_int_pol    = 0,
			},
		},
		{
            /*Proximity interrupt, PMIC GPIO40 in schematic*/
			39,
			{
				.direction      = PM_GPIO_DIR_IN,
				.pull           = PM_GPIO_PULL_NO,
				.vin_sel        = PM_GPIO_VIN_S3,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			},
		},

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


static struct resource resources_pm8058_charger[] = {
	{	.name = "CHGVAL",
		.start = PM8058_CHGVAL_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_CHGVAL_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{	.name = "CHGINVAL",
		.start = PM8058_CHGINVAL_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_CHGINVAL_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "CHGILIM",
		.start = PM8058_CHGILIM_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_CHGILIM_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "VCP",
		.start = PM8058_VCP_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_VCP_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
		{
		.name = "ATC_DONE",
		.start = PM8058_ATC_DONE_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_ATC_DONE_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "ATCFAIL",
		.start = PM8058_ATCFAIL_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_ATCFAIL_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "AUTO_CHGDONE",
		 .start = PM8058_AUTO_CHGDONE_IRQ(PM8058_IRQ_BASE),
		 .end = PM8058_AUTO_CHGDONE_IRQ(PM8058_IRQ_BASE),
		 .flags = IORESOURCE_IRQ,
	},
	{
		.name = "AUTO_CHGFAIL",
		.start = PM8058_AUTO_CHGFAIL_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_AUTO_CHGFAIL_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "CHGSTATE",
		.start = PM8058_CHGSTATE_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_CHGSTATE_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "FASTCHG",
		.start = PM8058_FASTCHG_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_FASTCHG_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "CHG_END",
		 .start = PM8058_CHG_END_IRQ(PM8058_IRQ_BASE),
		 .end = PM8058_CHG_END_IRQ(PM8058_IRQ_BASE),
		 .flags = IORESOURCE_IRQ,
	},
	{
		.name = "BATTTEMP",
		.start = PM8058_BATTTEMP_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_BATTTEMP_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "CHGHOT",
		.start = PM8058_CHGHOT_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_CHGHOT_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "CHGTLIMIT",
		.start = PM8058_CHGTLIMIT_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_CHGTLIMIT_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "CHG_GONE",
		 .start = PM8058_CHG_GONE_IRQ(PM8058_IRQ_BASE),
		 .end = PM8058_CHG_GONE_IRQ(PM8058_IRQ_BASE),
		 .flags = IORESOURCE_IRQ,
	},
	{
		.name = "VCPMAJOR",
		 .start = PM8058_VCPMAJOR_IRQ(PM8058_IRQ_BASE),
		 .end = PM8058_VCPMAJOR_IRQ(PM8058_IRQ_BASE),
		 .flags = IORESOURCE_IRQ,
	},
	{
		.name = "VBATDET",
		 .start = PM8058_VBATDET_IRQ(PM8058_IRQ_BASE),
		 .end = PM8058_VBATDET_IRQ(PM8058_IRQ_BASE),
		 .flags = IORESOURCE_IRQ,
	},
	{
		.name = "BATFET",
		 .start = PM8058_BATFET_IRQ(PM8058_IRQ_BASE),
		 .end = PM8058_BATFET_IRQ(PM8058_IRQ_BASE),
		 .flags = IORESOURCE_IRQ,
	},
	{
		.name = "BATT_REPLACE",
		.start = PM8058_BATT_REPLACE_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_BATT_REPLACE_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "BATTCONNECT",
		.start = PM8058_BATTCONNECT_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_BATTCONNECT_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "VBATDET_LOW",
		.start = PM8058_VBATDET_LOW_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_VBATDET_LOW_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
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
	int id, mode, max_mA;

	id = mode = max_mA = 0;
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
		break;

	case 6:
		id = PM_PWM_LED_FLASH;
		mode = PM_PWM_CONF_PWM1;
		max_mA = 300;
		break;

	case 7:
		id = PM_PWM_LED_FLASH1;
		mode = PM_PWM_CONF_PWM1;
		max_mA = 300;
		break;

	default:
		break;
	}

	if (ch >= 6 && ch <= 7) {
		if (!on) {
			mode = PM_PWM_CONF_NONE;
			max_mA = 0;
		}
		rc = pm8058_pwm_config_led(pwm, id, mode, max_mA);
		if (rc)
			pr_err("%s: pm8058_pwm_config_led(ch=%d): rc=%d\n",
			       __func__, ch, rc);
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

static struct resource resources_temp_alarm[] = {
       {
		.start  = PM8058_TEMP_ALARM_IRQ(PM8058_IRQ_BASE),
		.end    = PM8058_TEMP_ALARM_IRQ(PM8058_IRQ_BASE),
		.flags  = IORESOURCE_IRQ,
       },
};

static struct mfd_cell pm8058_subdevs[] = {
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
		.name = "pm8058-rtc",
		.id = -1,
		.num_resources  = ARRAY_SIZE(resources_rtc),
		.resources      = resources_rtc,
	},
	{
		.name = "pm8058-tm",
		.id = -1,
		.num_resources  = ARRAY_SIZE(resources_temp_alarm),
		.resources      = resources_temp_alarm,
	},
	{	.name = "pm8058-upl",
		.id		= -1,
	},
	{
		.name = "pm8058-charger",
		.id = -1,
		.num_resources = ARRAY_SIZE(resources_pm8058_charger),
		.resources = resources_pm8058_charger,
	},
};

static struct pm8058_platform_data pm8058_platform_data = {
	.irq_base = PM8058_IRQ_BASE,

	.num_subdevs = ARRAY_SIZE(pm8058_subdevs),
	.sub_devices = pm8058_subdevs,
	.irq_trigger_flags = IRQF_TRIGGER_LOW,
};

static struct i2c_board_info pm8058_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("pm8058-core", 0x55),
		.irq = MSM_GPIO_TO_INT(PM8058_GPIO_INT),
		.platform_data = &pm8058_platform_data,
	},
};
#endif /* CONFIG_PMIC8058 */

#ifdef CONFIG_PMIC8901

#define PM8901_GPIO_INT           91

static struct pm8901_gpio_platform_data pm8901_mpp_data = {
	.gpio_base	= PM8901_GPIO_PM_TO_SYS(0),
	.irq_base	= PM8901_MPP_IRQ(PM8901_IRQ_BASE, 0),
};

static struct resource pm8901_temp_alarm[] = {
	{
		.start = PM8901_TEMP_ALARM_IRQ(PM8901_IRQ_BASE),
		.end = PM8901_TEMP_ALARM_IRQ(PM8901_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = PM8901_TEMP_HI_ALARM_IRQ(PM8901_IRQ_BASE),
		.end = PM8901_TEMP_HI_ALARM_IRQ(PM8901_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
};

static struct regulator_consumer_supply pm8901_vreg_supply[PM8901_VREG_MAX] = {
	[PM8901_VREG_ID_MPP0] =     REGULATOR_SUPPLY("8901_mpp0",     NULL),
	[PM8901_VREG_ID_USB_OTG]  = REGULATOR_SUPPLY("8901_usb_otg",  NULL),
	[PM8901_VREG_ID_HDMI_MVS] = REGULATOR_SUPPLY("8901_hdmi_mvs", NULL),
};

#define PM8901_VREG_INIT(_id, _min_uV, _max_uV, _modes, _ops, _apply_uV, \
			 _always_on, _active_high) \
	[_id] = { \
		.init_data = { \
			.constraints = { \
				.valid_modes_mask = _modes, \
				.valid_ops_mask = _ops, \
				.min_uV = _min_uV, \
				.max_uV = _max_uV, \
				.input_uV = _min_uV, \
				.apply_uV = _apply_uV, \
				.always_on = _always_on, \
			}, \
			.num_consumer_supplies = 1, \
			.consumer_supplies = &pm8901_vreg_supply[_id], \
		}, \
		.active_high = _active_high, \
	}

#define PM8901_VREG_INIT_MPP(_id, _active_high) \
	PM8901_VREG_INIT(_id, 0, 0, REGULATOR_MODE_NORMAL, \
			REGULATOR_CHANGE_STATUS, 0, 0, _active_high)

#define PM8901_VREG_INIT_VS(_id) \
	PM8901_VREG_INIT(_id, 0, 0, REGULATOR_MODE_NORMAL, \
			REGULATOR_CHANGE_STATUS, 0, 0, 0)

static struct pm8901_vreg_pdata pm8901_vreg_init_pdata[PM8901_VREG_MAX] = {
	PM8901_VREG_INIT_MPP(PM8901_VREG_ID_MPP0, 1),

	PM8901_VREG_INIT_VS(PM8901_VREG_ID_USB_OTG),
	PM8901_VREG_INIT_VS(PM8901_VREG_ID_HDMI_MVS),
};

#define PM8901_VREG(_id) { \
	.name = "pm8901-regulator", \
	.id = _id, \
	.platform_data = &pm8901_vreg_init_pdata[_id], \
	.data_size = sizeof(pm8901_vreg_init_pdata[_id]), \
}

static struct mfd_cell pm8901_subdevs[] = {
	{	.name = "pm8901-mpp",
		.id		= -1,
		.platform_data	= &pm8901_mpp_data,
		.data_size	= sizeof(pm8901_mpp_data),
	},
	{	.name = "pm8901-tm",
		.id		= -1,
		.num_resources  = ARRAY_SIZE(pm8901_temp_alarm),
		.resources      = pm8901_temp_alarm,
	},
	PM8901_VREG(PM8901_VREG_ID_MPP0),
	PM8901_VREG(PM8901_VREG_ID_USB_OTG),
	PM8901_VREG(PM8901_VREG_ID_HDMI_MVS),
};

static struct pm8901_platform_data pm8901_platform_data = {
	.irq_base = PM8901_IRQ_BASE,
	.num_subdevs = ARRAY_SIZE(pm8901_subdevs),
	.sub_devices = pm8901_subdevs,
	.irq_trigger_flags = IRQF_TRIGGER_LOW,
};

static struct i2c_board_info pm8901_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("pm8901-core", 0x55),
		.irq = MSM_GPIO_TO_INT(PM8901_GPIO_INT),
		.platform_data = &pm8901_platform_data,
	},
};

#endif /* CONFIG_PMIC8901 */

#ifdef CONFIG_MFD_WM8994

#define WM8958_I2C_SLAVE_ADDR 0x1a
static struct regulator *vreg_wm8958;

static int wm8994_ldo_power(int enable)
{
	gpio_tlmm_config(GPIO_CFG(66, 0, GPIO_CFG_OUTPUT,GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(108, 0, GPIO_CFG_OUTPUT,GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	if (enable)
	{
		pr_err("%s: Power up the WM8994 LDOs\n", __func__);
		gpio_set_value_cansleep(66,1);
		gpio_set_value_cansleep(108,1);
	}else{
		pr_err("%s: Power down the WM8994 LDOs\n", __func__);
		gpio_set_value_cansleep(66,0);
		gpio_set_value_cansleep(108,0);
	}

	return 0;

}

static unsigned int msm_wm8958_setup_power(void)
{
	static struct regulator *tp_5v0 = NULL;
	int rc=0;

	pr_err("%s: codec power setup\n", __func__);

	if (!tp_5v0) {
		tp_5v0 = regulator_get(NULL, "vdd50_boost");
		if (IS_ERR(tp_5v0)) {
			pr_err("failed to get regulator 'vdd50_boost' with %ld\n",
				PTR_ERR(tp_5v0));
			tp_5v0 = NULL;
		}
	}

	if (tp_5v0) {
		rc = regulator_enable(tp_5v0);
		if (rc < 0) {
			pr_err("failed to enable regulator 'vdd50_boost' with %d\n", rc);
		}
	}

	vreg_wm8958 = regulator_get(NULL, "8058_s3");
	if (IS_ERR(vreg_wm8958)) {
		pr_err("%s: Unable to get 8058_s3\n", __func__);
		return -ENODEV;
	}

	if(regulator_set_voltage(vreg_wm8958, 1800000, 1800000)){
		pr_err("%s: Unable to set regulator voltage: votg_8058_s3\n", __func__);
		return -ENODEV;
	}

	rc = regulator_enable(vreg_wm8958);

	if (rc){
		pr_err("%s: Enable regulator 8058_s3 failed\n", __func__);
	}
	wm8994_ldo_power(1);
	mdelay(30);
	return rc;
}

static void msm_wm8958_shutdown_power(void)
{
	static struct regulator *tp_5v0 = NULL;
	int rc;

	pr_err("%s: codec power shutdown\n", __func__);

	if (!tp_5v0) {
		tp_5v0 = regulator_get(NULL, "vdd50_boost");
		if (IS_ERR(tp_5v0)) {
			pr_err("failed to get regulator 'vdd50_boost' with %ld\n",
				PTR_ERR(tp_5v0));
			tp_5v0 = NULL;
		}
	}

	if (tp_5v0) {
		rc = regulator_disable(tp_5v0);
		if (rc < 0) {
			pr_err("failed to disable regulator 'vdd50_boost' with %d\n", rc);
		}
	}

	wm8994_ldo_power(0);

	rc = regulator_disable(vreg_wm8958);

	if (rc)
		pr_err("%s: Disable regulator 8058_s3 failed\n", __func__);

	regulator_put(vreg_wm8958);
}

static struct wm8994_pdata wm8958_pdata = {
	.gpio_defaults = { 	0x0001, //GPIO1 AUDIO_AMP_EN write 0x41 to enable AMP
						0x8001, //GPIO2 MCLK2 Pull Control
						0x8001, //GPIO3 BCLK2 Pull Control
						0x8001, //GPIO4 DACLRCLK2 Pull Control
						0x8001, //GPIO5 DACDAT2 Pull Control
						0x2005, //GPIO6 HEAD_MIC_DET to GPIO57 8660
						0xa101, //GPIO7 N/A on wm8958, only wm8994
						0xa101, //GPIO8 0xa101 not use
						0xa101, //GPIO9 0xa101 not use
						0xa101, //GPIO10 0xa101 not use
						0xa101, //GPIO11 0xa101 not use
	},

	/* Put the line outputs into differential mode so that the driver
	 * knows it can power the chip down to cold without pop/click issues.
	 * Line outputs are not actually connected on the board.
	 */
	.num_enh_eq_cfgs = 1,
	.lineout1_diff = 1,
	.lineout2_diff = 1,
	.wm8994_setup = msm_wm8958_setup_power,
	.wm8994_shutdown = msm_wm8958_shutdown_power,
	.micdet_irq = TENDERLOIN_AUD_HEAD_MIC_DET_IRQ_GPIO,
	.micbias = { 0x0029, 0x002d, },
};

static struct i2c_board_info audio_boardinfo[] __initdata = {
    {
        I2C_BOARD_INFO("wm8958", WM8958_I2C_SLAVE_ADDR),
        .platform_data = &wm8958_pdata,
    },
};
#endif /* CONFIG_MFD_WM8994 */

#ifdef CONFIG_I2C
#define I2C_TENDERLOIN (1 << 0)

struct i2c_registry {
	u8                     machs;
	int                    bus;
	struct i2c_board_info *info;
	int                    len;
};

static struct i2c_registry msm8x60_i2c_devices[] __initdata = {
#ifdef CONFIG_INPUT_LSM303DLH
    {
        I2C_TENDERLOIN,
        MSM_GSBI3_QUP_I2C_BUS_ID,
        &lsm303dlh_acc_i2c_board_info,
        ARRAY_SIZE(lsm303dlh_acc_i2c_board_info),
    },
    {
        I2C_TENDERLOIN,
        MSM_GSBI3_QUP_I2C_BUS_ID,
        &lsm303dlh_mag_i2c_board_info,
        ARRAY_SIZE(lsm303dlh_mag_i2c_board_info),
    },
#endif
    {
        I2C_TENDERLOIN,
        MSM_GSBI3_QUP_I2C_BUS_ID,
        &mpu3050_i2c_board_info,
        ARRAY_SIZE(mpu3050_i2c_board_info),
    },
    {
        I2C_TENDERLOIN,
        MSM_GSBI3_QUP_I2C_BUS_ID,
        &isl29023_i2c_board_info,
        ARRAY_SIZE(isl29023_i2c_board_info),
    },
#ifdef CONFIG_PMIC8058
	{
		I2C_TENDERLOIN,
		MSM_SSBI1_I2C_BUS_ID,
		pm8058_boardinfo,
		ARRAY_SIZE(pm8058_boardinfo),
	},
#endif
#ifdef CONFIG_PMIC8901
	{
		I2C_TENDERLOIN,
		MSM_SSBI2_I2C_BUS_ID,
		pm8901_boardinfo,
		ARRAY_SIZE(pm8901_boardinfo),
	},
#endif
#ifdef CONFIG_MSM_CAMERA
	{
		I2C_TENDERLOIN,
		MSM_GSBI4_QUP_I2C_BUS_ID,
		msm_camera_boardinfo,
		ARRAY_SIZE(msm_camera_boardinfo),
	},
#endif
#ifdef CONFIG_MFD_WM8994
	{
		I2C_TENDERLOIN,
		MSM_GSBI7_QUP_I2C_BUS_ID,
		audio_boardinfo,
		ARRAY_SIZE(audio_boardinfo),
	},
#endif
	{
		I2C_TENDERLOIN,
		MSM_GSBI10_QUP_I2C_BUS_ID,   // use GSBI10 as touch i2c
		xMT1386_board_info,
		ARRAY_SIZE(xMT1386_board_info),
	},
#ifdef CONFIG_LEDS_LM8502

	{
		I2C_TENDERLOIN,
		MSM_GSBI8_QUP_I2C_BUS_ID,   // use GSBI8 as LM8502 i2c
		lm8502_board_info,
		ARRAY_SIZE(lm8502_board_info),
	},
#endif
};
#endif /* CONFIG_I2C */

static void fixup_i2c_configs(void)
{
#ifdef CONFIG_I2C
	/*
	 * Set PMIC 8901 MPP0 active_high to 0 for surf and charm_surf. This
	 * implies that the regulator connected to MPP0 is enabled when
	 * MPP0 is low.
	 */
	if (machine_is_tenderloin())
		pm8901_vreg_init_pdata[PM8901_VREG_ID_MPP0].active_high = 0;
	else
		pm8901_vreg_init_pdata[PM8901_VREG_ID_MPP0].active_high = 1;

#ifdef CONFIG_INPUT_LSM303DLH
	if (machine_is_tenderloin() && boardtype_is_3g()) {
		lsm303dlh_acc_pdata.negate_y = 1;
		lsm303dlh_acc_pdata.negate_z = 1;
	}
#endif
#endif
}

static void __init register_i2c_devices(void)
{
#ifdef CONFIG_I2C
	u8 mach_mask = 0;
	int i;
    int rc;

	/* Build the matching 'supported_machs' bitmask */
	if (machine_is_tenderloin())
		mach_mask = I2C_TENDERLOIN;
	else
		pr_err("unmatched machine ID in register_i2c_devices\n");

    gpio_request(TENDERLOIN_LS_INT, "isl29023 int");
    rc = gpio_direction_input(TENDERLOIN_LS_INT);
	if (rc) {
		pr_err("%s:Failed to configure GPIO %d\n",
				__func__, TENDERLOIN_LS_INT);
	}
    /* Run the array and install devices as appropriate */
	for (i = 0; i < ARRAY_SIZE(msm8x60_i2c_devices); ++i) {
		if (msm8x60_i2c_devices[i].machs & mach_mask)
			i2c_register_board_info(msm8x60_i2c_devices[i].bus,
						msm8x60_i2c_devices[i].info,
						msm8x60_i2c_devices[i].len);
	}

#endif
}

static void __init msm8x60_init_buses(void)
{
#ifdef CONFIG_I2C_QUP
	msm_gsbi3_qup_i2c_device.dev.platform_data = &msm_gsbi3_qup_i2c_pdata;
	msm_gsbi4_qup_i2c_device.dev.platform_data = &msm_gsbi4_qup_i2c_pdata;
	msm_gsbi7_qup_i2c_device.dev.platform_data = &msm_gsbi7_qup_i2c_pdata;
	msm_gsbi8_qup_i2c_device.dev.platform_data = &msm_gsbi8_qup_i2c_pdata;
	msm_gsbi9_qup_i2c_device.dev.platform_data = &msm_gsbi9_qup_i2c_pdata;
    msm_gsbi10_qup_i2c_device.dev.platform_data = &msm_gsbi10_qup_i2c_pdata;
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

#ifdef CONFIG_USB_GADGET_MSM_72K
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
#endif

#ifdef CONFIG_SERIAL_MSM_HS
	msm_uart_dm1_pdata.wakeup_irq = gpio_to_irq(54); /* GSBI6(2) */
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#endif
#ifdef CONFIG_MSM_BUS_SCALING

	/* RPM calls are only enabled on V2 */
	if (SOCINFO_VERSION_MAJOR(socinfo_get_version()) == 2) {
		msm_bus_apps_fabric_pdata.rpm_enabled = 1;
		msm_bus_sys_fabric_pdata.rpm_enabled = 1;
		msm_bus_mm_fabric_pdata.rpm_enabled = 1;
		msm_bus_sys_fpb_pdata.rpm_enabled = 1;
		msm_bus_cpss_fpb_pdata.rpm_enabled = 1;
	}

	msm_bus_apps_fabric.dev.platform_data = &msm_bus_apps_fabric_pdata;
	msm_bus_sys_fabric.dev.platform_data = &msm_bus_sys_fabric_pdata;
	msm_bus_mm_fabric.dev.platform_data = &msm_bus_mm_fabric_pdata;
	msm_bus_sys_fpb.dev.platform_data = &msm_bus_sys_fpb_pdata;
	msm_bus_cpss_fpb.dev.platform_data = &msm_bus_cpss_fpb_pdata;
#endif
}

static void __init msm8x60_map_io(void)
{
	msm_shared_ram_phys = MSM_SHARED_RAM_PHYS;
	msm_map_msm8x60_io();
	msm8x60_allocate_memory_regions();
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

		if (machine_is_msm8x60_surf() || machine_is_msm8x60_ffa() || machine_is_tenderloin())
			ebi2_cfg |= (1 << 4) | (1 << 5); /* CS2, CS3 */
		else if (machine_is_msm8x60_sim())
			ebi2_cfg |= (1 << 4); /* CS2 */
		else if (machine_is_msm8x60_rumi3())
			ebi2_cfg |= (1 << 5); /* CS3 */

		writel(ebi2_cfg, ebi2_cfg_ptr);
		iounmap(ebi2_cfg_ptr);
	}

	if (machine_is_tenderloin()) {
		ebi2_cfg_ptr = ioremap_nocache(0x1a110000, SZ_4K);
		if (ebi2_cfg_ptr != 0) {
			/* EBI2_XMEM_CFG:PWRSAVE_MODE off */
			writel(0UL, ebi2_cfg_ptr);

			/* CS2: Delay 9 cycles (140ns@64MHz) between SMSC
			 * LAN9221 Ethernet controller reads and writes.
			 * The lowest 4 bits are the read delay, the next
			 * 4 are the write delay. */
			writel(0x031F1C99, ebi2_cfg_ptr + 0x10);
#ifdef CONFIG_USB_PEHCI_HCD
			if(boardtype_is_3g()) {
				/*
				* RECOVERY=5, HOLD_WR=1
				* INIT_LATENCY_WR=1, INIT_LATENCY_RD=1
				* WAIT_WR=1, WAIT_RD=2
				*/
				writel(0x51010112, ebi2_cfg_ptr + 0x14);
				/*
				* HOLD_RD=1
				* ADV_OE_RECOVERY=0, ADDR_HOLD_ENA=1
				*/
				writel(0x01000020, ebi2_cfg_ptr + 0x34);
			}
			else {
				/* EBI2 CS3 muxed address/data,
				* two cyc addr enable */
				writel(0xA3030020, ebi2_cfg_ptr + 0x34);
			}
#else
			/* EBI2 CS3 muxed address/data,
			* two cyc addr enable */
			writel(0xA3030020, ebi2_cfg_ptr + 0x34);

#endif
			iounmap(ebi2_cfg_ptr);
		}
	}
}

struct msm8x60_tlmm_cfg_struct {
	unsigned gpio;
	u32      flags;
};

static uint32_t tenderloin_tlmm_cfgs[] = {

	/* 5V Bias Enable */
	GPIO_CFG(102, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA),

	/* Touch reset */
	GPIO_CFG(MXT1386_TS_PWR_RST_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
};

static void __init msm8x60_init_tlmm(void)
{
	unsigned n;

	for (n = 0; n < ARRAY_SIZE(tenderloin_tlmm_cfgs); ++n)
		gpio_tlmm_config(tenderloin_tlmm_cfgs[n], 0);
}

/*WLAN*/

#define WLAN_1V8_SDIO_ACT_LOAD  80000
#define WLAN_1V8_SDIO_MIN_LOAD  10000

static int tenderloin_wifi_power(int on)
{
	static struct regulator * wifi_S3A_1V8  = NULL;
	static struct regulator * wifi_L1B_3V3  = NULL;
	static struct regulator * wifi_L3B_3V3  = NULL;
	static struct regulator * wifi_L19A_1V8 = NULL;
	static int wifi_is_on = 0;
	static int rc = -EINVAL; /* remember if the gpio_requests succeeded */
	int gpios[] = {
			TENDERLOIN_GPIO_WL_HOST_WAKE,
			pin_table[TENDERLOIN_GPIO_HOST_WAKE_WL_PIN],
			pin_table[TENDERLOIN_GPIO_WLAN_RST_N_PIN]
		};

	// "8901_l3"  - 3.3V - VREG_L3B_3V3      - WLAN_PA_3V3
	// "8901_l1"  - 3.3V - VREG_L1B_3V3      - VDD_WLAN_3V3
	// "8058_l19" - 1.8V - VREG_L19A_1V8     - VDD_1.8
	// "8058_s3"  - 1.8V - VREG_S3A_1V8_WLAN - DVDD_SDIO_1V8 etc.

	if (!wifi_S3A_1V8) {
		// Always on
		wifi_S3A_1V8 = regulator_get(NULL, "8058_s3");
		if (IS_ERR(wifi_S3A_1V8)) {
			pr_err("%s: unable to get %s\n",
				__func__, "8058_s3");
		}
		if(regulator_set_voltage(wifi_S3A_1V8, 1800000, 1800000)){
			pr_err("%s: unable to set voltage for %s\n",
				__func__, "8058_s3");
		}
		if (regulator_enable(wifi_S3A_1V8)){
			pr_err("%s: Unable to enable %s\n",
				 __func__, "8058_s3");
		}
	}

	if (!wifi_L3B_3V3) {
		wifi_L3B_3V3 = regulator_get(NULL, "8901_l3");
		if (IS_ERR(wifi_L3B_3V3)) {
			pr_err("%s: unable to get %s\n",
				__func__, "8901_l3");
		}
		if (regulator_set_voltage(wifi_L3B_3V3, 3300000, 3300000)) {
			pr_err("%s: unable to set voltage for %s\n",
				__func__, "8901_l3");
		}
	}

	if (!wifi_L1B_3V3) {
		wifi_L1B_3V3 = regulator_get(NULL, "8901_l1");
		if (IS_ERR(wifi_L1B_3V3)) {
			pr_err("%s: unable to get %s\n",
				__func__, "8901_l1");
		}
		if (regulator_set_voltage(wifi_L1B_3V3, 3300000, 3300000)) {
			pr_err("%s: unable to set voltage for %s\n",
				__func__, "8901_l3");
		}
	}

	if (!wifi_L19A_1V8) {
		wifi_L19A_1V8 = regulator_get(NULL, "8058_l19");
		if (IS_ERR(wifi_L19A_1V8)) {
			pr_err("%s: unable to get %s\n",
				__func__, "8058_l19");
		}
		if (regulator_set_voltage(wifi_L19A_1V8, 1800000, 1800000)) {
			pr_err("%s: unable to set voltage for %s\n",
				__func__, "8058_l19");
		}
	}

	BUG_ON(!wifi_L3B_3V3);
	BUG_ON(!wifi_L1B_3V3);
	BUG_ON(!wifi_L19A_1V8);
	BUG_ON(!wifi_S3A_1V8);

	if (on) {
		//
		// B. enable power
		// 3.3V -> 1.8V -> wait (Tb 5) -> CHIP_PWD

		pr_info("wifi_power(%d) 1.8V sdio: set load\n", on);
		rc = regulator_set_optimum_mode(wifi_S3A_1V8,
			WLAN_1V8_SDIO_ACT_LOAD);
		if (rc < 0) {
			pr_err("%s: Unable (%d) to set opt load for %s\n",
				 __func__, rc, "8058_s3");
		} else {
			pr_info("%s: New regulator mode for %s: %d\n",
				 __func__, "8058_s3", rc );
		}

		pr_info("wifi_power(%d) 3.3V\n", on);
		rc = regulator_enable(wifi_L3B_3V3);
		if (rc) {
			pr_err("%s: Unable (%d) to enable %s\n",
				 __func__, rc, "8901_l3");
		}

		pr_info("wifi_power(%d) 8901_l1 3.3V\n", on);
		rc = regulator_enable(wifi_L1B_3V3);
		if (rc) {
			pr_err("%s: Unable (%d) to enable %s\n",
				 __func__, rc, "8901_l1");
		}

		pr_info("wifi_power(%d) 8058_l19 1.8V\n", on);
		rc = regulator_enable(wifi_L19A_1V8);
		if (rc) {
			pr_err("%s: Unable (%d) to enable %s\n",
				 __func__, rc, "8058_l19");
		}

		printk("wifi_power(%d) CHIP_PWD\n", on);
		mdelay(5);

		/* request gpio if not yet done */
		//printk("wifi_is_on(%d)\n", wifi_is_on);
		if (!wifi_is_on) {
			rc = configure_gpios(on, gpios, ARRAY_SIZE(gpios));
			if (rc) {
				pr_err("%s: gpio request failed\n", __func__);
				return -EINVAL;
			}
		}

		gpio_direction_output(pin_table[TENDERLOIN_GPIO_WLAN_RST_N_PIN], 0);
		mdelay(5);
		gpio_direction_output(pin_table[TENDERLOIN_GPIO_WLAN_RST_N_PIN], 1);
	}
	else
	{
		//CHIP_PWD -> wait (Tc 5)
		gpio_direction_output(pin_table[TENDERLOIN_GPIO_WLAN_RST_N_PIN], 0);
		mdelay(5);

		if (wifi_is_on) {
			rc = configure_gpios(on, gpios, ARRAY_SIZE(gpios));
		}

		rc = regulator_disable(wifi_L3B_3V3);
		if (rc) {
			pr_err("%s: Unable (%d) to disable %s\n",
				__func__, rc, "8901_l3");
		}

		rc = regulator_disable(wifi_L1B_3V3);
		if (rc) {
			pr_err("%s: Unable (%d) to disable %s\n",
				__func__, rc, "8901_l1");
		}

		rc = regulator_disable(wifi_L19A_1V8);
		if (rc) {
			pr_err("%s: Unable (%d) to disable %s\n",
				__func__, rc, "8058_l19");
		}

		rc = regulator_set_optimum_mode(wifi_S3A_1V8,
			WLAN_1V8_SDIO_MIN_LOAD);
		if (rc < 0) {
			pr_err("%s: Unable (%d) to set opt load for %s\n",
				 __func__, rc, "8058_s3");
		} else {
			pr_info("%s: New regulator mode for %s: %d\n",
				 __func__, "8058_s3", rc );
		}
	}

	wifi_is_on = on;
	return 0;
}

static struct mmc_host *wifi_mmc;
int board_sdio_wifi_enable(unsigned int param);
int board_sdio_wifi_disable(unsigned int param);

static void tenderloin_probe_wifi(int id, struct mmc_host *mmc)
{
	printk("%s: id %d mmc %p\n", __PRETTY_FUNCTION__, id, mmc);
	wifi_mmc = mmc;

       //TODO: hook up to PM later
//       board_sdio_wifi_enable(0);
}

static void tenderloin_remove_wifi(int id, struct mmc_host *mmc)
{
	printk("%s: id %d mmc %p\n", __PRETTY_FUNCTION__, id, mmc);
	wifi_mmc = NULL;

       //TODO: hook up to PM later
//       board_sdio_wifi_disable(0);
}

/*
 *  An API to enable wifi
 */
int board_sdio_wifi_enable(unsigned int param)
{
	printk(KERN_ERR "board_sdio_wifi_enable\n");

	tenderloin_wifi_power(1);
	if (wifi_mmc) {
		mmc_detect_change(wifi_mmc, msecs_to_jiffies(250));
	}

	return 0;
}
EXPORT_SYMBOL(board_sdio_wifi_enable);

/*
 *  An API to disable wifi
 */
int board_sdio_wifi_disable(unsigned int param)
{
	printk(KERN_ERR "board_sdio_wifi_disable\n");

	tenderloin_wifi_power(0);

	if (wifi_mmc) {
		mmc_detect_change(wifi_mmc, msecs_to_jiffies(100));
	}

	return 0;
}
EXPORT_SYMBOL(board_sdio_wifi_disable);

int board_get_wow_pin()
{
	return pin_table[TENDERLOIN_GPIO_HOST_WAKE_WL_PIN];
}
EXPORT_SYMBOL(board_get_wow_pin);

#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC5_SUPPORT))

/* 8x60 is having 5 SDCC controllers */
#define MAX_SDCC_CONTROLLER	5

struct msm_sdcc_gpio {
	/* maximum 10 GPIOs per SDCC controller */
	s16 no;
	/* name of this GPIO */
	const char *name;
};

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static struct msm_sdcc_gpio sdc1_gpio_cfg[] = {
	{159, "sdc1_dat_0"},
	{160, "sdc1_dat_1"},
	{161, "sdc1_dat_2"},
	{162, "sdc1_dat_3"},
#ifdef CONFIG_MMC_MSM_SDC1_8_BIT_SUPPORT
	{163, "sdc1_dat_4"},
	{164, "sdc1_dat_5"},
	{165, "sdc1_dat_6"},
	{166, "sdc1_dat_7"},
#endif
	{167, "sdc1_clk"},
	{168, "sdc1_cmd"}
};
#endif


struct msm_sdcc_pad_pull_cfg {
	enum msm_tlmm_pull_tgt pull;
	u32 pull_val;
};

struct msm_sdcc_pad_drv_cfg {
	enum msm_tlmm_hdrive_tgt drv;
	u32 drv_val;
};


#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static struct msm_sdcc_pad_drv_cfg sdc4_pad_on_drv_cfg[] = {
	{TLMM_HDRV_SDC4_CLK, GPIO_CFG_8MA},
	{TLMM_HDRV_SDC4_CMD, GPIO_CFG_8MA},
	{TLMM_HDRV_SDC4_DATA, GPIO_CFG_8MA}
};

static struct msm_sdcc_pad_pull_cfg sdc4_pad_on_pull_cfg[] = {
	{TLMM_PULL_SDC4_CMD, GPIO_CFG_PULL_UP},
	{TLMM_PULL_SDC4_DATA, GPIO_CFG_PULL_UP}
};

static struct msm_sdcc_pad_drv_cfg sdc4_pad_off_drv_cfg[] = {
	{TLMM_HDRV_SDC4_CLK, GPIO_CFG_8MA},
	{TLMM_HDRV_SDC4_CMD, GPIO_CFG_8MA},
	{TLMM_HDRV_SDC4_DATA, GPIO_CFG_8MA}
};

static struct msm_sdcc_pad_pull_cfg sdc4_pad_off_pull_cfg[] = {
	{TLMM_PULL_SDC4_CMD, GPIO_CFG_PULL_UP},
	{TLMM_PULL_SDC4_DATA, GPIO_CFG_PULL_UP}
};
#endif

struct msm_sdcc_pin_cfg {
	/*
	 * = 1 if controller pins are using gpios
	 * = 0 if controller has dedicated MSM pins
	 */
	u8 is_gpio;
	u8 cfg_sts;
	u8 gpio_data_size;
	struct msm_sdcc_gpio *gpio_data;
	struct msm_sdcc_pad_drv_cfg *pad_drv_on_data;
	struct msm_sdcc_pad_drv_cfg *pad_drv_off_data;
	struct msm_sdcc_pad_pull_cfg *pad_pull_on_data;
	struct msm_sdcc_pad_pull_cfg *pad_pull_off_data;
	u8 pad_drv_data_size;
	u8 pad_pull_data_size;
};


static struct msm_sdcc_pin_cfg sdcc_pin_cfg_data[5] = {
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	[0] = {
		.is_gpio = 1,
		.gpio_data_size = ARRAY_SIZE(sdc1_gpio_cfg),
		.gpio_data = sdc1_gpio_cfg
	},
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	[3] = {
		.is_gpio = 0,
		.pad_drv_on_data = sdc4_pad_on_drv_cfg,
		.pad_drv_off_data = sdc4_pad_off_drv_cfg,
		.pad_pull_on_data = sdc4_pad_on_pull_cfg,
		.pad_pull_off_data = sdc4_pad_off_pull_cfg,
		.pad_drv_data_size = ARRAY_SIZE(sdc4_pad_on_drv_cfg),
		.pad_pull_data_size = ARRAY_SIZE(sdc4_pad_on_pull_cfg)
	},
#endif
};

static int msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct msm_sdcc_pin_cfg *curr;
	int n;

	curr = &sdcc_pin_cfg_data[dev_id - 1];
	if (!curr->gpio_data)
		goto out;

	for (n = 0; n < curr->gpio_data_size; n++) {
		if (enable) {
			rc = gpio_request(curr->gpio_data[n].no,
				curr->gpio_data[n].name);
			if (rc) {
				pr_err("%s: gpio_request(%d, %s)"
					"failed", __func__,
					curr->gpio_data[n].no,
					curr->gpio_data[n].name);
				goto free_gpios;
			}
			/* set direction as output for all GPIOs */
			rc = gpio_direction_output(
				curr->gpio_data[n].no, 1);
			if (rc) {
				pr_err("%s: gpio_direction_output"
					"(%d, 1) failed\n", __func__,
					curr->gpio_data[n].no);
				goto free_gpios;
			}
		} else {
			/*
			 * now free this GPIO which will put GPIO
			 * in low power mode and will also put GPIO
			 * in input mode
			 */
			gpio_free(curr->gpio_data[n].no);
		}
	}
	curr->cfg_sts = enable;
	goto out;

free_gpios:
	for (; n >= 0; n--)
		gpio_free(curr->gpio_data[n].no);
out:
	return rc;
}

static int msm_sdcc_setup_pad(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct msm_sdcc_pin_cfg *curr;
	int n;

	curr = &sdcc_pin_cfg_data[dev_id - 1];
	if (!curr->pad_drv_on_data || !curr->pad_pull_on_data)
		goto out;

	if (enable) {
		/*
		 * set up the normal driver strength and
		 * pull config for pads
		 */
		for (n = 0; n < curr->pad_drv_data_size; n++)
			msm_tlmm_set_hdrive(curr->pad_drv_on_data[n].drv,
				curr->pad_drv_on_data[n].drv_val);
		for (n = 0; n < curr->pad_pull_data_size; n++)
			msm_tlmm_set_pull(curr->pad_pull_on_data[n].pull,
				curr->pad_pull_on_data[n].pull_val);
	} else {
		/* set the low power config for pads */
		for (n = 0; n < curr->pad_drv_data_size; n++)
			msm_tlmm_set_hdrive(
				curr->pad_drv_off_data[n].drv,
				curr->pad_drv_off_data[n].drv_val);
		for (n = 0; n < curr->pad_pull_data_size; n++)
			msm_tlmm_set_pull(
				curr->pad_pull_off_data[n].pull,
				curr->pad_pull_off_data[n].pull_val);
	}
	curr->cfg_sts = enable;
out:
	return rc;
}

struct sdcc_reg {
	/* VDD/VCC/VCCQ regulator name on PMIC8058/PMIC8089*/
	const char *reg_name;
	/*
	 * is set voltage supported for this regulator?
	 * 0 = not supported, 1 = supported
	 */
	unsigned char set_voltage_sup;
	/* voltage level to be set */
	unsigned int level;
	/* VDD/VCC/VCCQ voltage regulator handle */
	struct regulator *reg;
	/* is this regulator enabled? */
	bool enabled;
	/* is this regulator needs to be always on? */
	bool always_on;
	/* is operating power mode setting required for this regulator? */
	bool op_pwr_mode_sup;
	/* Load values for low power and high power mode */
	unsigned int lpm_uA;
	unsigned int hpm_uA;
};
/* all SDCC controllers requires VDD/VCC voltage */
static struct sdcc_reg sdcc_vdd_reg_data[MAX_SDCC_CONTROLLER];
/* only SDCC1 requires VCCQ voltage */
static struct sdcc_reg sdcc_vccq_reg_data[1];
/* all SDCC controllers may require voting for VDD PAD voltage */
static struct sdcc_reg sdcc_vddp_reg_data[MAX_SDCC_CONTROLLER];

struct sdcc_reg_data {
	struct sdcc_reg *vdd_data; /* keeps VDD/VCC regulator info */
	struct sdcc_reg *vccq_data; /* keeps VCCQ regulator info */
	struct sdcc_reg *vddp_data; /* keeps VDD Pad regulator info */
	unsigned char sts; /* regulator enable/disable status */
};
/* msm8x60 have 5 SDCC controllers */
static struct sdcc_reg_data sdcc_vreg_data[MAX_SDCC_CONTROLLER];

static int msm_sdcc_vreg_init_reg(struct sdcc_reg *vreg)
{
	int rc = 0;

	/* Get the regulator handle */
	vreg->reg = regulator_get(NULL, vreg->reg_name);
	if (IS_ERR(vreg->reg)) {
		rc = PTR_ERR(vreg->reg);
		pr_err("%s: regulator_get(%s) failed. rc=%d\n",
			__func__, vreg->reg_name, rc);
		goto out;
	}

	/* Set the voltage level if required */
	if (vreg->set_voltage_sup) {
		rc = regulator_set_voltage(vreg->reg, vreg->level,
					vreg->level);
		if (rc) {
			pr_err("%s: regulator_set_voltage(%s) failed rc=%d\n",
				__func__, vreg->reg_name, rc);
			goto vreg_put;
		}
	}
	goto out;

vreg_put:
	regulator_put(vreg->reg);
out:
	return rc;
}

static inline void msm_sdcc_vreg_deinit_reg(struct sdcc_reg *vreg)
{
	regulator_put(vreg->reg);
}

/* this init function should be called only once for each SDCC */
static int msm_sdcc_vreg_init(int dev_id, unsigned char init)
{
	int rc = 0;
	struct sdcc_reg *curr_vdd_reg, *curr_vccq_reg, *curr_vddp_reg;
	struct sdcc_reg_data *curr;

	curr = &sdcc_vreg_data[dev_id - 1];
	curr_vdd_reg = curr->vdd_data;
	curr_vccq_reg = curr->vccq_data;
	curr_vddp_reg = curr->vddp_data;

	if (init) {
		/*
		 * get the regulator handle from voltage regulator framework
		 * and then try to set the voltage level for the regulator
		 */
		if (curr_vdd_reg) {
			rc = msm_sdcc_vreg_init_reg(curr_vdd_reg);
			if (rc)
				goto out;
		}
		if (curr_vccq_reg) {
			rc = msm_sdcc_vreg_init_reg(curr_vccq_reg);
			if (rc)
				goto vdd_reg_deinit;
		}
		if (curr_vddp_reg) {
			rc = msm_sdcc_vreg_init_reg(curr_vddp_reg);
			if (rc)
				goto vccq_reg_deinit;
		}
		goto out;
	} else
		/* deregister with all regulators from regulator framework */
		goto vddp_reg_deinit;

vddp_reg_deinit:
	if (curr_vddp_reg)
		msm_sdcc_vreg_deinit_reg(curr_vddp_reg);
vccq_reg_deinit:
	if (curr_vccq_reg)
		msm_sdcc_vreg_deinit_reg(curr_vccq_reg);
vdd_reg_deinit:
	if (curr_vdd_reg)
		msm_sdcc_vreg_deinit_reg(curr_vdd_reg);
out:
	return rc;
}

static int msm_sdcc_vreg_enable(struct sdcc_reg *vreg)
{
	int rc;

	if (!vreg->enabled) {
		rc = regulator_enable(vreg->reg);
		if (rc) {
			pr_err("%s: regulator_enable(%s) failed. rc=%d\n",
				__func__, vreg->reg_name, rc);
			goto out;
		}
		vreg->enabled = 1;
	}

	/* Put always_on regulator in HPM (high power mode) */
	if (vreg->always_on && vreg->op_pwr_mode_sup) {
		rc = regulator_set_optimum_mode(vreg->reg, vreg->hpm_uA);
		if (rc < 0) {
			pr_err("%s: reg=%s: HPM setting failed"
				" hpm_uA=%d, rc=%d\n",
				__func__, vreg->reg_name,
				vreg->hpm_uA, rc);
			goto vreg_disable;
		}
		rc = 0;
	}
	goto out;

vreg_disable:
	regulator_disable(vreg->reg);
	vreg->enabled = 0;
out:
	return rc;
}

static int msm_sdcc_vreg_disable(struct sdcc_reg *vreg)
{
	int rc;

	/* Never disable always_on regulator */
	if (!vreg->always_on) {
		rc = regulator_disable(vreg->reg);
		if (rc) {
			pr_err("%s: regulator_disable(%s) failed. rc=%d\n",
				__func__, vreg->reg_name, rc);
			goto out;
		}
		vreg->enabled = 0;
	}

	/* Put always_on regulator in LPM (low power mode) */
	if (vreg->always_on && vreg->op_pwr_mode_sup) {
		rc = regulator_set_optimum_mode(vreg->reg, vreg->lpm_uA);
		if (rc < 0) {
			pr_err("%s: reg=%s: LPM setting failed"
				" lpm_uA=%d, rc=%d\n",
				__func__,
				vreg->reg_name,
				vreg->lpm_uA, rc);
			goto out;
		}
		rc = 0;
	}

out:
	return rc;
}

static int msm_sdcc_setup_vreg(int dev_id, unsigned char enable)
{
	int rc = 0;
	struct sdcc_reg *curr_vdd_reg, *curr_vccq_reg, *curr_vddp_reg;
	struct sdcc_reg_data *curr;

	if (4 == dev_id)
		goto out;

	curr = &sdcc_vreg_data[dev_id - 1];
	curr_vdd_reg = curr->vdd_data;
	curr_vccq_reg = curr->vccq_data;
	curr_vddp_reg = curr->vddp_data;

	/* check if regulators are initialized or not? */
	if ((curr_vdd_reg && !curr_vdd_reg->reg) ||
		(curr_vccq_reg && !curr_vccq_reg->reg) ||
		(curr_vddp_reg && !curr_vddp_reg->reg)) {
		/* initialize voltage regulators required for this SDCC */
		rc = msm_sdcc_vreg_init(dev_id, 1);
		if (rc) {
			pr_err("%s: regulator init failed = %d\n",
				__func__, rc);
			goto out;
		}
	}

	if (curr->sts == enable)
		goto out;

	if (curr_vdd_reg) {
		if (enable)
			rc = msm_sdcc_vreg_enable(curr_vdd_reg);
		else
			rc = msm_sdcc_vreg_disable(curr_vdd_reg);
		if (rc)
			goto out;
	}

	if (curr_vccq_reg) {
		if (enable)
			rc = msm_sdcc_vreg_enable(curr_vccq_reg);
		else
			rc = msm_sdcc_vreg_disable(curr_vccq_reg);
		if (rc)
			goto out;
	}

	if (curr_vddp_reg) {
		if (enable)
			rc = msm_sdcc_vreg_enable(curr_vddp_reg);
		else
			rc = msm_sdcc_vreg_disable(curr_vddp_reg);
		if (rc)
			goto out;
	}
	curr->sts = enable;

out:
	return rc;
}

static u32 msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	u32 rc_pin_cfg = 0;
	u32 rc_vreg_cfg = 0;
	u32 rc = 0;
	struct platform_device *pdev;
	struct msm_sdcc_pin_cfg *curr_pin_cfg;

	pdev = container_of(dv, struct platform_device, dev);

	/* setup gpio/pad */
	curr_pin_cfg = &sdcc_pin_cfg_data[pdev->id - 1];
	if (curr_pin_cfg->cfg_sts == !!vdd)
		goto setup_vreg;

	if (curr_pin_cfg->is_gpio)
		rc_pin_cfg = msm_sdcc_setup_gpio(pdev->id, !!vdd);
	else
		rc_pin_cfg = msm_sdcc_setup_pad(pdev->id, !!vdd);

setup_vreg:
	/* setup voltage regulators */
	rc_vreg_cfg = msm_sdcc_setup_vreg(pdev->id, !!vdd);

	if (rc_pin_cfg || rc_vreg_cfg)
		rc = rc_pin_cfg ? rc_pin_cfg : rc_vreg_cfg;

	return rc;
}

#ifdef	CONFIG_MMC_MSM_SDC4_SUPPORT
static int msm_sdcc_cfg_mpm_sdiowakeup(struct device *dev, unsigned mode)
{
	struct platform_device *pdev;
	enum msm_mpm_pin pin;

	pdev = container_of(dev, struct platform_device, dev);

	/* Only SDCC4 slot connected to WLAN chip has wakeup capability */
	if (pdev->id == 4)
		pin = MSM_MPM_PIN_SDC4_DAT1;
	else
		return -EINVAL;

	switch (mode) {
	case SDC_DAT1_DISABLE:
		msm_mpm_enable_pin(pin, 0);
		break;
	case SDC_DAT1_ENABLE:
		msm_mpm_set_pin_type(pin, IRQ_TYPE_LEVEL_LOW);
		msm_mpm_enable_pin(pin, 1);
		break;
	case SDC_DAT1_ENWAKE:
		msm_mpm_set_pin_wake(pin, 1);
		break;
	case SDC_DAT1_DISWAKE:
		msm_mpm_set_pin_wake(pin, 0);
		break;
	default:
		break;
	}


	return 0;
}
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
	.pclk_src_dfab	= 1,
#ifdef CONFIG_MMC_MSM_SDC1_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
};
#endif


#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static struct mmc_platform_data msm8x60_sdc4_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.board_probe	= tenderloin_probe_wifi,
	.board_remove	= tenderloin_remove_wifi,
	.dummy52_required = 1,
	.msmsdcc_fmin	= 400000,
	.msmsdcc_fmid	= 24000000,
	.msmsdcc_fmax	= 48000000,
	.nonremovable	= 1,
	.pclk_src_dfab  = 1,
	.cfg_mpm_sdiowakeup = msm_sdcc_cfg_mpm_sdiowakeup,
};
#endif


static void __init msm8x60_init_mmc(void)
{
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	/* SDCC1 : eMMC card connected */
	sdcc_vreg_data[0].vdd_data = &sdcc_vdd_reg_data[0];
	sdcc_vreg_data[0].vdd_data->reg_name = "8901_l5";
	sdcc_vreg_data[0].vdd_data->set_voltage_sup = 1;
	sdcc_vreg_data[0].vdd_data->level = 2850000;
	sdcc_vreg_data[0].vdd_data->always_on = 1;
	sdcc_vreg_data[0].vdd_data->op_pwr_mode_sup = 1;
	sdcc_vreg_data[0].vdd_data->lpm_uA = 9000;
	sdcc_vreg_data[0].vdd_data->hpm_uA = 200000;

	sdcc_vreg_data[0].vccq_data = &sdcc_vccq_reg_data[0];
	sdcc_vreg_data[0].vccq_data->reg_name = "8901_lvs0";
	sdcc_vreg_data[0].vccq_data->set_voltage_sup = 0;

	msm_add_sdcc(1, &msm8x60_sdc1_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	/* SDCC4 : WLAN WCN1314 chip is connected */
	sdcc_vreg_data[3].vdd_data = &sdcc_vdd_reg_data[3];
	sdcc_vreg_data[3].vdd_data->reg_name = "8058_s3";
	sdcc_vreg_data[3].vdd_data->set_voltage_sup = 1;
	sdcc_vreg_data[3].vdd_data->level = 1800000;

	sdcc_vreg_data[3].vccq_data = NULL;

	msm_add_sdcc(4, &msm8x60_sdc4_data);
#endif
}

#ifdef CONFIG_FB_MSM_LCDC_LG_XGA_PANEL
#define GPIO_LCD_PWR_EN  62
#define GPIO_LVDS_SHDN_N 63
#define GPIO_BACKLIGHT_EN  PM8058_GPIO_PM_TO_SYS(25-1)
/* PMIC8058 GPIO offset starts at 0 */
#define _GET_REGULATOR(var, name) do {              \
    var = regulator_get(NULL, name);            \
    if (IS_ERR(var)) {                  \
        panic("'%s' regulator not found, rc=%ld\n",    \
            name, IS_ERR(var));         \
    }                           \
} while (0)

static int lcd_panel_gpios[] = {
	0, /* lcdc_pclk */
	1, /* lcdc_hsync*/
	2, /* lcdc_vsync*/
	3, /* lcdc_den */
	4, /* lcdc_red7 */
	5, /* lcdc_red6 */
	6, /* lcdc_red5 */
	7, /* lcdc_red4 */
	8, /* lcdc_red3 */
	9, /* lcdc_red2 */
	10, /* lcdc_red1 */
	11, /* lcdc_red0 */
	12, /* lcdc_grn7 */
	13, /* lcdc_grn6 */
	14, /* lcdc_grn5 */
	15, /* lcdc_grn4 */
	16, /* lcdc_grn3 */
	17, /* lcdc_grn2 */
	18, /* lcdc_grn1 */
	19, /* lcdc_grn0 */
	20, /* lcdc_blu7 */
	21, /* lcdc_blu6 */
	22, /* lcdc_blu5 */
	23, /* lcdc_blu4 */
	24, /* lcdc_blu3 */
	25, /* lcdc_blu2 */
	26, /* lcdc_blu1 */
	27, /* lcdc_blu0 */
	//70, /* TOUCH reset */
};

static struct regulator *votg_l10 = NULL;
static struct regulator *votg_vdd5v = NULL;

DEFINE_SPINLOCK(lg_panel_pwr_lock);
DECLARE_MUTEX(lg_panel_pwr_sem);
static int lcdc_lg_panel_power(int on)
{
	int rc;
	unsigned long flags;
	static int isPowerOn = 0;

	down(&lg_panel_pwr_sem);
	/* VDD_LVDS_3.3V*/
	if(!votg_l10)
		_GET_REGULATOR(votg_l10, "8058_l10");

	/* Due to hardware change, it will not use GPIO102 as 5V boost Enable since EVT1*/
	if (board_type < TOPAZ_EVT1) {
		/* VDD_BACKLIGHT_5.0V*/
		if(!votg_vdd5v)
			_GET_REGULATOR(votg_vdd5v, "vdd50_boost");
	}

	if (on) {
		/* VDD_LVDS_3.3V ENABLE*/
		rc = regulator_set_voltage(votg_l10, 3050000, 3050000);
		if(rc) {
			pr_err("%s: Unable to set regulator voltage:"
					" votg_l10\n", __func__);
			up(&lg_panel_pwr_sem);
			return rc;
		}

		if (0 == isPowerOn) {
			printk(KERN_ERR "%s: l10 ENABLE\n", __FUNCTION__);
			rc = regulator_enable(votg_l10);
			if(rc) {
				pr_err("%s: Unable to enable the regulator:"
						" votg_l10\n", __func__);
				up(&lg_panel_pwr_sem);
				return rc;
			}

			/* Due to hardware change, it will not use GPIO102 as 5V boost Enable since EVT1*/
			if (board_type < TOPAZ_EVT1) {
				/* VDD_BACKLIGHT_5.0V ENABLE*/
				rc = regulator_enable(votg_vdd5v);
				if(rc) {
					pr_err("%s: Unable to enable the regulator:"
							" votg_vdd5v\n", __func__);
					up(&lg_panel_pwr_sem);
					return rc;
				}
			}

			isPowerOn = 1;
		}

		/* LVDS_SHDN_N*/
		rc = gpio_request(GPIO_LVDS_SHDN_N,"LVDS_SHDN_N");
		if (rc) {
			pr_err("%s: LVDS gpio %d request"
						"failed\n", __func__,
						 GPIO_LVDS_SHDN_N);
			up(&lg_panel_pwr_sem);
			return rc;
		}

		/* LCD_PWR_EN */
		rc = gpio_request(GPIO_LCD_PWR_EN, "LCD_PWR_EN");
		if (rc) {
			pr_err("%s: LCD Power gpio %d request"
						"failed\n", __func__,
						 GPIO_LCD_PWR_EN);
			gpio_free(GPIO_LVDS_SHDN_N);
			up(&lg_panel_pwr_sem);
			return rc;
		}

		/* BACKLIGHT */
		rc = gpio_request(GPIO_BACKLIGHT_EN, "BACKLIGHT_EN");
		if (rc) {
			pr_err("%s: BACKLIGHT gpio %d request"
						"failed\n", __func__,
						 GPIO_BACKLIGHT_EN);
			gpio_free(GPIO_LVDS_SHDN_N);
			gpio_free(GPIO_LCD_PWR_EN);
			up(&lg_panel_pwr_sem);
			return rc;
		}

		spin_lock_irqsave(&lg_panel_pwr_lock, flags);
		gpio_set_value(GPIO_LCD_PWR_EN, 1);
		udelay(500);
		gpio_set_value(GPIO_LVDS_SHDN_N, 1);
		spin_unlock_irqrestore(&lg_panel_pwr_lock, flags);
		msleep(200);
		gpio_set_value_cansleep(GPIO_BACKLIGHT_EN, 1);
	} else {
		if (1 == isPowerOn) {
			printk(KERN_ERR "%s: l10 DISABLE\n", __FUNCTION__);
			rc = regulator_disable(votg_l10);
			if (rc) {
				pr_err("%s: Unable to disable votg_l10\n",__func__);
				up(&lg_panel_pwr_sem);
				return rc;
			}

			/* Due to hardware change, it will not use GPIO102 as 5V boost Enable since EVT1*/
			if (board_type < TOPAZ_EVT1) {
				rc = regulator_disable(votg_vdd5v);
				if (rc) {
					pr_err("%s: Unable to disable votg_vdd5v\n",__func__);
					up(&lg_panel_pwr_sem);
					return rc;
				}
			}

			isPowerOn = 0;
		}

		gpio_set_value_cansleep(GPIO_BACKLIGHT_EN, 0);

		// msleep(200);
		spin_lock_irqsave(&lg_panel_pwr_lock, flags);
		gpio_set_value(GPIO_LVDS_SHDN_N, 0);
		gpio_set_value(GPIO_LCD_PWR_EN, 0);
		spin_unlock_irqrestore(&lg_panel_pwr_lock, flags);
		// msleep(400);
		gpio_free(GPIO_LVDS_SHDN_N);
		gpio_free(GPIO_LCD_PWR_EN);
		gpio_free(GPIO_BACKLIGHT_EN);
	}

	/* configure lcdc gpios */
	configure_gpiomux_gpios(on, lcd_panel_gpios, ARRAY_SIZE(lcd_panel_gpios));

	up(&lg_panel_pwr_sem);
	return 0;
}
#undef _GET_REGULATOR
#endif

/*
   There are currently special actions that need to associate with
   the first panel power off and power on after boot because of
   complications with bootie powering on the display and vreg-sharing
   with TP.

   TODO: Once the gpio mux and vreg ref-count mechanisms
         are established, this function should probably
         be cleaned up or eliminated.
*/

static int lcdc_panel_power(int on)
{
	int rc = 0;
	int flag_on = !!on;
	static int lcdc_power_save_on = 1;
	static int lcdc_steadycfg = 0;

	if (lcdc_power_save_on == flag_on)
		return 0;

	lcdc_power_save_on = flag_on;

	/*
	 * Right BEFORE the first display power off after boot
	 * the GPIOs will be requested (so that they can be
	 * freed).
	 *
	 * (TODO: These gpios should have been requested earlier
	 *        during init. However, one of the GPIOs is on
	 *        the PMIC, and it cannot be requested properly)
         */

	if (!lcdc_power_save_on && !lcdc_steadycfg) {

		rc = gpio_request(GPIO_LVDS_SHDN_N,"LVDS_SHDN_N");
		if (rc) {
			pr_err("%s: LVDS gpio %d request"
						"failed\n", __func__,
						 GPIO_LVDS_SHDN_N);
			return rc;
		}

		/* LCD_PWR_EN */
		rc = gpio_request(GPIO_LCD_PWR_EN, "LCD_PWR_EN");
		if (rc) {
			pr_err("%s: LCD Power gpio %d request"
						"failed\n", __func__,
						 GPIO_LCD_PWR_EN);
			gpio_free(GPIO_LVDS_SHDN_N);
			return rc;
		}

		/* BACKLIGHT */
		rc = gpio_request(GPIO_BACKLIGHT_EN, "BACKLIGHT_EN");
		if(rc) {
			pr_err("%s: BACKLIGHT gpio %d request"
						"failed\n", __func__,
						GPIO_BACKLIGHT_EN);
			gpio_free(GPIO_LVDS_SHDN_N);
			gpio_free(GPIO_LCD_PWR_EN);
			return rc;
		}
	}

#ifdef CONFIG_FB_MSM_LCDC_LG_XGA_PANEL
	rc = lcdc_lg_panel_power(on);
#endif

	/*
	 * Right AFTER the first display power on after boot,
	 * apply steady configuration.
         */
	if (lcdc_power_save_on && !lcdc_steadycfg) {
		msm8x60_gpiomux_lcdc_steadycfg();
		lcdc_steadycfg = 1;
	}

	return rc;
}
#ifdef CONFIG_MSM_BUS_SCALING
static struct msm_bus_vectors mdp_init_vectors[] = {
	/* For now, 0th array entry is reserved.
	 * Please leave 0 as is and don't use it
	 */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 0,
		.ib = 0,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors mdp_home_vectors[] = {
	/* One FB layer 1024x768
	 * For some reason it's not enough to plug in
	 * values that are 1024x768x4x60 for this
	 * use case so using the old default values
	 */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 334080000,
		.ib = 417600000,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 334080000,
		.ib = 417600000,
	},
};

static struct msm_bus_vectors mdp_app_vectors[] = {
	/* Two FB layers 1024x768 */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 377487360,
		.ib = 471859200,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 377487360,
		.ib = 471859200,
	},
};

static struct msm_bus_vectors mdp_lowres_vectors[] = {
	/* Lowres video 720x576 */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 414812160,
		.ib = 518515200,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 414812160,
		.ib = 518515200,
	},
};
static struct msm_bus_vectors mdp_pip_vectors[] = {
	/* 2 VGA video layers and less */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 432783360,
		.ib = 540979200,
	},
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 432783360,
		.ib = 540979200,
	},
};

static struct msm_bus_vectors mdp_720p_vectors[] = {
	/* 720p and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 460431360,
		.ib = 575539200,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 460431360,
		.ib = 575539200,
	},
};

static struct msm_bus_vectors mdp_1080p_vectors[] = {
	/* 1080p and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 564111360,
		.ib = 705139200,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 564111360,
		.ib = 705139200,
	},
};

static struct msm_bus_paths mdp_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(mdp_init_vectors),
		mdp_init_vectors,
	},
	{
		ARRAY_SIZE(mdp_home_vectors),
		mdp_home_vectors,
	},
	{
		ARRAY_SIZE(mdp_app_vectors),
		mdp_app_vectors,
	},
	{
		ARRAY_SIZE(mdp_lowres_vectors),
		mdp_lowres_vectors,
	},
	{
		ARRAY_SIZE(mdp_pip_vectors),
		mdp_pip_vectors,
	},
	{
		ARRAY_SIZE(mdp_720p_vectors),
		mdp_720p_vectors,
	},
	{
		ARRAY_SIZE(mdp_1080p_vectors),
		mdp_1080p_vectors,
	},
};
static struct msm_bus_scale_pdata mdp_bus_scale_pdata = {
	mdp_bus_scale_usecases,
	ARRAY_SIZE(mdp_bus_scale_usecases),
	.name = "mdp",
};

#endif
#ifdef CONFIG_MSM_BUS_SCALING
static struct msm_bus_vectors dtv_bus_init_vectors[] = {
	/* For now, 0th array entry is reserved.
	 * Please leave 0 as is and don't use it
	 */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 0,
		.ib = 0,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};
static struct msm_bus_vectors dtv_bus_def_vectors[] = {
	/* For now, 0th array entry is reserved.
	 * Please leave 0 as is and don't use it
	 */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 566092800,
		.ib = 707616000,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 566092800,
		.ib = 707616000,
	},
};
static struct msm_bus_paths dtv_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(dtv_bus_init_vectors),
		dtv_bus_init_vectors,
	},
	{
		ARRAY_SIZE(dtv_bus_def_vectors),
		dtv_bus_def_vectors,
	},
};
static struct msm_bus_scale_pdata dtv_bus_scale_pdata = {
	dtv_bus_scale_usecases,
	ARRAY_SIZE(dtv_bus_scale_usecases),
	.name = "dtv",
};

static struct lcdc_platform_data dtv_pdata = {
	.bus_scale_table = &dtv_bus_scale_pdata,
};
#endif


static struct lcdc_platform_data lcdc_pdata = {
	.lcdc_power_save   = lcdc_panel_power,
};

int mdp_core_clk_rate_table[] = {
	200000000,
	200000000,
	200000000,
	200000000,
};
static struct msm_panel_common_pdata mdp_pdata = {
	.mdp_core_clk_rate = 200000000,
	.mdp_core_clk_table = mdp_core_clk_rate_table,
	.num_mdp_clk = ARRAY_SIZE(mdp_core_clk_rate_table),
#ifdef CONFIG_MSM_BUS_SCALING
	.mdp_bus_scale_table = &mdp_bus_scale_pdata,
#endif
	.mdp_rev = MDP_REV_41,
};

#ifdef CONFIG_FB_MSM_TVOUT

#ifdef CONFIG_MSM_BUS_SCALING
static struct msm_bus_vectors atv_bus_init_vectors[] = {
	/* For now, 0th array entry is reserved.
	 * Please leave 0 as is and don't use it
	 */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 0,
		.ib = 0,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};
static struct msm_bus_vectors atv_bus_def_vectors[] = {
	/* For now, 0th array entry is reserved.
	 * Please leave 0 as is and don't use it
	 */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 236390400,
		.ib = 265939200,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 236390400,
		.ib = 265939200,
	},
};
static struct msm_bus_paths atv_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(atv_bus_init_vectors),
		atv_bus_init_vectors,
	},
	{
		ARRAY_SIZE(atv_bus_def_vectors),
		atv_bus_def_vectors,
	},
};
static struct msm_bus_scale_pdata atv_bus_scale_pdata = {
	atv_bus_scale_usecases,
	ARRAY_SIZE(atv_bus_scale_usecases),
	.name = "atv",
};
#endif

static struct tvenc_platform_data atv_pdata = {
	.poll		 = 0,
	.pm_vid_en	 = atv_dac_power,
#ifdef CONFIG_MSM_BUS_SCALING
	.bus_scale_table = &atv_bus_scale_pdata,
#endif
};
#endif

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);

	msm_fb_register_device("lcdc", &lcdc_pdata);
	msm_fb_register_device("mipi_dsi", 0);
#ifdef CONFIG_MSM_BUS_SCALING
	msm_fb_register_device("dtv", &dtv_pdata);
#endif
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

#define TENDERLOIN_CLOCK_FIXUP 1
#if TENDERLOIN_CLOCK_FIXUP
extern void msm_clock_fixup(uint32_t *fixup_clk_ids, uint32_t fixup_clk_num);
uint32_t fixup_clk_ids[] = {
	L_MDP_CLK,
	L_PIXEL_MDP_CLK,
	L_PIXEL_LCDC_CLK,
};
uint32_t fixup_clk_num = ARRAY_SIZE(fixup_clk_ids);
#endif

struct msm_board_data {
	struct msm_gpiomux_configs *gpiomux_cfgs;
};

static struct msm_board_data tenderloin_board_data __initdata = {
	.gpiomux_cfgs = tenderloin_gpiomux_cfgs,
};

static void __init platform_fixup_pin_numbers(void)
{
	// touchpanel
	xMT1386_board_info[0].irq =  MSM_GPIO_TO_INT(pin_table[MXT1386_TS_PEN_IRQ]);
	ctp_pins[0].gpio = pin_table[GPIO_CTP_WAKE_PIN];

	// BT
	bt_pins[0].gpio = pin_table[BT_RST_N_PIN];
	bt_pins[1].gpio = pin_table[BT_HOST_WAKE_PIN];

	// Lighting
	lm8502_platform_data.interrupt_gpio = pin_table[LM8502_LIGHTING_INT_IRQ_PIN];
	lm8502_board_info[0].irq = MSM_GPIO_TO_INT(pin_table[LM8502_LIGHTING_INT_IRQ_PIN]);

	// Charging
	max8903b_charger_pdata.IUSB_in = pin_table[MAX8903B_GPIO_USB_CHG_MODE_PIN];
	max8903b_charger_pdata.DOK_N_out = pin_table[MAX8903B_GPIO_DC_OK_PIN];

	// GPIO keys
	board_gpio_keys_buttons[0].gpio = pin_table[VOL_UP_GPIO_PIN];
	board_gpio_keys_buttons[1].gpio = pin_table[VOL_DN_GPIO_PIN];

	// A6
	tenderloin_a6_0_platform_data.pwr_gpio = pin_table[TENDERLOIN_A6_0_MSM_IRQ_PIN];
	tenderloin_a6_0_platform_data.sbw_tck_gpio = pin_table[TENDERLOIN_A6_0_TCK_PIN];
	tenderloin_a6_0_platform_data.sbw_tdio_gpio = pin_table[TENDERLOIN_A6_0_TDIO_PIN];
	tenderloin_a6_0_platform_data.sbw_wkup_gpio = pin_table[TENDERLOIN_A6_0_WAKEUP_PIN];

	tenderloin_a6_1_platform_data.pwr_gpio = pin_table[TENDERLOIN_A6_1_MSM_IRQ_PIN];
	tenderloin_a6_1_platform_data.sbw_tck_gpio = pin_table[TENDERLOIN_A6_1_TCK_PIN];
	tenderloin_a6_1_platform_data.sbw_tdio_gpio = pin_table[TENDERLOIN_A6_1_TDIO_PIN];
	tenderloin_a6_1_platform_data.sbw_wkup_gpio = pin_table[TENDERLOIN_A6_1_WAKEUP_PIN];

	if(boardtype_is_3g()) {
		tenderloin_a6_0_platform_data.sbw_init_gpio_config = a6_0_sbw_gpio_config_3g;
		tenderloin_a6_0_platform_data.sbw_init_gpio_config_size = ARRAY_SIZE(a6_0_sbw_gpio_config_3g);
		tenderloin_a6_0_platform_data.sbw_deinit_gpio_config = a6_0_sbw_gpio_config_3g;
		tenderloin_a6_0_platform_data.sbw_deinit_gpio_config_size = ARRAY_SIZE(a6_0_sbw_gpio_config_3g);

		tenderloin_a6_1_platform_data.sbw_init_gpio_config = a6_1_sbw_gpio_config_3g;
		tenderloin_a6_1_platform_data.sbw_init_gpio_config_size = ARRAY_SIZE(a6_1_sbw_gpio_config_3g);
		tenderloin_a6_1_platform_data.sbw_deinit_gpio_config = a6_1_sbw_gpio_config_3g;
		tenderloin_a6_1_platform_data.sbw_deinit_gpio_config_size = ARRAY_SIZE(a6_1_sbw_gpio_config_3g);

		// A6 irq
		/* 3G EVT4 boards use the same A6 gpios as DVT */
		if (board_type >= TOPAZ_3G_EVT4) {
			a6_0_config_data_3g [0] = TENDERLOIN_A6_0_MSM_IRQ_3G_DVT;
			a6_1_config_data_3g [0] = TENDERLOIN_A6_1_MSM_IRQ_3G_DVT;
			a6_0_sbw_gpio_config_3g [0] = TENDERLOIN_A6_0_TCK_3G_DVT;
		}

	} else {
		// A6 irq
		if (board_type >= TOPAZ_DVT) {
			a6_0_config_data [0] = TENDERLOIN_A6_0_MSM_IRQ_DVT;
			a6_1_config_data [0] = TENDERLOIN_A6_1_MSM_IRQ_DVT;
		}
	}

	a6_boardtype(pin_table);

}

/*
 *
 *  Do an extra board specific initialization of regulators
 *
 */
static void __init power_up_gyroscope(void)
{
	int rc;
	struct regulator *votg_l15 = NULL;
	struct regulator *votg_lvs3 = NULL;
	struct regulator *votg_l12 = NULL;
	int need_l12 =	(boardtype_is_3g() && board_type >= TOPAZ_3G_DVT) ||
			(!boardtype_is_3g() && board_type > TOPAZ_DVT);

	votg_l15 = regulator_get(NULL, "8058_l15");

	if (IS_ERR(votg_l15)) {
		pr_err("%s: failed to get regulator \"8058_l15\"\n", __func__);
		return;
	}

	rc = regulator_set_voltage(votg_l15, 2850000, 2850000);
	if (rc) {
		pr_err("%s: Unable to set regulator voltage:"
		       " votg_l15\n", __func__);
		regulator_put(votg_l15);
		return;
	}

	votg_lvs3 = regulator_get(NULL, "8901_lvs3");
	if (IS_ERR(votg_lvs3)) {
		pr_err("failed to get regulator '8901_lvs3' with %ld\n",
		       PTR_ERR(votg_lvs3));
		regulator_put(votg_l15);
		return;
	}

	if(need_l12)
	{
		votg_l12 = regulator_get(NULL, "8058_l12");
		if (IS_ERR(votg_l12)) {
			pr_err("%s: failed to get regulator \"8058_l12\"\n", __func__);
			regulator_put(votg_l15);
			regulator_put(votg_lvs3);
			return;
		}
		rc = regulator_set_voltage(votg_l12, 1800000, 1800000);
		if (rc) {
			pr_err("%s: Unable to set regulator voltage:"
			       " votg_l12\n", __func__);
			regulator_put(votg_l15);
			regulator_put(votg_lvs3);
			regulator_put(votg_l12);
			return;
		}
	}
	// Regulator reference counter will be initialized at 0 regardless of the
	// status of the regulator.
	// In the case the regulator is believed to be off the disable will fail
	// and the power up sequence won't be correctly executed.
	// Make sure the reference counter is correct before executing the OFF->ON
	// sequence.
	if (!regulator_is_enabled(votg_l15)) {
		rc = regulator_enable(votg_l15);
		if (rc < 0) {
			pr_err("failed to enable regulator 'votg_l15' with %d\n", rc);
			regulator_put(votg_l15);
			regulator_put(votg_lvs3);
			if(need_l12)
			{
				regulator_put(votg_l12);
			}
			return;
		}
	}
	if (!regulator_is_enabled(votg_lvs3)) {
		rc = regulator_enable(votg_lvs3);
		if (rc < 0) {
			pr_err("failed to enable regulator 'votg_lvs3' with %d\n", rc);
			regulator_disable(votg_l15);
			regulator_put(votg_l15);
			regulator_put(votg_lvs3);
			if(need_l12)
			{
				regulator_put(votg_l12);
			}
			return;
		}
	}
	if(need_l12)
	{
		if (!regulator_is_enabled(votg_l12)) {
			rc = regulator_enable(votg_l12);
			if (rc < 0) {
				pr_err("failed to enable regulator 'votg_l12' with %d\n", rc);
				regulator_put(votg_l15);
				regulator_put(votg_lvs3);
				regulator_put(votg_l12);
				return;
			}
		}
	}

	regulator_disable(votg_l15);
	regulator_disable(votg_lvs3);
	if (need_l12)
	{
		regulator_disable(votg_l12);
	}

	msleep(5);

	rc = regulator_enable(votg_l15);
	if (rc < 0) {
		pr_err("failed to enable regulator 'votg_l15' with %d\n", rc);
		regulator_put(votg_l15);
		regulator_put(votg_lvs3);
		if (need_l12)
		{
			regulator_put(votg_l12);
		}
		return;
	}

	rc = regulator_enable(votg_lvs3);
	if (rc < 0) {
		pr_err("failed to enable regulator 'votg_lvs3' with %d\n", rc);
		regulator_disable(votg_l15);
		regulator_put(votg_l15);
		regulator_put(votg_lvs3);
		if (need_l12)
		{
			regulator_put(votg_l12);
		}
		return;
	}
	if (need_l12)
	{
		mdelay(1000);
		rc = regulator_enable(votg_l12);
		if (rc < 0) {
			pr_err("failed to enable regulator 'votg_l12' with %d\n", rc);
			regulator_put(votg_l15);
			regulator_put(votg_lvs3);
			regulator_put(votg_l12);
			return;
		}
	}
}

/*
 *   S3A_1V8
 */
static struct regulator *board_S3A_1V8 = NULL;
static void __init board_setup_S3A_1V8 (void)
{
	int rc = 0;

#if 1
	/*
	 * Enable S3A_1V8 regulator and force it to HPM mode.
	 *
	 * Ideally, this regulator should be in HPM mode when WiFi is ON
	 * (even in suspend) or system is not in suspend.
	 *
	 * The code below is not optimal. It forces S3A_1V8 regulator to HPM
	 * mode all the time, even in suspend, which add extra 300uA to
	 * suspend power draw.
	 *
	 */
	board_S3A_1V8 = regulator_get(NULL, "8058_s3");
	if (IS_ERR(board_S3A_1V8)) {
		pr_err("%s: unable to get %s\n",
			__func__, "8058_s3");
	}
	rc = regulator_set_voltage(board_S3A_1V8, 1800000, 1800000);
	if (rc) {
		pr_err("%s: unable (%d) to set voltage for %s\n",
			__func__, rc, "8058_s3");
	}
	if (regulator_enable(board_S3A_1V8)) {
		pr_err("%s: Unable to enable %s\n",
			 __func__, "8058_s3");
	}
	rc = regulator_set_optimum_mode(board_S3A_1V8,
					2 * RPM_VREG_SMPS_HPM_MIN_LOAD);
	if (rc < 0) {
		pr_err("%s: unable (%d) to set opt mode for %s\n",
			__func__, rc, "8058_s3");

	}
	pr_info("%s: %s: forcing HPM mode (%d)\n",
			__func__, "8058_s3", rc );
#else
	/*
	 *   The Alternative approach would be to call rpm_vreg_set_voltage
	 *   API with sleep parameter set to zero. Regulator must be configured
	 *   as sleep_selectable.
	 */
	rc = rpm_vreg_set_voltage(RPM_VREG_ID_PM8058_S3,
				  enum rpm_vreg_voter voter, ??
				  0, ??
				  0);

#endif


	return;
}

static void __init board_setup_regulators (void)
{
	board_setup_S3A_1V8();

	power_up_gyroscope();
}

static int board_reboot_call(struct notifier_block *this, unsigned long code, void *_cmd)
{
	if(code == SYS_RESTART) {
		//pull down the boot config2 before reset the device
		gpio_set_value(82, 0);
		//wait enough time to discharge the gpio82
		mdelay(50);
	}
	return NOTIFY_DONE;
}

static struct notifier_block tenderloin_reboot_notifier = {
	.notifier_call = board_reboot_call,
};

static void board_register_reboot_notifier()
{
	if(boardtype_is_3g() &&(board_type == TOPAZ_3G_EVT4))
	{
		register_reboot_notifier(&tenderloin_reboot_notifier);
	}
}


#include <linux/input.h>
#include <linux/sched.h>
#include <linux/mm.h>

static int power_state 		= 0;
static int home_state 		= 0;
static int volume_dn_state	= 0;

static void diag_state_callback(struct work_struct *unused)
{
	if (power_state && home_state && volume_dn_state) {
		printk(KERN_WARNING "holy trinity detected, get ready to rumble...\n");
		
		show_state();
		show_mem();
	}
}

static DECLARE_WORK(diag_state_work, diag_state_callback);
static void diag_input_event(	struct input_handle *handle, 
				unsigned int type,
				unsigned int code, 
				int value)
{

	bool trigger_change = false;

	if (EV_KEY == type) {
		if (KEY_END == code) {
			power_state = value;
			trigger_change = true;
		}
		if (KEY_CENTER == code) {
			home_state = value;
			trigger_change = true;
		}
		if(KEY_VOLUMEDOWN == code) {
			volume_dn_state = value;
			trigger_change = true;
		}

		if (trigger_change)
			schedule_work_on(0, &diag_state_work);
	}
}

static int diag_input_connect(	struct input_handler *handler,
				struct input_dev *dev, 
				const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "diag";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void diag_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id diag_ids[] = {
	{ .driver_info = 1 },
	{ },
};

static struct input_handler diag_input_handler = {
	.event		= diag_input_event,
	.connect	= diag_input_connect,
	.disconnect	= diag_input_disconnect,
	.name		= "diag_catcher",
	.id_table	= diag_ids,
};



static void __init msm8x60_init(struct msm_board_data *board_data)
{
	uint32_t soc_platform_version;

	/*
	 * Initialize RPM first as other drivers and devices may need
	 * it for their initialization.
	 */
#ifdef CONFIG_MSM_RPM
	BUG_ON(msm_rpm_init(&msm_rpm_data));
#endif
	if (msm_xo_init())
		pr_err("Failed to initialize XO votes\n");

	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n",
		       __func__);
#ifdef CONFIG_MSM_KGSL_2D
	msm8x60_check_2d_hardware();
#endif

	/* Change SPM handling of core 1 if PMM 8160 is present. */
	soc_platform_version = socinfo_get_platform_version();
	if (SOCINFO_VERSION_MAJOR(soc_platform_version) == 1 &&
			SOCINFO_VERSION_MINOR(soc_platform_version) >= 2) {
		struct msm_spm_platform_data *spm_data;

		spm_data = &msm_spm_data_v1[1];
		spm_data->reg_init_values[MSM_SPM_REG_SAW_CFG] &= ~0x0F00UL;
		spm_data->reg_init_values[MSM_SPM_REG_SAW_CFG] |= 0x0100UL;

		spm_data = &msm_spm_data[1];
		spm_data->reg_init_values[MSM_SPM_REG_SAW_CFG] &= ~0x0F00UL;
		spm_data->reg_init_values[MSM_SPM_REG_SAW_CFG] |= 0x0100UL;
	}

	/*
	 * Initialize SPM before acpuclock as the latter calls into SPM
	 * driver to set ACPU voltages.
	 */
	if (SOCINFO_VERSION_MAJOR(socinfo_get_version()) != 1)
		msm_spm_init(msm_spm_data, ARRAY_SIZE(msm_spm_data));
	else
		msm_spm_init(msm_spm_data_v1, ARRAY_SIZE(msm_spm_data_v1));

	/*
	 * Disable regulator info printing so that regulator registration
	 * messages do not enter the kmsg log.
	 */
	regulator_suppress_info_printing();

	if(boardtype_is_3g()) {
		printk("Choosing tenderloin_pins_3g\n");
		if (board_type >= TOPAZ_3G_DVT) {
			pin_table = &tenderloin_pins_3g_dvt[0];
		} else if (board_type == TOPAZ_3G_EVT4) {
			pin_table = &tenderloin_pins_3g_evt4[0];
		} else {
			pin_table = &tenderloin_pins_3g[0];
		}
	}
	else {
		printk("Choosing tenderloin_pins_wifi\n");
		if (board_type >= TOPAZ_DVT) {
			pin_table = &tenderloin_pins_wifi_dvt[0];
		} else {
			pin_table = &tenderloin_pins_wifi[0];
		}
	}

	platform_fixup_pin_numbers();

	/* Initialize regulators needed for clock_init. */
	platform_add_devices(early_regulators, ARRAY_SIZE(early_regulators));

	msm_clock_init(msm_clocks_8x60, msm_num_clocks_8x60);

#if TENDERLOIN_CLOCK_FIXUP
	msm_clock_fixup(fixup_clk_ids, fixup_clk_num);
#endif

	/* Buses need to be initialized before early-device registration
	 * to get the platform data for fabrics.
	 */
	msm8x60_init_buses();
	platform_add_devices(early_devices, ARRAY_SIZE(early_devices));
	msm_acpu_clock_init(&msm8x60_acpu_clock_data);

	msm8x60_init_ebi2();
	msm8x60_init_tlmm();

	if(boardtype_is_3g()) {
		/* 3G EVT4 boards use the same gpiomux cfg as DVT */
		if (board_type >= TOPAZ_3G_EVT4) {
			msm8x60_init_gpiomux(tenderloin_3g_dvt_gpiomux_cfgs);
		} else {
			msm8x60_init_gpiomux(tenderloin_3g_gpiomux_cfgs);
		}
		mpu3050_i2c_board_info[0].irq = MSM_GPIO_TO_INT(TENDERLOIN_GYRO_INT_3G);

	} else {
		if (board_type >= TOPAZ_DVT) {
			msm8x60_init_gpiomux(tenderloin_dvt_gpiomux_cfgs);
		} else {
			msm8x60_init_gpiomux(tenderloin_gpiomux_cfgs);
		}
	}

	platform_add_devices(tenderloin_devices_early,
					ARRAY_SIZE(tenderloin_devices_early));

	msm8x60_init_mmc();

#if defined (CONFIG_TOUCHSCREEN_CY8CTMA395) \
	|| defined (CONFIG_TOUCHSCREEN_CY8CTMA395_MODULE)
	tenderloin_tp_init (true);
#endif

#ifdef CONFIG_MSM_DSPS
	msm8x60_init_dsps();
#endif

#ifdef CONFIG_MAX8903B_CHARGER
	//reverse the polarity of the max8903b USUS_pin on TopazWifi from DVT1 hwbuild
	if ((board_type > TOPAZ_EVT1) && (board_type != TOPAZ_3G_PROTO)) {
		max8903b_charger_pdata.USUS_in_polarity = 1;
	}
#endif

	platform_add_devices(tenderloin_devices,
					ARRAY_SIZE(tenderloin_devices));

	// Make sure the reference counters for LCD power controllers is correct
	lcdc_lg_panel_power(1);

#ifdef CONFIG_USB_EHCI_MSM
	msm_add_host(0, &msm_usb_host_pdata);
#endif
#ifdef CONFIG_USB_PEHCI_HCD
	if (boardtype_is_3g()) {
		board_register_reboot_notifier();
		msm8x60_cfg_isp1763();
		isp1763_modem_poweron(1);
	}
#endif

#ifdef CONFIG_USB_G_ANDROID
//    platform_device_register(&usb_diag_device);
    android_usb_pdata.products[0].product_id =
		android_usb_pdata.product_id;
    platform_device_register(&usb_mass_storage_device);
    platform_device_register(&rndis_device);
    platform_device_register(&android_usb_device);
#endif

	if (boardtype_is_3g())
		platform_device_register(&mdmgpio_device);
	msm_fb_add_devices();
	fixup_i2c_configs();
	register_i2c_devices();

	msm_pm_set_platform_data(msm_pm_data, ARRAY_SIZE(msm_pm_data));
	msm_cpuidle_set_states(msm_cstates, ARRAY_SIZE(msm_cstates),
				msm_pm_data);

#ifdef CONFIG_KEYBOARD_GPIO_PE
	/* GPIO KEYs*/
	board_gpio_keys_init();
#endif // CONFIG_KEYBOARD_GPIO_PE

#ifdef CONFIG_A6
	tenderloin_init_a6();
#endif

#ifdef CONFIG_MSM8X60_AUDIO
	msm_snddev_init();
#endif
	board_setup_regulators();

	input_register_handler(&diag_input_handler);
	
}

static void __init tenderloin_init(void)
{
	msm8x60_init(&tenderloin_board_data);
}

MACHINE_START(TENDERLOIN, "TENDERLOIN")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io = MSM_DEBUG_UART_PHYS & 0xfff00000,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.map_io = msm8x60_map_io,
	.init_irq = msm8x60_init_irq,
	.init_machine = tenderloin_init,
	.timer = &msm_timer,
MACHINE_END
