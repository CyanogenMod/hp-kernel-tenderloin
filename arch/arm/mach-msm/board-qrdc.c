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
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/bahama.h>
#include <linux/input/pmic8058-keypad.h>
#include <linux/pmic8058-pwrkey.h>
#include <linux/pmic8058-vibrator.h>
#include <linux/leds.h>
#include <linux/pmic8058-othc.h>
#include <linux/mfd/pmic8901.h>
#include <linux/regulator/pmic8901-regulator.h>
#include <linux/bootmem.h>
#include <linux/pwm.h>
#include <linux/pmic8058-pwm.h>
#include <linux/leds-pmic8058.h>
#include <linux/mfd/marimba.h>
#include <linux/i2c.h>
#include <linux/i2c/sx150x.h>
#include <linux/smsc911x.h>
#include <linux/spi/spi.h>
#include <linux/input/tdisc_shinetsu.h>
#include <linux/input/cy8c_ts.h>
#include <linux/input/qci_kbd.h>
#include <linux/mfd/wm8994/core.h>
#include <linux/mfd/wm8994/pdata.h>

#ifdef CONFIG_ANDROID_PMEM
#include <linux/android_pmem.h>
#endif

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>

#include <mach/mpp.h>
#include <mach/board.h>
#include <mach/irqs.h>
#include <mach/msm_spi.h>
#include <mach/msm_serial_hs.h>
#include <mach/msm_iomap.h>
#include <asm/mach/mmc.h>
#include <mach/msm_battery.h>
#include <mach/msm_hsusb.h>
#include <mach/msm_xo.h>
#include <mach/msm_bus_board.h>
#include <mach/tpm_st_i2c.h>
#include <mach/socinfo.h>
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
#include "rpm_log.h"
#include "timer.h"
#include "saw-regulator.h"
#include "rpm-regulator.h"
#include "gpiomux.h"
#include "gpiomux-8x60.h"

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

static struct msm_spm_platform_data msm_spm_data[] __initdata = {
	[0] = {
		.reg_base_addr = MSM_SAW0_BASE,

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

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x13,
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
};

static struct regulator_consumer_supply saw_s0_supply =
	REGULATOR_SUPPLY("8901_s0", NULL);
static struct regulator_consumer_supply saw_s1_supply =
	REGULATOR_SUPPLY("8901_s1", NULL);

static struct regulator_init_data saw_s0_init_data = {
		.constraints = {
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.min_uV = 840000,
			.max_uV = 1250000,
		},
		.num_consumer_supplies = 1,
		.consumer_supplies = &saw_s0_supply,
};

static struct regulator_init_data saw_s1_init_data = {
		.constraints = {
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.min_uV = 840000,
			.max_uV = 1250000,
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

#if defined(CONFIG_USB_GADGET_MSM_72K) || defined(CONFIG_USB_EHCI_MSM)
static struct regulator *ldo6_3p3;
static struct regulator *ldo7_1p8;
static struct regulator *vdd_cx;
static int msm_hsusb_ldo_init(int init)
{
	if (init) {
		ldo6_3p3 = regulator_get(NULL, "8058_l6");
		if (IS_ERR(ldo6_3p3))
			return PTR_ERR(ldo6_3p3);

		ldo7_1p8 = regulator_get(NULL, "8058_l7");
		if (IS_ERR(ldo7_1p8)) {
			regulator_put(ldo6_3p3);
			return PTR_ERR(ldo7_1p8);
		}
		/*digital core voltage for usb phy*/
		vdd_cx = regulator_get(NULL, "8058_s1");
		if (IS_ERR(vdd_cx)) {
			regulator_put(ldo6_3p3);
			regulator_put(ldo7_1p8);
			return PTR_ERR(vdd_cx);
		}

		regulator_set_voltage(vdd_cx,   1000000, 1200000);
		regulator_set_voltage(ldo7_1p8, 1800000, 1800000);
		regulator_set_voltage(ldo6_3p3, 3050000, 3050000);
	} else {
		regulator_put(ldo6_3p3);
		regulator_put(ldo7_1p8);
		regulator_put(vdd_cx);
	}
	return 0;
}

static int msm_hsusb_ldo_enable(int on)
{
	static int ldo_status;
	int ret = 0;

	if (!ldo7_1p8 || IS_ERR(ldo7_1p8)) {
		pr_err("%s: ldo7_1p8 is not initialized\n", __func__);
		return -ENODEV;
	}

	if (!ldo6_3p3 || IS_ERR(ldo6_3p3)) {
		pr_err("%s: ldo6_3p3 is not initialized\n", __func__);
		return -ENODEV;
	}

	if (!vdd_cx || IS_ERR(vdd_cx)) {
		pr_err("%s: vdd_cx is not initialized\n", __func__);
		return -ENODEV;
	}
	if (ldo_status == on)
		return 0;

	ldo_status = on;

	if (on) {
		ret = regulator_enable(ldo7_1p8);
		if (ret) {
			pr_err("%s: Unable to enable the regulator:"
				"ldo7_1p8\n", __func__);
			ldo_status = !on;
			return ret;
		}
		ret = regulator_enable(ldo6_3p3);
		if (ret) {
			pr_err("%s: Unable to enable the regulator:"
				"ldo6_3p3\n", __func__);
			regulator_disable(ldo7_1p8);
			ldo_status = !on;
			return ret;
		}
		ret = regulator_enable(vdd_cx);
		if (ret) {
			pr_err("%s: Unable to enable VDDCX digital core:"
				" vdd_dig\n", __func__);
			regulator_disable(ldo6_3p3);
			regulator_disable(ldo7_1p8);
			ldo_status = !on;
			return ret;
		}
	} else {
		/* calling regulator_disable when its already disabled might
		 * * print WARN_ON. Trying to avoid it by regulator_is_enable
		 * * */
		if (regulator_is_enabled(ldo6_3p3)) {
			ret = regulator_disable(ldo6_3p3);
			if (ret) {
				pr_err("%s: Unable to disable the regulator:"
					"ldo6_3p3\n", __func__);
				ldo_status = !on;
				return ret;
			}
		}

		if (regulator_is_enabled(ldo7_1p8)) {
			ret = regulator_disable(ldo7_1p8);
			if (ret) {
				pr_err("%s: Unable to enable the regulator:"
					" ldo7_1p8\n", __func__);
				ldo_status = !on;
				return ret;
			}
		}

		if (regulator_is_enabled(vdd_cx)) {
			ret = regulator_disable(vdd_cx);
			if (ret) {
				pr_err("%s: Unable to enable the regulator:"
					"vdd_cx\n", __func__);
				ldo_status = !on;
				return ret;
			}
		}
	}

	pr_debug("reg (%s)\n", on ? "ENABLED" : "DISABLED");
	return 0;
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

#if defined(CONFIG_BATTERY_MSM8X60) && !defined(CONFIG_USB_EHCI_MSM)
static int msm_hsusb_pmic_vbus_notif_init(void (*callback)(int online),
						int init)
{
	if (init) {
		/* TBD: right API will get filled here as a part of
		 * PMIC chanrger patch and removes thsi comments.*/
	} else {
		/* TBD: right API will get filled here as a part of
		 * PMIC chanrger patch and removes thsi comments.*/
	}
	return 0;
}
#endif
#define USB_SWITCH_EN_GPIO	132		/* !CS of analog switch */
#define USB_SWITCH_CNTL_GPIO	131		/* 0: Host, 1: Peripheral */
#define USB_HUB_RESET_GPIO	34		/* 0: HUB is RESET */

static int msm_otg_init_analog_switch_gpio(int on)
{
	int rc = 0;

	if (on) {
		/* USB SWITCH ENABLE*/
		rc = gpio_request(USB_SWITCH_EN_GPIO, "USB_SWITCH_ENABLE");
		if (rc) {
			pr_err("%s: SW_EN gpio %d request failed\n", __func__,
					USB_SWITCH_EN_GPIO);
			return rc;
		}

		/* USB SWITCH CONTROL */
		rc = gpio_request(USB_SWITCH_CNTL_GPIO, "USB_SWITCH_CONTROL");
		if (rc) {
			pr_err("%s: SW_CNTL gpio %d request failed\n", __func__,
					USB_SWITCH_CNTL_GPIO);
			goto fail_gpio_usb_switch_en;
		}

		/* USB HUB RESET */
		rc = gpio_request(USB_HUB_RESET_GPIO, "USB_HUB_RESET");
		if (rc) {
			pr_err("%s: HUB_RESET gpio %d request failed\n",
					__func__, USB_HUB_RESET_GPIO);
			goto fail_gpio_usb_switch_cntl;
		}
		/* Set direction of USB SWITCH ENABLE gpio */
		rc = gpio_direction_output(USB_SWITCH_EN_GPIO, 0);
		if (rc) {
			pr_err("%s: gpio_direction_output failed for %d\n",
					__func__, USB_SWITCH_EN_GPIO);
			goto fail_gpio_usb_hub_reset;
		}
		/* Set direction of USB SWITCH CONTROL gpio */
		rc = gpio_direction_output(USB_SWITCH_CNTL_GPIO, 0);
		if (rc) {
			pr_err("%s: gpio_direction_output failed for %d\n",
					__func__, USB_SWITCH_CNTL_GPIO);
			goto fail_gpio_usb_hub_reset;
		}
		/* Set direction of USB HUB RESET gpio */
		rc = gpio_direction_output(USB_HUB_RESET_GPIO, 0);
		if (rc) {
			pr_err("%s: gpio_direction_output failed for %d\n",
					__func__, USB_HUB_RESET_GPIO);
			goto fail_gpio_usb_hub_reset;
		}
		return rc;
	}

fail_gpio_usb_hub_reset:
	gpio_free(USB_HUB_RESET_GPIO);
fail_gpio_usb_switch_cntl:
	gpio_free(USB_SWITCH_CNTL_GPIO);
fail_gpio_usb_switch_en:
	gpio_free(USB_SWITCH_EN_GPIO);

	return rc;
}

static void msm_otg_setup_analog_switch_gpio(enum usb_switch_control mode)
{
	switch (mode) {
	case USB_SWITCH_HOST:
		/* Configure analog switch as USB host. */
		gpio_set_value(USB_SWITCH_EN_GPIO, 0);
		gpio_set_value(USB_SWITCH_CNTL_GPIO, 0);
		/* Bring HUB out of RESET */
		gpio_set_value(USB_HUB_RESET_GPIO, 1);
		break;

	case USB_SWITCH_PERIPHERAL:
		/* Configure analog switch as USB peripheral. */
		gpio_set_value(USB_SWITCH_EN_GPIO, 0);
		gpio_set_value(USB_SWITCH_CNTL_GPIO, 1);
		gpio_set_value(USB_HUB_RESET_GPIO, 0);
		break;

	case USB_SWITCH_DISABLE:
	default:
		/* Disable Switch */
		gpio_set_value(USB_SWITCH_EN_GPIO, 1);
		gpio_set_value(USB_HUB_RESET_GPIO, 0);
	}
}
#if defined(CONFIG_USB_GADGET_MSM_72K) || defined(CONFIG_USB_EHCI_MSM)
static struct msm_otg_platform_data msm_otg_pdata = {
	/* if usb link is in sps there is no need for
	 * usb pclk as dayatona fabric clock will be
	 * used instead
	 */
	.usb_in_sps = 1,
	.pemp_level		 = PRE_EMPHASIS_WITH_20_PERCENT,
	.cdr_autoreset		 = CDR_AUTO_RESET_DISABLE,
	.se1_gating		 = SE1_GATING_DISABLE,
	.init_gpio		 = msm_otg_init_analog_switch_gpio,
	.setup_gpio		 = msm_otg_setup_analog_switch_gpio,
#ifdef CONFIG_USB_EHCI_MSM
	.vbus_power = msm_hsusb_vbus_power,
#endif
#if defined(CONFIG_BATTERY_MSM8X60) && !defined(CONFIG_USB_EHCI_MSM)
	.pmic_vbus_notif_init         = msm_hsusb_pmic_vbus_notif_init,
#endif
	.otg_mode		= OTG_USER_CONTROL,
	.usb_mode		= USB_HOST_MODE,
	.ldo_init		 = msm_hsusb_ldo_init,
	.ldo_enable		 = msm_hsusb_ldo_enable,
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
	.use_gsbi_shared_mode = 1,
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
/* prim = 1366 (rounds up to 1376) x 768 x 4(bpp) x 2(pages)
 * hdmi = 1920 x 1080 x 2(bpp) x 1(page)
 * Note: must be multiple of 4096 */
#define MSM_FB_SIZE 0xC08000
#else /* CONFIG_FB_MSM_HDMI_MSM_PANEL */
#define MSM_FB_SIZE 0x811000
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */
#define MSM_PMEM_SF_SIZE 0x1700000

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
#endif

static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static int msm_fb_detect_panel(const char *name)
{
#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
	if (!strcmp(name, "hdmi_msm"))
		return 0;
#endif
	if (!strcmp(name, "lcdc_qrdc"))
		return 0;
	pr_warning("%s: not supported '%s'", __func__, name);
	return -ENODEV;
}

static void lcd_panel_power(int on);
static int msm_fb_lcdc_gpio_config(int on);
static int vga_enabled;
static int vga_enable(int select_vga)
{
	int rc = 0;

	vga_enabled = select_vga;
	rc = msm_fb_lcdc_gpio_config(1);
	if (rc)
		return rc;
	lcd_panel_power(1);

	return 0;
}

static struct msm_panel_common_pdata lcdc_qrdc_panel_data = {
	.vga_switch = vga_enable,
};

static struct platform_device lcdc_qrdc_panel_device = {
	.name = "lcdc_qrdc",
	.id = 0,
	.dev = {
		.platform_data = &lcdc_qrdc_panel_data,
	}
};

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources     = ARRAY_SIZE(msm_fb_resources),
	.resource          = msm_fb_resources,
	.dev    = {
		.platform_data = &msm_fb_pdata,
	}
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

static int hdmi_enable_5v(int on);
static int hdmi_core_power(int on, int show);
static int hdmi_cec_power(int on);

static struct msm_hdmi_platform_data hdmi_msm_data = {
	.irq = HDMI_IRQ,
	.enable_5v = hdmi_enable_5v,
	.core_power = hdmi_core_power,
	.cec_power = hdmi_cec_power,
};

static struct platform_device hdmi_msm_device = {
	.name = "hdmi_msm",
	.id = 0,
	.num_resources = ARRAY_SIZE(hdmi_msm_resources),
	.resource = hdmi_msm_resources,
	.dev.platform_data = &hdmi_msm_data,
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

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
       .inject_rx_on_wakeup = 1,
       .rx_to_inject = 0xFD,
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
 * than the low power threshold for the regulator, then the regulator will be
 * set to LPM.  Otherwise, it will be set to HPM.
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
	RPM_VREG_INIT_LDO(PM8058_L10, 0, 1, 0, 2600000, 2600000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L11, 0, 1, 0, 1500000, 1500000, LDO150HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L12, 0, 1, 0, 2900000, 2900000, LDO150HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L13, 0, 1, 0, 2050000, 2050000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L14, 0, 0, 0, 2850000, 2850000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L15, 0, 1, 0, 2850000, 2850000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L16, 1, 1, 1, 1800000, 1800000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L17, 0, 1, 0, 2600000, 2600000, LDO150HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L18, 0, 1, 1, 2200000, 2200000, LDO150HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L19, 0, 1, 0, 2500000, 2500000, LDO150HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L20, 0, 1, 0, 1800000, 1800000, LDO150HMIN, 0),
	RPM_VREG_INIT_LDO_PF(PM8058_L21, 1, 1, 0, 1200000, 1200000, LDO150HMIN,
		RPM_VREG_PIN_CTRL_NONE, RPM_VREG_PIN_FN_SLEEP_B),
	RPM_VREG_INIT_LDO(PM8058_L22, 0, 1, 0, 1200000, 1200000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L23, 0, 1, 0, 1200000, 1200000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L24, 0, 1, 0, 1200000, 1200000, LDO150HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L25, 0, 1, 0, 1200000, 1200000, LDO150HMIN, 0),

	RPM_VREG_INIT_SMPS(PM8058_S0, 0, 1, 1,  500000, 1250000,  SMPS_HMIN, 0,
		RPM_VREG_FREQ_1p60),
	RPM_VREG_INIT_SMPS(PM8058_S1, 0, 1, 1,  500000, 1250000,  SMPS_HMIN, 0,
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

static struct platform_device *early_devices[] __initdata = {
#ifdef CONFIG_MSM_BUS_SCALING
	&msm_bus_apps_fabric,
	&msm_bus_sys_fabric,
	&msm_bus_mm_fabric,
	&msm_bus_sys_fpb,
	&msm_bus_cpss_fpb,
#endif
};

static struct resource msm_aux_pcm_resources[] = {

	{
		.name   = "aux_pcm_dout",
		.start  = 111,
		.end    = 111,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_din",
		.start  = 112,
		.end    = 112,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_syncout",
		.start  = 113,
		.end    = 113,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_clkin_a",
		.start  = 114,
		.end    = 114,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device msm_aux_pcm_device = {
	.name   = "msm_aux_pcm",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_aux_pcm_resources),
	.resource       = msm_aux_pcm_resources,
};


static struct platform_device *qrdc_devices[] __initdata = {
	&msm_device_smd,
	&smsc911x_device,
	&msm_device_uart_dm3,
	&msm_device_dmov_adm0,
	&msm_device_dmov_adm1,
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
	&lcdc_qrdc_panel_device,
	&msm_device_kgsl,
	&lcdc_samsung_panel_device,
#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
	&hdmi_msm_device,
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */
	&mipi_dsi_video_toshiba_wvga_panel_device,
#ifdef CONFIG_MSM_GEMINI
	&msm_gemini_device,
#endif
#ifdef CONFIG_MSM_VPE
	&msm_vpe_device,
#endif
#if defined(CONFIG_MSM_RPM_LOG) || defined(CONFIG_MSM_RPM_LOG_MODULE)
	&msm_rpm_log_device,
#endif
	&msm_device_vidc,
	&msm_aux_pcm_device,
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
};

#ifdef CONFIG_PMIC8058
#define PMIC_GPIO_SDC3_DET 22
#define PMIC_GPIO_EXT_POWER 1

static int pm8058_gpios_init(void)
{
	int i;
	int rc;
	struct pm8058_gpio_cfg {
		int                gpio;
		struct pm8058_gpio cfg;
	};

	struct pm8058_gpio_cfg gpio_cfgs[] = {
		{ /* External power enable */
			PMIC_GPIO_EXT_POWER - 1,
			{
				.direction      = PM_GPIO_DIR_OUT,
				.output_value   = 1,
				.pull           = PM_GPIO_PULL_UP_30,
				.vin_sel        = 2,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			},
		},
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

static struct othc_regulator_config othc_reg = {
	.regulator	 = "8058_l5",
	.max_uV		 = 2850000,
	.min_uV		 = 2850000,
};

/* MIC_BIAS0 is configured as normal MIC BIAS */
static struct pmic8058_othc_config_pdata othc_config_pdata_0 = {
	.micbias_select = OTHC_MICBIAS_0,
	.micbias_capability = OTHC_MICBIAS,
	.micbias_enable = OTHC_SIGNAL_OFF,
	.micbias_regulator = &othc_reg,
};

/* MIC_BIAS1 is configured as HSED_BIAS for OTHC */
static struct pmic8058_othc_config_pdata othc_config_pdata_1 = {
	.micbias_select = OTHC_MICBIAS_1,
	.micbias_capability = OTHC_MICBIAS,
	.micbias_enable = OTHC_SIGNAL_OFF,
	.micbias_regulator = &othc_reg,
};

/* MIC_BIAS2 is configured as normal MIC BIAS */
static struct pmic8058_othc_config_pdata othc_config_pdata_2 = {
	.micbias_select = OTHC_MICBIAS_2,
	.micbias_capability = OTHC_MICBIAS,
	.micbias_enable = OTHC_SIGNAL_OFF,
	.micbias_regulator = &othc_reg,
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
		.name = "pm8058-othc",
		.id = 0,
		.platform_data = &othc_config_pdata_0,
		.data_size = sizeof(othc_config_pdata_0),
		.num_resources = ARRAY_SIZE(resources_othc_0),
		.resources = resources_othc_0,
	},
	{
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
		.num_resources  = ARRAY_SIZE(resources_rtc),
		.resources      = resources_rtc,
	},
	{	.name = "pm8058-upl",
		.id		= -1,
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
	int i;

	for (i = 0; i < ARRAY_SIZE(vregs_tdisc_name); i++)
		regulator_put(vregs_tdisc[i]);

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

static int tdisc_shinetsu_disable(void)
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

	return 0;
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

#define TPM_PWR_EN_GPIO 94
#define TPM_RESET_GPIO 66
#define TPM_DATA_AVAIL_GPIO 67
#define TPM_ACCEPT_CMD_GPIO 68

struct tpm_gpio {
	int gpio;
	const char *name;
};
static struct tpm_gpio tpm_gpios[] = {
	{
		.gpio = TPM_PWR_EN_GPIO,
		.name = "tpm_pwr_en",
	},
	{
		.gpio = TPM_RESET_GPIO,
		.name = "tpm_reset",
	},
	{
		.gpio = TPM_ACCEPT_CMD_GPIO,
		.name = "tpm_accept_cmd",
	},
	{
		.gpio = TPM_DATA_AVAIL_GPIO,
		.name = "tpm_data_avail",
	},
};

static void tpm_st_i2c_gpio_release(void)
{
	int i;

	gpio_set_value_cansleep(TPM_RESET_GPIO, 0);
	gpio_set_value_cansleep(TPM_PWR_EN_GPIO, 0);
	for (i = 0; i < ARRAY_SIZE(tpm_gpios); i++)
		gpio_free(tpm_gpios[i].gpio);
}

static int tpm_st_i2c_gpio_setup(void)
{
	int rc = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(tpm_gpios); i++) {
		rc = gpio_request(tpm_gpios[i].gpio, tpm_gpios[i].name);
		if (rc) {
			pr_err("%s: gpio request failed for gpio #%d, %s",
			       __func__, GPIO_PIN(tpm_gpios[i].gpio),
			       tpm_gpios[i].name);
			goto free_gpios;
		}
	}

	rc = gpio_direction_output(TPM_RESET_GPIO, 0);
	if (rc) {
		pr_err("%s: failed to setup tpm_reset\n", __func__);
		goto free_gpios;
	}
	rc = gpio_direction_output(TPM_PWR_EN_GPIO, 1);
	if (rc) {
		pr_err("%s: failed to set tpm_pwr_en\n", __func__);
		goto free_gpios;
	}
	msleep(1);
	gpio_set_value_cansleep(TPM_RESET_GPIO, 1);
	msleep(1);

	return rc;

free_gpios:
	for (i--; i >= 0; i--)
		gpio_free(tpm_gpios[i].gpio);
	return rc;
}

static struct tpm_st_i2c_platform_data tpm_st_i2c_data = {
	.accept_cmd_gpio = TPM_ACCEPT_CMD_GPIO,
	.data_avail_gpio = TPM_DATA_AVAIL_GPIO,
	.accept_cmd_irq = MSM_GPIO_TO_INT(TPM_ACCEPT_CMD_GPIO),
	.data_avail_irq = MSM_GPIO_TO_INT(TPM_DATA_AVAIL_GPIO),
	.gpio_setup = tpm_st_i2c_gpio_setup,
	.gpio_release = tpm_st_i2c_gpio_release,
};

static struct i2c_board_info msm_i2c_gsbi3_tpm_info[] = {
	{
		I2C_BOARD_INFO("tpm_st_i2c", 0x13),
		.platform_data = &tpm_st_i2c_data,
	},
};

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
		regulator_put(vreg_timpani_1);
		return -ENODEV;
	}

	rc = regulator_set_voltage(vreg_timpani_1, 1200000, 1200000);
	if (rc) {
		pr_err("%s: unable to set L0 voltage to 1.2V\n", __func__);
		goto fail;
	}

	rc = regulator_set_voltage(vreg_timpani_2, 1800000, 1800000);
	if (rc) {
		pr_err("%s: unable to set S3 voltage to 1.8V\n", __func__);
		goto fail;
	}

	rc = regulator_enable(vreg_timpani_1);
	if (rc) {
		pr_err("%s: Enable regulator 8058_l0 failed\n", __func__);
		goto fail;
	}

	rc = regulator_enable(vreg_timpani_2);
	if (rc) {
		pr_err("%s: Enable regulator 8058_s3 failed\n", __func__);
		regulator_disable(vreg_timpani_1);
		goto fail;
	}
	return rc;

fail:
	regulator_put(vreg_timpani_1);
	regulator_put(vreg_timpani_2);
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
			return rc;
		}
	}

	if (vreg_on) {

		rc = regulator_set_voltage(vreg_timpani_cdc_apwr,
				2200000, 2200000);
		if (rc) {
			pr_err("%s: unable to set 8058_s4 voltage to 2.2 V\n",
					__func__);
			goto vreg_fail;
		}

		rc = regulator_enable(vreg_timpani_cdc_apwr);
		if (rc) {
			pr_err("%s: vreg_enable failed %d\n", __func__, rc);
			goto vreg_fail;
		}
	} else {
		rc = regulator_disable(vreg_timpani_cdc_apwr);
		if (rc) {
			pr_err("%s: vreg_disable failed %d\n",
			__func__, rc);
			goto vreg_fail;
		}
	}

	return 0;

vreg_fail:
	regulator_put(vreg_timpani_cdc_apwr);
	vreg_timpani_cdc_apwr = NULL;
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

static struct qci_kbd_platform_data qci_i2ckbd_pdata = {
#ifdef CONFIG_KEYBOARD_QCIKBD_REPEAT
	.repeat = true,
#endif
       .standard_scancodes = true,
};

static struct i2c_board_info msm_i2c_gsbi3_qci_input_info[] = {
	{
		I2C_BOARD_INFO("qci-i2ckbd", 0x18),
		.irq = 58,
		.platform_data = &qci_i2ckbd_pdata,
	},
	{
		I2C_BOARD_INFO("qci-i2cpad", 0x19),
		.irq = 52,
	},
	{
		I2C_BOARD_INFO("qci-i2cec", 0x1A),
		.irq = 117,
	},
};

#ifdef CONFIG_MFD_WM8994

#define WM8994_I2C_SLAVE_ADDR 0x1A

struct wm8994_pdata wm8994_platform_data = {
	.gpio_defaults[0] = 0xA101,
};

static struct i2c_board_info msm_i2c_gsbi7_wm8994_info[] = {
  {
	I2C_BOARD_INFO("wm8994", WM8994_I2C_SLAVE_ADDR),
	.platform_data = &wm8994_platform_data,
  },
};

#endif /*CONFIG_MFD_WM8994 */

#ifdef CONFIG_I2C

struct i2c_registry {
	int                    bus;
	struct i2c_board_info *info;
	int                    len;
};

static struct i2c_registry msm8x60_i2c_devices[] __initdata = {
#ifdef CONFIG_PMIC8058
	{
		MSM_SSBI1_I2C_BUS_ID,
		pm8058_boardinfo,
		ARRAY_SIZE(pm8058_boardinfo),
	},
#endif
#ifdef CONFIG_PMIC8901
	{
		MSM_SSBI2_I2C_BUS_ID,
		pm8901_boardinfo,
		ARRAY_SIZE(pm8901_boardinfo),
	},
#endif
#if defined(CONFIG_TOUCHDISC_VTD518_SHINETSU) || \
		defined(CONFIG_TOUCHDISC_VTD518_SHINETSU_MODULE)
	{
		MSM_GSBI3_QUP_I2C_BUS_ID,
		msm_i2c_gsbi3_tdisc_info,
		ARRAY_SIZE(msm_i2c_gsbi3_tdisc_info),
	},
#endif
	{
		MSM_GSBI7_QUP_I2C_BUS_ID,
		msm_i2c_gsbi7_timpani_info,
		ARRAY_SIZE(msm_i2c_gsbi7_timpani_info),
	},
	{
		MSM_GSBI3_QUP_I2C_BUS_ID,
		msm_i2c_gsbi3_qci_input_info,
		ARRAY_SIZE(msm_i2c_gsbi3_qci_input_info),
	},
	{
		MSM_GSBI3_QUP_I2C_BUS_ID,
		msm_i2c_gsbi3_tpm_info,
		ARRAY_SIZE(msm_i2c_gsbi3_tpm_info),
	},
#ifdef CONFIG_MFD_WM8994
	{
		MSM_GSBI7_QUP_I2C_BUS_ID,
		msm_i2c_gsbi7_wm8994_info,
		ARRAY_SIZE(msm_i2c_gsbi7_wm8994_info),
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
	if (machine_is_msm8x60_surf() || machine_is_msm8x60_charm_surf())
		pm8901_vreg_init_pdata[PM8901_VREG_ID_MPP0].active_high = 0;
	else
		pm8901_vreg_init_pdata[PM8901_VREG_ID_MPP0].active_high = 1;
#endif
}

static void register_i2c_devices(void)
{
#ifdef CONFIG_I2C
	int i;

	/* Run the array and install devices as appropriate */
	for (i = 0; i < ARRAY_SIZE(msm8x60_i2c_devices); ++i) {
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
	void *gsbi_mem = ioremap_nocache(0x16200000, 4);
	/* Setting protocol code to 0x60 for dual UART/I2C in GSBI3 */
	writel(0x6 << 4, gsbi_mem);
	iounmap(gsbi_mem);
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
		ebi2_cfg |= (1 << 4) | (1 << 5); /* CS2, CS3 */
		writel(ebi2_cfg, ebi2_cfg_ptr);
		iounmap(ebi2_cfg_ptr);
	}

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

struct msm8x60_tlmm_cfg_struct {
	unsigned gpio;
	u32      flags;
};

static uint32_t msm8x60_tlmm_cfgs[] = {
#ifdef CONFIG_PMIC8058
	/* PMIC8058 */
	GPIO_CFG(PM8058_GPIO_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
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

	/* UARTDM_TX */
	GPIO_CFG(41, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	/* UARTDM_RX */
	GPIO_CFG(42, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
#endif
#ifdef CONFIG_PMIC8901
	/* PMIC8901 */
	GPIO_CFG(PM8901_GPIO_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
#endif
};

static void __init msm8x60_init_tlmm(void)
{
	unsigned n;

	for (n = 0; n < ARRAY_SIZE(msm8x60_tlmm_cfgs); ++n)
		gpio_tlmm_config(msm8x60_tlmm_cfgs[n], 0);
}

#define GPIO_SDC3_WP_SWITCH (GPIO_EXPANDER_GPIO_BASE + (16 * 1) + 6)
#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC5_SUPPORT))
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

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static struct msm_sdcc_gpio sdc2_gpio_cfg[] = {
	{143, "sdc2_dat_0"},
	{144, "sdc2_dat_1"},
	{145, "sdc2_dat_2"},
	{146, "sdc2_dat_3"},
#ifdef CONFIG_MMC_MSM_SDC2_8_BIT_SUPPORT
	{147, "sdc2_dat_4"},
	{148, "sdc2_dat_5"},
	{149, "sdc2_dat_6"},
	{150, "sdc2_dat_7"},
#endif
	{151, "sdc2_cmd"},
	{152, "sdc2_clk"}
};
#endif

#ifdef CONFIG_MMC_MSM_SDC5_SUPPORT
static struct msm_sdcc_gpio sdc5_gpio_cfg[] = {
	{95, "sdc5_cmd"},
	{96, "sdc5_dat_3"},
	{97, "sdc5_clk"},
	{98, "sdc5_dat_2"},
	{99, "sdc5_dat_1"},
	{100, "sdc5_dat_0"}
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

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static struct msm_sdcc_pad_drv_cfg sdc3_pad_on_drv_cfg[] = {
	{TLMM_HDRV_SDC3_CLK, GPIO_CFG_8MA},
	{TLMM_HDRV_SDC3_CMD, GPIO_CFG_8MA},
	{TLMM_HDRV_SDC3_DATA, GPIO_CFG_8MA}
};

static struct msm_sdcc_pad_pull_cfg sdc3_pad_on_pull_cfg[] = {
	{TLMM_PULL_SDC3_CMD, GPIO_CFG_PULL_UP},
	{TLMM_PULL_SDC3_DATA, GPIO_CFG_PULL_UP}
};

static struct msm_sdcc_pad_drv_cfg sdc3_pad_off_drv_cfg[] = {
	{TLMM_HDRV_SDC3_CLK, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC3_CMD, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC3_DATA, GPIO_CFG_2MA}
};

static struct msm_sdcc_pad_pull_cfg sdc3_pad_off_pull_cfg[] = {
	{TLMM_PULL_SDC3_CMD, GPIO_CFG_PULL_DOWN},
	{TLMM_PULL_SDC3_DATA, GPIO_CFG_PULL_DOWN}
};
#endif

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
	{TLMM_HDRV_SDC4_CLK, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC4_CMD, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC4_DATA, GPIO_CFG_2MA}
};

static struct msm_sdcc_pad_pull_cfg sdc4_pad_off_pull_cfg[] = {
	{TLMM_PULL_SDC4_CMD, GPIO_CFG_PULL_DOWN},
	{TLMM_PULL_SDC4_DATA, GPIO_CFG_PULL_DOWN}
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
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	[1] = {
		.is_gpio = 1,
		.gpio_data_size = ARRAY_SIZE(sdc2_gpio_cfg),
		.gpio_data = sdc2_gpio_cfg
	},
#endif
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	[2] = {
		.is_gpio = 0,
		.pad_drv_on_data = sdc3_pad_on_drv_cfg,
		.pad_drv_off_data = sdc3_pad_off_drv_cfg,
		.pad_pull_on_data = sdc3_pad_on_pull_cfg,
		.pad_pull_off_data = sdc3_pad_off_pull_cfg,
		.pad_drv_data_size = ARRAY_SIZE(sdc3_pad_on_drv_cfg),
		.pad_pull_data_size = ARRAY_SIZE(sdc3_pad_on_pull_cfg)
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
#ifdef CONFIG_MMC_MSM_SDC5_SUPPORT
	[4] = {
		.is_gpio = 1,
		.gpio_data_size = ARRAY_SIZE(sdc5_gpio_cfg),
		.gpio_data = sdc5_gpio_cfg
	}
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
};
/* all 5 SDCC controllers requires VDD/VCC voltage  */
static struct sdcc_reg sdcc_vdd_reg_data[5];
/* only SDCC1 requires VCCQ voltage */
static struct sdcc_reg sdcc_vccq_reg_data[1];

struct sdcc_reg_data {
	struct sdcc_reg *vdd_data; /* keeps VDD/VCC regulator info */
	struct sdcc_reg *vccq_data; /* keeps VCCQ regulator info */
	unsigned char sts; /* regulator enable/disable status */
};
/* msm8x60 have 5 SDCC controllers */
static struct sdcc_reg_data sdcc_vreg_data[5];

/* this init function should be called only once for each SDCC */
static int msm_sdcc_vreg_init(int dev_id, unsigned char init)
{
	int rc = 0;
	struct sdcc_reg *curr_vdd_reg;
	struct sdcc_reg *curr_vccq_reg;
	struct sdcc_reg_data *curr;

	curr = &sdcc_vreg_data[dev_id - 1];
	curr_vdd_reg = curr->vdd_data;
	curr_vccq_reg = curr->vccq_data;

	if (init) {
		/*
		 * get the regulator handle from voltage regulator framework
		 * and then try to set the voltage level for the regulator
		 */
		if (curr_vdd_reg) {
			curr_vdd_reg->reg =
				regulator_get(NULL, curr_vdd_reg->reg_name);
			if (IS_ERR(curr_vdd_reg->reg)) {
				rc = PTR_ERR(curr_vdd_reg->reg);
				pr_err("%s: regulator_get(%s) failed = %d\n",
					__func__, curr_vdd_reg->reg_name, rc);
				goto out;
			}

			if (curr_vdd_reg->set_voltage_sup) {
				rc = regulator_set_voltage(curr_vdd_reg->reg,
					curr_vdd_reg->level,
					curr_vdd_reg->level);
				if (rc) {
					pr_err("%s: regulator_set_voltage(%s)"
						" = %d\n", __func__,
						curr_vdd_reg->reg_name, rc);
					goto vdd_reg_put;
				}
			}
		}

		if (curr_vccq_reg) {
			curr_vccq_reg->reg =
				regulator_get(NULL, curr_vccq_reg->reg_name);
			if (IS_ERR(curr_vccq_reg->reg)) {
				rc = PTR_ERR(curr_vccq_reg->reg);
				pr_err("%s: regulator get of %s failed (%d)\n",
					__func__, curr_vccq_reg->reg_name, rc);
				goto vdd_reg_put;
			}
			if (curr_vccq_reg->set_voltage_sup) {
				rc = regulator_set_voltage(curr_vccq_reg->reg,
					curr_vccq_reg->level,
					curr_vccq_reg->level);
				if (rc) {
					pr_err("%s: regulator_set_voltage()"
						"= %d\n", __func__, rc);
					goto vccq_reg_put;
				}
			}
		}
	} else {
		/* deregister with voltage regulator framework */
		rc = 0;
		goto vccq_reg_put;
	}
	goto out;

vccq_reg_put:
	if (curr_vccq_reg)
		regulator_put(curr_vccq_reg->reg);
vdd_reg_put:
	if (curr_vdd_reg)
		regulator_put(curr_vdd_reg->reg);
out:
	return rc;
}

static int msm_sdcc_setup_vreg(int dev_id, unsigned char enable)
{
	int rc = 0;
	struct sdcc_reg *curr_vdd_reg;
	struct sdcc_reg *curr_vccq_reg;
	struct sdcc_reg_data *curr;

	curr = &sdcc_vreg_data[dev_id - 1];
	curr_vdd_reg = curr->vdd_data;
	curr_vccq_reg = curr->vccq_data;

	/* check if regulators are initialized or not? */
	if ((curr_vdd_reg && !curr_vdd_reg->reg) ||
		(curr_vccq_reg && !curr_vccq_reg->reg)) {
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

	if (enable) {
		if (curr_vdd_reg) {
			rc = regulator_enable(curr_vdd_reg->reg);
			if (rc) {
				pr_err("%s: regulator_enable(%s) failed"
					" = %d\n", __func__,
					curr_vdd_reg->reg_name, rc);
				goto out;
			}
		}
		if (curr_vccq_reg) {
			rc = regulator_enable(curr_vccq_reg->reg);
			if (rc) {
				pr_err("%s: regulator_enable(%s) failed"
					" = %d\n", __func__,
					curr_vccq_reg->reg_name, rc);
				goto vdd_reg_disable;
			}
		}
		/*
		 * now we can safely say that all required regulators
		 * are enabled for this SDCC
		 */
		curr->sts = enable;
	} else {
		if (curr_vdd_reg) {
			rc = regulator_disable(curr_vdd_reg->reg);
			if (rc) {
				pr_err("%s: regulator_disable(%s) = %d\n",
					__func__, curr_vdd_reg->reg_name, rc);
				goto out;
			}
		}

		if (curr_vccq_reg) {
			rc = regulator_disable(curr_vccq_reg->reg);
			if (rc) {
				pr_err("%s: regulator_disable(%s) = %d\n",
					__func__, curr_vccq_reg->reg_name, rc);
				goto out;
			}
		}
		/*
		 * now we can safely say that all required
		 * regulators are disabled for this SDCC
		 */
		curr->sts = enable;
	}
	goto out;

vdd_reg_disable:
	if (curr_vdd_reg)
		regulator_disable(curr_vdd_reg->reg);
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
		status = gpio_get_value_cansleep(GPIO_SDC3_WP_SWITCH);
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

	status = !(gpio_get_value_cansleep(
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
	.wpswitch	= msm_sdc3_get_wpswitch,
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
#ifdef CONFIG_MMC_MSM_SDC4_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
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
	/* SDCC1 : eMMC card connected */
	sdcc_vreg_data[0].vdd_data = &sdcc_vdd_reg_data[0];
	sdcc_vreg_data[0].vdd_data->reg_name = "8901_l5";
	sdcc_vreg_data[0].vdd_data->set_voltage_sup = 1;
	sdcc_vreg_data[0].vdd_data->level = 2850000;
	sdcc_vreg_data[0].vccq_data = &sdcc_vccq_reg_data[0];
	sdcc_vreg_data[0].vccq_data->reg_name = "8901_lvs0";
	sdcc_vreg_data[0].vccq_data->set_voltage_sup = 0;
	msm_add_sdcc(1, &msm8x60_sdc1_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	/* SDCC2 : NC (no card connected)*/
	sdcc_vreg_data[1].vdd_data = &sdcc_vdd_reg_data[1];
	sdcc_vreg_data[1].vdd_data->reg_name = "8058_s3";
	sdcc_vreg_data[1].vdd_data->set_voltage_sup = 1;
	sdcc_vreg_data[1].vdd_data->level = 1800000;
	sdcc_vreg_data[1].vccq_data = NULL;
	msm_add_sdcc(2, &msm8x60_sdc2_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	/* SDCC3 : External card slot connected */
	sdcc_vreg_data[2].vdd_data = &sdcc_vdd_reg_data[2];
	sdcc_vreg_data[2].vdd_data->reg_name = "8058_l14";
	sdcc_vreg_data[2].vdd_data->set_voltage_sup = 1;
	sdcc_vreg_data[2].vdd_data->level = 2850000;
	sdcc_vreg_data[2].vccq_data = NULL;
	msm_add_sdcc(3, &msm8x60_sdc3_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	/* SDCC4 : NC (no card connected)*/
	sdcc_vreg_data[3].vdd_data = &sdcc_vdd_reg_data[3];
	sdcc_vreg_data[3].vdd_data->reg_name = "8058_s3";
	sdcc_vreg_data[3].vdd_data->set_voltage_sup = 1;
	sdcc_vreg_data[3].vdd_data->level = 1800000;
	sdcc_vreg_data[3].vccq_data = NULL;
	msm_add_sdcc(4, &msm8x60_sdc4_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC5_SUPPORT
	/* SDCC4 : NC (no card connected)*/
	sdcc_vreg_data[4].vdd_data = &sdcc_vdd_reg_data[4];
	sdcc_vreg_data[4].vdd_data->reg_name = "8058_s3";
	sdcc_vreg_data[4].vdd_data->set_voltage_sup = 1;
	sdcc_vreg_data[4].vdd_data->level = 1800000;
	sdcc_vreg_data[4].vccq_data = NULL;
	msm_add_sdcc(5, &msm8x60_sdc5_data);
#endif
}

static int wifi_power(int on)
{
	int rc = 0;
	static struct regulator *reg_8901_l2;

	if (reg_8901_l2 == NULL) {
		reg_8901_l2 = regulator_get(NULL, "8901_l2");
		if (IS_ERR(reg_8901_l2)) {
			pr_err("%s: Unable to get 8901_l2\n", __func__);
			return PTR_ERR(reg_8901_l2);
		}
	}

	if (on) {
		rc = regulator_set_voltage(reg_8901_l2, 3300000, 3300000);
		if (rc) {
			pr_err("%s: error set 8901_l2 to 3.3V\n", __func__);
			goto wifi_power_fail;
		}

		rc = regulator_enable(reg_8901_l2);
		if (rc) {
			pr_err("%s: 8901_l2 enable failed, rc=%d\n",
				__func__, rc);
			goto wifi_power_fail;
		}
	} else {
		if (regulator_is_enabled(reg_8901_l2)) {
			rc = regulator_disable(reg_8901_l2);
			if (rc)
				pr_warning("%s: 8901_l2 disable failed, "
					"rc=%d\n", __func__, rc);
		}
	}

	return rc;

wifi_power_fail:
	regulator_put(reg_8901_l2);
	return rc;
}

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

static struct regulator *reg_8901_l1;
static struct regulator *vga_5v_reg;

static int lcdc_vga_panel_power(int on)
{
	int rc = 0;

	if (on) {
		rc = regulator_enable(vga_5v_reg);
		if (rc) {
			pr_err("%s: VGA 5v reg enable failed, rc=%d\n",
				__func__, rc);
			return rc;
		}
	} else
		if (regulator_is_enabled(vga_5v_reg)) {
			rc = regulator_disable(vga_5v_reg);
			if (rc)
				pr_warning("%s: VGA 5v reg disable failed, "
					"rc=%d\n", __func__, rc);
		}

	return rc;
}

static void lcd_panel_power(int on)
{
	int n;
	int rc = 0;

	if (!reg_8901_l1) {
		reg_8901_l1 = regulator_get(NULL, "8901_l1");
		if (IS_ERR(reg_8901_l1)) {
			pr_err("%s: Unable to get 8901_l1\n", __func__);
			return;
		}
	}

	if (!vga_5v_reg) {
		vga_5v_reg = regulator_get(NULL, "8901_usb_otg");
		if (IS_ERR(vga_5v_reg)) {
			pr_err("%s: Unable to get 8901_usb_otg\n", __func__);
			goto fail_get_vga_5v;
		}
	}

	if (on) {
		rc = regulator_set_voltage(reg_8901_l1, 3300000, 3300000);
		if (rc) {
			pr_err("%s: error set 8901_l1 to 3.3V\n", __func__);
			goto fail_l1_enable;
		}

		rc = regulator_enable(reg_8901_l1);
		if (rc) {
			pr_err("%s: 8901_l1 enable failed, rc=%d\n",
				__func__, rc);
			goto fail_l1_enable;
		}

		if (vga_enabled) {
			if (lcdc_vga_panel_power(1))
				goto fail_vga_enable;
		} else if (lcdc_vga_panel_power(0))
				pr_warning("%s: failed to turn off VGA "
					   "display, rc=%d\n", __func__, rc);
	} else {
		if (regulator_is_enabled(reg_8901_l1)) {
			rc = regulator_disable(reg_8901_l1);
			if (rc)
				pr_warning("%s: 8901_l1 disable failed, "
					"rc=%d\n", __func__, rc);
		}

		lcdc_vga_panel_power(0);
	}

	/*TODO if on = 0 free the gpio's */
	for (n = 0; n < ARRAY_SIZE(lcd_panel_gpios); ++n)
		gpio_tlmm_config(lcd_panel_gpios[n], 0);

	return;


fail_get_vga_5v:
	regulator_put(reg_8901_l1);
	reg_8901_l1 = NULL;

	return;


fail_vga_enable:
	regulator_disable(reg_8901_l1);
fail_l1_enable:
	regulator_put(reg_8901_l1);
	regulator_put(vga_5v_reg);
	reg_8901_l1 = NULL;
	vga_5v_reg = NULL;
}

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
#define _GET_REGULATOR(var, name) do {				\
	var = regulator_get(NULL, name);			\
	if (IS_ERR(var)) {					\
		pr_err("'%s' regulator not found, rc=%ld\n",	\
			name, IS_ERR(var));			\
		var = NULL;					\
		return -ENODEV;					\
	}							\
} while (0)

static int hdmi_enable_5v(int on)
{
	static struct regulator *reg_8901_hdmi_mvs;	/* HDMI_5V */
	static struct regulator *reg_8901_mpp0;		/* External 5V */
	static int prev_on;
	int rc;

	if (on == prev_on)
		return 0;

	if (!reg_8901_hdmi_mvs)
		_GET_REGULATOR(reg_8901_hdmi_mvs, "8901_hdmi_mvs");
	if (!reg_8901_mpp0)
		_GET_REGULATOR(reg_8901_mpp0, "8901_mpp0");

	if (on) {
		rc = regulator_enable(reg_8901_mpp0);
		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
				"reg_8901_mpp0", rc);
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
		rc = regulator_disable(reg_8901_hdmi_mvs);
		if (rc)
			pr_warning("'%s' regulator disable failed, rc=%d\n",
				"8901_hdmi_mvs", rc);
		rc = regulator_disable(reg_8901_mpp0);
		if (rc)
			pr_warning("'%s' regulator disable failed, rc=%d\n",
				"reg_8901_mpp0", rc);
		pr_info("%s(off): success\n", __func__);
	}

	prev_on = on;

	return 0;
}

static int hdmi_core_power(int on, int show)
{
	static struct regulator *reg_8058_l16;		/* VDD_HDMI */
	static int prev_on;
	int rc;

	if (on == prev_on)
		return 0;

	if (!reg_8058_l16)
		_GET_REGULATOR(reg_8058_l16, "8058_l16");

	if (on) {
		rc = regulator_set_voltage(reg_8058_l16, 1800000, 1800000);
		if (!rc)
			rc = regulator_enable(reg_8058_l16);
		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
				"8058_l16", rc);
			return rc;
		}
		rc = gpio_request(170, "HDMI_DDC_CLK");
		if (rc) {
			pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
				"HDMI_DDC_CLK", 170, rc);
			goto error1;
		}
		rc = gpio_request(171, "HDMI_DDC_DATA");
		if (rc) {
			pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
				"HDMI_DDC_DATA", 171, rc);
			goto error2;
		}
		rc = gpio_request(172, "HDMI_HPD");
		if (rc) {
			pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
				"HDMI_HPD", 172, rc);
			goto error3;
		}
		pr_info("%s(on): success\n", __func__);
	} else {
		gpio_free(170);
		gpio_free(171);
		gpio_free(172);
		rc = regulator_disable(reg_8058_l16);
		if (rc)
			pr_warning("'%s' regulator disable failed, rc=%d\n",
				"8058_l16", rc);
		pr_info("%s(off): success\n", __func__);
	}

	prev_on = on;

	return 0;

error3:
	gpio_free(171);
error2:
	gpio_free(170);
error1:
	regulator_disable(reg_8058_l16);
	return rc;
}

static int hdmi_cec_power(int on)
{
	static struct regulator *reg_8901_l3;		/* HDMI_CEC */
	static int prev_on;
	int rc;

	if (on == prev_on)
		return 0;

	if (!reg_8901_l3)
		_GET_REGULATOR(reg_8901_l3, "8901_l3");

	if (on) {
		rc = regulator_set_voltage(reg_8901_l3, 3300000, 3300000);
		if (!rc)
			rc = regulator_enable(reg_8901_l3);
		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
				"8901_l3", rc);
			return rc;
		}
		rc = gpio_request(169, "HDMI_CEC_VAR");
		if (rc) {
			pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
				"HDMI_CEC_VAR", 169, rc);
			goto error;
		}
		pr_info("%s(on): success\n", __func__);
	} else {
		gpio_free(169);
		rc = regulator_disable(reg_8901_l3);
		if (rc)
			pr_warning("'%s' regulator disable failed, rc=%d\n",
				"8901_l3", rc);
		pr_info("%s(off): success\n", __func__);
	}

	prev_on = on;

	return 0;
error:
	regulator_disable(reg_8901_l3);
	return rc;
}
#undef _GET_REGULATOR

#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */

#define GPIO_DAC_PWR_SAV 104
static int msm_fb_lcdc_gpio_config(int on)
{
	if (vga_enabled) {
		/* VGA DAC power save */
		gpio_set_value_cansleep(GPIO_DAC_PWR_SAV, !on);
		/* backlight */
		gpio_set_value_cansleep(30, 0);
	} else {
		/* VGA DAC power save */
		gpio_set_value_cansleep(GPIO_DAC_PWR_SAV, 1);
		/* backlight */
		gpio_set_value_cansleep(30, !!on);
	}

	return 0;
}

static int lcdc_panel_power(int on)
{
	int flag_on = !!on;
	static int lcdc_power_save_on;

	if (lcdc_power_save_on == flag_on)
		return 0;

	lcdc_power_save_on = flag_on;

	lcd_panel_power(on);

	return 0;
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

static struct msm_bus_vectors mdp_sd_smi_vectors[] = {
	/* Default case static display/UI/2d/3d if FB SMI */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 147460000,
		.ib = 184325000,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors mdp_sd_ebi_vectors[] = {
	/* Default case static display/UI/2d/3d if FB SMI */
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
		.ab = 334080000,
		.ib = 417600000,
	},
};
static struct msm_bus_vectors mdp_vga_vectors[] = {
	/* VGA and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 175110000,
		.ib = 218887500,
	},
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 175110000,
		.ib = 218887500,
	},
};

static struct msm_bus_vectors mdp_720p_vectors[] = {
	/* 720p and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 230400000,
		.ib = 288000000,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 230400000,
		.ib = 288000000,
	},
};

static struct msm_bus_vectors mdp_1080p_vectors[] = {
	/* 1080p and less video */
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
static struct msm_bus_paths mdp_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(mdp_init_vectors),
		mdp_init_vectors,
	},
	{
		ARRAY_SIZE(mdp_sd_smi_vectors),
		mdp_sd_smi_vectors,
	},
	{
		ARRAY_SIZE(mdp_sd_ebi_vectors),
		mdp_sd_ebi_vectors,
	},
	{
		ARRAY_SIZE(mdp_vga_vectors),
		mdp_vga_vectors,
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
		.ab = 435456000,
		.ib = 544320000,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 435456000,
		.ib = 544320000,
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
};

static struct lcdc_platform_data dtv_pdata = {
	.bus_scale_table = &dtv_bus_scale_pdata,
};
#endif


static struct lcdc_platform_data lcdc_pdata = {
	.lcdc_power_save   = lcdc_panel_power,
	.lcdc_gpio_config  = msm_fb_lcdc_gpio_config,
};

static int mdp_core_clk_rate_table[] = {
	59080000,
	128000000,
	160000000,
	200000000,
};
static struct msm_panel_common_pdata mdp_pdata = {
	.mdp_core_clk_rate = 200000000,
	.mdp_core_clk_table = mdp_core_clk_rate_table,
	.num_mdp_clk = ARRAY_SIZE(mdp_core_clk_rate_table),
#ifdef CONFIG_MSM_BUS_SCALING
	.mdp_bus_scale_table = &mdp_bus_scale_pdata,
#endif
};

static void __init msm_fb_add_devices(void)
{
	gpio_request(GPIO_DAC_PWR_SAV, "DAC_PWR_SAV");
	gpio_direction_output(GPIO_DAC_PWR_SAV, 1);
	gpio_request(30, "BACKLIGHT_EN");
	gpio_direction_output(30, 0);
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("lcdc", &lcdc_pdata);
	msm_fb_register_device("mipi_dsi", 0);
#ifdef CONFIG_MSM_BUS_SCALING
	msm_fb_register_device("dtv", &dtv_pdata);
#endif
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

static uint32_t auxpcm_gpio_table[] = {
	GPIO_CFG(111, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(112, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(113, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(114, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static void msm_auxpcm_init(void)
{
	gpio_tlmm_config(auxpcm_gpio_table[0], GPIO_CFG_ENABLE);
	gpio_tlmm_config(auxpcm_gpio_table[1], GPIO_CFG_ENABLE);
	gpio_tlmm_config(auxpcm_gpio_table[2], GPIO_CFG_ENABLE);
	gpio_tlmm_config(auxpcm_gpio_table[3], GPIO_CFG_ENABLE);
}

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT

#define WLAN_PWDN_N_GPIO	118

static void enable_wlan_bt(void)
{
	int rc = 0;

	rc = gpio_request(WLAN_PWDN_N_GPIO, "WLAN_PWDN_N");
	if (rc) {
		pr_err("%s: WLAN_PWDN_N gpio %d request failed: %d\n",
			__func__, WLAN_PWDN_N_GPIO, rc);
		return;
	}

	rc = gpio_direction_output(WLAN_PWDN_N_GPIO, 0);
	if (rc) {
		pr_err("%s: gpio_direction_output %d failed: %d\n",
			__func__, WLAN_PWDN_N_GPIO, rc);
		gpio_free(WLAN_PWDN_N_GPIO);
		return;
	}

	gpio_set_value(WLAN_PWDN_N_GPIO, 1);
	usleep(5);
	gpio_set_value(WLAN_PWDN_N_GPIO, 0);
	usleep(5);
	gpio_set_value(WLAN_PWDN_N_GPIO, 1);
}
#endif

struct msm_board_data {
	struct msm_gpiomux_configs *gpiomux_cfgs;
};

static struct msm_board_data msm8x60_qrdc_board_data __initdata = {
	.gpiomux_cfgs = msm8x60_qrdc_gpiomux_cfgs,
};

static void __init msm8x60_init(struct msm_board_data *board_data)
{
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


	/* initialize SPM before acpuclock as the latter calls into SPM
	 * driver to set ACPU voltages.
	 */
	msm_spm_init(msm_spm_data, ARRAY_SIZE(msm_spm_data));

	/*
	 * Disable regulator info printing so that regulator registration
	 * messages do not enter the kmsg log.
	 */
	regulator_suppress_info_printing();

	/* Initialize regulators needed for clock_init. */
	platform_add_devices(early_regulators, ARRAY_SIZE(early_regulators));

	msm_clock_init(msm_clocks_8x60, msm_num_clocks_8x60);

	/* Buses need to be initialized before early-device registration
	 * to get the platform data for fabrics.
	 */
	msm8x60_init_buses();
	platform_add_devices(early_devices, ARRAY_SIZE(early_devices));
	msm_acpu_clock_init(&msm8x60_acpu_clock_data);

	msm8x60_init_ebi2();
	msm8x60_init_tlmm();
	msm8x60_init_gpiomux(board_data->gpiomux_cfgs);
	msm8x60_init_uart12dm();
	msm8x60_init_mmc();
	msm8x60_cfg_smsc911x();
	if (SOCINFO_VERSION_MAJOR(socinfo_get_version()) != 1)
		platform_add_devices(msm_footswitch_devices,
				     msm_num_footswitch_devices);
	platform_add_devices(qrdc_devices,
			     ARRAY_SIZE(qrdc_devices));
	wifi_power(1);
#ifdef CONFIG_USB_EHCI_MSM
	msm_add_host(0, &msm_usb_host_pdata);
#endif
	msm_fb_add_devices();
	fixup_i2c_configs();
	register_i2c_devices();

	msm_pm_set_platform_data(msm_pm_data, ARRAY_SIZE(msm_pm_data));
	msm_cpuidle_set_states(msm_cstates, ARRAY_SIZE(msm_cstates),
				msm_pm_data);

	msm_auxpcm_init();

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	enable_wlan_bt();
#endif
}

static void __init msm8x60_qrdc_init(void)
{
	msm8x60_init(&msm8x60_qrdc_board_data);
}

MACHINE_START(MSM8X60_QRDC, "QCT MSM8X60 QRDC")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.map_io = msm8x60_map_io,
	.init_irq = msm8x60_init_irq,
	.init_machine = msm8x60_qrdc_init,
	.timer = &msm_timer,
MACHINE_END
