/* linux/include/mach/hsusb.h
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
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

#ifndef __ASM_ARCH_MSM_HSUSB_H
#define __ASM_ARCH_MSM_HSUSB_H

#include <linux/types.h>

#define PHY_TYPE_MASK		0x0F
#define PHY_TYPE_MODE		0xF0
#define PHY_MODEL_MASK		0xFF00
#define PHY_TYPE(x)		((x) & PHY_TYPE_MASK)
#define PHY_MODEL(x)		((x) & PHY_MODEL_MASK)

#define USB_PHY_MODEL_65NM	0x100
#define USB_PHY_MODEL_180NM	0x200
#define USB_PHY_MODEL_45NM	0x400
#define USB_PHY_UNDEFINED	0x00
#define USB_PHY_INTEGRATED	0x01
#define USB_PHY_EXTERNAL	0x02
#define USB_PHY_SERIAL_PMIC     0x04

#define REQUEST_STOP		0
#define REQUEST_START		1
#define REQUEST_RESUME		2
#define REQUEST_HNP_SUSPEND	3
#define REQUEST_HNP_RESUME	4

/* Flags required to read ID state of PHY for ACA */
#define PHY_ID_MASK		0xB0
#define PHY_ID_GND		0
#define PHY_ID_C		0x10
#define PHY_ID_B		0x30
#define PHY_ID_A		0x90

#define phy_id_state(ints)	((ints) & PHY_ID_MASK)
#define phy_id_state_a(ints)	(phy_id_state((ints)) == PHY_ID_A)
#define phy_id_state_b(ints)	(phy_id_state((ints)) == PHY_ID_B)
#define phy_id_state_c(ints)	(phy_id_state((ints)) == PHY_ID_C)
#define phy_id_state_gnd(ints)	(phy_id_state((ints)) == PHY_ID_GND)

enum hsusb_phy_type {
	UNDEFINED,
	INTEGRATED,
	EXTERNAL,
};
/* used to detect the OTG Mode */
enum otg_mode {
	OTG_ID = 0,   		/* ID pin detection */
	OTG_USER_CONTROL,  	/* User configurable */
	OTG_VCHG,     		/* Based on VCHG interrupt */
};

struct usb_function_map {
	char name[20];
	unsigned bit_pos;
};

#ifdef CONFIG_USB_FUNCTION
/* platform device data for msm_hsusb driver */
struct usb_composition {
	__u16   product_id;
	unsigned long functions;
};
#endif

#ifdef CONFIG_USB_GADGET_MSM_72K
enum chg_type {
	USB_CHG_TYPE__SDP,
	USB_CHG_TYPE__CARKIT,
	USB_CHG_TYPE__WALLCHARGER,
	USB_CHG_TYPE__INVALID
};
#endif

enum pre_emphasis_level {
	PRE_EMPHASIS_DEFAULT,
	PRE_EMPHASIS_DISABLE,
	PRE_EMPHASIS_WITH_10_PERCENT = (1 << 5),
	PRE_EMPHASIS_WITH_20_PERCENT = (3 << 4),
};
enum cdr_auto_reset {
	CDR_AUTO_RESET_DEFAULT,
	CDR_AUTO_RESET_ENABLE,
	CDR_AUTO_RESET_DISABLE,
};
enum hs_drv_amplitude {
	HS_DRV_AMPLITUDE_DEFAULT,
	HS_DRV_AMPLITUDE_ZERO_PERCENT,
	HS_DRV_AMPLITUDE_25_PERCENTI = (1 << 2),
	HS_DRV_AMPLITUDE_5_PERCENT = (1 << 3),
	HS_DRV_AMPLITUDE_75_PERCENT = (3 << 2),
};

struct msm_hsusb_gadget_platform_data {
	int *phy_init_seq;
	void (*phy_reset)(void);

	int self_powered;
};

struct msm_hsusb_platform_data {
	__u16   version;
	unsigned phy_info;
	__u16   vendor_id;
	char   	*product_name;
	char   	*serial_number;
	char   	*manufacturer_name;
	struct usb_composition *compositions;
	int num_compositions;
	struct usb_function_map *function_map;
	int num_functions;
	/* gpio mux function used for LPM */
	int (*config_gpio)(int config);
	/* ROC info for AHB Mode */
	unsigned int soc_version;

	int (*phy_reset)(void __iomem *addr);

	unsigned int core_clk;

	int vreg5v_required;

	u32 swfi_latency;
};

struct msm_otg_platform_data {
	int (*rpc_connect)(int);
	int (*phy_reset)(void __iomem *);
	unsigned int core_clk;
	int pmic_vbus_irq;
	/* if usb link is in sps there is no need for
	 * usb pclk as dayatona fabric clock will be
	 * used instead
	 */
	int usb_in_sps;
	enum pre_emphasis_level	pemp_level;
	enum cdr_auto_reset	cdr_autoreset;
	enum hs_drv_amplitude	drv_ampl;
	int			phy_reset_sig_inverted;

	u32 			swfi_latency;
	/* pmic notfications apis */
	int (*pmic_notif_init) (void);
	void (*pmic_notif_deinit) (void);
	int (*pmic_register_vbus_sn) (void (*callback)(int online));
	void (*pmic_unregister_vbus_sn) (void (*callback)(int online));
	int (*pmic_enable_ldo) (int);
	void (*setup_gpio)(unsigned int config);
	u8      otg_mode;
	void (*vbus_power) (unsigned phy_info, int on);

	/* charger notification apis */
	void (*chg_connected)(enum chg_type chg_type);
	void (*chg_vbus_draw)(unsigned ma);
	int  (*chg_init)(int init);
};

struct msm_usb_host_platform_data {
	unsigned phy_info;
	unsigned int power_budget;
	void (*config_gpio)(unsigned int config);
	void (*vbus_power) (unsigned phy_info, int on);
	int  (*vbus_init)(int init);
};

#endif
