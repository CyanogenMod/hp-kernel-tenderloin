/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __LINUX_USB_GADGET_MSM72K_OTG_H__
#define __LINUX_USB_GADGET_MSM72K_OTG_H__

#include <linux/usb.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>
#include <linux/wakelock.h>

#include <asm/mach-types.h>
#include <mach/msm_hsusb.h>

#define OTGSC_BSVIE            (1 << 27)
#define OTGSC_IDIE             (1 << 24)
#define OTGSC_BSVIS            (1 << 19)
#define OTGSC_ID               (1 << 8)
#define OTGSC_IDIS             (1 << 16)
#define OTGSC_BSV              (1 << 11)
#define OTGSC_DPIE             (1 << 30)
#define OTGSC_DPIS             (1 << 22)
#define OTGSC_HADP             (1 << 6)

#define ULPI_STP_CTRL   (1 << 30)
#define ASYNC_INTR_CTRL (1 << 29)

#define PORTSC_PHCD     (1 << 23)
#define PORTSC_CSC	(1 << 1)
#define disable_phy_clk() (writel(readl(USB_PORTSC) | PORTSC_PHCD, USB_PORTSC))
#define enable_phy_clk() (writel(readl(USB_PORTSC) & ~PORTSC_PHCD, USB_PORTSC))
#define is_phy_clk_disabled() (readl(USB_PORTSC) & PORTSC_PHCD)
#define is_usb_active()       (!(readl(USB_PORTSC) & PORTSC_SUSP))

/* Timeout (in msec) values (min - max) associated with OTG timers */

#define TA_WAIT_VRISE	100	/* ( - 100)  */
#define TA_WAIT_VFALL	500	/* ( - 1000) */
#define TA_WAIT_BCON	30000	/* (1100 - 30000) */
/* AIDL_BDIS should be 500 */
#define TA_AIDL_BDIS	200	/* (200 - ) */
#define TA_BIDL_ADIS	155	/* (155 - 200) */
#define TB_SRP_FAIL	6000	/* (5000 - 6000) */
#define TB_ASE0_BRST	155	/* (155 - ) */

/* TB_SSEND_SRP and TB_SE0_SRP are combined */
#define TB_SRP_INIT	2000	/* (1500 - ) */

/* Timeout variables */

#define A_WAIT_VRISE	0
#define A_WAIT_VFALL	1
#define A_WAIT_BCON	2
#define A_AIDL_BDIS	3
#define A_BIDL_ADIS	4
#define B_SRP_FAIL	5
#define B_ASE0_BRST	6

/* Internal flags like a_set_b_hnp_en, b_hnp_en are maintained
 * in usb_bus and usb_gadget
 */

#define A_BUS_DROP		0
#define A_BUS_REQ		1
#define A_SRP_DET		2
#define A_VBUS_VLD		3
#define B_CONN			4
#define ID			5
#define ADP_CHANGE		6
#define POWER_UP		7
#define A_CLR_ERR		8
#define A_BUS_RESUME		9
#define A_BUS_SUSPEND		10
#define A_CONN			11
#define B_BUS_REQ		12
#define B_SESS_VLD		13
#define ID_A			14
#define ID_B			15
#define ID_C			16

#define USB_IDCHG_MIN	500
#define USB_IDCHG_MAX	1500
#define USB_IB_UNCFG	2
#define OTG_ID_POLL_MS	1000

struct msm_otg {
	struct otg_transceiver otg;

	/* usb clocks */
	struct clk		*hs_clk;
	struct clk		*hs_pclk;
	struct clk		*hs_cclk;
	/* clk regime has created dummy clock id for phy so
	 * that generic clk_reset api can be used to reset phy
	 */
	struct clk		*phy_reset_clk;

	int			irq;
	int			vbus_on_irq;
	void __iomem		*regs;
	atomic_t		in_lpm;
	/* charger-type is modified by gadget for legacy chargers
	 * and OTG modifies it for ACA
	 */
	atomic_t 		chg_type;

	void (*start_host)	(struct usb_bus *bus, int suspend);
	/* Enable/disable the clocks */
	int (*set_clk)		(struct otg_transceiver *otg, int on);
	/* Reset phy and link */
	void (*reset)		(struct otg_transceiver *otg, int phy_reset);
	/* pmic notfications apis */
	u8 pmic_notif_supp;
	struct msm_otg_platform_data *pdata;

	spinlock_t lock; /* protects OTG state */
	struct wake_lock wlock;
	unsigned long b_last_se0_sess; /* SRP initial condition check */
	unsigned long inputs;
	unsigned long tmouts;
	u8 active_tmout;
	struct hrtimer timer;
	struct workqueue_struct *wq;
	struct work_struct sm_work; /* state machine work */
	struct work_struct otg_resume_work;
	struct notifier_block usbdev_nb;
#ifdef CONFIG_USB_MSM_ACA
	struct timer_list	id_timer;	/* drives id_status polling */
	unsigned		b_max_power;	/* ACA: max power of accessory*/
#endif
};

/* usb controller's protocol engine depends on AXI clock.
 * On some platforms this dependency is removed by
 * introducing usb core clock
 */
static inline int depends_on_axi_freq(struct otg_transceiver *xceiv)
{
	struct msm_otg *dev;

	if (!xceiv)
		return 0;

	/* for 8660 usb core is in sps and at the same time it does not
	 * have core clock
	 */
	if (machine_is_msm8x60_surf() || machine_is_msm8x60_ffa())
		return 0;

	dev = container_of(xceiv, struct msm_otg, otg);

	return !dev->pdata->core_clk;
}

#endif
