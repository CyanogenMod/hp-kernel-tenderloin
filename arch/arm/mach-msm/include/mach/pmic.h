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

#ifndef __ARCH_ARM_MACH_PMIC_H
#define __ARCH_ARM_MACH_PMIC_H

#include <linux/types.h>

enum spkr_left_right {
	LEFT_SPKR,
	RIGHT_SPKR,
};

enum spkr_gain {
	SPKR_GAIN_MINUS16DB,      /* -16 db */
	SPKR_GAIN_MINUS12DB,      /* -12 db */
	SPKR_GAIN_MINUS08DB,      /* -08 db */
	SPKR_GAIN_MINUS04DB,      /* -04 db */
	SPKR_GAIN_00DB,           /*  00 db */
	SPKR_GAIN_PLUS04DB,       /* +04 db */
	SPKR_GAIN_PLUS08DB,       /* +08 db */
	SPKR_GAIN_PLUS12DB,       /* +12 db */
};

enum spkr_dly {
	SPKR_DLY_10MS,            /* ~10  ms delay */
	SPKR_DLY_100MS,           /* ~100 ms delay */
};

enum spkr_hpf_corner_freq {
	SPKR_FREQ_1_39KHZ,         /* 1.39 kHz */
	SPKR_FREQ_0_64KHZ,         /* 0.64 kHz */
	SPKR_FREQ_0_86KHZ,         /* 0.86 kHz */
	SPKR_FREQ_0_51KHZ,         /* 0.51 kHz */
	SPKR_FREQ_1_06KHZ,         /* 1.06 kHz */
	SPKR_FREQ_0_57KHZ,         /* 0.57 kHz */
	SPKR_FREQ_0_73KHZ,         /* 0.73 kHz */
	SPKR_FREQ_0_47KHZ,         /* 0.47 kHz */
	SPKR_FREQ_1_20KHZ,         /* 1.20 kHz */
	SPKR_FREQ_0_60KHZ,         /* 0.60 kHz */
	SPKR_FREQ_0_76KHZ,         /* 0.76 kHz */
	SPKR_FREQ_0_49KHZ,         /* 0.49 kHz */
	SPKR_FREQ_0_95KHZ,         /* 0.95 kHz */
	SPKR_FREQ_0_54KHZ,         /* 0.54 kHz */
	SPKR_FREQ_0_68KHZ,         /* 0.68 kHz */
	SPKR_FREQ_0_45KHZ,         /* 0.45 kHz */
};

/* Turn the speaker on or off and enables or disables mute.*/
enum spkr_cmd {
	SPKR_DISABLE,  /* Enable Speaker */
	SPKR_ENABLE,   /* Disable Speaker */
	SPKR_MUTE_OFF, /* turn speaker mute off, SOUND ON */
	SPKR_MUTE_ON,  /* turn speaker mute on, SOUND OFF */
	SPKR_OFF,      /* turn speaker OFF (speaker disable and mute on) */
	SPKR_ON,        /* turn speaker ON (speaker enable and mute off)  */
	SPKR_SET_FREQ_CMD,    /* set speaker frequency */
	SPKR_GET_FREQ_CMD,    /* get speaker frequency */
	SPKR_SET_GAIN_CMD,    /* set speaker gain */
	SPKR_GET_GAIN_CMD,    /* get speaker gain */
	SPKR_SET_DELAY_CMD,   /* set speaker delay */
	SPKR_GET_DELAY_CMD,   /* get speaker delay */
	SPKR_SET_PDM_MODE,
	SPKR_SET_PWM_MODE,
};

struct spkr_config_mode {
	uint32_t is_right_chan_en;
	uint32_t is_left_chan_en;
	uint32_t is_right_left_chan_added;
	uint32_t is_stereo_en;
	uint32_t is_usb_with_hpf_20hz;
	uint32_t is_mux_bypassed;
	uint32_t is_hpf_en;
	uint32_t is_sink_curr_from_ref_volt_cir_en;
};

enum mic_volt {
	MIC_VOLT_2_00V,            /*  2.00 V  */
	MIC_VOLT_1_93V,            /*  1.93 V  */
	MIC_VOLT_1_80V,            /*  1.80 V  */
	MIC_VOLT_1_73V,            /*  1.73 V  */
};

enum ledtype {
	LED_LCD,
	LED_KEYPAD,
};

enum flash_led_mode {
	FLASH_LED_MODE__MANUAL,
	FLASH_LED_MODE__DBUS1,
	FLASH_LED_MODE__DBUS2,
	FLASH_LED_MODE__DBUS3,
};

enum flash_led_pol {
	FLASH_LED_POL__ACTIVE_HIGH,
	FLASH_LED_POL__ACTIVE_LOW,
};

enum switch_cmd {
	OFF_CMD,
	ON_CMD
};

enum vreg_lp_id {
	PM_VREG_LP_MSMA_ID,
	PM_VREG_LP_MSMP_ID,
	PM_VREG_LP_MSME1_ID,
	PM_VREG_LP_GP3_ID,
	PM_VREG_LP_MSMC_ID,
	PM_VREG_LP_MSME2_ID,
	PM_VREG_LP_GP4_ID,
	PM_VREG_LP_GP1_ID,
	PM_VREG_LP_RFTX_ID,
	PM_VREG_LP_RFRX1_ID,
	PM_VREG_LP_RFRX2_ID,
	PM_VREG_LP_WLAN_ID,
	PM_VREG_LP_MMC_ID,
	PM_VREG_LP_RUIM_ID,
	PM_VREG_LP_MSMC0_ID,
	PM_VREG_LP_GP2_ID,
	PM_VREG_LP_GP5_ID,
	PM_VREG_LP_GP6_ID,
	PM_VREG_LP_MPLL_ID,
	PM_VREG_LP_RFUBM_ID,
	PM_VREG_LP_RFA_ID,
	PM_VREG_LP_CDC2_ID,
	PM_VREG_LP_RFTX2_ID,
	PM_VREG_LP_USIM_ID,
	PM_VREG_LP_USB2P6_ID,
	PM_VREG_LP_TCXO_ID,
	PM_VREG_LP_USB3P3_ID,

	PM_VREG_LP_MSME_ID = PM_VREG_LP_MSME1_ID,
	/* backward compatible enums only */
	PM_VREG_LP_CAM_ID = PM_VREG_LP_GP1_ID,
	PM_VREG_LP_MDDI_ID = PM_VREG_LP_GP2_ID,
	PM_VREG_LP_RUIM2_ID = PM_VREG_LP_GP3_ID,
	PM_VREG_LP_AUX_ID = PM_VREG_LP_GP4_ID,
	PM_VREG_LP_AUX2_ID = PM_VREG_LP_GP5_ID,
	PM_VREG_LP_BT_ID = PM_VREG_LP_GP6_ID,
	PM_VREG_LP_MSMC_LDO_ID = PM_VREG_LP_MSMC_ID,
	PM_VREG_LP_MSME1_LDO_ID = PM_VREG_LP_MSME1_ID,
	PM_VREG_LP_MSME2_LDO_ID = PM_VREG_LP_MSME2_ID,
	PM_VREG_LP_RFA1_ID = PM_VREG_LP_RFRX2_ID,
	PM_VREG_LP_RFA2_ID = PM_VREG_LP_RFTX2_ID,
	PM_VREG_LP_XO_ID = PM_VREG_LP_TCXO_ID
};

enum vreg_id {
	PM_VREG_MSMA_ID = 0,
	PM_VREG_MSMP_ID,
	PM_VREG_MSME1_ID,
	PM_VREG_MSMC1_ID,
	PM_VREG_MSMC2_ID,
	PM_VREG_GP3_ID,
	PM_VREG_MSME2_ID,
	PM_VREG_GP4_ID,
	PM_VREG_GP1_ID,
	PM_VREG_TCXO_ID,
	PM_VREG_PA_ID,
	PM_VREG_RFTX_ID,
	PM_VREG_RFRX1_ID,
	PM_VREG_RFRX2_ID,
	PM_VREG_SYNT_ID,
	PM_VREG_WLAN_ID,
	PM_VREG_USB_ID,
	PM_VREG_BOOST_ID,
	PM_VREG_MMC_ID,
	PM_VREG_RUIM_ID,
	PM_VREG_MSMC0_ID,
	PM_VREG_GP2_ID,
	PM_VREG_GP5_ID,
	PM_VREG_GP6_ID,
	PM_VREG_RF_ID,
	PM_VREG_RF_VCO_ID,
	PM_VREG_MPLL_ID,
	PM_VREG_S2_ID,
	PM_VREG_S3_ID,
	PM_VREG_RFUBM_ID,
	PM_VREG_NCP_ID,
	PM_VREG_RF2_ID,
	PM_VREG_RFA_ID,
	PM_VREG_CDC2_ID,
	PM_VREG_RFTX2_ID,
	PM_VREG_USIM_ID,
	PM_VREG_USB2P6_ID,
	PM_VREG_USB3P3_ID,
	PM_VREG_EXTCDC1_ID,
	PM_VREG_EXTCDC2_ID,

	/* backward compatible enums only */
	PM_VREG_MSME_ID = PM_VREG_MSME1_ID,
	PM_VREG_MSME_BUCK_SMPS_ID = PM_VREG_MSME1_ID,
	PM_VREG_MSME1_LDO_ID = PM_VREG_MSME1_ID,
	PM_VREG_MSMC_ID = PM_VREG_MSMC1_ID,
	PM_VREG_MSMC_LDO_ID = PM_VREG_MSMC1_ID,
	PM_VREG_MSMC1_BUCK_SMPS_ID = PM_VREG_MSMC1_ID,
	PM_VREG_MSME2_LDO_ID = PM_VREG_MSME2_ID,
	PM_VREG_CAM_ID = PM_VREG_GP1_ID,
	PM_VREG_MDDI_ID = PM_VREG_GP2_ID,
	PM_VREG_RUIM2_ID = PM_VREG_GP3_ID,
	PM_VREG_AUX_ID = PM_VREG_GP4_ID,
	PM_VREG_AUX2_ID = PM_VREG_GP5_ID,
	PM_VREG_BT_ID = PM_VREG_GP6_ID,
	PM_VREG_RF1_ID = PM_VREG_RF_ID,
	PM_VREG_S1_ID = PM_VREG_RF1_ID,
	PM_VREG_5V_ID = PM_VREG_BOOST_ID,
	PM_VREG_RFA1_ID = PM_VREG_RFRX2_ID,
	PM_VREG_RFA2_ID = PM_VREG_RFTX2_ID,
	PM_VREG_XO_ID = PM_VREG_TCXO_ID
};

enum vreg_pdown_id {
	PM_VREG_PDOWN_MSMA_ID,
	PM_VREG_PDOWN_MSMP_ID,
	PM_VREG_PDOWN_MSME1_ID,
	PM_VREG_PDOWN_MSMC1_ID,
	PM_VREG_PDOWN_MSMC2_ID,
	PM_VREG_PDOWN_GP3_ID,
	PM_VREG_PDOWN_MSME2_ID,
	PM_VREG_PDOWN_GP4_ID,
	PM_VREG_PDOWN_GP1_ID,
	PM_VREG_PDOWN_TCXO_ID,
	PM_VREG_PDOWN_PA_ID,
	PM_VREG_PDOWN_RFTX_ID,
	PM_VREG_PDOWN_RFRX1_ID,
	PM_VREG_PDOWN_RFRX2_ID,
	PM_VREG_PDOWN_SYNT_ID,
	PM_VREG_PDOWN_WLAN_ID,
	PM_VREG_PDOWN_USB_ID,
	PM_VREG_PDOWN_MMC_ID,
	PM_VREG_PDOWN_RUIM_ID,
	PM_VREG_PDOWN_MSMC0_ID,
	PM_VREG_PDOWN_GP2_ID,
	PM_VREG_PDOWN_GP5_ID,
	PM_VREG_PDOWN_GP6_ID,
	PM_VREG_PDOWN_RF_ID,
	PM_VREG_PDOWN_RF_VCO_ID,
	PM_VREG_PDOWN_MPLL_ID,
	PM_VREG_PDOWN_S2_ID,
	PM_VREG_PDOWN_S3_ID,
	PM_VREG_PDOWN_RFUBM_ID,
	/* new for HAN */
	PM_VREG_PDOWN_RF1_ID,
	PM_VREG_PDOWN_RF2_ID,
	PM_VREG_PDOWN_RFA_ID,
	PM_VREG_PDOWN_CDC2_ID,
	PM_VREG_PDOWN_RFTX2_ID,
	PM_VREG_PDOWN_USIM_ID,
	PM_VREG_PDOWN_USB2P6_ID,
	PM_VREG_PDOWN_USB3P3_ID,

	/* backward compatible enums only */
	PM_VREG_PDOWN_CAM_ID = PM_VREG_PDOWN_GP1_ID,
	PM_VREG_PDOWN_MDDI_ID = PM_VREG_PDOWN_GP2_ID,
	PM_VREG_PDOWN_RUIM2_ID = PM_VREG_PDOWN_GP3_ID,
	PM_VREG_PDOWN_AUX_ID = PM_VREG_PDOWN_GP4_ID,
	PM_VREG_PDOWN_AUX2_ID = PM_VREG_PDOWN_GP5_ID,
	PM_VREG_PDOWN_BT_ID = PM_VREG_PDOWN_GP6_ID,
	PM_VREG_PDOWN_MSME_ID = PM_VREG_PDOWN_MSME1_ID,
	PM_VREG_PDOWN_MSMC_ID = PM_VREG_PDOWN_MSMC1_ID,
	PM_VREG_PDOWN_RFA1_ID = PM_VREG_PDOWN_RFRX2_ID,
	PM_VREG_PDOWN_RFA2_ID = PM_VREG_PDOWN_RFTX2_ID,
	PM_VREG_PDOWN_XO_ID = PM_VREG_PDOWN_TCXO_ID
};

enum mpp_which {
	PM_MPP_1,
	PM_MPP_2,
	PM_MPP_3,
	PM_MPP_4,
	PM_MPP_5,
	PM_MPP_6,
	PM_MPP_7,
	PM_MPP_8,
	PM_MPP_9,
	PM_MPP_10,
	PM_MPP_11,
	PM_MPP_12,
	PM_MPP_13,
	PM_MPP_14,
	PM_MPP_15,
	PM_MPP_16,
	PM_MPP_17,
	PM_MPP_18,
	PM_MPP_19,
	PM_MPP_20,
	PM_MPP_21,
	PM_MPP_22,

	PM_NUM_MPP_HAN = PM_MPP_4 + 1,
	PM_NUM_MPP_KIP = PM_MPP_4 + 1,
	PM_NUM_MPP_EPIC = PM_MPP_4 + 1,
	PM_NUM_MPP_PM7500 = PM_MPP_22 + 1,
	PM_NUM_MPP_PM6650 = PM_MPP_12 + 1,
	PM_NUM_MPP_PM6658 = PM_MPP_12 + 1,
	PM_NUM_MPP_PANORAMIX = PM_MPP_2 + 1,
	PM_NUM_MPP_PM6640 = PM_NUM_MPP_PANORAMIX,
	PM_NUM_MPP_PM6620 = PM_NUM_MPP_PANORAMIX
};

enum mpp_dlogic_level {
	PM_MPP__DLOGIC__LVL_MSME,
	PM_MPP__DLOGIC__LVL_MSMP,
	PM_MPP__DLOGIC__LVL_RUIM,
	PM_MPP__DLOGIC__LVL_MMC,
	PM_MPP__DLOGIC__LVL_VDD,
};

enum mpp_dlogic_in_dbus {
	PM_MPP__DLOGIC_IN__DBUS_NONE,
	PM_MPP__DLOGIC_IN__DBUS1,
	PM_MPP__DLOGIC_IN__DBUS2,
	PM_MPP__DLOGIC_IN__DBUS3,
};

enum mpp_dlogic_out_ctrl {
	PM_MPP__DLOGIC_OUT__CTRL_LOW,
	PM_MPP__DLOGIC_OUT__CTRL_HIGH,
	PM_MPP__DLOGIC_OUT__CTRL_MPP,
	PM_MPP__DLOGIC_OUT__CTRL_NOT_MPP,
};

enum mpp_i_sink_level {
	PM_MPP__I_SINK__LEVEL_5mA,
	PM_MPP__I_SINK__LEVEL_10mA,
	PM_MPP__I_SINK__LEVEL_15mA,
	PM_MPP__I_SINK__LEVEL_20mA,
	PM_MPP__I_SINK__LEVEL_25mA,
	PM_MPP__I_SINK__LEVEL_30mA,
	PM_MPP__I_SINK__LEVEL_35mA,
	PM_MPP__I_SINK__LEVEL_40mA,
};

enum mpp_i_sink_switch {
	PM_MPP__I_SINK__SWITCH_DIS,
	PM_MPP__I_SINK__SWITCH_ENA,
	PM_MPP__I_SINK__SWITCH_ENA_IF_MPP_HIGH,
	PM_MPP__I_SINK__SWITCH_ENA_IF_MPP_LOW,
};

enum pm_vib_mot_mode {
	PM_VIB_MOT_MODE__MANUAL,
	PM_VIB_MOT_MODE__DBUS1,
	PM_VIB_MOT_MODE__DBUS2,
	PM_VIB_MOT_MODE__DBUS3,
};

enum pm_vib_mot_pol {
	PM_VIB_MOT_POL__ACTIVE_HIGH,
	PM_VIB_MOT_POL__ACTIVE_LOW,
};

struct rtc_time {
	uint  sec;
};

enum rtc_alarm {
	PM_RTC_ALARM_1,
};

enum hsed_controller {
	PM_HSED_CONTROLLER_0,
	PM_HSED_CONTROLLER_1,
	PM_HSED_CONTROLLER_2,
};

enum hsed_switch {
	PM_HSED_SC_SWITCH_TYPE,
	PM_HSED_OC_SWITCH_TYPE,
};

enum hsed_enable {
	PM_HSED_ENABLE_OFF,
	PM_HSED_ENABLE_TCXO,
	PM_HSED_ENABLE_PWM_TCXO,
	PM_HSED_ENABLE_ALWAYS,
};

enum hsed_hyst_pre_div {
	PM_HSED_HYST_PRE_DIV_1,
	PM_HSED_HYST_PRE_DIV_2,
	PM_HSED_HYST_PRE_DIV_4,
	PM_HSED_HYST_PRE_DIV_8,
	PM_HSED_HYST_PRE_DIV_16,
	PM_HSED_HYST_PRE_DIV_32,
	PM_HSED_HYST_PRE_DIV_64,
	PM_HSED_HYST_PRE_DIV_128,
};

enum hsed_hyst_time {
	PM_HSED_HYST_TIME_1_CLK_CYCLES,
	PM_HSED_HYST_TIME_2_CLK_CYCLES,
	PM_HSED_HYST_TIME_3_CLK_CYCLES,
	PM_HSED_HYST_TIME_4_CLK_CYCLES,
	PM_HSED_HYST_TIME_5_CLK_CYCLES,
	PM_HSED_HYST_TIME_6_CLK_CYCLES,
	PM_HSED_HYST_TIME_7_CLK_CYCLES,
	PM_HSED_HYST_TIME_8_CLK_CYCLES,
	PM_HSED_HYST_TIME_9_CLK_CYCLES,
	PM_HSED_HYST_TIME_10_CLK_CYCLES,
	PM_HSED_HYST_TIME_11_CLK_CYCLES,
	PM_HSED_HYST_TIME_12_CLK_CYCLES,
	PM_HSED_HYST_TIME_13_CLK_CYCLES,
	PM_HSED_HYST_TIME_14_CLK_CYCLES,
	PM_HSED_HYST_TIME_15_CLK_CYCLES,
	PM_HSED_HYST_TIME_16_CLK_CYCLES,
};

enum hsed_period_pre_div {
	PM_HSED_PERIOD_PRE_DIV_2,
	PM_HSED_PERIOD_PRE_DIV_4,
	PM_HSED_PERIOD_PRE_DIV_8,
	PM_HSED_PERIOD_PRE_DIV_16,
	PM_HSED_PERIOD_PRE_DIV_32,
	PM_HSED_PERIOD_PRE_DIV_64,
	PM_HSED_PERIOD_PRE_DIV_128,
	PM_HSED_PERIOD_PRE_DIV_256,
};

enum hsed_period_time {
	PM_HSED_PERIOD_TIME_1_CLK_CYCLES,
	PM_HSED_PERIOD_TIME_2_CLK_CYCLES,
	PM_HSED_PERIOD_TIME_3_CLK_CYCLES,
	PM_HSED_PERIOD_TIME_4_CLK_CYCLES,
	PM_HSED_PERIOD_TIME_5_CLK_CYCLES,
	PM_HSED_PERIOD_TIME_6_CLK_CYCLES,
	PM_HSED_PERIOD_TIME_7_CLK_CYCLES,
	PM_HSED_PERIOD_TIME_8_CLK_CYCLES,
	PM_HSED_PERIOD_TIME_9_CLK_CYCLES,
	PM_HSED_PERIOD_TIME_10_CLK_CYCLES,
	PM_HSED_PERIOD_TIME_11_CLK_CYCLES,
	PM_HSED_PERIOD_TIME_12_CLK_CYCLES,
	PM_HSED_PERIOD_TIME_13_CLK_CYCLES,
	PM_HSED_PERIOD_TIME_14_CLK_CYCLES,
	PM_HSED_PERIOD_TIME_15_CLK_CYCLES,
	PM_HSED_PERIOD_TIME_16_CLK_CYCLES,
};

enum vreg_lpm_id {
	VREG_GP1_ID,
	VREG_GP2_ID,
	VREG_GP3_ID,
	VREG_GP4_ID,
	VREG_GP5_ID,
	VREG_GP6_ID,
	VREG_GP7_ID,
	VREG_GP8_ID,
	VREG_GP9_ID,
	VREG_GP10_ID,
	VREG_GP11_ID,
	VREG_GP12_ID,
	VREG_GP13_ID,
	VREG_GP14_ID,
	VREG_GP15_ID,
	VREG_GP16_ID,
	VREG_GP17_ID,
	VREG_MDDI_ID,
	VREG_MPLL_ID,
	VREG_MSMC1_ID,
	VREG_MSMC2_ID,
	VREG_MSME_ID,
	VREG_RF_ID,
	VREG_RF1_ID,
	VREG_RF2_ID,
	VREG_RFA_ID,
	VREG_SDCC1_ID,
	VREG_TCXO_ID,
	VREG_USB1P8_ID,
	VREG_USB3P3_ID,
	VREG_USIM_ID,
	VREG_WLAN1_ID,
	VREG_WLAN2_ID,
	VREG_XO_OUT_D0_ID,
	VREG_NCP_ID,
	VREG_LVSW0_ID,
	VREG_LVSW1_ID,
};

enum low_current_led {
	LOW_CURRENT_LED_DRV0,
	LOW_CURRENT_LED_DRV1,
	LOW_CURRENT_LED_DRV2,
};

enum ext_signal {
	EXT_SIGNAL_CURRENT_SINK_MANUAL_MODE,
	EXT_SIGNAL_CURRENT_SINK_PWM1,
	EXT_SIGNAL_CURRENT_SINK_PWM2,
	EXT_SIGNAL_CURRENT_SINK_PWM3,
	EXT_SIGNAL_CURRENT_SINK_DTEST1,
	EXT_SIGNAL_CURRENT_SINK_DTEST2,
	EXT_SIGNAL_CURRENT_SINK_DTEST3,
	EXT_SIGNAL_CURRENT_SINK_DTEST4,
};

enum high_current_led {
	HIGH_CURRENT_LED_FLASH_DRV0,
	HIGH_CURRENT_LED_FLASH_DRV1,
	HIGH_CURRENT_LED_KBD_DRV,
};


int pmic_lp_mode_control(enum switch_cmd cmd, enum vreg_lp_id id);
int pmic_vreg_set_level(enum vreg_id vreg, int level);
int pmic_vreg_pull_down_switch(enum switch_cmd cmd, enum vreg_pdown_id id);
int pmic_secure_mpp_control_digital_output(enum mpp_which which,
		enum mpp_dlogic_level level, enum mpp_dlogic_out_ctrl out);
int pmic_secure_mpp_config_i_sink(enum mpp_which which,
		enum mpp_i_sink_level level, enum mpp_i_sink_switch onoff);
int pmic_secure_mpp_config_digital_input(enum mpp_which	which,
		enum mpp_dlogic_level level, enum mpp_dlogic_in_dbus dbus);
int pmic_rtc_start(struct rtc_time *time);
int pmic_rtc_stop(void);
int pmic_rtc_get_time(struct rtc_time *time);
int pmic_rtc_enable_alarm(enum rtc_alarm alarm,
				struct rtc_time *time);
int pmic_rtc_disable_alarm(enum rtc_alarm alarm);
int pmic_rtc_get_alarm_time(enum rtc_alarm alarm,
				struct rtc_time *time);
int pmic_rtc_get_alarm_status(uint *status);
int pmic_rtc_set_time_adjust(uint adjust);
int pmic_rtc_get_time_adjust(uint *adjust);
int pmic_speaker_cmd(const enum spkr_cmd cmd);
int pmic_set_spkr_configuration(struct spkr_config_mode	*cfg);
int pmic_get_spkr_configuration(struct spkr_config_mode	*cfg);
int pmic_spkr_en_right_chan(uint enable);
int pmic_spkr_is_right_chan_en(uint *enabled);
int pmic_spkr_en_left_chan(uint enable);
int pmic_spkr_is_left_chan_en(uint *enabled);
int pmic_spkr_en(enum spkr_left_right left_right, uint enabled);
int pmic_spkr_is_en(enum spkr_left_right left_right, uint *enabled);
int pmic_spkr_set_gain(enum spkr_left_right left_right, enum spkr_gain gain);
int pmic_spkr_get_gain(enum spkr_left_right left_right, enum spkr_gain *gain);
int pmic_set_speaker_gain(enum spkr_gain gain);
int pmic_set_speaker_delay(enum spkr_dly delay);
int pmic_speaker_1k6_zin_enable(uint enable);
int pmic_spkr_set_mux_hpf_corner_freq(enum spkr_hpf_corner_freq	freq);
int pmic_spkr_get_mux_hpf_corner_freq(enum spkr_hpf_corner_freq	*freq);
int pmic_spkr_select_usb_with_hpf_20hz(uint enable);
int pmic_spkr_is_usb_with_hpf_20hz(uint *enabled);
int pmic_spkr_bypass_mux(uint enable);
int pmic_spkr_is_mux_bypassed(uint *enabled);
int pmic_spkr_en_hpf(uint enable);
int pmic_spkr_is_hpf_en(uint *enabled);
int pmic_spkr_en_sink_curr_from_ref_volt_cir(uint enable);
int pmic_spkr_is_sink_curr_from_ref_volt_cir_en(uint *enabled);
int pmic_spkr_set_delay(enum spkr_left_right left_right, enum spkr_dly delay);
int pmic_spkr_get_delay(enum spkr_left_right left_right, enum spkr_dly *delay);
int pmic_spkr_en_mute(enum spkr_left_right left_right, uint enabled);
int pmic_spkr_is_mute_en(enum spkr_left_right left_right, uint *enabled);
int pmic_mic_en(uint enable);
int pmic_mic_is_en(uint *enabled);
int pmic_mic_set_volt(enum mic_volt vol);
int pmic_mic_get_volt(enum mic_volt *voltage);
int pmic_set_led_intensity(enum ledtype type, int level);
int pmic_flash_led_set_current(uint16_t milliamps);
int pmic_flash_led_set_mode(enum flash_led_mode mode);
int pmic_flash_led_set_polarity(enum flash_led_pol pol);
int pmic_spkr_add_right_left_chan(uint enable);
int pmic_spkr_is_right_left_chan_added(uint *enabled);
int pmic_spkr_en_stereo(uint enable);
int pmic_spkr_is_stereo_en(uint	*enabled);
int pmic_vib_mot_set_volt(uint vol);
int pmic_vib_mot_set_mode(enum pm_vib_mot_mode mode);
int pmic_vib_mot_set_polarity(enum pm_vib_mot_pol pol);
int pmic_vid_en(uint enable);
int pmic_vid_is_en(uint *enabled);
int pmic_vid_load_detect_en(uint enable);

int pmic_hsed_set_period(
	enum hsed_controller controller,
	enum hsed_period_pre_div period_pre_div,
	enum hsed_period_time period_time
);

int pmic_hsed_set_hysteresis(
	enum hsed_controller controller,
	enum hsed_hyst_pre_div hyst_pre_div,
	enum hsed_hyst_time hyst_time
);

int pmic_hsed_set_current_threshold(
	enum hsed_controller controller,
	enum hsed_switch switch_hsed,
	uint32_t current_threshold
);

int pmic_hsed_enable(
	enum hsed_controller controller,
	enum hsed_enable enable
);

int pmic_high_current_led_set_current(enum high_current_led led,
		uint16_t milliamps);
int pmic_high_current_led_set_polarity(enum high_current_led led,
		enum flash_led_pol polarity);
int pmic_high_current_led_set_mode(enum high_current_led led,
		enum flash_led_mode mode);
int pmic_lp_force_lpm_control(enum switch_cmd cmd,
		enum vreg_lpm_id vreg);
int pmic_low_current_led_set_ext_signal(enum low_current_led led,
		enum ext_signal sig);
int pmic_low_current_led_set_current(enum low_current_led led,
		uint16_t milliamps);

#endif
