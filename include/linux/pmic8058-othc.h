/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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

#ifndef __PMIC8058_OTHC_H__
#define __PMIC8058_OTHC_H__

/* Accessory detecion flags */
#define OTHC_MICBIAS_DETECT	BIT(0)
#define OTHC_GPIO_DETECT	BIT(1)
#define OTHC_SWITCH_DETECT	BIT(2)
#define OTHC_ADC_DETECT		BIT(3)

enum othc_accessory_type {
	OTHC_NO_DEVICE = 0,
	OTHC_HEADSET = 1 << 0,
	OTHC_HEADPHONE = 1 << 1,
	OTHC_MICROPHONE = 1 << 2,
	OTHC_ANC_HEADSET = 1 << 3,
	OTHC_ANC_HEADPHONE = 1 << 4,
	OTHC_ANC_MICROPHONE = 1 << 5,
	OTHC_SVIDEO_OUT = 1 << 6,
};

struct accessory_adc_thres {
	int min_threshold;
	int max_threshold;
};

struct othc_accessory_info {
	unsigned int accessory;
	unsigned int detect_flags;
	unsigned int gpio;
	unsigned int active_low;
	unsigned int key_code;
	bool enabled;
	struct accessory_adc_thres adc_thres;
};

enum othc_headset_type {
	OTHC_HEADSET_NO,
	OTHC_HEADSET_NC,
};

struct othc_regulator_config {
	const char *regulator;
	unsigned int max_uV;
	unsigned int min_uV;
};

/* Signal control for OTHC module */
enum othc_micbias_enable {
	/* Turn off MICBIAS signal */
	OTHC_SIGNAL_OFF,
	/* Turn on MICBIAS signal when TCXO is enabled */
	OTHC_SIGNAL_TCXO,
	/* Turn on MICBIAS signal when PWM is high or TCXO is enabled */
	OTHC_SIGNAL_PWM_TCXO,
	/* MICBIAS always enabled */
	OTHC_SIGNAL_ALWAYS_ON,
};

/* Number of MICBIAS lines supported by PMIC8058 */
enum othc_micbias {
	OTHC_MICBIAS_0,
	OTHC_MICBIAS_1,
	OTHC_MICBIAS_2,
	OTHC_MICBIAS_MAX,
};

enum othc_micbias_capability {
	/* MICBIAS used only for BIAS with on/off capability */
	OTHC_MICBIAS,
	/* MICBIAS used to support HSED functionality */
	OTHC_MICBIAS_HSED,
};

struct othc_switch_info {
	u32 min_adc_threshold;
	u32 max_adc_threshold;
	u32 key_code;
};

struct othc_n_switch_config {
	u32 voltage_settling_time_ms;
	u8 num_adc_samples;
	uint32_t adc_channel;
	struct othc_switch_info *switch_info;
	u8 num_keys;
};

struct hsed_bias_config {
	enum othc_headset_type othc_headset;
	u16 othc_lowcurr_thresh_uA;
	u16 othc_highcurr_thresh_uA;
	u32 othc_hyst_prediv_us;
	u32 othc_period_clkdiv_us;
	u32 othc_hyst_clk_us;
	u32 othc_period_clk_us;
	int othc_wakeup;
};

/* Configuration data for HSED */
struct othc_hsed_config {
	struct hsed_bias_config *hsed_bias_config;
	unsigned long detection_delay_ms;
	/* Switch configuration */
	unsigned long switch_debounce_ms;
	bool othc_support_n_switch; /* Set if supporting > 1 switch */
	struct othc_n_switch_config *switch_config;
	/* Accessory configuration */
	bool accessories_support;
	bool accessories_adc_support;
	uint32_t accessories_adc_channel;
	struct othc_accessory_info *accessories;
	int othc_num_accessories;
	int video_out_gpio;
};

struct pmic8058_othc_config_pdata {
	enum othc_micbias micbias_select;
	enum othc_micbias_enable micbias_enable;
	enum othc_micbias_capability micbias_capability;
	struct othc_hsed_config *hsed_config;
	const char *hsed_name;
	struct othc_regulator_config *micbias_regulator;
};

int pm8058_micbias_enable(enum othc_micbias micbias,
			enum othc_micbias_enable enable);
int pm8058_othc_svideo_enable(enum othc_micbias micbias,
			bool enable);

#endif /* __PMIC8058_OTHC_H__ */
