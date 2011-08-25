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
 * Qualcomm XOADC Driver header file
 */

#ifndef _XOADC_H
#define _XOADC_H_

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/workqueue.h>

struct xoadc_conv_state {
	struct adc_conv_slot	*context;
	struct list_head	slots;
	struct mutex		list_lock;
};

#define CHANNEL_VCOIN		0
#define CHANNEL_VBAT		1
#define CHANNEL_VCHG		2
#define CHANNEL_CHG_MONITOR	3
#define CHANNEL_VPH_PWR		4
#define CHANNEL_MPP5		5
#define CHANNEL_MPP6		6
#define CHANNEL_MPP7		7
#define CHANNEL_MPP8		8
#define CHANNEL_MPP9		9
#define CHANNEL_USB_VBUS	0Xa
#define CHANNEL_DIE_TEMP	0Xb
#define CHANNEL_INTERNAL	0xc
#define CHANNEL_125V		0xd
#define CHANNEL_INTERNAL_2	0Xe
#define CHANNEL_MUXOFF		0xf

#define XOADC_MPP_3		0x2
#define XOADC_MPP_4             0X3
#define XOADC_MPP_5             0x4
#define XOADC_MPP_7             0x6
#define XOADC_MPP_8             0x7
#define XOADC_MPP_10		0X9

#define XOADC_PMIC_0		0x0

#define CHANNEL_ADC_625_MV      625

struct xoadc_platform_data {
	struct adc_properties *xoadc_prop;
	u32 (*xoadc_setup) (void);
	void (*xoadc_shutdown) (void);
	void (*xoadc_mpp_config) (void);
	int (*xoadc_vreg_set) (int);
	int (*xoadc_vreg_setup) (void);
	void (*xoadc_vreg_shutdown) (void);
	u32 xoadc_num;
	u32 xoadc_wakeup;
};

int32_t pm8058_xoadc_read_adc_code(uint32_t , int32_t *data);

int32_t pm8058_xoadc_select_chan_and_start_conv(uint32_t,
						struct adc_conv_slot *);

void pm8058_xoadc_slot_request(uint32_t, struct adc_conv_slot **slot);

void pm8058_xoadc_restore_slot(uint32_t, struct adc_conv_slot *slot);

struct adc_properties *pm8058_xoadc_get_properties(uint32_t);

int32_t pm8058_xoadc_calibrate(uint32_t, struct adc_conv_slot *, int *);

int32_t pm8058_xoadc_registered(void);

int32_t pm8058_xoadc_calib_device(uint32_t adc_instance);
#endif
