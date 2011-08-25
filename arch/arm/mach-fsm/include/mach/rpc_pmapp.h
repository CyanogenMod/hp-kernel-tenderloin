/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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

#ifndef __ASM_ARCH_MSM_RPC_PMAPP_H
#define __ASM_ARCH_MSM_RPC_PMAPP_H

#include <mach/msm_rpcrouter.h>

/* Clock voting ids */
enum {
	PMAPP_CLOCK_ID_DO = 0,
	PMAPP_CLOCK_ID_D1,
	PMAPP_CLOCK_ID_A0,
	PMAPP_CLOCK_ID_A1,
};

/* Clock voting types */
enum {
	PMAPP_CLOCK_VOTE_OFF = 0,
	PMAPP_CLOCK_VOTE_ON,
	PMAPP_CLOCK_VOTE_PIN_CTRL,
};

/* SMPS vreg ids */
enum {
	PMAPP_VREG_S3 = 21,
	PMAPP_VREG_S2 = 23,
	PMAPP_VREG_S4 = 24,
};

/* SMPS clock voting types */
enum {
	PMAPP_SMPS_CLK_VOTE_DONTCARE = 0,
	PMAPP_SMPS_CLK_VOTE_2P74,	/* 2.74 MHz */
	PMAPP_SMPS_CLK_VOTE_1P6,	/* 1.6 MHz */
};

/* SMPS mode voting types */
enum {
	PMAPP_SMPS_MODE_VOTE_DONTCARE = 0,
	PMAPP_SMPS_MODE_VOTE_PWM,
	PMAPP_SMPS_MODE_VOTE_PFM,
	PMAPP_SMPS_MODE_VOTE_AUTO
};

int msm_pm_app_rpc_init(void);
void msm_pm_app_rpc_deinit(void);
int msm_pm_app_register_vbus_sn(void (*callback)(int online));
void msm_pm_app_unregister_vbus_sn(void (*callback)(int online));
int msm_pm_app_enable_usb_ldo(int);

int pmapp_display_clock_config(uint enable);

int pmapp_clock_vote(const char *voter_id, uint clock_id, uint vote);
int pmapp_smps_clock_vote(const char *voter_id, uint vreg_id, uint vote);
int pmapp_vreg_level_vote(const char *voter_id, uint vreg_id, uint level);
int pmapp_smps_mode_vote(const char *voter_id, uint vreg_id, uint mode);

#endif
