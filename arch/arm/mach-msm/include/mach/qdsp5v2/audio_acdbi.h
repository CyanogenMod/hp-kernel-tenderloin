/* Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
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
#ifndef _MACH_QDSP5_V2_AUDIO_ACDBI_H
#define _MACH_QDSP5_V2_AUDIO_ACDBI_H

#define DBOR_SIGNATURE	0x524F4244

struct header {
	u32 dbor_signature;
	u32 abid;
	u32 iid;
	u32 data_len;
};

enum {
	ACDB_AGC_BLOCK			= 197,
	ACDB_IIR_BLOCK			= 245,
	ACDB_MBADRC_BLOCK		= 343
};

/* Structure to query for acdb parameter */
struct acdb_get_block {
	u32	acdb_id;
	u32	sample_rate_id;		/* Actual sample rate value */
	u32	interface_id;		/* Interface id's */
	u32	algorithm_block_id;	/* Algorithm block id */
	u32	total_bytes;		/* Length in bytes used by buffer for
						configuration */
	u32	*buf_ptr;		/* Address for storing configuration
						data */
};

struct acdb_agc_block {
	u16	enable_status;
	u16	comp_rlink_static_gain;
	u16	comp_rlink_aig_flag;
	u16	exp_rlink_threshold;
	u16	exp_rlink_slope;
	u16	comp_rlink_threshold;
	u16	comp_rlink_slope;
	u16	comp_rlink_aig_attack_k;
	u16	comp_rlink_aig_leak_down;
	u16	comp_rlink_aig_leak_up;
	u16	comp_rlink_aig_max;
	u16	comp_rlink_aig_min;
	u16	comp_rlink_aig_release_k;
	u16	comp_rlink_aig_sm_leak_rate_fast;
	u16	comp_rlink_aig_sm_leak_rate_slow;
	u16	comp_rlink_attack_k_msw;
	u16	comp_rlink_attack_k_lsw;
	u16	comp_rlink_delay;
	u16	comp_rlink_release_k_msw;
	u16	comp_rlink_release_k_lsw;
	u16	comp_rlink_rms_trav;
};


struct iir_coeff_type {
	u16	b0_lo;
	u16	b0_hi;
	u16	b1_lo;
	u16	b1_hi;
	u16	b2_lo;
	u16	b2_hi;
};

struct iir_coeff_stage_a {
	u16	a1_lo;
	u16	a1_hi;
	u16	a2_lo;
	u16	a2_hi;
};

struct acdb_iir_block {
	u16			enable_flag;
	u16			stage_count;
	struct iir_coeff_type	stages[4];
	struct iir_coeff_stage_a stages_a[4];
	u16			shift_factor[4];
	u16			pan[4];
};



struct mbadrc_band_config_type {
	u16	mbadrc_sub_band_enable;
	u16	mbadrc_sub_mute;
	u16	mbadrc_comp_rms_tav;
	u16	mbadrc_comp_threshold;
	u16	mbadrc_comp_slop;
	u16	mbadrc_comp_attack_msw;
	u16	mbadrc_comp_attack_lsw;
	u16	mbadrc_comp_release_msw;
	u16	mbadrc_comp_release_lsw;
	u16	mbadrc_make_up_gain;
};

struct mbadrc_parameter {
	u16				mbadrc_enable;
	u16				mbadrc_num_bands;
	u16				mbadrc_down_sample_level;
	u16				mbadrc_delay;
};

struct acdb_mbadrc_block {
	u16				ext_buf[196];
	struct mbadrc_band_config_type	band_config[5];
	struct mbadrc_parameter		parameters;
};

struct  acdb_calib_gain_rx {
	u16 audppcalgain;
	u16 reserved;
};

struct acdb_calib_gain_tx {
	u16 audprecalgain;
	u16 reserved;
};

struct acdb_pbe_block {
	s16 realbassmix;
	s16 basscolorcontrol;
	u16 mainchaindelay;
	u16 xoverfltorder;
	u16 bandpassfltorder;
	s16 adrcdelay;
	u16 downsamplelevel;
	u16 comprmstav;
	s16 expthreshold;
	u16 expslope;
	u16 compthreshold;
	u16 compslope;
	u16 cpmpattack_lsw;
	u16 compattack_msw;
	u16 comprelease_lsw;
	u16 comprelease_msw;
	u16 compmakeupgain;
	s16 baselimthreshold;
	s16 highlimthreshold;
	s16 basslimmakeupgain;
	s16 highlimmakeupgain;
	s16 limbassgrc;
	s16 limhighgrc;
	s16 limdelay;
	u16 filter_coeffs[90];
};

struct acdb_rmc_block  {
	s16 rmc_enable;
	u16 rmc_ipw_length_ms;
	u16 rmc_detect_start_threshdb;
	u16 rmc_peak_length_ms;
	s16 rmc_init_pulse_threshdb;
	u16 rmc_init_pulse_length_ms;
	u16 rmc_total_int_length_ms;
	u16 rmc_rampupdn_length_ms;
	u16 rmc_delay_length_ms;
	u16 reserved00;
	u16 reserved01;
	s16 reserved02;
	s16 reserved03;
	s16 reserved04;
};

s32 acdb_get_calibration_data(struct acdb_get_block *get_block);
#endif
