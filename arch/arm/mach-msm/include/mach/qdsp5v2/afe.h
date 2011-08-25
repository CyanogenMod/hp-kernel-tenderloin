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
#ifndef _MACH_QDSP5_V2_AFE_H
#define _MACH_QDSP5_V2_AFE_H

#include <asm/types.h>
#include <mach/qdsp5v2/audio_acdbi.h>

#define AFE_HW_PATH_CODEC_RX    1
#define AFE_HW_PATH_CODEC_TX    2
#define AFE_HW_PATH_AUXPCM_RX   3
#define AFE_HW_PATH_AUXPCM_TX   4
#define AFE_HW_PATH_MI2S_RX     5
#define AFE_HW_PATH_MI2S_TX     6

#define AFE_VOLUME_UNITY 0x4000 /* Based on Q14 */

struct msm_afe_config {
	u16 sample_rate;
	u16 channel_mode;
	u16 volume;
	/* To be expaned for AUX CODEC */
};

int afe_enable(u8 path_id, struct msm_afe_config *config);

int afe_disable(u8 path_id);

int afe_config_aux_codec(int pcm_ctl_value, int aux_codec_intf_value,
			int data_format_pad);
int afe_config_fm_codec(int fm_enable, uint16_t source);

int afe_config_fm_volume(uint16_t volume);
int afe_config_fm_calibration_gain(uint16_t device_id,
			uint16_t calibration_gain);
void afe_loopback(int enable);

void afe_device_volume_ctrl(u16 device_id, u16 device_volume);

int afe_config_rmc_block(struct acdb_rmc_block *acdb_rmc);
#endif
