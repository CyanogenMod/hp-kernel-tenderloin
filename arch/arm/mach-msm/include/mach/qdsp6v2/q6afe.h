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
 */
#ifndef __Q6AFE_H__
#define __Q6AFE_H__
#include <mach/qdsp6v2/apr_audio.h>

#define MSM_AFE_MONO		0
#define MSM_AFE_MONO_RIGHT	1
#define MSM_AFE_MONO_LEFT	2
#define MSM_AFE_STEREO		3

enum {
	IDX_PRIMARY_I2S_RX = 0,
	IDX_PRIMARY_I2S_TX = 1,
	IDX_PCM_RX = 2,
	IDX_PCM_TX = 3,
	IDX_SECONDARY_I2S_RX = 4,
	IDX_SECONDARY_I2S_TX = 5,
	IDX_MI2S_RX = 6,
	IDX_MI2S_TX = 7,
	IDX_HDMI_RX = 8,
	IDX_RSVD_2 = 9,
	IDX_RSVD_3 = 10,
	IDX_DIGI_MIC_TX = 11,
	IDX_VOICE_RECORD_RX = 12,
	IDX_VOICE_RECORD_TX = 13,
	IDX_VOICE_PLAYBACK_TX = 14,
	AFE_MAX_PORTS
};

int afe_open(u16 port_id, union afe_port_config *afe_config, int rate);
int afe_close(int port_id);
int afe_loopback(u16 enable, u16 rx_port, u16 tx_port);
int afe_sidetone(u16 tx_port_id, u16 rx_port_id, u16 enable, uint16_t gain);
int afe_loopback_gain(u16 port_id, u16 volume);
int afe_validate_port(u16 port_id);
int afe_get_port_index(u16 port_id);
int afe_start_pseudo_port(u16 port_id);
int afe_stop_pseudo_port(u16 port_id);

#endif /* __Q6AFE_H__ */
