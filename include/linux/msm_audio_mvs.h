/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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
#ifndef __MSM_AUDIO_MVS_H
#define __MSM_AUDIO_MVS_H

#include <linux/msm_audio.h>

#define AUDIO_GET_MVS_CONFIG _IOW(AUDIO_IOCTL_MAGIC, \
	(AUDIO_MAX_COMMON_IOCTL_NUM + 0), unsigned)
#define AUDIO_SET_MVS_CONFIG _IOR(AUDIO_IOCTL_MAGIC, \
	(AUDIO_MAX_COMMON_IOCTL_NUM + 1), unsigned)

/* MVS modes */
#define MVS_MODE_IS127 2
#define MVS_MODE_AMR 5
#define MVS_MODE_LINEAR_PCM 9
#define MVS_MODE_PCM 12
#define MVS_MODE_AMR_WB 13
#define MVS_MODE_G729A 14
#define MVS_MODE_G711A 15

enum msm_audio_amr_mode {
	MVS_AMR_MODE_0475, /* AMR 4.75 kbps */
	MVS_AMR_MODE_0515, /* AMR 5.15 kbps */
	MVS_AMR_MODE_0590, /* AMR 5.90 kbps */
	MVS_AMR_MODE_0670, /* AMR 6.70 kbps */
	MVS_AMR_MODE_0740, /* AMR 7.40 kbps */
	MVS_AMR_MODE_0795, /* AMR 7.95 kbps */
	MVS_AMR_MODE_1020, /* AMR 10.20 kbps */
	MVS_AMR_MODE_1220, /* AMR 12.20 kbps */
	MVS_AMR_MODE_0660, /* AMR-WB 6.60 kbps */
	MVS_AMR_MODE_0885, /* AMR-WB 8.85 kbps */
	MVS_AMR_MODE_1265, /* AMR-WB 12.65 kbps */
	MVS_AMR_MODE_1425, /* AMR-WB 14.25 kbps */
	MVS_AMR_MODE_1585, /* AMR-WB 15.85 kbps */
	MVS_AMR_MODE_1825, /* AMR-WB 18.25 kbps */
	MVS_AMR_MODE_1985, /* AMR-WB 19.85 kbps */
	MVS_AMR_MODE_2305, /* AMR-WB 23.05 kbps */
	MVS_AMR_MODE_2385, /* AMR-WB 23.85 kbps */
	MVS_AMR_MODE_UNDEF
};

enum msm_audio_voc_rate {
		MVS_VOC_0_RATE, /* Blank frame */
		MVS_VOC_8_RATE, /* 1/8 rate    */
		MVS_VOC_4_RATE, /* 1/4 rate    */
		MVS_VOC_2_RATE, /* 1/2 rate    */
		MVS_VOC_1_RATE	/* Full rate   */
};

enum msm_audio_amr_frame_type {
	MVS_AMR_SPEECH_GOOD,	      /* Good speech frame              */
	MVS_AMR_SPEECH_DEGRADED,      /* Speech degraded                */
	MVS_AMR_ONSET,		      /* Onset                          */
	MVS_AMR_SPEECH_BAD,	      /* Corrupt speech frame (bad CRC) */
	MVS_AMR_SID_FIRST,	      /* First silence descriptor       */
	MVS_AMR_SID_UPDATE,	      /* Comfort noise frame            */
	MVS_AMR_SID_BAD,	      /* Corrupt SID frame (bad CRC)    */
	MVS_AMR_NO_DATA,	      /* Nothing to transmit            */
	MVS_AMR_SPEECH_LOST	      /* Downlink speech lost           */
};

enum msm_audio_g711a_mode {
	MVS_G711A_MODE_MULAW,
	MVS_G711A_MODE_ALAW
};

enum msm_audio_g711a_frame_type {
	MVS_G711A_SPEECH_GOOD,
	MVS_G711A_SID,
	MVS_G711A_NO_DATA,
	MVS_G711A_ERASURE
};

enum msm_audio_g729a_frame_type {
	MVS_G729A_NO_DATA,
	MVS_G729A_SPEECH_GOOD,
	MVS_G729A_SID,
	MVS_G729A_ERASURE
};

struct msm_audio_mvs_config {
	uint32_t mvs_mode;
	uint32_t rate_type;
};

#define MVS_MAX_VOC_PKT_SIZE 320

struct msm_audio_mvs_frame {
	uint32_t frame_type;
	uint32_t len;
	uint8_t voc_pkt[MVS_MAX_VOC_PKT_SIZE];

};

#endif /* __MSM_AUDIO_MVS_H */
