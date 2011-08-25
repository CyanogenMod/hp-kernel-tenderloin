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
#ifndef __MACH_QDSP5_V2_AUX_PCM_H
#define __MACH_QDSP5_V2_AUX_PCM_H
#include <mach/qdsp5v2/audio_def.h>

/* define some values in AUX_CODEC_CTL register */
#define AUX_CODEC_CTL__ADSP_CODEC_CTL_EN__MSM_V 0 /* default */
#define AUX_CODEC_CTL__ADSP_CODEC_CTL_EN__ADSP_V 0x800
#define AUX_CODEC_CTL__PCM_SYNC_LONG_OFFSET_V  0x400
#define AUX_CODEC_CTL__PCM_SYNC_SHORT_OFFSET_V 0x200
#define AUX_CODEC_CTL__I2S_SAMPLE_CLK_SRC__SDAC_V 0
#define AUX_CODEC_CTL__I2S_SAMPLE_CLK_SRC__ICODEC_V 0x80
#define AUX_CODEC_CTL__I2S_SAMPLE_CLK_MODE__MASTER_V 0
#define AUX_CODEC_CTL__I2S_SAMPLE_CLK_MODE__SLAVE_V 0x40
#define AUX_CODEC_CTL__I2S_RX_MODE__REV_V 0
#define AUX_CODEC_CTL__I2S_RX_MODE__TRAN_V 0x20
#define AUX_CODEC_CTL__I2S_CLK_MODE__MASTER_V 0
#define AUX_CODEC_CTL__I2S_CLK_MODE__SLAVE_V 0x10
#define AUX_CODEC_CTL__AUX_PCM_MODE__PRIM_MASTER_V 0
#define AUX_CODEC_CTL__AUX_PCM_MODE__AUX_MASTER_V 0x4
#define AUX_CODEC_CTL__AUX_PCM_MODE__PRIM_SLAVE_V 0x8
#define AUX_CODEC_CTL__AUX_CODEC_MDOE__PCM_V 0
#define AUX_CODEC_CTL__AUX_CODEC_MODE__I2S_V 0x2

/* define some values in PCM_PATH_CTL register */
#define PCM_PATH_CTL__ADSP_CTL_EN__MSM_V 0
#define PCM_PATH_CTL__ADSP_CTL_EN__ADSP_V 0x8

/* define some values for aux codec config of AFE*/
/* PCM CTL */
#define PCM_CTL__RPCM_WIDTH__LINEAR_V 0x1
#define PCM_CTL__TPCM_WIDTH__LINEAR_V 0x2
/* AUX_CODEC_INTF_CTL */
#define AUX_CODEC_INTF_CTL__PCMINTF_DATA_EN_V 0x800
/* DATA_FORMAT_PADDING_INFO */
#define DATA_FORMAT_PADDING_INFO__RPCM_FORMAT_V 0x400
#define DATA_FORMAT_PADDING_INFO__TPCM_FORMAT_V 0x2000

void aux_codec_adsp_codec_ctl_en(bool msm_adsp_en);
void aux_codec_pcm_path_ctl_en(bool msm_adsp_en);
int aux_pcm_gpios_request(void);
void aux_pcm_gpios_free(void);

#endif
