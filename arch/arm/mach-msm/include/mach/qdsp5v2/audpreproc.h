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

#ifndef _MACH_QDSP5_V2_AUDPREPROC_H
#define _MACH_QDSP5_V2_AUDPREPROC_H

#include <mach/qdsp5v2/qdsp5audpreproccmdi.h>
#include <mach/qdsp5v2/qdsp5audpreprocmsg.h>

#define MAX_ENC_COUNT 3

#define MSM_ADSP_ENC_CODEC_WAV 0
#define MSM_ADSP_ENC_CODEC_AAC 1
#define MSM_ADSP_ENC_CODEC_SBC 2
#define MSM_ADSP_ENC_CODEC_AMRNB 3
#define MSM_ADSP_ENC_CODEC_EVRC 4
#define MSM_ADSP_ENC_CODEC_QCELP 5

#define MSM_ADSP_ENC_MODE_TUNNEL 24
#define MSM_ADSP_ENC_MODE_NON_TUNNEL 25

#define AUDPREPROC_CODEC_MASK 0x00FF
#define AUDPREPROC_MODE_MASK 0xFF00

#define MSM_AUD_ENC_MODE_TUNNEL  0x00000100
#define MSM_AUD_ENC_MODE_NONTUNNEL  0x00000200

#define SOURCE_PIPE_1	0x0001
#define SOURCE_PIPE_0	0x0000

/* event callback routine prototype*/
typedef void (*audpreproc_event_func)(void *private, unsigned id, void *msg);

struct audpreproc_event_callback {
	audpreproc_event_func fn;
	void *private;
};


/* Exported common api's from audpreproc layer */
int audpreproc_aenc_alloc(unsigned enc_type, const char **module_name,
		unsigned *queue_id);
void audpreproc_aenc_free(int enc_id);

int audpreproc_enable(int enc_id, audpreproc_event_func func, void *private);
void audpreproc_disable(int enc_id, void *private);

int audpreproc_send_audreccmdqueue(void *cmd, unsigned len);

int audpreproc_send_preproccmdqueue(void *cmd, unsigned len);

int audpreproc_dsp_set_agc(struct audpreproc_cmd_cfg_agc_params *agc,
	unsigned len);
int audpreproc_dsp_set_agc2(struct audpreproc_cmd_cfg_agc_params_2 *agc2,
	unsigned len);
int audpreproc_dsp_set_ns(struct audpreproc_cmd_cfg_ns_params *ns,
	unsigned len);
int audpreproc_dsp_set_iir(
struct audpreproc_cmd_cfg_iir_tuning_filter_params *iir, unsigned len);

int audpreproc_dsp_set_agc(struct audpreproc_cmd_cfg_agc_params *agc,
 unsigned int len);

int audpreproc_dsp_set_iir(
struct audpreproc_cmd_cfg_iir_tuning_filter_params *iir, unsigned int len);

int audpreproc_unregister_event_callback(struct audpreproc_event_callback *ecb);

int audpreproc_register_event_callback(struct audpreproc_event_callback *ecb);

int audpreproc_dsp_set_gain_tx(
	struct audpreproc_cmd_cfg_cal_gain *calib_gain_tx, unsigned len);

#endif /* _MACH_QDSP5_V2_AUDPREPROC_H */
