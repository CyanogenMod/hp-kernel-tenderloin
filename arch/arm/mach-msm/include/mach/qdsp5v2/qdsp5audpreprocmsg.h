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

#ifndef QDSP5AUDPREPROCMSG_H
#define QDSP5AUDPREPROCMSG_H

/*
 * ADSPREPROCTASK Messages
 * AUDPREPROCTASK uses audPreProcUpRlist to communicate with ARM
 * Location	: MEMB
 * Buffer size :  6
 * No of buffers in queue : 4
 */

/*
 * Message to indicate Pre processing config command is done
 */

#define AUDPREPROC_CMD_CFG_DONE_MSG 0x0001
#define	AUDPREPROC_CMD_CFG_DONE_MSG_LEN	\
	sizeof(struct audpreproc_cmd_cfg_done_msg)

#define AUD_PREPROC_TYPE_AGC		0x0
#define AUD_PREPROC_NOISE_REDUCTION	0x1
#define AUD_PREPROC_IIR_TUNNING_FILTER	0x2

#define AUD_PREPROC_CONFIG_ENABLED 	-1
#define AUD_PREPROC_CONFIG_DISABLED	 0

struct audpreproc_cmd_cfg_done_msg {
	unsigned short stream_id;
	unsigned short aud_preproc_type;
	signed short aud_preproc_status_flag;
} __attribute__((packed));

/*
 * Message to indicate Pre processing error messages
 */

#define AUDPREPROC_ERROR_MSG 0x0002
#define AUDPREPROC_ERROR_MSG_LEN \
	sizeof(struct audpreproc_err_msg)

#define AUD_PREPROC_ERR_IDX_WRONG_SAMPLING_FREQUENCY	0x00
#define AUD_PREPROC_ERR_IDX_ENC_NOT_SUPPORTED		0x01

struct audpreproc_err_msg {
	unsigned short stream_id;
	signed short aud_preproc_err_idx;
} __attribute__((packed));

/*
 * Message to indicate encoder config command
 */

#define AUDPREPROC_CMD_ENC_CFG_DONE_MSG	0x0003
#define AUDPREPROC_CMD_ENC_CFG_DONE_MSG_LEN \
	sizeof(struct audpreproc_cmd_enc_cfg_done_msg)

struct audpreproc_cmd_enc_cfg_done_msg {
	unsigned short stream_id;
	unsigned short rec_enc_type;
} __attribute__((packed));

/*
 * Message to indicate encoder param config command
 */

#define AUDPREPROC_CMD_ENC_PARAM_CFG_DONE_MSG	0x0004
#define AUDPREPROC_CMD_ENC_PARAM_CFG_DONE_MSG_LEN \
	sizeof(struct audpreproc_cmd_enc_param_cfg_done_msg)

struct audpreproc_cmd_enc_param_cfg_done_msg {
	unsigned short stream_id;
} __attribute__((packed));


/*
 * Message to indicate AFE config cmd for
 * audio recording is successfully recieved
 */

#define AUDPREPROC_AFE_CMD_AUDIO_RECORD_CFG_DONE_MSG  0x0005
#define AUDPREPROC_AFE_CMD_AUDIO_RECORD_CFG_DONE_MSG_LEN \
	sizeof(struct audpreproc_afe_cmd_audio_record_cfg_done)

struct audpreproc_afe_cmd_audio_record_cfg_done {
	unsigned short stream_id;
} __attribute__((packed));

/*
 * Message to indicate Routing mode
 * configuration success or failure
 */

#define AUDPREPROC_CMD_ROUTING_MODE_DONE_MSG  0x0007
#define AUDPREPROC_CMD_ROUTING_MODE_DONE_MSG_LEN \
	sizeof(struct audpreproc_cmd_routing_mode_done)

struct audpreproc_cmd_routing_mode_done {
	unsigned short stream_id;
	unsigned short configuration;
} __attribute__((packed));


#define AUDPREPROC_CMD_PCM_CFG_ARM_TO_PREPROC_DONE_MSG	0x0008
#define AUDPREPROC_CMD_PCM_CFG_ARM_TO_PREPROC_DONE_MSG_LEN \
	sizeof(struct audreproc_cmd_pcm_cfg_arm_to_preproc_done)

struct audreproc_cmd_pcm_cfg_arm_to_preproc_done {
	unsigned short stream_id;
	unsigned short configuration;
} __attribute__((packed));

#endif /* QDSP5AUDPREPROCMSG_H */
