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

#ifndef QDSP5AUDRECMSG_H
#define QDSP5AUDRECMSG_H

/*
 * AUDRECTASK MESSAGES
 * AUDRECTASK uses audRec[i]UpRlist, where i=0,1,2 to communicate with ARM
 * Location : MEMC
 * Buffer size : 5
 * No of buffers in a queue : 10
 */

/*
 * Message to notify 2 error conditions
 */

#define AUDREC_FATAL_ERR_MSG 0x0001
#define AUDREC_FATAL_ERR_MSG_LEN	\
	sizeof(struct audrec_fatal_err_msg)

#define AUDREC_FATAL_ERR_MSG_NO_PKT	0x00

struct audrec_fatal_err_msg {
	unsigned short audrec_err_id;
} __attribute__((packed));

/*
 * Message to indicate encoded packet is delivered to external buffer
 */

#define AUDREC_UP_PACKET_READY_MSG 0x0002
#define AUDREC_UP_PACKET_READY_MSG_LEN	\
	sizeof(struct audrec_up_pkt_ready_msg)

struct  audrec_up_pkt_ready_msg {
	unsigned short audrec_packet_write_cnt_lsw;
	unsigned short audrec_packet_write_cnt_msw;
	unsigned short audrec_up_prev_read_cnt_lsw;
	unsigned short audrec_up_prev_read_cnt_msw;
} __attribute__((packed));

/*
 * Message indicates arecmem cfg done
 */
#define AUDREC_CMD_MEM_CFG_DONE_MSG 0x0003

/* buffer conntents are nill only message id is required */

/*
 * Message to indicate pcm buffer configured
 */

#define AUDREC_CMD_PCM_CFG_ARM_TO_ENC_DONE_MSG 0x0004
#define AUDREC_CMD_PCM_CFG_ARM_TO_ENC_DONE_MSG_LEN	\
	sizeof(struct audrec_cmd_pcm_cfg_arm_to_enc_msg)

struct  audrec_cmd_pcm_cfg_arm_to_enc_msg {
	unsigned short configuration;
} __attribute__((packed));

/*
 * Message to indicate encoded packet is delivered to external buffer in FTRT
 */

#define AUDREC_UP_NT_PACKET_READY_MSG 0x0005
#define AUDREC_UP_NT_PACKET_READY_MSG_LEN	\
	sizeof(struct audrec_up_nt_packet_ready_msg)

struct  audrec_up_nt_packet_ready_msg {
	unsigned short audrec_packetwrite_cnt_lsw;
	unsigned short audrec_packetwrite_cnt_msw;
	unsigned short audrec_upprev_readcount_lsw;
	unsigned short audrec_upprev_readcount_msw;
} __attribute__((packed));

/*
 * Message to indicate pcm buffer is consumed
 */

#define AUDREC_CMD_PCM_BUFFER_PTR_UPDATE_ARM_TO_ENC_MSG 0x0006
#define AUDREC_CMD_PCM_BUFFER_PTR_UPDATE_ARM_TO_ENC_MSG_LEN	\
	sizeof(struct audrec_cmd_pcm_buffer_ptr_update_arm_to_enc_msg)

struct  audrec_cmd_pcm_buffer_ptr_update_arm_to_enc_msg {
	unsigned short buffer_readcnt_msw;
	unsigned short buffer_readcnt_lsw;
	unsigned short number_of_buffers;
	unsigned short buffer_address_length[];
} __attribute__((packed));

/*
 * Message to indicate flush acknowledgement
 */

#define AUDREC_CMD_FLUSH_DONE_MSG 0x0007

/*
 * Message to indicate End of Stream acknowledgement
 */

#define AUDREC_CMD_EOS_ACK_MSG 0x0008

#endif /* QDSP5AUDRECMSG_H */
