#ifndef QDSP5AUDRECMSGI_H
#define QDSP5AUDRECMSGI_H

/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*

    A U D I O   R E C O R D  M E S S A G E S

GENERAL DESCRIPTION
  This file contains defintions of format blocks of messages 
  that are sent by AUDREC Task

REFERENCES
  None

EXTERNALIZED FUNCTIONS
  None

Copyright (c) 1992-2009, Code Aurora Forum. All rights reserved.

This software is licensed under the terms of the GNU General Public
License version 2, as published by the Free Software Foundation, and
may be copied, distributed, and modified under those terms.
 
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/

/*===========================================================================

                      EDIT HISTORY FOR FILE

This section contains comments describing changes made to this file.
Notice that changes are listed in reverse chronological order.
   
 $Header: //source/qcom/qct/multimedia2/Audio/drivers/QDSP5Driver/QDSP5Interface/main/latest/qdsp5audrecmsg.h#3 $

============================================================================*/

/*
 * AUDRECTASK MESSAGES
 * AUDRECTASK uses audRecUpRlist to communicate with ARM
 * Location : MEMC
 * Buffer size : 4
 * No of buffers in a queue : 2
 */

/*
 * Message to notify that config command is done
 */

#define AUDREC_MSG_CMD_CFG_DONE_MSG	0x0002
#define AUDREC_MSG_CMD_CFG_DONE_MSG_LEN	\
	sizeof(struct audrec_msg_cmd_cfg_done_msg)


#define AUDREC_MSG_CFG_DONE_TYPE_0_ENA		0x4000
#define AUDREC_MSG_CFG_DONE_TYPE_0_DIS		0x0000

#define AUDREC_MSG_CFG_DONE_TYPE_0_NO_UPDATE	0x0000
#define AUDREC_MSG_CFG_DONE_TYPE_0_UPDATE	0x8000

#define AUDREC_MSG_CFG_DONE_TYPE_1_ENA		0x4000
#define AUDREC_MSG_CFG_DONE_TYPE_1_DIS		0x0000

#define AUDREC_MSG_CFG_DONE_TYPE_1_NO_UPDATE	0x0000
#define AUDREC_MSG_CFG_DONE_TYPE_1_UPDATE	0x8000

#define AUDREC_MSG_CFG_DONE_ENC_ENA		0x8000
#define AUDREC_MSG_CFG_DONE_ENC_DIS		0x0000

struct audrec_msg_cmd_cfg_done_msg {
	unsigned short	audrec_enc_type;
	unsigned short	audrec_obj_idx;
} __attribute__((packed));

/*
 * Message to notify arec0/1 or concurrent encoder cfg done
 * and recording params recieved by task
 */

#define	AUDREC_MSG_CMD_AREC_PARAM_CFG_DONE_MSG		0x0003
#define	AUDREC_MSG_CMD_AREC_PARAM_CFG_DONE_MSG_LEN	\
	sizeof(struct audrec_msg_cmd_arec_param_cfg_done_msg)


#define	AUDREC_MSG_AREC_PARAM_TYPE_0	0x0000
#define	AUDREC_MSG_AREC_PARAM_TYPE_1	0x0001

struct audrec_msg_cmd_arec_param_cfg_done_msg {
	unsigned short	audrec_obj_idx;
} __attribute__((packed));

/*
 * Message to notify no more buffers are available in ext mem to DME
 * Or no concurrent encoder supported
 */

#define AUDREC_MSG_FATAL_ERR_MSG		0x0004
#define AUDREC_MSG_FATAL_ERR_MSG_LEN	\
	sizeof(struct audrec_msg_fatal_err_msg)


#define AUDREC_MSG_FATAL_ERR_TYPE_0	0x0000
#define AUDREC_MSG_FATAL_ERR_TYPE_1	0x0001

struct audrec_msg_fatal_err_msg {
	unsigned short	audrec_obj_idx;
	unsigned short	audrec_err_id;
} __attribute__((packed));

/*
 * Message to notify DME deliverd the encoded pkt to ext pkt buffer
 */

#define AUDREC_MSG_PACKET_READY_MSG		0x0005
#define AUDREC_MSG_PACKET_READY_MSG_LEN	\
	sizeof(struct audrec_msg_packet_ready_msg)


#define AUDREC_MSG_PACKET_READY_TYPE_0	0x0000
#define AUDREC_MSG_PACKET_READY_TYPE_1	0x0001

struct audrec_msg_packet_ready_msg {
	unsigned short	audrec_obj_idx;
	unsigned short	pkt_counter_msw;
	unsigned short	pkt_counter_lsw;
	unsigned short	pkt_read_cnt_msw;
	unsigned short	pkt_read_cnt_lsw;
} __attribute__((packed));

/*
 * Message to notify external memory cfg done and recieved by task
 */

#define	AUDREC_MSG_CMD_AREC_MEM_CFG_DONE_MSG		0x0006
#define	AUDREC_MSG_CMD_AREC_MEM_CFG_DONE_MSG_LEN	\
	sizeof(struct audrec_msg_cmd_arec_mem_cfg_done_msg)


struct audrec_msg_cmd_arec_mem_cfg_done_msg {
	unsigned short	audrec_obj_idx;
} __attribute__((packed));

#define ADSP_MESSAGE_ID 0xFFFF

#endif
