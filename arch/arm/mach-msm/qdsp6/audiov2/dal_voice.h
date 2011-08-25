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

#ifndef __DAL_VOICE_H__
#define __DAL_VOICE_H__

#define VOICE_DAL_DEVICE 0x02000075
#define VOICE_DAL_PORT "DAL_AM_AUD"
#define VOICE_DAL_VERSION 0x00010000

#define APR_PKTV1_TYPE_EVENT_V 0
#define APR_UNDEFINED -1
#define APR_PKTV1_TYPE_MASK 0x00000010
#define APR_PKTV1_TYPE_SHFT 4

#define APR_SET_BITMASK(mask, shift, value) \
	(((value) << (shift)) & (mask))

#define APR_SET_FIELD(field, value) \
	APR_SET_BITMASK((field##_MASK), (field##_SHFT), (value))


enum {
	VOICE_OP_INIT = DAL_OP_FIRST_DEVICE_API,
	VOICE_OP_CONTROL,
};

struct apr_command_pkt {
	uint32_t size;
	uint32_t header;
	uint16_t reserved1;
	uint16_t src_addr;
	uint16_t dst_addr;
	uint16_t ret_addr;
	uint32_t src_token;
	uint32_t dst_token;
	uint32_t ret_token;
	uint32_t context;
	uint32_t opcode;
} __attribute__ ((packed));


#define APR_IBASIC_RSP_RESULT 0x00010000

#define APR_OP_CMD_CREATE 0x0001001B

#define APR_OP_CMD_DESTROY 0x0001001C

#define VOICE_OP_CMD_BRINGUP 0x0001001E

#define VOICE_OP_CMD_TEARDOWN 0x0001001F

#define VOICE_OP_CMD_SET_NETWORK 0x0001001D

#define VOICE_OP_CMD_STREAM_SETUP 0x00010027

#define VOICE_OP_CMD_STREAM_TEARDOWN 0x00010028

#endif
