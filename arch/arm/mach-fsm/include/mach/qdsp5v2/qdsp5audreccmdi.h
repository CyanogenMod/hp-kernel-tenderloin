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

#ifndef QDSP5AUDRECCMDI_H
#define QDSP5AUDRECCMDI_H

/*
 * AUDRECTASK COMMANDS
 * ARM uses 2 queues to communicate with the AUDRECTASK
 * 1.uPAudRec[i]CmdQueue, where i=0,1
 * Location :MEMC
 * Buffer Size : 5
 * No of Buffers in a queue : 2
 * 2.uPAudRec[i]BitstreamQueue, where i=0,1
 * Location : MEMC
 * Buffer Size : 3
 * No of buffers in a queue : 3
 */

/*
 * Commands on uPAudRec[i]CmdQueue, where i=0,1
 */

/*
 * Command to configure memory for enabled encoder
 */

#define AUDREC_CMD_MEM_CFG_CMD 0x0000
#define AUDREC_CMD_ARECMEM_CFG_LEN	\
	sizeof(struct audrec_cmd_arecmem_cfg)

struct audrec_cmd_arecmem_cfg {
	unsigned short cmd_id;
	unsigned short audrec_up_pkt_intm_count;
	unsigned short audrec_ext_pkt_start_addr_msw;
	unsigned short audrec_ext_pkt_start_addr_lsw;
	unsigned short audrec_ext_pkt_buf_number;
} __attribute__((packed));

/*
 * Commands on uPAudRec[i]BitstreamQueue, where i=0,1
 */

/*
 * Command to indicate current packet read count
 */

#define UP_AUDREC_PACKET_EXT_PTR 0x0000
#define UP_AUDREC_PACKET_EXT_PTR_LEN	\
	sizeof(up_audrec_packet_ext_ptr)

struct up_audrec_packet_ext_ptr {
	unsigned short cmd_id;
	unsigned short audrec_up_curr_read_count_lsw;
	unsigned short audrec_up_curr_read_count_msw;
} __attribute__((packed));

#endif /* QDSP5AUDRECCMDI_H */
