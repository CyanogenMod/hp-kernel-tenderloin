/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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

#ifndef DIAGCHAR_HDLC
#define DIAGCHAR_HDLC

enum diag_send_state_enum_type {
	DIAG_STATE_START,
	DIAG_STATE_BUSY,
	DIAG_STATE_CRC1,
	DIAG_STATE_CRC2,
	DIAG_STATE_TERM,
	DIAG_STATE_COMPLETE
};

struct diag_send_desc_type {
	const void *pkt;
	const void *last;	/* Address of last byte to send. */
	enum diag_send_state_enum_type state;
	unsigned char terminate;	/* True if this fragment
					   terminates the packet */
};

struct diag_hdlc_dest_type {
	void *dest;
	void *dest_last;
	/* Below: internal use only */
	uint16_t crc;
};

struct diag_hdlc_decode_type {
	uint8_t *src_ptr;
	unsigned int src_idx;
	unsigned int src_size;
	uint8_t *dest_ptr;
	unsigned int dest_idx;
	unsigned int dest_size;
	int escaping;

};

void diag_hdlc_encode(struct diag_send_desc_type *src_desc,
		      struct diag_hdlc_dest_type *enc);

int diag_hdlc_decode(struct diag_hdlc_decode_type *hdlc);

#define ESC_CHAR     0x7D
#define CONTROL_CHAR 0x7E
#define ESC_MASK     0x20

#endif
