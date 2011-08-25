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

#include <linux/types.h>
#include <linux/skbuff.h>

#ifndef _SDIO_DMUX_H
#define _SDIO_DMUX_H

enum {
	SDIO_DMUX_DATA_RMNET_0,
	SDIO_DMUX_DATA_RMNET_1,
	SDIO_DMUX_DATA_RMNET_2,
	SDIO_DMUX_DATA_RMNET_3,
	SDIO_DMUX_DATA_RMNET_4,
	SDIO_DMUX_DATA_RMNET_5,
	SDIO_DMUX_DATA_RMNET_6,
	SDIO_DMUX_DATA_RMNET_7,
	SDIO_DMUX_USB_RMNET_0,
	SDIO_DMUX_NUM_CHANNELS
};

int msm_sdio_dmux_open(uint32_t id, void *priv,
		       void (*receive_cb)(void *, struct sk_buff *),
		       void (*write_done)(void *, struct sk_buff *));

int msm_sdio_dmux_close(uint32_t id);

int msm_sdio_dmux_write(uint32_t id, struct sk_buff *skb);

#endif /* _SDIO_DMUX_H */
