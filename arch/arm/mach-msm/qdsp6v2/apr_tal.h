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
 *
 */
#ifndef __APR_TAL_H_
#define __APR_TAL_H_

#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>

/* APR Client IDs */
#define APR_CLIENT_AUDIO	0x0
#define APR_CLIENT_VOICE	0x1
#define APR_CLIENT_MAX	0x2

#define APR_DL_SMD    0
#define APR_DL_MAX    1

#define APR_DEST_MODEM 0
#define APR_DEST_QDSP6 1
#define APR_DEST_MAX   2

#define APR_MAX_BUF   8192

typedef void (*apr_svc_cb_fn)(void *buf, int len, void *priv);
struct apr_svc_ch_dev *apr_tal_open(uint32_t svc, uint32_t dest,
			uint32_t dl, apr_svc_cb_fn func, void *priv);
int apr_tal_write(struct apr_svc_ch_dev *apr_ch, void *data, int len);
int apr_tal_close(struct apr_svc_ch_dev *apr_ch);
struct apr_svc_ch_dev {
	struct smd_channel *ch;
	spinlock_t         lock;
	spinlock_t         w_lock;
	struct mutex       m_lock;
	apr_svc_cb_fn      func;
	char               data[APR_MAX_BUF];
	wait_queue_head_t  wait;
	void               *priv;
	uint32_t           smd_state;
	wait_queue_head_t  dest;
	uint32_t           dest_state;
};

#endif
