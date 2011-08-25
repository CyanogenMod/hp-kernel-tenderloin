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

#ifndef __DAL_H__
#define __DAL_H__

#include <linux/kernel.h>
#include <mach/msm_smd.h>

#define DALRPC_DEST_MODEM SMD_APPS_MODEM
#define DALRPC_DEST_QDSP SMD_APPS_QDSP

#define DALRPC_TIMEOUT_INFINITE -1

enum {
	DALDEVICE_ATTACH_IDX = 0,
	DALDEVICE_DETACH_IDX,
	DALDEVICE_INIT_IDX,
	DALDEVICE_DEINIT_IDX,
	DALDEVICE_OPEN_IDX,
	DALDEVICE_CLOSE_IDX,
	DALDEVICE_INFO_IDX,
	DALDEVICE_POWEREVENT_IDX,
	DALDEVICE_SYSREQUEST_IDX,
	DALDEVICE_FIRST_DEVICE_API_IDX
};

struct daldevice_info_t {
	uint32_t size;
	uint32_t version;
	char name[32];
};

#define DAL_CHUNK_NAME_LENGTH 12
struct dal_chunk_header {
	uint32_t size;
	char name[DAL_CHUNK_NAME_LENGTH];
	uint32_t lock;
	uint32_t reserved;
	uint32_t type;
	uint32_t version;
};

int daldevice_attach(uint32_t device_id, char *port, int cpu,
		     void **handle_ptr);

/* The caller must ensure there are no outstanding dalrpc calls on
 * the client before (and while) calling daldevice_detach. */
int daldevice_detach(void *handle);

uint32_t dalrpc_fcn_0(uint32_t ddi_idx, void *handle, uint32_t s1);
uint32_t dalrpc_fcn_1(uint32_t ddi_idx, void *handle, uint32_t s1,
		      uint32_t s2);
uint32_t dalrpc_fcn_2(uint32_t ddi_idx, void *handle, uint32_t s1,
		      uint32_t *p_s2);
uint32_t dalrpc_fcn_3(uint32_t ddi_idx, void *handle, uint32_t s1,
		      uint32_t s2, uint32_t s3);
uint32_t dalrpc_fcn_4(uint32_t ddi_idx, void *handle, uint32_t s1,
		      uint32_t s2, uint32_t *p_s3);
uint32_t dalrpc_fcn_5(uint32_t ddi_idx, void *handle, const void *ibuf,
		      uint32_t ilen);
uint32_t dalrpc_fcn_6(uint32_t ddi_idx, void *handle, uint32_t s1,
		      const void *ibuf, uint32_t ilen);
uint32_t dalrpc_fcn_7(uint32_t ddi_idx, void *handle, const void *ibuf,
		      uint32_t ilen, void *obuf, uint32_t olen,
		      uint32_t *oalen);
uint32_t dalrpc_fcn_8(uint32_t ddi_idx, void *handle, const void *ibuf,
		      uint32_t ilen, void *obuf, uint32_t olen);
uint32_t dalrpc_fcn_9(uint32_t ddi_idx, void *handle, void *obuf,
		      uint32_t olen);
uint32_t dalrpc_fcn_10(uint32_t ddi_idx, void *handle, uint32_t s1,
		       const void *ibuf, uint32_t ilen, void *obuf,
		       uint32_t olen, uint32_t *oalen);
uint32_t dalrpc_fcn_11(uint32_t ddi_idx, void *handle, uint32_t s1,
		       void *obuf, uint32_t olen);
uint32_t dalrpc_fcn_12(uint32_t ddi_idx, void *handle, uint32_t s1,
		       void *obuf, uint32_t olen, uint32_t *oalen);
uint32_t dalrpc_fcn_13(uint32_t ddi_idx, void *handle, const void *ibuf,
		       uint32_t ilen, const void *ibuf2, uint32_t ilen2,
		       void *obuf, uint32_t olen);
uint32_t dalrpc_fcn_14(uint32_t ddi_idx, void *handle, const void *ibuf,
		       uint32_t ilen, void *obuf1, uint32_t olen1,
		       void *obuf2, uint32_t olen2, uint32_t *oalen2);
uint32_t dalrpc_fcn_15(uint32_t ddi_idx, void *handle, const void *ibuf,
		       uint32_t ilen, const void *ibuf2, uint32_t ilen2,
		       void *obuf, uint32_t olen, uint32_t *oalen,
		       void *obuf2, uint32_t olen2);

static inline uint32_t daldevice_info(void *handle,
				      struct daldevice_info_t *info,
				      uint32_t info_size)
{
	return dalrpc_fcn_9(DALDEVICE_INFO_IDX, handle, info, info_size);
}

static inline uint32_t daldevice_sysrequest(void *handle, uint32_t req_id,
					    const void *src_ptr,
					    uint32_t src_len, void *dest_ptr,
					    uint32_t dest_len,
					    uint32_t *dest_alen)
{
	return dalrpc_fcn_10(DALDEVICE_SYSREQUEST_IDX, handle, req_id,
			     src_ptr, src_len, dest_ptr, dest_len, dest_alen);
}

static inline uint32_t daldevice_init(void *handle)
{
	return dalrpc_fcn_0(DALDEVICE_INIT_IDX, handle, 0);
}

static inline uint32_t daldevice_deinit(void *handle)
{
	return dalrpc_fcn_0(DALDEVICE_DEINIT_IDX, handle, 0);
}

static inline uint32_t daldevice_open(void *handle, uint32_t mode)
{
	return dalrpc_fcn_0(DALDEVICE_OPEN_IDX, handle, mode);
}

static inline uint32_t daldevice_close(void *handle)
{
	return dalrpc_fcn_0(DALDEVICE_CLOSE_IDX, handle, 0);
}

void *dalrpc_alloc_event(void *handle);
void *dalrpc_alloc_cb(void *handle,
		      void (*fn)(void *, uint32_t, void *, uint32_t),
		      void *context);
void dalrpc_dealloc_event(void *handle,
			  void *ev_h);
void dalrpc_dealloc_cb(void *handle,
		       void *cb_h);

#define dalrpc_event_wait(ev_h, timeout) \
	dalrpc_event_wait_multiple(1, &ev_h, timeout)

int dalrpc_event_wait_multiple(int num, void **ev_h, int timeout);

#endif /* __DAL_H__ */
