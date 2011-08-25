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
#ifndef _VCD_DEVICE_SM_H_
#define _VCD_DEVICE_SM_H_

#include "vcd_api.h"
#include "vcd_ddl_api.h"
#include "vcd_core.h"

struct vcd_dev_state_table;
struct vcd_dev_state_ctxt;
struct vcd_drv_ctxt;

enum vcd_dev_state_enum {
	VCD_DEVICE_STATE_NULL = 0,
	VCD_DEVICE_STATE_NOT_INIT,
	VCD_DEVICE_STATE_INITING,
	VCD_DEVICE_STATE_READY,
	VCD_DEVICE_STATE_INVALID,
	VCD_DEVICE_STATE_MAX,
	VCD_DEVICE_STATE_32BIT = 0x7FFFFFFF
};

struct vcd_dev_state_table {
	struct {
		u32(*init) (struct vcd_drv_ctxt *drv_ctxt,
				struct vcd_init_config *config,
				s32 *driver_handle);

		u32(*term) (struct vcd_drv_ctxt *drv_ctxt,
				s32 driver_handle);

		u32(*open) (struct vcd_drv_ctxt *drv_ctxt,
				s32 driver_handle, u32 decoding,
				void (*callback) (u32 event, u32 status,
					void *info, size_t sz, void *handle,
					void *const client_data),
				void *client_data);

		u32(*close) (struct vcd_drv_ctxt *drv_ctxt,
				struct vcd_clnt_ctxt *cctxt);

		u32(*resume) (struct vcd_drv_ctxt *drv_ctxt,
				struct vcd_clnt_ctxt *cctxt);

		u32(*set_dev_pwr) (struct vcd_drv_ctxt *drv_ctxt,
				enum vcd_power_state pwr_state);

		void (*dev_cb) (struct vcd_drv_ctxt *drv_ctxt,
				u32 event, u32 status, void *payload,
				size_t sz, u32 *ddl_handle,
				void *const client_data);

		void (*timeout) (struct vcd_drv_ctxt *drv_ctxt,
							void *user_data);
	} ev_hdlr;

	void (*entry) (struct vcd_drv_ctxt *drv_ctxt,
			s32 state_event);
	void (*exit) (struct vcd_drv_ctxt *drv_ctxt,
			s32 state_event);
};

#define   DEVICE_STATE_EVENT_NUMBER(ppf) \
	((u32 *) (&(((struct vcd_dev_state_table*)0)->ev_hdlr.ppf)) - \
	(u32 *) (&(((struct vcd_dev_state_table*)0)->ev_hdlr.init)) \
	+ 1)

struct vcd_dev_state_ctxt {
	const struct vcd_dev_state_table *state_table;

	enum vcd_dev_state_enum state;
};

struct vcd_drv_ctxt {
	struct vcd_dev_state_ctxt dev_state;
	struct vcd_dev_ctxt dev_ctxt;
	struct mutex dev_mutex;
};


extern struct vcd_drv_ctxt *vcd_get_drv_context(void);

void vcd_continue(void);

#endif
