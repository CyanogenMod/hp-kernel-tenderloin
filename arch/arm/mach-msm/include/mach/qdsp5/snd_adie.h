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
 */

#ifndef __SND_ADIE_SVC_H_
#define __SND_ADIE_SVC_H_

#define ADIE_SVC_PROG	0x30000002
#define ADIE_SVC_VERS	0x00020003

#define ADIE_SVC_CLIENT_STATUS_FUNC_PTR_TYPE_PROC 0xFFFFFF01
#define SND_ADIE_SVC_CLIENT_REGISTER_PROC 	34
#define SND_ADIE_SVC_CONFIG_ADIE_BLOCK_PROC 	35
#define SND_ADIE_SVC_CLIENT_DEREGISTER_PROC 	36

#define ADIE_SVC_MAX_CLIENTS 5

enum adie_svc_client_operation{
	ADIE_SVC_REGISTER_CLIENT,
	ADIE_SVC_DEREGISTER_CLIENT,
	ADIE_SVC_CONFIG_ADIE_BLOCK,
};

enum adie_svc_status_type{
	ADIE_SVC_STATUS_SUCCESS,
	ADIE_SVC_STATUS_FAILURE,
	ADIE_SVC_STATUS_INUSE
};

enum adie_block_enum_type{
	MIC_BIAS,
	HSSD,
	HPH_PA
};

enum adie_config_enum_type{
	DISABLE,
	ENABLE
};

struct adie_svc_client{
	int client_id;
	int cb_id;
	enum adie_svc_status_type status;
	bool adie_svc_cb_done;
	struct mutex lock;
	wait_queue_head_t wq;
	struct msm_rpc_client *rpc_client;
};

struct adie_svc_client_register_cb_cb_args {
	int cb_id;
	uint32_t size;
	int client_id;
	enum adie_block_enum_type adie_block;
	enum adie_svc_status_type status;
	enum adie_svc_client_operation client_operation;
};

struct adie_svc_client_register_cb_args {
	int cb_id;
};

struct adie_svc_client_deregister_cb_args {
	int client_id;
};

struct adie_svc_config_adie_block_cb_args {
	int client_id;
	enum adie_block_enum_type adie_block;
	enum adie_config_enum_type config;
};

int adie_svc_get(void);
int adie_svc_put(int id);
int adie_svc_config_adie_block(int id,
	enum adie_block_enum_type adie_block_type, bool enable);
#endif
