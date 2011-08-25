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
#ifndef _MACH_QDSP5_V2_VOICE_H
#define _MACH_QDSP5_V2_VOICE_H

#define VOICE_DALRPC_DEVICEID 0x02000075
#define VOICE_DALRPC_PORT_NAME "DAL00"
#define VOICE_DALRPC_CPU 0


/* Commands sent to Modem */
#define CMD_VOICE_INIT                  0x1
#define CMD_ACQUIRE_DONE                0x2
#define CMD_RELEASE_DONE                0x3
#define CMD_DEVICE_INFO                 0x4
#define CMD_DEVICE_CHANGE               0x6

/* EVENTS received from MODEM */
#define EVENT_ACQUIRE_START             0x51
#define EVENT_RELEASE_START             0x52
#define EVENT_CHANGE_START              0x54
#define EVENT_NETWORK_RECONFIG          0x53

/* voice state */
enum {
	VOICE_INIT = 0,
	VOICE_ACQUIRE,
	VOICE_CHANGE,
	VOICE_RELEASE,
};

enum {
	NETWORK_CDMA = 0,
	NETWORK_GSM,
	NETWORK_WCDMA,
	NETWORK_WCDMA_WB,
};

enum {
	VOICE_DALRPC_CMD = DALDEVICE_FIRST_DEVICE_API_IDX
};

/* device state */
enum {
	DEV_INIT = 0,
	DEV_READY,
	DEV_CHANGE,
	DEV_CONCUR,
	DEV_REL_DONE,
};

/* Voice Event */
enum{
	VOICE_RELEASE_START = 1,
	VOICE_CHANGE_START,
	VOICE_ACQUIRE_START,
	VOICE_NETWORK_RECONFIG,
};

/* Device Event */
#define DEV_CHANGE_READY                0x1

#define VOICE_CALL_START	0x1
#define VOICE_CALL_END		0

#define VOICE_DEV_ENABLED	0x1
#define VOICE_DEV_DISABLED	0

struct voice_header {
	uint32_t id;
	uint32_t data_len;
};

struct voice_init {
	struct voice_header hdr;
	void *cb_handle;
};


/* Device information payload structure */
struct voice_device {
	struct voice_header hdr;
	uint32_t rx_device;
	uint32_t tx_device;
	uint32_t rx_volume;
	uint32_t rx_mute;
	uint32_t tx_mute;
	uint32_t rx_sample;
	uint32_t tx_sample;
};

/*Voice command structure*/
struct voice_network {
	struct voice_header hdr;
	uint32_t network_info;
};

struct device_data {
	uint32_t dev_acdb_id;
	uint32_t volume; /* in percentage */
	uint32_t mute;
	uint32_t sample;
	uint32_t enabled;
	uint32_t dev_id;
};

#endif
