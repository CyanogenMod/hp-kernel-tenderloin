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

#ifndef __KGSL_CMDSTREAM_H
#define __KGSL_CMDSTREAM_H

#include <linux/msm_kgsl.h>
#include "kgsl_device.h"

#ifdef KGSL_DEVICE_SHADOW_MEMSTORE_TO_USER
#define KGSL_CMDSTREAM_USE_MEM_TIMESTAMP
#endif /* KGSL_DEVICE_SHADOW_MEMSTORE_TO_USER */

#ifdef KGSL_CMDSTREAM_USE_MEM_TIMESTAMP
#define KGSL_CMDSTREAM_GET_SOP_TIMESTAMP(device, data) 	\
		kgsl_sharedmem_readl(&device->memstore, (data),	\
				KGSL_DEVICE_MEMSTORE_OFFSET(soptimestamp))
#else
#define KGSL_CMDSTREAM_GET_SOP_TIMESTAMP(device, data)	\
		kgsl_yamato_regread(device, REG_CP_TIMESTAMP, (data))
#endif /* KGSL_CMDSTREAM_USE_MEM_TIMESTAMP */

#define KGSL_CMDSTREAM_GET_EOP_TIMESTAMP(device, data)	\
		kgsl_sharedmem_readl(&device->memstore, (data),	\
				KGSL_DEVICE_MEMSTORE_OFFSET(eoptimestamp))

/* Flags to control command packet settings */
#define KGSL_CMD_FLAGS_PMODE			0x00000001
#define KGSL_CMD_FLAGS_NO_TS_CMP		0x00000002

struct kgsl_mem_entry;

int kgsl_cmdstream_init(struct kgsl_device *device);

int kgsl_cmdstream_close(struct kgsl_device *device);

void kgsl_cmdstream_memqueue_drain(struct kgsl_device *device);

uint32_t
kgsl_cmdstream_readtimestamp(struct kgsl_device *device,
			     enum kgsl_timestamp_type type);

void kgsl_cmdstream_memqueue_cleanup(struct kgsl_device *device,
				     struct kgsl_process_private *private);

void
kgsl_cmdstream_freememontimestamp(struct kgsl_device *device,
				  struct kgsl_mem_entry *entry,
				  uint32_t timestamp,
				  enum kgsl_timestamp_type type);

void kgsl_cmdstream_memqueue_cleanup(struct kgsl_device *device,
				     struct kgsl_process_private *private);

static inline bool timestamp_cmp(unsigned int new, unsigned int old)
{
	int ts_diff = new - old;
	return (ts_diff >= 0) || (ts_diff < -20000);
}

#endif /* __KGSL_CMDSTREAM_H */
