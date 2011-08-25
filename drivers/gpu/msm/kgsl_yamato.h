/* Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
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
#ifndef _KGSL_YAMATO_H
#define _KGSL_YAMATO_H

#include "kgsl_drawctxt.h"
#include "kgsl_ringbuffer.h"

struct kgsl_yamato_device {
	struct kgsl_device dev;    /* Must be first field in this struct */
	struct kgsl_memregion gmemspace;
	struct kgsl_yamato_context *drawctxt_active;
	wait_queue_head_t ib1_wq;
	unsigned int *pfp_fw;
	size_t pfp_fw_size;
	unsigned int *pm4_fw;
	size_t pm4_fw_size;
	struct kgsl_ringbuffer ringbuffer;
};


irqreturn_t kgsl_yamato_isr(int irq, void *data);

int __init kgsl_yamato_init(struct kgsl_device *device);
int __init kgsl_yamato_init_pwrctrl(struct kgsl_device *device);

int kgsl_yamato_close(struct kgsl_device *device);

int kgsl_yamato_idle(struct kgsl_device *device, unsigned int timeout);
void kgsl_yamato_regread(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int *value);
void kgsl_yamato_regwrite(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int value);
void kgsl_yamato_regread_isr(struct kgsl_device *device,
			     unsigned int offsetwords,
			     unsigned int *value);
void kgsl_yamato_regwrite_isr(struct kgsl_device *device,
			      unsigned int offsetwords,
			      unsigned int value);
struct kgsl_device *kgsl_get_yamato_generic_device(void);
int kgsl_yamato_getfunctable(struct kgsl_functable *ftbl);

#endif /*_KGSL_YAMATO_H */
