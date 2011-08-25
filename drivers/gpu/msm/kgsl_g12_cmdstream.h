/* Copyright (c) 2002,2007-2010, Code Aurora Forum. All rights reserved.
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
#ifndef _GSL_CMDSTREAM_H
#define _GSL_CMDSTREAM_H

#include <linux/msm_kgsl.h>

struct kgsl_device;
struct kgsl_device_private;
struct kgsl_context;

int kgsl_g12_cmdstream_init(struct kgsl_device *device);

int kgsl_g12_cmdstream_start(struct kgsl_device *device);

void kgsl_g12_cmdstream_close(struct kgsl_device *device);

unsigned int kgsl_g12_cmdstream_readtimestamp(struct kgsl_device *device,
					enum kgsl_timestamp_type unused);
int kgsl_g12_cmdstream_issueibcmds(struct kgsl_device_private *dev_priv,
			struct kgsl_context *context,
			struct kgsl_ibdesc *ibdesc,
			unsigned int numibs,
			uint32_t *timestamp,
			unsigned int ctrl);
#endif  /* _GSL_CMDSTREAM_H */
