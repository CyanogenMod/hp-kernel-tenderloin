/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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

#ifndef __KGSL_CFFDUMP_H
#define __KGSL_CFFDUMP_H

#ifdef CONFIG_MSM_KGSL_CFF_DUMP

#include <linux/types.h>

#include "kgsl_device.h"

void kgsl_cffdump_init(void);
void kgsl_cffdump_destroy(void);
void kgsl_cffdump_open(enum kgsl_deviceid device_id);
void kgsl_cffdump_close(enum kgsl_deviceid device_id);
void kgsl_cffdump_syncmem(struct kgsl_device_private *dev_priv,
	const struct kgsl_memdesc *memdesc, uint physaddr, uint sizebytes,
	bool clean_cache);
void kgsl_cffdump_setmem(uint addr, uint value, uint sizebytes);
void kgsl_cffdump_regwrite(enum kgsl_deviceid device_id, uint addr,
	uint value);
void kgsl_cffdump_regpoll(enum kgsl_deviceid device_id, uint addr,
	uint value, uint mask);
bool kgsl_cffdump_parse_ibs(struct kgsl_device_private *dev_priv,
	const struct kgsl_memdesc *memdesc, uint gpuaddr, int sizedwords,
	bool check_only);
static inline bool kgsl_cffdump_flags_no_memzero(void) { return true; }

#else

#define kgsl_cffdump_init()					(void)0
#define kgsl_cffdump_destroy()					(void)0
#define kgsl_cffdump_open(device_id)				(void)0
#define kgsl_cffdump_close(device_id)				(void)0
#define kgsl_cffdump_syncmem(dev_priv, memdesc, addr, sizebytes, clean_cache) \
	(void) 0
#define kgsl_cffdump_setmem(addr, value, sizebytes)		(void)0
#define kgsl_cffdump_regwrite(device_id, addr, value)		(void)0
#define kgsl_cffdump_regpoll(device_id, addr, value, mask)	(void)0
#define kgsl_cffdump_parse_ibs(dev_priv, memdesc, gpuaddr, \
	sizedwords, check_only)					true
#define kgsl_cffdump_flags_no_memzero()				true

#endif /* CONFIG_MSM_KGSL_CFF_DUMP */

#endif /* __KGSL_CFFDUMP_H */
