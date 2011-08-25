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

#ifndef _VCD_DDL_UTILS_H_
#define _VCD_DDL_UTILS_H_

#include <linux/delay.h>
#include "vidc_type.h"

#ifdef DDL_MSG_LOG
#define DDL_MSG_LOW(x...)    printk(KERN_INFO x)
#define DDL_MSG_MED(x...)    printk(KERN_INFO x)
#define DDL_MSG_HIGH(x...)   printk(KERN_INFO x)
#else
#define DDL_MSG_LOW(x...)
#define DDL_MSG_MED(x...)
#define DDL_MSG_HIGH(x...)
#endif

#define DDL_MSG_ERROR(x...)  printk(KERN_INFO x)
#define DDL_MSG_FATAL(x...)  printk(KERN_INFO x)

#define DDL_ALIGN_SIZE(sz, guard_bytes, align_mask) \
	(((u32)(sz) + guard_bytes) & align_mask)
#define DDL_ADDR_IS_ALIGNED(addr, align_bytes) \
	(!((u32)(addr) & ((align_bytes) - 1)))
#define  DDL_ALIGN(val, grid) ((!(grid)) ? (val) : \
		((((val) + (grid) - 1) / (grid)) * (grid)))
#define  DDL_ALIGN_FLOOR(val, grid) ((!(grid)) ? (val) : \
		(((val) / (grid)) * (grid)))
#define DDL_OFFSET(base, addr) ((!(addr)) ? 0 : (u32)((u8 *) \
		(addr) - (u8 *) (base)))
#define DDL_ADDR_OFFSET(base, addr) DDL_OFFSET((base).align_physical_addr, \
		(addr).align_physical_addr)
#define DDL_GET_ALIGNED_VITUAL(x)   ((x).align_virtual_addr)
#define DDL_KILO_BYTE(x)   ((x)*1024)
#define DDL_MEGA_BYTE(x)   ((x)*1024*1024)
#define DDL_FRAMERATE_SCALE(x)            ((x) * 1000)

#define DDL_MIN(x, y)  ((x < y) ? x : y)
#define DDL_MAX(x, y)  ((x > y) ? x : y)

#ifdef DDL_PROFILE
void ddl_get_core_start_time(u8 codec);
void ddl_calc_core_time(u8 codec);
void ddl_reset_time_variables(u8 codec);
#endif

#endif
