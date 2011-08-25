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

#ifndef __ARCH_ARM_MACH_MSM_IDLE_STATS_H
#define __ARCH_ARM_MACH_MSM_IDLE_STATS_H

#include <linux/types.h>
#include <linux/ioctl.h>

enum msm_idle_stats_event {
	MSM_IDLE_STATS_EVENT_BUSY_TIMER_EXPIRED = 1,
	MSM_IDLE_STATS_EVENT_COLLECTION_TIMER_EXPIRED = 2,
	MSM_IDLE_STATS_EVENT_COLLECTION_FULL = 3,
	MSM_IDLE_STATS_EVENT_TIMER_MIGRATED = 4,
};

/*
 * All time, timer, and time interval values are in units of
 * microseconds unless stated otherwise.
 */
#define MSM_IDLE_STATS_NR_MAX_INTERVALS 100
#define MSM_IDLE_STATS_MAX_TIMER 1000000

struct msm_idle_stats {
	__u32 busy_timer;
	__u32 collection_timer;

	__u32 busy_intervals[MSM_IDLE_STATS_NR_MAX_INTERVALS];
	__u32 idle_intervals[MSM_IDLE_STATS_NR_MAX_INTERVALS];
	__u32 nr_collected;
	__s64 last_busy_start;
	__s64 last_idle_start;

	enum msm_idle_stats_event event;
	__s64 return_timestamp;
};

#define MSM_IDLE_STATS_IOC_MAGIC  0xD8
#define MSM_IDLE_STATS_IOC_COLLECT  \
		_IOWR(MSM_IDLE_STATS_IOC_MAGIC, 1, struct msm_idle_stats)

#endif  /* __ARCH_ARM_MACH_MSM_IDLE_STATS_H */
