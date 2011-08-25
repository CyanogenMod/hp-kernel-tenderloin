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

#ifndef __ARCH_ARM_MACH_MSM_CPUIDLE_H
#define __ARCH_ARM_MACH_MSM_CPUIDLE_H

#include <linux/notifier.h>
#include "pm.h"

struct msm_cpuidle_state {
	unsigned int cpu;
	int state_nr;
	char *name;
	char *desc;
	enum msm_pm_sleep_mode mode_nr;
};

void msm_cpuidle_set_states(struct msm_cpuidle_state *states,
	int nr_states, struct msm_pm_platform_data *pm_data);

int msm_cpuidle_init(void);

#ifdef CONFIG_MSM_SLEEP_STATS
enum {
	MSM_CPUIDLE_STATE_ENTER,
	MSM_CPUIDLE_STATE_EXIT
};

int msm_cpuidle_register_notifier(unsigned int cpu,
		struct notifier_block *nb);
int msm_cpuidle_unregister_notifier(unsigned int cpu,
		struct notifier_block *nb);

int msm_idle_register_cb(void (*pre)(int, unsigned int),
			void (*post)(int, unsigned int));
#else
static inline int msm_cpuidle_register_notifier(unsigned int cpu,
		struct notifier_block *nb)
{ return -ENODEV; }
static inline int msm_cpuidle_unregister_notifier(unsigned int cpu,
		struct notifier_block *nb)
{ return -ENODEV; }

static inline int msm_idle_register_cb(void (*pre)(int, unsigned int),
			void (*post)(int, unsigned int))
{ return -ENODEV; }
#endif

#endif /* __ARCH_ARM_MACH_MSM_CPUIDLE_H */
