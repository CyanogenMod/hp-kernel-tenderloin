/*
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#include <linux/module.h>

#include "cpuidle.h"
#include "idle.h"
#include "pm.h"

void arch_idle(void)
{ }

void msm_cpuidle_set_states(struct msm_cpuidle_state *states,
	int nr_states, struct msm_pm_platform_data *pm_data)
{ }

void msm_pm_set_platform_data(struct msm_pm_platform_data *data, int count)
{ }

void msm_pm_set_max_sleep_time(int64_t max_sleep_time_ns) { }
EXPORT_SYMBOL(msm_pm_set_max_sleep_time);
