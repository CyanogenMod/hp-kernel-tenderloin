/*
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007-2011, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/stddef.h>
#include <linux/spinlock.h>

#include <mach/clk.h>

#include "proc_comm.h"
#include "clock.h"
#include "clock-pcom.h"

struct clk_pcom {
	unsigned count;
	bool always_on;
};

static struct clk_pcom pcom_clocks[P_NR_CLKS] = {
	[P_EBI1_CLK] = { .always_on = true },
	[P_PBUS_CLK] = { .always_on = true },
};

static DEFINE_SPINLOCK(pc_clk_lock);

/*
 * glue for the proc_comm interface
 */
int pc_clk_enable(unsigned id)
{
	int rc;
	unsigned long flags;
	struct clk_pcom *clk = &pcom_clocks[id];

	if (clk->always_on)
		return 0;

	spin_lock_irqsave(&pc_clk_lock, flags);
	if (clk->count == 0) {
		rc = msm_proc_comm(PCOM_CLKCTL_RPC_ENABLE, &id, NULL);
		if (rc < 0)
			goto unlock;
		else if ((int)id < 0) {
			rc = -EINVAL;
			goto unlock;
		} else
			rc = 0;
	}
	clk->count++;
unlock:
	spin_unlock_irqrestore(&pc_clk_lock, flags);
	return rc;
}

void pc_clk_disable(unsigned id)
{
	unsigned long flags;
	struct clk_pcom *clk = &pcom_clocks[id];

	if (clk->always_on)
		return;

	spin_lock_irqsave(&pc_clk_lock, flags);
	if (WARN_ON(clk->count == 0))
		goto out;
	clk->count--;
	if (clk->count == 0)
		msm_proc_comm(PCOM_CLKCTL_RPC_DISABLE, &id, NULL);
out:
	spin_unlock_irqrestore(&pc_clk_lock, flags);
}

void pc_clk_auto_off(unsigned id)
{
	unsigned long flags;
	struct clk_pcom *clk = &pcom_clocks[id];

	spin_lock_irqsave(&pc_clk_lock, flags);
	if (clk->count == 0)
		msm_proc_comm(PCOM_CLKCTL_RPC_DISABLE, &id, NULL);
	spin_unlock_irqrestore(&pc_clk_lock, flags);
}

int pc_clk_reset(unsigned id, enum clk_reset_action action)
{
	int rc;

	if (action == CLK_RESET_ASSERT)
		rc = msm_proc_comm(PCOM_CLKCTL_RPC_RESET_ASSERT, &id, NULL);
	else
		rc = msm_proc_comm(PCOM_CLKCTL_RPC_RESET_DEASSERT, &id, NULL);

	if (rc < 0)
		return rc;
	else
		return (int)id < 0 ? -EINVAL : 0;
}

int pc_clk_set_rate(unsigned id, unsigned rate)
{
	/* The rate _might_ be rounded off to the nearest KHz value by the
	 * remote function. So a return value of 0 doesn't necessarily mean
	 * that the exact rate was set successfully.
	 */
	int rc = msm_proc_comm(PCOM_CLKCTL_RPC_SET_RATE, &id, &rate);
	if (rc < 0)
		return rc;
	else
		return (int)id < 0 ? -EINVAL : 0;
}

int pc_clk_set_min_rate(unsigned id, unsigned rate)
{
	int rc = msm_proc_comm(PCOM_CLKCTL_RPC_MIN_RATE, &id, &rate);
	if (rc < 0)
		return rc;
	else
		return (int)id < 0 ? -EINVAL : 0;
}

int pc_clk_set_max_rate(unsigned id, unsigned rate)
{
	int rc = msm_proc_comm(PCOM_CLKCTL_RPC_MAX_RATE, &id, &rate);
	if (rc < 0)
		return rc;
	else
		return (int)id < 0 ? -EINVAL : 0;
}

int pc_clk_set_flags(unsigned id, unsigned flags)
{
	int rc = msm_proc_comm(PCOM_CLKCTL_RPC_SET_FLAGS, &id, &flags);
	if (rc < 0)
		return rc;
	else
		return (int)id < 0 ? -EINVAL : 0;
}

unsigned pc_clk_get_rate(unsigned id)
{
	if (msm_proc_comm(PCOM_CLKCTL_RPC_RATE, &id, NULL))
		return 0;
	else
		return id;
}

unsigned pc_clk_is_enabled(unsigned id)
{
	if (msm_proc_comm(PCOM_CLKCTL_RPC_ENABLED, &id, NULL))
		return 0;
	else
		return id;
}

long pc_clk_round_rate(unsigned id, unsigned rate)
{

	/* Not really supported; pc_clk_set_rate() does rounding on it's own. */
	return rate;
}

struct clk_ops clk_ops_remote = {
	.enable = pc_clk_enable,
	.disable = pc_clk_disable,
	.auto_off = pc_clk_auto_off,
	.reset = pc_clk_reset,
	.set_rate = pc_clk_set_rate,
	.set_min_rate = pc_clk_set_min_rate,
	.set_max_rate = pc_clk_set_max_rate,
	.set_flags = pc_clk_set_flags,
	.get_rate = pc_clk_get_rate,
	.is_enabled = pc_clk_is_enabled,
	.round_rate = pc_clk_round_rate,
};

int pc_clk_set_rate2(unsigned id, unsigned rate)
{
	return pc_clk_set_rate(id, rate / 2);
}

int pc_clk_set_min_rate2(unsigned id, unsigned rate)
{
	return pc_clk_set_min_rate(id, rate / 2);
}

unsigned pc_clk_get_rate2(unsigned id)
{
	return pc_clk_get_rate(id) * 2;
}

struct clk_ops clk_ops_pcom_div2 = {
	.enable = pc_clk_enable,
	.disable = pc_clk_disable,
	.auto_off = pc_clk_auto_off,
	.reset = pc_clk_reset,
	.set_rate = pc_clk_set_rate2,
	.set_min_rate = pc_clk_set_min_rate2,
	.set_flags = pc_clk_set_flags,
	.get_rate = pc_clk_get_rate2,
	.is_enabled = pc_clk_is_enabled,
	.round_rate = pc_clk_round_rate,
};
