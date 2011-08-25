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

#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/clk.h>

#include "clock.h"
#include "clock-voter.h"

struct clk_voter {
	unsigned count;
	unsigned rate;
	struct hlist_node voter_list;
	struct clk *aggregator_clk;
};

static struct clk_voter voter_clocks[V_NR_CLKS];

static DEFINE_SPINLOCK(voter_clk_lock);

/* Aggregate the rate of clocks that are currently on. */
static unsigned voter_clk_aggregate_rate(const struct clk *parent)
{
	struct hlist_node *pos;
	struct clk_voter *clkh;
	unsigned rate = 0;

	hlist_for_each_entry(clkh, pos, &parent->voters, voter_list)
		if (clkh->count)
			rate = max(clkh->rate, rate);
	return rate;
}

static int voter_clk_set_rate(unsigned id, unsigned rate)
{
	int ret = 0;
	unsigned long flags;
	struct hlist_node *pos;
	struct clk_voter *clkh;
	struct clk_voter *clk = &voter_clocks[id];
	unsigned other_rate = 0;
	unsigned cur_rate, new_rate;

	spin_lock_irqsave(&voter_clk_lock, flags);

	if (clk->count) {
		struct clk *parent = clk->aggregator_clk;

		/*
		 * Get the aggregate rate without this clock's vote and update
		 * if the new rate is different than the current rate
		 */
		hlist_for_each_entry(clkh, pos, &parent->voters, voter_list)
			if (clkh->count && clkh != clk)
				other_rate = max(clkh->rate, other_rate);

		cur_rate = max(other_rate, clk->rate);
		new_rate = max(other_rate, rate);

		if (new_rate != cur_rate) {
			ret = clk_set_min_rate(parent, new_rate);
			if (ret)
				goto unlock;
		}
	}
	clk->rate = rate;
unlock:
	spin_unlock_irqrestore(&voter_clk_lock, flags);

	return ret;
}

static int voter_clk_enable(unsigned id)
{
	int ret = 0;
	unsigned long flags;
	unsigned cur_rate;
	struct clk_voter *clk = &voter_clocks[id];

	spin_lock_irqsave(&voter_clk_lock, flags);
	if (clk->count == 0) {
		struct clk *parent = clk->aggregator_clk;

		/*
		 * Increase the rate if this clock is voting for a higher rate
		 * than the current rate.
		 */
		cur_rate = voter_clk_aggregate_rate(parent);
		if (clk->rate > cur_rate) {
			ret = clk_set_min_rate(parent, clk->rate);
			if (ret)
				goto out;
		}
		ret = clk_enable(parent);
		if (ret)
			goto out;
	}
	clk->count++;
out:
	spin_unlock_irqrestore(&voter_clk_lock, flags);

	return ret;
}

static void voter_clk_disable(unsigned id)
{
	unsigned long flags;
	struct clk_voter *clk = &voter_clocks[id];
	unsigned cur_rate, new_rate;

	spin_lock_irqsave(&voter_clk_lock, flags);
	if (WARN_ON(clk->count == 0))
		goto out;
	clk->count--;
	if (clk->count == 0) {
		struct clk *parent = clk->aggregator_clk;

		/*
		 * Decrease the rate if this clock was the only one voting for
		 * the highest rate.
		 */
		new_rate = voter_clk_aggregate_rate(parent);
		cur_rate = max(new_rate, clk->rate);

		if (new_rate < cur_rate)
			clk_set_min_rate(parent, new_rate);

		clk_disable(clk->aggregator_clk);
	}
out:
	spin_unlock_irqrestore(&voter_clk_lock, flags);
}

static unsigned voter_clk_get_rate(unsigned id)
{
	unsigned rate;
	unsigned long flags;
	struct clk_voter *clk = &voter_clocks[id];

	spin_lock_irqsave(&voter_clk_lock, flags);
	rate = clk->rate;
	spin_unlock_irqrestore(&voter_clk_lock, flags);

	return rate;
}

static unsigned voter_clk_is_enabled(unsigned id)
{
	struct clk_voter *clk = &voter_clocks[id];
	return clk->count;
}

static long voter_clk_round_rate(unsigned id, unsigned rate)
{
	struct clk_voter *clk = &voter_clocks[id];
	return clk_round_rate(clk->aggregator_clk, rate);
}

static int voter_clk_set_parent(unsigned id, struct clk *parent)
{
	unsigned long flags;
	struct clk_voter *clk = &voter_clocks[id];

	spin_lock_irqsave(&voter_clk_lock, flags);
	clk->aggregator_clk = parent;
	hlist_add_head(&clk->voter_list, &parent->voters);
	spin_unlock_irqrestore(&voter_clk_lock, flags);

	return 0;
}

struct clk_ops clk_ops_voter = {
	.enable = voter_clk_enable,
	.disable = voter_clk_disable,
	.set_rate = voter_clk_set_rate,
	.set_min_rate = voter_clk_set_rate,
	.get_rate = voter_clk_get_rate,
	.is_enabled = voter_clk_is_enabled,
	.round_rate = voter_clk_round_rate,
	.set_parent = voter_clk_set_parent,
};
