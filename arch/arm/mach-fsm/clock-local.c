/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <mach/msm_iomap.h>
#include <mach/clk.h>

#include "clock.h"
#include "clock-local.h"

/* When enabling/disabling a clock, check the halt bit up to this number
 * number of times (with a 1 us delay in between) before continuing. */
#define HALT_CHECK_MAX_LOOPS	100
/* For clock without halt checking, wait this long after enables/disables. */
#define HALT_CHECK_DELAY_US	10

DEFINE_SPINLOCK(local_clock_reg_lock);
struct clk_freq_tbl local_dummy_freq = F_END;

#define MAX_SOURCES 20
static int src_votes[MAX_SOURCES];
static DEFINE_SPINLOCK(src_vote_lock);

unsigned local_sys_vdd_votes[NUM_SYS_VDD_LEVELS];
static DEFINE_SPINLOCK(sys_vdd_vote_lock);

static int local_clk_enable_nolock(unsigned id);
static int local_clk_disable_nolock(unsigned id);
static int local_src_enable_nolock(int src);
static int local_src_disable_nolock(int src);

/*
 * Common Set-Rate Functions
 */
/* For clocks with integer dividers only. */
void set_rate_basic(struct clk_local *clk, struct clk_freq_tbl *nf)
{
	uint32_t reg_val;

	reg_val = readl(clk->ns_reg);
	reg_val &= ~(clk->ns_mask);
	reg_val |= nf->ns_val;
	writel(reg_val, clk->ns_reg);
}

/* For clocks with MND dividers. */
void set_rate_mnd(struct clk_local *clk, struct clk_freq_tbl *nf)
{
	uint32_t ns_reg_val, cc_reg_val;

	/* Assert MND reset. */
	ns_reg_val = readl(clk->ns_reg);
	ns_reg_val |= B(7);
	writel(ns_reg_val, clk->ns_reg);

	/* Program M and D values. */
	writel(nf->md_val, clk->md_reg);

	/* Program NS register. */
	ns_reg_val &= ~(clk->ns_mask);
	ns_reg_val |= nf->ns_val;
	writel(ns_reg_val, clk->ns_reg);

	/* If the clock has a separate CC register, program it. */
	if (clk->ns_reg != clk->cc_reg) {
		cc_reg_val = readl(clk->cc_reg);
		cc_reg_val &= ~(clk->cc_mask);
		cc_reg_val |= nf->cc_val;
		writel(cc_reg_val, clk->cc_reg);
	}

	/* Deassert MND reset. */
	ns_reg_val &= ~B(7);
	writel(ns_reg_val, clk->ns_reg);
}

void set_rate_nop(struct clk_local *clk, struct clk_freq_tbl *nf)
{
	/* Nothing to do for fixed-rate clocks. */
}

/*
 * SYS_VDD voting functions
 */

/* Update system voltage level given the current votes. */
static int local_update_sys_vdd(void)
{
	static int cur_level;
	int level, rc = 0;

	if (local_sys_vdd_votes[HIGH])
		level = HIGH;
	else if (local_sys_vdd_votes[NOMINAL])
		level = NOMINAL;
	else
		level = LOW;

	if (level == cur_level)
		return rc;

	rc = soc_update_sys_vdd(level);
	if (!rc)
		cur_level = level;

	return rc;
}

/* Vote for a system voltage level. */
int local_vote_sys_vdd(unsigned level)
{
	int rc = 0;
	unsigned long flags;

	/* Bounds checking. */
	if (level >= ARRAY_SIZE(local_sys_vdd_votes))
		return -EINVAL;

	spin_lock_irqsave(&sys_vdd_vote_lock, flags);
	local_sys_vdd_votes[level]++;
	rc = local_update_sys_vdd();
	if (rc)
		local_sys_vdd_votes[level]--;
	spin_unlock_irqrestore(&sys_vdd_vote_lock, flags);

	return rc;
}

/* Remove vote for a system voltage level. */
int local_unvote_sys_vdd(unsigned level)
{
	int rc = 0;
	unsigned long flags;

	/* Bounds checking. */
	if (level >= ARRAY_SIZE(local_sys_vdd_votes))
		return -EINVAL;

	spin_lock_irqsave(&sys_vdd_vote_lock, flags);
	if (local_sys_vdd_votes[level])
		local_sys_vdd_votes[level]--;
	else {
		pr_warning("%s: Reference counts are incorrect for level %d!\n",
			__func__, level);
		goto out;
	}

	rc = local_update_sys_vdd();
	if (rc)
		local_sys_vdd_votes[level]++;
out:
	spin_unlock_irqrestore(&sys_vdd_vote_lock, flags);
	return rc;
}

/*
 * Clock source (PLL/XO) control functions
 */

/* Enable clock source without taking the lock. */
static int local_src_enable_nolock(int src)
{
	int rc = 0;

	if (!src_votes[src]) {
		if (soc_clk_sources[src].par != SRC_NONE)
			rc = local_src_enable_nolock(soc_clk_sources[src].par);
			if (rc)
				goto err_par;
		/* Perform source-specific enable operations. */
		if (soc_clk_sources[src].enable_func)
			rc = soc_clk_sources[src].enable_func(src, 1);
			if (rc)
				goto err_enable;
	}
	src_votes[src]++;

	return rc;

err_enable:
	if (soc_clk_sources[src].par != SRC_NONE)
		local_src_disable_nolock(soc_clk_sources[src].par);
err_par:
	return rc;
}

/* Enable clock source. */
int local_src_enable(int src)
{
	int rc = 0;
	unsigned long flags;

	if (src == SRC_NONE)
		return rc;

	spin_lock_irqsave(&src_vote_lock, flags);
	rc = local_src_enable_nolock(src);
	spin_unlock_irqrestore(&src_vote_lock, flags);

	return rc;
}

/* Disable clock source without taking the lock. */
static int local_src_disable_nolock(int src)
{
	int rc = 0;

	if (src_votes[src] > 0)
		src_votes[src]--;
	else {
		pr_warning("%s: Reference counts are incorrect for "
			   "src %d!\n", __func__, src);
		return rc;
	}

	if (src_votes[src] == 0) {
		/* Perform source-specific disable operations. */
		if (soc_clk_sources[src].enable_func)
			rc = soc_clk_sources[src].enable_func(src, 0);
			if (rc)
				goto err_disable;
		if (soc_clk_sources[src].par != SRC_NONE)
			rc = local_src_disable_nolock(soc_clk_sources[src].par);
			if (rc)
				goto err_disable_par;

	}

	return rc;

err_disable_par:
	soc_clk_sources[src].enable_func(src, 1);
err_disable:
	src_votes[src]++;
	return rc;
}

/* Disable clock source. */
int local_src_disable(int src)
{
	int rc = 0;
	unsigned long flags;

	if (src == SRC_NONE)
		return rc;

	spin_lock_irqsave(&src_vote_lock, flags);
	rc = local_src_disable_nolock(src);
	spin_unlock_irqrestore(&src_vote_lock, flags);

	return rc;
}

/*
 * Clock enable/disable functions
 */

/* Return non-zero if a clock status registers shows the clock is halted. */
static int local_clk_is_halted(unsigned id)
{
	struct clk_local *clk = &soc_clk_local_tbl[id];
	int invert = (clk->halt_check == ENABLE);
	int status_bit = readl(clk->halt_reg) & B(clk->halt_bit);
	return invert ? !status_bit : status_bit;
}

/* Perform any register operations required to enable the clock. */
void local_clk_enable_reg(unsigned id)
{
	struct clk_local *clk = &soc_clk_local_tbl[id];
	void *reg = clk->cc_reg;
	uint32_t reg_val;

	WARN((clk->type != NORATE) && (clk->current_freq == &local_dummy_freq),
		"Attempting to enable clock %d before setting its rate. "
		"Set the rate first!\n", id);

	/* Enable MN counter, if applicable. */
	reg_val = readl(reg);
	if (clk->type == MND) {
		reg_val |= clk->current_freq->mnd_en_mask;
		writel(reg_val, reg);
	}
	/* Enable root. */
	if (clk->root_en_mask) {
		reg_val |= clk->root_en_mask;
		writel(reg_val, reg);
	}
	/* Enable branch. */
	if (clk->br_en_mask) {
		reg_val |= clk->br_en_mask;
		writel(reg_val, reg);
	}

	/* Wait for clock to enable before returning. */
	if (clk->halt_check == DELAY)
		udelay(HALT_CHECK_DELAY_US);
	else if (clk->halt_check == ENABLE || clk->halt_check == HALT) {
		int count;
		/* Use a memory barrier since some halt status registers are
		 * not within the same 1K segment as the branch/root enable
		 * registers. */
		mb();

		/* Wait up to HALT_CHECK_MAX_LOOPS for clock to enable. */
		for (count = HALT_CHECK_MAX_LOOPS; local_clk_is_halted(id)
					&& count > 0; count--)
			udelay(1);
		if (count == 0)
			pr_warning("%s: clock %d status bit stuck off\n",
				   __func__, id);
	}
}

/* Perform any register operations required to enable the clock. */
void local_clk_disable_reg(unsigned id)
{
	struct clk_local *clk = &soc_clk_local_tbl[id];
	void *reg = clk->cc_reg;
	uint32_t reg_val;

	/* Disable branch. */
	reg_val = readl(reg);
	if (clk->br_en_mask) {
		reg_val &= ~(clk->br_en_mask);
		writel(reg_val, reg);
	}

	/* Wait for clock to disable before continuing. */
	if (clk->halt_check == DELAY)
		udelay(HALT_CHECK_DELAY_US);
	else if (clk->halt_check == ENABLE || clk->halt_check == HALT) {
		int count;
		/* Use a memory barrier since some halt status registers are
		 * not within the same 1K segment as the branch/root enable
		 * registers. */
		mb();

		/* Wait up to HALT_CHECK_MAX_LOOPS for clock to disable. */
		for (count = HALT_CHECK_MAX_LOOPS; !local_clk_is_halted(id)
					&& count > 0; count--)
			udelay(1);
		if (count == 0)
			pr_warning("%s: clock %d status bit stuck on\n",
				   __func__, id);
	}

	/* Disable root. */
	if (clk->root_en_mask) {
		reg_val &= ~(clk->root_en_mask);
		writel(reg_val, reg);
	}
	/* Disable MN counter, if applicable. */
	if (clk->type == MND) {
		reg_val &= ~(clk->current_freq->mnd_en_mask);
		writel(reg_val, reg);
	}
}

/* Enable a clock with no locking, enabling parent clocks as needed. */
static int local_clk_enable_nolock(unsigned id)
{
	struct clk_local *clk = &soc_clk_local_tbl[id];
	int rc = 0;

	if (clk->type == RESET)
		return -EPERM;

	if (!clk->count) {
		rc = local_vote_sys_vdd(clk->current_freq->sys_vdd);
		if (rc)
			goto err_vdd;
		if (clk->parent != C(NONE)) {
			rc = local_clk_enable_nolock(clk->parent);
			if (rc)
				goto err_par;
		}
		rc = local_src_enable(clk->current_freq->src);
		if (rc)
			goto err_src;
		local_clk_enable_reg(id);
	}
	clk->count++;

	return rc;

err_src:
	if (clk->parent != C(NONE))
		rc = local_clk_disable_nolock(clk->parent);
err_par:
	local_unvote_sys_vdd(clk->current_freq->sys_vdd);
err_vdd:
	return rc;
}

/* Disable a clock with no locking, disabling unused parents, too. */
static int local_clk_disable_nolock(unsigned id)
{
	struct clk_local *clk = &soc_clk_local_tbl[id];
	int rc = 0;

	if (clk->count > 0)
		clk->count--;
	else {
		pr_warning("%s: Reference counts are incorrect for clock %d!\n",
			__func__, id);
		return rc;
	}

	if (clk->count == 0) {
		local_clk_disable_reg(id);
		rc = local_src_disable(clk->current_freq->src);
		if (rc)
			goto err_src;
		if (clk->parent != C(NONE))
			rc = local_clk_disable_nolock(clk->parent);
			if (rc)
				goto err_par;
		rc = local_unvote_sys_vdd(clk->current_freq->sys_vdd);
		if (rc)
			goto err_vdd;
	}

	return rc;

err_vdd:
	if (clk->parent != C(NONE))
		rc = local_clk_enable_nolock(clk->parent);
err_par:
	local_src_enable(clk->current_freq->src);
err_src:
	local_clk_enable_reg(id);
	clk->count++;

	return rc;
}

/* Enable a clock and any related power rail. */
int local_clk_enable(unsigned id)
{
	int rc = 0;
	unsigned long flags;

	spin_lock_irqsave(&local_clock_reg_lock, flags);
	rc = local_clk_enable_nolock(id);
	if (rc)
		goto unlock;
	/*
	 * With remote rail control, the remote processor might modify
	 * the clock control register when the rail is enabled/disabled.
	 * Enable the rail inside the lock to protect against this.
	 */
	rc = soc_set_pwr_rail(id, 1);
	if (rc)
		local_clk_disable_nolock(id);
unlock:
	spin_unlock_irqrestore(&local_clock_reg_lock, flags);

	return rc;
}

/* Disable a clock and any related power rail. */
void local_clk_disable(unsigned id)
{
	unsigned long flags;

	spin_lock_irqsave(&local_clock_reg_lock, flags);
	soc_set_pwr_rail(id, 0);
	local_clk_disable_nolock(id);
	spin_unlock_irqrestore(&local_clock_reg_lock, flags);

	return;
}

/* Turn off a clock at boot, without checking refcounts or disabling parents. */
void local_clk_auto_off(unsigned id)
{
	unsigned long flags;

	spin_lock_irqsave(&local_clock_reg_lock, flags);
	local_clk_disable_reg(id);
	spin_unlock_irqrestore(&local_clock_reg_lock, flags);
}

/*
 * Frequency-related functions
 */

/* Set a clock's frequency. */
static int _local_clk_set_rate(unsigned id, struct clk_freq_tbl *nf)
{
	struct clk_local *clk = &soc_clk_local_tbl[id];
	struct clk_freq_tbl *cf;
	const int32_t *chld = clk->children;
	int i, rc = 0;
	unsigned long flags;

	spin_lock_irqsave(&local_clock_reg_lock, flags);

	/* Check if frequency is actually changed. */
	cf = clk->current_freq;
	if (nf == cf)
		goto release_lock;

	/* Disable branch if clock isn't dual-banked with a glitch-free MUX. */
	if (clk->banked_mnd_masks == NULL) {
		/* Disable all branches to prevent glitches. */
		for (i = 0; chld && chld[i] != C(NONE); i++) {
			struct clk_local *ch = &soc_clk_local_tbl[chld[i]];
			/* Don't bother turning off if it is already off.
			 * Checking ch->count is cheaper (cache) than reading
			 * and writing to a register (uncached/unbuffered). */
			if (ch->count)
				local_clk_disable_reg(chld[i]);
		}
		if (clk->count)
			local_clk_disable_reg(id);
	}

	if (clk->count) {
		/* Vote for voltage and source for new freq. */
		rc = local_vote_sys_vdd(nf->sys_vdd);
		if (rc)
			goto sys_vdd_vote_failed;
		rc = local_src_enable(nf->src);
		if (rc) {
			local_unvote_sys_vdd(nf->sys_vdd);
			goto src_enable_failed;
		}
	}

	/* Perform clock-specific frequency switch operations. */
	BUG_ON(!clk->set_rate);
	clk->set_rate(clk, nf);

	/* Release requirements of the old freq. */
	if (clk->count) {
		local_src_disable(cf->src);
		local_unvote_sys_vdd(cf->sys_vdd);
	}

	/* Current freq must be updated before local_clk_enable_reg()
	 * is called to make sure the MNCNTR_EN bit is set correctly. */
	clk->current_freq = nf;

src_enable_failed:
sys_vdd_vote_failed:
	/* Enable any clocks that were disabled. */
	if (clk->banked_mnd_masks == NULL) {
		if (clk->count)
			local_clk_enable_reg(id);
		/* Enable only branches that were ON before. */
		for (i = 0; chld && chld[i] != C(NONE); i++) {
			struct clk_local *ch = &soc_clk_local_tbl[chld[i]];
			if (ch->count)
				local_clk_enable_reg(chld[i]);
		}
	}

release_lock:
	spin_unlock_irqrestore(&local_clock_reg_lock, flags);
	return rc;
}

/* Set a clock to an exact rate. */
int local_clk_set_rate(unsigned id, unsigned rate)
{
	struct clk_local *clk = &soc_clk_local_tbl[id];
	struct clk_freq_tbl *nf;

	if (clk->type == NORATE || clk->type == RESET)
		return -EPERM;

	for (nf = clk->freq_tbl; nf->freq_hz != FREQ_END
			&& nf->freq_hz != rate; nf++)
		;

	if (nf->freq_hz == FREQ_END)
		return -EINVAL;

	return _local_clk_set_rate(id, nf);
}

/* Set a clock to a rate greater than some minimum. */
int local_clk_set_min_rate(unsigned id, unsigned rate)
{
	struct clk_local *clk = &soc_clk_local_tbl[id];
	struct clk_freq_tbl *nf;

	if (clk->type == NORATE || clk->type == RESET)
		return -EPERM;

	for (nf = clk->freq_tbl; nf->freq_hz != FREQ_END
			&& nf->freq_hz < rate; nf++)
		;

	if (nf->freq_hz == FREQ_END)
		return -EINVAL;

	return _local_clk_set_rate(id, nf);
}

/* Set a clock to a maximum rate. */
int local_clk_set_max_rate(unsigned id, unsigned rate)
{
	return -EPERM;
}

/* Get the currently-set rate of a clock in Hz. */
unsigned local_clk_get_rate(unsigned id)
{
	struct clk_local *clk = &soc_clk_local_tbl[id];
	unsigned long flags;
	unsigned ret = 0;

	if (clk->type == NORATE || clk->type == RESET)
		return 0;

	spin_lock_irqsave(&local_clock_reg_lock, flags);
	ret = clk->current_freq->freq_hz;
	spin_unlock_irqrestore(&local_clock_reg_lock, flags);

	/* Return 0 if the rate has never been set. Might not be correct,
	 * but it's good enough. */
	if (ret == FREQ_END)
		ret = 0;

	return ret;
}

/* Check if a clock is currently enabled. */
unsigned local_clk_is_enabled(unsigned id)
{
	struct clk_local *clk = &soc_clk_local_tbl[id];

	if (clk->type == RESET)
		return -EPERM;

	return !!(soc_clk_local_tbl[id].count);
}

/* Return a supported rate that's at least the specified rate. */
long local_clk_round_rate(unsigned id, unsigned rate)
{
	struct clk_local *clk = &soc_clk_local_tbl[id];
	struct clk_freq_tbl *f;

	if (clk->type == NORATE || clk->type == RESET)
		return -EINVAL;

	for (f = clk->freq_tbl; f->freq_hz != FREQ_END; f++)
		if (f->freq_hz >= rate)
			return f->freq_hz;

	return -EPERM;
}

/* Return the nth supported frequency for a given clock. */
int local_clk_list_rate(unsigned id, unsigned n)
{
	struct clk_local *clk = &soc_clk_local_tbl[id];

	if (!clk->freq_tbl || clk->freq_tbl->freq_hz == FREQ_END)
		return -ENXIO;

	return (clk->freq_tbl + n)->freq_hz;
}

