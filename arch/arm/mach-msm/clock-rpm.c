/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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

#include <linux/err.h>
#include <mach/clk.h>

#include "rpm_resources.h"
#include "clock.h"
#include "clock-rpm.h"

static DEFINE_SPINLOCK(rpm_clock_lock);

#define R_CLK(id, name, sc, ao) \
	[(id)] = { \
		.rpm_clk_id = MSM_RPM_ID_##name##_CLK, \
		.rpm_status_id = MSM_RPM_STATUS_ID_##name##_CLK, \
		.peer_clk_id = (sc), \
		.active_only = (ao), \
	}
static struct rpm_clk {
	const int rpm_clk_id;
	const int rpm_status_id;
	const int peer_clk_id;
	const int active_only;
	unsigned last_set_khz;
	/* 0 if active_only. Otherwise, same as last_set_khz. */
	unsigned last_set_sleep_khz;
	int count;
} rpm_clk[] = {
	R_CLK(R_AFAB_CLK,    APPS_FABRIC,    R_AFAB_A_CLK,	0),
	R_CLK(R_CFPB_CLK,    CFPB,           R_CFPB_A_CLK,	0),
	R_CLK(R_DFAB_CLK,    DAYTONA_FABRIC, R_DFAB_A_CLK,	0),
	R_CLK(R_EBI1_CLK,    EBI1,           R_EBI1_A_CLK,	0),
	R_CLK(R_MMFAB_CLK,   MM_FABRIC,      R_MMFAB_A_CLK,	0),
	R_CLK(R_MMFPB_CLK,   MMFPB,          R_MMFPB_A_CLK,	0),
	R_CLK(R_SFAB_CLK,    SYSTEM_FABRIC,  R_SFAB_A_CLK,	0),
	R_CLK(R_SFPB_CLK,    SFPB,           R_SFPB_A_CLK,	0),
	R_CLK(R_SMI_CLK,     SMI,            R_SMI_A_CLK,	0),
	R_CLK(R_AFAB_A_CLK,  APPS_FABRIC,    R_AFAB_CLK,	1),
	R_CLK(R_CFPB_A_CLK,  CFPB,           R_CFPB_CLK,	1),
	R_CLK(R_DFAB_A_CLK,  DAYTONA_FABRIC, R_DFAB_CLK,	1),
	R_CLK(R_EBI1_A_CLK,  EBI1,           R_EBI1_CLK,	1),
	R_CLK(R_MMFAB_A_CLK, MM_FABRIC,      R_MMFAB_CLK,	1),
	R_CLK(R_MMFPB_A_CLK, MMFPB,          R_MMFPB_CLK,	1),
	R_CLK(R_SFAB_A_CLK,  SYSTEM_FABRIC,  R_SFAB_CLK,	1),
	R_CLK(R_SFPB_A_CLK,  SFPB,           R_SFPB_CLK,	1),
	R_CLK(R_SMI_A_CLK,   SMI,            R_SMI_CLK,		1),
};

static int rpm_clk_enable(unsigned id)
{
	unsigned long flags;
	int rc = 0;

	spin_lock_irqsave(&rpm_clock_lock, flags);

	/* Don't send requests to the RPM if the rate has not been set. */
	if (rpm_clk[id].last_set_khz == 0)
		goto out;

	if (!rpm_clk[id].count) {
		struct msm_rpm_iv_pair iv;
		unsigned this_khz = rpm_clk[id].last_set_khz;
		unsigned this_sleep_khz = rpm_clk[id].last_set_sleep_khz;
		unsigned peer_id = rpm_clk[id].peer_clk_id;
		unsigned peer_khz = 0, peer_sleep_khz = 0;

		iv.id = rpm_clk[id].rpm_clk_id;

		/* Take peer clock's rate into account only if it's enabled. */
		if (rpm_clk[peer_id].count) {
			peer_khz = rpm_clk[peer_id].last_set_khz;
			peer_sleep_khz = rpm_clk[peer_id].last_set_sleep_khz;
		}

		iv.value = max(this_khz, peer_khz);
		rc = msm_rpmrs_set_noirq(MSM_RPM_CTX_SET_0, &iv, 1);
		if (rc)
			goto out;

		iv.value = max(this_sleep_khz, peer_sleep_khz);
		rc = msm_rpmrs_set_noirq(MSM_RPM_CTX_SET_SLEEP, &iv, 1);
	}
out:
	if (!rc)
		rpm_clk[id].count++;

	spin_unlock_irqrestore(&rpm_clock_lock, flags);

	return rc;
}

static void rpm_clk_disable(unsigned id)
{
	unsigned long flags;

	spin_lock_irqsave(&rpm_clock_lock, flags);

	if (rpm_clk[id].count > 0)
		rpm_clk[id].count--;
	else {
		pr_warning("%s: Reference counts are incorrect for clock %d!\n",
			   __func__, id);
		goto out;
	}

	if (!rpm_clk[id].count && rpm_clk[id].last_set_khz) {
		struct msm_rpm_iv_pair iv;
		unsigned peer_id = rpm_clk[id].peer_clk_id;
		unsigned peer_khz = 0, peer_sleep_khz = 0;
		int rc;

		iv.id = rpm_clk[id].rpm_clk_id;

		/* Take peer clock's rate into account only if it's enabled. */
		if (rpm_clk[peer_id].count) {
			peer_khz = rpm_clk[peer_id].last_set_khz;
			peer_sleep_khz = rpm_clk[peer_id].last_set_sleep_khz;
		}

		iv.value = peer_khz;
		rc = msm_rpmrs_set_noirq(MSM_RPM_CTX_SET_0, &iv, 1);
		if (rc)
			goto out;

		iv.value = peer_sleep_khz;
		rc = msm_rpmrs_set_noirq(MSM_RPM_CTX_SET_SLEEP, &iv, 1);
	}

out:
	spin_unlock_irqrestore(&rpm_clock_lock, flags);

	return;
}

static void rpm_clk_auto_off(unsigned id)
{
	/* Not supported */
}

static int rpm_clk_reset(unsigned id, enum clk_reset_action action)
{
	/* Not supported. */
	return -EPERM;
}

static int rpm_clk_set_rate(unsigned id, unsigned rate)
{
	/* Not supported. */
	return -EPERM;
}

static int rpm_clk_set_min_rate(unsigned id, unsigned rate)
{
	unsigned long flags;
	unsigned this_khz, this_sleep_khz;
	int rc = 0;

	this_khz = DIV_ROUND_UP(rate, 1000);

	spin_lock_irqsave(&rpm_clock_lock, flags);

	/* Ignore duplicate requests. */
	if (rpm_clk[id].last_set_khz == this_khz)
		goto out;

	/* Active-only clocks don't care what the rate is during sleep. So,
	 * they vote for zero. */
	if (rpm_clk[id].active_only)
		this_sleep_khz = 0;
	else
		this_sleep_khz = this_khz;

	if (rpm_clk[id].count) {
		struct msm_rpm_iv_pair iv;
		unsigned peer_id = rpm_clk[id].peer_clk_id;
		unsigned peer_khz = 0, peer_sleep_khz = 0;

		iv.id = rpm_clk[id].rpm_clk_id;

		/* Take peer clock's rate into account only if it's enabled. */
		if (rpm_clk[peer_id].count) {
			peer_khz = rpm_clk[peer_id].last_set_khz;
			peer_sleep_khz = rpm_clk[peer_id].last_set_sleep_khz;
		}

		iv.value = max(this_khz, peer_khz);
		rc = msm_rpmrs_set_noirq(MSM_RPM_CTX_SET_0, &iv, 1);
		if (rc)
			goto out;

		iv.value = max(this_sleep_khz, peer_sleep_khz);
		rc = msm_rpmrs_set_noirq(MSM_RPM_CTX_SET_SLEEP, &iv, 1);
	}
	if (!rc) {
		rpm_clk[id].last_set_khz = this_khz;
		rpm_clk[id].last_set_sleep_khz = this_sleep_khz;
	}

out:
	spin_unlock_irqrestore(&rpm_clock_lock, flags);

	return rc;
}

static int rpm_clk_set_max_rate(unsigned id, unsigned rate)
{
	/* Not supported. */
	return -EPERM;
}

static int rpm_clk_set_flags(unsigned id, unsigned flags)
{
	/* Not supported. */
	return -EPERM;
}

static unsigned rpm_clk_get_rate(unsigned id)
{
	struct msm_rpm_iv_pair iv = { rpm_clk[id].rpm_status_id };
	int rc;

	rc  = msm_rpm_get_status(&iv, 1);
	if (rc < 0)
		return rc;
	return iv.value * 1000;
}

static unsigned rpm_clk_is_enabled(unsigned id)
{
	return !!(rpm_clk_get_rate(id));
}

static long rpm_clk_round_rate(unsigned id, unsigned rate)
{
	/* Not supported. */
	return rate;
}

struct clk_ops clk_ops_remote = {
	.enable = rpm_clk_enable,
	.disable = rpm_clk_disable,
	.auto_off = rpm_clk_auto_off,
	.reset = rpm_clk_reset,
	.set_rate = rpm_clk_set_rate,
	.set_min_rate = rpm_clk_set_min_rate,
	.set_max_rate = rpm_clk_set_max_rate,
	.set_flags = rpm_clk_set_flags,
	.get_rate = rpm_clk_get_rate,
	.is_enabled = rpm_clk_is_enabled,
	.round_rate = rpm_clk_round_rate,
};
