/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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
#include <linux/module.h>
#include <linux/spinlock.h>

#include <mach/internal_power_rail.h>

#include "proc_comm.h"

static DEFINE_SPINLOCK(power_rail_lock);

static struct internal_rail {
	uint32_t id;
	uint32_t mode;
} rails[] = {
	{ PWR_RAIL_GRP_CLK, PWR_RAIL_CTL_AUTO },
	{ PWR_RAIL_GRP_2D_CLK, PWR_RAIL_CTL_AUTO },
	{ PWR_RAIL_MDP_CLK, PWR_RAIL_CTL_MANUAL },
	{ PWR_RAIL_MFC_CLK, PWR_RAIL_CTL_AUTO },
	{ PWR_RAIL_ROTATOR_CLK, PWR_RAIL_CTL_AUTO },
	{ PWR_RAIL_VDC_CLK, PWR_RAIL_CTL_AUTO },
	{ PWR_RAIL_VFE_CLK, PWR_RAIL_CTL_AUTO },
	{ PWR_RAIL_VPE_CLK, PWR_RAIL_CTL_AUTO },
};

static struct internal_rail *find_rail(unsigned rail_id)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(rails); i++)
		if (rails[i].id == rail_id)
			return rails + i;

	return NULL;
}

/* Enable or disable an internal power rail */
int internal_pwr_rail_ctl(unsigned rail_id, bool enable)
{
	int cmd, rc;

	cmd = enable ? 	PCOM_CLKCTL_RPC_RAIL_ENABLE :
			PCOM_CLKCTL_RPC_RAIL_DISABLE;

	rc = msm_proc_comm(cmd, &rail_id, NULL);

	return rc;

}
EXPORT_SYMBOL(internal_pwr_rail_ctl);

/* Enable or disable a rail if the rail is in auto mode. */
int internal_pwr_rail_ctl_auto(unsigned rail_id, bool enable)
{
	int rc = 0;
	unsigned long flags;
	struct internal_rail *rail = find_rail(rail_id);

	BUG_ON(!rail);

	spin_lock_irqsave(&power_rail_lock, flags);
	if (rail->mode == PWR_RAIL_CTL_AUTO)
		rc = internal_pwr_rail_ctl(rail_id, enable);
	spin_unlock_irqrestore(&power_rail_lock, flags);

	return rc;
}

/* Specify an internal power rail control mode (ex. auto, manual) */
int internal_pwr_rail_mode(unsigned rail_id, enum rail_ctl_mode mode)
{
	int rc;
	unsigned long flags;
	struct internal_rail *rail = find_rail(rail_id);

	spin_lock_irqsave(&power_rail_lock, flags);
	rc = msm_proc_comm(PCOM_CLKCTL_RPC_RAIL_CONTROL, &rail_id, &mode);
	if (rc)
		goto out;
	if (rail_id) {
		rc = -EINVAL;
		goto out;
	}

	if (rail)
		rail->mode = mode;
out:
	spin_unlock_irqrestore(&power_rail_lock, flags);
	return rc;
}
EXPORT_SYMBOL(internal_pwr_rail_mode);

