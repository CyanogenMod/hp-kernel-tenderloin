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

#include "clock.h"
#include "clock-rpm.h"

int rpm_clk_enable(unsigned id)
{
	/* Not yet supported. */
	return -EPERM;
}

void rpm_clk_disable(unsigned id)
{
	/* Not yet supported. */
}

int rpm_clk_reset(unsigned id, enum clk_reset_action action)
{
	/* Not yet supported. */
	return -EPERM;
}

int rpm_clk_set_rate(unsigned id, unsigned rate)
{
	/* Not yet supported. */
	return -EPERM;
}

int rpm_clk_set_min_rate(unsigned id, unsigned rate)
{
	 /*
	  * XXX Temporary until real ebi1_clk control is available from the RPM.
	  */
	if (id == R_EBI1_CLK)
		return 0;

	/* Not yet supported. */
	return -EPERM;
}

int rpm_clk_set_max_rate(unsigned id, unsigned rate)
{
	/* Not yet supported. */
	return -EPERM;
}

int rpm_clk_set_flags(unsigned id, unsigned flags)
{
	/* Not yet supported. */
	return -EPERM;
}

unsigned rpm_clk_get_rate(unsigned id)
{
	/* Not yet supported. */
	return 0;
}

signed rpm_clk_measure_rate(unsigned id)
{
	/* Not supported. */
	return -EPERM;
}

unsigned rpm_clk_is_enabled(unsigned id)
{
	/* Not yet supported. */
	return 0;
}

long rpm_clk_round_rate(unsigned id, unsigned rate)
{
	/* Not yet supported. */
	return rate;
}

struct clk_ops clk_ops_remote = {
	.enable = rpm_clk_enable,
	.disable = rpm_clk_disable,
	.auto_off = rpm_clk_disable,
	.reset = rpm_clk_reset,
	.set_rate = rpm_clk_set_rate,
	.set_min_rate = rpm_clk_set_min_rate,
	.set_max_rate = rpm_clk_set_max_rate,
	.set_flags = rpm_clk_set_flags,
	.get_rate = rpm_clk_get_rate,
	.measure_rate = rpm_clk_measure_rate,
	.is_enabled = rpm_clk_is_enabled,
	.round_rate = rpm_clk_round_rate,
};
