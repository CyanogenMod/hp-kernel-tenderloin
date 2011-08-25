/* Copyright (c) 2010, Hewlett-Packard Development Company, L.P. All rights reserved.
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
#include <linux/platform_device.h>

#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>

#define GPIO_5V_BIAS     102

static struct regulator_consumer_supply vdd50_boost_supply =
	REGULATOR_SUPPLY("vdd50_boost", NULL);

static struct regulator_init_data vdd50_boost_init_data = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.min_uV = 5000000,
		.max_uV = 5000000,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &vdd50_boost_supply,
};

static struct fixed_voltage_config vdd50_boost_config = {
	.supply_name = "vdd50_boost",
	.microvolts = 5000000,
	.gpio = GPIO_5V_BIAS,
	.enable_high = 1,
	.init_data = &vdd50_boost_init_data,
};

struct platform_device tenderloin_fixed_reg_device[] = {
	{
		.name          = "reg-fixed-voltage",
		.id            = 0,
		.dev           = {
				.platform_data = &vdd50_boost_config
		}
	},
};

