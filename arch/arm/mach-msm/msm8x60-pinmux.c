/* arch/arm/mach-msm/mux.c
 *
 * shank pin muxing
 *
 * Copyright (C) 2009 Palm, Inc.
 * Author: Dmitry Fink <dmitry.fink@palm.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/pinmux.h>

#include <mach/gpio.h>

#include "msm8x60-pinmux.h"

struct pin_config *msm8x60_pins = NULL;
int msm8x60_pins_sz = 0;

static struct pin_config *msm8x60_get_pin_config(const char *name)
{
	int i;

	for(i=0; i < msm8x60_pins_sz; i++)
	{
		if(0 == strcmp(msm8x60_pins[i].name, name))
		{
			return &msm8x60_pins[i];
		}
	}
	return NULL;
}

static int msm8x60_get_cfg(struct pin_config *pin_config, pinmux_cfg cfg,
	unsigned int *mux_mode)
{
	switch (cfg) {
	case PINMUX_CONFIG_ACTIVE:
		*mux_mode = pin_config->active_mode;
		return 0;
	case PINMUX_CONFIG_SLEEP:
		*mux_mode = pin_config->sleep_mode;
		return 0;
	}

	return -EINVAL;
}

static int msm8x60_set_cfg(struct pin_config *pin_config, pinmux_cfg cfg)
{
	int retval;
	unsigned int mux_cfg;

	if (PINMUX_CONFIG_POWER_COLLAPSE == cfg) {
		cfg = pin_config->active_power_collapse ?
			PINMUX_CONFIG_ACTIVE : PINMUX_CONFIG_SLEEP;
	}

	retval = msm8x60_get_cfg(pin_config, cfg, &mux_cfg);
	if (retval)
		return retval;

	gpio_tlmm_config(mux_cfg, cfg == PINMUX_CONFIG_ACTIVE ? GPIO_CFG_ENABLE : GPIO_CFG_DISABLE);
	pin_config->current_mode = cfg;
	return 0;
}

static int msm8x60_set_mux(const char *name, pinmux_cfg cfg)
{
	struct pin_config *pin_config;

	if(!name) {
		printk("msm8x60_set_cfg: name cannot be null\n");
		return -EINVAL;
	}

	pin_config = msm8x60_get_pin_config(name);
	if (!pin_config)
		goto error;

	if (msm8x60_set_cfg(pin_config, cfg))
		goto error;

	return 0;
error:
	printk("msm8x60_set_cfg: error configuring %s\n", name);
	return -EINVAL;
}

static int msm8x60_set_power_collapse(const char *name, int active)
{
	struct pin_config *pin_config;

	pin_config = msm8x60_get_pin_config(name);
	if (!pin_config)
		return -EINVAL;

	pin_config->active_power_collapse = active;
	return 0;
}

static void msm8x60_dump_pin_table(void)
{
	int i;

	for(i=0; i < msm8x60_pins_sz; i++)
	{
		printk("%s: active=0x%x, sleep=0x%x, power_collapse=%s\n",
			msm8x60_pins[i].name,
			msm8x60_pins[i].active_mode, msm8x60_pins[i].sleep_mode,
			msm8x60_pins[i].active_power_collapse ? "active" : "sleep");
	}
}

struct pinmux_ops msm8x60_pinmux_ops =
{
	.config = msm8x60_set_mux,
	.dump = msm8x60_dump_pin_table,
	.set_power_collapse = msm8x60_set_power_collapse,
};
