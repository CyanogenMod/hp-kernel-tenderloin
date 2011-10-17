/*
 * linux/include/linux/a6.h
 *
 * Driver for the A6 TP.
 *
 * Copyright (C) 2008 Palm, Inc.
 * Author: Raj Mojumder <raj.mojumder@palm.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License.
 */

#ifndef _A6_H
#define _A6_H

#include <linux/ioctl.h>
#include <linux/types.h>


#define  A6_DEVICE_0   "a6_0"
#define  A6_DEVICE_1   "a6_1"
#define  A6_DRIVER     "a6"

#define  A6_DEVICE     A6_DEVICE_0

#define MAX8903B_CONNECTED_PS_AC	(1U << 1)
#define MAX8903B_CONNECTED_PS_USB	(1U << 2)
#define MAX8903B_CONNECTED_PS_DOCK	(1U << 3)
#define MAX8903B_DOCK_DRAW_MA		1400

/* IOCTLs */
#define A6_IOCTL_SET_FW_DATA		_IOW('c', 0x01, int)
#define A6_IOCTL_VERIFY_FW_DATA		_IOW('c', 0x02, int)

/* Touch panel platform data structure */
struct a6_platform_data {
	char*	dev_name;   // device name
	int	pwr_gpio;
	int	sbw_tck_gpio;
	int 	sbw_tdio_gpio;
	int	sbw_wkup_gpio;
	void*	sbw_ops;
	void*	wake_ops;

	void*	sbw_init_gpio_config;
	int	sbw_init_gpio_config_size;
	void*	sbw_deinit_gpio_config;
	int	sbw_deinit_gpio_config_size;

	int	(*sbw_init)(struct a6_platform_data*);
	int	(*sbw_deinit)(struct a6_platform_data*);

	int	pwr_gpio_wakeup_cap;  /* set if pwr_gpio is wakeup capable */
	int	power_supply_connected;	/* Set to 1 if this is the a6 connected to battery, etc */
};

struct a6_wake_ops {
	void*	data;
	
	// external periodic sleep/wake interface
	int	(*enable_periodic_wake)(void *);
	int	(*disable_periodic_wake)(void *);

	// internal sleep/wake interface
	int	(*internal_wake_enable_state)(void*);
	int	(*internal_wake_period)(void*);

	// force sleep/wake interface (needed to force-wake A6 when
	// internal/external periodic sleep/wake in effect...
	int	(*force_wake)(void *);
	int	(*force_sleep)(void *);
};

void a6_charger_event (int otg_chg_type);

#endif // _A6_H
