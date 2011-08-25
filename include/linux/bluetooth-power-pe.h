/*
 * linux/include/bluetooth-power-pe.h
 *
 * Bluetooth power state driver - Used by the user space to indicate
 * the current power on status of the Bluetooth.
 * Copyright (C) 2009 Palm, Inc.
 * Author: Rajmohan Mani <rajmohan.mani@palm.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License.
 */

#ifndef __BLUETOOTH_POWER_STATE_PLATFORM_H__ 
#define __BLUETOOTH_POWER_STATE_PLATFORM_H__  

#define BLUETOOTH_POWER_STATE_DEVICE		"bt_power"
#define BLUETOOTH_POWER_STATE_DRIVER		"bt_power"

struct bluetooth_power_state_platform_data {
	char   *dev_name;
	int (* bt_power) (unsigned int on);
};

#endif /* __BLUETOOTH_POWER_STATE_PLATFORM_H__  */ 
