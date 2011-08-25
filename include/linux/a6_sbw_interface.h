/*
 * linux/include/linux/a6_sbw_interface.h
 *
 * Public interface for the SBW protocol layer. Declares callbacks used by the core protocol.
 * Interfaces include:
 * - per-A6-device interface: every A6 device must define its own implementation of this interface.
 * - per-target interfaces : each board-type must define its own implementation of these interfaces.
 * - per-host system: operating system specific implementations must be defined.
 *
 * Copyright (C) 2008 Palm, Inc.
 * Author: Raj Mojumder <raj.mojumder@palm.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License.
 */

#ifndef _A6_SBW_INTERFACE_H_
#define _A6_SBW_INTERFACE_H_

struct a6_sbw_interface {
        // per-A6-device interface (separate instantiation for every a6 device)
	struct {
		uint16_t (*SetSBWTCK)(void);
		uint16_t (*ClrSBWTCK)(void);
		uint16_t (*SetSBWTDIO)(void);
		uint16_t (*ClrSBWTDIO)(void);
		uint16_t (*SetInSBWTDIO)(void);
		uint16_t (*SetOutSBWTDIO)(void);
		uint16_t (*GetSBWTDIO)(void);
		uint16_t (*SetSBWAKEUP)(void);
		uint16_t (*ClrSBWAKEUP)(void);
	} a6_per_device_interface;

        // per-target interface (separate instantiation for every board)
	struct {
		void (*delay)(uint32_t delay_us);
	} a6_per_target_interface;
};


// per-host system: (operating system specific)
#ifdef __linux__
#define a6_disable_interrupts(flags) {flags=flags;local_irq_save(flags);}
#define a6_enable_interrupts(flags)  {local_irq_restore(flags);}
#else
#define a6_disable_interrupts(flags) {i_need_definition();}
#define a6_enable_interrupts(flags)  {i_need_definition();}
#endif


#endif // _A6_SBW_INTERFACE_H_
