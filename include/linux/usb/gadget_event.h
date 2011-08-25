/*
 *  gadget_event.h
 *
 *  Copyright (C) 2008-2009  Palm, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _LINUX_USB_GADGET_EVENT_H
#define _LINUX_USB_GADGET_EVENT_H

enum gadget_event_source_type {
	G_EV_SOURCE_UNKNOWN = -1,
	G_EV_SOURCE_NONE = 0,
	G_EV_SOURCE_BUS = 1,
	G_EV_SOURCE_CHARGER = 2,
	G_EV_SOURCE_DETECTED = 3,
};

extern void gadget_event_enable_storage_events(int enable);
extern void gadget_event_host_connected(int host_connected);
extern void gadget_event_host_connected_async(int host_connected, unsigned long delay);
extern void gadget_event_bus_suspended(int bus_suspended);
extern void gadget_event_media_loaded(int media_loaded);
extern void gadget_event_media_requested(int media_requested);
extern void gadget_event_power_state_changed(enum gadget_event_source_type source, int current_mA);

#endif /* _LINUX_USB_GADGET_EVENT_H */
