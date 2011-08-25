/* include/asm-arm/arch-msm/usbdiag.h
 *
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * All source code in this file is licensed under the following license except
 * where indicated.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 */

#ifndef _DRIVERS_USB_DIAG_H_
#define _DRIVERS_USB_DIAG_H_
#define ENOREQ -1
struct diag_request {
	char *buf;
	int length;
	int actual;
	int status;
	void *context;
};
struct diag_operations {

	int (*diag_connect)(void);
	int (*diag_disconnect)(void);
	int (*diag_char_write_complete)(struct diag_request *);
	int (*diag_char_read_complete)(struct diag_request *);
};

int diag_open(int);
void diag_close(void);
int diag_read(struct diag_request *);
int diag_write(struct diag_request *);

int diag_usb_register(struct diag_operations *);
int diag_usb_unregister(void);
int diag_read_from_cb(unsigned char * , int);
#endif
