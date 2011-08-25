/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#ifndef __SDIO_SMEM_H
#define __SDIO_SMEM_H

#include <linux/platform_device.h>
#include <linux/types.h>

#define SDIO_SMEM_EVENT_READ_DONE	0
#define SDIO_SMEM_EVENT_READ_ERR	1

int sdio_smem_register_client(void);

struct sdio_smem_client {
	void *buf;
	int size;
	struct platform_device plat_dev;
	int (*cb_func)(int event);
};

#endif	/* __SDIO_SMEM_H */
