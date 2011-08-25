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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <mach/msm_bus_board.h>
#include "msm_bus.h"

/* Stubbed out functions provided for API review */
int __init msm_bus_register_fabric_info(int id,
	struct msm_bus_node_info const *info, unsigned len)
{
	return -EIO;
}
EXPORT_SYMBOL(msm_bus_register_fabric_info);

uint32_t msm_bus_scale_register_client(struct msm_bus_scale_pdata *pdata)
{
	return -EIO;
}
EXPORT_SYMBOL(msm_bus_scale_register_client);

int msm_bus_scale_client_update_request(uint32_t cl, unsigned index)
{
	return -EIO;
}
EXPORT_SYMBOL(msm_bus_scale_client_update_request);

void msm_bus_scale_unregister_client(uint32_t cl)
{
}
EXPORT_SYMBOL(msm_bus_scale_unregister_client);

int msm_axi_client_port_halt(int master_port)
{
	return -EIO;
}
EXPORT_SYMBOL(msm_axi_client_port_halt);

int msm_axi_client_port_unhalt(int master_port)
{
	return -EIO;
}
EXPORT_SYMBOL(msm_axi_client_port_unhalt);

MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.2");
MODULE_ALIAS("platform:msm_bus");
