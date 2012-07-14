/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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

#include <linux/pm_qos_params.h>
#include <mach/camera.h>
#define MSM_AXI_QOS_NAME "msm_camera"

static struct pm_qos_request_list *pm_qos_req;

int add_axi_qos(void)
{
	pm_qos_req = pm_qos_add_request(PM_QOS_SYSTEM_BUS_FREQ,
					PM_QOS_DEFAULT_VALUE);
	if (!pm_qos_req) {
		CDBG("request AXI bus QOS fails.\n");
		return -1;
	}

CDBG("request AXI bus QOS succeed.\n");
	return 0;
}

int update_axi_qos(uint32_t rate)
{
	if (!pm_qos_req) {
		CDBG("add_axi_qos() has not been called\n");
		return -1;
	}

	pm_qos_update_request(pm_qos_req, rate);
	return 0;
}

void release_axi_qos(void)
{
	if (!pm_qos_req) {
		CDBG("add_axi_qos() has not been called\n");
	}

	pm_qos_remove_request(pm_qos_req);
}
