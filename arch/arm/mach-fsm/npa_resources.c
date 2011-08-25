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

#include <linux/init.h>
#include <linux/pm_qos_params.h>
#include "msm_pm_qos.h"
#include "npa_remote.h"

#ifdef CONFIG_MSM_NPA_REMOTE

DECLARE_RESOURCE_REMOTE_AGGREGATION(
	npa_memory_node,
	npa_memory_resource,
	NPA_MEMORY_NODE_NAME,
	"", 2);

#ifdef CONFIG_MSM_NPA_SYSTEM_BUS
#define SYSTEM_BUS_NPA_RESOURCE_NAME "/bus/arbiter"
static struct pm_qos_plugin system_bus_plugin = {
	.data = SYSTEM_BUS_NPA_RESOURCE_NAME,
	.add_fn = msm_pm_qos_add,
	.update_fn = msm_pm_qos_update,
	.remove_fn = msm_pm_qos_remove
};

DECLARE_RESOURCE_REMOTE_AGGREGATION(
	npa_system_bus_node,
	npa_system_bus_resource,
	SYSTEM_BUS_NPA_RESOURCE_NAME,
	"flow", UINT_MAX);

static int __init msm_pm_qos_plugin_init(void)
{
	return pm_qos_register_plugin(PM_QOS_SYSTEM_BUS_FREQ,
					&system_bus_plugin);
}
core_initcall(msm_pm_qos_plugin_init);
#endif /* CONFIG_MSM_NPA_SYSTEM_BUS */

static int __init npa_resource_init(void)
{
#ifdef CONFIG_MSM_NPA_SYSTEM_BUS
	npa_remote_define_node(&npa_system_bus_node, 0, NULL, NULL);
#endif
	npa_remote_define_node(&npa_memory_node, 0, NULL, NULL);

	return 0;
}
arch_initcall(npa_resource_init);

#endif /* CONFIG_MSM_NPA_REMOTE */
