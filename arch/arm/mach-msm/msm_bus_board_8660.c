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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/module.h>
#include <mach/msm_bus.h>
#include <mach/msm_bus_board.h>
#include <mach/board.h>
#include "msm_bus_core.h"
#include "rpm.h"

#define NMASTERS 39
#define NSLAVES 67

enum msm_bus_fabric_tiered_slave_type {
	MSM_BUS_SYSTEM_TIERED_SLAVE_FAB_APPSS = 1,
	MSM_BUS_TIERED_SLAVE_SYSTEM_IMEM,

	MSM_BUS_TIERED_SLAVE_SMI = 1,
	MSM_BUS_MMSS_TIERED_SLAVE_FAB_APPS,
	MSM_BUS_TIERED_SLAVE_MM_IMEM,

	MSM_BUS_TIERED_SLAVE_EBI_CH0 = 1,
	MSM_BUS_TIERED_SLAVE_SMPSS_L2,
};

enum msm_bus_8660_master_ports_type {
	MSM_BUS_SYSTEM_MASTER_PORT_APPSS_FAB = 0,
	MSM_BUS_MASTER_PORT_SPS,
	MSM_BUS_MASTER_PORT_ADM0_PORT0,
	MSM_BUS_MASTER_PORT_ADM0_PORT1,
	MSM_BUS_MASTER_PORT_ADM1_PORT0,
	MSM_BUS_MASTER_PORT_ADM1_PORT1,
	MSM_BUS_MASTER_PORT_LPASS_PROC,
	MSM_BUS_MASTER_PORT_MSS_PROCI,
	MSM_BUS_MASTER_PORT_MSM_MSS_PROCD,
	MSM_BUS_MASTER_PORT_MSM_MDM_PORT0,
	MSM_BUS_MASTER_PORT_LPASS,
	MSM_BUS_SYSTEM_MASTER_PORT_CPSS_FPB,
	MSM_BUS_MASTER_PORT_SYSTEM_FPB,
	MSM_BUS_MASTER_PORT_MMSS_FPB,
	MSM_BUS_MASTER_PORT_ADM1_AHB_CI,
	MSM_BUS_MASTER_PORT_ADM0_AHB_CI,
	MSM_BUS_MASTER_PORT_MSS_MDM_PORT1,

	MSM_BUS_MASTER_PORT_MDP_PORT0 = 0,
	MSM_BUS_MASTER_PORT_MDP_PORT1,
	MSM_BUS_MMSS_MASTER_PORT_ADM1_PORT0,
	MSM_BUS_MASTER_PORT_ROTATOR,
	MSM_BUS_MASTER_PORT_GRAPHICS_3D,
	MSM_BUS_MASTER_PORT_JPEG_DEC,
	MSM_BUS_MASTER_PORT_GRAPHICS_2D_CORE0,
	MSM_BUS_MASTER_PORT_VFE,
	MSM_BUS_MASTER_PORT_VPE,
	MSM_BUS_MASTER_PORT_JPEG_ENC,
	MSM_BUS_MASTER_PORT_GRAPHICS_2D_CORE1,
	MSM_BUS_MMSS_MASTER_PORT_APPS_FAB,
	MSM_BUS_MASTER_PORT_HD_CODEC_PORT0,
	MSM_BUS_MASTER_PORT_HD_CODEC_PORT1,

	MSM_BUS_MASTER_PORT_SMPSS_M0 = 0,
	MSM_BUS_MASTER_PORT_SMPSS_M1,
	MSM_BUS_APPSS_MASTER_PORT_FAB_MMSS,
	MSM_BUS_APPSS_MASTER_PORT_FAB_SYSTEM,

};

enum msm_bus_8660_slave_ports_type {
	MSM_BUS_SLAVE_PORT_SMI = 0,
	MSM_BUS_MMSS_SLAVE_PORT_APPS_FAB_0,
	MSM_BUS_MMSS_SLAVE_PORT_APPS_FAB_1,
	MSM_BUS_SLAVE_PORT_MM_IMEM,

	MSM_BUS_SLAVE_PORT_EBI_CH0 = 0,
	MSM_BUS_SLAVE_PORT_SMPSS_L2,
	MSM_BUS_APPSS_SLAVE_PORT_MMSS_FAB,
	MSM_BUS_SLAVE_PORT_SYSTEM_FAB,

	MSM_BUS_SYSTEM_SLAVE_PORT_APPSS_FAB = 0,
	MSM_BUS_SLAVE_PORT_SPS,
	MSM_BUS_SLAVE_PORT_SYSTEM_IMEM,
	MSM_BUS_SLAVE_PORT_SMPSS,
	MSM_BUS_SLAVE_PORT_MSS,
	MSM_BUS_SLAVE_PORT_LPASS,
	MSM_BUS_SYSTEM_SLAVE_PORT_CPSS_FPB,
	MSM_BUS_SYSTEM_SLAVE_PORT_SYSTEM_FPB,
	MSM_BUS_SLAVE_PORT_MMSS_FPB,
};

static uint32_t master_iids[NMASTERS];
static uint32_t slave_iids[NSLAVES];

static struct msm_bus_node_info apps_fabric_info[] = {
	{
		.id = MSM_BUS_MASTER_AMPSS_M0,
		.masterp = MSM_BUS_MASTER_PORT_SMPSS_M1,
		.tier = MSM_BUS_BW_TIER2,
	},
	{
		.id = MSM_BUS_MASTER_AMPSS_M1,
		.masterp = MSM_BUS_MASTER_PORT_SMPSS_M1,
		.tier = MSM_BUS_BW_TIER2,
	},
	{
		.id = MSM_BUS_SLAVE_EBI_CH0,
		.slavep = MSM_BUS_SLAVE_PORT_EBI_CH0,
		.tier = MSM_BUS_TIERED_SLAVE_EBI_CH0,
		.buswidth = 8,
		.slaveclk = "ebi1_msmbus_clk",
		.a_slaveclk = "ebi1_a_clk",
	},
	{
		.id = MSM_BUS_SLAVE_AMPSS_L2,
		.slavep = MSM_BUS_SLAVE_PORT_SMPSS_L2,
		.tier = MSM_BUS_TIERED_SLAVE_SMPSS_L2,
		.buswidth = 8,
	},
	{
		.id = MSM_BUS_FAB_MMSS,
		.gateway = 1,
		.slavep = MSM_BUS_APPSS_SLAVE_PORT_MMSS_FAB,
		.masterp = MSM_BUS_MMSS_MASTER_PORT_APPS_FAB,
		.buswidth = 8,
	},
	{
		.id = MSM_BUS_FAB_SYSTEM,
		.gateway = 1,
		.slavep = MSM_BUS_SLAVE_PORT_SYSTEM_FAB,
		.masterp = MSM_BUS_SYSTEM_MASTER_PORT_APPSS_FAB,
		.buswidth = 8,
	},
};

static struct msm_bus_node_info system_fabric_info[]  = {
	{
		.id = MSM_BUS_MASTER_SPS,
		.masterp = MSM_BUS_MASTER_PORT_SPS,
		.tier = MSM_BUS_BW_TIER2,
	},
	{
		.id = MSM_BUS_MASTER_ADM0_PORT0,
		.masterp = MSM_BUS_MASTER_PORT_ADM0_PORT0,
		.tier = MSM_BUS_BW_TIER2,
	},
	{
		.id = MSM_BUS_MASTER_ADM0_PORT1,
		.masterp = MSM_BUS_MASTER_PORT_ADM0_PORT1,
		.tier = MSM_BUS_BW_TIER2,
	},
	{
		.id = MSM_BUS_SYSTEM_MASTER_ADM1_PORT0,
		.masterp = MSM_BUS_MASTER_PORT_ADM1_PORT0,
		.tier = MSM_BUS_BW_TIER2,
	},
	{
		.id = MSM_BUS_MASTER_ADM1_PORT1,
		.masterp = MSM_BUS_MASTER_PORT_ADM1_PORT1,
		.tier = MSM_BUS_BW_TIER2,
	},
	{
		.id = MSM_BUS_MASTER_LPASS_PROC,
		.masterp = MSM_BUS_MASTER_PORT_LPASS_PROC,
		.tier = MSM_BUS_BW_TIER2,
	},
	{
		.id = MSM_BUS_MASTER_MSS_PROCI,
		.masterp = MSM_BUS_MASTER_PORT_MSS_PROCI,
		.tier = MSM_BUS_BW_TIER2,
	},
	{
		.id = MSM_BUS_MASTER_MSS_PROCD,
		.masterp = MSM_BUS_MASTER_PORT_MSM_MSS_PROCD,
		.tier = MSM_BUS_BW_TIER2,
	},
	{
		.id = MSM_BUS_MASTER_MSS_MDM_PORT0,
		.masterp = MSM_BUS_MASTER_PORT_MSM_MDM_PORT0,
		.tier = MSM_BUS_BW_TIER2,
	},
	{
		.id = MSM_BUS_MASTER_LPASS,
		.masterp = MSM_BUS_MASTER_PORT_LPASS,
		.tier = MSM_BUS_BW_TIER2,
	},
	{
		.id = MSM_BUS_SYSTEM_MASTER_MMSS_FPB,
		.masterp = MSM_BUS_MASTER_PORT_MMSS_FPB,
		.tier = MSM_BUS_BW_TIER2,
	},
	{
		.id = MSM_BUS_MASTER_ADM1_CI,
		.masterp = MSM_BUS_MASTER_PORT_ADM1_AHB_CI,
		.tier = MSM_BUS_BW_TIER2,
	},
	{
		.id = MSM_BUS_MASTER_ADM0_CI,
		.masterp = MSM_BUS_MASTER_PORT_ADM0_AHB_CI,
		.tier = MSM_BUS_BW_TIER2,
	},
	{
		.id = MSM_BUS_MASTER_MSS_MDM_PORT1,
		.masterp = MSM_BUS_MASTER_PORT_MSS_MDM_PORT1,
		.tier = MSM_BUS_BW_TIER2,
	},
	{
		.id = MSM_BUS_FAB_APPSS,
		.gateway = 1,
		.slavep = MSM_BUS_SYSTEM_SLAVE_PORT_APPSS_FAB,
		.masterp = MSM_BUS_APPSS_MASTER_PORT_FAB_SYSTEM,
		.tier = MSM_BUS_SYSTEM_TIERED_SLAVE_FAB_APPSS,
		.buswidth = 8,
	},
	{
		.id = MSM_BUS_FAB_SYSTEM_FPB,
		.gateway = 1,
		.slavep = MSM_BUS_SYSTEM_SLAVE_PORT_SYSTEM_FPB,
		.masterp = MSM_BUS_MASTER_PORT_SYSTEM_FPB,
		.buswidth = 4,
	},
	{
		.id = MSM_BUS_FAB_CPSS_FPB,
		.gateway = 1,
		.slavep = MSM_BUS_SYSTEM_SLAVE_PORT_CPSS_FPB,
		.masterp = MSM_BUS_SYSTEM_MASTER_PORT_CPSS_FPB,
		.buswidth = 4,
	},
	{
		.id = MSM_BUS_SLAVE_SPS,
		.slavep = MSM_BUS_SLAVE_PORT_SPS,
		.buswidth = 8,
	},
	{
		.id = MSM_BUS_SLAVE_SYSTEM_IMEM,
		.slavep = MSM_BUS_SLAVE_PORT_SYSTEM_IMEM,
		.tier = MSM_BUS_TIERED_SLAVE_SYSTEM_IMEM,
		.buswidth = 8,
	},
	{
		.id = MSM_BUS_SLAVE_AMPSS,
		.slavep = MSM_BUS_SLAVE_PORT_SMPSS,
		.buswidth = 8,
	},
	{
		.id = MSM_BUS_SLAVE_MSS,
		.slavep = MSM_BUS_SLAVE_PORT_MSS,
		.buswidth = 8,
	},
	{
		.id = MSM_BUS_SLAVE_LPASS,
		.slavep = MSM_BUS_SLAVE_PORT_LPASS,
		.buswidth = 8,
	},
	{
		.id = MSM_BUS_SYSTEM_SLAVE_MMSS_FPB,
		.slavep = MSM_BUS_SLAVE_PORT_MMSS_FPB,
		.buswidth = 8,
	},
};

static struct msm_bus_node_info mmss_fabric_info[]  = {
	{
		.id = MSM_BUS_MASTER_MDP_PORT0,
		.masterp = MSM_BUS_MASTER_PORT_MDP_PORT0,
		.tier = MSM_BUS_BW_TIER1,
	},
	{
		.id = MSM_BUS_MASTER_MDP_PORT1,
		.masterp = MSM_BUS_MASTER_PORT_MDP_PORT1,
		.tier = MSM_BUS_BW_TIER1,
	},
	{
		.id = MSM_BUS_MMSS_MASTER_ADM1_PORT0,
		.masterp = MSM_BUS_MMSS_MASTER_PORT_ADM1_PORT0,
		.tier = MSM_BUS_BW_TIER2,
	},
	{
		.id = MSM_BUS_MASTER_ROTATOR,
		.masterp = MSM_BUS_MASTER_PORT_ROTATOR,
		.tier = MSM_BUS_BW_TIER2,
	},
	{
		.id = MSM_BUS_MASTER_GRAPHICS_3D,
		.masterp = MSM_BUS_MASTER_PORT_GRAPHICS_3D,
		.tier = MSM_BUS_BW_TIER2,
	},
	{
		.id = MSM_BUS_MASTER_JPEG_DEC,
		.masterp = MSM_BUS_MASTER_PORT_JPEG_DEC,
		.tier = MSM_BUS_BW_TIER2,
	},
	{
		.id = MSM_BUS_MASTER_GRAPHICS_2D_CORE0,
		.masterp = MSM_BUS_MASTER_PORT_GRAPHICS_2D_CORE0,
		.tier = MSM_BUS_BW_TIER2,
	},
	{
		.id = MSM_BUS_MASTER_VFE,
		.masterp = MSM_BUS_MASTER_PORT_VFE,
		.tier = MSM_BUS_BW_TIER2,
	},
	{
		.id = MSM_BUS_MASTER_VPE,
		.masterp = MSM_BUS_MASTER_PORT_VPE,
		.tier = MSM_BUS_BW_TIER2,
	},
	{
		.id = MSM_BUS_MASTER_JPEG_ENC,
		.masterp = MSM_BUS_MASTER_PORT_JPEG_ENC,
		.tier = MSM_BUS_BW_TIER2,
	},
	/* This port has been added for V2. It is absent in V1 */
	{
		.id = MSM_BUS_MASTER_GRAPHICS_2D_CORE1,
		.masterp = MSM_BUS_MASTER_PORT_GRAPHICS_2D_CORE1,
		.tier = MSM_BUS_BW_TIER2,
	},
	{
		.id = MSM_BUS_MASTER_HD_CODEC_PORT0,
		.masterp = MSM_BUS_MASTER_PORT_HD_CODEC_PORT0,
		.tier = MSM_BUS_BW_TIER2,
	},
	{
		.id = MSM_BUS_MASTER_HD_CODEC_PORT1,
		.masterp = MSM_BUS_MASTER_PORT_HD_CODEC_PORT1,
		.tier = MSM_BUS_BW_TIER2,
	},
	{
		.id = MSM_BUS_SLAVE_SMI,
		.slavep = MSM_BUS_SLAVE_PORT_SMI,
		.tier = MSM_BUS_TIERED_SLAVE_SMI,
		.buswidth = 16,
		.slaveclk = "smi_clk",
		.a_slaveclk = "smi_a_clk",
	},
	{
		.id = MSM_BUS_MMSS_SLAVE_FAB_APPS_1,
		.slavep = MSM_BUS_MMSS_SLAVE_PORT_APPS_FAB_0,
		.buswidth = 8,
	},
	{
		.id = MSM_BUS_FAB_APPSS,
		.gateway = 1,
		.slavep = MSM_BUS_MMSS_SLAVE_PORT_APPS_FAB_0,
		.masterp = MSM_BUS_APPSS_MASTER_PORT_FAB_MMSS,
		.tier = MSM_BUS_MMSS_TIERED_SLAVE_FAB_APPS,
		.buswidth = 8,
	},
	{
		.id = MSM_BUS_SLAVE_MM_IMEM,
		.slavep = MSM_BUS_SLAVE_PORT_MM_IMEM,
		.tier = MSM_BUS_TIERED_SLAVE_MM_IMEM,
		.buswidth = 8,
	},
};

static struct msm_bus_node_info sys_fpb_fabric_info[]  = {
	{
		.id = MSM_BUS_FAB_SYSTEM,
		.gateway = 1,
		.slavep = MSM_BUS_SYSTEM_SLAVE_PORT_SYSTEM_FPB,
		.masterp = MSM_BUS_MASTER_PORT_SYSTEM_FPB,
		.buswidth = 4,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_MASTER_SPDM,
		.ahb = 1,
		.tier = MSM_BUS_BW_TIER2,
	},
	{
		.id = MSM_BUS_MASTER_RPM,
		.ahb = 1,
		.tier = MSM_BUS_BW_TIER2,
	},
	{
		.id = MSM_BUS_SLAVE_SPDM,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_RPM,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_RPM_MSG_RAM,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_MPM,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_PMIC1_SSBI1_A,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_PMIC1_SSBI1_B,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_PMIC1_SSBI1_C,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_PMIC2_SSBI2_A,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_PMIC2_SSBI2_B,
		.buswidth = 4,
		.ahb = 1,
	},
};

static struct msm_bus_node_info cpss_fpb_fabric_info[] = {
	{
		.id = MSM_BUS_FAB_SYSTEM,
		.gateway = 1,
		.slavep = MSM_BUS_SYSTEM_SLAVE_PORT_CPSS_FPB,
		.masterp = MSM_BUS_SYSTEM_MASTER_PORT_CPSS_FPB,
		.buswidth = 4,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_GSBI1_UART,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_GSBI2_UART,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_GSBI3_UART,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_GSBI4_UART,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_GSBI5_UART,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_GSBI6_UART,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_GSBI7_UART,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_GSBI8_UART,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_GSBI9_UART,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_GSBI10_UART,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_GSBI11_UART,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_GSBI12_UART,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_GSBI1_QUP,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_GSBI2_QUP,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_GSBI3_QUP,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_GSBI4_QUP,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_GSBI5_QUP,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_GSBI6_QUP,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_GSBI7_QUP,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_GSBI8_QUP,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_GSBI9_QUP,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_GSBI10_QUP,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_GSBI11_QUP,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_GSBI12_QUP,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_EBI2_NAND,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_EBI2_CS0,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_EBI2_CS1,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_EBI2_CS2,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_EBI2_CS3,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_EBI2_CS4,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_EBI2_CS5,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_USB_FS1,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_USB_FS2,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_TSIF,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_MSM_TSSC,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_MSM_PDM,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_MSM_DIMEM,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_MSM_TCSR,
		.buswidth = 8,
		.ahb = 1,
	},
	{
		.id = MSM_BUS_SLAVE_MSM_PRNG,
		.buswidth = 4,
		.ahb = 1,
	},
};

struct msm_bus_fabric_registration msm_bus_apps_fabric_pdata = {
	.id = MSM_BUS_FAB_APPSS,
	.name = "msm_apps_fab",
	.info = apps_fabric_info,
	.len = ARRAY_SIZE(apps_fabric_info),
	.ahb = 0,
	.fabclk = "afab_clk",
	.a_fabclk = "afab_a_clk",
	.haltid = MSM_RPM_ID_APPS_FABRIC_HALT_0,
	.offset = MSM_RPM_ID_APPS_FABRIC_ARB_0,
	.nmasters = 4,
	.nslaves = 4,
	.ntieredslaves = 2,
};

struct msm_bus_fabric_registration msm_bus_sys_fabric_pdata = {
	.id = MSM_BUS_FAB_SYSTEM,
	.name = "msm_sys_fab",
	system_fabric_info,
	ARRAY_SIZE(system_fabric_info),
	.ahb = 0,
	.fabclk = "sfab_clk",
	.a_fabclk = "sfab_a_clk",
	.haltid = MSM_RPM_ID_SYSTEM_FABRIC_HALT_0,
	.offset = MSM_RPM_ID_SYSTEM_FABRIC_ARB_0,
	.nmasters = 17,
	.nslaves = 9,
	.ntieredslaves = 2,
};

struct msm_bus_fabric_registration msm_bus_mm_fabric_pdata = {
	.id = MSM_BUS_FAB_MMSS,
	.name = "msm_mm_fab",
	mmss_fabric_info,
	ARRAY_SIZE(mmss_fabric_info),
	.ahb = 0,
	.fabclk = "mmfab_clk",
	.a_fabclk = "mmfab_a_clk",
	.haltid = MSM_RPM_ID_MM_FABRIC_HALT_0,
	.offset = MSM_RPM_ID_MM_FABRIC_ARB_0,
	.nmasters = 14,
	.nslaves = 4,
	.ntieredslaves = 3,
};

struct msm_bus_fabric_registration msm_bus_sys_fpb_pdata = {
	.id = MSM_BUS_FAB_SYSTEM_FPB,
	.name = "msm_sys_fpb",
	sys_fpb_fabric_info,
	ARRAY_SIZE(sys_fpb_fabric_info),
	.ahb = 1,
	.fabclk = "sfpb_clk",
	.fabclk = "sfpb_a_clk",
	.nmasters = 0,
	.nslaves = 0,
	.ntieredslaves = 0,
};

struct msm_bus_fabric_registration msm_bus_cpss_fpb_pdata = {
	.id = MSM_BUS_FAB_CPSS_FPB,
	.name = "msm_cpss_fpb",
	cpss_fpb_fabric_info,
	ARRAY_SIZE(cpss_fpb_fabric_info),
	.ahb = 1,
	.fabclk = "cfpb_clk",
	.a_fabclk = "cfpb_a_clk",
	.nmasters = 0,
	.nslaves = 0,
	.ntieredslaves = 0,
};

static void msm_bus_board_get_ids(
	struct msm_bus_fabric_registration *fabreg,
	int fabid)
{
	int i;
	for (i = 0; i < fabreg->len; i++) {
		if (!fabreg->info[i].gateway) {
			fabreg->info[i].priv_id = fabid + fabreg->info[i].id;
			if (fabreg->info[i].id < SLAVE_ID_KEY)
				master_iids[fabreg->info[i].id] =
					fabreg->info[i].priv_id;
			else
				slave_iids[fabreg->info[i].id - (SLAVE_ID_KEY)]
					= fabreg->info[i].priv_id;
		} else
			fabreg->info[i].priv_id = fabreg->info[i].id;
	}
}

void msm_bus_board_assign_iids(struct msm_bus_fabric_registration *fabreg,
	int fabid)
{
	msm_bus_board_get_ids(fabreg, fabid);
}
int msm_bus_board_get_iid(int id)
{
	return ((id < SLAVE_ID_KEY) ? master_iids[id] : slave_iids[id -
		SLAVE_ID_KEY]);
}

