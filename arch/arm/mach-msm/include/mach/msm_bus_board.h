/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, and the entire permission notice in its entirety,
 *    including the disclaimer of warranties.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * ALTERNATIVELY, this product may be distributed under the terms of
 * the GNU General Public License, version 2, in which case the provisions
 * of the GPL version 2 are required INSTEAD OF the BSD license.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ALL OF
 * WHICH ARE HEREBY DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF NOT ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#ifndef __ASM_ARCH_MSM_BUS_BOARD_H
#define __ASM_ARCH_MSM_BUS_BOARD_H

#include <linux/types.h>
#include <linux/input.h>

struct msm_bus_fabric_registration {
	unsigned int id;
	char *name;
	struct msm_bus_node_info *info;
	unsigned int len;
	int ahb;
	const char *fabclk;
	const char *a_fabclk;
	unsigned int offset;
	unsigned int haltid;
	unsigned int rpm_enabled;
	unsigned int nmasters;
	unsigned int nslaves;
	unsigned int ntieredslaves;
};

enum msm_bus_bw_tier_type {
	MSM_BUS_BW_TIER1 = 1,
	MSM_BUS_BW_TIER2,
	MSM_BUS_BW_COUNT,
	MSM_BUS_BW_SIZE = 0x7FFFFFFF,
};

struct msm_bus_halt_vector {
	uint32_t haltval;
	uint32_t haltmask;
};

extern struct msm_bus_fabric_registration msm_bus_apps_fabric_pdata;
extern struct msm_bus_fabric_registration msm_bus_sys_fabric_pdata;
extern struct msm_bus_fabric_registration msm_bus_mm_fabric_pdata;
extern struct msm_bus_fabric_registration msm_bus_sys_fpb_pdata;
extern struct msm_bus_fabric_registration msm_bus_cpss_fpb_pdata;

void msm_bus_board_assign_iids(struct msm_bus_fabric_registration
	*fabreg, int fabid);
int msm_bus_board_get_iid(int id);

/*
 * These macros specify the convention followed for allocating
 * ids to fabrics, masters and slaves for 8x60.
 *
 * A node can be identified as a master/slave/fabric by using
 * these ids.
 */
#define FABRIC_ID_KEY 1024
#define SLAVE_ID_KEY ((FABRIC_ID_KEY) >> 1)
#define NUM_FAB 5
#define MAX_FAB_KEY 7168  /* OR(All fabric ids) */

#define GET_FABID(id) ((id) & MAX_FAB_KEY)

#define NODE_ID(id) ((id) & (FABRIC_ID_KEY - 1))
#define IS_SLAVE(id) ((NODE_ID(id)) >= SLAVE_ID_KEY ? 1 : 0)

/*
 * The following macros are used for various operations on commit data.
 * Commit data is an array of 32 bit integers. The size of arrays is unique
 * to the fabric. Commit arrays are allocated at run-time based on the number
 * of masters, slaves and tiered-slaves registered.
 */

#define CREATE_BW_TIER_PAIR(type, bw) \
	((((type) == MSM_BUS_BW_TIER1 ? 1 : 0) << 15) | ((bw) & 0x7FFF))

#define MSM_BUS_GET_BW(val) ((val) & 0x7FFF)

#define MSM_BUS_GET_BW_INFO(val, type, bw) \
  do {	\
	(type) = MSM_BUS_GET_BW_TYPE(val); \
	(bw) = MSM_BUS_GET_BW(val);	\
	} while (0)

#define ROUNDED_BW_VAL_FROM_BYTES(bw) \
	((((bw) >> 17) + 1) & 0x8000 ? 0x7FFF : (((bw) >> 17) + 1))

#define BW_VAL_FROM_BYTES(bw) \
	((((bw) >> 17) & 0x8000) ? 0x7FFF : ((bw) >> 17))

#define MSM_BUS_BW_VAL_FROM_BYTES(bw) \
	((((bw) & 0x1FFFF) && (((bw) >> 17) == 0)) ? \
	 ROUNDED_BW_VAL_FROM_BYTES(bw) : BW_VAL_FROM_BYTES(bw))

#define MSM_BUS_CREATE_BW_TIER_PAIR_BYTES(type, bw) \
	((((type) == MSM_BUS_BW_TIER1 ? 1 : 0) << 15) | \
	 (MSM_BUS_BW_VAL_FROM_BYTES(bw)))

#define MSM_BUS_GET_BW_BYTES(val) \
	(((val) & 0x7FFF) << 17)

#define MSM_BUS_GET_BW_INFO_BYTES (val, type, bw) \
  do {	\
	(type) = MSM_BUS_GET_BW_TYPE(val); \
	(bw) = MSM_BUS_GET_BW_BYTES(val); \
	} while (0)

#define FAB_MAX_BW_BYTES(width, clk) ((uint32_t)(width) * (uint32_t)(clk))
#define FAB_BW_128K(bw) ((uint16_t)((bw) >> 17))
#define BW_TO_CLK_FREQ_HZ(width, bw) ((unsigned long)((bw) / (width)))
/* 8 bytes per clock @ 133 MHz */
#define SYSFAB_MAX_BW_BYTES FAB_MAX_BW_BYTES(8, 133000000)
/* 16 bytes per clock @ 166 MHz */
#define MMFAB_MAX_BW_BYTES FAB_MAX_BW_BYTES(16, 166000000)
/* 8 bytes per clock @ 266 MHz */
#define APPSFAB_MAX_BW_BYTES FAB_MAX_BW_BYTES(8, 266000000)

/*
 * The following macros are used to format the data for port halt
 * and unhalt requests.
 */
#define MSM_BUS_CLK_HALT 0x1
#define MSM_BUS_CLK_HALT_MASK 0x1
#define MSM_BUS_CLK_HALT_FIELDSIZE 0x1
#define MSM_BUS_CLK_UNHALT 0x0

#define MSM_BUS_MASTER_SHIFT(master, fieldsize) \
	((master) * (fieldsize))

#define MSM_BUS_SET_BITFIELD(word, fieldmask, fieldvalue) \
	{	\
		(word) &= ~(fieldmask);	\
		(word) |= (fieldvalue);	\
	}


#define MSM_BUS_MASTER_HALT(u32haltmask, u32haltval, master) \
	MSM_BUS_SET_BITFIELD(u32haltmask, \
		MSM_BUS_CLK_HALT_MASK<<MSM_BUS_MASTER_SHIFT((master),\
		MSM_BUS_CLK_HALT_FIELDSIZE), \
		MSM_BUS_CLK_HALT_MASK<<MSM_BUS_MASTER_SHIFT((master),\
		MSM_BUS_CLK_HALT_FIELDSIZE))\
	MSM_BUS_SET_BITFIELD(u32haltval, \
		MSM_BUS_CLK_HALT_MASK<<MSM_BUS_MASTER_SHIFT((master),\
		MSM_BUS_CLK_HALT_FIELDSIZE), \
		MSM_BUS_CLK_HALT<<MSM_BUS_MASTER_SHIFT((master),\
		MSM_BUS_CLK_HALT_FIELDSIZE))\

#define MSM_BUS_MASTER_UNHALT(u32haltmask, u32haltval, master) \
	MSM_BUS_SET_BITFIELD(u32haltmask, \
		MSM_BUS_CLK_HALT_MASK<<MSM_BUS_MASTER_SHIFT((master),\
		MSM_BUS_CLK_HALT_FIELDSIZE), \
		MSM_BUS_CLK_HALT_MASK<<MSM_BUS_MASTER_SHIFT((master),\
		MSM_BUS_CLK_HALT_FIELDSIZE))\
	MSM_BUS_SET_BITFIELD(u32haltval, \
		MSM_BUS_CLK_HALT_MASK<<MSM_BUS_MASTER_SHIFT((master),\
		MSM_BUS_CLK_HALT_FIELDSIZE), \
		MSM_BUS_CLK_UNHALT<<MSM_BUS_MASTER_SHIFT((master),\
		MSM_BUS_CLK_HALT_FIELDSIZE))\

/* Topology related enums */
enum msm_bus_fabric_type {
	MSM_BUS_FAB_APPSS = 0,
	MSM_BUS_FAB_SYSTEM = 1024,
	MSM_BUS_FAB_MMSS = 2048,
	MSM_BUS_FAB_SYSTEM_FPB = 3072,
	MSM_BUS_FAB_CPSS_FPB = 4096,
};

enum msm_bus_fabric_master_type {
	MSM_BUS_MASTER_AMPSS_M0 = 1,
	MSM_BUS_MASTER_AMPSS_M1,
	MSM_BUS_APPSS_MASTER_FAB_MMSS,
	MSM_BUS_APPSS_MASTER_FAB_SYSTEM,

	MSM_BUS_SYSTEM_MASTER_FAB_APPSS,
	MSM_BUS_MASTER_SPS,
	MSM_BUS_MASTER_ADM0_PORT0,
	MSM_BUS_MASTER_ADM0_PORT1,
	MSM_BUS_SYSTEM_MASTER_ADM1_PORT0,
	MSM_BUS_MASTER_ADM1_PORT1,
	MSM_BUS_MASTER_LPASS_PROC,
	MSM_BUS_MASTER_MSS_PROCI,
	MSM_BUS_MASTER_MSS_PROCD,
	MSM_BUS_MASTER_MSS_MDM_PORT0,
	MSM_BUS_MASTER_LPASS,
	MSM_BUS_SYSTEM_MASTER_CPSS_FPB,
	MSM_BUS_SYSTEM_MASTER_SYSTEM_FPB,
	MSM_BUS_SYSTEM_MASTER_MMSS_FPB,
	MSM_BUS_MASTER_ADM1_CI,
	MSM_BUS_MASTER_ADM0_CI,
	MSM_BUS_MASTER_MSS_MDM_PORT1,

	MSM_BUS_MASTER_MDP_PORT0,
	MSM_BUS_MASTER_MDP_PORT1,
	MSM_BUS_MMSS_MASTER_ADM1_PORT0,
	MSM_BUS_MASTER_ROTATOR,
	MSM_BUS_MASTER_GRAPHICS_3D,
	MSM_BUS_MASTER_JPEG_DEC,
	MSM_BUS_MASTER_GRAPHICS_2D_CORE0,
	MSM_BUS_MASTER_VFE,
	MSM_BUS_MASTER_VPE,
	MSM_BUS_MASTER_JPEG_ENC,
	MSM_BUS_MASTER_GRAPHICS_2D_CORE1,
	MSM_BUS_MMSS_MASTER_APPS_FAB,
	MSM_BUS_MASTER_HD_CODEC_PORT0,
	MSM_BUS_MASTER_HD_CODEC_PORT1,

	MSM_BUS_MASTER_SPDM,
	MSM_BUS_MASTER_RPM,

	MSM_BUS_SYSTEM_FPB_MASTER_SYSTEM =
		MSM_BUS_SYSTEM_MASTER_SYSTEM_FPB,
	MSM_BUS_CPSS_FPB_MASTER_SYSTEM =
		MSM_BUS_SYSTEM_MASTER_CPSS_FPB,
};

enum msm_bus_fabric_slave_type {
	MSM_BUS_SLAVE_EBI_CH0 = SLAVE_ID_KEY,
	MSM_BUS_SLAVE_AMPSS_L2,
	MSM_BUS_APPSS_SLAVE_FAB_MMSS,
	MSM_BUS_APPSS_SLAVE_FAB_SYSTEM,

	MSM_BUS_SYSTEM_SLAVE_FAB_APPS,
	MSM_BUS_SLAVE_SPS,
	MSM_BUS_SLAVE_SYSTEM_IMEM,
	MSM_BUS_SLAVE_AMPSS,
	MSM_BUS_SLAVE_MSS,
	MSM_BUS_SLAVE_LPASS,
	MSM_BUS_SYSTEM_SLAVE_CPSS_FPB,
	MSM_BUS_SYSTEM_SLAVE_SYSTEM_FPB,
	MSM_BUS_SYSTEM_SLAVE_MMSS_FPB,

	MSM_BUS_SLAVE_SMI,
	MSM_BUS_MMSS_SLAVE_FAB_APPS,
	/*
	 * APPS_1: This port is added for V2, and is absent on V1.
	 * This port is not connected on V2, but is needed in
	 * the topology.
	 * */
	MSM_BUS_MMSS_SLAVE_FAB_APPS_1,
	MSM_BUS_SLAVE_MM_IMEM,

	MSM_BUS_SLAVE_SPDM,
	MSM_BUS_SLAVE_RPM,
	MSM_BUS_SLAVE_RPM_MSG_RAM,
	MSM_BUS_SLAVE_MPM,
	MSM_BUS_SLAVE_PMIC1_SSBI1_A,
	MSM_BUS_SLAVE_PMIC1_SSBI1_B,
	MSM_BUS_SLAVE_PMIC1_SSBI1_C,
	MSM_BUS_SLAVE_PMIC2_SSBI2_A,
	MSM_BUS_SLAVE_PMIC2_SSBI2_B,

	MSM_BUS_SLAVE_GSBI1_UART,
	MSM_BUS_SLAVE_GSBI2_UART,
	MSM_BUS_SLAVE_GSBI3_UART,
	MSM_BUS_SLAVE_GSBI4_UART,
	MSM_BUS_SLAVE_GSBI5_UART,
	MSM_BUS_SLAVE_GSBI6_UART,
	MSM_BUS_SLAVE_GSBI7_UART,
	MSM_BUS_SLAVE_GSBI8_UART,
	MSM_BUS_SLAVE_GSBI9_UART,
	MSM_BUS_SLAVE_GSBI10_UART,
	MSM_BUS_SLAVE_GSBI11_UART,
	MSM_BUS_SLAVE_GSBI12_UART,
	MSM_BUS_SLAVE_GSBI1_QUP,
	MSM_BUS_SLAVE_GSBI2_QUP,
	MSM_BUS_SLAVE_GSBI3_QUP,
	MSM_BUS_SLAVE_GSBI4_QUP,
	MSM_BUS_SLAVE_GSBI5_QUP,
	MSM_BUS_SLAVE_GSBI6_QUP,
	MSM_BUS_SLAVE_GSBI7_QUP,
	MSM_BUS_SLAVE_GSBI8_QUP,
	MSM_BUS_SLAVE_GSBI9_QUP,
	MSM_BUS_SLAVE_GSBI10_QUP,
	MSM_BUS_SLAVE_GSBI11_QUP,
	MSM_BUS_SLAVE_GSBI12_QUP,
	MSM_BUS_SLAVE_EBI2_NAND,
	MSM_BUS_SLAVE_EBI2_CS0,
	MSM_BUS_SLAVE_EBI2_CS1,
	MSM_BUS_SLAVE_EBI2_CS2,
	MSM_BUS_SLAVE_EBI2_CS3,
	MSM_BUS_SLAVE_EBI2_CS4,
	MSM_BUS_SLAVE_EBI2_CS5,
	MSM_BUS_SLAVE_USB_FS1,
	MSM_BUS_SLAVE_USB_FS2,
	MSM_BUS_SLAVE_TSIF,
	MSM_BUS_SLAVE_MSM_TSSC,
	MSM_BUS_SLAVE_MSM_PDM,
	MSM_BUS_SLAVE_MSM_DIMEM,
	MSM_BUS_SLAVE_MSM_TCSR,
	MSM_BUS_SLAVE_MSM_PRNG,
	MSM_BUS_SYSTEM_FPB_SLAVE_SYSTEM =
		MSM_BUS_SYSTEM_SLAVE_SYSTEM_FPB,
	MSM_BUS_CPSS_FPB_SLAVE_SYSTEM =
		MSM_BUS_SYSTEM_SLAVE_CPSS_FPB,
};

#endif /*__ASM_ARCH_MSM_BUS_BOARD_H */
