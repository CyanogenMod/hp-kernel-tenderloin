/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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

#ifndef _ARCH_ARM_MACH_MSM_BUS_CORE_H
#define _ARCH_ARM_MACH_MSM_BUS_CORE_H

#include <linux/types.h>
#include <linux/device.h>
#include <linux/radix-tree.h>
#include <linux/platform_device.h>
#include <mach/msm_bus_board.h>
#include <mach/msm_bus.h>

#if defined DEBUG

#define MSM_BUS_DBG(msg, ...) \
	printk(KERN_DEBUG "AXI: %s(): " msg, __func__, ## __VA_ARGS__)
#define MSM_FAB_DBG(msg, ...) \
	dev_dbg(&fabric->fabdev.dev, "AXI: %s(): " msg, __func__, ## \
	__VA_ARGS__)

#else
#define MSM_BUS_DBG(msg, ...)
#define MSM_FAB_DBG(msg, ...)
#endif

#define MSM_BUS_ERR(msg, ...) \
	printk(KERN_ERR "AXI: %s(): " msg, __func__, ## __VA_ARGS__)
#define MSM_BUS_WARN(msg, ...) \
	printk(KERN_WARNING "AXI: %s(): " msg, __func__, ## __VA_ARGS__)
#define MSM_FAB_ERR(msg, ...) \
	dev_err(&fabric->fabdev.dev, "AXI: %s(): " msg, __func__, ## \
	__VA_ARGS__)

enum msm_bus_dbg_op_type {
	MSM_BUS_DBG_UNREGISTER = -2,
	MSM_BUS_DBG_REGISTER,
	MSM_BUS_DBG_OP = 1,
};

extern struct bus_type msm_bus_type;

struct path_node {
	unsigned long clk;
	unsigned long a_clk;
	unsigned long bw;
	unsigned long a_bw;
	unsigned long *sel_clk;
	unsigned long *sel_bw;
	int next;
};

struct msm_bus_link_info {
	unsigned long clk;
	unsigned long a_clk;
	unsigned long *sel_clk;
	unsigned long memclk;
	long  bw;
	long a_bw;
	long *sel_bw;
	int tier;
};

struct msm_bus_inode_info {
	struct msm_bus_node_info *node_info;
	unsigned long max_bw;
	unsigned long max_clk;
	struct msm_bus_link_info link_info;
	int num_pnodes;
	struct path_node *pnode;
	int commit_index;
	struct clk *nodeclk;
	struct clk *a_nodeclk;
	struct clk *memclk;
};

struct commit_data {
	uint16_t *bwsum;
	uint16_t *arb;
};

struct msm_bus_fabric_device {
	int id;
	const char *name;
	struct device dev;
	const struct msm_bus_fab_algorithm *algo;
	int visited;
};
#define to_msm_bus_fabric_device(d) container_of(d, \
		struct msm_bus_fabric_device, d)


struct msm_bus_fab_algorithm {
	int (*update_clks)(struct msm_bus_fabric_device *fabdev,
		struct msm_bus_inode_info *pme, int index,
		unsigned long curr_clk, unsigned long req_clk,
		unsigned long bwsum, int flag, int context,
		unsigned int cl_active_flag);
	int (*port_halt)(struct msm_bus_fabric_device *fabdev, int portid);
	int (*port_unhalt)(struct msm_bus_fabric_device *fabdev, int portid);
	int (*commit)(struct msm_bus_fabric_device *fabdev,
		int active_only);
	struct msm_bus_inode_info *(*find_node)(struct msm_bus_fabric_device
		*fabdev, int id);
	struct msm_bus_inode_info *(*find_gw_node)(struct msm_bus_fabric_device
		*fabdev, int id);
	struct list_head *(*get_gw_list)(struct msm_bus_fabric_device *fabdev);
	void (*update_bw)(struct msm_bus_fabric_device *fabdev, struct
		msm_bus_inode_info * hop, struct msm_bus_inode_info *info,
		int add_bw, int master_tier, int context);
};

/**
 * Used to store the list of fabrics and other info to be
 * maintained outside the fabric structure.
 * Used while calculating path, and to find fabric ptrs
 */
struct msm_bus_fabnodeinfo {
	struct list_head list;
	struct msm_bus_inode_info *info;
};

struct msm_bus_client {
	int id;
	struct msm_bus_scale_pdata *pdata;
	int *src_pnode;
	int curr;
};

int msm_bus_fabric_device_register(struct msm_bus_fabric_device *fabric);
void msm_bus_fabric_device_unregister(struct msm_bus_fabric_device *fabric);
struct msm_bus_fabric_device *msm_bus_get_fabric(int fabid);

#ifdef CONFIG_DEBUG_FS
void msm_bus_dbg_client_data(struct msm_bus_scale_pdata *pdata, int index,
	uint32_t cl);
void msm_bus_dbg_commit_data(const char *fabname, struct commit_data *cdata,
	int nmasters, int nslaves, int ntslaves, int op);
#else
static inline void msm_bus_dbg_client_data(struct msm_bus_scale_pdata *pdata,
	int index, uint32_t cl)
{
}
static inline void msm_bus_dbg_commit_data(const char *fabname,
	struct commit_data *cdata, int nmasters, int nslaves, int ntslaves,
	int op)
{
}
#endif

#endif /*_ARCH_ARM_MACH_MSM_BUS_CORE_H*/
