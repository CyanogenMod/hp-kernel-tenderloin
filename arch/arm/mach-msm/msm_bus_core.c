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
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/radix-tree.h>
#include <linux/clk.h>
#include <mach/msm_bus_board.h>
#include <mach/msm_bus.h>
#include "msm_bus_core.h"

#define INDEX_MASK 0x0000FFFF
#define PNODE_MASK 0xFFFF0000
#define SHIFT_VAL 16
#define BWMASK 0x7FFF
#define TIERMASK 0x8000
#define GET_TIER(n) (((n) & TIERMASK) >> 15)
#define CREATE_PNODE_ID(n, i) (((n) << SHIFT_VAL) | (i))
#define GET_INDEX(n) ((n) & INDEX_MASK)
#define GET_NODE(n) ((n) >> SHIFT_VAL)
#define IS_NODE(n) ((n) % FABRIC_ID_KEY)
#define ACTIVE_CTX 1
#define SEL_FAB_CLK 1
#define SEL_SLAVE_CLK 0

#define SELECT_BW_CLK(active_only, x) \
	do { \
		if (active_only) { \
			x.sel_bw = &x.a_bw; \
			x.sel_clk = &x.a_clk; \
		} else { \
			x.sel_bw = &x.bw; \
			x.sel_clk = &x.clk; \
		} \
	} while (0);

static DEFINE_MUTEX(msm_bus_lock);
static atomic_t num_fab = ATOMIC_INIT(0);

int msm_bus_device_match(struct device *dev, void* id)
{
	struct msm_bus_fabric_device *fabdev = to_msm_bus_fabric_device(dev);

	if (!fabdev) {
		MSM_BUS_WARN("Fabric %p returning 0\n", fabdev);
		return 0;
	}
	return (fabdev->id == (int)id);
}

struct bus_type msm_bus_type = {
	.name      = "msm-bus-type",
};
EXPORT_SYMBOL(msm_bus_type);

/**
 * msm_bus_get_fabric_device() - This function is used to search for
 * the fabric device on the bus
 * @fabid: Fabric id
 * Function returns: Pointer to the fabric device
 */
struct msm_bus_fabric_device *msm_bus_get_fabric_device(int fabid)
{
	struct device *dev;
	struct msm_bus_fabric_device *fabric;
	dev = bus_find_device(&msm_bus_type, NULL, (void *)fabid,
		msm_bus_device_match);
	fabric = to_msm_bus_fabric_device(dev);
	return fabric;
}

/**
 * add_path_node: Adds the path information to the current node
 * @info: Internal node info structure
 * @next: Combination of the id and index of the next node
 * Function returns: Number of pnodes (path_nodes) on success,
 * error on failure.
 *
 * Every node maintains the list of path nodes. A path node is
 * reached by finding the node-id and index stored at the current
 * node. This makes updating the paths with requested bw and clock
 * values efficient, as it avoids lookup for each update-path request.
 */
static int add_path_node(struct msm_bus_inode_info *info, int next)
{
	struct path_node *pnode;
	int i;
	if (!info) {
		MSM_BUS_ERR("Cannot find node info!: id :%d\n",
				info->node_info->priv_id);
		return -ENXIO;
	}

	for (i = 0; i <= info->num_pnodes; i++) {
		if (info->pnode[i].next == -2) {
			MSM_BUS_DBG("Reusing pnode for info: %d at index: %d\n",
				info->node_info->priv_id, i);
			info->pnode[i].clk = 0;
			info->pnode[i].a_clk = 0;
			info->pnode[i].a_bw = 0;
			info->pnode[i].bw = 0;
			info->pnode[i].next = next;
			MSM_BUS_DBG("%d[%d] : (%d, %d)\n",
				info->node_info->priv_id, i, GET_NODE(next),
				GET_INDEX(next));
			return i;
		}
	}

	info->num_pnodes++;
	pnode = krealloc(info->pnode,
		((info->num_pnodes + 1) * sizeof(struct path_node))
		, GFP_KERNEL);
	if (IS_ERR(pnode)) {
		MSM_BUS_ERR("Error creating path node!\n");
		info->num_pnodes--;
		return -ENOMEM;
	}
	info->pnode = pnode;
	info->pnode[info->num_pnodes].clk = 0;
	info->pnode[info->num_pnodes].bw = 0;
	info->pnode[info->num_pnodes].a_clk = 0;
	info->pnode[info->num_pnodes].a_bw = 0;
	info->pnode[info->num_pnodes].next = next;
	MSM_BUS_DBG("%d[%d] : (%d, %d)\n", info->node_info->priv_id,
		info->num_pnodes, GET_NODE(next), GET_INDEX(next));
	return info->num_pnodes;
}

static int clearvisitedflag(struct device *dev, void *data)
{
	struct msm_bus_fabric_device *fabdev = to_msm_bus_fabric_device(dev);
	fabdev->visited = false;
	return 0;
}

/**
 * getpath() - Finds the path from the topology between src and dest
 * @src: Source. This is the master from which the request originates
 * @dest: Destination. This is the slave to which we're trying to reach
 *
 * Function returns: next_pnode_id. The higher 16 bits of the next_pnode_id
 * represent the src id of the  next node on path. The lower 16 bits of the
 * next_pnode_id represent the "index", which is the next entry in the array
 * of pnodes for that node to fill in clk and bw values. This is created using
 * CREATE_PNODE_ID. The return value is stored in ret_pnode, and this is added
 * to the list of path nodes.
 *
 * This function recursively finds the path by updating the src to the
 * closest possible node to dest.
 */
static int getpath(int src, int dest)
{
	int pnode_num = -1, i;
	struct msm_bus_fabnodeinfo *fabnodeinfo;
	struct msm_bus_fabric_device *fabdev;
	int next_pnode_id = -1;
	struct msm_bus_inode_info *info = NULL;
	int _src = src/FABRIC_ID_KEY;
	int _dst = dest/FABRIC_ID_KEY;
	int ret_pnode = -1;
	int fabid = GET_FABID(src);

	/* Find the location of fabric for the src */
	MSM_BUS_DBG("%d --> %d\n", src, dest);

	fabdev = msm_bus_get_fabric_device(fabid);
	if (!fabdev) {
		MSM_BUS_WARN("Fabric Not yet registered. Try again\n");
		return -ENXIO;
	}

	/* Are we there yet? */
	if (src == dest) {
		info = fabdev->algo->find_node(fabdev, src);
		for (i = 0; i <= info->num_pnodes; i++) {
			if (info->pnode[i].next == -2) {
				MSM_BUS_DBG("src = dst  Reusing pnode for"
				" info: %d at index: %d\n",
				info->node_info->priv_id, i);
				next_pnode_id = CREATE_PNODE_ID(src, i);
				info->pnode[i].clk = 0;
				info->pnode[i].bw = 0;
				info->pnode[i].next = next_pnode_id;
				MSM_BUS_DBG("returning: %d, %d\n", GET_NODE
				(next_pnode_id), GET_INDEX(next_pnode_id));
				return next_pnode_id;
			}
		}
		next_pnode_id = CREATE_PNODE_ID(src, (info->num_pnodes + 1));
		pnode_num = add_path_node(info, next_pnode_id);
		if (pnode_num < 0) {
			MSM_BUS_DBG("error adding path node\n");
			return -ENXIO;
		}
		MSM_BUS_DBG("returning: %d, %d\n", GET_NODE(next_pnode_id),
			GET_INDEX(next_pnode_id));
		return next_pnode_id;
	} else if (_src == _dst) {
		/*
		 * src and dest belong to same fabric, find the destination
		 * from the radix tree
		 */
		info = fabdev->algo->find_node(fabdev, dest);
		ret_pnode = getpath(info->node_info->priv_id, dest);
		next_pnode_id = ret_pnode;
	} else {
		/* find the dest fabric */
		int trynextgw = true;
		struct list_head *gateways = fabdev->algo->get_gw_list(fabdev);
		list_for_each_entry(fabnodeinfo, gateways, list) {
		/* see if the destination is at a connected fabric */
			if (_dst == (fabnodeinfo->info->node_info->priv_id /
				FABRIC_ID_KEY) && !(fabdev->visited)) {
				/* Found the fab on which the device exists */
				info = fabnodeinfo->info;
				trynextgw = false;
				ret_pnode = getpath(info->node_info->priv_id,
					dest);
				pnode_num = add_path_node(info, ret_pnode);
				if (pnode_num < 0) {
					MSM_BUS_DBG("Error adding path node\n");
					return -ENXIO;
				}
				next_pnode_id = CREATE_PNODE_ID(
					info->node_info->priv_id, pnode_num);
				break;
			}
		}

		/* find the gateway */
		if (trynextgw) {
			gateways = fabdev->algo->get_gw_list(fabdev);
			list_for_each_entry(fabnodeinfo, gateways, list) {
				struct msm_bus_fabric_device *gwfab =
					msm_bus_get_fabric_device(fabnodeinfo->
						info->node_info->priv_id);
				if (!gwfab->visited) {
					MSM_BUS_DBG("VISITED ID: %d\n",
						gwfab->id);
					gwfab->visited = true;
					info = fabnodeinfo->info;
					ret_pnode = getpath(info->
						node_info->priv_id, dest);
					if (ret_pnode >= 0) {
						pnode_num = add_path_node(info,
							ret_pnode);
						if (pnode_num < 0) {
							MSM_BUS_ERR("Malloc"
							"failure in adding"
							"path node\n");
							return -ENXIO;
						}
						next_pnode_id = CREATE_PNODE_ID(
						info->node_info->priv_id,
						pnode_num);
						break;
					}
				}
			}
			if (next_pnode_id < 0)
				return -EPERM;
		}
	}

	if (!IS_NODE(src)) {
		MSM_BUS_DBG("Returning next_pnode_id:%d[%d]\n", GET_NODE(
			next_pnode_id), GET_INDEX(next_pnode_id));
		return next_pnode_id;
	}
	info = fabdev->algo->find_node(fabdev, src);
	if (!info) {
		MSM_BUS_ERR("Node info not found.\n");
		return -EPERM;
	}
	pnode_num = add_path_node(info, next_pnode_id);
	MSM_BUS_DBG(" Last: %d[%d] = (%d, %d)\n",
		src, info->num_pnodes, GET_NODE(next_pnode_id),
		GET_INDEX(next_pnode_id));
	MSM_BUS_DBG("returning: %d, %d\n", src, pnode_num);
	return CREATE_PNODE_ID(src, pnode_num);
}

/**
 * msm_bus_fabric_device_register() - Registers a fabric on msm bus
 * @fabdev: Fabric device to be registered
 */
int msm_bus_fabric_device_register(struct msm_bus_fabric_device *fabdev)
{
	int ret = 0;
	fabdev->dev.bus = &msm_bus_type;
	ret = dev_set_name(&fabdev->dev, fabdev->name);
	if (ret) {
		MSM_BUS_ERR("error setting dev name\n");
		goto err;
	}
	ret = device_register(&fabdev->dev);
	if (ret < 0) {
		MSM_BUS_ERR("error registering device%d %s\n",
				ret, fabdev->name);
		goto err;
	}
	atomic_inc(&num_fab);
err:
	return ret;
}

/**
 * msm_bus_fabric_device_unregister() - Unregisters the fabric
 * devices from the msm bus
 */
void msm_bus_fabric_device_unregister(struct msm_bus_fabric_device *fabdev)
{
	device_unregister(&fabdev->dev);
	atomic_dec(&num_fab);
}

/**
 * update_path() - Update the path with the bandwidth and clock values, as
 * requested by the client.
 *
 * @curr: Current source node, as specified in the client vector (master)
 * @pnode: The first-hop node on the path, stored in the internal client struct
 * @req_clk: Requested clock value from the vector
 * @req_bw: Requested bandwidth value from the vector
 * @curr_clk: Current clock frequency
 * @curr_bw: Currently allocated bandwidth
 *
 * This function updates the nodes on the path calculated using getpath(), with
 * clock and bandwidth values. The sum of bandwidths, and the max of clock
 * frequencies is calculated at each node on the path. Commit data to be sent
 * to RPM for each master and slave is also calculated here.
 */
static int update_path(int curr, int pnode, unsigned req_clk, unsigned req_bw,
		unsigned curr_clk, unsigned curr_bw, unsigned int active_ctx,
		unsigned int cl_active_flag)
{
	int index, ret = 0;
	struct msm_bus_inode_info *info;
	int next_pnode;
	int add_bw = req_bw - curr_bw;
	unsigned bwsum = 0;
	unsigned req_clk_hz, curr_clk_hz, bwsum_hz;
	int master_tier = 0;
	struct msm_bus_fabric_device *fabdev = msm_bus_get_fabric_device
		(GET_FABID(curr));

	MSM_BUS_DBG("args: %d %d %d %u %u %u %u %u\n",
		curr, GET_NODE(pnode), GET_INDEX(pnode), req_clk, req_bw,
		curr_clk, curr_bw, active_ctx);
	index = GET_INDEX(pnode);
	MSM_BUS_DBG("Client passed index :%d\n", index);
	info = fabdev->algo->find_node(fabdev, curr);
	if (!info) {
		MSM_BUS_ERR("Cannot find node info!\n");
		return -ENXIO;
	}

	SELECT_BW_CLK(active_ctx, info->link_info);
	SELECT_BW_CLK(active_ctx, info->pnode[index]);
	*info->link_info.sel_bw += add_bw;
	*info->pnode[index].sel_bw += add_bw;
	info->link_info.tier = info->node_info->tier;
	master_tier = info->node_info->tier;

	do {
		struct msm_bus_inode_info *hop;
		fabdev = msm_bus_get_fabric_device(GET_FABID(curr));
		if (!fabdev) {
			MSM_BUS_ERR("Fabric not found\n");
			return -ENXIO;
		}
		MSM_BUS_DBG("id: %d\n", info->node_info->priv_id);

		/* find next node and index */
		next_pnode = info->pnode[index].next;
		curr = GET_NODE(next_pnode);
		index = GET_INDEX(next_pnode);
		MSM_BUS_DBG("id:%d, next: %d\n", info->
		    node_info->priv_id, curr);

		/* Get hop */
		/* check if we are here as gateway, or does the hop belong to
		 * this fabric */
		if (IS_NODE(curr))
			hop = fabdev->algo->find_node(fabdev, curr);
		else
			hop = fabdev->algo->find_gw_node(fabdev, curr);
		if (!hop) {
			MSM_BUS_ERR("Null Info found for hop\n");
			return -ENXIO;
		}

		SELECT_BW_CLK(active_ctx, hop->link_info);
		SELECT_BW_CLK(active_ctx, hop->pnode[index]);

		*hop->link_info.sel_bw += add_bw;
		*hop->pnode[index].sel_clk = BW_TO_CLK_FREQ_HZ(hop->node_info->
			buswidth, req_clk);
		*hop->pnode[index].sel_bw += add_bw;
		MSM_BUS_DBG("fabric: %d slave: %d, slave-width: %d info: %d\n",
			fabdev->id, hop->node_info->priv_id, hop->node_info->
			buswidth, info->node_info->priv_id);
		/* Update Bandwidth */
		fabdev->algo->update_bw(fabdev, hop, info, add_bw,
			master_tier, active_ctx);
		bwsum = (uint16_t)*hop->link_info.sel_bw;
		/* Update Fabric clocks */
		curr_clk_hz = BW_TO_CLK_FREQ_HZ(hop->node_info->buswidth,
			curr_clk);
		req_clk_hz = BW_TO_CLK_FREQ_HZ(hop->node_info->buswidth,
			req_clk);
		bwsum_hz = BW_TO_CLK_FREQ_HZ(hop->node_info->buswidth,
			MSM_BUS_GET_BW_BYTES(bwsum));
		MSM_BUS_DBG("Calling update-clks: curr_hz: %u, req_hz: %u,"
			" bw_hz %u\n", curr_clk, req_clk, bwsum_hz);
		ret = fabdev->algo->update_clks(fabdev, hop, index,
			curr_clk_hz, req_clk_hz, bwsum_hz, SEL_FAB_CLK,
			active_ctx, cl_active_flag);
		if (ret)
			MSM_BUS_WARN("Failed to update clk\n");
		info = hop;
	} while (GET_NODE(info->pnode[index].next) != info->node_info->priv_id);

	/* Update BW, clk after exiting the loop for the last one */
	if (!info) {
		MSM_BUS_ERR("Cannot find node info!\n");
		return -ENXIO;
	}
	/* Update slave clocks */
	ret = fabdev->algo->update_clks(fabdev, info, index, curr_clk_hz,
	    req_clk_hz, bwsum_hz, SEL_SLAVE_CLK, active_ctx, cl_active_flag);
	if (ret)
		MSM_BUS_ERR("Failed to update clk\n");
	return ret;
}

/**
 * msm_bus_commit_fn() - Commits the data for fabric to rpm
 * @dev: fabric device
 * @data: NULL
 */
static int msm_bus_commit_fn(struct device *dev, void *data)
{
	int ret = 0;
	struct msm_bus_fabric_device *fabdev = to_msm_bus_fabric_device(dev);
	MSM_BUS_DBG("Committing: fabid: %d\n", fabdev->id);
	ret = fabdev->algo->commit(fabdev, (int)data);
	return ret;
}

/* msm bus client related EXPORTED functions */

/**
 * msm_bus_scale_register_client() - Register the clients with the msm bus
 * driver
 * @pdata: Platform data of the client, containing src, dest, ab, ib
 *
 * Client data contains the vectors specifying arbitrated bandwidth (ab)
 * and instantaneous bandwidth (ib) requested between a particular
 * src and dest.
 */
uint32_t msm_bus_scale_register_client(struct msm_bus_scale_pdata *pdata)
{
	struct msm_bus_client *client = NULL;
	int i;
	int src, dest;

	if (atomic_read(&num_fab) < NUM_FAB) {
		MSM_BUS_ERR("Can't register client!\n"
				"Num of fabrics up: %d\n",
				atomic_read(&num_fab));
		goto err;
	}

	client = kzalloc(sizeof(struct msm_bus_client), GFP_KERNEL);
	if (!client) {
		MSM_BUS_ERR("Error allocating client\n");
		goto err;
	}
	mutex_lock(&msm_bus_lock);
	client->pdata = pdata;
	client->curr = -1;
	for (i = 0; i < pdata->usecase->num_paths; i++) {
		int *pnode;
		struct msm_bus_fabric_device *srcfab;
		pnode = krealloc(client->src_pnode, ((i + 1) * sizeof(int)),
			GFP_KERNEL);
		if (!IS_ERR(pnode))
			client->src_pnode = pnode;
		else {
			MSM_BUS_ERR("Invalid Pnode ptr!\n");
			continue;
		}
		src = msm_bus_board_get_iid(pdata->usecase->vectors[i].src);
		dest = msm_bus_board_get_iid(pdata->usecase->vectors[i].dst);
		srcfab = msm_bus_get_fabric_device(GET_FABID(src));
		srcfab->visited = true;
		pnode[i] = getpath(src, dest);
		bus_for_each_dev(&msm_bus_type, NULL, NULL, clearvisitedflag);
		if (pnode[i] < 0) {
			MSM_BUS_ERR("Cannot register client now! Try again!\n");
			kfree(client->src_pnode);
			kfree(client);
			mutex_unlock(&msm_bus_lock);
			goto err;
		}
	}
	msm_bus_dbg_client_data(client->pdata, MSM_BUS_DBG_REGISTER,
		(uint32_t)client);
	mutex_unlock(&msm_bus_lock);
	MSM_BUS_DBG("ret: %u num_paths: %d\n", (uint32_t)client,
		pdata->usecase->num_paths);
	return (uint32_t)(client);
err:
	return 0;
}
EXPORT_SYMBOL(msm_bus_scale_register_client);

/**
 * msm_bus_scale_client_update_request() - Update the request for bandwidth
 * from a particular client
 *
 * cl: Handle to the client
 * index: Index into the vector, to which the bw and clock values need to be
 * updated
 */
int msm_bus_scale_client_update_request(uint32_t cl, unsigned index)
{
	int i, ret = 0;
	struct msm_bus_scale_pdata *pdata;
	unsigned int req_clk, req_bw, curr_clk, curr_bw;
	int pnode, src, curr;
	struct msm_bus_client *client = (struct msm_bus_client *)cl;
	int context;
	if (IS_ERR(client)) {
		MSM_BUS_ERR("msm_bus_scale_client update req error %d\n",
				(uint32_t)client);
		return -ENXIO;
	}

	mutex_lock(&msm_bus_lock);
	if (client->curr == index)
		goto err;

	curr = client->curr;
	pdata = client->pdata;
	MSM_BUS_DBG("cl: %u index: %d curr: %d"
			" num_paths: %d\n", cl, index, client->curr,
			client->pdata->usecase->num_paths);

	for (i = 0; i < pdata->usecase->num_paths; i++) {
		src = msm_bus_board_get_iid(client->pdata->usecase[index].
			vectors[i].src);
		pnode = client->src_pnode[i];
		req_clk = client->pdata->usecase[index].vectors[i].ib;
		req_bw = MSM_BUS_BW_VAL_FROM_BYTES(client->pdata->
				usecase[index].vectors[i].ab);
		if (curr < 0) {
			curr_clk = 0;
			curr_bw = 0;
		} else {
			curr_clk = client->pdata->usecase[curr].vectors[i].ib;
			curr_bw = MSM_BUS_BW_VAL_FROM_BYTES(client->pdata->
					usecase[curr].vectors[i].ab);
			MSM_BUS_DBG("ab: %d ib: %d\n", curr_bw, curr_clk);
		}

		if (!pdata->active_only) {
			ret = update_path(src, pnode, req_clk, req_bw,
				curr_clk, curr_bw, 0, pdata->active_only);
			if (ret) {
				MSM_BUS_ERR("Update path failed! %d\n", ret);
				goto err;
			}
		}

		ret = update_path(src, pnode, req_clk, req_bw, curr_clk,
				curr_bw, ACTIVE_CTX, pdata->active_only);
		if (ret) {
			MSM_BUS_ERR("Update Path failed! %d\n", ret);
			goto err;
		}
	}

	client->curr = index;
	context = ACTIVE_CTX;
	msm_bus_dbg_client_data(client->pdata, index, cl);
	bus_for_each_dev(&msm_bus_type, NULL, (void *)context,
		msm_bus_commit_fn);

err:
	mutex_unlock(&msm_bus_lock);
	return ret;
}
EXPORT_SYMBOL(msm_bus_scale_client_update_request);

int reset_pnodes(int curr, int pnode)
{
	struct msm_bus_inode_info *info;
	struct msm_bus_fabric_device *fabdev;
	int index, next_pnode;
	fabdev = msm_bus_get_fabric_device(GET_FABID(curr));
	index = GET_INDEX(pnode);
	info = fabdev->algo->find_node(fabdev, curr);
	if (!info) {
		MSM_BUS_ERR("Cannot find node info!\n");
		return -ENXIO;
	}
	MSM_BUS_DBG("Starting the loop--remove\n");
	do {
		struct msm_bus_inode_info *hop;
		fabdev = msm_bus_get_fabric_device(GET_FABID(curr));
		if (!fabdev) {
			MSM_BUS_ERR("Fabric not found\n");
				return -ENXIO;
		}

		next_pnode = info->pnode[index].next;
		info->pnode[index].next = -2;
		curr = GET_NODE(next_pnode);
		index = GET_INDEX(next_pnode);
		if (IS_NODE(curr))
			hop = fabdev->algo->find_node(fabdev, curr);
		else
			hop = fabdev->algo->find_gw_node(fabdev, curr);
		if (!hop) {
			MSM_BUS_ERR("Null Info found for hop\n");
			return -ENXIO;
		}

		MSM_BUS_DBG("%d[%d] = %d\n", info->node_info->priv_id, index,
			info->pnode[index].next);
		MSM_BUS_DBG("num_pnodes: %d: %d\n", info->node_info->priv_id,
			info->num_pnodes);
		info = hop;
	} while (GET_NODE(info->pnode[index].next) != info->node_info->priv_id);

	info->pnode[index].next = -2;
	MSM_BUS_DBG("%d[%d] = %d\n", info->node_info->priv_id, index,
		info->pnode[index].next);
	MSM_BUS_DBG("num_pnodes: %d: %d\n", info->node_info->priv_id,
		info->num_pnodes);
	return 0;
}

void msm_bus_scale_client_reset_pnodes(uint32_t cl)
{
	int i, src, pnode, index;
	struct msm_bus_client *client = (struct msm_bus_client *)(cl);
	if (IS_ERR(client)) {
		MSM_BUS_ERR("msm_bus_scale_reset_pnodes error\n");
		return;
	}
	index = 0;
	for (i = 0; i < client->pdata->usecase->num_paths; i++) {
		src = msm_bus_board_get_iid(
			client->pdata->usecase[index].vectors[i].src);
		pnode = client->src_pnode[i];
		MSM_BUS_DBG("(%d, %d)\n", GET_NODE(pnode), GET_INDEX(pnode));
		reset_pnodes(src, pnode);
	}
}

/**
 * msm_bus_scale_unregister_client() - Unregister the client from the bus driver
 * @cl: Handle to the client
 */
void msm_bus_scale_unregister_client(uint32_t cl)
{
	struct msm_bus_client *client = (struct msm_bus_client *)(cl);
	if (IS_ERR(client) || (!client))
		return;
	if (client->curr != 0)
		msm_bus_scale_client_update_request(cl, 0);
	MSM_BUS_DBG("Unregistering client %d\n", cl);
	mutex_lock(&msm_bus_lock);
	msm_bus_scale_client_reset_pnodes(cl);
	msm_bus_dbg_client_data(client->pdata, MSM_BUS_DBG_UNREGISTER, cl);
	mutex_unlock(&msm_bus_lock);
	kfree(client->src_pnode);
	kfree(client);
}
EXPORT_SYMBOL(msm_bus_scale_unregister_client);

/**
 * msm_bus_axi_porthalt() - Halt the given axi master port
 * @master_port: AXI Master port to be halted
 */
int msm_bus_axi_porthalt(int master_port)
{
	int ret = 0;
	int priv_id;
	struct msm_bus_fabric_device *fabdev;

	priv_id = msm_bus_board_get_iid(master_port);
	MSM_BUS_DBG("master_port: %d iid: %d fabid%d\n",
		master_port, priv_id, GET_FABID(priv_id));
	fabdev = msm_bus_get_fabric_device(GET_FABID(priv_id));
	if (IS_ERR(fabdev)) {
		MSM_BUS_ERR("Fabric device not found for mport: %d\n",
			master_port);
		return -ENODEV;
	}
	mutex_lock(&msm_bus_lock);
	ret = fabdev->algo->port_halt(fabdev, priv_id);
	mutex_unlock(&msm_bus_lock);
	return ret;
}
EXPORT_SYMBOL(msm_bus_axi_porthalt);

/**
 * msm_bus_axi_portunhalt() - Unhalt the given axi master port
 * @master_port: AXI Master port to be unhalted
 */
int msm_bus_axi_portunhalt(int master_port)
{
	int ret = 0;
	int priv_id;
	struct msm_bus_fabric_device *fabdev;

	priv_id = msm_bus_board_get_iid(master_port);
	MSM_BUS_DBG("master_port: %d iid: %d fabid: %d\n",
		master_port, priv_id, GET_FABID(priv_id));
	fabdev = msm_bus_get_fabric_device(GET_FABID(priv_id));
	if (IS_ERR(fabdev)) {
		MSM_BUS_ERR("Fabric device not found for mport: %d\n",
			master_port);
		return -ENODEV;
	}
	mutex_lock(&msm_bus_lock);
	ret = fabdev->algo->port_unhalt(fabdev, priv_id);
	mutex_unlock(&msm_bus_lock);
	return ret;
}
EXPORT_SYMBOL(msm_bus_axi_portunhalt);

static void __exit msm_bus_exit(void)
{
	bus_unregister(&msm_bus_type);
}

static int __init msm_bus_init(void)
{
	int retval = 0;
	retval = bus_register(&msm_bus_type);
	if (retval)
		MSM_BUS_ERR("bus_register error! %d\n",
			retval);
	return retval;
}
postcore_initcall(msm_bus_init);
module_exit(msm_bus_exit);
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.2");
MODULE_ALIAS("platform:msm_bus");
