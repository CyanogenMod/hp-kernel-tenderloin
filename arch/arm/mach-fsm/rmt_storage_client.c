/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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

#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/wakelock.h>
#include <linux/rmt_storage_client.h>
#include <linux/debugfs.h>

#include <asm/uaccess.h>
#include <asm/pgtable.h>
#include <mach/msm_rpcrouter.h>
#include "smd_private.h"

enum {
	RMT_STORAGE_EVNT_OPEN = 0,
	RMT_STORAGE_EVNT_CLOSE,
	RMT_STORAGE_EVNT_WRITE_BLOCK,
	RMT_STORAGE_EVNT_GET_DEV_ERROR,
	RMT_STORAGE_EVNT_WRITE_IOVEC,
	RMT_STORAGE_EVNT_SEND_USER_DATA,
} rmt_storage_event;

struct shared_ramfs_entry {
	uint32_t client_id;   	/* Client id to uniquely identify a client */
	uint32_t base_addr;	/* Base address of shared RAMFS memory */
	uint32_t size;		/* Size of the shared RAMFS memory */
	uint32_t client_sts;	/* This will be initialized to 1 when
				   remote storage RPC client is ready
				   to process requests */
};
struct shared_ramfs_table {
	uint32_t magic_id;  	/* Identify RAMFS details in SMEM */
	uint32_t version;	/* Version of shared_ramfs_table */
	uint32_t entries;	/* Total number of valid entries   */
	struct shared_ramfs_entry ramfs_entry[3];	/* List all entries */
};

struct rmt_storage_client_info {
	unsigned long cids;
	struct rmt_shrd_mem_param rmt_shrd_mem;
	int open_excl;
	atomic_t total_events;
	wait_queue_head_t event_q;
	struct list_head event_list;
	/* Lock to protect event list */
	spinlock_t lock;
	/* Wakelock to be acquired when processing requests from modem */
	struct wake_lock wlock;
	atomic_t wcount;
	struct shared_ramfs_entry *smem_info;
	int sync_token;
};

struct rmt_storage_kevent {
	struct list_head list;
	struct rmt_storage_event event;
};

static struct rmt_storage_client_info *_rmc;
static struct msm_rpc_client *client;

#ifdef CONFIG_MSM_RMT_STORAGE_CLIENT_STATS
struct rmt_storage_stats {
       char path[MAX_PATH_NAME];
       unsigned long count;
       ktime_t start;
       ktime_t min;
       ktime_t max;
       ktime_t total;
};
static struct rmt_storage_stats client_stats[MAX_NUM_CLIENTS];
static struct dentry *stats_dentry;
#endif

#define RMT_STORAGE_APIPROG            0x300000A7

#define RMT_STORAGE_FORCE_SYNC_PROC 7
#define RMT_STORAGE_GET_SYNC_STATUS_PROC 8
#define RMT_STORAGE_WRITE_FINISH_PROC 2
#define RMT_STORAGE_REGISTER_OPEN_PROC 3
#define RMT_STORAGE_REGISTER_WRITE_IOVEC_PROC 4
#define RMT_STORAGE_REGISTER_CB_PROC 5
#define RMT_STORAGE_UN_REGISTER_CB_PROC 6
#define RMT_STORAGE_OPEN_CB_TYPE_PROC 1
#define RMT_STORAGE_WRITE_IOVEC_CB_TYPE_PROC 2
#define RMT_STORAGE_EVENT_CB_TYPE_PROC 3

static int rmt_storage_send_sts_arg(struct msm_rpc_client *client,
				struct msm_rpc_xdr *xdr, void *data)
{
	struct rmt_storage_send_sts *args = data;

	xdr_send_uint32(xdr, &args->handle);
	xdr_send_uint32(xdr, &args->err_code);
	xdr_send_uint32(xdr, &args->data);
	return 0;
}

static void put_event(struct rmt_storage_client_info *rmc,
			struct rmt_storage_kevent *kevent)
{
	spin_lock(&rmc->lock);
	list_add_tail(&kevent->list, &rmc->event_list);
	spin_unlock(&rmc->lock);
}

static struct rmt_storage_kevent *get_event(struct rmt_storage_client_info *rmc)
{
	struct rmt_storage_kevent *kevent = NULL;

	spin_lock(&rmc->lock);
	if (!list_empty(&rmc->event_list)) {
		kevent = list_first_entry(&rmc->event_list,
			struct rmt_storage_kevent, list);
		list_del(&kevent->list);
	}
	spin_unlock(&rmc->lock);
	return kevent;
}

static int rmt_storage_event_open_cb(struct rmt_storage_event *event_args,
		struct msm_rpc_xdr *xdr)
{
	uint32_t cid, len, event_type;
	char *path;
	int ret;
#ifdef CONFIG_MSM_RMT_STORAGE_CLIENT_STATS
	struct rmt_storage_stats *stats;
#endif

	xdr_recv_uint32(xdr, &event_type);
	if (event_type != RMT_STORAGE_EVNT_OPEN)
		return -1;

	pr_info("%s: open callback received\n", __func__);
	cid = find_first_zero_bit(&_rmc->cids, sizeof(_rmc->cids));
	if (cid > MAX_NUM_CLIENTS) {
		pr_err("%s: Max clients are reached\n", __func__);
		cid = 0;
		return cid;
	}

	__set_bit(cid, &_rmc->cids);

	ret = xdr_recv_bytes(xdr, (void **)&path, &len);
	if (ret || !path)
		return -1;

	memcpy(event_args->path, path, len);
	pr_info("open partition %s\n", event_args->path);
#ifdef CONFIG_MSM_RMT_STORAGE_CLIENT_STATS
	stats = &client_stats[cid - 1];
	memcpy(stats->path, event_args->path, len);
	stats->count = 0;
	stats->min.tv64 = KTIME_MAX;
	stats->max.tv64 =  0;
	stats->total.tv64 = 0;
#endif
	event_args->id = RMT_STORAGE_OPEN;
	event_args->handle = cid;

	kfree(path);
	return cid;
}

struct rmt_storage_close_args {
	uint32_t handle;
};

struct rmt_storage_write_block_args {
	uint32_t handle;
	uint32_t data_phy_addr;
	uint32_t sector_addr;
	uint32_t num_sector;
};

struct rmt_storage_get_err_args {
	uint32_t handle;
};

struct rmt_storage_user_data_args {
	uint32_t handle;
	uint32_t data;
};

struct rmt_storage_event_params {
	uint32_t type;
	union {
		struct rmt_storage_close_args close;
		struct rmt_storage_write_block_args write_block;
		struct rmt_storage_get_err_args get_err;
		struct rmt_storage_user_data_args user_data;
	} params;
};

static int rmt_storage_parse_params(struct msm_rpc_xdr *xdr,
		struct rmt_storage_event_params *event)
{
	xdr_recv_uint32(xdr, &event->type);

	switch (event->type) {
	case RMT_STORAGE_EVNT_CLOSE: {
		struct rmt_storage_close_args *args;
		args = &event->params.close;

		xdr_recv_uint32(xdr, &args->handle);
		break;
	}

	case RMT_STORAGE_EVNT_WRITE_BLOCK: {
		struct rmt_storage_write_block_args *args;
		args = &event->params.write_block;

		xdr_recv_uint32(xdr, &args->handle);
		xdr_recv_uint32(xdr, &args->data_phy_addr);
		xdr_recv_uint32(xdr, &args->sector_addr);
		xdr_recv_uint32(xdr, &args->num_sector);
		break;
	}

	case RMT_STORAGE_EVNT_GET_DEV_ERROR: {
		struct rmt_storage_get_err_args *args;
		args = &event->params.get_err;

		xdr_recv_uint32(xdr, &args->handle);
		break;
	}

	case RMT_STORAGE_EVNT_SEND_USER_DATA: {
		struct rmt_storage_user_data_args *args;
		args = &event->params.user_data;

		xdr_recv_uint32(xdr, &args->handle);
		xdr_recv_uint32(xdr, &args->data);
		break;
	}

	default:
		pr_err("%s: unknown event %d\n", __func__, event->type);
		return -1;
	}
	return 0;
}

static int rmt_storage_event_close_cb(struct rmt_storage_event *event_args,
		struct msm_rpc_xdr *xdr)
{
	struct rmt_storage_event_params *event;
	struct rmt_storage_close_args *close;
	uint32_t event_type;
	int ret;

	xdr_recv_uint32(xdr, &event_type);
	if (event_type != RMT_STORAGE_EVNT_CLOSE)
		return -1;

	pr_info("%s: close callback received\n", __func__);
	ret = xdr_recv_pointer(xdr, (void **)&event,
			sizeof(struct rmt_storage_event_params),
			rmt_storage_parse_params);

	if (ret || !event)
		return -1;

	close = &event->params.close;
	event_args->handle = close->handle;
	event_args->id = RMT_STORAGE_CLOSE;
	__clear_bit(event_args->handle, &_rmc->cids);

	kfree(event);
	return RMT_STORAGE_NO_ERROR;
}

static int rmt_storage_event_write_block_cb(
		struct rmt_storage_event *event_args,
		struct msm_rpc_xdr *xdr)
{
	struct rmt_storage_event_params *event;
	struct rmt_storage_write_block_args *write_block;
	struct rmt_storage_iovec_desc *xfer;
	uint32_t event_type;
	int ret;

	xdr_recv_uint32(xdr, &event_type);
	if (event_type != RMT_STORAGE_EVNT_WRITE_BLOCK)
		return -1;

	pr_info("%s: write block callback received\n", __func__);
	ret = xdr_recv_pointer(xdr, (void **)&event,
			sizeof(struct rmt_storage_event_params),
			rmt_storage_parse_params);

	if (ret || !event)
		return -1;

	write_block = &event->params.write_block;
	event_args->handle = write_block->handle;
	xfer = &event_args->xfer_desc[0];
	xfer->sector_addr = write_block->sector_addr;
	xfer->data_phy_addr = write_block->data_phy_addr;
	xfer->num_sector = write_block->num_sector;

	if (xfer->data_phy_addr < _rmc->rmt_shrd_mem.start ||
	   xfer->data_phy_addr > (_rmc->rmt_shrd_mem.start +
	   _rmc->rmt_shrd_mem.size)) {
		kfree(event);
		return -1;
	}

	event_args->xfer_cnt = 1;
	event_args->id = RMT_STORAGE_WRITE;

	if (atomic_inc_return(&_rmc->wcount) == 1)
		wake_lock(&_rmc->wlock);

	pr_debug("sec_addr = %u, data_addr = %x, num_sec = %d\n\n",
		xfer->sector_addr, xfer->data_phy_addr,
		xfer->num_sector);

	kfree(event);
	return RMT_STORAGE_NO_ERROR;
}

static int rmt_storage_event_get_err_cb(struct rmt_storage_event *event_args,
		struct msm_rpc_xdr *xdr)
{
	struct rmt_storage_event_params *event;
	struct rmt_storage_get_err_args *get_err;
	uint32_t event_type;
	int ret;

	xdr_recv_uint32(xdr, &event_type);
	if (event_type != RMT_STORAGE_EVNT_GET_DEV_ERROR)
		return -1;

	pr_info("%s: get err callback received\n", __func__);
	ret = xdr_recv_pointer(xdr, (void **)&event,
			sizeof(struct rmt_storage_event_params),
			rmt_storage_parse_params);

	if (ret || !event)
		return -1;

	get_err = &event->params.get_err;
	event_args->handle = get_err->handle;
	kfree(event);
	/* Not implemented */
	return -1;

}

static int rmt_storage_event_user_data_cb(struct rmt_storage_event *event_args,
		struct msm_rpc_xdr *xdr)
{
	struct rmt_storage_event_params *event;
	struct rmt_storage_user_data_args *user_data;
	uint32_t event_type;
	int ret;

	xdr_recv_uint32(xdr, &event_type);
	if (event_type != RMT_STORAGE_EVNT_SEND_USER_DATA)
		return -1;

	pr_info("%s: send user data callback received\n", __func__);
	ret = xdr_recv_pointer(xdr, (void **)&event,
			sizeof(struct rmt_storage_event_params),
			rmt_storage_parse_params);

	if (ret || !event)
		return -1;

	user_data = &event->params.user_data;
	event_args->handle = user_data->handle;
	event_args->usr_data = user_data->data;
	event_args->id = RMT_STORAGE_SEND_USER_DATA;

	kfree(event);
	return RMT_STORAGE_NO_ERROR;
}

static int rmt_storage_event_write_iovec_cb(
		struct rmt_storage_event *event_args,
		struct msm_rpc_xdr *xdr)
{
	struct rmt_storage_iovec_desc *xfer;
	uint32_t i, ent, event_type;
#ifdef CONFIG_MSM_RMT_STORAGE_CLIENT_STATS
	struct rmt_storage_stats *stats;
#endif

	xdr_recv_uint32(xdr, &event_type);
	if (event_type != RMT_STORAGE_EVNT_WRITE_IOVEC)
		return -1;

	pr_info("%s: write iovec callback received\n", __func__);
	xdr_recv_uint32(xdr, &event_args->handle);
	xdr_recv_uint32(xdr, &ent);
	pr_debug("handle = %d\n", event_args->handle);

#ifdef CONFIG_MSM_RMT_STORAGE_CLIENT_STATS
	stats = &client_stats[event_args->handle - 1];
	stats->start = ktime_get();
#endif
	for (i = 0; i < ent; i++) {
		xfer = &event_args->xfer_desc[i];
		xdr_recv_uint32(xdr, &xfer->sector_addr);
		xdr_recv_uint32(xdr, &xfer->data_phy_addr);
		xdr_recv_uint32(xdr, &xfer->num_sector);

		if (xfer->data_phy_addr < _rmc->rmt_shrd_mem.start ||
		   xfer->data_phy_addr > (_rmc->rmt_shrd_mem.start +
		   _rmc->rmt_shrd_mem.size))
			return -1;

		pr_debug("sec_addr = %u, data_addr = %x, num_sec = %d\n",
			xfer->sector_addr, xfer->data_phy_addr,
			xfer->num_sector);
	}
	xdr_recv_uint32(xdr, &event_args->xfer_cnt);
	event_args->id = RMT_STORAGE_WRITE;
	if (atomic_inc_return(&_rmc->wcount) == 1)
		wake_lock(&_rmc->wlock);

	pr_debug("iovec transfer count = %d\n\n", event_args->xfer_cnt);
	return RMT_STORAGE_NO_ERROR;
}

static int handle_rmt_storage_call(struct msm_rpc_client *client,
				struct rpc_request_hdr *req,
				struct msm_rpc_xdr *xdr)
{
	int rc;
	uint32_t result = RMT_STORAGE_NO_ERROR;
	uint32_t rpc_status = RPC_ACCEPTSTAT_SUCCESS;
	struct rmt_storage_client_info *rmc = _rmc;
	struct rmt_storage_event *event_args;
	struct rmt_storage_kevent *kevent;

	kevent = kmalloc(sizeof(struct rmt_storage_kevent), GFP_KERNEL);
	if (!kevent) {
		rpc_status = RPC_ACCEPTSTAT_SYSTEM_ERR;
		goto out;
	}
	event_args = &kevent->event;

	switch (req->procedure) {
	case RMT_STORAGE_OPEN_CB_TYPE_PROC:
		/* fall through */

	case RMT_STORAGE_WRITE_IOVEC_CB_TYPE_PROC:
		/* fall through */

	case RMT_STORAGE_EVENT_CB_TYPE_PROC: {
		uint32_t cb_id;
		int (*cb_func)(struct rmt_storage_event *event_args,
				struct msm_rpc_xdr *xdr);

		xdr_recv_uint32(xdr, &cb_id);
		cb_func = msm_rpc_get_cb_func(client, cb_id);

		if (!cb_func) {
			rpc_status = RPC_ACCEPTSTAT_GARBAGE_ARGS;
			kfree(kevent);
			goto out;
		}

		rc = cb_func(event_args, xdr);
		if (rc < 0) {
			pr_err("%s: Invalid parameters received \n", __func__);
			result = RMT_STORAGE_ERROR_PARAM;
			kfree(kevent);
			goto out;
		}
		result = (uint32_t) rc;
		break;
	}

	default:
		kfree(kevent);
		pr_err("%s: unknown procedure %d\n", __func__, req->procedure);
		rpc_status = RPC_ACCEPTSTAT_PROC_UNAVAIL;
		goto out;
	}
	put_event(rmc, kevent);
	atomic_inc(&rmc->total_events);
	wake_up(&rmc->event_q);

out:
	xdr_start_accepted_reply(xdr, rpc_status);
	xdr_send_uint32(xdr, &result);
	rc = xdr_send_msg(xdr);
	if (rc)
		pr_err("%s: send accepted reply failed: %d\n", __func__, rc);

	return rc;
}

static int rmt_storage_open(struct inode *ip, struct file *fp)
{
	int ret = 0;

	spin_lock(&_rmc->lock);

	if (!_rmc->open_excl)
		_rmc->open_excl = 1;
	else
		ret = -EBUSY;

	_rmc->smem_info->client_sts = 1;
	spin_unlock(&_rmc->lock);
	return ret;
}

static int rmt_storage_release(struct inode *ip, struct file *fp)
{

	spin_lock(&_rmc->lock);
	_rmc->open_excl = 0;
	spin_unlock(&_rmc->lock);

	return 0;
}

static long rmt_storage_ioctl(struct file *fp, unsigned int cmd,
			    unsigned long arg)
{
	int ret = 0;
	struct rmt_storage_client_info *rmc = _rmc;
	struct rmt_storage_kevent *kevent;
	struct rmt_storage_send_sts status;
#ifdef CONFIG_MSM_RMT_STORAGE_CLIENT_STATS
	struct rmt_storage_stats *stats;
	ktime_t curr_stat;
#endif

	switch (cmd) {

	case RMT_STORAGE_SHRD_MEM_PARAM:
		pr_info("%s: get shared memory parameters ioctl\n", __func__);
		if (copy_to_user((void __user *)arg, &rmc->rmt_shrd_mem,
			sizeof(struct rmt_shrd_mem_param))) {
			pr_err("%s: copy to user failed\n\n", __func__);
			ret = -EFAULT;
		}
		break;

	case RMT_STORAGE_WAIT_FOR_REQ:
		pr_info("%s: wait for request ioctl\n", __func__);
		if (atomic_read(&rmc->total_events) == 0) {
			ret = wait_event_interruptible(rmc->event_q,
				atomic_read(&rmc->total_events) != 0);
		}
		if (ret < 0)
			break;
		atomic_dec(&rmc->total_events);

		kevent = get_event(rmc);
		WARN_ON(kevent == NULL);
		if (copy_to_user((void __user *)arg, &kevent->event,
			sizeof(struct rmt_storage_event))) {
			pr_err("%s: copy to user failed\n\n", __func__);
			ret = -EFAULT;
		}
		kfree(kevent);
		break;

	case RMT_STORAGE_SEND_STATUS:
		pr_info("%s: send status ioctl\n", __func__);
		if (copy_from_user(&status, (void __user *)arg,
				sizeof(struct rmt_storage_send_sts))) {
			pr_err("%s: copy from user failed\n\n", __func__);
			ret = -EFAULT;
			if (atomic_dec_return(&rmc->wcount) == 0)
				wake_unlock(&rmc->wlock);
			break;
		}
#ifdef CONFIG_MSM_RMT_STORAGE_CLIENT_STATS
		stats = &client_stats[status.handle - 1];
		curr_stat = ktime_sub(ktime_get(), stats->start);
		stats->total = ktime_add(stats->total, curr_stat);
		stats->count++;
		if (curr_stat.tv64 < stats->min.tv64)
			stats->min = curr_stat;
		if (curr_stat.tv64 > stats->max.tv64)
			stats->max = curr_stat;
#endif
		ret = msm_rpc_client_req2(client,
				RMT_STORAGE_WRITE_FINISH_PROC,
				rmt_storage_send_sts_arg,
				&status, NULL, NULL, -1);
		if (ret < 0)
			pr_err("%s: send status failed with ret val = %d\n",
				__func__, ret);
		if (atomic_dec_return(&rmc->wcount) == 0)
			wake_unlock(&rmc->wlock);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

struct rmt_storage_sync_recv_arg {
	int data;
};

static int rmt_storage_receive_sync_arg(struct msm_rpc_client *client,
				struct msm_rpc_xdr *xdr, void *data)
{
	struct rmt_storage_sync_recv_arg *args = data;

	xdr_recv_int32(xdr, &args->data);
	_rmc->sync_token = args->data;
	return 0;
}

static int rmt_storage_force_sync(void)
{
	struct rmt_storage_sync_recv_arg args;
	int rc;

	rc = msm_rpc_client_req2(client,
			RMT_STORAGE_FORCE_SYNC_PROC, NULL, NULL,
			rmt_storage_receive_sync_arg, &args, -1);
	if (rc) {
		pr_err("%s: force sync RPC req failed: %d\n", __func__, rc);
		return rc;
	}
	return 0;
}

struct rmt_storage_sync_sts_arg {
	int token;
};

static int rmt_storage_send_sync_sts_arg(struct msm_rpc_client *client,
				struct msm_rpc_xdr *xdr, void *data)
{
	struct rmt_storage_sync_sts_arg *req = data;

	xdr_send_int32(xdr, &req->token);
	return 0;
}

static int rmt_storage_receive_sync_sts_arg(struct msm_rpc_client *client,
				struct msm_rpc_xdr *xdr, void *data)
{
	struct rmt_storage_sync_recv_arg *args = data;

	xdr_recv_int32(xdr, &args->data);
	return 0;
}

static int rmt_storage_get_sync_status(void)
{
	struct rmt_storage_sync_recv_arg recv_args;
	struct rmt_storage_sync_sts_arg send_args;
	int rc;

	if (_rmc->sync_token < 0)
		return -EINVAL;

	send_args.token = _rmc->sync_token;
	rc = msm_rpc_client_req2(client,
			RMT_STORAGE_GET_SYNC_STATUS_PROC,
			rmt_storage_send_sync_sts_arg, &send_args,
			rmt_storage_receive_sync_sts_arg, &recv_args, -1);
	if (rc) {
		pr_err("%s: sync status RPC req failed: %d\n", __func__, rc);
		return rc;
	}
	return recv_args.data;
}

static int rmt_storage_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct rmt_storage_client_info *rmc = _rmc;
	unsigned long vsize = vma->vm_end - vma->vm_start;
	int ret = -EINVAL;

	if (vma->vm_pgoff != 0) {
		pr_err("%s: error: zero offset is required\n", __func__);
		goto out;
	}

	if (vsize > rmc->rmt_shrd_mem.size) {
		pr_err("%s: error: size mismatch\n", __func__);
		goto out;
	}

	ret = io_remap_pfn_range(vma, vma->vm_start,
			rmc->rmt_shrd_mem.start >> PAGE_SHIFT,
			vsize, vma->vm_page_prot);
	if (ret < 0)
		pr_err("%s: failed with return val %d \n", __func__, ret);
out:
	return ret;
}

struct rmt_storage_reg_cb_args {
	uint32_t event;
	uint32_t cb_id;
};

static int rmt_storage_arg_cb(struct msm_rpc_client *client,
		struct msm_rpc_xdr *xdr, void *data)
{
	struct rmt_storage_reg_cb_args *args = data;

	xdr_send_uint32(xdr, &args->event);
	xdr_send_uint32(xdr, &args->cb_id);
	return 0;
}

static int rmt_storage_reg_cb(uint32_t proc, uint32_t event, void *callback)
{
	struct rmt_storage_reg_cb_args args;
	int rc, cb_id;

	cb_id = msm_rpc_add_cb_func(client, callback);
	if ((cb_id < 0) && (cb_id != MSM_RPC_CLIENT_NULL_CB_ID))
		return cb_id;

	args.event = event;
	args.cb_id = cb_id;

	rc = msm_rpc_client_req2(client, proc, rmt_storage_arg_cb,
			&args, NULL, NULL, -1);
	if (rc)
		pr_err("%s: Failed to register callback for event %d \n",
				__func__, event);
	return rc;
}

#ifdef CONFIG_MSM_RMT_STORAGE_CLIENT_STATS
static int rmt_storage_stats_open(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t rmt_storage_stats_read(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	uint32_t tot_clients;
	char buf[512];
	int max, j, i = 0;
	struct rmt_storage_stats *stats;

	max = sizeof(buf) - 1;
	tot_clients = find_first_zero_bit(&_rmc->cids, sizeof(_rmc->cids)) - 1;

	for (j = 0; j < tot_clients; j++) {
		stats = &client_stats[j];
		i += scnprintf(buf + i, max - i, "stats for partition %s:\n",
				stats->path);
		i += scnprintf(buf + i, max - i, "Min time: %lld us\n",
				ktime_to_us(stats->min));
		i += scnprintf(buf + i, max - i, "Max time: %lld us\n",
				ktime_to_us(stats->max));
		i += scnprintf(buf + i, max - i, "Total time: %lld us\n",
				ktime_to_us(stats->total));
		i += scnprintf(buf + i, max - i, "Total requests: %ld\n",
				stats->count);
		if (stats->count)
			i += scnprintf(buf + i, max - i, "Avg time: %lld us\n",
			     div_s64(ktime_to_us(stats->total), stats->count));
	}
	return simple_read_from_buffer(ubuf, count, ppos, buf, i);
}

static const struct file_operations debug_ops = {
	.owner = THIS_MODULE,
	.open = rmt_storage_stats_open,
	.read = rmt_storage_stats_read,
};
#endif

const struct file_operations rmt_storage_fops = {
	.owner = THIS_MODULE,
	.open = rmt_storage_open,
	.unlocked_ioctl	 = rmt_storage_ioctl,
	.mmap = rmt_storage_mmap,
	.release = rmt_storage_release,
};

static struct miscdevice rmt_storage_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "rmt_storage",
	.fops = &rmt_storage_fops,
};

#define RAMFS_INFO_MAGICNUMBER		0x654D4D43
#define RAMFS_INFO_VERSION		0x00000001
#define RAMFS_MODEMSTORAGE_ID		0x4D454653

static int rmt_storage_get_ramfs(struct rmt_storage_client_info *rmc)
{
	struct shared_ramfs_table *ramfs_table;
	struct shared_ramfs_entry *ramfs_entry;
	int index;

	ramfs_table = smem_alloc(SMEM_SEFS_INFO,
			sizeof(struct shared_ramfs_table));

	if (!ramfs_table) {
		pr_err("%s: No RAMFS table in SMEM\n", __func__);
		return -ENOENT;
	}

	if ((ramfs_table->magic_id != (u32) RAMFS_INFO_MAGICNUMBER) ||
		(ramfs_table->version != (u32) RAMFS_INFO_VERSION)) {
		pr_err("%s: Magic / Version mismatch:, "
		       "magic_id=%#x, format_version=%#x\n", __func__,
		       ramfs_table->magic_id, ramfs_table->version);
		return -ENOENT;
	}

	for (index = 0; index < ramfs_table->entries; index++) {
		ramfs_entry = &ramfs_table->ramfs_entry[index];

		/* Find a match for the Modem Storage RAMFS area */
		if (ramfs_entry->client_id == (u32) RAMFS_MODEMSTORAGE_ID) {
			pr_info("%s: RAMFS Info (from SMEM): "
				"Baseaddr = 0x%08x, Size = 0x%08x\n", __func__,
				ramfs_entry->base_addr, ramfs_entry->size);

			rmc->rmt_shrd_mem.start = ramfs_entry->base_addr;
			rmc->rmt_shrd_mem.size = ramfs_entry->size;
			rmc->smem_info = ramfs_entry;
			return 0;
		}
	}
	pr_err("%s: No valid match in the RAMFS table\n", __func__);
	return -ENOENT;
}

static ssize_t
set_force_sync(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int value, rc;

	sscanf(buf, "%d", &value);
	if (!!value) {
		rc = rmt_storage_force_sync();
		if (rc)
			return rc;
	}
	return count;
}

/* Returns -EINVAL for invalid sync token and an error value for any failure
 * in RPC call. Upon success, it returns a sync status of 1 (sync done)
 * or 0 (sync still pending).
 */
static ssize_t
show_sync_sts(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", rmt_storage_get_sync_status());
}

static DEVICE_ATTR(force_sync, S_IRUGO | S_IWUSR, NULL, set_force_sync);
static DEVICE_ATTR(sync_sts, S_IRUGO | S_IWUSR, show_sync_sts, NULL);
static struct attribute *dev_attrs[] = {
	&dev_attr_force_sync.attr,
	&dev_attr_sync_sts.attr,
	NULL,
};
static struct attribute_group dev_attr_grp = {
	.attrs = dev_attrs,
};

static int rmt_storage_probe(struct platform_device *pdev)
{
	struct rpcsvr_platform_device *dev;
	struct rmt_storage_client_info *rmc;
	int ret;

	dev = container_of(pdev, struct rpcsvr_platform_device, base);
	rmc = kzalloc(sizeof(struct rmt_storage_client_info), GFP_KERNEL);
	if (!rmc) {
		pr_err("%s: Unable to allocate memory\n", __func__);
		ret = -ENOMEM;
		goto out;
	}

	ret = rmt_storage_get_ramfs(rmc);
	if (ret)
		goto rmc_free;

	/* Initialization */
	init_waitqueue_head(&rmc->event_q);
	spin_lock_init(&rmc->lock);
	atomic_set(&rmc->total_events, 0);
	INIT_LIST_HEAD(&rmc->event_list);
	/* The client expects a non-zero return value for
	 * its open requests. Hence reserve 0 bit.  */
	__set_bit(0, &rmc->cids);
	atomic_set(&rmc->wcount, 0);
	wake_lock_init(&rmc->wlock, WAKE_LOCK_SUSPEND, "rmt_storage");
	_rmc = rmc;

	/* Client Registration */
	client = msm_rpc_register_client2("rmt_storage",
			dev->prog, dev->vers, 1,
			handle_rmt_storage_call);
	if (IS_ERR(client)) {
		pr_err("%s: Unable to register client (prog %.8x vers %.8x)\n",
				__func__, dev->prog, dev->vers);
		ret = PTR_ERR(client);
		goto destroy_wlock;
	}

	pr_info("%s: Remote storage RPC client initialized\n", __func__);

	/* Register a callback for each event */
	ret = rmt_storage_reg_cb(RMT_STORAGE_REGISTER_OPEN_PROC,
			RMT_STORAGE_EVNT_OPEN,
			rmt_storage_event_open_cb);

	if (ret)
		goto unregister_client;

	ret = rmt_storage_reg_cb(RMT_STORAGE_REGISTER_CB_PROC,
			RMT_STORAGE_EVNT_CLOSE,
			rmt_storage_event_close_cb);

	if (ret)
		goto unregister_client;

	ret = rmt_storage_reg_cb(RMT_STORAGE_REGISTER_CB_PROC,
			RMT_STORAGE_EVNT_WRITE_BLOCK,
			rmt_storage_event_write_block_cb);

	if (ret)
		goto unregister_client;

	ret = rmt_storage_reg_cb(RMT_STORAGE_REGISTER_CB_PROC,
			RMT_STORAGE_EVNT_GET_DEV_ERROR,
			rmt_storage_event_get_err_cb);

	if (ret)
		goto unregister_client;

	ret = rmt_storage_reg_cb(RMT_STORAGE_REGISTER_WRITE_IOVEC_PROC,
			RMT_STORAGE_EVNT_WRITE_IOVEC,
			rmt_storage_event_write_iovec_cb);

	if (ret)
		goto unregister_client;

	ret = rmt_storage_reg_cb(RMT_STORAGE_REGISTER_CB_PROC,
			RMT_STORAGE_EVNT_SEND_USER_DATA,
			rmt_storage_event_user_data_cb);

	if (ret)
		goto unregister_client;

	ret = misc_register(&rmt_storage_device);
	if (ret) {
		pr_err("%s: Unable to register misc device %d\n", __func__,
				MISC_DYNAMIC_MINOR);
		goto unregister_client;
	}
#ifdef CONFIG_MSM_RMT_STORAGE_CLIENT_STATS
	stats_dentry = debugfs_create_file("rmt_storage_stats", 0444, 0,
					NULL, &debug_ops);
	if (!stats_dentry)
		pr_info("%s: Failed to create stats debugfs file\n", __func__);
#endif
	ret = sysfs_create_group(&pdev->dev.kobj, &dev_attr_grp);
	if (ret)
		pr_info("%s: Failed to create sysfs node: %d\n", __func__, ret);
	goto out;

unregister_client:
	msm_rpc_unregister_client(client);
destroy_wlock:
	wake_lock_destroy(&rmc->wlock);
rmc_free:
	kfree(rmc);
out:
	return ret;
}

static void rmt_storage_client_shutdown(struct platform_device *pdev)
{
	_rmc->smem_info->client_sts = 0;
}

static struct platform_driver rmt_storage_driver = {
	.probe	= rmt_storage_probe,
	.shutdown = rmt_storage_client_shutdown,
	.driver	= {
		.name 	= "rs00000000",
		.owner	= THIS_MODULE,
	},
};

static int __init rmt_storage_init(void)
{
	snprintf((char *)rmt_storage_driver.driver.name,
			strlen(rmt_storage_driver.driver.name)+1,
			"rs%.8x", RMT_STORAGE_APIPROG);
	return platform_driver_register(&rmt_storage_driver);
}

module_init(rmt_storage_init);
MODULE_DESCRIPTION("Remote Storage RPC Client");
MODULE_LICENSE("GPL v2");
