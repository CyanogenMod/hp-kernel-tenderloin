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
 */

#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/jiffies.h>
#include <asm/atomic.h>
#include <mach/qdsp6v2/apr_audio.h>
#include "audio_acdb.h"

#define TIMEOUT_MS 1000
#define AUDIO_RX 0x0
#define AUDIO_TX 0x1

struct adm_ctl {
	void *apr;
	atomic_t copp_id[AFE_MAX_PORTS];
	atomic_t copp_cnt[AFE_MAX_PORTS];
	atomic_t copp_stat[AFE_MAX_PORTS];
	wait_queue_head_t wait;
};

static struct adm_ctl			this_adm;

static int32_t adm_callback(struct apr_client_data *data, void *priv)
{
	uint32_t *payload;
	payload = data->payload;
	pr_debug("%s: code = 0x%x %x %x size = %d\n", __func__,
			data->opcode, payload[0], payload[1],
					data->payload_size);

	if (data->payload_size) {
		if (data->opcode == APR_BASIC_RSP_RESULT) {
			switch (payload[0]) {
			case ADM_CMD_COPP_CLOSE:
			case ADM_CMD_MEMORY_MAP:
			case ADM_CMD_MEMORY_UNMAP:
			case ADM_CMD_MEMORY_MAP_REGIONS:
			case ADM_CMD_MEMORY_UNMAP_REGIONS:
			case ADM_CMD_MATRIX_MAP_ROUTINGS:
			case ADM_CMD_SET_PARAMS:
				atomic_set(&this_adm.copp_stat[data->token],
									1);
				wake_up(&this_adm.wait);
				break;
			default:
				pr_err("%s: Unknown Cmd: 0x%x\n", __func__,
								payload[0]);
				break;
			}
			return 0;
		}

		switch (data->opcode) {
		case ADM_CMDRSP_COPP_OPEN: {
			struct adm_copp_open_respond *open = data->payload;
			atomic_set(&this_adm.copp_id[data->token],
							open->copp_id);
			atomic_set(&this_adm.copp_stat[data->token], 1);
			pr_debug("%s: coppid rxed=%d\n", __func__,
							open->copp_id);
			wake_up(&this_adm.wait);
			}
			break;
		default:
			pr_err("%s: Unknown cmd:0x%x\n", __func__,
							data->opcode);
			break;
		}
	}
	return 0;
}

void send_cal(int port_id, struct acdb_cal_block *aud_cal)
{
	s32				result;
	struct adm_set_params_command	adm_params;
	pr_debug("%s: Port id %d\n", __func__, port_id);

	if (!aud_cal || aud_cal->cal_size == 0) {
		pr_debug("%s: No calibration data to send!\n", __func__);
		goto done;
	}

	adm_params.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
		APR_HDR_LEN(20), APR_PKT_VER);
	adm_params.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
		sizeof(adm_params));
	adm_params.hdr.src_svc = APR_SVC_ADM;
	adm_params.hdr.src_domain = APR_DOMAIN_APPS;
	adm_params.hdr.src_port = port_id;
	adm_params.hdr.dest_svc = APR_SVC_ADM;
	adm_params.hdr.dest_domain = APR_DOMAIN_ADSP;
	adm_params.hdr.dest_port = atomic_read(&this_adm.copp_id[port_id]);
	adm_params.hdr.token = port_id;
	adm_params.hdr.opcode = ADM_CMD_SET_PARAMS;
	adm_params.payload = aud_cal->cal_paddr;
	adm_params.payload_size = aud_cal->cal_size;

	atomic_set(&this_adm.copp_stat[port_id], 0);
	pr_debug("%s: Sending SET_PARAMS payload = 0x%x, size = %d\n",
		__func__, adm_params.payload, adm_params.payload_size);
	result = apr_send_pkt(this_adm.apr, (uint32_t *)&adm_params);
	if (result < 0) {
		pr_err("%s: Set params failed port = %d payload = 0x%x\n",
			__func__, port_id, aud_cal->cal_paddr);
		goto done;
	}
	/* Wait for the callback */
	result = wait_event_timeout(this_adm.wait,
		atomic_read(&this_adm.copp_stat[port_id]),
		msecs_to_jiffies(TIMEOUT_MS));
	if (!result)
		pr_err("%s: Set params timed out port = %d, payload = 0x%x\n",
			__func__, port_id, aud_cal->cal_paddr);
done:
	return;
}

void send_adm_cal(int port_id, int path)
{
	s32			acdb_path;
	struct acdb_cal_block	aud_cal;

	pr_debug("%s\n", __func__);

	/* Maps audio_dev_ctrl path definition to ACDB definition */
	acdb_path = path - 1;
	if ((acdb_path >= NUM_AUDPROC_BUFFERS) ||
		(acdb_path < 0)) {
		pr_err("%s: Path is not RX or TX, path = %d\n",
			__func__, path);
		goto done;
	}

	pr_debug("%s: Sending audproc cal\n", __func__);
	get_audproc_cal(acdb_path, &aud_cal);
	send_cal(port_id, &aud_cal);

	pr_debug("%s: Sending audvol cal\n", __func__);
	get_audvol_cal(acdb_path, &aud_cal);
	send_cal(port_id, &aud_cal);
done:
	return;
}

int adm_open(int port_id, int session_id , int path,
				int rate, int channel_mode,
				int topology)
{
	struct adm_copp_open_command	open;
	struct adm_routings_command	route;
	int ret = 0;

	pr_debug("%s: port %d session 0x%x path:%d rate:%d mode:%d\n", __func__,
				port_id, session_id, path, rate, channel_mode);

	if (port_id >= AFE_MAX_PORTS) {
		pr_err("%s port idi[%d] out of limit[%d]\n", __func__,
						port_id, AFE_MAX_PORTS);
		return -ENODEV;
	}

	if (this_adm.apr == NULL) {
		this_adm.apr = apr_register("ADSP", "ADM", adm_callback,
						0xFFFFFFFF, &this_adm);
		if (this_adm.apr == NULL) {
			pr_err("%s: Unable to register ADM\n", __func__);
			ret = -ENODEV;
			return ret;
		}
	}


	/* Create a COPP if port id are not enabled */
	if (atomic_read(&this_adm.copp_cnt[port_id]) == 0) {

		open.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
		open.hdr.pkt_size = sizeof(open);
		open.hdr.src_svc = APR_SVC_ADM;
		open.hdr.src_domain = APR_DOMAIN_APPS;
		open.hdr.src_port = port_id;
		open.hdr.dest_svc = APR_SVC_ADM;
		open.hdr.dest_domain = APR_DOMAIN_ADSP;
		open.hdr.dest_port = port_id;
		open.hdr.token = port_id;
		open.hdr.opcode = ADM_CMD_COPP_OPEN;

		open.mode = path;
		open.endpoint_id1 = port_id & 0x00FF;
		open.endpoint_id2 = 0xFFFF;
		open.topology_id  = topology;

		open.channel_config = channel_mode & 0x00FF;
		open.rate  = rate;

		pr_debug("%s: channel_config=%d port_id=%d rate=%d\
			topology_id=0x%X\n", __func__, open.channel_config,\
			open.endpoint_id1, open.rate,\
			open.topology_id);

		atomic_set(&this_adm.copp_stat[port_id], 0);

		ret = apr_send_pkt(this_adm.apr, (uint32_t *)&open);
		if (ret < 0) {
			pr_err("%s:ADM enable for port %d failed\n",
						__func__, port_id);
			ret = -EINVAL;
			goto fail_cmd;
		}
		/* Wait for the callback with copp id */
		ret = wait_event_timeout(this_adm.wait,
			atomic_read(&this_adm.copp_stat[port_id]),
			msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			pr_err("%s ADM open failed for port %d\n", __func__,
								port_id);
			ret = -EINVAL;
			goto fail_cmd;
		}
	}
	atomic_inc(&this_adm.copp_cnt[port_id]);
	route.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	route.hdr.pkt_size = sizeof(route);
	route.hdr.src_svc = 0;
	route.hdr.src_domain = APR_DOMAIN_APPS;
	route.hdr.src_port = port_id;
	route.hdr.dest_svc = APR_SVC_ADM;
	route.hdr.dest_domain = APR_DOMAIN_ADSP;
	route.hdr.dest_port = atomic_read(&this_adm.copp_id[port_id]);
	route.hdr.token = port_id;
	route.hdr.opcode = ADM_CMD_MATRIX_MAP_ROUTINGS;
	route.num_sessions = 1;
	route.sessions[0].id = session_id;
	route.sessions[0].num_copps = 1;
	route.sessions[0].copp_id[0] =
			atomic_read(&this_adm.copp_id[port_id]);

	switch (path) {
	case 0x1:
		route.path = AUDIO_RX;
		break;
	case 0x2:
	case 0x3:
		route.path = AUDIO_TX;
		break;
	default:
		pr_err("%s: Wrong path set[%d]\n", __func__, path);
		break;
	}
	atomic_set(&this_adm.copp_stat[port_id], 0);

	ret = apr_send_pkt(this_adm.apr, (uint32_t *)&route);
	if (ret < 0) {
		pr_err("%s: ADM routing for port %d failed\n",
					__func__, port_id);
		ret = -EINVAL;
		goto fail_cmd;
	}
	ret = wait_event_timeout(this_adm.wait,
				atomic_read(&this_adm.copp_stat[port_id]),
				msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: ADM cmd Route failed for port %d\n",
					__func__, port_id);
		ret = -EINVAL;
		goto fail_cmd;
	}
	send_adm_cal(port_id, path);
	return 0;

fail_cmd:

	return ret;
}

int adm_memory_map_regions(uint32_t *buf_add, uint32_t mempool_id,
				uint32_t *bufsz, uint32_t bufcnt)
{
	struct  adm_cmd_memory_map_regions *mmap_regions = NULL;
	struct  adm_memory_map_regions *mregions = NULL;
	void    *mmap_region_cmd = NULL;
	void    *payload = NULL;
	int     ret = 0;
	int     i = 0;
	int     cmd_size = 0;

	pr_info("%s\n", __func__);
	if (this_adm.apr == NULL) {
		this_adm.apr = apr_register("ADSP", "ADM", adm_callback,
						0xFFFFFFFF, &this_adm);
		if (this_adm.apr == NULL) {
			pr_err("%s: Unable to register ADM\n", __func__);
			ret = -ENODEV;
			return ret;
		}
	}

	cmd_size = sizeof(struct adm_cmd_memory_map_regions)
			+ sizeof(struct adm_memory_map_regions) * bufcnt;

	mmap_region_cmd = kzalloc(cmd_size, GFP_KERNEL);
	mmap_regions = (struct adm_cmd_memory_map_regions *)mmap_region_cmd;
	mmap_regions->hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
								APR_PKT_VER);
	mmap_regions->hdr.pkt_size = cmd_size;
	mmap_regions->hdr.src_port = 0;
	mmap_regions->hdr.dest_port = 0;
	mmap_regions->hdr.token = 0;
	mmap_regions->hdr.opcode = ADM_CMD_MEMORY_MAP_REGIONS;
	mmap_regions->mempool_id = mempool_id & 0x00ff;
	mmap_regions->nregions = bufcnt & 0x00ff;
	pr_debug("%s: map_regions->nregions = %d\n", __func__,
				mmap_regions->nregions);
	payload = ((u8 *) mmap_region_cmd +
				sizeof(struct adm_cmd_memory_map_regions));
	mregions = (struct adm_memory_map_regions *)payload;

	for (i = 0; i < bufcnt; i++) {
		mregions->phys = buf_add[i];
		mregions->buf_size = bufsz[i];
		++mregions;
	}

	atomic_set(&this_adm.copp_stat[0], 0);
	ret = apr_send_pkt(this_adm.apr, (uint32_t *) mmap_region_cmd);
	if (ret < 0) {
		pr_err("%s: mmap_regions op[0x%x]rc[%d]\n", __func__,
					mmap_regions->hdr.opcode, ret);
		ret = -EINVAL;
		goto fail_cmd;
	}

	ret = wait_event_timeout(this_adm.wait,
			atomic_read(&this_adm.copp_stat[0]), 5 * HZ);
	if (!ret) {
		pr_err("%s: timeout. waited for memory_map\n", __func__);
		ret = -EINVAL;
		goto fail_cmd;
	}
fail_cmd:
	kfree(mmap_region_cmd);
	return ret;
}

int adm_memory_unmap_regions(uint32_t *buf_add, uint32_t *bufsz,
						uint32_t bufcnt)
{
	struct  adm_cmd_memory_unmap_regions *unmap_regions = NULL;
	struct  adm_memory_unmap_regions *mregions = NULL;
	void    *unmap_region_cmd = NULL;
	void    *payload = NULL;
	int     ret = 0;
	int     i = 0;
	int     cmd_size = 0;

	pr_info("%s\n", __func__);

	if (this_adm.apr == NULL) {
		pr_err("%s APR handle NULL\n", __func__);
		return -EINVAL;
	}

	cmd_size = sizeof(struct adm_cmd_memory_unmap_regions)
			+ sizeof(struct adm_memory_unmap_regions) * bufcnt;

	unmap_region_cmd = kzalloc(cmd_size, GFP_KERNEL);
	unmap_regions = (struct adm_cmd_memory_unmap_regions *)
						unmap_region_cmd;
	unmap_regions->hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
							APR_PKT_VER);
	unmap_regions->hdr.pkt_size = cmd_size;
	unmap_regions->hdr.src_port = 0;
	unmap_regions->hdr.dest_port = 0;
	unmap_regions->hdr.token = 0;
	unmap_regions->hdr.opcode = ADM_CMD_MEMORY_UNMAP_REGIONS;
	unmap_regions->nregions = bufcnt & 0x00ff;
	unmap_regions->reserved = 0;
	pr_debug("%s: unmap_regions->nregions = %d\n", __func__,
				unmap_regions->nregions);
	payload = ((u8 *) unmap_region_cmd +
			sizeof(struct adm_cmd_memory_unmap_regions));
	mregions = (struct adm_memory_unmap_regions *)payload;

	for (i = 0; i < bufcnt; i++) {
		mregions->phys = buf_add[i];
		++mregions;
	}
	atomic_set(&this_adm.copp_stat[0], 0);
	ret = apr_send_pkt(this_adm.apr, (uint32_t *) unmap_region_cmd);
	if (ret < 0) {
		pr_err("%s: mmap_regions op[0x%x]rc[%d]\n", __func__,
				unmap_regions->hdr.opcode, ret);
		ret = -EINVAL;
		goto fail_cmd;
	}

	ret = wait_event_timeout(this_adm.wait,
			atomic_read(&this_adm.copp_stat[0]), 5 * HZ);
	if (!ret) {
		pr_err("%s: timeout. waited for memory_unmap\n", __func__);
		ret = -EINVAL;
		goto fail_cmd;
	}
fail_cmd:
	kfree(unmap_region_cmd);
	return ret;
}

int adm_close(int port_id)
{
	struct apr_hdr close;

	int ret = 0;

	pr_debug("%s port_id=%d\n", __func__, port_id);

	if (!(atomic_read(&this_adm.copp_cnt[port_id]))) {
		pr_err("%s: copp count for port[%d]is 0\n", __func__, port_id);

		goto fail_cmd;
	}
	atomic_dec(&this_adm.copp_cnt[port_id]);
	if (!(atomic_read(&this_adm.copp_cnt[port_id]))) {

		close.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
			APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
		close.pkt_size = sizeof(close);
		close.src_svc = APR_SVC_ADM;
		close.src_domain = APR_DOMAIN_APPS;
		close.src_port = port_id;
		close.dest_svc = APR_SVC_ADM;
		close.dest_domain = APR_DOMAIN_ADSP;
		close.dest_port = atomic_read(&this_adm.copp_id[port_id]);
		close.token = port_id;
		close.opcode = ADM_CMD_COPP_CLOSE;

		atomic_set(&this_adm.copp_id[port_id], 0);
		atomic_set(&this_adm.copp_stat[port_id], 0);


		pr_debug("%s:coppid %d portid=%d coppcnt=%d\n",
				__func__,
				atomic_read(&this_adm.copp_id[port_id]),
				port_id,
				atomic_read(&this_adm.copp_cnt[port_id]));

		ret = apr_send_pkt(this_adm.apr, (uint32_t *)&close);
		if (ret < 0) {
			pr_err("%s ADM close failed\n", __func__);
			ret = -EINVAL;
			goto fail_cmd;
		}

		ret = wait_event_timeout(this_adm.wait,
				atomic_read(&this_adm.copp_stat[port_id]),
				msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			pr_err("%s: ADM cmd Route failed for port %d\n",
						__func__, port_id);
			ret = -EINVAL;
			goto fail_cmd;
		}
	}

fail_cmd:
	return ret;
}

static int __init adm_init(void)
{
	int i = 0;
	init_waitqueue_head(&this_adm.wait);
	this_adm.apr = NULL;

	for (i = 0; i < AFE_MAX_PORTS; i++) {
		atomic_set(&this_adm.copp_id[i], 0);
		atomic_set(&this_adm.copp_cnt[i], 0);
		atomic_set(&this_adm.copp_stat[i], 0);
	}
	return 0;
}

device_initcall(adm_init);
