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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <mach/peripheral-loader.h>
#include <mach/msm_smd.h>
#include <mach/qdsp6v2/apr.h>

#include "apr_tal.h"
#include "dsp_debug.h"

struct apr_q6 q6;
struct apr_client client[APR_DEST_MAX][APR_CLIENT_MAX];

inline int apr_fill_hdr(void *handle, uint32_t *buf, uint16_t src_port,
			uint16_t msg_type, uint16_t dest_port,
			uint32_t token, uint32_t opcode, uint16_t h_len)
{
	struct apr_svc *svc = handle;
	struct apr_client *clnt;
	struct apr_hdr *hdr;
	uint16_t dest_id;
	uint16_t client_id;
	uint16_t type;
	uint16_t hdr_len;
	uint16_t ver;

	if (!handle || !buf || h_len < APR_HDR_SIZE) {
		pr_err("APR: Wrong parameters\n");
		return -EINVAL;
	}
	dest_id = svc->dest_id;
	client_id = svc->client_id;
	clnt = &client[dest_id][client_id];

	if (!client[dest_id][client_id].handle) {
		pr_err("APR: Still service is not yet opened\n");
		return -EINVAL;
	}

	hdr = (struct apr_hdr *)buf;
	hdr_len = h_len >> 2;
	hdr->pkt_size = h_len;
	hdr->src_domain = APR_DOMAIN_APPS;
	hdr->src_svc = svc->id;
	hdr->dest_svc = svc->id;
	if (dest_id == APR_DEST_MODEM)
		hdr->dest_domain = APR_DOMAIN_MODEM;
	else if (dest_id == APR_DEST_QDSP6)
		hdr->dest_domain = APR_DOMAIN_ADSP;

	hdr->src_port = src_port;
	hdr->dest_port = dest_port;
	hdr->token = token;
	hdr->opcode = opcode;
	ver = APR_PKT_VER;
	type = msg_type;
	hdr->hdr_field = ((msg_type & 0x0003) << 0x8) |
			((hdr_len & 0x000F) << 0x4) | (ver & 0x000F);
	return 0;
}

int apr_send_pkt(void *handle, uint32_t *buf)
{
	struct apr_svc *svc = handle;
	struct apr_client *clnt;
	struct apr_hdr *hdr;
	uint16_t dest_id;
	uint16_t client_id;
	uint16_t w_len;
	unsigned long flags;

	if (!handle || !buf) {
		pr_err("APR: Wrong parameters\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&svc->w_lock, flags);
	dest_id = svc->dest_id;
	client_id = svc->client_id;
	clnt = &client[dest_id][client_id];

	if (!client[dest_id][client_id].handle) {
		pr_err("APR: Still service is not yet opened\n");
		spin_unlock_irqrestore(&svc->w_lock, flags);
		return -EINVAL;
	}
	hdr = (struct apr_hdr *)buf;

	hdr->src_domain = APR_DOMAIN_APPS;
	hdr->src_svc = svc->id;
	if (dest_id == APR_DEST_MODEM)
		hdr->dest_domain = APR_DOMAIN_MODEM;
	else if (dest_id == APR_DEST_QDSP6)
		hdr->dest_domain = APR_DOMAIN_ADSP;

	hdr->dest_svc = svc->id;

	w_len = apr_tal_write(clnt->handle, buf, hdr->pkt_size);
	if (w_len != hdr->pkt_size)
		pr_err("Unable to write APR pkt successfully: %d\n", w_len);
	spin_unlock_irqrestore(&svc->w_lock, flags);

	return w_len;
}

static void apr_cb_func(void *buf, int len, void *priv)
{
	struct apr_client_data data;
	struct apr_client *apr_client;
	struct apr_svc *c_svc;
	struct apr_hdr *hdr;
	uint16_t hdr_size;
	uint16_t msg_type;
	uint16_t ver;
	uint16_t src;
	uint16_t svc;
	uint16_t clnt;
	int i;
	int temp_port = 0;
	uint32_t *ptr;

	pr_debug("APR2: len = %d\n", len);
	ptr = buf;
	pr_debug("\n*****************\n");
	for (i = 0; i < len/4; i++)
		pr_debug("%x  ", ptr[i]);
	pr_debug("\n");
	pr_debug("\n*****************\n");

	if (!buf || len <= APR_HDR_SIZE) {
		pr_err("APR: Improper apr pkt received:%p %d\n",
								buf, len);
		return;
	}
	hdr = buf;

	ver = hdr->hdr_field;
	ver = (ver & 0x000F);
	if (ver > APR_PKT_VER + 1) {
		pr_err("APR: Wrong version: %d\n", ver);
		return;
	}

	hdr_size = hdr->hdr_field;
	hdr_size = ((hdr_size & 0x00F0) >> 0x4) * 4;
	if (hdr_size < APR_HDR_SIZE) {
		pr_err("APR: Wrong hdr size:%d\n", hdr_size);
		return;
	}

	if (hdr->pkt_size < APR_HDR_SIZE) {
		pr_err("APR: Wrong paket size\n");
		return;
	}
	msg_type = hdr->hdr_field;
	msg_type = (msg_type >> 0x08) & 0x0003;
	if (msg_type >= APR_MSG_TYPE_MAX &&
			msg_type != APR_BASIC_RSP_RESULT) {
		pr_err("APR: Wrong message type: %d\n", msg_type);
		return;
	}

	if (hdr->src_domain >= APR_DOMAIN_MAX ||
		hdr->dest_domain >= APR_DOMAIN_MAX ||
		hdr->src_svc >= APR_SVC_MAX ||
		hdr->dest_svc >= APR_SVC_MAX) {
		pr_err("APR: Wrong APR header\n");
		return;
	}

	svc = hdr->dest_svc;
	if (hdr->src_domain == APR_DOMAIN_MODEM) {
		src = APR_DEST_MODEM;
		if (svc == APR_SVC_MVS || svc == APR_SVC_MVM ||
			svc == APR_SVC_CVS || svc == APR_SVC_CVP ||
			svc == APR_SVC_TEST_CLIENT)
			clnt = APR_CLIENT_VOICE;
		else {
			pr_err("APR: Wrong svc :%d\n", svc);
			return;
		}
	} else if (hdr->src_domain == APR_DOMAIN_ADSP) {
		src = APR_DEST_QDSP6;
		if (svc == APR_SVC_AFE || svc == APR_SVC_ASM ||
			svc == APR_SVC_VSM || svc == APR_SVC_VPM ||
			svc == APR_SVC_ADM || svc == APR_SVC_ADSP_CORE ||
			svc == APR_SVC_TEST_CLIENT || svc == APR_SVC_ADSP_MVM ||
			svc == APR_SVC_ADSP_CVS || svc == APR_SVC_ADSP_CVP)
			clnt = APR_CLIENT_AUDIO;
		else {
			pr_err("APR: Wrong svc :%d\n", svc);
			return;
		}
	} else {
		pr_err("APR: Pkt from wrong source: %d\n", hdr->src_domain);
		return;
	}

	pr_debug("src =%d clnt = %d\n", src, clnt);
	apr_client = &client[src][clnt];
	for (i = 0; i < APR_SVC_MAX; i++)
		if (apr_client->svc[i].id == svc) {
			pr_debug("%d\n", apr_client->svc[i].id);
			c_svc = &apr_client->svc[i];
			break;
		}

	if (i == APR_SVC_MAX) {
		pr_err("APR: service is not registered\n");
		return;
	}
	pr_debug("svc_idx = %d\n", i);
	pr_debug("%x %x %x %p %p\n", c_svc->id, c_svc->dest_id,
			c_svc->client_id, c_svc->fn, c_svc->priv);
	data.payload_size = hdr->pkt_size - hdr_size;
	data.opcode = hdr->opcode;
	data.src = src;
	data.src_port = hdr->src_port;
	data.dest_port = hdr->dest_port;
	data.token = hdr->token;
	data.msg_type = msg_type;
	if (data.payload_size > 0)
		data.payload = (char *)hdr + hdr_size;

	temp_port = ((data.src_port >> 8) * 8) + (data.src_port & 0xFF);
	pr_debug("port = %d t_port = %d\n", data.src_port, temp_port);
	if (c_svc->port_cnt && c_svc->port_fn[temp_port])
		c_svc->port_fn[temp_port](&data,  c_svc->port_priv[temp_port]);
	else if (c_svc->fn)
		c_svc->fn(&data, c_svc->priv);
	else
		pr_err("APR: Rxed a packet for NULL callback\n");
}

struct apr_svc *apr_register(char *dest, char *svc_name, apr_fn svc_fn,
					uint32_t src_port, void *priv)
{
	int client_id = 0;
	int svc_idx = 0;
	int svc_id = 0;
	int dest_id = 0;
	int temp_port = 0;
	struct apr_svc *svc = NULL;

	if (!dest || !svc_name || !svc_fn)
		return NULL;

	if (!strcmp(dest, "ADSP"))
		dest_id = APR_DEST_QDSP6;
	else if (!strcmp(dest, "MODEM")) {
		dest_id = APR_DEST_MODEM;
	} else {
		pr_err("APR: wrong destination\n");
		goto done;
	}

	if (!strcmp(svc_name, "AFE")) {
		client_id = APR_CLIENT_AUDIO;
		svc_idx = 0;
		svc_id = APR_SVC_AFE;
	} else if (!strcmp(svc_name, "ASM")) {
		client_id = APR_CLIENT_AUDIO;
		svc_idx = 1;
		svc_id = APR_SVC_ASM;
	} else if (!strcmp(svc_name, "ADM")) {
		client_id = APR_CLIENT_AUDIO;
		svc_idx = 2;
		svc_id = APR_SVC_ADM;
	} else if (!strcmp(svc_name, "CORE")) {
		client_id = APR_CLIENT_AUDIO;
		svc_idx = 3;
		svc_id = APR_SVC_ADSP_CORE;
	} else if (!strcmp(svc_name, "TEST")) {
		if (dest_id == APR_DEST_QDSP6) {
			client_id = APR_CLIENT_AUDIO;
			svc_idx = 4;
		} else {
			client_id = APR_CLIENT_VOICE;
			svc_idx = 7;
		}
		svc_id = APR_SVC_TEST_CLIENT;
	} else if (!strcmp(svc_name, "VSM")) {
		client_id = APR_CLIENT_VOICE;
		svc_idx = 0;
		svc_id = APR_SVC_VSM;
	} else if (!strcmp(svc_name, "VPM")) {
		client_id = APR_CLIENT_VOICE;
		svc_idx = 1;
		svc_id = APR_SVC_VPM;
	} else if (!strcmp(svc_name, "MVS")) {
		client_id = APR_CLIENT_VOICE;
		svc_idx = 2;
		svc_id = APR_SVC_MVS;
	} else if (!strcmp(svc_name, "MVM")) {
		if (dest_id == APR_DEST_MODEM) {
			client_id = APR_CLIENT_VOICE;
			svc_idx = 3;
			svc_id = APR_SVC_MVM;
		} else {
			client_id = APR_CLIENT_AUDIO;
			svc_idx = 5;
			svc_id = APR_SVC_ADSP_MVM;
		}
	} else if (!strcmp(svc_name, "CVS")) {
		if (dest_id == APR_DEST_MODEM) {
			client_id = APR_CLIENT_VOICE;
			svc_idx = 4;
			svc_id = APR_SVC_CVS;
		} else {
			client_id = APR_CLIENT_AUDIO;
			svc_idx = 6;
			svc_id = APR_SVC_ADSP_CVS;
		}
	} else if (!strcmp(svc_name, "CVP")) {
		if (dest_id == APR_DEST_MODEM) {
			client_id = APR_CLIENT_VOICE;
			svc_idx = 5;
			svc_id = APR_SVC_CVP;
		} else {
			client_id = APR_CLIENT_AUDIO;
			svc_idx = 7;
			svc_id = APR_SVC_ADSP_CVP;
		}
	} else if (!strcmp(svc_name, "SRD")) {
		client_id = APR_CLIENT_VOICE;
		svc_idx = 6;
		svc_id = APR_SVC_SRD;
	} else {
		pr_err("APR: Wrong svc name\n");
		goto done;
	}

	pr_debug("svc name = %s c_id = %d dest_id = %d\n",
				svc_name, client_id, dest_id);
	mutex_lock(&q6.lock);
	if (q6.state == APR_Q6_NOIMG) {
		q6.pil = pil_get("q6");
		if (!q6.pil) {
			pr_err("APR: Unable to load q6 image\n");
			mutex_unlock(&q6.lock);
			return svc;
		}
		q6.state = APR_Q6_LOADED;
	}
	mutex_unlock(&q6.lock);
	mutex_lock(&client[dest_id][client_id].m_lock);
	if (!client[dest_id][client_id].handle) {
		client[dest_id][client_id].handle = apr_tal_open(client_id,
				dest_id, APR_DL_SMD, apr_cb_func, NULL);
		if (!client[dest_id][client_id].handle) {
			svc = NULL;
			pr_err("APR: Unable to open handle\n");
			mutex_unlock(&client[dest_id][client_id].m_lock);
			goto done;
		}
	}
	mutex_unlock(&client[dest_id][client_id].m_lock);
	mutex_lock(&client[dest_id][client_id].svc[svc_idx].m_lock);
	client[dest_id][client_id].id = client_id;
	client[dest_id][client_id].svc[svc_idx].priv = priv;
	client[dest_id][client_id].svc[svc_idx].id = svc_id;
	client[dest_id][client_id].svc[svc_idx].dest_id = dest_id;
	client[dest_id][client_id].svc[svc_idx].client_id = client_id;
	svc = &client[dest_id][client_id].svc[svc_idx];
	if (src_port != 0xFFFFFFFF) {
		temp_port = ((src_port >> 8) * 8) + (src_port & 0xFF);
		pr_debug("port = %d t_port = %d\n", src_port, temp_port);
		if (!svc->port_cnt && !svc->svc_cnt)
			client[dest_id][client_id].svc_cnt++;
		svc->port_cnt++;
		svc->port_fn[temp_port] = svc_fn;
		svc->port_priv[temp_port] = priv;
	} else {
		if (!client[dest_id][client_id].svc[svc_idx].fn) {
			if (!svc->port_cnt && !svc->svc_cnt)
				client[dest_id][client_id].svc_cnt++;
			client[dest_id][client_id].svc[svc_idx].fn = svc_fn;
			if (svc->port_cnt)
				svc->svc_cnt++;
		}
	}

	mutex_unlock(&client[dest_id][client_id].svc[svc_idx].m_lock);
done:
	return svc;
}

int apr_deregister(void *handle)
{
	struct apr_svc *svc = handle;
	struct apr_client *clnt;
	uint16_t dest_id;
	uint16_t client_id;

	if (!handle)
		return -EINVAL;

	mutex_lock(&svc->m_lock);
	dest_id = svc->dest_id;
	client_id = svc->client_id;
	clnt = &client[dest_id][client_id];

	if (svc->port_cnt > 0 || svc->svc_cnt > 0) {
		if (svc->port_cnt)
			svc->port_cnt--;
		else if (svc->svc_cnt)
			svc->svc_cnt--;
		if (!svc->port_cnt && !svc->svc_cnt)
			client[dest_id][client_id].svc_cnt--;
	} else if (client[dest_id][client_id].svc_cnt > 0)
		client[dest_id][client_id].svc_cnt--;

	if (!svc->port_cnt && !svc->svc_cnt) {
		svc->priv = NULL;
		svc->id = 0;
		svc->fn = NULL;
		svc->dest_id = 0;
		svc->client_id = 0;
	}
	if (client[dest_id][client_id].handle &&
		!client[dest_id][client_id].svc_cnt) {
		apr_tal_close(client[dest_id][client_id].handle);
		client[dest_id][client_id].handle = NULL;
	}
	mutex_unlock(&svc->m_lock);

	return 0;
}

void change_q6_state(int state)
{
	mutex_lock(&q6.lock);
	q6.state = state;
	mutex_unlock(&q6.lock);
}

void load_q6(void)
{
	static int client_id = APR_CLIENT_AUDIO;
	static int dest_id = APR_DEST_QDSP6;
	static int svc_idx;
	int delay_cnt = 0;

	mutex_lock(&q6.lock);
	if (q6.state == APR_Q6_NOIMG) {
		q6.pil = pil_get("q6");
		if (!q6.pil) {
			pr_info("APR: Unable to load q6 image\n");
			goto q6_unlock;
		}
		q6.state = APR_Q6_LOADING;
	}
	pr_info("Q6 loading done: Waiting for apr_init\n");
	mutex_lock(&client[dest_id][client_id].svc[svc_idx].m_lock);
	do {
		client[dest_id][client_id].handle = apr_tal_open(client_id,
				dest_id, APR_DL_SMD, apr_cb_func, NULL);
		if (!client[dest_id][client_id].handle) {
			if (q6.state == APR_Q6_LOADED) {
				pr_info("APR: Unable to open handle\n");
				goto unlock;
			}
			udelay(5);
			if (delay_cnt++ < 400000)
				continue;
		} else if (q6.state == APR_Q6_LOADING) {
			q6.state = APR_Q6_LOADED;
			pr_info("apr_init done\n");
			msleep(50);
			pr_info("Audio init done\n");
		}
		break;
	} while (1);

	if (delay_cnt >= 400000)
		pr_info("Q6 Init not yet done in 2 secs\n");
unlock:
	mutex_unlock(&client[dest_id][client_id].svc[svc_idx].m_lock);
q6_unlock:
	mutex_unlock(&q6.lock);
}

int adsp_state(int state)
{
	pr_info("dsp state = %d\n", state);
	return 0;
}

static int __init apr_init(void)
{
	int i, j, k;

	for (i = 0; i < APR_DEST_MAX; i++)
		for (j = 0; j < APR_CLIENT_MAX; j++) {
			mutex_init(&client[i][j].m_lock);
			for (k = 0; k < APR_SVC_MAX; k++) {
				mutex_init(&client[i][j].svc[k].m_lock);
				spin_lock_init(&client[i][j].svc[k].w_lock);
			}
		}
	mutex_init(&q6.lock);
	dsp_debug_register(adsp_state);
	return 0;
}
device_initcall(apr_init);
