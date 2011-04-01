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
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <mach/msm_smd.h>
#include <mach/qdsp6v2/apr.h>

struct dentry *dentry;
struct apr_svc_ch_dev *handle;
struct apr_svc *apr_handle_q;
struct apr_svc *apr_handle_m;
struct apr_client_data clnt_data;
char l_buf[4096];

static char *svc_names[] = {
					"NULL",
					"NULL",
					"TEST",
					"CORE",
					"AFE",
					"VSM",
					"VPM",
					"ASM",
					"ADM",
				};

static int32_t aprv2_core_fn_q(struct apr_client_data *data, void *priv)
{
	struct adsp_get_version *payload;
	uint32_t *payload1;
	struct adsp_service_info *svc_info;
	int i;

	pr_info("core msg: payload len = %d\n", data->payload_size);
	switch (data->opcode) {
	case APR_BASIC_RSP_RESULT:{
		payload1 = data->payload;
		if (payload1[0] == ADSP_CMD_SET_POWER_COLLAPSE_STATE) {
			pr_info("Cmd[0x%x] status[0x%x]\n", payload1[0],
							payload1[1]);
			break;
		} else
			pr_err("Invalid cmd rsp[0x%x][0x%x]\n", payload1[0],
								payload1[1]);
		break;
	}
	case ADSP_GET_VERSION_RSP:{
		if (data->payload_size) {
			payload = data->payload;
			svc_info = (struct adsp_service_info *)
			((char *)payload + sizeof(struct adsp_get_version));
			pr_info("----------------------------------------\n");
			pr_info("Build id          = %x\n", payload->build_id);
			pr_info("Number of services= %x\n", payload->svc_cnt);
			pr_info("----------------------------------------\n");
			for (i = 0; i < payload->svc_cnt; i++)
				pr_info("%s\t%x.%x\n",
				svc_names[svc_info[i].svc_id],
				(svc_info[i].svc_ver & 0xFFFF0000) >> 16,
				(svc_info[i].svc_ver & 0xFFFF));
			pr_info("-----------------------------------------\n");
		} else
			pr_info("zero payload for ADSP_GET_VERSION_RSP\n");
		break;
	}
	default:
		pr_err("Message id from adsp core svc: %d\n", data->opcode);
		break;
	}

	return 0;
}

static int32_t aprv2_debug_fn_q(struct apr_client_data *data, void *priv)
{
	pr_debug("Q6_Payload Length = %d\n", data->payload_size);
	if (memcmp(data->payload, l_buf + 20, data->payload_size))
		pr_info("FAIL: %d\n", data->payload_size);
	else
		pr_info("SUCCESS: %d\n", data->payload_size);
	return 0;
}

static int32_t aprv2_debug_fn_m(struct apr_client_data *data, void *priv)
{
	pr_info("M_Payload Length = %d\n", data->payload_size);
	return 0;
}

static ssize_t apr_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	pr_debug("apr debugfs opened\n");
	return 0;
}

static ssize_t apr_debug_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	int len;
	static int t_len;

	if (count < 0)
		return 0;
	len = count > 63 ? 63 : count;
	if (copy_from_user(l_buf + 20 , buf, len)) {
		pr_info("Unable to copy data from user space\n");
		return -EFAULT;
	}
	l_buf[len + 20] = 0;
	if (l_buf[len + 20 - 1] == '\n') {
		l_buf[len + 20 - 1] = 0;
		len--;
	}
	if (!strncmp(l_buf + 20, "open_q", 64)) {
		apr_handle_q = apr_register("ADSP", "TEST", aprv2_debug_fn_q,
							0xFFFFFFFF, NULL);
		pr_info("Open_q %p\n", apr_handle_q);
	} else if (!strncmp(l_buf + 20, "open_m", 64)) {
		apr_handle_m = apr_register("MODEM", "TEST", aprv2_debug_fn_m,
							0xFFFFFFFF, NULL);
		pr_info("Open_m %p\n", apr_handle_m);
	} else if (!strncmp(l_buf + 20, "write_q", 64)) {
		struct apr_hdr *hdr;

		t_len++;
		t_len = t_len % 450;
		if (!t_len % 99)
			msleep(2000);
		hdr = (struct apr_hdr *)l_buf;
		hdr->hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_EVENT,
					APR_HDR_LEN(20), APR_PKT_VER);
		hdr->pkt_size = APR_PKT_SIZE(20, t_len);
		hdr->src_port = 0;
		hdr->dest_port = 0;
		hdr->token = 0;
		hdr->opcode = 0x12345678;
		memset(l_buf + 20, 9, 4060);

		apr_send_pkt(apr_handle_q, (uint32_t *)l_buf);
		pr_debug("Write_q\n");
	} else if (!strncmp(l_buf + 20, "write_m", 64)) {
		struct apr_hdr *hdr;

		hdr = (struct apr_hdr *)l_buf;
		hdr->hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_EVENT,
					APR_HDR_LEN(20), APR_PKT_VER);
		hdr->pkt_size = APR_PKT_SIZE(20, 8);
		hdr->src_port = 0;
		hdr->dest_port = 0;
		hdr->token = 0;
		hdr->opcode = 0x12345678;
		memset(l_buf + 30, 9, 4060);

		apr_send_pkt(apr_handle_m, (uint32_t *)l_buf);
		pr_info("Write_m\n");
	} else if (!strncmp(l_buf + 20, "write_q4", 64)) {
		struct apr_hdr *hdr;

		hdr = (struct apr_hdr *)l_buf;
		hdr->hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_EVENT,
					APR_HDR_LEN(20), APR_PKT_VER);
		hdr->pkt_size = APR_PKT_SIZE(20, 4076);
		hdr->src_port = 0;
		hdr->dest_port = 0;
		hdr->token = 0;
		hdr->opcode = 0x12345678;
		memset(l_buf + 30, 9, 4060);

		apr_send_pkt(apr_handle_q, (uint32_t *)l_buf);
		pr_info("Write_q\n");
	} else if (!strncmp(l_buf + 20, "write_m4", 64)) {
		struct apr_hdr *hdr;

		hdr = (struct apr_hdr *)l_buf;
		hdr->hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_EVENT,
					APR_HDR_LEN(20), APR_PKT_VER);
		hdr->pkt_size = APR_PKT_SIZE(20, 4076);
		hdr->src_port = 0;
		hdr->dest_port = 0;
		hdr->token = 0;
		hdr->opcode = 0x12345678;
		memset(l_buf + 30, 9, 4060);

		apr_send_pkt(apr_handle_m, (uint32_t *)l_buf);
		pr_info("Write_m\n");
	} else if (!strncmp(l_buf + 20, "close", 64)) {
		if (apr_handle_q)
			apr_deregister(apr_handle_q);
	} else if (!strncmp(l_buf + 20, "loaded", 64)) {
		change_q6_state(APR_Q6_LOADED);
	} else if (!strncmp(l_buf + 20, "boom", 64)) {
		q6audio_dsp_not_responding();
	} else if (!strncmp(l_buf + 20, "dsp_ver", 64)) {
		struct apr_hdr *hdr;

		apr_handle_q = apr_register("ADSP", "CORE", aprv2_core_fn_q,
							0xFFFFFFFF, NULL);
		pr_info("Open_q %p\n", apr_handle_q);
		if (apr_handle_q) {
			hdr = (struct apr_hdr *)l_buf;
			hdr->hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_EVENT,
					APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
			hdr->pkt_size = APR_PKT_SIZE(APR_HDR_SIZE, 0);
			hdr->src_port = 0;
			hdr->dest_port = 0;
			hdr->token = 0;
			hdr->opcode = ADSP_GET_VERSION;

			apr_send_pkt(apr_handle_q, (uint32_t *)l_buf);
			pr_info("Write_q\n");
		} else
			pr_info("apr registration failed\n");
	} else if (!strncmp(l_buf + 20, "en_pwr_col", 64)) {
		struct adsp_power_collapse pc;

		apr_handle_q = apr_register("ADSP", "CORE", aprv2_core_fn_q,
							0xFFFFFFFF, NULL);
		pr_info("Open_q %p\n", apr_handle_q);
		if (apr_handle_q) {
			pc.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_EVENT,
					APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
			pc.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
						sizeof(uint32_t));;
			pc.hdr.src_port = 0;
			pc.hdr.dest_port = 0;
			pc.hdr.token = 0;
			pc.hdr.opcode = ADSP_CMD_SET_POWER_COLLAPSE_STATE;
			pc.power_collapse = 0x00000000;
			apr_send_pkt(apr_handle_q, (uint32_t *)&pc);
			pr_info("Write_q :enable power collapse\n");
		}
	} else if (!strncmp(l_buf + 20, "dis_pwr_col", 64)) {
		struct adsp_power_collapse pc;

		apr_handle_q = apr_register("ADSP", "CORE", aprv2_core_fn_q,
							0xFFFFFFFF, NULL);
		pr_info("Open_q %p\n", apr_handle_q);
		if (apr_handle_q) {
			pc.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_EVENT,
					APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
			pc.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
							sizeof(uint32_t));
			pc.hdr.src_port = 0;
			pc.hdr.dest_port = 0;
			pc.hdr.token = 0;
			pc.hdr.opcode = ADSP_CMD_SET_POWER_COLLAPSE_STATE;
			pc.power_collapse = 0x00000001;
			apr_send_pkt(apr_handle_q, (uint32_t *)&pc);
			pr_info("Write_q:disable power collapse\n");
		}
	} else
		pr_info("Unknown Command\n");

	return count;
}

static const struct file_operations apr_debug_fops = {
	.write = apr_debug_write,
	.open = apr_debug_open,
};

static int __init apr_init(void)
{
#ifdef CONFIG_DEBUG_FS
	dentry = debugfs_create_file("apr", S_IFREG | S_IWUGO,
				NULL, (void *) NULL, &apr_debug_fops);
#endif /* CONFIG_DEBUG_FS */
	return 0;
}
device_initcall(apr_init);

