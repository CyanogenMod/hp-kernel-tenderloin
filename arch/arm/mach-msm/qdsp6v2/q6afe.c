/*  Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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

#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/jiffies.h>
#include <mach/qdsp6v2/apr_audio.h>

struct afe_ctl {
	void *apr;
	atomic_t state;
	wait_queue_head_t wait;
};

static struct afe_ctl this_afe;

#define TIMEOUT_MS 1000

static int32_t afe_callback(struct apr_client_data *data, void *priv)
{
	if (data->payload_size) {
		uint32_t *payload;
		payload = data->payload;
		pr_debug("%s: cmd = 0x%x status = 0x%x\n", __func__,
					payload[0], payload[1]);
		if (data->opcode == APR_BASIC_RSP_RESULT) {
			switch (payload[0]) {
			case AFE_PORT_AUDIO_IF_CONFIG:
			case AFE_PORT_CMD_STOP:
			case AFE_PORT_CMD_START:
			case AFE_PORT_CMD_LOOPBACK:
			case AFE_PORT_CMD_SIDETONE_CTL:
				atomic_set(&this_afe.state, 0);
				wake_up(&this_afe.wait);
				break;
			default:
				pr_err("Unknown cmd 0x%x\n",
						payload[0]);
				break;
			}
		}
	}
	return 0;
}

int afe_open(u16 port_id, union afe_port_config *afe_config, int rate)
{
	struct afe_port_start_command start;
	struct afe_audioif_config_command config;
	int ret = 0;

	if (!afe_config) {
		pr_err("%s: Error, no configuration data\n", __func__);
		ret = -EINVAL;
		return ret;
	}

	pr_debug("%s: %d %d\n", __func__, port_id, rate);

	if (this_afe.apr == NULL) {
		this_afe.apr = apr_register("ADSP", "AFE", afe_callback,
					0xFFFFFFFF, &this_afe);
		pr_info("%s: Register AFE\n", __func__);
		if (this_afe.apr == NULL) {
			pr_err("%s: Unable to register AFE\n", __func__);
			ret = -ENODEV;
			return ret;
		}
	}

	config.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	config.hdr.pkt_size = sizeof(config);
	config.hdr.src_port = 0;
	config.hdr.dest_port = 0;
	config.hdr.token = 0;
	config.hdr.opcode = AFE_PORT_AUDIO_IF_CONFIG;

	if (port_id >= AFE_MAX_PORTS) {

		pr_err("%s: Failed : Invalid Port id = %d\n", __func__,
				port_id);
		ret = -EINVAL;
		goto fail_cmd;
	}

	config.port_id = port_id;
	config.port = *afe_config;

	atomic_set(&this_afe.state, 1);
	ret = apr_send_pkt(this_afe.apr, (uint32_t *) &config);
	if (ret < 0) {
		pr_err("%s: AFE enable for port %d failed\n", __func__,
				port_id);
		ret = -EINVAL;
		goto fail_cmd;
	}

	ret = wait_event_timeout(this_afe.wait,
			(atomic_read(&this_afe.state) == 0),
				msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		ret = -EINVAL;
		goto fail_cmd;
	}

	start.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	start.hdr.pkt_size = sizeof(start);
	start.hdr.src_port = 0;
	start.hdr.dest_port = 0;
	start.hdr.token = 0;
	start.hdr.opcode = AFE_PORT_CMD_START;
	start.port_id = port_id;
	start.gain = 0x2000;
	start.sample_rate = rate;

	atomic_set(&this_afe.state, 1);
	ret = apr_send_pkt(this_afe.apr, (uint32_t *) &start);
	if (ret < 0) {
		pr_err("%s: AFE enable for port %d failed\n", __func__,
				port_id);
		ret = -EINVAL;
		goto fail_cmd;
	}
	ret = wait_event_timeout(this_afe.wait,
			(atomic_read(&this_afe.state) == 0),
				msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		ret = -EINVAL;
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return ret;
}

int afe_loopback(u16 enable, u16 rx_port, u16 tx_port)
{
	struct afe_loopback_command lb_cmd;
	int ret = 0;
	if (this_afe.apr == NULL) {
		pr_err("%s:AFE is not opened\n", __func__);
		ret = -1;
		goto done;
	}
	lb_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(20), APR_PKT_VER);
	lb_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
						sizeof(lb_cmd) - APR_HDR_SIZE);
	lb_cmd.hdr.src_port = 0;
	lb_cmd.hdr.dest_port = 0;
	lb_cmd.hdr.token = 0;
	lb_cmd.hdr.opcode = AFE_PORT_CMD_LOOPBACK;
	lb_cmd.tx_port_id = tx_port;
	lb_cmd.rx_port_id = rx_port;
	lb_cmd.mode = 0xFFFF;
	lb_cmd.enable = (enable ? 1 : 0);
	atomic_set(&this_afe.state, 1);

	ret = apr_send_pkt(this_afe.apr, (uint32_t *) &lb_cmd);
	if (ret < 0) {
		pr_err("%s: AFE loopback failed\n", __func__);
		ret = -EINVAL;
		goto done;
	}
	ret = wait_event_timeout(this_afe.wait,
		(atomic_read(&this_afe.state) == 0),
				msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		ret = -EINVAL;
	}
done:
	return ret;
}

#ifdef CONFIG_DEBUG_FS
static struct dentry *debugfs_afelb;

static int afe_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	pr_info("debug intf %s\n", (char *) file->private_data);
	return 0;
}

static int afe_get_parameters(char *buf, long int *param1, int num_of_par)
{
	char *token;
	int base, cnt;

	token = strsep(&buf, " ");

	for (cnt = 0; cnt < num_of_par; cnt++) {
		if (token != NULL) {
			if ((token[1] == 'x') || (token[1] == 'X'))
				base = 16;
			else
				base = 10;

			if (strict_strtoul(token, base, &param1[cnt]) != 0)
				return -EINVAL;

			token = strsep(&buf, " ");
		} else
			return -EINVAL;
	}
	return 0;
}
#define AFE_LOOPBACK_ON (1)
#define AFE_LOOPBACK_OFF (0)
static ssize_t afe_debug_write(struct file *filp,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	char *lb_str = filp->private_data;
	char lbuf[32];
	int rc;
	unsigned long param[5];

	if (cnt > sizeof(lbuf) - 1)
		return -EINVAL;

	rc = copy_from_user(lbuf, ubuf, cnt);
	if (rc)
		return -EFAULT;

	lbuf[cnt] = '\0';

	if (!strcmp(lb_str, "afe_loopback")) {
		rc = afe_get_parameters(lbuf, param, 3);
		if (!rc) {
			pr_info("%s %lu %lu %lu\n", lb_str, param[0], param[1],
				param[2]);

			if ((param[0] != AFE_LOOPBACK_ON) && (param[0] !=
				AFE_LOOPBACK_OFF)) {
				pr_err("%s: Error, parameter 0 incorrect\n",
					__func__);
				rc = -EINVAL;
				goto afe_error;
			}
			if ((param[1] >= AFE_MAX_PORTS) || (param[2] >=
				AFE_MAX_PORTS)) {
				pr_err("%s: Error, invalid afe port\n",
					__func__);
			}
			if (this_afe.apr == NULL) {
				pr_err("%s: Error, AFE not opened\n", __func__);
				rc = -EINVAL;
			} else {
				rc = afe_loopback(param[0], param[1], param[2]);
			}
		} else {
			pr_err("%s: Error, invalid parameters\n", __func__);
			rc = -EINVAL;
		}
	}
afe_error:
	if (rc == 0)
		rc = cnt;
	else
		pr_err("%s: rc = %d\n", __func__, rc);

	return rc;
}

static const struct file_operations afe_debug_fops = {
	.open = afe_debug_open,
	.write = afe_debug_write
};
#endif
int afe_sidetone(u16 tx_port_id, u16 rx_port_id, u16 enable, uint16_t gain)
{
	struct afe_port_sidetone_command cmd_sidetone;
	int ret = 0;

	pr_info("%s: tx_port_id:%d rx_port_id:%d enable:%d gain:%d\n", __func__,
					tx_port_id, rx_port_id, enable, gain);
	cmd_sidetone.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	cmd_sidetone.hdr.pkt_size = sizeof(cmd_sidetone);
	cmd_sidetone.hdr.src_port = 0;
	cmd_sidetone.hdr.dest_port = 0;
	cmd_sidetone.hdr.token = 0;
	cmd_sidetone.hdr.opcode = AFE_PORT_CMD_SIDETONE_CTL;
	cmd_sidetone.tx_port_id = tx_port_id;
	cmd_sidetone.rx_port_id = rx_port_id;
	cmd_sidetone.gain = gain;
	cmd_sidetone.enable = enable;

	atomic_set(&this_afe.state, 1);
	ret = apr_send_pkt(this_afe.apr, (uint32_t *) &cmd_sidetone);
	if (ret < 0) {
		pr_err("%s: AFE sidetone failed for tx_port:%d rx_port:%d\n",
					__func__, tx_port_id, rx_port_id);
		ret = -EINVAL;
		goto fail_cmd;
	}

	ret = wait_event_timeout(this_afe.wait,
		(atomic_read(&this_afe.state) == 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (ret < 0) {
		pr_err("%s: wait_event timeout\n", __func__);
		ret = -EINVAL;
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return ret;
}

int afe_close(int port_id)
{
	struct afe_port_stop_command stop;
	int ret = 0;

	if (this_afe.apr == NULL) {
		pr_err("AFE is already closed\n");
		ret = -EINVAL;
		goto fail_cmd;
	}
	pr_debug("%s: port_id=%d\n", __func__, port_id);
	stop.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	stop.hdr.pkt_size = sizeof(stop);
	stop.hdr.src_port = 0;
	stop.hdr.dest_port = 0;
	stop.hdr.token = 0;
	stop.hdr.opcode = AFE_PORT_CMD_STOP;
	stop.port_id = port_id;
	stop.reserved = 0;

	atomic_set(&this_afe.state, 1);
	ret = apr_send_pkt(this_afe.apr, (uint32_t *) &stop);

	if (ret < 0) {
		pr_err("%s: AFE close failed\n", __func__);
		ret = -EINVAL;
		goto fail_cmd;
	}

	ret = wait_event_timeout(this_afe.wait,
			(atomic_read(&this_afe.state) == 0),
					msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		ret = -EINVAL;
		goto fail_cmd;
	}
fail_cmd:
	return ret;
}

static int __init afe_init(void)
{
	init_waitqueue_head(&this_afe.wait);
	atomic_set(&this_afe.state, 0);
	this_afe.apr = NULL;
#ifdef CONFIG_DEBUG_FS
	debugfs_afelb = debugfs_create_file("afe_loopback",
	S_IFREG | S_IWUGO, NULL, (void *) "afe_loopback",
	&afe_debug_fops);
#endif
	return 0;
}

static void __exit afe_exit(void)
{
#ifdef CONFIG_DEBUG_FS
	if (debugfs_afelb)
		debugfs_remove(debugfs_afelb);
#endif
}

device_initcall(afe_init);
__exitcall(afe_exit);
