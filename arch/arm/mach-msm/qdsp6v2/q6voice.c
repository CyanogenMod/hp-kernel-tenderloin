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
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/msm_audio.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <mach/qdsp6v2/audio_dev_ctl.h>
#include <mach/dal.h>
#include <mach/qdsp6v2/q6voice.h>
#include "audio_acdb.h"
#include <linux/msm_audio_mvs.h>

#define TIMEOUT_MS 3000
#define SNDDEV_CAP_TTY 0x20

#define CMD_STATUS_SUCCESS 0
#define CMD_STATUS_FAIL 1

#define VOC_PATH_PASSIVE 0
#define VOC_PATH_FULL 1

#define BUFFER_PAYLOAD_SIZE 4000

struct mvs_driver_info {
	uint32_t media_type;
	uint32_t rate;
	uint32_t network_type;
	ul_cb_fn ul_cb;
	dl_cb_fn dl_cb;
	void *private_data;
};

struct voice_data {
	int voc_state;/*INIT, CHANGE, RELEASE, RUN */
	uint32_t voc_path;

	wait_queue_head_t mvm_wait;
	wait_queue_head_t cvs_wait;
	wait_queue_head_t cvp_wait;

	uint32_t device_events;

	/* cache the values related to Rx and Tx */
	struct device_data dev_rx;
	struct device_data dev_tx;

	/* these default values are for all devices */
	uint32_t default_mute_val;
	uint32_t default_vol_val;
	uint32_t default_sample_val;

	/* call status */
	int v_call_status; /* Start or End */

	/* APR to MVM in the modem */
	void *apr_mvm;
	/* APR to CVS in the modem */
	void *apr_cvs;
	/* APR to CVP in the modem */
	void *apr_cvp;

	/* APR to MVM in the Q6 */
	void *apr_q6_mvm;
	/* APR to CVS in the Q6 */
	void *apr_q6_cvs;
	/* APR to CVP in the Q6 */
	void *apr_q6_cvp;

	u32 mvm_state;
	u32 cvs_state;
	u32 cvp_state;

	/* Handle to MVM in the modem */
	u16 mvm_handle;
	/* Handle to CVS in the modem */
	u16 cvs_handle;
	/* Handle to CVP in the modem */
	u16 cvp_handle;

	/* Handle to MVM in the Q6 */
	u16 mvm_q6_handle;
	/* Handle to CVS in the Q6 */
	u16 cvs_q6_handle;
	/* Handle to CVP in the Q6 */
	u16 cvp_q6_handle;

	struct mutex lock;

	struct mvs_driver_info mvs_info;
};

struct voice_data voice;

static void *voice_get_apr_mvm(struct voice_data *v)
{
	void *apr_mvm = NULL;

	if (v->voc_path == VOC_PATH_PASSIVE)
		apr_mvm = v->apr_mvm;
	else
		apr_mvm = v->apr_q6_mvm;

	pr_debug("%s: apr_mvm 0x%x\n", __func__, (unsigned int)apr_mvm);

	return apr_mvm;
}

static void voice_set_apr_mvm(struct voice_data *v, void *apr_mvm)
{
	pr_debug("%s: apr_mvm 0x%x\n", __func__, (unsigned int)apr_mvm);

	if (v->voc_path == VOC_PATH_PASSIVE)
		v->apr_mvm = apr_mvm;
	else
		v->apr_q6_mvm = apr_mvm;
}

static void *voice_get_apr_cvs(struct voice_data *v)
{
	void *apr_cvs = NULL;

	if (v->voc_path == VOC_PATH_PASSIVE)
		apr_cvs = v->apr_cvs;
	else
		apr_cvs = v->apr_q6_cvs;

	pr_debug("%s: apr_cvs 0x%x\n", __func__, (unsigned int)apr_cvs);

	return apr_cvs;
}

static void voice_set_apr_cvs(struct voice_data *v, void *apr_cvs)
{
	pr_debug("%s: apr_cvs 0x%x\n", __func__, (unsigned int)apr_cvs);

	if (v->voc_path == VOC_PATH_PASSIVE)
		v->apr_cvs = apr_cvs;
	else
		v->apr_q6_cvs = apr_cvs;
}

static void *voice_get_apr_cvp(struct voice_data *v)
{
	void *apr_cvp = NULL;

	if (v->voc_path == VOC_PATH_PASSIVE)
		apr_cvp = v->apr_cvp;
	else
		apr_cvp = v->apr_q6_cvp;

	pr_debug("%s: apr_cvp 0x%x\n", __func__, (unsigned int)apr_cvp);

	return apr_cvp;
}

static void voice_set_apr_cvp(struct voice_data *v, void *apr_cvp)
{
	pr_debug("%s: apr_cvp 0x%x\n", __func__, (unsigned int)apr_cvp);

	if (v->voc_path == VOC_PATH_PASSIVE)
		v->apr_cvp = apr_cvp;
	else
		v->apr_q6_cvp = apr_cvp;
}

static u16 voice_get_mvm_handle(struct voice_data *v)
{
	u16 mvm_handle = 0;

	if (v->voc_path == VOC_PATH_PASSIVE)
		mvm_handle = v->mvm_handle;
	else
		mvm_handle = v->mvm_q6_handle;

	pr_debug("%s: mvm_handle %d\n", __func__, mvm_handle);

	return mvm_handle;
}

static void voice_set_mvm_handle(struct voice_data *v, u16 mvm_handle)
{
	pr_debug("%s: mvm_handle %d\n", __func__, mvm_handle);

	if (v->voc_path == VOC_PATH_PASSIVE)
		v->mvm_handle = mvm_handle;
	else
		v->mvm_q6_handle = mvm_handle;
}

static u16 voice_get_cvs_handle(struct voice_data *v)
{
	u16 cvs_handle = 0;

	if (v->voc_path == VOC_PATH_PASSIVE)
		cvs_handle = v->cvs_handle;
	else
		cvs_handle = v->cvs_q6_handle;

	pr_debug("%s: cvs_handle %d\n", __func__, cvs_handle);

	return cvs_handle;
}

static void voice_set_cvs_handle(struct voice_data *v, u16 cvs_handle)
{
	pr_debug("%s: cvs_handle %d\n", __func__, cvs_handle);

	if (v->voc_path == VOC_PATH_PASSIVE)
		v->cvs_handle = cvs_handle;
	else
		v->cvs_q6_handle = cvs_handle;
}

static u16 voice_get_cvp_handle(struct voice_data *v)
{
	u16 cvp_handle = 0;

	if (v->voc_path == VOC_PATH_PASSIVE)
		cvp_handle = v->cvp_handle;
	else
		cvp_handle = v->cvp_q6_handle;

	pr_debug("%s: cvp_handle %d\n", __func__, cvp_handle);

	return cvp_handle;
}

static void voice_set_cvp_handle(struct voice_data *v, u16 cvp_handle)
{
	pr_debug("%s: cvp_handle %d\n", __func__, cvp_handle);

	if (v->voc_path == VOC_PATH_PASSIVE)
		v->cvp_handle = cvp_handle;
	else
		v->cvp_q6_handle = cvp_handle;
}

static void voice_auddev_cb_function(u32 evt_id,
			union auddev_evt_data *evt_payload,
			void *private_data);

static int32_t modem_mvm_callback(struct apr_client_data *data, void *priv);
static int32_t modem_cvs_callback(struct apr_client_data *data, void *priv);
static int32_t modem_cvp_callback(struct apr_client_data *data, void *priv);

static int voice_apr_register(struct voice_data *v)
{
	int rc = 0;
	void *apr_mvm = voice_get_apr_mvm(v);
	void *apr_cvs = voice_get_apr_cvs(v);
	void *apr_cvp = voice_get_apr_cvp(v);


	pr_debug("into voice_apr_register_callback\n");
	/* register callback to APR */
	if (apr_mvm == NULL) {
		pr_debug("start to register MVM callback\n");

		if (v->voc_path == VOC_PATH_PASSIVE) {
			apr_mvm = apr_register("MODEM", "MVM",
					       modem_mvm_callback, 0xFFFFFFFF,
					       v);
		} else {
			apr_mvm = apr_register("ADSP", "MVM",
					       modem_mvm_callback, 0xFFFFFFFF,
					       v);
		}

		if (apr_mvm == NULL) {
			pr_err("Unable to register MVM\n");
			rc = -ENODEV;
			goto done;
		}

		voice_set_apr_mvm(v, apr_mvm);
	}

	if (apr_cvs == NULL) {
		pr_debug("start to register CVS callback\n");

		if (v->voc_path == VOC_PATH_PASSIVE) {
			apr_cvs = apr_register("MODEM", "CVS",
					       modem_cvs_callback, 0xFFFFFFFF,
					       v);
		} else {
			apr_cvs = apr_register("ADSP", "CVS",
					       modem_cvs_callback, 0xFFFFFFFF,
					       v);
		}

		if (apr_cvs == NULL) {
			pr_err("Unable to register CVS\n");
			rc = -ENODEV;
			goto err;
		}

		voice_set_apr_cvs(v, apr_cvs);
	}

	if (apr_cvp == NULL) {
		pr_debug("start to register CVP callback\n");

		if (v->voc_path == VOC_PATH_PASSIVE) {
			apr_cvp = apr_register("MODEM", "CVP",
					       modem_cvp_callback, 0xFFFFFFFF,
					       v);
		} else {
			apr_cvp = apr_register("ADSP", "CVP",
					       modem_cvp_callback, 0xFFFFFFFF,
					       v);
	}

		if (apr_cvp == NULL) {
			pr_err("Unable to register CVP\n");
			rc = -ENODEV;
			goto err1;
		}

		voice_set_apr_cvp(v, apr_cvp);
	}
	return 0;

err1:
	apr_deregister(apr_cvs);
	apr_cvs = NULL;
	voice_set_apr_cvs(v, apr_cvs);
err:
	apr_deregister(apr_mvm);
	apr_mvm = NULL;
	voice_set_apr_mvm(v, apr_mvm);

done:
	return rc;
}

static int voice_create_mvm_cvs_session(struct voice_data *v)
{
	int ret = 0;
	struct mvm_create_passive_ctl_session_cmd mvm_session_cmd;
	struct cvs_create_passive_ctl_session_cmd cvs_session_cmd;
	struct cvs_create_full_ctl_session_cmd cvs_full_ctl_cmd;
	struct mvm_attach_stream_cmd attach_stream_cmd;
	void *apr_mvm = voice_get_apr_mvm(v);
	void *apr_cvs = voice_get_apr_cvs(v);
	void *apr_cvp = voice_get_apr_cvp(v);
	u16 mvm_handle = voice_get_mvm_handle(v);
	u16 cvs_handle = voice_get_cvs_handle(v);
	u16 cvp_handle = voice_get_cvp_handle(v);

	pr_info("%s:\n", __func__);

	/* start to ping if modem service is up */
	pr_debug("in voice_create_mvm_cvs_session, mvm_hdl=%d, cvs_hdl=%d\n",
					mvm_handle, cvs_handle);
	/* send cmd to create mvm session and wait for response */

	if (!mvm_handle) {
		mvm_session_cmd.hdr.hdr_field = APR_HDR_FIELD(
						APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
		mvm_session_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
					sizeof(mvm_session_cmd) - APR_HDR_SIZE);
		pr_info("send mvm create session pkt size = %d\n",
					mvm_session_cmd.hdr.pkt_size);
		mvm_session_cmd.hdr.src_port = 0;
		mvm_session_cmd.hdr.dest_port = 0;
		mvm_session_cmd.hdr.token = 0;
		if (v->voc_path == VOC_PATH_PASSIVE) {
			pr_info("%s: creating MVM passive ctrl\n", __func__);

		mvm_session_cmd.hdr.opcode =
				VSS_IMVM_CMD_CREATE_PASSIVE_CONTROL_SESSION;
		} else {
			pr_info("%s: creating MVM full ctrl\n", __func__);

			mvm_session_cmd.hdr.opcode =
			VSS_IMVM_CMD_CREATE_FULL_CONTROL_SESSION;
		}
		v->mvm_state = CMD_STATUS_FAIL;

		ret = apr_send_pkt(apr_mvm, (uint32_t *) &mvm_session_cmd);
		if (ret < 0) {
			pr_err("Fail in sending MVM_CONTROL_SESSION\n");
			goto fail;
		}
		ret = wait_event_timeout(v->mvm_wait,
					 (v->mvm_state == CMD_STATUS_SUCCESS),
					 msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			pr_err("%s: wait_event timeout\n", __func__);
			goto fail;
		}

		/* Get the created MVM handle. */
		mvm_handle = voice_get_mvm_handle(v);
	}

	/* send cmd to create cvs session */
	if (!cvs_handle) {
		if (v->voc_path == VOC_PATH_PASSIVE) {
			pr_info("%s:creating CVS passive session\n", __func__);

		cvs_session_cmd.hdr.hdr_field = APR_HDR_FIELD(
						APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
		cvs_session_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
					sizeof(cvs_session_cmd) - APR_HDR_SIZE);
		pr_info("send stream create session pkt size = %d\n",
					cvs_session_cmd.hdr.pkt_size);
		cvs_session_cmd.hdr.src_port = 0;
		cvs_session_cmd.hdr.dest_port = 0;
		cvs_session_cmd.hdr.token = 0;
		cvs_session_cmd.hdr.opcode =
				VSS_ISTREAM_CMD_CREATE_PASSIVE_CONTROL_SESSION;
		strcpy(cvs_session_cmd.cvs_session.name, "default modem voice");

		v->cvs_state = CMD_STATUS_FAIL;

		pr_info("%s: CVS create\n", __func__);
		ret = apr_send_pkt(apr_cvs, (uint32_t *) &cvs_session_cmd);
		if (ret < 0) {
			pr_err("Fail in sending STREAM_CONTROL_SESSION\n");
			goto fail;
		}
		ret = wait_event_timeout(v->cvs_wait,
					 (v->cvs_state == CMD_STATUS_SUCCESS),
					 msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			pr_err("%s: wait_event timeout\n", __func__);
			goto fail;
		}

			/* Get the created CVS handle. */
			cvs_handle = voice_get_cvs_handle(v);
		} else {
			pr_info("%s:creating CVS full session\n", __func__);

			cvs_full_ctl_cmd.hdr.hdr_field =
				APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					      APR_HDR_LEN(APR_HDR_SIZE),
					      APR_PKT_VER);

			cvs_full_ctl_cmd.hdr.pkt_size =
				APR_PKT_SIZE(APR_HDR_SIZE,
				       sizeof(cvs_full_ctl_cmd) - APR_HDR_SIZE);

			cvs_full_ctl_cmd.hdr.src_port = 0;
			cvs_full_ctl_cmd.hdr.dest_port = 0;
			cvs_full_ctl_cmd.hdr.token = 0;
			cvs_full_ctl_cmd.hdr.opcode =
			VSS_ISTREAM_CMD_CREATE_FULL_CONTROL_SESSION;
			cvs_full_ctl_cmd.cvs_session.direction = 2;

			cvs_full_ctl_cmd.cvs_session.enc_media_type =
							v->mvs_info.media_type;
			cvs_full_ctl_cmd.cvs_session.dec_media_type =
							v->mvs_info.media_type;
			cvs_full_ctl_cmd.cvs_session.network_id =
						       v->mvs_info.network_type;
			strcpy(cvs_full_ctl_cmd.cvs_session.name,
			       "default q6 voice");

			v->cvs_state = CMD_STATUS_FAIL;

			ret = apr_send_pkt(apr_cvs,
					   (uint32_t *) &cvs_full_ctl_cmd);

			if (ret < 0) {
				pr_err("%s: Err %d sending CREATE_FULL_CTRL\n",
					   __func__, ret);
				goto fail;
			}
			ret = wait_event_timeout(v->cvs_wait,
					(v->cvs_state == CMD_STATUS_SUCCESS),
					msecs_to_jiffies(TIMEOUT_MS));
			if (!ret) {
				pr_err("%s: wait_event timeout\n", __func__);
				goto fail;
			}

			/* Get the created CVS handle. */
			cvs_handle = voice_get_cvs_handle(v);

			/* Attach MVM to CVS. */
			pr_info("%s: Attach MVM to stream\n", __func__);

			attach_stream_cmd.hdr.hdr_field =
				APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					      APR_HDR_LEN(APR_HDR_SIZE),
					      APR_PKT_VER);

			attach_stream_cmd.hdr.pkt_size =
				APR_PKT_SIZE(APR_HDR_SIZE,
				      sizeof(attach_stream_cmd) - APR_HDR_SIZE);
			attach_stream_cmd.hdr.src_port = 0;
			attach_stream_cmd.hdr.dest_port = mvm_handle;
			attach_stream_cmd.hdr.token = 0;
			attach_stream_cmd.hdr.opcode =
						VSS_IMVM_CMD_ATTACH_STREAM;
			attach_stream_cmd.attach_stream.handle = cvs_handle;

			v->mvm_state = CMD_STATUS_FAIL;
			ret = apr_send_pkt(apr_mvm,
					   (uint32_t *) &attach_stream_cmd);
			if (ret < 0) {
				pr_err("%s: Error %d sending ATTACH_STREAM\n",
				       __func__, ret);
				goto fail;
			}
			ret = wait_event_timeout(v->mvm_wait,
					 (v->mvm_state == CMD_STATUS_SUCCESS),
					 msecs_to_jiffies(TIMEOUT_MS));
			if (!ret) {
				pr_err("%s: wait_event timeout\n", __func__);
				goto fail;
			}
		}
	}

	return 0;

fail:
	apr_deregister(apr_mvm);
	apr_mvm = NULL;
	voice_set_apr_mvm(v, apr_mvm);

	apr_deregister(apr_cvs);
	apr_cvs = NULL;
	voice_set_apr_cvs(v, apr_cvs);

	apr_deregister(apr_cvp);
	apr_cvp = NULL;
	voice_set_apr_cvp(v, apr_cvp);

	cvp_handle = 0;
	voice_set_cvp_handle(v, cvp_handle);

	cvs_handle = 0;
	voice_set_cvs_handle(v, cvs_handle);

	return -EINVAL;
}

static int voice_destroy_mvm_cvs_session(struct voice_data *v)
{
	int ret = 0;
	struct mvm_detach_stream_cmd detach_stream;
	struct apr_hdr mvm_destroy;
	struct apr_hdr cvs_destroy;
	void *apr_mvm = voice_get_apr_mvm(v);
	void *apr_cvs = voice_get_apr_cvs(v);
	u16 mvm_handle = voice_get_mvm_handle(v);
	u16 cvs_handle = voice_get_cvs_handle(v);

	/* MVM, CVS sessions are destroyed only for Full control sessions. */
	if (v->voc_path == VOC_PATH_FULL) {
		pr_info("%s: MVM detach stream\n", __func__);

		/* Detach voice stream. */
		detach_stream.hdr.hdr_field =
			APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				      APR_HDR_LEN(APR_HDR_SIZE),
				      APR_PKT_VER);
		detach_stream.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
					  sizeof(detach_stream) - APR_HDR_SIZE);
		detach_stream.hdr.src_port = 0;
		detach_stream.hdr.dest_port = mvm_handle;
		detach_stream.hdr.token = 0;
		detach_stream.hdr.opcode = VSS_IMVM_CMD_DETACH_STREAM;
		detach_stream.detach_stream.handle = cvs_handle;

		v->mvm_state = CMD_STATUS_FAIL;

		ret = apr_send_pkt(apr_mvm, (uint32_t *) &detach_stream);
		if (ret < 0) {
			pr_err("%s: Error %d sending DETACH_STREAM\n",
			       __func__, ret);

			goto fail;
		}

		ret = wait_event_timeout(v->mvm_wait,
					 (v->mvm_state == CMD_STATUS_SUCCESS),
					 msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			pr_err("%s: wait event timeout\n", __func__);
			goto fail;
		}

		/* Destroy CVS. */
		pr_info("%s: CVS destroy session\n", __func__);

		cvs_destroy.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						      APR_HDR_LEN(APR_HDR_SIZE),
						      APR_PKT_VER);
		cvs_destroy.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
					    sizeof(cvs_destroy) - APR_HDR_SIZE);
		cvs_destroy.src_port = 0;
		cvs_destroy.dest_port = cvs_handle;
		cvs_destroy.token = 0;
		cvs_destroy.opcode = APRV2_IBASIC_CMD_DESTROY_SESSION;

		v->cvs_state = CMD_STATUS_FAIL;

		ret = apr_send_pkt(apr_cvs, (uint32_t *) &cvs_destroy);
		if (ret < 0) {
			pr_err("%s: Error %d sending CVS DESTROY\n",
			       __func__, ret);

			goto fail;
		}

		ret = wait_event_timeout(v->cvs_wait,
					 (v->cvs_state == CMD_STATUS_SUCCESS),
					 msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			pr_err("%s: wait event timeout\n", __func__);

			goto fail;
		}
		cvs_handle = 0;
		voice_set_cvs_handle(v, cvs_handle);

		/* Destroy MVM. */
		pr_info("%s: MVM destroy session\n", __func__);

		mvm_destroy.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						      APR_HDR_LEN(APR_HDR_SIZE),
						      APR_PKT_VER);
		mvm_destroy.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
					    sizeof(mvm_destroy) - APR_HDR_SIZE);
		mvm_destroy.src_port = 0;
		mvm_destroy.dest_port = mvm_handle;
		mvm_destroy.token = 0;
		mvm_destroy.opcode = APRV2_IBASIC_CMD_DESTROY_SESSION;

		v->mvm_state = CMD_STATUS_FAIL;

		ret = apr_send_pkt(apr_mvm, (uint32_t *) &mvm_destroy);
		if (ret < 0) {
			pr_err("%s: Error %d sending MVM DESTROY\n",
			       __func__, ret);

			goto fail;
		}

		ret = wait_event_timeout(v->mvm_wait,
					 (v->mvm_state == CMD_STATUS_SUCCESS),
					 msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			pr_err("%s: wait event timeout\n", __func__);

			goto fail;
		}
		mvm_handle = 0;
		voice_set_mvm_handle(v, mvm_handle);
	}

fail:
	return 0;
}

static int voice_send_tty_mode_to_modem(struct voice_data *v)
{
	struct msm_snddev_info *dev_tx_info;
	struct msm_snddev_info *dev_rx_info;
	int tty_mode = 0;
	int ret = 0;
	struct mvm_set_tty_mode_cmd mvm_tty_mode_cmd;
	void *apr_mvm = voice_get_apr_mvm(v);
	u16 mvm_handle = voice_get_mvm_handle(v);

	dev_rx_info = audio_dev_ctrl_find_dev(v->dev_rx.dev_id);
	if (IS_ERR(dev_rx_info)) {
		pr_err("bad dev_id %d\n", v->dev_rx.dev_id);
		goto done;
	}

	dev_tx_info = audio_dev_ctrl_find_dev(v->dev_tx.dev_id);
	if (IS_ERR(dev_tx_info)) {
		pr_err("bad dev_id %d\n", v->dev_tx.dev_id);
		goto done;
	}

	if ((dev_rx_info->capability & SNDDEV_CAP_TTY) &&
		(dev_tx_info->capability & SNDDEV_CAP_TTY))
		tty_mode = 3; /* FULL */
	else if (!(dev_tx_info->capability & SNDDEV_CAP_TTY) &&
		(dev_rx_info->capability & SNDDEV_CAP_TTY))
		tty_mode = 2; /* VCO */
	else if ((dev_tx_info->capability & SNDDEV_CAP_TTY) &&
		!(dev_rx_info->capability & SNDDEV_CAP_TTY))
		tty_mode = 1; /* HCO */

	if (tty_mode) {
		/* send tty mode cmd to mvm */
		mvm_tty_mode_cmd.hdr.hdr_field = APR_HDR_FIELD(
			APR_MSG_TYPE_SEQ_CMD, APR_HDR_LEN(APR_HDR_SIZE),
								APR_PKT_VER);
		mvm_tty_mode_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
			sizeof(mvm_tty_mode_cmd) - APR_HDR_SIZE);
		pr_debug("pkt size = %d\n", mvm_tty_mode_cmd.hdr.pkt_size);
		mvm_tty_mode_cmd.hdr.src_port = 0;
		mvm_tty_mode_cmd.hdr.dest_port = mvm_handle;
		mvm_tty_mode_cmd.hdr.token = 0;
		mvm_tty_mode_cmd.hdr.opcode = VSS_ISTREAM_CMD_SET_TTY_MODE;
		mvm_tty_mode_cmd.tty_mode.mode = tty_mode;
		pr_info("tty mode =%d\n", mvm_tty_mode_cmd.tty_mode.mode);

		v->mvm_state = CMD_STATUS_FAIL;
		pr_info("%s: MVM set tty\n", __func__);
		ret = apr_send_pkt(apr_mvm, (uint32_t *) &mvm_tty_mode_cmd);
		if (ret < 0) {
			pr_err("Fail: sending VSS_ISTREAM_CMD_SET_TTY_MODE\n");
			goto done;
		}
		ret = wait_event_timeout(v->mvm_wait,
					 (v->mvm_state == CMD_STATUS_SUCCESS),
					 msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			pr_err("%s: wait_event timeout\n", __func__);
			goto done;
		}
	}
	return 0;
done:
	return -EINVAL;
}

static int voice_send_cvs_cal_to_modem(struct voice_data *v)
{
	struct apr_hdr cvs_cal_cmd_hdr;
	uint32_t *cmd_buf;
	struct acdb_cal_data cal_data;
	struct acdb_cal_block *cal_blk;
	int32_t cal_size_per_network;
	uint32_t *cal_data_per_network;
	int index = 0;
	int ret = 0;
	void *apr_cvs = voice_get_apr_cvs(v);
	u16 cvs_handle = voice_get_cvs_handle(v);

	/* fill the header */
	cvs_cal_cmd_hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
		APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	cvs_cal_cmd_hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
		sizeof(cvs_cal_cmd_hdr) - APR_HDR_SIZE);
	cvs_cal_cmd_hdr.src_port = 0;
	cvs_cal_cmd_hdr.dest_port = cvs_handle;
	cvs_cal_cmd_hdr.token = 0;
	cvs_cal_cmd_hdr.opcode =
		VSS_ISTREAM_CMD_CACHE_CALIBRATION_DATA;

	pr_debug("voice_send_cvs_cal_to_modem\n");
	/* get the cvs cal data */
	get_vocstrm_cal(&cal_data);
	if (cal_data.num_cal_blocks == 0) {
		pr_debug("%s: No calibration data to send!\n", __func__);
		goto done;
	}

	/* send cvs cal to modem */
	cmd_buf = kzalloc((sizeof(struct apr_hdr) + BUFFER_PAYLOAD_SIZE),
								GFP_KERNEL);
	if (!cmd_buf) {
		pr_err("No memory is allocated.\n");
		return -ENOMEM;
	}
	pr_debug("----- num_cal_blocks=%d\n", (s32)cal_data.num_cal_blocks);
	cal_blk = cal_data.cal_blocks;
	pr_debug("cal_blk =%x\n", (uint32_t)cal_data.cal_blocks);

	for (; index < cal_data.num_cal_blocks; index++) {
		cal_size_per_network = cal_blk[index].cal_size;
		pr_debug(" cal size =%d\n", cal_size_per_network);
		if (cal_size_per_network >= BUFFER_PAYLOAD_SIZE)
			pr_err("Cal size is too big\n");
		cal_data_per_network = (u32 *)cal_blk[index].cal_kvaddr;
		pr_debug(" cal data=%x\n", (uint32_t)cal_data_per_network);
		cvs_cal_cmd_hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
			cal_size_per_network);
		pr_debug("header size =%d,  pkt_size =%d\n",
			APR_HDR_SIZE, cvs_cal_cmd_hdr.pkt_size);
		memcpy(cmd_buf, &cvs_cal_cmd_hdr,  APR_HDR_SIZE);
		memcpy(cmd_buf + (APR_HDR_SIZE / sizeof(uint32_t)),
			cal_data_per_network, cal_size_per_network);
		pr_debug("send cvs cal: index =%d\n", index);
		v->cvs_state = CMD_STATUS_FAIL;
		ret = apr_send_pkt(apr_cvs, cmd_buf);
		if (ret < 0) {
			pr_err("Fail: sending cvs cal, idx=%d\n", index);
			continue;
		}
		ret = wait_event_timeout(v->cvs_wait,
					 (v->cvs_state == CMD_STATUS_SUCCESS),
					 msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			pr_err("%s: wait_event timeout\n", __func__);
			return -EINVAL;
		}
	}
	kfree(cmd_buf);
done:
	return 0;
}

static int voice_send_cvp_cal_to_modem(struct voice_data *v)
{
	struct apr_hdr cvp_cal_cmd_hdr;
	uint32_t *cmd_buf;
	struct acdb_cal_data cal_data;
	struct acdb_cal_block *cal_blk;
	int32_t cal_size_per_network;
	uint32_t *cal_data_per_network;
	int index = 0;
	int ret = 0;
	void *apr_cvp = voice_get_apr_cvp(v);
	u16 cvp_handle = voice_get_cvp_handle(v);


	/* fill the header */
	cvp_cal_cmd_hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
		APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	cvp_cal_cmd_hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
		sizeof(cvp_cal_cmd_hdr) - APR_HDR_SIZE);
	cvp_cal_cmd_hdr.src_port = 0;
	cvp_cal_cmd_hdr.dest_port = cvp_handle;
	cvp_cal_cmd_hdr.token = 0;
	cvp_cal_cmd_hdr.opcode =
		VSS_IVOCPROC_CMD_CACHE_CALIBRATION_DATA;

	/* get cal data */
	get_vocproc_cal(&cal_data);
	if (cal_data.num_cal_blocks == 0) {
		pr_debug("%s: No calibration data to send!\n", __func__);
		goto done;
	}

	/* send cal to modem */
	cmd_buf = kzalloc((sizeof(struct apr_hdr) + BUFFER_PAYLOAD_SIZE),
								GFP_KERNEL);
	if (!cmd_buf) {
		pr_err("No memory is allocated.\n");
		return -ENOMEM;
	}
	pr_debug("----- num_cal_blocks=%d\n", (s32)cal_data.num_cal_blocks);
	cal_blk = cal_data.cal_blocks;
	pr_debug(" cal_blk =%x\n", (uint32_t)cal_data.cal_blocks);

	for (; index < cal_data.num_cal_blocks; index++) {
		cal_size_per_network = cal_blk[index].cal_size;
		if (cal_size_per_network >= BUFFER_PAYLOAD_SIZE)
			pr_err("Cal size is too big\n");
		pr_debug(" cal size =%d\n", cal_size_per_network);
		cal_data_per_network = (u32 *)cal_blk[index].cal_kvaddr;
		pr_debug(" cal data=%x\n", (uint32_t)cal_data_per_network);

		cvp_cal_cmd_hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
			cal_size_per_network);
		memcpy(cmd_buf, &cvp_cal_cmd_hdr,  APR_HDR_SIZE);
		memcpy(cmd_buf + (APR_HDR_SIZE / sizeof(*cmd_buf)),
			cal_data_per_network, cal_size_per_network);
		pr_debug("Send cvp cal\n");
		v->cvp_state = CMD_STATUS_FAIL;
		pr_info("%s: CVP calib\n", __func__);
		ret = apr_send_pkt(apr_cvp, cmd_buf);
		if (ret < 0) {
			pr_err("Fail: sending cvp cal, idx=%d\n", index);
			continue;
		}
		ret = wait_event_timeout(v->cvp_wait,
					 (v->cvp_state == CMD_STATUS_SUCCESS),
					 msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			pr_err("%s: wait_event timeout\n", __func__);
			return -EINVAL;
		}
	}
	kfree(cmd_buf);
done:
	return 0;
}

static int voice_send_cvp_vol_tbl_to_modem(struct voice_data *v)
{
	struct apr_hdr cvp_vol_cal_cmd_hdr;
	uint32_t *cmd_buf;
	struct acdb_cal_data cal_data;
	struct acdb_cal_block *cal_blk;
	int32_t cal_size_per_network;
	uint32_t *cal_data_per_network;
	int index = 0;
	int ret = 0;
	void *apr_cvp = voice_get_apr_cvp(v);
	u16 cvp_handle = voice_get_cvp_handle(v);


	/* fill the header */
	cvp_vol_cal_cmd_hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
		APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	cvp_vol_cal_cmd_hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
		sizeof(cvp_vol_cal_cmd_hdr) - APR_HDR_SIZE);
	cvp_vol_cal_cmd_hdr.src_port = 0;
	cvp_vol_cal_cmd_hdr.dest_port = cvp_handle;
	cvp_vol_cal_cmd_hdr.token = 0;
	cvp_vol_cal_cmd_hdr.opcode =
		VSS_IVOCPROC_CMD_CACHE_VOLUME_CALIBRATION_TABLE;

	/* get cal data */
	get_vocvol_cal(&cal_data);
	if (cal_data.num_cal_blocks == 0) {
		pr_debug("%s: No calibration data to send!\n", __func__);
		goto done;
	}

	/* send cal to modem */
	cmd_buf = kzalloc((sizeof(struct apr_hdr) + BUFFER_PAYLOAD_SIZE),
								GFP_KERNEL);
	if (!cmd_buf) {
		pr_err("No memory is allocated.\n");
		return -ENOMEM;
	}
	pr_debug("----- num_cal_blocks=%d\n", (s32)cal_data.num_cal_blocks);
	cal_blk = cal_data.cal_blocks;
	pr_debug("Cal_blk =%x\n", (uint32_t)cal_data.cal_blocks);

	for (; index < cal_data.num_cal_blocks; index++) {
		cal_size_per_network = cal_blk[index].cal_size;
		cal_data_per_network = (u32 *)cal_blk[index].cal_kvaddr;
		pr_debug("Cal size =%d, index=%d\n", cal_size_per_network,
			index);
		pr_debug("Cal data=%x\n", (uint32_t)cal_data_per_network);
		cvp_vol_cal_cmd_hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
			cal_size_per_network);
		memcpy(cmd_buf, &cvp_vol_cal_cmd_hdr,  APR_HDR_SIZE);
		memcpy(cmd_buf + (APR_HDR_SIZE / sizeof(uint32_t)),
			cal_data_per_network, cal_size_per_network);
		pr_debug("Send vol table\n");

		v->cvp_state = CMD_STATUS_FAIL;
		ret = apr_send_pkt(apr_cvp, cmd_buf);
		if (ret < 0) {
			pr_err("Fail: sending cvp vol cal, idx=%d\n", index);
			continue;
		}
		ret = wait_event_timeout(v->cvp_wait,
					 (v->cvp_state == CMD_STATUS_SUCCESS),
			msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			pr_err("%s: wait_event timeout\n", __func__);
			return -EINVAL;
		}
	}
	kfree(cmd_buf);
done:
	return 0;
}

static int voice_config_cvs_vocoder(struct voice_data *v)
{
	int ret = 0;
	void *apr_cvs = voice_get_apr_cvs(v);
	u16 cvs_handle = voice_get_cvs_handle(v);

	/* Set media type. */
	struct cvs_set_media_type_cmd cvs_set_media_cmd;

	pr_info("%s: Setting media type\n", __func__);

	cvs_set_media_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
						APR_PKT_VER);
	cvs_set_media_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				      sizeof(cvs_set_media_cmd) - APR_HDR_SIZE);
	cvs_set_media_cmd.hdr.src_port = 0;
	cvs_set_media_cmd.hdr.dest_port = cvs_handle;
	cvs_set_media_cmd.hdr.token = 0;
	cvs_set_media_cmd.hdr.opcode = VSS_ISTREAM_CMD_SET_MEDIA_TYPE;
	cvs_set_media_cmd.media_type.tx_media_id = v->mvs_info.media_type;
	cvs_set_media_cmd.media_type.rx_media_id = v->mvs_info.media_type;

	v->cvs_state = CMD_STATUS_FAIL;

	ret = apr_send_pkt(apr_cvs, (uint32_t *) &cvs_set_media_cmd);
	if (ret < 0) {
		pr_err("%s: Error %d sending SET_MEDIA_TYPE\n",
		       __func__, ret);

		goto fail;
	}

	ret = wait_event_timeout(v->cvs_wait,
				 (v->cvs_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);

		goto fail;
	}

	/* Set encoder properties. */
	switch (v->mvs_info.media_type) {
	case VSS_MEDIA_ID_EVRC_MODEM: {
		struct cvs_set_cdma_enc_minmax_rate_cmd cvs_set_cdma_rate;

		pr_info("%s: Setting EVRC min-max rate\n", __func__);

		cvs_set_cdma_rate.hdr.hdr_field =
				APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					      APR_HDR_LEN(APR_HDR_SIZE),
					      APR_PKT_VER);
		cvs_set_cdma_rate.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				      sizeof(cvs_set_cdma_rate) - APR_HDR_SIZE);
		cvs_set_cdma_rate.hdr.src_port = 0;
		cvs_set_cdma_rate.hdr.dest_port = cvs_handle;
		cvs_set_cdma_rate.hdr.token = 0;
		cvs_set_cdma_rate.hdr.opcode =
				VSS_ISTREAM_CMD_CDMA_SET_ENC_MINMAX_RATE;
		cvs_set_cdma_rate.cdma_rate.min_rate = v->mvs_info.rate;
		cvs_set_cdma_rate.cdma_rate.max_rate = v->mvs_info.rate;

		v->cvs_state = CMD_STATUS_FAIL;

		ret = apr_send_pkt(apr_cvs, (uint32_t *) &cvs_set_cdma_rate);
		if (ret < 0) {
			pr_err("%s: Error %d sending SET_EVRC_MINMAX_RATE\n",
			       __func__, ret);

			goto fail;
		}

		ret = wait_event_timeout(v->cvs_wait,
					 (v->cvs_state == CMD_STATUS_SUCCESS),
					 msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			pr_err("%s: wait_event timeout\n", __func__);

			goto fail;
		}

		break;
	}

	case VSS_MEDIA_ID_AMR_NB_MODEM: {
		struct cvs_set_amr_enc_rate_cmd cvs_set_amr_rate;
		struct cvs_set_enc_dtx_mode_cmd cvs_set_dtx;

		pr_info("%s: Setting AMR rate\n", __func__);

		cvs_set_amr_rate.hdr.hdr_field =
				APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					      APR_HDR_LEN(APR_HDR_SIZE),
					      APR_PKT_VER);
		cvs_set_amr_rate.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				       sizeof(cvs_set_amr_rate) - APR_HDR_SIZE);
		cvs_set_amr_rate.hdr.src_port = 0;
		cvs_set_amr_rate.hdr.dest_port = cvs_handle;
		cvs_set_amr_rate.hdr.token = 0;
		cvs_set_amr_rate.hdr.opcode =
					VSS_ISTREAM_CMD_VOC_AMR_SET_ENC_RATE;
		cvs_set_amr_rate.amr_rate.mode = v->mvs_info.rate;

		v->cvs_state = CMD_STATUS_FAIL;

		ret = apr_send_pkt(apr_cvs, (uint32_t *) &cvs_set_amr_rate);
		if (ret < 0) {
			pr_err("%s: Error %d sending SET_AMR_RATE\n",
			       __func__, ret);

			goto fail;
		}

		ret = wait_event_timeout(v->cvs_wait,
					 (v->cvs_state == CMD_STATUS_SUCCESS),
					 msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			pr_err("%s: wait_event timeout\n", __func__);

			goto fail;
		}

		/* Disable DTX */
		pr_info("%s: Disabling DTX\n", __func__);

		cvs_set_dtx.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						      APR_HDR_LEN(APR_HDR_SIZE),
						      APR_PKT_VER);
		cvs_set_dtx.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				       sizeof(cvs_set_dtx) - APR_HDR_SIZE);
		cvs_set_dtx.hdr.src_port = 0;
		cvs_set_dtx.hdr.dest_port = cvs_handle;
		cvs_set_dtx.hdr.token = 0;
		cvs_set_dtx.hdr.opcode = VSS_ISTREAM_CMD_SET_ENC_DTX_MODE;
		cvs_set_dtx.dtx_mode.enable = 0;

		v->cvs_state = CMD_STATUS_FAIL;

		ret = apr_send_pkt(apr_cvs, (uint32_t *) &cvs_set_dtx);
		if (ret < 0) {
			pr_err("%s: Error %d sending SET_DTX\n",
			       __func__, ret);

			goto fail;
		}

		ret = wait_event_timeout(v->cvs_wait,
					 (v->cvs_state == CMD_STATUS_SUCCESS),
					 msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			pr_err("%s: wait_event timeout\n", __func__);

			goto fail;
		}

		break;
	}

	case VSS_MEDIA_ID_AMR_WB_MODEM: {
		struct cvs_set_amrwb_enc_rate_cmd cvs_set_amrwb_rate;
		struct cvs_set_enc_dtx_mode_cmd cvs_set_dtx;

		pr_info("%s: Setting AMR WB rate\n", __func__);

		cvs_set_amrwb_rate.hdr.hdr_field =
				APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					      APR_HDR_LEN(APR_HDR_SIZE),
					      APR_PKT_VER);
		cvs_set_amrwb_rate.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				     sizeof(cvs_set_amrwb_rate) - APR_HDR_SIZE);
		cvs_set_amrwb_rate.hdr.src_port = 0;
		cvs_set_amrwb_rate.hdr.dest_port = cvs_handle;
		cvs_set_amrwb_rate.hdr.token = 0;
		cvs_set_amrwb_rate.hdr.opcode =
					VSS_ISTREAM_CMD_VOC_AMRWB_SET_ENC_RATE;
		cvs_set_amrwb_rate.amrwb_rate.mode = v->mvs_info.rate;

		v->cvs_state = CMD_STATUS_FAIL;

		ret = apr_send_pkt(apr_cvs, (uint32_t *) &cvs_set_amrwb_rate);
		if (ret < 0) {
			pr_err("%s: Error %d sending SET_AMRWB_RATE\n",
			       __func__, ret);

			goto fail;
		}

		ret = wait_event_timeout(v->cvs_wait,
					 (v->cvs_state == CMD_STATUS_SUCCESS),
					 msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			pr_err("%s: wait_event timeout\n", __func__);

			goto fail;
		}

		/* Disable DTX */
		pr_info("%s: Disabling DTX\n", __func__);

		cvs_set_dtx.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						      APR_HDR_LEN(APR_HDR_SIZE),
						      APR_PKT_VER);
		cvs_set_dtx.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				       sizeof(cvs_set_dtx) - APR_HDR_SIZE);
		cvs_set_dtx.hdr.src_port = 0;
		cvs_set_dtx.hdr.dest_port = cvs_handle;
		cvs_set_dtx.hdr.token = 0;
		cvs_set_dtx.hdr.opcode = VSS_ISTREAM_CMD_SET_ENC_DTX_MODE;
		cvs_set_dtx.dtx_mode.enable = 0;

		v->cvs_state = CMD_STATUS_FAIL;

		ret = apr_send_pkt(apr_cvs, (uint32_t *) &cvs_set_dtx);
		if (ret < 0) {
			pr_err("%s: Error %d sending SET_DTX\n",
			       __func__, ret);

			goto fail;
		}

		ret = wait_event_timeout(v->cvs_wait,
					 (v->cvs_state == CMD_STATUS_SUCCESS),
					 msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			pr_err("%s: wait_event timeout\n", __func__);

			goto fail;
		}

		break;
	}

	case VSS_MEDIA_ID_G729:
	case VSS_MEDIA_ID_G711_ALAW:
	case VSS_MEDIA_ID_G711_MULAW: {
		struct cvs_set_enc_dtx_mode_cmd cvs_set_dtx;
		/* Disable DTX */
		pr_info("%s: Disabling DTX\n", __func__);

		cvs_set_dtx.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						      APR_HDR_LEN(APR_HDR_SIZE),
						      APR_PKT_VER);
		cvs_set_dtx.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
					    sizeof(cvs_set_dtx) - APR_HDR_SIZE);
		cvs_set_dtx.hdr.src_port = 0;
		cvs_set_dtx.hdr.dest_port = cvs_handle;
		cvs_set_dtx.hdr.token = 0;
		cvs_set_dtx.hdr.opcode = VSS_ISTREAM_CMD_SET_ENC_DTX_MODE;
		cvs_set_dtx.dtx_mode.enable = 0;

		v->cvs_state = CMD_STATUS_FAIL;

		ret = apr_send_pkt(apr_cvs, (uint32_t *) &cvs_set_dtx);
		if (ret < 0) {
			pr_err("%s: Error %d sending SET_DTX\n",
			       __func__, ret);

			goto fail;
		}

		ret = wait_event_timeout(v->cvs_wait,
					 (v->cvs_state == CMD_STATUS_SUCCESS),
					 msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			pr_err("%s: wait_event timeout\n", __func__);

			goto fail;
		}

		break;
	}

	default: {
		/* Do nothing. */
	}
	}

	return 0;

fail:
	return -EINVAL;
}

static int voice_send_start_voice_cmd(struct voice_data *v)
{
	struct apr_hdr mvm_start_voice_cmd;
	int ret = 0;
	void *apr_mvm = voice_get_apr_mvm(v);
	u16 mvm_handle = voice_get_mvm_handle(v);

	mvm_start_voice_cmd.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	mvm_start_voice_cmd.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(mvm_start_voice_cmd) - APR_HDR_SIZE);
	pr_info("send mvm_start_voice_cmd pkt size = %d\n",
				mvm_start_voice_cmd.pkt_size);
	mvm_start_voice_cmd.src_port = 0;
	mvm_start_voice_cmd.dest_port = mvm_handle;
	mvm_start_voice_cmd.token = 0;
	mvm_start_voice_cmd.opcode = VSS_IMVM_CMD_START_VOICE;

	v->mvm_state = CMD_STATUS_FAIL;
	ret = apr_send_pkt(apr_mvm, (uint32_t *) &mvm_start_voice_cmd);
	if (ret < 0) {
		pr_err("Fail in sending VSS_IMVM_CMD_START_VOICE\n");
		goto fail;
	}
	ret = wait_event_timeout(v->mvm_wait,
				 (v->mvm_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		goto fail;
	}
	return 0;
fail:
	return -EINVAL;
}

static int voice_send_stop_voice_cmd(struct voice_data *v)
{
	struct apr_hdr mvm_stop_voice_cmd;
	int ret = 0;
	void *apr_mvm = voice_get_apr_mvm(v);
	u16 mvm_handle = voice_get_mvm_handle(v);

	mvm_stop_voice_cmd.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	mvm_stop_voice_cmd.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(mvm_stop_voice_cmd) - APR_HDR_SIZE);
	pr_info("send mvm_stop_voice_cmd pkt size = %d\n",
				mvm_stop_voice_cmd.pkt_size);
	mvm_stop_voice_cmd.src_port = 0;
	mvm_stop_voice_cmd.dest_port = mvm_handle;
	mvm_stop_voice_cmd.token = 0;
	mvm_stop_voice_cmd.opcode = VSS_IMVM_CMD_STOP_VOICE;

	v->mvm_state = CMD_STATUS_FAIL;
	ret = apr_send_pkt(apr_mvm, (uint32_t *) &mvm_stop_voice_cmd);
	if (ret < 0) {
		pr_err("Fail in sending VSS_IMVM_CMD_STOP_VOICE\n");
		goto fail;
	}
	ret = wait_event_timeout(v->mvm_wait,
				 (v->mvm_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		goto fail;
	}

	return 0;
fail:
	return -EINVAL;
}

static int voice_setup_modem_voice(struct voice_data *v)
{
	struct cvp_create_full_ctl_session_cmd cvp_session_cmd;
	struct apr_hdr cvp_enable_cmd;
	struct mvm_attach_vocproc_cmd mvm_a_vocproc_cmd;
	int ret = 0;
	struct msm_snddev_info *dev_tx_info;
	void *apr_mvm = voice_get_apr_mvm(v);
	void *apr_cvp = voice_get_apr_cvp(v);
	u16 mvm_handle = voice_get_mvm_handle(v);
	u16 cvp_handle = voice_get_cvp_handle(v);

	/* create cvp session and wait for response */
	cvp_session_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	cvp_session_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(cvp_session_cmd) - APR_HDR_SIZE);
	pr_info(" send create cvp session, pkt size = %d\n",
				cvp_session_cmd.hdr.pkt_size);
	cvp_session_cmd.hdr.src_port = 0;
	cvp_session_cmd.hdr.dest_port = 0;
	cvp_session_cmd.hdr.token = 0;
	cvp_session_cmd.hdr.opcode =
		VSS_IVOCPROC_CMD_CREATE_FULL_CONTROL_SESSION;

	dev_tx_info = audio_dev_ctrl_find_dev(v->dev_tx.dev_id);
	if (IS_ERR(dev_tx_info)) {
		pr_err("bad dev_id %d\n", v->dev_tx.dev_id);
		goto fail;
	}

	if (dev_tx_info->channel_mode > 1)
		cvp_session_cmd.cvp_session.tx_topology_id =
			VSS_IVOCPROC_TOPOLOGY_ID_TX_DM_FLUENCE;
	else
		cvp_session_cmd.cvp_session.tx_topology_id =
			VSS_IVOCPROC_TOPOLOGY_ID_TX_SM_ECNS;
	cvp_session_cmd.cvp_session.rx_topology_id =
			VSS_IVOCPROC_TOPOLOGY_ID_RX_DEFAULT;
	cvp_session_cmd.cvp_session.direction = 2; /*tx and rx*/
	cvp_session_cmd.cvp_session.network_id = VSS_NETWORK_ID_DEFAULT;
	cvp_session_cmd.cvp_session.tx_port_id = v->dev_tx.dev_port_id;
	cvp_session_cmd.cvp_session.rx_port_id = v->dev_rx.dev_port_id;
	pr_info("topology=%d net_id=%d, dir=%d tx_port_id=%d, rx_port_id=%d\n",
			cvp_session_cmd.cvp_session.tx_topology_id,
			cvp_session_cmd.cvp_session.network_id,
			cvp_session_cmd.cvp_session.direction,
			cvp_session_cmd.cvp_session.tx_port_id,
			cvp_session_cmd.cvp_session.rx_port_id);

	v->cvp_state = CMD_STATUS_FAIL;
	ret = apr_send_pkt(apr_cvp, (uint32_t *) &cvp_session_cmd);
	if (ret < 0) {
		pr_err("Fail in sending VOCPROC_FULL_CONTROL_SESSION\n");
		goto fail;
	}
	pr_debug("wait for cvp create session event\n");
	ret = wait_event_timeout(v->cvp_wait,
				 (v->cvp_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		goto fail;
	}

	/* Get the created handle. */
	cvp_handle = voice_get_cvp_handle(v);

	/* send cvs cal */
	voice_send_cvs_cal_to_modem(v);

	/* send cvp cal */
	voice_send_cvp_cal_to_modem(v);

	/* send cvp vol table cal */
	voice_send_cvp_vol_tbl_to_modem(v);

	/* enable vocproc and wait for respose */
	cvp_enable_cmd.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	cvp_enable_cmd.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(cvp_enable_cmd) - APR_HDR_SIZE);
	pr_debug("cvp_enable_cmd pkt size = %d, cvp_handle=%d\n",
			cvp_enable_cmd.pkt_size, cvp_handle);
	cvp_enable_cmd.src_port = 0;
	cvp_enable_cmd.dest_port = cvp_handle;
	cvp_enable_cmd.token = 0;
	cvp_enable_cmd.opcode = VSS_IVOCPROC_CMD_ENABLE;

	v->cvp_state = CMD_STATUS_FAIL;
	ret = apr_send_pkt(apr_cvp, (uint32_t *) &cvp_enable_cmd);
	if (ret < 0) {
		pr_err("Fail in sending VSS_IVOCPROC_CMD_ENABLE\n");
		goto fail;
	}
	ret = wait_event_timeout(v->cvp_wait,
				 (v->cvp_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		goto fail;
	}

	/* attach vocproc and wait for response */
	mvm_a_vocproc_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	mvm_a_vocproc_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(mvm_a_vocproc_cmd) - APR_HDR_SIZE);
	pr_info("send mvm_a_vocproc_cmd pkt size = %d\n",
				mvm_a_vocproc_cmd.hdr.pkt_size);
	mvm_a_vocproc_cmd.hdr.src_port = 0;
	mvm_a_vocproc_cmd.hdr.dest_port = mvm_handle;
	mvm_a_vocproc_cmd.hdr.token = 0;
	mvm_a_vocproc_cmd.hdr.opcode = VSS_ISTREAM_CMD_ATTACH_VOCPROC;
	mvm_a_vocproc_cmd.mvm_attach_cvp_handle.handle = cvp_handle;

	v->mvm_state = CMD_STATUS_FAIL;
	ret = apr_send_pkt(apr_mvm, (uint32_t *) &mvm_a_vocproc_cmd);
	if (ret < 0) {
		pr_err("Fail in sending VSS_ISTREAM_CMD_ATTACH_VOCPROC\n");
		goto fail;
	}
	ret = wait_event_timeout(v->mvm_wait,
				 (v->mvm_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		goto fail;
	}

	/* send tty mode if tty device is used */
	voice_send_tty_mode_to_modem(v);

	if (v->voc_path == VOC_PATH_FULL) {
		struct mvm_set_network_cmd mvm_set_network;
		struct mvm_set_voice_timing_cmd mvm_set_voice_timing;

		ret = voice_config_cvs_vocoder(v);
		if (ret < 0) {
			pr_err("%s: Error %d configuring CVS voc",
			       __func__, ret);

			goto fail;
		}

		/* Set network ID. */
		pr_info("%s: Setting network ID\n", __func__);

		mvm_set_network.hdr.hdr_field =
				APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					      APR_HDR_LEN(APR_HDR_SIZE),
					      APR_PKT_VER);
		mvm_set_network.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
					sizeof(mvm_set_network) - APR_HDR_SIZE);
		mvm_set_network.hdr.src_port = 0;
		mvm_set_network.hdr.dest_port = mvm_handle;
		mvm_set_network.hdr.token = 0;
		mvm_set_network.hdr.opcode = VSS_ICOMMON_CMD_SET_NETWORK;
		mvm_set_network.network.network_id = v->mvs_info.network_type;

		v->mvm_state = CMD_STATUS_FAIL;

		ret = apr_send_pkt(apr_mvm, (uint32_t *) &mvm_set_network);
		if (ret < 0) {
			pr_err("%s: Error %d sending SET_NETWORK\n",
			       __func__, ret);

			goto fail;
		}

		ret = wait_event_timeout(v->mvm_wait,
					 (v->mvm_state == CMD_STATUS_SUCCESS),
					 msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			pr_err("%s: wait_event timeout\n", __func__);

			goto fail;
		}

		/* Set voice timing. */
		pr_info("%s: Setting voice timing\n", __func__);

		mvm_set_voice_timing.hdr.hdr_field =
				APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					      APR_HDR_LEN(APR_HDR_SIZE),
					      APR_PKT_VER);
		mvm_set_voice_timing.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(mvm_set_voice_timing) - APR_HDR_SIZE);
		mvm_set_voice_timing.hdr.src_port = 0;
		mvm_set_voice_timing.hdr.dest_port = mvm_handle;
		mvm_set_voice_timing.hdr.token = 0;
		mvm_set_voice_timing.hdr.opcode =
				VSS_ICOMMON_CMD_SET_VOICE_TIMING;
		mvm_set_voice_timing.timing.mode = 0;
		mvm_set_voice_timing.timing.enc_offset = 8000;
		mvm_set_voice_timing.timing.dec_req_offset = 3300;
		mvm_set_voice_timing.timing.dec_offset = 8300;

		v->mvm_state = CMD_STATUS_FAIL;

		ret = apr_send_pkt(apr_mvm, (uint32_t *) &mvm_set_voice_timing);
		if (ret < 0) {
			pr_err("%s: Error %d sending SET_TIMING\n",
				       __func__, ret);

				goto fail;
		}

		ret = wait_event_timeout(v->mvm_wait,
					 (v->mvm_state == CMD_STATUS_SUCCESS),
					 msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			pr_err("%s: wait_event timeout\n", __func__);

			goto fail;
		}
	}

	return 0;
fail:
	return -EINVAL;
}

static int voice_destroy_modem_voice(struct voice_data *v)
{
	struct mvm_detach_vocproc_cmd mvm_d_vocproc_cmd;
	struct apr_hdr cvp_destroy_session_cmd;
	int ret = 0;
	void *apr_mvm = voice_get_apr_mvm(v);
	void *apr_cvp = voice_get_apr_cvp(v);
	u16 mvm_handle = voice_get_mvm_handle(v);
	u16 cvp_handle = voice_get_cvp_handle(v);

	/* detach VOCPROC and wait for response from mvm */
	mvm_d_vocproc_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	mvm_d_vocproc_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(mvm_d_vocproc_cmd) - APR_HDR_SIZE);
	pr_info("mvm_d_vocproc_cmd  pkt size = %d\n",
				mvm_d_vocproc_cmd.hdr.pkt_size);
	mvm_d_vocproc_cmd.hdr.src_port = 0;
	mvm_d_vocproc_cmd.hdr.dest_port = mvm_handle;
	mvm_d_vocproc_cmd.hdr.token = 0;
	mvm_d_vocproc_cmd.hdr.opcode = VSS_ISTREAM_CMD_DETACH_VOCPROC;
	mvm_d_vocproc_cmd.mvm_detach_cvp_handle.handle = cvp_handle;

	v->mvm_state = CMD_STATUS_FAIL;
	ret = apr_send_pkt(apr_mvm, (uint32_t *) &mvm_d_vocproc_cmd);
	if (ret < 0) {
		pr_err("Fail in sending VSS_ISTREAM_CMD_DETACH_VOCPROC\n");
		goto fail;
	}
	ret = wait_event_timeout(v->mvm_wait,
				 (v->mvm_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		goto fail;
	}

	/* destrop cvp session */
	cvp_destroy_session_cmd.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	cvp_destroy_session_cmd.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(cvp_destroy_session_cmd) - APR_HDR_SIZE);
	pr_info("cvp_destroy_session_cmd pkt size = %d\n",
				cvp_destroy_session_cmd.pkt_size);
	cvp_destroy_session_cmd.src_port = 0;
	cvp_destroy_session_cmd.dest_port = cvp_handle;
	cvp_destroy_session_cmd.token = 0;
	cvp_destroy_session_cmd.opcode = APRV2_IBASIC_CMD_DESTROY_SESSION;

	v->cvp_state = CMD_STATUS_FAIL;
	ret = apr_send_pkt(apr_cvp, (uint32_t *) &cvp_destroy_session_cmd);
	if (ret < 0) {
		pr_err("Fail in sending APRV2_IBASIC_CMD_DESTROY_SESSION\n");
		goto fail;
	}
	ret = wait_event_timeout(v->cvp_wait,
				 (v->cvp_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		goto fail;
	}

	cvp_handle = 0;
	voice_set_cvp_handle(v, cvp_handle);

	return 0;

fail:
	return -EINVAL;
}

static int voice_send_mute_cmd_to_modem(struct voice_data *v)
{
	struct cvs_set_mute_cmd cvs_mute_cmd;
	int ret = 0;
	void *apr_cvs = voice_get_apr_cvs(v);
	u16 cvs_handle = voice_get_cvs_handle(v);

	/* send mute/unmute to cvs */
	cvs_mute_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	cvs_mute_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(cvs_mute_cmd) - APR_HDR_SIZE);
	cvs_mute_cmd.hdr.src_port = 0;
	cvs_mute_cmd.hdr.dest_port = cvs_handle;
	cvs_mute_cmd.hdr.token = 0;
	cvs_mute_cmd.hdr.opcode = VSS_ISTREAM_CMD_SET_MUTE;
	cvs_mute_cmd.cvs_set_mute.direction = 0; /*tx*/
	cvs_mute_cmd.cvs_set_mute.mute_flag = v->dev_tx.mute;

	pr_info(" mute value =%d\n", cvs_mute_cmd.cvs_set_mute.mute_flag);
	v->cvs_state = CMD_STATUS_FAIL;
	ret = apr_send_pkt(apr_cvs, (uint32_t *) &cvs_mute_cmd);
	if (ret < 0) {
		pr_err("Fail: send STREAM SET MUTE\n");
		goto fail;
	}
	ret = wait_event_timeout(v->cvs_wait,
				 (v->cvs_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret)
		pr_err("%s: wait_event timeout\n", __func__);

fail:
	return 0;
}

static int voice_send_vol_index_to_modem(struct voice_data *v)
{
	struct cvp_set_rx_volume_index_cmd cvp_vol_cmd;
	int ret = 0;
	void *apr_cvp = voice_get_apr_cvp(v);
	u16 cvp_handle = voice_get_cvp_handle(v);

	/* send volume index to cvp */
	cvp_vol_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
		APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	cvp_vol_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
		sizeof(cvp_vol_cmd) - APR_HDR_SIZE);
	cvp_vol_cmd.hdr.src_port = 0;
	cvp_vol_cmd.hdr.dest_port = cvp_handle;
	cvp_vol_cmd.hdr.token = 0;
	cvp_vol_cmd.hdr.opcode =
		VSS_IVOCPROC_CMD_SET_RX_VOLUME_INDEX;
	cvp_vol_cmd.cvp_set_vol_idx.vol_index = v->dev_rx.volume;
	v->cvp_state = CMD_STATUS_FAIL;
	ret = apr_send_pkt(apr_cvp, (uint32_t *) &cvp_vol_cmd);
	if (ret < 0) {
		pr_err("Fail in sending RX VOL INDEX\n");
		return -EINVAL;
	}
	ret = wait_event_timeout(v->cvp_wait,
				 (v->cvp_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static void voice_auddev_cb_function(u32 evt_id,
			union auddev_evt_data *evt_payload,
			void *private_data)
{
	struct voice_data *v = &voice;
	struct sidetone_cal sidetone_cal_data;

	pr_info("auddev_cb_function, evt_id=%d,\n", evt_id);
	if ((evt_id != AUDDEV_EVT_START_VOICE) ||
			(evt_id != AUDDEV_EVT_END_VOICE)) {
		if (evt_payload == NULL) {
			pr_err(" evt_payload is NULL pointer\n");
			return;
		}
	}

	switch (evt_id) {
	case AUDDEV_EVT_START_VOICE:
		mutex_lock(&v->lock);

		if ((v->voc_state == VOC_INIT) ||
				(v->voc_state == VOC_RELEASE)) {
			v->v_call_status = VOICE_CALL_START;
			if ((v->dev_rx.enabled == VOICE_DEV_ENABLED)
				&& (v->dev_tx.enabled == VOICE_DEV_ENABLED)) {
				voice_apr_register(v);
				voice_create_mvm_cvs_session(v);
				voice_setup_modem_voice(v);
				voice_send_start_voice_cmd(v);
				get_sidetone_cal(&sidetone_cal_data);
				msm_snddev_enable_sidetone(
					v->dev_rx.dev_id,
					sidetone_cal_data.enable,
					sidetone_cal_data.gain);
				v->voc_state = VOC_RUN;
			}
		}

		mutex_unlock(&v->lock);
		break;
	case AUDDEV_EVT_DEV_CHG_VOICE:
		if (v->dev_rx.enabled == VOICE_DEV_ENABLED)
			msm_snddev_enable_sidetone(v->dev_rx.dev_id, 0, 0);
		v->dev_rx.enabled = VOICE_DEV_DISABLED;
		v->dev_tx.enabled = VOICE_DEV_DISABLED;

		mutex_lock(&v->lock);

		if (v->voc_state == VOC_RUN) {
			/* send cmd to modem to do voice device change */
			voice_destroy_modem_voice(v);
			v->voc_state = VOC_CHANGE;
		}

		mutex_unlock(&v->lock);
		break;
	case AUDDEV_EVT_DEV_RDY:
		mutex_lock(&v->lock);

		if (v->voc_state == VOC_CHANGE) {
			/* get port Ids */
			if (evt_payload->voc_devinfo.dev_type == DIR_RX) {
				v->dev_rx.dev_port_id =
					evt_payload->voc_devinfo.dev_port_id;
				v->dev_rx.sample =
					evt_payload->voc_devinfo.dev_sample;
				v->dev_rx.dev_id =
				evt_payload->voc_devinfo.dev_id;
				v->dev_rx.enabled = VOICE_DEV_ENABLED;
			} else {
				v->dev_tx.dev_port_id =
					evt_payload->voc_devinfo.dev_port_id;
				v->dev_tx.sample =
					evt_payload->voc_devinfo.dev_sample;
				v->dev_tx.enabled = VOICE_DEV_ENABLED;
				v->dev_tx.dev_id =
				evt_payload->voc_devinfo.dev_id;
			}
			if ((v->dev_rx.enabled == VOICE_DEV_ENABLED) &&
				(v->dev_tx.enabled == VOICE_DEV_ENABLED)) {
				voice_setup_modem_voice(v);
				voice_send_mute_cmd_to_modem(v);
				voice_send_vol_index_to_modem(v);
				get_sidetone_cal(&sidetone_cal_data);
				msm_snddev_enable_sidetone(
					v->dev_rx.dev_id,
					sidetone_cal_data.enable,
					sidetone_cal_data.gain);
				v->voc_state = VOC_RUN;
			}
		} else if ((v->voc_state == VOC_INIT) ||
			(v->voc_state == VOC_RELEASE)) {
			/* get AFE ports */
			if (evt_payload->voc_devinfo.dev_type == DIR_RX) {
				/* get rx port id */
				v->dev_rx.dev_port_id =
					evt_payload->voc_devinfo.dev_port_id;
				v->dev_rx.sample =
					evt_payload->voc_devinfo.dev_sample;
				v->dev_rx.dev_id =
				evt_payload->voc_devinfo.dev_id;
				v->dev_rx.enabled = VOICE_DEV_ENABLED;
			} else {
				/* get tx port id */
				v->dev_tx.dev_port_id =
					evt_payload->voc_devinfo.dev_port_id;
				v->dev_tx.sample =
					evt_payload->voc_devinfo.dev_sample;
				v->dev_tx.dev_id =
				evt_payload->voc_devinfo.dev_id;
				v->dev_tx.enabled = VOICE_DEV_ENABLED;
			}
			if ((v->dev_rx.enabled == VOICE_DEV_ENABLED) &&
				(v->dev_tx.enabled == VOICE_DEV_ENABLED) &&
				(v->v_call_status == VOICE_CALL_START)) {
				voice_apr_register(v);
				voice_create_mvm_cvs_session(v);
				voice_setup_modem_voice(v);
				voice_send_start_voice_cmd(v);
				get_sidetone_cal(&sidetone_cal_data);
				msm_snddev_enable_sidetone(
					v->dev_rx.dev_id,
					sidetone_cal_data.enable,
					sidetone_cal_data.gain);
				v->voc_state = VOC_RUN;
			}
		}

		mutex_unlock(&v->lock);
		break;
	case AUDDEV_EVT_DEVICE_VOL_MUTE_CHG:
		/* cache the mute and volume index value */
		if (evt_payload->voc_devinfo.dev_type == DIR_TX) {
			v->dev_tx.mute =
				evt_payload->voc_vm_info.dev_vm_val.mute;

			mutex_lock(&v->lock);

			if (v->voc_state == VOC_RUN)
				voice_send_mute_cmd_to_modem(v);

			mutex_unlock(&v->lock);
		} else {
			v->dev_rx.volume = evt_payload->
				voc_vm_info.dev_vm_val.vol;

			mutex_lock(&v->lock);

			if (v->voc_state == VOC_RUN)
				voice_send_vol_index_to_modem(v);

			mutex_unlock(&v->lock);
		}
		break;
	case AUDDEV_EVT_REL_PENDING:

		mutex_lock(&v->lock);

		if (v->voc_state == VOC_RUN) {
			voice_destroy_modem_voice(v);
			v->voc_state = VOC_CHANGE;
		}

		mutex_unlock(&v->lock);

		if (evt_payload->voc_devinfo.dev_type == DIR_RX)
			v->dev_rx.enabled = VOICE_DEV_DISABLED;
		else
			v->dev_tx.enabled = VOICE_DEV_DISABLED;

		break;
	case AUDDEV_EVT_END_VOICE:
		/* recover the tx mute and rx volume to the default values */
		v->dev_tx.mute = v->default_mute_val;
		v->dev_rx.volume = v->default_vol_val;
		if (v->dev_rx.enabled == VOICE_DEV_ENABLED)
			msm_snddev_enable_sidetone(v->dev_rx.dev_id, 0, 0);

		mutex_lock(&v->lock);

		if (v->voc_state == VOC_RUN) {
			/* call stop modem voice */
			voice_send_stop_voice_cmd(v);
			voice_destroy_modem_voice(v);
			voice_destroy_mvm_cvs_session(v);
			v->voc_state = VOC_RELEASE;
		} else if (v->voc_state == VOC_CHANGE) {
			voice_send_stop_voice_cmd(v);
			voice_destroy_mvm_cvs_session(v);
			v->voc_state = VOC_RELEASE;
		}

		mutex_unlock(&v->lock);

		v->v_call_status = VOICE_CALL_END;

		break;
	default:
		pr_err("UNKNOWN EVENT\n");
	}
	return;
}
EXPORT_SYMBOL(voice_auddev_cb_function);

int voice_set_voc_path_full(uint32_t set)
{
	int rc = 0;

	pr_info("%s: %d\n", __func__, set);

	mutex_lock(&voice.lock);

	if (voice.voc_state == VOC_INIT || voice.voc_state == VOC_RELEASE) {
		if (set)
			voice.voc_path = VOC_PATH_FULL;
		else
			voice.voc_path = VOC_PATH_PASSIVE;
	} else {
		pr_err("%s: Invalid voc path set to %d, in state %d\n",
		       __func__, set, voice.voc_state);

		rc = -EPERM;
	}

	mutex_unlock(&voice.lock);

	return rc;
}
EXPORT_SYMBOL(voice_set_voc_path_full);

void voice_register_mvs_cb(ul_cb_fn ul_cb,
			   dl_cb_fn dl_cb,
			   void *private_data)
{
	voice.mvs_info.ul_cb = ul_cb;
	voice.mvs_info.dl_cb = dl_cb;
	voice.mvs_info.private_data = private_data;
}

void voice_config_vocoder(uint32_t media_type,
			  uint32_t rate,
			  uint32_t network_type)
{
	voice.mvs_info.media_type = media_type;
	voice.mvs_info.rate = rate;
	voice.mvs_info.network_type = network_type;
}

static int32_t modem_mvm_callback(struct apr_client_data *data, void *priv)
{
	uint32_t *ptr;
	struct voice_data *v = priv;

	pr_debug("%s\n", __func__);
	pr_debug("%s: Payload Length = %d, opcode=%x\n", __func__,
				data->payload_size, data->opcode);

	if (data->opcode == APR_BASIC_RSP_RESULT) {
		if (data->payload_size) {
			ptr = data->payload;

			pr_info("%x %x\n", ptr[0], ptr[1]);
			/* ping mvm service ACK */

			if (ptr[0] ==
			 VSS_IMVM_CMD_CREATE_PASSIVE_CONTROL_SESSION ||
			ptr[0] ==
			    VSS_IMVM_CMD_CREATE_FULL_CONTROL_SESSION) {
				/* Passive session is used for voice call
				 * through modem. Full session is used for voice
				 * call through Q6. */
				pr_debug("%s: cmd = 0x%x\n", __func__, ptr[0]);
				if (!ptr[1]) {
					pr_debug("%s: MVM handle is %d\n",
						 __func__, data->src_port);

					voice_set_mvm_handle(v, data->src_port);
				} else
					pr_info("got NACK for sending \
							MVM create session \n");
				v->mvm_state = CMD_STATUS_SUCCESS;
				wake_up(&v->mvm_wait);
			} else if (ptr[0] == VSS_IMVM_CMD_START_VOICE) {
				pr_debug("%s: cmd = 0x%x\n", __func__, ptr[0]);
				v->mvm_state = CMD_STATUS_SUCCESS;
				wake_up(&v->mvm_wait);
			} else if (ptr[0] == VSS_ISTREAM_CMD_ATTACH_VOCPROC) {
				pr_debug("%s: cmd = 0x%x\n", __func__, ptr[0]);
				v->mvm_state = CMD_STATUS_SUCCESS;
				wake_up(&v->mvm_wait);
			} else if (ptr[0] == VSS_IMVM_CMD_STOP_VOICE) {
				v->mvm_state = CMD_STATUS_SUCCESS;
				wake_up(&v->mvm_wait);
			} else if (ptr[0] == VSS_ISTREAM_CMD_DETACH_VOCPROC) {
				v->mvm_state = CMD_STATUS_SUCCESS;
				wake_up(&v->mvm_wait);
			} else if (ptr[0] == VSS_ISTREAM_CMD_SET_TTY_MODE) {
				v->mvm_state = CMD_STATUS_SUCCESS;
				wake_up(&v->mvm_wait);
			} else if (ptr[0] == APRV2_IBASIC_CMD_DESTROY_SESSION) {
				pr_debug("%s: DESTROY resp\n", __func__);

				v->mvm_state = CMD_STATUS_SUCCESS;
				wake_up(&v->mvm_wait);
			} else if (ptr[0] == VSS_IMVM_CMD_ATTACH_STREAM) {
				pr_debug("%s: ATTACH_STREAM resp 0x%x\n",
					__func__, ptr[1]);

				v->mvm_state = CMD_STATUS_SUCCESS;
				wake_up(&v->mvm_wait);
			} else if (ptr[0] == VSS_IMVM_CMD_DETACH_STREAM) {
				pr_debug("%s: DETACH_STREAM resp 0x%x\n",
					__func__, ptr[1]);

				v->mvm_state = CMD_STATUS_SUCCESS;
				wake_up(&v->mvm_wait);
			} else if (ptr[0] == VSS_ICOMMON_CMD_SET_NETWORK) {
				pr_debug("%s: SET_NETWORK resp 0x%x\n",
					__func__, ptr[1]);

				v->mvm_state = CMD_STATUS_SUCCESS;
				wake_up(&v->mvm_wait);
			} else if (ptr[0] == VSS_ICOMMON_CMD_SET_VOICE_TIMING) {
				pr_debug("%s: SET_VOICE_TIMING resp 0x%x\n",
					__func__, ptr[1]);

				v->mvm_state = CMD_STATUS_SUCCESS;
				wake_up(&v->mvm_wait);
			} else
				pr_debug("%s: not match cmd = 0x%x\n",
					__func__, ptr[0]);
		}
	}

	return 0;
}

static int32_t modem_cvs_callback(struct apr_client_data *data, void *priv)
{
	uint32_t *ptr;
	struct voice_data *v = priv;

	pr_debug("%s\n", __func__);
	pr_debug("%s: Payload Length = %d, opcode=%x\n", __func__,
					data->payload_size, data->opcode);

	if (data->opcode == APR_BASIC_RSP_RESULT) {
		if (data->payload_size) {
			ptr = data->payload;

			pr_debug("%x %x\n", ptr[0], ptr[1]);
			/*response from modem CVS */
			if (ptr[0] ==
			VSS_ISTREAM_CMD_CREATE_PASSIVE_CONTROL_SESSION ||
			    ptr[0] ==
			    VSS_ISTREAM_CMD_CREATE_FULL_CONTROL_SESSION) {
				if (!ptr[1]) {
					pr_debug("%s: CVS handle is %d\n",
						 __func__, data->src_port);
					voice_set_cvs_handle(v, data->src_port);
				} else
					pr_info("got NACK for sending \
							CVS create session \n");
				v->cvs_state = CMD_STATUS_SUCCESS;
				wake_up(&v->cvs_wait);
			} else if (ptr[0] ==
				VSS_ISTREAM_CMD_CACHE_CALIBRATION_DATA) {
				v->cvs_state = CMD_STATUS_SUCCESS;
				wake_up(&v->cvs_wait);
			} else if (ptr[0] ==
					VSS_ISTREAM_CMD_SET_MUTE) {
				v->cvs_state = CMD_STATUS_SUCCESS;
				wake_up(&v->cvs_wait);
			} else if (ptr[0] == VSS_ISTREAM_CMD_SET_MEDIA_TYPE) {
				pr_debug("%s: SET_MEDIA resp 0x%x\n",
					 __func__, ptr[1]);

				v->cvs_state = CMD_STATUS_SUCCESS;
				wake_up(&v->cvs_wait);
			} else if (ptr[0] ==
				   VSS_ISTREAM_CMD_VOC_AMR_SET_ENC_RATE) {
				pr_debug("%s: SET_AMR_RATE resp 0x%x\n",
					 __func__, ptr[1]);

				v->cvs_state = CMD_STATUS_SUCCESS;
				wake_up(&v->cvs_wait);
			} else if (ptr[0] ==
				   VSS_ISTREAM_CMD_VOC_AMRWB_SET_ENC_RATE) {
				pr_debug("%s: SET_AMR_WB_RATE resp 0x%x\n",
					 __func__, ptr[1]);

				v->cvs_state = CMD_STATUS_SUCCESS;
				wake_up(&v->cvs_wait);
			} else if (ptr[0] == VSS_ISTREAM_CMD_SET_ENC_DTX_MODE) {
				pr_debug("%s: SET_DTX resp 0x%x\n",
					 __func__, ptr[1]);

				v->cvs_state = CMD_STATUS_SUCCESS;
				wake_up(&v->cvs_wait);
			} else if (ptr[0] ==
				   VSS_ISTREAM_CMD_CDMA_SET_ENC_MINMAX_RATE) {
				pr_debug("%s: SET_CDMA_RATE resp 0x%x\n",
					 __func__, ptr[1]);

				v->cvs_state = CMD_STATUS_SUCCESS;
				wake_up(&v->cvs_wait);
			} else if (ptr[0] == APRV2_IBASIC_CMD_DESTROY_SESSION) {
				pr_debug("%s: DESTROY resp\n", __func__);

				v->cvs_state = CMD_STATUS_SUCCESS;
				wake_up(&v->cvs_wait);
			} else
				pr_debug("%s: cmd = 0x%x\n", __func__, ptr[0]);
		}
	} else if (data->opcode == VSS_ISTREAM_EVT_SEND_ENC_BUFFER) {
		uint32_t *voc_pkt = data->payload;
		uint32_t pkt_len = data->payload_size;

		if (voc_pkt != NULL && v->mvs_info.ul_cb != NULL) {
			pr_debug("%s: Media type is 0x%x\n",
				 __func__, voc_pkt[0]);

			/* Remove media ID from payload. */
			voc_pkt++;
			pkt_len = pkt_len - 4;

			v->mvs_info.ul_cb((uint8_t *)voc_pkt,
					  pkt_len,
					  v->mvs_info.private_data);
			} else {
				pr_debug("%s: voc_pkt is 0x%x ul_cb is 0x%x\n",
				       __func__, (unsigned int)voc_pkt,
				       (unsigned int) v->mvs_info.ul_cb);
			}
	} else if (data->opcode == VSS_ISTREAM_EVT_SEND_DEC_BUFFER) {
			pr_debug("%s: Send dec buf resp\n", __func__);
	} else if (data->opcode == VSS_ISTREAM_EVT_REQUEST_DEC_BUFFER) {
		struct cvs_send_dec_buf_cmd send_dec_buf;
		int ret = 0;
		uint32_t pkt_len = 0;

		if (v->mvs_info.dl_cb != NULL) {
			send_dec_buf.dec_buf.media_id = v->mvs_info.media_type;

			v->mvs_info.dl_cb(
				(uint8_t *)&send_dec_buf.dec_buf.packet_data,
				&pkt_len,
				v->mvs_info.private_data);

			send_dec_buf.hdr.hdr_field =
				APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					      APR_HDR_LEN(APR_HDR_SIZE),
					      APR_PKT_VER);
			send_dec_buf.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
			       sizeof(send_dec_buf.dec_buf.media_id) + pkt_len);
			send_dec_buf.hdr.src_port = 0;
			send_dec_buf.hdr.dest_port = voice_get_cvs_handle(v);
			send_dec_buf.hdr.token = 0;
			send_dec_buf.hdr.opcode =
					VSS_ISTREAM_EVT_SEND_DEC_BUFFER;

			ret = apr_send_pkt(voice_get_apr_cvs(v),
					   (uint32_t *) &send_dec_buf);
			if (ret < 0) {
				pr_err("%s: Error %d sending DEC_BUF\n",
				       __func__, ret);
				goto fail;
			}
		} else {
			pr_debug("%s: ul_cb is NULL\n", __func__);
		}
	} else {
		pr_debug("%s: Unknown opcode 0x%x\n", __func__, data->opcode);
	}

fail:
	return 0;
}

static int32_t modem_cvp_callback(struct apr_client_data *data, void *priv)
{
	uint32_t *ptr;
	struct voice_data *v = priv;

	pr_debug("%s\n", __func__);
	pr_debug("%s: Payload Length = %d, opcode=%x\n", __func__,
				data->payload_size, data->opcode);

	if (data->opcode == APR_BASIC_RSP_RESULT) {
		if (data->payload_size) {
			ptr = data->payload;

			pr_info("%x %x\n", ptr[0], ptr[1]);
			/*response from modem CVP */
			if (ptr[0] ==
				VSS_IVOCPROC_CMD_CREATE_FULL_CONTROL_SESSION) {
				pr_debug("%s: cmd = 0x%x\n", __func__, ptr[0]);
				if (!ptr[1]) {
					voice_set_cvp_handle(v, data->src_port);
					pr_debug("cvphdl=%d\n", data->src_port);
				} else
					pr_info("got NACK from CVP create \
						session response\n");
				v->cvp_state = CMD_STATUS_SUCCESS;
				wake_up(&v->cvp_wait);
			} else if (ptr[0] ==
				VSS_IVOCPROC_CMD_CACHE_CALIBRATION_DATA) {
				pr_debug("%s: cmd = 0x%x\n", __func__, ptr[0]);
				v->cvp_state = CMD_STATUS_SUCCESS;
				wake_up(&v->cvp_wait);
			} else if (ptr[0] ==
					VSS_IVOCPROC_CMD_SET_RX_VOLUME_INDEX) {
				v->cvp_state = CMD_STATUS_SUCCESS;
				wake_up(&v->cvp_wait);
			} else if (ptr[0] == VSS_IVOCPROC_CMD_ENABLE) {
				v->cvp_state = CMD_STATUS_SUCCESS;
				wake_up(&v->cvp_wait);
			} else if (ptr[0] == APRV2_IBASIC_CMD_DESTROY_SESSION) {
				v->cvp_state = CMD_STATUS_SUCCESS;
				wake_up(&v->cvp_wait);
			} else if (ptr[0] ==
				VSS_IVOCPROC_CMD_CACHE_VOLUME_CALIBRATION_TABLE
				) {

				pr_debug("%s: cmd = 0x%x\n", __func__, ptr[0]);
				v->cvp_state = CMD_STATUS_SUCCESS;
				wake_up(&v->cvp_wait);
			} else
				pr_debug("%s: not match cmd = 0x%x\n",
							__func__, ptr[0]);
		}
	}
	return 0;
}


static int __init voice_init(void)
{
	int rc = 0;
	struct voice_data *v = &voice;

	/* set default value */
	v->default_mute_val = 1;  /* default is mute */
	v->default_vol_val = 0;
	v->default_sample_val = 8000;

	/* initialize dev_rx and dev_tx */
	memset(&v->dev_tx, 0, sizeof(struct device_data));
	memset(&v->dev_rx, 0, sizeof(struct device_data));
	v->dev_rx.volume = v->default_vol_val;
	v->dev_tx.mute = v->default_mute_val;

	v->voc_state = VOC_INIT;
	v->voc_path = VOC_PATH_FULL;
	init_waitqueue_head(&v->mvm_wait);
	init_waitqueue_head(&v->cvs_wait);
	init_waitqueue_head(&v->cvp_wait);

	mutex_init(&v->lock);

	v->mvm_handle = 0;
	v->cvs_handle = 0;
	v->cvp_handle = 0;

	v->mvm_q6_handle = 0;
	v->cvs_q6_handle = 0;
	v->cvp_q6_handle = 0;

	v->apr_mvm = NULL;
	v->apr_cvs = NULL;
	v->apr_cvp = NULL;

	v->apr_q6_mvm = NULL;
	v->apr_q6_cvs = NULL;
	v->apr_q6_cvp = NULL;

	/* Initialize MVS info. */
	memset(&v->mvs_info, 0, sizeof(v->mvs_info));
	v->mvs_info.media_type = VSS_MEDIA_ID_PCM_NB;
	v->mvs_info.rate = MVS_AMR_MODE_UNDEF;
	v->mvs_info.network_type = VSS_NETWORK_ID_VOIP_NB;

	v->device_events = AUDDEV_EVT_DEV_CHG_VOICE |
			AUDDEV_EVT_DEV_RDY |
			AUDDEV_EVT_REL_PENDING |
			AUDDEV_EVT_START_VOICE |
			AUDDEV_EVT_END_VOICE |
			AUDDEV_EVT_DEVICE_VOL_MUTE_CHG |
			AUDDEV_EVT_FREQ_CHG;

	pr_debug("to register call back\n");
	/* register callback to auddev */
	auddev_register_evt_listner(v->device_events, AUDDEV_CLNT_VOC,
				0, voice_auddev_cb_function, v);

	return rc;
}

device_initcall(voice_init);
