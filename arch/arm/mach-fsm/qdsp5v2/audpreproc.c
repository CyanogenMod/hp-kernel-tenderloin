/*
 * Common code to deal with the AUDPREPROC dsp task (audio preprocessing)
 *
 * Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 *
 * Based on the audpp layer in arch/arm/mach-msm/qdsp5/audpp.c
 *
 * Copyright (C) 2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/wakelock.h>
#include <mach/msm_adsp.h>

#include <mach/qdsp5v2/audpreproc.h>
#include <mach/debug_mm.h>

#define MAX_ENC_COUNT 2

#define MSM_ADSP_ENC_CODEC_WAV 0
#define MSM_ADSP_ENC_CODEC_AAC 1
#define MSM_ADSP_ENC_CODEC_SBC 2
#define MSM_ADSP_ENC_CODEC_AMRNB 3
#define MSM_ADSP_ENC_CODEC_EVRC 4
#define MSM_ADSP_ENC_CODEC_QCELP 5

static DEFINE_MUTEX(audpreproc_lock);
static struct wake_lock audpre_wake_lock;

struct msm_adspenc_info {
	const char *module_name;
	unsigned module_queueids;
	int module_encid; /* streamid */
	int enc_formats; /* supported formats */
};

#define ENC_MODULE_INFO(name, queueids, encid, formats) { .module_name = name, \
	.module_queueids = queueids, .module_encid = encid, \
	.enc_formats = formats}

#define MAX_EVENT_CALLBACK_CLIENTS 1

struct msm_adspenc_database {
	unsigned num_enc;
	struct msm_adspenc_info *enc_info_list;
};

static struct msm_adspenc_info enc_info_list[] = {
	ENC_MODULE_INFO("AUDREC0TASK", \
			 ((QDSP_uPAudRec0BitStreamQueue << 16)| \
			   QDSP_uPAudRec0CmdQueue), 0, \
			 ((1 << MSM_ADSP_ENC_CODEC_WAV) | \
			  (1 << MSM_ADSP_ENC_CODEC_SBC))),
	ENC_MODULE_INFO("AUDREC1TASK", \
			 ((QDSP_uPAudRec1BitStreamQueue << 16)| \
			   QDSP_uPAudRec1CmdQueue), 1, \
			 ((1 << MSM_ADSP_ENC_CODEC_WAV) | \
			  (1 << MSM_ADSP_ENC_CODEC_AAC) | \
			  (1 << MSM_ADSP_ENC_CODEC_AMRNB) | \
			  (1 << MSM_ADSP_ENC_CODEC_EVRC) | \
			  (1 << MSM_ADSP_ENC_CODEC_QCELP))),
};

static struct msm_adspenc_database msm_enc_database = {
	.num_enc = ARRAY_SIZE(enc_info_list),
	.enc_info_list = enc_info_list,
};

struct audpreproc_state {
	struct msm_adsp_module *mod;
	audpreproc_event_func func[MAX_ENC_COUNT];
	void *private[MAX_ENC_COUNT];
	struct mutex *lock;
	unsigned open_count;
	unsigned enc_inuse;
	struct audpreproc_event_callback *cb_tbl[MAX_EVENT_CALLBACK_CLIENTS];
};

static struct audpreproc_state the_audpreproc_state = {
	.lock = &audpreproc_lock,
};

static inline void prevent_suspend(void)
{
	wake_lock(&audpre_wake_lock);
}
static inline void allow_suspend(void)
{
	wake_unlock(&audpre_wake_lock);
}

/* DSP preproc event handler */
static void audpreproc_dsp_event(void *data, unsigned id, size_t len,
			    void (*getevent)(void *ptr, size_t len))
{
	struct audpreproc_state *audpreproc = data;
	int n = 0;

	switch (id) {
	case AUDPREPROC_CMD_CFG_DONE_MSG: {
		struct audpreproc_cmd_cfg_done_msg cfg_done_msg;

		getevent(&cfg_done_msg, AUDPREPROC_CMD_CFG_DONE_MSG_LEN);
		MM_DBG("AUDPREPROC_CMD_CFG_DONE_MSG: stream id %d preproc \
			type %x\n", cfg_done_msg.stream_id, \
			cfg_done_msg.aud_preproc_type);
		if ((cfg_done_msg.stream_id < MAX_ENC_COUNT) &&
				audpreproc->func[cfg_done_msg.stream_id])
			audpreproc->func[cfg_done_msg.stream_id](
			audpreproc->private[cfg_done_msg.stream_id], id,
			&cfg_done_msg);
		break;
	}
	case AUDPREPROC_ERROR_MSG: {
		struct audpreproc_err_msg err_msg;

		getevent(&err_msg, AUDPREPROC_ERROR_MSG_LEN);
		MM_DBG("AUDPREPROC_ERROR_MSG: stream id %d err idx %d\n",
		err_msg.stream_id, err_msg.aud_preproc_err_idx);
		if ((err_msg.stream_id < MAX_ENC_COUNT) &&
				audpreproc->func[err_msg.stream_id])
			audpreproc->func[err_msg.stream_id](
			audpreproc->private[err_msg.stream_id], id,
			&err_msg);
		break;
	}
	case AUDPREPROC_CMD_ENC_CFG_DONE_MSG: {
		struct audpreproc_cmd_enc_cfg_done_msg enc_cfg_msg;

		getevent(&enc_cfg_msg, AUDPREPROC_CMD_ENC_CFG_DONE_MSG_LEN);
		MM_DBG("AUDPREPROC_CMD_ENC_CFG_DONE_MSG: stream id %d enc type \
			%d\n", enc_cfg_msg.stream_id, enc_cfg_msg.rec_enc_type);
		if ((enc_cfg_msg.stream_id < MAX_ENC_COUNT) &&
				audpreproc->func[enc_cfg_msg.stream_id])
			audpreproc->func[enc_cfg_msg.stream_id](
			audpreproc->private[enc_cfg_msg.stream_id], id,
			&enc_cfg_msg);
		for (n = 0; n < MAX_EVENT_CALLBACK_CLIENTS; ++n) {
			if (audpreproc->cb_tbl[n] && audpreproc->cb_tbl[n]->fn)
				audpreproc->cb_tbl[n]->fn( \
					audpreproc->cb_tbl[n]->private, \
					id, (void *) &enc_cfg_msg);
		}
		break;
	}
	case AUDPREPROC_CMD_ENC_PARAM_CFG_DONE_MSG: {
		struct audpreproc_cmd_enc_param_cfg_done_msg enc_param_msg;

		getevent(&enc_param_msg,
				AUDPREPROC_CMD_ENC_PARAM_CFG_DONE_MSG_LEN);
		MM_DBG("AUDPREPROC_CMD_ENC_PARAM_CFG_DONE_MSG: stream id %d\n",
				 enc_param_msg.stream_id);
		if ((enc_param_msg.stream_id < MAX_ENC_COUNT) &&
				audpreproc->func[enc_param_msg.stream_id])
			audpreproc->func[enc_param_msg.stream_id](
			audpreproc->private[enc_param_msg.stream_id], id,
			&enc_param_msg);
		break;
	}
	case AUDPREPROC_AFE_CMD_AUDIO_RECORD_CFG_DONE_MSG: {
		struct audpreproc_afe_cmd_audio_record_cfg_done
						record_cfg_done;
		getevent(&record_cfg_done,
			AUDPREPROC_AFE_CMD_AUDIO_RECORD_CFG_DONE_MSG_LEN);
		MM_DBG("AUDPREPROC_AFE_CMD_AUDIO_RECORD_CFG_DONE_MSG: \
			stream id %d\n", record_cfg_done.stream_id);
		if ((record_cfg_done.stream_id < MAX_ENC_COUNT) &&
				audpreproc->func[record_cfg_done.stream_id])
			audpreproc->func[record_cfg_done.stream_id](
			audpreproc->private[record_cfg_done.stream_id], id,
			&record_cfg_done);
		break;
	}
	default:
		MM_ERR("Unknown Event %d\n", id);
	}
	return;
}

static struct msm_adsp_ops adsp_ops = {
	.event = audpreproc_dsp_event,
};

/* EXPORTED API's */
int audpreproc_enable(int enc_id, audpreproc_event_func func, void *private)
{
	struct audpreproc_state *audpreproc = &the_audpreproc_state;
	int res = 0;

	if (enc_id < 0 || enc_id > MAX_ENC_COUNT)
		return -EINVAL;

	mutex_lock(audpreproc->lock);
	if (audpreproc->func[enc_id]) {
		res = -EBUSY;
		goto out;
	}

	audpreproc->func[enc_id] = func;
	audpreproc->private[enc_id] = private;

	/* First client to enable preproc task */
	if (audpreproc->open_count++ == 0) {
		MM_DBG("Get AUDPREPROCTASK\n");
		res = msm_adsp_get("AUDPREPROCTASK", &audpreproc->mod,
				&adsp_ops, audpreproc);
		if (res < 0) {
			MM_ERR("Can not get AUDPREPROCTASK\n");
			audpreproc->open_count = 0;
			audpreproc->func[enc_id] = NULL;
			audpreproc->private[enc_id] = NULL;
			goto out;
		}
		prevent_suspend();
		if (msm_adsp_enable(audpreproc->mod)) {
			MM_ERR("Can not enable AUDPREPROCTASK\n");
			audpreproc->open_count = 0;
			audpreproc->func[enc_id] = NULL;
			audpreproc->private[enc_id] = NULL;
			msm_adsp_put(audpreproc->mod);
			audpreproc->mod = NULL;
			res = -ENODEV;
			allow_suspend();
			goto out;
		}
	}
	res = 0;
out:
	mutex_unlock(audpreproc->lock);
	return res;
}
EXPORT_SYMBOL(audpreproc_enable);

void audpreproc_disable(int enc_id, void *private)
{
	struct audpreproc_state *audpreproc = &the_audpreproc_state;

	if (enc_id < 0 || enc_id > MAX_ENC_COUNT)
		return;

	mutex_lock(audpreproc->lock);
	if (!audpreproc->func[enc_id])
		goto out;
	if (audpreproc->private[enc_id] != private)
		goto out;

	audpreproc->func[enc_id] = NULL;
	audpreproc->private[enc_id] = NULL;

	/* Last client then disable preproc task */
	if (--audpreproc->open_count == 0) {
		msm_adsp_disable(audpreproc->mod);
		MM_DBG("Put AUDPREPROCTASK\n");
		msm_adsp_put(audpreproc->mod);
		audpreproc->mod = NULL;
		allow_suspend();
	}
out:
	mutex_unlock(audpreproc->lock);
	return;
}
EXPORT_SYMBOL(audpreproc_disable);


int audpreproc_register_event_callback(struct audpreproc_event_callback *ecb)
{
	struct audpreproc_state *audpreproc = &the_audpreproc_state;
	int i;

	for (i = 0; i < MAX_EVENT_CALLBACK_CLIENTS; ++i) {
		if (NULL == audpreproc->cb_tbl[i]) {
			audpreproc->cb_tbl[i] = ecb;
			return 0;
		}
	}
	return -1;
}
EXPORT_SYMBOL(audpreproc_register_event_callback);

int audpreproc_unregister_event_callback(struct audpreproc_event_callback *ecb)
{
	struct audpreproc_state *audpreproc = &the_audpreproc_state;
	int i;

	for (i = 0; i < MAX_EVENT_CALLBACK_CLIENTS; ++i) {
		if (ecb == audpreproc->cb_tbl[i]) {
			audpreproc->cb_tbl[i] = NULL;
			return 0;
		}
	}
	return -1;
}
EXPORT_SYMBOL(audpreproc_unregister_event_callback);


/* enc_type = supported encode format *
 * like pcm, aac, sbc, evrc, qcelp, amrnb etc ... *
 */
int audpreproc_aenc_alloc(unsigned enc_type, const char **module_name,
		     unsigned *queue_ids)
{
	struct audpreproc_state *audpreproc = &the_audpreproc_state;
	int encid = -1, idx;
	static int wakelock_init;

	mutex_lock(audpreproc->lock);
	for (idx = (msm_enc_database.num_enc - 1);
		idx >= 0; idx--) {
		/* encoder free and supports the format */
		if ((!(audpreproc->enc_inuse & (1 << idx))) &&
			(msm_enc_database.enc_info_list[idx].enc_formats &
				(1 << enc_type))) {
				break;
		}
	}

	if (idx >= 0) {
		audpreproc->enc_inuse |= (1 << idx);
		*module_name =
		    msm_enc_database.enc_info_list[idx].module_name;
		*queue_ids =
		    msm_enc_database.enc_info_list[idx].module_queueids;
		encid = msm_enc_database.enc_info_list[idx].module_encid;
	}

	if (!wakelock_init) {
		wake_lock_init(&audpre_wake_lock, WAKE_LOCK_SUSPEND, "audpre");
		wakelock_init = 1;
	}

	mutex_unlock(audpreproc->lock);
	return encid;
}
EXPORT_SYMBOL(audpreproc_aenc_alloc);

void audpreproc_aenc_free(int enc_id)
{
	struct audpreproc_state *audpreproc = &the_audpreproc_state;
	int idx;

	mutex_lock(audpreproc->lock);
	for (idx = 0; idx < msm_enc_database.num_enc; idx++) {
		if (msm_enc_database.enc_info_list[idx].module_encid ==
		    enc_id) {
			audpreproc->enc_inuse &= ~(1 << idx);
			break;
		}
	}
	mutex_unlock(audpreproc->lock);
	return;

}
EXPORT_SYMBOL(audpreproc_aenc_free);

int audpreproc_send_preproccmdqueue(void *cmd, unsigned len)
{
	return msm_adsp_write(the_audpreproc_state.mod,
				QDSP_uPAudPreProcCmdQueue, cmd, len);
}
EXPORT_SYMBOL(audpreproc_send_preproccmdqueue);

int audpreproc_send_audreccmdqueue(void *cmd, unsigned len)
{
	return msm_adsp_write(the_audpreproc_state.mod,
			      QDSP_uPAudPreProcAudRecCmdQueue, cmd, len);
}
EXPORT_SYMBOL(audpreproc_send_audreccmdqueue);


int audpreproc_dsp_set_agc(struct audpreproc_cmd_cfg_agc_params *agc,
				unsigned len)
{
	return msm_adsp_write(the_audpreproc_state.mod,
				QDSP_uPAudPreProcCmdQueue, agc, len);
}
EXPORT_SYMBOL(audpreproc_dsp_set_agc);

int audpreproc_dsp_set_agc2(struct audpreproc_cmd_cfg_agc_params_2 *agc2,
				unsigned len)
{
	return msm_adsp_write(the_audpreproc_state.mod,
				QDSP_uPAudPreProcCmdQueue, agc2, len);
}
EXPORT_SYMBOL(audpreproc_dsp_set_agc2);

int audpreproc_dsp_set_ns(struct audpreproc_cmd_cfg_ns_params *ns,
				unsigned len)
{
	return msm_adsp_write(the_audpreproc_state.mod,
				QDSP_uPAudPreProcCmdQueue, ns, len);
}
EXPORT_SYMBOL(audpreproc_dsp_set_ns);

int audpreproc_dsp_set_iir(
struct audpreproc_cmd_cfg_iir_tuning_filter_params *iir, unsigned len)
{
	return msm_adsp_write(the_audpreproc_state.mod,
				QDSP_uPAudPreProcCmdQueue, iir, len);
}
EXPORT_SYMBOL(audpreproc_dsp_set_iir);

int audpreproc_dsp_set_gain_tx(
		struct audpreproc_cmd_cfg_cal_gain *calib_gain_tx, unsigned len)
{
	return msm_adsp_write(the_audpreproc_state.mod,
			QDSP_uPAudPreProcCmdQueue, calib_gain_tx, len);
}
EXPORT_SYMBOL(audpreproc_dsp_set_gain_tx);
