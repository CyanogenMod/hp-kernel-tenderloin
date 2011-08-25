/*
 * evrc audio input device
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (C) 2008 HTC Corporation
 * Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/dma-mapping.h>
#include <linux/msm_audio_qcp.h>

#include <asm/atomic.h>
#include <asm/ioctls.h>

#include <mach/msm_adsp.h>
#include <mach/qdsp5v2/qdsp5audreccmdi.h>
#include <mach/qdsp5v2/qdsp5audrecmsg.h>
#include <mach/qdsp5v2/audpreproc.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>
#include <mach/debug_mm.h>

/* FRAME_NUM must be a power of two */
#define FRAME_NUM		(8)
#define FRAME_SIZE		(22 * 2) /* 36 bytes data */
#define DMASZ 			(FRAME_SIZE * FRAME_NUM)

struct buffer {
	void *data;
	uint32_t size;
	uint32_t read;
	uint32_t addr;
};

struct audio_in {
	struct buffer in[FRAME_NUM];

	spinlock_t dsp_lock;

	atomic_t in_bytes;
	atomic_t in_samples;

	struct mutex lock;
	struct mutex read_lock;
	wait_queue_head_t wait;
	wait_queue_head_t wait_enable;

	struct msm_adsp_module *audrec;

	/* configuration to use on next enable */
	uint32_t buffer_size; /* Frame size (36 bytes) */
	uint32_t enc_type;

	struct msm_audio_evrc_enc_config cfg;

	uint32_t dsp_cnt;
	uint32_t in_head; /* next buffer dsp will write */
	uint32_t in_tail; /* next buffer read() will read */
	uint32_t in_count; /* number of buffers available to read() */

	const char *module_name;
	unsigned queue_ids;
	uint16_t enc_id;

	uint16_t source; /* Encoding source bit mask */
	uint32_t device_events;
	uint32_t in_call;
	uint32_t dev_cnt;
	int voice_state;
	spinlock_t dev_lock;

	/* data allocated for various buffers */
	char *data;
	dma_addr_t phys;

	int opened;
	int enabled;
	int running;
	int stopped; /* set when stopped, cleared on flush */
};

struct audio_frame {
	uint16_t frame_count_lsw;
	uint16_t frame_count_msw;
	uint16_t frame_length;
	uint16_t erased_pcm;
	unsigned char raw_bitstream[]; /* samples */
} __attribute__((packed));

static struct audio_in the_audio_evrc_in;

/* Audrec Queue command sent macro's */
#define audrec_send_bitstreamqueue(audio, cmd, len) \
	msm_adsp_write(audio->audrec, ((audio->queue_ids & 0xFFFF0000) >> 16),\
			cmd, len)

#define audrec_send_audrecqueue(audio, cmd, len) \
	msm_adsp_write(audio->audrec, (audio->queue_ids & 0x0000FFFF),\
			cmd, len)

/* DSP command send functions */
static int audevrc_in_enc_config(struct audio_in *audio, int enable);
static int audevrc_in_param_config(struct audio_in *audio);
static int audevrc_in_mem_config(struct audio_in *audio);
static int audevrc_in_record_config(struct audio_in *audio, int enable);
static int audevrc_dsp_read_buffer(struct audio_in *audio, uint32_t read_cnt);

static void audevrc_in_get_dsp_frames(struct audio_in *audio);

static void audevrc_in_flush(struct audio_in *audio);

static void evrc_in_listener(u32 evt_id, union auddev_evt_data *evt_payload,
				void *private_data)
{
	struct audio_in *audio = (struct audio_in *) private_data;
	unsigned long flags;

	MM_DBG("evt_id = 0x%8x\n", evt_id);
	switch (evt_id) {
	case AUDDEV_EVT_DEV_RDY:
		MM_DBG("AUDDEV_EVT_DEV_RDY\n");
		spin_lock_irqsave(&audio->dev_lock, flags);
		audio->dev_cnt++;
		if (!audio->in_call)
			audio->source |= (0x1 << evt_payload->routing_id);
		spin_unlock_irqrestore(&audio->dev_lock, flags);

		if ((audio->running == 1) && (audio->enabled == 1))
			audevrc_in_record_config(audio, 1);

		break;
	case AUDDEV_EVT_DEV_RLS: {
		MM_DBG("AUDDEV_EVT_DEV_RLS\n");
		spin_lock_irqsave(&audio->dev_lock, flags);
		audio->dev_cnt--;
		if (!audio->in_call)
			audio->source &= ~(0x1 << evt_payload->routing_id);
		spin_unlock_irqrestore(&audio->dev_lock, flags);

		if ((!audio->running) || (!audio->enabled))
			break;

		/* Turn of as per source */
		if (audio->source)
			audevrc_in_record_config(audio, 1);
		else
			/* Turn off all */
			audevrc_in_record_config(audio, 0);

		break;
	}
	case AUDDEV_EVT_VOICE_STATE_CHG: {
		MM_DBG("AUDDEV_EVT_VOICE_STATE_CHG, state = %d\n",
				evt_payload->voice_state);
		audio->voice_state = evt_payload->voice_state;
		if (audio->in_call && audio->running) {
			if (audio->voice_state == VOICE_STATE_INCALL)
				audevrc_in_record_config(audio, 1);
			else if (audio->voice_state == VOICE_STATE_OFFCALL) {
				audevrc_in_record_config(audio, 0);
				wake_up(&audio->wait);
			}
		}

		break;
	}
	default:
		MM_ERR("wrong event %d\n", evt_id);
		break;
	}
}

/* ------------------- dsp preproc event handler--------------------- */
static void audpreproc_dsp_event(void *data, unsigned id,  void *msg)
{
	struct audio_in *audio = data;

	switch (id) {
	case AUDPREPROC_ERROR_MSG: {
		struct audpreproc_err_msg *err_msg = msg;

		MM_ERR("ERROR_MSG: stream id %d err idx %d\n",
		err_msg->stream_id, err_msg->aud_preproc_err_idx);
		/* Error case */
		wake_up(&audio->wait_enable);
		break;
	}
	case AUDPREPROC_CMD_CFG_DONE_MSG: {
		MM_DBG("CMD_CFG_DONE_MSG \n");
		break;
	}
	case AUDPREPROC_CMD_ENC_CFG_DONE_MSG: {
		struct audpreproc_cmd_enc_cfg_done_msg *enc_cfg_msg = msg;

		MM_DBG("CMD_ENC_CFG_DONE_MSG: stream id %d enc type \
			0x%8x\n", enc_cfg_msg->stream_id,
			enc_cfg_msg->rec_enc_type);
		/* Encoder enable success */
		if (enc_cfg_msg->rec_enc_type & ENCODE_ENABLE)
			audevrc_in_param_config(audio);
		else { /* Encoder disable success */
			audio->running = 0;
			audevrc_in_record_config(audio, 0);
		}
		break;
	}
	case AUDPREPROC_CMD_ENC_PARAM_CFG_DONE_MSG: {
		MM_DBG("CMD_ENC_PARAM_CFG_DONE_MSG \n");
		audevrc_in_mem_config(audio);
		break;
	}
	case AUDPREPROC_AFE_CMD_AUDIO_RECORD_CFG_DONE_MSG: {
		MM_DBG("AFE_CMD_AUDIO_RECORD_CFG_DONE_MSG \n");
		wake_up(&audio->wait_enable);
		break;
	}
	default:
		MM_ERR("Unknown Event id %d\n", id);
	}
}

/* ------------------- dsp audrec event handler--------------------- */
static void audrec_dsp_event(void *data, unsigned id, size_t len,
			    void (*getevent)(void *ptr, size_t len))
{
	struct audio_in *audio = data;

	switch (id) {
	case AUDREC_CMD_MEM_CFG_DONE_MSG: {
		MM_DBG("CMD_MEM_CFG_DONE MSG DONE\n");
		audio->running = 1;
		if ((!audio->in_call && (audio->dev_cnt > 0)) ||
			(audio->in_call &&
				(audio->voice_state == VOICE_STATE_INCALL)))
			audevrc_in_record_config(audio, 1);
		break;
	}
	case AUDREC_FATAL_ERR_MSG: {
		struct audrec_fatal_err_msg fatal_err_msg;

		getevent(&fatal_err_msg, AUDREC_FATAL_ERR_MSG_LEN);
		MM_ERR("FATAL_ERR_MSG: err id %d\n",
				fatal_err_msg.audrec_err_id);
		/* Error stop the encoder */
		audio->stopped = 1;
		wake_up(&audio->wait);
		break;
	}
	case AUDREC_UP_PACKET_READY_MSG: {
		struct audrec_up_pkt_ready_msg pkt_ready_msg;

		getevent(&pkt_ready_msg, AUDREC_UP_PACKET_READY_MSG_LEN);
		MM_DBG("UP_PACKET_READY_MSG: write cnt lsw  %d \
		write cnt msw %d read cnt lsw %d  read cnt msw %d \n",\
		pkt_ready_msg.audrec_packet_write_cnt_lsw, \
		pkt_ready_msg.audrec_packet_write_cnt_msw, \
		pkt_ready_msg.audrec_up_prev_read_cnt_lsw, \
		pkt_ready_msg.audrec_up_prev_read_cnt_msw);

		audevrc_in_get_dsp_frames(audio);
		break;
	}
	default:
		MM_ERR("Unknown Event id %d\n", id);
	}
}

static void audevrc_in_get_dsp_frames(struct audio_in *audio)
{
	struct audio_frame *frame;
	uint32_t index;
	unsigned long flags;

	index = audio->in_head;

	frame = (void *) (((char *)audio->in[index].data) - \
			 sizeof(*frame));

	spin_lock_irqsave(&audio->dsp_lock, flags);
	audio->in[index].size = frame->frame_length;

	/* statistics of read */
	atomic_add(audio->in[index].size, &audio->in_bytes);
	atomic_add(1, &audio->in_samples);

	audio->in_head = (audio->in_head + 1) & (FRAME_NUM - 1);

	/* If overflow, move the tail index foward. */
	if (audio->in_head == audio->in_tail) {
		audio->in_tail = (audio->in_tail + 1) & (FRAME_NUM - 1);
		MM_ERR("in_count = %d\n", audio->in_count);
	} else
		audio->in_count++;

	audevrc_dsp_read_buffer(audio, audio->dsp_cnt++);
	spin_unlock_irqrestore(&audio->dsp_lock, flags);

	wake_up(&audio->wait);
}
struct msm_adsp_ops audrec_evrc_adsp_ops = {
	.event = audrec_dsp_event,
};

static int audevrc_in_enc_config(struct audio_in *audio, int enable)
{
	struct audpreproc_audrec_cmd_enc_cfg cmd;

	memset(&cmd, 0, sizeof(cmd));
	cmd.cmd_id = AUDPREPROC_AUDREC_CMD_ENC_CFG;
	cmd.stream_id = audio->enc_id;

	if (enable)
		cmd.audrec_enc_type = audio->enc_type | ENCODE_ENABLE;
	else
		cmd.audrec_enc_type &= ~(ENCODE_ENABLE);

	return audpreproc_send_audreccmdqueue(&cmd, sizeof(cmd));
}

static int audevrc_in_param_config(struct audio_in *audio)
{
	struct audpreproc_audrec_cmd_parm_cfg_evrc cmd;

	memset(&cmd, 0, sizeof(cmd));
	cmd.common.cmd_id = AUDPREPROC_AUDREC_CMD_PARAM_CFG;
	cmd.common.stream_id = audio->enc_id;

	cmd.enc_min_rate = audio->cfg.min_bit_rate;
	cmd.enc_max_rate = audio->cfg.max_bit_rate;
	cmd.rate_modulation_cmd = 0;  /* Default set to 0 */

	return audpreproc_send_audreccmdqueue(&cmd, sizeof(cmd));
}

/* To Do: msm_snddev_route_enc(audio->enc_id); */
static int audevrc_in_record_config(struct audio_in *audio, int enable)
{
	struct audpreproc_afe_cmd_audio_record_cfg cmd;

	memset(&cmd, 0, sizeof(cmd));
	cmd.cmd_id = AUDPREPROC_AFE_CMD_AUDIO_RECORD_CFG;
	cmd.stream_id = audio->enc_id;
	if (enable)
		cmd.destination_activity = AUDIO_RECORDING_TURN_ON;
	else
		cmd.destination_activity = AUDIO_RECORDING_TURN_OFF;

	cmd.source_mix_mask = audio->source;

	return audpreproc_send_audreccmdqueue(&cmd, sizeof(cmd));
}

static int audevrc_in_mem_config(struct audio_in *audio)
{
	struct audrec_cmd_arecmem_cfg cmd;
	uint16_t *data = (void *) audio->data;
	int n;

	memset(&cmd, 0, sizeof(cmd));
	cmd.cmd_id = AUDREC_CMD_MEM_CFG_CMD;
	cmd.audrec_up_pkt_intm_count = 1;
	cmd.audrec_ext_pkt_start_addr_msw = audio->phys >> 16;
	cmd.audrec_ext_pkt_start_addr_lsw = audio->phys;
	cmd.audrec_ext_pkt_buf_number = FRAME_NUM;

	/* prepare buffer pointers:
	 * 36 bytes evrc packet + 4 halfword header
	 */
	for (n = 0; n < FRAME_NUM; n++) {
		audio->in[n].data = data + 4;
		data += (FRAME_SIZE/2); /* word increment */
		MM_DBG("0x%8x\n", (int)(audio->in[n].data - 8));
	}

	return audrec_send_audrecqueue(audio, &cmd, sizeof(cmd));
}

static int audevrc_dsp_read_buffer(struct audio_in *audio, uint32_t read_cnt)
{
	struct up_audrec_packet_ext_ptr cmd;

	memset(&cmd, 0, sizeof(cmd));
	cmd.cmd_id = UP_AUDREC_PACKET_EXT_PTR;
	cmd.audrec_up_curr_read_count_msw = read_cnt >> 16;
	cmd.audrec_up_curr_read_count_lsw = read_cnt;

	return audrec_send_bitstreamqueue(audio, &cmd, sizeof(cmd));
}

/* must be called with audio->lock held */
static int audevrc_in_enable(struct audio_in *audio)
{
	if (audio->enabled)
		return 0;

	if (audpreproc_enable(audio->enc_id, &audpreproc_dsp_event, audio)) {
		MM_ERR("msm_adsp_enable(audpreproc) failed\n");
		return -ENODEV;
	}

	if (msm_adsp_enable(audio->audrec)) {
		MM_ERR("msm_adsp_enable(audrec) failed\n");
		audpreproc_disable(audio->enc_id, audio);
		return -ENODEV;
	}
	audio->enabled = 1;
	audevrc_in_enc_config(audio, 1);

	return 0;
}

/* must be called with audio->lock held */
static int audevrc_in_disable(struct audio_in *audio)
{
	if (audio->enabled) {
		audio->enabled = 0;
		audevrc_in_enc_config(audio, 0);
		wake_up(&audio->wait);
		wait_event_interruptible_timeout(audio->wait_enable,
				audio->running == 0, 1*HZ);
		msm_adsp_disable(audio->audrec);
		audpreproc_disable(audio->enc_id, audio);
	}
	return 0;
}

static void audevrc_in_flush(struct audio_in *audio)
{
	int i;

	audio->dsp_cnt = 0;
	audio->in_head = 0;
	audio->in_tail = 0;
	audio->in_count = 0;
	for (i = 0; i < FRAME_NUM; i++) {
		audio->in[i].size = 0;
		audio->in[i].read = 0;
	}
	MM_DBG("in_bytes %d\n", atomic_read(&audio->in_bytes));
	MM_DBG("in_samples %d\n", atomic_read(&audio->in_samples));
	atomic_set(&audio->in_bytes, 0);
	atomic_set(&audio->in_samples, 0);
}

/* ------------------- device --------------------- */
static long audevrc_in_ioctl(struct file *file,
				unsigned int cmd, unsigned long arg)
{
	struct audio_in *audio = file->private_data;
	int rc = 0;

	MM_DBG("\n");
	if (cmd == AUDIO_GET_STATS) {
		struct msm_audio_stats stats;
		stats.byte_count = atomic_read(&audio->in_bytes);
		stats.sample_count = atomic_read(&audio->in_samples);
		if (copy_to_user((void *) arg, &stats, sizeof(stats)))
			return -EFAULT;
		return rc;
	}

	mutex_lock(&audio->lock);
	switch (cmd) {
	case AUDIO_START: {
		uint32_t freq;
		freq = 48000;
		MM_DBG("AUDIO_START\n");
		if (audio->in_call && (audio->voice_state !=
				VOICE_STATE_INCALL)) {
			rc = -EPERM;
			break;
		}
		rc = msm_snddev_request_freq(&freq, audio->enc_id,
					SNDDEV_CAP_TX, AUDDEV_CLNT_ENC);
		MM_DBG("sample rate configured %d\n", freq);
		if (rc < 0) {
			MM_DBG(" Sample rate can not be set, return code %d\n",
								 rc);
			msm_snddev_withdraw_freq(audio->enc_id,
					SNDDEV_CAP_TX, AUDDEV_CLNT_ENC);
			MM_DBG("msm_snddev_withdraw_freq\n");
			break;
		}
		rc = audevrc_in_enable(audio);
		if (!rc) {
			rc =
			wait_event_interruptible_timeout(audio->wait_enable,
				audio->running != 0, 1*HZ);
			MM_DBG("state %d rc = %d\n", audio->running, rc);

			if (audio->running == 0)
				rc = -ENODEV;
			else
				rc = 0;
		}
		break;
	}
	case AUDIO_STOP: {
		rc = audevrc_in_disable(audio);
		rc = msm_snddev_withdraw_freq(audio->enc_id,
					SNDDEV_CAP_TX, AUDDEV_CLNT_ENC);
		MM_DBG("msm_snddev_withdraw_freq\n");
		audio->stopped = 1;
		break;
	}
	case AUDIO_FLUSH: {
		if (audio->stopped) {
			/* Make sure we're stopped and we wake any threads
			 * that might be blocked holding the read_lock.
			 * While audio->stopped read threads will always
			 * exit immediately.
			 */
			wake_up(&audio->wait);
			mutex_lock(&audio->read_lock);
			audevrc_in_flush(audio);
			mutex_unlock(&audio->read_lock);
		}
		break;
	}
	case AUDIO_SET_STREAM_CONFIG: {
		struct msm_audio_stream_config cfg;
		if (copy_from_user(&cfg, (void *) arg, sizeof(cfg))) {
			rc = -EFAULT;
			break;
		}
		/* Allow only single frame */
		if (cfg.buffer_size != (FRAME_SIZE - 8))
			rc = -EINVAL;
		else
			audio->buffer_size = cfg.buffer_size;
		break;
	}
	case AUDIO_GET_STREAM_CONFIG: {
		struct msm_audio_stream_config cfg;
		memset(&cfg, 0, sizeof(cfg));
		cfg.buffer_size = audio->buffer_size;
		cfg.buffer_count = FRAME_NUM;
		if (copy_to_user((void *) arg, &cfg, sizeof(cfg)))
			rc = -EFAULT;
		break;
	}
	case AUDIO_GET_EVRC_ENC_CONFIG: {
		if (copy_to_user((void *) arg, &audio->cfg, sizeof(audio->cfg)))
			rc = -EFAULT;
		break;
	}
	case AUDIO_SET_EVRC_ENC_CONFIG: {
		struct msm_audio_evrc_enc_config cfg;
		if (copy_from_user(&cfg, (void *) arg, sizeof(cfg))) {
			rc = -EFAULT;
			break;
		}
		MM_DBG("0X%8x, 0x%8x, 0x%8x\n", cfg.min_bit_rate,
				cfg.max_bit_rate, cfg.cdma_rate);
		if (cfg.min_bit_rate > CDMA_RATE_FULL || \
				 cfg.min_bit_rate < CDMA_RATE_EIGHTH) {
			MM_ERR("invalid min bitrate\n");
			rc = -EFAULT;
			break;
		}
		if (cfg.max_bit_rate > CDMA_RATE_FULL || \
				cfg.max_bit_rate < CDMA_RATE_EIGHTH) {
			MM_ERR("invalid max bitrate\n");
			rc = -EFAULT;
			break;
		}
		/* Recording Does not support Erase and Blank */
		if (cfg.cdma_rate > CDMA_RATE_FULL ||
			cfg.cdma_rate < CDMA_RATE_EIGHTH) {
			MM_ERR("invalid qcelp cdma rate\n");
			rc = -EFAULT;
			break;
		}
		memcpy(&audio->cfg, &cfg, sizeof(cfg));
		break;
	}
	case AUDIO_SET_INCALL: {
		struct msm_voicerec_mode cfg;
		unsigned long flags;
		if (copy_from_user(&cfg, (void *) arg, sizeof(cfg))) {
			rc = -EFAULT;
			break;
		}
		if (cfg.rec_mode != VOC_REC_BOTH &&
			cfg.rec_mode != VOC_REC_UPLINK &&
			cfg.rec_mode != VOC_REC_DOWNLINK) {
			MM_ERR("invalid rec_mode\n");
			rc = -EINVAL;
			break;
		} else {
			spin_lock_irqsave(&audio->dev_lock, flags);
			if (cfg.rec_mode == VOC_REC_UPLINK)
				audio->source = VOICE_UL_SOURCE_MIX_MASK;
			else if (cfg.rec_mode == VOC_REC_DOWNLINK)
				audio->source = VOICE_DL_SOURCE_MIX_MASK;
			else
				audio->source = VOICE_DL_SOURCE_MIX_MASK |
						VOICE_UL_SOURCE_MIX_MASK ;
			audio->in_call = 1;
			spin_unlock_irqrestore(&audio->dev_lock, flags);
		}
		break;
	}
	case AUDIO_GET_SESSION_ID: {
		if (copy_to_user((void *) arg, &audio->enc_id,
			sizeof(unsigned short))) {
			rc = -EFAULT;
		}
		break;
	}
	default:
		rc = -EINVAL;
	}
	mutex_unlock(&audio->lock);
	return rc;
}

static ssize_t audevrc_in_read(struct file *file,
				char __user *buf,
				size_t count, loff_t *pos)
{
	struct audio_in *audio = file->private_data;
	unsigned long flags;
	const char __user *start = buf;
	void *data;
	uint32_t index;
	uint32_t size;
	int rc = 0;

	mutex_lock(&audio->read_lock);
	while (count > 0) {
		rc = wait_event_interruptible(
			audio->wait, (audio->in_count > 0) || audio->stopped
			|| (audio->in_call && audio->running &&
				(audio->voice_state == VOICE_STATE_OFFCALL)));
		if (rc < 0)
			break;

		if (!audio->in_count) {
			if (audio->stopped)  {
				rc = 0;/* End of File */
				break;
			} else if (audio->in_call && audio->running &&
				(audio->voice_state == VOICE_STATE_OFFCALL)) {
				MM_DBG("Not Permitted Voice Terminated\n");
				rc = -EPERM; /* Voice Call stopped */
				break;
			}
		}

		index = audio->in_tail;
		data = (uint8_t *) audio->in[index].data;
		size = audio->in[index].size;
		if (count >= size) {
			if (copy_to_user(buf, data, size)) {
				rc = -EFAULT;
				break;
			}
			spin_lock_irqsave(&audio->dsp_lock, flags);
			if (index != audio->in_tail) {
				/* overrun -- data is
				 * invalid and we need to retry */
				spin_unlock_irqrestore(&audio->dsp_lock, flags);
				continue;
			}
			audio->in[index].size = 0;
			audio->in_tail = (audio->in_tail + 1) & (FRAME_NUM - 1);
			audio->in_count--;
			spin_unlock_irqrestore(&audio->dsp_lock, flags);
			count -= size;
			buf += size;
		} else {
			MM_ERR("short read\n");
			break;
		}
	}
	mutex_unlock(&audio->read_lock);

	if (buf > start)
		return buf - start;

	return rc;
}

static ssize_t audevrc_in_write(struct file *file,
				const char __user *buf,
				size_t count, loff_t *pos)
{
	return -EINVAL;
}

static int audevrc_in_release(struct inode *inode, struct file *file)
{
	struct audio_in *audio = file->private_data;

	mutex_lock(&audio->lock);
	audio->in_call = 0;
	/* with draw frequency for session
	   incase not stopped the driver */
	msm_snddev_withdraw_freq(audio->enc_id, SNDDEV_CAP_TX,
					AUDDEV_CLNT_ENC);
	auddev_unregister_evt_listner(AUDDEV_CLNT_ENC, audio->enc_id);
	audevrc_in_disable(audio);
	audevrc_in_flush(audio);
	msm_adsp_put(audio->audrec);
	audpreproc_aenc_free(audio->enc_id);
	audio->audrec = NULL;
	audio->opened = 0;
	mutex_unlock(&audio->lock);
	return 0;
}


static int audevrc_in_open(struct inode *inode, struct file *file)
{
	struct audio_in *audio = &the_audio_evrc_in;
	int rc;
	int encid;

	mutex_lock(&audio->lock);
	if (audio->opened) {
		rc = -EBUSY;
		goto done;
	}

	/* Settings will be re-config at AUDIO_SET_CONFIG,
	 * but at least we need to have initial config
	 */
	audio->buffer_size = (FRAME_SIZE - 8);
	audio->enc_type = ENC_TYPE_EVRC;
	audio->cfg.cdma_rate = CDMA_RATE_FULL;
	audio->cfg.min_bit_rate = CDMA_RATE_FULL;
	audio->cfg.max_bit_rate = CDMA_RATE_FULL;

	encid = audpreproc_aenc_alloc(audio->enc_type, &audio->module_name,
			&audio->queue_ids);
	if (encid < 0) {
		MM_ERR("No free encoder available\n");
		rc = -ENODEV;
		goto done;
	}
	audio->enc_id = encid;

	rc = msm_adsp_get(audio->module_name, &audio->audrec,
			   &audrec_evrc_adsp_ops, audio);

	if (rc) {
		audpreproc_aenc_free(audio->enc_id);
		goto done;
	}

	audio->stopped = 0;
	audio->source = 0;

	audevrc_in_flush(audio);

	audio->device_events = AUDDEV_EVT_DEV_RDY | AUDDEV_EVT_DEV_RLS |
				AUDDEV_EVT_VOICE_STATE_CHG;

	audio->voice_state = msm_get_voice_state();
	rc = auddev_register_evt_listner(audio->device_events,
					AUDDEV_CLNT_ENC, audio->enc_id,
					evrc_in_listener, (void *) audio);
	if (rc) {
		MM_ERR("failed to register device event listener\n");
		goto evt_error;
	}
	file->private_data = audio;
	audio->opened = 1;
done:
	mutex_unlock(&audio->lock);
	return rc;
evt_error:
	msm_adsp_put(audio->audrec);
	audpreproc_aenc_free(audio->enc_id);
	mutex_unlock(&audio->lock);
	return rc;
}

static const struct file_operations audio_in_fops = {
	.owner		= THIS_MODULE,
	.open		= audevrc_in_open,
	.release	= audevrc_in_release,
	.read		= audevrc_in_read,
	.write		= audevrc_in_write,
	.unlocked_ioctl	= audevrc_in_ioctl,
};

struct miscdevice audio_evrc_in_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_evrc_in",
	.fops	= &audio_in_fops,
};

static int __init audevrc_in_init(void)
{
	the_audio_evrc_in.data = dma_alloc_coherent(NULL, DMASZ,
				       &the_audio_evrc_in.phys, GFP_KERNEL);
	MM_DBG("Memory addr = 0x%8x  Phy addr = 0x%8x ---- \n", \
		(int) the_audio_evrc_in.data, (int) the_audio_evrc_in.phys);

	if (!the_audio_evrc_in.data) {
		MM_ERR("Unable to allocate DMA buffer\n");
		return -ENOMEM;
	}
	mutex_init(&the_audio_evrc_in.lock);
	mutex_init(&the_audio_evrc_in.read_lock);
	spin_lock_init(&the_audio_evrc_in.dsp_lock);
	spin_lock_init(&the_audio_evrc_in.dev_lock);
	init_waitqueue_head(&the_audio_evrc_in.wait);
	init_waitqueue_head(&the_audio_evrc_in.wait_enable);
	return misc_register(&audio_evrc_in_misc);
}

device_initcall(audevrc_in_init);
