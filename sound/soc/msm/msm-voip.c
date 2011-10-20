/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * All source code in this file is licensed under the following license except
 * where indicated.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org.
 */

#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/wakelock.h>
#include <linux/dma-mapping.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <mach/msm_rpcrouter.h>
#include <mach/qdsp6v2/apr_audio.h>
#include <mach/qdsp6v2/q6asm.h>
#include <mach/qdsp6v2/q6voice.h>
#include <mach/qdsp6v2/audio_dev_ctl.h>
#include "msm_audio_mvs.h"


static struct audio_voip_info_type audio_voip_info;

static void audio_mvs_process_ul_pkt(uint8_t *voc_pkt,
				uint32_t pkt_len,
				void *private_data);
static void audio_mvs_process_dl_pkt(uint8_t *voc_pkt,
				uint32_t *pkt_len,
				void *private_data);

struct msm_audio_mvs_frame {
	uint32_t frame_type;
	uint32_t len;
	uint8_t voc_pkt[MVS_MAX_VOC_PKT_SIZE];
};

struct audio_mvs_buf_node {
	struct list_head list;
	struct msm_audio_mvs_frame frame;
};

static struct snd_pcm_hardware mvs_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_INTERLEAVED,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.rates			= (SNDRV_PCM_RATE_8000),
	.rate_min		= 8000,
	.rate_max		= 8000,
	.channels_min		= 1,
	.channels_max		= 2,
	.buffer_bytes_max	= MVS_MAX_VOC_PKT_SIZE * VOIP_MAX_Q_LEN,
	.period_bytes_min	= MVS_MAX_VOC_PKT_SIZE,
	.period_bytes_max	= MVS_MAX_VOC_PKT_SIZE,
	.periods_min		= VOIP_MAX_Q_LEN,
	.periods_max		= VOIP_MAX_Q_LEN,
	.fifo_size		= 0,
};

int mvs_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{

	struct audio_voip_info_type *audio = &audio_voip_info;
	pr_debug("%s cmd = %d\n", __func__, cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (substream->stream ==  SNDRV_PCM_STREAM_PLAYBACK)
			audio->playback_start = 1;
		else
			audio->capture_start = 1;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		if (substream->stream ==  SNDRV_PCM_STREAM_PLAYBACK)
			audio->playback_start = 0;
		else
			audio->capture_start = 0;
		break;
	default:
		break;
	}
	return 0;
}

int mvs_pcm_close(struct snd_pcm_substream *substream)
{
	int rc = 0;
	struct audio_voip_info_type *audio = &audio_voip_info;

	pr_debug("%s\n", __func__);
	mutex_lock(&audio->lock);

	audio->instance--;
	wake_up(&audio->out_wait);

	if (substream->stream ==  SNDRV_PCM_STREAM_PLAYBACK)
		audio->playback_state = AUDIO_MVS_CLOSED;
	else if (substream->stream ==  SNDRV_PCM_STREAM_CAPTURE)
		audio->capture_state = AUDIO_MVS_CLOSED;

	if (!audio->instance) {
		/* Derigstering the callbacks with voice driver */
		voice_register_mvs_cb(NULL, NULL, audio);
	} else if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		voice_register_mvs_cb(audio_mvs_process_ul_pkt,
			NULL, audio);
	} else {
		voice_register_mvs_cb(NULL, audio_mvs_process_dl_pkt,
				audio);
	}

	mutex_unlock(&audio->lock);

	/* Release the IO buffers. */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		audio->in_write = 0;
		audio->in_read = 0;
		memset(audio->in[0].voc_pkt, 0,
			 MVS_MAX_VOC_PKT_SIZE * VOIP_MAX_Q_LEN);
		audio->playback_substream = NULL;
	} else {
		audio->out_write = 0;
		audio->out_read = 0;
		memset(audio->out[0].voc_pkt, 0,
			 MVS_MAX_VOC_PKT_SIZE * VOIP_MAX_Q_LEN);
		audio->capture_substream = NULL;
	}
	return rc;
}

int mvs_pcm_open(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct audio_voip_info_type *audio = &audio_voip_info;

	pr_debug("%s:audio = %x\n", __func__, (unsigned int) audio);
	mutex_lock(&audio->lock);

	if (audio->playback_substream == NULL ||
		audio->capture_substream == NULL) {
		if (substream->stream ==
			SNDRV_PCM_STREAM_PLAYBACK) {
			audio->playback_substream = substream;
			runtime->hw = mvs_pcm_hardware;
			audio_voip_info.in_read = 0;
			audio_voip_info.in_write = 0;
			if (audio->playback_state < AUDIO_MVS_OPENED)
				audio->playback_state = AUDIO_MVS_OPENED;
		} else if (substream->stream ==
			SNDRV_PCM_STREAM_CAPTURE) {
			audio->capture_substream = substream;
			runtime->hw = mvs_pcm_hardware;
			audio_voip_info.out_read = 0;
			audio_voip_info.out_write = 0;
			if (audio->capture_state < AUDIO_MVS_OPENED)
				audio->capture_state = AUDIO_MVS_OPENED;
		}
	} else {
		pr_debug("%s: ERROR:both substream assigned\n", __func__);
		ret  = -EPERM;
		goto err;
	}
	ret = snd_pcm_hw_constraint_integer(runtime,
			SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0) {
		pr_debug("%s:snd_pcm_hw_constraint_integer failed\n", __func__);
		goto err;
	}
	audio->instance++;

err:
	mutex_unlock(&audio->lock);
	return ret;
}

int mvs_pcm_playback_copy(struct snd_pcm_substream *substream, int a,
				 snd_pcm_uframes_t hwoff, void __user *buf,
				 snd_pcm_uframes_t frames)
{
	int rc = 0;
	int count = 0;
	int i = 0;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct audio_voip_info_type *audio = &audio_voip_info;
	uint32_t index;
	pr_debug("%s\n", __func__);

	rc = wait_event_timeout(audio->in_wait,
		(audio->in_write - audio->in_read <= VOIP_MAX_Q_LEN-1),
		1 * HZ);
	if (rc < 0) {
		pr_debug("%s: write was interrupted\n", __func__);
		return  -ERESTARTSYS;
	}

	if (audio->playback_state == AUDIO_MVS_ENABLED) {
		for (i = 0; i <= frames / MVS_MAX_VOC_PKT_SIZE; i++) {
			index = audio->in_write % VOIP_MAX_Q_LEN;
			count = frames_to_bytes(runtime, frames);

			pr_debug("%s:write index = %d buf = %x\n", __func__, index,(unsigned int) buf);
			rc = copy_from_user(audio->in[index].voc_pkt,
						buf + i * MVS_MAX_VOC_PKT_SIZE,
						MVS_MAX_VOC_PKT_SIZE);
			if (!rc) {
				audio->in[index].len = count;
				audio->in_write++;
			} else {
				pr_debug("%s:Copy from user returned %d\n",
						__func__, rc);
				rc = -EFAULT;
			}
		}
	} else {
		pr_debug("%s:Write performed in invalid state %d\n",
					__func__, audio->playback_state);
		rc = -EINVAL;
	}
	pr_debug("%s: ret= %d\n", __func__, rc);
	return rc;
}

int mvs_pcm_capture_copy(struct snd_pcm_substream *substream,
			int channel, snd_pcm_uframes_t hwoff,
			void __user *buf, snd_pcm_uframes_t frames)
{
	int rc = 0;
	int count = 0;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct audio_voip_info_type *audio = &audio_voip_info;
	uint32_t index = 0;

	pr_debug("%s\n", __func__);

	/* Ensure the driver has been enabled. */
	if (audio->capture_state != AUDIO_MVS_ENABLED) {
		pr_debug("%s:Read performed in invalid state %d\n",
				__func__, audio->capture_state);
		return -EPERM;
	}
	rc = wait_event_timeout(audio->out_wait,
		((audio->out_read < audio->out_write) ||
		(audio->capture_state == AUDIO_MVS_CLOSING) ||
		(audio->capture_state == AUDIO_MVS_CLOSED)),
		1 * HZ);

	if (rc < 0) {
		pr_debug("%s: Read was interrupted\n", __func__);
		return  -ERESTARTSYS;
	}

	if (audio->capture_state  == AUDIO_MVS_CLOSING
		|| audio->capture_state == AUDIO_MVS_CLOSED) {
		pr_debug("%s:EBUSY STATE\n", __func__);
		rc = -EBUSY;
	} else {
		count = frames_to_bytes(runtime, frames);
		index = audio->out_read % VOIP_MAX_Q_LEN;
		pr_debug("%s:index=%d\n", __func__, index);
		if (audio->out[index].len <= count) {
				rc = copy_to_user(buf,
				audio->out[index].voc_pkt,
				audio->out[index].len);
				if (rc) {
					pr_debug("%s:Copy to user %d\n",
							__func__, rc);
					rc = -EFAULT;
				} else
					audio->out_read++;
		} else {
			pr_debug("%s:returning ENOMEM\n", __func__);
			rc = -ENOMEM;
		}
	}
	return rc;
}

int mvs_pcm_copy(struct snd_pcm_substream *substream, int a,
			snd_pcm_uframes_t hwoff, void __user *buf,
			snd_pcm_uframes_t frames)
{
	int ret = 0;
	pr_debug("%s\n", __func__);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ret = mvs_pcm_playback_copy(substream, a, hwoff, buf, frames);
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		ret = mvs_pcm_capture_copy(substream, a, hwoff, buf, frames);
	return ret;
}

/* Capture path */
void audio_mvs_process_ul_pkt(uint8_t *voc_pkt,
				uint32_t pkt_len,
				void *private_data)
{
	struct audio_voip_info_type *audio = private_data;
	uint32_t index;
	static int i;

	if (audio->capture_substream == NULL)
		return;
	if (audio->capture_start) {
		index = audio->out_write % VOIP_MAX_Q_LEN;
		pr_debug("%s voc_pkt %p\n", __func__, voc_pkt);
		pr_debug("%s pkt_len %d\n", __func__, pkt_len);
		pr_debug("%s audio->out[%d].voc_pkt = %p\n", __func__, index, audio->out[index].voc_pkt);
		memcpy(audio->out[index].voc_pkt, voc_pkt, pkt_len);
		audio->out[index].len = pkt_len;
		i++;
		audio->pcm_capture_irq_pos += audio->pcm_count;
		snd_pcm_period_elapsed(audio->capture_substream);
		audio->out_write++;
		wake_up(&audio->out_wait);
	}
}

/* Playback path */
void audio_mvs_process_dl_pkt(uint8_t *voc_pkt,
				uint32_t *pkt_len,
				void *private_data)
{
	struct audio_voip_info_type *audio = private_data;
	uint32_t index;
	static int i;
	unsigned long flag;

	if ((audio == NULL) || (audio->playback_substream == NULL)) {
		pr_debug("%s: playback_substream is NULL\n", __func__);
		return;
	}
	spin_lock_irqsave(&audio_voip_info.write_dsp_lock, flag);
	if ((audio->in_write - audio->in_read >= 0)
		&& (audio->playback_start)) {
		index = audio->in_read % VOIP_MAX_Q_LEN;
		*pkt_len = audio->pcm_count;
		memcpy(voc_pkt, audio->in[index].voc_pkt, *pkt_len);
		audio->in_read++;
		wake_up(&audio->in_wait);
		i++;
		audio->pcm_playback_irq_pos += audio->pcm_count;
		if (!(i%2))
			snd_pcm_period_elapsed(audio->playback_substream);
		pr_debug("%s:read_index=%d\n", __func__, index);
	}
	spin_unlock_irqrestore(&audio_voip_info.write_dsp_lock, flag);
}

int mvs_pcm_prepare(struct snd_pcm_substream *substream)
{
	int rc = 0;
	struct audio_voip_info_type *prtd = &audio_voip_info;
	pr_debug("%s\n", __func__);

	mutex_lock(&prtd->prepare_lock);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (prtd->playback_state == AUDIO_MVS_ENABLED)
			goto enabled;
	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		if (prtd->capture_state == AUDIO_MVS_ENABLED)
			goto enabled;
	}

	if (prtd->instance == 2) {
		pr_debug("%s:Register ul/dl cb with voice driver\n",
			__func__);
		voice_register_mvs_cb(audio_mvs_process_ul_pkt,
				audio_mvs_process_dl_pkt,
				prtd);
	} else {
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			pr_debug("%s:Register dl cb with voice driver\n",
				__func__);
			voice_register_mvs_cb(NULL,
					audio_mvs_process_dl_pkt,
					prtd);
		} else {
			pr_debug("%s:Register ul cb with voice driver\n",
				__func__);
			voice_register_mvs_cb(audio_mvs_process_ul_pkt,
					NULL,
					prtd);
		}
	}
enabled:
	prtd->pcm_count = snd_pcm_lib_period_bytes(substream);
	pr_debug("%s:prtd->pcm_count:%d\n", __func__, prtd->pcm_count);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		prtd->pcm_playback_size = snd_pcm_lib_buffer_bytes(substream);
		pr_debug("%s:prtd->pcm_playback_size:%d\n",
			__func__, prtd->pcm_playback_size);
		prtd->playback_state = AUDIO_MVS_ENABLED;
		prtd->pcm_playback_irq_pos = 0;
		prtd->pcm_playback_buf_pos = 0;
		/* rate and channels are sent to audio driver */
	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		prtd->capture_state = AUDIO_MVS_ENABLED;
		prtd->pcm_capture_size  = snd_pcm_lib_buffer_bytes(substream);
		prtd->pcm_capture_count = snd_pcm_lib_period_bytes(substream);
		prtd->pcm_capture_irq_pos = 0;
		prtd->pcm_capture_buf_pos = 0;
	}
	mutex_unlock(&prtd->prepare_lock);
	return rc;
}

snd_pcm_uframes_t
mvs_pcm_playback_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct audio_voip_info_type *audio = &audio_voip_info;

	if (audio->pcm_playback_irq_pos >= audio->pcm_playback_size)
		audio->pcm_playback_irq_pos = 0;
	pr_debug("%s: pcm_irq_pos = %d\n", __func__, audio->pcm_playback_irq_pos);
	return bytes_to_frames(runtime, (audio->pcm_playback_irq_pos));
}

snd_pcm_uframes_t
mvs_pcm_capture_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct audio_voip_info_type *audio = &audio_voip_info;

	if (audio->pcm_capture_irq_pos >= audio->pcm_capture_size)
		audio->pcm_capture_irq_pos = 0;
	pr_debug("%s: pcm_irq_pos = %d\n", __func__, audio->pcm_capture_irq_pos);
	return bytes_to_frames(runtime, (audio->pcm_capture_irq_pos));
}

snd_pcm_uframes_t mvs_pcm_pointer(struct snd_pcm_substream *substream)
{
	snd_pcm_uframes_t ret = 0;
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ret = mvs_pcm_playback_pointer(substream);
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		ret = mvs_pcm_capture_pointer(substream);
	return ret;
}

int mvs_pcm_remove(struct platform_device *devptr)
{
	struct snd_soc_device *socdev = platform_get_drvdata(devptr);
	snd_soc_free_pcms(socdev);
	kfree(socdev->card->codec);
	return 0;
}

int mvs_pcm_new(struct snd_card *card,
			struct snd_soc_dai *codec_dai,
			struct snd_pcm *pcm)
{
	int   i, ret, offset = 0;
	struct snd_pcm_substream *substream = NULL;
	struct snd_dma_buffer *dma_buffer = NULL;

	pr_debug("%s\n", __func__);
	memset(&audio_voip_info, 0, sizeof(audio_voip_info));
	mutex_init(&audio_voip_info.lock);
	mutex_init(&audio_voip_info.prepare_lock);
	init_waitqueue_head(&audio_voip_info.out_wait);
	init_waitqueue_head(&audio_voip_info.in_wait);
	spin_lock_init(&audio_voip_info.write_dsp_lock);

#ifndef CONFIG_MFD_WM8994
	ret = snd_pcm_new_stream(pcm, SNDRV_PCM_STREAM_PLAYBACK, 1);
	if (ret)
		return ret;
	ret = snd_pcm_new_stream(pcm, SNDRV_PCM_STREAM_CAPTURE, 1);
	if (ret)
		return ret;
#endif

	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	substream = pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
	if (!substream) {
		pr_debug("%s:playback substream is NULL\n", __func__);
		return -ENOMEM;
	}

	dma_buffer = &substream->dma_buffer;
	dma_buffer->dev.type = SNDRV_DMA_TYPE_DEV;
	dma_buffer->dev.dev = card->dev;
	dma_buffer->private_data = NULL;
	dma_buffer->area = dma_alloc_coherent(card->dev,
				(MVS_MAX_VOC_PKT_SIZE * VOIP_MAX_Q_LEN),
				&dma_buffer->addr, GFP_KERNEL);
	if (!dma_buffer->area) {
		pr_debug("%s:MSM VOIP dma_alloc failed\n", __func__);
		return -ENOMEM;
	}
	dma_buffer->bytes = MVS_MAX_VOC_PKT_SIZE * VOIP_MAX_Q_LEN;
	memset(dma_buffer->area, 0, MVS_MAX_VOC_PKT_SIZE * VOIP_MAX_Q_LEN);
	audio_voip_info.in_read = 0;
	audio_voip_info.in_write = 0;
	audio_voip_info.out_read = 0;
	audio_voip_info.out_write = 0;
	for (i = 0; i < VOIP_MAX_Q_LEN; i++) {
		audio_voip_info.in[i].voc_pkt =
		dma_buffer->area + offset;
		offset = offset + MVS_MAX_VOC_PKT_SIZE;
		pr_info("audio_voip_info.in[%d].voc_pkt = %p\n", i, audio_voip_info.in[i].voc_pkt);
	}
	substream = pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream;
	if (!substream) {
		pr_debug("%s:capture substream is NULL\n", __func__);
		return -ENOMEM;
	}

	dma_buffer = &substream->dma_buffer;
	dma_buffer->dev.type = SNDRV_DMA_TYPE_DEV;
	dma_buffer->dev.dev = card->dev;
	dma_buffer->private_data = NULL;
	dma_buffer->area = dma_alloc_coherent(card->dev,
					(MVS_MAX_VOC_PKT_SIZE * VOIP_MAX_Q_LEN),
					&dma_buffer->addr, GFP_KERNEL);
	if (!dma_buffer->area) {
		pr_debug("%s:MSM VOIP dma_alloc failed\n", __func__);
		return -ENOMEM;
	}
	memset(dma_buffer->area, 0, MVS_MAX_VOC_PKT_SIZE * VOIP_MAX_Q_LEN);
	offset = 0;
	dma_buffer->bytes = MVS_MAX_VOC_PKT_SIZE * VOIP_MAX_Q_LEN;
	for (i = 0; i < VOIP_MAX_Q_LEN; i++) {
		audio_voip_info.out[i].voc_pkt =
		dma_buffer->area + offset;
		offset = offset + MVS_MAX_VOC_PKT_SIZE;
		pr_info("audio_voip_info.out[%d].voc_pkt = %p\n", i, audio_voip_info.out[i].voc_pkt);
	}
	audio_voip_info.playback_substream = NULL;
	audio_voip_info.capture_substream = NULL;

	return 0;
}

void mvs_pcm_free_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_coherent(pcm->card->dev, buf->bytes,
				      buf->area, buf->addr);
		buf->area = NULL;
	}
}
