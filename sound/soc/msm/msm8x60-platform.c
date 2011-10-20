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

#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <asm/dma.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include "msm8x60-platform.h"

#define PCM_MEDIA_PLAYBACK_DEVICE 0
#define PCM_MEDIA_CAPTURE_DEVICE 1
#define PCM_MVS_PLAYBACK_DEVICE 2
#define PCM_MVS_CAPTURE_DEVICE 3

static int msm_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	pr_debug("%s\n", __func__);
	if (substream->pcm->device >= PCM_MVS_PLAYBACK_DEVICE)
		return mvs_pcm_trigger(substream, cmd);
	else
		return msm_dsp_trigger(substream, cmd);
}

static int msm_pcm_open(struct snd_pcm_substream *substream)
{
	pr_debug("%s\n", __func__);
	if (substream->pcm->device >= PCM_MVS_PLAYBACK_DEVICE)
		return mvs_pcm_open(substream);
	else
		return msm_dsp_open(substream);
}

static int msm_pcm_copy(struct snd_pcm_substream *substream, int a,
	 snd_pcm_uframes_t hwoff, void __user *buf, snd_pcm_uframes_t frames)
{
	pr_debug("%s\n", __func__);
	if (substream->pcm->device >= PCM_MVS_PLAYBACK_DEVICE)
		return mvs_pcm_copy(substream, a, hwoff, buf, frames);
	else
		return msm_dsp_copy(substream, a, hwoff, buf, frames);
}

static int msm_pcm_close(struct snd_pcm_substream *substream)
{
	pr_debug("%s\n", __func__);
	if (substream->pcm->device >= PCM_MVS_PLAYBACK_DEVICE)
		return mvs_pcm_close(substream);
	else
		return msm_dsp_close(substream);
}

static int msm_pcm_prepare(struct snd_pcm_substream *substream)
{
	pr_debug("%s\n", __func__);
	if (substream->pcm->device >= PCM_MVS_PLAYBACK_DEVICE)
		return mvs_pcm_prepare(substream);
	else
		return msm_dsp_prepare(substream);
}

static snd_pcm_uframes_t msm_pcm_pointer(struct snd_pcm_substream *substream)
{
	pr_debug("%s\n", __func__);
	if (substream->pcm->device >= PCM_MVS_PLAYBACK_DEVICE)
		return mvs_pcm_pointer(substream);
	else
		return msm_dsp_pointer(substream);
}

int msm_pcm_mmap(struct snd_pcm_substream *substream,
				struct vm_area_struct *vma)
{
	pr_debug("%s\n", __func__);
	if (substream->pcm->device >= PCM_MVS_PLAYBACK_DEVICE)
		return -EINVAL;
	else
		return msm_dsp_mmap(substream, vma);
}

int msm_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	pr_debug("%s\n", __func__);
	if (substream->pcm->device <= PCM_MEDIA_CAPTURE_DEVICE)
		return msm_dsp_hw_params(substream, params);
	else
		return 0;
}

static struct snd_pcm_ops msm_pcm_ops = {
	.open		= msm_pcm_open,
	.copy		= msm_pcm_copy,
	.hw_params	= msm_pcm_hw_params,
	.close		= msm_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.prepare	= msm_pcm_prepare,
	.trigger	= msm_pcm_trigger,
	.pointer	= msm_pcm_pointer,
	.mmap		= msm_pcm_mmap,
};

static int msm_pcm_remove(struct platform_device *devptr)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int msm_pcm_new(struct snd_card *card,
			struct snd_soc_dai *codec_dai,
			struct snd_pcm *pcm)
{
	pr_debug("%s\n", __func__);
	if (pcm->device == PCM_MVS_PLAYBACK_DEVICE)
		mvs_pcm_new(card, codec_dai, pcm);
	else if (pcm->device == PCM_MEDIA_PLAYBACK_DEVICE)
		msm_dsp_new(card, codec_dai, pcm);

#ifndef CONFIG_MFD_WM8994
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &msm_pcm_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &msm_pcm_ops);
#endif

	return 0;
}

struct snd_soc_platform msm_soc_platform = {
	.name		= "msm-audio",
	.remove		= msm_pcm_remove,
	.pcm_ops	= &msm_pcm_ops,
	.pcm_new	= msm_pcm_new,
};
EXPORT_SYMBOL(msm_soc_platform);

static int __init msm_soc_platform_init(void)
{
	return snd_soc_register_platform(&msm_soc_platform);
}
module_init(msm_soc_platform_init);

static void __exit msm_soc_platform_exit(void)
{
	snd_soc_unregister_platform(&msm_soc_platform);
}
module_exit(msm_soc_platform_exit);

MODULE_DESCRIPTION("PCM module platform driver");
MODULE_LICENSE("GPL v2");
