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
#include <linux/dma-mapping.h>
#include <linux/msm_audio.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <asm/dma.h>
#include <asm/mach-types.h>
#include <mach/debug_mm.h>
#include <mach/qdsp6v2/q6voice.h>

#include "msm_audio_mvs.h"

static struct platform_device *msm_audio_snd_device;
static int voc_path;

#define MVS_AMR_MODE_UNDEF	17

/* New mixer control for ALSA VOIP driver */
static int msm_voice_path_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int msm_voice_path_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = voc_path; /* Default modem */
	return 0;
}

static int msm_voice_path_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	voc_path = ucontrol->value.integer.value[0];
	pr_debug("%s:Setting vcoder path -> %d\n", __func__, voc_path);

	voice_set_voc_path_full(voc_path);
	/* MVS_AMR_MODE_UNDEF = 17 */
	voice_config_vocoder(VSS_MEDIA_ID_PCM_NB, MVS_AMR_MODE_UNDEF,
				VSS_NETWORK_ID_VOIP_NB);
	return 0;
}

#define MSM_EXT(xname, fp_info, fp_get, fp_put, addr) { \
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE, \
	.name = xname, \
	.info = fp_info,\
	.get = fp_get, .put = fp_put, \
	.private_value = addr, \
}


struct snd_kcontrol_new snd_msm_mvs_controls[] = {
	MSM_EXT("VocPath", msm_voice_path_info,
		msm_voice_path_get, msm_voice_path_put, 0),
};


static int msm_mvs_dai_init(struct snd_soc_codec *codec)
{
	unsigned int idx;
	int err = 0;
	struct snd_card *card = codec->card;

	for (idx = 0; idx < ARRAY_SIZE(snd_msm_mvs_controls); idx++) {
		err = snd_ctl_add(card,	snd_ctl_new1(&snd_msm_mvs_controls[idx],
					NULL));
		if (err < 0)
			pr_err("%s:ERR adding ctl\n", __func__);
	}
	return err;
}

static struct snd_soc_dai_link msm_mvs_dai = {
	.name = "ASOC_MVS",
	.stream_name = "ASOC_MVS",
	.codec_dai = &msm_mvs_dais[0],
	.cpu_dai = &msm_mvs_dais[1],
	.init	= msm_mvs_dai_init,
};

struct snd_soc_card snd_soc_voip_card_msm = {
	.name		= "msm-voip",
	.dai_link	= &msm_mvs_dai,
	.num_links	= 1,
	.platform	= &msm_voip_soc_platform,
};

/* msm_audio audio subsystem */
static struct snd_soc_device msm_mvs_audio_snd_devdata = {
	.card = &snd_soc_voip_card_msm,
	.codec_dev = &soc_codec_dev_msm_mvs,
};

static int __init mvs_audio_init(void)
{
	int ret;
	pr_debug("%s\n", __func__);
	msm_audio_snd_device =	platform_device_alloc("soc-audio", 1);

	if (!msm_audio_snd_device)
		return -ENOMEM;

	platform_set_drvdata(msm_audio_snd_device, &msm_mvs_audio_snd_devdata);
	msm_mvs_audio_snd_devdata.dev = &msm_audio_snd_device->dev;
	ret = platform_device_add(msm_audio_snd_device);
	if (ret) {
		platform_device_put(msm_audio_snd_device);
		return ret;
	}
	return ret;
}

static void __exit mvs_audio_exit(void)
{
	platform_device_unregister(msm_audio_snd_device);
}

module_init(mvs_audio_init);
module_exit(mvs_audio_exit);

MODULE_DESCRIPTION("PCM module");
MODULE_LICENSE("GPL v2");
