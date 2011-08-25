/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
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
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include "msm_audio_mvs.h"

struct snd_soc_dai msm_mvs_dais[] = {
{
	.name = "MVS_CODEC_DAI",
	.playback = {
		.stream_name = "Playback",
		.channels_max = 2,
		.rates = (SNDRV_PCM_RATE_8000),
		.rate_min = 8000,
		.rate_max = 8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_max = 2,
		.rate_min = 8000,
		.rates = (SNDRV_PCM_RATE_8000),

		.formats =  SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = "MVS_CPU_DAI",
	.id = 0,
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = (SNDRV_PCM_RATE_8000),
		.rate_min = 8000,
		.rate_max = 8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rate_min = 8000,
		.rates = (SNDRV_PCM_RATE_8000),
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
};
EXPORT_SYMBOL_GPL(msm_mvs_dais);

int msm_mvs_pcm_probe(struct platform_device *devptr)
{
	struct snd_soc_codec *codec;
	int ret;

	struct snd_soc_device *socdev = platform_get_drvdata(devptr);
	MM_DBG("%s\n", __func__);

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	codec->name = "MSM-MVS-CARD";
	codec->owner = THIS_MODULE;
	socdev->card->codec = codec;
	mutex_init(&codec->mutex);

	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		MM_DBG("msm_mvs_soc: failed to create pcms\n");
		goto __nopcm;
	}

	return 0;
__nopcm:
	kfree(codec);
	return ret;
}

struct snd_soc_codec_device soc_codec_dev_msm_mvs = {
	.probe          = msm_mvs_pcm_probe,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_msm_mvs);


static int __init msm_dai_init(void)
{
	return  snd_soc_register_dais(msm_mvs_dais, ARRAY_SIZE(msm_mvs_dais));
}

static void __exit msm_dai_exit(void)
{
	snd_soc_unregister_dais(msm_mvs_dais, ARRAY_SIZE(msm_mvs_dais));
}

module_init(msm_dai_init);
module_exit(msm_dai_exit);

/* Module information */
MODULE_DESCRIPTION("MSM Codec/Cpu Dai driver");
MODULE_LICENSE("GPL v2");
