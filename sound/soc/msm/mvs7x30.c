/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <asm/dma.h>
#include <linux/dma-mapping.h>
#include <linux/msm_audio.h>

#include "msm_audio_mvs.h"
#include <asm/mach-types.h>
#include <mach/debug_mm.h>

static struct platform_device *msm_audio_snd_device;

static struct snd_soc_dai_link msm_mvs_dai = {
	.name = "ASOC_MVS",
	.stream_name = "ASOC_MVS",
	.codec_dai = &msm_mvs_dais[0],
	.cpu_dai = &msm_mvs_dais[1],
};

struct snd_soc_card snd_soc_mvs_card_msm = {
	.name		= "msm-mvs-audio",
	.dai_link	= &msm_mvs_dai,
	.num_links	= 1,
	.platform	= &msm_mvs_soc_platform,
};

/* msm_audio audio subsystem */
static struct snd_soc_device msm_mvs_audio_snd_devdata = {
	.card = &snd_soc_mvs_card_msm,
	.codec_dev = &soc_codec_dev_msm_mvs,
};


static int __init mvs_audio_init(void)
{
	int ret;

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
