/* msm_audio_wma.h
 *
 * Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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
 *
 */

#ifndef __MSM_AUDIO_WMA_H
#define __MSM_AUDIO_WMA_H

#define AUDIO_GET_WMA_CONFIG  _IOR(AUDIO_IOCTL_MAGIC, \
	  (AUDIO_MAX_COMMON_IOCTL_NUM+0), unsigned)
#define AUDIO_SET_WMA_CONFIG  _IOW(AUDIO_IOCTL_MAGIC, \
	  (AUDIO_MAX_COMMON_IOCTL_NUM+1), unsigned)

#define AUDIO_GET_WMA_CONFIG_V2  _IOR(AUDIO_IOCTL_MAGIC, \
	  (AUDIO_MAX_COMMON_IOCTL_NUM+2), struct msm_audio_wma_config_v2)
#define AUDIO_SET_WMA_CONFIG_V2  _IOW(AUDIO_IOCTL_MAGIC, \
	  (AUDIO_MAX_COMMON_IOCTL_NUM+3), struct msm_audio_wma_config_v2)

struct msm_audio_wma_config {
	unsigned short 	armdatareqthr;
	unsigned short 	channelsdecoded;
	unsigned short 	wmabytespersec;
	unsigned short	wmasamplingfreq;
	unsigned short	wmaencoderopts;
};

struct msm_audio_wma_config_v2 {
	unsigned short	format_tag;
	unsigned short	numchannels;
	uint32_t	samplingrate;
	uint32_t	avgbytespersecond;
	unsigned short	block_align;
	unsigned short  validbitspersample;
	uint32_t	channelmask;
	unsigned short	encodeopt;
};

#endif /* __MSM_AUDIO_WMA_H */
