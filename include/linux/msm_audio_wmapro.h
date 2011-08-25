/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __MSM_AUDIO_WMAPRO_H
#define __MSM_AUDIO_WMAPRO_H

#define AUDIO_GET_WMAPRO_CONFIG  _IOR(AUDIO_IOCTL_MAGIC, \
	  (AUDIO_MAX_COMMON_IOCTL_NUM+0), unsigned)
#define AUDIO_SET_WMAPRO_CONFIG  _IOW(AUDIO_IOCTL_MAGIC, \
	  (AUDIO_MAX_COMMON_IOCTL_NUM+1), unsigned)

struct msm_audio_wmapro_config {
	unsigned short 	armdatareqthr;
	uint8_t         validbitspersample;
	uint8_t         numchannels;
	unsigned short  formattag;
	unsigned short  samplingrate;
	unsigned short  avgbytespersecond;
	unsigned short  asfpacketlength;
	unsigned short 	channelmask;
	unsigned short 	encodeopt;
	unsigned short	advancedencodeopt;
	uint32_t	advancedencodeopt2;
};
#endif /* __MSM_AUDIO_WMAPRO_H */
