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
#ifndef _MACH_QDSP5_V2_MI2S_H
#define _MACH_QDSP5_V2_MI2S_H

#define WT_16_BIT 0
#define WT_24_BIT 1
#define WT_32_BIT 2
#define WT_MAX 4

enum mi2s_ret_enum_type {
	MI2S_FALSE = 0,
	MI2S_TRUE
};

#define MI2S_CHAN_MONO_RAW 0
#define MI2S_CHAN_MONO_PACKED 1
#define MI2S_CHAN_STEREO 2
#define MI2S_CHAN_4CHANNELS 3
#define MI2S_CHAN_6CHANNELS 4
#define MI2S_CHAN_8CHANNELS 5
#define MI2S_CHAN_MAX_OUTBOUND_CHANNELS MI2S__CHAN_8CHANNELS

#define MI2S_SD_0    0x01
#define MI2S_SD_1    0x02
#define MI2S_SD_2    0x04
#define MI2S_SD_3    0x08

#define MI2S_SD_LINE_MASK    (MI2S_SD_0 | MI2S_SD_1 | MI2S_SD_2 |  MI2S_SD_3)

bool mi2s_set_hdmi_output_path(uint8_t channels, uint8_t size,
				uint8_t sd_line);

bool mi2s_set_hdmi_input_path(uint8_t channels, uint8_t size, uint8_t sd_line);

bool mi2s_set_codec_output_path(uint8_t channels, uint8_t size);

bool mi2s_set_codec_input_path(uint8_t channels, uint8_t size);

#endif /* #ifndef MI2S_H */
