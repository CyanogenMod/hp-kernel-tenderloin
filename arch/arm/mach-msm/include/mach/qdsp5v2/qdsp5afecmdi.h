/* Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
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
#ifndef __MACH_QDSP5_V2_QDSP5AFECMDI_H
#define __MACH_QDSP5_V2_QDSP5AFECMDI_H

#define QDSP5_DEVICE_mI2S_CODEC_RX 1     /* internal codec rx path  */
#define QDSP5_DEVICE_mI2S_CODEC_TX 2     /* internal codec tx path  */
#define QDSP5_DEVICE_AUX_CODEC_RX  3     /* external codec rx path  */
#define QDSP5_DEVICE_AUX_CODEC_TX  4     /* external codec tx path  */
#define QDSP5_DEVICE_mI2S_HDMI_RX  5     /* HDMI/FM block rx path   */
#define QDSP5_DEVICE_mI2S_HDMI_TX  6     /* HDMI/FM block tx path   */
#define QDSP5_DEVICE_ID_MAX        7

#define AFE_CMD_CODEC_CONFIG_CMD     0x1
#define AFE_CMD_CODEC_CONFIG_LEN sizeof(struct afe_cmd_codec_config)

struct afe_cmd_codec_config{
	uint16_t cmd_id;
	uint16_t device_id;
	uint16_t activity;
	uint16_t sample_rate;
	uint16_t channel_mode;
	uint16_t volume;
	uint16_t reserved;
} __attribute__ ((packed));

#define AFE_CMD_DEVICE_VOLUME_CTRL	0x2
#define AFE_CMD_DEVICE_VOLUME_CTRL_LEN \
		sizeof(struct afe_cmd_device_volume_ctrl)

struct afe_cmd_device_volume_ctrl {
	uint16_t cmd_id;
	uint16_t device_id;
	uint16_t device_volume;
	uint16_t reserved;
} __attribute__ ((packed));

#define AFE_CMD_AUX_CODEC_CONFIG_CMD 	0x3
#define AFE_CMD_AUX_CODEC_CONFIG_LEN sizeof(struct afe_cmd_aux_codec_config)

struct afe_cmd_aux_codec_config{
	uint16_t cmd_id;
	uint16_t dma_path_ctl;
	uint16_t pcm_ctl;
	uint16_t eight_khz_int_mode;
	uint16_t aux_codec_intf_ctl;
	uint16_t data_format_padding_info;
} __attribute__ ((packed));

#define AFE_CMD_FM_RX_ROUTING_CMD	0x6
#define AFE_CMD_FM_RX_ROUTING_LEN sizeof(struct afe_cmd_fm_codec_config)

struct afe_cmd_fm_codec_config{
	uint16_t cmd_id;
	uint16_t enable;
	uint16_t device_id;
} __attribute__ ((packed));

#define AFE_CMD_FM_PLAYBACK_VOLUME_CMD	0x8
#define AFE_CMD_FM_PLAYBACK_VOLUME_LEN sizeof(struct afe_cmd_fm_volume_config)

struct afe_cmd_fm_volume_config{
	uint16_t cmd_id;
	uint16_t volume;
	uint16_t reserved;
} __attribute__ ((packed));

#define AFE_CMD_FM_CALIBRATION_GAIN_CMD	0x11
#define AFE_CMD_FM_CALIBRATION_GAIN_LEN \
	sizeof(struct afe_cmd_fm_calibgain_config)

struct afe_cmd_fm_calibgain_config{
	uint16_t cmd_id;
	uint16_t device_id;
	uint16_t calibration_gain;
} __attribute__ ((packed));

#define AFE_CMD_LOOPBACK	0xD
#define AFE_CMD_LOOPBACK_LEN sizeof(struct afe_cmd_loopback)
#define AFE_LOOPBACK_ENABLE_COMMAND 0xFFFF
#define AFE_LOOPBACK_DISABLE_COMMAND 0x0000

struct afe_cmd_loopback {
	uint16_t cmd_id;
	uint16_t enable_flag;
	uint16_t reserved[2];
} __attribute__ ((packed));

#define AFE_CMD_CFG_RMC_PARAMS 0x12
#define AFE_CMD_CFG_RMC_LEN \
	sizeof(struct afe_cmd_cfg_rmc)

struct afe_cmd_cfg_rmc {
	unsigned short cmd_id;
	signed short   rmc_mode;
	unsigned short rmc_ipw_length_ms;
	unsigned short rmc_peak_length_ms;
	unsigned short rmc_init_pulse_length_ms;
	unsigned short rmc_total_int_length_ms;
	unsigned short rmc_rampupdn_length_ms;
	unsigned short rmc_delay_length_ms;
	unsigned short rmc_detect_start_threshdb;
	signed short   rmc_init_pulse_threshdb;
}  __attribute__((packed));

#endif
