
/*
 *
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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
 *
 */

#ifndef _APR_AUDIO_H_
#define _APR_AUDIO_H_

/* ASM opcodes without APR payloads*/
#include "apr.h"


/*
 * Audio Front End (AFE)
 */

/* Port ID */
enum {
	PRIMARY_I2S_RX = 0,
	PRIMARY_I2S_TX = 1,
	PCM_RX = 2,
	PCM_TX = 3,
	SECONDARY_I2S_RX = 4,
	SECONDARY_I2S_TX = 5,
	MI2S_RX = 6,
	RSVD_1 = 7,
	HDMI_RX = 8,
	RSVD_2 = 9,
	RSVD_3 = 10,
	DIGI_MIC_TX = 11,
	AFE_MAX_PORTS ,
};

#define AFE_PORT_CMD_START 0x000100ca
struct afe_port_start_command {
	struct apr_hdr hdr;
	u16 port_id;
	u16 gain;		/* Q13 */
	u32 sample_rate;	/* 8 , 16, 48khz */
} __attribute__ ((packed));

#define AFE_PORT_CMD_STOP 0x000100cb
struct afe_port_stop_command {
	struct apr_hdr hdr;
	u16 port_id;
	u16 reserved;
} __attribute__ ((packed));

#define AFE_PORT_CMD_APPLY_GAIN 0x000100cc
struct afe_port_gain_command {
	struct apr_hdr hdr;
	u16 port_id;
	u16 gain; 		/* Q13 */
} __attribute__ ((packed));

#define AFE_PORT_CMD_SIDETONE_CTL 0x000100cd
struct afe_port_sidetone_command {
	struct apr_hdr hdr;
	u16 tx_port_id;		/* Primary i2s rx = 0 */
				/* PCM rx = 2 */
				/* Secondary i2s rx = 4 */
				/* Mi2S rx = 6 */
				/* HDMI rx = 8 */
	u16 rx_port_id;		/* Primary i2s tx = 1 */
				/* PCM tx = 3 */
				/* Secondary i2s tx = 5 */
				/* Mi2s tx = 7 */
				/* Digital mic tx = 11 */
	u16 gain;		/* Q13 */
	u16 enable;		/* 1 = enable, 0 = disable */
} __attribute__ ((packed));

#define AFE_PORT_CMD_LOOPBACK 0x000100ce
struct afe_loopback_command {
	struct apr_hdr hdr;
	u16 tx_port_id;		/* Primary i2s rx = 0 */
				/* PCM rx = 2 */
				/* Secondary i2s rx = 4 */
				/* Mi2S rx = 6 */
				/* HDMI rx = 8 */
	u16 rx_port_id;		/* Primary i2s tx = 1 */
				/* PCM tx = 3 */
				/* Secondary i2s tx = 5 */
				/* Mi2s tx = 7 */
				/* Digital mic tx = 11 */
	u16 mode;		/* Default -1, DSP will conver
					the tx to rx format */
	u16 enable;		/* 1 = enable, 0 = disable */
} __attribute__ ((packed));

#define AFE_PSEUDOPORT_CMD_START 0x000100cf
struct afe_pseudoport_start_command {
	struct apr_hdr hdr;
	u16 port_id;		/* Pseudo Port 1 = 0x8000 */
				/* Pseudo Port 2 = 0x8001 */
				/* Pseudo Port 3 = 0x8002 */
	u16 timing;		/* FTRT = 0 , AVTimer = 1, */
} __attribute__ ((packed));

#define AFE_PSEUDOPORT_CMD_STOP 0x000100d0
struct afe_pseudoport_stop_command {
	struct apr_hdr hdr;
	u16 port_id;		/* Pseudo Port 1 = 0x8000 */
				/* Pseudo Port 2 = 0x8001 */
				/* Pseudo Port 3 = 0x8002 */
	u16 reserved;
} __attribute__ ((packed));

#define AFE_CMD_GET_ACTIVE_PORTS 0x000100d1


#define AFE_CMD_GET_ACTIVE_HANDLES_FOR_PORT 0x000100d2
struct afe_get_active_handles_command {
	struct apr_hdr hdr;
	u16 port_id;
	u16 reserved;
} __attribute__ ((packed));

struct afe_port_pcm_cfg {
	u16	port_id;
	u16	mode;	/* PCM (short sync) = 0, AUXPCM (long sync) = 1 */
	u16	sync;	/* external = 0 , internal = 1 */
	u16	frame;	/* 8 bpf = 0 */
			/* 16 bpf = 1 */
			/* 32 bpf = 2 */
			/* 64 bpf = 3 */
			/* 128 bpf = 4 */
			/* 256 bpf = 5 */
	u16     quant;
	u16	slot;	/* Slot for PCM stream , 0 - 31 */
	u16	data;	/* 0, PCM block is the only master */
			/* 1, PCM block is shares to driver data out signal */
			/*    other master                                  */
	u16	reserved;
} __attribute__ ((packed));

struct afe_port_mi2s_cfg {
	u16	port_id;
	u16	bitwidth;	/* 16,24,32 */
	u16	line;		/* i2s_sd0 = 1 */
				/* i2s_sd1 = 2 */
				/* i2s_sd2 = 3 */
				/* i2s_sd3 = 4 */
				/* i2s_quad01 = 5 */
				/* i2s_quad23 = 6 */
				/* i2s_6chs = 7 */
				/* i2s_8chs = 8 */
	u16	channel;	/* i2s mono = 0 */
				/* i2s mono right = 1 */
				/* i2s mono left = 2 */
				/* i2s stereo = 3 */
	u16	ws;		/* 0, word select signal from external source */
				/* 1, word select signal from internal source */
	u16	reserved;
} __attribute__ ((packed));



#define AFE_PORT_AUDIO_IF_CONFIG 0x000100d3
struct afe_audioif_config_command {
	struct apr_hdr hdr;
	union {
		struct afe_port_pcm_cfg		pcm;
		struct afe_port_mi2s_cfg	mi2s;
	} __attribute__((packed)) port;
} __attribute__ ((packed));



#define AFE_TEST_CODEC_LOOPBACK_CTL 0x000100d5
struct afe_codec_loopback_command {
	u16	port_inf;	/* Primary i2s = 0 */
				/* PCM = 2 */
				/* Secondary i2s = 4 */
				/* Mi2s = 6 */
	u16	enable;		/* 0, disable. 1, enable */
} __attribute__ ((packed));


#define AFE_EVENT_GET_ACTIVE_PORTS 0x00010100
struct afe_get_active_ports_rsp {
	u16	num_ports;
	u16	port_id;
} __attribute__ ((packed));


#define AFE_EVENT_GET_ACTIVE_HANDLES 0x00010102
struct afe_get_active_handles_rsp {
	u16	port_id;
	u16	num_handles;
	u16	mode;		/* 0, voice rx */
				/* 1, voice tx */
				/* 2, audio rx */
				/* 3, audio tx */
	u16	handle;
} __attribute__ ((packed));

#define ADM_MAX_COPPS 5

#define ADM_SERVICE_CMD_GET_COPP_HANDLES                 0x00010300
struct adm_get_copp_handles_command {
	struct apr_hdr hdr;
} __attribute__ ((packed));

#define ADM_CMD_MATRIX_MAP_ROUTINGS                      0x00010301
struct adm_routings_session {
	u16 id;
	u16 num_copps;
	u16 copp_id[ADM_MAX_COPPS];
} __attribute__ ((packed));

struct adm_routings_command {
	struct apr_hdr hdr;
	u32 path; /* 0 = Rx, 1 Tx */
	u32 num_sessions;
	struct adm_routings_session sessions[8];
} __attribute__ ((packed));


#define ADM_CMD_MATRIX_RAMP_GAINS                        0x00010302
struct adm_ramp_gain {
	struct apr_hdr hdr;
	u16 session_id;
	u16 copp_id;
	u16 initial_gain;
	u16 gain_increment;
	u16 ramp_duration;
	u16 reserved;
} __attribute__ ((packed));

struct adm_ramp_gains_command {
	struct apr_hdr hdr;
	u32 id;
	u32 num_gains;
	struct adm_ramp_gain gains[ADM_MAX_COPPS];
} __attribute__ ((packed));


#define ADM_CMD_COPP_OPEN                                0x00010304
struct adm_copp_open_command {
	struct apr_hdr hdr;
	u16 flags;
	u16 endpoint_id;
	u32 topology_id;
} __attribute__ ((packed));

#define ADM_CMD_COPP_CLOSE                               0x00010305


#define DEFAULT_TOPOLOGY		0x00010be4

struct asm_pp_param_data_hdr {
	u32 module_id;
	u32 param_id;
	u16 param_size;
	u16 updated_flag;
} __attribute__ ((packed));


#define VOLUME_CONTROL_MODULE_ID	0x00010bfe
#define MASTER_GAIN_PARAM_ID		0x00010bff
#define L_R_CHANNEL_GAIN_PARAM_ID	0x00010c00
#define MUTE_CONFIG_PARAM_ID 0x00010c01

#define IIR_FILTER_ENABLE_PARAM_ID 0x00010c03
#define IIR_FILTER_PREGAIN_PARAM_ID 0x00010c04
#define IIR_FILTER_CONFIG_PARAM_ID 0x00010c05

#define MBADRC_MODULE_ID 0x00010c06
#define MBADRC_ENABLE_PARAM_ID 0x00010c07
#define MBADRC_CONFIG_PARAM_ID 0x00010c08

struct asm_pp_params {
	struct asm_pp_param_data_hdr hdr;
} __attribute__ ((packed));

#define ADM_CMD_SET_PARAMS                               0x00010306
struct adm_set_params_command {
	struct apr_hdr hdr;
	struct asm_pp_params param;
} __attribute__ ((packed));


#define ADM_CMD_TAP_COPP_PCM                             0x00010307
struct adm_tap_copp_pcm_command {
	struct apr_hdr hdr;
} __attribute__ ((packed));


/* QDSP6 to Client messages
*/
#define ADM_SERVICE_CMDRSP_GET_COPP_HANDLES              0x00010308
struct adm_get_copp_handles_respond {
	struct apr_hdr hdr;
	u32 handles;
	u32 copp_id;
} __attribute__ ((packed));

#define ADM_CMDRSP_COPP_OPEN                             0x0001030A
struct adm_copp_open_respond {
	u32 status;
	u16 copp_id;
	u16 reserved;
} __attribute__ ((packed));

#define ASM_STREAM_PRIORITY_NORMAL	0
#define ASM_STREAM_PRIORITY_LOW		1
#define ASM_STREAM_PRIORITY_HIGH	2
#define ASM_STREAM_PRIORITY_RESERVED	3

#define ASM_END_POINT_DEVICE_MATRIX	0
#define ASM_END_POINT_STREAM		1

#define ASM_STREAM_CMD_CLOSE                             0x00010BCD
#define ASM_STREAM_CMD_FLUSH                             0x00010BCE
#define ASM_STREAM_CMD_SET_PP_PARAMS                     0x00010BCF
#define ASM_STREAM_CMD_GET_PP_PARAMS                     0x00010BD0
#define ASM_STREAM_CMDRSP_GET_PP_PARAMS                  0x00010BD1
#define ASM_SESSION_CMD_PAUSE                            0x00010BD3
#define ASM_SESSION_CMD_GET_SESSION_TIME                 0x00010BD4
#define ASM_DATA_CMD_EOS                                 0x00010BDB
#define ASM_DATA_EVENT_EOS                               0x00010BDD

#define ASM_SERVICE_CMD_GET_STREAM_HANDLES               0x00010C0B
#define ASM_STREAM_CMD_FLUSH_READBUFS                    0x00010C09

#define ASM_SESSION_EVENT_RX_UNDERFLOW			 0x00010C17
#define ASM_SESSION_EVENT_TX_OVERFLOW 			 0x00010C18
#define ASM_SERVICE_CMD_GET_WALLCLOCK_TIME               0x00010C19
#define ASM_DATA_CMDRSP_EOS                              0x00010C1C

/* ASM Data structures */

/* common declarations */
struct asm_pcm_cfg {
	u16 ch_cfg;
	u16 bits_per_sample;
	u32 sample_rate;
	u16 is_signed;
	u16 interleaved;
};

struct asm_adpcm_cfg {
	u16 ch_cfg;
	u16 bits_per_sample;
	u32 sample_rate;
	u32 block_size;
};

struct asm_yadpcm_cfg {
	u16 ch_cfg;
	u16 bits_per_sample;
	u32 sample_rate;
};

struct asm_midi_cfg {
	u32 nMode;
};

struct asm_wma_cfg {
	u16 format_tag;
	u16 ch_cfg;
	u32 sample_rate;
	u32 ave_bytes_per_sec;
	u16 block_align;
	u16 valid_bits_per_sample;
	u32 ch_mask;
	u16 encode_opt;
	u16 adv_encode_opt;
	u16 adv_encode_opt2;
	u32 drc_peak_ref;
	u32 drc_peak_target;
	u32 drc_ave_ref;
	u32 drc_ave_target;
};

struct asm_aac_cfg {
	u16 format;
	u16 aot;
	u16 ep_config;
	u16 section_data_resilience;
	u16 scalefactor_data_resilience;
	u16 spectral_data_resilience;
	u16 sbr_on;
	u16 sbr_ps_on;
	u16 ch_cfg;
	u16 reserved;
	u32 sample_rate;
};

struct asm_flac_cfg {
	u16 stream_info_present;
	u16 min_blk_size;
	u16 max_blk_size;
	u16 ch_cfg;
	u16 sample_size;
	u16 sample_rate;
	u16 md5_sum;
	u32 ext_sample_rate;
	u32 min_frame_size;
	u32 max_frame_size;
};

struct asm_vorbis_cfg {
	u32 ch_cfg;
	u32 bit_rate;
	u32 min_bit_rate;
	u32 max_bit_rate;
	u16 bit_depth_pcm_sample;
	u16 bit_stream_format;
};

struct asm_aac_read_cfg {
	u32 bitrate;
	u32 enc_mode;
	u16 format;
	u16 ch_cfg;
	u32 sample_rate;
};

struct asm_amrnb_read_cfg {
	u16 mode;
	u16 dtx_mode;
	u16 min_rate;
	u16 max_rate;
};

struct asm_evrc_read_cfg {
	u16 min_rate;
	u16 max_rate;
};

struct asm_qcelp13_read_cfg {
	u16 min_rate;
	u16 max_rate;
};

struct asm_sbc_read_cfg {
	u32 subband;
	u32 block_len;
	u32 ch_mode;
	u32 alloc_method;
	u32 bit_rate;
	u32 sample_rate;
};

struct asm_sbc_bitrate {
	u32 bitrate;
};

struct asm_aac_immed_decode {
	u16 mode;
	u16 padding;
};

struct asm_frame_meta_info {
	u32 offset_to_frame;
	u32 encoded_pcm_samples;
	u32 msw_ts;
	u32 lsw_ts;
};

/* Stream level commands */
#define ASM_STREAM_CMD_OPEN_READ                         0x00010BCB
struct asm_stream_cmd_open_read {
	struct apr_hdr hdr;
	u32            uMode;
	u32            src_endpoint;
	u32            frames_per_buf;
	u32            pre_proc_top;
	u32            format;
	u32            cfg_size;
	union {
		struct asm_pcm_cfg          pcm_cfg;
		struct asm_aac_read_cfg     aac_cfg;
		struct asm_amrnb_read_cfg   amr_cfg;
		struct asm_evrc_read_cfg    evrc_cfg;
		struct asm_qcelp13_read_cfg qcelp13_cfg;
		struct asm_sbc_read_cfg     sbc_cfg;
	} __attribute__((packed)) read_cfg;
} __attribute__((packed));

/* Supported formats */
#define LINEAR_PCM   0x00010BE5
#define DTMF         0x00010BE6
#define ADPCM        0x00010BE7
#define YADPCM       0x00010BE8
#define MP3          0x00010BE9
#define MPEG4_AAC    0x00010BEA
#define AMRNB_FS     0x00010BEB
#define V13K_FS      0x00010BED
#define EVRC_FS      0x00010BEE
#define EVRCB_FS     0x00010BEF
#define EVRCWB_FS    0x00010BF0
#define MIDI         0x00010BF1
#define SBC          0x00010BF2
#define WMA_V10PRO   0x00010BF3
#define WMA_V9       0x00010BF4
#define AMR_WB_PLUS  0x00010BF5
#define AC3_DECODER  0x00010BF6
#define G711_ALAW_FS 0x00010BF7
#define G711_MLAW_FS 0x00010BF8
#define G711_PCM_FS  0x00010BF9

#define ASM_STREAM_CMD_OPEN_WRITE                        0x00010BCA
struct asm_stream_cmd_open_write {
	struct apr_hdr hdr;
	u32            uMode;
	u16            sink_endpoint;
	u16            stream_handle;
	u32            post_proc_top;
	u32            format;
	u32            cfg_size;
	union {
		struct asm_pcm_cfg    pcm_cfg;
		struct asm_adpcm_cfg  adpcm_cfg;
		struct asm_yadpcm_cfg yadpcm_cfg;
		struct asm_midi_cfg   midi_cfg;
		struct asm_wma_cfg    wma_cfg;
		struct asm_aac_cfg    aac_cfg;
		struct asm_flac_cfg   flac_cfg;
		struct asm_vorbis_cfg vorbis_cfg;
	} __attribute__((packed)) write_cfg;
} __attribute__((packed));

#define ASM_STREAM_CMD_OPEN_READWRITE                    0x00010BCC
/* pcm in; aac, amrnb, evrc, qcelp13, sbc out */
struct asm_stream_cmd_open_read_pcmwrite {
	struct apr_hdr     hdr;
	u32                uMode;
	u32                post_proc_top;
	u32                sample_rate;
	u32                write_format;
	u32                write_cfg_size;
	u32                frames_per_buffer;
	u32                read_format;
	u32                read_cfg_size;
	struct asm_pcm_cfg pcm_write_cfg;
	union {
		struct asm_pcm_cfg          pcm_cfg;
		struct asm_aac_read_cfg     aac_cfg;
		struct asm_amrnb_read_cfg   amr_cfg;
		struct asm_evrc_read_cfg    evrc_cfg;
		struct asm_qcelp13_read_cfg qcelp13_cfg;
		struct asm_sbc_read_cfg     sbc_cfg;
	} __attribute__((packed)) read_cfg;
} __attribute__((packed));

/* adpcm in;pcm, aac, amrnb, evrc, qcelp13, sbc out */
struct asm_stream_cmd_open_read_adpcmwrite {
	struct apr_hdr       hdr;
	u32                  uMode;
	u32                  post_proc_top;
	u32                  sample_rate;
	u32                  write_format;
	u32                  write_cfg_size;
	u32                  frames_per_buffer;
	u32                  read_format;
	u32                  read_cfg_size;
	struct asm_adpcm_cfg adpcm_write_cfg;
	union {
		struct asm_pcm_cfg          pcm_cfg;
		struct asm_aac_read_cfg     aac_cfg;
		struct asm_amrnb_read_cfg   amr_cfg;
		struct asm_evrc_read_cfg    evrc_cfg;
		struct asm_qcelp13_read_cfg qcelp13_cfg;
		struct asm_sbc_read_cfg     sbc_cfg;
	} __attribute__((packed)) read_cfg;
} __attribute__((packed));

/* yadpcm in:pcm, aac, amrnb, evrc, qcelp13, sbc out */
struct asm_stream_cmd_open_read_yadpcmwrite {
	struct apr_hdr        hdr;
	u32                   uMode;
	u32                   post_proc_top;
	u32                   sample_rate;
	u32                   write_format;
	u32                   write_cfg_size;
	u32                   frames_per_buffer;
	u32                   read_format;
	u32                   read_cfg_size;
	struct asm_yadpcm_cfg yadpcm_write_cfg;
	union {
		struct asm_pcm_cfg          pcm_cfg;
		struct asm_aac_read_cfg     aac_cfg;
		struct asm_amrnb_read_cfg   amr_cfg;
		struct asm_evrc_read_cfg    evrc_cfg;
		struct asm_qcelp13_read_cfg qcelp13_cfg;
		struct asm_sbc_read_cfg     sbc_cfg;
	} __attribute__((packed)) read_cfg;
} __attribute__((packed));

/* midi in;pcm, aac, amrnb, evrc, qcelp13, sbc out */
struct asm_stream_cmd_open_read_midiwrite {
	struct apr_hdr      hdr;
	u32                 uMode;
	u32                 post_proc_top;
	u32                 sample_rate;
	u32                 write_format;
	u32                 write_cfg_size;
	u32                 frames_per_buffer;
	u32                 read_format;
	u32                 read_cfg_size;
	struct asm_midi_cfg midi_write_cfg;
	union {
		struct asm_pcm_cfg          pcm_cfg;
		struct asm_aac_read_cfg     aac_cfg;
		struct asm_amrnb_read_cfg   amr_cfg;
		struct asm_evrc_read_cfg    evrc_cfg;
		struct asm_qcelp13_read_cfg qcelp13_cfg;
		struct asm_sbc_read_cfg     sbc_cfg;
	} __attribute__((packed)) read_cfg;
} __attribute__((packed));

/* wma in: pcm, aac, amrnb, evrc, qcelp13, sbc out */
struct asm_stream_cmd_open_read_wmawrite {
	struct apr_hdr     hdr;
	u32                uMode;
	u32                post_proc_top;
	u32                sample_rate;
	u32                write_format;
	u32                write_cfg_size;
	u32                frames_per_buffer;
	u32                read_format;
	u32                read_cfg_size;
	struct asm_wma_cfg wma_write_cfg;
	union {
		struct asm_pcm_cfg          pcm_cfg;
		struct asm_aac_read_cfg     aac_cfg;
		struct asm_amrnb_read_cfg   amr_cfg;
		struct asm_evrc_read_cfg    evrc_cfg;
		struct asm_qcelp13_read_cfg qcelp13_cfg;
		struct asm_sbc_read_cfg     sbc_cfg;
	} __attribute__((packed)) read_cfg;
} __attribute__((packed));

/* aac in; pcm, aac, amrnb, evrc, qcelp13, sbc out */
struct asm_stream_cmd_open_read_aacwrite {
	struct apr_hdr     hdr;
	u32                uMode;
	u32                post_proc_top;
	u32                sample_rate;
	u32                write_format;
	u32                write_cfg_size;
	u32                frames_per_buffer;
	u32                read_format;
	u32                read_cfg_size;
	struct asm_aac_cfg aac_write_cfg;
	union {
		struct asm_pcm_cfg          pcm_cfg;
		struct asm_aac_read_cfg     aac_cfg;
		struct asm_amrnb_read_cfg   amr_cfg;
		struct asm_evrc_read_cfg    evrc_cfg;
		struct asm_qcelp13_read_cfg qcelp13_cfg;
		struct asm_sbc_read_cfg     sbc_cfg;
	} __attribute__((packed)) read_cfg;
} __attribute__((packed));

/* flac in; pcm, aac, amrnb, evrc, qcelp13, sbc out */
struct asm_stream_cmd_open_read_flacwrite {
	struct apr_hdr      hdr;
	u32                 uMode;
	u32                 post_proc_top;
	u32                 sample_rate;
	u32                 write_format;
	u32                 write_cfg_size;
	u32                 frames_per_buffer;
	u32                 read_format;
	u32                 read_cfg_size;
	struct asm_flac_cfg flac_write_cfg;
	union {
		struct asm_pcm_cfg          pcm_cfg;
		struct asm_aac_read_cfg     aac_cfg;
		struct asm_amrnb_read_cfg   amr_cfg;
		struct asm_evrc_read_cfg    evrc_cfg;
		struct asm_qcelp13_read_cfg qcelp13_cfg;
		struct asm_sbc_read_cfg     sbc_cfg;
	} __attribute__((packed)) read_cfg;
} __attribute__((packed));

/* vorbis in; pcm, aac, amrnb, evrc, qcelp13, sbc out */
struct asm_stream_cmd_open_read_vorbiswrite {
	struct apr_hdr        hdr;
	u32                   uMode;
	u32                   post_proc_top;
	u32                   sample_rate;
	u32                   write_format;
	u32                   write_cfg_size;
	u32                   frames_per_buffer;
	u32                   read_format;
	u32                   read_cfg_size;
	struct asm_vorbis_cfg vorbis_write_cfg;
	union {
		struct asm_pcm_cfg         pcm_cfg;
		struct asm_aac_read_cfg     aac_cfg;
		struct asm_amrnb_read_cfg   amr_cfg;
		struct asm_evrc_read_cfg    evrc_cfg;
		struct asm_qcelp13_read_cfg qcelp13_cfg;
		struct asm_sbc_read_cfg     sbc_cfg;
	} __attribute__((packed)) read_cfg;
} __attribute__((packed));


#define ASM_STREAM_CMD_SET_ENCDEC_PARAM                  0x00010C10
struct asm_stream_cmd_set_readwrite_param{
	struct apr_hdr hdr;
	u32            param_id;
	u16            param_size;
	u16            padding;
	union {
		struct asm_sbc_bitrate      sbc_bitrate;
		struct asm_aac_immed_decode aac_dec;
	} __attribute__((packed)) read_write_cfg;
} __attribute__((packed));

#define ASM_STREAM_CMD_GET_ENCDEC_PARAM                  0x00010C11
struct asm_stream_cmd_get_readwrite_param{
	struct apr_hdr hdr;
	u32            param_id;
	u16            param_size;
	u16            padding;
	union {
		struct asm_sbc_bitrate      sbc_bitrate;
		struct asm_aac_immed_decode aac_dec;
	} __attribute__((packed)) read_write_cfg;
} __attribute__((packed));

#define ASM_STREAM _CMD_ADJUST_SAMPLES                   0x00010C0A
struct asm_stream_cmd_adjust_samples{
	struct apr_hdr hdr;
	u16            nsamples;
	u16            reserved;
} __attribute__((packed));

#define ASM_STREAM_CMD_TAP_POPP_PCM                      0x00010BF9
struct asm_stream_cmd_tap_popp_pcm{
	struct apr_hdr hdr;
	u16            enable;
	u16            reserved;
	u32            module_id;
} __attribute__((packed));

/*  Session Level commands */

#define ASM_SESSION_CMD_RUN                              0x00010BD2
struct asm_stream_cmd_run{
	struct apr_hdr hdr;
	u32            flags;
	u32            msw_ts;
	u32            lsw_ts;
} __attribute__((packed));

/* Session level events */
#define ASM_SESSION_CMD_REGISTER_FOR_RX_UNDERFLOW_EVENTS 0x00010BD5
struct asm_stream_cmd_reg_rx_underflow_event{
	struct apr_hdr hdr;
	u16            enable;
	u16            reserved;
} __attribute__((packed));

#define ASM_SESSION_CMD_REGISTER_FOR_TX_OVERFLOW_EVENTS  0x00010BD6
struct asm_stream_cmd_reg_tx_overflow_event{
	struct apr_hdr hdr;
	u16            enable;
	u16            reserved;
} __attribute__((packed));

/* Data Path commands */
#define ASM_DATA_CMD_WRITE                               0x00010BD9
struct asm_stream_cmd_write{
	struct apr_hdr     hdr;
	u32	buf_add;
	u32	avail_bytes;
	u32	msw_ts;
	u32	lsw_ts;
	u32	uflags;
	u32	uid;
} __attribute__((packed));

#define ASM_DATA_CMD_READ                                0x00010BDA
struct asm_stream_cmd_read{
	struct apr_hdr     hdr;
	u32	buf_add;
	u32	buf_size;
	u32	uid;
} __attribute__((packed));

#define ASM_DATA_CMD_MEDIA_FORMAT_UPDATE                 0x00010BDC
#define ASM_DATA_EVENT_MEDIA_FORMAT_UPDATE               0x00010BDE
struct asm_stream_media_format_update{
	struct apr_hdr hdr;
	u32            format;
	u32            cfg_size;
	union {
		struct asm_pcm_cfg         pcm_cfg;
		struct asm_adpcm_cfg       adpcm_cfg;
		struct asm_yadpcm_cfg      yadpcm_cfg;
		struct asm_midi_cfg        midi_cfg;
		struct asm_wma_cfg         wma_cfg;
		struct asm_aac_cfg         aac_cfg;
		struct asm_flac_cfg        flac_cfg;
		struct asm_vorbis_cfg      vorbis_cfg;
	} __attribute__((packed)) write_cfg;
} __attribute__((packed));


/* Command Responses */
#define ASM_STREAM_CMDRSP_GET_ENCDEC_PARAM               0x00010C12
struct asm_stream_cmdrsp_get_readwrite_param{
	struct apr_hdr hdr;
	u32            status;
	u32            param_id;
	u16            param_size;
	u16            padding;
	union {
		struct asm_sbc_bitrate      sbc_bitrate;
		struct asm_aac_immed_decode aac_dec;
	} __attribute__((packed)) read_write_cfg;
} __attribute__((packed));


#define ASM_SESSION_CMDRSP_GET_SESSION_TIME              0x00010BD8
struct asm_stream_cmdrsp_get_session_time{
	struct apr_hdr hdr;
	u32            status;
	u32            msw_ts;
	u32            lsw_ts;
} __attribute__((packed));

#define ASM_DATA_EVENT_WRITE_DONE                        0x00010BDF
struct asm_data_event_write_done{
	u32	buf_add;
	u32            status;
} __attribute__((packed));

#define ASM_DATA_EVENT_READ_DONE                         0x00010BE0
struct asm_data_event_read_done{
	u32            status;
	u32            buffer_add;
	u32            enc_frame_size;
	u32            offset;
	u32            msw_ts;
	u32            lsw_ts;
	u32            flags;
	u32            num_frames;
	u32            id;
} __attribute__((packed));


/* service level events */

#define ASM_SERVICE_CMDRSP_GET_STREAM_HANDLES            0x00010C1B
struct asm_svc_cmdrsp_get_strm_handles{
	struct apr_hdr hdr;
	u32            num_handles;
	u32            stream_handles;
} __attribute__((packed));


#define ASM_SERVICE_CMDRSP_GET_WALLCLOCK_TIME            0x00010C1A
struct asm_svc_cmdrsp_get_wallclock_time{
	struct apr_hdr hdr;
	u32            status;
	u32            msw_ts;
	u32            lsw_ts;
} __attribute__((packed));

/*
 * Error code
*/
#define ADSP_EOK          0x00000000 /* Success / completed / no errors. */
#define ADSP_EFAILED      0x00000001 /* General failure. */
#define ADSP_EBADPARAM    0x00000002 /* Bad operation parameter(s). */
#define ADSP_EUNSUPPORTED 0x00000003 /* Unsupported routine/operation. */
#define ADSP_EVERSION     0x00000004 /* Unsupported version. */
#define ADSP_EUNEXPECTED  0x00000005 /* Unexpected problem encountered. */
#define ADSP_EPANIC       0x00000006 /* Unhandled problem occurred. */
#define ADSP_ENORESOURCE  0x00000007 /* Unable to allocate resource(s). */
#define ADSP_EHANDLE      0x00000008 /* Invalid handle. */
#define ADSP_EALREADY     0x00000009 /* Operation is already processed. */
#define ADSP_ENOTREADY    0x0000000A /* Operation not ready to be processed*/
#define ADSP_EPENDING     0x0000000B /* Operation is pending completion*/
#define ADSP_EBUSY        0x0000000C /* Operation could not be accepted or
					 processed. */
#define ADSP_EABORTED     0x0000000D /* Operation aborted due to an error. */
#define ADSP_EPREEMPTED   0x0000000E /* Operation preempted by higher priority*/
#define ADSP_ECONTINUE    0x0000000F /* Operation requests intervention
					to complete. */
#define ADSP_EIMMEDIATE   0x00000010 /* Operation requests immediate
					intervention to complete. */
#define ADSP_ENOTIMPL     0x00000011 /* Operation is not implemented. */
#define ADSP_ENEEDMORE    0x00000012 /* Operation needs more data or resources*/

#endif /*_APR_AUDIO_H_*/
