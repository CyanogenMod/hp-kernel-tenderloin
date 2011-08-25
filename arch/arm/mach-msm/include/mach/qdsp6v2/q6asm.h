/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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
 */
#ifndef __Q6_ASM_H__
#define __Q6_ASM_H__

#include <mach/qdsp6v2/apr.h>

#define IN                      0x000
#define OUT                     0x001
#define CH_MODE_MONO            0x001
#define CH_MODE_STEREO          0x002

#define FORMAT_LINEAR_PCM   0x0000
#define FORMAT_DTMF         0x0001
#define FORMAT_ADPCM	    0x0002
#define FORMAT_YADPCM       0x0003
#define FORMAT_MP3          0x0004
#define FORMAT_MPEG4_AAC    0x0005
#define FORMAT_AMRNB	    0x0006
#define FORMAT_AMRWB	    0x0007
#define FORMAT_V13K	    0x0008
#define FORMAT_EVRC	    0x0009
#define FORMAT_EVRCB	    0x000a
#define FORMAT_EVRCWB	    0x000b
#define FORMAT_MIDI	    0x000c
#define FORMAT_SBC	    0x000d
#define FORMAT_WMA_V10PRO   0x000e
#define FORMAT_WMA_V9	    0x000f
#define FORMAT_AMR_WB_PLUS  0x0010

#define ENCDEC_SBCBITRATE   0x0001
#define ENCDEC_IMMEDIATE_DECODE 0x0002
#define ENCDEC_CFG_BLK          0x0003

#define CMD_PAUSE          0x0001
#define CMD_FLUSH          0x0002
#define CMD_EOS            0x0003
#define CMD_CLOSE          0x0004

/* bit 0:1 represents priority of stream */
#define STREAM_PRIORITY_NORMAL	0x0000
#define STREAM_PRIORITY_LOW	0x0001
#define STREAM_PRIORITY_HIGH	0x0002

/* bit 4 represents META enable of encoded data buffer */
#define BUFFER_META_ENABLE	0x0010

#define ASYNC_IO_MODE	0x0002
#define SYNC_IO_MODE	0x0001
#define NO_TIMESTAMP    0xFF00
#define SET_TIMESTAMP   0x0000

typedef void (*app_cb)(uint32_t opcode, uint32_t token,
			uint32_t *payload, void *priv);

struct audio_buffer {
	dma_addr_t phys;
	void       *data;
	uint32_t   used;
	uint32_t   size;/* size of buffer */
	uint32_t   actual_size; /* actual number of bytes read by DSP */
};

struct audio_aio_write_param {
	unsigned long paddr;
	uint32_t uid;
	uint32_t len;
	uint32_t msw_ts;
	uint32_t lsw_ts;
	uint32_t flags;
};

struct audio_aio_read_param {
	unsigned long paddr;
	uint32_t len;
	uint32_t uid;
};

struct audio_port_data {
	struct audio_buffer *buf;
	uint32_t	    max_buf_cnt;
	uint32_t	    dsp_buf;
	uint32_t	    cpu_buf;
	/* read or write locks */
	struct mutex	    lock;
	spinlock_t	    dsp_lock;
};

struct audio_client {
	int                    session;
	/* idx:1 out port, 0: in port*/
	struct audio_port_data port[2];

	struct apr_svc         *apr;
	struct mutex	       cmd_lock;

	atomic_t		cmd_state;
	atomic_t		time_flag;
	wait_queue_head_t	cmd_wait;
	wait_queue_head_t	time_wait;

	app_cb			cb;
	void			*priv;
	uint32_t         io_mode;
	uint32_t         time_stamp;
};

void q6asm_audio_client_free(struct audio_client *ac);

struct audio_client *q6asm_audio_client_alloc(app_cb cb, void *priv);

int q6asm_audio_client_buf_alloc(unsigned int dir/* 1:Out,0:In */,
				struct audio_client *ac,
				unsigned int bufsz,
				unsigned int bufcnt);
int q6asm_audio_client_buf_alloc_contiguous(unsigned int dir
				/* 1:Out,0:In */,
				struct audio_client *ac,
				unsigned int bufsz,
				unsigned int bufcnt);

int q6asm_audio_client_buf_free_contiguous(unsigned int dir,
			struct audio_client *ac);

int q6asm_open_read(struct audio_client *ac, uint32_t format);

int q6asm_open_write(struct audio_client *ac, uint32_t format);

int q6asm_open_read_write(struct audio_client *ac,
			uint32_t rd_format,
			uint32_t wr_format);

int q6asm_write(struct audio_client *ac, uint32_t len, uint32_t msw_ts,
				uint32_t lsw_ts, uint32_t flags);

int q6asm_write_nolock(struct audio_client *ac, uint32_t len, uint32_t msw_ts,
				uint32_t lsw_ts, uint32_t flags);

int q6asm_async_write(struct audio_client *ac,
					  struct audio_aio_write_param *param);

int q6asm_async_read(struct audio_client *ac,
					  struct audio_aio_read_param *param);

int q6asm_read(struct audio_client *ac);
int q6asm_read_nolock(struct audio_client *ac);

int q6asm_memory_map(struct audio_client *ac, uint32_t buf_add,
			int dir, uint32_t bufsz, uint32_t bufcnt);

int q6asm_memory_unmap(struct audio_client *ac, uint32_t buf_add,
							int dir);

int q6asm_run(struct audio_client *ac, uint32_t flags,
		uint32_t msw_ts, uint32_t lsw_ts);

int q6asm_run_nowait(struct audio_client *ac, uint32_t flags,
		uint32_t msw_ts, uint32_t lsw_ts);

int q6asm_reg_tx_overflow(struct audio_client *ac, uint16_t enable);

int q6asm_cmd(struct audio_client *ac, int cmd);

int q6asm_cmd_nowait(struct audio_client *ac, int cmd);

void *q6asm_is_cpu_buf_avail(int dir, struct audio_client *ac,
				uint32_t *size, uint32_t *idx);

int q6asm_is_dsp_buf_avail(int dir, struct audio_client *ac);

/* File format specific configurations to be added below */

int q6asm_enc_cfg_blk_aac(struct audio_client *ac,
			 uint32_t frames_per_buf,
			uint32_t sample_rate, uint32_t channels,
			 uint32_t bit_rate,
			 uint32_t mode, uint32_t format);

int q6asm_enc_cfg_blk_pcm(struct audio_client *ac,
			uint32_t rate, uint32_t channels);

int q6asm_enc_cfg_blk_qcelp(struct audio_client *ac, uint32_t frames_per_buf,
		uint16_t min_rate, uint16_t max_rate,
		uint16_t reduced_rate_level, uint16_t rate_modulation_cmd);

int q6asm_enc_cfg_blk_evrc(struct audio_client *ac, uint32_t frames_per_buf,
		uint16_t min_rate, uint16_t max_rate,
		uint16_t rate_modulation_cmd);

int q6asm_enc_cfg_blk_amrnb(struct audio_client *ac, uint32_t frames_per_buf,
		uint16_t band_mode, uint16_t dtx_enable);

int q6asm_media_format_block_pcm(struct audio_client *ac,
			uint32_t rate, uint32_t channels);

int q6asm_media_format_block_wma(struct audio_client *ac,
			void *cfg);

int q6asm_media_format_block_wmapro(struct audio_client *ac,
			void *cfg);

/* PP specific */
int q6asm_equalizer(struct audio_client *ac, void *eq);

/* Send Volume Command */
int q6asm_set_volume(struct audio_client *ac, int volume);

/* Send left-right channel gain */
int q6asm_set_lrgain(struct audio_client *ac, int left_gain, int right_gain);

/* Enable Mute/unmute flag */
int q6asm_set_mute(struct audio_client *ac, int muteflag);

uint32_t q6asm_get_session_time(struct audio_client *ac);

/* Client can set the IO mode to either AIO/SIO mode */
int q6asm_set_io_mode(struct audio_client *ac, uint32_t mode);
#endif /* __Q6_ASM_H__ */
