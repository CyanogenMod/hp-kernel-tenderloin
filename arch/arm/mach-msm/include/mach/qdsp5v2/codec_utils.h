/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef CODEC_UTILS_H
#define CODEC_UTILS_H

#include <linux/earlysuspend.h>

#define ADRV_STATUS_AIO_INTF 0x00000001
#define ADRV_STATUS_OBUF_GIVEN 0x00000002
#define ADRV_STATUS_IBUF_GIVEN 0x00000004
#define ADRV_STATUS_FSYNC 0x00000008

#define PCM_BUFSZ_MIN 4800	/* Hold one stereo MP3 frame */
#define PCM_BUF_MAX_COUNT 5	/* DSP only accepts 5 buffers at most
				   but support 2 buffers currently */
#define ROUTING_MODE_FTRT 1
#define ROUTING_MODE_RT 2
/* Decoder status received from AUDPPTASK */
#define  AUDPP_DEC_STATUS_SLEEP	0
#define	 AUDPP_DEC_STATUS_INIT  1
#define  AUDPP_DEC_STATUS_CFG   2
#define  AUDPP_DEC_STATUS_PLAY  3
#define  AUDPP_DEC_STATUS_EOS   5

/* worst case delay of 3secs(3000ms) for AV Sync Query response */
#define AVSYNC_EVENT_TIMEOUT 3000

struct buffer {
	void *data;
	unsigned size;
	unsigned used;		/* Input usage actual DSP produced PCM size  */
	unsigned addr;
};

#ifdef CONFIG_HAS_EARLYSUSPEND
struct audio_suspend_ctl {
	struct early_suspend node;
	struct audio *audio;
};
#endif

struct codec_operations {
	long (*ioctl)(struct file *, unsigned int, unsigned long);
	void (*adec_params)(struct audio *);
};

struct audio {
	spinlock_t dsp_lock;

	uint8_t out_needed; /* number of buffers the dsp is waiting for */
	struct list_head out_queue; /* queue to retain output buffers */
	atomic_t out_bytes;

	struct mutex lock;
	struct mutex write_lock;
	wait_queue_head_t write_wait;

	struct msm_adsp_module *audplay;

	/* configuration to use on next enable */
	uint32_t out_sample_rate;
	uint32_t out_channel_mode;
	uint32_t out_bits; /* bits per sample (used by PCM decoder) */

	/* data allocated for various buffers */
	char *data;
	int32_t phys; /* physical address of write buffer */

	uint32_t drv_status;
	int wflush; /* Write flush */
	int opened;
	int enabled;
	int running;
	int stopped; /* set when stopped, cleared on flush */
	int buf_refresh;
	int teos; /* valid only if tunnel mode & no data left for decoder */
	enum msm_aud_decoder_state dec_state;	/* Represents decoder state */
	int reserved; /* A byte is being reserved */
	char rsv_byte; /* Handle odd length user data */

	const char *module_name;
	unsigned queue_id;
	uint16_t dec_id;
	uint32_t read_ptr_offset;
	int16_t source;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct audio_suspend_ctl suspend_ctl;
#endif

#ifdef CONFIG_DEBUG_FS
	struct dentry *dentry;
#endif

	wait_queue_head_t wait;
	struct list_head free_event_queue;
	struct list_head event_queue;
	wait_queue_head_t event_wait;
	spinlock_t event_queue_lock;
	struct mutex get_event_lock;
	int event_abort;
	/* AV sync Info */
	int avsync_flag;              /* Flag to indicate feedback from DSP */
	wait_queue_head_t avsync_wait;/* Wait queue for AV Sync Message     */
	/* flags, 48 bits sample/bytes counter per channel */
	uint16_t avsync[AUDPP_AVSYNC_CH_COUNT * AUDPP_AVSYNC_NUM_WORDS + 1];

	uint32_t device_events;
	uint32_t device_switch;       /* Flag to indicate device switch */
	uint64_t bytecount_consumed;
	uint64_t bytecount_head;
	uint64_t bytecount_given;
	uint64_t bytecount_query;

	struct list_head pmem_region_queue; /* protected by lock */

	int eq_enable;
	int eq_needs_commit;
	struct audpp_cmd_cfg_object_params_eqalizer eq;
	struct audpp_cmd_cfg_object_params_volume vol_pan;

	unsigned int minor_no;
	struct codec_operations codec_ops;
};

#endif /* !CODEC_UTILS_H */
