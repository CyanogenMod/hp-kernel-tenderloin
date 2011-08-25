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
#ifndef AUDIO_LPA_H
#define AUDIO_LPA_H

#include <linux/earlysuspend.h>

#define ADRV_STATUS_OBUF_GIVEN 0x00000001
#define ADRV_STATUS_IBUF_GIVEN 0x00000002
#define ADRV_STATUS_FSYNC 0x00000004
#define ADRV_STATUS_PAUSE 0x00000008

struct buffer {
	void *data;
	unsigned size;
	unsigned used;		/* Input usage actual DSP produced PCM size  */
	unsigned addr;
};

#ifdef CONFIG_HAS_EARLYSUSPEND
struct audlpa_suspend_ctl {
	struct early_suspend node;
	struct audio *audio;
};
#endif

struct codec_operations {
	long (*ioctl)(struct file *, unsigned int, unsigned long);
	int (*set_params)(void *);
};

struct audio {
	spinlock_t dsp_lock;

	uint8_t out_needed; /* number of buffers the dsp is waiting for */
	struct list_head out_queue; /* queue to retain output buffers */

	struct mutex lock;
	struct mutex write_lock;
	wait_queue_head_t write_wait;

	struct audio_client *ac;

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
	int out_enabled;
	int out_prefill;
	int running;
	int stopped; /* set when stopped, cleared on flush */
	int buf_refresh;
	int teos; /* valid only if tunnel mode & no data left for decoder */

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct audlpa_suspend_ctl suspend_ctl;
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

	uint32_t device_events;

	struct list_head pmem_region_queue; /* protected by lock */

	int eq_enable;
	int eq_needs_commit;
	uint32_t volume;

	unsigned int minor_no;
	struct codec_operations codec_ops;
	uint32_t buffer_size;
	uint32_t buffer_count;
};

#endif /* !AUDIO_LPA_H */
