/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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
 * helper functions for physically contiguous PMEM capture buffers
 */

#ifndef _VIDEOBUF_PMEM_CONTIG_H
#define _VIDEOBUF_PMEM_CONTIG_H

#include <media/videobuf-core.h>

struct videobuf_contig_pmem {
	u32 magic;
	void *vaddr;
	int phyaddr;
	unsigned long size;
	int is_userptr;
	uint32_t y_off;
	uint32_t cbcr_off;
	int buffer_type;
};

void videobuf_queue_pmem_contig_init(struct videobuf_queue *q,
			const struct videobuf_queue_ops *ops,
			struct device *dev,
			spinlock_t *irqlock,
			enum v4l2_buf_type type,
			enum v4l2_field field,
			unsigned int msize,
			void *priv);

int videobuf_to_pmem_contig(struct videobuf_buffer *buf);
int videobuf_pmem_contig_free(struct videobuf_queue *q,
			struct videobuf_buffer *buf);

#endif /* _VIDEOBUF_PMEM_CONTIG_H */
