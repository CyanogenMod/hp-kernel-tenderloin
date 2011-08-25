/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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

#ifndef __MSM_DMA_TEST__
#define __MSM_DMA_TEST__

#include <linux/ioctl.h>

#define MSM_DMA_IOC_MAGIC     0x83

/* The testing driver can manage a series of buffers.  These are
 * allocated and freed using these calls. */
struct msm_dma_alloc_req {
	int size;		/* Size of this request, in bytes. */
	int bufnum;		/* OUT: Number of buffer allocated. */
};
#define MSM_DMA_IOALLOC _IOWR(MSM_DMA_IOC_MAGIC, 2, struct msm_dma_alloc_req)

/* Free the specified buffer. */
#define MSM_DMA_IOFREE _IOW(MSM_DMA_IOC_MAGIC, 3, int)

/* Free all used buffers. */
#define MSM_DMA_IOFREEALL  _IO(MSM_DMA_IOC_MAGIC, 7)

/* Read/write data into kernel buffer. */
struct msm_dma_bufxfer {
	void *data;
	int size;
	int bufnum;
};
#define MSM_DMA_IOWBUF _IOW(MSM_DMA_IOC_MAGIC, 4, struct msm_dma_bufxfer)
#define MSM_DMA_IORBUF _IOW(MSM_DMA_IOC_MAGIC, 5, struct msm_dma_bufxfer)

/* Use the data mover to copy from one buffer to another. */
struct msm_dma_scopy {
	int srcbuf;
	int destbuf;
	int size;
};
#define MSM_DMA_IOSCOPY _IOW(MSM_DMA_IOC_MAGIC, 6, struct msm_dma_scopy)

#endif /* __MSM_DMA_TEST__ */
