/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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

#ifndef MSM_GEMINI_SYNC_H
#define MSM_GEMINI_SYNC_H

#include <linux/fs.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include "msm_gemini_core.h"

struct msm_gemini_q {
	char const	*name;
	struct list_head  q;
	spinlock_t	lck;
	wait_queue_head_t wait;
	int	       unblck;
};

struct msm_gemini_q_entry {
	struct list_head list;
	void   *data;
};

struct msm_gemini_device {
	struct platform_device *pdev;
	struct resource        *mem;
	int                     irq;
	void                   *base;

	struct device *device;
	struct cdev   cdev;
	struct mutex  lock;
	char	  open_count;
	uint8_t       op_mode;

	/* event queue including frame done & err indications
	 */
	struct msm_gemini_q evt_q;

	/* output return queue
	 */
	struct msm_gemini_q output_rtn_q;

	/* output buf queue
	 */
	struct msm_gemini_q output_buf_q;

	/* input return queue
	 */
	struct msm_gemini_q input_rtn_q;

	/* input buf queue
	 */
	struct msm_gemini_q input_buf_q;
};

int __msm_gemini_open(struct msm_gemini_device *pgmn_dev);
int __msm_gemini_release(struct msm_gemini_device *pgmn_dev);

long __msm_gemini_ioctl(struct msm_gemini_device *pgmn_dev,
	unsigned int cmd, unsigned long arg);

struct msm_gemini_device *__msm_gemini_init(struct platform_device *pdev);
int __msm_gemini_exit(struct msm_gemini_device *pgmn_dev);

#endif /* MSM_GEMINI_SYNC_H */
