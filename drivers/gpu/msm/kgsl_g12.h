/* Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
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
#ifndef _KGSL_G12_H
#define _KGSL_G12_H

#define IDX_2D(X) ((X)-KGSL_DEVICE_2D0)

struct kgsl_g12_ringbuffer {
	unsigned int prevctx;
	struct kgsl_memdesc      cmdbufdesc;
};

struct kgsl_g12_device {
	struct kgsl_device dev;    /* Must be first field in this struct */
	const char *iomemname;
	const char *irqname;
	const char *regulator;
	int current_timestamp;
	int timestamp;
	wait_queue_head_t wait_timestamp_wq;
	struct kgsl_g12_ringbuffer ringbuffer;
	spinlock_t cmdwin_lock;
};

irqreturn_t kgsl_g12_isr(int irq, void *data);
int kgsl_g12_setstate(struct kgsl_device *device, uint32_t flags);
int kgsl_g12_idle(struct kgsl_device *device, unsigned int timeout);
struct kgsl_device *kgsl_get_2d_device(enum kgsl_deviceid);
void kgsl_g12_regread(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int *value);
void kgsl_g12_regwrite(struct kgsl_device *device, unsigned int offsetwords,
			unsigned int value);
void kgsl_g12_regread_isr(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int *value);
void kgsl_g12_regwrite_isr(struct kgsl_device *device, unsigned int offsetwords,
			unsigned int value);

int __init kgsl_g12_config(struct kgsl_devconfig *,
		      struct platform_device *pdev, enum kgsl_deviceid dev_id);

int __init kgsl_g12_init(struct kgsl_device *device);
int __init kgsl_g12_init_pwrctrl(struct kgsl_device *device);

int kgsl_g12_close(struct kgsl_device *device);

int kgsl_g12_getfunctable(struct kgsl_functable *ftbl);


#endif /* _KGSL_G12_H */
