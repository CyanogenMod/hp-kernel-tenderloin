/* Copyright (c) 2002,2007-2011, Code Aurora Forum. All rights reserved.
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
#include <linux/string.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/msm_kgsl.h>

#include "kgsl_sharedmem.h"
#include "kgsl.h"
#include "kgsl_g12.h"
#include "kgsl_log.h"
#include "kgsl_g12_cmdwindow.h"
#include "kgsl_g12_vgv3types.h"
#include "g12_reg.h"
#include "kgsl_g12_drawctxt.h"

int
kgsl_g12_drawctxt_create(struct kgsl_device_private *dev_priv,
			 uint32_t unused,
			 struct kgsl_context *context)
{
	return 0;
}

int
kgsl_g12_drawctxt_destroy(struct kgsl_device *device,
			  struct kgsl_context *context)
{
	struct kgsl_g12_device *g12_device = KGSL_G12_DEVICE(device);

	kgsl_g12_idle(device, KGSL_TIMEOUT_DEFAULT);

	if (g12_device->ringbuffer.prevctx == context->id) {
		g12_device->ringbuffer.prevctx = KGSL_G12_INVALID_CONTEXT;
		device->mmu.hwpagetable = device->mmu.defaultpagetable;
		kgsl_setstate(device, KGSL_MMUFLAGS_PTUPDATE);
	}

	return 0;
}
