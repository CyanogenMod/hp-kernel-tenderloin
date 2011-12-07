/* Copyright (c) 2002,2007-2010, Code Aurora Forum. All rights reserved.
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
#ifndef __GSL_DRAWCTXT_H
#define __GSL_DRAWCTXT_H

/* Flags */

#define CTXT_FLAGS_NOT_IN_USE		0x00000000
#define CTXT_FLAGS_IN_USE			0x00000001

/* state shadow memory allocated */
#define CTXT_FLAGS_STATE_SHADOW		0x00000010

/* gmem shadow memory allocated */
#define CTXT_FLAGS_GMEM_SHADOW		0x00000100
/* gmem must be copied to shadow */
#define CTXT_FLAGS_GMEM_SAVE		0x00000200
/* gmem can be restored from shadow */
#define CTXT_FLAGS_GMEM_RESTORE		0x00000400
/* shader must be copied to shadow */
#define CTXT_FLAGS_SHADER_SAVE		0x00002000
/* shader can be restored from shadow */
#define CTXT_FLAGS_SHADER_RESTORE	0x00004000
/* Context has caused a GPU hang */
#define CTXT_FLAGS_GPU_HANG		0x00008000

#include <linux/msm_kgsl.h>
#include "kgsl_sharedmem.h"
#include "yamato_reg.h"

struct kgsl_device;
struct kgsl_yamato_device;
struct kgsl_device_private;
struct kgsl_context;

/*  types */

/* draw context */
struct gmem_shadow_t {
	struct kgsl_memdesc gmemshadow;	/* Shadow buffer address */

	/* 256 KB GMEM surface = 4 bytes-per-pixel x 256 pixels/row x
	* 256 rows. */
	/* width & height must be a multiples of 32, in case tiled textures
	 * are used. */
	enum COLORFORMATX format;
	unsigned int size;	/* Size of surface used to store GMEM */
	unsigned int width;	/* Width of surface used to store GMEM */
	unsigned int height;	/* Height of surface used to store GMEM */
	unsigned int pitch;	/* Pitch of surface used to store GMEM */
	unsigned int gmem_pitch;	/* Pitch value used for GMEM */
	unsigned int *gmem_save_commands;
	unsigned int *gmem_restore_commands;
	unsigned int gmem_save[3];
	unsigned int gmem_restore[3];
	struct kgsl_memdesc quad_vertices;
	struct kgsl_memdesc quad_texcoords;
};

struct kgsl_yamato_context {
	uint32_t         flags;
	struct kgsl_pagetable *pagetable;
	struct kgsl_memdesc       gpustate;
	unsigned int        reg_save[3];
	unsigned int        reg_restore[3];
	unsigned int        shader_save[3];
	unsigned int        shader_fixup[3];
	unsigned int        shader_restore[3];
	unsigned int		chicken_restore[3];
	unsigned int 	    bin_base_offset;
	/* Information of the GMEM shadow that is created in context create */
	struct gmem_shadow_t context_gmem_shadow;
};


int kgsl_drawctxt_create(struct kgsl_device_private *dev_priv,
			 uint32_t flags,
			 struct kgsl_context *context);

int kgsl_drawctxt_destroy(struct kgsl_device *device,
			  struct kgsl_context *context);

void kgsl_drawctxt_switch(struct kgsl_yamato_device *yamato_device,
				struct kgsl_yamato_context *drawctxt,
				unsigned int flags);
int kgsl_drawctxt_set_bin_base_offset(struct kgsl_device *device,
				      struct kgsl_context *context,
					unsigned int offset);

#endif  /* __GSL_DRAWCTXT_H */
