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
 *
 */
#ifndef _AUDIO_ACDB_H
#define _AUDIO_ACDB_H

#include <linux/msm_audio_acdb.h>
#include "q6adm.h"

#define NUM_AUDPROC_BUFFERS	6

struct acdb_cal_block {
	uint32_t		cal_size;
	uint32_t		cal_kvaddr;
	uint32_t		cal_paddr;
};

struct acdb_cal_data {
	uint32_t		num_cal_blocks;
	struct acdb_cal_block	*cal_blocks;
};

struct audproc_buffer_data {
	uint32_t	buf_size[NUM_AUDPROC_BUFFERS];
	uint32_t	phys_addr[NUM_AUDPROC_BUFFERS];
};

void get_audproc_buffer_data(struct audproc_buffer_data *cal_buffers);
void get_audproc_cal(int32_t path, struct acdb_cal_block *cal_block);
void get_audstrm_cal(int32_t path, struct acdb_cal_block *cal_block);
void get_audvol_cal(int32_t path, struct acdb_cal_block *cal_block);
void get_vocproc_cal(struct acdb_cal_data *cal_data);
void get_vocstrm_cal(struct acdb_cal_data *cal_data);
void get_vocvol_cal(struct acdb_cal_data *cal_data);
void get_sidetone_cal(struct sidetone_cal *cal_data);
void get_anc_cal(struct acdb_cal_block *cal_block);

#endif
