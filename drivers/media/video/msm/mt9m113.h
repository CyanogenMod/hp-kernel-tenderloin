/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of Code Aurora Forum, Inc. nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

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
#ifndef MT9M113_H
#define MT9M113_H
#include <linux/types.h>
#include <mach/board.h>
#include <mach/camera.h>

extern struct mt9m113_reg mt9m113_regs;
enum mt9m113_width {
	WORD_LEN,
	BYTE_LEN
};
struct mt9m113_i2c_reg_conf {
	unsigned short waddr;
	unsigned short wmask;
	unsigned short wdata;
	enum mt9m113_width width;
	unsigned short mdelay_time;
};
struct mt9m113_reg {
	const struct mt9m113_i2c_reg_conf *prev_snap_reg_tbl;
	uint16_t prev_snap_reg_tbl_size;
	const struct mt9m113_i2c_reg_conf *plltbl;
	uint16_t plltbl_size;
	const struct mt9m113_i2c_reg_conf *stbl;
	uint16_t stbl_size;
};

#endif /* MT9M113_H */
