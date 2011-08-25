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

#ifndef VB6801_H
#define VB6801_H

#include <mach/board.h>

extern struct vb6801_reg_t vb6801_regs;	/* from vb6801_reg.c */

struct reg_struct {
	uint16_t vt_pix_clk_div;	/*  0x0300 */
	uint16_t vt_sys_clk_div;	/*  0x0302 */
	uint16_t pre_pll_clk_div;	/*  0x0304 */
	uint16_t pll_multiplier;	/*  0x0306 */
	uint16_t op_pix_clk_div;	/*  0x0308 */
	uint16_t op_sys_clk_div;	/*  0x030A */
	uint16_t scale_m;	/*  0x0404 */
	uint16_t row_speed;	/*  0x3016 */
	uint16_t x_addr_start;	/*  0x3004 */
	uint16_t x_addr_end;	/*  0x3008 */
	uint16_t y_addr_start;	/*  0x3002 */
	uint16_t y_addr_end;	/*  0x3006 */
	uint16_t read_mode;	/*  0x3040 */
	uint16_t x_output_size;	/*  0x034C */
	uint16_t y_output_size;	/*  0x034E */
	uint16_t line_length_pck;	/*  0x300C */
	uint16_t frame_length_lines;	/*  0x300A */
	uint16_t coarse_int_time;	/*  0x3012 */
	uint16_t fine_int_time;	/*  0x3014 */
};

enum i2c_data_len {
	D_LEN_BYTE,
	D_LEN_WORD
};

struct vb6801_i2c_reg_conf_t {
	unsigned short waddr;
	unsigned short wdata;
	uint8_t bdata;
	enum i2c_data_len dlen;
};

struct vb6801_reg_t {
	struct reg_struct const *reg_pat;
	uint16_t reg_pat_size;
	struct vb6801_i2c_reg_conf_t const *ttbl;
	uint16_t ttbl_size;
	struct vb6801_i2c_reg_conf_t const *lctbl;
	uint16_t lctbl_size;
	struct vb6801_i2c_reg_conf_t const *rftbl;
	uint16_t rftbl_size;
};

#endif /* VB6801_H */
