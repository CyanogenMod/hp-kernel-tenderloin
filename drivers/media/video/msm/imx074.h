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

#ifndef IMX074_H
#define IMX074_H
#include <linux/types.h>
#include <mach/board.h>
extern struct imx074_reg imx074_regs;
struct reg_struct_init {
    /* PLL setting */
	uint8_t pre_pll_clk_div; /* 0x0305 */
	uint8_t plstatim; /* 0x302b */
	uint8_t reg_3024; /*ox3024*/
	uint8_t image_orientation;  /* 0x0101*/
	uint8_t vndmy_ablmgshlmt; /*0x300a*/
	uint8_t y_opbaddr_start_di; /*0x3014*/
	uint8_t reg_0x3015; /*0x3015*/
	uint8_t reg_0x301c; /*0x301c*/
	uint8_t reg_0x302c; /*0x302c*/
	uint8_t reg_0x3031; /*0x3031*/
	uint8_t reg_0x3041; /* 0x3041 */
	uint8_t reg_0x3051; /* 0x3051 */
	uint8_t reg_0x3053; /* 0x3053 */
	uint8_t reg_0x3057; /* 0x3057 */
	uint8_t reg_0x305c; /* 0x305c */
	uint8_t reg_0x305d; /* 0x305d */
	uint8_t reg_0x3060; /* 0x3060 */
	uint8_t reg_0x3065; /* 0x3065 */
	uint8_t reg_0x30aa; /* 0x30aa */
	uint8_t reg_0x30ab;
	uint8_t reg_0x30b0;
	uint8_t reg_0x30b2;
	uint8_t reg_0x30d3;
	uint8_t reg_0x3106;
	uint8_t reg_0x310c;
	uint8_t reg_0x3304;
	uint8_t reg_0x3305;
	uint8_t reg_0x3306;
	uint8_t reg_0x3307;
	uint8_t reg_0x3308;
	uint8_t reg_0x3309;
	uint8_t reg_0x330a;
	uint8_t reg_0x330b;
	uint8_t reg_0x330c;
	uint8_t reg_0x330d;
	uint8_t reg_0x330f;
	uint8_t reg_0x3381;
};

struct reg_struct {
	uint8_t pll_multiplier; /* 0x0307 */
	uint8_t frame_length_lines_hi; /* 0x0340*/
	uint8_t frame_length_lines_lo; /* 0x0341*/
	uint8_t y_addr_start;  /* 0x347 */
	uint8_t y_add_end;  /* 0x034b */
	uint8_t x_output_size_msb;  /* 0x034c */
	uint8_t x_output_size_lsb;  /* 0x034d */
	uint8_t y_output_size_msb; /* 0x034e */
	uint8_t y_output_size_lsb; /* 0x034f */
	uint8_t x_even_inc;  /* 0x0381 */
	uint8_t x_odd_inc; /* 0x0383 */
	uint8_t y_even_inc;  /* 0x0385 */
	uint8_t y_odd_inc; /* 0x0387 */
	uint8_t hmodeadd;   /* 0x3001 */
	uint8_t vmodeadd;   /* 0x3016 */
	uint8_t vapplinepos_start;/*ox3069*/
	uint8_t vapplinepos_end;/*306b*/
	uint8_t shutter;	/* 0x3086 */
	uint8_t haddave;	/* 0x30e8 */
	uint8_t lanesel;    /* 0x3301 */
};

struct imx074_i2c_reg_conf {
	unsigned short waddr;
	unsigned short wdata;
};

enum imx074_test_mode_t {
	TEST_OFF,
	TEST_1,
	TEST_2,
	TEST_3
};

enum imx074_resolution_t {
	QTR_SIZE,
	FULL_SIZE,
	INVALID_SIZE
};
enum imx074_setting {
	RES_PREVIEW,
	RES_CAPTURE
};
enum mt9p012_reg_update {
	/* Sensor egisters that need to be updated during initialization */
	REG_INIT,
	/* Sensor egisters that needs periodic I2C writes */
	UPDATE_PERIODIC,
	/* All the sensor Registers will be updated */
	UPDATE_ALL,
	/* Not valid update */
	UPDATE_INVALID
};

struct imx074_reg {
	const struct reg_struct_init  *reg_pat_init;
	const struct reg_struct  *reg_pat;
};
#endif /* IMX074_H */
