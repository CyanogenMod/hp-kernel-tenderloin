
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
#ifndef SN12M0PZ_H
#define SN12M0PZ_H

#include <linux/types.h>
extern struct sn12m0pz_reg sn12m0pz_regs; /* from mt9t013_reg.c */
struct reg_struct{
	uint8_t pll_multiplier_lsb;            /* 0x0307*/
	uint8_t coarse_integ_time_msb;   /* 0x0202*/
	uint8_t coarse_integ_time_lsb;   /* 0x0203*/
	uint8_t frame_length_lines_msb;        /* 0x0340*/
	uint8_t frame_length_lines_lsb;        /* 0x0341*/
	uint8_t line_length_pck_msb;           /* 0x0342*/
	uint8_t line_length_pck_lsb;           /* 0x0343*/
	uint8_t x_output_size_msb;             /* 0x034C*/
	uint8_t x_output_size_lsb;             /* 0x034D*/
	uint8_t y_output_size_msb;             /* 0x034E*/
	uint8_t y_output_size_lsb;             /* 0x034F*/
	uint8_t x_even_inc_lsb;                /* 0x0381*/
	uint8_t x_odd_inc_lsb;                 /* 0x0383*/
	uint8_t y_even_inc_lsb;                /* 0x0385*/
	uint8_t y_odd_inc_lsb;                 /* 0x0387*/
	uint8_t reg_0x3016;                    /* 0x3016 VMODEADD*/
	uint8_t reg_0x30E8;                    /* 0x30E8 HADDAVE*/
	uint8_t reg_0x3301;                    /* 0x3301 RGLANESEL*/
	/*added for 120fps support */
	uint8_t reg_0x0344;
	uint8_t reg_0x0345;
	uint8_t reg_0x0346;
	uint8_t reg_0x0347;
	uint8_t reg_0x0348;
	uint8_t reg_0x0349;
	uint8_t reg_0x034A;
	uint8_t reg_0x034B;
};
struct reg_struct_init{
	uint8_t reg_0x302B;/* 0x302B*/

	uint8_t reg_0x30E5;/* 0x30E5*/
	uint8_t reg_0x3300;   /* 0x3300*/

	uint8_t image_orient;   /* 0x0101*/

	uint8_t reg_0x300A;   /* 0x300A*/
	uint8_t reg_0x3014;   /* 0x3014*/
	uint8_t reg_0x3015;   /* 0x3015*/
	uint8_t reg_0x3017;   /* 0x3017*/
	uint8_t reg_0x301C;   /* 0x301C*/
	uint8_t reg_0x3031;   /* 0x3031*/
	uint8_t reg_0x3040;   /* 0x3040*/
	uint8_t reg_0x3041;   /* 0x3041*/
	uint8_t reg_0x3051;   /* 0x3051*/
	uint8_t reg_0x3053;   /* 0x3053*/
	uint8_t reg_0x3055;   /* 0x3055*/
	uint8_t reg_0x3057;   /* 0x3057*/
	uint8_t reg_0x3060;   /* 0x3060*/
	uint8_t reg_0x3065;   /* 0x3065*/
	uint8_t reg_0x30AA;   /* 0x30AA*/
	uint8_t reg_0x30AB;   /* 0x30AB*/
	uint8_t reg_0x30B0;   /* 0x30B0*/
	uint8_t reg_0x30B2;   /* 0x30B2*/

	uint8_t reg_0x30D3;   /* 0X30D3*/
	uint8_t reg_0x30D8;   /* 0X30D8*/

	uint8_t reg_0x3106;   /* 0x3106*/
	uint8_t reg_0x3108;   /* 0x3108*/
	uint8_t reg_0x310A;   /* 0x310A*/
	uint8_t reg_0x310C;   /* 0x310C*/
	uint8_t reg_0x310E;   /* 0x310E*/
	uint8_t reg_0x3126;   /* 0x3126*/
	uint8_t reg_0x312E;   /* 0x312E*/
	uint8_t reg_0x313C;   /* 0x313C*/
	uint8_t reg_0x313E;   /* 0x313E*/
	uint8_t reg_0x3140;   /* 0x3140*/
	uint8_t reg_0x3142;   /* 0x3142*/
	uint8_t reg_0x3144;   /* 0x3144*/
	uint8_t reg_0x3148;   /* 0x3148*/
	uint8_t reg_0x314A;   /* 0x314A*/
	uint8_t reg_0x3166;   /* 0x3166*/
	uint8_t reg_0x3168;   /* 0x3168*/
	uint8_t reg_0x316F;   /* 0x316F*/
	uint8_t reg_0x3171;   /* 0x3171*/
	uint8_t reg_0x3173;   /* 0x3173*/
	uint8_t reg_0x3175;   /* 0x3175*/
	uint8_t reg_0x3177;   /* 0x3177*/
	uint8_t reg_0x3179;   /* 0x3179*/
	uint8_t reg_0x317B;   /* 0x317B*/
	uint8_t reg_0x317D;   /* 0x317D*/
	uint8_t reg_0x317F;   /* 0x317F*/
	uint8_t reg_0x3181;   /* 0x3181*/
	uint8_t reg_0x3184;   /* 0x3184*/
	uint8_t reg_0x3185;   /* 0x3185*/
	uint8_t reg_0x3187;   /* 0x3187*/

	uint8_t reg_0x31A4;   /* 0x31A4*/
	uint8_t reg_0x31A6;   /* 0x31A6*/
	uint8_t reg_0x31AC;   /* 0x31AC*/
	uint8_t reg_0x31AE;   /* 0x31AE*/
	uint8_t reg_0x31B4;   /* 0x31B4*/
	uint8_t reg_0x31B6;   /* 0x31B6*/

	uint8_t reg_0x3254;   /* 0x3254*/
	uint8_t reg_0x3256;   /* 0x3256*/
	uint8_t reg_0x3258;   /* 0x3258*/
	uint8_t reg_0x325A;   /* 0x325A*/
	uint8_t reg_0x3260;   /* 0x3260*/
	uint8_t reg_0x3262;   /* 0x3262*/

	uint8_t reg_0x3304;   /* 0x3304*/
	uint8_t reg_0x3305;   /* 0x3305*/
	uint8_t reg_0x3306;   /* 0x3306*/
	uint8_t reg_0x3307;   /* 0x3307*/
	uint8_t reg_0x3308;   /* 0x3308*/
	uint8_t reg_0x3309;   /* 0x3309*/
	uint8_t reg_0x330A;   /* 0x330A*/
	uint8_t reg_0x330B;   /* 0x330B*/
	uint8_t reg_0x330C;   /* 0x330C*/
	uint8_t reg_0x330D;   /* 0x330D*/

};
struct sn12m0pz_reg{
	const struct reg_struct  *reg_pat;
	const struct reg_struct_init  *reg_pat_init;
};
#endif
