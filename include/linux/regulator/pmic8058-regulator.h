/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __PMIC8058_REGULATOR_H__
#define __PMIC8058_REGULATOR_H__

#include <linux/regulator/machine.h>

/* Low dropout regulator ids */
#define PM8058_VREG_ID_L0	0
#define PM8058_VREG_ID_L1	1
#define PM8058_VREG_ID_L2	2
#define PM8058_VREG_ID_L3	3
#define PM8058_VREG_ID_L4	4
#define PM8058_VREG_ID_L5	5
#define PM8058_VREG_ID_L6	6
#define PM8058_VREG_ID_L7	7
#define PM8058_VREG_ID_L8	8
#define PM8058_VREG_ID_L9	9
#define PM8058_VREG_ID_L10	10
#define PM8058_VREG_ID_L11	11
#define PM8058_VREG_ID_L12	12
#define PM8058_VREG_ID_L13	13
#define PM8058_VREG_ID_L14	14
#define PM8058_VREG_ID_L15	15
#define PM8058_VREG_ID_L16	16
#define PM8058_VREG_ID_L17	17
#define PM8058_VREG_ID_L18	18
#define PM8058_VREG_ID_L19	19
#define PM8058_VREG_ID_L20	20
#define PM8058_VREG_ID_L21	21
#define PM8058_VREG_ID_L22	22
#define PM8058_VREG_ID_L23	23
#define PM8058_VREG_ID_L24	24
#define PM8058_VREG_ID_L25	25

/* Switched-mode power supply regulator ids */
#define PM8058_VREG_ID_S0	26
#define PM8058_VREG_ID_S1	27
#define PM8058_VREG_ID_S2	28
#define PM8058_VREG_ID_S3	29
#define PM8058_VREG_ID_S4	30

/* Low voltage switch regulator ids */
#define PM8058_VREG_ID_LVS0	31
#define PM8058_VREG_ID_LVS1	32

/* Negative charge pump regulator id */
#define PM8058_VREG_ID_NCP	33

#define PM8058_VREG_MAX		(PM8058_VREG_ID_NCP + 1)

#define PM8058_VREG_PIN_CTRL_NONE	0x00
#define PM8058_VREG_PIN_CTRL_A0		0x01
#define PM8058_VREG_PIN_CTRL_A1		0x02
#define PM8058_VREG_PIN_CTRL_D0		0x04
#define PM8058_VREG_PIN_CTRL_D1		0x08

/* Pin ctrl enables/disables or toggles high/low power modes */
enum pm8058_vreg_pin_fn {
	PM8058_VREG_PIN_FN_ENABLE = 0,
	PM8058_VREG_PIN_FN_MODE,
};

struct pm8058_vreg_pdata {
	struct regulator_init_data	init_data;
	unsigned			pull_down_enable;
	unsigned			pin_ctrl;
	enum pm8058_vreg_pin_fn		pin_fn;
};

#endif
