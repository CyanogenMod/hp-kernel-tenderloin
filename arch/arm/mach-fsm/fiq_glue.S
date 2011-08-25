/* arch/arm/mach-msm/fiq_glue.S
 *
 * Copyright (C) 2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/linkage.h>
#include <asm/assembler.h>

		.text

		.global fiq_glue_end

ENTRY(fiq_glue)
		adr		r12, fiq_glue_ctxt_addr
		ldr		r8, [r12]
		ldmia	r8, {r9, r10, sp}

		/* store pc, cpsr from previous mode */
		sub		r11, lr, #4
		mrs		r12, spsr
		stmfd	sp!, {r11-r12,lr}

		/* store r8-r14 from previous mode */
		sub		sp, sp, #(7 * 4)
		stmia	sp, {r8-r14}^
		nop

		/* store r0-r7 from previous mode */
		stmfd	sp!, {r0-r7}

		/* call func(data,regs) */
		mov		r0, r10
		mov		r1, sp
		blx		r9

		/* restore/discard saved state */
		ldmfd	sp!, {r0-r7}
		add		sp, sp, #(9 * 4)
		ldmfd	sp!, {lr}

		subs	pc, lr, #4

fiq_glue_ctxt_addr:
		.long fiq_glue_ctxt
fiq_glue_end:

ENTRY(fiq_glue_setup) /* func, data, sp */
		ldr		r3, =fiq_glue_ctxt
		stmia	r3, {r0-r2}
		bx		lr

		.data
fiq_glue_ctxt:
		.long 0, 0, 0
