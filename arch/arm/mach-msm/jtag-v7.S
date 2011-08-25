/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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

/*
 * JTAG support functions for ARMv7-based Qualcomm SoCs.
 */
#include <linux/linkage.h>
#include <asm/assembler.h>

ENTRY(msm_save_jtag_debug)
	/* lock debug and preserve registers through power collapse */
	ldr     r3, =dbg_state        /* store state at dbg_state */

	ldr     r1, =0xc5ACCE55       /* set DBGOSLAR lock */
	mcr     p14,0,r1,c1,c0,4
	isb

	mrc     p14,0,r1,c1,c2,4      /* DBGOSSRR state register count */

	cmp     r1, #(0x20-1)         /* check for state overflow */
	movge   r1, #0                /* if not enough space, don't save */

	str     r1,[r3],#4            /* save count for restore */

1:	cmp     r1,#0
	mrcne   p14,0,r2,c1,c2,4      /* DBGOSSRR state value */
	strne   r2,[r3],#4            /* push value */
	subne   r1,r1,#1
	bne     1b

	/* unlock JTAG.  Works better than leaving locked. */
	stmfd   sp!, {lr}
	bl      msm_unlock_jtag_debug
	ldmfd   sp!, {lr}
	bx      lr

ENTRY(msm_unlock_jtag_debug)
	mov     r0, #0                /* unlock value */
	mcr     p14,0,r0,c1,c0,4      /* unlock DBGOSLAR */
	isb
	bx      lr

ENTRY(msm_restore_jtag_debug)
	/* restore debug registers after power collapse  */
	ldr     r3, =dbg_state        /* load state from dbg_state */

	ldr     r1, =0xc5ACCE55       /* set DBGOSLAR lock */
	mcr     p14,0,r1,c1,c0,4
	isb

	mrc     p14,0,r1,c1,c2,4      /* DBGOSSRR dummy read (required)*/
	ldr     r1,[r3],#4            /* load saved count */
	cmp     r1,#0                 /* skip if none stored
	beq     msm_pm_dbg_restore_end

	/* restores debug state except DBGDSCR */
1:	ldr     r2,[r3],#4
	cmp     r1,#0x10              /* DBGDSCR special case */
	biceq   r2,r2,#0xc000         /* DBGDSCR = DBGDSCR & ~0xc000 */
	mcr     p14,0,r2,c1,c2,4      /* DBGOSSRR write state value */
	subs    r1,r1,#1
	bne     1b
	isb

	/* second loop to restore DBGDSCR after other state restored */
	ldr     r3, =dbg_state        /* load state from dbg_state */

	ldr     r1, =0xc5ACCE55       /* set DBGOSLAR lock */
	mcr     p14,0,r1,c1,c0,4
	isb

	mrc     p14,0,r1,c1,c5,4      /* clear sticky power down bit */
	isb

	mrc     p14,0,r1,c1,c2,4      /* DBGOSSRR dummy read (required)*/
	ldr     r1,[r3],#4            /* load saved count */

1:	ldr     r2,[r3],#4
	mcr     p14,0,r2,c1,c2,4      /* DBGOSSRR write state value */
	subs    r1,r1,#1
	bne     1b
msm_pm_dbg_restore_end:
	mcr     p14,0,r1,c1,c0,4      /* unlock DBGOSLAR */
	isb
	bx      lr


	.data

dbg_state:
	.space  4 * 0x20

