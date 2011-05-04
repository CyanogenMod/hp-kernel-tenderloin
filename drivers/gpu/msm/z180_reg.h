/* Copyright (c) 2002,2007-2011, Code Aurora Forum. All rights reserved.
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
#ifndef __Z80_REG_H
#define __Z80_REG_H

#define REG_VGC_IRQSTATUS__MH_MASK                         0x00000001L
#define REG_VGC_IRQSTATUS__G2D_MASK                        0x00000002L
#define REG_VGC_IRQSTATUS__FIFO_MASK                       0x00000004L

#define	MH_ARBITER_CONFIG__SAME_PAGE_GRANULARITY__SHIFT    0x00000006
#define	MH_ARBITER_CONFIG__L1_ARB_ENABLE__SHIFT            0x00000007
#define	MH_ARBITER_CONFIG__L1_ARB_HOLD_ENABLE__SHIFT       0x00000008
#define	MH_ARBITER_CONFIG__L2_ARB_CONTROL__SHIFT           0x00000009
#define	MH_ARBITER_CONFIG__PAGE_SIZE__SHIFT                0x0000000a
#define	MH_ARBITER_CONFIG__TC_REORDER_ENABLE__SHIFT        0x0000000d
#define	MH_ARBITER_CONFIG__TC_ARB_HOLD_ENABLE__SHIFT       0x0000000e
#define	MH_ARBITER_CONFIG__IN_FLIGHT_LIMIT_ENABLE__SHIFT   0x0000000f
#define	MH_ARBITER_CONFIG__IN_FLIGHT_LIMIT__SHIFT          0x00000010
#define	MH_ARBITER_CONFIG__CP_CLNT_ENABLE__SHIFT           0x00000016
#define	MH_ARBITER_CONFIG__VGT_CLNT_ENABLE__SHIFT          0x00000017
#define	MH_ARBITER_CONFIG__TC_CLNT_ENABLE__SHIFT           0x00000018
#define	MH_ARBITER_CONFIG__RB_CLNT_ENABLE__SHIFT           0x00000019
#define	MH_ARBITER_CONFIG__PA_CLNT_ENABLE__SHIFT           0x0000001a

#define	MH_MMU_CONFIG__RB_W_CLNT_BEHAVIOR__SHIFT           0x00000004
#define	MH_MMU_CONFIG__CP_W_CLNT_BEHAVIOR__SHIFT           0x00000006
#define	MH_MMU_CONFIG__CP_R0_CLNT_BEHAVIOR__SHIFT          0x00000008
#define	MH_MMU_CONFIG__CP_R1_CLNT_BEHAVIOR__SHIFT          0x0000000a
#define	MH_MMU_CONFIG__CP_R2_CLNT_BEHAVIOR__SHIFT          0x0000000c
#define	MH_MMU_CONFIG__CP_R3_CLNT_BEHAVIOR__SHIFT          0x0000000e
#define	MH_MMU_CONFIG__CP_R4_CLNT_BEHAVIOR__SHIFT          0x00000010
#define	MH_MMU_CONFIG__VGT_R0_CLNT_BEHAVIOR__SHIFT         0x00000012
#define	MH_MMU_CONFIG__VGT_R1_CLNT_BEHAVIOR__SHIFT         0x00000014
#define	MH_MMU_CONFIG__TC_R_CLNT_BEHAVIOR__SHIFT           0x00000016
#define	MH_MMU_CONFIG__PA_W_CLNT_BEHAVIOR__SHIFT           0x00000018

#define ADDR_MH_ARBITER_CONFIG           0x0A40
#define ADDR_MH_INTERRUPT_CLEAR          0x0A44
#define ADDR_MH_INTERRUPT_MASK           0x0A42
#define ADDR_MH_INTERRUPT_STATUS         0x0A43
#define ADDR_MH_AXI_ERROR                0x0A45
#define ADDR_MH_AXI_HALT_CONTROL         0x0A50
#define ADDR_MH_CLNT_INTF_CTRL_CONFIG1   0x0A54
#define ADDR_MH_CLNT_INTF_CTRL_CONFIG2   0x0A55
#define ADDR_MH_MMU_CONFIG               0x0040
#define ADDR_MH_MMU_INVALIDATE           0x0045
#define ADDR_MH_MMU_MPU_BASE             0x0046
#define ADDR_MH_MMU_MPU_END              0x0047
#define ADDR_MH_MMU_PT_BASE              0x0042
#define ADDR_MH_MMU_TRAN_ERROR           0x0044
#define ADDR_MH_MMU_VA_RANGE             0x0041
#define ADDR_VGC_MH_READ_ADDR            0x0510
#define ADDR_VGC_MH_DATA_ADDR            0x0518
#define ADDR_MH_MMU_PAGE_FAULT           0x0043
#define ADDR_VGC_COMMANDSTREAM           0x0000
#define ADDR_VGC_IRQENABLE               0x0438
#define ADDR_VGC_IRQSTATUS               0x0418
#define ADDR_VGC_IRQ_ACTIVE_CNT          0x04E0
#define ADDR_VGC_MMUCOMMANDSTREAM        0x03FC
#define ADDR_VGV3_CONTROL                0x0070
#define ADDR_VGV3_LAST                   0x007F
#define ADDR_VGV3_MODE                   0x0071
#define ADDR_VGV3_NEXTADDR               0x0075
#define ADDR_VGV3_NEXTCMD                0x0076
#define ADDR_VGV3_WRITEADDR              0x0072

#endif /* __Z180_REG_H */
