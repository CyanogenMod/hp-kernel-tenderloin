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
#ifndef _LEIA_REG_H
#define _LEIA_REG_H

#define REG_LEIA_PC_INDX_OFFSET          REG_VGT_INDX_OFFSET
#define REG_LEIA_PC_VERTEX_REUSE_BLOCK_CNTL REG_VGT_VERTEX_REUSE_BLOCK_CNTL
#define REG_LEIA_PC_MAX_VTX_INDX         REG_VGT_MAX_VTX_INDX
#define REG_LEIA_RB_LRZ_VSC_CONTROL	 0x2209
#define REG_LEIA_GRAS_CONTROL            0x2210
#define REG_LEIA_VSC_BIN_SIZE            0x0C01
#define REG_LEIA_VSC_PIPE_DATA_LENGTH_7  0x0C1D


#define REG_LEIA_PC_DEBUG_CNTL		 0x0C38
#define REG_LEIA_PC_DEBUG_DATA		 0x0C39
#define REG_LEIA_RB_DEBUG_CNTL		 0x0f26
#define REG_LEIA_RB_DEBUG_DATA		 0x0f27
#define REG_LEIA_GRAS_DEBUG_CNTL	 0x0c80
#define REG_LEIA_GRAS_DEBUG_DATA	 0x0c81

#define REG_LEIA_SQ_DEBUG_CONST_MGR_FSM  0x0daf
#define REG_LEIA_SQ_DEBUG_EXP_ALLOC	 0x0db3
#define REG_LEIA_SQ_DEBUG_FSM_ALU_0	 0x0db1
#define REG_LEIA_SQ_DEBUG_FSM_ALU_1	 0x0db2
#define REG_LEIA_SQ_DEBUG_GPR_PIX	 0x0db6
#define REG_LEIA_SQ_DEBUG_GPR_VTX	 0x0db5
#define REG_LEIA_SQ_DEBUG_INPUT_FSM	 0x0dae
#define REG_LEIA_SQ_DEBUG_MISC		 0x0d05
#define REG_LEIA_SQ_DEBUG_MISC_0	 0x2309
#define REG_LEIA_SQ_DEBUG_MISC_1	 0x230a
#define REG_LEIA_SQ_DEBUG_PIX_TB_0	 0x0dbc
#define REG_LEIA_SQ_DEBUG_PIX_TB_STATE_MEM 0x0dc1
#define REG_LEIA_SQ_DEBUG_PIX_TB_STATUS_REG_0 0x0dbd
#define REG_LEIA_SQ_DEBUG_PIX_TB_STATUS_REG_1 0x0dbe
#define REG_LEIA_SQ_DEBUG_PIX_TB_STATUS_REG_2 0x0dbf
#define REG_LEIA_SQ_DEBUG_PIX_TB_STATUS_REG_3 0x0dc0
#define REG_LEIA_SQ_DEBUG_PTR_BUFF	 0x0db4
#define REG_LEIA_SQ_DEBUG_TB_STATUS_SEL  0x0db7
#define REG_LEIA_SQ_DEBUG_TP_FSM	 0x0db0
#define REG_LEIA_SQ_DEBUG_VTX_TB_0	 0x0db8
#define REG_LEIA_SQ_DEBUG_VTX_TB_1	 0x0db9
#define REG_LEIA_SQ_DEBUG_VTX_TB_STATE_MEM 0x0dbb
#define REG_LEIA_SQ_DEBUG_VTX_TB_STATE_REG 0x0dba

#define REG_LEIA_MH_DEBUG_CNTL 0x0a4e
#define REG_LEIA_MH_DEBUG_DATA 0x0a4f
#define REG_LEIA_RBBM_DEBUG_CNTL 0x3a1
#define REG_LEIA_RBBM_DEBUG_OUT 0x3a0
#define REG_LEIA_CP_DEBUG 0x1fc

#define REG_LEIA_CP_STATE_DEBUG_INDEX 0x1ec
#define REG_LEIA_CP_STATE_DEBUG_DATA 0x1ed


#define REG_LEIA_SQ_DEBUG_VTX_TB_STATUS_REG 0xdba

#define REG_LEIA_SQ_RESOURCE_MANAGMENT 0xd03

#define REG_LEIA_SQ_PIX_IN_CNTL 0xd0c
#define REG_LEIA_CP_ME_STATUS 0x1f7

#endif /*_LEIA_REG_H*/
