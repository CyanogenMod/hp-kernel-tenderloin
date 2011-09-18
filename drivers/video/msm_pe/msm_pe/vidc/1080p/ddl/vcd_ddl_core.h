/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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

#ifndef _VCD_DDL_CORE_H_
#define _VCD_DDL_CORE_H_

#define DDL_LINEAR_BUF_ALIGN_MASK         0xFFFFF800U
#define DDL_LINEAR_BUF_ALIGN_GUARD_BYTES  0x7FF
#define DDL_LINEAR_BUFFER_ALIGN_BYTES     2048
#define DDL_TILE_BUF_ALIGN_MASK           0xFFFFE000U
#define DDL_TILE_BUF_ALIGN_GUARD_BYTES    0x1FFF
#define DDL_TILE_BUFFER_ALIGN_BYTES       8192

#define DDL_YUV_BUF_TYPE_LINEAR 0
#define DDL_YUV_BUF_TYPE_TILE   1

#define DDL_NO_OF_MB(nWidth, nHeight) \
	((((nWidth) + 15) >> 4) * (((nHeight) + 15) >> 4))

#define DDL_MAX_FRAME_WIDTH   1920
#define DDL_MAX_FRAME_HEIGHT  1088

#define MAX_DPB_SIZE_L4PT0_MBS    DDL_KILO_BYTE(32)
#define MAX_FRAME_SIZE_L4PT0_MBS  DDL_KILO_BYTE(8)

#define DDL_MAX_MB_PER_FRAME (DDL_NO_OF_MB(DDL_MAX_FRAME_WIDTH,\
	DDL_MAX_FRAME_HEIGHT))

#define DDL_DB_LINE_BUF_SIZE\
	(((((DDL_MAX_FRAME_WIDTH * 4) - 1) / 256) + 1) * 8 * 1024)

#define DDL_MAX_FRAME_RATE               120
#define DDL_INITIAL_FRAME_RATE            30

#define DDL_MAX_BIT_RATE    (20*1024*1024)
#define DDL_MAX_MB_PER_SEC  (DDL_MAX_MB_PER_FRAME * DDL_INITIAL_FRAME_RATE)

#define DDL_SW_RESET_SLEEP               1
#define VCD_MAX_NO_CLIENT                4
#define VCD_SINGLE_FRAME_COMMAND_CHANNEL 1
#define VCD_DUAL_FRAME_COMMAND_CHANNEL   2
#define VCD_FRAME_COMMAND_DEPTH          VCD_SINGLE_FRAME_COMMAND_CHANNEL
#define VCD_GENEVIDC_COMMAND_DEPTH        1
#define VCD_COMMAND_EXCLUSIVE            true
#define DDL_HW_TIMEOUT_IN_MS             1000
#define DDL_STREAMBUF_ALIGN_GUARD_BYTES  0x7FF

#define DDL_CONTEXT_MEMORY (1024 * 15 * (VCD_MAX_NO_CLIENT + 1))

#define DDL_ENC_MIN_DPB_BUFFERS           2
#define DDL_ENC_MAX_DPB_BUFFERS           4

#define DDL_FW_AUX_HOST_CMD_SPACE_SIZE         (DDL_KILO_BYTE(10))
#define DDL_FW_INST_GLOBAL_CONTEXT_SPACE_SIZE  (DDL_KILO_BYTE(400))
#define DDL_FW_H264DEC_CONTEXT_SPACE_SIZE      (DDL_KILO_BYTE(800))
#define DDL_FW_OTHER_CONTEXT_SPACE_SIZE        (DDL_KILO_BYTE(10))

#define VCD_DEC_CPB_SIZE         (DDL_KILO_BYTE(512))
#define DDL_DBG_CORE_DUMP_SIZE   (DDL_KILO_BYTE(10))

#define DDL_BUFEND_PAD                    256
#define DDL_ENC_SEQHEADER_SIZE            (512+DDL_BUFEND_PAD)
#define DDL_MAX_BUFFER_COUNT              32

#define DDL_MPEG_REFBUF_COUNT             2
#define DDL_MPEG_COMV_BUF_NO              2
#define DDL_H263_COMV_BUF_NO              0
#define DDL_COMV_BUFLINE_NO               128
#define DDL_VC1_COMV_BUFLINE_NO           32

#define DDL_MAX_H264_QP            51
#define DDL_MAX_MPEG4_QP           31

#define DDL_ALLOW_DEC_FRAMESIZE(width, height) \
	((DDL_NO_OF_MB(width, height) <= \
	MAX_FRAME_SIZE_L4PT0_MBS) && \
	(width <= DDL_MAX_FRAME_WIDTH) && \
	(height <= DDL_MAX_FRAME_WIDTH) && \
	((width >= 32 && height >= 16) || \
	(width >= 16 && height >= 32)))

#define DDL_ALLOW_ENC_FRAMESIZE(width, height) \
	((DDL_NO_OF_MB(width, height) <= \
	MAX_FRAME_SIZE_L4PT0_MBS) && \
	(width <= DDL_MAX_FRAME_WIDTH) && \
	(height <= DDL_MAX_FRAME_WIDTH) && \
	((width >= 32 && height >= 32)))

#define DDL_LINEAR_ALIGN_WIDTH      16
#define DDL_LINEAR_ALIGN_HEIGHT     16
#define DDL_LINEAR_MULTIPLY_FACTOR  2048
#define DDL_TILE_ALIGN_WIDTH        128
#define DDL_TILE_ALIGN_HEIGHT       32
#define DDL_TILE_MULTIPLY_FACTOR    8192
#define DDL_TILE_ALIGN(val, grid) \
	(((val) + (grid) - 1) / (grid) * (grid))

#define VCD_DDL_720P_YUV_BUF_SIZE     ((1280*720*3) >> 1)
#define VCD_DDL_WVGA_BUF_SIZE         (800*480)

#define VCD_DDL_TEST_MAX_WIDTH        (DDL_MAX_FRAME_WIDTH)
#define VCD_DDL_TEST_MAX_HEIGHT       (DDL_MAX_FRAME_HEIGHT)

#define VCD_DDL_TEST_MAX_NUM_H264_DPB  8

#define VCD_DDL_TEST_NUM_ENC_INPUT_BUFS   6
#define VCD_DDL_TEST_NUM_ENC_OUTPUT_BUFS  4

#define VCD_DDL_TEST_DEFAULT_WIDTH       176
#define VCD_DDL_TEST_DEFAULT_HEIGHT      144

#define DDL_PIXEL_CACHE_NOT_IDLE          0x4000
#define DDL_PIXEL_CACHE_STATUS_READ_RETRY 10
#define DDL_PIXEL_CACHE_STATUS_READ_SLEEP 200

#endif
