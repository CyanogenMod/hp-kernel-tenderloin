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

#ifndef _VCD_DDL_SHARED_MEM_H_
#define _VCD_DDL_SHARED_MEM_H_

#include "vcd_ddl.h"

#define VIDC_SM_PROFILE_MPEG4_SIMPLE      (0)
#define VIDC_SM_PROFILE_MPEG4_ADV_SIMPLE  (1)

#define VIDC_SM_PROFILE_H264_BASELINE     (0)
#define VIDC_SM_PROFILE_H264_MAIN         (1)
#define VIDC_SM_PROFILE_H264_HIGH         (2)

#define VIDC_SM_PROFILE_H263_BASELINE     (0)

#define VIDC_SM_PROFILE_VC1_SIMPLE        (0)
#define VIDC_SM_PROFILE_VC1_MAIN          (1)
#define VIDC_SM_PROFILE_VC1_ADVANCED      (2)

#define VIDC_SM_PROFILE_MPEG2_MAIN        (4)
#define VIDC_SM_PROFILE_MPEG2_SIMPLE      (5)

#define VIDC_SM_LEVEL_MPEG2_LOW        (10)
#define VIDC_SM_LEVEL_MPEG2_MAIN        (8)
#define VIDC_SM_LEVEL_MPEG2_HIGH_1440   (6)
#define VIDC_SM_LEVEL_MPEG2_HIGH        (4)

#define VIDC_SM_LEVEL_VC1_LOW     (0)
#define VIDC_SM_LEVEL_VC1_MEDIUM  (2)
#define VIDC_SM_LEVEL_VC1_HIGH    (4)

#define VIDC_SM_LEVEL_VC1_ADV_0  (0)
#define VIDC_SM_LEVEL_VC1_ADV_1  (1)
#define VIDC_SM_LEVEL_VC1_ADV_2  (2)
#define VIDC_SM_LEVEL_VC1_ADV_3  (3)
#define VIDC_SM_LEVEL_VC1_ADV_4  (4)

enum VIDC_SM_frame_skip{
	VIDC_SM_FRAME_SKIP_DISABLE      = 0,
	VIDC_SM_FRAME_SKIP_ENABLE_LEVEL = 1,
	VIDC_SM_FRAME_SKIP_ENABLE_VBV   = 2
};
enum VIDC_SM_ref_picture{
	VIDC_SM_REF_PICT_FRAME_OR_TOP_FIELD   = 0,
	VIDC_SM_REF_PICT_BOTTOM_FIELD         = 1
};
void vidc_sm_get_extended_decode_status(struct ddl_buf_addr *shared_mem,
	u32 *pn_decode_status);
void vidc_sm_set_frame_tag(struct ddl_buf_addr *shared_mem,
	u32 frame_tag);
void vidc_sm_get_frame_tags(struct ddl_buf_addr *shared_mem,
	u32 *pn_frame_tag_top, u32 *pn_frame_tag_bottom);
void vidc_sm_get_picture_times(struct ddl_buf_addr *shared_mem,
	u32 *pn_time_top, u32 *pn_time_bottom);
void vidc_sm_set_start_byte_number(struct ddl_buf_addr *shared_mem,
	u32 byte_num);
void vidc_sm_get_crop_info(struct ddl_buf_addr *shared_mem, u32 *pn_left,
	u32 *pn_right, u32 *pn_top, u32 *pn_bottom);
void vidc_sm_get_displayed_picture_frame(struct ddl_buf_addr
	*shared_mem, u32 *n_disp_picture_frame);
void vidc_sm_get_available_luma_dpb_address(
	struct ddl_buf_addr *shared_mem, u32 *pn_free_luma_dpb_address);
void vidc_sm_set_extended_encoder_control(
	struct ddl_buf_addr *shared_mem, u32 hec_enable,
	enum VIDC_SM_frame_skip  frame_skip_mode, u32 seq_hdr_in_band,
	u32 vbv_buffer_size);
void vidc_sm_set_encoder_param_change(struct ddl_buf_addr *shared_mem,
	u32 bit_rate_chg, u32 frame_rate_chg, u32 i_period_chg);
void vidc_sm_set_encoder_vop_time(struct ddl_buf_addr *shared_mem,
	u32 vop_time_enable, u32 time_resolution, u32 frame_delta);
void vidc_sm_set_encoder_hec_period(struct ddl_buf_addr *shared_mem,
	u32 hec_period);
void vidc_sm_get_h264_encoder_reference_list0(
	struct ddl_buf_addr *shared_mem,
	enum VIDC_SM_ref_picture *pe_luma_picture0,
	u32 *pn_luma_picture_index0,
	enum VIDC_SM_ref_picture *pe_luma_picture1,
	u32 *pn_luma_picture_index1,
	enum VIDC_SM_ref_picture *pe_chroma_picture0,
	u32 *pn_chroma_picture_index0,
	enum VIDC_SM_ref_picture *pe_chroma_picture1,
	u32 *pn_chroma_picture_index1);

void vidc_sm_get_h264_encoder_reference_list1(
	struct ddl_buf_addr *shared_mem,
	enum VIDC_SM_ref_picture *pe_luma_picture,
	u32 *pn_luma_picture_index,
	enum VIDC_SM_ref_picture *pe_chroma_picture,
	u32 *pn_chroma_picture_index);
void vidc_sm_set_allocated_dpb_size(struct ddl_buf_addr *shared_mem,
	u32 y_size, u32 c_size);
void vidc_sm_set_allocated_h264_mv_size(struct ddl_buf_addr *shared_mem,
	u32 mv_size);
void vidc_sm_get_min_yc_dpb_sizes(struct ddl_buf_addr *shared_mem,
	u32 *pn_min_luma_dpb_size, u32 *pn_min_chroma_dpb_size);
void vidc_sm_set_metadata_enable(struct ddl_buf_addr *shared_mem,
	u32 extradata_enable, u32 qp_enable, u32 concealed_mb_enable,
	u32 vc1Param_enable, u32 sei_nal_enable, u32 vui_enable,
	u32 enc_slice_size_enable);
void vidc_sm_get_metadata_status(struct ddl_buf_addr *shared_mem,
	u32 *pb_metadata_present);
void vidc_sm_get_metadata_display_index(struct ddl_buf_addr *shared_mem,
	u32 *pn_dixplay_index);
void vidc_sm_set_metadata_start_address(struct ddl_buf_addr *shared_mem,
	u32 address);
void vidc_sm_set_extradata_presence(struct ddl_buf_addr *shared_mem,
	u32 extradata_present);
void vidc_sm_set_extradata_addr(struct ddl_buf_addr *shared_mem,
	u32 extradata_addr);
void vidc_sm_set_pand_b_frame_qp(struct ddl_buf_addr *shared_mem,
	u32 b_frame_qp, u32 p_frame_qp);
void vidc_sm_get_profile_info(struct ddl_buf_addr *shared_mem,
	u32 *pn_disp_profile_info, u32 *pn_disp_level_info);
void vidc_sm_set_encoder_new_bit_rate(struct ddl_buf_addr *shared_mem,
	u32 new_bit_rate);
void vidc_sm_set_encoder_new_frame_rate(struct ddl_buf_addr *shared_mem,
	u32 new_frame_rate);
void vidc_sm_set_encoder_new_i_period(struct ddl_buf_addr *shared_mem,
	u32 new_i_period);
void vidc_sm_set_encoder_init_rc_value(struct ddl_buf_addr *shared_mem,
	u32 new_rc_value);

#endif
