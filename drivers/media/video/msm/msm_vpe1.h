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
 *		 contributors may be used to endorse or promote products derived
 *		 from this software without specific prior written permission.
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
 */

#ifndef _msm_vpe1_h_
#define _msm_vpe1_h_

#include <mach/camera.h>

/***********  start of register offset *********************/
#define VPE_INTR_ENABLE_OFFSET                0x0020
#define VPE_INTR_STATUS_OFFSET                0x0024
#define VPE_INTR_CLEAR_OFFSET                 0x0028
#define VPE_DL0_START_OFFSET                  0x0030
#define VPE_HW_VERSION_OFFSET                 0x0070
#define VPE_SW_RESET_OFFSET                   0x0074
#define VPE_AXI_RD_ARB_CONFIG_OFFSET          0x0078
#define VPE_SEL_CLK_OR_HCLK_TEST_BUS_OFFSET   0x007C
#define VPE_CGC_EN_OFFSET                     0x0100
#define VPE_CMD_STATUS_OFFSET                 0x10008
#define VPE_PROFILE_EN_OFFSET                 0x10010
#define VPE_PROFILE_COUNT_OFFSET              0x10014
#define VPE_CMD_MODE_OFFSET                   0x10060
#define VPE_SRC_SIZE_OFFSET                   0x10108
#define VPE_SRCP0_ADDR_OFFSET                 0x1010C
#define VPE_SRCP1_ADDR_OFFSET                 0x10110
#define VPE_SRC_YSTRIDE1_OFFSET               0x1011C
#define VPE_SRC_FORMAT_OFFSET                 0x10124
#define VPE_SRC_UNPACK_PATTERN1_OFFSET        0x10128
#define VPE_OP_MODE_OFFSET                    0x10138
#define VPE_SCALE_PHASEX_INIT_OFFSET          0x1013C
#define VPE_SCALE_PHASEY_INIT_OFFSET          0x10140
#define VPE_SCALE_PHASEX_STEP_OFFSET          0x10144
#define VPE_SCALE_PHASEY_STEP_OFFSET          0x10148
#define VPE_OUT_FORMAT_OFFSET                 0x10150
#define VPE_OUT_PACK_PATTERN1_OFFSET          0x10154
#define VPE_OUT_SIZE_OFFSET                   0x10164
#define VPE_OUTP0_ADDR_OFFSET                 0x10168
#define VPE_OUTP1_ADDR_OFFSET                 0x1016C
#define VPE_OUT_YSTRIDE1_OFFSET               0x10178
#define VPE_OUT_XY_OFFSET                     0x1019C
#define VPE_SRC_XY_OFFSET                     0x10200
#define VPE_SRC_IMAGE_SIZE_OFFSET             0x10208
#define VPE_SCALE_CONFIG_OFFSET               0x10230
#define VPE_DEINT_STATUS_OFFSET               0x30000
#define VPE_DEINT_DECISION_OFFSET             0x30004
#define VPE_DEINT_COEFF0_OFFSET               0x30010
#define VPE_SCALE_STATUS_OFFSET               0x50000
#define VPE_SCALE_SVI_PARAM_OFFSET            0x50010
#define VPE_SCALE_SHARPEN_CFG_OFFSET          0x50020
#define VPE_SCALE_COEFF_LSP_0_OFFSET          0x50400
#define VPE_SCALE_COEFF_MSP_0_OFFSET          0x50404

#define VPE_SCALE_COEFF_LSBn(n)	(0x50400 + 8 * (n))
#define VPE_SCALE_COEFF_MSBn(n)	(0x50404 + 8 * (n))
#define VPE_SCALE_COEFF_NUM			32

/*********** end of register offset ********************/


#define VPE_HARDWARE_VERSION          0x00080308
#define VPE_SW_RESET_VALUE            0x00000010  /* bit 4 for PPP*/
#define VPE_AXI_RD_ARB_CONFIG_VALUE   0x124924
#define VPE_CMD_MODE_VALUE        0x1
#define VPE_DEFAULT_OP_MODE_VALUE     0x40FC0004
#define VPE_CGC_ENABLE_VALUE          0xffff
#define VPE_DEFAULT_SCALE_CONFIG      0x3c

#define VPE_CLOCK_RATE   160000000
/**************************************************/
/*********** Start of command id ******************/
/**************************************************/
enum VPE_CMD_ID_ENUM {
	VPE_DUMMY_0 = 0,
	VPE_SET_CLK,
	VPE_RESET,
	VPE_START,
	VPE_ABORT,
	VPE_OPERATION_MODE_CFG, /* 5 */
	VPE_INPUT_PLANE_CFG,
	VPE_OUTPUT_PLANE_CFG,
	VPE_INPUT_PLANE_UPDATE,
	VPE_SCALE_CFG_TYPE,
	VPE_ROTATION_CFG_TYPE, /* 10 */
	VPE_AXI_OUT_CFG,
	VPE_CMD_DIS_OFFSET_CFG,
};

/* Length of each command.  In bytes.  (payload only) */
#define VPE_OPERATION_MODE_CFG_LEN 8
#define VPE_INPUT_PLANE_CFG_LEN    24
#define VPE_OUTPUT_PLANE_CFG_LEN   20
#define VPE_INPUT_PLANE_UPDATE_LEN 12
#define VPE_SCALER_CONFIG_LEN      260
#define VPE_DIS_OFFSET_CFG_LEN     12
/**************************************************/
/*********** End of command id ********************/
/**************************************************/

struct msm_vpe_cmd {
	int32_t  id;
	uint16_t length;
	void     *value;
};

struct vpe_cmd_type {
	uint16_t id;
	uint32_t length;
};

struct vpe_isr_queue_cmd_type {
	struct list_head            list;
	uint32_t                    irq_status;
};

enum VPE_MESSAGE_ID {
    MSG_ID_VPE_OUTPUT_V = 7, /* To match with that of VFE */
};

struct vpe_device_type {
	/* device related. */
	int   vpeirq;
	void __iomem      *vpebase;
	struct resource	  *vpemem;
	struct resource   *vpeio;
	void        *device_extdata;
};

struct dis_offset_type {
    int32_t  dis_offset_x;
	int32_t  dis_offset_y;
    uint32_t  frame_id;
};

struct vpe_ctrl_type {
	spinlock_t        tasklet_lock;
	spinlock_t        state_lock;

	struct list_head  tasklet_q;
	void              *syncdata;
	uint16_t          op_mode;
	void              *extdata;
	uint32_t          extlen;
	struct msm_vpe_callback *resp;
	uint32_t           in_h_w;
	uint32_t          out_h;  /* this is BEFORE rotation. */
	uint32_t          out_w;  /* this is BEFORE rotation. */
	uint32_t          dis_en;
	uint32_t          state;
	struct timespec   ts;
	struct dis_offset_type   dis_offset;
	uint32_t          pcbcr_before_dis;
	uint32_t          pcbcr_dis_offset;
};

/*
* vpe_input_update
*
* Define the parameters for output plane
*/
/* this is the dimension of ROI.  width / height. */
struct vpe_src_size_packed {
	uint32_t        src_w;
	uint32_t        src_h;
};

struct vpe_src_xy_packed {
	uint32_t        src_x;
	uint32_t        src_y;
};

struct vpe_input_plane_update_type {
	struct vpe_src_size_packed             src_roi_size;
	/* DIS updates this set. */
	struct vpe_src_xy_packed               src_roi_offset;
	/* input address*/
	uint8_t                         *src_p0_addr;
	uint8_t                         *src_p1_addr;
};

struct vpe_msg_stats{
	uint32_t    buffer;
	uint32_t    frameCounter;
};

struct vpe_msg_output {
	uint8_t   output_id;
	uint32_t  yBuffer;
	uint32_t  cbcrBuffer;
	uint32_t  frameCounter;
};

struct vpe_message {
	uint8_t  _d;
	union {
		struct vpe_msg_output              msgOut;
		struct vpe_msg_stats               msgStats;
	} _u;
};

#define SCALER_PHASE_BITS 29
#define HAL_MDP_PHASE_STEP_2P50    0x50000000
#define HAL_MDP_PHASE_STEP_1P66    0x35555555
#define HAL_MDP_PHASE_STEP_1P25    0x28000000

struct phase_val_t {
	int32_t phase_init_x;
	int32_t phase_init_y;
	int32_t phase_step_x;
	int32_t phase_step_y;
};

extern struct vpe_ctrl_type    *vpe_ctrl;

int msm_vpe_open(void);
int msm_vpe_release(void);
int msm_vpe_reg(struct msm_vpe_callback *presp);
void msm_send_frame_to_vpe(uint32_t pyaddr, uint32_t pcbcraddr,
				struct timespec *ts);
int msm_vpe_config(struct msm_vpe_cfg_cmd *cmd, void *data);
int msm_vpe_cfg_update(void *pinfo);

#endif /*_msm_vpe1_h_*/

