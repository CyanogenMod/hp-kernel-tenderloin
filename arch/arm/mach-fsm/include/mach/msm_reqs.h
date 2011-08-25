/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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

#ifndef __ARCH_ARM_MACH_MSM_REQS_H
#define __ARCH_ARM_MACH_MSM_REQS_H

#include <linux/kernel.h>

enum system_bus_flow_ids {
	MSM_AXI_FLOW_INVALID = 0,
	MSM_AXI_FLOW_APPLICATION_LOW,
	MSM_AXI_FLOW_APPLICATION_MED,
	MSM_AXI_FLOW_APPLICATION_HI,
	MSM_AXI_FLOW_APPLICATION_MAX,
	MSM_AXI_FLOW_VIDEO_PLAYBACK_LOW,
	MSM_AXI_FLOW_VIDEO_PLAYBACK_MED,
	MSM_AXI_FLOW_VIDEO_PLAYBACK_HI,
	MSM_AXI_FLOW_VIDEO_PLAYBACK_MAX,
	MSM_AXI_FLOW_VIDEO_RECORD_LOW,
	MSM_AXI_FLOW_VIDEO_RECORD_MED,
	MSM_AXI_FLOW_VIDEO_RECORD_HI,
	MSM_AXI_FLOW_GRAPHICS_LOW,
	MSM_AXI_FLOW_GRAPHICS_MED,
	MSM_AXI_FLOW_GRAPHICS_HI,
	MSM_AXI_FLOW_VIEWFINDER_LOW,
	MSM_AXI_FLOW_VIEWFINDER_MED,
	MSM_AXI_FLOW_VIEWFINDER_HI,
	MSM_AXI_FLOW_LAPTOP_DATA_CALL,
	MSM_AXI_FLOW_APPLICATION_DATA_CALL,
	MSM_AXI_FLOW_GPS,
	MSM_AXI_FLOW_TV_OUT_LOW,
	MSM_AXI_FLOW_TV_OUT_MED,
	MSM_AXI_FLOW_ILCDC_WVGA,
	MSM_AXI_FLOW_VOYAGER_DEFAULT,
	MSM_AXI_FLOW_2D_GPU_HIGH,
	MSM_AXI_FLOW_3D_GPU_HIGH,
	MSM_AXI_FLOW_CAMERA_PREVIEW_HIGH,
	MSM_AXI_FLOW_CAMERA_SNAPSHOT_12MP,
	MSM_AXI_FLOW_CAMERA_RECORDING_720P,
	MSM_AXI_FLOW_JPEG_12MP,
	MSM_AXI_FLOW_MDP_LCDC_WVGA_2BPP,
	MSM_AXI_FLOW_MDP_MDDI_WVGA_2BPP,
	MSM_AXI_FLOW_MDP_DTV_720P_2BPP,
	MSM_AXI_FLOW_VIDEO_RECORDING_720P,
	MSM_AXI_FLOW_VIDEO_PLAYBACK_720P,
	MSM_AXI_FLOW_MDP_TVENC_720P_2BPP,
	MSM_AXI_FLOW_VIDEO_PLAYBACK_WVGA,
	MSM_AXI_FLOW_VIDEO_PLAYBACK_QVGA,
	MSM_AXI_FLOW_VIDEO_RECORDING_QVGA,

	MSM_AXI_NUM_FLOWS,
};

#define MSM_REQ_DEFAULT_VALUE 0

/**
 * msm_req_add - Creates an NPA request and returns a handle. Non-blocking.
 * @req_name:	Name of the request
 * @res_name:	Name of the NPA resource the request is for
 */
void *msm_req_add(char *res_name, char *client_name);

/**
 * msm_req_update - Updates an existing NPA request. May block.
 * @req:	Request handle
 * @value:	Request value
 */
int msm_req_update(void *req, s32 value);

/**
 * msm_req_remove - Removes an existing NPA request. May block.
 * @req:	Request handle
 */
int msm_req_remove(void *req);

#endif /* __ARCH_ARM_MACH_MSM_REQS_H */

