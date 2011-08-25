/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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
#ifndef __MACH_QDSP5_V2_ADIE_MARIMBA_H
#define __MACH_QDSP5_V2_ADIE_MARIMBA_H

#include <linux/types.h>

/* Value Represents a entry */
#define ADIE_CODEC_ACTION_ENTRY       0x1
/* Value representing a delay wait */
#define ADIE_CODEC_ACTION_DELAY_WAIT      0x2
/* Value representing a stage reached */
#define ADIE_CODEC_ACTION_STAGE_REACHED   0x3

/* This value is the state after the client sets the path */
#define ADIE_CODEC_PATH_OFF                                        0x0050

/* State to which client asks the drv to proceed to where it can
 * set up the clocks and 0-fill PCM buffers
 */
#define ADIE_CODEC_DIGITAL_READY                                   0x0100

/* State to which client asks the drv to proceed to where it can
 * start sending data after internal steady state delay
 */
#define ADIE_CODEC_DIGITAL_ANALOG_READY                            0x1000


/*  Client Asks adie to switch off the Analog portion of the
 *  the internal codec. After the use of this path
 */
#define ADIE_CODEC_ANALOG_OFF                                      0x0750


/* Client Asks adie to switch off the digital portion of the
 *  the internal codec. After switching off the analog portion.
 *
 *  0-fill PCM may or maynot be sent at this point
 *
 */
#define ADIE_CODEC_DIGITAL_OFF                                     0x0600

/* State to which client asks the drv to write the default values
 * to the registers */
#define ADIE_CODEC_FLASH_IMAGE 					   0x0001

/* Path type */
#define ADIE_CODEC_RX 0
#define ADIE_CODEC_TX 1
#define ADIE_CODEC_LB 3
#define ADIE_CODEC_MAX 4

#define ADIE_CODEC_PACK_ENTRY(reg, mask, val) ((val)|(mask << 8)|(reg << 16))

#define ADIE_CODEC_UNPACK_ENTRY(packed, reg, mask, val) \
	do { \
		((reg) = ((packed >> 16) & (0xff))); \
		((mask) = ((packed >> 8) & (0xff))); \
		((val) = ((packed) & (0xff))); \
	} while (0);

struct adie_codec_action_unit {
	u32 type;
	u32 action;
};

struct adie_codec_hwsetting_entry{
	struct adie_codec_action_unit *actions;
	u32 action_sz;
	u32 freq_plan;
	u32 osr;
	/* u32  VolMask;
	 * u32  SidetoneMask;
	 */
};

struct adie_codec_dev_profile {
	u32 path_type; /* RX or TX */
	u32 setting_sz;
	struct adie_codec_hwsetting_entry *settings;
};

#endif
