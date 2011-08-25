/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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
 */

#ifndef _INTERNAL_POWER_RAIL_H
#define _INTERNAL_POWER_RAIL_H

/* Clock power rail IDs */
#define PWR_RAIL_GRP_CLK	8
#define PWR_RAIL_GRP_2D_CLK	58
#define PWR_RAIL_MDP_CLK	14
#define PWR_RAIL_MFC_CLK	68
#define PWR_RAIL_ROTATOR_CLK	90
#define PWR_RAIL_VDC_CLK	39
#define PWR_RAIL_VFE_CLK	41
#define PWR_RAIL_VPE_CLK	76

enum rail_ctl_mode {
	PWR_RAIL_CTL_AUTO = 0,
	PWR_RAIL_CTL_MANUAL,
};

#ifdef CONFIG_ARCH_MSM8X60
static inline int __maybe_unused internal_pwr_rail_ctl(unsigned rail_id,
						       bool enable)
{
	/* Not yet implemented. */
	return 0;
}
static inline int __maybe_unused internal_pwr_rail_mode(unsigned rail_id,
							enum rail_ctl_mode mode)
{
	/* Not yet implemented. */
	return 0;
}
#else
int internal_pwr_rail_ctl(unsigned rail_id, bool enable);
int internal_pwr_rail_mode(unsigned rail_id, enum rail_ctl_mode mode);
#endif

#endif /* _INTERNAL_POWER_RAIL_H */

