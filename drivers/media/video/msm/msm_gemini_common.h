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
 */

#ifndef MSM_GEMINI_COMMON_H
#define MSM_GEMINI_COMMON_H

#define MSM_GEMINI_DEBUG
#ifdef MSM_GEMINI_DEBUG
#define GMN_DBG(fmt, args...) pr_debug(fmt, ##args)
#else
#define GMN_DBG(fmt, args...) do { } while (0)
#endif

#define GMN_PR_ERR   pr_err

enum GEMINI_MODE {
	GEMINI_MODE_DISABLE,
	GEMINI_MODE_OFFLINE,
	GEMINI_MODE_REALTIME,
	GEMINI_MODE_REALTIME_ROTATION
};

enum GEMINI_ROTATION {
	GEMINI_ROTATION_0,
	GEMINI_ROTATION_90,
	GEMINI_ROTATION_180,
	GEMINI_ROTATION_270
};

#endif /* MSM_GEMINI_COMMON_H */
