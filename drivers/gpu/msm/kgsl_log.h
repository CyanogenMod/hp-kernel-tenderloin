/* Copyright (c) 2002,2008-2011, Code Aurora Forum. All rights reserved.
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
#ifndef _GSL_LOG_H
#define _GSL_LOG_H

extern unsigned int kgsl_drv_log;
extern unsigned int kgsl_cmd_log;
extern unsigned int kgsl_ctxt_log;
extern unsigned int kgsl_mem_log;
extern unsigned int kgsl_cff_dump_enable;

struct device *kgsl_driver_getdevnode(void);
int kgsl_debug_init(void);

#define KGSL_LOG_VDBG(lvl, fmt, args...) \
	do { \
		if ((lvl) >= 7)  \
			dev_vdbg(kgsl_driver_getdevnode(), "|%s| " fmt, \
					__func__, ##args);\
	} while (0)

#define KGSL_LOG_DBG(lvl, fmt, args...) \
	do { \
		if ((lvl) >= 7)  \
			dev_dbg(kgsl_driver_getdevnode(), "|%s| " fmt, \
					__func__, ##args);\
	} while (0)

#define KGSL_LOG_INFO(lvl, fmt, args...) \
	do { \
		if ((lvl) >= 6)  \
			dev_info(kgsl_driver_getdevnode(), "|%s| " fmt, \
					__func__, ##args);\
	} while (0)

#define KGSL_LOG_WARN(lvl, fmt, args...) \
	do { \
		if ((lvl) >= 4)  \
			dev_warn(kgsl_driver_getdevnode(), "|%s| " fmt, \
					__func__, ##args);\
	} while (0)

#define KGSL_LOG_ERR(lvl, fmt, args...) \
	do { \
		if ((lvl) >= 3)  \
			dev_err(kgsl_driver_getdevnode(), "|%s| " fmt, \
					__func__, ##args);\
	} while (0)

#define KGSL_LOG_FATAL(lvl, fmt, args...) \
	do { \
		if ((lvl) >= 2) \
			dev_crit(kgsl_driver_getdevnode(), "|%s| " fmt, \
					__func__, ##args);\
	} while (0)

#define KGSL_LOG_POSTMORTEM_WRITE(fmt, args...) \
	do { \
		dev_crit(kgsl_driver_getdevnode(), fmt, \
			##args);\
	} while (0)

#define KGSL_LOG_DUMP(fmt, args...)	pr_err(fmt, ##args)

#define KGSL_DRV_VDBG(fmt, args...) KGSL_LOG_VDBG(kgsl_drv_log, fmt, ##args)
#define KGSL_DRV_DBG(fmt, args...)  KGSL_LOG_DBG(kgsl_drv_log, fmt, ##args)
#define KGSL_DRV_INFO(fmt, args...) KGSL_LOG_INFO(kgsl_drv_log, fmt, ##args)
#define KGSL_DRV_WARN(fmt, args...) KGSL_LOG_WARN(kgsl_drv_log, fmt, ##args)
#define KGSL_DRV_ERR(fmt, args...)  KGSL_LOG_ERR(kgsl_drv_log, fmt, ##args)
#define KGSL_DRV_FATAL(fmt, args...) KGSL_LOG_FATAL(kgsl_drv_log, fmt, ##args)

#define KGSL_CMD_VDBG(fmt, args...) KGSL_LOG_VDBG(kgsl_cmd_log, fmt, ##args)
#define KGSL_CMD_DBG(fmt, args...)  KGSL_LOG_DBG(kgsl_cmd_log, fmt, ##args)
#define KGSL_CMD_INFO(fmt, args...) KGSL_LOG_INFO(kgsl_cmd_log, fmt, ##args)
#define KGSL_CMD_WARN(fmt, args...) KGSL_LOG_WARN(kgsl_cmd_log, fmt, ##args)
#define KGSL_CMD_ERR(fmt, args...)  KGSL_LOG_ERR(kgsl_cmd_log, fmt, ##args)
#define KGSL_CMD_FATAL(fmt, args...) KGSL_LOG_FATAL(kgsl_cmd_log, fmt, ##args)

#define KGSL_CTXT_VDBG(fmt, args...) KGSL_LOG_VDBG(kgsl_ctxt_log, fmt, ##args)
#define KGSL_CTXT_DBG(fmt, args...)  KGSL_LOG_DBG(kgsl_ctxt_log, fmt, ##args)
#define KGSL_CTXT_INFO(fmt, args...) KGSL_LOG_INFO(kgsl_ctxt_log, fmt, ##args)
#define KGSL_CTXT_WARN(fmt, args...) KGSL_LOG_WARN(kgsl_ctxt_log, fmt, ##args)
#define KGSL_CTXT_ERR(fmt, args...)  KGSL_LOG_ERR(kgsl_ctxt_log, fmt, ##args)
#define KGSL_CTXT_FATAL(fmt, args...) KGSL_LOG_FATAL(kgsl_ctxt_log, fmt, ##args)

#define KGSL_MEM_VDBG(fmt, args...) KGSL_LOG_VDBG(kgsl_mem_log, fmt, ##args)
#define KGSL_MEM_DBG(fmt, args...)  KGSL_LOG_DBG(kgsl_mem_log, fmt, ##args)
#define KGSL_MEM_INFO(fmt, args...) KGSL_LOG_INFO(kgsl_mem_log, fmt, ##args)
#define KGSL_MEM_WARN(fmt, args...) KGSL_LOG_WARN(kgsl_mem_log, fmt, ##args)
#define KGSL_MEM_ERR(fmt, args...)  KGSL_LOG_ERR(kgsl_mem_log, fmt, ##args)
#define KGSL_MEM_FATAL(fmt, args...) KGSL_LOG_FATAL(kgsl_mem_log, fmt, ##args)

#endif /* _GSL_LOG_H */
