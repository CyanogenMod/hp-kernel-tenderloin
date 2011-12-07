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

extern unsigned int kgsl_cff_dump_enable;

#define KGSL_LOG_INFO(dev, lvl, fmt, args...) \
	do { \
		if ((lvl) >= 6)  \
			dev_info(dev, "|%s| " fmt, \
					__func__, ##args);\
	} while (0)

#define KGSL_LOG_WARN(dev, lvl, fmt, args...) \
	do { \
		if ((lvl) >= 4)  \
			dev_warn(dev, "|%s| " fmt, \
					__func__, ##args);\
	} while (0)

#define KGSL_LOG_ERR(dev, lvl, fmt, args...) \
	do { \
		if ((lvl) >= 3)  \
			dev_err(dev, "|%s| " fmt, \
					__func__, ##args);\
	} while (0)

#define KGSL_LOG_CRIT(dev, lvl, fmt, args...) \
	do { \
		if ((lvl) >= 2) \
			dev_crit(dev, "|%s| " fmt, \
					__func__, ##args);\
	} while (0)

#define KGSL_LOG_POSTMORTEM_WRITE(_dev, fmt, args...) \
	do { dev_crit(_dev->dev, fmt, ##args); } while (0)

#define KGSL_LOG_DUMP(_dev, fmt, args...)	dev_err(_dev->dev, fmt, ##args)

#define KGSL_DRV_INFO(_dev, fmt, args...) \
KGSL_LOG_INFO(_dev->dev, _dev->drv_log, fmt, ##args)
#define KGSL_DRV_WARN(_dev, fmt, args...) \
KGSL_LOG_WARN(_dev->dev, _dev->drv_log, fmt, ##args)
#define KGSL_DRV_ERR(_dev, fmt, args...)  \
KGSL_LOG_ERR(_dev->dev, _dev->drv_log, fmt, ##args)
#define KGSL_DRV_CRIT(_dev, fmt, args...) \
KGSL_LOG_CRIT(_dev->dev, _dev->drv_log, fmt, ##args)

#define KGSL_CMD_INFO(_dev, fmt, args...) \
KGSL_LOG_INFO(_dev->dev, _dev->cmd_log, fmt, ##args)
#define KGSL_CMD_WARN(_dev, fmt, args...) \
KGSL_LOG_WARN(_dev->dev, _dev->cmd_log, fmt, ##args)
#define KGSL_CMD_ERR(_dev, fmt, args...) \
KGSL_LOG_ERR(_dev->dev, _dev->cmd_log, fmt, ##args)
#define KGSL_CMD_CRIT(_dev, fmt, args...) \
KGSL_LOG_CRIT(_dev->dev, _dev->cmd_log, fmt, ##args)

#define KGSL_CTXT_INFO(_dev, fmt, args...) \
KGSL_LOG_INFO(_dev->dev, _dev->ctxt_log, fmt, ##args)
#define KGSL_CTXT_WARN(_dev, fmt, args...) \
KGSL_LOG_WARN(_dev->dev, _dev->ctxt_log, fmt, ##args)
#define KGSL_CTXT_ERR(_dev, fmt, args...)  \
KGSL_LOG_ERR(_dev->dev, _dev->ctxt_log, fmt, ##args)
#define KGSL_CTXT_CRIT(_dev, fmt, args...) \
KGSL_LOG_CRIT(_dev->dev, _dev->ctxt_log, fmt, ##args)

#define KGSL_MEM_INFO(_dev, fmt, args...) \
KGSL_LOG_INFO(_dev->dev, _dev->mem_log, fmt, ##args)
#define KGSL_MEM_WARN(_dev, fmt, args...) \
KGSL_LOG_WARN(_dev->dev, _dev->mem_log, fmt, ##args)
#define KGSL_MEM_ERR(_dev, fmt, args...)  \
KGSL_LOG_ERR(_dev->dev, _dev->mem_log, fmt, ##args)
#define KGSL_MEM_CRIT(_dev, fmt, args...) \
KGSL_LOG_CRIT(_dev->dev, _dev->mem_log, fmt, ##args)

#define KGSL_PWR_INFO(_dev, fmt, args...) \
KGSL_LOG_INFO(_dev->dev, _dev->pwr_log, fmt, ##args)
#define KGSL_PWR_WARN(_dev, fmt, args...) \
KGSL_LOG_WARN(_dev->dev, _dev->pwr_log, fmt, ##args)
#define KGSL_PWR_ERR(_dev, fmt, args...) \
KGSL_LOG_ERR(_dev->dev, _dev->pwr_log, fmt, ##args)
#define KGSL_PWR_CRIT(_dev, fmt, args...) \
KGSL_LOG_CRIT(_dev->dev, _dev->pwr_log, fmt, ##args)

/* Core error messages - these are for core KGSL functions that have
   no device associated with them (such as memory) */

#define KGSL_CORE_ERR(fmt, args...) \
pr_err("kgsl: %s: " fmt, __func__, ##args)

void kgsl_device_log_init(struct kgsl_device *device);
int kgsl_yamato_debugfs_init(struct kgsl_device *device);
int kgsl_debug_init(struct dentry *);

#endif /* _GSL_LOG_H */
