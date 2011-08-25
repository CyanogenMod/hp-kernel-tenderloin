/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/bug.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>

#include "mpm.h"
#include "rpm.h"
#include "rpm_resources.h"

/******************************************************************************
 * Debug Definitions
 *****************************************************************************/

enum {
	MSM_RPMRS_DEBUG_OUTPUT = BIT(0),
	MSM_RPMRS_DEBUG_BUFFER = BIT(1),
};

static int msm_rpmrs_debug_mask;
module_param_named(
	debug_mask, msm_rpmrs_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP
);

/******************************************************************************
 * Resource Definitions
 *****************************************************************************/

enum {
	MSM_RPMRS_PXO_OFF = 0,
	MSM_RPMRS_PXO_ON = 1,
};

enum {
	MSM_RPMRS_L2_CACHE_HSFS_OPEN = 0,
	MSM_RPMRS_L2_CACHE_ACTIVE = 3,
};

enum {
	MSM_RPMRS_VDD_MEM_RET_LOW = 500,
	MSM_RPMRS_VDD_MEM_RET_HIGH = 750,
	MSM_RPMRS_VDD_MEM_ACTIVE = 1000,
	MSM_RPMRS_VDD_MEM_MAX = 1250,
};

enum {
	MSM_RPMRS_VDD_DIG_RET_LOW = 500,
	MSM_RPMRS_VDD_DIG_RET_HIGH = 750,
	MSM_RPMRS_VDD_DIG_ACTIVE = 1000,
	MSM_RPMRS_VDD_DIG_MAX = 1250,
};

static bool msm_rpmrs_pxo_beyond_limits(struct msm_rpmrs_limits *limits);
static void msm_rpmrs_aggregate_pxo(struct msm_rpmrs_limits *limits,
		int from_idle);
static void msm_rpmrs_restore_pxo(void);
static bool msm_rpmrs_l2_cache_beyond_limits(struct msm_rpmrs_limits *limits);
static void msm_rpmrs_aggregate_l2_cache(struct msm_rpmrs_limits *limit,
		int from_idle);
static void msm_rpmrs_restore_l2_cache(void);
static bool msm_rpmrs_vdd_mem_beyond_limits(struct msm_rpmrs_limits *limits);
static void msm_rpmrs_aggregate_vdd_mem(struct msm_rpmrs_limits *limit,
		int from_idle);
static void msm_rpmrs_restore_vdd_mem(void);
static bool msm_rpmrs_vdd_dig_beyond_limits(struct msm_rpmrs_limits *limits);
static void msm_rpmrs_aggregate_vdd_dig(struct msm_rpmrs_limits *limits,
		int from_idle);
static void msm_rpmrs_restore_vdd_dig(void);
static void msm_rpmrs_aggregate_rpm_cpu(struct msm_rpmrs_limits *limits,
		int from_idle);
static void msm_rpmrs_restore_rpm_cpu(void);

struct msm_rpmrs_resource {
	struct msm_rpm_iv_pair rs;
	uint32_t enable_low_power;
	char *name;

	bool (*beyond_limits)(struct msm_rpmrs_limits *limits);
	void (*aggregate)(struct msm_rpmrs_limits *limits, int from_idle);
	void (*restore)(void);
};

static struct msm_rpmrs_resource msm_rpmrs_pxo = {
	.rs.id = MSM_RPM_ID_PXO_CLK,
	.name = "pxo",
	.beyond_limits = msm_rpmrs_pxo_beyond_limits,
	.aggregate = msm_rpmrs_aggregate_pxo,
	.restore = msm_rpmrs_restore_pxo,
};

static struct msm_rpmrs_resource msm_rpmrs_l2_cache = {
	.rs.id = MSM_RPM_ID_APPS_L2_CACHE_CTL,
	.name = "L2_cache",
	.beyond_limits = msm_rpmrs_l2_cache_beyond_limits,
	.aggregate = msm_rpmrs_aggregate_l2_cache,
	.restore = msm_rpmrs_restore_l2_cache,
};

static struct msm_rpmrs_resource msm_rpmrs_vdd_mem = {
	.rs.id = MSM_RPM_ID_SMPS0_0,
	.name = "vdd_mem",
	.beyond_limits = msm_rpmrs_vdd_mem_beyond_limits,
	.aggregate = msm_rpmrs_aggregate_vdd_mem,
	.restore = msm_rpmrs_restore_vdd_mem,
};

static struct msm_rpmrs_resource msm_rpmrs_vdd_dig = {
	.rs.id = MSM_RPM_ID_SMPS1_0,
	.name = "vdd_dig",
	.beyond_limits = msm_rpmrs_vdd_dig_beyond_limits,
	.aggregate = msm_rpmrs_aggregate_vdd_dig,
	.restore = msm_rpmrs_restore_vdd_dig,
};

static struct msm_rpmrs_resource msm_rpmrs_rpm_cpu = {
	.rs.id = MSM_RPM_ID_TRIGGER_SET_FROM,
	.name = "rpm_cpu",
	.beyond_limits = NULL,
	.aggregate = msm_rpmrs_aggregate_rpm_cpu,
	.restore = msm_rpmrs_restore_rpm_cpu,
};

static struct msm_rpmrs_resource *msm_rpmrs_resources[] = {
	&msm_rpmrs_pxo,
	&msm_rpmrs_l2_cache,
	&msm_rpmrs_vdd_mem,
	&msm_rpmrs_vdd_dig,
	&msm_rpmrs_rpm_cpu,
};

static uint32_t msm_rpmrs_buffer[MSM_RPM_ID_LAST + 1];
static DECLARE_BITMAP(msm_rpmrs_buffered, MSM_RPM_ID_LAST + 1);
static DECLARE_BITMAP(msm_rpmrs_listed, MSM_RPM_ID_LAST + 1);
static DEFINE_SPINLOCK(msm_rpmrs_lock);

#define MSM_RPMRS_VDD_MASK  0xfff
#define MSM_RPMRS_VDD(v)  ((v) & (MSM_RPMRS_VDD_MASK))

/******************************************************************************
 * Attribute Definitions
 *****************************************************************************/

struct msm_rpmrs_kboj_attribute {
	struct msm_rpmrs_resource *rs;
	struct kobj_attribute ka;
};

#define GET_RS_FROM_ATTR(attr) \
	(container_of(attr, struct msm_rpmrs_kboj_attribute, ka)->rs)

struct msm_rpmrs_resource_sysfs {
	struct attribute_group attr_group;
	struct attribute *attrs[2];
	struct msm_rpmrs_kboj_attribute kas;
};

/******************************************************************************
 * Power Level Definitions
 *****************************************************************************/

#define MSM_RPMRS_LIMITS(_pxo, _l2, _vdd_upper_b, _vdd) { \
	MSM_RPMRS_PXO_##_pxo, \
	MSM_RPMRS_L2_CACHE_##_l2, \
	MSM_RPMRS_VDD_MEM_##_vdd_upper_b, \
	MSM_RPMRS_VDD_MEM_##_vdd, \
	MSM_RPMRS_VDD_DIG_##_vdd_upper_b, \
	MSM_RPMRS_VDD_DIG_##_vdd, \
	{0}, {0}, \
}

struct msm_rpmrs_level {
	enum msm_pm_sleep_mode sleep_mode;
	struct msm_rpmrs_limits rs_limits;
	bool available;
	/* true when PXO is off or Vdd is below active level */
	bool use_mpm;

	uint32_t latency_us;
	uint32_t steady_state_power;
	uint32_t energy_overhead;
	uint32_t time_overhead_us;
};

static struct msm_rpmrs_level msm_rpmrs_levels[] = {
	{
		MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT,
		MSM_RPMRS_LIMITS(ON, ACTIVE, MAX, ACTIVE),
		true,
		false,
		1, 8000, 100000, 1,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE,
		MSM_RPMRS_LIMITS(ON, ACTIVE, MAX, ACTIVE),
		true,
		false,
		1500, 5000, 60100000, 3000,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(ON, ACTIVE, MAX, ACTIVE),
		false,
		false,
		1800, 5000, 60350000, 3500,
	},
	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(OFF, ACTIVE, MAX, ACTIVE),
		false,
		true,
		3800, 4500, 65350000, 5500,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(ON, HSFS_OPEN, MAX, ACTIVE),
		false,
		false,
		2800, 2500, 66850000, 4800,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(OFF, HSFS_OPEN, MAX, ACTIVE),
		false,
		true,
		4800, 2000, 71850000, 6800,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(OFF, HSFS_OPEN, ACTIVE, RET_HIGH),
		false,
		true,
		6800, 500, 75850000, 8800,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(OFF, HSFS_OPEN, RET_HIGH, RET_LOW),
		false,
		true,
		7800, 0, 76350000, 9800,
	},
};


/******************************************************************************
 * Resource Specific Functions
 *****************************************************************************/

static void msm_rpmrs_aggregate_sclk(uint32_t sclk_count, int from_idle)
{
	msm_rpmrs_buffer[MSM_RPM_ID_TRIGGER_TIMED_TO] = 0;
	set_bit(MSM_RPM_ID_TRIGGER_TIMED_TO, msm_rpmrs_buffered);
	msm_rpmrs_buffer[MSM_RPM_ID_TRIGGER_TIMED_SCLK_COUNT] = sclk_count;
	set_bit(MSM_RPM_ID_TRIGGER_TIMED_SCLK_COUNT, msm_rpmrs_buffered);
}

static void msm_rpmrs_restore_sclk(void)
{
	clear_bit(MSM_RPM_ID_TRIGGER_TIMED_SCLK_COUNT, msm_rpmrs_buffered);
	msm_rpmrs_buffer[MSM_RPM_ID_TRIGGER_TIMED_SCLK_COUNT] = 0;
	clear_bit(MSM_RPM_ID_TRIGGER_TIMED_TO, msm_rpmrs_buffered);
	msm_rpmrs_buffer[MSM_RPM_ID_TRIGGER_TIMED_TO] = 0;
}

static bool msm_rpmrs_pxo_beyond_limits(struct msm_rpmrs_limits *limits)
{
	struct msm_rpmrs_resource *rs = &msm_rpmrs_pxo;
	uint32_t pxo;

	if (rs->enable_low_power && test_bit(rs->rs.id, msm_rpmrs_buffered))
		pxo = msm_rpmrs_buffer[rs->rs.id];
	else
		pxo = MSM_RPMRS_PXO_ON;

	return pxo > limits->pxo;
}

static void msm_rpmrs_aggregate_pxo(struct msm_rpmrs_limits *limits,
		int from_idle)
{
	struct msm_rpmrs_resource *rs = &msm_rpmrs_pxo;
	uint32_t *buf = &msm_rpmrs_buffer[rs->rs.id];

	if (test_bit(rs->rs.id, msm_rpmrs_buffered)) {
		rs->rs.value = *buf;
		if (limits->pxo > *buf)
			*buf = limits->pxo;
		if (msm_mpm_gpio_irq_enabled(from_idle))
			*buf = MSM_RPMRS_PXO_ON;
		if (MSM_RPMRS_DEBUG_OUTPUT & msm_rpmrs_debug_mask)
			pr_info("%s: %d (0x%x)\n", __func__, *buf, *buf);
	}
}

static void msm_rpmrs_restore_pxo(void)
{
	struct msm_rpmrs_resource *rs = &msm_rpmrs_pxo;

	if (test_bit(rs->rs.id, msm_rpmrs_buffered))
		msm_rpmrs_buffer[rs->rs.id] = rs->rs.value;
}

static bool msm_rpmrs_l2_cache_beyond_limits(struct msm_rpmrs_limits *limits)
{
	struct msm_rpmrs_resource *rs = &msm_rpmrs_l2_cache;
	uint32_t l2_cache;

	if (rs->enable_low_power && test_bit(rs->rs.id, msm_rpmrs_buffered))
		l2_cache = msm_rpmrs_buffer[rs->rs.id];
	else
		l2_cache = MSM_RPMRS_L2_CACHE_ACTIVE;

	return l2_cache > limits->l2_cache;
}

static void msm_rpmrs_aggregate_l2_cache(struct msm_rpmrs_limits *limits,
		int from_idle)
{
	struct msm_rpmrs_resource *rs = &msm_rpmrs_l2_cache;
	uint32_t *buf = &msm_rpmrs_buffer[rs->rs.id];

	if (test_bit(rs->rs.id, msm_rpmrs_buffered)) {
		rs->rs.value = *buf;
		if (limits->l2_cache > *buf)
			*buf = limits->l2_cache;

		if (MSM_RPMRS_DEBUG_OUTPUT & msm_rpmrs_debug_mask)
			pr_info("%s: %d (0x%x)\n", __func__, *buf, *buf);
	}
}

static void msm_rpmrs_restore_l2_cache(void)
{
	struct msm_rpmrs_resource *rs = &msm_rpmrs_l2_cache;

	if (test_bit(rs->rs.id, msm_rpmrs_buffered))
		msm_rpmrs_buffer[rs->rs.id] = rs->rs.value;
}

static bool msm_rpmrs_vdd_mem_beyond_limits(struct msm_rpmrs_limits *limits)
{
	struct msm_rpmrs_resource *rs = &msm_rpmrs_vdd_mem;
	uint32_t vdd_mem;

	if (test_bit(rs->rs.id, msm_rpmrs_buffered)) {
		uint32_t buffered_value = msm_rpmrs_buffer[rs->rs.id];

		if (rs->enable_low_power == 0)
			vdd_mem = MSM_RPMRS_VDD_MEM_ACTIVE;
		else if (rs->enable_low_power == 1)
			vdd_mem = MSM_RPMRS_VDD_MEM_RET_HIGH;
		else
			vdd_mem = MSM_RPMRS_VDD_MEM_RET_LOW;

		if (MSM_RPMRS_VDD(buffered_value) > MSM_RPMRS_VDD(vdd_mem))
			vdd_mem = buffered_value;
	} else {
		vdd_mem = MSM_RPMRS_VDD_MEM_ACTIVE;
	}

	return MSM_RPMRS_VDD(vdd_mem) >=
				MSM_RPMRS_VDD(limits->vdd_mem_upper_bound);
}

static void msm_rpmrs_aggregate_vdd_mem(struct msm_rpmrs_limits *limits,
		int from_idle)
{
	struct msm_rpmrs_resource *rs = &msm_rpmrs_vdd_mem;
	uint32_t *buf = &msm_rpmrs_buffer[rs->rs.id];

	if (test_bit(rs->rs.id, msm_rpmrs_buffered)) {
		rs->rs.value = *buf;
		if (MSM_RPMRS_VDD(limits->vdd_mem) > MSM_RPMRS_VDD(*buf)) {
			*buf &= ~MSM_RPMRS_VDD_MASK;
			*buf |= MSM_RPMRS_VDD(limits->vdd_mem);
		}
		if ((MSM_RPMRS_VDD(*buf) < MSM_RPMRS_VDD_MEM_ACTIVE) &&
			msm_mpm_gic_irq_enabled(from_idle)) {
			*buf &= ~MSM_RPMRS_VDD_MASK;
			*buf |= MSM_RPMRS_VDD(MSM_RPMRS_VDD_MEM_ACTIVE);
		}

		if (MSM_RPMRS_DEBUG_OUTPUT & msm_rpmrs_debug_mask)
			pr_info("%s: vdd %d (0x%x)\n", __func__,
				MSM_RPMRS_VDD(*buf), MSM_RPMRS_VDD(*buf));
	}
}

static void msm_rpmrs_restore_vdd_mem(void)
{
	struct msm_rpmrs_resource *rs = &msm_rpmrs_vdd_mem;

	if (test_bit(rs->rs.id, msm_rpmrs_buffered))
		msm_rpmrs_buffer[rs->rs.id] = rs->rs.value;
}

static bool msm_rpmrs_vdd_dig_beyond_limits(struct msm_rpmrs_limits *limits)
{
	struct msm_rpmrs_resource *rs = &msm_rpmrs_vdd_dig;
	uint32_t vdd_dig;

	if (test_bit(rs->rs.id, msm_rpmrs_buffered)) {
		uint32_t buffered_value = msm_rpmrs_buffer[rs->rs.id];

		if (rs->enable_low_power == 0)
			vdd_dig = MSM_RPMRS_VDD_DIG_ACTIVE;
		else if (rs->enable_low_power == 1)
			vdd_dig = MSM_RPMRS_VDD_DIG_RET_HIGH;
		else
			vdd_dig = MSM_RPMRS_VDD_DIG_RET_LOW;

		if (MSM_RPMRS_VDD(buffered_value) > MSM_RPMRS_VDD(vdd_dig))
			vdd_dig = buffered_value;
	} else {
		vdd_dig = MSM_RPMRS_VDD_DIG_ACTIVE;
	}

	return MSM_RPMRS_VDD(vdd_dig) >=
				MSM_RPMRS_VDD(limits->vdd_dig_upper_bound);
}

static void msm_rpmrs_aggregate_vdd_dig(struct msm_rpmrs_limits *limits,
		int from_idle)
{
	struct msm_rpmrs_resource *rs = &msm_rpmrs_vdd_dig;
	uint32_t *buf = &msm_rpmrs_buffer[rs->rs.id];

	if (test_bit(rs->rs.id, msm_rpmrs_buffered)) {
		rs->rs.value = *buf;
		if (MSM_RPMRS_VDD(limits->vdd_dig) > MSM_RPMRS_VDD(*buf)) {
			*buf &= ~MSM_RPMRS_VDD_MASK;
			*buf |= MSM_RPMRS_VDD(limits->vdd_dig);
		}

		if ((MSM_RPMRS_VDD(*buf) < MSM_RPMRS_VDD_DIG_ACTIVE) &&
			msm_mpm_gic_irq_enabled(from_idle)) {
			*buf &= ~MSM_RPMRS_VDD_MASK;
			*buf |= MSM_RPMRS_VDD(MSM_RPMRS_VDD_DIG_ACTIVE);
		}

		if (MSM_RPMRS_DEBUG_OUTPUT & msm_rpmrs_debug_mask)
			pr_info("%s: vdd %d (0x%x)\n", __func__,
				MSM_RPMRS_VDD(*buf), MSM_RPMRS_VDD(*buf));
	}
}

static void msm_rpmrs_restore_vdd_dig(void)
{
	struct msm_rpmrs_resource *rs = &msm_rpmrs_vdd_dig;

	if (test_bit(rs->rs.id, msm_rpmrs_buffered))
		msm_rpmrs_buffer[rs->rs.id] = rs->rs.value;
}

static void msm_rpmrs_aggregate_rpm_cpu(struct msm_rpmrs_limits *limits,
		int from_idle)
{
	struct msm_rpmrs_resource *rs = &msm_rpmrs_rpm_cpu;

	if (test_bit(rs->rs.id, msm_rpmrs_buffered)) {
		rs->rs.value = msm_rpmrs_buffer[rs->rs.id];
		if (!msm_rpmrs_rpm_cpu.enable_low_power)
			msm_rpmrs_buffer[rs->rs.id] = 1;
	}
}

static void msm_rpmrs_restore_rpm_cpu(void)
{
	struct msm_rpmrs_resource *rs = &msm_rpmrs_rpm_cpu;

	if (test_bit(rs->rs.id, msm_rpmrs_buffered))
		msm_rpmrs_buffer[rs->rs.id] = rs->rs.value;
}

/******************************************************************************
 * Buffering Functions
 *****************************************************************************/

static void msm_rpmrs_update_levels(void)
{
	int i, k;

	for (i = 0; i < ARRAY_SIZE(msm_rpmrs_levels); i++) {
		struct msm_rpmrs_level *level = &msm_rpmrs_levels[i];

		if (level->sleep_mode != MSM_PM_SLEEP_MODE_POWER_COLLAPSE)
			continue;

		level->available = true;

		for (k = 0; k < ARRAY_SIZE(msm_rpmrs_resources); k++) {
			struct msm_rpmrs_resource *rs = msm_rpmrs_resources[k];

			if (rs->beyond_limits &&
					rs->beyond_limits(&level->rs_limits)) {
				level->available = false;
				break;
			}
		}
	}
}

/*
 * Return value:
 *   0: no entries in <req> is on our resource list
 *   1: one or more entries in <req> is on our resource list
 *   -EINVAL: invalid id in <req> array
 */
static int msm_rpmrs_buffer_request(struct msm_rpm_iv_pair *req, int count)
{
	bool listed;
	int i;

	for (i = 0; i < count; i++)
		if (req[i].id > MSM_RPM_ID_LAST)
			return -EINVAL;

	for (i = 0, listed = false; i < count; i++) {
		msm_rpmrs_buffer[req[i].id] = req[i].value;
		set_bit(req[i].id, msm_rpmrs_buffered);

		if (MSM_RPMRS_DEBUG_BUFFER & msm_rpmrs_debug_mask)
			pr_info("%s: reg %d: 0x%x\n",
				__func__, req[i].id, req[i].value);

		if (listed)
			continue;

		if (test_bit(req[i].id, msm_rpmrs_listed))
			listed = true;
	}

	return listed ? 1 : 0;
}

/*
 * Return value:
 *   0: no entries in <req> is on our resource list
 *   1: one or more entries in <req> is on our resource list
 *   -EINVAL: invalid id in <req> array
 */
static int msm_rpmrs_clear_buffer(struct msm_rpm_iv_pair *req, int count)
{
	bool listed;
	int i;

	for (i = 0; i < count; i++)
		if (req[i].id > MSM_RPM_ID_LAST)
			return -EINVAL;

	for (i = 0, listed = false; i < count; i++) {
		msm_rpmrs_buffer[req[i].id] = 0;
		clear_bit(req[i].id, msm_rpmrs_buffered);

		if (MSM_RPMRS_DEBUG_BUFFER & msm_rpmrs_debug_mask)
			pr_info("%s: reg %d\n", __func__, req[i].id);

		if (listed)
			continue;

		if (test_bit(req[i].id, msm_rpmrs_listed))
			listed = true;
	}

	return listed ? 1 : 0;
}

static int msm_rpmrs_flush_buffer(
	uint32_t sclk_count, struct msm_rpmrs_limits *limits, int from_idle)
{
	struct msm_rpm_iv_pair *req;
	int count;
	int rc;
	int i;

	msm_rpmrs_aggregate_sclk(sclk_count, from_idle);
	for (i = 0; i < ARRAY_SIZE(msm_rpmrs_resources); i++)
		msm_rpmrs_resources[i]->aggregate(limits, from_idle);

	count = bitmap_weight(msm_rpmrs_buffered, MSM_RPM_ID_LAST + 1);

	req = kmalloc(sizeof(*req) * count, GFP_ATOMIC);
	if (!req) {
		rc = -ENOMEM;
		goto flush_buffer_exit;
	}

	count = 0;
	i = find_first_bit(msm_rpmrs_buffered, MSM_RPM_ID_LAST + 1);

	while (i < MSM_RPM_ID_LAST + 1) {
		if (MSM_RPMRS_DEBUG_OUTPUT & msm_rpmrs_debug_mask)
			pr_info("%s: reg %d: 0x%x\n",
				__func__, i, msm_rpmrs_buffer[i]);

		req[count].id = i;
		req[count].value = msm_rpmrs_buffer[i];
		count++;

		i = find_next_bit(msm_rpmrs_buffered, MSM_RPM_ID_LAST+1, i+1);
	}

	rc = msm_rpm_set_noirq(MSM_RPM_CTX_SET_SLEEP, req, count);
	kfree(req);

	for (i = 0; i < ARRAY_SIZE(msm_rpmrs_resources); i++)
		msm_rpmrs_resources[i]->restore();
	msm_rpmrs_restore_sclk();

flush_buffer_exit:
	if (rc)
		pr_err("%s: failed: %d\n", __func__, rc);
	return rc;
}

static int msm_rpmrs_set_common(
	int ctx, struct msm_rpm_iv_pair *req, int count, bool noirq)
{
	if (ctx == MSM_RPM_CTX_SET_SLEEP) {
		unsigned long flags;
		int rc;

		spin_lock_irqsave(&msm_rpmrs_lock, flags);
		rc = msm_rpmrs_buffer_request(req, count);
		if (rc > 0) {
			msm_rpmrs_update_levels();
			rc = 0;
		}
		spin_unlock_irqrestore(&msm_rpmrs_lock, flags);

		return rc;
	}

	if (noirq)
		return msm_rpm_set_noirq(ctx, req, count);
	else
		return msm_rpm_set(ctx, req, count);
}

static int msm_rpmrs_clear_common(
	int ctx, struct msm_rpm_iv_pair *req, int count, bool noirq)
{
	if (ctx == MSM_RPM_CTX_SET_SLEEP) {
		unsigned long flags;
		int rc;

		spin_lock_irqsave(&msm_rpmrs_lock, flags);
		rc = msm_rpmrs_clear_buffer(req, count);
		if (rc > 0) {
			msm_rpmrs_update_levels();
			rc = 0;
		}
		spin_unlock_irqrestore(&msm_rpmrs_lock, flags);

		if (rc < 0)
			return rc;
	}

	if (noirq)
		return msm_rpm_clear_noirq(ctx, req, count);
	else
		return msm_rpm_clear(ctx, req, count);
}

/******************************************************************************
 * Attribute Functions
 *****************************************************************************/

static ssize_t msm_rpmrs_resource_attr_show(
	struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct kernel_param kp;
	unsigned long flags;
	unsigned int temp;
	int rc;

	spin_lock_irqsave(&msm_rpmrs_lock, flags);
	temp = GET_RS_FROM_ATTR(attr)->enable_low_power;
	spin_unlock_irqrestore(&msm_rpmrs_lock, flags);

	kp.arg = &temp;
	rc = param_get_uint(buf, &kp);

	if (rc > 0) {
		strcat(buf, "\n");
		rc++;
	}

	return rc;
}

static ssize_t msm_rpmrs_resource_attr_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct kernel_param kp;
	unsigned long flags;
	unsigned int temp;
	int rc;

	kp.arg = &temp;
	rc = param_set_uint(buf, &kp);
	if (rc)
		return rc;

	spin_lock_irqsave(&msm_rpmrs_lock, flags);
	GET_RS_FROM_ATTR(attr)->enable_low_power = temp;
	msm_rpmrs_update_levels();
	spin_unlock_irqrestore(&msm_rpmrs_lock, flags);

	return count;
}

static int __init msm_rpmrs_resource_sysfs_add(void)
{
	struct kobject *module_kobj;
	struct kobject *low_power_kboj;
	struct msm_rpmrs_resource_sysfs *rs;
	int i;
	int rc;

	module_kobj = kset_find_obj(module_kset, KBUILD_MODNAME);
	if (!module_kobj) {
		pr_err("%s: cannot find kobject for module %s\n",
			__func__, KBUILD_MODNAME);
		rc = -ENOENT;
		goto resource_sysfs_add_exit;
	}

	low_power_kboj = kobject_create_and_add(
				"enable_low_power", module_kobj);
	if (!low_power_kboj) {
		pr_err("%s: cannot create kobject\n", __func__);
		rc = -ENOMEM;
		goto resource_sysfs_add_exit;
	}

	for (i = 0; i < ARRAY_SIZE(msm_rpmrs_resources); i++) {
		rs = kzalloc(sizeof(*rs), GFP_KERNEL);
		if (!rs) {
			pr_err("%s: cannot allocate memory for attributes\n",
				__func__);
			rc = -ENOMEM;
			goto resource_sysfs_add_exit;
		}

		rs->kas.rs = msm_rpmrs_resources[i];
		rs->kas.ka.attr.name = msm_rpmrs_resources[i]->name;
		rs->kas.ka.attr.mode = 0644;
		rs->kas.ka.show = msm_rpmrs_resource_attr_show;
		rs->kas.ka.store = msm_rpmrs_resource_attr_store;

		rs->attrs[0] = &rs->kas.ka.attr;
		rs->attrs[1] = NULL;
		rs->attr_group.attrs = rs->attrs;

		rc = sysfs_create_group(low_power_kboj, &rs->attr_group);
		if (rc) {
			pr_err("%s: cannot create kobject attribute group\n",
				__func__);
			goto resource_sysfs_add_exit;
		}
	}

	rc = 0;

resource_sysfs_add_exit:
	return rc;
}

/******************************************************************************
 * Public Functions
 *****************************************************************************/

int msm_rpmrs_set(int ctx, struct msm_rpm_iv_pair *req, int count)
{
	return msm_rpmrs_set_common(ctx, req, count, false);
}

int msm_rpmrs_set_noirq(int ctx, struct msm_rpm_iv_pair *req, int count)
{
	WARN(!irqs_disabled(), "msm_rpmrs_set_noirq can only be called "
		"safely when local irqs are disabled.  Consider using "
		"msm_rpmrs_set or msm_rpmrs_set_nosleep instead.");
	return msm_rpmrs_set_common(ctx, req, count, true);
}

int msm_rpmrs_clear(int ctx, struct msm_rpm_iv_pair *req, int count)
{
	return msm_rpmrs_clear_common(ctx, req, count, false);
}

int msm_rpmrs_clear_noirq(int ctx, struct msm_rpm_iv_pair *req, int count)
{
	WARN(!irqs_disabled(), "msm_rpmrs_clear_noirq can only be called "
		"safely when local irqs are disabled.  Consider using "
		"msm_rpmrs_clear or msm_rpmrs_clear_nosleep instead.");
	return msm_rpmrs_clear_common(ctx, req, count, true);
}

void msm_rpmrs_show_resources(void)
{
	struct msm_rpmrs_resource *rs;
	unsigned long flags;
	int i;

	spin_lock_irqsave(&msm_rpmrs_lock, flags);
	for (i = 0; i < ARRAY_SIZE(msm_rpmrs_resources); i++) {
		rs = msm_rpmrs_resources[i];
		pr_info("%s: resource %s: buffered %d, value 0x%x\n",
			__func__, rs->name,
			test_bit(rs->rs.id, msm_rpmrs_buffered),
			msm_rpmrs_buffer[rs->rs.id]);
	}
	spin_unlock_irqrestore(&msm_rpmrs_lock, flags);
}

struct msm_rpmrs_limits *msm_rpmrs_lowest_limits(
	bool from_idle, enum msm_pm_sleep_mode sleep_mode, uint32_t latency_us,
	uint32_t sleep_us)
{
	unsigned int cpu = smp_processor_id();
	struct msm_rpmrs_level *best_level = NULL;
	bool irqs_detectable = false;
	int i;

	if (sleep_mode == MSM_PM_SLEEP_MODE_POWER_COLLAPSE)
		irqs_detectable = msm_mpm_irqs_detectable(from_idle);

	for (i = 0; i < ARRAY_SIZE(msm_rpmrs_levels); i++) {
		struct msm_rpmrs_level *level = &msm_rpmrs_levels[i];
		uint32_t power;

		if (!level->available)
			continue;

		if (sleep_mode != level->sleep_mode)
			continue;

		if (latency_us < level->latency_us)
			continue;

		if (sleep_us <= 1) {
			power = level->energy_overhead;
		} else if (sleep_us <= level->time_overhead_us) {
			power = level->energy_overhead / sleep_us;
		} else if ((sleep_us >> 10) > level->time_overhead_us) {
			power = level->steady_state_power;
		} else {
			power = (sleep_us - level->time_overhead_us);
			power *= level->steady_state_power;
			power /= sleep_us;
			power += level->energy_overhead / sleep_us;
		}

		if (!best_level ||
				best_level->rs_limits.power[cpu] >= power) {
			level->rs_limits.latency_us[cpu] = level->latency_us;
			level->rs_limits.power[cpu] = power;
			best_level = level;
		}
	}

	return best_level ? &best_level->rs_limits : NULL;
}

int msm_rpmrs_enter_sleep(
	bool from_idle, uint32_t sclk_count, struct msm_rpmrs_limits *limits)
{
	int rc;

	rc = msm_rpmrs_flush_buffer(sclk_count, limits, from_idle);
	if (rc)
		return rc;

	if (container_of(limits, struct msm_rpmrs_level, rs_limits)->use_mpm)
		msm_mpm_enter_sleep(from_idle);

	return 0;
}

void msm_rpmrs_exit_sleep(bool from_idle, struct msm_rpmrs_limits *limits)
{
	if (container_of(limits, struct msm_rpmrs_level, rs_limits)->use_mpm)
		msm_mpm_exit_sleep(from_idle);
}

static int __init msm_rpmrs_init(void)
{
	struct msm_rpm_iv_pair req;
	int rc;

	req.id = MSM_RPM_ID_APPS_L2_CACHE_CTL;
	req.value = 1;

	rc = msm_rpm_set(MSM_RPM_CTX_SET_0, &req, 1);
	if (rc) {
		pr_err("%s: failed to request L2 cache: %d\n", __func__, rc);
		goto init_exit;
	}

	req.id = MSM_RPM_ID_APPS_L2_CACHE_CTL;
	req.value = 0;

	rc = msm_rpmrs_set(MSM_RPM_CTX_SET_SLEEP, &req, 1);
	if (rc) {
		pr_err("%s: failed to initialize L2 cache for sleep: %d\n",
			__func__, rc);
		goto init_exit;
	}

	req.id = MSM_RPM_ID_TRIGGER_SET_FROM;
	req.value = 0;

	rc = msm_rpmrs_set(MSM_RPM_CTX_SET_SLEEP, &req, 1);
	if (rc) {
		pr_err("%s: failed to initialize RPM CPU for sleep: %d\n",
			__func__, rc);
		goto init_exit;
	}

	rc = msm_rpmrs_resource_sysfs_add();

init_exit:
	return rc;
}
device_initcall(msm_rpmrs_init);

static int __init msm_rpmrs_early_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(msm_rpmrs_resources); i++)
		set_bit(msm_rpmrs_resources[i]->rs.id, msm_rpmrs_listed);

	return 0;
}
early_initcall(msm_rpmrs_early_init);
