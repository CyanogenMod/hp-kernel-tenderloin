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
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <mach/clk.h>
#include <mach/dal_axi.h>
#include <mach/msm_bus.h>
#include <mach/msm_iomap.h>

#include "kgsl.h"
#include "kgsl_log.h"

#define SWITCH_OFF		200
#define SWITCH_OFF_RESET_TH 40
#define SKIP_COUNTER	500
#define TZ_UPDATE_ID		0x01404000
#define TZ_RESET_ID		0x01403000
#define UPDATE_BUSY_VAL	1000000
#define UPDATE_BUSY		50

#ifdef CONFIG_MSM_SECURE_IO
/* Trap into the TrustZone, and call funcs there. */
static int __secure_tz_entry(u32 cmd, u32 val)
{
	register u32 r0 asm("r0") = cmd;
	register u32 r1 asm("r1") = 0x0;
	register u32 r2 asm("r2") = val;

	__iowmb();
	asm(
		__asmeq("%0", "r0")
		__asmeq("%1", "r0")
		__asmeq("%2", "r1")
		__asmeq("%3", "r2")
		"smc    #0      @ switch to secure world\n"
		: "=r" (r0)
		: "r" (r0), "r" (r1), "r" (r2)
	);
	return r0;
}
#else
static int __secure_tz_entry(u32 cmd, u32 val)
{
	return 0;
}
#endif /* CONFIG_MSM_SECURE_IO */

/* Returns the requested update to our power level. *
 * Either up/down (-1/1) a level, or stay the same (0). */
static inline int kgsl_pwrctrl_tz_update(u32 idle)
{
	return __secure_tz_entry(TZ_UPDATE_ID, idle);
}

static inline void kgsl_pwrctrl_tz_reset(void)
{
	__secure_tz_entry(TZ_RESET_ID, 0);
}

void kgsl_pwrctrl_pwrlevel_change(struct kgsl_device *device,
					unsigned int new_level)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	if (new_level < (pwr->num_pwrlevels - 1) &&
		new_level >= pwr->thermal_pwrlevel &&
		new_level != pwr->active_pwrlevel) {
		pwr->active_pwrlevel = new_level;
		if ((pwr->power_flags & KGSL_PWRFLAGS_CLK_ON) ||
			(device->state == KGSL_STATE_NAP))
			clk_set_rate(pwr->grp_clks[0],
					pwr->pwrlevels[pwr->active_pwrlevel].
					gpu_freq);
		if (pwr->power_flags & KGSL_PWRFLAGS_AXI_ON) {
			if (pwr->pcl)
				msm_bus_scale_client_update_request(pwr->pcl,
					pwr->pwrlevels[pwr->active_pwrlevel].
					bus_freq);
			else if (pwr->ebi1_clk)
				clk_set_rate(pwr->ebi1_clk,
					pwr->pwrlevels[pwr->active_pwrlevel].
					bus_freq);
		}
		KGSL_PWR_WARN(device, "pwr level changed to %d\n",
					  pwr->active_pwrlevel);
	}
}

static int kgsl_pwrctrl_gpuclk_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	char temp[20];
	int i, delta = 5000000;
	unsigned long val;
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	snprintf(temp, sizeof(temp), "%.*s",
			 (int)min(count, sizeof(temp) - 1), buf);
	strict_strtoul(temp, 0, &val);

	mutex_lock(&device->mutex);
	/* Find the best match for the requested freq, if it exists */

	for (i = 0; i < pwr->num_pwrlevels; i++)
		if (abs(pwr->pwrlevels[i].gpu_freq - val) < delta) {
			pwr->thermal_pwrlevel = i;
			break;
		}

	kgsl_pwrctrl_pwrlevel_change(device, i);

	mutex_unlock(&device->mutex);

	return count;
}

static int kgsl_pwrctrl_gpuclk_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	return sprintf(buf, "%d\n",
			pwr->pwrlevels[pwr->active_pwrlevel].gpu_freq);
}

static int kgsl_pwrctrl_pwrnap_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	char temp[20];
	unsigned long val;
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	snprintf(temp, sizeof(temp), "%.*s",
			 (int)min(count, sizeof(temp) - 1), buf);
	strict_strtoul(temp, 0, &val);

	mutex_lock(&device->mutex);

	if (val == 1)
		pwr->nap_allowed = true;
	else if (val == 0)
		pwr->nap_allowed = false;

	mutex_unlock(&device->mutex);

	return count;
}

static int kgsl_pwrctrl_pwrnap_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	return sprintf(buf, "%d\n", pwr->nap_allowed);
}


static int kgsl_pwrctrl_idle_timer_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	char temp[20];
	unsigned long val;
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	const long div = 1000/HZ;
	static unsigned int org_interval_timeout = 1;

	snprintf(temp, sizeof(temp), "%.*s",
			 (int)min(count, sizeof(temp) - 1), buf);
	strict_strtoul(temp, 0, &val);

	if (org_interval_timeout == 1)
		org_interval_timeout = pwr->interval_timeout;

	mutex_lock(&device->mutex);

	/* Let the timeout be requested in ms, but convert to jiffies. */
	val /= div;
	if (val >= org_interval_timeout)
		pwr->interval_timeout = val;

	mutex_unlock(&device->mutex);

	return count;
}

static int kgsl_pwrctrl_idle_timer_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	return sprintf(buf, "%d\n", pwr->interval_timeout);
}

static int kgsl_pwrctrl_scaling_governor_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	char temp[20];
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	unsigned int reset = pwr->idle_pass;

	snprintf(temp, sizeof(temp), "%.*s",
			 (int)min(count, sizeof(temp) - 1), buf);
	if (strncmp(temp, "ondemand", 8) == 0)
		reset = 1;
	else if (strncmp(temp, "performance", 11) == 0)
		reset = 0;

	mutex_lock(&device->mutex);
	pwr->idle_pass = reset;
	if (pwr->idle_pass == 0)
		kgsl_pwrctrl_pwrlevel_change(device, pwr->thermal_pwrlevel);
	mutex_unlock(&device->mutex);

	return count;
}

static int kgsl_pwrctrl_scaling_governor_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	if (pwr->idle_pass)
		return snprintf(buf, 10, "ondemand\n");
	else
		return snprintf(buf, 13, "performance\n");
}

static int kgsl_pwrctrl_gpubusy_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int ret;
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_busy *b = &device->pwrctrl.busy;
	ret = snprintf(buf, 17, "%7d %7d\n",
				   b->on_time_old, b->time_old);
	if (device->pwrctrl.power_flags & KGSL_PWRFLAGS_AXI_OFF) {
		b->on_time_old = 0;
		b->time_old = 0;
	}
	return ret;
}

static struct device_attribute gpuclk_attr = {
	.attr = { .name = "gpuclk", .mode = 0644, },
	.show = kgsl_pwrctrl_gpuclk_show,
	.store = kgsl_pwrctrl_gpuclk_store,
};

static struct device_attribute pwrnap_attr = {
	.attr = { .name = "pwrnap", .mode = 0644, },
	.show = kgsl_pwrctrl_pwrnap_show,
	.store = kgsl_pwrctrl_pwrnap_store,
};

static struct device_attribute idle_timer_attr = {
	.attr = { .name = "idle_timer", .mode = 0644, },
	.show = kgsl_pwrctrl_idle_timer_show,
	.store = kgsl_pwrctrl_idle_timer_store,
};

static struct device_attribute scaling_governor_attr = {
	.attr = { .name = "scaling_governor", .mode = 0644, },
	.show = kgsl_pwrctrl_scaling_governor_show,
	.store = kgsl_pwrctrl_scaling_governor_store,
};

static struct device_attribute gpubusy_attr = {
	.attr = { .name = "gpubusy", .mode = 0444, },
	.show = kgsl_pwrctrl_gpubusy_show,
};

int kgsl_pwrctrl_init_sysfs(struct kgsl_device *device)
{
	int ret = 0;
	ret = device_create_file(device->dev, &pwrnap_attr);
	if (ret == 0)
		ret = device_create_file(device->dev, &gpuclk_attr);
	if (ret == 0)
		ret = device_create_file(device->dev, &idle_timer_attr);
	if (ret == 0)
		ret = device_create_file(device->dev, &scaling_governor_attr);
	if (ret == 0)
		ret = device_create_file(device->dev, &gpubusy_attr);
	return ret;
}

unsigned long kgsl_get_clkrate(struct clk *clk)
{
	if (clk != NULL)  {
		return clk_get_rate(clk);
	}  else   {
		return 0;
	}
}

void kgsl_pwrctrl_uninit_sysfs(struct kgsl_device *device)
{
	device_remove_file(device->dev, &gpuclk_attr);
	device_remove_file(device->dev, &pwrnap_attr);
	device_remove_file(device->dev, &idle_timer_attr);
	device_remove_file(device->dev, &scaling_governor_attr);
	device_remove_file(device->dev, &gpuclk_attr);
}

static void kgsl_pwrctrl_idle_calc(struct kgsl_device *device)
{
	int val;
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	struct kgsl_power_stats stats;

	device->ftbl.device_power_stats(device, &stats);

	if (stats.total_time == 0)
		return;

	/* If the GPU has stayed in turbo mode for a while, *
	 * stop writing out values. */
	if (pwr->active_pwrlevel == 0) {
		if (pwr->no_switch_cnt > SWITCH_OFF) {
			pwr->skip_cnt++;
			if (pwr->skip_cnt > SKIP_COUNTER) {
				pwr->no_switch_cnt -= SWITCH_OFF_RESET_TH;
				pwr->skip_cnt = 0;
			}
			return;
		}
		pwr->no_switch_cnt++;
	} else {
		pwr->no_switch_cnt = 0;
	}

	val = kgsl_pwrctrl_tz_update(stats.total_time - stats.busy_time);
	if (val)
		kgsl_pwrctrl_pwrlevel_change(device,
					pwr->active_pwrlevel + val);
}

/* Track the amount of time the gpu is on vs the total system time. *
 * Regularly update the percentage of busy time displayed by sysfs. */
static void kgsl_pwrctrl_busy_time(struct kgsl_device *device, bool on_time)
{
	struct kgsl_busy *b = &device->pwrctrl.busy;
	int elapsed;
	if (b->start.tv_sec == 0)
		do_gettimeofday(&(b->start));
	do_gettimeofday(&(b->stop));
	elapsed = (b->stop.tv_sec - b->start.tv_sec) * 1000000;
	elapsed += b->stop.tv_usec - b->start.tv_usec;
	b->time += elapsed;
	if (on_time)
		b->on_time += elapsed;
	/* Update the output regularly and reset the counters. */
	if ((b->time > UPDATE_BUSY_VAL) ||
		(device->pwrctrl.power_flags & KGSL_PWRFLAGS_AXI_OFF)) {
		b->on_time_old = b->on_time;
		b->time_old = b->time;
		b->on_time = 0;
		b->time = 0;
	}
	do_gettimeofday(&(b->start));
}

void kgsl_pwrctrl_clk(struct kgsl_device *device, unsigned int pwrflag)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	int i = 0;
	switch (pwrflag) {
	case KGSL_PWRFLAGS_CLK_OFF:
		if (pwr->power_flags & KGSL_PWRFLAGS_CLK_ON) {
			KGSL_PWR_INFO(device,
				"clocks off, device %d\n", device->id);
			for (i = KGSL_MAX_CLKS - 1; i > 0; i--)
				if (pwr->grp_clks[i])
					clk_disable(pwr->grp_clks[i]);
			if ((pwr->pwrlevels[0].gpu_freq > 0) &&
				(device->requested_state != KGSL_STATE_NAP))
				clk_set_rate(pwr->grp_clks[0],
					pwr->pwrlevels[pwr->num_pwrlevels - 1].
					gpu_freq);
			pwr->power_flags &=
					~(KGSL_PWRFLAGS_CLK_ON);
			pwr->power_flags |= KGSL_PWRFLAGS_CLK_OFF;
			kgsl_pwrctrl_busy_time(device, true);
		}
		return;
	case KGSL_PWRFLAGS_CLK_ON:
		if (pwr->power_flags & KGSL_PWRFLAGS_CLK_OFF) {
			KGSL_PWR_INFO(device,
				"clocks on, device %d\n", device->id);

			pwr->power_flags &=
				~(KGSL_PWRFLAGS_CLK_OFF);
			pwr->power_flags |= KGSL_PWRFLAGS_CLK_ON;
			kgsl_pwrctrl_busy_time(device, false);

			if ((pwr->pwrlevels[0].gpu_freq > 0) &&
				(device->state != KGSL_STATE_NAP))
				clk_set_rate(pwr->grp_clks[0],
					pwr->pwrlevels[pwr->active_pwrlevel].
						gpu_freq);

			/* as last step, enable grp_clk
			   this is to let GPU interrupt to come */
			for (i = KGSL_MAX_CLKS - 1; i > 0; i--)
				if (pwr->grp_clks[i])
					clk_enable(pwr->grp_clks[i]);
		}
		return;
	default:
		return;
	}
}

void kgsl_pwrctrl_axi(struct kgsl_device *device, unsigned int pwrflag)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	switch (pwrflag) {
	case KGSL_PWRFLAGS_AXI_OFF:
		if (pwr->power_flags & KGSL_PWRFLAGS_AXI_ON) {
			KGSL_PWR_INFO(device,
				"axi off, device %d\n", device->id);
			if (pwr->ebi1_clk)
				clk_disable(pwr->ebi1_clk);
			if (pwr->pcl)
				msm_bus_scale_client_update_request(pwr->pcl,
								    0);
			pwr->power_flags &=
				~(KGSL_PWRFLAGS_AXI_ON);
			pwr->power_flags |= KGSL_PWRFLAGS_AXI_OFF;
		}
		return;
	case KGSL_PWRFLAGS_AXI_ON:
		if (pwr->power_flags & KGSL_PWRFLAGS_AXI_OFF) {
			KGSL_PWR_INFO(device,
				"axi on, device %d\n", device->id);
			if (pwr->ebi1_clk)
				clk_enable(pwr->ebi1_clk);
			if (pwr->pcl)
				msm_bus_scale_client_update_request(pwr->pcl,
					pwr->pwrlevels[pwr->active_pwrlevel].
						bus_freq);
			pwr->power_flags &=
				~(KGSL_PWRFLAGS_AXI_OFF);
			pwr->power_flags |= KGSL_PWRFLAGS_AXI_ON;
		}
		return;
	default:
		return;
	}
}


void kgsl_pwrctrl_pwrrail(struct kgsl_device *device, unsigned int pwrflag)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	switch (pwrflag) {
	case KGSL_PWRFLAGS_POWER_OFF:
		if (pwr->power_flags & KGSL_PWRFLAGS_POWER_ON) {
			KGSL_PWR_INFO(device,
				"power off, device %d\n", device->id);
			if (internal_pwr_rail_ctl(pwr->pwr_rail, false)) {
				KGSL_DRV_ERR(device,
					"call internal_pwr_rail_ctl failed\n");
				return;
			}
			if (pwr->gpu_reg)
				regulator_disable(pwr->gpu_reg);
			pwr->power_flags &=
					~(KGSL_PWRFLAGS_POWER_ON);
			pwr->power_flags |=
					KGSL_PWRFLAGS_POWER_OFF;
		}
		return;
	case KGSL_PWRFLAGS_POWER_ON:
		if (pwr->power_flags & KGSL_PWRFLAGS_POWER_OFF) {
			KGSL_PWR_INFO(device,
				"power on, device %d\n", device->id);
			if (internal_pwr_rail_ctl(pwr->pwr_rail, true)) {
				KGSL_PWR_ERR(device,
					"call internal_pwr_rail_ctl failed\n");
				return;
			}

			if (pwr->gpu_reg)
				regulator_enable(pwr->gpu_reg);
			pwr->power_flags &=
					~(KGSL_PWRFLAGS_POWER_OFF);
			pwr->power_flags |=
					KGSL_PWRFLAGS_POWER_ON;
		}
		return;
	default:
		return;
	}
}


void kgsl_pwrctrl_irq(struct kgsl_device *device, unsigned int pwrflag)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	switch (pwrflag) {
	case KGSL_PWRFLAGS_IRQ_ON:
		if (pwr->power_flags & KGSL_PWRFLAGS_IRQ_OFF) {
			KGSL_PWR_INFO(device,
				"irq on, device %d\n", device->id);
			pwr->power_flags &=
				~(KGSL_PWRFLAGS_IRQ_OFF);
			pwr->power_flags |= KGSL_PWRFLAGS_IRQ_ON;
			enable_irq(pwr->interrupt_num);
		}
		return;
	case KGSL_PWRFLAGS_IRQ_OFF:
		if (pwr->power_flags & KGSL_PWRFLAGS_IRQ_ON) {
			KGSL_PWR_INFO(device,
				"irq off, device %d\n", device->id);
			disable_irq(pwr->interrupt_num);
			pwr->power_flags &=
				~(KGSL_PWRFLAGS_IRQ_ON);
			pwr->power_flags |= KGSL_PWRFLAGS_IRQ_OFF;
		}
		return;
	default:
		return;
	}
}

int kgsl_pwrctrl_init(struct kgsl_device *device)
{
	int i, result = 0;
	struct clk *clk;
	struct platform_device *pdev = device->pdev;
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	struct kgsl_device_platform_data *pdata_dev = pdev->dev.platform_data;
	struct kgsl_device_pwr_data *pdata_pwr = &pdata_dev->pwr_data;
	const char *clk_names[KGSL_MAX_CLKS] = {pwr->src_clk_name,
						pdata_dev->clk.name.clk,
						pdata_dev->clk.name.pclk,
						pdata_dev->imem_clk_name.clk,
						pdata_dev->imem_clk_name.pclk};

	/*acquire clocks */
	for (i = 1; i < KGSL_MAX_CLKS; i++) {
		if (clk_names[i]) {
			clk = clk_get(&pdev->dev, clk_names[i]);
			if (IS_ERR(clk))
				goto clk_err;
			pwr->grp_clks[i] = clk;
		}
	}
	/* Make sure we have a source clk for freq setting */
	clk = clk_get(&pdev->dev, clk_names[0]);
	pwr->grp_clks[0] = (IS_ERR(clk)) ? pwr->grp_clks[1] : clk;

	/* put the AXI bus into asynchronous mode with the graphics cores */
	if (pdata_pwr->set_grp_async != NULL)
		pdata_pwr->set_grp_async();

	if (pdata_pwr->num_levels > KGSL_MAX_PWRLEVELS) {
		KGSL_PWR_ERR(device, "invalid power level count: %d\n",
					 pdata_pwr->num_levels);
		result = -EINVAL;
		goto done;
	}
	pwr->num_pwrlevels = pdata_pwr->num_levels;
	pwr->active_pwrlevel = pdata_pwr->init_level;
	for (i = 0; i < pdata_pwr->num_levels; i++) {
		pwr->pwrlevels[i].gpu_freq =
		(pdata_pwr->pwrlevel[i].gpu_freq > 0) ?
		clk_round_rate(pwr->grp_clks[0],
					   pdata_pwr->pwrlevel[i].
					   gpu_freq) : 0;
		pwr->pwrlevels[i].bus_freq =
			pdata_pwr->pwrlevel[i].bus_freq;
		pwr->pwrlevels[i].io_fraction =
			pdata_pwr->pwrlevel[i].io_fraction;
	}
	/* Do not set_rate for targets in sync with AXI */
	if (pwr->pwrlevels[0].gpu_freq > 0)
		clk_set_rate(pwr->grp_clks[0], pwr->
				pwrlevels[pwr->num_pwrlevels - 1].gpu_freq);

	pwr->gpu_reg = regulator_get(NULL, pwr->regulator_name);
	if (IS_ERR(pwr->gpu_reg))
		pwr->gpu_reg = NULL;
	if (internal_pwr_rail_mode(device->pwrctrl.pwr_rail,
						PWR_RAIL_CTL_MANUAL)) {
		KGSL_PWR_ERR(device, "internal_pwr_rail_mode failed\n");
		result = -EINVAL;
		goto done;
	}

	pwr->power_flags = KGSL_PWRFLAGS_CLK_OFF |
			KGSL_PWRFLAGS_AXI_OFF | KGSL_PWRFLAGS_POWER_OFF |
			KGSL_PWRFLAGS_IRQ_OFF;
	pwr->nap_allowed = pdata_pwr->nap_allowed;
	pwr->pwrrail_first = pdata_pwr->pwrrail_first;
	pwr->idle_pass = pdata_pwr->idle_pass;
	pwr->interval_timeout = pdata_pwr->idle_timeout;
	pwr->ebi1_clk = clk_get(NULL, "ebi1_kgsl_clk");
	if (IS_ERR(pwr->ebi1_clk))
		pwr->ebi1_clk = NULL;
	else
		clk_set_rate(pwr->ebi1_clk,
					 pwr->pwrlevels[pwr->active_pwrlevel].
						bus_freq);
	if (pdata_dev->clk.bus_scale_table != NULL) {
		pwr->pcl =
			msm_bus_scale_register_client(pdata_dev->clk.
							bus_scale_table);
		if (!pwr->pcl) {
			KGSL_PWR_ERR(device,
					"msm_bus_scale_register_client failed: "
					"id %d table %p", device->id,
					pdata_dev->clk.bus_scale_table);
			result = -EINVAL;
			goto done;
		}
	}

	/*acquire interrupt */
	pwr->interrupt_num =
		platform_get_irq_byname(pdev, pwr->irq_name);

	if (pwr->interrupt_num <= 0) {
		KGSL_PWR_ERR(device, "platform_get_irq_byname failed: %d\n",
					 pwr->interrupt_num);
		result = -EINVAL;
		goto done;
	}

	register_early_suspend(&device->display_off);
	return result;

clk_err:
	result = PTR_ERR(clk);
	KGSL_PWR_ERR(device, "clk_get(%s) failed: %d\n",
				 clk_names[i], result);

done:
	return result;
}

void kgsl_pwrctrl_close(struct kgsl_device *device)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	int i;

	KGSL_PWR_INFO(device, "close device %d\n", device->id);

	unregister_early_suspend(&device->display_off);

	if (pwr->interrupt_num > 0) {
		if (pwr->have_irq) {
			free_irq(pwr->interrupt_num, NULL);
			pwr->have_irq = 0;
		}
		pwr->interrupt_num = 0;
	}

	clk_put(pwr->ebi1_clk);

	if (pwr->pcl)
		msm_bus_scale_unregister_client(pwr->pcl);

	pwr->pcl = 0;

	if (pwr->gpu_reg) {
		regulator_put(pwr->gpu_reg);
		pwr->gpu_reg = NULL;
	}

	for (i = 1; i < KGSL_MAX_CLKS; i++)
		if (pwr->grp_clks[i]) {
			clk_put(pwr->grp_clks[i]);
			pwr->grp_clks[i] = NULL;
		}

	pwr->grp_clks[0] = NULL;
	pwr->power_flags = 0;
}

void kgsl_idle_check(struct work_struct *work)
{
	struct kgsl_device *device = container_of(work, struct kgsl_device,
							idle_check_ws);

	mutex_lock(&device->mutex);
	if ((device->pwrctrl.idle_pass) &&
		(device->requested_state != KGSL_STATE_SLEEP))
		kgsl_pwrctrl_idle_calc(device);

	if (device->state & (KGSL_STATE_ACTIVE | KGSL_STATE_NAP)) {
		if (kgsl_pwrctrl_sleep(device) != 0) {
			mod_timer(&device->idle_timer,
					jiffies +
					device->pwrctrl.interval_timeout);
			/* If the GPU has been too busy to sleep, make sure *
			 * that is acurately reflected in the % busy numbers. */
			device->pwrctrl.busy.no_nap_cnt++;
			if (device->pwrctrl.busy.no_nap_cnt > UPDATE_BUSY) {
				kgsl_pwrctrl_busy_time(device, true);
				device->pwrctrl.busy.no_nap_cnt = 0;
			}
		}
	} else if (device->state & (KGSL_STATE_HUNG |
					KGSL_STATE_DUMP_AND_RECOVER)) {
		device->requested_state = KGSL_STATE_NONE;
	}

	mutex_unlock(&device->mutex);
}

void kgsl_timer(unsigned long data)
{
	struct kgsl_device *device = (struct kgsl_device *) data;

	KGSL_PWR_INFO(device, "idle timer expired device %d\n", device->id);
	if (device->requested_state != KGSL_STATE_SUSPEND) {
		device->requested_state = KGSL_STATE_SLEEP;
		/* Have work run in a non-interrupt context. */
		queue_work(device->work_queue, &device->idle_check_ws);
	}
}

void kgsl_pre_hwaccess(struct kgsl_device *device)
{
	BUG_ON(!mutex_is_locked(&device->mutex));
	if (device->state & (KGSL_STATE_SLEEP | KGSL_STATE_NAP))
		kgsl_pwrctrl_wake(device);
}

void kgsl_check_suspended(struct kgsl_device *device)
{
	if (device->requested_state == KGSL_STATE_SUSPEND ||
				device->state == KGSL_STATE_SUSPEND) {
		mutex_unlock(&device->mutex);
		wait_for_completion(&device->hwaccess_gate);
		mutex_lock(&device->mutex);
	}
	if (device->state == KGSL_STATE_DUMP_AND_RECOVER) {
		mutex_unlock(&device->mutex);
		wait_for_completion(&device->recovery_gate);
		mutex_lock(&device->mutex);
	}
 }


/******************************************************************/
/* Caller must hold the device mutex. */
int kgsl_pwrctrl_sleep(struct kgsl_device *device)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	KGSL_PWR_INFO(device, "sleep device %d\n", device->id);

	/* Work through the legal state transitions */
	if (device->requested_state == KGSL_STATE_NAP) {
		if (device->ftbl.device_isidle(device))
			goto nap;
	} else if (device->requested_state == KGSL_STATE_SLEEP) {
		if (device->state == KGSL_STATE_NAP ||
			device->ftbl.device_isidle(device))
			goto sleep;
	}

	device->requested_state = KGSL_STATE_NONE;
	return -EBUSY;

sleep:
	kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_IRQ_OFF);
	kgsl_pwrctrl_axi(device, KGSL_PWRFLAGS_AXI_OFF);
	if (pwr->pwrlevels[0].gpu_freq > 0)
		clk_set_rate(pwr->grp_clks[0],
				pwr->pwrlevels[pwr->num_pwrlevels - 1].
				gpu_freq);
	kgsl_pwrctrl_busy_time(device, false);
	pwr->busy.start.tv_sec = 0;
	device->pwrctrl.no_switch_cnt = 0;
	device->pwrctrl.time = 0;
	kgsl_pwrctrl_tz_reset();
	goto clk_off;

nap:
	kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_IRQ_OFF);
clk_off:
	kgsl_pwrctrl_clk(device, KGSL_PWRFLAGS_CLK_OFF);

	device->state = device->requested_state;
	device->requested_state = KGSL_STATE_NONE;
	wake_unlock(&device->idle_wakelock);
	KGSL_PWR_WARN(device, "state -> NAP/SLEEP(%d), device %d\n",
				  device->state, device->id);

	return 0;
}


/******************************************************************/
/* Caller must hold the device mutex. */
void kgsl_pwrctrl_wake(struct kgsl_device *device)
{
	if (device->state == KGSL_STATE_SUSPEND)
		return;

	if (device->state != KGSL_STATE_NAP) {
		if (device->pwrctrl.idle_pass)
			kgsl_pwrctrl_pwrlevel_change(device,
					device->pwrctrl.thermal_pwrlevel);
		kgsl_pwrctrl_axi(device, KGSL_PWRFLAGS_AXI_ON);
	}
	/* Turn on the core clocks */
	kgsl_pwrctrl_clk(device, KGSL_PWRFLAGS_CLK_ON);

	/* Enable state before turning on irq */
	device->state = KGSL_STATE_ACTIVE;
	KGSL_PWR_WARN(device, "state -> ACTIVE, device %d\n", device->id);
	kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_IRQ_ON);

	/* Re-enable HW access */
	mod_timer(&device->idle_timer,
				jiffies + device->pwrctrl.interval_timeout);

	wake_lock(&device->idle_wakelock);
	KGSL_PWR_INFO(device, "wake return for device %d\n", device->id);
}

