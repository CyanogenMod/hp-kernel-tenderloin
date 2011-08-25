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
#include <mach/clk.h>
#include <mach/dal_axi.h>
#include <mach/msm_bus.h>

#include "kgsl.h"
#include "kgsl_log.h"

static int kgsl_pwrctrl_fraction_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	unsigned long val;
	char temp[20];
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	snprintf(temp, sizeof(temp), "%.*s",
			 (int)min(count, sizeof(temp) - 1), buf);
	strict_strtoul(temp, 0, &val);

	mutex_lock(&device->mutex);

	if (val < 100)
		pwr->io_fraction = (unsigned int)val;
	else
		pwr->io_fraction = 100;

	mutex_unlock(&device->mutex);

	return count;
}

static int kgsl_pwrctrl_fraction_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	return sprintf(buf, "%d\n", pwr->io_fraction);
}

static struct device_attribute pwrio_fraction_attr = {
	.attr = { .name = "io_fraction", .mode = 0644, },
	.show = kgsl_pwrctrl_fraction_show,
	.store = kgsl_pwrctrl_fraction_store,
};

int kgsl_pwrctrl_init_sysfs(struct kgsl_device *device)
{
	int ret = 0;
	ret = device_create_file(device->dev, &pwrio_fraction_attr);
	return ret;
}

void kgsl_pwrctrl_uninit_sysfs(struct kgsl_device *device)
{
	device_remove_file(device->dev, &pwrio_fraction_attr);
}

int kgsl_pwrctrl_clk(struct kgsl_device *device, unsigned int pwrflag)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	switch (pwrflag) {
	case KGSL_PWRFLAGS_CLK_OFF:
		if (pwr->power_flags & KGSL_PWRFLAGS_CLK_ON) {
			if (pwr->grp_pclk)
				clk_disable(pwr->grp_pclk);
			clk_disable(pwr->grp_clk);
			if (pwr->imem_clk != NULL)
				clk_disable(pwr->imem_clk);
			if (pwr->imem_pclk != NULL)
				clk_disable(pwr->imem_pclk);
			if (pwr->clk_freq[KGSL_MIN_FREQ])
				clk_set_rate(pwr->grp_src_clk,
					pwr->clk_freq[KGSL_MIN_FREQ]);
			pwr->power_flags &=
					~(KGSL_PWRFLAGS_CLK_ON);
			pwr->power_flags |= KGSL_PWRFLAGS_CLK_OFF;
		}
		return KGSL_SUCCESS;
	case KGSL_PWRFLAGS_CLK_ON:
		if (pwr->power_flags & KGSL_PWRFLAGS_CLK_OFF) {
			if (pwr->clk_freq[KGSL_MAX_FREQ])
				clk_set_rate(pwr->grp_src_clk,
					pwr->clk_freq[KGSL_MAX_FREQ]);
			if (pwr->grp_pclk)
				clk_enable(pwr->grp_pclk);
			clk_enable(pwr->grp_clk);
			if (pwr->imem_clk != NULL)
				clk_enable(pwr->imem_clk);
			if (pwr->imem_pclk != NULL)
				clk_enable(pwr->imem_pclk);

			pwr->power_flags &=
				~(KGSL_PWRFLAGS_CLK_OFF);
			pwr->power_flags |= KGSL_PWRFLAGS_CLK_ON;
		}
		return KGSL_SUCCESS;
	default:
		return KGSL_FAILURE;
	}
}

int kgsl_pwrctrl_axi(struct kgsl_device *device, unsigned int pwrflag)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	switch (pwrflag) {
	case KGSL_PWRFLAGS_AXI_OFF:
		if (pwr->power_flags & KGSL_PWRFLAGS_AXI_ON) {
			if (pwr->clk_freq[KGSL_AXI_HIGH] && pwr->ebi1_clk)
				clk_disable(pwr->ebi1_clk);
			if (pwr->pcl)
				msm_bus_scale_client_update_request(pwr->pcl,
								BW_INIT);
			pwr->power_flags &=
				~(KGSL_PWRFLAGS_AXI_ON);
			pwr->power_flags |= KGSL_PWRFLAGS_AXI_OFF;
		}
		return KGSL_SUCCESS;
	case KGSL_PWRFLAGS_AXI_ON:
		if (pwr->power_flags & KGSL_PWRFLAGS_AXI_OFF) {
			if (pwr->clk_freq[KGSL_AXI_HIGH] && pwr->ebi1_clk)
				clk_enable(pwr->ebi1_clk);
			if (pwr->pcl)
				msm_bus_scale_client_update_request(pwr->pcl,
								BW_MAX);
			pwr->power_flags &=
				~(KGSL_PWRFLAGS_AXI_OFF);
			pwr->power_flags |= KGSL_PWRFLAGS_AXI_ON;
		}
		return KGSL_SUCCESS;
	default:
		return KGSL_FAILURE;
	}
}


int kgsl_pwrctrl_pwrrail(struct kgsl_device *device, unsigned int pwrflag)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	switch (pwrflag) {
	case KGSL_PWRFLAGS_POWER_OFF:
		if (pwr->power_flags & KGSL_PWRFLAGS_POWER_ON) {
			if (internal_pwr_rail_ctl(pwr->pwr_rail, KGSL_FALSE)) {
				KGSL_DRV_ERR(
					"call internal_pwr_rail_ctl failed\n");
				return KGSL_FAILURE;
			}
			if (pwr->gpu_reg)
				regulator_disable(pwr->gpu_reg);
			pwr->power_flags &=
					~(KGSL_PWRFLAGS_POWER_ON);
			pwr->power_flags |=
					KGSL_PWRFLAGS_POWER_OFF;
		}
		return KGSL_SUCCESS;
	case KGSL_PWRFLAGS_POWER_ON:
		if (pwr->power_flags & KGSL_PWRFLAGS_POWER_OFF) {
			if (internal_pwr_rail_ctl(pwr->pwr_rail, KGSL_TRUE)) {
				KGSL_DRV_ERR(
					"call internal_pwr_rail_ctl failed\n");
				return KGSL_FAILURE;
			}

			if (pwr->gpu_reg)
				regulator_enable(pwr->gpu_reg);
			pwr->power_flags &=
					~(KGSL_PWRFLAGS_POWER_OFF);
			pwr->power_flags |=
					KGSL_PWRFLAGS_POWER_ON;
		}
		return KGSL_SUCCESS;
	default:
		return KGSL_FAILURE;
	}
}


int kgsl_pwrctrl_irq(struct kgsl_device *device, unsigned int pwrflag)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	switch (pwrflag) {
	case KGSL_PWRFLAGS_IRQ_ON:
		if (pwr->power_flags & KGSL_PWRFLAGS_IRQ_OFF) {
			pwr->power_flags &=
				~(KGSL_PWRFLAGS_IRQ_OFF);
			pwr->power_flags |= KGSL_PWRFLAGS_IRQ_ON;
			enable_irq(pwr->interrupt_num);
		}
	return KGSL_SUCCESS;
	case KGSL_PWRFLAGS_IRQ_OFF:
		if (pwr->power_flags & KGSL_PWRFLAGS_IRQ_ON) {
			disable_irq(pwr->interrupt_num);
			pwr->power_flags &=
				~(KGSL_PWRFLAGS_IRQ_ON);
			pwr->power_flags |= KGSL_PWRFLAGS_IRQ_OFF;
		}
		return KGSL_SUCCESS;
	default:
		return KGSL_FAILURE;
	}
}

void kgsl_pwrctrl_close(struct kgsl_device *device)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
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

	if (pwr->grp_pclk) {
		clk_put(pwr->grp_pclk);
		pwr->grp_pclk = NULL;
	}

	if (pwr->grp_clk) {
		clk_put(pwr->grp_clk);
		pwr->grp_clk = NULL;
	}

	if (pwr->imem_clk != NULL) {
		clk_put(pwr->imem_clk);
		pwr->imem_clk = NULL;
	}

	pwr->grp_src_clk = NULL;
	pwr->power_flags = 0;
}

void kgsl_idle_check(struct work_struct *work)
{
	struct kgsl_device *device = container_of(work, struct kgsl_device,
							idle_check_ws);

	mutex_lock(&device->mutex);
	if (device->state & KGSL_STATE_HUNG) {
		device->requested_state = KGSL_STATE_NONE;
		goto done;
	}
	if (device->state & (KGSL_STATE_ACTIVE | KGSL_STATE_NAP)) {
		if (kgsl_pwrctrl_sleep(device) == KGSL_FAILURE)
			mod_timer(&device->idle_timer,
					jiffies +
					device->pwrctrl.interval_timeout);
	}
done:
	mutex_unlock(&device->mutex);
}

void kgsl_timer(unsigned long data)
{
	struct kgsl_device *device = (struct kgsl_device *) data;

	if (device->requested_state != KGSL_STATE_SUSPEND) {
		device->requested_state = KGSL_STATE_SLEEP;
		/* Have work run in a non-interrupt context. */
		queue_work(device->work_queue, &device->idle_check_ws);
	}
}

void kgsl_pre_hwaccess(struct kgsl_device *device)
{
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
 }


/******************************************************************/
/* Caller must hold the device mutex. */
int kgsl_pwrctrl_sleep(struct kgsl_device *device)
{
	KGSL_DRV_DBG("kgsl_pwrctrl_sleep device %d!!!\n", device->id);

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
	return KGSL_FAILURE;

sleep:
	kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_IRQ_OFF);
	kgsl_pwrctrl_axi(device, KGSL_PWRFLAGS_AXI_OFF);
	goto clk_off;

nap:
	kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_IRQ_OFF);
clk_off:
	kgsl_pwrctrl_clk(device, KGSL_PWRFLAGS_CLK_OFF);

	device->state = device->requested_state;
	device->requested_state = KGSL_STATE_NONE;
	wake_unlock(&device->idle_wakelock);

	return KGSL_SUCCESS;
}


/******************************************************************/
/* Caller must hold the device mutex. */
int kgsl_pwrctrl_wake(struct kgsl_device *device)
{
	int status = KGSL_SUCCESS;

	BUG_ON(!mutex_is_locked(&device->mutex));

	if (device->state == KGSL_STATE_SUSPEND)
		return status;

	/* Turn on the core clocks */
	status = kgsl_pwrctrl_clk(device, KGSL_PWRFLAGS_CLK_ON);
	if (device->state != KGSL_STATE_NAP) {
		kgsl_pwrctrl_axi(device, KGSL_PWRFLAGS_AXI_ON);
	}
	/* Enable state before turning on irq */
	device->state = KGSL_STATE_ACTIVE;
	kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_IRQ_ON);

	/* Re-enable HW access */
	mod_timer(&device->idle_timer, jiffies + FIRST_TIMEOUT);

	KGSL_DRV_VDBG("<-- kgsl_yamato_wake(). Return value %d\n", status);
	wake_lock(&device->idle_wakelock);

	return status;
}

