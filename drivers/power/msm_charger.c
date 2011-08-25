/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/mfd/pmic8058.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/msm-charger.h>
#include <linux/time.h>
#include <linux/slab.h>
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif

#include <asm/atomic.h>

#include <mach/msm_hsusb.h>

#define MSM_CHG_MAX_EVENTS		16
#define CHARGING_TEOC_MS		9000000
#define UPDATE_TIME_MS			60000
#define RESUME_CHECK_PERIOD_MS		60000

#define DEFAULT_BATT_MAX_V		4200
#define DEFAULT_BATT_MIN_V		3200
#define DEFAULT_BATT_RESUME_V		4100

#define MSM_CHARGER_GAUGE_MISSING_VOLTS 3500
#define MSM_CHARGER_GAUGE_MISSING_TEMP  35
/**
 * enum msm_battery_status
 * @BATT_STATUS_ABSENT: battery not present
 * @BATT_STATUS_ID_INVALID: battery present but the id is invalid
 * @BATT_STATUS_DISCHARGING: battery is present and is discharging
 * @BATT_STATUS_TRKL_CHARGING: battery is being trickle charged
 * @BATT_STATUS_FAST_CHARGING: battery is being fast charged
 * @BATT_STATUS_JUST_FINISHED_CHARGING: just finished charging,
 *		battery is fully charged. Do not begin charging untill the
 *		voltage falls below a threshold to avoid overcharging
 * @BATT_STATUS_TEMPERATURE_OUT_OF_RANGE: battery present,
					no charging, temp is hot/cold
 */
enum msm_battery_status {
	BATT_STATUS_ABSENT,
	BATT_STATUS_ID_INVALID,
	BATT_STATUS_DISCHARGING,
	BATT_STATUS_TRKL_CHARGING,
	BATT_STATUS_FAST_CHARGING,
	BATT_STATUS_JUST_FINISHED_CHARGING,
	BATT_STATUS_TEMPERATURE_OUT_OF_RANGE,
};

struct msm_hardware_charger_priv {
	struct list_head list;
	struct msm_hardware_charger *hw_chg;
	enum msm_hardware_charger_state hw_chg_state;
	unsigned int max_source_current;
	struct power_supply psy;
#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock wl;
#endif
};

struct msm_charger_event {
	enum msm_hardware_charger_event event;
	struct msm_hardware_charger *hw_chg;
};

struct msm_charger_mux {
	int inited;
	struct list_head msm_hardware_chargers;
	int count_chargers;
	struct mutex msm_hardware_chargers_lock;

	struct device *dev;

	unsigned int max_voltage;
	unsigned int min_voltage;
	unsigned int resume_voltage;

	unsigned int safety_time;
	struct delayed_work teoc_work;

	int stop_resume_check;
	int resume_count;
	struct delayed_work resume_work;

	unsigned int update_time;
	int stop_update;
	struct delayed_work update_heartbeat_work;

	struct mutex status_lock;
	enum msm_battery_status batt_status;
	struct msm_hardware_charger_priv *current_chg_priv;
	struct msm_hardware_charger_priv *current_mon_priv;

	unsigned int (*get_batt_capacity_percent) (void);

	struct msm_charger_event *queue;
	int tail;
	int head;
	spinlock_t queue_lock;
	int queue_count;
	struct work_struct queue_work;
	struct workqueue_struct *event_wq_thread;
};

static struct msm_charger_mux msm_chg;

static struct msm_battery_gauge *msm_batt_gauge;

static int is_chg_capable_of_charging(struct msm_hardware_charger_priv *priv)
{
	if (priv->hw_chg_state == CHG_READY_STATE
	    || priv->hw_chg_state == CHG_CHARGING_STATE)
		return 1;

	return 0;
}

static int is_batt_status_capable_of_charging(void)
{
	if (msm_chg.batt_status == BATT_STATUS_ABSENT
	    || msm_chg.batt_status == BATT_STATUS_TEMPERATURE_OUT_OF_RANGE
	    || msm_chg.batt_status == BATT_STATUS_ID_INVALID
	    || msm_chg.batt_status == BATT_STATUS_JUST_FINISHED_CHARGING)
		return 0;
	return 1;
}

static int is_batt_status_charging(void)
{
	if (msm_chg.batt_status == BATT_STATUS_TRKL_CHARGING
	    || msm_chg.batt_status == BATT_STATUS_FAST_CHARGING)
		return 1;
	return 0;
}

static int is_battery_present(void)
{
	if (msm_batt_gauge && msm_batt_gauge->is_battery_present)
		return msm_batt_gauge->is_battery_present();
	else {
		pr_err("msm-charger: no batt gauge batt=absent\n");
		return 0;
	}
}

static int is_battery_temp_within_range(void)
{
	if (msm_batt_gauge && msm_batt_gauge->is_battery_temp_within_range)
		return msm_batt_gauge->is_battery_temp_within_range();
	else {
		pr_err("msm-charger no batt gauge batt=out_of_temperatur\n");
		return 0;
	}
}

static int is_battery_id_valid(void)
{
	if (msm_batt_gauge && msm_batt_gauge->is_battery_id_valid)
		return msm_batt_gauge->is_battery_id_valid();
	else {
		pr_err("msm-charger no batt gauge batt=id_invalid\n");
		return 0;
	}
}

static int get_battery_mvolts(void)
{
	if (msm_batt_gauge && msm_batt_gauge->get_battery_mvolts)
		return msm_batt_gauge->get_battery_mvolts();
	else {
		pr_err("msm-charger no batt gauge assuming 3.5V\n");
		return MSM_CHARGER_GAUGE_MISSING_VOLTS;
	}
}

static int get_battery_temperature(void)
{
	if (msm_batt_gauge && msm_batt_gauge->get_battery_temperature)
		return msm_batt_gauge->get_battery_temperature();
	else {
		pr_err("msm-charger no batt gauge assuming 35 deg G\n");
		return MSM_CHARGER_GAUGE_MISSING_TEMP;
	}
}

 /* This function should only be called within handle_event or resume*/
static void update_batt_status(void)
{
	if (is_battery_present()) {
		if (is_battery_id_valid()) {
			if (msm_chg.batt_status == BATT_STATUS_ABSENT
				|| msm_chg.batt_status
					== BATT_STATUS_ID_INVALID) {
				msm_chg.stop_resume_check = 0;
				msm_chg.batt_status = BATT_STATUS_DISCHARGING;
			}
		} else
			msm_chg.batt_status = BATT_STATUS_ID_INVALID;
	 } else
		msm_chg.batt_status = BATT_STATUS_ABSENT;
}

static enum power_supply_property msm_power_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

static char *msm_power_supplied_to[] = {
	"battery",
};

static int msm_power_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct msm_hardware_charger_priv *priv;

	priv = container_of(psy, struct msm_hardware_charger_priv, psy);
	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = !(priv->hw_chg_state == CHG_ABSENT_STATE);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = (priv->hw_chg_state == CHG_READY_STATE)
			|| (priv->hw_chg_state == CHG_CHARGING_STATE);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property msm_batt_power_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
};

static int msm_batt_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (is_batt_status_charging())
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (msm_chg.batt_status ==
			 BATT_STATUS_JUST_FINISHED_CHARGING
			 && msm_chg.current_chg_priv != NULL)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		if (msm_chg.batt_status == BATT_STATUS_TRKL_CHARGING)
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		else if (msm_chg.batt_status == BATT_STATUS_FAST_CHARGING)
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		else
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (msm_chg.batt_status == BATT_STATUS_TEMPERATURE_OUT_OF_RANGE)
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = !(msm_chg.batt_status == BATT_STATUS_ABSENT);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_NiMH;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = msm_chg.max_voltage * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = msm_chg.min_voltage * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = get_battery_mvolts();
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = msm_chg.get_batt_capacity_percent();
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct power_supply msm_psy_batt = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = msm_batt_power_props,
	.num_properties = ARRAY_SIZE(msm_batt_power_props),
	.get_property = msm_batt_power_get_property,
};

static struct msm_hardware_charger_priv *usb_hw_chg_priv;
static void (*notify_vbus_state_func_ptr)(int);
static int usb_notified_of_insertion;

/* this is passed to the hsusb via platform_data msm_otg_pdata */
int msm_charger_register_vbus_sn(void (*callback)(int))
{
	pr_debug(KERN_INFO "%s\n", __func__);
	notify_vbus_state_func_ptr = callback;
	return 0;
}

/* this is passed to the hsusb via platform_data msm_otg_pdata */
void msm_charger_unregister_vbus_sn(void (*callback)(int))
{
	pr_debug(KERN_INFO "%s\n", __func__);
	notify_vbus_state_func_ptr = NULL;
}

static void notify_usb_of_the_plugin_event(struct msm_hardware_charger_priv
					   *hw_chg, int plugin)
{
	plugin = !!plugin;
	if (plugin == 1 && usb_notified_of_insertion == 0) {
		usb_notified_of_insertion = 1;
		if (notify_vbus_state_func_ptr) {
			dev_dbg(msm_chg.dev, "%s notifying plugin\n", __func__);
			(*notify_vbus_state_func_ptr) (plugin);
		} else
			dev_dbg(msm_chg.dev, "%s unable to notify plugin\n",
				__func__);
		usb_hw_chg_priv = hw_chg;
	}
	if (plugin == 0 && usb_notified_of_insertion == 1) {
		if (notify_vbus_state_func_ptr) {
			dev_dbg(msm_chg.dev, "%s notifying unplugin\n",
				__func__);
			(*notify_vbus_state_func_ptr) (plugin);
		} else
			dev_dbg(msm_chg.dev, "%s unable to notify unplugin\n",
				__func__);
		usb_notified_of_insertion = 0;
		usb_hw_chg_priv = NULL;
	}
}

static unsigned int msm_chg_get_batt_capacity_percent(void)
{
	unsigned int current_voltage = get_battery_mvolts();
	unsigned int low_voltage = msm_chg.min_voltage;
	unsigned int high_voltage = msm_chg.max_voltage;

	if (current_voltage <= low_voltage)
		return 0;
	else if (current_voltage >= high_voltage)
		return 100;
	else
		return (current_voltage - low_voltage) * 100
		    / (high_voltage - low_voltage);
}

#ifdef DEBUG
static inline void debug_print(const char *func,
			       struct msm_hardware_charger_priv *hw_chg_priv)
{
	dev_dbg(msm_chg.dev,
		"%s current=(%s)(s=%d)(r=%d) new=(%s)(s=%d)(r=%d) batt=%d En\n",
		func,
		msm_chg.current_chg_priv ? msm_chg.current_chg_priv->
		hw_chg->name : "none",
		msm_chg.current_chg_priv ? msm_chg.
		current_chg_priv->hw_chg_state : -1,
		msm_chg.current_chg_priv ? msm_chg.current_chg_priv->
		hw_chg->rating : -1,
		hw_chg_priv ? hw_chg_priv->hw_chg->name : "none",
		hw_chg_priv ? hw_chg_priv->hw_chg_state : -1,
		hw_chg_priv ? hw_chg_priv->hw_chg->rating : -1,
		msm_chg.batt_status);
}
#else
static inline void debug_print(const char *func,
			       struct msm_hardware_charger_priv *hw_chg_priv)
{
}
#endif

static struct msm_hardware_charger_priv *find_best_charger(void)
{
	struct msm_hardware_charger_priv *hw_chg_priv;
	struct msm_hardware_charger_priv *better;
	int rating;

	better = NULL;
	rating = 0;

	list_for_each_entry(hw_chg_priv, &msm_chg.msm_hardware_chargers, list) {
		if (is_chg_capable_of_charging(hw_chg_priv)) {
			if (hw_chg_priv->hw_chg->rating > rating) {
				rating = hw_chg_priv->hw_chg->rating;
				better = hw_chg_priv;
			}
		}
	}

	return better;
}

static int msm_charging_switched(struct msm_hardware_charger_priv *priv)
{
	int ret = 0;

	if (priv->hw_chg->charging_switched)
		ret = priv->hw_chg->charging_switched(priv->hw_chg);
	return ret;
}

static int msm_stop_charging(struct msm_hardware_charger_priv *priv)
{
	return priv->hw_chg->stop_charging(priv->hw_chg);
}

/* the best charger has been selected -start charging from current_chg_priv*/
static int msm_start_charging(void)
{
	int ret;
	struct msm_hardware_charger_priv *priv;

	priv = msm_chg.current_chg_priv;
	ret =
	    priv->hw_chg->start_charging(priv->hw_chg, msm_chg.max_voltage,
					 priv->max_source_current);
	if (ret) {
		dev_err(msm_chg.dev, "%s couldnt start chg error = %d",
			priv->hw_chg->name, ret);
	} else {
		priv->hw_chg_state = CHG_CHARGING_STATE;
	}

	return ret;
}

static void handle_charging_done(struct msm_hardware_charger_priv *priv)
{
	if (msm_chg.current_chg_priv == priv) {
		if (msm_chg.current_chg_priv->hw_chg_state ==
		    CHG_CHARGING_STATE)
			if (msm_stop_charging(msm_chg.current_chg_priv)) {
				dev_err(msm_chg.dev, "%s couldnt stop chg",
					msm_chg.current_chg_priv->hw_chg->name);
			}
		msm_chg.current_chg_priv->hw_chg_state = CHG_READY_STATE;

		msm_chg.batt_status = BATT_STATUS_JUST_FINISHED_CHARGING;
		dev_info(msm_chg.dev, "%s: stopping safety timer work",
				__func__);
		cancel_delayed_work(&msm_chg.teoc_work);
		dev_info(msm_chg.dev, "%s: starting resume timer work",
				__func__);
		queue_delayed_work(msm_chg.event_wq_thread,
					&msm_chg.resume_work,
				      round_jiffies_relative(msecs_to_jiffies
						     (RESUME_CHECK_PERIOD_MS)));
	}
}

static void teoc(struct work_struct *work)
{
	/* we have been charging too long - stop charging */
	dev_info(msm_chg.dev, "%s: safety timer work expired", __func__);

	mutex_lock(&msm_chg.status_lock);
	if (msm_chg.current_chg_priv != NULL
	    && msm_chg.current_chg_priv->hw_chg_state == CHG_CHARGING_STATE) {
		handle_charging_done(msm_chg.current_chg_priv);
	}
	mutex_unlock(&msm_chg.status_lock);
}

static void handle_battery_inserted(void)
{
	msm_chg.stop_resume_check = 0;
	/* if a charger is already present start charging */
	if (msm_chg.current_chg_priv != NULL) {
		if (msm_start_charging()) {
			dev_err(msm_chg.dev, "%s couldnt start chg",
				msm_chg.current_chg_priv->hw_chg->name);
			return;
		}
		msm_chg.batt_status = BATT_STATUS_TRKL_CHARGING;

		dev_info(msm_chg.dev, "%s: starting safety timer work",
				__func__);
		queue_delayed_work(msm_chg.event_wq_thread,
					&msm_chg.teoc_work,
				      round_jiffies_relative(msecs_to_jiffies
							     (msm_chg.
							      safety_time)));
	}
}

#define MSM_CHARGER_RESUME_COUNT 5
static void resume_charging(struct work_struct *work)
{
	dev_dbg(msm_chg.dev, "%s resuming charging %d", __func__,
						msm_chg.resume_count);

	if (msm_chg.stop_resume_check) {
		msm_chg.stop_resume_check = 0;
		pr_err("%s stopping resume", __func__);
		return;
	}

	update_batt_status();
	if (msm_chg.batt_status != BATT_STATUS_JUST_FINISHED_CHARGING) {
		pr_err("%s called outside JFC state", __func__);
		return;
	}

	if (get_battery_mvolts() < msm_chg.resume_voltage)
		msm_chg.resume_count++;
	else
		msm_chg.resume_count = 0;

	/*
	 * if we are within 500mV of min voltage range forget the count
	 * force start battery charging by increasing resume count
	 */
	if (get_battery_mvolts() < msm_chg.min_voltage + 500) {
		pr_err("%s: batt lost voltage rapidly -force resume charging\n",
					__func__);
		msm_chg.resume_count += MSM_CHARGER_RESUME_COUNT + 1;
	}

	if (msm_chg.resume_count > MSM_CHARGER_RESUME_COUNT) {
		/* the battery has dropped below 4.1V for 5 mins
		 * straight- resume charging */
		/* act as if the battery was just plugged in */
		mutex_lock(&msm_chg.status_lock);
		msm_chg.batt_status = BATT_STATUS_DISCHARGING;
		msm_chg.resume_count = 0;
		handle_battery_inserted();
		power_supply_changed(&msm_psy_batt);
		if (msm_chg.current_chg_priv != NULL)
			power_supply_changed(&msm_chg.current_chg_priv->psy);
		mutex_unlock(&msm_chg.status_lock);
	} else {
		/* reschedule resume check */
		dev_info(msm_chg.dev, "%s: rescheduling resume timer work",
				__func__);
		queue_delayed_work(msm_chg.event_wq_thread,
					&msm_chg.resume_work,
				      round_jiffies_relative(msecs_to_jiffies
						     (RESUME_CHECK_PERIOD_MS)));
	}
}

static void handle_battery_removed(void)
{
	/* if a charger is charging the battery stop it */
	if (msm_chg.current_chg_priv != NULL
	    && msm_chg.current_chg_priv->hw_chg_state == CHG_CHARGING_STATE) {
		if (msm_stop_charging(msm_chg.current_chg_priv)) {
			dev_err(msm_chg.dev, "%s couldnt stop chg",
				msm_chg.current_chg_priv->hw_chg->name);
		}
		msm_chg.current_chg_priv->hw_chg_state = CHG_READY_STATE;

		dev_info(msm_chg.dev, "%s: stopping safety timer work",
				__func__);
		cancel_delayed_work(&msm_chg.teoc_work);
	}
	msm_chg.stop_resume_check = 1;
	cancel_delayed_work(&msm_chg.resume_work);
}

static void update_heartbeat(struct work_struct *work)
{
	int temperature;

	if (msm_chg.batt_status == BATT_STATUS_ABSENT
		|| msm_chg.batt_status == BATT_STATUS_ID_INVALID) {
		if (is_battery_present())
			if (is_battery_id_valid()) {
				msm_chg.batt_status = BATT_STATUS_DISCHARGING;
				handle_battery_inserted();
			}
	} else {
		if (!is_battery_present()) {
			msm_chg.batt_status = BATT_STATUS_ABSENT;
			handle_battery_removed();
		}
		/*
		 * check battery id because a good battery could be removed
		 * and replaced with a invalid battery.
		 */
		if (!is_battery_id_valid()) {
			msm_chg.batt_status = BATT_STATUS_ID_INVALID;
			handle_battery_removed();
		}
	}
	pr_debug("msm-charger %s batt_status= %d\n",
				__func__, msm_chg.batt_status);

	if (msm_chg.current_chg_priv
		&& msm_chg.current_chg_priv->hw_chg_state
			== CHG_CHARGING_STATE) {
		temperature = get_battery_temperature();
		/* TODO implement JEITA SPEC*/
	}

	/* notify that the voltage has changed
	 * the read of the capacity will trigger a
	 * voltage read*/
	power_supply_changed(&msm_psy_batt);

	if (msm_chg.stop_update) {
		msm_chg.stop_update = 0;
		return;
	}
	queue_delayed_work(msm_chg.event_wq_thread,
				&msm_chg.update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (msm_chg.update_time)));
}

/* set the charger state to READY before calling this*/
static void handle_charger_ready(struct msm_hardware_charger_priv *hw_chg_priv)
{
	debug_print(__func__, hw_chg_priv);

	if (msm_chg.current_chg_priv != NULL
	    && hw_chg_priv->hw_chg->rating >
	    msm_chg.current_chg_priv->hw_chg->rating) {
		if (msm_chg.current_chg_priv->hw_chg_state ==
		    CHG_CHARGING_STATE) {
			if (msm_stop_charging(msm_chg.current_chg_priv)) {
				dev_err(msm_chg.dev, "%s couldnt stop chg",
					msm_chg.current_chg_priv->hw_chg->name);
				return;
			}
			if (msm_charging_switched(msm_chg.current_chg_priv)) {
				dev_err(msm_chg.dev, "%s couldnt stop chg",
					msm_chg.current_chg_priv->hw_chg->name);
				return;
			}
		}
		msm_chg.current_chg_priv->hw_chg_state = CHG_READY_STATE;
		msm_chg.current_chg_priv = NULL;
	}

	if (msm_chg.current_chg_priv == NULL) {
		msm_chg.current_chg_priv = hw_chg_priv;
		dev_info(msm_chg.dev,
			 "%s: best charger = %s", __func__,
			 msm_chg.current_chg_priv->hw_chg->name);

		if (!is_batt_status_capable_of_charging())
			return;

		/* start charging from the new charger */
		if (!msm_start_charging()) {
			/* if we simply switched chg continue with teoc timer
			 * else we update the batt state and set the teoc
			 * timer */
			if (!is_batt_status_charging()) {
				dev_info(msm_chg.dev,
					 "%s: starting safety timer", __func__);
				queue_delayed_work(msm_chg.event_wq_thread,
							&msm_chg.teoc_work,
						      round_jiffies_relative
						      (msecs_to_jiffies
						       (msm_chg.safety_time)));
				msm_chg.batt_status = BATT_STATUS_TRKL_CHARGING;
			}
		} else {
			/* we couldnt start charging from the new readied
			 * charger */
			if (is_batt_status_charging())
				msm_chg.batt_status = BATT_STATUS_DISCHARGING;
		}
	}
}

static void handle_charger_removed(struct msm_hardware_charger_priv
				   *hw_chg_removed, int new_state)
{
	struct msm_hardware_charger_priv *hw_chg_priv;

	debug_print(__func__, hw_chg_removed);

	if (msm_chg.current_chg_priv == hw_chg_removed) {
		if (msm_chg.current_chg_priv->hw_chg_state
						== CHG_CHARGING_STATE) {
			if (msm_stop_charging(hw_chg_removed)) {
				dev_err(msm_chg.dev, "%s couldnt stop chg",
					msm_chg.current_chg_priv->hw_chg->name);
			}
		}
		msm_chg.current_chg_priv = NULL;
	}

	hw_chg_removed->hw_chg_state = new_state;

	if (msm_chg.current_chg_priv == NULL) {
		hw_chg_priv = find_best_charger();
		if (hw_chg_priv == NULL) {
			dev_info(msm_chg.dev, "%s: no chargers ", __func__);
			/* if the battery was Just finished charging
			 * we keep that state as is so that we dont rush
			 * in to charging the battery when a charger is
			 * plugged in shortly. */
			if (is_batt_status_charging())
				msm_chg.batt_status = BATT_STATUS_DISCHARGING;
		} else {
			msm_chg.current_chg_priv = hw_chg_priv;
			dev_info(msm_chg.dev,
				 "%s: best charger = %s", __func__,
				 msm_chg.current_chg_priv->hw_chg->name);

			if (!is_batt_status_capable_of_charging())
				return;

			if (msm_start_charging()) {
				/* we couldnt start charging for some reason */
				msm_chg.batt_status = BATT_STATUS_DISCHARGING;
			}
		}
	}

	/* if we arent charging stop the safety timer */
	if (!is_batt_status_charging()) {
		dev_info(msm_chg.dev, "%s: stopping safety timer work",
				__func__);
		cancel_delayed_work(&msm_chg.teoc_work);
	}
}

static void handle_event(struct msm_hardware_charger *hw_chg, int event)
{
	struct msm_hardware_charger_priv *priv;

	if (hw_chg == NULL)
		return;

	priv = hw_chg->charger_private;

	dev_info(msm_chg.dev, "%s %d from %s\n", __func__, event, hw_chg->name);
	mutex_lock(&msm_chg.status_lock);

	switch (event) {
	case CHG_INSERTED_EVENT:
		if (priv->hw_chg_state != CHG_ABSENT_STATE) {
			dev_info(msm_chg.dev,
				 "%s insertion detected when cbl present",
				 hw_chg->name);
			break;
		}
		update_batt_status();
		if (hw_chg->type == CHG_TYPE_USB) {
			priv->hw_chg_state = CHG_PRESENT_STATE;
			notify_usb_of_the_plugin_event(priv, 1);
		} else {
			priv->hw_chg_state = CHG_READY_STATE;
			handle_charger_ready(priv);
		}
		wake_lock(&priv->wl);
		break;
	case CHG_ENUMERATED_EVENT:	/* only in USB types */
		if (priv->hw_chg_state == CHG_ABSENT_STATE) {
			dev_info(msm_chg.dev, "%s enum withuot presence",
				 hw_chg->name);
			break;
		}
		update_batt_status();
		dev_dbg(msm_chg.dev, "%s enum with %dmA to draw",
			 hw_chg->name, priv->max_source_current);
		if (priv->max_source_current == 0) {
			/* usb subsystem doesnt want us to draw
			 * charging current */
			/* act as if the charge is removed */
			if (priv->hw_chg_state != CHG_PRESENT_STATE)
				handle_charger_removed(priv, CHG_PRESENT_STATE);
		} else {
			if (priv->hw_chg_state != CHG_READY_STATE) {
				priv->hw_chg_state = CHG_READY_STATE;
				handle_charger_ready(priv);
			}
		}
		break;
	case CHG_REMOVED_EVENT:
		if (priv->hw_chg_state == CHG_ABSENT_STATE) {
			dev_info(msm_chg.dev, "%s cable already removed",
				 hw_chg->name);
			break;
		}
		update_batt_status();
		if (hw_chg->type == CHG_TYPE_USB)
			notify_usb_of_the_plugin_event(priv, 0);
		handle_charger_removed(priv, CHG_ABSENT_STATE);
		wake_unlock(&priv->wl);
		break;
	case CHG_DONE_EVENT:
		if (priv->hw_chg_state == CHG_CHARGING_STATE)
			handle_charging_done(priv);
		break;
	case CHG_BATT_BEGIN_FAST_CHARGING:
		/* only update if we are TRKL charging */
		if (msm_chg.batt_status == BATT_STATUS_TRKL_CHARGING)
			msm_chg.batt_status = BATT_STATUS_FAST_CHARGING;
		break;
	case CHG_BATT_TEMP_OUTOFRANGE:
		/* the batt_temp out of range can trigger
		 * when the battery is absent */
		if (!is_battery_present()
		    && msm_chg.batt_status != BATT_STATUS_ABSENT) {
			msm_chg.batt_status = BATT_STATUS_ABSENT;
			handle_battery_removed();
			break;
		}
		if (msm_chg.batt_status == BATT_STATUS_TEMPERATURE_OUT_OF_RANGE)
			break;
		msm_chg.batt_status = BATT_STATUS_TEMPERATURE_OUT_OF_RANGE;
		handle_battery_removed();
		break;
	case CHG_BATT_TEMP_INRANGE:
		if (msm_chg.batt_status != BATT_STATUS_TEMPERATURE_OUT_OF_RANGE)
			break;
		msm_chg.batt_status = BATT_STATUS_ID_INVALID;
		/* check id */
		if (!is_battery_id_valid())
			break;
		/* assume that we are discharging from the battery
		 * and act as if the battery was inserted
		 * if a charger is present charging will be resumed */
		msm_chg.batt_status = BATT_STATUS_DISCHARGING;
		handle_battery_inserted();
		break;
	case CHG_BATT_INSERTED:
		if (msm_chg.batt_status != BATT_STATUS_ABSENT)
			break;
		/* debounce */
		if (!is_battery_present())
			break;
		msm_chg.batt_status = BATT_STATUS_ID_INVALID;
		if (!is_battery_id_valid())
			break;
		/* assume that we are discharging from the battery */
		msm_chg.batt_status = BATT_STATUS_DISCHARGING;
		/* check if a charger is present */
		handle_battery_inserted();
		break;
	case CHG_BATT_REMOVED:
		if (msm_chg.batt_status == BATT_STATUS_ABSENT)
			break;
		/* debounce */
		if (is_battery_present())
			break;
		msm_chg.batt_status = BATT_STATUS_ABSENT;
		handle_battery_removed();
		break;
	}
	dev_dbg(msm_chg.dev, "%s %d done batt_status=%d\n", __func__,
		event, msm_chg.batt_status);

	/* update userspace */
	if (msm_batt_gauge)
		power_supply_changed(&msm_psy_batt);
	power_supply_changed(&priv->psy);

	mutex_unlock(&msm_chg.status_lock);
}

static int msm_chg_dequeue_event(struct msm_charger_event **event)
{
	unsigned long flags;

	spin_lock_irqsave(&msm_chg.queue_lock, flags);
	if (msm_chg.queue_count == 0) {
		spin_unlock_irqrestore(&msm_chg.queue_lock, flags);
		return -EINVAL;
	}
	*event = &msm_chg.queue[msm_chg.head];
	msm_chg.head = (msm_chg.head + 1) % MSM_CHG_MAX_EVENTS;
	pr_debug("%s dequeueing %d from %s\n", __func__,
				(*event)->event, (*event)->hw_chg->name);
	msm_chg.queue_count--;
	spin_unlock_irqrestore(&msm_chg.queue_lock, flags);
	return 0;
}

static int msm_chg_enqueue_event(struct msm_hardware_charger *hw_chg,
			enum msm_hardware_charger_event event)
{
	unsigned long flags;

	spin_lock_irqsave(&msm_chg.queue_lock, flags);
	if (msm_chg.queue_count == MSM_CHG_MAX_EVENTS) {
		spin_unlock_irqrestore(&msm_chg.queue_lock, flags);
		pr_err("%s: queue full cannot enqueue %d from %s\n",
				__func__, event, hw_chg->name);
		return -EAGAIN;
	}
	pr_debug("%s queueing %d from %s\n", __func__, event, hw_chg->name);
	msm_chg.queue[msm_chg.tail].event = event;
	msm_chg.queue[msm_chg.tail].hw_chg = hw_chg;
	msm_chg.tail = (msm_chg.tail + 1)%MSM_CHG_MAX_EVENTS;
	msm_chg.queue_count++;
	spin_unlock_irqrestore(&msm_chg.queue_lock, flags);
	return 0;
}

static void process_events(struct work_struct *work)
{
	struct msm_charger_event *event;
	int rc;

	do {
		rc = msm_chg_dequeue_event(&event);
		if (!rc)
			handle_event(event->hw_chg, event->event);
	} while (!rc);
}

/* USB calls these to tell us how much charging current we should draw*/
void msm_charger_vbus_draw(unsigned int mA)
{
	if (usb_hw_chg_priv) {
		usb_hw_chg_priv->max_source_current = mA;
		msm_charger_notify_event(usb_hw_chg_priv->hw_chg,
						CHG_ENUMERATED_EVENT);
	}
	else
		pr_err("%s called early;charger isnt initialized\n", __func__);
}

static int __init determine_initial_batt_status(void)
{
	int rc;

	if (is_battery_present())
		if (is_battery_id_valid())
			if (is_battery_temp_within_range())
				msm_chg.batt_status = BATT_STATUS_DISCHARGING;
			else
				msm_chg.batt_status
				    = BATT_STATUS_TEMPERATURE_OUT_OF_RANGE;
		else
			msm_chg.batt_status = BATT_STATUS_ID_INVALID;
	else
		msm_chg.batt_status = BATT_STATUS_ABSENT;

	if (is_batt_status_capable_of_charging())
		handle_battery_inserted();

	rc = power_supply_register(msm_chg.dev, &msm_psy_batt);
	if (rc < 0) {
		dev_err(msm_chg.dev, "%s: power_supply_register failed"
			" rc=%d\n", __func__, rc);
		return rc;
	}

	/* start updaing the battery powersupply every msm_chg.update_time
	 * milliseconds */
	queue_delayed_work(msm_chg.event_wq_thread,
				&msm_chg.update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (msm_chg.update_time)));

	pr_debug("%s:OK batt_status=%d\n", __func__, msm_chg.batt_status);
	return 0;
}

static int __devinit msm_charger_probe(struct platform_device *pdev)
{
	msm_chg.dev = &pdev->dev;
	if (pdev->dev.platform_data) {
		unsigned int milli_secs;

		struct msm_charger_platform_data *pdata
		    =
		    (struct msm_charger_platform_data *)pdev->dev.platform_data;

		milli_secs = pdata->safety_time * 60 * MSEC_PER_SEC;
		if (milli_secs > jiffies_to_msecs(MAX_JIFFY_OFFSET)) {
			dev_warn(&pdev->dev, "%s: safety time too large"
				 "%dms\n", __func__, milli_secs);
			milli_secs = jiffies_to_msecs(MAX_JIFFY_OFFSET);
		}
		msm_chg.safety_time = milli_secs;

		milli_secs = pdata->update_time * 60 * MSEC_PER_SEC;
		if (milli_secs > jiffies_to_msecs(MAX_JIFFY_OFFSET)) {
			dev_warn(&pdev->dev, "%s: safety time too large"
				 "%dms\n", __func__, milli_secs);
			milli_secs = jiffies_to_msecs(MAX_JIFFY_OFFSET);
		}
		msm_chg.update_time = milli_secs;

		msm_chg.max_voltage = pdata->max_voltage;
		msm_chg.min_voltage = pdata->min_voltage;
		msm_chg.resume_voltage = pdata->resume_voltage;
		msm_chg.get_batt_capacity_percent =
		    pdata->get_batt_capacity_percent;
	}
	if (msm_chg.safety_time == 0)
		msm_chg.safety_time = CHARGING_TEOC_MS;
	if (msm_chg.update_time == 0)
		msm_chg.update_time = UPDATE_TIME_MS;
	if (msm_chg.max_voltage == 0)
		msm_chg.max_voltage = DEFAULT_BATT_MAX_V;
	if (msm_chg.min_voltage == 0)
		msm_chg.min_voltage = DEFAULT_BATT_MIN_V;
	if (msm_chg.resume_voltage == 0)
		msm_chg.resume_voltage = DEFAULT_BATT_RESUME_V;
	if (msm_chg.get_batt_capacity_percent == NULL)
		msm_chg.get_batt_capacity_percent =
		    msm_chg_get_batt_capacity_percent;

	mutex_init(&msm_chg.status_lock);
	INIT_DELAYED_WORK(&msm_chg.teoc_work, teoc);
	INIT_DELAYED_WORK(&msm_chg.resume_work, resume_charging);
	INIT_DELAYED_WORK(&msm_chg.update_heartbeat_work, update_heartbeat);

	return 0;
}

static int __devexit msm_charger_remove(struct platform_device *pdev)
{
	mutex_destroy(&msm_chg.status_lock);
	power_supply_unregister(&msm_psy_batt);
	return 0;
}

int msm_charger_notify_event(struct msm_hardware_charger *hw_chg,
			     enum msm_hardware_charger_event event)
{
	msm_chg_enqueue_event(hw_chg, event);
	queue_work(msm_chg.event_wq_thread, &msm_chg.queue_work);
	return 0;
}
EXPORT_SYMBOL(msm_charger_notify_event);

int msm_charger_register(struct msm_hardware_charger *hw_chg)
{
	struct msm_hardware_charger_priv *priv;
	int rc = 0;

	if (!msm_chg.inited) {
		pr_err("%s: msm_chg is NULL,Too early to register\n", __func__);
		return -EAGAIN;
	}

	if (hw_chg->start_charging == NULL
		|| hw_chg->stop_charging == NULL
		|| hw_chg->name == NULL
		|| hw_chg->rating == 0) {
		pr_err("%s: invalid hw_chg\n", __func__);
		return -EINVAL;
	}

	priv = kzalloc(sizeof *priv, GFP_KERNEL);
	if (priv == NULL) {
		dev_err(msm_chg.dev, "%s kzalloc failed\n", __func__);
		return -ENOMEM;
	}

	priv->psy.name = hw_chg->name;
	if (hw_chg->type == CHG_TYPE_USB)
		priv->psy.type = POWER_SUPPLY_TYPE_USB;
	else
		priv->psy.type = POWER_SUPPLY_TYPE_MAINS;

	priv->psy.supplied_to = msm_power_supplied_to;
	priv->psy.num_supplicants = ARRAY_SIZE(msm_power_supplied_to);
	priv->psy.properties = msm_power_props;
	priv->psy.num_properties = ARRAY_SIZE(msm_power_props);
	priv->psy.get_property = msm_power_get_property;

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&priv->wl, WAKE_LOCK_SUSPEND, priv->psy.name);
#endif
	rc = power_supply_register(NULL, &priv->psy);
	if (rc) {
		dev_err(msm_chg.dev, "%s power_supply_register failed\n",
			__func__);
		goto out;
	}

	priv->hw_chg = hw_chg;
	priv->hw_chg_state = CHG_ABSENT_STATE;
	INIT_LIST_HEAD(&priv->list);
	mutex_lock(&msm_chg.msm_hardware_chargers_lock);
	list_add_tail(&priv->list, &msm_chg.msm_hardware_chargers);
	mutex_unlock(&msm_chg.msm_hardware_chargers_lock);
	hw_chg->charger_private = (void *)priv;
	return 0;

out:
	wake_lock_destroy(&priv->wl);
	kfree(priv);
	return rc;
}
EXPORT_SYMBOL(msm_charger_register);

void msm_battery_gauge_register(struct msm_battery_gauge *batt_gauge)
{
	if (msm_batt_gauge) {
		msm_batt_gauge = batt_gauge;
		pr_err("msm-charger %s multiple battery gauge called\n",
								__func__);
	} else {
		msm_batt_gauge = batt_gauge;
		determine_initial_batt_status();
	}
}
EXPORT_SYMBOL(msm_battery_gauge_register);

void msm_battery_gauge_unregister(struct msm_battery_gauge *batt_gauge)
{
	msm_batt_gauge = NULL;
}
EXPORT_SYMBOL(msm_battery_gauge_unregister);

int msm_charger_unregister(struct msm_hardware_charger *hw_chg)
{
	struct msm_hardware_charger_priv *priv;

	priv = (struct msm_hardware_charger_priv *)(hw_chg->charger_private);
	mutex_lock(&msm_chg.msm_hardware_chargers_lock);
	list_del(&priv->list);
	mutex_unlock(&msm_chg.msm_hardware_chargers_lock);
	wake_lock_destroy(&priv->wl);
	power_supply_unregister(&priv->psy);
	kfree(priv);
	return 0;
}
EXPORT_SYMBOL(msm_charger_unregister);

static int msm_charger_suspend(struct device *dev)
{
	dev_dbg(msm_chg.dev, "%s suspended\n", __func__);
	msm_chg.stop_update = 1;
	cancel_delayed_work(&msm_chg.update_heartbeat_work);
	/*
	 * we wont be charging in the suspend sequence, act as if the
	 * battery is removed - this will stop the resume delayed work
	 */
	handle_battery_removed();
	return 0;
}

static int msm_charger_resume(struct device *dev)
{
	dev_dbg(msm_chg.dev, "%s resumed\n", __func__);
	msm_chg.stop_update = 0;
	/* start updaing the battery powersupply every msm_chg.update_time
	 * milliseconds */
	queue_delayed_work(msm_chg.event_wq_thread,
				&msm_chg.update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (msm_chg.update_time)));
	handle_battery_inserted();
	return 0;
}

static SIMPLE_DEV_PM_OPS(msm_charger_pm_ops,
		msm_charger_suspend, msm_charger_resume);

static struct platform_driver msm_charger_driver = {
	.probe = msm_charger_probe,
	.remove = __devexit_p(msm_charger_remove),
	.driver = {
		   .name = "msm-charger",
		   .owner = THIS_MODULE,
		   .pm = &msm_charger_pm_ops,
	},
};

static int __init msm_charger_init(void)
{
	int rc;

	INIT_LIST_HEAD(&msm_chg.msm_hardware_chargers);
	msm_chg.count_chargers = 0;
	mutex_init(&msm_chg.msm_hardware_chargers_lock);

	msm_chg.queue = kzalloc(sizeof(struct msm_charger_event)
				* MSM_CHG_MAX_EVENTS,
				GFP_KERNEL);
	if (!msm_chg.queue) {
		rc = -ENOMEM;
		goto out;
	}
	msm_chg.tail = 0;
	msm_chg.head = 0;
	spin_lock_init(&msm_chg.queue_lock);
	msm_chg.queue_count = 0;
	INIT_WORK(&msm_chg.queue_work, process_events);
	msm_chg.event_wq_thread = create_workqueue("msm_charger_eventd");
	if (!msm_chg.event_wq_thread) {
		rc = -ENOMEM;
		goto free_queue;
	}
	rc = platform_driver_register(&msm_charger_driver);
	if (rc < 0) {
		pr_err("%s: FAIL: platform_driver_register. rc = %d\n",
		       __func__, rc);
		goto destroy_wq_thread;
	}
	msm_chg.inited = 1;
	return 0;

destroy_wq_thread:
	destroy_workqueue(msm_chg.event_wq_thread);
free_queue:
	kfree(msm_chg.queue);
out:
	return rc;
}

static void __exit msm_charger_exit(void)
{
	flush_workqueue(msm_chg.event_wq_thread);
	destroy_workqueue(msm_chg.event_wq_thread);
	kfree(msm_chg.queue);
	platform_driver_unregister(&msm_charger_driver);
}

module_init(msm_charger_init);
module_exit(msm_charger_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Abhijeet Dharmapurikar <adharmap@codeaurora.org>");
MODULE_DESCRIPTION("Battery driver for Qualcomm MSM chipsets.");
MODULE_VERSION("1.0");
