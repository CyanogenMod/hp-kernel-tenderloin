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

#include <linux/qcomwlan_pwrif.h>

#define GPIO_WLAN_DEEP_SLEEP_N  230
#define WLAN_RESET_OUT          1
#define WLAN_RESET              0

static const char *id = "WLAN";

int vos_chip_power_qrf8615(int on)
{
	static char wlan_on;
	static const char *vregs_qwlan_name[] = {
		"8058_l20",
		"8058_l8",
		"8901_s4",
		"8901_lvs1",
		"8901_l0",
		"8058_s2",
		"8058_s1",
	};
	static const int vregs_qwlan_val_min[] = {
		1800000,
		3050000,
		1225000,
		0,
		1200000,
		1300000,
		500000,
	};
	static const int vregs_qwlan_val_max[] = {
		1800000,
		3050000,
		1225000,
		0,
		1200000,
		1300000,
		1250000,
	};
	static const bool vregs_is_pin_controlled[] = {
		0,
		0,
		0,
		0,
		1,
		1,
		0,
	};
	static struct regulator *vregs_qwlan[ARRAY_SIZE(vregs_qwlan_name)];
	static struct msm_xo_voter *wlan_clock;
	int ret, i, rc = 0;

	/* WLAN RESET and CLK settings */
	if (on && !wlan_on) {
		/*
		 * Program U12 GPIO expander pin IO1 to de-assert (drive 0)
		 * WLAN_EXT_POR_N to put WLAN in reset
		 */
		rc = gpio_request(GPIO_WLAN_DEEP_SLEEP_N, "WLAN_DEEP_SLEEP_N");
		if (rc) {
			pr_err("WLAN reset GPIO %d request failed\n",
					GPIO_WLAN_DEEP_SLEEP_N);
			goto fail;
		}
		rc = gpio_direction_output(GPIO_WLAN_DEEP_SLEEP_N,
				WLAN_RESET_OUT);
		if (rc < 0) {
			pr_err("WLAN reset GPIO %d set output direction failed",
					GPIO_WLAN_DEEP_SLEEP_N);
			goto fail_gpio_dir_out;
		}

		/* Configure TCXO to be slave to WLAN_CLK_PWR_REQ */
		if (wlan_clock == NULL) {
			wlan_clock = msm_xo_get(MSM_XO_TCXO_A0, id);
			if (IS_ERR(wlan_clock)) {
				pr_err("Failed to get TCXO_A0 voter (%ld)\n",
						PTR_ERR(wlan_clock));
				goto fail_gpio_dir_out;
			}
		}

		rc = msm_xo_mode_vote(wlan_clock, MSM_XO_MODE_PIN_CTRL);
		if (rc < 0) {
			pr_err("Configuring TCXO to Pin controllable failed"
					"(%d)\n", rc);
			goto fail_xo_mode_vote;
		}
	} else if (!on && wlan_on) {
		if (wlan_clock != NULL)
			msm_xo_mode_vote(wlan_clock, MSM_XO_MODE_OFF);
		gpio_set_value_cansleep(GPIO_WLAN_DEEP_SLEEP_N, WLAN_RESET);
		gpio_free(GPIO_WLAN_DEEP_SLEEP_N);
	}

	/* WLAN VREG settings */
	for (i = 0; i < ARRAY_SIZE(vregs_qwlan_name); i++) {
		if (vregs_qwlan[i] == NULL) {
			vregs_qwlan[i] = regulator_get(NULL,
					vregs_qwlan_name[i]);
			if (IS_ERR(vregs_qwlan[i])) {
				pr_err("regulator get of %s failed (%ld)\n",
						vregs_qwlan_name[i],
						PTR_ERR(vregs_qwlan[i]));
				rc = PTR_ERR(vregs_qwlan[i]);
				goto vreg_get_fail;
			}
			if (vregs_qwlan_val_min[i] || vregs_qwlan_val_max[i]) {
				rc = regulator_set_voltage(vregs_qwlan[i],
						vregs_qwlan_val_min[i],
						vregs_qwlan_val_max[i]);
				if (rc) {
					pr_err("regulator_set_voltage(%s) failed\n",
							vregs_qwlan_name[i]);
					goto vreg_fail;
				}
			}
			/* vote for pin control (if needed) */
			if (vregs_is_pin_controlled[i]) {
				rc = regulator_set_mode(vregs_qwlan[i],
						REGULATOR_MODE_IDLE);
				if (rc) {
					pr_err("regulator_set_mode(%s) failed\n",
							vregs_qwlan_name[i]);
					goto vreg_fail;
				}
			}
		}
		if (on && !wlan_on) {
			rc = regulator_enable(vregs_qwlan[i]);
			if (rc < 0) {
				pr_err("vreg %s enable failed (%d)\n",
						vregs_qwlan_name[i], rc);
				goto vreg_fail;
			}
		} else if (!on && wlan_on) {
			rc = regulator_disable(vregs_qwlan[i]);
			if (rc < 0) {
				pr_err("vreg %s disable failed (%d)\n",
						vregs_qwlan_name[i], rc);
				goto vreg_fail;
			}
		}
	}
	if (on)
		wlan_on = true;
	else
		wlan_on = false;
	return 0;

vreg_fail:
	regulator_put(vregs_qwlan[i]);
vreg_get_fail:
	i--;
	while (i) {
		ret = !on ? regulator_enable(vregs_qwlan[i]) :
			regulator_disable(vregs_qwlan[i]);
		if (ret < 0) {
			pr_err("vreg %s %s failed (%d) in err path\n",
					vregs_qwlan_name[i],
					!on ? "enable" : "disable", ret);
		}
		regulator_put(vregs_qwlan[i]);
		i--;
	}
	if (!on)
		goto fail;
fail_xo_mode_vote:
	msm_xo_put(wlan_clock);
fail_gpio_dir_out:
	gpio_free(GPIO_WLAN_DEEP_SLEEP_N);
fail:
	return rc;
}
EXPORT_SYMBOL(vos_chip_power_qrf8615);
