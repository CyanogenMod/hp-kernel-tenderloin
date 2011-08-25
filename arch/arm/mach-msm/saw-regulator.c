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
 */

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/mfd/pmic8901.h>

#include "spm.h"
#include "saw-regulator.h"

#define SMPS_BAND_MASK			0xC0
#define SMPS_VPROG_MASK			0x3F

#define SMPS_BAND_1			0x40
#define SMPS_BAND_2			0x80
#define SMPS_BAND_3			0xC0

#define SMPS_BAND_1_VMIN		350000
#define SMPS_BAND_1_VMAX		650000
#define SMPS_BAND_1_VSTEP		6250

#define SMPS_BAND_2_VMIN		700000
#define SMPS_BAND_2_VMAX		1400000
#define SMPS_BAND_2_VSTEP		12500

#define SMPS_BAND_3_SETPOINT_VMIN	1500000
#define SMPS_BAND_3_VMIN		1400000
#define SMPS_BAND_3_VMAX		3300000
#define SMPS_BAND_3_VSTEP		50000

#define IS_PMIC_8901_V1(rev)		((rev) == PM_8901_REV_1p0 || \
					 (rev) == PM_8901_REV_1p1)

#define PMIC_8901_V1_SCALE(uV)		((((uV) - 62100) * 23) / 25)

/*
 * Band 1 of PMIC 8901 SMPS regulators only supports set points with the 3 LSB's
 * equal to 0.  This is accomplished in the macro by truncating the bits.
 */
#define PMIC_8901_SMPS_BAND_1_COMPENSATE(vprog)		((vprog) & 0xF8)

/*
 * This defines what the application processor believes the boot loader
 * has set the processor core voltages to (in uV).  This is needed because the
 * saw regulator must be initialized before SSBI is available, meaning that
 * the revision of the PMIC 8901 cannot be read when saw_set_voltage is first
 * called.
 */
#define BOOT_LOADER_CORE_VOLTAGE	1200000
#define MIN_CORE_VOLTAGE		950000

/* Specifies the PMIC internal slew rate in uV/us. */
#define REGULATOR_SLEW_RATE		1250

static int pmic8901_rev;

static int saw_set_voltage(struct regulator_dev *dev, int min_uV, int max_uV)
{
	/* Initialize min_uV to minimum possible set point value */
	static int prev_min_uV[NR_CPUS] = {
		[0 ... NR_CPUS - 1] = MIN_CORE_VOLTAGE
	};
	int rc, delay = 0, id;
	u8 vlevel, band;

	/* Calculate a time value to delay for so that voltage can stabalize. */
	id = rdev_get_id(dev);
	if (id >= 0 && id < num_possible_cpus()) {
		if (min_uV > prev_min_uV[id])
			delay = (min_uV - prev_min_uV[id]) /
				REGULATOR_SLEW_RATE;
		prev_min_uV[id] = min_uV;
	}

	/*
	 * Scale down setpoint for PMIC 8901 v1 as it outputs a voltage
	 * that is ~10% higher than the setpoint for SMPS regulators
	 */
	if (pmic8901_rev <= 0)
		pmic8901_rev = pm8901_rev(NULL);

	if (pmic8901_rev < 0) {
		/*
		 * If the PMIC 8901 revision value is unavailable, then trust
		 * the value set by the bootloader if it falls in the range:
		 * [min_uV, max_uV].
		 */
		if (BOOT_LOADER_CORE_VOLTAGE >= min_uV &&
		    BOOT_LOADER_CORE_VOLTAGE <= max_uV)
			return 0;
		else {
			pr_err("%s: PMIC 8901 revision unavailable and expected"
			       " bootloader core voltage: %d not in set point "
			       "range: %d to %d\n", __func__,
			       BOOT_LOADER_CORE_VOLTAGE, min_uV, max_uV);
			return -ENODEV;
		}

	} else if (IS_PMIC_8901_V1(pmic8901_rev))
		min_uV = PMIC_8901_V1_SCALE(min_uV);

	if (min_uV < SMPS_BAND_1_VMIN || min_uV > SMPS_BAND_3_VMAX)
		return -EINVAL;

	/* Round down for set points in the gaps between bands. */
	if (min_uV > SMPS_BAND_1_VMAX && min_uV < SMPS_BAND_2_VMIN)
		min_uV = SMPS_BAND_1_VMAX;
	else if (min_uV > SMPS_BAND_2_VMAX
			&& min_uV < SMPS_BAND_3_SETPOINT_VMIN)
		min_uV = SMPS_BAND_2_VMAX;

	if (min_uV < SMPS_BAND_2_VMIN) {
		vlevel = ((min_uV - SMPS_BAND_1_VMIN) / SMPS_BAND_1_VSTEP);
		vlevel = PMIC_8901_SMPS_BAND_1_COMPENSATE(vlevel);
		band = SMPS_BAND_1;
	} else if (min_uV < SMPS_BAND_3_SETPOINT_VMIN) {
		vlevel = ((min_uV - SMPS_BAND_2_VMIN) / SMPS_BAND_2_VSTEP);
		band = SMPS_BAND_2;
	} else {
		vlevel = ((min_uV - SMPS_BAND_3_VMIN) / SMPS_BAND_3_VSTEP);
		band = SMPS_BAND_3;
	}

	rc = msm_spm_set_vdd(rdev_get_id(dev), vlevel | band);
	if (rc)
		pr_err("%s: msm_spm_set_vdd failed %d\n", __func__, rc);
	/* Wait for voltage to stabalize */
	if (delay > 0)
		udelay(delay);

	return rc;
}

static struct regulator_ops saw_ops = {
	.set_voltage = saw_set_voltage,
};

static struct regulator_desc saw_descrip[] = {
	{
		.name	= "8901_s0",
		.id	= SAW_VREG_ID_S0,
		.ops	= &saw_ops,
		.type	= REGULATOR_VOLTAGE,
		.owner	= THIS_MODULE,
	},
	{
		.name	= "8901_s1",
		.id	= SAW_VREG_ID_S1,
		.ops	= &saw_ops,
		.type	= REGULATOR_VOLTAGE,
		.owner	= THIS_MODULE,
	},
};

static struct regulator_dev *saw_rdev[2];

static int __devinit saw_probe(struct platform_device *pdev)
{
	struct regulator_init_data *init_data;
	int rc = 0;

	if (pdev == NULL)
		return -EINVAL;

	if (pdev->id != SAW_VREG_ID_S0 && pdev->id != SAW_VREG_ID_S1)
		return -ENODEV;

	init_data = pdev->dev.platform_data;

	saw_rdev[pdev->id] = regulator_register(&saw_descrip[pdev->id],
			&pdev->dev, init_data, NULL);
	if (IS_ERR(saw_rdev[pdev->id]))
		rc = PTR_ERR(saw_rdev[pdev->id]);

	pr_info("%s: id=%d, rc=%d\n", __func__, pdev->id, rc);

	return rc;
}

static int __devexit saw_remove(struct platform_device *pdev)
{
	regulator_unregister(saw_rdev[pdev->id]);
	return 0;
}

static struct platform_driver saw_driver = {
	.probe = saw_probe,
	.remove = __devexit_p(saw_remove),
	.driver = {
		.name = "saw-regulator",
		.owner = THIS_MODULE,
	},
};

static int __init saw_init(void)
{
	return platform_driver_register(&saw_driver);
}

static void __exit saw_exit(void)
{
	platform_driver_unregister(&saw_driver);
}

postcore_initcall(saw_init);
module_exit(saw_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SAW regulator driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:saw-regulator");
