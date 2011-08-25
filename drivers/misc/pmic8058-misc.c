/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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
/*
 * Qualcomm PMIC8058 Misc Device driver
 *
 */

#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mfd/pmic8058.h>

/* Resource offsets. */
enum PM8058_MISC_IRQ {
	PM8058_MISC_IRQ_OSC_HALT = 0
};

struct pm8058_misc_device {
	struct pm8058_chip	*pm_chip;
	struct dentry		*dgb_dir;
	unsigned int		osc_halt_irq;
	u64			osc_halt_count;
};

/* Handle the OSC_HALT interrupt: 32 kHz XTAL oscillator has stopped. */
static irqreturn_t pm8058_osc_halt_isr(int irq, void *data)
{
	struct pm8058_misc_device *miscdev = data;
	u64 count = 0;

	if (miscdev) {
		miscdev->osc_halt_count++;
		count = miscdev->osc_halt_count;
	}

	pr_crit("%s: OSC_HALT interrupt has triggered, 32 kHz XTAL oscillator"
		" has halted (%llu)!\n", __func__, count);

	return IRQ_HANDLED;
}

#if defined(CONFIG_DEBUG_FS)

static int osc_halt_count_get(void *data, u64 *val)
{
	struct pm8058_misc_device *miscdev = data;

	if (miscdev == NULL) {
		pr_err("%s: null pointer input.\n", __func__);
		return -EINVAL;
	}

	*val = miscdev->osc_halt_count;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(dbg_osc_halt_fops, osc_halt_count_get, NULL, "%llu\n");

static int __devinit pmic8058_misc_dbg_probe(struct pm8058_misc_device *miscdev)
{
	struct dentry *dent;
	struct dentry *temp;

	if (miscdev == NULL) {
		pr_err("%s: no parent data passed in.\n", __func__);
		return -EINVAL;
	}

	dent = debugfs_create_dir("pm8058-misc", NULL);
	if (dent == NULL || IS_ERR(dent)) {
		pr_err("%s: ERR debugfs_create_dir: dent=0x%X\n",
					__func__, (unsigned)dent);
		return -ENOMEM;
	}

	temp = debugfs_create_file("osc_halt_count", S_IRUSR, dent,
					miscdev, &dbg_osc_halt_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("%s: ERR debugfs_create_file: dent=0x%X\n",
					__func__, (unsigned)temp);
		goto debug_error;
	}

	miscdev->dgb_dir = dent;
	return 0;

debug_error:
	debugfs_remove_recursive(dent);
	return -ENOMEM;
}

static int __devexit pmic8058_misc_dbg_remove(
		struct pm8058_misc_device *miscdev)
{
	if (miscdev->dgb_dir)
		debugfs_remove_recursive(miscdev->dgb_dir);

	return 0;
}

#else

static int __devinit pmic8058_misc_dbg_probe(struct pm8058_misc_device *miscdev)
{
	return 0;
}

static int __devexit pmic8058_misc_dbg_remove(
		struct pm8058_misc_device *miscdev)
{
	return 0;
}

#endif


static int __devinit pmic8058_misc_probe(struct platform_device *pdev)
{
	struct pm8058_misc_device *miscdev;
	struct pm8058_chip *pm_chip;
	unsigned int irq;
	int rc;

	pm_chip = platform_get_drvdata(pdev);
	if (pm_chip == NULL) {
		pr_err("%s: no driver data passed in.\n", __func__);
		return -EFAULT;
	}

	irq = platform_get_irq(pdev, PM8058_MISC_IRQ_OSC_HALT);
	if (!irq) {
		pr_err("%s: no IRQ passed in.\n", __func__);
		return -EFAULT;
	}

	miscdev = kzalloc(sizeof *miscdev, GFP_KERNEL);
	if (miscdev == NULL) {
		pr_err("%s: kzalloc() failed.\n", __func__);
		return -ENOMEM;
	}

	miscdev->pm_chip = pm_chip;
	platform_set_drvdata(pdev, miscdev);

	rc = request_threaded_irq(irq, NULL, pm8058_osc_halt_isr,
			 IRQF_TRIGGER_RISING | IRQF_DISABLED,
			 "pm8058-osc_halt-irq", miscdev);
	if (rc < 0) {
		pr_err("%s: request_irq(%d) FAIL: %d\n", __func__, irq, rc);
		platform_set_drvdata(pdev, miscdev->pm_chip);
		kfree(miscdev);
		return rc;
	}
	miscdev->osc_halt_irq = irq;
	miscdev->osc_halt_count = 0;

	rc = pmic8058_misc_dbg_probe(miscdev);
	if (rc)
		return rc;

	pr_notice("%s: OK\n", __func__);
	return 0;
}

static int __devexit pmic8058_misc_remove(struct platform_device *pdev)
{
	struct pm8058_misc_device *miscdev = platform_get_drvdata(pdev);

	pmic8058_misc_dbg_remove(miscdev);

	platform_set_drvdata(pdev, miscdev->pm_chip);
	free_irq(miscdev->osc_halt_irq, miscdev);
	kfree(miscdev);

	return 0;
}

static struct platform_driver pmic8058_misc_driver = {
	.probe	= pmic8058_misc_probe,
	.remove	= __devexit_p(pmic8058_misc_remove),
	.driver	= {
		.name = "pm8058-misc",
		.owner = THIS_MODULE,
	},
};

static int __init pm8058_misc_init(void)
{
	return platform_driver_register(&pmic8058_misc_driver);
}

static void __exit pm8058_misc_exit(void)
{
	platform_driver_unregister(&pmic8058_misc_driver);
}

module_init(pm8058_misc_init);
module_exit(pm8058_misc_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PMIC8058 Misc Device driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:pmic8058-misc");
