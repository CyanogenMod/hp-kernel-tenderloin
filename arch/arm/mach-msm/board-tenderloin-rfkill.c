/* linux/arch/arm/mach-msm/board-tenderloin-rfkill.c
 * Copyright (C) 2007-2009 HTC Corporation.
 * Author: Thomas Tsai <thomas_tsai@htc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/

/* Control bluetooth power for sapphire platform */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/rfkill.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <asm/mach-types.h>
#include "gpiomux-tenderloin.h"

static struct rfkill *bt_rfk;
static const char bt_name[] = "csrbc6";

extern int *pin_table;

/* helper function to manipulate group of gpios (msm_gpiomux)*/
static int configure_gpiomux_gpios(int on, int gpios[], int cnt)
{
	int ret = 0;
	int i;

	for (i = 0; i < cnt; i++) {
		//printk(KERN_ERR "%s:pin(%d):%s\n", __func__, gpios[i], on?"on":"off");
		if (on) {
			ret = msm_gpiomux_get(gpios[i]);
			if (unlikely(ret))
				break;
		} else {
			ret = msm_gpiomux_put(gpios[i]);
			if (unlikely(ret))
				return ret;
		}
	}
	if (ret)
		for (; i >= 0; i--)
			msm_gpiomux_put(gpios[i]);
	return ret;
}

static int bluetooth_set_power(void *data, bool blocked)
{
	int gpios[] = {pin_table[BT_POWER_PIN], pin_table[BT_WAKE_PIN]};

	printk(KERN_INFO "Powering %s BT\n", blocked?"off":"on");

	configure_gpiomux_gpios(!blocked, gpios, ARRAY_SIZE(gpios));


	if (!blocked) {
		gpio_set_value(pin_table[BT_POWER_PIN], 1);
	} else {
		gpio_set_value(pin_table[BT_POWER_PIN], 0);
	}
	return 0;
}

static struct rfkill_ops tenderloin_rfkill_ops = {
	.set_block = bluetooth_set_power,
};

static int tenderloin_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	bool default_state = true;  /* off */

	bluetooth_set_power(NULL, default_state);

	bt_rfk = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH,
			      &tenderloin_rfkill_ops, NULL);
	if (!bt_rfk)
		return -ENOMEM;

	/* userspace cannot take exclusive control */

	rfkill_set_states(bt_rfk, default_state, false);

	rc = rfkill_register(bt_rfk);

	if (rc)
		rfkill_destroy(bt_rfk);
	return rc;
}

static int tenderloin_rfkill_remove(struct platform_device *dev)
{
	rfkill_unregister(bt_rfk);
	rfkill_destroy(bt_rfk);

	return 0;
}

static struct platform_driver tenderloin_rfkill_driver = {
	.probe = tenderloin_rfkill_probe,
	.remove = tenderloin_rfkill_remove,
	.driver = {
		.name = "tenderloin_rfkill",
		.owner = THIS_MODULE,
	},
};

static int __init tenderloin_rfkill_init(void)
{
	return platform_driver_register(&tenderloin_rfkill_driver);
}

static void __exit tenderloin_rfkill_exit(void)
{
	platform_driver_unregister(&tenderloin_rfkill_driver);
}

module_init(tenderloin_rfkill_init);
module_exit(tenderloin_rfkill_exit);
MODULE_DESCRIPTION("tenderloin rfkill");
MODULE_AUTHOR("Nick Pelly <npelly@google.com>");
MODULE_LICENSE("GPL");
