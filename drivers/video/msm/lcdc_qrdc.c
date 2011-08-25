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
 */

#include <linux/wpce775x.h>
#include "msm_fb.h"

struct lcdc_qrdc_data {
	struct msm_panel_common_pdata *pdata;
	int vga_enabled;
	struct platform_device *fbpdev;
};

static struct lcdc_qrdc_data *dd;

static ssize_t show_vga_enable(struct device *device,
			       struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", dd->vga_enabled);
}

static ssize_t store_vga_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long enable;
	struct msm_fb_data_type *mfd;
	int rc;

	rc = strict_strtoul(buf, 10, &enable);
	if (rc)
		return -EINVAL;

	if (dd->pdata && dd->pdata->vga_switch) {
		rc = dd->pdata->vga_switch(enable);
		if (enable)
			/* turn off backlight */
			wpce_smbus_write_word_data(0xC4, 0);
		else {
			/* restore backlight */
			mfd = platform_get_drvdata(dd->fbpdev);
			wpce_smbus_write_word_data(0xC4, mfd->bl_level);
		}
	} else
		rc = -ENODEV;
	if (!rc) {
		dd->vga_enabled = enable;
		rc = count;
	}
	return rc;
}

static DEVICE_ATTR(vga_enable, S_IRUGO|S_IWUSR, show_vga_enable,
		   store_vga_enable);
static struct attribute *attrs[] = {
	&dev_attr_vga_enable.attr,
	NULL,
};
static struct attribute_group attr_group = {
	.attrs = attrs,
};

static void lcdc_qrdc_set_backlight(struct msm_fb_data_type *mfd)
{
	if (!dd->vga_enabled)
		wpce_smbus_write_word_data(0xC4, mfd->bl_level);
}

static int __devinit qrdc_probe(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	int rc = 0;

	if (pdev->id == 0) {
		dd = kzalloc(sizeof *dd, GFP_KERNEL);
		if (!dd)
			return -ENOMEM;
		dd->vga_enabled = 0;
		dd->pdata = pdev->dev.platform_data;
		return 0;
	} else if (!dd)
		return -ENODEV;

	dd->fbpdev = msm_fb_add_device(pdev);
	if (!dd->fbpdev) {
		dev_err(&pdev->dev, "failed to add msm_fb device\n");
		rc = -ENODEV;
		goto probe_exit;
	}

	mfd = platform_get_drvdata(dd->fbpdev);
	if (mfd && mfd->fbi && mfd->fbi->dev) {
		rc = sysfs_create_group(&mfd->fbi->dev->kobj, &attr_group);
		if (rc) {
			dev_err(&pdev->dev, "failed to create sysfs group\n");
			goto probe_exit;
		}
	} else {
		dev_err(&pdev->dev, "no dev to create sysfs group\n");
		rc = -ENODEV;
		goto probe_exit;
	}

	return 0;

probe_exit:
	if (rc && dd) {
		kfree(dd);
		dd = NULL;
	}
	return rc;
}

static int __devexit qrdc_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&dd->fbpdev->dev.kobj, &attr_group);
	kfree(dd);
	dd = NULL;
	return 0;
}

static struct platform_driver this_driver = {
	.probe  = qrdc_probe,
	.remove = qrdc_remove,
	.driver = {
		.name   = "lcdc_qrdc",
	},
};

static struct msm_fb_panel_data qrdc_panel_data = {
	.set_backlight = lcdc_qrdc_set_backlight,
};

static struct platform_device this_device = {
	.name   = "lcdc_qrdc",
	.id	= 1,
	.dev	= {
		.platform_data = &qrdc_panel_data,
	}
};

static int __init lcdc_qrdc_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

	if (msm_fb_detect_client("lcdc_qrdc"))
		return 0;

	ret = platform_driver_register(&this_driver);
	if (ret) {
		printk(KERN_ERR "%s not able to register the driver\n",
			 __func__);
		return ret;
	}

	pinfo = &qrdc_panel_data.panel_info;
	pinfo->xres = 1366;
	pinfo->yres = 768;
	pinfo->type = LCDC_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 24;
	pinfo->fb_num = 2;
	pinfo->clk_rate = 43192000;
	pinfo->bl_max = 255;
	pinfo->bl_min = 1;

	pinfo->lcdc.h_back_porch = 120;
	pinfo->lcdc.h_front_porch = 20;
	pinfo->lcdc.h_pulse_width = 40;
	pinfo->lcdc.v_back_porch = 25;
	pinfo->lcdc.v_front_porch = 1;
	pinfo->lcdc.v_pulse_width = 7;
	pinfo->lcdc.border_clr = 0;      /* blk */
	pinfo->lcdc.underflow_clr = 0xff;        /* blue */
	pinfo->lcdc.hsync_skew = 0;

	ret = platform_device_register(&this_device);
	if (ret) {
		printk(KERN_ERR "%s not able to register the device\n",
			 __func__);
		platform_driver_unregister(&this_driver);
	}
	return ret;
}

module_init(lcdc_qrdc_init);
