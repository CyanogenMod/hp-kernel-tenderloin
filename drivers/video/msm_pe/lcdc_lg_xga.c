/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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

#include <linux/delay.h>
#include <linux/pwm.h>
#ifdef CONFIG_PMIC8058_PWM
#include <linux/mfd/pmic8058.h>
#include <linux/pmic8058-pwm.h>
#endif
#include <mach/gpio.h>
#include "msm_fb.h"



#ifdef CONFIG_PMIC8058_PWM
static struct pwm_device *bl_pwm0;

/**
 * TPS61187's max PWM freq allowance is 22Khz.
 * If PWM_LEVEL is greater than 64 and If we use PMIC8058-PWM,
 * PMIC must be in 9bit modulation mode.
 */
#define PWM_FREQ_HZ 20000
#define PWM_PERIOD_USEC (USEC_PER_SEC / PWM_FREQ_HZ)
#define PWM_LEVEL 256
#define PWM_DUTY_LEVEL (PWM_PERIOD_USEC / PWM_LEVEL)
#endif

static struct msm_panel_common_pdata *lcdc_lg_pdata;


static int lcdc_lg_panel_on(struct platform_device *pdev)
{
	return 0;
}

static int lcdc_lg_panel_off(struct platform_device *pdev)
{
	return 0;
}


static void lcdc_lg_panel_set_backlight(struct msm_fb_data_type *mfd)
{
	unsigned int bl_level;
	int ret;

	bl_level = mfd->bl_level;

#ifdef CONFIG_PMIC8058_PWM
	if (bl_pwm0) 
    {
		ret = pwm_config2(bl_pwm0, bl_level, PWM_LEVEL, PWM_PERIOD_USEC);
		if (ret)
			printk(KERN_ERR "pwm_config on pwm 0 failed %d\n", ret);
        
        ret = pwm_enable(bl_pwm0);
		if (ret)
			printk(KERN_ERR "pwm_enable on pwm 0 failed %d\n", ret);
	}
#endif

}

static int __devinit lcdc_lg_probe(struct platform_device *pdev)
{
	if (pdev->id == 0) {
		lcdc_lg_pdata = pdev->dev.platform_data;
		return 0;
	}

#ifdef CONFIG_PMIC8058_PWM
    if(lcdc_lg_pdata == NULL)
        return 0;
    
	bl_pwm0 = pwm_request(lcdc_lg_pdata->gpio_num[0], "backlight");
	if (bl_pwm0 == NULL || IS_ERR(bl_pwm0)) 
    {
		pr_err("%s pwm_request() failed\n", __func__);
		bl_pwm0 = NULL;
	}

	printk(KERN_INFO "Lcdc_lg_probe: bl_pwm0=%p LPG_chan0=%d ",
			bl_pwm0, (int)lcdc_lg_pdata->gpio_num[0]
			);
#endif

	msm_fb_add_device(pdev);

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = lcdc_lg_probe,
	.driver = {
		.name   = "lcdc_lg_xga",
	},
};

static struct msm_fb_panel_data lg_panel_data = {    
 	.on = lcdc_lg_panel_on,
 	.off = lcdc_lg_panel_off,   
	.set_backlight = lcdc_lg_panel_set_backlight,
};

static struct platform_device this_device = {
	.name   = "lcdc_lg_xga",
	.id	= 1,
	.dev	= {
		.platform_data = &lg_panel_data,
	}
};

static int __init lcdc_lg_panel_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

#ifdef CONFIG_FB_MSM_MDDI_AUTO_DETECT
	if (msm_fb_detect_client("lcdc_lg_xga"))
		return 0;
#endif

	ret = platform_driver_register(&this_driver);
	if (ret)
		return ret;

	pinfo = &lg_panel_data.panel_info;
	pinfo->xres = 1024;
	pinfo->yres = 768;
	pinfo->type = LCDC_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 18;
	pinfo->fb_num = 2;
	pinfo->clk_rate = 96000000;
	pinfo->bl_max = PWM_LEVEL;
	pinfo->bl_min = 0;

	pinfo->lcdc.h_back_porch = 400;
	pinfo->lcdc.h_front_porch = 272;
	pinfo->lcdc.h_pulse_width = 328;
	pinfo->lcdc.v_back_porch = 6;
	pinfo->lcdc.v_front_porch = 10;
	pinfo->lcdc.v_pulse_width = 7;
	pinfo->lcdc.border_clr = 0;
	pinfo->lcdc.underflow_clr = 0xff;
	pinfo->lcdc.hsync_skew = 0;

	ret = platform_device_register(&this_device);
	if (ret)
		platform_driver_unregister(&this_driver);

	return ret;
}

module_init(lcdc_lg_panel_init);
