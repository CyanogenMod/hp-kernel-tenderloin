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

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_novatek.h"


static struct msm_panel_common_pdata *mipi_novatek_pdata;

static struct dsi_buf novatek_tx_buf;
static struct dsi_buf novatek_rx_buf;


/* novatek blue panel */

#ifdef NOVETAK_COMMANDS_UNUSED
static char display_config_cmd_mode1[] = {
	/* TYPE_DCS_LWRITE */
	0x2A, 0x00, 0x00, 0x01,
	0x3F, 0xFF, 0xFF, 0xFF
};

static char display_config_cmd_mode2[] = {
	/* DTYPE_DCS_LWRITE */
	0x2B, 0x00, 0x00, 0x01,
	0xDF, 0xFF, 0xFF, 0xFF
};

static char display_config_cmd_mode3_666[] = {
	/* DTYPE_DCS_WRITE1 */
	0x3A, 0x66, 0x15, 0x80 /* 666 Packed (18-bits) */
};

static char display_config_cmd_mode3_565[] = {
	/* DTYPE_DCS_WRITE1 */
	0x3A, 0x55, 0x15, 0x80 /* 565 mode */
};

static char display_config_321[] = {
	/* DTYPE_DCS_WRITE1 */
	0x66, 0x2e, 0x15, 0x00 /* Reg 0x66 : 2E */
};

static char display_config_323[] = {
	/* DTYPE_DCS_WRITE */
	0x13, 0x00, 0x05, 0x00 /* Reg 0x13 < Set for Normal Mode> */
};

static char display_config_2lan[] = {
	/* DTYPE_DCS_WRITE */
	0x61, 0x01, 0x02, 0xff /* Reg 0x61 : 01,02 < Set for 2 Data Lane > */
};

static char display_config_exit_sleep[] = {
	/* DTYPE_DCS_WRITE */
	0x11, 0x00, 0x05, 0x80 /* Reg 0x11 < exit sleep mode> */
};

static char display_config_TE_ON[] = {
	/* DTYPE_DCS_WRITE1 */
	0x35, 0x00, 0x15, 0x80
};

static char display_config_39H[] = {
	/* DTYPE_DCS_WRITE */
	0x39, 0x00, 0x05, 0x80
};

static char display_config_set_tear_scanline[] = {
	/* DTYPE_DCS_LWRITE */
	0x44, 0x00, 0x00, 0xff
};

static char display_config_set_twolane[] = {
	/* DTYPE_DCS_WRITE1 */
	0xae, 0x03, 0x15, 0x80
};

static char display_config_set_threelane[] = {
	/* DTYPE_DCS_WRITE1 */
	0xae, 0x05, 0x15, 0x80
};

static char led_pwm2[] = {0x53, 0x24}; /* DTYPE_DCS_LWRITE */
static char led_pwm3[] = {0x55, 0x00}; /* DTYPE_DCS_LWRITE */

#else

static char sw_reset[2] = {0x01, 0x00}; /* DTYPE_DCS_WRITE */
static char enter_sleep[2] = {0x10, 0x00}; /* DTYPE_DCS_WRITE */
static char exit_sleep[2] = {0x11, 0x00}; /* DTYPE_DCS_WRITE */
static char display_off[2] = {0x28, 0x00}; /* DTYPE_DCS_WRITE */
static char display_on[2] = {0x29, 0x00}; /* DTYPE_DCS_WRITE */



static char rgb_888[2] = {0x3A, 0x77}; /* DTYPE_DCS_WRITE1 */

#if defined(NOVATEK_TWO_LANE)
static char set_num_of_lanes[2] = {0xae, 0x03}; /* DTYPE_DCS_WRITE1 */
#else  /* 1 lane */
static char set_num_of_lanes[2] = {0xae, 0x01}; /* DTYPE_DCS_WRITE1 */
#endif
/* commands by Novatke */
static char novatek_f4[2] = {0xf4, 0x55}; /* DTYPE_DCS_WRITE1 */
static char novatek_8c[16] = { /* DTYPE_DCS_LWRITE */
	0x8C, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x08, 0x08, 0x00, 0x30, 0xC0, 0xB7, 0x37};
static char novatek_ff[2] = {0xff, 0x55 }; /* DTYPE_DCS_WRITE1 */

static char set_width[5] = { /* DTYPE_DCS_LWRITE */
	0x2A, 0x00, 0x00, 0x02, 0x1B}; /* 540 - 1 */
static char set_height[5] = { /* DTYPE_DCS_LWRITE */
	0x2B, 0x00, 0x00, 0x03, 0xBF}; /* 960 - 1 */
#endif

#ifdef NOVATEK_BACKLIGHT
static char led_pwm1[2] = {0x51, 0x0};	/* DTYPE_DCS_LWRITE */

static struct dsi_cmd_desc novatek_cmd_backlight_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(led_pwm1), led_pwm1},
};
#endif

static struct dsi_cmd_desc novatek_video_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 50,
		sizeof(sw_reset), sw_reset},
	{DTYPE_DCS_WRITE, 1, 0, 0, 10,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_WRITE, 1, 0, 0, 10,
		sizeof(display_on), display_on},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(set_num_of_lanes), set_num_of_lanes},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(rgb_888), rgb_888}
};

static struct dsi_cmd_desc novatek_cmd_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 50,
		sizeof(sw_reset), sw_reset},
	{DTYPE_DCS_WRITE, 1, 0, 0, 10,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_WRITE, 1, 0, 0, 10,
		sizeof(display_on), display_on},
	{DTYPE_MAX_PKTSIZE, 1, 0, 0, 0,
		sizeof(max_pktsize), max_pktsize},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 50,
		sizeof(novatek_f4), novatek_f4},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 50,
		sizeof(novatek_8c), novatek_8c},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 50,
		sizeof(novatek_ff), novatek_ff},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(set_num_of_lanes), set_num_of_lanes},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 50,
		sizeof(set_width), set_width},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 50,
		sizeof(set_height), set_height},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(rgb_888), rgb_888}
};

static struct dsi_cmd_desc novatek_display_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 10,
		sizeof(display_off), display_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(enter_sleep), enter_sleep}
};

static char manufacture_id[2] = {0x04, 0x00}; /* DTYPE_DCS_READ */

static struct dsi_cmd_desc novatek_manufacture_id_cmd = {
	DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(manufacture_id), manufacture_id};

static uint32 mipi_novatek_manufacture_id(struct msm_fb_data_type *mfd)
{
	struct dsi_buf *rp, *tp;
	struct dsi_cmd_desc *cmd;
	uint32 *lp;

	tp = &novatek_tx_buf;
	rp = &novatek_rx_buf;
	mipi_dsi_buf_init(rp);
	mipi_dsi_buf_init(tp);

	cmd = &novatek_manufacture_id_cmd;
	mipi_dsi_cmds_rx(mfd, tp, rp, cmd, 3);
	lp = (uint32 *)rp->data;
	pr_info("%s: manufacture_id=%x", __func__, *lp);
	return *lp;
}

static int mipi_novatek_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;

	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	mipi  = &mfd->panel_info.mipi;

	mutex_lock(&mfd->dma->ov_mutex);
	if (mipi->mode == DSI_VIDEO_MODE) {
		mipi_dsi_cmds_tx(mfd, &novatek_tx_buf, novatek_video_on_cmds,
			ARRAY_SIZE(novatek_video_on_cmds));
	} else {
		mipi_dsi_cmds_tx(mfd, &novatek_tx_buf, novatek_cmd_on_cmds,
			ARRAY_SIZE(novatek_cmd_on_cmds));

		mipi_dsi_cmd_bta_sw_trigger(); /* clean up ack_err_status */

		mipi_novatek_manufacture_id(mfd);
	}
	mutex_unlock(&mfd->dma->ov_mutex);

	return 0;
}

static int mipi_novatek_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	mutex_lock(&mfd->dma->ov_mutex);
	mipi_dsi_cmds_tx(mfd, &novatek_tx_buf, novatek_display_off_cmds,
			ARRAY_SIZE(novatek_display_off_cmds));
	mutex_unlock(&mfd->dma->ov_mutex);

	return 0;
}

#ifdef NOVATEK_BACKLIGHT
static int mipi_dsi_cmd_set_backlight(struct msm_fb_data_type *mfd)
{
	struct mipi_panel_info *mipi;
	static int bl_level_old;

	mipi  = &mfd->panel_info.mipi;
	if (bl_level_old == mfd->bl_level)
		return 0;

	led_pwm1[1] = (unsigned char)(mfd->bl_level);

	mutex_lock(&mfd->dma->ov_mutex);
	if (mipi->mode == DSI_CMD_MODE) {
		mipi_dsi_cmds_tx(mfd, &novatek_tx_buf,
				novatek_cmd_backlight_cmds,
				ARRAY_SIZE(novatek_cmd_backlight_cmds));
		bl_level_old = mfd->bl_level;
	}
	mutex_unlock(&mfd->dma->ov_mutex);
	return 0;
}
#endif

static int __devinit mipi_novatek_lcd_probe(struct platform_device *pdev)
{
	if (pdev->id == 0) {
		mipi_novatek_pdata = pdev->dev.platform_data;
		return 0;
	}

	msm_fb_add_device(pdev);

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_novatek_lcd_probe,
	.driver = {
		.name   = "mipi_novatek",
	},
};

static struct msm_fb_panel_data novatek_panel_data = {
	.on		= mipi_novatek_lcd_on,
	.off		= mipi_novatek_lcd_off,
};

static int ch_used[3];

int mipi_novatek_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	pdev = platform_device_alloc("mipi_novatek", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	novatek_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &novatek_panel_data,
		sizeof(novatek_panel_data));
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int __init mipi_novatek_lcd_init(void)
{
	mipi_dsi_buf_alloc(&novatek_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&novatek_rx_buf, DSI_BUF_SIZE);

	return platform_driver_register(&this_driver);
}

module_init(mipi_novatek_lcd_init);
