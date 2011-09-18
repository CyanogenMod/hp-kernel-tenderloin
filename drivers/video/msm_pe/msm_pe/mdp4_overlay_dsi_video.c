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
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/fb.h>
#include <asm/system.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include "mdp.h"
#include "msm_fb.h"
#include "mdp4.h"

#define DSI_VIDEO_BASE	0xE0000

static int first_pixel_start_x;
static int first_pixel_start_y;

static struct mdp4_overlay_pipe *dsi_pipe;

int mdp4_dsi_video_on(struct platform_device *pdev)
{
	int dsi_width;
	int dsi_height;
	int dsi_bpp;
	int dsi_border_clr;
	int dsi_underflow_clr;
	int dsi_hsync_skew;

	int hsync_period;
	int hsync_ctrl;
	int vsync_period;
	int display_hctl;
	int display_v_start;
	int display_v_end;
	int active_hctl;
	int active_h_start;
	int active_h_end;
	int active_v_start;
	int active_v_end;
	int ctrl_polarity;
	int h_back_porch;
	int h_front_porch;
	int v_back_porch;
	int v_front_porch;
	int hsync_pulse_width;
	int vsync_pulse_width;
	int hsync_polarity;
	int vsync_polarity;
	int data_en_polarity;
	int hsync_start_x;
	int hsync_end_x;
	uint8 *buf;
	int bpp, ptype;
	struct fb_info *fbi;
	struct fb_var_screeninfo *var;
	struct msm_fb_data_type *mfd;
	struct mdp4_overlay_pipe *pipe;
	int ret;

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;

	if (mfd->key != MFD_KEY)
		return -EINVAL;

	fbi = mfd->fbi;
	var = &fbi->var;

	bpp = fbi->var.bits_per_pixel / 8;
	buf = (uint8 *) fbi->fix.smem_start;
	buf += fbi->var.xoffset * bpp +
		fbi->var.yoffset * fbi->fix.line_length;

	if (dsi_pipe == NULL) {
		ptype = mdp4_overlay_format2type(mfd->fb_imgType);
		if (ptype < 0)
			printk(KERN_INFO "%s: format2type failed\n", __func__);
		pipe = mdp4_overlay_pipe_alloc(ptype, MDP4_MIXER0, 0);
		if (pipe == NULL) {
			printk(KERN_INFO "%s: pipe_alloc failed\n", __func__);
			return -EBUSY;
		}
		pipe->pipe_used++;
		pipe->mixer_stage  = MDP4_MIXER_STAGE_BASE;
		pipe->mixer_num  = MDP4_MIXER0;
		pipe->src_format = mfd->fb_imgType;
		mdp4_overlay_panel_mode(pipe->mixer_num, MDP4_PANEL_DSI_VIDEO);
		ret = mdp4_overlay_format2pipe(pipe);
		if (ret < 0)
			printk(KERN_INFO "%s: format2type failed\n", __func__);

		dsi_pipe = pipe; /* keep it */
	} else {
		pipe = dsi_pipe;
	}

	/* MDP cmd block enable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
	if (is_mdp4_hw_reset()) {
		mdp4_hw_init();
		outpdw(MDP_BASE + 0x0038, mdp4_display_intf);
	}

	pipe->src_height = fbi->var.yres;
	pipe->src_width = fbi->var.xres;
	pipe->src_h = fbi->var.yres;
	pipe->src_w = fbi->var.xres;
	pipe->src_y = 0;
	pipe->src_x = 0;
	pipe->srcp0_addr = (uint32) buf;
	pipe->srcp0_ystride = fbi->fix.line_length;

	mdp4_overlay_dmap_xy(pipe);	/* dma_p */
	mdp4_overlay_dmap_cfg(mfd, 1);

	mdp4_overlay_rgb_setup(pipe);

	mdp4_mixer_stage_up(pipe);

	mdp4_overlayproc_cfg(pipe);

	/*
	 * DSI timing setting
	 */
	h_back_porch = var->left_margin;
	h_front_porch = var->right_margin;
	v_back_porch = var->upper_margin;
	v_front_porch = var->lower_margin;
	hsync_pulse_width = var->hsync_len;
	vsync_pulse_width = var->vsync_len;
	dsi_border_clr = mfd->panel_info.lcdc.border_clr;
	dsi_underflow_clr = mfd->panel_info.lcdc.underflow_clr;
	dsi_hsync_skew = mfd->panel_info.lcdc.hsync_skew;
	dsi_width = mfd->panel_info.xres;
	dsi_height = mfd->panel_info.yres;
	dsi_bpp = mfd->panel_info.bpp;

	hsync_period = h_back_porch + dsi_width + h_front_porch;
	hsync_ctrl = (hsync_period << 16) | hsync_pulse_width;
	hsync_start_x = h_back_porch;
	hsync_end_x = hsync_period - h_front_porch - 1;
	display_hctl = (hsync_end_x << 16) | hsync_start_x;

	vsync_period =
	    (v_back_porch + dsi_height + v_front_porch) * hsync_period;
	display_v_start = v_back_porch * hsync_period + dsi_hsync_skew;
	display_v_end =
	    vsync_period - (v_front_porch * hsync_period) + dsi_hsync_skew - 1;

	if (dsi_width != var->xres) {
		active_h_start = hsync_start_x + first_pixel_start_x;
		active_h_end = active_h_start + var->xres - 1;
		active_hctl =
		    ACTIVE_START_X_EN | (active_h_end << 16) | active_h_start;
	} else {
		active_hctl = 0;
	}

	if (dsi_height != var->yres) {
		active_v_start =
		    display_v_start + first_pixel_start_y * hsync_period;
		active_v_end = active_v_start + (var->yres) * hsync_period - 1;
		active_v_start |= ACTIVE_START_Y_EN;
	} else {
		active_v_start = 0;
		active_v_end = 0;
	}

	dsi_underflow_clr |= 0x80000000;	/* enable recovery */
	hsync_polarity = 0;
	vsync_polarity = 0;
	data_en_polarity = 0;

	ctrl_polarity =
	    (data_en_polarity << 2) | (vsync_polarity << 1) | (hsync_polarity);

	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x4, hsync_ctrl);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x8, vsync_period);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0xc,
				vsync_pulse_width * hsync_period);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x10, display_hctl);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x14, display_v_start);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x18, display_v_end);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x1c, active_hctl);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x20, active_v_start);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x24, active_v_end);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x28, dsi_border_clr);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x2c, dsi_underflow_clr);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x30, dsi_hsync_skew);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x38, ctrl_polarity);
	mdp4_overlay_reg_flush(pipe, 1);
	mdp_histogram_ctrl(TRUE);

	ret = panel_next_on(pdev);
	if (ret == 0) {
		/* enable DSI block */
		MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE, 1);
		mdp_pipe_ctrl(MDP_OVERLAY0_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
	}
	/* MDP cmd block disable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);

	return ret;
}

int mdp4_dsi_video_off(struct platform_device *pdev)
{
	int ret = 0;

	/* MDP cmd block enable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE, 0);
	/* MDP cmd block disable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
	mdp_pipe_ctrl(MDP_OVERLAY0_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
	mdp_histogram_ctrl(FALSE);
	ret = panel_next_off(pdev);

#ifdef MIPI_DSI_RGB_UNSTAGE
	/* delay to make sure the last frame finishes */
	msleep(100);

	/* dis-engage rgb0 from mixer0 */
	if (dsi_pipe)
		mdp4_mixer_stage_down(dsi_pipe);
#endif

	return ret;
}

/*
 * mdp4_overlay1_done_dsi: called from isr
 */
void mdp4_overlay0_done_dsi_video()
{
	complete(&dsi_pipe->comp);
}


void mdp4_dsi_video_overlay(struct msm_fb_data_type *mfd)
{
	struct fb_info *fbi = mfd->fbi;
	uint8 *buf;
	int bpp;
	unsigned long flag;
	struct mdp4_overlay_pipe *pipe;

	if (!mfd->panel_power_on)
		return;

	/* no need to power on cmd block since it's dsi mode */
	bpp = fbi->var.bits_per_pixel / 8;
	buf = (uint8 *) fbi->fix.smem_start;
	buf += fbi->var.xoffset * bpp +
		fbi->var.yoffset * fbi->fix.line_length;

	mutex_lock(&mfd->dma->ov_mutex);

	pipe = dsi_pipe;
	pipe->srcp0_addr = (uint32) buf;
	mdp4_overlay_rgb_setup(pipe);
	mdp4_overlay_reg_flush(pipe, 1); /* rgb0 and mixer0 */

	/* enable irq */
	spin_lock_irqsave(&mdp_spin_lock, flag);
	mdp_enable_irq(MDP_OVERLAY0_TERM);
	INIT_COMPLETION(dsi_pipe->comp);
	mfd->dma->waiting = TRUE;
	outp32(MDP_INTR_CLEAR, INTR_OVERLAY0_DONE);
	mdp_intr_mask |= INTR_OVERLAY0_DONE;
	outp32(MDP_INTR_ENABLE, mdp_intr_mask);
	spin_unlock_irqrestore(&mdp_spin_lock, flag);
	wait_for_completion_killable(&dsi_pipe->comp);
	mdp_disable_irq(MDP_OVERLAY0_TERM);

	mdp4_stat.kickoff_dsi++;
	mdp4_overlay_resource_release();
	mutex_unlock(&mfd->dma->ov_mutex);
}
