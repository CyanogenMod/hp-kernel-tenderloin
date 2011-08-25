/* Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 * Qualcomm Marimba Core Driver header file
 */

#ifndef _MARIMBA_H
#define _MARIMBA_H_

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/input/msm_ts.h>
#include <mach/vreg.h>

#define MARIMBA_NUM_CHILD			4

#define MARIMBA_SLAVE_ID_MARIMBA	0x00
#define MARIMBA_SLAVE_ID_FM			0x01
#define MARIMBA_SLAVE_ID_CDC		0x02
#define MARIMBA_SLAVE_ID_QMEMBIST	0x03

#define MARIMBA_ID_TSADC			0x04

#define BAHAMA_SLAVE_ID_FM_ID		0x02
#define SLAVE_ID_BAHAMA			0x05
#define SLAVE_ID_BAHAMA_FM		0x07
#define SLAVE_ID_BAHAMA_QMEMBIST	0x08

#if defined(CONFIG_ARCH_MSM7X30)
#define MARIMBA_SSBI_ADAP		0x7
#elif defined(CONFIG_ARCH_MSM8X60)
#define MARIMBA_SSBI_ADAP		0X8
#endif

enum chip_id {
	MARIMBA_ID = 0,
	TIMPANI_ID,
	BAHAMA_ID,
	CHIP_ID_MAX
};

struct marimba{
	struct i2c_client *client;

	struct i2c_msg xfer_msg[2];

	struct mutex xfer_lock;

	int mod_id;
};

struct marimba_top_level_platform_data{
	int slave_id;     /* Member added for eg. */
};

struct marimba_fm_platform_data{
	int irq;
	int (*fm_setup)(struct marimba_fm_platform_data *pdata);
	void (*fm_shutdown)(struct marimba_fm_platform_data *pdata);
	struct vreg *vreg_s2;
	struct vreg *vreg_xo_out;
};

struct marimba_codec_platform_data{
	int (*marimba_codec_power)(int vreg_on);
	void (*snddev_profile_init) (void);
};

struct marimba_tsadc_setup_params {
	bool pen_irq_en;
	bool tsadc_en;
};

enum sample_period {
	TSADC_CLK_3 = 0,
	TSADC_CLK_24,
	TSADC_CLK_36,
	TSADC_CLK_48,
	TSADC_CLK_1,
	TSADC_CLK_2,
	TSADC_CLK_6,
	TSADC_CLK_12,
	TSADC_CLOCK_MAX
};

struct marimba_tsadc_config_params2 {
	unsigned long input_clk_khz;
	enum sample_period sample_prd;
};

struct marimba_tsadc_config_params3 {
	unsigned long prechg_time_nsecs;
	unsigned long stable_time_nsecs;
	unsigned long tsadc_test_mode;
};

struct marimba_tsadc_platform_data {
	int (*marimba_tsadc_power)(int vreg_on);
	int (*init)(void);
	int (*exit)(void);
	int (*level_vote)(int vote_on);
	bool tsadc_prechg_en;
	bool can_wakeup;
	struct marimba_tsadc_setup_params setup;
	struct marimba_tsadc_config_params2 params2;
	struct marimba_tsadc_config_params3 params3;

	struct msm_ts_platform_data *tssc_data;
};

/*
 * Marimba Platform Data
 * */
struct marimba_platform_data {
	struct marimba_top_level_platform_data	*marimba_tp_level;
	struct marimba_fm_platform_data		*fm;
	struct marimba_codec_platform_data	*codec;
	struct marimba_tsadc_platform_data	*tsadc;
	u8 slave_id[(MARIMBA_NUM_CHILD + 1) * CHIP_ID_MAX];
	u32 (*marimba_setup) (void);
	void (*marimba_shutdown) (void);
	u32 (*bahama_setup) (void);
	u32 (*bahama_shutdown) (int);
	u32 (*marimba_gpio_config) (int);
	u32 (*bahama_core_config) (int type);
};

/*
 * Read and Write to register
 * */
int marimba_read(struct marimba *, u8 reg, u8 *value, unsigned num_bytes);
int marimba_write(struct marimba *, u8 reg, u8 *value, unsigned num_bytes);

/*
 * Read and Write single 8 bit register with bit mask
 * */
int marimba_read_bit_mask(struct marimba *, u8 reg, u8 *value,
					unsigned num_bytes, u8 mask);
int marimba_write_bit_mask(struct marimba *, u8 reg, u8 *value,
					unsigned num_bytes, u8 mask);

/*
 * Read and Write to TSADC registers across the SSBI
 * * */
int marimba_ssbi_read(struct marimba *, u16 reg, u8 *value, int len);
int marimba_ssbi_write(struct marimba *, u16 reg , u8 *value, int len);

/* Read and write to Timpani */
int timpani_read(struct marimba*, u8 reg, u8 *value, unsigned num_bytes);
int timpani_write(struct marimba*, u8 reg, u8 *value,
				unsigned num_bytes);

/* Get the detected codec type */
int adie_get_detected_codec_type(void);
int adie_get_detected_connectivity_type(void);
int marimba_gpio_config(int gpio_value);
bool marimba_get_fm_status(struct marimba *);
bool marimba_get_bt_status(struct marimba *);
void marimba_set_fm_status(struct marimba *, bool);
void marimba_set_bt_status(struct marimba *, bool);

#endif
