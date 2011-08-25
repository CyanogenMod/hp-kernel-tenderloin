/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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
 * Qualcomm Bahama Core Driver header file
 */

#ifndef _BAHAMA_H
#define _BAHAMA_H_

#include <linux/types.h>
#include <linux/i2c.h>

#define BAHAMA_NUM_CHILD			1

#define BAHAMA_SLAVE_ID_BAHAMA			0x00
#define BAHAMA_SLAVE_ID_FM			0x01

struct bahama{
	struct i2c_client *client;

	struct i2c_msg xfer_msg[2];

	struct mutex xfer_lock;

	int mod_id;
};

struct bahama_top_level_platform_data{
	int slave_id;     /* Member added for eg. */
};

struct bahama_fm_platform_data{
	int irq;
	int (*fm_setup)(struct bahama_fm_platform_data *pdata);
	void (*fm_shutdown)(struct bahama_fm_platform_data *pdata);
};

/*
 * Bahama Platform Data
 * */
struct bahama_platform_data {
	struct bahama_top_level_platform_data	*bahama_tp_level;
	struct bahama_fm_platform_data		*fm;
	u8 slave_id[BAHAMA_NUM_CHILD + 1];
	int (*bahama_setup) (struct device *dev);
	void (*bahama_shutdown) (struct device *dev);
};

/*
 * Read and Write to register
 * */
int bahama_read(struct bahama *, u8 reg, u8 *value, unsigned num_bytes);
int bahama_write(struct bahama *, u8 reg, u8 *value, unsigned num_bytes);

/*
 * Read and Write single 8 bit register with bit mask
 * */
int bahama_read_bit_mask(struct bahama *, u8 reg, u8 *value,
					unsigned num_bytes, u8 mask);
int bahama_write_bit_mask(struct bahama *, u8 reg, u8 *value,
					unsigned num_bytes, u8 mask);
#endif
