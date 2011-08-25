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
/*
 * Qualcomm Bahama Core Driver
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/err.h>

#include <linux/i2c.h>
#include <linux/mfd/bahama.h>

#define BAHAMA_MODE				0x01

static int bahama_shadow[BAHAMA_NUM_CHILD + 1][0xff];

struct bahama bahama_modules[BAHAMA_NUM_CHILD + 1];

#define BAHAMA_VERSION_REG		0x00

/**
 * bahama_write_bit_mask - Sets n bit register using bit mask
 * @param mariba: bahama structure pointer passed by client
 * @param reg: register address
 * @param value: buffer to be written to the registers
 * @param num_bytes: n bytes to write
 * @param mask: bit mask corresponding to the registers
 *
 * @returns result of the operation.
 */
int bahama_write_bit_mask(struct bahama *bahama, u8 reg, u8 *value,
						unsigned num_bytes, u8 mask)
{
	int ret, i;
	struct i2c_msg *msg;
	u8 data[num_bytes + 1];
	u8 mask_value[num_bytes];

	if (bahama->mod_id > BAHAMA_SLAVE_ID_BAHAMA) {
		dev_err(&bahama->client->dev,
				"mod_id value unsupported in %s\n",
				__func__);
		return -EINVAL;
	}

	bahama = &bahama_modules[bahama->mod_id];

	mutex_lock(&bahama->xfer_lock);

	for (i = 0; i < num_bytes; i++)
		mask_value[i] = (bahama_shadow[bahama->mod_id][reg + i]
					& ~mask) | (value[i] & mask);

	msg = &bahama->xfer_msg[0];
	msg->addr = bahama->client->addr;
	msg->flags = 0;
	msg->len = num_bytes + 1;
	msg->buf = data;
	data[0] = reg;
	memcpy(data+1, mask_value, num_bytes);

	ret = i2c_transfer(bahama->client->adapter, bahama->xfer_msg, 1);

	/* Try again if the write fails */
	if (ret != 1)
		ret = i2c_transfer(bahama->client->adapter,
						bahama->xfer_msg, 1);

	if (ret == 1) {
		for (i = 0; i < num_bytes; i++)
			bahama_shadow[bahama->mod_id][reg + i]
							= mask_value[i];
	}

	mutex_unlock(&bahama->xfer_lock);

	return ret;
}
EXPORT_SYMBOL(bahama_write_bit_mask);

/**
 * bahama_write - Sets n bit register in Bahama
 * @param bahama: bahama structure pointer passed by client
 * @param reg: register address
 * @param value: buffer values to be written
 * @param num_bytes: n bytes to write
 *
 * @returns result of the operation.
 */
int bahama_write(struct bahama *bahama, u8 reg, u8 *value,
							unsigned num_bytes)
{
	return bahama_write_bit_mask(bahama, reg, value, num_bytes, 0xff);
}
EXPORT_SYMBOL(bahama_write);

/**
 * bahama_read_bit_mask - Reads a n bit register based on bit mask
 * @param bahama: bahama structure pointer passed by client
 * @param reg: register address
 * @param value: i2c read of the register to be stored
 * @param num_bytes: n bytes to be read.
 * @param mask: bit mask concerning its register
 *
 * @returns result of the operation.
*/
int bahama_read_bit_mask(struct bahama *bahama, u8 reg, u8 *value,
						unsigned num_bytes, u8 mask)
{
	int ret, i;

	struct i2c_msg *msg;

	if (bahama->mod_id > BAHAMA_SLAVE_ID_BAHAMA) {
		dev_err(&bahama->client->dev,
				"mod_id value unsupported in %s\n",
				__func__);
		return -EINVAL;
	}

	bahama = &bahama_modules[bahama->mod_id];

	mutex_lock(&bahama->xfer_lock);

	msg = &bahama->xfer_msg[0];
	msg->addr = bahama->client->addr;
	msg->len = 1;
	msg->flags = 0;
	msg->buf = &reg;

	msg = &bahama->xfer_msg[1];
	msg->addr = bahama->client->addr;
	msg->len = num_bytes;
	msg->flags = I2C_M_RD;
	msg->buf = value;

	ret = i2c_transfer(bahama->client->adapter, bahama->xfer_msg, 2);

	/* Try again if read fails first time */
	if (ret != 2)
		ret = i2c_transfer(bahama->client->adapter,
						bahama->xfer_msg, 2);

	if (ret == 2) {
		for (i = 0; i < num_bytes; i++) {
			bahama_shadow[bahama->mod_id][reg + i] = value[i];
			value[i] &= mask;
		}
	}

	mutex_unlock(&bahama->xfer_lock);

	return ret;
}
EXPORT_SYMBOL(bahama_read_bit_mask);

/**
 * bahama_read - Reads n bit registers in Bahama
 * @param bahama: bahama structure pointer passed by client
 * @param reg: register address
 * @param value: i2c read of the register to be stored
 * @param num_bytes: n bytes to read.
 * @param mask: bit mask concerning its register
 *
 * @returns result of the operation.
*/
int bahama_read(struct bahama *bahama, u8 reg, u8 *value, unsigned num_bytes)
{
	return bahama_read_bit_mask(bahama, reg, value, num_bytes, 0xff);
}
EXPORT_SYMBOL(bahama_read);

static int bahama_init_reg(struct i2c_client *client)
{
	struct bahama *bahama = &bahama_modules[BAHAMA_SLAVE_ID_BAHAMA];
	u8 buf[1], reg_version;

	buf[0] = 0x18;

	bahama->mod_id = BAHAMA_SLAVE_ID_BAHAMA;

	bahama_read(bahama, BAHAMA_VERSION_REG, &reg_version, 1);
	if (!(0x80 & reg_version)) {
		dev_err(&client->dev, "Bahama not found");
		return -EINVAL;
	}

	/* Enable the Bahama Mode */
	bahama_write(bahama, BAHAMA_MODE, buf, 1);

	dev_info(&client->dev, "version: %d\n", 0x1f & reg_version);
	return 0;
}

static int bahama_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct bahama_platform_data *pdata = client->dev.platform_data;
	struct bahama *bahama;
	int status;

	if (!pdata) {
		dev_err(&client->dev, "no platform data?\n");
		return -EINVAL;
	}

	if (i2c_check_functionality(client->adapter, I2C_FUNC_I2C) == 0) {
		dev_err(&client->dev, "can't talk I2C?\n");
		return -EIO;
	}

	if (pdata->bahama_setup != NULL) {
		status = pdata->bahama_setup(&client->dev);
		if (status < 0) {
			return status;
		}
	}

	bahama = &bahama_modules[BAHAMA_SLAVE_ID_BAHAMA];
	bahama->client = client;
	strlcpy(bahama->client->name, id->name,
				sizeof(bahama->client->name));
	mutex_init(&bahama->xfer_lock);

	status = bahama_init_reg(client);
	if (status < 0) {
		pdata->bahama_shutdown(&client->dev);
		return status;
	}


	return 0;
}

static int __devexit bahama_remove(struct i2c_client *client)
{
	struct bahama_platform_data *pdata;

	pdata = client->dev.platform_data;
	bahama_modules[BAHAMA_SLAVE_ID_BAHAMA].client = NULL;

	if (pdata->bahama_shutdown != NULL)
		pdata->bahama_shutdown(&client->dev);

	return 0;
}
static struct i2c_device_id bahama_id_table[] = {
	{"bahama", 0x0},
	{}
};
MODULE_DEVICE_TABLE(i2c, bahama_id_table);

static struct i2c_driver bahama_driver = {
		.driver			= {
			.owner		=	THIS_MODULE,
			.name		= 	"bahama-core",
		},
		.id_table		=	bahama_id_table,
		.probe			=	bahama_probe,
		.remove			=	__devexit_p(bahama_remove),
};

static int __init bahama_init(void)
{
	return i2c_add_driver(&bahama_driver);
}
module_init(bahama_init);

static void __exit bahama_exit(void)
{
	i2c_del_driver(&bahama_driver);
}
module_exit(bahama_exit);

MODULE_DESCRIPTION("Bahama Top level Driver");
MODULE_ALIAS("platform:bahama-core");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
