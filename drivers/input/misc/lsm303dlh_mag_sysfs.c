/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
*
* File Name		: lsm303dlh_mag_sys.c
* Authors		: MSH - Motion Mems BU - Application Team
*			: Carmine Iascone (carmine.iascone@st.com)
*			: Matteo Dameno (matteo.dameno@st.com)
*			: Both authors are willing to be considered the contact
*			: and update points for the driver.*
* Version		: V 1.0.7
* Date			: 16/02/2011
* Description		: LSM303DLH 6D module sensor device driver
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
********************************************************************************
 Revision 1.0.7: 22/11/2010
  corrects bug in enable/disable of polling polled device;
*******************************************************************************/

#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/input-polldev.h>
#include <linux/delay.h>

#include <linux/slab.h>
#include <linux/mm.h>

#include <linux/i2c/lsm303dlh.h>


/** Maximum polled-device-reported g value */
#define H_MAX			8100

/* Magnetometer registers */
#define CRA_REG_M		0x00	/* Configuration register A */
#define CRB_REG_M		0x01	/* Configuration register B */
#define MR_REG_M		0x02	/* Mode register */

/* resume state index */
#define RES_CRA_REG_M		0	/* Configuration register A */
#define RES_CRB_REG_M		1	/* Configuration register B */
#define RES_MR_REG_M		2	/* Mode register */

/* Output register start address*/
#define OUT_X_M			0x03

/* Magnetic Sensor Operation Mode */
#define NORMAL_MODE     	0x00
#define POS_BIAS         	0x01
#define NEG_BIAS         	0x02
#define CC_MODE          	0x00
#define SC_MODE			0x01
#define SLEEP_MODE		0x03

/* Magnetometer X-Y sensitivity  */
#define XY_SENSITIVITY_1_3	1055	/* XY sensitivity at 1.3G */
#define XY_SENSITIVITY_1_9	795	/* XY sensitivity at 1.9G */
#define XY_SENSITIVITY_2_5	635	/* XY sensitivity at 2.5G */
#define XY_SENSITIVITY_4_0	430	/* XY sensitivity at 4.0G */
#define XY_SENSITIVITY_4_7	375	/* XY sensitivity at 4.7G */
#define XY_SENSITIVITY_5_6	320	/* XY sensitivity at 5.6G */
#define XY_SENSITIVITY_8_1	230	/* XY sensitivity at 8.1G */

/* Magnetometer Z sensitivity  */
#define Z_SENSITIVITY_1_3	950	/* Z sensitivity at 1.3G */
#define Z_SENSITIVITY_1_9	710	/* Z sensitivity at 1.9G */
#define Z_SENSITIVITY_2_5	570	/* Z sensitivity at 2.5G */
#define Z_SENSITIVITY_4_0	385	/* Z sensitivity at 4.0G */
#define Z_SENSITIVITY_4_7	335	/* Z sensitivity at 4.7G */
#define Z_SENSITIVITY_5_6	285	/* Z sensitivity at 5.6G */
#define Z_SENSITIVITY_8_1	205	/* Z sensitivity at 8.1G */

/* Magnetometer output data rate  */
#define LSM303DLH_MAG_ODR_75		0x00	/* 0.75Hz output data rate */
#define LSM303DLH_MAG_ODR1_5		0x04	/* 1.5Hz output data rate */
#define LSM303DLH_MAG_ODR3_0		0x08	/* 3Hz output data rate */
#define LSM303DLH_MAG_ODR7_5		0x0C	/* 7.5Hz output data rate */
#define LSM303DLH_MAG_ODR15		0x10	/* 15Hz output data rate */
#define LSM303DLH_MAG_ODR30		0x14	/* 30Hz output data rate */
#define LSM303DLH_MAG_ODR75		0x18	/* 75Hz output data rate */

#define FUZZ			0
#define FLAT			0
#define I2C_RETRY_DELAY 5
#define I2C_RETRIES     5

#define SELFTEST_CICLES		6

struct output_rate {
	int poll_rate_ms;
	u8 mask;
};

static const struct output_rate odr_table[] = {

	{	13,	LSM303DLH_MAG_ODR75},
	{	34,	LSM303DLH_MAG_ODR30},
	{	67,	LSM303DLH_MAG_ODR15},
	{	134,	LSM303DLH_MAG_ODR7_5},
	{	334,	LSM303DLH_MAG_ODR3_0},
	{	667,	LSM303DLH_MAG_ODR1_5},
	{	1334,	LSM303DLH_MAG_ODR_75},
};

struct lsm303dlh_mag_data {
	struct i2c_client *client;
	struct lsm303dlh_mag_platform_data *pdata;

	struct mutex lock;

	struct input_polled_dev *input_poll_dev;

	int hw_initialized;
	atomic_t enabled;
	atomic_t self_test_enabled;

	u16 xy_sensitivity;
	u16 z_sensitivity;

	u8 resume_state[3];

	u8 reg_addr;
};

static int lsm303dlh_mag_i2c_read(struct lsm303dlh_mag_data *mag,
				  u8 *buf, int len)
{
	int err;
    	int tries = 0;

	struct i2c_msg msgs[] = {
		{
		 .addr = mag->client->addr,
		 .flags = mag->client->flags & I2C_M_TEN,
		 .len = 1,
		 .buf = buf,
		 },
		{
		 .addr = mag->client->addr,
		 .flags = (mag->client->flags & I2C_M_TEN) | I2C_M_RD,
		 .len = len,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(mag->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);

	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		dev_err(&mag->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int lsm303dlh_mag_i2c_write(struct lsm303dlh_mag_data *mag,
				   u8 *buf, int len)
{
	int err;
	int tries =0;

	struct i2c_msg msgs[] = {
		{
		 .addr = mag->client->addr,
		 .flags = mag->client->flags & I2C_M_TEN,
		 .len = len + 1,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(mag->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);

	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&mag->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;

}

static int lsm303dlh_mag_i2c_update(struct lsm303dlh_mag_data *mag,
			u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 rdbuf[1] = { reg_address };
	u8 wrbuf[2] = { reg_address , 0x00 };

	u8 init_val;
	u8 updated_val;
	err = lsm303dlh_mag_i2c_read(mag, rdbuf, 1);
	if (!(err < 0)) {
		init_val = rdbuf[0];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		wrbuf[1] = updated_val;
		err = lsm303dlh_mag_i2c_write(mag, wrbuf, 1);
	}
	return err;
}

int lsm303dlh_mag_update_h_range(struct lsm303dlh_mag_data *mag,
								u8 new_h_range)
{
	int err = -1;
	u8 buf[2];

	switch (new_h_range) {
	case LSM303DLH_MAG_H_1_3G:
		mag->xy_sensitivity = XY_SENSITIVITY_1_3;
		mag->z_sensitivity = Z_SENSITIVITY_1_3;
		break;
	case LSM303DLH_MAG_H_1_9G:
		mag->xy_sensitivity = XY_SENSITIVITY_1_9;
		mag->z_sensitivity = Z_SENSITIVITY_1_9;
		break;
	case LSM303DLH_MAG_H_2_5G:
		mag->xy_sensitivity = XY_SENSITIVITY_2_5;
		mag->z_sensitivity = Z_SENSITIVITY_2_5;
		break;
	case LSM303DLH_MAG_H_4_0G:
		mag->xy_sensitivity = XY_SENSITIVITY_4_0;
		mag->z_sensitivity = Z_SENSITIVITY_4_0;
		break;
	case LSM303DLH_MAG_H_4_7G:
		mag->xy_sensitivity = XY_SENSITIVITY_4_7;
		mag->z_sensitivity = Z_SENSITIVITY_4_7;
		break;
	case LSM303DLH_MAG_H_5_6G:
		mag->xy_sensitivity = XY_SENSITIVITY_5_6;
		mag->z_sensitivity = Z_SENSITIVITY_5_6;
		break;
	case LSM303DLH_MAG_H_8_1G:
		mag->xy_sensitivity = XY_SENSITIVITY_8_1;
		mag->z_sensitivity = Z_SENSITIVITY_8_1;
		break;
	default:
		return -EINVAL;
	}

	if (atomic_read(&mag->enabled)) {

		buf[0] = CRB_REG_M;
		buf[1] = new_h_range;
		err = lsm303dlh_mag_i2c_write(mag, buf, 1);
		if (err < 0)
			return err;
		mag->resume_state[RES_CRB_REG_M] = new_h_range;
	}

	return 0;
}

int lsm303dlh_mag_update_odr(struct lsm303dlh_mag_data *mag,
							int poll_interval)
{
	int err = -1;
	int i;
	u8 config[2];

	for (i = ARRAY_SIZE(odr_table)-1; i >= 0; i--) {
		if (odr_table[i].poll_rate_ms <= poll_interval)
			break;
	}

	config[1] = odr_table[i].mask;
	config[1] |= NORMAL_MODE;

	if (atomic_read(&mag->enabled)) {
		config[0] = CRA_REG_M;
		err = lsm303dlh_mag_i2c_write(mag, config, 1);
		if (err < 0)
			return err;
		mag->resume_state[RES_CRA_REG_M] = config[1];
	}



	return 0;
}

static int lsm303dlh_mag_selftest(struct lsm303dlh_mag_data *mag, u8 enable)
{
	int err = -1;
	char reg_address1, mask1, bit_values1;
	char reg_address2, mask2, bit_values2;
	static int count = 0;

	if ((enable > 0) && (count < SELFTEST_CICLES) ){

		reg_address1 = CRA_REG_M;
		mask1 = 0x03;
		bit_values1 = POS_BIAS ;
		err = lsm303dlh_mag_i2c_update(mag, reg_address1, mask1,
								bit_values1);
		if (err < 0)
			return err;
		reg_address2 = MR_REG_M;
		mask2 = 0x03;
		bit_values2 = SC_MODE ;
		err = lsm303dlh_mag_i2c_update(mag, reg_address2, mask2,
								bit_values2);
		if (err < 0)
		{
			lsm303dlh_mag_i2c_update(mag, reg_address1, mask1,
				mag->resume_state[RES_CRA_REG_M]);
			return err;
		}

		mag->resume_state[RES_CRA_REG_M] = ((mask1 & bit_values1) |
				( ~mask1 & mag->resume_state[RES_CRA_REG_M]));
		mag->resume_state[RES_MR_REG_M] = ((mask2 & bit_values2) |
				( ~mask2 & mag->resume_state[RES_MR_REG_M]));
		count ++;
		atomic_set(&mag->self_test_enabled, 1);

	}
	else
	{

		reg_address1 = MR_REG_M;
		mask1 = 0x03;
		bit_values1 = CC_MODE ;
		err = lsm303dlh_mag_i2c_update(mag, reg_address1, mask1,
								bit_values1);
		if (err < 0)
			return err;
		reg_address2 = CRA_REG_M;
		mask2 = 0x03;
		bit_values2 = NORMAL_MODE ;
		err = lsm303dlh_mag_i2c_update(mag, reg_address2, mask2,
								bit_values2);
		if (err < 0)
		{
			lsm303dlh_mag_i2c_update(mag, reg_address1, mask1,
				mag->resume_state[RES_MR_REG_M]);
			return err;
		}
		mag->resume_state[RES_CRA_REG_M] = ((mask1 & bit_values1) |
				( ~mask1 & mag->resume_state[RES_CRA_REG_M]));
		mag->resume_state[RES_MR_REG_M] = ((mask2 & bit_values2) |
				( ~mask2 & mag->resume_state[RES_MR_REG_M]));
		atomic_set(&mag->self_test_enabled, 0);
		count = 0;

	}
	return err;
}

static int lsm303dlh_mag_get_data(struct lsm303dlh_mag_data *mag,
					       int *xyz)
{
	int err = -1;
	/* Data bytes from hardware HxL, HxH, HyL, HyH, HzL, HzH */
	u8 mag_data[6];
	/* x,y,z hardware data */
	int hw_d[3] = { 0 };

	mag_data[0] = OUT_X_M;
	err = lsm303dlh_mag_i2c_read(mag, mag_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = (int) (((mag_data[0]) << 8) | mag_data[1]);
	hw_d[1] = (int) (((mag_data[2]) << 8) | mag_data[3]);
	hw_d[2] = (int) (((mag_data[4]) << 8) | mag_data[5]);

	hw_d[0] = (hw_d[0] & 0x8000) ? (hw_d[0] | 0xFFFF0000) : (hw_d[0]);
	hw_d[1] = (hw_d[1] & 0x8000) ? (hw_d[1] | 0xFFFF0000) : (hw_d[1]);
	hw_d[2] = (hw_d[2] & 0x8000) ? (hw_d[2] | 0xFFFF0000) : (hw_d[2]);

	if (hw_d[0] != 0XF000)
		hw_d[0] = hw_d[0] * 1000 / mag->xy_sensitivity;
	else
		hw_d[0] = 0X8000;
	if (hw_d[1] != 0XF000)
		hw_d[1] = hw_d[1] * 1000 / mag->xy_sensitivity;
	else
		hw_d[1] = 0x8000;
	if (hw_d[2] != 0XF000)
		hw_d[2] = hw_d[2] * 1000 / mag->z_sensitivity;
	else
		hw_d[2] = 0x8000;

	if ((hw_d[mag->pdata->axis_map_x] != 0x8000) && (mag->pdata->negate_x))
		xyz[0] = -hw_d[mag->pdata->axis_map_x];
	else
		xyz[0] = hw_d[mag->pdata->axis_map_x];

	if ((hw_d[mag->pdata->axis_map_y] != 0x8000) && (mag->pdata->negate_y))
		xyz[1] = -hw_d[mag->pdata->axis_map_y];
	else
		xyz[1] = hw_d[mag->pdata->axis_map_y];

	if ((hw_d[mag->pdata->axis_map_z] != 0x8000) && (mag->pdata->negate_z))
		xyz[2] = -hw_d[mag->pdata->axis_map_z];
	else
		xyz[2] = hw_d[mag->pdata->axis_map_z];

	return err;
}

static void lsm303dlh_mag_report_values(struct lsm303dlh_mag_data *mag,
					int *xyz)
{
	struct input_dev *input = mag->input_poll_dev->input;
	input_report_abs(input, ABS_X, xyz[0]);
	input_report_abs(input, ABS_Y, xyz[1]);
	input_report_abs(input, ABS_Z, xyz[2]);
	input_sync(input);
}

static int lsm303dlh_mag_hw_init(struct lsm303dlh_mag_data *mag)
{
	int err = -1;
	u8 buf[4];

	buf[0] = CRA_REG_M;
	buf[1] = mag->resume_state[RES_CRA_REG_M];
	buf[2] = mag->resume_state[RES_CRB_REG_M];
	buf[3] = mag->resume_state[RES_MR_REG_M];
	err = lsm303dlh_mag_i2c_write(mag, buf, 3);

	if (err < 0)
		return err;

	mag->hw_initialized = 1;

	return 0;
}

static void lsm303dlh_mag_device_power_off(struct lsm303dlh_mag_data *mag)
{
	int err;
	u8 buf[2] = { MR_REG_M, SLEEP_MODE };

	err = lsm303dlh_mag_i2c_write(mag, buf, 1);
	if (err < 0)
		dev_err(&mag->client->dev, "soft power off failed\n");

	if (mag->pdata->power_off) {
		mag->pdata->power_off();
		mag->hw_initialized = 0;
	}
}

static int lsm303dlh_mag_device_power_on(struct lsm303dlh_mag_data *mag)
{
	int err;
	u8 buf[2] = { MR_REG_M, NORMAL_MODE };

	if (mag->pdata->power_on) {
		err = mag->pdata->power_on();
		if (err < 0)
			return err;
	}

	if (!mag->hw_initialized) {
		err = lsm303dlh_mag_hw_init(mag);
		if (err < 0) {
			lsm303dlh_mag_device_power_off(mag);
			return err;
		}
	} else {
		err = lsm303dlh_mag_i2c_write(mag, buf, 1);
	}

	return 0;
}

static int lsm303dlh_mag_enable(struct lsm303dlh_mag_data *mag)
{
	int err;

	if (!atomic_cmpxchg(&mag->enabled, 0, 1)) {

		err = lsm303dlh_mag_device_power_on(mag);
		if (err < 0) {
			atomic_set(&mag->enabled, 0);
			return err;
		}
		schedule_delayed_work(&mag->input_poll_dev->work,
				      msecs_to_jiffies(mag->
						       pdata->poll_interval));
	}

	return 0;
}

static int lsm303dlh_mag_disable(struct lsm303dlh_mag_data *mag)
{
	if (atomic_cmpxchg(&mag->enabled, 1, 0)) {
		cancel_delayed_work_sync(&mag->input_poll_dev->work);
		lsm303dlh_mag_device_power_off(mag);
	}

	return 0;
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct lsm303dlh_mag_data *mag = dev_get_drvdata(dev);
	mutex_lock(&mag->lock);
	val = mag->pdata->poll_interval;
	mutex_unlock(&mag->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct lsm303dlh_mag_data *mag = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	mutex_lock(&mag->lock);
	mag->pdata->poll_interval = interval_ms;
	lsm303dlh_mag_update_odr(mag, interval_ms);
	mutex_unlock(&mag->lock);
	return size;
}

static ssize_t attr_get_range(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lsm303dlh_mag_data *mag = dev_get_drvdata(dev);
	int range = 0;
	char val;
	mutex_lock(&mag->lock);
	val = mag->pdata->h_range;
	printk(KERN_INFO "%s, h_range = %d", __func__, val);
	switch (val) {
	case LSM303DLH_MAG_H_1_3G:
		range = 1300;
		break;
	case LSM303DLH_MAG_H_1_9G:
		range = 1900;
		break;
	case LSM303DLH_MAG_H_2_5G:
		range = 2500;
		break;
	case LSM303DLH_MAG_H_4_0G:
		range = 4000;
		break;
	case LSM303DLH_MAG_H_4_7G:
		range = 4700;
		break;
	case LSM303DLH_MAG_H_5_6G:
		range = 5600;
		break;
	case LSM303DLH_MAG_H_8_1G:
		range = 8100;
		break;
	}
	mutex_unlock(&mag->lock);
	return sprintf(buf, "%d mGauss\n", range);
}

static ssize_t attr_set_range(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct lsm303dlh_mag_data *mag = dev_get_drvdata(dev);
	unsigned long val;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	mutex_lock(&mag->lock);
	mag->pdata->h_range = val;
	lsm303dlh_mag_update_h_range(mag, val);
	mutex_unlock(&mag->lock);
	return size;
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lsm303dlh_mag_data *mag = dev_get_drvdata(dev);
	int val = atomic_read(&mag->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lsm303dlh_mag_data *mag = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm303dlh_mag_enable(mag);
	else
		lsm303dlh_mag_disable(mag);

	return size;
}

static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct lsm303dlh_mag_data *mag = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&mag->lock);
	x[0] = mag->reg_addr;
	mutex_unlock(&mag->lock);
	x[1] = val;
	rc = lsm303dlh_mag_i2c_write(mag, x, 1);
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct lsm303dlh_mag_data *mag = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&mag->lock);
	data = mag->reg_addr;
	mutex_unlock(&mag->lock);
	rc = lsm303dlh_mag_i2c_read(mag, &data, 1);
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct lsm303dlh_mag_data *gyro = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&gyro->lock);

	gyro->reg_addr = val;

	mutex_unlock(&gyro->lock);

	return size;
}

static struct device_attribute attributes[] = {
	__ATTR(pollrate_ms, 0666, attr_get_polling_rate, attr_set_polling_rate),
	__ATTR(range, 0666, attr_get_range, attr_set_range),
	__ATTR(enable, 0666, attr_get_enable, attr_set_enable),
	__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0200, NULL, attr_addr_set),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}

static void lsm303dlh_mag_input_poll_func(struct input_polled_dev *dev)
{
	struct lsm303dlh_mag_data *mag = dev->private;

	int xyz[3] = { 0 };

	int err;

	if (atomic_read(&mag->self_test_enabled))
		lsm303dlh_mag_selftest(mag, 0x01);
	mutex_lock(&mag->lock);
	err = lsm303dlh_mag_get_data(mag, xyz);
	if (err < 0)
		dev_err(&mag->client->dev, "get_magnetometer_data failed\n");
	else
		lsm303dlh_mag_report_values(mag, xyz);

	mutex_unlock(&mag->lock);
}

int lsm303dlh_mag_input_open(struct input_dev *input)
{
	struct lsm303dlh_mag_data *mag = input_get_drvdata(input);

	return lsm303dlh_mag_enable(mag);
}

void lsm303dlh_mag_input_close(struct input_dev *dev)
{
	struct lsm303dlh_mag_data *mag = input_get_drvdata(dev);

	lsm303dlh_mag_disable(mag);
}

static int lsm303dlh_mag_validate_pdata(struct lsm303dlh_mag_data *mag)
{
	mag->pdata->poll_interval = max(mag->pdata->poll_interval,
					mag->pdata->min_interval);

	if (mag->pdata->axis_map_x > 2 ||
	    mag->pdata->axis_map_y > 2 || mag->pdata->axis_map_z > 2) {
		dev_err(&mag->client->dev,
			"invalid axis_map value x:%u y:%u z%u\n",
			mag->pdata->axis_map_x, mag->pdata->axis_map_y,
			mag->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (mag->pdata->negate_x > 1 || mag->pdata->negate_y > 1 ||
	    mag->pdata->negate_z > 1) {
		dev_err(&mag->client->dev,
			"invalid negate value x:%u y:%u z:%u\n",
			mag->pdata->negate_x, mag->pdata->negate_y,
			mag->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (mag->pdata->poll_interval < mag->pdata->min_interval) {
		dev_err(&mag->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lsm303dlh_mag_input_init(struct lsm303dlh_mag_data *mag)
{
	int err = -1;
	struct input_dev *input;


	mag->input_poll_dev = input_allocate_polled_device();
	if (!mag->input_poll_dev) {
		err = -ENOMEM;
		dev_err(&mag->client->dev, "input device allocate failed\n");
		goto err0;
	}

	mag->input_poll_dev->private = mag;
	mag->input_poll_dev->poll = lsm303dlh_mag_input_poll_func;
	mag->input_poll_dev->poll_interval = mag->pdata->poll_interval;

	input = mag->input_poll_dev->input;

	input->open = lsm303dlh_mag_input_open;
	input->close = lsm303dlh_mag_input_close;

	input->id.bustype = BUS_I2C;
	input->dev.parent = &mag->client->dev;

	input_set_drvdata(mag->input_poll_dev->input, mag);

	set_bit(EV_ABS, input->evbit);

	input_set_abs_params(input, ABS_X, -H_MAX, H_MAX, FUZZ, FLAT);
	input_set_abs_params(input, ABS_Y, -H_MAX, H_MAX, FUZZ, FLAT);
	input_set_abs_params(input, ABS_Z, -H_MAX, H_MAX, FUZZ, FLAT);

	input->name = LSM303DLH_MAG_DEV_NAME;

	err = input_register_polled_device(mag->input_poll_dev);
	if (err) {
		dev_err(&mag->client->dev,
			"unable to register input polled device %s\n",
			mag->input_poll_dev->input->name);
		goto err1;
	}

	return 0;

err1:
	input_free_polled_device(mag->input_poll_dev);
err0:
	return err;
}

static void lsm303dlh_mag_input_cleanup(struct lsm303dlh_mag_data *mag)
{
	input_unregister_polled_device(mag->input_poll_dev);
	input_free_polled_device(mag->input_poll_dev);
}

static int lsm303dlh_mag_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct lsm303dlh_mag_data *mag;

	int err = -1;

	pr_err("%s: probe start\n", LSM303DLH_MAG_DEV_NAME);

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL. exiting.\n");
		err = -ENODEV;
		goto err0;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto err0;
	}

	mag = kzalloc(sizeof(*mag), GFP_KERNEL);
	if (mag == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto err0;
	}

	mutex_init(&mag->lock);
	mutex_lock(&mag->lock);
	mag->client = client;

	mag->pdata = kmalloc(sizeof(*mag->pdata), GFP_KERNEL);
	if (mag->pdata == NULL)
		goto err1;

	memcpy(mag->pdata, client->dev.platform_data, sizeof(*mag->pdata));

	err = lsm303dlh_mag_validate_pdata(mag);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto err1_1;
	}

	i2c_set_clientdata(client, mag);

	if (mag->pdata->init) {
		err = mag->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err1_1;
		}
	}

	memset(mag->resume_state, 0, ARRAY_SIZE(mag->resume_state));

	mag->resume_state[RES_CRA_REG_M] = 0x90;
	mag->resume_state[RES_CRB_REG_M] = 0x20;
	mag->resume_state[RES_MR_REG_M] = 0x00;

	err = lsm303dlh_mag_device_power_on(mag);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err2;
	}

	atomic_set(&mag->enabled, 1);

	err = lsm303dlh_mag_update_h_range(mag, mag->pdata->h_range);
	if (err < 0) {
		dev_err(&client->dev, "update_h_range failed\n");
		goto err2;
	}

	err = lsm303dlh_mag_update_odr(mag, mag->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto err2;
	}

	err = lsm303dlh_mag_input_init(mag);
	if (err < 0)
		goto err3;

	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		dev_err(&client->dev, "%s register failed\n",
						LSM303DLH_MAG_DEV_NAME);
		goto err4;
	}

	lsm303dlh_mag_device_power_off(mag);

	atomic_set(&mag->enabled, 0);

	mutex_unlock(&mag->lock);

	dev_info(&client->dev, "lsm303dlh_mag probed\n");

	return 0;

err4:
	lsm303dlh_mag_input_cleanup(mag);
err3:
	lsm303dlh_mag_device_power_off(mag);
err2:
	if (mag->pdata->exit)
		mag->pdata->exit();
err1_1:
	mutex_unlock(&mag->lock);
	kfree(mag->pdata);
err1:
	kfree(mag);
err0:
	pr_err("%s: Driver Initialization failed\n", LSM303DLH_MAG_DEV_NAME);
	return err;
}

static int lsm303dlh_mag_remove(struct i2c_client *client)
{
	struct lsm303dlh_mag_data *mag = i2c_get_clientdata(client);
	#ifdef DEBUG
	pr_info(KERN_INFO "LSM303DLH driver removing\n");
	#endif
	lsm303dlh_mag_input_cleanup(mag);
	lsm303dlh_mag_device_power_off(mag);
	remove_sysfs_interfaces(&client->dev);

	kfree(mag->pdata);
	kfree(mag);
	return 0;
}

static int lsm303dlh_mag_suspend(struct device *dev)
{
	#ifdef CONFIG_SUSPEND
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlh_data *gyro = i2c_get_clientdata(client);
	#if DEBUG
	pr_info(KERN_INFO "lsm303dlh_suspend\n");
	#endif
	/* TO DO */
	#endif
	return 0;
}

static int lsm303dlh_mag_resume(struct device *dev)
{
	#ifdef CONFIG_SUSPEND
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlh_data *mag = i2c_get_clientdata(client);
	#if DEBUG
	pr_info(KERN_INFO "lsm303dlh_resume\n");
	#endif
	/* TO DO */
	#endif
	return 0;
}

static const struct i2c_device_id lsm303dlh_mag_id[] = {
	{LSM303DLH_MAG_DEV_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, lsm303dlh_mag_id);

static struct dev_pm_ops lsm303dlh_pm = {
	.suspend = lsm303dlh_mag_suspend,
	.resume = lsm303dlh_mag_resume,
};

static struct i2c_driver lsm303dlh_mag_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = LSM303DLH_MAG_DEV_NAME,
			.pm = &lsm303dlh_pm,
		   },
	.probe = lsm303dlh_mag_probe,
	.remove = __devexit_p(lsm303dlh_mag_remove),
	.id_table = lsm303dlh_mag_id,
};

static int __init lsm303dlh_mag_init(void)
{
	pr_info(KERN_INFO "lsm303dlh magnetometer driver\n");
	return i2c_add_driver(&lsm303dlh_mag_driver);
}

static void __exit lsm303dlh_mag_exit(void)
{
	i2c_del_driver(&lsm303dlh_mag_driver);
	return;
}

module_init(lsm303dlh_mag_init);
module_exit(lsm303dlh_mag_exit);

MODULE_DESCRIPTION("lsm303dlh driver for the magnetometer section");
MODULE_AUTHOR("Matteo Dameno, Carmine Iascone, STMicroelectronics");
MODULE_LICENSE("GPL");

