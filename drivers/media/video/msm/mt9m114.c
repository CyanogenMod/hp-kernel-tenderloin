/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>

#include "mt9m114.h"

/* Aptina MT9M114 Registers and their values */
/* Sensor Core Registers */
#define  REG_MT9M114_MODEL_ID 0x0000
#define  MT9M114_MODEL_ID     0x2481

/*  SOC Registers Page 1  */
#define  REG_MT9M114_SENSOR_RESET     0x301A
#define  REG_MT9M114_STANDBY_CONTROL  0x3202
#define  REG_MT9M114_MCU_BOOT         0x3386

#define SENSOR_DEBUG 0



struct mt9m114_work {
	struct work_struct work;
};

static struct  mt9m114_work *mt9m114_sensorw;
static struct  i2c_client *mt9m114_client;

struct mt9m114_ctrl {
	const struct msm_camera_sensor_info *sensordata;
};


static struct mt9m114_ctrl *mt9m114_ctrl;

static DECLARE_WAIT_QUEUE_HEAD(mt9m114_wait_queue);
DECLARE_MUTEX(mt9m114_sem);
//static int16_t mt9m114_effect = CAMERA_EFFECT_OFF;
static int32_t config_csi;

/*=============================================================
	EXTERNAL DECLARATIONS
==============================================================*/
extern struct mt9m114_reg mt9m114_regs;


/*=============================================================*/

static int32_t mt9m114_i2c_txdata(unsigned short saddr,
	unsigned char *txdata, int length)
{
	int32_t I2C_RETRY=0;
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};

	#if SENSOR_DEBUG
	if (length == 2)
		CDBG("msm_io_i2c_w: 0x%04x 0x%04x\n",
			*(u16 *) txdata, *(u16 *) (txdata + 2));
	else if (length == 4)
		CDBG("msm_io_i2c_w: 0x%04x\n", *(u16 *) txdata);
	else
		CDBG("msm_io_i2c_w: length = %d\n", length);
	#endif

	for(; I2C_RETRY<5; I2C_RETRY++)
	{
		if (i2c_transfer(mt9m114_client->adapter, msg, 1) < 0) {
			printk("mt9m114_i2c_txdata Addr:0x%x failed %d times!\n",saddr,I2C_RETRY);
			if(4==I2C_RETRY)
			{
				printk("mt9m114_i2c_txdata failed!\n");
				return -EIO;
			}
			mdelay(10);
		}
		else
		{
			break;
		}
	}


	return 0;
}

static int32_t mt9m114_i2c_write(unsigned short saddr,
	unsigned short waddr, unsigned short wdata, enum mt9m114_width width)
{
	int32_t rc = -EIO;
	unsigned char buf[4];

	memset(buf, 0, sizeof(buf));
	switch (width) {
		case WORD_LEN: {
			buf[0] = (waddr & 0xFF00)>>8;
			buf[1] = (waddr & 0x00FF);
			buf[2] = (wdata & 0xFF00)>>8;
			buf[3] = (wdata & 0x00FF);

			rc = mt9m114_i2c_txdata(saddr, buf, 4);
		}
		break;

		case BYTE_LEN: {
			buf[0] = (waddr & 0xFF00)>>8;
			buf[1] = (waddr & 0x00FF);
			buf[2] = wdata;
			rc = mt9m114_i2c_txdata(saddr, buf, 3);
			}
			break;

		default:
			break;
	}

	if (rc < 0)
		printk(
		"i2c_write failed, addr = 0x%x, val = 0x%x!\n",
		waddr, wdata);

	return rc;
}

static int mt9m114_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
	int32_t I2C_RETRY=0;

	struct i2c_msg msgs[] = {
		{
			.addr   = saddr,
			.flags = 0,
			.len   = 2,
			.buf   = rxdata,
		},
		{
			.addr   = saddr,
			.flags = I2C_M_RD,
			.len   = length,
			.buf   = rxdata,
		},
	};

	#if SENSOR_DEBUG
	if (length == 2)
		CDBG("msm_io_i2c_r: 0x%04x 0x%04x\n",
			*(u16 *) rxdata, *(u16 *) (rxdata + 2));
	else if (length == 4)
		CDBG("msm_io_i2c_r: 0x%04x\n", *(u16 *) rxdata);
	else
		CDBG("msm_io_i2c_r: length = %d\n", length);
	#endif

	for(; I2C_RETRY<5; I2C_RETRY++)
	{
		if (i2c_transfer(mt9m114_client->adapter, msgs, 2) < 0) {
			printk("mt9m114_i2c_rxdata Addr:0x%x failed %d times!\n",saddr,I2C_RETRY);
			if(4==I2C_RETRY)
			{
				printk("mt9m114_i2c_rxdata failed!\n");
				return -EIO;
			}
			mdelay(10);
		}
		else
		{
			break;
		}
	}
	return 0;
}

static int32_t mt9m114_i2c_read(unsigned short   saddr,
	unsigned short raddr, unsigned short *rdata, enum mt9m114_width width)
{
	int32_t rc = 0;
	unsigned char buf[4];

	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	switch (width) {
		case WORD_LEN: {
			buf[0] = (raddr & 0xFF00)>>8;
			buf[1] = (raddr & 0x00FF);

			rc = mt9m114_i2c_rxdata(saddr, buf, 2);
			if (rc < 0)
				return rc;

			*rdata = buf[0] << 8 | buf[1];
			}
			break;

		default:
			break;
	}

	if (rc < 0)
		printk("mt9m114_i2c_read REG:%x failed!\n",raddr);

	return rc;
}


static int32_t mt9m114_i2c_write_table(
	struct mt9m114_i2c_reg_conf const *reg_conf_tbl,
	int num_of_items_in_table)
{
	int i;
	int32_t rc = -EIO;

	for (i = 0; i < num_of_items_in_table; i++) {
		unsigned short rb_wdata = 0;
		if (reg_conf_tbl->wmask != 0xFFFF) {
			rc = mt9m114_i2c_read(mt9m114_client->addr,reg_conf_tbl->waddr, &rb_wdata, reg_conf_tbl->width);
			if (rc < 0)
				break;
			rc = mt9m114_i2c_write(mt9m114_client->addr,reg_conf_tbl->waddr,
				(reg_conf_tbl->wdata != 0) ?
				(rb_wdata | reg_conf_tbl->wmask) :
				(rb_wdata & ~reg_conf_tbl->wmask),
				reg_conf_tbl->width);
			if (rc < 0)
				break;
		}
		else {
			rc = mt9m114_i2c_write(mt9m114_client->addr,reg_conf_tbl->waddr, reg_conf_tbl->wdata,
				reg_conf_tbl->width);
			if (rc < 0)
				break;
		}
		if (reg_conf_tbl->mdelay_time != 0)
			mdelay(reg_conf_tbl->mdelay_time);
		reg_conf_tbl++;
	}

	return rc;
}



static int32_t mt9m114_reg_init(void)
{
	int32_t rc=0;

//Temporarily disable register initialize here, will adjust based on further tuning.
/*
	rc = mt9m114_i2c_write_table(&mt9m114_regs.prev_snap_reg_tbl[0],
								mt9m114_regs.prev_snap_reg_tbl_size);
	if(rc<0)
		{
			printk("%s failed!!\n", __func__);
			return rc;
		}
*/
	CDBG("%s done\n", __func__);
	return rc;

}

static int32_t mt9m114_set_effect(int mode, int effect)
{
	//Reserved for further use.
	return 0;
}

static int32_t mt9m114_set_sensor_mode(int mode)
{
	int32_t rc = 0;
	struct msm_camera_csi_params mt9m114_csi_params;

	CDBG("%s E config_csi=%d mode=%x\n",__func__,config_csi,mode);
	if (config_csi == 0) {
		mt9m114_csi_params.lane_cnt = 1;
		mt9m114_csi_params.data_format = CSI_8BIT;
		mt9m114_csi_params.lane_assign = 0xe4;
		mt9m114_csi_params.dpcm_scheme = 0;
		mt9m114_csi_params.settle_cnt = 0x14;

		CDBG("%s settle_cnt=%d\n", __func__, mt9m114_csi_params.settle_cnt  );
		rc = msm_camio_csi_config(&mt9m114_csi_params);
		mdelay(10);
		rc = mt9m114_i2c_write_table(&mt9m114_regs.prev_snap_reg_tbl[0],
								mt9m114_regs.prev_snap_reg_tbl_size);
		if(rc<0)
			{
				printk("%s reg_init failed!!\n", __func__);
				return rc;
			}
		config_csi =1;
		}

	switch (mode) {
	case SENSOR_PREVIEW_MODE:

		break;

	case SENSOR_SNAPSHOT_MODE:
		/* Switch to lower fps for Snapshot */

		break;

	default:
		return -EINVAL;
	}

	CDBG("%s X\n",__func__);
	return rc;
}

int mt9m114_sensor_init(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	CDBG("mt9m114_sensor_init E\n");
	mt9m114_ctrl = kzalloc(sizeof(struct mt9m114_ctrl), GFP_KERNEL);
	if (!mt9m114_ctrl) {
		printk("%s mt9m114_ctrl allocate failed!\n",__func__);
		rc = -ENOMEM;
		goto init_fail;
	}

	//In our driver codes we didn't use sensordata, which contains information from board file.
	if (data)
		mt9m114_ctrl->sensordata = data;

	/* Input MCLK = 24MHz */
	msm_camio_clk_rate_set(24000000);
	mdelay(5);

	config_csi = 0;

	rc = mt9m114_reg_init();
	if (rc < 0) {
		printk("mt9m114_sensor_init mt9m114_sensor_init_probe failed!\n");
		goto init_fail;
	}

	CDBG("mt9m114_sensor_init X\n");
	return rc;

init_fail:
	kfree(mt9m114_ctrl);
	printk("mt9m114_sensor_init failed\n");
	return rc;
}

static int mt9m114_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&mt9m114_wait_queue);
	return 0;
}

int mt9m114_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cfg_data;
	int32_t   rc = 0;

	if (copy_from_user(&cfg_data,
			(void *)argp,
			sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	/* down(&mt9m114_sem); */

	CDBG("mt9m114_ioctl, cfgtype = %d, mode = %d\n",
		cfg_data.cfgtype, cfg_data.mode);

		switch (cfg_data.cfgtype) {
		case CFG_SET_MODE:
			rc = mt9m114_set_sensor_mode(
						cfg_data.mode);
			break;

		case CFG_SET_EFFECT:
			rc = mt9m114_set_effect(cfg_data.mode,
						cfg_data.cfg.effect);
			break;

		case CFG_GET_AF_MAX_STEPS:
		default:
			rc = -EINVAL;
			break;
		}

	/* up(&mt9m114_sem); */

	return rc;
}

int mt9m114_sensor_release(void)
{
	int rc = 0;

	/* down(&mt9m114_sem); */

	kfree(mt9m114_ctrl);
	/* up(&mt9m114_sem); */

	return rc;
}

static int mt9m114_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		printk("%s i2c_check_functionality failed!\n",__func__);
		goto probe_failure;
	}

	mt9m114_sensorw =
		kzalloc(sizeof(struct mt9m114_work), GFP_KERNEL);

	if (!mt9m114_sensorw) {
		rc = -ENOMEM;
	    printk("%s mt9m114_sensorw alloc failed!\n",__func__);
		goto probe_failure;
	}

	i2c_set_clientdata(client, mt9m114_sensorw);
	mt9m114_init_client(client);
	mt9m114_client = client;

	CDBG("mt9m114_i2c_probe succeeded!\n");

	return 0;

probe_failure:
	kfree(mt9m114_sensorw);
	mt9m114_sensorw = NULL;
	printk("mt9m114_i2c_probe failed!\n");
	return rc;
}

static const struct i2c_device_id mt9m114_i2c_id[] = {
	{"mt9m114", 0},
	{ },
};

static int __exit mt9m114_i2c_remove(struct i2c_client *client)
{
	struct mt9m114_work_t_t *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
	mt9m114_client = NULL;
	kfree(sensorw);
	return 0;
}

static struct i2c_driver mt9m114_i2c_driver = {
	.id_table = mt9m114_i2c_id,
	.probe  = mt9m114_i2c_probe,
	.remove = __exit_p(mt9m114_i2c_remove),
	.driver = {
		.name = "mt9m114",
	},
};

static int mt9m114_sensor_probe(const struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{
	int rc=0;
	uint16_t model_id = 0;

	CDBG("mt9m114_sensor_probe E\n");
	rc = i2c_add_driver(&mt9m114_i2c_driver);
	if (rc < 0 || mt9m114_client == NULL) {
		rc = -ENOTSUPP;
		printk("%s i2c_add_driver failed\n",__func__);
		goto probe_fail;
	}

	/* Input MCLK = 24MHz */
	msm_camio_clk_rate_set(24000000);
	mdelay(5);

	/*Check whether camera sensor version match*/
	rc = mt9m114_i2c_read(mt9m114_client->addr,
		REG_MT9M114_MODEL_ID, &model_id, WORD_LEN);
	if (rc < 0)
	{
		printk("%s mt9m114_i2c_read failed\n", __func__);
		goto probe_fail;
	}

	CDBG("mt9m114 model_id = 0x%x\n", model_id);

	/* Check if it matches it with the value in Datasheet */
	if (model_id != MT9M114_MODEL_ID) {
		printk("mt9m114 version check failed, new model_id = 0x%x\n", model_id);
		rc = -ENODEV;
		goto probe_fail;
	}

	s->s_init = mt9m114_sensor_init;
	s->s_release = mt9m114_sensor_release;
	s->s_config  = mt9m114_sensor_config;
	s->s_camera_type = FRONT_CAMERA_2D;

	CDBG("mt9m114_sensor_probe X\n");
	return rc;

probe_fail:
	printk("%s %s:%d failed\n", __FILE__, __func__, __LINE__);
	i2c_del_driver(&mt9m114_i2c_driver);
	return rc;
}

static int __mt9m114_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, mt9m114_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __mt9m114_probe,
	.driver = {
		.name = "msm_camera_mt9m114",
		.owner = THIS_MODULE,
	},
};

static int __init mt9m114_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(mt9m114_init);
