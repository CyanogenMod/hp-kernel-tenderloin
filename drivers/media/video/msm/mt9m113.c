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

#include "mt9m113.h"

/* Aptina MT9M113 Registers and their values */
/* Sensor Core Registers */
#define  REG_MT9M113_MODEL_ID 0x0000
#define  MT9M113_MODEL_ID     0x2480

/*  SOC Registers Page 1  */
#define  REG_MT9M113_SENSOR_RESET     0x301A
#define  REG_MT9M113_STANDBY_CONTROL  0x3202
#define  REG_MT9M113_MCU_BOOT         0x3386

#define SENSOR_DEBUG 0

#define AE_MAX_DGAIN_AE1	0x2212 /* default 0x00A0 */
#define AE_SKIP_FRAMES		0xA208 /* default 0x0002 */
#define AE_JUMP_DIVISOR		0xA209 /* default 0x0002 */
#define AE_MAX_INDEX		0xA20C /* default 0x0028 */
#define AE_MAX_VIRTGAIN		0xA20E /* default 0x0080 */
#define AE_TARGETMAX		0xA24B /* default 0x0096 */
#define AE_TARGET_BUFFER_SPEED	0xA24C /* default 0x000C */
#define AE_BASETARGET		0xA24F /* default 0x0036 */

#define AWB_SATURATION		0xA354 /* default 0x0080 */

#define SEQ_MODE		0xA102 /* */
#define SEQ_CMD			0xA103 /* */
#define SEQ_CAP_MODE		0xA115 /* */
#define SEQ_CAP_NUM_FRAMES	0xA116 /* */

#define CORE_0248		0x0248 /* default N/A */

#define REG_ADDR		0x098C
#define REG_VALUE		0x0990

struct mt9m113_work {
	struct work_struct work;
};

static struct  mt9m113_work *mt9m113_sensorw;
static struct  i2c_client *mt9m113_client;

struct mt9m113_ctrl {
	const struct msm_camera_sensor_info *sensordata;
};


static struct mt9m113_ctrl *mt9m113_ctrl;

static DECLARE_WAIT_QUEUE_HEAD(mt9m113_wait_queue);
DECLARE_MUTEX(mt9m113_sem);
static int16_t mt9m113_effect = CAMERA_EFFECT_OFF;
static int32_t config_csi;

/*=============================================================
	EXTERNAL DECLARATIONS
==============================================================*/
extern struct mt9m113_reg mt9m113_regs;


/*=============================================================*/

static int32_t mt9m113_i2c_txdata(unsigned short saddr,
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
		if (i2c_transfer(mt9m113_client->adapter, msg, 1) < 0) {
			printk("mt9m113_i2c_txdata Addr:0x%x failed %d times!\n",saddr,I2C_RETRY);
			if(4==I2C_RETRY)
			{
				printk("mt9m113_i2c_txdata failed!\n");
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

static int32_t mt9m113_i2c_write(unsigned short saddr,
	unsigned short waddr, unsigned short wdata, enum mt9m113_width width)
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

			rc = mt9m113_i2c_txdata(saddr>>1, buf, 4);
		}
		break;

		case BYTE_LEN: {
			buf[0] = waddr;
			buf[1] = wdata;
			rc = mt9m113_i2c_txdata(saddr>>1, buf, 2);
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

static int mt9m113_i2c_rxdata(unsigned short saddr,
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
		if (i2c_transfer(mt9m113_client->adapter, msgs, 2) < 0) {
			printk("mt9m113_i2c_rxdata Addr:0x%x failed %d times!\n",saddr,I2C_RETRY);
			if(4==I2C_RETRY)
			{
				printk("mt9m113_i2c_rxdata failed!\n");
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

static int32_t mt9m113_i2c_read(unsigned short   saddr,
	unsigned short raddr, unsigned short *rdata, enum mt9m113_width width)
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

			rc = mt9m113_i2c_rxdata(saddr>>1, buf, 2);
			if (rc < 0)
				return rc;

			*rdata = buf[0] << 8 | buf[1];
			}
			break;

		default:
			break;
	}

	if (rc < 0)
		printk("mt9m113_i2c_read REG:%x failed!\n",raddr);

	return rc;
}

static void mt9m113_suspend_update_double_buffer(void)
{
	unsigned short value = 0x00;

	mt9m113_i2c_write(mt9m113_client->addr, REG_ADDR , CORE_0248 , WORD_LEN);
	mt9m113_i2c_read(mt9m113_client->addr, REG_VALUE , &value, WORD_LEN);

	CDBG("%s: CORE_0248:0x%04x\n", __func__, value);

	mt9m113_i2c_write(mt9m113_client->addr, REG_ADDR , CORE_0248 , WORD_LEN);
	mt9m113_i2c_write(mt9m113_client->addr, REG_VALUE , value | 0x8000, WORD_LEN);
}

static void mt9m113_resume_update_double_buffer(void)
{
	unsigned short value = 0x00;

	mt9m113_i2c_write(mt9m113_client->addr, REG_ADDR , CORE_0248 , WORD_LEN);
	mt9m113_i2c_read(mt9m113_client->addr, REG_VALUE , &value, WORD_LEN);

	CDBG("%s: CORE_0248:0x%04x\n", __func__, value);

	mt9m113_i2c_write(mt9m113_client->addr, REG_ADDR , CORE_0248 , WORD_LEN);
	mt9m113_i2c_write(mt9m113_client->addr, REG_VALUE , value & 0x7FFF, WORD_LEN);
}

static int32_t mt9m113_i2c_write_table(
	struct mt9m113_i2c_reg_conf const *reg_conf_tbl,
	int num_of_items_in_table)
{
	int i;
	int32_t rc = -EIO;

	mt9m113_suspend_update_double_buffer();

	for (i = 0; i < num_of_items_in_table; i++) {
		unsigned short rb_wdata = 0;
		if (reg_conf_tbl->wmask != 0xFFFF) {
			rc = mt9m113_i2c_read(mt9m113_client->addr,reg_conf_tbl->waddr, &rb_wdata, reg_conf_tbl->width);
			if (rc < 0)
				break;
			rc = mt9m113_i2c_write(mt9m113_client->addr,reg_conf_tbl->waddr,
				(reg_conf_tbl->wdata != 0) ?
				(rb_wdata | reg_conf_tbl->wmask) :
				(rb_wdata & ~reg_conf_tbl->wmask),
				reg_conf_tbl->width);
			if (rc < 0)
				break;
		}
		else {	
			rc = mt9m113_i2c_write(mt9m113_client->addr,reg_conf_tbl->waddr, reg_conf_tbl->wdata,
				reg_conf_tbl->width);
			if (rc < 0)
				break;
		}
		if (reg_conf_tbl->mdelay_time != 0)
			mdelay(reg_conf_tbl->mdelay_time);
		reg_conf_tbl++;
	}

	mt9m113_resume_update_double_buffer();

	return rc;
}



static int32_t mt9m113_reg_init(void)
{
	int32_t rc=0;
	unsigned short value=0,count=0;

	rc = mt9m113_i2c_write_table(&mt9m113_regs.prev_snap_reg_tbl[0],
			mt9m113_regs.prev_snap_reg_tbl_size);
	if(rc<0)
		return rc;
	//Move last part of initialize list here, and also use polling method to make sure MCU status ready before leave initialize.
	//Polling below can resolve "black screen issue" --0129
	do
	{
		rc = mt9m113_i2c_write(mt9m113_client->addr, 0x098C, 0xA103, WORD_LEN);
		if (rc < 0)
			return rc;
		rc = mt9m113_i2c_read(mt9m113_client->addr, 0x0990, &value, WORD_LEN);
		if (rc < 0)
			return rc;

		if(value!=0)
		{
			mdelay(10);
			count++;
		}
		else
		{
			CDBG("%s MCU refresh completed count:%d\n", __func__, count);
		}
	}while(value!=0 && count<100);

	rc = mt9m113_i2c_write(mt9m113_client->addr, 0x098C, 0xA103, WORD_LEN);
	if (rc < 0)
		return rc;
	rc = mt9m113_i2c_write(mt9m113_client->addr, 0x0990, 0x0005, WORD_LEN);
	if (rc < 0)
		return rc;

	do
	{
		rc = mt9m113_i2c_write(mt9m113_client->addr, 0x098C, 0xA103, WORD_LEN);
		if (rc < 0)
			return rc;
		rc = mt9m113_i2c_read(mt9m113_client->addr, 0x0990, &value, WORD_LEN);
		if (rc < 0)
			return rc;

		if(value!=0)
		{
			mdelay(10);
			count++;
		}
		else
		{
			CDBG("%s MCU refresh completed count:%d\n", __func__, count);
		}
	}while(value!=0 && count<100);
	//0x7a08 will enable LP mode, while 0x7A0C will let MIPI clock continuous.
	rc = mt9m113_i2c_write(mt9m113_client->addr, 0x3400, 0x7A08/*0x7A0C*/, WORD_LEN);
	if (rc < 0)
		return rc;
	CDBG("%s reg_init %d done\n", __func__, rc);
	return rc;

}

static int32_t mt9m113_set_effect(int mode, int effect)
{
	uint16_t reg_addr;
	uint16_t reg_val;
	int32_t rc = 0;

	CDBG("%s E mode=%x effect=%x\n", __func__, mode,effect );
	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		/* Context A Special Effects */
		reg_addr = 0x2759;
		break;

	case SENSOR_RAW_SNAPSHOT_MODE:
	case SENSOR_SNAPSHOT_MODE:
		/* Context B Special Effects */
		reg_addr = 0x275B;
		break;

	default:
		reg_addr = 0x2759;
		break;
	}

	switch (effect) {
	case CAMERA_EFFECT_OFF: 
		reg_val = 0x6440;

		rc = mt9m113_i2c_write(mt9m113_client->addr,
			0x098C, reg_addr, WORD_LEN);
		if (rc < 0)
			return rc;

		rc = mt9m113_i2c_write(mt9m113_client->addr,
			0x0990, reg_val, WORD_LEN);
		if (rc < 0)
			return rc;
		break;

	case CAMERA_EFFECT_MONO: 
		reg_val = 0x6441;
		rc = mt9m113_i2c_write(mt9m113_client->addr,
			0x098C, reg_addr, WORD_LEN);
		if (rc < 0)
			return rc;

		rc = mt9m113_i2c_write(mt9m113_client->addr,
			0x0990, reg_val, WORD_LEN);
		if (rc < 0)
			return rc;
		break;

	case CAMERA_EFFECT_SEPIA: 
		reg_val = 0x6442;
		rc = mt9m113_i2c_write(mt9m113_client->addr,
			0x098C, reg_addr, WORD_LEN);
		if (rc < 0)
			return rc;

		rc = mt9m113_i2c_write(mt9m113_client->addr,
			0x0990, reg_val, WORD_LEN);
		if (rc < 0)
			return rc;
		break;	

	case CAMERA_EFFECT_NEGATIVE: 
		reg_val = 0x6443;
		rc = mt9m113_i2c_write(mt9m113_client->addr,
			0x098C, reg_addr, WORD_LEN);
		if (rc < 0)
			return rc;

		rc = mt9m113_i2c_write(mt9m113_client->addr,
			0x0990, reg_val, WORD_LEN);
		if (rc < 0)
			return rc;
		break;

	case CAMERA_EFFECT_SOLARIZE: 
		reg_val = 0x6445; //SOLARIZE WITH -UV 
		//If set reg_val to 0x6444, then solarize with unchanged UV
		rc = mt9m113_i2c_write(mt9m113_client->addr,
			0x098C, reg_addr, WORD_LEN);
		if (rc < 0)
			return rc;

		rc = mt9m113_i2c_write(mt9m113_client->addr,
			0x0990, reg_val, WORD_LEN);
		if (rc < 0)
			return rc;

		break;

	default: 
		reg_val = 0x6440;
		rc = mt9m113_i2c_write(mt9m113_client->addr,
			0x098C, reg_addr, WORD_LEN);
		if (rc < 0)
			return rc;

		rc = mt9m113_i2c_write(mt9m113_client->addr,
			0x0990, reg_val, WORD_LEN);
		if (rc < 0)
			return rc;
    		break;
	}
	mt9m113_effect = effect;
	/* Refresh Sequencer */
	rc = mt9m113_i2c_write(mt9m113_client->addr,
		0x098C, 0xA103, WORD_LEN);
	if (rc < 0)
		return rc;

	rc = mt9m113_i2c_write(mt9m113_client->addr,
		0x0990, 0x0005, WORD_LEN);
	if (rc < 0)
		return rc;

	CDBG("%s X\n",__func__);
	return rc;
}

struct mt9m113_i2c_reg_conf mod_snapshot_mode_reg_tbl[] = {
	{ REG_ADDR,	0xFFFF,	AE_MAX_INDEX,		WORD_LEN, 1 },
	{ REG_VALUE,	0xFFFF,	0x0028,			WORD_LEN, 1 },
	{ REG_ADDR,	0xFFFF,	AE_MAX_VIRTGAIN,	WORD_LEN, 1 },
	{ REG_VALUE,	0xFFFF,	0x0060,			WORD_LEN, 1 },
	{ REG_ADDR,	0xFFFF,	AE_MAX_DGAIN_AE1,	WORD_LEN, 1 },
	{ REG_VALUE,	0xFFFF,	0x00C8,			WORD_LEN, 1 },
	{ REG_ADDR,	0xFFFF,	AE_JUMP_DIVISOR,	WORD_LEN, 1 },
	{ REG_VALUE,	0xFFFF,	0x0002,			WORD_LEN, 1 },
	{ REG_ADDR,	0xFFFF,	AE_SKIP_FRAMES,		WORD_LEN, 1 },
	{ REG_VALUE,	0xFFFF,	0x0002,			WORD_LEN, 1 },
};

struct mt9m113_i2c_reg_conf mod_preview_mode_reg_tbl[] = {
	{ REG_ADDR,	0xFFFF,	AE_MAX_INDEX,		WORD_LEN, 1 },
	{ REG_VALUE,	0xFFFF,	0x0008,			WORD_LEN, 1 },
	{ REG_ADDR,	0xFFFF,	AE_MAX_VIRTGAIN,	WORD_LEN, 1 },
	{ REG_VALUE,	0xFFFF,	0x00A0,			WORD_LEN, 1 },
	{ REG_ADDR,	0xFFFF,	AE_MAX_DGAIN_AE1,	WORD_LEN, 1 },
	{ REG_VALUE,	0xFFFF,	0x0150,			WORD_LEN, 1 },
	{ REG_ADDR,	0xFFFF,	AE_JUMP_DIVISOR,	WORD_LEN, 1 },
	{ REG_VALUE,	0xFFFF,	0x0001,			WORD_LEN, 1 },
	{ REG_ADDR,	0xFFFF,	AE_SKIP_FRAMES,		WORD_LEN, 1 },
	{ REG_VALUE,	0xFFFF,	0x0001,			WORD_LEN, 1 },
};

static int32_t mt9m113_set_sensor_mode(int mode)
{
	int32_t rc = 0;
	struct msm_camera_csi_params mt9m113_csi_params;
	unsigned short coarse_integration_time_A,fine_integration_time_A,
					coarse_integration_time_B,fine_integration_time_B;
	unsigned short value,count=0;

	CDBG("%s E config_csi=%d mode=%x\n",__func__,config_csi,mode);
	if (config_csi == 0) {
		mt9m113_csi_params.lane_cnt = 1;
		mt9m113_csi_params.data_format = CSI_8BIT;
		mt9m113_csi_params.lane_assign = 0xe4;
		mt9m113_csi_params.dpcm_scheme = 0;
		mt9m113_csi_params.settle_cnt = 0x14;

		CDBG("%s settle_cnt=%d\n", __func__, mt9m113_csi_params.settle_cnt  );
		rc = msm_camio_csi_config(&mt9m113_csi_params);
		mdelay(10);
		config_csi =1;

		//Configure camera to output data
		rc = mt9m113_i2c_write(mt9m113_client->addr, 0x3400, 0x7A08, WORD_LEN);
		if (rc < 0)
			return rc;
		rc = mt9m113_i2c_write(mt9m113_client->addr, 0x301A, 0x120C, WORD_LEN);
		if (rc < 0)
			return rc;
		}

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		rc = mt9m113_i2c_write(mt9m113_client->addr, 0x098C, 0xA115, WORD_LEN);
		if (rc < 0) return rc;
		rc = mt9m113_i2c_write(mt9m113_client->addr, 0x0990, 0x0030, WORD_LEN);
		if (rc < 0) return rc;
		mdelay(40);
		rc = mt9m113_i2c_write(mt9m113_client->addr, 0x098C, 0xA103, WORD_LEN);
		if (rc < 0) return rc;
		rc = mt9m113_i2c_write(mt9m113_client->addr, 0x0990, 0x0001, WORD_LEN);
		if (rc < 0) return rc;

		rc = mt9m113_i2c_write_table(&mod_preview_mode_reg_tbl[0], ARRAY_SIZE(mod_preview_mode_reg_tbl));
		if (rc < 0) return rc;

		//Delay settings here is very helpful for no preview issue(mentioned as "black screen issue"),
		//especially at low-rate clock(320M for example)
		//Use polling instead of static delay to make sure sensor ready before leave configuration
		//mdelay(20);
		break;

	case SENSOR_SNAPSHOT_MODE:
		/* Switch to lower fps for Snapshot */

		rc = mt9m113_i2c_read(mt9m113_client->addr, 0x3012, &coarse_integration_time_A, WORD_LEN);
		if (rc < 0) return rc;
		rc = mt9m113_i2c_read(mt9m113_client->addr, 0x3014, &fine_integration_time_A, WORD_LEN);
		if (rc < 0) return rc;

		rc = mt9m113_i2c_write(mt9m113_client->addr, 0x098C, 0xA115, WORD_LEN);
		if (rc < 0) return rc;
		rc = mt9m113_i2c_write(mt9m113_client->addr, 0x0990, 0x0000, WORD_LEN);
		if (rc < 0) return rc;
		mdelay(40);
		rc = mt9m113_i2c_write(mt9m113_client->addr, 0x098C, 0xA116, WORD_LEN);
		if (rc < 0) return rc;
		rc = mt9m113_i2c_write(mt9m113_client->addr, 0x0990, 0x0008, WORD_LEN);
		if (rc < 0) return rc;
		rc = mt9m113_i2c_write(mt9m113_client->addr, 0x098C, 0xA103, WORD_LEN);
		if (rc < 0) return rc;
		rc = mt9m113_i2c_write(mt9m113_client->addr, 0x0990, 0x0002, WORD_LEN);
		if (rc < 0) return rc;

		rc = mt9m113_i2c_read(mt9m113_client->addr, 0x3014, &fine_integration_time_B, WORD_LEN);
		if (rc < 0) return rc;

		rc = mt9m113_i2c_write(mt9m113_client->addr, 0x098C, 0xA102, WORD_LEN);
		if (rc < 0) return rc;
		rc = mt9m113_i2c_write(mt9m113_client->addr, 0x0990, 0x0000, WORD_LEN);
		if (rc < 0) return rc;

		coarse_integration_time_B = ((coarse_integration_time_A * 1228)
				+ fine_integration_time_A - fine_integration_time_B) / 1826;

		CDBG(KERN_ERR "+++++ MT0M113 SENSOR_SNAPSHOT_MODE coarse A= %d, coarse B= %d\n"
				,coarse_integration_time_A, coarse_integration_time_B );
		CDBG(KERN_ERR "+++++ MT0M113 SENSOR_SNAPSHOT_MODE fine_ A= %d, fine B= %d\n"
				,fine_integration_time_A, fine_integration_time_B );

		rc = mt9m113_i2c_write(mt9m113_client->addr, 0x3012, coarse_integration_time_B, WORD_LEN);
		if (rc < 0) return rc;

		rc = mt9m113_i2c_write(mt9m113_client->addr, 0x301A, 0x12CE, WORD_LEN);
		if (rc < 0) return rc;

		rc = mt9m113_i2c_write_table(&mod_snapshot_mode_reg_tbl[0], ARRAY_SIZE(mod_snapshot_mode_reg_tbl));
		if (rc < 0) return rc;

		//Use polling instead of static delay to make sure sensor ready before leave configuration.
		//mdelay(20);

		break;

	default:
		return -EINVAL;
	}
	//Codes below to ensure camera sensor data are ready before leaving configuration. 0124
	do
	{
		rc = mt9m113_i2c_write(mt9m113_client->addr, 0x098C, 0xA103, WORD_LEN);
		if (rc < 0) return rc;
		rc = mt9m113_i2c_read(mt9m113_client->addr, 0x0990, &value, WORD_LEN);
	if (rc < 0) return rc;

		if(value!=0)
		{
			msleep(10);
			count++;
		}

	}while(value!=0 && count<50);
	CDBG("%s X\n",__func__);
	return rc;
}

int mt9m113_sensor_init(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	CDBG("mt9m113_sensor_init E\n");
	mt9m113_ctrl = kzalloc(sizeof(struct mt9m113_ctrl), GFP_KERNEL);
	if (!mt9m113_ctrl) {
		printk("%s mt9m113_ctrl allocate failed!\n",__func__);
		rc = -ENOMEM;
		goto init_fail;
	}

	//In our driver codes we didn't use sensordata, which contains information from board file.
	if (data)
		mt9m113_ctrl->sensordata = data;

	//Pull CAM_PWDN pin low when camera is in using
	gpio_direction_output(data->sensor_pwd, 0);
	mdelay(20);
	/* Input MCLK = 24MHz */
	msm_camio_clk_rate_set(24000000);
	mdelay(5);

	config_csi = 0;

	rc = mt9m113_reg_init();
	if (rc < 0) {
		printk("mt9m113_sensor_init mt9m113_sensor_init_probe failed!\n");
		goto init_fail;
	}

	CDBG("mt9m113_sensor_init X\n");
	return rc;

init_fail:
	kfree(mt9m113_ctrl);
	printk("mt9m113_sensor_init failed\n");
	return rc;
}

static int mt9m113_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&mt9m113_wait_queue);
	return 0;
}

int mt9m113_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cfg_data;
	int32_t   rc = 0;

	if (copy_from_user(&cfg_data,
			(void *)argp,
			sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	/* down(&mt9m113_sem); */

	CDBG("mt9m113_ioctl, cfgtype = %d, mode = %d\n",
		cfg_data.cfgtype, cfg_data.mode);

		switch (cfg_data.cfgtype) {
		case CFG_SET_MODE:
			rc = mt9m113_set_sensor_mode(
						cfg_data.mode);
			break;

		case CFG_SET_EFFECT:
			rc = mt9m113_set_effect(cfg_data.mode,
						cfg_data.cfg.effect);
			break;

		case CFG_GET_AF_MAX_STEPS:
		default:
			rc = -EINVAL;
			break;
		}

	/* up(&mt9m113_sem); */

	return rc;
}

int mt9m113_sensor_release(void)
{
	int rc = 0;

	/* down(&mt9m113_sem); */
    //Put CAM_PWDN high when camera is not using
	gpio_direction_output(mt9m113_ctrl->sensordata->sensor_pwd, 1);
    mdelay(20);

	kfree(mt9m113_ctrl);
	/* up(&mt9m113_sem); */

	return rc;
}

static int mt9m113_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
     	printk("%s i2c_check_functionality failed!\n",__func__);		
		goto probe_failure;
	}

	mt9m113_sensorw =
		kzalloc(sizeof(struct mt9m113_work), GFP_KERNEL);

	if (!mt9m113_sensorw) {
		rc = -ENOMEM;
	    printk("%s mt9m113_sensorw alloc failed!\n",__func__);		
		goto probe_failure;
	}

	i2c_set_clientdata(client, mt9m113_sensorw);
	mt9m113_init_client(client);
	mt9m113_client = client;

	CDBG("mt9m113_i2c_probe succeeded!\n");

	return 0;

probe_failure:
	kfree(mt9m113_sensorw);
	mt9m113_sensorw = NULL;
	printk("mt9m113_i2c_probe failed!\n");
	return rc;
}

static const struct i2c_device_id mt9m113_i2c_id[] = {
	{"mt9m113", 0},
	{ },
};

static int __exit mt9m113_i2c_remove(struct i2c_client *client)
{
	struct mt9m113_work_t_t *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
	mt9m113_client = NULL;
	kfree(sensorw);
	return 0;
}

static struct i2c_driver mt9m113_i2c_driver = {
	.id_table = mt9m113_i2c_id,
	.probe  = mt9m113_i2c_probe,
	.remove = __exit_p(mt9m113_i2c_remove),
	.driver = {
		.name = "mt9m113",
	},
};

static int mt9m113_sensor_probe(const struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{
	int rc=0;
	uint16_t model_id = 0;

	CDBG("mt9m113_sensor_probe E\n");
	rc = i2c_add_driver(&mt9m113_i2c_driver);
	if (rc < 0 || mt9m113_client == NULL) {
		rc = -ENOTSUPP;
		printk("%s i2c_add_driver failed\n",__func__);
		goto probe_fail;
	}
	rc = gpio_request(info->sensor_pwd, "mt9m113");
	if (!rc) {
		CDBG("sensor_reset = %d\n", rc);
		gpio_direction_output(info->sensor_pwd, 0);
		mdelay(20);
		gpio_set_value_cansleep(info->sensor_pwd, 0);
		mdelay(20);
	}
	else
	{
		printk("%s: pwdn gpio_request failed\n",  __func__);
	}
	/* Input MCLK = 24MHz */
	msm_camio_clk_rate_set(24000000);
	mdelay(5);

	/*Check whether camera sensor version match*/
	rc = mt9m113_i2c_read(mt9m113_client->addr,
		REG_MT9M113_MODEL_ID, &model_id, WORD_LEN);
	if (rc < 0)
	{
		printk("%s mt9m113_i2c_read failed\n", __func__);
		goto probe_fail;
	}

	CDBG("mt9m113 model_id = 0x%x\n", model_id);

	/* Check if it matches it with the value in Datasheet */
	if (model_id != MT9M113_MODEL_ID) {
		printk("mt9m113 version check failed, new model_id = 0x%x\n", model_id);
		rc = -ENODEV;
		goto probe_fail;
	}


	s->s_init = mt9m113_sensor_init;
	s->s_release = mt9m113_sensor_release;
	s->s_config  = mt9m113_sensor_config;
	s->s_camera_type = FRONT_CAMERA_2D;
	s->s_mount_angle = 90;
	//Put CAM_PWDN pin high during initialize as camera is not using
	gpio_direction_output(info->sensor_pwd, 1);
	mdelay(20);
	CDBG("mt9m113_sensor_probe X\n");
	return rc;

probe_fail:
	printk("%s %s:%d failed\n", __FILE__, __func__, __LINE__);
	i2c_del_driver(&mt9m113_i2c_driver);
	return rc;
}

static int __mt9m113_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, mt9m113_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __mt9m113_probe,
	.driver = {
		.name = "msm_camera_mt9m113",
		.owner = THIS_MODULE,
	},
};

static int __init mt9m113_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(mt9m113_init);
