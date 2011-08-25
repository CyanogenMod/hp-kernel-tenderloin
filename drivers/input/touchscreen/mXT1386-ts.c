/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>

#include <asm/irq.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>

#include <mach/hardware.h>
#include <mach/io.h>
#include <mach/system.h>
#include <mach/gpio.h>
#include <mach/msm_touchpad.h>
#include "mXT1386-ts.h"

MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.2");
MODULE_DESCRIPTION("MSM Touchscreen driver");

#define X_MAX   1024
#define Y_MAX   768
#define PRESS_MAX  256
#define TS_PENUP_TIMEOUT_MS 20

static int         touchscreen_probe(struct i2c_client *client,
                  const struct i2c_device_id *id);
static void        touchscreen_work_f(struct work_struct *work);
static irqreturn_t touchscreen_interrupt(int irq, void *dev_id);
static void  Power(bool on);



#if 1
#define debug_try
#else

#endif

/* #define DEBUG */

#ifdef DEBUG
#define pr_dbg(x...) printk(KERN_ERR x)
#else
#define pr_dbg(x...)  ((void)0)
#endif

/******************************************************************************
*
*
*       QT602240 Object table init
*
*
* *****************************************************************************/
//General Object

gen_powerconfig_t7_config_t power_config = {0};                 //Power config settings.
gen_acquisitionconfig_t8_config_t acquisition_config = {0};     // Acquisition config.


//Touch Object

touch_multitouchscreen_t9_config_t touchscreen_config = {0};    //Multitouch screen config.
touch_keyarray_t15_config_t keyarray_config = {0};              //Key array config.
touch_proximity_t23_config_t proximity_config = {0};        //Proximity config.


//Signal Processing Objects

proci_gripfacesuppression_t20_config_t gripfacesuppression_config = {0};    //Grip / face suppression config.
procg_noisesuppression_t22_config_t noise_suppression_config = {0};         //Noise suppression config.
proci_onetouchgestureprocessor_t24_config_t onetouch_gesture_config = {0};  //One-touch gesture config.
proci_twotouchgestureprocessor_t27_config_t twotouch_gesture_config = {0};  //Two-touch gesture config.


//Support Objects

spt_gpiopwm_t19_config_t  gpiopwm_config = {0};             //GPIO/PWM config
spt_selftest_t25_config_t selftest_config = {0};            //Selftest config.
spt_cteconfig_t28_config_t cte_config = {0};                //Capacitive touch engine config.

spt_comcconfig_t18_config_t   comc_config = {0};            //Communication config settings.


#define QUALCOMM_VENDORID              0x5413

#define TOUCHSCREEN_NAME                  "MSM Touchscreen"
#define TOUCHSCREEN_DEVICE                "/dev/tpad"

struct touchscreen_data {
    struct input_dev *touchscreen_dev;
    struct timer_list timer;
    bool              calibrating;
    bool              calibrated;
    u32               x_min, y_min;
    u32               x_max, y_max;
    u32               x_last, y_last;
    u32               touchscreen_gpio;
    u32               touchscreen_suspend_gpio;
    struct i2c_client *clientp;
};

static struct touchscreen_data *tp_data;
static  DECLARE_COMPLETION(calibration_complete);
static  DECLARE_WORK(touchscreen_work, touchscreen_work_f);



///////////////////////////////////////////////////////
//  yegw add for touchscreen 20100118
// For sunflower touch QT602240

/*****************************************************************************
*
*
*       ATmel QT602240 Driver
*
*
* ***************************************************************************/



uint8_t write_mem( uint16_t Address, uint8_t ByteCount, uint8_t *Data )
{
    int ret;
    uint8_t ldat[ByteCount+2];
    struct touchscreen_data   *tpd = tp_data;
    struct i2c_msg msgs = {
            .addr   = tpd->clientp->addr,
            .flags  = 0,
            .buf    = (void *)ldat,
            .len    = ByteCount+2
    };
    ldat[0] = (uint8_t)Address;
    ldat[1] = (uint8_t)(Address>>8);
    memcpy(ldat+2,Data,ByteCount);


    ret = i2c_transfer( tpd->clientp->adapter, &msgs, 1);
    if(ret > 0)return READ_MEM_OK;
    return  READ_MEM_FAILED;

}

uint8_t read_mem( uint16_t Address, uint8_t ByteCount, uint8_t *Data )
{
    int ret;
    struct touchscreen_data   *tpd = tp_data;

    struct i2c_msg msgs[] = {
        [0] = {
            .addr   = tpd->clientp->addr,
            .flags  = 0,
            .buf    = (void *)&Address,
            .len    = 2
        },
        [1] = {
            .addr   = tpd->clientp->addr,
            .flags  = I2C_M_RD,
            .buf    = (void *)Data,
            .len    = ByteCount
        }
    };

    ret =  i2c_transfer(tpd->clientp->adapter, msgs, 2);
    if(ret > 0)return READ_MEM_OK;
    return  READ_MEM_FAILED;
}

/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t read_uint16_t( uint16_t Address, uint16_t *Data )
{
    uint8_t Temp[2];
    //  uint8_t *ReadDataPtr;
    uint8_t Status;

    Status = read_mem(Address, 2, Temp); // try reading 2 bytes
    // format result
    *Data = ((uint16_t)Temp[1] << 8) + (uint16_t)Temp[0];

    return Status;
}



/*****************************************************************************

*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t init_touch_driver(void)
{
    uint16_t i;
    uint8_t tmp;
    uint16_t current_address;
    uint16_t crc_address;

    info_id_t *id;
    object_t *object_table;
    uint32_t *CRC;
    uint8_t status;
    int current_report_id = 0;

 #ifdef  debug_try
    int  trytime =0;

    /* Poll device */
    while(trytime < 6)
    {
        status = read_mem(0, 1, (void *) &tmp);
        if(status == READ_MEM_OK)break;
        trytime++;
        QT_printf("[TSP]  try again  \n\r");
        mdelay(25);
    }
#else
    status = read_mem(0, 1, (void *) &tmp);
#endif

    if(status == READ_MEM_OK)
    {
        QT_printf("[TSP] found device  \n\r");
    }
    else
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }


    driver_setup = DRIVER_SETUP_INCOMPLETE;

    /* Read the info block data. */

    id = (info_id_t *) malloc(sizeof(info_id_t));
    if (id == NULL)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }

    if (read_id_block(id) != READ_MEM_OK)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }


    /* Read object table. */

    object_table = (object_t *) malloc(id->num_declared_objects * sizeof(object_t));
    if (object_table == NULL)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }

    /* Reading the whole object table block to memory directly doesn't work cause sizeof object_t
    isn't necessarily the same on every compiler/platform due to alignment issues. Endianness
    can also cause trouble. */

    current_address = OBJECT_TABLE_START_ADDRESS;

    max_report_id = 0;

    for (i = 0; i < id->num_declared_objects; i++)
    {
        status = read_mem(current_address, 1, &object_table[i].object_type);
        if (status != READ_MEM_OK)
        {
            return(DRIVER_SETUP_INCOMPLETE);
        }
        current_address++;

        status = read_uint16_t(current_address, &object_table[i].i2c_address);
        if (status != READ_MEM_OK)
        {
            return(DRIVER_SETUP_INCOMPLETE);
        }
        current_address += 2;

        status = read_mem(current_address, 1, &object_table[i].size);
        if (status != READ_MEM_OK)
        {
            return(DRIVER_SETUP_INCOMPLETE);
        }
        current_address++;

        status = read_mem(current_address, 1, &object_table[i].instances);
        if (status != READ_MEM_OK)
        {
            return(DRIVER_SETUP_INCOMPLETE);
        }
        current_address++;

        status = read_mem(current_address, 1, &object_table[i].num_report_ids);
        if (status != READ_MEM_OK)
        {
            return(DRIVER_SETUP_INCOMPLETE);
        }
        current_address++;

        max_report_id += object_table[i].num_report_ids;

        /* Find out the maximum message length. */
        if (object_table[i].object_type == GEN_MESSAGEPROCESSOR_T5)
        {
            max_message_length = object_table[i].size + 1;
        }

    }

    /* Check that message processor was found. */
    if (max_message_length == 0)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }

    /* Read CRC. */
    CRC = (uint32_t *) malloc(sizeof(info_id_t));
    if (CRC == NULL)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }


    info_block = malloc(sizeof(info_block_t));
    if (info_block == NULL)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }

    info_block->info_id = *id;

    info_block->objects = object_table;

    crc_address = OBJECT_TABLE_START_ADDRESS +
        id->num_declared_objects * OBJECT_TABLE_ELEMENT_SIZE;

    status = read_mem(crc_address, 1u, &tmp);
    if (status != READ_MEM_OK)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }
    info_block->CRC = tmp;

    status = read_mem(crc_address + 1u, 1u, &tmp);
    if (status != READ_MEM_OK)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }
    info_block->CRC |= (tmp << 8u);

    status = read_mem(crc_address + 2, 1, &info_block->CRC_hi);
    if (status != READ_MEM_OK)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }

    /* Store message processor address, it is needed often on message reads. */
    message_processor_address = get_object_address(GEN_MESSAGEPROCESSOR_T5, 0);
    if (message_processor_address == 0)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }

    /* Store command processor address. */
    command_processor_address = get_object_address(GEN_COMMANDPROCESSOR_T6, 0);
    if (command_processor_address == 0)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }

    msg = malloc(max_message_length);
    if (msg == NULL)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }

    /* Allocate memory for report id map now that the number of report id's
    * is known. */
     report_id_map = malloc(sizeof(report_id_map_t) * max_report_id + 1);

    if (report_id_map == NULL)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }

    /* Report ID 0 is reserved, so start from 1. */

    report_id_map[0].instance = 0;
    report_id_map[0].object_type = 0;
    current_report_id = 1;

    for (i = 0; i < id->num_declared_objects; i++)
    {
        if (object_table[i].num_report_ids != 0)
        {
            int instance;
            for (instance = 0; instance <= object_table[i].instances; instance++)
            {
                int start_report_id = current_report_id;
                for (; current_report_id < (start_report_id + object_table[i].num_report_ids); current_report_id++)
                {
                    report_id_map[current_report_id].instance = instance;
                    report_id_map[current_report_id].object_type =
                        object_table[i].object_type;
                }
            }
        }
    }

    if((id->version == 0x14))
    {
        if( id->build == 0x0B)
        {
            /* Store communication cfg processor address. */
            comc_cfg_address = get_object_address(SPT_COMCONFIG_T18, 0);
            if (command_processor_address == 0)
            {
                return(DRIVER_SETUP_INCOMPLETE);
            }
        }

        diagnostic_addr = get_object_address(DEBUG_DIAGNOSTIC_T37, 0);
        diagnostic_size = get_object_size(DEBUG_DIAGNOSTIC_T37);

        if (diagnostic_addr == 0)
        {
            return(DRIVER_SETUP_INCOMPLETE);
        }
    }

    driver_setup = DRIVER_SETUP_OK;

    return(DRIVER_SETUP_OK);

}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t close_touch_driver(void)
{
    free(info_block);
    free(report_id_map);
    free(msg);

    /* Do platform specific clean-up, interrupt disable etc. */
    //   platform_close();

    return(0);
}




/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t reset_chip(void)
{
    uint8_t data = 1u;
    pr_err("reset_chip\n\r");
    return(write_mem(command_processor_address + RESET_OFFSET, 1, &data));
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t calibrate_chip(void)
{
    uint8_t data = 1u;
    return(write_mem(command_processor_address + CALIBRATE_OFFSET, 1, &data));
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t diagnostic_chip(uint8_t mode)
{
    uint8_t status;
    status = write_mem(command_processor_address + DIAGNOSTIC_OFFSET, 1, &mode);
    return(status);
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t backup_config(void)
{

    /* Write 0x55 to BACKUPNV register to initiate the backup. */
    uint8_t data = 0x55u;
    return(write_mem(command_processor_address + BACKUP_OFFSET, 1, &data));
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t get_version(uint8_t *version)
{

    if (info_block)
    {
        *version = info_block->info_id.version;
    }
    else
    {
        return(ID_DATA_NOT_AVAILABLE);
    }
    return (ID_DATA_OK);
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t get_family_id(uint8_t *family_id)
{

    if (info_block)
    {
        *family_id = info_block->info_id.family_id;
    }
    else
    {
        return(ID_DATA_NOT_AVAILABLE);
    }
    return (ID_DATA_OK);
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t get_build_number(uint8_t *build)
{

    if (info_block)
    {
        *build = info_block->info_id.build;
    }
    else
    {
        return(ID_DATA_NOT_AVAILABLE);
    }
    return (ID_DATA_OK);
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t get_variant_id(uint8_t *variant)
{

    if (info_block)
    {
        *variant = info_block->info_id.variant_id;
    }
    else
    {
        return(ID_DATA_NOT_AVAILABLE);
    }
    return (ID_DATA_OK);
}



/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t write_power_config(gen_powerconfig_t7_config_t cfg)
{
    return(write_simple_config(GEN_POWERCONFIG_T7, 0, (void *) &cfg));
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t write_acquisition_config(gen_acquisitionconfig_t8_config_t cfg)
{
    return(write_simple_config(GEN_ACQUISITIONCONFIG_T8, 0, (void *) &cfg));
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t write_multitouchscreen_config(uint8_t instance, touch_multitouchscreen_t9_config_t cfg)
{
    uint16_t object_address;
    uint8_t *tmp;
    uint8_t status;
    uint8_t object_size;

    object_size = get_object_size(TOUCH_MULTITOUCHSCREEN_T9);
    if (object_size == 0)
    {
        return(CFG_WRITE_FAILED);
    }
    tmp = (uint8_t *) malloc(object_size);
    if (tmp == NULL)
    {
        return(CFG_WRITE_FAILED);
    }

    memset(tmp,0,object_size);


    /* 18 elements at beginning are 1 byte. */
    memcpy(tmp, &cfg, 18);

    /* Next two are 2 bytes. */

    *(tmp + 18) = (uint8_t) (cfg.xrange &  0xFF);
    *(tmp + 19) = (uint8_t) (cfg.xrange >> 8);

    *(tmp + 20) = (uint8_t) (cfg.yrange &  0xFF);
    *(tmp + 21) = (uint8_t) (cfg.yrange >> 8);

    /* And the last 4(8) 1 bytes each again. */

    *(tmp + 22) = cfg.xloclip;
    *(tmp + 23) = cfg.xhiclip;
    *(tmp + 24) = cfg.yloclip;
    *(tmp + 25) = cfg.yhiclip;

#if defined(__VER_1_4__)
    *(tmp + 26) = cfg.xedgectrl;
    *(tmp + 27) = cfg.xedgedist;
    *(tmp + 28) = cfg.yedgectrl;
    *(tmp + 29) = cfg.yedgedist;
#endif
    object_address = get_object_address(TOUCH_MULTITOUCHSCREEN_T9,
        instance);

    if (object_address == 0)
    {
        return(CFG_WRITE_FAILED);
    }

    status = write_mem(object_address, object_size, tmp);

    free(tmp);

    return(status);

}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t write_keyarray_config(uint8_t instance, touch_keyarray_t15_config_t cfg)
{

    return(write_simple_config(TOUCH_KEYARRAY_T15, instance, (void *) &cfg));

}



/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t write_linearization_config(uint8_t instance, proci_linearizationtable_t17_config_t cfg)
{

    uint16_t object_address;
    uint8_t *tmp;
    uint8_t status;
    uint8_t object_size;

    object_size = get_object_size(PROCI_LINEARIZATIONTABLE_T17);
    if (object_size == 0)
    {
        return(CFG_WRITE_FAILED);
    }
    tmp = (uint8_t *) malloc(object_size);

    if (tmp == NULL)
    {
        return(CFG_WRITE_FAILED);
    }

    memset(tmp,0,object_size);

    *(tmp + 0) = cfg.ctrl;
    *(tmp + 1) = (uint8_t) (cfg.xoffset & 0x00FF);
    *(tmp + 2) = (uint8_t) (cfg.xoffset >> 8);

    memcpy((tmp+3), &cfg.xsegment, 16);

    *(tmp + 19) = (uint8_t) (cfg.yoffset & 0x00FF);
    *(tmp + 20) = (uint8_t) (cfg.yoffset >> 8);

    memcpy((tmp+21), &cfg.ysegment, 16);

    object_address = get_object_address(PROCI_LINEARIZATIONTABLE_T17,
        instance);

    if (object_address == 0)
    {
        return(CFG_WRITE_FAILED);
    }

    status = write_mem(object_address, object_size, tmp);

    free(tmp);

    return(status);

}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t write_comc_config(uint8_t instance, spt_comcconfig_t18_config_t cfg)
{

    return(write_simple_config(SPT_COMCONFIG_T18, instance, (void *) &cfg));

}



/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t write_gpio_config(uint8_t instance, spt_gpiopwm_t19_config_t cfg)
{


    return(write_simple_config(SPT_GPIOPWM_T19, instance, (void *) &cfg));

}



/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t write_gripsuppression_config(uint8_t instance, proci_gripfacesuppression_t20_config_t cfg)
{

    return(write_simple_config(PROCI_GRIPFACESUPPRESSION_T20, instance,
        (void *) &cfg));
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t write_noisesuppression_config(uint8_t instance, procg_noisesuppression_t22_config_t cfg)
{

    return(write_simple_config(PROCG_NOISESUPPRESSION_T22, instance,
        (void *) &cfg));
}



/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t write_proximity_config(uint8_t instance, touch_proximity_t23_config_t cfg)
{
    uint16_t object_address;
    uint8_t *tmp;
    uint8_t status;
    uint8_t object_size;

    object_size = get_object_size(TOUCH_PROXIMITY_T23);
    if (object_size == 0)
    {
        return(CFG_WRITE_FAILED);
    }
    tmp = (uint8_t *) malloc(object_size);
    if (tmp == NULL)
    {
        return(CFG_WRITE_FAILED);
    }

    memset(tmp,0,object_size);

    *(tmp + 0) = cfg.ctrl;
    *(tmp + 1) = cfg.xorigin;
    *(tmp + 2) = cfg.yorigin;
    *(tmp + 3) = cfg.xsize;
    *(tmp + 4) = cfg.ysize;
    *(tmp + 5) = cfg.reserved_for_future_aks_usage;
    *(tmp + 6) = cfg.blen;

    *(tmp + 7) = (uint8_t) (cfg.tchthr & 0x00FF);
    *(tmp + 8) = (uint8_t) (cfg.tchthr >> 8);

    *(tmp + 9) = cfg.tchdi;
    *(tmp + 10) = cfg.average;

    *(tmp + 11) = (uint8_t) (cfg.rate & 0x00FF);
    *(tmp + 12) = (uint8_t) (cfg.rate >> 8);

    object_address = get_object_address(TOUCH_PROXIMITY_T23,
        instance);

    if (object_address == 0)
    {
        return(CFG_WRITE_FAILED);
    }

    status = write_mem(object_address, object_size, tmp);

    free(tmp);

    return(status);
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t write_onetouchgesture_config(uint8_t instance, proci_onetouchgestureprocessor_t24_config_t cfg)
{

    uint16_t object_address;
    uint8_t *tmp;
    uint8_t status;
    uint8_t object_size;

    object_size = get_object_size(PROCI_ONETOUCHGESTUREPROCESSOR_T24);
    if (object_size == 0)
    {
        return(CFG_WRITE_FAILED);
    }
    tmp = (uint8_t *) malloc(object_size);
    if (tmp == NULL)
    {
        return(CFG_WRITE_FAILED);
    }

    memset(tmp,0,object_size);

    *(tmp + 0) = cfg.ctrl;
#if defined(__VER_1_2__)
    *(tmp + 1) = 0;
#else
    *(tmp + 1) = cfg.numgest;
#endif

    *(tmp + 2) = (uint8_t) (cfg.gesten & 0x00FF);
    *(tmp + 3) = (uint8_t) (cfg.gesten >> 8);

    memcpy((tmp+4), &cfg.pressproc, 7);

    *(tmp + 11) = (uint8_t) (cfg.flickthr & 0x00FF);
    *(tmp + 12) = (uint8_t) (cfg.flickthr >> 8);

    *(tmp + 13) = (uint8_t) (cfg.dragthr & 0x00FF);
    *(tmp + 14) = (uint8_t) (cfg.dragthr >> 8);

    *(tmp + 15) = (uint8_t) (cfg.tapthr & 0x00FF);
    *(tmp + 16) = (uint8_t) (cfg.tapthr >> 8);

    *(tmp + 17) = (uint8_t) (cfg.throwthr & 0x00FF);
    *(tmp + 18) = (uint8_t) (cfg.throwthr >> 8);

    object_address = get_object_address(PROCI_ONETOUCHGESTUREPROCESSOR_T24,
        instance);

    if (object_address == 0)
    {
        return(CFG_WRITE_FAILED);
    }

    status = write_mem(object_address, object_size, tmp);

    free(tmp);

    return(status);
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t write_selftest_config(uint8_t instance, spt_selftest_t25_config_t cfg)
{

    uint16_t object_address;
    uint8_t *tmp;
    uint8_t status;
    uint8_t object_size;

    object_size = get_object_size(SPT_SELFTEST_T25);
    if (object_size == 0)
    {
        return(CFG_WRITE_FAILED);
    }
    tmp = (uint8_t *) malloc(object_size);


    if (tmp == NULL)
    {
        return(CFG_WRITE_FAILED);
    }

    memset(tmp,0,object_size);

    *(tmp + 0) = cfg.ctrl;
    *(tmp + 1) = cfg.cmd;
#if(NUM_OF_TOUCH_OBJECTS)
    *(tmp + 2) = (uint8_t) (cfg.siglim[0].upsiglim & 0x00FF);
    *(tmp + 3) = (uint8_t) (cfg.siglim[0].upsiglim >> 8);

    *(tmp + 4) = (uint8_t) (cfg.siglim[1].losiglim & 0x00FF);
    *(tmp + 5) = (uint8_t) (cfg.siglim[1].losiglim >> 8);
#endif
    object_address = get_object_address(SPT_SELFTEST_T25,
        instance);

    if (object_address == 0)
    {
        return(CFG_WRITE_FAILED);
    }

    status = write_mem(object_address, object_size, tmp);

    free(tmp);
    return(status);
}



/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t write_twotouchgesture_config(uint8_t instance, proci_twotouchgestureprocessor_t27_config_t cfg)
{

    uint16_t object_address;
    uint8_t *tmp;
    uint8_t status;
    uint8_t object_size;

    object_size = get_object_size(PROCI_TWOTOUCHGESTUREPROCESSOR_T27);
    if (object_size == 0)
    {
        return(CFG_WRITE_FAILED);
    }
    tmp = (uint8_t *) malloc(object_size);

    if (tmp == NULL)
    {
        return(CFG_WRITE_FAILED);
    }

    memset(tmp,0,object_size);

    *(tmp + 0) = cfg.ctrl;

#if defined(__VER_1_2__)
    *(tmp + 1) = 0;
#else
    *(tmp + 1) = cfg.numgest;
#endif

    *(tmp + 2) = 0;

    *(tmp + 3) = cfg.gesten;

    *(tmp + 4) = cfg.rotatethr;

    *(tmp + 5) = (uint8_t) (cfg.zoomthr & 0x00FF);
    *(tmp + 6) = (uint8_t) (cfg.zoomthr >> 8);

    object_address = get_object_address(PROCI_TWOTOUCHGESTUREPROCESSOR_T27,
        instance);

    if (object_address == 0)
    {
        return(CFG_WRITE_FAILED);
    }

    status = write_mem(object_address, object_size, tmp);

    free(tmp);

    return(status);

}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t write_CTE_config(spt_cteconfig_t28_config_t cfg)
{

    return(write_simple_config(SPT_CTECONFIG_T28, 0, (void *) &cfg));
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t write_simple_config(uint8_t object_type, uint8_t instance, void *cfg)
{
    uint16_t object_address;
    uint8_t object_size;

    object_address = get_object_address(object_type, instance);
    object_size = get_object_size(object_type);

    if ((object_size == 0) || (object_address == 0))
    {
        return(CFG_WRITE_FAILED);
    }

    return (write_mem(object_address, object_size, cfg));
}



/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t get_object_size(uint8_t object_type)
{
    uint8_t object_table_index = 0;
    uint8_t object_found = 0;
    uint16_t size = OBJECT_NOT_FOUND;

    object_t *object_table;
    object_t obj;
    object_table = info_block->objects;
    while ((object_table_index < info_block->info_id.num_declared_objects) &&
        !object_found)
    {
        obj = object_table[object_table_index];
        /* Does object type match? */
        if (obj.object_type == object_type)
        {
            object_found = 1;
            size = obj.size + 1;
        }
        object_table_index++;
    }

    return(size);
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t type_to_report_id(uint8_t object_type, uint8_t instance)
{

    uint8_t report_id = 1;
    int8_t report_id_found = 0;

    while((report_id <= max_report_id) && (report_id_found == 0))
    {
        if((report_id_map[report_id].object_type == object_type) &&
            (report_id_map[report_id].instance == instance))
        {
            report_id_found = 1;
        }
        else
        {
            report_id++;
        }
    }
    if (report_id_found)
    {
        return(report_id);
    }
    else
    {
        return(ID_MAPPING_FAILED);
    }
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t report_id_to_type(uint8_t report_id, uint8_t *instance)
{

    if (report_id <= max_report_id)
    {
        *instance = report_id_map[report_id].instance;
        return(report_id_map[report_id].object_type);
    }
    else
    {
        return(ID_MAPPING_FAILED);
    }

}



/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t read_id_block(info_id_t *id)
{
    uint8_t status;

    status = read_mem(0, 1, (void *) &id->family_id);
    if (status != READ_MEM_OK)
    {
        return(status);
    }

    status = read_mem(1, 1, (void *) &id->variant_id);
    if (status != READ_MEM_OK)
    {
        return(status);
    }

    status = read_mem(2, 1, (void *) &id->version);
    if (status != READ_MEM_OK)
    {
        return(status);
    }

    status = read_mem(3, 1, (void *) &id->build);
    if (status != READ_MEM_OK)
    {
        return(status);
    }

    status = read_mem(4, 1, (void *) &id->matrix_x_size);
    if (status != READ_MEM_OK)
    {
        return(status);
    }

    status = read_mem(5, 1, (void *) &id->matrix_y_size);
    if (status != READ_MEM_OK)
    {
        return(status);
    }

    status = read_mem(6, 1, (void *) &id->num_declared_objects);

    return(status);

}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t get_max_report_id(void)
{
    return(max_report_id);
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint16_t get_object_address(uint8_t object_type, uint8_t instance)
{
    uint8_t object_table_index = 0;
    uint8_t address_found = 0;
    uint16_t address = OBJECT_NOT_FOUND;

    object_t *object_table;
    object_t obj;
    object_table = info_block->objects;
    while ((object_table_index < info_block->info_id.num_declared_objects) &&
        !address_found)
    {
        obj = object_table[object_table_index];
        /* Does object type match? */
        if (obj.object_type == object_type)
        {

            address_found = 1;

            /* Are there enough instances defined in the FW? */
            if (obj.instances >= instance)
            {
                address = obj.i2c_address + (obj.size + 1) * instance;
            }
        }
        object_table_index++;
    }

    return(address);
}



/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint32_t get_stored_infoblock_crc()
{
    uint32_t crc;
    crc = (uint32_t) (((uint32_t) info_block->CRC_hi) << 16);
    crc = crc | info_block->CRC;
    return(crc);
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint8_t calculate_infoblock_crc(uint32_t *crc_pointer)
{

    uint32_t crc = 0;
    uint16_t crc_area_size;
    uint8_t *mem;

    uint8_t i;

    uint8_t status;

    /* 7 bytes of version data, 6 * NUM_OF_OBJECTS bytes of object table. */
    crc_area_size = 7 + info_block->info_id.num_declared_objects * 6;

    mem = (uint8_t *) malloc(crc_area_size);
    if (mem == NULL)
    {
        return(CRC_CALCULATION_FAILED);
    }

    status = read_mem(0, crc_area_size, mem);

    if (status != READ_MEM_OK)
    {
        return(CRC_CALCULATION_FAILED);
    }

    i = 0;
    while (i < (crc_area_size - 1))
    {
        crc = CRC_24(crc, *(mem + i), *(mem + i + 1));
        i += 2;
    }

    crc = CRC_24(crc, *(mem + i), 0);

    free(mem);

    /* Return only 24 bit CRC. */
    *crc_pointer = (crc & 0x00FFFFFF);
    return(CRC_CALCULATION_OK);

}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

uint32_t CRC_24(uint32_t crc, uint8_t byte1, uint8_t byte2)
{
    static const uint32_t crcpoly = 0x80001B;
    uint32_t result;
    uint16_t data_word;

    data_word = (uint16_t) ((uint16_t) (byte2 << 8u) | byte1);
    result = ((crc << 1u) ^ (uint32_t) data_word);

    if (result & 0x1000000)
    {
        result ^= crcpoly;
    }

    return(result);

}



#define __QT_CONFIG__
/*****************************************************************************
*
*
*
*
*
*       QT602240  Configuration Block
*
*
*
*
* ***************************************************************************/


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

void qt_Power_Config_Init(void)
{
    /* Set Idle Acquisition Interval to 32 ms. */
    power_config.idleacqint = 255;

    /* Set Active Acquisition Interval to 16 ms. */
    power_config.actvacqint = 255;

    /* Set Active to Idle Timeout to 4 s (one unit = 200ms). */
    power_config.actv2idleto = 0;


    /* Write power config to chip. */
    if (write_power_config(power_config) != CFG_WRITE_OK)
    {
        QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
    }
}




/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

void qt_Acquisition_Config_Init(void)
{
    acquisition_config.chrgtime = 10; // 2us
    acquisition_config.reserved = 0; // ATCHDRIFT
    acquisition_config.tchdrift = 0; // 4s
    acquisition_config.driftst = 0; // 4s
    acquisition_config.tchautocal = 0; // infinite
    acquisition_config.sync = 0; // disabled

#if defined(__VER_1_4__)
    acquisition_config.atchcalst = 5;
    acquisition_config.atchcalsthr = 0;
#endif
    acquisition_config.ATCHFRCCALTHR = 50;
    acquisition_config.ATCHFRCCALRATIO = 0;

    if (write_acquisition_config(acquisition_config) != CFG_WRITE_OK)
    {
        QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
    }
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

void qt_Multitouchscreen_Init(void)
{
    /*
        CTRL:  bit7    bit6   bit5    bit4    bit3   bit2   bit1  bit0
              SCANEN DISPRSS DISREL DISMOVE DISVECT DISAMP RPTEN ENABLE
              1      0       0      0       1       1       1     1
    */
    touchscreen_config.ctrl = 143; // enable + message-enable
    touchscreen_config.xorigin = 0;
    touchscreen_config.yorigin = 0;

    touchscreen_config.xsize = 33;  //number of channels in the X direction (bottom to top due to reorientation)
    touchscreen_config.ysize = 42;  //number of Y channels (right to left)

    touchscreen_config.akscfg = 0;
    touchscreen_config.blen = 32;  //48;  //  Sets the gain of the analog circuits in front of the ADC.
    touchscreen_config.tchthr = 70;//95;//0x1E; // The channel detection Touch Threshold value,sensitivity threshold, lower=>more sensitive
    touchscreen_config.tchdi = 2;     // touches must register above sensitivity threshold this many times before they're transmitted

    touchscreen_config.orient = 7;    // reorients the coordinates returned by the controller to match physical orientation of the display

    touchscreen_config.mrgtimeout = 0;
    touchscreen_config.movhysti = 5;  // touches must move 10 pixels to begin to register movement, this is to help keep ghost points that move several pixels suppressed
    touchscreen_config.movhystn = 5;
    touchscreen_config.movfilter = 0;  // disable movement (jitter) filter
    touchscreen_config.numtouch= 2;    // 1 -- 2 fingers supported
    touchscreen_config.mrghyst = 10;
    touchscreen_config.mrgthr = 10;
    touchscreen_config.amphyst = 10;  // prevent amplitude changes from triggering touch messages
    touchscreen_config.xrange = 768-1;  // note that the screen is mounted such that X is the long axis, this setting applies before the orientation setting
    touchscreen_config.yrange = 1024-1;
    touchscreen_config.xloclip = 0;  // crop pixels at bottom of screen
    touchscreen_config.xhiclip = 0;  // crop pixels at top of screen
    touchscreen_config.yloclip = 0;  // crop pixels at the right side of the screen (due to span bit expanding screen)
    touchscreen_config.yhiclip = 0;  // crop pixels at the left side of the screen
#if defined(__VER_1_4__)
    touchscreen_config.xedgectrl = 0;  // enable Span bit, this expands the coordinates the touch controller will return by one half channel in each direction
                                          // TODO: Enable gradient to correct for distortion that grows the farther one touches from the center of the screen
    touchscreen_config.xedgedist = 0;
    touchscreen_config.yedgectrl = 0;
    touchscreen_config.yedgedist = 0;
#endif
    touchscreen_config.JUMPLIMIT = 0;
    touchscreen_config.TCHHYST = 10;
    touchscreen_config.XPITCH = 45;
    touchscreen_config.YPITCH = 47;

    if (write_multitouchscreen_config(0, touchscreen_config) != CFG_WRITE_OK)
    {
        QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
    }
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

void qt_KeyArray_Init(void)
{
    keyarray_config.ctrl = 0;
    keyarray_config.xorigin = 0;
    keyarray_config.yorigin = 0;
    keyarray_config.xsize = 0;
    keyarray_config.ysize = 0;
    keyarray_config.akscfg = 0;
    keyarray_config.blen = 0;
    keyarray_config.tchthr = 0;
    keyarray_config.tchdi = 0;
    keyarray_config.reserved[0] = 0;
    keyarray_config.reserved[1] = 0;

    if (write_keyarray_config(0, keyarray_config) != CFG_WRITE_OK)
    {
        QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
    }
    if (write_keyarray_config(1, keyarray_config) != CFG_WRITE_OK)
    {
        QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
    }
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

void qt_ComcConfig_Init(void)
{
    comc_config.ctrl = 0;
    comc_config.cmd = 0;

    if (get_object_address(SPT_COMCONFIG_T18, 0) != OBJECT_NOT_FOUND)
    {
        if (write_comc_config(0, comc_config) != CFG_WRITE_OK)
        {
            QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
        }
    }
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

void qt_Gpio_Pwm_Init(void)
{
    gpiopwm_config.ctrl = 0;
    gpiopwm_config.reportmask = 0;
    gpiopwm_config.dir = 0;
    gpiopwm_config.intpullup = 0;
    gpiopwm_config.out = 0;
    gpiopwm_config.wake = 0;
    gpiopwm_config.pwm = 0;
    gpiopwm_config.period = 0;
    gpiopwm_config.duty[0] = 0;
    gpiopwm_config.duty[1] = 0;
    gpiopwm_config.duty[2] = 0;
    gpiopwm_config.duty[3] = 0;

    if (write_gpio_config(0, gpiopwm_config) != CFG_WRITE_OK)
    {
        QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
    }
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

void qt_Grip_Face_Suppression_Config_Init(void)
{
    gripfacesuppression_config.ctrl = 0;
    gripfacesuppression_config.xlogrip = 0;
    gripfacesuppression_config.xhigrip = 0;
    gripfacesuppression_config.ylogrip = 0;
    gripfacesuppression_config.yhigrip = 0;
    gripfacesuppression_config.maxtchs = 0;
    gripfacesuppression_config.reserved = 0;
    gripfacesuppression_config.szthr1 = 0;
    gripfacesuppression_config.szthr2 = 0;
    gripfacesuppression_config.shpthr1 = 0;
    gripfacesuppression_config.shpthr2 = 0;

#if defined(__VER_1_4__)
    gripfacesuppression_config.supextto = 0;
#endif

    /* Write grip suppression config to chip. */
    if (get_object_address(PROCI_GRIPFACESUPPRESSION_T20, 0) != OBJECT_NOT_FOUND)
    {
        if (write_gripsuppression_config(0, gripfacesuppression_config) !=
            CFG_WRITE_OK)
        {
            QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
        }
    }
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

void qt_Noise_Suppression_Config_Init(void)
{

    // -- yegw 2010-5-17  enable noise suppression function and Enables frequency hopping.
    //   Byte    Field   Bit 7   Bit 6    Bit 5    Bit 4    Bit 3    Bit 2    Bit 1    Bit 0
    //     0      CTRL   Reserved                   GCAFEN   MODE              RPTEN    ENABLE
    //
    noise_suppression_config.ctrl = 0;
    // -- End yegw

#if defined(__VER_1_2__)
    noise_suppression_config.outflen = 0;
#elif defined(__VER_1_4__)
    noise_suppression_config.reserved = 0;
#endif

    noise_suppression_config.reserved1 = 0;
    noise_suppression_config.gcaful = 0;
    noise_suppression_config.gcafll = 0;


#if defined(__VER_1_2__)
    noise_suppression_config.gcaflcount = 0;
#elif defined(__VER_1_4__)
    noise_suppression_config.actvgcafvalid = 0;
#endif

    noise_suppression_config.noisethr = 0;
    noise_suppression_config.freqhopscale = 0;


#if defined(__VER_1_2__)
    noise_suppression_config.freq0 = 0;
    noise_suppression_config.freq1 = 0;
    noise_suppression_config.freq2 = 0;
#elif defined(__VER_1_4__)
    noise_suppression_config.freq[0] = 0;
    noise_suppression_config.freq[1] = 10;
    noise_suppression_config.freq[2] = 15;
    noise_suppression_config.freq[3] = 20;
    noise_suppression_config.freq[4] = 40;
    noise_suppression_config.idlegcafvalid = 0;
#endif

    /* Write Noise suppression config to chip. */
    if (get_object_address(PROCG_NOISESUPPRESSION_T22, 0) != OBJECT_NOT_FOUND)
    {
        if (write_noisesuppression_config(0,noise_suppression_config) != CFG_WRITE_OK)
        {
            QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
        }
    }
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

void qt_Proximity_Config_Init(void)
{
    proximity_config.ctrl = 0;
    proximity_config.xorigin = 0;
    proximity_config.xsize = 0;
    proximity_config.ysize = 0;
    proximity_config.reserved_for_future_aks_usage = 0;
    proximity_config.blen = 0;
    proximity_config.tchthr = 0;
    proximity_config.tchdi = 0;
    proximity_config.average = 0;
    proximity_config.rate = 0;

    if (get_object_address(TOUCH_PROXIMITY_T23, 0) != OBJECT_NOT_FOUND)
    {
        if (write_proximity_config(0, proximity_config) != CFG_WRITE_OK)
        {
            QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
        }
    }
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

void qt_One_Touch_Gesture_Config_Init(void)
{
    /* Disable one touch gestures. */
    onetouch_gesture_config.ctrl = 0;
#if defined(__VER_1_2__)
    onetouch_gesture_config.reserved_1 = 0;
#elif defined(__VER_1_4__)
    onetouch_gesture_config.numgest = 0;
#endif

    onetouch_gesture_config.gesten = 0;
    onetouch_gesture_config.pressproc = 0;
    onetouch_gesture_config.tapto = 0;
    onetouch_gesture_config.flickto = 0;
    onetouch_gesture_config.dragto = 0;
    onetouch_gesture_config.spressto = 0;
    onetouch_gesture_config.lpressto = 0;
    onetouch_gesture_config.reppressto = 0;
    onetouch_gesture_config.flickthr = 0;
    onetouch_gesture_config.dragthr = 0;
    onetouch_gesture_config.tapthr = 0;
    onetouch_gesture_config.throwthr = 0;


    if (get_object_address(PROCI_ONETOUCHGESTUREPROCESSOR_T24, 0) !=
        OBJECT_NOT_FOUND)
    {
        if (write_onetouchgesture_config(0, onetouch_gesture_config) !=
            CFG_WRITE_OK)
        {
            QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
        }
    }
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

void qt_Selftest_Init(void)
{
    selftest_config.ctrl = 3;
    selftest_config.cmd = 0;

#if(NUM_OF_TOUCH_OBJECTS)
    selftest_config.siglim[0].upsiglim = 12000;
    selftest_config.siglim[0].losiglim = 5000;
    selftest_config.siglim[1].upsiglim = 0;
    selftest_config.siglim[1].losiglim = 0;
    selftest_config.siglim[2].upsiglim = 0;
    selftest_config.siglim[2].losiglim = 0;

#endif
    if (get_object_address(SPT_SELFTEST_T25, 0) != OBJECT_NOT_FOUND)
    {
        if (write_selftest_config(0,selftest_config) != CFG_WRITE_OK)
        {
            QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
        }
    }
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

void qt_Two_touch_Gesture_Config_Init(void)
{
    /* Disable two touch gestures. */
    twotouch_gesture_config.ctrl = 0;
#if defined(__VER_1_2__)
    twotouch_gesture_config.reserved1 = 0;
#elif defined(__VER_1_4__)
    twotouch_gesture_config.numgest = 0;
#endif
    twotouch_gesture_config.reserved2 = 0;
    twotouch_gesture_config.gesten = 0;
    twotouch_gesture_config.rotatethr = 0;
    twotouch_gesture_config.zoomthr = 0;


    if (get_object_address(PROCI_TWOTOUCHGESTUREPROCESSOR_T27, 0) !=
        OBJECT_NOT_FOUND)
    {
        if (write_twotouchgesture_config(0, twotouch_gesture_config) !=
            CFG_WRITE_OK)
        {
            QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
        }
    }
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

void qt_CTE_Config_Init(void)
{
    /* Set CTE config */
    cte_config.ctrl = 0;
    cte_config.cmd = 0;
    cte_config.mode = 0;

    // -- End yegw
    cte_config.idlegcafdepth = 8;
    cte_config.actvgcafdepth = 16;
    cte_config.VOLTAGE       = 60; //use avdd voltage as 3.3V

    /* Write CTE config to chip. */
    if (get_object_address(SPT_CTECONFIG_T28, 0) != OBJECT_NOT_FOUND)
    {
        if (write_CTE_config(cte_config) != CFG_WRITE_OK)
        {
            QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
        }
    }
}



/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

unsigned char Comm_Config_Process(unsigned char change_en)
{

    if(change_en == 1)
    {
        change_en = 0;

        if (get_object_address(SPT_COMCONFIG_T18, 0) != OBJECT_NOT_FOUND)
        {
            if((comc_config.cmd == COMM_MODE3))
            {
                if(0)//PIND_Bit2 == 1)
                {
                    comc_config.cmd = COMM_MODE1;
                    return (change_en);
                }

            }
            if (write_comc_config(0, comc_config) != CFG_WRITE_OK)
            {
                return (change_en);
            }
        }
    }
    return (change_en);
}


/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

int init_TouchPad(void)
{

    int return_val = 0;
    uint8_t touch_chip_found = 0;
    uint8_t report_id;
    uint8_t max_report_id;
    uint8_t object_type, instance=0;
    uint32_t crc, stored_crc;
    //uint16_t i;
    uint8_t version=0, family_id=0, variant=0, build=0;
    pr_err("init_TouchPad++\n\r");


    Power(true);//power on I2C devices
    if (init_touch_driver() == DRIVER_SETUP_OK)
    {
        touch_chip_found = 1;
    }

    if (touch_chip_found == 0)
    {
        QT_printf("Connect / info block read failed!\n\r");
        return_val = -ENODEV;
    }
    else
    {    /* only continue here if chip is identified */

        /* Get and show the version information. */
        get_family_id(&family_id);
        get_variant_id(&variant);
        get_version(&version);
        get_build_number(&build);


        QT_printf("Version:        0x%x\n\r", version);
        QT_printf("Family:         0x%x\n\r", family_id);
        QT_printf("Variant:        0x%x\n\r", variant);
        QT_printf("Build number:   0x%x\n\r", build);

        QT_printf("Matrix X size : %d\n\r", info_block->info_id.matrix_x_size);
        QT_printf("Matrix Y size : %d\n\r", info_block->info_id.matrix_y_size);

        if(calculate_infoblock_crc(&crc) != CRC_CALCULATION_OK)
        {
            QT_printf("Calculating CRC failed, skipping check!\n\r");
        }
        else
        {
            QT_printf("Calculated CRC:\t");
            //write_message_to_usart((uint8_t *) &crc, 4);
            QT_printf("\n\r");
        }


        stored_crc = get_stored_infoblock_crc();
        QT_printf("Stored CRC:\t");
        //write_message_to_usart((uint8_t *) &stored_crc, 4);
        QT_printf("\n\r");


        if (stored_crc != crc)
        {
            QT_printf("Warning: info block CRC value doesn't match the calculated!\n\r");
        }
        else
        {
            QT_printf("Info block CRC value OK.\n\n\r");

        }

        /* Test the report id to object type / instance mapping: get the maximum
        * report id and print the report id map. */

        QT_printf("Report ID to Object type / instance mapping:\n\r");

        max_report_id = get_max_report_id();
        for (report_id = 1; report_id <= max_report_id; report_id++)
        {
            object_type = report_id_to_type(report_id, &instance);
            QT_printf("[TSP] Report ID : %d, Object Type : T%d, Instance : %d\n\r",report_id ,object_type,instance);
        }




#ifdef OPTION_WRITE_CONFIG

        qt_Power_Config_Init();

        qt_Acquisition_Config_Init();

        qt_Multitouchscreen_Init();

        qt_KeyArray_Init();

        qt_ComcConfig_Init();

        //qt_Gpio_Pwm_Init(); no

        //qt_Grip_Face_Suppression_Config_Init(); no

        qt_Noise_Suppression_Config_Init();

        //qt_Proximity_Config_Init();

        qt_One_Touch_Gesture_Config_Init();

        qt_Selftest_Init();

        qt_Two_touch_Gesture_Config_Init();

        qt_CTE_Config_Init();

        /* Backup settings to NVM. */
        if (backup_config() != WRITE_MEM_OK)
        {
            QT_printf("Failed to backup, exiting...\n\r");
            return_val = -ENODEV;
        }
        else
        {
            //QT_printf("Backed up the config to non-volatile memory!\n\r");
        }

#else
        QT_printf("Chip setup sequence was bypassed!\n\r");
#endif /* OPTION_WRITE_CONFIG */

        /* Calibrate the touch IC. */
        if (calibrate_chip() != WRITE_MEM_OK)
        {
            QT_printf("Failed to calibrate, exiting...\n\r");
            return_val = -ENODEV;
        }
        else
        {
            //QT_printf("Chip calibrated!\n\r");

        }

        QT_printf("\nWaiting for touch chip messages...\n\n\r");
    }

    return return_val;
}


#ifdef CONFIG_PM
static int touchscreen_suspend(struct i2c_client *client,
                pm_message_t state)
{
    // --yegw 2010-5-25 enter deep sleep
    #if 0
    gen_powerconfig_t7_config_t cfg;
    /* Set Idle Acquisition Interval to 0 for deep sleep. */
    cfg.idleacqint = 0;
    /* Set Active Acquisition Interval to 16 ms. */
    cfg.actvacqint = 16;
    /* Set Active to Idle Timeout to 4 s (one unit = 200ms). */
    cfg.actv2idleto = 20;

    /* Write power config to chip. */
    if (write_power_config(cfg) != CFG_WRITE_OK)
    {
        QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
    }
    #endif
    Power(false);//power off I2C devices
    // -- yege end
    return 0;
}

static int touchscreen_resume(struct i2c_client *client)
{
    Power(true);//power on I2C devices
    #if 0
    // --yegw 2010-5-25 back from deep sleep
    // reconfig the power object apart form deep sleep
    qt_Power_Config_Init();
    // -- yegw end

    /* Calibrate the touch IC. */
    if (calibrate_chip() != WRITE_MEM_OK)
    {
        QT_printf("Failed to calibrate, exiting...\n\r");
    }
    else
    {
        //QT_printf("Chip calibrated!\n\r");
    }
    #endif
// HP Ye Gewang, we use chip backup function to reduce resume time
#if 0
    int rc,i,j;

    rc = init_TouchPad();
    if (rc) {
        dev_err(&client->dev,
               "touchscreen_dev_open FAILED: touchscreen_enable rc=%d\n", rc);
        return 0;
    }

    // Initialization
    for(i=0; i<2; i++)
    {
        for(j=0; j<2; j++)
            position[i][j] = INVALID_POS;
        Index[i] = 0;
    }

    // Clean invalue message
    read_mem(message_processor_address, max_message_length, tmsg);
#endif

    return 0;
}
#endif

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

static int touchscreen_enable(void)
{
    return init_TouchPad();
}

static void ts_update_pen_state(struct touchscreen_data *dd, int x, int y, int pressure, int finger)
{
    int  finger2_pressed = 0;
    if( ( (finger == 2)&&(position[1][0] != INVALID_POS) )||( (finger != 2)&&(position[0][0] != INVALID_POS) ) )
        finger2_pressed = 1;

    if(!finger2_pressed)
    {
        Index[0] = (finger == 2)?0:1;
        Index[1] = (finger == 2)?1:0;
    }

    if (pressure) {
        input_report_abs(dd->touchscreen_dev, ABS_X,  position[Index[0]][0]);
        input_report_abs(dd->touchscreen_dev, ABS_Y,  position[Index[0]][1]);
    }
    pr_dbg("1st point X=%d, Y=%d, press=%d \n\r",position[Index[0]][0],position[Index[0]][1],pressure);

    input_report_abs(dd->touchscreen_dev, ABS_PRESSURE, pressure);
    input_report_abs(dd->touchscreen_dev, ABS_TOOL_WIDTH, 5);
    input_report_key(dd->touchscreen_dev, BTN_TOUCH, !!pressure);
    input_report_key(dd->touchscreen_dev, BTN_2, finger2_pressed);
    if (finger2_pressed) {
        input_report_abs(dd->touchscreen_dev, ABS_HAT0X, position[Index[1]][0]);
        input_report_abs(dd->touchscreen_dev, ABS_HAT0Y, position[Index[1]][1]);
    }

    input_report_abs(dd->touchscreen_dev, ABS_MT_TOUCH_MAJOR, pressure);
    input_report_abs(dd->touchscreen_dev, ABS_MT_WIDTH_MAJOR, 5);
    input_report_abs(dd->touchscreen_dev, ABS_MT_POSITION_X, position[Index[0]][0]);
    input_report_abs(dd->touchscreen_dev, ABS_MT_POSITION_Y, position[Index[0]][1]);
    input_mt_sync(dd->touchscreen_dev);
    if (finger2_pressed) {
        input_report_abs(dd->touchscreen_dev, ABS_MT_TOUCH_MAJOR, 255);
        input_report_abs(dd->touchscreen_dev, ABS_MT_WIDTH_MAJOR, 5);
        input_report_abs(dd->touchscreen_dev, ABS_MT_POSITION_X, position[Index[1]][0]);
        input_report_abs(dd->touchscreen_dev, ABS_MT_POSITION_Y, position[Index[1]][1]);
        pr_dbg("2nd finger %d, %d, \n\r",position[Index[1]][0],position[Index[1]][1]);

        input_mt_sync(dd->touchscreen_dev);
    }

    input_sync(dd->touchscreen_dev);
}


/* Touchscreen work function. Reads data from touchscreen via i2c. */
static void touchscreen_work_f(struct work_struct *work)
{
    struct touchscreen_data   *dd = tp_data;
    u32                     x_val = 0, y_val = 0;
    int i,j;
    u32 retry = 0;

    // -- yegw 2010-5-12 retry 5 time to read message when i2c read failed
    while(read_mem(message_processor_address, max_message_length, tmsg) != READ_MEM_OK)
    {
        retry++;
        if(retry < 5)
        {
            mdelay(30);
            continue;
        }
        pr_err("touchscreen i2c failed\n\r");
        goto tp_wk_exit;
    }
    // -- End yegw 2010-5-12

    if(( tmsg[0] < 2) || ( tmsg[0] >= 12))
    {
        pr_err("touchscreen no finger %d\n\r",tmsg[0]);
        if((tmsg[0] > get_max_report_id())&&(tmsg[0] != 255))
        {
            reset_chip();
            mdelay(30);

            // Initialization
            for(i=0; i<2; i++)
            {
                for(j=0; j<2; j++)
                    position[i][j] = INVALID_POS;
                Index[i] = 0;
            }

            // Clean invalue message
            for(i = 0; i < 30; i++)
            {
                pr_dbg("gpio (%d),vlaue (%d)+\n",dd->touchscreen_gpio,gpio_get_value(dd->touchscreen_gpio));
                if(gpio_get_value(dd->touchscreen_gpio) == 1)
                {
                    break;
                }
                read_mem(message_processor_address, max_message_length, tmsg);
                mdelay(1);
            }

        }
        goto tp_wk_exit;
    }


    x_val = ((tmsg[2])*4)+((tmsg[4] & 0xC0)>>6);
    y_val = ((tmsg[3])*4)+((tmsg[4] & 0x0C)>>2);

    pr_dbg("ts_update_pen_state: msg: %02x %02x %02x %02x %02x %02x %02x %02x\n\r", tmsg[0], tmsg[1],tmsg[2], tmsg[3],tmsg[4], tmsg[5],tmsg[6], tmsg[7]);
    if((tmsg[1] & (1<<5))!=0)
    {
        if(tmsg[0]==2)
        {
             position[0][0] = x_val;
             position[0][1] = y_val;
        }
        else //if(tmsg[0]==3)
        {
            position[1][0] = x_val;
            position[1][1] = y_val;
        }

        ts_update_pen_state(dd, x_val, y_val, 0, tmsg[0]);

        position[0][0] = INVALID_POS;
        position[0][1] = INVALID_POS;

        position[1][0] = INVALID_POS;
        position[1][1] = INVALID_POS;

    }
    else
    {
        //pr_err("ts_update_pen_state X:%d, Y:%d, finger(%d),1\n\r",x_val,y_val,tmsg[0]);

        if(tmsg[0]==2)
        {
             position[0][0] = x_val;
             position[0][1] = y_val;
        }
        else //if(tmsg[0]==3)
        {
            position[1][0] = x_val;
            position[1][1] = y_val;
        }
        ts_update_pen_state(dd, x_val, y_val, 255, tmsg[0]);

    }

tp_wk_exit:
    enable_irq(MSM_GPIO_TO_INT(dd->touchscreen_gpio));
    return;
}

static int touchscreen_dev_open(struct input_dev *dev)
{
    int rc;
    int i,j;
    struct touchscreen_data *dd = input_get_drvdata(dev);

    pr_err("touchscreen_dev_open++\n\r");
    if (!dd->clientp) {
        printk(KERN_ERR "touchscreen_dev_open: no i2c adapter present\n");
        rc = -ENODEV;
        goto fail_irq;
    }


    /* configure touchscreen irq gpio */
    rc = gpio_tlmm_config(GPIO_CFG(dd->touchscreen_gpio, 0, GPIO_CFG_INPUT,
                GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 0);
    if (rc) {
        pr_err("%s: unable to configure gpio %d\n",
            __func__, dd->touchscreen_gpio);
        goto fail_irq;
    }

    rc = gpio_request(dd->touchscreen_gpio, "xMT1386_irq_gpio");
    if (rc) {
        pr_err("%s: unable to request gpio %d\n",
            __func__, dd->touchscreen_gpio);
        goto fail_irq;
    }
    rc = gpio_direction_input(dd->touchscreen_gpio);
    if (rc) {
        dev_err(&dd->clientp->dev,
               "touchscreen_dev_open FAILED: gpio_direction_input(%d) "
               "rc=%d\n", dd->touchscreen_gpio, rc);
        goto fail_irq;
    }


    // Initialization
    for(i=0; i<2; i++)
    {
        for(j=0; j<2; j++)
            position[i][j] = INVALID_POS;
        Index[i] = 0;
    }

    // Clean invalue message
    for(i = 0; i < 30; i++)
    {
        pr_err("gpio (%d),vlaue (%d)+\n",dd->touchscreen_gpio,gpio_get_value(dd->touchscreen_gpio));
        read_mem(message_processor_address, max_message_length, tmsg);
        mdelay(1);
        if(gpio_get_value(dd->touchscreen_gpio) == 1)
        {
            break;
        }
    }


    rc = request_irq(MSM_GPIO_TO_INT(dd->touchscreen_gpio),
               &touchscreen_interrupt,
               IRQF_TRIGGER_LOW,
               "Touchscreen",
               0);
    if (rc) {
        dev_err(&dd->clientp->dev,
               "touchscreen_dev_open FAILED: request_irq rc=%d\n", rc);
        goto fail_irq;
    }

    pr_err("touchscreen_dev_open--\n\r");

    return 0;
fail_irq:
    return rc;
}

static void touchscreen_dev_close(struct input_dev *dev)
{
    struct touchscreen_data *dd = input_get_drvdata(dev);
    free_irq(dd->clientp->irq, NULL);

    gpio_free(dd->touchscreen_gpio);
    close_touch_driver();

}

static irqreturn_t touchscreen_interrupt(int irq, void *dev_id)
{
    disable_irq_nosync(irq);
    schedule_work(&touchscreen_work);
    return IRQ_HANDLED;
}

static int __exit touchscreen_remove(struct i2c_client *client)
{
    struct touchscreen_data              *dd;
    struct msm_touchpad_platform_data *pd;

    dd = i2c_get_clientdata(client);
    pd = client->dev.platform_data;
    input_unregister_device(dd->touchscreen_dev);
    if (pd->gpio_shutdown != NULL)
        pd->gpio_shutdown();
    kfree(dd);
    i2c_set_clientdata(client, NULL);
    tp_data = NULL;

    return 0;
}

static const struct i2c_device_id touchscreen_id[] = {
    { "mXT1386", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, touchscreen_id);

static struct i2c_driver touchscreen_driver = {
    .driver = {
        .name   = "msm_touchscreen",
        .owner  = THIS_MODULE,
    },
    .probe    = touchscreen_probe,
    .remove   =  __exit_p(touchscreen_remove),
#ifdef CONFIG_PM
    .suspend  = touchscreen_suspend,
    .resume   = touchscreen_resume,
#endif
    .id_table = touchscreen_id,
};

static int touchscreen_probe(struct i2c_client *client,
              const struct i2c_device_id *id)
{
    int                                rc = EFAULT;
    struct msm_touchpad_platform_data *pd;
    struct touchscreen_data              *dd;

    printk(KERN_ERR "touchscreen_probe\n");

    if (tp_data) {
        dev_err(&client->dev,
            "touchscreen_probe: only a single touchscreen supported\n");
        rc = -EPERM;
        goto probe_exit;
    }

    dd = kzalloc(sizeof *dd, GFP_KERNEL);
    if (!dd) {
        rc = -ENOMEM;
        goto probe_exit;
    }
    tp_data = dd;
    i2c_set_clientdata(client, dd);

    dd->clientp = client;
    client->driver = &touchscreen_driver;
    pd = client->dev.platform_data;
    if (!pd) {
        dev_err(&client->dev,
            "touchscreen_probe: platform data not set\n");
        rc = -EFAULT;
        goto probe_free_exit;
    }

    dd->touchscreen_gpio = pd->gpioirq;

    if (pd->gpio_setup != NULL)
    {
        rc = pd->gpio_setup();
        if (rc)
            goto probe_free_exit;
    }

    dd->touchscreen_dev = input_allocate_device();
    if (!dd->touchscreen_dev) {
        rc = -ENOMEM;
        goto probe_gpiosus_exit;
    }
    input_set_drvdata(dd->touchscreen_dev, dd);
    dd->touchscreen_dev->open       = touchscreen_dev_open;
    dd->touchscreen_dev->close      = touchscreen_dev_close;
    dd->touchscreen_dev->name       = TOUCHSCREEN_NAME;
    dd->touchscreen_dev->phys       = TOUCHSCREEN_DEVICE;
    dd->touchscreen_dev->id.bustype = BUS_I2C;
    dd->touchscreen_dev->id.vendor  = QUALCOMM_VENDORID;
    dd->touchscreen_dev->id.product = 1;
    dd->touchscreen_dev->id.version = 2;

    set_bit(EV_SYN, dd->touchscreen_dev->evbit);
    set_bit(EV_KEY, dd->touchscreen_dev->evbit);
    set_bit(BTN_TOUCH,dd->touchscreen_dev->keybit);
    set_bit(BTN_2, dd->touchscreen_dev->keybit);
    set_bit(EV_ABS,dd->touchscreen_dev->evbit);

    input_set_abs_params(dd->touchscreen_dev, ABS_X, 0, X_MAX, 0, 0);
    input_set_abs_params(dd->touchscreen_dev, ABS_Y, 0, Y_MAX, 0, 0);
    input_set_abs_params(dd->touchscreen_dev, ABS_PRESSURE, 0, PRESS_MAX, 0, 0);
    input_set_abs_params(dd->touchscreen_dev, ABS_HAT0X, 0, X_MAX, 0, 0);
    input_set_abs_params(dd->touchscreen_dev, ABS_HAT0Y, 0, Y_MAX, 0, 0);
    input_set_abs_params(dd->touchscreen_dev, ABS_MT_POSITION_X, 0, X_MAX, 0, 0);
    input_set_abs_params(dd->touchscreen_dev, ABS_MT_POSITION_Y, 0, Y_MAX, 0, 0);
    input_set_abs_params(dd->touchscreen_dev, ABS_MT_TOUCH_MAJOR, 0, 256, 0, 0);
    input_set_abs_params(dd->touchscreen_dev, ABS_TOOL_WIDTH, 0, 15, 0, 0);

    rc = touchscreen_enable();
    if (rc) {
        pr_err("%s: touch screen enable failed %d\n",
            __func__, rc);
        rc = -ENODEV;
        goto probe_fail_reg_dev;
    }

    rc = input_register_device(dd->touchscreen_dev);
    if (rc) {
        dev_err(&client->dev,
            "touchscreen_probe: input_register_device rc=%d\n",
               rc);
        goto probe_fail_reg_dev;
    }

    return 0;

probe_fail_reg_dev:
    input_free_device(dd->touchscreen_dev);
probe_gpiosus_exit:
    if (pd->gpio_shutdown != NULL)
        pd->gpio_shutdown();
probe_free_exit:
    kfree(dd);
    tp_data = NULL;
    i2c_set_clientdata(client, NULL);
probe_exit:
    return rc;
}

static void  Power(bool on)
{

    struct regulator *votg_l10;
    struct regulator *votg_vdd5v;//To guarantee Touch panel power turn on before I2C operations. 1005 Bob Zhu

    /* VDD_BACKLIGHT_5.0V*/
    votg_vdd5v = regulator_get(NULL, "vdd50_boost");
    if (IS_ERR(votg_vdd5v))
    {
        pr_err("%s:Touch unable to get vot_vdd5v\n", __func__);
        return;
    }

    votg_l10 = regulator_get(NULL, "8058_l10");
    if (IS_ERR(votg_l10))
    {
        pr_err("%s: touch unable to get votg_l10\n", __func__);
        return;
    }

	if(on)
	{
		if(regulator_set_voltage(votg_vdd5v, 5000000, 5000000))
		{
			pr_err("%s:Touch Unable to set regulator voltage:"
					" votg_l10\n", __func__);
		}

		if(regulator_set_voltage(votg_l10, 3050000, 3050000))
		{
			pr_err("%s:Touch Unable to set regulator voltage:"
					" votg_l10\n", __func__);
		}

		/* VDD_BACKLIGHT_5.0V ENABLE*/
         if (regulator_enable(votg_vdd5v))
         {
             pr_err("%s:Touch Unable to enable the regulator:"
                     " votg_vdd5v\n", __func__);
            return;
         }

        /* VDD_LVDS_3.3V ENABLE*/
        if (regulator_enable(votg_l10))
        {
            pr_err("%s:Touch Unable to enable the regulator:"
                    " votg_l10\n", __func__);
           return;
        }
	    /* ensuring the chip is actually up and ready before returning */
    	usleep(50*1000);
    }
    else
    {
        pr_err("%s:Touch disable regulator voltage:"
                    " votg_l10\n", __func__);
        regulator_disable(votg_l10);
        regulator_put(votg_l10);
        regulator_disable(votg_vdd5v);
        regulator_put(votg_vdd5v);
    }
    return;

}

// change for TOPAZ_proto_MB_AUG_18
#define  ts_power_reset_gpio   70

static int __init touchscreen_init(void)
{
    int rc;
    printk(KERN_ERR "touchscreen_init\n");

#if 1
    /* configure touchscreen reset gpio */
    rc = gpio_tlmm_config(GPIO_CFG(ts_power_reset_gpio, 0, GPIO_CFG_OUTPUT,
                GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    if (rc) {
        pr_err("%s: unable to configure gpio %d\n",
            __func__, ts_power_reset_gpio);
        goto init_exit;
    }

    rc = gpio_request(ts_power_reset_gpio, "xMT1386_reset_gpio");
    if (rc) {
        pr_err("%s: unable to request gpio %d\n",
            __func__, ts_power_reset_gpio);
        goto init_exit;
    }


    // power reset
    gpio_direction_output(ts_power_reset_gpio, 0);
    mdelay(30);
    gpio_direction_output(ts_power_reset_gpio, 1);
    mdelay(30);
#endif
    rc = i2c_add_driver(&touchscreen_driver);
    if (rc) {
        printk(KERN_ERR "touchscreen_init FAILED: i2c_add_driver rc=%d\n",
               rc);
        goto init_exit;
    }

    return 0;

init_exit:
    return rc;
}

static void __exit touchscreen_exit(void)
{
    i2c_del_driver(&touchscreen_driver);
}

module_init(touchscreen_init);
module_exit(touchscreen_exit);
