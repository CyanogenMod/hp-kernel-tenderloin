/*
 * Includes
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/wait.h>
#include <linux/bitops.h>
#include <linux/ctype.h>
#include <linux/slab.h>
#include <linux/a6_sbw_interface.h>
#include <linux/a6.h>
#ifdef CONFIG_HIGH_RES_TIMERS
#include <linux/hrtimer.h>
#endif
#include <linux/cpufreq.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <asm/io.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/debugfs.h>
#include <linux/switch.h>
#include <linux/power_supply.h>
#include <mach/msm_hsusb.h>
#include <linux/max8903b_charger.h>

#include "high_level_funcs.h"

void max8903b_set_connected_ps(unsigned connected);

#define A2A_RD_BUFF_SIZE (4 * 1024)
#define A2A_WR_BUFF_SIZE (4 * 1024)

#define A6_DEBUG
#define A6_PQ
//#define A6_I2C_RETRY

#ifdef A6_DEBUG
#define ASSERT(i)  BUG_ON(!(i))

#else
#define ASSERT(i)  ((void)0)

#endif


enum {
	A6_DEBUG_VERBOSE = 0x01,
};

static int a6_debug_mask = 0x0;
static int a6_tp_irq_count = 0;
static int a6_t2s_dup_correct = 0;
static int a6_disable_dock_switch = 0;

module_param_named(
		   disable_dock_switch, a6_disable_dock_switch, int,
		   S_IRUGO | S_IWUSR | S_IWGRP
		  );

module_param_named(
		   debug_mask, a6_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP
		  );

module_param_named(
		   a6_irq_count, a6_tp_irq_count, int, S_IRUGO | S_IWUSR | S_IWGRP
		  );

module_param_named(
		   t2s_dup_correct, a6_t2s_dup_correct, int, S_IRUGO | S_IWUSR | S_IWGRP
		  );

#define A6_DPRINTK(mask, level, message, ...) \
	do { \
		if ((mask) & a6_debug_mask) \
			printk(level message , ##__VA_ARGS__); \
} while (0)

#define PROFILE_USAGE
#if defined PROFILE_USAGE
bool reset_active = false;
uint32_t start_time;
int32_t diff_time;

uint32_t start_last_a6_activity = 0;
#endif

long a6_last_ps_connect = 0;

/* page 0x00 */
/* host interrupts */
#define TS2_I2C_INT_MASK_0                             0x0000
#define TS2_I2C_INT_MASK_1                             0x0001
#define TS2_I2C_INT_MASK_2                             0x0002
#define TS2_I2C_INT_MASK_3                             0x0003
#define TS2_I2C_INT_STATUS_0                           0x0004
#define TS2_I2C_INT_STATUS_1                           0x0005
#define TS2_I2C_INT_STATUS_2                           0x0006
#define TS2_I2C_INT_STATUS_3                           0x0007
#define TS2_I2C_INT_0_GPIOA_7                          0x80
#define TS2_I2C_INT_0_GPIOA_6                          0x40
#define TS2_I2C_INT_0_GPIOA_5                          0x20
#define TS2_I2C_INT_0_GPIOA_4                          0x10
#define TS2_I2C_INT_0_GPIOA_3                          0x08
#define TS2_I2C_INT_0_GPIOA_2                          0x04
#define TS2_I2C_INT_0_GPIOA_1                          0x02
#define TS2_I2C_INT_0_GPIOA_0                          0x01
#define TS2_I2C_INT_0_GPIOA_MASK                       0xff
#define TS2_I2C_INT_1_COMM_RX_FULL                     0x02
#define TS2_I2C_INT_1_COMM_TX_EMPTY                    0x01
#define TS2_I2C_INT_2_BAT_TEMP_HIGH                    0x20
#define TS2_I2C_INT_2_BAT_TEMP_LOW                     0x10
#define TS2_I2C_INT_2_BAT_VOLT_LOW                     0x08
#define TS2_I2C_INT_2_BAT_RARC_CRIT                    0x04
#define TS2_I2C_INT_2_BAT_RARC_LOW2                    0x02
#define TS2_I2C_INT_2_BAT_RARC_LOW1                    0x01
#define TS2_I2C_INT_3_A2A_CONNECT_CHANGE               0x08
#define TS2_I2C_INT_3_FLAGS_CHANGE                     0x04
#define TS2_I2C_INT_3_LOG                              0x02
#define TS2_I2C_INT_3_RESET                            0x01


/* page 0x01 */
/* battery (airboard only); interface defined by phone teams */
#define TS2_I2C_BAT_STATUS                             0x0100
#define TS2_I2C_BAT_RARC                               0x0101
#define TS2_I2C_BAT_RSRC                               0x0102
#define TS2_I2C_BAT_AVG_CUR_MSB                        0x0103
#define TS2_I2C_BAT_AVG_CUR_LSB                        0x0104
#define TS2_I2C_BAT_TEMP_MSB                           0x0105
#define TS2_I2C_BAT_TEMP_LSB                           0x0106
#define TS2_I2C_BAT_VOLT_MSB                           0x0107
#define TS2_I2C_BAT_VOLT_LSB                           0x0108
#define TS2_I2C_BAT_CUR_MSB                            0x0109
#define TS2_I2C_BAT_CUR_LSB                            0x010a
#define TS2_I2C_BAT_COULOMB_MSB                        0x010b
#define TS2_I2C_BAT_COULOMB_LSB                        0x010c
#define TS2_I2C_BAT_AS                                 0x010d
#define TS2_I2C_BAT_FULL_MSB                           0x010e
#define TS2_I2C_BAT_FULL_LSB                           0x010f
#define TS2_I2C_BAT_FULL40_MSB                         0x0110
#define TS2_I2C_BAT_FULL40_LSB                         0x0111
#define TS2_I2C_BAT_RSNSP                              0x0112
#define TS2_I2C_BAT_RAAC_MSB                           0x0113
#define TS2_I2C_BAT_RAAC_LSB                           0x0114
#define TS2_I2C_BAT_SACR_MSB                           0x0115
#define TS2_I2C_BAT_SACR_LSB                           0x0116
#define TS2_I2C_BAT_ASL                                0x0117
#define TS2_I2C_BAT_FAC_MSB                            0x0118
#define TS2_I2C_BAT_FAC_LSB                            0x0119

#define TS2_I2C_BAT_ROMID_0                            0x0120
#define TS2_I2C_BAT_ROMID(x) \
        (TS2_I2C_BAT_ROMID_0 + (x))

#define TS2_I2C_BAT_COMMAND_STATUS                     0x0140
#define TS2_I2C_BAT_COMMAND_AUTH                 	0x81
#define TS2_I2C_BAT_COMMAND_REFRESH              	0x82
#define TS2_I2C_BAT_COMMAND_WAKE                 	0x83
#define TS2_I2C_BAT_COMMAND_OFF                  	0xe9
#define TS2_I2C_BAT_STATUS_AUTH_FAIL             	0x08
#define TS2_I2C_BAT_STATUS_AUTH_PASS             	0x04
#define TS2_I2C_BAT_STATUS_REGS_VALID            	0x02
#define TS2_I2C_BAT_STATUS_BUSY                  	0x01


/* battery configuration (airboard only) */
#define TS2_I2C_BAT_TEMP_LOW_MSB                       0x0180
#define TS2_I2C_BAT_TEMP_LOW_LSB                       0x0181
#define TS2_I2C_BAT_TEMP_HIGH_MSB                      0x0182
#define TS2_I2C_BAT_TEMP_HIGH_LSB                      0x0183
#define TS2_I2C_BAT_VOLT_LOW_MSB                       0x0184
#define TS2_I2C_BAT_VOLT_LOW_LSB                       0x0185
#define TS2_I2C_BAT_RARC_CRIT                          0x0186
#define TS2_I2C_BAT_RARC_LOW_2                         0x0187
#define TS2_I2C_BAT_RARC_LOW_1                         0x0188

#define TS2_I2C_BAT_CHALLENGE_0                        0x01e0
#define TS2_I2C_BAT_CHALLENGE(x) \
        (TS2_I2C_BAT_CHALLENGE_0 + (x))
#define TS2_I2C_BAT_RESPONSE_0 \
        (TS2_I2C_BAT_CHALLENGE_0 + 8)
#define TS2_I2C_BAT_RESPONSE(x) \
        (TS2_I2C_BAT_RESPONSE_0 + (x))


/* page 0x02 */
/* comms */
#define TS2_I2C_COMM_STATUS                            0x0200
#define TS2_I2C_COMM_STATUS_RX_FULL			0x02
#define TS2_I2C_COMM_STATUS_TX_EMPTY			0x01
#define TS2_I2C_COMM_TXDATA_RXDATA_RESERVED_1          0x0201 /* reserved 1 */
#define TS2_I2C_COMM_TXDATA_RXDATA_RESERVED_2          0x0202 /* reserved 2 */
#define TS2_I2C_COMM_TXDATA_RXDATA                     0x0203 /* side-effect */

/* page 0x03 */
/* log */
#define TS2_I2C_LOG_LEVEL                              0x0300
#define TS2_I2C_LOG_INT_THRESHOLD                      0x0301
#define TS2_I2C_LOG_LOST_MSB_RESERVED_1                0x0302 /* reserved 1 */
#define TS2_I2C_LOG_LOST_MSB_RESERVED_2                0x0303 /* reserved 2 */
#define TS2_I2C_LOG_LOST_MSB                           0x0304 /* side-effect */
#define TS2_I2C_LOG_LOST_LSB                           0x0305
#define TS2_I2C_LOG_COUNT_MSB                          0x0306
#define TS2_I2C_LOG_COUNT_LSB                          0x0307
#define TS2_I2C_LOG_ENTRY_MSB_RESERVED_1               0x0308 /* reserved 1 */
#define TS2_I2C_LOG_ENTRY_MSB_RESERVED_2               0x0309 /* reserved 2 */
#define TS2_I2C_LOG_ENTRY_MSB                          0x030a /* side-effect */
#define TS2_I2C_LOG_ENTRY_LSB                          0x030b


/* page 0x04 */
/* local enumeration structure */
/* Enumeration registers (local) */
#define	TS2_I2C_ENUM_STRUCT_VER                        0x0400
#define	TS2_I2C_ENUM_STRUCT_LEN                        0x0401
#define	TS2_I2C_ENUM_PROTOCOL_MAP_VER                  0x0402
#define	TS2_I2C_ENUM_MFGR_ID_HI                        0x0403
#define	TS2_I2C_ENUM_MFGR_ID_LO                        0x0404
#define	TS2_I2C_ENUM_PRODUCT_TYPE_HI                   0x0405
#define	TS2_I2C_ENUM_PRODUCT_TYPE_LO                   0x0406
#define	TS2_I2C_ENUM_SERNO_7                           0x0407
#define	TS2_I2C_ENUM_SERNO_6                           0x0408
#define	TS2_I2C_ENUM_SERNO_5                           0x0409
#define	TS2_I2C_ENUM_SERNO_4                           0x040A
#define	TS2_I2C_ENUM_SERNO_3                           0x040B
#define	TS2_I2C_ENUM_SERNO_2                           0x040C
#define	TS2_I2C_ENUM_SERNO_1                           0x040D
#define	TS2_I2C_ENUM_SERNO_0                           0x040E
#define	TS2_I2C_ENUM_ASSY_REV                          0x040F
#define	TS2_I2C_ENUM_FW_VER_2                          0x0410
#define	TS2_I2C_ENUM_FW_VER_1                          0x0411
#define	TS2_I2C_ENUM_FW_VER_0                          0x0412
#define	TS2_I2C_ENUM_VNODE_MIN_HI                      0x0413
#define	TS2_I2C_ENUM_VNODE_MIN_LO                      0x0414
#define	TS2_I2C_ENUM_VNODE_MAX_HI                      0x0415
#define	TS2_I2C_ENUM_VNODE_MAX_LO                      0x0416
#define	TS2_I2C_ENUM_INODE_MIN_HI                      0x0417
#define	TS2_I2C_ENUM_INODE_MIN_LO                      0x0418
#define	TS2_I2C_ENUM_INODE_MAX_HI                      0x0419
#define	TS2_I2C_ENUM_INODE_MAX_LO                      0x041A
#define	TS2_I2C_ENUM_TNODE_MIN                         0x041B
#define	TS2_I2C_ENUM_TNODE_MAX                         0x041C
#define	TS2_I2C_ENUM_POWER_MAX                         0x041D

#define	TS2_I2C_ENUM_ACCE_0                            0x0428
#define	TS2_I2C_ENUM_ACCE_1                            0x0429
#define	TS2_I2C_ENUM_ACCE_2                            0x042A
#define	TS2_I2C_ENUM_ACCE_3                            0x042B
#define	TS2_I2C_ENUM_ACCE_4                            0x042C
#define	TS2_I2C_ENUM_ACCE_5                            0x042D
#define	TS2_I2C_ENUM_ACCE_6                            0x042E
#define	TS2_I2C_ENUM_ACCE_7                            0x042F
#define	TS2_I2C_ENUM_ACCE_8                            0x0430
#define	TS2_I2C_ENUM_ACCE_9                            0x0431
#define	TS2_I2C_ENUM_ACCE_10                           0x0432
#define	TS2_I2C_ENUM_ACCE_11                           0x0433
#define	TS2_I2C_ENUM_ACCE_12                           0x0434
#define	TS2_I2C_ENUM_ACCE_13                           0x0435
#define	TS2_I2C_ENUM_ACCE_14                           0x0436
#define	TS2_I2C_ENUM_ACCE_15                           0x0437
#define	TS2_I2C_ENUM_MIN_PWM                           0x0438


/* page 0x05 */
/* remote enumeration structure */
#define	TS2_I2C_ENUM_REMOTE_STRUCT_VER                 0x0500
#define	TS2_I2C_ENUM_REMOTE_STRUCT_LEN                 0x0501
#define	TS2_I2C_ENUM_REMOTE_PROTOCOL_MAP_VER           0x0502
#define	TS2_I2C_ENUM_REMOTE_MFGR_ID_HI                 0x0503
#define	TS2_I2C_ENUM_REMOTE_MFGR_ID_LO                 0x0504
#define	TS2_I2C_ENUM_REMOTE_MFGR_ID_HI_V1              0x0505
#define	TS2_I2C_ENUM_REMOTE_MFGR_ID_LO_V1              0x0506
#define	TS2_I2C_ENUM_REMOTE_PRODUCT_TYPE_HI            0x0505
#define	TS2_I2C_ENUM_REMOTE_PRODUCT_TYPE_LO            0x0506
#define	TS2_I2C_ENUM_REMOTE_PRODUCT_TYPE_HI_V1         0x0507
#define	TS2_I2C_ENUM_REMOTE_PRODUCT_TYPE_LO_V1         0x0508
#define	TS2_I2C_ENUM_REMOTE_SERNO_7                    0x0507
#define	TS2_I2C_ENUM_REMOTE_SERNO_6                    0x0508
#define	TS2_I2C_ENUM_REMOTE_SERNO_5                    0x0509
#define	TS2_I2C_ENUM_REMOTE_SERNO_4                    0x050A
#define	TS2_I2C_ENUM_REMOTE_SERNO_3                    0x050B
#define	TS2_I2C_ENUM_REMOTE_SERNO_2                    0x050C
#define	TS2_I2C_ENUM_REMOTE_SERNO_1                    0x050D
#define	TS2_I2C_ENUM_REMOTE_SERNO_0                    0x050E
#define	TS2_I2C_ENUM_REMOTE_ASSY_REV                   0x050F
#define	TS2_I2C_ENUM_REMOTE_FW_VER_2                   0x0510
#define	TS2_I2C_ENUM_REMOTE_FW_VER_1                   0x0511
#define	TS2_I2C_ENUM_REMOTE_FW_VER_0                   0x0512
#define	TS2_I2C_ENUM_REMOTE_VNODE_MIN_HI               0x0513
#define	TS2_I2C_ENUM_REMOTE_VNODE_MIN_LO               0x0514
#define	TS2_I2C_ENUM_REMOTE_VNODE_MAX_HI               0x0515
#define	TS2_I2C_ENUM_REMOTE_VNODE_MAX_LO               0x0516
#define	TS2_I2C_ENUM_REMOTE_INODE_MIN_HI               0x0517
#define	TS2_I2C_ENUM_REMOTE_INODE_MIN_LO               0x0518
#define	TS2_I2C_ENUM_REMOTE_INODE_MAX_HI               0x0519
#define	TS2_I2C_ENUM_REMOTE_INODE_MAX_LO               0x051A
#define	TS2_I2C_ENUM_REMOTE_TNODE_MIN                  0x051B
#define	TS2_I2C_ENUM_REMOTE_TNODE_MAX                  0x051C
#define	TS2_I2C_ENUM_REMOTE_POWER_MAX                  0x051D



#define	TS2_I2C_ENUM_REMOTE_ACCE_0                     0x0528
#define	TS2_I2C_ENUM_REMOTE_ACCE_1                     0x0529
#define	TS2_I2C_ENUM_REMOTE_ACCE_2                     0x052A
#define	TS2_I2C_ENUM_REMOTE_ACCE_3                     0x052B
#define	TS2_I2C_ENUM_REMOTE_ACCE_4                     0x052C
#define	TS2_I2C_ENUM_REMOTE_ACCE_5                     0x052D
#define	TS2_I2C_ENUM_REMOTE_ACCE_6                     0x052E
#define	TS2_I2C_ENUM_REMOTE_ACCE_7                     0x052F
#define	TS2_I2C_ENUM_REMOTE_ACCE_8                     0x0530
#define	TS2_I2C_ENUM_REMOTE_ACCE_9                     0x0531
#define	TS2_I2C_ENUM_REMOTE_ACCE_10                    0x0532
#define	TS2_I2C_ENUM_REMOTE_ACCE_11                    0x0533
#define	TS2_I2C_ENUM_REMOTE_ACCE_12                    0x0534
#define	TS2_I2C_ENUM_REMOTE_ACCE_13                    0x0535
#define	TS2_I2C_ENUM_REMOTE_ACCE_14                    0x0536
#define	TS2_I2C_ENUM_REMOTE_ACCE_15                    0x0537


/* page 0x06 */
/* reserved */


/* page 0x07 */
/* voltage, current, temperature, power (puck only), pwm (puck only) */
#define TS2_I2C_ID                                     0x0700
#define TS2_I2C_ID_AIRBOARD_0				0x2c
#define TS2_I2C_FLAGS_0                                0x0701
#define	TS2_I2C_FLAGS_0_PUCK_PRIORITY			(1<<0)
#define TS2_I2C_FLAGS_1                                0x0702
#define TS2_I2C_FLAGS_2                                0x0703
#define	TS2_I2C_FLAGS_2_A2A_CONNECT			0x80
#define	TS2_I2C_FLAGS_2_A2A_ATTEMPT			0x40
#define	TS2_I2C_FLAGS_2_BATTERY_DETECT			0x20
#define	TS2_I2C_FLAGS_2_WIRED_CHARGE			0x10
#define	TS2_I2C_FLAGS_2_PUCK_CHARGE			0x08
#define TS2_I2C_FLAGS_2_WIRED				0x04
#define TS2_I2C_FLAGS_2_PUCK				0x02
#define TS2_I2C_FLAGS_2_PUCK_DETECT			0x01
#define TS2_I2C_V_LOCAL                                0x0704
#define TS2_I2C_I_LOCAL                                0x0705
#define TS2_I2C_T_LOCAL                                0x0706
#define TS2_I2C_P_LOCAL                                0x0707 /* puck only */
#define TS2_I2C_V_REMOTE                               0x0708
#define TS2_I2C_I_REMOTE                               0x0709
#define TS2_I2C_T_REMOTE                               0x070a
#define TS2_I2C_P_REMOTE                               0x070b /* puck only */
#define TS2_I2C_PWM                                    0x070c /* puck only */
#define TS2_I2C_PERIOD                                 0x070d /* puck only */

/* config */
#define TS2_I2C_TX_POWER_V1                            0x0740
#define TS2_I2C_TX_POWER_V2                            0x0741

/* puck only config */
#define TS2_I2C_PWM_PERIOD                             0x0780
#define TS2_I2C_PWM_DUTY_ENUM                          0x0781
#define TS2_I2C_PWM_DUTY_MIN_RUN                       0x0782
#define TS2_I2C_PWM_DUTY_MIN_BOOST                     0x0783
#define TS2_I2C_PWM_DEADTIME                           0x0784
#define TS2_I2C_PWM_EFFICIENCY_OFFSET                  0x0785
#define TS2_I2C_PWM_EFFICIENCY_SCALE                   0x0786

/* airboard only */
#define TS2_I2C_V_OFFSET                               0x07c0
#define TS2_I2C_WAKEUP_PERIOD                          0x07c1
#define TS2_I2C_CURRENT_ADJ_FOR_V1_SLOPE               0x07c2
#define TS2_I2C_CURRENT_ADJ_FOR_V1_OFFSET              0x07c3
#define TS2_I2C_VOLTAGE_ADJ_FOR_V1_SLOPE               0x07c4
#define TS2_I2C_VOLTAGE_ADJ_FOR_V1_OFFSET              0x07c5

/* misc registers */
#define TS2_I2C_COMMAND                                0x1000
#define TS2_I2C_COMMAND_RESET_HOST               	 0x01
#define TS2_I2C_COMMAND_CLEAR_BTN_SEQ_RESET_FLAG 	 0x02

#define TS2_I2C_COMMAND_COMM_CONNECT             	 0x10
#define TS2_I2C_COMMAND_COMM_DISCONNECT          	 0x11
#define TS2_I2C_COMMAND_REENUMERATE              	 0x12

#define TS2_I2C_COMMAND_FRAM_CHECKSUM_1400_READ  	 0x20
#define TS2_I2C_COMMAND_FRAM_CHECKSUM_READ_1     	 0x21
#define TS2_I2C_COMMAND_FRAM_CHECKSUM_READ_2     	 0x22

#define TS2_I2C_COMMAND_MEM_READ_BYTE            	 0x80
#define TS2_I2C_COMMAND_MEM_READ_CHAWMP          	 0x81
#define TS2_I2C_COMMAND_MEM_WRITE_BYTE           	 0x82
#define TS2_I2C_COMMAND_MEM_WRITE_CHAWMP         	 0x83
#define TS2_I2C_COMMAND_PMIC_READ                	 0x84
#define TS2_I2C_COMMAND_PMIC_WRITE               	 0x85
#define TS2_I2C_COMMAND_SP_READ                  	 0x86
#define TS2_I2C_COMMAND_SP_INDIRECT_READ         	 0x87
#define TS2_I2C_COMMAND_ADDR_MSB                       0x1001
#define TS2_I2C_COMMAND_ADDR_LSB                       0x1002
#define TS2_I2C_COMMAND_DATA_MSB                       0x1003
#define TS2_I2C_COMMAND_DATA_LSB                       0x1004

#define TS2_I2C_DEBUG_TRACE_DATA_RESERVED_1            0x101e /* reserved 1 */
#define TS2_I2C_DEBUG_TRACE_DATA_RESERVED_2            0x101f /* reserved 2 */
#define TS2_I2C_DEBUG_TRACE_DATA                       0x1020 /* side-effect */

#define A6_REG_TS2_I2C_INT_MASK_0	0
#define A6_REG_TS2_I2C_INT_MASK_1	1
#define A6_REG_TS2_I2C_INT_MASK_2	2
#define A6_REG_TS2_I2C_INT_MASK_3	3
#define A6_REG_TS2_I2C_INT_STATUS_0	4
#define A6_REG_TS2_I2C_INT_STATUS_1	5
#define A6_REG_TS2_I2C_INT_STATUS_2	6
#define A6_REG_TS2_I2C_INT_STATUS_3	7
#define A6_REG_TS2_I2C_BAT_STATUS	8
#define A6_REG_TS2_I2C_BAT_RARC	9
#define A6_REG_TS2_I2C_BAT_RSRC	10
#define A6_REG_TS2_I2C_BAT_AVG_CUR_LSB_MSB	11
#define A6_REG_TS2_I2C_BAT_TEMP_LSB_MSB	12
#define A6_REG_TS2_I2C_BAT_VOLT_LSB_MSB	13
#define A6_REG_TS2_I2C_BAT_CUR_LSB_MSB	14
#define A6_REG_TS2_I2C_BAT_COULOMB_LSB_MSB	15
// #define A6_REG_TS2_I2C_BAT_AS	16 // two entries? TODO
#define A6_REG_TS2_I2C_BAT_FULL_LSB_MSB	17
#define A6_REG_TS2_I2C_BAT_FULL40_LSB_MSB	18
#define A6_REG_TS2_I2C_BAT_RSNSP	19
#define A6_REG_TS2_I2C_BAT_ROMID_0	20
#define A6_REG_TS2_I2C_BAT_COMMAND_STATUS	21
#define A6_REG_TS2_I2C_BAT_TEMP_LOW_LSB_MSB	22
#define A6_REG_TS2_I2C_BAT_TEMP_HIGH_LSB_MSB	23
#define A6_REG_TS2_I2C_BAT_VOLT_LOW_LSB_MSB	24
#define A6_REG_TS2_I2C_BAT_RARC_CRIT	25
#define A6_REG_TS2_I2C_BAT_RARC_LOW_2	26
#define A6_REG_TS2_I2C_BAT_RARC_LOW_1	27
#define A6_REG_TS2_I2C_BAT_RAAC_MSB	28
#define A6_REG_TS2_I2C_ID	29
#define A6_REG_TS2_I2C_FLAGS_0	30
#define A6_REG_TS2_I2C_FLAGS_2	31
#define A6_REG_VERSION	32
#define A6_REG_TS2_I2C_V_OFFSET	33
#define A6_REG_TS2_I2C_WAKEUP_PERIOD	34
#define A6_REG_TS2_I2C_COMMAND	35
#define A6_REG_REMOTE_VERSION	36
#define A6_REG_ACCESSORY_DATA_0	37
#define A6_REG_ACCESSORY_DATA_1	38
#define A6_REG_ACCESSORY_DATA_2	39
#define A6_REG_ACCESSORY_DATA_3	40
#define A6_REG_ACCESSORY_DATA_4	41
#define A6_REG_ACCESSORY_DATA_5	42
#define A6_REG_ACCESSORY_DATA_6	43
#define A6_REG_ACCESSORY_DATA_7	44
#define A6_REG_ACCESSORY_DATA_8	45
#define A6_REG_ACCESSORY_DATA_9	46
#define A6_REG_ACCESSORY_DATA_10	47
#define A6_REG_ACCESSORY_DATA_11	48
#define A6_REG_ACCESSORY_DATA_12	49
#define A6_REG_REMOTE_ACCESSORY_DATA_0	50
#define A6_REG_REMOTE_ACCESSORY_DATA_1	51
#define A6_REG_REMOTE_ACCESSORY_DATA_2	52
#define A6_REG_REMOTE_ACCESSORY_DATA_3	53
#define A6_REG_REMOTE_ACCESSORY_DATA_4	54
#define A6_REG_REMOTE_ACCESSORY_DATA_5	55
#define A6_REG_REMOTE_ACCESSORY_DATA_6	56
#define A6_REG_REMOTE_ACCESSORY_DATA_7	57
#define A6_REG_REMOTE_ACCESSORY_DATA_8	58
#define A6_REG_REMOTE_ACCESSORY_DATA_9	59
#define A6_REG_REMOTE_ACCESSORY_DATA_10	60
#define A6_REG_REMOTE_ACCESSORY_DATA_11	61
#define A6_REG_REMOTE_ACCESSORY_DATA_12	62
#define A6_REG_TS2_I2C_COMM_STATUS	63
#define A6_REG_TS2_I2C_COMM_TXDATA_RXDATA	64
#define A6_REG_TS2_I2C_BAT_SACR_LSB_MSB	65
#define A6_REG_TS2_I2C_BAT_ASL	66
// #define A6_REG_TS2_I2C_BAT_AS	67 // two entries? TODO
#define A6_REG_TS2_I2C_BAT_FAC_LSB_MSB	68
#define A6_REG_MAX_POWER_AVAILABLE	69
#define A6_REG_TS2_I2C_ENUM_REMOTE_STRUCT_VER	70
#define A6_REG_ACCESSORY_DATA_13	71
#define A6_REG_ACCESSORY_DATA_14	72
#define A6_REG_ACCESSORY_DATA_15	73
#define A6_REG_REMOTE_ACCESSORY_DATA_13	74
#define A6_REG_REMOTE_ACCESSORY_DATA_14	75
#define A6_REG_REMOTE_ACCESSORY_DATA_15	76
#define A6_REG_TS2_I2C_ENUM_MIN_PWM	77
#define A6_REG_ACCESSORY_DATA_COMBO	78
#define A6_REG_REMOTE_ACCESSORY_DATA_COMBO	79
#define A6_REG_TS2_I2C_COMMAND_ADDR_LSB_MSB	80
#define A6_REG_TS2_I2C_COMMAND_DATA_LSB_MSB	81
#define A6_REG_REMOTE_MFGRID_V1	82
#define A6_REG_REMOTE_MFGRID_V2	83
#define A6_REG_REMOTE_PRODUCTID_V1	84
#define A6_REG_REMOTE_PRODUCTID_V2	85
#define A6_REG_REMOTE_SERNO_V1	86
#define A6_REG_REMOTE_SERNO_V2	87
#define A6_REG_LOCAL_MFGRID	88
#define A6_REG_LOCAL_PRODUCTID	89
#define A6_REG_LOCAL_SERNO	90


#define RSENSE_DEFAULT	20;
#define FORCE_WAKE_TIMER_EXPIRY (HZ/20)

#define a6_wait_event_ex(wq, condition)					\
do {									\
	if (condition)							\
		break;							\
	do {								\
		DEFINE_WAIT(__wait);					\
									\
		for (;;) {						\
			prepare_to_wait_exclusive(&wq, &__wait,		\
				TASK_UNINTERRUPTIBLE);			\
			if (condition)					\
				break;					\
			schedule();					\
		}							\
		finish_wait(&wq, &__wait);				\
	} while (0);							\
} while (0)


enum {
	DEVICE_BUSY_BIT = 0,
	IS_OPENED,
	IS_INITIALIZED_BIT,
	BOOTLOAD_ACTIVE_BIT,
	FORCE_WAKE_ACTIVE_BIT,
	READ_ACTIVE_BIT,
	WRITE_ACTIVE_BIT,
	A2A_CONNECTED,
	EXTRACT_INITIATED,
	// capabilities
	CAP_PERIODIC_WAKE,
#ifdef A6_PQ
	STARTING_AID_TASK,
	KILLING_AID_TASK,
	IS_QUIESCED,
#endif
	IS_SUSPENDED,
	INT_PENDING,
	SIZE_FLAGS
};

struct a6_device_state {
	struct i2c_client *i2c_dev;
	struct a6_platform_data *plat_data;
	struct file_operations fops;
	struct miscdevice mdev;

	struct mutex dev_mutex;
	unsigned int timestamping;

	struct timer_list	a6_force_wake_timer;
	struct work_struct	a6_force_wake_work;
	struct mutex		a6_force_wake_mutex;

	wait_queue_head_t	dev_busyq;
	struct work_struct	a6_irq_work;

	int32_t cpufreq_hold_flag;
	struct workqueue_struct* ka6d_workqueue;
	struct workqueue_struct* ka6d_fw_workqueue;

	uint32_t cached_rsense_val: 	16;
	uint32_t busy_count:		 8;
	DECLARE_BITMAP(flags, SIZE_FLAGS);

#ifdef A6_PQ
	struct completion aq_enq_complete;
	struct completion aid_exit_complete;
	struct mutex aq_mutex;
	struct list_head aq_head;
	struct task_struct* ai_dispatch_task;
#ifdef A6_DEBUG
	uint32_t dbgflg_kill_raid: 	1;

	uint8_t debug_restart_aid;
	uint8_t debug_flush_aiq;
	uint8_t debug_unused_01;
	uint8_t debug_unused_02;
#endif
#endif

	int cpufreq_hold;

	// pmem extract
	struct file_operations pmem_fops;
	struct miscdevice pmem_mdev;
	char pmem_dev_name[16];
	
	char *a2a_rd_buf, *a2a_wr_buf;
	char *a2a_rp, *a2a_wp;

	enum chg_type otg_chg_type;
	struct delayed_work charge_work;
	struct delayed_work init_connected_ps_work;
	int stop_heartbeat;
	int last_percent;

	/* used for the various generic_show calls, to be removed later */
	char *print_buffer;

	struct switch_dev *dock_switch;
};

struct a6_device_state *batt_state;

#ifdef A6_PQ
int32_t a6_start_ai_dispatch_task(struct a6_device_state* state);
int32_t a6_stop_ai_dispatch_task(struct a6_device_state* state);
int32_t flush_a6_action_items(struct a6_device_state* state);
#endif

static ssize_t a6_generic_show(struct device *dev, struct device_attribute *dev_attr, char *buf);
static ssize_t a6_generic_store(struct device *dev, struct device_attribute *dev_attr, const char *buf,
				size_t count);

typedef int32_t (*format_show_fn)(const struct a6_device_state* state, const uint8_t* val,
		  uint8_t* fmt_buffer, uint32_t size_buffer);
typedef int32_t (*format_store_fn)(const struct a6_device_state* state, const uint8_t* fmt_buffer,
		  uint8_t* val, uint32_t size_buffer);

int32_t format_current(const struct a6_device_state* state, const uint8_t* val,
		       uint8_t* fmt_buffer, uint32_t size_buffer);
int32_t format_voltage(const struct a6_device_state* state, const uint8_t* val,
		       uint8_t* fmt_buffer, uint32_t size_buffer);
int32_t r_format_voltage(const struct a6_device_state* state, const uint8_t* fmt_buffer,
			 uint8_t* val, uint32_t size_buffer);
int32_t format_rawcoulomb(const struct a6_device_state* state, const uint8_t* val,
		       uint8_t* fmt_buffer, uint32_t size_buffer);
int32_t format_coulomb(const struct a6_device_state* state, const uint8_t* val,
		       uint8_t* fmt_buffer, uint32_t size_buffer);
int32_t format_age(const struct a6_device_state* state, const uint8_t* val,
		       uint8_t* fmt_buffer, uint32_t size_buffer);
int32_t format_fullx(const struct a6_device_state* state, const uint8_t* val,
		       uint8_t* fmt_buffer, uint32_t size_buffer);
int32_t format_temp(const struct a6_device_state* state, const uint8_t* val,
		       uint8_t* fmt_buffer, uint32_t size_buffer);
int32_t r_format_temp(const struct a6_device_state* state, const uint8_t* fmt_buffer,
			 uint8_t* val, uint32_t size_buffer);
int32_t format_status(const struct a6_device_state* state, const uint8_t* val,
		       uint8_t* fmt_buffer, uint32_t size_buffer);
int32_t format_rsense(const struct a6_device_state* state, const uint8_t* val,
		       uint8_t* fmt_buffer, uint32_t size_buffer);
int32_t format_charge_source_status(const struct a6_device_state* state, const uint8_t* val,
		      uint8_t* fmt_buffer, uint32_t size_buffer);
int32_t format_version(const struct a6_device_state* state, const uint8_t* val,
		       uint8_t* fmt_buffer, uint32_t size_buffer);
int32_t format_v_offset(const struct a6_device_state* state, const uint8_t* val,
			uint8_t* fmt_buffer, uint32_t size_buffer);
int32_t format_command(const struct a6_device_state* state, const uint8_t* val,
		    uint8_t* fmt_buffer, uint32_t size_buffer);
int32_t format_accessory(const struct a6_device_state* state, const uint8_t* val,
		    uint8_t* fmt_buffer, uint32_t size_buffer);
int32_t format_comm_status(const struct a6_device_state* state, const uint8_t* val,
		    uint8_t* fmt_buffer, uint32_t size_buffer);
int32_t format_raw_unsigned(const struct a6_device_state* state, const uint8_t* val,
			   uint8_t* fmt_buffer, uint32_t size_buffer);
int32_t format_max_power_available(const struct a6_device_state* state, const uint8_t* val,
				   uint8_t* fmt_buffer, uint32_t size_buffer);
int32_t format_accessory_combo(const struct a6_device_state* state, const uint8_t* val,
			       uint8_t* fmt_buffer, uint32_t size_buffer);
int32_t format_u16_hex(const struct a6_device_state* state, const uint8_t* val,
		       uint8_t* fmt_buffer, uint32_t size_buffer);
int32_t format_serno_v1(const struct a6_device_state* state, const uint8_t* val,
			uint8_t* fmt_buffer, uint32_t size_buffer);
int32_t format_serno_v2(const struct a6_device_state* state, const uint8_t* val,
			uint8_t* fmt_buffer, uint32_t size_buffer);

static ssize_t a6_diag_show(struct device *dev, struct device_attribute *dev_attr, char *buf);
static ssize_t a6_diag_store(struct device *dev, struct device_attribute *dev_attr, const char *buf,
			     size_t count);
static ssize_t a6_val_cksum_show(struct device *dev, struct device_attribute *attr, char *buf);

static void a6_dock_update_state(struct a6_device_state *state);

static void a6_update_connected_ps(void);

void a6_force_wake_timer_callback(ulong data);
/*
 Note:  16-bit accesses comprising an LSB, MSB register pair must be specified in LSB:MSB order
	in the corresponding a6_register_desc struct. This is because an a6-fw constraint
	restricts 16-bit value accesses to MSB-leading only. The a6_i2c_read_reg and the
	a6_i2c_write_reg functions take the LSB:MSB definition in the a6_register_desc struct
	and re-orders the i2c txn to read in MSB:LSB order.
*/
struct a6_register_desc {
#define id_size 20

	const char* debug_name;
	struct device_attribute dev_attr;
	format_show_fn format;
	format_store_fn r_format;
	uint16_t id[id_size];
	uint32_t num_ids: 5;
	uint32_t ro: 1;
} a6_register_desc_arr[] = {
	/* interrupt control registers */
	[0] = { .debug_name = "TS2_I2C_INT_MASK_0",
		{{.name = "int_mask0", .mode = S_IRUGO | S_IWUGO},
		 .show = a6_generic_show, .store = a6_generic_store},
		.id = {TS2_I2C_INT_MASK_0}, .num_ids = 1, .ro = 0},
	[1] = { .debug_name = "TS2_I2C_INT_MASK_1",
		{{.name = "int_mask1", .mode = S_IRUGO | S_IWUGO},
		 .show = a6_generic_show, .store = a6_generic_store},
		.id = {TS2_I2C_INT_MASK_1}, .num_ids = 1, .ro = 0},
	[2] = { .debug_name = "TS2_I2C_INT_MASK_2",
		{{.name = "int_mask2", .mode = S_IRUGO | S_IWUGO},
		 .show = a6_generic_show, .store = a6_generic_store},
		.id = {TS2_I2C_INT_MASK_2}, .num_ids = 1, .ro = 0},
	[3] = { .debug_name = "TS2_I2C_INT_MASK_3",
		{{.name = "int_mask3", .mode = S_IRUGO | S_IWUGO},
		 .show = a6_generic_show, .store = a6_generic_store},
		.id = {TS2_I2C_INT_MASK_3}, .num_ids = 1, .ro = 0},
	[4] = { .debug_name = "TS2_I2C_INT_STATUS_0",
		{{.name = "int_status0", .mode = S_IRUGO | S_IWUGO},
		 .show = a6_generic_show, .store = a6_generic_store},
		.id = {TS2_I2C_INT_STATUS_0}, .num_ids = 1, .ro = 0},
	[5] = { .debug_name = "TS2_I2C_INT_STATUS_1",
		{{.name = "int_status1", .mode = S_IRUGO | S_IWUGO},
		 .show = a6_generic_show, .store = a6_generic_store},
		.id = {TS2_I2C_INT_STATUS_1}, .num_ids = 1, .ro = 0},
	[6] = { .debug_name = "TS2_I2C_INT_STATUS_2",
		{{.name = "int_status2", .mode = S_IRUGO | S_IWUGO},
		 .show = a6_generic_show, .store = a6_generic_store},
		.id = {TS2_I2C_INT_STATUS_2}, .num_ids = 1, .ro = 0},
	[7] = { .debug_name = "TS2_I2C_INT_STATUS_3",
		{{.name = "int_status3", .mode = S_IRUGO | S_IWUGO},
		 .show = a6_generic_show, .store = a6_generic_store},
		.id = {TS2_I2C_INT_STATUS_3}, .num_ids = 1, .ro = 0},

	/* battery control registers */
	[8] = { .debug_name = "TS2_I2C_BAT_STATUS",
		{{.name = "status", .mode = S_IRUGO},
		 .show = a6_generic_show, .store = NULL},
		.format = format_status,
		.id = {TS2_I2C_BAT_STATUS}, .num_ids = 1, .ro = 1},
	[9] = { .debug_name = "TS2_I2C_BAT_RARC",
		{{.name = "getpercent", .mode = S_IRUGO},
		 .show = a6_generic_show, .store = NULL},
		.id = {TS2_I2C_BAT_RARC}, .num_ids = 1, .ro = 1},
	[10] = { .debug_name = "TS2_I2C_BAT_RSRC",
		{{.name = "rsrc", .mode = S_IRUGO},
		 .show = a6_generic_show, .store = NULL},
		.id = {TS2_I2C_BAT_RSRC}, .num_ids = 1, .ro = 1},
	[11] = { .debug_name = "TS2_I2C_BAT_AVG_CUR_LSB_MSB",
		{{.name = "getavgcurrent", .mode = S_IRUGO},
		 .show = a6_generic_show, .store = NULL},
		.format = format_current,
		.id = {TS2_I2C_BAT_AVG_CUR_LSB, TS2_I2C_BAT_AVG_CUR_MSB},
			.num_ids = 2, .ro = 1},
	[12] = { .debug_name = "TS2_I2C_BAT_TEMP_LSB_MSB",
		{{.name = "gettemp", .mode = S_IRUGO},
		 .show = a6_generic_show, .store = NULL},
		.format = format_temp,
		.id = {TS2_I2C_BAT_TEMP_LSB, TS2_I2C_BAT_TEMP_MSB},
			.num_ids = 2, .ro = 1},
	[13] = { .debug_name = "TS2_I2C_BAT_VOLT_LSB_MSB",
		{{.name = "getvoltage", .mode = S_IRUGO},
		 .show = a6_generic_show, .store = NULL},
		.format = format_voltage,
		.id = {TS2_I2C_BAT_VOLT_LSB, TS2_I2C_BAT_VOLT_MSB},
			.num_ids = 2, .ro = 1},
	[14] = { .debug_name = "TS2_I2C_BAT_CUR_LSB_MSB",
		{{.name = "getcurrent", .mode = S_IRUGO},
		 .show = a6_generic_show, .store = NULL},
		.format = format_current,
		.id = {TS2_I2C_BAT_CUR_LSB, TS2_I2C_BAT_CUR_MSB},
			.num_ids = 2, .ro = 1},
	[15] = { .debug_name = "TS2_I2C_BAT_COULOMB_LSB_MSB",
		{{.name = "getrawcoulomb", .mode = S_IRUGO},
		 .show = a6_generic_show, .store = NULL},
		.format = format_rawcoulomb,
		.id = {TS2_I2C_BAT_COULOMB_LSB, TS2_I2C_BAT_COULOMB_MSB},
			.num_ids = 2, .ro = 1},
	[16] = { .debug_name = "TS2_I2C_BAT_AS",
		{{.name = "getage", .mode = S_IRUGO|S_IWUGO},
		 .show = a6_generic_show, .store = a6_generic_store},
		.format = format_age,
		.id = {TS2_I2C_BAT_AS}, .num_ids = 1, .ro = 0},
	[17] = { .debug_name = "TS2_I2C_BAT_FULL_LSB_MSB",
		{{.name = "getfull", .mode = S_IRUGO},
		 .show = a6_generic_show, .store = NULL},
		.format = format_fullx,
		.id = {TS2_I2C_BAT_FULL_LSB, TS2_I2C_BAT_FULL_MSB},
			.num_ids = 2, .ro = 1},
	[18] = {.debug_name = "TS2_I2C_BAT_FULL40_LSB_MSB",
		{{.name = "getfull40", .mode = S_IRUGO},
		 .show = a6_generic_show, .store = NULL},
		.format = format_fullx,
		.id = {TS2_I2C_BAT_FULL40_LSB, TS2_I2C_BAT_FULL40_MSB},
			.num_ids = 2, .ro = 1},
	[19] = {.debug_name = "TS2_I2C_BAT_RSNSP",
		{{.name = "getrsense", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_rsense,
		.id = {TS2_I2C_BAT_RSNSP}, .num_ids = 1, .ro = 1},

	[20] = {.debug_name = "TS2_I2C_BAT_ROMID_0",
		{{.name = "romid_0", .mode = S_IRUGO},
		 .show = a6_generic_show, .store = NULL},
		.id = {TS2_I2C_BAT_ROMID_0}, .num_ids = 1, .ro = 1},

	[21] = {.debug_name = "TS2_I2C_BAT_COMMAND_STATUS",
		{{.name = "command_status", .mode = S_IRUGO|S_IWUGO},
		 .show = a6_generic_show, .store = a6_generic_store},
		.id = {TS2_I2C_BAT_COMMAND_STATUS}, .num_ids = 1, .ro = 0},


	[22] = {.debug_name = "TS2_I2C_BAT_TEMP_LOW_LSB_MSB",
		{{.name = "temp_low", .mode = S_IRUGO|S_IWUGO},
		 .show = a6_generic_show, .store = a6_generic_store},
		.format = format_temp,
		.r_format = r_format_temp,
		.id = {TS2_I2C_BAT_TEMP_LOW_LSB, TS2_I2C_BAT_TEMP_LOW_MSB},
			.num_ids = 2, .ro = 0},
	[23] = {.debug_name = "TS2_I2C_BAT_TEMP_HIGH_LSB_MSB",
		{{.name = "temp_high", .mode = S_IRUGO|S_IWUGO},
		 .show = a6_generic_show, .store = a6_generic_store},
		.format = format_temp,
		.r_format = r_format_temp,
		.id = {TS2_I2C_BAT_TEMP_HIGH_LSB, TS2_I2C_BAT_TEMP_HIGH_MSB},
			.num_ids = 2, .ro = 0},
	[24] = {.debug_name = "TS2_I2C_BAT_VOLT_LOW_LSB_MSB",
		{{.name = "volt_low", .mode = S_IRUGO|S_IWUGO},
		 .show = a6_generic_show, .store = a6_generic_store},
		.format = format_voltage,
		.r_format = r_format_voltage,
		.id = {TS2_I2C_BAT_VOLT_LOW_LSB, TS2_I2C_BAT_VOLT_LOW_MSB},
			.num_ids = 2, .ro = 0},
	[25] = {.debug_name = "TS2_I2C_BAT_RARC_CRIT",
		{{.name = "rarc_crit", .mode = S_IRUGO|S_IWUGO},
		 .show = a6_generic_show, .store = a6_generic_store},
		.id = {TS2_I2C_BAT_RARC_CRIT}, .num_ids = 1, .ro = 0},
	[26] = {.debug_name = "TS2_I2C_BAT_RARC_LOW_2",
		{{.name = "rarc_low_2", .mode = S_IRUGO|S_IWUGO},
		 .show = a6_generic_show, .store = a6_generic_store},
		.id = {TS2_I2C_BAT_RARC_LOW_2}, .num_ids = 1, .ro = 0},
	[27] = {.debug_name = "TS2_I2C_BAT_RARC_LOW_1",
		{{.name = "rarc_low_1", .mode = S_IRUGO|S_IWUGO},
		 .show = a6_generic_show, .store = a6_generic_store},
		.id = {TS2_I2C_BAT_RARC_LOW_1}, .num_ids = 1, .ro = 0},
	[28] = {.debug_name = "TS2_I2C_BAT_RAAC_MSB",
		{{.name = "getcoulomb", .mode = S_IRUGO|S_IWUGO},
		 .show = a6_generic_show, .store = a6_generic_store},
		.format = format_coulomb,
		.id = {TS2_I2C_BAT_RAAC_LSB, TS2_I2C_BAT_RAAC_MSB},
			.num_ids = 2, .ro = 0},

	/* puck registers */
	[29] = {.debug_name = "TS2_I2C_ID",
		{{.name = "id", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.id = {TS2_I2C_ID}, .num_ids = 1, .ro = 1},
	[30] = {.debug_name = "TS2_I2C_FLAGS_0",
		{{.name = "puck_priority", .mode = S_IRUGO|S_IWUGO},
		.show = a6_generic_show, .store = a6_generic_store},
		.id = {TS2_I2C_FLAGS_0}, .num_ids = 1, .ro = 0},
	[31] = {.debug_name = "TS2_I2C_FLAGS_2",
		{{.name = "charger", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_charge_source_status,
		.id = {TS2_I2C_FLAGS_2}, .num_ids = 1, .ro = 1},

	/* enumeration registers */
	[32] = {.debug_name = "VERSION",
		{{.name = "getversion", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_version,
		.id = {	TS2_I2C_ENUM_MFGR_ID_LO, TS2_I2C_ENUM_MFGR_ID_HI,
			TS2_I2C_ENUM_PRODUCT_TYPE_LO, TS2_I2C_ENUM_PRODUCT_TYPE_HI,
			TS2_I2C_ENUM_SERNO_0, TS2_I2C_ENUM_SERNO_1,
			TS2_I2C_ENUM_SERNO_2, TS2_I2C_ENUM_SERNO_3,
			TS2_I2C_ENUM_SERNO_4, TS2_I2C_ENUM_SERNO_5,
			TS2_I2C_ENUM_SERNO_6, TS2_I2C_ENUM_SERNO_7,
			TS2_I2C_ENUM_ASSY_REV, TS2_I2C_ENUM_FW_VER_0,
			TS2_I2C_ENUM_FW_VER_1, TS2_I2C_ENUM_FW_VER_2},
		.num_ids = 16, .ro = 1},

	/* puck registers */
	[33] = {.debug_name = "TS2_I2C_V_OFFSET",
		{{.name = "v_offset", .mode = S_IRUGO|S_IWUGO},
		.show = a6_generic_show, .store = a6_generic_store},
		.format = format_v_offset,
		.id = {TS2_I2C_V_OFFSET}, .num_ids = 1, .ro = 0},

	/* self-wake registers */
	[34] = {.debug_name = "TS2_I2C_WAKEUP_PERIOD",
		{{.name = "periodic_wake_bit_params", .mode = S_IRUGO|S_IWUGO},
		.show = a6_generic_show, .store = a6_generic_store},
		.id = {TS2_I2C_WAKEUP_PERIOD}, .num_ids = 1, .ro = 0},

	/* command registers */
	[35] = {.debug_name = "TS2_I2C_COMMAND",
		{{.name = "command", .mode = S_IWUGO},
		.show = NULL, .store = a6_generic_store},
		.id = {TS2_I2C_COMMAND}, .num_ids = 1, .ro = 0},

	/* remote enumeration registers */
	[36] = {.debug_name = "REMOTE_VERSION",
		{{.name = "getremoteversion", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_version,
		.id = {	TS2_I2C_ENUM_REMOTE_MFGR_ID_LO, TS2_I2C_ENUM_REMOTE_MFGR_ID_HI,
			TS2_I2C_ENUM_REMOTE_PRODUCT_TYPE_LO, TS2_I2C_ENUM_REMOTE_PRODUCT_TYPE_HI,
			TS2_I2C_ENUM_REMOTE_SERNO_0, TS2_I2C_ENUM_REMOTE_SERNO_1,
			TS2_I2C_ENUM_REMOTE_SERNO_2, TS2_I2C_ENUM_REMOTE_SERNO_3,
			TS2_I2C_ENUM_REMOTE_SERNO_4, TS2_I2C_ENUM_REMOTE_SERNO_5,
			TS2_I2C_ENUM_REMOTE_SERNO_6, TS2_I2C_ENUM_REMOTE_SERNO_7,
			TS2_I2C_ENUM_REMOTE_ASSY_REV, TS2_I2C_ENUM_REMOTE_FW_VER_0,
			TS2_I2C_ENUM_REMOTE_FW_VER_1, TS2_I2C_ENUM_REMOTE_FW_VER_2},
		.num_ids = 16, .ro = 1},

	/* accessory data registers */
	[37] = {.debug_name = "ACCESSORY_DATA_0",
		{{.name = "acc_data_0", .mode = S_IRUGO|S_IWUGO},
		.show = a6_generic_show, .store = a6_generic_store},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_ACCE_0}, .num_ids = 1, .ro = 0},

	[38] = {.debug_name = "ACCESSORY_DATA_1",
		{{.name = "acc_data_1", .mode = S_IRUGO|S_IWUGO},
		.show = a6_generic_show, .store = a6_generic_store},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_ACCE_1}, .num_ids = 1, .ro = 0},

	[39] = {.debug_name = "ACCESSORY_DATA_2",
		{{.name = "acc_data_2", .mode = S_IRUGO|S_IWUGO},
		.show = a6_generic_show, .store = a6_generic_store},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_ACCE_2}, .num_ids = 1, .ro = 0},

	[40] = {.debug_name = "ACCESSORY_DATA_3",
		{{.name = "acc_data_3", .mode = S_IRUGO|S_IWUGO},
		.show = a6_generic_show, .store = a6_generic_store},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_ACCE_3}, .num_ids = 1, .ro = 0},

	[41] = {.debug_name = "ACCESSORY_DATA_4",
		{{.name = "acc_data_4", .mode = S_IRUGO|S_IWUGO},
		.show = a6_generic_show, .store = a6_generic_store},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_ACCE_4}, .num_ids = 1, .ro = 0},

	[42] = {.debug_name = "ACCESSORY_DATA_5",
		{{.name = "acc_data_5", .mode = S_IRUGO|S_IWUGO},
		.show = a6_generic_show, .store = a6_generic_store},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_ACCE_5}, .num_ids = 1, .ro = 0},

	[43] = {.debug_name = "ACCESSORY_DATA_6",
		{{.name = "acc_data_6", .mode = S_IRUGO|S_IWUGO},
		.show = a6_generic_show, .store = a6_generic_store},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_ACCE_6}, .num_ids = 1, .ro = 0},

	[44] = {.debug_name = "ACCESSORY_DATA_7",
		{{.name = "acc_data_7", .mode = S_IRUGO|S_IWUGO},
		.show = a6_generic_show, .store = a6_generic_store},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_ACCE_7}, .num_ids = 1, .ro = 0},

	[45] = {.debug_name = "ACCESSORY_DATA_8",
		{{.name = "acc_data_8", .mode = S_IRUGO|S_IWUGO},
		.show = a6_generic_show, .store = a6_generic_store},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_ACCE_8}, .num_ids = 1, .ro = 0},

	[46] = {.debug_name = "ACCESSORY_DATA_9",
		{{.name = "acc_data_9", .mode = S_IRUGO|S_IWUGO},
		.show = a6_generic_show, .store = a6_generic_store},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_ACCE_9}, .num_ids = 1, .ro = 0},

	[47] = {.debug_name = "ACCESSORY_DATA_10",
		{{.name = "acc_data_10", .mode = S_IRUGO|S_IWUGO},
		.show = a6_generic_show, .store = a6_generic_store},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_ACCE_10}, .num_ids = 1, .ro = 0},

	[48] = {.debug_name = "ACCESSORY_DATA_11",
		{{.name = "acc_data_11", .mode = S_IRUGO|S_IWUGO},
		.show = a6_generic_show, .store = a6_generic_store},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_ACCE_11}, .num_ids = 1, .ro = 0},

	[49] = {.debug_name = "ACCESSORY_DATA_12",
		{{.name = "acc_data_12", .mode = S_IRUGO|S_IWUGO},
		.show = a6_generic_show, .store = a6_generic_store},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_ACCE_12}, .num_ids = 1, .ro = 0},

	/* remote accessory data registers */
	[50] = {.debug_name = "REMOTE_ACCESSORY_DATA_0",
		{{.name = "remote_acc_data_0", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_REMOTE_ACCE_0}, .num_ids = 1, .ro = 1},

	[51] = {.debug_name = "REMOTE_ACCESSORY_DATA_1",
		{{.name = "remote_acc_data_1", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_REMOTE_ACCE_1}, .num_ids = 1, .ro = 1},

	[52] = {.debug_name = "REMOTE_ACCESSORY_DATA_2",
		{{.name = "remote_acc_data_2", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_REMOTE_ACCE_2}, .num_ids = 1, .ro = 1},

	[53] = {.debug_name = "REMOTE_ACCESSORY_DATA_3",
		{{.name = "remote_acc_data_3", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_REMOTE_ACCE_3}, .num_ids = 1, .ro = 1},

	[54] = {.debug_name = "REMOTE_ACCESSORY_DATA_4",
		{{.name = "remote_acc_data_4", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_REMOTE_ACCE_4}, .num_ids = 1, .ro = 1},

	[55] = {.debug_name = "REMOTE_ACCESSORY_DATA_5",
		{{.name = "remote_acc_data_5", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_REMOTE_ACCE_5}, .num_ids = 1, .ro = 1},

	[56] = {.debug_name = "REMOTE_ACCESSORY_DATA_6",
		{{.name = "remote_acc_data_6", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_REMOTE_ACCE_6}, .num_ids = 1, .ro = 1},

	[57] = {.debug_name = "REMOTE_ACCESSORY_DATA_7",
		{{.name = "remote_acc_data_7", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_REMOTE_ACCE_7}, .num_ids = 1, .ro = 1},

	[58] = {.debug_name = "REMOTE_ACCESSORY_DATA_8",
		{{.name = "remote_acc_data_8", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_REMOTE_ACCE_8}, .num_ids = 1, .ro = 1},

	[59] = {.debug_name = "REMOTE_ACCESSORY_DATA_9",
		{{.name = "remote_acc_data_9", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_REMOTE_ACCE_9}, .num_ids = 1, .ro = 1},

	[60] = {.debug_name = "REMOTE_ACCESSORY_DATA_10",
		{{.name = "remote_acc_data_10", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_REMOTE_ACCE_10}, .num_ids = 1, .ro = 1},

	[61] = {.debug_name = "REMOTE_ACCESSORY_DATA_11",
		{{.name = "remote_acc_data_11", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_REMOTE_ACCE_11}, .num_ids = 1, .ro = 1},

	[62] = {.debug_name = "REMOTE_ACCESSORY_DATA_12",
		{{.name = "remote_acc_data_12", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_REMOTE_ACCE_12}, .num_ids = 1, .ro = 1},

	[63] = {.debug_name = "TS2_I2C_COMM_STATUS",
		{{.name = "get_comm_status", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_comm_status,
		.id = {TS2_I2C_COMM_STATUS}, .num_ids = 1, .ro = 1},

	[64] = {.debug_name = "TS2_I2C_COMM_TXDATA_RXDATA",
		{{.name = "comm_txdata_rx_data", .mode = S_IRUGO|S_IWUGO},
		.show = a6_generic_show, .store = a6_generic_store},
		.format = format_comm_status,
		.id = {TS2_I2C_COMM_TXDATA_RXDATA}, .num_ids = 1, .ro = 0},

	[65] = { .debug_name = "TS2_I2C_BAT_SACR_LSB_MSB",
		{{.name = "getsacr", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_rawcoulomb,
		.id = {TS2_I2C_BAT_SACR_LSB, TS2_I2C_BAT_SACR_MSB},
			.num_ids = 2, .ro = 0},

	[66] = { .debug_name = "TS2_I2C_BAT_ASL",
		{{.name = "getasl", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_raw_unsigned,
		.id = {TS2_I2C_BAT_ASL}, .num_ids = 1, .ro = 0},

	[67] = { .debug_name = "TS2_I2C_BAT_AS",
		{{.name = "getrawas", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_raw_unsigned,
		.id = {TS2_I2C_BAT_AS}, .num_ids = 1, .ro = 0},

	[68] = { .debug_name = "TS2_I2C_BAT_FAC_LSB_MSB",
		{{.name = "getfac", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_rawcoulomb,
		.id = {TS2_I2C_BAT_FAC_LSB, TS2_I2C_BAT_FAC_MSB},
			.num_ids = 2, .ro = 0},

	[69] = {.debug_name = "MAX_POWER_AVAILABLE",
		{{.name = "getmaxpoweravail", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_max_power_available,
		.id = { TS2_I2C_ENUM_REMOTE_VNODE_MAX_LO, TS2_I2C_ENUM_REMOTE_VNODE_MAX_HI,
			TS2_I2C_ENUM_REMOTE_INODE_MAX_LO, TS2_I2C_ENUM_REMOTE_INODE_MAX_HI,
			TS2_I2C_ENUM_REMOTE_POWER_MAX},
		.num_ids = 5, .ro = 1},

	[70] = {.debug_name = "TS2_I2C_ENUM_REMOTE_STRUCT_VER",
		{{.name = "remote_struct_ver", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.id = {TS2_I2C_ENUM_REMOTE_STRUCT_VER}, .num_ids = 1, .ro = 1},

	[71] = {.debug_name = "ACCESSORY_DATA_13",
		{{.name = "acc_data_13", .mode = S_IRUGO|S_IWUGO},
		.show = a6_generic_show, .store = a6_generic_store},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_ACCE_13}, .num_ids = 1, .ro = 0},

	[72] = {.debug_name = "ACCESSORY_DATA_14",
		{{.name = "acc_data_14", .mode = S_IRUGO|S_IWUGO},
		.show = a6_generic_show, .store = a6_generic_store},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_ACCE_14}, .num_ids = 1, .ro = 0},

	[73] = {.debug_name = "ACCESSORY_DATA_15",
		{{.name = "acc_data_15", .mode = S_IRUGO|S_IWUGO},
		.show = a6_generic_show, .store = a6_generic_store},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_ACCE_15}, .num_ids = 1, .ro = 0},

	[74] = {.debug_name = "REMOTE_ACCESSORY_DATA_13",
		{{.name = "remote_acc_data_13", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_REMOTE_ACCE_13}, .num_ids = 1, .ro = 1},

	[75] = {.debug_name = "REMOTE_ACCESSORY_DATA_14",
		{{.name = "remote_acc_data_14", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_REMOTE_ACCE_14}, .num_ids = 1, .ro = 1},

	[76] = {.debug_name = "REMOTE_ACCESSORY_DATA_15",
		{{.name = "remote_acc_data_15", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_accessory,
		.id = {	TS2_I2C_ENUM_REMOTE_ACCE_15}, .num_ids = 1, .ro = 1},

	[77] = {.debug_name = "TS2_I2C_ENUM_MIN_PWM",
		{{.name = "min_pwm", .mode = S_IRUGO|S_IWUGO},
		.show = a6_generic_show, .store = a6_generic_store},
		.id = {	TS2_I2C_ENUM_MIN_PWM}, .num_ids = 1, .ro = 0},

	[78] = {.debug_name = "ACCESSORY_DATA_COMBO",
		{{.name = "acc_data_combo", .mode = S_IRUGO|S_IWUGO},
		.show = a6_generic_show, .store = a6_generic_store},
		.format = format_accessory_combo,
		.id = { TS2_I2C_ENUM_ACCE_0, TS2_I2C_ENUM_ACCE_1, TS2_I2C_ENUM_ACCE_2,
			TS2_I2C_ENUM_ACCE_3, TS2_I2C_ENUM_ACCE_4, TS2_I2C_ENUM_ACCE_5,
			TS2_I2C_ENUM_ACCE_6, TS2_I2C_ENUM_ACCE_7, TS2_I2C_ENUM_ACCE_8,
			TS2_I2C_ENUM_ACCE_9, TS2_I2C_ENUM_ACCE_10, TS2_I2C_ENUM_ACCE_11,
			TS2_I2C_ENUM_ACCE_12, TS2_I2C_ENUM_ACCE_13, TS2_I2C_ENUM_ACCE_14,
			TS2_I2C_ENUM_ACCE_15},
		.num_ids = 16, .ro = 0},

	[79] = {.debug_name = "REMOTE_ACCESSORY_DATA_COMBO",
		{{.name = "remote_acc_data_combo", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_accessory_combo,
		.id = { TS2_I2C_ENUM_REMOTE_ACCE_0, TS2_I2C_ENUM_REMOTE_ACCE_1,
			TS2_I2C_ENUM_REMOTE_ACCE_2, TS2_I2C_ENUM_REMOTE_ACCE_3,
			TS2_I2C_ENUM_REMOTE_ACCE_4, TS2_I2C_ENUM_REMOTE_ACCE_5,
			TS2_I2C_ENUM_REMOTE_ACCE_6, TS2_I2C_ENUM_REMOTE_ACCE_7,
			TS2_I2C_ENUM_REMOTE_ACCE_8, TS2_I2C_ENUM_REMOTE_ACCE_9,
			TS2_I2C_ENUM_REMOTE_ACCE_10, TS2_I2C_ENUM_REMOTE_ACCE_11,
			TS2_I2C_ENUM_REMOTE_ACCE_12, TS2_I2C_ENUM_REMOTE_ACCE_13,
			TS2_I2C_ENUM_REMOTE_ACCE_14, TS2_I2C_ENUM_REMOTE_ACCE_15},
		.num_ids = 16, .ro = 1},

	[80] = {.debug_name = "TS2_I2C_COMMAND_ADDR_LSB_MSB",
		{{.name = "command_addr", .mode = S_IRUGO|S_IWUGO},
		.show = a6_generic_show, .store = a6_generic_store},
		.id = {TS2_I2C_COMMAND_ADDR_LSB, TS2_I2C_COMMAND_ADDR_MSB},
		.format = format_u16_hex,
		.num_ids = 2, .ro = 0},
	[81] = {.debug_name = "TS2_I2C_COMMAND_DATA_LSB_MSB",
		{{.name = "command_data", .mode = S_IRUGO|S_IWUGO},
		.show = a6_generic_show, .store = a6_generic_store},
		.id = {TS2_I2C_COMMAND_DATA_LSB, TS2_I2C_COMMAND_DATA_MSB},
		.format = format_u16_hex,
		.num_ids = 2, .ro = 0},

	[82] = {.debug_name = "REMOTE_MFGRID_V1",
		{{.name = "remote_mfgrid_v1", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_u16_hex,
		.id = { TS2_I2C_ENUM_REMOTE_MFGR_ID_LO_V1,
			TS2_I2C_ENUM_REMOTE_MFGR_ID_HI_V1},
		.num_ids = 2, .ro = 1},
	[83] = {.debug_name = "REMOTE_MFGRID_V2",
		{{.name = "remote_mfgrid_v2", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_u16_hex,
		.id = { TS2_I2C_ENUM_REMOTE_MFGR_ID_LO,
			TS2_I2C_ENUM_REMOTE_MFGR_ID_HI},
		.num_ids = 2, .ro = 1},
	[84] = {.debug_name = "REMOTE_PRODUCTID_V1",
		{{.name = "remote_productid_v1", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_u16_hex,
		.id = { TS2_I2C_ENUM_REMOTE_PRODUCT_TYPE_LO_V1,
			TS2_I2C_ENUM_REMOTE_PRODUCT_TYPE_HI_V1},
		.num_ids = 2, .ro = 1},
	[85] = {.debug_name = "REMOTE_PRODUCTID_V2",
		{{.name = "remote_productid_v2", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_u16_hex,
		.id = { TS2_I2C_ENUM_REMOTE_PRODUCT_TYPE_LO,
			TS2_I2C_ENUM_REMOTE_PRODUCT_TYPE_HI},
		.num_ids = 2, .ro = 1},
	[86] = {.debug_name = "REMOTE_SERNO_V1",
		{{.name = "remote_serno_v1", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_serno_v1,
		.id = { TS2_I2C_ENUM_REMOTE_SERNO_0, TS2_I2C_ENUM_REMOTE_SERNO_1,
			TS2_I2C_ENUM_REMOTE_SERNO_2, TS2_I2C_ENUM_REMOTE_SERNO_3,
			TS2_I2C_ENUM_REMOTE_SERNO_4, TS2_I2C_ENUM_REMOTE_SERNO_5},
		.num_ids = 6, .ro = 1},
	[87] = {.debug_name = "REMOTE_SERNO_V2",
		{{.name = "remote_serno_v2", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_serno_v2,
		.id = { TS2_I2C_ENUM_REMOTE_SERNO_0, TS2_I2C_ENUM_REMOTE_SERNO_1,
			TS2_I2C_ENUM_REMOTE_SERNO_2, TS2_I2C_ENUM_REMOTE_SERNO_3,
			TS2_I2C_ENUM_REMOTE_SERNO_4, TS2_I2C_ENUM_REMOTE_SERNO_5,
			TS2_I2C_ENUM_REMOTE_SERNO_6, TS2_I2C_ENUM_REMOTE_SERNO_7},
		.num_ids = 8, .ro = 1},
	[88] = {.debug_name = "LOCAL_MFGRID",
		{{.name = "local_mfgrid", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_u16_hex,
		.id = { TS2_I2C_ENUM_MFGR_ID_LO, TS2_I2C_ENUM_MFGR_ID_HI},
		.num_ids = 2, .ro = 1},
	[89] = {.debug_name = "LOCAL_PRODUCTID",
		{{.name = "local_productid", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_u16_hex,
		.id = { TS2_I2C_ENUM_PRODUCT_TYPE_LO, TS2_I2C_ENUM_PRODUCT_TYPE_HI},
		.num_ids = 2, .ro = 1},
	[90] = {.debug_name = "LOCAL_SERNO",
		{{.name = "local_serno", .mode = S_IRUGO},
		.show = a6_generic_show, .store = NULL},
		.format = format_serno_v2,
		.id = { TS2_I2C_ENUM_SERNO_0, TS2_I2C_ENUM_SERNO_1,
			TS2_I2C_ENUM_SERNO_2, TS2_I2C_ENUM_SERNO_3,
			TS2_I2C_ENUM_SERNO_4, TS2_I2C_ENUM_SERNO_5,
			TS2_I2C_ENUM_SERNO_6, TS2_I2C_ENUM_SERNO_7},
			.num_ids = 8, .ro = 1},
};


struct device_attribute custom_devattr[] = {
	{{.name = "a6_diag", .mode = S_IRUGO | S_IWUGO},
	 .show = a6_diag_show, .store = a6_diag_store},
 	{{.name = "validate_cksum", .mode = S_IRUGO},
 	 .show = a6_val_cksum_show, .store = NULL},
};

static enum power_supply_property a6_fish_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
};

static enum power_supply_property a6_fish_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *supply_list[] = {
	"battery",
};

static int a6_fish_power_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val);

static int a6_fish_battery_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val);

static struct power_supply a6_fish_power_supplies[] = {
	{
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = a6_fish_battery_properties,
		.num_properties = ARRAY_SIZE(a6_fish_battery_properties),
		.get_property = a6_fish_battery_get_property,
	},
	{
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = a6_fish_power_properties,
		.num_properties = ARRAY_SIZE(a6_fish_power_properties),
		.get_property = a6_fish_power_get_property,
	},
	{
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = a6_fish_power_properties,
		.num_properties = ARRAY_SIZE(a6_fish_power_properties),
		.get_property = a6_fish_power_get_property,
	},
#ifdef CONFIG_A6_ENABLE_DOCK_PS
	{
		.name = "dock",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = a6_fish_power_properties,
		.num_properties = ARRAY_SIZE(a6_fish_power_properties),
		.get_property = a6_fish_power_get_property,
	},
#endif
};

#ifdef A6_PQ
// a6 debugfs interface...
#ifdef A6_DEBUG


static int32_t a6_restart_aid_thread_fn(void* param)
{
	struct a6_device_state* state = param;
	int32_t rc = 0;

	daemonize("dbg_aidrst_%s", state->plat_data->dev_name);
	while (!state->dbgflg_kill_raid) {
		// stop aid task
		rc = a6_stop_ai_dispatch_task(state);
		if (rc) {
			printk(KERN_ERR "%s: failed to stop ai_dispatch_task.\n", __func__);
			break;
		}

		// re-start aid task
		rc = a6_start_ai_dispatch_task(state);
		if (rc) {
			printk(KERN_ERR "%s: failed to start ai_dispatch_task.\n", __func__);
			break;
		}

		// sleep
		msleep(500);
	}

	rc = mutex_lock_interruptible(&state->dev_mutex);
	if (rc) {
		printk(KERN_ERR "%s: mutex_lock_interruptible interrupted\n", __func__);
		return -ERESTARTSYS;
	}
	state->debug_restart_aid = 0;
	state->dbgflg_kill_raid = 0;
	mutex_unlock(&state->dev_mutex);

	return rc;
}

static int a6_test_restart_aid_set(void *data, u64 val)
{
	int32_t rc = 0;
	pid_t aiq_flush_pid = 0;
	struct a6_device_state* state = data;
	uint8_t in_val, curr_val;

	in_val = val ? 1 : val;
	rc = mutex_lock_interruptible(&state->dev_mutex);
	if (rc) {
		printk(KERN_ERR "%s: mutex_lock_interruptible interrupted\n", __func__);
		return -ERESTARTSYS;
	}
	curr_val = state->debug_restart_aid;

	// are we actually changing state?
	if (in_val ^ curr_val) {
		if (in_val) {
			// prev task still being killed or active: fail
			if (state->dbgflg_kill_raid || state->debug_restart_aid) {
				printk(KERN_ERR "%s: prev task still being killed or active: fail\n",
				       __func__);
				goto err0;
			}
			// create ai dispatcher task...
			aiq_flush_pid = kernel_thread(a6_restart_aid_thread_fn, state, CLONE_KERNEL);
			ASSERT(aiq_flush_pid >= 0);
			state->debug_restart_aid = in_val;
		}
		else {
			// prev task still being killed or not active: fail
			if (state->dbgflg_kill_raid || !state->debug_restart_aid) {
				printk(KERN_ERR "%s: prev task still being killed or not active: fail\n",
				       __func__);
				goto err0;
			}
			// intent to kill task; task resets state as part of teardown
			state->dbgflg_kill_raid = 1;
		}
	}
	mutex_unlock(&state->dev_mutex);


err0:
	return rc;
}
static int a6_test_restart_aid_get(void *data, u64 *val)
{
	struct a6_device_state* state = data;

	*val = state->debug_restart_aid;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fops_a6_test_restart_aid, a6_test_restart_aid_get, a6_test_restart_aid_set, "%llu\n");

static int32_t a6_create_debug_interface(struct a6_device_state* state)
{
	int32_t rc = 0;
	struct dentry *dentry_parent, *dentry_child;

	dentry_parent = debugfs_create_dir("a6", 0);
	if (IS_ERR(dentry_parent)) {
		rc = PTR_ERR(dentry_parent);
		goto err0;
	}

	dentry_child = debugfs_create_file("periodic_restart_aid", 0644,
					   dentry_parent, state, &fops_a6_test_restart_aid);
	if (IS_ERR(dentry_child)) {
		rc = PTR_ERR(dentry_child);
		goto err0;
	}

	return 0;

err0:
	debugfs_remove_recursive(dentry_parent);
	return rc;
}
#endif
#endif // A6_PQ

#ifdef A6_PQ
// a6 action item types
enum {
	AI_I2C_TYPE,
};

struct a6_action_item {
	void* ai_payload;
	uint32_t ai_type: 4;
	uint32_t ai_complete:1;
	int32_t (*ai_do_action)(void* payload_param);
	int32_t* (*ai_ret_code)(void* payload_param);
	wait_queue_head_t ai_waitq;
	struct list_head list;
};

/*
 enq_a6_action_item: q's action item and blocks till action completion...
*/
int32_t enq_a6_action_item(struct a6_device_state* state, struct a6_action_item* ai, bool enq_tail)
{
	int32_t rc = 0;

	mutex_lock(&state->aq_mutex);
	if (true == enq_tail) {
		list_add_tail(&ai->list, &state->aq_head);
	}
	else {
		list_add(&ai->list, &state->aq_head);
	}
	mutex_unlock(&state->aq_mutex);

	// signal the action_item dispatcher thread
	complete(&state->aq_enq_complete);

	// ** wait for action_item to be processed
	a6_wait_event_ex(ai->ai_waitq, ai->ai_complete);
	rc = *ai->ai_ret_code(ai->ai_payload);

	return rc;
}

/*
 dq_a6_action_item: dq's head item from action item list...
*/
struct a6_action_item* dq_a6_action_item(struct a6_device_state* state)
{
	struct a6_action_item* ai = NULL;

	mutex_lock(&state->aq_mutex);
	if (!list_empty(&state->aq_head)) {
		ai = list_first_entry(&state->aq_head, struct a6_action_item, list);
		list_del(state->aq_head.next);
	}
	mutex_unlock(&state->aq_mutex);

	return ai;
}

/*
 flush_a6_action_items: iterates action item list, dq's each item, marks it cancelled
			and complete and then wakes up the requestor...
*/
int32_t flush_a6_action_items(struct a6_device_state* state)
{
	int32_t rc = 0;
	struct a6_action_item* ai = NULL;

	// lock list until all items removed
	mutex_lock(&state->aq_mutex);
	while (!list_empty(&state->aq_head)) {
		// get first entry...
		ai = list_first_entry(&state->aq_head, struct a6_action_item, list);
		// ... and remove from list
		list_del(state->aq_head.next);
		*ai->ai_ret_code(ai->ai_payload) = -ECANCELED;
		// set completion indicator
		ai->ai_complete = 1;
		// signal ai requestor
		wake_up(&ai->ai_waitq);
	}
	mutex_unlock(&state->aq_mutex);

	return rc;
}


struct ai_i2c_payload {
	struct i2c_client* client;
	struct i2c_msg* msg;
	uint32_t num_msgs;
	int32_t rc;
};

/*
 do_i2c_action_item: i2c-specific action implementation...
*/
int32_t do_i2c_action_item(void* payload_param)
{
	struct ai_i2c_payload* trf_data = (struct ai_i2c_payload*)payload_param;

	trf_data->rc = i2c_transfer(trf_data->client->adapter, trf_data->msg, trf_data->num_msgs);
	udelay(700);
	if (trf_data->rc < 0) {
		printk(KERN_ERR "%s: err code: %d\n", __func__, trf_data->rc);
		goto err0;
	}

	trf_data->rc = 0;
err0:
	return trf_data->rc;
}

/*
 i2c_ret_code: i2c-specific ret-code reference retrieval...
*/
int32_t* i2c_ret_code(void* payload_param)
{
	struct ai_i2c_payload* trf_data = (struct ai_i2c_payload*)payload_param;

	return (&trf_data->rc);
}

/*
 q_a6_i2c_action_item: creates the action item structure and enq's it...
*/
int32_t q_a6_i2c_action_item(struct i2c_client* client, struct i2c_msg* msg, uint32_t num_msgs)
{
	int32_t ret = 0;
	struct ai_i2c_payload data = {
		.client = client,
		.msg = msg,
		.num_msgs = num_msgs,
		.rc = 0
	};

	struct a6_action_item a6_i2c_ai = {
		.ai_payload = &data,
		.ai_type = AI_I2C_TYPE,
		.ai_complete = 0,
		.ai_do_action = do_i2c_action_item,
		.ai_ret_code = i2c_ret_code,
		.list = LIST_HEAD_INIT(a6_i2c_ai.list)
	};

	init_waitqueue_head(&a6_i2c_ai.ai_waitq);
	ret = enq_a6_action_item(i2c_get_clientdata(client), &a6_i2c_ai, true/*enq tail*/);

	return ret;
}

/*
 ai_dispatch_thread_fn: function that handles ai processing in the aid task...
*/
int ai_dispatch_thread_fn(void* param)
{
	int32_t rc = 0;
	struct a6_action_item* ai;
	struct a6_device_state* state = param;

	daemonize("aid_%s", state->plat_data->dev_name);
	do {
		A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: about to wait for ai enq complete.\n", __func__);
		// wait for ai to be enq'd
		rc = wait_for_completion_interruptible(&state->aq_enq_complete);
		if (rc >= 0) {
			// check kill status: intent to kill?
			if (test_bit(KILLING_AID_TASK, state->flags)) {
				// remove all items from q
				flush_a6_action_items(state);
				// and signal killer before exiting stage...
				complete(&state->aid_exit_complete);
				break;
			}

			ai = dq_a6_action_item(state);
			if (ai) {
				int32_t ret_val;

				A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: got ai.\n", __func__);
				// invoke ai-specific action
				ret_val = ai->ai_do_action(ai->ai_payload);
				if (ret_val) {
					printk(KERN_ERR "%s: ai_do_action failed.\n", __func__);
				}

				// set completion indicator
				ai->ai_complete = 1;
				// signal ai requestor
				wake_up(&ai->ai_waitq);
			}
			else {
				A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: no ai.\n", __func__);
			}
		}
		// wait interrupted by -ERESTARTSYS: let's exit
		else {
			printk(KERN_ERR "%s: wait for action_item enq interrupted.\n",
			       __func__);
			// force a panic...
			BUG_ON(rc < 0);
		}
	} while(rc >= 0);

	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "*** %s: exiting ai_dispatch_thread_fn.***\n", __func__);

	return rc;
}
#endif  // A6_PQ


int32_t __a6_i2c_read_reg(struct i2c_client* client, const uint16_t* ids, uint32_t num_ids, uint8_t* out)
{
	int32_t ret = 0, i;
	uint16_t swp_addr[num_ids];
	struct i2c_msg msg[num_ids*2], *msg_itr;
	struct a6_device_state* state = i2c_get_clientdata(client);
#ifdef A6_I2C_PROFILE
	ktime_t start, end;
#endif

#ifdef A6_PQ
	if (test_bit(IS_QUIESCED, ((struct a6_device_state*)i2c_get_clientdata(client))->flags)) {
		ret = -ECANCELED;
		goto err0;
	}
#endif // A6_PQ

	// force A6 wakeup...
	if (start_last_a6_activity) {
		long diff_time = (long)jiffies - (long)start_last_a6_activity;
		A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: time since last activity: %ld ms\n",
		       __func__, diff_time * 1000/HZ);
	}
	start_last_a6_activity = jiffies;

	// prevent timer expiry during force_wake related action...
	del_timer(&state->a6_force_wake_timer);

	mutex_lock(&state->a6_force_wake_mutex);
	// a6 external wake enabled?
	if (test_bit(CAP_PERIODIC_WAKE, state->flags)) {
		if (!test_bit(FORCE_WAKE_ACTIVE_BIT, state->flags)) {
			struct a6_wake_ops* wake_ops = (struct a6_wake_ops*)state->plat_data->wake_ops;

			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR,
			   	"%s: disabling periodic_wake and switching to force_wake\n",
			   	__func__);

			// periodic wake active: disable it and switch to force wake
			if (wake_ops->disable_periodic_wake) {
				wake_ops->disable_periodic_wake(wake_ops->data);
			}
			if (wake_ops->force_wake) {
				wake_ops->force_wake(wake_ops->data);
			}

			set_bit(FORCE_WAKE_ACTIVE_BIT, state->flags);
		}
	}
	mutex_unlock(&state->a6_force_wake_mutex);

#ifdef A6_I2C_PROFILE
	start = ktime_get();
#endif
	msg_itr = &msg[0];
	for (i = num_ids-1; i >= 0; i--) {
		msg_itr->addr = client->addr;
		msg_itr->flags = 0;
		msg_itr->len = sizeof(uint16_t);
		swp_addr[i] = ids[i] >> 8 | ids[i] << 8;
		msg_itr->buf = (uint8_t*)&swp_addr[i];

		(msg_itr+1)->addr = client->addr;
		(msg_itr+1)->flags = I2C_M_RD;
		(msg_itr+1)->len = sizeof(uint8_t);
		(msg_itr+1)->buf = &out[i];
#ifdef CONFIG_A6_I2C_SINGLE_BYTE
#ifdef A6_PQ
		ret = q_a6_i2c_action_item(client, msg, 2);
#else
		ret = i2c_transfer(client->adapter, msg, 2);
#endif // A6_PQ
#endif
		msg_itr += 2;
	}


#ifndef CONFIG_A6_I2C_SINGLE_BYTE
#ifdef A6_PQ
	ret = q_a6_i2c_action_item(client, msg, num_ids*2);
#else
	ret = i2c_transfer(client->adapter, msg, num_ids*2);
#endif // A6_PQ
#endif
	//msleep(1);
#ifdef A6_I2C_PROFILE
	{
		end = ktime_get();
		printk(KERN_ERR "%s[0x%02x]: elpased time: %lld\n",
		       __func__, client->addr, ktime_to_ns(ktime_sub(end, start)));
	}
#endif
	if (ret < 0) {
		printk(KERN_ERR "%s[0x%02x]: err code: %d\n", __func__, client->addr, ret);
		// reset the force_wake timer inedependent of i2c failure
		//goto err0;
	}
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "ret val: 0x%x\n", *out);

	if (test_bit(CAP_PERIODIC_WAKE, state->flags)) {
		// [re]start force-wake expiry timer
		mod_timer(&state->a6_force_wake_timer, jiffies+FORCE_WAKE_TIMER_EXPIRY);
	}

err0:
	return ret;
}

int32_t a6_i2c_read_reg(struct i2c_client* client, const uint16_t* ids, uint32_t num_ids, uint8_t* out)
{
	int32_t ret;
#ifdef A6_I2C_RETRY
	int32_t retry = 5;
#else
	int32_t retry = 0;
#endif

	do {
		ret = __a6_i2c_read_reg(client, ids, num_ids, out);
		if (ret < 0) {
			printk("%s: a6 i2c transaction failed. %s...\n", __func__, retry ? "retry" : " ");
			msleep(30);
		}
	} while (0 != ret && retry-- > 0);

	return ret;
}

int32_t __a6_i2c_write_reg(struct i2c_client* client, const uint16_t* ids, uint32_t num_ids, const uint8_t* in)
{
	int32_t ret = 0, i;
	uint8_t i2c_buf[(2+1)*num_ids];
	struct i2c_msg msg[num_ids], *msg_itr;
	struct a6_device_state* state = i2c_get_clientdata(client);

#ifdef A6_PQ
	if (test_bit(IS_QUIESCED, ((struct a6_device_state*)i2c_get_clientdata(client))->flags)) {
		ret = -ECANCELED;
		goto err0;
	}
#endif // A6_PQ

	// force A6 wakeup...
	if (start_last_a6_activity) {
		long diff_time = (long)jiffies - (long)start_last_a6_activity;
		A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: time since last activity: %ld ms\n",
		       __func__, diff_time * 1000/HZ);
	}
	start_last_a6_activity = jiffies;

	// prevent timer expiry during force_wake related action...
	del_timer(&state->a6_force_wake_timer);

	mutex_lock(&state->a6_force_wake_mutex);
	if (test_bit(CAP_PERIODIC_WAKE, state->flags)) {
		if (!test_and_set_bit(FORCE_WAKE_ACTIVE_BIT, state->flags)) {
			struct a6_wake_ops* wake_ops = (struct a6_wake_ops*)state->plat_data->wake_ops;

			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR,
				"%s: disabling periodic_wake and switching to force_wake\n",
				__func__);

			// periodic wake active: disable it and switch to force wake
			if (wake_ops->disable_periodic_wake) {
				wake_ops->disable_periodic_wake(wake_ops->data);
			}
			if (wake_ops->force_wake) {
				wake_ops->force_wake(wake_ops->data);
			}

			set_bit(FORCE_WAKE_ACTIVE_BIT, state->flags);
		}
	}
	mutex_unlock(&state->a6_force_wake_mutex);

	msg_itr = &msg[0];
	for (i = num_ids-1; i >= 0; i--) {
		i2c_buf[(i*(2+1))+0] = (uint8_t)(ids[i] >> 8);
		i2c_buf[(i*(2+1))+1] = (uint8_t)(ids[i]);
		i2c_buf[(i*(2+1))+2] = in[i];

		msg_itr->addr = client->addr;
		msg_itr->flags = 0;
		msg_itr->len = 2+1;
		msg_itr->buf = &i2c_buf[i*(2+1)];
#ifdef CONFIG_A6_I2C_SINGLE_BYTE_WRITE
#ifdef A6_PQ
		ret = q_a6_i2c_action_item(client, msg_itr, 1);
#else
		ret = i2c_transfer(client->adapter, msg_itr, 1);
#endif // A6_PQ
		udelay(700);
	      if (ret < 0)
			break;
#endif
		msg_itr++;
	}

#ifndef CONFIG_A6_I2C_SINGLE_BYTE_WRITE
#ifdef A6_PQ
	ret = q_a6_i2c_action_item(client, msg, num_ids);
#else
	ret = i2c_transfer(client->adapter, msg, num_ids);
#endif // A6_PQ
	//msleep(1);
#endif
	if (ret < 0) {
		printk(KERN_ERR "%s[0x%02x]: err code: %d\n", __func__, client->addr, ret);
		// reset the force_wake timer inedependent of i2c failure
		//goto err0;
	}

	if (test_bit(CAP_PERIODIC_WAKE, state->flags)) {
		// [re]start force-wake expiry timer
		mod_timer(&state->a6_force_wake_timer, jiffies+FORCE_WAKE_TIMER_EXPIRY);
	}

err0:
	return ret;
}

int32_t a6_i2c_write_reg(struct i2c_client* client, const uint16_t* ids, uint32_t num_ids, const uint8_t* in)
{
	int32_t ret;
#ifdef A6_I2C_RETRY
	int32_t retry = 5;
#else
	int32_t retry = 0;
#endif

	do {
		ret = __a6_i2c_write_reg(client, ids, num_ids, in);
		if (ret < 0) {
			printk("%s: a6 i2c transaction failed. %s...\n", __func__, retry ? "retry" : " ");
			msleep(30);
		}
	} while (0 != ret && retry-- > 0);

	return ret;
}


uint8_t _convert_hex_char_to_decimal(uint8_t x)
{
	x -= '0';
	if (x > 9) {
		x = x - ('A' - ('9' + 1));
		if (x > 15) {
			x = x - ('a' - 'A');
		}
	}

	return x;
}

int32_t a6_init_state(struct i2c_client *client)
{
	int32_t ret = 0;
	struct a6_device_state* state = i2c_get_clientdata(client);
	struct a6_register_desc* reg_desc;
	uint8_t vals[id_size];

	// early initialization of cached rsense to prevent un-initialized usage
	// due to early-boot i2c failures.
	state->cached_rsense_val = RSENSE_DEFAULT;

	/* (1) enable external/internal wake, if required */
	/* periodic wake capability enabled? */
	if (test_bit(CAP_PERIODIC_WAKE, state->flags)) {
		struct a6_wake_ops* wake_ops = (struct a6_wake_ops*)state->plat_data->wake_ops;

		// initialize timer used to force sleep after a force wake...
		setup_timer(&state->a6_force_wake_timer, a6_force_wake_timer_callback, (ulong)state);

		/* enable external periodic wake? */
		if (wake_ops->enable_periodic_wake) {
			printk(KERN_ERR "%s: enabling external PMIC-generated A6 wake.\n", __func__);

			wake_ops->enable_periodic_wake(wake_ops->data);

			/* disable a6 internal-wake... */
			reg_desc = &a6_register_desc_arr[34];
			/* format wakeup parameters in register format */
			vals[0] = 0;
			ret = a6_i2c_write_reg(client, reg_desc->id, reg_desc->num_ids, vals);
			if (ret < 0) {
				goto err0;
			}
		}
		/* enable internal periodic wake? */
		else if (wake_ops->internal_wake_enable_state) {
			uint32_t wake_period = 0, wake_enable = 1;

			printk(KERN_ERR "%s: enabling A6 internal wake.\n", __func__);
			/* default wake_period, if required */
			if (!wake_ops->internal_wake_period) {
				wake_period = 0x0;
			}
			else {
				wake_period = wake_ops->internal_wake_period(wake_ops->data);
				if (wake_period > 0x100) {
					wake_period = 0x100;
				}
			}

			reg_desc = &a6_register_desc_arr[34];

			if (!wake_period) {
				wake_enable = 0x0;
			}

			// bit [4]: Enable sleep.
			// bit [3]: Enable automatic wakeup.
			// bits [2:0]: Sleep period.
			if (wake_period) {
				wake_period = find_last_bit((const unsigned long*)&wake_period, 32);
				wake_period--;
			}
			vals[0] = 0x10 | wake_enable << 3 | (wake_period & 0x07);
			printk(KERN_ERR "%s: TS2_I2C_WAKEUP_PERIOD = 0x%02x\n",
			       __func__, vals[0]);
			ret = a6_i2c_write_reg(client, reg_desc->id, reg_desc->num_ids, vals);
			if (ret < 0) {
				goto err0;
			}
		}
		else {
			BUG();
		}
	}
	/* periodic wake capability not defined: force a6 awake using SBW_WAKEUP */
	else {
		printk(KERN_ERR "%s: permanently forcing A6 awake.\n", __func__);
		gpio_set_value(state->plat_data->sbw_wkup_gpio, 1);
	}

	/* (2) cache rsense val */
	reg_desc = &a6_register_desc_arr[19];

	memset(vals, 0, sizeof(vals));
	ret = a6_i2c_read_reg(client, reg_desc->id, reg_desc->num_ids, vals);
	if (ret < 0) {
		//goto err0;
	}

	/* rsense == 0: invalid and results in default
	   (logic compatibile with legacy w1 driver implementation) */
	if (vals[0]) {
		state->cached_rsense_val = 1000/vals[0];
		if (!state->cached_rsense_val) {
			state->cached_rsense_val = RSENSE_DEFAULT;
		}
	}
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR,
		   "%s: (cached) rsense value: %u\n", __func__, state->cached_rsense_val);

	/* (3) enable irqs: MASK_A2A_CONNECT_CHANGE, MASK_FLAGS_CHANGE */
	reg_desc = &a6_register_desc_arr[3];

	vals[0] = 0xf3;
	ret = a6_i2c_write_reg(client, reg_desc->id, reg_desc->num_ids, vals);
	if (ret < 0) {
		goto err0;
	}

	/* (4) enable irqs:
	       MASK_BAT_TEMP_HIGH, MASK_BAT_TEMP_LOW,
	       MASK_BAT_VOLT_LOW, MASK_BAT_RARC_CRIT,
	       MASK_BAT_RARC_LOW2, MASK_BAT_RARC_LOW1
	*/
	reg_desc = &a6_register_desc_arr[2];

	vals[0] = 0xc0;
	ret = a6_i2c_write_reg(client, reg_desc->id, reg_desc->num_ids, vals);
	if (ret < 0) {
		goto err0;
	}
	/* read version */
	reg_desc = &a6_register_desc_arr[32];

	memset(vals, 0, sizeof(vals));
	ret = a6_i2c_read_reg(client, reg_desc->id, reg_desc->num_ids, vals);
	if (ret < 0) {
		printk(KERN_ERR "%s: error reading A6 version\n", __func__);
	}
	else {
		uint8_t buf[150];

		reg_desc->format(state, vals, buf, sizeof(buf));
		printk(KERN_INFO "%s", buf);
	}

	// if first init (device boot): clear all interrupt status regs because we might
	// have missed the level-triggered a6-irq during device boot and the user-space
	// components read a6 registers to set their initial state correctly...
	if (!test_bit(IS_INITIALIZED_BIT, state->flags)) {
		vals[0] = 0xff;

		/* clear int_status3 */
		reg_desc = &a6_register_desc_arr[7];
		ret = a6_i2c_write_reg(state->i2c_dev, reg_desc->id, reg_desc->num_ids, vals);
		if (ret < 0) {
			printk(KERN_ERR "%s: error writing reg: %s, id: 0x%x\n",
			__func__, reg_desc->debug_name, reg_desc->id[0]);
			goto err0;
		}

		/* clear int_status2 */
		reg_desc = &a6_register_desc_arr[6];
		ret = a6_i2c_write_reg(state->i2c_dev, reg_desc->id, reg_desc->num_ids, vals);
		if (ret < 0) {
			printk(KERN_ERR "%s: error writing reg: %s, id: 0x%x\n",
			__func__, reg_desc->debug_name, reg_desc->id[0]);
			goto err0;
		}

		/* clear int_status1 */
		reg_desc = &a6_register_desc_arr[5];
		ret = a6_i2c_write_reg(state->i2c_dev, reg_desc->id, reg_desc->num_ids, vals);
		if (ret < 0) {
			printk(KERN_ERR "%s: error writing reg: %s, id: 0x%x\n",
			__func__, reg_desc->debug_name, reg_desc->id[0]);
			goto err0;
		}
	}

	set_bit(IS_INITIALIZED_BIT, state->flags);

err0:
	return ret;
}

int32_t format_current(const struct a6_device_state* state, const uint8_t* val,
		       uint8_t* fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;
	int32_t conv_val =  *(int16_t*)val; // (val[0]<<8) + val[1];

	/* in uA */
	/* TODO: Bootie source code states:
	 *  Define R sense as 20 mOhms, as opposed as a variable and reading it
	 *  from the pack, not all packs have the correct R sense programmed
	 */
	conv_val = (conv_val * 3125)/2/(int32_t)state->cached_rsense_val;

	ret = scnprintf(fmt_buffer, size_buffer, "%d\n", conv_val);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR,
		   "Current value (uA): %s\n", fmt_buffer);
	return ret;
}

int32_t format_voltage(const struct a6_device_state* state, const uint8_t* val,
		       uint8_t* fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;
	int32_t conv_val =  *(int16_t*)val;  // (val[0]<<8) + val[1];

	/* in uV -- 11-bit signed value, unit = 4880uV */
	conv_val = (conv_val>>5) * 4880;
	ret = scnprintf(fmt_buffer, size_buffer, "%d\n", conv_val);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "Voltage value (uV): %s\n",
		fmt_buffer);
	return ret;
}

int32_t r_format_voltage(const struct a6_device_state* state, const uint8_t* fmt_buffer,
			 uint8_t* val, uint32_t size_buffer)
{
	int32_t ret;
	uint32_t conv_val;
	const char *bufp;
	char *endp = NULL;

	bufp = fmt_buffer;
	// let strtoul perform base determination (handles '0x' and '0' prefixes) ...
	conv_val = simple_strtoul(bufp, &endp, 0);
	/* in uV -- 11-bit signed value, unit = 4880uV */
	conv_val = (conv_val / 4880) << 5;
	// capped at 16 bits...
	if (conv_val >> 0x10) {
		ret = 0;
		goto err0;
	}
	ret = (uint32_t)endp - (uint32_t)bufp;

	*(uint16_t*)val = conv_val;

	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: ip: %s op: %d\n",
		   __func__, fmt_buffer, conv_val);

err0:
	return ret;
}
/**
 * Take the Accumulated Current Register (ACR), which is expressed in units of 6.25uVh
 * and convert to uAh.
 */
int32_t format_rawcoulomb(const struct a6_device_state* state, const uint8_t* val,
			  uint8_t* fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;
	int32_t conv_val =  *(int16_t*)val; // (val[0]<<8) + val[1];

	// in uAh
	conv_val = (conv_val * 6250) / (int32_t)state->cached_rsense_val;
	ret = scnprintf(fmt_buffer, size_buffer, "%d.%03d\n", conv_val/1000,
		       ((conv_val >= 0)? conv_val : -conv_val) % 1000);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "Raw Coulomb value (uAh): %s\n",
		fmt_buffer);
	return ret;
}

/**
 * Take the Remaining Active Absolute Capacity (RAAC) register, which is in
 * units of 1.6 mAh and convert to mAh
 */
int32_t format_coulomb(const struct a6_device_state* state, const uint8_t* val,
			  uint8_t* fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;
	int32_t conv_val =  *(int16_t*)val;

	// in mAh
	conv_val = (conv_val * 8) / 5;
	ret = scnprintf(fmt_buffer, size_buffer, "%d\n", conv_val);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "Coulomb value (mAh): %s\n",
		fmt_buffer);
	return ret;
}

int32_t format_age(const struct a6_device_state* state, const uint8_t* val,
		   uint8_t* fmt_buffer, uint32_t size_buffer)
{
	int32_t ret, conv_val;

	// Age conversion factor, in .01 percent. Raw value 0x80 = 100%
	conv_val = (10000*val[0]) >> 7;
	ret = scnprintf(fmt_buffer, size_buffer, "%d.%02d\n",
		conv_val/100, conv_val%100);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "Battery life left (%%): %s\n",
		fmt_buffer);
	return ret;
}

int32_t format_fullx(const struct a6_device_state* state, const uint8_t* val,
		     uint8_t* fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;
	int32_t conv_val =  *(int16_t*)val; // (val[0]<<8) + val[1];

	// Convert 6.25uVh to uAh
	conv_val = (conv_val * 6250) / state->cached_rsense_val;
	ret = scnprintf(fmt_buffer, size_buffer,
		"%d.%03d\n", conv_val/1000, conv_val%1000);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "Battery life left (uAh): %s\n",
		fmt_buffer);
	return ret;
}

int32_t format_temp(const struct a6_device_state* state, const uint8_t* val,
		    uint8_t* fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;
	int conv_val = val[1];	/* Ignoring the fraction part */

	// in C
	ret = snprintf(fmt_buffer, size_buffer, "%d\n", (int8_t)conv_val);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "Temperature (C): %s\n",
		fmt_buffer);
	return ret;
}

int32_t format_command(const struct a6_device_state* state, const uint8_t* val,
		    uint8_t* fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;
	int conv_command_debug_code = val[0];

	ret = snprintf(fmt_buffer, size_buffer, "%d\n", (int8_t)conv_command_debug_code);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "debug code(C): %s\n",
		fmt_buffer);
	return ret;
}

int32_t format_accessory(const struct a6_device_state* state, const uint8_t* val,
		    uint8_t* fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;

    ret = scnprintf(fmt_buffer, size_buffer, "%02X\n", val[0]);
 	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "%s", fmt_buffer);
	return ret;
}

int32_t format_comm_status(const struct a6_device_state* state, const uint8_t* val,
		    uint8_t* fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;

    ret = scnprintf(fmt_buffer, size_buffer, "%02X\n", val[0]);
 	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "%s", fmt_buffer);
	return ret;
}

int32_t r_format_temp(const struct a6_device_state* state, const uint8_t* fmt_buffer,
		      uint8_t* val, uint32_t size_buffer)
{
	int32_t ret;
	int32_t conv_val;
	const char *bufp;
	char *endp = NULL;

	bufp = fmt_buffer;
	// let strtoul perform base determination (handles '0x' and '0' prefixes) ...
	conv_val = simple_strtol(bufp, &endp, 0);
	printk(KERN_ERR "conv_val : %d, hex: %x\n", conv_val, conv_val);
	// capped at 8 bits...
	if (conv_val < -128) {
		ret = 0;
		goto err0;
	}
	ret = (uint32_t)endp - (uint32_t)bufp;

	/* Ignoring the fraction part */
	*(uint16_t*)val = (uint8_t)conv_val << 8;

	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: ip: %s op: %d\n",
		   __func__, fmt_buffer, conv_val<<8);

err0:
	return ret;
}

int32_t format_status(const struct a6_device_state* state, const uint8_t* val,
		      uint8_t* fmt_buffer, uint32_t size_buffer)
{
	uint32_t conv_val = *val;
	int32_t count, i;

	const char* status_bits[] = {
		NULL,
		"power-on-reset",
		"undervoltage",
		NULL,
		"learn",
		"standby-empty",
		"active-empty",
		"charge-termination"};

	for (i = count = 0; i < sizeof(status_bits)/sizeof(status_bits[0]); i++) {
		if ((conv_val & (1 << i)) && status_bits[i] != NULL) {
			count += scnprintf(fmt_buffer + count, size_buffer-count, "%s\n", status_bits[i]);
		}
	}
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR,
		   "Battery status: %s\n", fmt_buffer);

	return count;
}

int32_t format_rsense(const struct a6_device_state* state, const uint8_t* val,
		      uint8_t* fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;
	uint32_t conv_val = *val;

	ret = scnprintf(fmt_buffer, size_buffer, "%u\n", (!conv_val) ? conv_val : 1000/conv_val);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "rsense (ohm): %s\n",
		fmt_buffer);
	return ret;
}

/**
 * Determine what, if any chargers are attached to the system.  Only one
 * result will be returned, even if multiple chargers are detected.
 *
 * Output is a combination of:
 *   0x01 - Puck Detected
 *   0x02 - Puck Connected
 *   0x04 - Wired Power Detected
 *   0x08 - Puck Power Connected
 *   0x10 - Wired Power Connected
 *   0x20 - Battery Present
 *
 * Not all values are currently supported.
 */
int32_t format_charge_source_status(const struct a6_device_state* state, const uint8_t* val,
				    uint8_t* fmt_buffer, uint32_t size_buffer)
{
	struct charge_source_status_map {
		uint32_t cs_mask;
		const char* cs_name;
	} cs_map[] = {
		[0] = {0x01, "Puck Detected"},
		[1] = {0x02, "Puck Connected"},
		[2] = {0x04, "Wired Power Detected"},
		[3] = {0x08, "Puck Power Connected"},
		[4] = {0x10, "Wired Power Connected"},
		[5] = {0x20, "Battery Present"},
	};
	uint32_t conv_val = *val, count = 0;
	int32_t i;

	for(i = 0; i < sizeof(cs_map)/sizeof(cs_map[0]); i++) {
		if (conv_val & cs_map[i].cs_mask) {
			count += scnprintf(fmt_buffer + count, (size_buffer - 1) - count, "%s\n", cs_map[i].cs_name);
		}
	}

	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR,
		   "charge source status (formatted): %s\n", fmt_buffer);

#ifdef A6_REPORT_CONNECTED_ONLY
	conv_val = 0;
	/* map charge source status to user-space values */
	/* TODO: differentiate wall charger from USB */
	if (val[0] & TS2_I2C_FLAGS_2_WIRED_CHARGE) conv_val |= 1;
	if (val[0] & TS2_I2C_FLAGS_2_PUCK_CHARGE) conv_val |= 4;
#endif

	/* output register contents */
	count = scnprintf(fmt_buffer, size_buffer, "%u\n", conv_val);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR,
		   "charge source status (literal): %s\n", fmt_buffer);
	return count;
}

int32_t format_version(const struct a6_device_state* state, const uint8_t* val,
		       uint8_t* fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;

	ret = scnprintf(fmt_buffer, size_buffer,
			"A6 Version: HW: %d, FW (M.m.B): %d.%d.%d, ManID: %d, ProdTyp: %d\n",
			val[12], val[15], val[14], val[13],
			*(int16_t*)&(val[0]), *(int16_t*)&(val[2]));
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "%s", fmt_buffer);
	return ret;
}

int32_t format_v_offset(const struct a6_device_state* state, const uint8_t* val,
			uint8_t* fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;
	uint32_t conv_val = *val;

	ret = scnprintf(fmt_buffer, size_buffer, "%u\n", conv_val);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "v_offset: %s\n", fmt_buffer);
	return ret;
}

int32_t format_raw_unsigned(const struct a6_device_state* state, const uint8_t* val,
			    uint8_t* fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;
	uint32_t conv_val = *val;

	ret = scnprintf(fmt_buffer, size_buffer, "%u\n", conv_val);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "raw_unsigned: %s\n", fmt_buffer);
	return ret;
}

int32_t format_u16_hex(const struct a6_device_state* state, const uint8_t* val,
		       uint8_t* fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;
	uint16_t conv_val = *(uint16_t*)val;

	ret = scnprintf(fmt_buffer, size_buffer, "0x%04hx\n", conv_val);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "u16_hex: %s\n", fmt_buffer);
	return ret;
}

int32_t format_max_power_available(const struct a6_device_state* state, const uint8_t* val,
				   uint8_t* fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;
	uint32_t vnode_max_val = *(uint16_t*)val;
	uint32_t inode_max_val = *(uint16_t*)(val+2);
	uint32_t power_val     = *(val+4);
	uint32_t conv_val;

	conv_val = (vnode_max_val * inode_max_val * power_val)/2560;
	ret = scnprintf(fmt_buffer, size_buffer, "%u\n", conv_val);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "%s", fmt_buffer);
	return ret;
}

int32_t format_accessory_combo(const struct a6_device_state* state, const uint8_t* val,
			       uint8_t* fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;

	ret = scnprintf(fmt_buffer, size_buffer,
			"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x "
			"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x \n",
			val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7],
			val[8], val[9], val[10], val[11], val[12], val[13], val[14], val[15]);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "%s", fmt_buffer);
	return ret;
}

int32_t format_serno_v1(const struct a6_device_state* state, const uint8_t* val,
			       uint8_t* fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;

	ret = scnprintf(fmt_buffer, size_buffer, "%02x%02x%02x%02x%02x%02x\n",
			val[0], val[1], val[2], val[3], val[4], val[5]);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "%s", fmt_buffer);
	return ret;
}

int32_t format_serno_v2(const struct a6_device_state* state, const uint8_t* val,
			uint8_t* fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;

	ret = scnprintf(fmt_buffer, size_buffer, "%02x%02x%02x%02x%02x%02x%02x%02x\n",
			val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7]);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "%s", fmt_buffer);
	return ret;
}

static ssize_t a6_generic_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int32_t ret = 0, i = 0;
	uint8_t vals[id_size];
	struct a6_register_desc* reg_desc;
	struct a6_device_state* state = i2c_get_clientdata(client);

	// critsec for manipulating flags
	ret = mutex_lock_interruptible(&state->dev_mutex);
	if (ret) {
		printk(KERN_ERR "%s: mutex_lock_interruptible interrupted(1)\n", __func__);
		return -ERESTARTSYS;
	}

	// are we busy?
	while (test_and_set_bit(DEVICE_BUSY_BIT, state->flags)) {
		// yes: are we in bootload phase?
		if (!test_bit(BOOTLOAD_ACTIVE_BIT, state->flags)) {
			// no: so go ahead and allow concurrent i2c ops (these are
			// synchronized separately to allow priority based execution).
			break;
		}

		// in bootload phase: get on a waitq
		mutex_unlock(&state->dev_mutex);
		A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: about to wait for device non-busy...\n", __func__);

		// bootload bit set? wait to be cleared (at least 5 mins: in jiffies)
		ret = wait_event_interruptible_timeout(state->dev_busyq,
				!test_bit(BOOTLOAD_ACTIVE_BIT, state->flags), 300*HZ);
		if (!ret) {
			printk(KERN_ERR "%s: wait on device busy timed-out/interrupted\n", __func__);
			// reset busy state
			clear_bit(DEVICE_BUSY_BIT, state->flags);
			// and continue...
		}

		// we're about to manipulate flags again: acquire critsec
		ret = mutex_lock_interruptible(&state->dev_mutex);
		if (ret) {
			printk(KERN_ERR "%s: mutex_lock_interruptible interrupted(2)\n", __func__);
			return -ERESTARTSYS;
		}
	}

	// increment busy refcount
	state->busy_count++;
	// done with flags: exit critsec
	mutex_unlock(&state->dev_mutex);

	reg_desc = container_of(attr, struct a6_register_desc, dev_attr);

	memset(vals, 0, sizeof(vals));
	ret = a6_i2c_read_reg(client, reg_desc->id, reg_desc->num_ids, vals);
	if (ret < 0) {
		goto err0;
	}

	if (reg_desc->format) {
		ret = reg_desc->format(state, vals, buf, PAGE_SIZE);
	}
	else {
		/* if output restricted to 2 8-bit values, show as int16_t
		   (common case of msb+lsb retrieval) */
		if (2 == reg_desc->num_ids) {
			ret = scnprintf(buf, PAGE_SIZE, "%d\n", *(int16_t*)vals);
		}
		/* if output > or < 2 8-bit values, show as
		 * space-delimited list of int8_t values
		 */
		else {
			i = ret = 0;
			while(i < reg_desc->num_ids) {
				ret += scnprintf(buf+ret, PAGE_SIZE-ret, "%d ", (int8_t)vals[i]);
				i++;
			}
			ret += scnprintf(buf+ret, PAGE_SIZE-ret, "\n");
		}

		A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "Buffer: %s\n", buf);
	}

#ifdef A6_DEBUG
	{
		uint8_t d_ids[reg_desc->num_ids * (4+2+1) + 1];
		uint8_t d_vals[reg_desc->num_ids * (2+2+1) + 1];
		int32_t i = 0, ret_ids = 0, ret_vals = 0;

		while (i < reg_desc->num_ids) {
			ret_ids += sprintf(d_ids+ret_ids, "0x%04x ", reg_desc->id[i]);
			ret_vals += sprintf(d_vals+ret_vals, "0x%02x ", vals[i]);
			i++;
		}

		A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO,
			"showing reg name: %s, num_ids: %d, ids: %s, vals: %s\n",
			reg_desc->debug_name, reg_desc->num_ids, d_ids, d_vals);
	}
#endif

err0:
	// reset busy state
	mutex_lock(&state->dev_mutex);
	// decrement busy refcount
	if (state->busy_count) {
		state->busy_count--;
	}
	if (!state->busy_count) {
		clear_bit(DEVICE_BUSY_BIT, state->flags);
	}
	mutex_unlock(&state->dev_mutex);
	wake_up_interruptible(&state->dev_busyq);

	return ret;
}

// constraints:
// * multi-valued stores must have individual values delimited by whitespace
// * multi-valued stores have individual values constrained to reg size (8-bit)
// * single-valued stores can specify 16-bit values to update two regs (msb+lsb)
// * value count must exactly match for multi-values stores
// * value-count for single value stores may be one less than the reg count specified.
static ssize_t a6_generic_store(struct device *dev, struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int32_t ret = 0, num_ids, val_cnt, i;
	uint32_t val;
	char *endp = NULL, *bufp;
	uint16_t parsed_vals[id_size];
	uint8_t in_vals[id_size];
	struct a6_register_desc* reg_desc;
	struct a6_device_state* state = i2c_get_clientdata(client);

	// critsec for manipulating flags
	ret = mutex_lock_interruptible(&state->dev_mutex);
	if (ret) {
		printk(KERN_ERR "%s: mutex_lock_interruptible interrupted(1)\n", __func__);
		return -ERESTARTSYS;
	}

	// are we busy?
	while (test_and_set_bit(DEVICE_BUSY_BIT, state->flags)) {
		// yes: are we in bootload phase?
		if (!test_bit(BOOTLOAD_ACTIVE_BIT, state->flags)) {
			// no: so go ahead and allow concurrent i2c ops (these are
			// synchronized separately to allow priority based execution).
			break;
		}

		// in bootload phase: get on a waitq
		mutex_unlock(&state->dev_mutex);
		A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: about to wait for device non-busy...\n", __func__);

		// bootload bit set? wait to be cleared (at least 5 mins: in jiffies)
		ret = wait_event_interruptible_timeout(state->dev_busyq,
				!test_bit(BOOTLOAD_ACTIVE_BIT, state->flags), 300*HZ);
		if (!ret) {
			printk(KERN_ERR "%s: wait on device busy timed-out/interrupted\n", __func__);
			// reset busy state
			clear_bit(DEVICE_BUSY_BIT, state->flags);
			// and continue...
		}

		// we're about to manipulate flags again: acquire critsec
		ret = mutex_lock_interruptible(&state->dev_mutex);
		if (ret) {
			printk(KERN_ERR "%s: mutex_lock_interruptible interrupted(2)\n", __func__);
			return -ERESTARTSYS;
		}
	}

	// increment busy refcount
	state->busy_count++;
	// done with flags: exit critsec
	mutex_unlock(&state->dev_mutex);

	reg_desc = container_of(attr, struct a6_register_desc, dev_attr);
	num_ids = reg_desc->num_ids;

	if (reg_desc->r_format)
	{
		ret = reg_desc->r_format(state, buf, in_vals, count);
	}
	else
	{
		bufp = (char*)buf;
		val_cnt = 0;
		do {
			// let strtoul perform base determination (handles '0x' and '0' prefixes) ...
			val = (uint16_t)simple_strtoul(bufp, &endp, 0);
			// each space-delimited write unit is capped at 16 bits...
			if (val > 0xffff) {
				ret = -EINVAL;
				goto err0;
			}

			parsed_vals[val_cnt] = val;

			// skip whitespace
			while (*endp && isspace(*endp)) {
				endp++;
			}
			// reset for next iteration
			bufp = endp;
		} while ((++val_cnt < num_ids) && ((endp - buf) < count));

		// three levels of value count calidation:
		// * if extra values: fail
		// * if multiple values and does not match reg count specified: fail
		// * if single value and reg count > 2: fail
		if (*endp) {
			ret = -EINVAL;
			goto err0;
		}
		else if (val_cnt > 1) {
			if (val_cnt != num_ids) {
				ret = -EINVAL;
				goto err0;
			}
		}
		else {
			if (num_ids > 2) {
				ret = -EINVAL;
				goto err0;
			}
		}

		// if multi-valued store or single-valued store to one reg: we constrain each
		// value to reg size (8 bits)
		if ((val_cnt > 1) || (1 == num_ids)) {
			for (i = 0; i < val_cnt; i++) {
				if (parsed_vals[i] > 0xff) {
					ret = -EINVAL;
					goto err0;
				}

				in_vals[i] = parsed_vals[i];
			}
		}
		// single-valued store can be 8-bit or 16-bit (for msb+lsb)
		else {
			((uint16_t*)in_vals)[0] = ((uint16_t*)parsed_vals)[0];
		}
	}

#ifdef A6_DEBUG
	{
		uint8_t d_ids[reg_desc->num_ids * (4+2+1)];
		uint8_t d_vals[reg_desc->num_ids * (2+2+1)];
		int32_t i = 0, ret_ids = 0, ret_vals = 0;

		while (i < reg_desc->num_ids) {
			ret_ids = sprintf(d_ids+ret_ids, "0x%04x ", reg_desc->id[i]);
			ret_vals = sprintf(d_vals+ret_vals, "0x%02x ", in_vals[i]);
			i++;
		}

		A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR,
			"storing reg name: %s, num_ids: %d, ids: %s, vals: %s\n",
			reg_desc->debug_name, reg_desc->num_ids, d_ids, d_vals);
	}
#endif

	ret = a6_i2c_write_reg(client, reg_desc->id, num_ids, in_vals);
	if (ret < 0) {
		goto err0;
	}

	ret = count;

err0:
	// reset busy state
	mutex_lock(&state->dev_mutex);
	// decrement busy refcount
	if (state->busy_count) {
		state->busy_count--;
	}
	if (!state->busy_count) {
		clear_bit(DEVICE_BUSY_BIT, state->flags);
	}
	mutex_unlock(&state->dev_mutex);
	wake_up_interruptible(&state->dev_busyq);

	return ret;
}

enum {
	ACTIVATE_EXTRACT,
	NONE
};

static ssize_t a6_val_cksum_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int32_t rc = 0, reloop = 0, failed = 0;
	struct a6_device_state* state = i2c_get_clientdata(client);
	uint16_t cksum1, cksum2, cksum_cycles, cksum_errors;
	struct a6_register_desc *reg_desc;
	uint8_t vals[id_size];


	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: enter\n", __func__);

	// critsec for manipulating flags
	rc = mutex_lock_interruptible(&state->dev_mutex);
	if (rc) {
		printk(KERN_ERR "%s: mutex_lock interrupted\n", __func__);
		return -EIO;
	}

	// are we busy?
	while (test_and_set_bit(DEVICE_BUSY_BIT, state->flags)) {
		// yes: get on a waitq
		mutex_unlock(&state->dev_mutex);

		A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: about to wait for device non-busy...\n", __func__);

		// busy bit set? wait to be cleared (at least 5 minutes: in jiffies)
		rc = wait_event_interruptible_timeout(state->dev_busyq,
				!test_bit(DEVICE_BUSY_BIT, state->flags), 300*HZ);
		if (!rc) {
			printk(KERN_ERR "%s: wait on device busy timed-out/interrupted\n", __func__);
			// reset busy state
			clear_bit(DEVICE_BUSY_BIT, state->flags);
			// and continue...
		}

		// we're about to manipulate flags again: acquire critsec
		rc = mutex_lock_interruptible(&state->dev_mutex);
		if (rc) {
			printk(KERN_ERR "%s: mutex_lock interrupted(2)\n", __func__);
			goto err0;
		}
	}

	rc = test_and_set_bit(BOOTLOAD_ACTIVE_BIT, state->flags);
	ASSERT(!rc);

	// we're done with flags: exit critsec
	mutex_unlock(&state->dev_mutex);

	do {
		/* write command (cksum1) */
		reg_desc = &a6_register_desc_arr[35];
		vals[0] = TS2_I2C_COMMAND_FRAM_CHECKSUM_READ_1;
		rc = a6_i2c_write_reg(state->i2c_dev, reg_desc->id, reg_desc->num_ids, vals);
		if (rc < 0) {
			printk(KERN_ERR "%s: error writing reg: %s, id: 0x%x\n",
			__func__, reg_desc->debug_name, reg_desc->id[0]);
			break;
		}

		/* read cksum1 */
		reg_desc = &a6_register_desc_arr[80];
		memset(vals, 0, sizeof(vals));
		rc = a6_i2c_read_reg(state->i2c_dev, reg_desc->id, reg_desc->num_ids, vals);
		if (rc < 0) {
			printk(KERN_ERR "%s: error reading reg: %s, ids: 0x%x 0x%x\n",
			__func__, reg_desc->debug_name, reg_desc->id[0], reg_desc->id[1]);
			break;
		}
		cksum1 = vals[0] | vals[1] << 8;

		/* read cksum cycle counter */
		reg_desc = &a6_register_desc_arr[81];
		memset(vals, 0, sizeof(vals));
		rc = a6_i2c_read_reg(state->i2c_dev, reg_desc->id, reg_desc->num_ids, vals);
		if (rc < 0) {
			printk(KERN_ERR "%s: error reading reg: %s, ids: 0x%x 0x%x\n",
			__func__, reg_desc->debug_name, reg_desc->id[0], reg_desc->id[1]);
			break;
		}
		cksum_cycles = vals[0] | vals[1] << 8;

		/* write command (cksum2) */
		reg_desc = &a6_register_desc_arr[35];
		vals[0] = TS2_I2C_COMMAND_FRAM_CHECKSUM_READ_2;
		rc = a6_i2c_write_reg(state->i2c_dev, reg_desc->id, reg_desc->num_ids, vals);
		if (rc < 0) {
			printk(KERN_ERR "%s: error writing reg: %s, id: 0x%x\n",
			__func__, reg_desc->debug_name, reg_desc->id[0]);
			break;
		}

		/* read cksum2 */
		reg_desc = &a6_register_desc_arr[80];
		memset(vals, 0, sizeof(vals));
		rc = a6_i2c_read_reg(state->i2c_dev, reg_desc->id, reg_desc->num_ids, vals);
		if (rc < 0) {
			printk(KERN_ERR "%s: error reading reg: %s, ids: 0x%x 0x%x\n",
			__func__, reg_desc->debug_name, reg_desc->id[0], reg_desc->id[1]);
			break;
		}
		cksum2 = vals[0] | vals[1] << 8;

		/* read cksum error counter */
		reg_desc = &a6_register_desc_arr[81];
		memset(vals, 0, sizeof(vals));
		rc = a6_i2c_read_reg(state->i2c_dev, reg_desc->id, reg_desc->num_ids, vals);
		if (rc < 0) {
			printk(KERN_ERR "%s: error reading reg: %s, ids: 0x%x 0x%x\n",
			       __func__, reg_desc->debug_name, reg_desc->id[0], reg_desc->id[1]);
			break;
		}
		cksum_errors = vals[0] | vals[1] << 8;

		/* validate cksum */
		if (reloop || !cksum1 || !cksum2 || (cksum1 != cksum2)) {
			printk(KERN_ERR "A6 checksum (%s) validation failure:\n"
					"cksum1: 0x%02hx; cksum2: 0x%02hx; cksum_cycles: 0x%02hx;"
					" cksum_errors: 0x%02hx\n",
					(!reloop) ? "first-stage" : "second-stage",
					cksum1, cksum2, cksum_cycles, cksum_errors);
			if (reloop) {
				reloop--;
				failed = 1;
			}
			else {
				reloop++;
			}
		}
	} while (reloop);

	if (rc < 0) {
		printk(KERN_ERR "%s: checksum retrieval enountered i2c errors: "
				"fallback to sbw(jtag) access.\n", __func__);
		get_checksum_data_sbw((struct a6_sbw_interface*)state->plat_data->sbw_ops, &cksum1,
				       &cksum2, &cksum_cycles, &cksum_errors);
	}

	/* validate cksum */
	if (failed || !cksum1 || !cksum2 || (cksum1 != cksum2)) {
		printk(KERN_ERR "A6 checksum validation failed:\n"
				"cksum1: 0x%02hx; cksum2: 0x%02hx; cksum_cycles: 0x%02hx;"
				" cksum_errors: 0x%02hx\n",
				cksum1, cksum2, cksum_cycles, cksum_errors);
		rc = snprintf(buf, PAGE_SIZE, "%d\n", 0);
	}
	else {
		rc = snprintf(buf, PAGE_SIZE, "%d\n", 1);
	}

err0:
	mutex_lock(&state->dev_mutex);
	clear_bit(BOOTLOAD_ACTIVE_BIT, state->flags);
	clear_bit(DEVICE_BUSY_BIT, state->flags);
	mutex_unlock(&state->dev_mutex);
	wake_up_interruptible(&state->dev_busyq);

	return rc;
}


static char* activation_strlist[] = {
	[ACTIVATE_EXTRACT] = "extract",
	[NONE] = "none"
};

static ssize_t a6_diag_store(struct device *dev, struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int32_t rc = 0, ret_val, action_i = 0;
	struct a6_device_state* state = i2c_get_clientdata(client);
	bool skip = false;

	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: enter\n", __func__);

	action_i = (sizeof(activation_strlist)/sizeof(char*));
	do {
		if (0 == strncmp(buf, activation_strlist[action_i-1],
		    strlen(activation_strlist[action_i-1]))) {
			break;
		}
	} while(action_i--);

	if (!action_i) {
		return -EINVAL;
	}

	// store the action index
	action_i--;

	// critsec for manipulating flags
	rc = mutex_lock_interruptible(&state->dev_mutex);
	if (rc) {
		printk(KERN_ERR "%s: mutex_lock interrupted\n", __func__);
		return -EIO;
	}

	// are we busy?
	while (test_and_set_bit(DEVICE_BUSY_BIT, state->flags)) {
		// yes: get on a waitq
		mutex_unlock(&state->dev_mutex);

		A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: about to wait for device non-busy...\n", __func__);

		// busy bit set? wait to be cleared (at least 5 minutes: in jiffies)
		rc = wait_event_interruptible_timeout(state->dev_busyq,
				!test_bit(DEVICE_BUSY_BIT, state->flags), 300*HZ);
		if (!rc) {
			printk(KERN_ERR "%s: wait on device busy timed-out/interrupted\n", __func__);
			// reset busy state
			clear_bit(DEVICE_BUSY_BIT, state->flags);
			// and continue...
		}

		// we're about to manipulate flags again: acquire critsec
		rc = mutex_lock_interruptible(&state->dev_mutex);
		if (rc) {
			printk(KERN_ERR "%s: mutex_lock interrupted(2)\n", __func__);
			goto err0;
		}
	}

	if (ACTIVATE_EXTRACT == action_i) {
		if (test_bit(EXTRACT_INITIATED, state->flags)) {
			skip = true;
		}
		else {
			ret_val = test_and_set_bit(BOOTLOAD_ACTIVE_BIT, state->flags);
			ASSERT(!ret_val);
			ret_val = test_and_set_bit(EXTRACT_INITIATED, state->flags);
			ASSERT(!ret_val);
		}
	}
	else if (NONE == action_i) {
		if (!(test_bit(EXTRACT_INITIATED, state->flags))) {
			skip = true;
		}
	}
	else {
		// invalid
	}

	// we're done with flags: exit critsec
	mutex_unlock(&state->dev_mutex);

	if (true == skip) goto err0;
	
	if (ACTIVATE_EXTRACT == action_i) {
		// reset force-wake state to always force wake on first i2c txn
		// after pmem extraction
		del_timer(&state->a6_force_wake_timer);
		mutex_lock(&state->a6_force_wake_mutex);
		if (test_bit(CAP_PERIODIC_WAKE, state->flags)) {
			clear_bit(FORCE_WAKE_ACTIVE_BIT, state->flags);
		}
		mutex_unlock(&state->a6_force_wake_mutex);

		// start pmem extract
		printk(KERN_ERR "%s: starting ttf_extract.\n", __func__);
		rc = ttf_extract_fw_sbw((struct a6_sbw_interface*)state->plat_data->sbw_ops);
		if (rc) {
			printk(KERN_ERR "Failed to ttf_extract a6 pmem.\n");
			goto err0;
		}
		else {	
			printk(KERN_ERR "A6: Completed ttf_extract a6 pmem.\n");
			// wait for the A6 to boot...
			msleep(3000);
			// - re-init state
			// - if init fails: ignore
			a6_init_state(state->i2c_dev);
		}
	}
	else if (NONE == action_i) {
		printk(KERN_ERR "%s: clearing ttf_extract cache.\n", __func__);
		rc = ttf_extract_cache_clear();
	}
	else {
		// invalid
	}

err0:
	mutex_lock(&state->dev_mutex);
	if (false == skip) {
		if (rc || NONE == action_i) {
			if (test_bit(EXTRACT_INITIATED, state->flags)) {
				clear_bit(EXTRACT_INITIATED, state->flags);
			}
		}
		if (test_bit(BOOTLOAD_ACTIVE_BIT, state->flags)) {
			clear_bit(BOOTLOAD_ACTIVE_BIT, state->flags);
		}
	}
	clear_bit(DEVICE_BUSY_BIT, state->flags);
	mutex_unlock(&state->dev_mutex);
	wake_up_interruptible(&state->dev_busyq);

	return (rc ? rc : count);
}

static ssize_t a6_diag_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int32_t rc = 0;
	struct a6_device_state* state = i2c_get_clientdata(client);

	
	mutex_lock(&state->dev_mutex);
	if (test_bit(EXTRACT_INITIATED, state->flags)) {
		rc = snprintf(buf, PAGE_SIZE, "%s\n", activation_strlist[ACTIVATE_EXTRACT]);
	}
	mutex_unlock(&state->dev_mutex);

	return rc;
}

int32_t a6_create_dev_files(struct a6_device_state* state, struct device* dev)
{
	int32_t rc = 0, reg_cnt = sizeof(a6_register_desc_arr)/sizeof(struct a6_register_desc);
	int32_t idx = 0, cust_idx = 0;

	for (idx = 0; idx < reg_cnt; idx++) {
		rc = device_create_file(dev, &a6_register_desc_arr[idx].dev_attr);
		if (rc < 0) {
			printk(KERN_ERR "%s: failed to create dev_attr file for %s.\n",
			       __func__, a6_register_desc_arr[idx].dev_attr.attr.name);
			goto err0;
		}
	}

	reg_cnt = sizeof(custom_devattr)/sizeof(struct device_attribute);
	for (cust_idx = 0; cust_idx < reg_cnt; cust_idx++) {
		rc = device_create_file(dev, &custom_devattr[cust_idx]);
		if (rc < 0) {
			printk(KERN_ERR "%s: failed to create dev_attr file for %s.\n",
			       __func__, custom_devattr[cust_idx].attr.name);
			goto err0;
		}
	}

	rc = sysfs_create_link(&state->mdev.this_device->kobj, &dev->kobj, "regs");
	if (rc) {
		printk(KERN_ERR "%s: error in creating symlink.\n", __func__);
	}

	return 0;

err0:
	// error: cleanup files already created.
	for (idx--; idx >= 0; idx--) {
		device_remove_file(dev, &a6_register_desc_arr[idx].dev_attr);
	}
	for (cust_idx--; cust_idx >= 0; cust_idx--) {
		device_remove_file(dev, &custom_devattr[cust_idx]);
	}
	return rc;

}

void a6_remove_dev_files(struct a6_device_state* state, struct device* dev)
{
	int32_t reg_cnt = sizeof(a6_register_desc_arr)/sizeof(struct a6_register_desc);
	int32_t idx;

	for (idx = 0; idx < reg_cnt; idx++) {
		device_remove_file(dev, &a6_register_desc_arr[idx].dev_attr);
	}

	reg_cnt = sizeof(custom_devattr)/sizeof(struct device_attribute);
	for (idx = 0; idx < reg_cnt; idx++) {
		device_remove_file(dev, &custom_devattr[idx]);
	}
}

typedef enum {
	A6_PROGAM_AND_VERIFY_FW = 1,
	A6_VERIFY_FW
} a6_pgm_thread_op;

struct a6_pgm_thread_params {
	struct a6_sbw_interface* sbw_ops;
	uint32_t buffer_p;
	int32_t ret_code;
	a6_pgm_thread_op op;
	struct completion a6_flash_thread_nice;
	struct completion a6_flash_thread_exit;
};

int a6_pgm_thread_fn(void* param)
{
	int32_t ret_val;

	struct a6_pgm_thread_params* t_p = (struct a6_pgm_thread_params*)param;

	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: buffer_p val: 0x%x\n", __func__, t_p->buffer_p);

	// wait for parent re-nice completion...
	ret_val = wait_for_completion_interruptible(&t_p->a6_flash_thread_nice);
	if (-ERESTARTSYS == ret_val) {
		printk(KERN_ERR "A6: waiting for parent re-nice completion interrupted.\n");
		t_p->ret_code = -EINTR;
		goto err0;
	}

	if (A6_PROGAM_AND_VERIFY_FW == t_p->op) {
		ret_val = program_device_sbw(t_p->sbw_ops, t_p->buffer_p);
	}
	else {
		ret_val = verify_device_sbw(t_p->sbw_ops, t_p->buffer_p);
		// ignore error returns for the moment as the verification fn
		// does not differentiate between code and r/w data sections.
		// also, the fw section allocation changes fairly dynamically
		// between fw drops and we don't yet support parameterizing the
		// section map for the verification code.
		ret_val = 0;
	}
	if (ret_val) {
		t_p->ret_code = -EINVAL;
	}

	// signal thread completion
	complete(&t_p->a6_flash_thread_exit);

err0:
	return 0;
}

static int a6_ioctl(struct inode * inode, struct file *file,
                         unsigned int cmd, unsigned long args)
{
	int32_t rc = 0;
	struct a6_device_state* state = file->private_data;
	void* usr_ptr   = (void*)args;
	//uint32_t usr_bytes = _IOC_SIZE(cmd);
	//uint32_t usr_val = 0x0;

	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: cmd: %d\n", __func__, _IOC_NR(cmd));

	// critsec for manipulating flags
	rc = mutex_lock_interruptible(&state->dev_mutex);
	if (rc) {
		printk(KERN_ERR "%s: mutex_lock interrupted(1)\n", __func__);
		return -ERESTARTSYS;
	}

	// are we busy?
	while (test_and_set_bit(DEVICE_BUSY_BIT, state->flags)) {
		// yes: get on a waitq
		mutex_unlock(&state->dev_mutex);

		A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: about to wait for device non-busy...\n", __func__);

		// busy bit set? wait to be cleared (at least 3 seconds: in jiffies)
		rc = wait_event_interruptible_timeout(state->dev_busyq,
		                                      !test_bit(DEVICE_BUSY_BIT, state->flags), 3*HZ);
		if (!rc) {
			printk(KERN_ERR "%s: wait on device busy timed-out/interrupted\n", __func__);
			// reset busy state
			clear_bit(DEVICE_BUSY_BIT, state->flags);
			// and continue...
		}

		// we're about to manipulate flags again: acquire critsec
		rc = mutex_lock_interruptible(&state->dev_mutex);
		if (rc) {
			printk(KERN_ERR "%s: mutex_lock interrupted(2)\n", __func__);
			return -ERESTARTSYS;
		}
	}

	// bootload request: set flag in critsec
	if (A6_IOCTL_SET_FW_DATA == cmd || A6_IOCTL_VERIFY_FW_DATA == cmd) {
		int32_t ret_val;
		ret_val = test_and_set_bit(BOOTLOAD_ACTIVE_BIT, state->flags);
		ASSERT(!ret_val);
	}

	// we're done with flags: exit critsec
	mutex_unlock(&state->dev_mutex);

	switch (cmd) {
		case A6_IOCTL_SET_FW_DATA:
		case A6_IOCTL_VERIFY_FW_DATA:
		{
			uint8_t *buffer_p = NULL;
			uint32_t payload_size = 0;
			struct a6_sbw_interface* sbw_ops =
					(struct a6_sbw_interface*)state->plat_data->sbw_ops;
			uint8_t* a6_fw_buffer;
			struct task_struct* pgm_worker_task;
			pid_t pgm_worker_task_pid;
			struct a6_pgm_thread_params t_params;

			// reset force-wake state to always force wake on first i2c txn
			// after flashing/verification
			del_timer(&state->a6_force_wake_timer);
			mutex_lock(&state->a6_force_wake_mutex);
			if (test_bit(CAP_PERIODIC_WAKE, state->flags)) {
				clear_bit(FORCE_WAKE_ACTIVE_BIT, state->flags);
			}
			mutex_unlock(&state->a6_force_wake_mutex);

			// copy payload ptr from ioctl parameter
			if (copy_from_user(&buffer_p, usr_ptr, sizeof(buffer_p))) {
				rc = -EFAULT;
				break;
			}
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "buffer_p val: 0x%p\n", buffer_p);

			// copy payload size from ioctl parameter
			if (copy_from_user(&payload_size, (uint8_t*)usr_ptr + sizeof(uint32_t), sizeof(payload_size))) {
				rc = -EFAULT;
				break;
			}

			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "payload_size: %d\n", payload_size);

			// alloc kernel buffer for payload
			a6_fw_buffer = (uint8_t*)kmalloc(payload_size, GFP_KERNEL);
			if (!a6_fw_buffer) {
				printk(KERN_ERR "A6: failed to allocate fw buffer.\n");
				rc = -ENOMEM;
				break;
			}
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "fw buffer ptr_val: 0x%p\n", a6_fw_buffer);

			// copy-in payload data
			if (copy_from_user(a6_fw_buffer, buffer_p, payload_size)) {
				rc = -EFAULT;
				break;
			}

			printk(KERN_ERR "A6: Starting flashing sequence.\n");
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "A6: parent task nice value: %d; pri: %d\n",
				   task_nice(current), task_prio(current));

			// initialize worker task params
			init_completion(&t_params.a6_flash_thread_nice);
			init_completion(&t_params.a6_flash_thread_exit);
			t_params.sbw_ops = sbw_ops;
			t_params.buffer_p = (uint32_t)a6_fw_buffer;
			t_params.ret_code = 0;
			t_params.op = (A6_IOCTL_SET_FW_DATA == cmd) ?
					A6_PROGAM_AND_VERIFY_FW : A6_VERIFY_FW;

			// create worker task...
			pgm_worker_task_pid = kernel_thread(a6_pgm_thread_fn, &t_params,
					CLONE_KERNEL);
			if (pgm_worker_task_pid < 0) {
				printk(KERN_ERR "A6: failed to create pgm worker task.\n");
				rc = -EIO;
				break;
			}

			// retrieve worker task struct
			pgm_worker_task = get_pid_task(find_get_pid(pgm_worker_task_pid), PIDTYPE_PID);
			// re-nice worker task
			//set_user_nice(pgm_worker_task, 10);
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "A6: pgm worker task nice value: %d; pri: %d\n",
				   task_nice(pgm_worker_task), task_prio(pgm_worker_task));

			/* hold cpu at max freq before signaling worker thread to commence sbw */
#ifdef CONFIG_CPU_FREQ_GOV_ONDEMAND_TICKLE
			CPUFREQ_HOLD_CHECK(&state->cpufreq_hold_flag);
#endif
			// signal re-nice completion...
			complete(&t_params.a6_flash_thread_nice);

			// wait for worker task-end completion...
			rc = wait_for_completion_interruptible(&t_params.a6_flash_thread_exit);
			/* unhold cpu */
#ifdef CONFIG_CPU_FREQ_GOV_ONDEMAND_TICKLE
			CPUFREQ_UNHOLD_CHECK(&state->cpufreq_hold_flag);
#endif
			kfree(a6_fw_buffer);

			if (-ERESTARTSYS == rc) {
				printk(KERN_ERR "A6: waiting for pgm worker start interrupted.\n");
				break;
			}

			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "A6: pgm worker task exit code: %d\n",
				   t_params.ret_code);
			if (t_params.ret_code) {
				printk(KERN_ERR "A6: Failed to completed flashing sequence. ret: %d\n",
				       t_params.ret_code);
				// propagate error to caller...
				rc = t_params.ret_code;
			}
			else {
				printk(KERN_ERR "A6: Completed flashing sequence.\n");
				// wait for the A6 to boot...
				msleep(3000);
				// - flashing fw forces a device power-up sequence: re-init state
				// - if init fails: treat as fw flash failure by returning rc
				rc = a6_init_state(state->i2c_dev);
				if (rc < 0) {
					printk(KERN_ERR "%s: failed to initialize, err: %d\n", A6_DRIVER, rc);
				}
			}

			mutex_lock(&state->dev_mutex);
			clear_bit(BOOTLOAD_ACTIVE_BIT, state->flags);
			mutex_unlock(&state->dev_mutex);
		}
		break;

		default:
		{
			rc = -EINVAL;
		}
		break;
	}


//Done:
	mutex_lock(&state->dev_mutex);
	if (rc) {
		if (test_bit(BOOTLOAD_ACTIVE_BIT, state->flags)) {
			clear_bit(BOOTLOAD_ACTIVE_BIT, state->flags);
		}
		clear_bit(DEVICE_BUSY_BIT, state->flags);
	}
	else {
		if (!test_bit(BOOTLOAD_ACTIVE_BIT, state->flags)) {
			clear_bit(DEVICE_BUSY_BIT, state->flags);
		}
	}
	mutex_unlock(&state->dev_mutex);

	wake_up_interruptible(&state->dev_busyq);

	return rc;
}

static int a6_open(struct inode *inode, struct file *file)
{
	struct a6_device_state* state;

	/* get device */
	state = container_of(file->f_op, struct a6_device_state, fops);

	/* Allow only read. */
	//if ((file->f_mode & (FMODE_READ|FMODE_WRITE)) != FMODE_READ) {
	//	    return -EINVAL;
	//}

	///* check if it is in use */
	//if (test_and_set_bit(IS_OPENED, state->flags)) {
	//	return -EBUSY;
	//}

	/* attach private data */
	file->private_data = state;

	return 0;
}

static int a6_close(struct inode *inode, struct file *file)
{
	struct a6_device_state* state = (struct  a6_device_state*) file->private_data;

	state = state;
	///* mark it as unused */
	//clear_bit(IS_OPENED, state->flags);

	return 0;
}

#define A2A_DGRAM_PREAMBLE (0x5AC35AC3)
struct a2a_dgram_hdr {
	uint32_t preamble;
	uint16_t len;
	uint16_t cksum;
};

static ssize_t a6_read(struct file *file, char __user *buf, size_t count, loff_t *ppos )
{
# define A2A_RX_MISS_THRESHOLD (300)

	ssize_t rc = 0;
	struct a6_device_state* state;
	struct a6_register_desc *reg_desc_comm_status, *reg_desc_comm_rxtx;
	uint8_t vals[id_size];
	int32_t miss_count, rd_count;
	//struct a2a_dgram_hdr hdr;
	uint32_t start_time;
	uint8_t elt_val = 0, prev_byte;

	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: enter\n", __func__);

	start_time = jiffies;
	/* input validations */
	if (!count) {
		return -EINVAL;
	}
	else if (count > A2A_RD_BUFF_SIZE) {
		return -EFBIG;
	}

	/* get state */
	state = container_of(file->f_op, struct a6_device_state, fops);

	// acquire critsec
	rc = mutex_lock_interruptible(&state->dev_mutex);
	if (rc) {
		printk(KERN_ERR "%s: mutex_lock interrupted.\n", __func__);
		return -ERESTARTSYS;
	}

	/* not connected? exit */
	if (!test_bit(A2A_CONNECTED, state->flags)) {
		mutex_unlock(&state->dev_mutex);
		printk(KERN_ERR "%s: no a2a connection detected.\n", __func__);
		return -EINVAL;
	}

	// busy?
	if (test_and_set_bit(READ_ACTIVE_BIT, state->flags)) {
		// yes: get on a waitq
		mutex_unlock(&state->dev_mutex);
		printk(KERN_ERR "%s: re-entrant call disallowed.\n", __func__);
		return -EINVAL;
	}

	mutex_unlock(&state->dev_mutex);

	/* init a2a read ptr */
	state->a2a_rp = state->a2a_rd_buf;

	rd_count = 0;
	miss_count = 0;
	prev_byte = 0;
	do {
		/* read comm status */
		reg_desc_comm_status = &a6_register_desc_arr[63];
		memset(vals, 0, sizeof(vals));
		rc = a6_i2c_read_reg(state->i2c_dev, reg_desc_comm_status->id,
				     reg_desc_comm_status->num_ids, vals);
		if (rc < 0) {
			printk(KERN_ERR "%s: error reading reg: %s, id: 0x%x\n",
			       __func__, reg_desc_comm_status->debug_name, reg_desc_comm_status->id[0]);
			goto err0;
		}

		if (!(vals[0] & TS2_I2C_COMM_STATUS_RX_FULL)) {
			if (++miss_count > A2A_RX_MISS_THRESHOLD) {
				if (test_bit(A2A_CONNECTED, state->flags)) {
					continue;
				}
				else {
					break;
				}
			}
			continue;
		}

		/* read rx data */
		memset(vals, 0, sizeof(vals));
		reg_desc_comm_rxtx = &a6_register_desc_arr[64];
		rc = a6_i2c_read_reg(state->i2c_dev, reg_desc_comm_rxtx->id,
				      reg_desc_comm_rxtx->num_ids, vals);
		if (rc < 0) {
			printk(KERN_ERR "%s: error writing reg: %s, id: 0x%x\n",
			       __func__, reg_desc_comm_rxtx->debug_name, reg_desc_comm_rxtx->id[0]);
			rc = -EIO;
			goto err0;
		}

		A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: read byte: %d successfully.\n",
			   __func__, rd_count);

		if (a6_t2s_dup_correct) {
			if ((prev_byte & 0x80) ^ (vals[0] & 0x80)) {
				prev_byte = vals[0];
				if (vals[0] & 0x80) {
					elt_val = _convert_hex_char_to_decimal(vals[0] & 0x7f) << 4;
					continue;
				}
				else {
					elt_val |= (_convert_hex_char_to_decimal(vals[0]) & 0x0f);
				}
			}
			else {
				printk(KERN_ERR "%s: t2s duplicate detected; char: 0x%02x.\n",
				       __func__, vals[0]);
				continue;
			}
		}
		else {
			elt_val = vals[0];
		}

		*state->a2a_rp = elt_val;
		state->a2a_rp++;
		rd_count++;
	} while (rd_count < count);

	if (!rd_count) {
		printk(KERN_ERR "%s: rx failed; A2A connection terminated.\n", __func__);
		rc = -EINVAL;
	}
	else {
		long diff_time;

		if (copy_to_user(buf, state->a2a_rd_buf, rd_count)) {
			rc = -EFAULT;
			goto err0;
		}
		rc = rd_count;
		diff_time = (long)jiffies - (long)start_time;
		printk(KERN_ERR "%s: elapsed time: %ld ms; count: %u\n",
		       __func__, diff_time * 1000/HZ, rd_count);
	}


err0:
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: exit\n", __func__);
	clear_bit(READ_ACTIVE_BIT, state->flags);
	return rc;
}

static ssize_t a6_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos )
{
# define A2A_TX_MISS_THRESHOLD (300)

	ssize_t rc = 0;
	struct a6_device_state* state;
	struct a6_register_desc *reg_desc_comm_status, *reg_desc_comm_rxtx;
	uint8_t vals[id_size];
	int32_t miss_count, wr_count;
	//struct a2a_dgram_hdr hdr;
	uint32_t start_time;
	uint8_t elt_buf[2], *elt_bufp = NULL;
	int elt_bufsize;

	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: enter\n", __func__);

	start_time = jiffies;
	/* input validations */
	if (!count) {
		return -EINVAL;
	}
	else if (count > A2A_WR_BUFF_SIZE) {
		return -EFBIG;
	}

	/* get state */
	state = container_of(file->f_op, struct a6_device_state, fops);

	// acquire critsec
	rc = mutex_lock_interruptible(&state->dev_mutex);
	if (rc) {
		printk(KERN_ERR "%s: mutex_lock interrupted.\n", __func__);
		return -ERESTARTSYS;
	}

	/* not connected? exit */
	if (!test_bit(A2A_CONNECTED, state->flags)) {
		mutex_unlock(&state->dev_mutex);
		printk(KERN_ERR "%s: no a2a connection detected.\n", __func__);
		return -EINVAL;
	}

	// busy?
	if (test_and_set_bit(WRITE_ACTIVE_BIT, state->flags)) {
		mutex_unlock(&state->dev_mutex);
		printk(KERN_ERR "%s: re-entrant call disallowed.\n", __func__);
		return -EINVAL;
	}

	mutex_unlock(&state->dev_mutex);

	/* copy to kernel buffer */
	//hdr.preamble = A2A_DGRAM_PREAMBLE;
	//hdr.len = count;
	//hdr.cksum = 0;
	///* header */
	//if (copy_from_user(state->a2a_wr_buf, &hdr, sizeof(hdr))) {
	//	rc = -EFAULT;
	//	mutex_unlock(&state->dev_mutex);
	//	goto err0;
	//}
	/* data */
	if (copy_from_user(state->a2a_wr_buf/*+sizeof(hdr)*/, buf, count)) {
		rc = -EFAULT;
		goto err0;
	}
	/* init a2a write ptr */
	state->a2a_wp = state->a2a_wr_buf;

	wr_count = 0;
	miss_count = 0;
	elt_bufsize = 0;
	do {
		/* read comm status */
		reg_desc_comm_status = &a6_register_desc_arr[63];
		memset(vals, 0, sizeof(vals));
		rc = a6_i2c_read_reg(state->i2c_dev, reg_desc_comm_status->id,
				      reg_desc_comm_status->num_ids, vals);
		if (rc < 0) {
			printk(KERN_ERR "%s: error reading reg: %s, id: 0x%x\n",
			__func__, reg_desc_comm_status->debug_name, reg_desc_comm_status->id[0]);
			goto err0;
		}

		if (!(vals[0] & TS2_I2C_COMM_STATUS_TX_EMPTY)) {
			if (++miss_count > A2A_TX_MISS_THRESHOLD) {
				if (test_bit(A2A_CONNECTED, state->flags)) {
					continue;
				}
				else {
					break;
				}
			}
			continue;
		}

		/* write tx data */
		reg_desc_comm_rxtx = &a6_register_desc_arr[64];
		if (!elt_bufsize) {
			if (a6_t2s_dup_correct) {
				elt_bufsize = 2;
				sprintf(elt_buf, "%02x", *state->a2a_wp);
				elt_buf[0] |= 0x80;
			}
			else {
				elt_bufsize = 1;
				elt_buf[0] = *state->a2a_wp;
			}
			elt_bufp = elt_buf;
		}
		rc = a6_i2c_write_reg(state->i2c_dev, reg_desc_comm_rxtx->id,
				       reg_desc_comm_rxtx->num_ids, elt_bufp);
		if (rc < 0) {
			printk(KERN_ERR "%s: error writing reg: %s, id: 0x%x\n",
			__func__, reg_desc_comm_rxtx->debug_name, reg_desc_comm_rxtx->id[0]);
			rc = -EIO;
			goto err0;
		}
		elt_bufp++;
		elt_bufsize--;

		if (!elt_bufsize) {
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: written byte: %d successfully.\n",
				   __func__, wr_count);
			state->a2a_wp++;
			wr_count++;
		}
	} while (wr_count < count);

	if (!wr_count) {
		printk(KERN_ERR "%s: tx failed; A2A connection terminated.\n",
		       __func__);
		rc = -EIO;
		goto err0;
	}
	else {
		long diff_time;
		rc = wr_count;
		diff_time = (long)jiffies - (long)start_time;
		printk(KERN_ERR "%s: elapsed time: %ld ms; count: %u\n", __func__,
		       diff_time * 1000/HZ, wr_count);
	}


err0:
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: exit\n", __func__);
	clear_bit(WRITE_ACTIVE_BIT, state->flags);
	return rc;
}

static unsigned int a6_poll(struct file* file, struct poll_table_struct* wait)
{
	unsigned int  mask = 0;

	return mask;
}

struct file_operations a6_fops = {
	.owner   = THIS_MODULE,
	.read    = a6_read,
	.write    = a6_write,
	.poll    = a6_poll,
	.ioctl   = a6_ioctl,
	.open    = a6_open,
	.release = a6_close,
};

/*
 *  a6 interrupt handler
 */
static irqreturn_t a6_irq(int irq, void *dev_id)
{
	struct a6_device_state* state = (struct a6_device_state *)dev_id;

	a6_tp_irq_count++;
#if defined PROFILE_USAGE
	/*
	if (true == reset_active) {
		diff_time = (long)jiffies - (long)start_time;
		reset_active = false;
		A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "A6_IRQ toggle post power-cycle after: %d ms\n",
				 diff_time*1000/HZ);
	}
	*/
#endif

	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: entry.\n", __func__);

	if (test_bit(IS_SUSPENDED, state->flags)) {
		set_bit(INT_PENDING, state->flags);
	} else {
		queue_work(state->ka6d_workqueue, &state->a6_irq_work);
	}
	
	return IRQ_HANDLED;
}

void a6_irq_work_handler(struct work_struct *work)
{
	struct a6_device_state* state =
			container_of(work, struct a6_device_state, a6_irq_work);
	struct a6_register_desc *reg_desc_status3, *reg_desc_status2;
	uint8_t vals[id_size], reg_val_status3 = 0, reg_val_status2 = 0;
	bool charge_source_changed = false;
	bool battery_changed = false;
	int32_t ret = 0;
	char *envp[] = {
		[0] = "A6_ACTION=EMERGENCY_RESET_NOTIFY",
		[1] = NULL,
		[2] = "A6_ACTION=LOG_THRESHOLD_NOTIFY",
		[3] = NULL,
		[4] = "A6_ACTION=CHARGE_SOURCE_NOTIFY",
		[5] = NULL,
		[6] = "A6_ACTION=PERCENT_LOW_WARN1_NOTIFY",
		[7] = NULL,
		[8] = "A6_ACTION=PERCENT_LOW_WARN2_NOTIFY",
		[9] = NULL,
		[10] = "A6_ACTION=PECENT_LOW_CRIT_NOTIFY",
		[11] = NULL,
		[12] = "A6_ACTION=VOLTAGE_LOW_CRIT_NOTIFY",
		[13] = NULL,
		[14] = "A6_ACTION=TEMP_LOW_CRIT_NOTIFY",
		[15] = NULL,
		[16] = "A6_ACTION=TEMP_HIGH_CRIT_NOTIFY",
		[17] = NULL,
		[18] = "A6_ACTION=A2A_CONNECT_NOTIFY",
		[19] = NULL
	};

	// block irq processing while a6 fw flashing is in progress because i2c requests
	// will fail anyway and we dont' want to fiddle with SBW_WKUP while flashing is
	// in progress...

	// critsec for manipulating flags
	mutex_lock(&state->dev_mutex);

	// busy?
	while (test_and_set_bit(DEVICE_BUSY_BIT, state->flags)) {
		// yes: are we in bootload phase?
		if (!test_bit(BOOTLOAD_ACTIVE_BIT, state->flags)) {
			// no: so go ahead and allow concurrent i2c ops (these are
			// synchronized separately to allow priority based execution).
			break;
		}

		// in bootload phase: get on a waitq
		mutex_unlock(&state->dev_mutex);
		printk(KERN_ERR "%s: about to wait for device non-busy...\n", __func__);

		// bootload bit set? wait to be cleared (at least 5 mins: in jiffies)
		ret = wait_event_interruptible_timeout(state->dev_busyq,
				!test_bit(BOOTLOAD_ACTIVE_BIT, state->flags), 300*HZ);
		if (!ret) {
			printk(KERN_ERR "%s: wait on device busy timed-out/interrupted\n", __func__);
			// reset busy state
			clear_bit(BOOTLOAD_ACTIVE_BIT, state->flags);
			clear_bit(DEVICE_BUSY_BIT, state->flags);
			// and continue...
		}

		// we're about to manipulate flags again: acquire critsec
		mutex_lock(&state->dev_mutex);
	}

	/* increment busy refcount */
	state->busy_count++;

	// done with flags: exit critsec
	mutex_unlock(&state->dev_mutex);

	/* determine irq cause */
	reg_desc_status3 = &a6_register_desc_arr[7];
	reg_desc_status2 = &a6_register_desc_arr[6];

	/* (1) read int_status3: emergency reset and charge-source change */
	memset(vals, 0, sizeof(vals));
	ret = a6_i2c_read_reg(state->i2c_dev, reg_desc_status3->id, reg_desc_status3->num_ids, vals);
	if (ret < 0) {
		printk(KERN_ERR "%s: error reading reg: %s, id: 0x%x\n",
		       __func__, reg_desc_status3->debug_name, reg_desc_status3->id[0]);
		goto err0;
	}
	reg_val_status3 = vals[0];

	/* (2) read int_status2: other irq causes */
	memset(vals, 0, sizeof(vals));
	ret = a6_i2c_read_reg(state->i2c_dev, reg_desc_status2->id, reg_desc_status2->num_ids, vals);
	if (ret < 0) {
		printk(KERN_ERR "%s: error reading reg: %s, id: 0x%x\n",
		       __func__, reg_desc_status2->debug_name, reg_desc_status2->id[0]);
		goto err0;
	}
	reg_val_status2 = vals[0];

	/* (3) now clear all status bits: this "releases" the irq line early */
	if (reg_val_status3) {
		vals[0] = reg_val_status3;
		ret = a6_i2c_write_reg(state->i2c_dev, reg_desc_status3->id, reg_desc_status3->num_ids, vals);
		if (ret < 0) {
			printk(KERN_ERR "%s: error writing reg: %s, id: 0x%x\n",
			       __func__, reg_desc_status3->debug_name, reg_desc_status3->id[0]);
			goto err0;
		}
	}
	if (reg_val_status2) {
		vals[0] = reg_val_status2;
		ret = a6_i2c_write_reg(state->i2c_dev, reg_desc_status2->id, reg_desc_status2->num_ids, vals);
		if (ret < 0) {
			printk(KERN_ERR "%s: error writing reg: %s, id: 0x%x\n",
			       __func__, reg_desc_status3->debug_name, reg_desc_status3->id[0]);
			goto err0;
		}
	}

	/* (4) process emergency reset and charge-source changes...*/
	if (reg_val_status3) {
		/* emergency reset? */
		if (reg_val_status3 & TS2_I2C_INT_3_RESET) {
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: emergency reset detected.\n",
				   __func__);

			/* Send uevent */
			kobject_uevent_env(&state->i2c_dev->dev.kobj, KOBJ_CHANGE, &envp[0]);
			/* this condition overrides all others and we can probably skip
			 resetting the status bits */
			goto err0;
		}

		/* a2a connect change? */
		if (reg_val_status3 & TS2_I2C_INT_3_A2A_CONNECT_CHANGE) {
			struct a6_register_desc *reg_desc_charger;
			uint8_t chg_vals[id_size];

			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: a2a connect change detected.\n",
				   __func__);
			printk(KERN_ERR "%s: a2a connect change detected.\n", __func__);
			reg_desc_charger = &a6_register_desc_arr[31];
			memset(chg_vals, 0, sizeof(chg_vals));
			if (a6_i2c_read_reg(state->i2c_dev, reg_desc_charger->id,
					     reg_desc_charger->num_ids, chg_vals) < 0) {
				printk(KERN_ERR "%s: error reading reg: %s, id: 0x%x\n",
				       __func__, reg_desc_charger->debug_name,
				       reg_desc_charger->id[0]);
			}
			else {
				if (chg_vals[0] & TS2_I2C_FLAGS_2_A2A_CONNECT) {
					set_bit(A2A_CONNECTED, state->flags);
				}
				else {
					clear_bit(A2A_CONNECTED, state->flags);
				}
				/* Send uevent */
				kobject_uevent_env(&state->i2c_dev->dev.kobj, KOBJ_CHANGE, &envp[18]);
			}
		}

		/* charger-source change? */
		if (reg_val_status3 & TS2_I2C_INT_3_FLAGS_CHANGE) {
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: charger-source change detected.\n",
				   __func__);

			/* next, unblock task on charger_source_notify node */
			//sysfs_notify_dirent(state->notify_nodes[DIRENT_CHG_SRC_NOTIFY]);
			/* Send uevent */
			kobject_uevent_env(&state->i2c_dev->dev.kobj, KOBJ_CHANGE, &envp[4]);
			charge_source_changed = true;
		}

		/* log threshold change? */
		if (reg_val_status3 & TS2_I2C_INT_3_LOG) {
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: log threshold detected.\n",
				   __func__);

			/* Send uevent */
			kobject_uevent_env(&state->i2c_dev->dev.kobj, KOBJ_CHANGE, &envp[2]);
		}
	}

	/* (5) process other irq causes... */
	if (reg_val_status2) {
		/* battery low critical? */
		if (reg_val_status2 & TS2_I2C_INT_2_BAT_RARC_CRIT) {
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: battery low critical detected.\n",
				   __func__);

			/* Send uevent */
			kobject_uevent_env(&state->i2c_dev->dev.kobj, KOBJ_CHANGE, &envp[10]);
			battery_changed = true;
		}

		/* battery voltage low critical? */
		if (reg_val_status2 & TS2_I2C_INT_2_BAT_VOLT_LOW) {
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: battery voltage low critical detected.\n",
				   __func__);

			/* Send uevent */
			kobject_uevent_env(&state->i2c_dev->dev.kobj, KOBJ_CHANGE, &envp[12]);
			battery_changed = true;
		}

		/* battery temp high critical? */
		if (reg_val_status2 & TS2_I2C_INT_2_BAT_TEMP_HIGH) {
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: battery temp high critical detected.\n",
				   __func__);

			/* Send uevent */
			kobject_uevent_env(&state->i2c_dev->dev.kobj, KOBJ_CHANGE, &envp[16]);
		}

		/* battery temp low critical? */
		if (reg_val_status2 & TS2_I2C_INT_2_BAT_TEMP_LOW) {
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: battery temp low critical detected.\n",
				   __func__);

			/* Send uevent */
			kobject_uevent_env(&state->i2c_dev->dev.kobj, KOBJ_CHANGE, &envp[14]);
		}

		/* battery low percent warn2? */
		if (reg_val_status2 & TS2_I2C_INT_2_BAT_RARC_LOW2) {
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: battery low percent warn2 detected.\n",
				   __func__);

			/* Send uevent */
			kobject_uevent_env(&state->i2c_dev->dev.kobj, KOBJ_CHANGE, &envp[8]);
			battery_changed = true;
		}

		/* battery low percent warn1? */
		if (reg_val_status2 & TS2_I2C_INT_2_BAT_RARC_LOW1) {
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: battery low percent warn1 detected.\n",
				   __func__);

			/* Send uevent */
			kobject_uevent_env(&state->i2c_dev->dev.kobj, KOBJ_CHANGE, &envp[6]);
			battery_changed = true;
		}
	}

err0:
	/* decrement busy refcount */
	if (state->busy_count)
		state->busy_count--;
	if (!state->busy_count)
		clear_bit(DEVICE_BUSY_BIT, state->flags);

	if (charge_source_changed) {
		a6_dock_update_state(state);
	}
	if (state->plat_data->power_supply_connected == 1 && batt_state != NULL) {
		if (battery_changed) {
			power_supply_changed(&a6_fish_power_supplies[0]);
		}
		if (charge_source_changed) {
			power_supply_changed(&a6_fish_power_supplies[1]);
			power_supply_changed(&a6_fish_power_supplies[2]);
#ifdef CONFIG_A6_ENABLE_DOCK_PS
			power_supply_changed(&a6_fish_power_supplies[3]);
#endif
			a6_update_connected_ps();
		}
	}

	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: Visited\n", __func__);
}


#ifdef A6_PQ
int32_t a6_stop_ai_dispatch_task(struct a6_device_state* state)
{
	int32_t rc = 0;

	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: entered.\n", __func__);
	// critsec for manipulating flags
	rc = mutex_lock_interruptible(&state->dev_mutex);
	if (rc) {
		printk(KERN_ERR "%s: mutex_lock interrupted\n", __func__);
		return -ERESTARTSYS;
	}
	// stopping during a start? fail
	if (test_bit(STARTING_AID_TASK, state->flags)) {
		printk(KERN_ERR "%s: aid task not fully started. failing op.\n", __func__);
		rc = -EBUSY;
		mutex_unlock(&state->dev_mutex);
		goto err0;
	}

	// stopping during a stop? fail
	if (test_bit(KILLING_AID_TASK, state->flags)) {
		printk(KERN_ERR "%s: aid task being stopped. failing op.\n", __func__);
		rc = -EBUSY;
		mutex_unlock(&state->dev_mutex);
		goto err0;
	}

	// task never started? fail
	if (!state->ai_dispatch_task) {
		printk(KERN_ERR "%s: no aid task. failing op.\n", __func__);
		rc = -EPERM;
		mutex_unlock(&state->dev_mutex);
		goto err0;
	}

	// transition to quiesced state: stops accepting new requests
	set_bit(IS_QUIESCED, state->flags);
	// declare intent to kill ai dispatch task
	set_bit(KILLING_AID_TASK, state->flags);
	// exit critsec
	mutex_unlock(&state->dev_mutex);

	// kick task if asleep
	complete(&state->aq_enq_complete);

	// wait for ai dispatch task exit
	rc = wait_for_completion_interruptible(&state->aid_exit_complete);
	if (rc < 0) {
		printk(KERN_ERR "%s: wait for ai dispatch task exit interrupted.\n",
		       __func__);
	}

	// critsec for manipulating flags
	rc = mutex_lock_interruptible(&state->dev_mutex);
	if (rc) {
		printk(KERN_ERR "%s: mutex_lock interrupted(1)\n", __func__);
		return -ERESTARTSYS;
	}
	// ai dispatch task exited (assume it did if we get interrupted): reset state
	clear_bit(KILLING_AID_TASK, state->flags);
	state->ai_dispatch_task = NULL;
	// exit critsec
	mutex_unlock(&state->dev_mutex);


err0:
	return rc;
}

int32_t a6_start_ai_dispatch_task(struct a6_device_state* state)
{
	int32_t rc = 0;
	pid_t ai_dispatch_pid;

	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: entered.\n", __func__);

	// critsec for manipulating flags
	rc = mutex_lock_interruptible(&state->dev_mutex);
	if (rc) {
		printk(KERN_ERR "%s: mutex_lock interrupted\n", __func__);
		return -ERESTARTSYS;
	}
	// starting during a start? fail
	if (test_bit(STARTING_AID_TASK, state->flags)) {
		printk(KERN_ERR "%s: aid task not fully started. failing op.\n", __func__);
		rc = -EBUSY;
		mutex_unlock(&state->dev_mutex);
		goto err0;
	}

	// starting during a stop? fail
	if (test_bit(KILLING_AID_TASK, state->flags)) {
		printk(KERN_ERR "%s: aid task being stopped. failing op.\n", __func__);
		rc = -EBUSY;
		mutex_unlock(&state->dev_mutex);
		goto err0;
	}

	// task never stopped? fail
	if (state->ai_dispatch_task) {
		printk(KERN_ERR "%s: aid task exists. failing op.\n", __func__);
		rc = -EPERM;
		mutex_unlock(&state->dev_mutex);
		goto err0;
	}

	// declare intent to start ai dispatch task
	set_bit(STARTING_AID_TASK, state->flags);
	// exit critsec
	mutex_unlock(&state->dev_mutex);


	// create ai dispatcher task...
	ai_dispatch_pid = kernel_thread(ai_dispatch_thread_fn, state,
					CLONE_KERNEL);
	if (ai_dispatch_pid < 0) {
		printk(KERN_ERR "%s: failed to create ai dispatcher task.\n", __func__);
		rc = -EIO;
		goto err0;
	}

	rc = mutex_lock_interruptible(&state->dev_mutex);
	if (rc) {
		printk(KERN_ERR "%s: mutex_lock interrupted(1)\n", __func__);
		return -ERESTARTSYS;
	}
	// retrieve worker task struct
	state->ai_dispatch_task = get_pid_task(find_get_pid(ai_dispatch_pid), PIDTYPE_PID);
	ASSERT(state->ai_dispatch_task);

	// transition to active state: start accepting new requests
	clear_bit(IS_QUIESCED, state->flags);
	// ai dispatch task started: reset state
	clear_bit(STARTING_AID_TASK, state->flags);
	// exit critsec
	mutex_unlock(&state->dev_mutex);

err0:
	return rc;
}
#endif // A6_PQ

void a6_force_wake_work_handler(struct work_struct *work)
{
	struct a6_device_state* state =
			container_of(work, struct a6_device_state, a6_force_wake_work);
	struct a6_wake_ops* wake_ops = (struct a6_wake_ops*)state->plat_data->wake_ops;
	long diff_time;

	diff_time = (long)jiffies - (long)start_last_a6_activity;
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: forcing sleep after: %ld ms\n",
		   __func__, diff_time * 1000/HZ);

	start_last_a6_activity = 0;

	// force A6 sleep and switch back to periodic wake...
	// * timer may be scheduled just after we clear FORCE_WAKE_ACTIVE_BIT 
	//   and hence will result in the callback being invoked with
	//   FORCE_WAKE_ACTIVE_BIT cleared. We just ignore this case...
	mutex_lock(&state->a6_force_wake_mutex);
	if (!test_bit(DEVICE_BUSY_BIT, state->flags)) {
		if (test_bit(FORCE_WAKE_ACTIVE_BIT, state->flags)) {
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR,
				"%s: disabling force_wake and enabling periodic_wake\n",
				__func__);
			/* force A6 sleep */
			if (wake_ops->force_sleep) {
				wake_ops->force_sleep(wake_ops->data);
			}
			/* enable periodic a6 wake (if defined) */
			if (wake_ops->enable_periodic_wake) {
				wake_ops->enable_periodic_wake(wake_ops->data);
			}
			/* now we are ready to clear FORCE_WAKE_ACTIVE_BIT */
			clear_bit(FORCE_WAKE_ACTIVE_BIT, state->flags);
		}
	}
	mutex_unlock(&state->a6_force_wake_mutex);
}


// timer callback used to force sleep after a force wake
void a6_force_wake_timer_callback(ulong data)
{
	struct a6_device_state* state = (struct a6_device_state*)data;
	int32_t rc;

	rc = queue_work(state->ka6d_fw_workqueue, &state->a6_force_wake_work);
	if (!rc) {
		printk(KERN_ERR "**** %s: failed queueing force_wake work item.\n", __func__);
	}
}

static int a6_pmem_open(struct inode *inode, struct file *file)
{
	struct a6_device_state* state;

	/* get device */
	state = container_of(file->f_op, struct a6_device_state, pmem_fops);

	/* Allow only read. */
	if ((file->f_mode & (FMODE_READ|FMODE_WRITE)) != FMODE_READ) {
		    return -EINVAL;
	}

	/* check if it is in use */
	if (test_and_set_bit(IS_OPENED, state->flags)) {
		return -EBUSY;
	}

	/* attach private data */
	file->private_data = state;
	return 0;
}

static int a6_pmem_close(struct inode *inode, struct file *file)
{
	struct a6_device_state* state = (struct  a6_device_state*) file->private_data;

	/* mark it as unused */
	clear_bit(IS_OPENED, state->flags);
	return 0;
}

static ssize_t a6_pmem_read(struct file *file, char __user *buf, size_t count, loff_t *ppos )
{

	ssize_t rc = 0;
	struct a6_device_state* state;

	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: enter\n", __func__);

	/* input validations */
	if (!count) {
		return -EINVAL;
	}

	/* get state */
	state = container_of(file->f_op, struct a6_device_state, fops);
	rc = ttf_image_read(buf, count, ppos);
	
	return rc;
}

struct file_operations a6_pmem_fops = {
	.owner   = THIS_MODULE,
	.read    = a6_pmem_read,
	.open    = a6_pmem_open,
	.release = a6_pmem_close,
};

static int a6_fish_battery_get_percent(struct device *dev)
{
	int temp_val = 0;
	struct a6_device_state *state = (struct a6_device_state*) dev_get_drvdata(dev);

	state->print_buffer[0] = '\0';
	a6_generic_show (dev, &a6_register_desc_arr[9].dev_attr, state->print_buffer);
	sscanf (state->print_buffer, "%d", &temp_val);

	return temp_val;
}

static unsigned a6_calc_connected_ps(void)
{
	struct power_supply *psy;
	struct a6_device_state *state;
	unsigned int temp_val = 0;
	unsigned connected = 0;

	psy = &a6_fish_power_supplies[0];

	if (!(psy->dev)) {
		printk(KERN_ERR "%s: psy->dev is NULL\n", __func__);
		return 0;
	}
	if (!(psy->dev->parent)) {
		printk(KERN_ERR "%s: psy->dev->parent is NULL\n", __func__);
		return 0;
	}

	state = (struct a6_device_state*)dev_get_drvdata(psy->dev->parent);
	state->print_buffer[0] = '\0';

	a6_generic_show (psy->dev->parent,
			&a6_register_desc_arr[31].dev_attr, state->print_buffer);

	sscanf (state->print_buffer, "%u", &temp_val);

	if (state->otg_chg_type == USB_CHG_TYPE__WALLCHARGER) {
		connected |= MAX8903B_CONNECTED_PS_AC;
	}

	if (state->otg_chg_type == USB_CHG_TYPE__SDP
			&& (temp_val & TS2_I2C_FLAGS_2_WIRED_CHARGE)) {
		/* NOTE: USB will not show as connected if DOCK is connected */
		connected |= MAX8903B_CONNECTED_PS_USB;
	}

	if(temp_val & TS2_I2C_FLAGS_2_PUCK_CHARGE) {
		connected |= MAX8903B_CONNECTED_PS_DOCK;
	}

	return connected;
}

static void a6_update_connected_ps()
{
	unsigned connected = a6_calc_connected_ps();

	printk(KERN_INFO "%s: ac=%d usb=%d dock=%d\n", __func__,
			(connected & MAX8903B_CONNECTED_PS_AC) ? 1 : 0,
			(connected & MAX8903B_CONNECTED_PS_USB) ? 1 : 0,
			(connected & MAX8903B_CONNECTED_PS_DOCK) ? 1 : 0);

	max8903b_set_connected_ps(connected);
	a6_last_ps_connect = (long)jiffies;
}

static int a6_fish_power_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	unsigned connected;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		connected = a6_calc_connected_ps();

		if (
				(psy->type == POWER_SUPPLY_TYPE_MAINS
					&& (connected & MAX8903B_CONNECTED_PS_AC)
#ifdef CONFIG_A6_ENABLE_DOCK_PS
					&& !(connected & MAX8903B_CONNECTED_PS_DOCK)
					&& !strcmp(psy->name, "ac")
#endif
					)
						||
				(psy->type == POWER_SUPPLY_TYPE_MAINS
				 	&& (connected & MAX8903B_CONNECTED_PS_DOCK)
#ifdef CONFIG_A6_ENABLE_DOCK_PS
					&& !strcmp(psy->name, "dock")
#endif
					)
						||
				(psy->type == POWER_SUPPLY_TYPE_USB 
				 	&& (connected & MAX8903B_CONNECTED_PS_USB)
				 	&& !(connected & MAX8903B_CONNECTED_PS_DOCK)
					)
				) {
			val->intval = 1;
		} else {
			val->intval = 0;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}


#define A6_BATT_STATS_DELAY (msecs_to_jiffies (15*1000))
static int a6_fish_battery_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	int temp_val = 0;
	unsigned connected;
	struct a6_device_state *state = 
			(struct a6_device_state*)dev_get_drvdata(psy->dev->parent);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (a6_fish_battery_get_percent(psy->dev->parent) == 100) {
			val->intval = POWER_SUPPLY_STATUS_FULL;
		} else if (a6_last_ps_connect &&
				jiffies > a6_last_ps_connect + A6_BATT_STATS_DELAY) {
			state->print_buffer[0] = '\0';
			a6_generic_show (psy->dev->parent, 
				/* TODO use A6_REG_TS2_I2C_BAT_AVG_CUR_LSB_MSB for 11 */
				&a6_register_desc_arr[11].dev_attr,
				state->print_buffer);
			sscanf (state->print_buffer, "%d", &temp_val);
			if (temp_val > 0) {
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			} else {
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			}
		} else {
			connected = a6_calc_connected_ps();
			if (connected && !(connected == MAX8903B_CONNECTED_PS_USB)) {
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			} else {
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			}
		}
		break;
	case POWER_SUPPLY_PROP_HEALTH:
#if 0
		state->print_buffer[0] = '\0';
		a6_generic_show (psy->dev->parent, &a6_register_desc_arr[8].dev_attr, state->print_buffer);
#endif
		// TODO: parse temps and set OVERHEAT, parse "age" and set DEAD */

		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = a6_fish_battery_get_percent(psy->dev->parent);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		state->print_buffer[0] = '\0';
		a6_generic_show (psy->dev->parent,
			&a6_register_desc_arr[A6_REG_TS2_I2C_BAT_CUR_LSB_MSB].dev_attr,
			state->print_buffer);
		sscanf (state->print_buffer, "%d", &temp_val);
		val->intval = temp_val;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		state->print_buffer[0] = '\0';
		a6_generic_show (psy->dev->parent,
			&a6_register_desc_arr[A6_REG_TS2_I2C_BAT_VOLT_LSB_MSB].dev_attr,
			state->print_buffer);
		sscanf (state->print_buffer, "%d", &temp_val);
		val->intval = temp_val;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		state->print_buffer[0] = '\0';
		a6_generic_show (psy->dev->parent,
			&a6_register_desc_arr[A6_REG_TS2_I2C_BAT_TEMP_LSB_MSB].dev_attr,
			state->print_buffer);
		sscanf (state->print_buffer, "%d", &temp_val);
		val->intval = temp_val * 10; /* convert C to .1 C */
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		state->print_buffer[0] = '\0';
		a6_generic_show (psy->dev->parent,
			&a6_register_desc_arr[A6_REG_TS2_I2C_BAT_FULL40_LSB_MSB].dev_attr,
			state->print_buffer);
		sscanf (state->print_buffer, "%d", &temp_val);
		val->intval = temp_val * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		state->print_buffer[0] = '\0';
		a6_generic_show (psy->dev->parent,
			&a6_register_desc_arr[A6_REG_TS2_I2C_BAT_COULOMB_LSB_MSB].dev_attr,
			state->print_buffer);
		sscanf (state->print_buffer, "%d", &temp_val);
		val->intval = temp_val * 1000;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

void a6_charger_event (int otg_chg_type)
{
	if (batt_state) {
		batt_state->otg_chg_type = otg_chg_type;

		power_supply_changed(&a6_fish_power_supplies[1]);
		power_supply_changed(&a6_fish_power_supplies[2]);
#ifdef CONFIG_A6_ENABLE_DOCK_PS
		power_supply_changed(&a6_fish_power_supplies[3]);
#endif
		a6_update_connected_ps();
	}
	return;
}
EXPORT_SYMBOL (a6_charger_event);

#define A6_BATT_HB_PERIOD (msecs_to_jiffies (5*60*1000))
#define A6_INIT_CONNECTED_PS_DELAY (msecs_to_jiffies (30*1000))
static void a6_battery_heartbeat(struct work_struct *a6_battery_work)
{
	int percent = 0;

	struct delayed_work *temp_charge_work =
			container_of (a6_battery_work, struct delayed_work, work);
	struct a6_device_state* state =
			container_of(temp_charge_work, struct a6_device_state, charge_work);

	if (state->stop_heartbeat)
		return;

	if ( (percent = a6_fish_battery_get_percent(&state->i2c_dev->dev))
			!= state->last_percent){

		state->last_percent = percent;
		power_supply_changed(&a6_fish_power_supplies[0]);
	}

	if (!state->stop_heartbeat)
		schedule_delayed_work(&state->charge_work,
						A6_BATT_HB_PERIOD);
}

static void a6_init_connected_ps(struct work_struct *work)
{
	a6_update_connected_ps();
}

static int a6_fish_battery_probe(struct a6_device_state *state)
{
	int i;
	int rc = 0;
	struct i2c_client *client = state->i2c_dev;

	/* init power supplier framework */
	for (i = 0; i < ARRAY_SIZE(a6_fish_power_supplies); i++) {
	
		rc = power_supply_register(&client->dev, &a6_fish_power_supplies[i]);
		if (rc)
			pr_err("%s: Failed to register power supply (%d)\n",
			       __func__, rc);
	}

	INIT_DELAYED_WORK(&state->charge_work, a6_battery_heartbeat);
	schedule_delayed_work(&state->charge_work, A6_BATT_HB_PERIOD);

	INIT_DELAYED_WORK(&state->init_connected_ps_work, a6_init_connected_ps);
	schedule_delayed_work(&state->init_connected_ps_work, A6_INIT_CONNECTED_PS_DELAY);

	return rc;
}


static int a6_fish_battery_remove(struct a6_device_state *state)
{
	int i;

	state->stop_heartbeat = true;
	cancel_delayed_work_sync(&state->charge_work);

	/* init power supplier framework */
	for (i = 0; i < ARRAY_SIZE(a6_fish_power_supplies); i++) {
		power_supply_unregister(&a6_fish_power_supplies[i]);
	}

	return 0;
}

static int a6_fish_battery_suspend (struct a6_device_state *state)
{

	if (state->plat_data->power_supply_connected == 1 &&
			delayed_work_pending(&state->charge_work)) {
		state->stop_heartbeat = true;
		smp_mb();
		cancel_delayed_work_sync(&state->charge_work);
	}

	return 0;
}

static int a6_fish_battery_resume (struct a6_device_state *state)
{
	if (state->plat_data->power_supply_connected == 1) {
		state->stop_heartbeat = false;
		schedule_delayed_work(&state->charge_work,
				A6_BATT_HB_PERIOD);

		power_supply_changed(&a6_fish_power_supplies[0]);
	}
	return 0;
}

static ssize_t a6_dock_print_name(struct switch_dev *sdev, char *buf)
{
	bool docked = switch_get_state(sdev) != 0;
	return sprintf(buf, docked ? "DESK\n" : "None\n");
}

static void a6_dock_update_state(struct a6_device_state *state)
{
	unsigned int value, dock;

	if (state->dock_switch == NULL) {
		return;
	}

	state->print_buffer[0] = '\0';
	a6_generic_show (&state->i2c_dev->dev, &a6_register_desc_arr[31].dev_attr, state->print_buffer);
	sscanf (state->print_buffer, "%u", &value);

	if (a6_disable_dock_switch) {
		dock = 0;
	} else {
		dock = value & TS2_I2C_FLAGS_2_PUCK ? 1 : 0;
	}
	switch_set_state(state->dock_switch, dock);
}

static int a6_dock_probe(struct a6_device_state *state)
{
	int ret;

	state->dock_switch = kzalloc(sizeof(struct switch_dev), GFP_KERNEL);
	if (state->dock_switch == NULL) {
		return -ENOMEM;
	}

	state->dock_switch->name = "dock";
	state->dock_switch->print_name = a6_dock_print_name;

	ret = switch_dev_register(state->dock_switch);
	if (ret < 0) {
		kfree(state->dock_switch);
		state->dock_switch = NULL;
		return ret;
	}

	a6_dock_update_state(state);

	return 0;
}

static void a6_dock_remove(struct a6_device_state *state)
{
	switch_dev_unregister(state->dock_switch);
	kfree(state->dock_switch);
	state->dock_switch = NULL;
}

/******************************************************************************
* a6_i2c_probe()
******************************************************************************/
static int a6_i2c_probe(struct i2c_client *client, const struct i2c_device_id *dev_id)
{
	int rc = 0;
	struct a6_device_state* state = NULL;
	struct a6_platform_data* plat_data = client->dev.platform_data;
	struct a6_wake_ops *wake_ops = (struct a6_wake_ops *) plat_data->wake_ops;

	if (plat_data == NULL) {
		return ENODEV;
	}

	state = kzalloc(sizeof(struct a6_device_state), GFP_KERNEL);
	if(!state) {
		return ENOMEM;
	}

	state->a2a_rd_buf = kzalloc(A2A_RD_BUFF_SIZE, GFP_KERNEL);
	if(!state->a2a_rd_buf) {
		rc = -ENOMEM;
		goto err0;
	}
	state->a2a_wr_buf = kzalloc(A2A_WR_BUFF_SIZE, GFP_KERNEL);
	if(!state->a2a_wr_buf) {
		rc = -ENOMEM;
		goto err1;
	}

	state->print_buffer = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!state->print_buffer) {
		rc = -ENOMEM;
		goto err2;
	}

	// store i2c client device in state
	state->i2c_dev = client;

	// set platform data in device-specific driver data
	state->plat_data = plat_data;

	mutex_init(&state->dev_mutex);
#ifdef A6_PQ
	init_completion(&state->aq_enq_complete);
	init_completion(&state->aid_exit_complete);
	mutex_init(&state->aq_mutex);
	INIT_LIST_HEAD(&state->aq_head);
#endif // A6_PQ
	mutex_init(&state->a6_force_wake_mutex);

	// zero-init wait flags
	bitmap_zero(state->flags, SIZE_FLAGS);

	// a6 external wake enabled?
	if (wake_ops) {
		set_bit(CAP_PERIODIC_WAKE, state->flags);
	}

	state->ka6d_workqueue = create_workqueue("ka6d");
	if (!state->ka6d_workqueue) {
		printk(KERN_ERR "%s: Failed to create ka6d workqueue.\n", A6_DRIVER);
		goto err3;
	}

	// separate wq for handling force wake timer expiry...
	state->ka6d_fw_workqueue  = create_workqueue("ka6d_fwd");
	if (!state->ka6d_fw_workqueue) {
		printk(KERN_ERR "%s: Failed to create ka6d_fwd workqueue.\n", A6_DRIVER);
		goto err4;
	}

	init_waitqueue_head(&state->dev_busyq);

	INIT_WORK(&state->a6_irq_work, a6_irq_work_handler);
	if (test_bit(CAP_PERIODIC_WAKE, state->flags)) {
		INIT_WORK(&state->a6_force_wake_work, a6_force_wake_work_handler);
	}

	state->cpufreq_hold_flag = 0;

	// set device-specific driver data
	i2c_set_clientdata(client, state);

	// configure sbw_tck
	rc = gpio_request(plat_data->sbw_tck_gpio, "a6_sbwtck");
	if (rc != 0) {
		printk(KERN_ERR "%s: request failed for sbw_tck gpio, val: %d.\n",
		       A6_DRIVER, plat_data->sbw_tck_gpio);
		goto err5;
	}

	rc = gpio_direction_output(plat_data->sbw_tck_gpio, 0);
	if (rc != 0) {
		printk(KERN_ERR "%s: set direction output failed for sbw_tck gpio, val: %d.\n",
		       A6_DRIVER, plat_data->sbw_tck_gpio);
		goto err6;
	}

	// configure sbw_wkup (this is the app -> a6 wakeup used in the JTAG handshaking)
	rc = gpio_request(plat_data->sbw_wkup_gpio, "a6_sbwwkup");
	if (rc != 0) {
		printk(KERN_ERR "%s: request failed for sbw_wkup gpio, val: %d.\n",
		       A6_DRIVER, plat_data->sbw_wkup_gpio);
		goto err6;
	}

	rc = gpio_direction_output(plat_data->sbw_wkup_gpio, 0);
	if (rc != 0) {
		printk(KERN_ERR "%s: set direction output failed for sbw_wkup gpio, val: %d.\n",
		       A6_DRIVER, plat_data->sbw_wkup_gpio);
		goto err7;
	}

	// configure sbw_tdio (initially configured as output and the re-configured on use)
	rc = gpio_request(plat_data->sbw_tdio_gpio, "a6_sbwtdio");
	if (rc != 0) {
		printk(KERN_ERR "%s: request failed for sbw_tdio gpio, val: %d.\n",
		       A6_DRIVER, plat_data->sbw_tdio_gpio);
		goto err7;
	}

	rc = gpio_direction_output(plat_data->sbw_tdio_gpio, 1);
	if (rc != 0) {
		printk(KERN_ERR "%s: set direction output failed for sbw_tdio gpio, val: %d.\n",
		       A6_DRIVER, plat_data->sbw_tdio_gpio);
		goto err8;
	}

	// configure pwr interrupt (a6 -> app)...
	rc = gpio_request(plat_data->pwr_gpio, "a6_pwr");
	if (rc != 0) {
		printk(KERN_ERR "%s: request failed for pwr gpio, val: %d., err: %d\n",
		       A6_DRIVER, plat_data->pwr_gpio, rc);
		goto err8;
	}

	rc = request_irq(gpio_to_irq(plat_data->pwr_gpio), a6_irq,
			 IRQF_TRIGGER_FALLING,
			 "a6", state);
	if (rc != 0) {
		printk(KERN_ERR "%s: request irq failed for pwr_gpio: val: %d, err: %d\n", A6_DRIVER,
		       plat_data->pwr_gpio, rc);
		goto err9;
	}

#if 0
	/* register as misc device */
	memcpy(&state->fops, &a6_fops, sizeof(struct file_operations));
	state->mdev.minor = MISC_DYNAMIC_MINOR;
	state->mdev.name = plat_data->dev_name;
	state->mdev.fops = &state->fops;
	rc = misc_register(&state->mdev);
	if (rc < 0) {
		printk(KERN_ERR "%s: Failed to register as misc device\n", A6_DRIVER);
		goto err10;
	}

	memcpy(&state->pmem_fops, &a6_pmem_fops, sizeof(struct file_operations));
	state->pmem_mdev.minor = MISC_DYNAMIC_MINOR;
	snprintf(state->pmem_dev_name, sizeof(state->pmem_dev_name),
		 "%s_diag", plat_data->dev_name);
	state->pmem_mdev.name = state->pmem_dev_name;
	state->pmem_mdev.fops = &state->pmem_fops;
	rc = misc_register(&state->pmem_mdev);
	if (rc < 0) {
		printk(KERN_ERR "%s: Failed to register as a6 pmem misc device\n", A6_DRIVER);
		goto err11;
	}

	rc = a6_create_dev_files(state, &client->dev);
	if (rc < 0) {
		goto err12;
	}
#endif

#ifdef A6_PQ
	rc = a6_start_ai_dispatch_task(state);
	if (rc < 0) {
		goto err13;
	}
#endif // A6_PQ

	// ignore errors during initialization: these may be symptomatic of missing/corrupt
	// a6 fw which will need to be remedied via the A6_IOCTL_SET_FW_DATA ioctl to
	// re-flash fw. so its important the driver initializes successfully to handle
	// the request.
	rc = a6_init_state(client);
	if (rc < 0) {
		printk(KERN_ERR "%s: failed to initialize, err: %d\n", A6_DRIVER, rc);
		rc = 0;
	}

	// not needed: pending work items will "flow through" if no status changes
	// are detected...
	//flush_workqueue(ka6d_workqueue);

#ifdef A6_PQ
#ifdef A6_DEBUG
	rc = a6_create_debug_interface(state);
	if (rc < 0) {
		printk(KERN_ERR "%s: Failed to create A6 debug interface.\n", A6_DRIVER);
		rc = 0;
	}
#endif
#endif

	if (plat_data->power_supply_connected == 1){
		if ( (rc = a6_fish_battery_probe (state)) < 0) {
			printk(KERN_ERR "%s: Failed to register power supplies, rc: %d.\n",
					A6_DRIVER, rc);
			rc = 0;
		}

		batt_state = state;

		if ((rc = a6_dock_probe(state)) < 0) {
			printk(KERN_ERR "%s: Failed to register dock device, rc: %d.\n",
					A6_DRIVER, rc);
			rc = 0;
		}
	}

	printk(KERN_NOTICE "A6 driver initialized successfully!\n");
	return 0;

err13:
#if 0
	a6_remove_dev_files(state, &client->dev);
err12:
	 misc_register(&state->pmem_mdev);
err11:
	misc_unregister(&state->mdev);
err10:
#endif
	free_irq(gpio_to_irq(plat_data->pwr_gpio), state);
err9:
	gpio_free(plat_data->pwr_gpio);
err8:
	gpio_free(plat_data->sbw_tdio_gpio);
err7:
	gpio_free(plat_data->sbw_wkup_gpio);
err6:
	gpio_free(plat_data->sbw_tck_gpio);
err5:
	destroy_workqueue(state->ka6d_fw_workqueue);
err4:
	destroy_workqueue(state->ka6d_workqueue);
err3:
	kfree(state->print_buffer);
err2:
	kfree(state->a2a_wr_buf);
err1:
	kfree(state->a2a_rd_buf);
err0:
	kfree(state);

	return rc;
}

/******************************************************************************
* a6_i2c_remove()
******************************************************************************/
static int a6_i2c_remove(struct i2c_client *client)
{
	struct a6_device_state* state = (struct a6_device_state*)i2c_get_clientdata(client);

	if (state->plat_data->power_supply_connected == 1){
		a6_fish_battery_remove (state);
	}
	if (state->dock_switch) {
		a6_dock_remove(state);
	}

#if 0
	a6_remove_dev_files(state, &client->dev);
#endif

	if (state->ka6d_workqueue) {
		destroy_workqueue(state->ka6d_workqueue);
	}

	if (state->ka6d_fw_workqueue) {
		destroy_workqueue(state->ka6d_fw_workqueue);
	}

	if (state->a2a_rd_buf) {
		kfree(state->a2a_rd_buf);
	}

	if (state->a2a_wr_buf) {
		kfree(state->a2a_wr_buf);
	}

	if (state->print_buffer) {
		kfree(state->print_buffer);
	}

	return 0;
}

#ifdef CONFIG_PM
/******************************************************************************
* a6_i2c_suspend
******************************************************************************/
static int a6_i2c_suspend(struct i2c_client *dev, pm_message_t event)
{
	struct a6_device_state* state = (struct a6_device_state*)i2c_get_clientdata(dev);

	set_bit(IS_SUSPENDED, state->flags);
	// configure a6 irq as wake-source to handle wake on a6 notifications
	// (charge source detection, various threshold transgressions and emergency
	// reset detection)...
	if (state->plat_data->pwr_gpio_wakeup_cap) {
		enable_irq_wake(gpio_to_irq(state->plat_data->pwr_gpio));
	}
	
	a6_fish_battery_suspend (state);

	return 0;
}

/******************************************************************************
* a6_i2c_resume
******************************************************************************/
static int a6_i2c_resume (struct i2c_client *dev)
{
	struct a6_device_state* state = (struct a6_device_state*)i2c_get_clientdata(dev);

	// un-configure a6 irq as wake-source...
	if (state->plat_data->pwr_gpio_wakeup_cap) {
		disable_irq_wake(gpio_to_irq(state->plat_data->pwr_gpio));
	}
	
	clear_bit(IS_SUSPENDED, state->flags);
	if (test_and_clear_bit(INT_PENDING, state->flags)) {
		queue_work(state->ka6d_workqueue, &state->a6_irq_work);
	}

	a6_fish_battery_resume (state);

	return 0;
}
#else
#define a6_i2c_suspend  NULL
#define a6_i2c_resume   NULL
#endif  /* CONFIG_PM */

static const struct i2c_device_id a6_ids[] = {
	{A6_DEVICE_0, },
	{A6_DEVICE_1, },
	{},
};

MODULE_DEVICE_TABLE(i2c, a6_ids);


static struct i2c_driver a6_i2c_driver = {
	.driver = {
		.name = A6_DRIVER,
		.owner = THIS_MODULE,
	},
		.id_table	= a6_ids,
		.probe		= a6_i2c_probe,
		.remove		= __devexit_p(a6_i2c_remove),
		.suspend	= a6_i2c_suspend,
		.resume		= a6_i2c_resume,
};

/*********************************************************************************
* a6_module_init(void)
***********************************************************************************/
static int __init a6_module_init(void)
{
	printk(KERN_INFO "Before a6 call to i2c_add_driver.\n");
	return i2c_add_driver(&a6_i2c_driver);
}

/*********************************************************************************
* cy8c24894_module_exit(void)
***********************************************************************************/
static void __exit a6_module_exit(void)
{
	i2c_del_driver(&a6_i2c_driver);
}

module_init(a6_module_init);
module_exit(a6_module_exit);

MODULE_DESCRIPTION("A6 driver");
MODULE_LICENSE("GPL");
