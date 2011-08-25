/* -*- linux-c -*-
 * include/linux/ma87710.h
 *
 */
#ifndef _MA87712_H_
#define _MA87712_H_

#include <linux/types.h>

#define MA87712_NAME		"MosArt MA87712 keypad MCU"
#define MA87712_DRIVER		"ma87712"
#define MA87712_DEVICE		"ma87712"
#define MA87712_VERSION		"1.0"

/*
 * registers list
 * ==============
 */

#define REG_STATUS			0x40
#define REG_KEY_TABLE_L		0x41
#define REG_KEY_TABLE_H		0x42
#define REG_SW_VERSION		0x7E
#define REG_DEVICE_ID 		0x7F
#define SLAVE_ID 			0x87

#define FIFO_LENGTH			14
#define KEY_RELEASED		0x8000
#define KEY_PRESSED			0x8000

#define MA87712_SLAVE_ADDR	0x73

/*
 * exported data
 * =============
 */

struct ma87712_platform_data {
	int (*set_power)(int val);
	u8 auto_on;
	u8 caplock_on;
	u8 mute_on;
	u8 gpio_rst_n;
	u8 gpio_pd_n;
	u8 gpio_int_n;
	u8 dim_status;
	u8* keycode;
	u8 keycodemax;
	u8 keycodesize;
};

#if 0
#define REG_MOUSE_ABS		0x30
#define REG_MOUSE_X_L		0x30
#define REG_MOUSE_X_H		0x31
#define REG_MOUSE_Y_L		0x32
#define REG_MOUSE_Y_H		0x33


#define REG_CTRL0		0x40
#define REG_CTRL1		0x41

#define REG_REPEAT_INT		0x42
#define REG_ACCUM_ENABLE	0x42
#define REG_ACCUM_ENABLE_X	0x42

#define REG_STATUS		0x43
#define REG_STATUS1		0x44

#define REG_MOUSE_DATA		0x45
#define REG_MOUSE_BUTTON	0x45
#define REG_MOUSE_X		0x46
#define REG_MOUSE_Y		0x47
#define REG_MOUSE_Z		0x48

#define REG_KEY_TABLE_L		0x49
#define REG_KEY_TABLE_H		0x4A
#define REG_DECS_PULSE		0x4B
#define REG_WAKEUP		0x4C
#define	REG_BAT_REREAD		0x4D
#define REG_ACCUM_ENABLE_Y	0x4E

#define REG_LED_CTRL		0x50
#define REG_PWM_PERIOD_ON	0x51
#define REG_PWM_PERIOD_OFF	0x52
#define REG_KSTATUS_LIGHT	0x53

#define	REG_LS_CTRL		0x58
#define	REG_LS_AMBIENT		0x59

#define REG_BAT_CAPA_LOW	0x60
#define REG_BAT_EMER_LOW	0x61
#define REG_BAT_TEMP1_L		0x62
#define REG_BAT_TEMP1_H		0x63
#define REG_BAT_TEMP2_L		0x64
#define REG_BAT_TEMP2_H		0x65
#define REG_BAT_DECT_T		0x66
#define REG_BAT_CAPA		0x67
#define REG_BAT_VOLT_L		0x68
#define REG_BAT_VOLT_H		0x69
#define REG_BAT_TEMP_L		0x6A
#define REG_BAT_TEMP_H		0x6B
#define REG_BAT_CUR_L		0x6C
#define REG_BAT_CUR_H		0x6D
#define REG_PER_DEVICE		0x6E

#define SLAVE_ADDR		0x73

#define REG_SW_VERSION		0x7E
#define REG_DEVICE_ID 		0x7F
#define SLAVE_ID 		0x87

#define DEVICE_RESET		0x3

#define PULSE			1
#define FLAG			0

#define PWM_MAX			255
#define FIFO_LENGTH		14
#define KEY_RELEASED		0x8000
#define KEY_PRESSED			0x8000

/*
 * registers usage
 * ===============
 */

/*
 * REG_CTRL0 (0x40)
 *
 * MCU_MODE = 00b (normal), 01b (w/ idle), 10b (w/ sleep), 11b (reset)
 * HOST_STATUS = 1b (sleep), 0b (normal)
 * ENABLE_WAKEUP = 1b (enable), 0b (disable)
 * POWER_PS2 = 1b (on), 0b (off)
 * ENABLE_AUTOIDLE = 1b (enabled), 0b (disabled)
 *
 */
#define ENABLE_AUTOIDLE_SHIFT	5
#define ENABLE_AUTOIDLE_MASK	(0x1U<<ENABLE_AUTOIDLE_SHIFT)
#define POWER_PS2_SHIFT		4
#define POWER_PS2_MASK		(0x1U<<POWER_PS2_SHIFT)
#define ENABLE_WAKEUP_SHIFT	3
#define ENABLE_WAKEUP_MASK	(0x1U<<ENABLE_WAKEUP_SHIFT)
#define HOST_STATUS_SHIFT	2
#define HOST_STATUS_MASK	(0x1U<<HOST_STATUS_SHIFT)
#define MCU_MODE_SHIFT		0
#define MCU_MODE_MASK		(0x3U<<MCU_MODE_SHIFT)

#define MCU_MODE_NORMAL		0x0U
#define MCU_MODE_W_IDLE		0x1U
#define MCU_MODE_W_SLEEP	0x2U
#define MCU_MODE_RESET		0x3U

#define HOST_STATUS_NORMAL	0x0U
#define HOST_STATUS_SLEEP	0x1U

#define PS2_POWER_ON		0x1U
#define PS2_POWER_OFF		0x0U

/*
 * REG_CTRL1 (0x41)
 *
 * REPORT_Z = 1 to enable, 0 to disable
 * IRQ_TYPE = 00b pulse / 10b flag
 * IDLE_TIMEOUT = timeout(sec)-1(sec)
 */
#define REPORT_Z_SHIFT		6
#define REPORT_Z_MASK		(0x1 << REPORT_Z_SHIFT)
#define IRQ_TYPE_SHIFT		5
#define IRQ_TYPE_MASK		(0x3 << IRQ_TYPE_SHIFT)
#define IDLE_TIMEOUT_SHIFT	0
#define IDLE_TIMEOUT_MASK	(0xf << IDLE_TIMEOUT_SHIFT)

#define IRQ_TYPE_PULSE		0x0U
#define IRQ_TYPE_FLAG		0x2U

#define REPORT_Z_ON		0x1U
#define REPORT_Z_OFF		0x0U

/*
 * REG_ACCUM_ENABLE_X (0x42) on firmware >= 4.4
 * REG_ACCUM_ENABLE_Y (0x4E) on firmware >= 4.5
 *
 * D7 En_TP_Acc    Enable the accumulation of mouse (TP)
 *                 1: Enable
 *                 0: Disable
 * D6 TP_Acc6      Set X/Y-Axis threshold level of accumulation (Level=TP_Acc+1,
 *                 default 03 level=4)
 * D5 TP_Acc5
 * D4 TP_Acc4
 * D3 TP_Acc3
 * D2 TP_Acc2
 * D1 TP_Acc1
 * D0 TP_Acc0
 */
#define ENABLE_ACCUM_SHIFT	7
#define ENABLE_ACCUM_MASK	(0x1U<<ENABLE_ACCUM_SHIFT)
#define ACCUMULATOR_VALUE_SHIFT	0
#define ACCUMULATOR_VALUE_MASK	(0x7fU<<ACCUMULATOR_VALUE_SHIFT)

/*
 * REG_MOUSE_DATA (0x45)
 *
 * KEY_CLICK = 1 if if button is pressed
 *
 * X_SIGN = 1 if X_MOV is negative
 * Y_SIGN = 1 if Y_MOV is negative
 *
 * X_OVF = 1 if Y-axis overflow
 * Y_OVF = 1 if Y-axis overflow
 */
#define Y_OVF_SHIFT		7
#define Y_OVF_MASK		(0x1U << Y_OVF_SHIFT)
#define X_OVF_SHIFT		6
#define X_OVF_MASK		(0x1U << X_OVF_SHIFT)
#define Y_SIGN_SHIFT		5
#define Y_SIGN_MASK		(0x1U << Y_SIGN_SHIFT)
#define X_SIGN_SHIFT		4
#define X_SIGN_MASK		(0x1U << X_SIGN_SHIFT)
#define KEY_CLICK_SHIFT		0
#define KEY_CLICK_MASK		(0x1U << KEY_CLICK_SHIFT)

/*
 * REG_LED_CTRL (0x50)
 */

#define LED_STATE_SHIFT		5
#define LED_STATE_MASK		(0x7U << LED_STATE_SHIFT)
#define PWM3_CTRL_SHIFT		4
#define PWM3_CTRL_MASK		(0x1U << PWM3_CTRL_SHIFT)
#define BACKLIGHT_LEVEL_SHIFT	0
#define BACKLIGHT_LEVEL_MASK	(0xfU << BACKLIGHT_LEVEL_SHIFT)

#define LED_STATE_OFF		0x0U
#define LED_STATE_DEVICE_ERR	0x1U
#define LED_STATE_BATT_FULL	0x2U
#define LED_STATE_BATT_CHARGING	0x3U
#define LED_STATE_BATT_LOW	0x4U

/*
 * REG_WAKEUP (0x4c)
 */

#define TOUCHPAD_ANGLE_SHIFT	7
#define TOUCHPAD_ANGLE_MASK	(0x1U << TOUCHPAD_ANGLE_SHIFT)

#define TOUCHPAD_ANGLE_0	0x0U
#define TOUCHPAD_ANGLE_180	0x1U

#define WAKEUP_ON_CHARGE_SHIFT	6
#define WAKEUP_ON_CHARGE_MASK	(0x1U << WAKEUP_ON_CHARGE_SHIFT)

/*
 * REG_DECS_PULSE (0x4B)
 */

#define DECS_HIGH_SHIFT		0
#define DECS_HIGH_MASK		(0x7U << DECS_HIGH_SHIFT)
#define DECS_LOW_SHIFT		3
#define DECS_LOW_MASK		(0x1f << DECS_LOW_SHIFT)


/* ====================================
		CM3212 Commands definitions
   ====================================*/
/* Gain bit7/6  */
#define  GAIN_DIV_1		(0x0U  << 6)
#define  GAIN_DIV_2		(0x01U << 6)
#define  GAIN_DIV_4		(0x02U << 6)
#define  GAIN_DIV_8		(0x03U << 6)

/* IT: integration time setting  */
#define R_EXT_300K   // 300K Ohm for RSET(mock-up hw)
#ifdef  R_EXT_300K
#define	IT_50MS			(0x0U << 2)
#define	IT_100MS		(0x01U << 2)
#define	IT_200MS		(0x02U << 2)
#define	IT_400MS		(0x03U << 2)
#else   //  603K ohm
#define	IT_100MS		(0x0U << 2)
#define	IT_200MS		(0x01U << 2)
#define	IT_400MS		(0x02U << 2)
#define	IT_800MS		(0x03U << 2)
#endif

/* BYTE / WORD mode */
#define	WDM_BYTE_MODE	(0x0U << 1)
#define WDM_WORD_MODE	(0x1U << 1)

/* SD : Power down  */
#define 	SD_DISABLE			0x0U   /* power Up */
#define 	SD_ENABLE			0x1U	/* power save */

/*    CM3212  Power On / OFF ( to keep the other bits in command )  */
#define	CM3212_POWER_UP \
			(GAIN_DIV_8 | IT_100MS | WDM_BYTE_MODE | SD_DISABLE)
#define CM3212_POWER_DOWN \
			(GAIN_DIV_8 | IT_100MS | WDM_BYTE_MODE | SD_ENABLE)
#define CM3212_INIT_SETUP	CM3212_POWER_UP
/* [CM3212] end of cm3212 definition */


/*
 * exported data
 * =============
 */

struct ma87710_platform_data {
	u8 bl_pwm;
	u8 auto_on;
	u8 caplock_on;
	u8 mute_on;
	u8 leds_state;
	u8 gpio;
	u8 dim_status;
	u8 pwm_output_en;
};

/*
 * exported symbols
 * ================
 */

void ma87710_led_blink_set(int flag);
int ma87710_pwm_output_control(int pwm_on_off);
#endif // if 0

#endif	/* _MA87710_H_ */
