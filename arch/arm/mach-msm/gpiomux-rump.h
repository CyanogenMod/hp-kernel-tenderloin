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
 */
#ifndef __ARCH_ARM_MACH_MSM_GPIOMUX_8X60_H
#define __ARCH_ARM_MACH_MSM_GPIOMUX_8X60_H

#include "gpiomux.h"

void __init msm8x60_init_gpiomux(struct msm_gpiomux_configs *cfgs);
void msm8x60_gpiomux_lcdc_steadycfg(void);

extern struct msm_gpiomux_configs rump_gpiomux_cfgs[] __initdata;

#define UART1DM_RTS_GPIO		56
#define UART1DM_CTS_GPIO		55
#define UART1DM_RX_GPIO			54
#define UART1DM_TX_GPIO			53

#define BT_RST_N			138
#define BT_POWER			130
#define BT_WAKE				131
#define BT_HOST_WAKE			129

#define BT_RST_N_3G			122
#define BT_POWER_3G			110
#define BT_WAKE_3G			131
#define BT_HOST_WAKE_3G			50

#ifdef CONFIG_KEYBOARD_GPIO_PE
/* GPIO Keys */
#define CORE_NAVI_GPIO			40
#define VOL_UP_GPIO			103
#define VOL_DN_GPIO			104
#endif

#ifdef CONFIG_MAX8903B_CHARGER
/* max8903b control GPIOs */
#define MAX8903B_GPIO_DC_CHG_MODE	42
#define MAX8903B_GPIO_USB_CHG_MODE	133
#define MAX8903B_GPIO_USB_CHG_MODE_3G	134
#define MAX8903B_GPIO_USB_CHG_SUS	33
#define MAX8903B_GPIO_CHG_D_ISET_1	34
#define MAX8903B_GPIO_CHG_D_ISET_2	30
#define MAX8903B_GPIO_CHG_EN		41
#define MAX8903B_GPIO_DC_OK		140
#define MAX8903B_GPIO_DC_OK_3G		86
#define MAX8903B_GPIO_STATUS_N		36
#define MAX8903B_GPIO_FAULT_N		35
#endif  //CONFIG_MAX8903B_CHARGER

#define MXT1386_TS_PEN_IRQ_GPIO		123
#define MXT1386_TS_PEN_IRQ_GPIO_3G	45
#define MXT1386_TS_PWR_RST_GPIO		70

#define GPIO_CTP_INT			123
#define GPIO_CTP_SCL			73
#define GPIO_CTP_SDA			72
#define GPIO_CTP_RX			71
#define GPIO_CY8CTMA395_XRES		70

#define RUMP_GPIO_WL_HOST_WAKE	93
#define	RUMP_GPIO_WL_IRQ		94
#define RUMP_GPIO_HOST_WAKE_WL	137
#define RUMP_GPIO_WLAN_RST_N	135
#define RUMP_GPIO_WLAN_RST_N	135
#define RUMP_GPIO_WLAN_RST_N_3G	28

/* a6 */
#define RUMP_A6_0_TCK		157
#define RUMP_A6_0_WAKEUP		155
#define RUMP_A6_0_TDIO		158
#define RUMP_A6_0_MSM_IRQ		156

#define RUMP_A6_1_TCK		115
#define RUMP_A6_1_WAKEUP		141
#define RUMP_A6_1_TDIO		116
#define RUMP_A6_1_MSM_IRQ		132


#define RUMP_A6_0_TCK_3G		68
#define RUMP_A6_0_WAKEUP_3G	155
#define RUMP_A6_0_TDIO_3G		170
#define RUMP_A6_0_MSM_IRQ_3G	156

#define RUMP_A6_1_TCK_3G		115
#define RUMP_A6_1_WAKEUP_3G	78
#define RUMP_A6_1_TDIO_3G		116
#define RUMP_A6_1_MSM_IRQ_3G	132

/* camera */
#define RUMP_CAM_I2C_DATA		47
#define RUMP_CAM_I2C_CLK		48
#define RUMP_CAMIF_MCLK		32
#define RUMP_WEBCAM_RST		106
#define RUMP_WEBCAM_PWDN		107

/* gyro */
#define RUMP_GYRO_INT		125
#define RUMP_GYRO_INT_3G		75

/* audio */
#define RUMP_AUD_LDO1_EN	66
#define RUMP_AUD_LDO2_EN	108

/* lighting */
#define LM8502_LIGHTING_INT_IRQ_GPIO 	128
#define LM8502_LIGHTING_EN_GPIO 	121
#define LM8502_LIGHTING_INT_IRQ_GPIO_3G	77

/* usb host */
#define ISP1763_INT_GPIO		172
#define ISP1763_DACK_GPIO		169
#define ISP1763_DREQ_GPIO		29
#define ISP1763_RST_GPIO		152
#define GPIO_3G_3V3_EN			158
#define GPIO_3G_DISABLE_N		171
#define GPIO_3G_WAKE_N			38
#define GPIO_3G_UIM_CD_N		61
#endif
