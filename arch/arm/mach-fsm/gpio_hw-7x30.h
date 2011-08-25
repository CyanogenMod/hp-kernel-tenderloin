/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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

#ifndef __ARCH_ARM_MACH_MSM_GPIO_HW_7X30_H
#define __ARCH_ARM_MACH_MSM_GPIO_HW_7X30_H

/* output value */
#define GPIO_OUT_0         GPIO1_REG(0x00)   /* gpio  15-0   */
#define GPIO_OUT_1         GPIO2_REG(0x00)   /* gpio  43-16  */
#define GPIO_OUT_2         GPIO1_REG(0x04)   /* gpio  67-44  */
#define GPIO_OUT_3         GPIO1_REG(0x08)   /* gpio  94-68  */
#define GPIO_OUT_4         GPIO1_REG(0x0C)   /* gpio 106-95  */
#define GPIO_OUT_5         GPIO1_REG(0x50)   /* gpio 133-107 */
#define GPIO_OUT_6         GPIO1_REG(0xC4)   /* gpio 150-134 */
#define GPIO_OUT_7         GPIO1_REG(0x214)  /* gpio 181-151 */

/* same pin map as above, output enable */
#define GPIO_OE_0          GPIO1_REG(0x10)
#define GPIO_OE_1          GPIO2_REG(0x08)
#define GPIO_OE_2          GPIO1_REG(0x14)
#define GPIO_OE_3          GPIO1_REG(0x18)
#define GPIO_OE_4          GPIO1_REG(0x1C)
#define GPIO_OE_5          GPIO1_REG(0x54)
#define GPIO_OE_6          GPIO1_REG(0xC8)
#define GPIO_OE_7          GPIO1_REG(0x218)

/* same pin map as above, input read */
#define GPIO_IN_0          GPIO1_REG(0x34)
#define GPIO_IN_1          GPIO2_REG(0x20)
#define GPIO_IN_2          GPIO1_REG(0x38)
#define GPIO_IN_3          GPIO1_REG(0x3C)
#define GPIO_IN_4          GPIO1_REG(0x40)
#define GPIO_IN_5          GPIO1_REG(0x44)
#define GPIO_IN_6          GPIO1_REG(0xCC)
#define GPIO_IN_7          GPIO1_REG(0x21C)

/* same pin map as above, 1=edge 0=level interrup */
#define GPIO_INT_EDGE_0    GPIO1_REG(0x60)
#define GPIO_INT_EDGE_1    GPIO2_REG(0x50)
#define GPIO_INT_EDGE_2    GPIO1_REG(0x64)
#define GPIO_INT_EDGE_3    GPIO1_REG(0x68)
#define GPIO_INT_EDGE_4    GPIO1_REG(0x6C)
#define GPIO_INT_EDGE_5    GPIO1_REG(0xC0)
#define GPIO_INT_EDGE_6    GPIO1_REG(0xD0)
#define GPIO_INT_EDGE_7    GPIO1_REG(0x240)

/* same pin map as above, 1=positive 0=negative */
#define GPIO_INT_POS_0     GPIO1_REG(0x70)
#define GPIO_INT_POS_1     GPIO2_REG(0x58)
#define GPIO_INT_POS_2     GPIO1_REG(0x74)
#define GPIO_INT_POS_3     GPIO1_REG(0x78)
#define GPIO_INT_POS_4     GPIO1_REG(0x7C)
#define GPIO_INT_POS_5     GPIO1_REG(0xBC)
#define GPIO_INT_POS_6     GPIO1_REG(0xD4)
#define GPIO_INT_POS_7     GPIO1_REG(0x228)

/* same pin map as above, interrupt enable */
#define GPIO_INT_EN_0      GPIO1_REG(0x80)
#define GPIO_INT_EN_1      GPIO2_REG(0x60)
#define GPIO_INT_EN_2      GPIO1_REG(0x84)
#define GPIO_INT_EN_3      GPIO1_REG(0x88)
#define GPIO_INT_EN_4      GPIO1_REG(0x8C)
#define GPIO_INT_EN_5      GPIO1_REG(0xB8)
#define GPIO_INT_EN_6      GPIO1_REG(0xD8)
#define GPIO_INT_EN_7      GPIO1_REG(0x22C)

/* same pin map as above, write 1 to clear interrupt */
#define GPIO_INT_CLEAR_0   GPIO1_REG(0x90)
#define GPIO_INT_CLEAR_1   GPIO2_REG(0x68)
#define GPIO_INT_CLEAR_2   GPIO1_REG(0x94)
#define GPIO_INT_CLEAR_3   GPIO1_REG(0x98)
#define GPIO_INT_CLEAR_4   GPIO1_REG(0x9C)
#define GPIO_INT_CLEAR_5   GPIO1_REG(0xB4)
#define GPIO_INT_CLEAR_6   GPIO1_REG(0xDC)
#define GPIO_INT_CLEAR_7   GPIO1_REG(0x230)

/* same pin map as above, 1=interrupt pending */
#define GPIO_INT_STATUS_0  GPIO1_REG(0xA0)
#define GPIO_INT_STATUS_1  GPIO2_REG(0x70)
#define GPIO_INT_STATUS_2  GPIO1_REG(0xA4)
#define GPIO_INT_STATUS_3  GPIO1_REG(0xA8)
#define GPIO_INT_STATUS_4  GPIO1_REG(0xAC)
#define GPIO_INT_STATUS_5  GPIO1_REG(0xB0)
#define GPIO_INT_STATUS_6  GPIO1_REG(0xE0)
#define GPIO_INT_STATUS_7  GPIO1_REG(0x234)

#endif
