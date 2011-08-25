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
#ifndef __ARCH_ARM_MACH_MSM_DEVICES_MSM8X60_H
#define __ARCH_ARM_MACH_MSM_DEVICES_MSM8X60_H

#define MSM_GSBI3_QUP_I2C_BUS_ID 0
#define MSM_GSBI4_QUP_I2C_BUS_ID 1
#define MSM_GSBI9_QUP_I2C_BUS_ID 2
#define MSM_GSBI8_QUP_I2C_BUS_ID 3
#define MSM_GSBI7_QUP_I2C_BUS_ID 4
#define MSM_SSBI1_I2C_BUS_ID     6
#define MSM_SSBI2_I2C_BUS_ID     7
#define MSM_SSBI3_I2C_BUS_ID     8

#ifdef CONFIG_SPI_QUP
extern struct platform_device msm_gsbi1_qup_spi_device;
#endif

extern struct platform_device msm_device_smd;
extern struct platform_device msm_device_kgsl;
extern struct platform_device msm_device_gpio;
extern struct platform_device msm_device_vidc;

#endif
