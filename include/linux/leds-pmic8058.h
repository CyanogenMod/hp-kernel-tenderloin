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
#ifndef __LEDS_PMIC8058_H__
#define __LEDS_PMIC8058_H__

enum pmic8058_leds {
	PMIC8058_ID_LED_KB_LIGHT = 1,
	PMIC8058_ID_LED_0,
	PMIC8058_ID_LED_1,
	PMIC8058_ID_LED_2,
	PMIC8058_ID_FLASH_LED_0,
	PMIC8058_ID_FLASH_LED_1,
};

struct pmic8058_led {
	const char	*name;
	const char	*default_trigger;
	unsigned	max_brightness;
	int		id;
};

struct pmic8058_leds_platform_data {
	int	num_leds;
	struct pmic8058_led *leds;
};

int pm8058_set_flash_led_current(enum pmic8058_leds id, unsigned mA);
int pm8058_set_led_current(enum pmic8058_leds id, unsigned mA);

#endif /* __LEDS_PMIC8058_H__ */
