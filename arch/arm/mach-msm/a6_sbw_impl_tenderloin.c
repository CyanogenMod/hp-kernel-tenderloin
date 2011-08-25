#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include "gpiomux-tenderloin.h"

/*
A6_TCK_GPIO 	157
A6_TDIO_GPIO 	158
A6_WAKEUP_GPIO 	155
A6_MSM_INT_GPIO	156
*/

static int *a6_pin_table = NULL;

void a6_boardtype(int *pin_table)
{
	a6_pin_table = pin_table;
}

uint16_t a6_0_set_sbwtck(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(a6_pin_table[TENDERLOIN_A6_0_TCK_PIN], 1);
	return 0;
}

uint16_t a6_0_clr_sbwtck(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(a6_pin_table[TENDERLOIN_A6_0_TCK_PIN], 0);
	return 0;
}

uint16_t a6_0_set_sbwtdio(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(a6_pin_table[TENDERLOIN_A6_0_TDIO_PIN], 1);
	return 0;
}

uint16_t a6_0_clr_sbwtdio(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(a6_pin_table[TENDERLOIN_A6_0_TDIO_PIN], 0);
	return 0;
}

uint16_t a6_0_set_in_sbwtdio(void)
{
	gpio_direction_input(a6_pin_table[TENDERLOIN_A6_0_TDIO_PIN]);
	return 0;
}

uint16_t a6_0_set_out_sbwtdio(void)
{
	gpio_direction_output(a6_pin_table[TENDERLOIN_A6_0_TDIO_PIN], 0);
	return 0;
}

uint16_t a6_0_get_sbwtdio(void)
{
	return gpio_get_value(a6_pin_table[TENDERLOIN_A6_0_TDIO_PIN]);
}

uint16_t a6_0_set_sbwakeup(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(a6_pin_table[TENDERLOIN_A6_0_WAKEUP_PIN], 1);
	return 0;
}

uint16_t a6_0_clr_sbwakeup(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(a6_pin_table[TENDERLOIN_A6_0_WAKEUP_PIN], 0);
	return 0;
}


/*
A6_TCK_GPIO 	115
A6_TDIO_GPIO 	116
A6_WAKEUP_GPIO 	141
A6_MSM_INT_GPIO	132
*/

uint16_t a6_1_set_sbwtck(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(a6_pin_table[TENDERLOIN_A6_1_TCK_PIN], 1);
	return 0;
}

uint16_t a6_1_clr_sbwtck(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(a6_pin_table[TENDERLOIN_A6_1_TCK_PIN], 0);
	return 0;
}

uint16_t a6_1_set_sbwtdio(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(a6_pin_table[TENDERLOIN_A6_1_TDIO_PIN], 1);
	return 0;
}

uint16_t a6_1_clr_sbwtdio(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(a6_pin_table[TENDERLOIN_A6_1_TDIO_PIN], 0);
	return 0;
}

uint16_t a6_1_set_in_sbwtdio(void)
{
	gpio_direction_input(a6_pin_table[TENDERLOIN_A6_1_TDIO_PIN]);
	return 0;
}

uint16_t a6_1_set_out_sbwtdio(void)
{
	gpio_direction_output(a6_pin_table[TENDERLOIN_A6_1_TDIO_PIN], 0);
	return 0;
}

uint16_t a6_1_get_sbwtdio(void)
{
	return gpio_get_value(a6_pin_table[TENDERLOIN_A6_1_TDIO_PIN]);
}

uint16_t a6_1_set_sbwakeup(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(a6_pin_table[TENDERLOIN_A6_1_WAKEUP_PIN], 1);
	return 0;
}

uint16_t a6_1_clr_sbwakeup(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(a6_pin_table[TENDERLOIN_A6_1_WAKEUP_PIN], 0);
	return 0;
}

/****/
/* per-target functions */
/****/
// delay in usecs
void a6_delay_impl(uint32_t delay_us)
{
	if ((delay_us >> 10) <= MAX_UDELAY_MS) {
		udelay(delay_us);
	}
	else {
		mdelay(delay_us >> 10);
	}
}

