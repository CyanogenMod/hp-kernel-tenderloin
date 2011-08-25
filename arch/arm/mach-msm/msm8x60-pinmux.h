#ifndef _MSM_PINMUX_H
#define _MSM_PINMUX_H

struct pin_config {
	char *name;
	unsigned int current_mode;
	unsigned int active_mode;
	unsigned int sleep_mode;

	int active_power_collapse;
};

extern struct pin_config *msm8x60_pins;
extern int msm8x60_pins_sz;
extern struct pinmux_ops msm8x60_pinmux_ops;

#endif
