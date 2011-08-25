#ifndef _GPIO_KEYS_H
#define _GPIO_KEYS_H

struct gpio_keys_button {
	/* Configuration parameters */
	int code;		/* input event code (KEY_*, SW_*) */
	int gpio;
	int active_low;
	char *desc;
	int debounce;		/* debounce interval (msec) */
	int type;		/* input event type (EV_KEY, EV_SW) */
	int wakeup;		/* configure the button as a wake-up source */
	int options;		/* device specific options */
	int noise_mode;		/* mode to determine how to interpret noise */
	int (*is_wake_source)(int gpio);	/* callback to see if this button woke us up */
	int gpio_controller_cansleep;
};

struct gpio_keys_platform_data {
	struct gpio_keys_button *buttons;
	int nbuttons;
};

#define OPT_REBOOT_TRIGGER       (1 << 0)  // can be reboot trigger  
#define OPT_REBOOT_TRIGGER_LEVEL (0)       // triggered by level
#define OPT_REBOOT_TRIGGER_EDGE  (1 << 1)  // triggered by edge

#define OPT_CONSOLE_TRIGGER      (1 << 2)  // can be console switch trigger

/*
 * The following options are used to determine the case when interrupt fires, and
 * there is no change in state; whether it should be interpreted as missing 
 * interrupt (OPT_MISSING_INT_INTERPRETATION) or noise on the line (OPT_NOISE_INTERPRETATION)
 */
#define MODE_NOISE_INTERPRETATION (1 << 0)
#define MODE_MISSING_INT_INTERPRETATION (1 << 1)

#endif
