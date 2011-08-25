#ifndef  __USER_PINS_INCLUDED__
#define  __USER_PINS_INCLUDED__

typedef enum {
	PIN_MODE_ACTIVE,
	PIN_MODE_SUSPENDED
} PIN_MODE;

struct user_pin {
	const char *name;  // pin name 
	int gpio;          // gpio num/id
	int options;       // options
	int act_level;     // active level
	int direction;     // 1 - an input, 0 - output
	int def_level;     // default level: 0, 1 or -1 if undefined
	int sysfs_mask;    // sysfs file mode
	char *pin_mode;    // board specific pin mode
	irqreturn_t (*irq_handler)(int irq, void *data);
	int (*pinmux)(int gpio, int mode);
	int irq_config;
	int irq_handle_mode;
};

struct user_pin_set {
	const char  *set_name;   // pin set name  
	int          num_pins;   // number of pins in the group 
	struct user_pin *pins;   // pins array.
};

struct user_pins_platform_data {
	int              num_sets;   // number of pin sets 
	struct user_pin_set *sets;   // pin sets.
};

/* Pin option constants */
#define PIN_READ_ONLY		(1 << 0)    //  pin is read only
#define PIN_WAKEUP_SOURCE	(1 << 1)    //  pin is a wakeup source
#define PIN_IRQ			(1 << 2)    //  pin generates irq

#define IRQ_HANDLE_NONE         (0)         //  IRQ handling is not defined
#define IRQ_HANDLE_AUTO         (1 << 0)    //  IRQ handling is automatic
#define IRQ_HANDLE_OFF          (1 << 1)    //  IRQ handling is off

#endif // __USER_PINS_INCLUDED__
