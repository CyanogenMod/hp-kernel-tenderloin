#ifndef _PINMUX_H_
#define _PINMUX_H_

enum
{
	PINMUX_CONFIG_UNDEFINED = 0,
	PINMUX_CONFIG_ACTIVE,
	PINMUX_CONFIG_SLEEP,
	PINMUX_CONFIG_POWER_COLLAPSE,
};
typedef unsigned int pinmux_cfg;

struct pinmux_ops;

/** 
* @brief Register architecture specific pinmux functions.
* 
* @param  funcs 
* 
* @retval
*/
int pinmux_register(struct pinmux_ops *funcs);

/** 
* @brief Configure named pin.
* 
* @param  name 
* @param  cfg 
* 
* @retval
*/
int pinmux_config(const char *name, pinmux_cfg cfg);

/** 
* @brief Configure named pin for power collapse.
*
* Equivalent to pinmux_config(name, PINMUX_CONFIG_POWER_COLLAPSE).
* 
* @param  name 
* 
* @retval
*/
int pinmux_config_power_collapse(const char *name);


/** 
* @brief Set mode when in power collapse to be active.
* 
* @param  name 
* @param  active 
* 
* @retval
*/
int pinmux_set_power_collapse(const char *name, int active);

/** 
* @brief Print pinmux table.
*/
void pinmux_dump(void);

/** 
* @brief Pinmux arch-specific operations
*/
struct pinmux_ops
{
	typeof(pinmux_config)             *config;
	typeof(pinmux_dump)               *dump;
	typeof(pinmux_set_power_collapse) *set_power_collapse;
};

#endif // _PINMUX_H_
