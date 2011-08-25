#include <linux/pinmux.h>
#include <linux/errno.h>

static struct pinmux_ops *the_pinmux_ops;

int pinmux_register(struct pinmux_ops *ops)
{
	the_pinmux_ops = ops;
	return 0;
}

int pinmux_config(const char *name, pinmux_cfg cfg)
{
	if (the_pinmux_ops && the_pinmux_ops->config) {
		the_pinmux_ops->config(name, cfg);
		return 0;
	}
	return -EINVAL;
}

int pinmux_config_power_collapse(const char *name)
{
	return pinmux_config(name, PINMUX_CONFIG_POWER_COLLAPSE);
}

int pinmux_set_power_collapse(const char *name, int active)
{
	if (the_pinmux_ops && the_pinmux_ops->set_power_collapse) {
		the_pinmux_ops->set_power_collapse(name, active);
		return 0;
	}
	return -EINVAL;
}


