#define CY8CTMA395_DEVICE	"cy8ctma395"
#define CY8CTMA395_DRIVER	"cy8ctma395"

struct cy8ctma395_platform_data {
	int		(*swdck_request)(int request);
	int		(*swdio_request)(int request);
	void		(*vdd_enable)(int enable);
	unsigned	xres;
	unsigned long	xres_us;
	unsigned	swdck;
	unsigned	swdio;
	int		swd_wait_retries;
	int		port_acquire_retries;
	int		status_reg_timeout_ms;
	int		nr_blocks;
};
