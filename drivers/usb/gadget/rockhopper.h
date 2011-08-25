#ifndef	__ROCKHOPPER_H
#define	__ROCKHOPPER_H

struct device *rockhopper_create_device(const char *name);
void rockhopper_destroy_device(struct device *dev);

#endif	/* __ROCKHOPPER_H */
