/*
 *  include/linux/cpufreq_tickle.h
 *
 *  Userspace interface to cpufreq_ondemand_tickle.
 *
 *  Copyright (C) 2009 Palm, Inc.
 *                     Corey Tabaka <corey.tabaka@palm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _LINUX_CPUFREQ_TICKLE_H
#define _LINUX_CPUFREQ_TICKLE_H

#include <linux/ioctl.h>

/*
 * The 'T' after TICKLE_IOC stands for "Tell" and denotes a direct argument value.
 */
#define TICKLE_IOC_MAGIC 'k'
#define TICKLE_IOCT_TICKLE			_IO(TICKLE_IOC_MAGIC, 0)
#define TICKLE_IOCT_FLOOR			_IO(TICKLE_IOC_MAGIC, 1)
#define TICKLE_IOC_TICKLE_HOLD		_IO(TICKLE_IOC_MAGIC, 2)
#define TICKLE_IOC_TICKLE_UNHOLD	_IO(TICKLE_IOC_MAGIC, 3)
#define TICKLE_IOCT_FLOOR_HOLD		_IO(TICKLE_IOC_MAGIC, 4)
#define TICKLE_IOC_FLOOR_UNHOLD		_IO(TICKLE_IOC_MAGIC, 5)
#define TICKLE_IOC_TICKLE_HOLD_SYNC	_IO(TICKLE_IOC_MAGIC, 6)
#define TICKLE_IOC_MAXNR 6

#endif /* _LINUX_CPUFREQ_TICKLE_H */
