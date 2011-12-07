/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include "kgsl.h"
#include "kgsl_log.h"
#include "kgsl_cmdstream.h"
#include "kgsl_sharedmem.h"
#include "kgsl_yamato.h"

int kgsl_cmdstream_close(struct kgsl_device *device)
{
	struct kgsl_mem_entry *entry, *entry_tmp;

	BUG_ON(!mutex_is_locked(&device->mutex));

	list_for_each_entry_safe(entry, entry_tmp, &device->memqueue, list) {
		list_del(&entry->list);
		kgsl_destroy_mem_entry(entry);
	}
	return 0;
}

uint32_t
kgsl_cmdstream_readtimestamp(struct kgsl_device *device,
			     enum kgsl_timestamp_type type)
{
	uint32_t timestamp = 0;

	if (type == KGSL_TIMESTAMP_CONSUMED)
		KGSL_CMDSTREAM_GET_SOP_TIMESTAMP(device,
						 (unsigned int *)&timestamp);
	else if (type == KGSL_TIMESTAMP_RETIRED)
		KGSL_CMDSTREAM_GET_EOP_TIMESTAMP(device,
						 (unsigned int *)&timestamp);
	rmb();

	return timestamp;
}

