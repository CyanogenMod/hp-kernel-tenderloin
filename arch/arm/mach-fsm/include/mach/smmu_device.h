/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef SMMU_DEVICE_H
#define SMMU_DEVICE_H

#include <mach/smmu_driver.h>

/* Maximum number of Machine IDs that we are allowing to be mapped to the same
 * context bank. The number of MIDs mapped to the same CB does not affect
 * performance, but there is a practical limit on how many distinct MIDs may
 * be present. These mappings are typically determined at design time and are
 * not expected to change at run time.
 */
#define MAX_NUM_MIDS	16

/**
 * struct smmu_device - a single SMMU hardware instance
 * name		Human-readable name given to this SMMU HW instance
 * clk		Name of the AXI clock used for the config space of this SMMU
 * clk_rate	Rate to set for the AXI clock. 0 means don't set a rate
 * num_ctx	Number of context banks on this SMMU device
 *
 */
struct smmu_device {
	char *name;
	char *clk;
	unsigned long clk_rate;
	unsigned int num_ctx;
};

/**
 * struct smmu_ctx - an SMMU context bank instance
 * name		Human-readable name given to this context bank
 * num		Index of this context bank within the hardware
 * mids		List of Machine IDs that are to be mapped into this context
 *		bank, terminated by -1. The MID is a set of signals on the
 *		AXI bus that identifies the function associated with a specific
 *		memory request. (See ARM spec).
 */
struct smmu_ctx {
	char *name;
	int num;
	int mids[MAX_NUM_MIDS];
};

/**
 * smmu_get_ctx_instance() - Find and initialize a context instance by name
 * @name - the name of the context bank as defined in the board file
 *
 * Instantiates a handle to a context instance specified by the given name.
 * The name must correspond to one of the contexts defined as platform devices.
 * The returned instance should be freed using smmu_free_ctx_instance when it
 * is no longer needed.
 *
 * Returns a pointer to a new smmu_dev structure, or NULL on failure.
 */
struct smmu_dev *smmu_get_ctx_instance(char *ctx_name);

/**
 * smmu_free_ctx_instance() - frees a previously allocated context instance
 * @dev		The context instance to free
 *
 * Deallocates an smmu context instance that was previously allocated using
 * smmu_get_ctx_instance. The context instance must not have any mappings and
 * must not be active.
 *
 * Returns zero on success and -ENODEV for a bad input, and -EBUSY for a device
 * that is still active or still has mappings.
 */
int smmu_free_ctx_instance(struct smmu_dev *dev);


/**
 * smmu_get_base_addr() - Retrieve the base address of an SMMU HW block
 * @dev		The SMMU device whose base address is requested
 *
 * The return value is the (virtual) address of the SMMU config block,
 * or -1 if the input parameter is not a valid smmu device pointer.
 */
unsigned long smmu_get_base_addr(struct smmu_dev *dev);

#endif
