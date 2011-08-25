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

#ifndef SMMU_DRIVER_H
#define SMMU_DRIVER_H

#include <linux/vcm_types.h>
#include <linux/list.h>

#define FL_OFFSET(va)	(((va) & 0xFFF00000) >> 20)
#define SL_OFFSET(va)	(((va) & 0xFF000) >> 12)

#define NUM_FL_PTE 	4096
#define NUM_SL_PTE	256

/**
 * struct smmu_driver - A single SMMU hardware instance
 *
 * base			SMMU config port base address
 * irq			Interrupt number
 * list_active		List of activated context structures on this SMMU
 *
 * A smmu_driver holds the global information about a single piece
 * of SMMU hardware.
 *
 */
struct smmu_driver {
	unsigned long base;
	int irq;
	struct list_head list_active;
};


/**
 * struct smmu_dev - A single SMMU context instance
 *
 * drv			  Pointer to hardware instance for this context
 * context		  The context number (index) for this context
 * base			  Config port base address of SMMU (a copy of drv->base)
 * fl_table		  First-level page table associated with this mapping
 *			  (this is a virtual address)
 * smmu_vcm_handler	  Interrupt handler for faults in this context
 * smmu_vcm_handler_data  Private data for smmu_vcm_handler
 * dev_elm		  List of all contexts on this SMMU instance
 *
 * A smmu_dev holds the mapping-specific information associated with each
 * context of the SMMU.
 *
 */
struct smmu_dev {
	int base;
	int context;
	vcm_handler smmu_vcm_handler;
	void *smmu_vcm_handler_data;
	unsigned long *fl_table;
	struct list_head dev_elm;
	struct smmu_driver *drv;
};

void v7_flush_kern_cache_all(void);

/**
 * smmu_drvdata_init() - Initialize a SMMU hardware instance
 * @drv		Pointer to a newly-allocated smmu_driver structure that is to
 *		be populated with information regarding this HW instance
 * @base	The base address of the SMMU's config port
 * @irq		The interrupt number associated with this SMMU's context
 * 		interrupt #0.
 *
 * Initialize an SMMU hardware instance and the handle to it.
 *
 * The return value is zero on success and non-zero on failure.
 */
int smmu_drvdata_init(struct smmu_driver *drv, unsigned long base, int irq);


/**
 * smmu_ctx_init() - Initialize an SMMU context
 * @ctx		The context number (index) to initialize
 *
 * Create a handle to an SMMU context. The SMMU must have already been
 * initialized using smmu_drvdata_init and the context number must be a valid
 * context index on that particular SMMU hardware instance.
 *
 * The return value is a pointer to a smmu_dev structure, which represents
 * a handle to this context. The return value will be NULL if the memory could
 * not be allocated or if the parameters were not valid.
 */
struct smmu_dev *smmu_ctx_init(int ctx);


/**
 * smmu_ctx_bind() - associate a smmu_dev with a specific SMMU HW instance
 * @ctx 	The context instance to bind to the HW instance
 * @drv		The HW instance to which the context instance is to be bound
 *
 * Associate the context instance structure with a particular hardware instance
 * The parameters must be valid pointers to their respective structures and the
 * specific context instance must not have already been bound to a DIFFERENT
 * hardware instance. The context instance must have been initialized with a
 * valid context number for the given hardware instance.
 *
 * The return value is zero on success and non-zero on failure.
 */
int smmu_ctx_bind(struct smmu_dev *ctx, struct smmu_driver *drv);


/**
 * smmu_ctx_deinit() - Tear down an SMMU context
 * @dev		The smmu_dev to tear down
 *
 * Tears down the SMMU context and frees the associated memory. The context
 * mapping must have already been deactivated (if it had been activated prior)
 * using smmu_deactivate().
 *
 * The return value is 0 if the context was successfully deinitialized or
 * nonzero if the operation could not be completed. The memory allocated for
 * the smmu_dev structure will be freed and will no longer be valid after this
 * function returns 0.
 */
int smmu_ctx_deinit(struct smmu_dev *dev);



/**
 * smmu_get_driver() - Retrieves the smmu_driver for this context
 * @dev		The smmu_dev structure whose smmu_driver is needed
 *
 * Returns a pointer to the smmu_driver structure associated with a given
 * smmu_dev (context), or NULL if the context passed in is not valid.
 */
struct smmu_driver *smmu_get_driver(struct smmu_dev *dev);


/**
 * smmu_activate() - Activates a context mapping
 * @dev		The context to activate
 *
 * Enables the mapping associated with this context. After this function returns
 * successfully, the hardware requests associated with this SMMU instance and
 * this context will be translated according to the mapping specified for this
 * context.
 *
 * Returns 0 if the activation was successful and nonzero otherwise.
 */
int smmu_activate(struct smmu_dev *dev);


/**
 * smmu_deactivate() - Deactivates a context mapping
 * @dev		The context to deactivate
 *
 * Disables the mapping associated with this context. Hawrdware requests
 * associated with this SMMU instance and this context will effectively
 * bypass the SMMU.
 *
 * Returns 0 if the deactivation was successful and nonzero otherwise.
 */
int smmu_deactivate(struct smmu_dev *dev);


/**
 * smmu_is_active() - Tests if a context mapping is activated
 * @dev		The smmu_dev structure to check
 *
 * Returns >0 if the mapping is activated, 0 if it is not activated, and <0 if
 * the supplied smmu_dev is not a valid context instance.
 */
int smmu_is_active(struct smmu_dev *dev);


/**
 * smmu_update_start() - Prepare to update mappings
 * @dev		The context instance whose mappings will be updated
 *
 * This function must be called prior to making any smmu_map or smmu_unmap calls
 * It is an error to allow a SMMU client to issue bus requests while mappings
 * are being updated (that is, if there is an outstanding smmu_update_start
 * call without a corresponding smmu_update_done call). The translation result
 * in this case is UNPREDICTABLE due to potentially stale TLB entries.
 * It is an error to call smmu_map or smmu_unmap unless a new call to
 * smmu_update_start (not followed by smmu_update_done) has been made.
 * The return value is zero on success and non-zero on failure.
 */
int smmu_update_start(struct smmu_dev *dev);


/**
 * smmu_update_done() - Finalize mapping updates
 * @dev		The context instance whose mappings have been updated
 *
 * This function must be called after making any smmu_map or smmu_unmap calls.
 * It must correspond to a previous outstanding smmu_update_start call. Note
 * that the mapping changes MIGHT take effect before smmu_update_done is called.
 * It is a programming error to allow an SMMU client to issue translation
 * requests if there is an outstanding call to smmu_update_start without being
 * followed by smmu_update_done.
 *
 * The return value is zero on success and non-zero on failure.
 */
int smmu_update_done(struct smmu_dev *dev);


/**
 * smmu_map() - Maps a virtual page into physical space
 * @dev		The context to in which to map the page
 * @pa 		The physical address for the mapping
 * @va		The virtual address to map
 * @len		The page size (4KB, 64KB, 1MB, or 16MB only).
 * @attr	Mapping attributes (cacheability / shareability)
 *
 * Maps the specified virtual page to the specified physical page. The page
 * size must be one of 4KB, 64KB, 1MB, or 16MB. The physical and virtual
 * addresses must be aligned to the page size specified. Care must be taken so
 * that no hardware requests are made within this context while this function
 * runs if the context is active. The function will return an error if the given
 * virtual address is already part of a different mapping. The attribute is one
 * of the VCM_DEV_xxx shareability attributes ORed with one of the VCM_DEV_xxx
 * cacheability attributes.
 *
 * This function may be called multiple times using the same physical and
 * virtual addresses (without first calling smmu_unmap), as long as the
 * length is UNCHANGED across all such invocations. The attributes may be
 * different, in which case the new attributes replace the previous ones.
 *
 * Returns 0 if mapping was successfully made or nonzero if the parameters
 * were not valid or resources could not be allocated, or if a conflicting
 * mapping had already been made for that virtual address.
 */
int smmu_map(struct smmu_dev *dev, unsigned long pa, unsigned long va,
	     unsigned long len, unsigned int attr);


/**
 * __smmu_map() - smmu_map() without locking
 */
int __smmu_map(struct smmu_dev *dev, unsigned long pa, unsigned long va,
	       unsigned long len, unsigned int attr);


/**
 * smmu_unmap() - Removes a page mapping
 * @dev		  The context to in which to unmap the page
 * @va		  The virtual address to unmap
 * @len		  The page size (4KB, 64KB, 1MB, or 16MB only).
 *
 * Removes the specified page mapping. The virtual address and the length must
 * be the same as the va and the length that were passed to smmu_map when this
 * mapping had been set up. Care must be taken so that no hardware requests are
 * made within this context while this function runs if the context is active.
 *
 * Returns 0 if mapping was successfully removed or nonzero if the parameters
 * were not valid, or if the specified mapping had been previously removed.
 */
int smmu_unmap(struct smmu_dev *dev, unsigned long va, unsigned long len);


/**
 * smmu_translate() - Performs a VA to PA translation on a single address
 * @dev		  The context to in which to perform the translation
 * @va		  The virtual address to translate
 *
 * Translates the virtual address to a physical address using the SMMU hardware
 * and the mappings that have been configured for this context. The context
 * must have been activated before calling this function. If the specified
 * virtual address has not been mapped, a user-supplied interrupt handler will
 * be called.
 *
 * Returns the physical address associated with the given virtual address. If
 * no relevant mapping exists, an interrupt is raised *and* -1 (0xFFFFFF...) is
 * immediately returned to indicate the error.
 */
unsigned long smmu_translate(struct smmu_dev *dev, unsigned long va);


/**
 * smmu_hook_irpt() - Registers an interrupt handler for this context
 * @dev		  The context for which to register the interrupt
 * @vcm_handler	  Pointer to the user-supplied interrupt handler
 * @data	  Data to be supplied to the interrupt handler
 *
 * Registers an interrupt handler to be called when a fault occurs within the
 * current context. Registering a different interrupt handler for the same
 * context will result in the new handler replacing the old one. Registering a
 * NULL handler will disable the user interrupt handler for this context.
 *
 * Returns 0 if the interrupt handler was successfully registered or nonzero if
 * the smmu_dev passed in was not valid.
 */
int smmu_hook_irpt(struct smmu_dev *dev, vcm_handler handler, void *data);

#endif /* SMMU_DRIVER_H */
