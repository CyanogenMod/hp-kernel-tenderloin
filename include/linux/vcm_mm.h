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

/* Architecture-specific VCM functions */

/* Device attributes */

/*
 * Sharing attributes. Pick only one.
 */
#define VCM_DEV_ATTR_NON_SH 	(0x00)
#define VCM_DEV_ATTR_SH		(0x04)

/*
 * Caching attributes. Pick only one.
 */
#define VCM_DEV_ATTR_NONCACHED		(0x00)
#define VCM_DEV_ATTR_CACHED_WB_WA	(0x01)
#define VCM_DEV_ATTR_CACHED_WB_NWA	(0x02)
#define VCM_DEV_ATTR_CACHED_WT		(0x03)

/*
 * A "good" default set of attributes: shareable and non-cacheable.
 */
#define VCM_DEV_DEFAULT_ATTR	(VCM_DEV_ATTR_SH | VCM_DEV_ATTR_NONCACHED)

/**
 * set_arm7_pte_attr() - Set ARMv7 page table attributes
 * pt_base	Virtual address of the first-level page table
 * @va		Virtual address whose attributes are to be set
 * @len		Page size used to map the given virtual address
 * @attr	Attributes to set for this mapping.
 *
 * Modify a mapping attribute. The base address of the page table must
 * be a virtual address containing a valid ARMv7 page table.  The
 * virtual address must refer to an existing mapping and must be
 * aligned to the length with which it was mapped. The mapping length
 * must similarly be the same as was specified when the mapping was
 * made (one of 4KB, 64KB, 1MB, or 16MB). The attribute must be one of
 * the shareability attributes above ORed with one of the cacheability
 * attributes. Any previous attributes are completely replaced by the
 * most recent call to this function. This function only sets the
 * cacheability and shareability attributes. This is accomplished by
 * modifying the TEX class and the S bit in the PTE. It is an error to
 * call this function without having called vcm_setup_tex_classes at
 * least once.
 *
 * The return value is zero on success and non-zero on failure.
 */
int set_arm7_pte_attr(unsigned long pt_base, unsigned long va,
		     unsigned long len,	unsigned int attr);


/**
 * cpu_set_attr() - Set page table attributes on the CPU's page tables
 * @va		Virtual address whose attributes are to be set
 * @len		Page size used to map the given virtual address
 * @attr	Attributes to set for this mapping.
 *
 * Modify a mapping attribute within the ARM page tables. The va must
 * refer to an existing mapping and must be aligned to the length with
 * which it was mapped. The mapping length must similarly be the same
 * as was specified when the mapping was made (one of 4KB, 64KB, 1MB,
 * or 16MB). The attribute must be one of the shareability attributes
 * above ORed with one of the cacheability attributes. Any previous
 * attributes are completely replaced by the most recent call to this
 * function. This function only sets the cacheability and shareability
 * attributes. This is accomplished by modifying the TEX class and the
 * S bit in the PTE. It is an error to call this function without
 * having called vcm_setup_tex_classes at least once. It is an error
 * to call this function on any system using a memory configuration
 * that is anything OTHER than ARMv7 with TEX remap enabled. Only the
 * HW page tables are modified; the Linux page tables are left
 * untouched.
 *
 * The return value is zero on success and non-zero on failure.
 */
int cpu_set_attr(unsigned long va, unsigned long len, unsigned int attr);


/**
 * vcm_setup_tex_classes() - Prepare TEX class table for use
 *
 * Initialize the attribute mapping table by examining the TEX classes
 * used by the CPU and finding the classes that match the device
 * attributes (VCM_DEV_xx) defined above. This function is only
 * relevant if TEX remap is enabled. The results will be unpredictable
 * and irrelevant if TEX remap is not in use. It is an error to call
 * this function in any system using a memory configuration of
 * anything OTHER than ARMv7 with TEX remap enabled.
 *
 * The return value is zero on success or non-zero on failure. In the
 * present version, a failure will result in a panic.
 */
int vcm_setup_tex_classes(void);
