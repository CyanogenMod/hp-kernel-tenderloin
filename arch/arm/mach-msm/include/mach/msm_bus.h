/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, and the entire permission notice in its entirety,
 *    including the disclaimer of warranties.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * ALTERNATIVELY, this product may be distributed under the terms of
 * the GNU General Public License, version 2, in which case the provisions
 * of the GPL version 2 are required INSTEAD OF the BSD license.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ALL OF
 * WHICH ARE HEREBY DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF NOT ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#ifndef _ARCH_ARM_MACH_MSM_BUS_H
#define _ARCH_ARM_MACH_MSM_BUS_H

#include <linux/types.h>
#include <linux/input.h>

/*
 * Macros for clients to convert their data to ib and ab
 * Ws : Time window over which to transfer the data in SECONDS
 * Bs : Size of the data block in bytes
 * Per : Recurrence period
 * Tb : Throughput bandwidth to prevent stalling
 * R  : Ratio of actual bandwidth used to Tb
 * Ib : Instantaneous bandwidth
 * Ab : Arbitrated bandwidth
 *
 * IB_RECURRBLOCK and AB_RECURRBLOCK:
 * These are used if the requirement is to transfer a
 * recurring block of data over a known time window.
 *
 * IB_THROUGHPUTBW and AB_THROUGHPUTBW:
 * These are used for CPU style masters. Here the requirement
 * is to have minimum throughput bandwidth available to avoid
 * stalling.
 */
#define IB_RECURRBLOCK(Ws, Bs) ((Ws) == 0 ? 0 : ((Bs)/(Ws)))
#define AB_RECURRBLOCK(Ws, Per) ((Ws) == 0 ? 0 : ((Bs)/(Per)))
#define IB_THROUGHPUTBW(Tb) (Tb)
#define AB_THROUGHPUTBW(Tb, R) ((Tb) * (R))

struct msm_bus_node_info {
	unsigned int id;
	unsigned int priv_id;
	int gateway;
	int masterp;
	int slavep;
	int tier;
	int ahb;
	const char *slaveclk;
	const char *a_slaveclk;
	const char *memclk;
	unsigned int buswidth;
};

struct msm_bus_vectors {
	int src; /* Master */
	int dst; /* Slave */
	unsigned int ab; /* Arbitrated bandwidth */
	unsigned int ib; /* Instantaneous bandwidth */
};

struct msm_bus_paths {
	int num_paths;
	struct msm_bus_vectors *vectors;
};

struct msm_bus_scale_pdata {
	struct msm_bus_paths *usecase;
	int num_usecases;
	const char *name;
	/*
	 * If the active_only flag is set to 1, the BW request is applied
	 * only when at least one CPU is active (powered on). If the flag
	 * is set to 0, then the BW request is always applied irrespective
	 * of the CPU state.
	 */
	unsigned int active_only;
};

/* Scaling APIs */

/*
 * This function returns a handle to the client. This should be used to
 * call msm_bus_scale_client_update_request.
 * The function returns 0 if bus driver is unable to register a client
 */

#ifdef CONFIG_MSM_BUS_SCALING
uint32_t msm_bus_scale_register_client(struct msm_bus_scale_pdata *pdata);
int msm_bus_scale_client_update_request(uint32_t cl, unsigned int index);
void msm_bus_scale_unregister_client(uint32_t cl);
#else
static inline uint32_t
msm_bus_scale_register_client(struct msm_bus_scale_pdata *pdata)
{
	return 1;
}

static inline int
msm_bus_scale_client_update_request(uint32_t cl, unsigned int index)
{
	return 0;
}

static inline void
msm_bus_scale_unregister_client(uint32_t cl)
{
}
#endif

/* AXI Port configuration APIs */
int msm_bus_axi_porthalt(int master_port);
int msm_bus_axi_portunhalt(int master_port);

#endif /*_ARCH_ARM_MACH_MSM_BUS_H*/
