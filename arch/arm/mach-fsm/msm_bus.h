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

#ifndef _ARCH_ARM_MACH_MSM_BUS_H
#define _ARCH_ARM_MACH_MSM_BUS_H

#include <linux/types.h>
#include <linux/input.h>

/*----------Macros for clients to specify Ib, Ab---------------*/

/*Macros for clients to convert their data to ib and ab
*
* Ws: Time window over which to transfer the data in SECONDS
* Bs : Size of the data block in bytes
* Per: Recurrence period
* Tb : Throughput bandwidth to prevent stalling
* R  : Ratio of actual bandwidth used to Tb
* */
#define IB_RECURRBLOCK(Ws, Bs) ((Ws) == 0 ? 0 : ((Bs)/(Ws)))
#define AB_RECURRBLOCK(Ws, Per) ((Ws) == 0 ? 0 : ((Bs)/(Per)))
#define IB_THROUGHPUTBW(Tb) (Tb)
#define AB_THROUGHPUTBW(Tb, R) ((Tb) * (R))

struct msm_bus_node_info {
	unsigned int id;
	int gateway;
	int masterp;
	int slavep;
	int tier;
};

struct msm_bus_vectors {
	int src;
	int dst;
	int ab;
	int ib;
};

struct msm_bus_paths {
	int num_paths;
	struct msm_bus_vectors *vectors;
};

struct msm_bus_scale_pdata {
	struct msm_bus_paths *usecase;
	int num_usecases;
};

/* Topology Configuration API */
int msm_bus_register_fabric_info(int id, struct msm_bus_node_info const *info,
					unsigned len);

/* Scaling APIs */
uint32_t msm_bus_scale_register_client(struct msm_bus_scale_pdata *pdata);
int msm_bus_scale_client_update_request(uint32_t cl, unsigned index);
void msm_bus_scale_unregister_client(uint32_t cl);

/* AXI port configuration APIs */
int msm_bus_axi_porthalt(int master_port);
int msm_bus_axi_portunhalt(int master_port);

#endif /*_ARCH_ARM_MACH_MSM_BUS_H*/
