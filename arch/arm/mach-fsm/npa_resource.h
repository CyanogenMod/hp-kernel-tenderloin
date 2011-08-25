/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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

/*
 * Node Power Architecture (NPA) resource header file.
 */

#ifndef NPA_RESOURCE_H
#define NPA_RESOURCE_H

#include <linux/workqueue.h>
#include "npa.h"

#define ACTIVE_REQUEST		0
#define PENDING_REQUEST		1
#define ACTIVE_STATE(client)	((client)->work[ACTIVE_REQUEST].state)
#define PENDING_STATE(client)	((client)->work[PENDING_REQUEST].state)

#define NPA_LOG_MASK_NO_LOG	0
#define NPA_LOG_MASK_RESOURCE	(1<<0)
#define NPA_LOG_MASK_CLIENT	(1<<1)
#define NPA_LOG_MASK_EVENT	(1<<2)
#define NPA_LOG_MASK_LIST	(1<<3)
#define NPA_LOG_MASK_PLUGIN	(1<<4)
#define NPA_LOG_MASK_LOCKS	(1<<5)

#define npa_log(lm, res, f, ...) _npa_log(lm, res, KERN_INFO f, ##__VA_ARGS__)

struct npa_client;
struct npa_event;
struct npa_event_data;
struct npa_node_definition;
struct npa_resource_definition;
struct npa_node_dependency;
struct npa_resource;

/* Resource attributes that can override default NPA behavior.
 */
enum npa_resource_attribute {
	NPA_RESOURCE_DEFAULT,
	NPA_RESOURCE_REPORT_ALL_REQS,/* Execute driver function
					irrespective of the update fn
					return value.  */
	NPA_RESOURCE_SINGLE_CLIENT,  /* Support only a single client for a
					resource */
};

/* Node attributes
 */
enum npa_node_attribute {
	NPA_NODE_DEFAULT,
};

/* Update function calculates the new resource state.
 *
 * Note: The update function will be passed the requested new state
 * in the client's pending work structure. The update function uses this
 * pending client's request and all active requests from other clients of
 * this resource to calculate the new resource state.The value returned by
 * the update function will be used as the active value of the client.
 * If this new value is different from the resource's  active_state, then
 * the resource's driver function is called after clipping the value to the
 * active_max. The update function is called with resource locked. Any call
 * back to NPA that needs to lock resource is prone to error.
 * A value of zero in the client's pending state request to this call, means
 * the client's existing request is to be excluded in calculating the new
 * resource state.
 */
typedef unsigned int (*npa_resource_update_fn)(
		struct npa_resource *resource, struct npa_client *client);

/* Driver function applies the new state on the resource.
 *
 * Note: The driver function is called with the requested new state in
 * resource's requested_state member.
 * The resource must return the actual value set on the underlying hardware.
 * The driver function is called with the resource locked. Hence, any call back
 * into NPA that require resource locks is an error.
 */
typedef unsigned int (*npa_resource_driver_fn)(
		struct npa_resource *resource,
		struct npa_client *client, unsigned int state);

/* Plugin data structure to define the dynamic function pointers of a
 * resource. The supported clients is the bitmask of the npa_client_type
 * that will be supported by this resource. When a client of this resource is
 * created, the create_client_fn is called and when the client is destroyed,
 * the destroy_client_fn is called.
 */
struct npa_resource_plugin_ops {
	npa_resource_update_fn		update_fn;
	unsigned int			supported_clients;
	void (*create_client_fn) (struct npa_client *);
	void (*destroy_client_fn) (struct npa_client *);
};

/* PRE-DEFINED NPA PLUGINS */

/* Binary plugin update function defines the binary OR of client requests.
 */
extern const struct npa_resource_plugin_ops npa_binary_plugin;

/* Max plugin update function defines the max of all clients requests.
 */
extern const struct npa_resource_plugin_ops npa_max_plugin;

/* Min plugin update function defines the min of all client requests.
 */
extern const struct npa_resource_plugin_ops npa_min_plugin;

/* Sum plugin update function defines the sum of all client requests.
 */
extern const struct npa_resource_plugin_ops npa_sum_plugin;

/* Always-on plugin's update function turns on the resource state if either
 * the current state or the new requested state requires the resource to be
 * on.
 */
extern const struct npa_resource_plugin_ops npa_always_on_plugin;

/* Defines the dependency graph of this node. A node can be dependent on
 * another resource as long as the graph remains acyclic and there is no way
 * the request will make its way back the resource that issued the request. An
 * NPA client of the type is created for each dependency. Only, when all the
 * dependencies are available, the node will be created. The framework will
 * construct and save the handle to each NPA client.
 */
struct npa_node_dependency {
	const char 			*name;
	enum npa_client_type 		client_type;
	struct npa_client 		*handle;
};

/* The definition of the resource. The resource definition is not copied and
 * hence is expected to be available throughout the duration of the resource.
 * Resource must have an unique name in the system. The max string length is
 * defined by NPA_NAME_MAX. Resource attributes help provide directive to NPA
 * to treat the resource in a certain way. The max value help clip the output
 * of the update function before the driver function is called with the new
 * value. The pointer to the npa_resource structure is internal to NPA.
 */
struct npa_resource_definition {
	const char			*name;
	const char			*units;
	unsigned int			attributes;
	unsigned int			max;
	const struct npa_resource_plugin_ops *plugin;
	void				*data;
	struct npa_resource		*resource;
};

/* A node is a collection of resources. The node binds the driver function,
 * along with the dependencies of the resource collection. A node helps expose
 * different interfaces for the same underlying hardware. The different
 * resources of the node share the locks as well. So a request to any one
 * resource of the node will lock all clients of other resources of the node
 * from making any simultaneous requests.
 */
struct npa_node_definition {
	const char			*name;
	unsigned int			attributes;
	npa_resource_driver_fn		driver_fn;
	struct npa_node_dependency	*dependencies;
	unsigned int			dependency_count;
	struct npa_resource_definition	*resources;
	unsigned int			resource_count;
	void				*data;
};

/* This is an NPA internal data structure that binds the resources with its
 * related objects.
 */
struct npa_resource {
	struct npa_resource_definition  *definition;
	struct npa_node_definition	*node; /* Node that this resource is a
						* part of. */
	struct list_head		list; /* Active/Waiting resource list*/
	struct list_head		clients; /* Clients */
	struct list_head		events;  /* Change events */
	struct list_head		watermarks; /* Watermark events */
	unsigned int			requested_state; /* Value before the
							  * driver function */
	unsigned int			internal_state[4]; /* Resource use */
	unsigned int			active_state; /* Value after driver
						       * function. */
	unsigned int			active_max; /* Clipping value after
						     * update function */
	int				active_headroom;
	const struct npa_resource_plugin_ops *active_plugin; /* Overridable */
	struct mutex			*resource_lock; /* Node lock */
	unsigned int			level; /* Resource depth for locks */
	struct work_struct		work;
};

/* NPA work structure */
struct npa_work_request {
	unsigned int			state;
	int				start;
	int				end;
};


/* NPA client structure binds information regarding resource name, type,
 * requests information pertaining to a resource.
 */
struct npa_client {
	const char 			*name;		/* Client name */
	const char 			*resource_name; /* Resource name */
	struct npa_resource 		*resource;
	struct list_head		list; /* Part of resource's clients */
	enum npa_client_type 		type;
	struct npa_work_request 	work[2]; 	/* Active and Pending */
	void 				*resource_data; /* Place holder for
							 * resource to
							 * associate data */
};

/* NPA events are fired when the event is registered or the resource changes
 * state and/or when the resoure states hit a watermark.
 */
struct npa_event {
	enum npa_event_type		type;
	const char			*handler_name;
	struct list_head		list; /* Part of resource's events
					       * or watermarks list */
	struct npa_resource		*resource;
	int				lo_watermark;
	int				hi_watermark;
	npa_cb_fn			callback;
	void				*user_data;
	struct work_struct		work;
};

#ifdef CONFIG_MSM_NPA
/* NPA RESOURCE FUNCTIONS */

/* Defines the node.
 *
 * @initial_state: The initial state vector of the resources.
 * @callback: Function to be called when the resource is defined and active.
 * @user_data: Argument to the callback function.
 *
 * Note: A node is a collection of resource definitions and the associated
 * dependencies, driver function etc.
 * Resource definition pointer provided should be available through out
 * the system lifetime. A resource will be deemed active and the callback
 * will be called only when all the dependent resources are defined.
 * The callback function will be called with the @user_data as the first
 * argument as below
 * callback(user_data, resource->active_state, NULL, 0);
 *
 * Return:
 *   0: The resource is created or awaiting dependencies.
 *   -EINVAL: Invalid input.
 */
int npa_define_node(struct npa_node_definition *node,
		unsigned int initial_state[],
		npa_cb_fn callback, void *user_data);

/* Resource alias name.
 *
 * @resource_name: The name registered by the resource.
 * @alias_name: The alias name for the resource.
 * @callback: Function to be called when the resource is defined and active.
 *
 * Note: Aliases also are limited by the NPA_NAME_MAX string length for name.
 * The callback function will be called with the @user_data as the first
 * argument as below
 * callback(user_data, resource->active_state, NULL, 0);
 * When creating an alias of an alias, ensure that the the latter is known to
 * the NPA framework before aliasing the same. Otherwise, the former will wait
 * forever since the later will not be defined using npa_define_node.
 *
 * Return:
 *   0: The resource is created or awaiting dependencies.
 *   -EINVAL: Invalid input.
 */
int npa_alias_resource(const char *resource_name, const char *alias_name,
			npa_cb_fn callback, void *user_data);

/* Update the NPA resource state and publish events.
 *
 * @resource: The handle returned to an existing NPA resource object.
 *
 * Note: This function uses a resource pointer to a resource that is active.
 * This is a helper function that a resource may use to change the resource
 * state and send NPA events if the resource state has changed. Must be called
 * with the resource locked.
 *
 * Return:
 *   0: The resource is availble and the state is set.
 *   -EINVAL: Invalid resource.
 */
int npa_assign_resource_state(struct npa_resource *resource,
			unsigned int state);

#else
int npa_define_node(struct npa_node_definition *node,
		unsigned int initial_state[],
		npa_cb_fn callback, void *user_data)
{ return -ENOSYS; }
int npa_alias_resource(const char *resource_name, const char *alias_name,
			npa_cb_fn callback, void *user_data)
{ return -ENOSYS; }
int npa_assign_resource_state(struct npa_resource *resource,
			unsigned int state)
{ return -ENOSYS; }

#endif

#ifdef CONFIG_MSM_NPA_LOG
/* NPA logging */
extern int npa_log_mask;
extern char npa_log_resource_name[];
extern struct npa_resource *npa_log_resource;
extern int npa_log_reset;

void _npa_log(int log_mask, struct npa_resource *res, const char *fmt, ...);

/* Debug functions to print resource, resource-states etc */
/* Should be called in a resource locked context Eg. driver_function */
void __print_resource(struct npa_resource *r);
void __print_resources(void);
void __print_client_states(struct npa_resource *resource);
void __print_aliases(void);
#else
static inline void _npa_log(int log_mask, struct npa_resource *res,
		const char *fmt, ...) {}
static inline void __print_resource(struct npa_resource *r) {}
static inline void __print_resources(void) {}
static inline void __print_client_states(struct npa_resource *resource) {}
static inline void __print_aliases(void) {}
#endif

#ifdef CONFIG_MSM_NPA_DEBUG
/* Clear all internal NPA resources, their clients and events. */
void npa_reset(void);
#else
static inline void npa_reset(void) {}
#endif

#endif
