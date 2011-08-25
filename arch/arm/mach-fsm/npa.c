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

/*
 * Node Power Architecture (NPA) implementation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/workqueue.h>
#include <linux/spinlock_types.h>

#include "npa.h"
#include "npa_resource.h"

#define NPA_STR_NPA_INTERNAL "NPA-Internal"
#define NPA_STR_USER_CALLBACK "User-Callback"

/* Resource lock could be called with NULL mutex, when only the container
 * is created and the node is not defined yet.
 */
#ifdef CONFIG_MSM_NPA_LOG
#define RESOURCE_LOCK(r) do { \
	BUG_ON(!r); \
	if ((r)->definition)\
		npa_log(NPA_LOG_MASK_LOCKS, r,\
			"NPA: Resource [%s] level [%u] locked [%p] at "\
			"line [%d] by process [%p]\n",\
			(r)->definition->name, (r)->level, (r)->resource_lock,\
			__LINE__, current);\
	if ((r)->resource_lock)\
		mutex_lock_nested((r)->resource_lock, (r)->level);\
	} \
	while (0)
#define RESOURCE_UNLOCK(r) do { \
	BUG_ON(!r); \
	if ((r)->definition)\
		npa_log(NPA_LOG_MASK_LOCKS, r,\
		"NPA: Resource [%s] level [%u] unlocked [%p] at "\
		"line [%d] by process [%p]\n",\
		(r)->definition->name, (r)->level, (r)->resource_lock,\
		__LINE__, current);\
	if ((r)->resource_lock)\
		mutex_unlock((r)->resource_lock);\
	} \
	while (0)
#else
#define RESOURCE_LOCK(r)  do { \
	BUG_ON(!r); \
	if ((r)->resource_lock) \
		mutex_lock_nested((r)->resource_lock, (r)->level) ;\
	} \
	while (0)
#define RESOURCE_UNLOCK(r) do { \
	BUG_ON(!r); \
	if ((r)->resource_lock) \
		mutex_unlock((r)->resource_lock); \
	} \
	while (0)
#endif

#ifdef CONFIG_MSM_NPA_LOG
int npa_log_mask;
EXPORT_SYMBOL(npa_log_mask);
char npa_log_resource_name[NPA_NAME_MAX] = {0};
EXPORT_SYMBOL(npa_log_resource_name);
int npa_log_reset;
EXPORT_SYMBOL(npa_log_reset);

module_param_named(log_mask, npa_log_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);
MODULE_PARM_DESC(npa_log_mask,
	"Specify NPA logging for any or all of "
	"resource, client states, events, lists, plugin or locks.");
module_param_string(log_resource_name, npa_log_resource_name,
		sizeof(npa_log_resource_name), S_IRUGO | S_IWUSR | S_IWGRP);
MODULE_PARM_DESC(npa_log_resource_name,
	"Specify logging for a specific NPA resource.");
module_param_named(log_reset, npa_log_reset, int, S_IRUGO | S_IWUSR | S_IWGRP);
MODULE_PARM_DESC(npa_log_reset, "Reset NPA module parameters.");
struct npa_resource *npa_log_resource;
#endif

/* Maps resource name to alias name.*/
struct npa_alias_list {
	struct list_head 	list;
	const char 		*alias;
	struct npa_resource 	*resource;
};

static LIST_HEAD(alias_list); 	/* List of aliases */
static LIST_HEAD(active_list); 	/* Resources defined and active */
static LIST_HEAD(waiting_list);	/* Resources pending definition or waiting
				   on dependencies. */
static DEFINE_RWLOCK(list_lock);/* RW lock for locking active and waiting
				   lists */
static struct workqueue_struct *npa_wq;

static struct npa_resource *__get_resource(const char *resource_name);
static void resource_creation_handler(void *data, unsigned int state,
		void *called_data, unsigned int value);
static void send_single_event(struct work_struct *work);
static void send_update_events(struct work_struct *work);

#ifdef CONFIG_MSM_NPA_LOG
void _npa_log(int log_mask, struct npa_resource *res, const char *fmt, ...)
{
	if (npa_log_reset) {
		npa_log_mask = 0;
		npa_log_resource = NULL;
		npa_log_resource_name[0] = 0;
		npa_log_reset = 0;
	}

	if (npa_log_resource_name[0] && !npa_log_resource) {
		npa_log_resource = __get_resource(npa_log_resource_name);
		if (npa_log_resource)
			pr_info("NPA: Logging for resource [%s]\n",
					npa_log_resource_name);
	}

	if (!npa_log_resource ||
			(npa_log_resource && (res == npa_log_resource)))
		if (log_mask & npa_log_mask) {
			va_list ap;
			va_start(ap, fmt);
			vprintk(fmt, ap);
			va_end(ap);
		}
}

void __print_resource(struct npa_resource *r)
{
	struct npa_client *client = NULL;
	struct npa_event *e = NULL;
	struct npa_alias_list *a = NULL;
	const char *rname = r->definition->name;

	if (!(npa_log_mask & NPA_LOG_MASK_RESOURCE))
		return;

	list_for_each_entry(a, &alias_list, list) {
		if (!strncmp(a->resource->definition->name,
			r->definition->name, NPA_NAME_MAX)) {
			pr_info("NPA: Resource [%s] Alias: [%s]\n",
					rname, a->alias);
		}
	}

	pr_info("NPA: Resource [%s] Units: [%s]\n",
			rname, r->definition->units);
	pr_info("NPA: Resource [%s] Node name: [%s]\n",
			rname, r->node->name);
	pr_info("NPA: Resource [%s] Level: [%u]\n",
			rname, r->level);

	list_for_each_entry(client, &r->clients, list) {
		pr_info("NPA: Resource [%s] client [%s: %u]\n",
				rname, client->name, ACTIVE_STATE(client));
	}

	list_for_each_entry(e, &r->events, list) {
		pr_info("NPA: Reource [%s] event [%s, %d]\n",
				rname, e->handler_name, e->type);
	}

	pr_info("NPA: Resource [%s] Active state: [%u]\n",
			rname, r->active_state);
	pr_info("NPA: Resource [%s] Active max: [%u]\n",
			rname, r->active_max);
	pr_info("NPA: Resource [%s] Active headroom: [%u]\n",
			rname, r->active_headroom);
}
EXPORT_SYMBOL(__print_resource);

void __print_resources(void)
{
	struct npa_resource *resource = NULL;

	if (!(npa_log_mask & NPA_LOG_MASK_RESOURCE))
		return;

	list_for_each_entry(resource, &active_list, list)
		__print_resource(resource);

	list_for_each_entry(resource, &waiting_list, list)
		__print_resource(resource);
}
EXPORT_SYMBOL(__print_resources);

void __print_client_states(struct npa_resource *resource)
{
	struct npa_client *client = NULL;

	if (!(npa_log_mask & NPA_LOG_MASK_LIST))
		return;

	list_for_each_entry(client, &resource->clients, list) {
		pr_info("NPA: Resource [%s] client [%s: %u]\n",
				resource->definition->name,
				client->name, ACTIVE_STATE(client));
	}
}
EXPORT_SYMBOL(__print_client_states);

void __print_aliases(void)
{
	struct npa_alias_list *a = NULL;

	if (npa_log_mask & NPA_LOG_MASK_LIST)
		return;

	list_for_each_entry(a, &alias_list, list) {
		pr_info("NPA: Alias[%s --> %s]\n",
				a->alias, a->resource->definition->name);
	}
}
EXPORT_SYMBOL(__print_aliases);
#endif

/* Returns the resource, if the name matches any element in the list.
 * NULL if not found.
 * @resource_name can be original resource name or an alias.
 */
static struct npa_resource *__get_resource(const char *resource_name)
{
	struct npa_alias_list *alias = NULL;
	struct npa_alias_list *temp = NULL;

	list_for_each_entry_safe(alias, temp, &alias_list, list) {
		if (!strncmp(resource_name, alias->alias, NPA_NAME_MAX))
			return alias->resource;
	}

	return NULL;
}

static int __is_active_resource(struct npa_resource *resource)
{
	struct npa_resource *res_itr = NULL;

	if (resource && !list_empty(&active_list)) {
		list_for_each_entry(res_itr, &active_list, list) {
			if (resource == res_itr)
				return 1;
		}
	}

	return 0;
}

static struct npa_resource *active_resource(const char *resource_name)
{
	struct npa_resource *resource = NULL;

	read_lock(&list_lock);
	resource = __get_resource(resource_name);
	if (resource)
		resource = __is_active_resource(resource) ? resource : NULL;
	read_unlock(&list_lock);

	return resource;
}

/* Creates a resource object with the definition provided. The created
 * resource is automatically added to the waiting list. create_resource can be
 * called with an node pointer or NULL, indicating the node has to be created.
 * If the node already exists, then the resource definition and the resource
 * objects will be replaced with the ones provided.
 * Node = NULL, resource_defn = valid, name = valid:
 * 	Create a new resource for the first time and create a node for it.
 * Node = valid, resource_defn = valid, name = valid:
 * 	Link a new node to an existing resource.
 */
static struct npa_resource *create_resource(struct npa_node_definition *node,
		struct npa_resource_definition *resource_defn,
		const char *name)
{
	struct npa_alias_list *new_alias = NULL;
	struct npa_resource *new_resource = NULL;
	struct npa_resource *resource = NULL;
	struct npa_node_definition *old_node = NULL;
	struct npa_node_definition *new_node = NULL;
	struct npa_resource_definition *old_defn = NULL;
	struct npa_resource_definition *new_defn = NULL;

	if (!node) {
		new_node = kzalloc(sizeof(struct npa_node_definition),
				GFP_KERNEL);
		new_defn = kzalloc(sizeof(struct npa_resource_definition),
				GFP_KERNEL);
		new_defn->name = name;
		new_node->resources = new_defn;
		new_node->resource_count = 1;
		new_node->dependencies = NULL;
		new_node->dependency_count = 0;
	} else {
		new_node = node;
		new_defn = resource_defn;
	}

	write_lock(&list_lock);
	resource = __get_resource(name);
	if (resource && !node)
		goto done;
	if (!resource) {
		new_resource = kzalloc(sizeof(struct npa_resource), GFP_ATOMIC);
		new_alias = kzalloc(sizeof(struct npa_alias_list), GFP_ATOMIC);
		new_alias->alias = name;
		new_alias->resource = new_resource;
		INIT_LIST_HEAD(&new_alias->list);

		new_resource->node = new_node;
		new_resource->definition = new_defn;
		INIT_LIST_HEAD(&new_resource->list);
		INIT_LIST_HEAD(&new_resource->clients);
		INIT_LIST_HEAD(&new_resource->events);
		INIT_LIST_HEAD(&new_resource->watermarks);
		INIT_WORK(&new_resource->work, send_update_events);

		list_add(&new_alias->list, &alias_list);
		list_add(&new_resource->list, &waiting_list);
		resource = new_resource;
	} else {
		/* Resource exists */
		old_node = resource->node;
		old_defn = resource->definition;
		/* Replace the alias name pointer to use new defn. */
		if (old_defn) {
			struct npa_alias_list *a = NULL;
			list_for_each_entry(a, &alias_list, list) {
				if (a->alias == old_defn->name)
					a->alias = new_defn->name;
			}
		}
		/* Replace the node and resource defn. */
		resource->node = new_node;
		resource->definition = new_defn;
	}
done:
	write_unlock(&list_lock);

	npa_log(NPA_LOG_MASK_RESOURCE, resource,
		"NPA: Resource [%s] definition created\n",
		resource->definition->name);

	if (resource != new_resource) {
		kfree(old_defn);
		kfree(old_node);
		if (!node) {
			kfree(new_node);
			kfree(new_defn);
		}
		kfree(new_resource);
		kfree(new_alias);
	}

	return resource;
}

/* Register callback function for a resource creation event
 * (NPA_EVENT_RESERVED1). The event object malloc'd in this function is
 * released is to be released in the handler. NPA_EVENT_RESERVED1 events
 * are use and throw events and should be removed from the list once notified.
 */
static int register_resource_event(struct npa_resource *resource,
		npa_cb_fn callback, void *data)
{
	struct npa_event *event = NULL;

	if (!callback)
		return 0; /* Nothing to wait for */

	event = kzalloc(sizeof(struct npa_event), GFP_KERNEL);

	INIT_LIST_HEAD(&event->list);
	INIT_WORK(&event->work, send_single_event);
	event->type = NPA_EVENT_RESERVED1;
	event->resource = resource;
	/* Helpful identification string to distinguish user callbacks
	 * from NPA internal resource completion events.
	 */
	if (callback == resource_creation_handler)
		event->handler_name = NPA_STR_NPA_INTERNAL;
	else
		event->handler_name = NPA_STR_USER_CALLBACK;
	event->callback = callback;
	event->user_data = data;

	RESOURCE_LOCK(resource);
	list_add(&event->list, &resource->events);
	RESOURCE_UNLOCK(resource);

	npa_log(NPA_LOG_MASK_RESOURCE, resource,
		"NPA: Registered for NPA_EVENT_RESERVED1 from resource [%s]\n",
		resource->definition->name);

	return 0;
}

/* Sends notification to this particular event object. This is mostly used
 * when the event is added to the resource.
 */
static void send_single_event(struct work_struct *work)
{
	struct npa_event *event = container_of(work, struct npa_event, work);
	struct npa_resource *resource = event->resource;
	struct npa_event_data event_data;

	/* Read the state when it is not being modified. */
	RESOURCE_LOCK(resource);
	event_data.resource_name = resource->definition->name;
	event_data.state = resource->active_state;
	event_data.headroom = resource->active_headroom;
	RESOURCE_UNLOCK(resource);

	npa_log(NPA_LOG_MASK_EVENT, resource,
		"NPA: Sending single event [%s] for resource [%s]\n",
		event->handler_name, resource->definition->name);

	event->callback(event->user_data, event->type, &event_data, 0);
}

/* Executes callbacks on all event objects. Callbacks are called in the
 * sequence of the event objects. This function does not entertain any
 * priority. If a callback blocks, it will affect all others that are
 * pending in the event list, as well as blocking the NPA work queue.
 */
static void send_update_events(struct work_struct *work)
{
	int expired = 0;
	struct npa_event *event = NULL;
	struct npa_event *temp = NULL;
	struct npa_resource *resource =
		container_of(work, struct npa_resource, work);
	struct npa_event_data event_data;

	npa_log(NPA_LOG_MASK_EVENT, resource,
		"NPA: Resource [%s] sending change events\n",
		resource->definition->name);

	/* Read the state when it is not being modified. */
	RESOURCE_LOCK(resource);
	event_data.resource_name = resource->definition->name;
	event_data.state = resource->active_state;
	event_data.headroom = resource->active_headroom;
	RESOURCE_UNLOCK(resource);

	list_for_each_entry_safe(event, temp, &resource->events, list) {
		expired = 0;
		switch (event->type) {
		case NPA_EVENT_RESERVED1:
			 /* Destroy after execution. */
			list_del(&event->list);
			expired = 1;
			break;
		case NPA_EVENT_CHANGE:
			break;
		default:
			BUG();
			break;
		}

		npa_log(NPA_LOG_MASK_EVENT, resource,
			"NPA: Resource [%s] sending event [%d] to [%s] "
			"with state [%u] headroom [%d] \n",
			resource->definition->name, event->type,
			event->handler_name, event_data.state,
			event_data.headroom);

		event->callback(event->user_data, event->type, &event_data, 0);
		if (expired)
			kfree(event);
	}

	npa_log(NPA_LOG_MASK_EVENT, resource,
		"NPA: Resource [%s] sending watermark events\n",
		resource->definition->name);

	/* Send the watermark events */
	list_for_each_entry_safe(event, temp, &resource->watermarks, list) {
		if (event_data.state <= event->lo_watermark) {

			npa_log(NPA_LOG_MASK_EVENT, resource,
				"NPA: Resource [%s] sending "
				"low watermark event to [%s] "
				"with state [%u] headroom [%d]\n",
				resource->definition->name,
				event->handler_name,
				event_data.state,
				event_data.headroom);

			event->callback(event->user_data,
					NPA_EVENT_LO_WATERMARK,
					&event_data, 0);
		}
		if (event_data.state >= event->hi_watermark) {

			npa_log(NPA_LOG_MASK_EVENT, resource,
				"NPA: Resource [%s] sending "
				"high watermark event to [%s] "
				"with state [%u] headroom [%d]\n",
				resource->definition->name,
				event->handler_name,
				event_data.state,
				event_data.headroom);

			event->callback(event->user_data,
					NPA_EVENT_HI_WATERMARK,
					&event_data, 0);
		}
	}
}

/* Schedules a work to notify all those registered for events of the change
 * in resource state.
 */
static void publish_resource_state(struct npa_resource *resource)
{
	npa_log(NPA_LOG_MASK_EVENT, resource,
		"NPA: Queueing work for resource [%s]\n",
		resource->definition->name);

	queue_work(npa_wq, &resource->work);
}

/* Activate this resource and all other resources that are part of this node.
 * Resources when created by default are put in waiting lists.
 * This function is to be called, when all the dependencies of this
 * resources are available and the resource is ready to be used.
 */
static int activate_node(struct npa_resource *resource)
{
	struct npa_resource_definition *def = NULL;
	struct npa_node_dependency *node_dep = NULL;
	struct npa_client dummy = {
		.type = NPA_CLIENT_REQUIRED,
		.name = NPA_STR_NPA_INTERNAL,
		.resource_name = resource->definition->name,
	};
	unsigned int count = 0;
	unsigned int node_level = 0;

	npa_log(NPA_LOG_MASK_RESOURCE, resource,
		"NPA: Resource [%s] being activated\n",
		resource->definition->name);


	/* Create clients for all the dependencies, so that the driver
	 * function can send the requests to these resources as well.
	 */
	RESOURCE_LOCK(resource);
	node_dep = resource->node->dependencies;
	count = resource->node->dependency_count;
	while (count) {
		struct npa_client *client =
			npa_create_sync_client(node_dep->name,
					resource->node->name,
					node_dep->client_type);
		BUG_ON((client == NULL) || IS_ERR(client));
		node_dep->handle = client;
		if (client->resource->level > node_level)
			node_level = client->resource->level;
		count--;
		node_dep++;
	}

	def = resource->node->resources;
	count = resource->node->resource_count;
	while (count) {
		write_lock(&list_lock);
		list_del(&def->resource->list); /* Remove from waiting list */
		list_add(&def->resource->list, &active_list);
		write_unlock(&list_lock);
		def->resource->active_state =
			def->resource->node->driver_fn(def->resource, &dummy,
					def->resource->requested_state);
		def->resource->active_headroom = def->resource->active_max -
					def->resource->active_state;
		def->resource->level = node_level + 1;
		count--;
		def++;
	}
	RESOURCE_UNLOCK(resource);

	/* Schedule a work to signal all dependencies that may depend
	 * on this resource.
	 */
	def = resource->node->resources;
	count = resource->node->resource_count;
	while (count) {
		publish_resource_state(def->resource);
		def++;
		count--;
	}
	return 0;
}

static void request_state(struct npa_resource *resource,
		struct npa_client *client, unsigned int state)
{
	unsigned int new_state;

	npa_log(NPA_LOG_MASK_CLIENT, resource,
		"NPA: Resource [%s] client [%s] requested state [%u]\n",
		resource->definition->name, client->name, state);

	RESOURCE_LOCK(resource);
	PENDING_STATE(client) = state;
	new_state = resource->active_plugin->update_fn(resource, client);
	ACTIVE_STATE(client) = PENDING_STATE(client);
	resource->requested_state = new_state;

	if (new_state > resource->active_max)
		new_state = resource->active_max;

	if ((resource->definition->attributes &
				NPA_RESOURCE_REPORT_ALL_REQS) ||
				new_state != resource->active_state) {
		resource->active_state =
			resource->node->driver_fn(resource, client,
					new_state);
		resource->active_headroom = resource->active_max -
						resource->active_state;
	}
	RESOURCE_UNLOCK(resource);

	npa_log(NPA_LOG_MASK_CLIENT, resource,
		"NPA: Resource [%s] state set to [%u]\n",
		resource->definition->name, resource->active_state);

	publish_resource_state(resource);
}

/* Return if any dependency of this resource is still not active.
 * Returns NULL if there are no pending dependencies.
 */
static struct npa_node_dependency *get_first_pending_dependency(
		struct npa_resource *resource)
{
	struct npa_node_dependency *dep = NULL;
	unsigned int count = 0;

	dep = resource->node->dependencies;
	count = resource->node->dependency_count;
	while (count) {
		if (!active_resource(dep->name))
			return dep;
		count--;
		dep++;
	}

	return NULL;
}

/* Callback function for NPA_EVENT_RESERVED1. This function registers
 * itself with the event queue for the first pending dependency of the
 * resource. If there are no dependency, then this activates the resource
 * as well.
 */
static void resource_creation_handler(void *data, unsigned int type,
		void *event_data, unsigned int value)
{
	struct npa_resource *resource = (struct npa_resource *)data;
	struct npa_node_dependency *dep = NULL;

	npa_log(NPA_LOG_MASK_RESOURCE, resource,
		"NPA: Resource [%s] creation handler called\n",
		resource->definition->name);

	dep = get_first_pending_dependency(resource);
	if (dep) {
		npa_log(NPA_LOG_MASK_RESOURCE, resource,
			"NPA: Resource [%s] has dependency on resource [%s]\n",
			resource->definition->name, dep->name);

		npa_resource_available(dep->name,
				resource_creation_handler, resource);
		return;
	}

	/* All dependencies are complete. Finish the resource completion. */
	activate_node(resource);
}

static void resource_destruction_handler(void *data, unsigned int value,
		void *event_data, unsigned int type)
{
	struct npa_resource *resource = data;

	BUG_ON(!list_empty(&resource->clients));

	if (!list_empty(&resource->events)) {
		struct npa_event *event = NULL;
		struct npa_event *temp = NULL;
		list_for_each_entry_safe(event, temp, &resource->events, list) {
			list_del(&event->list);

			npa_log(NPA_LOG_MASK_EVENT, resource,
				"NPA: Resource [%s] sending "
				"event [%d] to [%s]\n",
				resource->definition->name,
				event->type, event->handler_name);

			event->callback(event->user_data, value, NULL, 0);
			kfree(event);
		}
	}

	BUG_ON(!list_empty(&resource->watermarks));

	kfree(resource->definition);
	kfree(resource->node);
	kfree(resource);
}

/* Defines the node and individual resources.
 * All resources need to be defined by an external entity before the resource
 * can be used. This function checks for any pending dependency. This function
 * adds the callback to its event queue and when the resource is completely
 * defined, the callback is invoked.
 */
int npa_define_node(struct npa_node_definition *node,
			unsigned int initial_state[],
			npa_cb_fn callback, void *user_data)
{
	struct npa_resource *resource = NULL;
	struct npa_resource_definition *res_def = NULL;
	struct npa_node_dependency *node_dep = NULL;
	struct mutex *common_lock = NULL;
	unsigned int count = 0;
	int ret = 0;
	int idx = 0;

	if (!node || !node->name[0] || !node->resources || !node->driver_fn)
		return -EINVAL;

	npa_log(NPA_LOG_MASK_RESOURCE, NULL,
		"NPA: Defining node: [%s]\n", node->name);

	common_lock = kzalloc(sizeof(struct mutex), GFP_KERNEL);
	mutex_init(common_lock);

	/* For each resource definition in this node definition, see if a
	 * npa_resource object already exists, if not create and add to the
	 * waiting list. We then check for dependencies and wait on those
	 * using the first resource object that we have. Once the dependencies
	 * are satisfied for this resource, we go ahead and activate all
	 * the resources of this node.
	 */
	res_def = node->resources;
	count = node->resource_count;
	while (count) {
		BUG_ON(!res_def->plugin);
		resource = create_resource(node, res_def, res_def->name);
		BUG_ON(__is_active_resource(resource));
		resource->definition = res_def;
		resource->node = node;
		resource->requested_state = initial_state[idx++];
		resource->active_max = res_def->max;
		resource->active_plugin = res_def->plugin;
		resource->resource_lock = common_lock;
		res_def->resource = resource;

		npa_log(NPA_LOG_MASK_RESOURCE, resource,
			"NPA: Defining resource: [%s]\n", res_def->name);

		count--;
		res_def++;
	}

	/* Get the first resource in the node definition */
	resource = node->resources[0].resource;

	/* Save the callback function to be called if the resource is ready.*/
	ret = register_resource_event(resource, callback, user_data);
	if (ret)
		return ret;

	node_dep = get_first_pending_dependency(resource);
	if (node_dep) {
		npa_log(NPA_LOG_MASK_RESOURCE, resource,
			"NPA: Resource [%s] has dependency on resource [%s]\n",
			resource->definition->name, node_dep->name);

		return npa_resource_available(node_dep->name,
				resource_creation_handler,
				(void *)resource);
	}

	/* Only executed when there are no pending dependencies of this node.*/
	activate_node(resource);

	return ret;
}
EXPORT_SYMBOL(npa_define_node);

/* Checks if the resource is available in the NPA active or waiting lists.
 * If not, then creates a dummy resource and adds it to the waiting list.
 * When the resource is available, the callback function is executed when
 * the events are fired.
 */
int npa_resource_available(const char *resource_name,
			npa_cb_fn callback, void *user_data)
{
	struct npa_resource *resource = NULL;

	if (!resource_name || !resource_name[0] || !callback)
		return -EINVAL;

	npa_log(NPA_LOG_MASK_RESOURCE, resource,
		"NPA: Checking if resource [%s] is available\n", resource_name);

	resource = create_resource(NULL, NULL, resource_name);
	read_lock(&list_lock);
	if (!__is_active_resource(resource)) {
		read_unlock(&list_lock);
		/* Register for NPA_EVENT_RESERVED1 event callback and exit */
		return register_resource_event(resource, callback, user_data);
	}
	read_unlock(&list_lock);

	npa_log(NPA_LOG_MASK_RESOURCE, resource,
		"NPA: Resource [%s] is available\n", resource_name);

	/* Resource is active. Call in the current context. */
	callback(user_data, resource->active_state, NULL, 0);

	return 0;
}
EXPORT_SYMBOL(npa_resource_available);

/* Define an alias name for a resource. */
int npa_alias_resource(const char *resource_name, const char *alias_name,
			npa_cb_fn callback, void *user_data)
{
	struct npa_resource *parent = NULL;
	struct npa_resource *resource = NULL;
	struct npa_alias_list *ralias = NULL;

	if (!resource_name || !resource_name[0] ||
			!alias_name || !alias_name[0] || !callback)
		return -EINVAL;

	npa_log(NPA_LOG_MASK_RESOURCE, NULL,
		"NPA: Creating alias [%s] for resource [%s]\n",
		alias_name, resource_name);

	ralias = kzalloc(sizeof(struct npa_alias_list), GFP_KERNEL);

	parent = create_resource(NULL, NULL, resource_name);

	write_lock(&list_lock);
	resource = __get_resource(alias_name);
	if (resource) {
		struct npa_alias_list *alias = NULL;
		list_for_each_entry(alias, &alias_list, list) {
			if (alias->resource == resource)
				alias->resource = parent;
		}
		list_del(&resource->list);
	} else {
		INIT_LIST_HEAD(&ralias->list);
		ralias->alias = alias_name;
		ralias->resource = parent;
		list_add(&ralias->list, &alias_list);
	}
	write_unlock(&list_lock);

	read_lock(&list_lock);
	if (__is_active_resource(parent)) {
		read_unlock(&list_lock);
		if (resource)
			resource_destruction_handler(resource,
				parent->active_state, NULL, 0);
		callback(user_data, 0, NULL, 0);
	} else {
		read_unlock(&list_lock);
		register_resource_event(parent, callback, user_data);
		if (resource)
			register_resource_event(parent,
				resource_destruction_handler, resource);
	}

#ifdef CONFIG_MSM_NPA_LOG
	/* If we were looking to log an alias, then re-acquire the resource* */
	npa_log_resource = __get_resource(npa_log_resource_name);
#endif

	npa_log(NPA_LOG_MASK_RESOURCE, parent,
		"NPA: Alias [%s] created for resource [%s]\n",
		alias_name, parent->definition->name);

	return 0;
}
EXPORT_SYMBOL(npa_alias_resource);

/* Update the resource state with this state. The driver function is expected
 * to update the underlying h/w state or whatever this resource represents.
 * This function sends update events to the listeners.
 */
int npa_assign_resource_state(struct npa_resource *resource,
				unsigned int state)
{
	npa_log(NPA_LOG_MASK_RESOURCE, resource,
		"NPA: Resource [%s] is assigned state [%u]\n",
		resource->definition->name, state);

	resource->active_state = state;
	resource->active_headroom = resource->active_max - state;
	publish_resource_state(resource);

	return 0;
}
EXPORT_SYMBOL(npa_assign_resource_state);

/* Create a sync client for this resource.
 * Returns an handle to the NPA client object if the resource is active,
 * error condition otherwise.
 */
struct npa_client *npa_create_sync_client(const char *resource_name,
			const char *client_name, enum npa_client_type type)
{
	struct npa_client *client = NULL;
	struct npa_resource *resource = NULL;
	int flag = 0;

	if (!resource_name || !resource_name[0] ||
			!client_name || !client_name[0])
		return ERR_PTR(-EINVAL);

	resource = active_resource(resource_name);
	if (!resource) {
		/* Clients for pending resources are not allowed. */
		npa_log(NPA_LOG_MASK_CLIENT, resource,
			"NPA: Resource [%s] requested by "
			"client [%s] not active\n",
			resource_name, client_name);
		return ERR_PTR(-ENODEV);
	}

	RESOURCE_LOCK(resource);
	if (!(type & resource->active_plugin->supported_clients))
		flag = -1;
	if ((resource->definition->attributes &
				NPA_RESOURCE_SINGLE_CLIENT) &&
			!list_empty(&resource->clients))
		flag = -2;
	RESOURCE_UNLOCK(resource);

	if (flag == -1) {
		npa_log(NPA_LOG_MASK_CLIENT, resource,
			"NPA: Client [%s, %d] is not "
			"supported by resource [%s]\n",
			client_name, type, resource_name);
		return ERR_PTR(-EINVAL);
	}

	if (flag == -2) {
		npa_log(NPA_LOG_MASK_CLIENT, resource,
			"NPA: Resource [%s] supports only a single"
			"client. Client [%s] create failed\n",
			resource_name, client_name);
		return ERR_PTR(-EBUSY);
	}

	npa_log(NPA_LOG_MASK_CLIENT, resource,
		"NPA: Creating sync client [%s] for "
		"resource [%s]\n", client_name, resource_name);

	client = kzalloc(sizeof(struct npa_client), GFP_KERNEL);
	INIT_LIST_HEAD(&client->list);
	client->name = client_name;
	client->resource_name = resource_name;
	client->resource = resource;
	client->type = type;
	RESOURCE_LOCK(resource);
	list_add(&client->list, &resource->clients);
	if (resource->active_plugin->create_client_fn)
		resource->active_plugin->create_client_fn(client);
	RESOURCE_UNLOCK(resource);

	npa_log(NPA_LOG_MASK_CLIENT, resource,
		"NPA: Client [%s, %d] for resource [%s] created\n",
		client_name, type, resource_name);

	return client;
}
EXPORT_SYMBOL(npa_create_sync_client);

/* Removes a client from a resource. */
void npa_destroy_client(struct npa_client *client)
{
	struct npa_resource *resource = NULL;

	if (client) {
		npa_log(NPA_LOG_MASK_CLIENT, client->resource,
			"NPA: Resource [%s] client [%s, %d] destroyed\n",
			client->resource->definition->name,
			client->name, client->type);

		resource = client->resource;
		RESOURCE_LOCK(resource);
		if (resource->active_plugin->destroy_client_fn)
			resource->active_plugin->destroy_client_fn(client);
		list_del(&client->list);
		RESOURCE_UNLOCK(resource);
		kfree(client);
	}
}
EXPORT_SYMBOL(npa_destroy_client);

/* Issues a required workflow request call to the client. If the update function
 * returns a state different from the current resource state, then the resource
 * driver function is called in the same context to update the resource and the
 * update events will be scheduled to fire.
 */
int npa_issue_required_request(struct npa_client *client, unsigned int state)
{
	if (client->type != NPA_CLIENT_REQUIRED)
		return -EINVAL;

	npa_log(NPA_LOG_MASK_CLIENT, client->resource,
		"NPA: Resource [%s] received request from client [%s, %u]\n",
		client->resource->definition->name,
		client->name, state);

	request_state(client->resource, client, state);

	return 0;
}
EXPORT_SYMBOL(npa_issue_required_request);

int npa_modify_required_request(struct npa_client *client, int delta)
{
	if (client->type != NPA_CLIENT_REQUIRED)
		return -EINVAL;

	/* Check if the active state of the client is non-zero. */
	if (!ACTIVE_STATE(client))
		return -EINVAL;

	npa_log(NPA_LOG_MASK_CLIENT, client->resource,
		"NPA: Resource [%s] modify request by client [%s, %d]\n",
		client->resource->definition->name, client->name, delta);

	request_state(client->resource, client, ACTIVE_STATE(client) + delta);

	return 0;
}
EXPORT_SYMBOL(npa_modify_required_request);

int npa_issue_impulse_request(struct npa_client *client)
{
	if (client->type != NPA_CLIENT_IMPULSE)
		return -EINVAL;

	npa_log(NPA_LOG_MASK_CLIENT, client->resource,
		"NPA: Resource [%s] impulse request by client [%s]\n",
		client->resource->definition->name, client->name);

	request_state(client->resource, client,
			client->resource->definition->max);

	return 0;
}
EXPORT_SYMBOL(npa_issue_impulse_request);

int npa_issue_isoc_request(struct npa_client *client, unsigned int duration,
		unsigned int level)
{
	if (client->type != NPA_CLIENT_ISOCHRONOUS)
		return -EINVAL;

	npa_log(NPA_LOG_MASK_CLIENT, client->resource,
		"NPA: Resource [%s] isoc request by "
		"client [%s] with duration [%u], level [%u]\n",
		client->resource->definition->name,
		client->name, duration, level);

	/* TODO: Update work struct with time values */
	client->work[PENDING_REQUEST].state = level;
	request_state(client->resource, client, PENDING_STATE(client));

	return 0;
}
EXPORT_SYMBOL(npa_issue_isoc_request);

int npa_issue_limit_max_request(struct npa_client *client, unsigned int max)
{
	int val = max;
	struct npa_resource *resource = client->resource;
	struct npa_client *cl_itr = NULL;

	if (client->type != NPA_CLIENT_LIMIT_MAX)
		return -EINVAL;

	npa_log(NPA_LOG_MASK_CLIENT, client->resource,
		"NPA: Resource [%s] limit max [%u] request by client [%s] \n",
		client->resource->definition->name, max, client->name);

	RESOURCE_LOCK(resource);
	list_for_each_entry(cl_itr, &resource->clients, list) {
		if ((cl_itr->type & NPA_CLIENT_LIMIT_MAX))
			if (ACTIVE_STATE(cl_itr) &&
					(ACTIVE_STATE(cl_itr) < val))
				val = ACTIVE_STATE(cl_itr);
	}
	resource->active_max = val;
	RESOURCE_UNLOCK(resource);

	/* Issue the request to the update_fn and limit the current state if
	 * needed and call the driver fn. Send events if resource state is
	 * changed.
	 */
	request_state(client->resource, client, max);

	return 0;
}
EXPORT_SYMBOL(npa_issue_limit_max_request);

void npa_complete_request(struct npa_client *client)
{
	npa_log(NPA_LOG_MASK_CLIENT, client->resource,
		"NPA: Client [%s] completed request\n", client->name);

	/* Complete this request. */
	request_state(client->resource, client, 0);
}
EXPORT_SYMBOL(npa_complete_request);

void npa_cancel_request(struct npa_client *client)
{
	npa_log(NPA_LOG_MASK_CLIENT, client->resource,
		"NPA: Client [%s] cancelled request\n", client->name);

	/* TODO: In future flush any history for this client that may have
	 * been captured. And complete this request.
	 */
	request_state(client->resource, client, 0);
}
EXPORT_SYMBOL(npa_cancel_request);

unsigned int npa_get_state(struct npa_client *client)
{
	return client->resource->active_state;
}
EXPORT_SYMBOL(npa_get_state);

/* Create an event object and attach to the resource event list.
 * This function expects that the resource be active for the event to
 * be registered with the resource.
 */
struct npa_event *npa_create_change_event(const char *resource_name,
		const char *handler_name, npa_cb_fn event_cb, void *user_data)
{
	struct npa_event *event = NULL;
	struct npa_resource *resource = NULL;

	if (!resource_name || !resource_name[0] ||
			!handler_name || !event_cb)
		return ERR_PTR(-EINVAL);

	resource = active_resource(resource_name);
	if (!resource)
		return ERR_PTR(-ENODEV);

	event = kzalloc(sizeof(struct npa_event), GFP_KERNEL);
	INIT_LIST_HEAD(&event->list);
	INIT_WORK(&event->work, send_single_event);
	event->type = NPA_EVENT_CHANGE;
	event->handler_name = handler_name;
	event->resource = resource;
	event->callback = event_cb;
	event->user_data = user_data;

	RESOURCE_LOCK(resource);
	list_add(&event->list, &resource->events);
	RESOURCE_UNLOCK(resource);

	npa_log(NPA_LOG_MASK_EVENT, resource,
		"NPA: Resource [%s] adding change event for [%s]\n",
		resource->definition->name, handler_name);

	/* Callback with the current resource state only for this event */
	queue_work(npa_wq, &event->work);

	return event;
}
EXPORT_SYMBOL(npa_create_change_event);

struct npa_event *npa_create_watermark_event(const char *resource_name,
		const char *handler_name, npa_cb_fn event_cb, void *user_data)
{
	struct npa_event *event = NULL;
	struct npa_resource *resource = NULL;

	if (!resource_name || !resource_name[0] ||
			!handler_name || !event_cb)
		return ERR_PTR(-EINVAL);

	resource = active_resource(resource_name);
	if (!resource)
		return ERR_PTR(-ENODEV);

	event = kzalloc(sizeof(struct npa_event), GFP_KERNEL);
	INIT_LIST_HEAD(&event->list);
	INIT_WORK(&event->work, send_single_event);
	event->type = NPA_EVENT_LO_WATERMARK;
	event->handler_name = handler_name;
	event->resource = resource;
	event->callback = event_cb;
	event->user_data = user_data;

	RESOURCE_LOCK(resource);
	list_add(&event->list, &resource->watermarks);
	RESOURCE_UNLOCK(resource);

	npa_log(NPA_LOG_MASK_EVENT, resource,
		"NPA: Resource [%s] adding watermark events for [%s]\n",
		resource->definition->name, handler_name);

	return event;
}
EXPORT_SYMBOL(npa_create_watermark_event);

/* Removes an event from the event queue of this resource. */
void npa_destroy_event(struct npa_event *event)
{
	/* TODO BUG: Possible race condition with send-events(), event
	 * list being modified outside.
	 */
	RESOURCE_LOCK(event->resource);
	list_del(&event->list);
	RESOURCE_UNLOCK(event->resource);

	npa_log(NPA_LOG_MASK_EVENT, event->resource,
		"NPA: Resource [%s] event [%s] destroyed\n",
		event->resource->definition->name, event->handler_name);

	kfree(event);

}
EXPORT_SYMBOL(npa_destroy_event);

/* Set watermarks for this event. The events are fired only if the resource
 * state lie outside the watermarks.
 */
int npa_set_event_watermarks(struct npa_event *event,
				int lo_watermark, int hi_watermark)
{
	unsigned int state;
	int publish_event = 0;

	if (!event)
		return -EINVAL;

	RESOURCE_LOCK(event->resource);
	state = event->resource->active_state;
	event->lo_watermark = lo_watermark;
	event->hi_watermark = hi_watermark;
	if (state <= event->lo_watermark) {
		event->type = NPA_EVENT_LO_WATERMARK;
		publish_event = 1;
	}
	if (state >= event->hi_watermark) {
		event->type = NPA_EVENT_HI_WATERMARK;
		publish_event = 1;
	}
	RESOURCE_UNLOCK(event->resource);

	npa_log(NPA_LOG_MASK_EVENT, event->resource,
		"NPA: Resource [%s] event [%s] watermarks [%d, %d] set\n",
		event->resource->definition->name, event->handler_name,
		lo_watermark, hi_watermark);

	if (publish_event)
		queue_work(npa_wq, &event->work);

	return 0;
}
EXPORT_SYMBOL(npa_set_event_watermarks);

/* Pre-defined NPA update functions.
 * Currently we only consider "required" client requests.
 */
static unsigned int binary_update(struct npa_resource *resource,
					struct npa_client *client)
{
	struct npa_client *cl_itr = NULL;
	unsigned int val = PENDING_STATE(client);

	__print_client_states(resource);

	if (!(client->type & NPA_CLIENT_REQUIRED))
		return resource->active_state;

	if ((resource->active_state == ACTIVE_STATE(client)) &&
		(ACTIVE_STATE(client) != val))
		return val;

	list_for_each_entry(cl_itr, &resource->clients, list) {
		if ((cl_itr->type & NPA_CLIENT_REQUIRED) && (cl_itr != client))
				val |= ACTIVE_STATE(cl_itr);
	}

	npa_log(NPA_LOG_MASK_PLUGIN, resource,
		"NPA: Binary plugin: Calculated [%u] for resource [%s] "
		"client [%s]\n",
		val, resource->definition->name, client->name);

	return val;
}

static unsigned int min_update(struct npa_resource *resource,
					struct npa_client *client)
{
	struct npa_client *cl_itr = NULL;
	unsigned int val = PENDING_STATE(client);

	__print_client_states(resource);

	if (!(client->type & NPA_CLIENT_REQUIRED))
		return resource->active_state;

	if (val && (resource->active_state == ACTIVE_STATE(client)) &&
		(val <= ACTIVE_STATE(client))) {

		npa_log(NPA_LOG_MASK_PLUGIN, resource,
			"NPA: Min plugin: Current request [%u] is min for "
			"resource [%s]\n",
			val, resource->definition->name);

		return val;
	}

	list_for_each_entry(cl_itr, &resource->clients, list) {
		if (!(cl_itr->type & NPA_CLIENT_REQUIRED))
			continue;
		if ((cl_itr != client) && ACTIVE_STATE(cl_itr)) {
			if (!val)
				val = ACTIVE_STATE(cl_itr);
			if (ACTIVE_STATE(cl_itr) < val)
				val = ACTIVE_STATE(cl_itr);
		}
	}

	npa_log(NPA_LOG_MASK_PLUGIN, resource,
		"NPA: Min plugin: Calculated [%u] for resource [%s] "
		"client [%s]\n",
		val, resource->definition->name, client->name);

	return val;
}

static unsigned int max_update(struct npa_resource *resource,
					struct npa_client *client)
{
	struct npa_client *cl_itr = NULL;
	unsigned int val = PENDING_STATE(client);

	__print_client_states(resource);

	if (!(client->type & NPA_CLIENT_REQUIRED))
		return resource->active_state;

	if ((val != 0) &&
		(resource->active_state == ACTIVE_STATE(client)) &&
		(val >= ACTIVE_STATE(client))) {

		npa_log(NPA_LOG_MASK_PLUGIN, resource,
			"NPA: Max plugin: Current request [%u] is max for "
			"resource [%s]\n",
			val, resource->definition->name);

		return val;
	}

	list_for_each_entry(cl_itr, &resource->clients, list) {
		if (!(cl_itr->type & NPA_CLIENT_REQUIRED))
			continue;
		if ((cl_itr != client) && ACTIVE_STATE(cl_itr)) {
			if (!val)
				val = ACTIVE_STATE(cl_itr);
			if (ACTIVE_STATE(cl_itr) > val)
				val = ACTIVE_STATE(cl_itr);
		}
	}

	npa_log(NPA_LOG_MASK_PLUGIN, resource,
		"NPA: Max plugin: Calculated [%u] for resource [%s] "
		"client [%s]\n",
		val, resource->definition->name, client->name);

	return val;
}

static unsigned int sum_update(struct npa_resource *resource,
					struct npa_client *client)
{
	unsigned int val = resource->active_state;

	__print_client_states(resource);

	if (!(client->type & NPA_CLIENT_REQUIRED))
		return resource->active_state;

	val -= ACTIVE_STATE(client);
	val += PENDING_STATE(client);

	npa_log(NPA_LOG_MASK_PLUGIN, resource,
		"NPA: Sum plugin: Calculated [%u] for resource [%s] "
		"client [%s]\n",
		val, resource->definition->name, client->name);

	return val;
}

static unsigned int always_update(struct npa_resource *resource,
					struct npa_client *client)
{
	unsigned int val = 1;

	__print_client_states(resource);

	if (!(client->type & NPA_CLIENT_REQUIRED))
		return resource->active_state;

	npa_log(NPA_LOG_MASK_PLUGIN, resource,
		"NPA: Always on plugin: Calculated [%u] for resource [%s] "
		"client [%s]\n",
		val, resource->definition->name, client->name);

	return val;
}

const struct npa_resource_plugin_ops npa_binary_plugin = {
	.update_fn 		= binary_update,
	.supported_clients 	= NPA_CLIENT_REQUIRED,
	.create_client_fn 	= NULL,
	.destroy_client_fn	= NULL,
};
EXPORT_SYMBOL(npa_binary_plugin);

const struct npa_resource_plugin_ops npa_min_plugin = {
	.update_fn 		= min_update,
	.supported_clients 	= NPA_CLIENT_REQUIRED,
	.create_client_fn 	= NULL,
	.destroy_client_fn	= NULL,
};
EXPORT_SYMBOL(npa_min_plugin);

const struct npa_resource_plugin_ops npa_max_plugin = {
	.update_fn 		= max_update,
	.supported_clients 	= NPA_CLIENT_REQUIRED,
	.create_client_fn 	= NULL,
	.destroy_client_fn	= NULL,
};
EXPORT_SYMBOL(npa_max_plugin);

const struct npa_resource_plugin_ops npa_sum_plugin = {
	.update_fn 		= sum_update,
	.supported_clients 	= NPA_CLIENT_REQUIRED,
	.create_client_fn 	= NULL,
	.destroy_client_fn	= NULL,
};
EXPORT_SYMBOL(npa_sum_plugin);

const struct npa_resource_plugin_ops npa_always_on_plugin = {
	.update_fn 		= always_update,
	.supported_clients 	= NPA_CLIENT_REQUIRED,
	.create_client_fn 	= NULL,
	.destroy_client_fn	= NULL,
};
EXPORT_SYMBOL(npa_always_on_plugin);

/* NPA Init function. */
static int npa_init(void)
{
	npa_wq = create_workqueue("npa");
	BUG_ON(!npa_wq);

	pr_info("NPA: Init done.\n");

	return 0;
}

#ifdef CONFIG_MSM_NPA_DEBUG
/* ** DEBUG use only **
 * NPA reset function. Releases all resources and resource states.
 * This function should be called only when there all the clients and events
 * are destroyed. No request should be made during this call.
 */
void npa_reset(void)
{
	struct npa_alias_list *a = NULL;
	struct npa_alias_list *b = NULL;
	struct npa_alias_list *temp_a = NULL;
	struct npa_event *e = NULL;
	struct npa_event *temp_e = NULL;
	struct npa_client *c = NULL;
	struct npa_client *temp_c = NULL;
	struct npa_resource *resource = NULL;
	struct npa_resource_definition *def = NULL;
	unsigned int count = 0;

	write_lock(&list_lock);
	list_for_each_entry_safe(a, temp_a, &alias_list, list) {
		list_del(&a->list);
		kfree(a);
		if (!resource)
			continue;
		resource = a->resource;
		list_for_each_entry_safe(e, temp_e, &resource->events, list) {
			list_del(&e->list);
			kfree(e);
		}
		list_for_each_entry_safe(e, temp_e, &resource->watermarks,
				list){
			list_del(&e->list);
			kfree(e);
		}
		list_for_each_entry_safe(c, temp_c, &resource->clients, list) {
			list_del(&c->list);
			kfree(c);
		}
		if (!__is_active_resource(resource)) {
			kfree(resource->definition);
			kfree(resource->node);
		}
		list_for_each_entry(b, &alias_list, list) {
			if (b->resource == resource)
				b->resource = NULL;
		}
		resource->definition->resource = NULL;
		kfree(resource->resource_lock);
		def = resource->node->resources;
		count = resource->node->resource_count;
		while (count) {
			if (def->resource)
				def->resource->resource_lock = NULL;
			count--;
			def++;
		}
		kfree(resource);
	}
	write_unlock(&list_lock);
}
EXPORT_SYMBOL(npa_reset);
#endif

/* NPA needs to be made available before the resources. Should be way up in the
 * initialization list of the kernel.
 */
postcore_initcall(npa_init);
