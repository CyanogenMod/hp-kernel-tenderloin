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

/*
 * Node Power Architecture (NPA) remote header file.
 */

#ifndef NPA_REMOTE_H
#define NPA_REMOTE_H

#include <linux/errno.h>

#include "npa.h"
#include "npa_resource.h"

#ifdef CONFIG_MSM_NPA_REMOTE

#define NPA_REMOTE_VERSION_MAJOR	1
#define NPA_REMOTE_VERSION_MINOR	0
#define NPA_REMOTE_VERSION_BUILD	0

/* Return Error Codes */
#define NPA_REMOTE_PROTOCOL_FAILURE	-1
#define NPA_REMOTE_FAILURE		1

enum npa_remote_client_type {
	NPA_REMOTE_CLIENT_RESERVED1,
	NPA_REMOTE_CLIENT_RESERVED2,
	NPA_REMOTE_CLIENT_CUSTOM1,
	NPA_REMOTE_CLIENT_CUSTOM2,
	NPA_REMOTE_CLIENT_CUSTOM3,
	NPA_REMOTE_CLIENT_CUSTOM4,
	NPA_REMOTE_CLIENT_REQUIRED,
	NPA_REMOTE_CLIENT_ISOCHRONOUS,
	NPA_REMOTE_CLIENT_IMPULSE,
	NPA_REMOTE_CLIENT_LIMIT_MAX,
	NPA_REMOTE_CLIENT_SIZE = 0x7FFFFFFF, /* Signed 32 bit max */
};

typedef int (*npa_remote_callback)(void *context, unsigned int event_type,
		int *data, unsigned int data_size);

/* NPA Remoting Library API. */

/* NULL procedure to test if the remote server is reachable or not.
 */
int npa_remote_null(void);

/* Remote setup initialization.
 *
 * @major: The major number of this remoting API.
 * @minor: The minor number of this remoting API.
 * @build: The build number of this remoting API.
 * @calback: The callback function that will be invoked when the version
 * number matches.
 * @context: The context pointer that is passed will be returned.
 *
 * Return:
 *   0: Success, the version information matches
 *   NPA_REMOTE_FAILURE: The server returned a failure.
 *   NPA_REMOTE_PROTOCOL_FAILURE: The transport protocol layer failed.
 */
int npa_remote_init(unsigned int major, unsigned int minor, unsigned int build,
		npa_remote_callback callback, void *context);

/* Checks if the remote resource is available.
 * The return value does not indicate the availability of the resource.
 * If the resource is available the callback is called, if its not, then
 * the callback is executed when the resource becomes available.
 *
 * @resource_name: NULL terminated name limited to NPA_NAME_MAX
 * @callback: The callback to be invoked when the resource is available.
 * @context: The passed in context pointer that will be returned with the
 * callback.
 *
 * Returns:
 *   0: Success if the server received the request.
 *   NPA_REMOTE_FAILURE: The server returned a failure.
 *   NPA_REMOTE_PROTOCOL_FAILURE: The transport protocol layer failed.
 */
int npa_remote_resource_available(const char *resource_name,
		npa_remote_callback callback, void *context);

/* Creates a remote client on the server.
 * The server returns a handle that identifies the npa_client on the server.
 *
 * @resource_name: NULL terminated name limited to NPA_NAME_MAX
 * @client_name: NULL terminated name limited to NPA_NAME_MAX
 * @client_type: The type of the sync client to be created.
 * @handle: The handle to the remote client returned by the server.
 *
 * Returns:
 *   0: Success if the server created a remote client.
 *   NPA_REMOTE_FAILURE: The server returned a failure.
 *   NPA_REMOTE_PROTOCOL_FAILURE: The transport protocol layer failed.
 */
int npa_remote_create_sync_client(const char *resource_name,
		const char *client_name,
		enum npa_remote_client_type client_type,
		void **handle);

/* Destroy a remote client.
 *
 * @handle: Handle to the remote client.
 *
 * Returns:
 *   0: Success if the server received the request.
 *   NPA_REMOTE_FAILURE: The server returned a failure.
 *   NPA_REMOTE_PROTOCOL_FAILURE: The transport protocol layer failed.
 */
int npa_remote_destroy_client(void *handle);

/* Issues a required request to the remote server through the client handle.
 *
 * @handle: Handle to the remote client
 * @state: Required new state
 * @new_state: The resource state after this request. The state returned
 * will be the old state if the remote request failed.
 *
 * Returns:
 *   0: Success if the server received the request.
 *   NPA_REMOTE_FAILURE: The server returned a failure.
 *   NPA_REMOTE_PROTOCOL_FAILURE: The transport protocol layer failed.
 *
 * Note: If CONFIG_MSM_NPA_PROC_COMM is defined, then this API uses PROC COMM
 * to issue requests to the modem.
 */
int npa_remote_issue_required_request(void *handle, unsigned int state,
		unsigned int *new_state);

/** PUBLIC NPA REMOTE API **/

/* Helper Driver functions */
unsigned int npa_remote_agg_driver_fn(struct npa_resource *resource,
		struct npa_client *client, unsigned int state);
unsigned int npa_local_agg_driver_fn(struct npa_resource *resource,
		struct npa_client *client, unsigned int state);

/* Remote aggregation plugin */
extern const struct npa_resource_plugin_ops npa_remote_agg_plugin;

#define DECLARE_RESOURCE_LOCAL_AGGREGATION(n, r, r_name, u_name, val, p) \
	struct npa_resource_definition r = { \
		.name = r_name, \
		.units = u_name, \
		.attributes = NPA_RESOURCE_DEFAULT, \
		.max = val, \
		.plugin = &p, \
		.data = NULL, \
	}; \
	struct npa_node_definition n = { \
		.name = r_name, \
		.attributes = NPA_NODE_DEFAULT, \
		.driver_fn = npa_local_agg_driver_fn, \
		.dependencies = NULL, \
		.dependency_count = 0, \
		.resources = &r, \
		.resource_count = 1, \
	};

#define DECLARE_RESOURCE_REMOTE_AGGREGATION(n, r, r_name, u_name, val) \
	struct npa_resource_definition r = { \
		.name = r_name, \
		.units = u_name, \
		.attributes = NPA_RESOURCE_DEFAULT, \
		.max = val, \
		.plugin = &npa_remote_agg_plugin, \
		.data = NULL, \
	}; \
	struct npa_node_definition n = {\
		.name = r_name, \
		.attributes = NPA_NODE_DEFAULT, \
		.driver_fn = npa_remote_agg_driver_fn, \
		.dependencies = NULL, \
		.dependency_count = 0, \
		.resources = &r, \
		.resource_count = 1, \
	};

/* Defines the remote node.
 *
 * @initial_state: The initial state vector of the resources.
 * @callback: Function to be called when the resource is defined and active
 * on the remote processor .
 * @user_data: Argument to the callback function.
 *
 * NOTE: Remoting supports only single resource node. If there are multiple
 * resources for the actual node, multiple remote nodes need to be created.
 *
 * Return:
 *   -ENODEV: If the remote server was not initialized.
 *   -EINVAL: Invalid input.
 *   0: The resource is created or awaiting dependencies.
 */
int npa_remote_define_node(struct npa_node_definition *node,
		unsigned int init_state, npa_cb_fn callback, void *data);

#else

static inline int npa_remote_define_node(struct npa_node_definition *node,
		unsigned int init_state, npa_cb_fn callback, void *data)
{
	return -ENOSYS;
}

#endif

#endif
