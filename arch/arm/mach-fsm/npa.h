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
 * Node Power Architecture (NPA) header file.
 */

#ifndef NPA_H
#define NPA_H

#include <linux/err.h>
#include <linux/errno.h>

/* The max string length for a resource/client/event name */
#define NPA_NAME_MAX 64

/* Enums depicting client work flows */
enum npa_client_type {
	NPA_CLIENT_RESERVED1 	= (1<<0),
	NPA_CLIENT_RESERVED2 	= (1<<1),
	NPA_CLIENT_CUSTOM1 	= (1<<2),
	NPA_CLIENT_CUSTOM2 	= (1<<3),
	NPA_CLIENT_CUSTOM3 	= (1<<4),
	NPA_CLIENT_CUSTOM4 	= (1<<5),
	NPA_CLIENT_REQUIRED 	= (1<<6),
	NPA_CLIENT_ISOCHRONOUS 	= (1<<7),
	NPA_CLIENT_IMPULSE 	= (1<<8),
	NPA_CLIENT_LIMIT_MAX	= (1<<9),
};

/* Event types */
enum npa_event_type {
	NPA_EVENT_RESERVED1,	/* Reserved for NPA internal use */
	NPA_EVENT_LO_WATERMARK,	/* Resource state hit the low watermark */
	NPA_EVENT_HI_WATERMARK,	/* Resource state hit the high watermark */
	NPA_EVENT_CHANGE,	/* Resource state changed */
	NPA_NUM_EVENT_TYPES
};

/* Event data passed as the third argument to the event callback */
struct npa_event_data {
	const char		*resource_name;
	unsigned int		state;
	int			headroom; /* delta from the resource max */
};

/* The NPA callback function */
typedef void (*npa_cb_fn)(void *, unsigned int, void *, unsigned int);

#ifdef CONFIG_MSM_NPA
/* NPA CLIENT AND EVENT FUNCTIONS */

/* NPA functions in general do not copy strings and data pointed by pointers,
 * instead saves the pointers provided in the internal data structures.
 * It is imperative, that the data be available during the course of the
 * execution.
 *
 * Resource, client, event name string lengths are limited to NPA_NAME_MAX.
 */

/* Create NPA client for the provided work model.
 *
 * @resource_name: The resource name should be valid and defined.
 * @handler_name: The client identification. Useful in logging.
 * @type: The client type specified by npa_client_type enum. Client types
 * typically are required, impulse, isoc etc.
 *
 * Returns:
 *   Non-null handle: If the resource is available,
 *   -EINVAL: If the resource does not support the client type or invalid input.
 *   -ENODEV: If the resource is not active
 *   -EBUSY: If the resource single client requirement is already satisified.
 */
struct npa_client *npa_create_sync_client(const char *resource_name,
		const char *handler_name, enum npa_client_type type);

/* Deletes a client from NPA.
 *
 * @client: Should be client handle returned by the create client call.
 *
 * NOTE:
 * It is the responsibility of the caller to ensure that no requests are
 * pending for this client when the client is being destroyed.
 */
void npa_destroy_client(struct npa_client *client);

/* Issue a "required" workflow request to the resource.
 *
 * @client: Should be client handle returned by the create client call.
 * @state: The requested resource state. It is an absolute value.
 *
 * Note: A client can issue only one request at a time and its outside NPA
 * to synchronize the requests by the same client.
 */
int npa_issue_required_request(struct npa_client *client, unsigned int state);

/* Modify an issued required request.
 *
 * @client: Should be client handle returned by the create client call.
 * @delta: The requested new state. It is an relative value to the previous
 * request.
 *
 * Note: This function is helpful in tuning the resource request in a
 * feedback loop. The argument is a difference in relation to the previous
 * request by this client. This function should be called only when there is a
 * valid request issued. If called without a valid request returns error.
 *
 * Returns:
 *   0: Request issued.
 *   -EINVAL: Previous request did not exist for this resource.
 */
int npa_modify_required_request(struct npa_client *client, int delta);

/* Issue an impulse request.
 *
 * @client: Should be client handle returned by the create client call.
 *
 * Returns:
 *   0: Request issued.
 *   -EINVAL: Invalid request for this client.
 */
int npa_issue_impulse_request(struct npa_client *client);

/* Issue an isochronous request for a certain time.
 *
 * @client: Should be client handle returned by the create client call.
 * @level: Level is an hint to the resource to set the resource state to.
 *
 * Note: The @level is a hint to the resource and may not be the resultant
 * updated resource state.
 *
 * Returns:
 *   0: Request issued.
 *   -EINVAL: Invalid request for this client.
 */
int npa_issue_isoc_request(struct npa_client *client, unsigned int duration,
		unsigned int level);

/* Issue a limit_max request.
 *
 * @client: Should be client handle returned by the create client call.
 * @max: The limit max request value.
 *
 * Note: The minimum of all limit max requests will be used to limit the
 * resource max.
 *
 * Returns:
 *   0: Request issued.
 *   -EINVAL: Invalid request for this client.
 */
int npa_issue_limit_max_request(struct npa_client *client, unsigned int max);

/* Notify the resource that the request is no longer required.
 *
 * @client: Should be client handle returned by the create client call.
 */
void npa_complete_request(struct npa_client *client);

/* Cancel the requests issued by this client.
 *
 * @client: Should be client handle returned by the create client call.
 *
 * Note: This completes the request for the @client and in addition also,
 * indicates to the resource to flush out any historic data recorded for
 * this client.
 */
void npa_cancel_request(struct npa_client *client);

/* Get the current resource state.
 *
 * @client: Should be the client handle returned by the create client call.
 */
unsigned int npa_get_state(struct npa_client *client);

/* Notifies when the requested resource is available.
 *
 * If the resource is already available the callback will be called in the
 * same context as this call otherwise, otherwise will be invoked in a
 * different context.
 *
 * Return:
 *   0: Success. Callback will be called when the resource is availble.
 *   -EINVAL: Invalid input.
 */
int npa_resource_available(const char *resource_name,
		npa_cb_fn callback, void *user_data);

/* Create change event.
 *
 * @resource_name: The resource name should be valid and defined.
 * @handler_name: The event handler name, useful for logging.
 *
 * Note: Event callback occurs when the resource state changes.
 * The event callback will be called with the following arguments in order
 * (user_data passed as input, event-type, event-data pointer, int).
 * The int parameter is unused.
 * The callback will be first fired when the event is created and subsequently
 * whenever the resource state is changed.
 *
 * Return:
 *  Non-null handle: An handle to the event if the resource is available.
 *  -EINVAL: If the input is invalid.
 *  -ENODEV: If the resource is unavailable.
 */
struct npa_event *npa_create_change_event(const char *resource_name,
		const char *handler_name, npa_cb_fn event_cb, void *user_data);

/* Create watermark event.
 *
 * @resource_name: The resource name should be valid and defined.
 * @handler_name: The client identification. Useful in logging.
 *
 * Note:  Event callback occurs when the resource state hits either the high
 * or low watermark.
 * The event callback will be called with the following arguments in order
 * (user_data passed as input, event-type, event-data pointer, int).
 * The callback will not fired at creation.
 *
 * Return:
 *  Non-null handle: An handle to the event if the resource is available.
 *  -EINVAL: If the input is invalid.
 *  -ENODEV: If the resource is unavailable.
 */
struct npa_event *npa_create_watermark_event(const char *resource_name,
		const char *handler_name, npa_cb_fn event_cb, void *user_data);

/* Remove an change or watermark events.
 *
 * Note: The event should have been created  by NPA and returned in one of
 * the create_xxx_event functions.
 */
void npa_destroy_event(struct npa_event *event);

/* Set watermarks to trigger watermark events.
 *
 * Note: The event should have been created  by NPA and returned by the
 * npa_create_watermark_event function.
 * If the current state of the resource is outside the low or high watermark
 * then it will be fired immediately.
 *
 * Return:
 *   0: Success.
 *   -EINVAL: Invalid input.
 */
int npa_set_event_watermarks(struct npa_event *event,
		int lo_watermark, int hi_watermark);

#else
static inline struct npa_client *npa_create_sync_client(
		const char *resource_name,
		const char *handler_name,
		enum npa_client_type type) { return ERR_PTR(-ENOSYS); }
static inline void npa_destroy_client(struct npa_client *client) {}
static inline int npa_issue_required_request(struct npa_client *client,
		unsigned int state) { return -ENOSYS; }
static inline int npa_modify_required_request(struct npa_client *client,
		int delta) { return -ENOSYS; }
static inline int npa_issue_impulse_request(struct npa_client *client)
{ return -ENOSYS; }
static inline int npa_issue_isoc_request(struct npa_client *client,
		unsigned int duration,
		unsigned int level) { return -ENOSYS; }
static inline int npa_issue_limit_max_request(struct npa_client *client,
		unsigned int max) { return -ENOSYS; }
static inline void npa_complete_request(struct npa_client *client) {}
static inline void npa_cancel_request(struct npa_client *client) {}
static inline unsigned int npa_get_state(struct npa_client *client)
{ return 0; }
static inline int npa_resource_available(const char *resource_name,
		npa_cb_fn callback, void *user_data) { return -ENOSYS; }
static inline struct npa_event *npa_create_change_event(
		const char *resource_name,
		const char *handler_name, npa_cb_fn event_cb, void *user_data)
{ return ERR_PTR(-ENOSYS); }
static inline struct npa_event *npa_create_watermark_event(
		const char *resource_name,
		const char *handler_name, npa_cb_fn event_cb, void *user_data)
{ return ERR_PTR(-ENOSYS); }
static inline void npa_destroy_event(struct npa_event *event) {}
static inline int npa_set_event_watermarks(struct npa_event *event,
		int lo_watermark, int hi_watermark) { return -ENOSYS; }

#endif
#endif
