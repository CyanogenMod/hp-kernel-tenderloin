/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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
 *
 * Subsystem restart notifier API header
 *
 */

#ifndef _SUBSYS_NOTIFIER_H
#define _SUBSYS_NOTIFIER_H

#include <linux/notifier.h>

enum subsys_notif_type {
	SUBSYS_BEFORE_SHUTDOWN,
	SUBSYS_AFTER_SHUTDOWN,
	SUBSYS_BEFORE_POWERUP,
	SUBSYS_AFTER_POWERUP,
	SUBSYS_NOTIF_TYPE_COUNT
};

#if defined(CONFIG_MSM_SUBSYSTEM_RESTART)
/* Use the subsys_notif_register_notifier API to register for notifications for
 * a particular subsystem. This API will return a handle that can be used to
 * un-reg for notifications using the subsys_notif_unregister_notifier API by
 * passing in that handle as an argument.
 *
 * On receiving a notification, the second (unsigned long) argument of the
 * notifier callback will contain the notification type, and the third (void *)
 * argument will contain the handle that was returned by
 * subsys_notif_register_notifier.
 */
void *subsys_notif_register_notifier(
			const char *subsys_name, struct notifier_block *nb);
int subsys_notif_unregister_notifier(void *subsys_handle,
				struct notifier_block *nb);

/* Use the subsys_notif_init_subsys API to initialize the notifier chains form
 * a particular subsystem. This API will return a handle that can be used to
 * queue notifications using the subsys_notif_queue_notification API by passing
 * in that handle as an argument.
 */
void *subsys_notif_add_subsys(const char *);
int subsys_notif_queue_notification(void *subsys_handle,
					enum subsys_notif_type notif_type);
#else

static inline void *subsys_notif_register_notifier(
			const char *subsys_name, struct notifier_block *nb)
{
	return NULL;
}

static inline int subsys_notif_unregister_notifier(void *subsys_handle,
					struct notifier_block *nb)
{
	return 0;
}

static inline void *subsys_notif_add_subsys(const char *subsys_name)
{
	return NULL;
}

static inline int subsys_notif_queue_notification(void *subsys_handle,
					enum subsys_notif_type notif_type)
{
	return 0;
}
#endif /* CONFIG_MSM_SUBSYSTEM_RESTART */

#endif
