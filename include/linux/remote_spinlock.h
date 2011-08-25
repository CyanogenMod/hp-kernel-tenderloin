/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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
#ifndef __LINUX_REMOTE_SPINLOCK_H
#define __LINUX_REMOTE_SPINLOCK_H

#include <linux/spinlock.h>
#include <linux/mutex.h>

#include <asm/remote_spinlock.h>

/* Grabbing a local spin lock before going for a remote lock has several
 * advantages:
 * 1. Get calls to preempt enable/disable and IRQ save/restore for free.
 * 2. For UP kernel, there is no overhead.
 * 3. Reduces the possibility of executing the remote spin lock code. This is
 *    especially useful when the remote CPUs' mutual exclusion instructions
 *    don't work with the local CPUs' instructions. In such cases, one has to
 *    use software based mutex algorithms (e.g. Lamport's bakery algorithm)
 *    which could get expensive when the no. of contending CPUs is high.
 * 4. In the case of software based mutex algorithm the exection time will be
 *    smaller since the no. of contending CPUs is reduced by having just one
 *    contender for all the local CPUs.
 * 5. Get most of the spin lock debug features for free.
 * 6. The code will continue to work "gracefully" even when the remote spin
 *    lock code is stubbed out for debug purposes or when there is no remote
 *    CPU in some board/machine types.
 */
typedef struct {
	spinlock_t local;
	_remote_spinlock_t remote;
} remote_spinlock_t;

#define remote_spin_lock_init(lock, id) \
	({ \
		spin_lock_init(&((lock)->local)); \
		_remote_spin_lock_init(id, &((lock)->remote)); \
	})
#define remote_spin_lock(lock) \
	do { \
		spin_lock(&((lock)->local)); \
		_remote_spin_lock(&((lock)->remote)); \
	} while (0)
#define remote_spin_unlock(lock) \
	do { \
		_remote_spin_unlock(&((lock)->remote)); \
		spin_unlock(&((lock)->local)); \
	} while (0)
#define remote_spin_lock_irqsave(lock, flags) \
	do { \
		spin_lock_irqsave(&((lock)->local), flags); \
		_remote_spin_lock(&((lock)->remote)); \
	} while (0)
#define remote_spin_unlock_irqrestore(lock, flags) \
	do { \
		_remote_spin_unlock(&((lock)->remote)); \
		spin_unlock_irqrestore(&((lock)->local), flags); \
	} while (0)
#define remote_spin_trylock(lock) \
	({ \
		spin_trylock(&((lock)->local)) \
		? _remote_spin_trylock(&((lock)->remote)) \
			? 1 \
			: ({ spin_unlock(&((lock)->local)); 0; }) \
		: 0; \
	})
#define remote_spin_trylock_irqsave(lock, flags) \
	({ \
		spin_trylock_irqsave(&((lock)->local), flags) \
		? _remote_spin_trylock(&((lock)->remote)) \
			? 1 \
			: ({ spin_unlock_irqrestore(&((lock)->local), flags); \
				0; }) \
		: 0; \
	})


typedef struct {
	struct mutex local;
	_remote_mutex_t remote;
} remote_mutex_t;

#define remote_mutex_init(lock, id) \
	({ \
		mutex_init(&((lock)->local)); \
		_remote_mutex_init(id, &((lock)->remote)); \
	})
#define remote_mutex_lock(lock) \
	do { \
		mutex_lock(&((lock)->local)); \
		_remote_mutex_lock(&((lock)->remote)); \
	} while (0)
#define remote_mutex_trylock(lock) \
	({ \
		mutex_trylock(&((lock)->local)) \
		? _remote_mutex_trylock(&((lock)->remote)) \
			? 1 \
			: ({mutex_unlock(&((lock)->local)); 0; }) \
		: 0; \
	})
#define remote_mutex_unlock(lock) \
	do { \
		_remote_mutex_unlock(&((lock)->remote)); \
		mutex_unlock(&((lock)->local)); \
	} while (0)

#endif
