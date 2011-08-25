/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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
 * Part of this this code is based on the standard ARM spinlock
 * implementation (asm/spinlock.h) found in the 2.6.29 kernel.
 */

#ifndef __ASM__ARCH_QC_REMOTE_SPINLOCK_H
#define __ASM__ARCH_QC_REMOTE_SPINLOCK_H

#include <linux/types.h>

/* Remote spinlock definitions. */

struct dek_spinlock {
	volatile uint8_t self_lock;
	volatile uint8_t other_lock;
	volatile uint8_t next_yield;
	uint8_t pad;
};

typedef union {
	volatile uint32_t lock;
	struct dek_spinlock dek;
} raw_remote_spinlock_t;

typedef raw_remote_spinlock_t *_remote_spinlock_t;

#define remote_spinlock_id_t const char *

static inline void __raw_remote_ex_spin_lock(raw_remote_spinlock_t *lock)
{
	unsigned long tmp;

	__asm__ __volatile__(
"1:	ldrex	%0, [%1]\n"
"	teq	%0, #0\n"
"	strexeq	%0, %2, [%1]\n"
"	teqeq	%0, #0\n"
"	bne	1b"
	: "=&r" (tmp)
	: "r" (&lock->lock), "r" (1)
	: "cc");

	smp_mb();
}

static inline int __raw_remote_ex_spin_trylock(raw_remote_spinlock_t *lock)
{
	unsigned long tmp;

	__asm__ __volatile__(
"	ldrex	%0, [%1]\n"
"	teq	%0, #0\n"
"	strexeq	%0, %2, [%1]\n"
	: "=&r" (tmp)
	: "r" (&lock->lock), "r" (1)
	: "cc");

	if (tmp == 0) {
		smp_mb();
		return 1;
	}
	return 0;
}

static inline void __raw_remote_ex_spin_unlock(raw_remote_spinlock_t *lock)
{
	smp_mb();

	__asm__ __volatile__(
"	str	%1, [%0]\n"
	:
	: "r" (&lock->lock), "r" (0)
	: "cc");
}

static inline void __raw_remote_swp_spin_lock(raw_remote_spinlock_t *lock)
{
	unsigned long tmp;

	__asm__ __volatile__(
"1:	swp	%0, %2, [%1]\n"
"	teq	%0, #0\n"
"	bne	1b"
	: "=&r" (tmp)
	: "r" (&lock->lock), "r" (1)
	: "cc");

	smp_mb();
}

static inline int __raw_remote_swp_spin_trylock(raw_remote_spinlock_t *lock)
{
	unsigned long tmp;

	__asm__ __volatile__(
"	swp	%0, %2, [%1]\n"
	: "=&r" (tmp)
	: "r" (&lock->lock), "r" (1)
	: "cc");

	if (tmp == 0) {
		smp_mb();
		return 1;
	}
	return 0;
}

static inline void __raw_remote_swp_spin_unlock(raw_remote_spinlock_t *lock)
{
	smp_mb();

	__asm__ __volatile__(
"	str	%1, [%0]"
	:
	: "r" (&lock->lock), "r" (0)
	: "cc");
}

#define DEK_LOCK_REQUEST		1
#define DEK_LOCK_YIELD			(!DEK_LOCK_REQUEST)
#define DEK_YIELD_TURN_SELF		0
static inline void __raw_remote_dek_spin_lock(raw_remote_spinlock_t *lock)
{
	lock->dek.self_lock = DEK_LOCK_REQUEST;

	while (lock->dek.other_lock) {

		if (lock->dek.next_yield == DEK_YIELD_TURN_SELF)
			lock->dek.self_lock = DEK_LOCK_YIELD;

		while (lock->dek.other_lock)
			;

		lock->dek.self_lock = DEK_LOCK_REQUEST;
	}
	lock->dek.next_yield = DEK_YIELD_TURN_SELF;

	smp_mb();
}

static inline int __raw_remote_dek_spin_trylock(raw_remote_spinlock_t *lock)
{
	lock->dek.self_lock = DEK_LOCK_REQUEST;

	if (lock->dek.other_lock) {
		lock->dek.self_lock = DEK_LOCK_YIELD;
		return 0;
	}

	lock->dek.next_yield = DEK_YIELD_TURN_SELF;

	smp_mb();
	return 1;
}

static inline void __raw_remote_dek_spin_unlock(raw_remote_spinlock_t *lock)
{
	smp_mb();

	lock->dek.self_lock = DEK_LOCK_YIELD;
}

#ifdef CONFIG_MSM_SMD
int _remote_spin_lock_init(remote_spinlock_id_t, _remote_spinlock_t *lock);
#else
static inline
int _remote_spin_lock_init(remote_spinlock_id_t id, _remote_spinlock_t *lock)
{
	return -EINVAL;
}
#endif

#if defined(CONFIG_MSM_REMOTE_SPINLOCK_DEKKERS)
/* Use Dekker's algorithm when LDREX/STREX and SWP are unavailable for
 * shared memory */
#define _remote_spin_lock(lock)		__raw_remote_dek_spin_lock(*lock)
#define _remote_spin_unlock(lock)	__raw_remote_dek_spin_unlock(*lock)
#define _remote_spin_trylock(lock)	__raw_remote_dek_spin_trylock(*lock)
#elif defined(CONFIG_MSM_REMOTE_SPINLOCK_SWP)
/* Use SWP-based locks when LDREX/STREX are unavailable for shared memory. */
#define _remote_spin_lock(lock)		__raw_remote_swp_spin_lock(*lock)
#define _remote_spin_unlock(lock)	__raw_remote_swp_spin_unlock(*lock)
#define _remote_spin_trylock(lock)	__raw_remote_swp_spin_trylock(*lock)
#else
/* Use LDREX/STREX for shared memory locking, when available */
#define _remote_spin_lock(lock)		__raw_remote_ex_spin_lock(*lock)
#define _remote_spin_unlock(lock)	__raw_remote_ex_spin_unlock(*lock)
#define _remote_spin_trylock(lock)	__raw_remote_ex_spin_trylock(*lock)
#endif

/* Remote mutex definitions. */

typedef struct {
	_remote_spinlock_t	r_spinlock;
	uint32_t		delay_us;
} _remote_mutex_t;

struct remote_mutex_id {
	remote_spinlock_id_t	r_spinlock_id;
	uint32_t		delay_us;
};

#ifdef CONFIG_MSM_SMD
int _remote_mutex_init(struct remote_mutex_id *id, _remote_mutex_t *lock);
void _remote_mutex_lock(_remote_mutex_t *lock);
void _remote_mutex_unlock(_remote_mutex_t *lock);
int _remote_mutex_trylock(_remote_mutex_t *lock);
#else
static inline
int _remote_mutex_init(struct remote_mutex_id *id, _remote_mutex_t *lock)
{
	return -EINVAL;
}
static inline void _remote_mutex_lock(_remote_mutex_t *lock) {}
static inline void _remote_mutex_unlock(_remote_mutex_t *lock) {}
static inline int _remote_mutex_trylock(_remote_mutex_t *lock)
{
	return 0;
}
#endif

#endif /* __ASM__ARCH_QC_REMOTE_SPINLOCK_H */
