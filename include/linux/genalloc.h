/*
 * Basic general purpose allocator for managing special purpose memory
 * not managed by the regular kmalloc/kfree interface.
 * Uses for this includes on-device special memory, uncached memory
 * etc.
 *
 * This source code is licensed under the GNU General Public License,
 * Version 2.  See the file COPYING for more details.
 */

struct gen_pool;

struct gen_pool *__must_check gen_pool_create(unsigned order, int nid);

int __must_check gen_pool_add(struct gen_pool *pool, unsigned long addr,
			      size_t size, int nid);

void gen_pool_destroy(struct gen_pool *pool);

unsigned long __must_check
gen_pool_alloc_aligned(struct gen_pool *pool, size_t size,
		       unsigned alignment_order);

/**
 * gen_pool_alloc() - allocate special memory from the pool
 * @pool:	Pool to allocate from.
 * @size:	Number of bytes to allocate from the pool.
 *
 * Allocate the requested number of bytes from the specified pool.
 * Uses a first-fit algorithm.
 */
static inline unsigned long __must_check
gen_pool_alloc(struct gen_pool *pool, size_t size)
{
	return gen_pool_alloc_aligned(pool, size, 0);
}

void gen_pool_free(struct gen_pool *pool, unsigned long addr, size_t size);
