/* Copyright (c) 2002,2007-2011, Code Aurora Forum. All rights reserved.
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
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/genalloc.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <asm/cacheflush.h>

#include "kgsl_sharedmem.h"
#include "kgsl_device.h"
#include "kgsl.h"
#include "kgsl_log.h"
#include "kgsl_cffdump.h"

static struct kgsl_process_private *
_get_priv_from_kobj(struct kobject *kobj)
{
	struct kgsl_process_private *private;
	unsigned long name;

	if (!kobj)
		return NULL;

	if (sscanf(kobj->name, "%ld", &name) != 1)
		return NULL;

	list_for_each_entry(private, &kgsl_driver.process_list, list) {
		if (private->pid == name)
			return private;
	}

	return NULL;
}

/* sharedmem / memory sysfs files */

static ssize_t
process_show_vmalloc(struct kobject *kobj,
		   struct kobj_attribute *attr,
		   char *buf)
{

	struct kgsl_process_private *priv;
	int ret = 0;

	mutex_lock(&kgsl_driver.process_mutex);
	priv = _get_priv_from_kobj(kobj);

	if (priv)
		ret += sprintf(buf, "%d\n", priv->stats.vmalloc);

	mutex_unlock(&kgsl_driver.process_mutex);
	return ret;
}

static ssize_t
process_show_vmalloc_max(struct kobject *kobj,
		       struct kobj_attribute *attr,
		       char *buf)
{

	struct kgsl_process_private *priv;
	int ret = 0;

	mutex_lock(&kgsl_driver.process_mutex);
	priv = _get_priv_from_kobj(kobj);

	if (priv)
		ret += sprintf(buf, "%d\n", priv->stats.vmalloc_max);

	mutex_unlock(&kgsl_driver.process_mutex);
	return ret;
}

static ssize_t
process_show_exmem(struct kobject *kobj,
		 struct kobj_attribute *attr,
		 char *buf)
{

	struct kgsl_process_private *priv;
	int ret = 0;

	mutex_lock(&kgsl_driver.process_mutex);
	priv = _get_priv_from_kobj(kobj);

	if (priv)
		ret += sprintf(buf, "%d\n", priv->stats.exmem);

	mutex_unlock(&kgsl_driver.process_mutex);
	return ret;
}

static ssize_t
process_show_exmem_max(struct kobject *kobj,
		     struct kobj_attribute *attr,
		     char *buf)
{

	struct kgsl_process_private *priv;
	int ret = 0;

	mutex_lock(&kgsl_driver.process_mutex);
	priv = _get_priv_from_kobj(kobj);

	if (priv)
		ret += sprintf(buf, "%d\n", priv->stats.exmem_max);

	mutex_unlock(&kgsl_driver.process_mutex);
	return ret;
}

static ssize_t
process_show_flushes(struct kobject *kobj,
		   struct kobj_attribute *attr,
		   char *buf)
{
	struct kgsl_process_private *priv;
	int ret = 0;

	mutex_lock(&kgsl_driver.process_mutex);
	priv = _get_priv_from_kobj(kobj);

	if (priv)
		ret += sprintf(buf, "%d\n", priv->stats.flushes);

	mutex_unlock(&kgsl_driver.process_mutex);
	return ret;
}

static struct kobj_attribute attr_vmalloc = {
	.attr = { .name = "vmalloc", .mode = 0444 },
	.show = process_show_vmalloc,
	.store = NULL,
};

static struct kobj_attribute attr_vmalloc_max = {
	.attr = { .name = "vmalloc_max", .mode = 0444 },
	.show = process_show_vmalloc_max,
	.store = NULL,
};

static struct kobj_attribute attr_exmem = {
	.attr = { .name = "exmem", .mode = 0444 },
	.show = process_show_exmem,
	.store = NULL,
};

static struct kobj_attribute attr_exmem_max = {
	.attr = { .name = "exmem_max", .mode = 0444 },
	.show = process_show_exmem_max,
	.store = NULL,
};

static struct kobj_attribute attr_flushes = {
	.attr = { .name = "flushes", .mode = 0444 },
	.show = process_show_flushes,
	.store = NULL,
};

static struct attribute *process_attrs[] = {
	&attr_vmalloc.attr,
	&attr_vmalloc_max.attr,
	&attr_exmem.attr,
	&attr_exmem_max.attr,
	&attr_flushes.attr,
	NULL
};

static struct attribute_group process_attr_group = {
	.attrs = process_attrs,
};

void
kgsl_process_uninit_sysfs(struct kgsl_process_private *private)
{
	/* Remove the sysfs entry */
	if (private->kobj) {
		sysfs_remove_group(private->kobj, &process_attr_group);
		kobject_put(private->kobj);
	}
}

void
kgsl_process_init_sysfs(struct kgsl_process_private *private)
{
	unsigned char name[16];

	/* Add a entry to the sysfs device */
	snprintf(name, sizeof(name), "%d", private->pid);
	private->kobj = kobject_create_and_add(name, kgsl_driver.prockobj);

	/* sysfs failure isn't fatal, just annoying */
	if (private->kobj != NULL) {
		if (sysfs_create_group(private->kobj, &process_attr_group)) {
			kobject_put(private->kobj);
			private->kobj = NULL;
		}
	}
}

static int kgsl_drv_vmalloc_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	return sprintf(buf, "%d\n", kgsl_driver.stats.vmalloc);
}

static int kgsl_drv_vmalloc_max_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	return sprintf(buf, "%d\n", kgsl_driver.stats.vmalloc_max);
}

static int kgsl_drv_coherent_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	return sprintf(buf, "%d\n", kgsl_driver.stats.coherent);
}

static int kgsl_drv_coherent_max_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	return sprintf(buf, "%d\n", kgsl_driver.stats.coherent_max);
}

static int kgsl_drv_histogram_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	int len = 0;
	int i;

	for (i = 0; i < 16; i++)
		len += sprintf(buf + len, "%d ",
			kgsl_driver.stats.histogram[i]);

	len += sprintf(buf + len, "\n");
	return len;
}

static struct device_attribute drv_vmalloc_attr = {
	.attr = { .name = "vmalloc", .mode = 0444, },
	.show = kgsl_drv_vmalloc_show,
	.store = NULL,
};

static struct device_attribute drv_vmalloc_max_attr = {
	.attr = { .name = "vmalloc_max", .mode = 0444, },
	.show = kgsl_drv_vmalloc_max_show,
	.store = NULL,
};

static struct device_attribute drv_coherent_attr = {
	.attr = { .name = "coherent", .mode = 0444, },
	.show = kgsl_drv_coherent_show,
	.store = NULL,
};

static struct device_attribute drv_coherent_max_attr = {
	.attr = { .name = "coherent_max", .mode = 0444, },
	.show = kgsl_drv_coherent_max_show,
	.store = NULL,
};

static struct device_attribute drv_histogram_attr = {
	.attr = { .name = "histogram", .mode = 0444, },
	.show = kgsl_drv_histogram_show,
	.store = NULL,
};

void
kgsl_sharedmem_uninit_sysfs(void)
{
	device_remove_file(&kgsl_driver.virtdev, &drv_vmalloc_attr);
	device_remove_file(&kgsl_driver.virtdev, &drv_vmalloc_max_attr);
	device_remove_file(&kgsl_driver.virtdev, &drv_coherent_attr);
	device_remove_file(&kgsl_driver.virtdev, &drv_coherent_max_attr);
	device_remove_file(&kgsl_driver.virtdev, &drv_histogram_attr);
}

int
kgsl_sharedmem_init_sysfs(void)
{
	int ret;

	ret  = device_create_file(&kgsl_driver.virtdev,
				  &drv_vmalloc_attr);
	ret |= device_create_file(&kgsl_driver.virtdev,
				  &drv_vmalloc_max_attr);
	ret |= device_create_file(&kgsl_driver.virtdev,
				  &drv_coherent_attr);
	ret |= device_create_file(&kgsl_driver.virtdev,
				  &drv_coherent_max_attr);
	ret |= device_create_file(&kgsl_driver.virtdev,
				  &drv_histogram_attr);

	return ret;
}

#ifdef CONFIG_OUTER_CACHE
static void _outer_cache_range_op(int op, unsigned long addr, size_t size)
{
	switch (op) {
	case KGSL_CACHE_OP_FLUSH:
		outer_flush_range(addr, addr + size);
		break;
	case KGSL_CACHE_OP_CLEAN:
		outer_clean_range(addr, addr + size);
		break;
	case KGSL_CACHE_OP_INV:
		outer_inv_range(addr, addr + size);
		break;
	}

	mb();
}
#endif

static unsigned long kgsl_vmalloc_physaddr(struct kgsl_memdesc *memdesc,
					   unsigned int offset)
{
	unsigned int addr;

	if (offset > memdesc->size)
		return 0;

	addr = vmalloc_to_pfn(memdesc->hostptr + offset);
	return addr << PAGE_SHIFT;
}

#ifdef CONFIG_OUTER_CACHE
static void kgsl_vmalloc_outer_cache(struct kgsl_memdesc *memdesc, int op)
{
	void *vaddr = memdesc->hostptr;
	for (; vaddr < (memdesc->hostptr + memdesc->size); vaddr += PAGE_SIZE) {
		unsigned long paddr = page_to_phys(vmalloc_to_page(vaddr));
		_outer_cache_range_op(op, paddr, PAGE_SIZE);
	}
}
#endif

static int kgsl_vmalloc_vmfault(struct kgsl_memdesc *memdesc,
				struct vm_area_struct *vma,
				struct vm_fault *vmf)
{
	unsigned long offset, pg;
	struct page *page;

	offset = (unsigned long) vmf->virtual_address - vma->vm_start;
	pg = (unsigned long) memdesc->hostptr + offset;

	page = vmalloc_to_page((void *) pg);
	if (page == NULL)
		return VM_FAULT_SIGBUS;

	get_page(page);

	vmf->page = page;
	return 0;
}

static int kgsl_vmalloc_vmflags(struct kgsl_memdesc *memdesc)
{
	return VM_RESERVED | VM_DONTEXPAND;
}

static void kgsl_vmalloc_free(struct kgsl_memdesc *memdesc)
{
	kgsl_driver.stats.vmalloc -= memdesc->size;
	vfree(memdesc->hostptr);
}

static void kgsl_coherent_free(struct kgsl_memdesc *memdesc)
{
	kgsl_driver.stats.coherent -= memdesc->size;
	dma_free_coherent(NULL, memdesc->size,
			  memdesc->hostptr, memdesc->physaddr);
}

static unsigned long kgsl_contig_physaddr(struct kgsl_memdesc *memdesc,
					  unsigned int offset)
{
	if (offset > memdesc->size)
		return 0;

	return memdesc->physaddr + offset;
}

#ifdef CONFIG_OUTER_CACHE
static void kgsl_contig_outer_cache(struct kgsl_memdesc *memdesc, int op)
{
	_outer_cache_range_op(op, memdesc->physaddr, memdesc->size);
}
#endif

#ifdef CONFIG_OUTER_CACHE
static void kgsl_userptr_outer_cache(struct kgsl_memdesc *memdesc, int op)
{
	void *vaddr = memdesc->hostptr;
	for (; vaddr < (memdesc->hostptr + memdesc->size); vaddr += PAGE_SIZE) {
		unsigned long paddr = kgsl_virtaddr_to_physaddr(vaddr);
		if (paddr)
			_outer_cache_range_op(op, paddr, PAGE_SIZE);
	}
}
#endif

static unsigned long kgsl_userptr_physaddr(struct kgsl_memdesc *memdesc,
					   unsigned int offset)
{
	return kgsl_virtaddr_to_physaddr(memdesc->hostptr + offset);
}

/* Global - also used by kgsl_drm.c */
struct kgsl_memdesc_ops kgsl_vmalloc_ops = {
	.physaddr = kgsl_vmalloc_physaddr,
	.free = kgsl_vmalloc_free,
	.vmflags = kgsl_vmalloc_vmflags,
	.vmfault = kgsl_vmalloc_vmfault,
#ifdef CONFIG_OUTER_CACHE
	.outer_cache = kgsl_vmalloc_outer_cache,
#endif
};
EXPORT_SYMBOL(kgsl_vmalloc_ops);

static struct kgsl_memdesc_ops kgsl_coherent_ops = {
	.physaddr = kgsl_contig_physaddr,
	.free = kgsl_coherent_free,
#ifdef CONFIG_OUTER_CACHE
	.outer_cache = kgsl_contig_outer_cache,
#endif
};

/* Global - also used by kgsl.c and kgsl_drm.c */
struct kgsl_memdesc_ops kgsl_contig_ops = {
	.physaddr = kgsl_contig_physaddr,
#ifdef CONFIG_OUTER_CACHE
	.outer_cache = kgsl_contig_outer_cache
#endif
};
EXPORT_SYMBOL(kgsl_contig_ops);

/* Global - also used by kgsl.c */
struct kgsl_memdesc_ops kgsl_userptr_ops = {
	.physaddr = kgsl_userptr_physaddr,
#ifdef CONFIG_OUTER_CACHE
	.outer_cache = kgsl_userptr_outer_cache,
#endif
};
EXPORT_SYMBOL(kgsl_userptr_ops);

void kgsl_cache_range_op(struct kgsl_memdesc *memdesc, int op)
{
	void *addr = memdesc->hostptr;
	int size = memdesc->size;

	switch (op) {
	case KGSL_CACHE_OP_FLUSH:
		dmac_flush_range(addr, addr + size);
		break;
	case KGSL_CACHE_OP_CLEAN:
		dmac_clean_range(addr, addr + size);
		break;
	case KGSL_CACHE_OP_INV:
		dmac_inv_range(addr, addr + size);
		break;
	}

	if (memdesc->ops->outer_cache)
		memdesc->ops->outer_cache(memdesc, op);
}
EXPORT_SYMBOL(kgsl_cache_range_op);

static int
_kgsl_sharedmem_vmalloc(struct kgsl_memdesc *memdesc,
			struct kgsl_pagetable *pagetable,
			void *ptr, size_t size, unsigned int protflags)
{
	int result;

	memdesc->size = size;
	memdesc->pagetable = pagetable;
	memdesc->priv = KGSL_MEMFLAGS_CACHED;
	memdesc->ops = &kgsl_vmalloc_ops;
	memdesc->hostptr = (void *) ptr;

	kgsl_cache_range_op(memdesc, KGSL_CACHE_OP_INV);

	result = kgsl_mmu_map(pagetable, memdesc, protflags);

	if (result) {
		kgsl_sharedmem_free(memdesc);
	} else {
		int order;

		KGSL_STATS_ADD(size, kgsl_driver.stats.vmalloc,
			kgsl_driver.stats.vmalloc_max);

		order = get_order(size);

		if (order < 16)
			kgsl_driver.stats.histogram[order]++;
	}

	return result;
}

int
kgsl_sharedmem_vmalloc(struct kgsl_memdesc *memdesc,
		       struct kgsl_pagetable *pagetable, size_t size)
{
	void *ptr;

	BUG_ON(size == 0);

	size = ALIGN(size, PAGE_SIZE * 2);
	ptr = vmalloc(size);

	if (ptr  == NULL) {
		KGSL_CORE_ERR("vmalloc(%d) failed\n", size);
		return -ENOMEM;
	}

	return _kgsl_sharedmem_vmalloc(memdesc, pagetable, ptr, size,
		GSL_PT_PAGE_RV | GSL_PT_PAGE_WV);
}
EXPORT_SYMBOL(kgsl_sharedmem_vmalloc);

int
kgsl_sharedmem_vmalloc_user(struct kgsl_memdesc *memdesc,
			    struct kgsl_pagetable *pagetable,
			    size_t size, int flags)
{
	void *ptr;
	unsigned int protflags;

	BUG_ON(size == 0);
	ptr = vmalloc_user(size);

	if (ptr == NULL) {
		KGSL_CORE_ERR("vmalloc_user(%d) failed: allocated=%d\n",
			      size, kgsl_driver.stats.vmalloc);
		return -ENOMEM;
	}

	protflags = GSL_PT_PAGE_RV;
	if (!(flags & KGSL_MEMFLAGS_GPUREADONLY))
		protflags |= GSL_PT_PAGE_WV;

	return _kgsl_sharedmem_vmalloc(memdesc, pagetable, ptr, size,
		protflags);
}
EXPORT_SYMBOL(kgsl_sharedmem_vmalloc_user);

int
kgsl_sharedmem_alloc_coherent(struct kgsl_memdesc *memdesc, size_t size)
{
	size = ALIGN(size, PAGE_SIZE);

	memdesc->hostptr = dma_alloc_coherent(NULL, size, &memdesc->physaddr,
					      GFP_KERNEL);
	if (memdesc->hostptr == NULL) {
		KGSL_CORE_ERR("dma_alloc_coherent(%d) failed\n", size);
		return -ENOMEM;
	}

	memdesc->size = size;
	memdesc->ops = &kgsl_coherent_ops;

	/* Record statistics */

	KGSL_STATS_ADD(size, kgsl_driver.stats.coherent,
		       kgsl_driver.stats.coherent_max);

	return 0;
}
EXPORT_SYMBOL(kgsl_sharedmem_alloc_coherent);

void kgsl_sharedmem_free(struct kgsl_memdesc *memdesc)
{
	if (memdesc == NULL || memdesc->size == 0)
		return;

	if (memdesc->gpuaddr)
		kgsl_mmu_unmap(memdesc->pagetable, memdesc);

	if (memdesc->ops->free)
		memdesc->ops->free(memdesc);

	memset(memdesc, 0, sizeof(*memdesc));
}
EXPORT_SYMBOL(kgsl_sharedmem_free);

int
kgsl_sharedmem_readl(const struct kgsl_memdesc *memdesc,
			uint32_t *dst,
			unsigned int offsetbytes)
{
	BUG_ON(memdesc == NULL || memdesc->hostptr == NULL || dst == NULL);
	WARN_ON(offsetbytes + sizeof(unsigned int) > memdesc->size);

	if (offsetbytes + sizeof(unsigned int) > memdesc->size)
		return -ERANGE;

	*dst = readl(memdesc->hostptr + offsetbytes);
	return 0;
}
EXPORT_SYMBOL(kgsl_sharedmem_readl);

int
kgsl_sharedmem_writel(const struct kgsl_memdesc *memdesc,
			unsigned int offsetbytes,
			uint32_t src)
{
	BUG_ON(memdesc == NULL || memdesc->hostptr == NULL);
	BUG_ON(offsetbytes + sizeof(unsigned int) > memdesc->size);

	kgsl_cffdump_setmem(memdesc->physaddr + offsetbytes,
		src, sizeof(uint));
	writel(src, memdesc->hostptr + offsetbytes);
	return 0;
}
EXPORT_SYMBOL(kgsl_sharedmem_writel);

int
kgsl_sharedmem_set(const struct kgsl_memdesc *memdesc, unsigned int offsetbytes,
			unsigned int value, unsigned int sizebytes)
{
	BUG_ON(memdesc == NULL || memdesc->hostptr == NULL);
	BUG_ON(offsetbytes + sizebytes > memdesc->size);

	kgsl_cffdump_setmem(memdesc->physaddr + offsetbytes, value,
		sizebytes);
	memset(memdesc->hostptr + offsetbytes, value, sizebytes);
	return 0;
}
EXPORT_SYMBOL(kgsl_sharedmem_set);
