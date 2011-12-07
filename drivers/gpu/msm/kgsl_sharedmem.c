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
static void _outer_cache_range_op(unsigned long addr, int size,
				  unsigned int flags)
{
	unsigned long end;

	for (end = addr; end < (addr + size); end += PAGE_SIZE) {
		unsigned long physaddr = 0;

		if (flags & KGSL_MEMFLAGS_VMALLOC_MEM)
			physaddr = page_to_phys(vmalloc_to_page((void *) end));
		else if (flags & KGSL_MEMFLAGS_HOSTADDR)
			physaddr = kgsl_virtaddr_to_physaddr(end);
		else if (flags & KGSL_MEMFLAGS_CONPHYS)
			physaddr = __pa(end);

		if (physaddr == 0) {
			KGSL_CORE_ERR("Unable to find physaddr for "
				"address: %x\n", (unsigned int)end);
			return;
		}

		if (flags & KGSL_MEMFLAGS_CACHE_FLUSH)
			outer_flush_range(physaddr, physaddr + PAGE_SIZE);
		else if (flags & KGSL_MEMFLAGS_CACHE_CLEAN)
			outer_clean_range(physaddr, physaddr + PAGE_SIZE);
		else if (flags & KGSL_MEMFLAGS_CACHE_INV)
			outer_inv_range(physaddr, physaddr + PAGE_SIZE);
	}
	mb();
}
#else
static void _outer_cache_range_op(unsigned long addr, int size,
				  unsigned int flags)
{
}
#endif

void kgsl_cache_range_op(unsigned long addr, int size,
			 unsigned int flags)
{
	BUG_ON(addr & (PAGE_SIZE - 1));
	BUG_ON(size & (PAGE_SIZE - 1));

	if (flags & KGSL_MEMFLAGS_CACHE_FLUSH)
		dmac_flush_range((const void *)addr,
				 (const void *)(addr + size));
	else if (flags & KGSL_MEMFLAGS_CACHE_CLEAN)
		dmac_clean_range((const void *)addr,
				 (const void *)(addr + size));
	else if (flags & KGSL_MEMFLAGS_CACHE_INV)
		dmac_inv_range((const void *)addr,
			       (const void *)(addr + size));

	_outer_cache_range_op(addr, size, flags);

}

int
kgsl_sharedmem_vmalloc(struct kgsl_memdesc *memdesc,
		       struct kgsl_pagetable *pagetable, size_t size)
{
	int result;

	size = ALIGN(size, PAGE_SIZE * 2);

	memdesc->hostptr = vmalloc(size);
	if (memdesc->hostptr == NULL) {
		KGSL_CORE_ERR("vmalloc(%d) failed\n", size);
		return -ENOMEM;
	}

	memdesc->size = size;
	memdesc->pagetable = pagetable;
	memdesc->priv = KGSL_MEMFLAGS_VMALLOC_MEM | KGSL_MEMFLAGS_CACHE_CLEAN;

	kgsl_cache_range_op((unsigned int) memdesc->hostptr,
			    size, KGSL_MEMFLAGS_CACHE_INV |
			    KGSL_MEMFLAGS_VMALLOC_MEM);

	result = kgsl_mmu_map(pagetable, (unsigned long) memdesc->hostptr,
			      memdesc->size,
			      GSL_PT_PAGE_RV | GSL_PT_PAGE_WV,
			      &memdesc->gpuaddr,
			      KGSL_MEMFLAGS_ALIGN8K |
			      KGSL_MEMFLAGS_VMALLOC_MEM);

	if (result) {
		vfree(memdesc->hostptr);
		memset(memdesc, 0, sizeof(*memdesc));
	} else {
		/* Add the allocation to the driver statistics */
		KGSL_STATS_ADD(size, kgsl_driver.stats.vmalloc,
			       kgsl_driver.stats.vmalloc_max);
	}

	return result;
}

int
kgsl_sharedmem_alloc_coherent(struct kgsl_memdesc *memdesc, size_t size)
{
	size = ALIGN(size, PAGE_SIZE);

	memdesc->hostptr = dma_alloc_coherent(NULL, size, &memdesc->physaddr,
					      GFP_KERNEL);
	if (!memdesc->hostptr) {
		KGSL_CORE_ERR("dma_alloc_coherent(%d) failed\n", size);
		return -ENOMEM;
	}

	memdesc->size = size;
	memdesc->priv = KGSL_MEMFLAGS_CONPHYS;

	/* Record statistics */

	KGSL_STATS_ADD(size, kgsl_driver.stats.coherent,
		       kgsl_driver.stats.coherent_max);

	return 0;
}

void
kgsl_sharedmem_free(struct kgsl_memdesc *memdesc)
{
	BUG_ON(memdesc == NULL);

	if (memdesc->size > 0) {
		if (memdesc->priv & KGSL_MEMFLAGS_VMALLOC_MEM) {
			if (memdesc->gpuaddr)
				kgsl_mmu_unmap(memdesc->pagetable,
					       memdesc->gpuaddr,
					       memdesc->size);

			if (memdesc->hostptr)
				vfree(memdesc->hostptr);

			kgsl_driver.stats.vmalloc -= memdesc->size;

		} else if (memdesc->priv & KGSL_MEMFLAGS_CONPHYS) {
			dma_free_coherent(NULL, memdesc->size,
					  memdesc->hostptr,
					  memdesc->physaddr);

			kgsl_driver.stats.coherent -= memdesc->size;
		}
		else
			BUG();
	}

	memset(memdesc, 0, sizeof(struct kgsl_memdesc));
}

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

uint kgsl_get_physaddr(const struct kgsl_memdesc *memdesc)
{
	BUG_ON(memdesc == NULL);
	BUG_ON(memdesc->size == 0);

	if ((memdesc->priv & KGSL_MEMFLAGS_CONPHYS) || memdesc->physaddr) {
		BUG_ON(memdesc->physaddr == 0);
		return memdesc->physaddr;
	}
#ifdef CONFIG_MSM_KGSL_MMU
	if ((memdesc->priv & KGSL_MEMFLAGS_HOSTADDR) || memdesc->hostptr) {
		uint addr;
		BUG_ON(memdesc->hostptr == NULL);
		addr = kgsl_virtaddr_to_physaddr((uint)memdesc->hostptr);
		BUG_ON(addr == 0);
		return addr;
	}
#endif
	KGSL_CORE_ERR("invalid memory type: %x\n", memdesc->priv);
	BUG();
	return 0;
}

int
kgsl_sharedmem_writel(const struct kgsl_memdesc *memdesc,
			unsigned int offsetbytes,
			uint32_t src)
{
	BUG_ON(memdesc == NULL || memdesc->hostptr == NULL);
	BUG_ON(offsetbytes + sizeof(unsigned int) > memdesc->size);

	kgsl_cffdump_setmem(kgsl_get_physaddr(memdesc) + offsetbytes,
		src, sizeof(uint));
	writel(src, memdesc->hostptr + offsetbytes);
	return 0;
}

int
kgsl_sharedmem_set(const struct kgsl_memdesc *memdesc, unsigned int offsetbytes,
			unsigned int value, unsigned int sizebytes)
{
	BUG_ON(memdesc == NULL || memdesc->hostptr == NULL);
	BUG_ON(offsetbytes + sizebytes > memdesc->size);

	kgsl_cffdump_setmem(kgsl_get_physaddr(memdesc) + offsetbytes, value,
		sizebytes);
	memset(memdesc->hostptr + offsetbytes, value, sizebytes);
	return 0;
}

