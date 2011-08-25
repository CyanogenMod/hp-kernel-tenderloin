/* Copyright (c) 2008-2011, Code Aurora Forum. All rights reserved.
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
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/android_pmem.h>
#include <linux/highmem.h>
#include <linux/vmalloc.h>
#include <linux/notifier.h>
#include <linux/pm_runtime.h>
#include <asm/atomic.h>

#include <linux/ashmem.h>

#include "kgsl.h"
#include "kgsl_mmu.h"
#include "kgsl_yamato.h"
#include "kgsl_g12.h"
#include "kgsl_cmdstream.h"
#include "kgsl_postmortem.h"

#include "kgsl_log.h"
#include "kgsl_drm.h"
#include "kgsl_cffdump.h"


static void kgsl_put_phys_file(struct file *file);

/* Allocate a new context id */

struct kgsl_context *
kgsl_create_context(struct kgsl_device_private *dev_priv)
{
	struct kgsl_context *context;
	int ret, id;

	context = kzalloc(sizeof(*context), GFP_KERNEL);

	if (context == NULL)
		return NULL;

	while (1) {
		if (idr_pre_get(&dev_priv->device->context_idr,
				GFP_KERNEL) == 0) {
			kfree(context);
			return NULL;
		}

		ret = idr_get_new(&dev_priv->device->context_idr,
				  context, &id);

		if (ret != -EAGAIN)
			break;
	}

	if (ret) {
		kfree(context);
		return NULL;
	}

	context->id = id;
	context->dev_priv = dev_priv;

	return context;
}

void
kgsl_destroy_context(struct kgsl_device_private *dev_priv,
		     struct kgsl_context *context)
{
	int id;

	if (context == NULL)
		return;

	/* Fire a bug if the devctxt hasn't been freed */
	BUG_ON(context->devctxt);

	id = context->id;
	kfree(context);

	idr_remove(&dev_priv->device->context_idr, id);
}

static int kgsl_runpending(struct kgsl_device *device)
{
	kgsl_cmdstream_memqueue_drain(device);

	return KGSL_SUCCESS;
}

static int kgsl_runpending_unlocked(struct kgsl_device *device)
{
	int ret;

	mutex_lock(&device->mutex);
	kgsl_check_suspended(device);
	ret = kgsl_runpending(device);
	mutex_unlock(&device->mutex);

	return ret;
}

static void kgsl_check_idle_locked(struct kgsl_device *device)
{
	if (device->pwrctrl.nap_allowed == true &&
	    device->state & KGSL_STATE_ACTIVE) {
		device->requested_state = KGSL_STATE_NAP;
		if (kgsl_pwrctrl_sleep(device) == KGSL_FAILURE)
			mod_timer(&device->idle_timer,
				  jiffies +
				  device->pwrctrl.interval_timeout);
	}
}

static void kgsl_check_idle(struct kgsl_device *device)
{
	mutex_lock(&device->mutex);
	kgsl_check_idle_locked(device);
	mutex_unlock(&device->mutex);
}

static void kgsl_clean_cache_all(struct kgsl_process_private *private)
{
	struct kgsl_mem_entry *entry = NULL;

	spin_lock(&private->mem_lock);
	list_for_each_entry(entry, &private->mem_list, list) {
		if (KGSL_MEMFLAGS_CACHE_MASK & entry->memdesc.priv) {
			    kgsl_cache_range_op((unsigned long)entry->
						   memdesc.hostptr,
						   entry->memdesc.size,
						   entry->memdesc.priv);
		}
	}
	spin_unlock(&private->mem_lock);
}

/*this is used for logging, so that we can call the dev_printk
 functions without export struct kgsl_driver everywhere*/
struct device *kgsl_driver_getdevnode(void)
{
	BUG_ON(kgsl_driver.pdev == NULL);
	return &kgsl_driver.pdev->dev;
}

struct kgsl_device *kgsl_get_device(int dev_idx)
{
	BUG_ON(dev_idx >= KGSL_DEVICE_MAX || dev_idx < KGSL_DEVICE_YAMATO);
	return kgsl_driver.devp[dev_idx];
}

int kgsl_register_ts_notifier(struct kgsl_device *device,
			      struct notifier_block *nb)
{
	BUG_ON(device == NULL);
	return atomic_notifier_chain_register(&device->ts_notifier_list,
					      nb);
}

int kgsl_unregister_ts_notifier(struct kgsl_device *device,
				struct notifier_block *nb)
{
	BUG_ON(device == NULL);
	return atomic_notifier_chain_unregister(&device->ts_notifier_list,
						nb);
}

int kgsl_check_timestamp(struct kgsl_device *device, unsigned int timestamp)
{
	unsigned int ts_processed;
	BUG_ON(device->ftbl.device_cmdstream_readtimestamp == NULL);

	ts_processed = device->ftbl.device_cmdstream_readtimestamp(
			device, KGSL_TIMESTAMP_RETIRED);

	return timestamp_cmp(ts_processed, timestamp);
}

int kgsl_setstate(struct kgsl_device *device, uint32_t flags)
{
	int status = -ENXIO;

	if (flags && device->ftbl.device_setstate) {
		status = device->ftbl.device_setstate(device, flags);
	} else
		status = 0;

	return status;
}

int kgsl_idle(struct kgsl_device *device, unsigned int timeout)
{
	int status = -ENXIO;

	if (device->ftbl.device_idle)
		status = device->ftbl.device_idle(device, timeout);

	return status;
}


int kgsl_setup_pt(struct kgsl_pagetable *pt)
{
	int i = 0;
	int status = 0;

	for (i = 0; i < kgsl_driver.num_devs; i++) {
		struct kgsl_device *device = kgsl_driver.devp[i];
		if (device) {
			status = device->ftbl.device_setup_pt(device, pt);
			if (status)
				goto error_pt;
		}
	}
	return status;
error_pt:
	while (i >= 0) {
		struct kgsl_device *device = kgsl_driver.devp[i];
		if (device)
			device->ftbl.device_cleanup_pt(device, pt);
		i--;
	}
	return status;
}

int kgsl_cleanup_pt(struct kgsl_pagetable *pt)
{
	int i;
	for (i = 0; i < kgsl_driver.num_devs; i++) {
		struct kgsl_device *device = kgsl_driver.devp[i];
		if (device)
			device->ftbl.device_cleanup_pt(device, pt);
	}
	return 0;
}

/*Suspend function*/
static int kgsl_suspend(struct platform_device *dev, pm_message_t state)
{
	int i;
	struct kgsl_device *device;
	unsigned int nap_allowed_saved;

	KGSL_DRV_DBG("suspend start\n");

	for (i = 0; i < KGSL_DEVICE_MAX; i++) {
		device = kgsl_driver.devp[i];
		if (!device)
			continue;

		mutex_lock(&device->mutex);
		nap_allowed_saved = device->pwrctrl.nap_allowed;
		device->pwrctrl.nap_allowed = false;
		device->requested_state = KGSL_STATE_SUSPEND;
		/* Make sure no user process is waiting for a timestamp *
		 * before supending */
		if (device->active_cnt != 0) {
			mutex_unlock(&device->mutex);
			wait_for_completion(&device->suspend_gate);
			mutex_lock(&device->mutex);
		}
		/* Don't let the timer wake us during suspended sleep. */
		del_timer(&device->idle_timer);
		switch (device->state) {
		case KGSL_STATE_INIT:
			break;
		case KGSL_STATE_ACTIVE:
			/* Wait for the device to become idle */
			device->ftbl.device_idle(device, KGSL_TIMEOUT_DEFAULT);
		case KGSL_STATE_NAP:
		case KGSL_STATE_SLEEP:
			/* Get the completion ready to be waited upon. */
			INIT_COMPLETION(device->hwaccess_gate);
			device->ftbl.device_suspend_context(device);
			device->ftbl.device_stop(device);
			break;
		default:
			mutex_unlock(&device->mutex);
			return KGSL_FAILURE;
		}
		device->state = KGSL_STATE_SUSPEND;
		device->requested_state = KGSL_STATE_NONE;
		device->pwrctrl.nap_allowed = nap_allowed_saved;

		mutex_unlock(&device->mutex);
	}
	KGSL_DRV_DBG("suspend end\n");
	return KGSL_SUCCESS;
}

/*Resume function*/
static int kgsl_resume(struct platform_device *dev)
{
	int i, status = KGSL_SUCCESS;
	struct kgsl_device *device;

	KGSL_DRV_DBG("resume start\n");

	for (i = 0; i < KGSL_DEVICE_MAX; i++) {
		device = kgsl_driver.devp[i];
		if (!device)
			continue;

		mutex_lock(&device->mutex);
		if (device->state == KGSL_STATE_SUSPEND) {
			device->requested_state = KGSL_STATE_ACTIVE;
			status = device->ftbl.device_start(device, 0);
			if (status == KGSL_SUCCESS) {
				device->state = KGSL_STATE_ACTIVE;
			} else {
				KGSL_DRV_ERR("resume failed for dev->id=%d\n",
					device->id);
				device->state = KGSL_STATE_INIT;
				mutex_unlock(&device->mutex);
				return status;
			}
			status = device->ftbl.device_resume_context(device);
			complete_all(&device->hwaccess_gate);
		}
		device->requested_state = KGSL_STATE_NONE;
		mutex_unlock(&device->mutex);
	}
	KGSL_DRV_DBG("resume end\n");
	return status;
}

/* file operations */
static struct kgsl_process_private *
kgsl_get_process_private(struct kgsl_device_private *cur_dev_priv)
{
	struct kgsl_process_private *private;

	mutex_lock(&kgsl_driver.process_mutex);
	list_for_each_entry(private, &kgsl_driver.process_list, list) {
		if (private->pid == task_tgid_nr(current)) {
			private->refcnt++;
			goto out;
		}
	}

	/* no existing process private found for this dev_priv, create one */
	private = kzalloc(sizeof(struct kgsl_process_private), GFP_KERNEL);
	if (private == NULL) {
		KGSL_DRV_ERR("Error: cannot allocate process private data\n");
		goto out;
	}

	spin_lock_init(&private->mem_lock);
	private->refcnt = 1;
	private->pid = task_tgid_nr(current);

	INIT_LIST_HEAD(&private->mem_list);

#ifdef CONFIG_MSM_KGSL_MMU
	{
		struct kgsl_device *device;
		unsigned long pt_name;

#ifdef CONFIG_KGSL_PER_PROCESS_PAGE_TABLE
		pt_name = task_tgid_nr(current);
#else
		pt_name = KGSL_MMU_GLOBAL_PT;
#endif
		device = kgsl_get_device(KGSL_DEVICE_YAMATO);
		private->pagetable = kgsl_mmu_getpagetable(&device->mmu,
							   pt_name);
		if (private->pagetable == NULL) {
			KGSL_DRV_ERR("Error: Unable to get the page table\n");
			kfree(private);
			private = NULL;
			goto out;
		}
	}
#endif

	list_add(&private->list, &kgsl_driver.process_list);
out:
	mutex_unlock(&kgsl_driver.process_mutex);
	return private;
}

static void
kgsl_put_process_private(struct kgsl_device *device,
			 struct kgsl_process_private *private)
{
	struct kgsl_mem_entry *entry = NULL;
	struct kgsl_mem_entry *entry_tmp = NULL;

	if (!private)
		return;

	mutex_lock(&kgsl_driver.process_mutex);

	if (--private->refcnt)
		goto unlock;

	list_del(&private->list);

	list_for_each_entry_safe(entry, entry_tmp, &private->mem_list, list) {
		list_del(&entry->list);
		kgsl_destroy_mem_entry(entry);
	}

#ifdef CONFIG_MSM_KGSL_MMU
	if (private->pagetable != NULL)
		kgsl_mmu_putpagetable(private->pagetable);
#endif

	kfree(private);
unlock:
	mutex_unlock(&kgsl_driver.process_mutex);
}

static int kgsl_release(struct inode *inodep, struct file *filep)
{
	int result = 0;
	struct kgsl_device_private *dev_priv = NULL;
	struct kgsl_process_private *private = NULL;
	struct kgsl_device *device;
	struct kgsl_context *context;
	int next = 0;

	device = kgsl_driver.devp[iminor(inodep)];
	BUG_ON(device == NULL);

	dev_priv = (struct kgsl_device_private *) filep->private_data;
	BUG_ON(dev_priv == NULL);
	BUG_ON(device != dev_priv->device);
	/* private could be null if kgsl_open is not successful */
	private = dev_priv->process_priv;
	filep->private_data = NULL;

	mutex_lock(&device->mutex);
	kgsl_check_suspended(device);

	while (1) {
		context = idr_get_next(&dev_priv->device->context_idr, &next);
		if (context == NULL)
			break;

		if (context->dev_priv == dev_priv) {
			device->ftbl.device_drawctxt_destroy(device, context);
			kgsl_destroy_context(dev_priv, context);
		}

		next = next + 1;
	}

	device->open_count--;
	if (device->open_count == 0) {
		result = device->ftbl.device_stop(device);
		device->state = KGSL_STATE_INIT;
	}
	/* clean up any to-be-freed entries that belong to this
	 * process and this device
	 */
	kgsl_cmdstream_memqueue_cleanup(device, private);

	mutex_unlock(&device->mutex);
	kfree(dev_priv);

	kgsl_put_process_private(device, private);

	BUG_ON(kgsl_driver.pdev == NULL);
	pm_runtime_put(&kgsl_driver.pdev->dev);

	return result;
}

static int kgsl_open(struct inode *inodep, struct file *filep)
{
	int result;
	struct kgsl_device_private *dev_priv;
	struct kgsl_device *device;
	unsigned int minor = iminor(inodep);
	struct device *dev;

	BUG_ON(kgsl_driver.pdev == NULL);
	dev = &kgsl_driver.pdev->dev;

	result = pm_runtime_get_sync(dev);
	if (result < 0) {
		dev_err(dev,
			"Runtime PM: Unable to wake up the device, rc = %d\n",
			result);
		return result;
	}
	result = 0;

	KGSL_DRV_DBG("file %p pid %d minor %d\n",
		      filep, task_pid_nr(current), minor);

	device = kgsl_get_device(minor);
	BUG_ON(device == NULL);

	if (filep->f_flags & O_EXCL) {
		KGSL_DRV_ERR("O_EXCL not allowed\n");
		return -EBUSY;
	}

	dev_priv = kzalloc(sizeof(struct kgsl_device_private), GFP_KERNEL);
	if (dev_priv == NULL) {
		KGSL_DRV_ERR("cannot allocate device private data\n");
		result = -ENOMEM;
		goto err_pmruntime;
	}

	dev_priv->device = device;
	filep->private_data = dev_priv;

	/* Get file (per process) private struct */
	dev_priv->process_priv = kgsl_get_process_private(dev_priv);
	if (dev_priv->process_priv ==  NULL) {
		KGSL_DRV_ERR("cannot allocate or find file private data\n");
		result = -ENOMEM;
		goto err_freedevpriv;
	}

	mutex_lock(&device->mutex);
	kgsl_check_suspended(device);

	if (device->open_count == 0) {
		result = device->ftbl.device_start(device, true);

		if (result) {
			mutex_unlock(&device->mutex);
			goto err_putprocess;
		}
		device->state = KGSL_STATE_ACTIVE;
		KGSL_DRV_WARN("state -> ACTIVE, device %d\n", minor);
	}
	device->open_count++;
	mutex_unlock(&device->mutex);
	return result;

err_putprocess:
	kgsl_put_process_private(device, dev_priv->process_priv);
err_freedevpriv:
	filep->private_data = NULL;
	kfree(dev_priv);
err_pmruntime:
	pm_runtime_put(dev);
	return result;
}


/*call with private->mem_lock locked */
static struct kgsl_mem_entry *
kgsl_sharedmem_find(struct kgsl_process_private *private, unsigned int gpuaddr)
{
	struct kgsl_mem_entry *entry = NULL, *result = NULL;

	BUG_ON(private == NULL);

	list_for_each_entry(entry, &private->mem_list, list) {
		if (entry->memdesc.gpuaddr == gpuaddr) {
			result = entry;
			break;
		}
	}
	return result;
}

/*call with private->mem_lock locked */
struct kgsl_mem_entry *
kgsl_sharedmem_find_region(struct kgsl_process_private *private,
				unsigned int gpuaddr,
				size_t size)
{
	struct kgsl_mem_entry *entry = NULL, *result = NULL;

	BUG_ON(private == NULL);

	list_for_each_entry(entry, &private->mem_list, list) {
		if (gpuaddr >= entry->memdesc.gpuaddr &&
		    ((gpuaddr + size) <=
			(entry->memdesc.gpuaddr + entry->memdesc.size))) {
			result = entry;
			break;
		}
	}

	return result;
}

uint8_t *kgsl_gpuaddr_to_vaddr(const struct kgsl_memdesc *memdesc,
	unsigned int gpuaddr, unsigned int *size)
{
	uint8_t *ptr = NULL;

	if ((memdesc->priv & KGSL_MEMFLAGS_VMALLOC_MEM) &&
		(memdesc->physaddr || !memdesc->hostptr))
		ptr = (uint8_t *)memdesc->physaddr;
	else if (memdesc->hostptr == NULL)
		ptr = __va(memdesc->physaddr);
	else
		ptr = memdesc->hostptr;

	if (memdesc->size <= (gpuaddr - memdesc->gpuaddr))
		ptr = NULL;

	*size = ptr ? (memdesc->size - (gpuaddr - memdesc->gpuaddr)) : 0;
	return (uint8_t *)(ptr ? (ptr  + (gpuaddr - memdesc->gpuaddr)) : NULL);
}

uint8_t *kgsl_sharedmem_convertaddr(struct kgsl_device *device,
	unsigned int pt_base, unsigned int gpuaddr, unsigned int *size)
{
	uint8_t *result = NULL;
	struct kgsl_mem_entry *entry;
	struct kgsl_process_private *priv;
	struct kgsl_yamato_device *yamato_device = KGSL_YAMATO_DEVICE(device);
	struct kgsl_ringbuffer *ringbuffer = &yamato_device->ringbuffer;

	if (kgsl_gpuaddr_in_memdesc(&ringbuffer->buffer_desc, gpuaddr)) {
		return kgsl_gpuaddr_to_vaddr(&ringbuffer->buffer_desc,
					gpuaddr, size);
	}

	if (kgsl_gpuaddr_in_memdesc(&ringbuffer->memptrs_desc, gpuaddr)) {
		return kgsl_gpuaddr_to_vaddr(&ringbuffer->memptrs_desc,
					gpuaddr, size);
	}

	if (kgsl_gpuaddr_in_memdesc(&device->memstore, gpuaddr)) {
		return kgsl_gpuaddr_to_vaddr(&device->memstore,
					gpuaddr, size);
	}

	mutex_lock(&kgsl_driver.process_mutex);
	list_for_each_entry(priv, &kgsl_driver.process_list, list) {
		if (pt_base != 0
			&& priv->pagetable
			&& priv->pagetable->base.gpuaddr != pt_base) {
			continue;
		}

		spin_lock(&priv->mem_lock);
		entry = kgsl_sharedmem_find_region(priv, gpuaddr,
						sizeof(unsigned int));
		if (entry) {
			result = kgsl_gpuaddr_to_vaddr(&entry->memdesc,
							gpuaddr, size);
			spin_unlock(&priv->mem_lock);
			mutex_unlock(&kgsl_driver.process_mutex);
			return result;
		}
		spin_unlock(&priv->mem_lock);
	}
	mutex_unlock(&kgsl_driver.process_mutex);

	BUG_ON(!mutex_is_locked(&device->mutex));
	list_for_each_entry(entry, &device->memqueue, list) {
		if (kgsl_gpuaddr_in_memdesc(&entry->memdesc, gpuaddr)) {
			result = kgsl_gpuaddr_to_vaddr(&entry->memdesc,
							gpuaddr, size);
			break;
		}

	}
	return result;
}

/*call all ioctl sub functions with driver locked*/
static long kgsl_ioctl_device_getproperty(struct kgsl_device_private *dev_priv,
					 void __user *arg)
{
	int result = 0;
	struct kgsl_device_getproperty param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}
	result = dev_priv->device->ftbl.device_getproperty(dev_priv->device,
					 param.type,
					 param.value, param.sizebytes);
done:
	return result;
}

static long kgsl_ioctl_device_regread(struct kgsl_device_private *dev_priv,
				     void __user *arg)
{
	int result = 0;
	struct kgsl_device_regread param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	if (param.offsetwords*sizeof(uint32_t) >=
	    dev_priv->device->regspace.sizebytes) {
		KGSL_DRV_ERR("invalid offset %d\n", param.offsetwords);
		return -ERANGE;
	}

	kgsl_regread(dev_priv->device, param.offsetwords, &param.value);

	if (result != 0)
		goto done;

	if (copy_to_user(arg, &param, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}
done:
	return result;
}


static long kgsl_ioctl_device_waittimestamp(struct kgsl_device_private
						*dev_priv, void __user *arg)
{
	int result = 0;
	struct kgsl_device_waittimestamp param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	/* Don't wait forever, set a max value for now */
	if (param.timeout == -1)
		param.timeout = 10 * MSEC_PER_SEC;
	result = dev_priv->device->ftbl.device_waittimestamp(dev_priv->device,
				     param.timestamp,
				     param.timeout);

	kgsl_runpending(dev_priv->device);
done:
	return result;
}
static bool check_ibdesc(struct kgsl_device_private *dev_priv,
			 struct kgsl_ibdesc *ibdesc, unsigned int numibs,
			 bool parse)
{
	bool result = true;
	unsigned int i;
	for (i = 0; i < numibs; i++) {
		struct kgsl_mem_entry *entry;
		spin_lock(&dev_priv->process_priv->mem_lock);
		entry = kgsl_sharedmem_find_region(dev_priv->process_priv,
			ibdesc[i].gpuaddr, ibdesc[i].sizedwords * sizeof(uint));
		spin_unlock(&dev_priv->process_priv->mem_lock);
		if (entry == NULL) {
			KGSL_DRV_ERR("invalid cmd buffer gpuaddr %08x " \
						"sizedwords %d\n",
						ibdesc[i].gpuaddr,
						ibdesc[i].sizedwords);
			result = false;
			break;
		}

		if (parse && !kgsl_cffdump_parse_ibs(dev_priv, &entry->memdesc,
			ibdesc[i].gpuaddr, ibdesc[i].sizedwords, true)) {
			KGSL_DRV_ERR("invalid cmd buffer gpuaddr %08x " \
				"sizedwords %d numibs %d/%d\n",
				ibdesc[i].gpuaddr,
				ibdesc[i].sizedwords, i+1, numibs);
			result = false;
			break;
		}
	}
	return result;
}

static long kgsl_ioctl_rb_issueibcmds(struct kgsl_device_private *dev_priv,
				     void __user *arg)
{
	int result = 0;
	struct kgsl_ringbuffer_issueibcmds param;
	struct kgsl_ibdesc *ibdesc;
	struct kgsl_context *context;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	context = kgsl_find_context(dev_priv, param.drawctxt_id);
	if (context == NULL) {
		result = -EINVAL;
		KGSL_DRV_ERR("invalid drawctxt drawctxt_id %d\n",
				      param.drawctxt_id);
		goto done;
	}

	if (param.flags & KGSL_CONTEXT_SUBMIT_IB_LIST) {
		KGSL_DRV_INFO("Using IB list mode for ib submission, numibs:"
				" %d\n", param.numibs);
		if (!param.numibs) {
			KGSL_DRV_ERR("Invalid numibs as parameter: %d\n",
					param.numibs);
			result = -EINVAL;
			goto done;
		}

		ibdesc = kzalloc(sizeof(struct kgsl_ibdesc) * param.numibs,
					GFP_KERNEL);
		if (!ibdesc) {
			KGSL_MEM_ERR("kzalloc failed to allocate memory for "
				"ibdesc , size: %x\n",
				sizeof(struct kgsl_ibdesc) * param.numibs);
			result = -ENOMEM;
			goto done;
		}

		if (copy_from_user(ibdesc, (void *)param.ibdesc_addr,
				sizeof(struct kgsl_ibdesc) * param.numibs)) {
			result = -EFAULT;
			KGSL_DRV_ERR("Failed to copy ibdesc from user"
					" address space\n");
			goto free_ibdesc;
		}
	} else {
		KGSL_DRV_INFO("Using single IB submission mode for ib"
				" submission\n");
		/* If user space driver is still using the old mode of
		 * submitting single ib then we need to support that as well */
		ibdesc = kzalloc(sizeof(struct kgsl_ibdesc), GFP_KERNEL);
		if (!ibdesc) {
			KGSL_MEM_ERR("kzalloc failed to allocate memory for"
				" ibdesc, size: %x\n",
				sizeof(struct kgsl_ibdesc));
			result = -ENOMEM;
			goto done;
		}
		ibdesc[0].gpuaddr = param.ibdesc_addr;
		ibdesc[0].sizedwords = param.numibs;
		param.numibs = 1;
	}

	if (!check_ibdesc(dev_priv, ibdesc, param.numibs, true)) {
		KGSL_DRV_ERR("bad ibdesc");
		result = -EINVAL;
		goto free_ibdesc;
	}

	result = dev_priv->device->ftbl.device_issueibcmds(dev_priv,
					     context,
					     ibdesc,
					     param.numibs,
					     &param.timestamp,
					     param.flags);

	if (result != 0)
		goto free_ibdesc;

	/* this is a check to try to detect if a command buffer was freed
	 * during issueibcmds().
	 */
	if (!check_ibdesc(dev_priv, ibdesc, param.numibs, false)) {
		KGSL_DRV_ERR("bad ibdesc AFTER issue");
		result = -EINVAL;
		goto free_ibdesc;
	}

	if (copy_to_user(arg, &param, sizeof(param))) {
		result = -EFAULT;
		goto free_ibdesc;
	}
free_ibdesc:
	kfree(ibdesc);
done:
	return result;
}

static long kgsl_ioctl_cmdstream_readtimestamp(struct kgsl_device_private
						*dev_priv, void __user *arg)
{
	int result = 0;
	struct kgsl_cmdstream_readtimestamp param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	param.timestamp = dev_priv->device->ftbl.device_cmdstream_readtimestamp
							(dev_priv->device,
							param.type);

	if (copy_to_user(arg, &param, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}
done:
	return result;
}

static long kgsl_ioctl_cmdstream_freememontimestamp(struct kgsl_device_private
						*dev_priv, void __user *arg)
{
	int result = 0;
	struct kgsl_cmdstream_freememontimestamp param;
	struct kgsl_mem_entry *entry = NULL;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	spin_lock(&dev_priv->process_priv->mem_lock);
	entry = kgsl_sharedmem_find(dev_priv->process_priv, param.gpuaddr);
	if (entry)
		list_del(&entry->list);
	spin_unlock(&dev_priv->process_priv->mem_lock);

	if (entry) {
#ifdef CONFIG_MSM_KGSL_MMU
		if (entry->memdesc.priv & KGSL_MEMFLAGS_VMALLOC_MEM)
			entry->memdesc.priv &= ~KGSL_MEMFLAGS_CACHE_MASK;
#endif
		kgsl_cmdstream_freememontimestamp(dev_priv->device, entry,
						  param.timestamp, param.type);
		kgsl_runpending(dev_priv->device);
	} else {
		KGSL_DRV_ERR("invalid gpuaddr %08x\n", param.gpuaddr);
		result = -EINVAL;
	}
done:
	return result;
}

static long kgsl_ioctl_drawctxt_create(struct kgsl_device_private *dev_priv,
				      void __user *arg)
{
	int result = 0;
	struct kgsl_drawctxt_create param;
	struct kgsl_context *context = NULL;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	context = kgsl_create_context(dev_priv);

	if (context == NULL) {
		result = -ENOMEM;
		goto done;
	}

	result = dev_priv->device->ftbl.device_drawctxt_create(dev_priv,
					param.flags,
					context);

	if (result != 0)
		goto done;

	param.drawctxt_id = context->id;

	if (copy_to_user(arg, &param, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

done:
	if (result && context)
		kgsl_destroy_context(dev_priv, context);

	return result;
}

static long kgsl_ioctl_drawctxt_destroy(struct kgsl_device_private *dev_priv,
				       void __user *arg)
{
	int result = 0;
	struct kgsl_drawctxt_destroy param;
	struct kgsl_context *context;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	context = kgsl_find_context(dev_priv, param.drawctxt_id);

	if (context == NULL) {
		result = -EINVAL;
		goto done;
	}

	result = dev_priv->device->ftbl.device_drawctxt_destroy(
							dev_priv->device,
							context);

	kgsl_destroy_context(dev_priv, context);

done:
	return result;
}

void kgsl_destroy_mem_entry(struct kgsl_mem_entry *entry)
{
	kgsl_mmu_unmap(entry->memdesc.pagetable,
			entry->memdesc.gpuaddr & KGSL_PAGEMASK,
			entry->memdesc.size);
	if (KGSL_MEMFLAGS_VMALLOC_MEM & entry->memdesc.priv)
		vfree((void *)entry->memdesc.physaddr);
	else if (KGSL_MEMFLAGS_HOSTADDR & entry->memdesc.priv &&
			entry->file_ptr)
		put_ashmem_file(entry->file_ptr);
	else
		kgsl_put_phys_file(entry->file_ptr);

	kfree(entry);
}

static long kgsl_ioctl_sharedmem_free(struct kgsl_process_private *private,
				     void __user *arg)
{
	int result = 0;
	struct kgsl_sharedmem_free param;
	struct kgsl_mem_entry *entry = NULL;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	spin_lock(&private->mem_lock);
	entry = kgsl_sharedmem_find(private, param.gpuaddr);
	if (entry)
		list_del(&entry->list);
	spin_unlock(&private->mem_lock);

	if (entry) {
		kgsl_destroy_mem_entry(entry);
	} else {
		KGSL_DRV_ERR("invalid gpuaddr %08x\n", param.gpuaddr);
		result = -EINVAL;
	}
done:
	return result;
}

static struct vm_area_struct *kgsl_get_vma_from_start_addr(unsigned int addr)
{
	struct vm_area_struct *vma;
	int len;

	down_read(&current->mm->mmap_sem);
	vma = find_vma(current->mm, addr);
	up_read(&current->mm->mmap_sem);
	if (!vma) {
		KGSL_MEM_ERR("Could not find vma for address %x\n",
			   addr);
		return NULL;
	}
	len = vma->vm_end - vma->vm_start;
	if (vma->vm_pgoff || !KGSL_IS_PAGE_ALIGNED(len) ||
	  !KGSL_IS_PAGE_ALIGNED(vma->vm_start)) {
		KGSL_MEM_ERR
		("user address mapping must be at offset 0 and page aligned\n");
		return NULL;
	}
	if (vma->vm_start != addr) {
		KGSL_MEM_ERR
		  ("vma start address is not equal to mmap address\n");
		return NULL;
	}
	return vma;
}

#ifdef CONFIG_MSM_KGSL_MMU
static long
kgsl_ioctl_sharedmem_from_vmalloc(struct kgsl_process_private *private,
				  void __user *arg)
{
	int result = 0, len = 0;
	struct kgsl_sharedmem_from_vmalloc param;
	struct kgsl_mem_entry *entry = NULL;
	void *vmalloc_area;
	struct vm_area_struct *vma;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto error;
	}

	if (!param.hostptr) {
		KGSL_DRV_ERR
		    ("Invalid host pointer of malloc passed: param.hostptr "
		     "%08x\n", param.hostptr);
		result = -EINVAL;
		goto error;
	}

	vma = kgsl_get_vma_from_start_addr(param.hostptr);
	if (!vma) {
		result = -EINVAL;
		goto error;
	}
	len = vma->vm_end - vma->vm_start;

	entry = kzalloc(sizeof(struct kgsl_mem_entry), GFP_KERNEL);
	if (entry == NULL) {
		result = -ENOMEM;
		goto error;
	}

	/* allocate memory and map it to user space */
	vmalloc_area = vmalloc_user(len);
	if (!vmalloc_area) {
		KGSL_MEM_ERR("vmalloc failed\n");
		result = -ENOMEM;
		goto error_free_entry;
	}
	kgsl_cache_range_op((unsigned int)vmalloc_area, len,
		KGSL_MEMFLAGS_CACHE_INV | KGSL_MEMFLAGS_VMALLOC_MEM);

	result = kgsl_mmu_map(private->pagetable,
			      (unsigned long)vmalloc_area, len,
			      GSL_PT_PAGE_RV |
			      ((param.flags & KGSL_MEMFLAGS_GPUREADONLY) ?
			      0 : GSL_PT_PAGE_WV),
			      &entry->memdesc.gpuaddr, KGSL_MEMFLAGS_ALIGN4K |
			      KGSL_MEMFLAGS_VMALLOC_MEM);
	if (result != 0)
		goto error_free_vmalloc;

	entry->memdesc.pagetable = private->pagetable;
	entry->memdesc.size = len;
	entry->memdesc.priv = KGSL_MEMFLAGS_VMALLOC_MEM |
			    KGSL_MEMFLAGS_CACHE_CLEAN |
			    (param.flags & KGSL_MEMFLAGS_GPUREADONLY);
	entry->memdesc.physaddr = (unsigned long)vmalloc_area;
	entry->priv = private;

	if (!kgsl_cache_enable)
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	result = remap_vmalloc_range(vma, vmalloc_area, 0);
	if (result) {
		KGSL_MEM_ERR("remap_vmalloc_range returned %d\n", result);
		goto error_unmap_entry;
	}

	entry->memdesc.hostptr = (void *)param.hostptr;

	param.gpuaddr = entry->memdesc.gpuaddr;

	if (copy_to_user(arg, &param, sizeof(param))) {
		result = -EFAULT;
		goto error_unmap_entry;
	}
	spin_lock(&private->mem_lock);
	list_add(&entry->list, &private->mem_list);
	spin_unlock(&private->mem_lock);

	return 0;

error_unmap_entry:
	kgsl_mmu_unmap(private->pagetable, entry->memdesc.gpuaddr,
		       entry->memdesc.size);

error_free_vmalloc:
	vfree(vmalloc_area);

error_free_entry:
	kfree(entry);

error:
	return result;
}
#endif /*CONFIG_MSM_KGSL_MMU*/

static int kgsl_get_phys_file(int fd, unsigned long *start, unsigned long *len,
			      struct file **filep)
{
	struct file *fbfile;
	int put_needed;
	unsigned long vstart = 0;
	int ret = 0;
	dev_t rdev;
	struct fb_info *info;

	*filep = NULL;
	if (!get_pmem_file(fd, start, &vstart, len, filep))
		return 0;

	fbfile = fget_light(fd, &put_needed);
	if (fbfile == NULL)
		return -1;

	rdev = fbfile->f_dentry->d_inode->i_rdev;
	info = MAJOR(rdev) == FB_MAJOR ? registered_fb[MINOR(rdev)] : NULL;
	if (info) {
		*start = info->fix.smem_start;
		*len = info->fix.smem_len;
		ret = 0;
	} else
		ret = -1;
	fput_light(fbfile, put_needed);

	return ret;
}

static void kgsl_put_phys_file(struct file *file)
{
	KGSL_DRV_DBG("put phys file %p\n", file);
	if (file)
		put_pmem_file(file);
}

static int kgsl_ioctl_map_user_mem(struct kgsl_process_private *private,
						void __user *arg,
						unsigned int cmd)
{
	int result = 0;
	struct kgsl_map_user_mem param;
	struct kgsl_mem_entry *entry = NULL;
	unsigned long start = 0, len = 0;
	struct file *file_ptr = NULL;
	uint64_t total_offset;

	if (IOCTL_KGSL_SHAREDMEM_FROM_PMEM == cmd) {
		if (copy_from_user(&param, arg,
			sizeof(struct kgsl_sharedmem_from_pmem))) {
			result = -EFAULT;
			goto error;
		}
		param.memtype = KGSL_USER_MEM_TYPE_PMEM;
	} else if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto error;
	}

	switch (param.memtype) {
	case KGSL_USER_MEM_TYPE_PMEM:
		if (kgsl_get_phys_file(param.fd, &start,
					&len, &file_ptr)) {
			KGSL_DRV_ERR("Failed to get pmem region "
				"with fd(%d) details\n", param.fd);
			result = -EINVAL;
			goto error;
		}
		if (!param.len)
			param.len = len;

		total_offset = param.offset + param.len;
		if (total_offset > (uint64_t)len) {
			KGSL_DRV_ERR("%s: region too large "
					"0x%x + 0x%x >= 0x%lx\n",
				     __func__, param.offset, param.len, len);
			result = -EINVAL;
			goto error_put_file_ptr;
		}
		break;
	case KGSL_USER_MEM_TYPE_ADDR:
	case KGSL_USER_MEM_TYPE_ASHMEM:
	{
		struct vm_area_struct *vma;
#ifndef CONFIG_MSM_KGSL_MMU
			KGSL_DRV_ERR("cannot map non-contig "
				"memory as MMU is turned off\n");
			result = -EINVAL;
			goto error;
#endif
		if (!param.hostptr) {
			result = -EINVAL;
			goto error;
		}
		start = param.hostptr;

		if (param.memtype == KGSL_USER_MEM_TYPE_ADDR) {
			down_read(&current->mm->mmap_sem);
			vma = find_vma(current->mm, start);
			up_read(&current->mm->mmap_sem);

			if (!vma) {
				KGSL_MEM_ERR("Could not find vma for"
						"address %lx\n", start);
				result = -EINVAL;
				goto error;
			}

			/* We don't necessarily start at vma->vm_start */
			len = vma->vm_end - param.hostptr;

			if (!KGSL_IS_PAGE_ALIGNED(len) ||
					!KGSL_IS_PAGE_ALIGNED(start)) {
				KGSL_MEM_ERR("user address error: len(%lu)"
						"and start(0x%lx) must be page"
						"aligned\n", len, start);
				result = -EINVAL;
				goto error;
			}
		} else {
			vma = kgsl_get_vma_from_start_addr(param.hostptr);
			if (vma == NULL) {
				result = -EINVAL;
				goto error;
			}
			len = vma->vm_end - vma->vm_start;
		}

		if (!param.len)
			param.len = len;
		if (param.memtype == KGSL_USER_MEM_TYPE_ASHMEM) {
			struct file *ashmem_vm_file;
			if (get_ashmem_file(param.fd, &file_ptr,
					&ashmem_vm_file, &len)) {
				KGSL_DRV_ERR("could not get ashmem "
						"file pointer\n");
				result = -EINVAL;
				goto error;
			}
			if (ashmem_vm_file != vma->vm_file) {
				KGSL_DRV_ERR("ashmem shmem file(%p) does not "
				"match to given vma->vm_file(%p)\n",
				ashmem_vm_file, vma->vm_file);
				result = -EINVAL;
				goto error_put_file_ptr;
			}
			if (len != (vma->vm_end - vma->vm_start)) {
				KGSL_DRV_ERR("ashmem region len(%ld) does not "
				"match vma region len(%ld)",
				len, vma->vm_end - vma->vm_start);
				result = -EINVAL;
				goto error_put_file_ptr;
			}
		}
		break;
	}
	default:
		KGSL_MEM_ERR("Invalid memory type used\n");
		result = -EINVAL;
		goto error;
	}

	KGSL_MEM_INFO("get phys file %p start 0x%lx len 0x%lx\n",
		      file_ptr, start, len);
	KGSL_DRV_DBG("locked phys file %p\n", file_ptr);

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (entry == NULL) {
		result = -ENOMEM;
		goto error_put_file_ptr;
	}

	entry->file_ptr = file_ptr;

	entry->memdesc.pagetable = private->pagetable;

	/* Any MMU mapped memory must have a length in multiple of PAGESIZE */
	entry->memdesc.size = ALIGN(param.len, PAGE_SIZE);
	/*we shouldn't need to write here from kernel mode */
	entry->memdesc.hostptr = NULL;
	/* ensure that MMU mappings are at page boundary */
	entry->memdesc.physaddr = start + (param.offset & KGSL_PAGEMASK);
	if (param.memtype != KGSL_USER_MEM_TYPE_PMEM) {
		result = kgsl_mmu_map(private->pagetable,
				entry->memdesc.physaddr, entry->memdesc.size,
				GSL_PT_PAGE_RV | GSL_PT_PAGE_WV,
				&entry->memdesc.gpuaddr,
				KGSL_MEMFLAGS_ALIGN4K | KGSL_MEMFLAGS_HOSTADDR);
		entry->memdesc.priv = KGSL_MEMFLAGS_HOSTADDR;
	} else {
		result = kgsl_mmu_map(private->pagetable,
				entry->memdesc.physaddr, entry->memdesc.size,
				GSL_PT_PAGE_RV | GSL_PT_PAGE_WV,
				&entry->memdesc.gpuaddr,
				KGSL_MEMFLAGS_ALIGN4K | KGSL_MEMFLAGS_CONPHYS);
	}
	if (result)
		goto error_free_entry;

	/* If the offset is not at 4K boundary then add the correct offset
	 * value to gpuaddr */
	total_offset = entry->memdesc.gpuaddr + (param.offset & ~KGSL_PAGEMASK);
	if (total_offset > (uint64_t)UINT_MAX) {
		result = -EINVAL;
		goto error_unmap_entry;
	}
	entry->priv = private;
	entry->memdesc.gpuaddr = total_offset;
	param.gpuaddr = entry->memdesc.gpuaddr;

	if (IOCTL_KGSL_SHAREDMEM_FROM_PMEM == cmd) {
		if (copy_to_user(arg, &param,
			sizeof(struct kgsl_sharedmem_from_pmem))) {
			result = -EFAULT;
			goto error_unmap_entry;
		}
	} else if (copy_to_user(arg, &param, sizeof(param))) {
		result = -EFAULT;
		goto error_unmap_entry;
	}
	spin_lock(&private->mem_lock);
	list_add(&entry->list, &private->mem_list);
	spin_unlock(&private->mem_lock);
	return result;

error_unmap_entry:
	kgsl_mmu_unmap(entry->memdesc.pagetable,
			entry->memdesc.gpuaddr & KGSL_PAGEMASK,
			entry->memdesc.size);
error_free_entry:
	kfree(entry);

error_put_file_ptr:
	if ((param.memtype != KGSL_USER_MEM_TYPE_PMEM) && file_ptr)
		put_ashmem_file(file_ptr);
	else
		kgsl_put_phys_file(file_ptr);

error:
	return result;
}

#ifdef CONFIG_MSM_KGSL_MMU
/*This function flushes a graphics memory allocation from CPU cache
 *when caching is enabled with MMU*/
static long
kgsl_ioctl_sharedmem_flush_cache(struct kgsl_process_private *private,
				 void __user *arg)
{
	int result = 0;
	struct kgsl_mem_entry *entry;
	struct kgsl_sharedmem_free param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	spin_lock(&private->mem_lock);
	entry = kgsl_sharedmem_find(private, param.gpuaddr);
	if (!entry) {
		KGSL_DRV_ERR("invalid gpuaddr %08x\n", param.gpuaddr);
		result = -EINVAL;
	} else {
		if (!entry->memdesc.hostptr)
			entry->memdesc.hostptr =
				kgsl_gpuaddr_to_vaddr(&entry->memdesc,
					param.gpuaddr, &entry->memdesc.size);

		if (!entry->memdesc.hostptr) {
			KGSL_DRV_ERR("invalid hostptr with gpuaddr %08x\n",
								param.gpuaddr);
			goto done;
		}

		kgsl_cache_range_op((unsigned long)entry->memdesc.hostptr,
				    entry->memdesc.size,
				    KGSL_MEMFLAGS_CACHE_CLEAN |
				    KGSL_MEMFLAGS_HOSTADDR);
		/* Mark memory as being flushed so we don't flush it again */
		entry->memdesc.priv &= ~KGSL_MEMFLAGS_CACHE_MASK;
	}
	spin_unlock(&private->mem_lock);
done:
	return result;
}
#endif /*CONFIG_MSM_KGSL_MMU*/

static long
kgsl_memory_ioctl(struct kgsl_device_private *dev_priv,
		  unsigned int cmd, unsigned long arg)
{
	struct kgsl_device *device = dev_priv->device;
	int result;

	switch (cmd) {
#ifdef CONFIG_MSM_KGSL_MMU
	case IOCTL_KGSL_SHAREDMEM_FROM_VMALLOC:
		kgsl_runpending_unlocked(device);

		result = kgsl_ioctl_sharedmem_from_vmalloc(
			dev_priv->process_priv,
			(void __user *)arg);

		/* The runpending probably woke up the hardware - try to
		   lull it back to sleep */

		kgsl_check_idle(device);
		break;

	case IOCTL_KGSL_SHAREDMEM_FLUSH_CACHE:
		result = kgsl_ioctl_sharedmem_flush_cache(
			dev_priv->process_priv,
			(void __user *)arg);
		break;
#endif
	case IOCTL_KGSL_SHAREDMEM_FROM_PMEM:
	case IOCTL_KGSL_MAP_USER_MEM:
		kgsl_runpending_unlocked(device);
		result = kgsl_ioctl_map_user_mem(dev_priv->process_priv,
						 (void __user *)arg,
						 cmd);

		/* The runpending probably woke up the hardware - try to
		   lull it back to sleep */

		kgsl_check_idle(device);
		break;

	case IOCTL_KGSL_SHAREDMEM_FREE:
		result = kgsl_ioctl_sharedmem_free(dev_priv->process_priv,
						   (void __user *)arg);
		break;

	default:
		result = -EINVAL;
		break;
	}

	return result;
}

static long kgsl_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	int result = 0;
	struct kgsl_device_private *dev_priv = filep->private_data;
	struct inode *inodep = filep->f_path.dentry->d_inode;
	struct kgsl_device *device;

	BUG_ON(dev_priv == NULL);
	device = kgsl_driver.devp[iminor(inodep)];
	BUG_ON(device == NULL);
	BUG_ON(device != dev_priv->device);

	KGSL_DRV_VDBG("filep %p cmd 0x%08x arg 0x%08lx\n", filep, cmd, arg);

	/* Memory related functions don't always rely on locking hardware,
	   so it is cleanest to handle them seperately without a lot of
	   nasty if statements in this function */

	if (cmd == IOCTL_KGSL_SHAREDMEM_FROM_VMALLOC ||
	    cmd == IOCTL_KGSL_SHAREDMEM_FROM_PMEM ||
	    cmd == IOCTL_KGSL_MAP_USER_MEM ||
	    cmd == IOCTL_KGSL_SHAREDMEM_FLUSH_CACHE ||
	    cmd == IOCTL_KGSL_SHAREDMEM_FREE)
		return kgsl_memory_ioctl(dev_priv, cmd, arg);

	mutex_lock(&device->mutex);
	kgsl_check_suspended(device);
	device->active_cnt++;

	switch (cmd) {

	case IOCTL_KGSL_DEVICE_GETPROPERTY:
		result =
		    kgsl_ioctl_device_getproperty(dev_priv, (void __user *)arg);
		break;

	case IOCTL_KGSL_DEVICE_REGREAD:
		result =
		    kgsl_ioctl_device_regread(dev_priv, (void __user *)arg);
		break;

	case IOCTL_KGSL_DEVICE_WAITTIMESTAMP:
		result = kgsl_ioctl_device_waittimestamp(dev_priv,
							(void __user *)arg);
		/* order reads to the buffer written to by the GPU */
		rmb();
		break;

	case IOCTL_KGSL_RINGBUFFER_ISSUEIBCMDS:
#ifdef CONFIG_MSM_KGSL_MMU
		if (kgsl_cache_enable)
			kgsl_clean_cache_all(dev_priv->process_priv);
#endif
#ifdef CONFIG_MSM_KGSL_DRM
		kgsl_gpu_mem_flush(DRM_KGSL_GEM_CACHE_OP_TO_DEV);
#endif
		result =
		    kgsl_ioctl_rb_issueibcmds(dev_priv, (void __user *)arg);
#ifdef CONFIG_MSM_KGSL_DRM
		kgsl_gpu_mem_flush(DRM_KGSL_GEM_CACHE_OP_FROM_DEV);
#endif
		break;

	case IOCTL_KGSL_CMDSTREAM_READTIMESTAMP:
		result =
		    kgsl_ioctl_cmdstream_readtimestamp(dev_priv,
							(void __user *)arg);
		break;

	case IOCTL_KGSL_CMDSTREAM_FREEMEMONTIMESTAMP:
		result =
		    kgsl_ioctl_cmdstream_freememontimestamp(dev_priv,
						    (void __user *)arg);
		break;

	case IOCTL_KGSL_DRAWCTXT_CREATE:
		result = kgsl_ioctl_drawctxt_create(dev_priv,
							(void __user *)arg);
		break;

	case IOCTL_KGSL_DRAWCTXT_DESTROY:
		result =
		    kgsl_ioctl_drawctxt_destroy(dev_priv, (void __user *)arg);
		break;

	default:
		/* call into device specific ioctls */
		result = device->ftbl.device_ioctl(dev_priv, cmd, arg);
		break;
	}
	INIT_COMPLETION(device->suspend_gate);
	device->active_cnt--;
	if (device->active_cnt == 0)
		complete(&device->suspend_gate);

	kgsl_check_idle_locked(device);

	mutex_unlock(&device->mutex);
	KGSL_DRV_VDBG("result %d\n", result);
	return result;
}

static int kgsl_mmap(struct file *file, struct vm_area_struct *vma)
{
	int result = 0;
	struct kgsl_memdesc *memdesc = NULL;
	unsigned long vma_size = vma->vm_end - vma->vm_start;
	unsigned long vma_offset = vma->vm_pgoff << PAGE_SHIFT;
	struct inode *inodep = file->f_path.dentry->d_inode;
	struct kgsl_device *device;

	device = kgsl_driver.devp[iminor(inodep)];
	BUG_ON(device == NULL);

	mutex_lock(&device->mutex);

	/*allow device memstore to be mapped read only */
	if (vma_offset == device->memstore.physaddr) {
		if (vma->vm_flags & VM_WRITE) {
			result = -EPERM;
			goto done;
		}
		memdesc = &device->memstore;
	} else {
		result = -EINVAL;
		goto done;
	}

	if (memdesc->size != vma_size) {
		KGSL_MEM_ERR("file %p bad size %ld, should be %d\n",
			file, vma_size, memdesc->size);
		result = -EINVAL;
		goto done;
	}
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	result = remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
				vma_size, vma->vm_page_prot);
	if (result != 0) {
		KGSL_MEM_ERR("remap_pfn_range returned %d\n",
				result);
		goto done;
	}
done:
	mutex_unlock(&device->mutex);
	return result;
}

static int kgsl_pm_suspend(struct device *dev)
{
	pm_message_t arg = {0};
	dev_dbg(dev, "pm: suspending...\n");
	kgsl_suspend(NULL, arg);
	return 0;
}

static int kgsl_pm_resume(struct device *dev)
{
	dev_dbg(dev, "pm: resuming...\n");
	kgsl_resume(NULL);
	return 0;
}

static int kgsl_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: suspending...\n");
	return 0;
}

static int kgsl_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: resuming...\n");
	return 0;
}

static struct dev_pm_ops kgsl_dev_pm_ops = {
	.suspend = kgsl_pm_suspend,
	.resume = kgsl_pm_resume,
	.runtime_suspend = kgsl_runtime_suspend,
	.runtime_resume = kgsl_runtime_resume,
};

static const struct file_operations kgsl_fops = {
	.owner = THIS_MODULE,
	.release = kgsl_release,
	.open = kgsl_open,
	.mmap = kgsl_mmap,
	.unlocked_ioctl = kgsl_ioctl,
};

struct kgsl_driver kgsl_driver  = {
	.process_mutex = __MUTEX_INITIALIZER(kgsl_driver.process_mutex),
	.pt_mutex = __MUTEX_INITIALIZER(kgsl_driver.pt_mutex),
};

static void kgsl_device_unregister(void)
{
	int j;
	struct kgsl_device *device;

	for (j = 0; j < kgsl_driver.num_devs; j++) {
		device_destroy(kgsl_driver.class,
					MKDEV(MAJOR(kgsl_driver.dev_num), j));
		device =  kgsl_driver.devp[j];
		kgsl_pwrctrl_uninit_sysfs(device);
	}

	class_destroy(kgsl_driver.class);
	cdev_del(&kgsl_driver.cdev);
	unregister_chrdev_region(kgsl_driver.dev_num, kgsl_driver.num_devs);
}


static void kgsl_driver_cleanup(void)
{
	if (kgsl_driver.global_pt) {
		kgsl_mmu_putpagetable(kgsl_driver.global_pt);
		kgsl_driver.global_pt = NULL;
	}

	kgsl_yamato_close(kgsl_get_yamato_generic_device());

	kgsl_g12_close(kgsl_get_2d_device(KGSL_DEVICE_2D0));
	kgsl_g12_close(kgsl_get_2d_device(KGSL_DEVICE_2D1));

	kgsl_ptpool_destroy(&kgsl_driver.ptpool);

	kgsl_driver.pdev = NULL;
}

static int kgsl_add_device(int dev_idx)
{
	struct kgsl_device *device;
	int err = 0;

	switch (dev_idx) {
	case (KGSL_DEVICE_YAMATO):
		device = kgsl_get_yamato_generic_device();
		kgsl_driver.devp[KGSL_DEVICE_YAMATO] = device;
	break;
	case (KGSL_DEVICE_2D0):
		device = kgsl_get_2d_device(KGSL_DEVICE_2D0);
		kgsl_driver.devp[KGSL_DEVICE_2D0] = device;
	break;
	case (KGSL_DEVICE_2D1):
		device = kgsl_get_2d_device(KGSL_DEVICE_2D1);
		kgsl_driver.devp[KGSL_DEVICE_2D1] = device;
	break;
	default:
		err = -EINVAL;
		goto done;
	break;
	}

	if (device == NULL) {
		err = -EINVAL;
		goto done;
	}

 done:
	return err;
}

static int kgsl_register_dev(int num_devs)
{
	int err;
	int j, i;
	dev_t dev;

	/* alloc major and minor device numbers */
	err = alloc_chrdev_region(&kgsl_driver.dev_num, 0, num_devs,
				DRIVER_NAME);
	if (err < 0) {
		KGSL_DRV_ERR("alloc_chrdev_region failed err = %d\n", err);
		return err;
	}

	/* create sysfs entries */
	kgsl_driver.class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(kgsl_driver.class)) {
		err = PTR_ERR(kgsl_driver.class);
		KGSL_DRV_ERR("failed to create class %s", CLASS_NAME);
		goto error_class_create;
	}

	for (i = 0; i < num_devs; i++) {
		struct kgsl_device *device = kgsl_driver.devp[i];

		dev = MKDEV(MAJOR(kgsl_driver.dev_num), i);
		device->dev = device_create(kgsl_driver.class,
					    &kgsl_driver.pdev->dev,
					    dev, NULL, device->name);
		if (IS_ERR(device->dev)) {
			err = PTR_ERR(device->dev);
			KGSL_DRV_ERR("device_create failed err=%d\n", err);
			for (j = 0; j < i; j++)
				device_destroy(kgsl_driver.class,
					MKDEV(MAJOR(kgsl_driver.dev_num), j));
			goto error_device_create;
		}

		kgsl_pwrctrl_init_sysfs(device);
	}

	/* add char dev(s) */
	cdev_init(&kgsl_driver.cdev, &kgsl_fops);
	kgsl_driver.cdev.owner = THIS_MODULE;
	kgsl_driver.cdev.ops = &kgsl_fops;
	err = cdev_add(&kgsl_driver.cdev, MKDEV(MAJOR(kgsl_driver.dev_num), 0),
			num_devs);
	if (err) {
		KGSL_DRV_ERR("kgsl: cdev_add() failed, dev_num= %d,"
				" result= %d\n", kgsl_driver.dev_num, err);
		goto error_cdev_add;
	}

	KGSL_DRV_DBG("register_dev successful, cdev.dev=0x%x\n",
						kgsl_driver.cdev.dev);
	return err;

error_cdev_add:
	for (j = 0; j < num_devs; j++) {
		device_destroy(kgsl_driver.class,
					MKDEV(MAJOR(kgsl_driver.dev_num), j));
	}
error_device_create:
	class_destroy(kgsl_driver.class);
error_class_create:
	unregister_chrdev_region(kgsl_driver.dev_num, num_devs);

	return err;
}

static int __devinit kgsl_platform_probe(struct platform_device *pdev)
{
	int i, result = 0;
	struct kgsl_platform_data *pdata = NULL;
	struct kgsl_device *device = kgsl_get_yamato_generic_device();
	struct kgsl_device *device_2d0 = kgsl_get_2d_device(KGSL_DEVICE_2D0);
	struct kgsl_device *device_2d1 = kgsl_get_2d_device(KGSL_DEVICE_2D1);

	kgsl_debug_init();
	kgsl_cffdump_init();

	for (i = 0; i < KGSL_DEVICE_MAX; i++)
		kgsl_driver.devp[i] = NULL;

	kgsl_driver.num_devs = 0;
	INIT_LIST_HEAD(&kgsl_driver.process_list);

	kgsl_driver.pdev = pdev;
	pdata = pdev->dev.platform_data;
	BUG_ON(pdata == NULL);

	kgsl_yamato_init_pwrctrl(device);

	if (pdata->grp2d0_clk_name != NULL) {
		/* 2d0 pwrctrl */
		result = kgsl_g12_init_pwrctrl(device_2d0);
		if (result != 0) {
			KGSL_DRV_ERR(
				"kgsl_2d0_init_pwrctrl returned error=%d\n",
				result);
			goto done;
		}
	}

	if (pdata->grp2d1_clk_name != NULL) {
		/* 2d1 pwrctrl */
		result = kgsl_g12_init_pwrctrl(device_2d1);
		if (result != 0) {
			KGSL_DRV_ERR(
				"kgsl_2d1_init_pwrctrl returned error=%d\n",
				result);
			goto done;
		}
	}

	kgsl_driver.ptsize =
		KGSL_PAGETABLE_ENTRIES(CONFIG_MSM_KGSL_PAGE_TABLE_SIZE) *
		KGSL_PAGETABLE_ENTRY_SIZE;
	kgsl_driver.ptsize = ALIGN(kgsl_driver.ptsize, KGSL_PAGESIZE);

	result = kgsl_ptpool_init(&kgsl_driver.ptpool, kgsl_driver.ptsize,
		KGSL_PAGETABLE_COUNT);

	result = kgsl_drm_init(pdev);

	INIT_LIST_HEAD(&kgsl_driver.pagetable_list);
	pm_runtime_enable(&pdev->dev);

	result = kgsl_yamato_init(device);

	if (result) {
		KGSL_DRV_ERR("yamato_init failed %d", result);
		goto done;
	}

	if (kgsl_add_device(KGSL_DEVICE_YAMATO) == KGSL_FAILURE) {
		result = -EINVAL;
		goto done;
	}
	kgsl_driver.num_devs++;

	if (pdata->grp2d0_clk_name) {
		result = kgsl_g12_init(device_2d0);
		if (result) {
			KGSL_DRV_ERR("2d0_init failed %d", result);
			goto done;
		}
		if (kgsl_add_device(KGSL_DEVICE_2D0) == KGSL_FAILURE) {
			result = -EINVAL;
			goto done;
		}
		kgsl_driver.num_devs++;
	}

	if (pdata->grp2d1_clk_name) {
		result = kgsl_g12_init(device_2d1);
		if (result) {
			KGSL_DRV_ERR("2d1_init failed %d", result);
			goto done;
		}
		if (kgsl_add_device(KGSL_DEVICE_2D1) == KGSL_FAILURE) {
			result = -EINVAL;
			goto done;
		}
		kgsl_driver.num_devs++;
	}

	device = kgsl_get_device(KGSL_DEVICE_YAMATO);
	kgsl_driver.global_pt = kgsl_mmu_getpagetable(&device->mmu,
						      KGSL_MMU_GLOBAL_PT);
	if (kgsl_driver.global_pt == NULL) {
		result = -ENOMEM;
		goto done;
	}
done:
	if (result)
		kgsl_driver_cleanup();
	else
		result = kgsl_register_dev(kgsl_driver.num_devs);

	if (!result)
		KGSL_DRV_DBG("platform probe successful, numdevs=%d\n",
						kgsl_driver.num_devs);
	return result;
}

static int kgsl_platform_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);

	kgsl_driver_cleanup();
	kgsl_drm_exit();
	kgsl_device_unregister();
	kgsl_cffdump_destroy();

	return 0;
}

static struct platform_driver kgsl_platform_driver = {
	.probe = kgsl_platform_probe,
	.remove = __devexit_p(kgsl_platform_remove),
	.suspend = kgsl_suspend,
	.resume = kgsl_resume,
	.driver = {
		.owner = THIS_MODULE,
		.name = DRIVER_NAME,
		.pm = &kgsl_dev_pm_ops,
	}
};

static int __init kgsl_mod_init(void)
{
	return platform_driver_register(&kgsl_platform_driver);
}

static void __exit kgsl_mod_exit(void)
{
	platform_driver_unregister(&kgsl_platform_driver);
}

#ifdef MODULE
module_init(kgsl_mod_init);
#else
late_initcall(kgsl_mod_init);
#endif
module_exit(kgsl_mod_exit);

MODULE_DESCRIPTION("Graphics driver for QSD8x50, MSM7x27, and MSM7x30");
MODULE_VERSION("1.1");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:kgsl");
