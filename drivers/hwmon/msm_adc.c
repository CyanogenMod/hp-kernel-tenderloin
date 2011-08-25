/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/msm_adc.h>
#include <linux/slab.h>

#include <mach/dal.h>

#define MSM_ADC_DRIVER_NAME		"msm_adc"
#define MSM_ADC_MAX_NUM_DEVS		3
#define MSM_ADC_MAX_FNAME		15

#define MSM_ADC_DALRPC_DEVICEID		0x02000067
#define MSM_ADC_DALRPC_PORT_NAME	"DAL00"
#define MSM_ADC_DALRPC_CPU		SMD_APPS_MODEM

#define MSM_ADC_DALRPC_CMD_REQ_CONV	9
#define MSM_ADC_DALRPC_CMD_INPUT_PROP	11

#define MSM_ADC_DALRC_CONV_TIMEOUT	(5 * HZ)  /* 5 seconds */

enum dal_error {
	DAL_ERROR_INVALID_DEVICE_IDX = 1,
	DAL_ERROR_INVALID_CHANNEL_IDX,
	DAL_ERROR_NULL_POINTER,
	DAL_ERROR_DEVICE_QUEUE_FULL,
	DAL_ERROR_INVALID_PROPERTY_LENGTH,
	DAL_ERROR_REMOTE_EVENT_POOL_FULL
};

enum dal_result_status {
	DAL_RESULT_STATUS_INVALID,
	DAL_RESULT_STATUS_VALID
};

struct msm_client_data {
	struct list_head		complete_list;
	bool				online;
	uint32_t			num_complete;
	uint32_t			num_outstanding;
	wait_queue_head_t		data_wait;
	wait_queue_head_t		outst_wait;
	struct mutex lock;
};

struct adc_dev_spec {
	uint32_t			hwmon_dev_idx;
	struct dal_dev_spec {
		uint32_t		dev_idx;
		uint32_t		chan_idx;
	} dal;
};

struct dal_conv_request {
	struct dal_dev_spec		target;
	void				*cb_h;
};

struct dal_adc_result {
	uint32_t			status;
	uint32_t			token;
	uint32_t			dev_idx;
	uint32_t			chan_idx;
	int				physical;
	uint32_t			percent;
	uint32_t			microvolts;
	uint32_t			reserved;
};

struct dal_conv_slot {
	void				*cb_h;
	struct dal_adc_result		result;
	struct completion		comp;
	struct list_head		list;
	uint32_t			idx;
	uint32_t			chan_idx;
	bool				blocking;
	struct msm_client_data		*client;
};

struct dal_conv_state {
	struct dal_conv_slot		context[MSM_ADC_DEV_MAX_INFLIGHT];
	struct list_head		slots;
	struct mutex			list_lock;
	struct semaphore		slot_count;
};

struct dal_translation {
	uint32_t			dal_dev_idx;
	uint32_t			hwmon_dev_idx;
	uint32_t			hwmon_start;
	uint32_t			hwmon_end;
};

struct adc_dev {
	char				*name;
	uint32_t			nchans;
	struct dal_conv_state		conv;
	struct dal_translation		transl;
	struct sensor_device_attribute	*sens_attr;
	char				**fnames;
};

struct msm_adc_drv {
	void				*dev_h;
	struct platform_device		*pdev;
	struct adc_dev			*devs[MSM_ADC_MAX_NUM_DEVS];
	struct device			*hwmon;
	struct miscdevice		misc;
	struct mutex			prop_lock;
	atomic_t			online;
	atomic_t			total_outst;
	wait_queue_head_t		total_outst_wait;
};

/* Needed to support file_op interfaces */
static struct msm_adc_drv *msm_adc_drv;

static ssize_t msm_adc_show_curr(struct device *dev,
				struct device_attribute *devattr, char *buf);

static int msm_adc_blocking_conversion(struct msm_adc_drv *msm_adc,
					  uint32_t chan, uint32_t *result);

static int msm_adc_open(struct inode *inode, struct file *file)
{
	struct msm_client_data *client;
	struct msm_adc_drv *msm_adc = msm_adc_drv;
	struct platform_device *pdev = msm_adc->pdev;

	client = kzalloc(sizeof(struct msm_client_data), GFP_KERNEL);
	if (!client) {
		dev_err(&pdev->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (!try_module_get(THIS_MODULE)) {
		kfree(client);
		return -EACCES;
	}

	mutex_init(&client->lock);
	INIT_LIST_HEAD(&client->complete_list);
	init_waitqueue_head(&client->data_wait);
	init_waitqueue_head(&client->outst_wait);

	client->online = 1;

	file->private_data = client;

	return nonseekable_open(inode, file);
}

static inline void msm_adc_restore_slot(struct dal_conv_state *conv_s,
					struct dal_conv_slot *slot)
{
	mutex_lock(&conv_s->list_lock);
	list_add(&slot->list, &conv_s->slots);
	mutex_unlock(&conv_s->list_lock);

	up(&conv_s->slot_count);
}

static int no_pending_client_requests(struct msm_client_data *client)
{
	mutex_lock(&client->lock);

	if (client->num_outstanding == 0) {
		mutex_unlock(&client->lock);
		return 1;
	}

	mutex_unlock(&client->lock);

	return 0;
}

static int data_avail(struct msm_client_data *client, uint32_t *pending)
{
	uint32_t completed;

	mutex_lock(&client->lock);
	completed = client->num_complete;
	mutex_unlock(&client->lock);

	if (completed > 0) {
		if (pending != NULL)
			*pending = completed;
		return 1;
	}

	return 0;
}

static int msm_adc_release(struct inode *inode, struct file *file)
{
	struct msm_client_data *client = file->private_data;
	struct dal_conv_slot *slot, *tmp;
	struct dal_conv_state *conv_s;
	int rc;

	module_put(THIS_MODULE);

	mutex_lock(&client->lock);

	/* prevent any further requests while we teardown the client */
	client->online = 0;

	mutex_unlock(&client->lock);

	/*
	 * We may still have outstanding transactions in flight from this
	 * client that have not completed. Make sure they're completed
	 * before removing the client.
	 */
	rc = wait_event_interruptible(client->outst_wait,
				      no_pending_client_requests(client));
	if (rc) {
		pr_err("%s: wait_event_interruptible failed rc = %d\n",
								__func__, rc);
		return rc;
	}

	/*
	 * All transactions have completed. Add slot resources back to the
	 * appropriate devices.
	 */
	list_for_each_entry_safe(slot, tmp, &client->complete_list, list) {
		conv_s = container_of(slot, struct dal_conv_state,
					      context[slot->idx]);
		slot->client = NULL;
		list_del(&slot->list);
		msm_adc_restore_slot(conv_s, slot);
	}

	kfree(client);

	return 0;
}

static int msm_adc_translate_dal_to_hwmon(struct msm_adc_drv *msm_adc,
					  uint32_t chan,
					  struct adc_dev_spec *dest)
{
	struct dal_translation *transl;
	struct msm_adc_platform_data *pdata = msm_adc->pdev->dev.platform_data;
	int i;

	for (i = 0; i < pdata->num_adc; i++) {
		transl = &msm_adc->devs[i]->transl;
		if (chan >= transl->hwmon_start &&
		    chan <= transl->hwmon_end) {
			dest->dal.dev_idx = transl->dal_dev_idx;
			dest->hwmon_dev_idx = transl->hwmon_dev_idx;
			dest->dal.chan_idx = chan - transl->hwmon_start;
			return 0;
		}
	}
	return -EINVAL;
}

static int msm_adc_translate_hwmon_to_dal(struct msm_adc_drv *msm_adc,
					  struct adc_dev_spec *source,
					  uint32_t *chan)
{
	struct msm_adc_platform_data *pdata = msm_adc->pdev->dev.platform_data;
	struct dal_translation *transl;
	int i;

	for (i = 0; i < pdata->num_adc; i++) {
		transl = &msm_adc->devs[i]->transl;
		if (source->dal.dev_idx != transl->dal_dev_idx)
			continue;
		*chan = transl->hwmon_start + source->dal.chan_idx;
		return 0;
	}
	return -EINVAL;
}

static int msm_adc_getinputproperties(struct msm_adc_drv *msm_adc,
					  const char *lookup_name,
					  struct adc_dev_spec *result)
{
	struct device *dev = &msm_adc->pdev->dev;
	int rc;

	mutex_lock(&msm_adc->prop_lock);

	rc = dalrpc_fcn_8(MSM_ADC_DALRPC_CMD_INPUT_PROP, msm_adc->dev_h,
			  lookup_name, strlen(lookup_name) + 1,
			  &result->dal, sizeof(struct dal_dev_spec));
	if (rc) {
		dev_err(dev, "DAL getprop request failed: rc = %d\n", rc);
		mutex_unlock(&msm_adc->prop_lock);
		return -EIO;
	}

	mutex_unlock(&msm_adc->prop_lock);
	return rc;
}

static int msm_adc_lookup(struct msm_adc_drv *msm_adc,
			  struct msm_adc_lookup *lookup)
{
	struct platform_device *pdev = msm_adc->pdev;
	struct adc_dev_spec target;
	int rc;

	rc = msm_adc_getinputproperties(msm_adc, lookup->name, &target);
	if (rc) {
		dev_err(&pdev->dev, "Lookup failed for %s\n", lookup->name);
		return rc;
	}

	rc = msm_adc_translate_hwmon_to_dal(msm_adc, &target,
						&lookup->chan_idx);
	if (rc)
		dev_err(&pdev->dev, "Translation failed for %s\n",
						lookup->name);
	return rc;
}

static int msm_adc_aio_conversion(struct msm_adc_drv *msm_adc,
				  struct msm_adc_conversion *request,
				  struct msm_client_data *client,
				  bool block_res)
{
	struct dal_conv_request params;
	struct device *dev = &msm_adc->pdev->dev;
	struct adc_dev *adc_dev;
	struct dal_conv_state *conv_s;
	struct dal_conv_slot *slot;
	struct adc_dev_spec dest;
	int rc;

	rc = msm_adc_translate_dal_to_hwmon(msm_adc, request->chan, &dest);
	if (rc) {
		dev_err(dev, "%s: translation from chan %u failed\n",
						__func__, request->chan);
		return -EINVAL;
	}

	atomic_inc(&msm_adc->total_outst);

	mutex_lock(&client->lock);

	adc_dev = msm_adc->devs[dest.hwmon_dev_idx];
	conv_s = &adc_dev->conv;

	if (down_trylock(&conv_s->slot_count) != 0) {
		if (block_res) {
			/* prevent thread from deadlocking itself */
			if (client->num_outstanding > 0) {
				mutex_unlock(&client->lock);
				rc = -EDEADLK;
				goto err_signal_remove;
			} else {
				client->num_outstanding++;
				mutex_unlock(&client->lock);
				down(&conv_s->slot_count);
			}
		} else {
			mutex_unlock(&client->lock);
			rc = -EWOULDBLOCK;
			goto err_signal_remove;
		}
	} else {
		client->num_outstanding++;
		mutex_unlock(&client->lock);
	}

	/* we could block here, but only for a bounded time */
	mutex_lock(&conv_s->list_lock);

	slot = list_first_entry(&conv_s->slots, struct dal_conv_slot, list);
	list_del(&slot->list);
	BUG_ON(!slot);

	mutex_unlock(&conv_s->list_lock);

	/* indicates non blocking request to callback handler */
	slot->blocking = 0;
	slot->chan_idx = request->chan;

	slot->client = client;

	params.target.dev_idx = dest.dal.dev_idx;
	params.target.chan_idx = dest.dal.chan_idx;
	params.cb_h = slot->cb_h;

	rc = dalrpc_fcn_8(MSM_ADC_DALRPC_CMD_REQ_CONV, msm_adc->dev_h,
			&params, sizeof(params), NULL, 0);
	if (rc) {
		dev_err(dev, "%s: Conversion for device = %u channel = %u"
				" failed\n", __func__, params.target.dev_idx,
						       params.target.chan_idx);
		rc = -EIO;
		goto aio_conv_cleanup_slot;
	}

	return 0;

aio_conv_cleanup_slot:
	msm_adc_restore_slot(conv_s, slot);
err_signal_remove:
	atomic_dec(&msm_adc->total_outst);

	return rc;
}

static int msm_adc_poll_complete(struct msm_adc_drv *msm_adc,
			     struct msm_client_data *client, uint32_t *pending)
{
	int rc;

	/*
	 * Don't proceed if there there's nothing queued on this client.
	 * We could deadlock otherwise in a single threaded scenario.
	 */
	if (no_pending_client_requests(client))
		return -EDEADLK;

	rc = wait_event_interruptible(client->data_wait,
				data_avail(client, pending));
	if (rc)
		return rc;

	return 0;
}

static int msm_adc_read_result(struct msm_adc_drv *msm_adc,
			       struct msm_client_data *client,
			       struct msm_adc_aio_result *result)
{
	struct dal_conv_slot *slot;
	struct dal_conv_state *conv_s;
	int rc = 0;

	mutex_lock(&client->lock);

	slot = list_first_entry(&client->complete_list,
				struct dal_conv_slot, list);
	if (!slot)
		return -ENOMSG;

	slot->client = NULL;
	list_del(&slot->list);

	client->num_complete--;

	mutex_unlock(&client->lock);

	result->chan = slot->chan_idx;
	result->result = slot->result.physical;

	if (slot->result.status == DAL_RESULT_STATUS_INVALID)
		rc = -ENODATA;

	conv_s = container_of(slot, struct dal_conv_state,
					      context[slot->idx]);

	/* restore this slot to reserve */
	msm_adc_restore_slot(conv_s, slot);

	return rc;
}

static long msm_adc_ioctl(struct file *file, unsigned int cmd,
					     unsigned long arg)
{
	struct msm_client_data *client = file->private_data;
	struct msm_adc_drv *msm_adc = msm_adc_drv;
	struct platform_device *pdev = msm_adc->pdev;
	uint32_t block_res = 0;
	int rc;

	switch (cmd) {
	case MSM_ADC_REQUEST:
		{
			struct msm_adc_conversion conv;

			if (copy_from_user(&conv, (void __user *)arg,
					sizeof(struct msm_adc_conversion)))
				return -EFAULT;

			rc = msm_adc_blocking_conversion(msm_adc, conv.chan,
							&conv.result);
			if (rc) {
				dev_dbg(&pdev->dev, "BLK conversion failed\n");
				return rc;
			}

			if (copy_to_user((void __user *)arg, &conv,
					sizeof(struct msm_adc_conversion)))
				return -EFAULT;
			break;
		}
	case MSM_ADC_AIO_REQUEST_BLOCK_RES:
		block_res = 1;
	case MSM_ADC_AIO_REQUEST:
		{
			struct msm_adc_conversion conv;

			if (copy_from_user(&conv, (void __user *)arg,
					sizeof(struct msm_adc_conversion)))
				return -EFAULT;

			rc = msm_adc_aio_conversion(msm_adc, &conv, client,
								block_res);
			if (rc) {
				dev_dbg(&pdev->dev, "AIO conversion failed\n");
				return rc;
			}
			if (copy_to_user((void __user *)arg, &conv,
					sizeof(struct msm_adc_conversion)))
				return -EFAULT;
			break;
		}
	case MSM_ADC_AIO_POLL:
		{
			uint32_t completed;

			rc = msm_adc_poll_complete(msm_adc, client, &completed);
			if (rc) {
				dev_dbg(&pdev->dev, "poll request failed\n");
				return rc;
			}

			if (copy_to_user((void __user *)arg, &completed,
					sizeof(uint32_t)))
				return -EFAULT;

			break;
		}
	case MSM_ADC_AIO_READ:
		{
			struct msm_adc_aio_result result;

			rc = msm_adc_read_result(msm_adc, client, &result);
			if (rc) {
				dev_dbg(&pdev->dev, "read result failed\n");
				return rc;
			}

			if (copy_to_user((void __user *)arg, &result,
					sizeof(struct msm_adc_aio_result)))
				return -EFAULT;
			break;
		}
	case MSM_ADC_LOOKUP:
		{
			struct msm_adc_lookup lookup;

			if (copy_from_user(&lookup, (void __user *)arg,
					sizeof(struct msm_adc_lookup)))
				return -EFAULT;

			rc = msm_adc_lookup(msm_adc, &lookup);
			if (rc) {
				dev_dbg(&pdev->dev, "No such channel: %s\n",
						lookup.name);
				return rc;
			}

			if (copy_to_user((void __user *)arg, &lookup,
					sizeof(struct msm_adc_lookup)))
				return -EFAULT;
			break;
		}
	default:
		return -EINVAL;
	}

	return 0;
}

const struct file_operations msm_adc_fops = {
	.open = msm_adc_open,
	.release = msm_adc_release,
	.unlocked_ioctl = msm_adc_ioctl,
};

static ssize_t msm_adc_show_curr(struct device *dev,
				 struct device_attribute *devattr, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct msm_adc_drv *msm_adc = dev_get_drvdata(dev);
	int rc;
	uint32_t result;

	rc = msm_adc_blocking_conversion(msm_adc, attr->index, &result);
	if (rc)
		return 0;

	return sprintf(buf, "%u\n", result);
}

static int msm_adc_blocking_conversion(struct msm_adc_drv *msm_adc,
					uint32_t hwmon_chan, uint32_t *result)
{
	struct dal_conv_request params;
	struct device *dev = &msm_adc->pdev->dev;
	struct adc_dev *adc_dev;
	struct dal_conv_state *conv_s;
	struct dal_conv_slot *slot;
	struct adc_dev_spec dest;
	int timeout, rc = 0;

	rc = msm_adc_translate_dal_to_hwmon(msm_adc, hwmon_chan, &dest);
	if (rc) {
		dev_err(dev, "%s: translation from chan %u failed\n",
							__func__, hwmon_chan);
		return -EINVAL;
	}

	adc_dev = msm_adc->devs[dest.hwmon_dev_idx];
	conv_s = &adc_dev->conv;

	down(&conv_s->slot_count);

	mutex_lock(&conv_s->list_lock);

	slot = list_first_entry(&conv_s->slots, struct dal_conv_slot, list);
	list_del(&slot->list);
	BUG_ON(!slot);

	mutex_unlock(&conv_s->list_lock);

	/* indicates blocking request to callback handler */
	slot->blocking = 1;

	params.target.dev_idx = dest.dal.dev_idx;
	params.target.chan_idx = dest.dal.chan_idx;
	params.cb_h = slot->cb_h;

	rc = dalrpc_fcn_8(MSM_ADC_DALRPC_CMD_REQ_CONV, msm_adc->dev_h,
			&params, sizeof(params), NULL, 0);
	if (rc) {
		dev_err(dev, "%s: Conversion for device = %u channel = %u"
			     " failed\n", __func__, params.target.dev_idx,
						    params.target.chan_idx);

		rc = -EIO;
		goto blk_conv_err;
	}

	timeout = wait_for_completion_interruptible_timeout(&slot->comp,
					      MSM_ADC_DALRC_CONV_TIMEOUT);
	if (timeout == 0) {
		dev_err(dev, "read for device = %u channel = %u timed out\n",
				params.target.dev_idx, params.target.chan_idx);
		rc = -ETIMEDOUT;
		goto blk_conv_err;
	} else if (timeout < 0) {
		rc = -EINTR;
		goto blk_conv_err;
	}

	*result = (uint32_t)slot->result.physical;

	if (slot->result.status == DAL_RESULT_STATUS_INVALID)
		rc = -ENODATA;

blk_conv_err:
	msm_adc_restore_slot(conv_s, slot);

	return rc;
}

static void msm_adc_conv_cb(void *context, u32 param,
			    void *evt_buf, u32 len)
{
	struct dal_adc_result *result = evt_buf;
	struct dal_conv_slot *slot = context;
	struct msm_adc_drv *msm_adc = msm_adc_drv;

	memcpy(&slot->result, result, sizeof(slot->result));

	/* for blocking requests, signal complete */
	if (slot->blocking)
		complete(&slot->comp);

	/* for non-blocking requests, add slot to the client completed list */
	else {
		struct msm_client_data *client = slot->client;

		mutex_lock(&client->lock);

		list_add(&slot->list, &client->complete_list);
		client->num_complete++;
		client->num_outstanding--;

		/*
		 * if the client release has been invoked and this is call
		 * corresponds to the last request, then signal release
		 * to complete.
		 */
		if (slot->client->online == 0 && client->num_outstanding == 0)
			wake_up_interruptible_all(&client->outst_wait);

		mutex_unlock(&client->lock);

		wake_up_interruptible_all(&client->data_wait);

		atomic_dec(&msm_adc->total_outst);

		/* verify driver remove has not been invoked */
		if (atomic_read(&msm_adc->online) == 0 &&
				atomic_read(&msm_adc->total_outst) == 0)
			wake_up_interruptible_all(&msm_adc->total_outst_wait);
	}
}

static void msm_adc_teardown_device_conv(struct platform_device *pdev,
				    struct adc_dev *adc_dev)
{
	struct dal_conv_state *conv_s = &adc_dev->conv;
	struct msm_adc_drv *msm_adc = platform_get_drvdata(pdev);
	struct dal_conv_slot *slot;
	int i;

	for (i = 0; i < MSM_ADC_DEV_MAX_INFLIGHT; i++) {
		slot = &conv_s->context[i];
		if (slot->cb_h) {
			dalrpc_dealloc_cb(msm_adc->dev_h, slot->cb_h);
			slot->cb_h = NULL;
		}
	}
}

static void msm_adc_teardown_device(struct platform_device *pdev,
				    struct adc_dev *adc_dev)
{
	struct dal_translation *transl = &adc_dev->transl;
	int i, num_chans = transl->hwmon_end - transl->hwmon_start + 1;

	if (adc_dev->sens_attr)
		for (i = 0; i < num_chans; i++)
			device_remove_file(&pdev->dev,
					&adc_dev->sens_attr[i].dev_attr);

	msm_adc_teardown_device_conv(pdev, adc_dev);

	kfree(adc_dev->fnames);
	kfree(adc_dev->sens_attr);
	kfree(adc_dev);
}

static void msm_adc_teardown_devices(struct platform_device *pdev)
{
	struct msm_adc_platform_data *pdata = pdev->dev.platform_data;
	struct msm_adc_drv *msm_adc = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < pdata->num_adc; i++) {
		if (msm_adc->devs[i]) {
			msm_adc_teardown_device(pdev, msm_adc->devs[i]);
			msm_adc->devs[i] = NULL;
		} else
			break;
	}
}

static void msm_adc_teardown(struct platform_device *pdev)
{
	struct msm_adc_drv *msm_adc = platform_get_drvdata(pdev);
	int rc;

	if (!msm_adc)
		return;

	if (msm_adc->hwmon)
		hwmon_device_unregister(msm_adc->hwmon);

	msm_adc_teardown_devices(pdev);

	if (msm_adc->dev_h) {
		rc = daldevice_detach(msm_adc->dev_h);
		if (rc)
			dev_err(&pdev->dev, "Cannot detach from dal device\n");
		msm_adc->dev_h = NULL;
	}

	kfree(msm_adc);
	platform_set_drvdata(pdev, NULL);
}

static int __devinit msm_adc_device_conv_init(struct msm_adc_drv *msm_adc,
					      struct adc_dev *adc_dev)
{
	struct platform_device *pdev = msm_adc->pdev;
	struct dal_conv_state *conv_s = &adc_dev->conv;
	struct dal_conv_slot *slot = conv_s->context;
	int rc, i;

	sema_init(&conv_s->slot_count, MSM_ADC_DEV_MAX_INFLIGHT);
	mutex_init(&conv_s->list_lock);
	INIT_LIST_HEAD(&conv_s->slots);

	for (i = 0; i < MSM_ADC_DEV_MAX_INFLIGHT; i++) {
		list_add(&slot->list, &conv_s->slots);
		slot->cb_h = dalrpc_alloc_cb(msm_adc->dev_h,
					     msm_adc_conv_cb, slot);
		if (!slot->cb_h) {
			dev_err(&pdev->dev, "Unable to allocate DAL callback"
							" for slot %d\n", i);
			rc = -ENOMEM;
			goto dal_err_cb;
		}
		init_completion(&slot->comp);
		slot->idx = i;
		slot++;
	}

	return 0;

dal_err_cb:
	msm_adc_teardown_device_conv(pdev, adc_dev);

	return rc;
}

static struct sensor_device_attribute msm_adc_curr_in_attr =
	SENSOR_ATTR(NULL, S_IRUGO, msm_adc_show_curr, NULL, 0);

static int __devinit msm_adc_device_init_hwmon(struct platform_device *pdev,
					       struct adc_dev *adc_dev)
{
	struct dal_translation *transl = &adc_dev->transl;
	int i, rc, num_chans = transl->hwmon_end - transl->hwmon_start + 1;
	const char prefix[] = "curr", postfix[] = "_input";
	char tmpbuf[5];

	adc_dev->fnames = kzalloc(num_chans * MSM_ADC_MAX_FNAME +
				  num_chans * sizeof(char *), GFP_KERNEL);
	if (!adc_dev->fnames) {
		dev_err(&pdev->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	adc_dev->sens_attr = kzalloc(num_chans *
			    sizeof(struct sensor_device_attribute), GFP_KERNEL);
	if (!adc_dev->sens_attr) {
		dev_err(&pdev->dev, "Unable to allocate memory\n");
		rc = -ENOMEM;
		goto hwmon_err_fnames;
	}

	for (i = 0; i < num_chans; i++) {
		adc_dev->fnames[i] = (char *)adc_dev->fnames +
			i * MSM_ADC_MAX_FNAME + num_chans * sizeof(char *);
		strcpy(adc_dev->fnames[i], prefix);
		sprintf(tmpbuf, "%d", transl->hwmon_start + i);
		strcat(adc_dev->fnames[i], tmpbuf);
		strcat(adc_dev->fnames[i], postfix);

		msm_adc_curr_in_attr.index = transl->hwmon_start + i;
		msm_adc_curr_in_attr.dev_attr.attr.name = adc_dev->fnames[i];
		memcpy(&adc_dev->sens_attr[i], &msm_adc_curr_in_attr,
						sizeof(msm_adc_curr_in_attr));

		rc = device_create_file(&pdev->dev,
				&adc_dev->sens_attr[i].dev_attr);
		if (rc) {
			dev_err(&pdev->dev, "device_create_file failed for "
					    "dal dev %u chan %d\n",
					    adc_dev->transl.dal_dev_idx, i);
			goto hwmon_err_sens;
		}
	}

	return 0;

hwmon_err_sens:
	kfree(adc_dev->sens_attr);
hwmon_err_fnames:
	kfree(adc_dev->fnames);

	return rc;
}

static int __devinit msm_adc_device_init(struct platform_device *pdev)
{
	struct msm_adc_platform_data *pdata = pdev->dev.platform_data;
	struct msm_adc_drv *msm_adc = platform_get_drvdata(pdev);
	struct adc_dev *adc_dev;
	struct adc_dev_spec target;
	int i, rc, hwmon_cntr = 1;

	if (pdata->num_adc == 0) {
		dev_err(&pdev->dev, "No adc devices specified in platform\n");
		return -EINVAL;
	}

	for (i = 0; i < pdata->num_adc; i++) {
		adc_dev = kzalloc(sizeof(struct adc_dev), GFP_KERNEL);
		if (!adc_dev) {
			dev_err(&pdev->dev, "Unable to allocate memory\n");
			rc = -ENOMEM;
			goto dev_init_err;
		}

		msm_adc->devs[i] = adc_dev;
		adc_dev->name = pdata->dev_names[i];

		rc = msm_adc_device_conv_init(msm_adc, adc_dev);
		if (rc) {
			dev_err(&pdev->dev, "DAL device[%s] failed conv init\n",
							adc_dev->name);
			goto dev_init_err;
		}

		/* DAL device lookup */
		rc = msm_adc_getinputproperties(msm_adc, adc_dev->name,
								&target);
		if (rc) {
			dev_err(&pdev->dev, "No such DAL device[%s]\n",
							adc_dev->name);
			goto dev_init_err;
		}

		adc_dev->transl.dal_dev_idx = target.dal.dev_idx;
		adc_dev->transl.hwmon_dev_idx = i;
		adc_dev->nchans = target.dal.chan_idx;
		adc_dev->transl.hwmon_start = hwmon_cntr;
		adc_dev->transl.hwmon_end = hwmon_cntr + adc_dev->nchans - 1;
		hwmon_cntr += adc_dev->nchans;

		rc = msm_adc_device_init_hwmon(pdev, adc_dev);
		if (rc)
			goto dev_init_err;
	}

	return 0;

dev_init_err:
	msm_adc_teardown_devices(pdev);
	return rc;
}

static int __devinit msm_adc_probe(struct platform_device *pdev)
{
	struct msm_adc_platform_data *pdata = pdev->dev.platform_data;
	struct msm_adc_drv *msm_adc;
	int rc = 0;

	if (!pdata) {
		dev_err(&pdev->dev, "no platform data?\n");
		return -EINVAL;
	}

	msm_adc = kzalloc(sizeof(struct msm_adc_drv), GFP_KERNEL);
	if (!msm_adc) {
		dev_err(&pdev->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, msm_adc);
	msm_adc_drv = msm_adc;
	msm_adc->pdev = pdev;

	mutex_init(&msm_adc->prop_lock);

	rc = daldevice_attach(MSM_ADC_DALRPC_DEVICEID,
			MSM_ADC_DALRPC_PORT_NAME,
			MSM_ADC_DALRPC_CPU,
			&msm_adc->dev_h);
	if (rc) {
		dev_err(&pdev->dev, "Cannot attach to dal device\n");
		kfree(msm_adc);
		return rc;
	}

	rc = msm_adc_device_init(pdev);
	if (rc) {
		dev_err(&pdev->dev, "msm_adc_dev_init failed\n");
		goto err_cleanup;
	}

	msm_adc->hwmon = hwmon_device_register(&pdev->dev);
	if (IS_ERR(msm_adc->hwmon)) {
		dev_err(&pdev->dev, "hwmon_device_register failed.\n");
		rc = PTR_ERR(msm_adc->hwmon);
		goto err_cleanup;
	}

	msm_adc->misc.name = MSM_ADC_DRIVER_NAME;
	msm_adc->misc.minor = MISC_DYNAMIC_MINOR;
	msm_adc->misc.fops = &msm_adc_fops;

	if (misc_register(&msm_adc->misc)) {
		dev_err(&pdev->dev, "Unable to register misc device!\n");
		goto err_cleanup;
	}

	init_waitqueue_head(&msm_adc->total_outst_wait);
	atomic_set(&msm_adc->online, 1);
	atomic_set(&msm_adc->total_outst, 0);

	pr_info("msm_adc successfully registered\n");

	return 0;

err_cleanup:
	msm_adc_teardown(pdev);

	return rc;
}

static int __devexit msm_adc_remove(struct platform_device *pdev)
{
	int rc;

	struct msm_adc_drv *msm_adc = platform_get_drvdata(pdev);

	atomic_set(&msm_adc->online, 0);

	misc_deregister(&msm_adc->misc);

	hwmon_device_unregister(msm_adc->hwmon);
	msm_adc->hwmon = NULL;

	/*
	 * We may still have outstanding transactions in flight that have not
	 * completed. Make sure they're completed before tearing down.
	 */
	rc = wait_event_interruptible(msm_adc->total_outst_wait,
				      atomic_read(&msm_adc->total_outst) == 0);
	if (rc) {
		pr_err("%s: wait_event_interruptible failed rc = %d\n",
								__func__, rc);
		return rc;
	}

	msm_adc_teardown(pdev);

	pr_info("msm_adc unregistered\n");

	return 0;
}

static struct platform_driver msm_adc_driver = {
	.probe = msm_adc_probe,
	.remove = __devexit_p(msm_adc_remove),
	.driver = {
		.name = MSM_ADC_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init msm_adc_init(void)
{
	return platform_driver_register(&msm_adc_driver);
}
module_init(msm_adc_init);

static void __exit msm_adc_exit(void)
{
	platform_driver_unregister(&msm_adc_driver);
}
module_exit(msm_adc_exit);

MODULE_DESCRIPTION("MSM ADC Driver");
MODULE_ALIAS("platform:msm_adc");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.1");
