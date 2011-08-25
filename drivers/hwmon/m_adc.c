/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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
#include <linux/m_adc.h>
#include <linux/pmic8058-xoadc.h>
#include <linux/slab.h>

#define MSM_ADC_DRIVER_NAME             "msm_adc"

struct msm_adc_drv {
	void				*dev_h;
	struct platform_device		*pdev;
	struct sensor_device_attribute	*sens_attr;
	struct workqueue_struct		*wq;
	struct device			*hwmon;
	struct miscdevice		misc;
	atomic_t			online;
	atomic_t			total_outst;
	wait_queue_head_t		total_outst_wait;
};

/* Needed to support file_op interfaces */
static struct msm_adc_drv *msm_adc_drv;

static int conv_first_request;

static ssize_t msm_adc_show_curr(struct device *dev,
				struct device_attribute *devattr, char *buf);

static int msm_adc_blocking_conversion(struct msm_adc_drv *msm_adc,
				uint32_t chan, struct adc_chan_result *result);

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
	struct adc_conv_slot *slot, *tmp;
	int rc;
	struct msm_adc_platform_data *pdata =
					msm_adc_drv->pdev->dev.platform_data;
	struct msm_adc_channels *channel = pdata->channel;

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
		slot->client = NULL;
		list_del(&slot->list);
		channel[slot->conv.result.chan].adc_access_fn->adc_restore_slot(
		channel[slot->conv.result.chan].adc_dev_instance, slot);
	}

	kfree(client);

	return 0;
}

static int msm_adc_lookup(struct msm_adc_drv *msm_adc,
			  struct msm_adc_lookup *lookup)
{
	struct platform_device *pdev = msm_adc->pdev;
	struct msm_adc_platform_data *pdata = msm_adc->pdev->dev.platform_data;
	int rc = 0, i = 0;

	do {
		if (strcmp(lookup->name, pdata->channel[i].name))
			i++;
		else
			break;
	} while (i < pdata->num_chan_supported);

	if (i == pdata->num_chan_supported)
		rc = -EINVAL;

	if (rc) {
		dev_err(&pdev->dev, "Lookup failed for %s\n", lookup->name);
		return rc;
	}

	lookup->chan_idx = i;
	return rc;
}

static int msm_adc_aio_conversion(struct msm_adc_drv *msm_adc,
				  struct adc_chan_result *request,
				  struct msm_client_data *client)
{
	struct msm_adc_platform_data *pdata =
					msm_adc_drv->pdev->dev.platform_data;
	struct msm_adc_channels *channel = &pdata->channel[request->chan];
	struct adc_conv_slot *slot;

	/* we could block here, but only for a bounded time */
	channel->adc_access_fn->adc_slot_request(channel->adc_dev_instance,
									&slot);

	if (slot) {
		atomic_inc(&msm_adc->total_outst);
		mutex_lock(&client->lock);
		client->num_outstanding++;
		mutex_unlock(&client->lock);

		/* indicates non blocking request to callback handler */
		slot->blocking = 0;
		slot->compk = NULL;/*For kernel space usage; n/a for usr space*/
		slot->conv.result.chan = client->adc_chan = request->chan;
		slot->client = client;
		slot->adc_request = START_OF_CONV;
		slot->chan_path = channel->chan_path_type;
		slot->chan_adc_config = channel->adc_config_type;
		slot->chan_adc_calib = channel->adc_calib_type;
		queue_work(msm_adc->wq, &slot->work);
		return 0;
	}
	return -EBUSY;
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
			       struct adc_chan_result *result)
{
	struct msm_adc_platform_data *pdata = msm_adc->pdev->dev.platform_data;
	struct msm_adc_channels *channel = pdata->channel;
	struct adc_conv_slot *slot;
	int rc = 0;

	mutex_lock(&client->lock);

	slot = list_first_entry(&client->complete_list,
				struct adc_conv_slot, list);
	if (!slot) {
		mutex_unlock(&client->lock);
		return -ENOMSG;
	}

	slot->client = NULL;
	list_del(&slot->list);

	client->num_complete--;

	mutex_unlock(&client->lock);

	*result = slot->conv.result;

	/* restore this slot to reserve */
	channel[slot->conv.result.chan].adc_access_fn->adc_restore_slot(
		channel[slot->conv.result.chan].adc_dev_instance, slot);

	return rc;
}

static long msm_adc_ioctl(struct file *file, unsigned int cmd,
					     unsigned long arg)
{
	struct msm_client_data *client = file->private_data;
	struct msm_adc_drv *msm_adc = msm_adc_drv;
	struct platform_device *pdev = msm_adc->pdev;
	int rc;

	switch (cmd) {
	case MSM_ADC_REQUEST:
		{
			struct adc_chan_result conv;

			if (copy_from_user(&conv, (void __user *)arg,
					sizeof(struct adc_chan_result)))
				return -EFAULT;

			rc = msm_adc_blocking_conversion(msm_adc, conv.chan,
							&conv);
			if (rc) {
				dev_dbg(&pdev->dev, "BLK conversion failed\n");
				return rc;
			}

			if (copy_to_user((void __user *)arg, &conv,
					sizeof(struct adc_chan_result)))
				return -EFAULT;
			break;
		}
	case MSM_ADC_AIO_REQUEST:
		{
			struct adc_chan_result conv;

			if (copy_from_user(&conv, (void __user *)arg,
					sizeof(struct adc_chan_result)))
				return -EFAULT;

			rc = msm_adc_aio_conversion(msm_adc, &conv, client);
			if (rc) {
				dev_dbg(&pdev->dev, "AIO conversion failed\n");
				return rc;
			}
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
			struct adc_chan_result result;

			rc = msm_adc_read_result(msm_adc, client, &result);
			if (rc) {
				dev_dbg(&pdev->dev, "read result failed\n");
				return rc;
			}

			if (copy_to_user((void __user *)arg, &result,
					sizeof(struct adc_chan_result)))
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
	struct adc_chan_result result;

	rc = pm8058_xoadc_registered();
	if (rc <= 0)
		return -ENODEV;

	rc = msm_adc_blocking_conversion(msm_adc, attr->index, &result);
	if (rc)
		return 0;

	return sprintf(buf, "Result: %lld Raw: %d\n", result.physical,
				result.adc_code);
}

static int msm_adc_blocking_conversion(struct msm_adc_drv *msm_adc,
			uint32_t hwmon_chan, struct adc_chan_result *result)
{
	struct adc_conv_slot *slot;
	struct msm_adc_platform_data *pdata =
					msm_adc_drv->pdev->dev.platform_data;
	struct msm_adc_channels *channel = &pdata->channel[hwmon_chan];
	int ret;

	if (!conv_first_request) {
		ret = pm8058_xoadc_calib_device(channel->adc_dev_instance);
		if (ret) {
			pr_err("pmic8058 xoadc calibration failed, retry\n");
			return ret;
		}
		conv_first_request = 1;
	}

	channel->adc_access_fn->adc_slot_request(channel->adc_dev_instance,
									&slot);
	if (slot) {
		slot->conv.result.chan = hwmon_chan;
		/* indicates blocking request to callback handler */
		slot->blocking = 1;
		slot->adc_request = START_OF_CONV;
		slot->chan_path = channel->chan_path_type;
		slot->chan_adc_config = channel->adc_config_type;
		slot->chan_adc_calib = channel->adc_calib_type;
		queue_work(msm_adc_drv->wq, &slot->work);

		wait_for_completion_interruptible(&slot->comp);
		*result = slot->conv.result;
		channel->adc_access_fn->adc_restore_slot(
					channel->adc_dev_instance, slot);
		return 0;
	}
	return -EBUSY;
}

void msm_adc_conv_cb(void *context, u32 param,
			    void *evt_buf, u32 len)
{
	struct adc_conv_slot *slot = context;
	struct msm_adc_drv *msm_adc = msm_adc_drv;

	switch (slot->adc_request) {
	case START_OF_CONV:
		slot->adc_request = END_OF_CONV;
	break;
	case START_OF_CALIBRATION:
		slot->adc_request = END_OF_CALIBRATION;
	break;
	case END_OF_CALIBRATION:
	case END_OF_CONV:
	break;
	}
	queue_work(msm_adc->wq, &slot->work);
}
EXPORT_SYMBOL(msm_adc_conv_cb);

static void msm_adc_teardown_device(struct platform_device *pdev,
				    struct msm_adc_drv *msm_adc)
{
	struct msm_adc_platform_data *pdata = pdev->dev.platform_data;
	int i, num_chans = pdata->num_chan_supported;

	if (msm_adc->sens_attr)
		for (i = 0; i < num_chans; i++)
			device_remove_file(&pdev->dev,
					&msm_adc->sens_attr[i].dev_attr);

	kfree(msm_adc->sens_attr);
}

static void msm_adc_teardown(struct platform_device *pdev)
{
	struct msm_adc_drv *msm_adc = platform_get_drvdata(pdev);

	if (!msm_adc)
		return;

	misc_deregister(&msm_adc->misc);

	if (msm_adc->hwmon)
		hwmon_device_unregister(msm_adc->hwmon);

	msm_adc_teardown_device(pdev, msm_adc);

	if (msm_adc->dev_h)
		msm_adc->dev_h = NULL;

	kfree(msm_adc);
	platform_set_drvdata(pdev, NULL);
}

/*
 * Process the deferred job
 */
void msm_adc_wq_work(struct work_struct *work)
{
	struct adc_properties *adc_properties;
	struct adc_conv_slot *slot = container_of(work,
						struct adc_conv_slot, work);
	uint32_t idx = slot->conv.result.chan;
	struct msm_adc_platform_data *pdata =
					msm_adc_drv->pdev->dev.platform_data;
	struct msm_adc_channels *channel = &pdata->channel[idx];
	int32_t adc_code;

	switch (slot->adc_request) {
	case START_OF_CONV:
			channel->adc_access_fn->adc_select_chan_and_start_conv(
					channel->adc_dev_instance, slot);
	break;
	case END_OF_CONV:
		adc_properties = channel->adc_access_fn->adc_get_properties(
						channel->adc_dev_instance);
		if (channel->adc_access_fn->adc_read_adc_code)
			channel->adc_access_fn->adc_read_adc_code(
					channel->adc_dev_instance, &adc_code);
		if (channel->chan_processor)
			channel->chan_processor(adc_code, adc_properties,
				&slot->chan_properties, &slot->conv.result);
		/* Intentionally a fall thru here.  Calibraton does not need
		to perform channel processing, etc.  However, both
		end of conversion and end of calibration requires the below
		fall thru code to be executed. */
	case END_OF_CALIBRATION:
		/* for blocking requests, signal complete */
		if (slot->blocking)
			complete(&slot->comp);
		else {
			struct msm_client_data *client = slot->client;

			mutex_lock(&client->lock);

			if (slot->adc_request == END_OF_CONV) {
				list_add(&slot->list, &client->complete_list);
				client->num_complete++;
			}
			client->num_outstanding--;

		/*
		 * if the client release has been invoked and this is call
		 * corresponds to the last request, then signal release
		 * to complete.
		 */
			if (slot->client->online == 0 &&
						client->num_outstanding == 0)
				wake_up_interruptible_all(&client->outst_wait);

			mutex_unlock(&client->lock);

			wake_up_interruptible_all(&client->data_wait);

			atomic_dec(&msm_adc_drv->total_outst);

			/* verify driver remove has not been invoked */
			if (atomic_read(&msm_adc_drv->online) == 0 &&
				atomic_read(&msm_adc_drv->total_outst) == 0)
				wake_up_interruptible_all(
					&msm_adc_drv->total_outst_wait);

			if (slot->compk) /* Kernel space request */
				complete(slot->compk);
			if (slot->adc_request == END_OF_CALIBRATION)
				channel->adc_access_fn->adc_restore_slot(
					channel->adc_dev_instance, slot);
		}
	break;
	case START_OF_CALIBRATION: /* code here to please code reviewers
					to satisfy silly compiler warnings */
	break;
	}
}
EXPORT_SYMBOL(msm_adc_wq_work);

static struct sensor_device_attribute msm_adc_curr_in_attr =
	SENSOR_ATTR(NULL, S_IRUGO, msm_adc_show_curr, NULL, 0);

static int __devinit msm_adc_init_hwmon(struct platform_device *pdev,
					       struct msm_adc_drv *msm_adc)
{
	struct msm_adc_platform_data *pdata = pdev->dev.platform_data;
	struct msm_adc_channels *channel = pdata->channel;
	int i, rc, num_chans = pdata->num_chan_supported;

	if (!channel)
		return -EINVAL;

	msm_adc->sens_attr = kzalloc(num_chans *
			    sizeof(struct sensor_device_attribute), GFP_KERNEL);
	if (!msm_adc->sens_attr) {
		dev_err(&pdev->dev, "Unable to allocate memory\n");
		rc = -ENOMEM;
		goto hwmon_err_sens;
	}

	for (i = 0; i < num_chans; i++) {
		msm_adc_curr_in_attr.index = i;
		msm_adc_curr_in_attr.dev_attr.attr.name = channel[i].name;
		memcpy(&msm_adc->sens_attr[i], &msm_adc_curr_in_attr,
						sizeof(msm_adc_curr_in_attr));

		rc = device_create_file(&pdev->dev,
				&msm_adc->sens_attr[i].dev_attr);
		if (rc) {
			dev_err(&pdev->dev, "device_create_file failed for "
					    "dal dev %s\n",
					    channel[i].name);
			goto hwmon_err_sens;
		}
	}

	return 0;

hwmon_err_sens:
	kfree(msm_adc->sens_attr);

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

	rc = msm_adc_init_hwmon(pdev, msm_adc);
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

	msm_adc->wq = create_singlethread_workqueue("msm_adc");
	if (!msm_adc->wq)
		goto err_cleanup;

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


int32_t adc_channel_open(uint32_t channel, void **h)
{
	struct msm_client_data *client;
	struct msm_adc_drv *msm_adc = msm_adc_drv;
	struct msm_adc_platform_data *pdata;
	struct platform_device *pdev;
	int i = 0, rc;

	if (!msm_adc_drv)
		return -EFAULT;

	rc = pm8058_xoadc_registered();

	if (rc <= 0)
		return -ENODEV;

	pdata = msm_adc->pdev->dev.platform_data;
	pdev = msm_adc->pdev;

	while (i < pdata->num_chan_supported) {
		if (channel == pdata->channel[i].channel_name)
			break;
		else
			i++;
	}

	if (i == pdata->num_chan_supported)
		return -EBADF; /* unknown channel */

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
	client->adc_chan = i;
	*h = (void *)client;
	return 0;
}

int32_t adc_channel_close(void *h)
{
	struct msm_client_data *client = (struct msm_client_data *)h;

	kfree(client);
	return 0;
}

int32_t adc_channel_request_conv(void *h, struct completion *conv_complete_evt)
{
	struct msm_client_data *client = (struct msm_client_data *)h;
	struct msm_adc_platform_data *pdata =
					msm_adc_drv->pdev->dev.platform_data;
	struct msm_adc_channels *channel = &pdata->channel[client->adc_chan];
	struct adc_conv_slot *slot;
	int ret;

	if (!conv_first_request) {
		ret = pm8058_xoadc_calib_device(channel->adc_dev_instance);
		if (ret) {
			pr_err("pm8058 xoadc calibration failed, retry\n");
			return ret;
		}
		conv_first_request = 1;
	}

	channel->adc_access_fn->adc_slot_request(channel->adc_dev_instance,
									&slot);

	if (slot) {
		atomic_inc(&msm_adc_drv->total_outst);
		mutex_lock(&client->lock);
		client->num_outstanding++;
		mutex_unlock(&client->lock);

		slot->conv.result.chan = client->adc_chan;
		slot->blocking = 0;
		slot->compk = conv_complete_evt;
		slot->client = client;
		slot->adc_request = START_OF_CONV;
		slot->chan_path = channel->chan_path_type;
		slot->chan_adc_config = channel->adc_config_type;
		slot->chan_adc_calib = channel->adc_calib_type;
		queue_work(msm_adc_drv->wq, &slot->work);
		return 0;
	}
	return -EBUSY;
}

int32_t adc_channel_read_result(void *h, struct adc_chan_result *chan_result)
{
	struct msm_client_data *client = (struct msm_client_data *)h;
	struct msm_adc_platform_data *pdata =
					msm_adc_drv->pdev->dev.platform_data;
	struct msm_adc_channels *channel = pdata->channel;
	struct adc_conv_slot *slot;
	int rc = 0;

	mutex_lock(&client->lock);

	slot = list_first_entry(&client->complete_list,
				struct adc_conv_slot, list);
	if (!slot) {
		mutex_unlock(&client->lock);
		return -ENOMSG;
	}

	slot->client = NULL;
	list_del(&slot->list);

	client->num_complete--;

	mutex_unlock(&client->lock);

	*chan_result = slot->conv.result;

	/* restore this slot to reserve */
	channel[slot->conv.result.chan].adc_access_fn->adc_restore_slot(
		channel[slot->conv.result.chan].adc_dev_instance, slot);

	return rc;
}

int32_t adc_calib_request(void *h, struct completion *calib_complete_evt)
{
	struct msm_client_data *client = (struct msm_client_data *)h;
	struct msm_adc_platform_data *pdata =
					msm_adc_drv->pdev->dev.platform_data;
	struct msm_adc_channels *channel = &pdata->channel[client->adc_chan];
	struct adc_conv_slot *slot;
	int rc, calib_status;

	channel->adc_access_fn->adc_slot_request(channel->adc_dev_instance,
									&slot);
	if (slot) {
		slot->conv.result.chan = client->adc_chan;
		slot->blocking = 0;
		slot->compk = calib_complete_evt;
		slot->adc_request = START_OF_CALIBRATION;
		slot->chan_path = channel->chan_path_type;
		slot->chan_adc_config = channel->adc_config_type;
		slot->chan_adc_calib = channel->adc_calib_type;
		rc = channel->adc_access_fn->adc_calibrate(
			channel->adc_dev_instance, slot, &calib_status);

		if (calib_status == CALIB_NOT_REQUIRED) {
			channel->adc_access_fn->adc_restore_slot(
					channel->adc_dev_instance, slot);
			/* client will always wait in case when
				calibration is not required */
			complete(calib_complete_evt);
		} else {
			atomic_inc(&msm_adc_drv->total_outst);
			mutex_lock(&client->lock);
			client->num_outstanding++;
			mutex_unlock(&client->lock);
		}

		return rc;
	}
	return -EBUSY;
}
