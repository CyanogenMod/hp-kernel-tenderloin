/* drivers/usb/gadget/f_diag.c
 * Diag Function Device - Route ARM9 and ARM11 DIAG messages
 * between HOST and DEVICE.
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <mach/usbdiag.h>
#include <mach/rpc_hsusb.h>

#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include <linux/usb/android_composite.h>
#include <linux/workqueue.h>

static DEFINE_SPINLOCK(ch_lock);
static LIST_HEAD(usb_diag_ch_list);

static struct usb_interface_descriptor intf_desc = {
	.bLength            =	sizeof intf_desc,
	.bDescriptorType    =	USB_DT_INTERFACE,
	.bNumEndpoints      =	2,
	.bInterfaceClass    =	0xFF,
	.bInterfaceSubClass =	0xFF,
	.bInterfaceProtocol =	0xFF,
};

static struct usb_endpoint_descriptor hs_bulk_in_desc = {
	.bLength 			=	USB_DT_ENDPOINT_SIZE,
	.bDescriptorType 	=	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes 		=	USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize 	=	__constant_cpu_to_le16(512),
	.bInterval 			=	0,
};
static struct usb_endpoint_descriptor fs_bulk_in_desc = {
	.bLength          =	USB_DT_ENDPOINT_SIZE,
	.bDescriptorType  =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes     =	USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize   = __constant_cpu_to_le16(64),
	.bInterval        =	0,
};

static struct usb_endpoint_descriptor hs_bulk_out_desc = {
	.bLength          =	USB_DT_ENDPOINT_SIZE,
	.bDescriptorType  =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes     =	USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize   = __constant_cpu_to_le16(512),
	.bInterval        =	0,
};

static struct usb_endpoint_descriptor fs_bulk_out_desc = {
	.bLength          =	USB_DT_ENDPOINT_SIZE,
	.bDescriptorType  =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes     =	USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize   = __constant_cpu_to_le16(64),
	.bInterval        =	0,
};

static struct usb_descriptor_header *fs_diag_desc[] = {
	(struct usb_descriptor_header *) &intf_desc,
	(struct usb_descriptor_header *) &fs_bulk_in_desc,
	(struct usb_descriptor_header *) &fs_bulk_out_desc,
	NULL,
	};
static struct usb_descriptor_header *hs_diag_desc[] = {
	(struct usb_descriptor_header *) &intf_desc,
	(struct usb_descriptor_header *) &hs_bulk_in_desc,
	(struct usb_descriptor_header *) &hs_bulk_out_desc,
	NULL,
};

/**
 * struct diag_context - USB diag function driver private structure
 * @android_function: Used for registering with Android composite driver
 * @function: function structure for USB interface
 * @out: USB OUT endpoint struct
 * @in: USB IN endpoint struct
 * @in_desc: USB IN endpoint descriptor struct
 * @out_desc: USB OUT endpoint descriptor struct
 * @read_pool: List of requests used for Rx (OUT ep)
 * @write_pool: List of requests used for Tx (IN ep)
 * @config_work: Work item schedule after interface is configured to notify
 *               CONNECT event to diag char driver and updating product id
 *               and serial number to MODEM/IMEM.
 * @lock: Spinlock to proctect read_pool, write_pool lists
 * @cdev: USB composite device struct
 * @pdata: Platform data for this driver
 * @ch: USB diag channel
 *
 */
struct diag_context {
#ifdef CONFIG_USB_ANDROID_DIAG
	struct android_usb_function android_function;
#endif
	struct usb_function function;
	struct usb_ep *out;
	struct usb_ep *in;
	struct usb_endpoint_descriptor  *in_desc;
	struct usb_endpoint_descriptor  *out_desc;
	struct list_head read_pool;
	struct list_head write_pool;
	struct work_struct config_work;
	spinlock_t lock;
	unsigned configured;
	struct usb_composite_dev *cdev;
	struct usb_diag_platform_data *pdata;
	struct usb_diag_ch ch;
};

static inline struct diag_context *func_to_dev(struct usb_function *f)
{
	return container_of(f, struct diag_context, function);
}

static void usb_config_work_func(struct work_struct *work)
{
	struct diag_context *ctxt = container_of(work,
			struct diag_context, config_work);
	struct usb_composite_dev *cdev = ctxt->cdev;
	struct usb_gadget_strings *table;
	struct usb_string *s;

	if (ctxt->ch.notify)
		ctxt->ch.notify(ctxt->ch.priv, USB_DIAG_CONNECT, NULL);

	if (!ctxt->pdata || !ctxt->pdata->update_pid_and_serial_num)
		return;

	/* pass on product id and serial number to dload */
	if (!cdev->desc.iSerialNumber) {
		ctxt->pdata->update_pid_and_serial_num(
					cdev->desc.idProduct, 0);
		return;
	}

	/*
	 * Serial number is filled by the composite driver. So
	 * it is fair enough to assume that it will always be
	 * found at first table of strings.
	 */
	table = *(cdev->driver->strings);
	for (s = table->strings; s && s->s; s++)
		if (s->id == cdev->desc.iSerialNumber) {
			ctxt->pdata->update_pid_and_serial_num(
					cdev->desc.idProduct, s->s);
			break;
		}
}

static void diag_write_complete(struct usb_ep *ep,
		struct usb_request *req)
{
	struct diag_context *ctxt = ep->driver_data;
	struct diag_request *d_req = req->context;
	unsigned long flags;

	if (!req->status) {
		if ((req->length >= ep->maxpacket) &&
				((req->length % ep->maxpacket) == 0)) {
			req->length = 0;
			d_req->actual = req->actual;
			d_req->status = req->status;
			/* Queue zero length packet */
			usb_ep_queue(ctxt->in, req, GFP_ATOMIC);
			return;
		}
	}

	spin_lock_irqsave(&ctxt->lock, flags);
	list_add_tail(&req->list, &ctxt->write_pool);
	if (req->length != 0) {
		d_req->actual = req->actual;
		d_req->status = req->status;
	}
	spin_unlock_irqrestore(&ctxt->lock, flags);

	if (ctxt->ch.notify)
		ctxt->ch.notify(ctxt->ch.priv, USB_DIAG_WRITE_DONE, d_req);
}

static void diag_read_complete(struct usb_ep *ep,
		struct usb_request *req)
{
	struct diag_context *ctxt = ep->driver_data;
	struct diag_request *d_req = req->context;
	unsigned long flags;

	d_req->actual = req->actual;
	d_req->status = req->status;

	spin_lock_irqsave(&ctxt->lock, flags);
	list_add_tail(&req->list, &ctxt->read_pool);
	spin_unlock_irqrestore(&ctxt->lock, flags);

	if (ctxt->ch.notify)
		ctxt->ch.notify(ctxt->ch.priv, USB_DIAG_READ_DONE, d_req);
}

/**
 * usb_diag_open() - Open a diag channel over USB
 * @name: Name of the channel
 * @priv: Private structure pointer which will be passed in notify()
 * @notify: Callback function to receive notifications
 *
 * This function iterates overs the available channels and returns
 * the channel handler if the name matches. The notify callback is called
 * for CONNECT, DISCONNECT, READ_DONE and WRITE_DONE events.
 *
 */
struct usb_diag_ch *usb_diag_open(const char *name, void *priv,
		void (*notify)(void *, unsigned, struct diag_request *))
{
	struct usb_diag_ch *ch;
	unsigned long flags;
	int found = 0;

	spin_lock_irqsave(&ch_lock, flags);

	list_for_each_entry(ch, &usb_diag_ch_list, list) {
		if (!strcmp(name, ch->name)) {
			found = 1;
			break;
		}
	}
	if (!found) {
		ch =  ERR_PTR(-ENOENT);
		goto out;
	}
	if (ch->priv) {
		ch = ERR_PTR(-EBUSY);
		goto out;
	}

	ch->priv = priv;
	ch->notify = notify;
out:
	spin_unlock_irqrestore(&ch_lock, flags);
	return ch;
}
EXPORT_SYMBOL(usb_diag_open);

/**
 * usb_diag_close() - Close a diag channel over USB
 * @ch: Channel handler
 *
 * This function closes the diag channel.
 *
 */
void usb_diag_close(struct usb_diag_ch *ch)
{
	unsigned long flags;

	spin_lock_irqsave(&ch_lock, flags);
	ch->priv = NULL;
	ch->notify = NULL;
	spin_unlock_irqrestore(&ch_lock, flags);
}
EXPORT_SYMBOL(usb_diag_close);

/**
 * usb_diag_free_req() - Free USB requests
 * @ch: Channel handler
 *
 * This function free read and write USB requests for the interface
 * associated with this channel.
 *
 */
void usb_diag_free_req(struct usb_diag_ch *ch)
{
	struct diag_context *ctxt = ch->priv_usb;
	struct usb_request *req;
	struct list_head *act, *tmp;

	if (!ctxt)
		return;

	list_for_each_safe(act, tmp, &ctxt->write_pool) {
		req = list_entry(act, struct usb_request, list);
		list_del(&req->list);
		usb_ep_free_request(ctxt->in, req);
	}

	list_for_each_safe(act, tmp, &ctxt->read_pool) {
		req = list_entry(act, struct usb_request, list);
		list_del(&req->list);
		usb_ep_free_request(ctxt->out, req);
	}
}
EXPORT_SYMBOL(usb_diag_free_req);

/**
 * usb_diag_alloc_req() - Allocate USB requests
 * @ch: Channel handler
 * @n_write: Number of requests for Tx
 * @n_read: Number of requests for Rx
 *
 * This function allocate read and write USB requests for the interface
 * associated with this channel. The actual buffer is not allocated.
 * The buffer is passed by diag char driver.
 *
 */
int usb_diag_alloc_req(struct usb_diag_ch *ch, int n_write, int n_read)
{
	struct diag_context *ctxt = ch->priv_usb;
	struct usb_request *req;
	int i;

	if (!ctxt)
		return -ENODEV;

	for (i = 0; i < n_write; i++) {
		req = usb_ep_alloc_request(ctxt->in, GFP_ATOMIC);
		if (!req)
			goto fail;
		req->complete = diag_write_complete;
		list_add_tail(&req->list, &ctxt->write_pool);
	}

	for (i = 0; i < n_read; i++) {
		req = usb_ep_alloc_request(ctxt->out, GFP_ATOMIC);
		if (!req)
			goto fail;
		req->complete = diag_read_complete;
		list_add_tail(&req->list, &ctxt->read_pool);
	}

	return 0;

fail:
	usb_diag_free_req(ch);
	return -ENOMEM;

}
EXPORT_SYMBOL(usb_diag_alloc_req);

/**
 * usb_diag_read() - Read data from USB diag channel
 * @ch: Channel handler
 * @d_req: Diag request struct
 *
 * Enqueue a request on OUT endpoint of the interface corresponding to this
 * channel. This function returns proper error code when interface is not
 * in configured state, no Rx requests available and ep queue is failed.
 *
 * This function operates asynchronously. READ_DONE event is notified after
 * completion of OUT request.
 *
 */
int usb_diag_read(struct usb_diag_ch *ch, struct diag_request *d_req)
{
	struct diag_context *ctxt = ch->priv_usb;
	unsigned long flags;
	struct usb_request *req;

	spin_lock_irqsave(&ctxt->lock, flags);

	if (!ctxt->configured) {
		spin_unlock_irqrestore(&ctxt->lock, flags);
		return -EIO;
	}

	if (list_empty(&ctxt->read_pool)) {
		spin_unlock_irqrestore(&ctxt->lock, flags);
		ERROR(ctxt->cdev, "%s: no requests available\n", __func__);
		return -EAGAIN;
	}

	req = list_first_entry(&ctxt->read_pool, struct usb_request, list);
	list_del(&req->list);
	spin_unlock_irqrestore(&ctxt->lock, flags);

	req->buf = d_req->buf;
	req->length = d_req->length;
	req->context = d_req;
	if (usb_ep_queue(ctxt->out, req, GFP_ATOMIC)) {
		/* If error add the link to linked list again*/
		spin_lock_irqsave(&ctxt->lock, flags);
		list_add_tail(&req->list, &ctxt->read_pool);
		spin_unlock_irqrestore(&ctxt->lock, flags);
		ERROR(ctxt->cdev, "%s: cannot queue"
				" read request\n", __func__);
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL(usb_diag_read);

/**
 * usb_diag_write() - Write data from USB diag channel
 * @ch: Channel handler
 * @d_req: Diag request struct
 *
 * Enqueue a request on IN endpoint of the interface corresponding to this
 * channel. This function returns proper error code when interface is not
 * in configured state, no Tx requests available and ep queue is failed.
 *
 * This function operates asynchronously. WRITE_DONE event is notified after
 * completion of IN request.
 *
 */
int usb_diag_write(struct usb_diag_ch *ch, struct diag_request *d_req)
{
	struct diag_context *ctxt = ch->priv_usb;
	unsigned long flags;
	struct usb_request *req = NULL;

	spin_lock_irqsave(&ctxt->lock, flags);

	if (!ctxt->configured) {
		spin_unlock_irqrestore(&ctxt->lock, flags);
		return -EIO;
	}

	if (list_empty(&ctxt->write_pool)) {
		spin_unlock_irqrestore(&ctxt->lock, flags);
		ERROR(ctxt->cdev, "%s: no requests available\n", __func__);
		return -EAGAIN;
	}

	req = list_first_entry(&ctxt->write_pool, struct usb_request, list);
	list_del(&req->list);
	spin_unlock_irqrestore(&ctxt->lock, flags);

	req->buf = d_req->buf;
	req->length = d_req->length;
	req->context = d_req;
	if (usb_ep_queue(ctxt->in, req, GFP_ATOMIC)) {
		/* If error add the link to linked list again*/
		spin_lock_irqsave(&ctxt->lock, flags);
		list_add_tail(&req->list, &ctxt->write_pool);
		spin_unlock_irqrestore(&ctxt->lock, flags);
		ERROR(ctxt->cdev, "%s: cannot queue"
				" read request\n", __func__);
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL(usb_diag_write);

static void diag_function_disable(struct usb_function *f)
{
	struct diag_context  *dev = func_to_dev(f);
	unsigned long flags;

	DBG(dev->cdev, "diag_function_disable\n");

	spin_lock_irqsave(&dev->lock, flags);
	dev->configured = 0;
	spin_unlock_irqrestore(&dev->lock, flags);

	if (dev->ch.notify)
		dev->ch.notify(dev->ch.priv, USB_DIAG_DISCONNECT, NULL);

	usb_ep_disable(dev->in);
	dev->in->driver_data = NULL;

	usb_ep_disable(dev->out);
	dev->out->driver_data = NULL;

}

static int diag_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct diag_context  *dev = func_to_dev(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	unsigned long flags;
	int rc = 0;

	dev->in_desc = ep_choose(cdev->gadget,
			&hs_bulk_in_desc, &fs_bulk_in_desc);
	dev->out_desc = ep_choose(cdev->gadget,
			&hs_bulk_out_desc, &fs_bulk_in_desc);
	dev->in->driver_data = dev;
	rc = usb_ep_enable(dev->in, dev->in_desc);
	if (rc) {
		ERROR(dev->cdev, "can't enable %s, result %d\n",
						dev->in->name, rc);
		return rc;
	}
	dev->out->driver_data = dev;
	rc = usb_ep_enable(dev->out, dev->out_desc);
	if (rc) {
		ERROR(dev->cdev, "can't enable %s, result %d\n",
						dev->out->name, rc);
		usb_ep_disable(dev->in);
		return rc;
	}
	schedule_work(&dev->config_work);

	spin_lock_irqsave(&dev->lock, flags);
	dev->configured = 1;
	spin_unlock_irqrestore(&dev->lock, flags);

	return rc;
}

static void diag_function_unbind(struct usb_configuration *c,
		struct usb_function *f)
{
	struct diag_context *ctxt = func_to_dev(f);

	if (gadget_is_dualspeed(c->cdev->gadget))
		usb_free_descriptors(f->hs_descriptors);

	usb_free_descriptors(f->descriptors);
	ctxt->ch.priv_usb = NULL;
}

static int diag_function_bind(struct usb_configuration *c,
		struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct diag_context *ctxt = func_to_dev(f);
	struct usb_ep *ep;
	int status = -ENODEV;

	intf_desc.bInterfaceNumber =  usb_interface_id(c, f);

	ep = usb_ep_autoconfig(cdev->gadget, &fs_bulk_in_desc);
	ctxt->in = ep;
	ep->driver_data = ctxt;

	ep = usb_ep_autoconfig(cdev->gadget, &fs_bulk_out_desc);
	ctxt->out = ep;
	ep->driver_data = ctxt;

	/* copy descriptors, and track endpoint copies */
	f->descriptors = usb_copy_descriptors(fs_diag_desc);
	if (!f->descriptors)
		goto fail;

	if (gadget_is_dualspeed(c->cdev->gadget)) {
		hs_bulk_in_desc.bEndpointAddress =
				fs_bulk_in_desc.bEndpointAddress;
		hs_bulk_out_desc.bEndpointAddress =
				fs_bulk_out_desc.bEndpointAddress;

		/* copy descriptors, and track endpoint copies */
		f->hs_descriptors = usb_copy_descriptors(hs_diag_desc);
	}
	return 0;
fail:
	if (ctxt->out)
		ctxt->out->driver_data = NULL;
	if (ctxt->in)
		ctxt->in->driver_data = NULL;
	return status;

}

int diag_function_add(struct usb_configuration *c)
{
	struct diag_context *dev;
	struct usb_diag_ch *_ch;
	int found = 0, ret;

	DBG(c->cdev, "diag_function_add\n");

	list_for_each_entry(_ch, &usb_diag_ch_list, list) {
		/* Find an unused channel */
		if (!_ch->priv_usb) {
			found = 1;
			break;
		}
	}
	if (!found) {
		ERROR(c->cdev, "unable to get diag usb channel\n");
		return -ENODEV;
	}

	dev = container_of(_ch, struct diag_context, ch);
	/* claim the channel for this USB interface */
	_ch->priv_usb = dev;

	dev->cdev = c->cdev;
	dev->function.name = _ch->name;
	dev->function.descriptors = fs_diag_desc;
	dev->function.hs_descriptors = hs_diag_desc;
	dev->function.bind = diag_function_bind;
	dev->function.unbind = diag_function_unbind;
	dev->function.set_alt = diag_function_set_alt;
	dev->function.disable = diag_function_disable;
	spin_lock_init(&dev->lock);
	INIT_LIST_HEAD(&dev->read_pool);
	INIT_LIST_HEAD(&dev->write_pool);
	INIT_WORK(&dev->config_work, usb_config_work_func);

	ret = usb_add_function(c, &dev->function);
	if (ret) {
		INFO(c->cdev, "usb_add_function failed\n");
		_ch->priv_usb = NULL;
	}

	return ret;
}

static struct
usb_diag_ch *diag_setup(struct usb_diag_platform_data *pdata)
{
	struct usb_diag_ch *ch;
	struct diag_context *ctxt;
	unsigned long flags;

	ctxt = kzalloc(sizeof(*ctxt), GFP_KERNEL);
	if (!ctxt)
		return ERR_PTR(-ENOMEM);

	ctxt->pdata = pdata;
	ch = &ctxt->ch;

	spin_lock_irqsave(&ch_lock, flags);
	ch->name = pdata->ch_name;
	list_add_tail(&ch->list, &usb_diag_ch_list);
	spin_unlock_irqrestore(&ch_lock, flags);

	return ch;
}

static void diag_cleanup(struct usb_diag_ch *_ch)
{
	struct diag_context *dev = container_of(_ch, struct diag_context, ch);
	unsigned long flags;

	WARN_ON(_ch->priv || _ch->priv_usb);

	spin_lock_irqsave(&ch_lock, flags);
	list_del(&_ch->list);
	spin_unlock_irqrestore(&ch_lock, flags);

	kfree(dev);

}

#ifdef CONFIG_USB_ANDROID_DIAG
static int __init diag_probe(struct platform_device *pdev)
{
	struct diag_context *dev;
	struct usb_diag_ch *_ch;
	struct android_usb_function *func;
	struct usb_diag_platform_data *pdata = pdev->dev.platform_data;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	_ch = diag_setup(pdata);
	if (IS_ERR(_ch)) {
		dev_err(&pdev->dev, "unable to create diag channel\n");
		return PTR_ERR(_ch);
	}

	platform_set_drvdata(pdev, _ch);
	dev = container_of(_ch, struct diag_context, ch);
	func = &dev->android_function;
	func->name = pdata->ch_name;
	func->bind_config = diag_function_add;

	android_register_function(func);

	return 0;
}

static int diag_remove(struct platform_device *pdev)
{
	struct usb_diag_ch *ch = platform_get_drvdata(pdev);

	diag_cleanup(ch);

	return 0;
}

static struct platform_driver usb_diag_driver = {
	.remove		= diag_remove,
	.driver = {
		.name = "usb_diag",
		.owner = THIS_MODULE,
	},

};

static int __init usb_diag_init(void)
{
	return platform_driver_probe(&usb_diag_driver, diag_probe);
}
module_init(usb_diag_init);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("usb diag gadget driver");
MODULE_VERSION("2.00");
#endif /* CONFIG_USB_ANDROID_DIAG */
