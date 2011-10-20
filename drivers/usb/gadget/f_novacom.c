/*
 * f_novacom.c -- novacom function driver
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*
 * TODO:
 * Should remove the leftover codes from gadgetfs (inode.c)
 * especially the interface to the userspace should be simplified!
 *
 * inode.c -- user mode filesystem api for usb gadget controllers
 *
 * Copyright (C) 2003-2004 David Brownell
 * Copyright (C) 2003 Agilent Technologies
 */

//#define VERBOSE_DEBUG

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/poll.h>

#include <linux/types.h>
#include <linux/miscdevice.h>

#include <linux/usb/gadgetfs.h>
#include <linux/usb/gadget.h>

#define NOVACOM_DEVNAME_EP0	"novacom_ep0"
#define NOVACOM_DEVNAME_IN	"novacom_ep_in"
#define NOVACOM_DEVNAME_OUT	"novacom_ep_out"

/*-------------------------------------------------------------------------*/

#undef DBG
#undef VDBG
#undef ERROR
#undef WARNING
#undef INFO

#ifdef VERBOSE_DEBUG
#define DBG(d, fmt, args...) \
	dev_info(&(d)->gadget->dev , fmt , ## args)
#define VDBG(d, fmt, args...) \
	dev_info(&(d)->gadget->dev , fmt , ## args)
#else
#define DBG(d, fmt, args...) \
	dev_dbg(&(d)->gadget->dev , fmt , ## args)
#define VDBG(d, fmt, args...) \
	dev_vdbg(&(d)->gadget->dev , fmt , ## args)
#endif
#define ERROR(d, fmt, args...) \
	dev_err(&(d)->gadget->dev , fmt , ## args)
#define WARNING(d, fmt, args...) \
	dev_warn(&(d)->gadget->dev , fmt , ## args)
#define INFO(d, fmt, args...) \
	dev_info(&(d)->gadget->dev , fmt , ## args)

/*-------------------------------------------------------------------------*/

enum connect_state {
	STATE_DISCONNECTED,
	STATE_CONNECTED,
};

enum novacom_state {
	STATE_DEV_UNBOUND = 0,
	STATE_DEV_CLOSED,
	STATE_DEV_OPENED,
};

enum novacom_ep_state {
	STATE_EP_DISABLED = 0,
	STATE_EP_ENABLED,
};

#define	N_EVENT		5

static const char novacom_shortname[] = "novacom";

struct novacom_ep_descs {
	struct usb_endpoint_descriptor	*in;
	struct usb_endpoint_descriptor	*out;
};

struct novacom_ep {
	struct semaphore		sem;
	enum novacom_ep_state		state;
	struct f_novacom		*novacom;
	/* must hold dev->lock before accessing ep or req */
	struct usb_ep			*ep;
	struct usb_request		*req;
	ssize_t				status;
	char				name [16];
	struct usb_endpoint_descriptor	*desc;
	wait_queue_head_t		wait;
};

struct f_novacom {
	struct usb_function		function;
	struct usb_composite_dev	*cdev;
	int				data_id;

	struct novacom_ep		ep_in;
	struct novacom_ep		ep_out;

	struct novacom_ep_descs		fs;
	struct novacom_ep_descs		hs;

	/* ep0 */
	spinlock_t			lock;
	struct novacom			*novacom;
	enum novacom_state		state;	/* P: lock */
	enum connect_state		connect_state;
	struct usb_gadgetfs_event	event [N_EVENT];
	unsigned			ev_next;
	struct fasync_struct		*fasync;
	wait_queue_head_t		ep0_wait;
};

static inline struct f_novacom *func_to_novacom(struct usb_function *f)
{
	return container_of(f, struct f_novacom, function);
}

static struct f_novacom		*the_novacom;

/*----------------------------------------------------------------------*/

static struct usb_interface_descriptor novacom_interface_desc = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	/* .bInterfaceNumber = DYNAMIC */
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass =	0x47,
	.bInterfaceProtocol =	0x11,
	/* .iInterface = DYNAMIC */
};

/* full speed support: */

static struct usb_endpoint_descriptor novacom_fs_in_desc __initdata = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor novacom_fs_out_desc __initdata = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *novacom_fs_function[] __initdata = {
	(struct usb_descriptor_header *) &novacom_interface_desc,
	(struct usb_descriptor_header *) &novacom_fs_in_desc,
	(struct usb_descriptor_header *) &novacom_fs_out_desc,
	NULL,
};

/* high speed support: */

static struct usb_endpoint_descriptor novacom_hs_in_desc __initdata = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16 (512),
};

static struct usb_endpoint_descriptor novacom_hs_out_desc __initdata = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16 (512),
	.bInterval =		1,
};

static struct usb_descriptor_header *novacom_hs_function [] __initdata = {
	(struct usb_descriptor_header *) &novacom_interface_desc,
	(struct usb_descriptor_header *) &novacom_hs_in_desc,
	(struct usb_descriptor_header *) &novacom_hs_out_desc,
	NULL,
};

/* string descriptors: */

/* static strings, in UTF-8 */
static struct usb_string novacom_string_defs [] = {
	[0].s =	"Novacom",
	{ /* ZEROES END LIST */ },
};

static struct usb_gadget_strings novacom_string_table = {
	.language =	0x0409,	/* "en-us" */
	.strings =	novacom_string_defs,
};

static struct usb_gadget_strings *novacom_strings[] = {
	&novacom_string_table,
	NULL,
};

/*----------------------------------------------------------------------*/
/* SYNCHRONOUS ENDPOINT OPERATIONS (bulk/intr/iso)
 */

int
novacom_enable_ep (struct novacom_ep *nep)
{
	struct usb_ep *ep = nep->ep;

	VDBG(nep->novacom->cdev, "novacom: %s: enter (%s)\n", __func__, ep->name);

	if (nep->state == STATE_EP_DISABLED) {
		VDBG(nep->novacom->cdev, "novacom: %s: STATE_EP_ENABLED\n", nep->name);
		nep->state = STATE_EP_ENABLED;
	}
	return 0;
}

void
novacom_disable_ep (struct novacom_ep *nep)
{
	if (nep->state == STATE_EP_ENABLED) {
		VDBG(nep->novacom->cdev, "novacom: %s: STATE_EP_DISABLED\n", nep->name);
		nep->state = STATE_EP_DISABLED;
	}
}

void
novacom_epio_complete_in (struct usb_ep *ep, struct usb_request *req)
{
	struct f_novacom	*novacom = the_novacom;
	struct novacom_ep	*nep = &novacom->ep_in;

	VDBG(nep->novacom->cdev, "novacom: %s: enter\n", __func__);

	if (!req->context) {
		WARNING(novacom->cdev,
			"novacom: %s has no context\n", nep->name);
		return;
	}
	if (req->status)
		nep->status = req->status;
	else
		nep->status = req->actual;
	complete ((struct completion *)req->context);
}

void
novacom_epio_complete_out (struct usb_ep *ep, struct usb_request *req)
{
	struct f_novacom	*novacom = the_novacom;
	struct novacom_ep	*nep = &novacom->ep_out;

	VDBG(nep->novacom->cdev, "novacom: %s: enter\n", __func__);

	if (!req->context) {
		WARNING(novacom->cdev,
			"novacom: %s has no context\n", nep->name);
		return;
	}
	if (req->status)
		nep->status = req->status;
	else
		nep->status = req->actual;
	complete ((struct completion *)req->context);
}

/* tasklock endpoint, returning when it's connected.
 * still need dev->lock to use novacom_ep->ep.
 */
int
novacom_get_ready_ep (unsigned f_flags, struct novacom_ep *nep)
{
	int	val;

	if (f_flags & O_NONBLOCK) {
		if (down_trylock (&nep->sem) != 0)
			goto nonblock;
		if (nep->state != STATE_EP_ENABLED) {
			up (&nep->sem);
nonblock:
			val = -EAGAIN;
		} else
			val = 0;
		return val;
	}

	if ((val = down_interruptible (&nep->sem)) < 0)
		return val;

	switch (nep->state) {
	case STATE_EP_ENABLED:
		break;
	case STATE_EP_DISABLED:
	default:				/* error! */
		DBG(nep->novacom->cdev, "novacom: ep %p not available, state %d\n",
			   nep, nep->state);
		val = -ENODEV;
		up (&nep->sem);
	}
	return val;
}

ssize_t
novacom_ep_io (struct novacom_ep *nep, void *buf, unsigned len, int dir)
{
	DECLARE_COMPLETION_ONSTACK (done);
	struct f_novacom	*novacom = nep->novacom;
	int value;

	VDBG(novacom->cdev, "novacom: %s: enter (len=%d)\n", __func__, len);

	spin_lock_irq (&novacom->lock);
	if (likely (nep->ep != NULL)) {
		struct usb_request	*req = nep->req;

		req->context = &done;
		if (dir == USB_DIR_IN)
			req->complete = novacom_epio_complete_in;
		else
			req->complete = novacom_epio_complete_out;
		req->buf = buf;
		req->length = len;
		value = usb_ep_queue (nep->ep, req, GFP_ATOMIC);
	} else
		value = -ENODEV;
	spin_unlock_irq (&novacom->lock);

	if (likely (value == 0)) {
		value = wait_event_interruptible (done.wait, done.done);
		if (value != 0) {
			spin_lock_irq (&novacom->lock);
			if (likely (nep->ep != NULL)) {
				WARNING(novacom->cdev,
					"novacom: %s i/o interrupted\n",
					nep->name);
				usb_ep_dequeue (nep->ep, nep->req);
				spin_unlock_irq (&novacom->lock);

				if (wait_event_timeout (
					    done.wait, done.done,
					    msecs_to_jiffies(1000)) == 0)
					ERROR(novacom->cdev,
					      "novacom: i/o completion timeout\n");
				if (nep->status == -ECONNRESET)
					nep->status = -EINTR;
			} else {
				spin_unlock_irq (&novacom->lock);

				ERROR(novacom->cdev, "novacom: endpoint gone\n");
				nep->status = -ENODEV;
			}
		}
		return nep->status;
	}
	return value;
}


/* handle a synchronous OUT bulk/intr/iso transfer */
ssize_t
novacom_ep_read (struct file *fd, char __user *buf, size_t len, loff_t *ptr)
{
	struct novacom_ep	*nep = fd->private_data;
	struct f_novacom	*novacom = nep->novacom;
	void			*kbuf;
	ssize_t			value;

	VDBG(novacom->cdev, "novacom: %s: enter\n", __func__);

	if (len == 0) {
		INFO(novacom->cdev, "novacom_ep_read: len=0\n");
		return 0;
	}

	if ((value = novacom_get_ready_ep (fd->f_flags, nep)) < 0)
		return value;

	/* halt any endpoint by doing a "wrong direction" i/o call */
	if (nep->desc->bEndpointAddress & USB_DIR_IN) {
		if ((nep->desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
				== USB_ENDPOINT_XFER_ISOC)
			return -EINVAL;
		INFO(novacom->cdev, "novacom: %s halt\n", nep->name);
		spin_lock_irq (&novacom->lock);
		if (likely (nep->ep != NULL))
			usb_ep_set_halt (nep->ep);
		spin_unlock_irq (&novacom->lock);
		up (&nep->sem);
		return -EBADMSG;
	}

	/* FIXME readahead for O_NONBLOCK and poll(); careful with ZLPs */

	value = -ENOMEM;
	kbuf = kmalloc (len, GFP_KERNEL);
	if (unlikely (!kbuf))
		goto free1;

	value = novacom_ep_io (nep, kbuf, len, USB_DIR_OUT);
	pr_vdebug ("%s read %zu OUT, status %d\n",
		nep->name, len, (int) value);
	if (value >= 0 && copy_to_user (buf, kbuf, value))
		value = -EFAULT;

free1:
	up (&nep->sem);
	kfree (kbuf);
	return value;
}

/* handle a synchronous IN bulk/intr/iso transfer */
ssize_t
novacom_ep_write (struct file *fd, const char __user *buf, size_t len, loff_t *ptr)
{
	struct novacom_ep	*nep = fd->private_data;
	struct f_novacom	*novacom = nep->novacom;
	void			*kbuf;
	ssize_t			value;

	VDBG(novacom->cdev, "novacom: %s: enter\n", __func__);

	if (len == 0) {
		INFO(novacom->cdev, "novacom_ep_write: len=0\n");
		return 0;
	}

	if ((value = novacom_get_ready_ep (fd->f_flags, nep)) < 0)
		return value;

	/* halt any endpoint by doing a "wrong direction" i/o call */
	if (!(nep->desc->bEndpointAddress & USB_DIR_IN)) {
		if ((nep->desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
				== USB_ENDPOINT_XFER_ISOC)
			return -EINVAL;
		INFO(novacom->cdev, "novacom: %s halt\n", nep->name);
		spin_lock_irq (&novacom->lock);
		if (likely (nep->ep != NULL))
			usb_ep_set_halt (nep->ep);
		spin_unlock_irq (&novacom->lock);
		up (&nep->sem);
		return -EBADMSG;
	}

	/* FIXME writebehind for O_NONBLOCK and poll(), qlen = 1 */
	value = -ENOMEM;

	kbuf = kmalloc (len, GFP_KERNEL);
	if (!kbuf)
		goto free1;
	if (copy_from_user (kbuf, buf, len)) {
		value = -EFAULT;
		goto free1;
	}

	value = novacom_ep_io (nep, kbuf, len, USB_DIR_IN);
	pr_vdebug ("%s write %zu IN, status %d\n", nep->name, len, (int) value);
free1:
	up (&nep->sem);
	kfree (kbuf);
	return value;
}

void novacom_disable_ep (struct novacom_ep *nep);

int
novacom_ep_release (struct inode *inode, struct file *fd)
{
	struct novacom_ep	*nep = fd->private_data;

	VDBG(nep->novacom->cdev, "novacom: %s: enter\n", __func__);

	//novacom_disable_ep (nep);
	return 0;
}

int
novacom_ep_open (struct inode *inode, struct file *fd, int dir)
{
	struct f_novacom	*novacom = the_novacom;
	struct novacom_ep	*nep;
	int			value = -EBUSY;

	VDBG(novacom->cdev, "novacom: %s: enter\n", __func__);

	if (dir == USB_DIR_IN)
		nep = &novacom->ep_in;
	else
		nep = &novacom->ep_out;

	if (down_interruptible (&nep->sem) != 0)
		return -EINTR;
	spin_lock_irq (&novacom->lock);
	if (nep->state == STATE_EP_ENABLED) {
		value = 0;
		fd->private_data = nep;
		VDBG(novacom->cdev, "novacom: %s opened\n", nep->name);
	} else {
		WARNING(novacom->cdev, "novacom: %s busy (nep->state=%d)\n", nep->name, nep->state);
	}
	spin_unlock_irq (&novacom->lock);
	up (&nep->sem);
	return value;
}

int novacom_ep_in_open (struct inode *inode, struct file *fd)
{
	return novacom_ep_open(inode, fd, USB_DIR_IN);
}

int novacom_ep_out_open (struct inode *inode, struct file *fd)
{
	return novacom_ep_open(inode, fd, USB_DIR_OUT);
}

static const struct file_operations novacom_ep_in_fops = {
	.owner =	THIS_MODULE,
	.llseek =	no_llseek,

	.open =		novacom_ep_in_open,
	.read =		novacom_ep_read,
	.write =	novacom_ep_write,
	.release =	novacom_ep_release,
};

static const struct file_operations novacom_ep_out_fops = {
	.owner =	THIS_MODULE,
	.llseek =	no_llseek,

	.open =		novacom_ep_out_open,
	.read =		novacom_ep_read,
	.write =	novacom_ep_write,
	.release =	novacom_ep_release,
};

static struct miscdevice novacom_ep_in_device = {
	.minor =	MISC_DYNAMIC_MINOR,
	.name =		NOVACOM_DEVNAME_IN,
	.fops =		&novacom_ep_in_fops,
};

static struct miscdevice novacom_ep_out_device = {
	.minor =	MISC_DYNAMIC_MINOR,
	.name =		NOVACOM_DEVNAME_OUT,
	.fops =		&novacom_ep_out_fops,
};

/*----------------------------------------------------------------------*/

struct usb_gadgetfs_event *
novacom_next_event (struct f_novacom *novacom, enum usb_gadgetfs_event_type type)
{
	struct usb_gadgetfs_event	*event;
	unsigned			i;

	VDBG(novacom->cdev, "novacom: %s: enter\n", __func__);

	switch (type) {
	/* these events purge the queue */
	case GADGETFS_DISCONNECT:
	case GADGETFS_CONNECT:
		novacom->ev_next = 0;
		break;
	case GADGETFS_SETUP:		/* previous request timed out */
	case GADGETFS_SUSPEND:		/* same effect */
		/* these events can't be repeated */
		for (i = 0; i != novacom->ev_next; i++) {
			if (novacom->event [i].type != type)
				continue;
			VDBG(novacom->cdev, "novacom: discard old event[%d] %d\n", i, type);
			novacom->ev_next--;
			if (i == novacom->ev_next)
				break;
			/* indices start at zero, for simplicity */
			memmove (&novacom->event [i], &novacom->event [i + 1],
				sizeof (struct usb_gadgetfs_event)
					* (novacom->ev_next - i));
		}
		break;
	default:
		BUG ();
	}
	VDBG(novacom->cdev, "novacom: event[%d] = %d\n", novacom->ev_next, type);
	event = &novacom->event [novacom->ev_next++];
	BUG_ON (novacom->ev_next > N_EVENT);
	memset (event, 0, sizeof *event);
	event->type = type;
	return event;
}

inline void
novacom_ep0_send_event (struct f_novacom *novacom, int value)
{
	struct usb_gadgetfs_event	*event;

	VDBG(novacom->cdev, "novacom: %s: value=%d\n", __func__, value);

	if (value > 0) {
		/* set_config */
		event = novacom_next_event (novacom, GADGETFS_SETUP);
		event->u.setup.bRequest = USB_REQ_SET_CONFIGURATION;
		event->u.setup.wValue = value;
	} else {
		/* disconnect */
		novacom_next_event (novacom, GADGETFS_DISCONNECT);
	}
	wake_up (&novacom->ep0_wait);
	kill_fasync (&novacom->fasync, SIGIO, POLL_IN);
}

ssize_t
novacom_ep0_read (struct file *fd, char __user *buf, size_t len, loff_t *ptr)
{
	struct f_novacom	*novacom = the_novacom;
	ssize_t			retval;
	enum connect_state	connect_state;

	if (!novacom) {
		return -ENOENT;
	}

	VDBG(novacom->cdev, "novacom: %s: enter\n", __func__);

	spin_lock_irq (&novacom->lock);

	connect_state = novacom->connect_state;

	if (len < sizeof novacom->event [0]) {
		retval = -EINVAL;
		goto done;
	}
	len -= len % sizeof (struct usb_gadgetfs_event);

scan:
	/* return queued events right away */
	if (novacom->ev_next != 0) {
		unsigned		n;

		n = len / sizeof (struct usb_gadgetfs_event);
		if (novacom->ev_next < n)
			n = novacom->ev_next;

		spin_unlock_irq (&novacom->lock);
		len = n * sizeof (struct usb_gadgetfs_event);
		if (copy_to_user (buf, &novacom->event, len))
			retval = -EFAULT;
		else
			retval = len;
		if (len > 0) {
			/* NOTE this doesn't guard against broken drivers;
			 * concurrent ep0 readers may lose events.
			 */
			spin_lock_irq (&novacom->lock);
			if (novacom->ev_next > n) {
				memmove(&novacom->event[0], &novacom->event[n],
					sizeof (struct usb_gadgetfs_event)
						* (novacom->ev_next - n));
			}
			novacom->ev_next -= n;
			spin_unlock_irq (&novacom->lock);
		}
		return retval;
	}
	if (fd->f_flags & O_NONBLOCK) {
		retval = -EAGAIN;
		goto done;
	}

	switch (connect_state) {
	default:
		ERROR(novacom->cdev, "novacom: fail connect_state=%d\n", connect_state);
		retval = -ESRCH;
		break;
	case STATE_DISCONNECTED:
	case STATE_CONNECTED:
		spin_unlock_irq (&novacom->lock);
		VDBG(novacom->cdev, "novacom: wait\n");

		/* wait for events */
		retval = wait_event_interruptible (novacom->ep0_wait,
				novacom->ev_next != 0);
		if (retval < 0)
			return retval;
		spin_lock_irq (&novacom->lock);
		goto scan;
	}

done:
	spin_unlock_irq (&novacom->lock);
	return retval;
}

int
novacom_ep0_fasync (int f, struct file *fd, int on)
{
	struct f_novacom	*novacom = the_novacom;

	if (!novacom) {
		return -ENOENT;
	}

	// caller must F_SETOWN before signal delivery happens
	VDBG(novacom->cdev, "novacom: %s %s\n", __func__, on ? "on" : "off");
	return fasync_helper (f, fd, on, &novacom->fasync);
}

int
novacom_ep0_open (struct inode *inode, struct file *fd)
{
	struct f_novacom	*novacom = the_novacom;
	int			value = -EBUSY;

	if (!novacom) {
		return -ENOENT;
	}

	VDBG(novacom->cdev, "novacom: %s: enter (novacom=%p)\n", __func__, novacom);
	spin_lock_irq(&novacom->lock);
	if (novacom->state == STATE_DEV_CLOSED) {
		/* novacom->ev_next = 0; */
		VDBG(novacom->cdev, "novacom: STATE_DEV_OPENED\n");
		novacom->state = STATE_DEV_OPENED;
		value = 0;

		if (novacom->connect_state == STATE_CONNECTED) {
			value = novacom_enable_ep (&novacom->ep_in);
			if (value != 0) {
				printk("can't enable IN ep\n");
				goto fail;
			}
			value = novacom_enable_ep (&novacom->ep_out);
			if (value != 0) {
				printk("can't enable OUT ep\n");
				goto fail;
			}
			novacom_ep0_send_event (novacom, 1);
		}
	}
fail:
	spin_unlock_irq(&novacom->lock);
	return value;
}

int
novacom_ep0_release (struct inode *inode, struct file *fd)
{
	struct f_novacom	*novacom = the_novacom;

	if (!novacom) {
		return -ENOENT;
	}

	/* closing ep0 === shutdown all */

	novacom_disable_ep (&novacom->ep_in);
	novacom_disable_ep (&novacom->ep_out);

	/* at this point "good" hardware has disconnected the
	 * device from USB; the host won't see it any more.
	 * alternatively, all host requests will time out.
	 */

	fasync_helper (-1, fd, 0, &novacom->fasync);

	/* other endpoints were all decoupled from this device */
	spin_lock_irq(&novacom->lock);
	VDBG(novacom->cdev, "novacom: STATE_DEV_CLOSED\n");
	novacom->state = STATE_DEV_CLOSED;
	spin_unlock_irq(&novacom->lock);
	return 0;
}

unsigned int
novacom_ep0_poll (struct file *fd, poll_table *wait)
{
	struct f_novacom	*novacom = the_novacom;
	int			mask = 0;

	if (!novacom) {
		return -ENOENT;
	}

	VDBG(novacom->cdev, "novacom: %s: enter\n", __func__);

	poll_wait(fd, &novacom->ep0_wait, wait);

	spin_lock_irq (&novacom->lock);

	VDBG(novacom->cdev, "novacom: novacom->ev_next=%d\n", novacom->ev_next);

	if (novacom->ev_next != 0)
		mask = POLLIN;

	spin_unlock_irq(&novacom->lock);
	return mask;
}

/* used after device configuration */
static const struct file_operations novacom_ep0_fops = {
	.owner =	THIS_MODULE,
	.llseek =	no_llseek,

	.open =		novacom_ep0_open,
	.read =		novacom_ep0_read,
	.fasync =	novacom_ep0_fasync,
	.poll =		novacom_ep0_poll,
	.release =	novacom_ep0_release,
};

static struct miscdevice novacom_ep0_device = {
	.minor =	MISC_DYNAMIC_MINOR,
	.name =		NOVACOM_DEVNAME_EP0,
	.fops =		&novacom_ep0_fops,
};

/*----------------------------------------------------------------------*/

int novacom_connect(struct f_novacom *novacom)
{
	int status;

	status = novacom_enable_ep(&novacom->ep_in);
	if (status) {
		return status;
	}
	status = novacom_enable_ep(&novacom->ep_out);
	if (status) {
		return status;
	}

	spin_lock (&novacom->lock);
	VDBG(novacom->cdev, "novacom: STATE_CONNECTED\n");
	novacom->connect_state = STATE_CONNECTED;

	if (novacom->state == STATE_DEV_OPENED)
		novacom_ep0_send_event (novacom, 1);

	spin_unlock (&novacom->lock);

	return 0;
}

void novacom_disconnect(struct f_novacom *novacom)
{
	VDBG(novacom->cdev, "novacom: %s: enter\n", __func__);

	spin_lock (&novacom->lock);
	if (novacom->connect_state != STATE_DISCONNECTED) {
		VDBG(novacom->cdev, "novacom: STATE_DISCONNECTED\n");
		novacom->connect_state = STATE_DISCONNECTED;

		novacom_ep0_send_event (novacom, 0);
	}
	spin_unlock (&novacom->lock);

	novacom_disable_ep(&novacom->ep_out);
	novacom_disable_ep(&novacom->ep_in);
}

/*----------------------------------------------------------------------*/

int __init
novacom_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_novacom	*novacom = func_to_novacom(f);
	int		id;
	int		status;
	struct usb_ep	*ep;

	/* allocate interface ID(s) */
	id = usb_interface_id(c, f);
	if (id < 0)
		goto fail;
	novacom->data_id = id;

	novacom_interface_desc.bInterfaceNumber = id;

	status = -ENODEV;

	/* allocate endpoints */
	ep = usb_ep_autoconfig(cdev->gadget, &novacom_fs_in_desc);
	if (!ep)
		goto fail;
	novacom->ep_in.ep = ep;
	ep->driver_data = cdev; /* claim */

	ep = usb_ep_autoconfig(cdev->gadget, &novacom_fs_out_desc);
	if (!ep)
		goto fail;
	novacom->ep_out.ep = ep;
	ep->driver_data = cdev; /* claim */

	/* copy descriptors, and track endpoint copies */
	f->descriptors = usb_copy_descriptors(novacom_fs_function);
	if (!f->descriptors)
		goto fail;

	novacom->fs.in = usb_find_endpoint(novacom_fs_function,
			f->descriptors, &novacom_fs_in_desc);
	novacom->fs.out = usb_find_endpoint(novacom_fs_function,
			f->descriptors, &novacom_fs_out_desc);

	/* support all relevant hardware speeds... we expect that when
	 * hardware is dual speed, all bulk-capable endpoints work at
	 * both speeds
	 */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		novacom_hs_in_desc.bEndpointAddress =
				novacom_fs_in_desc.bEndpointAddress;
		novacom_hs_out_desc.bEndpointAddress =
				novacom_fs_out_desc.bEndpointAddress;

		/* copy descriptors, and track endpoint copies */
		f->hs_descriptors = usb_copy_descriptors(novacom_hs_function);

		novacom->hs.in = usb_find_endpoint(novacom_hs_function,
				f->hs_descriptors, &novacom_hs_in_desc);
		novacom->hs.out = usb_find_endpoint(novacom_hs_function,
				f->hs_descriptors, &novacom_hs_out_desc);
	}

	novacom->ep_in.req =
		usb_ep_alloc_request(novacom->ep_in.ep, GFP_KERNEL);
	if (!novacom->ep_in.req) {
		ERROR(cdev, "usb_ep_alloc_request failed\n");
		goto fail;
	}
	novacom->ep_out.req =
		usb_ep_alloc_request(novacom->ep_out.ep, GFP_KERNEL);
	if (!novacom->ep_out.req) {
		ERROR(cdev, "usb_ep_alloc_request failed\n");
		goto fail;
	}

	VDBG(cdev, "novacom: STATE_DEV_CLOSED\n");
	novacom->state = STATE_DEV_CLOSED;

	DBG(cdev, "novacom: %s speed IN/%s OUT/%s\n",
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			novacom->ep_in.name, novacom->ep_out.name);
	return 0;

fail:
	if (novacom->ep_out.req) {
		usb_ep_free_request(novacom->ep_out.ep, novacom->ep_out.req);
		novacom->ep_out.req = NULL;
	}
	if (novacom->ep_in.req) {
		usb_ep_free_request(novacom->ep_in.ep, novacom->ep_in.req);
		novacom->ep_in.req = NULL;
	}

	/* we might as well release our claims on endpoints */
	if (novacom->ep_out.ep)
		novacom->ep_out.ep->driver_data = NULL;
	if (novacom->ep_in.ep)
		novacom->ep_in.ep->driver_data = NULL;

	ERROR(cdev, "%s/%p: can't bind, err %d\n", f->name, f, status);

	return status;
}

void
novacom_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct f_novacom		*novacom = func_to_novacom(f);

	spin_lock_irq(&novacom->lock);

	if (novacom->ep_out.req) {
		usb_ep_free_request(novacom->ep_out.ep, novacom->ep_out.req);
		novacom->ep_out.req = NULL;
	}
	if (novacom->ep_in.req) {
		usb_ep_free_request(novacom->ep_in.ep, novacom->ep_in.req);
		novacom->ep_in.req = NULL;
	}

	novacom->state = STATE_DEV_UNBOUND;

	spin_unlock_irq(&novacom->lock);

	if (gadget_is_dualspeed(c->cdev->gadget))
		usb_free_descriptors(f->hs_descriptors);
	usb_free_descriptors(f->descriptors);

	misc_deregister(&novacom_ep_out_device);
	misc_deregister(&novacom_ep_in_device);
	misc_deregister(&novacom_ep0_device);
	kfree(novacom);
	the_novacom = NULL;
	novacom_string_defs[0].id = 0;
}

/*----------------------------------------------------------------------*/

int novacom_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct f_novacom	*novacom = func_to_novacom(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	int value;

	/* we know alt is zero, so this is an activation or a reset */
	if (novacom->ep_in.ep->driver_data) {
		DBG(cdev, "reset novacom\n");
		novacom_disconnect(novacom);
		usb_ep_disable(novacom->ep_in.ep);
		novacom->ep_in.ep->driver_data = NULL;
		usb_ep_disable(novacom->ep_out.ep);
		novacom->ep_out.ep->driver_data = NULL;
	} else {
		DBG(cdev, "activate novacom");
		novacom->ep_in.desc = ep_choose(cdev->gadget,
				novacom->hs.in, novacom->fs.in);
		novacom->ep_out.desc = ep_choose(cdev->gadget,
				novacom->hs.out, novacom->fs.out);
	}

	value = usb_ep_enable (novacom->ep_in.ep, novacom->ep_in.desc);
	if (value != 0) {
		ERROR(cdev,
		      "novacom: usb_ep_enable ep_in failed --> %d\n",
		      value);
		return value;
	}

	value = usb_ep_enable (novacom->ep_out.ep, novacom->ep_out.desc);
	if (value != 0) {
		ERROR(cdev,
		      "novacom: usb_ep_enable ep_out failed --> %d\n",
		      value);
		return value;
	}

	novacom_connect(novacom);
	return 0;
}

void novacom_disable(struct usb_function *f)
{
	struct f_novacom	*novacom = func_to_novacom(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	DBG(cdev, "novacom deactivated\n");
	novacom_disconnect(novacom);
	usb_ep_disable(novacom->ep_in.ep);
	novacom->ep_in.ep->driver_data = NULL;
	usb_ep_disable(novacom->ep_out.ep);
	novacom->ep_out.ep->driver_data = NULL;
}

/*----------------------------------------------------------------------*/

int __init novacom_bind_config(struct usb_configuration *c)
{
	struct usb_composite_dev	*cdev = c->cdev;
	struct f_novacom	*novacom;
	int		status;

	if (novacom_string_defs[0].id == 0) {
		status = usb_string_id(cdev);
		if (status < 0)
			return status;
		novacom_string_defs[0].id = status;

		novacom_interface_desc.iInterface = status;
	}

	/* allocate and initialize one new instance */
	novacom = kzalloc(sizeof *novacom, GFP_KERNEL);
	if (!novacom)
		return -ENOMEM;

	novacom->function.name = "novacom";
	novacom->function.strings = novacom_strings;
	/* descriptors are per-instance copies */
	novacom->function.bind = novacom_bind;
	novacom->function.unbind = novacom_unbind;
	novacom->function.set_alt = novacom_set_alt;
	novacom->function.disable = novacom_disable;

	novacom->cdev = cdev;

	strcpy(novacom->ep_in.name, "ep_in");
	novacom->ep_in.state = STATE_EP_DISABLED;
	novacom->ep_in.novacom = novacom;
	init_MUTEX (&novacom->ep_in.sem);
	init_waitqueue_head (&novacom->ep_in.wait);

	strcpy(novacom->ep_out.name, "ep_out");
	novacom->ep_out.state = STATE_EP_DISABLED;
	novacom->ep_out.novacom = novacom;
	init_MUTEX (&novacom->ep_out.sem);
	init_waitqueue_head (&novacom->ep_out.wait);

	VDBG(cdev, "novacom: STATE_DEV_UNBOUND\n");
	novacom->state = STATE_DEV_UNBOUND;
	VDBG(cdev, "novacom: STATE_DISCONNECTED\n");
	novacom->connect_state = STATE_DISCONNECTED;
	spin_lock_init (&novacom->lock);
	init_waitqueue_head (&novacom->ep0_wait);

	the_novacom = novacom;

	status = misc_register(&novacom_ep0_device);
	if (status) {
		ERROR(cdev, "novacom: failed to register ep0_device (%d)\n",
		      status);
		goto err1;
	}
	status = misc_register(&novacom_ep_in_device);
	if (status) {
		ERROR(cdev, "novacom: failed to register ep_in_device (%d)\n",
		      status);
		goto err2;
	}
	status = misc_register(&novacom_ep_out_device);
	if (status) {
		ERROR(cdev, "novacom: failed to register ep_out_device (%d)\n",
		      status);
		goto err3;
	}
	status = usb_add_function(c, &novacom->function);
	if (status) {
		ERROR(cdev, "novacom: failed to usb_add_function (%d)\n",
		      status);
		goto err4;
	}
	return 0;

err4:
	misc_deregister(&novacom_ep_out_device);
err3:
	misc_deregister(&novacom_ep_in_device);
err2:
	misc_deregister(&novacom_ep0_device);
err1:
	kfree(novacom);
	printk("novacom gadget driver failed to initialize\n");
	return status;
}
