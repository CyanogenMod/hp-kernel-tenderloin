/*
 * f_racm.c -- USB Reduced CDC serial (ACM) function driver
 *
 * Copyright (C) 2003 Al Borchers (alborchers@steinerpoint.com)
 * Copyright (C) 2008 by David Brownell
 * Copyright (C) 2008 by Nokia Corporation
 * Copyright (C) 2009 by Palm, Inc.
 *
 * This software is distributed under the terms of the GNU General
 * Public License ("GPL") as published by the Free Software Foundation,
 * either version 2 of that License or (at your option) any later version.
 */

/* #define VERBOSE_DEBUG */

#include <linux/kernel.h>
#include <linux/device.h>

#include "u_serial.h"
#include "gadget_chips.h"

/*-------------------------------------------------------------------------*/

#undef DBG
#undef VDBG
#undef ERROR
#undef WARNING
#undef INFO

#define DBG(d, fmt, args...) \
	dev_dbg(&(d)->gadget->dev , fmt , ## args)
#define VDBG(d, fmt, args...) \
	dev_vdbg(&(d)->gadget->dev , fmt , ## args)
#define ERROR(d, fmt, args...) \
	dev_err(&(d)->gadget->dev , fmt , ## args)
#define WARNING(d, fmt, args...) \
	dev_warn(&(d)->gadget->dev , fmt , ## args)
#define INFO(d, fmt, args...) \
	dev_info(&(d)->gadget->dev , fmt , ## args)

/*-------------------------------------------------------------------------*/

/*
 * This CDC ACM function support just wraps control functions and
 * notifications around the generic serial-over-usb code.
 *
 * Because CDC ACM is standardized by the USB-IF, many host operating
 * systems have drivers for it.  Accordingly, ACM is the preferred
 * interop solution for serial-port type connections.  The control
 * models are often not necessary, and in any case don't do much in
 * this bare-bones implementation.
 *
 * Note that even MS-Windows has some support for ACM.  However, that
 * support is somewhat broken because when you use ACM in a composite
 * device, having multiple interfaces confuses the poor OS.  It doesn't
 * seem to understand CDC Union descriptors.  The new "association"
 * descriptors (roughly equivalent to CDC Unions) may sometimes help.
 */

struct racm_ep_descs {
	struct usb_endpoint_descriptor	*in;
	struct usb_endpoint_descriptor	*out;
	struct usb_endpoint_descriptor	*notify;
};

struct racm_icount {
	u32	dtr, rts;
};

struct f_racm {
	struct gserial			port;
	u8				data_id; /* no ctrl_id */
	u8				port_num;

	u8				pending;
	u8				connected;

	/* lock is mostly for pending and notify_req ... they get accessed
	 * by callbacks both from tty (open/close/break) under its spinlock,
	 * and notify_req.complete() which can't use that lock.
	 */
	spinlock_t			lock;

	struct racm_ep_descs		fs;
	struct racm_ep_descs		hs;

	struct usb_ep			*notify;
	struct usb_endpoint_descriptor	*notify_desc;
	struct usb_request		*notify_req;

	struct usb_cdc_line_coding	port_line_coding;	/* 8-N-1 etc */

	/* SetControlLineState request -- CDC 1.1 section 6.2.14 (INPUT) */
	u16				port_handshake_bits;
#define ACM_CTRL_RTS	(1 << 1)	/* unused with full duplex */
#define ACM_CTRL_DTR	(1 << 0)	/* host is ready for data r/w */

	/* SerialState notification -- CDC 1.1 section 6.3.5 (OUTPUT) */
	u16				serial_state;
#define ACM_CTRL_OVERRUN	(1 << 6)
#define ACM_CTRL_PARITY		(1 << 5)
#define ACM_CTRL_FRAMING	(1 << 4)
#define ACM_CTRL_RI		(1 << 3)
#define ACM_CTRL_BRK		(1 << 2)
#define ACM_CTRL_DSR		(1 << 1)
#define ACM_CTRL_DCD		(1 << 0)

	struct racm_icount		icount;
	wait_queue_head_t		state_wait;
};

static inline struct f_racm *func_to_racm(struct usb_function *f)
{
	return container_of(f, struct f_racm, port.func);
}

static inline struct f_racm *port_to_racm(struct gserial *p)
{
	return container_of(p, struct f_racm, port);
}

/*-------------------------------------------------------------------------*/

/* notification endpoint uses smallish and infrequent fixed-size messages */

#define GS_LOG2_NOTIFY_INTERVAL		5	/* 1 << 5 == 32 msec */
#define GS_NOTIFY_MAXPACKET		10	/* notification + 2 bytes */

/* interface and class descriptors: */

static struct usb_interface_descriptor racm_interface_desc __initdata = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	/* .bInterfaceNumber = DYNAMIC */
	.bNumEndpoints =	3,
	.bInterfaceClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceProtocol =	USB_CLASS_VENDOR_SPEC,
	/* .iInterface = DYNAMIC */
};

/* full speed support: */

static struct usb_endpoint_descriptor racm_fs_notify_desc __initdata = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	__constant_cpu_to_le16(GS_NOTIFY_MAXPACKET),
	.bInterval =		1 << GS_LOG2_NOTIFY_INTERVAL,
};

static struct usb_endpoint_descriptor racm_fs_in_desc __initdata = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor racm_fs_out_desc __initdata = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *racm_fs_function[] __initdata = {
	(struct usb_descriptor_header *) &racm_interface_desc,
	(struct usb_descriptor_header *) &racm_fs_notify_desc,
	(struct usb_descriptor_header *) &racm_fs_in_desc,
	(struct usb_descriptor_header *) &racm_fs_out_desc,
	NULL,
};

/* high speed support: */

static struct usb_endpoint_descriptor racm_hs_notify_desc __initdata = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	__constant_cpu_to_le16(GS_NOTIFY_MAXPACKET),
	.bInterval =		GS_LOG2_NOTIFY_INTERVAL+4,
};

static struct usb_endpoint_descriptor racm_hs_in_desc __initdata = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor racm_hs_out_desc __initdata = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
};

static struct usb_descriptor_header *racm_hs_function[] __initdata = {
	(struct usb_descriptor_header *) &racm_interface_desc,
	(struct usb_descriptor_header *) &racm_hs_notify_desc,
	(struct usb_descriptor_header *) &racm_hs_in_desc,
	(struct usb_descriptor_header *) &racm_hs_out_desc,
	NULL,
};

/* string descriptors: */

/* static strings, in UTF-8 */
static struct usb_string racm_string_defs[] = {
	[0].s = "Reduced CDC Abstract Control Model (R-ACM)",
	{  /* ZEROES END LIST */ },
};

static struct usb_gadget_strings racm_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		racm_string_defs,
};

static struct usb_gadget_strings *racm_strings[] = {
	&racm_string_table,
	NULL,
};

/*-------------------------------------------------------------------------*/

/* ACM control ... data handling is delegated to tty library code.
 * The main task of this function is to activate and deactivate
 * that code based on device state; track parameters like line
 * speed, handshake state, and so on; and issue notifications.
 */

static void racm_complete_set_line_coding(struct usb_ep *ep,
		struct usb_request *req)
{
	struct f_racm	*racm = ep->driver_data;
	struct usb_composite_dev *cdev = racm->port.func.config->cdev;

	if (req->status != 0) {
		DBG(cdev, "racm ttyGS%d completion, err %d\n",
				racm->port_num, req->status);
		return;
	}

	/* REVISIT: why is req->actual zero? */
	/* Windows tries to set 115.2Kbps. We need to ignore it. */
#if 0
	/* normal completion */
	if (req->actual != sizeof(racm->port_line_coding)) {
		DBG(cdev, "racm ttyGS%d short resp, len %d\n",
				racm->port_num, req->actual);
		usb_ep_set_halt(ep);
	} else {
		struct usb_cdc_line_coding	*value = req->buf;

		/* REVISIT:  we currently just remember this data.
		 * If we change that, (a) validate it first, then
		 * (b) update whatever hardware needs updating,
		 * (c) worry about locking.  This is information on
		 * the order of 9600-8-N-1 ... most of which means
		 * nothing unless we control a real RS232 line.
		 */
		racm->port_line_coding = *value;
	}
#endif
}

static int racm_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct f_racm		*racm = func_to_racm(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_request	*req = cdev->req;
	int			value = -EOPNOTSUPP;
	u16			w_index = le16_to_cpu(ctrl->wIndex);
	u16			w_value = le16_to_cpu(ctrl->wValue);
	u16			w_length = le16_to_cpu(ctrl->wLength);

	/* composite driver infrastructure handles everything except
	 * CDC class messages; interface activation uses set_alt().
	 *
	 * Note CDC spec table 4 lists the ACM request profile.  It requires
	 * encapsulated command support ... we don't handle any, and respond
	 * to them by stalling.  Options include get/set/clear comm features
	 * (not that useful) and SEND_BREAK.
	 */
	switch ((ctrl->bRequestType << 8) | ctrl->bRequest) {

	/* SET_LINE_CODING ... just read and save what the host sends */
	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_REQ_SET_LINE_CODING:
		if (w_length != sizeof(struct usb_cdc_line_coding))
			goto invalid;

		value = w_length;
		cdev->gadget->ep0->driver_data = racm;
		req->complete = racm_complete_set_line_coding;
		break;

	/* GET_LINE_CODING ... return what host sent, or initial value */
	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_REQ_GET_LINE_CODING:
		value = min_t(unsigned, w_length,
				sizeof(struct usb_cdc_line_coding));
		memcpy(req->buf, &racm->port_line_coding, value);
		break;

	/* SET_CONTROL_LINE_STATE ... save what the host sent */
	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_REQ_SET_CONTROL_LINE_STATE:
		value = 0;

		/* FIXME we should not allow data to flow until the
		 * host sets the ACM_CTRL_DTR bit; and when it clears
		 * that bit, we should return to that no-flow state.
		 */

		spin_lock(&racm->lock);
		if ((racm->port_handshake_bits & ACM_CTRL_DTR) !=
		    (w_value & ACM_CTRL_DTR))
			racm->icount.dtr++;
		if ((racm->port_handshake_bits & ACM_CTRL_RTS) !=
		    (w_value & ACM_CTRL_RTS))
			racm->icount.rts++;
		racm->port_handshake_bits = w_value;
		wake_up_interruptible(&racm->state_wait);
		spin_unlock(&racm->lock);
		break;

	default:
invalid:
		VDBG(cdev, "invalid control req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
	}

	/* respond with data transfer or status phase? */
	if (value >= 0) {
		DBG(cdev, "racm ttyGS%d req%02x.%02x v%04x i%04x l%d\n",
			racm->port_num, ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
		req->zero = 0;
		req->length = value;
		value = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (value < 0)
			ERROR(cdev, "racm response on ttyGS%d, err %d\n",
					racm->port_num, value);
	}

	/* device either stalls (value < 0) or reports success */
	return value;
}

static int racm_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct f_racm		*racm = func_to_racm(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	/* we know alt == 0, so this is an activation or a reset */

	if (racm->notify->driver_data) {
		VDBG(cdev, "reset racm control interface %d\n", intf);
		usb_ep_disable(racm->notify);
	} else {
		VDBG(cdev, "init racm ctrl interface %d\n", intf);
		racm->notify_desc = ep_choose(cdev->gadget,
					     racm->hs.notify,
					     racm->fs.notify);
	}
	usb_ep_enable(racm->notify, racm->notify_desc);
	racm->notify->driver_data = racm;

	if (racm->port.in->driver_data) {
		DBG(cdev, "reset racm ttyGS%d\n", racm->port_num);
		gserial_disconnect(&racm->port);
	} else {
		DBG(cdev, "activate racm ttyGS%d\n", racm->port_num);
		racm->port.in_desc = ep_choose(cdev->gadget,
					      racm->hs.in, racm->fs.in);
		racm->port.out_desc = ep_choose(cdev->gadget,
					       racm->hs.out, racm->fs.out);
	}
	gserial_connect(&racm->port, racm->port_num);

	/* REVISIT: need a copy in racm? */
	racm->port_line_coding = racm->port.port_line_coding;

	return 0;
}

static void racm_disable(struct usb_function *f)
{
	struct f_racm	*racm = func_to_racm(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	DBG(cdev, "racm ttyGS%d deactivated\n", racm->port_num);
	gserial_disconnect(&racm->port);
	usb_ep_disable(racm->notify);
	racm->notify->driver_data = NULL;
}

/*-------------------------------------------------------------------------*/

/**
 * racm_cdc_notify - issue CDC notification to host
 * @racm: wraps host to be notified
 * @type: notification type
 * @value: Refer to cdc specs, wValue field.
 * @data: data to be sent
 * @length: size of data
 * Context: irqs blocked, racm->lock held, racm_notify_req non-null
 *
 * Returns zero on sucess or a negative errno.
 *
 * See section 6.3.5 of the CDC 1.1 specification for information
 * about the only notification we issue:  SerialState change.
 */
static int racm_cdc_notify(struct f_racm *racm, u8 type, u16 value,
		void *data, unsigned length)
{
	struct usb_ep			*ep = racm->notify;
	struct usb_request		*req;
	struct usb_cdc_notification	*notify;
	const unsigned			len = sizeof(*notify) + length;
	void				*buf;
	int				status;

	req = racm->notify_req;
	racm->notify_req = NULL;
	racm->pending = false;

	req->length = len;
	notify = req->buf;
	buf = notify + 1;

	notify->bmRequestType = USB_DIR_IN | USB_TYPE_CLASS
			| USB_RECIP_INTERFACE;
	notify->bNotificationType = type;
	notify->wValue = cpu_to_le16(value);
	notify->wIndex = cpu_to_le16(racm->data_id);
	notify->wLength = cpu_to_le16(length);
	memcpy(buf, data, length);

	/* ep_queue() can complete immediately if it fills the fifo... */
	spin_unlock(&racm->lock);
	status = usb_ep_queue(ep, req, GFP_ATOMIC);
	spin_lock(&racm->lock);

	if (status < 0) {
		ERROR(racm->port.func.config->cdev,
				"racm ttyGS%d can't notify serial state, %d\n",
				racm->port_num, status);
		racm->notify_req = req;
	}

	return status;
}

static int racm_notify_serial_state(struct f_racm *racm)
{
	struct usb_composite_dev *cdev = racm->port.func.config->cdev;
	int			status;

	spin_lock(&racm->lock);
	if (racm->notify_req) {
		DBG(cdev, "racm ttyGS%d serial state %04x\n",
				racm->port_num, racm->serial_state);
		status = racm_cdc_notify(racm, USB_CDC_NOTIFY_SERIAL_STATE,
				0, &racm->serial_state, sizeof(racm->serial_state));
	} else {
		racm->pending = true;
		status = 0;
	}
	spin_unlock(&racm->lock);
	return status;
}

static void racm_cdc_notify_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_racm		*racm = req->context;
	u8			doit = false;

	/* on this call path we do NOT hold the port spinlock,
	 * which is why ACM needs its own spinlock
	 */
	spin_lock(&racm->lock);
	if (req->status != -ESHUTDOWN)
		doit = racm->pending;
	racm->notify_req = req;
	spin_unlock(&racm->lock);

	if (doit)
		racm_notify_serial_state(racm);
}

/* connect == the TTY link is open */

static void racm_connect(struct gserial *port)
{
	struct f_racm		*racm = port_to_racm(port);

	racm->connected = 1;
#if 0
	racm->serial_state |= ACM_CTRL_DSR | ACM_CTRL_DCD;
	racm_notify_serial_state(racm);
#endif
}

static void racm_disconnect(struct gserial *port)
{
	struct f_racm		*racm = port_to_racm(port);

	racm->connected = 0;
	wake_up_interruptible(&racm->state_wait);
#if 0
	racm->serial_state &= ~(ACM_CTRL_DSR | ACM_CTRL_DCD);
	racm_notify_serial_state(racm);
#endif
}

static int racm_send_break(struct gserial *port, int duration)
{
	struct f_racm		*racm = port_to_racm(port);
	u16			state;

	state = racm->serial_state;
	state &= ~ACM_CTRL_BRK;
	if (duration)
		state |= ACM_CTRL_BRK;

	racm->serial_state = state;
	return racm_notify_serial_state(racm);
}

/*-------------------------------------------------------------------------*/

struct gs_icounter_struct {
	int dtr, rts;
};

#define G_TIOCMGET	_IOR('g', 101, int)
#define G_TIOCMSET	_IOW('g', 102, int)
#define G_TIOCMIWAIT	_IOW('g', 103, struct gs_icounter_struct)
#define G_TIOCGICOUNT	_IOR('g', 104, struct gs_icounter_struct)

static int racm_g_tiocmget(struct f_racm *racm)
{
	return (racm->port_handshake_bits & ACM_CTRL_DTR ? TIOCM_DTR : 0) |
		(racm->port_handshake_bits & ACM_CTRL_RTS ? TIOCM_RTS : 0) |
		(racm->serial_state & ACM_CTRL_DSR ? TIOCM_DSR : 0) |
		(racm->serial_state & ACM_CTRL_RI  ? TIOCM_RI  : 0) |
		(racm->serial_state & ACM_CTRL_DCD ? TIOCM_CD  : 0);
}

static int racm_g_tiocmset(struct f_racm *racm, unsigned int set)
{
	u16		state;

	state = racm->serial_state;
	if (set & TIOCM_DSR)
		state |= ACM_CTRL_DSR;
	if (set & TIOCM_RI)
		state |= ACM_CTRL_RI;
	if (set & TIOCM_CD)
		state |= ACM_CTRL_DCD;
	racm->serial_state = state;
	return racm_notify_serial_state(racm);
}

static int racm_wait_serial_state(struct f_racm *racm,
				struct gs_icounter_struct __user *icnt)
{
	struct gs_icounter_struct icount;
	DECLARE_WAITQUEUE(wait, current);
	struct racm_icount cprev, cnow;
	int ret;
	unsigned long flags;

	if (copy_from_user(&icount, icnt, sizeof(icount)))
		return -EFAULT;

	spin_lock_irqsave(&racm->lock, flags);
	memcpy(&cprev, &racm->icount, sizeof(struct racm_icount));
	spin_unlock_irqrestore(&racm->lock, flags);

	if ((icount.dtr != cprev.dtr) ||
	    (icount.rts != cprev.rts)) {
		ret = 0;
		return ret;
	}

	add_wait_queue(&racm->state_wait, &wait);

	for (;;) {
		spin_lock_irqsave(&racm->lock, flags);
		memcpy(&cnow, &racm->icount, sizeof(struct racm_icount));
		spin_unlock_irqrestore(&racm->lock, flags);

		set_current_state(TASK_INTERRUPTIBLE);

		if ((cnow.dtr != cprev.dtr) ||
		    (cnow.rts != cprev.rts)) {
			ret = 0;
			break;
		}

		schedule();

		spin_lock_irqsave(&racm->lock, flags);
		if (!racm->connected) {
			spin_unlock_irqrestore(&racm->lock, flags);
			ret = -EIO;
			break;
		}
		spin_unlock_irqrestore(&racm->lock, flags);

		/* see if a signal did it */
		if (signal_pending(current)) {
			ret = -EINTR;
			break;
		}

		cprev = cnow;
	}

	current->state = TASK_RUNNING;
	remove_wait_queue(&racm->state_wait, &wait);

	return ret;
}

static int racm_get_count(struct f_racm *racm,
			  struct gs_icounter_struct __user *icnt)
{
	struct gs_icounter_struct icount;
	struct racm_icount cnow;
	unsigned long flags;

	spin_lock_irqsave(&racm->lock, flags);
	memcpy(&cnow, &racm->icount, sizeof(struct racm_icount));
	spin_unlock_irqrestore(&racm->lock, flags);

	icount.dtr         = cnow.dtr;
	icount.rts         = cnow.rts;

	return copy_to_user(icnt, &icount, sizeof(icount)) ? -EFAULT : 0;
}

static int racm_ioctl(struct gserial *port, struct tty_struct *tty,
		      struct file *file, unsigned int cmd, unsigned long arg)
{
	struct f_racm	*racm = port_to_racm(port);
	int		status;
	unsigned int	val;
	void __user	*uarg = (void __user *)arg;

	switch (cmd) {
	case G_TIOCMGET:
		status = racm_g_tiocmget(racm);
		if (status >= 0)
			status = put_user(status, (unsigned __user *)uarg);
		break;
	case G_TIOCMSET:
		status = get_user(val, (unsigned __user *)uarg);
		if (status)
			return status;
		status = racm_g_tiocmset(racm, val);
		break;
	case G_TIOCMIWAIT:
		status = racm_wait_serial_state(racm, uarg);
		break;
	case G_TIOCGICOUNT:
		status = racm_get_count(racm, uarg);
		break;
	default:
		status = -ENOIOCTLCMD;	   /* could not handle ioctl */
	}
	return status;
}

/*-------------------------------------------------------------------------*/

/* ACM function driver setup/binding */
static int __init
racm_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_racm		*racm = func_to_racm(f);
	int			status;
	struct usb_ep		*ep;

	/* allocate instance-specific interface IDs, and patch descriptors */
	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	racm->data_id = status;

	racm_interface_desc.bInterfaceNumber = status;

	status = -ENODEV;

	/* allocate instance-specific endpoints */
	ep = usb_ep_autoconfig(cdev->gadget, &racm_fs_in_desc);
	if (!ep)
		goto fail;
	racm->port.in = ep;
	ep->driver_data = cdev;	/* claim */

	ep = usb_ep_autoconfig(cdev->gadget, &racm_fs_out_desc);
	if (!ep)
		goto fail;
	racm->port.out = ep;
	ep->driver_data = cdev;	/* claim */

	ep = usb_ep_autoconfig(cdev->gadget, &racm_fs_notify_desc);
	if (!ep)
		goto fail;
	racm->notify = ep;
	ep->driver_data = cdev;	/* claim */

	/* allocate notification */
	racm->notify_req = gs_alloc_req(ep,
			sizeof(struct usb_cdc_notification) + 2,
			GFP_KERNEL);
	if (!racm->notify_req)
		goto fail;

	racm->notify_req->complete = racm_cdc_notify_complete;
	racm->notify_req->context = racm;

	/* copy descriptors, and track endpoint copies */
	f->descriptors = usb_copy_descriptors(racm_fs_function);
	if (!f->descriptors)
		goto fail;

	racm->fs.in = usb_find_endpoint(racm_fs_function,
			f->descriptors, &racm_fs_in_desc);
	racm->fs.out = usb_find_endpoint(racm_fs_function,
			f->descriptors, &racm_fs_out_desc);
	racm->fs.notify = usb_find_endpoint(racm_fs_function,
			f->descriptors, &racm_fs_notify_desc);

	/* support all relevant hardware speeds... we expect that when
	 * hardware is dual speed, all bulk-capable endpoints work at
	 * both speeds
	 */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		racm_hs_in_desc.bEndpointAddress =
				racm_fs_in_desc.bEndpointAddress;
		racm_hs_out_desc.bEndpointAddress =
				racm_fs_out_desc.bEndpointAddress;
		racm_hs_notify_desc.bEndpointAddress =
				racm_fs_notify_desc.bEndpointAddress;

		/* copy descriptors, and track endpoint copies */
		f->hs_descriptors = usb_copy_descriptors(racm_hs_function);

		racm->hs.in = usb_find_endpoint(racm_hs_function,
				f->hs_descriptors, &racm_hs_in_desc);
		racm->hs.out = usb_find_endpoint(racm_hs_function,
				f->hs_descriptors, &racm_hs_out_desc);
		racm->hs.notify = usb_find_endpoint(racm_hs_function,
				f->hs_descriptors, &racm_hs_notify_desc);
	}

	DBG(cdev, "racm ttyGS%d: %s speed IN/%s OUT/%s NOTIFY/%s\n",
			racm->port_num,
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			racm->port.in->name, racm->port.out->name,
			racm->notify->name);
	return 0;

fail:
	if (racm->notify_req)
		gs_free_req(racm->notify, racm->notify_req);

	/* we might as well release our claims on endpoints */
	if (racm->notify)
		racm->notify->driver_data = NULL;
	if (racm->port.out)
		racm->port.out->driver_data = NULL;
	if (racm->port.in)
		racm->port.in->driver_data = NULL;

	ERROR(cdev, "%s/%p: can't bind, err %d\n", f->name, f, status);

	return status;
}

static void
racm_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct f_racm		*racm = func_to_racm(f);

	if (gadget_is_dualspeed(c->cdev->gadget))
		usb_free_descriptors(f->hs_descriptors);
	usb_free_descriptors(f->descriptors);
	gs_free_req(racm->notify, racm->notify_req);
	kfree(racm);
	racm_string_defs[0].id = 0;
}

/**
 * racm_bind_config - add a CDC ACM function to a configuration
 * @c: the configuration to support the CDC ACM instance
 * @port_num: /dev/ttyGS* port this interface will use
 * Context: single threaded during gadget setup
 *
 * Returns zero on success, else negative errno.
 *
 * Caller must have called @gserial_setup() with enough ports to
 * handle all the ones it binds.  Caller is also responsible
 * for calling @gserial_cleanup() before module unload.
 */
int __init racm_bind_config(struct usb_configuration *c, u8 port_num)
{
	struct f_racm	*racm;
	int		status;

	/* REVISIT might want instance-specific strings to help
	 * distinguish instances ...
	 */

	/* maybe allocate device-global string IDs, and patch descriptors */
	if (racm_string_defs[0].id == 0) {
		status = usb_string_id(c->cdev);
		if (status < 0)
			return status;
		racm_string_defs[0].id = status;
		racm_interface_desc.iInterface = status;
	}

	/* allocate and initialize one new instance */
	racm = kzalloc(sizeof *racm, GFP_KERNEL);
	if (!racm)
		return -ENOMEM;

	spin_lock_init(&racm->lock);

	init_waitqueue_head(&racm->state_wait);

	racm->port_num = port_num;

	racm->port.connect = racm_connect;
	racm->port.disconnect = racm_disconnect;
	racm->port.send_break = racm_send_break;
	racm->port.ioctl = racm_ioctl;

	racm->port.func.name = "racm";
	racm->port.func.strings = racm_strings;
	/* descriptors are per-instance copies */
	racm->port.func.bind = racm_bind;
	racm->port.func.unbind = racm_unbind;
	racm->port.func.set_alt = racm_set_alt;
	racm->port.func.setup = racm_setup;
	racm->port.func.disable = racm_disable;

	status = usb_add_function(c, &racm->port.func);
	if (status)
		kfree(racm);
	return status;
}
