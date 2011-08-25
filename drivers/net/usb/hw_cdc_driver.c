/*
 * CDC Ethernet based the networking peripherals of Huawei data card devices
 * This driver is developed based on usbnet.c and cdc_ether.c
 * Copyright (C) 2009 by Franko Fang (Huawei Technologies Co., Ltd.)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will support Huawei data card devices for Linux networking,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */



#include <linux/module.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/workqueue.h>
#include <linux/mii.h>
#include <linux/usb.h>
#include <linux/sched.h>
#include <linux/ctype.h>
#include <linux/usb/cdc.h>
#include <linux/usbdevice_fs.h>

#include <linux/version.h>
/////////////////////////////////////////////////////////////////////////////////////////////////
#define DRIVER_VERSION "v2.05.01.00"
#define DRIVER_AUTHOR "Franko Fang <huananhu@huawei.com>"
#define DRIVER_DESC "Huawei ether driver for 3G data card ether device"
//////////////////////////////////////////////////////////////////////////////////////////////////////
#define RX_MAX_QUEUE_MEMORY (60 * 1518)
#define	RX_QLEN(dev) (((dev)->udev->speed == USB_SPEED_HIGH) ? \
			(RX_MAX_QUEUE_MEMORY/(dev)->rx_urb_size) : 4)
#define	TX_QLEN(dev) (((dev)->udev->speed == USB_SPEED_HIGH) ? \
			(RX_MAX_QUEUE_MEMORY/(dev)->hard_mtu) : 4)

// reawaken network queue this soon after stopping; else watchdog barks
#define TX_TIMEOUT_JIFFIES	(5*HZ)

// throttle rx/tx briefly after some faults, so khubd might disconnect()
// us (it polls at HZ/4 usually) before we report too many false errors.
#define THROTTLE_JIFFIES	(HZ/8)

// between wakeups
#define UNLINK_TIMEOUT_MS	3
//////////////////////////////////////////////////////////////////////////////////////////////
// randomly generated ethernet address
static u8	node_id [ETH_ALEN];

static const char driver_name [] = "hw_cdc_net";

/* use ethtool to change the level for any given device */
static int msg_level = -1;
module_param (msg_level, int, 0);
MODULE_PARM_DESC (msg_level, "Override default message level");
//////////////////////////////////////////////////////////////////////////////////////////
#define HW_TLP_MASK_SYNC   0xF800
#define HW_TLP_MASK_LENGTH 0x07FF
#define HW_TLP_BITS_SYNC   0xF800
#pragma pack(push, 1)
struct hw_cdc_tlp
{
	unsigned short pktlength;
	unsigned char payload;
};
#define HW_TLP_HDR_LENGTH sizeof(unsigned short)
#pragma pack(pop)

typedef enum __HW_TLP_BUF_STATE {
	HW_TLP_BUF_STATE_IDLE = 0,
	HW_TLP_BUF_STATE_PARTIAL_FILL,
	HW_TLP_BUF_STATE_PARTIAL_HDR,
	HW_TLP_BUF_STATE_HDR_ONLY,
	HW_TLP_BUF_STATE_ERROR
}HW_TLP_BUF_STATE;

struct hw_cdc_tlp_tmp{
	void *buffer;
	unsigned short pktlength;
	unsigned short bytesneeded;
};
/*max ethernet pkt size 1514*/
#define HW_USB_RECEIVE_BUFFER_SIZE    1600L
/*for Tin-layer-protocol (TLP)*/
#define HW_USB_MRECEIVE_BUFFER_SIZE   4096L
/*for TLP*/
#define HW_USB_MRECEIVE_MAX_BUFFER_SIZE (1024*16)

#define HW_JUNGO_BCDDEVICE_VALUE 0x0102
///////////////////////////////////////////////////////////////////////////////////////////
#define EVENT_TX_HALT 0
#define EVENT_RX_HALT 1
#define EVENT_RX_MEMORY 2
#define EVENT_STS_SPLIT 3
#define EVENT_LINK_RESET 4

struct hw_cdc_net{
	/* housekeeping */
	struct usb_device	*udev;
	struct usb_interface	*intf;
	const char		*driver_name;
	const char 		*driver_desc;
	void			*driver_priv;
	wait_queue_head_t	*wait;
	struct mutex		phy_mutex;
	unsigned char		suspend_count;

	/* i/o info: pipes etc */
	unsigned		in, out;
	struct usb_host_endpoint *status;
	unsigned		maxpacket;
	struct timer_list	delay;

	/* protocol/interface state */
	struct net_device	*net;
	struct net_device_stats	stats;
	int			msg_enable;
	unsigned long		data [5];
	u32			xid;
	u32			hard_mtu;	/* count any extra framing */
	size_t			rx_urb_size;	/* size for rx urbs */
	struct mii_if_info	mii;

	/* various kinds of pending driver work */
	struct sk_buff_head	rxq;
	struct sk_buff_head	txq;
	struct sk_buff_head	done;
	struct urb		*interrupt;
	struct tasklet_struct	bh;

	struct work_struct	kevent;
	struct delayed_work status_work;//fangxiaozhi added for work
	int			qmi_sync;
	unsigned long		flags;

	/*The state and buffer for the data of TLP*/
	HW_TLP_BUF_STATE hw_tlp_buffer_state;
	struct hw_cdc_tlp_tmp hw_tlp_tmp_buf;
	/*indicate the download tlp feature is activated or not*/
	int hw_tlp_download_is_actived;
};

static inline struct usb_driver *driver_of(struct usb_interface *intf)
{
	return to_usb_driver(intf->dev.driver);
}


/* Drivers that reuse some of the standard USB CDC infrastructure
 * (notably, using multiple interfaces according to the CDC
 * union descriptor) get some helper code.
 */
struct hw_dev_state {
	struct usb_cdc_header_desc	*header;
	struct usb_cdc_union_desc	*u;
	struct usb_cdc_ether_desc	*ether;
	struct usb_interface		*control;
	struct usb_interface		*data;
};


/* we record the state for each of our queued skbs */
enum skb_state {
	illegal = 0,
	tx_start, tx_done,
	rx_start, rx_done, rx_cleanup
};

struct skb_data {	/* skb->cb is one of these */
	struct urb		*urb;
	struct hw_cdc_net		*dev;
	enum skb_state		state;
	size_t			length;
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef DEBUG
#define devdbg(hw_cdc_net, fmt, arg...) \
	printk(KERN_DEBUG "%s: " fmt "\n" , (hw_cdc_net)->net->name , ## arg)
#else
#define devdbg(hw_cdc_net, fmt, arg...) do {} while(0)
#endif

#define deverr(hw_cdc_net, fmt, arg...) \
	printk(KERN_ERR "%s: " fmt "\n" , (hw_cdc_net)->net->name , ## arg)
#define devwarn(hw_cdc_net, fmt, arg...) \
	printk(KERN_WARNING "%s: " fmt "\n" , (hw_cdc_net)->net->name , ## arg)

#define devinfo(hw_cdc_net, fmt, arg...) \
	printk(KERN_INFO "%s: " fmt "\n" , (hw_cdc_net)->net->name , ## arg); \


////////////////////////////////////////////////////////////////////////////////
static void hw_cdc_status(struct hw_cdc_net *dev, struct urb *urb);
static inline int hw_get_ethernet_addr(struct hw_cdc_net *dev);
static int hw_cdc_bind(struct hw_cdc_net *dev, struct usb_interface *intf);
void hw_cdc_unbind(struct hw_cdc_net *dev, struct usb_interface *intf);


/*Begin : fangxiaozhi added for work*/
//static void hw_cdc_check_status_work(struct work_struct *work);
/*{
	struct delayed_work *option_suspend_wq
}*/

/*End : fangxiaozhi added for work*/





/* handles CDC Ethernet and many other network "bulk data" interfaces */
int hw_get_endpoints(struct hw_cdc_net *dev, struct usb_interface *intf)
{
	int				tmp;
	struct usb_host_interface	*alt = NULL;
	struct usb_host_endpoint	*in = NULL, *out = NULL;
	struct usb_host_endpoint	*status = NULL;

	for (tmp = 0; tmp < intf->num_altsetting; tmp++) {
		unsigned	ep;

		in = out = status = NULL;
		alt = intf->altsetting + tmp;

		/* take the first altsetting with in-bulk + out-bulk;
		 * remember any status endpoint, just in case;
		 * ignore other endpoints and altsetttings.
		 */
		for (ep = 0; ep < alt->desc.bNumEndpoints; ep++) {

			struct usb_host_endpoint	*e;
			int				intr = 0;

			e = alt->endpoint + ep;
			switch (e->desc.bmAttributes) {
			case USB_ENDPOINT_XFER_INT:
				if (!usb_endpoint_dir_in(&e->desc))
					continue;
				intr = 1;
				/* FALLTHROUGH */
			case USB_ENDPOINT_XFER_BULK:
				break;
			default:
				continue;
			}
			if (usb_endpoint_dir_in(&e->desc)) {
				if (!intr && !in)
					in = e;
				else if (intr && !status)
					status = e;
			} else {
				if (!out)
					out = e;
			}
		}
		if (in && out)
			break;
	}
	if (!alt || !in || !out)
		return -EINVAL;
	if (alt->desc.bAlternateSetting != 0) {
		tmp = usb_set_interface (dev->udev, alt->desc.bInterfaceNumber,
				alt->desc.bAlternateSetting);
		if (tmp < 0)
			return tmp;
	}

	dev->in = usb_rcvbulkpipe (dev->udev,
			in->desc.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);
	dev->out = usb_sndbulkpipe (dev->udev,
			out->desc.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);
	dev->status = status;
	return 0;
}
EXPORT_SYMBOL_GPL(hw_get_endpoints);

static void intr_complete (struct urb *urb);

static int init_status (struct hw_cdc_net *dev, struct usb_interface *intf)
{
	char		*buf = NULL;
	unsigned	pipe = 0;
	unsigned	maxp;
	unsigned	period;


	pipe = usb_rcvintpipe (dev->udev,
			dev->status->desc.bEndpointAddress
				& USB_ENDPOINT_NUMBER_MASK);
	maxp = usb_maxpacket (dev->udev, pipe, 0);

	/* avoid 1 msec chatter:  min 8 msec poll rate */
	period = max ((int) dev->status->desc.bInterval,
		(dev->udev->speed == USB_SPEED_HIGH) ? 7 : 3);

	buf = kmalloc (maxp, GFP_KERNEL);
	if (buf) {
		dev->interrupt = usb_alloc_urb (0, GFP_KERNEL);
		if (!dev->interrupt) {
			kfree (buf);
			return -ENOMEM;
		} else {
			usb_fill_int_urb(dev->interrupt, dev->udev, pipe,
				buf, maxp, intr_complete, dev, period);
			dev_dbg(&intf->dev,
				"status ep%din, %d bytes period %d\n",
				usb_pipeendpoint(pipe), maxp, period);
		}
	}
	return 0;
}

/* Passes this packet up the stack, updating its accounting.
 * Some link protocols batch packets, so their rx_fixup paths
 * can return clones as well as just modify the original skb.
 */
void hw_skb_return (struct hw_cdc_net *dev, struct sk_buff *skb)
{
	int	status;

	skb->protocol = eth_type_trans (skb, dev->net);
	dev->stats.rx_packets++;
	dev->stats.rx_bytes += skb->len;

	if (netif_msg_rx_status (dev))
		devdbg (dev, "< rx, len %zu, type 0x%x",
			skb->len + sizeof (struct ethhdr), skb->protocol);
	memset (skb->cb, 0, sizeof (struct skb_data));
	status = netif_rx (skb);
	if (status != NET_RX_SUCCESS && netif_msg_rx_err (dev))
		devdbg (dev, "netif_rx status %d", status);
}
EXPORT_SYMBOL_GPL(hw_skb_return);

// unlink pending rx/tx; completion handlers do all other cleanup

static int unlink_urbs (struct hw_cdc_net *dev, struct sk_buff_head *q)
{
	unsigned long		flags;
	struct sk_buff		*skb, *skbnext;
	int			count = 0;

	spin_lock_irqsave (&q->lock, flags);
	for (skb = q->next; skb != (struct sk_buff *) q; skb = skbnext) {
		struct skb_data		*entry;
		struct urb		*urb;
		int			retval;

		entry = (struct skb_data *) skb->cb;
		urb = entry->urb;
		skbnext = skb->next;

		// during some PM-driven resume scenarios,
		// these (async) unlinks complete immediately
		retval = usb_unlink_urb (urb);
		if (retval != -EINPROGRESS && retval != 0)
			devdbg (dev, "unlink urb err, %d", retval);
		else
			count++;

	}
	spin_unlock_irqrestore (&q->lock, flags);
	return count;
}


// Flush all pending rx urbs
// minidrivers may need to do this when the MTU changes

void hw_unlink_rx_urbs(struct hw_cdc_net *dev)
{
	if (netif_running(dev->net)) {
		(void) unlink_urbs (dev, &dev->rxq);
		tasklet_schedule(&dev->bh);
	}
}
EXPORT_SYMBOL_GPL(hw_unlink_rx_urbs);


/*-------------------------------------------------------------------------
 *
 * Network Device Driver (peer link to "Host Device", from USB host)
 *
 *-------------------------------------------------------------------------*/

static int hw_change_mtu (struct net_device *net, int new_mtu)
{
	struct hw_cdc_net	*dev = netdev_priv(net);
	int		ll_mtu = new_mtu + net->hard_header_len;
	int		old_hard_mtu = dev->hard_mtu;
	int		old_rx_urb_size = dev->rx_urb_size;

	if (new_mtu <= 0)
		return -EINVAL;
	// no second zero-length packet read wanted after mtu-sized packets
	if ((ll_mtu % dev->maxpacket) == 0)
		return -EDOM;
	net->mtu = new_mtu;

	dev->hard_mtu = net->mtu + net->hard_header_len;
	if (dev->rx_urb_size == old_hard_mtu) {
		dev->rx_urb_size = dev->hard_mtu;
		if (dev->rx_urb_size > old_rx_urb_size)
			hw_unlink_rx_urbs(dev);
	}

	return 0;
}

/*-------------------------------------------------------------------------*/
//#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 30)
static struct net_device_stats *hw_get_stats (struct net_device *net)
{
	struct hw_cdc_net	*dev = netdev_priv(net);
	return &dev->stats;
}
//#endif
/*-------------------------------------------------------------------------*/

static void tx_defer_bh(struct hw_cdc_net *dev, struct sk_buff *skb, struct sk_buff_head *list)
{
	unsigned long		flags;

	spin_lock_irqsave(&list->lock, flags);
	__skb_unlink(skb, list);
	spin_unlock(&list->lock);
	spin_lock(&dev->done.lock);
	__skb_queue_tail(&dev->done, skb);
	if (1 <= dev->done.qlen)
		tasklet_schedule(&dev->bh);
	spin_unlock_irqrestore(&dev->done.lock, flags);
}
////////////////////////////////////////////
static HW_TLP_BUF_STATE submit_skb(struct hw_cdc_net *dev, unsigned char *data, unsigned int len)
{
	struct sk_buff		*skb;
	struct skb_data * entry;

	unsigned long flags;

	if (len > dev->rx_urb_size){
		devdbg(dev, "The package length is too large\n");
		return HW_TLP_BUF_STATE_ERROR;
	}

	if ((skb = alloc_skb (len + NET_IP_ALIGN, GFP_ATOMIC)) == NULL) {
		return HW_TLP_BUF_STATE_ERROR;
	}
	skb_reserve (skb, NET_IP_ALIGN);


	entry = (struct skb_data *) skb->cb;
	entry->urb = NULL;
	entry->dev = dev;
	entry->state = rx_done;
	entry->length = skb->len;

	memcpy(skb->data, data, len);
	skb->len = len;

	spin_lock_irqsave(&dev->done.lock, flags);
	__skb_queue_tail(&dev->done, skb);
	if (1 <= dev->done.qlen){
		tasklet_schedule(&dev->bh);
	}
	spin_unlock_irqrestore(&dev->done.lock, flags);
	return HW_TLP_BUF_STATE_IDLE;
}
static void reset_tlp_tmp_buf(struct hw_cdc_net *dev)
{
	dev->hw_tlp_tmp_buf.bytesneeded = 0;
	dev->hw_tlp_tmp_buf.pktlength = 0;
}
static void rx_tlp_parse(struct hw_cdc_net *dev, struct sk_buff *skb)
{
	struct hw_cdc_tlp *tlp = NULL;
	int remain_bytes = (int)skb->len;
	unsigned short pktlen = 0;
	unsigned char *cur_ptr = skb->data;
	unsigned char *payload_ptr = NULL;
	unsigned char *buf_start = skb->data;
	unsigned char *buf_end = buf_start + skb->len;

	/*decoding the TLP packets into the ether packet*/
	while (remain_bytes > 0){
		switch (dev->hw_tlp_buffer_state){
			case HW_TLP_BUF_STATE_IDLE:
			{
				if (HW_TLP_HDR_LENGTH < remain_bytes ){
					tlp = (struct hw_cdc_tlp *)cur_ptr;
					pktlen = (tlp->pktlength & HW_TLP_MASK_LENGTH);
					payload_ptr = (unsigned char *)&(tlp->payload);

					//validate the tlp packet header
					if (HW_TLP_BITS_SYNC != (tlp->pktlength & HW_TLP_MASK_SYNC)){
						devdbg(dev, "The pktlength is error");
						dev->hw_tlp_buffer_state = HW_TLP_BUF_STATE_ERROR;
						break;
					}
					/*The receiced buffer has the whole ether packet */
					if ( (payload_ptr + pktlen) <= buf_end){
						/*Get the ether packet from the TLP packet, and put it into the done queue*/
						submit_skb(dev, payload_ptr, pktlen);
						cur_ptr = payload_ptr + pktlen;
						remain_bytes = buf_end - cur_ptr;
					}else{/*has the part of the ether packet*/
						if (pktlen > dev->rx_urb_size){
							devdbg(dev, "The pktlen is invalid");
							dev->hw_tlp_buffer_state = HW_TLP_BUF_STATE_ERROR;
							break;
						}
						dev->hw_tlp_tmp_buf.bytesneeded = (payload_ptr + pktlen) - buf_end;
						dev->hw_tlp_tmp_buf.pktlength = buf_end - payload_ptr;
						memcpy(dev->hw_tlp_tmp_buf.buffer, payload_ptr, dev->hw_tlp_tmp_buf.pktlength);
						dev->hw_tlp_buffer_state = HW_TLP_BUF_STATE_PARTIAL_FILL;
						remain_bytes = 0;
					}
				}
				else if (HW_TLP_HDR_LENGTH == remain_bytes){
					memcpy(dev->hw_tlp_tmp_buf.buffer, cur_ptr, remain_bytes);
					dev->hw_tlp_tmp_buf.bytesneeded = 0;
					dev->hw_tlp_tmp_buf.pktlength = remain_bytes;
					dev->hw_tlp_buffer_state = HW_TLP_BUF_STATE_HDR_ONLY;
					remain_bytes = 0;
				}
				else if (remain_bytes > 0){
					memcpy(dev->hw_tlp_tmp_buf.buffer, cur_ptr, remain_bytes);
					dev->hw_tlp_tmp_buf.bytesneeded = HW_TLP_HDR_LENGTH - remain_bytes;
					dev->hw_tlp_tmp_buf.pktlength = remain_bytes;
					dev->hw_tlp_buffer_state = HW_TLP_BUF_STATE_PARTIAL_HDR;
					remain_bytes = 0;
				}
				else{
					dev->hw_tlp_buffer_state = HW_TLP_BUF_STATE_ERROR;
				}
				break;
			}
			case HW_TLP_BUF_STATE_HDR_ONLY:
			{
				tlp->pktlength = *((unsigned short*)dev->hw_tlp_tmp_buf.buffer);
				pktlen = (tlp->pktlength & HW_TLP_MASK_LENGTH);
			    payload_ptr = cur_ptr;
				reset_tlp_tmp_buf(dev);
				/*validate the tlp packet header*/
				if (HW_TLP_BITS_SYNC != (tlp->pktlength & HW_TLP_MASK_SYNC)){
					devdbg(dev, "The pktlength is error");
					dev->hw_tlp_buffer_state = HW_TLP_BUF_STATE_ERROR;
					break;
				}
				if ( (payload_ptr + pktlen) <= buf_end){
					submit_skb(dev, payload_ptr, pktlen);
					cur_ptr = payload_ptr + pktlen;
					remain_bytes = buf_end - cur_ptr;
					dev->hw_tlp_buffer_state = HW_TLP_BUF_STATE_IDLE;
				}else{
					if (pktlen > dev->rx_urb_size){
						dev->hw_tlp_buffer_state = HW_TLP_BUF_STATE_ERROR;
						break;
					}
					dev->hw_tlp_tmp_buf.bytesneeded = (payload_ptr + pktlen) - buf_end;
					dev->hw_tlp_tmp_buf.pktlength = buf_end - payload_ptr;
					memcpy(dev->hw_tlp_tmp_buf.buffer, payload_ptr, dev->hw_tlp_tmp_buf.pktlength);
					dev->hw_tlp_buffer_state = HW_TLP_BUF_STATE_PARTIAL_FILL;
					remain_bytes = 0;
				}
				break;
			}
			case HW_TLP_BUF_STATE_PARTIAL_HDR:
			{
				memcpy(dev->hw_tlp_tmp_buf.buffer + dev->hw_tlp_tmp_buf.pktlength, cur_ptr,
					dev->hw_tlp_tmp_buf.bytesneeded);
				cur_ptr += dev->hw_tlp_tmp_buf.bytesneeded;
				dev->hw_tlp_buffer_state = HW_TLP_BUF_STATE_HDR_ONLY;
				remain_bytes -= dev->hw_tlp_tmp_buf.bytesneeded;
				break;
			}
			case HW_TLP_BUF_STATE_PARTIAL_FILL:
			{
				if (remain_bytes < dev->hw_tlp_tmp_buf.bytesneeded){
					memcpy(dev->hw_tlp_tmp_buf.buffer + dev->hw_tlp_tmp_buf.pktlength, cur_ptr,
						remain_bytes);
					dev->hw_tlp_tmp_buf.pktlength += remain_bytes;
					dev->hw_tlp_tmp_buf.bytesneeded -= remain_bytes;
					dev->hw_tlp_buffer_state = HW_TLP_BUF_STATE_PARTIAL_FILL;
					cur_ptr += remain_bytes;
					remain_bytes = 0;
				}else{
					unsigned short tmplen = dev->hw_tlp_tmp_buf.bytesneeded + dev->hw_tlp_tmp_buf.pktlength;
					if (HW_USB_RECEIVE_BUFFER_SIZE < tmplen){
						unsigned char *ptr = NULL;
						devdbg(dev, "The tlp length is larger than 1600");
						ptr = (unsigned char *)kmalloc(dev->hw_tlp_tmp_buf.bytesneeded + dev->hw_tlp_tmp_buf.pktlength
									, GFP_KERNEL);
						if (NULL != ptr){
							memcpy(ptr, dev->hw_tlp_tmp_buf.buffer, dev->hw_tlp_tmp_buf.pktlength);
							memcpy(ptr + dev->hw_tlp_tmp_buf.pktlength, cur_ptr,
								dev->hw_tlp_tmp_buf.bytesneeded);
							submit_skb(dev, ptr, tmplen);
							kfree(ptr);
						}

					}else{
						memcpy(dev->hw_tlp_tmp_buf.buffer + dev->hw_tlp_tmp_buf.pktlength,
							cur_ptr, dev->hw_tlp_tmp_buf.bytesneeded);
						submit_skb(dev, dev->hw_tlp_tmp_buf.buffer, tmplen);
					}
					remain_bytes -= dev->hw_tlp_tmp_buf.bytesneeded;
					cur_ptr += dev->hw_tlp_tmp_buf.bytesneeded;
					dev->hw_tlp_buffer_state = HW_TLP_BUF_STATE_IDLE;
					reset_tlp_tmp_buf(dev);
				}
				break;
			}
			case HW_TLP_BUF_STATE_ERROR:
			default:
			{
				remain_bytes = 0;
				reset_tlp_tmp_buf(dev);
				dev->hw_tlp_buffer_state = HW_TLP_BUF_STATE_IDLE;
				break;
			}
		}
	}
}

static void rx_defer_bh(struct hw_cdc_net *dev, struct sk_buff *skb, struct sk_buff_head *list)
{
	unsigned long		flags;
	spin_lock_irqsave(&list->lock, flags);
	__skb_unlink(skb, list);
	spin_unlock_irqrestore(&list->lock, flags);

	/*deal with the download tlp feature*/
	if (1 == dev->hw_tlp_download_is_actived){
		rx_tlp_parse(dev, skb);
		dev_kfree_skb_any(skb);
	}else{
		spin_lock_irqsave(&dev->done.lock, flags);
		__skb_queue_tail(&dev->done, skb);
		if (1 <= dev->done.qlen){
			tasklet_schedule(&dev->bh);
		}
		spin_unlock_irqrestore(&dev->done.lock, flags);
	}
}
////////////////////////

/* some work can't be done in tasklets, so we use keventd
 *
 * NOTE:  annoying asymmetry:  if it's active, schedule_work() fails,
 * but tasklet_schedule() doesn't.  hope the failure is rare.
 */
void hw_defer_kevent (struct hw_cdc_net *dev, int work)
{
	set_bit (work, &dev->flags);
	if (!schedule_work (&dev->kevent))
		deverr (dev, "kevent %d may have been dropped", work);
	else
		devdbg (dev, "kevent %d scheduled", work);
}
EXPORT_SYMBOL_GPL(hw_defer_kevent);

/*-------------------------------------------------------------------------*/




static void rx_complete (struct urb *urb);
static void rx_submit (struct hw_cdc_net *dev, struct urb *urb, gfp_t flags)
{
	struct sk_buff		*skb;
	struct skb_data		*entry;
	int			retval = 0;
	unsigned long		lockflags;
	size_t			size = dev->rx_urb_size;

	if ((skb = alloc_skb (size + NET_IP_ALIGN, flags)) == NULL) {
		if (netif_msg_rx_err (dev))
			devdbg (dev, "no rx skb");
		hw_defer_kevent (dev, EVENT_RX_MEMORY);
		usb_free_urb (urb);
		return;
	}
	skb_reserve (skb, NET_IP_ALIGN);

	entry = (struct skb_data *) skb->cb;
	entry->urb = urb;
	entry->dev = dev;
	entry->state = rx_start;
	entry->length = 0;

	usb_fill_bulk_urb (urb, dev->udev, dev->in,
		skb->data, size, rx_complete, skb);

	spin_lock_irqsave (&dev->rxq.lock, lockflags);

	if (netif_running (dev->net)
			&& netif_device_present (dev->net)
			&& !test_bit (EVENT_RX_HALT, &dev->flags)) {
		switch (retval = usb_submit_urb (urb, GFP_ATOMIC)) {
		case 0://submit successfully
			__skb_queue_tail (&dev->rxq, skb);
			break;
		case -EPIPE:
			hw_defer_kevent (dev, EVENT_RX_HALT);
			break;
		case -ENOMEM:
			hw_defer_kevent (dev, EVENT_RX_MEMORY);
			break;
		case -ENODEV:
			if (netif_msg_ifdown (dev))
				devdbg (dev, "device gone");
			netif_device_detach (dev->net);
			break;
		default:
			if (netif_msg_rx_err (dev))
				devdbg (dev, "rx submit, %d", retval);
			tasklet_schedule (&dev->bh);
			break;
		}
	} else {
		if (netif_msg_ifdown (dev))
			devdbg (dev, "rx: stopped");
		retval = -ENOLINK;
	}
	spin_unlock_irqrestore (&dev->rxq.lock, lockflags);
	if (retval) {
		dev_kfree_skb_any (skb);
		usb_free_urb (urb);
	}
}

/*-------------------------------------------------------------------------*/

static inline void rx_process (struct hw_cdc_net *dev, struct sk_buff *skb)
{
	if (skb->len){
		hw_skb_return (dev, skb);
	}
	else {
		if (netif_msg_rx_err (dev))
			devdbg (dev, "drop");

		dev->stats.rx_errors++;
		skb_queue_tail (&dev->done, skb);
	}
}

/*-------------------------------------------------------------------------*/
static void rx_complete (struct urb *urb)
{
	struct sk_buff		*skb = (struct sk_buff *) urb->context;
	struct skb_data		*entry = (struct skb_data *) skb->cb;
	struct hw_cdc_net		*dev = entry->dev;
	int			urb_status = urb->status;

	skb_put (skb, urb->actual_length);
	entry->state = rx_done;
	entry->urb = NULL;

	switch (urb_status) {
	/* success */
	case 0:
		if (skb->len < dev->net->hard_header_len) {
			entry->state = rx_cleanup;
			dev->stats.rx_errors++;
			dev->stats.rx_length_errors++;
			if (netif_msg_rx_err (dev))
				devdbg (dev, "rx length %d", skb->len);
		}
		break;

	/* stalls need manual reset. this is rare ... except that
	 * when going through USB 2.0 TTs, unplug appears this way.
	 * we avoid the highspeed version of the ETIMEOUT/EILSEQ
	 * storm, recovering as needed.
	 */
	case -EPIPE:
		dev->stats.rx_errors++;
		hw_defer_kevent (dev, EVENT_RX_HALT);
		// FALLTHROUGH

	/* software-driven interface shutdown */
	case -ECONNRESET:		/* async unlink */
	case -ESHUTDOWN:		/* hardware gone */
		if (netif_msg_ifdown (dev))
			devdbg (dev, "rx shutdown, code %d", urb_status);
		goto block;

	/* we get controller i/o faults during khubd disconnect() delays.
	 * throttle down resubmits, to avoid log floods; just temporarily,
	 * so we still recover when the fault isn't a khubd delay.
	 */
	case -EPROTO:
	case -ETIME:
	case -EILSEQ:
		dev->stats.rx_errors++;
		if (!timer_pending (&dev->delay)) {
			mod_timer (&dev->delay, jiffies + THROTTLE_JIFFIES);
			if (netif_msg_link (dev))
				devdbg (dev, "rx throttle %d", urb_status);
		}
block:
		entry->state = rx_cleanup;
		entry->urb = urb;
		urb = NULL;
		break;

	/* data overrun ... flush fifo? */
	case -EOVERFLOW:
		dev->stats.rx_over_errors++;
		// FALLTHROUGH

	default:
		entry->state = rx_cleanup;
		dev->stats.rx_errors++;
		if (netif_msg_rx_err (dev))
			devdbg (dev, "rx status %d", urb_status);
		break;
	}

	rx_defer_bh(dev, skb, &dev->rxq);

	if (urb) {
		if (netif_running (dev->net)
				&& !test_bit (EVENT_RX_HALT, &dev->flags)) {
			rx_submit (dev, urb, GFP_ATOMIC);
			return;
		}
		usb_free_urb (urb);
	}
	if (netif_msg_rx_err (dev))
		devdbg (dev, "no read resubmitted");
}
static void intr_complete (struct urb *urb)
{
	struct hw_cdc_net	*dev = urb->context;
	int		status = urb->status;
	switch (status) {
	/* success */
	case 0:
		hw_cdc_status(dev, urb);
		break;

	/* software-driven interface shutdown */
	case -ENOENT:		/* urb killed */
	case -ESHUTDOWN:	/* hardware gone */
		if (netif_msg_ifdown (dev))
			devdbg (dev, "intr shutdown, code %d", status);
		return;

	/* NOTE:  not throttling like RX/TX, since this endpoint
	 * already polls infrequently
	 */
	default:
		devdbg (dev, "intr status %d", status);
		break;
	}

	if (!netif_running (dev->net))
		return;

	memset(urb->transfer_buffer, 0, urb->transfer_buffer_length);
	status = usb_submit_urb (urb, GFP_ATOMIC);
	if (status != 0 && netif_msg_timer (dev))
		deverr(dev, "intr resubmit --> %d", status);
}

/*-------------------------------------------------------------------------*/




/*-------------------------------------------------------------------------*/

// precondition: never called in_interrupt

static int hw_stop (struct net_device *net)
{
	struct hw_cdc_net		*dev = netdev_priv(net);
	int			temp;
	DECLARE_WAIT_QUEUE_HEAD_ONSTACK (unlink_wakeup);
	DECLARE_WAITQUEUE (wait, current);

	netif_stop_queue (net);

	if (netif_msg_ifdown (dev))
		devinfo (dev, "stop stats: rx/tx %ld/%ld, errs %ld/%ld",
			dev->stats.rx_packets, dev->stats.tx_packets,
			dev->stats.rx_errors, dev->stats.tx_errors
			);

	// ensure there are no more active urbs
	add_wait_queue (&unlink_wakeup, &wait);
	dev->wait = &unlink_wakeup;
	temp = unlink_urbs (dev, &dev->txq) + unlink_urbs (dev, &dev->rxq);

	// maybe wait for deletions to finish.
	while (!skb_queue_empty(&dev->rxq)
			&& !skb_queue_empty(&dev->txq)
			&& !skb_queue_empty(&dev->done)) {
		msleep(UNLINK_TIMEOUT_MS);
		if (netif_msg_ifdown (dev))
			devdbg (dev, "waited for %d urb completions", temp);
	}
	dev->wait = NULL;
	remove_wait_queue (&unlink_wakeup, &wait);

	/*cleanup the data for TLP*/
	dev->hw_tlp_buffer_state = HW_TLP_BUF_STATE_IDLE;
	if (NULL != dev->hw_tlp_tmp_buf.buffer){
		kfree(dev->hw_tlp_tmp_buf.buffer);
		dev->hw_tlp_tmp_buf.buffer = NULL;
	}
	dev->hw_tlp_tmp_buf.pktlength = 0;
	dev->hw_tlp_tmp_buf.bytesneeded = 0;

	usb_kill_urb(dev->interrupt);

	/* deferred work (task, timer, softirq) must also stop.
	 * can't flush_scheduled_work() until we drop rtnl (later),
	 * else workers could deadlock; so make workers a NOP.
	 */
	dev->flags = 0;
	del_timer_sync (&dev->delay);
	tasklet_kill (&dev->bh);
	usb_autopm_put_interface(dev->intf);

	return 0;
}

/*-------------------------------------------------------------------------*/

// posts reads, and enables write queuing

// precondition: never called in_interrupt

static int hw_open (struct net_device *net)
{
	struct hw_cdc_net		*dev = netdev_priv(net);
	int			retval;
	if ((retval = usb_autopm_get_interface(dev->intf)) < 0) {
		if (netif_msg_ifup (dev))
			devinfo (dev,
				"resumption fail (%d) hw_cdc_net usb-%s-%s, %s",
				retval,
				dev->udev->bus->bus_name, dev->udev->devpath,
			dev->driver_desc);
		goto done_nopm;
	}

	/*Initialized the data for TLP*/
	dev->hw_tlp_buffer_state = HW_TLP_BUF_STATE_IDLE;
	dev->hw_tlp_tmp_buf.buffer = kmalloc(HW_USB_RECEIVE_BUFFER_SIZE, GFP_KERNEL);
	if (NULL != dev->hw_tlp_tmp_buf.buffer){
		memset(dev->hw_tlp_tmp_buf.buffer, 0, HW_USB_RECEIVE_BUFFER_SIZE);
	}
	dev->hw_tlp_tmp_buf.pktlength = 0;
	dev->hw_tlp_tmp_buf.bytesneeded = 0;


	/* start any status interrupt transfer */
	if (dev->interrupt) {
		retval = usb_submit_urb (dev->interrupt, GFP_KERNEL);
		if (retval < 0) {
			if (netif_msg_ifup (dev))
				deverr (dev, "intr submit %d", retval);
			goto done;
		}
	}

	netif_start_queue (net);

	// delay posting reads until we're fully open
	tasklet_schedule (&dev->bh);
	return retval;
done:
	usb_autopm_put_interface(dev->intf);
done_nopm:
	return retval;
}

/*-------------------------------------------------------------------------*/

/* ethtool methods; minidrivers may need to add some more, but
 * they'll probably want to use this base set.
 */

int hw_get_settings (struct net_device *net, struct ethtool_cmd *cmd)
{
	struct hw_cdc_net *dev = netdev_priv(net);

	if (!dev->mii.mdio_read)
		return -EOPNOTSUPP;

	return mii_ethtool_gset(&dev->mii, cmd);
}
EXPORT_SYMBOL_GPL(hw_get_settings);

int hw_set_settings (struct net_device *net, struct ethtool_cmd *cmd)
{
	struct hw_cdc_net *dev = netdev_priv(net);
	int retval;

	if (!dev->mii.mdio_write)
		return -EOPNOTSUPP;

	retval = mii_ethtool_sset(&dev->mii, cmd);

	return retval;

}
EXPORT_SYMBOL_GPL(hw_set_settings);

u32 hw_get_link (struct net_device *net)
{
	struct hw_cdc_net *dev = netdev_priv(net);

	/* if the device has mii operations, use those */
	if (dev->mii.mdio_read)
		return mii_link_ok(&dev->mii);

	/* Otherwise, say we're up (to avoid breaking scripts) */
	return 1;
}
EXPORT_SYMBOL_GPL(hw_get_link);

int hw_nway_reset(struct net_device *net)
{
	struct hw_cdc_net *dev = netdev_priv(net);

	if (!dev->mii.mdio_write)
		return -EOPNOTSUPP;

	return mii_nway_restart(&dev->mii);
}
EXPORT_SYMBOL_GPL(hw_nway_reset);

void hw_get_drvinfo (struct net_device *net, struct ethtool_drvinfo *info)
{
	struct hw_cdc_net *dev = netdev_priv(net);

	strncpy (info->driver, dev->driver_name, sizeof info->driver);
	strncpy (info->version, DRIVER_VERSION, sizeof info->version);
	strncpy (info->fw_version, dev->driver_desc,
		sizeof info->fw_version);
	usb_make_path (dev->udev, info->bus_info, sizeof info->bus_info);
}
EXPORT_SYMBOL_GPL(hw_get_drvinfo);

u32 hw_get_msglevel (struct net_device *net)
{
	struct hw_cdc_net *dev = netdev_priv(net);

	return dev->msg_enable;
}
EXPORT_SYMBOL_GPL(hw_get_msglevel);

void hw_set_msglevel (struct net_device *net, u32 level)
{
	struct hw_cdc_net *dev = netdev_priv(net);

	dev->msg_enable = level;
}
EXPORT_SYMBOL_GPL(hw_set_msglevel);

/* drivers may override default ethtool_ops in their bind() routine */
static struct ethtool_ops hw_ethtool_ops = {
	.get_settings		= hw_get_settings,
	.set_settings		= hw_set_settings,
	.get_link		= hw_get_link,
	.nway_reset		= hw_nway_reset,
	.get_drvinfo		= hw_get_drvinfo,
	.get_msglevel		= hw_get_msglevel,
	.set_msglevel		= hw_set_msglevel,
};

/*-------------------------------------------------------------------------*/

/* work that cannot be done in interrupt context uses keventd.
 *
 * NOTE:  with 2.5 we could do more of this using completion callbacks,
 * especially now that control transfers can be queued.
 */
static void
kevent (struct work_struct *work)
{
	struct hw_cdc_net		*dev =
		container_of(work, struct hw_cdc_net, kevent);
	int			status;

	/* usb_clear_halt() needs a thread context */
	if (test_bit (EVENT_TX_HALT, &dev->flags)) {
		unlink_urbs (dev, &dev->txq);
		status = usb_clear_halt (dev->udev, dev->out);
		if (status < 0
				&& status != -EPIPE
				&& status != -ESHUTDOWN) {
			if (netif_msg_tx_err (dev))
				deverr (dev, "can't clear tx halt, status %d",
					status);
		} else {
			clear_bit (EVENT_TX_HALT, &dev->flags);
			if (status != -ESHUTDOWN)
				netif_wake_queue (dev->net);
		}
	}
	if (test_bit (EVENT_RX_HALT, &dev->flags)) {
		unlink_urbs (dev, &dev->rxq);
		status = usb_clear_halt (dev->udev, dev->in);
		if (status < 0
				&& status != -EPIPE
				&& status != -ESHUTDOWN) {
			if (netif_msg_rx_err (dev))
				deverr (dev, "can't clear rx halt, status %d",
					status);
		} else {
			clear_bit (EVENT_RX_HALT, &dev->flags);
			tasklet_schedule (&dev->bh);
		}
	}

	/* tasklet could resubmit itself forever if memory is tight */
	if (test_bit (EVENT_RX_MEMORY, &dev->flags)) {
		struct urb	*urb = NULL;

		if (netif_running (dev->net))
			urb = usb_alloc_urb (0, GFP_KERNEL);
		else
			clear_bit (EVENT_RX_MEMORY, &dev->flags);
		if (urb != NULL) {
			clear_bit (EVENT_RX_MEMORY, &dev->flags);
			rx_submit (dev, urb, GFP_KERNEL);
			tasklet_schedule (&dev->bh);
		}
	}

	if (test_bit (EVENT_LINK_RESET, &dev->flags)) {
		clear_bit (EVENT_LINK_RESET, &dev->flags);
	}

	if (dev->flags)
		devdbg (dev, "kevent done, flags = 0x%lx",
			dev->flags);
}

/*-------------------------------------------------------------------------*/

static void tx_complete (struct urb *urb)
{
	struct sk_buff		*skb = (struct sk_buff *) urb->context;
	struct skb_data		*entry = (struct skb_data *) skb->cb;
	struct hw_cdc_net		*dev = entry->dev;

	if (urb->status == 0) {
		dev->stats.tx_packets++;
		dev->stats.tx_bytes += entry->length;
	} else {
		dev->stats.tx_errors++;

		switch (urb->status) {
		case -EPIPE:
			hw_defer_kevent (dev, EVENT_TX_HALT);
			break;

		/* software-driven interface shutdown */
		case -ECONNRESET:		// async unlink
		case -ESHUTDOWN:		// hardware gone
			break;

		// like rx, tx gets controller i/o faults during khubd delays
		// and so it uses the same throttling mechanism.
		case -EPROTO:
		case -ETIME:
		case -EILSEQ:
			if (!timer_pending (&dev->delay)) {
				mod_timer (&dev->delay,
					jiffies + THROTTLE_JIFFIES);
				if (netif_msg_link (dev))
					devdbg (dev, "tx throttle %d",
							urb->status);
			}
			netif_stop_queue (dev->net);
			break;
		default:
			if (netif_msg_tx_err (dev))
				devdbg (dev, "tx err %d", entry->urb->status);
			break;
		}
	}

	urb->dev = NULL;
	entry->state = tx_done;
	tx_defer_bh(dev, skb, &dev->txq);
}

/*-------------------------------------------------------------------------*/

static void hw_tx_timeout (struct net_device *net)
{
	struct hw_cdc_net		*dev = netdev_priv(net);

	unlink_urbs (dev, &dev->txq);
	tasklet_schedule (&dev->bh);

	// FIXME: device recovery -- reset?
}

/*-------------------------------------------------------------------------*/

static int hw_start_xmit (struct sk_buff *skb, struct net_device *net)
{
	struct hw_cdc_net		*dev = netdev_priv(net);
	int			length;
	int			retval = NET_XMIT_SUCCESS;
	struct urb		*urb = NULL;
	struct skb_data		*entry;
	unsigned long		flags;

	length = skb->len;

	if (!(urb = usb_alloc_urb (0, GFP_ATOMIC))) {
		if (netif_msg_tx_err (dev))
			devdbg (dev, "no urb");
		goto drop;
	}

	entry = (struct skb_data *) skb->cb;
	entry->urb = urb;
	entry->dev = dev;
	entry->state = tx_start;
	entry->length = length;

	usb_fill_bulk_urb (urb, dev->udev, dev->out,
			skb->data, skb->len, tx_complete, skb);

	/* don't assume the hardware handles USB_ZERO_PACKET
	 * NOTE:  strictly conforming cdc-ether devices should expect
	 * the ZLP here, but ignore the one-byte packet.
	 */
	if ((length % dev->maxpacket) == 0) {
		urb->transfer_buffer_length++;
		if (skb_tailroom(skb)) {
			skb->data[skb->len] = 0;
			__skb_put(skb, 1);
		}
	}

	spin_lock_irqsave (&dev->txq.lock, flags);

	switch ((retval = usb_submit_urb (urb, GFP_ATOMIC))) {
	case -EPIPE:
		netif_stop_queue (net);
		hw_defer_kevent (dev, EVENT_TX_HALT);
		break;
	default:
		if (netif_msg_tx_err (dev))
			devdbg (dev, "tx: submit urb err %d", retval);
		break;
	case 0:
		net->trans_start = jiffies;
		__skb_queue_tail (&dev->txq, skb);
		if (dev->txq.qlen >= TX_QLEN (dev))
			netif_stop_queue (net);
	}
	spin_unlock_irqrestore (&dev->txq.lock, flags);

	if (retval) {
		if (netif_msg_tx_err (dev))
			devdbg (dev, "drop, code %d", retval);
drop:
		retval = NET_XMIT_SUCCESS;
		dev->stats.tx_dropped++;
		if (skb)
			dev_kfree_skb_any (skb);
		usb_free_urb (urb);
	} else if (netif_msg_tx_queued (dev)) {
		devdbg (dev, "> tx, len %d, type 0x%x",
			length, skb->protocol);
	}
	return retval;
}


/*-------------------------------------------------------------------------*/

// tasklet (work deferred from completions, in_irq) or timer

static void hw_bh (unsigned long param)
{
	struct hw_cdc_net		*dev = (struct hw_cdc_net *) param;
	struct sk_buff		*skb;
	struct skb_data		*entry;

	while ((skb = skb_dequeue (&dev->done))) {
		entry = (struct skb_data *) skb->cb;
		switch (entry->state) {
		case rx_done:
			entry->state = rx_cleanup;
			rx_process (dev, skb);
			continue;
		case tx_done:
		case rx_cleanup:
			usb_free_urb (entry->urb);
			dev_kfree_skb (skb);
			continue;
		default:
			devdbg (dev, "bogus skb state %d", entry->state);
		}
	}

	// waiting for all pending urbs to complete?
	if (dev->wait) {
		if ((dev->txq.qlen + dev->rxq.qlen + dev->done.qlen) == 0) {
			wake_up (dev->wait);
		}

	// or are we maybe short a few urbs?
	} else if (netif_running (dev->net)
			&& netif_device_present (dev->net)
			&& !timer_pending (&dev->delay)
			&& !test_bit (EVENT_RX_HALT, &dev->flags)) {
		int	temp = dev->rxq.qlen;
		int	qlen = RX_QLEN (dev);

		if (temp < qlen) {
			struct urb	*urb;
			int		i;

			// don't refill the queue all at once
			for (i = 0; i < 10 && dev->rxq.qlen < qlen; i++) {
				urb = usb_alloc_urb (0, GFP_ATOMIC);
				if (urb != NULL)
					rx_submit (dev, urb, GFP_ATOMIC);
			}
			if (temp != dev->rxq.qlen && netif_msg_link (dev))
				devdbg (dev, "rxqlen %d --> %d",
						temp, dev->rxq.qlen);
			if (dev->rxq.qlen < qlen)
				tasklet_schedule (&dev->bh);
		}
		if (dev->txq.qlen < TX_QLEN (dev))
			netif_wake_queue (dev->net);
	}
}



/*-------------------------------------------------------------------------
 *
 * USB Device Driver support
 *
 *-------------------------------------------------------------------------*/

// precondition: never called in_interrupt

void hw_disconnect (struct usb_interface *intf)
{
	struct hw_cdc_net		*dev;
	struct usb_device	*xdev;
	struct net_device	*net;

	dev = usb_get_intfdata(intf);
	usb_set_intfdata(intf, NULL);
	if (!dev)
		return;

	xdev = interface_to_usbdev (intf);

	if (netif_msg_probe (dev))
		devinfo (dev, "unregister '%s' usb-%s-%s, %s",
			intf->dev.driver->name,
			xdev->bus->bus_name, xdev->devpath,
			dev->driver_desc);

	cancel_delayed_work_sync(&dev->status_work);//Added by fangxz 2010-3-26

	net = dev->net;
	unregister_netdev (net);

	/* we don't hold rtnl here ... */
	flush_scheduled_work ();

	hw_cdc_unbind(dev, intf);

	free_netdev(net);
	usb_put_dev (xdev);
}
EXPORT_SYMBOL_GPL(hw_disconnect);


/*-------------------------------------------------------------------------*/
#if !(LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 30))
static int hw_eth_mac_addr(struct net_device *dev, void *p)
{
	/*Begin: Change the MAC address to fix device 20101226*/
	dev->dev_addr[0] = 0x00;
	dev->dev_addr[1] = 0x01;
	dev->dev_addr[2] = 0x02;
	dev->dev_addr[3] = 0x03;
	dev->dev_addr[4] = 0x04;
	dev->dev_addr[5] = 0x05;
	/*End: Change the MAC address to fix device 20101226*/
	return 0;
}
static const struct net_device_ops hw_netdev_ops = {
	.ndo_open = hw_open,
	.ndo_stop = hw_stop,
	.ndo_start_xmit = hw_start_xmit,
	.ndo_tx_timeout = hw_tx_timeout,
	.ndo_change_mtu = hw_change_mtu,
	.ndo_set_mac_address = hw_eth_mac_addr,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_get_stats = hw_get_stats,
};
#endif

int hw_send_tlp_download_request(struct usb_interface *intf);
// precondition: never called in_interrupt
int hw_check_conn_status(struct usb_interface *intf);

int
hw_cdc_probe (struct usb_interface *udev, const struct usb_device_id *prod)
{
	struct hw_cdc_net			*dev;
	struct net_device		*net;
	struct usb_host_interface	*interface;
	struct usb_device		*xdev;
	int				status;
	const char			*name;
//	DECLARE_MAC_BUF(mac);

	name = udev->dev.driver->name;
	xdev = interface_to_usbdev (udev);
	interface = udev->cur_altsetting;

	usb_get_dev (xdev);

	status = -ENOMEM;

	// set up our own records
	net = alloc_etherdev(sizeof(*dev));
	if (!net) {
		dbg ("can't kmalloc dev");
		goto out;
	}

	dev = netdev_priv(net);
	dev->udev = xdev;
	dev->intf = udev;
	dev->driver_name = name;
	dev->driver_desc = "Huawei Ethernet Device";
	dev->msg_enable = netif_msg_init (msg_level, NETIF_MSG_DRV
				| NETIF_MSG_PROBE | NETIF_MSG_LINK);
	skb_queue_head_init (&dev->rxq);
	skb_queue_head_init (&dev->txq);
	skb_queue_head_init (&dev->done);
	dev->bh.func = hw_bh;
	dev->bh.data = (unsigned long) dev;
	INIT_WORK (&dev->kevent, kevent);
	dev->delay.function = hw_bh;
	dev->delay.data = (unsigned long) dev;
	init_timer (&dev->delay);
	mutex_init (&dev->phy_mutex);

	dev->net = net;
	memcpy (net->dev_addr, node_id, sizeof node_id);

	/* rx and tx sides can use different message sizes;
	 * bind() should set rx_urb_size in that case.
	 */
	dev->hard_mtu = net->mtu + net->hard_header_len;

#if !(LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 30))
	net->netdev_ops = &hw_netdev_ops;
#else
	net->change_mtu = hw_change_mtu;
	net->get_stats = hw_get_stats;
	net->hard_start_xmit = hw_start_xmit;
	net->open = hw_open;
	net->stop = hw_stop;
	net->tx_timeout = hw_tx_timeout;
#endif
	net->watchdog_timeo = TX_TIMEOUT_JIFFIES;
	net->ethtool_ops = &hw_ethtool_ops;


	status = hw_cdc_bind (dev, udev);
	if (status < 0)
		goto out1;

	strcpy (net->name, "wwan%d");
	/* maybe the remote can't receive an Ethernet MTU */
	if (net->mtu > (dev->hard_mtu - net->hard_header_len))
		net->mtu = dev->hard_mtu - net->hard_header_len;

	if (status >= 0 && dev->status)
		status = init_status (dev, udev);
	if (status < 0)
		goto out3;

	if (!dev->rx_urb_size)
		dev->rx_urb_size = dev->hard_mtu;
	dev->maxpacket = usb_maxpacket (dev->udev, dev->out, 1);

	SET_NETDEV_DEV(net, &udev->dev);
	status = register_netdev (net);
	if (status)
		goto out3;
	// ok, it's ready to go.
	usb_set_intfdata (udev, dev);

	/*activate the download tlp feature*/
	if (0 < hw_send_tlp_download_request(udev)){
		devdbg(dev, "%s: The tlp is activated", __FUNCTION__);
		dev->hw_tlp_download_is_actived = 1;//activated successfully
	}else{
		dev->hw_tlp_download_is_actived = 0;//activated failed
	}

	netif_device_attach (net);

	//kernel_thread(hw_check_conn_status, (void *)net, 0);

	/*Begin: Set the carrier on as default 20101222*/
	netif_carrier_on(net);
	/*netif_carrier_off(net);
	if (HW_JUNGO_BCDDEVICE_VALUE != dev->udev->descriptor.bcdDevice) {
		dev->qmi_sync = 0;
		INIT_DELAYED_WORK(&dev->status_work, hw_cdc_check_status_work);
    		schedule_delayed_work(&dev->status_work, 10*HZ);
	}*/
	//hw_check_conn_status(udev);
	//
	/*End: Set the carrier on as default 20101222*/
	return 0;

out3:
	hw_cdc_unbind (dev, udev);
out1:
	free_netdev(net);
out:
	usb_put_dev(xdev);
	return status;
}
EXPORT_SYMBOL_GPL(hw_cdc_probe);

/*-------------------------------------------------------------------------*/

/*
 * suspend the whole driver as soon as the first interface is suspended
 * resume only when the last interface is resumed
 */

int hw_suspend (struct usb_interface *intf, pm_message_t message)
{
	struct hw_cdc_net		*dev = usb_get_intfdata(intf);

	if (!dev->suspend_count++) {
		/*
		 * accelerate emptying of the rx and queues, to avoid
		 * having everything error out.
		 */
		netif_device_detach (dev->net);
		(void) unlink_urbs (dev, &dev->rxq);
		(void) unlink_urbs (dev, &dev->txq);
		/*
		 * reattach so runtime management can use and
		 * wake the device
		 */
		netif_device_attach (dev->net);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(hw_suspend);

int hw_resume (struct usb_interface *intf)
{
	struct hw_cdc_net		*dev = usb_get_intfdata(intf);

	if (!--dev->suspend_count)
		tasklet_schedule (&dev->bh);

	return 0;
}
EXPORT_SYMBOL_GPL(hw_resume);

static int hw_cdc_reset_resume(struct usb_interface *intf)
{
	return hw_resume (intf);
}

int hw_send_tlp_download_request(struct usb_interface *intf)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct usb_host_interface *interface = intf->cur_altsetting;
	struct usbdevfs_ctrltransfer req = {0};
	unsigned char buf[256] = {0};
	int retval = 0;
	req.bRequestType = 0xC0;
	req.bRequest = 0x02;//activating the download tlp feature request
	req.wIndex = interface->desc.bInterfaceNumber;
	req.wValue = 1;
	req.wLength = 1;
	req.data = buf;
	req.timeout = 1000;
	retval = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0), req.bRequest,
		req.bRequestType, req.wValue, req.wIndex,
		req.data, req.wLength, req.timeout);
	/*check the TLP feature is activated or not, response value 0x01 indicates success*/
	if (0 < retval && 0x01 == buf[0]){
		return retval;
	}else{
		return 0;
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * probes control interface, claims data interface, collects the bulk
 * endpoints, activates data interface (if needed), maybe sets MTU.
 * all pure cdc
 */
//int hw_generic_cdc_bind(struct hw_cdc_net *dev, struct usb_interface *intf)
#define USB_DEVICE_HUAWEI_DATA 0xFF
static int hw_cdc_bind(struct hw_cdc_net *dev, struct usb_interface *intf)
{
	u8				*buf = intf->cur_altsetting->extra;
	int				len = intf->cur_altsetting->extralen;
	struct usb_interface_descriptor	*d;
	struct hw_dev_state		*info = (void *) &dev->data;
	int				status;
	struct usb_driver		*driver = driver_of(intf);

	if (sizeof dev->data < sizeof *info)
		return -EDOM;



	memset(info, 0, sizeof *info);
	info->control = intf;
	while (len > 3) {
		if (buf [1] != USB_DT_CS_INTERFACE)
			goto next_desc;

		switch (buf [2]) {
		case USB_CDC_HEADER_TYPE:
			if (info->header) {
				dev_dbg(&intf->dev, "extra CDC header\n");
				goto bad_desc;
			}
			info->header = (void *) buf;
			if (info->header->bLength != sizeof *info->header) {
				dev_dbg(&intf->dev, "CDC header len %u\n",
					info->header->bLength);
				goto bad_desc;
			}
			break;
		case USB_CDC_UNION_TYPE:
			if (info->u) {
				dev_dbg(&intf->dev, "extra CDC union\n");
				goto bad_desc;
			}
			info->u = (void *) buf;
			if (info->u->bLength != sizeof *info->u) {
				dev_dbg(&intf->dev, "CDC union len %u\n",
					info->u->bLength);
				goto bad_desc;
			}

			/* we need a master/control interface (what we're
			 * probed with) and a slave/data interface; union
			 * descriptors sort this all out.
			 */
			info->control = usb_ifnum_to_if(dev->udev,
						info->u->bMasterInterface0);
			info->data = usb_ifnum_to_if(dev->udev,
						info->u->bSlaveInterface0);
			if (!info->control || !info->data) {
				dev_dbg(&intf->dev,
					"master #%u/%p slave #%u/%p\n",
					info->u->bMasterInterface0,
					info->control,
					info->u->bSlaveInterface0,
					info->data);
				goto bad_desc;
			}
			if (info->control != intf) {
				dev_dbg(&intf->dev, "bogus CDC Union\n");
				/* Ambit USB Cable Modem (and maybe others)
				 * interchanges master and slave interface.
				 */
				if (info->data == intf) {
					info->data = info->control;
					info->control = intf;
				} else
					goto bad_desc;
			}

			/*For Jungo solution, the NDIS device has no data interface, so needn't detect data interface*/
			if (HW_JUNGO_BCDDEVICE_VALUE != dev->udev->descriptor.bcdDevice) {
				/* a data interface altsetting does the real i/o */
				d = &info->data->cur_altsetting->desc;
			//if (d->bInterfaceClass != USB_CLASS_CDC_DATA) { /*delete the standard CDC slave class detect*/
			if (d->bInterfaceClass !=  USB_DEVICE_HUAWEI_DATA && d->bInterfaceClass != USB_CLASS_CDC_DATA) {  /*Add to detect CDC slave class either Huawei defined or standard*/
					dev_dbg(&intf->dev, "slave class %u\n",
						d->bInterfaceClass);
					goto bad_desc;
				}
			}
			break;
		case USB_CDC_ETHERNET_TYPE:
			if (info->ether) {
				dev_dbg(&intf->dev, "extra CDC ether\n");
				goto bad_desc;
			}
			info->ether = (void *) buf;
			if (info->ether->bLength != sizeof *info->ether) {
				dev_dbg(&intf->dev, "CDC ether len %u\n",
					info->ether->bLength);
				goto bad_desc;
			}
			dev->hard_mtu = le16_to_cpu(
						info->ether->wMaxSegmentSize);
			/* because of Zaurus, we may be ignoring the host
			 * side link address we were given.
			 */
			break;
		}
next_desc:
		len -= buf [0];	/* bLength */
		buf += buf [0];
	}

	if (!info->header || !info->u || !info->ether) {
		dev_dbg(&intf->dev, "missing cdc %s%s%sdescriptor\n",
			info->header ? "" : "header ",
			info->u ? "" : "union ",
			info->ether ? "" : "ether ");
		goto bad_desc;
	}

	/*if the NDIS device is not Jungo solution, then assume that it has the data interface, and claim for it*/
	if (HW_JUNGO_BCDDEVICE_VALUE != dev->udev->descriptor.bcdDevice) {
		/* claim data interface and set it up ... with side effects.
	 	 * network traffic can't flow until an altsetting is enabled.
	 	 */
		status = usb_driver_claim_interface(driver, info->data, dev);
		if (status < 0)
			return status;
	}

	status = hw_get_endpoints(dev, info->data);
	if (status < 0) {
		/* ensure immediate exit from hw_disconnect */
		usb_set_intfdata(info->data, NULL);
		usb_driver_release_interface(driver, info->data);
		return status;
	}

	/* status endpoint: optional for CDC Ethernet, */
	dev->status = NULL;
	if (HW_JUNGO_BCDDEVICE_VALUE == dev->udev->descriptor.bcdDevice || info->control->cur_altsetting->desc.bNumEndpoints == 1) {
		struct usb_endpoint_descriptor	*desc;

		dev->status = &info->control->cur_altsetting->endpoint [0];
		desc = &dev->status->desc;
		if (((desc->bmAttributes  & USB_ENDPOINT_XFERTYPE_MASK) != USB_ENDPOINT_XFER_INT)
				|| ((desc->bEndpointAddress & USB_ENDPOINT_DIR_MASK) != USB_DIR_IN)
				|| (le16_to_cpu(desc->wMaxPacketSize)
					< sizeof(struct usb_cdc_notification))
				|| !desc->bInterval) {
			printk(KERN_ERR"fxz-%s:bad notification endpoint\n", __func__);
			dev->status = NULL;
		}
	}

	return hw_get_ethernet_addr(dev);

bad_desc:
	dev_info(&dev->udev->dev, "bad CDC descriptors\n");
	return -ENODEV;
}

void hw_cdc_unbind(struct hw_cdc_net *dev, struct usb_interface *intf)
{
	struct hw_dev_state		*info = (void *) &dev->data;
	struct usb_driver		*driver = driver_of(intf);

	/* disconnect master --> disconnect slave */
	if (intf == info->control && info->data) {
		/* ensure immediate exit from usbnet_disconnect */
		usb_set_intfdata(info->data, NULL);
		usb_driver_release_interface(driver, info->data);
		info->data = NULL;
	}

	/* and vice versa (just in case) */
	else if (intf == info->data && info->control) {
		/* ensure immediate exit from usbnet_disconnect */
		usb_set_intfdata(info->control, NULL);
		usb_driver_release_interface(driver, info->control);
		info->control = NULL;
	}
}
EXPORT_SYMBOL_GPL(hw_cdc_unbind);


/*-------------------------------------------------------------------------
 *
 * Communications Device Class, Ethernet Control model
 *
 * Takes two interfaces.  The DATA interface is inactive till an altsetting
 * is selected.  Configuration data includes class descriptors.  There's
 * an optional status endpoint on the control interface.
 *
 * This should interop with whatever the 2.4 "CDCEther.c" driver
 * (by Brad Hards) talked with, with more functionality.
 *
 *-------------------------------------------------------------------------*/

static void dumpspeed(struct hw_cdc_net *dev, __le32 *speeds)
{
	if (netif_msg_timer(dev))
		devinfo(dev, "link speeds: %u kbps up, %u kbps down",
			__le32_to_cpu(speeds[0]) / 1000,
		__le32_to_cpu(speeds[1]) / 1000);
}

static inline int hw_get_ethernet_addr(struct hw_cdc_net *dev)
{

	/*Begin: Change the MAC address to fix device 20101226*/
	dev->net->dev_addr[0] = 0x00;
	dev->net->dev_addr[1] = 0x01;
	dev->net->dev_addr[2] = 0x02;
	dev->net->dev_addr[3] = 0x03;
	dev->net->dev_addr[4] = 0x04;
	dev->net->dev_addr[5] = 0x05;
	/*End: Change the MAC address to fix device 20101226*/
	return 0;
}


enum {WRITE_REQUEST=0x21, READ_RESPONSE=0xa1};
#define HW_CDC_OK 0
#define HW_CDC_FAIL -1
/*-------------------------------------------------------------------------*/
/*The ioctl is called to send the qmi request to the device
  * or get the qmi response from the device*/
static int hw_cdc_ioctl (struct usb_interface *intf, unsigned int code,
			void *buf)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct hw_cdc_net *hwnet = (struct hw_cdc_net *)dev_get_drvdata(&intf->dev);
	struct usb_host_interface *interface = intf->cur_altsetting;
	struct usbdevfs_ctrltransfer *req = (struct usbdevfs_ctrltransfer *)buf;
	char *pbuf = NULL;
	int ret = -1;
	if (HW_JUNGO_BCDDEVICE_VALUE != hwnet->udev->descriptor.bcdDevice) {
		if (1 == hwnet->qmi_sync) {
			deverr(hwnet, "%s: The ndis port is busy.", __FUNCTION__);
			return HW_CDC_FAIL;
		}
	}

	if (USBDEVFS_CONTROL != code || NULL == req){
		deverr(hwnet, "%s: The request is not supported.", __FUNCTION__);
		return HW_CDC_FAIL;
	}

	if (0 < req->wLength){
		pbuf = (char *)kmalloc(req->wLength + 1, GFP_KERNEL);
		if (NULL == pbuf){
			deverr(hwnet, "%s: Kmalloc the buffer failed.", __FUNCTION__);
			return HW_CDC_FAIL;
		}
		memset(pbuf, 0, req->wLength);
	}

	switch (req->bRequestType)
	{
		case WRITE_REQUEST:
		{
			if (NULL != req->data && 0 < req->wLength){
				if (copy_from_user(pbuf, req->data, req->wLength)){
					deverr(hwnet, "usbnet_cdc_ioctl: copy_from_user failed");
					goto op_error;
				}

			}else{
				pbuf = NULL;
				req->wLength = 0;
			}
			pbuf[req->wLength] = 0;
			ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), req->bRequest,
					req->bRequestType, req->wValue, interface->desc.bInterfaceNumber,
					pbuf, req->wLength, req->timeout);
			break;
		}
		case READ_RESPONSE:
		{
			if (NULL == req->data || 0 >= req->wLength || NULL == pbuf){
				deverr(hwnet, "%s: The buffer is null, can not read the response.", __FUNCTION__);
				goto op_error;
			}
			ret = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0), req->bRequest, req->bRequestType, req->wValue, interface->desc.bInterfaceNumber, pbuf, req->wLength, req->timeout);

			if (0 < ret){
				if (HW_JUNGO_BCDDEVICE_VALUE != hwnet->udev->descriptor.bcdDevice) {
					/*check the connection indication*/
					if (0x04 == pbuf[6] && 0x22 == pbuf[9] && 0x00 == pbuf[10]){
						if (0x02 == pbuf[16]){
							if (hwnet){
								netif_carrier_on(hwnet->net);
							}
							}else{
								if (hwnet){
									netif_carrier_off(hwnet->net);
							}
						}
					}
				}
				if (copy_to_user(req->data, pbuf, req->wLength)){
					deverr(hwnet, "%s: copy_from_user failed", __FUNCTION__);
					goto op_error;
				}
			}
			break;
		}
		default:
			break;
	}

	if (NULL != pbuf){
		kfree(pbuf);
		pbuf = NULL;
	}

	return HW_CDC_OK;

op_error:
	if (NULL != pbuf){
		kfree(pbuf);
		pbuf = NULL;
	}
	return HW_CDC_FAIL;

}
#define	HUAWEI_ETHER_INTERFACE \
	.bInterfaceClass	= USB_CLASS_COMM, \
	.bInterfaceSubClass	= USB_CDC_SUBCLASS_ETHERNET, \
	.bInterfaceProtocol	= USB_CDC_PROTO_NONE


#define	HUAWEI_NDIS_INTERFACE \
	.bInterfaceClass	= USB_CLASS_COMM, \
	.bInterfaceSubClass	= USB_CDC_SUBCLASS_ETHERNET, \
	.bInterfaceProtocol	= 0xff


/*Add for PID optimized fangxz 20091105*/
#define	HUAWEI_NDIS_OPTIMIZED_INTERFACE \
	.bInterfaceClass	= 0xFF, \
	.bInterfaceSubClass	= 0x01, \
	.bInterfaceProtocol	= 0x09

/*Add for PID optimized marui 20100628*/
#define	HUAWEI_NDIS_OPTIMIZED_INTERFACE_VDF \
	.bInterfaceClass	= 0xFF, \
	.bInterfaceSubClass	= 0x01, \
	.bInterfaceProtocol	= 0x39

/*Add for PID optimized marui 20100811*/
#define	HUAWEI_NDIS_OPTIMIZED_INTERFACE_JUNGO \
	.bInterfaceClass	= 0xFF, \
	.bInterfaceSubClass	= 0x02, \
	.bInterfaceProtocol	= 0x07

/*Add for PID optimized marui 20100811*/
#define	HUAWEI_NDIS_OPTIMIZED_INTERFACE_VDF_JUNGO \
	.bInterfaceClass	= 0xFF, \
	.bInterfaceSubClass	= 0x02, \
	.bInterfaceProtocol	= 0x37


static const struct usb_device_id	hw_products [] = {
	{
		.match_flags	=   USB_DEVICE_ID_MATCH_INT_INFO
			  | USB_DEVICE_ID_MATCH_VENDOR,
		.idVendor		= 0x12d1,
		HUAWEI_ETHER_INTERFACE,
	},
	{
		.match_flags	=   USB_DEVICE_ID_MATCH_INT_INFO
			  | USB_DEVICE_ID_MATCH_VENDOR,
		.idVendor		= 0x12d1,
		HUAWEI_NDIS_INTERFACE,
	},
	/*Add for PID optimized fangxz 20091105*/
	{
		.match_flags	=   USB_DEVICE_ID_MATCH_INT_INFO
			  | USB_DEVICE_ID_MATCH_VENDOR,
		.idVendor		= 0x12d1,
		HUAWEI_NDIS_OPTIMIZED_INTERFACE,
	},
	/*Add for VDF PID optimized marui 20100628*/
	{
		.match_flags	=   USB_DEVICE_ID_MATCH_INT_INFO
			  | USB_DEVICE_ID_MATCH_VENDOR,
		.idVendor		= 0x12d1,
		HUAWEI_NDIS_OPTIMIZED_INTERFACE_VDF,
	},
	/*Add for PID optimized marui 20100811*/
	{
		.match_flags	=   USB_DEVICE_ID_MATCH_INT_INFO
			  | USB_DEVICE_ID_MATCH_VENDOR,
		.idVendor		= 0x12d1,
		HUAWEI_NDIS_OPTIMIZED_INTERFACE_JUNGO,
	},
	/*Add for VDF PID optimized marui 20100811*/
	{
		.match_flags	=   USB_DEVICE_ID_MATCH_INT_INFO
			  | USB_DEVICE_ID_MATCH_VENDOR,
		.idVendor		= 0x12d1,
		HUAWEI_NDIS_OPTIMIZED_INTERFACE_VDF_JUNGO,
	},
	{ },		// END
};
MODULE_DEVICE_TABLE(usb, hw_products);

static int hw_cdc_reset_resume(struct usb_interface *intf);
static struct usb_driver hw_ether_driver = {
	.name =		"huawei_ether",
	.id_table =	hw_products,
	.probe =	hw_cdc_probe,
	.disconnect =	hw_disconnect,
#if !(LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36))
	.unlocked_ioctl		=	hw_cdc_ioctl,
#else
        .ioctl = hw_cdc_ioctl,
#endif
	.suspend =	hw_suspend,
	.resume =	hw_resume,
	.reset_resume = hw_cdc_reset_resume,
};


static void hw_cdc_status(struct hw_cdc_net *dev, struct urb *urb)
{
	struct usb_cdc_notification	*event;

	if (urb->actual_length < sizeof *event)
		return;

	/* SPEED_CHANGE can get split into two 8-byte packets */
	if (test_and_clear_bit(EVENT_STS_SPLIT, &dev->flags)) {
		devdbg(dev, "The speed is changed by status event");
		dumpspeed(dev, (__le32 *) urb->transfer_buffer);
		return;
	}

	event = urb->transfer_buffer;
	switch (event->bNotificationType) {
	case USB_CDC_NOTIFY_NETWORK_CONNECTION:
		if (netif_msg_timer(dev))
			devdbg(dev, "CDC: carrier %s",
					event->wValue ? "on" : "off");
		if (event->wValue)
			netif_carrier_on(dev->net);
		else
			netif_carrier_off(dev->net);

		break;
	case USB_CDC_NOTIFY_SPEED_CHANGE:	/* tx/rx rates */
		if (netif_msg_timer(dev))
			devdbg(dev, "CDC: speed change (len %d)",
					urb->actual_length);
		if (urb->actual_length != (sizeof *event + 8))
			set_bit(EVENT_STS_SPLIT, &dev->flags);
		else
			dumpspeed(dev, (__le32 *) &event[1]);
		break;

	case USB_CDC_NOTIFY_RESPONSE_AVAILABLE:
	{
		break;
	}

	default:
		devdbg(dev, "%s: CDC: unexpected notification %02x!", __FUNCTION__,
				 event->bNotificationType);
		break;
	}
}


static int __init hw_cdc_init(void)
{
	BUG_ON((sizeof(((struct hw_cdc_net *)0)->data)
			< sizeof(struct hw_dev_state)));

 	return usb_register(&hw_ether_driver);
}
fs_initcall(hw_cdc_init);
#if 0
static int hw_send_qmi_request(struct usb_interface *intf,
				unsigned char *snd_req, int snd_len,
				unsigned char *read_resp, int resp_len);
static int hw_send_qmi_request_no_resp(struct usb_interface *intf,
				unsigned char *snd_req, int snd_len,
				unsigned char *read_resp, int resp_len);


//int hw_check_conn_status(struct usb_interface *intf)

static void hw_cdc_check_status_work(struct work_struct *work)

{
	//struct hw_cdc_net *net = usb_get_intfdata(intf);
	//usb_device *udev = interface_to_usbdev(intf);
	struct hw_cdc_net *dev = container_of(work, struct hw_cdc_net, status_work.work);

	int ret;
	int repeat = 0;
	unsigned char resp_buf[56] = {0};
	unsigned char client_id_req[0x10] = {0x01, 0x0f, 0x00, 0x00, 0x00,
										0x00, 0x00, 0x06, 0x22, 0x00,
										0x04, 0x00, 0x01, 0x01, 0x00, 0x01};
	unsigned char rel_client_id_req[0x11] = {0x01, 0x10, 0x00, 0x00, 0x00,
										0x00,  0x00, 0x00, 0x23,0x00,
										0x05, 0x00, 0x01, 0x02, 0x00,
										0x01, 0x00};
	unsigned char status_req[13] = {0x01, 0x0c, 0x00, 0x00, 0x01,
					0x00,  0x00, 0x02, 0x00,
					0x22, 0x00, 0x00, 0x00};
	unsigned char set_instance_req[0x10] = {0x01, 0x0f, 0x00, 0x00, 0x00,
										0x00, 0x00, 0x06, 0x20, 0x00,
										0x04, 0x00, 0x01, 0x01, 0x00, 0x00};
	dev->qmi_sync = 1;

 	hw_send_qmi_request_no_resp(dev->intf, set_instance_req, 0x10, resp_buf, 56);

	ret = hw_send_qmi_request(dev->intf, client_id_req, 0x10, resp_buf, 56);
	if (0 == ret){
		printk(KERN_ERR"%s: Get client ID failed\n", __FUNCTION__);
		goto failed;
	}
	status_req[5] = resp_buf[23];
	memset(resp_buf, 0, 56 * sizeof (unsigned char));

	for (repeat = 0; repeat < 3; repeat ++)
	{
		ret = hw_send_qmi_request(dev->intf, status_req, 13, resp_buf, 56);
		if (0 == ret){
			printk(KERN_ERR"%s: Get connection status failed\n", __FUNCTION__);
			continue;
		}

		if (0x02 == resp_buf[23]){
			printk(KERN_ERR"%s: carrier on\n", __FUNCTION__);
			netif_carrier_on(dev->net);
			break;
		} else {

			printk(KERN_ERR"%s: carrier off\n", __FUNCTION__);
			//netif_carrier_off(dev->net);
		}
	}
failed:
	rel_client_id_req[0x0f] = 0x02;
	rel_client_id_req[0x10] = status_req[5];
	memset(resp_buf, 0, 56 * sizeof (unsigned char));

	ret = hw_send_qmi_request_no_resp(dev->intf, rel_client_id_req, 0x11, resp_buf, 56);

	dev->qmi_sync = 0;
	cancel_delayed_work(&dev->status_work);
	//memset(resp_buf, 0, 56 * sizeof (unsigned char));
	return;

}
static int hw_send_qmi_request_no_resp(struct usb_interface *intf,
				unsigned char *snd_req, int snd_len,
				unsigned char *read_resp, int resp_len)
{
	int ret;
	int index = 0;
	struct usb_device *udev = interface_to_usbdev(intf);
	for (index = 0; index < 3; index++)
{
		ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), 0x00,
					0x21, 0x00, intf->cur_altsetting->desc.bInterfaceNumber,
					snd_req, snd_len, 5000);
		if (ret < 0){
			printk(KERN_ERR"%s: send the qmi request failed\n", __FUNCTION__);
			continue;
		}
		else {
			break;
		}
	}
	return ret;
}

static int hw_send_qmi_request(struct usb_interface *intf,
				unsigned char *snd_req, int snd_len,
				unsigned char *read_resp, int resp_len)
{
	int ret;
	int index = 0;
	struct usb_device *udev = interface_to_usbdev(intf);
	struct hw_cdc_net *net = usb_get_intfdata(intf);

	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), 0x00,
					0x21, 0x00, intf->cur_altsetting->desc.bInterfaceNumber,
					snd_req, snd_len, 5000);

	if (ret < 0){
		printk(KERN_ERR"%s: send the qmi request failed\n", __FUNCTION__);
		return ret;
	}

	while(index < 10){
		ret = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0), 0x01,
					0xA1, 0x00, intf->cur_altsetting->desc.bInterfaceNumber,
					read_resp, resp_len, 1000);
		if (ret <= 0){
			printk(KERN_ERR"%s: %d Get response failed\n", __FUNCTION__, index);
			msleep(10);
		} else {
			if (0x00 == read_resp[4]){
				if (0x01 == read_resp[6] && snd_req[5] == read_resp[5]
					&& snd_req[8] == read_resp[8] && snd_req[9] == read_resp[9]) {
					ret = 1;
					break;
				}
			} else if (0x01 == read_resp[4]) {
				if (0x02 == read_resp[6] && snd_req[5] == read_resp[5]
					&& snd_req[9] == read_resp[9] && snd_req[10] == read_resp[10]) {
					printk(KERN_ERR"%s: get the conn status req=%02x resp\n", __FUNCTION__, snd_req[9]);
					ret = 1;
					break;
				}
			} else if (0x04 == read_resp[4]){
				if (snd_req[9] == read_resp[9] && snd_req[10] == read_resp[10] && 0x02 == read_resp[16]) {
					printk(KERN_ERR"%s: get the conn status ind= carrier on\n", __FUNCTION__);
					netif_carrier_on(net->net);
				}
			}
		}
		index ++;
		continue;
	}

	if (index >= 10){
		ret = 0;
	}
	return ret;
}
#endif
static void __exit hw_cdc_exit(void)
{
 	usb_deregister(&hw_ether_driver);
}
module_exit(hw_cdc_exit);


MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");
