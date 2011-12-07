/*
 * Driver for HighSpeed USB Client Controller in MSM7K
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 * Author: Mike Lockwood <lockwood@android.com>
 *         Brian Swetland <swetland@google.com>
 *
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
#include <linux/list.h>

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/workqueue.h>
#include <linux/switch.h>
#include <linux/pm_runtime.h>

#include <mach/msm72k_otg.h>
#include <linux/io.h>
#include <linux/clk.h>

#include <asm/mach-types.h>

#include <mach/board.h>
#include <mach/msm_hsusb.h>
#include <mach/htc_battery_common.h>
#include <linux/device.h>
#include <linux/usb/composite.h>
#include <mach/msm_hsusb_hw.h>
#include <mach/clk.h>
#include <linux/uaccess.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#ifdef CONFIG_USB_ACCESSORY_DETECT_BY_ADC
#include <mach/htc_headset_mgr.h>
#ifdef CONFIG_HTC_HEADSET_MISC
#include <mach/htc_headset_misc.h>
#endif
#endif
#ifdef CONFIG_CABLE_DETECT_ACCESSORY
#include <mach/cable_detect.h>
#endif
#include <linux/wakelock.h>
#include <mach/perflock.h>


static const char driver_name[] = "msm72k_udc";

/* #define DEBUG */
/* #define VERBOSE */

#define MSM_USB_BASE ((unsigned) ui->addr)

#define	DRIVER_DESC		"MSM 72K USB Peripheral Controller"
#define	DRIVER_NAME		"MSM72K_UDC"

#define EPT_FLAG_IN        0x0001

#define SETUP_BUF_SIZE     8

#if (defined(CONFIG_DOCK_ACCESSORY_DETECT) || \
		defined(CONFIG_USB_ACCESSORY_DETECT))
#ifdef CONFIG_USB_ACCESSORY_DETECT_BY_ADC
extern int htc_get_usb_accessory_adc_level(uint32_t *buffer);
#endif
#ifdef CONFIG_USB_ACCESSORY_DETECT
static struct switch_dev dock_switch = {
	.name = "dock",
};
#endif
#define DOCK_STATE_UNDOCKED     0
#define DOCK_STATE_DESK         (1 << 0)
#define DOCK_STATE_CAR          (1 << 1)
#define DOCK_STATE_USB_HEADSET  (1 << 2)
#define DOCK_STATE_MHL          (1 << 3)

#define DOCK_DET_DELAY		HZ/4

/* #define MHL_REDETECT */
#define ADC_RETRY 3
#define ADC_RETRY_DELAY HZ/5
#endif

static const char *const ep_name[] = {
	"ep0out", "ep1out", "ep2out", "ep3out",
	"ep4out", "ep5out", "ep6out", "ep7out",
	"ep8out", "ep9out", "ep10out", "ep11out",
	"ep12out", "ep13out", "ep14out", "ep15out",
	"ep0in", "ep1in", "ep2in", "ep3in",
	"ep4in", "ep5in", "ep6in", "ep7in",
	"ep8in", "ep9in", "ep10in", "ep11in",
	"ep12in", "ep13in", "ep14in", "ep15in"
};

/*To release the wakelock from debugfs*/
static int release_wlocks;

static int use_mfg_serialno;
static char mfg_df_serialno[16];
static struct wake_lock vbus_idle_wake_lock;
static struct perf_lock usb_perf_lock;

struct msm_request {
	struct usb_request req;

	/* saved copy of req.complete */
	void	(*gadget_complete)(struct usb_ep *ep,
					struct usb_request *req);


	struct usb_info *ui;
	struct msm_request *next;
	struct msm_request *prev;

	unsigned busy:1;
	unsigned live:1;
	unsigned alloced:1;
	unsigned dead:1;

	dma_addr_t dma;
	dma_addr_t item_dma;

	struct ept_queue_item *item;
};

#define to_msm_request(r) container_of(r, struct msm_request, req)
#define to_msm_endpoint(r) container_of(r, struct msm_endpoint, ep)
#define to_msm_otg(xceiv)  container_of(xceiv, struct msm_otg, otg)
#define is_b_sess_vld()	((OTGSC_BSV & readl(USB_OTGSC)) ? 1 : 0)
#define is_usb_online(ui) (ui->usb_state != USB_STATE_NOTATTACHED)

struct msm_endpoint {
	struct usb_ep ep;
	struct usb_info *ui;
	struct msm_request *req; /* head of pending requests */
	struct msm_request *last;
	unsigned flags;

	/* bit number (0-31) in various status registers
	** as well as the index into the usb_info's array
	** of all endpoints
	*/
	unsigned char bit;
	unsigned char num;

	unsigned wedged:1;
	/* pointers to DMA transfer list area */
	/* these are allocated from the usb_info dma space */
	struct ept_queue_head *head;
};

static void usb_do_work(struct work_struct *w);
static void usb_do_remote_wakeup(struct work_struct *w);
#ifdef CONFIG_USB_ACCESSORY_DETECT
static void accessory_detect_work(struct work_struct *w);
static void accessory_detect_init(struct usb_info *ui);
#endif

#define USB_STATE_IDLE    0
#define USB_STATE_ONLINE  1
#define USB_STATE_OFFLINE 2

#define USB_FLAG_START          0x0001
#define USB_FLAG_VBUS_ONLINE    0x0002
#define USB_FLAG_VBUS_OFFLINE   0x0004
#define USB_FLAG_RESET          0x0008
#define USB_FLAG_SUSPEND        0x0010
#define USB_FLAG_CONFIGURED     0x0020

#define USB_CHG_DET_DELAY	msecs_to_jiffies(1000)
#define REMOTE_WAKEUP_DELAY	msecs_to_jiffies(1000)
#define DELAY_FOR_CHECK_CHG	msecs_to_jiffies(300)

struct usb_info {
	/* lock for register/queue/device state changes */
	spinlock_t lock;

	/* single request used for handling setup transactions */
	struct usb_request *setup_req;

	struct platform_device *pdev;
	int irq;
	void *addr;

	unsigned state;
	unsigned flags;

	atomic_t configured;
	atomic_t running;

	struct dma_pool *pool;

	/* dma page to back the queue heads and items */
	unsigned char *buf;
	dma_addr_t dma;

	struct ept_queue_head *head;

	/* used for allocation */
	unsigned next_item;
	unsigned next_ifc_num;

	/* endpoints are ordered based on their status bits,
	** so they are OUT0, OUT1, ... OUT15, IN0, IN1, ... IN15
	*/
	struct msm_endpoint ept[32];

	int connect_type_ready;

	int *phy_init_seq;
	void (*phy_reset)(void);
	void (*usb_uart_switch)(int);

	/* max power requested by selected configuration */
	unsigned b_max_pow;
	unsigned chg_current;
	struct delayed_work chg_det;
	struct delayed_work chg_stop;

	struct work_struct work;
	struct work_struct notifier_work;
	enum usb_connect_type connect_type;
	unsigned phy_status;
	unsigned phy_fail_count;

	struct usb_gadget		gadget;
	struct usb_gadget_driver	*driver;
	struct switch_dev sdev;

#define ep0out ept[0]
#define ep0in  ept[16]

	atomic_t ep0_dir;
	atomic_t test_mode;
	atomic_t offline_pending;
	atomic_t softconnect;
#ifdef CONFIG_USB_OTG
	u8 hnp_avail;
#endif

	atomic_t remote_wakeup;
	atomic_t self_powered;
	struct delayed_work rw_work;

	struct otg_transceiver *xceiv;
	enum usb_device_state usb_state;
	struct wake_lock wlock;

	unsigned int usb_id_pin_gpio;
	unsigned int idpin_irq;
	u8 accessory_detect;
	void (*usb_mhl_switch)(bool);
	void (*config_usb_id_gpios)(bool output_enable);
	void (*change_phy_voltage)(int);
	/* 0: none, 1: carkit, 2: usb headset, 3: desk (cradle) 4: mhl */
	u8 accessory_type;
	u8 mfg_usb_carkit_enable;
	u8 cable_redetect;
	struct delayed_work detect_work;
	struct workqueue_struct *usb_wq;
	int ac_9v_gpio;
	void (*configure_ac_9v_gpio) (int);
	unsigned int usb_id2_pin_gpio;
	void (*usb_host_switch)(int);
	struct timer_list ac_detect_timer;
	int ac_detect_count;
};

static const struct usb_ep_ops msm72k_ep_ops;
static struct usb_info *the_usb_info;

static int msm72k_wakeup(struct usb_gadget *_gadget);
static int msm72k_pullup_internal(struct usb_gadget *_gadget, int is_active);
static int msm72k_set_halt(struct usb_ep *_ep, int value);
static void flush_endpoint(struct msm_endpoint *ept);
static void usb_reset(struct usb_info *ui);
static int usb_ept_set_halt(struct usb_ep *_ep, int value);

static DEFINE_MUTEX(notify_sem);
static void send_usb_connect_notify(struct work_struct *w)
{
	static struct t_usb_status_notifier *notifier;
	struct usb_info *ui = container_of(w, struct usb_info,
		notifier_work);
	if (!ui)
		return;

	ui->connect_type_ready = 1;
	USB_INFO("send connect type %d\n", ui->connect_type);
	mutex_lock(&notify_sem);
	list_for_each_entry(notifier, &g_lh_usb_notifier_list, notifier_link) {
		if (notifier->func != NULL) {
			/* Notify other drivers about connect type. */
			/* use slow charging for unknown type*/
			if (ui->connect_type == CONNECT_TYPE_UNKNOWN)
				notifier->func(CONNECT_TYPE_USB);
			else
				notifier->func(ui->connect_type);
		}
	}
	mutex_unlock(&notify_sem);
	/* release the wake lock anyway */
	wake_unlock(&ui->wlock);
}

int usb_register_notifier(struct t_usb_status_notifier *notifier)
{
	if (!notifier || !notifier->name || !notifier->func)
		return -EINVAL;

	mutex_lock(&notify_sem);
	list_add(&notifier->notifier_link,
		&g_lh_usb_notifier_list);
	mutex_unlock(&notify_sem);
	return 0;
}

static void msm_hsusb_set_speed(struct usb_info *ui)
{
	unsigned long flags;

	spin_lock_irqsave(&ui->lock, flags);
	switch (readl(USB_PORTSC) & PORTSC_PSPD_MASK) {
	case PORTSC_PSPD_FS:
		USB_INFO("portchange USB_SPEED_FULL\n");
		ui->gadget.speed = USB_SPEED_FULL;
		break;
	case PORTSC_PSPD_LS:
		USB_INFO("portchange USB_SPEED_LOW\n");
		ui->gadget.speed = USB_SPEED_LOW;
		break;
	case PORTSC_PSPD_HS:
		USB_INFO("portchange USB_SPEED_HIGH\n");
		ui->gadget.speed = USB_SPEED_HIGH;
		break;
	}
	spin_unlock_irqrestore(&ui->lock, flags);
}

static void msm_hsusb_set_state(enum usb_device_state state)
{
	unsigned long flags;

	spin_lock_irqsave(&the_usb_info->lock, flags);
	the_usb_info->usb_state = state;
	spin_unlock_irqrestore(&the_usb_info->lock, flags);
}

static enum usb_device_state msm_hsusb_get_state(void)
{
	unsigned long flags;
	enum usb_device_state state;

	spin_lock_irqsave(&the_usb_info->lock, flags);
	state = the_usb_info->usb_state;
	spin_unlock_irqrestore(&the_usb_info->lock, flags);

	return state;
}

void msm_hsusb_request_reset(void)
{
	struct usb_info *ui = the_usb_info;
	unsigned long flags;
	if (!ui)
		return;
	spin_lock_irqsave(&ui->lock, flags);
	ui->flags |= USB_FLAG_RESET;
	queue_work(ui->usb_wq, &ui->work);
	spin_unlock_irqrestore(&ui->lock, flags);
}

static ssize_t print_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", DRIVER_NAME);
}

static ssize_t print_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", sdev->state ? "online" : "offline");
}

static inline enum chg_type usb_get_chg_type(struct usb_info *ui)
{
	if ((readl(USB_PORTSC) & PORTSC_LS) == PORTSC_LS)
		return USB_CHG_TYPE__WALLCHARGER;
	else
		return USB_CHG_TYPE__SDP;
}

#define USB_WALLCHARGER_CHG_CURRENT 1800
static int usb_get_max_power(struct usb_info *ui)
{
	struct msm_otg *otg = to_msm_otg(ui->xceiv);
	unsigned long flags;
	enum chg_type temp;
	int suspended;
	int configured;
	unsigned bmaxpow;

	if (ui->gadget.is_a_peripheral)
		return -EINVAL;

	temp = atomic_read(&otg->chg_type);
	spin_lock_irqsave(&ui->lock, flags);
	suspended = ui->usb_state == USB_STATE_SUSPENDED ? 1 : 0;
	configured = atomic_read(&ui->configured);
	bmaxpow = ui->b_max_pow;
	spin_unlock_irqrestore(&ui->lock, flags);

	USB_INFO("%s: type = %d, maxpow = %d\n", __func__, temp, bmaxpow);
	if (temp == USB_CHG_TYPE__INVALID)
		return -ENODEV;

	if (temp == USB_CHG_TYPE__WALLCHARGER)
		return USB_WALLCHARGER_CHG_CURRENT;

	if (suspended || !configured)
		return 0;

	return bmaxpow;
}

static void usb_chg_stop(struct work_struct *w)
{
	USB_INFO("disable charger\n");
	htc_battery_charger_disable();
}

static void usb_chg_detect(struct work_struct *w)
{
	struct usb_info *ui = container_of(w, struct usb_info, chg_det.work);
	struct msm_otg *otg = to_msm_otg(ui->xceiv);
	enum chg_type temp = USB_CHG_TYPE__INVALID;
	unsigned long flags;
	int maxpower;

	spin_lock_irqsave(&ui->lock, flags);
	if (ui->usb_state == USB_STATE_NOTATTACHED) {
		spin_unlock_irqrestore(&ui->lock, flags);
		return;
	}

	temp = usb_get_chg_type(ui);
	spin_unlock_irqrestore(&ui->lock, flags);

	atomic_set(&otg->chg_type, temp);
	maxpower = usb_get_max_power(ui);
	if (maxpower > 0)
		otg_set_power(ui->xceiv, maxpower);

	/* USB driver prevents idle and suspend power collapse(pc)
	 * while USB cable is connected. But when dedicated charger is
	 * connected, driver can vote for idle and suspend pc.
	 * OTG driver handles idle pc as part of above otg_set_power call
	 * when wallcharger is attached. To allow suspend pc, release the
	 * wakelock which will be re-acquired for any sub-sequent usb interrupts
	 * */
	if (temp == USB_CHG_TYPE__WALLCHARGER)
		pm_runtime_put_sync(&ui->pdev->dev);
}

static int usb_ep_get_stall(struct msm_endpoint *ept)
{
	unsigned int n;
	struct usb_info *ui = ept->ui;

	n = readl(USB_ENDPTCTRL(ept->num));
	if (ept->flags & EPT_FLAG_IN)
		return (CTRL_TXS & n) ? 1 : 0;
	else
		return (CTRL_RXS & n) ? 1 : 0;
}

static void ulpi_write(struct usb_info *ui, unsigned val, unsigned reg)
{
	unsigned timeout = 10000;

	/* initiate write operation */
	writel(ULPI_RUN | ULPI_WRITE |
	       ULPI_ADDR(reg) | ULPI_DATA(val),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while ((readl(USB_ULPI_VIEWPORT) & ULPI_RUN) && (--timeout))
		;

	if (timeout == 0)
		USB_ERR("ulpi_write: timeout\n");
}

static void ulpi_init(struct usb_info *ui)
{
	int *seq = ui->phy_init_seq;

	if (!seq)
		return;

	while (seq[0] >= 0) {
		USB_INFO("ulpi: write 0x%02x to 0x%02x\n", seq[0], seq[1]);
		ulpi_write(ui, seq[0], seq[1]);
		seq += 2;
	}
}

static void init_endpoints(struct usb_info *ui)
{
	unsigned n;

	for (n = 0; n < 32; n++) {
		struct msm_endpoint *ept = ui->ept + n;

		ept->ui = ui;
		ept->bit = n;
		ept->num = n & 15;
		ept->ep.name = ep_name[n];
		ept->ep.ops = &msm72k_ep_ops;

		if (ept->bit > 15) {
			/* IN endpoint */
			ept->head = ui->head + (ept->num << 1) + 1;
			ept->flags = EPT_FLAG_IN;
		} else {
			/* OUT endpoint */
			ept->head = ui->head + (ept->num << 1);
			ept->flags = 0;
		}

	}
}

static void config_ept(struct msm_endpoint *ept)
{
	unsigned cfg = CONFIG_MAX_PKT(ept->ep.maxpacket) | CONFIG_ZLT;

	/* ep0 out needs interrupt-on-setup */
	if (ept->bit == 0)
		cfg |= CONFIG_IOS;

	ept->head->config = cfg;
	ept->head->next = TERMINATE;
#if 0
	if (ept->ep.maxpacket)
		USB_DEBUG("ept #%d %s max:%d head:%p bit:%d\n",
		       ept->num,
		       (ept->flags & EPT_FLAG_IN) ? "in" : "out",
		       ept->ep.maxpacket, ept->head, ept->bit);
#endif
}

static void configure_endpoints(struct usb_info *ui)
{
	unsigned n;

	for (n = 0; n < 32; n++)
		config_ept(ui->ept + n);
}

struct usb_request *usb_ept_alloc_req(struct msm_endpoint *ept,
			unsigned bufsize, gfp_t gfp_flags)
{
	struct usb_info *ui = ept->ui;
	struct msm_request *req;

	req = kzalloc(sizeof(*req), gfp_flags);
	if (!req)
		goto fail1;

	req->item = dma_pool_alloc(ui->pool, gfp_flags, &req->item_dma);
	if (!req->item)
		goto fail2;

	if (bufsize) {
		req->req.buf = kmalloc(bufsize, gfp_flags);
		if (!req->req.buf)
			goto fail3;
		req->alloced = 1;
	}

	return &req->req;

fail3:
	dma_pool_free(ui->pool, req->item, req->item_dma);
fail2:
	kfree(req);
fail1:
	return 0;
}

static void do_free_req(struct usb_info *ui, struct msm_request *req)
{
	if (req->alloced)
		kfree(req->req.buf);

	dma_pool_free(ui->pool, req->item, req->item_dma);
	kfree(req);
}

static void usb_ept_enable(struct msm_endpoint *ept, int yes,
		unsigned char ep_type)
{
	struct usb_info *ui = ept->ui;
	int in = ept->flags & EPT_FLAG_IN;
	unsigned n;

	n = readl(USB_ENDPTCTRL(ept->num));

	if (in) {
		if (yes) {
			n = (n & (~CTRL_TXT_MASK)) |
				(ep_type << CTRL_TXT_EP_TYPE_SHIFT);
			n |= CTRL_TXE | CTRL_TXR;
		} else
			n &= (~CTRL_TXE);
	} else {
		if (yes) {
			n = (n & (~CTRL_RXT_MASK)) |
				(ep_type << CTRL_RXT_EP_TYPE_SHIFT);
			n |= CTRL_RXE | CTRL_RXR;
		} else
			n &= ~(CTRL_RXE);
	}
	/* complete all the updates to ept->head before enabling endpoint*/
	dma_coherent_pre_ops();
	writel(n, USB_ENDPTCTRL(ept->num));
#if 0
	dev_dbg(&ui->pdev->dev, "ept %d %s %s\n",
	       ept->num, in ? "in" : "out", yes ? "enabled" : "disabled");
#endif
}

static void usb_ept_start(struct msm_endpoint *ept)
{
	struct usb_info *ui = ept->ui;
	struct msm_request *req = ept->req;
	int i, cnt;
	unsigned n = 1 << ept->bit;

	BUG_ON(req->live);

	while (req) {
		req->live = 1;
		/* prepare the transaction descriptor item for the hardware */
		req->item->info =
			INFO_BYTES(req->req.length) | INFO_IOC | INFO_ACTIVE;
		req->item->page0 = req->dma;
		req->item->page1 = (req->dma + 0x1000) & 0xfffff000;
		req->item->page2 = (req->dma + 0x2000) & 0xfffff000;
		req->item->page3 = (req->dma + 0x3000) & 0xfffff000;

		if (req->next == NULL) {
			req->item->next = TERMINATE;
			break;
		}
		req->item->next = req->next->item_dma;
		req = req->next;
	}

	/* link the hw queue head to the request's transaction item */
	ept->head->next = ept->req->item_dma;
	ept->head->info = 0;

	/* flush buffers before priming ept */
	dma_coherent_pre_ops();

	/* during high throughput testing it is observed that
	 * ept stat bit is not set even thoguh all the data
	 * structures are updated properly and ept prime bit
	 * is set. To workaround the issue, try to check if
	 * ept stat bit otherwise try to re-prime the ept
	 */
	for (i = 0; i < 5; i++) {
		writel(n, USB_ENDPTPRIME);
		for (cnt = 0; cnt < 3000; cnt++) {
			if (!(readl(USB_ENDPTPRIME) & n) &&
					(readl(USB_ENDPTSTAT) & n))
				return;
			udelay(1);
		}
	}

	if ((readl(USB_ENDPTPRIME) & n) && !(readl(USB_ENDPTSTAT) & n))
		USB_ERR("Unable to prime the ept%d%s\n",
				ept->num,
				ept->flags & EPT_FLAG_IN ? "in" : "out");
}

int usb_ept_queue_xfer(struct msm_endpoint *ept, struct usb_request *_req)
{
	unsigned long flags;
	struct msm_request *req = to_msm_request(_req);
	struct msm_request *last;
	struct usb_info *ui = ept->ui;
	unsigned length = req->req.length;

	if (length > 0x4000)
		return -EMSGSIZE;

	spin_lock_irqsave(&ui->lock, flags);

	if (req->busy) {
		req->req.status = -EBUSY;
		spin_unlock_irqrestore(&ui->lock, flags);
		USB_INFO("usb_ept_queue_xfer() tried to queue busy request\n");
		return -EBUSY;
	}

	if (!atomic_read(&ui->configured) && (ept->num != 0)) {
		req->req.status = -ESHUTDOWN;
		spin_unlock_irqrestore(&ui->lock, flags);
		USB_INFO("usb_ept_queue_xfer() called while offline\n");
		return -ESHUTDOWN;
	}

	if (ui->usb_state == USB_STATE_SUSPENDED) {
		if (!atomic_read(&ui->remote_wakeup)) {
			req->req.status = -EAGAIN;
			spin_unlock_irqrestore(&ui->lock, flags);
			USB_INFO("%s: cannot queue as bus is suspended "
				"ept #%d %s max:%d head:%p bit:%d\n",
				__func__, ept->num,
				(ept->flags & EPT_FLAG_IN) ? "in" : "out",
				ept->ep.maxpacket, ept->head, ept->bit);

			return -EAGAIN;
		}

		otg_set_suspend(ui->xceiv, 0);
		schedule_delayed_work(&ui->rw_work, REMOTE_WAKEUP_DELAY);
	}

	req->busy = 1;
	req->live = 0;
	req->next = 0;
	req->req.status = -EBUSY;

	req->dma = dma_map_single(NULL, req->req.buf, length,
				  (ept->flags & EPT_FLAG_IN) ?
				  DMA_TO_DEVICE : DMA_FROM_DEVICE);

	/* Add the new request to the end of the queue */
	last = ept->last;
	if (last) {
		/* Already requests in the queue. add us to the
		 * end, but let the completion interrupt actually
		 * start things going, to avoid hw issues
		 */
		last->next = req;
	} else {
		/* queue was empty -- kick the hardware */
		ept->req = req;
		usb_ept_start(ept);
	}
	ept->last = req;

	spin_unlock_irqrestore(&ui->lock, flags);
	return 0;
}

/* --- endpoint 0 handling --- */

static void ep0_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct msm_request *r = to_msm_request(req);
	struct msm_endpoint *ept = to_msm_endpoint(ep);
	struct usb_info *ui = ept->ui;

	req->complete = r->gadget_complete;
	r->gadget_complete = 0;
	if	(req->complete)
		req->complete(&ui->ep0in.ep, req);
}

static void ep0_queue_ack_complete(struct usb_ep *ep,
	struct usb_request *_req)
{
	struct msm_request *r = to_msm_request(_req);
	struct msm_endpoint *ept = to_msm_endpoint(ep);
	struct usb_info *ui = ept->ui;
	struct usb_request *req = ui->setup_req;

	/* queue up the receive of the ACK response from the host */
	if (_req->status == 0 && _req->actual == _req->length) {
		req->length = 0;
		if (atomic_read(&ui->ep0_dir) == USB_DIR_IN)
			usb_ept_queue_xfer(&ui->ep0out, req);
		else
			usb_ept_queue_xfer(&ui->ep0in, req);
		_req->complete = r->gadget_complete;
		r->gadget_complete = 0;
		if (_req->complete)
			_req->complete(&ui->ep0in.ep, _req);
	} else
		ep0_complete(ep, _req);
}

static void ep0_setup_ack_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct msm_endpoint *ept = to_msm_endpoint(ep);
	struct usb_info *ui = ept->ui;
	unsigned int temp;
	int test_mode = atomic_read(&ui->test_mode);

	if (!test_mode)
		return;

	switch (test_mode) {
	case J_TEST:
		USB_INFO("usb electrical test mode: (J)\n");
		temp = readl(USB_PORTSC) & (~PORTSC_PTC);
		writel(temp | PORTSC_PTC_J_STATE, USB_PORTSC);
		break;

	case K_TEST:
		USB_INFO("usb electrical test mode: (K)\n");
		temp = readl(USB_PORTSC) & (~PORTSC_PTC);
		writel(temp | PORTSC_PTC_K_STATE, USB_PORTSC);
		break;

	case SE0_NAK_TEST:
		USB_INFO("usb electrical test mode: (SE0-NAK)\n");
		temp = readl(USB_PORTSC) & (~PORTSC_PTC);
		writel(temp | PORTSC_PTC_SE0_NAK, USB_PORTSC);
		break;

	case TST_PKT_TEST:
		USB_INFO("usb electrical test mode: (TEST_PKT)\n");
		temp = readl(USB_PORTSC) & (~PORTSC_PTC);
		writel(temp | PORTSC_PTC_TST_PKT, USB_PORTSC);
		break;
	}
}

static void ep0_setup_ack(struct usb_info *ui)
{
	struct usb_request *req = ui->setup_req;
	req->length = 0;
	req->complete = ep0_setup_ack_complete;
	usb_ept_queue_xfer(&ui->ep0in, req);
}

static void ep0_setup_stall(struct usb_info *ui)
{
	writel((1<<16) | (1<<0), USB_ENDPTCTRL(0));
}

static void ep0_setup_send(struct usb_info *ui, unsigned length)
{
	struct usb_request *req = ui->setup_req;
	struct msm_request *r = to_msm_request(req);
	struct msm_endpoint *ept = &ui->ep0in;

	req->length = length;
	req->complete = ep0_queue_ack_complete;
	r->gadget_complete = 0;
	usb_ept_queue_xfer(ept, req);
}

static void handle_setup(struct usb_info *ui)
{
	struct usb_ctrlrequest ctl;
	struct usb_request *req = ui->setup_req;
	int ret;
#ifdef CONFIG_USB_OTG
	u8 hnp;
	unsigned long flags;
#endif

	memcpy(&ctl, ui->ep0out.head->setup_data, sizeof(ctl));
	writel(EPT_RX(0), USB_ENDPTSETUPSTAT);

	if (ctl.bRequestType & USB_DIR_IN)
		atomic_set(&ui->ep0_dir, USB_DIR_IN);
	else
		atomic_set(&ui->ep0_dir, USB_DIR_OUT);

	/* any pending ep0 transactions must be canceled */
	flush_endpoint(&ui->ep0out);
	flush_endpoint(&ui->ep0in);
#if 0
	USB_INFO("setup: type=%02x req=%02x val=%04x idx=%04x len=%04x\n",
	       ctl.bRequestType, ctl.bRequest, ctl.wValue,
	       ctl.wIndex, ctl.wLength);
#endif
	if ((ctl.bRequestType & (USB_DIR_IN | USB_TYPE_MASK)) ==
					(USB_DIR_IN | USB_TYPE_STANDARD)) {
		if (ctl.bRequest == USB_REQ_GET_STATUS) {
			/* OTG supplement Rev 2.0 introduces another device
			 * GET_STATUS request for HNP polling with length = 1.
			 */
			u8 len = 2;
			switch (ctl.bRequestType & USB_RECIP_MASK) {
			case USB_RECIP_ENDPOINT:
			{
				struct msm_endpoint *ept;
				unsigned num =
					ctl.wIndex & USB_ENDPOINT_NUMBER_MASK;
				u16 temp = 0;

				if (num == 0) {
					memset(req->buf, 0, 2);
					break;
				}
				if (ctl.wIndex & USB_ENDPOINT_DIR_MASK)
					num += 16;
				ept = &ui->ep0out + num;
				temp = usb_ep_get_stall(ept);
				temp = temp << USB_ENDPOINT_HALT;
				memcpy(req->buf, &temp, 2);
				break;
			}
			case USB_RECIP_DEVICE:
			{
				u16 temp = 0;

				if (ctl.wIndex == OTG_STATUS_SELECTOR) {
#ifdef CONFIG_USB_OTG
					spin_lock_irqsave(&ui->lock, flags);
					hnp = (ui->gadget.host_request <<
							HOST_REQUEST_FLAG);
					ui->hnp_avail = 1;
					spin_unlock_irqrestore(&ui->lock,
							flags);
					memcpy(req->buf, &hnp, 1);
					len = 1;
#else
					goto stall;
#endif
				} else {
					temp = (atomic_read(&ui->self_powered)
						<< USB_DEVICE_SELF_POWERED);
					temp |= (atomic_read(&ui->remote_wakeup)
						<< USB_DEVICE_REMOTE_WAKEUP);
					memcpy(req->buf, &temp, 2);
				}
				break;
			}
			case USB_RECIP_INTERFACE:
				memset(req->buf, 0, 2);
				break;
			default:
				goto stall;
			}
			ep0_setup_send(ui, len);
			return;
		}
	}
	if (ctl.bRequestType ==
		    (USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_ENDPOINT)) {
		if ((ctl.bRequest == USB_REQ_CLEAR_FEATURE) ||
				(ctl.bRequest == USB_REQ_SET_FEATURE)) {
			if ((ctl.wValue == 0) && (ctl.wLength == 0)) {
				unsigned num = ctl.wIndex & 0x0f;

				if (num != 0) {
					struct msm_endpoint *ept;

					if (ctl.wIndex & 0x80)
						num += 16;
					ept = &ui->ep0out + num;

					if (ept->wedged)
						goto ack;
					if (ctl.bRequest == USB_REQ_SET_FEATURE)
						usb_ept_set_halt(&ept->ep, 1);
					else
						usb_ept_set_halt(&ept->ep, 0);
				}
				goto ack;
			}
		}
	}
	if (ctl.bRequestType == (USB_DIR_OUT | USB_TYPE_STANDARD)) {
		if (ctl.bRequest == USB_REQ_SET_CONFIGURATION) {
			atomic_set(&ui->configured, !!ctl.wValue);
			msm_hsusb_set_state(USB_STATE_CONFIGURED);
		} else if (ctl.bRequest == USB_REQ_SET_ADDRESS) {
			/*
			 * Gadget speed should be set when PCI interrupt
			 * occurs. But sometimes, PCI interrupt is not
			 * occuring after reset. Hence update the gadget
			 * speed here.
			 */
			if (ui->gadget.speed == USB_SPEED_UNKNOWN) {
				dev_info(&ui->pdev->dev,
					"PCI intr missed"
					"set speed explictly\n");
				msm_hsusb_set_speed(ui);
			}
			msm_hsusb_set_state(USB_STATE_ADDRESS);

			/* write address delayed (will take effect
			** after the next IN txn)
			*/
			writel((ctl.wValue << 25) | (1 << 24), USB_DEVICEADDR);
			goto ack;
		} else if (ctl.bRequest == USB_REQ_SET_FEATURE) {
			switch (ctl.wValue) {
			case USB_DEVICE_TEST_MODE:
				switch (ctl.wIndex) {
				case J_TEST:
				case K_TEST:
				case SE0_NAK_TEST:
					if (!atomic_read(&ui->test_mode))
						schedule_delayed_work(&ui->chg_stop, 0);
				case TST_PKT_TEST:
					atomic_set(&ui->test_mode, ctl.wIndex);
					goto ack;
				}
				goto stall;
			case USB_DEVICE_REMOTE_WAKEUP:
				atomic_set(&ui->remote_wakeup, 1);
				goto ack;
#ifdef CONFIG_USB_OTG
			case USB_DEVICE_B_HNP_ENABLE:
				ui->gadget.b_hnp_enable = 1;
				goto ack;
			case USB_DEVICE_A_HNP_SUPPORT:
			case USB_DEVICE_A_ALT_HNP_SUPPORT:
				/* B-devices compliant to OTG spec
				 * Rev 2.0 are not required to
				 * suppport these features.
				 */
				goto stall;
#endif
			}
		} else if ((ctl.bRequest == USB_REQ_CLEAR_FEATURE) &&
				(ctl.wValue == USB_DEVICE_REMOTE_WAKEUP)) {
			atomic_set(&ui->remote_wakeup, 0);
			goto ack;
		}
	}

	/* delegate if we get here */
	if (ui->driver) {
		ret = ui->driver->setup(&ui->gadget, &ctl);
		if (ret >= 0)
			return;
	}

stall:
	/* stall ep0 on error */
	ep0_setup_stall(ui);
	return;

ack:
	ep0_setup_ack(ui);
}

static void handle_endpoint(struct usb_info *ui, unsigned bit)
{
	struct msm_endpoint *ept = ui->ept + bit;
	struct msm_request *req;
	unsigned long flags;
	unsigned info;

	/*
	INFO("handle_endpoint() %d %s req=%p(%08x)\n",
		ept->num, (ept->flags & EPT_FLAG_IN) ? "in" : "out",
		ept->req, ept->req ? ept->req->item_dma : 0);
	*/

	/* expire all requests that are no longer active */
	spin_lock_irqsave(&ui->lock, flags);
	while ((req = ept->req)) {
		/* if we've processed all live requests, time to
		 * restart the hardware on the next non-live request
		 */
		if (!req->live) {
			usb_ept_start(ept);
			break;
		}

		/* clean speculative fetches on req->item->info */
		dma_coherent_post_ops();
		info = req->item->info;
		/* if the transaction is still in-flight, stop here */
		if (info & INFO_ACTIVE)
			break;

		/* advance ept queue to the next request */
		ept->req = req->next;
		if (ept->req == 0)
			ept->last = 0;

		dma_unmap_single(NULL, req->dma, req->req.length,
				 (ept->flags & EPT_FLAG_IN) ?
				 DMA_TO_DEVICE : DMA_FROM_DEVICE);

		if (info & (INFO_HALTED | INFO_BUFFER_ERROR | INFO_TXN_ERROR)) {
			/* XXX pass on more specific error code */
			req->req.status = -EIO;
			req->req.actual = 0;
			USB_INFO("ept %d %s error. info=%08x\n",
			       ept->num,
			       (ept->flags & EPT_FLAG_IN) ? "in" : "out",
			       info);
		} else {
			req->req.status = 0;
			req->req.actual =
				req->req.length - ((info >> 16) & 0x7FFF);
		}
		req->busy = 0;
		req->live = 0;

		if (req->dead)
			do_free_req(ui, req);
		if (req->req.complete) {
			spin_unlock_irqrestore(&ui->lock, flags);
			req->req.complete(&ept->ep, &req->req);
			spin_lock_irqsave(&ui->lock, flags);
		}
	}
	spin_unlock_irqrestore(&ui->lock, flags);
}

static void flush_endpoint_hw(struct usb_info *ui, unsigned bits)
{
	/* flush endpoint, canceling transactions
	** - this can take a "large amount of time" (per databook)
	** - the flush can fail in some cases, thus we check STAT
	**   and repeat if we're still operating
	**   (does the fact that this doesn't use the tripwire matter?!)
	*/
	do {
		writel(bits, USB_ENDPTFLUSH);
		while (readl(USB_ENDPTFLUSH) & bits)
			udelay(100);
	} while (readl(USB_ENDPTSTAT) & bits);
}

static void flush_endpoint_sw(struct msm_endpoint *ept)
{
	struct usb_info *ui = ept->ui;
	struct msm_request *req, *next_req = NULL;
	unsigned long flags;

	/* inactive endpoints have nothing to do here */
	if (ept->ep.maxpacket == 0)
		return;

	/* put the queue head in a sane state */
	ept->head->info = 0;
	ept->head->next = TERMINATE;

	/* cancel any pending requests */
	spin_lock_irqsave(&ui->lock, flags);
	req = ept->req;
	ept->req = 0;
	ept->last = 0;
	while (req != 0) {
		req->busy = 0;
		req->live = 0;
		req->req.status = -ESHUTDOWN;
		req->req.actual = 0;

		/* Gadget driver may free the request in completion
		 * handler. So keep a copy of next req pointer
		 * before calling completion handler.
		 */
		next_req = req->next;
		if (req->req.complete) {
			spin_unlock_irqrestore(&ui->lock, flags);
			req->req.complete(&ept->ep, &req->req);
			spin_lock_irqsave(&ui->lock, flags);
		}
		if (req->dead)
			do_free_req(ui, req);
		req = next_req;
	}
	spin_unlock_irqrestore(&ui->lock, flags);
}

static void flush_endpoint(struct msm_endpoint *ept)
{
	flush_endpoint_hw(ept->ui, (1 << ept->bit));
	flush_endpoint_sw(ept);
}

static irqreturn_t usb_interrupt(int irq, void *data)
{
	struct usb_info *ui = data;
	unsigned n;
	unsigned long flags;
	struct usb_composite_dev *cdev = get_gadget_data(&ui->gadget);

	n = readl(USB_USBSTS);
	writel(n, USB_USBSTS);

	/* somehow we got an IRQ while in the reset sequence: ignore it */
	if (!atomic_read(&ui->running))
		return IRQ_HANDLED;

	if (n & STS_PCI) {
		msm_hsusb_set_speed(ui);
		if (atomic_read(&ui->configured)) {
			spin_lock_irqsave(&ui->lock, flags);
			ui->usb_state = USB_STATE_CONFIGURED;
			ui->flags = USB_FLAG_CONFIGURED;
			spin_unlock_irqrestore(&ui->lock, flags);

			ui->driver->resume(&ui->gadget);
			queue_work(ui->usb_wq, &ui->work);
		} else {
			msm_hsusb_set_state(USB_STATE_DEFAULT);
		}

#ifdef CONFIG_USB_OTG
		/* notify otg to clear A_BIDL_ADIS timer */
		if (ui->gadget.is_a_peripheral)
			otg_set_suspend(ui->xceiv, 0);
#endif
	}

	if (n & STS_URI) {
		USB_INFO("reset\n");

#ifdef CONFIG_USB_OTG
		/* notify otg to clear A_BIDL_ADIS timer */
		if (ui->gadget.is_a_peripheral)
			otg_set_suspend(ui->xceiv, 0);
		spin_lock_irqsave(&ui->lock, flags);
		/* Host request is persistent across reset */
		ui->gadget.b_hnp_enable = 0;
		ui->hnp_avail = 0;
		spin_unlock_irqrestore(&ui->lock, flags);
#endif
		msm_hsusb_set_state(USB_STATE_DEFAULT);
		atomic_set(&ui->remote_wakeup, 0);

		writel(readl(USB_ENDPTSETUPSTAT), USB_ENDPTSETUPSTAT);
		writel(readl(USB_ENDPTCOMPLETE), USB_ENDPTCOMPLETE);
		writel(0xffffffff, USB_ENDPTFLUSH);
		writel(0, USB_ENDPTCTRL(1));

		if (atomic_read(&ui->configured)) {
			/* marking us offline will cause ept queue attempts
			** to fail
			*/
			atomic_set(&ui->configured, 0);
			/* Defer sending offline uevent to userspace */
			atomic_set(&ui->offline_pending, 1);

			/* XXX: we can't seem to detect going offline,
			 * XXX:  so deconfigure on reset for the time being
			 */
			if (ui->driver) {
				USB_INFO("usb: notify offline\n");
				ui->driver->disconnect(&ui->gadget);
			}
			/* cancel pending ep0 transactions */
			flush_endpoint(&ui->ep0out);
			flush_endpoint(&ui->ep0in);

		}

		if (ui->connect_type != CONNECT_TYPE_USB) {
			ui->connect_type = CONNECT_TYPE_USB;
			queue_work(ui->usb_wq, &ui->notifier_work);
			ui->ac_detect_count = 0;
			del_timer_sync(&ui->ac_detect_timer);
		}
	}

	if (n & STS_SLI) {
		USB_INFO("suspend\n");

		spin_lock_irqsave(&ui->lock, flags);
		ui->usb_state = USB_STATE_SUSPENDED;
		ui->flags = USB_FLAG_SUSPEND;
		spin_unlock_irqrestore(&ui->lock, flags);

		ui->driver->suspend(&ui->gadget);
		queue_work(ui->usb_wq, &ui->work);
#ifdef CONFIG_USB_OTG
		/* notify otg for
		 * 1. kicking A_BIDL_ADIS timer in case of A-peripheral
		 * 2. disabling pull-up and kicking B_ASE0_RST timer
		 */
		if (ui->gadget.b_hnp_enable || ui->gadget.is_a_peripheral)
			otg_set_suspend(ui->xceiv, 1);
#endif
	}

	if (n & STS_UI) {
		n = readl(USB_ENDPTSETUPSTAT);
		if (n & EPT_RX(0))
			handle_setup(ui);

		n = readl(USB_ENDPTCOMPLETE);
		writel(n, USB_ENDPTCOMPLETE);
		while (n) {
			unsigned bit = __ffs(n);
			handle_endpoint(ui, bit);
			n = n & (~(1 << bit));
		}
	}
	return IRQ_HANDLED;
}

int usb_is_connect_type_ready(void)
{
	if (!the_usb_info)
		return 0;
	return the_usb_info->connect_type_ready;
}
EXPORT_SYMBOL(usb_is_connect_type_ready);

int usb_get_connect_type(void)
{
	if (!the_usb_info)
		return 0;
	return the_usb_info->connect_type;
}
EXPORT_SYMBOL(usb_get_connect_type);

static ssize_t show_usb_cable_connect(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;
	if (!the_usb_info)
		return 0;
	length = sprintf(buf, "%d\n",
		(the_usb_info->connect_type == CONNECT_TYPE_USB)?1:0);
	return length;
}

static ssize_t show_USB_ID_status(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	int value = 1;
	unsigned length;
#if defined(CONFIG_CABLE_DETECT_ACCESSORY)
	value = cable_get_usb_id_level();
#else
	struct usb_info *ui = the_usb_info;

	if (!ui)
		return 0;
	if (ui->usb_id_pin_gpio != 0) {
		value = gpio_get_value(ui->usb_id_pin_gpio);
		USB_INFO("id pin status %d\n", value);
	}
#endif
	length = sprintf(buf, "%d", value);
	return length;
}

static ssize_t show_usb_car_kit_enable(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct usb_info *ui = the_usb_info;
	int value = 1;
	unsigned length;

	if (!ui)
		return 0;
	if (ui->accessory_detect == 0)
		value = 0;

	USB_INFO("USB_car_kit_enable %d\n", ui->accessory_detect);
	length = sprintf(buf, "%d", value);
	return length;
}
static DEVICE_ATTR(usb_cable_connect, 0444, show_usb_cable_connect, NULL);
static DEVICE_ATTR(USB_ID_status, 0444, show_USB_ID_status, NULL);
/*for kar kit AP check if car kit enable*/
static DEVICE_ATTR(usb_car_kit_enable, 0444, show_usb_car_kit_enable, NULL);

#ifdef CONFIG_USB_ACCESSORY_DETECT
static ssize_t show_mfg_carkit_enable(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned length;
	struct usb_info *ui = the_usb_info;

	length = sprintf(buf, "%d", ui->mfg_usb_carkit_enable);
	USB_INFO("%s: %d\n", __func__, ui->mfg_usb_carkit_enable);
	return length;
}

static ssize_t store_mfg_carkit_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct usb_info *ui = the_usb_info;
	unsigned char uc;

	if (buf[0] != '0' && buf[0] != '1') {
		USB_ERR("Can't enable/disable carkit\n");
		return -EINVAL;
	}
	uc = buf[0] - '0';
	USB_INFO("%s: %d\n", __func__, uc);
	ui->mfg_usb_carkit_enable = uc;
	if (uc == 1 && ui->accessory_type == 1 &&
		board_mfg_mode() == 1) {
		switch_set_state(&dock_switch, DOCK_STATE_CAR);
		USB_INFO("carkit: set state %d\n", DOCK_STATE_CAR);
	}
	return count;
}
static DEVICE_ATTR(usb_mfg_carkit_enable, 0644,
	show_mfg_carkit_enable, store_mfg_carkit_enable);
#endif

static void usb_prepare(struct usb_info *ui)
{
	int ret;

	spin_lock_init(&ui->lock);

	memset(ui->buf, 0, 4096);
	ui->head = (void *) (ui->buf + 0);

	/* only important for reset/reinit */
	memset(ui->ept, 0, sizeof(ui->ept));
	ui->next_item = 0;
	ui->next_ifc_num = 0;

	init_endpoints(ui);

	ui->ep0in.ep.maxpacket = 64;
	ui->ep0out.ep.maxpacket = 64;

	ui->setup_req =
		usb_ept_alloc_req(&ui->ep0in, SETUP_BUF_SIZE, GFP_KERNEL);

	ui->usb_wq = create_singlethread_workqueue("msm_hsusb");

	INIT_WORK(&ui->work, usb_do_work);
	INIT_WORK(&ui->notifier_work, send_usb_connect_notify);
	INIT_DELAYED_WORK(&ui->chg_det, usb_chg_detect);
	INIT_DELAYED_WORK(&ui->chg_stop, usb_chg_stop);
	INIT_DELAYED_WORK(&ui->rw_work, usb_do_remote_wakeup);
#ifdef CONFIG_USB_ACCESSORY_DETECT
	INIT_DELAYED_WORK(&ui->detect_work, accessory_detect_work);
	if (ui->accessory_detect)
		accessory_detect_init(ui);
#endif

	ret = device_create_file(&ui->pdev->dev,
		&dev_attr_usb_cable_connect);
	if (ret != 0)
		USB_WARNING("dev_attr_usb_cable_connect failed\n");

	ret = device_create_file(&ui->pdev->dev,
		&dev_attr_USB_ID_status);
	if (ret != 0)
		USB_WARNING("dev_attr_USB_ID_status failed\n");

	/*for kar kit AP check if car kit enable*/
	ret = device_create_file(&ui->pdev->dev,
		&dev_attr_usb_car_kit_enable);
	if (ret != 0)
		USB_WARNING("dev_attr_usb_car_kit_enable failed\n");

#ifdef CONFIG_USB_ACCESSORY_DETECT
	ret = device_create_file(&ui->pdev->dev,
		&dev_attr_usb_mfg_carkit_enable);
	if (ret != 0)
		USB_WARNING("dev_attr_usb_mfg_carkit_enable failed\n");
#endif

	ui->sdev.name = driver_name;
	ui->sdev.print_name = print_switch_name;
	ui->sdev.print_state = print_switch_state;

	ret = switch_dev_register(&ui->sdev);
	if (ret != 0)
		USB_WARNING("switch class can't be registered\n");
}

static void usb_reset(struct usb_info *ui)
{
	struct msm_otg *otg = to_msm_otg(ui->xceiv);
	struct usb_composite_dev *cdev = get_gadget_data(&ui->gadget);

	USB_INFO("reset controller\n");

	atomic_set(&ui->running, 0);
	/* marking us offline will cause ept queue attempts to fail */
	atomic_set(&ui->configured, 0);

	/*
	 * PHY reset takes minimum 100 msec. Hence reset only link
	 * during HNP. Reset PHY and link in B-peripheral mode.
	 */
	if (ui->gadget.is_a_peripheral)
		otg->reset(ui->xceiv, 0);
	else
		otg->reset(ui->xceiv, 1);

	/* set usb controller interrupt threshold to zero*/
	writel((readl(USB_USBCMD) & ~USBCMD_ITC_MASK) | USBCMD_ITC(0),
							USB_USBCMD);

	ulpi_init(ui);

	writel(ui->dma, USB_ENDPOINTLISTADDR);

	configure_endpoints(ui);

	if (ui->driver) {
		USB_INFO("usb: notify offline\n");
		ui->driver->disconnect(&ui->gadget);
	}

	/* cancel pending ep0 transactions */
	flush_endpoint(&ui->ep0out);
	flush_endpoint(&ui->ep0in);

	/* enable interrupts */
	writel(STS_URI | STS_SLI | STS_UI | STS_PCI, USB_USBINTR);

	atomic_set(&ui->running, 1);
}

static void usb_start(struct usb_info *ui)
{
	unsigned long flags;

	spin_lock_irqsave(&ui->lock, flags);
	ui->flags |= USB_FLAG_START;
	queue_work(ui->usb_wq, &ui->work);
	spin_unlock_irqrestore(&ui->lock, flags);
}

static int usb_free(struct usb_info *ui, int ret)
{
	USB_INFO("usb_free(%d)\n", ret);

	if (ui->xceiv)
		otg_put_transceiver(ui->xceiv);

	if (ui->irq)
		free_irq(ui->irq, 0);
	if (ui->pool)
		dma_pool_destroy(ui->pool);
	if (ui->dma)
		dma_free_coherent(&ui->pdev->dev, 4096, ui->buf, ui->dma);
	kfree(ui);
	return ret;
}

#ifdef CONFIG_USB_ACCESSORY_DETECT
static ssize_t dock_status_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct usb_info *ui = the_usb_info;
	if (ui->accessory_type == 1)
		return sprintf(buf, "online\n");
	else if (ui->accessory_type == 3) /*desk dock*/
		return sprintf(buf, "online\n");
	else
		return sprintf(buf, "offline\n");
}
static DEVICE_ATTR(status, S_IRUGO | S_IWUSR, dock_status_show, NULL);

static void carkit_detect(struct usb_info *ui)
{
	return;
}

#ifdef CONFIG_USB_ACCESSORY_DETECT_BY_ADC
static void accessory_type_switch(int type)
{
	struct usb_info *ui = the_usb_info;
	USB_INFO("%s accessory_type %d, type %d\n",
			__func__, ui->accessory_type, type);

	if (type == 0 || ui->accessory_type != type)
		switch_set_state(&dock_switch, DOCK_STATE_UNDOCKED);

	switch (type) {
	case 1:
		USB_INFO("carkit inserted\n");
		ui->accessory_type = 1;
		if ((board_mfg_mode() == 0) || (board_mfg_mode() == 1 &&
			ui->mfg_usb_carkit_enable == 1)) {
			switch_set_state(&dock_switch, DOCK_STATE_CAR);
			USB_INFO("carkit: set state %d\n", DOCK_STATE_CAR);
		}
		break;
	case 2:
		USB_INFO("headset inserted\n");
		ui->accessory_type = 2;
		headset_ext_detect(USB_AUDIO_OUT);
		break;
	case 3:
		USB_INFO("Desk Cradle inserted\n");
		ui->accessory_type = 3;
		switch_set_state(&dock_switch, DOCK_STATE_DESK);
		USB_INFO("Desk Cradle: set state %d\n", DOCK_STATE_DESK);
		break;
	case 4:
		USB_INFO("MHL inserted\n");
#ifdef CONFIG_MSM_HDMI_MHL
		ui->accessory_type = 4;
		if (ui->usb_mhl_switch)
			ui->usb_mhl_switch(1);
		switch_set_state(&dock_switch, DOCK_STATE_MHL);
		USB_INFO("MHL: set state %d\n", DOCK_STATE_MHL);
#endif
		break;
#if (defined(CONFIG_USB_OTG) && defined(CONFIG_USB_OTG_HOST))
	case 5:
	{
		unsigned long flags;
		struct msm_otg *dev = to_msm_otg(ui->xceiv);

		if (ui->usb_host_switch)
			ui->usb_host_switch(1);

		if (dev->hs_pclk)
			clk_enable(dev->hs_pclk);
		if (dev->hs_cclk)
			clk_enable(dev->hs_cclk);

		usb_reset(ui);
		spin_lock_irqsave(&ui->lock, flags);
		writel(readl(USB_USBCMD) | USBCMD_RS, USB_USBCMD);
		spin_unlock_irqrestore(&ui->lock, flags);
		msleep(10);
		/* D+/D- does not short, indicating host mode */
		if ((readl(USB_PORTSC) & PORTSC_LS) != PORTSC_LS) {
			USB_INFO("Set IDGND = 0, enter HOST mode\n");
			if (ui->usb_id2_pin_gpio)
				gpio_set_value(ui->usb_id2_pin_gpio, 0);
			ui->accessory_type = 5;
		} else {
			if (atomic_read(&dev->in_lpm)) {
				if (dev->hs_pclk)
					clk_disable(dev->hs_pclk);
				if (dev->hs_cclk)
					clk_disable(dev->hs_cclk);
			}
			if (ui->usb_host_switch)
				ui->usb_host_switch(0);
			USB_INFO("Not HOST mode\n");
		}
		break;
	}
#endif
	case -1:
		break;
	case 0:
#if (defined(CONFIG_USB_OTG) && defined(CONFIG_USB_OTG_HOST))
		USB_INFO("Set IDGND = 1\n");
		if (ui->usb_id2_pin_gpio)
			gpio_set_value(ui->usb_id2_pin_gpio, 1);
#endif
		switch (ui->accessory_type) {
		case 1:
			USB_INFO("carkit removed\n");
			break;
		case 2:
			USB_INFO("headset removed\n");
			headset_ext_detect(USB_NO_HEADSET);
			break;
		case 3:
			USB_INFO("Desk Cradle removed\n");
			break;
		case 4:
			/*MHL*/
			break;
#if (defined(CONFIG_USB_OTG) && defined(CONFIG_USB_OTG_HOST))
		case 5:
			/* host mode */
			if (ui->usb_host_switch)
				ui->usb_host_switch(0);
			ui->accessory_type = 0;
			break;
#endif
		default:
			break;
		}

		ui->accessory_type = 0;
		break;
	}
}
static int mhl_detect(struct usb_info *ui)
{
	uint32_t adc_value = 0xffffffff;
	int type = 0;

	if (ui->config_usb_id_gpios)
		ui->config_usb_id_gpios(1);

	htc_get_usb_accessory_adc_level(&adc_value);
	USB_INFO("[2nd] accessory adc = 0x%x\n", adc_value);

	if (adc_value >= 0x5999 && adc_value <= 0x76B0)
		type = 4;
	else
		type = 5;

	if (ui->config_usb_id_gpios)
		ui->config_usb_id_gpios(0);

	return type;
}

static int accessory_detect_by_adc(struct usb_info *ui)
{
	int value, type;
	static int prev_type, stable_count;

	value = gpio_get_value(ui->usb_id_pin_gpio);
	USB_INFO("%s: usb ID pin = %d\n", __func__, value);

	if (stable_count >= ADC_RETRY)
		stable_count = 0;

	if (value == 0 || ui->cable_redetect) {
		uint32_t adc_value = 0xffffffff;
		htc_get_usb_accessory_adc_level(&adc_value);
		USB_INFO("accessory adc = 0x%x\n", adc_value);

		if (adc_value >= 0x2112 && adc_value <= 0x3D53) {
			type = 2;
		} else if (adc_value >= 0x401D && adc_value <= 0x45EE) {
			/* Cradle: 0.551 mV ~ 0.601 mV */
			type = 3;
		} else if (adc_value >= 0x1174 && adc_value <= 0x1E38) {
			type = 1;
		} else if (adc_value >= 0x0 && adc_value < 0x1174) {
			type = mhl_detect(ui);
		} else
			type = -1;
	} else
		type = 0;

	if (prev_type == type)
		stable_count++;
	else
		stable_count = 0;

	USB_INFO("%s prev_type %d, type %d, stable_count %d\n",
				__func__, prev_type, type, stable_count);

	if (stable_count >= ADC_RETRY)
		accessory_type_switch(type);
	prev_type = type;
	return stable_count;
}
#endif

static void accessory_detect_work(struct work_struct *w)
{
	struct usb_info *ui = container_of(
			w, struct usb_info, detect_work.work);
	int value;
#ifdef CONFIG_USB_ACCESSORY_DETECT_BY_ADC
	int stable_count;
#endif

	if (!ui->accessory_detect)
		return;

	if (ui->accessory_detect == 1)
		carkit_detect(ui);
#ifdef CONFIG_USB_ACCESSORY_DETECT_BY_ADC
	else if (ui->accessory_detect == 2) {
		stable_count = accessory_detect_by_adc(ui);
		if (stable_count < ADC_RETRY) {
			queue_delayed_work(ui->usb_wq,
				&ui->detect_work, ADC_RETRY_DELAY);
			return;
		}
	}
#endif

	value = gpio_get_value(ui->usb_id_pin_gpio);
	USB_INFO("%s ID pin %d, type %d\n", __func__,
				value, ui->accessory_type);

#ifdef CONFIG_MSM_HDMI_MHL
	if (ui->accessory_type == 4)
		return;
#endif
	if (ui->accessory_type == 0)
		set_irq_type(ui->idpin_irq,
			value ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
	else
		set_irq_type(ui->idpin_irq, IRQF_TRIGGER_HIGH);

	enable_irq(ui->idpin_irq);
}

static irqreturn_t usbid_interrupt(int irq, void *data)
{
	struct usb_info *ui = data;

	disable_irq_nosync(ui->idpin_irq);
	USB_INFO("id interrupt\n");
	ui->cable_redetect = 0;
	queue_delayed_work(ui->usb_wq, &ui->detect_work, ADC_RETRY_DELAY);
	return IRQ_HANDLED;
}

static void accessory_detect_init(struct usb_info *ui)
{
	int ret;
	USB_INFO("%s: id pin %d\n", __func__, ui->usb_id_pin_gpio);

	if (ui->usb_id_pin_gpio == 0)
		return;
	if (ui->idpin_irq == 0)
		ui->idpin_irq = gpio_to_irq(ui->usb_id_pin_gpio);

	set_irq_flags(ui->idpin_irq, IRQF_VALID | IRQF_NOAUTOEN);
	ret = request_irq(ui->idpin_irq, usbid_interrupt,
				IRQF_TRIGGER_LOW,
				"car_kit_irq", ui);
	if (ret < 0) {
		USB_ERR("%s: request_irq failed\n", __func__);
		return;
	}

	ret = set_irq_wake(ui->idpin_irq, 1);
	if (ret < 0) {
		USB_ERR("%s: set_irq_wake failed\n", __func__);
		goto err;
	}

	if (switch_dev_register(&dock_switch) < 0) {
		USB_ERR(" fail to register dock switch!\n");
		goto err;
	}

	ret = device_create_file(dock_switch.dev, &dev_attr_status);
	if (ret != 0)
		USB_WARNING("dev_attr_status failed\n");

	enable_irq(ui->idpin_irq);

	return;
err:
	free_irq(ui->idpin_irq, 0);
}
#endif

static void charger_detect(struct usb_info *ui)
{
	msleep(10);

	/* detect shorted D+/D-, indicating AC power */
	if ((readl(USB_PORTSC) & PORTSC_LS) != PORTSC_LS) {
		if (ui->connect_type == CONNECT_TYPE_USB) {
			USB_INFO("USB charger is already detected\n");
		} else {
			USB_INFO("not AC charger\n");
			ui->connect_type = CONNECT_TYPE_UNKNOWN;
			queue_work(ui->usb_wq, &ui->notifier_work);
			queue_delayed_work(ui->usb_wq, &ui->chg_det,
					DELAY_FOR_CHECK_CHG);
			mod_timer(&ui->ac_detect_timer, jiffies + (3 * HZ));
		}
	} else {
		if (ui->usb_id_pin_gpio != 0) {
			if (gpio_get_value(ui->usb_id_pin_gpio) == 0) {
				USB_INFO("9V AC charger\n");
				ui->connect_type = CONNECT_TYPE_9V_AC;
			} else {
				USB_INFO("AC charger\n");
				ui->connect_type = CONNECT_TYPE_AC;
			}
		} else {
			USB_INFO("AC charger\n");
			ui->connect_type = CONNECT_TYPE_AC;
		}
		queue_work(ui->usb_wq, &ui->notifier_work);
		msleep(10);
		if (ui->change_phy_voltage)
			ui->change_phy_voltage(0);
	}
}

static void charger_detect_by_9v_gpio(struct usb_info *ui)
{
	int ac_9v_charger = 0;

	msleep(10);
	if (ui->configure_ac_9v_gpio)
		ui->configure_ac_9v_gpio(1);
	mdelay(5);
	ac_9v_charger = gpio_get_value(ui->ac_9v_gpio);
	if (ui->configure_ac_9v_gpio)
		ui->configure_ac_9v_gpio(0);

	if (ac_9v_charger) {
		USB_INFO("9V AC charger\n");
		ui->connect_type = CONNECT_TYPE_9V_AC;
	} else if ((readl(USB_PORTSC) & PORTSC_LS) == PORTSC_LS) {
		if ((ui->usb_id_pin_gpio) &&
				gpio_get_value(ui->usb_id_pin_gpio)) {
			USB_INFO("9V AC charger\n");
			ui->connect_type = CONNECT_TYPE_9V_AC;
		} else {
			USB_INFO("AC charger\n");
			ui->connect_type = CONNECT_TYPE_AC;
		}
	} else {
		USB_INFO("not AC charger\n");
		ui->connect_type = CONNECT_TYPE_UNKNOWN;
		queue_delayed_work(ui->usb_wq, &ui->chg_det,
				DELAY_FOR_CHECK_CHG);
		mod_timer(&ui->ac_detect_timer, jiffies + (3 * HZ));
	}
	queue_work(ui->usb_wq, &ui->notifier_work);
}

static void usb_do_work_check_vbus(struct usb_info *ui)
{
	unsigned long iflags;

	spin_lock_irqsave(&ui->lock, iflags);

	if (is_usb_online(ui))
		ui->flags |= USB_FLAG_VBUS_ONLINE;
	else
		ui->flags |= USB_FLAG_VBUS_OFFLINE;
	spin_unlock_irqrestore(&ui->lock, iflags);
}

static void usb_do_work(struct work_struct *w)
{
	struct usb_info *ui = container_of(w, struct usb_info, work);
	struct msm_otg *otg = to_msm_otg(ui->xceiv);
	unsigned long iflags;
	unsigned flags, _vbus;

	for (;;) {
		spin_lock_irqsave(&ui->lock, iflags);
		flags = ui->flags;
		ui->flags = 0;
		_vbus = is_usb_online(ui);
		spin_unlock_irqrestore(&ui->lock, iflags);

		/* give up if we have nothing to do */
		if (flags == 0)
			break;

		switch (ui->state) {
		case USB_STATE_IDLE:
			if (flags & USB_FLAG_START) {
				int ret;

				if (!_vbus) {
					ui->state = USB_STATE_OFFLINE;
					break;
				}

				pm_runtime_get_noresume(&ui->pdev->dev);
				pm_runtime_resume(&ui->pdev->dev);
				USB_INFO("msm72k_udc: IDLE -> ONLINE\n");
				usb_reset(ui);
				ret = request_irq(otg->irq, usb_interrupt,
							IRQF_SHARED,
							ui->pdev->name, ui);
				/* FIXME: should we call BUG_ON when
				 * requst irq fails
				 */
				if (ret) {
					USB_ERR("hsusb: peripheral: request irq"
						" failed:(%d)", ret);
					break;
				}

				ui->irq = otg->irq;
				ui->state = USB_STATE_ONLINE;
				usb_do_work_check_vbus(ui);

				if (!atomic_read(&ui->softconnect))
					break;

				msm72k_pullup_internal(&ui->gadget, 1);
				if (ui->ac_9v_gpio && ui->configure_ac_9v_gpio)
					charger_detect_by_9v_gpio(ui);
				else
					charger_detect(ui);
			}
			break;
		case USB_STATE_ONLINE:
			if (atomic_read(&ui->offline_pending)) {
				switch_set_state(&ui->sdev, 0);
				atomic_set(&ui->offline_pending, 0);
			}

			/* If at any point when we were online, we received
			 * the signal to go offline, we must honor it
			 */
			if (flags & USB_FLAG_VBUS_OFFLINE) {

				ui->chg_current = 0;
				/* wait incase chg_detect is running */
				if (!ui->gadget.is_a_peripheral)
					cancel_delayed_work_sync(&ui->chg_det);

				USB_INFO("msm72k_udc: ONLINE -> OFFLINE\n");

				atomic_set(&ui->running, 0);
				atomic_set(&ui->remote_wakeup, 0);
				atomic_set(&ui->configured, 0);

				if (ui->connect_type != CONNECT_TYPE_NONE) {
					ui->connect_type = CONNECT_TYPE_NONE;
					queue_work(ui->usb_wq, &ui->notifier_work);
				}

				if (ui->driver) {
					USB_DEBUG("usb: notify offline\n");
					ui->driver->disconnect(&ui->gadget);
				}
				/* cancel pending ep0 transactions */
				flush_endpoint(&ui->ep0out);
				flush_endpoint(&ui->ep0in);

				/* synchronize with irq context */
				spin_lock_irqsave(&ui->lock, iflags);
#ifdef CONFIG_USB_OTG
				ui->gadget.host_request = 0;
				ui->gadget.b_hnp_enable = 0;
				ui->hnp_avail = 0;
#endif
				msm72k_pullup_internal(&ui->gadget, 0);
				spin_unlock_irqrestore(&ui->lock, iflags);


				/* if charger is initialized to known type
				 * we must let modem know about charger
				 * disconnection
				 */
				otg_set_power(ui->xceiv, 0);

				if (ui->irq) {
					free_irq(ui->irq, ui);
					ui->irq = 0;
				}


				switch_set_state(&ui->sdev, 0);

				ui->ac_detect_count = 0;
				del_timer_sync(&ui->ac_detect_timer);

				ui->state = USB_STATE_OFFLINE;
				usb_do_work_check_vbus(ui);
				pm_runtime_put_noidle(&ui->pdev->dev);
				pm_runtime_suspend(&ui->pdev->dev);
				break;
			}
			if (flags & USB_FLAG_SUSPEND) {
				int maxpower = usb_get_max_power(ui);

				if (maxpower < 0)
					break;

				otg_set_power(ui->xceiv, 0);

				/* TBD: Initiate LPM at usb bus suspend */
				break;
			}
			if (flags & USB_FLAG_CONFIGURED) {
				int maxpower = usb_get_max_power(ui);

				/* We may come here even when no configuration
				 * is selected. Send online/offline event
				 * accordingly.
				 */
				switch_set_state(&ui->sdev,
						atomic_read(&ui->configured));

				if (maxpower < 0)
					break;

				ui->chg_current = maxpower;
				otg_set_power(ui->xceiv, maxpower);
				break;
			}
			if (flags & USB_FLAG_RESET) {
				USB_INFO("msm72k_udc: ONLINE -> RESET\n");
				msm72k_pullup_internal(&ui->gadget, 0);
				usb_reset(ui);
				msm72k_pullup_internal(&ui->gadget, 1);
				USB_INFO("msm72k_udc: RESET -> ONLINE\n");
				break;
			}
			break;
		case USB_STATE_OFFLINE:
			/* If we were signaled to go online and vbus is still
			 * present when we received the signal, go online.
			 */
			if ((flags & USB_FLAG_VBUS_ONLINE) && _vbus) {
				int ret;

				pm_runtime_get_noresume(&ui->pdev->dev);
				pm_runtime_resume(&ui->pdev->dev);
				USB_INFO("msm72k_udc: OFFLINE -> ONLINE\n");

				usb_reset(ui);
				ui->state = USB_STATE_ONLINE;
				usb_do_work_check_vbus(ui);
				ret = request_irq(otg->irq, usb_interrupt,
							IRQF_SHARED,
							ui->pdev->name, ui);
				/* FIXME: should we call BUG_ON when
				 * requst irq fails
				 */
				if (ret) {
					USB_ERR("hsusb: peripheral: request irq"
						" failed:(%d)", ret);
					break;
				}

				ui->irq = otg->irq;
				enable_irq_wake(otg->irq);

				if (!atomic_read(&ui->softconnect))
					break;
				msm72k_pullup_internal(&ui->gadget, 1);
				if (ui->ac_9v_gpio && ui->configure_ac_9v_gpio)
					charger_detect_by_9v_gpio(ui);
				else
					charger_detect(ui);
			}
			break;
		}
	}
}

void msm_hsusb_set_vbus_state(int online)
{
	struct usb_info *ui = the_usb_info;
	unsigned long flags;

	if (ui) {
		spin_lock_irqsave(&ui->lock, flags);

		if (online) {
			/*USB*/
			if (ui->usb_uart_switch)
				ui->usb_uart_switch(0);
		} else {
			/*UART*/
			if (ui->usb_uart_switch)
				ui->usb_uart_switch(1);
		}
		spin_unlock_irqrestore(&ui->lock, flags);
		/* hold a wake lock to distinguish cable type */
		wake_lock_timeout(&ui->wlock, HZ*5);
	}
#ifdef CONFIG_USB_OTG
	htc_otg_set_vbus_state(online);
#else
	msm_hsusb_set_vbus_state_internal(online);
#endif
}

/* FIXME - the callers of this function should use a gadget API instead.
 * This is called from htc_battery.c and board-halibut.c
 * WARNING - this can get called before this driver is initialized.
 */
static void msm_hsusb_set_vbus_state_internal(int online)
{
	unsigned long flags;
	struct usb_info *ui = the_usb_info;

	USB_INFO("%s: %d\n", __func__, online);

	if (!ui) {
		USB_ERR("msm_hsusb_set_vbus_state called"
			" before driver initialized\n");
		return;
	}

	spin_lock_irqsave(&ui->lock, flags);

	if (is_usb_online(ui) ==  online)
		goto out;

	if (online) {
		ui->usb_state = USB_STATE_POWERED;
		ui->flags |= USB_FLAG_VBUS_ONLINE;
#ifdef CONFIG_MSM_HDMI_MHL
		if (ui->cable_redetect) {
			USB_INFO("mhl re-detect\n");
			disable_irq_nosync(ui->idpin_irq);
			queue_delayed_work(ui->usb_wq, &ui->detect_work, HZ/4);
		}
#endif
	} else {
		ui->gadget.speed = USB_SPEED_UNKNOWN;
		ui->usb_state = USB_STATE_NOTATTACHED;
		ui->flags |= USB_FLAG_VBUS_OFFLINE;
	}
	if (in_interrupt()) {
		queue_work(ui->usb_wq, &ui->work);
	} else {
		spin_unlock_irqrestore(&ui->lock, flags);
		usb_do_work(&ui->work);
		return;
	}
out:
	spin_unlock_irqrestore(&ui->lock, flags);
}

#if defined(CONFIG_DEBUG_FS)

void usb_function_reenumerate(void)
{
	struct usb_info *ui = the_usb_info;

	/* disable and re-enable the D+ pullup */
	USB_INFO("disable pullup\n");
	writel(readl(USB_USBCMD) & ~USBCMD_RS, USB_USBCMD);

	msleep(10);

	USB_INFO("enable pullup\n");
	writel(readl(USB_USBCMD) | USBCMD_RS, USB_USBCMD);
}

static char debug_buffer[PAGE_SIZE];

static ssize_t debug_read_status(struct file *file, char __user *ubuf,
				 size_t count, loff_t *ppos)
{
	struct usb_info *ui = file->private_data;
	char *buf = debug_buffer;
	unsigned long flags;
	struct msm_endpoint *ept;
	struct msm_request *req;
	int n;
	int i = 0;

	spin_lock_irqsave(&ui->lock, flags);

	i += scnprintf(buf + i, PAGE_SIZE - i,
		   "regs: setup=%08x prime=%08x stat=%08x done=%08x\n",
		   readl(USB_ENDPTSETUPSTAT),
		   readl(USB_ENDPTPRIME),
		   readl(USB_ENDPTSTAT),
		   readl(USB_ENDPTCOMPLETE));
	i += scnprintf(buf + i, PAGE_SIZE - i,
		   "regs:   cmd=%08x   sts=%08x intr=%08x port=%08x\n\n",
		   readl(USB_USBCMD),
		   readl(USB_USBSTS),
		   readl(USB_USBINTR),
		   readl(USB_PORTSC));


	for (n = 0; n < 32; n++) {
		ept = ui->ept + n;
		if (ept->ep.maxpacket == 0)
			continue;

		i += scnprintf(buf + i, PAGE_SIZE - i,
			"ept%d %s cfg=%08x active=%08x next=%08x info=%08x\n",
			ept->num, (ept->flags & EPT_FLAG_IN) ? "in " : "out",
			ept->head->config, ept->head->active,
			ept->head->next, ept->head->info);

		for (req = ept->req; req; req = req->next)
			i += scnprintf(buf + i, PAGE_SIZE - i,
			"  req @%08x next=%08x info=%08x page0=%08x %c %c\n",
				req->item_dma, req->item->next,
				req->item->info, req->item->page0,
				req->busy ? 'B' : ' ',
				req->live ? 'L' : ' ');
	}

	i += scnprintf(buf + i, PAGE_SIZE - i,
			   "phy failure count: %d\n", ui->phy_fail_count);

	spin_unlock_irqrestore(&ui->lock, flags);

	return simple_read_from_buffer(ubuf, count, ppos, buf, i);
}

static ssize_t debug_write_reset(struct file *file, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	struct usb_info *ui = file->private_data;
	unsigned long flags;

	spin_lock_irqsave(&ui->lock, flags);
	ui->flags |= USB_FLAG_RESET;
	queue_work(ui->usb_wq, &ui->work);
	spin_unlock_irqrestore(&ui->lock, flags);

	return count;
}

static ssize_t debug_write_cycle(struct file *file, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	usb_function_reenumerate();
	return count;
}

static int debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

const struct file_operations debug_stat_ops = {
	.open = debug_open,
	.read = debug_read_status,
};

const struct file_operations debug_reset_ops = {
	.open = debug_open,
	.write = debug_write_reset,
};

const struct file_operations debug_cycle_ops = {
	.open = debug_open,
	.write = debug_write_cycle,
};

static ssize_t debug_read_release_wlocks(struct file *file, char __user *ubuf,
				 size_t count, loff_t *ppos)
{
	char kbuf[10];
	size_t c = 0;

	memset(kbuf, 0, 10);

	c = scnprintf(kbuf, 10, "%d", release_wlocks);

	if (copy_to_user(ubuf, kbuf, c))
		return -EFAULT;

	return c;
}
static ssize_t debug_write_release_wlocks(struct file *file,
		const char __user *buf, size_t count, loff_t *ppos)
{
	char kbuf[10];
	long temp;

	memset(kbuf, 0, 10);

	if (copy_from_user(kbuf, buf, count > 10 ? 10 : count))
		return -EFAULT;

	if (strict_strtol(kbuf, 10, &temp))
		return -EINVAL;

	if (temp)
		release_wlocks = 1;
	else
		release_wlocks = 0;

	return count;
}
static int debug_wake_lock_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}
const struct file_operations debug_wlocks_ops = {
	.open = debug_wake_lock_open,
	.read = debug_read_release_wlocks,
	.write = debug_write_release_wlocks,
};
static void usb_debugfs_init(struct usb_info *ui)
{
	struct dentry *dent;
	dent = debugfs_create_dir(dev_name(&ui->pdev->dev), 0);
	if (IS_ERR(dent))
		return;

	debugfs_create_file("status", 0444, dent, ui, &debug_stat_ops);
	debugfs_create_file("reset", 0220, dent, ui, &debug_reset_ops);
	debugfs_create_file("cycle", 0220, dent, ui, &debug_cycle_ops);
	debugfs_create_file("release_wlocks", 0660, dent, ui,
						&debug_wlocks_ops);
}
#else
static void usb_debugfs_init(struct usb_info *ui) {}
#endif

static int
msm72k_enable(struct usb_ep *_ep, const struct usb_endpoint_descriptor *desc)
{
	struct msm_endpoint *ept = to_msm_endpoint(_ep);
	unsigned char ep_type =
			desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;

	_ep->maxpacket = le16_to_cpu(desc->wMaxPacketSize);
	config_ept(ept);
	ept->wedged = 0;
	usb_ept_enable(ept, 1, ep_type);
	return 0;
}

static int msm72k_disable(struct usb_ep *_ep)
{
	struct msm_endpoint *ept = to_msm_endpoint(_ep);

	usb_ept_enable(ept, 0, 0);
	flush_endpoint(ept);
	return 0;
}

static struct usb_request *
msm72k_alloc_request(struct usb_ep *_ep, gfp_t gfp_flags)
{
	return usb_ept_alloc_req(to_msm_endpoint(_ep), 0, gfp_flags);
}

static void
msm72k_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct msm_request *req = to_msm_request(_req);
	struct msm_endpoint *ept = to_msm_endpoint(_ep);
	struct usb_info *ui = ept->ui;
	unsigned long flags;
	int dead = 0;

	spin_lock_irqsave(&ui->lock, flags);
	/* defer freeing resources if request is still busy */
	if (req->busy) {
		USB_INFO("Warning: msm72k_free_request req busy\n");
		dead = req->dead = 1;
	}
	spin_unlock_irqrestore(&ui->lock, flags);

	/* if req->dead, then we will clean up when the request finishes */
	if (!dead)
		do_free_req(ui, req);
}

static int
msm72k_queue(struct usb_ep *_ep, struct usb_request *req, gfp_t gfp_flags)
{
	struct msm_endpoint *ep = to_msm_endpoint(_ep);
	struct usb_info *ui = ep->ui;

	if (ep == &ui->ep0in) {
		struct msm_request *r = to_msm_request(req);
		if (!req->length)
			goto ep_queue_done;
		r->gadget_complete = req->complete;
		/* ep0_queue_ack_complete queue a receive for ACK before
		** calling req->complete
		*/
		req->complete = ep0_queue_ack_complete;
		if (atomic_read(&ui->ep0_dir) == USB_DIR_OUT)
			ep = &ui->ep0out;
		goto ep_queue_done;
	}

ep_queue_done:
	return usb_ept_queue_xfer(ep, req);
}

static int msm72k_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct msm_endpoint *ep = to_msm_endpoint(_ep);
	struct msm_request *req = to_msm_request(_req);
	struct usb_info *ui = ep->ui;

	struct msm_request *temp_req;
	unsigned long flags;

	if (!(ui && req && ep->req))
		return -EINVAL;

	spin_lock_irqsave(&ui->lock, flags);
	if (!req->busy) {
		USB_DEBUG("%s: !req->busy\n", __func__);
		spin_unlock_irqrestore(&ui->lock, flags);
		return -EINVAL;
	}
	/* Stop the transfer */
	do {
		writel((1 << ep->bit), USB_ENDPTFLUSH);
		while (readl(USB_ENDPTFLUSH) & (1 << ep->bit))
			udelay(100);
	} while (readl(USB_ENDPTSTAT) & (1 << ep->bit));

	req->req.status = 0;
	req->busy = 0;

	if (ep->req == req) {
		ep->req = req->next;
		ep->head->next = req->item->next;
	} else {
		req->prev->next = req->next;
		if (req->next)
			req->next->prev = req->prev;
		req->prev->item->next = req->item->next;
	}

	if (!req->next)
		ep->last = req->prev;

	/* initialize request to default */
	req->item->next = TERMINATE;
	req->item->info = 0;
	req->live = 0;
	dma_unmap_single(NULL, req->dma, req->req.length,
		(ep->flags & EPT_FLAG_IN) ?
		DMA_TO_DEVICE : DMA_FROM_DEVICE);

	if (req->req.complete) {
		req->req.status = -ECONNRESET;
		spin_unlock_irqrestore(&ui->lock, flags);
		req->req.complete(&ep->ep, &req->req);
		spin_lock_irqsave(&ui->lock, flags);
	}

	if (req->dead)
		do_free_req(ui, req);

	if (!req->live) {
		/* Reprime the endpoint for the remaining transfers */
		for (temp_req = ep->req ; temp_req ; temp_req = temp_req->next)
			temp_req->live = 0;
		if (ep->req)
			usb_ept_start(ep);
		spin_unlock_irqrestore(&ui->lock, flags);
		return 0;
	}
	spin_unlock_irqrestore(&ui->lock, flags);
	return 0;
}

static int
usb_ept_set_halt(struct usb_ep *_ep, int value)
{
	struct msm_endpoint *ept = to_msm_endpoint(_ep);
	struct usb_info *ui = ept->ui;
	unsigned int in = ept->flags & EPT_FLAG_IN;
	unsigned int n;
	unsigned long flags;

	spin_lock_irqsave(&ui->lock, flags);

	n = readl(USB_ENDPTCTRL(ept->num));

	if (in) {
		if (value)
			n |= CTRL_TXS;
		else {
			n &= ~CTRL_TXS;
			n |= CTRL_TXR;
		}
	} else {
		if (value)
			n |= CTRL_RXS;
		else {
			n &= ~CTRL_RXS;
			n |= CTRL_RXR;
		}
	}
	writel(n, USB_ENDPTCTRL(ept->num));
	if (!value)
		ept->wedged = 0;
	spin_unlock_irqrestore(&ui->lock, flags);

	return 0;
}

static int
msm72k_set_halt(struct usb_ep *_ep, int value)
{
	struct msm_endpoint *ept = to_msm_endpoint(_ep);
	unsigned int in = ept->flags & EPT_FLAG_IN;

	if (value && in && ept->req)
		return -EAGAIN;

	usb_ept_set_halt(_ep, value);

	return 0;
}

static int
msm72k_fifo_status(struct usb_ep *_ep)
{
	return -EOPNOTSUPP;
}

static void
msm72k_fifo_flush(struct usb_ep *_ep)
{
	flush_endpoint(to_msm_endpoint(_ep));
}
static int msm72k_set_wedge(struct usb_ep *_ep)
{
	struct msm_endpoint *ept = to_msm_endpoint(_ep);

	if (ept->num == 0)
		return -EINVAL;

	ept->wedged = 1;

	return msm72k_set_halt(_ep, 1);
}

static const struct usb_ep_ops msm72k_ep_ops = {
	.enable		= msm72k_enable,
	.disable	= msm72k_disable,

	.alloc_request	= msm72k_alloc_request,
	.free_request	= msm72k_free_request,

	.queue		= msm72k_queue,
	.dequeue	= msm72k_dequeue,

	.set_halt	= msm72k_set_halt,
	.set_wedge	= msm72k_set_wedge,
	.fifo_status	= msm72k_fifo_status,
	.fifo_flush	= msm72k_fifo_flush,
};

static int msm72k_get_frame(struct usb_gadget *_gadget)
{
	struct usb_info *ui = container_of(_gadget, struct usb_info, gadget);

	/* frame number is in bits 13:3 */
	return (readl(USB_FRINDEX) >> 3) & 0x000007FF;
}

/* VBUS reporting logically comes from a transceiver */
static int msm72k_udc_vbus_session(struct usb_gadget *_gadget, int is_active)
{
	msm_hsusb_set_vbus_state_internal(is_active);
	return 0;
}

/*
 SW workarounds
 Issue #1	- USB Spoof Disconnect Failure
 Symptom	- Writing 0 to run/stop bit of USBCMD doesn't cause disconnect
 SW workaround	- Making opmode non-driving and SuspendM set in function
		register of SMSC phy
*/
/* drivers may have software control over D+ pullup */
static int msm72k_pullup_internal(struct usb_gadget *_gadget, int is_active)
{
	struct usb_info *ui = container_of(_gadget, struct usb_info, gadget);
	unsigned long flags;

	if (is_active) {
		spin_lock_irqsave(&ui->lock, flags);
		if (is_usb_online(ui) && ui->driver) {
			USB_INFO("%s: pullup\n", __func__);
			writel(readl(USB_USBCMD) | USBCMD_RS, USB_USBCMD);
		}
		spin_unlock_irqrestore(&ui->lock, flags);
	} else {
		writel(readl(USB_USBCMD) & ~USBCMD_RS, USB_USBCMD);
		/* S/W workaround, Issue#1 */
		ulpi_write(ui, 0x48, 0x04);
	}

	return 0;
}

static int msm72k_pullup(struct usb_gadget *_gadget, int is_active)
{
	struct usb_info *ui = container_of(_gadget, struct usb_info, gadget);

	atomic_set(&ui->softconnect, is_active);

	/* Reset PHY before enabling pull-up to workaround
	 * PHY stuck issue during mutiple times of function
	 * enable/disable.
	 */
	if (is_active)
		usb_reset(ui);
	else
		atomic_set(&ui->offline_pending, 1);

	msm72k_pullup_internal(_gadget, is_active);

	if (is_active && !ui->gadget.is_a_peripheral)
		schedule_delayed_work(&ui->chg_det, USB_CHG_DET_DELAY);

	return 0;
}

static int msm72k_wakeup(struct usb_gadget *_gadget)
{
	struct usb_info *ui = container_of(_gadget, struct usb_info, gadget);
	struct msm_otg *otg = to_msm_otg(ui->xceiv);

	if (!atomic_read(&ui->remote_wakeup)) {
		USB_ERR("%s: remote wakeup not supported\n", __func__);
		return -ENOTSUPP;
	}

	if (!atomic_read(&ui->configured)) {
		USB_ERR("%s: device is not configured\n", __func__);
		return -ENODEV;
	}
	otg_set_suspend(ui->xceiv, 0);

	disable_irq(otg->irq);

	if (!is_usb_active())
		writel(readl(USB_PORTSC) | PORTSC_FPR, USB_PORTSC);

	enable_irq(otg->irq);

	return 0;
}

static int msm72k_set_selfpowered(struct usb_gadget *_gadget, int set)
{
	struct usb_info *ui = container_of(_gadget, struct usb_info, gadget);
	struct msm_hsusb_gadget_platform_data *pdata =
				ui->pdev->dev.platform_data;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&ui->lock, flags);
	if (set) {
		if (pdata && pdata->self_powered)
			atomic_set(&ui->self_powered, 1);
		else
			ret = -EOPNOTSUPP;
	} else {
		/* We can always work as a bus powered device */
		atomic_set(&ui->self_powered, 0);
	}
	spin_unlock_irqrestore(&ui->lock, flags);

	return ret;

}

static const struct usb_gadget_ops msm72k_ops = {
	.get_frame	= msm72k_get_frame,
	.vbus_session	= msm72k_udc_vbus_session,
	.pullup		= msm72k_pullup,
	.wakeup		= msm72k_wakeup,
	.set_selfpowered = msm72k_set_selfpowered,
};

static void usb_do_remote_wakeup(struct work_struct *w)
{
	struct usb_info *ui = the_usb_info;

	msm72k_wakeup(&ui->gadget);
}

static ssize_t usb_remote_wakeup(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct usb_info *ui = the_usb_info;

	msm72k_wakeup(&ui->gadget);

	return count;
}

static ssize_t show_usb_state(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	size_t i;
	char *state[] = {"USB_STATE_NOTATTACHED", "USB_STATE_ATTACHED",
			"USB_STATE_POWERED", "USB_STATE_UNAUTHENTICATED",
			"USB_STATE_RECONNECTING", "USB_STATE_DEFAULT",
			"USB_STATE_ADDRESS", "USB_STATE_CONFIGURED",
			"USB_STATE_SUSPENDED"
	};

	i = scnprintf(buf, PAGE_SIZE, "%s\n", state[msm_hsusb_get_state()]);
	return i;
}

static ssize_t show_usb_speed(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct usb_info *ui = the_usb_info;
	size_t i;
	char *speed[] = {"USB_SPEED_UNKNOWN", "USB_SPEED_LOW",
			"USB_SPEED_FULL", "USB_SPEED_HIGH"};

	i = scnprintf(buf, PAGE_SIZE, "%s\n", speed[ui->gadget.speed]);
	return i;
}

static ssize_t show_usb_chg_type(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct usb_info *ui = the_usb_info;
	struct msm_otg *otg = to_msm_otg(ui->xceiv);
	size_t count;
	char *chg_type[] = {"STD DOWNSTREAM PORT",
			"CARKIT",
			"DEDICATED CHARGER",
			"INVALID"};

	count = sprintf(buf, "%s",
			chg_type[atomic_read(&otg->chg_type)]);

	return count;
}
static DEVICE_ATTR(wakeup, S_IWUSR, 0, usb_remote_wakeup);
static DEVICE_ATTR(usb_state, S_IRUSR, show_usb_state, 0);
static DEVICE_ATTR(usb_speed, S_IRUSR, show_usb_speed, 0);
static DEVICE_ATTR(chg_type, S_IRUSR, show_usb_chg_type, 0);

#ifdef CONFIG_USB_OTG
static ssize_t store_host_req(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct usb_info *ui = the_usb_info;
	unsigned long val, flags;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	USB_DEBUG("%s host request\n", val ? "set" : "clear");

	spin_lock_irqsave(&ui->lock, flags);
	if (ui->hnp_avail)
		ui->gadget.host_request = !!val;
	spin_unlock_irqrestore(&ui->lock, flags);

	return count;
}
static DEVICE_ATTR(host_request, S_IWUSR, NULL, store_host_req);

/* How do we notify user space about HNP availability?
 * As we are compliant to Rev 2.0, Host will not set a_hnp_support.
 * Introduce hnp_avail flag and set when HNP polling request arrives.
 * The expectation is that user space checks hnp availability before
 * requesting host role via above sysfs node.
 */
static ssize_t show_host_avail(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct usb_info *ui = the_usb_info;
	size_t count;
	unsigned long flags;

	spin_lock_irqsave(&ui->lock, flags);
	count = sprintf(buf, "%d\n", ui->hnp_avail);
	spin_unlock_irqrestore(&ui->lock, flags);

	return count;
}
static DEVICE_ATTR(host_avail, S_IRUSR, show_host_avail, NULL);

static struct attribute *otg_attrs[] = {
	&dev_attr_host_request.attr,
	&dev_attr_host_avail.attr,
	NULL,
};

static struct attribute_group otg_attr_grp = {
	.name  = "otg",
	.attrs = otg_attrs,
};
#endif
#if (defined(CONFIG_USB_ACCESSORY_DETECT) && defined(CONFIG_MSM_HDMI_MHL))
static void mhl_status_notifier_func(bool isMHL, bool irq_enble)
{
	struct usb_info *ui = the_usb_info;
	int id_pin = gpio_get_value(ui->usb_id_pin_gpio);
	static uint8_t mhl_connected;
	USB_INFO("%s: isMHL %d, id_pin %d\n", __func__, isMHL, id_pin);
#ifdef CONFIG_HTC_HEADSET_MISC
	headset_mhl_audio_jack_enable(isMHL);
#endif
	if (!isMHL && ui->usb_mhl_switch && ui->accessory_type == 4) {
		USB_INFO("MHL removed\n");
		ui->usb_mhl_switch(0);
		ui->accessory_type = 0;
#ifdef MHL_REDETECT
		if (mhl_connected == 0) {
			USB_INFO("MHL re-detect\n");
			set_irq_type(ui->idpin_irq,
				id_pin ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
			ui->cable_redetect = 1;
		}
#endif
		mhl_connected = 0;

		enable_irq(ui->idpin_irq);
		return;
	}

	mhl_connected = 1;
	set_irq_type(ui->idpin_irq,
		id_pin ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
}

static struct t_mhl_status_notifier mhl_status_notifier = {
	.name = "mhl_detect",
	.func = mhl_status_notifier_func,
};
#endif

/* The dedicated 9V detection GPIO will be high if VBUS is in and over 6V.
 * Since D+/D- status is not involved, there is no timing issue between
 * D+/D- and VBUS. 9V AC should NOT be found here.
 */
static void ac_detect_expired(unsigned long _data)
{
	struct usb_info *ui = (struct usb_info *) _data;
	u32 delay = 0;

	USB_INFO("%s: count = %d, connect_type = %d\n", __func__,
			ui->ac_detect_count, ui->connect_type);

	if (ui->connect_type == CONNECT_TYPE_USB || ui->ac_detect_count >= 3)
		return;

	/* detect shorted D+/D-, indicating AC power */
	if ((readl(USB_PORTSC) & PORTSC_LS) != PORTSC_LS) {

		/* Some carkit can't be recognized as AC mode.
		 * Add SW solution here to notify battery driver should
		 * work as AC charger when car mode activated.
		 */
#ifdef CONFIG_CABLE_DETECT_ACCESSORY
		if (cable_get_accessory_type() == DOCK_STATE_CAR) {
#else
		if (ui->accessory_type == 1) {
#endif
			USB_INFO("car mode charger\n");
			ui->connect_type = CONNECT_TYPE_AC;
			queue_work(ui->usb_wq, &ui->notifier_work);
			return;
		}

		ui->ac_detect_count++;
		/* detect delay: 3 sec, 5 sec, 10 sec */
		if (ui->ac_detect_count == 1)
			delay = 5 * HZ;
		else if (ui->ac_detect_count == 2)
			delay = 10 * HZ;
		mod_timer(&ui->ac_detect_timer, jiffies + delay);
	} else {
		if ((ui->usb_id_pin_gpio) &&
			gpio_get_value(ui->usb_id_pin_gpio) == 0) {
			USB_INFO("9V AC charger\n");
			ui->connect_type = CONNECT_TYPE_9V_AC;
		} else {
			USB_INFO("AC charger\n");
			ui->connect_type = CONNECT_TYPE_AC;
		}
		queue_work(ui->usb_wq, &ui->notifier_work);
	}
}

static int msm72k_probe(struct platform_device *pdev)
{
	struct usb_info *ui;
	struct msm_hsusb_platform_data *pdata;
	struct msm_otg *otg;
	int retval;
	char *serialno = "000000000000";

	USB_INFO("msm72k_probe\n");
	ui = kzalloc(sizeof(struct usb_info), GFP_KERNEL);
	if (!ui)
		return -ENOMEM;

	ui->pdev = pdev;

	if (pdev->dev.platform_data) {
		pdata = pdev->dev.platform_data;
		ui->phy_reset = pdata->phy_reset;
		ui->phy_init_seq = pdata->phy_init_seq;
		ui->usb_uart_switch = pdata->usb_uart_switch;
		ui->usb_id_pin_gpio = pdata->usb_id_pin_gpio;
		ui->accessory_detect = pdata->accessory_detect;
		ui->usb_mhl_switch = pdata->usb_mhl_switch;
		ui->ac_9v_gpio = pdata->ac_9v_gpio;
		ui->configure_ac_9v_gpio = pdata->configure_ac_9v_gpio;
		ui->config_usb_id_gpios = pdata->config_usb_id_gpios;
		ui->change_phy_voltage = pdata->change_phy_voltage;
		ui->usb_id2_pin_gpio = pdata->usb_id2_pin_gpio;
		ui->usb_host_switch = pdata->usb_host_switch;
		ui->idpin_irq = pdata->id_pin_irq;
	}

	ui->buf = dma_alloc_coherent(&pdev->dev, 4096, &ui->dma, GFP_KERNEL);
	if (!ui->buf)
		return usb_free(ui, -ENOMEM);

	ui->pool = dma_pool_create("msm72k_udc", NULL, 32, 32, 0);
	if (!ui->pool)
		return usb_free(ui, -ENOMEM);

	ui->xceiv = otg_get_transceiver();
	if (!ui->xceiv)
		return usb_free(ui, -ENODEV);

	otg = to_msm_otg(ui->xceiv);
	ui->addr = otg->regs;

	ui->gadget.ops = &msm72k_ops;
	ui->gadget.is_dualspeed = 1;
	device_initialize(&ui->gadget.dev);
	dev_set_name(&ui->gadget.dev, "gadget");
	ui->gadget.dev.parent = &pdev->dev;
	ui->gadget.dev.dma_mask = pdev->dev.dma_mask;

#ifdef CONFIG_USB_OTG
	ui->gadget.is_otg = 1;
#endif

	ui->sdev.name = DRIVER_NAME;
	ui->sdev.print_name = print_switch_name;
	ui->sdev.print_state = print_switch_state;

	retval = switch_dev_register(&ui->sdev);
	if (retval)
		return usb_free(ui, retval);

	the_usb_info = ui;

	wake_lock_init(&ui->wlock,
			WAKE_LOCK_SUSPEND, "usb_bus_active");

	usb_debugfs_init(ui);

	usb_prepare(ui);

#ifdef CONFIG_USB_OTG
	retval = sysfs_create_group(&pdev->dev.kobj, &otg_attr_grp);
	if (retval) {
		USB_ERR("failed to create otg sysfs directory:"
			"err:(%d)\n", retval);
	}
#endif

	retval = otg_set_peripheral(ui->xceiv, &ui->gadget);
	if (retval) {
		USB_ERR("%s: Cannot bind the transceiver, retval:(%d)\n",
			__func__, retval);
		switch_dev_unregister(&ui->sdev);
		wake_lock_destroy(&ui->wlock);
		return usb_free(ui, retval);
	}

	pm_runtime_enable(&pdev->dev);

	if (board_mfg_mode() == 1) {
		use_mfg_serialno = 1;
		wake_lock_init(&vbus_idle_wake_lock, WAKE_LOCK_IDLE,
				"usb_idle_lock");
		perf_lock_init(&usb_perf_lock, PERF_LOCK_HIGHEST, "usb");
	} else
		use_mfg_serialno = 0;
	strncpy(mfg_df_serialno, serialno, strlen(serialno));

	ui->connect_type_ready = 0;
#if (defined(CONFIG_USB_ACCESSORY_DETECT) && defined(CONFIG_MSM_HDMI_MHL))
	mhl_detect_register_notifier(&mhl_status_notifier);
#endif

	ui->ac_detect_count = 0;
	ui->ac_detect_timer.data = (unsigned long) ui;
	ui->ac_detect_timer.function = ac_detect_expired;
	init_timer(&ui->ac_detect_timer);
	return 0;
}

int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	struct usb_info *ui = the_usb_info;
	int			retval, n;

	if (!driver
			|| driver->speed < USB_SPEED_FULL
			|| !driver->bind
			|| !driver->disconnect
			|| !driver->setup)
		return -EINVAL;
	if (!ui)
		return -ENODEV;
	if (ui->driver)
		return -EBUSY;

	/* first hook up the driver ... */
	ui->driver = driver;
	ui->gadget.dev.driver = &driver->driver;
	ui->gadget.name = driver_name;
	INIT_LIST_HEAD(&ui->gadget.ep_list);
	ui->gadget.ep0 = &ui->ep0in.ep;
	INIT_LIST_HEAD(&ui->gadget.ep0->ep_list);
	ui->gadget.speed = USB_SPEED_UNKNOWN;
	atomic_set(&ui->softconnect, 1);

	for (n = 1; n < 16; n++) {
		struct msm_endpoint *ept = ui->ept + n;
		list_add_tail(&ept->ep.ep_list, &ui->gadget.ep_list);
		ept->ep.maxpacket = 512;
	}
	for (n = 17; n < 32; n++) {
		struct msm_endpoint *ept = ui->ept + n;
		list_add_tail(&ept->ep.ep_list, &ui->gadget.ep_list);
		ept->ep.maxpacket = 512;
	}

	retval = device_add(&ui->gadget.dev);
	if (retval)
		goto fail;

	retval = driver->bind(&ui->gadget);
	if (retval) {
		USB_ERR("bind to driver %s --> error %d\n",
				driver->driver.name, retval);
		device_del(&ui->gadget.dev);
		goto fail;
	}

	retval = device_create_file(&ui->gadget.dev, &dev_attr_wakeup);
	if (retval != 0)
		USB_ERR("failed to create sysfs entry:"
			"(wakeup) error: (%d)\n", retval);
	retval = device_create_file(&ui->gadget.dev, &dev_attr_usb_state);
	if (retval != 0)
		USB_ERR("failed to create sysfs entry:"
			" (usb_state) error: (%d)\n", retval);

	retval = device_create_file(&ui->gadget.dev, &dev_attr_usb_speed);
	if (retval != 0)
		USB_ERR("failed to create sysfs entry:"
			" (usb_speed) error: (%d)\n", retval);

	retval = device_create_file(&ui->gadget.dev, &dev_attr_chg_type);
	if (retval != 0)
		USB_ERR("failed to create sysfs entry(chg_type): err:(%d)\n",
					retval);

	usb_start(ui);

	USB_INFO("msm72k_udc: registered gadget driver '%s'\n",
			driver->driver.name);

	return 0;

fail:
	ui->driver = NULL;
	ui->gadget.dev.driver = NULL;
	return retval;
}
EXPORT_SYMBOL(usb_gadget_register_driver);

int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct usb_info *dev = the_usb_info;

	if (!dev)
		return -ENODEV;
	if (!driver || driver != dev->driver || !driver->unbind)
		return -EINVAL;

	msm72k_pullup_internal(&dev->gadget, 0);
	if (dev->irq) {
		free_irq(dev->irq, dev);
		dev->irq = 0;
	}
	dev->state = USB_STATE_IDLE;
	atomic_set(&dev->configured, 0);
	switch_set_state(&dev->sdev, 0);
	device_remove_file(&dev->gadget.dev, &dev_attr_wakeup);
	device_remove_file(&dev->gadget.dev, &dev_attr_usb_state);
	device_remove_file(&dev->gadget.dev, &dev_attr_usb_speed);
	device_remove_file(&dev->gadget.dev, &dev_attr_chg_type);
	driver->disconnect(&dev->gadget);
	driver->unbind(&dev->gadget);
	dev->gadget.dev.driver = NULL;
	dev->driver = NULL;

	device_del(&dev->gadget.dev);

	USB_DEBUG("unregistered gadget driver '%s'\n", driver->driver.name);
	return 0;
}
EXPORT_SYMBOL(usb_gadget_unregister_driver);

static struct platform_driver usb_driver = {
	.probe = msm72k_probe,
	.driver = {
		.name = "msm_hsusb",
	},
};

static int __init init(void)
{
	return platform_driver_register(&usb_driver);
}
module_init(init);

static void __exit cleanup(void)
{
	platform_driver_unregister(&usb_driver);
}
module_exit(cleanup);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Mike Lockwood, Brian Swetland");
MODULE_LICENSE("GPL");
