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

/*
 * SDIO-Abstraction-Layer Module.
 *
 * To be used with Qualcomm's SDIO-Client connected to this host.
 */

#include <linux/module.h>
#include <linux/scatterlist.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/wakelock.h>

#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>

#include <mach/sdio_al.h>

#define MODULE_NAME "sdio_al"
#define DRV_VERSION "1.09"

/* #define DEBUG_SDIO_AL_UNIT_TEST 1 */

/**
 *  Func#0 has SDIO standard registers
 *  Func#1 is for Mailbox.
 *  Functions 2..7 are for channels.
 *  Currently only functions 2..5 are active due to SDIO-Client
 *  number of pipes.
 *
 */
#define SDIO_AL_MAX_CHANNELS 4

/** Func 1..5 */
#define SDIO_AL_MAX_FUNCS    (SDIO_AL_MAX_CHANNELS+1)
#define SDIO_AL_WAKEUP_FUNC  6

/** Number of SDIO-Client pipes */
#define SDIO_AL_MAX_PIPES    16

/** CMD53/CMD54 Block size */
#define SDIO_AL_BLOCK_SIZE   128

/** Func#1 hardware Mailbox base address	 */
#define HW_MAILBOX_ADDR			0x1000

/** Func#1 peer sdioc software version.
 *  The header is duplicated also to the mailbox of the other
 *  functions. It can be used before other functions are enabled. */
#define SDIOC_SW_HEADER_ADDR		0x0400

/** Func#2..7 software Mailbox base address at 16K */
#define SDIOC_SW_MAILBOX_ADDR			0x4000

/** Some Mailbox registers address, written by host for
 control */
#define PIPES_THRESHOLD_ADDR		0x01000

#define PIPES_0_7_IRQ_MASK_ADDR 	0x01048

#define PIPES_8_15_IRQ_MASK_ADDR	0x0104C

#define EOT_PIPES_ENABLE		0x00

/** Maximum read/write data available is SDIO-Client limitation */
#define MAX_DATA_AVAILABLE   		(16*1024)
#define INVALID_DATA_AVAILABLE  	(0x8000)

/** SDIO-Client HW threshold to generate interrupt to the
 *  SDIO-Host on write available bytes.
 */
#define DEFAULT_WRITE_THRESHOLD 	(1024)

/** SDIO-Client HW threshold to generate interrupt to the
 *  SDIO-Host on read available bytes, for streaming (non
 *  packet) rx data.
 */
#define DEFAULT_READ_THRESHOLD  	(1024)

/** SW threshold to trigger reading the mailbox. */
#define DEFAULT_MIN_WRITE_THRESHOLD 	(1024)

#define THRESHOLD_DISABLE_VAL  		(0xFFFFFFFF)

#define DEFAULT_POLL_DELAY_MSEC		10

/** The SDIO-Client prepares N buffers of size X per Tx pipe.
 *  Even when the transfer fills a partial buffer,
 *  that buffer becomes unusable for the next transfer. */
#define DEFAULT_PEER_TX_BUF_SIZE	(128)

#define ROUND_UP(x, n) (((x + n - 1) / n) * n)

/** Func#2..7 FIFOs are r/w via
 sdio_readsb() & sdio_writesb(),when inc_addr=0 */
#define PIPE_RX_FIFO_ADDR   0x00
#define PIPE_TX_FIFO_ADDR   0x00

/** Inactivity time to go to sleep in mseconds */
#define INACTIVITY_TIME_MSEC 1000
#define INITIAL_INACTIVITY_TIME_MSEC 5000

/** Context validity check */
#define SDIO_AL_SIGNATURE 0xAABBCCDD

/* Vendor Specific Command */
#define SD_IO_RW_EXTENDED_QCOM 54

/** Channel priority */
enum sdio_priority {
	SDIO_PRIORITY_HIGH = 1,
	SDIO_PRIORITY_MED  = 5,
	SDIO_PRIORITY_LOW  = 9,
};

/**
 *  Mailbox structure.
 *  The Mailbox is located on the SDIO-Client Function#1.
 *  The mailbox size is 128 bytes, which is one block.
 *  The mailbox allows the host ton:
 *  1. Get the number of available bytes on the pipes.
 *  2. Enable/Disable SDIO-Client interrupt, related to pipes.
 *  3. Set the Threshold for generating interrupt.
 *
 */
struct sdio_mailbox {
	u32 pipe_bytes_threshold[SDIO_AL_MAX_PIPES]; /* Addr 0x1000 */

	/* Mask USER interrupts generated towards host - Addr 0x1040 */
	u32 mask_irq_func_1:8; /* LSB */
	u32 mask_irq_func_2:8;
	u32 mask_irq_func_3:8;
	u32 mask_irq_func_4:8;

	u32 mask_irq_func_5:8;
	u32 mask_irq_func_6:8;
	u32 mask_irq_func_7:8;
	u32 mask_mutex_irq:8;

	/* Mask PIPE interrupts generated towards host - Addr 0x1048 */
	u32 mask_eot_pipe_0_7:8;
	u32 mask_thresh_above_limit_pipe_0_7:8;
	u32 mask_overflow_pipe_0_7:8;
	u32 mask_underflow_pipe_0_7:8;

	u32 mask_eot_pipe_8_15:8;
	u32 mask_thresh_above_limit_pipe_8_15:8;
	u32 mask_overflow_pipe_8_15:8;
	u32 mask_underflow_pipe_8_15:8;

	/* Status of User interrupts generated towards host - Addr 0x1050 */
	u32 user_irq_func_1:8;
	u32 user_irq_func_2:8;
	u32 user_irq_func_3:8;
	u32 user_irq_func_4:8;

	u32 user_irq_func_5:8;
	u32 user_irq_func_6:8;
	u32 user_irq_func_7:8;
	u32 user_mutex_irq:8;

	/* Status of PIPE interrupts generated towards host */
	/* Note: All sources are cleared once they read. - Addr 0x1058 */
	u32 eot_pipe_0_7:8;
	u32 thresh_above_limit_pipe_0_7:8;
	u32 overflow_pipe_0_7:8;
	u32 underflow_pipe_0_7:8;

	u32 eot_pipe_8_15:8;
	u32 thresh_above_limit_pipe_8_15:8;
	u32 overflow_pipe_8_15:8;
	u32 underflow_pipe_8_15:8;

	u16 pipe_bytes_avail[SDIO_AL_MAX_PIPES];
};

/** Track pending Rx Packet size */
struct rx_packet_size {
	u32 size; /* in bytes */
	struct list_head	list;
};

/**
 * Peer SDIO-Client channel configuration.
 *
 *  @is_ready - channel is ready and the data is valid.
 *
 *  @max_rx_threshold - maximum rx threshold, according to the
 *  		      total buffers size on the peer pipe.
 *  @max_tx_threshold - maximum tx threshold, according to the
 *  		      total buffers size on the peer pipe.
 *  @tx_buf_size - size of a single buffer on the peer pipe; a
 *  		 transfer smaller than the buffer size still
 *  		 make the buffer unusable for the next transfer.
 * @max_packet_size
 * @is_host_ok_to_sleep - Host marks this bit when it's okay to
 *      		sleep (no pending transactions)
 */
struct peer_sdioc_channel_config {
	u32 is_ready;
	u32 max_rx_threshold; /* Downlink */
	u32 max_tx_threshold; /* Uplink */
	u32 tx_buf_size;
	u32 max_packet_size;
	u32 is_host_ok_to_sleep;
	u32 reserved[26];
};

#define PEER_SDIOC_SW_MAILBOX_SIGNATURE 0xFACECAFE

#define PEER_SDIOC_VERSION		0x20001


/**
 * Peer SDIO-Client software header.
 */
struct peer_sdioc_sw_header {
	u32 signature;
	u32 version;
	u32 max_channels;
	u32 reserved[29];
};

/**
 * Peer SDIO-Client software mailbox.
 */
struct peer_sdioc_sw_mailbox {
	struct peer_sdioc_sw_header sw_header;
	struct peer_sdioc_channel_config ch_config[6];
};

/**
 *  SDIO Channel context.
 *
 *  @name - channel name. Used by the caller to open the
 *  	  channel.
 *
 *  @priority - channel priority.
 *
 *  @read_threshold - Threshold on SDIO-Client mailbox for Rx
 *  				Data available bytes. When the limit exceed
 *  				the SDIO-Client generates an interrupt to the
 *  				host.
 *
 *  @write_threshold - Threshold on SDIO-Client mailbox for Tx
 *  				Data available bytes. When the limit exceed
 *  				the SDIO-Client generates an interrupt to the
 *  				host.
 *
 *  @def_read_threshold - Default theshold on SDIO-Client for Rx
 *
 *  @min_write_avail - Threshold of minimal available bytes
 *  					 to write. Below that threshold the host
 *  					 will initiate reading the mailbox.
 *
 *  @poll_delay_msec - Delay between polling the mailbox. When
 *  				 the SDIO-Client doesn't generates EOT
 *  				 interrupt for Rx Available bytes, the host
 *  				 should poll the SDIO-Client mailbox.
 *
 *  @is_packet_mode - The host get interrupt when a packet is
 *  				available at the SDIO-client (pipe EOT
 *  				indication).
 *
 *  @num - channel number.
 *
 *  @notify - Client's callback. Should not call sdio read/write.
 *
 *  @priv - Client's private context, provided to callback.
 *
 *  @is_open - Channel is open.
 *
 *  @func - SDIO Function handle.
 *
 *  @rx_pipe_index - SDIO-Client Pipe Index for Rx Data.
 *
 *  @tx_pipe_index - SDIO-Client Pipe Index for Tx Data.
 *
 *  @ch_lock - Channel lock to protect channel specific Data
 *
 *  @rx_pending_bytes - Total number of Rx pending bytes, at Rx
 *  				  packet list. Maximum of 16KB-1 limited by
 *  				  SDIO-Client specification.
 *
 *  @read_avail - Available bytes to read.
 *
 *  @write_avail - Available bytes to write.
 *
 *  @rx_size_list_head - The head of Rx Pending Packets List.
 *
 *  @pdev - platform device - clients to probe for the sdio-al.
 *
 *  @signature - Context Validity check.
 *
 */
struct sdio_channel {
	/* Channel Configuration Parameters*/
	const char *name;
	int priority;
	int read_threshold;
	int write_threshold;
	int def_read_threshold;
	int min_write_avail;
	int poll_delay_msec;
	int is_packet_mode;

	/* Channel Info */
	int num;

	void (*notify)(void *priv, unsigned channel_event);
	void *priv;

	int is_open;
	int is_suspend;

	struct sdio_func *func;

	int rx_pipe_index;
	int tx_pipe_index;

	struct mutex ch_lock;

	u32 read_avail;
	u32 write_avail;

	u32 peer_tx_buf_size;

	u16 rx_pending_bytes;

	struct list_head rx_size_list_head;

	struct platform_device pdev;

	u32 total_rx_bytes;
	u32 total_tx_bytes;

	u32 signature;
};

/**
 *  SDIO Abstraction Layer driver context.
 *
 *  @card - card claimed.
 *
 *  @mailbox - A shadow of the SDIO-Client mailbox.
 *
 *  @channel - Channels context.
 *
 *  @workqueue - workqueue to read the mailbox and handle
 *  		   pending requests according to priority.
 *  		   Reading the mailbox should not happen in
 *  		   interrupt context.
 *
 *  @work - work to submit to workqueue.
 *
 *  @is_ready - driver is ready.
 *
 *  @ask_mbox - Flag to request reading the mailbox,
 *					  for different reasons.
 *
 *  @wake_lock - Lock when can't sleep.
 *
 *  @lpm_chan - Channel to use for LPM (low power mode)
 *            communication.
 *
 *  @is_ok_to_sleep - Mark if driver is OK with going to sleep
 * 			(no pending transactions).
 *
 *  @inactivity_time - time allowed to be in inactivity before
 * 		going to sleep
 *
 *  @timer - timer to use for polling the mailbox.
 *
 *  @poll_delay_msec - timer delay for polling the mailbox.
 *
 *  @use_irq - allow to work in polling mode or interrupt mode.
 *
 *  @is_err - error detected.
 *
 *  @signature - Context Validity Check.
 *
 */
struct sdio_al {
	struct mmc_card *card;
	struct sdio_mailbox *mailbox;
	struct sdio_channel channel[SDIO_AL_MAX_CHANNELS];

	struct peer_sdioc_sw_header *sdioc_sw_header;

	struct workqueue_struct *workqueue;
	struct work_struct work;

	int is_ready;

	wait_queue_head_t   wait_mbox;
	int ask_mbox;

	struct wake_lock wake_lock;
	int lpm_chan;
	int is_ok_to_sleep;
	unsigned long inactivity_time;

	struct timer_list timer;
	u32 poll_delay_msec;

	int use_irq;

	int is_err;

	u32 signature;
};

/** The driver context */
static struct sdio_al *sdio_al;

/* Static functions declaration */
static int enable_eot_interrupt(int pipe_index, int enable);
static int enable_threshold_interrupt(int pipe_index, int enable);
static void sdio_func_irq(struct sdio_func *func);
static void timer_handler(unsigned long data);
static int get_min_poll_time_msec(void);
static u32 check_pending_rx_packet(struct sdio_channel *ch, u32 eot);
static u32 remove_handled_rx_packet(struct sdio_channel *ch);
static int set_pipe_threshold(int pipe_index, int threshold);

/**
 *  Write SDIO-Client lpm information
 *  Should only be called with host claimed.
 */
static int write_lpm_info(void)
{
	struct sdio_func *lpm_func =
		sdio_al->card->sdio_func[sdio_al->lpm_chan+1];
	int offset = offsetof(struct peer_sdioc_sw_mailbox, ch_config)+
		sizeof(struct peer_sdioc_channel_config) * sdio_al->lpm_chan+
		offsetof(struct peer_sdioc_channel_config, is_host_ok_to_sleep);
	int ret;

	pr_debug(MODULE_NAME ":writing lpm info is_ok_to_sleep=%d\n",
		 sdio_al->is_ok_to_sleep);

	ret = sdio_memcpy_toio(lpm_func, SDIOC_SW_MAILBOX_ADDR+offset,
				&sdio_al->is_ok_to_sleep, sizeof(u32));
	if (ret)
		pr_err(MODULE_NAME ":fail to write sdioc sw header.\n");

	return ret;
}

/* Set inactivity counter to intial value to allow clients come up */
static inline void start_inactive_time(void)
{
	sdio_al->inactivity_time = jiffies +
		msecs_to_jiffies(INITIAL_INACTIVITY_TIME_MSEC);
}

static inline void restart_inactive_time(void)
{
	sdio_al->inactivity_time = jiffies +
		msecs_to_jiffies(INACTIVITY_TIME_MSEC);
}

static inline int is_inactive_time_expired(void)
{
	return time_after(jiffies, sdio_al->inactivity_time);
}

/**
 *  Read SDIO-Client Mailbox from Function#1.thresh_pipe
 *
 *  The mailbox contain the bytes available per pipe,
 *  and the End-Of-Transfer indication per pipe (if available).
 *
 * WARNING: Each time the Mailbox is read from the client, the
 * read_bytes_avail is incremented with another pending
 * transfer. Therefore, a pending rx-packet should be added to a
 * list before the next read of the mailbox.
 *
 * This function should run from a workqueue context since it
 * notifies the clients.
 *
 */
static int read_mailbox(int from_isr)
{
	int ret;
	struct sdio_func *func1 = sdio_al->card->sdio_func[0];
	struct sdio_mailbox *mailbox = sdio_al->mailbox;
	u32 new_write_avail = 0;
	u32 old_write_avail = 0;
	u32 any_read_avail = 0;
	int i;
	u32 rx_notify_bitmask = 0;
	u32 tx_notify_bitmask = 0;
	u32 eot_pipe = 0;
	u32 thresh_pipe = 0;
	u32 overflow_pipe = 0;
	u32 underflow_pipe = 0;
	u32 thresh_intr_mask = 0;

	if (sdio_al->is_err) {
		pr_info(MODULE_NAME ":In Error state, ignore request\n");
		return 0;
	}

	pr_debug(MODULE_NAME ":start %s from_isr = %d.\n", __func__, from_isr);

	if (!from_isr)
		sdio_claim_host(sdio_al->card->sdio_func[0]);
	pr_debug(MODULE_NAME ":before sdio_memcpy_fromio.\n");
	ret = sdio_memcpy_fromio(func1, mailbox,
			HW_MAILBOX_ADDR, sizeof(*mailbox));
	pr_debug(MODULE_NAME ":after sdio_memcpy_fromio.\n");

	eot_pipe =	(mailbox->eot_pipe_0_7) |
			(mailbox->eot_pipe_8_15<<8);
	thresh_pipe = 	(mailbox->thresh_above_limit_pipe_0_7) |
			(mailbox->thresh_above_limit_pipe_8_15<<8);

	overflow_pipe = (mailbox->overflow_pipe_0_7) |
			(mailbox->overflow_pipe_8_15<<8);
	underflow_pipe = mailbox->underflow_pipe_0_7 |
			(mailbox->underflow_pipe_8_15<<8);
	thresh_intr_mask =
		(mailbox->mask_thresh_above_limit_pipe_0_7) |
		(mailbox->mask_thresh_above_limit_pipe_8_15<<8);

	if (ret) {
		pr_err(MODULE_NAME ":Fail to read Mailbox,"
				    " goto error state\n");
		sdio_al->is_err = true;
		/* Stop the timer to stop reading the mailbox */
		sdio_al->poll_delay_msec = 0;
		goto exit_err;
	}

	if (overflow_pipe || underflow_pipe)
		pr_info(MODULE_NAME ":Mailbox ERROR "
				"overflow=0x%x, underflow=0x%x\n",
				overflow_pipe, underflow_pipe);


	pr_debug(MODULE_NAME ":eot=0x%x, thresh=0x%x\n",
			 eot_pipe, thresh_pipe);

	/* Scan for Rx Packets available and update read vailable bytes */
	for (i = 0; i < SDIO_AL_MAX_CHANNELS; i++) {
		struct sdio_channel *ch = &sdio_al->channel[i];
		u32 old_read_avail;
		u32 read_avail;
		u32 new_packet_size = 0;

		if (!ch->is_open)
			continue;

		old_read_avail = ch->read_avail;
		read_avail = mailbox->pipe_bytes_avail[ch->rx_pipe_index];

		if (read_avail > INVALID_DATA_AVAILABLE) {
			pr_info(MODULE_NAME
				 ":Invalid read_avail 0x%x for pipe %d\n",
				 read_avail, ch->rx_pipe_index);
			continue;
		}
		any_read_avail |= read_avail | old_read_avail;

		if (ch->is_packet_mode)
			new_packet_size = check_pending_rx_packet(ch, eot_pipe);
		else
			ch->read_avail = read_avail;

		if ((ch->is_packet_mode) && (new_packet_size > 0))
			rx_notify_bitmask |= (1<<ch->num);

		if ((!ch->is_packet_mode) && (ch->read_avail > 0) &&
		    (old_read_avail == 0))
			rx_notify_bitmask |= (1<<ch->num);
	}

	/* Update Write available */
	for (i = 0; i < SDIO_AL_MAX_CHANNELS; i++) {
		struct sdio_channel *ch = &sdio_al->channel[i];

		if (!ch->is_open)
			continue;

		new_write_avail = mailbox->pipe_bytes_avail[ch->tx_pipe_index];

		if (new_write_avail > INVALID_DATA_AVAILABLE) {
			pr_info(MODULE_NAME
				 ":Invalid write_avail 0x%x for pipe %d\n",
				 new_write_avail, ch->tx_pipe_index);
			continue;
		}

		old_write_avail = ch->write_avail;
		ch->write_avail = new_write_avail;

		if ((old_write_avail <= ch->min_write_avail) &&
			(new_write_avail >= ch->min_write_avail))
			tx_notify_bitmask |= (1<<ch->num);
	}

	if ((rx_notify_bitmask == 0) && (tx_notify_bitmask == 0) &&
	    !any_read_avail) {
		pr_debug(MODULE_NAME ":Nothing to Notify\n");
	} else {
		pr_info(MODULE_NAME ":Notify bitmask rx=0x%x, tx=0x%x.\n",
			rx_notify_bitmask, tx_notify_bitmask);
		/* Restart inactivity timer if any activity on the channel */
		restart_inactive_time();
	}

	for (i = 0; i < SDIO_AL_MAX_CHANNELS; i++) {
		struct sdio_channel *ch = &sdio_al->channel[i];

		if ((!ch->is_open) || (ch->notify == NULL))
			continue;

		if (rx_notify_bitmask & (1<<ch->num))
			ch->notify(ch->priv,
					   SDIO_EVENT_DATA_READ_AVAIL);

		if (tx_notify_bitmask & (1<<ch->num))
			ch->notify(ch->priv,
					   SDIO_EVENT_DATA_WRITE_AVAIL);
	}

	/* Enable/Disable of Interrupts */
	for (i = 0; i < SDIO_AL_MAX_CHANNELS; i++) {
		struct sdio_channel *ch = &sdio_al->channel[i];
		u32 pipe_thresh_intr_disabled = 0;

		if (!ch->is_open)
			continue;


		pipe_thresh_intr_disabled = thresh_intr_mask &
			(1<<ch->tx_pipe_index);
	}

	if (is_inactive_time_expired()) {
		/* Go to sleep */
		pr_info(MODULE_NAME  "Inactivity timer expired."
			" Going to sleep\n");
		/* Stop mailbox timer */
		sdio_al->poll_delay_msec = 0;
		del_timer_sync(&sdio_al->timer);
		/* Make sure we get interrupt for non-packet-mode right away */
		for (i = 0; i < SDIO_AL_MAX_CHANNELS; i++) {
			struct sdio_channel *ch = &sdio_al->channel[i];
			if (ch->is_packet_mode == false) {
				ch->read_threshold = 1;
				set_pipe_threshold(ch->rx_pipe_index,
						   ch->read_threshold);
			}
		}
		/* Mark HOST_OK_TOSLEEP */
		sdio_al->is_ok_to_sleep = 1;
		write_lpm_info();
		pr_info(MODULE_NAME "Finished sleep sequence. Sleep now.\n");
		/* Release wakelock */
		wake_unlock(&sdio_al->wake_lock);
	}

	pr_debug(MODULE_NAME ":end %s.\n", __func__);

exit_err:
	if (!from_isr)
		sdio_release_host(sdio_al->card->sdio_func[0]);


	return ret;
}

/**
 *  Check pending rx packet when reading the mailbox.
 */
static u32 check_pending_rx_packet(struct sdio_channel *ch, u32 eot)
{
	u32 rx_pending;
	u32 rx_avail;
	u32 new_packet_size = 0;

	mutex_lock(&ch->ch_lock);

	rx_pending = ch->rx_pending_bytes;
	rx_avail = sdio_al->mailbox->pipe_bytes_avail[ch->rx_pipe_index];

	pr_debug(MODULE_NAME ":pipe %d rx_avail=0x%x , rx_pending=0x%x\n",
	   ch->rx_pipe_index, rx_avail, rx_pending);


	/* new packet detected */
	if (eot & (1<<ch->rx_pipe_index)) {
		struct rx_packet_size *p = NULL;
		new_packet_size = rx_avail - rx_pending;

		if ((rx_avail <= rx_pending)) {
			pr_info(MODULE_NAME ":Invalid new packet size."
					    " rx_avail=%d.\n", rx_avail);
			new_packet_size = 0;
			goto exit_err;
		}

		p = kzalloc(sizeof(*p), GFP_KERNEL);
		if (p == NULL)
			goto exit_err;
		p->size = new_packet_size;
		/* Add new packet as last */
		list_add_tail(&p->list, &ch->rx_size_list_head);
		ch->rx_pending_bytes += new_packet_size;

		if (ch->read_avail == 0)
			ch->read_avail = new_packet_size;
	}

exit_err:
	mutex_unlock(&ch->ch_lock);

	return new_packet_size;
}



/**
 *  Remove first pending packet from the list.
 */
static u32 remove_handled_rx_packet(struct sdio_channel *ch)
{
	struct rx_packet_size *p = NULL;

	mutex_lock(&ch->ch_lock);

	ch->rx_pending_bytes -= ch->read_avail;

	if (!list_empty(&ch->rx_size_list_head)) {
		p = list_first_entry(&ch->rx_size_list_head,
			struct rx_packet_size, list);
		list_del(&p->list);
		kfree(p);
	}

	if (list_empty(&ch->rx_size_list_head))	{
		ch->read_avail = 0;
	} else {
		p = list_first_entry(&ch->rx_size_list_head,
			struct rx_packet_size, list);
		ch->read_avail = p->size;
	}

	mutex_unlock(&ch->ch_lock);

	return ch->read_avail;
}

/**
 *  Worker function.
 *
 *  @note: clear the ask_mbox flag only after
 *  	 reading the mailbox, to ignore more requests while
 *  	 reading the mailbox.
 */
static void worker(struct work_struct *work)
{
	int ret = 0;

	pr_debug(MODULE_NAME ":Worker Started..\n");
	while ((sdio_al->is_ready) && (ret == 0)) {
		pr_debug(MODULE_NAME ":Wait for read mailbox request..\n");
		wait_event(sdio_al->wait_mbox, sdio_al->ask_mbox);
		ret = read_mailbox(false);
		sdio_al->ask_mbox = false;
	}
	pr_debug(MODULE_NAME ":Worker Exit!\n");
}

/**
 *  Write command using CMD54 rather than CMD53.
 *  Writing with CMD54 generate EOT interrupt at the
 *  SDIO-Client.
 *  Based on mmc_io_rw_extended()
 */
static int sdio_write_cmd54(struct mmc_card *card, unsigned fn,
	unsigned addr, const u8 *buf,
	unsigned blocks, unsigned blksz)
{
	struct mmc_request mrq;
	struct mmc_command cmd;
	struct mmc_data data;
	struct scatterlist sg;
	int incr_addr = 1; /* MUST */
	int write = 1;

	BUG_ON(!card);
	BUG_ON(fn > 7);
	BUG_ON(blocks == 1 && blksz > 512);
	WARN_ON(blocks == 0);
	WARN_ON(blksz == 0);

	write = true;
	pr_debug(MODULE_NAME ":sdio_write_cmd54()"
		"fn=%d,buf=0x%x,blocks=%d,blksz=%d\n",
		fn, (u32) buf, blocks, blksz);

	memset(&mrq, 0, sizeof(struct mmc_request));
	memset(&cmd, 0, sizeof(struct mmc_command));
	memset(&data, 0, sizeof(struct mmc_data));

	mrq.cmd = &cmd;
	mrq.data = &data;

	cmd.opcode = SD_IO_RW_EXTENDED_QCOM;

	cmd.arg = write ? 0x80000000 : 0x00000000;
	cmd.arg |= fn << 28;
	cmd.arg |= incr_addr ? 0x04000000 : 0x00000000;
	cmd.arg |= addr << 9;
	if (blocks == 1 && blksz <= 512)
		cmd.arg |= (blksz == 512) ? 0 : blksz;  /* byte mode */
	else
		cmd.arg |= 0x08000000 | blocks; 	/* block mode */
	cmd.flags = MMC_RSP_SPI_R5 | MMC_RSP_R5 | MMC_CMD_ADTC;

	data.blksz = blksz;
	data.blocks = blocks;
	data.flags = write ? MMC_DATA_WRITE : MMC_DATA_READ;
	data.sg = &sg;
	data.sg_len = 1;

	sg_init_one(&sg, buf, blksz * blocks);

	mmc_set_data_timeout(&data, card);

	mmc_wait_for_req(card->host, &mrq);

	if (cmd.error)
		return cmd.error;
	if (data.error)
		return data.error;

	if (mmc_host_is_spi(card->host)) {
		/* host driver already reported errors */
	} else {
		if (cmd.resp[0] & R5_ERROR)
			return -EIO;
		if (cmd.resp[0] & R5_FUNCTION_NUMBER)
			return -EINVAL;
		if (cmd.resp[0] & R5_OUT_OF_RANGE)
			return -ERANGE;
	}

	return 0;
}


/**
 *  Write data to channel.
 *  Handle different data size types.
 *
 */
static int sdio_ch_write(struct sdio_channel *ch, const u8 *buf, u32 len)
{
	int ret = 0;
	unsigned blksz = ch->func->cur_blksize;
	int blocks = len / blksz;
	int remain_bytes = len % blksz;
	struct mmc_card *card = NULL;
	u32 fn = ch->func->num;

	if (len == 0)
		return -EINVAL;

	card = ch->func->card;

	if (remain_bytes) {
		/* CMD53 */
		if (blocks)
			ret = sdio_memcpy_toio(ch->func, PIPE_TX_FIFO_ADDR,
					       (void *) buf, blocks*blksz);

		if (ret != 0)
			return ret;

		buf += (blocks*blksz);

		ret = sdio_write_cmd54(card, fn, PIPE_TX_FIFO_ADDR,
				buf, 1, remain_bytes);
	} else {
		ret = sdio_write_cmd54(card, fn, PIPE_TX_FIFO_ADDR,
				buf, blocks, blksz);
	}

	return ret;
}

/**
 * Set Channels Configuration.
 *
 * @todo read channel configuration from platform data rather
 *  	  than hard-coded.
 *
 * @note  When agregation is used there is No EOT interrupt
 *  	  for Rx packet ready, therefore need polling.
 */
static void set_default_channels_config(void)
{
	int i;

	/* Set Default Channels configuration */
	for (i = 0; i < SDIO_AL_MAX_CHANNELS; i++) {
		struct sdio_channel *ch = &sdio_al->channel[i];

		ch->num = i;
		ch->read_threshold  = DEFAULT_READ_THRESHOLD;
		ch->write_threshold = DEFAULT_WRITE_THRESHOLD;
		ch->min_write_avail = DEFAULT_MIN_WRITE_THRESHOLD;
		if (sdio_al->use_irq)
			ch->poll_delay_msec = 0;
		else
			ch->poll_delay_msec = DEFAULT_POLL_DELAY_MSEC;
		ch->is_packet_mode = true;
		ch->peer_tx_buf_size = DEFAULT_PEER_TX_BUF_SIZE;
	}


	sdio_al->channel[0].name = "SDIO_RPC";
	sdio_al->channel[0].priority = SDIO_PRIORITY_HIGH;

	sdio_al->channel[1].name = "SDIO_RMNET_DATA";
	sdio_al->channel[1].priority = SDIO_PRIORITY_MED;
	sdio_al->channel[1].is_packet_mode = false;  /* No EOT for Rx Data */
	sdio_al->channel[1].poll_delay_msec = 30;

	sdio_al->channel[1].read_threshold  = 14*1024;
	sdio_al->channel[1].write_threshold = 2*1024;
	sdio_al->channel[1].min_write_avail = 1600;
	sdio_al->lpm_chan = 1;

	sdio_al->channel[2].name = "SDIO_QMI";
	sdio_al->channel[2].priority = SDIO_PRIORITY_LOW;

	sdio_al->channel[3].name = "SDIO_DIAG";
	sdio_al->channel[3].priority = SDIO_PRIORITY_LOW;
}


/**
 *  Read SDIO-Client software header
 *
 */
static int read_sdioc_software_header(struct peer_sdioc_sw_header *header)
{
	int ret;
	struct sdio_func *func1 = sdio_al->card->sdio_func[0];

	pr_debug(MODULE_NAME ":reading sdioc sw header.\n");

	ret = sdio_memcpy_fromio(func1, header,
			SDIOC_SW_HEADER_ADDR, sizeof(*header));
	if (ret) {
		pr_err(MODULE_NAME ":fail to read sdioc sw header.\n");
		goto exit_err;
	}

	if (header->signature != (u32) PEER_SDIOC_SW_MAILBOX_SIGNATURE) {
		pr_err(MODULE_NAME ":SDIOC SW invalid signature. 0x%x\n",
			header->signature);
		goto exit_err;
	}
	/* Upper byte has to be equal - no backward compatibtiyy for unequal */
	if ((header->version << 16) != (PEER_SDIOC_VERSION << 16)) {
		pr_err(MODULE_NAME ":SDIOC SW older version. 0x%x\n",
			header->version);
		goto exit_err;
	}

	pr_info(MODULE_NAME ":SDIOC SW version 0x%x\n", header->version);

	return 0;

exit_err:
	sdio_al->is_err = true;
	memset(header, 0, sizeof(*header));

	return -1;
}

/**
 *  Read SDIO-Client channel configuration
 *
 */
static int read_sdioc_channel_config(struct sdio_channel *ch)
{
	int ret;
	struct peer_sdioc_sw_mailbox *sw_mailbox = NULL;
	struct peer_sdioc_channel_config *ch_config = NULL;

	if (sdio_al->sdioc_sw_header->version == 0)
		return -1;

	pr_debug(MODULE_NAME ":reading sw mailbox %s channel.\n", ch->name);

	sw_mailbox = kzalloc(sizeof(*sw_mailbox), GFP_KERNEL);
	if (sw_mailbox == NULL)
		return -ENOMEM;

	ret = sdio_memcpy_fromio(ch->func, sw_mailbox,
			SDIOC_SW_MAILBOX_ADDR, sizeof(*sw_mailbox));
	if (ret) {
		pr_info(MODULE_NAME ":fail to read sw mailbox.\n");
		goto exit_err;
	}

	ch_config = &sw_mailbox->ch_config[ch->num];

	if (!ch_config->is_ready) {
		pr_info(MODULE_NAME ":sw mailbox channel not ready.\n");
		goto exit_err;
	}

	pr_info(MODULE_NAME ":ch %s max_rx_threshold=%d.\n",
		ch->name, ch_config->max_rx_threshold);
	pr_info(MODULE_NAME ":ch %s max_tx_threshold=%d.\n",
		ch->name, ch_config->max_tx_threshold);
	pr_info(MODULE_NAME ":ch %s tx_buf_size=%d.\n",
		ch->name, ch_config->tx_buf_size);

	/* Aggregation up to 90% of the maximum size */
	ch_config->max_rx_threshold = (ch_config->max_rx_threshold * 9) / 10;
	/* Threshold on 50% of the maximum size , sdioc uses double-buffer */
	ch_config->max_tx_threshold = (ch_config->max_tx_threshold * 5) / 10;

	ch->read_threshold = min(ch->read_threshold,
				 (int) ch_config->max_rx_threshold);

	ch->write_threshold = min(ch->write_threshold,
				 (int) ch_config->max_tx_threshold);

	ch->def_read_threshold = ch->read_threshold;

	if (ch->min_write_avail > ch->write_threshold)
		ch->min_write_avail = ch->write_threshold;

	ch->peer_tx_buf_size = ch_config->tx_buf_size;

	kfree(sw_mailbox);

	return 0;

exit_err:
	pr_info(MODULE_NAME ":Reading SW Mailbox error.\n");
	kfree(sw_mailbox);

	return -1;
}


/**
 *  Enable/Disable EOT interrupt of a pipe.
 *
 */
static int enable_eot_interrupt(int pipe_index, int enable)
{
	int ret = 0;
	struct sdio_func *func1 = sdio_al->card->sdio_func[0];
	u32 mask;
	u32 pipe_mask;
	u32 addr;

	if (pipe_index < 8) {
		addr = PIPES_0_7_IRQ_MASK_ADDR;
		pipe_mask = (1<<pipe_index);
	} else {
		addr = PIPES_8_15_IRQ_MASK_ADDR;
		pipe_mask = (1<<(pipe_index-8));
	}

	mask = sdio_readl(func1, addr, &ret);
	if (ret) {
		pr_debug(MODULE_NAME ":enable_eot_interrupt fail\n");
		goto exit_err;
	}

	if (enable)
		mask &= (~pipe_mask); /* 0 = enable */
	else
		mask |= (pipe_mask);  /* 1 = disable */

	sdio_writel(func1, mask, addr, &ret);

exit_err:
	return ret;
}

/**
 *  Enable/Disable Threshold interrupt of a pipe.
 *
 */
static int enable_threshold_interrupt(int pipe_index, int enable)
{
	int ret = 0;
	struct sdio_func *func1 = sdio_al->card->sdio_func[0];
	u32 mask;
	u32 pipe_mask;
	u32 addr;

	if (pipe_index < 8) {
		addr = PIPES_0_7_IRQ_MASK_ADDR;
		pipe_mask = (1<<pipe_index);
	} else {
		addr = PIPES_8_15_IRQ_MASK_ADDR;
		pipe_mask = (1<<(pipe_index-8));
	}

	mask = sdio_readl(func1, addr, &ret);
	if (ret) {
		pr_debug(MODULE_NAME ":enable_threshold_interrupt fail\n");
		goto exit_err;
	}

	pipe_mask = pipe_mask<<8; /* Threshold bits 8..15 */
	if (enable)
		mask &= (~pipe_mask); /* 0 = enable */
	else
		mask |= (pipe_mask);  /* 1 = disable */

	sdio_writel(func1, mask, addr, &ret);

exit_err:
	return ret;
}

/**
 *  Set the threshold to trigger interrupt from SDIO-Card on
 *  pipe available bytes.
 *
 */
static int set_pipe_threshold(int pipe_index, int threshold)
{
	int ret = 0;
	struct sdio_func *func1 = sdio_al->card->sdio_func[0];

	sdio_writel(func1, threshold,
			PIPES_THRESHOLD_ADDR+pipe_index*4, &ret);
	if (ret)
		pr_err(MODULE_NAME ":set_pipe_threshold err=%d\n", -ret);

	return ret;
}

/**
 *  Enable func w/ retries
 *
 */
static int sdio_al_enable_func_retry(struct sdio_func *func, const char *name)
{
	int ret, i;
	for (i = 0; i < 10; i++) {
		ret = sdio_enable_func(func);
		if (ret) {
			pr_err(MODULE_NAME ":retry enable %s func#%d red=%d\n",
					 name, func->num, ret);
			msleep(500);
		} else
			break;
	}

	return ret;
}

/**
 *  Open Channel
 *
 *  1. Init Channel Context.
 *  2. Init the Channel SDIO-Function.
 *  3. Init the Channel Pipes on Mailbox.
 */
static int open_channel(struct sdio_channel *ch)
{
	int ret = 0;

	/* Init channel Context */
	/** Func#1 is reserved for mailbox */
	ch->func = sdio_al->card->sdio_func[ch->num+1];
	ch->rx_pipe_index = ch->num*2;
	ch->tx_pipe_index = ch->num*2+1;
	ch->signature = SDIO_AL_SIGNATURE;

	ch->total_rx_bytes = 0;
	ch->total_tx_bytes = 0;

	ch->write_avail = 0;
	ch->read_avail = 0;
	ch->rx_pending_bytes = 0;

	mutex_init(&ch->ch_lock);

	pr_debug(MODULE_NAME ":open_channel %s func#%d\n",
			 ch->name, ch->func->num);

	INIT_LIST_HEAD(&(ch->rx_size_list_head));

	/* Init SDIO Function */
	ret = sdio_al_enable_func_retry(ch->func, ch->name);
	if (ret) {
		pr_info(MODULE_NAME ":sdio_enable_func() err=%d\n", -ret);
		goto exit_err;
	}

	/* Note: Patch Func CIS tuple issue */
	ret = sdio_set_block_size(ch->func, SDIO_AL_BLOCK_SIZE);
	if (ret) {
		pr_info(MODULE_NAME ":sdio_set_block_size() err=%d\n", -ret);
		goto exit_err;
	}

	ch->func->max_blksize = SDIO_AL_BLOCK_SIZE;

	sdio_set_drvdata(ch->func, ch);

	/* Get channel parameters from the peer SDIO-Client */
	read_sdioc_channel_config(ch);

	/* Set Pipes Threshold on Mailbox */
	ret = set_pipe_threshold(ch->rx_pipe_index, ch->read_threshold);
	if (ret)
		goto exit_err;
	ret = set_pipe_threshold(ch->tx_pipe_index, ch->write_threshold);
	if (ret)
		goto exit_err;

	/* lpm mechanism lives under the assumption there is always a timer */
	/* Check if need to start the timer */
	if  ((ch->poll_delay_msec) && (sdio_al->poll_delay_msec == 0)) {
			sdio_al->poll_delay_msec = ch->poll_delay_msec;

			init_timer(&sdio_al->timer);
			sdio_al->timer.data = (unsigned long) sdio_al;
			sdio_al->timer.function = timer_handler;
			sdio_al->timer.expires = jiffies +
				msecs_to_jiffies(sdio_al->poll_delay_msec);
			add_timer(&sdio_al->timer);
	}

	/* Set flag before interrupts are enabled to allow notify */
	ch->is_open = true;

	/* The new delay will be updated at the next time
	   that the timer expires */
	sdio_al->poll_delay_msec = get_min_poll_time_msec();

	/* Enable Pipes Interrupts */
	enable_eot_interrupt(ch->rx_pipe_index, true);
	enable_eot_interrupt(ch->tx_pipe_index, true);

	enable_threshold_interrupt(ch->rx_pipe_index, true);
	enable_threshold_interrupt(ch->tx_pipe_index, true);

exit_err:

	return ret;
}

/**
 *  Channel Close
 *
 *  Disable the relevant pipes interrupt.
 *  Disable the relevant SDIO-Client function.
 *  Update/stop the timer.
 *  Remove all pending Rx Packet list.
 *
 *  @note: The timer will not restart after expired if
 *  poll time is zero
 *
 */
static int close_channel(struct sdio_channel *ch)
{
	int ret;

	enable_eot_interrupt(ch->rx_pipe_index, false);
	enable_eot_interrupt(ch->tx_pipe_index, false);

	enable_threshold_interrupt(ch->rx_pipe_index, false);
	enable_threshold_interrupt(ch->tx_pipe_index, false);

	ret = sdio_disable_func(ch->func);

	return ret;
}


/**
 *  Ask the worker to read the mailbox.
 */
static void ask_reading_mailbox(void)
{
	if (!sdio_al->ask_mbox) {
		pr_debug(MODULE_NAME ":ask_reading_mailbox\n");
		sdio_al->ask_mbox = true;
		wake_up(&sdio_al->wait_mbox);
	}
}

/**
 *  Start the timer
 */
static void start_timer(void)
{
	if (sdio_al->poll_delay_msec) {
		sdio_al->timer.expires = jiffies +
			msecs_to_jiffies(sdio_al->poll_delay_msec);
		add_timer(&sdio_al->timer);
	}
}

/**
 *  Restart(postpone) the already working timer
 */
static void restart_timer(void)
{
	if (sdio_al->poll_delay_msec) {
		ulong expires =	jiffies +
			msecs_to_jiffies(sdio_al->poll_delay_msec);
		mod_timer(&sdio_al->timer, expires);
	}
}

/**
 *  Do the wakup sequence
 *
 *  Wake up sequence
 *  1. Get lock
 *  2. Enable wake up function if needed
 *  3. Mark NOT OK to sleep and write it
 *  4. Restore default thresholds
 *  5. Start the mailbox and inactivity timer again
 */
static int sdio_al_wake_up(u32 enable_wake_up_func, int is_host_claimed)
{
	int ret = 0, i;
	struct sdio_func *wk_func =
		sdio_al->card->sdio_func[SDIO_AL_WAKEUP_FUNC-1];

	/* Wake up sequence */
	wake_lock(&sdio_al->wake_lock);
	pr_info(MODULE_NAME "Wake up");

	if (!is_host_claimed)
		sdio_claim_host(wk_func);
	if (enable_wake_up_func) {
		/* Enable Wake up Function */
		ret = sdio_al_enable_func_retry(wk_func, "wakeup func");
		if (ret) {
			pr_err(MODULE_NAME ":sdio_enable_func() err=%d\n",
			       -ret);
			ret = -EIO;
			WARN_ON(ret);
		}
	}
	/* Mark NOT OK_TOSLEEP */
	sdio_al->is_ok_to_sleep = 0;
	write_lpm_info();

	/* Restore default thresh for non packet channels */
	for (i = 0; i < SDIO_AL_MAX_CHANNELS; i++) {
		struct sdio_channel *ch = &sdio_al->channel[i];
		if (ch->is_packet_mode == false) {
			ch->read_threshold = ch->def_read_threshold;
			set_pipe_threshold(ch->rx_pipe_index,
					   ch->read_threshold);
		}
	}
	if (enable_wake_up_func)
		sdio_disable_func(wk_func);
	if (!is_host_claimed)
		sdio_release_host(wk_func);

	/* Start the timer again*/
	restart_inactive_time();
	sdio_al->poll_delay_msec = get_min_poll_time_msec();
	start_timer();

	pr_info(MODULE_NAME "Finished Wake up sequence");

	return ret;
}


/**
 *  SDIO Function Interrupt handler.
 *
 *  Interrupt shall be triggered by SDIO-Client when:
 *  1. End-Of-Transfer (EOT) detected in packet mode.
 *  2. Bytes-available reached the threshold.
 *
 *  Reading the mailbox clears the EOT/Threshold interrupt
 *  source.
 *  The interrupt source should be cleared before this ISR
 *  returns. This ISR is called from IRQ Thread and not
 *  interrupt, so it may sleep.
 *
 */
static void sdio_func_irq(struct sdio_func *func)
{
	pr_debug(MODULE_NAME ":start %s.\n", __func__);

	if (sdio_al->is_ok_to_sleep)
		sdio_al_wake_up(0, 1);
	else
		restart_timer();

	read_mailbox(true);

	pr_debug(MODULE_NAME ":end %s.\n", __func__);
}

/**
 *  Timer Expire Handler
 *
 */
static void timer_handler(unsigned long data)
{
	pr_debug(MODULE_NAME " Timer Expired\n");

	ask_reading_mailbox();

	start_timer();
}

/**
 *  Driver Setup.
 *
 */
static int sdio_al_setup(void)
{
	int ret = 0;
	struct mmc_card *card = sdio_al->card;
	struct sdio_func *func1;
	int i = 0;
	int fn = 0;

	pr_info(MODULE_NAME ":sdio_al_setup\n");

	if (card == NULL) {
		pr_info(MODULE_NAME ":No Card detected\n");
		return -ENODEV;
	}

	func1 = card->sdio_func[0];

	sdio_claim_host(sdio_al->card->sdio_func[0]);

	sdio_al->mailbox = kzalloc(sizeof(struct sdio_mailbox), GFP_KERNEL);
	if (sdio_al->mailbox == NULL)
		return -ENOMEM;

	sdio_al->sdioc_sw_header
		= kzalloc(sizeof(*sdio_al->sdioc_sw_header), GFP_KERNEL);
	if (sdio_al->sdioc_sw_header == NULL)
		return -ENOMEM;

	/* Init Func#1 */
	ret = sdio_enable_func(func1);
	if (ret) {
		pr_info(MODULE_NAME ":Fail to enable Func#%d\n", func1->num);
		goto exit_err;
	}

	/* Patch Func CIS tuple issue */
	ret = sdio_set_block_size(func1, SDIO_AL_BLOCK_SIZE);
	func1->max_blksize = SDIO_AL_BLOCK_SIZE;

	ret = read_sdioc_software_header(sdio_al->sdioc_sw_header);
	if (ret)
		goto exit_err;
	pr_info(MODULE_NAME ":SDIO-AL SW version %s.\n", DRV_VERSION);

	sdio_al->workqueue = create_singlethread_workqueue("sdio_al_wq");
	INIT_WORK(&sdio_al->work, worker);

	init_waitqueue_head(&sdio_al->wait_mbox);

	/* disable all pipes interrupts before claim irq.
	   since all are enabled by default. */
	for (i = 0 ; i < SDIO_AL_MAX_PIPES; i++) {
		enable_eot_interrupt(i, false);
		enable_threshold_interrupt(i, false);
	}

	/* Disable all SDIO Functions before claim irq. */
	for (fn = 1 ; fn <= card->sdio_funcs; fn++)
		sdio_disable_func(card->sdio_func[fn-1]);

	if (sdio_al->use_irq) {
		sdio_set_drvdata(func1, sdio_al);

		ret = sdio_claim_irq(func1, sdio_func_irq);
		if (ret) {
			pr_info(MODULE_NAME ":Fail to claim IRQ\n");
			goto exit_err;
		}
	} else {
		pr_debug(MODULE_NAME ":Not using IRQ\n");
	}

	sdio_release_host(sdio_al->card->sdio_func[0]);
	sdio_al->is_ready = true;

	/* Start worker before interrupt might happen */
	queue_work(sdio_al->workqueue, &sdio_al->work);

	start_inactive_time();

	pr_debug(MODULE_NAME ":Ready.\n");

	return 0;

exit_err:
	sdio_release_host(sdio_al->card->sdio_func[0]);
	pr_err(MODULE_NAME ":Setup Failure.\n");

	return ret;
}

/**
 *  Driver Tear-Down.
 *
 */
static void sdio_al_tear_down(void)
{
	if (sdio_al->is_ready) {
		struct sdio_func *func1;

		func1 = sdio_al->card->sdio_func[0];

		sdio_al->is_ready = false; /* Flag worker to exit */
		ask_reading_mailbox(); /* Wakeup worker */
		msleep(100); /* allow gracefully exit of the worker thread */

		flush_workqueue(sdio_al->workqueue);
		destroy_workqueue(sdio_al->workqueue);

		sdio_claim_host(func1);
		sdio_release_irq(func1);
		sdio_disable_func(func1);
		sdio_release_host(func1);
		wake_unlock(&sdio_al->wake_lock);
	}
}

/**
 *  Find channel by name.
 *
 */
static struct sdio_channel *find_channel_by_name(const char *name)
{
	struct sdio_channel *ch = NULL;
	int i;

	for (i = 0; i < SDIO_AL_MAX_CHANNELS; i++)
		if (strcmp(sdio_al->channel[i].name, name) == 0) {
			ch = &sdio_al->channel[i];
			break;
		}

	WARN_ON(ch == NULL);

	return ch;
}

/**
 *  Find the minimal poll time.
 *
 */
static int get_min_poll_time_msec(void)
{
	int i;
	int poll_delay_msec = 0x0FFFFFFF;

	for (i = 0; i < SDIO_AL_MAX_CHANNELS; i++)
		if ((sdio_al->channel[i].is_open) &&
		    (sdio_al->channel[i].poll_delay_msec > 0) &&
		    (sdio_al->channel[i].poll_delay_msec < poll_delay_msec))
			poll_delay_msec = sdio_al->channel[i].poll_delay_msec;

	if (poll_delay_msec == 0x0FFFFFFF)
		poll_delay_msec = 0;

	pr_debug(MODULE_NAME ":poll delay time is %d msec\n", poll_delay_msec);

	return poll_delay_msec;
}

/**
 *  Open SDIO Channel.
 *
 *  Enable the channel.
 *  Set the channel context.
 *  Trigger reading the mailbox to check available bytes.
 *
 */
int sdio_open(const char *name, struct sdio_channel **ret_ch, void *priv,
		 void (*notify)(void *priv, unsigned ch_event))
{
	int ret = 0;
	struct sdio_channel *ch = NULL;

	*ret_ch = NULL; /* default */

	if (sdio_al == NULL) {
		pr_info(MODULE_NAME
			":Try to open ch %s before Module Init\n", name);
		return -ENODEV;
	}

	if (sdio_al->is_err) {
		pr_info(MODULE_NAME ":In Error state, ignore %s\n", __func__);
		return -ENODEV;
	}

	if (!sdio_al->is_ready) {
		ret = sdio_al_setup();
		if (ret)
			return -ENODEV;
	}

	ch = find_channel_by_name(name);
	if (ch == NULL) {
		pr_info(MODULE_NAME ":Can't find channel name %s\n", name);
		return -EINVAL;
	}

	if (ch->is_open) {
		pr_info(MODULE_NAME ":Channel already opened %s\n", name);
		return -EPERM;
	}

	if (sdio_al->is_ok_to_sleep) {
		ret = sdio_al_wake_up(1, 0);
		if (ret)
			return ret;
	}

	ch->name = name;
	ch->notify = notify;
	ch->priv = priv;

	/* Note: Set caller returned context before interrupts are enabled */
	*ret_ch = ch;

	if (ch->is_suspend) {
		pr_info(MODULE_NAME ":Resume channel %s.\n", name);
		ch->is_suspend = false;
		ch->is_open = true;
		ask_reading_mailbox();
		return 0;
	}

	sdio_claim_host(sdio_al->card->sdio_func[0]);
	ret = open_channel(ch);
	sdio_release_host(sdio_al->card->sdio_func[0]);

	if (ret)
		pr_info(MODULE_NAME ":sdio_open %s err=%d\n", name, -ret);
	else
		pr_info(MODULE_NAME ":sdio_open %s completed OK\n", name);

	/* Read the mailbox after the channel is open to detect
	   pending rx packets */
	if ((!ret) && (!sdio_al->use_irq))
		ask_reading_mailbox();

	return ret;
}
EXPORT_SYMBOL(sdio_open);

/**
 *  Close SDIO Channel.
 *
 */
int sdio_close(struct sdio_channel *ch)
{
	int ret;

	BUG_ON(ch->signature != SDIO_AL_SIGNATURE);

	if (!ch->is_open)
		return -EINVAL;

	if (sdio_al->is_ok_to_sleep) {
		ret = sdio_al_wake_up(1, 0);
		if (ret)
			return ret;
	}

	pr_info(MODULE_NAME ":sdio_close %s\n", ch->name);

	/* Stop channel notifications, and read/write operations. */
	ch->is_open = false;
	ch->is_suspend = true;

	ch->notify = NULL;

	sdio_claim_host(sdio_al->card->sdio_func[0]);
	ret = close_channel(ch);
	sdio_release_host(sdio_al->card->sdio_func[0]);

	do
		ret = remove_handled_rx_packet(ch);
	while (ret > 0);

	if  (ch->poll_delay_msec > 0)
		sdio_al->poll_delay_msec = get_min_poll_time_msec();

	return ret;
}
EXPORT_SYMBOL(sdio_close);

/**
 *  Get the number of available bytes to write.
 *
 */
int sdio_write_avail(struct sdio_channel *ch)
{
	BUG_ON(ch->signature != SDIO_AL_SIGNATURE);

	pr_debug(MODULE_NAME ":sdio_write_avail %s 0x%x\n",
			 ch->name, ch->write_avail);

	return ch->write_avail;
}
EXPORT_SYMBOL(sdio_write_avail);

/**
 *  Get the number of available bytes to read.
 *
 */
int sdio_read_avail(struct sdio_channel *ch)
{
	BUG_ON(ch->signature != SDIO_AL_SIGNATURE);

	pr_debug(MODULE_NAME ":sdio_read_avail %s 0x%x\n",
			 ch->name, ch->read_avail);

	return ch->read_avail;

}
EXPORT_SYMBOL(sdio_read_avail);

/**
 *  Read from SDIO Channel.
 *
 *  Reading from the pipe will trigger interrupt if there are
 *  other pending packets on the SDIO-Client.
 *
 */
int sdio_read(struct sdio_channel *ch, void *data, int len)
{
	int ret = 0;

	BUG_ON(ch->signature != SDIO_AL_SIGNATURE);
	/* lpm policy says we can't go to sleep when we have pending rx data,
	   so either we had rx interrupt and woken up, or we never went to
	   sleep */
	BUG_ON(sdio_al->is_ok_to_sleep);

	if (sdio_al->is_err) {
		pr_info(MODULE_NAME ":In Error state, ignore sdio_read\n");
		return -ENODEV;
	}

	if (!ch->is_open) {
		pr_info(MODULE_NAME ":reading from closed channel %s\n",
				 ch->name);
		return -EINVAL;
	}

	pr_info(MODULE_NAME ":start ch %s read %d avail %d.\n",
		ch->name, len, ch->read_avail);

	restart_inactive_time();

	if ((ch->is_packet_mode) && (len != ch->read_avail)) {
		pr_info(MODULE_NAME ":sdio_read ch %s len != read_avail\n",
				 ch->name);
		return -EINVAL;
	}

	if (len > ch->read_avail) {
		pr_info(MODULE_NAME ":ERR ch %s read %d avail %d.\n",
				ch->name, len, ch->read_avail);
		return -ENOMEM;
	}

	sdio_claim_host(sdio_al->card->sdio_func[0]);
	ret = sdio_memcpy_fromio(ch->func, data, PIPE_RX_FIFO_ADDR, len);

	if (ret)
		pr_info(MODULE_NAME ":sdio_read err=%d\n", -ret);

	/* Remove handled packet from the list regardless if ret is ok */
	if (ch->is_packet_mode)
		remove_handled_rx_packet(ch);
	else
		ch->read_avail -= len;

	ch->total_rx_bytes += len;
	pr_info(MODULE_NAME ":end ch %s read %d avail %d total %d.\n",
		ch->name, len, ch->read_avail, ch->total_rx_bytes);

	sdio_release_host(sdio_al->card->sdio_func[0]);

	if ((ch->read_avail == 0) &&
	    !((ch->is_packet_mode) && (sdio_al->use_irq)))
		ask_reading_mailbox();

	return ret;
}
EXPORT_SYMBOL(sdio_read);

/**
 *  Write to SDIO Channel.
 *
 */
int sdio_write(struct sdio_channel *ch, const void *data, int len)
{
	int ret = 0;

	BUG_ON(ch->signature != SDIO_AL_SIGNATURE);
	WARN_ON(len > ch->write_avail);

	if (sdio_al->is_err) {
		pr_info(MODULE_NAME ":In Error state, ignore sdio_write\n");
		return -ENODEV;
	}

	if (!ch->is_open) {
		pr_info(MODULE_NAME ":writing to closed channel %s\n",
				 ch->name);
		return -EINVAL;
	}

	if (sdio_al->is_ok_to_sleep) {
		ret = sdio_al_wake_up(1, 0);
		if (ret)
			return ret;
	} else {
		restart_inactive_time();
	}

	pr_info(MODULE_NAME ":start ch %s write %d avail %d.\n",
		ch->name, len, ch->write_avail);

	if (len > ch->write_avail) {
		pr_info(MODULE_NAME ":ERR ch %s write %d avail %d.\n",
				ch->name, len, ch->write_avail);
		return -ENOMEM;
	}

	sdio_claim_host(sdio_al->card->sdio_func[0]);
	ret = sdio_ch_write(ch, data, len);

	ch->total_tx_bytes += len;
	pr_info(MODULE_NAME ":end ch %s write %d avail %d total %d.\n",
		ch->name, len, ch->write_avail, ch->total_tx_bytes);

	if (ret) {
		pr_info(MODULE_NAME ":sdio_write err=%d\n", -ret);
	} else {
		/* Round up to whole buffer size */
		len = ROUND_UP(len, ch->peer_tx_buf_size);
		/* Protect from wraparound */
		len = min(len, (int) ch->write_avail);
		ch->write_avail -= len;
	}

	sdio_release_host(sdio_al->card->sdio_func[0]);

	if (ch->write_avail < ch->min_write_avail)
		ask_reading_mailbox();

	return ret;
}
EXPORT_SYMBOL(sdio_write);

/**
 *  Set the threshold to trigger interrupt from SDIO-Card on
 *  available bytes to write.
 *
 */
int sdio_set_write_threshold(struct sdio_channel *ch, int threshold)
{
	int ret;

	BUG_ON(ch->signature != SDIO_AL_SIGNATURE);
	if (sdio_al->is_err) {
		pr_info(MODULE_NAME ":In Error state, ignore %s\n", __func__);
		return -ENODEV;
	}
	if (sdio_al->is_ok_to_sleep) {
		ret = sdio_al_wake_up(1, 0);
		if (ret)
			return ret;
	}

	ch->write_threshold = threshold;

	pr_debug(MODULE_NAME ":sdio_set_write_threshold %s 0x%x\n",
			 ch->name, ch->write_threshold);

	sdio_claim_host(sdio_al->card->sdio_func[0]);
	ret = set_pipe_threshold(ch->tx_pipe_index, ch->write_threshold);
	sdio_release_host(sdio_al->card->sdio_func[0]);

	return ret;
}
EXPORT_SYMBOL(sdio_set_write_threshold);

/**
 *  Set the threshold to trigger interrupt from SDIO-Card on
 *  available bytes to read.
 *
 */
int sdio_set_read_threshold(struct sdio_channel *ch, int threshold)
{
	int ret;

	BUG_ON(ch->signature != SDIO_AL_SIGNATURE);
	if (sdio_al->is_err) {
		pr_info(MODULE_NAME ":In Error state, ignore %s\n", __func__);
		return -ENODEV;
	}
	if (sdio_al->is_ok_to_sleep) {
		ret = sdio_al_wake_up(1, 0);
		if (ret)
			return ret;
	}

	ch->read_threshold = threshold;

	pr_debug(MODULE_NAME ":sdio_set_write_threshold %s 0x%x\n",
			 ch->name, ch->read_threshold);

	sdio_claim_host(sdio_al->card->sdio_func[0]);
	ret = set_pipe_threshold(ch->rx_pipe_index, ch->read_threshold);
	sdio_release_host(sdio_al->card->sdio_func[0]);

	return ret;
}
EXPORT_SYMBOL(sdio_set_read_threshold);


/**
 *  Set the polling delay.
 *
 */
int sdio_set_poll_time(struct sdio_channel *ch, int poll_delay_msec)
{
	BUG_ON(ch->signature != SDIO_AL_SIGNATURE);
	if (sdio_al->is_err) {
		pr_info(MODULE_NAME ":In Error state, ignore %s\n", __func__);
		return -ENODEV;
	}

	if (poll_delay_msec <= 0 || poll_delay_msec > INACTIVITY_TIME_MSEC)
		return -EPERM;

	if (sdio_al->is_ok_to_sleep) {
		int ret;

		ret = sdio_al_wake_up(1, 0);
		if (ret)
			return ret;
	}

	ch->poll_delay_msec = poll_delay_msec;

	/* The new delay will be updated at the next time
	   that the timer expires */
	sdio_al->poll_delay_msec = get_min_poll_time_msec();

	return sdio_al->poll_delay_msec;
}
EXPORT_SYMBOL(sdio_set_poll_time);

/**
 *  Default platform device release function.
 *
 */
static void default_sdio_al_release(struct device *dev)
{
	pr_info(MODULE_NAME ":platform device released.\n");
}

/**
 *  Probe to claim the SDIO card.
 *
 */
static int mmc_probe(struct mmc_card *card)
{
	int ret = 0;
	int i;

	dev_info(&card->dev, "Probing..\n");

	if (!mmc_card_sdio(card))
		return -ENODEV;

	if (card->sdio_funcs < SDIO_AL_MAX_FUNCS) {
		dev_info(&card->dev,
			 "SDIO-functions# %d less than expected.\n",
			 card->sdio_funcs);
		return -ENODEV;
	}

	dev_info(&card->dev, "vendor_id = 0x%x, device_id = 0x%x\n",
			 card->cis.vendor, card->cis.device);

	dev_info(&card->dev, "SDIO Card claimed.\n");

	sdio_al->card = card;
	wake_lock_init(&sdio_al->wake_lock, WAKE_LOCK_SUSPEND, MODULE_NAME);

	/* Don't allow sleep until all required clients register */
	wake_lock(&sdio_al->wake_lock);

	#ifdef DEBUG_SDIO_AL_UNIT_TEST
	pr_info(MODULE_NAME ":==== SDIO-AL UNIT-TEST ====\n");
	#else
	/* Allow clients to probe for this driver */
	for (i = 0; i < SDIO_AL_MAX_CHANNELS; i++) {
		sdio_al->channel[i].pdev.name = sdio_al->channel[i].name;
		sdio_al->channel[i].pdev.dev.release = default_sdio_al_release;
		platform_device_register(&sdio_al->channel[i].pdev);
	}
	#endif

	return ret;
}

/**
 *  Release the SDIO card.
 *
 */
static void mmc_remove(struct mmc_card *card)
{
	#ifndef DEBUG_SDIO_AL_UNIT_TEST
	int i;

	for (i = 0; i < SDIO_AL_MAX_CHANNELS; i++)
		platform_device_unregister(&sdio_al->channel[i].pdev);
	#endif

	pr_info(MODULE_NAME ":sdio card removed.\n");
}

static struct mmc_driver mmc_driver = {
	.drv		= {
		.name   = "sdio_al",
	},
	.probe  	= mmc_probe,
	.remove 	= mmc_remove,
};

/**
 *  Module Init.
 *
 *  @warn: allocate sdio_al context before registering driver.
 *
 */
static int __init sdio_al_init(void)
{
	int ret = 0;

	pr_debug(MODULE_NAME ":sdio_al_init\n");

	sdio_al = kzalloc(sizeof(struct sdio_al), GFP_KERNEL);
	if (sdio_al == NULL)
		return -ENOMEM;

	sdio_al->is_ready = false;

	sdio_al->use_irq = true;

	sdio_al->signature = SDIO_AL_SIGNATURE;

	set_default_channels_config();

	ret = mmc_register_driver(&mmc_driver);

	return ret;
}

/**
 *  Module Exit.
 *
 *  Free allocated memory.
 *  Disable SDIO-Card.
 *  Unregister driver.
 *
 */
static void __exit sdio_al_exit(void)
{
	if (sdio_al == NULL)
		return;

	pr_debug(MODULE_NAME ":sdio_al_exit\n");

	sdio_al_tear_down();

	kfree(sdio_al->sdioc_sw_header);
	kfree(sdio_al->mailbox);
	kfree(sdio_al);

	mmc_unregister_driver(&mmc_driver);

	pr_debug(MODULE_NAME ":sdio_al_exit complete\n");
}

module_init(sdio_al_init);
module_exit(sdio_al_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SDIO Abstraction Layer");
MODULE_AUTHOR("Amir Samuelov <amirs@codeaurora.org>");
MODULE_VERSION(DRV_VERSION);

