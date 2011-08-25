/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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
 * SDIO-Downloader
 *
 * To be used with Qualcomm's SDIO-Client connected to this host.
 */

/* INCLUDES */
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/mmc/card.h>
#include <linux/dma-mapping.h>
#include <mach/dma.h>
#include <linux/mmc/sdio_func.h>
#include <mach/sdio_al.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/kthread.h>
#include <linux/version.h>
#include <linux/errno.h>

/* DEFINES AND MACROS */
#define MAX_NUM_DEVICES			1
#define TTY_SDIO_DEV			"tty_sdio_0"
#define SDIOC_MAILBOX_SIZE		24
#define SDIOC_MAILBOX_ADDRESS		0
#define SDIO_DL_BLOCK_SIZE		512
#define THRESHOLD_PERCENTAGE		20
#define PERCENTAGE_RATIO		100
#define SDIO_DL_MAIN_THREAD_NAME	"sdio_tty_main_thread"
#define SDIOC_DL_BUFF_SIZE_OFFSET	0
#define SDIOC_UP_BUFF_SIZE_OFFSET	0x4
#define SDIOC_DL_WR_PTR			0x8
#define SDIOC_DL_RD_PTR			0xC
#define SDIOC_UL_WR_PTR			0x10
#define SDIOC_UL_RD_PTR			0x14
#define SDIOC_EXIT_PTR			0x18
#define SDIOC_DL_BUFF_BASE		0x4000
#define SDIOC_UL_BUFF_BASE		0x6000
#define WRITE_RETRIES			0xFFFFFFFF
#define INPUT_SPEED			4800
#define OUTPUT_SPEED			4800
#define SDIOC_EXIT_CODE			0xDEADDEAD
#define SLEEP_MS			10
#define PRINTING_GAP			200
#define SIZE_2K				2048
#define TIMER_DURATION			10

#define THRESH_CHANGE
#define PUSH_STRING

/* FORWARD DECLARATIONS */
static int sdio_dld_open(struct tty_struct *tty, struct file *file);
static void sdio_dld_close(struct tty_struct *tty, struct file *file);
static int sdio_dld_write_callback(struct tty_struct *tty,
				   const unsigned char *buf, int count);
static int sdio_dld_main_task(void *card);

/* STRUCTURES AND TYPES */
struct sdioc_reg_sequential_chunk {
	unsigned int dl_buff_size;
	unsigned int ul_buff_size;
	unsigned int dl_wr_ptr;
	unsigned int dl_rd_ptr;
	unsigned int up_wr_ptr;
	unsigned int up_rd_ptr;
};

struct sdioc_reg {
	unsigned int reg_val;
	unsigned int reg_offset;
};

struct sdioc_reg_chunk {
	struct sdioc_reg dl_buff_size;
	struct sdioc_reg ul_buff_size;
	struct sdioc_reg dl_wr_ptr;
	struct sdioc_reg dl_rd_ptr;
	struct sdioc_reg up_wr_ptr;
	struct sdioc_reg up_rd_ptr;
	struct sdioc_reg good_to_exit_ptr;
};

struct sdio_data {
	char *data;
	int data_thresh;
	int offset_read_p;
	int offset_write_p;
	int buffer_size;
	int num_of_bytes_in_use;
};

struct sdio_dld_data {
	struct sdioc_reg_chunk sdioc_reg;
	struct sdio_data incoming_data;
	struct sdio_data outgoing_data;
};

struct sdio_dld_wait_event {
	wait_queue_head_t wait_event;
	int wake_up_signal;
};

struct sdio_dld_task {
	struct task_struct *dld_task;
	const char *task_name;
	struct sdio_dld_wait_event exit_wait;
	atomic_t please_close;
};

struct sdio_downloader {
	int sdioc_boot_func;
	struct sdio_dld_wait_event write_callback_event;
	struct sdio_dld_task dld_main_thread;
	struct tty_driver *tty_drv;
	struct tty_struct *tty_str;
	struct sdio_dld_data sdio_dloader_data;
	struct mmc_card *card;
	int(*done_callback)(void);
	struct sdio_dld_wait_event main_loop_event;
	struct timer_list timer;
	int poll_ms;
};

static const struct tty_operations sdio_dloader_tty_ops = {
	.open = sdio_dld_open,
	.close = sdio_dld_close,
	.write = sdio_dld_write_callback,
};

/* GLOBAL VARIABLES */
struct sdio_downloader sdio_dld;

static DEFINE_SPINLOCK(lock1);
static unsigned long lock_flags1;
static DEFINE_SPINLOCK(lock2);
static unsigned long lock_flags2;

/**
  * thresh_set
  * calcualtes the threshold to be THRESHOLD_PERCENTAGE of size, rounded to
  * lower value of n*SDIO_DL_BLOCK_SIZE
  *
  * @size: the base number to calculate THRESHOLD_PERCENTAGE
  * @return the calculated value.
  */
static inline unsigned int thresh_set(unsigned int size)
{
	unsigned int result;

	if (size < SDIO_DL_BLOCK_SIZE)
		return size;

	result = (((size * THRESHOLD_PERCENTAGE / PERCENTAGE_RATIO) /
		   SDIO_DL_BLOCK_SIZE) * SDIO_DL_BLOCK_SIZE);

	result = max(SDIO_DL_BLOCK_SIZE, (int)result);

	return result;
}

/**
  * sdio_dld_allocate_local_buffers
  * allocates local outgoing and incoming buffers and also sets
  * threshold for outgoing data.
  *
  * @return 0 on success or negative value on error.
  */
static int sdio_dld_allocate_local_buffers(void)
{
	struct sdioc_reg_chunk *reg_str = &sdio_dld.sdio_dloader_data.
		sdioc_reg;
	struct sdio_data *outgoing = &sdio_dld.sdio_dloader_data.outgoing_data;
	struct sdio_data *incoming = &sdio_dld.sdio_dloader_data.incoming_data;

	/* threshold writing is initialized */
	outgoing->data_thresh = thresh_set(reg_str->ul_buff_size.reg_val);
	pr_debug(MODULE_NAME ": %s. Sending Threshold=%d\n",
	       __func__,
	       outgoing->data_thresh);

	incoming->data =
		kzalloc(reg_str->dl_buff_size.reg_val, GFP_KERNEL);

	if (!incoming->data) {
		pr_err(MODULE_NAME ": %s - param ""incoming->data"" is NULL. "
		       "Couldn't allocate incoming_data local buffer\n",
		       __func__);
		return -ENOMEM;
	}

	incoming->buffer_size = reg_str->dl_buff_size.reg_val;

	outgoing->data =
		kzalloc(reg_str->ul_buff_size.reg_val, GFP_KERNEL);

	/* if allocating of outgoing buffer failed, free incoming buffer */
	if (!outgoing->data) {
		kfree(incoming->data);
		pr_err(MODULE_NAME ": %s - param ""outgoing->data"" is NULL. "
		       "Couldn't allocate outgoing->data local buffer\n",
		       __func__);
		return -ENOMEM;
	}

	outgoing->buffer_size =
		reg_str->ul_buff_size.reg_val;

	return 0;
}

/**
  * sdio_dld_dealloc_local_buffers frees incoming and outgoing
  * buffers.
  *
  * @return None.
  */
static void sdio_dld_dealloc_local_buffers(void)
{
	kfree((void *)sdio_dld.sdio_dloader_data.incoming_data.data);
	kfree((void *)sdio_dld.sdio_dloader_data.outgoing_data.data);
}

/**
  * mailbox_to_seq_chunk_read
  * reads 6 registers of mailbox from str_func, as a sequentail
  * chunk in memory, and updates global struct accordingly.
  *
  * @str_func: a pointer to func struct.
  * @return 0 on success or negative value on error.
  */
static int mailbox_to_seq_chunk_read(struct sdio_func *str_func)
{
	struct sdioc_reg_sequential_chunk seq_chunk;
	struct sdioc_reg_chunk *reg = &sdio_dld.sdio_dloader_data.sdioc_reg;
	int status = 0;

	struct sdio_data *outgoing = &sdio_dld.sdio_dloader_data.outgoing_data;
	static int counter = 1;
	static int offset_write_p;
	static int offset_read_p;
	static int up_wr_ptr;
	static int up_rd_ptr;
	static int dl_wr_ptr;
	static int dl_rd_ptr;

	if (!str_func) {
		pr_err(MODULE_NAME ": %s - param ""str_func"" is NULL.\n",
		       __func__);
		return -EINVAL;
	}

	sdio_claim_host(str_func);

	/* reading SDIOC_MAILBOX_SIZE bytes from SDIOC_MAILBOX_ADDRESS */
	status = sdio_memcpy_fromio(str_func,
				    (void *)&seq_chunk,
				    SDIOC_MAILBOX_ADDRESS,
				    SDIOC_MAILBOX_SIZE);
	if (status) {
		pr_err(MODULE_NAME ": %s - sdio_memcpy_fromio()"
		       " READING MAILBOX failed. status=%d.\n",
		       __func__, status);
	}

	sdio_release_host(str_func);

	reg->dl_buff_size.reg_val = seq_chunk.dl_buff_size;
	reg->dl_rd_ptr.reg_val = seq_chunk.dl_rd_ptr;
	reg->dl_wr_ptr.reg_val = seq_chunk.dl_wr_ptr;
	reg->ul_buff_size.reg_val = seq_chunk.ul_buff_size;
	reg->up_rd_ptr.reg_val = seq_chunk.up_rd_ptr;
	reg->up_wr_ptr.reg_val = seq_chunk.up_wr_ptr;

	/* DEBUG - if there was a change in value */
	if ((offset_write_p !=	outgoing->offset_write_p) ||
	    (offset_read_p != outgoing->offset_read_p) ||
	    (up_wr_ptr != reg->up_wr_ptr.reg_val) ||
	    (up_rd_ptr != reg->up_rd_ptr.reg_val) ||
	    (dl_wr_ptr != reg->dl_wr_ptr.reg_val) ||
	    (dl_rd_ptr != reg->dl_rd_ptr.reg_val) ||
	    (counter % PRINTING_GAP == 0)) {
		counter = 1;
		pr_debug(MODULE_NAME ": %s MailBox pointers: BLOCK_SIZE=%d, "
				   "d_s=%d, u_s=%d, hw=%d, hr=%d, cuw=%d, "
				   "cur=%d, cdw=%d, cdr=%d\n",
			 __func__,
			 SDIO_DL_BLOCK_SIZE,
			 seq_chunk.dl_buff_size,
			 seq_chunk.ul_buff_size,
			 outgoing->offset_write_p,
			 outgoing->offset_read_p,
			 reg->up_wr_ptr.reg_val,
			 reg->up_rd_ptr.reg_val,
			 reg->dl_wr_ptr.reg_val,
			 reg->dl_rd_ptr.reg_val);

		/* update static variables */
		offset_write_p = outgoing->offset_write_p;
		offset_read_p =	outgoing->offset_read_p;
		up_wr_ptr = reg->up_wr_ptr.reg_val;
		up_rd_ptr = reg->up_rd_ptr.reg_val;
		dl_wr_ptr = reg->dl_wr_ptr.reg_val;
		dl_rd_ptr = reg->dl_rd_ptr.reg_val;
	} else {
		counter++;
	}

	return status;
}

/**
  * sdio_dld_init_func
  * enables the sdio func, and sets the func block size.
  *
  * @str_func: a pointer to func struct.
  * @return 0 on success or negative value on error.
  */
static int sdio_dld_init_func(struct sdio_func *str_func)
{
	int status1 = 0;
	int status2 = 0;

	if (!str_func) {
		pr_err(MODULE_NAME ": %s - param ""str_func"" is NULL.\n",
		       __func__);
		return -EINVAL;
	}

	sdio_claim_host(str_func);

	status1 = sdio_enable_func(str_func);
	if (status1) {
		sdio_release_host(str_func);
		pr_err(MODULE_NAME ": %s - sdio_enable_func() failed. "
		       "status=%d\n", __func__, status1);
		return status1;
	}

	status2 = sdio_set_block_size(str_func, SDIO_DL_BLOCK_SIZE);
	if (status2) {
		pr_err(MODULE_NAME ": %s - sdio_set_block_size() failed. "
		       "status=%d\n", __func__, status2);
		status1 = sdio_disable_func(str_func);
		if (status1) {
			pr_err(MODULE_NAME ": %s - sdio_disable_func() "
		       "failed. status=%d\n", __func__, status1);
		}
		sdio_release_host(str_func);
		return status2;
	}

	sdio_release_host(str_func);
	str_func->max_blksize = SDIO_DL_BLOCK_SIZE;
	return 0;
}

/**
  * init_func_and_allocate
  * initializes the sdio func, and then reads the mailbox, in
  * order to allocate incoming and outgoing buffers according to
  * the size that was read from the mailbox.
  *
  * @str_func: a pointer to func struct.
  * @return 0 on success or negative value on error.
  */
static int init_func_and_allocate(struct sdio_func *str_func)
{
	int status = 0;

	if (!str_func) {
		pr_err(MODULE_NAME ": %s - param ""str_func"" is NULL.\n",
		       __func__);
		return -EINVAL;
	}

	status = sdio_dld_init_func(str_func);
	if (status) {
		pr_err(MODULE_NAME ": %s - Failure in Function "
		       "sdio_dld_init_func(). status=%d\n",
		       __func__, status);
		return status;
	}

	status = mailbox_to_seq_chunk_read(str_func);
	if (status) {
		pr_err(MODULE_NAME ": %s - Failure in Function "
		       "mailbox_to_seq_chunk_read(). status=%d\n",
		       __func__, status);
		return status;
	}

	status = sdio_dld_allocate_local_buffers();
	if (status) {
		pr_err(MODULE_NAME ": %s - Failure in Function "
		       "sdio_dld_allocate_local_buffers(). status=%d\n",
		       __func__, status);
		return status;
	}

	return 0;
}

/**
  * sdio_dld_create_thread
  * creates thread and wakes it up.
  *
  * @return 0 on success or negative value on error.
  */
static int sdio_dld_create_thread(void)
{
	sdio_dld.dld_main_thread.task_name = SDIO_DL_MAIN_THREAD_NAME;

	sdio_dld.dld_main_thread.dld_task =
		kthread_create(sdio_dld_main_task,
			       (void *)(sdio_dld.card),
			       sdio_dld.dld_main_thread.task_name);

	if (IS_ERR(sdio_dld.dld_main_thread.dld_task)) {
		pr_err(MODULE_NAME ": %s - kthread_create() failed\n",
			__func__);
		return -ENOMEM;
	}
	wake_up_process(sdio_dld.dld_main_thread.dld_task);
	return 0;
}


/**
  * start_timer - sets the timer and starts.
  *
  * @return None.
  */
static void start_timer(void)
{
	if (sdio_dld.poll_ms) {
		sdio_dld.timer.expires = jiffies +
			msecs_to_jiffies(sdio_dld.poll_ms);
		add_timer(&sdio_dld.timer);
	}
}

/**
  * timer_handler - this is the timer handler.
  * whenever it is invoked, it wakes up the main loop task, and the write
  * callback, and starts the timer again.
  *
  * @data: a pointer to the tty device driver structure.
  * @return None.
  */

static void timer_handler(unsigned long data)
{
	pr_debug(MODULE_NAME " Timer Expired\n");
	spin_lock_irqsave(&lock2, lock_flags2);
	if (sdio_dld.main_loop_event.wake_up_signal == 0) {
		sdio_dld.main_loop_event.wake_up_signal = 1;
		wake_up(&sdio_dld.main_loop_event.wait_event);
	}
	spin_unlock_irqrestore(&lock2, lock_flags2);

	sdio_dld.write_callback_event.wake_up_signal = 1;
	wake_up(&sdio_dld.write_callback_event.wait_event);

	start_timer();
}


/**
  * sdio_dld_open
  * this is the open callback of the tty driver.
  * it initializes the sdio func, allocates the buffers, and
  * creates the main thread.
  *
  * @tty: a pointer to the tty struct.
  * @file: file descriptor.
  * @return 0 on success or negative value on error.
  */
static int sdio_dld_open(struct tty_struct *tty, struct file *file)
{
	int status = 0;
	int func_in_array =
		REAL_FUNC_TO_FUNC_IN_ARRAY(sdio_dld.sdioc_boot_func);
	struct sdio_func *str_func = sdio_dld.card->sdio_func[func_in_array];

	if (!tty) {
		pr_err(MODULE_NAME ": %s - param ""tty"" is NULL.\n",
		       __func__);
		return -EINVAL;
	}

	if (!str_func) {
		pr_err(MODULE_NAME ": %s - param ""str_func"" is NULL.\n",
		       __func__);
		return -EINVAL;
	}

	atomic_set(&sdio_dld.dld_main_thread.please_close, 0);
	sdio_dld.dld_main_thread.exit_wait.wake_up_signal = 0;

	status = init_func_and_allocate(str_func);
	if (status) {
		sdio_dld_dealloc_local_buffers();
		pr_err(MODULE_NAME ": %s, failed in init_func_and_allocate()."
				   "status=%d\n", __func__, status);
		return status;
	}

	status = sdio_dld_create_thread();
	if (status) {
		sdio_dld_dealloc_local_buffers();
		pr_err(MODULE_NAME ": %s, failed in sdio_dld_create_thread()."
				   "status=%d\n", __func__, status);
		return status;
	}

	/* init waiting event of the write callback */
	init_waitqueue_head(&sdio_dld.write_callback_event.wait_event);


	/* init waiting event of the main loop */
	init_waitqueue_head(&sdio_dld.main_loop_event.wait_event);

	/* configure and init the timer */
	sdio_dld.poll_ms = TIMER_DURATION;
	init_timer(&sdio_dld.timer);
	sdio_dld.timer.data = (unsigned long) &sdio_dld;
	sdio_dld.timer.function = timer_handler;
	sdio_dld.timer.expires = jiffies +
		msecs_to_jiffies(sdio_dld.poll_ms);
	add_timer(&sdio_dld.timer);

	return 0;
}

/**
  * sdio_dld_close
  * this is the close callback of the tty driver. it requests
  * the main thread to exit, and waits for notification of it.
  * it also de-allocates the buffers, and unregisters the tty
  * driver and device.
  *
  * @tty: a pointer to the tty struct.
  * @file: file descriptor.
  * @return None.
  */
static void sdio_dld_close(struct tty_struct *tty, struct file *file)
{
	int status = 0;

	/* informing the SDIOC that it can exit boot phase */
	sdio_dld.sdio_dloader_data.sdioc_reg.good_to_exit_ptr.reg_val =
		SDIOC_EXIT_CODE;

	atomic_set(&sdio_dld.dld_main_thread.please_close, 1);

	pr_debug(MODULE_NAME ": %s - CLOSING - WAITING...", __func__);

	wait_event(sdio_dld.dld_main_thread.exit_wait.wait_event,
		   sdio_dld.dld_main_thread.exit_wait.wake_up_signal);
	pr_debug(MODULE_NAME ": %s - CLOSING - WOKE UP...", __func__);

	sdio_dld_dealloc_local_buffers();

	tty_unregister_device(sdio_dld.tty_drv, 0);

	status = tty_unregister_driver(sdio_dld.tty_drv);

	if (status) {
		pr_err(MODULE_NAME ": %s - tty_unregister_driver() failed\n",
		       __func__);
	}

	if (sdio_dld.done_callback)
		sdio_dld.done_callback();
}

/**
  * writing_size_to_buf
  * writes from src buffer into dest buffer. if dest buffer
  * reaches its end, rollover happens.
  *
  * @dest: destination buffer.
  * @src: source buffer.
  * @dest_wr_ptr: writing pointer in destination buffer.
  * @dest_size: destination buffer size.
  * @dest_rd_ptr: reading pointer in destination buffer.
  * @size_to_write: size of bytes to write.
  * @return -how many bytes actually written to destination
  * buffer.
  *
  * ONLY destination buffer is treated as cyclic buffer.
  */
static int writing_size_to_buf(char *dest,
			       const unsigned char *src,
			       int *dest_wr_ptr,
			       int dest_size,
			       int dest_rd_ptr,
			       int size_to_write)
{
	int actually_written = 0;

	if (!dest) {
		pr_err(MODULE_NAME ": %s - param ""dest"" is NULL.\n",
		       __func__);
		return -EINVAL;
	}

	if (!src) {
		pr_err(MODULE_NAME ": %s - param ""src"" is NULL.\n",
		       __func__);
		return -EINVAL;
	}

	if (!dest_wr_ptr) {
		pr_err(MODULE_NAME ": %s - param ""dest_wr_ptr"" is NULL.\n",
		       __func__);
		return -EINVAL;
	}

	for (actually_written = 0 ;
	      actually_written < size_to_write ; ++actually_written) {
		/* checking if buffer is full */
		if ((((*dest_wr_ptr) + 1) % dest_size) == dest_rd_ptr) {
			pr_err(MODULE_NAME ": %s(), outgoing buffer is full. "
			       "writePtr=%d, readPtr=%d\n",
			       __func__, *dest_wr_ptr, dest_rd_ptr);
			return actually_written;
		}

		dest[*dest_wr_ptr] = src[actually_written];
		/* cyclic buffer */
		*dest_wr_ptr = ((*dest_wr_ptr + 1) % dest_size);
	}

	return actually_written;
}

/**
  * sdioc_bytes_free_in_buffer
  * this routine calculates how many bytes are free in a buffer
  * and how many are in use, according to its reading and
  * writing pointer offsets.
  *
  * @write_ptr: writing pointer.
  * @read_ptr: reading pointer.
  * @total_size: buffer size.
  * @free_bytes: return value-how many free bytes in buffer.
  * @bytes_in_use: return value-how many bytes in use in buffer.
  * @return 0 on success or negative value on error.
  *
  * buffer is treated as a cyclic buffer.
  */
static int sdioc_bytes_free_in_buffer(int write_ptr,
				      int read_ptr,
				      int total_size,
				      int *free_bytes,
				      int *bytes_in_use)
{
	if (!free_bytes) {
		pr_err(MODULE_NAME ": %s - param ""free_bytes"" is NULL.\n",
		       __func__);
		return -EINVAL;
	}

	if (!bytes_in_use) {
		pr_err(MODULE_NAME ": %s - param ""bytes_in_use"" is NULL.\n",
		       __func__);
		return -EINVAL;
	}

	/* if pointers equel - buffers are empty. nothing to read/write */
	if (write_ptr == read_ptr) {
		*free_bytes = total_size - 1;
		*bytes_in_use = 0;
	} else {
		*bytes_in_use = (write_ptr > read_ptr) ?
			(write_ptr - read_ptr) :
			(total_size - (read_ptr - write_ptr));
		*free_bytes = total_size - *bytes_in_use - 1;
	}

	return 0;
}

/**
  * sdio_dld_write_callback
  * this is the write callback of the tty driver.
  *
  * @tty: pointer to tty struct.
  * @buf: buffer to write from.
  * @count: number of bytes to write.
  * @return bytes written or negative value on error.
  *
  * if destination buffer has not enough room for the incoming
  * data, returns an error.
  */
static int sdio_dld_write_callback(struct tty_struct *tty,
				   const unsigned char *buf, int count)
{
	struct sdio_data *outgoing = &sdio_dld.sdio_dloader_data.outgoing_data;
	int dst_free_bytes = 0;
	int dummy = 0;
	int status = 0;
	int bytes_written = 0;
	int total_written = 0;
	static int write_retry;
	int pending_to_write = count;

	pr_debug(MODULE_NAME ": %s - WRITING CALLBACK CALLED WITH %d bytes\n",
		 __func__, count);

	if (!outgoing->data) {
		pr_err(MODULE_NAME ": %s - param ""outgoing->data"" is NULL.\n",
		       __func__);
		return -EINVAL;
	}

	pr_debug(MODULE_NAME ": %s - WRITE CALLBACK size to write to outgoing"
		 " buffer %d\n", __func__, count);

	/* as long as there is something to write to outgoing buffer */
	do {
		int bytes_to_write = 0;
		status = sdioc_bytes_free_in_buffer(
			outgoing->offset_write_p,
			outgoing->offset_read_p,
			outgoing->buffer_size,
			&dst_free_bytes,
			&dummy);

		if (status) {
			pr_err(MODULE_NAME ": %s - Failure in Function "
			       "sdioc_bytes_free_in_buffer(). status=%d\n",
			       __func__, status);
			return status;
		}

		/*
		 * if there is free room in outgoing buffer
		 * lock mutex and request trigger notification from the main
		 * task. unlock mutex, and wait for sinal
		 */
		if (dst_free_bytes > 0) {
			write_retry = 0;
			/*
			 * if there is more data to write to outgoing buffer
			 * than it can receive, wait for signal from main task
			 */
			if (pending_to_write > dst_free_bytes) {

				/* sampling updated dst_free_bytes */
				status = sdioc_bytes_free_in_buffer(
				outgoing->offset_write_p,
				outgoing->offset_read_p,
				outgoing->buffer_size,
				&dst_free_bytes,
				&dummy);

				if (status) {
					pr_err(MODULE_NAME ": %s - Failure in "
							   "Function "
					       "sdioc_bytes_free_in_buffer(). "
					       "status=%d\n", __func__, status);
					return status;
				}
			}

			bytes_to_write = min(pending_to_write, dst_free_bytes);
			bytes_written =
				writing_size_to_buf(outgoing->data,
						    buf+total_written,
						    &outgoing->offset_write_p,
						    outgoing->buffer_size,
						    outgoing->offset_read_p,
						    bytes_to_write);

			spin_lock_irqsave(&lock2, lock_flags2);
			if (sdio_dld.main_loop_event.wake_up_signal == 0) {
				sdio_dld.main_loop_event.wake_up_signal = 1;
				wake_up(&sdio_dld.main_loop_event.wait_event);
			}
			spin_unlock_irqrestore(&lock2, lock_flags2);

			/*
			 * although outgoing buffer has enough room, writing
			 * failed
			 */
			if (bytes_written != bytes_to_write) {
				pr_err(MODULE_NAME ": %s - couldn't write "
				       "%d bytes to " "outgoing buffer."
				       "bytes_written=%d\n",
				       __func__, bytes_to_write,
				       bytes_written);
			       return -EIO;
			}

			total_written += bytes_written;
			pending_to_write -= bytes_written;
			outgoing->num_of_bytes_in_use += bytes_written;

			pr_debug(MODULE_NAME ": %s - WRITE CHUNK to outgoing "
					   "buffer. pending_to_write=%d, "
					   "outgoing_free_bytes=%d, "
					   "bytes_written=%d\n",
				 __func__,
				 pending_to_write,
				 dst_free_bytes,
				 bytes_written);

		} else {
			write_retry++;

			pr_debug(MODULE_NAME ": %s - WRITE CALLBACK - NO ROOM."
			       " pending_to_write=%d, write_retry=%d\n",
				 __func__,
				 pending_to_write,
				 write_retry);

			spin_lock_irqsave(&lock1, lock_flags1);
			sdio_dld.write_callback_event.wake_up_signal = 0;
			spin_unlock_irqrestore(&lock1, lock_flags1);

			pr_debug(MODULE_NAME ": %s - WRITE CALLBACK - "
					     "WAITING...", __func__);

			wait_event(sdio_dld.write_callback_event.wait_event,
				   sdio_dld.write_callback_event.
				   wake_up_signal);

			pr_debug(MODULE_NAME ": %s - WRITE CALLBACK - "
					     "WOKE UP...", __func__);

		}

	} while (pending_to_write > 0 && write_retry < WRITE_RETRIES);

	if (pending_to_write > 0) {

		pr_err(MODULE_NAME ": %s - WRITE CALLBACK - pending data is "
				   "%d out of %d > 0. total written in this "
				   "callback = %d\n",
		       __func__, pending_to_write, count, total_written);

	}

	if (write_retry == WRITE_RETRIES) {
		pr_err(MODULE_NAME ": %s, write_retry=%d= max\n",
		       __func__, write_retry);
	}

	return total_written;
}

/**
  * sdio_memcpy_fromio_wrapper -
  * reads from sdioc, and updats the sdioc registers according
  * to how many bytes were actually read.
  *
  * @str_func: a pointer to func struct.
  * @client_rd_ptr: sdioc value of downlink read ptr.
  * @client_wr_ptr: sdioc value of downlink write ptr.
  * @buffer_to_store: buffer to store incoming data.
  * @address_to_read: address to start reading from in sdioc.
  * @size_to_read: size of bytes to read.
  * @client_buffer_size: sdioc downlink buffer size.
  * @return 0 on success or negative value on error.
  */
static int sdio_memcpy_fromio_wrapper(struct sdio_func *str_func,
				      unsigned int client_rd_ptr,
				      unsigned int client_wr_ptr,
				      void *buffer_to_store,
				      unsigned int address_to_read_from,
				      int size_to_read,
				      int client_buffer_size)
{
	int status = 0;
	struct sdioc_reg_chunk *reg_str =
		&sdio_dld.sdio_dloader_data.sdioc_reg;

	if (!str_func) {
		pr_err(MODULE_NAME ": %s - param ""str_func"" is NULL.\n",
		       __func__);
		return -EINVAL;
	}

	if (!buffer_to_store) {
		pr_err(MODULE_NAME ": %s - param ""buffer_to_store"" is "
				   "NULL.\n",
		       __func__);
		return -EINVAL;
	}

	if (size_to_read < 0) {
		pr_err(MODULE_NAME ": %s - invalid size to read=%d\n",
			__func__, size_to_read);
		return -EINVAL;
	}

	sdio_claim_host(str_func);

	pr_debug(MODULE_NAME ": %s, READING DATA - from add %d, "
			   "size_to_read=%d\n",
	       __func__, address_to_read_from, size_to_read);

	status = sdio_memcpy_fromio(str_func,
				    (void *)buffer_to_store,
				    address_to_read_from,
				    size_to_read);
	if (status) {
		pr_err(MODULE_NAME ": %s - sdio_memcpy_fromio()"
		       " DATA failed. status=%d.\n",
		       __func__, status);
		sdio_release_host(str_func);
		return status;
	}

	/* updating an offset according to cyclic buffer size */
	reg_str->dl_rd_ptr.reg_val =
		(reg_str->dl_rd_ptr.reg_val + size_to_read) %
		client_buffer_size;

	status = sdio_memcpy_toio(str_func,
				  reg_str->dl_rd_ptr.reg_offset,
				  (void *)&reg_str->dl_rd_ptr.reg_val,
				  sizeof(reg_str->dl_rd_ptr.reg_val));

	if (status) {
		pr_err(MODULE_NAME ": %s - sdio_memcpy_toio() "
		       "UPDATE PTR failed. status=%d.\n",
		       __func__, status);
	}

	sdio_release_host(str_func);
	return status;
}

/**
  * sdio_memcpy_toio_wrapper
  * reads from sdioc, and updats the sdioc registers according
  * to how many bytes were actually read.
  *
  * @str_func: a pointer to func struct.
  * @client_wr_ptr: sdioc downlink write ptr.
  * @h_read_ptr: host incoming read ptrs
  * @buf_write_from: buffer to write from.
  * @bytes_to_write: number of bytes to write.
  * @return 0 on success or negative value on error.
  */
static int sdio_memcpy_toio_wrapper(struct sdio_func *str_func,
				    unsigned int client_wr_ptr,
				    unsigned int h_read_ptr,
				    void *buf_write_from,
				    int bytes_to_write)
{
	int status = 0;
	struct sdioc_reg_chunk *reg_str =
		&sdio_dld.sdio_dloader_data.sdioc_reg;
	struct sdio_data *outgoing = &sdio_dld.sdio_dloader_data.outgoing_data;

	if (!str_func) {
		pr_err(MODULE_NAME ": %s - param ""str_func"" is NULL.\n",
		       __func__);
		return -EINVAL;
	}

	if (!buf_write_from) {
		pr_err(MODULE_NAME ": %s - param ""buf_write_from"" is NULL.\n",
		       __func__);
		return -EINVAL;
	}

	sdio_claim_host(str_func);

	pr_debug(MODULE_NAME ": %s, WRITING DATA TOIO to address 0x%x, "
	       "bytes_to_write=%d\n",
	       __func__, SDIOC_UL_BUFF_BASE+reg_str->up_wr_ptr.reg_val,
	       bytes_to_write);

	status = sdio_memcpy_toio(str_func,
				  SDIOC_UL_BUFF_BASE+reg_str->up_wr_ptr.reg_val,
				  (void *) (outgoing->data + h_read_ptr),
				  bytes_to_write);

	if (status) {
		pr_err(MODULE_NAME ": %s - sdio_memcpy_toio() "
		       "DATA failed. status=%d.\n", __func__, status);
		sdio_release_host(str_func);
		return status;
	}

	outgoing->num_of_bytes_in_use -= bytes_to_write;

	/*
	 * if writing to client succeeded, then
	 * 1. update the client up_wr_ptr
	 * 2. update the host outgoing rd ptr
	 **/
	reg_str->up_wr_ptr.reg_val =
		((reg_str->up_wr_ptr.reg_val + bytes_to_write) %
		 reg_str->ul_buff_size.reg_val);

	outgoing->offset_read_p =
		((outgoing->offset_read_p + bytes_to_write) %
		  outgoing->buffer_size);

	/* updating uplink write pointer according to size that was written */
	status = sdio_memcpy_toio(str_func,
				  reg_str->up_wr_ptr.reg_offset,
				  (void *)(&reg_str->up_wr_ptr.reg_val),
				  sizeof(reg_str->up_wr_ptr.reg_val));
	if (status) {
		pr_err(MODULE_NAME ": %s - sdio_memcpy_toio() "
				       "UPDATE PTR failed. status=%d.\n",
		       __func__, status);
	}

	sdio_release_host(str_func);
	return status;
}

/**
  * sdio_dld_write - writes to sdioc
  * @str_func: a pointer to func struct.
  * @reg_str: sdioc register shadowing struct.
  * @bytes_to_write: number of bytes to write.
  * @h_out_rd_ptr: host outgoing buffer read pointer.
  * @h_out_wr_ptr: host outgoing buffer write pointer.
  * @c_ul_rd_ptr: client uplink read pointer.
  * @c_ul_wr_ptr: client uplink write pointer.
  * @return 0 on success or negative value on error.
  */
static int sdio_dld_write(struct sdio_func *str_func,
			  struct sdioc_reg_chunk *reg_str,
			  int bytes_to_write,
			  int h_out_rd_ptr,
			  int h_out_wr_ptr,
			  int c_ul_rd_ptr,
			  int c_ul_wr_ptr)
{
	struct sdio_data *outgoing = &sdio_dld.sdio_dloader_data.outgoing_data;
	int status = 0;

	/* if there is NO rollover in host outgoing buffer */
	if ((h_out_rd_ptr + bytes_to_write) < outgoing->buffer_size) {
		status = sdio_memcpy_toio_wrapper(
			str_func,
			reg_str->up_wr_ptr.reg_val,
			outgoing->offset_read_p,
			(void *)((char *)outgoing->data +
				 outgoing->offset_read_p),
			bytes_to_write);

		if (status) {
			pr_err(MODULE_NAME ": %s - Failure in Function "
			       "sdio_memcpy_toio_wrapper(). "
			       "SINGLE CHUNK WRITE. status=%d\n",
			       __func__, status);
			return status;
		}
	}
	/*
	* there is rollover in the outgoing buffer,so must
	* send in 2 chunks
	*/
	else {
		/* writing chunk#1 - from read ptr till the
		*  end of outgoing buffer
		*/
		int chunk_1_size = outgoing->buffer_size -
			outgoing->offset_read_p;
		int chunk_2_size = bytes_to_write - chunk_1_size;

		status = sdio_memcpy_toio_wrapper(
			str_func,
			reg_str->up_wr_ptr.reg_val,
			outgoing->offset_read_p,
			(void *)((char *)outgoing->data +
				outgoing->offset_read_p),
			chunk_1_size);

		if (status) {
			pr_err(MODULE_NAME ": %s - Failure in Function "
			       "sdio_memcpy_toio_wrapper(). "
			       "1 of 2 CHUNKS WRITE. status=%d\n",
			       __func__, status);
			return status;
		}

		status = sdio_memcpy_toio_wrapper(
			str_func,
			reg_str->up_wr_ptr.reg_val,
			outgoing->offset_read_p,
			(void *)((char *)outgoing->data
			+ outgoing->offset_read_p),
			chunk_2_size);

		if (status) {
			pr_err(MODULE_NAME ": %s - Failure in Function "
			       "sdio_memcpy_toio_wrapper(). "
			       "2 of 2 CHUNKS WRITE. status=%d\n",
			       __func__, status);
			return status;
		}
	}
	return 0;
}

/**
  * sdio_dld_read
  * reads from sdioc
  *
  * @client_rd_ptr: sdioc downlink read ptr.
  * @client_wr_ptr: sdioc downlink write ptr.
  * @reg_str: sdioc register shadowing struct.
  * @str_func: a pointer to func struct.
  * @return 0 on success or negative value on error.
  */
static int sdio_dld_read(unsigned int client_rd_ptr,
			 unsigned int client_wr_ptr,
			 struct sdioc_reg_chunk *reg_str,
			 struct sdio_func *str_func)
{
	int status = 0;
	struct sdio_data *incoming = &sdio_dld.sdio_dloader_data.incoming_data;

	if (!reg_str) {
		pr_err(MODULE_NAME ": %s - param ""reg_str"" is NULL.\n",
		       __func__);
		return -EINVAL;
	}

	if (!str_func) {
		pr_err(MODULE_NAME ": %s - param ""str_func"" is NULL.\n",
		       __func__);
		return -EINVAL;
	}

	/* there is data to read in ONE chunk */
	if (client_wr_ptr > client_rd_ptr) {
		status = sdio_memcpy_fromio_wrapper(
			str_func,
			client_rd_ptr,
			client_wr_ptr,
			(void *)incoming->data,
			SDIOC_DL_BUFF_BASE + client_rd_ptr,
			client_wr_ptr - client_rd_ptr,
			reg_str->dl_buff_size.reg_val);

		if (status) {
			pr_err(MODULE_NAME ": %s - Failure in Function "
			       "sdio_memcpy_fromio_wrapper(). "
			       "SINGLE CHUNK READ. status=%d\n",
			       __func__, status);
			return status;
		}

		incoming->num_of_bytes_in_use += client_wr_ptr - client_rd_ptr;
	}

	/* there is data to read in TWO chunks */
	else {
		int dl_buf_size = reg_str->dl_buff_size.reg_val;
		int tail_size = dl_buf_size - client_rd_ptr;

		/* reading chunk#1: from rd_ptr to the end of the buffer */
		status = sdio_memcpy_fromio_wrapper(
			str_func,
			client_rd_ptr,
			dl_buf_size,
			(void *)incoming->data,
			SDIOC_DL_BUFF_BASE + client_rd_ptr,
			tail_size,
			dl_buf_size);

		if (status) {
			pr_err(MODULE_NAME ": %s - Failure in Function "
			       "sdio_memcpy_fromio_wrapper(). "
			       "1 of 2 CHUNKS READ. status=%d\n",
			       __func__, status);
			return status;
		}

		incoming->num_of_bytes_in_use += tail_size;

		/* reading chunk#2: reading from beginning buffer */
		status = sdio_memcpy_fromio_wrapper(
			str_func,
			client_rd_ptr,
			client_wr_ptr,
			(void *)(incoming->data + tail_size),
			SDIOC_DL_BUFF_BASE,
			client_wr_ptr,
			reg_str->dl_buff_size.reg_val);

		if (status) {
			pr_err(MODULE_NAME ": %s - Failure in Function "
			       "sdio_memcpy_fromio_wrapper(). "
			       "2 of 2 CHUNKS READ. status=%d\n",
			       __func__, status);
			return status;
		}

		incoming->num_of_bytes_in_use += client_wr_ptr;
	}
	return 0;
}

/**
  * sdio_dld_main_task
  * sdio downloader main task. reads mailboxf checks if there is
  * anything to read, checks if host has anything to
  * write.
  *
  * @card: a pointer to mmc_card.
  * @return 0 on success or negative value on error.
  */
static int sdio_dld_main_task(void *card)
{
	int status = 0;
	struct tty_struct *tty = sdio_dld.tty_str;
	struct sdioc_reg_chunk *reg_str =
		&sdio_dld.sdio_dloader_data.sdioc_reg;
	int func = sdio_dld.sdioc_boot_func;
	struct sdio_func *str_func = NULL;
	struct sdio_data *outgoing = &sdio_dld.sdio_dloader_data.outgoing_data;
	struct sdio_data *incoming = &sdio_dld.sdio_dloader_data.incoming_data;
	struct sdio_dld_task *task = &sdio_dld.dld_main_thread;

	msleep(SLEEP_MS);

	if (!card) {
		pr_err(MODULE_NAME ": %s - param ""card"" is NULL.\n",
		       __func__);
		return -EINVAL;
	}

	if (!tty) {
		pr_err(MODULE_NAME ": %s - param ""tty"" is NULL.\n",
		       __func__);
		return -EINVAL;
	}

	str_func = ((struct mmc_card *)card)->
		sdio_func[REAL_FUNC_TO_FUNC_IN_ARRAY(func)];

	if (!str_func) {
		pr_err(MODULE_NAME ": %s - param ""str_func"" is NULL.\n",
		       __func__);
		return -EINVAL;
	}

	while (true) {
		/* client pointers for both buffers */
		int client_ul_wr_ptr = 0;
		int client_ul_rd_ptr = 0;
		int client_dl_wr_ptr = 0;
		int client_dl_rd_ptr = 0;

		/* host pointer for outgoing buffer */
		int h_out_wr_ptr = 0;
		int h_out_rd_ptr = 0;

		int h_bytes_rdy_wr = 0;
		int c_bytes_rdy_rcve = 0;

		int need_to_write = 0;
		int need_to_read = 0;

		/*
		 * forever, checking for signal to die, then read MailBox.
		 * if nothing to read or nothing to write to client, sleep,
		 * and again read MailBox
		 */
		do {
			int dummy = 0;

			/*  checking if a signal to die was sent */
			if (atomic_read(&task->please_close) == 1) {

				pr_debug(MODULE_NAME ": %s - 0x%x was written "
					 "to 9K\n", __func__, SDIOC_EXIT_CODE);

				/* returned value is not checked on purpose */
				sdio_memcpy_toio(
					str_func,
					reg_str->good_to_exit_ptr.reg_offset,
					(void *)&reg_str->good_to_exit_ptr.
					reg_val,
					sizeof(reg_str->good_to_exit_ptr.
					       reg_val));

				task->exit_wait.wake_up_signal = 1;
				wake_up(&task->exit_wait.wait_event);
				return 0;
			}

			status = mailbox_to_seq_chunk_read(str_func);
			if (status) {
				pr_err(MODULE_NAME ": %s - Failure in Function "
				       "mailbox_to_seq_chunk_read(). "
				       "status=%d\n", __func__, status);
				return status;
			}

			/* calculate how many bytes the host has send */
			h_out_wr_ptr = outgoing->offset_write_p;
			h_out_rd_ptr = outgoing->offset_read_p;

			status = sdioc_bytes_free_in_buffer(
				h_out_wr_ptr,
				h_out_rd_ptr,
				outgoing->buffer_size,
				&dummy,
				&h_bytes_rdy_wr);

			if (status) {
				pr_err(MODULE_NAME ": %s - Failure in Function "
				       "sdioc_bytes_free_in_buffer(). "
				       "status=%d\n", __func__, status);
				return status;
			}
#ifdef THRESH_CHANGE
			/* if host has no data to send, reset threshold */
			if (h_bytes_rdy_wr == 0)
				outgoing->data_thresh =
				thresh_set(reg_str->ul_buff_size.reg_val);
#endif

			/* is there something to read from client */
			client_dl_wr_ptr = reg_str->dl_wr_ptr.reg_val;
			client_dl_rd_ptr = reg_str->dl_rd_ptr.reg_val;

			if (client_dl_rd_ptr != client_dl_wr_ptr)
				need_to_read = 1;

			/*
			 *  calculate how many bytes the client can receive
			 *  from host
			 */
			client_ul_wr_ptr = reg_str->up_wr_ptr.reg_val;
			client_ul_rd_ptr = reg_str->up_rd_ptr.reg_val;

			status = sdioc_bytes_free_in_buffer(
				client_ul_wr_ptr,
				client_ul_rd_ptr,
				reg_str->ul_buff_size.reg_val,
				&c_bytes_rdy_rcve,
				&dummy);

			if (status) {
				pr_err(MODULE_NAME ": %s - Failure in Function "
				       "sdioc_bytes_free_in_buffer(). "
				       "status=%d\n", __func__, status);
				return status;
			}

			/*
			 * if host has enough data to send AND client has
			 * enough room to receive it
			 */
			if ((h_bytes_rdy_wr >= outgoing->data_thresh) &&
			    (c_bytes_rdy_rcve >
			      min(h_bytes_rdy_wr, outgoing->data_thresh))) {
				need_to_write = 1;
			}

			/*
			 * if host has little to send
			 * AND client has room to recieve it
			 */
			if ((h_bytes_rdy_wr > 0) &&
			     (c_bytes_rdy_rcve >
			      min(h_bytes_rdy_wr, outgoing->data_thresh))) {
				need_to_write = 1;
			}

			if (need_to_write || need_to_read)
				break;

			spin_lock_irqsave(&lock2, lock_flags2);
			sdio_dld.main_loop_event.wake_up_signal = 0;
			spin_unlock_irqrestore(&lock2, lock_flags2);

			pr_debug(MODULE_NAME ": %s - MAIN LOOP - WAITING...\n",
				 __func__);

			wait_event(sdio_dld.main_loop_event.wait_event,
				   sdio_dld.main_loop_event.wake_up_signal);


			pr_debug(MODULE_NAME ": %s - MAIN LOOP - WOKE UP...\n",
				 __func__);

		} while (1);

		/* CHECK IF THERE IS ANYTHING TO READ IN CLIENT */
		if (need_to_read) {
#ifdef PUSH_STRING
			int num_push = 0;
			int total_push = 0;
			int left = 0;
#else
			int i;
#endif
			need_to_read = 0;

			status = sdio_dld_read(client_dl_rd_ptr,
					       client_dl_wr_ptr,
					       reg_str,
					       str_func);

			if (status) {
				pr_err(MODULE_NAME ": %s - Failure in Function "
				       "sdio_dld_read(). status=%d\n",
				       __func__, status);
				return status;
			}

#ifdef PUSH_STRING
			left = incoming->num_of_bytes_in_use;
			do {
				num_push = tty_insert_flip_string(
					tty,
					incoming->data+total_push,
					left);
				total_push += num_push;
				left -= num_push;
				tty_flip_buffer_push(tty);
			} while (left != 0);

			if (total_push != incoming->num_of_bytes_in_use) {
				pr_err(MODULE_NAME ": %s - failed\n",
				       __func__);
			}
#else
			pr_debug(MODULE_NAME ": %s - NEED TO READ %d\n",
			       __func__, incoming->num_of_bytes_in_use);

			for (i = 0 ; i < incoming->num_of_bytes_in_use ; ++i) {
				int err = 0;
				err = tty_insert_flip_char(tty,
							   incoming->data[i],
							   TTY_NORMAL);
				tty_flip_buffer_push(tty);
			}

			pr_debug(MODULE_NAME ": %s - JUST READ\n", __func__);

#endif /*PUSH_STRING*/
			incoming->num_of_bytes_in_use = 0;
			tty_flip_buffer_push(tty);
		}

		/* CHECK IF THERE IS ANYTHING TO WRITE IN HOST AND HOW MUCH */
		if (need_to_write) {
			int dummy = 0;

			do {
				int bytes_to_write = min(c_bytes_rdy_rcve,
							 h_bytes_rdy_wr);

				if (c_bytes_rdy_rcve < h_bytes_rdy_wr)
					pr_debug(MODULE_NAME ":%s - 9K NOT FAST"
						 " ENOUGH! c_bytes_rdy_rcve=%d,"
						 " h_bytes_rdy_wr=%d\n",
						 __func__,
						 c_bytes_rdy_rcve,
						 h_bytes_rdy_wr);

				need_to_write = 0;
#ifdef THRESH_CHANGE
				/*
				 * if host has data to send,
				 * setting threshold to 0
				 */
				if (h_bytes_rdy_wr >= SIZE_2K)
					outgoing->data_thresh = 0;
#endif
				pr_debug(MODULE_NAME ": %s - NEED TO WRITE "
					 "TOIO %d\n",
					 __func__, bytes_to_write);

				sdio_dld_write(str_func,
					       reg_str,
					       bytes_to_write,
					       h_out_rd_ptr,
					       h_out_wr_ptr,
					       client_ul_rd_ptr,
					       client_ul_wr_ptr);

				sdio_claim_host(str_func);

				status = sdio_memcpy_fromio(
					str_func,
					(void *)&reg_str->up_rd_ptr.reg_val,
					SDIOC_UL_RD_PTR,
					sizeof(reg_str->up_rd_ptr.reg_val));

				if (status) {
					pr_err(MODULE_NAME ": %s - "
					       "sdio_memcpy_fromio() "
					       "failed. status=%d\n",
					       __func__, status);
					sdio_release_host(str_func);

					return status;
				}

				sdio_release_host(str_func);

				spin_lock_irqsave(&lock1, lock_flags1);
				if (sdio_dld.write_callback_event.
				    wake_up_signal == 0) {
					sdio_dld.write_callback_event.
						wake_up_signal = 1;
					wake_up(&sdio_dld.
						write_callback_event.
						wait_event);
				}

				spin_unlock_irqrestore(&lock1, lock_flags1);
				client_ul_wr_ptr = reg_str->up_wr_ptr.reg_val;
				client_ul_rd_ptr = reg_str->up_rd_ptr.reg_val;
				status = sdioc_bytes_free_in_buffer(
					client_ul_wr_ptr,
					client_ul_rd_ptr,
					reg_str->ul_buff_size.reg_val,
					&c_bytes_rdy_rcve,
					&dummy);

				/* calculate how many bytes host has to send */
				h_out_wr_ptr = outgoing->offset_write_p;
				h_out_rd_ptr = outgoing->offset_read_p;
				status = sdioc_bytes_free_in_buffer(
					h_out_wr_ptr,
					h_out_rd_ptr,
					outgoing->buffer_size,
					&dummy,
					&h_bytes_rdy_wr);
				} while (h_out_wr_ptr != h_out_rd_ptr);
		}
	}
	return 0;
}

/**
  * sdio_dld_init_global
  * initialization of sdio_dld global struct
  *
  * @card: a pointer to mmc_card.
  * @return 0 on success or negative value on error.
  */
static int sdio_dld_init_global(struct mmc_card *card,
				int(*done)(void))
{
	if (!card) {
		pr_err(MODULE_NAME ": %s - param ""card"" is NULL.\n",
		       __func__);
		return -EINVAL;
	}

	if (!done) {
		pr_err(MODULE_NAME ": %s - param ""done"" is NULL.\n",
		       __func__);
		return -EINVAL;
	}

	sdio_dld.done_callback = done;
	sdio_dld.card = card;
	init_waitqueue_head(&sdio_dld.dld_main_thread.exit_wait.wait_event);
	sdio_dld.write_callback_event.wake_up_signal = 1;
	sdio_dld.main_loop_event.wake_up_signal = 1;

	sdio_dld.sdio_dloader_data.sdioc_reg.dl_buff_size.reg_offset =
		SDIOC_DL_BUFF_SIZE_OFFSET;
	sdio_dld.sdio_dloader_data.sdioc_reg.dl_rd_ptr.reg_offset =
		SDIOC_DL_RD_PTR;
	sdio_dld.sdio_dloader_data.sdioc_reg.dl_wr_ptr.reg_offset =
		SDIOC_DL_WR_PTR;
	sdio_dld.sdio_dloader_data.sdioc_reg.ul_buff_size.reg_offset =
		SDIOC_UP_BUFF_SIZE_OFFSET;
	sdio_dld.sdio_dloader_data.sdioc_reg.up_rd_ptr.reg_offset =
		SDIOC_UL_RD_PTR;
	sdio_dld.sdio_dloader_data.sdioc_reg.up_wr_ptr.reg_offset =
		SDIOC_UL_WR_PTR;
	sdio_dld.sdio_dloader_data.sdioc_reg.good_to_exit_ptr.reg_offset =
		SDIOC_EXIT_PTR;

	return 0;
}

/**
 * sdio_downloader_setup
 * initializes the TTY driver
 *
 * @card: a pointer to mmc_card.
 * @num_of_devices: number of devices.
 * @channel_number: channel number.
 * @return 0 on success or negative value on error.
 *
 * The TTY stack needs to know in advance how many devices it should
 * plan to manage. Use this call to set up the ports that will
 * be exported through SDIO.
 */
int sdio_downloader_setup(struct mmc_card *card,
			  unsigned int num_of_devices,
			  int channel_number,
			  int(*done)(void))
{
	unsigned i;
	int status = 0;

	if (num_of_devices == 0 || num_of_devices > MAX_NUM_DEVICES) {
		pr_err(MODULE_NAME ": %s - invalid number of devices\n",
		       __func__);
		return -EINVAL;
	}

	if (!card) {
		pr_err(MODULE_NAME ": %s - param ""card"" is NULL.\n",
		       __func__);
		return -EINVAL;
	}

	if (!done) {
		pr_err(MODULE_NAME ": %s - param ""done"" is NULL.\n",
		       __func__);
		return -EINVAL;
	}

	status = sdio_dld_init_global(card, done);

	if (status) {
		pr_err(MODULE_NAME ": %s - Failure in Function "
		       "sdio_dld_init_global(). status=%d\n",
		       __func__, status);
		return status;
	}

	sdio_dld.tty_drv = alloc_tty_driver(num_of_devices);

	if (!sdio_dld.tty_drv) {
		pr_err(MODULE_NAME ": %s - param ""sdio_dld.tty_drv"" is "
				   "NULL.\n", __func__);
		return -EINVAL;
	}

	sdio_dld.tty_drv->owner = THIS_MODULE;
	sdio_dld.tty_drv->driver_name = "SDIO_Dloader";
	sdio_dld.tty_drv->name = TTY_SDIO_DEV;

	/* uses dynamically assigned dev_t values */
	sdio_dld.tty_drv->type = TTY_DRIVER_TYPE_SERIAL;
	sdio_dld.tty_drv->subtype = SERIAL_TYPE_NORMAL;
	sdio_dld.tty_drv->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV
				| TTY_DRIVER_RESET_TERMIOS;

	/* initializing the tty driver */
	sdio_dld.tty_drv->init_termios = tty_std_termios;
	sdio_dld.tty_drv->init_termios.c_cflag =
		B4800 | CS8 | CREAD | HUPCL | CLOCAL;
	sdio_dld.tty_drv->init_termios.c_ispeed = INPUT_SPEED;
	sdio_dld.tty_drv->init_termios.c_ospeed = OUTPUT_SPEED;

	tty_set_operations(sdio_dld.tty_drv, &sdio_dloader_tty_ops);
	sdio_dld.sdioc_boot_func = SDIOC_CHAN_TO_FUNC_NUM(channel_number);

	status = tty_register_driver(sdio_dld.tty_drv);
	if (status) {
		put_tty_driver(sdio_dld.tty_drv);
		pr_err(MODULE_NAME ": %s - tty_register_driver() failed\n",
			__func__);

		sdio_dld.tty_drv = NULL;
		return status;
	}

	for (i = 0; i < num_of_devices ; i++) {
		struct device *tty_dev;

		tty_dev = tty_register_device(sdio_dld.tty_drv, i, NULL);
		if (IS_ERR(tty_dev)) {
			pr_err(MODULE_NAME ": %s - tty_register_device() "
				"failed\n", __func__);
			tty_unregister_driver(sdio_dld.tty_drv);
			return PTR_ERR(tty_dev);
		}
	}

	sdio_dld.tty_str = tty_init_dev(sdio_dld.tty_drv, 0, 1);
	if (!sdio_dld.tty_str) {
		pr_err(MODULE_NAME ": %s - param ""sdio_dld.tty_str"" is "
				   "NULL.\n", __func__);

		tty_unregister_device(sdio_dld.tty_drv, 0);
		status = tty_unregister_driver(sdio_dld.tty_drv);
		if (status)
			pr_err(MODULE_NAME ": %s - tty_unregister_driver() "
					   "failed\n", __func__);
		return -EINVAL;
	}

	sdio_dld.tty_str->low_latency = 1;
	sdio_dld.tty_str->icanon = 0;

	return 0;
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SDIO Downloader");
MODULE_AUTHOR("Yaniv Gardi <ygardi@codeaurora.org>");
MODULE_VERSION(DRV_VERSION);

