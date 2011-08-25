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
 * SDIO-Abstraction-Layer Test Module.
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/delay.h>

#include <mach/sdio_al.h>

/** Module name string */
#define MODULE_NAME "sdio_al_test"

/* Force Debug ON
#undef pr_debug
#define pr_debug pr_info */

#define TEST_SIGNATURE 0x12345678

#define MAX_XFER_SIZE (16*1024)

struct test_context {
	dev_t dev_num;
	struct device *dev;
	struct cdev *cdev;

	struct sdio_channel *ch;
	u32 *buf;
	u32 buf_size;

	struct workqueue_struct *workqueue;
	struct work_struct work;

	u32 rx_bytes;
	u32 tx_bytes;

	wait_queue_head_t   wait_q;
	int rx_notify_count;
	int tx_notify_count;

	long testcase;

	const char *name;

	int exit_flag;

	struct mutex lock;

	int wait_counter;

	u32 signature;
};

static struct test_context *test_ctx;

/**
 * Loopback Test
 */
static void loopback_test(void)
{
	int ret = 0 ;
	u32 read_avail = 0;
	u32 write_avail = 0;


	while (1) {

		if (test_ctx->exit_flag) {
			pr_info(MODULE_NAME ":Exit Test.\n");
			return;
		}


		pr_info(MODULE_NAME "--LOOPBACK WAIT FOR EVENT--.\n");

		/* wait for data ready event */
		wait_event(test_ctx->wait_q, test_ctx->rx_notify_count);
		test_ctx->rx_notify_count--;

		read_avail = sdio_read_avail(test_ctx->ch);
		if (read_avail == 0)
			continue;


		write_avail = sdio_write_avail(test_ctx->ch);
		if (write_avail < read_avail) {
			pr_info(MODULE_NAME
				":not enough write avail.\n");
			continue;
		}


		ret = sdio_read(test_ctx->ch, test_ctx->buf, read_avail);
		if (ret) {
			pr_info(MODULE_NAME
			       ":worker, sdio_read err=%d.\n", -ret);
			continue;
		}
		test_ctx->rx_bytes += read_avail;

		pr_debug(MODULE_NAME ":worker total rx bytes = 0x%x.\n",
			 test_ctx->rx_bytes);


			ret = sdio_write(test_ctx->ch,
					 test_ctx->buf, read_avail);
			if (ret) {
				pr_info(MODULE_NAME
					":loopback sdio_write err=%d.\n",
					-ret);
				continue;
			}
			test_ctx->tx_bytes += read_avail;

			pr_debug(MODULE_NAME
				 ":loopback total tx bytes = 0x%x.\n",
				 test_ctx->tx_bytes);
		} /* end of while */
}

/**
 * sender Test
 */
static void sender_test(void)
{
	int ret = 0 ;
	u32 read_avail = 0;
	u32 write_avail = 0;
	int packet_count = 0;
	int size = 512;
	u16 *buf16 = (u16 *) test_ctx->buf;
	int i;

	for (i = 0 ; i < size / 2 ; i++)
		buf16[i] = (u16) (i & 0xFFFF);

	sdio_set_write_threshold(test_ctx->ch, 4*1024);
	sdio_set_read_threshold(test_ctx->ch, 16*1024); /* N/A with Rx EOT  */
	sdio_set_poll_time(test_ctx->ch, 0); /* N/A with Rx EOT  */

	while (packet_count < 100) {

		if (test_ctx->exit_flag) {
			pr_info(MODULE_NAME ":Exit Test.\n");
			return;
		}

		pr_info(MODULE_NAME "--SENDER WAIT FOR EVENT--.\n");

		/* wait for data ready event */
		write_avail = sdio_write_avail(test_ctx->ch);
		pr_debug(MODULE_NAME ":write_avail=%d\n", write_avail);
		if (write_avail < size) {
			wait_event(test_ctx->wait_q, test_ctx->tx_notify_count);
			test_ctx->tx_notify_count--;
		}

		write_avail = sdio_write_avail(test_ctx->ch);
		pr_debug(MODULE_NAME ":write_avail=%d\n", write_avail);
		if (write_avail < size) {
			pr_info(MODULE_NAME ":not enough write avail.\n");
			continue;
		}

		test_ctx->buf[0] = packet_count;
		test_ctx->buf[(size/4)-1] = packet_count;

		ret = sdio_write(test_ctx->ch, test_ctx->buf, size);
		if (ret) {
			pr_info(MODULE_NAME ":sender sdio_write err=%d.\n",
				-ret);
			goto exit_err;
		}


		/* wait for read data ready event */
		pr_debug(MODULE_NAME ":sender wait for rx data.\n");
		read_avail = sdio_read_avail(test_ctx->ch);
		wait_event(test_ctx->wait_q, test_ctx->rx_notify_count);
		test_ctx->rx_notify_count--;

		read_avail = sdio_read_avail(test_ctx->ch);

		if (read_avail != size) {
			pr_info(MODULE_NAME
				":read_avail size %d not as expected.\n",
				read_avail);
			goto exit_err;
		}

		memset(test_ctx->buf, 0x00, size);

		ret = sdio_read(test_ctx->ch, test_ctx->buf, size);
		if (ret) {
			pr_info(MODULE_NAME ":sender sdio_read err=%d.\n",
				-ret);
			goto exit_err;
		}


		if ((test_ctx->buf[0] != packet_count) ||
		    (test_ctx->buf[(size/4)-1] != packet_count)) {
			pr_info(MODULE_NAME ":sender sdio_read WRONG DATA.\n");
			goto exit_err;
		}

		test_ctx->tx_bytes += size;
		test_ctx->rx_bytes += size;
		packet_count++;

		pr_debug(MODULE_NAME
			 ":sender total rx bytes = 0x%x , packet#=%d.\n",
			 test_ctx->rx_bytes, packet_count);
		pr_debug(MODULE_NAME
			 ":sender total tx bytes = 0x%x , packet#=%d.\n",
			 test_ctx->tx_bytes, packet_count);

	} /* end of while */

	sdio_close(test_ctx->ch);

	pr_info(MODULE_NAME ": TEST PASS.\n");
	return;

exit_err:
	sdio_close(test_ctx->ch);

	pr_info(MODULE_NAME ": TEST FAIL.\n");
	return;
}

int wait_any_notify(void)
{
	int wait_time_msec = 0;
	int max_wait_time_msec = 60*1000; /* 60 seconds */

	test_ctx->wait_counter++;

	pr_debug(MODULE_NAME ":Waiting for event %d ...\n",
		 test_ctx->wait_counter);
	while (wait_time_msec < max_wait_time_msec) {
		if (test_ctx->tx_notify_count > 0)
			return 0;

		if (test_ctx->rx_notify_count > 0)
			return 0;

		msleep(10);
		wait_time_msec += 10;
	}

	pr_info(MODULE_NAME ":Wait for event %d sec.\n",
		max_wait_time_msec/1000);

	return -1;
}

/**
 * A2 Perf Test
 */
static void a2_performance_test(void)
{
	int ret = 0 ;
	u32 read_avail = 0;
	u32 write_avail = 0;
	int tx_packet_count = 0;
	int rx_packet_count = 0;
	int size = 0;
	u16 *buf16 = (u16 *) test_ctx->buf;
	int i;
	int total_bytes = 0;
	int max_packets = 10000;

	u64 start_jiffy, end_jiffy, delta_jiffies;
	unsigned int time_msec = 0;

	for (i = 0; i < test_ctx->buf_size / 2; i++)
		buf16[i] = (u16) (i & 0xFFFF);

	pr_info(MODULE_NAME "--A2 PERFORMANCE TEST START --.\n");

	sdio_set_write_threshold(test_ctx->ch, 2*1024);
	sdio_set_read_threshold(test_ctx->ch, 14*1024);
	sdio_set_poll_time(test_ctx->ch, 0);

	start_jiffy = get_jiffies_64(); /* read the current time */

	while (tx_packet_count < max_packets) {

		if (test_ctx->exit_flag) {
			pr_info(MODULE_NAME ":Exit Test.\n");
			return;
		}

		/* wait for data ready event */
		/* use a func to avoid compiler optimizations */
		write_avail = sdio_write_avail(test_ctx->ch);
		read_avail = sdio_read_avail(test_ctx->ch);
		if ((write_avail == 0) && (read_avail == 0)) {
			ret = wait_any_notify();
			if (ret)
				goto exit_err;
		}

		mutex_lock(&test_ctx->lock);

		write_avail = sdio_write_avail(test_ctx->ch);
		if (write_avail > 0) {
			size = min(test_ctx->buf_size, write_avail) ;
			pr_debug(MODULE_NAME ":tx size = %d.\n", size);
			if (test_ctx->tx_notify_count > 0)
				test_ctx->tx_notify_count--;

			test_ctx->buf[0] = tx_packet_count;
			test_ctx->buf[(size/4)-1] = tx_packet_count;

			ret = sdio_write(test_ctx->ch, test_ctx->buf, size);
			if (ret) {
				pr_info(MODULE_NAME ":sdio_write err=%d.\n",
					-ret);
				goto exit_err;
			}
			tx_packet_count++;
			test_ctx->tx_bytes += size;
		}

		read_avail = sdio_read_avail(test_ctx->ch);
		if (read_avail > 0) {
			size = min(test_ctx->buf_size, read_avail);
			pr_debug(MODULE_NAME ":rx size = %d.\n", size);
			if (test_ctx->rx_notify_count > 0)
				test_ctx->rx_notify_count--;

			ret = sdio_read(test_ctx->ch, test_ctx->buf, size);

			if (ret) {
				pr_info(MODULE_NAME ": sdio_read err=%d.\n",
					-ret);
				goto exit_err;
			}
			rx_packet_count++;
			test_ctx->rx_bytes += size;
		}

		pr_debug(MODULE_NAME
			 ":total rx bytes = %d , rx_packet#=%d.\n",
			 test_ctx->rx_bytes, rx_packet_count);
		pr_debug(MODULE_NAME
			 ":total tx bytes = %d , tx_packet#=%d.\n",
			 test_ctx->tx_bytes, tx_packet_count);
loop_cont:
		mutex_unlock(&test_ctx->lock);

	   /* pr_info(MODULE_NAME ":packet#=%d.\n", tx_packet_count); */

	} /* while (tx_packet_count < max_packets ) */

	end_jiffy = get_jiffies_64(); /* read the current time */

	delta_jiffies = end_jiffy - start_jiffy;
	time_msec = jiffies_to_msecs(delta_jiffies);

	pr_info(MODULE_NAME ":total rx bytes = 0x%x , rx_packet#=%d.\n",
		test_ctx->rx_bytes, rx_packet_count);
	pr_info(MODULE_NAME ":total tx bytes = 0x%x , tx_packet#=%d.\n",
		test_ctx->tx_bytes, tx_packet_count);

	total_bytes = (test_ctx->tx_bytes + test_ctx->rx_bytes);
	pr_err(MODULE_NAME ":total bytes = %d, time msec = %d.\n",
		   total_bytes , (int) time_msec);

	pr_err(MODULE_NAME ":Performance = %d Mbit/sec.\n",
	(total_bytes / time_msec) * 8 / 1000) ;

	pr_err(MODULE_NAME "--A2 PERFORMANCE TEST END --.\n");

	pr_err(MODULE_NAME ": TEST PASS.\n");
	return;

exit_err:
	pr_err(MODULE_NAME ": TEST FAIL.\n");
	return;
}

/**
 * Worker thread to handle read/write events
 */
static void worker(struct work_struct *work)
{
	switch (test_ctx->testcase) {
	case 1:
		loopback_test();
		break;
	case 2:
		sender_test();
		break;
	case 3:
		a2_performance_test();
		break;
	default:
		pr_info(MODULE_NAME "Bad Test number = %d.\n",
			(int) test_ctx->testcase);
	}
}


/**
 * Notification Callback
 *
 * Notify the worker
 *
 */
static void notify(void *priv, unsigned channel_event)
{
	struct test_context *test_ctx = (struct test_context *) priv;

	pr_debug(MODULE_NAME ":notify event=%d.\n", channel_event);

	if (test_ctx->ch == NULL) {
		pr_info(MODULE_NAME ":notify before ch ready.\n");
		return;
	}

	mutex_lock(&test_ctx->lock);

	BUG_ON(test_ctx->signature != TEST_SIGNATURE);

	switch (channel_event) {
	case SDIO_EVENT_DATA_READ_AVAIL:
		test_ctx->rx_notify_count++;
		pr_debug(MODULE_NAME ":rx_notify_count=%d.\n",
			 test_ctx->rx_notify_count);
		wake_up(&test_ctx->wait_q);
		break;

	case SDIO_EVENT_DATA_WRITE_AVAIL:
		test_ctx->tx_notify_count++;
		pr_debug(MODULE_NAME ":tx_notify_count=%d.\n",
			 test_ctx->tx_notify_count);
		wake_up(&test_ctx->wait_q);
		break;

	default:
		BUG();
	}
	mutex_unlock(&test_ctx->lock);
}

/**
 * Test Main
 */
static int test_start(void)
{
	int ret = -ENOMEM;
	struct sdio_channel *ch = NULL;

	pr_debug(MODULE_NAME ":Starting Test ....\n");

	test_ctx->buf_size = MAX_XFER_SIZE;

	test_ctx->buf = kzalloc(test_ctx->buf_size, GFP_KERNEL);
	if (test_ctx->buf == NULL)
		goto err_alloc_buf;

	test_ctx->workqueue = create_singlethread_workqueue("sdio_al_test_wq");

	INIT_WORK(&test_ctx->work, worker);

	init_waitqueue_head(&test_ctx->wait_q);
	test_ctx->rx_bytes = 0;
	test_ctx->tx_bytes = 0;

	test_ctx->tx_notify_count = 0;
	test_ctx->rx_notify_count = 0;

	ret = sdio_open(test_ctx->name , &test_ctx->ch, test_ctx, notify);
	if (ret)
		goto err_sdio_open;
	ch = test_ctx->ch;

	memset(test_ctx->buf, 0x00, test_ctx->buf_size);

	queue_work(test_ctx->workqueue, &test_ctx->work);

	mutex_init(&test_ctx->lock);

	pr_debug(MODULE_NAME ":Test Start completed OK..\n");

	return 0;

err_sdio_open:
	kfree(test_ctx->buf);
err_alloc_buf:
	kfree(test_ctx);

	pr_debug(MODULE_NAME ":Test Start Failure..\n");

	return ret;
}

/**
 * Write File.
 *
 * @note Trigger the test from user space by:
 * echo 1 > /dev/sdio_al_test
 *
 */
ssize_t test_write(struct file *filp, const char __user *buf, size_t size,
		   loff_t *f_pos)
{
	int ret = 0;

	ret = strict_strtol(buf, 10, &test_ctx->testcase);

	switch (test_ctx->testcase) {
	case 1:
		pr_debug(MODULE_NAME " --Loopback--.\n");
		test_ctx->name = "SDIO_RPC";
		break;
	case 2:
		pr_debug(MODULE_NAME " --sender--.\n");
		test_ctx->name = "SDIO_QMI";
		break;
	case 3:
		pr_debug(MODULE_NAME " --A2 Performance--.\n");
		test_ctx->name = "SDIO_RMNET_DATA";
		break;
	default:
		pr_info(MODULE_NAME "Bad Test number = %d.\n",
			(int) test_ctx->testcase);
		return 0;

	}

	ret = test_start();

	return size;
}

static struct class *test_class;

const struct file_operations test_fops = {
	.owner = THIS_MODULE,
	.write = test_write,
};

/**
 * Module Init.
 */
static int __init test_init(void)
{
	int ret;

	pr_debug(MODULE_NAME ":test_init.\n");

	test_ctx = kzalloc(sizeof(*test_ctx), GFP_KERNEL);
	if (test_ctx == NULL) {
		pr_err(MODULE_NAME ":kzalloc err.\n");
		return -ENOMEM;
	}
	test_ctx->ch = NULL;
	test_ctx->signature = TEST_SIGNATURE;

	test_ctx->name = "UNKNOWN";

	test_class = class_create(THIS_MODULE, MODULE_NAME);

	ret = alloc_chrdev_region(&test_ctx->dev_num, 0, 1, MODULE_NAME);
	if (ret) {
		pr_err(MODULE_NAME "alloc_chrdev_region err.\n");
		return -ENODEV;
	}

	test_ctx->dev = device_create(test_class, NULL, test_ctx->dev_num,
				      test_ctx, MODULE_NAME);
	if (IS_ERR(test_ctx->dev)) {
		pr_err(MODULE_NAME ":device_create err.\n");
		return -ENODEV;
	}

	test_ctx->cdev = cdev_alloc();
	if (test_ctx->cdev == NULL) {
		pr_err(MODULE_NAME ":cdev_alloc err.\n");
		return -ENODEV;
	}
	cdev_init(test_ctx->cdev, &test_fops);
	test_ctx->cdev->owner = THIS_MODULE;

	ret = cdev_add(test_ctx->cdev, test_ctx->dev_num, 1);
	if (ret)
		pr_err(MODULE_NAME ":cdev_add err=%d\n", -ret);
	else
		pr_debug(MODULE_NAME ":SDIO-AL-Test init OK..\n");

	return ret;
}

/**
 * Module Exit.
 */
static void __exit test_exit(void)
{
	pr_debug(MODULE_NAME ":test_exit.\n");

	test_ctx->exit_flag = true;

	msleep(100); /* allow gracefully exit of the worker thread */

	sdio_close(test_ctx->ch);

	cdev_del(test_ctx->cdev);
	device_destroy(test_class, test_ctx->dev_num);
	unregister_chrdev_region(test_ctx->dev_num, 1);

	kfree(test_ctx->buf);
	kfree(test_ctx);

	pr_debug(MODULE_NAME ":test_exit complete.\n");
}

module_init(test_init);
module_exit(test_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SDIO_AL Test");
MODULE_AUTHOR("Amir Samuelov <amirs@codeaurora.org>");


