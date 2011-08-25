/* Copyright (c) 2008-2011, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef DIAGCHAR_H
#define DIAGCHAR_H

#include <linux/init.h>
#include <linux/module.h>
#include <linux/mempool.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <mach/msm_smd.h>
#include <asm/atomic.h>
#include <asm/mach-types.h>
/* Size of the USB buffers used for read and write*/
#define USB_MAX_OUT_BUF 4096
#define IN_BUF_SIZE		8192
#define MAX_IN_BUF_SIZE	32768
/* Size of the buffer used for deframing a packet
  reveived from the PC tool*/
#define HDLC_MAX 4096
#define HDLC_OUT_BUF_SIZE	8192
#define POOL_TYPE_COPY		1
#define POOL_TYPE_HDLC		2
#define POOL_TYPE_WRITE_STRUCT	4
#define POOL_TYPE_ALL		7
#define MODEM_DATA 		1
#define QDSP_DATA  		2
#define APPS_DATA  		3
#define SDIO_DATA		4
#define MSG_MASK_SIZE 8000
#define LOG_MASK_SIZE 8000
#define EVENT_MASK_SIZE 1000
#define PKT_SIZE 4096
#define MAX_EQUIP_ID 12
/* This is the maximum number of pkt registrations supported at initialization*/
extern unsigned int diag_max_registration;
extern unsigned int diag_threshold_registration;

#define APPEND_DEBUG(ch) \
do {							\
	diag_debug_buf[diag_debug_buf_idx] = ch; \
	(diag_debug_buf_idx < 1023) ? \
	(diag_debug_buf_idx++) : (diag_debug_buf_idx = 0); \
} while (0)

struct diag_master_table {
	uint16_t cmd_code;
	uint16_t subsys_id;
	uint16_t cmd_code_lo;
	uint16_t cmd_code_hi;
	int process_id;
};

struct diag_write_device {
	void *buf;
	int length;
};

struct diag_client_map {
	char name[20];
	int pid;
};

/* This structure is defined in USB header file */
#ifndef CONFIG_DIAG_OVER_USB
struct diag_request {
	char *buf;
	int length;
	int actual;
	int status;
	void *context;
};
#endif

struct diagchar_dev {

	/* State for the char driver */
	unsigned int major;
	unsigned int minor_start;
	int num;
	struct cdev *cdev;
	char *name;
	int dropped_count;
	struct class *diagchar_class;
	int ref_count;
	struct mutex diagchar_mutex;
	wait_queue_head_t wait_q;
	struct diag_client_map *client_map;
	int *data_ready;
	int num_clients;
	struct diag_write_device *buf_tbl;

	/* Memory pool parameters */
	unsigned int itemsize;
	unsigned int poolsize;
	unsigned int itemsize_hdlc;
	unsigned int poolsize_hdlc;
	unsigned int itemsize_write_struct;
	unsigned int poolsize_write_struct;
	unsigned int debug_flag;
	unsigned int alert_count;
	/* State for the mempool for the char driver */
	mempool_t *diagpool;
	mempool_t *diag_hdlc_pool;
	mempool_t *diag_write_struct_pool;
	struct mutex diagmem_mutex;
	int count;
	int count_hdlc_pool;
	int count_write_struct_pool;
	int used;

	/* State for diag forwarding */
	unsigned char *buf_in_1;
	unsigned char *buf_in_2;
	unsigned char *buf_in_qdsp_1;
	unsigned char *buf_in_qdsp_2;
	unsigned char *usb_buf_out;
	unsigned char *apps_rsp_buf;
	smd_channel_t *ch;
	smd_channel_t *chqdsp;
	int in_busy_1;
	int in_busy_2;
	int in_busy_qdsp_1;
	int in_busy_qdsp_2;
	int read_len_legacy;
	unsigned char *hdlc_buf;
	unsigned hdlc_count;
	unsigned hdlc_escape;
#ifdef CONFIG_DIAG_OVER_USB
	int usb_connected;
	struct usb_diag_ch *legacy_ch;
	struct work_struct diag_proc_hdlc_work;
	struct work_struct diag_read_work;
#endif
	struct workqueue_struct *diag_wq;
	struct work_struct diag_drain_work;
	struct work_struct diag_read_smd_work;
	struct work_struct diag_read_smd_qdsp_work;
	uint8_t *msg_masks;
	uint8_t *log_masks;
	int log_masks_length;
	uint8_t *event_masks;
	struct diag_master_table *table;
	uint8_t *pkt_buf;
	int pkt_length;
	struct diag_request *write_ptr_1;
	struct diag_request *write_ptr_2;
	struct diag_request *usb_read_ptr;
	struct diag_request *write_ptr_svc;
	struct diag_request *write_ptr_qdsp_1;
	struct diag_request *write_ptr_qdsp_2;
	int logging_mode;
	int logging_process_id;
#ifdef CONFIG_DIAG_SDIO_PIPE
	unsigned char *buf_in_sdio;
	unsigned char *usb_buf_mdm_out;
	struct sdio_channel *sdio_ch;
	int read_len_mdm;
	int in_busy_sdio;
	struct usb_diag_ch *mdm_ch;
	struct work_struct diag_read_mdm_work;
	struct workqueue_struct *diag_sdio_wq;
	struct work_struct diag_read_sdio_work;
	struct diag_request *usb_read_mdm_ptr;
	struct diag_request *write_ptr_mdm;
#endif
};

extern struct diagchar_dev *driver;
#endif
