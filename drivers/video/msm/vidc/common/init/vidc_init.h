/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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

#ifndef VIDC_INIT_H
#define VIDC_INIT_H

#include "vidc_type.h"

#define VIDC_MAX_NUM_CLIENTS 4
#define MAX_VIDEO_NUM_OF_BUFF 100

enum buffer_dir {
	BUFFER_TYPE_INPUT,
	BUFFER_TYPE_OUTPUT
};

struct buf_addr_table {
	unsigned long user_vaddr;
	unsigned long kernel_vaddr;
	unsigned long phy_addr;
	int pmem_fd;
	struct file *file;
};

struct video_client_ctx {
	void *vcd_handle;
	u32 num_of_input_buffers;
	u32 num_of_output_buffers;
	struct buf_addr_table input_buf_addr_table[MAX_VIDEO_NUM_OF_BUFF];
	struct buf_addr_table output_buf_addr_table[MAX_VIDEO_NUM_OF_BUFF];
	struct list_head msg_queue;
	struct mutex msg_queue_lock;
	wait_queue_head_t msg_wait;
	struct completion event;
	u32 event_status;
	u32 seq_header_set;
	u32 stop_msg;
	u32 stop_called;
	u32 stop_sync_cb;
};

void __iomem *vidc_get_ioaddr(void);
int vidc_load_firmware(void);
void vidc_release_firmware(void);
u32 vidc_lookup_addr_table(struct video_client_ctx *client_ctx,
	enum buffer_dir buffer, u32 search_with_user_vaddr,
	unsigned long *user_vaddr, unsigned long *kernel_vaddr,
	unsigned long *phy_addr, int *pmem_fd, struct file **file,
	s32 *buffer_index);
u32 vidc_insert_addr_table(struct video_client_ctx *client_ctx,
	enum buffer_dir buffer, unsigned long user_vaddr,
	unsigned long *kernel_vaddr, int pmem_fd,
	unsigned long buffer_addr_offset,
	unsigned int max_num_buffers);
u32 vidc_delete_addr_table(struct video_client_ctx *client_ctx,
	enum buffer_dir buffer, unsigned long user_vaddr,
	unsigned long *kernel_vaddr);

u32 vidc_timer_create(void (*timer_handler)(void *),
	void *user_data, void **timer_handle);
void  vidc_timer_release(void *timer_handle);
void  vidc_timer_start(void *timer_handle, u32 time_out);
void  vidc_timer_stop(void *timer_handle);


#endif
