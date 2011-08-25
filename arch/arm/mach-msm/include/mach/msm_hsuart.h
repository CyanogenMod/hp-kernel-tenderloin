/*
 * include/asm/arch-msm/msm_hsuart.h
 *
 * Copyright (C) 2008 Palm, Inc.
 * Author: Amir Frenkel <amir.frenkel@palm.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __MSM_HSUART_H__
#define __MSM_HSUART_H__

struct	buffer_item {
	/*
	 * linked list member so we can chain the buffer into a linked list.
	 */
	struct list_head	list_item;

	/* In bytes */
	size_t			size;

	/*
	 * The physical start addr of the DMA buffer.
	 */
	dma_addr_t		phys_addr;

	/* 
	 * Pointer to the virtual address that correspond to the phys mem 
	 * allocated for the DMA 
	 */
	char*			p_vaddr;

	int			read_index;
	int			write_index;
	int			fullness;

};

struct rxtx_lists {
	/*
	 * Currently, all the lists use the same lock
	 * as a future enhancement, we may consider having lock per 
	 * list
	 */
	spinlock_t			lock;

	/*
	 * List of empty buffers, can be filled with data.
	 */
	struct list_head		empty;

	/*
	 * List of full buffers, contains data which was not
	 * consumed yet.
	 */
	struct list_head		full;

	/*
	 * List of buffer that are currently used by the underlying
	 * platform - uart driver to rx or tx data.
	 */
	struct list_head		used;

	/*
	 * pool of buffers to be used in empty and full list, 
	 * these buffers point to the underlying memory in which Rx
	 * will be read into or Tx will be writing from.
	 */
	struct	buffer_item*		p_buffer_pool;

	/*
	 * The number of vacant buffers is the list.
	 */
	int				vacant_buffers;

	/*
	 * The total number of buffers.
	 */	
	int				buffer_cnt;
};



/*
 * The following flags indicates features/configuration 
 * option for the HSUART context
 */
#define HSUART_CFG_RX_PIO		       (1 << 0)
#define HSUART_CFG_TX_PIO		       (1 << 1)
#define HSUART_CFG_RX_DM		       (1 << 2)
#define HSUART_CFG_TX_DM		       (1 << 3)
#define HSUART_CFG_SCHED_RT		       (1 << 4)

/*
 * Generic configuration parameters.
 */
struct hsuart_config {
	/*
	 * The uart port flags. See definition above
	 */
	u32			flags;

	/*
	 * The uart port number to read/write from/to.
	 */
	int			port_id;

	/*
	 * The uart port max packet size.
	 */
	int   max_packet_size; 

	/*
	 * The uart port min packet size
	 */
	int   min_packet_size; 

	/*
	 * The uart port tx latency in bytes at current speed 
	 */
	int   rx_latency;     

	/*
	 * The uart callback for pin muxing 
	 */
	int (*p_board_pin_mux_cb) ( int on );
	int (*p_board_gsbi_config_cb) ( void );

	int (*p_board_rts_pin_deassert_cb) ( int deassert );
	/*
	 * Callback Zone...
	 */
	struct {
		struct buffer_item* (*p_cbk)(void* p_data, int free_bytes);
		void*	p_data;
	} rx_get_buffer;

	struct {
		void (*p_cbk) (void* p_data, struct buffer_item* p_buffer);
		void*	p_data;
	} rx_put_buffer;

	struct {
		struct buffer_item* (*p_cbk)(void* p_data);
		void*	p_data;
	} tx_get_buffer;

	struct {
		void (*p_cbk) (void* p_data, struct buffer_item* p_buffer, int transaction_size);
		void*	p_data;
	} tx_put_buffer;
};

int
msm_hsuart_write(int context_id_handle);

int
msm_hsuart_read(int context_id_handle, struct buffer_item* io_p_buffer);

int
msm_hsuart_open_context(struct hsuart_config* io_p_cfg, 
			int* o_p_context_id_handle);

int
msm_hsuart_close_context(int context_id_handle);

int
msm_hsuart_suspend(int context_id_handle);

int
msm_hsuart_resume(int context_id_handle);

int 
msm_hsuart_register_rx_put_buffer(
		int context_id_handle, 
		void (*p_cbk)(void* p_data, struct buffer_item* p_buffer),
		void* p_data);
int 
msm_hsuart_register_rx_get_buffer(
		int context_id_handle, 
		struct buffer_item* (*p_cbk)(void* p_data, int free_bytes),
		void* p_data);

int 
msm_hsuart_register_tx_put_buffer(
		int context_id_handle, 
		void (*p_cbk)(void* p_data, struct buffer_item* p_buffer, int transaction_size),
		void* p_data);
int 
msm_hsuart_register_tx_get_buffer(
		int context_id_handle, 
		struct buffer_item* (*p_cbk)(void* p_data),
		void* p_data);
/*
 * The possible values for 'flow' parameter are defined in hsuart.h
 */
int 
msm_hsuart_set_flow(
		int context_id_handle, 
		int flow);
int 
msm_hsuart_set_parity(
		int context_id_handle, 
		int parity);
int 
msm_hsuart_set_baud_rate(int context_id_handle, 
			uint32_t baud_rate);

int 
msm_hsuart_rx_fifo_has_bytes( int context_id_handle);

#endif	/* __MSM_HSUART_H__ */
