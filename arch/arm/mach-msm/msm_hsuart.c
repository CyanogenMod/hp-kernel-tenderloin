/*
 * arch/arm/mach-msm/hsuart.c - Driver for data mover
 * Data mover can be either DMA client or pio workerthread 
 * that emulates the DMA client.
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

#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <mach/dma.h>

#include <linux/sched.h>
#include <linux/hsuart.h>
#include <mach/msm_hsuart.h>
#include <mach/msm_uart_dm.h>

/*
 *	Debug related macros and defs
 */
#define  DRIVER_NAME      			"msm_hsuart"
#define  DRIVER_VERSION   			 (0x100)


/*
 * Debug ctl for the module
 */
#define MSM_HSUART_FEATURE_PRINT_RX_DATA 0
#define MSM_HSUART_FEATURE_PRINT_TX_DATA 0

#define MSM_HSUART_DEBUG_ENABLE		0
#define MSM_HSUART_FUNC_LOG_ENABLE	0

#if MSM_HSUART_DEBUG_ENABLE
#define MSM_HSUART_DEBUG(args...)	(printk(KERN_DEBUG args))
#define MSM_HSUART_INFO(args...)	(printk(KERN_INFO args))
#define MSM_HSUART_ERR(args...)		(printk(KERN_ERR args))
#else
#define MSM_HSUART_INFO(args...)
#define MSM_HSUART_DEBUG(args...)
#define MSM_HSUART_ERR(args...)
#endif // MSM_HSUART_DEBUG_ENABLE					

#if MSM_HSUART_FUNC_LOG_ENABLE
#define	MSM_HSUART_ENTER()		(printk(KERN_INFO"%s: %s, %u [msec] enter\n",\
					DRIVER_NAME, __PRETTY_FUNCTION__, jiffies_to_msecs(jiffies)))
#if 0
					i(printk(KERN_INFO"%s: %s, enter\n",\
					DRIVER_NAME, __PRETTY_FUNCTION__))
#endif
#define	MSM_HSUART_EXIT()		(printk(KERN_INFO"%s: %s, %u [msec] exit\n",\
					DRIVER_NAME, __PRETTY_FUNCTION__, jiffies_to_msecs(jiffies)))
#if 0
					(printk(KERN_INFO"%s: %s, exit\n",\
					DRIVER_NAME, __PRETTY_FUNCTION__))
#endif
#else
#define MSM_HSUART_ENTER()
#define MSM_HSUART_EXIT()
#endif

/*
 * definition of data-types and data-structures for hsuart...
 */

#define RX_MODE_PIO(p_context)                  (p_context->flags & HSUART_CFG_RX_PIO )
#define RX_MODE_DM(p_context)                   (p_context->flags & HSUART_CFG_RX_DM )
#define RX_MODE_DM_FIXED_PACKET_LEN(p_context) ((p_context->flags & HSUART_CFG_RX_DM ) \
												&& (p_context->rx.max_packet_size == p_context->rx.min_packet_size))

#define TX_MODE_PIO(p_context)                  (p_context->flags & HSUART_CFG_TX_PIO )
#define TX_MODE_DM(p_context)                   (p_context->flags & HSUART_CFG_TX_DM )

#define MSM_UART_DEBUG_TIMING      0 
#define MSM_UART_DEBUG_TIMING_PORT 1

#if MSM_UART_DEBUG_TIMING
#include <linux/hres_counter.h>
static char* dbg_strings[] = { "msm uart get buff enter",
                               "msm uart buff exit",
                               "msm uart buff enter",
                               "msm uart buff exit",

                               "msm uart write enter",
                               "msm uart write exit",
                               "msm uart read  enter",
                               "msm uart read  exit",

                               "msm uart pio stale enter",
                               "msm uart pio stale exit",

                               "msm uart pio rx level enter",
                               "msm uart pio rx level exit",

                               "msm uart dm stale enter",
                               "msm uart dm stale exit",

                               "msm uart dm cb enter",
                               "msm uart dm cb exit",

                               "msm uart dm wq enter",
                               "msm uart dm wq exit",

                               "msm uart open context enter",
                               "msm uart open context exit",

                               "msm uart close context enter",
                               "msm uart close context exit",

                               "msm uart dm rx config enter",
                               "msm uart dm rx config exit",
                               "msm uart set baud rate enter",
                               "msm uart set baud rate exit",

                               "msm uart write worker enter",
                               "msm uart write worker exit",

                               "msm uart suspend enter",
                               "msm uart suspend exit",

                               "msm uart resume enter",
                               "msm uart resume exit",

                               "msm uart tx pio event",
			     };

#define MSM_UART_GB_ENT    	    0	
#define MSM_UART_GB_EXT    	    1	
#define MSM_UART_PB_ENT    	    2	
#define MSM_UART_PB_EXT    	    3	
#define MSM_UART_WRITE_ENT 	    4	
#define MSM_UART_WRITE_EXT 	    5	
#define MSM_UART_READ_ENT           6	
#define MSM_UART_READ_EXT           7
#define MSM_UART_PIO_STALE_ENT      8
#define MSM_UART_PIO_STALE_EXT      9
#define MSM_UART_PIO_RX_LEVEL_ENT  10
#define MSM_UART_PIO_RX_LEVEL_EXT  11
#define MSM_UART_DM_STALE_ENT      12
#define MSM_UART_DM_STALE_EXT      13
#define MSM_UART_DM_CB_ENT         14
#define MSM_UART_DM_CB_EXT         15
#define MSM_UART_DM_WQ_ENT         16
#define MSM_UART_DM_WQ_EXT         17
#define MSM_UART_OPEN_CONTEXT_ENT  18
#define MSM_UART_OPEN_CONTEXT_EXT  19
#define MSM_UART_CLOSE_CONTEXT_ENT 20
#define MSM_UART_CLOSE_CONTEXT_EXT 21
#define MSM_UART_DM_CONFIG_RX_ENT  22
#define MSM_UART_DM_CONFIG_RX_EXT  23
#define MSM_UART_SET_BAUD_RATE_ENT 24
#define MSM_UART_SET_BAUD_RATE_EXT 25
#define MSM_UART_WRITE_WORKER_ENT  26
#define MSM_UART_WRITE_WORKER_EXT  27
#define MSM_UART_SUSPEND_ENT       28	
#define MSM_UART_SUSPEND_EXT       29
#define MSM_UART_RESUME_ENT        30
#define MSM_UART_RESUME_EXT        31
#define MSM_UART_TX_PIO_EVENT      32


#define MSM_HSUART_LOG(p_context, eventid, arg1, arg2 )                    \
         if ( p_context->p_uart_port->id == MSM_UART_DEBUG_TIMING_PORT ) { \
	 	hres_event(dbg_strings[eventid], (u32) arg1, (u32) arg2 ); \
         }                                                                 \

#else
	#define MSM_HSUART_LOG(args...)
#endif
/*
 * TODO:MSM_UARTDM, right now each context can support read and write, consider changing it and have context per action to simplify the code
 */
struct hsuart_worker {
	/* 
	 * The work-queue for this context
	 */
	struct workqueue_struct*	p_worker;
	struct work_struct		worker;
	char				name[20];

	/*
	 * Indicate the completion of single xfer.
	 */
	struct completion		xfer_done;
};

/*
* RX transaction state
* Idle           - No DM transaction configured
* DM in progress - DM request has been submitted
* DM flushing    - flushing DM request
*/
#define MSM_HSUART_RX_STATE_IDLE               0
#define MSM_HSUART_RX_STATE_DM_IN_PROGRESS     1
#define MSM_HSUART_RX_STATE_DM_FLUSHING        2

/*
* TX transaction state
* Idle            - No transaction configured
* In progress     - executing transfer
* Wait completion - all the data was submitted to HW FIFO , now waiting for the FIFO to be emptied
*/
#define MSM_HSUART_TX_STATE_IDLE            0
#define MSM_HSUART_TX_STATE_IN_PROGRESS     1
#define MSM_HSUART_TX_STATE_WAIT_COMPLETION 2

/*
* Hsuart context state
* Active         - Active state
* Suspending     - suspend ongoing
* Suspended      - suspend complete
*/
#define MSM_HSUART_STATE_ACTIVE          0
#define MSM_HSUART_STATE_SUSPENDING      1
#define MSM_HSUART_STATE_SUSPENDED       2

#define MSM_HSUART_FLOW_DIR_RX 0
#define MSM_HSUART_FLOW_DIR_TX 1

#define MSM_HSUART_FLOW_STATE_ASSERT   0
#define MSM_HSUART_FLOW_STATE_DEASSERT 1

struct hsuart_context {
	/*
	 * The unique ID of the context
	 */
	int		context_id;

	int flags;

	/*
	 * Lock to protect the context
	 */
	spinlock_t	lock;
	/*
	 * Work related stuff
	 */
	struct hsuart_worker reader;
	struct hsuart_worker writer;

	/*
	 * Rx/Tx related buffer management lists.
	 */
	struct rxtx_lists		lists;

	/*
	 * TODO:MSM_HSUART Since I am lazy, added p_buffer to point at the only buffer
	 * we now hold when reading....should move it to the above member which support list of buffers....
	 * When switching to use the rxtx_lists, should also revise the locking to lock the lists when
	 * accessing list members, whereas right now I am locking the entire context.
	 */
	struct buffer_item*		p_rx_buffer;
	struct buffer_item*		p_tx_buffer;
	/*
	 * Transaction complete lock
	 */
	struct mutex read_complete_lock;
	struct mutex write_complete_lock;

	/*
	 * Pointer to uart-port device handle, this port represent the physical 
	 * uart port that is used by the lower hs-uart driver (platform-specific)
	 */
	struct generic_uart_port* p_uart_port;

	struct {
		/*
		 * Indicates the number of valid bytes that should be recieved
		 * in the current rx transaction.
		 */
		int valid_byte_cnt;
		/*
		 * If set a new request to the HW must be issued
		 * to request the next chunk of data.
		 */
		int the_end;

		/*
		 * Indicate the number of bytes that were read
		 * during the current transaction.
		 */
		int read_byte_cnt;

		/*
		 * Max packet size
		 */
		int   max_packet_size;  

		/*
		 * Min packet size
		 */
		int   min_packet_size; 

		/*
		 * Latency in bytes at current speed
		 */
		int   latency;         

		/*
		 * 
		 */
		bool enable_stale;

		/*
		 * transfer state
		 */
		int state;

		/*
		 *  rx transaction complete flag
		 */
		wait_queue_head_t transaction_complete;
		/*
		 * rx flow control
		 */
		int flow_ctl;
		/*
		 * rx flow state
		 */
		int flow_state;
	} rx;

	struct {
		/*
		 * transaction size in bytes
		 */
		int transaction_size;

		/*
		 * transfer state
		 */
		int state;

		/*
		 *  tx transaction complete flag
		 */
		wait_queue_head_t transaction_complete;
		/*
		 * tx flow control
		 */
		int flow_ctl;
	} tx;

	/*
	 * Callback Zone...
	 */
	int (*p_board_pin_mux_cb) ( int on );
	int (*p_board_rts_pin_deassert_cb) ( int deassert );

	struct {
		struct buffer_item* (*p_cbk)(void* p_data, int free_bytes);
		void*	p_data;
	} rx_get_buffer;
	struct {
		void (*p_cbk)(void* p_data, struct buffer_item* p_buffer);
		void*	p_data;
	} rx_put_buffer;
	struct {
		struct buffer_item* (*p_cbk)(void* p_data);
		void*	p_data;
	} tx_get_buffer;
	struct {
		void (*p_cbk)(void* p_data, struct buffer_item* p_buffer, int transaction_size);
		void*	p_data;
	} tx_put_buffer;
	/*
	 * Misc parameters of the UART context.
	 */
	msm_uartdm_parity_t  parity;
	int	             baud_rate;

	/*
	 * Context state
	 */
	int state;
};

static int context_cnt = 0;

/*
 * The Rx callback function. The function is in fact doing 
 * the read from the msm uartdm HW FIFO into the local buffer of the hsuart
 * driver 
 * 
 * @param[in][out]	- p_data - pointer to the hsuart context to work on.
 *
 * return		- None
 *
 */

static int irq_fire_cnt = 0;
void __msm_hsuart_rx_level_cbk(void* p_data)
{
	struct hsuart_context*	p_context;
	unsigned long		flags;

	MSM_HSUART_ENTER();	
	if (NULL == p_data) {
		MSM_HSUART_ERR("%s: %s, invalid(null) contxt\n",
			  DRIVER_NAME,
			  __PRETTY_FUNCTION__);
		return;
	}
	p_context = (struct hsuart_context*)p_data;

	MSM_HSUART_LOG(p_context, MSM_UART_PIO_RX_LEVEL_ENT, 0 , 0 ); 

	if (p_context->p_rx_buffer) {
		spin_lock_irqsave(&(p_context->lock), flags);
		if (p_context->rx.the_end) {
			spin_unlock_irqrestore(&(p_context->lock), flags);
			printk("RX LEVEL IRQ after STALE\n");
		}
		else {
			p_context->rx.valid_byte_cnt += (4*(msm_uartdm_get_rx_fifo_fullness(p_context->p_uart_port, NULL) - 1));

			MSM_HSUART_DEBUG("%s valid_cnt %d\n",__FUNCTION__, p_context->rx.valid_byte_cnt);

			irq_fire_cnt++;
			msm_uartdm_disable_rx_irqs(p_context->p_uart_port);
			spin_unlock_irqrestore(&(p_context->lock), flags);
			queue_work(p_context->reader.p_worker, 
				   &(p_context->reader.worker));
		}
	}
	else {
		MSM_HSUART_ERR("%s: %s, no buffer available for this context!\n",
			DRIVER_NAME,
			__PRETTY_FUNCTION__);
	}

	MSM_HSUART_LOG(p_context, MSM_UART_PIO_RX_LEVEL_EXT, p_context->rx.valid_byte_cnt , 0 ); 

	MSM_HSUART_EXIT();
}

/*
 * The Rx pio stale callback function. We read the number of valid bytes 
 * read by the HW and update the context to hold this information,
 * then call the standard rx callback to read bytes from the FIFO.
 * 
 * @param[in][out]	- p_data - pointer to the hsuart context to work on.
 *
 * return		- None
 *
 */
void __msm_hsuart_rx_pio_stale_cbk(void* p_data)
{
	struct hsuart_context*	p_context;
	unsigned long		flags;

	MSM_HSUART_ENTER();

	if (NULL == p_data) {
		MSM_HSUART_ERR("%s: %s, invalid(null) contxt\n",
			  DRIVER_NAME,
			  __PRETTY_FUNCTION__);
		return;
	}
	p_context = (struct hsuart_context*)p_data;

	MSM_HSUART_LOG(p_context, MSM_UART_PIO_STALE_ENT, 0 , 0 ); 

	if (p_context->p_rx_buffer) {
		spin_lock_irqsave(&(p_context->lock), flags);
		irq_fire_cnt++;

		p_context->rx.valid_byte_cnt = 
			msm_uartdm_get_received_byte_cnt(p_context->p_uart_port);

		p_context->rx.the_end = 1;
		MSM_HSUART_DEBUG("%s valid_cnt %d\n",__FUNCTION__, p_context->rx.valid_byte_cnt);
		msm_uartdm_disable_rx_irqs(p_context->p_uart_port);
		spin_unlock_irqrestore(&(p_context->lock), flags);
		queue_work(p_context->reader.p_worker, 
				   &(p_context->reader.worker));
	}
	MSM_HSUART_EXIT();
}

/*
 * The Rx dm stale callback function. We read the number of valid bytes 
 * read by the HW and invoke DMA flush function if the number of bytes read 
 * greater than 0. DMA flush function handles invokes call the standard dma 
 * rx callback .
 * 
 * @param[in][out]	- p_data - pointer to the hsuart context to work on.
 *
 * return		- None
 *
 */
void __msm_hsuart_rx_dm_stale_cbk(void* p_data)
{
	struct hsuart_context*	p_context;
	unsigned long		flags;

	MSM_HSUART_ENTER();

	if (NULL == p_data) {
		MSM_HSUART_ERR("%s: %s, invalid(null) contxt\n",
			  DRIVER_NAME,
			  __PRETTY_FUNCTION__);
		return;
	}
	p_context = (struct hsuart_context*)p_data;

	MSM_HSUART_LOG(p_context, MSM_UART_DM_STALE_ENT, 0 , 0 ); 

	if (p_context->p_rx_buffer) {
		int byte_cnt;

		spin_lock_irqsave(&(p_context->lock), flags);
		irq_fire_cnt++;
		
		byte_cnt = msm_uartdm_get_received_byte_cnt(p_context->p_uart_port);

		if ( byte_cnt > 0 ) {

			msm_uartdm_disable_rx_irqs(p_context->p_uart_port);

			msm_uartdm_rx_dm_flush(p_context->p_uart_port);
				
		} else {
			printk("RX DM empty STALE\n");
		
		}

		spin_unlock_irqrestore(&(p_context->lock), flags);
		MSM_HSUART_LOG(p_context, MSM_UART_DM_STALE_EXT, byte_cnt , 0 ); 

	}
	MSM_HSUART_EXIT();
}

/*
 * This routine is called when we are done with a DMA transfer or the
 * a flush has been sent to the data mover driver.
 *
 * This routine is registered with Data mover when we set up a Data Mover
 *  transfer. It is called from Data mover ISR when the DMA transfer is done.
 */
static void __msm_hsuart_rx_dm_cbk(void* p_data)
{
	struct hsuart_context *p_context;

	MSM_HSUART_ENTER();	

	if (NULL == p_data) {
		MSM_HSUART_ERR("%s: %s, invalid(null) contxt\n",
			  DRIVER_NAME,
			  __PRETTY_FUNCTION__);
		return;
	}


	p_context = (struct hsuart_context*)p_data;

	MSM_HSUART_LOG(p_context, MSM_UART_DM_CB_ENT, 0 , 0 ); 

	msm_uartdm_disable_rx_irqs(p_context->p_uart_port);

	if (p_context->p_rx_buffer) {
		irq_fire_cnt++;
		p_context->rx.valid_byte_cnt = 
			msm_uartdm_get_received_byte_cnt(p_context->p_uart_port);

		
        	MSM_HSUART_DEBUG("%s valid_cnt %d\n",__FUNCTION__, p_context->rx.valid_byte_cnt);
		MSM_HSUART_LOG(p_context, MSM_UART_DM_CB_EXT, p_context->rx.valid_byte_cnt , 0 ); 
		queue_work(p_context->reader.p_worker, 
				   &(p_context->reader.worker));
	}

	MSM_HSUART_EXIT();
}

/*
 * The Tx RDY callback function. This function is called to indicate that the TX FIFO is empty
 * 
 * @param[in][out]	- p_data - pointer to the hsuart context to work on.
 *
 * return		- None
 *
 */
void __msm_hsuart_tx_rdy_cbk(void* p_data)
{
	struct hsuart_context*	p_context;

	MSM_HSUART_ENTER();

	if (NULL == p_data) {
		MSM_HSUART_ERR("%s: %s, invalid(null) contxt\n",
			  DRIVER_NAME,
			  __PRETTY_FUNCTION__);
		return;
	}
	p_context = (struct hsuart_context*)p_data;
	if (NULL != p_context) {
		msm_uartdm_disable_tx_rdy(p_context->p_uart_port);
		if ( p_context->state == MSM_HSUART_STATE_SUSPENDING ) {
			wake_up(&p_context->rx.transaction_complete);
		}

		p_context->tx.state = MSM_HSUART_TX_STATE_IDLE;
		queue_work(p_context->writer.p_worker, 
			   &(p_context->writer.worker));
	}
	MSM_HSUART_EXIT();
}

/*
 * The Tx level callback function. This function is called to indicate that the TX FIFO is 
 * below the configured threshold.
 * 
 * @param[in][out]	- p_data - pointer to the hsuart context to work on.
 *
 * return		- None
 *
 */
void __msm_hsuart_tx_level_cbk(void* p_data)
{
	struct hsuart_context*	p_context;

	MSM_HSUART_ENTER();

	if (NULL == p_data) {
		MSM_HSUART_ERR("%s: %s, invalid(null) contxt\n",
			  DRIVER_NAME,
			  __PRETTY_FUNCTION__);
		return;
	}
	p_context = (struct hsuart_context*)p_data;
	if (NULL != p_context) {
		msm_uartdm_disable_tx_level(p_context->p_uart_port);
		queue_work(p_context->writer.p_worker, 
			   &(p_context->writer.worker));
	}
	MSM_HSUART_EXIT();
}
/*
 * ============= Worker threads ===============
 */

/**
*
* Worker thread, responsible to write to the UART port indicated by the context
*
* @param[in][out]       io_p_work - Pointer to the workqueue.
*
* @return None.
*
*/
static void
__msm_hsuart_write_worker(struct work_struct* io_p_work)
{
	struct hsuart_worker*	p_wr_worker; 
	struct hsuart_context*	p_context;
	struct buffer_item*	p_buffer = NULL;
	unsigned long		flags;
	unsigned int		data;
	int			ready = 0;
	int			start_xfer = 0;

	int tx_state ;

	MSM_HSUART_ENTER();

	p_wr_worker = container_of(io_p_work, struct hsuart_worker, worker);

	p_context = container_of(p_wr_worker, struct hsuart_context, writer);


	if ( p_context ) {
	

		spin_lock_irqsave(&(p_context->lock), flags);

		MSM_HSUART_LOG(p_context, MSM_UART_TX_PIO_EVENT, 1 , p_context->tx.state ); 
		tx_state = p_context->tx.state;
		if ( !p_context->p_tx_buffer && ( p_context->state == MSM_HSUART_STATE_ACTIVE )) {
			p_context->p_tx_buffer = p_context->tx_get_buffer.p_cbk( p_context->tx_get_buffer.p_data );
		}

		p_buffer = p_context->p_tx_buffer;

		if ( !p_buffer || tx_state == MSM_HSUART_TX_STATE_WAIT_COMPLETION ) {
			MSM_HSUART_LOG(p_context, MSM_UART_TX_PIO_EVENT, 2 , p_buffer ); 
			spin_unlock_irqrestore(&(p_context->lock), flags);
			goto exit;
		}

		if ( tx_state == MSM_HSUART_TX_STATE_IDLE ) {
			MSM_HSUART_LOG(p_context, MSM_UART_TX_PIO_EVENT, 3 , p_buffer->fullness );
			start_xfer = 1;
			p_context->tx.transaction_size = p_context->p_tx_buffer->fullness;
			p_context->tx.state = tx_state = MSM_HSUART_TX_STATE_IN_PROGRESS;
		}	

		spin_unlock_irqrestore(&(p_context->lock), flags);

		MSM_HSUART_LOG(p_context, MSM_UART_WRITE_WORKER_ENT, p_buffer->fullness , start_xfer ); 
		
		if (start_xfer) {

			msm_uartdm_config_write_size(p_context->p_uart_port,
						p_buffer->fullness);
		}

		{
			//printk("%s,%d\n",__FUNCTION__, p_buffer->fullness);
			for (;p_buffer->fullness > 3;) {
	
				ready = msm_uartdm_tx_ready(p_context->p_uart_port);
				if (!ready) {
	//				printk(KERN_ERR"\nzz%d,%d\n",__LINE__,p_buffer->fullness);
					goto done_write_chunk;
				}
				data = *(int *)(&p_buffer->p_vaddr[p_buffer->read_index]);
	#if MSM_HSUART_FEATURE_PRINT_TX_DATA 
				printk(KERN_ERR"<<<<data is 0x%x>>>>\n", data);
	#endif // MSM_HSUART_FEATURE_PRINT_TX_DATA
				msm_uartdm_send_dword(p_context->p_uart_port, data);
				p_buffer->fullness -= 4;
				p_buffer->read_index += 4;
				if (p_buffer->read_index >= p_buffer->size) {
					p_buffer->read_index -= p_buffer->size;
				}
			}
			/*
			* Handle the last bytes which are less than a dword (4 byte)
			* - Make sure to reset the buffer bookkeeping when done.
			*/
			if (p_buffer->fullness) {
				ready = msm_uartdm_tx_ready(p_context->p_uart_port);
				if (!ready) {
					//printk(KERN_ERR"\nzz%d\n",__LINE__);
					goto done_write_chunk;
				}
	
				data = *(int *)(&p_buffer->p_vaddr[p_buffer->read_index]);
	#if MSM_HSUART_FEATURE_PRINT_TX_DATA
			{
				printk(KERN_ERR"fullness %d\n",p_buffer->fullness);
				printk(KERN_ERR"============\n");
				int byte_cnt = p_buffer->fullness;
			//		for (;byte_cnt;byte_cnt--){
			//			int shift = 8 * (byte_cnt-1);
			//			printk(KERN_ERR"data: is %c\n", (data >> shift) & 0xFF);
			//		}
				printk(KERN_ERR"============\n");
				printk(KERN_ERR"<<<<data is 0x%x>>>>\n", data);	
			}
	#endif //MSM_HSUART_FEATURE_PRINT_TX_DATA
				
				msm_uartdm_send_dword(p_context->p_uart_port, data);
			}

			spin_lock_irqsave(&(p_context->lock), flags);

			p_buffer->fullness      = 0;
			p_buffer->read_index    = 0;
			p_buffer->write_index   = 0;
			p_context->tx_put_buffer.p_cbk(
					p_context->tx_put_buffer.p_data,
					p_buffer,
					p_context->tx.transaction_size);		
			p_context->tx.transaction_size = 0;

			p_context->p_tx_buffer =  NULL;

			p_context->tx.state = MSM_HSUART_TX_STATE_WAIT_COMPLETION;
			msm_uartdm_enable_tx_rdy(p_context->p_uart_port);

			spin_unlock_irqrestore(&(p_context->lock), flags);
		

		}

		goto exit;
	}
	else {
		MSM_HSUART_ERR("%s:%s, error, no buffer associated with the context\n", 
			DRIVER_NAME,
			__FUNCTION__);
	}

done_write_chunk:
	msm_uartdm_enable_tx_level(p_context->p_uart_port);


exit:
	MSM_HSUART_LOG(p_context, MSM_UART_WRITE_WORKER_EXT, 0 , 0 ); 
	MSM_HSUART_EXIT();
}
/**
*
* Private function to configure the flow control of the UART port.
*
* @param[in][out]	io_p_context - Pointer to the uart context 
*					to operate on.
* @param[in]		flow_dir - RX or TX
* 
* @param[in]		flow_ctl - flow control , SW or HW
* 
* @param[in]		flow_state - assert or de-assert flow ctl line 
*                        Appplicable for rx flow line in SW ctl mode only
* 
* @return 0 for success -1 otherwise.
*
* @Note		The function assume that any locking of the context DB
*		will be done by the calling layer.
*/
static void __msm_hsuart_set_flow(struct hsuart_context* io_p_context, int flow_dir, int flow_ctl, int flow_state)
{
	if ( flow_dir == MSM_HSUART_FLOW_DIR_RX ) {
		if (HSUART_MODE_FLOW_CTRL_HW == flow_ctl) {
			msm_uartdm_set_rx_flow(io_p_context->p_uart_port, 1 , 0);
		}
		else {
			int set_flow_state; 

			if ( HSUART_MODE_FLOW_CTRL_NONE == flow_ctl ) {
				flow_state = MSM_HSUART_FLOW_STATE_DEASSERT;
			}

			set_flow_state = ( flow_state == MSM_HSUART_FLOW_STATE_ASSERT ) ? 1 : 0;

			msm_uartdm_set_rx_flow(io_p_context->p_uart_port, 0, set_flow_state);
		}
		io_p_context->rx.flow_ctl   = flow_ctl;
		io_p_context->rx.flow_state = flow_state;
	}

	if ( flow_dir == MSM_HSUART_FLOW_DIR_TX ) {
		if (HSUART_MODE_FLOW_CTRL_HW == flow_ctl) {
			msm_uartdm_set_tx_flow(io_p_context->p_uart_port, 1);
		}
		else {
			msm_uartdm_set_tx_flow(io_p_context->p_uart_port, 0);
		}
		io_p_context->tx.flow_ctl   = flow_ctl;
	}
}

int
msm_hsuart_config_dm_rx(struct hsuart_context* p_context,struct buffer_item* io_p_buffer , bool enable_stale)
{
	size_t read_size    = p_context->rx.max_packet_size;

	MSM_HSUART_LOG(p_context, MSM_UART_DM_CONFIG_RX_ENT, 0 , 0 ); 
	
	if ( io_p_buffer ) {
		p_context->p_rx_buffer = io_p_buffer;
	}

	p_context->rx.state = MSM_HSUART_RX_STATE_DM_IN_PROGRESS;
	msm_uartdm_rx_dm_config( p_context->p_uart_port, 
                                 (p_context->p_rx_buffer->phys_addr + p_context->p_rx_buffer->write_index),
                                 read_size);

	msm_uartdm_enable_rx_irqs(p_context->p_uart_port, enable_stale);
	MSM_HSUART_LOG(p_context, MSM_UART_DM_CONFIG_RX_EXT, 0 , 0 ); 

	return read_size;
}

/**
*
* Rx DM flush function - determines whether DM flush is required and flushes DM ( if needed )
*
* @param[in][out]       p_context - Pointer to context
*
* @return true   - DM flush was required and executed
*         false  - DM flush was not required 
*/
bool
msm_hsuart_flush_dm(struct hsuart_context* p_context)
{
	unsigned long flags;
	bool flushing;
	 
	spin_lock_irqsave(&(p_context->lock), flags);

	flushing = ( p_context->rx.state == MSM_HSUART_RX_STATE_DM_IN_PROGRESS );

	if ( flushing ) {
		p_context->rx.state = MSM_HSUART_RX_STATE_DM_FLUSHING;
		msm_uartdm_disable_rx_irqs(p_context->p_uart_port);
		msm_uartdm_rx_dm_flush(p_context->p_uart_port);
	}

	spin_unlock_irqrestore(&(p_context->lock), flags);

	if ( flushing ) {
		wait_event_interruptible(p_context->rx.transaction_complete, p_context->rx.state == MSM_HSUART_RX_STATE_IDLE );
	}

	return flushing;
}

/**
*
* Rx DM resume function - resumes RX DM transactions ( called after DM Flush was executed )
*
* @param[in][out]       p_context - Pointer to context
*
* @return None.
*
*/
void
msm_hsuart_resume_dm(struct hsuart_context* p_context)
{
	msm_hsuart_config_dm_rx(p_context, NULL, p_context->rx.enable_stale); 
}

/**
*
* Worker thread, responsible to read from the UART in PIO mode
*
* @param[in][out]       io_p_work - Pointer to the workqueue.
*
* @return None.
*
*/
static void
__msm_hsuart_read_pio_worker(struct work_struct* io_p_work)
{
	struct hsuart_worker*	p_rd_worker;
	struct hsuart_context*	p_context;
	struct buffer_item*	p_buffer = NULL;
	unsigned long		flags;
	int			write_index;
	int			fullness;
	int			size;
	unsigned int*	p_word;
	/*
	 * born ready!
	 */
	int			ready		= 1;

	MSM_HSUART_ENTER();
	p_rd_worker = container_of(io_p_work, struct hsuart_worker, worker);

	p_context = container_of(p_rd_worker, struct hsuart_context, reader);

	if (p_context && (p_context->p_rx_buffer)) {
		spin_lock_irqsave(&(p_context->lock), flags);
		MSM_HSUART_DEBUG("%s, %d, irq_cnt %d, valid_bytes %d read_byte_cnt %d\n",__FUNCTION__, __LINE__,irq_fire_cnt, p_context->rx.valid_byte_cnt, p_context->rx.read_byte_cnt);
		p_buffer	= p_context->p_rx_buffer;
		write_index	= p_buffer->write_index;
		fullness	= p_buffer->fullness;
		size		= p_buffer->size;

		for (;0 < (p_context->rx.valid_byte_cnt - p_context->rx.read_byte_cnt);) {
			ready = msm_uartdm_rx_ready(p_context->p_uart_port);
			if (!ready) {
				printk("\n\n*********!rdy**************\n\n");
				break;
			}
			while (size - fullness < 4) {
				/*
				 * First, deposit the fresh buffer.
				 */
				p_buffer->write_index = write_index;
				p_buffer->fullness = fullness;
				spin_unlock_irqrestore(&(p_context->lock), flags);

				p_context->rx_put_buffer.p_cbk(
						p_context->rx_put_buffer.p_data,
						p_buffer);
				/*
				 * Ask for the next buffer to read into.
				 */
				p_buffer = p_context->rx_get_buffer.p_cbk(
							p_context->rx_get_buffer.p_data,
							4);
				write_index	= p_buffer->write_index;
				fullness	= p_buffer->fullness;
				size		= p_buffer->size;
				spin_lock_irqsave(&(p_context->lock), flags);
			}

			p_word = (unsigned int*)&(p_buffer->p_vaddr[write_index]);
			(*p_word) = 
				msm_uartdm_get_dword(p_context->p_uart_port);
#if MSM_HSUART_FEATURE_PRINT_RX_DATA
			printk("+++++ 0x%x +++++\n",(*p_word));
#endif // HSUART_FEATURE_PRINT_RX_DATA
			write_index += 4;
			fullness += 4;
			p_context->rx.read_byte_cnt += 4;
			if (write_index >= size) {
				write_index -= size;
			}
		}

		p_buffer->write_index	= write_index;
		p_buffer->fullness	= fullness;
		p_context->p_rx_buffer	= p_buffer;
MSM_HSUART_DEBUG("%s, write_index = %d; fullness = %d; valid_byte_cnt = %d\n",
		__FUNCTION__,
		write_index,
		fullness,
		p_context->rx.valid_byte_cnt);

		/*
		 * End of rx transaction....
		 */

		if (0 >= (p_context->rx.valid_byte_cnt - p_context->rx.read_byte_cnt)) {
			if (p_context->rx.the_end) {
				/*
				 * In case that we received a number that is not 
				 * a multiple of dwords (4bytes), make sure to 
				 * mark the relevant bytes only.
				 */
				if (0 != (p_context->rx.valid_byte_cnt & 0x3)) {
					p_buffer->fullness -= 
						(4 - (p_context->rx.valid_byte_cnt & 0x3));
					//TODO:	need to fix the write_index as well
				}
				MSM_HSUART_DEBUG(" THE END!\n");
				MSM_HSUART_DEBUG(" DMRX at reader 0x%x\n", msm_uartdm_get_received_byte_cnt(p_context->p_uart_port));
				msm_uartdm_config_read_size(p_context->p_uart_port, 
							    0xFFFF);
				p_context->rx.the_end = 0;
				p_context->rx.read_byte_cnt = 0;
				p_context->rx.valid_byte_cnt = 0;
				msm_uartdm_enable_rx_irqs(p_context->p_uart_port, 1);
	
	MSM_HSUART_DEBUG("==> END OF READ XFER (fullness %d, valid_byte_cnt %d)<==\n",p_buffer->fullness, p_context->rx.valid_byte_cnt);
				spin_unlock_irqrestore(&(p_context->lock), flags);

				/*
				 * First, deposit the fresh buffer.
				 */
				p_context->rx_put_buffer.p_cbk(
						p_context->rx_put_buffer.p_data,
						p_context->p_rx_buffer);
				/*
				 * Ask for the next buffer to read into.
				 */
				p_context->p_rx_buffer = p_context->rx_get_buffer.p_cbk(
							p_context->rx_get_buffer.p_data, 
							4); 
			}
			else {
				msm_uartdm_enable_rx_irqs(p_context->p_uart_port, 0);
				spin_unlock_irqrestore(&(p_context->lock), flags);

			}
		}
		else {
			msm_uartdm_enable_rx_irqs(p_context->p_uart_port, 0);
			spin_unlock_irqrestore(&(p_context->lock), flags);

		}

		MSM_HSUART_DEBUG("%s, %d, irq_cnt %d, valid_bytes %d read_byte_cnt %d\n",__FUNCTION__, __LINE__,irq_fire_cnt, p_context->rx.valid_byte_cnt, p_context->rx.read_byte_cnt);
	}
	else {
		MSM_HSUART_ERR("%s:%s, error, no buffer associated with the context\n", 
			DRIVER_NAME,
			__FUNCTION__);
	}
	MSM_HSUART_DEBUG("%s, %d, irq_cnt %d, valid_bytes %d read_byte_cnt %d\n",__FUNCTION__, __LINE__,irq_fire_cnt, p_context->rx.valid_byte_cnt, p_context->rx.read_byte_cnt);

	MSM_HSUART_EXIT();
}


/**
*
* Worker thread, responsible to read from the UART in DMA mode
*
* @param[in][out]       io_p_work - Pointer to the workqueue.
*
* @return None.
*
*/
void hsuart_tty_flip(void);

static void
__msm_hsuart_read_dm_worker(struct work_struct* io_p_work)
{
	struct hsuart_worker*	p_rd_worker;
	struct hsuart_context*	p_context;
	struct buffer_item*	p_buffer = NULL;

	MSM_HSUART_ENTER();

	p_rd_worker = container_of(io_p_work, struct hsuart_worker, worker);

	p_context = container_of(p_rd_worker, struct hsuart_context, reader);


	if (p_context && (p_context->p_rx_buffer)) {

		unsigned long flags;
		bool flushing ; 

		MSM_HSUART_LOG(p_context, MSM_UART_DM_WQ_ENT, p_context->rx.valid_byte_cnt , 0 ); 

		spin_lock_irqsave(&(p_context->lock), flags);

		flushing = ( p_context->rx.state == MSM_HSUART_RX_STATE_DM_FLUSHING );

		p_buffer = p_context->p_rx_buffer;
			
		p_context->rx.state = MSM_HSUART_RX_STATE_IDLE;

		if ( unlikely(flushing) ) {
			p_context->rx.valid_byte_cnt = 0;
			wake_up(&p_context->rx.transaction_complete);
		} else {
			p_buffer->fullness += p_context->rx.valid_byte_cnt; 
			p_buffer->write_index += p_context->rx.valid_byte_cnt; 
	
	
			if ( p_context->rx.valid_byte_cnt > 0 ) {
				p_context->rx_put_buffer.p_cbk(
						p_context->rx_put_buffer.p_data,
						p_context->p_rx_buffer);
		
				p_context->p_rx_buffer = p_context->rx_get_buffer.p_cbk(
							p_context->rx_get_buffer.p_data, 
							p_context->rx.max_packet_size);
				/*
				 * buffers to DM must be word aligned.
				 */
				if ( ((unsigned int)p_context->p_rx_buffer & 0xFFFFFFFC) != (unsigned int)p_context->p_rx_buffer){
					MSM_HSUART_ERR("rx buffer is not word aligned 0x%x, fixing to 0x%x\n", 
						p_context->p_rx_buffer, 
						(unsigned int)p_context->p_rx_buffer & 0xFFFFFFFC);
					p_context->p_rx_buffer = (struct buffer_item*)(((unsigned int)p_context->p_rx_buffer) & 0xFFFFFFFC);
				}
			}
			else {
				MSM_HSUART_DEBUG( " read work empty received \n");
			}
		
			msm_hsuart_config_dm_rx( p_context, NULL , p_context->rx.enable_stale);
		}

		spin_unlock_irqrestore(&(p_context->lock), flags);

		MSM_HSUART_LOG(p_context, MSM_UART_DM_WQ_EXT, 0 , 0 ); 

	}

	MSM_HSUART_EXIT();
	hsuart_tty_flip();
}

static int __msm_hsuart_suspend(struct hsuart_context* p_context) 
{
	msm_uartdm_port_suspend(p_context->p_uart_port);

	return 0;
}

static int __msm_hsuart_resume(struct hsuart_context* p_context) 
{
	msm_uartdm_port_resume(p_context->p_uart_port);

	__msm_hsuart_set_flow(p_context, MSM_HSUART_FLOW_DIR_RX, p_context->rx.flow_ctl , p_context->rx.flow_state);

	__msm_hsuart_set_flow(p_context, MSM_HSUART_FLOW_DIR_TX, p_context->tx.flow_ctl , 0);

	msm_uartdm_enable_rx(p_context->p_uart_port);
	msm_uartdm_enable_tx(p_context->p_uart_port);

	return 0;
}
	
/**
*
* opens a msm hsuart context.
*
* @param[in]		i_p_cfg - the configuration to use.
* @param[out]		o_p_context_id_handle - Pointer to a container to be filled 
*					with the newly created context in case 
*					of success otherwise undefined.
* @return 0 for success -1 otherwise.
*
* @Note	    	
*/
int
msm_hsuart_open_context(struct hsuart_config* io_p_cfg, int* o_p_context_id_handle)
{
	int				ret = 0;
	struct hsuart_context*		p_context = NULL;
	struct generic_uart_config	cfg = {0};

	MSM_HSUART_ENTER();
	
	if ((NULL == io_p_cfg) || (NULL == o_p_context_id_handle)) {
		ret = -EINVAL;
	}
	else {
		/* 
		 * TODO:HSUART Lock the global driver DB
		 */
		
		/*
		 * Allocate control block.
		 */
		p_context = kzalloc(sizeof(struct hsuart_context), GFP_KERNEL);
		if (NULL == p_context) {
			ret = -EINVAL;
			MSM_HSUART_ERR("%s, error, failed allocing context\n", __FUNCTION__);
			goto exit;
		}
		
		p_context->flags = io_p_cfg->flags;
		p_context->state = MSM_HSUART_STATE_ACTIVE;

		p_context->rx.max_packet_size = io_p_cfg->max_packet_size;
		p_context->rx.min_packet_size = io_p_cfg->min_packet_size;
		p_context->rx.latency         = io_p_cfg->rx_latency;
		p_context->rx.state           = MSM_HSUART_RX_STATE_IDLE;

		p_context->p_board_pin_mux_cb          = io_p_cfg->p_board_pin_mux_cb;
		p_context->p_board_rts_pin_deassert_cb = io_p_cfg->p_board_rts_pin_deassert_cb;

		init_waitqueue_head(&p_context->rx.transaction_complete);
		init_waitqueue_head(&p_context->tx.transaction_complete);

		p_context->tx.state             = MSM_HSUART_TX_STATE_IDLE;
		p_context->tx.transaction_size  = 0;
		/* 
		 * TODO:MSM_HSUART, init  rxtx_lists and other structs in the context 
		 */
		p_context->context_id = context_cnt++;
		init_completion(&(p_context->reader.xfer_done));
		init_completion(&(p_context->writer.xfer_done));
		mutex_init(&(p_context->read_complete_lock));
		mutex_init(&(p_context->write_complete_lock));

		spin_lock_init(&(p_context->lock));
		/*
		 * Get the desired UART port ID from the context into 
		 * the configuration request.
		 */
		cfg.port_id = io_p_cfg->port_id;

		if ( RX_MODE_PIO(p_context) ) {
			cfg.flags |= UART_DM_MODE_RX_PIO;
		}

		if ( RX_MODE_DM(p_context) ) {
			cfg.flags |= UART_DM_MODE_RX_DM;
		}
	
		cfg.rx_latency = p_context->rx.latency;
		cfg.p_board_pin_mux_cb          = p_context->p_board_pin_mux_cb;
		cfg.p_board_config_gsbi_cb      = io_p_cfg->p_board_gsbi_config_cb;
		cfg.p_board_rts_pin_deassert_cb = p_context->p_board_rts_pin_deassert_cb;


		p_context->rx_get_buffer.p_cbk = io_p_cfg->rx_get_buffer.p_cbk;
		p_context->rx_get_buffer.p_data = io_p_cfg->rx_get_buffer.p_data;
		p_context->rx_put_buffer.p_cbk  = io_p_cfg->rx_put_buffer.p_cbk;
		p_context->rx_put_buffer.p_data = io_p_cfg->rx_put_buffer.p_data;

		p_context->tx_get_buffer.p_cbk  = io_p_cfg->tx_get_buffer.p_cbk;
		p_context->tx_get_buffer.p_data = io_p_cfg->tx_get_buffer.p_data;
		p_context->tx_put_buffer.p_cbk  = io_p_cfg->tx_put_buffer.p_cbk;
		p_context->tx_put_buffer.p_data = io_p_cfg->tx_put_buffer.p_data;

		ret = msm_uartdm_port_open(&cfg, &(p_context->p_uart_port));

		MSM_HSUART_LOG(p_context, MSM_UART_OPEN_CONTEXT_ENT, 0 , 0 ); 

		if ( RX_MODE_DM(p_context) ) {
			//Stale is enabled in dm mode when expecting variable size packets
			p_context->rx.enable_stale = !RX_MODE_DM_FIXED_PACKET_LEN(p_context);
		} 
	
		if ( RX_MODE_PIO(p_context) ) {
			p_context->rx.enable_stale = 1;
		}

		MSM_HSUART_DEBUG("%s: %s, allocated platform hsuart, handle_0x%x\n",
				DRIVER_NAME, 
				__PRETTY_FUNCTION__, 
				(unsigned int)p_context->p_uart_port);

		if ( RX_MODE_DM(p_context) ) {
			if (0 == ret) {
				ret = msm_uartdm_set_rx_dm_cbk(p_context->p_uart_port,
							       __msm_hsuart_rx_dm_cbk,
							       (void*)p_context);
				MSM_HSUART_DEBUG("%s: %s, registered rx dma cbk, handle_0x%x, ret %d\n",
					DRIVER_NAME, 
					__PRETTY_FUNCTION__, 
					(unsigned int)p_context->p_uart_port, 
					ret);
			}

			if (0 == ret && p_context->rx.enable_stale) {
				ret = msm_uartdm_set_rx_stale_cbk(p_context->p_uart_port,
							__msm_hsuart_rx_dm_stale_cbk,
							(void*) p_context);
				MSM_HSUART_DEBUG("%s: %s, registered dm rx stale cbk, handle_0x%x, ret %d\n",
					DRIVER_NAME, 
					__PRETTY_FUNCTION__, 
					(unsigned int)p_context->p_uart_port, 
					ret);
			}

		}


		if ( TX_MODE_PIO(p_context) ) {

			if (0 == ret) {
				ret = msm_uartdm_set_tx_level_cbk(p_context->p_uart_port,
								__msm_hsuart_tx_level_cbk,
								(void*)p_context);
				MSM_HSUART_DEBUG("%s: %s, registered tx level cbk, handle_0x%x, ret %d\n",
					DRIVER_NAME, 
					__PRETTY_FUNCTION__, 
					(unsigned int)p_context->p_uart_port, 
					ret);
			}

			if (0 == ret) {
				ret = msm_uartdm_set_tx_rdy_cbk(p_context->p_uart_port,
								__msm_hsuart_tx_rdy_cbk,
								(void*)p_context);
				MSM_HSUART_DEBUG("%s: %s, registered tx rdy cbk, handle_0x%x, ret %d\n",
					DRIVER_NAME, 
					__PRETTY_FUNCTION__, 
					(unsigned int)p_context->p_uart_port, 
					ret);
			}
		}

		if ( RX_MODE_PIO(p_context) ) {

			if (0 == ret) {
				ret = msm_uartdm_set_rx_level_cbk(p_context->p_uart_port,
							__msm_hsuart_rx_level_cbk,
							(void*) p_context);
				MSM_HSUART_DEBUG("%s: %s, registered rx level cbk, handle_0x%x, ret %d\n",
					DRIVER_NAME, 
					__PRETTY_FUNCTION__, 
					(unsigned int)p_context->p_uart_port, 
					ret);
			}

			if (0 == ret) {
				ret = msm_uartdm_set_rx_stale_cbk(p_context->p_uart_port,
							__msm_hsuart_rx_pio_stale_cbk,
							(void*) p_context);
				MSM_HSUART_DEBUG("%s: %s, registered rx pio stale cbk, handle_0x%x, ret %d\n",
					DRIVER_NAME, 
					__PRETTY_FUNCTION__, 
					(unsigned int)p_context->p_uart_port, 
					ret);
			}

		}


		if (0 == ret) {
			ret = msm_uartdm_port_init(p_context->p_uart_port);
			MSM_HSUART_DEBUG("%s: %s, initialized platform hsuart, handle_0x%x, ret %d\n",
					DRIVER_NAME, 
					__PRETTY_FUNCTION__, 
					(unsigned int)p_context->p_uart_port, 
					ret);
		}

		/* 
		 * TODO:HSUART Add the control block into list of ctl blocks
		 */
		
		/*
		 * Check and handle the configuration options for the context
		 */
		if (RX_MODE_DM(p_context)) {
			/*
				* Config and launch worker thread to handle
				* read request
				*/
			snprintf(p_context->reader.name, 
					sizeof(p_context->reader.name),
					"hsuart_rd_dm_%d", 
					context_cnt);
			p_context->reader.p_worker = 
				__create_workqueue(p_context->reader.name,
							1,	/* singlethread */
							0,	/* freezeable */
							p_context->flags & HSUART_CFG_SCHED_RT);
			INIT_WORK(&(p_context->reader.worker), 
					__msm_hsuart_read_dm_worker);
		} else if (RX_MODE_PIO(p_context)) {
			/*
				* Config and launch worker thread to handle
				* read request
				*/
			snprintf(p_context->reader.name, 
					sizeof(p_context->reader.name),
					"hsuart_rd_pio_%d", 
					context_cnt);
			p_context->reader.p_worker = 
				__create_workqueue(p_context->reader.name,
							1,	/* singlethread */
							0,	/* freezeable */
							p_context->flags & HSUART_CFG_SCHED_RT);

			INIT_WORK(&(p_context->reader.worker), 
					__msm_hsuart_read_pio_worker);
		}

		if (TX_MODE_PIO(p_context) || TX_MODE_DM(p_context)) {
			/*
				* Config and launch worker thread to handle
				* write request
				*/
			snprintf(p_context->writer.name, 
					sizeof(p_context->writer.name),
					"hsuart_wr_%d", 
					context_cnt);
			p_context->writer.p_worker = 
				__create_workqueue(p_context->reader.name,
							1,	/* singlethread */
							0,	/* freezeable */
							p_context->flags & HSUART_CFG_SCHED_RT);
			INIT_WORK(&(p_context->writer.worker), 
					__msm_hsuart_write_worker);

		}

		/* 
		 * Unlock he driver DB
		 */
	}

	MSM_HSUART_EXIT();
exit:
	if (0 != ret) {
		/*
		 * Cleanup time...
		 */
		if (p_context) {
			if (p_context->p_uart_port) {
				/*
				 * TODO:HSUART remove the rx/tx cbks.
				 */
				msm_uartdm_port_close(p_context->p_uart_port);
			}
			kfree(p_context);
		}
	}
	else {
		(*o_p_context_id_handle) = (int)&(p_context->context_id);
	}

	MSM_HSUART_LOG(p_context, MSM_UART_OPEN_CONTEXT_EXT, 0 , 0 ); 

	return ret;
}

EXPORT_SYMBOL(msm_hsuart_open_context);


/**
*
* close msm hsuart context.
*
* @param[in]		context_id_handle - Handle that serves as the 
*					context ID to close
*
* @return 0 for success -1 otherwise.
*
* @Note	    	
*/
int
msm_hsuart_close_context(int context_id_handle)
{
	int				ret = 0;
	struct hsuart_context*		p_context;

	MSM_HSUART_ENTER();
	
	p_context = container_of((void*)context_id_handle, 
				struct hsuart_context, 
				context_id);
	if (NULL != p_context) {

		MSM_HSUART_LOG(p_context, MSM_UART_CLOSE_CONTEXT_ENT, 0 , 0 ); 
		
		msm_hsuart_flush_dm(p_context);
		/*
		 * Stop the workqueues.
		 */
		if (RX_MODE_DM(p_context) || RX_MODE_PIO(p_context)) {
			flush_workqueue(p_context->reader.p_worker);
			destroy_workqueue(p_context->reader.p_worker);
		}

		if (TX_MODE_DM(p_context) || TX_MODE_PIO(p_context)) {
			flush_workqueue(p_context->writer.p_worker);
			destroy_workqueue(p_context->writer.p_worker);
		}
	
		ret = msm_uartdm_port_close(p_context->p_uart_port);
		if (p_context->p_rx_buffer) {
			/*
			 * Deposit the buffer back to where it came from...
			 */
			p_context->rx_put_buffer.p_cbk(
				p_context->rx_put_buffer.p_data,
				p_context->p_rx_buffer);
		}
		MSM_HSUART_DEBUG("%s: %s, released platform hsuart, handle_0x%x, ret %d\n",
			    DRIVER_NAME,
			    __PRETTY_FUNCTION__, 
			    (unsigned int)p_context->p_uart_port,
			    ret);

		p_context->p_uart_port = NULL;

		kfree(p_context);
	}
	else {
		ret = -EINVAL;
	}
	MSM_HSUART_EXIT();
	return ret;
}

EXPORT_SYMBOL(msm_hsuart_close_context);

static int __msm_hsuart_write(struct hsuart_context* p_context)
{
	int				ret = 0;

	MSM_HSUART_ENTER();

	if ((NULL != p_context)) {

		queue_work(p_context->writer.p_worker, 
			   &(p_context->writer.worker));

	}
	else {
		MSM_HSUART_ERR("%s, %s, error - illegal parameters p_ctxt_0x%x\n", 
			DRIVER_NAME,
			__FUNCTION__,
			(unsigned int) p_context);
		ret = -EINVAL;
	}
	MSM_HSUART_EXIT();
	return ret;
}

/**
*
* start/continue writing data to the msm hsuart context.
*
* @param[in]		context_id_handle - Handle that serves as the 
*					context ID to write from.
*
*
* @return 0 for success -1 otherwise.
*
* @Note	    	
*/
int
msm_hsuart_write(int context_id_handle)
{
	int				ret = 0;
	struct hsuart_context*		p_context;

	MSM_HSUART_ENTER();
	p_context = container_of((void*)context_id_handle, 
				struct hsuart_context, 
				context_id);
	if ((NULL != p_context)) {
		/*
		 * Make sure we don't initiate another tx before the current 
		 * one is complete.
		 */
		__msm_hsuart_write(p_context);
	}
	else {
		MSM_HSUART_ERR("%s, %s, error - illegal parameters p_ctxt_0x%x\n", 
			DRIVER_NAME,
			__FUNCTION__,
			(unsigned int) p_context);
		ret = -EINVAL;
	}
	MSM_HSUART_EXIT();
	return ret;
}

EXPORT_SYMBOL(msm_hsuart_write);

static void __msm_hsuart_tx_wait_for_completion(struct hsuart_context*	p_context)
{
	//Wait for TX transaction to complete	
	wait_event_interruptible(p_context->tx.transaction_complete, p_context->tx.state == MSM_HSUART_TX_STATE_IDLE );
}

/**
*
* suspend msm hsuart context
*
* @param[in]		context_id_handle - Handle that serves as the 
*					context ID to suspend
*
* @return 0 for success -1 otherwise.
*
* @Note	    	
*/
int msm_hsuart_suspend(int context_id_handle)
{
	int				ret = 0;
	struct hsuart_context*		p_context;
	unsigned long			flags;

	MSM_HSUART_ENTER();

	p_context = container_of((void*)context_id_handle, 
				struct hsuart_context, 
				context_id);

	if (NULL != p_context) {

		MSM_HSUART_LOG(p_context, MSM_UART_SUSPEND_ENT, 0 , 0 ); 

		spin_lock_irqsave(&(p_context->lock), flags);

		if ( p_context->state == MSM_HSUART_STATE_ACTIVE ) {

			p_context->state = MSM_HSUART_STATE_SUSPENDING;
	
			spin_unlock_irqrestore(&(p_context->lock), flags);
	
			__msm_hsuart_tx_wait_for_completion(p_context);
	
			if ( RX_MODE_DM(p_context) ) {
				msm_hsuart_flush_dm(p_context);
			}
	
			spin_lock_irqsave(&(p_context->lock), flags);
	
			p_context->state = MSM_HSUART_STATE_SUSPENDED;
			ret = __msm_hsuart_suspend(p_context);	
		}
	
		spin_unlock_irqrestore(&(p_context->lock), flags);
		MSM_HSUART_LOG(p_context, MSM_UART_SUSPEND_EXT, 0 , 0 ); 

	}

	MSM_HSUART_EXIT();
	return ret;
}

EXPORT_SYMBOL(msm_hsuart_suspend);

/**
*
* resume msm hsuart context
*
* @param[in]		context_id_handle - Handle that serves as the 
*					context ID to resume
*
* @return 0 for success -1 otherwise.
*
* @Note	    	
*/
int msm_hsuart_resume(int context_id_handle)
{
	int				ret = 0;
	struct hsuart_context*		p_context;
	unsigned long			flags;

	MSM_HSUART_ENTER();
	
	p_context = container_of((void*)context_id_handle, 
				struct hsuart_context, 
				context_id);

	if (NULL != p_context) {

		MSM_HSUART_LOG(p_context, MSM_UART_RESUME_ENT, 0 , 0 ); 

		spin_lock_irqsave(&(p_context->lock), flags);

		if ( p_context->state != MSM_HSUART_STATE_ACTIVE ) {
			
	
			ret = __msm_hsuart_resume(p_context);	
	
			if ( !ret ) {
				p_context->state = MSM_HSUART_STATE_ACTIVE;			
	
				if ( RX_MODE_DM(p_context) ) {
					msm_hsuart_config_dm_rx( p_context, NULL , p_context->rx.enable_stale);
				}

				if (TX_MODE_DM(p_context) || TX_MODE_PIO(p_context)) {
					__msm_hsuart_write(p_context);
				}
			}
		}

		spin_unlock_irqrestore(&(p_context->lock), flags);

		MSM_HSUART_LOG(p_context, MSM_UART_RESUME_EXT, 0 , 0 ); 
	}

	MSM_HSUART_EXIT();
	return ret;
}

EXPORT_SYMBOL(msm_hsuart_resume);

/**
*
* Initiate data read from the msm hsuart context in DMA mode
*
* @param[in]		context_id_handle - Handle that serves as the 
*					context ID to read from
*
* @param[in][out]	io_p_buffer - Pointer to the buffer structure 
*					to use for read from.
*
* @return 0 for success -1 otherwise.
*
* @Note	    	
*/
int
msm_hsuart_read_pio(int context_id_handle, struct buffer_item* io_p_buffer)
{
	int				ret = 0;
	struct hsuart_context*		p_context;
	unsigned long			flags;

	MSM_HSUART_DEBUG("%s, enter context_id 0x%x, p_buffer 0x%x, sz 0x%x\n", 
		__FUNCTION__, 
		(unsigned int)context_id_handle, 
		(unsigned int)io_p_buffer, 
		io_p_buffer->size);
	p_context = container_of((void*)context_id_handle, 
				struct hsuart_context, 
				context_id);
	if ((NULL != p_context) && (NULL != io_p_buffer)) {
		/*
		 * Make sure we don't initiate another read before the current 
		 * one is complete.
		 */
#if 0
		ret = mutex_lock_interruptible(&(p_context->read_complete_lock));
		if (ret) {
			MSM_HSUART_ERR("%s, %s:failed locking the mutex ret:%d\n",
					DRIVER_NAME,
					__PRETTY_FUNCTION__,
					ret);
			goto exit;
		}
#endif
		spin_lock_irqsave(&(p_context->lock), flags);
		/*
		 * Cache that buffer to used for the current read.
		 */
		p_context->p_rx_buffer = io_p_buffer; 

		/*
		 * Be positive - assume we are going to get all the bytes that 
		 * we asked for...
		 */
		//use the fifo threshold level
		p_context->rx.the_end = 0;
		p_context->rx.read_byte_cnt = 0;
		p_context->rx.valid_byte_cnt = 0;//io_p_buffer->size;

		msm_uartdm_config_read_size(p_context->p_uart_port, 0xFFFF);
		msm_uartdm_enable_rx_irqs(p_context->p_uart_port, 1);

		spin_unlock_irqrestore(&(p_context->lock), flags);

#if 0
		wait_for_completion_interruptible(&(p_context->reader.xfer_done));

		mutex_unlock(&(p_context->read_complete_lock));
#endif		
	}
	else {
		ret = -EINVAL;
	}
	MSM_HSUART_EXIT();
	return ret;
}

/**
*
* Initiate data read from the msm hsuart context in DMA mode
*
* @param[in]		context_id_handle - Handle that serves as the 
*					context ID to read from
*
* @param[in][out]	io_p_buffer - Pointer to the buffer structure 
*					to use for read from.
*
* @return 0 for success -1 otherwise.
*
* @Note	    	
*/
int
msm_hsuart_read_dm(int context_id_handle, struct buffer_item* io_p_buffer)
{

	int				ret = 0;
	struct hsuart_context*		p_context;

	MSM_HSUART_DEBUG("%s, enter context_id 0x%x, p_buffer 0x%x, sz 0x%x\n", 
		__FUNCTION__, 
		(unsigned int)context_id_handle, 
		(unsigned int)io_p_buffer, 
		io_p_buffer->size);

	p_context = container_of((void*)context_id_handle, 
				struct hsuart_context, 
				context_id);

	if ((NULL != p_context) && (NULL != io_p_buffer)) {
		unsigned long flags;

		spin_lock_irqsave(&(p_context->lock), flags);

		p_context->rx.valid_byte_cnt = 0;
		msm_hsuart_config_dm_rx( p_context ,io_p_buffer , p_context->rx.enable_stale);

		spin_unlock_irqrestore(&(p_context->lock), flags);

	}

	MSM_HSUART_EXIT();
	return ret;
}

/**
*
* Initiate data read from the msm hsuart 
*
* @param[in]		context_id_handle - Handle that serves as the 
*					context ID to read from
*
* @param[in][out]	io_p_buffer - Pointer to the buffer structure 
*					to use for read from.
*
* @return 0 for success -1 otherwise.
*
* @Note	    	
*/
int
msm_hsuart_read(int context_id_handle, struct buffer_item* io_p_buffer)
{

	int				ret = 0;
	struct hsuart_context*		p_context;

	MSM_HSUART_DEBUG("%s, enter context_id 0x%x, p_buffer 0x%x, sz 0x%x\n", 
		__FUNCTION__, 
		(unsigned int)context_id_handle, 
		(unsigned int)io_p_buffer, 
		io_p_buffer->size);

	p_context = container_of((void*)context_id_handle, 
				struct hsuart_context, 
				context_id);

	if ( RX_MODE_PIO(p_context)){
		ret = msm_hsuart_read_pio(context_id_handle, io_p_buffer);

	} else {
		ret = msm_hsuart_read_dm(context_id_handle, io_p_buffer);
	}

	MSM_HSUART_EXIT();
	return ret;
}

EXPORT_SYMBOL(msm_hsuart_read);


/**
*
* Register 'set buffer' callback function
*
* @param[in]	context_id_handle - Handle to register the callback to.
* @param[in]	p_cbk - Pointer to a callback function that will be 
*			called to get the next free buffer to read data into.
* @param[in]	p_data -Pointer to data to pass to the callback function 
*			when it is called.
*
* @return 0 for success -1 otherwise.
*
* @Note	    	
*/
int 
msm_hsuart_register_rx_put_buffer(
		int context_id_handle, 
		void (*p_cbk)(void* p_data, struct buffer_item* p_buffer),
		void* p_data)
{
	int				ret		= 0;
	unsigned long			flags;
	struct hsuart_context*		p_context	= NULL;

	MSM_HSUART_DEBUG("%s, enter context_id 0x%x\n", 
		__FUNCTION__, 
		(unsigned int)context_id_handle); 
	
	p_context = container_of((void*)context_id_handle, 
				struct hsuart_context, 
				context_id);
	if (	(NULL != p_context) && 
		(NULL == p_context->rx_put_buffer.p_cbk) &&
		(NULL == p_context->rx_put_buffer.p_data)	) {

		spin_lock_irqsave(&(p_context->lock), flags);
		p_context->rx_put_buffer.p_cbk = p_cbk;
		p_context->rx_put_buffer.p_data = p_data;
		spin_unlock_irqrestore(&(p_context->lock), flags);
	}
	else {
		MSM_HSUART_ERR("%s, %s, error - illegal parameters p_ctxt_0x%x\n", 
			DRIVER_NAME,
			__FUNCTION__,
			(unsigned int) p_context);
		ret = -EINVAL;
	}
	MSM_HSUART_EXIT();

	return ret;
}
EXPORT_SYMBOL(msm_hsuart_register_rx_put_buffer);

/**
*
* Register 'get buffer' callback function
*
* @param[in]	context_id_handle - Handle to register the callback to.
* @param[in]	p_cbk - Pointer to a callback function that will be 
*			called to get the next free buffer to read data into.
* @param[in]	p_data -Pointer to data to pass to the callback function 
*			when it is called.
*
* @return 0 for success -1 otherwise.
*
* @Note	    	
*/
int 
msm_hsuart_register_rx_get_buffer(
		int context_id_handle, 
		struct buffer_item* (*p_cbk)(void* p_data, int free_bytes),
		void* p_data)
{
	int				ret		= 0;
	unsigned long			flags;
	struct hsuart_context*		p_context	= NULL;

	MSM_HSUART_DEBUG("%s, enter context_id 0x%x\n", 
		__FUNCTION__, 
		(unsigned int)context_id_handle); 
	
	p_context = container_of((void*)context_id_handle, 
				struct hsuart_context, 
				context_id);
	if (	(NULL != p_context) && 
		(NULL == p_context->rx_get_buffer.p_cbk) &&
		(NULL == p_context->rx_get_buffer.p_data)	) {

		spin_lock_irqsave(&(p_context->lock), flags);
		p_context->rx_get_buffer.p_cbk = p_cbk;
		p_context->rx_get_buffer.p_data = p_data;
		spin_unlock_irqrestore(&(p_context->lock), flags);
	}
	else {
		MSM_HSUART_ERR("%s, %s, error - illegal parameters p_ctxt_0x%x\n", 
			DRIVER_NAME,
			__FUNCTION__,
			(unsigned int) p_context);
		ret = -EINVAL;
	}
	MSM_HSUART_EXIT();

	return ret;
}
EXPORT_SYMBOL(msm_hsuart_register_rx_get_buffer);

/**
*
* Register 'set buffer' callback function
*
* @param[in]	context_id_handle - Handle to register the callback to.
* @param[in]	p_cbk - Pointer to a callback function that will be 
*			called to get the next free buffer to read data into.
* @param[in]	p_data -Pointer to data to pass to the callback function 
*			when it is called.
*
* @return 0 for success -1 otherwise.
*
* @Note	    	
*/
int 
msm_hsuart_register_tx_put_buffer(
		int context_id_handle, 
		void (*p_cbk)(void* p_data, struct buffer_item* p_buffer, int transaction_size),
		void* p_data)
{
	int				ret		= 0;
	unsigned long			flags;
	struct hsuart_context*		p_context	= NULL;

	MSM_HSUART_DEBUG("%s, enter context_id 0x%x\n", 
		__FUNCTION__, 
		(unsigned int)context_id_handle); 
	
	p_context = container_of((void*)context_id_handle, 
				struct hsuart_context, 
				context_id);
	if (	(NULL != p_context) && 
		(NULL == p_context->tx_put_buffer.p_cbk) &&
		(NULL == p_context->tx_put_buffer.p_data)	) {

		spin_lock_irqsave(&(p_context->lock), flags);
		p_context->tx_put_buffer.p_cbk = p_cbk;
		p_context->tx_put_buffer.p_data = p_data;
		spin_unlock_irqrestore(&(p_context->lock), flags);
	}
	else {
		MSM_HSUART_ERR("%s, %s, error - illegal parameters p_ctxt_0x%x\n", 
			DRIVER_NAME,
			__FUNCTION__,
			(unsigned int) p_context);
		ret = -EINVAL;
	}
	MSM_HSUART_EXIT();

	return ret;
}
EXPORT_SYMBOL(msm_hsuart_register_tx_put_buffer);

/**
*
* Register 'get buffer' callback function
*
* @param[in]	context_id_handle - Handle to register the callback to.
* @param[in]	p_cbk - Pointer to a callback function that will be 
*			called to get the next free buffer to read data into.
* @param[in]	p_data -Pointer to data to pass to the callback function 
*			when it is called.
*
* @return 0 for success -1 otherwise.
*
* @Note	    	
*/
int 
msm_hsuart_register_tx_get_buffer(
		int context_id_handle, 
		struct buffer_item* (*p_cbk)(void* p_data),
		void* p_data)
{
	int				ret		= 0;
	unsigned long			flags;
	struct hsuart_context*		p_context	= NULL;

	MSM_HSUART_DEBUG("%s, enter context_id 0x%x\n", 
		__FUNCTION__, 
		(unsigned int)context_id_handle); 
	
	p_context = container_of((void*)context_id_handle, 
				struct hsuart_context, 
				context_id);
	if (	(NULL != p_context) && 
		(NULL == p_context->tx_get_buffer.p_cbk) &&
		(NULL == p_context->tx_get_buffer.p_data)	) {

		spin_lock_irqsave(&(p_context->lock), flags);
		p_context->tx_get_buffer.p_cbk = p_cbk;
		p_context->tx_get_buffer.p_data = p_data;
		spin_unlock_irqrestore(&(p_context->lock), flags);
	}
	else {
		MSM_HSUART_ERR("%s, %s, error - illegal parameters p_ctxt_0x%x\n", 
			DRIVER_NAME,
			__FUNCTION__,
			(unsigned int) p_context);
		ret = -EINVAL;
	}
	MSM_HSUART_EXIT();

	return ret;
}
EXPORT_SYMBOL(msm_hsuart_register_tx_get_buffer);


/**
*
* Configure the flow control of the UART port.
*
* @param[in]	context_id_handle - Handle to register the callback to.
* @param[in]	flow - The flow mode to apply. See hsuart.h for definition of 
*			the possible modes.
* 
* @return 0 for success -1 otherwise.
*
* @Note	    	
*/
int 
msm_hsuart_set_flow(
		int context_id_handle, 
		int flow)
{
	int				ret		= 0;
	unsigned long			flags;
	struct hsuart_context*		p_context	= NULL;

	MSM_HSUART_DEBUG( "%s, enter context_id 0x%x flow 0x%x\n", 
		__FUNCTION__, 
		(uint32_t)context_id_handle,
		flow); 

	p_context = container_of((void*)context_id_handle, 
				struct hsuart_context, 
				context_id);

	if (NULL != p_context){
		int flow_dir = flow & HSUART_MODE_FLOW_DIRECTION_MASK;
		bool set_rx_flow = ( flow_dir == HSUART_MODE_FLOW_DIRECTION_RX_TX ) || ( flow_dir == HSUART_MODE_FLOW_DIRECTION_RX_ONLY );
		bool set_tx_flow = ( flow_dir == HSUART_MODE_FLOW_DIRECTION_RX_TX ) || ( flow_dir == HSUART_MODE_FLOW_DIRECTION_TX_ONLY );
		int flow_ctl = ( flow & HSUART_MODE_FLOW_CTRL_MODE_MASK );
		int flow_state = ( flow & HSUART_MODE_FLOW_STATE_MASK);
		int msm_hsuart_flow_state = (flow_state == HSUART_MODE_FLOW_STATE_ASSERT ) ? MSM_HSUART_FLOW_STATE_ASSERT : MSM_HSUART_FLOW_STATE_DEASSERT;

		spin_lock_irqsave(&(p_context->lock), flags);

		if ( set_rx_flow ) {
			__msm_hsuart_set_flow(p_context, MSM_HSUART_FLOW_DIR_RX, flow_ctl, msm_hsuart_flow_state);
		}

		if ( set_tx_flow ) {
			__msm_hsuart_set_flow(p_context, MSM_HSUART_FLOW_DIR_TX, flow_ctl, msm_hsuart_flow_state);
		} 		

		spin_unlock_irqrestore(&(p_context->lock), flags);
	}
	else {
		MSM_HSUART_ERR( "%s, %s, error - illegal parameters p_ctxt_0x%x\n", 
			DRIVER_NAME,
			__FUNCTION__,
			(uint32_t) p_context);
		ret = -EINVAL;
	}
	MSM_HSUART_EXIT();

	return ret;
}
EXPORT_SYMBOL(msm_hsuart_set_flow);
/**
*
* Configure the parity of the UART port.
*
* @param[in]	context_id_handle - Handle to register the callback to.
* @param[in]	parity - The parity mode to apply. See hsuart.h for definition of 
*			the possible modes.
* 
* @return 0 for success -1 otherwise.
*
* @Note	    	
*/
int 
msm_hsuart_set_parity(
		int context_id_handle, 
		int parity)
{
	int				ret		= 0;
	struct hsuart_context*		p_context	= NULL;
	msm_uartdm_parity_t		msm_uartdm_parity;
	unsigned long			flags;

	MSM_HSUART_ERR( "%s, enter context_id 0x%x parity 0x%x\n", 
		__FUNCTION__, 
		(uint32_t)context_id_handle,
		parity); 

	p_context = container_of((void*)context_id_handle, 
				struct hsuart_context, 
				context_id);

	if (NULL != p_context){
		spin_lock_irqsave(&(p_context->lock), flags);
		switch (parity) {
			case HSUART_MODE_PARITY_NONE:
				msm_uartdm_parity = MSM_UARTDM_PARITY_NONE; 
			break;
			case HSUART_MODE_PARITY_ODD:
				msm_uartdm_parity = MSM_UARTDM_PARITY_ODD;
			break;
			case HSUART_MODE_PARITY_EVEN:
				msm_uartdm_parity = MSM_UARTDM_PARITY_EVEN;
			break;
			default:
				MSM_HSUART_ERR(KERN_ERR "%s, %s, error, invalid parity %d\n",
					DRIVER_NAME,
					__FUNCTION__,
					parity);
				goto Done;
			break;
		}
		p_context->parity = msm_uartdm_parity;
		msm_uartdm_set_parity(
				p_context->p_uart_port, 
				msm_uartdm_parity);
Done:
		spin_unlock_irqrestore(&(p_context->lock), flags);

	}
	else {
		MSM_HSUART_ERR("%s, %s, error - illegal parameters p_ctxt_0x%x\n", 
			DRIVER_NAME,
			__FUNCTION__,
			(uint32_t) p_context);
		ret = -EINVAL;
	}

	MSM_HSUART_EXIT();
	return ret;
}
EXPORT_SYMBOL(msm_hsuart_set_parity);

/**
*
* Indicates whether there are bytes in rx fifo
*
* @param[in]       context_id_handle - context id
*
* @return 0 if rx fifo is empty and 1 otherwise.
*
*/
int 
msm_hsuart_rx_fifo_has_bytes( int context_id_handle)
{
	int				ret		= 0;
	unsigned long			flags;
	struct hsuart_context*		p_context	= NULL;

	p_context = container_of((void*)context_id_handle, 
				struct hsuart_context, 
				context_id);

	if (NULL != p_context){
		int packing_bytes = 0;
		int rx_fifo_fullness;

		spin_lock_irqsave(&(p_context->lock), flags);

		rx_fifo_fullness = msm_uartdm_get_rx_fifo_fullness(p_context->p_uart_port, &packing_bytes);

		ret = !!rx_fifo_fullness || !!packing_bytes;

		spin_unlock_irqrestore(&(p_context->lock), flags);
	}

	return ret;
}

EXPORT_SYMBOL(msm_hsuart_rx_fifo_has_bytes);


/**
*
* Configure the uart port to the requested baud-rate.
*
* @param[in]       i_p_port - The UART port to configure.
* @param[in]       baud - The requested baud rate.
*
* @return 0 for success -1/-ErrCode otherwise.
*
*/
int 
msm_hsuart_set_baud_rate(int context_id_handle, uint32_t baud_rate)
{
	int				ret		= 0;
	struct hsuart_context*		p_context	= NULL;
	unsigned long			flags;
	int				rx_irqs_enable;
	int				tx_rdy_enable;
	int				tx_lvl_enable;
	struct generic_uart_port*	p_port;

	MSM_HSUART_DEBUG("%s, enter context_id 0x%x speed %d\n", 
		__FUNCTION__, 
		(uint32_t)context_id_handle,
		baud_rate); 

	p_context = container_of((void*)context_id_handle, 
				struct hsuart_context, 
				context_id);
	if (NULL != p_context){
		bool dm_flush;

		dm_flush = msm_hsuart_flush_dm(p_context);

		spin_lock_irqsave(&(p_context->lock), flags);

		p_port = p_context->p_uart_port;

		/*
		 * Save IRQ status to restore it later.
		 */
		rx_irqs_enable 	= msm_uartdm_disable_rx_irqs(p_port);
		tx_rdy_enable 	= msm_uartdm_disable_tx_rdy(p_port);
		tx_lvl_enable 	= msm_uartdm_disable_tx_level(p_port);

		p_context->baud_rate = baud_rate;
		ret = msm_uartdm_set_baud_rate(
				p_context->p_uart_port, 
				baud_rate);
		if (ret) {
			MSM_HSUART_ERR("%s, %s, error at %d ret %d",
				DRIVER_NAME,
				__FUNCTION__,
				__LINE__,
				ret);
			goto Done;
		}
		//flush_workqueue(p_context->reader.p_worker);
		//flush_workqueue(p_context->writer.p_worker);

		if (tx_lvl_enable) {
			msm_uartdm_enable_tx_level(p_context->p_uart_port);
		}
		if (tx_rdy_enable) {
			msm_uartdm_enable_tx_rdy(p_context->p_uart_port);
		}
		if (rx_irqs_enable) {
			msm_uartdm_enable_rx_irqs(p_context->p_uart_port, p_context->rx.enable_stale);
		}
Done:

		if ( dm_flush ) {
			msm_hsuart_resume_dm(p_context);
		}

		//asm("nop");
		spin_unlock_irqrestore(&(p_context->lock), flags);
	}
	else {
		MSM_HSUART_ERR("%s, %s, error - illegal parameters p_ctxt_0x%x\n", 
			DRIVER_NAME,
			__FUNCTION__,
			(uint32_t) p_context);
		ret = -EINVAL;
	}

	MSM_HSUART_EXIT();
	return ret;
}
EXPORT_SYMBOL(msm_hsuart_set_baud_rate);


static int __init
msm_init_hsuart(void)
{
	int ret = 0;

	return ret;
}

arch_initcall(msm_init_hsuart);

MODULE_AUTHOR("Amir Frenkel <amir.frenkel@palm.com>");
MODULE_DESCRIPTION("Driver for msm7x high speed uart");
MODULE_LICENSE("GPL");
