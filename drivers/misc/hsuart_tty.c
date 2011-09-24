/*
 *  linux/drivers/misc/hsuart.c - High speed UART driver
 *
 *  Copyright (C) 2008 Palm Inc,
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Author: Amir Frenkel (amir.frenkel@palm.com)
 * Based on drivers/misc/omap-misc-hsuart.c
 *
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/bitops.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/hsuart.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/poll.h>

#include <linux/time.h>
#include <linux/hrtimer.h>

#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>

#include <mach/msm_hsuart.h>


#define  DRIVER_NAME      			"hsuart_tty"
#define  DRIVER_VERSION   			 (0x100)

/*
 * global switch to enable debug msgs in the module
 */
static int	_dbg_lvl_ = 0x01;
#define HSUART_DEBUG_LEVEL_ERR		(0x1)
#define HSUART_DEBUG_LEVEL_INFO		(0x2)
#define HSUART_DEBUG_LEVEL_DEBUG	(0x4)
#define HSUART_DEBUG_LEVEL_ENTER	(0x8)
#define HSUART_DEBUG_LEVEL_EXIT		(0x10)


#define HSUART_DEBUG_ENABLE			0
#define HSUART_FUNC_LOG_ENABLE		0
#if HSUART_DEBUG_ENABLE
#define HSUART_DEBUG(args...)	{if (_dbg_lvl_ & HSUART_DEBUG_LEVEL_DEBUG) \
					printk(KERN_ERR args);}
#define HSUART_INFO(args...)	{if (_dbg_lvl_ & HSUART_DEBUG_LEVEL_INFO) \
					printk(KERN_ERR args);}
#define HSUART_ERR(args...)	{if (_dbg_lvl_ & HSUART_DEBUG_LEVEL_ERR)  \
					printk(KERN_ERR args);}
#else
#define HSUART_INFO(args...)
#define HSUART_DEBUG(args...)
#define HSUART_ERR(args...)
#endif // HSUART_DEBUG_ENABLE					

#if HSUART_FUNC_LOG_ENABLE

#define	HSUART_ENTER()		{if (_dbg_lvl_ & HSUART_DEBUG_LEVEL_ENTER)		\
					printk(KERN_INFO"%s: %s, %u[msec] enter\n",	\
					DRIVER_NAME, __PRETTY_FUNCTION__, jiffies_to_msecs(jiffies));}
#define	HSUART_EXIT()		{if (_dbg_lvl_ & HSUART_DEBUG_LEVEL_EXIT)		\
					printk(KERN_INFO"%s: %s, %u[msec] exit\n",\
					DRIVER_NAME, __PRETTY_FUNCTION__, jiffies_to_msecs(jiffies));}
#define	HSUART_EXIT_RET(ret)	{if (_dbg_lvl_ & HSUART_DEBUG_LEVEL_EXIT)		\
					printk(KERN_INFO"%s: %s, ret %d %u[msec] exit\n",\
					DRIVER_NAME, __PRETTY_FUNCTION__, ret, jiffies_to_msecs(jiffies));}

#else
#define HSUART_ENTER()
#define HSUART_EXIT()
#define HSUART_EXIT_RET(ret)

#endif

#define HSUART_DEBUG_TIMING      0 
#define HSUART_DEBUG_TIMING_PORT 1

#if HSUART_DEBUG_TIMING
#include <linux/hres_counter.h>

static char* dbg_strings[] = { "hsuart rx get buff enter",
                               "hsuart rx get buff exit",
                               "hsuart rx put buff enter",
                               "hsuart rx put buff exit",

                               "hsuart write enter",
                               "hsuart write exit",
                               "hsuart read  enter",
                               "hsuart read  exit",

                               "hsuart tx get buff evt",
                               "hsuart tx put buff evt",
			     };

#define HS_UART_GB_ENT    0	
#define HS_UART_GB_EXT    1	
#define HS_UART_PB_ENT    2	
#define HS_UART_PB_EXT    3	
#define HS_UART_WRITE_ENT 4	
#define HS_UART_WRITE_EXT 5	
#define HS_UART_READ_ENT  6	
#define HS_UART_READ_EXT  7	
#define HS_UART_TX_GET_BUFF_EVT  8	
#define HS_UART_TX_PUT_BUFF_EVT  9	


	#define HSUART_LOG(p_context, eventid, arg1, arg2 )                        \
		if ( p_context->uart_port_number == HSUART_DEBUG_TIMING_PORT  ) {  \
			hres_event(dbg_strings[eventid], arg1, arg2 );             \
		}

#else
	#define HSUART_LOG(args...)
#endif

 
struct dev_ctxt {
	int           uart_port_number;
	int           uart_speed;
	unsigned int  uart_flags;
	unsigned long is_opened;
	unsigned long is_initilized;

	/* misc char device */
//	struct miscdevice       mdev;
//	struct file_operations  fops;
	struct platform_device *pdev;
	const char             *dev_name; 
	struct hsuart_platform_data *pdata;
	spinlock_t              lock;
	struct mutex            rx_mlock;
	struct mutex            tx_mlock;

	/*
	 * Platform hsuart context ID
	 */
	int hsuart_id;

	/* requirements for Rx Tx buffers */
	int   tx_buf_size;
	int   tx_buf_num;      
	int   rx_buf_size;     
	int   rx_buf_num;

	/* RX DMA buffer information */
	struct buffer_item	rx;

	/* TX DMA buffer information */
	struct buffer_item	tx;


	/*
	 * Rx related buffer management lists.
	 */
	struct rxtx_lists		rx_lists;

	/*
	 * Tx related buffer management lists.
	 */
	struct rxtx_lists		tx_lists;

	wait_queue_head_t		got_rx_buffer;
	wait_queue_head_t		got_tx_buffer;

	int 				min_packet_sz;

	/*
	 * stats
	 */
	unsigned long  			rx_ttl;
	unsigned long  			rx_dropped;
	unsigned long  			tx_ttl;

	struct tty_struct *tty;
};

struct dev_ctxt tty_info[1];

/*
 * Event loggin support.
 */
/*
static inline void
log_txrx_event(u32 type, u32 arg1, u32 arg2)
{
	if(unlikely(type >= ARRAY_SIZE(event_names)))
		return;
	if(event_log_mask & (1 << type)) {
		hres_event((char*)event_names[type], arg1, arg2 );
	}
}
*/


/*
 * Sysfs area
 */
static ssize_t 
debug_lvl_show(struct device *dev, 
               struct device_attribute *attr, 
               char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%x\n", 
	                _dbg_lvl_);
}

static ssize_t 
debug_lvl_store(struct device *dev,
             struct device_attribute *attr,
             const char *buf, size_t count)
{
	_dbg_lvl_ = simple_strtol(buf, NULL, 10);
	return count;
}


static DEVICE_ATTR(dbg_lvl, S_IRUGO | S_IWUSR, debug_lvl_show, debug_lvl_store);




/*
 *	RxTx Buffer Management logic
 */

/*
*
* Helper function, allocate the Rx related buffers.
*
* @param[in][out]       io_p_contxt - The device context to use.
*
* @return 0 for success -ENOMEM otherwise.
*
*/
static int 
hsuart_alloc_rx_dma_buf(struct dev_ctxt* io_p_contxt)
{
	int			ret		= 0;

	HSUART_ENTER();

	/*
	 *	1. Calculate the required size for rx dma buffer based on
	 *	inputs from the board file (stored int he context)
	 * 	2. Allocate coherent memory to be used by DMA engine.
	 */
	
	io_p_contxt->rx.size = 	io_p_contxt->rx_buf_num * 
					io_p_contxt->rx_buf_size;

	/* TODO: use the dev instead of NULL */
	io_p_contxt->rx.p_vaddr = dma_alloc_coherent(
				NULL, //&(io_p_contxt->pdev->dev), 
	                        io_p_contxt->rx.size, 
	                        (dma_addr_t *)&(io_p_contxt->rx.phys_addr), 
				GFP_KERNEL );

	if (NULL == io_p_contxt->rx.p_vaddr) {
		ret = -ENOMEM;
		HSUART_ERR("%s:%s, failed allocating virt 0x%x, phys 0x%x size 0x%x\n",
				io_p_contxt->dev_name, 
				__PRETTY_FUNCTION__,
				(unsigned int)io_p_contxt->rx.p_vaddr,
				io_p_contxt->rx.phys_addr,
				io_p_contxt->rx.size);
	}
	else {
		HSUART_DEBUG("%s:%s, allocated virt 0x%x, phys 0x%x size 0x%x\n",
				io_p_contxt->dev_name, 
				__PRETTY_FUNCTION__,
				(unsigned int)io_p_contxt->rx.p_vaddr,
				io_p_contxt->rx.phys_addr,
				io_p_contxt->rx.size);
	}	
	HSUART_EXIT();

	return ret;
}

/*
*
* Helper function, allocate the Tx related buffers.
*
* @param[in][out]       io_p_contxt - The device context to use.
*
* @return 0 for success -ENOMEM otherwise.
*
*/
static int 
hsuart_alloc_tx_dma_buf(struct dev_ctxt* io_p_contxt)
{
	int    ret	= 0;

	HSUART_ENTER();

	/*
	 *	1. Calculate the required size for dma buffer based on inputs 
	 *	from the board file (stored int the context)
	 * 	2. Allocate coherent memory to be used by DMA engine.
	 */
	
	io_p_contxt->tx.size = 	io_p_contxt->tx_buf_num * 
					io_p_contxt->tx_buf_size;

	io_p_contxt->tx.p_vaddr = dma_alloc_coherent(
/* TODO:use the driver instead of NULL */
				NULL, //&(io_p_contxt->pdev->dev), 
	                        io_p_contxt->tx.size, 
	                        (dma_addr_t *)&(io_p_contxt->tx.phys_addr), 
				GFP_KERNEL );

	if (NULL == io_p_contxt->tx.p_vaddr) {
		ret = -ENOMEM;
		HSUART_ERR("%s:%s, failed allocating virt 0x%x, phys 0x%x size 0x%x\n",
				io_p_contxt->dev_name, 
				__PRETTY_FUNCTION__,
				(unsigned int)io_p_contxt->tx.p_vaddr,
				io_p_contxt->tx.phys_addr,
				io_p_contxt->tx.size);
	}
	else {
		HSUART_DEBUG("%s:%s, allocated virt 0x%x, phys 0x%x size 0x%x\n",
				io_p_contxt->dev_name, 
				__PRETTY_FUNCTION__,
				(unsigned int)io_p_contxt->tx.p_vaddr,
				io_p_contxt->tx.phys_addr,
				io_p_contxt->tx.size);
	}	
	HSUART_EXIT();

	return ret;
}

/**
*
* Helper function, used as predicate to determine how many bytes
* can be read

* @param[in][out]       io_p_lists - The lists structure to look 
*				for the buffer in.

* @return 1 in case that bytes are buffered 0 if not and for success; 
* -EINVL in case of error.
*
*/
static int
hsuart_rx_avail(struct rxtx_lists* io_p_lists)
{
	int			ret = 0;
	unsigned long		flags;
	int			empty;
	struct buffer_item*	p_buffer;

	HSUART_ENTER();

	spin_lock_irqsave(&(io_p_lists->lock), flags);

	if (NULL != io_p_lists) {	
		empty = list_empty(&(io_p_lists->full));
		if (!empty) {
			p_buffer = list_first_entry(&(io_p_lists->used),
						    struct buffer_item,
						    list_item);
			ret = p_buffer->fullness;
		}
		else {
			empty = list_empty(&(io_p_lists->used));
			if (!empty) {
				p_buffer = list_first_entry(&(io_p_lists->used),
						    struct buffer_item,
						    list_item);
				if (p_buffer->fullness) {
					ret = p_buffer->fullness;
				}
			}
		}
	}
	else {
		HSUART_ERR("%s: %s, invalid params\n",
			DRIVER_NAME,
			__PRETTY_FUNCTION__);
		ret = 0;
	}

	spin_unlock_irqrestore(&(io_p_lists->lock), flags);
	HSUART_EXIT();
	return ret;
}

/**
*
* Helper function, used as predicate to indicate whether there are 
* buffer read bytes.

* @param[in][out]       io_p_lists - The lists structure to look 
*				for the buffer in.

* @return 1 in case that bytes are buffered 0 if not and for success; 
* -EINVL in case of error.
*
*/
static int
hsuart_rx_data_exist(struct rxtx_lists* io_p_lists)
{
	int			ret = 0;
	unsigned long		flags;
	int			empty;
	struct buffer_item*	p_buffer;

	HSUART_ENTER();

	spin_lock_irqsave(&(io_p_lists->lock), flags);

	if (NULL != io_p_lists) {	
		empty = list_empty(&(io_p_lists->full));
		if (!empty) {
			ret = 1;
		}
		else {
			empty = list_empty(&(io_p_lists->used));
			if (!empty) {
				p_buffer = list_first_entry(&(io_p_lists->used),
						    struct buffer_item,
						    list_item);
				if (p_buffer->fullness) {
					ret = 1;
				}
			}
		}
	}
	else {
		HSUART_ERR("%s: %s, invalid params\n",
			DRIVER_NAME,
			__PRETTY_FUNCTION__);
		ret = -EINVAL;
	}

	spin_unlock_irqrestore(&(io_p_lists->lock), flags);
	HSUART_EXIT();
	return ret;
}



/**
*
* Helper function, predicate that checks if there is at least one empty buffer
*
* @param[in][out]       io_p_lists - The lists structure to look 
*				for the buffer in.
* @param[in]		min_free_bytes - The minimal amount of free bytes 
*				in the buffer.

* @return 1 to indicate that we have a free buffer, 0 in case we don't and
* -ENOMEM in case that we failed.
*
*/
static int
hsuart_vacant_tx_buf_exist(struct rxtx_lists* io_p_lists) 
{
	int			ret		= 0;
	unsigned long		flags;

	HSUART_ENTER();

	if (NULL != io_p_lists) {
		spin_lock_irqsave(&(io_p_lists->lock), flags);

		if (io_p_lists->vacant_buffers >= (io_p_lists->buffer_cnt >>1)){
			ret = 1;
		}
		spin_unlock_irqrestore(&(io_p_lists->lock), flags);
	}
	else {
		HSUART_ERR("%s: %s, invalid params\n",
			DRIVER_NAME,
			__PRETTY_FUNCTION__);
		ret = -EINVAL;
	}
	HSUART_EXIT();

	return ret;	
}
/**
*
* Helper function, predicate that checks if there are no queued tx/rx buffers.
*
* @param[in][out]       i_io_p_lists - The lists structure to check

* @return 1 to indicate that the list is empty, 0 otherwise.
*
*/

static int
hsuart_rxtx_is_empty(struct rxtx_lists* i_p_lists)
{
	int			ret 		= 0;
	unsigned long		flags;
	int 			empty;
	struct	buffer_item* 	p_buffer 	= NULL;

	HSUART_ENTER();

	spin_lock_irqsave(&(i_p_lists->lock), flags);

	/*
	 * First case, all the buffers are deposited in the 'empty' list.
	 */
	if(i_p_lists->vacant_buffers == i_p_lists->buffer_cnt) {
		ret = 1;
	}
	/*
	 * Second case, all the buffers but one are in the empty list,
	 * and the buffer that is in the used list (pushed to the lower level
	 * driver) is empty.
	 */
	else if (i_p_lists->vacant_buffers == (i_p_lists->buffer_cnt-1)){
		empty = list_empty(&(i_p_lists->used));
		if (!empty) {
			p_buffer = list_first_entry(&(i_p_lists->used),
						    struct buffer_item,
						    list_item);
			if (0 == p_buffer->fullness) {
				ret = 1;
			}
		}		
	}
	spin_unlock_irqrestore(&(i_p_lists->lock), flags);

	HSUART_EXIT();
	return ret;
}



/**
*
* Helper function, returns the next 'full' buffer (i.e. buffer with data)
* As the current implementation of the lists is FIFO, the function
* will first try to check whether there is a buffer in the 'full' list
* and if so return the first (head) of he list, otherwise, it will check
* the 'used' list and if there is buffer, it will be returned
*
* @param[in][out]       io_p_lists - The lists structure to look 
*				for the buffer in.
  @param[out]		o_pp_buffer - Pointer to container to fill 
*				with pointer to the buffer, in case of 
*				failure to find empty buffer, will be 
*				set to NULL.
*
* @return 0 for success; -ENOMEM in case that we failed to find a 
*	matching buffer.
*
*
*/
static int
__hsuart_rxtx_get_next_full_buf(struct rxtx_lists* io_p_lists, 
			  struct buffer_item** o_pp_buffer)
{
	int			ret = 0;
	int			empty;
	struct buffer_item*	p_buffer = NULL;

	HSUART_ENTER();

	if ((NULL != io_p_lists) && (NULL != o_pp_buffer)) {	
		empty = list_empty(&(io_p_lists->full));
		if (empty) {
			empty = list_empty(&(io_p_lists->used));
			if (!empty) {
				p_buffer = list_first_entry(&(io_p_lists->used), 
							    struct buffer_item, 
							    list_item);
			}
//			else {
//				panic("%s\n %d",__FUNCTION__, __LINE__);
//			}
		}
		else {
			p_buffer = list_first_entry(&(io_p_lists->full),
						    struct buffer_item,
						    list_item);
		}

		if (NULL == p_buffer) {
			HSUART_DEBUG("%s: %s, can't find full buffer, empty %d.\n",
				DRIVER_NAME,
				__PRETTY_FUNCTION__,
				empty);
			ret = -ENOMEM;
		}
	}
	else {
		HSUART_ERR("%s: %s, invalid params\n",
			DRIVER_NAME,
			__PRETTY_FUNCTION__);
		ret = -EINVAL;
	}

	if (!ret) {
		(*o_pp_buffer) = p_buffer;
		
		/*
		 * Remove the buffer from the list it used to be in.
		 */
		list_del(&(p_buffer->list_item));

		HSUART_DEBUG("%s: %s, p_buffer 0x%x\n",
				DRIVER_NAME,
				__FUNCTION__,
				(unsigned int)p_buffer);
	}

	HSUART_EXIT();
	return ret;
}

static int
hsuart_rxtx_get_next_full_buf(struct rxtx_lists* io_p_lists, 
			  struct buffer_item** o_pp_buffer)
{
	int			ret = 0;
	unsigned long		flags;

	spin_lock_irqsave(&(io_p_lists->lock), flags);
	
	ret = __hsuart_rxtx_get_next_full_buf(io_p_lists, o_pp_buffer);

	spin_unlock_irqrestore(&(io_p_lists->lock), flags);

	return ret;
}

/**
*
* Helper function, returns the next buffer which has at least the 
* specified vacancy.
* As the current implementation of the lists is FIFO, the function
* will first try to check whether the buffer in the 'used' list exist
* and if so, check whether it has enough free space.
* In case that there is no 'used' buffer the function will get the 
* first buffer from the 'empty' list
*
* @param[in][out]       io_p_lists - The lists structure to look 
*				for the buffer in.
* @param[in]		min_free_bytes - The minimal amount of free bytes 
*				in the buffer.
* @param[out]		o_pp_buffer - Pointer to container to fill 
*				with pointer to the buffer, in case of 
*				failure to find empty buffer, will be 
*				set to NULL.
*
* @return 0 for success; -ENOMEM in case that we failed to find a 
*	matching buffer.
*
*
*/
static int
hsuart_tx_buf_get_empty(struct rxtx_lists* io_p_lists, 
			  int min_free_bytes, 
			  struct buffer_item** o_pp_buffer)
{
	struct buffer_item*	p_buffer	= NULL;
	int			ret		= 0;
	int			empty;
	unsigned long		flags;

	HSUART_ENTER();

	if ((NULL != io_p_lists) && (NULL != o_pp_buffer)) {
		spin_lock_irqsave(&(io_p_lists->lock), flags);
/*
 * TODO: optimize by adding support for 'used' buffers when writing.
		empty = list_empty(&(io_p_lists->used));
		if (!empty) {
			p_buffer = list_first_entry(&(io_p_lists->used),
						    struct buffer_item,
						    list_item);
			list_del(&(p_buffer->list_item));
			if (min_free_bytes > (p_buffer->size - p_buffer->fullness)) {
panic("%s %d\n",__FUNCTION__, __LINE__);
				list_add_tail(&(p_buffer->list_item),
					      &(io_p_lists->full));
				p_buffer = NULL;
			}
		}
*/		
		if (NULL == p_buffer) {
			empty = list_empty(&(io_p_lists->empty));
			if (!empty) {
				p_buffer = list_first_entry(&(io_p_lists->empty), 
							    struct buffer_item, 
							    list_item);
				list_del(&(p_buffer->list_item));
				io_p_lists->vacant_buffers--;
				BUG_ON(io_p_lists->vacant_buffers < 0);
				BUG_ON(io_p_lists->vacant_buffers > io_p_lists->buffer_cnt);
			}
		}

		if (NULL == p_buffer) {
			HSUART_ERR("%s: %s, can't find empty buffer, empty%d\n",
				DRIVER_NAME,
				__PRETTY_FUNCTION__,
				empty);
			
			ret = -ENOMEM;
		}
		else {
			(*o_pp_buffer) = p_buffer;
		}
		spin_unlock_irqrestore(&(io_p_lists->lock), flags);
	}
	else {
		HSUART_ERR("%s: %s, invalid params\n",
			DRIVER_NAME,
			__PRETTY_FUNCTION__);
		ret = -EINVAL;
	}
	HSUART_EXIT();

	return ret;	
}

/**
*
* Helper function, returns the next buffer which has at least the 
* specified vacancy.
* Will try to first find a buffer in the empty list and if failed
* reuse the buffer in the used list
*
* @param[in][out]       io_p_lists - The lists structure to look 
*				for the buffer in.
* @param[in]		min_free_bytes - The minimal amount of free bytes 
*				in the buffer.
* @param[out]		o_pp_buffer - Pointer to container to fill 
*				with pointer to the buffer, in case of 
*				failure to find empty buffer, will be 
*				set to NULL.
*
* @return 0 for success; -ENOMEM in case that we failed to find a 
*	matching buffer.
*
*
*/
static int
hsuart_rx_buf_get_empty(struct rxtx_lists* io_p_lists, 
			  int min_free_bytes, 
			  struct buffer_item** o_pp_buffer)
{
	struct buffer_item*	p_buffer	= NULL;
	int			ret		= 0;
	int			empty;
	int			item_from_empty = 0;

	HSUART_ENTER();
	if ((NULL != io_p_lists) && (NULL != o_pp_buffer)) {
		empty = list_empty(&(io_p_lists->empty));
		if (empty) {
			empty = list_empty(&(io_p_lists->used));
			if (!empty) {
				p_buffer = list_first_entry(&(io_p_lists->used), 
							    struct buffer_item, 
							    list_item);
			}
		}
		else {
			p_buffer = list_first_entry(&(io_p_lists->empty),
						    struct buffer_item,
						    list_item);
			item_from_empty = 1;
		}

		if (empty || (min_free_bytes > (p_buffer->size - p_buffer->fullness))) {
			HSUART_ERR("%s: %s, can't find empty buffer, empty%d, req_size%d, sz%d, fullness%d\n",
				DRIVER_NAME,
				__PRETTY_FUNCTION__,
				empty,
				min_free_bytes,
				p_buffer ? p_buffer->size : 666,
				p_buffer ? p_buffer->fullness: 666);			
			ret = -ENOMEM;
		}
		else {
			(*o_pp_buffer) = p_buffer;
			/*
			 * Remove the buffer  from the list it used to be in.
			 */
			list_del(&(p_buffer->list_item));
			if (item_from_empty) {
				io_p_lists->vacant_buffers--;
				BUG_ON(io_p_lists->vacant_buffers < 0);
				BUG_ON(io_p_lists->vacant_buffers > io_p_lists->buffer_cnt);
			}
		}

		if (ret) {
			(*o_pp_buffer) = NULL;
		}
	}
	else {
		HSUART_ERR("%s: %s, invalid params\n",
			DRIVER_NAME,
			__PRETTY_FUNCTION__);
		ret = -EINVAL;
	}
	HSUART_EXIT();

	return ret;	
}


/**
*
* Helper function, initialize the rxtx_lists structure:
* - initialize the lists within the rxtx_lists structure, 
* - init the lock protecting the lists, 
* - allocate and initialize the nodes pointing to the chunks of memory
*   that are served as buffers for rx/tx.
*
* @param[in][out]       io_p_lists - The lists structure to initialize.
* @param[in]		num_buffers - The number of buffers (memory chunks)
*				to initialize.
* @param[in]		buffer_size - The size (in bytes) of a buffer.
* @param[in]		phys_addr_start - The start address (physical) of the 
*				buffer to be managed by the rxtx_list we are 
*				about to init in this function.
* @param[in]		i_p_vaddr_start - The start address (virtual) of the 
*				buffer to be managed by the rxtx_list we are 
*				about to initialize in this function
*
* @return 0 for success; -ENOMEM in case that we failed to allocate memory.
*
*/
static int
hsuart_rxtx_lists_init(	struct rxtx_lists*	io_p_lists, 
			int			num_buffers, 
			int			buffer_size,
			dma_addr_t		phys_addr_start,
			char*			i_p_vaddr_start)
{
	int			ret = 0;
	struct buffer_item*	p_buffer = NULL;
	int			i;

	/*
	 * Initialize Rx lists:
	 * - Init the spin lock
	 * - set full and used buffer list to be empty.
	 * - set empty buffer list to hold all the allocated buffers.
	 */
	spin_lock_init(&(io_p_lists->lock));

	INIT_LIST_HEAD(&(io_p_lists->full));
	INIT_LIST_HEAD(&(io_p_lists->empty));
	INIT_LIST_HEAD(&(io_p_lists->used));

	io_p_lists->p_buffer_pool = 
			kzalloc(num_buffers * sizeof(struct buffer_item), 
				GFP_KERNEL);

	if (NULL == io_p_lists->p_buffer_pool) {
		ret = -ENOMEM;
		HSUART_ERR("%s:%s, failed allocating buffer pool, size %d\n",
			DRIVER_NAME, 
			__PRETTY_FUNCTION__,
			num_buffers * sizeof(struct buffer_item));
	}
	else {
		HSUART_DEBUG("%s:%s, allocated buff 0x%x\n",
			DRIVER_NAME, 
			__PRETTY_FUNCTION__,
			(unsigned int)io_p_lists->p_buffer_pool);
	}

	/*
	 * It's all free.
	 */
	io_p_lists->vacant_buffers	= num_buffers;
	io_p_lists->buffer_cnt		= num_buffers;

	p_buffer = io_p_lists->p_buffer_pool;
	for (i = 0; i < num_buffers; i++) {
		INIT_LIST_HEAD(&(p_buffer->list_item));
		p_buffer->size		= buffer_size;
		p_buffer->read_index	= 0;
		p_buffer->write_index	= 0;
		p_buffer->fullness	= 0;
		p_buffer->phys_addr	= phys_addr_start + (i * buffer_size);
		p_buffer->p_vaddr	= i_p_vaddr_start + (i * buffer_size);

		list_add_tail(&(p_buffer->list_item), &(io_p_lists->empty));
		/*
		 * Point to the next element in the buffer pool
		 */
		p_buffer++;
	}
	return ret;
}


/**
*
* Constructor, creates, allocate and initialize the RxTx related buffers.
*
* @param[in][out]       io_p_contxt - The device context to use.
*
* @return 0 for success; -ENOMEM in case that we can't allocate buffers.
*
*/
static int
hsuart_rxtx_buf_init(struct dev_ctxt* io_p_contxt)
{
	int			ret = 0;

	HSUART_ENTER();

	ret = hsuart_alloc_rx_dma_buf(io_p_contxt);
	if (0 == ret) {
		ret = hsuart_rxtx_lists_init(
					&(io_p_contxt->rx_lists),
					io_p_contxt->rx_buf_num,
					io_p_contxt->rx_buf_size,
					io_p_contxt->rx.phys_addr,
					io_p_contxt->rx.p_vaddr);
	}
	if (0 == ret) {
		ret = hsuart_alloc_tx_dma_buf(io_p_contxt);
	}
	if (0 == ret) {
		ret = hsuart_rxtx_lists_init(
					&(io_p_contxt->tx_lists),
					io_p_contxt->tx_buf_num,
					io_p_contxt->tx_buf_size,
					io_p_contxt->tx.phys_addr,
					io_p_contxt->tx.p_vaddr);
	}
	
	/*
	 * TODO: add proper cleanup in case of failure.
	 */
	HSUART_EXIT();

	return ret;	
}

static struct buffer_item*
__rx_get_buffer_cbk(void* p_data, int free_bytes)
{
	struct dev_ctxt*	p_context; 
	int			empty		= 0;
	int			err;
	unsigned long		flags;
	struct buffer_item*	p_buffer	= NULL;
	/*
	 * In case that we will reuse an existing driver, we will use this
	 * var to register how many bytes were "dropped".
	 */
	int			dropped = 0;

	p_context = (struct dev_ctxt*)p_data;

	HSUART_ENTER();
	if (NULL != p_context) {
		HSUART_LOG(p_context, HS_UART_GB_ENT, 0 , 0 ); 
		/*
		 * Find a free buffer and ask the platform-specific hsuart
		 * code to fill it.
		 */
		spin_lock_irqsave(&(p_context->rx_lists.lock), flags);

		/* TODO: handle the case that free_bytes is large number, more
			than the buffer size.....*/
		err = hsuart_rx_buf_get_empty(&(p_context->rx_lists), 
						free_bytes, 
						&p_buffer);
		/*
		 * If we failed to get vacant buffer, reuse the last one
		 */
		if ((!p_buffer) || err) {
			int available_bytes;
			int required_bytes;

			empty = list_empty(&(p_context->rx_lists.used));
			if (!empty) {
				p_buffer = list_first_entry(
						&(p_context->rx_lists.used),
						struct buffer_item, 
						list_item);
			}
			else {
				panic("%s line %d, err %d\n",__FUNCTION__, __LINE__, err);
			}
			list_del(&(p_buffer->list_item));

			if ( p_buffer->size < free_bytes ) {
				panic("%s:%d, not enough bytes to reuse %d %d\n"
					,__FUNCTION__,
					__LINE__,
					p_buffer->size,
					free_bytes);			
			}

			available_bytes = p_buffer->size - p_buffer->fullness;

			if (  free_bytes > available_bytes ) {
				required_bytes = free_bytes - available_bytes;
				p_buffer->fullness -= required_bytes;
				dropped = required_bytes;
				p_buffer->write_index -= required_bytes;
			}
		}
		spin_unlock_irqrestore(&(p_context->rx_lists.lock), flags);

		spin_lock_irqsave(&(p_context->lock), flags);
		p_context->rx_dropped += dropped;
		spin_unlock_irqrestore(&(p_context->lock), flags);

		HSUART_DEBUG("%s: %s, got new buffer? - %s 0x%x\n",
				p_context->dev_name,
				__PRETTY_FUNCTION__,
				empty? "false": "true",
				p_buffer ? (uint32_t)p_buffer: 0x666);
		if(p_buffer){
//printk(KERN_ERR"%s p_buffer 0x%x read_index %d write_index %d fullness %d\n", __FUNCTION__,(uint32_t)p_buffer, p_buffer->read_index, p_buffer->write_index, p_buffer->fullness);
		}
		else {
			panic("%s %d\n",__FUNCTION__, __LINE__);
		}

		HSUART_LOG(p_context, HS_UART_GB_EXT, p_buffer->fullness , p_buffer->size ); 
	}

	HSUART_DEBUG("%s: %s, exit, p_buffer 0x%x, p_data 0x%x\n",
			DRIVER_NAME,
			__PRETTY_FUNCTION__,
			(uint32_t)p_buffer,
			(uint32_t)p_data);

	return p_buffer;
}
static void
__rx_put_buffer_cbk(void* p_data, struct buffer_item* p_buffer)
{
	struct dev_ctxt*	p_context; 
	int			empty;
	int			fullness;
	struct buffer_item*	p_last_buffer = NULL;
	unsigned long		flags;


	p_context = (struct dev_ctxt*)p_data;
	
	HSUART_DEBUG("%s: %s, enter, p_buffer 0x%x, p_data 0x%x\n",
		DRIVER_NAME,
		__PRETTY_FUNCTION__,
		(uint32_t)p_buffer,
		(uint32_t)p_data);
//printk(KERN_ERR"%s p_buffer 0x%x read_index %d write_index %d fullness %d\n", __FUNCTION__,(uint32_t)p_buffer, p_buffer->read_index, p_buffer->write_index, p_buffer->fullness);

	if ((NULL != p_context) && (NULL != p_buffer)) {
		HSUART_LOG(p_context, HS_UART_PB_ENT, p_buffer->fullness , 0 ); 

		spin_lock_irqsave(&(p_context->lock), flags);
		p_context->rx_ttl += p_buffer->fullness;
		spin_unlock_irqrestore(&(p_context->lock), flags);
		/*
		 * Check the used list, if it is not empty, move
		 * the buffer from it to the full list.
		 * Put the new buffer into the tail of the used list.
		 */
		spin_lock_irqsave(&(p_context->rx_lists.lock), flags);
		fullness = p_buffer->fullness;

		if (fullness) {
			empty = list_empty(&(p_context->rx_lists.used));
			if (!empty) {
				p_last_buffer = list_first_entry(&(p_context->rx_lists.used),
							    struct buffer_item,
							    list_item);
				list_del(&(p_last_buffer->list_item));
				BUG_ON(NULL == p_last_buffer);
				HSUART_DEBUG("we have buffer 0x%x in the used list, push it to full\n",(uint32_t)p_last_buffer);
				list_add_tail(&(p_last_buffer->list_item), &(p_context->rx_lists.full));
			}

			list_add_tail(&(p_buffer->list_item), &(p_context->rx_lists.used));
		}

		else {
			list_add_tail(&(p_buffer->list_item), &(p_context->rx_lists.empty));
			p_context->rx_lists.vacant_buffers++;
		}

		spin_unlock_irqrestore(&(p_context->rx_lists.lock), flags);

		if (fullness)
			wake_up_interruptible(&(p_context->got_rx_buffer));

		HSUART_LOG(p_context, HS_UART_PB_EXT, 0 , 0 ); 
	}
	else {
		HSUART_ERR("%s: %s, invalid parameters p_context 0x%x, p_buffer 0x%x!!!\n",
				DRIVER_NAME,
				__PRETTY_FUNCTION__,
				(uint32_t) p_context,
				(uint32_t) p_buffer);
	}

	HSUART_EXIT();
}

static struct buffer_item*
__tx_get_buffer_cbk(void* p_data )
{

	struct dev_ctxt*	p_context;
	int			ret;
	struct buffer_item*	p_buffer = NULL;
	unsigned long flags;

	p_context = (struct dev_ctxt*)p_data;

	HSUART_LOG(p_context, HS_UART_TX_GET_BUFF_EVT, 0 , 0 );

	spin_lock_irqsave(&(p_context->tx_lists.lock), flags);
	ret = __hsuart_rxtx_get_next_full_buf(&(p_context->tx_lists), &p_buffer);
	spin_unlock_irqrestore(&(p_context->tx_lists.lock), flags);

	HSUART_LOG(p_context, HS_UART_TX_GET_BUFF_EVT, 1 , ret );

	return p_buffer;
}
static void
__tx_put_buffer_cbk(void* p_data, struct buffer_item* p_buffer, int transaction_size)
{
	struct dev_ctxt*	p_context;
	unsigned long		flags;
	HSUART_ENTER();

	p_context = (struct dev_ctxt*)p_data;

	HSUART_LOG(p_context, HS_UART_TX_PUT_BUFF_EVT, 0 , transaction_size ); 

	spin_lock_irqsave(&(p_context->lock), flags);
	p_context->tx_ttl += transaction_size;
	spin_unlock_irqrestore(&(p_context->lock), flags);

	spin_lock_irqsave(&(p_context->tx_lists.lock), flags);	
	/* 
		* No more data in the buffer...add it to the empty list
		*/
	list_add(&(p_buffer->list_item), 
			&p_context->tx_lists.empty);
	p_context->tx_lists.vacant_buffers++;
	BUG_ON(p_context->tx_lists.vacant_buffers < 0);
	BUG_ON(p_context->tx_lists.vacant_buffers > p_context->tx_lists.buffer_cnt);

	spin_unlock_irqrestore(&(p_context->tx_lists.lock), 
				flags);

	HSUART_LOG(p_context, HS_UART_TX_PUT_BUFF_EVT, 1 , 0 ); 

	HSUART_EXIT();
}

/**
*
* Helper function, allocate and initialize the HSUART port.
*
* @param[in][out]       io_p_contxt - The device context to use.
*
* @return 0 for success -1 otherwise.
*
* @Note			call the platform specific function to open a uart 
*			port and set it up with the default configuration.
*
*/
static int 
hsuart_uart_port_init(struct dev_ctxt* io_p_contxt)
{
	int				            ret             = 0;
	struct hsuart_config		cfg             = {0};
	struct buffer_item*	    	p_buffer        = NULL;
	int                         max_packet_size = io_p_contxt->pdata->max_packet_size;

	HSUART_ENTER();

	/*
	 * TODO:amir
	 * Right now, we call the MSM specific code, need to change it so it will 
	 * call virtual function which will be filled in the board file.
	 */
	/*
	 * Get the desired UART port ID from the context into 
	 * the configuration request.
	 */
	cfg.port_id = io_p_contxt->uart_port_number;

	if(io_p_contxt->pdata->options & HSUART_OPTION_TX_PIO)
	{
		cfg.flags |= HSUART_CFG_TX_PIO;
	}

	if(io_p_contxt->pdata->options & HSUART_OPTION_RX_PIO)
	{
		cfg.flags |= HSUART_CFG_RX_PIO;
	}

	if(io_p_contxt->pdata->options & HSUART_OPTION_TX_DM)
	{
		cfg.flags |= HSUART_CFG_TX_DM;
	}

	if(io_p_contxt->pdata->options & HSUART_OPTION_RX_DM)
	{
		cfg.flags |= HSUART_CFG_RX_DM;
	}

	if(io_p_contxt->pdata->options & HSUART_OPTION_SCHED_RT)
	{
		cfg.flags |= HSUART_CFG_SCHED_RT;
	}

	cfg.rx_get_buffer.p_cbk  = __rx_get_buffer_cbk;		
	cfg.rx_get_buffer.p_data = io_p_contxt;
	cfg.rx_put_buffer.p_cbk  = __rx_put_buffer_cbk;
	cfg.rx_put_buffer.p_data = io_p_contxt;

	cfg.tx_get_buffer.p_cbk  = __tx_get_buffer_cbk;		
	cfg.tx_get_buffer.p_data = io_p_contxt;
	cfg.tx_put_buffer.p_cbk  = __tx_put_buffer_cbk;
	cfg.tx_put_buffer.p_data = io_p_contxt;

	cfg.max_packet_size    = max_packet_size;
	cfg.min_packet_size    = io_p_contxt->pdata->min_packet_size;
	cfg.rx_latency         = io_p_contxt->pdata->rx_latency;

	cfg.p_board_pin_mux_cb          = io_p_contxt->pdata->p_board_pin_mux_cb;
	cfg.p_board_gsbi_config_cb      = io_p_contxt->pdata->p_board_config_gsbi_cb;
	cfg.p_board_rts_pin_deassert_cb = io_p_contxt->pdata->p_board_rts_pin_deassert_cb;


	ret = msm_hsuart_open_context(&cfg, &(io_p_contxt->hsuart_id));
	/*TODO: consider flushing the existing fifo*/
	HSUART_DEBUG("%s: %s, allocated platform hsuart, handle_0x%x\n",
			io_p_contxt->dev_name, 
			__PRETTY_FUNCTION__, 
			(unsigned int)io_p_contxt->hsuart_id);

	msm_hsuart_set_flow(
		io_p_contxt->hsuart_id, 
		io_p_contxt->uart_flags & HSUART_MODE_FLOW_CTRL_MASK);

	msm_hsuart_set_parity(
		io_p_contxt->hsuart_id, 
		io_p_contxt->uart_flags & HSUART_MODE_PARITY_MASK);

	msm_hsuart_set_baud_rate(
		io_p_contxt->hsuart_id,
		io_p_contxt->uart_speed);
	/*
	 * Start the reader, to make sure we start listening on the port
	 * and collect all incoming data.
	 */
	p_buffer = __rx_get_buffer_cbk(io_p_contxt, max_packet_size);

	//TODO: should be called enable/initiate read or something like that
	msm_hsuart_read(io_p_contxt->hsuart_id, p_buffer);
	
	HSUART_EXIT();

	return ret;
}

/**
*
* Helper function, release allocated handle of the HSUART port.
*
* @param[in][out]       io_p_contxt - The device context to use.
*
* @return 0 for success -1 otherwise.
*
* @Note			call the platform specific function to close uart 
*			port.
*/
static int 
hsuart_uart_port_release(struct dev_ctxt* io_p_contxt)
{
	int    ret	= 0;

	HSUART_ENTER();

	ret = msm_hsuart_close_context(io_p_contxt->hsuart_id);

	HSUART_EXIT();

	return ret;
}


/************************************************************************
 *
 *	 IO calls
 *
 ************************************************************************/
/*
*
* Helper function, copy from the specified buffer_item to user-space pointer.
*
* @param[in][out]       io_p_buffer	- The buffer to copy from.
* @param[out]		o_p_buf		- The destination buffer.
* @param[in]		count		- The number of bytes to copy.
*
* @return	negative number - error code, 
*		0 or positive - the number of bytes that we copied.
*
*/
static ssize_t
hsuart_copy_buf_to_user(struct buffer_item* io_p_buffer,
			char* o_p_buf, int count)
{
	int			ret		= 0;
	int			ret_cnt		= 0;
	struct buffer_item*	p_buffer	= io_p_buffer;
	HSUART_ENTER();

	BUG_ON(count > p_buffer->fullness);
	if (!p_buffer->fullness) {
		ret_cnt = 0;
	}
	else if (p_buffer->read_index < p_buffer->write_index) {
		HSUART_DEBUG("%s: %s, about to copy %d bytes read_index %d to 0x%x\n",
			DRIVER_NAME,
			__PRETTY_FUNCTION__,
			count,
			p_buffer->read_index,
			(unsigned int)o_p_buf);

		ret = memcpy(
			o_p_buf, 
			&(p_buffer->p_vaddr[p_buffer->read_index]),
			count)==0;
		if (ret) {
			HSUART_ERR("%s: %s, failed copying data to user err 0x%x line %d.\n",
			    DRIVER_NAME, 
			    __PRETTY_FUNCTION__, 
			    __LINE__,
			    ret);
		}
		else {
			ret_cnt = count;
			p_buffer->fullness -= ret_cnt;
			p_buffer->read_index += ret_cnt;
			if (p_buffer->read_index >= p_buffer->size) {
				p_buffer->read_index -= p_buffer->size;
			}
		}
	}
	else {
		/*
		 * In this case, we have chunks to copy....
		 */
		int length1;
		int length2;

		length1 = p_buffer->size - p_buffer->read_index;
		length2 = p_buffer->write_index;

		/*
		 * In case we need to read from both chunks, make sure
		 * to adjust the number of bytes to read from the 2nd 
		 * chunk, to account the data we read in the first.
		 */
		if (count > length1) {
			HSUART_DEBUG("%s: %s, about to copy %d bytes read_index %d to 0x%x",
				DRIVER_NAME,
				__PRETTY_FUNCTION__,
				length1,
				p_buffer->read_index,
				(unsigned int)o_p_buf);

			length2 = (count - length1);

			ret = memcpy(
				o_p_buf, 
				&p_buffer->p_vaddr[p_buffer->read_index],
				length1)==0;
			if (!ret) {
				ret_cnt += length1;
				p_buffer->fullness -= ret_cnt;
				p_buffer->read_index += ret_cnt;
				if (p_buffer->read_index >= p_buffer->size) {
					p_buffer->read_index -= p_buffer->size;
				}

				HSUART_DEBUG("%s: %s, about to copy %d bytes read_index %d to 0x%x",
					DRIVER_NAME,
					__PRETTY_FUNCTION__,
					length2,
					p_buffer->read_index,
					(unsigned int)o_p_buf);

				ret = copy_to_user(
					o_p_buf + length1, 
					&p_buffer->p_vaddr[0],
					length2);
				if (!ret) {
					ret_cnt += length2;
					p_buffer->fullness -= length2;
					p_buffer->read_index += length2;
					if (p_buffer->read_index >= p_buffer->size) {
						p_buffer->read_index -= p_buffer->size;
					}
				}
			}
		}
		/*
		 * In case we need to read only one chunk, make sure 
		 * we don't read more than requested.
		 */
		else {
			HSUART_DEBUG("%s: %s, about to copy %d bytes read_index %d to 0x%x",
				DRIVER_NAME,
				__PRETTY_FUNCTION__,
				length1,
				p_buffer->read_index,
				(unsigned int)o_p_buf);

			length1 = count;
	
			ret = memcpy(
				o_p_buf, 
				&p_buffer->p_vaddr[p_buffer->read_index],
				length1)==0;
			ret_cnt = length1;
			p_buffer->fullness -= ret_cnt;
			p_buffer->read_index += ret_cnt;
			if (p_buffer->read_index >= p_buffer->size) {
				p_buffer->read_index -= p_buffer->size;
			}
		}
		if (ret) {
			HSUART_ERR("%s: %s, failed copying data to user err 0x%x line %d.\n",
			    DRIVER_NAME, 
			    __PRETTY_FUNCTION__, 
			    (unsigned int)__LINE__,
			    ret);
		}
	}
	
	/*
	 * In case we didnt face any error, return the number of bytes copied.
	 */
	if (!ret) {
		ret = ret_cnt;
	}
	HSUART_EXIT();
	return ret;
}
static int
hsuart_read(struct dev_ctxt* p_contxt, const unsigned char *buf, int count)
{
	unsigned long		flags;
	int			ret;
	struct buffer_item*	p_buffer = NULL;
	int			copied_cnt = 0;

//	p_contxt = container_of(file->f_op, struct dev_ctxt, fops);

	HSUART_ENTER();
	HSUART_DEBUG("%s: %s called, count %d\n",
			p_contxt->dev_name, __PRETTY_FUNCTION__, count);

	HSUART_LOG(p_contxt, HS_UART_READ_ENT, count , 0 ); 

	for (;count > copied_cnt;) {
		int			bytes_to_copy;
		/*
		 * Try to get the next buffer which contains data.
		 */
		HSUART_DEBUG("%s, count %d, copy_cnt %d\n",__FUNCTION__, count, copied_cnt);
		ret = hsuart_rxtx_get_next_full_buf(&(p_contxt->rx_lists),
						       &p_buffer);
		HSUART_DEBUG("ret %d, pbuffer0x%x\n",ret, (unsigned int)p_buffer);
		if ((0 == ret) && (NULL != p_buffer)) {
			bytes_to_copy = min(p_buffer->fullness, (int)(count - copied_cnt)); 

//			printk("Going to read, fullness %d, count %d, copied %d\n", p_buffer->fullness, count ,copied_cnt);
			ret = hsuart_copy_buf_to_user(
						p_buffer, 
						buf + copied_cnt, 
						bytes_to_copy);
			if (0 <= ret) {
				BUG_ON(ret != bytes_to_copy);
				copied_cnt += ret;
			}
			else {
				HSUART_ERR("%s:%s, copy to user failed p_buffer0x%x, rd_idx0x%x, o_p_buf0x%x, copied_cnt%d\n",
					p_contxt->dev_name, 
					__PRETTY_FUNCTION__,
					(unsigned int)p_buffer,
					(unsigned int)p_buffer->read_index,
					(unsigned int)buf,
					copied_cnt);
				ret = -EFAULT;
				goto done_read_copy;
			}
			/*
			 * The buffer is now empty, move it over to the
			 * empty list.
			 */
			//TODO: push me into a function...
			spin_lock_irqsave(&(p_contxt->rx_lists.lock), flags);

			if (0 == p_buffer->fullness) {
				p_buffer->read_index = 0;
				p_buffer->write_index = 0;
				list_add_tail(&(p_buffer->list_item), 
					      &(p_contxt->rx_lists.empty));
				p_contxt->rx_lists.vacant_buffers++;
			}
			else {
				/*
				 * Put the buffer to the head of the full list
				 */
				list_add(&(p_buffer->list_item), 
					 &p_contxt->rx_lists.full);
			}
			spin_unlock_irqrestore(&(p_contxt->rx_lists.lock), flags);
		}
		else {
			/*
			 * No buffer is available
			 */
			if (0 == copied_cnt) { 
				copied_cnt = -EAGAIN;
			}
			goto done_read_copy;
			ret = wait_event_interruptible(
					p_contxt->got_rx_buffer, 
					(1 == hsuart_rx_data_exist(&(p_contxt->rx_lists))));
			HSUART_DEBUG("%s, got 'got-rx-buffer' event\n",__FUNCTION__);
			if (ret) {
				printk(KERN_ERR"%s %d ret %d\n",__func__, __LINE__, ret);
				// TODO add better cleanup.
				goto done_read_copy;
			}
		}
	}	

done_read_copy:
	HSUART_DEBUG("--->%s exit %d<---\n",__FUNCTION__, copied_cnt);

	HSUART_LOG(p_contxt, HS_UART_READ_EXT, copied_cnt , 0 ); 

	return copied_cnt;
}


/**
*
* Helper function, copy from user-space to the provided buffer_item.
*
* @param[in][out]       io_p_buffer	- The buffer to copy to.
* @param[out]		i_p_buf		- The source buffer.
* @param[in]		count		- The number of bytes to copy.
*
* @return	negative number - error code, 
*		0 or positive - the number of bytes that we copied.
*
*/
static ssize_t
hsuart_copy_user_to_buf(struct buffer_item* io_p_buffer, const char* i_p_buf, int count)
{
	struct buffer_item*	p_buffer	= io_p_buffer;
	int			ret		= 0;
	int			ret_cnt		= 0;
	int			err		= 0;
	int i;

	HSUART_DEBUG("%s: %s called, io_p_buffer 0x%x, i_p_buf 0x%x, count %d\n", 
			DRIVER_NAME, 
			__PRETTY_FUNCTION__, 
			(uint32_t)io_p_buffer,
			(uint32_t)i_p_buf,
			count);
	HSUART_DEBUG("%s: %s called, read_index %d, write_index %d, fullness %d\n",
			DRIVER_NAME,
			__PRETTY_FUNCTION__,
			p_buffer->read_index,
			p_buffer->write_index,
			p_buffer->fullness);

	BUG_ON(NULL == io_p_buffer);
	BUG_ON(NULL == i_p_buf);

#if 0

	printk("Wrote: ");
	for(i=0; i < count; i++)
		printk("%2.2X ", i_p_buf[i]);
	printk("\n");
#endif
	/*
	 * Write pointer is smaller than the read pointer, we can copy
	 * the whole thing...
	 */
	if (p_buffer->read_index <= p_buffer->write_index) {
		err = memcpy(
			&(p_buffer->p_vaddr[p_buffer->write_index]), 
			i_p_buf, 
			count)==0;
		if (err) {
			HSUART_ERR("%s:%s, copy from user failed p_vaddr0x%x, write_index0x%x, i_p_buf0x%x, count%d\n",
				DRIVER_NAME, 
				__PRETTY_FUNCTION__,
				(unsigned int)p_buffer->p_vaddr,
				(unsigned int)p_buffer->write_index,
				(unsigned int)i_p_buf,
				count);
			ret = -EFAULT;
			goto done_copy;
		}
		else {
			p_buffer->write_index += count;
			p_buffer->fullness += count;
			ret_cnt += count;
		}
	}
	/*
	 * We may need to copy in two chunks...
	 */
	else {
		int length1 = p_buffer->size - p_buffer->write_index;
		err = memcpy(
			&(p_buffer->p_vaddr[p_buffer->write_index]), 
			i_p_buf, 
			length1)==1;
		if (!err) {
			err = memcpy(
				p_buffer->p_vaddr,
				i_p_buf + length1,
				count - length1)==0;
			if (err) {
				HSUART_ERR("%s:%s, copy from user failed p_vaddr0x%x, write_index0x%x, i_p_buf0x%x, count%d\n",
					DRIVER_NAME, 
					__PRETTY_FUNCTION__,
					(unsigned int)p_buffer->p_vaddr,
					(unsigned int)p_buffer->write_index,
					(unsigned int)i_p_buf + length1,
					count - length1);
				ret = -EFAULT;
				goto done_copy;

			}
			else {
				p_buffer->write_index = count - length1;
				p_buffer->fullness += count;
				ret_cnt += count;
			}
		}
		else {
			HSUART_ERR("%s:%s, copy from user failed p_vaddr0x%x, write_index0x%x, i_p_buf0x%x, count%d\n",
				DRIVER_NAME, 
				__PRETTY_FUNCTION__,
				(unsigned int)p_buffer->p_vaddr,
				(unsigned int)p_buffer->write_index,
				(unsigned int)i_p_buf,
				count);
			ret = -EFAULT;
		}
	}
done_copy:
	if (!ret) {
		ret = ret_cnt;
	}
	HSUART_EXIT();

	return ret;
}

static ssize_t 
hsuart_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
	struct dev_ctxt*	p_context;
	ssize_t			ret		= 0;
	int			copied_cnt	= 0;

	/*
	 * The number of available bytes in the current buffer.
	 */
	int			available_bytes;
	/*
	 * The num of remaining bytes to copy.
	 */
	int			bytes_to_copy	= count;
	int			empty;
	struct buffer_item*	p_buffer	= NULL;
	unsigned long		flags;

//	p_context = container_of(file->f_op, struct dev_ctxt, fops);
	p_context = tty->driver_data;
	HSUART_DEBUG("%s: %s called, count %d\n",
			p_context->dev_name, __PRETTY_FUNCTION__, count);

	HSUART_LOG(p_context, HS_UART_WRITE_ENT, count , 0 ); 

	for (;bytes_to_copy > 0;) {
		int cnt;
		empty = hsuart_tx_buf_get_empty(&(p_context->tx_lists), 0, &p_buffer);
		if ((!empty) && (NULL != p_buffer)) {
			/*
			 * Now, lets copy the data
			 */
			available_bytes = p_buffer->size - p_buffer->fullness;
			cnt = min(bytes_to_copy, available_bytes);

//			printk("Going to write, size %d, fullness %d, count %d, avail %d\n", p_buffer->size, p_buffer->fullness, bytes_to_copy , available_bytes);

			ret = hsuart_copy_user_to_buf(
						p_buffer, 
						buf + copied_cnt, 
						cnt);
			ret = cnt;
			if (ret >= 0) {
				BUG_ON(cnt != ret);
				copied_cnt += ret;
				bytes_to_copy -= ret;
				/*
				 * Place the buffer back into the appropriate list
				 */
				//TODO: push me toa function.
				spin_lock_irqsave(&(p_context->tx_lists.lock), 
						flags);
			/*
			 * future optimization...
				if (p_buffer->fullness == p_buffer->size) {
					printk(KERN_ERR"the buffer is full\n");
					list_add_tail(&(p_buffer->list_item), &(p_contxt->tx_lists.full));
				}
				else {
					printk(KERN_ERR"the buffer is not full yet - move it to used list\n");
					list_add_tail(&(p_buffer->list_item), &(p_contxt->tx_lists.used));
				}
			*/
				list_add_tail(&(p_buffer->list_item), 
						&(p_context->tx_lists.full));
				spin_unlock_irqrestore(&(p_context->tx_lists.lock), 
							flags);

				msm_hsuart_write(p_context->hsuart_id);
			}
			else {
				printk(KERN_ERR"%s, error %d\n",__FUNCTION__, ret);
			}
		}
		else {
			{
				HSUART_DEBUG("%s:%s, no more free space, try again later...\n",
					p_context->dev_name, 
					__PRETTY_FUNCTION__);
				ret = -EAGAIN;
				printk(KERN_ERR"%s, %d, p_buffer 0x%x, empty %d\n",__FUNCTION__, __LINE__, (uint32_t)p_buffer, empty);
				goto hsuart_write_done;
			}
			ret = wait_event_interruptible(
					p_context->got_tx_buffer, 
					hsuart_vacant_tx_buf_exist(&(p_context->tx_lists)));
			if (ret) {
				HSUART_ERR("%s:%s, failed getting got_tx_buffer event\n",
					DRIVER_NAME, 
					__PRETTY_FUNCTION__);
				// TODO: add err handlingand some cleanup
				goto hsuart_write_done;
			}
		}
	}

hsuart_write_done:
	/*
	 * In case that we didn't get any error (ret is not -ErrCode)
	 * assign the number of bytes that were copied as the ret value.
	 */
	if (ret >= 0) {
		ret = copied_cnt;
	}

	HSUART_DEBUG("---> %s ret %d<---\n",__FUNCTION__,ret);

	HSUART_EXIT();
	return ret;
}

static unsigned int 
hsuart_poll(struct file* file, struct poll_table_struct* wait)
{
	unsigned long		flags;
	unsigned int		mask = 0;
	struct dev_ctxt*	p_context;
	int			rx_rdy;
	int			tx_rdy;

	HSUART_ENTER();
//	p_context = container_of(file->f_op, struct dev_ctxt, fops);
	p_context = file->private_data;

	poll_wait(file, &(p_context->got_tx_buffer), wait);
	poll_wait(file, &(p_context->got_rx_buffer), wait);

//FIXME: context lock is useless... need to use lists lock
	spin_lock_irqsave(&(p_context->lock), flags);
	tx_rdy = hsuart_vacant_tx_buf_exist(&(p_context->tx_lists)); 
	if(tx_rdy > 0) {
		mask |= POLLOUT | POLLWRNORM;
	}
	rx_rdy = hsuart_rx_data_exist(&(p_context->rx_lists)); 
	if(rx_rdy > 0) {
		mask |= POLLIN  | POLLRDNORM;
	}

	HSUART_DEBUG("%s, tx_rdy %d, rx_rdy %d\n",__FUNCTION__, tx_rdy, rx_rdy);
//	HSUART_ERR("%s, tx_rdy %d, rx_rdy %d\n",__FUNCTION__, tx_rdy, rx_rdy);

	spin_unlock_irqrestore(&(p_context->lock), flags);

	HSUART_EXIT();

	return mask;

}

static void 
hsuart_flush_rx_queue(struct dev_ctxt* io_p_context)
{
	unsigned long		flags;
	struct buffer_item*	p_buffer	= NULL;
	int			ret		= 0;

	/*
	* Stop RX
	*/
	spin_lock_irqsave(&(io_p_context->rx_lists.lock), flags);

	for (;;) {
		ret = __hsuart_rxtx_get_next_full_buf(
					&(io_p_context->rx_lists),
					&p_buffer);
		HSUART_ERR("%s, %d ret %d, p_buffer 0x%x\n",
			__FUNCTION__, __LINE__,ret, (uint32_t)p_buffer);

		if (ret) {
			break;
		}
		p_buffer->read_index	= 0;
		p_buffer->write_index	= 0;
		p_buffer->fullness	= 0;
		list_add(&(p_buffer->list_item), 
				&(io_p_context->rx_lists.empty));
		io_p_context->rx_lists.vacant_buffers++;
	}

	spin_unlock_irqrestore(&(io_p_context->rx_lists.lock), flags);
}

static void 
hsuart_flush_tx_queue(struct dev_ctxt* io_p_context)
{
	unsigned long		flags;
	struct buffer_item*	p_buffer	= NULL;
	int			ret		= 0;

	/*
	* Disable TX
	*/
	spin_lock_irqsave(&(io_p_context->tx_lists.lock), flags);

	for (;;) {
		ret = __hsuart_rxtx_get_next_full_buf(
					&(io_p_context->tx_lists),
					&p_buffer);
		HSUART_ERR("%s, %d ret %d, p_buffer 0x%x\n",
			__FUNCTION__, __LINE__,ret, (uint32_t)p_buffer);

		if (ret) {
			break;
		}
		p_buffer->read_index	= 0;
		p_buffer->write_index	= 0;
		p_buffer->fullness	= 0;
		list_add(&(p_buffer->list_item), 
				&(io_p_context->tx_lists.empty));
		io_p_context->tx_lists.vacant_buffers++;
	}

	spin_unlock_irqrestore(&(io_p_context->tx_lists.lock), flags);

}

static int
hsuart_ioctl_flush(struct dev_ctxt* io_p_context, int args)
{
	HSUART_DEBUG("%s called, args 0x%x\n",__FUNCTION__, args);

	if (args & (HSUART_TX_QUEUE | HSUART_TX_FIFO)) {

		if(args & HSUART_TX_QUEUE) {
			hsuart_flush_tx_queue(io_p_context);
		}

		if(args & HSUART_TX_FIFO) {
			//TODO: implement
		}
	}
	if (args & (HSUART_RX_QUEUE | HSUART_RX_FIFO)) {

		if (args & HSUART_RX_FIFO) { 
			/* 
			* The followign suspend function flushes teh current rx transaction then
			* waits till the current tx transaction is complete	
			* and then suspends HS UART HW port. Hence tx transactions 
			* are not affected by suspend
			*/
			msm_hsuart_suspend(io_p_context->hsuart_id);
		}

		if (args & HSUART_RX_QUEUE) {
			hsuart_flush_rx_queue(io_p_context);
		}

		if (args & HSUART_RX_FIFO) { 
			msm_hsuart_resume(io_p_context->hsuart_id);
		}

	}

	HSUART_EXIT();
	return 0;
}


static int
hsuart_tx_do_drain(struct dev_ctxt* i_p_context, unsigned long timeout)
{
	int ret = 0;

	if( timeout == 0 ) {
		// non blocking case
		if(!hsuart_rxtx_is_empty(&(i_p_context->tx_lists)))
			ret |= 2; 
	
//		if(!hsuart_tx_fifo_is_empty ( ctxt ))
//			rc |= 1; 
	
		return ret; 
	}
	
	// timeout in jiffies
	timeout = msecs_to_jiffies(timeout);
	
	ret = wait_event_interruptible_timeout( i_p_context->got_tx_buffer,
	                                        hsuart_rxtx_is_empty(&(i_p_context->tx_lists)),
	                                        timeout);
	if (ret < 0) {
		return ret; // interrupted by signal or error
	}
		
	if (ret == 0) {
		return 2;  // expired but condition was not reached
	}
	
	timeout  = jiffies + ret;
	while (time_before(jiffies, timeout)) {
		if (hsuart_rxtx_is_empty(&(i_p_context->tx_lists))) {
			return 0;
		}
		msleep (1);
	}
	return 1;
}

static int
hsuart_ioctl_tx_drain(struct dev_ctxt* i_p_context, unsigned long timeout )
{
	int			ret;
	HSUART_ENTER();
	ret = hsuart_tx_do_drain(i_p_context, timeout);
	HSUART_EXIT_RET(ret);
	return 0;
}

static int
hsuart_ioctl_rx_bytes(struct dev_ctxt* i_p_context)
{
	int		ret = 0;
	int		rx_data_exists = 0;
	HSUART_ENTER();
	
	rx_data_exists = hsuart_rx_data_exist(&(i_p_context->rx_lists));

	if (rx_data_exists) {
		//Buffers have data
		ret |= 1;
	}

	if ( msm_hsuart_rx_fifo_has_bytes(i_p_context->hsuart_id) ){
		//fifo has data
		ret |= 2;
	}

	HSUART_EXIT();
	return ret;
}

static int
hsuart_ioctl_rx_flow ( struct dev_ctxt *ctxt, int opcode )
{
	int ret = 0;

	HSUART_ENTER();

	printk( KERN_ERR "hsuart_ioctl_rx_flow %X\n", opcode );
	
	//Make sure we change rx flow only
 	opcode &= ~(HSUART_MODE_FLOW_DIRECTION_MASK);
	opcode |= HSUART_MODE_FLOW_DIRECTION_RX_ONLY;

	ret = msm_hsuart_set_flow(
		ctxt->hsuart_id, 
		opcode & HSUART_MODE_FLOW_CTRL_MASK);

	HSUART_EXIT();
	return ret;
}


static int
hsuart_ioctl_set_uart_mode(struct dev_ctxt* io_p_context, 
                             void *usr_ptr, int usr_bytes )
{

	int ret = 0;
	unsigned int changed;
	struct hsuart_mode mode;
	
	HSUART_ENTER();
	if( copy_from_user ( &mode, usr_ptr, usr_bytes )) {
		ret = -EFAULT; 
		goto Done;
	}
	printk("%s, speed 0x%x, flags 0x%x\n",
		__FUNCTION__,mode.speed, mode.flags);

	if(mode.speed != io_p_context->uart_speed ) {
		ret = msm_hsuart_set_baud_rate(io_p_context->hsuart_id,
					       mode.speed );
		if(ret != 0){
			goto Done;
		}
		io_p_context->uart_speed = mode.speed;
	}

	changed = io_p_context->uart_flags ^ mode.flags;

	if (changed & HSUART_MODE_FLOW_CTRL_MASK) { 
		/*
		 * flow control changed
		 */
		msm_hsuart_set_flow(io_p_context->hsuart_id, 
				    mode.flags & HSUART_MODE_FLOW_CTRL_MASK);
		io_p_context->uart_flags &= ~HSUART_MODE_FLOW_CTRL_MASK;
		io_p_context->uart_flags |=  
				mode.flags & HSUART_MODE_FLOW_CTRL_MASK;
	}
	
	if(changed & HSUART_MODE_PARITY_MASK) { 
		/*
		 * parity changed
		 */
		msm_hsuart_set_parity(	io_p_context->hsuart_id, 
					(mode.flags & HSUART_MODE_PARITY_MASK));
		io_p_context->uart_flags &= ~HSUART_MODE_PARITY_MASK;
		io_p_context->uart_flags |=  mode.flags & HSUART_MODE_PARITY_MASK;
	}
Done:
	HSUART_EXIT();
	return ret;

}

static int 
hsuart_ioctl(struct tty_struct *tty, struct file *file,
			 unsigned int cmd, unsigned long args)
{
	int 				ret = -ENOIOCTLCMD;;
	struct dev_ctxt* 		p_contxt;
	void *				usr_ptr   = (void*) (args);
	int   				usr_bytes = _IOC_SIZE(cmd);

	p_contxt = (struct dev_ctxt*)tty->driver_data;

	HSUART_ENTER();
	HSUART_DEBUG("%s, cmd 0x%x, args 0x%lx\n",
			__FUNCTION__, cmd, args);
	switch ( cmd ) {
		case  HSUART_IOCTL_GET_VERSION: 
		{
			int ver = DRIVER_VERSION;
			if( copy_to_user(usr_ptr, &ver, usr_bytes)) {
				ret = -EFAULT; 
				goto Done;
			}
		} break;

		case  HSUART_IOCTL_GET_BUF_INF: 
		{
			struct hsuart_buf_inf binf;

			binf.rx_buf_num  = p_contxt->rx_buf_num;
			binf.tx_buf_num  = p_contxt->tx_buf_num;
			binf.rx_buf_size = p_contxt->rx_buf_size;
			binf.tx_buf_size = p_contxt->tx_buf_size;
			if( copy_to_user (usr_ptr, &binf, usr_bytes)) {
				ret = -EFAULT; 
				goto Done;
			}
		}
		break;

		case HSUART_IOCTL_GET_STATS:
		{
			struct hsuart_stat stat;
			stat.tx_bytes	= p_contxt->tx_ttl;
			stat.rx_bytes	= p_contxt->rx_ttl;
			stat.rx_dropped = p_contxt->rx_dropped;
			if( copy_to_user ( usr_ptr, &stat, usr_bytes )) {
				ret = -EFAULT; 
				goto Done;
			}
		} break;

		case  HSUART_IOCTL_GET_UARTMODE: 
		{
			struct hsuart_mode mode;
			mode.speed = p_contxt->uart_speed;
			mode.flags = p_contxt->uart_flags;
			if( copy_to_user ( usr_ptr, &mode, usr_bytes )) {
				ret = -EFAULT; 
				goto Done;
			}
		} break;

		case HSUART_IOCTL_SET_RXLAT:
			p_contxt->pdata->rx_latency = args;
		//	hsuart_recalc_timeout( ctxt );
		break;
		
		case  HSUART_IOCTL_SET_UARTMODE:
			ret = hsuart_ioctl_set_uart_mode(p_contxt, usr_ptr, usr_bytes );
		break;

		case  HSUART_IOCTL_CLEAR_FIFO:
		case  HSUART_IOCTL_FLUSH:
			ret = hsuart_ioctl_flush(p_contxt, args);
		break;

		case  HSUART_IOCTL_TX_DRAIN:
			ret = hsuart_ioctl_tx_drain(p_contxt, args);
		break;

		case  HSUART_IOCTL_RX_BYTES:
			ret = hsuart_ioctl_rx_bytes(p_contxt);
		break;

		case  HSUART_IOCTL_RX_FLOW:
			ret = hsuart_ioctl_rx_flow(p_contxt, args);
		break;

		case  HSUART_IOCTL_RESET_UART:
			HSUART_INFO("%s: reset_uart\n", __FUNCTION__ );
		break;
		
	}
Done:	 
	return ret;
}

static int 
hsuart_open(struct tty_struct *tty, struct file *f)
{
	struct dev_ctxt*	p_context;
	int			ret=0;

	HSUART_ENTER();

	p_context = tty_info + tty->index;
	tty->driver_data = p_context;
	p_context->tty = tty;

	HSUART_DEBUG( " Hsuart open id %d opened %d initialized %d\n" , 
                         p_context->hsuart_id, 
                         (int) p_context->is_opened, 
                         (int) p_context->is_initilized );
	/*
	 * check if it is in use
	 */
	if (test_and_set_bit (0, &(p_context->is_opened))) {
		return -EBUSY;
	}

	if (0 == p_context->is_initilized) {

		p_context->uart_flags = p_context->pdata->uart_mode; 
		p_context->uart_speed = p_context->pdata->uart_speed;

		ret = hsuart_uart_port_init(p_context);


		if (ret) {
			clear_bit(0, &(p_context->is_opened));
			return ret;
		}
	
		 p_context->is_initilized = 1;

		HSUART_DEBUG( " Hsuart port init id %d opened %d initialized %d\n" , 
				p_context->hsuart_id, 
				(int) p_context->is_opened, 
				(int) p_context->is_initilized );
	//	hsuart_start_rx_xfer ( ctxt );

	}
	/*
	 * attach private data to the file handle for future use
	 */
	p_context->tx_ttl = 0;
	p_context->rx_ttl = 0;
	p_context->rx_dropped = 0;
    
	HSUART_EXIT();

	return ret;
}

static int 
hsuart_close(struct tty_struct *tty, struct file *f)
{
	struct dev_ctxt*	p_contxt;
	int			ret = 0;

	HSUART_ENTER();

	p_contxt = tty->driver_data;

	HSUART_DEBUG( " Hsuart close id %d opened %d initialized %d\n" , 
                         p_contxt->hsuart_id, 
                         (int)p_contxt->is_opened, 
                         (int)p_contxt->is_initilized );

	if ( 0 != p_contxt->is_initilized ) {
		ret = hsuart_uart_port_release(p_contxt);
		p_contxt->is_initilized = 0;
	}

	/* mark it as unused */
	clear_bit(0, &(p_contxt)->is_opened);

	HSUART_EXIT();
	
	return ret;
}

static int hsuart_write_room(struct tty_struct *tty)
{
	return 4096;
}

static int hsuart_chars_in_buffer(struct tty_struct *tty)
{
	struct dev_ctxt*	p_contxt;
	p_contxt = tty->driver_data;
	return 0;//hsuart_rx_avail(&(p_contxt->rx_lists));
}

char rcv_buf[4096];
void hsuart_tty_flip(void)
{
	struct dev_ctxt*	p_contxt=tty_info;
	int bytes;
	char *ptr = rcv_buf;
	struct tty_struct *tty = p_contxt->tty;
	int i=0;

	bytes = hsuart_rx_avail(&(p_contxt->rx_lists));
	if(bytes<=0)
		return;

	hsuart_read(p_contxt,rcv_buf,bytes);
#if 0
	printk("Bytes: \n");
	for(i=0; i < bytes; i++)
		printk("%2.2X ", rcv_buf[i]);
	printk("\n");
#endif
	tty_insert_flip_string(tty, ptr, bytes);
	tty_flip_buffer_push(tty);
}

#if 0
static struct file_operations hsuart_fops = {
	.llseek  = no_llseek,
	.read    = hsuart_read,
	.write   = hsuart_write,
//	.fsync   = hsuart_fsync,
	.poll    = hsuart_poll,
	.ioctl   = hsuart_ioctl,
	.open    = hsuart_open,
	.release = hsuart_close,
};
#endif

static struct tty_operations uart_tty_ops = {
	.open = hsuart_open,
	.close = hsuart_close,
	.write = hsuart_write,
	.write_room = hsuart_write_room,
	.chars_in_buffer = hsuart_chars_in_buffer,
//	.unthrottle = hsuart_unthrottle,
//	.tiocmget = hsuart_tiocmget,
//	.tiocmset = hsuart_tiocmset,
	.ioctl = hsuart_ioctl,
};




static int __devexit
hsuart_remove ( struct platform_device *dev )
{
	struct dev_ctxt*	p_contxt;
	int			ret = 0;

	HSUART_ENTER();
	
	/*
	 * TODO: fixme, add proper cleanups!!!
	 */

	/*
	 * Remove sysfs entries.
	 */
	device_remove_file(&(dev->dev), &dev_attr_dbg_lvl);

	p_contxt = platform_get_drvdata(dev);
	if(NULL != p_contxt) {
		platform_set_drvdata (dev, NULL);
	}
	return ret;	 
}

struct platform_driver hsuart_dummy_driver;
static int hsuart_dummy_probe(struct platform_device *pdev)
{
	return 0;
}

static int __devinit 
hsuart_probe(struct platform_device  *dev)
{
	static struct tty_driver *	tty_driver;
	struct hsuart_platform_data*	p_data;
	struct dev_ctxt*		p_contxt = NULL;
	int				ret;

	HSUART_ENTER();	

	p_data = dev->dev.platform_data;
	if(p_data == NULL) {
		HSUART_ERR("%s: no platform data\n", DRIVER_NAME);
		return -ENODEV;
	}

	p_contxt = tty_info;

	/* Attach the context to its device */
	platform_set_drvdata(dev, p_contxt);

	/* Attach platfor-device and data to the context */
	p_contxt->pdev = dev;
	p_contxt->pdata = p_data;	

	ret = device_create_file(&(dev->dev), &dev_attr_dbg_lvl);
	if(ret) 
		goto probe_cleanup;

	/*
	 * Init main spin lock
	 */
	spin_lock_init(&(p_contxt->lock));

	/*
	 * Import data from the board file
	 */

	/* Get the name */
	if (p_data->dev_name) {
		p_contxt->dev_name = p_data->dev_name;
	}
	else {
		p_contxt->dev_name = dev->name;
	}
	
	/* Get the port number */
	p_contxt->uart_port_number = dev->id;
	
	/* Init Rx/Tx buffer sub-system */
	p_contxt->rx_buf_size = p_data->rx_buf_size;
	p_contxt->rx_buf_num =  p_data->rx_buf_num;

	p_contxt->tx_buf_size = p_data->tx_buf_size;
	p_contxt->tx_buf_num =  p_data->tx_buf_num;

	p_contxt->min_packet_sz = p_data->min_packet_size;

	ret = hsuart_rxtx_buf_init(p_contxt);
	if (ret) {
		goto probe_cleanup;
	}

	//memcpy(&p_contxt->fops, &hsuart_fops, sizeof(struct file_operations));
	tty_driver = alloc_tty_driver(1);
	tty_driver->owner = THIS_MODULE;
	tty_driver->driver_name = "hsuart_tty";
	tty_driver->name = "ttyHS"; //p_contxt->dev_name;
	tty_driver->major = 90;
	tty_driver->minor_start = 0;
	tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	tty_driver->subtype = SERIAL_TYPE_NORMAL;
	tty_driver->init_termios = tty_std_termios;
	tty_driver->init_termios.c_iflag = 0;
	tty_driver->init_termios.c_oflag = 0;
	tty_driver->init_termios.c_cflag = B115200 | CS8 | CREAD;
	tty_driver->init_termios.c_lflag = 0;
	tty_driver->flags = TTY_DRIVER_RESET_TERMIOS |
		TTY_DRIVER_REAL_RAW | TTY_DRIVER_INSTALLED;//TTY_DRIVER_DYNAMIC_DEV;
	tty_set_operations(tty_driver, &uart_tty_ops);

	tty_register_driver(tty_driver);

	p_contxt->uart_flags = p_data->uart_mode; 
	p_contxt->uart_speed = p_data->uart_speed;

	init_waitqueue_head(&(p_contxt->got_rx_buffer));
	init_waitqueue_head(&(p_contxt->got_tx_buffer));

//	tty_register_device(tty_driver, 0, 0);

	hsuart_dummy_driver.probe = hsuart_dummy_probe;
	hsuart_dummy_driver.driver.name = "whatev";
	hsuart_dummy_driver.driver.owner = THIS_MODULE;
	ret = platform_driver_register(&hsuart_dummy_driver);

	printk("%s:created '%s' device on UART %d\n", 
	        	DRIVER_NAME, 
			p_contxt->dev_name, 
			p_contxt->uart_port_number );

	/* Handle non-defferred initialization */
	if(!(p_contxt->pdata->options & HSUART_OPTION_DEFERRED_LOAD)) {
/*
		ret = hsuart_init_uart ( ctxt );
		if( ret ) {
			goto err_misc_unregister;
		}
		hsuart_start_rx_xfer ( ctxt );
*/
	}

	return 0;

probe_cleanup:
	/* TODO: break this into multiple steps of clean upand jump to each 
		step based on where we failed */
	HSUART_ERR("%s: FIXME\n", DRIVER_NAME);
	HSUART_ERR("%s: Failed (%d) to initialize device\n", 
	        	DRIVER_NAME, ret );
	
	return ret;
}

#ifdef CONFIG_PM
static int 
hsuart_suspend(struct platform_device *dev, pm_message_t state)
{
	int    ret	= 0;

	struct dev_ctxt* io_p_contxt = (struct dev_ctxt*) platform_get_drvdata(dev);

	HSUART_ENTER();

	HSUART_DEBUG( " Hsuart suspend id %d opened %d initialized %d\n" , 
                         io_p_contxt->hsuart_id, 
                         (int)io_p_contxt->is_opened, 
                         (int)io_p_contxt->is_initilized );

	if ( !io_p_contxt->is_opened || !io_p_contxt->is_initilized ) {
		goto out;
	}

	ret = msm_hsuart_suspend(io_p_contxt->hsuart_id);

	if (io_p_contxt->pdata->options & HSUART_OPTION_RX_FLUSH_QUEUE_ON_SUSPEND ) {
		hsuart_flush_rx_queue(io_p_contxt);
	}

	if (io_p_contxt->pdata->options & HSUART_OPTION_TX_FLUSH_QUEUE_ON_SUSPEND ) {
		hsuart_flush_tx_queue(io_p_contxt);
	}

out:
	HSUART_EXIT();

	return ret;
}

static int 
hsuart_resume (struct platform_device *dev)
{
	int    ret	= 0;

	struct dev_ctxt* io_p_contxt = (struct dev_ctxt*) platform_get_drvdata(dev);

	HSUART_ENTER();

	HSUART_DEBUG( " Hsuart resume id %d opened %d initialized %d\n" , 
                         io_p_contxt->hsuart_id, 
                         (int)io_p_contxt->is_opened, 
                         (int)io_p_contxt->is_initilized );

	if ( !io_p_contxt->is_opened || !io_p_contxt->is_initilized ) {
		goto out;
	}

	ret = msm_hsuart_resume(io_p_contxt->hsuart_id);
out:
	HSUART_EXIT();

	return ret;
}
#else
#define hsuart_suspend	NULL
#define hsuart_resume	NULL
#endif	/* CONFIG_PM */

static struct platform_driver hsuart_driver = {
	.driver   = {
		.name = DRIVER_NAME,
	},
	.probe	  = hsuart_probe,
	.remove   = __devexit_p(hsuart_remove),
	.suspend  = hsuart_suspend,
	.resume   = hsuart_resume,
};

/*
 *
 */
static int __init 
hsuart_init(void)
{
	return platform_driver_register(&hsuart_driver);
}

/*
 *
 */
static void __exit 
hsuart_exit(void)
{
	platform_driver_unregister ( &hsuart_driver );
}

module_init(hsuart_init);
module_exit(hsuart_exit);

