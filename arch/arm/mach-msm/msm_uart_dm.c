/*
 * arch/arm/mach-msm/msm_uart_dm.c - Driver for MSM uart DM ports
 *
 *
 * Copyright (C) 2008 Palm, Inc.
 * Author: Amir Frenkel <amir.frenkel@palm.com>
 *
 * Based on drivers/serial/msm_serial.c driver implementation
 * Copyright (C) 2007 Google, Inc.
 * Author: Robert Love <rlove@google.com>
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
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/time.h>

#include <linux/clk.h>
#include <linux/platform_device.h>

#include <mach/dma.h>
#include <linux/dma-mapping.h>

#include <mach/msm_uart_dm.h>

/*
 *	Debug related macros and defs
 */
#define  DRIVER_NAME      			"msm_uartdm"
#define  DRIVER_VERSION   			 (0x100)


#define MSM_UARTDM_DEBUG_ENABLE		0
#define MSM_UARTDM_FUNC_LOG_ENABLE	0

#if MSM_UARTDM_DEBUG_ENABLE
#define MSM_UARTDM_DEBUG(args...)	(printk(KERN_DEBUG args))
#define MSM_UARTDM_INFO(args...)	(printk(KERN_INFO args))
#define MSM_UARTDM_ERR(args...)		(printk(KERN_ERR args))
#else
#define MSM_UARTDM_INFO(args...)
#define MSM_UARTDM_DEBUG(args...)
#define MSM_UARTDM_ERR(args...)
#endif // MSM_UARTDM_DEBUG_ENABLE

#if MSM_UARTDM_FUNC_LOG_ENABLE
#define	MSM_UARTDM_ENTER()		(printk(KERN_INFO"%s: %s, %u[usec] enter\n",\
					DRIVER_NAME, __PRETTY_FUNCTION__, jiffies_to_msecs(jiffies)))

#define	MSM_UARTDM_EXIT()		(printk(KERN_INFO"%s: %s, %u[usec] exit\n",\
					DRIVER_NAME, __PRETTY_FUNCTION__, jiffies_to_msecs(jiffies)))

#else
#define MSM_UARTDM_ENTER()
#define MSM_UARTDM_EXIT()
#endif

#define RX_MODE_PIO(p_port) (p_port->flags & UART_DM_MODE_RX_PIO)
#define RX_MODE_DM(p_port)  (p_port->flags & UART_DM_MODE_RX_DM)

#define MSM_UARTDM_BURST_SIZE 16

/*
 *	UART clk related definitions.
 */

/*
 * The MSM possible fundamental clk for UART DM
 * Don't know how to represent it in C, but having the
 * numbers here is a good start. Keep in mind that when sending them over to the
 * modem via proc-comm, the granularity is hz, hence 3.6864 Mhz should be
 * sent as 3686
 * - 3.6864Mhz
 * - 7.3728Mhz
 * - 14.7456Mhz
 * - 46.4Mhz

 * - 51.2Mhz
 * - 58.9824Mhz
 * - 64Mhz
 */

#define GSBI_CONTROL_ADDR 0x0
#define GSBI_PROTOCOL_UART 0x40
#define GSBI_PROTOCOL_IDLE 0x0

#define TCSR_ADM_1_A_CRCI_MUX_SEL 0x78
#define TCSR_ADM_1_B_CRCI_MUX_SEL 0x7C
#define ADM1_CRCI_GSBI6_RX_SEL 0x800
#define ADM1_CRCI_GSBI6_TX_SEL 0x400

/*
 * static port DB definition
 */

struct uart_port_item {
	struct generic_uart_port	port;

	/*
	 * 1 - used
	 * 0 - not used
	 */
	int				used;
};

#define UARTDM_NUM_PORTS		(2)
#define FIFO_SZ	(64)
static struct uart_port_item	ports_db[UARTDM_NUM_PORTS] =
{
	{
		.port =
		{
			/*
			 * The UARTDM has a total of 128*32bit SRAM for Rx/Tx
			 * FIFO The split between Rx and Tx is configurable,
			 * hence the rx and tx fifos may be of different size.
			 * Currently for simplicity - assuming they are equal
			 * in size
			 */
			.rx_fifo_size = FIFO_SZ,
			.tx_fifo_size = FIFO_SZ,
			.p_clk_name = "uartdm_clk",
			.p_pclk_name = "uartdm_pclk",
			.id	= 0,
			.rx_dm = {0},
		},

		.used = 0,
	},

	{
		.port =
		{
			/*
			 * The UARTDM has a total of 128*32bit SRAM for Rx/Tx
			 * FIFO The split between Rx and Tx is configurable,
			 * hence the rx and tx fifos may be of different size.
			 * Currently for simplicity - assuming they are equal
			 * in size
			 */
			.rx_fifo_size = FIFO_SZ,
			.tx_fifo_size = FIFO_SZ,
			.p_clk_name = "uartdm_clk",
			.p_pclk_name = "uartdm_pclk",
			.id	= 1,
			.rx_dm  = {0},
		},

		.used = 0,
	}

};
#define UART_NR	ARRAY_SIZE(ports_db)

static void
__msm_uartdm_set_baud_rate(struct generic_uart_port* i_p_port, unsigned int baud);

static void
__msm_uartdm_set_stale_timeout(struct generic_uart_port* i_p_port);

static void
__msm_uartdm_reset(struct generic_uart_port *port);

static inline void msm_write(struct generic_uart_port *port, unsigned int val,
			     unsigned int off)
{
	__raw_writel(val, port->p_membase + off);
}

static inline unsigned int msm_read(struct generic_uart_port *port, unsigned int off)
{
	return __raw_readl(port->p_membase + off);
}


/**
 * Disable 'tx-level' interrupt
 *
 * @param[in]	i_p_port - The UART DM port.
 *
 * @return	1 - tx-level was enabled BEFORE the call to this function,
 *		0 otherwise.
 */
int
msm_uartdm_disable_tx_level(struct generic_uart_port* i_p_port)
{
	struct msm_uart_port* 	p_msm_uart_port = GEN_UART_TO_MSM(i_p_port);
	unsigned long 		flags;
	int			ret;

	spin_lock_irqsave(&(p_msm_uart_port->lock), flags);
	ret = (p_msm_uart_port->imr & UART_DM_IMR_TX_LEV);
	ret = !!ret;
	p_msm_uart_port->imr &= ~UART_DM_IMR_TX_LEV;
	msm_write(i_p_port, p_msm_uart_port->imr, UART_DM_IMR);
	spin_unlock_irqrestore(&(p_msm_uart_port->lock), flags);

	return ret;
}
EXPORT_SYMBOL(msm_uartdm_disable_tx_level);

void
msm_uartdm_enable_tx_level(struct generic_uart_port* i_p_port)
{
	unsigned long		flags;
	struct msm_uart_port* 	p_msm_uart_port = GEN_UART_TO_MSM(i_p_port);

	spin_lock_irqsave(&(p_msm_uart_port->lock), flags);
	p_msm_uart_port->imr |= UART_DM_IMR_TX_LEV;
	msm_write(i_p_port, p_msm_uart_port->imr, UART_DM_IMR);
	spin_unlock_irqrestore(&(p_msm_uart_port->lock), flags);

}
EXPORT_SYMBOL(msm_uartdm_enable_tx_level);

/**
 * Disable 'tx-ready' interrupt
 *
 * @param[in]	i_p_port - The UART DM port.
 *
 * @return	1 - tx-ready was enabled BEFORE the call to this function,
 *		0 otherwise.
 */
int
msm_uartdm_disable_tx_rdy(struct generic_uart_port* i_p_port)
{
	unsigned long		flags;
	struct msm_uart_port* 	p_msm_uart_port = GEN_UART_TO_MSM(i_p_port);
	int			ret;

	spin_lock_irqsave(&(p_msm_uart_port->lock), flags);

	ret = (p_msm_uart_port->imr & UART_DM_IMR_TX_RDY);
	ret = !!ret;
	p_msm_uart_port->imr &= ~UART_DM_IMR_TX_RDY;
	msm_write(i_p_port, p_msm_uart_port->imr, UART_DM_IMR);
	spin_unlock_irqrestore(&(p_msm_uart_port->lock), flags);

	return ret;
}
EXPORT_SYMBOL(msm_uartdm_disable_tx_rdy);

void
msm_uartdm_enable_tx_rdy(struct generic_uart_port* i_p_port)
{
	struct msm_uart_port* 	p_msm_uart_port = GEN_UART_TO_MSM(i_p_port);
	unsigned long 		flags;

	MSM_UARTDM_ENTER();
	spin_lock_irqsave(&(p_msm_uart_port->lock), flags);

	p_msm_uart_port->imr |= UART_DM_IMR_TX_RDY;
	msm_write(i_p_port, p_msm_uart_port->imr, UART_DM_IMR);

	spin_unlock_irqrestore(&(p_msm_uart_port->lock), flags);
	MSM_UARTDM_EXIT();
}
EXPORT_SYMBOL(msm_uartdm_enable_tx_rdy);

void
msm_uartdm_enable_rx_irqs(struct generic_uart_port *port, int enable_stale)
{
	unsigned long 		flags;
	struct msm_uart_port* 	p_msm_uart_port = GEN_UART_TO_MSM(port);
	u32                     imr = 0;
	MSM_UARTDM_ENTER();

	if ( RX_MODE_PIO(p_msm_uart_port) ) {
		imr = UART_DM_IMR_RX_LEV | UART_DM_IMR_RX_STALE;
	}
	else if ( RX_MODE_DM(p_msm_uart_port)) {
		imr = UART_DM_IMR_RX_STALE;
	}


	spin_lock_irqsave(&(p_msm_uart_port->lock), flags);

	/*
	* Enable stale event if needed
	*/
	if (enable_stale) {
		msm_write(port, UART_DM_CR_GCMD_EN_STALE_EVENT, UART_DM_CR);
	}

	/*
	* Enable RX interrupts.
	*/
	p_msm_uart_port->imr |= imr;
	msm_write(port, p_msm_uart_port->imr, UART_DM_IMR);

	spin_unlock_irqrestore(&(p_msm_uart_port->lock), flags);

	MSM_UARTDM_EXIT();
}
EXPORT_SYMBOL(msm_uartdm_enable_rx_irqs);

/**
 * Disable 'rx-level/stale' interrupts
 *
 * @param[in]	i_p_port - The UART DM port.
 *
 * @return	1 - any of the rx intr was enabled BEFORE the call to
 * 		this function, 0 otherwise.
 */
int
msm_uartdm_disable_rx_irqs(struct generic_uart_port* i_p_port)
{
	struct msm_uart_port* 	p_msm_uart_port = GEN_UART_TO_MSM(i_p_port);
	int			ret = 0;
	unsigned long		flags;
	u32                     imr;

	MSM_UARTDM_ENTER();

	if ( RX_MODE_PIO(p_msm_uart_port) ) {
		imr = UART_DM_IMR_RX_LEV | UART_DM_IMR_RX_STALE;
	}
	else if ( RX_MODE_DM(p_msm_uart_port)) {
		imr = UART_DM_IMR_RX_STALE;
		//Disable stale event
		msm_write(i_p_port, UART_DM_CR_GCMD_DIS_STALE_EVENT, UART_DM_CR);
	}
	else {
		printk(KERN_ERR"%s, %d, invalid port settings\n",
			__func__,
			__LINE__);
		goto done;
	}
	spin_lock_irqsave(&(p_msm_uart_port->lock), flags);

	/*
	* Test and see if the rx irqs were disabled already.
	*/
	ret = p_msm_uart_port->imr & imr;
	ret = !!ret;
	p_msm_uart_port->imr &= ~imr;
	msm_write(i_p_port, p_msm_uart_port->imr, UART_DM_IMR);

	spin_unlock_irqrestore(&(p_msm_uart_port->lock), flags);

	MSM_UARTDM_EXIT();
done:
	return ret;
}
EXPORT_SYMBOL(msm_uartdm_disable_rx_irqs);

void
msm_uartdm_enable_rx(struct generic_uart_port* i_p_port)
{
	struct msm_uart_port* p_msm_uart_port = GEN_UART_TO_MSM(i_p_port);
	MSM_UARTDM_ENTER();

	msm_write(p_msm_uart_port,
		  UART_DM_CR_RX_ENABLE,
		  UART_DM_CR);

	MSM_UARTDM_EXIT();
}
EXPORT_SYMBOL(msm_uartdm_enable_rx);

/**
 * Reset the receiver as if HW reset was issued.
 * The receiver is disabled, and the HW FIFO and packing
 * and shift registers are flushed.
 *
 * @param[in]	- The UART port to operate on.
 * @return	- None.
 *
 */
void
msm_uartdm_reset_rx(struct generic_uart_port* i_p_port)
{
	struct msm_uart_port* p_msm_uart_port = GEN_UART_TO_MSM(i_p_port);
	MSM_UARTDM_ENTER();

	msm_write(p_msm_uart_port,
		  UART_DM_CR_CMD_RESET_RX,
		  UART_DM_CR);

	MSM_UARTDM_EXIT();
}
EXPORT_SYMBOL(msm_uartdm_reset_rx);



void
msm_uartdm_disable_rx(struct generic_uart_port* i_p_port)
{
	struct msm_uart_port* p_msm_uart_port = GEN_UART_TO_MSM(i_p_port);
	MSM_UARTDM_ENTER();

	msm_write(p_msm_uart_port,
		  UART_DM_CR_RX_DISABLE,
		  UART_DM_CR);

	MSM_UARTDM_EXIT();
}
EXPORT_SYMBOL(msm_uartdm_disable_rx);

void
msm_uartdm_enable_tx(struct generic_uart_port* i_p_port)
{
	struct msm_uart_port* p_msm_uart_port = GEN_UART_TO_MSM(i_p_port);
	MSM_UARTDM_ENTER();

	msm_write(p_msm_uart_port,
		  UART_DM_CR_TX_ENABLE,
		  UART_DM_CR);

	MSM_UARTDM_EXIT();
}
EXPORT_SYMBOL(msm_uartdm_enable_tx);

void
msm_uartdm_disable_tx(struct generic_uart_port* i_p_port)
{
	struct msm_uart_port* p_msm_uart_port = GEN_UART_TO_MSM(i_p_port);
	MSM_UARTDM_ENTER();

	msm_write(p_msm_uart_port,
		  UART_DM_CR_TX_DISABLE,
		  UART_DM_CR);

	MSM_UARTDM_EXIT();
}
EXPORT_SYMBOL(msm_uartdm_disable_tx);

/**
 * Reset the transmitter as if HW reset was issued.
 * The transmitter signal goes high and the HW FIFO and packing
 * and shift registers are flushed.
 *
 * @param[in]	- The UART port to operate on.
 * @return	- None.
 *
 */
void
msm_uartdm_reset_tx(struct generic_uart_port* i_p_port)
{
	struct msm_uart_port* p_msm_uart_port = GEN_UART_TO_MSM(i_p_port);
	MSM_UARTDM_ENTER();

	msm_write(p_msm_uart_port,
		  UART_DM_CR_CMD_RESET_TX,
		  UART_DM_CR);

	MSM_UARTDM_EXIT();
}
EXPORT_SYMBOL(msm_uartdm_reset_tx);


/*
static void msm_enable_ms(struct generic_uart_port* i_p_port)
{
	struct msm_uart_port* p_msm_uart_port = GEN_UART_TO_MSM(i_p_port);

	p_msm_uart_port->imr |= UART_DM_IMR_DELTA_CTS;
	msm_write(i_p_port, p_msm_uart_port->imr, UART_DM_IMR);
}
*/
static void
handle_rx_stale(struct generic_uart_port* i_p_port)
{
	MSM_UARTDM_ENTER();

	/*
	 * Clear the interrupt bit.
	 */
	msm_write(i_p_port, UART_DM_CR_CMD_CLR_STALE, UART_DM_CR);

	/*
	 * Handle overrun. My understanding of the hardware is that overrun
	 * is not tied to the RX buffer, so we handle the case out of band.
	 */
	if ((msm_read(i_p_port, UART_DM_SR) & UART_DM_SR_OVERRUN)) {
		msm_write(i_p_port, UART_DM_CR_CMD_RESET_ERR, UART_DM_CR);
	}

	/*
	 * Invoke callback if one is configured.
	 */
	if (NULL != i_p_port->p_rx_stale_callback) {
		MSM_UARTDM_DEBUG("%s, FIFO LEVEL %d\n",__FUNCTION__, msm_read(i_p_port, UART_DM_RXFS));

		i_p_port->p_rx_stale_callback(i_p_port->p_rx_stale_data);
	}
	MSM_UARTDM_EXIT();
}



static void
handle_rx_level(struct generic_uart_port* i_p_port)
{
	MSM_UARTDM_ENTER();

	/*
	 * Handle overrun. My understanding of the hardware is that overrun
	 * is not tied to the RX buffer, so we handle the case out of band.
	 */
	if ((msm_read(i_p_port, UART_DM_SR) & UART_DM_SR_OVERRUN)) {
		msm_write(i_p_port, UART_DM_CR_CMD_RESET_ERR, UART_DM_CR);
	}

	/*
	 * Invoke callback if one is configured.
	 */
	if (NULL != i_p_port->p_rx_level_callback) {
		MSM_UARTDM_DEBUG("%s, FIFO LEVEL %d\n",__FUNCTION__, msm_read(i_p_port, UART_DM_RXFS));
		i_p_port->p_rx_level_callback(i_p_port->p_rx_level_data);
	}

	MSM_UARTDM_EXIT();
}

static void
msm_uartdm_clear_dm_error(struct generic_uart_port* i_p_port)
{
	msm_write( i_p_port, UART_DM_DMEN_RX_DM_DIS, UART_DM_DMEN  );
	__msm_uartdm_reset(i_p_port);

	msm_dmov_clear_error_condition(i_p_port->dma_rx_channel, i_p_port->dma_rx_crci);

	msm_write( i_p_port,
			UART_DM_CR_RX_ENABLE | UART_DM_CR_TX_ENABLE,
			UART_DM_CR);

	msm_write( i_p_port, UART_DM_DMEN_RX_DM_EN, UART_DM_DMEN  );
}

static void
handle_rx_dm(struct msm_dmov_cmd *cmd_ptr,
	       unsigned int result,
	       struct msm_dmov_errdata *e
	       __maybe_unused)
{
	struct generic_uart_port* i_p_port;

	MSM_UARTDM_ENTER();

	if (NULL == cmd_ptr) {
		MSM_UARTDM_ERR("%s: %s, invalid(null) contxt\n",
			  DRIVER_NAME,
			  __PRETTY_FUNCTION__);
		return;
	}

	i_p_port = container_of(cmd_ptr, struct generic_uart_port, rx_xfer);

	if ( result & DMOV_RSLT_ERROR ) {
		msm_uartdm_clear_dm_error(i_p_port);
	}

	/*
	 * Invoke callback if one is configured.
	 */
	if (NULL != i_p_port->p_rx_dm_callback) {
		MSM_UARTDM_DEBUG("%s, FIFO LEVEL %d\n",__FUNCTION__, msm_read(i_p_port, UART_DM_RXFS));
		i_p_port->p_rx_dm_callback(i_p_port->p_rx_dm_data);
	}

	MSM_UARTDM_EXIT();
}

unsigned int msm_uartdm_read_reg(struct generic_uart_port* i_p_port, int addr)
{
	unsigned int ret;
	ret = msm_read(i_p_port, addr);
	return ret;
}

EXPORT_SYMBOL(msm_uartdm_read_reg);

static void
handle_tx_level(struct generic_uart_port* i_p_port)
{
	struct msm_uart_port*		p_msm_port = GEN_UART_TO_MSM(i_p_port);

	MSM_UARTDM_ENTER();

	/*
	 * Invoke callback if one is configured.
	 */
	if (NULL != i_p_port->p_tx_level_callback) {
		i_p_port->p_tx_level_callback(i_p_port->p_tx_level_data);
	}
	else {
		/* disable tx level interrupts */
		p_msm_port->imr &= ~UART_DM_IMR_TX_LEV;
		msm_write(i_p_port, p_msm_port->imr, UART_DM_IMR);
	}
	MSM_UARTDM_EXIT();
}
static void
handle_tx_rdy(struct generic_uart_port* i_p_port)
{
	struct msm_uart_port*		p_msm_port = GEN_UART_TO_MSM(i_p_port);

	MSM_UARTDM_ENTER();

	/*
	 * Clear the interrupt bit.
	 */
	msm_write(i_p_port, UART_DM_CR_GCMD_RESET_TX_RDY_INT, UART_DM_CR);

	/*
	 * Invoke callback if one is configured.
	 */
	if (NULL != i_p_port->p_tx_rdy_callback) {
		i_p_port->p_tx_rdy_callback(i_p_port->p_tx_rdy_data);
	}
	else {
		/* disable tx rdy interrupts */
		p_msm_port->imr &= ~UART_DM_IMR_TX_RDY;
		msm_write(i_p_port, p_msm_port->imr, UART_DM_IMR);
	}
	MSM_UARTDM_EXIT();
}


/*
 * Helper function, predicate which check whether the TX fifo has room and can
 * pushed with more data
 *
 * @param[in]	i_p_port - The UART DM port to check.
 *
 * @return	0 no more space in tx fifo, 1 we have room in the tx fifo.
 */
int
msm_uartdm_tx_ready(struct generic_uart_port* i_p_port)

{

	return (msm_read(i_p_port, UART_DM_SR) & UART_DM_SR_TX_READY);
}

EXPORT_SYMBOL(msm_uartdm_tx_ready);

/**
 * Helper function, predicate which check whether the RX fifo has more dat
 *
 * @param[in]	i_p_port - The UART DM port to check.
 *
 * @return	0 no more data in the rx fifo, 1 we have more data in
 *		the rx fifo.
 */
int
msm_uartdm_rx_ready(struct generic_uart_port* i_p_port)

{
	return (msm_read(i_p_port, UART_DM_SR) & UART_DM_SR_RX_READY);
}

EXPORT_SYMBOL(msm_uartdm_rx_ready);


/*
static void handle_delta_cts(struct generic_uart_port *port)
{
	msm_write(port, UART_CR_CMD_RESET_CTS, UART_CR);
	port->icount.cts++;
	wake_up_interruptible(&port->info->delta_msr_wait);
}

*/
static irqreturn_t
msm_uartdm_irq(int irq, void *dev_id)
{
	struct generic_uart_port*	p_port = dev_id;
	struct msm_uart_port*		p_msm_port = GEN_UART_TO_MSM(p_port);
	unsigned int misr;
	unsigned int sr;
	unsigned int isr;

	//TODO: cleanup the logic a bit (locks, disable interrupts etc).
//	spin_lock(&(p_port->lock));
	misr = msm_read(p_port, UART_DM_MISR);
	isr = msm_read(p_port, UART_DM_ISR);
	sr = msm_read(p_port, UART_DM_SR);

	MSM_UARTDM_DEBUG("%s enter misr0x%x, isr0x%x, sr0x%x\n", __FUNCTION__,misr, isr, sr);
	msm_write(p_port, 0, UART_DM_IMR); /* disable interrupt */
	if (misr & UART_DM_IMR_RX_LEV) {
		handle_rx_level(p_port);
	}
	if (misr & UART_DM_IMR_RX_STALE) {
		handle_rx_stale(p_port);
	}
	if (misr & UART_DM_IMR_TX_LEV) {
		handle_tx_level(p_port);
	}
	if (misr & UART_DM_IMR_TX_RDY) {
		/*
		 * Clear TX-RDY interrupt.
		 */
		handle_tx_rdy(p_port);
	}

	if (misr & UART_DM_IMR_DELTA_CTS) {
	//	handle_delta_cts(port);
	}

	msm_write(p_port, p_msm_port->imr, UART_DM_IMR); /* restore interrupt */

//	spin_unlock(&(p_port->lock));
	misr = msm_read(p_port, UART_DM_MISR);
	isr = msm_read(p_port, UART_DM_ISR);
	sr = msm_read(p_port, UART_DM_SR);

	MSM_UARTDM_DEBUG("%s exit misr0x%x, isr0x%x, sr0x%x\n", __FUNCTION__,misr, isr, sr);

	return IRQ_HANDLED;
}
/**
 * Function to control the Rx flow control.
 * The MSM UARTDM support
 * - on for HW RX flow
 * - off for HW Rx Flow (which can be implemented as SW based flow)
 *
 * @param[in]	i_p_port    - The UART DM port to check.
 * @param[in]	flow_ctl    - 1 for HW flow control, 0 for SW flow control.
 * @param[in]	flow_state  - 1 flow line asserted , 0 flow line de-asserted
 *                            Applicable in case of SW flow control only
 *
 * @return	None
 */
static void
__msm_uartdm_set_rx_flow(struct generic_uart_port* i_p_port, uint32_t flow_ctl, uint32_t flow_state)
{
	unsigned int	tmp;
	struct msm_uart_port*	p_msm_port = GEN_UART_TO_MSM(i_p_port);

	MSM_UARTDM_DEBUG("%s, port0x%x, flow %d\n",
			__FUNCTION__, (uint32_t)i_p_port, flow_ctl);

	tmp = msm_read(i_p_port, UART_DM_MR1);
	if (flow_ctl) {
		tmp |= UART_MR1_RX_RDY_CTL;
		msm_write(i_p_port, tmp, UART_DM_MR1);

		if ( i_p_port->p_board_rts_pin_deassert_cb ) {
			i_p_port->p_board_rts_pin_deassert_cb(0);
		}
	}
	else {
		if ( flow_state ) {
			/*
			* The the RFR to low
			*/
			msm_write(i_p_port, UART_DM_CR_CMD_SET_RFR, UART_DM_CR);

			if ( i_p_port->p_board_rts_pin_deassert_cb ) {
				i_p_port->p_board_rts_pin_deassert_cb(0);
			}
		} else {
			/*
			* The the RFR to high
			*/
			if ( i_p_port->p_board_rts_pin_deassert_cb ) {
				i_p_port->p_board_rts_pin_deassert_cb(1);
			}

			msm_write(i_p_port, UART_DM_CR_CMD_RESET_RFR, UART_DM_CR);
		}

		tmp &= ~UART_MR1_RX_RDY_CTL;
		msm_write(i_p_port, tmp, UART_DM_MR1);
	}

	p_msm_port->rx_flow_ctl   = flow_ctl;
	p_msm_port->rx_flow_state = flow_state;


}

void
msm_uartdm_set_rx_flow(struct generic_uart_port* i_p_port, uint32_t flow_ctl, uint32_t flow_state)
{
	unsigned long	flags;
	spin_lock_irqsave(&(i_p_port->lock), flags);

	__msm_uartdm_set_rx_flow(i_p_port, flow_ctl, flow_state);

	spin_unlock_irqrestore(&(i_p_port->lock), flags);
}
EXPORT_SYMBOL(msm_uartdm_set_rx_flow);

/**
 * Function to control the Tx flow control.
 * The MSM UARTDM support
 * - on for HW flow
 * - off for HW Flow
 *
 * @param[in]	i_p_port - The UART DM port.
 * @param[in]	flow_ctl    - 1 for HW flow control, 0 for SW flow control.
 * @return	None
 */
static void
__msm_uartdm_set_tx_flow(struct generic_uart_port* i_p_port, uint32_t flow_ctl)
{
	uint32_t	tmp;
	struct msm_uart_port*	p_msm_port = GEN_UART_TO_MSM(i_p_port);

	MSM_UARTDM_DEBUG("%s, port0x%x, flow %d\n",
			__FUNCTION__, (uint32_t)i_p_port, flow_ctl);

	tmp = msm_read(i_p_port, UART_DM_MR1);

	if (flow_ctl) {
		tmp |= UART_MR1_CTS_CTL;
		msm_write(i_p_port, tmp, UART_DM_MR1);
	}
	else {
		tmp &= ~UART_MR1_CTS_CTL;
		msm_write(i_p_port, tmp, UART_DM_MR1);
	}

	p_msm_port->tx_flow_ctl = flow_ctl;
}

void
msm_uartdm_set_tx_flow(struct generic_uart_port* i_p_port, uint32_t flow_ctl)
{
	unsigned long	flags;

	spin_lock_irqsave(&(i_p_port->lock), flags);

	__msm_uartdm_set_tx_flow(i_p_port, flow_ctl);

	spin_unlock_irqrestore(&(i_p_port->lock), flags);
}
EXPORT_SYMBOL(msm_uartdm_set_tx_flow);

/**
 * Function to control the parity (NONE, ODD or EVEN)
 *
 * @param[in]	i_p_port - The UART DM port.
 * @param[in]	parity - 1 for HW flow control, 0 for no flow control.
 *
 * @return	None
 */
void
msm_uartdm_set_parity(struct generic_uart_port* i_p_port,
			msm_uartdm_parity_t parity)
{
	uint32_t	data;
	unsigned long	flags;
	unsigned int	tmp;

	MSM_UARTDM_DEBUG("%s, port0x%x, parity %d\n",
			__FUNCTION__, (uint32_t)i_p_port, parity);

	switch(parity) {
		case MSM_UARTDM_PARITY_NONE:
			data = UART_DM_MR2_PARITY_MODE_NONE;
		break;
		case MSM_UARTDM_PARITY_EVEN:
			data = UART_DM_MR2_PARITY_MODE_EVEN;
		break;
		case MSM_UARTDM_PARITY_ODD:
			data = UART_DM_MR2_PARITY_MODE_ODD;
		break;
		default:
			goto Done;
		break;
	}
	spin_lock_irqsave(&(i_p_port->lock), flags);

	tmp = msm_read(i_p_port, UART_DM_MR2);
	tmp &= ~UART_DM_MR2_PARITY_MODE_MASK;
	tmp |= data;
	msm_write(i_p_port, tmp, UART_DM_MR2);
	i_p_port->parity_data = data;

	spin_unlock_irqrestore(&(i_p_port->lock), flags);
Done:
	return;
}
EXPORT_SYMBOL(msm_uartdm_set_parity);


/*
static void msm_break_ctl(struct generic_uart_port *port, int break_ctl)
{
	if (break_ctl)
		msm_write(port, UART_DM_CR_CMD_START_BREAK, UART_DM_CR);
	else
		msm_write(port, UART_DM_CR_CMD_STOP_BREAK, UART_DM_CR);
}

*/
/**
 * Helper function to dump the content of a uart port structure
 *
 * @param[in]	- i_p_port - the port to print.
 *
 * @return	- None.
 */
static void
_uartdm_print_port(struct generic_uart_port* i_p_port, int line_num)
{
return;
	MSM_UARTDM_DEBUG("%s: %s, Dumping port 0x%x at line %d\n",
			DRIVER_NAME,
			__PRETTY_FUNCTION__,
			(int)i_p_port,
			line_num);
	MSM_UARTDM_DEBUG("name %s\n", &(i_p_port->name[0]));
	MSM_UARTDM_DEBUG("clk name %s\n", i_p_port->p_clk_name);
	MSM_UARTDM_DEBUG("clk_rate %d\n", i_p_port->clk_rate);
	MSM_UARTDM_DEBUG("irq %d\n", i_p_port->irq);
	MSM_UARTDM_DEBUG("id %d\n", i_p_port->id);
	MSM_UARTDM_DEBUG("mapbase 0x%x\n",i_p_port->mapbase);
	MSM_UARTDM_DEBUG("mem_size 0x%x, %d\n", i_p_port->mem_size, i_p_port->mem_size);
	MSM_UARTDM_DEBUG("p_membase 0x%x\n",(int)i_p_port->p_membase);
}

int
msm_uartdm_init_dma(struct generic_uart_port *i_p_port)
{
	int	ret = 0;
	MSM_UARTDM_ENTER();

	if ( RX_MODE_DM(i_p_port)) {

		i_p_port->rx_dm.command_ptr = (dmov_box *)
					dma_alloc_coherent(NULL,
					sizeof(dmov_box),
					&(i_p_port->rx_dm.command_ptr_phys),
					GFP_KERNEL);


		i_p_port->rx_dm.command_ptr_ptr = (u32 *)dma_alloc_coherent(NULL,
					sizeof(u32),
					&(i_p_port->rx_dm.command_ptr_ptr_phys),
					GFP_KERNEL);

		i_p_port->rx_dm.command_ptr->cmd = CMD_LC | CMD_SRC_CRCI( i_p_port->dma_rx_crci ) | CMD_MODE_BOX;

		i_p_port->rx_dm.command_ptr->src_dst_len = (MSM_UARTDM_BURST_SIZE << 16) | (MSM_UARTDM_BURST_SIZE);

		i_p_port->rx_dm.command_ptr->row_offset = MSM_UARTDM_BURST_SIZE;

		i_p_port->rx_dm.command_ptr->src_row_addr = (unsigned int)i_p_port->mapbase + UART_DM_RF;

		i_p_port->rx_xfer.complete_func = handle_rx_dm;
	}
	MSM_UARTDM_EXIT();
	return ret;
}

int
msm_uartdm_destroy_dma(struct generic_uart_port *i_p_port)
{
	int ret = 0;

	MSM_UARTDM_ENTER();

	if (RX_MODE_DM(i_p_port)) {

		//TODO Should we flush the dm first ? and wait for completion ?
		if ( i_p_port->rx_dm.command_ptr ) {
			dma_free_coherent(NULL, sizeof(dmov_box), i_p_port->rx_dm.command_ptr, i_p_port->rx_dm.command_ptr_phys);
		}

		if (  i_p_port->rx_dm.command_ptr_ptr ) {
			dma_free_coherent(NULL, sizeof(u32), i_p_port->rx_dm.command_ptr_ptr, i_p_port->rx_dm.command_ptr_ptr_phys);
		}

		i_p_port->rx_dm.command_ptr     = NULL;
		i_p_port->rx_dm.command_ptr_ptr = NULL;
	}

	MSM_UARTDM_EXIT();
	return ret;
}

/**
*
* Open the UART port, mark it as "busy" any subsequent open request to the same
*  UART port will fail, unless the port has been closed.
* The function allocate a control strucutre and mark it as taken, registers
* callbacks as specified in the input parameters.
*
* @param[in]    i_p_config - The UART port configure.
* @param[out]	o_pp_port - Pointer to pointer container to be filled with
*				pointer to the newly allocate port.
*
*
* @return 0 for success -1 otherwise.
*
* @Note	This function DOES NOT reset/configure the UART HW, it only deals with
*	managing the data-structures used for handling the UART port.
*/

int
msm_uartdm_port_open(	struct generic_uart_config* i_p_config,
			struct generic_uart_port** o_pp_port)
{
	int	ret = 0;
	int	i;
	int	line_num;
	struct generic_uart_port*	p_port	 = NULL;

	MSM_UARTDM_ENTER();

	/*
	 * Basic sanity check
	 */
	if (NULL == o_pp_port) {
		ret		= -EINVAL;
		line_num	= __LINE__;
		goto	uartdm_open_err;
	}


	/* TODO: amir, Add spinlock to msmuartdm layer to protect global data structures.*/

	/*
	 * See if the uart port is vacant and if so can a pointer to its
	 * control structure
	 */

	for (i = 0; i < UARTDM_NUM_PORTS; i++) {
		if ((0 == ports_db[i].used)&& (ports_db[i].port.id == i_p_config->port_id)) {
			ports_db[i].used = 1;
			p_port = &(ports_db[i].port);
			break;
		}
	}

	/*
	 * Check if we couldn't find free port and bail out in case we didn't.
	 */
	if (NULL == p_port) {
		ret		= -EBUSY;
		line_num	= __LINE__;
		goto uartdm_open_err;
	}

	/*
	 *	Init spinlock
	 */
	spin_lock_init(&(p_port->lock));

	/*
	 *	Clear the callback zone.
	 */
	p_port->p_rx_level_callback	= NULL;
	p_port->p_rx_level_data		= NULL;
	p_port->p_rx_stale_callback	= NULL;
	p_port->p_rx_stale_data		= NULL;
	p_port->p_tx_level_callback 	= NULL;
	p_port->p_tx_level_data 	= NULL;
	p_port->p_tx_rdy_callback 	= NULL;
	p_port->p_tx_rdy_data		= NULL;

	p_port->rx_flow_ctl		= 1;
	p_port->tx_flow_ctl		= 1;
	/*
	 * Default baud rate
	 */
	p_port->baud_rate = 115200;
	p_port->clk_rate = 7372800;

	p_port->parity_data = UART_DM_MR2_PARITY_MODE_NONE;

	p_port->flags = i_p_config->flags;

	p_port->rx_latency = i_p_config->rx_latency;

	p_port->p_board_pin_mux_cb          = i_p_config->p_board_pin_mux_cb;
	p_port->p_board_rts_pin_deassert_cb = i_p_config->p_board_rts_pin_deassert_cb;

	/*
	 * Enable the UARTDM clock.
	 */
	clk_set_rate(p_port->p_clk, p_port->clk_rate);
	clk_enable(p_port->p_clk);
	if (p_port->p_pclk)
		clk_enable(p_port->p_pclk);

	if (i_p_config->p_board_config_gsbi_cb)
		i_p_config->p_board_config_gsbi_cb();

	msm_uartdm_init_dma(p_port);

	/*
	 * At last, initialization are done, assign the newly allocated
	 * control structure to be returned via the pointer
	 */
	(*o_pp_port) = p_port;

	MSM_UARTDM_EXIT();
	return ret;

uartdm_open_err:
	MSM_UARTDM_ERR("%s: %s, error %d at line %d\n",
			DRIVER_NAME, __PRETTY_FUNCTION__, ret, line_num);

	_uartdm_print_port(p_port, __LINE__);
	MSM_UARTDM_EXIT();
	return ret;

}
EXPORT_SYMBOL(msm_uartdm_port_open);

/**
*
* Disable the UART port
*
* @param[in]       io_p_port - The UART port to configure.
*
* @Note
*/
static void __msm_uartdm_port_disable(struct generic_uart_port* io_p_port)
{
	msm_uartdm_disable_rx(io_p_port);

	//Un-mux the relevant pins
	if (io_p_port->p_board_pin_mux_cb) {
		io_p_port->p_board_pin_mux_cb(0);
	}

	msm_uartdm_disable_tx(io_p_port);
	/*
	* Turn off the UART interrupts
	*/
	msm_write(io_p_port, 0, UART_DM_IMR);


}
/**
*
* Close the UART port, unregisters callbacks.
*
* @param[in]       io_p_port - The UART port to configure.
*
* @return 0 for success -1 otherwise.
*
* @Note
*/
int
msm_uartdm_port_close(struct generic_uart_port* io_p_port)
{
	int				ret = 0;
	struct		uart_port_item*	p_item;
	unsigned long			irq_flags;

	MSM_UARTDM_ENTER();
	/*
	 * TODO: amir, make sure to abort/cancel any ongoing activity on the uart port BEFORE marking it as vacant.
	 *	export the disable-port logic to its own internal function so we can call it from other locations as well.
	 */
	if (NULL == io_p_port) {
		ret = -EINVAL;
	}
	else {
		/*
		 * lock the DB
		 */
		spin_lock_irqsave(&(io_p_port->lock), irq_flags);

		p_item = container_of(io_p_port, struct uart_port_item, port);

/* TODO:amir, ioremap and release mem was done at open/close, moved to probe */
#if 0
		release_mem_region(io_p_port->mapbase, io_p_port->mem_size);
		iounmap(io_p_port->p_membase);
		io_p_port->p_membase = NULL;
		io_p_port->mem_size = 0;
#endif
		__msm_uartdm_port_disable(io_p_port);

		io_p_port->p_rx_level_callback	= NULL;
		io_p_port->p_rx_level_data	= NULL;
		io_p_port->p_rx_stale_callback	= NULL;
		io_p_port->p_rx_stale_data	= NULL;
		io_p_port->p_tx_level_callback 	= NULL;
		io_p_port->p_tx_level_data 	= NULL;
		io_p_port->p_tx_rdy_callback 	= NULL;
		io_p_port->p_tx_rdy_data	= NULL;
		io_p_port->p_rx_dm_callback	= NULL;

		io_p_port->p_board_pin_mux_cb	        = NULL;
		io_p_port->p_board_rts_pin_deassert_cb	= NULL;

		spin_unlock_irqrestore(&(io_p_port->lock), irq_flags);

		free_irq(io_p_port->irq, io_p_port);

		msm_uartdm_destroy_dma(io_p_port);

		p_item->used = 0;

		clk_disable(io_p_port->p_clk);
		if (io_p_port->p_pclk) {
			clk_disable(io_p_port->p_pclk);
		}
	}

	MSM_UARTDM_EXIT();

	return ret;
}
EXPORT_SYMBOL(msm_uartdm_port_close);


/**
*
* Suspend the UART port
*
* @param[in]       io_p_port - The UART port to configure.
*
* @Note
*/
int msm_uartdm_port_suspend(struct generic_uart_port* io_p_port)
{
	int				ret = 0;
	unsigned long			irq_flags;

	MSM_UARTDM_ENTER();

	if (NULL == io_p_port) {
		ret = -EINVAL;
	}
	else {
		/*
		 * lock the DB
		 */
		spin_lock_irqsave(&(io_p_port->lock), irq_flags);

		__msm_uartdm_port_disable(io_p_port);

		clk_disable(io_p_port->p_clk);
		if (io_p_port->p_pclk) {
			clk_disable(io_p_port->p_pclk);
		}

		spin_unlock_irqrestore(&(io_p_port->lock), irq_flags);
	}

	MSM_UARTDM_EXIT();

	return ret;
}

EXPORT_SYMBOL(msm_uartdm_port_suspend);

/**
*
* Initialize the UART port, this function is configuring the UART related HW
* subscribe to IRQs ( if needed ) , configure register and all the good stuff that makes
* the UART tick...
* The settings that will be used (e.g. baud rate etc) are those that were
* configured before calling this function, if parameter is not set, the
* default value will be used.
* It is strongly advised not to rely on default values and configure the
* parameters to the desired value....
*
* @param[in]       io_p_port    - The UART port control structure.
*                  parity       - UART parity
*                  need_irq 	- flags whether to request irq or not
*                  enable_rx_tx - flags whether enable rx and tx or not
*
* @return 0 for success -1 otherwise.
*
*/
static int __msm_uartdm_port_init(struct generic_uart_port* io_p_port)
{
	int 		ret = 0;
	u32		rfr_level;
	unsigned int	data;
	struct msm_uart_port*	p_msm_port = GEN_UART_TO_MSM(io_p_port);

	MSM_UARTDM_ENTER();


	/*
	 * Turn off the UART interrupts
	 */
	msm_write(io_p_port, 0, UART_DM_IMR);

	//Mux the relevant pins as functional
	if (io_p_port->p_board_pin_mux_cb) {
		io_p_port->p_board_pin_mux_cb(1);
	}

	/*
	 * Set the UART speed
	 */
	__msm_uartdm_set_baud_rate(io_p_port, io_p_port->baud_rate);

	/* Reset UART */
	/* TODO: amir - move to a function to do more generic configuraion */
	msm_write(io_p_port,
		  UART_DM_MR2_BITS_PER_CHAR_8 | UART_DM_MR2_STOP_BIT_LEN_ONE | io_p_port->parity_data,
		  UART_DM_MR2);	/* 8N1 */

	/* Configure RFR for proper flow control */
	if (likely(io_p_port->rx_fifo_size > 12)) {
		rfr_level = io_p_port->rx_fifo_size - 12;
	}
	else {
		rfr_level = io_p_port->rx_fifo_size;
	}

	/* set automatic RFR level */
	data = msm_read(io_p_port, UART_DM_MR1);
	data &= ~UART_DM_MR1_AUTO_RFR_LEVEL1;
	data &= ~UART_DM_MR1_AUTO_RFR_LEVEL0;
	data |= UART_DM_MR1_AUTO_RFR_LEVEL1 & (rfr_level << 2);
	data |= UART_DM_MR1_AUTO_RFR_LEVEL0 & rfr_level;
	msm_write(io_p_port, data, UART_DM_MR1);

	/* Set flow control */
	__msm_uartdm_set_rx_flow(io_p_port, p_msm_port->rx_flow_ctl, p_msm_port->rx_flow_state);
	__msm_uartdm_set_tx_flow(io_p_port, p_msm_port->tx_flow_ctl);

	/* Configure stale settings */
	/* make sure that RXSTALE count is non-zero */
	data = msm_read(io_p_port, UART_DM_IPR);
	if (unlikely(!data)) {
		data |= UART_DM_IPR_STALE_TIMEOUT_LSB_MSK;
		msm_write(io_p_port, data, UART_DM_IPR);
	}

	if ( RX_MODE_DM(p_msm_port) ) {
		msm_write( io_p_port, UART_DM_DMEN_RX_DM_EN, UART_DM_DMEN  );
	}
	/* Reset UART */
	__msm_uartdm_reset(io_p_port);

	MSM_UARTDM_EXIT();

	return ret;
}

static int __msm_uartdm_port_init_imr(struct generic_uart_port* io_p_port)
{
	int ret = 0;
	struct msm_uart_port*	p_msm_port = GEN_UART_TO_MSM(io_p_port);

	/*
	 * Configure IMR
	 */
	p_msm_port->imr = UART_DM_IMR_CURRENT_CTS ;

	msm_write(io_p_port,
		  p_msm_port->imr,
		  UART_DM_IMR);

	return ret;
}
/**
*
* Resumes the UART port, this function is configuring the UART related HW
* subscribe  configure register and all the good stuff that makes
* the UART tick...
* The settings that will be used (e.g. baud rate etc) are those that were
* configured before calling this function, if parameter is not set, the
* default value will be used.
* It is strongly advised not to rely on default values and configure the
* parameters to the desired value....
*
* @param[in]       io_p_port - The UART port control structure.
*
*
* @return 0 for success -1 otherwise.
*
*/
int msm_uartdm_port_resume(struct generic_uart_port* io_p_port)
{
	int ret = 0;
	unsigned long	irq_flags;

	MSM_UARTDM_ENTER();

	spin_lock_irqsave(&(io_p_port->lock), irq_flags);

	/*
	 * Enable the UARTDM clock.
	 */

	clk_enable(io_p_port->p_clk);
	if (io_p_port->p_pclk)
		clk_enable(io_p_port->p_pclk);

	ret = __msm_uartdm_port_init(io_p_port);

	if (ret) {
		MSM_UARTDM_DEBUG("%s: %s, to initialize udartdm port 0x%xn",
			DRIVER_NAME,
			__PRETTY_FUNCTION__,
			ret);
		goto end;
	}

	ret = __msm_uartdm_port_init_imr(io_p_port);

end:
	spin_unlock_irqrestore(&(io_p_port->lock), irq_flags);

	MSM_UARTDM_EXIT();

	return ret;
}

EXPORT_SYMBOL(msm_uartdm_port_resume);

/**
*
* Initialize the UART port, this function is configuring the UART related HW
* subscribe to IRQs, configure register and all the good stuff that makes
* the UART tick...
* The settings that will be used (e.g. baud rate etc) are those that were
* configured before calling this function, if parameter is not set, the
* default value will be used.
* It is strongly advised not to rely on default values and configure the
* parameters to the desired value....
*
* @param[in]       io_p_port - The UART port control structure.
*
* @return 0 for success -1 otherwise.
*
*/
int
msm_uartdm_port_init(struct generic_uart_port*	io_p_port)
{
	int 		        ret = 0;
	unsigned long		irq_flags;

	struct msm_uart_port*	p_msm_port = GEN_UART_TO_MSM(io_p_port);

	MSM_UARTDM_ENTER();

	spin_lock_irqsave(&(io_p_port->lock), irq_flags);


	ret = __msm_uartdm_port_init(io_p_port);

	if (ret) {
		MSM_UARTDM_DEBUG("%s: %s, to initialize udartdm port 0x%xn",
			DRIVER_NAME,
			__PRETTY_FUNCTION__,
			ret);
		goto end;
	}

	spin_unlock_irqrestore(&(io_p_port->lock), irq_flags);

	/* Register for IRQ */
	ret = request_irq(io_p_port->irq,
			msm_uartdm_irq,
			IRQF_TRIGGER_HIGH,
			p_msm_port->name,
			io_p_port);
	if (ret) {
		MSM_UARTDM_DEBUG("%s: %s, failed to register IRQ err 0x%xn",
			DRIVER_NAME,
			__PRETTY_FUNCTION__,
			ret);
		goto end;
	}

	spin_lock_irqsave(&(io_p_port->lock), irq_flags);

	/* Enable Rx/Tx */
	msm_write(	io_p_port,
			UART_DM_CR_RX_ENABLE | UART_DM_CR_TX_ENABLE,
			UART_DM_CR);

	ret = __msm_uartdm_port_init_imr(io_p_port);

end:
	spin_unlock_irqrestore(&(io_p_port->lock), irq_flags);

	MSM_UARTDM_EXIT();
	return ret;
}

static void
__msm_uartdm_set_stale_timeout(struct generic_uart_port* i_p_port)
{
	/* RX stale watermark */
	int watermark;
	int latency = i_p_port->rx_latency;

	BUG_ON( latency == 0 );

	watermark = UART_DM_IPR_STALE_TIMEOUT_LSB_MSK & latency;
	watermark |= UART_DM_IPR_STALE_TIMEOUT_MSB_MSK &
                     ( (latency >> UART_DM_IPR_STALE_TIMEOUT_LSB_SIZE) <<   UART_DM_IPR_STALE_TIMEOUT_MSB_OFFSET );

	msm_write(i_p_port, watermark, UART_DM_IPR);
}
/**
*
* Configure the UARTDM port to the requested baud-rate.
*
* @param[in]       i_p_port - The UART port to configure.
* @param[in]       baud - The requested baud rate.
*
* @return 0 for success -1 otherwise.
*
* @Note	The baud rate is calculated as follows:
*	fundamental_clk / CSR = 16 * baud_rate
*	hence, once the baud_rate is specified, we should pick CSR value and
*	fundamental clk that satisfies the above formula.
*/
static void
__msm_uartdm_set_baud_rate(struct generic_uart_port* i_p_port, unsigned int baud)
{
	u32 csr;
	u32 fund_clk_freq;
	u32 watermark;
	u32 read_clk;

	MSM_UARTDM_DEBUG("%s: %s, enter port_0x%x baud %d\n",
			 DRIVER_NAME,
			 __PRETTY_FUNCTION__
			 ,(unsigned int)i_p_port,
			 baud);

	i_p_port->baud_rate = baud;

	switch (baud) {
	case 4000000:
		fund_clk_freq 	= 64000000;
		csr		= UART_DM_CSR_RX_DIV_1|UART_DM_CSR_TX_DIV_1;
		break;
	case 3686400:
		/* Set the fund_clk in hz */
		fund_clk_freq 	= 58982400;
		csr		= UART_DM_CSR_RX_DIV_1|UART_DM_CSR_TX_DIV_1;
		break;
	case 3000000:
		fund_clk_freq 	= 48000000;
		csr		= UART_DM_CSR_RX_DIV_1|UART_DM_CSR_TX_DIV_1;
		break;			
	case 1843200:
		/* Set the fund_clk in hz */
		fund_clk_freq 	= 58982400;
		csr		= UART_DM_CSR_RX_DIV_2 | UART_DM_CSR_TX_DIV_2;
		break;
	case 1228800:
		/* Set the fund_clk in hz */
		fund_clk_freq 	= 58982400;
		csr		= UART_DM_CSR_RX_DIV_3 | UART_DM_CSR_TX_DIV_3;
		break;
	case 921600:
		/* Set the fund_clk in hz */
		fund_clk_freq 	= 58982400;
		csr		= UART_DM_CSR_RX_DIV_4 | UART_DM_CSR_TX_DIV_4;
		break;
	case 614400:
		/* Set the fund_clk in hz */
		fund_clk_freq 	= 58982400;
		csr		= UART_DM_CSR_RX_DIV_6 | UART_DM_CSR_TX_DIV_6;
		break;
	case 460800:
		/* Set the fund_clk in hz */
		fund_clk_freq 	= 58982400;
		csr		= UART_DM_CSR_RX_DIV_8 | UART_DM_CSR_TX_DIV_8;
		break;
	case 115200:
		/* Set the fund_clk in hz */
		fund_clk_freq 	= 7372800;
		csr		= UART_DM_CSR_RX_DIV_4 | UART_DM_CSR_TX_DIV_4;
		break;
	default:
		MSM_UARTDM_ERR("%s: %s, invalid baud rate specified %d, using default\n",
				DRIVER_NAME,
				__PRETTY_FUNCTION__,
				baud);

		/* Default to 115200 - Set the fund_clk in hz */
		i_p_port->baud_rate = 115200;
		fund_clk_freq 	    = 7372800;
		csr		    = UART_DM_CSR_RX_DIV_4 | UART_DM_CSR_TX_DIV_4;
		break;
	}
	clk_set_rate(i_p_port->p_clk, fund_clk_freq);

	read_clk = clk_get_rate(i_p_port->p_clk);
	if (fund_clk_freq != read_clk) {
		MSM_UARTDM_ERR("%s, error, read_clk(%d), fund_clk_freq(%d)\n",
			__PRETTY_FUNCTION__, read_clk, fund_clk_freq);
	}
	/*
	 * Make sure to update the clk rate in the port structure.
	 */
	i_p_port->clk_rate = read_clk;

	msm_write(i_p_port, csr, UART_DM_CSR);

	__msm_uartdm_set_stale_timeout(i_p_port);

	/* set RX watermark */
	watermark = 32;//(i_p_port->rx_fifo_size * 3) / 16;
	msm_write(i_p_port, watermark, UART_DM_RFWR);

	/* set TX watermark */
	/* TODO: amir this is nice, find better value for Tx watermark if possible */
	msm_write(i_p_port, 32, UART_DM_TFWR);

	MSM_UARTDM_EXIT();
}


/**
*
* Configure the UARTDM port to the requested baud-rate.
*
* @param[in]       i_p_port - The UART port to configure.
* @param[in]       baud - The requested baud rate.
*
* @return 0 for success -1/-ErrCode otherwise.
*
*/
int
msm_uartdm_set_baud_rate(struct generic_uart_port* i_p_port, unsigned int baud)
{
	int				ret	  = 0;
	struct msm_uart_port*		p_msm_port= GEN_UART_TO_MSM(i_p_port);

	unsigned long	flags;

	MSM_UARTDM_ENTER();

	if (NULL != i_p_port) {
		spin_lock_irqsave(&(i_p_port->lock), flags);
		/*
		 * Turn off the UART interrupts
		 */
		msm_write(i_p_port, 0, UART_DM_IMR);

		__msm_uartdm_set_baud_rate(i_p_port, baud);

		/* Reset UART */
		/* TODO: amir - move to a function to do more generic configuraion */
		msm_write(i_p_port,
			  UART_DM_MR2_BITS_PER_CHAR_8 | UART_DM_MR2_STOP_BIT_LEN_ONE | i_p_port->parity_data,
			  UART_DM_MR2);	/* 8N1 */

		msm_write(i_p_port,
			  p_msm_port->imr,
			  UART_DM_IMR);

		if ( RX_MODE_DM(p_msm_port)) {
			msm_write( i_p_port, UART_DM_DMEN_RX_DM_EN, UART_DM_DMEN );
		}

		/* Reset UART */
		__msm_uartdm_reset(i_p_port);

		/* Enable Rx/Tx */
		msm_write(i_p_port,
			  UART_DM_CR_RX_ENABLE | UART_DM_CR_TX_ENABLE,
			  UART_DM_CR);

		spin_unlock_irqrestore(&(i_p_port->lock), flags);
	}
	else {
		ret = -EINVAL;
		MSM_UARTDM_ERR("%s: %s, invalid port ID 0x%x\n",
			DRIVER_NAME, __FUNCTION__, (uint32_t)i_p_port);
	}

	MSM_UARTDM_EXIT();
	return ret;
}
EXPORT_SYMBOL(msm_uartdm_set_baud_rate);

/**
*
* Resets the UARTDM controller.
*
* @param[in]       i_p_port - Pointer to a generic UART structure that
*				corresponds to the port to reset.
*
* @return 0 for success -1 otherwise.
*
*/
static void
__msm_uartdm_reset(struct generic_uart_port* i_p_port)
{
	struct msm_uart_port*	p_msm_port = GEN_UART_TO_MSM(i_p_port);

	MSM_UARTDM_DEBUG("%s: %s enter, port 0x%x\n",
			 DRIVER_NAME,
			 __PRETTY_FUNCTION__,
			 (unsigned int)i_p_port);

	msm_write(i_p_port, UART_DM_CR_CMD_RESET_RX, UART_DM_CR);
	/* Restore rx flow control */
	__msm_uartdm_set_rx_flow(i_p_port, p_msm_port->rx_flow_ctl, p_msm_port->rx_flow_state);

	msm_write(i_p_port, UART_DM_CR_CMD_RESET_TX, UART_DM_CR);
	/* Restore tx flow control */
	__msm_uartdm_set_tx_flow(i_p_port, p_msm_port->tx_flow_ctl);

	msm_write(i_p_port, UART_DM_CR_CMD_RESET_ERR, UART_DM_CR);
	msm_write(i_p_port, UART_DM_CR_CMD_RESET_BCI, UART_DM_CR);
	msm_write(i_p_port, UART_DM_CR_CMD_CLR_CTS, UART_DM_CR);
	msm_write(i_p_port, UART_DM_CR_CMD_CLR_STALE, UART_DM_CR);
	msm_write(i_p_port, UART_DM_CR_CMD_CLR_TX_ERR, UART_DM_CR);


	msm_write(i_p_port, UART_DM_CR_CMD_CLR_TX_ERR, UART_DM_CR);
	msm_write(i_p_port, UART_DM_CR_CMD_CLR_TX_DONE, UART_DM_CR);

	MSM_UARTDM_EXIT();
}

static void
msm_uartdm_reset(struct generic_uart_port* i_p_port)
{
	unsigned long 		irq_flags;

	spin_lock_irqsave(&(i_p_port->lock), irq_flags);

	__msm_uartdm_reset(i_p_port);

	spin_unlock_irqrestore(&(i_p_port->lock), irq_flags);
}

void
msm_uartdm_config_write_size(struct generic_uart_port* i_p_port, int num_bytes)
{
	MSM_UARTDM_ENTER();
	msm_write(i_p_port, num_bytes, UART_DM_NUM_CHARS_FOR_TX);
	MSM_UARTDM_EXIT();
}
EXPORT_SYMBOL(msm_uartdm_config_write_size);

void
msm_uartdm_config_read_size(struct generic_uart_port* i_p_port, int num_bytes)
{
	MSM_UARTDM_ENTER();
	/*
	 * Clear stale event
	 */
	msm_write(i_p_port, UART_DM_CR_CMD_CLR_STALE, UART_DM_CR);
	msm_write(i_p_port, num_bytes, UART_DM_DMRX);
	MSM_UARTDM_EXIT();
}
EXPORT_SYMBOL(msm_uartdm_config_read_size);

int
msm_uartdm_get_received_byte_cnt(struct generic_uart_port* i_p_port)
{
	int ret;
	MSM_UARTDM_ENTER();
	ret = msm_read(i_p_port, UART_DM_RX_TOTAL_SNAP);
	MSM_UARTDM_DEBUG("%s: %s, exit, port 0x%x, rx bytes 0x%x\n",
			 DRIVER_NAME,
			 __PRETTY_FUNCTION__,
			 (unsigned int)i_p_port,
			 ret);
	MSM_UARTDM_EXIT();
	return ret;
}
EXPORT_SYMBOL(msm_uartdm_get_received_byte_cnt);

int
msm_uartdm_rx_dm_config(struct generic_uart_port* i_p_port, uint32_t dst_phys_addr, size_t read_size )
{
	int ret = 0;
	struct msm_dmov_cmd *rx_xfer_ptr;

	MSM_UARTDM_ENTER();

	BUG_ON(!i_p_port);
	i_p_port->rx_dm.command_ptr->num_rows = ((read_size >> 4) << 16) | (read_size >> 4);

	i_p_port->rx_dm.command_ptr->dst_row_addr = dst_phys_addr;

	//BUG_ON(p_context->p_rx_buffer->write_index != 0 );

	*(i_p_port->rx_dm.command_ptr_ptr) = CMD_PTR_LP | DMOV_CMD_ADDR(i_p_port->rx_dm.command_ptr_phys);

	rx_xfer_ptr = &(i_p_port->rx_xfer);

	rx_xfer_ptr->cmdptr = DMOV_CMD_PTR_LIST | DMOV_CMD_ADDR(i_p_port->rx_dm.command_ptr_ptr_phys);

	msm_dmov_enqueue_cmd(i_p_port->dma_rx_channel, rx_xfer_ptr);

	msm_uartdm_config_read_size( i_p_port,  read_size);

	MSM_UARTDM_EXIT();
	return ret;
}
EXPORT_SYMBOL(msm_uartdm_rx_dm_config);

void
msm_uartdm_rx_dm_flush(struct generic_uart_port* i_p_port)
{
	MSM_UARTDM_ENTER();

	msm_dmov_flush(i_p_port->dma_rx_channel);

	MSM_UARTDM_EXIT();
}
EXPORT_SYMBOL(msm_uartdm_rx_dm_flush);

int
msm_uartdm_get_rx_fifo_fullness(struct generic_uart_port* i_p_port, int *o_p_packing_bytes)
{
	int ret;

	MSM_UARTDM_ENTER();

	ret = msm_read(i_p_port, UART_DM_RXFS);

	if ( o_p_packing_bytes ) {
		*o_p_packing_bytes = (ret & UART_DM_RX_BUFFER_STATE_MASK) >> UART_DM_RX_BUFFER_STATE_SHIFT;
	}

	ret = (ret & UART_DM_RX_FIFO_STATE_LSB) | ((ret & UART_DM_RX_FIFO_STATE_MSB) >> 2);


	MSM_UARTDM_EXIT();
	return ret;
}
EXPORT_SYMBOL(msm_uartdm_get_rx_fifo_fullness);
void
msm_uartdm_send_dword(struct generic_uart_port* i_p_port, unsigned int data)
{
	//TODO: see if we can remove the check...
//	while (!(msm_read(i_p_port, UART_DM_SR) & UART_DM_SR_TX_READY))
//		;
	msm_write(i_p_port, data, UART_DM_TF);
}
EXPORT_SYMBOL(msm_uartdm_send_dword);

unsigned int
msm_uartdm_get_dword(struct generic_uart_port* i_p_port)
{
	if (!(msm_read(i_p_port, UART_DM_SR) & UART_DM_SR_RX_READY)) {
		return -1;
	}
	return msm_read(i_p_port, UART_DM_RF);
}
EXPORT_SYMBOL(msm_uartdm_get_dword);

/**
*
* Sets the callback function to be called in case that tx-level event has occured.
* This means that the level in the TX FIFO is below the pre-configured threashold.
*
* @param[in][out]       io_p_port - The UART port to configure.
* @param[in]       	callback  - callback function.
*
* @return 0 for success, -EINVAL for invalid input and -EPERM otherwise.
*
* @Note	Currently we support only one callback function per event at any
* 	given point in time. Note that passing NULL as callback parameter will
*	clear the previous callback.
*	Calling this function with non-NULL 2 consecutive times without
*	clearing the callback function in between will cause the
*	function to fail.
*
*/
int
msm_uartdm_set_tx_level_cbk(struct generic_uart_port* io_p_port,
		       void (* pcallback)(void *pdata),
		       void* pdata)
{
	int 			ret = 0;
	unsigned long 		irq_flags;

	MSM_UARTDM_DEBUG("%s: %s, enter port 0x%x\n",
			 DRIVER_NAME,
			 __PRETTY_FUNCTION__,
			 (unsigned int)io_p_port);

	spin_lock_irqsave(&(io_p_port->lock), irq_flags);
	if (NULL == io_p_port) {
		MSM_UARTDM_ERR("%s, invalid port handle", __PRETTY_FUNCTION__);
		ret = -EINVAL;
	}
	else if ((NULL != pcallback) &&
		 (NULL != io_p_port->p_tx_level_callback) &&
		 (NULL != io_p_port->p_tx_level_data)) {
		MSM_UARTDM_ERR("%s, setting cbk while another cbk is valid is not allowed",
			 __PRETTY_FUNCTION__);
		ret = -EPERM;
	}
	else {
		io_p_port->p_tx_level_callback = pcallback;
		io_p_port->p_tx_level_data = pdata;
	}

	spin_unlock_irqrestore(&(io_p_port->lock), irq_flags);
	MSM_UARTDM_DEBUG("%s: %s, exit, port 0x%x, ret %d\n",
			 DRIVER_NAME,
			 __PRETTY_FUNCTION__,
			 (unsigned int)io_p_port,
			 ret);

	return ret;
}

EXPORT_SYMBOL(msm_uartdm_set_tx_level_cbk);

/**
*
* Sets the callback function to be called in case that tx-ready  event has occured.
* This means that the level in the TX FIFO is empty or if we sent the number of
* characters that were prog before the write transaction.
*
* @param[in][out]       io_p_port - The UART port to configure.
* @param[in]       	callback  - callback function.
*
* @return 0 for success, -EINVAL for invalid input and -EPERM otherwise.
*
* @Note	Currently we support only one callback function per event at any
* 	given point in time. Note that passing NULL as callback parameter will
*	clear the previous callback.
*	Calling this function with non-NULL 2 consecutive times without
*	clearing the callback function in between will cause the
*	function to fail.
*
*/
int
msm_uartdm_set_tx_rdy_cbk(struct generic_uart_port* io_p_port,
		       void (* pcallback)(void *pdata),
		       void* pdata)
{
	int 			ret = 0;
	unsigned long 		irq_flags;

	MSM_UARTDM_DEBUG("%s: %s, enter port 0x%x\n",
			 DRIVER_NAME,
			 __PRETTY_FUNCTION__,
			 (unsigned int)io_p_port);

	spin_lock_irqsave(&(io_p_port->lock), irq_flags);
	if (NULL == io_p_port) {
		MSM_UARTDM_ERR("%s, invalid port handle", __PRETTY_FUNCTION__);
		ret = -EINVAL;
	}
	else if ((NULL != pcallback) &&
		 (NULL != io_p_port->p_tx_rdy_callback) &&
		 (NULL != io_p_port->p_tx_rdy_data)) {
		MSM_UARTDM_ERR("%s, setting cbk while another cbk is valid is not allowed",
			 __PRETTY_FUNCTION__);
		ret = -EPERM;
	}
	else {
		io_p_port->p_tx_rdy_callback = pcallback;
		io_p_port->p_tx_rdy_data = pdata;
	}

	spin_unlock_irqrestore(&(io_p_port->lock), irq_flags);
	MSM_UARTDM_DEBUG("%s: %s, exit, port 0x%x, ret %d\n",
			 DRIVER_NAME,
			 __PRETTY_FUNCTION__,
			 (unsigned int)io_p_port,
			 ret);

	return ret;
}

EXPORT_SYMBOL(msm_uartdm_set_tx_rdy_cbk);

/**
*
* Sets the callback function to be called in case that rx level
* event has occured.
*
* @param[in][out]       io_p_port - The UART port to configure.
* @param[in]       	callback  - callback function.
*
* @return 0 for success, -EINVAL for invalid input and -EPERM otherwise.
*
* @Note	Currently we support only one callback function per event at any
* 	given point in time. Note that passing NULL as callback parameter will
*	clear the previous callback.
*	Calling this function with non-NULL 2 consecutive times without
*	clearing the callback function in between will cause the
*	function to fail.
*
*/
int
msm_uartdm_set_rx_level_cbk(struct generic_uart_port* io_p_port,
		       void (* pcallback)(void *pdata),
		       void* pdata)
{
	int 			ret = 0;
	unsigned long 		irq_flags;

	MSM_UARTDM_DEBUG("%s: %s, enter port 0x%x\n",
			 DRIVER_NAME,
			 __PRETTY_FUNCTION__,
			 (unsigned int)io_p_port);

	spin_lock_irqsave(&(io_p_port->lock), irq_flags);
	if (NULL == io_p_port) {
		MSM_UARTDM_ERR("%s, invalid port handle", __PRETTY_FUNCTION__);
		ret = -EINVAL;
	}
	else if ((NULL != pcallback) &&
		 (NULL != io_p_port->p_rx_level_callback) &&
		 (NULL != io_p_port->p_rx_level_data)) {
		MSM_UARTDM_ERR("%s, setting cbk while another cbk is valid is not allowed",
			 __PRETTY_FUNCTION__);
		ret = -EPERM;
	}
	else {
		io_p_port->p_rx_level_callback = pcallback;
		io_p_port->p_rx_level_data = pdata;
	}

	spin_unlock_irqrestore(&(io_p_port->lock), irq_flags);
	MSM_UARTDM_DEBUG("%s: %s, exit, port 0x%x, ret %d\n",
			 DRIVER_NAME,
			 __PRETTY_FUNCTION__,
			 (unsigned int)io_p_port,
			 ret);

	return ret;
}

EXPORT_SYMBOL(msm_uartdm_set_rx_level_cbk);

/**
*
* Sets the callback function to be called in case that rx data mover ( dma )
* event has occured.
*
* @param[in][out]       io_p_port - The UART port to configure.
* @param[in]       	callback  - callback function.
*
* @return 0 for success, -EINVAL for invalid input and -EPERM otherwise.
*
* @Note	Currently we support only one callback function per event at any
* 	given point in time. Note that passing NULL as callback parameter will
*	clear the previous callback.
*	Calling this function with non-NULL 2 consecutive times without
*	clearing the callback function in between will cause the
*	function to fail.
*
*/
int
msm_uartdm_set_rx_dm_cbk(struct generic_uart_port* io_p_port,
		       void (* pcallback)(void *pdata),
		       void* pdata)
{
	int 			ret = 0;
	unsigned long 		irq_flags;

	MSM_UARTDM_DEBUG("%s: %s, enter port 0x%x\n",
			 DRIVER_NAME,
			 __PRETTY_FUNCTION__,
			 (unsigned int)io_p_port);

	spin_lock_irqsave(&(io_p_port->lock), irq_flags);
	if (NULL == io_p_port) {
		MSM_UARTDM_ERR("%s, invalid port handle", __PRETTY_FUNCTION__);
		ret = -EINVAL;
	}
	else if ((NULL != pcallback) &&
		 (NULL != io_p_port->p_rx_dm_callback) &&
		 (NULL != io_p_port->p_rx_dm_data)) {
		MSM_UARTDM_ERR("%s, setting cbk while another cbk is valid is not allowed",
			 __PRETTY_FUNCTION__);
		ret = -EPERM;
	}
	else {
		io_p_port->p_rx_dm_callback = pcallback;
		io_p_port->p_rx_dm_data = pdata;
	}

	spin_unlock_irqrestore(&(io_p_port->lock), irq_flags);
	MSM_UARTDM_DEBUG("%s: %s, exit, port 0x%x, ret %d\n",
			 DRIVER_NAME,
			 __PRETTY_FUNCTION__,
			 (unsigned int)io_p_port,
			 ret);

	return ret;
}

EXPORT_SYMBOL(msm_uartdm_set_rx_dm_cbk);

/**
*
* Sets the callback function to be called in case that rx stale event
* has occured.
*
* @param[in][out]       io_p_port - The UART port to configure.
* @param[in]       	callback  - callback function.
*
* @return 0 for success, -EINVAL for invalid input and -EPERM otherwise.
*
* @Note	Currently we support only one callback function per event at any
* 	given point in time. Note that passing NULL as callback parameter will
*	clear the previous callback.
*	Calling this function with non-NULL 2 consecutive times without
*	clearing the callback function in between will cause the
*	function to fail.
*
*/
int
msm_uartdm_set_rx_stale_cbk(struct generic_uart_port* io_p_port,
		       void (* pcallback)(void *pdata),
		       void* pdata)
{
	int 			ret = 0;
	unsigned long 		irq_flags;

	MSM_UARTDM_DEBUG("%s: %s, enter port 0x%x\n",
			 DRIVER_NAME,
			 __PRETTY_FUNCTION__,
			 (unsigned int)io_p_port);

	spin_lock_irqsave(&(io_p_port->lock), irq_flags);
	if (NULL == io_p_port) {
		MSM_UARTDM_ERR("%s, invalid port handle", __PRETTY_FUNCTION__);
		ret = -EINVAL;
	}
	else if ((NULL != pcallback) &&
		 (NULL != io_p_port->p_rx_stale_callback) &&
		 (NULL != io_p_port->p_rx_stale_data)) {
		MSM_UARTDM_ERR("%s, setting cbk while another cbk is valid is not allowed",
			 __PRETTY_FUNCTION__);
		ret = -EPERM;
	}
	else {
		io_p_port->p_rx_stale_callback = pcallback;
		io_p_port->p_rx_stale_data = pdata;
	}

	spin_unlock_irqrestore(&(io_p_port->lock), irq_flags);
	MSM_UARTDM_DEBUG("%s: %s, exit, port 0x%x, ret %d\n",
			 DRIVER_NAME,
			 __PRETTY_FUNCTION__,
			 (unsigned int)io_p_port,
			 ret);

	return ret;
}

EXPORT_SYMBOL(msm_uartdm_set_rx_stale_cbk);

static int __devinit
msm_uartdm_probe(struct platform_device *pdev)
{
	struct msm_uart_port*		p_msm_port;
	struct resource*		p_resource;
	struct generic_uart_port*	p_port = NULL;
	int				ret	= 0;
	int				line_number = 0;
	int				i;

	MSM_UARTDM_ENTER();

	if (unlikely(pdev->id < 0 || pdev->id >= UART_NR)) {
		ret =  -ENXIO;
		line_number = __LINE__;
		goto probe_err_invalid_param;
	}

	MSM_UARTDM_INFO("%s: detected port #%d\n", DRIVER_NAME, pdev->id);
//TODO: amir, add alternative function to get the UART control structure from the id
	for (i = 0; i < UARTDM_NUM_PORTS; i++) {
		if (pdev->id == ports_db[i].port.id) {
			p_port = &(ports_db[i].port);
			break;
		}
	}

	if (NULL == p_port) {
		ret =  -ENXIO;
		line_number = __LINE__;
		goto probe_err_invalid_param;
	}

	snprintf(p_port->name, sizeof(p_port->name),
		 "msm_uartdm%d", p_port->id+1);

	p_port->p_device = &pdev->dev;
	p_msm_port = GEN_UART_TO_MSM(p_port);

	p_msm_port->p_clk = clk_get(&pdev->dev, p_msm_port->p_clk_name);
	if (unlikely(NULL == p_msm_port->p_clk)) {
		ret =  -ENXIO;
		line_number = __LINE__;
		goto probe_err_invalid_param;
	}

	p_msm_port->p_pclk = clk_get(&pdev->dev, p_msm_port->p_pclk_name);
	if (IS_ERR(p_msm_port->p_pclk))
		p_msm_port->p_pclk = NULL;


	/*
	 * Get and save the memory mapped area for the UART
	 */
	p_resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(NULL == p_resource)) {
		ret =  -ENXIO;
		line_number = __LINE__;
		goto probe_err_invalid_param;
	}
	p_port->mapbase		= p_resource->start;
	p_port->mem_size	= p_resource->end - p_resource->start + 1;

        p_resource = platform_get_resource_byname( pdev, IORESOURCE_DMA, "uartdm_channels" );
	if (likely(NULL != p_resource)) {
		p_port->dma_tx_channel	= p_resource->start;
		p_port->dma_rx_channel	= p_resource->end;
	}

        p_resource = platform_get_resource_byname( pdev, IORESOURCE_DMA, "uartdm_crci" );
	if (likely(NULL != p_resource)) {
		p_port->dma_tx_crci	= p_resource->start;
		p_port->dma_rx_crci	= p_resource->end;
	}

	MSM_UARTDM_DEBUG("%s: %s mapbase start 0x%x, end 0x%x, size %d\n",
			 DRIVER_NAME,
			 __PRETTY_FUNCTION__,
			 (unsigned int)p_resource->start,
			 (unsigned int)p_resource->end,
			 p_port->mem_size);
	/*
	 * Map phy to virt UART memory mapped area
	 */
	if (unlikely(!request_mem_region(p_port->mapbase, p_port->mem_size, "uartdm"))) {
		ret		= -EBUSY;
		line_number	= __LINE__;
		goto probe_err_invalid_param;
	}
_uartdm_print_port(p_port,__LINE__);
	p_port->p_membase = ioremap(p_port->mapbase, p_port->mem_size);
	if (NULL == p_port->p_membase) {
		release_mem_region(p_port->mapbase, p_port->mem_size);
		ret		= -EBUSY;
		line_number	= __LINE__;
		goto probe_err_invalid_param;
	}
_uartdm_print_port(p_port, __LINE__);

	p_port->irq = platform_get_irq(pdev, 0);

	MSM_UARTDM_DEBUG("%s: %s, irq %d\n",
			 DRIVER_NAME,
			 __PRETTY_FUNCTION__,
			 p_port->irq);

	if (unlikely(p_port->irq < 0)) {
		ret =  -ENXIO;
		line_number = __LINE__;
		goto probe_err_invalid_param;
	}

	MSM_UARTDM_EXIT();

	return ret;
probe_err_invalid_param:
	MSM_UARTDM_ERR("%s, %s invalid param at line %d",
			DRIVER_NAME,
			__PRETTY_FUNCTION__,
			line_number );
	ret = -ENXIO;

	MSM_UARTDM_EXIT();
	return ret;


}

static int __devexit
msm_uartdm_remove(struct platform_device *pdev)
{
	struct msm_uart_port *msm_uart_port = NULL;
	int i;

	if (unlikely(pdev->id < 0 || pdev->id >= UART_NR))
		return -ENXIO;

	for (i = 0; i < UARTDM_NUM_PORTS; i++) {
		if (pdev->id == ports_db[i].port.id) {
			msm_uart_port = GEN_UART_TO_MSM(&(ports_db[i].port));
			break;
		}
	}

	if (!msm_uart_port) {
		return -ENODEV;
	}

	clk_put(msm_uart_port->p_clk);

	if (msm_uart_port->p_pclk) {
		clk_put(msm_uart_port->p_pclk);
	}

	return 0;
}


static struct platform_driver msm_platform_driver = {
	.probe = msm_uartdm_probe,
	.remove = msm_uartdm_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init msm_uartdm_init(void)
{
	int ret;

	MSM_UARTDM_ENTER();

	ret = platform_driver_probe(&msm_platform_driver, msm_uartdm_probe);
	if (unlikely(ret)) {
		MSM_UARTDM_ERR("%s: failed initialization\n", DRIVER_NAME);
	}
	else {
		MSM_UARTDM_INFO("%s: initialized\n", DRIVER_NAME);
	}
	MSM_UARTDM_EXIT();
	return ret;
}

static void __exit msm_uartdm_exit(void)
{
	platform_driver_unregister(&msm_platform_driver);
}

module_init(msm_uartdm_init);
module_exit(msm_uartdm_exit);

MODULE_AUTHOR("Amir Frenkel <amir.frenkel@palm.com>");
MODULE_DESCRIPTION("Driver for msm7x UART DM device");
MODULE_LICENSE("GPL");
