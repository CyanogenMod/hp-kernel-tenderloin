/*
 * arch/arm/mach-msm/msm_uart_dm.h
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

#ifndef __MSM_UART_DM_H__
#define __MSM_UART_DM_H__

/* Register */
#define UART_DM_MR1			(0x0000)

/* BitMask of the fields */
#define UART_DM_MR1_AUTO_RFR_LEVEL0	(0x3F)
#define UART_DM_MR1_AUTO_RFR_LEVEL1	(0xFFFFFF00)
#define UART_MR1_RX_RDY_CTL		(1 << 7)
#define UART_MR1_CTS_CTL       		(1 << 6)

/* Register */
#define UART_DM_MR2			(0x0004)

/* Bit fields value definition and masks */
#define UART_DM_MR2_ERROR_MODE		(1 << 6)

#define UART_DM_MR2_BITS_PER_CHAR_MASK	(0x3 << 4)
#define UART_DM_MR2_BITS_PER_CHAR_5	(0x0 << 4)
#define UART_DM_MR2_BITS_PER_CHAR_6	(0x1 << 4)
#define UART_DM_MR2_BITS_PER_CHAR_7	(0x2 << 4)
#define UART_DM_MR2_BITS_PER_CHAR_8	(0x3 << 4)

#define UART_DM_MR2_STOP_BIT_LEN_MASK	(0x3 << 2)
#define UART_DM_MR2_STOP_BIT_LEN_0_563	(0x0 << 2)
#define UART_DM_MR2_STOP_BIT_LEN_ONE	(0x1 << 2)
#define UART_DM_MR2_STOP_BIT_LEN_1_563	(0x2 << 2)
#define UART_MR2_STOP_BIT_LEN_TWO	(0x3 << 2)

#define UART_DM_MR2_PARITY_MODE_MASK	(0x3 << 0)
#define UART_DM_MR2_PARITY_MODE_NONE	(0x0 << 0)
#define UART_DM_MR2_PARITY_MODE_ODD	(0x1 << 0)
#define UART_DM_MR2_PARITY_MODE_EVEN	(0x2 << 0)
#define UART_DM_MR2_PARITY_MODE_SPACE	(0x3 << 0)


/* Clk selection Register */
#define UART_DM_CSR			(0x0008)
/* 
 * The bellow values are used to divide the 'uart fundemntal clk' 
 * by X - which is the number specificed in the enum
 * TX - CSR clk sel is in bits 0-3
 * Rx - CSR clk sel is in bits 4-7
 */
#define	UART_DM_CSR_TX_DIV_1		(0xF)
#define	UART_DM_CSR_TX_DIV_2		(0xE)
#define	UART_DM_CSR_TX_DIV_3		(0xD)
#define	UART_DM_CSR_TX_DIV_4		(0xC)
#define	UART_DM_CSR_TX_DIV_6		(0xB)
#define	UART_DM_CSR_TX_DIV_8		(0xA)
#define	UART_DM_CSR_TX_DIV_12		(0x9)
#define	UART_DM_CSR_TX_DIV_16		(0x8)
#define	UART_DM_CSR_TX_DIV_24		(0x7)
#define	UART_DM_CSR_TX_DIV_32		(0x6)
#define	UART_DM_CSR_TX_DIV_48		(0x5)
#define	UART_DM_CSR_TX_DIV_96		(0x4)
#define	UART_DM_CSR_TX_DIV_192		(0x3)
#define	UART_DM_CSR_TX_DIV_384		(0x2)
#define	UART_DM_CSR_TX_DIV_768		(0x1)
#define	UART_DM_CSR_TX_DIV_1536		(0x0)

/*
 *
 */
#define	UART_DM_CSR_RX_DIV_1		(0xF << 4)
#define	UART_DM_CSR_RX_DIV_2		(0xE << 4)
#define	UART_DM_CSR_RX_DIV_3		(0xD << 4)
#define	UART_DM_CSR_RX_DIV_4		(0xC << 4)
#define	UART_DM_CSR_RX_DIV_6		(0xB << 4)
#define	UART_DM_CSR_RX_DIV_8		(0xA << 4)
#define	UART_DM_CSR_RX_DIV_12		(0x9 << 4)
#define	UART_DM_CSR_RX_DIV_16		(0x8 << 4)
#define	UART_DM_CSR_RX_DIV_24		(0x7 << 4)
#define	UART_DM_CSR_RX_DIV_32		(0x6 << 4)
#define	UART_DM_CSR_RX_DIV_48		(0x5 << 4)
#define	UART_DM_CSR_RX_DIV_96		(0x4 << 4)
#define	UART_DM_CSR_RX_DIV_192		(0x3 << 4)
#define	UART_DM_CSR_RX_DIV_384		(0x2 << 4)
#define	UART_DM_CSR_RX_DIV_768		(0x1 << 4)
#define	UART_DM_CSR_RX_DIV_1536		(0x0 << 4)


/* 
 * TX - CSR clk sel is in bits 0-3
 * Rx - CSR clk sel is in bits 4-7
 */
#define UART_DM_CSR_RX_OFF			(


/* Command Register */
#define UART_DM_CR			(0x0010)

/*
 *	 CHANNEL COMMAND bit definition 
 */
#define UART_DM_CR_CMD_NULL		(0x0 << 4)
#define UART_DM_CR_CMD_RESET_RX		(0x1 << 4)
#define UART_DM_CR_CMD_RESET_TX		(0x2 << 4)
/* reset error status */
#define UART_DM_CR_CMD_RESET_ERR	(0x3 << 4)
/* reset break change interrupt */
#define UART_DM_CR_CMD_RESET_BCI	(0x4 << 4)
#define UART_DM_CR_CMD_START_BREAK	(0x5 << 4)
#define UART_DM_CR_CMD_STOP_BREAK	(0x6 << 4)
/* clear CTS interrupt */
#define UART_DM_CR_CMD_CLR_CTS		(0x7 << 4)
/* Reset stale interrupt */
#define UART_DM_CR_CMD_CLR_STALE	(0x8 << 4)
/* enable 16x mode */
#define UART_DM_CR_CMD_PACKET_MODE	(0x9 << 4)
/* disable 16x mode */
#define UART_DM_CR_CMD_RESET_MODE	(0xC << 4)
/* assert rx ready (active low) */
#define UART_DM_CR_CMD_SET_RFR		(0xD << 4)
/* deassert receive ready */
#define UART_DM_CR_CMD_RESET_RFR	(0xE << 4)
/* Clear TX err interrupt */
#define UART_DM_CR_CMD_CLR_TX_ERR	(0x10 << 4)
/* Clear TX done interrupt */
#define UART_DM_CR_CMD_CLR_TX_DONE	(0x11 << 4)

#define UART_DM_CR_RX_ENABLE		(0x1 << 0)
#define UART_DM_CR_RX_DISABLE		(0x1 << 1)
#define UART_DM_CR_TX_ENABLE		(0x1 << 2)
#define UART_DM_CR_TX_DISABLE		(0x1 << 3)

/*
 *	GENERAL command definitions
 */
#define UART_DM_CR_GCMD_NULL			(0 << 8)
#define UART_DM_CR_GCMD_CR_PROTECTION_EN	(1 << 8)
#define UART_DM_CR_GCMD_CR_PROTECTION_DIS	(2 << 8)
#define UART_DM_CR_GCMD_RESET_TX_RDY_INT	(3 << 8)
#define UART_DM_CR_GCMD_SW_FORCE_STALE		(4 << 8)
#define UART_DM_CR_GCMD_EN_STALE_EVENT		(5 << 8)
#define UART_DM_CR_GCMD_DIS_STALE_EVENT		(6 << 8)

/* 
 * 	Interrupt Mask Register
 */
#define UART_DM_IMR			(0x0014)

/* IMR bit definitions */

/* Only relevant in UIM/SIM mode */
#define UART_DM_IMR_TX_DONE		(1 << 9)
/* Only used in UIM mode */
#define UART_DM_IMR_TX_ERROR		(1 << 8)

#define UART_DM_IMR_TX_RDY		(1 << 7)
/* Indicates the current state of CTS, never generates interrupt */
#define UART_DM_IMR_CURRENT_CTS		(1 << 6)
#define UART_DM_IMR_DELTA_CTS		(1 << 5)
/* Set when Rx FIFO is above watermark value*/
#define UART_DM_IMR_RX_LEV		(1 << 4)

/* Stale event */
#define UART_DM_IMR_RX_STALE		(1 << 3)

#define UART_DM_IMR_RX_BREAK		(1 << 2)
#define UART_DM_IMR_RX_HUNT		(1 << 1)

/* Set when The Tx FIFO is below or eq the watermark value */
#define UART_DM_IMR_TX_LEV		(1 << 0)

/* 
 *	Interrupt Program Register 
 */
#define UART_DM_IPR			(0x0018)

/* IPR bit definitions */
#define UART_DM_IPR_STALE_TIMEOUT_MSB_OFFSET 7
#define UART_DM_IPR_STALE_TIMEOUT_LSB_SIZE   5

#define UART_DM_IPR_STALE_TIMEOUT_MSB_MSK (0xFFFFFF80)
#define UART_DM_IPR_STALE_TIMEOUT_LSB_MSK (0x1F)
#define UART_DM_IPR_SAMPLE_DATA		  (0x40)


/*
 *	Tx FIFO watermark register 
 */
#define UART_DM_TFWR			(0x001C)

/*
 *	Rx FIFO watermark register 
 */
#define UART_DM_RFWR			(0x0020)

/*
 * 	Hunt character Register 
 */
#define UART_DM_HCR			(0x0024)


/*
 *	Write: In DM mode used as the handshake size (in chars) 
 *		between the FIFO and DM.
 *	Read: Number of bytes received since the last xfer.
 */
#define UART_DM_DMRX			(0x0034)



/*
 * Bits 23:0 - Hold the number of valid characters received
 * since the end of the last rx transaction.
 * End of rx transaction is defined as:
 * - The number of bytes written into DMRX register has been read.
 * or 
 * - Stale event occured.
 */
#define UART_DM_RX_TOTAL_SNAP		(0x0038)

/*
 * Bits 31:10 TX_FIFO_STATE_MSB
 * Bits 9:7 TX_BUFFER_STATE
 * Bits 6:0 TX_FIFO_STATE_LSB
 * The number if valid entries (dword) in the RX-FIFO
 */
#define UART_DM_TXFS			(0x004C)

#define UART_DM_TX_FIFO_STATE_LSB	(0x7F)
#define UART_DM_TX_FIFO_STATE_MSB	(0xFFFFFC00)
#define UART_DM_TX_FIFO_STATE_SHIFT	(10)

/*
 * Bits 31:10 RX_FIFO_STATE_MSB
 * Bits 9:7 RX_BUFFER_STATE
 * Bits 6:0 RX_FIFO_STATE_LSB
 * The number if valid entries (dword) in the RX-FIFO
 */
#define UART_DM_RXFS			(0x0050)

#define UART_DM_RX_FIFO_STATE_LSB	(0x7F)
#define UART_DM_RX_FIFO_STATE_MSB	(0xFFFFFC00)
#define UART_DM_RX_FIFO_STATE_SHIFT	(10)
#define UART_DM_RX_BUFFER_STATE_SHIFT	(7)
#define UART_DM_RX_BUFFER_STATE_MASK	( 7 << UART_DM_RX_BUFFER_STATE_SHIFT)

/*
 * UART_DM_DMEN
 * Data mover enable.
 */
#define UART_DM_DMEN			(0x003C)

/* 
 * UART_DM_DMEN bit definitions
 */
#define UART_DM_DMEN_RX_DM_EN		(1 << 1)
#define UART_DM_DMEN_RX_DM_DIS		(0 << 1)
#define UART_DM_DMEN_TX_DM_EN		(1 << 0)
#define UART_DM_DMEN_TX_DM_DIS		(0 << 0)

/*
 *	The total number of chars for transmission_complete_intr
 */
#define UART_DM_NUM_CHARS_FOR_TX	(0x0040)

/*
 *	The Status Register
 */
#define UART_DM_SR			(0x0008)
#define UART_DM_SR_HUNT_CHAR		(1 << 7)
#define UART_DM_SR_RX_BREAK		(1 << 6)
#define UART_DM_SR_PAR_FRAME_ERR	(1 << 5)
#define UART_DM_SR_OVERRUN		(1 << 4)
#define UART_DM_SR_TX_EMPTY		(1 << 3)
#define UART_DM_SR_TX_READY		(1 << 2)
#define UART_DM_SR_RX_FULL		(1 << 1)
#define UART_DM_SR_RX_READY		(1 << 0)

/* 
 *	Masked Interrupt status register
 */
#define UART_DM_MISR			(0x0010)

/*
 *	Interrupt status register
 */
#define UART_DM_ISR			(0x0014)

/* Bit fields definition of ISR */
#define UART_DM_ISR_TX_DONE		(1 << 9)
#define UART_DM_ISR_TX_ERR		(1 << 8)
#define UART_DM_ISR_TX_READY		(1 << 7)
#define UART_DM_ISR_CURRENT_CTS		(1 << 6)
#define UART_DM_ISR_DELTA_CTS		(1 << 5)
#define UART_DM_ISR_RX_LEV		(1 << 4)
#define UART_DM_ISR_RX_STALE		(1 << 3)
#define UART_DM_ISR_RX_BREAK		(1 << 2)
#define UART_DM_ISR_RX_HUNT		(1 << 1)
#define UART_DM_ISR_TX_LEV		(1 << 0)


/*
 *	*** Write ***
 * Addr 0x70-0x80 is used for TX FIFO
 */ 
#define UART_DM_TF		(0x0070)
#define UART_DM_TF2		(0x0074)
#define UART_DM_TF3		(0x0078)
#define UART_DM_TF4		(0x007C)

/*
 *	*** Read ***
 * Addr 0x70-0x80 is used for RX FIFO
 */ 
#define UART_DM_RF		(0x0070)
#define UART_DM_RF2		(0x0074)
#define UART_DM_RF3		(0x0078)
#define UART_DM_RF4		(0x007C)


#define UART_DM_MODE_RX_PIO      (1 << 0) 
#define UART_DM_MODE_RX_DM       (1 << 1) 

struct msm_uart_port {
	char			name[16];
	struct clk*		p_clk;
	struct clk*		p_pclk;
	const char*		p_clk_name;
	const char*		p_pclk_name;

	int flags; 
	/*
	 * Pointer to the device structure from the board file
	 */
	struct device*		p_device;

	/*
	 * id - used for debug and identification purpose
	 */
	u32			id;

	/*
	 * The IRQ number to be used for UART
	 */
	/* TODO:move irq to generic generic_uart_port structure.*/
	unsigned int		irq;

	/*
	 * The UART clock rate
	 */
	unsigned int		clk_rate;

	/* TODO:move fifo size to generic generic_uart_port structure.*/
	/* 
	 * The size of the rx/tx fifo in bytes, we assume same size for 
	 * rx and tx
	 */
	int			rx_fifo_size;
	int			tx_fifo_size;

	/*
	 * The memory base address to be used for uart access in the driver.
	 * this pointer was generated by mapping a physical memory mapped area.
	 */
	unsigned char __iomem*	p_membase;

	/*
	 * The base physical address for uart mem-mapped registers, should be
	 * mapped into p_membase to be able to use it by the driver
	 */
	unsigned int		mapbase;

	/*
	 * The size in bytes of the UART memory mapped area
	 */
	unsigned int		mem_size;
	
	/* 
	 * 	port lock 
	 */
	spinlock_t		lock;	

	/*
	 * The desired port baud rate
	 */
	unsigned int		baud_rate;		

	/*
	 * Port parity data
	 */
	uint32_t	        parity_data;	
	/* 
	 * Write-only registers, hence we keep a shadow register to 
	 * tell us the last value written to the HW was
	 */
	u32			imr;

	/* 
	 * RX DMA related structure
	 */
	struct {

		dmov_box *command_ptr;
		dma_addr_t command_ptr_phys;
		u32 *command_ptr_ptr;
		dma_addr_t command_ptr_ptr_phys;

	} rx_dm;

	int dma_rx_channel;
	int dma_tx_channel;
	
	int dma_rx_crci;
	int dma_tx_crci;

	int rx_latency;

	int rx_flow_ctl;
	int rx_flow_state;

	int tx_flow_ctl;

	struct msm_dmov_cmd rx_xfer;
	
	/*
	 * callback zone...
	 */
	int     (*p_board_pin_mux_cb) ( int on );
	int     (*p_board_rts_pin_deassert_cb) ( int deassert );

	void 	(* p_tx_level_callback)(void *pdata);
	void*	p_tx_level_data;
	void 	(* p_tx_rdy_callback)(void *pdata);
	void*	p_tx_rdy_data;

	void	(* p_rx_stale_callback)(void *pdata);
	void*	p_rx_stale_data;
	void	(* p_rx_level_callback)(void *pdata);
	void*	p_rx_level_data;

	void	(* p_rx_dm_callback)(void *pdata);
	void*	p_rx_dm_data;

};

#define generic_uart_port	 msm_uart_port

#define GEN_UART_TO_MSM(generic_uart_port)	\
			((struct msm_uart_port *) generic_uart_port)

/*
 * Generic UART configuration parameters.
 */
struct generic_uart_config {
	/*
	 * The phys UART port ID.
	 * Correspond to the UART port ID in the board file.
	 */
	int port_id;

	int flags;

	int rx_latency; 

	int (*p_board_pin_mux_cb) ( int on );
	int (*p_board_config_gsbi_cb) ( void );
	int (*p_board_rts_pin_deassert_cb) ( int deassert );
};

typedef enum {
	MSM_UARTDM_PARITY_ODD = 1,
	MSM_UARTDM_PARITY_EVEN,
	MSM_UARTDM_PARITY_NONE
} msm_uartdm_parity_t;
int
msm_uartdm_port_open(struct generic_uart_config* i_p_config,
		     struct generic_uart_port** o_pp_port);

int
msm_uartdm_port_close(struct generic_uart_port* io_p_port);

int
msm_uartdm_port_suspend(struct generic_uart_port* io_p_port);

int
msm_uartdm_port_resume(struct generic_uart_port* io_p_uart_port);

int
msm_uartdm_port_init(struct generic_uart_port*	io_p_uart_port);

void 
msm_uartdm_enable_tx_level(struct generic_uart_port* port);

int 
msm_uartdm_disable_tx_level(struct generic_uart_port* port);

int 
msm_uartdm_disable_tx_rdy(struct generic_uart_port* port);

void 
msm_uartdm_enable_tx_rdy(struct generic_uart_port* port);

void 
msm_uartdm_enable_rx_irqs(struct generic_uart_port* port, int enable_stale);

int 
msm_uartdm_disable_rx_irqs(struct generic_uart_port* port);

void 
msm_uartdm_enable_tx(struct generic_uart_port* port);

void 
msm_uartdm_disable_tx(struct generic_uart_port* port);

void
msm_uartdm_reset_tx(struct generic_uart_port* port);

void 
msm_uartdm_enable_rx(struct generic_uart_port* port);

void 
msm_uartdm_disable_rx(struct generic_uart_port* port);

void
msm_uartdm_reset_rx(struct generic_uart_port* port);

int 
msm_uartdm_set_tx_level_cbk(struct generic_uart_port* io_p_port,
		       void (* pcallback)(void *pdata),
		       void* pdata);
int 
msm_uartdm_set_tx_rdy_cbk(struct generic_uart_port* io_p_port,
		       void (* pcallback)(void *pdata),
		       void* pdata);
int 
msm_uartdm_set_baud_rate(struct generic_uart_port* i_p_port, 
			uint32_t baud_rate);

int 
msm_uartdm_set_rx_level_cbk(struct generic_uart_port* io_p_port,
		       void (* pcallback)(void *pdata),
		       void* pdata);
int 
msm_uartdm_set_rx_dm_cbk(struct generic_uart_port* io_p_port,
		       void (* pcallback)(void *pdata),
		       void* pdata);

int 
msm_uartdm_set_rx_stale_cbk(struct generic_uart_port* io_p_port,
		       void (* pcallback)(void *pdata),
		       void* pdata);
int
msm_uartdm_tx_ready(struct generic_uart_port* i_p_port);

void
msm_uartdm_config_write_size(struct generic_uart_port* i_p_port, int num_bytes);

void
msm_uartdm_config_read_size(struct generic_uart_port* i_p_port, int num_bytes);

int
msm_uartdm_get_received_byte_cnt(struct generic_uart_port* i_p_port);
int 
msm_uartdm_get_rx_fifo_fullness(struct generic_uart_port* i_p_port, int *o_p_packing_bytes);

void 
msm_uartdm_send_dword(struct generic_uart_port* i_p_port, unsigned int data);

int
msm_uartdm_rx_ready(struct generic_uart_port* i_p_port);
unsigned int
msm_uartdm_get_dword(struct generic_uart_port* i_p_port);

int
msm_uartdm_rx_dm_config(struct generic_uart_port* i_p_port, uint32_t dst_phys_addr, size_t read_size );

void 
msm_uartdm_rx_dm_flush(struct generic_uart_port* i_p_port);

// TODO: I am here for debug, remove me :)
unsigned int msm_uartdm_read_reg(struct generic_uart_port* i_p_port, int addr);

void 
msm_uartdm_set_rx_flow(struct generic_uart_port* i_p_port, uint32_t flow_ctl, uint32_t flow_state);

void 
msm_uartdm_set_tx_flow(struct generic_uart_port* i_p_port, uint32_t flow_ctl);

void 
msm_uartdm_set_parity(struct generic_uart_port* i_p_port,
		      msm_uartdm_parity_t parity);

#endif	/* __MSM_UART_DM_H__ */
