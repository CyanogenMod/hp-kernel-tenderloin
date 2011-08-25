/* Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
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
 *
 */
/*
 * SPI driver for Qualcomm MSM platforms
 *
 */
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <mach/msm_spi.h>
#include <linux/dma-mapping.h>
#include <linux/sched.h>
#include <mach/dma.h>
#include <asm/atomic.h>
#include <linux/mutex.h>
#include <linux/remote_spinlock.h>
#include <linux/pm_qos_params.h>

#define SPI_DRV_NAME                  "spi_qsd"
#if defined(CONFIG_SPI_QSD) || defined(CONFIG_SPI_QSD_MODULE)

#define QSD_REG(x) (x)
#define QUP_REG(x)

#define SPI_FIFO_WORD_CNT             0x0048

#elif defined(CONFIG_SPI_QUP) || defined(CONFIG_SPI_QUP_MODULE)

#define QSD_REG(x)
#define QUP_REG(x) (x)

#define QUP_CONFIG                    0x0000 /* N & NO_INPUT/NO_OUPUT bits */
#define QUP_ERROR_FLAGS               0x0308
#define QUP_ERROR_FLAGS_EN            0x030C
#define QUP_ERR_MASK                  0x3
#define SPI_OUTPUT_FIFO_WORD_CNT      0x010C
#define SPI_INPUT_FIFO_WORD_CNT       0x0214
#define QUP_MX_WRITE_COUNT            0x0150
#define QUP_MX_WRITE_CNT_CURRENT      0x0154

#define QUP_CONFIG_SPI_MODE           0x0100

#define GSBI_CTRL_REG                 0x0
#define GSBI_SPI_CONFIG               0x30
#endif

#define SPI_CONFIG                    QSD_REG(0x0000) QUP_REG(0x0300)
#define SPI_IO_CONTROL                QSD_REG(0x0004) QUP_REG(0x0304)
#define SPI_IO_MODES                  QSD_REG(0x0008) QUP_REG(0x0008)
#define SPI_SW_RESET                  QSD_REG(0x000C) QUP_REG(0x000C)
#define SPI_TIME_OUT                  QSD_REG(0x0010) QUP_REG(0x0010)
#define SPI_TIME_OUT_CURRENT          QSD_REG(0x0014) QUP_REG(0x0014)
#define SPI_MX_OUTPUT_COUNT           QSD_REG(0x0018) QUP_REG(0x0100)
#define SPI_MX_OUTPUT_CNT_CURRENT     QSD_REG(0x001C) QUP_REG(0x0104)
#define SPI_MX_INPUT_COUNT            QSD_REG(0x0020) QUP_REG(0x0200)
#define SPI_MX_INPUT_CNT_CURRENT      QSD_REG(0x0024) QUP_REG(0x0204)
#define SPI_MX_READ_COUNT             QSD_REG(0x0028) QUP_REG(0x0208)
#define SPI_MX_READ_CNT_CURRENT       QSD_REG(0x002C) QUP_REG(0x020C)
#define SPI_OPERATIONAL               QSD_REG(0x0030) QUP_REG(0x0018)
#define SPI_ERROR_FLAGS               QSD_REG(0x0034) QUP_REG(0x001C)
#define SPI_ERROR_FLAGS_EN            QSD_REG(0x0038) QUP_REG(0x0020)
#define SPI_DEASSERT_WAIT             QSD_REG(0x003C) QUP_REG(0x0310)
#define SPI_OUTPUT_DEBUG              QSD_REG(0x0040) QUP_REG(0x0108)
#define SPI_INPUT_DEBUG               QSD_REG(0x0044) QUP_REG(0x0210)
#define SPI_TEST_CTRL                 QSD_REG(0x004C) QUP_REG(0x0024)
#define SPI_OUTPUT_FIFO               QSD_REG(0x0100) QUP_REG(0x0110)
#define SPI_INPUT_FIFO                QSD_REG(0x0200) QUP_REG(0x0218)
#define SPI_STATE                     QSD_REG(SPI_OPERATIONAL) QUP_REG(0x0004)

/* SPI_CONFIG fields */
#define SPI_CFG_INPUT_FIRST           0x00000200
#define SPI_NO_INPUT                  0x00000080
#define SPI_NO_OUTPUT                 0x00000040
#define SPI_CFG_LOOPBACK              0x00000100
#define SPI_CFG_N                     0x0000001F

/* SPI_IO_CONTROL fields */
#define SPI_IO_C_CLK_IDLE_HIGH        0x00000400
#define SPI_IO_C_MX_CS_MODE           0x00000100
#define SPI_IO_C_CS_N_POLARITY        0x000000F0
#define SPI_IO_C_CS_N_POLARITY_0      0x00000010
#define SPI_IO_C_CS_SELECT            0x0000000C
#define SPI_IO_C_TRISTATE_CS          0x00000002
#define SPI_IO_C_NO_TRI_STATE         0x00000001

/* SPI_IO_MODES fields */
#define SPI_IO_M_OUTPUT_BIT_SHIFT_EN  QSD_REG(0x00004000) QUP_REG(0x00010000)
#define SPI_IO_M_PACK_EN              QSD_REG(0x00002000) QUP_REG(0x00008000)
#define SPI_IO_M_UNPACK_EN            QSD_REG(0x00001000) QUP_REG(0x00004000)
#define SPI_IO_M_INPUT_MODE           QSD_REG(0x00000C00) QUP_REG(0x00003000)
#define SPI_IO_M_OUTPUT_MODE          QSD_REG(0x00000300) QUP_REG(0x00000C00)
#define SPI_IO_M_INPUT_FIFO_SIZE      QSD_REG(0x000000C0) QUP_REG(0x00000380)
#define SPI_IO_M_INPUT_BLOCK_SIZE     QSD_REG(0x00000030) QUP_REG(0x00000060)
#define SPI_IO_M_OUTPUT_FIFO_SIZE     QSD_REG(0x0000000C) QUP_REG(0x0000001C)
#define SPI_IO_M_OUTPUT_BLOCK_SIZE    QSD_REG(0x00000003) QUP_REG(0x00000003)

#define INPUT_BLOCK_SZ_SHIFT          QSD_REG(4)          QUP_REG(5)
#define INPUT_FIFO_SZ_SHIFT           QSD_REG(6)          QUP_REG(7)
#define OUTPUT_BLOCK_SZ_SHIFT         QSD_REG(0)          QUP_REG(0)
#define OUTPUT_FIFO_SZ_SHIFT          QSD_REG(2)          QUP_REG(2)
#define OUTPUT_MODE_SHIFT             QSD_REG(8)          QUP_REG(10)
#define INPUT_MODE_SHIFT              QSD_REG(10)         QUP_REG(12)

/* SPI_OPERATIONAL fields */
#define SPI_OP_MAX_INPUT_DONE_FLAG    0x00000800
#define SPI_OP_MAX_OUTPUT_DONE_FLAG   0x00000400
#define SPI_OP_INPUT_SERVICE_FLAG     0x00000200
#define SPI_OP_OUTPUT_SERVICE_FLAG    0x00000100
#define SPI_OP_INPUT_FIFO_FULL        0x00000080
#define SPI_OP_OUTPUT_FIFO_FULL       0x00000040
#define SPI_OP_IP_FIFO_NOT_EMPTY      0x00000020
#define SPI_OP_OP_FIFO_NOT_EMPTY      0x00000010
#define SPI_OP_STATE_VALID            0x00000004
#define SPI_OP_STATE                  0x00000003

#define SPI_OP_STATE_CLEAR_BITS       0x2
enum msm_spi_state {
	SPI_OP_STATE_RESET = 0x00000000,
	SPI_OP_STATE_RUN   = 0x00000001,
	SPI_OP_STATE_PAUSE = 0x00000003,
};

/* SPI_ERROR_FLAGS fields */
#define SPI_ERR_OUTPUT_OVER_RUN_ERR   0x00000020
#define SPI_ERR_INPUT_UNDER_RUN_ERR   0x00000010
#define SPI_ERR_OUTPUT_UNDER_RUN_ERR  0x00000008
#define SPI_ERR_INPUT_OVER_RUN_ERR    0x00000004
#define SPI_ERR_CLK_OVER_RUN_ERR      0x00000002
#define SPI_ERR_CLK_UNDER_RUN_ERR     0x00000001
#define SPI_ERR_MASK                  0x0000007F

/* We don't allow transactions larger than 4K-64 or 64K-64 due to
   mx_input/output_cnt register size */
#define SPI_MAX_TRANSFERS             QSD_REG(0xFC0) QUP_REG(0xFC0)
#define SPI_MAX_LEN                   (SPI_MAX_TRANSFERS * dd->bytes_per_word)

#define SPI_NUM_CHIPSELECTS           4
#define SPI_SUPPORTED_MODES  (SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LOOP)

#define SPI_DELAY_THRESHOLD           1
/* Default timeout is 10 milliseconds */
#define SPI_DEFAULT_TIMEOUT           10
/* 250 microseconds */
#define SPI_TRYLOCK_DELAY             250

/* Data Mover burst size */
#define DM_BURST_SIZE                 16
/* Data Mover commands should be aligned to 64 bit(8 bytes) */
#define DM_BYTE_ALIGN                 8

enum msm_spi_mode {
	SPI_FIFO_MODE  = 0x0,  /* 00 */
	SPI_BLOCK_MODE = 0x1,  /* 01 */
	SPI_DMOV_MODE  = 0x2,  /* 10 */
	SPI_MODE_NONE  = 0xFF, /* invalid value */
};

/* Structures for Data Mover */
struct spi_dmov_cmd {
	dmov_box box;      /* data aligned to max(dm_burst_size, block_size)
							   (<= fifo_size) */
	dmov_s single_pad; /* data unaligned to max(dm_burst_size, block_size)
			      padded to fit */
	dma_addr_t cmd_ptr;
};

MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.3");
MODULE_ALIAS("platform:"SPI_DRV_NAME);

static struct pm_qos_request_list *qos_req_list;

#ifdef CONFIG_DEBUG_FS
/* Used to create debugfs entries */
static const struct {
	const char *name;
	mode_t mode;
	int offset;
} debugfs_spi_regs[] = {
	{"config",                S_IRUGO | S_IWUSR, SPI_CONFIG},
	{"io_control",            S_IRUGO | S_IWUSR, SPI_IO_CONTROL},
	{"io_modes",              S_IRUGO | S_IWUSR, SPI_IO_MODES},
	{"sw_reset",                        S_IWUSR, SPI_SW_RESET},
	{"time_out",              S_IRUGO | S_IWUSR, SPI_TIME_OUT},
	{"time_out_current",      S_IRUGO,           SPI_TIME_OUT_CURRENT},
	{"mx_output_count",       S_IRUGO | S_IWUSR, SPI_MX_OUTPUT_COUNT},
	{"mx_output_cnt_current", S_IRUGO,           SPI_MX_OUTPUT_CNT_CURRENT},
	{"mx_input_count",        S_IRUGO | S_IWUSR, SPI_MX_INPUT_COUNT},
	{"mx_input_cnt_current",  S_IRUGO,           SPI_MX_INPUT_CNT_CURRENT},
	{"mx_read_count",         S_IRUGO | S_IWUSR, SPI_MX_READ_COUNT},
	{"mx_read_cnt_current",   S_IRUGO,           SPI_MX_READ_CNT_CURRENT},
	{"operational",           S_IRUGO | S_IWUSR, SPI_OPERATIONAL},
	{"error_flags",           S_IRUGO | S_IWUSR, SPI_ERROR_FLAGS},
	{"error_flags_en",        S_IRUGO | S_IWUSR, SPI_ERROR_FLAGS_EN},
	{"deassert_wait",         S_IRUGO | S_IWUSR, SPI_DEASSERT_WAIT},
	{"output_debug",          S_IRUGO,           SPI_OUTPUT_DEBUG},
	{"input_debug",           S_IRUGO,           SPI_INPUT_DEBUG},
	{"test_ctrl",             S_IRUGO | S_IWUSR, SPI_TEST_CTRL},
	{"output_fifo",                     S_IWUSR, SPI_OUTPUT_FIFO},
	{"input_fifo" ,           S_IRUSR,           SPI_INPUT_FIFO},
	{"spi_state",             S_IRUGO | S_IWUSR, SPI_STATE},
#if defined(CONFIG_SPI_QSD) || defined(CONFIG_SPI_QSD_MODULE)
	{"fifo_word_cnt",         S_IRUGO,           SPI_FIFO_WORD_CNT},
#elif defined(CONFIG_SPI_QUP) || defined(CONFIG_SPI_QUP_MODULE)
	{"qup_config",            S_IRUGO | S_IWUSR, QUP_CONFIG},
	{"qup_error_flags",       S_IRUGO | S_IWUSR, QUP_ERROR_FLAGS},
	{"qup_error_flags_en",    S_IRUGO | S_IWUSR, QUP_ERROR_FLAGS_EN},
	{"mx_write_cnt",          S_IRUGO | S_IWUSR, QUP_MX_WRITE_COUNT},
	{"mx_write_cnt_current",  S_IRUGO,           QUP_MX_WRITE_CNT_CURRENT},
	{"output_fifo_word_cnt",  S_IRUGO,           SPI_OUTPUT_FIFO_WORD_CNT},
	{"input_fifo_word_cnt",   S_IRUGO,           SPI_INPUT_FIFO_WORD_CNT},
#endif
};
#endif

struct msm_spi {
	u8                      *read_buf;
	const u8                *write_buf;
	void __iomem            *base;
	void __iomem            *gsbi_base;
	struct device           *dev;
	spinlock_t               queue_lock;
	struct mutex             core_lock;
	struct list_head         queue;
	struct workqueue_struct *workqueue;
	struct work_struct       work_data;
	struct spi_message      *cur_msg;
	struct spi_transfer     *cur_transfer;
	struct completion        transfer_complete;
	struct clk              *clk;
	struct clk              *pclk;
	unsigned long            mem_phys_addr;
	size_t                   mem_size;
	unsigned long            gsbi_mem_phys_addr;
	size_t                   gsbi_mem_size;
	int                      input_fifo_size;
	int                      output_fifo_size;
	u32                      rx_bytes_remaining;
	u32                      tx_bytes_remaining;
	u32                      clock_speed;
	u32                      irq_in;
	int                      read_xfr_cnt;
	int                      write_xfr_cnt;
	int                      write_len;
	int                      read_len;
#if defined(CONFIG_SPI_QSD) || defined(CONFIG_SPI_QSD_MODULE)
	u32                      irq_out;
	u32                      irq_err;
#endif
	int                      bytes_per_word;
	bool                     suspended;
	bool                     transfer_pending;
	wait_queue_head_t        continue_suspend;
	/* DMA data */
	enum msm_spi_mode        mode;
	bool                     use_dma;
	int                      tx_dma_chan;
	int                      tx_dma_crci;
	int                      rx_dma_chan;
	int                      rx_dma_crci;
	/* Data Mover Commands */
	struct spi_dmov_cmd      *tx_dmov_cmd;
	struct spi_dmov_cmd      *rx_dmov_cmd;
	/* Physical address of the tx dmov box command */
	dma_addr_t               tx_dmov_cmd_dma;
	dma_addr_t               rx_dmov_cmd_dma;
	struct msm_dmov_cmd      tx_hdr;
	struct msm_dmov_cmd      rx_hdr;
	int                      input_block_size;
	int                      output_block_size;
	int                      burst_size;
	atomic_t                 rx_irq_called;
	/* Used to pad messages unaligned to block size */
	u8                       *tx_padding;
	dma_addr_t               tx_padding_dma;
	u8                       *rx_padding;
	dma_addr_t               rx_padding_dma;
	u32                      unaligned_len;
	/* DMA statistics */
	int                      stat_dmov_tx_err;
	int                      stat_dmov_rx_err;
	int                      stat_rx;
	int                      stat_dmov_rx;
	int                      stat_tx;
	int                      stat_dmov_tx;
#ifdef CONFIG_DEBUG_FS
	struct dentry *dent_spi;
	struct dentry *debugfs_spi_regs[ARRAY_SIZE(debugfs_spi_regs)];
#endif
	struct msm_spi_platform_data *pdata; /* Platform data */
	/* Remote Spinlock Data */
	bool                     use_rlock;
	remote_mutex_t           r_lock;
	uint32_t                 pm_lat;
	/* When set indicates a write followed by read transfer */
	bool                     multi_xfr;
	bool                     done;
};

/* Forward declaration */
static irqreturn_t msm_spi_input_irq(int irq, void *dev_id);
static irqreturn_t msm_spi_output_irq(int irq, void *dev_id);
static irqreturn_t msm_spi_error_irq(int irq, void *dev_id);
static inline int msm_spi_set_state(struct msm_spi *dd,
				    enum msm_spi_state state);
static void msm_spi_write_word_to_fifo(struct msm_spi *dd);
static inline void msm_spi_write_rmn_to_fifo(struct msm_spi *dd);

#if defined(CONFIG_SPI_QSD) || defined(CONFIG_SPI_QSD_MODULE)
/* Interrupt Handling */
static inline int msm_spi_get_irq_data(struct msm_spi *dd,
				       struct platform_device *pdev)
{
	dd->irq_in  = platform_get_irq_byname(pdev, "spi_irq_in");
	dd->irq_out = platform_get_irq_byname(pdev, "spi_irq_out");
	dd->irq_err = platform_get_irq_byname(pdev, "spi_irq_err");
	if ((dd->irq_in < 0) || (dd->irq_out < 0) || (dd->irq_err < 0))
		return -1;
	return 0;
}

static inline int msm_spi_get_gsbi_resource(struct msm_spi *dd,
					    struct platform_device *pdev)
{
	return 0;
}

static inline int msm_spi_request_gsbi(struct msm_spi *dd) { return 0; }
static inline void msm_spi_release_gsbi(struct msm_spi *dd) {}
static inline void msm_spi_init_gsbi(struct msm_spi *dd) {}

static inline void msm_spi_disable_irqs(struct msm_spi *dd)
{
	disable_irq(dd->irq_in);
	disable_irq(dd->irq_out);
	disable_irq(dd->irq_err);
}

static inline void msm_spi_enable_irqs(struct msm_spi *dd)
{
	enable_irq(dd->irq_in);
	enable_irq(dd->irq_out);
	enable_irq(dd->irq_err);
}

static inline int msm_spi_request_irq(struct msm_spi *dd,
				      const char *name,
				      struct spi_master *master)
{
	int rc;
	rc = request_irq(dd->irq_in, msm_spi_input_irq, IRQF_TRIGGER_RISING,
			   name, dd);
	if (rc)
		goto error_irq1;
	rc = request_irq(dd->irq_out, msm_spi_output_irq, IRQF_TRIGGER_RISING,
			   name, dd);
	if (rc)
		goto error_irq2;
	rc = request_irq(dd->irq_err, msm_spi_error_irq, IRQF_TRIGGER_RISING,
			   name, master);
	if (rc)
		goto error_irq3;
	return 0;

error_irq3:
	free_irq(dd->irq_out, dd);
error_irq2:
	free_irq(dd->irq_in, dd);
error_irq1:
	return rc;
}

static inline void msm_spi_free_irq(struct msm_spi *dd,
				    struct spi_master *master)
{
	free_irq(dd->irq_err, master);
	free_irq(dd->irq_out, dd);
	free_irq(dd->irq_in, dd);
}

static inline void msm_spi_get_clk_err(struct msm_spi *dd, u32 *spi_err) {}
static inline void msm_spi_ack_clk_err(struct msm_spi *dd) {}
static inline void msm_spi_set_qup_config(struct msm_spi *dd, int bpw) {}

static inline int msm_spi_prepare_for_write(struct msm_spi *dd) { return 0; }
static inline void msm_spi_start_write(struct msm_spi *dd, u32 read_count)
{
	msm_spi_write_word_to_fifo(dd);
}
static inline void msm_spi_set_write_count(struct msm_spi *dd, int val) {}

static inline void msm_spi_complete(struct msm_spi *dd)
{
	complete(&dd->transfer_complete);
}

#elif defined(CONFIG_SPI_QUP) || defined(CONFIG_SPI_QUP_MODULE)

/* Interrupt Handling */
/* In QUP the same interrupt line is used for intput, output and error*/
static inline int msm_spi_get_irq_data(struct msm_spi *dd,
				       struct platform_device *pdev)
{
	dd->irq_in  = platform_get_irq_byname(pdev, "spi_irq_in");
	if (dd->irq_in < 0)
		return -1;
	return 0;
}

static inline int msm_spi_get_gsbi_resource(struct msm_spi *dd,
				       struct platform_device *pdev)
{
	struct resource *resource;

	resource  = platform_get_resource_byname(pdev,
						 IORESOURCE_MEM, "gsbi_base");
	if (!resource)
		return -ENXIO;
	dd->gsbi_mem_phys_addr = resource->start;
	dd->gsbi_mem_size = resource_size(resource);

	return 0;
}

static inline void msm_spi_release_gsbi(struct msm_spi *dd)
{
	iounmap(dd->gsbi_base);
	release_mem_region(dd->gsbi_mem_phys_addr, dd->gsbi_mem_size);
}

static inline int msm_spi_request_gsbi(struct msm_spi *dd)
{
	if (!request_mem_region(dd->gsbi_mem_phys_addr, dd->gsbi_mem_size,
				SPI_DRV_NAME)) {
		return -ENXIO;
	}
	dd->gsbi_base = ioremap(dd->gsbi_mem_phys_addr, dd->gsbi_mem_size);
	if (!dd->gsbi_base) {
		release_mem_region(dd->gsbi_mem_phys_addr, dd->gsbi_mem_size);
		return -ENXIO;
	}
	return 0;
}

static inline void msm_spi_init_gsbi(struct msm_spi *dd)
{
	/* Set GSBI to SPI mode, and CRCI_MUX_CTRL to SPI CRCI ports */
	writel_relaxed(GSBI_SPI_CONFIG, dd->gsbi_base + GSBI_CTRL_REG);
}

/* Figure which irq occured and call the relevant functions */
static irqreturn_t msm_spi_qup_irq(int irq, void *dev_id)
{
	u32 op, ret = IRQ_NONE;
	struct msm_spi *dd = dev_id;

	if (readl_relaxed(dd->base + SPI_ERROR_FLAGS) ||
	    readl_relaxed(dd->base + QUP_ERROR_FLAGS)) {
		struct spi_master *master = dev_get_drvdata(dd->dev);
		ret |= msm_spi_error_irq(irq, master);
	}

	op = readl_relaxed(dd->base + SPI_OPERATIONAL);
	if (op & SPI_OP_INPUT_SERVICE_FLAG) {
		writel_relaxed(SPI_OP_INPUT_SERVICE_FLAG,
			       dd->base + SPI_OPERATIONAL);
		ret |= msm_spi_input_irq(irq, dev_id);
	}

	if (op & SPI_OP_OUTPUT_SERVICE_FLAG) {
		writel_relaxed(SPI_OP_OUTPUT_SERVICE_FLAG,
			       dd->base + SPI_OPERATIONAL);
		ret |= msm_spi_output_irq(irq, dev_id);
	}

	if (dd->done) {
		complete(&dd->transfer_complete);
		dd->done = 0;
	}
	return ret;
}

static inline int msm_spi_request_irq(struct msm_spi *dd,
				      const char *name,
				      struct spi_master *master)
{
	return request_irq(dd->irq_in, msm_spi_qup_irq, IRQF_TRIGGER_HIGH,
			   name, dd);
}

static inline void msm_spi_free_irq(struct msm_spi *dd,
				    struct spi_master *master)
{
	free_irq(dd->irq_in, dd);
}

static inline void msm_spi_free_output_irq(struct msm_spi *dd) { }
static inline void msm_spi_free_error_irq(struct msm_spi *dd,
					  struct spi_master *master) { }

static inline void msm_spi_disable_irqs(struct msm_spi *dd)
{
	disable_irq(dd->irq_in);
}

static inline void msm_spi_enable_irqs(struct msm_spi *dd)
{
	enable_irq(dd->irq_in);
}

static inline void msm_spi_get_clk_err(struct msm_spi *dd, u32 *spi_err)
{
	*spi_err = readl_relaxed(dd->base + QUP_ERROR_FLAGS);
}

static inline void msm_spi_ack_clk_err(struct msm_spi *dd)
{
	writel_relaxed(QUP_ERR_MASK, dd->base + QUP_ERROR_FLAGS);
}

static inline void msm_spi_add_configs(struct msm_spi *dd, u32 *config, int n);

/* QUP has no_input, no_output, and N bits at QUP_CONFIG */
static inline void msm_spi_set_qup_config(struct msm_spi *dd, int bpw)
{
	u32 qup_config = readl_relaxed(dd->base + QUP_CONFIG);

	msm_spi_add_configs(dd, &qup_config, bpw-1);
	writel_relaxed(qup_config | QUP_CONFIG_SPI_MODE,
		       dd->base + QUP_CONFIG);
}

static inline int msm_spi_prepare_for_write(struct msm_spi *dd)
{
	if (msm_spi_set_state(dd, SPI_OP_STATE_RUN))
		return -1;
	if (msm_spi_set_state(dd, SPI_OP_STATE_PAUSE))
		return -1;
	return 0;
}

static inline void msm_spi_start_write(struct msm_spi *dd, u32 read_count)
{
	if (read_count <= dd->input_fifo_size)
		msm_spi_write_rmn_to_fifo(dd);
	else
		msm_spi_write_word_to_fifo(dd);
}

static inline void msm_spi_set_write_count(struct msm_spi *dd, int val)
{
	writel_relaxed(val, dd->base + QUP_MX_WRITE_COUNT);
}

static inline void msm_spi_complete(struct msm_spi *dd)
{
	dd->done = 1;
}

#endif

static void msm_spi_clock_set(struct msm_spi *dd, int speed)
{
	int rc;

	rc = clk_set_rate(dd->clk, speed);
	if (!rc)
		dd->clock_speed = speed;
}

static int msm_spi_calculate_size(int *fifo_size,
				  int *block_size,
				  int block,
				  int mult)
{
	int words;

	switch (block) {
	case 0:
		words = 1; /* 4 bytes */
		break;
	case 1:
		words = 4; /* 16 bytes */
		break;
	case 2:
		words = 8; /* 32 bytes */
		break;
	default:
		return -1;
	}
	switch (mult) {
	case 0:
		*fifo_size = words * 2;
		break;
	case 1:
		*fifo_size = words * 4;
		break;
	case 2:
		*fifo_size = words * 8;
		break;
	case 3:
		*fifo_size = words * 16;
		break;
	default:
		return -1;
	}
	*block_size = words * sizeof(u32); /* in bytes */
	return 0;
}

static void get_next_transfer(struct msm_spi *dd)
{
	struct spi_transfer *t = dd->cur_transfer;

	if (t->transfer_list.next != &dd->cur_msg->transfers) {
		dd->cur_transfer = list_entry(t->transfer_list.next,
					      struct spi_transfer,
					      transfer_list);
		dd->write_buf          = dd->cur_transfer->tx_buf;
		dd->read_buf           = dd->cur_transfer->rx_buf;
	}
}

static void __init msm_spi_calculate_fifo_size(struct msm_spi *dd)
{
	u32 spi_iom;
	int block;
	int mult;

	spi_iom = readl_relaxed(dd->base + SPI_IO_MODES);

	block = (spi_iom & SPI_IO_M_INPUT_BLOCK_SIZE) >> INPUT_BLOCK_SZ_SHIFT;
	mult = (spi_iom & SPI_IO_M_INPUT_FIFO_SIZE) >> INPUT_FIFO_SZ_SHIFT;
	if (msm_spi_calculate_size(&dd->input_fifo_size, &dd->input_block_size,
				   block, mult)) {
		goto fifo_size_err;
	}

	block = (spi_iom & SPI_IO_M_OUTPUT_BLOCK_SIZE) >> OUTPUT_BLOCK_SZ_SHIFT;
	mult = (spi_iom & SPI_IO_M_OUTPUT_FIFO_SIZE) >> OUTPUT_FIFO_SZ_SHIFT;
	if (msm_spi_calculate_size(&dd->output_fifo_size,
				   &dd->output_block_size, block, mult)) {
		goto fifo_size_err;
	}
	/* DM mode is not available for this block size */
	if (dd->input_block_size == 4 || dd->output_block_size == 4)
		dd->use_dma = 0;

	/* DM mode is currently unsupported for different block sizes */
	if (dd->input_block_size != dd->output_block_size)
		dd->use_dma = 0;

	if (dd->use_dma)
		dd->burst_size = max(dd->input_block_size, DM_BURST_SIZE);

	return;

fifo_size_err:
	dd->use_dma = 0;
	printk(KERN_WARNING "%s: invalid FIFO size, SPI_IO_MODES=0x%x\n",
	       __func__, spi_iom);
	return;
}

static void msm_spi_read_word_from_fifo(struct msm_spi *dd)
{
	u32   data_in;
	int   i;
	int   shift;

	data_in = readl_relaxed(dd->base + SPI_INPUT_FIFO);
	if (dd->read_buf) {
		for (i = 0; (i < dd->bytes_per_word) &&
			     dd->rx_bytes_remaining; i++) {
			/* The data format depends on bytes_per_word:
			   4 bytes: 0x12345678
			   3 bytes: 0x00123456
			   2 bytes: 0x00001234
			   1 byte : 0x00000012
			*/
			shift = 8 * (dd->bytes_per_word - i - 1);
			*dd->read_buf++ = (data_in & (0xFF << shift)) >> shift;
			dd->rx_bytes_remaining--;
		}
	} else {
		if (dd->rx_bytes_remaining >= dd->bytes_per_word)
			dd->rx_bytes_remaining -= dd->bytes_per_word;
		else
			dd->rx_bytes_remaining = 0;
	}
	dd->read_xfr_cnt++;
	if (dd->multi_xfr) {
		if (!dd->rx_bytes_remaining)
			dd->read_xfr_cnt = 0;
		else if ((dd->read_xfr_cnt * dd->bytes_per_word) ==
						dd->read_len) {
			struct spi_transfer *t = dd->cur_transfer;
			t = list_entry(t->transfer_list.next,
					struct spi_transfer,
					transfer_list);
			dd->read_buf = t->rx_buf;
			dd->read_len = t->len;
			dd->read_xfr_cnt = 0;
		}
	}
}

static inline bool msm_spi_is_valid_state(struct msm_spi *dd)
{
	u32 spi_op = readl_relaxed(dd->base + SPI_STATE);

	return spi_op & SPI_OP_STATE_VALID;
}

static inline int msm_spi_wait_valid(struct msm_spi *dd)
{
	unsigned long delay = 0;
	unsigned long timeout = 0;

	if (dd->clock_speed == 0)
		return -EINVAL;
	/*
	 * Based on the SPI clock speed, sufficient time
	 * should be given for the SPI state transition
	 * to occur
	 */
	delay = (10 * USEC_PER_SEC) / dd->clock_speed;
	/*
	 * For small delay values, the default timeout would
	 * be one jiffy
	 */
	if (delay < SPI_DELAY_THRESHOLD)
		delay = SPI_DELAY_THRESHOLD;
	timeout = jiffies + msecs_to_jiffies(delay * SPI_DEFAULT_TIMEOUT);
	while (!msm_spi_is_valid_state(dd)) {
		if (time_after(jiffies, timeout)) {
			dd->cur_msg->status = -EIO;
			dev_err(dd->dev, "%s: SPI operational state not valid"
				"\n", __func__);
			return -1;
		}
		/*
		 * For smaller values of delay, context switch time
		 * would negate the usage of usleep
		 */
		if (delay > 20)
			usleep(delay);
		else if (delay)
			udelay(delay);
	}
	return 0;
}

static inline int msm_spi_set_state(struct msm_spi *dd,
				    enum msm_spi_state state)
{
	enum msm_spi_state cur_state;
	if (msm_spi_wait_valid(dd))
		return -1;
	cur_state = readl_relaxed(dd->base + SPI_STATE);
	/* Per spec:
	   For PAUSE_STATE to RESET_STATE, two writes of (10) are required */
	if (((cur_state & SPI_OP_STATE) == SPI_OP_STATE_PAUSE) &&
			(state == SPI_OP_STATE_RESET)) {
		writel_relaxed(SPI_OP_STATE_CLEAR_BITS, dd->base + SPI_STATE);
		writel_relaxed(SPI_OP_STATE_CLEAR_BITS, dd->base + SPI_STATE);
	} else {
		writel_relaxed((cur_state & ~SPI_OP_STATE) | state,
		       dd->base + SPI_STATE);
	}
	if (msm_spi_wait_valid(dd))
		return -1;

	return 0;
}

static inline void msm_spi_add_configs(struct msm_spi *dd, u32 *config, int n)
{
	*config &= ~(SPI_NO_INPUT|SPI_NO_OUTPUT);

	if (n != (*config & SPI_CFG_N))
		*config = (*config & ~SPI_CFG_N) | n;

	if ((dd->mode == SPI_DMOV_MODE) && (!dd->multi_xfr)) {
		if (dd->read_buf == NULL)
			*config |= SPI_NO_INPUT;
		if (dd->write_buf == NULL)
			*config |= SPI_NO_OUTPUT;
	}
}

static void msm_spi_set_config(struct msm_spi *dd, int bpw)
{
	u32 spi_config;

	spi_config = readl_relaxed(dd->base + SPI_CONFIG);

	if (dd->cur_msg->spi->mode & SPI_CPHA)
		spi_config &= ~SPI_CFG_INPUT_FIRST;
	else
		spi_config |= SPI_CFG_INPUT_FIRST;
	if (dd->cur_msg->spi->mode & SPI_LOOP)
		spi_config |= SPI_CFG_LOOPBACK;
	else
		spi_config &= ~SPI_CFG_LOOPBACK;
	msm_spi_add_configs(dd, &spi_config, bpw-1);
	writel_relaxed(spi_config, dd->base + SPI_CONFIG);
	msm_spi_set_qup_config(dd, bpw);
}

static void msm_spi_setup_dm_transfer(struct msm_spi *dd)
{
	dmov_box *box;
	int bytes_to_send, num_rows, bytes_sent;
	u32 num_transfers;
	u32 write_transfers;
	u32 read_transfers;

	atomic_set(&dd->rx_irq_called, 0);
	bytes_sent = dd->cur_transfer->len - dd->tx_bytes_remaining;
	/* We'll send in chunks of SPI_MAX_LEN if larger */
	bytes_to_send = dd->tx_bytes_remaining / SPI_MAX_LEN ?
			  SPI_MAX_LEN : dd->tx_bytes_remaining;
	num_transfers = DIV_ROUND_UP(bytes_to_send, dd->bytes_per_word);
	dd->unaligned_len = bytes_to_send % dd->burst_size;
	num_rows = bytes_to_send / dd->burst_size;

	dd->mode = SPI_DMOV_MODE;

	if (num_rows) {
		/* src in 16 MSB, dst in 16 LSB */
		box = &dd->tx_dmov_cmd->box;
		box->src_row_addr = dd->cur_transfer->tx_dma + bytes_sent;
		box->src_dst_len = (dd->burst_size << 16) | dd->burst_size;
		box->num_rows = (num_rows << 16) | num_rows;
		box->row_offset = (dd->burst_size << 16) | 0;

		box = &dd->rx_dmov_cmd->box;
		box->dst_row_addr = dd->cur_transfer->rx_dma + bytes_sent;
		box->src_dst_len = (dd->burst_size << 16) | dd->burst_size;
		box->num_rows = (num_rows << 16) | num_rows;
		box->row_offset = (0 << 16) | dd->burst_size;

		dd->tx_dmov_cmd->cmd_ptr = CMD_PTR_LP |
				   DMOV_CMD_ADDR(dd->tx_dmov_cmd_dma +
				   offsetof(struct spi_dmov_cmd, box));
		dd->rx_dmov_cmd->cmd_ptr = CMD_PTR_LP |
				   DMOV_CMD_ADDR(dd->rx_dmov_cmd_dma +
				   offsetof(struct spi_dmov_cmd, box));
	} else {
		dd->tx_dmov_cmd->cmd_ptr = CMD_PTR_LP |
				   DMOV_CMD_ADDR(dd->tx_dmov_cmd_dma +
				   offsetof(struct spi_dmov_cmd, single_pad));
		dd->rx_dmov_cmd->cmd_ptr = CMD_PTR_LP |
				   DMOV_CMD_ADDR(dd->rx_dmov_cmd_dma +
				   offsetof(struct spi_dmov_cmd, single_pad));
	}

	if (!dd->unaligned_len) {
		dd->tx_dmov_cmd->box.cmd |= CMD_LC;
		dd->rx_dmov_cmd->box.cmd |= CMD_LC;
	} else {
		dmov_s *tx_cmd = &(dd->tx_dmov_cmd->single_pad);
		dmov_s *rx_cmd = &(dd->rx_dmov_cmd->single_pad);
		u32 offset = dd->cur_transfer->len - dd->unaligned_len;

		dd->tx_dmov_cmd->box.cmd &= ~CMD_LC;
		dd->rx_dmov_cmd->box.cmd &= ~CMD_LC;

		memset(dd->tx_padding, 0, dd->burst_size);
		memset(dd->rx_padding, 0, dd->burst_size);
		if (dd->write_buf)
			memcpy(dd->tx_padding, dd->write_buf + offset,
			       dd->unaligned_len);

		tx_cmd->src = dd->tx_padding_dma;
		rx_cmd->dst = dd->rx_padding_dma;
		tx_cmd->len = rx_cmd->len = dd->burst_size;
	}
	/* This also takes care of the padding dummy buf
	   Since this is set to the correct length, the
	   dummy bytes won't be actually sent */
	if (dd->read_len && dd->write_len) {
		if (dd->read_len + dd->write_len > 0) {
			/*
			 *  The read following a write transfer must take
			 *  into account, that the bytes pertaining to
			 *  the write transfer needs to be discarded,
			 *  before the actual read begins.
			 */
			write_transfers = DIV_ROUND_UP(dd->write_len,
						       dd->bytes_per_word);
			read_transfers = DIV_ROUND_UP(
						dd->read_len + dd->write_len,
						dd->bytes_per_word);
			writel_relaxed(write_transfers,
				       dd->base + SPI_MX_OUTPUT_COUNT);
			writel_relaxed(read_transfers,
				       dd->base + SPI_MX_INPUT_COUNT);
		}
	} else {
		if (dd->write_buf)
			writel_relaxed(num_transfers,
				       dd->base + SPI_MX_OUTPUT_COUNT);
		if (dd->read_buf)
			writel_relaxed(num_transfers,
				       dd->base + SPI_MX_INPUT_COUNT);
	}
}

static void msm_spi_enqueue_dm_commands(struct msm_spi *dd)
{
	dma_coherent_pre_ops();
	if (dd->write_buf)
		msm_dmov_enqueue_cmd(dd->tx_dma_chan, &dd->tx_hdr);
	if (dd->read_buf)
		msm_dmov_enqueue_cmd(dd->rx_dma_chan, &dd->rx_hdr);
}

/* SPI core can send maximum of 4K transfers, because there is HW problem
   with infinite mode.
   Therefore, we are sending several chunks of 3K or less (depending on how
   much is left).
   Upon completion we send the next chunk, or complete the transfer if
   everything is finished.
*/
static int msm_spi_dm_send_next(struct msm_spi *dd)
{
	/* By now we should have sent all the bytes in FIFO mode,
	 * However to make things right, we'll check anyway.
	 */
	if (dd->mode != SPI_DMOV_MODE)
		return 0;

	/* We need to send more chunks, if we sent max last time */
	if (dd->tx_bytes_remaining > SPI_MAX_LEN) {
		dd->tx_bytes_remaining -= SPI_MAX_LEN;
		if (msm_spi_set_state(dd, SPI_OP_STATE_RESET))
			return 0;
		dd->read_len = dd->write_len = 0;
		msm_spi_setup_dm_transfer(dd);
		msm_spi_enqueue_dm_commands(dd);
		if (msm_spi_set_state(dd, SPI_OP_STATE_RUN))
			return 0;
		return 1;
	} else if (dd->multi_xfr) {
		u32 prev_xfr_len = dd->cur_transfer->len;
		dd->tx_bytes_remaining -= dd->cur_transfer->len;
		if (list_is_last(&dd->cur_transfer->transfer_list,
					    &dd->cur_msg->transfers))
			return 0;
		get_next_transfer(dd);
		if (msm_spi_set_state(dd, SPI_OP_STATE_PAUSE))
			return 0;
		dd->tx_bytes_remaining = dd->rx_bytes_remaining =
					 dd->cur_transfer->len + prev_xfr_len;
		dd->read_len = dd->write_len = -1;
		msm_spi_setup_dm_transfer(dd);
		msm_spi_enqueue_dm_commands(dd);
		if (msm_spi_set_state(dd, SPI_OP_STATE_RUN))
			return 0;
		return 1;
	}
	return 0;
}

static inline void msm_spi_ack_transfer(struct msm_spi *dd)
{
	writel_relaxed(SPI_OP_MAX_INPUT_DONE_FLAG |
		       SPI_OP_MAX_OUTPUT_DONE_FLAG,
		       dd->base + SPI_OPERATIONAL);
}

static irqreturn_t msm_spi_input_irq(int irq, void *dev_id)
{
	struct msm_spi	       *dd = dev_id;

	dd->stat_rx++;

	if (dd->mode == SPI_MODE_NONE)
		return IRQ_HANDLED;

	if (dd->mode == SPI_DMOV_MODE) {
		u32 op = readl_relaxed(dd->base + SPI_OPERATIONAL);
		if ((!dd->read_buf || op & SPI_OP_MAX_INPUT_DONE_FLAG) &&
		    (!dd->write_buf || op & SPI_OP_MAX_OUTPUT_DONE_FLAG)) {
			msm_spi_ack_transfer(dd);
			if (dd->unaligned_len == 0) {
				if (atomic_inc_return(&dd->rx_irq_called) == 1)
					return IRQ_HANDLED;
			}
			msm_spi_complete(dd);
			return IRQ_HANDLED;
		}
		return IRQ_NONE;
	}

	if (dd->mode == SPI_FIFO_MODE) {
		while ((readl_relaxed(dd->base + SPI_OPERATIONAL) &
			SPI_OP_IP_FIFO_NOT_EMPTY) &&
			(dd->rx_bytes_remaining > 0)) {
			msm_spi_read_word_from_fifo(dd);
		}
		if (dd->rx_bytes_remaining == 0)
			msm_spi_complete(dd);
	}

	return IRQ_HANDLED;
}

static void msm_spi_write_word_to_fifo(struct msm_spi *dd)
{
	u32    word;
	u8     byte;
	int    i;

	word = 0;
	if (dd->write_buf) {
		for (i = 0; (i < dd->bytes_per_word) &&
			     dd->tx_bytes_remaining; i++) {
			dd->tx_bytes_remaining--;
			byte = *dd->write_buf++;
			word |= (byte << (BITS_PER_BYTE * (3 - i)));
		}
	} else
		if (dd->tx_bytes_remaining > dd->bytes_per_word)
			dd->tx_bytes_remaining -= dd->bytes_per_word;
		else
			dd->tx_bytes_remaining = 0;
	dd->write_xfr_cnt++;
	if (dd->multi_xfr) {
		if (!dd->tx_bytes_remaining)
			dd->write_xfr_cnt = 0;
		else if ((dd->write_xfr_cnt * dd->bytes_per_word) ==
						dd->write_len) {
			struct spi_transfer *t = dd->cur_transfer;
			t = list_entry(t->transfer_list.next,
					struct spi_transfer,
					transfer_list);
			dd->write_buf = t->tx_buf;
			dd->write_len = t->len;
			dd->write_xfr_cnt = 0;
		}
	}
	writel_relaxed(word, dd->base + SPI_OUTPUT_FIFO);
}

static inline void msm_spi_write_rmn_to_fifo(struct msm_spi *dd)
{
	int count = 0;

	while ((dd->tx_bytes_remaining > 0) && (count < dd->input_fifo_size) &&
	       !(readl_relaxed(dd->base + SPI_OPERATIONAL) &
		SPI_OP_OUTPUT_FIFO_FULL)) {
		msm_spi_write_word_to_fifo(dd);
		count++;
	}
}

static irqreturn_t msm_spi_output_irq(int irq, void *dev_id)
{
	struct msm_spi	       *dd = dev_id;

	dd->stat_tx++;

	if (dd->mode == SPI_MODE_NONE)
		return IRQ_HANDLED;

	if (dd->mode == SPI_DMOV_MODE) {
		/* TX_ONLY transaction is handled here
		   This is the only place we send complete at tx and not rx */
		if (dd->read_buf == NULL &&
		    readl_relaxed(dd->base + SPI_OPERATIONAL) &
		    SPI_OP_MAX_OUTPUT_DONE_FLAG) {
			msm_spi_ack_transfer(dd);
			msm_spi_complete(dd);
			return IRQ_HANDLED;
		}
		return IRQ_NONE;
	}

	/* Output FIFO is empty. Transmit any outstanding write data. */
	if (dd->mode == SPI_FIFO_MODE)
		msm_spi_write_rmn_to_fifo(dd);

	return IRQ_HANDLED;
}

static irqreturn_t msm_spi_error_irq(int irq, void *dev_id)
{
	struct spi_master	*master = dev_id;
	struct msm_spi          *dd = spi_master_get_devdata(master);
	u32                      spi_err;

	spi_err = readl_relaxed(dd->base + SPI_ERROR_FLAGS);
	if (spi_err & SPI_ERR_OUTPUT_OVER_RUN_ERR)
		dev_warn(master->dev.parent, "SPI output overrun error\n");
	if (spi_err & SPI_ERR_INPUT_UNDER_RUN_ERR)
		dev_warn(master->dev.parent, "SPI input underrun error\n");
	if (spi_err & SPI_ERR_OUTPUT_UNDER_RUN_ERR)
		dev_warn(master->dev.parent, "SPI output underrun error\n");
	if (spi_err & SPI_ERR_INPUT_OVER_RUN_ERR) {
		/*
		 * When switching from Run to Reset state, the core generates
		 * a bogus input overrun error on 8660. We ignore such errors
		 * when the transfer has completed successfully.
		 */
		if (dd->mode != SPI_MODE_NONE)
			dev_warn(master->dev.parent,
				 "SPI input overrun error\n");
	}
	msm_spi_get_clk_err(dd, &spi_err);
	if (spi_err & SPI_ERR_CLK_OVER_RUN_ERR)
		dev_warn(master->dev.parent, "SPI clock overrun error\n");
	if (spi_err & SPI_ERR_CLK_UNDER_RUN_ERR)
		dev_warn(master->dev.parent, "SPI clock underrun error\n");
	writel_relaxed(SPI_ERR_MASK, dd->base + SPI_ERROR_FLAGS);
	msm_spi_ack_clk_err(dd);
	return IRQ_HANDLED;
}

static void msm_spi_unmap_dma_buffers(struct msm_spi *dd)
{
	struct device *dev;

	dev = &dd->cur_msg->spi->dev;
	if (!dd->cur_msg->is_dma_mapped) {
		if (dd->cur_transfer->rx_buf)
			dma_unmap_single(dev, dd->cur_transfer->rx_dma,
					 dd->cur_transfer->len,
					 DMA_FROM_DEVICE);
		if (dd->cur_transfer->tx_buf)
			dma_unmap_single(dev, dd->cur_transfer->tx_dma,
					 dd->cur_transfer->len,
					 DMA_TO_DEVICE);
	}
	/* If we padded the transfer, we copy it from the padding buf */
	if (dd->unaligned_len && dd->read_buf) {
		u32 offset = dd->cur_transfer->len - dd->unaligned_len;
		dma_coherent_post_ops();
		memcpy(dd->read_buf + offset, dd->rx_padding,
		       dd->unaligned_len);
	}
}

/**
 * msm_use_dm - decides whether to use data mover for this
 * 		transfer
 * @dd:       device
 * @tr:       transfer
 *
 * Start using DM if:
 * 1. Transfer is longer than 3*block size.
 * 2. Buffers should be aligned to cache line.
  */
static inline int msm_use_dm(struct msm_spi *dd, struct spi_transfer *tr,
				u32 xfr_len)
{
	u32 cache_line = dma_get_cache_alignment();

	if (!dd->use_dma)
		return 0;

	if (xfr_len < 3*dd->input_block_size)
		return 0;

	if (tr->tx_buf) {
		if (!IS_ALIGNED((size_t)tr->tx_buf, cache_line))
			return 0;
	}
	if (tr->rx_buf) {
		if (!IS_ALIGNED((size_t)tr->rx_buf, cache_line))
			return 0;
	}
	return 1;
}

static void msm_spi_process_transfer(struct msm_spi *dd)
{
	u8  bpw;
	u32 spi_ioc;
	u32 spi_iom;
	u32 spi_ioc_orig;
	u32 max_speed;
	u32 chip_select;
	u32 read_count;
	u32 timeout;
	u32 transfer_len = 0;
	u32 int_loopback = 0;

	if (dd->read_len && dd->write_len) {
		dd->tx_bytes_remaining = dd->write_len;
		dd->rx_bytes_remaining = dd->read_len;
		dd->multi_xfr = 1;
		transfer_len = dd->write_len + dd->read_len;
	} else {
		dd->tx_bytes_remaining = dd->cur_transfer->len;
		dd->rx_bytes_remaining = dd->cur_transfer->len;
		dd->multi_xfr = 0;
		transfer_len = dd->cur_transfer->len;
	}
	dd->read_buf           = dd->cur_transfer->rx_buf;
	dd->write_buf          = dd->cur_transfer->tx_buf;
	init_completion(&dd->transfer_complete);
	if (dd->cur_transfer->bits_per_word)
		bpw = dd->cur_transfer->bits_per_word;
	else
		if (dd->cur_msg->spi->bits_per_word)
			bpw = dd->cur_msg->spi->bits_per_word;
		else
			bpw = 8;
	dd->bytes_per_word = (bpw + 7) / 8;

	if (dd->cur_transfer->speed_hz)
		max_speed = dd->cur_transfer->speed_hz;
	else
		max_speed = dd->cur_msg->spi->max_speed_hz;
	if (!dd->clock_speed || max_speed < dd->clock_speed)
		msm_spi_clock_set(dd, max_speed);

	read_count = DIV_ROUND_UP(transfer_len, dd->bytes_per_word);
	if (dd->cur_msg->spi->mode & SPI_LOOP)
		int_loopback = 1;
	if (int_loopback && dd->read_len && dd->write_len &&
			(read_count > dd->input_fifo_size)) {
		printk(KERN_WARNING
		"%s:Internal Loopback does not support > fifo size\
		for write-then-read transactions\n",
		__func__);
		return;
	}
	if ((!msm_use_dm(dd, dd->cur_transfer, transfer_len))) {
		dd->mode = SPI_FIFO_MODE;
		dd->tx_bytes_remaining = transfer_len;
		dd->rx_bytes_remaining = transfer_len;
		if (dd->multi_xfr) {
			dd->read_len = dd->cur_transfer->len;
			dd->write_len = dd->cur_transfer->len;
		}
		/* read_count cannot exceed fifo_size, and only one READ COUNT
		   interrupt is generated per transaction, so for transactions
		   larger than fifo size READ COUNT must be disabled.
		   For those transactions we usually move to Data Mover mode.
		*/
		if (read_count <= dd->input_fifo_size) {
			writel_relaxed(read_count,
				       dd->base + SPI_MX_READ_COUNT);
			msm_spi_set_write_count(dd, read_count);
		} else {
			writel_relaxed(0, dd->base + SPI_MX_READ_COUNT);
			msm_spi_set_write_count(dd, 0);
		}
	} else
		dd->mode = SPI_DMOV_MODE;

	/* Write mode - fifo or data mover*/
	spi_iom = readl_relaxed(dd->base + SPI_IO_MODES);
	spi_iom &= ~(SPI_IO_M_INPUT_MODE | SPI_IO_M_OUTPUT_MODE);
	spi_iom = (spi_iom | (dd->mode << OUTPUT_MODE_SHIFT));
	spi_iom = (spi_iom | (dd->mode << INPUT_MODE_SHIFT));
	/* Turn on packing for data mover */
	if (dd->mode == SPI_DMOV_MODE)
		spi_iom |= SPI_IO_M_PACK_EN | SPI_IO_M_UNPACK_EN;
	else
		spi_iom &= ~(SPI_IO_M_PACK_EN | SPI_IO_M_UNPACK_EN);
	writel_relaxed(spi_iom, dd->base + SPI_IO_MODES);

	msm_spi_set_config(dd, bpw);

	spi_ioc = readl_relaxed(dd->base + SPI_IO_CONTROL);
	spi_ioc_orig = spi_ioc;
	if (dd->cur_msg->spi->mode & SPI_CPOL)
		spi_ioc |= SPI_IO_C_CLK_IDLE_HIGH;
	else
		spi_ioc &= ~SPI_IO_C_CLK_IDLE_HIGH;
	chip_select = dd->cur_msg->spi->chip_select << 2;
	if ((spi_ioc & SPI_IO_C_CS_SELECT) != chip_select)
		spi_ioc = (spi_ioc & ~SPI_IO_C_CS_SELECT) | chip_select;
	if (!dd->cur_transfer->cs_change)
		spi_ioc |= SPI_IO_C_MX_CS_MODE;
	if (spi_ioc != spi_ioc_orig)
		writel_relaxed(spi_ioc, dd->base + SPI_IO_CONTROL);

	if (dd->mode == SPI_DMOV_MODE) {
		msm_spi_setup_dm_transfer(dd);
		msm_spi_enqueue_dm_commands(dd);
	}
	/* The output fifo interrupt handler will handle all writes after
	   the first. Restricting this to one write avoids contention
	   issues and race conditions between this thread and the int handler
	*/
	else if (dd->mode == SPI_FIFO_MODE) {
		if (msm_spi_prepare_for_write(dd))
			goto transfer_end;
		msm_spi_start_write(dd, read_count);
	}

	/* Only enter the RUN state after the first word is written into
	   the output FIFO. Otherwise, the output FIFO EMPTY interrupt
	   might fire before the first word is written resulting in a
	   possible race condition.
	 */
	if (msm_spi_set_state(dd, SPI_OP_STATE_RUN))
		goto transfer_end;

	timeout = 100 * msecs_to_jiffies(
	      DIV_ROUND_UP(dd->cur_transfer->len * 8,
		 DIV_ROUND_UP(max_speed, MSEC_PER_SEC)));

	/* Assume success, this might change later upon transaction result */
	dd->cur_msg->status = 0;
	do {
		if (!wait_for_completion_timeout(&dd->transfer_complete,
						 timeout)) {
				dev_err(dd->dev, "%s: SPI transaction "
						 "timeout\n", __func__);
				dd->cur_msg->status = -EIO;
				if (dd->mode == SPI_DMOV_MODE) {
					msm_dmov_flush(dd->tx_dma_chan);
					msm_dmov_flush(dd->rx_dma_chan);
				}
				break;
		}
	} while (msm_spi_dm_send_next(dd));

transfer_end:
	if (dd->mode == SPI_DMOV_MODE)
		msm_spi_unmap_dma_buffers(dd);
	dd->mode = SPI_MODE_NONE;

	msm_spi_set_state(dd, SPI_OP_STATE_RESET);
	writel_relaxed(spi_ioc & ~SPI_IO_C_MX_CS_MODE,
		       dd->base + SPI_IO_CONTROL);
}

static int get_transfer_length(struct spi_transfer *tr, struct spi_message *msg)
{
	struct spi_transfer *next_transfer = NULL;
	u32 write_len;
	u32 read_len;

	write_len = read_len = 0;

	if (tr->tx_buf)
		write_len = tr->len;

	if (tr->rx_buf)
		read_len = tr->len;

	if (write_len && read_len)
		return tr->len;
	else if ((tr->transfer_list.next != &msg->transfers) && (!read_len)) {
		next_transfer = list_entry(tr->transfer_list.next,
					struct spi_transfer,
					transfer_list);
		if (next_transfer->tx_buf && next_transfer->rx_buf)
			return tr->len;

		if (next_transfer->tx_buf && !write_len)
			write_len = next_transfer->len;

		if (next_transfer->rx_buf && !read_len)
			read_len = next_transfer->len;

		if (write_len && read_len)
			return write_len + read_len;
	}
	return tr->len;
}

static void msm_spi_process_message(struct msm_spi *dd)
{
	u32 xfr_len = 0;
	int next_xfr_processed = 0;

	list_for_each_entry(dd->cur_transfer,
			    &dd->cur_msg->transfers,
			    transfer_list) {
		if (!dd->cur_transfer->len)
			return;

		dd->write_xfr_cnt = dd->read_xfr_cnt = 0;
		dd->read_len = dd->write_len = 0;
		if (next_xfr_processed) {
			next_xfr_processed = 0;
			continue;
		}
		xfr_len = get_transfer_length(dd->cur_transfer, dd->cur_msg);
		if (xfr_len == dd->cur_transfer->len)
			msm_spi_process_transfer(dd);
		else {
			if (dd->cur_transfer->tx_buf) {
				dd->write_len =  dd->cur_transfer->len;
				dd->read_len = xfr_len - dd->write_len;
			} else {
				dd->read_len = dd->cur_transfer->len;
				dd->write_len = xfr_len - dd->read_len;
			}
			msm_spi_process_transfer(dd);
			next_xfr_processed = 1;
		}
	}
}

/* workqueue - pull messages from queue & process */
static void msm_spi_workq(struct work_struct *work)
{
	struct msm_spi      *dd =
		container_of(work, struct msm_spi, work_data);
	unsigned long        flags;
	u32                  status_error = 0;

	mutex_lock(&dd->core_lock);

	/* Don't allow power collapse until we release mutex */
	pm_qos_update_request(qos_req_list,
				  dd->pm_lat);
	if (dd->use_rlock)
		remote_mutex_lock(&dd->r_lock);

	clk_enable(dd->clk);
	if (dd->pclk)
		clk_enable(dd->pclk);
	msm_spi_enable_irqs(dd);

	if (!msm_spi_is_valid_state(dd)) {
		dev_err(dd->dev, "%s: SPI operational state not valid\n",
			__func__);
		status_error = 1;
	}

	spin_lock_irqsave(&dd->queue_lock, flags);
	while (!list_empty(&dd->queue)) {
		dd->cur_msg = list_entry(dd->queue.next,
					 struct spi_message, queue);
		list_del_init(&dd->cur_msg->queue);
		spin_unlock_irqrestore(&dd->queue_lock, flags);
		if (status_error)
			dd->cur_msg->status = -EIO;
		else
			msm_spi_process_message(dd);
		if (dd->cur_msg->complete)
			dd->cur_msg->complete(dd->cur_msg->context);
		spin_lock_irqsave(&dd->queue_lock, flags);
	}
	dd->transfer_pending = 0;
	spin_unlock_irqrestore(&dd->queue_lock, flags);

	msm_spi_disable_irqs(dd);
	clk_disable(dd->clk);
	if (dd->pclk)
		clk_disable(dd->pclk);

	if (dd->use_rlock)
		remote_mutex_unlock(&dd->r_lock);

	pm_qos_update_request(qos_req_list,
				  PM_QOS_DEFAULT_VALUE);

	mutex_unlock(&dd->core_lock);
	/* If needed, this can be done after the current message is complete,
	   and work can be continued upon resume. No motivation for now. */
	if (dd->suspended)
		wake_up_interruptible(&dd->continue_suspend);
}

static int msm_spi_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct msm_spi	*dd;
	unsigned long    flags;
	struct spi_transfer *tr;
	int ret = -EINVAL;
	u32 xfr_len = 0;
	struct spi_transfer *next_transfer;
	int next_xfr_processed = 0;

	dd = spi_master_get_devdata(spi->master);
	if (dd->suspended)
		return -EBUSY;

	if (list_empty(&msg->transfers) || !msg->complete)
		return -EINVAL;

	list_for_each_entry(tr, &msg->transfers, transfer_list) {
		void *tx_buf = (void *)tr->tx_buf;
		void       *rx_buf = tr->rx_buf;
		unsigned len = tr->len;
		next_transfer = NULL;

		/* Check message parameters */
		if (tr->speed_hz > dd->pdata->max_clock_speed ||
		    (tr->bits_per_word &&
		     (tr->bits_per_word < 4 || tr->bits_per_word > 32)) ||
		    (tx_buf == NULL && rx_buf == NULL)) {
			dev_err(&spi->dev, "Invalid transfer: %d Hz, %d bpw"
					   "tx=%p, rx=%p\n",
					    tr->speed_hz, tr->bits_per_word,
					    tx_buf, rx_buf);
			goto error;
		}

		if (next_xfr_processed) {
			next_xfr_processed = 0;
			continue;
		}

		xfr_len = get_transfer_length(tr, msg);
		if (!msm_use_dm(dd, tr, xfr_len) || msg->is_dma_mapped)
			continue;

		/* Do DMA mapping "early" for better error reporting */
		do {
			if (tx_buf != NULL) {
				tr->tx_dma = dma_map_single(&spi->dev, tx_buf,
							     len,
							     DMA_TO_DEVICE);
				if (dma_mapping_error(NULL, tr->tx_dma)) {
					dev_err(&spi->dev,
						"dma %cX %d bytes error\n",
						'T', len);
					goto error;
				}
			}

			if (rx_buf != NULL) {
				tr->rx_dma = dma_map_single(&spi->dev, rx_buf,
						len,
						DMA_FROM_DEVICE);
				if (dma_mapping_error(NULL, tr->rx_dma)) {
					dev_err(&spi->dev,
						"dma %cX %d bytes error\n",
						'R', len);
					if (tx_buf != NULL)
						dma_unmap_single(NULL,
								tr->tx_dma,
								len,
								DMA_TO_DEVICE);
					ret = -EINVAL;
					goto error;
				}
			}
			if (next_xfr_processed)
				break;
			if (tr->len != xfr_len) {
				next_transfer = list_entry(
						tr->transfer_list.next,
						struct spi_transfer,
						transfer_list);
				tr = next_transfer;
				tx_buf = (void *)tr->tx_buf;
				rx_buf = tr->rx_buf;
				len = tr->len;
				next_xfr_processed = 1;
			}
		} while (next_xfr_processed);
	}

	spin_lock_irqsave(&dd->queue_lock, flags);
	if (dd->suspended) {
		spin_unlock_irqrestore(&dd->queue_lock, flags);
		ret = -EBUSY;
		goto error;
	}
	dd->transfer_pending = 1;
	list_add_tail(&msg->queue, &dd->queue);
	spin_unlock_irqrestore(&dd->queue_lock, flags);
	queue_work(dd->workqueue, &dd->work_data);
	return 0;

error:
	list_for_each_entry_continue_reverse(tr, &msg->transfers, transfer_list)
	{
		if (msm_use_dm(dd, tr, xfr_len) && !msg->is_dma_mapped) {
			if (tr->rx_buf != NULL)
				dma_unmap_single(&spi->dev, tr->rx_dma, tr->len,
						 DMA_TO_DEVICE);
			if (tr->tx_buf != NULL)
				dma_unmap_single(&spi->dev, tr->tx_dma, tr->len,
						  DMA_FROM_DEVICE);
		}
	}
	return ret;
}

static int msm_spi_setup(struct spi_device *spi)
{
	struct msm_spi	*dd;
	int              rc = 0;
	u32              spi_ioc;
	u32              spi_config;
	u32              mask;

	if (spi->bits_per_word < 4 || spi->bits_per_word > 32) {
		dev_err(&spi->dev, "%s: invalid bits_per_word %d\n",
			__func__, spi->bits_per_word);
		rc = -EINVAL;
	}
	if (spi->chip_select > SPI_NUM_CHIPSELECTS-1) {
		dev_err(&spi->dev, "%s, chip select %d exceeds max value %d\n",
			__func__, spi->chip_select, SPI_NUM_CHIPSELECTS - 1);
		rc = -EINVAL;
	}

	if (rc)
		goto err_setup_exit;

	dd = spi_master_get_devdata(spi->master);

	mutex_lock(&dd->core_lock);
	if (dd->suspended) {
		mutex_unlock(&dd->core_lock);
		return -EBUSY;
	}

	if (dd->use_rlock)
		remote_mutex_lock(&dd->r_lock);

	clk_enable(dd->clk);
	if (dd->pclk)
		clk_enable(dd->pclk);

	spi_ioc = readl_relaxed(dd->base + SPI_IO_CONTROL);
	mask = SPI_IO_C_CS_N_POLARITY_0 << spi->chip_select;
	if (spi->mode & SPI_CS_HIGH)
		spi_ioc |= mask;
	else
		spi_ioc &= ~mask;
	if (spi->mode & SPI_CPOL)
		spi_ioc |= SPI_IO_C_CLK_IDLE_HIGH;
	else
		spi_ioc &= ~SPI_IO_C_CLK_IDLE_HIGH;

	writel_relaxed(spi_ioc, dd->base + SPI_IO_CONTROL);

	spi_config = readl_relaxed(dd->base + SPI_CONFIG);
	if (spi->mode & SPI_LOOP)
		spi_config |= SPI_CFG_LOOPBACK;
	else
		spi_config &= ~SPI_CFG_LOOPBACK;
	if (spi->mode & SPI_CPHA)
		spi_config &= ~SPI_CFG_INPUT_FIRST;
	else
		spi_config |= SPI_CFG_INPUT_FIRST;
	writel_relaxed(spi_config, dd->base + SPI_CONFIG);

	clk_disable(dd->clk);
	if (dd->pclk)
		clk_disable(dd->pclk);

	if (dd->use_rlock)
		remote_mutex_unlock(&dd->r_lock);
	mutex_unlock(&dd->core_lock);

err_setup_exit:
	return rc;
}

#ifdef CONFIG_DEBUG_FS
static int debugfs_iomem_x32_set(void *data, u64 val)
{
	writel_relaxed(val, data);
	wmb();
	return 0;
}

static int debugfs_iomem_x32_get(void *data, u64 *val)
{
	*val = readl_relaxed(data);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fops_iomem_x32, debugfs_iomem_x32_get,
			debugfs_iomem_x32_set, "0x%08llx\n");

static void spi_debugfs_init(struct msm_spi *dd)
{
	dd->dent_spi = debugfs_create_dir(dev_name(dd->dev), NULL);
	if (dd->dent_spi) {
		int i;
		for (i = 0; i < ARRAY_SIZE(debugfs_spi_regs); i++) {
			dd->debugfs_spi_regs[i] =
			   debugfs_create_file(
			       debugfs_spi_regs[i].name,
			       debugfs_spi_regs[i].mode,
			       dd->dent_spi,
			       dd->base + debugfs_spi_regs[i].offset,
			       &fops_iomem_x32);
		}
	}
}

static void spi_debugfs_exit(struct msm_spi *dd)
{
	if (dd->dent_spi) {
		int i;
		debugfs_remove_recursive(dd->dent_spi);
		dd->dent_spi = NULL;
		for (i = 0; i < ARRAY_SIZE(debugfs_spi_regs); i++)
			dd->debugfs_spi_regs[i] = NULL;
	}
}
#else
static void spi_debugfs_init(struct msm_spi *dd) {}
static void spi_debugfs_exit(struct msm_spi *dd) {}
#endif

/* ===Device attributes begin=== */
static ssize_t show_stats(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct msm_spi *dd =  spi_master_get_devdata(master);

	return snprintf(buf, PAGE_SIZE,
			"Device       %s\n"
			"rx fifo_size = %d spi words\n"
			"tx fifo_size = %d spi words\n"
			"use_dma ?    %s\n"
			"rx block size = %d bytes\n"
			"tx block size = %d bytes\n"
			"burst size = %d bytes\n"
			"DMA configuration:\n"
			"tx_ch=%d, rx_ch=%d, tx_crci= %d, rx_crci=%d\n"
			"--statistics--\n"
			"Rx isrs  = %d\n"
			"Tx isrs  = %d\n"
			"DMA error  = %d\n"
			"--debug--\n"
			"NA yet\n",
			dev_name(dev),
			dd->input_fifo_size,
			dd->output_fifo_size,
			dd->use_dma ? "yes" : "no",
			dd->input_block_size,
			dd->output_block_size,
			dd->burst_size,
			dd->tx_dma_chan,
			dd->rx_dma_chan,
			dd->tx_dma_crci,
			dd->rx_dma_crci,
			dd->stat_rx + dd->stat_dmov_rx,
			dd->stat_tx + dd->stat_dmov_tx,
			dd->stat_dmov_tx_err + dd->stat_dmov_rx_err
			);
}

/* Reset statistics on write */
static ssize_t set_stats(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct msm_spi *dd = dev_get_drvdata(dev);
	dd->stat_rx = 0;
	dd->stat_tx = 0;
	dd->stat_dmov_rx = 0;
	dd->stat_dmov_tx = 0;
	dd->stat_dmov_rx_err = 0;
	dd->stat_dmov_tx_err = 0;
	return count;
}

static DEVICE_ATTR(stats, S_IRUGO | S_IWUSR, show_stats, set_stats);

static struct attribute *dev_attrs[] = {
	&dev_attr_stats.attr,
	NULL,
};

static struct attribute_group dev_attr_grp = {
	.attrs = dev_attrs,
};
/* ===Device attributes end=== */

/**
 * spi_dmov_tx_complete_func - DataMover tx completion callback
 *
 * Executed in IRQ context (Data Mover's IRQ) DataMover's
 * spinlock @msm_dmov_lock held.
 */
static void spi_dmov_tx_complete_func(struct msm_dmov_cmd *cmd,
				      unsigned int result,
				      struct msm_dmov_errdata *err)
{
	struct msm_spi *dd;

	if (!(result & DMOV_RSLT_VALID)) {
		pr_err("Invalid DMOV result: rc=0x%08x, cmd = %p", result, cmd);
		return;
	}
	/* restore original context */
	dd = container_of(cmd, struct msm_spi, tx_hdr);
	if (result & DMOV_RSLT_DONE)
		dd->stat_dmov_tx++;
	else {
		/* Error or flush */
		if (result & DMOV_RSLT_ERROR) {
			dev_err(dd->dev, "DMA error (0x%08x)\n", result);
			dd->stat_dmov_tx_err++;
		}
		if (result & DMOV_RSLT_FLUSH) {
			/*
			 * Flushing normally happens in process of
			 * removing, when we are waiting for outstanding
			 * DMA commands to be flushed.
			 */
			dev_info(dd->dev,
				 "DMA channel flushed (0x%08x)\n", result);
		}
		if (err)
			dev_err(dd->dev,
				"Flush data(%08x %08x %08x %08x %08x %08x)\n",
				err->flush[0], err->flush[1], err->flush[2],
				err->flush[3], err->flush[4], err->flush[5]);
		dd->cur_msg->status = -EIO;
		complete(&dd->transfer_complete);
	}
}

/**
 * spi_dmov_rx_complete_func - DataMover rx completion callback
 *
 * Executed in IRQ context (Data Mover's IRQ)
 * DataMover's spinlock @msm_dmov_lock held.
 */
static void spi_dmov_rx_complete_func(struct msm_dmov_cmd *cmd,
				      unsigned int result,
				      struct msm_dmov_errdata *err)
{
	struct msm_spi *dd;

	if (!(result & DMOV_RSLT_VALID)) {
		pr_err("Invalid DMOV result(rc = 0x%08x, cmd = %p)",
		       result, cmd);
		return;
	}
	/* restore original context */
	dd = container_of(cmd, struct msm_spi, rx_hdr);
	if (result & DMOV_RSLT_DONE) {
		dd->stat_dmov_rx++;
		if (atomic_inc_return(&dd->rx_irq_called) == 1)
			return;
		complete(&dd->transfer_complete);
	} else {
		/** Error or flush  */
		if (result & DMOV_RSLT_ERROR) {
			dev_err(dd->dev, "DMA error(0x%08x)\n", result);
			dd->stat_dmov_rx_err++;
		}
		if (result & DMOV_RSLT_FLUSH) {
			dev_info(dd->dev,
				"DMA channel flushed(0x%08x)\n", result);
		}
		if (err)
			dev_err(dd->dev,
				"Flush data(%08x %08x %08x %08x %08x %08x)\n",
				err->flush[0], err->flush[1], err->flush[2],
				err->flush[3], err->flush[4], err->flush[5]);
		dd->cur_msg->status = -EIO;
		complete(&dd->transfer_complete);
	}
}

static inline u32 get_chunk_size(struct msm_spi *dd)
{
	u32 cache_line = dma_get_cache_alignment();

	return (roundup(sizeof(struct spi_dmov_cmd), DM_BYTE_ALIGN) +
			  roundup(dd->burst_size, cache_line))*2;
}

static void msm_spi_teardown_dma(struct msm_spi *dd)
{
	int limit = 0;

	if (!dd->use_dma)
		return;

	while (dd->mode == SPI_DMOV_MODE && limit++ < 50) {
		msm_dmov_flush(dd->tx_dma_chan);
		msm_dmov_flush(dd->rx_dma_chan);
		msleep(10);
	}

	dma_free_coherent(NULL, get_chunk_size(dd), dd->tx_dmov_cmd,
			  dd->tx_dmov_cmd_dma);
	dd->tx_dmov_cmd = dd->rx_dmov_cmd = NULL;
	dd->tx_padding = dd->rx_padding = NULL;
}

static __init int msm_spi_init_dma(struct msm_spi *dd)
{
	dmov_box *box;
	u32 cache_line = dma_get_cache_alignment();

	/* Allocate all as one chunk, since all is smaller than page size */

	/* We send NULL device, since it requires coherent_dma_mask id
	   device definition, we're okay with using system pool */
	dd->tx_dmov_cmd = dma_alloc_coherent(NULL, get_chunk_size(dd),
					     &dd->tx_dmov_cmd_dma, GFP_KERNEL);
	if (dd->tx_dmov_cmd == NULL)
		return -ENOMEM;

	/* DMA addresses should be 64 bit aligned aligned */
	dd->rx_dmov_cmd = (struct spi_dmov_cmd *)
			  ALIGN((size_t)&dd->tx_dmov_cmd[1], DM_BYTE_ALIGN);
	dd->rx_dmov_cmd_dma = ALIGN(dd->tx_dmov_cmd_dma +
			      sizeof(struct spi_dmov_cmd), DM_BYTE_ALIGN);

	/* Buffers should be aligned to cache line */
	dd->tx_padding = (u8 *)ALIGN((size_t)&dd->rx_dmov_cmd[1], cache_line);
	dd->tx_padding_dma = ALIGN(dd->rx_dmov_cmd_dma +
			      sizeof(struct spi_dmov_cmd), cache_line);
	dd->rx_padding = (u8 *)ALIGN((size_t)(dd->tx_padding + dd->burst_size),
				     cache_line);
	dd->rx_padding_dma = ALIGN(dd->tx_padding_dma + dd->burst_size,
				      cache_line);

	/* Setup DM commands */
	box = &(dd->rx_dmov_cmd->box);
	box->cmd = CMD_MODE_BOX | CMD_SRC_CRCI(dd->rx_dma_crci);
	box->src_row_addr = (uint32_t)dd->mem_phys_addr + SPI_INPUT_FIFO;
	dd->rx_hdr.cmdptr = DMOV_CMD_PTR_LIST |
				   DMOV_CMD_ADDR(dd->rx_dmov_cmd_dma +
				   offsetof(struct spi_dmov_cmd, cmd_ptr));
	dd->rx_hdr.complete_func = spi_dmov_rx_complete_func;
	dd->rx_hdr.crci_mask = msm_dmov_build_crci_mask(1, dd->rx_dma_crci);

	box = &(dd->tx_dmov_cmd->box);
	box->cmd = CMD_MODE_BOX | CMD_DST_CRCI(dd->tx_dma_crci);
	box->dst_row_addr = (uint32_t)dd->mem_phys_addr + SPI_OUTPUT_FIFO;
	dd->tx_hdr.cmdptr = DMOV_CMD_PTR_LIST |
			    DMOV_CMD_ADDR(dd->tx_dmov_cmd_dma +
			    offsetof(struct spi_dmov_cmd, cmd_ptr));
	dd->tx_hdr.complete_func = spi_dmov_tx_complete_func;
	dd->tx_hdr.crci_mask = msm_dmov_build_crci_mask(1, dd->tx_dma_crci);

	dd->tx_dmov_cmd->single_pad.cmd = CMD_MODE_SINGLE | CMD_LC |
					  CMD_DST_CRCI(dd->tx_dma_crci);
	dd->tx_dmov_cmd->single_pad.dst = (uint32_t)dd->mem_phys_addr +
					   SPI_OUTPUT_FIFO;
	dd->rx_dmov_cmd->single_pad.cmd = CMD_MODE_SINGLE | CMD_LC |
					  CMD_SRC_CRCI(dd->rx_dma_crci);
	dd->rx_dmov_cmd->single_pad.src = (uint32_t)dd->mem_phys_addr +
					  SPI_INPUT_FIFO;

	/* Clear remaining activities on channel */
	msm_dmov_flush(dd->tx_dma_chan);
	msm_dmov_flush(dd->rx_dma_chan);

	return 0;
}

static int __init msm_spi_probe(struct platform_device *pdev)
{
	struct spi_master      *master;
	struct msm_spi	       *dd;
	struct resource	       *resource;
	int                     rc = -ENXIO;
	int                     locked = 0;
	int                     clk_enabled = 0;
	int                     pclk_enabled = 0;
	struct msm_spi_platform_data *pdata = pdev->dev.platform_data;

	master = spi_alloc_master(&pdev->dev, sizeof(struct msm_spi));
	if (!master) {
		rc = -ENOMEM;
		dev_err(&pdev->dev, "master allocation failed\n");
		goto err_probe_exit;
	}

	master->bus_num        = pdev->id;
	master->mode_bits      = SPI_SUPPORTED_MODES;
	master->num_chipselect = SPI_NUM_CHIPSELECTS;
	master->setup          = msm_spi_setup;
	master->transfer       = msm_spi_transfer;
	platform_set_drvdata(pdev, master);
	dd = spi_master_get_devdata(master);

	dd->pdata = pdata;
	rc = msm_spi_get_irq_data(dd, pdev);
	if (rc)
		goto err_probe_res;
	resource = platform_get_resource_byname(pdev,
						 IORESOURCE_MEM, "spi_base");
	if (!resource) {
		rc = -ENXIO;
		goto err_probe_res;
	}
	dd->mem_phys_addr = resource->start;
	dd->mem_size = resource_size(resource);

	rc = msm_spi_get_gsbi_resource(dd, pdev);
	if (rc)
		goto err_probe_res2;

	if (pdata) {
		if (pdata->dma_config) {
			rc = pdata->dma_config();
			if (rc) {
				dev_warn(&pdev->dev,
					"%s: DM mode not supported\n",
					__func__);
				dd->use_dma = 0;
				goto skip_dma_resources;
			}
		}
		resource = platform_get_resource_byname(pdev,
							IORESOURCE_DMA,
							"spidm_channels");
		if (resource) {
			dd->rx_dma_chan = resource->start;
			dd->tx_dma_chan = resource->end;

			resource = platform_get_resource_byname(pdev,
							IORESOURCE_DMA,
							"spidm_crci");
			if (!resource) {
				rc = -ENXIO;
				goto err_probe_res;
			}
			dd->rx_dma_crci = resource->start;
			dd->tx_dma_crci = resource->end;
			dd->use_dma = 1;
			master->dma_alignment =	dma_get_cache_alignment();
		}

skip_dma_resources:
		if (pdata->gpio_config) {
			rc = pdata->gpio_config();
			if (rc) {
				dev_err(&pdev->dev,
					"%s: error configuring GPIOs\n",
					__func__);
				goto err_probe_gpio;
			}
		}
	}

	spin_lock_init(&dd->queue_lock);
	mutex_init(&dd->core_lock);
	INIT_LIST_HEAD(&dd->queue);
	INIT_WORK(&dd->work_data, msm_spi_workq);
	init_waitqueue_head(&dd->continue_suspend);
	dd->workqueue = create_singlethread_workqueue(
		dev_name(master->dev.parent));
	if (!dd->workqueue)
		goto err_probe_workq;

	if (!request_mem_region(dd->mem_phys_addr, dd->mem_size,
				SPI_DRV_NAME)) {
		rc = -ENXIO;
		goto err_probe_reqmem;
	}

	dd->base = ioremap(dd->mem_phys_addr, dd->mem_size);
	if (!dd->base)
		goto err_probe_ioremap;
	rc = msm_spi_request_gsbi(dd);
	if (rc)
		goto err_probe_ioremap2;
	if (pdata && pdata->rsl_id) {
		struct remote_mutex_id rmid;
		rmid.r_spinlock_id = pdata->rsl_id;
		rmid.delay_us = SPI_TRYLOCK_DELAY;

		rc = remote_mutex_init(&dd->r_lock, &rmid);
		if (rc) {
			dev_err(&pdev->dev, "%s: unable to init remote_mutex "
				"(%s), (rc=%d)\n", rmid.r_spinlock_id,
				__func__, rc);
			goto err_probe_rlock_init;
		}
		dd->use_rlock = 1;
		dd->pm_lat = pdata->pm_lat;
		qos_req_list = pm_qos_add_request(PM_QOS_CPU_DMA_LATENCY, 
					    PM_QOS_DEFAULT_VALUE);
		if (IS_ERR(qos_req_list)) {
			dev_err(&pdev->dev, "%s: Failed to request pm_qos\n",
				__func__);
			goto err_probe_add_pm_qos;
		}
	}
	mutex_lock(&dd->core_lock);
	if (dd->use_rlock)
		remote_mutex_lock(&dd->r_lock);
	locked = 1;

	dd->dev = &pdev->dev;
	if (!pdata || !pdata->clk_name) {
		rc = -ENXIO;
		goto err_probe_clk_undefined;
	}
	dd->clk = clk_get(&pdev->dev, pdata->clk_name);
	if (IS_ERR(dd->clk)) {
		dev_err(&pdev->dev, "%s: unable to get %s\n", __func__,
			pdata->clk_name);
		rc = PTR_ERR(dd->clk);
		goto err_probe_clk_get;
	}
	if (pdata && pdata->pclk_name) {
		dd->pclk = clk_get(&pdev->dev, pdata->pclk_name);
		if (IS_ERR(dd->pclk)) {
			dev_err(&pdev->dev, "%s: unable to get %s\n", __func__,
				pdata->pclk_name);
			rc = PTR_ERR(dd->clk);
			goto err_probe_pclk_get;
		}
	}
	msm_spi_init_gsbi(dd);
	if (pdata && pdata->max_clock_speed)
		msm_spi_clock_set(dd, dd->pdata->max_clock_speed);

	rc = clk_enable(dd->clk);
	if (rc) {
		dev_err(&pdev->dev, "%s: unable to enable %s\n",
			__func__, pdata->clk_name);
		goto err_probe_clk_enable;
	}
	clk_enabled = 1;

	if (dd->pclk) {
		rc = clk_enable(dd->pclk);
		if (rc) {
			dev_err(&pdev->dev, "%s: unable to enable %s\n",
			__func__, pdata->pclk_name);
			goto err_probe_pclk_enable;
		}
		pclk_enabled = 1;
	}

	msm_spi_calculate_fifo_size(dd);
	if (dd->use_dma) {
		rc = msm_spi_init_dma(dd);
		if (rc)
			goto err_probe_dma;
	}

	/* Initialize registers */
	writel_relaxed(0x00000001, dd->base + SPI_SW_RESET);
	msm_spi_set_state(dd, SPI_OP_STATE_RESET);

	writel_relaxed(0x00000000, dd->base + SPI_OPERATIONAL);
	writel_relaxed(0x00000000, dd->base + SPI_CONFIG);
	writel_relaxed(0x00000000, dd->base + SPI_IO_MODES);
	writel_relaxed(0x0000007C, dd->base + SPI_ERROR_FLAGS_EN);

	writel_relaxed(SPI_IO_C_NO_TRI_STATE, dd->base + SPI_IO_CONTROL);
	rc = msm_spi_set_state(dd, SPI_OP_STATE_RESET);
	if (rc)
		goto err_probe_state;

	clk_disable(dd->clk);
	if (dd->pclk)
		clk_disable(dd->pclk);
	clk_enabled = 0;
	pclk_enabled = 0;

	dd->suspended = 0;
	dd->transfer_pending = 0;
	dd->multi_xfr = 0;
	dd->mode = SPI_MODE_NONE;

	rc = msm_spi_request_irq(dd, pdev->name, master);
	if (rc)
		goto err_probe_irq;

	msm_spi_disable_irqs(dd);
	if (dd->use_rlock)
		remote_mutex_unlock(&dd->r_lock);

	mutex_unlock(&dd->core_lock);
	locked = 0;

	rc = spi_register_master(master);
	if (rc)
		goto err_probe_reg_master;

	rc = sysfs_create_group(&(dd->dev->kobj), &dev_attr_grp);
	if (rc) {
		dev_err(&pdev->dev, "failed to create dev. attrs : %d\n", rc);
		goto err_attrs;
	}

	spi_debugfs_init(dd);

	return 0;

err_attrs:
err_probe_reg_master:
	msm_spi_free_irq(dd, master);
err_probe_irq:
err_probe_state:
	msm_spi_teardown_dma(dd);
err_probe_dma:
	if (pclk_enabled)
		clk_disable(dd->pclk);
err_probe_pclk_enable:
	if (clk_enabled)
		clk_disable(dd->clk);
err_probe_clk_enable:
	if (dd->pclk)
		clk_put(dd->pclk);
err_probe_pclk_get:
	clk_put(dd->clk);
err_probe_clk_get:
err_probe_clk_undefined:
	if (locked) {
		if (dd->use_rlock)
			remote_mutex_unlock(&dd->r_lock);
		mutex_unlock(&dd->core_lock);
	}
err_probe_add_pm_qos:
err_probe_rlock_init:
	msm_spi_release_gsbi(dd);
err_probe_ioremap2:
	iounmap(dd->base);
err_probe_ioremap:
	release_mem_region(dd->mem_phys_addr, dd->mem_size);
err_probe_reqmem:
	destroy_workqueue(dd->workqueue);
err_probe_workq:
err_probe_gpio:
	if (pdata && pdata->gpio_release)
		pdata->gpio_release();
err_probe_res2:
err_probe_res:
	spi_master_put(master);
err_probe_exit:
	return rc;
}

#ifdef CONFIG_PM
static int msm_spi_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct msm_spi    *dd;
	unsigned long      flags;
	struct msm_spi_platform_data *pdata = pdev->dev.platform_data;

	if (!master)
		goto suspend_exit;
	dd = spi_master_get_devdata(master);
	if (!dd)
		goto suspend_exit;

	/* Make sure nothing is added to the queue while we're suspending */
	spin_lock_irqsave(&dd->queue_lock, flags);
	dd->suspended = 1;
	spin_unlock_irqrestore(&dd->queue_lock, flags);

	/* Release GPIOs on suspending */
	if (pdata && pdata->gpio_release)
		pdata->gpio_release();

	/* Wait for transactions to end, or time out */
	wait_event_interruptible(dd->continue_suspend, !dd->transfer_pending);

suspend_exit:
	return 0;
}

static int msm_spi_resume(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct msm_spi    *dd;
	struct msm_spi_platform_data *pdata = pdev->dev.platform_data;

	if (!master)
		goto resume_exit;
	dd = spi_master_get_devdata(master);
	if (!dd)
		goto resume_exit;

	/* Re-initialize GPIOs on resuming */
	if (pdata && pdata->gpio_config)
		pdata->gpio_config();

	dd->suspended = 0;
resume_exit:
	return 0;
}
#else
#define msm_spi_suspend NULL
#define msm_spi_resume NULL
#endif /* CONFIG_PM */

static int __devexit msm_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct msm_spi    *dd = spi_master_get_devdata(master);
	struct msm_spi_platform_data *pdata = pdev->dev.platform_data;

	pm_qos_remove_request(qos_req_list);
	spi_debugfs_exit(dd);
	sysfs_remove_group(&pdev->dev.kobj, &dev_attr_grp);

	msm_spi_free_irq(dd, master);
	msm_spi_teardown_dma(dd);

	if (pdata && pdata->gpio_release)
		pdata->gpio_release();

	iounmap(dd->base);
	release_mem_region(dd->mem_phys_addr, dd->mem_size);
	msm_spi_release_gsbi(dd);
	clk_put(dd->clk);
	if (dd->pclk)
		clk_put(dd->pclk);
	destroy_workqueue(dd->workqueue);
	platform_set_drvdata(pdev, 0);
	spi_unregister_master(master);
	spi_master_put(master);

	return 0;
}

static struct platform_driver msm_spi_driver = {
	.driver		= {
		.name	= SPI_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend        = msm_spi_suspend,
	.resume         = msm_spi_resume,
	.remove		= __exit_p(msm_spi_remove),
};

static int __init msm_spi_init(void)
{
	return platform_driver_probe(&msm_spi_driver, msm_spi_probe);
}
module_init(msm_spi_init);

static void __exit msm_spi_exit(void)
{
	platform_driver_unregister(&msm_spi_driver);
}
module_exit(msm_spi_exit);
