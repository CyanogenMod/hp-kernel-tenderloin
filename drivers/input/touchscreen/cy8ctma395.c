//#define DEBUG

#include <linux/cy8ctma395.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/ihex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define BLOCK_LEN			256
#define ECC_LEN				32
#define DEVICE_CONFIG_ECCEN		0x08

#define DATA_RECORD_ADDR		0x00000000
#define DATA_RECORD_LEN			64
#define ECC_RECORD_ADDR			0x80000000
#define ECC_RECORD_LEN			64
#define NVL_RECORD_ADDR			0x90000000
#define NVL_RECORD_LEN			4

#define APACC_ADDR_WRITE		0x8b
#define APACC_DATA_READ			0x9f
#define APACC_DATA_WRITE		0xbb
#define DPACC_DATA_WRITE		0x99
#define DPACC_DP_CONFIG_WRITE		0xa9
#define DPACC_IDCODE_READ		0xa5

#define RESPONSE_OK			0x1
#define RESPONSE_WAIT			0x2
#define RESPONSE_FAULT			0x4

struct addr_data_pair {
	u32 addr;
	u32 data;
};

struct program_row {
	const struct ihex_binrec	*dat_rec;
	u16 				dat_rec_off;
	const struct ihex_binrec	*ecc_rec;
	u16 				ecc_rec_off;
	const struct ihex_binrec	*nvl_rec;
	int				ecc_enabled;
	u32 				sram_addr;
	u32				temp[2];
	u32				nr;
	u32				phub_ch_status_addr;
	u32				phub_ch_status_data;
	u32				phub_ch_basic_cfg_addr;
	u32				phub_cfgmem_cfg0_addr;
	u32				phub_cfgmem_cfg1_addr;
	u32				phub_tdmem_orig_td0_addr;
	u32				phub_tdmem_orig_td1_addr;
	u32				phub_ch_action_addr;
};

struct cy8ctma395_device_data {
	int	last_swdio_bit;
};

static int swd_read_bit(struct device *dev, int fast)
{
	int bit;
	struct cy8ctma395_platform_data *pdat = dev->platform_data;

	gpio_set_value(pdat->swdck, 0);
	if (!fast) udelay(1);
	bit = gpio_get_value(pdat->swdio);
	gpio_set_value(pdat->swdck, 1);

	return (bit);
}

static void swd_write_bit(struct device *dev, int bit, int fast)
{
	struct cy8ctma395_device_data *dat = dev_get_drvdata(dev);
	struct cy8ctma395_platform_data *pdat = dev->platform_data;

	if (bit == dat->last_swdio_bit) {
		gpio_set_value(pdat->swdck, 0);
		gpio_set_value(pdat->swdck, 1);
	}

	else {
		gpio_set_value(pdat->swdck, 0);
		gpio_set_value(pdat->swdio, dat->last_swdio_bit = bit);
		if (!fast) udelay(1);
		gpio_set_value(pdat->swdck, 1);
	}
}

static void swd_turnaround(struct device *dev, int out)
{
	struct cy8ctma395_device_data *dat = dev_get_drvdata(dev);
	struct cy8ctma395_platform_data *pdat = dev->platform_data;

	if (out)
		gpio_direction_output(pdat->swdio, dat->last_swdio_bit = 1);

	else
		gpio_direction_input(pdat->swdio);

	gpio_set_value(pdat->swdck, 0);
	gpio_set_value(pdat->swdck, 1);
}

static u8 swd_read_response(struct device *dev, int fast)
{
	u8 response;

	response = swd_read_bit(dev, fast);
	response |= swd_read_bit(dev, fast) << 1;
	response |= swd_read_bit(dev, fast) << 2;

	return (response);
}

static u8 swd_read_byte(struct device *dev)
{
	u8 byte;

	byte = swd_read_bit(dev, 0);
	byte |= swd_read_bit(dev, 0) << 1;
	byte |= swd_read_bit(dev, 0) << 2;
	byte |= swd_read_bit(dev, 0) << 3;
	byte |= swd_read_bit(dev, 0) << 4;
	byte |= swd_read_bit(dev, 0) << 5;
	byte |= swd_read_bit(dev, 0) << 6;
	byte |= swd_read_bit(dev, 0) << 7;

	return (byte);
}

static void swd_write_byte(struct device *dev, u8 byte, int fast)
{
	swd_write_bit(dev, (byte >> 0) & 0x01, fast);
	swd_write_bit(dev, (byte >> 1) & 0x01, fast);
	swd_write_bit(dev, (byte >> 2) & 0x01, fast);
	swd_write_bit(dev, (byte >> 3) & 0x01, fast);
	swd_write_bit(dev, (byte >> 4) & 0x01, fast);
	swd_write_bit(dev, (byte >> 5) & 0x01, fast);
	swd_write_bit(dev, (byte >> 6) & 0x01, fast);
	swd_write_bit(dev, (byte >> 7) & 0x01, fast);
}

static int even_parity(u32 data)
{
	int parity = 0;

	for (; data; data >>= 1)
		parity ^= data;

	return (parity & 0x1);
}

static int swd_read_data(struct device *dev, u32 *data)
{
	int rc;
	int parity;

	*data = swd_read_byte(dev);
	*data |= (u32)swd_read_byte(dev) << 8;
	*data |= (u32)swd_read_byte(dev) << 16;
	*data |= (u32)swd_read_byte(dev) << 24;
	parity = swd_read_bit(dev, 0);

	if (parity != even_parity(*data)) {
		dev_err(dev, "swd data parity error, data=0x%08x parity=%x\n",
			*data, parity);
		rc = -EIO;
		goto exit;
	}

	rc = 0;
exit:
	return (rc);
}

static void swd_write_data(struct device *dev, u32 data, int fast)
{
	swd_write_byte(dev, data >> 0, fast);
	swd_write_byte(dev, data >> 8, fast);
	swd_write_byte(dev, data >> 16, fast);
	swd_write_byte(dev, data >> 24, fast);
	swd_write_bit(dev, even_parity(data), fast);
}

static int swd_read(struct device *dev, u8 command, u32 *data)
{
	u8 response;
	int rc;
	int retries = 0;
	struct cy8ctma395_platform_data *pdat = dev->platform_data;

retry:
	swd_write_byte(dev, command, 0);
	swd_turnaround(dev, 0);
	response = swd_read_response(dev, 0);

	rc = swd_read_data(dev, data);
	if (rc < 0)
		goto exit;

	swd_turnaround(dev, 1);

	if ((response == RESPONSE_WAIT) && (retries++ < pdat->swd_wait_retries))
		goto retry;

	if (response != RESPONSE_OK) {
		dev_err(dev, "swd read failed, command=%02x response=%x\n",
			command, response);
		rc = -EIO;
		goto exit;
	}

	rc = 0;
exit:
	return (rc);
}

static u8 __swd_write(struct device *dev, u8 command, u32 data, int fast)
{
	u8 response;

	swd_write_byte(dev, command, fast);
	swd_turnaround(dev, 0);
	response = swd_read_response(dev, fast);
	swd_turnaround(dev, 1);
	swd_write_data(dev, data, fast);

	return (response);
}

static int swd_write(struct device *dev, u8 command, u32 data)
{
	u8 response;
	int rc;
	int retries = 0;
	struct cy8ctma395_platform_data *pdat = dev->platform_data;

retry:
	response = __swd_write(dev, command, data, 0);

	if ((response == RESPONSE_WAIT) && (retries++ < pdat->swd_wait_retries))
		goto retry;

	if (response != RESPONSE_OK) {
		dev_err(dev, "swd write failed, command=%02x data=%08x response=%x\n",
			command, data, response);
		rc = -EIO;
		goto exit;
	}

	rc = 0;
exit:
	return (rc);
}

static int apacc_addr_write(struct device *dev, u32 addr)
{
	dev_dbg(dev, "apacc addr write [%08x]\n", addr);

	return (swd_write(dev, APACC_ADDR_WRITE, addr));
}

static int apacc_data_read(struct device *dev, u32 *data, int nr)
{
	int i;
	int rc;
	u32 unused;

	rc = swd_read(dev, APACC_DATA_READ, &unused);
	if (rc < 0)
		goto exit;

	dev_dbg(dev, "apacc data read (dummy) [%08x]\n", unused);

	for (i = 0; i < nr; i++) {
		rc = swd_read(dev, APACC_DATA_READ, &data[i]);
		if (rc < 0)
			goto exit;

		dev_dbg(dev, "apacc data read [%08x]\n", data[i]);
	}

exit:
	return (rc);
}

static int apacc_addr_write_data_read(struct device *dev, u32 addr, u32 *data,
					int nr)
{
	int rc;

	rc = apacc_addr_write(dev, addr);
	if (rc < 0)
		goto exit;

	rc = apacc_data_read(dev, data, nr);
exit:
	return (rc);
}

static inline int apacc_data_write(struct device *dev, u32 data)
{
	dev_dbg(dev, "apacc data write [%08x]\n", data);

	return (swd_write(dev, APACC_DATA_WRITE, data));
}

static int apacc_addr_data_write(struct device *dev, u32 addr, u32 data)
{
	int rc;

	rc = apacc_addr_write(dev, addr);
	if (rc < 0)
		goto exit;

	rc = apacc_data_write(dev, data);
exit:
	return (rc);
}

static int apacc_addr_data_write_seq(struct device *dev,
					struct addr_data_pair *seq, int nr)
{
	int i;
	int rc;

	for (i = 0; i < nr; i++) {
		rc = apacc_addr_data_write(dev, seq[i].addr, seq[i].data);
		if (rc < 0)
			goto exit;
	}

	rc = 0;
exit:
	return (rc);
}

static int port_acquire(struct device *dev, u32 *id, u8 *rev)
{
	int rc;
	u32 data[2];
	struct cy8ctma395_device_data *dat = dev_get_drvdata(dev);
	struct cy8ctma395_platform_data *pdat = dev->platform_data;

	rc = pdat->swdck_request(1);
	if (rc < 0)
		goto request_swdck_failed;

	rc = pdat->swdio_request(1);
	if (rc < 0)
		goto request_swdio_failed;

	gpio_set_value(pdat->swdck, 1);
	gpio_set_value(pdat->swdio, dat->last_swdio_bit = 1);
	gpio_set_value(pdat->xres, 0);
	usleep(pdat->xres_us);

	local_irq_disable();
	{
		u8 response;
		int retries = 0;

		gpio_set_value(pdat->xres, 1);
retry:
		response = __swd_write(dev, DPACC_DATA_WRITE, 0x7B0C06DB, 1);
		if (response != RESPONSE_OK) {
			if (retries++ < pdat->port_acquire_retries)
				goto retry;

			rc = -EIO;
			goto enable;
		}

		rc = apacc_addr_data_write(dev, 0x00050210, 0xEA7E30A9);
	}

enable:
	local_irq_enable();

	if (rc < 0) {
		dev_err(dev, "failed to acquire port\n");
		goto acquire_failed;
	}

	{
		struct addr_data_pair seq[] = {
			{0x00050220, 0x000000B3},
			{0x000046EA, 0x00000001},
			{0x000043A0, 0x000000BF},
			{0x00004200, 0x00000000},
		};

		rc = apacc_addr_data_write_seq(dev, seq, ARRAY_SIZE(seq));
		if (rc < 0)
			goto acquire_failed;
	}

	dev_dbg(dev, "dpacc idcode read\n");
	rc = swd_read(dev, DPACC_IDCODE_READ, &data[0]);
	if (rc < 0)
		goto acquire_failed;

	rc = apacc_addr_write_data_read(dev, 0x000046EC, &data[1], 1);
	if (rc < 0)
		goto acquire_failed;

	if (id)
		*id = data[0];

	if (rev)
		*rev = data[1];

	return (0);

acquire_failed:
	gpio_set_value(pdat->xres, 0);
	usleep(pdat->xres_us);
	gpio_set_value(pdat->xres, 1);
	(void)pdat->swdio_request(0);
request_swdio_failed:
	(void)pdat->swdck_request(0);
request_swdck_failed:

	return (rc);
}

static void port_release(struct device *dev)
{
	struct cy8ctma395_platform_data *pdat = dev->platform_data;

	gpio_set_value(pdat->xres, 0);
	usleep(pdat->xres_us);
	gpio_set_value(pdat->xres, 1);
	(void)pdat->swdio_request(0);
	(void)pdat->swdck_request(0);
}

static int poll_status_reg(struct device *dev, u8 expected, const char *step)
{
	int rc;
#ifdef DEBUG
	u64 ms;
#endif /* DEBUG */
	u32 data;
	struct timespec now;
	struct timespec expiry;
	struct cy8ctma395_platform_data *pdat = dev->platform_data;

	ktime_get_ts(&expiry);
	timespec_add_ns(&expiry, pdat->status_reg_timeout_ms * NSEC_PER_MSEC);

	for (;;) {
		rc = apacc_addr_write_data_read(dev, 0x00004722, &data, 1);
		if (rc < 0)
			goto exit;

		/* only low byte is relevant in status reg */
		if ((data & 0xff) == expected)
			break;

		ktime_get_ts(&now);
		if (timespec_compare(&now, &expiry) >= 0) {
			dev_err(dev, "timed out waiting for '%s'\n", step);
			rc = -ETIME;
			goto exit;
		}

		msleep(jiffies_to_msecs(1));
	}

#ifdef DEBUG
	ktime_get_ts(&now);
	timespec_add_ns(&now, NSEC_PER_SEC);
	ms = timespec_to_ns(&now) - timespec_to_ns(&expiry);
	do_div(ms, NSEC_PER_MSEC);
	dev_dbg(dev, "'%s' time=%lldms\n", step, ms);
#endif /* DEBUG */

	rc = 0;
exit:
	return (rc);
}

static int find_record(struct device *dev, const struct ihex_binrec *beg,
			u32 addr, u16 len, const struct ihex_binrec **rec)
{
	int rc;

	for (*rec = beg; be32_to_cpu((*rec)->addr) != addr;) {
		*rec = ihex_next_binrec(*rec);
		if (!*rec) {
			dev_err(dev, "no record with address 0x%08x\n", addr);
			rc = -EINVAL;
			goto exit;
		}
	}

	if (be16_to_cpu((*rec)->len) != len) {
		dev_err(dev, "record 0x%08x is %hu bytes, expected %hu\n", addr,
			be16_to_cpu((*rec)->len), len);
		rc = -EINVAL;
		goto exit;
	}

	rc = 0;
exit:
	return (rc);
}

static int load_row(struct device *dev, const struct ihex_binrec **rec,
				u16 *rec_off, u32 beg, u32 end)
{
	int rc;
	u32 data;

	for (; beg < end; beg += 4) {
		if (*rec_off >= be16_to_cpu((*rec)->len)) {
			rc = find_record(dev, *rec, be32_to_cpu((*rec)->addr)
						+ be16_to_cpu((*rec)->len),
						be16_to_cpu((*rec)->len), rec);
			if (rc < 0)
				goto exit;

			*rec_off = 0;
		}

		data = (*rec)->data[*rec_off] << 0;
		data |= (*rec)->data[*rec_off+1] << 8;
		data |= (*rec)->data[*rec_off+2] << 16;
		data |= (*rec)->data[*rec_off+3] << 24;
		rc = apacc_addr_data_write(dev, beg+4, data);
		if (rc < 0)
			goto exit;

		*rec_off += 4;
	}

	rc = 0;
exit:
	return (rc);
}

static int program_row(struct device *dev, struct program_row *row)
{
	int rc;
	int len = BLOCK_LEN;

	rc = apacc_addr_data_write(dev, row->sram_addr, 0x0002D5B6);
	if (rc < 0)
		goto exit;

	rc = load_row(dev, &row->dat_rec, &row->dat_rec_off, row->sram_addr,
			row->sram_addr + BLOCK_LEN);
	if (rc < 0)
		goto exit;

	if (!row->ecc_enabled) {
		rc = load_row(dev, &row->ecc_rec, &row->ecc_rec_off,
				row->sram_addr + BLOCK_LEN,
				row->sram_addr + BLOCK_LEN + ECC_LEN);
		if (rc < 0)
			goto exit;

		len += ECC_LEN;
	}

	{
		struct addr_data_pair seq[] = {
			{row->sram_addr + len + 0x4, 0xB6000000},
			{row->sram_addr + len + 0x8, 0x000007DA},
			{row->sram_addr + len + 0xC,
				((row->temp[1] & 0xff) << 16)
					| ((row->temp[0] & 0xff) << 8)
					| (row->nr & 0xff)},
			{row->phub_ch_status_addr, row->phub_ch_status_data},
			{row->phub_ch_basic_cfg_addr, 0x00000021},
			{row->phub_cfgmem_cfg0_addr, 0x00000080},
			{row->phub_cfgmem_cfg1_addr, 0x00000000},
			{row->phub_tdmem_orig_td0_addr, 0x01FF0000 + len + 0xf},
			{row->phub_tdmem_orig_td1_addr, 0x47200000 + row->sram_addr},
		};

		rc = apacc_addr_data_write_seq(dev, seq, ARRAY_SIZE(seq));
		if (rc < 0)
			goto exit;
	}

	rc = poll_status_reg(dev, 0x2, "dma");
	if (rc < 0)
		goto exit;

	rc = apacc_addr_data_write(dev, row->phub_ch_action_addr, 0x00000001);
exit:
	if (rc < 0)
		dev_err(dev, "failure programming row %u\n", row->nr);

	return (rc);
}

static int checksum(struct device *dev, u16 *sum)
{
	int rc;
	u32 data[4];
	struct cy8ctma395_platform_data *pdat = dev->platform_data;

	{
		struct addr_data_pair seq[] = {
			{0x00004720, 0x000000B6},
			{0x00004720, 0x000000DF},
			{0x00004720, 0x0000000C},
			{0x00004720, 0x00000000},
			{0x00004720, 0x00000000},
			{0x00004720, 0x00000000},
			{0x00004720, 0x00000000},
			{0x00004720, pdat->nr_blocks - 1},
		};

		rc = apacc_addr_data_write_seq(dev, seq, ARRAY_SIZE(seq));
		if (rc < 0)
			goto exit;
	}

	rc = poll_status_reg(dev, 0x1, "checksum");
	if (rc < 0)
		goto exit;

	rc = apacc_addr_write_data_read(dev, 0x00004720, data, 4);
	if (rc < 0)
		goto exit;

	rc = poll_status_reg(dev, 0x2, "idle");
	if (rc < 0)
		goto exit;

	*sum = ((data[2] & 0xff) << 8) | (data[3] & 0xff);
exit:
	return (rc);
}

static ssize_t cy8ctma395_attr_checksum_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	u16 sum;
	ssize_t rc;

	rc = port_acquire(dev, NULL, NULL);
	if (rc < 0)
		goto exit;

	rc = checksum(dev, &sum);
	if (rc < 0)
		goto release;

	rc = snprintf(buf, PAGE_SIZE, "%04x\n", sum);
release:
	port_release(dev);
exit:
	return (rc);
}

static struct device_attribute cy8ctma395_attr_checksum = {
        .attr = {
                .name = "checksum",
                .mode = S_IRUSR,
        },
        .show = cy8ctma395_attr_checksum_show,
};

static ssize_t cy8ctma395_attr_id_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u8 rev;
	u32 id;
	ssize_t rc;

	rc = port_acquire(dev, &id, &rev);
	if (rc < 0)
		goto exit;

	rc = snprintf(buf, PAGE_SIZE, "%08x %02x\n", id, rev);
	port_release(dev);
exit:
	return (rc);
}

static struct device_attribute cy8ctma395_attr_id = {
        .attr = {
                .name = "id",
                .mode = S_IRUSR,
        },
        .show = cy8ctma395_attr_id_show,
};

static int read_device_config(struct device *dev, u8 *regs)
{
	int i;
	int rc;
	u32 data[4];

	{
		struct addr_data_pair seq[] = {
			{0x00005112, 0x00000000},
			{0x00005113, 0x00000004},
			{0x00005114, 0x00000000},
			{0x00005110, 0x00000004},
		};

		rc = apacc_addr_data_write_seq(dev, seq, ARRAY_SIZE(seq));
		if (rc < 0)
			goto exit;
	}

	for (i = 0; i < 4; i++) {
		struct addr_data_pair seq[] = {
			{0x00004720, 0x000000B6},
			{0x00004720, 0x000000D6},
			{0x00004720, 0x00000003},
			{0x00004720, 0x00000080},
			{0x00004720, i},
		};

		rc = apacc_addr_data_write_seq(dev, seq, ARRAY_SIZE(seq));
		if (rc < 0)
			goto exit;

		rc = poll_status_reg(dev, 0x1, "device config");
		if (rc < 0)
			goto exit;

		rc = apacc_addr_write_data_read(dev, 0x00004720, &data[i], 1);
		if (rc < 0)
			goto exit;

		regs[i] = data[i];

		rc = poll_status_reg(dev, 0x2, "idle");
		if (rc < 0)
			goto exit;
	}

exit:
	return (rc);
}

static int write_device_config(struct device *dev, u8 *regs)
{
	int i;
	int rc;

	for (i = 0; i < 4; i++) {
		struct addr_data_pair seq[] = {
			{0x00004720, 0x000000B6},
			{0x00004720, 0x000000D3},
			{0x00004720, 0x00000000},
			{0x00004720, 0x00000080},
			{0x00004720, i},
			{0x00004720, regs[i]},
		};

		rc = apacc_addr_data_write_seq(dev, seq, ARRAY_SIZE(seq));
		if (rc < 0)
			goto exit;

		rc = poll_status_reg(dev, 0x2, "device config");
		if (rc < 0)
			goto exit;
	}

	{
		struct addr_data_pair seq[] = {
			{0x00004720, 0x000000B6},
			{0x00004720, 0x000000D9},
			{0x00004720, 0x00000006},
			{0x00004720, 0x00000080},
		};

		rc = apacc_addr_data_write_seq(dev, seq, ARRAY_SIZE(seq));
		if (rc < 0)
			goto exit;
	}

	rc = poll_status_reg(dev, 0x2, "device config");
	if (rc < 0)
		goto exit;

 exit:
	return (rc);

}

static ssize_t cy8ctma395_attr_device_config_show(
			struct device *dev, struct device_attribute *attr,
			char *buf)
{
	u8 regs[4];
	ssize_t rc;

	rc = port_acquire(dev, NULL, NULL);
	if (rc < 0)
		goto exit;

	rc = read_device_config(dev, regs);
	if (rc < 0)
		goto release;

	rc = snprintf(buf, PAGE_SIZE, "%02x %02x %02x %02x\n", regs[0], regs[1],
			regs[2], regs[3]);
release:
	port_release(dev);
exit:
	return (rc);
}

static ssize_t cy8ctma395_attr_device_config_store(
			struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	u8 regs[4];
	ssize_t rc;

	rc = sscanf(buf, "%hhx %hhx %hhx %hhx", &regs[0], &regs[1], &regs[2],
			&regs[3]);
	if (rc < 4) {
		rc = -EINVAL;
		goto exit;
	}

	rc = port_acquire(dev, NULL, NULL);
	if (rc < 0)
		goto exit;

	rc = write_device_config(dev, regs);
	if (rc < 0)
		goto release;

	rc = count;
release:
	port_release(dev);
exit:
	return (rc);
}

static struct device_attribute cy8ctma395_attr_device_config = {
        .attr = {
                .name = "device_config",
                .mode = S_IRUSR|S_IWUSR,
        },
        .show = cy8ctma395_attr_device_config_show,
        .store = cy8ctma395_attr_device_config_store,
};

static ssize_t cy8ctma395_attr_flash_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	u8 rev;
	u8 regs[4];
	int i;
	u16 sum;
	u32 id;
	ssize_t rc;
	struct program_row row;
	const struct firmware *fw = NULL;
	struct cy8ctma395_platform_data *pdat = dev->platform_data;

	rc = request_ihex_firmware(&fw, buf, dev);
	if (rc < 0) {
		dev_err(dev, "error %d requesting firmware %s\n", rc, buf);
		goto exit;
	}

	row.dat_rec_off = 0;
	rc = find_record(dev, (struct ihex_binrec *)fw->data, DATA_RECORD_ADDR,
				DATA_RECORD_LEN, &row.dat_rec);
	if (rc < 0)
		goto release_firmware;

	row.ecc_rec_off = 0;
	rc = find_record(dev, (struct ihex_binrec *)fw->data, ECC_RECORD_ADDR,
				ECC_RECORD_LEN, &row.ecc_rec);
	if (rc < 0)
		goto release_firmware;

	rc = find_record(dev, (struct ihex_binrec *)fw->data, NVL_RECORD_ADDR,
				NVL_RECORD_LEN, &row.nvl_rec);
	if (rc < 0)
		goto release_firmware;

	rc = port_acquire(dev, &id, &rev);
	if (rc < 0)
		goto release_firmware;

	dev_info(dev, "jtag id=%08x revision=%08x\n", id, rev);

	/* erase all */
	{
		struct addr_data_pair seq[] = {
			{0x00004720, 0x000000B6},
			{0x00004720, 0x000000DC},
			{0x00004720, 0x00000009},
		};

		rc = apacc_addr_data_write_seq(dev, seq, ARRAY_SIZE(seq));
		if (rc < 0)
			goto port_release;
	}

	rc = poll_status_reg(dev, 0x2, "erase all");
	if (rc < 0)
		goto port_release;

	/* read_device config */
	rc = read_device_config(dev, regs);
	if (rc < 0)
		goto port_release;
	
	/* check if we need to flash device_config by comparing */
	/* the NVL bytes from target device with those in hex file */
	if (memcmp(regs, row.nvl_rec->data, NVL_RECORD_LEN) != 0) {
	  rc = write_device_config(dev, (u8 *) row.nvl_rec->data);
		if (rc < 0)
			goto port_release;
	}

	/* program */
	for (i = 0; i < 2; i++) {
		struct addr_data_pair seq[] = {
			{0x00004720, 0x000000B6},
			{0x00004720, 0x000000E1},
			{0x00004720, 0x0000000E},
			{0x00004720, 0x00000003},
		};

		rc = apacc_addr_data_write_seq(dev, seq, ARRAY_SIZE(seq));
		if (rc < 0)
			goto port_release;

		rc = poll_status_reg(dev, 0x1, "temperature data");
		if (rc < 0)
			goto port_release;

		rc = apacc_addr_write_data_read(dev, 0x00004720, row.temp, 2);
		if (rc < 0)
			goto port_release;

		rc = poll_status_reg(dev, 0x2, "idle");
		if (rc < 0)
			goto port_release;
	}

	dev_info(dev, "temperature sign=%08x magnitude=%08x\n", row.temp[0],
			row.temp[1]);

	dev_dbg(dev, "dpacc dp config write [%08x]\n", 0x00000004);
	rc = swd_write(dev, DPACC_DP_CONFIG_WRITE, 0x00000004);
	if (rc < 0)
		goto port_release;

	row.ecc_enabled = !!(regs[3] & DEVICE_CONFIG_ECCEN);
	dev_info(dev, "ecc is %s\n", row.ecc_enabled ? "enabled" : "disabled");

	for (i = 0; i < pdat->nr_blocks;) {
		row.sram_addr = 0x000;
		row.nr = i++;
		row.phub_ch_status_addr = 0x00007018;
		row.phub_ch_status_data = 0x00000000;
		row.phub_ch_basic_cfg_addr = 0x00007010;
		row.phub_cfgmem_cfg0_addr = 0x00007600;
		row.phub_cfgmem_cfg1_addr = 0x00007604;
		row.phub_tdmem_orig_td0_addr = 0x00007800;
		row.phub_tdmem_orig_td1_addr = 0x00007804;
		row.phub_ch_action_addr = 0x00007014;
		rc = program_row(dev, &row);
		if (rc < 0)
			goto port_release;

		row.sram_addr = 0x200;
		row.nr = i++;
		row.phub_ch_status_addr = 0x00007028;
		row.phub_ch_status_data = 0x00000100;
		row.phub_ch_basic_cfg_addr = 0x00007020;
		row.phub_cfgmem_cfg0_addr = 0x00007608;
		row.phub_cfgmem_cfg1_addr = 0x0000760C;
		row.phub_tdmem_orig_td0_addr = 0x00007808;
		row.phub_tdmem_orig_td1_addr = 0x0000780C;
		row.phub_ch_action_addr = 0x00007024;
		rc = program_row(dev, &row);
		if (rc < 0)
			goto port_release;
	}

	rc = poll_status_reg(dev, 0x2, "idle");
	if (rc < 0)
		goto port_release;

	rc = checksum(dev, &sum);
	if (rc < 0)
		goto port_release;

	dev_info(dev, "checksum %04x\n", sum);
	rc = count;
port_release:
	port_release(dev);
release_firmware:
	release_firmware(fw);
exit:
	return (rc);
}

static struct device_attribute cy8ctma395_attr_flash = {
        .attr = {
                .name = "flash",
                .mode = S_IWUSR,
        },
        .store = cy8ctma395_attr_flash_store,
};

static ssize_t cy8ctma395_attr_vdd_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct cy8ctma395_platform_data *pdat = dev->platform_data;

	pdat->vdd_enable(!!simple_strtoul(buf, NULL, 10));

	return (count);
}

static struct device_attribute cy8ctma395_attr_vdd = {
        .attr = {
                .name = "vdd",
                .mode = S_IWUSR,
        },
        .store = cy8ctma395_attr_vdd_store,
};

static void cy8ctma395_xres_assert(struct cy8ctma395_platform_data *pdat, int assert)
{
	if (assert) {
		gpio_set_value(pdat->xres, 0);
		udelay(pdat->xres_us);
	}
	else
		gpio_set_value(pdat->xres, 1);

}

static ssize_t cy8ctma395_attr_xres_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	int assert = !!simple_strtoul(buf, NULL, 10);
	struct cy8ctma395_platform_data *pdat = dev->platform_data;

	cy8ctma395_xres_assert(pdat, assert);

	return (count);
}

static struct device_attribute cy8ctma395_attr_xres = {
	.attr = {
		.name = "xres",
		.mode = S_IWUGO,
	},
	.store = cy8ctma395_attr_xres_store,
};

static int cy8ctma395_device_probe(struct platform_device *pdev)
{
	int rc;
	struct cy8ctma395_device_data *dat;
	struct cy8ctma395_platform_data *pdat = pdev->dev.platform_data;

	if (!pdat) {
		rc = -ENODEV;
		goto failed;
	}

	dat = kzalloc(sizeof(*dat), 0);
	if (!dat) {
		rc = -ENOMEM;
		goto failed;
	}

	dev_set_drvdata(&pdev->dev, dat);

	rc = device_create_file(&pdev->dev, &cy8ctma395_attr_checksum);
	if (rc < 0)
		goto attr_checksum_failed;

	rc = device_create_file(&pdev->dev, &cy8ctma395_attr_flash);
	if (rc < 0)
		goto attr_flash_failed;

	rc = device_create_file(&pdev->dev, &cy8ctma395_attr_id);
	if (rc < 0)
		goto attr_id_failed;

	rc = device_create_file(&pdev->dev, &cy8ctma395_attr_device_config);
	if (rc < 0)
		goto attr_device_config_failed;

	rc = device_create_file(&pdev->dev, &cy8ctma395_attr_xres);
	if (rc < 0)
		goto attr_xres_failed;

	if (pdat->vdd_enable) {
		cy8ctma395_xres_assert(pdat, 1);
		pdat->vdd_enable(1);
		cy8ctma395_xres_assert(pdat, 0);

		rc = device_create_file(&pdev->dev, &cy8ctma395_attr_vdd);
		if (rc < 0)
			goto attr_vdd_failed;
	}
	
	rc = 0;
	goto exit;

attr_vdd_failed:
	pdat->vdd_enable(0);

	device_remove_file(&pdev->dev, &cy8ctma395_attr_xres);
attr_xres_failed:
	device_remove_file(&pdev->dev, &cy8ctma395_attr_device_config);
attr_device_config_failed:
	device_remove_file(&pdev->dev, &cy8ctma395_attr_id);
attr_id_failed:
	device_remove_file(&pdev->dev, &cy8ctma395_attr_flash);
attr_flash_failed:
	device_remove_file(&pdev->dev, &cy8ctma395_attr_checksum);
attr_checksum_failed:
	kfree(dat);
failed:
	dev_err(&pdev->dev, "probe failed with %d\n", rc);
exit:
	return (rc);
}

static int cy8ctma395_device_remove(struct platform_device *pdev)
{
	struct cy8ctma395_device_data *dat = dev_get_drvdata(&pdev->dev);
	struct cy8ctma395_platform_data *pdat = pdev->dev.platform_data;

	if (pdat->vdd_enable)
		device_remove_file(&pdev->dev, &cy8ctma395_attr_vdd);

	device_remove_file(&pdev->dev, &cy8ctma395_attr_xres);
	device_remove_file(&pdev->dev, &cy8ctma395_attr_device_config);
	device_remove_file(&pdev->dev, &cy8ctma395_attr_id);
	device_remove_file(&pdev->dev, &cy8ctma395_attr_flash);
	device_remove_file(&pdev->dev, &cy8ctma395_attr_checksum);
	kfree(dat);

	return (0);
}

static struct platform_driver cy8ctma395_driver = {
	.driver = {
		.name = CY8CTMA395_DRIVER,
	},
	.probe = cy8ctma395_device_probe,
	.remove = __devexit_p(cy8ctma395_device_remove),
};

static int __init cy8ctma395_module_init(void)
{
	int rc;

	rc = platform_driver_register(&cy8ctma395_driver);

	return (rc);
}

static void __exit cy8ctma395_module_exit(void)
{
	platform_driver_unregister(&cy8ctma395_driver);
}

module_init(cy8ctma395_module_init);
module_exit(cy8ctma395_module_exit);

MODULE_DESCRIPTION("cy8ctma395 driver");
MODULE_LICENSE("GPL");
