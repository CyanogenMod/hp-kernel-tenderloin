#ifndef _high_level_funcs_h_
#define _high_level_funcs_h_

int program_device_sbw(struct a6_sbw_interface* sbw_ops, uint32_t read_address);
int verify_device_sbw(struct a6_sbw_interface* sbw_ops, uint32_t read_address);
int ttf_extract_fw_sbw(struct a6_sbw_interface* sbw_ops);
int ttf_extract_cache_clear(void);
int ttf_image_read(char *buf, size_t count, loff_t *ppos);
int get_checksum_data_sbw(struct a6_sbw_interface* sbw_ops, unsigned short* cksum1,
	unsigned short* cksum2, unsigned short* cksum_cycles,
	unsigned short* cksum_errors);

#endif
