/****************************************************************************/
/* Includes                                                                 */
/****************************************************************************/
#include "a6_host_adapter.h"       // Maps function calls to host porting-layer implementations
#include "jtag_funcs.h"	        	// Spy-by-wire JTAG functions
#include "low_level_funcs.h"		// low level user functions

#define LOCAL_TRACE 0

/****************************************************************************/
/* Global types                                                             */
/****************************************************************************/

/****************************************************************************/
/* Main section of Replicator program: User can modify/insert code as needed*/
/****************************************************************************/
/*
   Note: All High Level JTAG Functions are applied here.
*/

// definition for current implementation mappings used by the sbw code...
uint16_t (*SetSBWTCK)(void) = NULL;
uint16_t (*ClrSBWTCK)(void) = NULL;
uint16_t (*SetSBWTDIO)(void) = NULL;
uint16_t (*ClrSBWTDIO)(void) = NULL;
uint16_t (*SetInSBWTDIO)(void) = NULL;
uint16_t (*SetOutSBWTDIO)(void) = NULL;
uint16_t (*GetSBWTDIO)(void) = NULL;
uint16_t (*SetSBWAKEUP)(void) = NULL;
uint16_t (*ClrSBWAKEUP)(void) = NULL;
void (*delay)(uint32_t delay_us) = NULL;
//

typedef enum {
	SBW_OK = 0,
	SBW_TOK,
	SBW_EOL,
	SBW_EOS,
	SBW_SOS,
	SBW_EOI,
	SBW_STATE_ERROR
} SBW_STATE_CODE;

#define SIZEOF_NEWLINE (1)

static int hexval(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	else if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	else if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;

	return 0;
}

SBW_STATE_CODE sbw_get_token(uint8_t* read_p, uint8_t* write_p, uint32_t* read_len_p, uint32_t* write_len_p)
{
	SBW_STATE_CODE ret;

	//assert(read_p && read_len_p && write_len_p);


	// end-of-line
	if (0x0d == *read_p && 0x0a == *(read_p+1)) {
		*read_len_p = 2;
		*write_len_p = 0;
		ret = SBW_EOL;

		//printk("<EOL>\n");
	}
	// end-of-image
	else if (('q' == *read_p) || ('Q' == *read_p)) {
		*read_len_p = 1;
		*write_len_p = 0;
		ret = SBW_EOI;

		//printk("<EOI>\n");
	}
	else {
		uint32_t val = 0;

		// section start
		if ('@' == read_p[0]) {
			ret = SBW_SOS;
			val = hexval(read_p[1 + 0]) << 12;
			val |= hexval(read_p[1 +1]) << 8;
			val |= hexval(read_p[1 +2]) << 4;
			val |= hexval(read_p[1 +3]);

			*read_len_p = 1+4+2; //'@' + XXXX + CRLF
			*write_len_p = 2;    // two bytes written
			//printk("<SOS>\n");
		}
		// data
		else {
			ret = SBW_TOK;
			val = hexval(read_p[0]) << 4;
			val |= hexval(read_p[1]);

			// handle variation: the last 2-byte value on a line may not include trailing space
			*read_len_p = 2+1; //XX + ' '
			//*read_len_p = 2 + (' ' == read_p[2]) ? 1 : 0; //XX + ' '
			*write_len_p = 1;    // one byte written
		}

		// no target? skip the actual write...
		if (write_p) {
			//*((uint32_t*)write_p) = val;
			write_p[0] = val & 0x000000ff;
			write_p[1] = (val >> 8) & 0x000000ff;
			write_p[2] = (val >> 16) & 0x000000ff;
			write_p[3] = (val >> 24) & 0x000000ff;
		}

		//printk("%02x ", val);
	}

	return ret;
}


SBW_STATE_CODE sbw_parse_line(uint8_t* read_p, uint8_t* write_p, uint32_t* read_len_p, uint32_t* write_len_p)
{
	SBW_STATE_CODE ret;
	uint32_t total_read_len = 0, total_write_len = 0, val = 0, r_len = 0, w_len = 0;

	do {
		ret = sbw_get_token(read_p, (uint8_t*)&val, &r_len, &w_len);
		// end-of-line; break out of loop
		if (SBW_EOL == ret) {
			total_read_len += r_len;
			*read_len_p = total_read_len;
			total_write_len += w_len;
			*write_len_p = total_write_len;
		}
		// regular token; keep looping
		else if (SBW_TOK == ret) {
			*((uint8_t*)write_p) = (uint8_t)val;
			total_read_len += r_len;
			read_p += r_len;
			total_write_len += w_len;
			write_p += w_len;
		}
		// map start-of-section/end-of-image to end-of-section
		else if ((SBW_SOS == ret) || (SBW_EOI == ret)) {
			*read_len_p = *write_len_p = 0;
			ret = SBW_EOS;
		}
		// state mismatch
		else {
			printk("SBW_ERROR[sbw_parse_line]: wrong state returned; state:  %d\n", ret);
			ret =  SBW_STATE_ERROR;
		}
	} while (SBW_TOK == ret);


	return ret;
}


SBW_STATE_CODE sbw_parse_section(uint8_t* read_p, uint8_t* write_p, uint32_t* read_len_p, uint32_t* write_len_p)
{
	SBW_STATE_CODE ret;
	uint32_t total_read_len = 0, total_write_len = 0, r_len = 0, w_len = 0;

	do {
		ret = sbw_parse_line(read_p, write_p, &r_len, &w_len);
		// end-of-section; break out of loop
		if (SBW_EOS == ret){
			total_read_len += r_len;
			*read_len_p = total_read_len;
			total_write_len += w_len;
			*write_len_p = total_write_len;
		}
		// end-of-line; keep looping
		else if (SBW_EOL == ret) {
			total_read_len += r_len;
			read_p += r_len;
			total_write_len += w_len;
			write_p += w_len;
		}
		// state mismatch
		else {
			printk("SBW_ERROR[sbw_parse_section]: wrong state returned; state:  %d\n", ret);
			ret =  SBW_STATE_ERROR;
		}
	} while (SBW_EOL == ret);


	return ret;
}

typedef struct {
	uint32_t sec_addr[75];
	uint32_t sec_len[75];
	uint32_t num_sections;
} sec_info_struct;

sec_info_struct sec_info;
int32_t sec_index = 0;


SBW_STATE_CODE sbw_parse_image(uint8_t* read_p, uint8_t* write_p, uint32_t* read_len_p, uint32_t* write_len_p)
{
	SBW_STATE_CODE ret;
	uint32_t total_read_len = 0, total_write_len = 0, r_len = 0, w_len = 0, val = 0;

	memset(&sec_info, 0, sizeof(sec_info));


	do {
		ret = sbw_get_token(read_p, (uint8_t*)&val, &r_len, &w_len);
		if (SBW_SOS != ret) {
			if (!sec_info.num_sections) {
				printk("SBW_ERROR[sbw_parse_image]: does not start with section; value:  %d\n", ret);
				return SBW_STATE_ERROR;
			}

			if (SBW_EOI == ret) {
				*read_len_p = total_read_len;
				*write_len_p = total_write_len;
				ret = SBW_OK;
				break;
			}
			else {
				printk("SBW_ERROR[sbw_parse_image]: wrong state returned; state:  %d\n", ret);
				ret =  SBW_STATE_ERROR;
				break;
			}
		}

		//printk("[Status]: SOS detected; address: %x, r_len: %d, w_len: %d\n", val, r_len, w_len);

		total_read_len += r_len;
		read_p += r_len;

		sec_info.sec_addr[sec_info.num_sections] = val;

		ret = sbw_parse_section(read_p, write_p, &r_len, &w_len);
		// end-of-section; keep looping
		if (SBW_EOS == ret) {
			total_read_len += r_len;
			read_p += r_len;

			if (w_len & 1) {
				write_p[w_len] = 0xff;
				w_len++;
			}
			total_write_len += w_len;
			write_p += w_len;

			// sec_len converted to A6 words (16-bit)
			sec_info.sec_len[sec_info.num_sections] = w_len/2;
			sec_info.num_sections++;
		}
		// state mismatch
		else {
			printk("SBW_ERROR[sbw_parse_image:1]: wrong state returned; state:  %d\n", ret);
			ret =  SBW_STATE_ERROR;
		}
	} while (SBW_EOS == ret);

	if (SBW_OK == ret) {
		int idx = 0;

		printk("Parsing complete. Read size: %d, Write size: %d. Num sections: %d\n",
		       *read_len_p, *write_len_p, sec_info.num_sections);
		while (idx < (int)sec_info.num_sections) {
			printk("Section idx: %d; Addr: 0x%04x; Length: %d\n",
			       idx, sec_info.sec_addr[idx], sec_info.sec_len[idx]);
			idx++;
		}

/*
		printk("\nDumping converted data:\n");
		for (idx = 0; idx < (int)*write_len_p; idx++) {
			if (!(idx % 16)) {
				printk("\n");
			}

			printk("%02x ", (write_p-*write_len_p)[idx]);
		}
*/
	}


	return ret;
}

int program_device_sbw(struct a6_sbw_interface* sbw_ops, uint32_t read_address)
{
	uint32_t read_len = 0, write_len = 0;
	SBW_STATE_CODE parse_ret;
	int retry = 0, ret_val = 0;
	uint16_t addr;


	if (read_address & 1) {
		printk("program_fw: Please enter an even read address.\n");
		return -1;
	}

	// set up the current mappings for the sbw code...
	SetSBWTCK = sbw_ops->a6_per_device_interface.SetSBWTCK;
	ClrSBWTCK = sbw_ops->a6_per_device_interface.ClrSBWTCK;
	SetSBWTDIO = sbw_ops->a6_per_device_interface.SetSBWTDIO;
	ClrSBWTDIO = sbw_ops->a6_per_device_interface.ClrSBWTDIO;
	SetInSBWTDIO = sbw_ops->a6_per_device_interface.SetInSBWTDIO;
	SetOutSBWTDIO = sbw_ops->a6_per_device_interface.SetOutSBWTDIO;
	GetSBWTDIO = sbw_ops->a6_per_device_interface.GetSBWTDIO;
	SetSBWAKEUP = sbw_ops->a6_per_device_interface.SetSBWAKEUP;
	ClrSBWAKEUP = sbw_ops->a6_per_device_interface.ClrSBWAKEUP;
	delay = sbw_ops->a6_per_target_interface.delay;

	parse_ret = sbw_parse_image((uint8_t*)read_address, (uint8_t*)read_address/*write_p*/, &read_len, &write_len);
	if (SBW_OK != parse_ret) {
		printk("Error in parsing A6 fw file...\n");
		return -1;
	}

/* TEMP: Workaround for occasional verification failure. Not root-Caused yet but,
   empirically, a retry always works. Revisit.*/
retry_0:

	InitTarget();

	// Start of SBW access to the Target
	if (GetDevice() != STATUS_OK)         // Set DeviceId
	{
		printk("Error in GetDevice()\n");      // stop here if invalid JTAG ID or
	                                               // time-out. (error: red LED is ON)
		ret_val = -1;
		goto err0;
	}


	// Program the boot code
	if (!WriteAllSections((const unsigned short*)read_address, (const unsigned long *)&sec_info.sec_addr[0],
			      (const unsigned long *)&sec_info.sec_len[0], sec_info.num_sections))
	{
		printk("Error in WriteAllSections(all)\n");
		ret_val = -1;
		goto err0;
	}


	if (!VerifyAllSections((const unsigned short*)read_address, (const unsigned long *)&sec_info.sec_addr[0],
			       (const unsigned long *)&sec_info.sec_len[0], sec_info.num_sections))
	{
		printk("Error in VerifyAllSections(all)\n");
		printk("Retrying...\n\n");
		if (retry++ < 15) {
			addr = ReadMem_430Xv2(F_WORD, V_RESET);
			ReleaseDevice(addr, ERROR);               // set PC to V_RESET contents
			ReleaseTarget();
			goto retry_0;
		}
		else {
			printk("Failure to write and verify fw file after %d retries\n", retry);
			ret_val = -1;
		}
	}


err0:

	addr = ReadMem_430Xv2(F_WORD, V_RESET);
	if (ReleaseDevice(addr, PROGRAM) < 0) {               // set PC to V_RESET contents
		printk(KERN_ERR "Checksum validation failed post-flashing.\n");
		if (retry < 15) {
			printk(KERN_ERR "Retrying...\n\n");
			retry++;
			ReleaseTarget();
			goto retry_0;
		}
		else {
			printk(KERN_ERR "Failure to program fw after %d retries.\n", retry);
			ret_val = -1;
		}
	}

	// if fail to set JTAG mode
	if (ret_val == -1 && retry == 0 ) {
		retry++;
		ret_val = 0;
		goto  retry_0;
	}

	ReleaseTarget();
	return ret_val;
}


int verify_device_sbw(struct a6_sbw_interface* sbw_ops, uint32_t read_address)
{
	uint32_t read_len = 0, write_len = 0;
	SBW_STATE_CODE parse_ret;
	int ret_val = 0;
	uint16_t addr;


	if (read_address & 1) {
		printk("program_fw: Please enter an even read address.\n");
		return -1;
	}

	// set up the current mappings for the sbw code...
	SetSBWTCK = sbw_ops->a6_per_device_interface.SetSBWTCK;
	ClrSBWTCK = sbw_ops->a6_per_device_interface.ClrSBWTCK;
	SetSBWTDIO = sbw_ops->a6_per_device_interface.SetSBWTDIO;
	ClrSBWTDIO = sbw_ops->a6_per_device_interface.ClrSBWTDIO;
	SetInSBWTDIO = sbw_ops->a6_per_device_interface.SetInSBWTDIO;
	SetOutSBWTDIO = sbw_ops->a6_per_device_interface.SetOutSBWTDIO;
	GetSBWTDIO = sbw_ops->a6_per_device_interface.GetSBWTDIO;
	SetSBWAKEUP = sbw_ops->a6_per_device_interface.SetSBWAKEUP;
	ClrSBWAKEUP = sbw_ops->a6_per_device_interface.ClrSBWAKEUP;
	delay = sbw_ops->a6_per_target_interface.delay;

	parse_ret = sbw_parse_image( (uint8_t*)read_address,
				     (uint8_t*)read_address/*write_p*/,
				      &read_len, &write_len);
	if (SBW_OK != parse_ret) {
		printk("Error in parsing A6 fw file...\n");
		return -1;
	}

	InitTarget();

	// Start of SBW access to the Target
	if (GetDevice() != STATUS_OK)         // Set DeviceId
	{
		printk("Error in GetDevice()\n");      // stop here if invalid JTAG ID or
	                                               // time-out. (error: red LED is ON)
		ret_val = -1;
		goto err0;
	}

	if (!VerifyAllSections((const unsigned short*)read_address,
	     (const unsigned long *)&sec_info.sec_addr[0],
	     (const unsigned long *)&sec_info.sec_len[0], sec_info.num_sections)) {
		printk("Error in VerifyAllSections(all)\n");
		ret_val = -1;
	}


err0:
	addr = ReadMem_430Xv2(F_WORD, V_RESET);
	ReleaseDevice(addr, VERIFY);  // set PC to V_RESET contents
	ReleaseTarget();

	return ret_val;
}

int ttf_extract_fw_sbw(struct a6_sbw_interface* sbw_ops)
{
	int ret_val = 0;
	uint16_t addr;


	// set up the current mappings for the sbw code...
	SetSBWTCK = sbw_ops->a6_per_device_interface.SetSBWTCK;
	ClrSBWTCK = sbw_ops->a6_per_device_interface.ClrSBWTCK;
	SetSBWTDIO = sbw_ops->a6_per_device_interface.SetSBWTDIO;
	ClrSBWTDIO = sbw_ops->a6_per_device_interface.ClrSBWTDIO;
	SetInSBWTDIO = sbw_ops->a6_per_device_interface.SetInSBWTDIO;
	SetOutSBWTDIO = sbw_ops->a6_per_device_interface.SetOutSBWTDIO;
	GetSBWTDIO = sbw_ops->a6_per_device_interface.GetSBWTDIO;
	SetSBWAKEUP = sbw_ops->a6_per_device_interface.SetSBWAKEUP;
	ClrSBWAKEUP = sbw_ops->a6_per_device_interface.ClrSBWAKEUP;
	delay = sbw_ops->a6_per_target_interface.delay;

	InitTarget();

	// Start of SBW access to the Target
	if (GetDevice() != STATUS_OK)         // Set DeviceId
	{
		printk("Error in GetDevice()\n");      // stop here if invalid JTAG ID or
	                                               // time-out. (error: red LED is ON)
		ret_val = -1;
		goto err0;
	}

	if (!TTFExtractAllSections()) {
		printk("Error in TTFExtractAllSections\n");
		ret_val = -1;
	}


err0:

	addr = ReadMem_430Xv2(F_WORD, V_RESET);
	ReleaseDevice(addr, VERIFY);  // set PC to V_RESET contents
	ReleaseTarget();
	return ret_val;
}

int ttf_image_read(char *buf, size_t count, loff_t *ppos)
{
	return TTFImageRead(buf, count, ppos);
}

int ttf_extract_cache_clear(void)
{
	TTFExtractCacheClear();
	return 0;
}

int get_checksum_data_sbw(struct a6_sbw_interface* sbw_ops, unsigned short* cksum1,
			  unsigned short* cksum2, unsigned short* cksum_cycles,
			  unsigned short* cksum_errors)
{
	// set up the current mappings for the sbw code...
	SetSBWTCK = sbw_ops->a6_per_device_interface.SetSBWTCK;
	ClrSBWTCK = sbw_ops->a6_per_device_interface.ClrSBWTCK;
	SetSBWTDIO = sbw_ops->a6_per_device_interface.SetSBWTDIO;
	ClrSBWTDIO = sbw_ops->a6_per_device_interface.ClrSBWTDIO;
	SetInSBWTDIO = sbw_ops->a6_per_device_interface.SetInSBWTDIO;
	SetOutSBWTDIO = sbw_ops->a6_per_device_interface.SetOutSBWTDIO;
	GetSBWTDIO = sbw_ops->a6_per_device_interface.GetSBWTDIO;
	SetSBWAKEUP = sbw_ops->a6_per_device_interface.SetSBWAKEUP;
	ClrSBWAKEUP = sbw_ops->a6_per_device_interface.ClrSBWAKEUP;
	delay = sbw_ops->a6_per_target_interface.delay;

	return GetChecksumData(cksum1, cksum2, cksum_cycles, cksum_errors);
}
