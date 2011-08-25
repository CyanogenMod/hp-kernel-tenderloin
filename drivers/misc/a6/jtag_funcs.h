#ifndef __BYTEWORD__
#define __BYTEWORD__
typedef unsigned int  word;
typedef unsigned char byte;
#endif

/****************************************************************************/
/* Define section for constants                                             */
/****************************************************************************/

// Constants for the JTAG instruction register (IR, requires LSB first).
// The MSB has been interchanged with LSB due to use of the same shifting
// function as used for the JTAG data register (DR, requires MSB first).

// Instructions for the JTAG control signal register
#define IR_CNTRL_SIG_16BIT         0xC8   // 0x13 original values
#define IR_CNTRL_SIG_CAPTURE       0x28   // 0x14
#define IR_CNTRL_SIG_RELEASE       0xA8   // 0x15
// Instructions for the JTAG Fuse
#define IR_PREPARE_BLOW            0x44   // 0x22
#define IR_EX_BLOW                 0x24   // 0x24
// Instructions for the JTAG data register
#define IR_DATA_16BIT              0x82   // 0x41
#define IR_DATA_QUICK              0xC2   // 0x43
// Instructions for the JTAG PSA mode
#define IR_DATA_PSA                0x22   // 0x44
#define IR_SHIFT_OUT_PSA           0x62   // 0x46
// Instructions for the JTAG address register
#define IR_ADDR_16BIT              0xC1   // 0x83
#define IR_ADDR_CAPTURE            0x21   // 0x84
#define IR_DATA_TO_ADDR            0xA1   // 0x85
// Bypass instruction
#define IR_BYPASS                  0xFF   // 0xFF

// JTAG identification value for all existing Flash-based MSP430 devices
#define JTAG_ID                    0x89
#define JTAG_ID91                  0x91
// Jtag 17 
#define DEVICE_HAS_JTAG17          1         

// additional instructions for JTAG_ID91 architectures
#define IR_COREIP_ID               0xE8   // 0x17
#define IR_DEVICE_ID               0xE1   // 0x87
// Instructions for the JTAG mailbox
#define IR_JMB_EXCHANGE            0x86   // 0x61
#define IR_TEST_REG                0x54   // 0x2A

// Constants for JTAG mailbox data exchange
#define OUT1RDY 0x0008
#define IN0RDY  0x0001
#define JMB32B  0x0010
#define OUTREQ  0x0004
#define INREQ   0x0001

// Constants for data formats, dedicated addresses
#ifndef __DATAFORMATS__
#define __DATAFORMATS__
#define F_BYTE                     8
#define F_WORD                     16
#define F_ADDR                     20
#define F_LONG                     32
#endif
#define V_RESET                    0xFFFE
#define V_BOR                      0x1B08

// Constants for VPP connection at Blow-Fuse
#define VPP_ON_TDI                 0
#define VPP_ON_TEST                1

// ReleaseDevice parameters
#define INIT    0 // ReleaseDevice() status
#define ERROR   1 // inc verify error count
#define VERIFY  2 // inc verify pass count
#define PROGRAM 3 // inc reprogram count

/****************************************************************************/
/* Function prototypes                                                      */
/****************************************************************************/

// Low level JTAG functions
//static word DR_Shift16(word Data);
//static unsigned long DR_Shift20(unsigned long address);
//static word IR_Shift(byte Instruction);
//static void ResetTAP(void);
//static word ExecutePOR_430Xv2(void);
//static void SetPC_430Xv2(unsigned long Addr);
word VerifyPSA_430Xv2(unsigned long StartAddr, unsigned long Length, word *DataArray);

// High level JTAG functions
word GetDevice_430Xv2(void);
#define GetDevice GetDevice_430Xv2
int ReleaseDevice_430Xv2(unsigned long Addr, byte Stat);
#define ReleaseDevice ReleaseDevice_430Xv2
void WriteMem_430Xv2(word Format, unsigned long Addr, word Data);
#define WriteMem WriteMem_430Xv2
void WriteMemQuick_430Xv2(unsigned long StartAddr, unsigned long Length, word *DataArray);
#define WriteMemQuick WriteMemQuick_430Xv2
word WriteAllSections_430Xv2(const unsigned short *data, const unsigned long *address, const unsigned long *length_of_sections, const unsigned long sections);
#define WriteAllSections WriteAllSections_430Xv2
word VerifyAllSections_430Xv2(const unsigned short *data, const unsigned long *address, const unsigned long *length_of_sections, const unsigned long sections);
#define VerifyAllSections VerifyAllSections_430Xv2
word ReadMem_430Xv2(word Format, unsigned long Addr);
#define ReadMem ReadMem_430Xv2
void ReadMemQuick_430Xv2(unsigned long StartAddr, unsigned long Length, word *DataArray);
#define ReadMemQuick ReadMemQuick_430Xv2
word VerifyMem_430Xv2(unsigned long StartAddr, unsigned long Length, word *DataArray);
#define VerifyMem VerifyMem_430Xv2


typedef int (*extract_conv_fn)( const unsigned short inp_data,
	      unsigned char* op_data, unsigned int count);

word TTFExtractSection_430Xv2
	(const unsigned long sec_addr, const unsigned long sec_len,
	 unsigned char* sec_databuf, extract_conv_fn ttf_conv,
	 unsigned long* sec_fmt_len);
#define TTFExtractSection TTFExtractSection_430Xv2
word TTFExtractAllSections_430Xv2(void);
#define TTFExtractAllSections TTFExtractAllSections_430Xv2
word TTFExtractCacheClear_430Xv2(void);
#define TTFExtractCacheClear TTFExtractCacheClear_430Xv2
int TTFImageRead_430Xv2(char *buf, size_t count, loff_t *ppos);
#define TTFImageRead TTFImageRead_430Xv2
int GetChecksumData_430Xv2(unsigned short* cksum1, unsigned short* cksum2,
	unsigned short* cksum_cycles, unsigned short* cksum_errors);
#define GetChecksumData GetChecksumData_430Xv2