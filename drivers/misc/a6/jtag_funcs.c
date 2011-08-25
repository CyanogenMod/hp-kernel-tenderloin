#include "a6_host_adapter.h"
#include "low_level_funcs.h"
#include "jtag_funcs.h"

#define LOCAL_TRACE 1

// declarations for active implementation mappings used by the sbw code...
extern uint16_t (*SetSBWTCK)(void);
extern uint16_t (*ClrSBWTCK)(void);
extern uint16_t (*SetSBWTDIO)(void);
extern uint16_t (*ClrSBWTDIO)(void);
extern uint16_t (*SetInSBWTDIO)(void);
extern uint16_t (*SetOutSBWTDIO)(void);
extern uint16_t (*GetSBWTDIO)(void);
extern uint16_t (*SetSBWAKEUP)(void);
extern uint16_t (*ClrSBWAKEUP)(void);
extern void (*delay)(uint32_t delay_us);
//

/****************************************************************************/
/* Low level routines for accessing the target device via JTAG:             */
/****************************************************************************/

#define VCC_LEVEL  36
static int8_t SetTargetVcc(int8_t level) {

    return level;
}


//----------------------------------------------------------------------------
/* Function for shifting a given 16-bit word into the JTAG data register
   through TDI.
   Arguments: word data (16-bit data, MSB first)
   Result:    word (value is shifted out via TDO simultaneously)
*/
static word DR_Shift16(word data)
{
    // JTAG FSM state = Run-Test/Idle
    if (TCLK_saved)
    {
        TMSH_TDIH();
    }
    else
    {
        TMSH_TDIL();
    }
    // JTAG FSM state = Select DR-Scan
    TMSL_TDIH();
    // JTAG FSM state = Capture-DR
    TMSL_TDIH();
    // JTAG FSM state = Shift-DR, Shift in TDI (16-bit)
    return(AllShifts(F_WORD, data));
    // JTAG FSM state = Run-Test/Idle
}

//----------------------------------------------------------------------------
/* Function for shifting a given 20-bit address word into the
   JTAG address register through TDI.
   Arguments: unsigned long address (20-bit address word, MSB first)
   Result:    unsigned long TDOvalue (is shifted out via TDO simultaneously)
*/
static unsigned long DR_Shift20(unsigned long address)
{
    // JTAG FSM state = Run-Test/Idle
    if (TCLK_saved)
    {
        TMSH_TDIH();
    }
    else
    {
        TMSH_TDIL();
    }
    // JTAG FSM state = Select DR-Scan
    TMSL_TDIH();
    // JTAG FSM state = Capture-DR
    TMSL_TDIH();
    // JTAG FSM state = Shift-DR, Shift in TDI (16-bit)
    return(AllShifts(F_ADDR, address));
    // JTAG FSM state = Run-Test/Idle
}

//----------------------------------------------------------------------------
/* Function for shifting a new instruction into the JTAG instruction
   register through TDI (MSB first, but with interchanged MSB - LSB, to
   simply use the same shifting function, Shift(), as used in DR_Shift16).
   Arguments: byte Instruction (8bit JTAG instruction, MSB first)
   Result:    word TDOword (value shifted out from TDO = JTAG ID)
*/
static word IR_Shift(byte instruction)
{
    // JTAG FSM state = Run-Test/Idle
    if (TCLK_saved)
    {
        TMSH_TDIH();
    }
    else
    {
        TMSH_TDIL();
    }
    // JTAG FSM state = Select DR-Scan
    TMSH_TDIH();

    // JTAG FSM state = Select IR-Scan
    TMSL_TDIH();
    // JTAG FSM state = Capture-IR
    TMSL_TDIH();
    // JTAG FSM state = Shift-IR, Shift in TDI (8-bit)
    return(AllShifts(F_BYTE, instruction));
    // JTAG FSM state = Run-Test/Idle
}

//----------------------------------------------------------------------------
/* Reset target JTAG interface and perform fuse-HW check.
   Arguments: None
   Result:    None
*/
static void ResetTAP(void)
{
    word i;

    // Now fuse is checked, Reset JTAG FSM
    for (i = 6; i > 0; i--)      // 6 is nominal
    {
        TMSH_TDIH();
    }
    // JTAG FSM is now in Test-Logic-Reset
    TMSL_TDIH();                 // now in Run/Test Idle
}

//----------------------------------------------------------------------------
/* Function to execute a Power-On Reset (POR) using JTAG CNTRL SIG register
   Arguments: None
   Result:    word (STATUS_OK if target is in Full-Emulation-State afterwards,
                    STATUS_ERROR otherwise)
*/
static word ExecutePOR_430Xv2(void)
{
  word i = 0;

  // provide one clock
  ClrTCLK();
  SetTCLK();

  // prepare access to the JTAG CNTRL SIG register  
  IR_Shift(IR_CNTRL_SIG_16BIT);
  // release CPUSUSP signal and apply POR signal
  DR_Shift16(0x0C01);
  // release POR signal again
  DR_Shift16(0x0401);
  
  // provide 5 clock cycles
  for (i = 0; i < 5; i++)
  {
    ClrTCLK();
    SetTCLK();
  }
  // now set CPUSUSP signal again
  DR_Shift16(0x0501);
  // and provide one more clock
  ClrTCLK();
  SetTCLK();
  // the CPU is now in 'Full-Emulation-State'
  
  // disable Watchdog Timer on target device now by setting the HOLD signal
  // in the WDT_CNTRL register
  WriteMem_430Xv2(F_WORD, 0x015C, 0x5A80);

  // Check if device is again in Full-Emulation-State and return status
  IR_Shift(IR_CNTRL_SIG_CAPTURE);
  if(DR_Shift16(0) & 0x0301)
  {
    return(STATUS_OK);
  }
  
  return(STATUS_ERROR);
}

//----------------------------------------------------------------------------
/* Load a given address into the target CPU's program counter (PC).
   Argument: unsigned long Addr (destination address)
   Result:   None
*/
static void SetPC_430Xv2(unsigned long Addr)
{
  unsigned short Mova;
  unsigned short Pc_l;
  
  Mova  = 0x0080;
  Mova += (unsigned short)((Addr>>8) & 0x00000F00);
  Pc_l  = (unsigned short)((Addr & 0xFFFF));
  
  // Check Full-Emulation-State at the beginning
  IR_Shift(IR_CNTRL_SIG_CAPTURE);
  if(DR_Shift16(0) & 0x0301)
  {
    // MOVA #imm20, PC
    ClrTCLK();
    // take over bus control during clock LOW phase
    IR_Shift(IR_DATA_16BIT);
    SetTCLK();
    DR_Shift16(Mova);
    IR_Shift(IR_CNTRL_SIG_16BIT);
    DR_Shift16(0x1400);
    IR_Shift(IR_DATA_16BIT);
    ClrTCLK();
    SetTCLK();
    DR_Shift16(Pc_l);
    ClrTCLK();
    SetTCLK();
    DR_Shift16(0x4303);    
    ClrTCLK();
    IR_Shift(IR_ADDR_CAPTURE);    
    DR_Shift20(0x00000);
    SetTCLK();
  }
}

/****************************************************************************/
/* High level routines for accessing the target device via JTAG:            */
/*                                                                          */
/* From the following, the user is relieved from coding anything.           */
/* To provide better understanding and clearness, some functionality is     */
/* coded generously. (Code and speed optimization enhancements may          */
/* be desired)                                                              */
/****************************************************************************/

#define MAX_ENTRY_TRY 4

static word JtagId = 0;
static word CoreId = 0;
static unsigned long DeviceIdPointer = 0;
static word DeviceId = 0;

static void ConnectJTAG(void)
{
	/* 8051-CODE */
	// drive JTAG/TEST signals
	{
		DrvSignals();

                //P2_7 = 0;
		ClrSBWAKEUP(); // prepare to close sbw isolation switch
		ClrSBWTCK();
		usDelay(10);

                //P2_7 = 1;
		SetSBWAKEUP();
		usDelay(4);
		SetSBWTCK();
		usDelay(4);

		//P2_7 = 0;
		ClrSBWAKEUP();
		usDelay(4);
		ClrSBWTCK();
		usDelay(4);     }  // <=== got left out of first email
}


static void StartJtag(void)
{

	/* 8051-CODE */
        // reset TEST logic, spybiwire jtag restart
	ClrSBWTCK();
	usDelay(100);

        // ensure A6 powered up
	SetSBWAKEUP();
	usDelay(100);

        // ensure Rst negated
	SetSBWTDIO();
	usDelay(100); 

        // PHASE 1 -> TEST PIN TO 1
	SetSBWTCK(); // prepare to activate TEST logic
	usDelay(100); 

        // PHASE 2 -> TEST PIN TO 0
	ClrSBWTCK(); // issue phantom spybiwire clock

        // PHASE 3
	usDelay(1); // low time < 7 uSec to enalbe mcu sbw jtag

        // phase 4 -> TEST PIN TO 1
	SetSBWTCK();
	usDelay(100);

        // phase 5
        // MsDelay(5);
}

static void StopJtag (void)
{
	/* 8051-CODE */
	// release JTAG/TEST signals
	{
		RlsSignals();
		//P2MDIN &= ~0x80;
		//P2MDOUT &= ~0x80;
		//P2_7 = 1;
		SetSBWAKEUP();
		MsDelay(1);
	}
}

static word GetCoreID (void)
{
  word i;
  for (i = 0; i < MAX_ENTRY_TRY; i++)
  {
    // initialize JtagId with an invalid value
    JtagId = 0;  
    // release JTAG/TEST signals to savely reset the test logic
    StopJtag();        
    // establish the physical connection to the JTAG interface
    ConnectJTAG();               
    // Apply again 4wire/SBW entry Sequence. 
    // set ResetPin =1    
      StartJtag();
    // reset TAP state machine -> Run-Test/Idle
    ResetTAP();  
    // shift out JTAG ID
    JtagId = (word)IR_Shift(IR_CNTRL_SIG_CAPTURE);  
     
    //printk("JTAG ID returned: %x; expected: %x\n", JtagId, JTAG_ID91);
    // break if a valid JTAG ID is being returned
    if(JtagId == JTAG_ID91) 
      break;
    // 
    SetTargetVcc( 0 );
    MsDelay(200);
    SetTargetVcc( VCC_LEVEL );
    ConnectJTAG(); 
      StartJtag();
    ResetTAP(); 

    JtagId = (word)IR_Shift(IR_CNTRL_SIG_CAPTURE);
    //printk("(2) JTAG ID returned: %x; expected: %x\n", JtagId, JTAG_ID91);
    if(JtagId == JTAG_ID91)
      break;
  }
  if(i >= MAX_ENTRY_TRY)
  {         
    // if connected device is MSP4305438 JTAG Mailbox is not usable
    #ifdef ACTIVATE_MAGIC_PATTERN
    /* xxx for(i = 0; i < MAX_ENTRY_TRY; i++)
    {
        // if no JTAG ID is  beeing returnd -> apply magic pattern to stop user cd excecution     
      if((JtagId = magicPattern()) == 1 || i >= MAX_ENTRY_TRY)
      {
          // if magic pattern faild and 4 tries passed -> return status error
          return(STATUS_ERROR);
      }
      else
      {
          break; 
      }
    }*/
    // is MSP4305438 mailbox is not usalbe
    #else
       return(STATUS_ERROR);
    #endif   
  }
  if(JtagId == JTAG_ID91)
  {
    // Get Core identification info
    IR_Shift(IR_COREIP_ID);
    CoreId = DR_Shift16(0);
    //printk("CoreId returned: %x\n", CoreId);
    if(CoreId == 0)
    {
      return(STATUS_ERROR);
    }
    IR_Shift(IR_DEVICE_ID);
    DeviceIdPointer = DR_Shift20(0);
    // The ID pointer is an un-scrambled 20bit value
    DeviceIdPointer = ((DeviceIdPointer & 0xFFFF) << 4 )  + (DeviceIdPointer >> 16 );
    //printk("DeviceIdPointer returned: %lx\n", DeviceIdPointer);
    
    return(STATUS_OK);
  }
  else
  {
    return(STATUS_ERROR);
  }
}

static word SyncJtag_AssertPor (void)
{
  word i = 0;
  word jid = 0;

  IR_Shift(IR_CNTRL_SIG_16BIT);
  DR_Shift16(0x1501);                  // Set device into JTAG mode + read
  jid = IR_Shift(IR_CNTRL_SIG_CAPTURE);
  if (jid != JTAG_ID91)
  {
    printk("%s: Failed JTAGID verification. Expected: %x; ret: %x\n", __func__, JTAG_ID91, jid);
    return(STATUS_ERROR);
  }
  // wait for sync
  while(!(DR_Shift16(0) & 0x0200) && i < 50)
  {
    i++;
  };
  // continues if sync was sucessfull
  if(i >= 50)
  {
    return(STATUS_ERROR);
  }

  // execute a Power-On-Reset
  if(ExecutePOR_430Xv2() != STATUS_OK)
  {
    printk("%s: Failed ExecutePOR_430Xv2.\n", __func__);
    return(STATUS_ERROR);
  }
  
  return(STATUS_OK);
}
//----------------------------------------------------------------------------
/* Function to clear and confirm watchdog disabled.
   Result:    word (STATUS_ERROR if unable to confirm VMON_MODE
                    bit 7 has been cleared)
*/

byte pmicWriteRead(byte addr, byte wdat)
{
  byte rdat;

  // PB(DIR|OUT)  (bit 7 = sel (adr), 4 = rw (wrt),  bit 0 = en)

  // spg430 control (port 3) control
  WriteMem_430Xv2(F_BYTE, 0x222, 0x90 ); // set adr/wrt, clear en
  WriteMem_430Xv2(F_BYTE, 0x224, 0x91 ); // enable control outputs

//	bic.b	#0x01, &PBOUT_L		; clear en	(0x222)
//	or.b	#0x90, &PBOUT_L		; set rw, sel	(0x222)
//	or.b	#0x91, &PBDIR_L		; enable ctl	(0x224)

  // spg430 data (port 2) output
  WriteMem_430Xv2(F_BYTE, 0x205, 0xFF ); // enable data outputs
  //              F_WORD, 0x204, 0xFF00

//	mov.b	#0xff, &PADIR_H		; enable data	(0x205)

  WriteMem_430Xv2(F_BYTE, 0x203, addr ); // select vmon mode reg
  //              F_WORD, 0x202, 0x7E00
  WriteMem_430Xv2(F_BYTE, 0x222, 0x91 ); // set en
  WriteMem_430Xv2(F_BYTE, 0x222, 0x90 ); // clear en

//	mov.b	#PMIC_VMON_MODE, &PAOUT_H	;	(0x203)
//	or.b	#0x01, &PBOUT_L		; set en	(0x222)
//	nop
//	bic.b	#0x01, &PBOUT_L		; clear en	(0x222)

  WriteMem_430Xv2(F_BYTE, 0x203, wdat ); // set write data
  //              F_WORD, 0x202, 0x0200
  WriteMem_430Xv2(F_BYTE, 0x222, 0x10 ); // clear sel
  WriteMem_430Xv2(F_BYTE, 0x222, 0x11 ); // set en
  WriteMem_430Xv2(F_BYTE, 0x222, 0x10 ); // clear en

//	mov.b	#PMIC_SBW_EN, &PAOUT_H		;	(0x203)
//	bic.b	#0x80, &PBOUT_L		; clear sel	(0x222)
//	or.b	#0x01, &PBOUT_L		; set en	(0x222)
//	nop
//	bic.b	#0x01, &PBOUT_L		; clear en	(0x222)

  // spg430 data (port 2) input
  WriteMem_430Xv2(F_BYTE, 0x205, 0x00 ); // disable data outputs
  //              F_WORD, 0x204, 0x0000

//	mov.b	#0x00, &PADIR_H		; disable data	(0x205)

  WriteMem_430Xv2(F_BYTE, 0x222, 0x00 ); // clear sel,rw
  WriteMem_430Xv2(F_BYTE, 0x222, 0x01 ); // set en
  rdat = (byte) ReadMem_430Xv2(F_BYTE, 0x201); // readback vmon mode reg
  //                         F_WORD, 0x200
  WriteMem_430Xv2(F_BYTE, 0x222, 0x00 ); // clear en

//	bic.b	#0x00, &PBOUT_L		; clear sel,rw 	(0x222)
//	or.b	#0x01, &PBOUT_L		; set en	(0x222)
//	nop
//	nop
//	nop
//	mov.b	&PAOUT_H, DATA		; read data	(0x201)
//	bic.b	#0x01, &PBOUT_L		; clear en	(0x222)

  return rdat;
}

signed char ClearWatchDogEnable(void)
{

   if (pmicWriteRead(0x7e,0x02) !=  0x02) // vmon_mode reg
       return(STATUS_ERROR);

   return(STATUS_OK); // wdt clear, sbw set
}


//----------------------------------------------------------------------------
/* Function to take target device under JTAG control. Disables the target
   watchdog. Sets the global DEVICE variable as read from the target device.
   Arguments: None
   Result:    word (STATUS_ERROR if fuse is blown, incorrect JTAG ID or
                    synchronizing time-out; STATUS_OK otherwise)
*/
word GetDevice_430Xv2(void)
{
  uint32_t t1_val, t2_val, t3_val, t4_val;
  int32_t i;
  word wd_stat;
  unsigned long flags = 0;

  i = 20;
  DisableInterrupts(flags);
  do {
	t1_val = hres_get_counter();
	if(GetCoreID () != STATUS_OK) {
		printk("GetCoreID failed.\n");
	}

	t2_val = hres_get_counter();
	if(SyncJtag_AssertPor() != STATUS_OK) {
		printk("SyncJtag_AssertPor failed.\n");
	}

	t3_val = hres_get_counter();
	wd_stat = ClearWatchDogEnable();
	t4_val = hres_get_counter();

	// disable write protection...
	WriteMem_430Xv2(F_WORD, 0x120, 0xa500 );
	WriteMem_430Xv2(F_WORD, 0x122, 0x0000 );

	pmicWriteRead(0x2c,0x00); // 1-wire mode_status: clr DQ

	printk("T1 %lu T2 %lu T3 %lu Total %lu\n",
	       hres_get_delta_usec(t1_val, t2_val)/USEC_PER_MSEC,
	       hres_get_delta_usec(t2_val, t3_val)/USEC_PER_MSEC,
	       hres_get_delta_usec(t3_val, t4_val)/USEC_PER_MSEC,
	       hres_get_delta_usec(t1_val, t4_val)/USEC_PER_MSEC);
	t1_val = t2_val = t3_val = t4_val = 0;
	if ( wd_stat == STATUS_OK ) {
		break;
	}
	else {
		printk("ClearWatchDogEnable failed.\n");
	}
  } while(--i);

  EnableInterrupts(flags);

  if (i == 0) return(STATUS_ERROR);

  // CPU is now in Full-Emulation-State

  // TEMP-HACK: CPU now under JTAG control and will transition to sleep; release WAKEUP...
  //ClrSBWAKEUP();

  // read DeviceId from memory
  ReadMemQuick_430Xv2(DeviceIdPointer + 4, 1, (word*)&DeviceId);
    
  return(STATUS_OK);
}


//----------------------------------------------------------------------------
/* Function to release the target device from JTAG control
   Argument: word Addr (0xFFFE: Perform Reset, means Load Reset Vector into PC,
                        otherwise: Load Addr into PC)
   Result:   None
*/
int ReleaseDevice_430Xv2(unsigned long Addr, byte Stat)
{
	word i,prev,check,count;
	word error_count = 0, cksum_cycle_count = 0;
	int ret = 0;

	i = 1;
	do {
		ClrSBWAKEUP(); // to permit A6 firmware to clear sbw_en

		switch(Addr)
		{
		case V_BOR: // no longer approved for use
			//// perform a BOR via JTAG - we loose control of the device then...
			//IR_Shift(IR_TEST_REG);
			//DR_Shift16(0x0200);
			//MsDelay(5);     // wait some time before doing any other action
			// JTAG control is lost now - GetDevice() needs to be called again to gain control.
			break;
		case V_RESET:
			IR_Shift(IR_CNTRL_SIG_16BIT);
			DR_Shift16(0x0C01);                 // Perform a reset
			DR_Shift16(0x0401);
			IR_Shift(IR_CNTRL_SIG_RELEASE);
			break;
		default:
			SetPC_430Xv2(Addr);                 // Set target CPU's PC

			IR_Shift(IR_CNTRL_SIG_16BIT);
			DR_Shift16(0x0401);
			IR_Shift(IR_ADDR_CAPTURE);
			IR_Shift(IR_CNTRL_SIG_RELEASE);
		}

		ReleaseTarget();

		if (i) { // entry pass

			if (Addr == V_BOR) { // boot code
				MsDelay(1000);
			}
			else {            // mcu restart
				MsDelay(250);
			}

			GetDevice_430Xv2(); // halt to inquire value computed over code
			check = ReadMem_430Xv2(F_WORD, 0x1A1A); // current/1a1a
			prev = ReadMem_430Xv2(F_WORD, 0x1A1E); // previous/1a1e
			printk(KERN_ERR "A6 checksum validation: master: 0x%04x, current: 0x%04x\n", prev, check);

			if (check != prev) {
				ret = -1;
				printk(KERN_ERR "Error: A6 checksum validation failed!!\n");
			}
			// cheksum validated...
			error_count = ReadMem_430Xv2(F_WORD, 0x1a20); // read error count
			cksum_cycle_count = ReadMem_430Xv2(F_WORD, 0x1a22); // read cksum cycle count
			printk(KERN_ERR "A6 error_count: %d; cksum cycle count: %d\n", error_count, cksum_cycle_count);

			if (Stat == PROGRAM && 0 == ret) {
				count = ReadMem_430Xv2(F_WORD, 0xFFD0); // read prog cycle count
				printk(KERN_ERR "A6 program counter: %d\n", count);
				WriteMem_430Xv2(F_WORD, 0xFFD0, count+1); // increment write back
			}
		}

	} while (i--);

	return ret;
}

//----------------------------------------------------------------------------
/* This function writes one byte/word at a given address ( <0xA00)
   Arguments: word Format (F_BYTE or F_WORD)
              word Addr (Address of data to be written)
              word Data (shifted data)
   Result:    None
*/
void WriteMem_430Xv2(word Format, unsigned long Addr, word Data)
{
  // Check Init State at the beginning
  IR_Shift(IR_CNTRL_SIG_CAPTURE);
  if(DR_Shift16(0) & 0x0301)
  {
    ClrTCLK();
    IR_Shift(IR_CNTRL_SIG_16BIT);
    if  (Format == F_WORD)
    {
      DR_Shift16(0x0500);
    }
    else
    {
      DR_Shift16(0x0510);
    }
    IR_Shift(IR_ADDR_16BIT);
    DR_Shift20(Addr);
    
    SetTCLK();
    // New style: Only apply data during clock high phase
    IR_Shift(IR_DATA_TO_ADDR);
    DR_Shift16(Data);           // Shift in 16 bits
    ClrTCLK();
    IR_Shift(IR_CNTRL_SIG_16BIT);
    DR_Shift16(0x0501);
    SetTCLK();
    // one or more cycle, so CPU is driving correct MAB
    ClrTCLK();
    SetTCLK();
    // Processor is now again in Init State
  }
}

//----------------------------------------------------------------------------
/* This function writes an array of words into the target memory.
   Arguments: word StartAddr (Start address of target memory)
              word Length (Number of words to be programmed)
              word *DataArray (Pointer to array with the data)
   Result:    None
*/
void WriteMemQuick_430Xv2(unsigned long StartAddr, unsigned long Length, word *DataArray)
{
  unsigned long i;
  
  for (i = 0; i < Length; i++)
  {
    WriteMem_430Xv2(F_WORD, StartAddr, DataArray[i]);
    StartAddr += 2;
  }
}

//----------------------------------------------------------------------------
/* This function programs a set of data arrays of words into a Memeory
   by using the "WriteMemQuick()" function. It conforms with the
   "CodeArray" structure convention of file "Target_Code.s43" or "Target_Code.h".
   Arguments: 	const unsigned int  *DataArray (Pointer to array with the data)
		const unsigned long *address (Pointer to array with the startaddresses)
		const unsigned long *length_of_sections (Pointer to array with the number of words counting from startaddress)
		const unsigned long sections (Number of sections in code file)
   Result:      word (STATUS_OK if verification was successful,
                    STATUS_ERROR otherwise)
*/
word WriteAllSections_430Xv2(const unsigned short *data, const unsigned long *address, const unsigned long *length_of_sections, const unsigned long sections)
{
    int i, pos = 0;
    //time_t start_time;
    
    //start_time = current_time();

    printk("\n\n");
    for(i = 0; i < (int)sections; i++)
    {
	    printk("WriteMemQuick: addr: 0x%lx; length(d): %ld", address[i], length_of_sections[i]);
	    /*printk("; first 4 bytes: 0x%02x 0x%02x 0x%02x 0x%02x",
		   *(char*)&data[pos], *((char*)&data[pos]+1), *((char*)&data[pos]+2), *((char*)&data[pos]+3)); */
	    printk("\n");

	    WriteMemQuick(address[i],length_of_sections[i],(word*)&data[pos]);
	    pos+=length_of_sections[i];
	    yield();
    }

    //printk("[WriteAllSections] elapsed time: %d ms\n", current_time() - start_time);


    return(STATUS_OK);
}

//----------------------------------------------------------------------------
/* This function verifies a set of data arrays of words from Memeory
   by using the "VerifyMem()" function. It conforms with the
   "CodeArray" structure convention of file "Target_Code.s43" or "Target_Code.h".
   Arguments: 	const unsigned int  *DataArray (Pointer to array with the data)
		const unsigned long *address (Pointer to array with the startaddresses)
		const unsigned long *length_of_sections (Pointer to array with the number of words counting from startaddress)
		const unsigned long sections (Number of sections in code file)
   Result:      word (STATUS_OK if verification was successful,
                    STATUS_ERROR otherwise)
*/
word VerifyAllSections_430Xv2(const unsigned short *data, const unsigned long *address, const unsigned long *length_of_sections, const unsigned long sections)
{
    int i, pos = 0;
    word ret = STATUS_OK, latched_ret = STATUS_OK;

    printk("\n\n");
    for(i = 0; i < (int)sections; i++)
    {
	    printk("VerifyMem: addr: 0x%lx; length(d): %ld", address[i], length_of_sections[i]);
	    /*printk("; first 4 bytes: 0x%02x 0x%02x 0x%02x 0x%02x",
		    *(char*)&data[pos], *((char*)&data[pos]+1), *((char*)&data[pos]+2), *((char*)&data[pos]+3));*/
	    printk("\n");

	    ret = VerifyMem(address[i],length_of_sections[i],(word*)&data[pos]);
	    if (STATUS_ERROR == ret) {
		    latched_ret = STATUS_ERROR;
		    printk("VerifyAllSections: Failed for section addr: 0x%lx; length(d): %ld\n",
			   address[i], length_of_sections[i]);
	    }
	    yield();

	    pos+=length_of_sections[i];
    }    

    return(latched_ret);
}

word TTFExtractSection_430Xv2
(const unsigned long sec_addr, const unsigned long sec_len,
 unsigned char* sec_databuf, extract_conv_fn ttf_conv,
 unsigned long* sec_fmt_len)
{
	unsigned int idx;
	unsigned short data;
	int conv_bytes;
	unsigned char* w_buf = sec_databuf;

	SetPC_430Xv2(sec_addr);
	IR_Shift(IR_CNTRL_SIG_16BIT);
	DR_Shift16(0x0501);
	IR_Shift(IR_ADDR_CAPTURE);

	IR_Shift(IR_DATA_QUICK);

	for (idx = 0; idx < sec_len; idx++)
	{
		SetTCLK();
		ClrTCLK();
		data = DR_Shift16(0);  // Read data from memory.
		conv_bytes = ttf_conv(data, w_buf, idx);
		w_buf += conv_bytes;
	}

	if (idx % 8) {
		sprintf(w_buf, "\x0d\x0a");
		w_buf += 2;
	}

	IR_Shift(IR_CNTRL_SIG_CAPTURE);
	*sec_fmt_len = (unsigned long)(w_buf - sec_databuf);

	return STATUS_OK;
}

#define NUM_PMEM_SECTIONS  (4)
#define SECTION_PREFIX_LEN (7)

struct section_map_desc {
	unsigned long sec_addr[NUM_PMEM_SECTIONS];
	unsigned long sec_len[NUM_PMEM_SECTIONS];
	void* sec_databuf[NUM_PMEM_SECTIONS];
	unsigned long sec_fmt_len[NUM_PMEM_SECTIONS];
};

struct section_map_desc ttf_extract_smap =
{
	.sec_addr = {0x1400, 0x1800, 0x1c00, 0xc800},
	.sec_len =  {0x200,  0x200,  0x200,  0x1c00}, // double-byte length
};

int ttf_extract_conv_fn
	(const unsigned short inp_data, unsigned char* op_data, unsigned int count)
{
	int ret;
	
	ret = sprintf(op_data, "%02X %02X ",
		      (unsigned char)inp_data,
		      (unsigned char)(inp_data >> 8));
	if (0 == (count+1) % 8) {
		ret += sprintf(op_data + ret, "\x0d\x0a");
	}

	return ret;
}

word TTFExtractAllSections_430Xv2(void)
{
	int idx = 0, hdr_len;
	unsigned char* sec_buf;
	unsigned long sec_fmt_len = 0;

	for (idx = 0; idx < NUM_PMEM_SECTIONS; idx++) {
		/* allocate buffers */
		sec_buf = kzalloc(
			(ttf_extract_smap.sec_len[idx] * 6) +         // 4 digits + 2 spaces
			((ttf_extract_smap.sec_len[idx] / 8) * 2) +   // newline for row of 8
			0x14,					      // overhead
			GFP_KERNEL);
		ttf_extract_smap.sec_databuf[idx] = sec_buf;

		hdr_len = sprintf(sec_buf, "@%04hX\x0d\x0a",
				  (unsigned short)ttf_extract_smap.sec_addr[idx]);
		TTFExtractSection(
			ttf_extract_smap.sec_addr[idx],
			ttf_extract_smap.sec_len[idx],
			sec_buf + hdr_len,
			ttf_extract_conv_fn,
			&sec_fmt_len);
		ttf_extract_smap.sec_fmt_len[idx] = sec_fmt_len + SECTION_PREFIX_LEN;

		printk(KERN_ERR "Section Start (fmt len: %ld)\n", sec_fmt_len);
	}

	return STATUS_OK;
}

int TTFImageRead_430Xv2(char *buf, size_t count, loff_t *ppos)
{
	int idx, s_off, b_off = 0, ret;
	unsigned int accum_size = 0, copy_size = 0, rem_count = count;
	loff_t offset = *ppos;

	for (idx = 0; idx < NUM_PMEM_SECTIONS; idx++) {
		if (offset < (accum_size + ttf_extract_smap.sec_fmt_len[idx])) {
			s_off = offset - accum_size;
			copy_size = ((ttf_extract_smap.sec_fmt_len[idx] - s_off) >  rem_count) ?
					rem_count : (ttf_extract_smap.sec_fmt_len[idx] - s_off);
			ret = copy_to_user(buf + b_off,
					   ((char*)ttf_extract_smap.sec_databuf[idx]) + s_off,
					   copy_size);
			rem_count -= copy_size;
			offset += copy_size;
			b_off += copy_size;
		}

		if (!rem_count) break;
		accum_size += ttf_extract_smap.sec_fmt_len[idx];
	}

	*ppos = offset;
	return (count - rem_count);
}


word TTFExtractCacheClear_430Xv2(void)
{
	int idx = 0;

	for (idx = 0; idx < NUM_PMEM_SECTIONS; idx++) {
		/* free buffers */
		if (ttf_extract_smap.sec_databuf[idx]) {
			kfree(ttf_extract_smap.sec_databuf[idx]);
			ttf_extract_smap.sec_databuf[idx] = NULL;
			ttf_extract_smap.sec_fmt_len[idx] = 0;
		}
	}

	return STATUS_OK;
}


int GetChecksumData_430Xv2(unsigned short* cksum1, unsigned short* cksum2,
			   unsigned short* cksum_cycles, unsigned short* cksum_errors)
{
	word prev, check;
	word error_count = 0, cycle_count = 0;
	int ret = 0;
	word addr;

	InitTarget();
	// halt to inquire value computed over code
	GetDevice_430Xv2();

	check = ReadMem_430Xv2(F_WORD, 0x1A1A); // current/1a1a
	prev = ReadMem_430Xv2(F_WORD, 0x1A1E); // previous/1a1e
	error_count = ReadMem_430Xv2(F_WORD, 0x1a20); // read error count
	cycle_count = ReadMem_430Xv2(F_WORD, 0x1a22); // read cksum cycle count

	if (cksum1) *cksum1 = prev;
	if (cksum2) *cksum2 = check;
	if (cksum_cycles) *cksum_cycles = cycle_count;
	if (cksum_errors) *cksum_errors = error_count;

	// reset
	addr = ReadMem_430Xv2(F_WORD, V_RESET);
	SetPC_430Xv2(addr);                 // Set target CPU's PC
	IR_Shift(IR_CNTRL_SIG_16BIT);
	DR_Shift16(0x0401);
	IR_Shift(IR_ADDR_CAPTURE);
	IR_Shift(IR_CNTRL_SIG_RELEASE);

	MsDelay(1000);

	return ret;
}
//----------------------------------------------------------------------------
/* This function reads one byte/word from a given address in memory
   Arguments: word Format (F_BYTE or F_WORD)
              word Addr (address of memory)
   Result:    word (content of the addressed memory location)
*/
word ReadMem_430Xv2(word Format, unsigned long Addr)
{
  word TDOword = 0;
  
  // Check Init State at the beginning
  IR_Shift(IR_CNTRL_SIG_CAPTURE);
  if(DR_Shift16(0) & 0x0301)
  {
    // Read Memory
    ClrTCLK();
    IR_Shift(IR_CNTRL_SIG_16BIT);
    if  (Format == F_WORD)
    {
      DR_Shift16(0x0501);             // Set word read
    }
    else
    {
      DR_Shift16(0x0511);             // Set byte read
    }
    IR_Shift(IR_ADDR_16BIT);
    DR_Shift20(Addr);                   // Set address
    IR_Shift(IR_DATA_TO_ADDR);
    SetTCLK();
    ClrTCLK();
    TDOword = DR_Shift16(0x0000);       // Shift out 16 bits
    
    SetTCLK();
    // one or more cycle, so CPU is driving correct MAB
    ClrTCLK();
    SetTCLK();
    // Processor is now again in Init State
  }
  
  return TDOword;
}

//----------------------------------------------------------------------------
/* This function reads an array of words from a memory.
   Arguments: word StartAddr (Start address of memory to be read)
              word Length (Number of words to be read)
              word *DataArray (Pointer to array for the data)
   Result:    None
*/
void ReadMemQuick_430Xv2(unsigned long StartAddr, unsigned long Length, word *DataArray)
{
  unsigned long i;
  
  SetPC_430Xv2(StartAddr);
  IR_Shift(IR_CNTRL_SIG_16BIT);
  DR_Shift16(0x0501);
  IR_Shift(IR_ADDR_CAPTURE);
  
  IR_Shift(IR_DATA_QUICK);
  
  for (i = 0; i < Length; i++)
  {
    SetTCLK();
    ClrTCLK();
    *DataArray++   = DR_Shift16(0);  // Read data from memory.
  }
  IR_Shift(IR_CNTRL_SIG_CAPTURE);
}

//----------------------------------------------------------------------------
/* This function performs a Verification over the given memory range
   Arguments: word StartAddr (Start address of memory to be verified)
              word Length (Number of words to be verified)
              word *DataArray (Pointer to array with the data)
   Result:    word (STATUS_OK if verification was successful, STATUS_ERROR otherwise)
*/
word VerifyMem_430Xv2(unsigned long StartAddr, unsigned long Length, word *DataArray)
{
  unsigned long i;
  word Data;
  char err_flag = 0;
  
  SetPC_430Xv2(StartAddr);
  IR_Shift(IR_CNTRL_SIG_16BIT);
  DR_Shift16(0x0501);
  IR_Shift(IR_ADDR_CAPTURE);
  
  IR_Shift(IR_DATA_QUICK);

  for (i = 0; i < Length; i++)
  {
    SetTCLK();
    ClrTCLK();
    Data = DR_Shift16(0);  // Read data from memory.

    if(Data != DataArray[i])
    {
	    printk("VerifyMem failed. Idx; 0x%lx; Expected: 0x%04x; Read; 0x%04x\n", i, DataArray[i], Data);
      err_flag = 1;
      //break;
    }
  }
  IR_Shift(IR_CNTRL_SIG_CAPTURE);
  
  return((err_flag == 0) ? STATUS_OK : STATUS_ERROR);
}

/****************************************************************************/
/*                         END OF SOURCE FILE                               */
/****************************************************************************/
