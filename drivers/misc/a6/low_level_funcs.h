#ifndef _low_level_funcs_h_
#define _low_level_funcs_h_

/****************************************************************************/
/* Macros and Pin-to-Signal assignments which have to be programmed         */
/* by the user. This implementation assumes use of an MSP430F149 as the host*/
/* controller and the corresponding hardware given in the application       */
/* report TBD Appendix A.                                                   */
/*                                                                          */
/* The following MSP430 example acts as a hint of how to generally          */
/* implement a micro-controller programmer solution for the MSP430 flash-   */
/* based devices.                                                           */
/****************************************************************************/

#ifndef __BYTEWORD__
#define __BYTEWORD__
typedef unsigned int   word;
typedef unsigned char   byte;
#endif

//----------------------------------------------------------------------------
// Pin-to-Signal Assignments
//----------------------------------------------------------------------------

#define   TMSH    {SetSBWTDIO();   nNOPS ClrSBWTCK(); nNOPS                         SetSBWTCK();}// TMS = 1
#define   TMSL    {ClrSBWTDIO();   nNOPS ClrSBWTCK(); nNOPS                         SetSBWTCK();} // TMS = 0
#define   TMSLDH  {ClrSBWTDIO();   nNOPS ClrSBWTCK(); nNOPS SetSBWTDIO();           SetSBWTCK();} // TMS = 0, then TCLK(TDI) immediately = 1
#define   TDIH    {SetSBWTDIO();   nNOPS ClrSBWTCK(); nNOPS                         SetSBWTCK();} // TDI = 1
#define   TDIL    {ClrSBWTDIO();   nNOPS ClrSBWTCK(); nNOPS                         SetSBWTCK();} // TDI = 0
#define   TDOsbw  {SetSBWTDIO();SetInSBWTDIO(); nNOPS ClrSBWTCK(); nNOPS SetSBWTCK(); SetOutSBWTDIO();} // TDO cycle without reading TDO
#define   TDO_RD  {SetSBWTDIO();SetInSBWTDIO(); nNOPS ClrSBWTCK(); nNOPS tdo_bit = GetSBWTDIO(); SetSBWTCK(); SetOutSBWTDIO();} // TDO cycle with TDO read

 void ClrTCLK_sbw(void);
 void SetTCLK_sbw(void);
 #define ClrTCLK()       ClrTCLK_sbw()
 #define SetTCLK()       SetTCLK_sbw() 
 
 #define SetRST()	SetSBWTDIO()
 #define ClrRST()	ClrSBWTDIO()
 #define ReleaseRST()   () 
 #define SetTST()	SetSBWTCK()
 #define ClrTST()	ClrSBWTCK() 
 

/*----------------------------------------------------------------------------
   Definition of global variables
*/
extern byte TCLK_saved;      // holds the last value of TCLK before entering a JTAG sequence


/*----------------------------------------------------------------------------
   Low Level function prototypes
*/

void	TMSL_TDIL(void);
void	TMSH_TDIL(void);
void	TMSL_TDIH(void);
void	TMSH_TDIH(void);
void	TMSL_TDIH_TDOrd(void);
void	TMSL_TDIL_TDOrd(void);
void	TMSH_TDIH_TDOrd(void);
void	TMSH_TDIL_TDOrd(void);

unsigned long AllShifts(word Format, unsigned long Data);
void	DrvSignals(void);
void	RlsSignals(void);
void	InitTarget(void);
void	ReleaseTarget(void);


#endif
