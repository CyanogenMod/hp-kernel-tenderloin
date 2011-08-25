#ifndef A6_HOST_ADAPTER_H
#define A6_HOST_ADAPTER_H

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/hres_counter.h>

#include <linux/a6_sbw_interface.h>
#include <linux/a6.h>

#ifndef __BYTEWORD__
#define __BYTEWORD__
typedef unsigned short int   word;
typedef unsigned char   byte;
#endif

//---------------- Should be selected desired option ------------------------

#define ACTIVATE_MAGIC_PATTERN 1


//---------------------------------------------------------------------------

#ifndef __DATAFORMATS__
#define __DATAFORMATS__
#define F_BYTE                     8
#define F_WORD                     16
#define F_ADDR                     20
#define F_LONG                     32
#endif

// Constants for runoff status
#define STATUS_ERROR     0      // false
#define STATUS_OK        1      // true
#define STATUS_FUSEBLOWN 2      // GetDevice returns if the security fuse is blown

#define STATUS_ACTIVE    2
#define STATUS_IDLE      3

#define   nNOPS   {delay(1);}  //{ _NOP(); _NOP(); _NOP(); _NOP();  _NOP(); _NOP(); _NOP(); }


/********/
/* Host adapter for the sbw layer */
/********/
// per-target functions (separate implementation per target)
#define   DisableInterrupts(flags)   (a6_disable_interrupts(flags))
#define   EnableInterrupts(flags)    (a6_enable_interrupts(flags))

#define MsDelay(milliseconds) {delay(milliseconds * 1000);}      // millisecond delay loop
#define	usDelay(microseconds)  {delay(microseconds);}             // microsecond delay loop

#endif
