#ifndef __HRES_COUNTER_INCLUDED__
#define __HRES_COUNTER_INCLUDED__

#include <linux/types.h>

struct hres_counter_platform_data {
	/* Initialize/obtain the timer resource */
	int (*init_hres_timer)(void **);

	/* Release the timer resource*/
	int (*release_hres_timer)(void *);

	/* PM functions */
	int (*suspend_hres_timer)(void *);
	int (*resume_hres_timer)(void *);

	/* Read native timer count value */
	u32 (*read_hres_timer)(void *);

	/* Convert native timer value to desired human */
	/* readable format (usec or msec, etc) */
	u32 (*convert_hres_timer)(u32);
};

#define LOG_MMC_TIMEOUT_TIMING_MEASUREMENTS 1
#if !defined(CONFIG_HRES_COUNTER) && LOG_MMC_TIMEOUT_TIMING_MEASUREMENTS
#error "MMC timeout measurements can only be done with hires counters"
#endif 

#ifdef CONFIG_HRES_COUNTER

extern u32   hres_get_counter ( void );
extern u32   hres_get_delta_usec ( u32 start, u32 end );
extern void  hres_ch_reset    ( uint ch ); 
extern void  hres_event_cnt   ( uint ch );
extern void  hres_event_start ( uint ch );
extern u32   hres_event_end   ( uint ch );
extern void  hres_event       ( char *type, u32 arg1, u32 arg2 );
extern int   hres_evlog_enable  ( void );
extern int   hres_evlog_disable ( void );
extern void  hres_evlog_print   ( void );
extern void  hres_evlog_reset   ( void );

#else

#define hres_get_counter(args...)
#define hres_get_delta_usec(args...)
#define hres_ch_reset(args...)
#define hres_event_cnt(args...)
#define hres_event_start(args...)
#define hres_event_end(args...)
#define hres_event(args...)
#define hres_evlog_enable(args...)
#define hres_evlog_disable(args...)
#define hres_evlog_print(args...)
#define hres_evlog_reset(args...)

#endif


#endif // __HRES_COUNTER_INCLUDED__



