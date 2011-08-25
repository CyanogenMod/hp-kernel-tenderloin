#ifndef __QT_602240_H__
#define __QT_602240_H__

#include <linux/kernel.h>
#define QT_EXT  
#define __VER_1_4__
//*****************************************************************************
//
//
//		Common Defines
//
//
//*****************************************************************************
/*****************************************************************************
*
*   Build Options
*
* ***************************************************************************/
#define OPTION_WRITE_CONFIG     /* uncomment to force chip setup at startup */

#define QT_printf   pr_err
#define malloc(a) kmalloc(a, GFP_KERNEL)
#define free(a) kfree(a)
#define NUM_OF_TOUCH_OBJECTS   3

typedef enum 
{
    QT_PAGE_UP         = 0x01,
    QT_PAGE_DOWN       = 0x02,
    QT_DELTA_MODE      = 0x10,
    QT_REFERENCE_MODE  = 0x11,
    QT_CTE_MODE        = 0x31 
}diagnostic_debug_command;


typedef enum 
{
    QT_RESET   = 0x80,  /*!< The chip has reset */
    QT_OFL     = 0x40,  /*!< Overflow - the acquisition cycle length has overflowed the desired power mode interval
                     *(controlled by the IDLEACQINT and ACTVACQINT fields in the Power Configuration object) */
    QT_SIGERR  = 0x20,  /*!< There is an error in the acquisition  */
    QT_CAL     = 0x10,  /*!< The device is calibrating. */
    QT_CFGERR  = 0x08,  /*!< There is a configuration error */ 
    QT_COMSERR = 0x04   /*!< There is an error with the IC-compatible checksum. */
    
}commandprocessor_status;

typedef enum  
{
    QT_DETECT      = 0x80,
    QT_PRESS       = 0x40,
    QT_RELEASE     = 0x20,
    QT_MOVE        = 0x10,
    QT_VECTOR      = 0x08,
    QT_AMP         = 0x04,
    QT_SUPPRESS    = 0x02
}multitouchscreen_status;

typedef enum 
{
    NO_COMMAND = 0u,
    COMM_MODE1 = 1u,
    COMM_MODE2 = 2u,
    COMM_MODE3 = 3u
}comm_cfg;


typedef enum 
{
    QT_WAITING_BOOTLOAD_COMMAND = 0xC0,
    QT_WAITING_FRAME_DATA       = 0x80,
    QT_APP_CRC_FAIL             = 0x40,
    QT_FRAME_CRC_CHECK          = 0x02,
    QT_FRAME_CRC_PASS           = 0x04,
    QT_FRAME_CRC_FAIL           = 0x03,

}qt_boot_status_t;


//*****************************************************************************
//
//
//		info_block_driver
//
//
//*****************************************************************************

/*! \brief Object table element struct. */
typedef struct
{
	uint8_t object_type;     /*!< Object type ID. */
	uint16_t i2c_address;    /*!< Start address of the obj config structure. */
	uint8_t size;            /*!< Byte length of the obj config structure -1.*/
	uint8_t instances;       /*!< Number of objects of this obj. type -1. */
    uint8_t num_report_ids;  /*!< The max number of touches in a screen,
                              *   max number of sliders in a slider array, etc.*/
} object_t;


/*! \brief Info ID struct. */
typedef struct
{
	uint8_t family_id;            /* address 0 */
	uint8_t variant_id;           /* address 1 */
	
	uint8_t version;              /* address 2 */
	uint8_t build;                /* address 3 */
	
	uint8_t matrix_x_size;        /* address 4 */
	uint8_t matrix_y_size;        /* address 5 */
	
								  /*! Number of entries in the object table. The actual number of objects
    * can be different if any object has more than one instance. */
	uint8_t num_declared_objects; /* address 6 */
} info_id_t;

/*! \brief Info block struct holding ID and object table data and their CRC sum.
*
* Info block struct. Similar to one in info_block.h, but since
* the size of object table is not known beforehand, it's pointer to an
* array instead of an array. This is not defined in info_block.h unless
* we are compiling with IAR AVR or AVR32 compiler (__ICCAVR__ or __ICCAVR32__
* is defined). If this driver is compiled with those compilers, the
* info_block.h needs to be edited to not include that struct definition.
*
* CRC is 24 bits, consisting of CRC and CRC_hi; CRC is the lower 16 bytes and
* CRC_hi the upper 8.
*
*/

typedef struct
{
    /*! Info ID struct. */
    info_id_t info_id;
	
    /*! Pointer to an array of objects. */
    object_t *objects;
	
    /*! CRC field, low bytes. */
    uint16_t CRC;
	
    /*! CRC field, high byte. */
    uint8_t CRC_hi;
} info_block_t;



typedef struct
{
	uint8_t object_type;     /*!< Object type. */
	uint8_t instance;        /*!< Instance number. */
} report_id_map_t;



//*****************************************************************************
//
//
//		std_objects_driver
//
//
//*****************************************************************************

/*! ===Header File Version Number=== */
#define OBJECT_LIST__VERSION_NUMBER     0x11


/*! \defgroup OBJECT_LIST ===Object Type Number List===
 This list contains the enumeration of each of the object types used in the
 chip information table.

 The Structure of the name used is:
 <OBJECTPURPOSE>_<OBJECTDESCRIPTION>_T<TYPENUMBER>

 Possible Object Purposes include:
 DEBUG, GEN (General), TOUCH (Touch Sensitive Objects), PROCI
 (Processor - instance), PROCT (Processor - type), PROCG (Processor - global)

 Below is the actual list, reserved entries should not be used, spare entries
 are available for use. No entries should ever be re-used once they have been
 included in the list. Once an entry is added its configuration and/or status
 structures should be completed and commented below.

 @{
 */
#define RESERVED_T0                               0u
#define RESERVED_T1                               1u
#define DEBUG_DELTAS_T2                           2u
#define DEBUG_REFERENCES_T3                       3u
#define DEBUG_SIGNALS_T4                          4u
#define GEN_MESSAGEPROCESSOR_T5                   5u
#define GEN_COMMANDPROCESSOR_T6                   6u
#define GEN_POWERCONFIG_T7                        7u
#define GEN_ACQUISITIONCONFIG_T8                  8u
#define TOUCH_MULTITOUCHSCREEN_T9                 9u
#define TOUCH_SINGLETOUCHSCREEN_T10               10u
#define TOUCH_XSLIDER_T11                         11u
#define TOUCH_YSLIDER_T12                         12u
#define TOUCH_XWHEEL_T13                          13u
#define TOUCH_YWHEEL_T14                          14u
#define TOUCH_KEYARRAY_T15                        15u
#define PROCG_SIGNALFILTER_T16                    16u
#define PROCI_LINEARIZATIONTABLE_T17              17u
#define SPT_COMCONFIG_T18                         18u
#define SPT_GPIOPWM_T19                           19u
#define PROCI_GRIPFACESUPPRESSION_T20             20u
#define RESERVED_T21                              21u
#define PROCG_NOISESUPPRESSION_T22                22u
#define TOUCH_PROXIMITY_T23                       23u
#define PROCI_ONETOUCHGESTUREPROCESSOR_T24        24u
#define SPT_SELFTEST_T25                          25u
#define DEBUG_CTERANGE_T26                        26u
#define PROCI_TWOTOUCHGESTUREPROCESSOR_T27        27u
#define SPT_CTECONFIG_T28                         28u
#define SPT_GPI_T29                               29u
#define SPT_GATE_T30                              30u
#define TOUCH_KEYSET_T31                          31u
#define TOUCH_XSLIDERSET_T32                      32u
#define RESERVED_T33                              33u
#define GEN_MESSAGEBLOCK_T34                      34u
#define SPT_GENERICDATA_T35                       35u
#define RESERVED_T36                              36u
#define DEBUG_DIAGNOSTIC_T37                      37u
#define SPARE_T38                                 38u
#define SPARE_T39                                 39u
#define SPARE_T40                                 40u
#define SPARE_T41                                 41u
#define SPARE_T42                                 42u
#define SPARE_T43                                 43u
#define SPARE_T44                                 44u
#define SPARE_T45                                 45u
#define SPARE_T46                                 46u
#define SPARE_T47                                 47u
#define SPARE_T48                                 48u
#define SPARE_T49                                 49u
#define SPARE_T50                                 50u
/*
 * All entries spare up to 255
*/
#define RESERVED_T255                             255u

/*! @} */

/*----------------------------------------------------------------------------
  type definitions
----------------------------------------------------------------------------*/



/*! ===Object Type Definitions===
 This section contains the description, and configuration structures
 used by each object type.

The host should ALWAYS use the CHG
 line as an indication that a message is available to read;
the host should notpoll the device for messages or read the Message Processor object at any
other time. If the Message Processor object is read when the CHG
 line is not asserted, an
¡°invalid message¡± report ID is returned in the Message Processor object.
*/

/*! ==DEBUG_DELTAS_T2==
 DEBUG_DELTAS_T2 does not have a structure. It is a int16_t array which
 stores its value in LSByte first and MSByte second.
*/


/*! ==DEBUG_REFERENCES_T3==
 DEBUG_REFERENCES_T3 does not have a structure. It is a uint16_t array which
 stores its value in LSByte first and MSByte second.
*/


/*! ==DEBUG_SIGNALS_T4==
 DEBUG_SIGNALS_T4 does not have a structure. It is a uint16_t array which
 stores its value in LSByte first and MSByte second.
*/


/*! ==GEN_COMMANDPROCESSOR_T6==
 The T6 CommandProcessor allows commands to be issued to the chip
 by writing a non-zero vale to the appropriate register.
*/
/*! \brief
 This structure is used to configure the commandprocessorand should be made
 accessible to the host controller.
*/

typedef struct
{
   uint8_t reset;       /*!< Force chip reset             */
   uint8_t backupnv;    /*!< Force backup to eeprom/flash */
   uint8_t calibrate;   /*!< Force recalibration          */
   uint8_t reportall;   /*!< Force all objects to report  */
   uint8_t reserve;   /*!< Turn on output of debug data */
#if defined(__VER_1_4__)
   uint8_t diagnostic;  /*!< Controls the diagnostic object */
#endif
} gen_commandprocessor_t6_config_t;


/*! ==GEN_POWERCONFIG_T7==
 The T7 Powerconfig controls the power consumption by setting the
 acquisition frequency.
*/
/*! \brief
 This structure is used to configure the powerconfig and should be made
 accessible to the host controller.
*/
typedef struct
{
   uint8_t idleacqint;    /*!< Idle power mode sleep length in ms           */
   uint8_t actvacqint;    /*!< Active power mode sleep length in ms         */
   uint8_t actv2idleto;   /*!< Active to idle power mode delay length in units of 0.2s*/
   
} gen_powerconfig_t7_config_t;


/*! ==GEN_ACQUISITIONCONFIG_T8==
 The T8 AcquisitionConfig object controls how the device takes each
 capacitive measurement.
*/
/*! \brief
 This structure is used to configure the acquisitionconfig and should be made
 accessible to the host controller.
*/

typedef struct
{ 
   uint8_t chrgtime;          /*!< Charge-transfer dwell time             */  
   uint8_t reserved;          /*!< reserved                               */
   uint8_t tchdrift;          /*!< Touch drift compensation period        */
   uint8_t driftst;           /*!< Drift suspend time                     */
   uint8_t tchautocal;        /*!< Touch automatic calibration delay in units of 0.2s*/
   uint8_t sync;              /*!< Measurement synchronisation control    */
#if defined(__VER_1_4__)
   uint8_t atchcalst;         /*!< recalibration suspend time after last detection */
   uint8_t atchcalsthr;       /*!< Anti-touch calibration suspend threshold */
#endif
   uint8_t ATCHFRCCALTHR;
   uint8_t  ATCHFRCCALRATIO;
} gen_acquisitionconfig_t8_config_t;


/*! ==TOUCH_MULTITOUCHSCREEN_T9==
 The T9 Multitouchscreen is a multi-touch screen capable of tracking up to 8
 independent touches.
*/

/*! \brief
 This structure is used to configure the multitouchscreen and should be made
 accessible to the host controller.
*/
typedef struct
{
   /* Screen Configuration */
   uint8_t ctrl;            /*!< ACENABLE LCENABLE Main configuration field  */

   /* Physical Configuration */
   uint8_t xorigin;         /*!< LCMASK ACMASK Object x start position on matrix  */
   uint8_t yorigin;         /*!< LCMASK ACMASK Object y start position on matrix  */
   uint8_t xsize;           /*!< LCMASK ACMASK Object x size (i.e. width)         */
   uint8_t ysize;           /*!< LCMASK ACMASK Object y size (i.e. height)        */

   /* Detection Configuration */
   uint8_t akscfg;          /*!< Adjacent key suppression config     */
   uint8_t blen;            /*!< Sets the gain of the analog circuits in front of the ADC. The gain should be set in
                            conjunction with the burst length to optimize the signal acquisition. Maximum gain values for
                            a given object/burst length can be obtained following a full calibration of the system. GAIN
                            has a maximum setting of 4; settings above 4 are capped at 4.*/
   uint8_t tchthr;          /*!< ACMASK Threshold for all object channels   */
   uint8_t tchdi;           /*!< Detect integration config           */

   uint8_t orient;  /*!< LCMASK Controls flipping and rotating of touchscreen
                        *   object */
   uint8_t mrgtimeout; /*!< Timeout on how long a touch might ever stay
                        *   merged - units of 0.2s, used to tradeoff power
                        *   consumption against being able to detect a touch
                        *   de-merging early */

   /* Position Filter Configuration */
   uint8_t movhysti;   /*!< Movement hysteresis setting used after touchdown */
   uint8_t movhystn;   /*!< Movement hysteresis setting used once dragging   */
   uint8_t movfilter;  /*!< Position filter setting controlling the rate of  */

   /* Multitouch Configuration */
   uint8_t numtouch;   /*!< The number of touches that the screen will attempt
                        *   to track */
   uint8_t mrghyst;    /*!< The hysteresis applied on top of the merge threshold
                        *   to stop oscillation */
   uint8_t mrgthr;     /*!< The threshold for the point when two peaks are
                        *   considered one touch */

   uint8_t amphyst;          /*!< TBD */

  /* Resolution Controls */
  uint16_t xrange;       /*!< LCMASK */
  uint16_t yrange;       /*!< LCMASK */
  uint8_t xloclip;       /*!< LCMASK */
  uint8_t xhiclip;       /*!< LCMASK */
  uint8_t yloclip;       /*!< LCMASK */
  uint8_t yhiclip;       /*!< LCMASK */
#if defined(__VER_1_4__)
  /* edge correction controls */
  uint8_t xedgectrl;     /*!< LCMASK */
  uint8_t xedgedist;     /*!< LCMASK */
  uint8_t yedgectrl;     /*!< LCMASK */
  uint8_t yedgedist;     /*!< LCMASK */
#endif
  uint8_t JUMPLIMIT; 
  uint8_t TCHHYST; 
  uint8_t XPITCH; 
  uint8_t YPITCH; 
} touch_multitouchscreen_t9_config_t;


/*! ==TOUCH_XSLIDER_T11==
 The T11 XSlider is a slider along a y line.
*/
/*! ==TOUCH_YSLIDER_T12==
 The T11 YSlider is a slider along an x line.
*/
/*! ==TOUCH_XWHEEL_T13==
 The T11 XWheel is a slider along a y line.
*/
/*! ==TOUCH_YWHEEL_T14==
 The T11 YWheel is a slider along an x line.
*/
/*! = Config Structure =
 This structure is used to configure a slider or wheel and should be made
 accessible to the host controller.
*/
typedef struct
{
   /* Key Array Configuration */
   uint8_t ctrl;           /*!< ACENABLE LCENABLE Main configuration field           */

   /* Physical Configuration */
   uint8_t xorigin;        /*!< ACMASK LCMASK Object x start position on matrix  */
   uint8_t yorigin;        /*!< ACMASK LCMASK Object y start position on matrix  */
   uint8_t size;           /*!< ACMASK LCMASK Object x size (i.e. width)         */

   /* Detection Configuration */
   uint8_t akscfg;         /*!< Adjacent key suppression config     */
   uint8_t blen;           /*!< ACMASK Burst length for all object channels*/
   uint8_t tchthr;         /*!< ACMASK Threshold for all object channels   */
   uint8_t tchdi;          /*!< Detect integration config           */
   uint8_t reserved[2];    /*!< Spare x2 */
   uint8_t movhysti;       /*!< Movement hysteresis setting used after touchdown */
   uint8_t movhystn;       /*!< Movement hysteresis setting used once dragging */
   uint8_t movfilter;      /*!< Position filter setting controlling the rate of  */

} touch_slider_wheel_t11_t12_t13_t14_config_t;


/*! ==TOUCH_KEYARRAY_T15==
 The T15 Keyarray is a key array of upto 32 keys
*/
/*! \brief
 This structure is used to configure the keyarray and should be made
 accessible to the host controller.
*/
typedef struct
{
   /* Key Array Configuration */
   uint8_t ctrl;               /*!< ACENABLE LCENABLE Main configuration field           */

   /* Physical Configuration */
   uint8_t xorigin;           /*!< ACMASK LCMASK Object x start position on matrix  */
   uint8_t yorigin;           /*!< ACMASK LCMASK Object y start position on matrix  */
   uint8_t xsize;             /*!< ACMASK LCMASK Object x size (i.e. width)         */
   uint8_t ysize;             /*!< ACMASK LCMASK Object y size (i.e. height)        */

   /* Detection Configuration */
   uint8_t akscfg;             /*!< Adjacent key suppression config     */
   uint8_t blen;               /*!< ACMASK Burst length for all object channels*/
   uint8_t tchthr;             /*!< ACMASK LCMASK Threshold for all object channels   */
   uint8_t tchdi;              /*!< Detect integration config           */
   uint8_t reserved[2];        /*!< Spare x2 */

} touch_keyarray_t15_config_t;


/*! ==PROCI_LINEARIZATIONTABLE_T17==
The LinearizationTable operates upon either a multi touch screen and/or a single touch screen.
The linearization table is setup to remove non-linear touch sensor properties.
*/
/*! \brief
 This structure is used to configure the linearizationtable and should be made
 accessible to the host controller.
*/
typedef struct
{
 uint8_t ctrl;
 uint16_t xoffset;
 uint8_t  xsegment[16];
 uint16_t yoffset;
 uint8_t  ysegment[16];

} proci_linearizationtable_t17_config_t;

/*! ==SPT_COMCCONFIG_T18==
The communication configuration object adjust the communication behaviour of
the device.

*/
/*! \brief
This structure is used to configure the COMCCONFIG object and should be made
accessible to the host controller.
*/

typedef struct
{
    uint8_t  ctrl;
    uint8_t  cmd;
} spt_comcconfig_t18_config_t;

/*! ==SPT_GPIOPWM_T19==
 The T19 GPIOPWM object provides input, output, wake on change and PWM support
for one port.
*/
/*! \brief
 This structure is used to configure the GPIOPWM object and should be made
 accessible to the host controller.
*/
typedef struct
{
   /* GPIOPWM Configuration */
   uint8_t ctrl;             /*!< Main configuration field           */
   uint8_t reportmask;       /*!< Event mask for generating messages to
                              *   the host */
   uint8_t dir;              /*!< Port DIR register   */
   uint8_t intpullup;        /*!< Port pull-up per pin enable register */
   uint8_t out;              /*!< Port OUT register*/
   uint8_t wake;             /*!< Port wake on change enable register  */
   uint8_t pwm;              /*!< Port pwm enable register    */
   uint8_t period;           /*!< PWM period (min-max) percentage*/
   uint8_t duty[4];          /*!< PWM duty cycles percentage */

} spt_gpiopwm_t19_config_t;


/*! ==PROCI_GRIPFACESUPPRESSION_T20==
 The T20 GRIPFACESUPPRESSION object provides grip boundary suppression and
 face suppression
*/
/*! \brief
 This structure is used to configure the GRIPFACESUPPRESSION object and
 should be made accessible to the host controller.
*/
typedef struct
{
   uint8_t ctrl;
   uint8_t xlogrip;
   uint8_t xhigrip;
   uint8_t ylogrip;
   uint8_t yhigrip;
   uint8_t maxtchs;
   uint8_t reserved;
   uint8_t szthr1;
   uint8_t szthr2;
   uint8_t shpthr1;
   uint8_t shpthr2;
#if defined(__VER_1_4__)
   uint8_t supextto;
#endif
} proci_gripfacesuppression_t20_config_t;


/*! ==PROCG_NOISESUPPRESSION_T22==
 The T22 NOISESUPPRESSION object provides frequency hopping acquire control,
 outlier filtering and grass cut filtering.
*/
/*! \brief
 This structure is used to configure the NOISESUPPRESSION object and
 should be made accessible to the host controller.
*/

typedef struct
{

   uint8_t ctrl;

#if defined(__VER_1_2__)
   uint8_t outflen;
#elif defined(__VER_1_4__)
   uint8_t reserved;
#endif

   uint8_t reserved1;
   int16_t gcaful;
   int16_t gcafll;
   
#if defined(__VER_1_2__)
   uint8_t gcaflcount;
#elif defined(__VER_1_4__)
   uint8_t actvgcafvalid;        /* LCMASK */
#endif

   
   uint8_t noisethr;
   uint8_t reserved2;
   uint8_t freqhopscale;

#if defined(__VER_1_2__)
   uint8_t freq0;
   uint8_t freq1;
   uint8_t freq2;
#elif defined(__VER_1_4__)
   uint8_t freq[5u];             /* LCMASK ACMASK */
   uint8_t idlegcafvalid;        /* LCMASK */
#endif
     
} procg_noisesuppression_t22_config_t;


/*! ==TOUCH_PROXIMITY_T23==
 The T23 Proximity is a proximity key made of upto 16 channels
*/
/*! \brief
 This structure is used to configure the prox object and should be made
 accessible to the host controller.
*/
typedef struct
{
   /* Prox Configuration */
   uint8_t ctrl;               /*!< ACENABLE LCENABLE Main configuration field           */

   /* Physical Configuration */
   uint8_t xorigin;           /*!< ACMASK LCMASK Object x start position on matrix  */
   uint8_t yorigin;           /*!< ACMASK LCMASK Object y start position on matrix  */
   uint8_t xsize;             /*!< ACMASK LCMASK Object x size (i.e. width)         */
   uint8_t ysize;             /*!< ACMASK LCMASK Object y size (i.e. height)        */
   uint8_t reserved_for_future_aks_usage;
   /* Detection Configuration */
   uint8_t blen;               /*!< ACMASK Burst length for all object channels*/
   uint16_t tchthr;             /*!< LCMASK Threshold    */
   uint8_t tchdi;              /*!< Detect integration config           */
   uint8_t average;            /*!< LCMASK Sets the filter length on the prox signal */
   uint16_t rate;               /*!< Sets the rate that prox signal must exceed */

} touch_proximity_t23_config_t;


/*! ==PROCI_ONETOUCHGESTUREPROCESSOR_T24==
 The T24 ONETOUCHGESTUREPROCESSOR object provides gesture recognition from
 touchscreen touches.
*/
/*! \brief
 This structure is used to configure the ONETOUCHGESTUREPROCESSOR object and
 should be made accessible to the host controller.
*/

typedef struct
{
   uint8_t  ctrl;
#if defined(__VER_1_2__)
   uint8_t  reserved_1;
#elif defined(__VER_1_4__)
   uint8_t  numgest;
#endif
   uint16_t gesten;
   uint8_t  pressproc;
   uint8_t  tapto;
   uint8_t  flickto;
   uint8_t  dragto;
   uint8_t  spressto;
   uint8_t  lpressto;
   uint8_t  reppressto;
   uint16_t flickthr;
   uint16_t dragthr;
   uint16_t tapthr;
   uint16_t throwthr;
} proci_onetouchgestureprocessor_t24_config_t;


/*! ==SPT_SELFTEST_T25==
 The T25 SEFLTEST object provides self testing routines
*/
/*! \brief
 This structure is used to configure the SELFTEST object and
 should be made accessible to the host controller.
*/

/*! = signal limit test structure = */

typedef struct
{
   uint16_t upsiglim;              /* LCMASK */
   uint16_t losiglim;              /* LCMASK */

} siglim_t;

/*! = Config Structure = */

typedef struct
{
  uint8_t  ctrl;                 /* LCENABLE */
  uint8_t  cmd;
#if(NUM_OF_TOUCH_OBJECTS)
  siglim_t siglim[NUM_OF_TOUCH_OBJECTS];   /* LCMASK */
#endif

} spt_selftest_t25_config_t;


/*! ==PROCI_TWOTOUCHGESTUREPROCESSOR_T27==
 The T27 TWOTOUCHGESTUREPROCESSOR object provides gesture recognition from
 touchscreen touches.
*/
/*! \brief
 This structure is used to configure the TWOTOUCHGESTUREPROCESSOR object and
 should be made accessible to the host controller.
*/

typedef struct
{
   uint8_t  ctrl;          /*!< Bit 0 = object enable, bit 1 = report enable */

#if defined(__VER_1_2__)
    uint8_t  reserved1;    
#elif defined(__VER_1_4__)
    uint8_t  numgest;       /*!< Runtime control for how many two touch gestures
                            *   to process */
#endif    
    uint8_t reserved2;
    
    uint8_t gesten;        /*!< Control for turning particular gestures on or
                            *   off */
    uint8_t  rotatethr;
    uint16_t zoomthr;

} proci_twotouchgestureprocessor_t27_config_t;


/*! ==SPT_CTECONFIG_T28==
 The T28 CTECONFIG object provides controls for the CTE.
*/

/*! \brief
 This structure is used to configure the CTECONFIG object and
 should be made accessible to the host controller.
*/


typedef struct
{
   uint8_t ctrl;          /*!< Ctrl field reserved for future expansion */
   uint8_t cmd;           /*!< Cmd field for sending CTE commands */
   uint8_t mode;          /*!< LCMASK CTE mode configuration field */
   uint8_t idlegcafdepth; /*!< LCMASK The global gcaf number of averages when idle */
   uint8_t actvgcafdepth; /*!< LCMASK The global gcaf number of averages when active */
   uint8_t VOLTAGE;
} spt_cteconfig_t28_config_t; /*This field defines the nominal AVdd voltage level in user's design*/



/*! ==DEBUG_DIAGNOSTIC_T37==
 The Diagnostic Debug object holds the debug data. 
*/

/*! \brief
Three modes of data are available,
determined by the mode setting in the DIAGNOSTIC field in the Command Processor object
* Deltas mode - Signed (two¡¯s complement) 16-bit delta values for all channels
* References mode - Unsigned 16-bit reference values for all channels
* CTE mode - Recommended gains for all Y lines received from the capacitive touch engine 
(CTE) in the device
*/

typedef struct
{
    uint8_t mode;
    uint8_t page;
    uint8_t data[128];
    
} debug_diagnositc_t37_t;

typedef struct
{
    uint8_t mode;
    uint8_t page;
    int8_t data[128];
    
} debug_diagnositc_t37_delta_t;

typedef struct
{
    uint8_t mode;
    uint8_t page;
    uint16_t data[64];
    
} debug_diagnositc_t37_reference_t;

typedef struct
{
    uint8_t mode;
    uint8_t page;
    uint8_t data[128];
    
} debug_diagnositc_t37_cte_t;

QT_EXT debug_diagnositc_t37_t dgb_data;
QT_EXT debug_diagnositc_t37_delta_t dbg_delta;
QT_EXT debug_diagnositc_t37_reference_t dbg_ref;


//*****************************************************************************
//
//
//		touch_driver
//
//
//*****************************************************************************

/*! \brief Touch driver version number. */
#define TOUCH_DRIVER_VERSION    0x02u

#define I2C_APPL_ADDR_0         0x4Au
#define I2C_APPL_ADDR_1         0x4Bu
#define I2C_BOOT_ADDR_0         0x24u
#define I2C_BOOT_ADDR_1         0x25u

#define NUM_OF_I2C_ADDR         4

/* \brief Defines CHANGE line active mode. */
#define CHANGELINE_NEGATED          0u
#define CHANGELINE_ASSERTED         1u


#define CONNECT_OK                  1u
#define CONNECT_ERROR               2u

#define READ_MEM_OK                 1u
#define READ_MEM_FAILED             2u

#define MESSAGE_READ_OK             1u
#define MESSAGE_READ_FAILED         2u

#define WRITE_MEM_OK                1u
#define WRITE_MEM_FAILED            2u

#define CFG_WRITE_OK                1u
#define CFG_WRITE_FAILED            2u

#define CFG_READ_OK                 1u
#define CFG_READ_FAILED             2u

#define I2C_INIT_OK                 1u
#define I2C_INIT_FAIL               2u

#define CRC_CALCULATION_OK          1u
#define CRC_CALCULATION_FAILED      2u

#define ID_MAPPING_OK               1u
#define ID_MAPPING_FAILED           2u

#define ID_DATA_OK                  1u
#define ID_DATA_NOT_AVAILABLE       2u


enum driver_setup_t {DRIVER_SETUP_OK, DRIVER_SETUP_INCOMPLETE};


/* Array of I2C addresses where we are trying to find the chip. */
extern uint8_t i2c_addresses[];

/*! \brief Returned by get_object_address() if object is not found. */
#define OBJECT_NOT_FOUND   0u

/*! Address where object table starts at touch IC memory map. */
#define OBJECT_TABLE_START_ADDRESS      7U

/*! Size of one object table element in touch IC memory map. */
#define OBJECT_TABLE_ELEMENT_SIZE       6U

/*! Offset to RESET register from the beginning of command processor. */
#define RESET_OFFSET                    0u

/*! Offset to BACKUP register from the beginning of command processor. */
#define BACKUP_OFFSET       1u

/*! Offset to CALIBRATE register from the beginning of command processor. */
#define CALIBRATE_OFFSET    2u

/*! Offset to REPORTALL register from the beginning of command processor. */
#define REPORTATLL_OFFSET   3u

/*! Offset to DEBUG_CTRL register from the beginning of command processor. */
#define DEBUG_CTRL_OFFSET   4u

/*! Offset to DIAGNOSTIC_CTRL register from the beginning of command processor. */
#define DIAGNOSTIC_OFFSET   5u

/*----------------------------------------------------------------------------
Function prototypes.
----------------------------------------------------------------------------*/


/* Initializes the touch driver: tries to connect to given address,
* sets the message handler pointer, reads the info block and object
* table, sets the message processor address etc. */

uint8_t init_touch_driver(void);
uint8_t close_touch_driver(void);
uint8_t reset_chip(void);
uint8_t calibrate_chip(void);
uint8_t diagnostic_chip(uint8_t mode);
uint8_t backup_config(void);
uint8_t get_variant_id(uint8_t *variant);
uint8_t get_family_id(uint8_t *family_id);
uint8_t get_build_number(uint8_t *build);
uint8_t get_version(uint8_t *version);
uint8_t write_power_config(gen_powerconfig_t7_config_t power_config);
uint8_t write_acquisition_config(gen_acquisitionconfig_t8_config_t acq_config);
uint8_t write_multitouchscreen_config(uint8_t screen_number, touch_multitouchscreen_t9_config_t cfg);
uint8_t write_keyarray_config(uint8_t key_array_number, touch_keyarray_t15_config_t cfg);
uint8_t write_linearization_config(uint8_t instance, proci_linearizationtable_t17_config_t cfg);
uint8_t write_comc_config(uint8_t instance, spt_comcconfig_t18_config_t cfg);
uint8_t write_gpio_config(uint8_t instance, spt_gpiopwm_t19_config_t cfg);
uint8_t write_gripsuppression_config(uint8_t instance, proci_gripfacesuppression_t20_config_t cfg);
uint8_t write_noisesuppression_config(uint8_t instance, procg_noisesuppression_t22_config_t cfg);
uint8_t write_proximity_config(uint8_t instance, touch_proximity_t23_config_t cfg);
uint8_t write_onetouchgesture_config(uint8_t instance, proci_onetouchgestureprocessor_t24_config_t cfg);
uint8_t write_selftest_config(uint8_t instance, spt_selftest_t25_config_t cfg);
uint8_t write_twotouchgesture_config(uint8_t instance, proci_twotouchgestureprocessor_t27_config_t cfg);
uint8_t write_CTE_config(spt_cteconfig_t28_config_t cfg);
uint8_t write_simple_config(uint8_t object_type, uint8_t instance, void *cfg);
uint8_t get_object_size(uint8_t object_type);
uint8_t type_to_report_id(uint8_t object_type, uint8_t instance);
uint8_t report_id_to_type(uint8_t report_id, uint8_t *instance);
uint16_t get_object_address(uint8_t object_type, uint8_t instance);
uint32_t get_stored_infoblock_crc(void);
uint8_t calculate_infoblock_crc(uint32_t *crc_pointer);
uint32_t CRC_24(uint32_t crc, uint8_t byte1, uint8_t byte2);
uint8_t read_id_block(info_id_t *id);
uint8_t get_max_report_id(void);




QT_EXT info_block_t *info_block;
QT_EXT report_id_map_t *report_id_map;
QT_EXT int max_report_id ;
QT_EXT uint8_t max_message_length;
QT_EXT uint16_t message_processor_address;
QT_EXT uint16_t command_processor_address;
QT_EXT enum driver_setup_t driver_setup;
QT_EXT uint8_t *msg;

QT_EXT uint16_t comc_cfg_address;


/*! Touch device I2C Address */
QT_EXT  uint8_t QT_i2c_address;
QT_EXT  uint8_t QT_i2c_boot_address;

QT_EXT uint8_t chip_detected_flag;
QT_EXT uint8_t com_cfg_change;

//*****************************************************************************
//
//
//		main
//
//
//*****************************************************************************

QT_EXT void get_message(void);
QT_EXT uint8_t write_mem( uint16_t Address, uint8_t ByteCount, uint8_t *Data );
QT_EXT uint8_t read_mem( uint16_t Address, uint8_t ByteCount, uint8_t *Data );
QT_EXT uint8_t read_uint16_t( uint16_t Address, uint16_t *Data );

QT_EXT uint8_t ChangeLineStatus( void );
QT_EXT void message_handler(uint8_t *msg, uint8_t length);
QT_EXT void Change_TouchNum(uint8_t touch_num);




QT_EXT unsigned short ChkPress;
QT_EXT unsigned char KeyPress;



QT_EXT unsigned char tmsg[9];
QT_EXT unsigned int position[2][2];
QT_EXT unsigned int Index[2];
//QT_EXT unsigned short x_pos, y_pos;


#define INVALID_POS         0x3FF
#define TOUCH_PRESS         1u
#define TOUCH_RELEASE       0u


//*****************************************************************************
//
//
//  QT602240 Configuration Functions
//
//
//*****************************************************************************

void qt_Power_Config_Init(void);
void qt_Acquisition_Config_Init(void);
void qt_Multitouchscreen_Init(void);
void qt_KeyArray_Init(void);
void qt_ComcConfig_Init(void);
void qt_Gpio_Pwm_Init(void);
void qt_Grip_Face_Suppression_Config_Init(void);
void qt_Noise_Suppression_Config_Init(void);
void qt_Proximity_Config_Init(void);
void qt_One_Touch_Gesture_Config_Init(void);
void qt_Selftest_Init(void);
void qt_Two_touch_Gesture_Config_Init(void);
void qt_CTE_Config_Init(void);
unsigned char Comm_Config_Process(unsigned char change_en);




//*****************************************************************************
//
//
//  Diagnostic Function
//
//
//*****************************************************************************


unsigned char read_diagnostic_reference(debug_diagnositc_t37_reference_t * dbg,unsigned char page);
unsigned char read_diagnostic_delta(debug_diagnositc_t37_delta_t * dbg, unsigned char page);
unsigned char read_diagnostic_debug(debug_diagnositc_t37_t *dbg, unsigned char mode, unsigned char page);

QT_EXT unsigned short diagnostic_addr;
QT_EXT unsigned short diagnostic_size;
#endif //__QT_602240_H__