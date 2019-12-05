#ifndef INCLUDE_COMMON_H_
#define INCLUDE_COMMON_H_

#include <ch.h>
#include <hal.h>

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#define     PROGRAM_ROUTINE_MASTER                      0
/***    DRIVE CONTROL RELATED           ***/
#define     PROGRAM_ROUTINE_TEST_LL_DRIVER              1
#define     PROGRAM_ROUTINE_TEST_RAW_LL_DRIVE           2
/***    ENCODER / ODOMETRY RELATED      ***/
#define     PROGRAM_ROUTINE_TEST_ENCODER                4
#define     PROGRAM_ROUTINE_TEST_ODOMETRY               5
#define     PROGRAM_ROUTINE_TEST_RESET_ODOMETRY         6
#define		PROGRAM_ROUTINE_ODOMETRY_RC					7 
#define		PROGRAM_ROUTINE_X_DIST_ODOMETRY				8
/***    REMOTE CONTROL RELATED          ***/
#define     PROGRAM_ROUTINE_TEST_RC                     10

/***    STEERING RELATED                ***/
#define     PROGRAM_ROUTINE_TEST_STEERING_CS            12
#define     PROGRAM_ROUTINE_TEST_STEER_ANGL_SEND        13
/***    SPEED RELATED                   ***/
#define     PROGRAM_ROUTINE_TEST_SPEED_CS               14
#define     PROGRAM_ROUTINE_TEST_SPEED_LIMIT_CALIB      15
#define     PROGRAM_ROUTINE_TEST_SPEED_FILTER           16
/***    BOTH CS RELATED                 ***/
#define     PROGRAM_ROUTINE_TEST_UART_CS                17
/*** 	LIGHT RELATED					***/
#define		PROGRAM_ROUTINE_TEST_LIGHT					18 
#define		PROGRAM_ROUTINE_TEST_LED_MATRIX				19
/***    BUTTON RELATED                  ***/
#define     PROGRAM_ROUTINE_TEST_BUTTON_STATE           20
/***    LINK RELATED                     ***/
#define     PROGRAM_ROUTINE_TEST_LINK_CONTROL           61
#define     PROGRAM_ROUTINE_TEST_LINK                   62
#define     PROGRAM_ROUTINE_TEST_LINK_ADC_CALIB         63

#define     MAIN_PROGRAM_ROUTINE                        PROGRAM_ROUTINE_TEST_LINK_CONTROL

/*============================================================================*/
/* MACROS 																	  */
/*============================================================================*/

#define CLIP_VALUE(x, min, max) ((x) < (min) ? (min) :      \
                                 (x) > (max) ? (max) : (x))

/*============================================================================*/
/* LLD LIMITS 																  */
/*============================================================================*/



#define CONTROL_MAX         100
#define CONTROL_NULL        0
#define CONTROL_MIN         (-100)


/*============================================================================*/
/* DEBUG 																	  */
/*============================================================================*/
/**
* @brief    Initialization of usb-serial unit
* @note     Stable for repeated calls 
*/
void debug_stream_init( void );

/**
* @brief    Print aka chprintf 
*/
void dbgprintf( const char* format, ... );


/*============================================================================*/
/* OBJECT CONFIGURATION 													  */
/*============================================================================*/

#define WHEEL_RADIUS_CM     4
#define WHEEL_RADIUS_M      0.04
#define WHEEL_BASE_CM       30
#define WHEEL_BASE_M        0.3


/**
 * @brief   Initialize EXT driver with empty config
 * @note    Safe to call any times, it checks state of previous call
 * @note    Must be called before EXT driver work
 */
void commonExtDriverInit ( void );

/**
 * @brief   Initialize all base units
 */
void mainUnitsInit( void );

/**
 * @brief   Base control system
 */
void mainControlTask( void );


/*** Functions for math ***/

typedef struct range_map
{
    float k;
    float b;
} range_map_t;

void range_map_init(range_map_t   *ctx, 
                    float         in_min, 
                    float         in_max, 
                    float         out_min, 
                    float         out_max);

void range_map_init_raw(range_map_t   *ctx, 
                        float         k, 
                        float         b);

float range_map_call(range_map_t   *ctx,
                     float         val);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_COMMON_H_ */
