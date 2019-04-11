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
#define     PROGRAM_ROUTINE_TEST_ESC_CALIBRATION        3
/***    ENCODER / ODOMETRY RELATED      ***/
#define     PROGRAM_ROUTINE_TEST_ENCODER                4
#define     PROGRAM_ROUTINE_TEST_ODOMETRY               5
#define     PROGRAM_ROUTINE_TEST_RESET_ODOMETRY         6
/***    REMOTE CONTROL RELATED          ***/
#define     PROGRAM_ROUTINE_TEST_RC                     7
/***    STEERING RELATED                ***/
#define     PROGRAM_ROUTINE_TEST_STEERING_CS            8
#define     PROGRAM_ROUTINE_TEST_STEER_ANGL_SEND        9
/***    SPEED RELATED                   ***/
#define     PROGRAM_ROUTINE_TEST_SPEED_CS               11
#define     PROGRAM_ROUTINE_TEST_SPEED_LIMIT_CALIB      12
#define     PROGRAM_ROUTINE_TEST_SPEED_FILTER           13
/*** 	LIGHT RELATED					***/
#define		PROGRAM_ROUTINE_TEST_LIGHT					15 
#define		PROGRAM_ROUTINE_TEST_LED_MATRIX				16
/***    BUTTON RELATED                  ***/
#define     PROGRAM_ROUTINE_TEST_BUTTON_STATE           20
/***    ROS RELATED                     ***/
#define     PROGRAM_ROUTINE_TEST_ROS_ODOMETRY           30
#define     PROGRAM_ROUTINE_TEST_ROS_CONTROL            35
#define     PROGRAM_ROUTINE_TEST_GUI_SERVER             40
#define     PROGRAM_ROUTINE_TEST_ROS                    60
/***    TIMER RELATED                   ***/
#define     PROGRAM_ROUTINE_TEST_SYSTEM_TIMER           61

#define     MAIN_PROGRAM_ROUTINE                        PROGRAM_ROUTINE_MASTER


/**************/
/*** MACROS ***/
/**************/

#define CLIP_VALUE(x, min, max) ((x) < (min) ? (min) :      \
                                 (x) > (max) ? (max) : (x))

/******************/
/*** LLD LIMITS ***/
/******************/

#define SPEED_MAX           1650    // 2000
#define SPEED_NULL_FORWARD  1550    // 1500

#define SPEED_ZERO          1500

#define SPEED_NULL_BACK     1450
#define SPEED_MIN           1350

#define STEER_MAX           2300
#define STEER_NULL          1400
#define STEER_MIN           500

#define CONTROL_MAX         100
#define CONTROL_NULL        0
#define CONTROL_MIN         (-100)


/*************/
/*** DEBUG ***/
/*************/

void debug_stream_init( void );
void dbgprintf( const char* format, ... );


/****************************/
/*** OBJECT CONFIGURATION ***/
/****************************/

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

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_COMMON_H_ */
