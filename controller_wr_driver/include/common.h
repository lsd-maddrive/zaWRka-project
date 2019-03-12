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
#define     PROGRAM_ROUTINE_TEST_LL_DRIVER              1
#define     PROGRAM_ROUTINE_TEST_RAW_LL_DRIVE           2
#define     PROGRAM_ROUTINE_TEST_ESC_CALIBRATION        3
#define     PROGRAM_ROUTINE_TEST_ENCODER                4
#define     PROGRAM_ROUTINE_TEST_ODOMETRY               5
#define     PROGRAM_ROUTINE_TEST_RC                     6
#define     PROGRAM_ROUTINE_TEST_STEERING_CS            7
#define     PROGRAM_ROUTINE_TEST_STEER_ANGL_CALC        8
#define     PROGRAM_ROUTINE_TEST_STEER_ANGL_SEND        9
#define     PROGRAM_ROUTINE_TEST_STEER_ANGLE_LLD_CONTRL 10
#define     PROGRAM_ROUTINE_TEST_SPEED_CS               11
#define     PROGRAM_ROUTINE_TEST_SPEED_LL_DRV           12
#define     PROGRAM_ROUTINE_TEST_SPEED_FILTER           13
#define     PROGRAM_ROUTINE_TEST_BUTTON_STATE           20
#define     PROGRAM_ROUTINE_TEST_ROS_ODOMETRY           30
#define     PROGRAM_ROUTINE_TEST_ROS_CONTROL            35
#define     PROGRAM_ROUTINE_TEST_GUI_SERVER             40
#define     PROGRAM_ROUTINE_TEST_ROS                    60

#define     MAIN_PROGRAM_ROUTINE                        PROGRAM_ROUTINE_TEST_ROS_CONTROL


/**************/
/*** MACROS ***/
/**************/

#define CLIP_VALUE(x, min, max) ((x) < (min) ? (min) :      \
                                 (x) > (max) ? (max) : (x))

/******************/
/*** LLD LIMITS ***/
/******************/

#define SPEED_MAX           1643    //2000

#define SPEED_ZERO          1500

#define SPEED_NULL_FORWARD  1543    // 1500

#define SPEED_NULL_BACK     1457
#define SPEED_MIN           1357    //1000

#define STEER_MAX           2080
#define STEER_NULL          1620
#define STEER_MIN           1160

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


#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_COMMON_H_ */
