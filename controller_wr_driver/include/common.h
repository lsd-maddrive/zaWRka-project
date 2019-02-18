#ifndef INCLUDE_COMMON_H_
#define INCLUDE_COMMON_H_

#include <ch.h>
#include <hal.h>

#include <stdint.h>
#include <stdlib.h>

#define     PROGRAM_ROUTINE_MASTER                      0
#define     PROGRAM_ROUTINE_TEST_LL_DRIVER              1
#define     PROGRAM_ROUTINE_TEST_RAW_LL_DRIVE           2
#define     PROGRAM_ROUTINE_TEST_ENCODER                3
#define     PROGRAM_ROUTINE_TEST_SPEED_ENCODER          4
#define     PROGRAM_ROUTINE_TEST_ODOMETRY               5
#define     PROGRAM_ROUTINE_TEST_RC                     6



#define     MAIN_PROGRAM_ROUTINE                        PROGRAM_ROUTINE_TEST_ENCODER


/**************/
/*** Macros ***/
/**************/

#define CLIP_VALUE(x, min, max) ((x) < (min) ? (min) :      \
                                 (x) > (max) ? (max) : (x))



/******************/
/*** LLD LIMITS ***/
/******************/

#define SPEED_MAX           1920
#define SPEED_NULL_FORWARD  1520
#define SPEED_NULL_BACK     1480
#define SPEED_MIN           1160


#define STEER_MAX           2080
#define STEER_NULL          1620
#define STEER_MIN           1160


#define CONTROL_MAX         100
#define CONTROL_NULL        0
#define CONTROL_MIN         (-100)

/****************************/
/*** OBJECT CONFIGURATION ***/
/****************************/

#define WHEEL_RADIUS_CM     4
#define WHEEL_BASE_CM       30
#define WHEEL_BASE_M        0.3

#endif /* INCLUDE_COMMON_H_ */
