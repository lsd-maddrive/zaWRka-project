#ifndef INCLUDE_TESTS_H_
#define INCLUDE_TESTS_H_

#include <common.h>

/*============================================================================*/
/* Driver Control tests                                                       */
/*============================================================================*/

/*
 * @brief   Routine of low level driver control testing
 * @note    The routine has internal infinite loop
 * @note    Changing raw values of pwm dutycycle
 */
void testRawWheelsControlRoutine( void );

/*
 * @brief   Test steering and speed lld control
 * @note    Linear speed of object is also displayed
 */
void testWheelsControlRoutines( void );

/*
 * @brief   Calibration of ESC for driving wheels
 * @note    send Neutral, then MAX, then MIN
 */
void testDrivingWheelsESCCalibration ( void );

/*
 * @brief   Test for speed max/min limits calibration
 * @note    show linear speed and control signal in %
 */
void testSpeedLimitsCalibrationRoutine( void );

/*============================================================================*/
/* Encoder tests                                                              */
/*============================================================================*/

/**
 * @brief   Test ticks and revs counting, also direction detection
 * @note    chprintf works only when encoder is rotation
 */
void testEncoderCommonRoutine( void );

/*============================================================================*/
/*    Odometry tests                                                          */
/*============================================================================*/

/*
 * @brief   Test Odometry Routine
 * @note    There are 5 options:
 *          * TOTAL_ODOMETRY (
 *          * SPEED_ODOMETRY
 *          * POS_ODOMETRY
 *          * TETTA_ODOMETRY
 *          * JUST_ODOMETRY
*/
void testOdometryRoutine( void );

/*
 * @brief   Test for odometry reset
 */
void testResetOdometryRoutine( void );

/*
 * @brief   Test for odometry (x, y, tetta values)
 * @note    Control via RC 
*/
void testRCOdodmetry( void );

/*
 * @brief   Test odometry
 * @note    Reference value of x-distance = 20 cm 
 *          Speed is constant (0.1 m/s)
*/
void testXdistanceOdometry( void );

/*============================================================================*/
/* Remote Control tests                                                       */
/*============================================================================*/

/*
 * @brief   Routine of remote control testing
 * @note    The routine has internal infinite loop
 *          and strictly depended on time
 */
void testRemoteControlRoutine( void );

/*============================================================================*/
/* Steering Angle tests                                                       */
/*============================================================================*/

/*
 * @brief   Test for routine of getting steering angle
 * @note    There are 2 options:
 *          * send data to Matlab (STEER_FB_MATLAB)
 *          * send data to Terminal (STEER_FB_TERMINAL)
*/
void testSteerAngleSendData( void );


/*============================================================================*/
/* Control System tests                                                       */
/*============================================================================*/

/*
 * @brief   Test steering control system with feedback
 * @note    There are 2 options:
 *          - Show data in Terminal
 *          - Send limited number of data to Matlab
*/
void testSteeringCS ( void );

/*
 * @brief   Test speed control system with feedback
*/
void testSpeedCS ( void );

/*
 * @brief   Test LPF for speed control system
*/
void testSpeedFilter( void );

/*
 * @brief   Test speed CS and steer CS via UART 7
*/
void testUARTControl( void );

/*============================================================================*/
/* Light tests                                                                */
/*============================================================================*/

/**
 * @brief   Test lights (LEDs) with different input conditions 
 */
void testLightRoutine( void ); 

/**
 * @brief   Test button click state changing module 
 */
void testButtonRoutine( void );

/**
 * @brief   Test GUI (Android) connection via sending data to serial-mproto and showing on Android device
 */
void testGUIRoutineServer ( void );

/**
 * @brief   Test link (serial - mproto) connection setting CS values and showing all info about car
 */
void testLinkControl( void );

/**
 * @brief   Test link (serial - mproto) connection setting raw PWM values and representing raw ADC values
 */
void testLinkADCCalib( void );

/**
 * @brief   Test link (serial - mproto) connection sending some sample bytes
 */
void testLinkConnection( void );

#endif /* INCLUDE_TESTS_H_ */
