#ifndef INCLUDE_TESTS_H_
#define INCLUDE_TESTS_H_

#include <common.h>

/****************************/
/*** Driver Control tests ***/
/****************************/

/*
 * @brief	Routine of ROS connection test via USB
 * @note 	USB bus is used (for ROS activity, not test)
 */
void testROSConnection( void );

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


void testWheelsSpeedControlRoutine( void );

/*********************/
/*** Encoder tests ***/
/*********************/

/**
 * @brief   Test ticks and revs counting, also direction detection
 * @note    chprintf works only when encoder is rotation
 */
void testEncoderCommonRoutine( void );

/**************************/
/***    Odometry tests  ***/
/**************************/

void testOdometryRoutine( void );

/****************************/
/*** Remote Control tests ***/
/****************************/
/*
 * @brief   Routine of remote control testing
 * @note    The routine has internal infinite loop
 *          and strictly depended on time
 */
void testRemoteControlRoutine( void );

/****************************/
/*** Steering Angle tests ***/
/****************************/

/*
 * @brief   Test for routine of getting steering angle
 * @note    There are 2 options:
 *          * send data to Matlab
 *          * send data to Terminal
*/
void testSteerAngleSendData( void );

/*
 * @brief   Control steering wheels to get angle
 * @note    Control ONLY steering wheels
*/
void testSteerAngleDetection( void );

/*
 * @brief
 */
void testSteerAngleGetControlAngleCoeffitient( void );


/*************************************/
/***    Control System tests       ***/
/*************************************/

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


/***********************/
/***    GUI tests    ***/
/***********************/

/*
 * @brief   Test GUI with odometry
*/
void testGUIRoutineServer ( void );


/***********************/
/***    ROS tests    ***/
/***********************/

/*
 * @brief   Test odometry via ROS
 * @note    Frequency = 50 Hz
*/
void testRoutineROSOdometry( void );

/*
 * @brief   Test odometry, speed and steering control via ROS
 * @note    Frequency = 50 Hz
*/
void testRosRoutineControl( void );


void testSpeedFilter( void );

void testButtonRoutine( void );



static inline void testsRoutines( void )
{
#if (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_ROS)

    testROSConnection( );

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_LL_DRIVER)

    testWheelsControlRoutines( );

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_RAW_LL_DRIVE)

    testRawWheelsControlRoutine( );

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_SPEED_LL_DRV )

    testWheelsSpeedControlRoutine( );

#elif ( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_ENCODER )

    testEncoderCommonRoutine( );

#elif ( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_RC )

    testRemoteControlRoutine( );

#elif ( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_ODOMETRY )

    testOdometryRoutine( );

#elif ( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_STEER_ANGL_CALC)

    testSteerAngleDetection( );

#elif( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_STEER_ANGL_SEND )

    testSteerAngleSendData( );

#elif ( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_GUI_SERVER )

    testGUIRoutineServer( );

#elif( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_ROS_ODOMETRY )

    testRoutineROSOdometry( );

#elif( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_ROS_CONTROL )

    testRosRoutineControl( );

#elif( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_STEERING_CS )

    testSteeringCS( );

#elif( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_STEER_ANGLE_LLD_CONTRL )

    testSteerAngleGetControlAngleCoeffitient( );

#elif( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_SPEED_CS )

    testSpeedCS( );

#elif( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_SPEED_FILTER )

    testSpeedFilter( );

#elif( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_ESC_CALIBRATION )

    testDrivingWheelsESCCalibration( );


#elif( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_BUTTON_STATE)

    testButtonRoutine( );

#endif
}

#endif /* INCLUDE_TESTS_H_ */
