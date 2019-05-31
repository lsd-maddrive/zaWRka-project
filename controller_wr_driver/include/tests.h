#ifndef INCLUDE_TESTS_H_
#define INCLUDE_TESTS_H_

#include <common.h>

/****************************/
/*** Driver Control tests ***/
/****************************/

/*
 * @brief   Routine of system timer test
 */
void testSystemTimer( void );

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


/*
 * @brief   Test for speed max/min limits calibration
 * @note    show linear speed and control signal in %
 */
void testSpeedLimitsCalibrationRoutine( void );

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

/****************************/
/*** Remote Control tests ***/
/****************************/
/*
 * @brief   Routine of remote control testing
 * @note    The routine has internal infinite loop
 *          and strictly depended on time
 */
void testRemoteControlRoutine( void );

/*
 * @brief   Routine to test RC-mode&Odometry&CS-mode
 */
void testRemoteControlOdometryRoutine( void );

/****************************/
/*** Steering Angle tests ***/
/****************************/

/*
 * @brief   Test for routine of getting steering angle
 * @note    There are 2 options:
 *          * send data to Matlab (STEER_FB_MATLAB)
 *          * send data to Terminal (STEER_FB_TERMINAL)
*/
void testSteerAngleSendData( void );


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

/*
 * @brief   Test LPF for speed control system
*/
void testSpeedFilter( void );

/*
 * @brief   Test speed CS and steer CS via UART 7
*/
void testUARTControl( void );

/***********************/
/***   Light tests   ***/
/***********************/

/**
 * @brief   Test lights (LEDs) with different input conditions 
 */
void testLightRoutine( void ); 

/*
 * NEED COMMENT
 */
void testLedMatrixRoutine( void );

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



void testButtonRoutine( void );



static inline void testsRoutines( void )
{
#if (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_SYSTEM_TIMER)

    testSystemTimer();

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_ROS)

    testROSConnection( );

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_LL_DRIVER)

    testWheelsControlRoutines( );

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_RAW_LL_DRIVE)

    testRawWheelsControlRoutine( );

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_SPEED_LIMIT_CALIB )

    testSpeedLimitsCalibrationRoutine( );

#elif ( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_ENCODER )

    testEncoderCommonRoutine( );

#elif ( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_RC )

    testRemoteControlRoutine( );

#elif ( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_RC_ODOMETRY )

    testRemoteControlOdometryRoutine( );

#elif ( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_ODOMETRY )

    testOdometryRoutine( );

#elif ( MAIN_PROGRAM_ROUTINE ==PROGRAM_ROUTINE_TEST_STEER_ANGL_SEND)

    testRemoteControlOdometryRoutine( );

#elif ( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_RESET_ODOMETRY )

    testResetOdometryRoutine( );

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

#elif( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_SPEED_CS )

    testSpeedCS( );

#elif( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_SPEED_FILTER )

    testSpeedFilter( );

#elif( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_UART_CS)

    testUARTControl( );

#elif( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_ESC_CALIBRATION )

    testDrivingWheelsESCCalibration( );


#elif( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_BUTTON_STATE)

    testButtonRoutine( );

#elif( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_LIGHT )

    testLightRoutine( );

#elif( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_LED_MATRIX )

    testLedMatrixRoutine( );

#endif
}

#endif /* INCLUDE_TESTS_H_ */
