#ifndef INCLUDE_TESTS_H_
#define INCLUDE_TESTS_H_

#include <common.h>

/****************************/
/*** Driver Control tests ***/
/****************************/

/*
 * @brief   Routine of low level driver control testing
 * @note    The routine has internal infinite loop
 * @note    Changing raw values of pwm dutycycle
 */
void testRawWheelsControlRoutine( void );

/*
 * @brief   Routine of low level driver control testing
 * @note    The routine has internal infinite loop
 */
void testWheelsControlRoutines( void );

/*********************/
/*** Encoder tests ***/
/*********************/

/**
 * @brief   Test ticks and revs counting, also direction detection
 * @note    chprintf works only when encoder is rotation
 */
void testEncoderCommonRoutine( void );

/**
 * @brief   Test speed detection [TPS, RPS, MPS, MPM, KPH]
 */
void testEncoderSpeedRoutine( void );


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






static inline void testsRoutines( void )
{
#if (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_LL_DRIVER)

    testWheelsControlRoutines( );

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_RAW_LL_DRIVE)

    testRawWheelsControlRoutine( );

#elif ( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_ENCODER )

    testEncoderCommonRoutine( );

#elif ( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_RC )

    testRemoteControlRoutine( );
#elif ( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_SPEED_ENCODER )

    testEncoderSpeedRoutine( );
#elif ( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_ODOMETRY )

    testOdometryRoutine( );
#endif
}

#endif /* INCLUDE_TESTS_H_ */
