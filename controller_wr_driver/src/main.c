#include <common.h>
#include <chprintf.h>

#include "tests.h"

static void testsRoutines( void );

int main(void)
{
    chSysInit();
    halInit();

    #if (MAIN_PROGRAM_ROUTINE != PROGRAM_ROUTINE_MASTER)
        testsRoutines();
    #else
        mainUnitsInit( );
        mainControlTask( );
    #endif
}


static void testsRoutines( void )
{
#if (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_LL_DRIVER)
    testWheelsControlRoutines( );
#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_RAW_LL_DRIVE)
    testRawWheelsControlRoutine( );

#elif ( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_ENCODER )
    testEncoderCommonRoutine( );
#elif ( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_ODOMETRY )
    testOdometryRoutine( );
#elif ( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_ODOMETRY_RC )
    testRCOdodmetry( ); 
#elif ( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_X_DIST_ODOMETRY )
    testXdistanceOdometry( );
#elif ( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_RESET_ODOMETRY )
    testResetOdometryRoutine( );

#elif( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_STEER_ANGL_SEND )
    testSteerAngleSendData( );
#elif( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_STEERING_CS )
    testSteeringCS( );
#elif( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_SPEED_CS )
    testSpeedCS( );
#elif( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_SPEED_FILTER )
    testSpeedFilter( );
#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_SPEED_LIMIT_CALIB )
    testSpeedLimitsCalibrationRoutine( );

#elif( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_UART_CS)
    testUARTControl( );

#elif ( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_RC )
    testRemoteControlRoutine( );

#elif( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_LIGHT )
    testLightRoutine( );
#elif( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_LED_MATRIX )
    testLedMatrixRoutine( );

#elif( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_BUTTON_STATE)
    testButtonRoutine( );

#elif( MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_LINK_CONTROL )
    testLinkControl( );
#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_LINK_ADC_CALIB)
    testLinkADCCalib( );
#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_LINK)
    testLinkConnection( );
#endif
}
