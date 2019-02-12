#include <tests.h>
#include <encoder.h>
#include <odometry.h>

static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};


void testOdometryRoutine( void )
{
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    encoderInit( );

    float                   wheel_speed_mps     = 0;
    float                   test_speed_x             = 0;
    float                   test_speed_y             = 0;
    float                   test_speed_teta          = 0;
    float                   test_pos_x               = 0;
    float                   test_pos_y               = 0;
    float                   test_pos_teta            = 0;

    uint32_t                counter             = 0;

    chprintf( (BaseSequentialStream *)&SD7, "TEST ODOMETRY\n\r" );

    while( 1 )
    {
        counter += 1;

        wheel_speed_mps         = getEncWheelSpeedMPS( );
        test_speed_teta         = getObjSpeedChangingOrientation( wheel_speed_mps, 1.57 );
        test_pos_teta           = getObjAngleOrientation( wheel_speed_mps, 1.57 );
        test_speed_x            = getObjSpeedChangingX( wheel_speed_mps );
        test_pos_x              = getObjPosX( );
        test_speed_y            = getObjSpeedChangingY( wheel_speed_mps );
        test_pos_y              = getObjPosY( );

        if( counter == 20 )
        {
            chprintf( (BaseSequentialStream *)&SD7, "Teta:(%d)\tX:(%d)\tY:(%d)\n\r\t",
                      (int)test_pos_teta, (int)(test_pos_x*100), (int)(test_pos_y*100) );

            counter = 0;
        }
    }
}
