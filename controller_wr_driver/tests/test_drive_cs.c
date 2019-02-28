#include <tests.h>
#include <drive_cs.h>

static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0,
  .cr2 = 0,
  .cr3 = 0
};



void testSteeringCS ( void )
{
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );    // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );    // RX

    driveSteerCSInit( );
//    lldSteerAngleFBInit();

    steerAngleDegValue_t    steer_feedback      = 0;
    steerAngleDegValue_t    steer_pos           = 0;
    controlValue_t          steer_control_prct   = 0;

    chprintf( (BaseSequentialStream *)&SD7, "TEST DRIVE CS\n\r" );

    while( 1 )
    {
        char rc_data    = sdGetTimeout( &SD7, TIME_IMMEDIATE );
        switch( rc_data )
        {
          case 'a':
            steer_pos   += 5;
            break;
          case 'd':
            steer_pos -= 5;
            break;
          default:
            ;
        }

        steer_feedback      = lldGetSteerAngleDeg( );
        steer_pos           = CLIP_VALUE( steer_pos, -25, 25 );
        steer_control_prct  = driveSteerCSSetPosition( steer_pos );
        lldControlSetSteerMotorPower( steer_control_prct );

        chprintf( (BaseSequentialStream *)&SD7, "TASK:(%d)\tDEG:(%d)\tCONTROL:(%d)\n\r",
                  (int)steer_pos, (int)(steer_feedback ), steer_control_prct );

        chThdSleepMilliseconds( 200 );
    }
}
