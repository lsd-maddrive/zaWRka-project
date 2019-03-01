#include <tests.h>
#include <drive_cs.h>

static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0,
  .cr2 = 0,
  .cr3 = 0
};

#define STEER_CS_MATLAB

void testSteeringCS ( void )
{
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );    // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );    // RX

    driveSteerCSInit( );

    steerAngleDegValue_t    steer_feedback      = 0;
    int16_t                 steer_matlab_temp   = 0;
    steerAngleDegValue_t    steer_pos           = 0;
    int16_t                 steer_pos_temp      = 0;
    steerAngleDegValue_t    steer_pos_delta     = 25;
    controlValue_t          steer_control_prct  = 0;

#ifdef STEER_CS_MATLAB
    uint8_t                 steer_start_flag    = 0;
#endif

#ifdef STEER_CS_TERMINAL
    uint32_t                show_count          = 0;

    chprintf( (BaseSequentialStream *)&SD7, "TEST DRIVE CS\n\r" );
#endif
    while( 1 )
    {
#ifdef STEER_CS_TERMINAL
        show_count += 1;
#endif
        char rc_data    = sdGetTimeout( &SD7, TIME_IMMEDIATE );
        switch( rc_data )
        {
          case 'a': // left
            steer_pos   += steer_pos_delta;
            break;
          case 'd': // right
            steer_pos   -= steer_pos_delta;
            break;
#ifdef STEER_CS_MATLAB
          case 'p': // start measuring
            steer_start_flag = 1;
            break;
#endif
          default:
            ;
        }

        steer_feedback      = lldGetSteerAngleDeg( );
        steer_pos           = CLIP_VALUE( steer_pos, -25, 25 );
        steer_control_prct  = driveSteerCSSetPosition( steer_pos );
        lldControlSetSteerMotorPower( steer_control_prct );
#ifdef STEER_CS_TERMINAL
       if( show_count == 10 )
       {
              chprintf( (BaseSequentialStream *)&SD7, "TASK:(%d)\tDEG:(%d)\tCONTROL:(%d)\n\r",
                          (int)steer_pos, (int)(steer_feedback * 100 ), steer_control_prct );
              show_count = 0;
       }
#endif

#ifdef STEER_CS_MATLAB
       if( steer_start_flag == 1)
       {
           palToggleLine( LINE_LED3 );
           steer_matlab_temp = (int)( steer_feedback * 100 );
           steer_pos_temp    = (int)steer_pos;
           sdWrite(&SD7, (uint8_t*) &steer_matlab_temp, 2);
           sdWrite(&SD7, (uint8_t*) &steer_pos_temp, 2);
       }


#endif
        chThdSleepMilliseconds( 20 );
    }
}
