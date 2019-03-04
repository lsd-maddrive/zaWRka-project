#include <tests.h>
#include <drive_cs.h>
#include <lld_encoder.h>


#define STEER_CS_TERMINAL

/*
 * @brief   Test steering control system with feedback
 * @note    There are 2 options:
 *          - Show data in Terminal
 *          - Send limited number of data to Matlab
*/
void testSteeringCS ( void )
{
    driverCSInit( );
    debug_stream_init( );


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

#endif
    while( 1 )
    {
#ifdef STEER_CS_TERMINAL
        show_count += 1;
#endif
        char rc_data    = sdGetTimeout( &SD3, TIME_IMMEDIATE );
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
        driveSteerCSSetPosition( steer_pos );

#ifdef STEER_CS_TERMINAL
       if( show_count == 10 )
       {
              dbgprintf( "TASK:(%d)\tDEG:(%d)\tCONTROL:(%d)\n\r",
                          (int)steer_pos, (int)(steer_feedback * 100 ), steer_control_prct );
              show_count = 0;
       }
       chThdSleepMilliseconds( 100 );
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

       chThdSleepMilliseconds( 200 );

#endif

    }
}

#define MATLAB_SPEED


#ifdef MATLAB_SPEED
static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};
#endif

/*
 * @brief   Test speed control system with feedback
*/
void testSpeedCS ( void )
{

#ifdef MATLAB_SPEED
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    int32_t matlab_cntrl    = 0;
    int32_t matlab_speed    = 0;
    int32_t matlab_revs     = 0;

    char    steer_matlab_start  = 0;
    uint8_t steer_start_flag    = 0;

#endif
    driverCSInit( );
    lldOdometryInit( );
    debug_stream_init( );

    float                test_speed_ref     = 0;
    odometrySpeedValue_t test_speed_mps     = 0;
    controlValue_t       test_speed_cntrl   = 0;
    float                test_speed_delta   = 0.1;

    float                b_ing              = 0;
    float                f_ing              = 0;

    odometryRawSpeedValue_t current_revs    = 0;
    odometryRawSpeedValue_t previous_revs   = 0;

    float                dist               = 0;

    while( 1 )
    {
      lldControlSetSteerMotorPower( 0 );

#ifdef MATLAB_SPEED
      char rc_data    = sdGetTimeout( &SD7, TIME_IMMEDIATE );
#else
      char rc_data    = sdGetTimeout( &SD3, TIME_IMMEDIATE );
#endif
      switch( rc_data )
      {
        case 'a': // left
          test_speed_ref   += test_speed_delta;
          break;
        case 'd': // right
          test_speed_ref   -= test_speed_delta;
          break;

#ifdef MATLAB_SPEED
        case 'p':
          steer_start_flag = 1;
          break;
#endif
        case ' ':
          test_speed_ref = 0;
          break;

        default:
          ;
      }
      test_speed_mps    = lldGetOdometryObjSpeedMPS( );
      current_revs      = lldGetEncoderRawRevs( );
      previous_revs     = lldOdometryGetPrevRevs( );
      dist              = lldGetOdometryObjDistance( OBJ_DIST_CM );

      f_ing             = driveSpeedFInteg( );
      b_ing             = driveSpeedBInteg( );


      driveSpeedCSSetSpeed( test_speed_ref );

      lldControlSetSteerMotorPower( 0 );

      test_speed_cntrl  = driveSpeedGetControlVal( );

#ifdef MATLAB_SPEED

      if( steer_start_flag == 1)
      {
          palToggleLine( LINE_LED3 );

          matlab_cntrl  = test_speed_cntrl;
          matlab_speed  = (int)( test_speed_mps * 100 );
          matlab_revs   = (int)( current_revs * 500 );

//          sdWrite(&SD7, (uint8_t*) &matlab_cntrl, 2);
          sdWrite(&SD7, (uint8_t*) &matlab_speed, 2);
      }
#else
      dbgprintf( "C:(%d)\tREF:(%d)\tReal_v:(%d)\tD:(%d)\tR:(%d)\n\r",
                 test_speed_cntrl, (int)(test_speed_ref * 100), (int)(test_speed_mps * 100),
                 (int)(dist * 10), (int)(current_revs * 10));

//      dbgprintf( "C:(%d)\tREF:(%d)\tReal_v:(%d)\tF_i:(%d)\tB_i:(%d)\n\r",
//                       test_speed_cntrl, (int)(test_speed_ref * 100), (int)(test_speed_mps * 100),
//                       (int)f_ing, (int)b_ing );
#endif
      chThdSleepMilliseconds( 10 );
    }
}
