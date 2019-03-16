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
    driverCSInit( NORMALPRIO );
#ifdef STEER_CS_MATLAB
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX
#else
    debug_stream_init( );
#endif

    steerAngleDegValue_t    steer_feedback      = 0;

    steerAngleDegValue_t    steer_pos           = 0;

    steerAngleDegValue_t    steer_pos_delta     = 25;
    controlValue_t          steer_control_prct  = 0;

#ifdef STEER_CS_MATLAB
    uint8_t                 steer_start_flag    = 0;
    int16_t                 matlab_steer_pos    = 0;
    int16_t                 matlab_steer_fb_ang = 0;
#endif

#ifdef STEER_CS_TERMINAL
    uint32_t                show_count          = 0;

#endif
    while( 1 )
    {
#ifdef STEER_CS_TERMINAL
        show_count += 1;
        char rc_data    = sdGetTimeout( &SD3, TIME_IMMEDIATE );
#elif STEER_CS_MATLAB
        char rc_data    = sdGetTimeout( &SD7, TIME_IMMEDIATE );
#endif
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
           matlab_steer_fb_ang = (int)( steer_feedback * 100 );
           matlab_steer_pos  = (int)steer_pos;
           sdWrite(&SD7, (uint8_t*) &matlab_steer_fb_ang, 2);
           sdWrite(&SD7, (uint8_t*) &matlab_steer_pos, 2);
       }

       chThdSleepMilliseconds( 10 );
#endif
    }
}

//#define SPEED_CS_MATLAB
#ifdef SPEED_CS_MATLAB
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
      driverCSInit( NORMALPRIO );
#ifdef SPEED_CS_MATLAB
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    int16_t matlab_cntrl    = 0;
    int16_t matlab_speed    = 0;
    int16_t matlab_revs     = 0;

    int16_t matlab_get_cntr = 0;

    int16_t matlab_rare_speed = 0;
    int16_t matlab_lpf_speed  = 0;

    char    matlab_start  = 0;
    uint8_t matlab_start_flag    = 0;
#else
    debug_stream_init( );
#endif

    float                test_speed_ref     = 0;
    float                test_speed_delta   = 0.05;
    float                check_glob_ref_sp  = 0;

    odometrySpeedValue_t test_speed_mps     = 0;
    odometrySpeedValue_t test_speed_lpf     = 0;
    controlValue_t       test_speed_cntrl   = 0;

    while( 1 )
    {
      lldControlSetSteerMotorPower( 0 );

#ifdef SPEED_CS_MATLAB
      char rc_data    = sdGetTimeout( &SD7, TIME_IMMEDIATE );
#else
      char rc_data    = sdGetTimeout( &SD3, TIME_IMMEDIATE );
#endif
      switch( rc_data )
      {
        case 'a':           // forward
          test_speed_ref   += test_speed_delta;
          break;
        case 's':           // backward
          test_speed_ref   -= (1 * test_speed_delta);
          break;
        case 'd':           // backward
          test_speed_ref   -= test_speed_delta;
          break;

#ifdef SPEED_CS_MATLAB
        case 'p':
          matlab_start_flag = 1;
          break;
#endif
        case ' ':
          test_speed_ref = 0;
          break;

        default:
          ;
      }
      test_speed_mps    = lldGetOdometryObjSpeedMPS( );
      test_speed_lpf    = lldOdometryGetLPFObjSpeedMPS( );

      test_speed_cntrl  = driveSpeedGetControlVal( );

      driveSpeedCSSetSpeed( test_speed_ref );

      lldControlSetSteerMotorPower( 0 );


#ifdef SPEED_CS_MATLAB
      if( matlab_start_flag == 1)
      {
          matlab_speed          = (int)( test_speed_mps * 100 );
          matlab_lpf_speed      = (int)( test_speed_lpf * 1000 );
          matlab_cntrl          = (int)( test_speed_ref * 1000 );
          matlab_get_cntr       = (int)( test_speed_cntrl);
          sdWrite(&SD7, (uint8_t*) &matlab_cntrl, 2);
//          sdWrite(&SD7, (uint8_t*) &matlab_get_cntr, 2);
          sdWrite(&SD7, (uint8_t*) &matlab_lpf_speed, 2);
      }
      chThdSleepMilliseconds( 10 );
#else
      dbgprintf( "C:(%d)]\tREF:(%d)\tReal_v:(%d)\n\r",
                 test_speed_cntrl,
                 (int)(test_speed_ref * 100),
                 (int)(test_speed_lpf * 100) );

      chThdSleepMilliseconds( 100 );
#endif
    }
}


/*
 * @brief   Test LPF for speed control system
*/
void testSpeedFilter( void )
{
    driverCSInit( NORMALPRIO );
#ifdef SPEED_CS_MATLAB
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    int32_t matlab_cntrl    = 0;
    int32_t matlab_speed    = 0;
    int32_t matlab_revs     = 0;

    int32_t matlab_rare_speed = 0;
    int32_t matlab_lpf_speed  = 0;

    char    matlab_start  = 0;
    uint8_t matlab_start_flag    = 0;
 #else
    debug_stream_init( );
 #endif

    float                test_speed_ref     = 0;
    float                test_speed_delta   = 0.1;

    odometrySpeedValue_t test_speed_mps     = 0;
    odometrySpeedValue_t test_speed_lpf     = 0;


     while( 1 )
     {
       lldControlSetSteerMotorPower( 0 );

 #ifdef SPEED_CS_MATLAB
       char rc_data    = sdGetTimeout( &SD7, TIME_IMMEDIATE );
 #else
       char rc_data    = sdGetTimeout( &SD3, TIME_IMMEDIATE );
 #endif
       switch( rc_data )
       {
         case 'a':           // forward
           test_speed_ref   += test_speed_delta;
           break;
         case 's':           // backward
           test_speed_ref   -= test_speed_delta;
           break;
         case 'd':           // backward
           test_speed_ref   -= (2 * test_speed_delta);
           break;
 #ifdef SPEED_CS_MATLAB
         case 'p':
           matlab_start_flag = 1;
           break;
 #endif
         case ' ':
           test_speed_ref = 0;
           break;

         default:
           ;
       }
       test_speed_mps    = lldGetOdometryObjSpeedMPS( );
       test_speed_lpf    = lldOdometryGetLPFObjSpeedMPS( );

       driveSpeedCSSetSpeed( test_speed_ref );

       lldControlSetSteerMotorPower( 0 );


 #ifdef SPEED_CS_MATLAB
       if( matlab_start_flag == 1)
       {
           matlab_speed          = (int)( test_speed_mps * 100 );
           matlab_lpf_speed      = (int)( test_speed_lpf * 100 );

           sdWrite(&SD7, (uint8_t*) &matlab_speed, 2);
           sdWrite(&SD7, (uint8_t*) &matlab_lpf_speed, 2);
       }
       chThdSleepMilliseconds( 10 );
 #else
       dbgprintf( "REF:(%d)\tReal_v:(%d)\n\r",
                  (int)(test_speed_ref * 100),
                  (int)(test_speed_lpf * 100) );

       chThdSleepMilliseconds( 100 );
 #endif
     }

}