#include <tests.h>
#include <drive_cs.h>
#include <lld_encoder.h>
 

#define STEER_CS_TERMINAL

static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};

/*
 * @brief   Test steering control system with feedback
 * @note    There are 2 options:
 *          - Show data in Terminal
 *          - Send limited number of data to Matlab
*/
void testSteeringCS ( void )
{
    driverCSInit( NORMALPRIO );
    driverIsEnableCS(true);
#ifdef STEER_CS_MATLAB
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX
#endif
    debug_stream_init( );

    steerAngleDegValue_t    steer_feedback      = 0;

    steerAngleDegValue_t    steer_pos           = 0;

    steerAngleDegValue_t    steer_pos_delta     = 1;
    controlValue_t          steer_control_prct  = 0;

#ifdef STEER_CS_MATLAB
    uint8_t                 steer_start_flag    = 0;
    int16_t                 matlab_steer_pos    = 0;
    int16_t                 matlab_steer_fb_ang = 0;
#endif

#ifdef STEER_CS_TERMINAL
    uint32_t                show_count          = 0;
#endif

    systime_t   time = chVTGetSystemTimeX( );
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

          case ' ':
            steer_pos = 0;
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
       time = chThdSleepUntilWindowed( time, time + MS2ST( 10 ) );
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

       time = chThdSleepUntilWindowed( time, time + MS2ST( 10 ) );
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
    driverIsEnableCS(true);

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
#ifdef SPEED_CS_MATLAB
    odometrySpeedValue_t test_speed_mps     = 0;
#endif
    odometrySpeedValue_t test_speed_lpf     = 0;
    controlValue_t       test_speed_cntrl   = 0;

    systime_t   time = chVTGetSystemTimeX( );
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
#ifdef SPEED_CS_MATLAB
      test_speed_mps    = lldGetOdometryObjSpeedMPS( );
#endif
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
      time =  chThdSleepUntilWindowed( time, time + MS2ST( 10 ) );
#else
      dbgprintf( "C:(%d)]\tREF:(%d)\tReal_v:(%d)\n\r",
                 test_speed_cntrl,
                 (int)(test_speed_ref * 100),
                 (int)(test_speed_lpf * 100) );

      time =  chThdSleepUntilWindowed( time, time + MS2ST( 100 ) );
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
#ifdef SPEED_CS_MATLAB
    odometrySpeedValue_t test_speed_mps     = 0;
#endif
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
#ifdef SPEED_CS_MATLAB
       test_speed_mps    = lldGetOdometryObjSpeedMPS( );
#endif
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

#define DEBUG_UART

// #define DEBUG_MIXED

/*
 * @brief   Test speed CS and steer CS via UART 7
*/
void testUARTControl( void )
{
#ifdef DEBUG_UART
    debug_stream_init( );
#endif

#ifdef DEBUG_MIXED
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX
    debug_stream_init( );
#endif
 

    driverCSInit( NORMALPRIO );
    driverIsEnableCS(true);

    int8_t                speed_cntrl   = 0;
    int8_t                steer_cntrl   = 0;
    const char            speed_lim     = 20;
    const char            steer_lim     = 25;

    systime_t   time = chVTGetSystemTimeX( );
    while( 1 )
    {
#ifdef DEBUG_UART
        int8_t rc_data    = sdGetTimeout( &SD3, TIME_IMMEDIATE );
#endif

#ifdef DEBUG_MIXED    
        int8_t rc_data    = sdGetTimeout( &SD7, TIME_IMMEDIATE );
#endif
        switch( rc_data )
        {
            case 's':   // ASCII 73
#ifdef DEBUG_UART
              speed_cntrl = sdGet( &SD3 );
#endif
#ifdef DEBUG_MIXED 
              speed_cntrl = sdGet( &SD7 );
#endif
              if( speed_cntrl == 0 ) speed_cntrl = 0;
              else if( speed_cntrl > speed_lim && speed_cntrl < -speed_lim)
                speed_cntrl = 0;
              // else
              //   speed_cntrl -= 48; // decode from ASCII to int
              break;

            case 'r':   // ASCII 72
#ifdef DEBUG_UART
              steer_cntrl = sdGet( &SD3 );
#endif
#ifdef DEBUG_MIXED 
              steer_cntrl = sdGet( &SD7 );
#endif
              if( steer_cntrl == 0 ) steer_cntrl = 0;
              else if( steer_cntrl > steer_lim && steer_cntrl < -steer_lim)
                steer_cntrl = 0;
              // else
              //   steer_cntrl -= 48; // decode from ASCII to int
              break;
        }

        steer_cntrl = CLIP_VALUE( steer_cntrl, -steer_lim, steer_lim );
        driveSteerCSSetPosition( steer_cntrl );
        speed_cntrl = CLIP_VALUE( speed_cntrl, -speed_lim, speed_lim );
        driveSpeedCSSetSpeed( speed_cntrl );    // m/s
#ifdef DEBUG_UART
        dbgprintf( "RC: %c\tST: %i\tSP: %i\n\r", rc_data, (steer_cntrl), speed_cntrl );
#endif

#ifdef DEBUG_MIXED
        dbgprintf( "RC: %c\tST: %i\tSP: %i\n\r", rc_data, (steer_cntrl), speed_cntrl );
#endif

        time = chThdSleepUntilWindowed( time, time + MS2ST( 20 ) );
    }
}
