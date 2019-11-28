#include <tests.h>
#include <lld_steer_angle_fb.h>
#include <lld_control.h>

#define STEER_FB_TERMINAL
// #define ADC_CHECK
#define ANGLE_CHECK
// #define ANGLE_CHECK

//#define STEER_FB_MATLAB

#ifdef STEER_FB_MATLAB
static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};
#endif

/*
 * @brief   Test for routine of getting steering angle
 * @note    There are 2 options:
 *          * send data to Matlab (STEER_FB_MATLAB)
 *          * send data to Terminal (STEER_FB_TERMINAL)
*/
void testSteerAngleSendData( void )
{
#ifdef STEER_FB_MATLAB
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX
#endif

    lldSteerAngleFBInit( );
    lldControlInit( );
    debug_stream_init( );

    dbgprintf("Start steering test!\n");

#ifdef ADC_CHECK
    uint16_t                test_raw_steer          = 0;
    uint16_t                test_filtr_raw_steer    = 0;
#endif
#ifdef ANGLE_CHECK
    steerAngleRadValue_t    test_rad_angle          = 0;
    steerAngleDegValue_t    test_deg_angle          = 0;
    uint16_t                test_filtr_raw_steer    = 0;
#endif
    controlValue_t          test_steer_cntrl        = 0;
    controlValue_t          test_delta_steer_cntr   = 5;

#ifdef STEER_FB_MATLAB
    uint8_t                 steer_start_flag    = 0;
    uint16_t                matlab_time         = 0;
#endif

    systime_t   time = chVTGetSystemTimeX();
    while( true )
    {
#ifdef ADC_CHECK
        test_raw_steer      = lldGetSteerAngleRawADC( );
        test_filtr_raw_steer  = lldGetSteerAngleFiltrRawADC( );
#endif
#ifdef ANGLE_CHECK
        test_rad_angle      = lldGetSteerAngleRad( );
        test_deg_angle      = lldGetSteerAngleDeg( );
        test_filtr_raw_steer  = lldGetSteerAngleFiltrRawADC( );
#endif
#ifdef STEER_FB_MATLAB
        char rc_data = sdGetTimeout( &SD7, TIME_IMMEDIATE );
#else
        char rc_data = sdGetTimeout( &SD3, TIME_IMMEDIATE );
#endif
        switch( rc_data )
        {
          case 'a':     // turn right
            test_steer_cntrl    = -20;
            break;
          case 's':     // center
            test_steer_cntrl    = 20;
            break;
          case 'd':     // turn left
            lldControlSetSteerMotorPower( 100 );
            break;
          case 'q':
            test_steer_cntrl += test_delta_steer_cntr;
            break;
          case 'w':
            test_steer_cntrl -= test_delta_steer_cntr;
            break;
#ifdef STEER_FB_MATLAB
          case 'p':
            steer_start_flag = 1;
            break;
#endif
          case ' ':
            test_steer_cntrl  = 0;
            break;

          default:
            ;
        }

        test_steer_cntrl = CLIP_VALUE( test_steer_cntrl, CONTROL_MIN, CONTROL_MAX );
        lldControlSetSteerMotorPower( test_steer_cntrl );

#ifdef ADC_CHECK
        dbgprintf( "C:(%d)\tA_RAW:(%d)\tMEAN_RAW:(%d)\n\r",
                  test_steer_cntrl, test_raw_steer, test_filtr_raw_steer );

        time = chThdSleepUntilWindowed( time, time + MS2ST( 100 ) );
#endif
#ifdef ANGLE_CHECK

        dbgprintf( "CONTROL:(%d)\tRAD:(%d)\tDEG:(%d)\tRAW:(%d)\n\r",
                  test_steer_cntrl, (int)(test_rad_angle * 10),
                  (int)test_deg_angle, test_filtr_raw_steer );

        time = chThdSleepUntilWindowed( time, time + MS2ST( 100 ) );
#endif
#ifdef STEER_FB_MATLAB
        if( steer_start_flag == 1)
        {
            sdWrite( &SD7, (uint8_t*) &matlab_time, 2);
            sdWrite( &SD7, (uint8_t*) &test_raw_steer, 2);
            sdWrite( &SD7, (uint8_t*) &test_filtr_raw_steer, 2);
            matlab_time += 10;
        }

        time = chThdSleepUntilWindowed( time, time + MS2ST( 10 ) );
#endif
    }
}


