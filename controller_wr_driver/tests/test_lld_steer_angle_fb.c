#include <tests.h>
#include <lld_steer_angle_fb.h>
#include <lld_control.h>

#define STEER_FB_MATLAB
//#define ADC_CHECK
#define TEST_STEER_LPF

static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0,
  .cr2 = 0,
  .cr3 = 0
};

/*
 * @brief   Test for routine of getting steering angle
 * @note    There are 2 options:
 *          * send data to Matlab
 *          * send data to Terminal
*/
void testSteerAngleSendData( void )
{
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );    // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );    // RX

    lldSteerAngleFBInit( );
    lldControlInit( );

    steerAngleRawValue_t    test_raw_steer          = 0;
    steerAngleRawValue_t    test_mean_raw_steer     = 0;
    steerAngleRawValue_t    test_lpf_raw_steer      = 0;

    steerAngleRadValue_t    test_rad_angle          = 0;
    steerAngleDegValue_t    test_deg_angle          = 0;


    controlValue_t          test_steer_cntrl        = 0;
    controlValue_t          test_delta_steer_cntr   = 10;

#ifdef STEER_FB_MATLAB
    char                    steer_matlab_start  = 0;
    uint8_t                 steer_start_flag    = 0;
#endif

    while( true )
    {

        test_raw_steer      = lldGetSteerAngleRawADC( );
        test_mean_raw_steer = lldGetSteerAngleFiltrMeanRawADC( );
        test_lpf_raw_steer  = lldGetSteerAngleFiltrLPFRawADC( );

        palToggleLine( LINE_LED1 );

        test_rad_angle      = lldGetSteerAngleRad( );
        test_deg_angle      = lldGetSteerAngleDeg( );



#ifdef STEER_FB_TERMINAL
        char rc_data = sdGetTimeout( &SD7, TIME_IMMEDIATE );

        switch( rc_data )
        {
          case 'a':     // turn max right
            lldControlSetSteerMotorPower( -100 );
            break;
          case 's':     // center
            lldControlSetSteerMotorPower( 0 );
            break;
          case 'd':     // turn max left
            lldControlSetSteerMotorPower( 100 );
            break;
          case 'q':
            test_steer_cntrl += test_delta_steer_cntr;
            break;
          case 'w':
            test_steer_cntrl -= test_delta_steer_cntr;
            break;

          default:
            ;
        }

        test_steer_cntrl = CLIP_VALUE( test_steer_cntrl, CONTROL_MIN, CONTROL_MAX );
        lldControlSetSteerMotorPower( test_steer_cntrl );

#ifdef ADC_CHECK

        chprintf( (BaseSequentialStream *)&SD7, "CONTROL:(%d)\tADC_RAW:(%d)\tMEAN_RAW:(%d)\n\r",
                  test_steer_cntrl, test_raw_steer, test_mean_raw_steer );
#endif

#ifdef ANGLE_CHECK

        chprintf( (BaseSequentialStream *)&SD7, "CONTROL:(%d)\tRAD:(%d)\tDEG:(%d)\n\r",
                  test_steer_cntrl, (int)(test_rad_angle * 10), (int)test_deg_angle );
#endif
        chThdSleepMilliseconds( 300 );
#endif

#ifdef STEER_FB_MATLAB
        steer_matlab_start = sdGetTimeout( &SD7, TIME_IMMEDIATE );

        if( steer_matlab_start == 'p' ) steer_start_flag = 1;

        if( steer_start_flag == 1)
        {
            palToggleLine( LINE_LED3 );
            sdWrite(&SD7, (uint8_t*) &test_raw_steer, 2);

#ifdef  TEST_STEER_MEAN_FILTER
            sdWrite(&SD7, (uint8_t*) &test_mean_raw_steer, 2);
#endif

#ifdef  TEST_STEER_LPF
            sdWrite(&SD7, (uint8_t*) &test_lpf_raw_steer, 2);
#endif
        }

        chThdSleepMilliseconds( 10 );
#endif

    }

}

/*
 * @brief   Control steering wheels to get angle
 * @note    Control ONLY steering wheels
*/
void testSteerAngleDetection( void )
{
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );    // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );    // RX

    lldControlInit( );

    while(1)
    {
        char rc_data = sdGet( &SD7 );
        switch( rc_data )
        {
             case 'a':      // turn right
               lldControlSetSteerMotorPower( -100 );
               break;
             case 'q':      // center
               lldControlSetSteerMotorPower( 0 );
               break;
             case 'z':      // turn left
               lldControlSetSteerMotorPower( 100 );
               break;
        }

        chThdSleepMilliseconds( 200 );
    }


}

