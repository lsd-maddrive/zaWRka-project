#include <tests.h>
#include <lld_control.h>
#include <lld_odometry.h>

/*
 * @brief   Routine of low level driver control testing
 * @note    The routine has internal infinite loop
 * @note    Changing raw values of pwm dutycycle
 */
void testRawWheelsControlRoutine( void )
{
    debug_stream_init( );
    lldControlInit();

    controlValue_t  speed_values_delta  = 10;
    controlValue_t  speed_value         = lldControlGetDrMotorZeroPower();

    controlValue_t  steer_values_delta  = 10;
    controlValue_t  steer_value         = lldControlGetSteerMotorZeroPower();

    systime_t   time = chVTGetSystemTimeX( );
    while ( 1 )
    {
        char rcv_data = sdGetTimeout( &SD3, TIME_IMMEDIATE );
        switch ( rcv_data )
        {
            case 'a':   // Positive speed
              speed_value += speed_values_delta;
              break;

            case 's':   // Negative speed
              speed_value -= speed_values_delta;
              break;

            case ' ':   // Stop
              speed_value = lldControlGetDrMotorZeroPower();
              steer_value = lldControlGetSteerMotorZeroPower();
              break;

            case 'q':   // On the left
              steer_value += steer_values_delta;
              break;

            case 'w':   // On the right
              steer_value -= steer_values_delta;
              break;

            default:
                ;
        }

        lldControlSetDrMotorRawPower( speed_value );
        lldControlSetSteerMotorRawPower( steer_value );

        dbgprintf( "SP:(%d)\tST:(%d)\n\r", speed_value, steer_value );

        time = chThdSleepUntilWindowed( time, time + MS2ST( 100 ) );
    }
}

//#define SERIAL_SD7
#define DEBUG

#ifdef SERIAL_SD7
static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};
#endif

/*
 * @brief   Test steering and speed lld control
 * @note    Linear speed of object is also displayed
 */
void testWheelsControlRoutines( void )
{
#ifdef SERIAL_SD7
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    uint8_t     matlab_start_flag   = 0;
    uint16_t    matlab_speed_cmps   = 0;
#endif
#ifdef DEBUG
    debug_stream_init( );
#endif
    lldControlInit( );
    lldOdometryInit( );

    controlValue_t          speed_values_delta  = 5;
    controlValue_t          speed_value         = 0;

    controlValue_t          steer_values_delta  = 1;
    controlValue_t          steer_value         = 0;

    odometrySpeedValue_t    test_speed_lpf      = 0;

    systime_t   time = chVTGetSystemTimeX( );

    while ( 1 )
    {
#ifdef SERIAL_SD7
        char rcv_data = sdGetTimeout( &SD7, TIME_IMMEDIATE );
#endif
#ifdef DEBUG
        char rcv_data = sdGetTimeout( &SD3, TIME_IMMEDIATE );
#endif
        switch ( rcv_data )
        {
            case 'a':   // Positive speed
              speed_value += speed_values_delta;
              break;

            case 'd':   // Negative speed
              speed_value -= speed_values_delta;
              break;

            case 's':
              speed_value = 0;
              break;
#ifdef SERIAL_SD7
            case 'p':
              matlab_start_flag = 1;
              break;
#endif
            case 'q':   // On the left
              steer_value += steer_values_delta;
              break;

            case 'w':   // On the right
              steer_value -= steer_values_delta;
              break;

            case ' ':
              speed_value = 0;
              steer_value = 0;
              break;

            default:
               ;
        }

        test_speed_lpf  = lldOdometryGetLPFObjSpeedMPS( );

        lldControlSetDrMotorPower( speed_value );
        lldControlSetSteerMotorPower( steer_value );
#ifdef SERIAL_SD7
        if( matlab_start_flag == 1 )
        {
          matlab_speed_cmps = (int)( test_speed_lpf * 100 );
          sdWrite(&SD7, (uint8_t*) &speed_value, 2);
          sdWrite(&SD7, (uint8_t*) &matlab_speed_cmps, 2);
        }
        time = chThdSleepUntilWindowed( time, time + MS2ST( 10 ) );
#endif
#ifdef DEBUG
        dbgprintf( "SP(%d)\tR_SP:(%d)\tST(%d)\t\n\r",
                         speed_value, (int)( test_speed_lpf * 100 ), steer_value );

        time = chThdSleepUntilWindowed( time, time + MS2ST( 200 ) );
#endif
    }
}

#define VT_PRINT_PERIOD         100

/*
 * @brief   Test for speed max/min limits calibration
 * @note    show linear speed and control signal in %
 */
void testSpeedLimitsCalibrationRoutine( void )
{
    lldControlInit( );
    lldOdometryInit( );

    debug_stream_init( );

    controlValue_t          speed_values_delta  = 1;
    controlValue_t          speed_value         = 0;
    odometrySpeedValue_t    speed_mps     = 0;

    systime_t time = chVTGetSystemTimeX();

    while ( 1 )
    {
        time += MS2ST(VT_PRINT_PERIOD);

        char rcv_data = sdGetTimeout( &SD3, TIME_IMMEDIATE );
        switch ( rcv_data )
        {
            case 'a':   // Positive speed
              speed_value += speed_values_delta;
              break;

            case 's':   // Negative speed
              speed_value -= speed_values_delta;
              break;

           case ' ':    // Reset = Stop
             speed_value = 0;
             break;

           default:
              ;
       }
       speed_mps = lldGetOdometryObjSpeedMPS( );
       speed_value = CLIP_VALUE( speed_value, -100, 100 );

       lldControlSetDrMotorPower( speed_value );

       dbgprintf( "Speed:(%d)\tC:(%d)\n\r",
                 (int)(speed_mps * 100 ), speed_value );

       chThdSleepUntil(time);
   }
}

