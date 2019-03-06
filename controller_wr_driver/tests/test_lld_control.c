#include <tests.h>
#include <lld_control.h>
#include <lld_odometry.h>

#include "chprintf.h"

/***************************************************/

static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};


void testRawWheelsControlRoutine( void )
{
    palSetLine( LINE_LED1 );
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    lldControlInit();

    controlValue_t  speed_values_delta  = 20;
    controlValue_t  speed_value         = 1520;

    controlValue_t  steer_values_delta  = 20;
    controlValue_t  steer_value         = 1620;


    chprintf( (BaseSequentialStream *)&SD7, "TEST RAW\n\r" );

    while ( 1 )
    {
        char rcv_data = sdGet( &SD7 );
        switch ( rcv_data )
        {
            case 'a':   // Positive speed
            speed_value += speed_values_delta;
            break;

            case 's':   // Negative speed
            speed_value -= speed_values_delta;
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

        speed_value = CLIP_VALUE( speed_value, 1160, 1920 );
        lldControlSetDrMotorRawPower( speed_value );

        steer_value = CLIP_VALUE( steer_value, 1200, 2040 );
        lldControlSetSteerMotorRawPower( steer_value );

        chprintf( (BaseSequentialStream *)&SD7, "Powers:\n\r\tSteer:(%d)\tSpeed:(%d)\n\r\t", steer_value, speed_value );

        chThdSleepMilliseconds( 100 );
    }

}

#define SERIAL_SD7
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
#else
    debug_stream_init( );
#endif
    debug_stream_init( );
    lldControlInit( );
    lldOdometryInit( );

    controlValue_t          speed_values_delta  = 1;
    controlValue_t          speed_value         = 0;

    controlValue_t          steer_values_delta  = 1;
    controlValue_t          steer_value         = 0;

    odometrySpeedValue_t    test_speed_lpf      = 0;


    while ( 1 )
    {
#ifdef SERIAL_SD7
        char rcv_data = sdGetTimeout( &SD3, TIME_IMMEDIATE );
        char matlab_data = sdGetTimeout( &SD7, TIME_IMMEDIATE );
        if( matlab_data == 'p' ) matlab_start_flag = 1;
#else
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
        dbgprintf( "C:(%d)\n\r", speed_value );
        speed_value     = CLIP_VALUE( speed_value, CONTROL_MIN, CONTROL_MAX );
        steer_value     = CLIP_VALUE( steer_value, CONTROL_MIN, CONTROL_MAX );
        test_speed_lpf  = lldOdometryGetLPFObjSpeedMPS( );


        lldControlSetDrMotorPower( speed_value );
        lldControlSetSteerMotorPower( steer_value );
#ifdef SERIAL_SD7
        if( matlab_start_flag == 1 )
        {
          matlab_speed_cmps = (int)( test_speed_lpf * 100 );
          sdWrite(&SD7, (uint8_t*) &matlab_speed_cmps, 2);
        }
//        chprintf( (BaseSequentialStream *)&SD7, "Speed(%d)\tSteer(%d)\n\r",
//                         speed_value, steer_value );
#else
        dbgprintf( "SP(%d)\tR_SP:(%d)\tST(%d)\t\n\r",
                         speed_value, (int)( test_speed_lpf * 100 ), steer_value );
#endif
        chThdSleepMilliseconds( 10 );
    }
}


void testWheelsSpeedControlRoutine( void )
{
    lldControlInit( );
    lldOdometryInit( );

    debug_stream_init( );

    controlValue_t          speed_values_delta  = 1;
    controlValue_t          speed_value         = 0;
    odometrySpeedValue_t    speed_mps     = 0;

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

           case ' ':
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

       chThdSleepMilliseconds( 100 );
   }

}
