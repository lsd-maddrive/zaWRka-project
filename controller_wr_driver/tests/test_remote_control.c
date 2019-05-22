#include <tests.h>
#include <remote_control.h>
#include <lld_control.h>

#include <lld_odometry.h>
#include <lld_steer_angle_fb.h>

// #ifdef MATLAB_RC
static const SerialConfig sdcfg = {
  .speed = 38400,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};
// #endif

void testRemoteControlRoutine( void )
{
#ifdef MATLAB_RC
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX
#else
    debug_stream_init( );
#endif
    debug_stream_init( );
    lldControlInit( );
    remoteControlInit( NORMALPRIO );

    pwmValue_t          rc_speed        =   0;
    pwmValue_t          rc_steer        =   0;
    icuControlValue_t   rc_steer_prt    =   0;
    icuControlValue_t   rc_speed_prt    =   0;
    bool                mode            =   false;

    uint32_t    show_counter = 0;

    systime_t time = chVTGetSystemTimeX();

    while( 1 )
    {

        show_counter += 1;
        mode = rcModeIsEnabled();

        if( mode == true )
        {
            rc_speed        = rcGetSpeedDutyCycleValue( );
            rc_steer        = rcGetSteerDutyCycleValue( );
            rc_steer_prt    = rcGetSteerControlValue( );
            rc_speed_prt    = rcGetSpeedControlValue( );

            lldControlSetDrMotorPower( rc_speed_prt );
            lldControlSetSteerMotorPower( rc_steer_prt );

        }
        else
        {
            lldControlSetSteerMotorRawPower( STEER_NULL );
            lldControlSetDrMotorRawPower( 1510 );
        }

        if( show_counter == 20 )
        {
            dbgprintf( "ST:(%d)\tSP:(%d)\tST_PNT:(%d)\tSP_PNT:(%d)\n\r\t",
                      rc_steer, rc_speed, rc_steer_prt, rc_speed_prt );
            show_counter = 0;
        }

        time = chThdSleepUntilWindowed( time, time + MS2ST( 10 ) );
    }
}

#define UART

void testRemoteControlOdometryRoutine( void )
{
#ifdef UART
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX
#endif

#ifdef DEBUG
    debug_stream_init( );
#endif  
    lldControlInit( );
    remoteControlInit( NORMALPRIO );

    lldOdometryInit( );
    lldSteerAngleFBInit( );

    uint32_t    show_counter = 0;
    int16_t     speed_cmps   = 0;
    int16_t     steer_angl   = 0;

    int8_t   rc_steer_prt    =   0;
    int8_t   rc_speed_prt    =   0;
    bool                mode            =   false;

    uint8_t start = '#';

    systime_t time = chVTGetSystemTimeX();

    while( 1 )
    {
        show_counter += 1;
        mode = rcModeIsEnabled();

        speed_cmps = lldGetOdometryObjSpeedCMPS( );
        steer_angl = lldGetSteerAngleDeg( ); 

        if( mode == true )
        {
            rc_steer_prt    = rcGetSteerControlValue( );
            rc_speed_prt    = rcGetSpeedControlValue( );

            lldControlSetDrMotorPower( rc_speed_prt );
            lldControlSetSteerMotorPower( rc_steer_prt );
        }
        else
        {
            lldControlSetDrMotorPower( 0 );
            lldControlSetSteerMotorPower( 0 );
        }

        if( show_counter == 1 ) // 20 ms 
        {
#ifdef UART
            if( speed_cmps >= 127 ) speed_cmps = 127;
            else if (speed_cmps <= -127) speed_cmps = -127;
            sdWrite(&SD7, (uint8_t*) &start, 1);
            sdWrite(&SD7, (uint8_t*) &speed_cmps, 1);
            sdWrite(&SD7, (uint8_t*) &steer_angl, 1);
#endif 

#ifdef DEBUG 
            dbgprintf("SP:%d\tANG:%d\n\r", speed_cmps, steer_angl);
#endif 
            show_counter = 0; 
        }

        time = chThdSleepUntilWindowed( time, time + MS2ST( 20 ) );
    }

}