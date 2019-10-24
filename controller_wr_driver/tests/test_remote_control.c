#include <tests.h>
#include <remote_control.h>
#include <drive_cs.h>

#ifdef MATLAB_RC

static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};
#endif

/*
 * @brief   Routine of remote control testing
 * @note    The routine has internal infinite loop
 *          and strictly depended on time
 */
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
            lldControlSetSteerMotorRawPower( lldControlGetSteerMotorZeroPower() );
            lldControlSetDrMotorRawPower( lldControlGetDrMotorZeroPower() );
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
