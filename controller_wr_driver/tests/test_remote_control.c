#include <tests.h>
#include <remote_control.h>
#include <lld_control.h>


static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};


void testRemoteControlRoutine( void )
{
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    lldControlInit( );
    remoteControlInit( 0 );

    pwmValue_t          rc_speed        =   0;
    pwmValue_t          rc_steer        =   0;
    icuControlValue_t   rc_steer_prt    =   0;
    icuControlValue_t   rc_speed_prt    =   0;
    bool                mode            =   false;

    uint32_t    show_counter = 0;

    chprintf( (BaseSequentialStream *)&SD7, "RC TEST\n\r" );

    systime_t time = chVTGetSystemTimeX();
    chprintf( (BaseSequentialStream *)&SD7, "Start_time:(%d)\n\r", time );
    while( 1 )
    {
        time += MS2ST(20); // Next deadline
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
            lldControlSetSteerMotorRawPower( 1600 );
            lldControlSetDrMotorRawPower( 1500 );
        }

        if( show_counter == 4)
        {
            chprintf( (BaseSequentialStream *)&SD7, "Steer:(%d)\tSpeed:(%d)\tSTEER_PNT:(%d)\tSPEED_PNT:(%d)\n\r\t",
                      rc_steer, rc_speed, rc_steer_prt, rc_speed_prt );
            show_counter = 0;
        }

        chThdSleepUntil(time);
    }
}

