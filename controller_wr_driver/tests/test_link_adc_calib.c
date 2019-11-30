#include <tests.h>
#include <lld_odometry.h>
#include <drive_cs.h>

#include "link_def.h"

static controlValue_t          test_steer_cntr = 0;
static controlValue_t          test_speed_cntr = 0;

static virtual_timer_t         checker_vt;

static void is_dead_cb( void *arg )
{
    arg = arg; 
    test_speed_cntr = 0;
    test_steer_cntr = 0;
    dbgprintf( "ROS is dead\n\r" );
}

static void alive( void )
{
    palToggleLine( LINE_LED1 ); // just to check
    chVTSet( &checker_vt, MS2ST( 2000 ), is_dead_cb, NULL );
}

static void cntrl_handler (float speed, float steer)
{
    test_speed_cntr = speed;
    test_steer_cntr = steer;
    alive();
}

/**
 * @brief   ADC calibration process
 * @note    Frequency = 50 Hz
*/
void testLinkADCCalib( void )
{
    mproto_driver_cb_ctx_t cb_ctx      = mproto_driver_get_new_cb_ctx();
    cb_ctx.raw_cmd_cb               = cntrl_handler;

    mproto_driver_init( NORMALPRIO, &cb_ctx );

    lldControlInit();
    lldSteerAngleFBInit();

    debug_stream_init();
    dbgprintf( "ADC calibration test start\n" );

    chVTObjectInit(&checker_vt);

    steerAngleRawValue_t    raw_adc_value       = 0;

    uint32_t                print_cntr          = 0;

    systime_t time              = chVTGetSystemTimeX();
    systime_t send_period_ms    = 20;

    while( 1 )
    {
        print_cntr += 1;

        lldControlSetDrMotorPower( test_speed_cntr );
        lldControlSetSteerMotorPower( test_steer_cntr );

        raw_adc_value  = lldGetSteerAngleFiltrRawADC();
        mproto_driver_send_raw_steering( raw_adc_value );

        time = chThdSleepUntilWindowed( time, time + MS2ST( send_period_ms ) );
    }
}
