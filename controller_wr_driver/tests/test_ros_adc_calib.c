#include <tests.h>
#include <lld_odometry.h>
#include <drive_cs.h>

#include <ros_protos.h>

controlValue_t          test_ros_steer_cntr = 0;
controlValue_t          test_ros_speed_cntr = 0;

virtual_timer_t         ros_checker_vt;

static void ros_is_dead_cb( void *arg )
{
    arg = arg; 
    test_ros_speed_cntr = 0;
    test_ros_steer_cntr = 0;
    dbgprintf( "ROS is dead\n\r" );
}

void ros_alive( void )
{
    palToggleLine( LINE_LED1 ); // just to check
    chVTSet( &ros_checker_vt, MS2ST( 500 ), ros_is_dead_cb, NULL );
}

void cntrl_handler (float speed, float steer)
{
    systime_t ros_time = chVTGetSystemTimeX();
    test_ros_speed_cntr = speed;
    test_ros_steer_cntr = steer;
    dbgprintf( "Time:%d\n\r", (int)(ros_time * 1000.0 / CH_CFG_ST_FREQUENCY) );
    ros_alive( );
}

/**
 * @brief   ADC calibration process
 * @note    Frequency = 50 Hz
*/
void testRosRoutineADCCalib( void )
{
    ros_driver_cb_ctx_t cb_ctx      = ros_driver_get_new_cb_ctx();
    cb_ctx.raw_cmd_cb               = cntrl_handler;

    ros_driver_init( NORMALPRIO, &cb_ctx );

    lldControlInit();
    lldSteerAngleFBInit();

    debug_stream_init();

    chVTObjectInit(&ros_checker_vt);

    steerAngleRawValue_t    raw_adc_value       = 0;

    uint32_t                print_cntr          = 0;

    systime_t time              = chVTGetSystemTimeX();
    systime_t send_period_ms    = 20;

    while( 1 )
    {
        print_cntr += 1;

        lldControlSetDrMotorPower( test_ros_speed_cntr );
        lldControlSetSteerMotorPower( test_ros_steer_cntr );

        raw_adc_value  = lldGetSteerAngleFiltrRawADC();
        ros_driver_send_raw_adc( raw_adc_value );

        time = chThdSleepUntilWindowed( time, time + MS2ST( send_period_ms ) );

    }
}
