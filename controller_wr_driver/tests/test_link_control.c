#include <tests.h>
#include <lld_odometry.h>
#include <drive_cs.h>

#include "link_def.h"

controlValue_t          test_steer_cntr = 0;
float                   test_speed_cntr = 0;

virtual_timer_t         checker_vt;

static void dead_cb( void *arg )
{
    arg = arg; 
    test_speed_cntr = 0;
    test_steer_cntr = 0;
    dbgprintf( "ROS is dead\n\r" );
}

void alive( void )
{
    palToggleLine( LINE_LED1 ); // just to check
    chVTSet( &checker_vt, MS2ST( 500 ), dead_cb, NULL );
}

void cntrl_handler (float speed, float steer)
{
    test_speed_cntr = speed;
    test_steer_cntr = steer;
    alive();
}

/*
 * @brief   Test odometry, speed and steering control via ROS
 * @note    Frequency = 50 Hz
*/
void testLinkControl( void )
{
    mproto_driver_cb_ctx_t cb_ctx      = mproto_driver_get_new_cb_ctx();
    cb_ctx.cmd_cb                   = cntrl_handler;
    cb_ctx.reset_odometry_cb        = lldResetOdometry;

    mproto_driver_init( NORMALPRIO, &cb_ctx );

    lldOdometryInit( );
    driverCSInit( NORMALPRIO );
    driverIsEnableCS( true );

    debug_stream_init( );

    chVTObjectInit(&checker_vt);

    odometryValue_t         test_x_pos          = 0;
    odometryValue_t         test_y_pos          = 0;
    odometryValue_t         test_tetta_deg      = 0;

    odometryRawSpeedValue_t test_enc_speed_rps  = 0;
    odometrySpeedValue_t    test_speed_radps    = 0;

    float                   test_speed_lpf_mps  = 0;
    uint32_t                print_cntr          = 0;

    systime_t time = chVTGetSystemTimeX();

    dbgprintf( "Start test!\n\r" );

    while( 1 )
    {
        print_cntr += 1;

        driveSteerCSSetPosition( test_steer_cntr );
        driveSpeedCSSetSpeed( test_speed_cntr );

        test_enc_speed_rps  = lldGetOdometryRawSpeedRPS( );
        test_speed_radps    = lldGetOdometryObjTettaSpeedRadPS( );

        test_speed_lpf_mps  = lldOdometryGetLPFObjSpeedMPS( );

        test_x_pos          = lldGetOdometryObjX( OBJ_DIST_M );
        test_y_pos          = lldGetOdometryObjY( OBJ_DIST_M );
        test_tetta_deg      = lldGetOdometryObjTettaDeg( );

        steerAngleDegValue_t steering_deg  = lldGetSteerAngleDeg();

        mproto_driver_send_encoder_speed( test_enc_speed_rps );
        mproto_driver_send_pose( test_x_pos, test_y_pos, test_tetta_deg, test_speed_lpf_mps, test_speed_radps );
        mproto_driver_send_steering( steering_deg );

        time = chThdSleepUntilWindowed( time, time + MS2ST( 20 ) );

        if ( print_cntr == 50 ) {
            dbgprintf( "Values:%d/%d\n\r", (int)(test_speed_cntr*10), (int)(test_steer_cntr) );
            print_cntr = 0;
        }
    }
}
