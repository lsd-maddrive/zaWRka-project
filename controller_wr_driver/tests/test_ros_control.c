#include <tests.h>
#include <lld_odometry.h>
#include <drive_cs.h>

#include <ros_protos.h>

controlValue_t          test_ros_steer_cntr = 0;
float                   test_ros_speed_cntr = 0;

virtual_timer_t         ros_checker_vt;


static void ros_is_dead_cb( void *arg )
{
    test_ros_speed_cntr = 0;
    test_ros_steer_cntr = 0;

    chSysLockFromISR();
    chVTSet( &ros_checker_vt, MS2ST( 500 ), ros_is_dead_cb, NULL );
    chSysUnlockFromISR();
}

void ros_is_alive( void )
{
    palToggleLine( LINE_LED1 ); // just to check
    chVTSet( &ros_checker_vt, MS2ST( 500 ), ros_is_dead_cb, NULL );
}

void cntrl_handler (float speed, float steer)
{
    test_ros_speed_cntr = speed;
    test_ros_steer_cntr = steer;
}

void changeSteerParams( float min, float max )
{
    /* change limits */
}

control_params_setup_t get_esc_control_params( void )
{
    control_params_setup_t params;

    params.esc_min_dc_offset = SPEED_ZERO - SPEED_MIN;
    params.esc_max_dc_offset = SPEED_MAX - SPEED_ZERO;

    return params;
}


/*
 * @brief   Test odometry, speed and steering control via ROS
 * @note    Frequency = 50 Hz
*/
void testRosRoutineControl( void )
{
    ros_driver_cb_ctx_t cb_ctx      = ros_driver_get_new_cb_ctx();
    cb_ctx.cmd_cb                   = cntrl_handler;
    // cb_ctx.set_steer_params_cb      = changeSteerParams;
    cb_ctx.reset_odometry_cb        = lldResetOdomety;
    // cb_ctx.get_control_params       = get_esc_control_params;

    ros_driver_init( NORMALPRIO, &cb_ctx );

//    ros_driver_set_control_cb( cntrl_handler );

    lldOdometryInit( );
    driverCSInit( );
    debug_stream_init( );

    chVTObjectInit(&ros_checker_vt);
    chVTSet( &ros_checker_vt, MS2ST( 500 ), ros_is_dead_cb, NULL );


    odometryValue_t         test_x_pos          = 0;
    odometryValue_t         test_y_pos          = 0;
    odometryValue_t         test_tetta_deg      = 0;

    odometrySpeedValue_t    test_speed_mps      = 0;
    odometryRawSpeedValue_t test_enc_speed_rps  = 0;
    odometrySpeedValue_t    test_speed_radps    = 0;

    float                   test_steer_angl_deg = 0;
    float                   test_speed_lpf_mps  = 0;
    uint32_t                print_cntr          = 0;

    systime_t time = chVTGetSystemTimeX();
    while( 1 )
    {
        time += MS2ST(20);

        print_cntr += 1;

        driveSteerCSSetPosition( test_ros_steer_cntr );
        driveSpeedCSSetSpeed( test_ros_speed_cntr );

        test_enc_speed_rps  = lldGetOdometryRawSpeedRPS( );
        test_speed_mps      = lldGetOdometryObjSpeedMPS( );
        test_speed_radps    = lldGetOdometryObjTettaSpeedRadPS( );

        test_steer_angl_deg = lldGetSteerAngleDeg( );
        test_speed_lpf_mps  = lldOdometryGetLPFObjSpeedMPS( );

        test_x_pos          = lldGetOdometryObjX( OBJ_DIST_M );
        test_y_pos          = lldGetOdometryObjY( OBJ_DIST_M );
        test_tetta_deg      = lldGetOdometryObjTettaDeg( );

        ros_driver_send_encoder_speed( test_enc_speed_rps );
        ros_driver_send_pose( test_x_pos, test_y_pos, test_tetta_deg, test_speed_mps, test_speed_radps );

        if( print_cntr == 10 )
        {
          dbgprintf( "ST:(%d)\tANG:(%d)\tSP_R:(%d)\tSP:(%d)\n\r",
                     (int)test_ros_steer_cntr, (int)test_steer_angl_deg,
                     (int)( test_ros_speed_cntr * 100 ), (int)( test_speed_lpf_mps * 100 ) );

          print_cntr = 0;
        }

        chThdSleepUntil(time);
    }
}
