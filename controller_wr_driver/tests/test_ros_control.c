#include <tests.h>
#include <lld_odometry.h>
//#include <lld_control.h>
#include <drive_cs.h>

#include <ros_protos.h>

controlValue_t          test_ros_steer_cntr = 0;
controlValue_t          test_ros_speed_cntr = 0;

//virtual_timer_t         ros_checker_vt;


void cntrl_handler (float speed, float steer)
{
    test_ros_speed_cntr = speed;
    test_ros_steer_cntr = steer;
}

/*
 * @brief   Test odometry, speed and steering control via ROS
 * @note    Frequency = 50 Hz
*/
void testRosRoutineControl( void )
{
    ros_driver_init( NORMALPRIO );

    ros_driver_set_control_cb( cntrl_handler );

    lldOdometryInit( );
    driverCSInit( );
    debug_stream_init( );

//    chVTSet( ros_checker_vt, MS2ST( 1000 ), )


    odometryValue_t         test_x_pos          = 0;
    odometryValue_t         test_y_pos          = 0;
    odometryValue_t         test_tetta_deg      = 0;

    odometrySpeedValue_t    test_speed_mps      = 0;
    odometryRawSpeedValue_t test_enc_speed_rps  = 0;
    odometrySpeedValue_t    test_speed_radps    = 0;

    float    test_steer_angl_deg = 0;
    uint32_t                print_cntr          = 0;

    while( 1 )
    {
        print_cntr += 1;
        driveSteerCSSetPosition( test_ros_steer_cntr );
        driveSpeedCSSetSpeed( test_ros_speed_cntr );

//        lldControlSetDrMotorPower(  test_ros_speed_cntr );

        test_enc_speed_rps  = lldGetOdometryRawSpeedRPS( );
        test_speed_mps      = lldGetOdometryObjSpeedMPS( );
        test_speed_radps    = lldGetOdometryObjTettaSpeedRadPS( );

        test_steer_angl_deg  = lldGetSteerAngleDeg( );

        test_x_pos          = lldGetOdometryObjX( OBJ_DIST_M );
        test_y_pos          = lldGetOdometryObjY( OBJ_DIST_M );
        test_tetta_deg      = lldGetOdometryObjTettaDeg( );

        ros_driver_send_encoder_speed( test_enc_speed_rps );
        ros_driver_send_pose( test_x_pos, test_y_pos, test_tetta_deg, test_speed_mps, test_speed_radps );

        if( print_cntr == 10 )
        {
          palToggleLine( LINE_LED2 );
          dbgprintf( "ST:(%d)\tSCS:(%d)\tANG:(%d)\n\r",
                     (int)test_ros_steer_cntr, driveSteerGetControlVal( ), (int)test_steer_angl_deg );

          print_cntr = 0;
        }

        chThdSleepMilliseconds( 20 );

    }


}
