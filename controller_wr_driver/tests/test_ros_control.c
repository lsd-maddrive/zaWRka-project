#include <tests.h>
#include <lld_odometry.h>
#include <lld_control.h>
#include <ros_protos.h>

/*
 * @brief   Test odometry, speed and steering control via ROS
 * @note    Frequency = 50 Hz
*/
void testRosRoutineControl( void )
{
    ros_driver_init( NORMALPRIO );

    lldOdometryInit( );
    driveSteerCSInit( );

    odometryValue_t         test_x_pos          = 0;
    odometryValue_t         test_y_pos          = 0;
    odometryValue_t         test_tetta_deg      = 0;

    odometrySpeedValue_t    test_speed_mps      = 0;
    odometryRawSpeedValue_t test_enc_speed_rps  = 0;

    controlValue_t          test_ros_steer_cntr = 0;
    controlValue_t          test_ros_speed_cntr = 0;


    while( 1 )
    {

//        test_ros_steer_cntr = getRosSteerPrct( );
//        test_ros_speed_cntr = getRosSpeedPrct( );


        driveSteerCSSetPosition  (  test_ros_steer_cntr );
        lldControlSetDrMotorPower(  test_ros_speed_cntr );

        test_enc_speed_rps  = lldGetOdometryRawSpeedRPS( );
        test_speed_mps      = lldGetOdometryObjSpeedMPS( );
        test_x_pos          = lldGetOdometryObjX( OBJ_DIST_M );
        test_y_pos          = lldGetOdometryObjY( OBJ_DIST_M );
        test_tetta_deg      = lldGetOdometryObjTettaDeg( );

        ros_driver_send_pose( test_x_pos, test_y_pos, test_tetta_deg /*, test_speed_mps */);

        chThdSleepMilliseconds( 20 );

    }


}
