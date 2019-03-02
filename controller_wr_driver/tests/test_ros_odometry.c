#include <tests.h>
#include <lld_steer_angle_fb.h>
#include <lld_encoder.h>
#include <lld_odometry.h>
#include <ros_protos.h>

/*
 * @brief   Test odometry via ROS
 * @note    Frequency = 50 Hz
*/
void testRoutineROSOdometry( void )
{
    ros_driver_init( NORMALPRIO );
//    lldSteerAngleFBInit( );
    lldOdometryInit( );

    odometryValue_t         test_x_pos      = 0;
    odometryValue_t         test_y_pos      = 0;
    odometryValue_t         test_tetta_deg  = 0;

    while( 1 )
    {
        test_x_pos      = lldGetOdometryObjX( OBJ_DIST_M );
        test_y_pos      = lldGetOdometryObjY( OBJ_DIST_M );
        test_tetta_deg  = lldGetOdometryObjTettaDeg( );

        ros_driver_send_pose( test_x_pos, test_y_pos, test_tetta_deg, 0, 0 );

        chThdSleepMilliseconds( 20 );
    }

}
