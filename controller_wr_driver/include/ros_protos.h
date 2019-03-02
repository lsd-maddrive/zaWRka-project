#ifndef ROS_PROTOS_H_
#define ROS_PROTOS_H_

#include <ch.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/***********/
/*** ROS ***/
/***********/

void ros_driver_init( tprio_t prio );
void ros_driver_send_encoder_raw( int32_t value );

/* value - Encoder rotation speed [rev/s] */
void ros_driver_send_encoder_speed( float value );

void ros_driver_send_steering( float steer_angle );
/*
 * x - Pose x coordinate [meters]
 * y - Pose y coordinate [meters]
 * dir - Direction angle [degree]
 * vx - Transltaion speed [m/s]
 * uz - Rotation speed [rad/s]
 */
void ros_driver_send_pose( float x, float y, float dir, float vx, float uz );

/*
 * Cb returned arguments:
 * 		speed - Speed task (-100;100) [%]
 * 		steer - Steering task (-25;25) [deg]
 */
void ros_driver_set_control_cb( void (*cb_func)(float speed, float steer) );

#ifdef __cplusplus
}
#endif

#endif // ROS_PROTOS_H_
