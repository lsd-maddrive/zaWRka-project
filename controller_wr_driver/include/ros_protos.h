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
void ros_driver_send_encoder_speed( float value );
void ros_driver_send_steering( float steer_angle );
/*
 * x - [meters]
 * y - [meters]
 * dir - [degree]
 */
void ros_driver_send_pose( float x, float y, float dir );

#ifdef __cplusplus
}
#endif

#endif // ROS_PROTOS_H_
