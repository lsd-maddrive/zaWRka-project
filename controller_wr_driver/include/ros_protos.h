#ifndef ROS_PROTOS_H_
#define ROS_PROTOS_H_

/***********/
/*** ROS ***/
/***********/

void ros_driver_init( tprio_t prio );
void ros_driver_send_odometry( int32_t value );
void ros_driver_send_odom_speed( float value );
void ros_driver_send_steering( float steer_angle );
/*
 * x - [meters]
 * y - [meters]
 * dir - [degree]
 */
void ros_driver_send_pose( float x, float y, float dir );

#endif // ROS_PROTOS_H_
