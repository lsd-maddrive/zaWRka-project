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
 *	Not set yet, just sending
 */
void ros_driver_send_state( int8_t state );

typedef struct
{
	/*
	 * Cb returned arguments:
	 * 		speed - Speed task (-0.5;0.5) [m/s]
	 * 		steer - Steering task (-25;25) [deg]
	 */
	void (*cmd_cb)( float speed, float steer );

	/*
	 * It is better to use structure with parameters
	 */
	void (*set_steer_params_cb)( float k_left, float k_right );

	void (*reset_odometry_cb)( void );

} ros_driver_cb_ctx_t;

/*
 * Use this function to get clean (default) cb_ctx first!
 */
ros_driver_cb_ctx_t ros_driver_get_new_cb_ctx( void );

/*
 * Put filled cb_ctx pointer to this to set new callbacks
 * Setting NULL resets all callbacks to default behaviour (no callbacks)
 */
void ros_driver_set_cb_ctx( ros_driver_cb_ctx_t *ctx );



#ifdef __cplusplus
}
#endif

#endif // ROS_PROTOS_H_
