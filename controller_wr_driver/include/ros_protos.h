#ifndef ROS_PROTOS_H_
#define ROS_PROTOS_H_

#include <ch.h>
#include <stdint.h>
#include <params.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* ROS 																		  */
/*============================================================================*/

/**
 * @brief	Send current wheel rotation state
 * @param
 * 			value - Wheel rotation position [rev]
 */
void ros_driver_send_encoder_raw( int32_t value );

/** 
 * @brief	Send current wheel rotation speed
 * @param
 * 			value - Wheel rotation speed [rev/s] 
 */
void ros_driver_send_encoder_speed( float value );

/** 
 * @brief	Send current steering wheels angle
 * @param
 * 			steer_angle - Steering wheel angle [degree] 
 */
void ros_driver_send_steering( float steer_angle );

/**
 * @brief	Send state in terms of position/speed
 * @param
 * 			x - Pose x coordinate [meters]
 * 			y - Pose y coordinate [meters]
 * 			dir - Direction angle [degree]
 * 			vx - Transltaion speed [m/s]
 * 			uz - Rotation speed [rad/s]
 */
void ros_driver_send_pose( float x, float y, float dir, float vx, float uz );

/**
 * @brief	Send processing state of car
 * @param
 *			state - index of state
 *				0 - IDLE state
 *				1 - RUN state
 */
void ros_driver_send_state( int8_t state );

/***
 * WARNING! Never create this structure by yourself, use ros_driver_get_new_cb_ctx() instead
 */
typedef struct
{
	#define ROS_INPUT_CMD_SPEED_LIMIT_MPS	1
	#define ROS_INPUT_CMD_STEER_LIMIT_DEG	25
	/*
	 * Cb returned arguments:
	 * 		speed - Speed task (-0.5;0.5) [m/s]
	 * 		steer - Steering task (-25;25) [deg]
	 */
	void (*cmd_cb)( float speed, float steer );

	/*
	 * It is better to use structure with parameters
	 */
	void (*set_odom_params_cb)( float k_left, float k_right );

	void (*reset_odometry_cb)( void );

	// control_params_setup_t (*get_control_params)( void );
	// void (*set_control_params_cb)( control_params_setup_t * );

} ros_driver_cb_ctx_t;

/**
 * @brief	Get callback context
 * @return
 * 			<ros_driver_cb_ctx_t> - structure with callback functions pointers
 */
ros_driver_cb_ctx_t ros_driver_get_new_cb_ctx( void );

/**
 * @brief	Initialize ROS module
 * @param
 * 			prio - priority of polling thread
 * 			ctx  - pointer to callback context function
 * 					To get default <ros_driver_cb_ctx_t> use ros_driver_get_new_cb_ctx()
 * 					Setting this field as NULL resets all callbacks
 */
void ros_driver_init( tprio_t prio, ros_driver_cb_ctx_t *ctx );

#ifdef __cplusplus
}
#endif

#endif // ROS_PROTOS_H_
