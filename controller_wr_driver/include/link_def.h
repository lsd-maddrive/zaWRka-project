#ifndef MPROTO_DEFS_H_
#define MPROTO_DEFS_H_

#include <ch.h>
#include <stdint.h>
#include <params.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Send current wheel rotation state
 * @param
 * 			value - Wheel rotation position [rev]
 */
void mproto_driver_send_encoder_raw( float value );

/** 
 * @brief	Send current wheel rotation speed
 * @param
 * 			value - Wheel rotation speed [rev/s] 
 */
void mproto_driver_send_encoder_speed( float value );

/** 
 * @brief	Send current steering wheels angle
 * @param
 * 			steer_angle - Steering wheel angle [degree] 
 */
void mproto_driver_send_steering( float steer_angle );

/**
 * @brief	Send state in terms of position/speed
 * @param
 * 			x - Pose x coordinate [meters]
 * 			y - Pose y coordinate [meters]
 * 			dir - Direction angle [degree]
 * 			vx - Transltaion speed [m/s]
 * 			uz - Rotation speed [rad/s]
 */
void mproto_driver_send_pose( float x, float y, float dir, float vx, float uz );

/**
 * @brief	Send processing state of car
 * @param
 *			state - index of state
 *				0 - IDLE state
 *				1 - RUN state
 */
void mproto_driver_send_state( int8_t state );

/**
 * @brief	Send raw Steering value
 * @param
 * 			raw_adc - steering value [0; 4096] (12-bit)
 */
void mproto_driver_send_raw_steering( uint16_t raw_steering );

/***
 * WARNING! Never create this structure by yourself, use _driver_get_new_cb_ctx() instead
 */
typedef struct
{
	#define WR_INPUT_CMD_SPEED_LIMIT_MPS	1
	#define WR_INPUT_CMD_STEER_LIMIT_DEG	25
	/*
	 * Cb returned arguments:
	 * 		speed - Speed task (-0.5;0.5) [m/s]
	 * 		steer - Steering task (-25;25) [deg]
	 */
	void (*cmd_cb)( float speed, float steer );

	/*
	 * Cb returned arguments:
	 * 		speed - Speed task (-100;100) [%]
	 * 		steer - Steering task (-100;100) [%]
	 */
	void (*raw_cmd_cb)( float speed, float steer );

	/*
	 * It is better to use structure with parameters
	 */
	void (*set_odom_params_cb)( float k_left, float k_right );

	void (*reset_odometry_cb)( void );

	// control_params_setup_t (*get_control_params)( void );
	// void (*set_control_params_cb)( control_params_setup_t * );

} mproto_driver_cb_ctx_t;

/**
 * @brief	Get callback context
 * @return
 * 			<_cb_ctx_t> - structure with callback functions pointers
 */
mproto_driver_cb_ctx_t mproto_driver_get_new_cb_ctx( void );

/**
 * @brief	Initialize Link module
 * @param
 * 			prio - priority of polling thread
 * 			ctx  - pointer to callback context function
 * 					To get default <_cb_ctx_t> use _driver_get_new_cb_ctx()
 * 					Setting this field as NULL resets all callbacks
 */
void mproto_driver_init( tprio_t prio, mproto_driver_cb_ctx_t *ctx );

#ifdef __cplusplus
}
#endif

#endif // MPROTO_DEFS_H_
