#ifndef INCLUDE_DRIVE_CS_H_
#define INCLUDE_DRIVE_CS_H_

#include <lld_steer_angle_fb.h>
#include <lld_control.h>
#include <lld_odometry.h>

typedef float   controllerRate_t;
typedef float   controllerError_t;
typedef float   controllerResponse_t;
typedef float   csControlCSValue_t;

typedef struct{
  controllerRate_t  kp;
  controllerRate_t  ki;
  controllerRate_t  kd;
  controllerRate_t  kr;

  controllerError_t integSaturation;
  controllerError_t proptDeadZone;

} pidControllerContext_t;

/**
 * @brief       Initialization of units that are needed for steering CS
 * @note        lldControl and lldSteerAngleFB are used
 */
void driverCSInit( tprio_t prio );

/**
 * @brief       Set parameters for Steering controller
 */
void driveSteerCSSetParam( pidControllerContext_t steer_param );

/**
 * @brief       Set parameters for Speed controller
 */
void driveSpeedCSSetParam( pidControllerContext_t speed_forward_param, pidControllerContext_t speed_backward_param );


/**
 * @brief       Control system for steering wheels
 * @param       value of angle in degrees [ -25; 25 ]
 * @note        max left    =>     25 |  100
 *              center      =>     0  |   0
 *              max_right   =>    -25 | -100
 */
void driveSteerCSSetPosition( steerAngleDegValue_t input_angl_deg );

/**
 * @brief       Get control value for steering driver
 */
controlValue_t driveSteerGetControlVal( void );

/**
 * @brief       Control system for speed
 * @param       value of speed [m/s]
 */
void driveSpeedCSSetSpeed ( float input_speed );

/**
 * @brief       Get control value for speed driver
 */
controlValue_t driveSpeedGetControlVal ( void );

/*
 * @brief       Get value of reference speed
 * @note        Only for debugging
 */
float driveSpeedGetGlobalRefSpeed( void );

/*
 * @brief       Reset all components for PID-controller
 */
void driverResetCS( void );

/*
 * @brief       The Control system is dis/enable
 *              true  - enable
 *              false - disable
 */
void driverIsEnableCS( bool permition );


#endif /* INCLUDE_DRIVE_CS_H_ */
