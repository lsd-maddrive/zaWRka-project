#ifndef INCLUDE_DRIVE_CS_H_
#define INCLUDE_DRIVE_CS_H_

#include <lld_steer_angle_fb.h>
#include <lld_control.h>


typedef float   controllerRate_t;
typedef float   controllerError_t;
typedef float   controllerResponse_t;




typedef struct{
  controllerRate_t  kp;
  controllerRate_t  ki;
  controllerRate_t  kd;

} pidControllerContext_t;


extern pidControllerContext_t steerPIDparam;


/**
 * @brief       Initialization of units that are needed for steering CS
 * @note        lldControl and lldSteerAngleFB are used
 */
void driveSteerCSInit( void );

/**
 * @brief       Control system for steering wheels
 * @param       value of angle in degrees [ -25; 25 ]
 * @return      control value in percent  [ -100; 100 ]
 * @note        max left    =>     25
 *              center      =>     0
 *              max_right   =>    -25
 *
 *              max_left control =>
 */
controlValue_t driveSteerCSSetPosition( steerAngleDegValue_t input_angl_deg );


#endif /* INCLUDE_DRIVE_CS_H_ */
