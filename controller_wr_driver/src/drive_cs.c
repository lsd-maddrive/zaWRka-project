#include <tests.h>
#include <drive_cs.h>
#include <lld_steer_angle_fb.h>
#include <lld_control.h>

extern pidControllerContext_t steerPIDparam = {

  .kp = 5,
  .ki = 0.01,
  .kd = 0


};


static bool             isInitialized = false;

/**
 * @brief       Initialization of units that are needed for steering CS
 * @note        lldControl and lldSteerAngleFB are used
 */
void driveSteerCSInit( void )
{
  if( isInitialized )
    return;

  lldControlInit( );
  lldSteerAngleFBInit( );

  isInitialized = true;

}



#define STEER_MAX_LIMIT_LEFT    25
#define STEER_MAX_LIMIT_RIGHT   (-25)
#define STEER_DEADZONE          4

steerAngleDegValue_t steer_angl_deg        = 0;

static controllerError_t prev_steer_angl_deg_err    = 0;
static controllerError_t steer_angl_deg_err         = 0;
static controllerError_t steer_angl_deg_dif         = 0;
static controllerError_t steer_angl_deg_intg        = 0;

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
controlValue_t driveSteerCSSetPosition( steerAngleDegValue_t input_angl_deg )
{
    controlValue_t steer_cntl_prc   = 0;
    input_angl_deg = CLIP_VALUE( input_angl_deg, STEER_MAX_LIMIT_RIGHT, STEER_MAX_LIMIT_LEFT );


    steer_angl_deg      =   lldGetSteerAngleDeg( );

    steer_angl_deg_err  =   input_angl_deg - steer_angl_deg;
    steer_angl_deg_dif  =   steer_angl_deg_err - prev_steer_angl_deg_err;
    steer_angl_deg_intg +=  steer_angl_deg_err;

    if( abs( steer_angl_deg_err ) <= STEER_DEADZONE )
      steer_angl_deg_err = 0;

    if( steer_angl_deg_err == 0 ) steer_angl_deg_intg = 0;

    steer_cntl_prc = steer_angl_deg_err * steerPIDparam.kp +  steer_angl_deg_dif * steerPIDparam.ki + steer_angl_deg_intg * steerPIDparam.kd;

    prev_steer_angl_deg_err = steer_angl_deg_err;

    steer_cntl_prc = CLIP_VALUE( steer_cntl_prc, CONTROL_MIN, CONTROL_MAX );
    return steer_cntl_prc;

}


