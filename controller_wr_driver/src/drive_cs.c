#include <tests.h>
#include <drive_cs.h>

pidControllerContext_t steerPIDparam = {
  .kp               = 0,
  .ki               = 0.1,
  .kd               = 0,
  .integSaturation  = 30,
  .proptDeadZone    = 2
};

/***    Not used for now, I guess will be used after testing    ***/
// pidControllerContext_t speedPIDparam = {
//   .kp               = 80,
//   .ki               = 0.1,
//   .kd               = 0,
//   .integSaturation  = 15,
//   .proptDeadZone    = 0,
//   .kr               = 0
// };

pidControllerContext_t  f_speedPIDparam = {
  .kp               = 50,
  .ki               = 1.5,
  .kd               = 0,
  .integSaturation  = 60,
  .proptDeadZone    = 0.09,
  .kr               = 0
};

int32_t     brake_cntr_value    = 30;

pidControllerContext_t  b_speedPIDparam = {
  .kp               = 40,
  .ki               = 0.4,
  .kd               = 0,
  .integSaturation  = 30,
  .proptDeadZone    = 0,
  .kr               = 0
};

/*============================================================================*/
/* CONTROLLER PARAMETRS                                                       */
/*============================================================================*/

#define STEER_LEFT_BUST_K   3.15
#define STEER_LEFT_BUST_B   (0)

#define STEER_RIGHT_BUST_K  (3.05)
#define STEER_RIGHT_BUST_B  (0)

/************************************/

#define STEER_MAX_LIMIT_LEFT    25
#define STEER_MAX_LIMIT_RIGHT   (-25)

#define SPEED_MAX_MPS           3
#define SPEED_MIN_MPS           (-3)

/***  reference steering angle in degrees  ***/
static steerAngleDegValue_t   steer_angl_deg_ref    = 0;

/***      Variables for Steering PID Controller     ***/
// static controllerError_t prev_steer_angl_deg_err    = 0;
// static controllerError_t steer_angl_deg_err         = 0;
// static controllerError_t steer_angl_deg_dif         = 0;
// static controllerError_t steer_angl_deg_intg        = 0;

/***     reference value of speed [m/s]     ***/
static odometrySpeedValue_t speed_ref               = 0;

/***      Variables for Speed PID Controller        ***/
static controllerError_t prev_speed_err             = 0;
static controllerError_t speed_err                  = 0;
static controllerError_t speed_dif                  = 0;
//static controllerError_t speed_intg                 = 0;
static controllerError_t f_speed_intg               = 0;
static controllerError_t f_cntr_intg                = 0;
static controllerError_t b_speed_intg               = 0;
static controllerError_t b_cntr_intg                = 0;

/***    Generated control value for lld***/
controlValue_t          steer_cntl_prc              = 0;
controlValue_t          speed_cntrl_prc             = 0;


bool                    permition_flag              = false;

/**
 * @brief       Set parameters for Steering controller
 */
void driveSteerCSSetParam( pidControllerContext_t steer_param )
{
    steerPIDparam.kp                = steer_param.kp;
    steerPIDparam.ki                = steer_param.ki;
    steerPIDparam.kd                = steer_param.kd;
    steerPIDparam.integSaturation   = steer_param.integSaturation;
    steerPIDparam.proptDeadZone     = steer_param.proptDeadZone;
}

/**
 * @brief       Set parameters for Speed controller
 */
void driveSpeedCSSetParam( pidControllerContext_t speed_forward_param, pidControllerContext_t speed_backward_param )
{
    f_speedPIDparam.kp                = speed_forward_param.kp;
    f_speedPIDparam.ki                = speed_forward_param.ki;
    f_speedPIDparam.kd                = speed_forward_param.kd;
    f_speedPIDparam.integSaturation   = speed_forward_param.integSaturation;
    f_speedPIDparam.proptDeadZone     = speed_forward_param.proptDeadZone;

    b_speedPIDparam.kp                = speed_backward_param.kp;
    b_speedPIDparam.ki                = speed_backward_param.ki;
    b_speedPIDparam.kd                = speed_backward_param.kd;
    b_speedPIDparam.integSaturation   = speed_backward_param.integSaturation;
    b_speedPIDparam.proptDeadZone     = speed_backward_param.proptDeadZone;
}

/**
 * @brief       Control system for steering wheels
 * @param       value of angle in degrees [ -25; 25 ]
 * @note        max left    =>     25 |  100
 *              center      =>     0  |   0
 *              max_right   =>    -25 | -100
 */
void driveSteerCSSetPosition( csControlCSValue_t input_angl_deg )
{
    input_angl_deg = CLIP_VALUE( input_angl_deg, STEER_MAX_LIMIT_RIGHT, STEER_MAX_LIMIT_LEFT );

    steer_angl_deg_ref = input_angl_deg;
}

/**
 * @brief       Get control value for steering driver
 */
controlValue_t driveSteerGetControlVal ( void )
{
    return steer_cntl_prc;
}

/**
 * @brief       Control system for speed
 * @param       value of speed [m/s]
 */
void driveSpeedCSSetSpeed ( csControlCSValue_t input_speed )
{
    input_speed = CLIP_VALUE( input_speed, SPEED_MIN_MPS, SPEED_MAX_MPS );

    speed_ref = input_speed;
}

/**
 * @brief       Get control value for speed driver
 */
controlValue_t driveSpeedGetControlVal ( void )
{
    return speed_cntrl_prc;
}


static THD_WORKING_AREA(waController, 256); // 128 - stack size
static THD_FUNCTION(Controller, arg)
{
    arg = arg; // to avoid warnings

    while( 1 )
    {
        systime_t   th_time     = chVTGetSystemTimeX();

        if( !permition_flag )
        {
            chThdSleepUntilWindowed( th_time, th_time + MS2ST(CONTROL_SYSTEMS_PERIOD_MS) );
            continue;
        }

        /***    Steering CS    ***/
        // steerAngleDegValue_t steer_angl_deg      =   lldGetSteerAngleDeg( );

        // steer_angl_deg_err  =   steer_angl_deg_ref - steer_angl_deg;
        // steer_angl_deg_dif  =   steer_angl_deg_err - prev_steer_angl_deg_err;
        // steer_angl_deg_intg +=  steer_angl_deg_err;

        // steer_angl_deg_intg = CLIP_VALUE( steer_angl_deg_intg, -steerPIDparam.integSaturation, steerPIDparam.integSaturation );

        // if( abs( steer_angl_deg_err ) <= steerPIDparam.proptDeadZone ) steer_angl_deg_intg = 0;

        if( steer_angl_deg_ref >= 0 )   // left
           steer_cntl_prc  = ( steer_angl_deg_ref * STEER_LEFT_BUST_K + STEER_LEFT_BUST_B); // + steer_angl_deg_intg * steerPIDparam.ki;
        else if( steer_angl_deg_ref < 0 )
           steer_cntl_prc  = ( steer_angl_deg_ref * STEER_RIGHT_BUST_K + STEER_RIGHT_BUST_B); // + steer_angl_deg_intg * steerPIDparam.ki;

        // prev_steer_angl_deg_err = steer_angl_deg_err;

        // -> Not required as it is done inside
        // steer_cntl_prc = CLIP_VALUE( steer_cntl_prc, CONTROL_MIN, CONTROL_MAX );

        lldControlSetSteerMotorPower( steer_cntl_prc );

        /*******************************************/

        /***    Speed CS    ***/
        odometrySpeedValue_t speed_obj_mps   = lldOdometryGetLPFObjSpeedMPS( );

        speed_err       =  speed_ref - speed_obj_mps;
        speed_dif       =  speed_err - prev_speed_err;
        prev_speed_err  = speed_err;

        if( speed_ref >= 0 )
        {
            if( speed_ref == 0 )
            {
                if( speed_err >= 0 )
                {
                    if( speed_err <= f_speedPIDparam.proptDeadZone )
                    {
                        f_cntr_intg    = 0;
                        speed_cntrl_prc = 0;
                    }
                    else
                    {
                        speed_cntrl_prc = brake_cntr_value;
                    }
                }
                else if( speed_err < 0 )
                {
                    if( speed_err >= (-f_speedPIDparam.proptDeadZone) )
                    {
                        f_speed_intg    = 0;
                        speed_cntrl_prc = 0;
                    }
                    else
                    {
                        speed_cntrl_prc = -brake_cntr_value;
                    }
                }
            }
            else if (speed_ref <= 0.1)
            {
                f_speedPIDparam.kp = 300;
                f_speedPIDparam.ki = 0.9;
                f_speedPIDparam.integSaturation = 50;

                f_speed_intg     += f_speedPIDparam.ki * speed_err;
                b_speed_intg     = 0;
                f_cntr_intg      =  f_speed_intg;
                f_cntr_intg      =  CLIP_VALUE( f_cntr_intg, -f_speedPIDparam.integSaturation, f_speedPIDparam.integSaturation );

                speed_cntrl_prc = f_speedPIDparam.kp * speed_err + f_speedPIDparam.kr * speed_ref + f_cntr_intg + f_speedPIDparam.kd * speed_dif;
            }
            else
            {
              f_speed_intg     += f_speedPIDparam.ki * speed_err;
              b_speed_intg     = 0;
              f_cntr_intg      =  f_speed_intg;
              f_cntr_intg      =  CLIP_VALUE( f_cntr_intg, -f_speedPIDparam.integSaturation, f_speedPIDparam.integSaturation );

              speed_cntrl_prc = f_speedPIDparam.kp * speed_err + f_speedPIDparam.kr * speed_ref + f_cntr_intg + f_speedPIDparam.kd * speed_dif;
            }
        }
        else if( speed_ref < 0 )
        {
            b_speed_intg      += b_speedPIDparam.ki * speed_err;
            f_speed_intg      =  0;
            b_cntr_intg       =  b_speed_intg;
            b_cntr_intg       =  CLIP_VALUE( b_cntr_intg, -b_speedPIDparam.integSaturation, b_speedPIDparam.integSaturation );
            speed_cntrl_prc = b_speedPIDparam.kp * speed_err + b_speedPIDparam.kr * speed_ref + b_cntr_intg + b_speedPIDparam.kd * speed_dif;
        }

        lldControlSetDrMotorPower( speed_cntrl_prc );
        chThdSleepUntilWindowed( th_time, th_time + MS2ST(CONTROL_SYSTEMS_PERIOD_MS) );
    }
}

/**
 * @brief       Get reference value of speed 
 * @note        Used for debugging used
 */
float driveSpeedGetGlobalRefSpeed( void )
{
    return speed_ref;
}

static bool             isInitialized = false;

/**
 * @brief       Initialization of units that are needed for steering CS
 * @note        lldControl and lldSteerAngleFB are used
 */
void driverCSInit( tprio_t prio )
{
    if( isInitialized )
      return;

    lldControlInit( );
    lldSteerAngleFBInit( );
    lldOdometryInit( );

    chThdCreateStatic( waController, sizeof(waController), prio , Controller, NULL );

    isInitialized = true;
}


/*
 * @brief       The Control system is dis/enable
 *              true  - enable
 *              false - disable
 */
void driverIsEnableCS( bool permition )
{
  permition_flag = permition;

}

/*
 * @brief       Reset all components for PID-controller
 */
void driverResetCS( void )
{
    speed_ref = steer_angl_deg_ref =  0;

    // prev_steer_angl_deg_err = 0;
    // steer_angl_deg_err      = 0;
    // steer_angl_deg_intg     = 0;

    steer_cntl_prc = 0;
    lldControlSetSteerMotorPower( steer_cntl_prc );

    prev_speed_err  = 0;
    speed_err       = 0;

    b_speed_intg    = 0;
    f_speed_intg    = 0;

    speed_cntrl_prc = 0;
    lldControlSetDrMotorPower( speed_cntrl_prc );
}
