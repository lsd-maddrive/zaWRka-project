#include <tests.h>
#include <drive_cs.h>

pidControllerContext_t steerPIDparam = {
  .kp               = 0,
  .ki               = 0.3,
  .kd               = 0,
  .integSaturation  = 100,
  .proptDeadZone    = 2
};

/***    Not used for now, I guess will be used after testing    ***/
pidControllerContext_t speedPIDparam = {
  .kp               = 80,
  .ki               = 0.1,
  .kd               = 0,
  .integSaturation  = 15,
  .proptDeadZone    = 0,
  .kr               = 0
};

pidControllerContext_t  f_speedPIDparam = {
  .kp               = 150,
  .ki               = 2,
  .kd               = 0,
  .integSaturation  = 70,
  .proptDeadZone    = 0,
  .kr               = 0
};

pidControllerContext_t  b_speedPIDparam = {
  .kp               = 50,
  .ki               = 0.1,
  .kd               = 0,
  .integSaturation  = 10,
  .proptDeadZone    = 0,
  .kr               = 0
};

#if 0
/************************************/
/***    GPT Configuration Zone    ***/
/************************************/
#define gpt_cs_Freq     10000
static  GPTDriver       *gptDriver = &GPTD3;
/************************************/
#endif
/************************************/
/***    CONTROLLER PARAMETRS      ***/
/************************************/

#define STEER_LEFT_BUST_K   3.652
#define STEER_LEFT_BUST_B   (0)

#define STEER_RIGHT_BUST_K  (2.9641)
#define STEER_RIGHT_BUST_B  (0)


/************************************/

#define STEER_MAX_LIMIT_LEFT    25
#define STEER_MAX_LIMIT_RIGHT   (-25)

#define SPEED_MAX_MPS           3
#define SPEED_MIN_MPS           (-3)

/***        data from steer feedback       ***/
static steerAngleDegValue_t   steer_angl_deg        = 0;
/***  reference steering angle in degrees  ***/
static steerAngleDegValue_t   steer_angl_deg_ref    = 0;

/***      Variables for Steering PID Controller     ***/
static controllerError_t prev_steer_angl_deg_err    = 0;
static controllerError_t steer_angl_deg_err         = 0;
static controllerError_t steer_angl_deg_dif         = 0;
static controllerError_t steer_angl_deg_intg        = 0;

/***     speed [m/s] feedback               ***/
static odometrySpeedValue_t speed_obj_mps           = 0;
/***     reference value of speed [m/s]     ***/
static odometrySpeedValue_t speed_ref               = 0;

/***      Variables for Speed PID Controller        ***/
static controllerError_t prev_speed_err             = 0;
static controllerError_t speed_err                  = 0;
static controllerError_t speed_dif                  = 0;
static controllerError_t speed_intg                 = 0;
static controllerError_t f_speed_intg               = 0;
static controllerError_t f_cntr_intg                = 0;
static controllerError_t b_speed_intg               = 0;
static controllerError_t b_cntr_intg                = 0;

/***    Generated control value for lld***/
controlValue_t          steer_cntl_prc              = 0;
controlValue_t          speed_cntrl_prc             = 0;


float                   check_cntrl_val             = 0;

#define                 VT_PID_CALC_MS              5
static virtual_timer_t  pid_update_vt;

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
void driveSpeedCSSetParam( pidControllerContext_t speed_param )
{
    speedPIDparam.kp                = speed_param.kp;
    speedPIDparam.ki                = speed_param.ki;
    speedPIDparam.kd                = speed_param.kd;
    speedPIDparam.integSaturation   = speed_param.integSaturation;
    speedPIDparam.proptDeadZone     = speed_param.proptDeadZone;
}

/**
 * @brief       Control system for steering wheels
 * @param       value of angle in degrees [ -25; 25 ]
 * @note        max left    =>     25 |  100
 *              center      =>     0  |   0
 *              max_right   =>    -25 | -100
 */
void driveSteerCSSetPosition( steerAngleDegValue_t input_angl_deg )
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
void driveSpeedCSSetSpeed ( float input_speed )
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

#if 0
static void gptcb (GPTDriver *gptd)
{

    gptd = gptd;
/***    I-controller for Steering CS    ***/
    steer_angl_deg      =   lldGetSteerAngleDeg( );

    steer_angl_deg_err  =   steer_angl_deg_ref - steer_angl_deg;
    steer_angl_deg_dif  =   steer_angl_deg_err - prev_steer_angl_deg_err;
    steer_angl_deg_intg +=  steer_angl_deg_err;

    steer_angl_deg_intg = CLIP_VALUE( steer_angl_deg_intg, -steerPIDparam.integSaturation, steerPIDparam.integSaturation );
    if( abs( steer_angl_deg_err ) <= steerPIDparam.proptDeadZone ) steer_angl_deg_intg = 0;

    if( steer_angl_deg_ref >= 0 )   // left
       steer_cntl_prc  = ( steer_angl_deg_ref * STEER_LEFT_BUST_K + STEER_LEFT_BUST_B) + steer_angl_deg_intg * steerPIDparam.ki;
    else if( steer_angl_deg_ref < 0 )
       steer_cntl_prc  = ( steer_angl_deg_ref * STEER_RIGHT_BUST_K + STEER_RIGHT_BUST_B) + steer_angl_deg_intg * steerPIDparam.ki;

    prev_steer_angl_deg_err = steer_angl_deg_err;

    steer_cntl_prc = CLIP_VALUE( steer_cntl_prc, CONTROL_MIN, CONTROL_MAX );

    lldControlSetSteerMotorPower( steer_cntl_prc );

    /*******************************************/

/***    Controller for Speed CS    ***/
    speed_obj_mps   = lldOdometryGetLPFObjSpeedMPS( );

    speed_err       = speed_ref - speed_obj_mps;
    speed_dif       = speed_err - prev_speed_err;

    if( speed_ref >= 0 )
    {

        f_speed_intg      += speed_err;
        b_speed_intg      = 0;
        f_speed_intg      = CLIP_VALUE( f_speed_intg, -f_speedPIDparam.integSaturation, f_speedPIDparam.integSaturation );

        if( speed_ref == 0 ) speed_err      = 0;
        if( speed_err == 0 ) f_speed_intg   = 0;

        check_cntrl_val = f_speedPIDparam.kp * speed_err + f_speedPIDparam.kr * speed_ref + f_speedPIDparam.ki * f_speed_intg + f_speedPIDparam.kd * speed_dif;

        speed_cntrl_prc = f_speedPIDparam.kp * speed_err + f_speedPIDparam.kr * speed_ref + f_speedPIDparam.ki * f_speed_intg + f_speedPIDparam.kd * speed_dif;
    }
    else if( speed_ref < 0 )
    {
        b_speed_intg      += speed_err;
        f_speed_intg      = 0;
        b_speed_intg  = CLIP_VALUE( b_speed_intg, -b_speedPIDparam.integSaturation, b_speedPIDparam.integSaturation );

        check_cntrl_val = b_speedPIDparam.kp * speed_err + b_speedPIDparam.kr * speed_ref + b_speedPIDparam.ki * b_speed_intg + b_speedPIDparam.kd * speed_dif;

        speed_cntrl_prc = b_speedPIDparam.kp * speed_err + b_speedPIDparam.kr * speed_ref + b_speedPIDparam.ki * b_speed_intg + b_speedPIDparam.kd * speed_dif;
    }
    prev_speed_err = speed_err;

    speed_cntrl_prc = CLIP_VALUE( speed_cntrl_prc, CONTROL_MIN, CONTROL_MAX );
    lldControlSetDrMotorPower( speed_cntrl_prc );
}



static const GPTConfig gpt3cfg = {
  .frequency =  gpt_cs_Freq,
  .callback  =  gptcb,
  .cr2       =  0,
  .dier      =  0U
};
#endif

static void pid_update_vt_cb( void *arg)
{
    arg = arg; // to avoid warnings
    palToggleLine( LINE_LED2 );

/***    I-controller for Steering CS    ***/
    steer_angl_deg      =   lldGetSteerAngleDeg( );

    steer_angl_deg_err  =   steer_angl_deg_ref - steer_angl_deg;
    steer_angl_deg_dif  =   steer_angl_deg_err - prev_steer_angl_deg_err;
    steer_angl_deg_intg +=  steer_angl_deg_err;

    steer_angl_deg_intg = CLIP_VALUE( steer_angl_deg_intg, -steerPIDparam.integSaturation, steerPIDparam.integSaturation );
    if( abs( steer_angl_deg_err ) <= steerPIDparam.proptDeadZone ) steer_angl_deg_intg = 0;

    if( steer_angl_deg_ref >= 0 )   // left
       steer_cntl_prc  = ( steer_angl_deg_ref * STEER_LEFT_BUST_K + STEER_LEFT_BUST_B) + steer_angl_deg_intg * steerPIDparam.ki;
    else if( steer_angl_deg_ref < 0 )
       steer_cntl_prc  = ( steer_angl_deg_ref * STEER_RIGHT_BUST_K + STEER_RIGHT_BUST_B) + steer_angl_deg_intg * steerPIDparam.ki;

    prev_steer_angl_deg_err = steer_angl_deg_err;

    steer_cntl_prc = CLIP_VALUE( steer_cntl_prc, CONTROL_MIN, CONTROL_MAX );

    lldControlSetSteerMotorPower( steer_cntl_prc );

    /*******************************************/

/***    Controller for Speed CS    ***/
    speed_obj_mps   = lldOdometryGetLPFObjSpeedMPS( );

    speed_err       =  speed_ref - speed_obj_mps;
    speed_dif       =  speed_err - prev_speed_err;
//    speed_intg      += speed_err;

    if( speed_ref >= 0 )
    {
        f_speed_intg      += speed_err;
        b_speed_intg      = 0;
        f_cntr_intg      =  f_speedPIDparam.ki * f_speed_intg;
        f_speed_intg      =  CLIP_VALUE( f_speed_intg, -f_speedPIDparam.integSaturation, f_speedPIDparam.integSaturation );

        //        if( speed_ref == 0 ) speed_err = 0;
        speed_cntrl_prc = f_speedPIDparam.kp * speed_err + f_speedPIDparam.kr * speed_ref + f_cntr_intg + f_speedPIDparam.kd * speed_dif;
    }
    else if( speed_ref < 0 )
    {
        b_speed_intg      += speed_err;
        f_speed_intg      = 0;
        b_cntr_intg      =  b_speedPIDparam.ki * b_speed_intg;
        b_speed_intg      =  CLIP_VALUE( b_speed_intg, -b_speedPIDparam.integSaturation, b_speedPIDparam.integSaturation );
        if( speed_ref == 0 ) speed_err      = 0;
        speed_cntrl_prc = b_speedPIDparam.kp * speed_err + b_speedPIDparam.kr * speed_ref + b_cntr_intg + b_speedPIDparam.kd * speed_dif;
    }


//    if( speed_obj_mps == speed_ref ) speed_intg = 0;


#if 0
    if( speed_ref >= 0 )
    {

        f_speed_intg      += speed_err;
        b_speed_intg      = 0;
        f_speed_intg      = CLIP_VALUE( f_speed_intg, -f_speedPIDparam.integSaturation, f_speedPIDparam.integSaturation );

        if( speed_ref == 0 ) speed_err      = 0;
        if( speed_err == 0 ) f_speed_intg   = 0;

        check_cntrl_val = f_speedPIDparam.kp * speed_err + f_speedPIDparam.kr * speed_ref + f_speedPIDparam.ki * f_speed_intg + f_speedPIDparam.kd * speed_dif;

        speed_cntrl_prc = f_speedPIDparam.kp * speed_err + f_speedPIDparam.kr * speed_ref + f_speedPIDparam.ki * f_speed_intg + f_speedPIDparam.kd * speed_dif;
    }
    else if( speed_ref < 0 )
    {
        b_speed_intg      += speed_err;
        f_speed_intg      = 0;
        b_speed_intg      = CLIP_VALUE( b_speed_intg, -b_speedPIDparam.integSaturation, b_speedPIDparam.integSaturation );

        check_cntrl_val = b_speedPIDparam.kp * speed_err + b_speedPIDparam.kr * speed_ref + b_speedPIDparam.ki * b_speed_intg + b_speedPIDparam.kd * speed_dif;

        speed_cntrl_prc = b_speedPIDparam.kp * speed_err + b_speedPIDparam.kr * speed_ref + b_speedPIDparam.ki * b_speed_intg + b_speedPIDparam.kd * speed_dif;
    }
#endif
    prev_speed_err = speed_err;

    speed_cntrl_prc = CLIP_VALUE( speed_cntrl_prc, CONTROL_MIN, CONTROL_MAX );
    lldControlSetDrMotorPower( speed_cntrl_prc );

    chSysLockFromISR();
    // reset VT
    chVTSetI(&pid_update_vt, MS2ST( VT_PID_CALC_MS ), pid_update_vt_cb, NULL);
    chSysUnlockFromISR();

}

float driveSpeedGetGlobalFloatControl( void )
{
    return check_cntrl_val;
}
float driveSpeedGetGlobalRefSpeed( void )
{
    return speed_ref;
}



static bool             isInitialized = false;

/**
 * @brief       Initialization of units that are needed for steering CS
 * @note        lldControl and lldSteerAngleFB are used
 */
void driverCSInit( void )
{
    if( isInitialized )
      return;

    lldControlInit( );
    lldSteerAngleFBInit( );
    lldOdometryInit( );
#if 0
    /*** Start working GPT driver in asynchronous mode ***/
    gptStart( gptDriver, &gpt3cfg );
    uint32_t gpt_period = gpt_cs_Freq * 0.01;   // 10 ms => 100 Hz
    gptStartContinuous( gptDriver, gpt_period );
#endif
    chVTObjectInit(&pid_update_vt);
    chVTSet( &pid_update_vt, MS2ST( VT_PID_CALC_MS ), pid_update_vt_cb, NULL );

    isInitialized = true;
}
