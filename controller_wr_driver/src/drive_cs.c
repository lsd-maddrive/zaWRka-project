#include <tests.h>
#include <drive_cs.h>

pidControllerContext_t steerPIDparam = {
  .kp               = 0,
  .ki               = 0.3,
  .kd               = 0,
  .integSaturation  = 100,
  .proptDeadZone    = 2
};

pidControllerContext_t  f_speedPIDparam = {
  .kp               = 60,
  .ki               = 0.1,
  .kd               = 0,
  .integSaturation  = 100,
  .proptDeadZone    = 1,
  .kr               = 70
};

pidControllerContext_t  b_speedPIDparam = {
  .kp               = 100,
  .ki               = 0.1,
  .kd               = 0,
  .integSaturation  = 100,
  .proptDeadZone    = 1,
  .kr               = 250
};


/************************************/
/***    GPT Configuration Zone    ***/
/************************************/
#define gpt_cs_Freq     10000
static  GPTDriver       *gptDriver = &GPTD3;
/************************************/

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
static controllerError_t f_speed_intg                 = 0;
static controllerError_t b_speed_intg                 = 0;

/***    Generated control value for lld***/
controlValue_t          steer_cntl_prc              = 0;
controlValue_t          speed_cntrl_prc             = 0;

/**
 * @brief       Control system for steering wheels
 * @param       value of angle in degrees [ -25; 25 ]
 * @note        max left    =>     25 |  100
 *              center      =>     0  |   0
 *              max_right   =>    -25 | -100
 */
void driveSteerCSSetPosition( steerAngleDegValue_t input_angl_deg )
{
    palToggleLine( LINE_LED1 );
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
    speed_ref = input_speed;


}

/**
 * @brief       Get control value for speed driver
 */
controlValue_t driveSpeedGetControlVal ( void )
{
    return speed_cntrl_prc;
}

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
    speed_obj_mps   = lldGetOdometryObjSpeedMPS( );
//    if( abs(speed_obj_mps) > 2 * speed_ref ) speed_obj_mps = 0;

    speed_err       = speed_ref - speed_obj_mps;
    speed_dif       = speed_err - prev_speed_err;

    if( speed_ref >= 0 )
    {
      f_speed_intg      += speed_err;
      b_speed_intg      = 0;
      f_speed_intg  = CLIP_VALUE( f_speed_intg, -f_speedPIDparam.integSaturation, f_speedPIDparam.integSaturation );
      speed_cntrl_prc = f_speedPIDparam.kp * speed_err + f_speedPIDparam.kr * speed_ref + f_speedPIDparam.ki * f_speed_intg + f_speedPIDparam.kd * speed_dif;
    }
    else if( speed_ref < 0 )
    {
      b_speed_intg      += speed_err;
      f_speed_intg      = 0;
      b_speed_intg  = CLIP_VALUE( b_speed_intg, -b_speedPIDparam.integSaturation, b_speedPIDparam.integSaturation );
      speed_cntrl_prc = b_speedPIDparam.kp * speed_err + b_speedPIDparam.kr * speed_ref + b_speedPIDparam.ki * b_speed_intg + b_speedPIDparam.kd * speed_dif;
    }
    prev_speed_err = speed_err;

    speed_cntrl_prc = CLIP_VALUE( speed_cntrl_prc, CONTROL_MIN, CONTROL_MAX );
    lldControlSetDrMotorPower( speed_cntrl_prc );


}

float driveSpeedFInteg( void )
{
    return f_speed_intg;
}

float driveSpeedBInteg( void )
{
    return b_speed_intg;
}



static const GPTConfig gpt3cfg = {
  .frequency =  gpt_cs_Freq,
  .callback  =  gptcb,
  .cr2       =  0,
  .dier      =  0U
};

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

    /*** Start working GPT driver in asynchronous mode ***/
    gptStart( gptDriver, &gpt3cfg );
    uint32_t gpt_period = gpt_cs_Freq * 0.02;   // 20 ms => 50 Hz
    gptStartContinuous( gptDriver, gpt_period );

    isInitialized = true;
}
