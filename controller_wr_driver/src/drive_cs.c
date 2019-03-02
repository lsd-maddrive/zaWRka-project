#include <tests.h>
#include <drive_cs.h>
#include <lld_steer_angle_fb.h>
#include <lld_control.h>


pidControllerContext_t steerPIDparam = {

  .kp = 0,
  .ki = 0.3,
  .kd = 0
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

#define STEER_DEADZONE          2
#define STEER_SATURATION_LIMIT  100

/************************************/

#define STEER_MAX_LIMIT_LEFT    25
#define STEER_MAX_LIMIT_RIGHT   (-25)

/***        data from steer feedback       ***/
static steerAngleDegValue_t   steer_angl_deg        = 0;
/***  reference steering angle in degrees  ***/
static steerAngleDegValue_t   steer_angl_deg_ref    = 0;

/***      Variables for PID Controller     ***/
static controllerError_t prev_steer_angl_deg_err    = 0;
static controllerError_t steer_angl_deg_err         = 0;
static controllerError_t steer_angl_deg_dif         = 0;
static controllerError_t steer_angl_deg_intg        = 0;

/***    Generated control value for lld***/
static controlValue_t    steer_cntl_prc             = 0;

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
void driveSteerCSSetPosition( steerAngleDegValue_t input_angl_deg )
{
    input_angl_deg = CLIP_VALUE( input_angl_deg, STEER_MAX_LIMIT_RIGHT, STEER_MAX_LIMIT_LEFT );

    steer_angl_deg_ref = input_angl_deg;

    lldControlSetSteerMotorPower( steer_cntl_prc );
}

static void gptcb (GPTDriver *gptd)
{

    gptd = gptd;

    steer_angl_deg      =   lldGetSteerAngleDeg( );

    steer_angl_deg_err  =   steer_angl_deg_ref - steer_angl_deg;
    steer_angl_deg_dif  =   steer_angl_deg_err - prev_steer_angl_deg_err;

    steer_angl_deg_intg +=  steer_angl_deg_err;

    steer_angl_deg_intg = CLIP_VALUE( steer_angl_deg_intg, -STEER_SATURATION_LIMIT, STEER_SATURATION_LIMIT );

    if( abs( steer_angl_deg_err ) <= STEER_DEADZONE )
    {
      steer_angl_deg_intg = 0;
    }

   //    steer_cntl_prc = steer_angl_deg_err * steerPIDparam.kp + steer_angl_deg_intg * steerPIDparam.ki +  steer_angl_deg_dif * steerPIDparam.kd;

    if( steer_angl_deg_ref >= 0 )   // left
       steer_cntl_prc  = ( steer_angl_deg_ref * STEER_LEFT_BUST_K + STEER_LEFT_BUST_B) + steer_angl_deg_intg * steerPIDparam.ki;
    else if( steer_angl_deg_ref < 0 )
       steer_cntl_prc  = ( steer_angl_deg_ref * STEER_RIGHT_BUST_K + STEER_RIGHT_BUST_B) + steer_angl_deg_intg * steerPIDparam.ki;

    prev_steer_angl_deg_err = steer_angl_deg_err;

    steer_cntl_prc = CLIP_VALUE( steer_cntl_prc, CONTROL_MIN, CONTROL_MAX );

}

static const GPTConfig gpt3cfg = {
  .frequency =  gpt_cs_Freq,
  .callback  =  gptcb,
  .cr2       =  0,
  .dier      =  0U
};

static bool             isInitialized = false;

virtual_timer_t         ros_checker_vt;


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

    /*** Start working GPT driver in asynchronous mode ***/
    gptStart( gptDriver, &gpt3cfg );
    uint32_t gpt_period = gpt_cs_Freq * 0.02;   // 20 ms => 50 Hz
    gptStartContinuous( gptDriver, gpt_period );

    /***    Check ROS condition      ***/

//    chVTSet( ros_checker_vt, MS2ST())


    isInitialized = true;

}

void isROSalive( void )
{
//    chVTReset (*ros_checker_vt);


}





