#include <tests.h>
#include <lld_control.h>


#define pwm1Freq        1000000
#define pwm1Period      10000           // 100 Hz

/* These values are only for driver, must be closed! */
#define STEER_PWM_MAX           1650 //1650    // Left
#define STEER_PWM_NULL          1140
#define STEER_PWM_MIN           500  //650     // Right

#define SPEED_MAX               1650
#define SPEED_NULL_FORWARD      1550
#define SPEED_ZERO              1500
#define SPEED_NULL_BACK         1450
#define SPEED_MIN               1350

/***  PWM configuration pins    ***/
/***  PE9 - Driving wheels      ***/
#define PE9_ACTIVE      PWM_OUTPUT_ACTIVE_HIGH
#define PE9_DISABLE     PWM_OUTPUT_DISABLED
#define drivePWMch      0
/***  PE11 - Steering wheels     ***/
#define PE11_ACTIVE     PWM_OUTPUT_ACTIVE_HIGH
#define PE11_DISABLE    PWM_OUTPUT_DISABLED
#define steerPWMch      1
/***  PE13, PE14 - not used     ***/
#define PE13_ACTIVE     PWM_OUTPUT_ACTIVE_HIGH
#define PE13_DISABLE    PWM_OUTPUT_DISABLED
#define PE14_ACTIVE     PWM_OUTPUT_ACTIVE_HIGH
#define PE14_DISABLE    PWM_OUTPUT_DISABLED

static  PWMDriver       *pwmDriver      = &PWMD1;

#define pwm1LineCh0     PAL_LINE( GPIOE, 9 )
#define pwm1LineCh1     PAL_LINE( GPIOE, 11 )


/*** Configuration structures ***/

PWMConfig pwm1conf = {
    .frequency = pwm1Freq,
    .period    = pwm1Period, /* 1/1000 s = 10 ms => 100 Hz
                             * PWM period = period/frequency [s] */
    .callback  = NULL,
    .channels  = {
                  {.mode = PE9_ACTIVE,      .callback = NULL},
                  {.mode = PE11_ACTIVE,     .callback = NULL},
                  {.mode = PE13_DISABLE,    .callback = NULL},
                  {.mode = PE14_DISABLE,    .callback = NULL}
                  },
    .cr2        = 0,
    .dier       = 0
};

static bool         isInitialized       = false;

range_map_t         speed_forward_map;
range_map_t         speed_backward_map;
range_map_t         steer_left_map;
range_map_t         steer_right_map;

/**
 * @brief   Initialize periphery connected to driver control
 * @note    Stable for repeated calls
 */
void lldControlInit( void )
{
    if ( isInitialized )
        return;

    /*** PWM pins configuration ***/
    palSetLineMode( pwm1LineCh0,  PAL_MODE_ALTERNATE(1) );
    palSetLineMode( pwm1LineCh1,  PAL_MODE_ALTERNATE(1) );

    pwmStart( pwmDriver, &pwm1conf );

    /*** Calibration coefficients calculation ***/
    range_map_init( &speed_forward_map, CONTROL_NULL, CONTROL_MAX, SPEED_NULL_FORWARD, SPEED_MAX );
    range_map_init( &speed_backward_map, CONTROL_MIN, CONTROL_NULL, SPEED_MIN, SPEED_NULL_BACK );
    range_map_init( &steer_left_map, CONTROL_NULL, CONTROL_MAX, STEER_PWM_NULL, STEER_PWM_MAX );
    range_map_init( &steer_right_map, CONTROL_MIN, CONTROL_NULL, STEER_PWM_MIN, STEER_PWM_NULL );

    /* Set initialization flag */
    isInitialized = true;
}

/**
 * @brief   Set power for driving motor
 * @param   inputPrc   Motor power value [-100 100]
 */
void lldControlSetDrMotorPower( controlValue_t inputPrc )
{
    rawPwmValue_t input_dc;

    inputPrc = CLIP_VALUE(inputPrc, CONTROL_MIN, CONTROL_MAX);

    if( inputPrc == 0 )
    {
        input_dc   = SPEED_ZERO;
    }
    else if( inputPrc > 0)
    {
        input_dc = range_map_call(&speed_forward_map, inputPrc);
    }
    else
    {
        input_dc = range_map_call(&speed_backward_map, inputPrc);
    }

    lldControlSetDrMotorRawPower(input_dc);
}

/**
 * @brief   Set power (in ticks) for driving motor
 * @param   drDuty   dutycycle for speed control
 */
void lldControlSetDrMotorRawPower( rawPwmValue_t dutyCycleSpeed)
{
    dutyCycleSpeed = CLIP_VALUE( dutyCycleSpeed, SPEED_MIN, SPEED_MAX );
    pwmEnableChannel(pwmDriver, drivePWMch, dutyCycleSpeed);
}

/**
 * @brief   Get zero power
 */
rawPwmValue_t lldControlGetDrMotorZeroPower()
{
    return SPEED_ZERO;
}

/**
 * @brief   Set power for steering motor
 * @param   inputPrc   Motor power value [-100 100]
 *                     100  - max left
 *                     center = 0
 *                     -100 - max right
 */
void lldControlSetSteerMotorPower( controlValue_t inputPrc )
{
    rawPwmValue_t input_dc;

    inputPrc = CLIP_VALUE(inputPrc, CONTROL_MIN, CONTROL_MAX);
    if( inputPrc == 0 )
    {
        input_dc   = STEER_PWM_NULL;
    }
    else if( inputPrc > 0)
    {
        input_dc = range_map_call(&steer_left_map, inputPrc);
    }
    else
    {
        input_dc = range_map_call(&steer_right_map, inputPrc);
    }

    lldControlSetSteerMotorRawPower(input_dc);
}



/**
 * @brief   Set power (in ticks) for steering motor
 * @param   steerDuty   dutycycle for steering control
 */
void lldControlSetSteerMotorRawPower( rawPwmValue_t dutyCycleSteer)
{
    dutyCycleSteer = CLIP_VALUE( dutyCycleSteer, STEER_PWM_MIN, STEER_PWM_MAX );
    pwmEnableChannel(pwmDriver, steerPWMch, dutyCycleSteer);
}

/**
 * @brief   Get zero power
 */
rawPwmValue_t lldControlGetSteerMotorZeroPower()
{
    return STEER_PWM_NULL;
}
