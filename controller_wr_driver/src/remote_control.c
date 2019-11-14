#include <tests.h>
#include <remote_control.h>

/*============================================================================*/
/* VALUE LIMITS                                                               */
/*============================================================================*/

#define RC_STEER_MAX    1700
#define RC_STEER_NULL   1400
#define RC_STEER_MIN    1100

#define RC_SPEED_MAX    1935
#define RC_SPEED_NULL   1520
#define RC_SPEED_MIN    1060

/*============================================================================*/
/* PINS CONFIGURATION                                                         */
/*============================================================================*/

#define icuSteering         PAL_LINE( GPIOE, 5 )
#define icuSpeed            PAL_LINE( GPIOC, 6 )

/***    ICU Steering uses Timer 9 ***/
static ICUDriver *icuSteerDriver      = &ICUD9;

/***    ICU Driving uses Timer 8 ***/
static ICUDriver *icuSpeedDriver      = &ICUD8;

/***    width in ticks from RC for steering wheels  ***/
pwmValue_t  steer_rc    =   0;
/***    width in ticks from RC for driving wheels   ***/
pwmValue_t  speed_rc    =   0;
/***    flag for RC mode                            ***/
bool        rc_mode     = false;

static thread_reference_t trp_rcmode = NULL;

static void icuWidthcb_steer(ICUDriver *icup)
{
    steer_rc = icuGetWidthX(icup);                // ...X - can work anywhere
                                                    // return width in ticks
    chSysLockFromISR();
    chThdResumeI(&trp_rcmode, MSG_OK);            /* Resuming the thread with message.*/
    chSysUnlockFromISR();
}

static void icuWidthcb_speed(ICUDriver *icup)
{
    speed_rc  = icuGetWidthX(icup);               // ...X - can work anywhere
                                                    // return width in ticks
}

/*============================================================================*/
/* CONFIGURATION ZONE                                                         */
/*============================================================================*/


/*** Configuration structures for Steering channel ***/

static const ICUConfig icucfg_steer = {
  .mode         = ICU_INPUT_ACTIVE_HIGH,       // Trigger on rising edge
  .frequency    = 1000000,                     // do not depend on PWM freq
  .width_cb     = icuWidthcb_steer,            // callback for Steering
  .period_cb    = NULL,
  .overflow_cb  = NULL,
  .channel      = ICU_CHANNEL_1,               // for Timer
  .dier         = 0
};

/*** Configuration structures for Steering channel ***/

static const ICUConfig icucfg_speed = {
  .mode         = ICU_INPUT_ACTIVE_HIGH,       // Trigger on rising edge
  .frequency    = 1000000,                     // do not depend on PWM freq
  .width_cb     = icuWidthcb_speed,            // callback for Speed
  .period_cb    = NULL,
  .overflow_cb  = NULL,
  .channel      = ICU_CHANNEL_1,               // for Timer
  .dier         = 0
};


static THD_WORKING_AREA(waRCModeDetect, 256); // 128 - stack size
static THD_FUNCTION(RCModeDetect, arg)
{
    arg = arg;

    msg_t msg_mode;     // var for mode detection

    while( true )
    {
      /* Waiting for the IRQ to happen */
      chSysLock();
      msg_mode = chThdSuspendTimeoutS(&trp_rcmode, MS2ST(100)); // 100 ms timeout
      chSysUnlock();

      if(msg_mode == MSG_OK)
      {
        rc_mode = true;
        palSetLine( LINE_LED3 );
      }
      else if(msg_mode == MSG_TIMEOUT)
      {
        rc_mode = false;
        palClearLine( LINE_LED3 );
      }
    }
}

static bool         isInitialized       = false;

range_map_t         steer_map;
range_map_t         speed_map;

/**
 * @brief   Initialize periphery connected to remote control
 * @note    Stable for repeated calls
 * @param   prio defines priority of inside thread
 *          IMPORTANT! NORMALPRIO + prio
 */
void remoteControlInit( tprio_t prio )
{
    if ( isInitialized )
        return;

    /* Swapped for inverse */
    range_map_init(&steer_map, RC_STEER_MIN, RC_STEER_MAX, CONTROL_MAX, CONTROL_MIN);
    range_map_init(&speed_map, RC_SPEED_MIN, RC_SPEED_MAX, CONTROL_MIN, CONTROL_MAX);

    icuStart( icuSteerDriver, &icucfg_steer );
    palSetLineMode( icuSteering, PAL_MODE_ALTERNATE(3) );
    icuStartCapture( icuSteerDriver );
    icuEnableNotifications( icuSteerDriver );

    icuStart( icuSpeedDriver, &icucfg_speed );
    palSetLineMode( icuSpeed, PAL_MODE_ALTERNATE(3) );
    icuStartCapture( icuSpeedDriver );
    icuEnableNotifications( icuSpeedDriver );

    chThdCreateStatic( waRCModeDetect, sizeof(waRCModeDetect), prio , RCModeDetect, NULL );

    /* Set initialization flag */

    isInitialized = true;
}

/**
 * @brief   Detect working mode
 * @return  true    - RC mode enable
 *          false   - RC mode disable
 * @note    this function should be called before
 *          getting width value
 */
bool rcModeIsEnabled( void )
{
    return rc_mode;
}


/**
 * @brief   Return speed control signal (width) in ticks
 * @return  width for speed
 * @note    Before this function
 *          rcModeIsEnabled must be checked
 */
pwmValue_t rcGetSpeedDutyCycleValue( void )
{
    return speed_rc;
}

/**
 * @brief   Return steering control signal (width) in ticks
 * @return  width for steering
 * @note    Before this function
 *          rcModeIsEnabled must be checked
 */
pwmValue_t rcGetSteerDutyCycleValue( void )
{
    return steer_rc;
}

/**
 * @brief   Get control values for driving wheels
 * @return  control signal [-100; 100]
 * @note    Before this function
 *          rcModeIsEnabled must be called
 */
icuControlValue_t rcGetSpeedControlValue( void )
{
    icuControlValue_t speed_prt_cntr = range_map_call(&speed_map, speed_rc);
    speed_prt_cntr = CLIP_VALUE( speed_prt_cntr, CONTROL_MIN, CONTROL_MAX);
    return speed_prt_cntr;
}

/**
 * @brief   Get control values for steering wheels
 * @return  control signal [-100; 100]
 * @note    Before this function
 *          rcModeIsEnabled must be checked
 */
icuControlValue_t rcGetSteerControlValue( void )
{
    icuControlValue_t steer_prt_cntr = range_map_call(&steer_map, steer_rc);
    steer_prt_cntr = CLIP_VALUE( steer_prt_cntr, CONTROL_MIN, CONTROL_MAX);
    return steer_prt_cntr;
}

