#include <tests.h>
#include <lld_start_button.h>

#define START_BUTTON_LINE   PAL_LINE( GPIOC, 13 )


uint32_t        b_count = 0;
system_state    cur_system_state = IDLE;

static thread_reference_t trp_button = NULL;

static void ext_but_cb(EXTDriver *extp, expchannel_t channel)
{
    extp = extp; channel = channel;

    chSysLockFromISR();                   // ISR Critical Area
    chThdResumeI( &trp_button, MSG_OK );
    chSysUnlockFromISR();                 // Close ISR Critical Area
}

static THD_WORKING_AREA(waButton, 128); // 128 - stack size
static THD_FUNCTION(Button, arg)
{
  arg = arg; // just to avoid warnings

  msg_t msg_button;     // var for mode detection

  while( true )
  {
    /* Waiting for the IRQ to happen */
    chSysLock();
    msg_button = chThdSuspendS( &trp_button );
    chSysUnlock();

    if( msg_button == MSG_OK )
    {
      chThdSleepMicroseconds( 50 );
      if( palReadLine( START_BUTTON_LINE ) == 0)
      {
          b_count = (b_count == 3) ? 0 : ++b_count;
          if( b_count == 1 ) cur_system_state = WAIT;
//          cur_system_state = ( b_count == 1 ) ? WAIT : IDLE;
      }
    }
  }
}

/**
 * @brief   Get number of presses on button
 */
uint32_t lldGetStartButtonPressedNumber( void )
{
    return b_count;
}

static THD_WORKING_AREA(waStateChecker, 128); // 128 - stack size
static THD_FUNCTION(StateChecker, arg)
{
    arg = arg;

    while( true )
    {
        switch( cur_system_state )
        {
          case 0:   // IDLE
            break;

          case 1:   // WAIT
            palSetLine( LINE_LED2);
            chThdSleepSeconds(5);
            palClearLine( LINE_LED2);
            cur_system_state = RUN; // need to fix!!!
            break;

          case 2:   // RUN
            lldResetOdomety( );


          default:
            ;
       }
       chThdSleepMicroseconds( 100 ); // need to fix by SleepUntil?
    }
}

/**
 * @brief   Get system mode
 * @return  Values from mySystemState
 *          IDLE
 *          WAIT
 *          RUN
 */
system_state lldGetSystemState( void )
{
    return cur_system_state;
}

static bool         isInitialized       = false;

/**
 * @brief   Initialize EXT driver for button
 * @note    Stable for repeated calls
 */
void startButtonInit( int8_t priority )
{
    if ( isInitialized )
            return;

    EXTChannelConfig trg_but_conf = {
         .mode = EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOC,
         .cb = ext_but_cb
     };

     commonExtDriverInit();

     extSetChannelMode( &EXTD1, 13, &trg_but_conf ); // PC13 = Button

     chThdCreateStatic(waButton, sizeof(waButton), NORMALPRIO + priority, Button, NULL);
     chThdCreateStatic(waStateChecker, sizeof(waStateChecker), NORMALPRIO + priority - 1, StateChecker, NULL);

     /* Set initialization flag */
     isInitialized = true;
}
