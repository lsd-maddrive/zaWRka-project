#include <tests.h>
#include <lld_start_button.h>

#define START_BUTTON_LINE   PAL_LINE( GPIOC, 13 )

system_state    cur_system_state = IDLE;

virtual_timer_t stop_vt;

static thread_reference_t trp_button = NULL;

static void ext_but_cb(EXTDriver *extp, expchannel_t channel)
{
    extp = extp; channel = channel;

    chSysLockFromISR();                   // ISR Critical Area
    chThdResumeI( &trp_button, MSG_OK );
    chSysUnlockFromISR();                 // Close ISR Critical Area
}

/*    Set system in IDLE state,
 *    when timer counted 4 minutes
 */
static void stop_vt_cb( void *arg )
{
    arg = arg;  // to avoid warnings

    cur_system_state  = IDLE;
}

static THD_WORKING_AREA(waButton, 256); // 128 - stack size
static THD_FUNCTION(Button, arg)
{
    arg = arg;            // just to avoid warnings

    msg_t msg_button;     // var for mode detection

    while( true )
    {
        /* Waiting for the IRQ to happen */
        chSysLock();
        msg_button = chThdSuspendS( &trp_button );
        chSysUnlock();

        if( msg_button == MSG_OK )
        {
            chThdSleepMilliseconds( 500 ); // to avoid button bounce
            if( palReadLine( START_BUTTON_LINE ) == 0)
            {
                if( cur_system_state == IDLE )
                {
                    cur_system_state = RUN;
                    chVTSet( &stop_vt, S2ST( 240 ), stop_vt_cb, NULL );
                }
                else if( cur_system_state == RUN )
                {
                    cur_system_state = IDLE;
                    chVTReset( &stop_vt );
                }
            }
        }
    }
}

/**
 * @brief   Get system mode
 * @return  Values from mySystemState
 *          IDLE
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
void startButtonInit( tprio_t priority )
{
    if ( isInitialized )
            return;

    EXTChannelConfig trg_but_conf = {
         .mode = EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOC,
         .cb = ext_but_cb
     };

     commonExtDriverInit();

     extSetChannelMode( &EXTD1, 13, &trg_but_conf ); // PC13 = Button
     chVTObjectInit(&stop_vt);
     chThdCreateStatic(waButton, sizeof(waButton), priority, Button, NULL);

     /* Set initialization flag */
     isInitialized = true;
}
