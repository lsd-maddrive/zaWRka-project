#include <lld_light.h>

#define                 VT_TURN_MS              300
static virtual_timer_t  turn_update_vt;

#define                 RIGHT_TURN_LINE         PAL_LINE( GPIOG, 2 )
#define                 LEFT_TURN_LINE          PAL_LINE( GPIOG, 3 )

bool    turn_right_flag = 0;
bool    turn_left_flag  = 0;


void lightDetectTurnState( float  )
{

}

static void turn_update_vt_cb( void *arg)
{
    if( turn_right_flag == 1 )
    {
        palToggleLine( RIGHT_TURN_LINE );
        palClearLine( LEFT_TURN_LINE );
    }
    else
    {
        palClearLine( RIGHT_TURN_LINE );
    }

    if( turn_left_flag == 1 )
    {
        palToggleLine( LEFT_TURN_LINE );
        palClearLine( RIGHT_TURN_LINE );
    }
    else
    {
        palClearLine( LEFT_TURN_LINE );
    }

    chSysLockFromISR();
    // reset VT
    chVTSetI(&turn_update_vt, MS2ST( VT_TURN_MS ), turn_update_vt_cb, NULL);
    chSysUnlockFromISR();


}


static bool             isInitialized = false;

void lldLightInit( void )
{
    if( isInitialized )
          return;

    palSetLineMode( RIGHT_TURN_LINE, PAL_MODE_OUTPUT_PUSHPULL );
    palSetLineMode( LEFT_TURN_LINE,  PAL_MODE_OUTPUT_PUSHPULL );

    chVTObjectInit(&turn_update_vt);
    chVTSet( &turn_update_vt, MS2ST( VT_TURN_MS ), turn_update_vt_cb, NULL );

    isInitialized = true;
}
