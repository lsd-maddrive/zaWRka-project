#include <tests.h>
#include <lld_light.h>
#include <lld_start_button.h>

#define             RIGHT_TURN_LINE         PAL_LINE( GPIOG, 2 )
#define             LEFT_TURN_LINE          PAL_LINE( GPIOG, 3 )

bool                turn_right_flag = 0;
bool                turn_left_flag  = 0;
turn_light_state    turn_state = STOP; 

/*
 * @brief   Automatically set the state of turn lights
 *          depends on input conditions 
 */
void lldLightDetectTurnState( float steer_cntrl, float speed_cntrl, system_state s_state )
{
    if( steer_cntrl > 0 ) turn_state = LEFT; 
    else if( steer_cntrl < 0 ) turn_state = RIGHT;
    else if ( steer_cntrl == 0 && speed_cntrl == 0 && s_state == RUN )
    {
        turn_state = STOP; 
    }
    else if( s_state == IDLE )
    {
        turn_state = REMOTE;
    }
}

/*
 * @brief   Get current state of turn lights  
 */
turn_light_state lldGetLightState( void )
{
    return turn_state; 
}

static THD_WORKING_AREA(waTurnRoutine, 128); // 128 - stack size
static THD_FUNCTION(TurnRoutine, arg)
{
    arg = arg; 

    if( turn_state == RIGHT )
    {
        palToggleLine( RIGHT_TURN_LINE );
        palClearLine( LEFT_TURN_LINE );
    }
    else if( turn_state == LEFT )
    {
        palToggleLine( LEFT_TURN_LINE );
        palClearLine( RIGHT_TURN_LINE );
    }
    else if( turn_state == STOP )
    {
        palToggleLine( LEFT_TURN_LINE );
        palToggleLine( RIGHT_TURN_LINE );
    }
    else if( turn_state == REMOTE )
    {
        palClearLine( LEFT_TURN_LINE );
        palClearLine( RIGHT_TURN_LINE );
    }

    chThdSleepMilliseconds( 300 );    

}

#define SPI_MOSI_LINE   PAL_LINE( GPIOC, 3)
#define SPI_SCLK_LINE   PAL_LINE( GPIOB, 10)
#define SPI_MISO_LINE   PAL_LINE( GPIOC, 2)

static bool             isInitialized = false;

/**
 * @brief   Initialize periphery connected LEDs
 * @note    Stable for repeated calls
 */
void lldLightInit( tprio_t priority )+
{
    if( isInitialized )
          return;

    palSetLineMode( RIGHT_TURN_LINE, PAL_MODE_OUTPUT_PUSHPULL );
    palSetLineMode( LEFT_TURN_LINE,  PAL_MODE_OUTPUT_PUSHPULL );

    /***    LED Matrix   ***/
    palSetLineMode( SPI_SCLK_LINE, PAL_MODE_ALTERNATE(5) );
    palSetLineMode( SPI_MOSI_LINE, PAL_MODE_ALTERNATE(5) );
    palSetLineMode( SPI_MISO_LINE, PAL_MODE_ALTERNATE(5) );





    chThdCreateStatic(waTurnRoutine, sizeof(waTurnRoutine), priority, TurnRoutine, NULL);

    isInitialized = true;
}
