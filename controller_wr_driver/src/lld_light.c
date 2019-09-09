#include <tests.h>
#include <lld_light.h>
#include <lld_start_button.h>

#include <max7219.h> // for led-matrix is required 

/***    Turning light Configuration pins     ***/
#define             RIGHT_TURN_LINE         PAL_LINE( GPIOG, 2 )
#define             LEFT_TURN_LINE          PAL_LINE( GPIOG, 3 )
/***********************************************/

/***    LED-matrix Configuration pins        ***/
#define SCK_LINE    PAL_LINE( GPIOC, 10 )
#define MISO_LINE   PAL_LINE( GPIOC, 11 )

#define MOSI_LINE   PAL_LINE( GPIOC, 12 )
#define CS_PORT     GPIOC
#define CS_PIN      9
#define CS_LINE     PAL_LINE( CS_PORT, CS_PIN )

static SPIDriver    *spiDriver  = &SPID3;
/***********************************************/

/*** Configuration structures for LED-matrix ***/
//Maximum speed SPI configuration (27MHz, CPHA=0, CPOL=0, MSb first).
static const SPIConfig spicfg = {
  .end_cb   = NULL,                         // Operation complete callback
  .ssport   = CS_PORT,                      // The chip select line port
  .sspad    = CS_PIN,                       // The chip select line pad number
  .cr1      = SPI_CR1_BR | SPI_CR1_BR_0,    // 64
  .cr2      = SPI_CR2_DS                    // 16-bit size mode
};
/***********************************************/

/*============================================================================*/
/* LED-matrix Related                                                         */
/*============================================================================*/

static uint8_t signs[MAX_PRESET_NUMB][MAX_DIGITS] = {
   {0x00, 0x00, 0x00, 0x7E, 0x7E, 0x00, 0x00, 0x00}, /** 0 = Brick sign             */
   {0x00, 0x18, 0x3C, 0x5A, 0x18, 0x18, 0x18, 0x00}, /** 1 = Only Forward sign      */
   {0x00, 0x08, 0x04, 0x7E, 0x7E, 0x64, 0x68, 0x00}, /** 2 = Only Right sign        */
   {0x00, 0x10, 0x20, 0x7E, 0x7E, 0x26, 0x16, 0x00}, /** 3 = Only Left sign         */
   {0x20, 0x70, 0xA8, 0x20, 0x22, 0x3F, 0x22, 0x24}, /** 4 = Forward OR Right sign  */
   {0x04, 0x0E, 0x15, 0x04, 0x44, 0xFC, 0x44, 0x24}, /** 5 = Forward OR Left sign   */
   {0x00, 0x24, 0x24, 0x24, 0x81, 0x42, 0x3C, 0x00}, /** 6 = Happy Face             */
   {0x00, 0x24, 0x24, 0x24, 0x00, 0x3C, 0x42, 0x81}, /** 7 = Sad Face               */
   {0x24, 0x24, 0x24, 0x00, 0xFF, 0x05, 0x02, 0x00}, /** 8 = Tongue Face            */
   {0x7C, 0x62, 0x62, 0x62, 0x7C, 0x60, 0x60, 0x60}, /** 9 = Parking                */

};

/*
 * @brief   Show figure in led-matrix 
 * @note    Be carefull with direction of installation of matrix
 * @param   figure shoulb be choosed from enum-table sign  
 */
void led_show(sign figure)
{
  for(int i = 0; i < MAX_DIGITS; i++)
  {
    max7219WriteRegister(spiDriver, MAX7219_AD_DIGIT_0 + (i << 8), signs[figure][i]);
  }
}

/*============================================================================*/
/* Turning lights Related                                                     */
/*============================================================================*/
bool                turn_right_flag = 0;
bool                turn_left_flag  = 0;
turn_light_state    turn_state = REMOTE; 

/*
 * @brief   Automatically set the state of turn lights
 *          depends on input conditions 
 */
void lldLightDetectTurnState( float steer_cntrl, float speed_cntrl, system_state s_state )
{
    if( steer_cntrl > 0 && s_state != IDLE) turn_state = LEFT; 
    else if( steer_cntrl < 0 && s_state != IDLE ) turn_state = RIGHT;
    else if ( steer_cntrl == 0 && speed_cntrl == 0 && s_state == RUN )
    {
        turn_state = STOP; 
    }
    else if( s_state == IDLE )
    {
        turn_state = REMOTE;
    }
}

void lldLightResetTurnState( void )
{
    turn_state = REMOTE;
}

/*
 * @brief   Get current state of turn lights  
 */
turn_light_state lldGetLightState( void )
{
    return turn_state; 
}

static THD_WORKING_AREA(waTurnRoutine, 256); // 128 - stack size
static THD_FUNCTION(TurnRoutine, arg)
{
    arg = arg; 

    while( 1 )
    {
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

        chThdSleepMilliseconds( 500 );    
    }
}

static bool             isInitialized = false;

/**
 * @brief   Initialize periphery connected LEDs
 * @note    Stable for repeated calls
 */
void lldLightInit( tprio_t priority )
{
    if( isInitialized )
          return;
    /*** Turning lights initialization  ***/
    palSetLineMode( RIGHT_TURN_LINE, PAL_MODE_OUTPUT_PUSHPULL );
    palSetLineMode( LEFT_TURN_LINE,  PAL_MODE_OUTPUT_PUSHPULL );

    /*** LED-matrix initialization  ***/
    palSetLineMode( SCK_LINE,  PAL_MODE_ALTERNATE( 6 ) | PAL_STM32_OSPEED_HIGHEST);
    palSetLineMode( MISO_LINE, PAL_MODE_ALTERNATE( 6 ) | PAL_STM32_OSPEED_HIGHEST);
    palSetLineMode( MOSI_LINE, PAL_MODE_ALTERNATE( 6 ) | PAL_STM32_OSPEED_HIGHEST);

    palSetLineMode( CS_LINE, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
    palSetLine( CS_LINE );

    spiStart( spiDriver, &spicfg );

    // LED-matrix Configuration for MAX7219
    max7219WriteRegister(spiDriver, MAX7219_AD_DISPLAY_TEST, FALSE);
    max7219WriteRegister(spiDriver, MAX7219_AD_SHUTDOWN, MAX7219_OM_Normal);
    max7219WriteRegister(spiDriver, MAX7219_AD_SCAN_LIMIT, MAX7219_SL_7);
    max7219WriteRegister(spiDriver, MAX7219_AD_DECODE_MODE, MAX7219_DM_No_decode);
    max7219WriteRegister(spiDriver, MAX7219_AD_INTENSITY, MAX7219_IM_31_32);

    /*** Turning lights thread  ***/
    chThdCreateStatic( waTurnRoutine, sizeof(waTurnRoutine), priority, TurnRoutine, NULL);

    isInitialized = true;
}