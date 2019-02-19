#include <tests.h>
#include <lld_encoder.h>

#define MAX_TICK_NUM        500


#define ENCODER_GREEN_LINE  PAL_LINE( GPIOD, 3 )
#define ENCODER_WHITE_LINE  PAL_LINE( GPIOD, 4 )
#define ENCODER_NULL_LINE   PAL_LINE( GPIOD, 5 )

rawEncoderValue_t   enc_tick_cntr       = 0;
rawEncoderValue_t   enc_revs_cntr       = 0;

rawEncoderValue_t   enc_null_revs_cntr  = 0;

bool                enc_dir_state       = 0;

static void extcb_base(EXTDriver *extp, expchannel_t channel)
{
    (void)extp;
    (void)channel;

    /***    To define direction of encoder rotation  ***/
    if( palReadLine( ENCODER_WHITE_LINE ) == 0 )
    {
        enc_tick_cntr    += 1;
        enc_dir_state    = 1;       // counterclockwise
    }
    else
    {
        enc_tick_cntr    -= 1;
        enc_dir_state    = 0;       // clockwise
    }

    /***    Reset counter when it reaches the MAX value  ***/
    /***    Count encoder revolutions                    ***/
    if( enc_tick_cntr == MAX_TICK_NUM )
    {
        enc_revs_cntr   += 1;
        enc_tick_cntr    = 0;
    }
    else if( enc_tick_cntr == -MAX_TICK_NUM )
    {
        enc_revs_cntr   -= 1;
        enc_tick_cntr    = 0;
    }

}

static void extcb_dir(EXTDriver *extp, expchannel_t channel)
{
    (void)extp;
    (void)channel;

}



static void extcb_null(EXTDriver *extp, expchannel_t channel)
{
    (void)extp;
    (void)channel;

    if( enc_dir_state == 0 )  enc_null_revs_cntr -= 1;
    else                      enc_null_revs_cntr += 1;


}

/********************************/
/*** Configuration structures ***/
/********************************/

static const EXTConfig extcfg =
{
   {
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOD , extcb_base}, // PD3
    {EXT_CH_MODE_BOTH_EDGES  | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOD , extcb_dir},  // PD4
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOD , extcb_null}, // PD5
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL}
  }
};

static bool         isInitialized       = false;

/**
 * @brief   Initialize periphery connected to encoder
 * @note    Stable for repeated calls
 */
void lldEncoderInit( void )
{
    if ( isInitialized )
            return;

    extStart( &EXTD1, &extcfg );

    /* Set initialization flag */

    isInitialized = true;
}

/**
 * @brief   Get number of encoder ticks
 * @note    Max number of ticks is defined by MAX_TICK_NUM
 * @return  Encoder ticks number depends on direction of rotation
 */
rawEncoderValue_t lldGetEncoderRawTicks( void )
{
    return enc_tick_cntr;
}

/**
 * @brief   Get direction of encoder rotation
 * @return  clockwise           -> 0
 *          counterclockwise    -> 1
 */
bool lldGetEncoderDirection( void )
{
    return enc_dir_state;
}

/**
 * @brief   Get number of encoder revolutions
 * @note    1 revolution = MAX_TICK_NUM ticks
 * @return  Encoder revolutions number depends on direction of rotation
 */
rawEncoderValue_t   lldGetEncoderRawRevs( void )
{
    return enc_revs_cntr;
}

/**
 * @brief   Get number of encoder revolutions
 * @note    If you use absolute encoder!!!
 * @return  Encoder revolutions number depends on direction of rotation
 */
rawEncoderValue_t   lldGetAbsoluteEncoderRawRevs( void )
{
    return enc_null_revs_cntr;
}

