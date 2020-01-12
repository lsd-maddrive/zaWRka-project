#include <tests.h>
#include <lld_encoder.h>

#define ENC_MAX_TICK_NUM        500

/*============================================================================*/
/* LINE CONFIGURATION                                                         */
/*============================================================================*/

#define ENCODER_GREEN_LINE  PAL_LINE( GPIOD, 5 )
#define ENCODER_WHITE_LINE  PAL_LINE( GPIOD, 4 )
#define ENCODER_NULL_LINE   PAL_LINE( GPIOD, 3 )

/*============================================================================*/
/* Variable CONFIGURATION                                                     */
/*============================================================================*/

rawEncoderValue_t       enc_tick_cntr       = 0;
rawRevEncoderValue_t    enc_revs_cntr       = 0;

rawEncoderValue_t       enc_null_revs_cntr  = 0;

bool                    enc_dir_state       = 0;

/***    Base channel processing     ***/
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
    if( enc_tick_cntr == (ENC_MAX_TICK_NUM - 1) )
    {
        enc_revs_cntr   += 1;
        enc_tick_cntr    = 0;
    }
    else if( enc_tick_cntr == (-ENC_MAX_TICK_NUM + 1) )
    {
        enc_revs_cntr   -= 1;
        enc_tick_cntr    = 0;
    }
}

/***    NULL Point of encoder processing    ***/
static void extcb_null(EXTDriver *extp, expchannel_t channel)
{
    (void)extp;
    (void)channel;

    if( enc_dir_state == 0 )  enc_null_revs_cntr -= 1;
    else                      enc_null_revs_cntr += 1;
}

static bool         isInitialized       = false;

/**
 * @brief   Initialize periphery connected to encoder
 * @note    Stable for repeated calls
 */
void lldEncoderInit( void )
{
    if ( isInitialized )
            return;

    EXTChannelConfig base_conf = {
         .mode = EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOD,
         .cb = extcb_base

    };

    EXTChannelConfig null_conf = {
         .mode = EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOD,
         .cb = extcb_null

    };

    commonExtDriverInit( );

    extSetChannelMode( &EXTD1, 5, &base_conf );
    extSetChannelMode( &EXTD1, 3, &null_conf );

    /* Set initialization flag */
    isInitialized = true;
}

/**
 * @brief   Get number of encoder ticks
 * @note    Max number of ticks is defined by MAX_TICK_NUM
 * @return  Encoder ticks number depends on direction of rotation
 *          [0; ENC_MAX_TICK_NUM]
 *          after ENC_MAX_TICK_NUM it resets
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
 * @brief   Get number of encoder revolutions [double]
 * @note    1 revolution = MAX_TICK_NUM ticks
 * @return  Encoder revolutions number depends on direction of rotation
 */
rawRevEncoderValue_t   lldGetEncoderRawRevs( void )
{
    return ( enc_revs_cntr + enc_tick_cntr / (float)ENC_MAX_TICK_NUM );
}

/**
 * @brief   Get number of encoder revolutions [int]
 * @note    If you use absolute encoder!!!
 * @return  Encoder revolutions number depends on direction of rotation
 */
rawEncoderValue_t   lldGetAbsoluteEncoderRawRevs( void )
{
    return enc_null_revs_cntr;
}

/*
 * @brief   Reset all variable for lld-encoder unit
 */
void lldResetEncoder( void )
{
    enc_tick_cntr       = 0;
    enc_revs_cntr       = 0;
    enc_null_revs_cntr  = 0;
    enc_dir_state       = 0;
}

