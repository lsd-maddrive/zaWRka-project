#include <tests.h>
#include <lld_encoder.h>

#define MAX_TICK_NUM        360


#define ENCODER_GREEN_LINE  PAL_LINE( GPIOD, 3 )
#define ENCODER_WHITE_LINE  PAL_LINE( GPIOD, 4 )

/**
 * @brief   Get decimal values depends on 2 channels encoder state
 * @return  values [0, 3]
 */
rawEncoderValue_t getEncoderState( void )
{
    rawEncoderValue_t res_enc = 0;
    if( palReadLine( ENCODER_GREEN_LINE ) )     res_enc |= 0b01;
    if( palReadLine( ENCODER_WHITE_LINE ) )     res_enc |= 0b10;

    return res_enc;
}

static void extcb1(EXTDriver *extp, expchannel_t channel)
{
    (void)extp;
    (void)channel;

    enc_trigger_counter += 1;

#if 0
    if( enc_trigger_counter > (MAX_TICK_NUM * 2) )      enc_trigger_counter = 0;

    uint8_t i = 0;

    rawEncoderValue_t encoder_state = getEncoderState( );

    for( i = 0; i < 4; i++ )
    {
        if( encoder_state == encoder_decode_table[i])
        {
            curr_enc_state = i;
            break;
        }
    }

    if( curr_enc_state > prev_enc_state )
    {
        if( prev_enc_state == 0 && curr_enc_state == 3 ) enc_dir = 'F';
        else    enc_dir = 'B';
    }
    else
    {
        if( prev_enc_state == 3 && curr_enc_state == 0 ) enc_dir = 'B';
        else    enc_dir = 'F';
    }

    prev_enc_state = curr_enc_state;

    enc_trigger_counter += 1;

    if( enc_trigger_counter > (MAX_TICK_NUM * 2) )      enc_trigger_counter = 0;
    if( enc_trigger_counter == (MAX_TICK_NUM * 2) )
    {
        if( enc_dir == 'F') enc_rev_number += 1;
        else enc_rev_number -= 1;
    }

    enc_table_val = encoder_state;
#endif

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
    {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOD , extcb1}, // PD3
    {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOD , extcb1}, // PD4
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

    /* Start working GPT driver in asynchronous mode */
    gptStart( gptDriver, &gpt2cfg );
    gptStartContinuous( gptDriver, gptFreq );

    /* Set initialization flag */

    isInitialized = true;
}


