#include <tests.h>
#include <lld_encoder.h>

#define MAX_TICK_NUM        500


#define ENCODER_GREEN_LINE  PAL_LINE( GPIOD, 3 )
#define ENCODER_WHITE_LINE  PAL_LINE( GPIOD, 4 )

rawEncoderValue_t   enc_trg_cntr    = 0;
rawEncoderValue_t   enc_revs_cntr   = 0;

EncoderValue_t      enc_rev_number      = 0;
EncoderValue_t      enc_dir             = 0;

rawEncoderValue_t   enc_table_val       = 0;
rawEncoderValue_t   curr_enc_state      = 0;
rawEncoderValue_t   prev_enc_state      = 0;


rawEncoderValue_t   enc_decode_table[4] = {0, 1, 3, 2};

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

    enc_trg_cntr += 1;


    uint8_t i = 0;

    rawEncoderValue_t enc_state = getEncoderState( );

    for( i = 0; i < 4; i++ )
    {
        if( enc_state == enc_decode_table[i])
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



    if( enc_trg_cntr > (MAX_TICK_NUM * 2) )      enc_trg_cntr = 0;
    if( enc_trg_cntr == (MAX_TICK_NUM * 2) )
    {
        if( enc_dir == 'F') enc_rev_number += 1;
        else enc_rev_number -= 1;
    }

    enc_table_val = enc_state;

}



static void extcb2(EXTDriver *extp, expchannel_t channel)
{
    (void)extp;
    (void)channel;

    if( enc_dir == 'F' )    enc_revs_cntr += 1;
    else                    enc_revs_cntr -= 1;

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
    {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOD , extcb2}, // PD5
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
 * @brief   Get raw encoder value
 * @return  raw encoder values (ticks)
 */
rawEncoderValue_t getEncoderRawTickNumber( void )
{
    return enc_trg_cntr;
}

/**
 * @brief   Get encoder revolutions number
 * @return  number of motor revs
 */
rawEncoderValue_t getEncoderRevsNumber( void )
{
    return enc_revs_cntr;
}


dirEncoderValue_t getEncoderDirection( void )
{
    return enc_dir;
}
