#include <tests.h>
#include <encoder.h>
//#include <math.h> // does not work

#define MAX_TICK_NUM        360

/* need checking, maybe should 0.105 / 6 */
#define GAIN                0.105

#define ENCODER_GREEN_LINE  PAL_LINE( GPIOD, 3 )
#define ENCODER_WHITE_LINE  PAL_LINE( GPIOD, 4 )

rawEncoderValue_t   enc_trigger_counter = 0;
encoderValue_t      enc_rev_number      = 0;
encoderValue_t      enc_dir             = 0;

rawEncoderValue_t   enc_table_val       = 0;
rawEncoderValue_t   curr_enc_state      = 0;
rawEncoderValue_t   prev_enc_state      = 0;

rawEncoderValue_t   encoder_decode_table[4] = {0, 1, 3, 2};

/***    GPT Configuration Zone    ***/

#define gptFreq     100000 // 1 Hz => 1 s
#define gpt10ms     (int)( gptFreq * 0.01 )

static GPTDriver    *gptDriver  =   &GPTD2;

rawEncoderValue_t   ticks_per_sec      = 0;
rawEncoderValue_t   prev_trg_counter   = 0;

encoderValue_t      revs_per_sec       = 0;
encoderValue_t      prev_rev_number    = 0;

encSpeedValue_t     speed_cmps         = 0;
encoderValue_t      prev_dist          = 0;


static void gpt2_cb (GPTDriver *gptd)
{
    gptd = gptd;

    int32_t cur_dist = getEncoderDistanceCm();
    int32_t t_period = 1; // 1 sec
    ticks_per_sec  = abs( enc_trigger_counter - prev_trg_counter );
    revs_per_sec   = abs( enc_rev_number - prev_rev_number );


    prev_trg_counter = enc_trigger_counter;
    prev_rev_number  = enc_rev_number;

    speed_cmps       = abs( (cur_dist - prev_dist ) * t_period );

    prev_dist = cur_dist;

    palToggleLine( LINE_LED2 );


}

static const GPTConfig gpt2cfg = {
  .frequency =  gptFreq,
  .callback  =  gpt2_cb,
  .cr2       =  0,
  .dier      =  0U
};

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
}


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
 * @brief   Initialize periphery connected to encoder control
 * @note    Stable for repeated calls
 */
void encoderInit( void )
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

/**
 * @brief   Get raw encoder value
 * @return  raw encoder values (ticks)
 */
rawEncoderValue_t getEncoderRawTickNumber( void )
{
    return enc_trigger_counter;
}


/**
 * @brief   Get encoder revs value
 * @return  number of motor revs
 */
encoderValue_t getEncoderRevNumber( void )
{
    return enc_rev_number;
}

/**
 * @brief   Define direction of rotation
 * @return  'F' = forward  = clockwise
 *          'B' = backward = counterclockwise
 */
encoderValue_t getEncoderDirectionState( void )
{

    return enc_dir;
}

/**
 * @brief   Get table values of encoder
 * @return  numbers [0; 3]
 * @note    0 -> 1 -> 3 -> 2 = backward = counterclockwise
 *          1 -> 0 -> 2 -> 3 = forward  = clockwise
 */
rawEncoderValue_t getEncoderValTable( void )
{
    return enc_table_val;
}

/**
 * @brief   Get distance in cm
 * @return  (int) distance in cm
 */
encoderValue_t getEncoderDistanceCm( void )
{
    encoderValue_t distance = 0;
    encoderValue_t revs = 0;
    revs = getEncoderRevNumber( );
    distance = revs * 2 * 3.14 * WHEEL_RADIUS_CM * GAIN;
    return distance;
}

/**
 * @brief   Get speed [ticks per second]
 * @return  (int) absolute speed TPS
 */
rawEncoderValue_t getEncoderSpeedTPS( void )
{
  return ticks_per_sec;
}

/**
 * @brief   Get speed [revs per second]
 * @return  (int) absolute speed RPS
 */
encoderValue_t getEncoderSpeedRPS( void )
{
  return revs_per_sec;
}

/**
 * @brief   Get speed [cm per second]
 * @return  (int) absolute speed CMPS
 */
encSpeedValue_t getEncWheelSpeedCmPS( void )
{
    return speed_cmps;
}

/**
 * @brief   Get speed [metr per second]
 * @return  absolute speed MPS
 */
float getEncWheelSpeedMPS( void )
{
    /***    cm/s => m/s  ***/
    return ( speed_cmps * 0.01 );
}

/**
 * @brief   Get speed [km per second]
 * @return  absolute speed KPS
 */
float getEncWheelSpeedKPH( void )
{
    float wheel_speed_mps = getEncWheelSpeedMPS( );
    /***  m/s => km/h  ***/
    return( wheel_speed_mps * 0.001 * 3600 );
}
