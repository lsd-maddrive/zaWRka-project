//#include <tests.h>
//#include <odometry_unit.h>
//#include <math.h>
//
//#define MAX_TICK_NUM        360
//
///* need checking, maybe should 0.105 / 6 */
//#define GAIN                0.105
//
//#define ENCODER_GREEN_LINE  PAL_LINE( GPIOD, 3 )
//#define ENCODER_WHITE_LINE  PAL_LINE( GPIOD, 4 )
//
//rawEncoderValue_t   enc_trigger_counter = 0;
//encoderValue_t      enc_rev_number      = 0;
//encoderValue_t      enc_dir             = 0;
//
//rawEncoderValue_t   enc_table_val       = 0;
//rawEncoderValue_t   curr_enc_state      = 0;
//rawEncoderValue_t   prev_enc_state      = 0;
//
//rawEncoderValue_t   encoder_decode_table[4] = {0, 1, 3, 2};
//
///***    GPT Configuration Zone    ***/
//
//#define gptFreq     1000 // 100 Hz => 0,01 s => 10 ms
//#define gpt10ms     (int)( gptFreq * 0.01 )
//
//static GPTDriver    *gptDriver              =   &GPTD2;
//
//rawEncoderValue_t   ticks_per_sec      = 0;
//rawEncoderValue_t   prev_trg_counter   = 0;
//
//encoderValue_t      revs_per_sec       = 0;
//encoderValue_t      prev_rev_number    = 0;
//
//float               speed_cm_per_sec   = 0;
//float               speed_m_per_sec    = 0;
//
//encoderValue_t      prev_dist          = 0;
//
//uint32_t            gpt_counter        = 0;
//
//double               tetta_rad_angle_per_sec = 0;
//double               tetta_deg_angle_per_sec = 0;
//
//double               x_pos_m = 0;
//double               y_pos_m = 0;
//
//static void gpt2_cb (GPTDriver *gptd)
//{
//    gptd = gptd;
//
////    gpt_counter += 1;
//
//    int32_t cur_dist    = getEncoderDistanceCm();
//
//    float t_period      = 0.1; // 10 ms
//
//    /***    Get speed in ticks and revs        ***/
//    ticks_per_sec  = (enc_trigger_counter - prev_trg_counter) * t_period * 1000;
//    revs_per_sec   = enc_rev_number - prev_rev_number;
//
//    prev_trg_counter = enc_trigger_counter;
//    prev_rev_number  = enc_rev_number;
//    /*********************************************/
//
//    /***    Get speed in cm/s and m/s          ***/
//    /*  [cm/10ms] * 1000 = [cm/c]   */
//    speed_cm_per_sec    = (cur_dist - prev_dist ) * t_period * 1000;
//    /*  [cm/s] * 0.01 = [m/s]       */
//    speed_m_per_sec     = speed_cm_per_sec * 0.01;
//
//    prev_dist = cur_dist;
//    /*********************************************/
//
//    /***        Tetta calculation              ***/
//    /* steer_angle = getSteerAngleRadValue() !!!!! need to add !!!*/
//    float steer_angl_rad = 0; // just to avoid errors!!!
//    tetta_rad_angle_per_sec +=  ((speed_m_per_sec * tan( steer_angl_rad )) / WHEEL_BASE_M) * t_period * 1000;
//    tetta_deg_angle_per_sec = tetta_rad_angle_per_sec * 180 / 3.14;
//    /*OR*/
//    /*tetta_grad_angle_per_sec = tetta_rad_angle_per_sec * 57.3;*/
//    if(tetta_deg_angle_per_sec == 360) tetta_rad_angle_per_sec = 0;
//    /**********************************************/
//
//    /***        X calculation                  ***/
//    x_pos_m += (speed_m_per_sec * cos(tetta_rad_angle_per_sec)) * t_period * 1000;
//    /*********************************************/
//
//    /***        Y calculation                  ***/
//    y_pos_m += (speed_m_per_sec * sin(tetta_rad_angle_per_sec)) * t_period * 1000;
//    /*********************************************/
//
//    palToggleLine( LINE_LED2 );
//
//}
//
//static const GPTConfig gpt2cfg = {
//  .frequency =  gptFreq,
//  .callback  =  gpt2_cb,
//  .cr2       =  0,
//  .dier      =  0U
//};
//
/////**
//// * @brief   Get decimal values depends on 2 channels encoder state
//// * @return  values [0, 3]
//// */
////rawEncoderValue_t getEncoderState( void )
////{
////    rawEncoderValue_t res_enc = 0;
////    if( palReadLine( ENCODER_GREEN_LINE ) )     res_enc |= 0b01;
////    if( palReadLine( ENCODER_WHITE_LINE ) )     res_enc |= 0b10;
////
////    return res_enc;
////}
//
//
//static void extcb1(EXTDriver *extp, expchannel_t channel)
//{
//    (void)extp;
//    (void)channel;
//
//
//    uint8_t i = 0;
//
//    rawEncoderValue_t encoder_state = getEncoderState( );
//
//    for( i = 0; i < 4; i++ )
//    {
//        if( encoder_state == encoder_decode_table[i])
//        {
//            curr_enc_state = i;
//            break;
//        }
//    }
//
//    if( curr_enc_state > prev_enc_state )
//    {
//        if( prev_enc_state == 0 && curr_enc_state == 3 ) enc_dir = 'F';
//        else    enc_dir = 'B';
//    }
//    else
//    {
//        if( prev_enc_state == 3 && curr_enc_state == 0 ) enc_dir = 'B';
//        else    enc_dir = 'F';
//    }
//
//    prev_enc_state = curr_enc_state;
//
//    enc_trigger_counter += 1;
//
//    if( enc_trigger_counter > (MAX_TICK_NUM * 2) )      enc_trigger_counter = 0;
//    if( enc_trigger_counter == (MAX_TICK_NUM * 2) )
//    {
//        if( enc_dir == 'F') enc_rev_number += 1;
//        else enc_rev_number -= 1;
//    }
//
//    enc_table_val = encoder_state;
//}
//
//
//static const EXTConfig extcfg =
//{
//   {
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOD , extcb1}, // PD3
//    {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOD , extcb1}, // PD4
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL}
//  }
//};
//
//static bool         isInitialized       = false;
//
///**
// * @brief   Initialize periphery connected to encoder control
// * @note    Stable for repeated calls
// */
//void encoderInit( void )
//{
//    if ( isInitialized )
//            return;
//
//    extStart( &EXTD1, &extcfg );
//
//    /* Start working GPT driver in asynchronous mode */
//    gptStart( gptDriver, &gpt2cfg );
//    gptStartContinuous( gptDriver, gptFreq );
//
//    /* Set initialization flag */
//
//    isInitialized = true;
//}
//
/////**
//// * @brief   Get raw encoder value
//// * @return  raw encoder values (ticks)
//// */
////rawEncoderValue_t getEncoderRawTickNumber( void )
////{
////    return enc_trigger_counter;
////}
//
//
/////**
//// * @brief   Get encoder revs value
//// * @return  number of motor revs
//// */
////encoderValue_t getEncoderRevNumber( void )
////{
////    return enc_rev_number;
////}
//
///**
// * @brief   Define direction of rotation
// * @return  'F' = forward  = clockwise
// *          'B' = backward = counterclockwise
// */
//encoderValue_t getEncoderDirectionState( void )
//{
//
//    return enc_dir;
//}
//
///**
// * @brief   Get table values of encoder
// * @return  numbers [0; 3]
// * @note    0 -> 1 -> 3 -> 2 = backward = counterclockwise
// *          1 -> 0 -> 2 -> 3 = forward  = clockwise
// */
//rawEncoderValue_t getEncoderValTable( void )
//{
//    return enc_table_val;
//}
//
///**
// * @brief   Get distance in cm
// * @return  (int) distance in cm
// */
//encoderValue_t getEncoderDistanceCm( void )
//{
//    encoderValue_t distance = 0;
//    encoderValue_t revs = 0;
//    revs = getEncoderRevNumber( );
//    distance = revs * 2 * 3.14 * WHEEL_RADIUS_CM * GAIN;
//    return distance;
//}
//
///**
// * @brief   Get speed [ticks per second]
// * @return  (int) absolute speed TPS
// */
//rawEncoderValue_t getEncoderSpeedTPS( void )
//{
//  return ticks_per_sec;
//}
//
///**
// * @brief   Get speed [revs per second]
// * @return  (int) absolute speed RPS
// */
//encoderValue_t getEncoderSpeedRPS( void )
//{
//  return revs_per_sec;
//}
//
///**
// * @brief   Get speed [cm per second]
// * @return  speed CMPS
// */
//float getEncWheelSpeedCmPS( void )
//{
//    return speed_cm_per_sec;
//}
//
///**
// * @brief   Get speed [metr per second]
// * @return  speed MPS
// */
//float getEncWheelSpeedMPS( void )
//{
//    return speed_m_per_sec;
//}
//
///**
// * @brief   Get speed [km per second]
// * @return  speed KPS
// */
//float getEncWheelSpeedKPH( void )
//{
//    /***  m/s => km/h  ***/
//    return( speed_m_per_sec * 0.001 * 3600 );
//}
//
//
///**
// * @brief   Get orientation of Object
// * @note    tetta value is updated each 10 ms
// * @return  tetta angle in radians [rad]
// */
//double getObjTetaAngleRad( void )
//{
//    return tetta_rad_angle_per_sec;
//}
//
///**
// * @brief   Get orientation of Object
// * @note    tetta value is updated each 10 ms
// * @return  tetta angle in degrees [deg]
// */
//double getObjTettaAngleDeg( void )
//{
//    return tetta_deg_angle_per_sec;
//}
//
///**
// * @brief   Get X coordinate of Object
// * @note    X value is updated each 10 ms
// * @return  X value in meters [m]
// */
//double getObjPosX( void )
//{
//    return x_pos_m;
//}
//
///**
// * @brief   Get Y coordinate of Object
// * @note    Y value is updated each 10 ms
// * @return  Y value in meters [m]
// */
//double getObjPosY( void )
//{
//    return y_pos_m;
//}
