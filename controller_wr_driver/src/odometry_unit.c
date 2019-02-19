#include <tests.h>
#include <odometry_unit.h>

///* need checking, maybe should 0.105 / 6 */
//#define GAIN                0.105

///***    GPT Configuration Zone    ***/

#define gptFreq     1000 // 100 Hz => 0,01 s => 10 ms
#define gpt10ms     (int)( gptFreq * 0.01 )

static GPTDriver    *gptDriver              =   &GPTD2;
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
static void gpt2_cb (GPTDriver *gptd)
{
    gptd = gptd;
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

}

static const GPTConfig gpt2cfg = {
  .frequency =  gptFreq,
  .callback  =  gpt2_cb,
  .cr2       =  0,
  .dier      =  0U
};

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

static bool         isInitialized       = false;

/**
 * @brief   Initialize periphery connected to odometry unit
 * @note    Stable for repeated calls
 */
void encoderInit( void )
{
    if ( isInitialized )
            return;

    /* Start working GPT driver in asynchronous mode */
    gptStart( gptDriver, &gpt2cfg );
    gptStartContinuous( gptDriver, gptFreq );

    /* Set initialization flag */

    isInitialized = true;
}


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
