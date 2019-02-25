#include <tests.h>
#include <lld_encoder.h>
#include <lld_odometry.h>
#include <lld_steer_angle_fb.h>

/************************************/
/***    GPT Configuration Zone    ***/
/************************************/
#define gptFreq     10000
static  GPTDriver   *gptDriver = &GPTD2;
/************************************/

/************************************/
/***     Coefficient variables    ***/
/************************************/

float    odom_dist_4_obj    = 0;
/* need to check, maybe should 0.105 / 6 */
float    obj_gain       = 0.105;

float    tetta_k_rad    = 0;
float    tetta_k_deg    = 0;

float    d_t            = 0.01;

float    cm_2_mm        = 0;
float    cm_2_m         = 0;


#define  MS_2_SEC       100; // 10 ms - > 1 s
#define  CM_2_M         (float)0.01;

/************************************/


/**
 * @brief   Get distance that object has passed
 * @return  Distance in specified units
 */
odometryValue_t lldGetOdometryObjDistance( odometryDistanceUnit_t units )
{
    rawRevEncoderValue_t   revs = lldGetEncoderRawRevs( );

    return ( revs * odom_dist_4_obj * units );
}

/************************************/
/***     Variables for speed      ***/
/************************************/

float                       prev_revs           = 0;
odometryRawSpeedValue_t     revs_per_sec        = 0;

odometryValue_t             prev_distance       = 0;
odometrySpeedValue_t        speed_cm_per_sec    = 0;
odometrySpeedValue_t        speed_m_per_sec     = 0;

/************************************/
/***     Variables for odometry   ***/
/************************************/

odometryValue_t             tetta_rad_angle     = 0;
odometryValue_t             x_pos_m             = 0;
odometryValue_t             y_pos_m             = 0;




static void gptcb (GPTDriver *gptd)
{
    gptd = gptd;

    /***         Speed calculation                         ***/

    /***         Get speed of encoder in revolutions       ***/
    rawRevEncoderValue_t   cur_revs   = lldGetEncoderRawRevs( );

    revs_per_sec    = (cur_revs - prev_revs) * MS_2_SEC;

    prev_revs       = cur_revs;
    /*********************************************/

    /***    Get speed of object in cm/s and m/s          ***/
    odometryValue_t cur_distance  = lldGetOdometryObjDistance( OBJ_DIST_CM );

    /*  [cm/10ms] * 1000 = [cm/c]   */
    speed_cm_per_sec    = (cur_distance - prev_distance ) * MS_2_SEC;
    /*  [cm/s] * 0.01 = [m/s]       */
    speed_m_per_sec     = speed_cm_per_sec * 0.01;//CM_2_M;

    prev_distance       = cur_distance;
    /*********************************************/

    /***        Tetta calculation              ***/
    /* steer_angle = getSteerAngleRadValue() !!!!! need to add !!!*/
    steerAngleRawValue_t    steer_mean_adc_val = lldGetFiltrMeanRawADCSteerAngleFB( );
    odometryValue_t         steer_angl_rad = lldGetRadSteerAngleFB( steer_mean_adc_val );
    /*** It is tetta angle, not changing speed of tetta! ***/
    tetta_rad_angle +=  ( speed_m_per_sec * tan( steer_angl_rad ) * tetta_k_rad );

    /*** Reset tetta integral ***/
    /*** NOTE 0 = 360         ***/
    if(tetta_rad_angle > ( 2 * M_PI ) )
    {
            tetta_rad_angle   = tetta_rad_angle - ( 2 * M_PI );
    }
    if( tetta_rad_angle < 0 )
    {
        tetta_rad_angle = ( 2 * M_PI ) + tetta_rad_angle;
    }
    /**********************************************/

    /***        X calculation                  ***/
    x_pos_m += (speed_m_per_sec * cos(tetta_rad_angle)) * d_t;
    /*********************************************/

    /***        Y calculation                  ***/
    y_pos_m += (speed_m_per_sec * sin(tetta_rad_angle)) * d_t;
    /*********************************************/

}

static const GPTConfig gpt2cfg = {
  .frequency =  gptFreq,
  .callback  =  gptcb,
  .cr2       =  0,
  .dier      =  0U
};

static bool         isInitialized       = false;

/**
 * @brief   Initialize periphery connected to odometry unit
 * @note    Stable for repeated calls
 *          lldEncoderInit is done in this function,
 *          do not initialize it again!
 */
void lldOdometryInit( void )
{
    if ( isInitialized )
            return;

    /*** Start working GPT driver in asynchronous mode ***/
    gptStart( gptDriver, &gpt2cfg );
    uint32_t gpt_period = gptFreq * 0.01;   // 10 ms
    gptStartContinuous( gptDriver, gpt_period );

    lldEncoderInit( );

    /***    Coefficient calculation     ***/
    odom_dist_4_obj = 2 * M_PI * WHEEL_RADIUS_M * obj_gain;

    tetta_k_rad = d_t / WHEEL_BASE_M;
    tetta_k_deg = 180 / M_PI;

    cm_2_mm       = 10;
    cm_2_m        = 0.01;

    /***    Set initialization flag     ***/

    isInitialized = true;
}

/**
 * @brief   Get speed of encoder rotation
 * @return  Speed in revolutions per second [rps]
 */
odometryRawSpeedValue_t lldGetOdometryRawSpeedRPS( void )
{
    return revs_per_sec;
}

/**
 * @brief   Get speed of object
 * @return  Speed in cm per second [cmps]
 */
odometrySpeedValue_t lldGetOdometryObjSpeedCMPS( void )
{
    return speed_cm_per_sec;
}

/**
 * @brief   Get speed of object
 * @return  Speed in m per second [mps]
 */
odometrySpeedValue_t lldGetOdometryObjSpeedMPS( void )
{
    return speed_m_per_sec;
}


/**
 * @brief   Get tetta (orientation) of objects
 * @return  angle in radians [rad]
 */
odometryValue_t lldGetOdometryObjTettaRad( void )
{
    return tetta_rad_angle;
}

/**
 * @brief   Get tetta (orientation) of objects
 * @return  angle in degrees [deg]
 */
odometryValue_t lldGetOdometryObjTettaDeg( void )
{
    return ( tetta_rad_angle * tetta_k_deg );
}

/**
 * @brief   Get x-coordinate of objects
 * @return  object movement
 */
odometryValue_t lldGetOdometryObjX( odometryDistanceUnit_t units )
{
  return ( x_pos_m * units );
}

/**
 * @brief   Get y-coordinate of objects
 * @return  object movement
 */
odometryValue_t lldGetOdometryObjY( odometryDistanceUnit_t units )
{
    return ( y_pos_m * units );
}

