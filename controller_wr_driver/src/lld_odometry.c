#include <tests.h>
#include <lld_encoder.h>
#include <lld_odometry.h>
#include <lld_steer_angle_fb.h>


/*============================================================================*/
/* Coefficient Variables                                                      */
/*============================================================================*/

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

/*****************************************************************************/


/**
 * @brief   Get distance that object has passed
 * @return  Distance in specified units
 */
odometryValue_t lldGetOdometryObjDistance( odometryDistanceUnit_t units )
{
    rawRevEncoderValue_t   revs = lldGetEncoderRawRevs( );

    return ( revs * odom_dist_4_obj * units );
}

/*============================================================================*/
/* Variables for speed                                                        */
/*============================================================================*/

odometryRawSpeedValue_t     prev_revs           = 0;
odometryRawSpeedValue_t     revs_per_sec        = 0;

odometryValue_t             prev_distance       = 0;
odometrySpeedValue_t        speed_cm_per_sec    = 0;
odometrySpeedValue_t        speed_m_per_sec     = 0;

/*============================================================================*/
/* Variables for odometry                                                     */
/*============================================================================*/

odometryValue_t             tetta_rad_angle     = 0;
odometryValue_t             x_pos_m             = 0;
odometryValue_t             y_pos_m             = 0;

odometrySpeedValue_t        tetta_speed_rad_s   = 0;

#ifdef LOW_FREQ_CS
uint8_t                     speed_cs_count      = 0;
uint8_t                     speed_s_period      = 5;
odometrySpeedValue_t        speed_cs_cm_p_sec   = 0;
odometryValue_t             prev_dist_cs        = 0;
#endif
odometrySpeedValue_t        prev_speed_lpf      = 0;
odometrySpeedValue_t        speed_lpf           = 0;

#define SPEED_LPF               (float)0.8


#define                 VT_ODOM_MS              10
static virtual_timer_t  odom_update_vt;

/**
 * @brief   Get filtered by LPF speed of objects
 */
odometrySpeedValue_t lldOdometryGetLPFObjSpeedMPS( void )
{
    return speed_lpf;
}

#ifdef LOW_FREQ_CS
/**
 * @brief   Get speed of objects
 * @note    Low frequency = 20 Hz
 */
odometrySpeedValue_t lldOdometryGetObjCSSpeedMPS( void )
{
  return ( speed_cs_cm_p_sec * 0.01 );
}
#endif

static float correction_k_left  = 1; // 0.65;
static float correction_k_right = 1; // 0.6;

/*
 * @brief   Set correction coeffitients 
            to merge visual odometry and hardware-odometry 
 */
void lldOdometrySetCorrectionRates( float k_left, float k_right )
{
    correction_k_left   = k_left;
    correction_k_right  = k_right;
}

static void odom_update_vt_cb( void *arg )
{
    arg = arg;

/***         Speed calculation                         ***/

    /***         Get speed of encoder in revolutions       ***/
    rawRevEncoderValue_t cur_revs   = lldGetEncoderRawRevs( );

    revs_per_sec    = (cur_revs - prev_revs) * MS_2_SEC;

    prev_revs       = cur_revs;

    /*********************************************/

    /***    Get speed of object in cm/s and m/s          ***/
    odometryValue_t cur_distance  = lldGetOdometryObjDistance( OBJ_DIST_CM );
#ifdef LOW_FREQ_CS
    /*******************************************************/
    speed_cs_count += 1;
    if( speed_cs_count == 1 )
    {
        prev_dist_cs = cur_distance;
    }
    else if( speed_cs_count == (speed_s_period + 1) )
    {
        speed_cs_cm_p_sec = ( cur_distance - prev_dist_cs ) * 20 ;
        prev_dist_cs = cur_distance;
        speed_cs_count = 0;
    }
    /*******************************************************/
#endif

    /*  [cm/10ms] * 100 = [cm/s]   */
    speed_cm_per_sec    = (cur_distance - prev_distance ) * MS_2_SEC;
    /*  [cm/s] * 0.01 = [m/s]       */
    speed_m_per_sec     = speed_cm_per_sec * CM_2_M;

    /***    LPF - FILTER     ***/
    speed_lpf           = speed_m_per_sec * ( 1 - SPEED_LPF ) + prev_speed_lpf * SPEED_LPF;
    prev_speed_lpf      = speed_lpf;

    prev_distance       = cur_distance;
    /*********************************************/

    /***        Tetta calculation              ***/

    odometryValue_t         steer_angl_rad = lldGetSteerAngleRad( );

    /* TODO - Added for test, check if required */
    if ( steer_angl_rad > 0 )
        steer_angl_rad *= correction_k_left;
    else
        steer_angl_rad *= correction_k_right;

    tetta_speed_rad_s = ( speed_m_per_sec * tan( steer_angl_rad ) * tetta_k_rad );

    /*** It is tetta angle, not changing speed of tetta! ***/
    tetta_rad_angle +=  tetta_speed_rad_s;

    /*** Reset tetta integral ***/
    /*** NOTE!! 0 = 360       ***/
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


    chSysLockFromISR();
    chVTSetI(&odom_update_vt, MS2ST( VT_ODOM_MS ), odom_update_vt_cb, NULL);
    chSysUnlockFromISR();
}

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
#if 0
    /*** Start working GPT driver in asynchronous mode ***/
    gptStart( gptDriver, &gpt2cfg );
    uint32_t gpt_period = gptFreq * 0.01;   // 10 ms
    gptStartContinuous( gptDriver, gpt_period );
#endif
    chVTObjectInit(&odom_update_vt);
    chVTSet( &odom_update_vt, MS2ST( VT_ODOM_MS ), odom_update_vt_cb, NULL );

    lldEncoderInit( );

    lldSteerAngleFBInit( );

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
 * @return  Speed in revolutions per second [rev/s]
 */
odometryRawSpeedValue_t lldGetOdometryRawSpeedRPS( void )
{
    return revs_per_sec;
}

/**
 * @brief   Get speed of object
 * @return  Speed in cm per second [cm/s]
 */
odometrySpeedValue_t lldGetOdometryObjSpeedCMPS( void )
{
    return speed_cm_per_sec;
}

/**
 * @brief   Get speed of object
 * @return  Speed in m per second [m/s]
 */
odometrySpeedValue_t lldGetOdometryObjSpeedMPS( void )
{
    return speed_m_per_sec;
}

/**
 * @brief   Get speed of changing orientation of object
 * @return  Speed in radians per second [rad/s]
 */
odometrySpeedValue_t lldGetOdometryObjTettaSpeedRadPS( void )
{
    return tetta_speed_rad_s;
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

/**
 * @brief   Reset x, y, tetta
 * @note    x [m], y [m], tetta [rad]
 */
void lldResetOdometry( void )
{
    dbgprintf( "Called %s\n\r", __FUNCTION__ );
    lldResetEncoder( );

    prev_revs           = 0;
    prev_distance       = 0;

    x_pos_m             = 0;
    y_pos_m             = 0;
    tetta_rad_angle     = 0;
    tetta_speed_rad_s   = 0;
    speed_cm_per_sec    = 0;
}
