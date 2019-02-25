#ifndef INCLUDE_LLD_ODOMETRY_H_
#define INCLUDE_LLD_ODOMETRY_H_

typedef float  odometryValue_t;
typedef float  odometryRawSpeedValue_t;
typedef float  odometrySpeedValue_t;

typedef enum mySpeedUnits{
  MM_S  = 1000,
  CM_S  = 100,
  M_S   = 1
}odometrySpeedUnit_t;

typedef enum myDistUnits{
  OBJ_DIST_MM   = 1000,
  OBJ_DIST_CM   = 100,
  OBJ_DIST_M    = 1
}odometryDistanceUnit_t;


/**
 * @brief   Initialize periphery connected to odometry unit
 * @note    Stable for repeated calls
 *          lldEncoderInit is done in this function,
 *          do not initialize it again!
 */
void lldOdometryInit( void );

/**
 * @brief   Get distance that object has passed
 * @return  Distance in specified units
 */
odometryValue_t lldGetOdometryObjDistance( odometryDistanceUnit_t units );


/**********************************/
/***    Functions for speed     ***/
/**********************************/

/**
 * @brief   Get speed of encoder rotation
 * @return  Speed in ticks per second [tps]
 */
odometryRawSpeedValue_t lldGetOdometryRawSpeedRPS( void );

/**
 * @brief   Get speed of object
 * @return  Speed in cm per second [cmps]
 */
odometrySpeedValue_t lldGetOdometryObjSpeedCMPS( void );

/**
 * @brief   Get speed of object
 * @return  Speed in m per second [mps]
 */
odometrySpeedValue_t lldGetOdometryObjSpeedMPS( void );

/**********************************/
/***    Functions for odometry  ***/
/**********************************/

/**
 * @brief   Get tetta (orientation) of objects
 * @return  angle in radians [rad]
 */
odometryValue_t lldGetOdometryObjTettaRad( void );

/**
 * @brief   Get tetta (orientation) of objects
 * @return  angle in degrees [deg]
 */
odometryValue_t lldGetOdometryObjTettaDeg( void );

/**
 * @brief   Get x-coordinate of objects
 * @return  object movement
 */
odometryValue_t lldGetOdometryObjX( odometryDistanceUnit_t units );

/**
 * @brief   Get y-coordinate of objects
 * @return  object movement
 */
odometryValue_t lldGetOdometryObjY( odometryDistanceUnit_t units );

#endif /* INCLUDE_LLD_ODOMETRY_H_ */
