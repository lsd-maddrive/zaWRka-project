#ifndef INCLUDE_LLD_ODOMETRY_H_
#define INCLUDE_LLD_ODOMETRY_H_

typedef double  odometryValue_t;
typedef int32_t odometryRawSpeedValue_t;
typedef double  odometrySpeedValue_t;

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
 * @note    units = 1   -> mm
 *          units = 10  -> cm
 *          units = 100 -> m
 */
odometryValue_t lldGetOdometryObjDistance( uint8_t units );

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
 * @note    units = 1   -> [ mm ]
 *          units = 10  -> [ cm ]
 *          units = 100 -> [ m  ]
 */
odometryValue_t lldGetOdometryObjX( uint8_t units );

/**
 * @brief   Get y-coordinate of objects
 * @return  object movement
 * @note    units = 1   -> [ mm ]
 *          units = 10  -> [ cm ]
 *          units = 100 -> [ m  ]
 */
odometryValue_t lldGetOdometryObjY( uint8_t units );

#endif /* INCLUDE_LLD_ODOMETRY_H_ */
