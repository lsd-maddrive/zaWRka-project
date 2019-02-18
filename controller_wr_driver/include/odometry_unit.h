#ifndef INCLUDE_ODOMETRY_UNIT_H_
#define INCLUDE_ODOMETRY_UNIT_H_

/*** Variables ***/
typedef int32_t rawEncoderValue_t;
typedef int32_t encoderValue_t;
typedef int32_t encSpeedValue_t;

/**
 * @brief   Initialize periphery connected to encoder control
 * @note    Stable for repeated calls
 */
void encoderInit( void );

///**
// * @brief   Get raw encoder value
// * @return  raw encoder values (ticks)
// */
//rawEncoderValue_t getEncoderRawTickNumber( void );
//
///**
// * @brief   Get encoder revs value
// * @return  number of motor revs
// */
//encoderValue_t getEncoderRevNumber( void );

/**
 * @brief   Get table values of encoder
 * @return  numbers [0; 3]
 * @note    0 -> 1 -> 3 -> 2 = backward = counterclockwise
 *          1 -> 0 -> 2 -> 3 = forward  = clockwise
 */
//rawEncoderValue_t getEncoderValTable( void );

/**
 * @brief   Get decimal values depends on 2 channels encoder state
 * @return  values [0, 3]
 */
//rawEncoderValue_t getEncoderState( void );

/**
 * @brief   Define direction of rotation
 * @return  'F' = forward  = clockwise
 *          'B' = backward = counterclockwise
 */
encoderValue_t getEncoderDirectionState( void );

/**
 * @brief   Get distance in cm
 * @return  (int) distance in cm
 */
encoderValue_t getEncoderDistanceCm( void );

/**
 * @brief   Get speed [ticks per second]
 * @return  (int) absolute speed TPS
 */
rawEncoderValue_t getEncoderSpeedTPS( void );

/**
 * @brief   Get speed [revs per second]
 * @return  (int) absolute speed RPS
 */
encoderValue_t getEncoderSpeedRPS( void );

/**
 * @brief   Get speed [cm per second]
 * @return  speed CMPS
 */
float getEncWheelSpeedCmPS( void );

/**
 * @brief   Get speed [metr per second]
 * @return  speed MPS
 */
float getEncWheelSpeedMPS( void );

/**
 * @brief   Get speed [km per second]
 * @return  speed KPS
 */
float getEncWheelSpeedKPH( void );

/**
 * @brief   Get orientation of Object
 * @note    tetta value is updated each 10 ms
 * @return  tetta angle in radians [rad]
 */
double getObjTetaAngleRad( void );

/**
 * @brief   Get orientation of Object
 * @note    tetta value is updated each 10 ms
 * @return  tetta angle in degrees [deg]
 */
double getObjTettaAngleDeg( void );

/**
 * @brief   Get X coordinate of Object
 * @note    X value is updated each 10 ms
 * @return  X value in meters [m]
 */
double getObjPosX( void );

/**
 * @brief   Get Y coordinate of Object
 * @note    Y value is updated each 10 ms
 * @return  Y value in meters [m]
 */
double getObjPosY( void );


#endif /* INCLUDE_ODOMETRY_UNIT_H_ */
