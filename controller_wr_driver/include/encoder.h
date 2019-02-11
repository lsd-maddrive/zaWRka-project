#ifndef INCLUDE_ENCODER_H_
#define INCLUDE_ENCODER_H_

/*** Variables ***/
typedef uint32_t rawEncoderValue_t;
typedef uint32_t encoderValue_t;

/**
 * @brief   Initialize periphery connected to encoder control
 * @note    Stable for repeated calls
 */
void encoderInit( void );

/**
 * @brief   Get raw encoder value
 * @return  raw encoder values (ticks)
 */
rawEncoderValue_t getEncoderRawTickNumber( void );

/**
 * @brief   Get encoder revs value
 * @return  number of motor revs
 */
encoderValue_t getEncoderRevNumber( void );

/**
 * @brief   Get table values of encoder
 * @return  numbers [0; 3]
 * @note    0 -> 1 -> 3 -> 2 = backward = counterclockwise
 *          1 -> 0 -> 2 -> 3 = forward  = clockwise
 */
rawEncoderValue_t getEncoderValTable( void );

/**
 * @brief   Get decimal values depends on 2 channels encoder state
 * @return  values [0, 3]
 */
rawEncoderValue_t getEncoderState( void );

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


#endif /* INCLUDE_ENCODER_H_ */
