#ifndef INCLUDE_LLD_ENCODER_H_
#define INCLUDE_LLD_ENCODER_H_

typedef int32_t    rawEncoderValue_t;
typedef int32_t    EncoderValue_t;
typedef char    dirEncoderValue_t;


/**
 * @brief   Initialize periphery connected to encoder
 * @note    Stable for repeated calls
 */
void lldEncoderInit( void );


/**
 * @brief   Get raw encoder value
 * @return  raw encoder values (ticks)
 */
rawEncoderValue_t getEncoderRawTickNumber( void );

/**
 * @brief   Get encoder revolutions number
 * @return  number of motor revs
 */
rawEncoderValue_t getEncoderRevsNumber( void );

/**
 * @brief   Get decimal values depends on 2 channels encoder state
 * @return  values [0, 3]
 */
rawEncoderValue_t getEncoderState( void );

dirEncoderValue_t getEncoderDirection( void );


#endif /* INCLUDE_LLD_ENCODER_H_ */
