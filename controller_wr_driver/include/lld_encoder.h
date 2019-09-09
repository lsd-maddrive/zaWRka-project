#ifndef INCLUDE_LLD_ENCODER_H_
#define INCLUDE_LLD_ENCODER_H_

typedef int32_t    rawEncoderValue_t;
typedef float      rawRevEncoderValue_t;

/**
 * @brief   Initialize periphery connected to encoder
 * @note    Stable for repeated calls
 */
void lldEncoderInit( void );

/**
 * @brief   Get number of encoder ticks
 * @note    Max number of ticks is defined by MAX_TICK_NUM
 * @return  Encoder ticks number depends on direction of rotation
 *          [0; ENC_MAX_TICK_NUM]
 *          after ENC_MAX_TICK_NUM it resets
 */
rawEncoderValue_t lldGetEncoderRawTicks( void );

/**
 * @brief   Get direction of encoder rotation
 * @return  clockwise           -> 0
 *          counterclockwise    -> 1
 */
bool lldGetEncoderDirection( void );

/**
 * @brief   Get number of encoder revolutions [double]
 * @note    1 revolution = MAX_TICK_NUM ticks
 * @return  Encoder revolutions number depends on direction of rotation
 */
rawRevEncoderValue_t   lldGetEncoderRawRevs( void );


/***    In case of ABSOLUTE ENCODER     ***/
/**
 * @brief   Get number of encoder revolutions [int]
 * @note    If you use absolute encoder!!!
 * @return  Encoder revolutions number depends on direction of rotation
 */
rawEncoderValue_t   lldGetAbsoluteEncoderRawRevs( void );

/*
 * @brief   Reset all variable for lld-encoder unit
 */
void lldResetEncoder( void );

#endif /* INCLUDE_LLD_ENCODER_H_ */
