#ifndef INCLUDE_LLD_STEER_ANGLE_FB_H_
#define INCLUDE_LLD_STEER_ANGLE_FB_H_

typedef uint32_t    steerAngleRawValue_t;
typedef float       steerAngleRadValue_t;
typedef float       steerAngleDegValue_t;



/**
 * @brief       Initialization of unit for steering angle feedback
 * @note        ADC initialization and GPT initialization
*/
void lldSteerAngleFBInit ( void );

/*============================================================================*/
/* ADC FUNCS                                                                  */
/* ===========================================================================*/

/**
 * @brief       Get raw value of steering angle
 * @return      ADC value [0; 4096]
*/
steerAngleRawValue_t lldGetSteerAngleRawADC (void);

/***     ADC FILTER FUNCS    ***/

/**
 * @brief       Get raw filtered value of steering angle
 * @return      ADC value [0; 4096]
 * @note        Mean filter
 *              NEED TO SET
 *              STEER_ACTIVE_FILTER = STEER_FILTER_MEAN
*/
steerAngleRawValue_t lldGetSteerAngleFiltrMeanRawADC ( void );

/**
 * @brief       Get raw filtered value of steering angle
 * @return      ADC value [0; 4096]
 * @note        Low pass Filter ( LPF )
 *              NEED TO SET
 *              STEER_ACTIVE_FILTER = STEER_FILTER_LPF
*/
steerAngleRawValue_t lldGetSteerAngleFiltrLPFRawADC ( void );

/*============================================================================*/
/* ANGLE FUNCS                                                                */
/* ===========================================================================*/

/**
 * @brief       Get steering angle [rad]
 * @return      max_right   ->  STEER_RAD_RIGHT
 *              center      ->  STEER_RAD_CENTER
 *              max_left    ->  STEER_RAD_LEFT
*/
steerAngleRadValue_t lldGetSteerAngleRad ( void );

/**
 * @brief       Get steering angle [deg]
 * @return      max_right   ->  -34
 *              center      ->  0
 *              max_left    ->  28
 *@note         IMPORTANT!
 *              Use AFTER Initialization
*/
steerAngleDegValue_t lldGetSteerAngleDeg( void );

#endif /* INCLUDE_LLD_STEER_ANGLE_FB_H_ */
