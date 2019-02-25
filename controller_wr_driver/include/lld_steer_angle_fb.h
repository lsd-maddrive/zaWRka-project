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

/**
 * @brief       Get raw value of steering angle
 * @return      ADC value [0; 4096]
*/
steerAngleRawValue_t lldGetRawADCSteerAngleFB (void);

/**
 * @brief       Get raw filtered value of steering angle
 * @return      ADC value [0; 4096]
 * @note        Mean filter
*/
steerAngleRawValue_t lldGetFiltrMeanRawADCSteerAngleFB ( void );

/**
 * @brief       Get steering angle [rad]
 * @return      max_right   ->  STEER_RAD_RIGHT
 *              center      ->  STEER_RAD_CENTER
 *              max_left    ->  STEER_RAD_LEFT
*/
steerAngleRadValue_t lldGetRadSteerAngleFB ( steerAngleRawValue_t steer_adc_val );

/**
 * @brief       Get steering angle [deg]
 * @return      max_right   ->  -34
 *              center      ->  0
 *              max_left    ->  28
 *@note         IMPORTANT!
 *              Use AFTER Initialization
*/
steerAngleDegValue_t lldGetDegSteerAngleFB( steerAngleRadValue_t rad_angle );



/***    NOT FIXED    ***/

typedef int16_t           degSteerAngleValue_t;

/*
 * @brief   Initialize front wheels control
 */
void lldSteeringControlInit (void);

/*
 * @brief   Get ADC value
 * @return  ADC value from 0 to 4095 equal front wheels position
 */
int16_t lldSteeringControlGetAdcVal (void);

/*
 * @brief   Get ADC filtered value
 * @return  ADC value from 0 to 4095 equal front wheels position
 */
int16_t lldSteeringControlGetAdcVal_Kalman (void);

/*
 * @brief   Get ADC filtered value
 * @return  ADC value from 0 to 4095 equal front wheels position
 */
int16_t lldSteeringControlGetAdcVal_filt (void);

/*
 * @brief   Get ADC double-filtered value
 * @return  ADC value from 0 to 4095 equal front wheels position
 */
int16_t lldSteeringControlGetAdcVal_doublefilt (void);


/*
 * @brief   Get angle of rotate
 * @return  Angle of wheels rotate
 */
int16_t lldGetSteerDegAngle (void);

#endif /* INCLUDE_LLD_STEER_ANGLE_FB_H_ */
