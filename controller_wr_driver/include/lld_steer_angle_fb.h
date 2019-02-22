#ifndef INCLUDE_LLD_STEER_ANGLE_FB_H_
#define INCLUDE_LLD_STEER_ANGLE_FB_H_

#define ADC_RES_CONF                ADC_CR1_12B_RESOLUTION

/*Steer sensor position input */
#define ADC_SEQ1_NUM                ADC_CHANNEL_IN10
#define ADC_SEQ1_LINE               PAL_LINE( GPIOC, 0 )
#define ADC_SEQ1_CH                 0

/**
 * @brief       Preparing to start ADC operations
*/
void InitAdc ( void );

/**
 * @brief       Get value of ADC channel
 * @return      Value of ADC sampling
*/
adcsample_t GetAdcVal ( void );



/* Additional ADC constants */
#define ADC_CR1_12B_RESOLUTION      (0)
#define ADC_CR1_10B_RESOLUTION      (ADC_CR1_RES_0)
#define ADC_CR1_8B_RESOLUTION       (ADC_CR1_RES_1)
#define ADC_CR1_6B_RESOLUTION       (ADC_CR1_RES_0 | ADC_CR1_RES_1)


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
