#ifndef INCLUDE_LLD_CONTROL_H_
#define INCLUDE_LLD_CONTROL_H_

/*** Variables ***/
typedef int32_t     controlValue_t;
typedef uint32_t    rawPwmValue_t;


/**
 * @brief   Initialize periphery connected to driver control
 * @note    Stable for repeated calls
 */
void lldControlInit ( void );

/**
 * @brief   		   Set power for driving motor
 * @param   inputPrc   Motor power value [-100 100]
 */
void lldControlSetDrMotorPower( controlValue_t inputPrc );

/**
 * @brief   			Set power for steering motor
 * @param   inputPrc    Motor power value [-100 100]
 *                      100  	- max left
 *                      center 	- 0
 *                      -100 	- max right
 */
void lldControlSetSteerMotorPower( controlValue_t inputPrc );

/**
 * @brief   			Set power (in ticks) for driving motor
 * @param   drDuty  	dutycycle for speed control
 */
void lldControlSetDrMotorRawPower( rawPwmValue_t dutyCycleSpeed );

/**
 * @brief   			Set power (in ticks) for steering motor
 * @param   steerDuty   dutycycle for steering control
 */
void lldControlSetSteerMotorRawPower( rawPwmValue_t dutyCycleSteer );


/**
 * @brief   Get zero power
 */
rawPwmValue_t lldControlGetDrMotorZeroPower( void );

/**
 * @brief   Get zero power
 */
rawPwmValue_t lldControlGetSteerMotorZeroPower( void );

#endif /* INCLUDE_LLD_CONTROL_H_ */
