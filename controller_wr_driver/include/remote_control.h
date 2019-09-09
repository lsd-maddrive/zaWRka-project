#ifndef INCLUDE_REMOTE_CONTROL_H_
#define INCLUDE_REMOTE_CONTROL_H_

typedef uint32_t    pwmValue_t;
typedef int32_t     icuControlValue_t;

/**
 * @brief   Initialize periphery connected to remote control
 * @note    Stable for repeated calls
 * @param   prio defines priority of inside thread
 *          IMPORTANT! NORMALPRIO + prio
 */
void remoteControlInit( tprio_t prio );

/**
 * @brief   Return speed control signal (width) in ticks
 * @return  width for speed
 * @note    Before this function
 *          rcModeIsEnabled must be checked
 */
pwmValue_t rcGetSpeedDutyCycleValue( void );

/**
 * @brief   Return steering control signal (width) in ticks
 * @return  width for steering
 * @note    Before this function
 *          rcModeIsEnabled must be checked
 */
pwmValue_t rcGetSteerDutyCycleValue( void );

/**
 * @brief   Detect working mode
 * @return  true    - RC mode enable
 *          false   - RC mode disable
 * @note    this function should be called before
 *          getting width value
 */
bool rcModeIsEnabled( void );

/**
 * @brief   Get control values for driving wheels
 * @return  control signal [-100; 100]
 * @note    Before this function
 *          rcModeIsEnabled must be called
 */
icuControlValue_t rcGetSpeedControlValue( void );

/**
 * @brief   Get control values for steering wheels
 * @return  control signal [-100; 100]
 * @note    Before this function
 *          rcModeIsEnabled must be checked
 */
icuControlValue_t rcGetSteerControlValue( void );

#endif /* INCLUDE_REMOTE_CONTROL_H_ */
