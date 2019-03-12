#ifndef INCLUDE_LLD_START_BUTTON_H_
#define INCLUDE_LLD_START_BUTTON_H_


typedef enum mySystemState{
  IDLE  = 0,
  WAIT  = 1,
  RUN   = 2
}system_state;


/**
 * @brief   Initialize EXT driver for button
 * @note    Stable for repeated calls
 */
void startButtonInit( int8_t priority );

/**
 * @brief   Get number of presses on button
 */
uint32_t lldGetStartButtonPressedNumber( void );

/**
 * @brief   Get system mode
 * @return  Values from mySystemState
 *          IDLE
 *          WAIT
 *          RUN
 */
system_state lldGetSystemState( void );


#endif /* INCLUDE_LLD_START_BUTTON_H_ */
