#ifndef INCLUDE_LLD_START_BUTTON_H_
#define INCLUDE_LLD_START_BUTTON_H_


typedef enum {
  IDLE  = 0,
  RUN   = 1
}system_state;


/**
 * @brief   Initialize EXT driver for button
 * @note    Stable for repeated calls
 */
void startButtonInit( tprio_t priority );

/**
 * @brief   Get system mode
 * @return  Values from mySystemState
 *          IDLE
 *          RUN
 */
system_state lldGetSystemState( void );

#endif /* INCLUDE_LLD_START_BUTTON_H_ */
