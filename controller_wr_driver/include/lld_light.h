#ifndef INCLUDE_LLD_LIGHT_H_
#define INCLUDE_LLD_LIGHT_H_

#include <lld_start_button.h>

typedef enum {
	RIGHT = 0,
	LEFT = 1,
	STOP = 2, 
	REMOTE	= 3
} turn_light_state; 

/**
 * @brief   Initialize periphery connected LEDs
 * @note    Stable for repeated calls
 */
void lldLightInit( tprio_t priority );

/*
 * @brief	Automatically set the state of turn lights
 *        	depends on input conditions 
 */
void lldLightDetectTurnState( float steer_cntrl, float speed_cntrl, system_state s_state );

/*
 * @brief	Get current state of turn lights  
 */
turn_light_state lldGetLightState( void );


#endif /* INCLUDE_LLD_LIGHT_H_ */
