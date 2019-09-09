#ifndef INCLUDE_LLD_LIGHT_H_
#define INCLUDE_LLD_LIGHT_H_

#include <lld_start_button.h>

/**********************************/
/********* Turning lights *********/
/**********************************/

/***	Turning lights states	***/
typedef enum {
	RIGHT = 0,
	LEFT = 1,
	STOP = 2, 
	REMOTE	= 3
} turn_light_state; 


/**********************************/
/********* LED-matrix 8x8 *********/
/**********************************/

#define MAX_DIGITS          8	// led-matrix 8x8 
#define MAX_PRESET_NUMB     10	// number of led-matrix states 

/***	LED-matrix 8x8 states	***/
typedef enum{
  brick_sign        = 0,
  only_forward      = 1,
  only_right        = 2,
  only_left         = 3,
  forward_or_right  = 4,
  forward_or_left   = 5,
  happy_face        = 6,
  sad_face          = 7,
  tongue_face       = 8,
  parking           = 9

} sign;
/***********************************************/

/***********************************************/
/*********     LED-matrix Related      *********/
/***********************************************/
/*
 * @brief   Show figure in led-matrix 
 * @note    Be carefull with direction of installation of matrix
 * @param   figure shoulb be choosed from enum-table sign  
 */
void led_show(sign figure);

/***********************************************/
/*******     Turning lights Related      *******/
/***********************************************/

/**
 * @brief   Initialize periphery connected LEDs
 			Included: 
 						Turning lights
						LED-matrix 8x8 
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

/**
 * @brief	Reset state of turning lights
 			turn_state = REMOTE 
 */
void lldLightResetTurnState( void );

#endif /* INCLUDE_LLD_LIGHT_H_ */
