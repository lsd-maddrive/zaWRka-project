#include <tests.h>
#include <lld_light.h>
#include <lld_start_button.h>

/**
 * @brief   Test lights (LEDs) with different input conditions 
 */
void testLightRoutine( void )
{
	lldLightInit( NORMALPRIO ); 
	debug_stream_init( );

	turn_light_state	test_light_state = REMOTE; 

	float	test_steer			= 0;
	float	test_delta_steer	= 1;

	float 	test_speed			= 0;
	float	test_delta_speed	= 0.1; 

	system_state test_global_state	= IDLE; 
	
	systime_t	time = chVTGetSystemTimeX( ); 
	
	while( 1 )
	{
		char rc_data	= sdGetTimeout( &SD3, TIME_IMMEDIATE ); 

		switch( rc_data )
		{
			case 'a':
				test_steer += test_delta_steer;
				break;

			case 's':
				test_speed += test_delta_speed;
				break; 

			case ' ':
				test_steer = 0;
				test_speed = 0;
				break; 

			case 'd':
				test_global_state = RUN; 
				break; 

			case 'f':
				test_global_state = IDLE; 
				break; 

			default:
				; 
		}


		lldLightDetectTurnState( test_steer, test_speed, test_global_state );
		test_light_state = lldGetLightState( ); 

		dbgprintf( "Steer:(%d)\tSpeed:(%d)\tS_state:(%d)\tL_state:(%d)\n\r",
			(int)test_steer, (int)(test_speed * 100), test_global_state, test_light_state );

		time = chThdSleepUntilWindowed( time, time + 100 );
	}
}

/*
 * NEED COMMENT
 */
void testLedMatrixRoutine( void )
{
	palSetLine( LINE_LED1 );
	lldLightInit( NORMALPRIO );
	// palSetLine( LINE_LED2 );

	debug_stream_init( );

	systime_t time = chVTGetSystemTimeX( );
	// palSetLine( LINE_LED3 );
	while( 1 )
	{
		palToggleLine( LINE_LED3 );

		time = chThdSleepUntilWindowed( time, time + 100 );
	}

}