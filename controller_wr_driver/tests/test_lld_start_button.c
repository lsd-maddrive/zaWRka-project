#include <tests.h>
#include <lld_start_button.h>

/**
* @brief    Test start button routine 
*/
void testButtonRoutine( void )
{
    debug_stream_init( );
    startButtonInit( NORMALPRIO );


    system_state            test_s_state    = IDLE;

    systime_t time = chVTGetSystemTimeX();

    while( 1 )
    {
        test_s_state    = lldGetSystemState( );

        dbgprintf( "SS:(%d)\n\r", (int)test_s_state );

        time = chThdSleepUntilWindowed( time, time + MS2ST( 100 ) );
    }
}


