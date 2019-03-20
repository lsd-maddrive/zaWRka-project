#include <tests.h>
#include <lld_start_button.h>

void testButtonRoutine( void )
{
    debug_stream_init( );
    startButtonInit( NORMALPRIO );


    system_state            test_s_state    = IDLE;

    systime_t time = chVTGetSystemTimeX();

    while( 1 )
    {
        time += MS2ST(100);
        test_s_state    = lldGetSystemState( );

        dbgprintf( "SS:(%d)\n\r",
                   (int)test_s_state );

        chThdSleepUntil(time);
    }
}


