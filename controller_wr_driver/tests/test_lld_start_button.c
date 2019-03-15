#include <tests.h>
#include <lld_start_button.h>

void testButtonRoutine( void )
{
    debug_stream_init( );
    startButtonInit( NORMALPRIO );


    system_state            test_s_state    = IDLE;
    uint32_t                test_button_cnt = 0;

    systime_t time = chVTGetSystemTimeX();

    while( 1 )
    {
        time += MS2ST(100);
        test_s_state    = lldGetSystemState( );
        test_button_cnt = lldGetStartButtonPressedNumber( );

        dbgprintf( "SS:(%d)\tBC:(%d)\n\r",
                   (int)test_s_state, test_button_cnt );

        chThdSleepUntil(time);
    }
}


