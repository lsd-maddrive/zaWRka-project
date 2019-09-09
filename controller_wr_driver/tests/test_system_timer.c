#include <common.h>

/*
 * @brief   Routine of system timer test
 */
void testSystemTimer( void )
{
    debug_stream_init();

    systime_t time = chVTGetSystemTimeX();

    while ( 1 )
    {
        time += MS2ST(1000);

        dbgprintf( "Time: %d\n\r", chVTGetSystemTimeX() );

        chThdSleepUntil( time );
    }
}
