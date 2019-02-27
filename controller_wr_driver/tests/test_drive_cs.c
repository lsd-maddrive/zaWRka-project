#include <tests.h>
#include <drive_cs.h>

static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0,
  .cr2 = 0,
  .cr3 = 0
};



void testSteeringCS ( void )
{
    driveSteerCSInit( );
}
