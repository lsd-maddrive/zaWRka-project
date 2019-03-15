#include <tests.h>
#include <lld_odometry.h>
#include <drive_cs.h>


virtual_timer_t         checker_vt;

float                   test_steer_cntrl    = 10;
float                   test_speed_cntrl    = 0.1;

systime_t               t_check_time = 0;

static void is_dead_cb( void *arg )
{
    dbgprintf( "No command\n\r" );
    test_steer_cntrl    = 0;
    test_speed_cntrl    = 0;
    palSetLine( LINE_LED3 );
}

void is_alive( void )
{
    t_check_time = chVTGetSystemTimeX();
    palClearLine( LINE_LED3 );
    dbgprintf( "A!\tT:(%d)\n\r", ST2MS(t_check_time) );
    chVTSet( &checker_vt, MS2ST( 500 ), is_dead_cb, NULL );
}



void testSimulationRosControlRoutine( void )
{

    lldOdometryInit( );
    driverCSInit( NORMALPRIO );
    debug_stream_init( );

//    chVTObjectInit(&checker_vt);
//    chVTSet( &checker_vt, MS2ST( 500 ), is_dead_cb, NULL );

    odometryValue_t         test_x_pos          = 0;
    odometryValue_t         test_y_pos          = 0;
    odometryValue_t         test_tetta_deg      = 0;

    odometrySpeedValue_t    test_speed_mps      = 0;
    odometryRawSpeedValue_t test_enc_speed_rps  = 0;
    odometrySpeedValue_t    test_speed_radps    = 0;

    float                   test_steer_angl_deg = 0;
    float                   test_speed_lpf_mps  = 0;

    systime_t time = chVTGetSystemTimeX();

    while( 1 )
    {

//        char rc_data = sdGetTimeout( &SD3, TIME_IMMEDIATE );
//        switch( rc_data )
//        {
//          case 'a':
//
//            break;
//          case 's':
//            break;
//          default:
//            ;
//        }
//
//        driveSteerCSSetPosition( test_steer_cntrl );
//        driveSpeedCSSetSpeed( test_speed_cntrl );
//
//        test_enc_speed_rps  = lldGetOdometryRawSpeedRPS( );
//        test_speed_mps      = lldGetOdometryObjSpeedMPS( );
//        test_speed_radps    = lldGetOdometryObjTettaSpeedRadPS( );
//
//        test_steer_angl_deg = lldGetSteerAngleDeg( );
//        test_speed_lpf_mps  = lldOdometryGetLPFObjSpeedMPS( );
//
//        test_x_pos          = lldGetOdometryObjX( OBJ_DIST_M );
//        test_y_pos          = lldGetOdometryObjY( OBJ_DIST_M );
//        test_tetta_deg      = lldGetOdometryObjTettaDeg( );

//        is_alive( );

        dbgprintf("Time:%d\n\r", ST2MS(time));

        /*time =*/ chThdSleepUntilWindowed( time, time + MS2ST( 100 ) );

    }



}
