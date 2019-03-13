#include <tests.h>
#include <lld_steer_angle_fb.h>
#include <lld_encoder.h>
#include <ros_protos.h>


void testGUIRoutineServer( void )
{
    ros_driver_init( NORMALPRIO, NULL );
    lldEncoderInit( );
    lldSteeringControlInit( );

    steerAngleDegValue_t    deg_steer_angle = 0;
    rawEncoderValue_t       enc_revs = 0;

    while( 1 )
    {
        deg_steer_angle = lldGetSteerAngleDeg( );
        enc_revs        = lldGetEncoderRawRevs( );

        ros_driver_send_encoder_raw( enc_revs );

        ros_driver_send_steering( deg_steer_angle );

        chThdSleepMilliseconds( 100 );
    }
}
