#include <common.h>
#include <lld_start_button.h>
#include <lld_odometry.h>
#include <lld_encoder.h>
#include <drive_cs.h>
#include <remote_control.h>
#include <lld_light.h>

#include "link_def.h"


#define VT_ROS_CHECK_PERIOD_MS     500

virtual_timer_t         link_check_vt;

#define VT_IDLE_MS                  10
#define VT_WAIT_MS                  10
#define VT_RUN_MS                   10

systime_t               time_checker_vt = 0;

//system_state            state_now      = IDLE;

float                   _steer_control   = 0;
float                   _speed_control   = 0;

odometryValue_t         x_pos          = 0;
odometryValue_t         y_pos          = 0;
odometryValue_t         tetta_deg      = 0;

odometrySpeedValue_t    speed_mps      = 0;
odometryRawSpeedValue_t enc_speed_rps  = 0;
odometrySpeedValue_t    speed_radps    = 0;

float                   steer_angl_deg = 0;
float                   odom_speed_lpf_mps  = 0;

/*
 * Check if ROS is spamming with control values,
 * if not, stops the car
 */
static void link_dead_cb( void *arg )
{
    arg = arg;  // to avoid warnings

    _speed_control = 0;
    _steer_control = 0;

    dbgprintf( "ROS is dead!\n\r", time_checker_vt);
}

void link_alive( void )
{
    chVTSet( &link_check_vt, MS2ST( VT_ROS_CHECK_PERIOD_MS ), link_dead_cb, NULL );
}

void _control_handler( float speed, float steer )
{
    _speed_control = speed;
    _steer_control = steer;
    link_alive( );
}

static THD_WORKING_AREA(waStateSender, 128); // 128 - stack size
static THD_FUNCTION(StateSender, arg)
{
    arg = arg;            // to avoid warnings

    while(1)
    {
        system_state state_now  = lldGetSystemState( );
        mproto_driver_send_state( state_now );

        chThdSleepMilliseconds( 200 );
    }
}

/**
 * @brief   Initialize all base units
 */
void mainUnitsInit( void )
{
    lldOdometryInit( );
    driverCSInit( NORMALPRIO );
    debug_stream_init( );
    startButtonInit( NORMALPRIO + 1 );
    remoteControlInit( NORMALPRIO );
    lldLightInit( NORMALPRIO - 1 );

    mproto_driver_cb_ctx_t cb_ctx   = mproto_driver_get_new_cb_ctx();
    cb_ctx.cmd_cb                   = _control_handler;
    cb_ctx.reset_odometry_cb        = lldResetOdometry;

    mproto_driver_init( NORMALPRIO, &cb_ctx );

    chThdCreateStatic(waStateSender, sizeof(waStateSender), NORMALPRIO - 1, StateSender, NULL);

    chVTObjectInit(&link_check_vt);

    dbgprintf( "INIT done!\n\r");
}

void sendOdometryToRos( void )
{
    speed_mps      = lldGetOdometryObjSpeedMPS( );
    speed_radps    = lldGetOdometryObjTettaSpeedRadPS( );

    steer_angl_deg = lldGetSteerAngleDeg( );
    odom_speed_lpf_mps  = lldOdometryGetLPFObjSpeedMPS( );

    x_pos          = lldGetOdometryObjX( OBJ_DIST_M );
    y_pos          = lldGetOdometryObjY( OBJ_DIST_M );
    tetta_deg      = lldGetOdometryObjTettaDeg( );

    mproto_driver_send_pose( x_pos, y_pos, tetta_deg, speed_mps, speed_radps );
}

#define SHOW_PERIOD 20

/**
 * @brief   Base control system
 */
void mainControlTask( void )
{
    uint32_t                print_cntr          = 0;
    system_state            state_prev          = 3; 

    systime_t time = chVTGetSystemTimeX();

    while( 1 )
    {
        print_cntr += 1;    // for debug print 
        system_state state_now  = lldGetSystemState( );

        sendOdometryToRos( );
        mproto_driver_send_steering( lldGetSteerAngleDeg() );
        mproto_driver_send_encoder_raw( lldGetEncoderRawRevs() );

        if( state_now == IDLE )
        {
            lldLightResetTurnState( );  // turn off leds

            if( state_prev != state_now )
            {
                dbgprintf( "IDLE\n\r", print_cntr );
                print_cntr = 0;
            }

            bool mode = rcModeIsEnabled();

            if( mode )
            {

                driverIsEnableCS( false );
                icuControlValue_t rc_steer_prt    = rcGetSteerControlValue( );
                icuControlValue_t rc_speed_prt    = rcGetSpeedControlValue( );

                lldControlSetDrMotorPower( rc_speed_prt );
                lldControlSetSteerMotorPower( rc_steer_prt );

            }
            else
            {
                driverIsEnableCS( true );
                driveSteerCSSetPosition( _steer_control );
                driveSpeedCSSetSpeed( _speed_control );
                // if( print_cntr == SHOW_PERIOD-1 )
                // {
                //     dbgprintf( "ROS_SP:(%d)\tROS_ST:(%d)\n\r",
                //              (int)(_speed_control*100), (int)_steer_control );
                //     print_cntr = 0; 
                // }
            }
        }
        else if( state_now == RUN )
        {
            driverIsEnableCS( true );
            
            if( state_prev != state_now )
            {
                dbgprintf( "RUN\n\r" );
            }

            driveSteerCSSetPosition( _steer_control );
            driveSpeedCSSetSpeed( _speed_control );
            lldLightDetectTurnState( _steer_control, _speed_control, state_now );
        }

        state_prev = state_now; 
        time = chThdSleepUntilWindowed( time, time + MS2ST( 25 ) );
    }
}
