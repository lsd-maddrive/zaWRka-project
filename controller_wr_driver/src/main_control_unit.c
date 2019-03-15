#include <common.h>
#include <lld_start_button.h>
#include <lld_odometry.h>
#include <drive_cs.h>
#include <remote_control.h>

#include <ros_protos.h>


#define VT_ROS_CHECK_PERIOD_MS     500

virtual_timer_t         ros_check_vt;

#define VT_IDLE_MS                  10
#define VT_WAIT_MS                  10
#define VT_RUN_MS                   10

systime_t               time_checker_vt = 0;

//system_state            state_now      = IDLE;

float                   ros_steer_control   = 0;
float                   ros_speed_control   = 0;

odometryValue_t         x_pos          = 0;
odometryValue_t         y_pos          = 0;
odometryValue_t         tetta_deg      = 0;

odometrySpeedValue_t    speed_mps      = 0;
odometryRawSpeedValue_t enc_speed_rps  = 0;
odometrySpeedValue_t    speed_radps    = 0;

float                   steer_angl_deg = 0;
float                   speed_lpf_mps  = 0;

static void is_ros_dead_cb( void *arg )
{
      ros_speed_control = 0;
      ros_steer_control = 0;

      dbgprintf( "ROS is dead!\n\r", time_checker_vt);
}

void ros_is_alive( void )
{
//    palToggleLine( LINE_LED1 ); // just to check
    chVTSet( &ros_check_vt, MS2ST( VT_ROS_CHECK_PERIOD_MS ), is_ros_dead_cb, NULL );
}

void ros_control_handler( float speed, float steer )
{
    ros_speed_control = speed;
    ros_steer_control = steer;
    ros_is_alive( );
}

static THD_WORKING_AREA(waStateSender, 128); // 128 - stack size
static THD_FUNCTION(StateSender, arg)
{
    while(1)
    {
        system_state state_now  = lldGetSystemState( );
        ros_driver_send_state( state_now );

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

    ros_driver_cb_ctx_t cb_ctx      = ros_driver_get_new_cb_ctx();
    cb_ctx.cmd_cb                   = ros_control_handler;
    cb_ctx.reset_odometry_cb        = lldResetOdometry;

    ros_driver_init( NORMALPRIO, &cb_ctx );

    chThdCreateStatic(waStateSender, sizeof(waStateSender), NORMALPRIO - 1, StateSender, NULL);

    chVTObjectInit(&ros_check_vt);

    dbgprintf( "INIT done!\n\r");
}

void setRosControl( void )
{
    driveSteerCSSetPosition( ros_steer_control );
    driveSpeedCSSetSpeed( ros_speed_control );
}


void sendOdometryToRos( void )
{
    enc_speed_rps  = lldGetOdometryRawSpeedRPS( );
    speed_mps      = lldGetOdometryObjSpeedMPS( );
    speed_radps    = lldGetOdometryObjTettaSpeedRadPS( );

    steer_angl_deg = lldGetSteerAngleDeg( );
    speed_lpf_mps  = lldOdometryGetLPFObjSpeedMPS( );

    x_pos          = lldGetOdometryObjX( OBJ_DIST_M );
    y_pos          = lldGetOdometryObjY( OBJ_DIST_M );
    tetta_deg      = lldGetOdometryObjTettaDeg( );

    ros_driver_send_encoder_speed( enc_speed_rps );
    ros_driver_send_pose( x_pos, y_pos, tetta_deg, speed_mps, speed_radps );
}


/**
 * @brief   Base control system
 */
void mainControlTask( void )
{
    uint32_t                print_cntr          = 0;

    systime_t time = chVTGetSystemTimeX();
    while( 1 )
    {
        print_cntr += 1;
        system_state state_now  = lldGetSystemState( );

        if( state_now == IDLE )
        {
            if( print_cntr == 5 )
            {
              dbgprintf( "IDLE\n\r" );
              print_cntr = 0;
            }
            bool mode = rcModeIsEnabled();

            if( mode )
            {
                icuControlValue_t rc_steer_prt    = rcGetSteerControlValue( );
                icuControlValue_t rc_speed_prt    = rcGetSpeedControlValue( );

                lldControlSetDrMotorPower( rc_speed_prt );
                lldControlSetSteerMotorPower( rc_steer_prt );
            }
            else
            {
                lldControlSetSteerMotorRawPower( STEER_NULL );
                lldControlSetDrMotorRawPower( SPEED_ZERO );
            }
        }
        else if( state_now == RUN )
        {
            if( print_cntr == 5 )
            {
              dbgprintf( "RUN\n\r" );
              print_cntr = 0;
            }
            setRosControl( );
            sendOdometryToRos( );
        }

        time = chThdSleepUntilWindowed( time, time + MS2ST( 25 ) );
    }
}
