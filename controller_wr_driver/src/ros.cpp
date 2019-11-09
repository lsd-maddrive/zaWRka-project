#include <ros.h>
#include <ros_protos.h>
#include <common.h>

/*===========================================================================*/
/* ROS things                                                                */
/*===========================================================================*/

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Twist.h>

#include <std_srvs/Trigger.h>
#include <wr8_msgs/SteerParams.h>
#include <wr8_msgs/ControlParams.h>

static const ros_driver_cb_ctx_t default_cb_ctx = {
    .cmd_cb                 = NULL,
    .raw_cmd_cb             = NULL,

    .set_odom_params_cb     = NULL,
    .reset_odometry_cb      = NULL

    // .get_control_params     = NULL,
    // .set_control_params_cb  = NULL
};

static ros_driver_cb_ctx_t last_cb_ctx = default_cb_ctx;

ros_driver_cb_ctx_t ros_driver_get_new_cb_ctx( void )
{
    return default_cb_ctx;
}

void trigger_task_cb( const std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp )
{   
    (void)req;
    (void)resp;

    dbgprintf( "Called [%s]\n\r", __FUNCTION__ );

    resp.success = true;
}

void steer_params_cb( const wr8_msgs::SteerParamsRequest &req, wr8_msgs::SteerParamsResponse &resp )
{
    (void)req;
    (void)resp;

    dbgprintf( "Called [%s]\n\r", __FUNCTION__ );

    if ( last_cb_ctx.set_odom_params_cb )
    {
        last_cb_ctx.set_odom_params_cb( req.left_k, req.right_k );
    }
}

void control_params_cb( const wr8_msgs::ControlParamsRequest &req, wr8_msgs::ControlParamsResponse &resp )
{
    (void)req;
    (void)resp;

    dbgprintf( "Called %d [%s]\n\r", req.request_only, __FUNCTION__ );

    // if ( req.request_only && last_cb_ctx.get_control_params )
    // {
    //     control_params_setup_t params = last_cb_ctx.get_control_params();

    //     static const size_t data_length = 2;
    //     static float data[data_length];

    //     data[0] = params.esc_min_dc_offset;
    //     data[1] = params.esc_max_dc_offset;

    //     resp.params = data;
    //     resp.params_length = data_length;
    // }

    // if ( last_cb_ctx.set_control_params_cb )
    // {
    //     control_params_setup_t params;
    //     params.esc_min_dc_offset = req.esc_min_dc_offset;
    //     params.esc_max_dc_offset = req.esc_max_dc_offset;

    //     last_cb_ctx.set_control_params_cb( &params );
    // }
}

void reset_odometry_cb( const std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp )
{
    (void)req;
    (void)resp;

    dbgprintf( "Called [%s]\n\r", __FUNCTION__ );

    if ( last_cb_ctx.reset_odometry_cb )
    {
        last_cb_ctx.reset_odometry_cb();
    }
}

ros::ServiceServer<std_srvs::TriggerRequest, std_srvs::TriggerResponse>             srvc_check("check", &trigger_task_cb);                             
ros::ServiceServer<std_srvs::TriggerRequest, std_srvs::TriggerResponse>             srvc_rst_odom("reset_odometry", &reset_odometry_cb);                             
ros::ServiceServer<wr8_msgs::SteerParamsRequest, wr8_msgs::SteerParamsResponse>     srvc_steer_params("set_odom_params", &steer_params_cb);
ros::ServiceServer<wr8_msgs::ControlParamsRequest, wr8_msgs::ControlParamsResponse> srvc_cntrl_params("set_control_params", &control_params_cb);

void cmd_vel_cb( const geometry_msgs::Twist &msg )
{
    if ( last_cb_ctx.cmd_cb == NULL )
    {
        return;
    }

    float cmd_speed = msg.linear.x;
    float cmd_steer = msg.angular.z * 180 / M_PI;

    cmd_speed = CLIP_VALUE(cmd_speed, -ROS_INPUT_CMD_SPEED_LIMIT_MPS, ROS_INPUT_CMD_SPEED_LIMIT_MPS);
    cmd_steer = CLIP_VALUE(cmd_steer, -ROS_INPUT_CMD_STEER_LIMIT_DEG, ROS_INPUT_CMD_STEER_LIMIT_DEG);

    last_cb_ctx.cmd_cb( cmd_speed, cmd_steer );
}

void raw_vel_cb( const geometry_msgs::Twist &msg )
{
    if ( last_cb_ctx.cmd_cb == NULL )
    {
        return;
    }

    float cmd_speed = msg.linear.x;
    float cmd_steer = msg.angular.z;

    cmd_speed = CLIP_VALUE(cmd_speed, -100, 100);
    cmd_steer = CLIP_VALUE(cmd_steer, -100, 100);

    last_cb_ctx.cmd_cb( cmd_speed, cmd_steer );
}


ros::Subscriber<geometry_msgs::Twist>           topic_cmd("cmd_vel", &cmd_vel_cb);
ros::Subscriber<geometry_msgs::Twist>           topic_cmd_raw("raw_cmd_vel", &raw_vel_cb); 


ros::NodeHandle             ros_node;

std_msgs::Int32             i32_enc_raw_msg;
std_msgs::Float32           f32_encspeed_raw_msg;
std_msgs::Float32           f32_steer_angle_msg;
std_msgs::Float32MultiArray odometry_pose;
std_msgs::Int8              i8_state_msg;
std_msgs::Int32             i32_adc_raw_msg; 

ros::Publisher              topic_encoder_raw("encoder_raw", &i32_enc_raw_msg);
ros::Publisher              topic_encspeed_raw("encspeed_raw", &f32_encspeed_raw_msg);
ros::Publisher              topic_pose("odom_pose", &odometry_pose);
ros::Publisher              topic_steer("steer_angle", &f32_steer_angle_msg);
ros::Publisher              topic_state("state", &i8_state_msg);
ros::Publisher              topic_adc_raw("steer_raw_adc", &ros_driver_send_raw_adc);

/*
 * ROS spin thread - used to receive messages
 */

static THD_WORKING_AREA(waSpinner, 256);
static THD_FUNCTION(Spinner, arg)
{
    (void)arg;
    chRegSetThreadName("Spinner");

    while (true)
    {
        ros_node.spinOnce();
        chThdSleepMilliseconds( 20 );
    }
}

//=======================================================

void ros_driver_send_state( int8_t state )
{
    i8_state_msg.data = state;

    topic_state.publish( &i8_state_msg );
}

void ros_driver_send_steering( float steer_angle )
{
    f32_steer_angle_msg.data = steer_angle;

    topic_steer.publish( &f32_steer_angle_msg );
}

void ros_driver_send_pose( float x, float y, float dir, float vx, float uz )
{
    static const size_t data_length = 5;
    static float data[data_length];

    data[0] = x;
    data[1] = y;
    data[2] = dir;
    data[3] = vx;
    data[4] = uz;

    odometry_pose.data          = data;
    odometry_pose.data_length   = data_length;

    topic_pose.publish( &odometry_pose );
}

void ros_driver_send_encoder_raw( int32_t value )
{
    i32_enc_raw_msg.data = value;

    topic_encoder_raw.publish( &i32_enc_raw_msg );
}

void ros_driver_send_encoder_speed( float value )
{
    f32_encspeed_raw_msg.data = value;

    topic_encspeed_raw.publish( &f32_encspeed_raw_msg );
}

void ros_driver_send_raw_adc( uint16_t raw_adc )
{
    i32_adc_raw_msg.data = raw_adc; 

    topic_adc_raw.publish( &i32_adc_raw_msg );
}


/*===========================================================================*/
/* SDU relative                                                              */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

extern const USBConfig usbcfg;
extern SerialUSBConfig serusbcfg;
extern SerialUSBDriver SDU1;

#ifdef __cplusplus
}
#endif

// SerialDriver    *ros_sd     = &SD5;
/* Must be named <ros_sd_ptr> for ChibiOSHardware */
BaseChannel     *ros_sd_ptr = (BaseChannel *)&SDU1;

void ros_driver_init( tprio_t prio, ros_driver_cb_ctx_t *ctx )
{
    sduObjectInit( &SDU1 );
    sduStart( &SDU1, &serusbcfg );

    if ( ctx )
        last_cb_ctx = *ctx;
    else
        last_cb_ctx = default_cb_ctx;

    /*
     * Activates the USB driver and then the USB bus pull-up on D+.
     * Note, a delay is inserted in order to not have to disconnect the cable
     * after a reset.
     */
    usbDisconnectBus( serusbcfg.usbp );
    chThdSleepMilliseconds( 1500 );
    usbStart( serusbcfg.usbp, &usbcfg );
    usbConnectBus( serusbcfg.usbp );

    /* Serial driver */
    // sdStart( ros_sd, &sdcfg );
    // palSetPadMode( GPIOC, 12, PAL_MODE_ALTERNATE(8) );      // TX
    // palSetPadMode( GPIOD, 2, PAL_MODE_ALTERNATE(8) );       // RX

    /* ROS setup */
    ros_node.initNode();
    ros_node.setSpinTimeout( 20 );

    /* ROS publishers */
    ros_node.advertise( topic_encoder_raw );
    ros_node.advertise( topic_encspeed_raw );
    ros_node.advertise( topic_steer );
    ros_node.advertise( topic_pose );
    ros_node.advertise( topic_state );
    ros_node.advertise( topic_adc_raw );

    /* ROS subscribers */
    ros_node.subscribe( topic_cmd );
    ros_node.subscribe( topic_raw_cmd );

    /* ROS service servers */
    ros_node.advertiseService( srvc_check );
    ros_node.advertiseService( srvc_steer_params );
    // ros_node.advertiseService( srvc_cntrl_params );
    ros_node.advertiseService( srvc_rst_odom );

    chThdCreateStatic( waSpinner, sizeof(waSpinner), prio, Spinner, NULL );
}
