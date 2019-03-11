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

ros_driver_cb_ctx_t default_cb_ctx = {
    .cmd_cb                 = NULL,
    .set_steer_params_cb    = NULL
};

ros_driver_cb_ctx_t last_cb_ctx = default_cb_ctx;

void ros_driver_set_cb_ctx( ros_driver_cb_ctx_t *ctx )
{
    if ( ctx )
        last_cb_ctx = *ctx;
    else
        last_cb_ctx = default_cb_ctx;
}

void trigger_task_cb( const std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp )
{   
    (void)req;

    dbgprintf( "Called %s\n\r", __FUNCTION__ );

    resp.success = true;
}

void steer_params_cb( const wr8_msgs::SteerParamsRequest &req, wr8_msgs::SteerParamsResponse &resp )
{
    (void)req;
    resp = resp;

    dbgprintf( "Called %s\n\r", __FUNCTION__ );

    if ( last_cb_ctx.set_steer_params_cb )
    {
        last_cb_ctx.set_steer_params_cb( req.left_k, req.right_k );
    }
}

ros::ServiceServer<std_srvs::TriggerRequest, std_srvs::TriggerResponse>             srvc_check("check", &trigger_task_cb);                             
ros::ServiceServer<wr8_msgs::SteerParamsRequest, wr8_msgs::SteerParamsResponse>     srvc_steer_params("set_steer_params", &steer_params_cb);

void cmd_vel_cb( const geometry_msgs::Twist &msg )
{
    float cmd_speed = msg.linear.x;
    float cmd_steer = msg.angular.z * 180 / M_PI;

    if ( last_cb_ctx.cmd_cb )
    {
        last_cb_ctx.cmd_cb( cmd_speed, cmd_steer );
    }
}

ros::Subscriber<geometry_msgs::Twist>           topic_cmd("cmd_vel", &cmd_vel_cb);


ros::NodeHandle             ros_node;

std_msgs::Int32             i32_enc_raw_msg;
std_msgs::Float32           f32_encspeed_raw_msg;
std_msgs::Float32           f32_steer_angle_msg;
std_msgs::Float32MultiArray odometry_pose;

ros::Publisher              topic_encoder_raw("encoder_raw", &i32_enc_raw_msg);
ros::Publisher              topic_encspeed_raw("encspeed_raw", &f32_encspeed_raw_msg);
ros::Publisher              topic_pose("odom_pose", &odometry_pose);
ros::Publisher              topic_steer("steer_angle", &f32_steer_angle_msg);

/*
 * ROS spin thread - used to receive messages
 */

static THD_WORKING_AREA(waSpinner, 128);
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

void ros_driver_send_steering( float steer_angle )
{
    f32_steer_angle_msg.data = steer_angle;

    topic_steer.publish(&f32_steer_angle_msg);
}

void ros_driver_send_pose( float x, float y, float dir, float vx, float uz )
{
    static const int data_length = 5;
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

    topic_encoder_raw.publish(&i32_enc_raw_msg);
}

void ros_driver_send_encoder_speed( float value )
{
    f32_encspeed_raw_msg.data = value;

    topic_encspeed_raw.publish(&f32_encspeed_raw_msg);
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

void ros_driver_init( tprio_t prio )
{
    sduObjectInit( &SDU1 );
    sduStart( &SDU1, &serusbcfg );

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

    /* ROS subscribers */
    ros_node.subscribe( topic_cmd );

    /* ROS service servers */
    ros_node.advertiseService( srvc_check );
    ros_node.advertiseService( srvc_steer_params );

    chThdCreateStatic( waSpinner, sizeof(waSpinner), prio, Spinner, NULL );
}
