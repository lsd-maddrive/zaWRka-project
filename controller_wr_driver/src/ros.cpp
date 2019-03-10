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
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Twist.h>

#include <std_srvs/Trigger.h>

void trigger_task_cb( const std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp )
{   
    dbgprintf( "Called\n\r" );

    resp.success = true;
}


void (*g_cb_func)(float speed, float steer) = NULL;

void ros_driver_set_control_cb( void (*cb_func)(float speed, float steer) )
{
    g_cb_func = cb_func;
}

void cmd_vel_cb( const geometry_msgs::Twist &msg )
{
    float cmd_speed = msg.linear.x;
    float cmd_steer = msg.angular.z * 180 / M_PI;

    if ( g_cb_func )
    {
        g_cb_func( cmd_speed, cmd_steer );
    }
}

ros::NodeHandle             ros_node;
std_msgs::Int32             i32_enc_raw_msg;
std_msgs::Float32           f32_encspeed_raw_msg;
std_msgs::Float32           f32_steer_angle_msg;
geometry_msgs::Point32      odometry_pose;

ros::ServiceServer<std_srvs::TriggerRequest, std_srvs::TriggerResponse>    srvc_check("check", &trigger_task_cb);                             

ros::Publisher                                  topic_encoder_raw("encoder_raw", &i32_enc_raw_msg);
ros::Publisher                                  topic_encspeed_raw("encspeed_raw", &f32_encspeed_raw_msg);
ros::Publisher                                  topic_pose("odom_pose", &odometry_pose);
ros::Publisher                                  topic_steer("steer_angle", &f32_steer_angle_msg);

ros::Subscriber<geometry_msgs::Twist>           topic_cmd("cmd_vel", &cmd_vel_cb);


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
    vx = vx; uz = uz;

    odometry_pose.x = x;
    odometry_pose.y = y;
    odometry_pose.z = dir;

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

    chThdCreateStatic( waSpinner, sizeof(waSpinner), prio, Spinner, NULL );
}
