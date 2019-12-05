#include <iostream>
#include <memory>
#include <mutex>
#include <chrono>
#include <queue>
#include <thread>
#include <condition_variable>
using namespace std;

#include <boost/asio.hpp>
#include <boost/thread.hpp>
namespace asio = boost::asio;

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_broadcaster.h>

#include "uc_pc_link_defs.h"
#include "mproto.h"

mproto_ctx_t mproto_ctx = NULL;

class SerialMadProto
{
public:
    bool open(string name, int baud_rate)
    {
        boost::system::error_code ec;

        if ( port_ )
            close();

        port_ = make_shared<asio::serial_port>(io_); 

        port_->open(name, ec);
        if ( ec ) {
            ROS_ERROR_STREAM("Failed to open serial port: " << name << " / info: " << ec.message().c_str());
            return false;
        }

        port_->set_option(asio::serial_port_base::baud_rate(baud_rate));
        // port_->set_option(asio::serial_port_base::character_size(8));
        // port_->set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
        // port_->set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
        // port_->set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));

        async_read_some_();

        t_ = boost::thread(boost::bind(&asio::io_service::run, &io_));

        pollerThread_ = make_shared<thread>(&SerialMadProto::run, this);

        return true;
    }

    void close()
    {
        unique_lock<mutex> lock(portMutex_);

        if ( port_ )
        {
            port_->cancel();
            port_->close();
            port_.reset();
        }

        io_.stop();
        io_.reset();
    }

    size_t bytes_available()
    {
        unique_lock<mutex> lock(portMutex_);
        return fromSerialBytes_.size();
    }

    uint8_t get_byte()
    {
        unique_lock<mutex> lock(portMutex_);

        uint8_t data = fromSerialBytes_.front();
        fromSerialBytes_.pop();
        return data;
    }

    void send_bytes(uint8_t *data, size_t len)
    {
        port_->write_some(boost::asio::buffer(data, len));
    }

    void async_read_some_()
    {
        // ROS_INFO_STREAM("Start reading");

        if (port_.get() == NULL || !port_->is_open()) {
            ROS_ERROR_STREAM("Failed to test port (1)");
            return;
        }

        port_->async_read_some( 
            asio::buffer(readBuffer_, sizeof(readBuffer_)),
            boost::bind(
                &SerialMadProto::on_receive_, this, 
                asio::placeholders::error, 
                asio::placeholders::bytes_transferred));
    }

    void wait_for_data()
    {
        unique_lock<mutex> lock(portMutex_);
        confvarNotifier_.wait(lock);
    }

    void on_receive_(const boost::system::error_code& ec, size_t bytes_transferred)
    {
        unique_lock<mutex> lock(portMutex_);

        if ( port_.get() == NULL || !port_->is_open() ) {
            ROS_ERROR_STREAM("Failed to test port (2)");
            return;
        }
        
        if ( ec ) {
            ROS_ERROR_STREAM("Failed to read / info: " << ec.message().c_str());
            // async_read_some_();
            isPollerActive_ = false;
            this_thread::sleep_for(1s);
            ros::shutdown();
            return;
        }

        // ROS_INFO_STREAM("Readed " << to_string(bytes_transferred));

        for (size_t i = 0; i < bytes_transferred; ++i) {
            fromSerialBytes_.push(readBuffer_[i]);
        }

// cout << "Read" << endl;
        confvarNotifier_.notify_all();
        async_read_some_();
    }

    void run()
    {
        isPollerActive_ = true;
        ROS_INFO_STREAM("Poller thread started");
        
        while ( isPollerActive_ )
        {
            while ( bytes_available() == 0 ) {
                wait_for_data();
            }
            mproto_spin(mproto_ctx, 0);
        }
    }

private:
    asio::io_service                io_;
    boost::thread                   t_; 
    
    shared_ptr<asio::serial_port>   port_;
    std::mutex                      portMutex_;
    std::condition_variable         confvarNotifier_;
    uint8_t                         readBuffer_[256];

    shared_ptr<thread>              pollerThread_;
    bool                            isPollerActive_;

    queue<uint8_t>                  fromSerialBytes_;
};

shared_ptr<tf::TransformBroadcaster>    br;
ros::Publisher                          odom_pub;
ros::Publisher                          raw_steer_pub;
ros::Publisher                          steer_pub;
ros::Publisher                          state_pub;
ros::Publisher                          encoder_rotation_pub;
ros::Publisher                          encoder_speed_pub;

void publishEncoderSpeed(float value)
{
    std_msgs::Float32 msg;
    msg.data = value;
    encoder_speed_pub.publish(msg);
}

void publishEncoderValue(float value)
{
    std_msgs::Float32 msg;
    msg.data = value;
    encoder_rotation_pub.publish(msg);
}

void publishStateData(int8_t value)
{
    std_msgs::Int8 msg;
    msg.data = value;
    state_pub.publish(msg);
}

void publishSteerData(float value)
{
    std_msgs::Float32 msg;
    msg.data = value;
    steer_pub.publish(msg);
}

void publishRawSteerData(float value)
{
    std_msgs::Float32 msg;
    msg.data = value;
    raw_steer_pub.publish(msg);
}

void publishOdometryData(float data[5])
{
    nav_msgs::Odometry msg;

    float x = data[0];
    float y = data[1];
    float teta_deg = data[2];
    float vx = data[3];
    float uz = data[4];

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(teta_deg / 180 * M_PI);
    ros::Time current_time = ros::Time::now();

    /* Odometry topic */
    msg.header.stamp = current_time;
    msg.header.frame_id = "odom";

    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation = odom_quat;

    msg.child_frame_id = "base_link";
    msg.twist.twist.linear.x = vx;
    msg.twist.twist.angular.z = uz;

    odom_pub.publish(msg);

    /* TF */
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    br->sendTransform(odom_trans);
}

void mproto_cmd_cb(mpcmd_t cmd, uint8_t *data, size_t len)
{
// cout << "Command: " << to_string(cmd) << endl;

    if ( cmd == WR_OUT_CMD_ODOM_DATA ) {
        if ( len != sizeof(float[5]) )
            return;

        publishOdometryData((float *)data);

    } else if ( cmd == WR_OUT_CMD_STEER_RAW ) {
        if ( len != sizeof(float) )
            return;

        publishRawSteerData(*((float *)data));
    } else if ( cmd == WR_OUT_CMD_STEER_ANGLE ) {
        if ( len != sizeof(float) )
            return;

        publishSteerData(*((float *)data));
    } else if ( cmd == WR_OUT_CMD_STATE ) {
        if ( len != sizeof(int8_t) )
            return;

        publishStateData(*((int8_t *)data));
    } else if ( cmd == WR_OUT_CMD_ENCODER_VALUE ) {
        if ( len != sizeof(float) )
            return;

        publishEncoderValue(*((float *)data));
    } else if ( cmd == WR_OUT_CMD_ENCODER_SPEED ) {
        if ( len != sizeof(float) )
            return;

        publishEncoderSpeed(*((float *)data));
    }
}

/* Global to be used in class */
SerialMadProto g_serial;

static int16_t read_byte()
{
    if ( g_serial.bytes_available() > 0 )
        return g_serial.get_byte();
    
    return -1;
}

static void write_bytes(uint8_t *data, size_t len)
{
    g_serial.send_bytes(data, len);
}

static mptime_t get_time()
{
    chrono::milliseconds ms = chrono::duration_cast<chrono::milliseconds >(
        chrono::system_clock::now().time_since_epoch()
    );

    return ms.count() % 100000000;
}


mproto_func_ctx_t funcs_ctx = {
    .get_byte = read_byte,
    .put_bytes = write_bytes,
    .get_time = get_time
};

void rawCmdVelCb(const geometry_msgs::Twist &msg)
{
    // msg
    float data[2];

    data[0] = msg.linear.x;
    data[1] = msg.angular.z;

    mproto_send_data(mproto_ctx, WR_IN_CMD_RAW_VEL, (uint8_t *)data, sizeof(data));
}

void cmdVelCb(const geometry_msgs::Twist &msg)
{
    // msg
    float data[2];

    data[0] = msg.linear.x;
    data[1] = msg.angular.z * 180 / M_PI;

    mproto_send_data(mproto_ctx, WR_IN_CMD_VEL, (uint8_t *)data, sizeof(data));
}

void resetOdomCb(const std_msgs::Empty &msg)
{
    mproto_send_data(mproto_ctx, WR_IN_CMD_RESET_ODOM, NULL, 0);
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "uc_ros_link");
    ros::NodeHandle n_pr("~");
    ros::NodeHandle n;

    /* Publishers - subscribers */
    br = make_shared<tf::TransformBroadcaster>();

    raw_steer_pub = n.advertise<std_msgs::Float32>("steer_raw_adc", 50);
    steer_pub = n.advertise<std_msgs::Float32>("steer_angle", 50);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    state_pub = n.advertise<std_msgs::Int8>("state", 50);
    encoder_rotation_pub = n.advertise<std_msgs::Float32>("enc_rot", 50);
    encoder_speed_pub = n.advertise<std_msgs::Float32>("enc_speed", 50);

    ros::Subscriber sub_cmd_vel = n.subscribe("cmd_vel", 50, cmdVelCb);
    ros::Subscriber sub_raw_cmd_vel = n.subscribe("raw_cmd_vel", 50, rawCmdVelCb);
    ros::Subscriber sub_rst_odom = n.subscribe("reset_odom", 50, resetOdomCb);

    /* Main program */
    std::string portName;
    if ( !n_pr.getParam("port", portName) )
    {
        ROS_ERROR_STREAM("Parameter 'port' not set! Exit.");
        return EXIT_FAILURE;
    }

    ROS_INFO_STREAM("Opening port: " << portName);

    int baudRate = 115200;
    if ( !n_pr.getParam("baud", baudRate) )
    {
        ROS_WARN_STREAM("Parameter 'baud' not set -> use default " << to_string(baudRate));
    }

    if ( !g_serial.open(portName, baudRate) )
    {
        ROS_ERROR_STREAM("Failed to open port, exit");
        this_thread::sleep_for(1s);
        return EXIT_FAILURE;
    }

    mproto_ctx = mproto_init(&funcs_ctx);
    mproto_register_command(mproto_ctx, WR_OUT_CMD_ODOM_DATA, mproto_cmd_cb);
    mproto_register_command(mproto_ctx, WR_OUT_CMD_STEER_RAW, mproto_cmd_cb);
    mproto_register_command(mproto_ctx, WR_OUT_CMD_STEER_ANGLE, mproto_cmd_cb);
    mproto_register_command(mproto_ctx, WR_OUT_CMD_STATE, mproto_cmd_cb);
    mproto_register_command(mproto_ctx, WR_OUT_CMD_ENCODER_VALUE, mproto_cmd_cb);
    mproto_register_command(mproto_ctx, WR_OUT_CMD_ENCODER_SPEED, mproto_cmd_cb);

    try {
        ros::spin();
    } catch (const exception &e) {
        cerr << e.what() << endl;
    }

    g_serial.close();

    return EXIT_SUCCESS;
}
