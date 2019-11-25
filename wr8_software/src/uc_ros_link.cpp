#include <iostream>
#include <memory>
#include <mutex>
#include <chrono>
#include <queue>
#include <thread>
using namespace std;

#include <boost/asio.hpp>
#include <boost/thread.hpp>
namespace asio = boost::asio;

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_broadcaster.h>

#include "uc_pc_link_defs.h"
#include "mproto.h"

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
        port_->set_option(asio::serial_port_base::character_size(8));
        port_->set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
        port_->set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
        port_->set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));

        boost::thread t(boost::bind(&asio::io_service::run, &io_));

        async_read_some_();

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
        if (port_.get() == NULL || !port_->is_open()) return;

        port_->async_read_some( 
            asio::buffer(readBuffer_, sizeof(readBuffer_)),
            boost::bind(
                &SerialMadProto::on_receive_, this, 
                asio::placeholders::error, 
                asio::placeholders::bytes_transferred));
    }

    void on_receive_(const boost::system::error_code& ec, size_t bytes_transferred)
    {
        unique_lock<mutex> lock(portMutex_);

        if ( port_.get() == NULL || !port_->is_open() ) 
            return;
        
        if ( ec ) {
            ROS_ERROR_STREAM("Failed to read / info: " << ec.message().c_str());
            async_read_some_();
            return;
        }

        for (size_t i = 0; i < bytes_transferred; ++i) {
            fromSerialBytes_.push(readBuffer_[i]);
        }

        async_read_some_();
    }


private:
    asio::io_service                io_;
    shared_ptr<asio::serial_port>   port_;
    std::mutex                      portMutex_;
    uint8_t                         readBuffer_[256];

    queue<uint8_t>                  fromSerialBytes_;
};

shared_ptr<tf::TransformBroadcaster>    br;
ros::Publisher                          odom_pub;
ros::Publisher                          raw_steer_pub;

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
    if ( cmd == WR_OUT_CMD_ODOM_DATA ) {
        if ( len != sizeof(float[5]) )
            return;

        publishOdometryData((float *)data);

    } else if ( cmd == WR_OUT_CMD_STEER_RAW ) {
        if ( len != sizeof(float) )
            return;

        publishRawSteerData(*((float *)data));
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

mproto_ctx_t mproto_ctx = NULL;

void cmdVelCb(const geometry_msgs::Twist &msg)
{
    // msg
    float data[2];

    data[0] = msg.linear.x;
    data[1] = msg.angular.z * 180 / M_PI;

    mproto_send_data(mproto_ctx, WR_IN_CMD_VEL, (uint8_t *)data, sizeof(data));
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "uc_ros_link");
    ros::NodeHandle n_pr("~");
    ros::NodeHandle n;

    /* Publishers - subscribers */
    br = make_shared<tf::TransformBroadcaster>();

    raw_steer_pub = n.advertise<std_msgs::Float32>("steer_raw_adc", 50);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

    ros::Subscriber sub = n.subscribe("cmd_vel", 50, cmdVelCb);

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
        return EXIT_FAILURE;
    }

    mproto_ctx = mproto_init(&funcs_ctx);
    mproto_register_command(mproto_ctx, WR_OUT_CMD_ODOM_DATA, mproto_cmd_cb);

    while ( true )
    {
        /* This thread works as ROS receiver side */
        /* Subscribers will send data to serial comminucator */
        try {
            ros::spinOnce();
        } catch (const exception &e) {
            cerr << e.what() << endl;
            break;
        }

        mproto_spin(mproto_ctx, 20);
    }


    g_serial.close();

    return EXIT_SUCCESS;
}
