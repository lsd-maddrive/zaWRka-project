#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

static const char* NODE_NAME = "frequency_converter";

static const std::string IMAGE_RAW = "/image_raw";
static const std::string CAMERA_INFO = "/camera_info";
static const std::string LEFT = "/left";
static const std::string RIGHT = "/right";

static const size_t IN_QUEUE_SIZE = 1;
static const size_t OUT_QUEUE_SIZE = 1;
static const float MIN_OUT_FREQUENCY = 0.1;
static const float DEFAULT_OUT_FREQUENCY = 1;
static const float MAX_OUT_FREQUENCY = 50;

static sensor_msgs::CameraInfo::ConstPtr left_camera_info, right_camera_info;
static sensor_msgs::Image::ConstPtr left_image, right_image;


void left_info_cb(const sensor_msgs::CameraInfo::ConstPtr& camera_info){
    left_camera_info = camera_info;
}

void left_raw_cb(const sensor_msgs::Image::ConstPtr& image){
    left_image = image;
}

void right_info_cb(const sensor_msgs::CameraInfo::ConstPtr& camera_info){
    right_camera_info = camera_info;
}

void right_raw_cb(const sensor_msgs::Image::ConstPtr& image){
    right_image = image;
}

int main(int argc, char** argv){
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    float frequency;
    std::string in_stereo_ns, out_stereo_ns;
    if(!private_nh.getParam("frequency", frequency) || (frequency < MIN_OUT_FREQUENCY) || 
       (frequency > MAX_OUT_FREQUENCY)){
        frequency = DEFAULT_OUT_FREQUENCY;
    }
    private_nh.param("in_stereo_ns", in_stereo_ns, std::string("in_stereo_ns"));
    private_nh.param("out_stereo_ns", out_stereo_ns, std::string("out_stereo_ns"));

    const std::string SUB_LEFT_INFO = in_stereo_ns + LEFT + CAMERA_INFO;
    const std::string SUB_LEFT_RAW = in_stereo_ns + LEFT + IMAGE_RAW;
    const std::string SUB_RIGHT_INFO = in_stereo_ns + RIGHT + CAMERA_INFO;
    const std::string SUB_RIGHT_RAW = in_stereo_ns + RIGHT + IMAGE_RAW;

    const std::string PUB_LEFT_INFO = out_stereo_ns + LEFT + CAMERA_INFO;
    const std::string PUB_LEFT_RAW = out_stereo_ns + LEFT + IMAGE_RAW;
    const std::string PUB_RIGHT_INFO = out_stereo_ns + RIGHT + CAMERA_INFO;
    const std::string PUB_RIGHT_RAW = out_stereo_ns + RIGHT + IMAGE_RAW;

    auto sub_left_info = nh.subscribe(SUB_LEFT_INFO.c_str(), IN_QUEUE_SIZE, left_info_cb);
    auto sub_left_raw = nh.subscribe(SUB_LEFT_RAW.c_str(), IN_QUEUE_SIZE, left_raw_cb);
    auto sub_right_info = nh.subscribe(SUB_RIGHT_INFO.c_str(), IN_QUEUE_SIZE, right_info_cb);
    auto sub_right_raw = nh.subscribe(SUB_RIGHT_RAW.c_str(), IN_QUEUE_SIZE, right_raw_cb);

    auto pub_left_info = nh.advertise<sensor_msgs::CameraInfo>(PUB_LEFT_INFO.c_str(), OUT_QUEUE_SIZE);
    auto pub_left_raw = nh.advertise<sensor_msgs::Image>(PUB_LEFT_RAW.c_str(), OUT_QUEUE_SIZE);
    auto pub_right_info = nh.advertise<sensor_msgs::CameraInfo>(PUB_RIGHT_INFO.c_str(), OUT_QUEUE_SIZE);
    auto pub_right_raw = nh.advertise<sensor_msgs::Image>(PUB_RIGHT_RAW.c_str(), OUT_QUEUE_SIZE);

    ros::Rate loop_rate(frequency);
    while(ros::ok()){
        if(left_camera_info != nullptr){
            pub_left_info.publish(*left_camera_info);
        }
        if(left_image != nullptr){
            pub_left_raw.publish(*left_image);
        }
        if(right_camera_info != nullptr){
            pub_right_info.publish(*right_camera_info);
        }
        if(right_image != nullptr){
            pub_right_raw.publish(*right_image);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
