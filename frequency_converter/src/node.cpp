#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

static const char* NODE_NAME = "stereo_camera/frequency_converter";

static const char* SUB_LEFT_INFO = "stereo_camera_temp/left/camera_info";
static const char* SUB_LEFT_RAW = "stereo_camera_temp/left/image_raw";
static const char* SUB_RIGHT_INFO = "stereo_camera_temp/right/camera_info";
static const char* SUB_RIGHT_RAW = "stereo_camera_temp/right/image_raw";

static const char* PUB_LEFT_INFO = "stereo_camera/left/camera_info";
static const char* PUB_LEFT_RAW = "stereo_camera/left/image_raw";
static const char* PUB_RIGHT_INFO = "stereo_camera/right/camera_info";
static const char* PUB_RIGHT_RAW = "stereo_camera/right/image_raw";
static const char* FRAME_NAME = "stereo_camera_optical_frame";

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
    ros::NodeHandle n;

    float frequency;
    if (!ros::param::get("/frequency_converter/frequency", frequency) ||
        (frequency < MIN_OUT_FREQUENCY) || (frequency > MAX_OUT_FREQUENCY)){
        frequency = DEFAULT_OUT_FREQUENCY;
    }
    ROS_INFO("Output frequency = %f", frequency);

    ros::Subscriber sub_left_info = n.subscribe(SUB_LEFT_INFO, IN_QUEUE_SIZE, left_info_cb);
    ros::Subscriber sub_left_raw = n.subscribe(SUB_LEFT_RAW, IN_QUEUE_SIZE, left_raw_cb);
    ros::Subscriber sub_right_info = n.subscribe(SUB_RIGHT_INFO, IN_QUEUE_SIZE, right_info_cb);
    ros::Subscriber sub_right_raw = n.subscribe(SUB_RIGHT_RAW, IN_QUEUE_SIZE, right_raw_cb);

    ros::Publisher pub_left_info = n.advertise<sensor_msgs::CameraInfo>(PUB_LEFT_INFO, OUT_QUEUE_SIZE);
    ros::Publisher pub_left_raw = n.advertise<sensor_msgs::Image>(PUB_LEFT_RAW, OUT_QUEUE_SIZE);
    ros::Publisher pub_right_info = n.advertise<sensor_msgs::CameraInfo>(PUB_RIGHT_INFO, OUT_QUEUE_SIZE);
    ros::Publisher pub_right_raw = n.advertise<sensor_msgs::Image>(PUB_RIGHT_RAW, OUT_QUEUE_SIZE);

    ros::Rate loop_rate(frequency);

    while ( ros::ok() ){
        if(left_camera_info != nullptr){
            sensor_msgs::CameraInfo info = *left_camera_info;
            info.header.frame_id = FRAME_NAME;
            pub_left_info.publish(info);
        }
        if(left_image != nullptr){
            sensor_msgs::Image image = *left_image;
            image.header.frame_id = FRAME_NAME;
            pub_left_raw.publish(image);
        }
        if(right_camera_info != nullptr){
            sensor_msgs::CameraInfo info = *right_camera_info;
            info.header.frame_id = FRAME_NAME;
            pub_right_info.publish(info);
        }
        if(right_image != nullptr){
            sensor_msgs::Image image = *right_image;
            image.header.frame_id = FRAME_NAME;
            pub_right_raw.publish(image);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
