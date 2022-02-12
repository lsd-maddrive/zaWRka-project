#!/bin/bash

if [ -z "$ROS_DISTRO" ]
then
	echo "ROS_DISTRO is not set!"
	exit 1
fi
sudo apt install \
	ros-$ROS_DISTRO-realsense2-camera\
    ros-$ROS_DISTRO-rtabmap \
    ros-$ROS_DISTRO-rtabmap-ros \
	ros-$ROS_DISTRO-imu-filter-madgwick \
					

