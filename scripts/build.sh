#!/usr/bin/env bash

catkin build \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE="-O3" --env-cache \
    wr8_description \
    wr8_software \
    wp_global_planner \
	frequency_converter \
	parking_detector \
	car_parking \
    ydlidar_ros_driver \
    realsense2_camera \
    realsense2_description \
