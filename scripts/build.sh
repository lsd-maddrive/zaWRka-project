#!/usr/bin/env bash

catkin build \
    wr8_description \
    wr8_software \
    wp_global_planner \
	frequency_converter \
	parking_detector \
	car_parking \
    ydlidar \
    elp_stereo_camera \
    teleop_tools
