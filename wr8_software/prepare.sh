#!/bin/bash

sudo apt install ros-$ROS_DISTRO-hector-mapping \
					ros-$ROS_DISTRO-move-base \
					ros-$ROS_DISTRO-gmapping \
					ros-$ROS_DISTRO-global-planner \
					ros-$ROS_DISTRO-costmap-converter \
					ros-$ROS_DISTRO-gazebo9-plugins \
					ros-$ROS_DISTRO-map-server \
					ros-$ROS_DISTRO-amcl \
					ros-$ROS_DISTRO-laser-scan-matcher \
					ros-$ROS_DISTRO-stereo-image-proc \
					ros-$ROS_DISTRO-image-view \
					ros-$ROS_DISTRO-teleop-twist-keyboard \
					ros-$ROS_DISTRO-rosserial-msgs \
					ros-$ROS_DISTRO-usb-cam

# pip install pygame
cd scripts/graph_path; pyrcc5 -o resources.py my.qrc
