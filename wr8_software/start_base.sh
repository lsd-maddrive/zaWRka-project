#!/bin/bash

rosrun wr8_software stop_base.sh

sleep 3

roslaunch wr8_software base_launch.launch   camera_s:=true \
											camera_r:=true \
											solver:=false \
											lidar:=true \
											gui_server:=false \
											uc:=true \
											& disown
