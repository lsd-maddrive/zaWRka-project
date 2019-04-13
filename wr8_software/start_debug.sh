#!/bin/zsh

# stop_base.sh

roslaunch wr8_software base_launch.launch   camera_s:=false \
											camera_r:=false \
											solver:=false \
											lidar:=false \
											gui_server:=true \
											uc:=true \
											bag_record:=false
