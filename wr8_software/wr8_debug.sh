#!/bin/zsh

# stop_base.sh

roslaunch wr8_software debug_launch.launch  camera_s:=false \
											camera_r:=false \
											solver:=false \
											lidar:=false \
											gui_server:=false \
											uc:=true \
											stereo:=true \
											bag_record:=false
