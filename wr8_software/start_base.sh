#!/bin/zsh

# stop_base.sh

roslaunch wr8_software base_launch.launch   camera_s:=true \
											camera_r:=false \
											solver:=true \
											lidar:=true \
											gui_server:=false \
											uc:=true
