#!/bin/zsh

# stop_base.sh

roslaunch wr8_software base_launch.launch   camera_s:=true \
											camera_r:=true \
											solver:=false \
											lidar:=false \
											gui_server:=false \
											uc:=true
