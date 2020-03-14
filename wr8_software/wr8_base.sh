#!/bin/zsh

rosrun wr8_software wr8_stop.sh

roslaunch wr8_software base_start.launch   camera_s:=true \
											solver:=false \
											lidar:=true \
											gui_server:=false \
											uc:=true \
											stereo_r:=true \
											detector:=true
