#!/bin/zsh

rosrun wr8_software wr8_stop.sh

# roslaunch wr8_software base_start.launch lidar:=true &

# sleep 5

roslaunch wr8_software base_start.launch   camera_s:=false \
											camera_r:=false \
											solver:=false \
											gui_server:=false \
											uc:=true
