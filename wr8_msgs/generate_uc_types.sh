#!/bin/bash

LIB_DST_DIR=`rospack find wr8_msgs`
rm -rf $LIB_DST_DIR/ros_lib
rosrun wr8_msgs make_uc_lib.py $LIB_DST_DIR
