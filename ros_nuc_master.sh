#!/bin/bash

INTERFACE=wlx60e32714423f

export ROS_MASTER_URI="http://$NUC_ADDR:11311/"
export ROS_IP=`ifconfig "$INTERFACE" | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p'`
#export ROS_IP=192.168.43.28
#export ROS_IP=9.9.9.2

echo "ROS_IP: $ROS_IP / ROS_MASTER_URI: $ROS_MASTER_URI"
