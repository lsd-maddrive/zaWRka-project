#!/bin/bash

export ROS_HOSTNAME=$(hostname).local
export ROS_HOSTNAME=192.168.1.36
export ROS_MASTER_URI=http://zawrka.local:11311
export ROS_MASTER_URI=http://192.168.1.37:11311

echo "Master: $ROS_MASTER_URI / Hostname: $ROS_HOSTNAME"
