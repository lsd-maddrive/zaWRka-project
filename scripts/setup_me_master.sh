#!/bin/bash

export ROS_HOSTNAME=$(hostname).local
export ROS_HOSTNAME=192.168.88.95
export ROS_MASTER_URI=http://$ROS_HOSTNAME:11311

echo "Master: $ROS_MASTER_URI / Hostname: $ROS_HOSTNAME"
