#!/bin/bash

export ROS_HOSTNAME=$(hostname).local
export ROS_MASTER_URI=http://10.140.8.110:11311

echo "Master: $ROS_MASTER_URI / Hostname: $ROS_HOSTNAME"
