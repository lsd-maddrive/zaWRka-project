#!/bin/bash

export ROS_HOSTNAME=$(hostname).local
export ROS_MASTER_URI=http://usersaunuc-NUC7i5BNK.local:11311

echo "Master: $ROS_MASTER_URI / Hostname: $ROS_HOSTNAME"

