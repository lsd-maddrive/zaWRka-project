#!/bin/bash

export ROS_HOSTNAME=$(hostname).local
export ROS_MASTER_URI=http://usersaujtx-desktop.local:11311

echo "Master: $ROS_MASTER_URI / Hostname: $ROS_HOSTNAME"
