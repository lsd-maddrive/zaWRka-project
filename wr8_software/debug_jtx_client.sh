#!/usr/bin/env bash

export ROS_HOSTNAME=$(hostname).local
export ROS_MASTER_URI=http://alexey-home-pc.local:11311

echo "Master: $ROS_MASTER_URI / Hostname: $ROS_HOSTNAME"
