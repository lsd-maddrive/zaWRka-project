#!/bin/bash

export ROS_HOSTNAME=$(hostname).local
<<<<<<< HEAD
export ROS_HOSTNAME=192.168.1.44
export ROS_MASTER_URI=http://zawrka.local:11311
export ROS_MASTER_URI=http://192.168.1.45:11311
=======
export ROS_HOSTNAME=192.168.1.39
# export ROS_MASTER_URI=http://zawrka.local:11311
export ROS_MASTER_URI=http://192.168.1.40:11311
>>>>>>> dbae8ec50732d5052183678f8313086efb08a440

echo "Master: $ROS_MASTER_URI / Hostname: $ROS_HOSTNAME"
