#!/bin/bash

rosrun wr8_software stop_base.sh

sleep 3

roslaunch wr8_software base_launch.launch & disown
