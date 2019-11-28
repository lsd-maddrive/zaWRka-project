#!/bin/bash

#export PKG_CONFIG_PATH="$PKG_CONFIG_PATH:`catkin locate`/src/csm/sm/pkg-config"
#echo $PKG_CONFIG_PATH

# Set hector_geotiff ignoring
touch hector_slam/hector_geotiff/CATKIN_IGNORE
touch hector_slam/hector_geotiff_plugins/CATKIN_IGNORE

catkin build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE="-O3" --env-cache

# Clear to be ok for git
rm hector_slam/hector_geotiff/CATKIN_IGNORE
rm hector_slam/hector_geotiff_plugins/CATKIN_IGNORE
