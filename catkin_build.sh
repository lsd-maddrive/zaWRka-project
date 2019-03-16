#!/bin/bash

export PKG_CONFIG_PATH="$PKG_CONFIG_PATH:`catkin locate`/src/csm/sm/pkg-config"
echo $PKG_CONFIG_PATH

catkin build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE="-O3" --env-cache
