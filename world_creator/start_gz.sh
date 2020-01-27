#!/bin/bash

GAZEBO_MODEL_PATH=`pwd`/models:$GAZEBO_MODEL_PATH gazebo worlds/maze2020v1.world
