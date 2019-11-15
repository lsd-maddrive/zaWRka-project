#!/bin/bash

GAZEBO_MODEL_PATH=`pwd`/models:$GAZEBO_MODEL_PATH gazebo new_world.world
