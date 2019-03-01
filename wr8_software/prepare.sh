#!/bin/bash

sudo apt install ros-kinetic-hector-mapping \
					ros-kinetic-move-base \
					ros-kinetic-gmapping \
					ros-kinetic-global-planner \
					ros-kinetic-costmap-converter \
					ros-kinetic-gazebo9-plugins \
					ros-kinetic-map-server \
					ros-kinetic-amcl
pip install pygame

cd scripts/graph_path; pyrcc5 -o resources.py my.qrc
