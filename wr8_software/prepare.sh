#!/bin/bash

sudo apt install ros-kinetic-hector-mapping ros-kinetic-move-base ros-kinetic-gmapping ros-kinetic-global-planner
pip install pygame

cd scripts/graph_path; pyrcc5 -o resources.py my.qrc