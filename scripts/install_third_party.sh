#!/usr/bin/env bash

THIRD_PARTY_DIR=third_party
mkdir -p $THIRD_PARTY_DIR

# Mad Detector (package for signs detection)

git -C $THIRD_PARTY_DIR clone  https://github.com/lsd-maddrive/mad_detector.git

# ELP stereocamera driver
#   NB - version not set as driver is under our development

git -C $THIRD_PARTY_DIR clone https://github.com/lsd-maddrive/elp_stereo_camera

# Madproto - protocol for serial communication

git -C $THIRD_PARTY_DIR clone https://github.com/KaiL4eK/madproto.git

# Teleop-tools

git -C $THIRD_PARTY_DIR clone https://github.com/KaiL4eK/teleop_tools.git

# Ydlidar driver

git -C $THIRD_PARTY_DIR clone https://github.com/EAIBOT/ydlidar.git -b 1.3.9

# World Creator package

git -C $THIRD_PARTY_DIR clone https://github.com/PonomarevDA/world_creator.git && git -C $THIRD_PARTY_DIR/world_creator checkout a713b13
