#!/bin/bash

rosnode kill -a
# Just to give it a rest
sleep 1
killall -9 rosmaster
