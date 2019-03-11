#!/bin/bash

HEAD_ADDR=$NUC_ADDR
HEAD_USER=user-sau-nuc

DST="$HEAD_USER@$HEAD_ADDR:~/catkin_ws/src"

rsync -avzPc ../AutoNetChallenge/ 	$DST/AutoNetChallenge/ --exclude='.git' \
															--exclude='controller_wr_driver'
