#!/bin/bash

HEAD_ADDR=$NUC_ADDR
HEAD_USER=user-sau-nuc

echo "Sync with $HEAD_USER@$HEAD_ADDR"

DST="$HEAD_USER@$HEAD_ADDR:~/catkin_ws/src"

rsync -avzPc ../AutoNetChallenge/ 	$DST/AutoNetChallenge/ --exclude='sync_target.sh' \
															--exclude='controller_wr_driver' \
															--exclude='neural_networks' \
															--exclude='build-*' \
															--delete
