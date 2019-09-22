#!/bin/bash

HEAD_ADDR=$NUC_ADDR
HEAD_USER=user-sau-nuc

echo "Sync bags with $HEAD_USER@$HEAD_ADDR"

SRC="$HEAD_USER@$HEAD_ADDR:~/bags/"

rsync -avzPc $SRC $HOME/bags/
