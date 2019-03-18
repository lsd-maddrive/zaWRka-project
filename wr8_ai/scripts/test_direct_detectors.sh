#!/bin/bash

PKG_PATH=`rospack find wr8_ai`
INPUT_PATH=$1

python $PKG_PATH/src/wr8_ai/detector_ncs.py -c $PKG_PATH/best/TSR/config.json -i $INPUT_PATH \
		-m rec -g $PKG_PATH/best/TSR/CustomMobileNetv2.ncsg 
# python $PKG_PATH/src/wr8_ai/detector_ncs.py -c $PKG_PATH/best/TSR/config.json -i $INPUT_PATH \
		# -m reccpu -w $PKG_PATH/best/TSR/TSR_CustomMobileNetv2_32x32_ep043-val_loss0.002-loss0.182.h5 
