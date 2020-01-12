#!/bin/bash

PKG_PATH=`rospack find wr8_ai`
INPUT_PATH=$1


python $PKG_PATH/src/wr8_ai/detectors.py -i $INPUT_PATH -cr $PKG_PATH/best/TSR/config -cd $PKG_PATH/best/TSD/config \
		-m full -g $PKG_PATH/best/TSD/graph -w $PKG_PATH/best/TSR/weight
		


		# -m yolo -g $PKG_PATH/best/TSD/MobileNetv2_35.ncsg -c $PKG_PATH/best/TSD/rf_signs_mobilev2.json
		# -m reccpu -w $PKG_PATH/best/TSR/TSR_CustomMobileNetv2_32x32_ep043-val_loss0.002-loss0.182.h5 -c $PKG_PATH/best/TSR/config.json 
		# -m rec -g $PKG_PATH/best/TSR/CustomMobileNetv2.ncsg -c $PKG_PATH/best/TSR/config.json	
