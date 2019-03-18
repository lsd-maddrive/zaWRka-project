#!/usr/bin/env python

import rospy
import json
from wr8_ai.msg import BoundingBox
import wr8_ai.utils as ut
import wr8_ai.ncs as ncs
from wr8_ai.yolo import yolo
from wr8_ai.yolo import bbox

class DetectorNCS:
    def __init__(self):
        pass

    def init(self, stick_idx, graph_path, config_path):
        model = ncs.InferNCS(graph_path, device_idx=stick_idx, fp16=False)

        if not model.is_opened():
            rospy.logerr('Failed to init device')
            return False

        rospy.loginfo('Model created')

        with open(config_path) as config_buffer:    
            config = json.load(config_buffer)
        
        rospy.loginfo('Config opened')

        self.labels = ['brick', 'forward', 'forward_left', 'forward_right', 'left', 'right']
        anchors = config['model']['anchors']

        net_h, net_w = config['model']['infer_shape']
        self.obj_thresh, nms_thresh = 0.5, 0.45

        self.inferer = yolo.YOLO(model, net_h, net_w, anchors, self.obj_thresh, nms_thresh)

        return True

    def get_signs(self, cv_img, render_img=None):
        # Return (ROS signs boxes, render_img) 

        boxes = self.inferer.make_infer([cv_img])[0]

        ros_bboxes = []
        for box in boxes:
            ros_bboxes += [ut.yolo_bbox_2_ros_bbox(box, self.labels)]

        if render_img is not None:
            bbox.draw_boxes(render_img, boxes, self.labels, self.obj_thresh)

        return ros_bboxes, render_img


import wr8_ai.tsr.image as tsr_im
import numpy as np
import keras

class RecognizerCPU:
    def __init__(self):
        pass

    def init(self, stick_idx, weights_path, config_path):
        from keras.models import load_model

        self.model = load_model(weights_path)

        rospy.loginfo('Model created')

        with open(config_path) as config_buffer:    
            config = json.load(config_buffer)
        
        rospy.loginfo('Config opened')

        self.labels = config['model']['labels']
        self.net_h, self.net_w = config['model']['infer_shape']

        return True

    def infer(self, cv_img):
        # Return (ROS signs boxes, render_img) 
        image = tsr_im.image_preprocess(cv_img, self.net_h, self.net_w)

        inf_img = np.expand_dims(image, axis=0)
        result = self.model.predict(inf_img)[0]
        print(result)
        max_idx = np.argmax(result)

        return self.labels[max_idx]


class RecognizerNCS:
    def __init__(self):
        pass

    def init(self, stick_idx, graph_path, config_path):
        self.model = ncs.InferNCS(graph_path, device_idx=stick_idx, fp16=False)

        if not self.model.is_opened():
            rospy.logerr('Failed to init device')
            return False

        rospy.loginfo('Model created')

        with open(config_path) as config_buffer:    
            config = json.load(config_buffer)
        
        rospy.loginfo('Config opened')

        self.labels = config['model']['labels']
        self.net_h, self.net_w = config['model']['infer_shape']

        return True

    def infer(self, cv_img):
        # Return (ROS signs boxes, render_img) 
        image = tsr_im.image_preprocess(cv_img, self.net_h, self.net_w)

        result = self.model.infer(image)
        print(result)
        max_idx = np.argmax(result)

        return self.labels[max_idx]

import cv2
import argparse
argparser = argparse.ArgumentParser(description='Test NCS NNs')
argparser.add_argument('-m', '--mode', help='yolo/rec/reccpu')
argparser.add_argument('-c', '--config', help='path to configuration file')
argparser.add_argument('-i', '--input', help='path to an image, a directory of images, a video, or webcam')
argparser.add_argument('-g', '--graph', help='graph path')
argparser.add_argument('-w', '--weights', help='weights path')

args = argparser.parse_args()

from wr8_ai.yolo import fps
import wr8_ai.nn_utils as nn_ut
import time

if __name__ == '__main__':
    input_path = args.input
    graph_path = args.graph
    weights_path = args.weights
    config_path = args.config
    nn_mode = args.mode

    if nn_mode == 'yolo':
        pass
    elif nn_mode == 'reccpu':
        recogn = RecognizerCPU()
        if not recogn.init(0, weights_path, config_path):
            print('Failed to init NCS recognizer')
    elif nn_mode == 'rec':
        recogn = RecognizerNCS()
        if not recogn.init(0, graph_path, config_path):
            print('Failed to init NCS recognizer')
    else:
        print('Invalid mode, check help!')

    data_generator = nn_ut.data_generator(input_path)
    fps_meter = fps.FPSMeter()

    for _, img in data_generator:
        start = time.time()

        result_cl = recogn.infer(img)
        
        fps_meter.update(time.time() - start)
        if fps_meter.milliseconds > 5000:
            fps_meter.print_statistics()
            fps_meter.reset()

        print(result_cl)

        cv2.imshow('1', img)
        if 27 == cv2.waitKey(0):
            break

