#!/usr/bin/env python

import rospy
import json
from wr8_ai.msg import BoundingBox
import wr8_ai.utils as ut
import wr8_ai.ncs as ncs
from wr8_ai.yolo import yolo
from wr8_ai.yolo import bbox
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage

class DetectorNCS:
    def __init__(self):
        pass

    def init(self, stick_idx, graph_path, config_path):
        model = ncs.InferNCS(graph_path, device_idx=stick_idx, fp16=False)

        rospy.loginfo('Opened params: {} / {}'.format(graph_path, config_path))

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

    def infer(self, cv_img):
        # Return (ROS signs boxes, render_img) 

        boxes = self.inferer.make_infer([cv_img])[0]

        ros_bboxes = []
        for box in boxes:
            # Fixed - current NMS removes class porbabilities, 
            if box.get_score() > 0:
                ros_bboxes += [ut.yolo_bbox_2_ros_bbox(box, self.labels)]

        yolo_bboxes = boxes

        return ros_bboxes, yolo_bboxes

    def draw_boxes(self, img, ros_boxes):
        bbox.draw_ros_boxes(img, ros_boxes, self.labels, self.obj_thresh)


import wr8_ai.tsr.image as tsr_im
import numpy as np
import keras

class RecognizerCPU:
    def __init__(self):
        pass

    def init(self, weights_path, config_path):
        from keras.models import load_model

        rospy.loginfo('Opened params: {} / {}'.format(weights_path, config_path))

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
        # print(result)
        max_idx = np.argmax(result)

        return self.labels[max_idx]


class RecognizerNCS:
    def __init__(self):
        pass

    def init(self, stick_idx, graph_path, config_path):
        self.model = ncs.InferNCS(graph_path, device_idx=stick_idx, fp16=False)

        rospy.loginfo('Opened params: {} / {}'.format(graph_path, config_path))

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


class DoubleDetector:
    def __init__(self):
        pass

    def init(self, config_rec, weights_rec, graph_rec, config_det, graph_det):
        self.nn_det = DetectorNCS()
        if not self.nn_det.init(0, graph_det, config_det):
            print('Failed to init NCS detector')
            return False

        if 1:
            self.nn_model = RecognizerCPU()
            if not self.nn_model.init(weights_rec, config_rec):
                print('Failed to init CPU recognizer')
                return False

        else:
            self.nn_model = RecognizerNCS()
            if not self.nn_model.init(1, graph_rec, config_rec):
                print('Failed to init CPU recognizer')
                return False

        return True

    def infer(self, cv_img, yolo_img, corr_img):
        # render_img = img.copy()
        ros_bboxes, yolo_bboxes = self.nn_det.infer(cv_img)
        
        yolo_result_img = cv_img.copy()

        self.nn_det.draw_boxes(yolo_img, ros_bboxes)

        corrected_ros_bboxes = []

        for ros_box in ros_bboxes:
            # print('Expected class: {}'.format(ros_box.Class))
            small_img = cv_img[ros_box.ymin:ros_box.ymax, ros_box.xmin:ros_box.xmax]
            result_cl = self.nn_model.infer(small_img)
            
            if result_cl == 'negative':
                continue

            ros_box.Class = result_cl

            corrected_ros_bboxes += [ros_box]
            # print('Predicted class: {}'.format(result_cl))

        self.nn_det.draw_boxes(corr_img, corrected_ros_bboxes)

        return corrected_ros_bboxes


class SignsDetector:

    def __init__(self):
        pass

    def init(self):
        rec_weights_path = rospy.get_param('~rec_weights_path')
        rec_config_path = rospy.get_param('~rec_config_path')
        rec_graph_path = rospy.get_param('~rec_graph_path')

        det_graph_path = rospy.get_param('~det_graph_path')
        det_config_path = rospy.get_param('~det_config_path')

        self.detector = DoubleDetector()
        if not self.detector.init(rec_config_path, rec_weights_path, rec_graph_path, det_config_path, det_graph_path):
            rospy.logerr('Failed to init detectors')
            self.initialized = False
            return False

        self.initialized = True

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("nn_input", Image, self.callback_img, queue_size=1)

        self.image_pub_yolo = rospy.Publisher("yolo/compressed", CompressedImage)
        self.image_pub_yolo_corr = rospy.Publisher("yolo_corr/compressed", CompressedImage)
        # self.image_sub = rospy.Subscriber("camera_compr", CompressedImage, self.callback_img_compressed, queue_size=1)

        self.cv_image = None
        # self.cv_image_comp = None

        return True

    def callback_img(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(e)

    def get_signs(self):
        if self.cv_image is None:
            return []

        yolo_img = self.cv_image.copy()
        corr_yolo_img = self.cv_image.copy()

        ros_boxes = self.detector.infer(self.cv_image, yolo_img, corr_yolo_img)

        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "png"
        msg.data = np.array(cv2.imencode('.png', yolo_img)[1]).tostring()
        self.image_pub_yolo.publish(msg)

        msg.data = np.array(cv2.imencode('.png', corr_yolo_img)[1]).tostring()
        self.image_pub_yolo_corr.publish(msg)

        return ros_boxes

    def get_image(self):
        return self.cv_image


import cv2
import argparse
from wr8_ai.yolo import fps
import wr8_ai.nn_utils as nn_ut
import time

def full_test(config_rec, weights_rec, config_det, graph_det, input_path):
    print('Full test!')

    detector = DoubleDetector()
    if not detector.init(config_rec, weights_rec, config_det, graph_det):
        exit(1)

    data_generator = nn_ut.data_generator(input_path)
    fps_meter = fps.FPSMeter()

    for _, img in data_generator:
        start = time.time()

        yolo_img = img.copy()
        corr_yolo_img = img.copy()

        ros_boxes = detector.infer(img, yolo_img, corr_yolo_img)

        fps_meter.update(time.time() - start)
        if fps_meter.milliseconds > 5000:
            fps_meter.print_statistics()
            fps_meter.reset()

        cv2.imshow('result', yolo_img)  
        cv2.imshow('corr_result', corr_yolo_img)  

        if 27 == cv2.waitKey(0):
            break

        fps_meter.print_statistics()



if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description='Test NCS NNs')
    argparser.add_argument('-m', '--mode', help='yolo/rec/reccpu')
    argparser.add_argument('-cr', '--configrec', help='path to configuration file')
    argparser.add_argument('-cd', '--configdet', help='path to configuration file')
    argparser.add_argument('-i', '--input', help='path to an image, a directory of images, a video, or webcam')
    argparser.add_argument('-g', '--graph', help='graph path')
    argparser.add_argument('-w', '--weights', help='weights path')

    args = argparser.parse_args()

    input_path = args.input
    graph_path = args.graph
    weights_path = args.weights
    config_rec_path = args.configrec
    config_det_path = args.configdet
    nn_mode = args.mode

    if nn_mode == 'yolo':
        nn_model = DetectorNCS()
        if not nn_model.init(0, graph_path, config_det_path):
            print('Failed to init NCS detector')
            exit(1)
    elif nn_mode == 'reccpu':
        nn_model = RecognizerCPU()
        if not nn_model.init(weights_path, config_rec_path):
            print('Failed to init CPU recognizer')
            exit(1)
    elif nn_mode == 'rec':
        nn_model = RecognizerNCS()
        if not nn_model.init(0, graph_path, config_rec_path):
            print('Failed to init NCS recognizer')
            exit(1)
    elif nn_mode == 'full':
        full_test(config_rec_path, weights_path, config_det_path, graph_path, input_path)
        exit(0)
    else:
        print('Invalid mode, check help!')

    data_generator = nn_ut.data_generator(input_path)
    fps_meter = fps.FPSMeter()

    for _, img in data_generator:
        start = time.time()

        if nn_mode == 'yolo':
            render_img = img.copy()
            result_bxs = nn_model.infer(img, render_img=render_img)
        else:
            result_cl = nn_model.infer(img)


        fps_meter.update(time.time() - start)
        if fps_meter.milliseconds > 5000:
            fps_meter.print_statistics()
            fps_meter.reset()

        if nn_mode == 'yolo':
            cv2.imshow('result', render_img)            
        else:
            print(result_cl)
            cv2.imshow('orig', img)

        if 27 == cv2.waitKey(0):
            break

        fps_meter.print_statistics()
