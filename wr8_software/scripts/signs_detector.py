#!/usr/bin/env python2
import rospy
from threading import Timer
from sensor_msgs.msg import Image
from mad_detector.detector import RFSignsDetector, draw_results
from mad_detector.msg import Detection, Detections
from get_light import LightColorDetector


class SignDetector:
    def __init__(self, receiver, publisher, frequency=1):
        MODEL_PATH = rospy.get_param('~mad_detector/model_path')
        DETECTED_SIGN_TOPIC = rospy.get_param('~detected_sign_type_topic', "detected_sign_type")
        self._PERIOD = 1.0 / frequency
        self._sign_pub = rospy.Publisher(DETECTED_SIGN_TOPIC, Detections, queue_size=5)
        # self._sign_image_pub = rospy.Publisher()
        self._detector = RFSignsDetector(MODEL_PATH)
        self._img = None
        self._enable = False
        self._timer = None
        self._img_receiver = receiver
        self._img_publisher = publisher

    def start(self):
        self._enable = True
        if self._timer is None and not rospy.is_shutdown():
            self._timer = Timer(self._PERIOD, self._process)
            self._timer.start()

    def stop(self):
        self._enable = False
        if self._timer is not None:
            self._timer.cancel()

    def _process(self):
        if self._enable is True and not rospy.is_shutdown():
            self._timer = Timer(self._PERIOD, self._process)
            self._timer.start()
        
        img = self._img_receiver.get()
        if img is None:
            rospy.logerr('SignDetector: there is no image')
            return

        bboxes, label_names, scores = self._detector.find_signs(img)
        self._detections = self._create_msg(img, bboxes, label_names, scores)
        try:
            self._sign_pub.publish(self._detections)
        except rospy.exceptions.ROSException as e:
            rospy.logerr('SignDetector: %s', str(e))

        # Send image to topic
        if self._img_publisher.isEnabled():
            try:
                # Draw results and show
                canvas = draw_results(img, (bboxes, label_names, scores))
                self._img_publisher.send(canvas)
                
            except rospy.exceptions.ROSException as e:
                rospy.logerr('SignDetector: %s', str(e))

    def _create_msg(self, img, bboxes, label_names, scores):
        detections = Detections()
        for idx in range(len(label_names)):
            detection = Detection()
            # TODO - move to Enum
            if label_names[idx] == 'traffic_light':
                light_color_detector = LightColorDetector()
                color = light_color_detector.get_light(img, bboxes[idx])
                # TODO - move to Enum
                if color == 'RED':
                    label_names[idx] = 'traffic_light_red'
                else:
                    label_names[idx] = 'traffic_light_green'

            detection.object_class = label_names[idx]
            detection.probability = float(scores[idx])
            # we are not interested in ul_point and br_point yet, so we ignore it
            detection.size_px.x = bboxes[idx][2]
            detection.size_px.y = bboxes[idx][3]

            detections.detections.append(detection)

        return detections

if __name__ == "__main__":
    rospy.init_node('test_node')
    detector = SignDetector()
    RATE = rospy.Rate(1)
    while not rospy.is_shutdown():
        detector.process()
        RATE.sleep()