import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage


class ImageReceiverROS:

    def __init__(self, topic_name):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic_name, Image, self.callback, queue_size=1)

        self.cv_image = None
        self.skip_cntr = 0

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(e)

    def getImage(self):
        if self.cv_image is None:
            self.skip_cntr += 1

            if self.skip_cntr > 3:
                self.skip_cntr = 0
                rospy.logwarn('No image for 3 times...')

        return self.cv_image


def yolo_bbox_2_ros_bbox(yolo_bbox, labels):
    from wr8_ai.msg import BoundingBox

    # print("Input: {}",format(yolo_bbox.get_str()))

    ros_box = BoundingBox()
    ros_box.Class = labels[yolo_bbox.get_label()]
    ros_box.probability = yolo_bbox.c
    ros_box.xmin = yolo_bbox.xmin
    ros_box.ymin = yolo_bbox.ymin
    ros_box.xmax = yolo_bbox.xmax
    ros_box.ymax = yolo_bbox.ymax

    # print("Output: {}",format(ros_box))

    return ros_box

class ImageReceiverROS:
    from cv_bridge import CvBridge, CvBridgeError
    from sensor_msgs.msg import Image, CompressedImage

    def __init__(self, topic_name):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic_name, Image, self.callback_img, queue_size=1)
        # self.image_sub_compres = rospy.Subscriber("camera_compr", CompressedImage, self.callback_img_compressed, queue_size=1)

        self.cv_image = None
        self.cv_image_comp = None

    def callback_img(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(e)

    def callback_img_compressed(self, data):
        np_arr = np.fromstring(data.data, np.uint8)
        self.cv_image_comp = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def get_image(self):
        return self.cv_image

    def get_image_compressed(self):
        return self.cv_image_comp

import cv2
import numpy as np

class ImagePublisherROS:
    def __init__(self, topic_name):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher(topic_name + '/compressed', CompressedImage, queue_size=10)

    def publish(self, cv_image):
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "png"
        msg.data = np.array(cv2.imencode('.png', cv_image)[1]).tostring()

        self.image_pub.publish(msg)


