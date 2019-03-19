import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class ImageReceiverROS:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera", Image, self.callback, queue_size=1)

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
