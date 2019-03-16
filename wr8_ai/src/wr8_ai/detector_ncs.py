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

        self.labels = ['brick', 'forward', 'forward and left', 'forward and right', 'left', 'right']
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
