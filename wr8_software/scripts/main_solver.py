#!/usr/bin/env python

import rospy
import roslaunch
from std_msgs.msg import Int8
from std_srvs.srv import Trigger, TriggerRequest 

import maze_solver as mz 
import time

state_topic_name = 'robot/state'
reset_odon_srv_name = 'robot/reset_odometry'

gazebo_test = False

class StateProcessor:

    IDLE = 0
    RUN = 1
    STOP = 2
    
    def __init__(self):

        state_sub = rospy.Subscriber(state_topic_name, Int8, self.state_cb, queue_size = 10)

        # self.state_names = ['Idle', 'Run']
        self.states = {self.IDLE: self.idle_init_state, 
                       self.RUN: self.run_init_state,
                       self.STOP: self.stop_init_state}

        self.last_state = self.IDLE
        self.state_changed = False

        rospy.loginfo('StateProcessor initialized')

    def state_cb(self, msg):

        new_state = msg.data

        if new_state not in self.states:
            rospy.logwarn('Invalid state: {}'.format(new_state))
            rospy.logwarn('Possible states: {}'.format(self.states))
            return

        if self.last_state == new_state:
            return
        else:
            self.last_state = new_state
            self.states[new_state]()

            self.state_changed = True

    def idle_init_state(self):
        rospy.loginfo('New state - IDLE')

    def stop_init_state(self):
        rospy.loginfo('New state - STOP')

    def run_init_state(self):
        rospy.loginfo('New state - RUN')
        
        if not gazebo_test:
            rospy.wait_for_service(reset_odon_srv_name)
            reset_odom_prx = rospy.ServiceProxy(reset_odon_srv_name, Trigger)

            try:
                resp = reset_odom_prx()
                rospy.loginfo('Reset resp: {}'.format(resp))
            except:
                rospy.logwarn('Bad service resp')

    def is_state_changed(self):
        if self.state_changed:
            self.state_changed = False
            return True

        return False


class ControllerLocalization:
    def __init__(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        rospy.loginfo('UUID generated: {}'.format(uuid))

        roslaunch.configure_logging(uuid)

        if gazebo_test:
            lidar_cli_args = ['wr8_software', 'gz_localization.launch']
        else:
            lidar_cli_args = ['wr8_software', 'localization.launch']

        roslaunch_lidar_file = roslaunch.rlutil.resolve_launch_arguments(lidar_cli_args)

        self.launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_lidar_file, verbose=True, force_screen=True)

        self.is_enabled = False
        rospy.loginfo("ControllerLocalization initialized")

    def __del__(self):
        self.stop()

    def start(self):
        if self.is_enabled:
            return

        self.is_enabled = True
        self.launch.start()
        rospy.loginfo("Enabled")

    def stop(self):
        if not self.is_enabled:
            return

        self.is_enabled = False
        self.launch.shutdown()
        rospy.loginfo("Stopped")

from wr8_ai.detectors import DoubleDetector
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
import cv2
import numpy as np

class SignsDetector:

    def __init__(self):
        pass

    def init(self):
        rec_weights_path = rospy.get_param('~rec_weights_path')
        rec_config_path = rospy.get_param('~rec_config_path')

        det_graph_path = rospy.get_param('~det_graph_path')
        det_config_path = rospy.get_param('~det_config_path')

        self.detector = DoubleDetector()
        if not self.detector.init(rec_config_path, rec_weights_path, det_config_path, det_graph_path):
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


from graph_path.car_state import *

def main():
    rospy.init_node('main_solver')

    state_pr = StateProcessor()
    slam_ctr = ControllerLocalization()

    signs_detector = None#SignsDetector()
    # if not signs_detector.init():
        # rospy.logerr('Failed to initilize detector')
        # signs_detector = None

    rate = rospy.Rate(20)

    solver = mz.MazeSolver()
    
    first_start_interrupted = False

    last_point_idx = -1
    last_points = [(13.5, -14.6, 0), (14, 0.8, 90)]

    while not rospy.is_shutdown():
        if signs_detector:
            current_signs_rbboxes = signs_detector.get_signs()

        if state_pr.is_state_changed():

            if state_pr.last_state == StateProcessor.RUN:
                if first_start_interrupted:
                    rospy.logwarn('Solver called after movement, reset node!')
                else:
                    time.sleep(5)
                    rospy.loginfo('Sleep end, recover behaviour')

                    slam_ctr.start()
                    solver.init()
                    solver.set_next_cross_goal()
            elif state_pr.last_state == StateProcessor.STOP:
                slam_ctr.stop()
                first_start_interrupted = True
            elif state_pr.last_state == StateProcessor.IDLE:
                slam_ctr.stop()
                first_start_interrupted = True

        if state_pr.last_state == StateProcessor.RUN and not first_start_interrupted:
            # Main logic
            if not solver.is_maze_completed:

                if solver.is_goal_completed:
                    # Signs test
                    if solver.is_sign_required():
                        seleceted_sign = SIGNS_NONE
                        solver.set_vision_sign(seleceted_sign)

                        rospy.loginfo('Set selected sign: {} [{}]'.format(sign_names[seleceted_sign], seleceted_sign))

                    if not solver.set_next_cross_goal():
                        rospy.loginfo('Failed to get maze path')

                        solver.set_vision_sign(SIGNS_NONE)
                        if not solver.set_next_cross_goal():
                            # Check again, failed
                            rospy.loginfo('Failed to get maze path (2)')
                            break
                elif solver.is_goal_failed:
                    solver.recover_current_goal()
                else:
                    time.sleep(0.1)

            else:
                from actionlib_msgs.msg import GoalStatus

                if solver.is_goal_completed:
                    last_point_idx += 1
                    if last_point_idx >= len(last_points):
                        break

                    point = last_points[last_point_idx]
                    solver.set_direct_goal((point[0], point[1]), point[2])

                elif solver.is_goal_failed:
                    point = last_points[last_point_idx]
                    solver.set_direct_goal((point[0], point[1]), point[2])
                
                else:
                    time.sleep(0.1)

        rate.sleep()

    rospy.loginfo('Solver completed, yeah!')


if __name__ == '__main__':
    main()
