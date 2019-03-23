#!/usr/bin/env python

import rospy
import roslaunch
from std_msgs.msg import Int8
from std_srvs.srv import Trigger, TriggerRequest 

import maze_solver as mz 
import time

state_topic_name = 'robot/state'
reset_odon_srv_name = 'robot/reset_odometry'

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

        maze_cli_args = ['wr8_software', 'localization.launch', 'fast:=false']
        speed_cli_args = ['wr8_software', 'localization.launch', 'fast:=true']

        roslaunch_maze_file = roslaunch.rlutil.resolve_launch_arguments(maze_cli_args)
        roslaunch_speed_file = roslaunch.rlutil.resolve_launch_arguments(speed_cli_args)

        self.maze_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_maze_file, verbose=True)
        self.speed_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_speed_file, verbose=True)

        self.is_maze_enabled = False
        self.is_speed_enabled = False
        rospy.loginfo("Controller Localization initialized")

    def __del__(self):
        self.stop()

    def start_maze(self):
        if self.is_maze_enabled:
            return

        self.is_maze_enabled = True
        self.maze_launch.start()
        rospy.loginfo("Maze enabled")

    def start_speed(self):
        if self.is_speed_enabled:
            return

        self.is_speed_enabled = True
        self.speed_launch.start()
        rospy.loginfo("Speed enabled")


    def stop(self):
        if self.is_maze_enabled:
            self.is_maze_enabled = False
            self.maze_launch.shutdown()

            rospy.loginfo("Maze stopped")

        if self.is_speed_enabled:
            self.is_speed_enabled = False
            self.speed_launch.shutdown()

            rospy.loginfo("Speed stopped")


from wr8_ai.detectors import DoubleDetector
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
import cv2
import numpy as np
from wr8_ai.detectors import SignsDetector

from std_msgs.msg import UInt8

from graph_path.car_state import *
from wr8_ai.yolo import fps

node_idx = 0

def idx_cb(msg):
    global node_idx
    node_idx = msg.data
    rospy.loginfo('New idx: {}'.format(node_idx))

from wr8_ai.srv import ObjectDetection
from wr8_ai.srv import WhiteLineDetection

def main():
    rospy.init_node('main_solver')
    
    c_node_idx = 0
    wait4cmd = False
    node_idx_sub = rospy.Subscriber('node_idx', UInt8, idx_cb)

    state_pr = StateProcessor()
    slam_ctr = ControllerLocalization()

    rate = rospy.Rate(20)

    solver = mz.MazeSolver()
    
    first_start_interrupted = False

    last_point_idx = -1
    last_points = [(13.5, -14.6, 0), (14, 0.8, 90)]

    maze_flag = False
    speed_region_flag = False
    init_stage_idx = 0

    test_tl_flag = False

    complete_time = 0

    signs_stats = {}

    wl_detection_service = 'wl/detect'
    # rospy.wait_for_service(wl_detection_service)
    # detect_wl_srv = rospy.ServiceProxy(wl_detection_service, WhiteLineDetection)

    nn_detection_service = 'nn/detect_signs'
    # rospy.wait_for_service(nn_detection_service)
    # detect_signs_srv = rospy.ServiceProxy(nn_detection_service, ObjectDetection)

    rospy.loginfo('Ready 2 go!')

    while not rospy.is_shutdown():
        if state_pr.is_state_changed():

            if state_pr.last_state == StateProcessor.RUN:
                if first_start_interrupted:
                    rospy.logwarn('Solver called after movement, reset node!')
                else:
                    start_init_time = time.time()
                    init_stage_idx += 1
            elif state_pr.last_state == StateProcessor.STOP:
                slam_ctr.stop()
                maze_flag = False
                speed_region_flag = False
                init_stage_idx = 0
            elif state_pr.last_state == StateProcessor.IDLE:
                slam_ctr.stop()
                maze_flag = False
                speed_region_flag = False
                init_stage_idx = 0


        # Initialization block
        if init_stage_idx == 1:
            if time.time() - start_init_time > 5:
                init_stage_idx += 1
                rospy.loginfo('Sleep end, recover behaviour')
        elif init_stage_idx == 2:
            slam_ctr.start_maze()
            solver.init()
            rospy.loginfo('Initialized')
            init_stage_idx += 1
        elif init_stage_idx == 3:
            solver.set_next_cross_goal()
            first_start_interrupted = True
            maze_flag = True
            init_stage_idx = 0
            signs_stats = {}

            # # For debug
            # time.sleep(1)
            # slam_ctr.stop()

            # init_stage_idx = 11
            # print('>>>', init_stage_idx)
            # continue

        elif init_stage_idx == 11:
            slam_ctr.start_speed()
            solver.init()
            speed_region_flag = True
            init_stage_idx = 0
            rospy.loginfo('Start speed region: {}'.format(solver.is_goal_completed))


        if 0 and not speed_region_flag:
            current_signs = []
            try:
                rospy.wait_for_service(nn_detection_service, timeout=0.5)
                current_signs = detect_signs_srv()
            except:
                rospy.logwarn('Failed to call NN service')
            
            for sign_bbox in current_signs.bboxes:
                if sign_bbox.Class in signs_stats:
                    signs_stats[sign_bbox.Class] += 1
                else:
                    signs_stats[sign_bbox.Class] = 1

            wl_coordinates = -1
            try:
                rospy.wait_for_service(nn_detection_service, timeout=0.5)
                wl_coordinates = detect_wl_srv()
                wl_coordinates = wl_coordinates.y_coord.data
            except:
                rospy.logwarn('Failed to call WL service')
        

        # Maze logic
        if maze_flag:
            # Main logic
            if not solver.is_maze_completed:
 
                if solver.is_goal_completed:
                    if complete_time == 0:
                        complete_time = time.time()

                    # Wait for signs
                    if time.time() - complete_time > 1:
                        rate.sleep()
                        continue

                    complete_time = 0

                    if solver.is_sign_required():
                        seleceted_sign = SIGNS_NONE

                        if signs_stats:
                            seen_sign = max(signs_stats, key=signs_stats.get)

                            tans_signs = {
                                            'brick' : SIGNS_NO_PATH,
                                            'forward' : SIGNS_FORWARD, 
                                            'forward_left' : SIGNS_FORWARD_LEFT,
                                            'forward_right' : SIGNS_FORWARD_RIGHT, 
                                            'left' : SIGNS_LEFT, 
                                            'right' : SIGNS_RIGHT, 
                                            'negative' : SIGNS_NONE,
                                            'traffic_sign' : SIGNS_NONE
                                        }

                            if signs_stats[seen_sign] < 100:
                                seleceted_sign = SIGNS_NONE    
                            else:
                                seleceted_sign = tans_signs[seen_sign]

                        solver.set_vision_sign(seleceted_sign)

                        rospy.loginfo('Stats: {}'.format(signs_stats))
                        rospy.loginfo('Set selected sign: {} [{}]'.format(sign_names[seleceted_sign], seleceted_sign))

                    # if c_node_idx >= node_idx:
                        # print('Sleep for next node {}, stats: {}'.format(node_idx, signs_stats))
                        # wait4cmd = True
                        # continue

                    # if wait4cmd:
                        signs_stats = {}

                    # if c_node_idx >= node_idx:
                    #     rospy.loginfo('Node is waiting')
                    #     continue

                    c_node_idx += 1
                    
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
                    rate.sleep()
                    continue
            else:
                slam_ctr.stop()

                init_stage_idx = 11
                maze_flag = False

        if speed_region_flag:
                from actionlib_msgs.msg import GoalStatus

                if solver.is_goal_completed:
                    rospy.loginfo('Set goal')
                    
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
