#!/usr/bin/env python2
import time
import math
from copy import copy
import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose2D, TwistWithCovariance, Twist
import numpy as np
from nav_msgs.msg import Odometry
from threading import Timer

def euler_to_quaternion(roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

class LsmPoseConverter:
    def __init__(self):
        self.pub = rospy.Publisher('lsm_pose_estimation', PoseWithCovarianceStamped, queue_size = 1, latch=True)
        self.frame_id = rospy.get_param('~frame_id', 'odom')
        rospy.Subscriber('pose2D', Pose2D, self.handle)
        self.angle = 0

        # params
        self.cov_x = 1
        self.cov_y = 1
        self.cov_yaw = 1e-3

    def handle(self, pose):
        pose_with_covariance = PoseWithCovarianceStamped()
        now = rospy.get_rostime()
        pose_with_covariance.header.stamp.secs = now.secs
        pose_with_covariance.header.stamp.nsecs = now.nsecs
        pose_with_covariance.header.frame_id = self.frame_id
        pose_with_covariance.pose.pose.position.x = pose.x
        pose_with_covariance.pose.pose.position.y = pose.y
        quat = euler_to_quaternion(0, 0, pose.theta)
        pose_with_covariance.pose.pose.orientation.z = quat[2]
        pose_with_covariance.pose.pose.orientation.w = quat[3]
        pose_with_covariance.pose.covariance[0] = self.cov_x
        pose_with_covariance.pose.covariance[7] = self.cov_y
        pose_with_covariance.pose.covariance[35] = self.cov_yaw
        self.pub.publish(pose_with_covariance)
        self.angle = pose.theta
    
    def get_angle(self):
        return self.angle

class BicycleModelTrackEstimator:
    def __init__(self, angle_getter):
        self.pub = rospy.Publisher('bicycle_model_pose_estimation', PoseWithCovarianceStamped, queue_size = 1, latch=True)
        self.tw_pub = rospy.Publisher('bicycle_model_twist', Twist, queue_size = 1, latch=True)
        self.frame_id = rospy.get_param('~frame_id', 'odom')
        self.angle_getter = angle_getter
        rospy.Subscriber('cmd_vel_metric', Twist, self.callback)

        self.msg_crnt_twist = None
        self.msg_prev_twist = None
        self.msg_crnt_timestamp = rospy.get_rostime()

        # timer
        frequency = 5
        self._PERIOD = 1.0 / frequency if frequency != 0 else 1.0
        self._timer = Timer(self._PERIOD, self.publish_pose)
        self._timer.start()
        
        self._tw_timer = Timer(self._PERIOD, self.publish_twist)
        self._tw_timer.start()

        # params:
        self.wheel_base = 0.340
        self.cov_x = 0.1
        self.cov_y = 0.1
        self.cov_yaw = 1e-3

        # state
        self.x = 0
        self.y = 0
        self.yaw = 0
        
        self.dx = 0
        self.dy = 0
        self.dyaw = 0

    def callback(self, twist_msg):
        self.msg_prev_twist = copy(self.msg_crnt_twist)
        self.msg_crnt_twist = copy(twist_msg)

        self.msg_prev_timestamp = copy(self.msg_crnt_timestamp)
        self.msg_crnt_timestamp = copy(rospy.get_rostime())
        if self.msg_prev_twist is not None and self.msg_crnt_twist is not None:
            self.update_data()
        else:
            rospy.logerr('BicycleModelTrackEstimator is not ready yet')

    def update_data(self):
        angle = self.angle_getter()
        speed = self.msg_crnt_twist.linear.x
        steer = self.msg_crnt_twist.angular.z if self.msg_prev_twist.angular.z != 0 else 0.0001
        time_step = self.msg_crnt_timestamp.secs - self.msg_prev_timestamp.secs + \
                    (self.msg_crnt_timestamp.nsecs - self.msg_prev_timestamp.nsecs) / 1000000000.0

        linear = speed * time_step
        angular = linear * math.tan(steer) / self.wheel_base

        if abs(angular) > 1e-3:
            curvature_radius = self.wheel_base / math.cos(math.pi / 2.0 - steer)
            elapsed_distance = linear
            elapsed_angle = elapsed_distance / curvature_radius
            x_curvature = curvature_radius * math.sin(elapsed_angle)
            y_curvature = curvature_radius * (math.cos(elapsed_angle) - 1.0)
            wheel_heading = angle + steer
            self.dx = x_curvature * math.sin(wheel_heading) + y_curvature * math.cos(wheel_heading)
            self.dy = x_curvature * math.cos(wheel_heading) - y_curvature * math.sin(wheel_heading)
            self.yaw = angle
            self.dyaw = elapsed_angle
        else:
            self.dx = linear * math.sin(angle)
            self.dy = linear * math.cos(angle)
            self.yaw = angle
            self.dyaw = 0
        
        self.x += self.dx
        self.y += self.dy

        self.dx /= time_step
        self.dy /= time_step
        self.dyaw /= time_step

    def publish_twist(self):
        if not rospy.is_shutdown():
            self._tw_timer = Timer(self._PERIOD, self.publish_twist)
            self._tw_timer.start()
            
        msg = Twist()
        msg.linear.x = self.dy
        msg.linear.y = self.dx
        msg.angular.z = self.dyaw

        self.tw_pub.publish(msg)

    def publish_pose(self):
        if not rospy.is_shutdown():
            self._timer = Timer(self._PERIOD, self.publish_pose)
            self._timer.start()
            
        pose_with_covariance = PoseWithCovarianceStamped()
        now = rospy.get_rostime()
        pose_with_covariance.header.stamp.secs = now.secs
        pose_with_covariance.header.stamp.nsecs = now.nsecs
        pose_with_covariance.header.frame_id = self.frame_id
        pose_with_covariance.pose.pose.position.x = self.y
        pose_with_covariance.pose.pose.position.y = self.x
        quat = euler_to_quaternion(0, 0, self.yaw)
        pose_with_covariance.pose.pose.orientation.z = quat[2]
        pose_with_covariance.pose.pose.orientation.w = quat[3]
        pose_with_covariance.pose.covariance[0] = self.cov_x
        pose_with_covariance.pose.covariance[7] = self.cov_y
        pose_with_covariance.pose.covariance[35] = self.cov_yaw
        self.pub.publish(pose_with_covariance)

class OdometryBroadcaster:
    def __init__(self):
        rospy.Subscriber('/odometry/filtered', Odometry, self.handle)

    def handle(self, msg):
        br = tf.TransformBroadcaster()
        quant = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                         quant,
                         rospy.Time.now(),
                         'base_footprint',
                         "odom")

if __name__ == "__main__":
    # Init
    rospy.init_node('supported_stuff_for_ekf')
    
    listener = tf.TransformListener()
    def tf_angle_getter():
        # Inverted order to keep signs
        _, rot = listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
        _, _, yaw = tf.transformations.euler_from_quaternion(rot)
        
        return yaw
    
    pose_converter = LsmPoseConverter()
    angle_getter = lambda: pose_converter.get_angle()
    # angle_getter works better than TF getter
    cmd_vel_tracker_estimator = BicycleModelTrackEstimator(angle_getter)
    odom_broadcaster = OdometryBroadcaster()

    rospy.spin()
