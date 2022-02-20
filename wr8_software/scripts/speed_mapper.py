#!/usr/bin/env python2
import rospy
from copy import copy
import numpy as np

from geometry_msgs.msg import Twist

class CoefficientApproximator:
    """
    Using set of knowed command and speed correspondences, return coefficients
    of the linear model.
    """
    def __init__(self):
        self.data_set_positive = np.array([
            # [0.615, 0.7],   # rosbag
            [0.486, 0.57],   # rosbag
            [0, 0.475],       # let's assume
            ])
        self.data_set_negative = np.array([
            # [-0.615, -0.7],   # let's assume
            [-0.486, -0.57],   # let's assume
            [-0, -0.475],       # let's assume
            ])

    def calculate_coefs_from_metric_to_relative(self):
        x = self.data_set_positive[:, 0]
        y = self.data_set_positive[:, 1]
        positive_K1, positive_K0 = np.polyfit(x, y, 1, rcond=None, full=False, w=None, cov=False)

        x = self.data_set_negative[:, 0]
        y = self.data_set_negative[:, 1]
        negative_K1, negative_K0 = np.polyfit(x, y, 1, rcond=None, full=False, w=None, cov=False)
        return positive_K1, positive_K0, negative_K1, negative_K0

    def calculate_coefs_from_relative_to_metric(self):
        x = self.data_set_positive[:, 1]
        y = self.data_set_positive[:, 0]
        positive_K1, positive_K0 = np.polyfit(x, y, 1, rcond=None, full=False, w=None, cov=False)

        x = self.data_set_negative[:, 1]
        y = self.data_set_negative[:, 0]
        negative_K1, negative_K0 = np.polyfit(x, y, 1, rcond=None, full=False, w=None, cov=False)
        return positive_K1, positive_K0, negative_K1, negative_K0

class CmdVelMapper:
    """
    It allows to get metric cmd of speed and map it into the actual speed
    command received by robot package.
    """
    def __init__(self, positive_K1, positive_K0, negative_K1, negative_K0,
                 inverse_mode=False,
                 relative_topic='cmd_vel', metric_topic='cmd_vel_metric'):
        self.positive_K0 = positive_K0
        self.positive_K1 = positive_K1
        self.negative_K0 = negative_K0
        self.negative_K1 = negative_K1
        self.rad_to_relative = 1 / (25 * np.pi / 180)

        if inverse_mode is False:
            sub_topic = metric_topic
            pub_topic = relative_topic
        else:
            sub_topic = relative_topic
            pub_topic = metric_topic

        self.pub = rospy.Publisher(pub_topic, Twist, queue_size = 1, latch=True)
        rospy.Subscriber(sub_topic, Twist, self.handle)

    def handle(self, metric_cmd):
        actual_cmd = copy(metric_cmd)

        if metric_cmd.linear.x > 0:
            actual_cmd.linear.x = self.positive_K0 + self.positive_K1 * metric_cmd.linear.x
        elif metric_cmd.linear.x < 0:
            actual_cmd.linear.x = self.negative_K0 + self.negative_K1 * metric_cmd.linear.x

        actual_cmd.angular.z = metric_cmd.angular.z * self.rad_to_relative

        actual_cmd.linear.x = np.clip(actual_cmd.linear.x, -1, 1)
        actual_cmd.angular.z = np.clip(actual_cmd.angular.z, -1, 1)

        self.pub.publish(actual_cmd)


if __name__ == "__main__":
    # Init
    rospy.init_node('msg_converter_from_lsm_to_ekf')
    
    coef_approximator = CoefficientApproximator()
    metric_to_relative_coefs = coef_approximator.calculate_coefs_from_metric_to_relative()
    relative_to_metric_coefs = coef_approximator.calculate_coefs_from_relative_to_metric()

    print('metric_to_relative: POS_K1 = {}, POS_K0 = {}, NEG_K1 = {}, NEG_K0 = {}'.format(*metric_to_relative_coefs))
    print('relative_to_metric: POS_K1 = {}, POS_K0 = {}, NEG_K1 = {}, NEG_K0 = {}'.format(*relative_to_metric_coefs))

    is_inverse_mapping = rospy.get_param('~speed_mapper/is_inverse_mapping', False)
    if is_inverse_mapping is False:
        cmd_vel_mapper = CmdVelMapper(*metric_to_relative_coefs)
    else:
        cmd_vel_mapper = CmdVelMapper(*relative_to_metric_coefs, inverse_mode=True)

    rospy.spin()