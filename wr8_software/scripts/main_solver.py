#!/usr/bin/env python

import rospy
import roslaunch
from std_msgs.msg import Int8
from std_srvs.srv import Trigger, TriggerRequest 

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

		time.sleep(5)
		rospy.loginfo('Sleep end, recover behaviour')


	def is_state_changed(self):
		if self.state_changed:
			self.state_changed = False
			return True

		return False


class ControllerSLAM:
	def __init__(self):
		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		rospy.loginfo('UUID generated: {}'.format(uuid))

		roslaunch.configure_logging(uuid)

		lidar_cli_args = ['wr8_software', 'slam.launch']
		roslaunch_lidar_file = roslaunch.rlutil.resolve_launch_arguments(lidar_cli_args)

		self.launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_lidar_file, verbose=True, force_screen=True)

		self.is_enabled = False
		rospy.loginfo("ControllerSLAM initialized")

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


def main():
	rospy.init_node('main_solver')

	state_pr = StateProcessor()

	slam_ctr = ControllerSLAM()

	rate = rospy.Rate(20)

	while not rospy.is_shutdown():
		if state_pr.is_state_changed():

			if state_pr.last_state == StateProcessor.RUN:
				slam_ctr.start()
			elif state_pr.last_state == StateProcessor.STOP:
				slam_ctr.stop()
			elif state_pr.last_state == StateProcessor.IDLE:
				slam_ctr.stop()

		rate.sleep()


if __name__ == '__main__':
	main()
