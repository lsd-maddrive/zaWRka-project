#!/usr/bin/env python

import roslaunch
import rospy

rospy.init_node('prepare')
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

print('UUID generated: {}'.format(uuid))

lidar_cli_args = ['wr8_software', 'lidar.launch']
roslaunch_lidar_file = roslaunch.rlutil.resolve_launch_arguments(lidar_cli_args)

print('Software package root: {}'.format(roslaunch_lidar_file))

launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_lidar_file)

rospy.loginfo("created")

launch.start()
rospy.loginfo("started")

while not rospy.is_shutdown():
	rospy.loginfo('Processing')
	rospy.sleep(3)

launch.shutdown()
rospy.loginfo("stopped")
