#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from time import sleep

print("Hello from test path publisher.")
node_name = "testPathPublisher"
pub_name = "/path"
frameId = "map"


PATH = tuple(((0, 0), (6, 0), (6, -4), (12, -4), (12, -8), (2, -8), (2,-14), (14, -14), (14, 0) ))
def pub_path():
	path = Path()
	path.header.seq = 1
	path.header.frame_id = frameId
	for i in range(0, len(PATH)):
		pose = PoseStamped()
		pose.header.seq = 1
		pose.header.frame_id = frameId
		pose.pose.position.x = PATH[i][0]
		pose.pose.position.y = PATH[i][1]
		path.poses.append(pose)
	path_pub.publish(path)
	print("published from node {} to topic {}".format(node_name, pub_name))


rospy.init_node(node_name)
path_pub = rospy.Publisher(pub_name, Path, queue_size=5)
sleep(1)
pub_path()
rospy.spin()

