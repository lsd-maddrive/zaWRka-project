#!/usr/bin/env python
import math

import rospy
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point
from car_parking.msg import Point2D
from car_parking.msg import Points2D
from car_parking.msg import Polygons
from car_parking.msg import Statuses

NODE_NAME = 'parking_test_pub_sub_node'
POLY_PUB_TOPIC = "parking_polygones"
RVIZ_PUB_TOPIC_1 = "parking_polygones_visualization_1"
RVIZ_PUB_TOPIC_2 = "parking_polygones_visualization_2"
RVIZ_PUB_TOPIC_3 = "parking_polygones_visualization_3"
STATUS_SUB_TOPIC = "parking_status"
CMD_TOPIC = "parking_cmd"

def status_callback(msg):
    print "I heard statuses with length {}: ".format(len(msg.fullness)),
    for i in range(0, len(msg.fullness)):
        print msg.messages[i],
        if msg.fullness[i] != -1:
            print "(" + str(msg.fullness[i]) + ") ",
    print ""

OFFSET_X = float(rospy.get_param('/car_parking_test/start_x'))
OFFSET_Y = float(rospy.get_param('/car_parking_test/start_y'))
OFFSET_Z = float(rospy.get_param('/car_parking_test/start_z'))

def create_polygon(polygon):
    p1 = Point2D(); p1.x = polygon[0][0]; p1.y = polygon[0][1]
    p2 = Point2D(); p2.x = polygon[1][0]; p2.y = polygon[1][1]
    p3 = Point2D(); p3.x = polygon[2][0]; p3.y = polygon[2][1]
    p4 = Point2D(); p4.x = polygon[3][0]; p4.y = polygon[3][1]
    poly = Points2D()
    poly.points = (p1, p2, p3, p4)
    return poly

def create_polygon_stamped(polygon):
    p1 = Point(); p1.x = polygon[0][0]; p1.y = polygon[0][1]
    p2 = Point(); p2.x = polygon[1][0]; p2.y = polygon[1][1]
    p3 = Point(); p3.x = polygon[2][0]; p3.y = polygon[2][1]
    p4 = Point(); p4.x = polygon[3][0]; p4.y = polygon[3][1]

    polygons_stumped = PolygonStamped()
    polygons_stumped.header.stamp = rospy.get_rostime()
    polygons_stumped.header.frame_id = 'map'
    polygons_stumped.polygon.points = tuple((p1, p2, p3, p4))
    return polygons_stumped

def gz_to_map_for_point(point):
    x = (point[0] - OFFSET_X) * math.cos(-OFFSET_Z) - (point[1] - OFFSET_Y) * math.sin(-OFFSET_Z)
    y = (point[0] - OFFSET_X) * math.sin(-OFFSET_Z) + (point[1] - OFFSET_Y) * math.cos(-OFFSET_Z)
    return (x, y)

def gz_to_map_for_polygon(polygon):
    new_polygon = tuple()
    for point in polygon:
        new_polygon += (gz_to_map_for_point(point),) # , means singleton
    return new_polygon

def create_parking():
    gz_polygon1 = ((4.0, 12.4), (5.6, 14.4), (5.6, 15.6), (4.0, 13.6))
    gz_polygon2 = ((4.0, 14.4), (5.6, 16.4), (5.6, 17.6), (4.0, 15.6))
    gz_polygon3 = ((4.0, 16.4), (5.6, 18.4), (5.6, 19.6), (4.0, 17.6))

    #gz_polygon1 = ((4.0, 12.0), (6.0, 12.0), (6.0, 16.0), (4.0, 16.0))
    #gz_polygon2 = ((4.0, 14.0), (6.0, 14.0), (6.0, 18.0), (4.0, 18.0))
    #gz_polygon3 = ((4.0, 18.0), (6.0, 18.0), (6.0, 20.0), (4.0, 20.0))

    map_polygon1 = gz_to_map_for_polygon(gz_polygon1)
    map_polygon2 = gz_to_map_for_polygon(gz_polygon2)
    map_polygon3 = gz_to_map_for_polygon(gz_polygon3)

    polygon1 = create_polygon(map_polygon1)
    polygon2 = create_polygon(map_polygon2)
    polygon3 = create_polygon(map_polygon3)

    polygon_stamped1 = create_polygon_stamped(map_polygon1)
    polygon_stamped2 = create_polygon_stamped(map_polygon2)
    polygon_stamped3 = create_polygon_stamped(map_polygon3)

    polygons = Polygons()
    polygons.polygons = (polygon1, polygon2, polygon3)
    polygons_stamped = (polygon_stamped1, polygon_stamped2, polygon_stamped3)
    both_polygons = [polygons, polygons_stamped]
    return both_polygons

def talker():
    rate = rospy.Rate(0.2)
    both_polygons = create_parking()
    while not rospy.is_shutdown():
        rospy.loginfo("I published poly.")
        PUB_TO_PARKING.publish(both_polygons[0])
        PUB_TO_RVIZ_1.publish(both_polygons[1][0])
        PUB_TO_RVIZ_2.publish(both_polygons[1][1])
        PUB_TO_RVIZ_3.publish(both_polygons[1][2])
        rate.sleep()

if __name__=="__main__":
    try:
        rospy.init_node(NODE_NAME)
        SUB = rospy.Subscriber(STATUS_SUB_TOPIC, Statuses, status_callback, queue_size=10)
        PUB_TO_PARKING = rospy.Publisher(POLY_PUB_TOPIC, Polygons, queue_size=10)
        PUB_TO_RVIZ_1 = rospy.Publisher(RVIZ_PUB_TOPIC_1, PolygonStamped, queue_size=10)
        PUB_TO_RVIZ_2 = rospy.Publisher(RVIZ_PUB_TOPIC_2, PolygonStamped, queue_size=10)
        PUB_TO_RVIZ_3 = rospy.Publisher(RVIZ_PUB_TOPIC_3, PolygonStamped, queue_size=10)
        talker()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.logerr('Exception catched')
