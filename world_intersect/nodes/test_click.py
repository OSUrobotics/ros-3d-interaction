#!/usr/bin/env python
import roslib; roslib.load_manifest('projected_lightswitch_interface')
import rospy
import numpy as np
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import PointCloud2, PointCloud
from pr2_object_manipulation_msgs.msg import ImageClick
from copy import deepcopy
from pr2_python.pointclouds import xyz_array_to_pointcloud2

int_pub = None

def click_cb(msg):
	point = msg.ray.direction
	origin = msg.ray.origin
	cloud = xyz_array_to_pointcloud2(np.array([[
		point.x,
		point.y,
		point.z
	]]))
	cloud.header = msg.ray.header
	int_pub.publish(cloud)
	print 'published', point

if __name__ == '__main__':
	rospy.init_node('test_cursor')
	int_pub = rospy.Publisher('intersected_points', PointCloud2)
	# int_pub = rospy.Publisher('intersected_points_a', PointCloud)
	rospy.Subscriber('/cursor_click_marker', ImageClick, click_cb)
	rospy.spin()