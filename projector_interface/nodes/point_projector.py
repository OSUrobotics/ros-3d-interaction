#!/usr/bin/env python
import roslib; roslib.load_manifest('projector_interface')
import rospy
from sensor_msgs.msg import CameraInfo, 

info = None

def info_sub(msg):
	global info
 	info = msg

if __name__ == '__main__':
	rospy.init_node('point_projector')
	rospy.Subscriber('camera_info', CameraInfo, info_sub)
