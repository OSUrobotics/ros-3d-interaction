#!/usr/bin/env python
import roslib; roslib.load_manifest('world_intersect')
import rospy
from brain_stuff.msg import Mouse
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Joy

mouse_pub = None
click = False

def pt_cb(point):
	point.header.stamp = rospy.Time.now()
	mouse = Mouse(point, Mouse.NOTCLICK if not click else Mouse.CLICK)
	mouse_pub.publish(mouse)

def undo_click():
	click = False

def joy_cb(msg):
	global click
	click = msg.buttons[0]
	print click

if __name__ == '__main__':
	rospy.init_node('point_to_click')
	rospy.Subscriber('intersected_point', PointStamped, pt_cb)
	rospy.Subscriber('joy', Joy, joy_cb)
	mouse_pub = rospy.Publisher('/brain_interface/mouse_info', Mouse)
	rospy.spin()
