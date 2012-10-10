#!/usr/bin/env python
import roslib; roslib.load_manifest('projected_lightswitch_interface')
import rospy
import tf
from geometry_msgs.msg import Point, PointStamped, PolygonStamped
from pr2_object_manipulation_msgs.msg import ImageClick
from projector_interface.srv import GetCursorStats, DrawPolygon, CircleInhibit
from copy import deepcopy
tfl = None
poly_proxy = None

SWITCH_WIDTH  = 0.08
SWITCH_HEIGHT = 0.125
BUTTON_WIDTH  = SWITCH_HEIGHT
BUTTON_HEIGHT = BUTTON_WIDTH

def rectangle(header, width, height, center):
	poly = PolygonStamped()
	poly.header = deepcopy(header)
	# poly.header.frame_id = '/table'
	# center_stamped = PointStamped(point=deepcopy(center), header=deepcopy(header))
	# center_stamped.header.stamp = rospy.Time(0)
	# center = tfl.transformPoint('table', center_stamped).point
	x,y,z = center.x,center.y,center.z

	poly.polygon.points.append(Point(x,y-width/2,z-height/2))
	poly.polygon.points.append(Point(x,y+width/2,z-height/2))
	poly.polygon.points.append(Point(x,y+width/2,z+height/2))
	poly.polygon.points.append(Point(x,y-width/2,z+height/2))

	return poly

def click_cb(msg):
	poly_switch = rectangle(msg.ray.header, SWITCH_WIDTH, SWITCH_HEIGHT, msg.ray.direction)
	on_center = deepcopy(msg.ray.direction)
	on_center.y += SWITCH_WIDTH/2 + BUTTON_WIDTH/2
	on_center.z += BUTTON_HEIGHT/2
	poly_on     = rectangle(msg.ray.header, BUTTON_WIDTH, BUTTON_HEIGHT, on_center)
	off_center = deepcopy(on_center)
	# off_center.y += SWITCH_WIDTH/2 + BUTTON_WIDTH/2
	off_center.z -= BUTTON_HEIGHT
	poly_off    = rectangle(msg.ray.header, BUTTON_WIDTH, BUTTON_HEIGHT, off_center)


	# rospy.loginfo("Sent polygon %s" % str(poly))
	polygon_proxy("Light\nSwitch", True, poly_switch)
	polygon_proxy("Turn On", True, poly_on)
	polygon_proxy("Turn Off", True, poly_off)
	rospy.loginfo("Sent polygons")


if __name__ == '__main__':
	rospy.init_node('click_to_if')
	tfl = tf.TransformListener()
	polygon_proxy = rospy.ServiceProxy('/draw_polygon', DrawPolygon)
	rospy.loginfo("Waiting for polygon service")
	polygon_proxy.wait_for_service()
	rospy.loginfo("polygon service ready")
	rospy.Subscriber('/interactive_manipulation_image_click', ImageClick, click_cb)
	rospy.spin()