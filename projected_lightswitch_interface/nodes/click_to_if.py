#!/usr/bin/env python
import roslib; roslib.load_manifest('projected_lightswitch_interface')
import rospy
import tf
from geometry_msgs.msg import Point, PolygonStamped
from pr2_object_manipulation_msgs.msg import ImageClick
from projector_interface.srv import GetCursorStats, DrawPolygon, CircleInhibit
from copy import deepcopy
# tfl = None
poly_proxy = None

def click_cb(msg):
	poly = PolygonStamped()
	poly.header = msg.ray.header
	poly.polygon.points.append(msg.ray.direction)
	tmp_pt = msg.ray.direction
	tmp_pt = deepcopy(tmp_pt); tmp_pt.y += 0.1
	poly.polygon.points.append(tmp_pt)
	tmp_pt = deepcopy(tmp_pt); tmp_pt.z += 0.1
	poly.polygon.points.append(tmp_pt)
	tmp_pt = deepcopy(tmp_pt); tmp_pt.y -= 0.1
	poly.polygon.points.append(tmp_pt)
	rospy.loginfo("Sent polygon %s" % str(poly))
	polygon_proxy("Turn On", True, poly)

if __name__ == '__main__':
	rospy.init_node('click_to_if')
	polygon_proxy = rospy.ServiceProxy('/draw_polygon', DrawPolygon)
	rospy.loginfo("Waiting for polygon service")
	polygon_proxy.wait_for_service()
	rospy.loginfo("polygon service ready")
	rospy.Subscriber('/interactive_manipulation_image_click', ImageClick, click_cb)
	rospy.spin()