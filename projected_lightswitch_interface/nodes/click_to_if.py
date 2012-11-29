#!/usr/bin/env python
import roslib; roslib.load_manifest('projected_lightswitch_interface')
import rospy
import tf
from geometry_msgs.msg import Point, PointStamped, PolygonStamped
from pr2_object_manipulation_msgs.msg import ImageClick
from std_msgs.msg import ColorRGBA, String
from projector_interface.srv import GetCursorStats, DrawPolygon, CircleInhibit
from copy import deepcopy


polygon_viz = None


tfl = None
poly_proxy = None
hilight_proxy = None
clear_hilight_proxy = None
inited = False

SWITCH_WIDTH  = 0.08
SWITCH_HEIGHT = 0.125
BUTTON_WIDTH  = SWITCH_HEIGHT
BUTTON_HEIGHT = BUTTON_WIDTH

WHITE  = ColorRGBA(255,255,255,0)
PURPLE = ColorRGBA(255,0  ,255,0)
#HILIGHT_COLOR = PURPLE
HILIGHT_COLOR = WHITE

poly_switch = None
poly_on = None
poly_off = None


polygons = dict()

def rectangle(header, width, height, center):
	poly = PolygonStamped()
	poly.header = deepcopy(header)
	# poly.header.frame_id = '/table'
	# center_stamped = PointStamped(point=deepcopy(center), header=deepcopy(header))
	# center_stamped.header.stamp = rospy.Time(0)
	# center = tfl.transformPoint('table', center_stamped).point
	x,y,z = center.x,center.y,center.z

	# this is for doing it in image coordinates
	# poly.polygon.points.append(Point(x,y-width/2,z-height/2))
	# poly.polygon.points.append(Point(x,y+width/2,z-height/2))
	# poly.polygon.points.append(Point(x,y+width/2,z+height/2))
	# poly.polygon.points.append(Point(x,y-width/2,z+height/2))

	# normal frame alignment
	poly.polygon.points.append(Point(x-width/2,y-height/2,z))
	poly.polygon.points.append(Point(x+width/2,y-height/2,z))
	poly.polygon.points.append(Point(x+width/2,y+height/2,z))
	poly.polygon.points.append(Point(x-width/2,y+height/2,z))


	return poly

def click_cb(msg):
	global poly_switch, poly_on, poly_off
	poly_switch = rectangle(msg.ray.header, SWITCH_WIDTH, SWITCH_HEIGHT, msg.ray.direction)
	on_center   = deepcopy(msg.ray.direction)
	on_center.y += SWITCH_WIDTH/2 + BUTTON_WIDTH/2
	on_center.z += BUTTON_HEIGHT/2
	poly_on     = rectangle(msg.ray.header, BUTTON_WIDTH, BUTTON_HEIGHT, on_center)
	off_center = deepcopy(on_center)
	# off_center.y += SWITCH_WIDTH/2 + BUTTON_WIDTH/2
	off_center.z -= BUTTON_HEIGHT
	poly_off    = rectangle(msg.ray.header, BUTTON_WIDTH, BUTTON_HEIGHT, off_center)

	global polygons
	polygons["Light\nSwitch"] = poly_switch
	polygons["Turn On"]       = poly_on
	polygons["Turn Off"]      = poly_off

	# rospy.loginfo("Sent polygon %s" % str(poly))
	polygon_proxy("Light\nSwitch", True, poly_switch, WHITE)
	print 'published viz'
	# import pdb; pdb.set_trace()
	polygon_viz.publish(poly_switch)

	print poly_switch
	if inited:
		polygon_proxy("Turn On", True, poly_on, WHITE)
		polygon_proxy("Turn Off", True, poly_off, WHITE)
	rospy.loginfo("Sent polygons")

def if_click_cb(msg):
	rospy.loginfo('if click cb')
	global inited
	polygon_proxy(msg.data, True, polygons[msg.data], HILIGHT_COLOR)
	print msg.data
	if msg.data == "Light\nSwitch":
		inited = True
		polygon_proxy("Turn On", True, poly_on, WHITE)
		polygon_proxy("Turn Off", True, poly_off, WHITE)

def outline_switch():
	fake_click = ImageClick()
	fake_click.ray.header.frame_id = '/4x4_0'
	fake_click.ray.header.stamp = rospy.Time.now()
	fake_click.ray.direction.x = -0.11
	fake_click.ray.direction.y =  0.0
	fake_click.ray.direction.z =  0.0
	click_cb(fake_click)

if __name__ == '__main__':
	rospy.init_node('click_to_if')
	tfl = tf.TransformListener()
	polygon_proxy = rospy.ServiceProxy('/draw_polygon', DrawPolygon)
	rospy.loginfo("Waiting for polygon service")
	polygon_proxy.wait_for_service()
	rospy.loginfo("polygon service ready")

	polygon_viz = rospy.Publisher('/polygon_viz', PolygonStamped)
	
	# rospy.loginfo("Waiting for hilight service")
	# hilight_proxy = rospy.ServiceProxy("hilight_object")
	# rospy.loginfo("hilight service ready")
	# rospy.loginfo("waiting for clear hilight service")
	# clearhilight_proxy = rospy.ServiceProxy("clear_hilights")
	# rospy.loginfo("clear hilight service ready")
	outline_switch()
	# rospy.Subscriber('/interactive_manipulation_image_click', ImageClick, click_cb)
	# rospy.Subscriber('/clicked_object', String, if_click_cb)
	rospy.spin()