import roslib; roslib.load_manifest('world_intersect')
import rospy
from brain_stuff.msg import Mouse
from geometry_msgs.msg import PointStamped

mouse_pub = None

def pt_cb(point):
	point.header.stamp = rospy.Time.now()
	mouse = Mouse(point, Mouse.NOTCLICK)
	mouse_pub.publish(mouse)

if __name__ == '__main__':
	rospy.init_node('point_to_click')
	rospy.Subscriber('intersected_point', PointStamped, pt_cb)
	mouse_pub = rospy.Publisher('/brain_interface/mouse_info', Mouse)
	rospy.spin()
