#!/usr/bin/env python
import roslib; roslib.load_manifest('projector_interface')
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
import rospy

click_inhibit = False

def uninhibit(args):
    global click_inhibit
    click_inhibit = False

def joy_cb(msg, args):
    global click_inhibit
    click = msg.buttons[16]
    if click and not click_inhibit:
        click_inhibit = True
        args[0].publish()
        rospy.Timer(rospy.Duration(1.0), uninhibit, oneshot=True)
		

if __name__ == '__main__':
    rospy.init_node('point_to_click')
    click_pub = rospy.Publisher('click', Empty)
    rospy.Subscriber('joy', Joy, joy_cb, [click_pub])
    rospy.spin()
