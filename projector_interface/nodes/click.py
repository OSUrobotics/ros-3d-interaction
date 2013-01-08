#!/usr/bin/env python
import roslib; roslib.load_manifest('projector_interface')
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
import rospy

click_inhibit = False
last_val = 0

def uninhibit(args):
    global click_inhibit
    click_inhibit = False

def joy_cb(msg, args):
    global click_inhibit
    global last_val

    click = msg.buttons[2]
    if click and (last_val == 0) and not click_inhibit:
        click_inhibit = True
        args[0].publish()
        rospy.Timer(rospy.Duration(1.0), uninhibit, oneshot=True)
        last_val = 1
    elif click:
        pass
    else:
        last_val = 0
		

if __name__ == '__main__':
    rospy.init_node('point_to_click')
    click_pub = rospy.Publisher('click', Empty)
    rospy.Subscriber('joy', Joy, joy_cb, [click_pub])
    rospy.spin()
