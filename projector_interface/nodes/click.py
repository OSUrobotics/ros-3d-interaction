#!/usr/bin/env python
import roslib; roslib.load_manifest('projector_interface')
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
import rospy

new_click = True

def joy_cb(msg, args):
    global new_click
    click = msg.buttons[2]
    if click and new_click:
        new_click = False
        args[0].publish()
    elif not click:
        new_click = True

if __name__ == '__main__':
    rospy.init_node('point_to_click')
    click_pub = rospy.Publisher('click', Empty)
    rospy.Subscriber('joy', Joy, joy_cb, [click_pub])
    rospy.spin()
