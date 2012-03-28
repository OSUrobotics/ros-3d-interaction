#!/usr/bin/env python
import roslib; roslib.load_manifest('projector_interface')
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
import rospy

def joy_cb(msg, args):
    click = msg.buttons[0]
    if click:
        args[0].publish()

if __name__ == '__main__':
    rospy.init_node('point_to_click')
    click_pub = rospy.Publisher('click', Empty)
    rospy.Subscriber('joy', Joy, joy_cb, [click_pub])
    rospy.spin()