#!/usr/bin/env python
import roslib; roslib.load_manifest('world_intersect')
import rospy
from std_msgs.msg import Empty

from pymouse import PyMouse
import random, time
from signal import signal, SIGINT

def stop(signum, frame):
    cleanup_stop_thread();
    sys.exit()
    signal(SIGINT, stop)

from pymouse import PyMouseEvent

class event(PyMouseEvent):
    def __init__(self):
        super(event, self).__init__()
        self.click_pub = rospy.Publisher('click', Empty)

    def move(self, x, y):
        pass

    def click(self, x, y, button, press):
        if not press:
            self.click_pub.publish()

if __name__ == '__main__':
	rospy.init_node('mouse_click')
	e = event()
	e.capture = False
	e.daemon = False
	e.start()
	m = PyMouse()
	try:
	    while not rospy.is_shutdown():
	        rospy.sleep(1)
	except KeyboardInterrupt:
	    pass
	e.stop()
