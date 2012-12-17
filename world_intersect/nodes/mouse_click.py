#!/usr/bin/env python
import roslib; roslib.load_manifest('world_intersect')
import rospy
from std_msgs.msg import Empty

import Xlib
import Xlib.display
from Xlib.protocol import rq
from Xlib import X
from Xlib.ext import record

class MouseClick:
	def __init__(self):
		#disp = Xlib.display.Display()
		self.disp = Xlib.display.Display(':0')
		self.click_pub = rospy.Publisher('click', Empty)

	def mouse_cb(self, evt):
		data = evt.data
		while len(data):
			event, data = rq.EventField(None).parse_binary_value(data, self.disp.display, None, None)
			if event.type == X.ButtonRelease:
				self.click_pub.publish()	

	def start(self):
		ctx = self.disp.record_create_context(
			0,
			[record.AllClients],
			[{
					'core_requests': (0, 0),
					'core_replies': (0, 0),
					'ext_requests': (0, 0, 0, 0),
					'ext_replies': (0, 0, 0, 0),
					'delivered_events': (0, 0),
					'device_events': (X.ButtonPressMask, X.ButtonReleaseMask),
					'errors': (0, 0),
					'client_started': False,
					'client_died': False,
		}])
		root = self.disp.screen().root
		root.grab_pointer(True, X.ButtonPressMask | X.ButtonReleaseMask, X.GrabModeAsync, X.GrabModeAsync, 0, 0, X.CurrentTime)
		self.disp.record_enable_context(ctx, self.mouse_cb)

if __name__ == '__main__':
	rospy.init_node('mouse_click')
	mc = MouseClick()
	mc.start()
