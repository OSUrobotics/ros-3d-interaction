#!/usr/bin/env python
import roslib; roslib.load_manifest('projector_calibration')
from projector_calibration.msg import Homography
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import RegionOfInterest
import cv2
import numpy as np

import PySide
from PySide import QtGui, QtCore
from PySide.QtGui import QPalette
import sys
from math import hypot

class Circler(QtGui.QWidget):
	H = None
	roi = None	

	def escHandler(self, e):
		sys.exit(0)

	def initUI(self):
		#self.addKeyHandler(16777216, self.escHandler)
		p = QPalette()
		p.setColor(QPalette.Background, QtGui.QColor(0,0,0))
		self.setPalette(p)
		self.showFullScreen()


	def homography_cb(self, msg):
		self.H = msg.mat.reshape(3,3)

	def roi_cb(self, msg):
		self.roi = msg

	def paintEvent(self, e):
		if rospy.is_shutdown(): sys.exit()
		if self.roi is not None:
			pts = np.float64([[(self.roi.x_offset, self.roi.y_offset)]])
			xformed = cv2.perspectiveTransform(pts, self.H)
			r = 150
			qp = QtGui.QPainter()
			qp.begin(self)
			color = QtGui.QColor(255,255,255)
			qp.setPen(color)
			pen = qp.pen()
			pen.setWidth(5)
			qp.setPen(pen)
            # from pprint import pprint as pp
            # pp(dir(qp))
            # sys.exit(0)
			rect = QtCore.QRectF(xformed[:,:,1]-r/2, xformed[:,:,0]-r/2, r, r)
			qp.drawArc(rect, 0, 360*16)
			qp.end()


	def __init__(self):
		super(Circler, self).__init__()
		rospy.Subscriber('homography', numpy_msg(Homography), self.homography_cb)
		r = rospy.Rate(10)
		while self.H is None:
			rospy.loginfo('waiting for homography...')
			r.sleep()
		print self.H
		self.initUI()
		rospy.Subscriber('roi', RegionOfInterest, self.roi_cb)
		timer = PySide.QtCore.QTimer(self)
		#self.connect(timer, PySide.QtCore.SIGNAL('timeout()'), partial(self.publish_image, grid_pub))
		timer.timeout.connect(self.update)
		timer.start()
		sys.exit(app.exec_())
		rospy.spin()

if __name__ == '__main__':
	rospy.init_node('projector_demo')
	app = PySide.QtGui.QApplication(sys.argv)
	c = Circler()
