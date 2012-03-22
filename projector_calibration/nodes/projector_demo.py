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

class Circler(QtGui.QWidget):
	H = None
	roi = None	

	def escHandler(self, e):
		sys.exit(0)

	def initUI(self):
		#self.addKeyHandler(16777216, self.escHandler)
		p = QPalette()
		p.setColor(QPalette.Background, QtGui.QColor(255,255,255))
		self.setPalette(p)
		self.showFullScreen()


	def homography_cb(self, msg):
		self.H = msg.mat.reshape(3,3)

	def roi_cb(self, msg):
		self.roi = msg

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
		sys.exit(app.exec_())
		rospy.spin()

if __name__ == '__main__':
	rospy.init_node('projector_demo')
	app = PySide.QtGui.QApplication(sys.argv)
	c = Circler()
