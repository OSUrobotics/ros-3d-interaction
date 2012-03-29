#!/usr/bin/env python
import roslib; roslib.load_manifest('projector_calibration')
from projector_calibration import CalibrationGrid
from projector_calibration.msg import Homography
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import PySide
import PySide.QtCore
import sys
from functools import partial
import numpy as np
bridge = CvBridge()

import cv, cv2

calib_im = None

class Calibrator(object):
	image = None
	def publish_image(self, grid_pub):
		im = self.grid.getPatternAsImage(im_type='OPENCV')

		msg = bridge.cv_to_imgmsg(self.removeAlpha(im), 'rgb8')
		msg.header.stamp = rospy.Time.now()
		grid_pub.publish(msg)

	def removeAlpha(self, im):
		r = cv.CreateMat(im.height, im.width, cv.CV_8UC1)
		g = cv.CreateMat(im.height, im.width, cv.CV_8UC1)
		b = cv.CreateMat(im.height, im.width, cv.CV_8UC1)
		cv.Split(im,r,g,b,None)
		imrgb = cv.CreateMat(im.height, im.width, cv.CV_8UC3)
		cv.Merge(r,g,b,None,imrgb)
		return imrgb

	def calibrate(self, *args):
		print 'calibrate'
		# TODO wait until we can detect a checkerboard
		while not self.image and not rospy.is_shutdown():
			rospy.sleep(0.1)
			print 'waiting'
		print 'ready'
		im1 = bridge.imgmsg_to_cv(self.image)
		corners1 = cv.FindChessboardCorners(im1, (self.grid.nCols-1, self.grid.nRows-1))
		corners2 = np.array(self.grid.corners, dtype=np.float32)

		from pprint import pprint
		H, mask = cv2.findHomography(np.array(corners1[1], dtype=np.float32), corners2, method=cv2.RANSAC)
		pprint(H)
		self.homography_pub.publish(H.flatten())

		c1 = np.array([corners1[1]])
		c2 = np.array([corners2])
		err = cv2.perspectiveTransform(c1,H) - c2
		#f = cv.CreateMat(3, 3, cv.CV_32FC1)
		#cv.FindFundamentalMat(cv.fromarray(np.array(corners1[1])), cv.fromarray(np.array(corners2[1])), f)
		self.grid.hide()
		
	def image_cb(self, msg):
		self.image = msg

	def maybe_shutdown(self):
		if rospy.is_shutdown(): sys.exit(0)
	
	def __init__(self):
		rospy.init_node('grid')
		rows, cols = [int(n) for n in rospy.get_param('~grid_size', default='5x7').split('x')]
		print rows, cols
		rospy.Subscriber('image', Image, self.image_cb)
		grid_pub = rospy.Publisher('grid', Image)
		self.homography_pub = rospy.Publisher('homography', numpy_msg(Homography), latch=True)
		app = PySide.QtGui.QApplication(sys.argv)
		self.grid = CalibrationGrid(nCols=cols, nRows=rows)
		self.grid.addKeyHandler(67, self.calibrate)
		self.grid.showFullScreen()
		qtimer = PySide.QtCore.QTimer(self.grid)
		qtimer.timeout.connect(self.maybe_shutdown)
		qtimer.start()
		PySide.QtCore.QTimer.singleShot(1000, self.calibrate)
		#self.grid.setCursor(PySide.QtGui.QCursor(PySide.QtCore.Qt.BlankCursor))
		#timer = PySide.QtCore.QTimer(self.grid)
		#self.grid.connect(timer, PySide.QtCore.SIGNAL('timeout()'), partial(self.publish_image, grid_pub))
		#timer.start(100)
		sys.exit(app.exec_())

if __name__ == '__main__':
	Calibrator()
