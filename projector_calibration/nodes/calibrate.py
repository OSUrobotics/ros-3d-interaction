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
	fail_count = 0
	grid_corners = np.zeros((0,2))
	img_corners  = np.zeros((0,2))
	
	def detect(self):
		try:
			im1 = bridge.imgmsg_to_cv(self.image)
		except AttributeError, e:
			rospy.logwarn("Tried to convert a None image")
			return False
		corners1 = cv.FindChessboardCorners(im1, (self.grid.nCols-1, self.grid.nRows-1))
		corners2 = np.array(self.grid.corners, dtype=np.float32)

		if np.asarray(corners1[1]).shape == corners2.shape:
			self.img_corners  = np.vstack((self.img_corners,  np.float32(corners1[1])))
			self.grid_corners = np.vstack((self.grid_corners, corners2))
			return True
		return False
	
	def find_homography(self):
		if len(self.img_corners) == len(self.grid_corners):
			from pprint import pprint
			H, mask = cv2.findHomography(self.img_corners, self.grid_corners, method=cv2.RANSAC)
			pprint(H)
			rospy.set_param('/homography', H.flatten().tolist())
	        # self.homography_pub.publish(H.flatten())

	        # c1 = np.array([corners1[1]])
	        # c2 = np.array([corners2])
	        # err = cv2.perspectiveTransform(c1,H) - c2
			sys.exit(0)
			return True
		return False
		
	
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

	def calibrate(self, keep_going=False, retry=True):
		print 'waiting...'
		while not self.image and not rospy.is_shutdown():
			rospy.sleep(0.1)
		print 'ready'
		if self.detect():
			if not keep_going:
				sys.exit(self.find_homography())
		if retry or keep_going:
			if rospy.is_shutdown(): return
			rospy.logwarn('Wrong number of corners. Is there something obstructing the calibration grid?')
			self.fail_count += 1
			if self.fail_count > 3:
				self.grid.scale = 0.5
				
				if len(self.origins) > 0:
					if self.fail_count % 4:
						self.grid.origin = self.origins.pop()
						self.grid.repaint()
						self.grid.repaint()
						self.grid.repaint()
						self.grid.repaint()
						rospy.sleep(2)
					self.calibrate(keep_going=(len(self.origins)>0), retry=False)
				else:
					self.grid.origin = None
					self.grid.scale = 1.0
					self.grid.repaint()
			self.calibrate()
		# sys.exit(self.find_homography())
		self.find_homography()
		
	def image_cb(self, msg):
		self.image = msg

	def maybe_shutdown(self):
		if rospy.is_shutdown():
			sys.exit(0)
	
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
		self.origins = [
			(self.grid.width(),self.grid.padding),
			(self.grid.width(),self.grid.height()-self.grid.padding),
			(self.grid.padding,self.grid.height()-self.grid.padding),
			(self.grid.padding,self.grid.padding)
		]
		
		PySide.QtCore.QTimer.singleShot(1000, self.calibrate)
		timer = PySide.QtCore.QTimer(self.grid)
		timer.timeout.connect(self.grid.update)
		timer.start(100)
		sys.exit(app.exec_())

if __name__ == '__main__':
	Calibrator()
