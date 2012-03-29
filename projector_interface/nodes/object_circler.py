#!/usr/bin/env python
import roslib; roslib.load_manifest('projector_interface')
from projector_calibration.msg import Homography
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import CameraInfo, PointCloud2
from geometry_msgs.msg import PointStamped
import image_geometry
from projector_interface._point_cloud import read_points_np
import cv2
import numpy as np
import tf

import PySide
from PySide import QtGui, QtCore
from PySide.QtGui import QPalette
import sys

class Circler(QtGui.QWidget):
	H = None
	objects = None	
	model = None
	camera_frame = None
	object_header = None
	
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

	def info_cb(self, info):
		tmp_model = image_geometry.PinholeCameraModel()
		tmp_model.fromCameraInfo(info)
		self.model = tmp_model
		self.camera_frame = info.header.frame_id

	def object_cb(self, msg):
		self.objects = read_points_np(msg, masked=False)
		self.object_header = msg.header

	def projectPoints(points):
		pts_out = []
		for point in points:
			pt = PointStamped()
			pt.header = self.object_header
			pt_out = self.tfl.transformPoint(self.camera_frame, pt).point
			pts_out.append(self.model.project3dToPixel(pt_out.x, pt_out.y, pt_out.z))
		return np.array(pts_out)
			

	def paintEvent(self, e):
		if rospy.is_shutdown(): sys.exit()
		
		if self.objects is not None:
			transformed_objects = self.transformPoints(self.objects)
			projected_objects = cv2.perspectiveTransform(transformed_objects, self.H)
			for xformed in projected_objects:				 
				# pts = np.float64([[(self.roi.x_offset, self.roi.y_offset)]])
				xformed = cv2.perspectiveTransform(pts, self.H)
				r = 150
				qp = QtGui.QPainter()
				qp.begin(self)
				color = QtGui.QColor(255,255,255)
				qp.setPen(color)
				pen = qp.pen()
				pen.setWidth(5)
				qp.setPen(pen)
				rect = QtCore.QRectF(xformed[:,:,1]-r/2, xformed[:,:,0]-r/2, r, r)
				qp.drawArc(rect, 0, 360*16)
				qp.end()

	def __init__(self):
		super(Circler, self).__init__()
		rospy.Subscriber('homography', numpy_msg(Homography), self.homography_cb)
		self.tfl = tf.TransformListener()
		r = rospy.Rate(10)
		while self.H is None:
			rospy.loginfo('waiting for homography...')
			r.sleep()
		print self.H
		self.initUI()
		rospy.Subscriber('object_cloud', PointCloud2, self.object_cb)
		rospy.Subscriber('camera_info', CameraInfo, self.info_cb)
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
