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
		self.camera_stamp = info.header.stamp
		self.camera_frame = info.header.frame_id

	def object_cb(self, msg):
		objects = read_points_np(msg, masked=False)
		self.object_header = msg.header
		self.objects = objects
		transformed_objects = self.projectPoints(self.objects)
		# print 'projected   ==>\n', transformed_objects
		print transformed_objects
		print self.H.shape
		self.projected_objects = cv2.perspectiveTransform(transformed_objects, self.H)
		print '=======object_cb======='
		# print 'transformed ==>\n', self.projected_objects
		# print self.objects.shape
		# print self.objects
		# print self.projected_objects
		

	def projectPoints(self, points):
		pts_out = []
		print 'points.shape is', points.shape
		for point in points[0]:
			# print '-->', point
			pt = PointStamped()
			pt.point.x, pt.point.y, pt.point.z = point.tolist()
			stamp = self.tfl.getLatestCommonTime(self.camera_frame, self.object_header.frame_id)
			pt.header = self.object_header
			pt.header.stamp = stamp
			# first, transform the point into the camera frame
			pt_out = self.tfl.transformPoint(self.camera_frame, pt).point
			# print self.tfl.transformPoint(self.camera_frame, pt)
			# project it onto the image plane
			px = self.model.project3dToPixel((pt_out.x, pt_out.y, pt_out.z))
			pts_out.append(px)
		return np.array([pts_out])
			
	def paintEvent(self, e):
		if rospy.is_shutdown(): sys.exit()
        # print 'there are', len(self.objects), 'objects'
		#print '-------------------'
		if self.objects is not None and len(self.objects) > 0:
			for xformed in self.projected_objects[0]:
				# pts = np.float64([[(self.roi.x_offset, self.roi.y_offset)]])
				# xformed = cv2.perspectiveTransform(pts, self.H)
				# print xformed
				# sys.exit(0)
				print 'circle at', xformed
				r = 150
				qp = QtGui.QPainter()
				qp.begin(self)
				color = QtGui.QColor(255,255,255)
				qp.setPen(color)
				pen = qp.pen()
				pen.setWidth(5)
				qp.setPen(pen)
				rect = QtCore.QRectF(800-xformed[1]-r/2, 600-xformed[0]-r/2, r, r)
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
