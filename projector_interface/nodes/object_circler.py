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

from threading import RLock

import PySide
from PySide import QtGui, QtCore
from PySide.QtGui import QPalette
import sys

class Circler(QtGui.QWidget):
	H = None
	objects = None	
	projected_objects = None
	int_objects = None	
	int_projected_objects = None
	int_ages = []
	model = None
	camera_frame = None
	object_header = None
	object_lock = RLock()
	intersected_lock = RLock()
	
	def escHandler(self, e):
		sys.exit(0)

	def initUI(self):
		#self.addKeyHandler(16777216, self.escHandler)
		p = QPalette()
		p.setColor(QPalette.Background, QtGui.QColor(0,0,0))
		self.setPalette(p)
		self.showFullScreen()


    # def homography_cb(self, msg):
    #     self.H = msg.mat.reshape(3,3)

	def info_cb(self, info):
		tmp_model = image_geometry.PinholeCameraModel()
		tmp_model.fromCameraInfo(info)
		self.model = tmp_model
		self.camera_stamp = info.header.stamp
		self.camera_frame = info.header.frame_id

	def object_cb(self, msg):
		with self.object_lock:
			objects = read_points_np(msg, masked=False)
			if objects.shape[1] == 0: return
			self.object_header = msg.header
			self.objects = objects
			transformed_objects = self.projectPoints(self.objects, msg.header)
			self.projected_objects = cv2.perspectiveTransform(transformed_objects, self.H)
		
	def intersected_cb(self, msg):
		#rospy.loginfo('internsected cb')
		with self.intersected_lock:
			objects = read_points_np(msg, masked=False)
			print objects.shape
			if objects.shape[1] == 0:
				print objects
				#self.int_objects = None 
				#self.int_projected_objects = None
				return

			self.int_object_header = msg.header
			self.int_objects = objects
			transformed_objects = self.projectPoints(objects, msg.header)
			self.int_projected_objects = cv2.perspectiveTransform(transformed_objects, self.H)
			for pt in transformed_objects[0]:
				self.int_ages.append(rospy.Time.now())

	def projectPoints(self, points, point_header):
		pts_out = []
		for point in points[0]:
			pt = PointStamped()
			pt.point.x, pt.point.y, pt.point.z = point.tolist()
			stamp = self.tfl.getLatestCommonTime(self.camera_frame, point_header.frame_id)
			pt.header = point_header
			pt.header.stamp = stamp
			# first, transform the point into the camera frame
			pt_out = self.tfl.transformPoint(self.camera_frame, pt).point
			# project it onto the image plane
			px = self.model.project3dToPixel((pt_out.x, pt_out.y, pt_out.z))
			pts_out.append(px)
		return np.array([pts_out])
			
	def paintEvent(self, e):
		if rospy.is_shutdown(): sys.exit()
		with self.object_lock:
			if self.objects is not None and self.projected_objects is not None:
				for xformed in self.projected_objects[0]:
					#print 'circle at', xformed
					r = 150
					qp = QtGui.QPainter()
					qp.begin(self)
					color = QtGui.QColor(255,255,255)
					qp.setPen(color)
					pen = qp.pen()
					pen.setWidth(5)
					qp.setPen(pen)
					rect = QtCore.QRectF(800-xformed[1]-r/2 + 25, 600-xformed[0]-r, r, r)
					#rect = QtCore.QRectF(800-xformed[1]-r/2 + 25, 600-xformed[0]-r/2, r, r)
					#rect = QtCore.QRectF(xformed[1]-r/2 + 25, xformed[0]-r/2, r, r)
					qp.drawArc(rect, 0, 360*16)
					qp.end()

		#print '======================'
		with self.intersected_lock:
			if self.int_objects is not None and self.int_projected_objects is not None:
				for (xformed, age) in zip(self.int_projected_objects[0], self.int_ages):
					#if (rospy.Time.now() - age).to_sec() > 5: continue
					#rospy.loginfo('intersected object at %s' % str(xformed))
					#print 'green circle at', xformed
					r = 100
					qp = QtGui.QPainter()
					qp.begin(self)
					color = QtGui.QColor(0,255,0)
					qp.setPen(color)
					pen = qp.pen()
					pen.setWidth(5)
					qp.setPen(pen)
					rect = QtCore.QRectF(800-xformed[1]-r/2 + 25, 600-xformed[0]-r, r, r)
					#rect = QtCore.QRectF(800-xformed[1]-r/2 + 25, 600-xformed[0]-r/2, r, r)
					#rect = QtCore.QRectF(xformed[1]-r/2 + 25, xformed[0]-r/2, r, r)
					qp.drawArc(rect, 0, 360*16)
					qp.end()

	def __init__(self):
		super(Circler, self).__init__()
        # rospy.Subscriber('homography', numpy_msg(Homography), self.homography_cb)
		self.tfl = tf.TransformListener()
		r = rospy.Rate(10)
		# while self.H is None:
		rospy.loginfo('waiting for homography...')
		while (not rospy.has_param('/homography')) and (not rospy.is_shutdown()):
			r.sleep()
		self.H = np.float64(rospy.get_param('/homography')).reshape(3,3)
		print self.H
		self.initUI()
		rospy.Subscriber('object_cloud', PointCloud2, self.object_cb)
		rospy.Subscriber('intersected_points', PointCloud2, self.intersected_cb)
		rospy.Subscriber('camera_info', CameraInfo, self.info_cb)
		timer = PySide.QtCore.QTimer(self)
		timer.setInterval(50)
		timer.timeout.connect(self.update)
		timer.start()
		sys.exit(app.exec_())
		rospy.spin()

if __name__ == '__main__':
	rospy.init_node('projector_demo')
	app = PySide.QtGui.QApplication(sys.argv)
	c = Circler()
