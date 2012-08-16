#!/usr/bin/env python
import roslib; roslib.load_manifest('projector_interface')
import rospy
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Point
import image_geometry
from pr2_python import transform_listener

import cv2
import numpy as np
import sys
import tf
from cv_bridge import CvBridge, CvBridgeError
from pr2_python.head import Head

import PySide
from PySide import QtGui, QtCore
from PySide.QtGui import QPalette
from PySide import QtOpenGL


class Gradient(QtGui.QWidget):
    image = None
    model = None
    	
    def keyPressEvent(self, e):
	if 16777216 == e.key():
	    sys.exit(0)

    def initUI(self):
	self.showFullScreen()
	
    def paintEvent(self, e):
	p = QPalette()
	print self.width(), self.height()
	gradient = QtGui.QRadialGradient(self.width()/2, self.height()/2, min(self.width(), self.height()))
	gradient.setColorAt(0.7, QtGui.QColor(0,0,0))
	gradient.setColorAt(0.0, QtGui.QColor(0,255,0))
        p.setBrush(QPalette.Window, QtGui.QBrush(gradient))
        self.setPalette(p)
	
    def info_cb(self, info):
        tmp_model = image_geometry.PinholeCameraModel()
        tmp_model.fromCameraInfo(info)
        self.model = tmp_model
    
    def image_cb(self, msg):
	self.image = msg
	
    def center(self, *args):
	while (self.image is None or self.model is None) and not rospy.is_shutdown():
	    rospy.sleep(0.1)

	if rospy.is_shutdown(): sys.exit(0)

	im = np.asarray(self.bridge.imgmsg_to_cv(self.image))
	dists = np.sum((im - [0,255,0])**2, axis=2)
	px = np.unravel_index(np.argmin(dists.flatten()), dists.shape)
	target_pt = self.model.projectPixelTo3dRay((px[1],px[0]))
	print 'pointing at %s' % str(target_pt)
	pt = transform_listener.transform_point('base_footprint', self.model.tf_frame, Point(*target_pt))
	self.head.look_at_relative_point(pt.x,pt.y,pt.z)
	self.center()
    
    def __init__(self):
        super(Gradient, self).__init__()
	self.head = Head()
	self.head.pointing_frame = '/narrow_stereo_optical_frame'
	self.bridge = CvBridge()
        self.initUI()
        rospy.Subscriber('camera_info', CameraInfo, self.info_cb)
	rospy.Subscriber('image', Image, self.image_cb)
	
	PySide.QtCore.QTimer.singleShot(1000, self.center)
	
        sys.exit(app.exec_())
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('gradient')
    app = PySide.QtGui.QApplication(sys.argv)
    g = Gradient()
