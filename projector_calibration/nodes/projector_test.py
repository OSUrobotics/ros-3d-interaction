#!/usr/bin/env python
import roslib; roslib.load_manifest('projector_calibration')
import rospy
import cv2
from sensor_msgs.msg import RegionOfInterest, Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
bridge = CvBridge()
roi_pub = None
def img_cb(msg):
    cv2.imshow('win', np.asarray(bridge.imgmsg_to_cv(msg)))
    cv2.waitKey(10)

def click(s, x, y, click, u):
	if click:
		roi = RegionOfInterest()
		#r = 10
		roi.x_offset = x#max(x-r,0)
		roi.y_offset = y#max(y-r,0)
		#roi.width = 2*r
		#roi.height= 2*r
		roi_pub.publish(roi)
		print roi.x_offset, roi.y_offset

if __name__ == '__main__':
	cv2.namedWindow('win')
	cv2.setMouseCallback('win', click)
	rospy.init_node('projector_test')
	rospy.Subscriber('image', Image, img_cb)
	roi_pub = rospy.Publisher('roi', RegionOfInterest)
	rospy.spin()
	cv2.destroyAllWindows()
