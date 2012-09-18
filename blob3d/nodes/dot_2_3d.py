#!/usr/bin/env python
import roslib; roslib.load_manifest('blob3d')
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2, Image
from projector_interface._point_cloud import read_points_np, create_cloud_np
import numpy as np
import cv2, cv

blobs = []
cloud = None
cloud_header = None
fields = None
bridge = None
point = None

def process(im):
    # convert to Luv
    imluv = cv2.cvtColor(im, cv.CV_RGB2Luv)
    # get channels
    l,u,v = cv2.split(imluv)
    r,g,b = cv2.split(im)
    minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(0.9*v + 0.1*g)
    # minG, maxG, minLocG, maxLocG = cv2.minMaxLoc(g)
    return maxLoc

def image_cb(msg):
    global point
    im = np.asarray(bridge.imgmsg_to_cv(msg, "passthrough"))
    point = process(im)

def cloud_cb(msg):
    global cloud, cloud_header, fields
    cloud = read_points_np(msg)
    cloud_header = msg.header
    fields = msg.fields

if __name__ == '__main__':
    rospy.init_node('dot_2_3d')
    bridge = CvBridge()
    rospy.Subscriber('image', Image, image_cb)
    rospy.Subscriber('cloud', PointCloud2, cloud_cb)
    cloud_pub = rospy.Publisher('dot_cloud', PointCloud2)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if (not point) or (cloud is None):
            continue
        blob_pts = []
        blob_pts.append(cloud[point[1],point[0]])
        
        out_cloud = create_cloud_np(cloud_header, fields, np.array(blob_pts))
        cloud_pub.publish(out_cloud)
        rate.sleep()