#!/usr/bin/env python
import roslib; roslib.load_manifest('blob3d')
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2, Image, CameraInfo, PointField
from projector_interface._point_cloud import read_points_np, create_cloud_np
import image_geometry
import numpy as np
import cv2, cv

blobs = []
depth = None
depth_header = None
bridge = None
point = None
model = None

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

def depth_cb(msg):
    global depth, depth_header
    depth = np.asarray(bridge.imgmsg_to_cv(msg, "passthrough"))
    depth_header = msg.header
    
def info_cb(msg):
    global model
    tmp_model = image_geometry.PinholeCameraModel()
    tmp_model.fromCameraInfo(msg)
    model = tmp_model

if __name__ == '__main__':
    rospy.init_node('dot_2_3d')
    bridge = CvBridge()
    rospy.Subscriber('image', Image, image_cb)
    rospy.Subscriber('depth_image', Image, depth_cb)
    rospy.Subscriber('camera_info', CameraInfo, info_cb)
    cloud_pub = rospy.Publisher('dot_cloud', PointCloud2)
    
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if (not point) or (depth is None) or (model is None):
            continue
        blob_pts = []
        point3 = np.array(model.projectPixelTo3dRay(point)) * depth[point[1],point[0]]
        blob_pts.append(point3)
        
        out_cloud = create_cloud_np(depth_header, fields, np.array(blob_pts))
        cloud_pub.publish(out_cloud)
        rate.sleep()