#!/usr/bin/env python
import roslib; roslib.load_manifest('projector_interface')
from tabletop_object_detector.srv import TabletopSegmentation, TabletopSegmentationRequest
from pr2_python.pointclouds import xyz_array_to_pointcloud2

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2

def pt2np(pt):
    return np.array([pt.x,pt.y,pt.z])

def cloud2np(cloud):
	return np.asarray([pt2np(pt) for pt in cloud.points])		

def location_from_cluster(cluster):
	return cloud2np(cluster).mean(axis=0)

def detect(detect_srv):
    detection = detect_srv.call()
    objects = detection.clusters
    header = rospy.Header()
    header.stamp = rospy.Time.now()
    frame_id = ''
    if objects:
        frame_id = objects[0].header.frame_id
    return xyz_array_to_pointcloud2(np.asarray([location_from_cluster(obj) for obj in objects]), stamp=rospy.Time.now(), frame_id=frame_id)

if __name__ == '__main__':
    rospy.init_node('find_objects')
    object_pub = rospy.Publisher('object_cloud', PointCloud2)
    detect_srv = rospy.ServiceProxy('/tabletop_segmentation', TabletopSegmentation)
    detect_srv.wait_for_service()
    while not rospy.is_shutdown():
        object_cloud = detect(detect_srv)
        object_pub.publish(object_cloud)
        print 'published'
