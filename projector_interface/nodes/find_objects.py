#!/usr/bin/env python
import roslib; roslib.load_manifest('projector_interface')
from tabletop_object_detector.srv import TabletopSegmentation, TabletopSegmentationRequest
from pr2_python.pointclouds import xyz_array_to_pointcloud2
from pr2_python.transform_listener import transform_point_stamped
from tf import TransformBroadcaster
import tf_conversions.posemath as pm
from threading import Thread, RLock

import rospy
import tf
import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped

table_pose = None
br = None
tf_lock = RLock()

def pt2np(pt):
    return np.array([pt.x,pt.y,pt.z])

def cloud2np(cloud):
	return np.asarray([pt2np(pt) for pt in cloud.points])		

def location_from_cluster(cluster):
    pt = cloud2np(cluster).mean(axis=0)
    pt_stamped = PointStamped()
    pt_stamped.header = cluster.header
    pt_stamped.point.x, pt_stamped.point.y, pt_stamped.point.z = pt
    
    try:
        ptt = transform_point_stamped('/table', pt_stamped)
        return np.array([ptt.point.x,ptt.point.y,0])
    except tf.Exception as e:
        print 'exception...'
        return np.array([np.nan]*3)
    # return cloud2np(cluster).mean(axis=0)

def detect(detect_srv):
    detection = detect_srv.call()
    objects = detection.clusters
    header = rospy.Header()
    header.stamp = rospy.Time.now()
    if objects:
        frame_id = objects[0].header.frame_id
        # cloud = xyz_array_to_pointcloud2(np.asarray([location_from_cluster(obj) for obj in objects]), stamp=rospy.Time.now(), frame_id=frame_id)
        cloud = xyz_array_to_pointcloud2(np.asarray([location_from_cluster(obj) for obj in objects]), stamp=rospy.Time.now(), frame_id='/table')
        return cloud, detection.table.pose
    return None, None

# def broadcast_table_frame(table_pose, br, stamp=None):
def broadcast_table_frame(args):
    if table_pose is None: return
    with tf_lock:
        trans, rot = pm.toTf(pm.fromMsg(table_pose.pose))
        # if stamp is None: stamp = rospy.Time.now()
        br.sendTransform(trans, rot, rospy.Time.now(), '/table', table_pose.header.frame_id)

if __name__ == '__main__':
    rospy.init_node('find_objects')
    object_pub = rospy.Publisher('object_cloud', PointCloud2)
    br = TransformBroadcaster()	
    detect_srv = rospy.ServiceProxy('/tabletop_segmentation', TabletopSegmentation)
    detect_srv.wait_for_service()
    print 'starting timer'
    Thread(target=rospy.Timer(rospy.Duration(0.05), broadcast_table_frame).run)
    print 'started timer'
    while not rospy.is_shutdown():
        object_cloud, table_pose_tmp = detect(detect_srv)
        if object_cloud is not None:
            with tf_lock:
                table_pose = table_pose_tmp
            # broadcast_table_frame(table_pose, br, stamp=object_cloud.header.stamp)
            object_pub.publish(object_cloud)
