#!/usr/bin/env python
import roslib; roslib.load_manifest('projector_interface')
from projector_interface._point_cloud import create_cloud_np_xyz
from pr2_pick_and_place_demos.pick_and_place_manager import PickAndPlaceManager
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2

def pt2Np(pt):
    return np.array([pt.x,pt.y,pt.z])

def detect(manager):
    detection = manager.call_tabletop_detection(update_table=1, clear_attached_objects=1)
    objects = detection[0]
    header = rospy.Header()
    header.stamp = rospy.Time.now()
    if objects:
        header.frame_id = objects[0].pose.header.frame_id
    object_points = np.array([pt2Np(o.pose.pose.position) for o in objects])
    object_cloud = create_cloud_np_xyz(np.float32(object_points), header)
    return object_cloud
    

if __name__ == '__main__':
    rospy.init_node('find_objects')
    manager = PickAndPlaceManager()
    # rate = rospy.Rate(5)
    object_pub = rospy.Publisher('object_cloud', PointCloud2)
    while not rospy.is_shutdown():
        object_cloud = detect(manager)
        # print object_cloud
        object_pub.publish(object_cloud)
        print 'published'
        # rate.sleep()