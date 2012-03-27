#!/usr/bin/env python
import roslib; roslib.load_manifest('projector_interface')
from projector_interface._point_cloud import create_cloud_np_xyz
from pr2_pick_and_place_demos.pick_and_place_manager import PickAndPlaceManager
import rospy
import numpy as np

def pt2Np(pt):
    return np.array([pt.x,pt.y,pt.z])

if __name__ == '__main__':
    # rospy.init_node('find_objects')
    manager = PickAndPlaceManager()
    detection = manager.call_tabletop_detection(update_table=1, clear_attached_objects=1)
    objects = detection[0]
    object_points = np.array([pt2Np(o.pose.pose.position) for o in objects])
    object_cloud = create_cloud_np_xyz(object_points, rospy.Header()) #TODO fill the header in
    print object_points