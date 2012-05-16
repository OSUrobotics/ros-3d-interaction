#!/usr/bin/env rosh
import roslib; roslib.load_manifest('projector_interface')
from pr2_python.geometry_tools import *
import numpy as np

for pose in topics.head_pose[:]:
    phi,theta,psi = quaternion_to_euler(pose.pose.orientation)
    theta += np.radians(5)
    psi   += np.radians(1.5)
    pose.pose.orientation = euler_to_quaternion(phi,theta,psi)
    topics.head_pose_adjusted(pose)