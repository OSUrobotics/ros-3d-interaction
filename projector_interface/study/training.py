#!/usr/bin/env python
import roslib; roslib.load_manifest('projector_interface')
from std_msgs.msg import ColorRGBA
from pr2_python.pointclouds import pointcloud2_to_xyz_array, xyz_array_to_pointcloud2
import datetime
import scipy.io
import numpy as np

load('rosh_robot', globals())

xmin = -0.79651666
xmax =  0.06501853
ymin =  0.03261997
ymax =  0.50817382

# xs = np.linspace(xmin,xmax,4)
ys = np.linspace(ymin,ymax,3)
xs = np.arange(xmin+0.05, xmax, ys[1] - ys[0])
xx, yy = np.meshgrid(xs, ys)
points = np.array([(x,y,0) for x,y in zip(xx.ravel(), yy.ravel())])
points_msg = xyz_array_to_pointcloud2(points, now(), '/table')
topics.object_cloud(points_msg)
services.set_selection_method(0)
services.clear_hilights()

# points = np.array([[xmin+(xmax-xmin)/2, ymin+(ymax-ymin)/2, 0]])
# points_msg = xyz_array_to_pointcloud2(points, now(), '/table')
# topics.object_cloud(points_msg)

def shutdown(event):
    pt = PointStamped()
    pt.header.frame_id = '/table'
    pt.header.stamp = now()
    pt.point.x, pt.point.y, pt.point.z = points[0]
    
    services.hilight_object(pt, ColorRGBA(0,0,0,0))
    rospy.signal_shutdown(0)
    
rospy.Timer(rospy.Duration(120), shutdown, oneshot=True)

rospy.spin()