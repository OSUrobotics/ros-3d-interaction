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

def sameObject(self, p1, p2):
    return self.dist(p1,p2) < SAME_OBJ_THRESH
    
order = [
    0,  2,  1,  3,
    # 7,  5,  6,  4,
    # 8, 10,  9, 11,
    # 0,  4,  8,
    # 1,  5,  9,
    # 2,  6,  10,
    # 3,  7,  11
]

targets = []
stds = []

# for point in points:
for idx in order:
    point = points[idx]
    targets.append(point)
    print 'hilighting ', point
    services.clear_hilights()
    
    pt = PointStamped()
    pt.header.frame_id = '/table'
    pt.header.stamp = now()
    pt.point.x, pt.point.y, pt.point.z = point
    
    services.hilight_object(pt, ColorRGBA(255,0,0,0))
    # click_pts_msg = topics.click_stats[0]
    click_pts_msg = services.get_cursor_stats().points
    click_pts = pointcloud2_to_xyz_array(click_pts_msg)
    stds.append(click_pts.std(0))
    print 'std = ', stds[-1]
    sleep(3)
        
services.clear_hilights()
path = '/home/robotics/lazewatskyd/ros-pkgs/wu-ros-pkg/3d_interaction/projector_interface/study/data/user_%s.mat'
timestr = datetime.datetime.today().isoformat()
scipy.io.savemat(path % timestr, dict(targets=targets, stds=stds))
print 'Saved %s' % (path % timestr)