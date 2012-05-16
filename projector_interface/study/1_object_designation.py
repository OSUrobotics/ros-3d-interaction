#!/usr/bin/env python
import roslib; roslib.load_manifest('projector_interface')
from std_msgs.msg import ColorRGBA
from pr2_python.pointclouds import pointcloud2_to_xyz_array, xyz_array_to_pointcloud2
import datetime
import scipy.io
import numpy as np
from itertools import chain
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

def sameObject(self, p1, p2):
    return self.dist(p1,p2) < SAME_OBJ_THRESH
    
order = [
    [0,  2,  1,  3],
    [7,  5,  6,  4],
    [8, 10,  9, 11],
    [0,  4,  8 ],
    [1,  5,  9 ],
    [2,  6,  10],
    [3,  7,  11]
]

targets = []
stds = []
times = []
cursor_history = []
history_size = 0

start_time = now()
for idx in chain(*order):
    point = points[idx]
    targets.append(point)
    print 'hilighting ', point
    services.clear_hilights()
    
    pt = PointStamped()
    pt.header.frame_id = '/table'
    pt.header.stamp = now()
    pt.point.x, pt.point.y, pt.point.z = point
    
    t1 = now()
    services.hilight_object(pt, ColorRGBA(255,0,0,0))
    # click_pts_msg = topics.click_stats[0]
    click_pts_msg = services.get_cursor_stats().points
    t2 = now()
    times.append((t2-t1).to_sec())
    click_pts = pointcloud2_to_xyz_array(click_pts_msg)
    history_size = click_pts.shape[0]
    cursor_history.extend(click_pts)
    stds.append(click_pts.std(0))
    print 'std = ', stds[-1]
    sleep(3)
        
services.clear_hilights()
path = '/home/robotics/lazewatskyd/ros-pkgs/wu-ros-pkg/3d_interaction/projector_interface/study/data/1_object_designation_%s.mat'
timestr = datetime.datetime.today().strftime('%d-%m-%y-%H.%M.%f')
scipy.io.savemat(path % timestr,    dict(targets=targets,
                                         stds=stds,
                                         times=times,
                                         cursor_history=cursor_history,
                                         history_size=history_size,
                                         total_time=(now()-start_time).to_sec()))
print 'Saved %s' % (path % timestr)