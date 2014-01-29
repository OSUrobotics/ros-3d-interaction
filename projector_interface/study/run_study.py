#!/usr/bin/env python
import roslib; roslib.load_manifest('projector_interface')
from std_msgs.msg import ColorRGBA
from pr2_python.pointclouds import pointcloud2_to_xyz_array, xyz_array_to_pointcloud2
import datetime
import scipy.io
import os
import numpy as np
from itertools import chain

from point_tests import sameObject

load('rosh_robot', globals())

while not rospy.has_param('/screen/height'):
    rospy.sleep(0.1)

data_dir = rospy.get_param('/study_data_dir')

width = float(rospy.get_param('/screen/width',))
height = float(rospy.get_param('/screen/height'))

border_x = float(rospy.get_param('/screen/border_x', 0))
border_y = float(rospy.get_param('/screen/border_y', 0))

xmin = 0 + border_x
xmax = width - border_x
ymin = 0 + border_y
ymax = height - border_y

services.clear_polygons()

xs = np.linspace(xmin,xmax,4)
ys = np.linspace(ymin,ymax,3)
# xs = np.arange(xmin+0.05, xmax, abs(ys[1] - ys[0]))
xx, yy = np.meshgrid(xs, ys)
points = np.array([(x,y,0) for x,y in zip(xx.ravel(), yy.ravel())])
points_msg = xyz_array_to_pointcloud2(points, now(), '/bottom_left')
# topics.object_cloud(points_msg)
services.set_selection_method(0)

std_order = [
    0,  2,  1,  3,
    7,  5,  6,  4,
    8, 10,  9, 11,
    0,  4,  8 ,
    1,  5,  9 ,
    2,  6,  10,
    3,  7,  11
]


condition = rospy.get_param('~condition')
print condition
if condition == 'standard':
    order = std_order
if condition == 'random':
    import random
    # use sample instead of shuffle because shuffle is in-place
    order = random.sample(std_order, len(std_order))
if condition == 'reversed':
    order = reversed(order)

targets = []
stds = []
times = []
cursor_history = []
history_size = 0

# sleep(2) # wait for the interface to be ready before trying to hilight

start_time = now()
for idx in order:
    topics.object_cloud(xyz_array_to_pointcloud2(np.array([points[idx]]), now(), '/bottom_left'))
    t1 = now()

    point = points[idx]
    targets.append(point)
    # print 'hilighting ', point
    # services.clear_hilights()
    
    pt = PointStamped()
    pt.header.frame_id = '/bottom_left'
    pt.header.stamp = now()
    pt.point.x, pt.point.y, pt.point.z = point
    
    # services.hilight_object(pt, ColorRGBA(255,0,0,0))
    # import pdb; pdb.set_trace()
    click_pts_msg = services.get_cursor_stats()
    click_pt = [
        click_pts_msg.click_pos.point.x,
        click_pts_msg.click_pos.point.y,
        click_pts_msg.click_pos.point.z
    ]
    r = Rate(10)
    print click_pt
    print point
    print '====='

    while not sameObject(click_pt, point):
        print click_pt
        print point
        print '====='
        click_pts_msg = services.get_cursor_stats()
        click_pt = [
            click_pts_msg.click_pos.point.x,
            click_pts_msg.click_pos.point.y,
            click_pts_msg.click_pos.point.z
        ]
        r.sleep()
    # services.clear_hilights()
    click_pts = pointcloud2_to_xyz_array(click_pts_msg.points)
    t2 = now()
    times.append((t2-t1).to_sec())
    history_size = click_pts.shape[0]
    cursor_history.extend(click_pts)
    stds.append(click_pts.std(0))
    print 'std = ', stds[-1]
    sleep(3)
        
# services.clear_hilights()

filename = '%s_%s.mat'
path = os.path.join(data_dir, filename)
timestr = datetime.datetime.today().strftime('%d-%m-%y-%H.%M.%f')
filename = path % (condition, timestr)
scipy.io.savemat(
    filename,
    dict(targets=targets,
        stds=stds,
        times=times,
        cursor_history=cursor_history,
        history_size=history_size,
        total_time=(now()-start_time).to_sec()
    )
)
print 'Saved\n', filename