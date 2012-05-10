#!/usr/bin/env python
import roslib; roslib.load_manifest('projector_interface')
from std_msgs.msg import ColorRGBA
from pr2_python.pointclouds import pointcloud2_to_xyz_array

load('rosh_robot', globals())

points = [
    [-0.5365211367607117,   0.2706190347671509,     0.0],
    [-0.7820538282394409,   0.29613807797431946,    0.0],
    [-0.21059279143810272,  0.267086386680603,      0.0],
    [-0.2007334679365158,  -0.010890659876167774,   0.0],
    [0.05050656571984291,   0.31135380268096924,    0.0],
    [-0.5381582379341125,   0.512862503528595,      0.0],
    [-0.5768512487411499,  -0.011733915656805038,   0.0],
    [-0.2181294560432434,   0.5227704048156738,     0.0]
]

for point in points:
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
    print 'std = ', click_pts.std(0)
    sleep(3)
    
    
services.clear_hilights()