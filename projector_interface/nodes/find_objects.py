#!/usr/bin/env python
# Copyright (c) 2013, Oregon State University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Oregon State University nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL OREGON STATE UNIVERSITY BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author Dan Lazewatsky/lazewatd@engr.orst.edu

import roslib; roslib.load_manifest('projector_interface')
from tabletop_object_detector.srv import TabletopSegmentation, TabletopSegmentationRequest
from tabletop_object_detector.msg import Table
from tabletop_object_detector.msg import Table
from pr2_python.pointclouds import xyz_array_to_pointcloud2
from pr2_python.transform_listener import transform_point_stamped, transform_pose_stamped
from tf import TransformBroadcaster
import tf_conversions.posemath as pm
import dynamic_reconfigure.client
from threading import Thread, RLock

import rospy
import tf
import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped

import rospy.service

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

def detect(detect_srv):
    detection = detect_srv.call()
    objects = detection.clusters
    header = rospy.Header()
    header.stamp = rospy.Time.now()
    cloud = None
    table = None
    if objects:
        frame_id = objects[0].header.frame_id
        cloud = xyz_array_to_pointcloud2(np.asarray([location_from_cluster(obj) for obj in objects]), stamp=rospy.Time.now(), frame_id='/table')
    if detection.table.pose.header.frame_id:
        table = detection.table
    return cloud, table

def broadcast_table_frame(args):
    if table_pose is None: return
    with tf_lock:
        trans, rot = pm.toTf(pm.fromMsg(table_pose.pose))
        br.sendTransform(trans, rot, rospy.Time.now(), '/table', table_pose.header.frame_id)
        br.sendTransform(trans, rot, rospy.Time.now() + rospy.Duration(0.005), '/table', table_pose.header.frame_id)
        br.sendTransform(trans, rot, rospy.Time.now() - rospy.Duration(0.005), '/table', table_pose.header.frame_id)

def set_table_filter_limits(table, clients):
    for field, client in clients.iteritems():
        params = dict(
            filter_field_name = field
        )
        if field in ('x','y'):
            # params['input_frame'] = table.pose.header.frame_id
            params['input_frame']  = '/table'
            params['output_frame'] = '/table'
            
            params['filter_limit_min'] = table.__getattribute__('%s_min' % field)
            params['filter_limit_max'] = table.__getattribute__('%s_max' % field)
        else:
            params['input_frame']  = '/table'
            params['output_frame'] = '/table'
            params['filter_limit_min'] = -0.01
            params['filter_limit_max'] =  0.01
            # params['filter_limit_min'] = -5
            # params['filter_limit_max'] =  5
        client.update_configuration(params)
    

if __name__ == '__main__':
    rospy.init_node('find_objects')
    object_pub = rospy.Publisher('object_cloud', PointCloud2)
    table_pub  = rospy.Publisher('table', Table)
    rate = rospy.get_param('~detect_rate', default=0.5)
    br = TransformBroadcaster()	
    detect_srv = rospy.ServiceProxy('/tabletop_segmentation', TabletopSegmentation)
    detect_srv.wait_for_service()
    Thread(target=rospy.Timer(rospy.Duration(0.05), broadcast_table_frame).run)
    #clients = dict(
    #    x = dynamic_reconfigure.client.Client('xfilter', timeout=float('inf')),
    #    y = dynamic_reconfigure.client.Client('yfilter', timeout=float('inf')),
    #    z = dynamic_reconfigure.client.Client('zfilter', timeout=float('inf'))
    #)
    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
        try:
            object_cloud, table = detect(detect_srv)
            if table is not None:
                with tf_lock:
                    if not table_pose or np.linalg.norm(pt2np(table_pose.pose.position) - pt2np(table.pose.pose.position)) > 0.1:
                        table_pose = table.pose
                table_pub.publish(table)

            if object_cloud is not None:
                #set_table_filter_limits(table, clients)
                object_pub.publish(object_cloud)
            r.sleep()
        except rospy.service.ServiceException, e:
            pass