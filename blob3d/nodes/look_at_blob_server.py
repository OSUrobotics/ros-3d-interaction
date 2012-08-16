#!/usr/bin/env python
import roslib; roslib.load_manifest('blob3d')
import rospy
from pr2_python.head import Head
from pr2_python import transform_listener
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from projector_interface import _point_cloud
import numpy as np
from blob3d.srv import LookAtBlob, LookAtBlobResponse
from std_msgs.msg import Bool

head = None
sub = None
def cloud_cb(msg):
    global cloud_msg
    cloud_msg = msg
    
def handle_look_at_blob(msg):
    target_color = np.array([msg.blob_color.r, msg.blob_color.g, msg.blob_color.b])
    try:
	cloud  = _point_cloud.read_points_np(cloud_msg)
	colors = _point_cloud.float2rgb(cloud[:,:,3])
	
	color_dists = [np.linalg.norm(c-target_color) for c in colors[0,:]]
	
	target_pt = cloud[0,np.argmin(color_dists),:3]

	pt = transform_listener.transform_point('base_footprint', cloud_msg.header.frame_id, Point(*target_pt))
	head.look_at_relative_point(pt.x+0.2,pt.y-0.09,pt.z)
    except Exception, e:
	print e
	return LookAtBlobResponse(Bool(False))
    else:
	return LookAtBlobResponse(Bool(True))

if __name__ == '__main__':
    rospy.init_node('look_at_blob_server')
    head = Head()
    head.pointing_frame = '/narrow_stereo_optical_frame'
    sub = rospy.Subscriber('blob_cloud', PointCloud2, cloud_cb)
    s = rospy.Service('look_at_blob', LookAtBlob, handle_look_at_blob)
    rospy.spin()