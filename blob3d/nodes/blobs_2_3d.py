import roslib; roslib.load_manifest('blob3d')
import rospy
from cmvision.msg import Blobs
from sensor_msgs.msg import PointCloud2
from projector_interface._point_cloud import read_points_np, create_cloud_np
#from pr2_python.pointclouds import xyz_array_to_pointcloud2
import numpy as np

blobs = []
cloud = None
cloud_header = None
fields = None

def blob_cb(msg):
	global blobs
	blobs = msg

def cloud_cb(msg):
	global cloud, cloud_header, fields
	cloud = read_points_np(msg)
	cloud_header = msg.header
	fields = msg.fields

if __name__ == '__main__':
	rospy.init_node('blob_2_3d')
	rospy.Subscriber('/blobs', Blobs, blob_cb)
	rospy.Subscriber('cloud', PointCloud2, cloud_cb)
	cloud_pub = rospy.Publisher('blob_cloud', PointCloud2)
	while not rospy.is_shutdown():
		if not blobs:
			continue
		if not cloud is not None:
			continue
		blob_pts = []
		for blob in blobs.blobs:
			point = cloud[blob.y, blob.x]
			blob_pts.append(point)
		
		out_cloud = create_cloud_np(cloud_header, fields, np.array(blob_pts))
		cloud_pub.publish(out_cloud)