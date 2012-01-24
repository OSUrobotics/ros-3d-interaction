#!/usr/bin/env python
import roslib; roslib.load_manifest('external_camera_localizer')

from laser_assembler.srv import AssembleScans
from sensor_msgs.msg import PointCloud
import rospy

if __name__ == '__main__':
	rospy.init_node('tilt_scan_to_pointcloud')
	rospy.wait_for_service('assemble_scans')
	assemble_scans = rospy.ServiceProxy('assemble_scans', AssembleScans)
	cloud_pub = rospy.Publisher('laser_cloud', PointCloud)
	loop_rate = rospy.Rate(10)
	while not rospy.is_shutdown():
    	try:
			resp = assemble_scans(rospy.Time(0), rospy.Time.now())
			cloud_pub.publish(resp.cloud)
			loop_rate.sleep()
		except rospy.ServiceException, e:
			rospy.logerr('Failed to assemble scans: %s' % e)