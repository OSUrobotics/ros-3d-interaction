#!/usr/bin/env python
import roslib; roslib.load_manifest('world_intersect')
import rospy
from tabletop_object_detector.msg import Table
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from sensor_msgs.msg import PointCloud2
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import sin, cos, pi
import tf
from pr2_python.pointclouds import xyz_array_to_pointcloud2

def cast_ray(pose, table, tfl):
    # assume the plane passes through table.pose
    # and the vector table.pose.x,table.pose.y,table.pose.z+1 is a
    # normal for the table in its frame
    
    # point
    q = np.array([
        table.pose.pose.position.x,
        table.pose.pose.position.y,
        table.pose.pose.position.z
    ])
    
    # normal
    m = q + [0,0,1]
    
    pose.header.stamp = rospy.Time(0)
    pose_transformed = tfl.transformPose(table.pose.header.frame_id, pose)
       
    # origin vector
    p = np.array([pose_transformed.pose.position.x, pose_transformed.pose.position.y, pose_transformed.pose.position.z])
    quat = (
        pose_transformed.pose.orientation.x,
        pose_transformed.pose.orientation.y,
        pose_transformed.pose.orientation.z,
        pose_transformed.pose.orientation.w
    )
    ax,ay,az = euler_from_quaternion(quat)
    # direction vector
    # a pose is basically spherical coordinates, so convert to cartesian
    d = np.array([
        -cos(az)*cos(ay),
        -sin(az)*cos(ay),
         sin(ay)
    ])
    # intersection
    t = (q - p).dot(m) / d.dot(m)
    if t < 0: # some normal must be flipped since t is normally > 0
        v = PointStamped()
        v.header = table.pose.header
        v.point.x, v.point.y, v.point.z = p + t*d
        return v
    return False
    
class Intersector(object):
    pose    = None
    table   = None
    rate    = rospy.Rate(20)
    tfl     = tf.TransformListener()
    int_pub = rospy.Publisher('intersected_points', PointCloud2)
    
    def pose_cb(self, pose):
        self.pose = pose
        
    def table_cb(self, table):
        self.table = table
        
    def run(self):
        while not rospy.is_shutdown():
            intersection = cast_ray(self.pose, self.table, self.tfl)
            if intersection:
                cloud = xyz_array_to_pointcloud2(np.array([
                    intersection.point.x,
                    intersection.point.y,
                    intersection.point.z]
                ]))
                cloud.header = self.table.header
                cloud.header.stamp = self.pose.header.stamp
                self.int_pub.publish(cloud)
            self.rate.sleep()
    
if __name__ == '__main__':
    rospy.init_node('intersect_plane')
    intersector = Intersector()
    pose_sub  = rospy.Subscriber('pose',  PoseStamped, intersector.pose_cb)
    table_sub = rospy.Subscriber('table', Table,       intersector.table_cb)

def test():
    rospy.init_node('intersect_plane_test')
    marker_pub = rospy.Publisher('table_marker', Marker)
    pose_pub = rospy.Publisher('pose', PoseStamped)
    int_pub = rospy.Publisher('intersected_points', PointCloud2)
    tfl = tf.TransformListener()
    
    # Test table
    table = Table()
    table.pose.header.frame_id = 'base_link'
    table.pose.pose.orientation.x, table.pose.pose.orientation.y, table.pose.pose.orientation.z, table.pose.pose.orientation.w = (0,0,0,1)
    table.x_min = -0.5
    table.x_max =  0.5
    table.y_min = -0.5
    table.y_max =  0.5

    # A marker for that table
    marker = Marker()
    marker.header.frame_id = table.pose.header.frame_id
    marker.id = 1
    marker.type = Marker.LINE_STRIP
    marker.action = 0
    marker.pose = table.pose.pose
    marker.scale.x, marker.scale.y, marker.scale.z = (0.005,0.005,0.005)
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.0,1.0,1.0,1.0) 
    marker.frame_locked = False
    marker.ns = 'table'
    marker.points = [
        Point(table.x_min,table.y_min, table.pose.pose.position.z),
        Point(table.x_min,table.y_max, table.pose.pose.position.z),
        Point(table.x_max,table.y_max, table.pose.pose.position.z),
        Point(table.x_max,table.y_min, table.pose.pose.position.z),
        Point(table.x_min,table.y_min, table.pose.pose.position.z),
    ]
    marker.colors = []
    marker.text = ''
    # marker.mesh_resource = ''
    marker.mesh_use_embedded_materials = False
    marker.header.stamp = rospy.Time.now()

    # A test pose
    pose = PoseStamped()
    pose.header = table.pose.header
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (0,0,0.5)
    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quaternion_from_euler(0,-pi/5,pi/5)
    
    intersection = cast_ray(pose, table, tfl)
    cloud = xyz_array_to_pointcloud2(np.array([[intersection.point.x, intersection.point.y, intersection.point.z]]))
    cloud.header = pose.header
    
    while not rospy.is_shutdown():
        marker_pub.publish(marker)
        pose_pub.publish(pose)
        int_pub.publish(cloud)
        rospy.loginfo('published')
        rospy.sleep(0.1)