#!/usr/bin/env python
import roslib; roslib.load_manifest('projector_interface')
from pr2_pick_and_place_demos.pick_and_place_manager import PickAndPlaceManager
from geometry_msgs.msg import PointStamped, PoseStamped, PolygonStamped, Point
from sensor_msgs.msg import PointCloud2
from projector_interface.srv import GetCursorStats, DrawPolygon, CircleInhibit
from std_msgs.msg import Empty
from threading import RLock
import rospy
from matplotlib.nxutils import pnpoly
from pr2_python.geometry_tools import euler_to_quaternion
from object_manipulation_msgs.msg import ManipulationResult
from copy import deepcopy
import numpy as np
from projector_interface._point_cloud import read_points_np


NOTGRASPING = 0
GRASPING    = 1

x =  0.1405776
y = -0.03434887
w = 0.27
h = 0.22

RECYCLING = PolygonStamped()
TRASH     = PolygonStamped()
RECYCLING.header.frame_id = 'table'
TRASH.header.frame_id     = 'table'

RECYCLING.polygon.points.append(Point(x    , y    ,  0.))
RECYCLING.polygon.points.append(Point(x - w, y    ,  0.))
RECYCLING.polygon.points.append(Point(x - w, y + h,  0.))
RECYCLING.polygon.points.append(Point(x    , y + h,  0.))

TRASH.polygon.points.append(    Point(x    , y + h + 0.12    ,  0.))
TRASH.polygon.points.append(    Point(x - w, y + h + 0.12    ,  0.))
TRASH.polygon.points.append(    Point(x - w, y + h + 0.12 + h,  0.))
TRASH.polygon.points.append(    Point(x    , y + h + 0.12 + h,  0.))

class Manipulator(object):
    state = NOTGRASPING
    point = None
    grasp_arm = -1
    INPROGRESS = False
    click_lock = RLock()
    def __init__(self):
        self.place_viz_pub = rospy.Publisher('place_pose', PoseStamped)
        self.manager = PickAndPlaceManager()
        self.cursor_proxy = rospy.ServiceProxy('get_cursor_stats', GetCursorStats)
        #~ self.cursor_proxy.wait_for_service()
        self.polygon_proxy = rospy.ServiceProxy('draw_polygon', DrawPolygon)
        #~ self.polygon_proxy.wait_for_service()
        self.circle_inhibit_proxy = rospy.ServiceProxy('circle_inhibit', CircleInhibit)
        #~ self.circle_inhibit_proxy.wait_for_service()
        
        #~ self.polygon_proxy('Recycling', True, RECYCLING)
        #~ self.polygon_proxy('Garbage', True, TRASH)
        
    
    def click(self, msg):
        if self.click_lock.acquire(blocking=False):
            #~ self.circle_inhibit_proxy(True)
            rospy.logwarn('Calling tabletop detection')
            self.manager.call_tabletop_detection()
            # self.point = self.cursor_proxy().click_pos
            print 'STATE = ',self.state
            if self.state == NOTGRASPING:
                rospy.logwarn('About to grasp')
                self.grasp()
            elif self.state == GRASPING:
                rospy.logwarn('About to drop')
                self.drop()
            self.click_lock.release()
            #~ self.circle_inhibit_proxy(False)

    def point_cb(self, msg):
        #self.point = msg
        cloud = read_points_np(msg)
        point = PointStamped()
        point.header = msg.header
        if cloud.shape[1] == 0: return
        point.point.x, point.point.y, point.point.z = cloud[0][0]
        self.point = point
    

    def transformAtLatestCommonTime(self, target_frame, point):
        lct = self.manager.tf_listener.getLatestCommonTime(point.header.frame_id, target_frame)
        point_lct = deepcopy(point)
        point_lct.header.stamp = lct
        return self.manager.tf_listener.transformPoint(target_frame, point_lct)

    #TODO need to make sure we can't trigger grasp/drop multiple times concurrently
    def grasp(self):
        print self.state, self.point
        if (self.state == NOTGRASPING) and (self.point is not None):
            rospy.loginfo('===grasping')
            self.manager.move_arm_to_side(0)
            ptt = self.transformAtLatestCommonTime('base_link', self.point)
            status = self.manager.pick_up_object_near_point(ptt, 0)
            self.grasp_arm = 0
            # if not success:
            #     self.manager.move_arm_to_side(1)
            #     success = self.manager.pick_up_object_near_point(self.point, 1)
            #     self.grasp_arm = 1
            if status == ManipulationResult.SUCCESS:
                self.state = GRASPING
                rospy.logerr('SUCCESSFUL GRASP')
            else:
                rospy.logerr('FAILED TO GRASP')
        
    def drop(self):
        if self.state == GRASPING:
            #success = self.manager.put_down_object(self.grasp_arm, max_place_tries=25, use_place_override=1)
            place_pose = PoseStamped()
            ptt = self.transformAtLatestCommonTime('table', self.point)
            place_pose.header = ptt.header 
            place_pose.pose.position = ptt.point
            place_pose.pose.orientation = euler_to_quaternion(0,0,0)
            place_pose.pose.position.z = 0.03
            # see if it's in one of our predefined boxes
            
            pw,ph = 0.1,0.1
       
            #boxes = [('TRASH', TRASH), ('RECYCLING', RECYCLING)]
            boxes = []
            for name, polygon in boxes:
                pts_arr = np.array([[(p.x,p.y) for p in polygon.polygon.points]])
                print pts_arr
                if pnpoly(ptt.point.x, ptt.point.y, pts_arr.squeeze()):
                    # we're doing this from the top right corner (don't ask)
                    # place_pose.pose.position = polygon.polygon.points[0]
                    poly_center = pts_arr.squeeze().mean(0)
                    place_pose.pose.position.x = poly_center[0]
                    place_pose.pose.position.y = poly_center[1]
                    pw,ph = w,h
                    rospy.loginfo('Placing in box: %s' % name)
                    
            rospy.loginfo('Placing at %s' % place_pose)
            self.place_viz_pub.publish(place_pose)
            
            #import pdb; pdb.set_trace()

            self.manager.set_place_area(place_pose, (pw,ph))
            # status = self.manager.place_object(self.grasp_arm, place_pose, padding=0) 
            status = self.manager.put_down_object(self.grasp_arm, max_place_tries=25, use_place_override=1)
            if status == ManipulationResult.SUCCESS:
                self.state = NOTGRASPING
                self.grasp_arm = -1

if __name__ == '__main__':
    rospy.init_node('manipulator')
    manipulator = Manipulator()
    rospy.Subscriber('click', Empty, manipulator.click, queue_size=1)
    rospy.Subscriber('intersected_points', PointCloud2, manipulator.point_cb)
    rospy.loginfo('READY')
    rospy.spin()
