#!/usr/bin/env python
import roslib; roslib.load_manifest('projector_interface')
from pr2_pick_and_place_demos.pick_and_place_manager import PickAndPlaceManager
from geometry_msgs.msg import PointStamped, PoseStamped, PolygonStamped
from projector_interface.srv import GetCursorStats, DrawPolygon
from std_msgs.msg import Empty
from threading import RLock
import rospy
from matplotlib.nxutils import pnpoly
from pr2_python.geometry_tools import euler_to_quaternion
from object_manipulation_msgs.msg import ManipulationResult
from copy import deepcopy

NOTGRASPING = 0
GRASPING    = 1

x =  0.1405776
y = -0.03434887
w = 0.3
h = 0.2

RECYCLING = PolygonStamped()
TRASH     = PolygonStamped()
RECYCLING.header.frame_id = 'table'
TRASH.header.frame_id     = 'table'

RECYCLING.polygon.points.append(Point(x    , y    ,  0.))
RECYCLING.polygon.points.append(Point(x - w, y    ,  0.))
RECYCLING.polygon.points.append(Point(x - w, y + h,  0.))
RECYCLING.polygon.points.append(Point(x    , y + h,  0.))

TRASH.polygon.points.append(    Point(x    , y + h + 0.01    ,  0.))
TRASH.polygon.points.append(    Point(x - w, y + h + 0.01    ,  0.))
TRASH.polygon.points.append(    Point(x - w, y + h + 0.01 + h,  0.))
TRASH.polygon.points.append(    Point(x    , y + h + 0.01 + h,  0.))

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
        self.cursor_proxy.wait_for_service()
        self.polygon_proxy = rospy.ServiceProxy('draw_polygon', DrawPolygon)
        self.polygon_proxy.wait_for_service()
        
        self.polygon_proxy('Recycling', True, RECYCLING)
        self.polygon_proxy('Garbage', True, TRASH)
        
    
    def click(self, msg):
        if self.click_lock.acquire(blocking=False):
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

    def point_cb(self, msg):
        self.point = msg
    

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
            #self.point.header.stamp = self.point.header.stamp - rospy.Duration(0.03)
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
            place_pose.pose.position.z = 0
            rospy.loginfo('Placing at %s' % place_pose)
            self.place_viz_pub.publish(place_pose)
            # see if it's in one of our predefined boxes
            
            pw,ph = 0.1,0.1
            
            for polygon in [TRASH, RECYCLING]:
                pts_arr = np.array([[(p.x,p.y) for p in polygon.polygon.points]])
                if pnpoly(ptt.point.x, ptt.point.y, pts_arr.squeeze()):
                    # we're doing this from the top right corner (don't ask)
                    place_pose.position = polygon.polygon.points[0]
                    pw,ph = w,h
                    
            self.manager.set_place_area(place_pose, (pw,ph))
            #status = self.manager.place_object(self.grasp_arm, place_pose, padding=0) 
            status = self.manager.put_down_object(self.grasp_arm, max_place_tries=25, use_place_override=1)
            if status == ManipulationResult.SUCCESS:
                self.state = NOTGRASPING
                self.grasp_arm = -1

if __name__ == '__main__':
    rospy.init_node('manipulator')
    manipulator = Manipulator()
    rospy.Subscriber('click', Empty, manipulator.click, queue_size=1)
    rospy.Subscriber('intersected_point', PointStamped, manipulator.point_cb)
    rospy.loginfo('READY')
    rospy.spin()