#!/usr/bin/env python
import roslib; roslib.load_manifest('projector_interface')
from pr2_pick_and_place_demos.pick_and_place_manager import PickAndPlaceManager
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Empty

NOTGRASPING = 0
GRASPING    = 1

class Manipulator(object):
    state = NOTGRASPING
    point = None
    grasp_arm = -1
    def __init__(self):
         self.manager = PickAndPlaceManager()
    
    def click(self, msg):
        if self.state == NOTGRASPING:
            self.grasp()
        elif self.state == GRASPING:
            self.drop()

    def point_cb(self, msg):
        self.point = msg
    
    #TODO need to make sure we can't trigger grasp/drop multiple times concurrently
    def grasp(self):
        if (self.state == NOTGRASPING) and (self.point is not None):
            self.manager.move_arm_to_side(0)
            success = self.manager.pick_up_object_near_point(self.point, 0)
            self.grasp_arm = 0
            if not success:
                self.manager.move_arm_to_side(1)
                success = self.manager.pick_up_object_near_point(self.point, 1)
                self.grasp_arm = 1
            if success:
                self.state = GRASPING
        
    def drop(self):
        if self.state == GRASPING:
            success = self.manager.put_down_object(self.grasp_arm, max_place_tries=25, use_place_override=1)
            if success:
                self.state = NOTGRASPING
                self.grasp_arm = -1

if __name__ == '__main__':
    rospy.init_node('manipulator')
    manipulator = Manipulator()
    rospy.Subscriber('click', Empty, manipulator.click)
    rospy.Subscriber('intersected_point', PointStamped, manipulator.point_cb)