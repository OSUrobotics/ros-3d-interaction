#!/usr/bin/env rosh

while ok():
    goal = msg.pr2_controllers_msgs.PointHeadGoal(target=topics.intersected_point[0], min_duration=Duration(0.1))
    topics.head_traj_controller.point_head_action.goal(goal = goal)
    sleep(0.1)

