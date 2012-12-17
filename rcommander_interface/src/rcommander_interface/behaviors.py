import roslib; roslib.load_manifest('rcommander_interface')
import actionlib
import rospy
from rcommander_web.msg import RunScriptAction, RunScriptGoal

BEHAVIOR_PATH = '/u/lazewatskyd/robot_behaviors'
import os

class Behaviors:
	def __init__(self):
		ns = '/run_rcommander_action_web'
		self.action_client = actionlib.SimpleActionClient(ns, RunScriptAction)
		self.action_client.wait_for_server()
		self.rocker_on_goal  = RunScriptGoal(os.path.join(BEHAVIOR_PATH, 'Behaviors/rcommander_behaviors/rocker_switch/rocker_on'))
		self.rocker_off_goal = RunScriptGoal(os.path.join(BEHAVIOR_PATH, 'Behaviors/rcommander_behaviors/rocker_switch/rocker_off'))

	def rocker_on(self, wait=True):
		self.action_client.send_goal(self.rocker_on_goal)
		if wait:
			return self.action_client.wait_for_result()

	def rocker_off(self, wait=True):
		self.action_client.send_goal(self.rocker_off_goal)
		if wait:
			return self.action_client.wait_for_result()


# if __name__ == '__main__':
# 	rospy.init_node('action_test')
# 	b = Behaviors()
# 	while not rospy.is_shutdown():
# 		b.rocker_on()
# 		b.rocker_off()