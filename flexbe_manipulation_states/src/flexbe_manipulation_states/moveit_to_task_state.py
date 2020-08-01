#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyPublisher

from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, DisplayTrajectory, JointConstraint, MoveItErrorCodes

'''
Created on 07.31.2020

@author: Swimming Kim
'''

moveit_commander.roscpp_initialize(sys.argv)

class MoveitToTaskStateTopic(EventState):
	'''
	Uses MoveIt to plan and move the specified joints to the target configuration.

	-- move_group		string		Name of the move group to be used for planning.
									Specified joint names need to exist in the given group.
	-- publish_topic 	string 		Topic on which MoveIt is publishing for calls.

	<= reached 						Target joint configuration has been reached.
	<= planning_failed 				Failed to find a plan to the given joint configuration.
	<= control_failed 				Failed to move the arm along the planned trajectory.

	'''
	def __init__(self, move_group = 'arm'): #, publish_topic = '/move_group/display_planned_path'):
		'''
		Constructor
		'''
		super(MoveitToTaskStateTopic, self).__init__(outcomes=['reached', 'planning_failed', 'control_failed'])
		# self._publish_topic = publish_topic
		# self._publisher = ProxyPublisher({self._publish_topic: DisplayTrajectory})
		
		self._move_group = move_group
		self._moveit_group = moveit_commander.MoveGroupCommander(self._move_group)

		self._current_pose = None
		self._pose_goal = None

		self._planning_failed = False
		self._control_failed = False
		self._success = False

	def execute(self, userdata):
		'''
		Execute this state
		'''
		if self._planning_failed:
			return 'planning_failed'
		if self._control_failed:
			return 'control_failed'
		if self._success:
			return 'reached'

		self._moveit_group.set_pose_target(self._pose_goal)
		plan = self._moveit_group.plan()

		if len(plan.joint_trajectory.points) == 0:
			Logger.logwarn("Planning Fail")
			return "planning_failed"
		else:
			self._moveit_group.go(wait=True)
			rospy.sleep(5)
			moveit_commander.roscpp_shutdown()
			return "reached"


	def on_enter(self, userdata):
		self._current_pose = self._moveit_group.get_current_pose()

		self._planning_failed = False
		self._control_failed = False
		self._success = False
		self._pose_goal = geometry_msgs.msg.Pose()
		self._pose_goal.orientation.w = 1.0
		self._pose_goal.position.x = 0.0
		self._pose_goal.position.y = 0.0
		self._pose_goal.position.z = 0.4