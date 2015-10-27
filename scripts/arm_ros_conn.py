#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import sys
import rospy
import arm
import actionlib
import numpy as np
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionResult, FollowJointTrajectoryActionFeedback, FollowJointTrajectoryResult
from control_msgs.msg import GripperCommandAction, GripperCommandActionResult, GripperCommandFeedback
from actionlib_msgs.msg import GoalStatus
from time import sleep
from math import *

lJointNames = ["arm_base_to_link1", "link_1_2_joint", "link_2_3_joint", "link_3_4_joint", "link_4_5_joint", "left_gripper_joint", "right_gripper_joint"]


class ARMRosConn():
	_feedback = FollowJointTrajectoryActionFeedback()
	_result = FollowJointTrajectoryActionResult()
	_gripper_feedback = GripperCommandFeedback()
	_gripper_result = GripperCommandActionResult()

	def __init__(self):
		rospy.init_node('arm')

		self.speed = 220
		self.lAngles = [0] * 6
		arm.switch(0)
		arm.switch(2)
		arm.set_hall_mode(3, 0)
		arm.set_hall_mode(5, 0)
		arm.set_tolerance(3, 0)
		arm.set_tolerance(5, 0)

		self._as_arm = actionlib.SimpleActionServer("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction, execute_cb=self.execute_joint_trajectory, auto_start = False)
		self._as_arm.start()
		self._as_gripper = actionlib.SimpleActionServer("gripper_controller/gripper_action", GripperCommandAction, execute_cb=self.execute_gripper_action, auto_start = False)
		self._as_gripper.start()
		self.pub_joint_states = rospy.Publisher("joint_states", JointState, queue_size=16)
		self.run()

	def run(self):
		rate = rospy.Rate(20)
		while not rospy.is_shutdown():
			self.publish_joint_states()
			rate.sleep()
	
	def publish_joint_states(self):
		joint_state = JointState()
		joint_state.header.stamp = rospy.Time.now()
		joint_state.name = lJointNames
		self.lAngles = [-arm.get_angle(0), arm.get_angle(1), -arm.get_angle(2), -arm.get_angle(3), arm.get_angle(4), 0.35-arm.get_angle(5)]
		joint_state.position = self.lAngles[:-1] + [self.lAngles[-1], self.lAngles[-1]]
		self.pub_joint_states.publish(joint_state)

	def execute_joint_trajectory(self, goal):
		self._result.status = FollowJointTrajectoryResult.SUCCESSFUL
		for point in goal.trajectory.points:
			print goal.trajectory.joint_names
			print point.positions
			lGoalPosOrdered = [
				point.positions[goal.trajectory.joint_names.index(lJointNames[0])],
				point.positions[goal.trajectory.joint_names.index(lJointNames[1])],
				point.positions[goal.trajectory.joint_names.index(lJointNames[2])],
				point.positions[goal.trajectory.joint_names.index(lJointNames[3])],
				point.positions[goal.trajectory.joint_names.index(lJointNames[4])],
			]
			try:
				arm.to_angle(0, self.speed, -lGoalPosOrdered[0])
				arm.to_angle(1, self.speed,  lGoalPosOrdered[1])
				arm.to_angle(2, self.speed, -lGoalPosOrdered[2])
				arm.to_angle(3, self.speed, -lGoalPosOrdered[3])
				arm.to_angle(4, self.speed,  lGoalPosOrdered[4])
			except arm.RangeError as e:
				print >> sys.stderr, e.message
				self._feedback.status = GoalStatus.REJECTED
				self._as_arm.publish_feedback(self._feedback.feedback)
				self._result.status = FollowJointTrajectoryResult.INVALID_GOAL
				break

			error = 0
			while True:
				error = np.array(lGoalPosOrdered) - np.array(self.lAngles[:-1])
				print "Error", error
				if all(abs(f) < 0.02 for f in error):
					print "Position reached"
					break

				if self._as_arm.is_preempt_requested():
					self._as_arm.set_preempted()
					break
				sleep(0.001)

			self._feedback.status = GoalStatus.SUCCEEDED
			self._feedback.feedback.joint_names = lJointNames[:-1]
			self._feedback.feedback.desired.positions = lGoalPosOrdered
			self._feedback.feedback.actual.positions = self.lAngles[:-1]
			self._feedback.feedback.error.positions = error
			self._as_arm.publish_feedback(self._feedback.feedback)
		self._as_arm.set_succeeded(self._result.result)


	def execute_gripper_action(self, goal):
		arm.to_angle(5, self.speed, 0.35-goal.command.position)
		while True:
			error = goal.command.position - self.lAngles[-1]
			if abs(error) < 0.02:
				break

			self._gripper_feedback.position = self.lAngles[-1]
			self._gripper_feedback.stalled = False
			self._gripper_feedback.reached_goal = False

			if self._as_gripper.is_preempt_requested():
				self._as_gripper.set_preempted()
				break
			sleep(0.001)
		self._gripper_result.status = GoalStatus.SUCCEEDED
		self._gripper_result.result.position = goal.command.position
		self._gripper_result.result.stalled = False
		self._gripper_result.result.reached_goal = True
		self._as_gripper.set_succeeded(self._gripper_result.result)

if __name__ == '__main__':
	ARMRosConn()
