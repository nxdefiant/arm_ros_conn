#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import rospy
import arm
from sensor_msgs.msg import JointState

class ARMRosConn():
	def __init__(self):
		rospy.init_node('arm')

		arm.switch(0)
		arm.switch(2)
		arm.set_hall_mode(3, 0)
		arm.set_hall_mode(5, 0)
		arm.set_tolerance(3, 0)
		arm.set_tolerance(5, 0)

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
		joint_state.name = ["base_to_link1", "link_1_2_joint", "link_2_3_joint", "gripper_joint_1", "gripper_joint_2", "left_gripper_joint", "right_gripper_joint"]
		joint_state.position = [arm.get_angle(0), arm.get_angle(1), arm.get_angle(2), arm.get_angle(3), arm.get_angle(4), arm.get_angle(5)/2, arm.get_angle(5)/2]
		self.pub_joint_states.publish(joint_state)


if __name__ == '__main__':
	ARMRosConn()
