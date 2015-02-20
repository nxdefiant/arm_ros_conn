#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import rospy
import tf
from nav_msgs.msg import Odometry

class ARMRosConn():
	def __init__(self):
		rospy.init_node('arm')

		rospy.Subscriber("odom", Odometry, self.odom_received)
		self.tf_broadcaster = tf.broadcaster.TransformBroadcaster()

		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			rate.sleep()

	def odom_received(self, msg):
		pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
		orientation = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
		self.tf_broadcaster.sendTransform(pos, orientation, rospy.Time.now(), "base_link", "odom")

if __name__ == '__main__':
	ARMRosConn()
