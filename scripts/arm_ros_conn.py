#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import rospy
import tf
import prctl
#import cProfile
import threading
from pyshared.json_client import JsonClient
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class ARMRosConn():
	def __init__(self):
		rospy.init_node('arm')
		prctl.set_name("arm_ros_bridge")
		self.__lock_cmd = threading.Lock()
		self.pComm = JsonClient(("arm", 10002))

		rospy.Subscriber("cmd_vel", Twist, self.cmdVelReceived)
		self.sub_scan = None
		self.motion_enabled_last = None
		self.pub_odom = rospy.Publisher("odom", Odometry, queue_size=16)
		self.tf_broadcaster = tf.broadcaster.TransformBroadcaster()

		self.run()

	def command(self, cmd):
		with self.__lock_cmd:
			return self.pComm.write(cmd)

	def run(self):
		rate = rospy.Rate(10.0)
		#pr = cProfile.Profile()
		#pr.enable()
		while not rospy.is_shutdown():
			self.send_odometry()

			rate.sleep()
		#pr.disable()
		#pr.dump_stats("/tmp/test.stats")

	def send_odometry(self):
		current_time = rospy.Time.now()
                lPos, fAngle, lSpeed = self.command("get engine pose")

		# since all odometry is 6DOF we'll need a quaternion created from yaw
		odom_quat = tf.transformations.quaternion_from_euler(0, 0, fAngle)

		# first, we'll publish the transform over tf
		self.tf_broadcaster.sendTransform((lPos[0], lPos[1], 0.0), odom_quat, current_time, "base_link", "odom")

		# next, we'll publish the odometry message over ROS
		odom = Odometry()
		odom.header.stamp = current_time
		odom.header.frame_id = "/odom"

		# set the position
		odom.pose.pose.position.x = lPos[0]
		odom.pose.pose.position.y = lPos[1]
		odom.pose.pose.position.z = 0.0
		odom.pose.pose.orientation.x = odom_quat[0]
		odom.pose.pose.orientation.y = odom_quat[1]
		odom.pose.pose.orientation.z = odom_quat[2]
		odom.pose.pose.orientation.w = odom_quat[3]

		# set the velocity
		odom.child_frame_id = "base_link"
		odom.twist.twist.linear.x = lSpeed[0]
		odom.twist.twist.linear.y = 0.0
		odom.twist.twist.angular.z = lSpeed[1]

		# publish the message
		self.pub_odom.publish(odom)

	# test with rostopic pub -1 cmd_vel geometry_msgs/Twist '[0, 0, 0]' '[0, 0, 0]'
	def cmdVelReceived(self, msg):
		trans = msg.linear.x
		rot = msg.angular.z

		self.command("set engine speed %f %f" % (trans, rot))

if __name__ == '__main__':
	ARMRosConn()
