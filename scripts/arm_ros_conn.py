#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import rospy

class ARMRosConn():
	def __init__(self):
		rospy.init_node('arm')

		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			rate.sleep()


if __name__ == '__main__':
	ARMRosConn()
