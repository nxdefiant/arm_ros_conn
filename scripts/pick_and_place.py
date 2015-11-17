#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import sys
import rospkg
import rospy
import moveit_commander
import tf
import numpy as np
from math import *
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation
from moveit_msgs.msg import MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler

class MoveItDemo:
	def __init__(self):
		# Initialize the move_group API
		moveit_commander.roscpp_initialize(sys.argv)

		rospack = rospkg.RosPack()
		rospy.init_node('moveit_demo')

		self.tflistener = tf.TransformListener()
		scene	= moveit_commander.PlanningSceneInterface()
		# Create a scene publisher to push changes to the scene
		self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=16)
		self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped)
		self.arm	= moveit_commander.MoveGroupCommander("arm")
		self.arm.set_planner_id("RRTkConfigDefault");
		self.gripper = moveit_commander.MoveGroupCommander("gripper")

		scene.remove_world_object("tube1")
		scene.remove_attached_object("gripper_center", "tube1")
		self.arm.set_goal_position_tolerance(0.04)
		self.arm.set_goal_orientation_tolerance(5*pi/180)

		self.gripper_open()
		self.arm.set_named_target('Home')
		self.arm.go()

		tube1_pose = PoseStamped()
		tube1_pose.header.frame_id = "/arm_base"
		tube1_pose.pose.position.x = 0.5
		tube1_pose.pose.position.y = 0.0
		tube1_pose.pose.position.z = 0.03
		tube1_pose.pose.orientation.w = 1.0
		scene.add_mesh("tube1", tube1_pose, rospack.get_path('arm_ros_conn') + "/meshes/vrohr.stl")
		self.setColor("tube1", 1.0, 1.0, 0, 1.0)

		place_pose = PoseStamped()
		place_pose.header.frame_id = "/arm_base"
		place_pose.pose.position.x = -0.5
		place_pose.pose.position.y = 0.0
		place_pose.pose.position.z = 0.03
		place_pose.pose.orientation.w = 1.0

		rospy.sleep(2) # Give the scene a chance to catch up

		tube1_pose.pose.position.z += 0.02
		if self.pick("tube1", tube1_pose):
			rospy.sleep(2) # Give the scene a chance to catch up
			self.place("tube1", place_pose)
			rospy.sleep(2) # Give the scene a chance to catch up

		self.arm.set_named_target('Home')
		self.arm.go()
		rospy.sleep(2) # Give the scene a chance to catch up

		if self.pick("tube1", place_pose):
			rospy.sleep(2) # Give the scene a chance to catch up
			self.place("tube1", tube1_pose)
			rospy.sleep(2) # Give the scene a chance to catch up

		# Shut down MoveIt cleanly
		moveit_commander.roscpp_shutdown()

		# Exit the script
		moveit_commander.os._exit(0)

	def pick(self, item, pose):
		# Generate a list of grasps
		grasps = self.make_grasps(pose, [item], [5*pi/180] * 2)
		for grasp in grasps:
			self.gripper_pose_pub.publish(grasp.grasp_pose)
			rospy.sleep(0.2)
		# Repeat until we succeed or run out of attempts
		for i in range(3):
			rospy.loginfo("Pick attempt: " + str(i))
			result = self.arm.pick(item, grasps)
			if result == MoveItErrorCodes.SUCCESS:
				rospy.loginfo("Pick succeeded")
				return True
			rospy.loginfo("Pick failed")
			rospy.sleep(0.2)
		
		return False

	def place(self, item, place_pose):
		# Generate valid place poses
		# Rotate the gripper around the z axis to let it point to the target, reference is the x axis
		v = place_pose.pose.position.__getstate__()[:2] # x, y component of position
		v /= np.linalg.norm(v) # unit vector
		q = quaternion_from_euler(0, 0, acos(np.dot(v, [1, 0]))) # use the dot product to get the angle
		place_pose.pose.orientation.x = q[0]
		place_pose.pose.orientation.y = q[1]
		place_pose.pose.orientation.z = q[2]
		place_pose.pose.orientation.w = q[3]

		# Publish the place pose so they it be viewed in RViz
		self.gripper_pose_pub.publish(place_pose)
		rospy.sleep(0.2)

		for i in range(3):
			rospy.loginfo("Place attempt: " + str(i))
			if self.arm.place(item, place_pose):
				rospy.loginfo("Place succeeded")
				break
			rospy.sleep(0.2)

	def gripper_to(self, to):
		self.gripper.set_joint_value_target([to, to])
		self.gripper.go()

	def gripper_open(self):
		self.gripper.set_named_target('open')
		self.gripper.go()
	
	def grasp_tube(self):
		self.gripper_to(0.05)

	# See http://wiki.ros.org/turtlebot_arm_interactive_markers/Tutorials/UsingArmInteractiveMarkers?action=AttachFile&do=get&target=arm_markers_rviz6.jpgt the color of an object
	def setColor(self, name, r, g, b, a=0.9):
		# Initialize a MoveIt color object
		color = ObjectColor()

		# Set the id to the name given as an argument
		color.id = name

		# Set the rgb and alpha values given as input
		color.color.r = r
		color.color.g = g
		color.color.b = b
		color.color.a = a

		# Initialize a planning scene object
		p = PlanningScene()
		# Need to publish a planning scene diff
		p.is_diff = True
		p.object_colors.append(color)
		# Publish the scene diff
		self.scene_pub.publish(p)

	# Get the gripper posture as a JointTrajectory
	def make_gripper_posture(self, joint_positions):
		# Initialize the joint trajectory for the gripper joints
		t = JointTrajectory()

		# Set the joint names to the gripper joint names
		t.joint_names = ["left_gripper_joint", "right_gripper_joint"]

		# Initialize a joint trajectory point to represent the goal
		tp = JointTrajectoryPoint()

		# Assign the trajectory joint positions to the input positions
		tp.positions = joint_positions

		# Set the gripper effort
		tp.effort = [1.0, 1.0]

		tp.time_from_start = rospy.Duration(2.0)

		# Append the goal point to the trajectory points
		t.points.append(tp)

		# Return the joint trajectory
		return t

	# Generate a gripper translation in the direction given by vector
	def make_gripper_translation(self, min_dist, desired, vector):
		# Initialize the gripper translation object
		g = GripperTranslation()

		# Set the direction vector components to the input
		g.direction.vector.x = vector[0]
		g.direction.vector.y = vector[1]
		g.direction.vector.z = vector[2]

		# The vector is relative to the gripper frame
		g.direction.header.frame_id = "gripper_center"

		# Assign the min and desired distances from the input
		g.min_distance = min_dist
		g.desired_distance = desired

		return g

	# Generate a list of possible grasps
	def make_grasps(self, initial_pose_stamped, allowed_touch_objects, grasp_opening=[0, 0]):
		# Initialize the grasp object
		g = Grasp()

		# Set the pre-grasp and grasp postures appropriately;
		# grasp_opening should be a bit smaller than target width
		g.pre_grasp_posture = self.make_gripper_posture([0.35, 0.35])
		g.grasp_posture = self.make_gripper_posture(grasp_opening)

		# Set the approach and retreat parameters as desired
		g.pre_grasp_approach = self.make_gripper_translation(0.05, 0.1, [1.0, 0.0, 0.0])
		g.post_grasp_retreat = self.make_gripper_translation(0.05, 0.1, [0.0, 0.0, 1.0])

		# Set the first grasp pose to the input pose
		g.grasp_pose = initial_pose_stamped

		# A list to hold the grasps
		grasps = []

		# Rotate the gripper around the z axis to let it point to the target, reference is the x axis
		v = initial_pose_stamped.pose.position.__getstate__()[:2] # x, y component of position
		v /= np.linalg.norm(v) # unit vector
		q = quaternion_from_euler(0, 0, acos(np.dot(v, [1, 0]))) # use the dot product to get the angle

		# Set the grasp pose orientation accordingly
		g.grasp_pose.pose.orientation.x = q[0]
		g.grasp_pose.pose.orientation.y = q[1]
		g.grasp_pose.pose.orientation.z = q[2]
		g.grasp_pose.pose.orientation.w = q[3]

		# Set and id for this grasp (simply needs to be unique)
		g.id = str(len(grasps))

		# Set the allowed touch objects to the input list
		g.allowed_touch_objects = allowed_touch_objects

		# Don't restrict contact force
		g.max_contact_force = 0

		# Degrade grasp quality for increasing pitch angles
		g.grasp_quality = 1.0

		# Append the grasp to the list
		grasps.append(g)

		# Return the list
		return grasps

if __name__ == "__main__":
	MoveItDemo()
