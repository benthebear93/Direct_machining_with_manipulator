#!/usr/bin/env python2

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import geometry_msgs
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np
from moveit_msgs.srv import GetPositionFK

def all_close(goal, actual, tolerance):
	all_equal = True
	if type(goal) is list:
		for index in range(len(goal)):
			if abs(actual[index] - goal[index]) > tolerance:
				return False

	elif type(goal) is geometry_msgs.msg.PoseStamped:
		return all_close(goal.pose, actual.pose, tolerance)

	elif type(goal) is geometry_msgs.msg.Pose:
		return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

	return True

def quaternion2euler(quaternion):
	euler = tf.transformations.euler_from_quaternion(quaternion)
	roll = euler[0]
	pitch = euler[1]
	yaw = euler[2]
	print("roll :", np.rad2deg(roll),"pitch :", np.rad2deg(pitch),"yaw :", np.rad2deg(yaw))

class StaubliScanning(object):
	def __init__(self):
		super(StaubliScanning, self).__init__()

		#init moveit_commander 
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('move_group_python_interface', anonymous=True)

		#init RobotCommander object
		self.robot = moveit_commander.RobotCommander()

		#init planning scene obejct
		self.scene = moveit_commander.PlanningSceneInterface()
		rospy.sleep(2)

		group_name = "tx_90"
		self.move_group =  moveit_commander.MoveGroupCommander(group_name)

		# create 'display Trajectory',  publisher which is used later to publish
		# trajectories visualized
		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
		planning_frame = self.move_group.get_planning_frame()
		print ("============ Reference frame: %s" % planning_frame)

		# We can also print the name of the end-effector link for this group:
		eef_link = self.move_group.get_end_effector_link()
		print ("============ End effector: %s" % eef_link)

		# We can get a list of all the groups in the robot:
		group_names = self.robot.get_group_names()
		print ("============ Robot Groups:", self.robot.get_group_names())

		# Sometimes for debugging it is useful to print the entire state of the
		# robot:
		print ("============ Printing robot state")
		#print self.robot.get_current_state()
		#print ""
		## END_SUB_TUTORIAL

	# def depthcam_pose(self):


	def find_curr_pose(self):
		current_pose = self.move_group.get_current_pose().pose
		print("pose :", current_pose.position)
		print("ori :", current_pose.orientation)
		quaternion = (current_pose.orientation.x,
			current_pose.orientation.y,
			current_pose.orientation.z,
			current_pose.orientation.w)
		quaternion2euler(quaternion)

	def go_to_joint_state(self):
		move_group = self.move_group

		joint_goal = move_group.get_current_joint_values()
		joint_goal[0] = 0
		joint_goal[1] = np.deg2rad(0)
		joint_goal[2] = np.deg2rad(90)
		joint_goal[3] = 0
		joint_goal[4] = np.deg2rad(0)
		joint_goal[5] = np.deg2rad(0)  # 1/6 of a turn

		# The go command can be called with joint values, poses, or without any
		# parameters if you have already set the pose or joint target for the group
		move_group.go(joint_goal, wait=True)

		# Calling ``stop()`` ensures that there is no residual movement
		move_group.stop()

		## END_SUB_TUTORIAL

		# For testing:
		current_joints = move_group.get_current_joint_values()
		print("                              ")
		print("current joint ",current_joints)
		print("                              ")
		return all_close(joint_goal, current_joints, 0.01)

	def go_to_pose_goal(self):
		move_group = self.move_group


		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.position.x = 0.778-0.003 # 3mm offset 
		pose_goal.position.y = -0.018+0.004 # 5mm offset
		pose_goal.position.z = -0.260 + 0.478 # negative is actual z position value 

		pose_goal.orientation.x = 0.001674
		pose_goal.orientation.y = -0.932839
		pose_goal.orientation.z = -0.001748                                                                                              
		pose_goal.orientation.w = -0.36028 	

		move_group.set_pose_target(pose_goal)

		plan = move_group.go(wait=True)
		move_group.stop()
		move_group.clear_pose_targets()


	def plan_cartesian_path(self, scale=1):
		print ("plan cartersian")
		group = self.move_group
		waypoints = []

		wpose = group.get_current_pose().pose
		for i in range(7):
			wpose.position.z -= scale * 0.1
			waypoints.append(copy.deepcopy(wpose))
			print("waypoints ", waypoints[i])
		for i in range(7):
			wpose.position.x += scale * 0.1 
			waypoints.append(copy.deepcopy(wpose))
			print("waypoints ", waypoints[i])
		# We want the Cartesian path to be interpolated at a resolution of 1 cm
		# which is why we will specify 0.01 as the eef_step in Cartesian
		# translation.  We will disable the jump threshold by setting it to 0.0 disabling:
		(plan, fraction) = group.compute_cartesian_path(
										waypoints,   # waypoints to follow
										0.01,        # eef_step
										0.0)         # jump_threshold
		# Note: We are just planning, not asking move_group to actually move the robot yet:
		return plan, fraction

	def execute_plan(self, plan):
		self.move_group.execute(plan, wait=True)

if __name__ == '__main__':
	staubli_client = StaubliScanning()
	
	# cartesian_plan, fraction = staubli_client.plan_cartesian_path()
	# staubli_client.execute_plan(cartesian_plan)

	# staubli_client.go_to_joint_state()
	staubli_client.go_to_pose_goal()