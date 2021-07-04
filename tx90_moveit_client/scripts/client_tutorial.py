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
	"""
	Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
	@param: goal       A list of floats, a Pose or a PoseStamped
	@param: actual     A list of floats, a Pose or a PoseStamped
	@param: tolerance  A float
	@returns: bool
	"""
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
		return all_close(joint_goal, current_joints, 0.01)

	def go_to_pose_goal(self):
		move_group = self.move_group

		## BEGIN_SUB_TUTORIAL plan_to_pose
		##
		## Planning to a Pose Goal
		## ^^^^^^^^^^^^^^^^^^^^^^^
		## We can plan a motion for this group to a desired pose for the
		## end-effector:
		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.position.x = 0.823
		pose_goal.position.y = -0.149
		pose_goal.position.z = 0.926026
		pose_goal.orientation.x = 0.0001659
		pose_goal.orientation.y = 0.7070751
		pose_goal.orientation.z = -0.000276
		pose_goal.orientation.w = 0.7071383
		
		move_group.set_pose_target(pose_goal)

		## Now, we call the planner to compute the plan and execute it.
		plan = move_group.go(wait=True)
		# Calling `stop()` ensures that there is no residual movement
		move_group.stop()
		# It is always good to clear your targets after planning with poses.
		# Note: there is no equivalent function for clear_joint_value_targets()
		move_group.clear_pose_targets()

		## END_SUB_TUTORIAL

		# For testing:
		# Note that since this section of code will not be included in the tutorials
		# we use the class variable rather than the copied state variable
		# # curret_pose = self.move_group.get_curret_pose().pose
		# pose_goal = geometry_msgs.msg.Pose()
		# pose_goal.orientation.w = 1.0
		# pose_goal.position.x = 0.1
		# pose_goal.position.y = 0.1
		# pose_goal.position.z = 0.4
		# self.move_group.set_pose_target(pose_goal)

		# plan = self.move_group.go(wait=True)

		# self.move_group.stop()

		# self.move_group.clear_pos_targets()

		# curret_pose = self.move_group.get_curret_pose().pose
		# print("curret_pose", curret_pose)
		# return all_close(pose_goal, curret_pose, 0.01)

	def plan_cartesian_path(self, scale=1):
		print ("plan cartersian")
		group = self.move_group
		waypoints = []

		wpose = group.get_current_pose().pose
		# wpose.position.z -= scale * 0.1  # First move up (z)
		# wpose.position.y += scale * 1.5  # and sideways (y)
		# waypoints.append(copy.deepcopy(wpose))1`1

		# wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
		# waypoints.append(copy.deepcopy(wpose))
		for i in range(15):
			#wpose.position.x += scale * 0.1 
			#wpose.position.z += scale * 0.1 
			wpose.position.z += scale * 0.02
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

	def display_trajectory(self,plan):
		display_trajectory_publisher = self.display_trajectory_publisher

		display_trajectory = moveit_msgs.msg.DisplayTrajectory()
		display_trajectory.trajectory_start = self.robot.get_current_state()
		display_trajectory.trajectory.append(plan)
		display_trajectory_publisher.publish(display_trajectory)

	def execute_plan(self, plan):
		self.move_group.execute(plan, wait=True)

def main():
	staubli_client = StaubliScanning()
	# staubli_client.go_to_pose_goal()
	staubli_client.go_to_joint_state()
	cartesian_plan, fraction = staubli_client.plan_cartesian_path()
	staubli_client.display_trajectory(cartesian_plan)
	staubli_client.execute_plan(cartesian_plan)
	# print "=========cartesian done========="
	# rospy.sleep(2) 
	#plan , faction = staubli_client.plan_cartesian_path()
	#staubli_client.execute_plan(plan)
	staubli_client.find_curr_pose()
	#print"=========go to pose goal========="
if __name__ == '__main__':
	staubli_client = StaubliScanning()
	######

	staubli_client.go_to_joint_state()
	# cartesian_plan, fraction = staubli_client.plan_cartesian_path()
	# staubli_client.display_trajectory(cartesian_plan)
	# staubli_client.execute_plan(cartesian_plan)
	
	#######

	# staubli_client.go_to_pose_goal()
	staubli_client.find_curr_pose()
	#main()