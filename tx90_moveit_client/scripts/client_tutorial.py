#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

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

class StaubliScanning(object):
	def __init__(self):
		super(StaubliScanning, self).__init__()

		#init moveit_commander 
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('mvoe_gour_python_interface', anonymous=True)

		#init RobotCommander object

		self.robot = moveit_commander.RobotCommander()

		#init planning scene obejct
		self.scene = moveit_commander.PlanningSceneInterface()
		rospy.sleep(2)
		## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
		## to one group of joints.  In this case the group is the joints in the Panda
		## arm so we set ``group_name = panda_arm``. If you are using a different robot,
		## you should change this value to the name of your robot arm planning group.
		## This interface can be used to plan and execute motions on the Panda:
		group_name = "tx_90"
		self.move_group =  moveit_commander.MoveGroupCommander(group_name)

		# create 'display Trajectory',  publisher which is used later to publish
		# trajectories visualized
		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
		planning_frame = self.move_group.get_planning_frame()
		print "============ Reference frame: %s" % planning_frame

		# We can also print the name of the end-effector link for this group:
		eef_link = self.move_group.get_end_effector_link()
		print "============ End effector: %s" % eef_link

		# We can get a list of all the groups in the robot:
		group_names = self.robot.get_group_names()
		print "============ Robot Groups:", self.robot.get_group_names()

		# Sometimes for debugging it is useful to print the entire state of the
		# robot:
		print "============ Printing robot state"
		print self.robot.get_current_state()
		print ""
		## END_SUB_TUTORIAL

	def go_to_joint_state(self, j_val):
		# Copy class variables to local variables to make the web tutorials more clear.
		# In practice, you should use the class variables directly unless you have a good
		# reason not to.

		## BEGIN_SUB_TUTORIAL plan_to_joint_state
		##
		## Planning to a Joint Goal
		## ^^^^^^^^^^^^^^^^^^^^^^^^
		## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
		## thing we want to do is move it to a slightly better configuration.
		# We can get the joint values from the group and adjust some of the values:
		joint_goal = self.move_group.get_current_joint_values()
		joint_goal[0] = j_val[0]
		joint_goal[1] = j_val[1]
		joint_goal[2] = j_val[2]
		joint_goal[3] = j_val[3]
		joint_goal[4] = j_val[4]
		joint_goal[5] = j_val[5]
		# The go command can be called with joint values, poses, or without any
		# parameters if you have already set the pose or joint target for the group
		self.move_group.go(joint_goal, wait=True)

		# Calling ``stop()`` ensures that there is no residual movement
		self.move_group.stop()

		## END_SUB_TUTORIAL

		# For testing:
		# Note that since this section of code will not be included in the tutorials
		# we use the class variable rather than the copied state variable
		current_joints = self.move_group.get_current_joint_values()
		return all_close(joint_goal, current_joints, 0.01)

	def go_to_pose_goal(self):

		curret_pose = self.move_group.get_curret_pose().pose
		# pose_goal = geometry_msgs.msg.Pose()
		# pose_goal.orientation.w = 1.0
		# pose_goal.position.x = 0.1
		# pose_goal.position.y = 0.1
		# pose_goal.position.z = 0.4
		# self.move_group.set_pose_target(pose_goal)

		# plan = self.move_group.go(wait=True)
		# self.move_group.stop()
		# self.move_group.clear_pos_targets()

		#curret_pose = self.move_group.get_curret_pose().pose
		return all_close(pose_goal, curret_pose, 0.01)

	def plan_cartesian_path(self, scale=1):
		group = self.move_group
		waypoints = []

		wpose = group.get_current_pose().pose
		# wpose.position.z -= scale * 0.1  # First move up (z)
		wpose.position.y += scale * 1.5  # and sideways (y)
		waypoints.append(copy.deepcopy(wpose))

		# wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
		# waypoints.append(copy.deepcopy(wpose))

		# wpose.position.y -= scale * 0.1  # Third move sideways (y)
		# waypoints.append(copy.deepcopy(wpose))

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
	ori = [0, 0, 0, 0, 0, 0]
	second = [0, 0, pi/2 ,0 ,0, 0]
	staubli_client.go_to_joint_state(ori)
	staubli_client.go_to_joint_state(second)
	# cartesian_plan, fraction = staubli_client.plan_cartesian_path()
	# staubli_client.display_trajectory(cartesian_plan)
	# staubli_client.execute_plan(cartesian_plan)
	# print "=========cartesian done========="
	# rospy.sleep(2) 
	staubli_client.go_to_pose_goal()
	print"=========go to pose goal========="
if __name__ == '__main__':
	main()