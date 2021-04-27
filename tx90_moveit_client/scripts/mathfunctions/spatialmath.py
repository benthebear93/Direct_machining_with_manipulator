# Basic math function for Direct machining pkg

# Classes for 3D operations, based on Spatial Math Toolbox for python by Peter Corke

# This for refactoring the code
import geometry_msgs.msg
import tf
import sys
import copy
import rospy

class Spatialmath(object):
	def __init__(self):
		super(Spatialmath, self).__init__()

	def euler2quaternion(self, quaternion):
		euler = tf.transformations.euler_from_quaternion(quaternion)
		roll = euler[0]
		pitch = euler[1]
		yaw = euler[2]
		print("roll :", roll,"pitch :", pitch,"yaw :", yaw)