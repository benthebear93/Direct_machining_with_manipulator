# Basic math function for Direct machining pkg

# Classes for 3D operations, based on Spatial Math Toolbox for python by Peter Corke

# This for refactoring the code


class Spatialmath():
	def __init__(self):
		super().__init__()

	def euler2quaternion(quaternion):
	euler = tf.transformations.euler_from_quaternion(quaternion)
	roll = euler[0]
	pitch = euler[1]
	yaw = euler[2]
	print("roll :", roll,"pitch :", pitch,"yaw :", yaw)

if __name__='__main__':
	print("spatialmath is run")