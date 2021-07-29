import pybullet as p
import time
import pybullet_data
import utils_tx90
import math as m

import numpy as np
def deg2rad(degree):
	return degree * m.pi/180

class Tx90():
	def __init__(self):
		self.physicsClient = p.connect(p.GUI)
		self.sisbotUrdfPath ="/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/tx90_moveit_client/scripts/RL/urdf/tx90.urdf"
		p.setAdditionalSearchPath(pybullet_data.getDataPath())
		self.load_urdf()

	def load_urdf(self):
		#p.setGravity(0,0,-9.8)
		self.planeId = p.loadURDF("plane.urdf")

		robotStartPos = [0,0,0.0]
		robotStartOrn = p.getQuaternionFromEuler([0,0,0])
		self.robotID = p.loadURDF(self.sisbotUrdfPath, robotStartPos, robotStartOrn,useFixedBase = True,flags=p.URDF_USE_INERTIA_FROM_FILE)
		print("----------------------------------------")
		print("Loading robot from {}".format(self.sisbotUrdfPath))
		self.eefID = 9
		self.joints, self.controlJoints = utils_tx90.setup_sisbot(p, self.robotID)
		self.joint_init = [0, 0, deg2rad(90), 0, 0, 0]
		self.home_pose()
		self.next_state = self.get_state()
		print("next_state :", self.next_state)
		#print("control joints ", self.controlJoints)
		# print(self.ee_pos())

	def home_pose(self):
		for i,name in enumerate(self.controlJoints):
			p.resetJointState(self.robotID,self.joints[name].id,targetValue=self.joint_init[i],targetVelocity=0)
		p.stepSimulation()

	def get_ee_pos(self):
		self.pos = p.getLinkState(self.robotID, self.eefID)[0]
		self.ori = p.getLinkState(self.robotID, self.eefID)[1]
		pos = [self.pos, self.ori]
		return pos 

	def get_state(self):
		joint_states = p.getJointStates(self.robotID, range(p.getNumJoints(self.robotID)))
		joint_positions = [state[0] for state in joint_states] #  -> (8,)
		joint_pos = np.array(joint_positions)
		ee_state = p.getLinkState(self.robotID, self.eefID)
		ee_pos = np.array(ee_state[0])
		ee_ori = ee_state[1]

		obs = np.concatenate([joint_pos, ee_pos])

		print("obs state", obs)
		print("slice state", obs[11:15])

	def move(self, action):
		theta0 = np.interp(10, (-1, 1), (deg2rad(-130), deg2rad(140)))
		print("theta0 : ", theta0)
		

if __name__=='__main__':
	robot=Tx90()
	while True:
		p.stepSimulation()