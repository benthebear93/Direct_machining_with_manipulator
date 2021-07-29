import gym
from gym import spaces

import numpy as np
import pybullet as p
import math
import random

def distance(curr_pose, goal_pose):
	dist = np.sqrt(np.sum((curr_pose-goal_pose)**2, axis=0))
	return dist

class Tx90Env(gym.Env):
	def __init__(self):

		self.done = None
		self.reward = None
		self.max_steps = 1000 # limit the max episode step
		self.iterator = 0
		self.time_step = 0
		self.NrofJoints = 6
		self.distance_threshold = 0.01
		self.controlInit = True
		self.done = None
		# learning parameters
		self.x_min = -0.3
		self.x_max = 1.2
		self.y_min = -1.0
		self.y_max = 1.0
		self.z_min = 0.3
		self.z_max = 1.3
		self.theta = self.generate_random_angle()
		# workspace limit (observation limit)

		self.min_action = -1.0
		self.max_action = 1.0
		# action range

		self.joint_min = np.deg2rad(np.array([-180,-130,-145,-270,-115,-270], dtype=np.float32))
		self.joint_max = np.deg2rad(np.array([180, 147.5,145,270,140,270], dtype=np.float32))
		# low  = np.deg2rad(np.array([-180,-130,-145,-270,-115,-270], dtype=np.float32))
		# high = np.deg2rad(np.array([180, 147.5,145,270,140,270], dtype=np.float32))

		coord_low = np.array([x_min, y_min, z_min], dtype=np.float32)
		coord_high = np.array([x_max, y_max, z_max], dtype=np.float32)

		low = np.concatenate(coord_low, self.joint_min)
		high = np.concatenate(coord_high, self.joint_max) 
		# observation space : joint position and ee position 9 
		self.action_space = spaces.Box(low=self.min_action, high=self.max_action, shape=(6,), dtype=np.float32)
		self.observation_space = spaces.Box(low, high, dtype=np.float32)

		self.tcp_pos = [0.0, 0.0, 0.0] # unit = meter
		self.tcp_ori = [0.0, 0.0, 0.0] # unit = meter

	def loadURDF(self):
		self.URDFPath = "/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/tx90_moveit_client/scripts/RL/urdf/tx90.urdf"
		self.physicsClient = p.connect(p.GUI)
		p.setAdditionalSearchPath(pybullet_data.getDataPath())
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

	def home_pose(self):
		for i,name in enumerate(self.controlJoints):
			p.resetJointState(self.robotID,self.joints[name].id,targetValue=self.joint_init[i],targetVelocity=0)
		self.pos = p.getLinkState(self.robotID, self.eefID)[0]
		self.ori = p.getLinkState(self.robotID, self.eefID)[1]
		print(self.pos, self.ori)
		p.stepSimulation()

	def step(self, action):
		goal_pos = generate_random_point()
		if selt.controlInit:
			self.controlInit = False
			self.move(action)
			self.next_state = self.get_state() # jointpos + ee position
			ee_pos = self.next_state[11:15]
			
			self.done = False
			error = distance(ee_pos, goal_pose)
			if error > 0.01
				self.done = True
				print('out of range')
				reward = 0
			if error < 0.01
				self.done = True
				reward = 30
				print ('near to point')
			return self.next_state, reward, self.done

	def get_state(self):
		joint_states = p.getJointStates(self.robotID, range(p.getNumJoints(self.robotID)))
		joint_positions = [state[0] for state in joint_states] #  -> (8,)
		joint_pos = np.array(joint_positions)

		ee_state = p.getLinkState(self.robotID, self.eefID)
		ee_pos = np.array(ee_state[0])
		ee_ori = ee_state[1]

		obs = np.concatenate([joint_pos, ee_pos])
		return obs

    def generate_random_angle(self):
        theta = np.zeros(self.NrofJoints)
        theta[0] = random.uniform(self.joint_min[0], self.joint_max[0])
        theta[1] = random.uniform(self.joint_min[1], self.joint_max[1])
        theta[2] = random.uniform(self.joint_min[2], self.joint_max[2])
        theta[3] = random.uniform(self.joint_min[3], self.joint_max[3])
        theta[4] = random.uniform(self.joint_min[4], self.joint_max[4])
        theta[5] = random.uniform(self.joint_min[5], self.joint_max[5])
        return theta

    def generate_random_point(self):
    	pos = np.zeros(7)
        pos[0] = random.uniform(self.x_min, self.x_max)
        pos[1] = random.uniform(self.y_min, self.y_max)
        pos[2] = random.uniform(self.z_min, self.z_max)
        pos[3] = 0
        pos[4] = 0.70707
        pos[5] = 0
        pos[6] = 0.70714
        return pos

	def reset(self):
		print('reset')
		self.home_pose()
		self.state = self.get_state()
		self.done = False
		return self.state

	def move(self, action):
		self.theta[0] = np.interp(10, (-1, 1), (self.joint_min[0], self.joint_max[0]))
		self.theta[1] = np.interp(10, (-1, 1), (self.joint_min[0], self.joint_max[0]))
		self.theta[2] = np.interp(10, (-1, 1), (self.joint_min[0], self.joint_max[0]))
		self.theta[3] = np.interp(10, (-1, 1), (self.joint_min[0], self.joint_max[0]))
		self.theta[4] = np.interp(10, (-1, 1), (self.joint_min[0], self.joint_max[0]))
		self.theta[5] = np.interp(10, (-1, 1), (self.joint_min[0], self.joint_max[0]))




