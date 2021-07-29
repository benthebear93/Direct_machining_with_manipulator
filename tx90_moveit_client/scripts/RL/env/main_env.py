import gym
import numpy as np
from gym import spaces

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

		self.x_min = -0.3
		self.x_max = 1.2
		self.y_min = -1.0
		self.y_max = 1.0
		self.z_min = 0.3
		self.z_max = 1.3
		self.distance_threshold = 0.01

        self.min_action = -1.0
        self.max_action = 1.0
        self.joint_min = np.deg2rad(np.array([-180,-130,-145,-270,-115,-270], dtype=np.float32))
    	self.joint_max = np.deg2rad(np.array([180, 147.5,145,270,140,270], dtype=np.float32))
		# low  = np.deg2rad(np.array([-180,-130,-145,-270,-115,-270], dtype=np.float32))
		# high = np.deg2rad(np.array([180, 147.5,145,270,140,270], dtype=np.float32))

		coord_low = np.array([x_min, y_min, z_min], dtype=np.float32)
		coord_high = np.array([x_max, y_max, z_max], dtype=np.float32)

		low = np.concatenate(coord_low, self.joint_min)
		high = np.concatenate(coord_high, self.joint_max)

		self.action_space = spaces.Box(low=self.min_action, high=self.max_action, shape=(6,), dtype=np.float32)
		self.observation_space = spaces.Box(low, high, dtype=np.float32)


    def generate_random_angle(self):
        theta = np.zeros(self.NrofJoints)
        theta[0] = random.uniform(self.joint_min[0], self.joint_max[0])
        theta[1] = random.uniform(self.joint_min[1], self.joint_max[1])
        theta[2] = random.uniform(self.joint_min[2], self.joint_max[2])
        theta[3] = random.uniform(self.joint_min[3], self.joint_max[3])
        theta[4] = random.uniform(self.joint_min[4], self.joint_max[4])
        theta[5] = random.uniform(self.joint_min[5], self.joint_max[5])
        return theta

	def get_state(self):
		joint_states = p.getJointStates(self.robotID, range(p.getNumJoints(self.robotID)))
		joint_positions = [state[0] for state in joint_states] #  -> (8,)

		ee_state = p.getLinkState(self.robotID, self.eefID)
		ee_pos = ee_state[0]
		ee_ori = ee_state[1]

		print("joint state", joint_positions)

	def get_reward(self, goal):

		curr_pose = np.array(self.state[0])
		d = goal_distance(curr_pose, goal_pose)
		return d 

	def get_done(self):

		curr_pose = np.array(self.state[0])
		d = goal_distance(curr_pose, goal_pose)

		if self.time_step == self.max_steps:
			return True

		elif d < self.distance_threshold:
			return True

		else:
			return False

	def step(self, action, time_step):
		done = False
		prev_distance_to_goal = self.dis
		self.time_step = time_step

		if self.time_step == 0:
			print("Start training!")

		self.state = self.get_state()

		reward = self.get_reward()

		done = self.get_done()

		return self.state, rewar, done

		

	def reset(self):
		self.state = self.get_state()
		self.done = False





