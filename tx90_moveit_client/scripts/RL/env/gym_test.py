import gym
import numpy as np
from gym import spaces

min_action = -1.0
max_action = 1.0
action_space = spaces.Box(low=min_action, high=max_action, shape=(6,), dtype=np.float32)
print("action space ", action_space)
low  = np.deg2rad(np.array([-180,-130,-145,-270,-115,-270], dtype=np.float32))
high = np.deg2rad(np.array([180, 147.5,145,270,140,270], dtype=np.float32))
action_space = spaces.Box(low, high, dtype=np.float32)
print("action space ", action_space)

joint_low  = np.deg2rad(np.array([-180,-130,-145,-270,-115,-270], dtype=np.float32))
joint_high = np.deg2rad(np.array([180, 147.5,145,270,140,270], dtype=np.float32))

low = np.concatenate(low, joint_low)
high = np.concatenate(high, joint_high)

observation_space = spaces.Box(low,high, dtype=np.float32)