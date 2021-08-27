#!/usr/bin/env python2
import pptk
import numpy as np
from plyfile import PlyData, PlyElement
filepath = '/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/test_bed/pcd_data'
data = PlyData.read(filepath + '/profile.ply')['vertex']
print(data)
xyz = np.c_[data['x'], data['y'], data['z']]
# rgb = np.c_[data['red'], data['green'], data['blue']]
# n = np.c_[data['nx'], data['ny'], data['nz']]

v = pptk.viewer(xyz)
# v.attributes(rgb / 255., 0.5 * (1 + n))
