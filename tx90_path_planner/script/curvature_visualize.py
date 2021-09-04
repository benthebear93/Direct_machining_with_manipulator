#!/usr/bin/env python2
import pptk
import numpy as np
from plyfile import PlyData, PlyElement
import matplotlib.pyplot as plt
filepath = '/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/tx90_path_planner/pcd_data'
data = PlyData.read(filepath + '/new_cluster7.ply')['vertex']
data2 = PlyData.read(filepath + '/surface.ply')['vertex']

data = '/new_cluster7.ply'
data2 = '/surface2.ply'
class CurveChecker():
    def __init__(self, filename):
        self.data = PlyData.read(filepath + filename)['vertex']
        self.data_len = len(self.data)
        self.selected_x = 743
        self.total_data =[]

    def slice_line(self):
        y = []
        z = []
        for i in range(self.data_len):
            x_data = int(self.data[i][0]*1000) #same line
            if x_data == self.selected_x:
                yz = []
                yz.append(int(self.data[i][1]*1000))
                yz.append(int(self.data[i][2]*1000))
                self.total_data.append(yz)
                
        self.total_data = sorted(self.total_data)

        for i in range(len(self.total_data)):
            y.append(self.total_data[i][0])
            z.append(self.total_data[i][1])
        return y, z

Curve1 = CurveChecker(data)
y , z = Curve1.slice_line()

Curve2 = CurveChecker(data2)
y2, z2 = Curve2.slice_line()

for i in range(len(z2)):
    z2[i]-=92

fig, ax = plt.subplots()
ax.set_title("Depth camera vs Line scanner", fontsize=16)
ax.set_xlabel('y (mm)', fontsize = 15)
ax.set_ylabel('z (mm)', fontsize = 15)
ax.tick_params(axis="y", direction="in", which='major', labelsize=14)
ax.tick_params(axis="x", direction="in")
ax.plot(y,z)
ax.plot(y2,z2)
plt.show()