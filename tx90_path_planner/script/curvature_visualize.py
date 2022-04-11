#!/usr/bin/env python2
import pptk
import numpy as np
from plyfile import PlyData, PlyElement
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
filepath = '/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/tx90_path_planner/pcd_data'
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
    z2[i]-=92 #set offset

fig, ax = plt.subplots()
plt.setp(ax.spines.values(), linewidth=1.7)
ax.set_title("Depth camera vs Line scanner", fontsize=17)
ax.set_xlabel('y (mm)', fontsize = 16)
ax.set_ylabel('z (mm)', fontsize = 16)

ax.xaxis.set_minor_locator(ticker.AutoMinorLocator())
ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())

ax.xaxis.set_ticks_position('both')
ax.yaxis.set_ticks_position('both')
#ax.yaxis.set_tick_params(which='minor',direction="in")
ax.tick_params(axis="x", direction="in", which='major', labelsize=13, width=2)
ax.tick_params(axis="x", direction="in", which='minor', labelsize=13)

ax.tick_params(axis="y", direction="in", which='major', labelsize=13, width=2)
ax.tick_params(axis="y", direction="in", which='minor', labelsize=13)

ax.plot(y,z, label='Depth camera')
ax.plot(y2,z2, label='Line scanner')
plt.legend(loc='upper right', ncol=1)
plt.grid(True)
plt.show()