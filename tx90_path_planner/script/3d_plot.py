#!/usr/bin/env python2
import numpy as np
import math as m
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt


fig = plt.figure()
ax = plt.axes(projection='3d')

xline = [0, 0.01939]
yline = [0, 0.01784]
zline = [0, 0.99965]
ax.plot3D(xline, yline, zline, 'gray')
plt.show()