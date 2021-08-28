import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

norm_end = [1, 1, 1] #x, y, z
norm_start = [0, 0, 0]

print(norm_start[2])
fig = plt.figure(figsize=(5, 5))
ax = fig.add_subplot(111, projection='3d')

ax.plot([norm_start[0],norm_end[0]], [norm_start[0],norm_end[0]],[norm_start[0],norm_end[0]])

ax.set_xlabel("x axis")
ax.set_ylabel("y axis")
ax.set_zlabel("z axis")
ax.set_xlim3d(0, 3)
ax.set_ylim3d(0, 3)
ax.set_zlim3d(0, 3)
plt.show()