import numpy as np
from matplotlib.patches import Polygon
from scipy.spatial import ConvexHull, convex_hull_plot_2d
import matplotlib.pyplot as plt
from matplotlib.path import Path

twodarray1 = np.zeros((10, 5))
twodarray2 = np.zeros((5, 10))
print(twodarray1)
print("   ")
test = []
for i in range(0,10):
	for j in range(0,5):
		test.append(5)

print(test)

plt.plot(twodarray1)
plt.show()