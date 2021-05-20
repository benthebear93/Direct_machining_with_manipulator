from sklearn.decomposition import PCA
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
#library imports
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree

class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)

x = 0.37885
y = -0.00052
z = 0.92545
a = Arrow3D([0, x], [0, y], [0, z], mutation_scale=20, lw=3, arrowstyle="-|>", color="r")
plt3d = plt.figure().gca(projection='3d')
plt3d.set_xlabel('$X$', fontsize=10)
plt3d.set_ylabel('$Y$', fontsize=10)
plt3d.set_zlabel('$Z$', fontsize=10)
plt3d.set_xlim(-1, 1)
plt3d.set_ylim(-1, 1)
plt3d.set_zlim(-1, 1)
plt3d.add_artist(a)
plt.show()