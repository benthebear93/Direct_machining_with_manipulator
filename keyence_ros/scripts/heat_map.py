import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sb
import numpy as np

data = [[3.244,-1.354,1.345,-6.431], [6.452,3.678,2.41,-5.743]]
data = np.array(data)
print(data)
heat_map = sb.heatmap(data)
plt.show()