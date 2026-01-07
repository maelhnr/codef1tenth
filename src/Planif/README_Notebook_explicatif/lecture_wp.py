import numpy as np
import csv
import matplotlib.pyplot as plt
print("tuple : ", (i for i in range(3)))

with open('waypoints.csv', 'r') as f:
    reader = csv.reader(f)
    data = list(reader)

data_array = np.array(data, dtype=float)
print("taille : ", len(data_array))
print(data_array.shape)

plt.scatter(data_array[:3000,0], data_array[:3000,1])
plt.show()
