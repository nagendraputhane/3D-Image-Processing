import numpy as np
import pandas as pd
import matplotlib.pyplot as plt	

datapd = pd.read_csv('/home/iq9/Desktop/Accel.txt') #read linear acceleration vector
data1 = datapd.to_numpy() #[7] - x, [8] - y, [9] - z
data1 = data1[:, 0] #linear acceleration along Y axis

for i in range(0, (data1.shape[0]-1)):
	data1[i] = data1[i+1] - data1[i]
	print(data1[i])

plt.plot(data1[:data1.shape[0]-1], label = "Delta")
plt.grid()
plt.legend()
plt.show()
