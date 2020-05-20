import numpy as np
import pandas as pd
import matplotlib.pyplot as plt	

def normalize(data):
	data = data - data.mean() #subtract the mean to centre it around 0
	data = data / data.max() #divide by max to scale it to [-1, 1]
	return data, 'Normalized'

datapd = pd.read_csv('/home/iq9/1_nagendra/1_time_stamp_testing/2_dataset/ios/6_T_X_P_Y_720/MotARH.csv') #read linear acceleration vector
data1 = datapd.to_numpy() #[7] - x, [8] - y, [9] - z
y1 = data1[:, 0]
y1 = y1 - y1[0]
data1 = data1[:, 8] #linear acceleration along Y axis
data1, norma = normalize(data1)

datapd = pd.read_csv('/home/iq9/1_nagendra/1_time_stamp_testing/2_dataset/ios/6_T_X_P_Y_720/results/points_to_track.csv') #read middle points of edges
data2 = datapd.to_numpy()
#Middle-point along width -> [0] - x , [1] - y; Middle-point along height -> [2] - x, [3] - y 
data2 = data2[:, 0] #read X along height
data2, norma = normalize(data2)

print(data1.shape)
print(data2.shape)

#y1 = np.arange(0, data1.shape[0], 1)
#y2 = np.arange(0, data2.shape[0], 1)
#y2 = y2 * 3

datapd = pd.read_csv('/home/iq9/1_nagendra/1_time_stamp_testing/2_dataset/ios/6_T_X_P_Y_720/Frames.csv') #read linear acceleration vector
y2 = datapd.to_numpy() #[7] - x, [8] - y, [9] - z
print(" y : ", y2[0])
y2 = y2 - y2[0]

plt.plot(y1, data1, label = "Linear Accelerometer data")
plt.plot(y2[:-1, 0], data2, label = "Edge points")
plt.grid()
plt.legend()
plt.show()
