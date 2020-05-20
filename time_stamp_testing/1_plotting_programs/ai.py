import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def normalize(data):
	data = data - data.mean() #subtract the mean to centre it around 0
	data = data / data.max() #divide by max to scale it to [-1, 1]
	return data, 'Normalized'

def linAcc():
	datapd = pd.read_csv('/home/iq9/1_nagendra/1_time_stamp_testing/2_dataset/ios/6_T_X_P_Y_720/MotARH.csv') #read linear acceleration vector
	data1 = datapd.to_numpy() #[7] - x, [8] - y, [9] - z
	data1 = data1[:, 8] #linear acceleration along Y axis
	norma = 'Not Normalized'
	data1, norma = normalize(data1) #normalize the data around 0
	time_stamp_1 = datapd.to_numpy()
	time_stamp_1 = time_stamp_1[:, 0]
	time_stamp_1 = time_stamp_1 - time_stamp_1[0]
	return time_stamp_1, data1, 'Y axis Linear Accelerometer data', norma


def points():
	datapd = pd.read_csv('/home/iq9/1_nagendra/1_time_stamp_testing/2_dataset/ios/6_T_X_P_Y_720/results/points_to_track.csv') #read middle points of edges
	data2 = datapd.to_numpy()
	#Middle-point along width -> [0] - x , [1] - y; Middle-point along height -> [2] - x, [3] - y 
	data2 = data2[:, 0] #read X along height
	norma = 'Not Normalized'
	data2, norma = normalize(data2)
	datapd = pd.read_csv('/home/iq9/1_nagendra/1_time_stamp_testing/2_dataset/ios/6_T_X_P_Y_720/Frames.csv')
	time_stamp_2 = datapd.to_numpy()
	time_stamp_2 = time_stamp_2[:, 0]
	time_stamp_2 = time_stamp_2[:-1]
	time_stamp_2 = time_stamp_2 - time_stamp_2[0]
	return time_stamp_2, data2, 'Edge point along Y', norma

def gravAcc():
	datapd = pd.read_csv('/home/iq9/1_nagendra/1_time_stamp_testing/2_dataset/ios/6_T_X_P_Y_720/Accel.csv') #read accelerometer data
	data1 = datapd.to_numpy()
	data1 = data1[:, 2] #[1] - x, [2] - y, [3] - z
	datapd = pd.read_csv('/home/iq9/1_nagendra/1_time_stamp_testing/2_dataset/ios/6_T_X_P_Y_720/MotARH.csv') #read gravity vector
	grav = datapd.to_numpy()
	grav = grav[:, 5] #[4] - x, [5] - y, [6] - z
	data1 = np.subtract(data1, grav[1:]) #subtract gravity vector from accelerometer data
	norma = 'Not Normalized'
	data1, norma = normalize(grav)
	#Y axis -> Gravity data subtracted from Accelerometer data
	time_stamp_1 = datapd.to_numpy()
	time_stamp_1 = time_stamp_1[:, 0]
	time_stamp_1 = time_stamp_1 - time_stamp_1[0]
	return time_stamp_1, data1, 'Y axis -> Gravity data subtracted from Accelerometer data', norma

def acc():
	datapd = pd.read_csv('/home/iq9/1_nagendra/1_time_stamp_testing/2_dataset/ios/6_T_X_P_Y_720/Accel.csv') #read accelerometer data
	data1 = datapd.to_numpy()
	#data1[0:600, 2] = data1[600:4000,2].mean();
	#data1[4000:, 2] = data1[600:4000,2].mean();
	data1 = data1[:, 2] #[1] - x, [2] - y, [3] - z
	norma = 'Not Normalized'
	data1, norma = normalize(data1)
	time_stamp_1 = datapd.to_numpy()
	time_stamp_1 = time_stamp_1[:, 0]
	time_stamp_1 = time_stamp_1 - time_stamp_1[0]
	return time_stamp_1, data1, 'Y axis Accelerometer data', norma


time_stamp_1, data1, str1, norma = linAcc()
time_stamp_2, data2, str2, norma = points()

time_difference = time_stamp_2[-1] - time_stamp_2[0]
plt.plot(time_stamp_1, data1, label = str1)
plt.plot(time_stamp_2, data2, label = str2)
plt.xticks(np.arange(0, time_difference, 1))
plt.grid()
plt.legend()
plt.show()
