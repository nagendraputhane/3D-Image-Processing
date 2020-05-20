import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def compare():
	datapd = pd.read_csv('/home/iq9/nagendra/delay/points_to_track.csv')
	edge_points = datapd.to_numpy()

	plt.subplot(2, 2, 1)
	plt.title('Middle-point along width')
	plt.plot(edge_points[:, 0], label = 'x')
	plt.plot(edge_points[:, 1], label = 'y' )
	plt.yticks(np.arange(0, 400, 50))
	plt.legend()

	plt.subplot(2, 2, 2)
	plt.title('Middle-point along height')
	plt.plot(edge_points[:, 2], label = 'x')
	plt.plot(edge_points[:, 3], label = 'y' )
	plt.yticks(np.arange(0, 400, 50))
	plt.legend()

	plt.subplot(2, 2, 3)
	datapd = pd.read_csv('/home/iq9/nagendra/delay/IMU_and_Image_data/Vertical/20-02-19 11:35:41/data_lin_acc.csv')
	lin_acc = datapd.to_numpy()

	plt.title('Accelerometer (Linear)')
	plt.plot(lin_acc[:, 1], label = 'x')
	plt.plot(lin_acc[:, 2], label = 'y' )
	plt.plot(lin_acc[:, 3], label = 'z')
	plt.yticks(np.arange(-0.4, 0.4, 0.05))
	plt.legend()

	plt.subplot(2, 2, 4)
	plt.title('Clubbed data')
	plt.plot(edge_points[:, 0], label = 'x along width')
	plt.plot(edge_points[:, 1], label = 'y along width' )
	plt.plot(edge_points[:, 2], label = 'x along height')
	plt.plot(edge_points[:, 3], label = 'y along height' )
	plt.plot(lin_acc[:1200, 1], label = 'lin_acc x')
	plt.plot(lin_acc[:1200, 2], label = 'lin_acc y' )
	plt.plot(lin_acc[:1200, 3], label = 'lin_acc z')
	plt.xticks(np.arange(0, 1200, 100))
	plt.yticks(np.arange(0, 400, 50))
	plt.legend()

	plt.show()

	plt.title('Clubbed data')
	plt.plot((edge_points[:, 0]/np.linalg.norm(edge_points[:, 0])), label = 'x along width')
	plt.plot(edge_points[:, 1]/np.linalg.norm(edge_points[:, 1]), label = 'y along width' )
	plt.plot(edge_points[:, 2]/np.linalg.norm(edge_points[:, 2]), label = 'x along height')
	plt.plot(edge_points[:, 3]/np.linalg.norm(edge_points[:, 3]), label = 'y along height' )
	plt.plot(lin_acc[:1200, 1]/np.linalg.norm(lin_acc[:1200, 1]), label = 'lin_acc x')
	plt.plot(lin_acc[:1200, 2]/np.linalg.norm(lin_acc[:1200, 2]), label = 'lin_acc y' )
	plt.plot(lin_acc[:1200, 3]/np.linalg.norm(lin_acc[:1200, 3]), label = 'lin_acc z')
	plt.xticks(np.arange(0, 1200, 100))
	plt.yticks(np.arange(-0.15, 0.15, 0.01))
	plt.legend()

	plt.show()

def plot_corners():
	datapd = pd.read_csv('/home/iq9/nagendra/delay/dataset/corners.csv')
	datanp = datapd.to_numpy()

	plt.title('Top Left')
	plt.plot(datanp[:, 0], label = 'x')
	plt.plot(datanp[:, 1], label = 'y' )
	plt.yticks(np.arange(0, 800, 50))
	plt.legend()
	plt.show()

	plt.title('Bottom Left')
	plt.plot(datanp[:, 2], label = 'x')
	plt.plot(datanp[:, 3], label = 'y' )
	plt.yticks(np.arange(0, 800, 50))
	plt.legend()
	plt.show()

	plt.title('Top Right')
	plt.plot(datanp[:, 4], label = 'x')
	plt.plot(datanp[:, 5], label = 'y' )
	plt.yticks(np.arange(0, 1400, 100))
	plt.legend()
	plt.show()

	plt.title('Bottom Right')
	plt.plot(datanp[:, 6], label = 'x')
	plt.plot(datanp[:, 7], label = 'y' )
	plt.yticks(np.arange(0, 1400, 100))
	plt.legend()
	plt.show()

def plot_edge():
	datapd = pd.read_csv('/home/iq9/nagendra/delay/points_to_track.csv')
	datanp = datapd.to_numpy()

	plt.title('Middle-point along width')
	plt.plot(datanp[:, 0], label = 'x')
	plt.plot(datanp[:, 1], label = 'y' )
	plt.yticks(np.arange(0, 400, 50))
	plt.legend()
	plt.show()

	plt.title('Middle-point along height')
	plt.plot(datanp[:, 2], label = 'x')
	plt.plot(datanp[:, 3], label = 'y' )
	plt.yticks(np.arange(0, 400, 50))
	plt.legend()
	plt.show()

def plot_imu():
	#####

	datapd = pd.read_csv('/home/iq9/1_nagendra/1_time_stamp_testing/2_dataset/android/Y/7_T_Y_P_Y/data_acc.csv')
	datanp = datapd.to_numpy()

	plt.title('Accelerometer')
	plt.plot((datanp[:, 1]), label = 'x')
	plt.plot((datanp[:, 2]), label = 'y' )
	plt.plot((datanp[:, 3]), label = 'z')
	plt.yticks(np.arange(-2, 13, 0.5))
	plt.legend()
	plt.show()

	#####
	datapd = pd.read_csv('/home/iq9/1_nagendra/1_time_stamp_testing/2_dataset/android/Y/7_T_Y_P_Y/data_lin_acc.csv')
	datanp = datapd.to_numpy()

	#plt.title('Gravity')
	#plt.plot((datanp[:, 4]*9.81), label = 'x')
	#plt.plot((datanp[:, 5]*9.81), label = 'y' )
	#plt.plot((datanp[:, 6]*9.81), label = 'z')
	#plt.yticks(np.arange(-4, 11, 0.5))
	#plt.legend()
	#plt.show()

	#####
	datapd = pd.read_csv('/home/iq9/1_nagendra/1_time_stamp_testing/2_dataset/android/Y/7_T_Y_P_Y/data_gyro.csv')
	datanp = datapd.to_numpy()

	#plt.subplot(3, 1, 1)
	plt.title('Gyroscope')
	plt.plot(datanp[:, 1], label = 'x')
	plt.plot(datanp[:, 2], label = 'y' )
	plt.plot(datanp[:, 3], label = 'z')
	plt.yticks(np.arange(-2, 2, 0.5))
	plt.legend()
	plt.show()

	#####
	datapd = pd.read_csv('/home/iq9/1_nagendra/1_time_stamp_testing/2_dataset/android/Y/7_T_Y_P_Y/data_lin_acc.csv')
	datanp = datapd.to_numpy()
	ax = datanp[:, 1]

	plt.title('Accelerometer (Linear)')
	plt.plot(datanp[:, 1], label = 'x')
	plt.plot(datanp[:, 2], label = 'y' )
	plt.plot(datanp[:, 3], label = 'z')
	plt.yticks(np.arange(-1.0, 1.0, 0.25))
	plt.legend()
	plt.show()

#compare()
#plot_corners()
#plot_edge()
plot_imu()
