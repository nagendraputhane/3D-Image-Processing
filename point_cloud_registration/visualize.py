import pandas as pd
import numpy as np
import pptk
import plyfile

datapd = pd.read_csv('file.csv') #CSV to Pandas
datanp = datapd.to_numpy() #Pandas to NumPy
three = np.delete(datanp, 3, 1) #delete last column (axis 3 of 1[column])

datapdtwo = pd.read_csv('filetwo.csv') #CSV to Pandas
datanptwo = datapdtwo.to_numpy() #Pandas to NumPy
threetwo = np.delete(datanptwo, 6, 1) #delete last column (axis 3 of 1[column])
threetwo = np.delete(threetwo, 5, 1) #delete the normals
threetwo = np.delete(threetwo, 4, 1)
threetwo = np.delete(threetwo, 3, 1)
print(threetwo)

datapdthree = pd.read_csv('filethree.csv') #CSV to Pandas
datanpthree = datapdthree.to_numpy() #Pandas to NumPy
threethree = np.delete(datanpthree, 6, 1) #delete last column (axis 3 of 1[column])
threethree = np.delete(threethree, 5, 1) #delete the normals
threethree = np.delete(threethree, 4, 1)
threethree = np.delete(threethree, 3, 1)

xyz = np.concatenate((three, threetwo, threethree), axis=0) #xyz 

first_np = np.full((three.shape[0],3), [255., 0, 0])
second_np = np.full((threetwo.shape[0],3), [0, 255., 0])
third_np = np.full((threethree.shape[0],3), [0, 0, 255.])
rgb = np.concatenate((first_np, second_np, third_np), axis=0) #rgb 

print(three.shape[0])
v = pptk.viewer(xyz)
v.set(point_size=10)
v.attributes(rgb / 255.)





