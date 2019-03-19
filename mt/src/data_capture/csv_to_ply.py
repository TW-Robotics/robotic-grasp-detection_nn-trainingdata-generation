import numpy as np
import cv2

path = "/home/johannes/catkin_ws/src/mt/mt/src/data_capture/data/"
#path = "/home/mluser/catkin_ws/src/data/"

data = np.genfromtxt(str(path)+'0_0_d.csv', delimiter=';')

f = open(str(path) + "pc.ply", "w")
f.write("ply\nformat ascii 1.0\nelement vertex " + str(len(data)*(len(data[0])-1)) +"\nproperty float x\nproperty float y\nproperty float z\nend_header\n")
for row in range(len(data)):			#1280
	for col in range(len(data[0])-1):		#720
		f.write(str(float(row) / 1000.) + " " + str(float(col) / 1000.) + " " + str(float(data[row][col]) / 1000.) + "\n")
f.close()