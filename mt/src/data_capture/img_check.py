import numpy as np
import cv2
import rospy

img = cv2.imread('/home/mluser/catkin_ws/src/data/test12/1_0_d.png')
cv2.imshow("img", img)
print img[100][100]
cv2.waitKey(1)

data = np.genfromtxt('/home/mluser/catkin_ws/src/data/test12/1_0_d.csv', delimiter=';')
print data[100][100]
cv2.imwrite('/home/mluser/catkin_ws/src/data/test12/1_0_dFromCSV.png', data)