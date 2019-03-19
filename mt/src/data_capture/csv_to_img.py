import numpy as np
import cv2
data = np.genfromtxt('text1.csv', delimiter=';')
cv2.imwrite('test1.png', data)