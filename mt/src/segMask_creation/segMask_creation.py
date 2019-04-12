#!/usr/bin/env python
import numpy as np
import sys
import rospy
import tf
import math
import os
import csv
import json
import re

from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ur5_control import ur5_control

bridge = CvBridge()

def main(args):
	img = cv2.imread("/home/johannes/git/mt/mt/src/segMask_creation/render_color.png")
	# Get green channel
	b, g, r = cv2.split(img)
	# Make all pixels to 255 which are >254 because they are background, and rest to 0
	val, img = cv2.threshold(g, 254, 255, cv2.THRESH_BINARY_INV)

	'''
	min = 1000
	for i in range(len(img)):
		for j in range(len(img[0])):
			if img[i][j] != 0 and img[i][j] != 255:
				min = img[i][j]
	print min'''

	dilation = cv2.erode(img, np.ones((5,5), np.uint8), iterations = 3)

	x,y,w,h = cv2.boundingRect(dilation)
	print x, y, w, h
	#cv2.rectangle(dilation,(x,y),(x+w,y+h),(0,255,0),2)

	cv2.imwrite("/home/johannes/git/mt/mt/src/segMask_creation/test1.png", img)
	cv2.imwrite("/home/johannes/git/mt/mt/src/segMask_creation/test1dilation.png", dilation)
	return


if __name__ == '__main__':
	main(sys.argv)
