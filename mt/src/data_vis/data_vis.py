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

def calculate_projection(points, intrinsics):
	cuboidProj = []
	for i in range(len(points)):
		p = [0, 0]
		p[0] = int(intrinsics[2] + 1/points[i][2] * (intrinsics[0] * points[i][0]))
		p[1] = int(intrinsics[3] + 1/points[i][2] * (intrinsics[1] * points[i][1]))
		cuboidProj.append(p)
	return cuboidProj

def draw_cuboids(img, cuboidProj):
	for i in range(len(cuboidProj)):
		cv2.circle(img, (cuboidProj[i][0], cuboidProj[i][1]), 5, (255, 0, 0), 5)
		print int(cuboidProj[i][0]), int(cuboidProj[i][1])
	linePoints = [[cuboidProj[0], cuboidProj[1]],
				  [cuboidProj[1], cuboidProj[2]],
				  [cuboidProj[2], cuboidProj[3]],
				  [cuboidProj[3], cuboidProj[0]],

				  [cuboidProj[4], cuboidProj[5]],
				  [cuboidProj[5], cuboidProj[6]],
				  [cuboidProj[6], cuboidProj[7]],
				  [cuboidProj[7], cuboidProj[4]],

				  [cuboidProj[0], cuboidProj[4]],
				  [cuboidProj[1], cuboidProj[5]],
				  [cuboidProj[2], cuboidProj[6]],
				  [cuboidProj[3], cuboidProj[7]],
				 ]
	for i in range(4):
		cv2.line(img, (linePoints[i][0][0], linePoints[i][0][1]), (linePoints[i][1][0], linePoints[i][1][1]), (0, 255, 0), 2)
	for i in range(4, 8):
		cv2.line(img, (linePoints[i][0][0], linePoints[i][0][1]), (linePoints[i][1][0], linePoints[i][1][1]), (0, 0, 255), 2)
	for i in range(8, 12):
		cv2.line(img, (linePoints[i][0][0], linePoints[i][0][1]), (linePoints[i][1][0], linePoints[i][1][1]), (0, 255, 255), 2)
	offset = 10
	for i in range(len(points)):
		cv2.putText(img, str(i), (cuboidProj[i][0] + offset, cuboidProj[i][1] + offset), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
	return img

def main(args):
	img = cv2.imread("/home/johannes/git/3_0_rgb.png")
	scaleFac = 720./400.
	imgWidth = int(round(1280./scaleFac, 0))
	horizontalStart = int(round(imgWidth/2., 0))-200
	img = cv2.resize(img, (imgWidth, 400), interpolation=cv2.INTER_AREA)
	#img = img[160:160+400, 440:440+400]
	img = img[0:400, horizontalStart:horizontalStart+400]
	print len(img)
	print len(img[0])
	intrinsics = [925.112183, 925.379517, 647.22644, 357.068359]
	intrinsics = [i/scaleFac for i in intrinsics]

	intrinsics[2] = intrinsics[2]-horizontalStart

	points = [[0.289130581217, 0.010806052175, 0.585754207636],
			  [0.391352787228, 0.153213179645, 0.677813029697],
			  [0.362503297494, 0.213126918093, 0.617166100043], 
			  [0.260281091484, 0.070719790623, 0.525107277983],
			  [0.148562148174, 0.046003555756, 0.687394004156],
			  [0.250784354184, 0.188410683226, 0.779452826216],
			  [0.221934864451, 0.248324421674, 0.718805896563],
			  [0.119712658440, 0.105917294204, 0.626747074503],
			  [0.255532722834, 0.129565236925, 0.652280052100],
			  ]
	cuboidProj = calculate_projection(points, intrinsics)
	draw_cuboids(img, cuboidProj)

	cv2.imwrite("/home/johannes/git/test1.png", img)
	return


if __name__ == '__main__':
	main(sys.argv)
