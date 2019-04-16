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
import argparse
import glob
import cv2

globArgs = None

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
		#print int(cuboidProj[i][0]), int(cuboidProj[i][1])
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
	for i in range(len(cuboidProj)):
		cv2.putText(img, str(i), (cuboidProj[i][0] + offset, cuboidProj[i][1] + offset), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
	return img

def draw_boundingBox(img, boundingBox):
	 cv2.rectangle(img, tuple(boundingBox[0]), tuple(boundingBox[1]), (255, 0, 0), 1)

def conv_to_int(points):
	points = np.asarray(points)
	points = points.astype(int)

	'''for i in range(len(points)):
		for j in range(len(points[i])):
			points[i][j] = int(points[i][j])'''
	return points

def visualize(root):
	def create_data(pathToFiles):
		'''# Open json-file and extract camera-info
		cameraSettingsPath = pathToFiles + "/_camera_settings" + ".json"
		if os.path.exists(cameraSettingsPath):
			intrinsics = []
			with open(cameraSettingsPath) as json_file:
				data = json.load(json_file)
				settings = data["camera_settings"]
				intrinsics[0] = settings[0]["intrinsic_settings"]["fx"]
				intrinsics[0] = settings[0]["intrinsic_settings"]["fy"]
				intrinsics[0] = settings[0]["intrinsic_settings"]["cx"]
				intrinsics[0] = settings[0]["intrinsic_settings"]["cy"]
		else:
			print("File not found! Images in Folder not added! " + cameraSettingsPath)
			return'''

		# For each image in the folder
		for imgpath in glob.glob(pathToFiles+"/*.png"):
			# If there is a png and a json file
			if os.path.exists(imgpath) and os.path.exists(imgpath.replace("png","json")):
				fileName = os.path.splitext(os.path.basename(imgpath))[0]
				if globArgs.delete == False:
					# Open json-file, extract cuboid and bounding box
					with open(imgpath.replace("png","json")) as json_file:
						data = json.load(json_file)

						img = cv2.imread(imgpath)
						
						cuboidPoints = data["objects"][0]["projected_cuboid"]
						cuboidPoints.append(data["objects"][0]["projected_cuboid_centroid"])
						boundingBox = [data["objects"][0]["bounding_box"]["top_left"], data["objects"][0]["bounding_box"]["bottom_right"]]

						if globArgs.cuboids == True or globArgs.boundingBox == False:
							draw_cuboids(img, conv_to_int(cuboidPoints))
						if globArgs.boundingBox == True:
							draw_boundingBox(img, conv_to_int(boundingBox))

						cv2.imwrite(pathToFiles + "/" + fileName + ".vis.png", img)
				else:
					os.remove(pathToFiles + "/" + fileName + ".vis.png")

	def explore(pathToFiles):
		# Return if the path is no directory
		if not os.path.isdir(pathToFiles):
			return
		# Add all subfolders to variable
		folders = [os.path.join(pathToFiles, o) for o in os.listdir(pathToFiles) 
						if os.path.isdir(os.path.join(pathToFiles,o))]
		# If folder has a subfolder, explore this folder (or these folders)
		if len(folders) > 0:
			for path_entry in folders:
				explore(path_entry)
		# If folder has no subfolders, create render for each image in the folder
		else:
			create_data(pathToFiles)

	explore(root)


def main(args):
	global globArgs
	parser = argparse.ArgumentParser(description='Visualize Data')
	parser.add_argument('Path', metavar='Path', type=str,	help='Path to folder, where recorded data is stored')
	parser.add_argument('--delete', action="store_true", default=False, help='Delete .viz.png-Images')
	parser.add_argument('--boundingBox', action="store_true", default=False, help='Draw bounding box')
	parser.add_argument('--cuboids', action="store_true", default=False, help='Draw cuboid positions')

	globArgs = parser.parse_args()

	visualize(globArgs.Path)

	'''scaleFac = 720./400.
	imgWidth = int(round(1280./scaleFac, 0))
	horizontalStart = int(round(imgWidth/2., 0))-200
	img = cv2.resize(img, (imgWidth, 400), interpolation=cv2.INTER_AREA)
	#img = img[160:160+400, 440:440+400]
	img = img[0:400, horizontalStart:horizontalStart+400]
	print len(img)
	print len(img[0])
	intrinsics = [925.112183, 925.379517, 647.22644, 357.068359]
	intrinsics = [i/scaleFac for i in intrinsics]
	intrinsics[2] = intrinsics[2]-horizontalStart'''

	return


if __name__ == '__main__':
	main(sys.argv)
