#!/usr/bin/env python
import numpy as np
import sys
import rospy
import tf
import math
import os
import csv
import json
import cv2
import random
import argparse
import glob
import matplotlib.pyplot as plt
import tf

globArgs = None
dist_z = []
roll = []
pitch = []
yaw = []
centerx = []
centery = []

def plot_hist(values, title, bins, range):
	plt.hist(values, bins=bins, range=range)
	plt.xlim(range[0],range[1])
	plt.title(title)
	plt.xlabel("Value")
	plt.ylabel("Count")
#	plt.show()

def plot_heatmap(centerx, centery):
	#fig2 = plt.figure()
	# Range to make empty areas same color as background
	plt.hist2d(centerx, centery, bins=150, range=np.array([(0, 400), (0, 400)]))
	plt.xlabel('x')
	plt.ylabel('y')
	cbar = plt.colorbar()
	cbar.ax.set_ylabel('Counts')
	plt.axis([0, 400, 0, 400])
#	plt.show()

def visualize(root):
	def add_data(pathToFiles):
		global dist_z
		global roll
		global pitch
		global yaw
		global centerx
		global centery

		for filePath in glob.glob(pathToFiles + "/*.json"):
			if os.path.exists(filePath) and os.path.exists(filePath.replace("json","png")):
				fileName = os.path.splitext(os.path.basename(filePath))[0]
				with open(filePath.replace("png","json")) as json_file:
					data = json.load(json_file)
					pos = data["objects"][0]["location"]
					ori = data["objects"][0]["quaternion_xyzw"]
					euler = tf.transformations.euler_from_quaternion(ori, axes='sxyz')	#TODO Achsreihenfolge?!
					#points2d = data["objects"][0]["projected_cuboid"]
					#points3d = data["objects"][0]["cuboid"]
					centroid_point2d = data["objects"][0]["projected_cuboid_centroid"]
					#centroid_point3d = data["objects"][0]["cuboid_centroid"]
					#points2d.append(data["objects"][0]["projected_cuboid_centroid"])
					#points3d.append(data["objects"][0]["cuboid_centroid"])
					if pos[2] < 40:
						print "Problem with distance of:"
						print fileName
					if centroid_point2d[0] > 400 or centroid_point2d[1] > 400 or centroid_point2d[0] < 0 or centroid_point2d[1] < 0:
						print "Problem with centroid of:"
						print fileName
						print centroid_point2d

				roll.append(euler[0]*180/math.pi)
				pitch.append(euler[1]*180/math.pi)
				yaw.append(euler[2]*180/math.pi)
				dist_z.append(pos[2])
				centerx.append(centroid_point2d[0])
				centery.append(centroid_point2d[1])

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
			print pathToFiles
			add_data(pathToFiles)

	explore(root)

def main(args):
	global globArgs
	global dist_z
	global roll
	global pitch
	global yaw
	global centerx
	global centery

	parser = argparse.ArgumentParser(description='Visualize Distribution of Data')
	parser.add_argument('Path', metavar='Path', type=str, help='Path to folder, where recorded data is stored')
	globArgs = parser.parse_args()

	visualize(globArgs.Path)
	print "Number of Datapoints: " + str(len(roll))
	if len(roll) != 0:
		plt.subplot(3, 2, 1)
		plot_hist(roll, "Roll", 35, [-180, 180])
		plt.subplot(3, 2, 3)
		plot_hist(pitch, "Pitch", 35, [-180, 180])
		plt.subplot(3, 2, 5)
		plot_hist(yaw, "Yaw", 35, [-180, 180])
		plt.subplot(3, 2, 2)
		plot_hist(dist_z, "Distance z", 20, [min(dist_z), max(dist_z)]) #[min(dist_z), max(dist_z)]
		plt.subplot(3, 2, 4)
		plot_heatmap(centerx, centery)
		plt.show()

	return


if __name__ == '__main__':
	main(sys.argv)
