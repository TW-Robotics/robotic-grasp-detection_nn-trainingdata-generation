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
import random
import shutil

globArgs = None
dataPointId = 0
destPathTrain = None
destPathTest = None

def createDatasets(root):
	def copyAndRename(pathToFiles):
		global globArgs
		global dataPointId
		global destPathTrain
		# For each image in the folder
		for imgpath in glob.glob(pathToFiles+"/*.png"):
			# If there is a png and a json file
			if os.path.exists(imgpath) and os.path.exists(imgpath.replace("png","json")):
				fileName = os.path.splitext(os.path.basename(imgpath))[0]
				shutil.copyfile(imgpath, destPathTrain + "/" + '{0:06d}'.format(dataPointId) + ".png")
				shutil.copyfile(pathToFiles + "/" + fileName + ".json", destPathTrain + "/" + '{0:06d}'.format(dataPointId) + ".json")
				dataPointId = dataPointId + 1

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
			copyAndRename(pathToFiles)

	def split(percentTestData):
		global dataPointId
		global destPathTrain
		global destPathTest
		#numDataPoints = dataPointId + 1

		numTestDataPoints = int(dataPointId * float(percentTestData)/100) #2958
		list = range(dataPointId)
		sampling = random.sample(list, k=numTestDataPoints)

		for id in sampling:
			shutil.move(destPathTrain + "/" + '{0:06d}'.format(id) + ".png", destPathTest + "/" + '{0:06d}'.format(id) + ".png")
			shutil.move(destPathTrain + "/" + '{0:06d}'.format(id) + ".json", destPathTest + "/" + '{0:06d}'.format(id) + ".json")

		return len(sampling)
	
	explore(root)
	return split(globArgs.percentTestData)

def main(args):
	global globArgs
	global destPathTrain
	global destPathTest
	global dataPointId
	parser = argparse.ArgumentParser(description='Dataset creator')
	parser.add_argument('Path', metavar='Path', type=str,	help='Path to folder, where recorded data is stored')
	parser.add_argument('--percentTestData', type=int, default=20, help='Percentage of dataset which should go to test-dataset. Default: 20')

	globArgs = parser.parse_args()
	path = os.path.dirname(os.path.dirname(globArgs.Path))	# Go one level up so this folder will not be searched
	destPathTrain = path + "/" + "dataset_train"
	destPathTest = path + "/" + "dataset_test"

	os.makedirs(destPathTrain)
	os.makedirs(destPathTest)

	numTestData = createDatasets(globArgs.Path)
	print "Number of total files: " + str(dataPointId)
	print "Number in training / test-dataset: " + str(numTestData) + " / " + str((dataPointId) - numTestData)
	print "Don't forget to copy a camera_settings.json and object_settings.json file!"

	return


if __name__ == '__main__':
	main(sys.argv)
