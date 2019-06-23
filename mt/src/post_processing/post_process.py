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

# Write data to json-file (formatted)
def write_json(data, path):
	dump = json.dumps(data, sort_keys=False, indent=4)
	new_data = re.sub('\n +', lambda match: '\n' + '\t' * (len(match.group().strip('\n')) / 3), dump)
	print >> open(str(path), 'w'), new_data

def post_process(root):
	def create_data(pathToFiles):
		inLoop = False
		# Create mask and extract bounding box for each image in the folder
		for imgpath in glob.glob(pathToFiles+"/*.render.png"):
			inLoop = True
			fileName = os.path.splitext(os.path.splitext(os.path.basename(imgpath))[0])[0]
			jsonFilePath = pathToFiles + "/" + fileName + ".json"
			# If there is a .render.png and a .json file
			if os.path.exists(imgpath) and os.path.exists(jsonFilePath):
				# Open json-file and read content, to write it with bounding box back
				with open(jsonFilePath) as json_file:
					data = json.load(json_file)

					img = cv2.imread(imgpath)
					# Get green channel
					b, g, r = cv2.split(img)
					# Make all pixels to 255 which are >254 because they are background, and rest to 0
					val, img = cv2.threshold(g, 254, 255, cv2.THRESH_BINARY)

					'''
					min = 1000
					for i in range(len(img)):
						for j in range(len(img[0])):
							if img[i][j] != 0 and img[i][j] != 255:
								min = img[i][j]
					print min'''

					eroded = cv2.erode(img, np.ones((globArgs.kernel,globArgs.kernel), np.uint8), iterations = 3)

					x,y,w,h = cv2.boundingRect(~eroded)
					data["objects"][0]["bounding_box"]["top_left"] = [x, y]
					data["objects"][0]["bounding_box"]["bottom_right"] = [x+w, y+h]
					write_json(data, jsonFilePath)

					if globArgs.vis_contour:
						contoursImg = None
						# Visualisation of contour
						imgOrig = cv2.imread(pathToFiles + "/" + fileName + ".png")
						
						# Not Eroded
						ret, thresh = cv2.threshold(img, 127, 255, 0)
						img, contoursImg, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

						# Eroded
						ret, thresh = cv2.threshold(eroded, 127, 255, 0)
						img, contoursEroded, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

						if len(contoursImg) >= 2 and len(contoursEroded) >= 2:
							cv2.drawContours(imgOrig, contoursImg, 1, (255,0,0), 1)
							cv2.drawContours(imgOrig, contoursEroded, 1, (0,0,255), 1)

						eroded = imgOrig

					#print x, y, w, h
					#cv2.rectangle(eroded,(x,y),(x+w,y+h),(0,255,0),2)
					if globArgs.create_no_masks == False:
						cv2.imwrite(pathToFiles + "/" + fileName + ".segmentation.png", eroded)
					if globArgs.delete_render == True:
						os.remove(imgpath)
		if inLoop != True:
			print(".render.png-File not found")
			print(pathToFiles)

		if globArgs.delete_masks == True:
			for imgpath in glob.glob(pathToFiles+"/*.segmentation.png"):
				os.remove(imgpath)

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
	parser = argparse.ArgumentParser(description='Create Segmentation Mask and Calculate Bounding Box')
	parser.add_argument('Path', metavar='Path', type=str,	help='Path to folder, where recorded data is stored')
	parser.add_argument('--kernel', action="store", type=int, default=4, help='Kernal Size for mask-creation (default: 4)')
	parser.add_argument('--delete_render', action="store_true", default=False, help='Delete rendered images after processing')
	parser.add_argument('--delete_masks', action="store_true", default=False, help='Delete rendered masks')
	parser.add_argument('--create_no_masks', action="store_true", default=False, help='Create no segmentation masks')
	parser.add_argument('--vis_contour', action="store_true", default=False, help='Print contour on original image instead of segmentation mask')
	globArgs = parser.parse_args()

	if globArgs.kernel == 0:
		globArgs.kernel = 1
	path = globArgs.Path

	post_process(path)

	return


if __name__ == '__main__':
	main(sys.argv)
