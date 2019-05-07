#!/usr/bin/env python

# Copyright (c) 2018 NVIDIA Corporation. All rights reserved.
# This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
# https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode

"""
This file takes all test-images from a folder, runs them through dope
and compares the output of dope with the ground-truth.
"""

import sys
import yaml
import os
import csv

import numpy as np
import cv2

from PIL import Image
from PIL import ImageDraw

# Import DOPE code
sys.path.append('../inference')
from cuboid import *
from detector import *

### Global Variables
g_draw = None


### Code to visualize the neural network output
def DrawLine(point1, point2, lineColor, lineWidth):
	'''Draws line on image'''
	global g_draw
	if not point1 is None and point2 is not None:
		g_draw.line([point1,point2], fill=lineColor, width=lineWidth)

def DrawDot(point, pointColor, pointRadius):
	'''Draws dot (filled circle) on image'''
	global g_draw
	if point is not None:
		xy = [
			point[0]-pointRadius, 
			point[1]-pointRadius, 
			point[0]+pointRadius, 
			point[1]+pointRadius
		]
		g_draw.ellipse(xy, 
			fill=pointColor, 
			outline=pointColor
		)

def DrawCube(points, color=(255, 0, 0)):
	'''
	Draws cube with a thick solid line across 
	the front top edge and an X on the top face.
	'''

	lineWidthForDrawing = 2

	# draw front
	DrawLine(points[0], points[1], color, lineWidthForDrawing)
	DrawLine(points[1], points[2], color, lineWidthForDrawing)
	DrawLine(points[3], points[2], color, lineWidthForDrawing)
	DrawLine(points[3], points[0], color, lineWidthForDrawing)
	
	# draw back
	DrawLine(points[4], points[5], color, lineWidthForDrawing)
	DrawLine(points[6], points[5], color, lineWidthForDrawing)
	DrawLine(points[6], points[7], color, lineWidthForDrawing)
	DrawLine(points[4], points[7], color, lineWidthForDrawing)
	
	# draw sides
	DrawLine(points[0], points[4], color, lineWidthForDrawing)
	DrawLine(points[7], points[3], color, lineWidthForDrawing)
	DrawLine(points[5], points[1], color, lineWidthForDrawing)
	DrawLine(points[2], points[6], color, lineWidthForDrawing)

	# draw dots
	DrawDot(points[0], pointColor=color, pointRadius = 4)
	DrawDot(points[1], pointColor=color, pointRadius = 4)

	# draw x on the top 
	DrawLine(points[0], points[5], color, lineWidthForDrawing)
	DrawLine(points[1], points[4], color, lineWidthForDrawing)


def calc_distance(est_points, gt_points):
	dist = []
	for i in range(len(est_points)):
		squared_dist = np.sum(est_points[i]**2 + gt_points[i]**2, axis=0)
		dist[i] = np.sqrt(squared_dist)
	return dist, sum(dist)/len(dist)

def test_dope(params, testDataFolder):
	global g_draw

	pubs = {}
	models = {}
	pnp_solvers = {}
	pub_dimension = {}
	draw_colors = {}

	# Initialize parameters
	matrix_draw_colorscamera = np.zeros((3,3))
	matrix_camera = np.zeros((3,3))
	matrix_camera[0,0] = params["camera_settings"]['fx']
	matrix_camera[1,1] = params["camera_settings"]['fy']
	matrix_camera[0,2] = params["camera_settings"]['cx']
	matrix_camera[1,2] = params["camera_settings"]['cy']
	matrix_camera[2,2] = 1
	dist_coeffs = np.zeros((4,1))

	if "dist_coeffs" in params["camera_settings"]:
		dist_coeffs = np.array(params["camera_settings"]['dist_coeffs'])
	config_detect = lambda: None
	config_detect.mask_edges = 1
	config_detect.mask_faces = 1
	config_detect.vertex = 1
	config_detect.threshold = 0.5
	config_detect.softmax = 1000
	config_detect.thresh_angle = params['thresh_angle']
	config_detect.thresh_map = params['thresh_map']
	config_detect.sigma = params['sigma']
	config_detect.thresh_points = params["thresh_points"]

	# For each object to detect, load network model and create PNP solver
	for model in params['weights']:
		models[model] = ModelData(model, "../../weights/" + params['weights'][model])
		models[model].load_net_model()
		
		draw_colors[model] = tuple(params["draw_colors"][model])
		pnp_solvers[model] = CuboidPNPSolver(model,	matrix_camera, Cuboid3d(params['dimensions'][model]), dist_coeffs=dist_coeffs)

	print ("Testing DOPE...")

	# For all images in folder
	for imgpath in glob.glob(params['path_to_images'] + "/*.png"):
		if os.path.exists(imgpath) and o12642s.path.exists(imgpath.replace("png","json")):
			fileName = os.path.splitext(os.path.basename(imgpath))[0]
			# Load image and json-file
			with open(imgpath.replace("png","json")) as json_file:
				data = json.load(json_file)
				img = cv2.imread(imgpath)
				
				gt_pos = data["objects"][0]["location"]
				gt_ori = data["objects"][0]["quaternion_xyzw"]
				gt_points2d = data["objects"][0]["projected_cuboid"]
				gt_points3d = data["objects"][0]["cuboid"]
				gt_centroid_point2d = data["objects"][0]["projected_cuboid_centroid"]
				gt_centroid_point3d = data["objects"][0]["cuboid_centroid"]
				gt_points2d.append(data["objects"][0]["projected_cuboid_centroid"])
				gt_points3d.append(data["objects"][0]["cuboid_centroid"])

			# Copy and draw image
			img_copy = img.copy()
			im = Image.fromarray(img_copy)
			g_draw = ImageDraw.Draw(im)

			filenames = []
			dists3d = []
			dists2d = []
			meanDists3d = []
			meanDists2d = []

			for m in models:
				# Detect object
				results = ObjectDetector.detect_object_in_image(models[m].net, pnp_solvers[m], img, config_detect)
				
				# Get pose and overlay cube on image
				for i_r, result in enumerate(results):
					if result["location"] is None:
						continue
					loc = result["location"]
					ori = result["quaternion"]

					# Draw the cube
					if None not in result['projected_points']:
						points2d = []
						points3d = []
						for pair in result['projected_points']:
							points2d.append(tuple(pair))
						for pair in results['points_3d']:
							points3d.append(tuple(pair))
						DrawCube(points2d, draw_colors[m])
				
				dist3d, meanDist3d = calc_distance(points3d, gt_points3d)
				dist2d, meanDist2d = calc_distance(points2d, gt_points2d)
				dists3d.append(dist3d)
				dists2d.append(dist2d)
				meanDists3d.append(meanDist3d)
				meanDists2d.append(meanDist2d)
				filenames.append(fileName)

			# Store image with results overlaid
			cv2.imwrite(testDataFolder + "/" + fileName + ".vis.png", img)

	# filename; meanDist3d; meanDist2d; dist3D; dist2D
	with open(testDataFolder + "/evaluation.csv", "wb") as f:
		writer = csv.writer(f, delimiter=";")
		writer.writerow("filename; meanDist3d; meanDist2d; dist3D0; dist3D1; dist3D2; dist3D3; dist3D4; dist3D5; dist3D6; dist3D7; dist3Dcentroid; dist2D0; dist2D1; dist2D2; dist2D3; dist2D4; dist2D5; dist2D6; dist2D7; dist2Dcentroid;")
		for i in range(len(filenames)):
			line = str(filenames[i]) + " ;" + str(meanDist3d) + " ;" + str(meanDist2d) + " ;"
			line = line + str(dist3d[0]) + " ;" + str(dist3d[1]) + " ;" + str(dist3d[2]) + " ;" + str(dist3d[3]) + " ;" + str(dist3d[4]) + " ;" + str(dist3d[5]) + " ;" + str(dist3d[6]) + " ;" + str(dist3d[7]) + " ;" + str(dist3d[8]) + " ;"
			line = line + str(dist2d[0]) + " ;" + str(dist2d[1]) + " ;" + str(dist2d[2]) + " ;" + str(dist2d[3]) + " ;" + str(dist2d[4]) + " ;" + str(dist2d[5]) + " ;" + str(dist2d[6]) + " ;" + str(dist2d[7]) + " ;" + str(dist2d[8]) + " ;"
			writer.writerow(line)

if __name__ == "__main__":
	'''Main routine to run DOPE test-code'''

	if len(sys.argv) > 1:
		config_name = sys.argv[1]
	else:
		config_name = "config_pose_test.yaml"
	params = None
	yaml_path = '../../config/{}'.format(config_name)
	with open(yaml_path, 'r') as stream:
		try:
			print("Loading DOPE parameters from '{}'...".format(yaml_path))
			params = yaml.load(stream)
			print('    Parameters loaded.')
		except yaml.YAMLError as exc:
			print(exc)

	testDataFolder = params['path_to_images'] + "/" + "test_data"
	if not os.path.exists(testDataFolder):
		os.makedirs(testDataFolder)

	test_dope(params, testDataFolder)
