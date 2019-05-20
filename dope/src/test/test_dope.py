#!/usr/bin/env python

# Copyright (c) 2018 NVIDIA Corporation. All rights reserved.
# This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
# https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode

"""
This file takes all test-images from a folder, runs them through dope
and compares the output of dope with the ground-truth.
"""

import time
import sys
import yaml
import os
import csv
import glob

import numpy as np
import math
import cv2

from PIL import Image
from PIL import ImageDraw
from cv_bridge import CvBridge, CvBridgeError

# Import DOPE code
sys.path.append('../inference')
from cuboid import *
from detector import *
from pyquaternion import Quaternion

import tf

### Global Variables
g_draw = None

###########################################################
# VISUALISATION ###########################################
###########################################################

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


###########################################################
# EVALUATION ###########################################
###########################################################

def calc_distance(est_points, gt_points):
	dist = []
	for i in range(len(est_points)):
		try:
			if len(est_points[i]) == 3:
				dist.append(math.sqrt(  math.pow(est_points[i][0] - gt_points[i][0], 2) +
										math.pow(est_points[i][1] - gt_points[i][1], 2) +
										math.pow(est_points[i][2] - gt_points[i][2], 2)* 1.0))
		except:
			print "Error when calculating distance!"
			return [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 0, 0, 0

		try:
			if len(est_points[i]) == 2:
				dist.append(math.sqrt(  math.pow(est_points[i][0] - gt_points[i][0], 2) +
										math.pow(est_points[i][1] - gt_points[i][1], 2)* 1.0))
		except:
			print "Error when calculating distance!"
			return [0, 0, 0, 0, 0, 0, 0, 0, 0], 0, 0, 0

	return dist, sum(dist)/len(dist), min(dist), max(dist)

def test_dope(params, testDataFolder):
	global g_draw

	models = {}
	pnp_solvers = {}
	draw_colors = {}

	# Initialize parameters
	# Camera-Matrix for test-images
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

	cuboid_points_3d = np.array([[ 9.9,-4.5, 8.85, 1],
								[-9.9,-4.5, 8.85, 1],
								[-9.9, 4.5, 8.85, 1],
								[ 9.9, 4.5, 8.85, 1],
								[ 9.9,-4.5,-8.85, 1],
								[-9.9,-4.5,-8.85, 1],
								[-9.9, 4.5,-8.85, 1],
								[ 9.9, 4.5,-8.85, 1],
								[ 0. , 0. , 0.  , 1],   # Centroid
								[ 0. , 2.5, 7.527, 1]]) # Grasp-Point

	# For each object to detect, load network model and create PNP solver
	for model in params['weights']:
		models[model] = ModelData(model, "../../weights/" + params['weights'][model])
		models[model].load_net_model()
		
		draw_colors[model] = tuple(params["draw_colors"][model])
		pnp_solvers[model] = CuboidPNPSolver(model,	matrix_camera, Cuboid3d(params['dimensions'][model]), dist_coeffs=dist_coeffs)

		# Init pnp-Solver for ground-truth pose calculation
		pnp_solvers["gt"] = CuboidPNPSolver(model, matrix_camera, Cuboid3d(params['dimensions'][model]), dist_coeffs=dist_coeffs)

	print ("Testing DOPE...")

	model_testing_start_time = time.time()

	# Init variables to store results
	filenamesSuccess = []
	filenamesFailure = []
	dists3d = []
	dists2d = []
	meanDists3d = []
	meanDists2d = []
	locs = []
	oris = []
	rots = []
	gt_locs = []
	gt_oris = []
	gt_rots = []

	# For all images in folder
	print "Analysing images in folder " + str(params['path_to_images'])
	for imgpath in glob.glob(params['path_to_images'] + "/*.png"):
		if os.path.exists(imgpath) and os.path.exists(imgpath.replace("png","json")):
			fileName = os.path.splitext(os.path.basename(imgpath))[0]
			
			img = cv2.imread(imgpath)
			
			# Copy and draw image
			img_copy = img.copy()
			im = Image.fromarray(img_copy)
			g_draw = ImageDraw.Draw(im)

			for m in models:
				# Detect object
				results = ObjectDetector.detect_object_in_image(models[m].net, pnp_solvers[m], img, config_detect)

				# If no object could be detected copy file to failed-folder
				if len(results) == 0:
					cv2.imwrite(testDataFolder + "/" + "fail" + "/" + fileName + ".failed.png", np.array(im)[...,::-1])
					filenamesFailure.append(fileName)

				# Get pose and overlay cube on image
				for i_r, result in enumerate(results):
					if result["location"] is None:
						continue
					loc = result["location"]
					ori = result["quaternion"]

					# Get homogenous transformation matrix and put translation in last column
					M_trans = tf.transformations.quaternion_matrix(ori)
					M_trans[0][3] = loc[0]
					M_trans[1][3] = loc[1]
					M_trans[2][3] = loc[2]

					# Transform cuboid-corners and centroid and store them
					cuboid_points_3d_transfrom = []
					for point in cuboid_points_3d:
						cuboid_points_3d_transfrom.append(M_trans.dot(point))
					cuboid_points_3d_transfrom = np.delete(cuboid_points_3d_transfrom, 3, 1)	# Delete 1s from homogenous transformation

					# Draw the cube
					if None not in result['projected_points']:
						points2d = []
						points3d = []
						for pair in result['projected_points']:
							points2d.append(tuple(pair))
						DrawCube(points2d, draw_colors[m])

					# Load json-file and get ground-truth data
					with open(imgpath.replace("png","json")) as json_file:
						data = json.load(json_file)
						
						gt_loc = data["objects"][0]["location"]
						gt_ori = data["objects"][0]["quaternion_xyzw"]
						gt_points2d = data["objects"][0]["projected_cuboid"]
						gt_points3d = data["objects"][0]["cuboid"]
						gt_points2d.append(data["objects"][0]["projected_cuboid_centroid"])
						gt_points3d.append(data["objects"][0]["cuboid_centroid"])

					# Calculate ground-truth quaternions from pnp-solver
					gt_result = pnp_solvers["gt"].solve_pnp(gt_points2d)
					gt_ori = gt_result[1]
					gt_loc = gt_result[0]	# Location berechnen weil gt_loc falscher Bezug ist (object-coordinate system ist auf Base und nicht Cuboid)

					# Calculate grasp-point according to pnp-solver quaternions
					M_trans = tf.transformations.quaternion_matrix(gt_ori)
					M_trans[0][3] = gt_loc[0]
					M_trans[1][3] = gt_loc[1]
					M_trans[2][3] = gt_loc[2]

					gt_grasp_point = M_trans.dot(cuboid_points_3d[9])
					gt_grasp_point = gt_grasp_point[:-1]	# Delete 1 from homogenous transformation
					gt_points3d.append(gt_grasp_point)

					# Calculate distance between ground truth and estimation
					dist3d, meanDist3d, minDist3d, maxDist3d = calc_distance(cuboid_points_3d_transfrom, np.array(gt_points3d))
					dist2d, meanDist2d, minDist2d, maxDist2d = calc_distance(points2d, np.array(gt_points2d))

					# Transform rotations into euler angles
					rot = tf.transformations.euler_from_quaternion(ori)	# TODO Achsreihenfolge?!
					rot = [rot[i] / math.pi*180 for i in range(len(rot))]
					gt_rot = tf.transformations.euler_from_quaternion(gt_ori)#, axes='szyx')
					gt_rot = [gt_rot[i] / math.pi*180 for i in range(len(gt_rot))]

					# Store all calculated results in correct format in arrays
					locs.append([loc[i] * 10 for i in range(len(loc))])
					oris.append(ori)
					rots.append(rot)

					gt_locs.append([gt_loc[i] * 10 for i in range(len(gt_loc))])
					gt_oris.append(gt_ori)
					gt_rots.append(gt_rot)

					dists3d.append([dist3d[i] * 10 for i in range(len(dist3d))])
					dists2d.append(dist2d)
					meanDists3d.append(meanDist3d * 10)
					meanDists2d.append(meanDist2d)
					filenamesSuccess.append(fileName)

					# Store image with results overlaid
					cv2.imwrite(testDataFolder + "/" + fileName + ".vis.png", np.array(im)[...,::-1])
	
	# Write results to file
	with open(testDataFolder + "/evaluation.csv", "wb") as f: #q1; q2; q3; q4; gtQ1; gtQ2; gtQ3; gtQ4;
		f.write("filename; success; failure; locX; locY; locZ; gtLocX; gtLocY; gtLocZ; rx; ry; rz; gtRx; gtRy; gtRz; meanDist3d; meanDist2d; dist3D0; dist3D1; dist3D2; dist3D3; dist3D4; dist3D5; dist3D6; dist3D7; dist3Dcentroid; dist3DGrasp; dist2D0; dist2D1; dist2D2; dist2D3; dist2D4; dist2D5; dist2D6; dist2D7; dist2Dcentroid;\n")
		for i in range(len(filenamesSuccess)):
			line = str(filenamesSuccess[i]) + "; " + str(1) + "; " + str(0) + "; "
			line = line + str(locs[i][0]) + "; " + str(locs[i][1]) + "; " + str(locs[i][2]) + "; " + str(gt_locs[i][0]) + "; " + str(gt_locs[i][1]) + "; " + str(gt_locs[i][2]) + "; "
			#line = line + str(oris[i][0]) + "; " + str(oris[i][1]) + "; " + str(oris[i][2]) + "; " + str(oris[i][3]) + "; " + str(gt_oris[i][0]) + "; " + str(gt_oris[i][1]) + "; " + str(gt_oris[i][2]) + "; " + str(gt_oris[i][3]) + "; "
			line = line + str(rots[i][0]) + "; " + str(rots[i][1]) + "; " + str(rots[i][2]) + "; " + str(gt_rots[i][0]) + "; " + str(gt_rots[i][1]) + "; " + str(gt_rots[i][2]) + "; "
			line = line + str(meanDists3d[i]) + "; " + str(meanDists2d[i]) + "; "
			line = line + str(dists3d[i][0]) + "; " + str(dists3d[i][1]) + "; " + str(dists3d[i][2]) + "; " + str(dists3d[i][3]) + "; " + str(dists3d[i][4]) + "; " + str(dists3d[i][5]) + "; " + str(dists3d[i][6]) + "; " + str(dists3d[i][7]) + "; " + str(dists3d[i][8]) + "; " + str(str(dists3d[i][9])) + "; "
			line = line + str(dists2d[i][0]) + "; " + str(dists2d[i][1]) + "; " + str(dists2d[i][2]) + "; " + str(dists2d[i][3]) + "; " + str(dists2d[i][4]) + "; " + str(dists2d[i][5]) + "; " + str(dists2d[i][6]) + "; " + str(dists2d[i][7]) + "; " + str(dists2d[i][8]) + "; "
			f.write(line + "\n")

		for i in range(len(filenamesFailure)):
			line = str(filenamesFailure[i]) + "; " + str(0) + "; " + str(1) + "; "
			f.write(line + "\n")

	print('    Model tested in {} seconds.'.format(time.time() - model_testing_start_time))
	print '    Number of samples success / failure: ' + str(len(filenamesSuccess)) + " / " + str(len(filenamesFailure))

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
	testDataFolderFail = testDataFolder + "/" + "fail"
	if not os.path.exists(testDataFolderFail):
		os.makedirs(testDataFolderFail)

	test_dope(params, testDataFolder)
