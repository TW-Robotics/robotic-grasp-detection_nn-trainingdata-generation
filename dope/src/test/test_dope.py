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
		if len(est_points[i]) == 3:
			dist.append(math.sqrt(  math.pow(est_points[i][0] - gt_points[i][0], 2) +
									math.pow(est_points[i][1] - gt_points[i][1], 2) +
									math.pow(est_points[i][2] - gt_points[i][2], 2)* 1.0))
		elif len(est_points[i]) == 2:
			dist.append(math.sqrt(  math.pow(est_points[i][0] - gt_points[i][0], 2) +
									math.pow(est_points[i][1] - gt_points[i][1], 2)* 1.0))	
	return dist, sum(dist)/len(dist), min(dist), max(dist)

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

	# Open json-file and extract camera-info
	cameraSettingsPath = params['path_to_images'] + "/_camera_settings" + ".json"
	if os.path.exists(cameraSettingsPath):
		intrinsics = []
		with open(cameraSettingsPath) as json_file:
			data = json.load(json_file)
			settings = data["camera_settings"]
			matrix_camera_gt = np.zeros((3,3))
			matrix_camera_gt[0,0] = settings[0]["intrinsic_settings"]["fx"]
			matrix_camera_gt[1,1] = settings[0]["intrinsic_settings"]['fy']
			matrix_camera_gt[0,2] = settings[0]["intrinsic_settings"]['cx']
			matrix_camera_gt[1,2] = settings[0]["intrinsic_settings"]['cy']
			matrix_camera_gt[2,2] = 1
	else:
		print("No camera settings path found in test-folder! File not found: " + str(cameraSettingsPath))
		return

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
		pnp_solver_gt = CuboidPNPSolver(model, matrix_camera_gt, Cuboid3d(params['dimensions'][model]), dist_coeffs=dist_coeffs)

	cuboid_points_3d = np.array([[ 9.9,-4.5, 8.85, 1],
								[-9.9,-4.5, 8.85, 1],
								[-9.9, 4.5, 8.85, 1],
								[ 9.9, 4.5, 8.85, 1],
								[ 9.9,-4.5,-8.85, 1],
								[-9.9,-4.5,-8.85, 1],
								[-9.9, 4.5,-8.85, 1],
								[ 9.9, 4.5,-8.85, 1],
								[ 0. , 0. , 0.  , 1]])

	print ("Testing DOPE...")

	model_testing_start_time = time.time()

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
	print params['path_to_images']
	for imgpath in glob.glob(params['path_to_images'] + "/*.png"):
		#print imgpath
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
				
				#print results
				#print results[0]['points_3d']

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
					#print M_trans

					# Transform cuboid-corners and centroid and store them
					cuboid_points_3d_transfrom = []
					for point in cuboid_points_3d:
						cuboid_points_3d_transfrom.append(M_trans.dot(point))
					cuboid_points_3d_transfrom = np.delete(cuboid_points_3d_transfrom, 3, 1)	# Delete 1s from homogenous transformation
					#print cuboid_points_3d_transfrom

					# Draw the cube
					if None not in result['projected_points']:
						points2d = []
						points3d = []
						for pair in result['projected_points']:
							points2d.append(tuple(pair))
						DrawCube(points2d, draw_colors[m])

					# Load json-file
					with open(imgpath.replace("png","json")) as json_file:
						data = json.load(json_file)
						
						gt_pos = data["objects"][0]["location"]
						gt_ori = data["objects"][0]["quaternion_xyzw"]
						gt_points2d = data["objects"][0]["projected_cuboid"]
						gt_points3d = data["objects"][0]["cuboid"]
						#gt_centroid_point2d = data["objects"][0]["projected_cuboid_centroid"]
						#gt_centroid_point3d = data["objects"][0]["cuboid_centroid"]
						gt_points2d.append(data["objects"][0]["projected_cuboid_centroid"])
						gt_points3d.append(data["objects"][0]["cuboid_centroid"])
					
					#print type(gt_points3d)
					#print type(cuboid_points_3d_transfrom)
					#print gt_points3d

					res = pnp_solver_gt.solve_pnp(gt_points2d)
					print res[1]#["quaternion"]
					print ori
					print res[0]
					print gt_pos
					print np.array(gt_points2d)
					print res[2]

					M_trans = tf.transformations.quaternion_matrix(res[1])
					M_trans[0][3] = gt_pos[0]
					M_trans[1][3] = gt_pos[1]
					M_trans[2][3] = gt_pos[2]
					#print M_trans

					'''gt_cuboid_points_3d_transfrom = []
					for point in cuboid_points_3d:
						gt_cuboid_points_3d_transfrom.append(M_trans.dot(point))
					gt_cuboid_points_3d_transfrom = np.delete(gt_cuboid_points_3d_transfrom, 3, 1)	# Delete 1s from homogenous transformation
					print gt_cuboid_points_3d_transfrom
					print np.array(gt_points3d)
					print cuboid_points_3d_transfrom'''

					dist3d, meanDist3d, minDist3d, maxDist3d = calc_distance(cuboid_points_3d_transfrom, np.array(gt_points3d))
					#print dist3d, meanDist3d, minDist3d, maxDist3d
					dist2d, meanDist2d, minDist2d, minDist3d = calc_distance(points2d, np.array(gt_points2d))

					rot = tf.transformations.euler_from_quaternion(ori)	# TODO Achsreihenfolge?!
					rot = [rot[i] / math.pi*180 for i in range(len(rot))]
					gt_rot = tf.transformations.euler_from_quaternion(res[1])#, axes='szyx')
					gt_rot = [gt_rot[i] / math.pi*180 for i in range(len(gt_rot))]

					'''est_rot = Quaternion(ori[3], ori[0], ori[1], ori[2])
					gt_rot = Quaternion(res[1][3], res[1][0], res[1][1], res[1][2])
					diff = est_rot - gt_rot
					#print diff.is_unit()
					diff = diff.normalised
					#print diff.is_unit()
					#print diff
					diffEuler = tf.transformations.euler_from_quaternion((diff[1], diff[2], diff[3], diff[0]))
					print [diffEuler[i] / math.pi*180 for i in range(len(diffEuler))]'''

					locs.append([loc[i] * 10 for i in range(len(loc))])
					oris.append(ori)
					rots.append(rot)

					gt_locs.append([gt_pos[i] * 10 for i in range(len(gt_pos))])
					gt_oris.append(gt_ori)
					gt_rots.append(gt_rot)

					dists3d.append([dist3d[i] * 10 for i in range(len(dist3d))])
					dists2d.append(dist2d)
					meanDists3d.append(meanDist3d * 10)
					meanDists2d.append(meanDist2d)
					filenamesSuccess.append(fileName)

					# Store image with results overlaid
					#im.save(sys.stdout, "png")
					#convImg = CvBridge().cv2_to_imgmsg(np.array(im)[...,::-1], "bgr8")
					#print type(myImg)
					#print type(np.array(im)[...,::-1])
					#print imgpath
					cv2.imwrite(testDataFolder + "/" + fileName + ".vis.png", np.array(im)[...,::-1])
	
	#print filenamesSuccess
	# filename; meanDist3d; meanDist2d; dist3D; dist2D
	print type(oris[0])
	with open(testDataFolder + "/evaluation.csv", "wb") as f:
		#writer = csv.writer(f, delimiter=";")
		f.write("filename; success; failure; locX; locY; locZ; gtLocX; gtLocY; gtLocZ; q1; q2; q3; q4; gtQ1; gtQ2; gtQ3; gtQ4; rx; ry; rz; gtRx; gtRy; gtRz; meanDist3d; meanDist2d; dist3D0; dist3D1; dist3D2; dist3D3; dist3D4; dist3D5; dist3D6; dist3D7; dist3Dcentroid; dist2D0; dist2D1; dist2D2; dist2D3; dist2D4; dist2D5; dist2D6; dist2D7; dist2Dcentroid;\n")
		for i in range(len(filenamesSuccess)):
			line = str(filenamesSuccess[i]) + "; " + str(1) + "; " + str(0) + "; "
			line = line + "\n" + str(locs[i][0]) + "; " + str(locs[i][1]) + "; " + str(locs[i][2]) + "; " + str(gt_locs[i][0]) + "; " + str(gt_locs[i][1]) + "; " + str(gt_locs[i][2]) + "; "
			#line = line + "\n" + str(oris[i][0]) + "; " + str(oris[i][1]) + "; " + str(oris[i][2]) + "; " + str(oris[i][3]) + "; " + str(gt_oris[i][0]) + "; " + str(gt_oris[i][1]) + "; " + str(gt_oris[i][2]) + "; " + str(gt_oris[i][3]) + "; "
			line = line + "\n" + str(rots[i][0]) + "; " + str(rots[i][1]) + "; " + str(rots[i][2]) + "; " + str(gt_rots[i][0]) + "; " + str(gt_rots[i][1]) + "; " + str(gt_rots[i][2]) + "; "
			line = line + "\n" + str(meanDists3d[i]) + "; " + str(meanDists2d[i]) + "; "
			line = line + "\n" + str(dists3d[i][0]) + "; " + str(dists3d[i][1]) + "; " + str(dists3d[i][2]) + "; " + str(dists3d[i][3]) + "; " + str(dists3d[i][4]) + "; " + str(dists3d[i][5]) + "; " + str(dists3d[i][6]) + "; " + str(dists3d[i][7]) + "; " + str(dists3d[i][8]) + "; "
			line = line + "\n" + str(dists2d[i][0]) + "; " + str(dists2d[i][1]) + "; " + str(dists2d[i][2]) + "; " + str(dists2d[i][3]) + "; " + str(dists2d[i][4]) + "; " + str(dists2d[i][5]) + "; " + str(dists2d[i][6]) + "; " + str(dists2d[i][7]) + "; " + str(dists2d[i][8]) + "; "
			f.write(line + "\n")

		for i in range(len(filenamesFailure)):
			line = str(filenamesFailure[i]) + "; " + str(0) + "; " + str(1) + "; "
			f.write(line + "\n")

	print('    Model tested in {} seconds.'.format(time.time() - model_testing_start_time))

	'''writer = csv.writer(f, delimiter=";")
		writer.writerow("filename; locX; locY; locZ; gtLocX; gtLocY; gtLocZ; q1; q2; q3; q4; gtQ1; gtQ2; gtQ3; gtQ4; rx; ry; rz; gtRx; gtRy; gtRz; meanDist3d; meanDist2d; dist3D0; dist3D1; dist3D2; dist3D3; dist3D4; dist3D5; dist3D6; dist3D7; dist3Dcentroid; dist2D0; dist2D1; dist2D2; dist2D3; dist2D4; dist2D5; dist2D6; dist2D7; dist2Dcentroid;")
		for i in range(len(filenames)):
			line = str(filenames[i]) + "; "
			line = line + str(loc[0]) + ";" + str(loc[1]) + ";" + str(loc[2]) + ";" + str(gt_pos[0]) + ";" + str(gt_pos[1]) + ";" + str(gt_pos[2]) + ";"
			line = line + str(ori[0]) + ";" + str(ori[1]) + ";" + str(ori[2]) + ";" + str(ori[3]) + ";" + str(gt_ori[0]) + ";" + str(gt_ori[1]) + ";" + str(gt_ori[2]) + ";" + str(gt_ori[3]) + ";"
			line = line + str(rot[0]) + ";" + str(rot[1]) + ";" + str(rot[2]) + ";" + str(gt_rot[0]) + ";" + str(gt_rot[1]) + ";" + str(gt_rot[2]) + ";"
			line = line + str(meanDist3d) + "; " + str(meanDist2d) + "; "
			line = line + str(dist3d[0]) + "; " + str(dist3d[1]) + "; " + str(dist3d[2]) + "; " + str(dist3d[3]) + "; " + str(dist3d[4]) + "; " + str(dist3d[5]) + "; " + str(dist3d[6]) + "; " + str(dist3d[7]) + "; " + str(dist3d[8]) + "; "
			line = line + str(dist2d[0]) + "; " + str(dist2d[1]) + "; " + str(dist2d[2]) + "; " + str(dist2d[3]) + "; " + str(dist2d[4]) + "; " + str(dist2d[5]) + "; " + str(dist2d[6]) + "; " + str(dist2d[7]) + "; " + str(dist2d[8]) + "; "
			writer.writerow(line)'''

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
