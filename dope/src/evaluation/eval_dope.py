#!/usr/bin/env python

# Copyright (c) 2018 NVIDIA Corporation. All rights reserved.
# This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
# https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode

"""
This file takes all eval-images from a folder, runs them through dope
and compares the output of dope with the ground-truth.
"""

import time
import sys
import yaml
import os
import csv
import glob
import argparse
import shutil
from scipy import spatial

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

class eval_dope():
	def __init__(self, params, net):
		self.models = {}
		self.pnp_solvers = {}
		self.draw_colors = {}

		# Initialize parameters
		# Camera-Matrix for eval-images
		self.matrix_camera = np.zeros((3,3))
		self.matrix_camera[0,0] = params["camera_settings"]['fx']
		self.matrix_camera[1,1] = params["camera_settings"]['fy']
		self.matrix_camera[0,2] = params["camera_settings"]['cx']
		self.matrix_camera[1,2] = params["camera_settings"]['cy']
		self.matrix_camera[2,2] = 1

		self.dist_coeffs = np.zeros((4,1))
		if "self.dist_coeffs" in params["camera_settings"]:
			self.dist_coeffs = np.array(params["camera_settings"]['self.dist_coeffs'])
		self.config_detect = lambda: None
		self.config_detect.mask_edges = 1
		self.config_detect.mask_faces = 1
		self.config_detect.vertex = 1
		self.config_detect.threshold = 0.5
		self.config_detect.softmax = 1000
		self.config_detect.thresh_angle = params['thresh_angle']
		self.config_detect.thresh_map = params['thresh_map']
		self.config_detect.sigma = params['sigma']
		self.config_detect.thresh_points = params["thresh_points"]

		self.cuboid_points_3d = np.array([[ 9.9,-4.5, 8.85, 1],
									[-9.9,-4.5, 8.85, 1],
									[-9.9, 4.5, 8.85, 1],
									[ 9.9, 4.5, 8.85, 1],
									[ 9.9,-4.5,-8.85, 1],
									[-9.9,-4.5,-8.85, 1],
									[-9.9, 4.5,-8.85, 1],
									[ 9.9, 4.5,-8.85, 1],
									[ 0. , 0. , 0.  , 1]])   # Centroid
		#self.graspPoint_3d = [ 0. , 2.5, 7.527, 1]])

		# For each object to detect, load network model and create PNP solver
		self.model = "carrier_empty"
		self.models[self.model] = self.modelData(self.model, "../../weights/" + net)
		self.models[self.model].load_net_self.model()
		
		self.draw_colors[self.model] = tuple(params["self.draw_colors"][self.model])
		self.pnp_solvers[self.model] = CuboidPNPSolver(self.model,	self.matrix_camera, Cuboid3d(params['dimensions'][self.model]), dist_coeffs=self.dist_coeffs)

		# Init pnp-Solver for ground-truth pose calculation
		self.pnp_solvers["gt"] = CuboidPNPSolver(self.model, self.matrix_camera, Cuboid3d(params['dimensions'][self.model]), dist_coeffs=self.dist_coeffs)

		print ("Evaluatinging DOPE...")
		model_evaling_start_time = time.time()

		# Init variables to store results
		self.filenamesSuccess = []
		self.filenamesFailure = []
		self.dists3d = []
		self.dists2d = []
		self.meanDists3d = []
		self.meanDists2d = []
		self.locs_est = []
		self.oris_est = []
		self.rots_est = []
		self.locs_gt = []
		self.oris_gt = []
		self.rots_gt = []
		self.Ms_est = []
		self.Ms_gt = []

		self.createFolders(params, net)
		self.eval(params['path_to_images'])
		self.writeCSV(net)

		print('    self.model evaled in {} seconds.'.format(time.time() - model_evaling_start_time))
		print '    Number of samples success / failure: ' + str(len(self.filenamesSuccess)) + " / " + str(len(self.filenamesFailure))

	def createFolders(self, params, net):
		path_to_images = params['path_to_images']
		path_to_store_results = os.path.dirname(os.path.dirname(path_to_images))	# Go one level up so this folder will not be searched
		
		self.evalDataFolder = path_to_store_results + "/" + "eval_data_" + net
		if not os.path.exists(evalDataFolder):
			os.makedirs(evalDataFolder)
		self.evalDataFolderFail = evalDataFolder + "/" + "fail_" + net
		if not os.path.exists(evalDataFolderFail):
			os.makedirs(evalDataFolderFail)
		self.evalDataFolderSuccess = evalDataFolder + "/" + "success_" + net
		if not os.path.exists(evalDataFolderSuccess):
			os.makedirs(evalDataFolderSuccess)

	###########################################################
	# EVALUATION ##############################################
	###########################################################

	def ADDErrorCuboid(pose_gu, pose_gt, cuboid):
		"""
			Compute the ADD error for a given cuboid. 
			pose_gu is the predicted pose as a matrix
			pose_gt is the ground thruth pose as a matrix
			cuboid is a Cuboid3D object (see inference/cuboid.py)
		"""
		vertices = np.array(cuboid._vertices)
		vertices = np.insert(vertices,3,1,axis=1)
		vertices = np.rot90(vertices,3)

		obj = vertices
		pred_obj = np.matmul(pose_gu, obj)

		actual_obj = np.matmul(pose_gt, obj) 
		dist = spatial.distance.cdist(pred_obj.T, actual_obj.T, 'euclidean')
		true_dist = [dist[i][i] for i in range(len(dist))]
		return np.mean(true_dist)

	def calc_ADD_cuboid(self, est_points, gt_points):
		dist = spatial.distance.cdist(est_points, gt_points, 'euclidean')
		true_dist = [dist[i][i] for i in range(len(dist))]
		return true_dist, np.mean(true_dist)		

	def calc_distance(self, est_points, gt_points):
		dist = []
		for i in range(len(est_points)):
			try:
				if len(est_points[i]) == 3:
					dist.append(math.sqrt(  math.pow(est_points[i][0] - gt_points[i][0], 2) +
											math.pow(est_points[i][1] - gt_points[i][1], 2) +
											math.pow(est_points[i][2] - gt_points[i][2], 2)* 1.0))
			except:
				print "Error when calculating distance!"
				return [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 0

			try:
				if len(est_points[i]) == 2:
					dist.append(math.sqrt(  math.pow(est_points[i][0] - gt_points[i][0], 2) +
											math.pow(est_points[i][1] - gt_points[i][1], 2)* 1.0))
			except:
				print "Error when calculating distance!"
				return [0, 0, 0, 0, 0, 0, 0, 0, 0], 0

		return dist, sum(dist)/len(dist)

	def getMatrixFromQuaternions(self, quat, transl):
		# Get homogenous transformation matrix and put translation in last column
		M = tf.transformations.quaternion_matrix(quat)
		M[0][3] = transl[0]
		M[1][3] = transl[1]
		M[2][3] = transl[2]
		return M

	def transformPoints(self, points_3d, M):
		points_3d = np.rot90(points_3d, 3)
		points_3d_transf = np.matmul(M, points_3d) #M.dot(points_3d)
		return np.rot90(points_3d_transf)

		'''points_3d_transform = []
		for point in points_3d:
			points_3d_transform.append(M.dot(point))
		return points_3d_transform'''

	def transformPoint(self, point_3d, M):
		point = M.dot(point_3d)
		return point[:-1]	# Delete 1 from homogenous transformation

	def loadJson(self, path):
		# Load json-file and get ground-truth data
		with open(path.replace("png","json")) as json_file:
			data = json.load(json_file)
			gt_loc = data["objects"][0]["location"]
			gt_ori = data["objects"][0]["quaternion_xyzw"]
			gt_points2d = data["objects"][0]["projected_cuboid"]
			gt_points3d = data["objects"][0]["cuboid"]
			gt_points2d.append(data["objects"][0]["projected_cuboid_centroid"])
			gt_points3d.append(data["objects"][0]["cuboid_centroid"])
		return gt_loc, gt_ori, gt_points2d, gt_points3d

	def eval(self, pathToFiles):
		# For all images in folder
		print "Analysing images in folder " + pathToFiles)
		for imgpath in pathToFiles + "/*.png"):
			if os.path.exists(imgpath) and os.path.exists(imgpath.replace("png","json")):
				fileName = os.path.splitext(os.path.basename(imgpath))[0]
		
				# Read and copy image
				img = cv2.imread(imgpath)
				img_copy = img.copy()
				im = Image.fromarray(img_copy)
				self.image_draw = ImageDraw.Draw(im)

				# Detect object
				m = self.model
				results = ObjectDetector.detect_object_in_image(self.models[m].net, self.pnp_solvers[m], img, self.config_detect)

				# Get ground-truth data
				gt_loc, gt_ori, points2d_gt, points3d_gt = loadJson(imgpath)

				# Get pose and overlay cube on image, if results is not empty
				for result in enumerate(results):
					if result["location"] is None:
						continue
					est_loc = result["location"]
					est_ori = result["quaternion"]

					# Get transformation matrix and transform cuboid points
					M_est = self.getMatrixFromQuaternions(est_ori, est_loc)
					points3d_est = self.transformPoints(self.cuboid_points_3d, M_est)

					if None not in result['projected_points']:
						points2d_est = []
						for pair in result['projected_points']:
							points2d_est.append(tuple(pair))

					# Calculate ground-truth quaternions from pnp-solver
					gt_result = self.pnp_solvers["gt"].solve_pnp(points2d_gt)
					gt_ori = gt_result[1]
					gt_loc = gt_result[0]	# Location berechnen weil gt_loc falscher Bezug ist (object-coordinate system ist auf Base und nicht Centroid)

					# Get transformation matrix and 3d grasp-point
					M_gt = self.getMatrixFromQuaternions(gt_ori, gt_loc)
					#gt_grasp_point = self.transformPoint(self.graspPoint_3d, M_gt) Not necessary since same point is transformed for gt and est -> hebt sich auf

					# Calculate distance between ground truth and estimation
					dist3d, meanDist3d = self.calc_ADD_cuboid(points3d_est, np.array(points3d_gt))
					#dist3d, meanDist3d = self.calc_distance(points3d_est, np.array(points3d_gt))
					#dist2d, meanDist2d = self.calc_distance(points2d_est, np.array(points2d_gt))

					# Transform rotations into euler angles
					#rot = tf.transformations.euler_from_quaternion(est_ori)	# TODO Achsreihenfolge?!
					#rot = [rot[i] / math.pi*180 for i in range(len(rot))]
					#gt_rot = tf.transformations.euler_from_quaternion(gt_ori)#, axes='szyx')
					#gt_rot = [gt_rot[i] / math.pi*180 for i in range(len(gt_rot))]

					# Store all calculated results in correct format in arrays
					self.locs_est.append([est_loc[i] * 10 for i in range(len(est_loc))])
					self.oris_est.append(est_ori)
					#self.rots_est.append(rot)

					self.Ms_est.append(M_est)
					self.Ms_gt.append(M_gt)

					self.locs_gt.append([gt_loc[i] * 10 for i in range(len(gt_loc))])
					self.oris_gt.append(gt_ori)
					#self.rots_gt.append(gt_rot)

					self.dists3d.append([dist3d[i] * 10 for i in range(len(dist3d))])
					#self.dists2d.append(dist2d)
					self.meanDists3d.append(meanDist3d * 10)
					#self.meanDists2d.append(meanDist2d)

				# Draw ground-truth cube to image
				DrawCube(points2d_gt, tuple([13, 255, 128])) # Green		
				# If no object could be detected copy file to failed-folder
				if len(results) == 0:
					self.filenamesFailure.append(fileName)
					cv2.imwrite(self.evalDataFolderFail + "/" + fileName + ".failed.png", np.array(im)[...,::-1])
					shutil.copyfile(pathToFiles + fileName + ".json", self.evalDataFolderFail "/" + fileName + ".failed.png")
				else:
					self.filenamesSuccess.append(fileName)
					# Draw the cube
					DrawCube(points2d_est, self.draw_colors[m])

					# Store image with results overlaid and json-file
					cv2.imwrite(self.evalDataFolderSuccess + "/" + fileName + ".success.png", np.array(im)[...,::-1])
					shutil.copyfile(pathToFiles + fileName + ".json", self.evalDataFolderSuccess + "/" + fileName + ".success.json")

	def writeCSV(net):
		# Write results to file
		with open(self.evalDataFolder + "/evaluation_" + net +".csv", "wb") as f: #q1; q2; q3; q4; gtQ1; gtQ2; gtQ3; gtQ4;
			f.write("filename; success; failure; locX; locY; locZ; gtLocX; gtLocY; gtLocZ; rx; ry; rz; gtRx; gtRy; gtRz; meanDist3d; meanDist2d; dist3D0; dist3D1; dist3D2; dist3D3; dist3D4; dist3D5; dist3D6; dist3D7; dist3Dcentroid; dist3DGrasp; dist2D0; dist2D1; dist2D2; dist2D3; dist2D4; dist2D5; dist2D6; dist2D7; dist2Dcentroid;\n")
			for i in range(len(self.filenamesSuccess)):
				line = str(self.filenamesSuccess[i]) + "; " + str(1) + "; " + str(0) + "; "
				line = line + str(self.locs_est[i][0]) + "; " + str(self.locs_est[i][1]) + "; " + str(self.locs_est[i][2]) + "; " + str(self.locs_gt[i][0]) + "; " + str(self.locs_gt[i][1]) + "; " + str(self.locs_gt[i][2]) + "; "
				#line = line + str(self.oris_est[i][0]) + "; " + str(self.oris_est[i][1]) + "; " + str(self.oris_est[i][2]) + "; " + str(oris[i][3]) + "; " + str(self.oris_gt[i][0]) + "; " + str(self.oris_gt[i][1]) + "; " + str(self.oris_gt[i][2]) + "; " + str(self.oris_gt[i][3]) + "; "
				line = line + str(self.rots_est[i][0]) + "; " + str(self.rots_est[i][1]) + "; " + str(self.rots_est[i][2]) + "; " + str(self.rots_gt[i][0]) + "; " + str(self.rots_gt[i][1]) + "; " + str(self.rots_gt[i][2]) + "; "
				line = line + str(self.meanDists3d[i]) + "; "# + str(self.meanDists2d[i]) + "; "
				line = line + str(self.dists3d[i][0]) + "; " + str(self.dists3d[i][1]) + "; " + str(self.dists3d[i][2]) + "; " + str(self.dists3d[i][3]) + "; " + str(self.dists3d[i][4]) + "; " + str(self.dists3d[i][5]) + "; " + str(self.dists3d[i][6]) + "; " + str(self.dists3d[i][7]) + "; " + str(self.dists3d[i][8]) + "; " + str(str(self.dists3d[i][9])) + "; "
				#line = line + str(self.dists2d[i][0]) + "; " + str(self.dists2d[i][1]) + "; " + str(self.dists2d[i][2]) + "; " + str(self.dists2d[i][3]) + "; " + str(self.dists2d[i][4]) + "; " + str(self.dists2d[i][5]) + "; " + str(self.dists2d[i][6]) + "; " + str(self.dists2d[i][7]) + "; " + str(self.dists2d[i][8]) + "; "
				f.write(line + "\n")

			for i in range(len(self.filenamesFailure)):
				line = str(self.filenamesFailure[i]) + "; " + str(0) + "; " + str(1) + "; "
				f.write(line + "\n")

###########################################################
# VISUALISATION ###########################################
###########################################################

	### Code to visualize the neural network output
	def DrawLine(self, point1, point2, lineColor, lineWidth):
		'''Draws line on image'''
		if not point1 is None and point2 is not None:
			self.image_draw.line([point1,point2], fill=lineColor, width=lineWidth)

	def DrawDot(self, point, pointColor, pointRadius):
		'''Draws dot (filled circle) on image'''
		if point is not None:
			xy = [
				point[0]-pointRadius, 
				point[1]-pointRadius, 
				point[0]+pointRadius, 
				point[1]+pointRadius
			]
			self.image_draw.ellipse(xy, 
				fill=pointColor, 
				outline=pointColor
			)

	def DrawCube(self, points, color=(255, 0, 0)):
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


if __name__ == "__main__":
	'''Main routine to run DOPE eval-code'''

	parser = argparse.ArgumentParser(description='Evaluate Dope')
	parser.add_argument('Net', metavar='Net', type=str, help='Name of network in folder "weights" to evaluate')
	net = parser.parse_args().Net
	print("Using network self.model '{}'...".format(net))

	params = None
	config_name = "config_quantitative.yaml"
	yaml_path = '../../config/{}'.format(config_name)
	with open(yaml_path, 'r') as stream:
		try:
			print("Loading DOPE parameters from '{}'...".format(yaml_path))
			params = yaml.load(stream)
			print('    Parameters loaded.')
		except yaml.YAMLError as exc:
			print(exc)

	eval_dope(params, net)
