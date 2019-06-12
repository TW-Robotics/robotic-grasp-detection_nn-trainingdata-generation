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
import re
import matplotlib.pyplot as plt

import numpy as np
import math

import json
import tf

class analyse_evaluation():
	def __init__(self, path):
		print ("Analysing Evaluation...")
		model_evaling_start_time = time.time()

		# Init variables to store results
		self.filenamesSuccess = []
		self.filenamesFailure = []
		self.dists3d = []
		self.ADDCuboids = []
		self.ADDModels = []
		self.transl_errors = []
		self.rot_errors = []
		self.poses_gt = []
		self.poses_est = []

		self.load_json(path)
		self.analyse()

		print('    Evaluation analysed in {} seconds.'.format(time.time() - model_evaling_start_time))

	###########################################################
	# EVALUATION ##############################################
	###########################################################

	def load_json(self, path):
		with open(path) as json_file:  
			data = json.load(json_file)
		
		for i in range(len(data["success"])):
			self.filenamesSuccess.append(data["success"][i]["filename"])
			self.dists3d.append(data["success"][i]["cuboid_dists3d_mm"])
			self.ADDCuboids.append(data["success"][i]["ADDCuboids_mm"])
			self.ADDModels.append(data["success"][i]["ADDModel_mm"])
			self.transl_errors.append(data["success"][i]["transl_error_mm"])
			self.rot_errors.append(data["success"][i]["rot_error_deg"])
			self.poses_gt.append(data["success"][i]["pose_gt_mm"])
			self.poses_est.append(data["success"][i]["pose_est_mm"])

		for i in range(len(data["failure"])):
			self.filenamesFailure.append(data["failure"][i]["filename"])

	def plot_hist(self, values, title, bins, range):
		#n, bins, patches = plt.hist(self.ADDCuboids)
		#plt.clf() # Clear figure
		#if bins[len(bins)-1] > 30:
		#	print "Bigger 30"
		n, bins, patches = plt.hist(values, bins=bins, range=range)
		plt.xlim(range[0],range[1])
		plt.title(title)
		plt.xlabel("Value")
		plt.ylabel("Count")
		#plt.show()

	def analyse(self):
		plt.subplot(2, 2, 1)
		self.plot_hist(self.ADDCuboids, "ADD Cuboids", 100, [0, 30])
		plt.subplot(2, 2, 3)
		self.plot_hist(self.ADDModels, "ADD Models", 100, [0, 30])
		plt.subplot(2, 2, 2)
		self.plot_hist(self.transl_errors, "Translational Errors", 100, [0, 30])
		plt.subplot(2, 2, 4)
		self.plot_hist(self.rot_errors, "Rotational Errors", 100, [0, 10])
		plt.show()

###########################################################
# VISUALISATION ###########################################
###########################################################

if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='Analyse Evaluation')
	parser.add_argument('Path', metavar='Path', type=str, help='Path to evaluation-file.json to analyse')
	path = parser.parse_args().Path
	print("Analysing evaluation '{}'...".format(path))

	analyse_evaluation(path)
