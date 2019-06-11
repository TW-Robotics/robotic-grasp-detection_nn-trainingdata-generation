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
			self.dists3d.append(data["success"][i]["cuboid_dists3d"])
			self.ADDCuboids.append(data["success"][i]["ADDCuboids"])
			self.ADDModels.append(data["success"][i]["ADDModel"])
			self.transl_errors.append(data["success"][i]["transl_error"])
			self.rot_errors.append(data["success"][i]["rot_error"])
			self.poses_gt.append(data["success"][i]["pose_gt"])
			self.poses_est.append(data["success"][i]["pose_est"])

		for i in range(len(data["failure"])):
			self.filenamesFailure.append(data["failure"][i]["filename"])

	def analyse(self):
		#TODO


###########################################################
# VISUALISATION ###########################################
###########################################################

if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='Analyse Evaluation')
	parser.add_argument('Path', metavar='Path', type=str, help='Path to evaluation-file.json to analyse')
	path = parser.parse_args().Path
	print("Analysing evaluation '{}'...".format(Path))

	analyse_evaluation(path)
