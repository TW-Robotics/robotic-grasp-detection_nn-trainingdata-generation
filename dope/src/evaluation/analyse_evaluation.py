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

		self.analyse()

		print('    Evaluation analysed in {} seconds.'.format(time.time() - model_evaling_start_time))

	###########################################################
	# EVALUATION ##############################################
	###########################################################

	def analyse(self):

###########################################################
# VISUALISATION ###########################################
###########################################################

if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='Analyse Evaluation')
	parser.add_argument('Path', metavar='Path', type=str, help='Path to evaluation-file.json to analyse')
	path = parser.parse_args().Path
	print("Analysing evaluation '{}'...".format(Path))

	analyse_evaluation(path)
