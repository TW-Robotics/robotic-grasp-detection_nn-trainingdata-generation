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

def val(value):
	value = str(round(value,2))
	return value.replace(".",",")

class analyse_evaluation():
	def __init__(self, path):
		#model_evaling_start_time = time.time()

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
		b_addC, n_addC, n_addM, n_trans, n_rot, b_addC_cum, n_addC_cum, n_addM_cum, n_trans_cum, n_rot_cum = self.analyse()
		self.write_csv(path, b_addC, n_addC, n_addM, n_trans, n_rot, b_addC_cum, n_addC_cum, n_addM_cum, n_trans_cum, n_rot_cum)
		#print('    Evaluation analysed in {} seconds.'.format(time.time() - model_evaling_start_time))

	###########################################################
	# EVALUATION ##############################################
	###########################################################

	def load_json(self, path):
		with open(path) as json_file:  
			data = json.load(json_file)
		
		self.numSuccess = len(data["success"])
		self.numFail = len(data["failure"])

		for i in range(self.numSuccess):
			self.filenamesSuccess.append(data["success"][i]["filename"])
			self.dists3d.append(data["success"][i]["cuboid_dists3d_mm"])
			self.ADDCuboids.append(data["success"][i]["ADDCuboids_mm"])
			self.ADDModels.append(data["success"][i]["ADDModel_mm"])
			self.transl_errors.append(data["success"][i]["transl_error_mm"])
			self.rot_errors.append(data["success"][i]["rot_error_deg"])
			self.poses_gt.append(data["success"][i]["pose_gt_mm"])
			self.poses_est.append(data["success"][i]["pose_est_mm"])

		for i in range(self.numFail):
			self.filenamesFailure.append(data["failure"][i]["filename"])

	def calc_total_metrics(self):
		total = self.numSuccess + self.numFail
		percFailDet = self.numFail * 100 / float(total)

		return total, self.numFail, percFailDet, 100-percFailDet

	def calc_sub_metrics(self, thresholds, x, y):
		numSmallerThresholds = thresholds
		for i_thr in range(len(thresholds)):
			for i in range(len(y)):
				if x[i] < thresholds[i_thr]:
					numSmallerThresholds[i_thr] = numSmallerThresholds[i_thr] + y[i]
				else:
					break
		return self.calc_perc_from_num(numSmallerThresholds)

	def calc_perc_from_num(self, numSmallerThresholds):
		percSmallerThresholds = numSmallerThresholds

		total = self.numSuccess + self.numFail
		for i in range(len(percSmallerThresholds)):
			percSmallerThresholds[i] = numSmallerThresholds[i] * 100 / float(total)
		return percSmallerThresholds

	def calc_cumulative_accuracy(self, values, numBins, rangeHist):
		# cumulative-flag sums up n in bins
		n, bins, patches = plt.hist(values, bins=numBins, range=rangeHist, cumulative=True)
		b = []
		for i in range(numBins):
			# calculate percentage
			n[i] = n[i] * 100 / float(self.numSuccess)
			# calculate x-values (in the center of two bins)
			b.append((bins[i] + bins[i+1]) / 2)
		plt.clf() # Clear figure
		return b, n

	def plot_hist(self, values, title, numBins, rangeHist):
		#n, bins, patches = plt.hist(self.ADDCuboids)
		#plt.clf() # Clear figure
		#if bins[len(bins)-1] > 30:
		#	print "Bigger 30"
		n, bins, patches = plt.hist(values, bins=numBins, range=rangeHist) #cumulative=True, histtype='step'
		plt.xlim(rangeHist[0],rangeHist[1])
		plt.title(title)
		plt.xlabel("Value")
		plt.ylabel("Count")
		#plt.show()
		return bins, n

	def write_csv(self, path, b, n_addC, n_addM, n_trans, n_rot, b_cum, n_addC_cum, n_addM_cum, n_trans_cum, n_rot_cum):
		total, numFailDet, percFailDet, percSuccDet = self.calc_total_metrics()
		# Write results to file
		with open(os.path.dirname(path) + "/evaluation_" + os.path.splitext(os.path.basename(path))[0] +".csv", "wb") as f:
			f.write("numEvalImages; numFailDet; percFailDet; percSuccDet;\n")
			f.write(val(total) + "; " + val(numFailDet) + "; " + val(percFailDet) + "; " + val(percSuccDet) + ";\n\n")
			f.write("Threshold; ADD_model_cum_mm; ADD_cuboids_cum_mm; Transl_error_cum_mm; Rot_error_cum_mm; Threshold; ; ADD_model_mm; ADD_cuboids_mm; Transl_error_mm; Rot_error_mm; \n")
			for i in range(max(len(b_cum), len(b))):
				line = ""
				if i < len(b_cum):
					line = line + val(b_cum[i]) + "; " + val(n_addC_cum[i]) + "; " + val(n_addM_cum[i]) + "; " + val(n_trans_cum[i]) + "; " + val(n_rot_cum[i]) + "; "
				else:
					line = line + "; ; ; ; ;"
				if i < len(b)-1:
					line = line + "; " + (val(b[i]) + "; " + val(n_addC[i]) + "; " + val(n_addM[i]) + "; " + val(n_trans[i]) + "; " + val(n_rot[i]) + "; ")
				line = line + "\n"
				f.write(line)

	def analyse(self):
		'''plt.hist(self.ADDModels, 10, [0, 30])
		plt.show()
		plt.hist(self.transl_errors, 10, [0, 30])
		plt.show()'''

		plt.subplot(2, 2, 1)
		b_addC, n_addC = self.plot_hist(self.ADDCuboids, "ADD Cuboids", 100, [0, 30])
		plt.subplot(2, 2, 3)
		b_addM, n_addM = self.plot_hist(self.ADDModels, "ADD Models", 100, [0, 30])
		plt.subplot(2, 2, 2)
		b_trans, n_trans = self.plot_hist(self.transl_errors, "Translational Errors", 100, [0, 30])
		plt.subplot(2, 2, 4)
		b_rot, n_rot = self.plot_hist(self.rot_errors, "Rotational Errors", 100, [0, 10])
		plt.show()

		b_addC_cum, n_addC_cum = self.calc_cumulative_accuracy(self.ADDCuboids, 100, [0, 30])
		b_addM_cum, n_addM_cum = self.calc_cumulative_accuracy(self.ADDModels, 100, [0, 30])
		b_trans_cum, n_trans_cum = self.calc_cumulative_accuracy(self.transl_errors, 100, [0, 30])
		b_rot_cum, n_rot_cum = self.calc_cumulative_accuracy(self.rot_errors, 100, [0, 30])
		#plt.subplot(2, 2, 1)
		plt.plot(b_addC_cum, n_addC_cum)
		plt.title("ADD Cuboids")
		#plt.subplot(2, 2, 3)
		plt.plot(b_addM_cum, n_addM_cum)
		plt.title("ADD Models")
		#plt.subplot(2, 2, 2)
		plt.plot(b_trans_cum, n_trans_cum)
		plt.title("Translational Errors")
		#plt.subplot(2, 2, 4)
		plt.plot(b_rot_cum, n_rot_cum)
		plt.title("Rotational Errors")
		plt.show()

		#total, numFailDet, percFailDet, persSuccDet =
		print(self.calc_total_metrics())
		print(self.calc_sub_metrics([15, 20, 28], b_addC, n_addC))
		print(self.calc_sub_metrics([15, 20, 28], b_addM, n_addM))
		print(self.calc_sub_metrics([15, 20, 28], b_trans, n_trans))

		return b_addC, n_addC, n_addM, n_trans, n_rot, b_addC_cum, n_addC_cum, n_addM_cum, n_trans_cum, n_rot_cum

###########################################################
# VISUALISATION ###########################################
###########################################################

if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='Analyse Evaluation')
	parser.add_argument('Path', metavar='Path', type=str, help='Path to evaluation-file.json to analyse')
	path = parser.parse_args().Path
	print("Analysing evaluation '{}'...".format(path))

	analyse_evaluation(path)
