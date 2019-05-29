#!/usr/bin/env python
import numpy as np
import sys
import random
import rospy
import tf
import math
import os
import csv
import json
import re

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ur5_control import ur5_control
#from data_vis import *

debug = False
if rospy.get_param("print_debug") == True:
	print "Debug-Mode ON"
	debug = True		# Print Debug-Messages
storePC = False			# Store Point-Cloud-Message
storeFullRes = False

class dataCapture():
	def __init__(self, path, pathFullRes):
		# Init variables
		self.goals = PoseArray()		# Poses to move robot to
		self.camPoses = []				# Store poses where camera was
		self.actStoreID = 0 			# Counter for file-name
		self.actPoseID = 0 				# Counter for poses
		self.lastPoseID = 0 			# Counter for poses to enable to increment actPoseID
		self.actStorage = -1 			# Counter for sub-poses
		self.path = path
		self.pathFullRes = pathFullRes

		# Images and Camera-Settings
		self.bridge = CvBridge()
		self.rgb_img = None				
		self.d_img = None
		self.rgb_img_resized = None
		self.camera_settings_rgb = None
		self.camera_settings_rgb_resized = None
		self.camera_settings_depth = None
		self.intrinsics = [0., 0., 0., 0.]
		self.intrinsics_resized = None

		# Parameters for randomization
		self.rotateTiltRMin = rospy.get_param("PoseRandomization/rotate4Min") 	# Joint 4: How far to rotate
		self.rotateTiltRMax = rospy.get_param("PoseRandomization/rotate4Max")
		self.rotateUpRMin = rospy.get_param("PoseRandomization/rotate5Min") 	# Joint 5: How far to rotate
		self.rotateUpRMax = rospy.get_param("PoseRandomization/rotate5Max")
		self.rotateRMin = rospy.get_param("PoseRandomization/rotate6Min")		# Joint 6: How far can EEF be rotated
		self.rotateRMax = rospy.get_param("PoseRandomization/rotate6Max")

		# Parameters for Image-Calculations
		self.objectName = rospy.get_param("object_name")
		self.imgOutputSize = rospy.get_param("outputImage_size")
		self.imgWidth = rospy.get_param("camera_width")
		self.imgHeight = rospy.get_param("camera_height")
		self.img_scaleFac = float(self.imgHeight)/self.imgOutputSize
		self.resized_imgWidth = int(round(float(self.imgWidth)/self.img_scaleFac, 0))
		self.resized_img_horizontalStart = int(round(self.resized_imgWidth/2., 0)) - self.imgOutputSize/2
		#print self.img_scaleFac, self.resized_imgWidth, self.resized_img_horizontalStart

		# Transformation-Listener
		self.listener = tf.TransformListener()

		# Subscribers
		rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_image_callback)					# RGB-Image

	###########################################################
	# CALLBACKS ###############################################
	###########################################################
	def rgb_image_callback(self, data):
		try:
			cv_rgb_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
			# Store image locally and resize it
			self.rgb_img = cv_rgb_image.copy()
			self.rgb_resize()
		except CvBridgeError as e:
			print(e)

	###########################################################
	# HELPERS #################################################
	###########################################################
	# Resize the RGB-image to be a square image of output-size
	def rgb_resize(self):
		# Copy image
		rgb_img_resized = self.rgb_img.copy()
		# Change scale so 720px become 400px
		rgb_img_resized = cv2.resize(rgb_img_resized, (self.resized_imgWidth, self.imgOutputSize), interpolation=cv2.INTER_AREA)
		# Cut off pixels at left and right to make image squared
		self.rgb_img_resized = rgb_img_resized[0:self.imgOutputSize, self.resized_img_horizontalStart:self.resized_img_horizontalStart+self.imgOutputSize]
		#print len(img), len(img[0])
		#output = self.rgb_img_resized
		cv2.imshow("Image-Stream", self.rgb_img_resized)
		cv2.waitKey(1)


	# Set the intrinsic camera-parameters according to parameters of rgb-image and resize-factor
	def set_resized_intrinsics(self):
		# Scale the paramters because of 720 to 400px shrinking
		intrinsics_resized = [i/self.img_scaleFac for i in self.intrinsics]
		# move cx to locate it at center again
		intrinsics_resized[2] = intrinsics_resized[2] - self.resized_img_horizontalStart
		self.intrinsics_resized = intrinsics_resized

	###########################################################
	# STORE DATA ##############################################
	###########################################################
	# Store images and poses at actual position to json-files
	def store_state(self):
		# Calculate pose-info for user output
		if self.lastPoseID == self.actPoseID:
			self.actStorage = self.actStorage + 1
		else:
			self.lastPoseID = self.actPoseID
			self.actStorage = 0
		poseInfo = str(self.actPoseID) + "_" + str(self.actStorage)
		fileName = '{0:06d}'.format(self.actStoreID)
		self.actStoreID = self.actStoreID + 1

		# Make sure to refresh data -> Set None and wait until new images arrive
		self.rgb_img = None
		self.rgb_img_resized = None
		counter = 0
		while self.rgb_img is None or self.rgb_img_resized is None:
			counter = counter + 1
			if counter > 200:
				print "No images arrived! Continuing with next pose..."
			rospy.sleep(0.05)

		# Store Images locally
		rgb_img = self.rgb_img.copy()
		rgb_img_resized = self.rgb_img_resized.copy()

		# Save OpenCV2 images
		cv2.imwrite(str(self.pathFullRes) + str(fileName) + "_rgb.png", rgb_img)
		cv2.imwrite(str(self.path) + str(fileName) + ".png", rgb_img_resized)

# Print debug messages
def print_debug(dStr):
	global debug
	if debug == True:
		print dStr

def main(args):
	# Init node
	rospy.init_node('data_capture', anonymous=True, disable_signals=True)

	# Check arguments and store path
	if len(args) < 2:
		print "Please specify folder to store files!"
		return
	else:
		path = rospy.get_param("path_to_store")
		pathFullRes = path + str(args[1]) + "_full_res" + "/"
		path = path + str(args[1]) + "/"
		if not os.path.exists(path):
			os.makedirs(path)
			os.makedirs(pathFullRes)
		else:
			rospy.logwarn("You are writing to an existing folder!")
	
	# Init dataCapture-Module and start capturing
	dc = dataCapture(path, pathFullRes)
	startID = 0
	actStoreID = 0
	if len(args) == 4:
		startID = int(args[2])
		actStoreID = int(args[3])
		print "Starting at pose no. " + str(startID) + " to record data no. " + str(actStoreID)

	while True:
		inp = raw_input("y to Store, e to Exit: ")[0]
		if inp == 'y':
			dc.store_state()
		elif inp == 'e':
			return

if __name__ == '__main__':
	main(sys.argv)
